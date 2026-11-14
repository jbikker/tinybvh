// tinybvh_custom.cpp: builds a custom single-header TinyBVH from tiny_bvh.h and its five
// platform headers (tiny_bvh_base.h, tiny_bvh_x86_float.h, tiny_bvh_x86_double.h,
// tiny_bvh_arm_float.h, tiny_bvh_arm_double.h).
//
// On Windows this is a small window with a checkbox per part; given arguments, it runs as a
// command-line tool instead (run it with --help). Elsewhere it is the command-line tool only.
//
// Parts that can be left out: each layout except BVH; VoxelSet; presplitting, the full-sweep
// and H-PLOC builders, the optimizer and BVH::MergeLeafs; threading (the built-in thread pool,
// and the job hooks in BVHContext); the x86 (SSE / AVX / AVX2) and ARM (NEON) code; and the
// templating on <Float, Index>, in which case the float / uint32_t code is emitted as plain,
// non-templated classes. Helpers, macros and comments that only served left-out code go with
// it. Asking a reduced build for a builder or feature it lacks is a fatal error. MBVH and
// BVH_Verbose are intermediate layouts: they are included whenever something needs them.
// With everything included, the result is the six headers merged, code unchanged.
//
// Build:
//   Visual Studio (Developer Command Prompt):  cl /std:c++17 /O2 /EHsc tinybvh_custom.cpp
//   MinGW-w64:  g++ -std=c++17 -O2 -municode -mwindows tinybvh_custom.cpp -o tinybvh_custom.exe -lcomdlg32 -lshell32 -lole32
//   Linux / macOS (command line only):  g++ -std=c++17 -O2 tinybvh_custom.cpp -o tinybvh_custom
//
// How it works: the headers are merged in place of their #include lines; conditionals on
// the SIMD macros of a left-out platform are resolved; the code of left-out layouts and
// features is removed declaration by declaration, using a scanner that understands C++
// scopes, comments and literals; and with the templates off, the template machinery is
// rewritten away. The result is checked before it is written: no left-out class may still
// be named in code. A few spots where kept code shares a function with a left-out layout
// are handled by the hand-written rules in kRewrites, which report when the sources no
// longer match them.

#ifdef _WIN32
#ifndef UNICODE
#define UNICODE
#endif
#ifndef _UNICODE
#define _UNICODE
#endif
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <commdlg.h>
#include <shellapi.h>
#include <shlobj.h>
#ifdef _MSC_VER
#pragma comment( lib, "user32.lib" )
#pragma comment( lib, "gdi32.lib" )
#pragma comment( lib, "comdlg32.lib" )
#pragma comment( lib, "shell32.lib" )
#pragma comment( lib, "ole32.lib" )
#pragma comment( linker, "/SUBSYSTEM:WINDOWS" )
#pragma comment( linker, "\"/manifestdependency:type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"" )
#pragma warning( disable: 4996 ) // fopen, wcscpy
#endif
#endif

#include <string>
#include <vector>
#include <set>
#include <map>
#include <tuple>
#include <algorithm>
#include <filesystem>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cctype>

namespace gen {

// ----------------------------------------------------------------------------
// Text utilities
// ----------------------------------------------------------------------------

static bool IsIdent( const char c ) { return isalnum( (unsigned char)c ) || c == '_'; }
static bool IsBlank( const char c ) { return c == ' ' || c == '\t' || c == '\r'; }

static std::string Trim( const std::string& s )
{
	size_t b = 0, e = s.size();
	while (b < e && isspace( (unsigned char)s[b] )) b++;
	while (e > b && isspace( (unsigned char)s[e - 1] )) e--;
	return s.substr( b, e - b );
}

static bool StartsWith( const std::string& s, const std::string& p ) { return s.compare( 0, p.size(), p ) == 0; }

// Whole-word search; 'w' must be an identifier.
static size_t FindWord( const std::string& s, const std::string& w, size_t from = 0 )
{
	for (size_t p = s.find( w, from ); p != std::string::npos; p = s.find( w, p + 1 ))
	{
		const bool l = p == 0 || !IsIdent( s[p - 1] ), r = p + w.size() >= s.size() || !IsIdent( s[p + w.size()] );
		if (l && r) return p;
	}
	return std::string::npos;
}
static bool HasWord( const std::string& s, const std::string& w ) { return FindWord( s, w ) != std::string::npos; }

// Replaces comments with a single space and string / character literal contents with nothing,
// so that searches for identifiers only see code.
static std::string CodeOnly( const std::string& s )
{
	std::string r;
	r.reserve( s.size() );
	for (size_t i = 0; i < s.size(); )
	{
		if (s.compare( i, 2, "//" ) == 0) { while (i < s.size() && s[i] != '\n') i++; r += ' '; continue; }
		if (s.compare( i, 2, "/*" ) == 0) { const size_t e = s.find( "*/", i + 2 ); i = e == std::string::npos ? s.size() : e + 2; r += ' '; continue; }
		if (s[i] == '"' || s[i] == '\'')
		{
			const char q = s[i++];
			while (i < s.size() && s[i] != q) { if (s[i] == '\\') i++; i++; }
			i++, r += q, r += q;
			continue;
		}
		r += s[i++];
	}
	return r;
}

static std::string Collapse( const std::string& s ) // whitespace runs become one space
{
	std::string r;
	bool sp = false;
	for (const char c : s) if (isspace( (unsigned char)c )) sp = true; else { if (sp && !r.empty()) r += ' '; sp = false, r += c; }
	return r;
}

static size_t LineStart( const std::string& t, size_t p ) { while (p > 0 && t[p - 1] != '\n') p--; return p; }
static size_t LineEnd( const std::string& t, size_t p ) { const size_t e = t.find( '\n', p ); return e == std::string::npos ? t.size() : e + 1; }

// ----------------------------------------------------------------------------
// Scanner: splits a scope (namespace level or class body) into items
// ----------------------------------------------------------------------------

enum ItemKind { IT_PP, IT_ENT, IT_NSOPEN, IT_NSCLOSE, IT_ACCESS };
struct Item
{
	ItemKind kind;
	size_t b, e;		// span in the text
	std::string head;	// code before the first '{' (or the whole item), comments removed, collapsed
	bool body = false;	// has a braced body
	size_t bodyB = 0, bodyE = 0; // the outermost '{' and its matching '}' (for class bodies)
};

struct ScanError { std::string what; };

// Skips a comment or literal starting at i; returns the position after it, or i if none starts there.
static size_t SkipNonCode( const std::string& t, size_t i, size_t e )
{
	if (t.compare( i, 2, "//" ) == 0) { while (i < e && t[i] != '\n') i++; return i; }
	if (t.compare( i, 2, "/*" ) == 0) { const size_t c = t.find( "*/", i + 2 ); return c == std::string::npos ? e : c + 2; }
	if (t[i] == '"' || t[i] == '\'')
	{
		const char q = t[i++];
		while (i < e && t[i] != q) { if (t[i] == '\\') i++; i++; }
		return i + 1;
	}
	return i;
}

static std::string HeadOf( const std::string& t, size_t b, size_t e )
{
	// the head runs to the first '{' outside comments and literals
	size_t i = b;
	int paren = 0;
	while (i < e)
	{
		const size_t j = SkipNonCode( t, i, e );
		if (j != i) { i = j; continue; }
		if (t[i] == '(') paren++; else if (t[i] == ')') paren--;
		else if (t[i] == '{' && paren == 0) break;
		i++;
	}
	return Collapse( CodeOnly( t.substr( b, i - b ) ) );
}

// Scan [b,e). classScope: the range is the inside of a class body.
static std::vector<Item> Scan( const std::string& t, size_t b, size_t e, bool classScope )
{
	std::vector<Item> items;
	std::vector<char> stack; // 'n' namespace, 'b' block
	size_t i = b, entStart = std::string::npos, firstBrace = std::string::npos;
	int paren = 0; // '{}' inside parentheses (braced default arguments) never ends an item
	bool lineStart = true;
	auto atScope = [&]() { for (const char c : stack) if (c != 'n') return false; return classScope ? stack.empty() : true; };
	auto emit = [&]( size_t end )
	{
		Item it;
		it.kind = IT_ENT, it.b = entStart, it.e = end, it.head = HeadOf( t, entStart, end );
		it.body = firstBrace != std::string::npos;
		if (it.body)
		{
			it.bodyB = firstBrace;
			// matching brace of the first one
			int depth = 0;
			for (size_t k = firstBrace; k < end; )
			{
				const size_t j = SkipNonCode( t, k, end );
				if (j != k) { k = j; continue; }
				if (t[k] == '{') depth++;
				else if (t[k] == '}' && --depth == 0) { it.bodyE = k; break; }
				k++;
			}
		}
		items.push_back( it );
		entStart = firstBrace = std::string::npos;
	};
	while (i < e)
	{
		const char c = t[i];
		if (lineStart)
		{
			size_t j = i;
			while (j < e && IsBlank( t[j] )) j++;
			if (j < e && t[j] == '#')
			{
				size_t k = j, eol;
				while (true)
				{
					eol = t.find( '\n', k );
					if (eol == std::string::npos || eol >= e) { eol = e; break; }
					size_t q = eol;
					while (q > k && t[q - 1] == '\r') q--;
					if (q > k && t[q - 1] == '\\') { k = eol + 1; continue; }
					break;
				}
				// a directive while a declaration is pending at this scope, with no '(' or '{' open,
				// ends it: that was a macro invocation without a ';' (WARNING( "..." ), for one)
				if (entStart != std::string::npos && atScope() && paren == 0 && firstBrace == std::string::npos)
				{
					size_t end = j;
					while (end > entStart && isspace( (unsigned char)t[end - 1] )) end--;
					emit( end );
				}
				if (entStart == std::string::npos && atScope())
				{
					Item it;
					it.kind = IT_PP, it.b = j, it.e = eol, it.head = Collapse( CodeOnly( t.substr( j, eol - j ) ) );
					items.push_back( it );
				}
				i = eol, lineStart = false;
				continue;
			}
		}
		if (c == '\n') { lineStart = true, i++; continue; }
		if (isspace( (unsigned char)c )) { i++; continue; }
		lineStart = false;
		{
			const size_t j = SkipNonCode( t, i, e );
			if (j != i) { i = j; continue; }
		}
		if (atScope() && entStart == std::string::npos)
		{
			if (!classScope)
			{
				if (t.compare( i, 9, "namespace" ) == 0 && !IsIdent( t[i + 9] ))
				{
					size_t k = i + 9;
					while (k < e && isspace( (unsigned char)t[k] )) k++;
					while (k < e && IsIdent( t[k] )) k++;
					while (k < e && isspace( (unsigned char)t[k] )) k++;
					if (k < e && t[k] == '{')
					{
						stack.push_back( 'n' );
						Item it;
						it.kind = IT_NSOPEN, it.b = i, it.e = k + 1, it.head = Collapse( t.substr( i, k + 1 - i ) );
						items.push_back( it );
						i = k + 1;
						continue;
					}
				}
				if (c == '}')
				{
					if (stack.empty()) throw ScanError{ "unbalanced '}' at namespace scope" };
					stack.pop_back();
					Item it;
					it.kind = IT_NSCLOSE, it.b = i, it.e = i + 1;
					items.push_back( it );
					i++;
					continue;
				}
			}
			else
			{
				static const char* acc[3] = { "public", "protected", "private" };
				bool isAccess = false;
				for (const char* a : acc)
				{
					const size_t n = strlen( a );
					if (t.compare( i, n, a ) == 0 && !IsIdent( t[i + n] ))
					{
						size_t k = i + n;
						while (k < e && IsBlank( t[k] )) k++;
						if (k < e && t[k] == ':' && t[k + 1] != ':')
						{
							Item it;
							it.kind = IT_ACCESS, it.b = i, it.e = k + 1, it.head = a;
							items.push_back( it );
							i = k + 1, isAccess = true;
						}
						break;
					}
				}
				if (isAccess) continue;
			}
			entStart = i;
		}
		if (c == '(') { paren++, i++; continue; }
		if (c == ')') { if (paren > 0) paren--; i++; continue; }
		if (c == '{')
		{
			if (entStart != std::string::npos && atScope() && firstBrace == std::string::npos && paren == 0) firstBrace = i;
			stack.push_back( 'b' ), i++;
			continue;
		}
		if (c == '}')
		{
			if (stack.empty()) throw ScanError{ "unbalanced '}'" };
			stack.pop_back(), i++;
			if (entStart != std::string::npos && atScope() && paren == 0)
			{
				// a block closed at this scope: the item ends, unless it was a braced member
				// initializer (followed by ',' or by the body's '{'). A ';' right after is absorbed.
				size_t j = i;
				while (j < e && isspace( (unsigned char)t[j] )) j++;
				if (j < e && (t[j] == ',' || t[j] == '{'))
				{
					if (t[j] == '{') firstBrace = std::string::npos; // the real body comes next
					continue;
				}
				j = i;
				while (j < e && IsBlank( t[j] )) j++;
				if (j < e && t[j] == ';') i = j + 1;
				emit( i );
			}
			continue;
		}
		if (c == ';' && entStart != std::string::npos && atScope() && paren == 0) { i++, emit( i ); continue; }
		i++;
	}
	if (entStart != std::string::npos) throw ScanError{ "unterminated declaration at end of scope, starting at line " + std::to_string( std::count( t.begin(), t.begin() + entStart, '\n' ) + 1 ) + ": " + Collapse( t.substr( entStart, 80 ) ) };
	if (!classScope && !stack.empty()) throw ScanError{ "unbalanced namespace braces" };
	return items;
}

// ----------------------------------------------------------------------------
// Classification of namespace-scope entities and class members
// ----------------------------------------------------------------------------

enum EntKind { EK_OTHER, EK_CLASS, EK_MEMBERDEF, EK_ALIAS, EK_INSTANTIATION, EK_FREEFUNC };
struct Entity
{
	Item item;
	EntKind kind = EK_OTHER;
	std::string owner;		// class the entity declares or belongs to (the innermost, for nested classes)
	std::string outer;		// outermost class of a qualified member definition ('BVH' in BVH::BVHNode::f)
	std::string name;		// member / function / alias name
	int arity = -1;			// parameter count for functions
	std::string sig;		// normalized parameter types, for telling overloads apart
	bool specialization = false;	// 'template <>'
	bool templated = false;		// TEMPLATED / TEMPLATED_M prefix
	bool isStatic = false;
	bool inImpl = false;		// inside an #ifdef TINYBVH_IMPLEMENTATION region
	std::string cond;		// enclosing preprocessor condition (guards and IMPLEMENTATION removed)
	std::string code;		// item text without comments and literals
};

static bool SkipTemplateArgs( const std::string& h, size_t& p ) // p at '<': moves past the matching '>'
{
	int depth = 0;
	for (; p < h.size(); p++)
	{
		if (h[p] == '<') depth++;
		else if (h[p] == '>' && --depth == 0) { p++; return true; }
		else if (h[p] == '(' || h[p] == ';' || h[p] == '{') return false;
	}
	return false;
}

// Parameter count of the parameter list starting at '(' at position p.
static int Arity( const std::string& h, size_t p )
{
	const size_t open = p;
	int depth = 0, commas = 0;
	bool any = false;
	for (; p < h.size(); p++)
	{
		const char c = h[p];
		if (c == '(' || c == '<' || c == '[' || c == '{') { if (depth++ == 0) continue; }
		else if (c == ')' || c == '>' || c == ']' || c == '}') { if (--depth == 0) break; }
		else if (c == ',' && depth == 1) commas++;
		if (depth >= 1 && !isspace( (unsigned char)c )) any = true;
	}
	if (!any) return 0;
	const std::string inner = Trim( h.substr( open + 1, p - open - 1 ) );
	if (inner.empty() || inner == "void") return 0;
	return commas + 1;
}

// A normalized parameter signature, for telling overloads apart: types only (names and default
// arguments dropped), no spaces, the aliases of the float / uint32_t instantiation resolved,
// and a trailing 'const' qualifier kept.
static std::string Signature( const std::string& h, size_t open )
{
	int depth = 0;
	size_t close = std::string::npos;
	for (size_t p = open; p < h.size(); p++) { if (h[p] == '(') depth++; else if (h[p] == ')' && --depth == 0) { close = p; break; } }
	if (close == std::string::npos) return "?";
	std::vector<std::string> params;
	std::string cur;
	depth = 0;
	for (size_t p = open + 1; p < close; p++)
	{
		const char c = h[p];
		if (c == '(' || c == '<' || c == '[' || c == '{') depth++;
		else if (c == ')' || c == '>' || c == ']' || c == '}') depth--;
		if (c == ',' && depth == 0) { params.push_back( cur ), cur.clear(); continue; }
		cur += c;
	}
	params.push_back( cur );
	std::string sig;
	for (std::string prm : params)
	{
		// default argument
		int d = 0;
		for (size_t p = 0; p < prm.size(); p++)
		{
			const char c = prm[p];
			if (c == '(' || c == '<' || c == '[' || c == '{') d++;
			else if (c == ')' || c == '>' || c == ']' || c == '}') d--;
			else if (c == '=' && d == 0) { prm = prm.substr( 0, p ); break; }
		}
		prm = Trim( prm );
		if (prm.empty() || prm == "void") continue;
		// the parameter name, unless what precedes it is only qualifiers ('const uint32_t')
		if (IsIdent( prm.back() ))
		{
			size_t a = prm.size();
			while (a > 0 && IsIdent( prm[a - 1] )) a--;
			const std::string before = Trim( prm.substr( 0, a ) );
			bool qualOnly = true;
			for (size_t p = 0; p < before.size(); )
			{
				while (p < before.size() && before[p] == ' ') p++;
				size_t q = p;
				while (q < before.size() && IsIdent( before[q] )) q++;
				const std::string w = before.substr( p, q - p );
				if (q == p) { qualOnly = false; break; }
				if (w != "const" && w != "volatile" && w != "unsigned" && w != "signed" && w != "struct" && w != "class") qualOnly = false;
				p = q;
			}
			if (!before.empty() && !qualOnly) prm = before;
		}
		std::string n;
		for (size_t p = 0; p <= prm.size(); )
		{
			if (p < prm.size() && IsIdent( prm[p] ))
			{
				size_t q = p;
				while (q < prm.size() && IsIdent( prm[q] )) q++;
				std::string w = prm.substr( p, q - p );
				if (w == "Float") w = "float"; else if (w == "Index") w = "uint32_t"; else if (w == "Vec3") w = "bvhvec3";
				else if (w == "Vertex") w = "bvhvec4"; else if (w == "Slice") w = "bvhvec4slice"; else if (w == "Mat4") w = "bvhmat4";
				else if (w == "BinBox") w = "BVHBinBox<float>";
				else if (w == "impl" && prm.compare( q, 2, "::" ) == 0) { p = q + 2; continue; } // 'impl::X' is 'X
				if (!n.empty() && IsIdent( n.back() )) n += ' ';
				n += w, p = q;
				continue;
			}
			if (p < prm.size() && prm[p] != ' ') n += prm[p];
			p++;
		}
		// '<Float, Index>' / '<float, uint32_t>' arguments say nothing once the class is fixed
		for (const char* drop : { "<float,uint32_t>", "<float,uint32_t >" })
			for (size_t q = n.find( drop ); q != std::string::npos; q = n.find( drop )) n.erase( q, strlen( drop ) );
		sig += (sig.empty() ? "" : ",") + n;
	}
	std::string tail = Trim( h.substr( close + 1 ) );
	if (StartsWith( tail, "const" )) sig += ")const"; else sig += ")";
	return "(" + sig;
}

// Position of the parameter list's '(' of a function declarator in a head, skipping macro calls
// such as ALIGNED( 16 ) and template argument lists; npos if the head declares no function.
static size_t FunctionParen( const std::string& h )
{
	static const char* skipMacros[] = { "ALIGNED", "TINYBVH_ALIGNED", "alignas", "__declspec", "__attribute__", "decltype", "sizeof", "noexcept" };
	// operator functions: the parameter list follows the operator symbol
	const size_t op = FindWord( h, "operator" );
	if (op != std::string::npos)
	{
		size_t p = op + 8;
		while (p < h.size() && h[p] == ' ') p++;
		if (h.compare( p, 2, "()" ) == 0) p += 2;
		while (p < h.size() && h[p] != '(') p++;
		return p < h.size() ? p : std::string::npos;
	}
	for (size_t p = 0; p < h.size(); p++)
	{
		if (h[p] == '=') return std::string::npos; // an initializer: not a function
		if (h[p] != '(') continue;
		size_t q = p;
		while (q > 0 && h[q - 1] == ' ') q--;
		size_t s = q;
		while (s > 0 && IsIdent( h[s - 1] )) s--;
		const std::string word = h.substr( s, q - s );
		bool macro = false;
		for (const char* m : skipMacros) if (word == m) macro = true;
		if (word == "operator" || (s >= 8 && h.compare( s - 8, 8, "operator" ) == 0)) return p;
		if (macro || word.empty())
		{
			// skip the macro's argument list
			int depth = 0;
			for (; p < h.size(); p++) { if (h[p] == '(') depth++; else if (h[p] == ')' && --depth == 0) break; }
			continue;
		}
		return p;
	}
	return std::string::npos;
}

// The identifier right before position p (skipping spaces); also accepts '~name' and 'operator..'.
static std::string NameBefore( const std::string& h, size_t p, size_t* startOut = 0 )
{
	size_t q = p;
	while (q > 0 && h[q - 1] == ' ') q--;
	size_t s = q;
	while (s > 0 && IsIdent( h[s - 1] )) s--;
	if (s > 0 && h[s - 1] == '~') s--;
	std::string n = h.substr( s, q - s );
	// operator symbols: back up to 'operator'
	const size_t op = h.rfind( "operator", q );
	if (op != std::string::npos && op < q && (n.empty() || !IsIdent( n[0] )))
	{
		n = Collapse( h.substr( op, q - op ) ), s = op;
	}
	if (startOut) *startOut = s;
	return n;
}

static void ClassifyEntity( Entity& en )
{
	const std::string& h = en.item.head;
	en.specialization = StartsWith( h, "template <>" ) || StartsWith( h, "template<>" );
	en.templated = StartsWith( h, "TEMPLATED " ) || StartsWith( h, "TEMPLATED_M " );
	en.isStatic = StartsWith( h, "static " ) || h.find( " static " ) < 40;
	// strip leading template / storage words for the patterns below
	std::string s = h;
	for (bool again = true; again; )
	{
		again = false;
		for (const char* w : { "TEMPLATED_M ", "TEMPLATED ", "static ", "inline ", "constexpr ", "TINYBVH_FORCEINLINE ", "PER_OCTANT ", "[[noreturn]] ", "extern " })
			if (StartsWith( s, w )) { s = s.substr( strlen( w ) ), again = true; }
		if (StartsWith( s, "template" ))
		{
			size_t p = s.find( '<' );
			if (p != std::string::npos && SkipTemplateArgs( s, p ))
			{
				const std::string rest = Trim( s.substr( p ) );
				if (StartsWith( s, "template <" ) || StartsWith( s, "template<" )) { s = rest, again = true; }
			}
		}
	}
	if (StartsWith( s, "using " ) || StartsWith( s, "typedef " ))
	{
		en.kind = EK_ALIAS;
		size_t p = s.find( ' ' ) + 1, q = p;
		while (q < s.size() && IsIdent( s[q] )) q++;
		en.name = s.substr( p, q - p );
		return;
	}
	if (StartsWith( h, "template class " ) || StartsWith( h, "template struct " ))
	{
		en.kind = EK_INSTANTIATION;
		const size_t p = h.find( "impl::" );
		size_t q = p == std::string::npos ? h.find( ' ', 15 ) + 1 : p + 6, r = q;
		while (r < h.size() && IsIdent( h[r] )) r++;
		en.owner = h.substr( q, r - q );
		return;
	}
	// class / struct / union / enum definitions and forward declarations
	if (StartsWith( s, "enum " )) { en.kind = EK_OTHER; return; } // enumerators are not members
	for (const char* kw : { "class ", "struct ", "union " })
	{
		if (!StartsWith( s, kw )) continue;
		size_t p = strlen( kw );
		std::string rest = s.substr( p );
		if (StartsWith( rest, "class " )) rest = rest.substr( 6 ); // enum class
		if (StartsWith( rest, "ALIGNED(" ) || StartsWith( rest, "ALIGNED (" ))
		{
			const size_t c = rest.find( ')' );
			rest = Trim( rest.substr( c + 1 ) );
		}
		if (StartsWith( rest, "impl::" )) rest = rest.substr( 6 );
		size_t q = 0;
		while (q < rest.size() && IsIdent( rest[q] )) q++;
		const std::string name = rest.substr( 0, q );
		// 'struct X f(...)' would be a function; tinybvh does not write those
		if (!name.empty() && FunctionParen( rest ) == std::string::npos)
		{
			en.kind = EK_CLASS, en.owner = en.name = name;
			return;
		}
	}
	const size_t paren = FunctionParen( s );
	if (paren != std::string::npos)
	{
		size_t nameStart = 0;
		const std::string name = NameBefore( s, paren, &nameStart );
		en.name = name, en.arity = Arity( s, paren ), en.sig = Signature( s, paren );
		// qualified member: 'X<...>::name', 'impl::X<...>::name', 'X<...>::Nested::name'
		std::vector<std::string> chain;
		size_t q = nameStart;
		while (true)
		{
			while (q > 0 && s[q - 1] == ' ') q--;
			if (!(q >= 2 && s[q - 1] == ':' && s[q - 2] == ':')) break;
			size_t r = q - 2;
			while (r > 0 && s[r - 1] == ' ') r--;
			if (r > 0 && s[r - 1] == '>')
			{
				int depth = 0;
				while (r > 0) { r--; if (s[r] == '>') depth++; else if (s[r] == '<' && --depth == 0) break; }
			}
			size_t a = r;
			while (a > 0 && s[a - 1] == ' ') a--;
			size_t c = a;
			while (c > 0 && IsIdent( s[c - 1] )) c--;
			const std::string cls = s.substr( c, a - c );
			if (cls.empty()) break;
			if (cls != "impl" && cls != "tinybvh" && cls != "std") chain.insert( chain.begin(), cls );
			q = c;
		}
		if (!chain.empty())
		{
			en.kind = EK_MEMBERDEF, en.owner = chain.back(), en.outer = chain.front();
			return;
		}
		en.kind = EK_FREEFUNC;
		return;
	}
	en.kind = EK_OTHER;
}

// Member of a class body: name and arity, or the friend / using target.
struct Member
{
	Item item;
	std::string name;
	int arity = -1;
	std::string sig;
	bool isFriend = false, isUsing = false, isTemplated = false;
	std::string code;
};

static Member ClassifyMember( const std::string& t, const Item& it )
{
	Member m;
	m.item = it;
	m.code = CodeOnly( t.substr( it.b, it.e - it.b ) );
	std::string s = it.head;
	m.isTemplated = StartsWith( s, "TEMPLATED " );
	m.isFriend = s.find( "friend " ) != std::string::npos;
	m.isUsing = StartsWith( s, "using " ) || StartsWith( s, "typedef " );
	for (bool again = true; again; )
	{
		again = false;
		for (const char* w : { "TEMPLATED ", "PER_OCTANT ", "static ", "inline ", "constexpr ", "virtual ", "explicit ", "TINYBVH_FORCEINLINE ", "template <> " })
			if (StartsWith( s, w )) { s = s.substr( strlen( w ) ), again = true; }
	}
	// a function pointer member, 'R (*name)( ... )': its name is the identifier after '(*'
	const size_t fp = s.find( "(*" );
	if (fp != std::string::npos && !m.isUsing && fp == s.find( '(' ))
	{
		size_t a = fp + 2;
		while (a < s.size() && s[a] == ' ') a++;
		size_t z = a;
		while (z < s.size() && IsIdent( s[z] )) z++;
		m.name = s.substr( a, z - a );
		return m;
	}
	const size_t paren = FunctionParen( s );
	if (paren != std::string::npos && !m.isUsing) m.name = NameBefore( s, paren ), m.arity = Arity( s, paren ), m.sig = Signature( s, paren );
	else if (!m.isUsing && !m.isFriend)
	{
		// a data member: the identifier before '=', '[', ':' or ';'
		size_t q = s.find_first_of( "=[;:" );
		if (q == std::string::npos) q = s.size();
		while (q > 0 && s[q - 1] == ' ') q--;
		size_t a = q;
		while (a > 0 && IsIdent( s[a - 1] )) a--;
		m.name = s.substr( a, q - a );
	}
	return m;
}

// ----------------------------------------------------------------------------
// Line helpers
// ----------------------------------------------------------------------------

static std::vector<std::string> SplitLines( const std::string& t )
{
	std::vector<std::string> lines;
	size_t p = 0;
	while (p < t.size())
	{
		const size_t e = t.find( '\n', p );
		if (e == std::string::npos) { lines.push_back( t.substr( p ) ); break; }
		lines.push_back( t.substr( p, e - p ) ), p = e + 1;
	}
	return lines;
}

static std::string JoinLines( const std::vector<std::string>& lines )
{
	std::string r;
	size_t n = 0;
	for (const auto& l : lines) n += l.size() + 1;
	r.reserve( n );
	for (const auto& l : lines) r += l, r += '\n';
	return r;
}

// The directive keyword of a line ("if", "ifdef", "endif", ...), or "" for non-directive lines.
static std::string Directive( const std::string& line, std::string* rest = 0 )
{
	size_t p = 0;
	while (p < line.size() && IsBlank( line[p] )) p++;
	if (p >= line.size() || line[p] != '#') return "";
	p++;
	while (p < line.size() && IsBlank( line[p] )) p++;
	size_t q = p;
	while (q < line.size() && IsIdent( line[q] )) q++;
	if (rest) *rest = Trim( CodeOnly( line.substr( q ) ) );
	return line.substr( p, q - p );
}
static bool IsIfLike( const std::string& d ) { return d == "if" || d == "ifdef" || d == "ifndef"; }

// ----------------------------------------------------------------------------
// Merging the six headers into one text
// ----------------------------------------------------------------------------

struct SourceSet
{
	std::map<std::string, std::string> files; // file name -> contents ('\n' line ends)
	bool crlf = false;
};

static const char* kHeaders[6] = { "tiny_bvh.h", "tiny_bvh_base.h", "tiny_bvh_x86_float.h", "tiny_bvh_x86_double.h", "tiny_bvh_arm_float.h", "tiny_bvh_arm_double.h" };

static std::string StripIncludeCheck( const std::string& t, const std::string& file )
{
	// "#ifndef TINY_BVH_H_ / #error "Include tiny_bvh.h ..." / #endif": meaningless once merged
	std::vector<std::string> lines = SplitLines( t );
	for (size_t i = 0; i + 2 < lines.size(); i++)
	{
		std::string r;
		if (Directive( lines[i], &r ) == "ifndef" && r == "TINY_BVH_H_" && Directive( lines[i + 1] ) == "error" && Directive( lines[i + 2] ) == "endif")
		{
			lines.erase( lines.begin() + i, lines.begin() + i + 3 );
			return JoinLines( lines );
		}
	}
	(void)file;
	return t;
}

static std::string Merge( const SourceSet& src, std::string& log )
{
	std::string t = src.files.at( "tiny_bvh.h" );
	for (int h = 1; h < 6; h++)
	{
		const std::string inc = std::string( "#include \"" ) + kHeaders[h] + "\"";
		const size_t p = t.find( inc );
		if (p == std::string::npos || t.find( inc, p + 1 ) != std::string::npos)
			throw ScanError{ std::string( "tiny_bvh.h: expected exactly one '" ) + inc + "'" };
		const size_t ls = LineStart( t, p ), le = LineEnd( t, p );
		std::string body = StripIncludeCheck( src.files.at( kHeaders[h] ), kHeaders[h] );
		std::string banner = std::string( "// ---- begin " ) + kHeaders[h] + " ----\n";
		std::string footer = std::string( "// ---- end " ) + kHeaders[h] + " ----\n";
		t = t.substr( 0, ls ) + banner + body + footer + t.substr( le );
	}
	log += "merged the six headers: " + std::to_string( SplitLines( t ).size() ) + " lines\n";
	return t;
}

// ----------------------------------------------------------------------------
// Preamble rewrites
// ----------------------------------------------------------------------------

// Index of the directive closing or continuing the group opened at line 'from' (at the same
// level), whose keyword is one of 'want'; -1 if none.
static int SameLevel( const std::vector<std::string>& lines, int from, std::initializer_list<const char*> want )
{
	int depth = 0;
	for (int i = from + 1; i < (int)lines.size(); i++)
	{
		const std::string d = Directive( lines[i] );
		if (IsIfLike( d )) { depth++; continue; }
		if (d == "endif") { if (depth == 0) { for (const char* w : want) if (d == w) return i; return -1; } depth--; continue; }
		if (depth == 0) for (const char* w : want) if (d == w) return i;
	}
	return -1;
}

// Removes the '#ifndef <optout> / #define <macro> / #endif' switch of a feature that is left out.
static void RemoveSwitch( std::vector<std::string>& lines, const char* optout, const char* macro )
{
	for (size_t i = 0; i + 2 < lines.size(); i++)
	{
		std::string r, r2;
		if (Directive( lines[i], &r ) == "ifndef" && r == optout && Directive( lines[i + 1], &r2 ) == "define" && r2 == macro && Directive( lines[i + 2] ) == "endif")
		{
			lines.erase( lines.begin() + i, lines.begin() + i + 3 );
			return;
		}
	}
	throw ScanError{ std::string( "tiny_bvh.h: the switch for " ) + macro + " was not found" };
}

static void RewritePreamble( std::string& t, bool x86, bool arm, bool templates, bool voxels, bool threading, std::string& log )
{
	std::vector<std::string> lines = SplitLines( t );
	// the SIMD detection: "#if defined __x86_64__ ..." / "#elif defined __aarch64__ ..." / "#else ..."
	int x = -1;
	for (int i = 0; i < (int)lines.size(); i++)
	{
		std::string r;
		if (Directive( lines[i], &r ) == "if" && r.find( "__x86_64__" ) != std::string::npos) { x = i; break; }
	}
	if (x < 0) throw ScanError{ "tiny_bvh.h: SIMD detection ('#if defined __x86_64__') not found" };
	const int a = SameLevel( lines, x, { "elif" } );
	if (a < 0 || lines[a].find( "__aarch64__" ) == std::string::npos) throw ScanError{ "tiny_bvh.h: ARM detection ('#elif defined __aarch64__') not found" };
	const int o = SameLevel( lines, a, { "else", "endif" } );
	if (o < 0) throw ScanError{ "tiny_bvh.h: end of the SIMD detection not found" };
	// replace from the back, so the indices stay valid
	if (!arm)
	{
		lines.erase( lines.begin() + a + 1, lines.begin() + o );
		lines.insert( lines.begin() + a + 1, "#define TINYBVH_NO_SIMD // ARM NEON code is not part of this build of TinyBVH." );
		log += "ARM NEON: excluded\n";
	}
	if (!x86)
	{
		lines.erase( lines.begin() + x + 1, lines.begin() + a );
		lines.insert( lines.begin() + x + 1, "#define TINYBVH_NO_SIMD // x86 SIMD code (SSE / AVX / AVX2) is not part of this build of TinyBVH." );
		log += "x86 SIMD: excluded\n";
	}
	if (!templates) RemoveSwitch( lines, "NO_DOUBLE_PRECISION_SUPPORT", "DOUBLE_PRECISION_SUPPORT" );
	if (!voxels) RemoveSwitch( lines, "NO_VOXEL_SUPPORT", "ENABLE_VOXEL_SUPPORT" ), log += "VoxelSet: excluded\n";
	if (!threading) RemoveSwitch( lines, "NO_THREADED_BUILDS", "ENABLE_THREADED_BUILDS" ), log += "threading: excluded\n";
	t = JoinLines( lines );
}

// ----------------------------------------------------------------------------
// Partial preprocessor: resolves #if / #ifdef / #elif / #else for macros known to be undefined,
// keeping every other conditional as it is (the approach of 'unifdef').
// ----------------------------------------------------------------------------

struct PPValue { bool known; long long v; };

class CondEval
{
public:
	CondEval( const std::string& s, const std::set<std::string>& undef ) : undef( undef ) { Tokenize( s ); }
	PPValue Eval() { p = 0; PPValue v = Or(); if (p != tok.size()) return { false, 0 }; return v; }
private:
	const std::set<std::string>& undef;
	std::vector<std::string> tok;
	size_t p = 0;
	void Tokenize( const std::string& s )
	{
		for (size_t i = 0; i < s.size(); )
		{
			const char c = s[i];
			if (isspace( (unsigned char)c )) { i++; continue; }
			if (IsIdent( c )) { size_t j = i; while (j < s.size() && IsIdent( s[j] )) j++; tok.push_back( s.substr( i, j - i ) ); i = j; continue; }
			static const char* two[] = { "&&", "||", "==", "!=", "<=", ">=", "<<", ">>" };
			bool done = false;
			for (const char* t2 : two) if (s.compare( i, 2, t2 ) == 0) { tok.push_back( t2 ), i += 2, done = true; break; }
			if (!done) tok.push_back( std::string( 1, c ) ), i++;
		}
	}
	bool Is( const char* s ) { return p < tok.size() && tok[p] == s; }
	PPValue Or()
	{
		PPValue a = And();
		while (Is( "||" ))
		{
			p++;
			PPValue b = And();
			if ((a.known && a.v) || (b.known && b.v)) a = { true, 1 };
			else if (a.known && b.known) a = { true, 0 };
			else a = { false, 0 };
		}
		return a;
	}
	PPValue And()
	{
		PPValue a = Cmp();
		while (Is( "&&" ))
		{
			p++;
			PPValue b = Cmp();
			if ((a.known && !a.v) || (b.known && !b.v)) a = { true, 0 };
			else if (a.known && b.known) a = { true, 1 };
			else a = { false, 0 };
		}
		return a;
	}
	PPValue Cmp()
	{
		PPValue a = Unary();
		while (Is( "==" ) || Is( "!=" ) || Is( "<" ) || Is( ">" ) || Is( "<=" ) || Is( ">=" ))
		{
			const std::string op = tok[p++];
			PPValue b = Unary();
			if (!a.known || !b.known) { a = { false, 0 }; continue; }
			long long r = 0;
			if (op == "==") r = a.v == b.v; else if (op == "!=") r = a.v != b.v; else if (op == "<") r = a.v < b.v;
			else if (op == ">") r = a.v > b.v; else if (op == "<=") r = a.v <= b.v; else r = a.v >= b.v;
			a = { true, r };
		}
		return a;
	}
	PPValue Unary()
	{
		if (Is( "!" )) { p++; PPValue v = Unary(); return v.known ? PPValue{ true, !v.v } : v; }
		if (Is( "(" )) { p++; PPValue v = Or(); if (Is( ")" )) p++; else return { false, 0 }; return v; }
		if (Is( "defined" ))
		{
			p++;
			bool par = Is( "(" );
			if (par) p++;
			if (p >= tok.size()) return { false, 0 };
			const std::string name = tok[p++];
			if (par) { if (Is( ")" )) p++; else return { false, 0 }; }
			if (undef.count( name )) return { true, 0 };
			return { false, 0 };
		}
		if (p < tok.size() && isdigit( (unsigned char)tok[p][0] ))
		{
			const std::string n = tok[p++];
			return { true, strtoll( n.c_str(), 0, 0 ) };
		}
		if (p < tok.size() && IsIdent( tok[p][0] ))
		{
			const std::string name = tok[p++];
			if (Is( "(" )) { int d = 0; do { if (Is( "(" )) d++; else if (Is( ")" )) d--; p++; } while (p < tok.size() && d > 0); return { false, 0 }; }
			if (undef.count( name )) return { true, 0 };
			return { false, 0 };
		}
		if (p < tok.size()) p++;
		return { false, 0 };
	}
};

static std::string Unifdef( const std::string& t, const std::set<std::string>& undef, std::string& log )
{
	if (undef.empty()) return t;
	struct Frame { bool parentActive, emitted, done, active; };
	std::vector<Frame> st;
	std::vector<std::string> in = SplitLines( t ), out;
	out.reserve( in.size() );
	bool inComment = false;
	size_t dropped = 0;
	auto active = [&]() { return st.empty() || (st.back().parentActive && st.back().active); };
	for (size_t i = 0; i < in.size(); i++)
	{
		const std::string& line = in[i];
		std::string rest;
		const std::string d = inComment ? std::string() : Directive( line, &rest );
		// track block comments, so that a '#' inside one is not taken for a directive
		{
			size_t p = 0;
			while (p < line.size())
			{
				if (inComment) { const size_t e = line.find( "*/", p ); if (e == std::string::npos) break; inComment = false, p = e + 2; continue; }
				const size_t a = line.find( "/*", p ), c = line.find( "//", p );
				if (a == std::string::npos || (c != std::string::npos && c < a)) break;
				inComment = true, p = a + 2;
			}
		}
		if (IsIfLike( d ))
		{
			const bool pa = active();
			if (!pa) { st.push_back( { false, false, true, false } ), dropped++; continue; }
			std::string expr = rest;
			if (d == "ifdef") expr = "defined " + rest;
			if (d == "ifndef") expr = "!defined " + rest;
			const PPValue v = CondEval( expr, undef ).Eval();
			if (v.known && v.v) st.push_back( { true, false, true, true } );
			else if (v.known) st.push_back( { true, false, false, false } );
			else { st.push_back( { true, true, false, true } ), out.push_back( line ); continue; }
			dropped++;
			continue;
		}
		if (d == "elif" || d == "else" || d == "endif")
		{
			if (st.empty()) throw ScanError{ "unbalanced #" + d + " at line " + std::to_string( i + 1 ) };
			Frame& f = st.back();
			if (d == "endif")
			{
				if (f.parentActive && f.emitted) out.push_back( line ); else dropped++;
				st.pop_back();
				continue;
			}
			if (!f.parentActive) { dropped++; continue; }
			if (f.done) { f.active = false, dropped++; continue; }
			if (d == "else")
			{
				if (f.emitted) out.push_back( line ); else dropped++;
				f.active = true, f.done = true;
				continue;
			}
			const PPValue v = CondEval( rest, undef ).Eval();
			if (v.known && v.v)
			{
				if (f.emitted) out.push_back( "#else // " + Trim( line ) );
				f.active = true, f.done = true;
			}
			else if (v.known) { f.active = false, dropped++; }
			else
			{
				if (f.emitted) out.push_back( line );
				else out.push_back( "#if " + rest ), f.emitted = true;
				f.active = true;
			}
			continue;
		}
		if (active()) out.push_back( line ); else dropped++;
	}
	if (!st.empty()) throw ScanError{ "unterminated #if" };
	std::string names;
	for (const auto& n : undef) names += (names.empty() ? "" : ", ") + n;
	log += "resolved conditionals for undefined " + names + ": " + std::to_string( dropped ) + " lines dropped\n";
	return JoinLines( out );
}

// ----------------------------------------------------------------------------
// Preprocessor condition stack, for entities at namespace scope
// ----------------------------------------------------------------------------

struct CondTracker
{
	struct Frame { std::vector<std::string> prev; std::string cur; bool guard = false; };
	std::vector<Frame> st;
	static bool IsGuardCond( const std::string& c )
	{
		// include guards and the implementation switch say nothing about the platform
		if (c.find( "TINYBVH_IMPLEMENTATION" ) != std::string::npos) return true;
		std::string id;
		for (size_t i = 0; i <= c.size(); i++)
		{
			if (i < c.size() && IsIdent( c[i] )) { id += c[i]; continue; }
			if (id.size() > 3 && (id.compare( id.size() - 2, 2, "H_" ) == 0 || id.find( "_H_IMPL" ) != std::string::npos) && StartsWith( id, "TINY_BVH" )) return true;
			id.clear();
		}
		return false;
	}
	void Feed( const std::string& head ) // a preprocessor line
	{
		std::string rest;
		const std::string d = Directive( head, &rest );
		if (IsIfLike( d ))
		{
			Frame f;
			f.cur = d == "ifdef" ? "defined " + rest : d == "ifndef" ? "!defined " + rest : rest;
			f.guard = IsGuardCond( f.cur );
			st.push_back( f );
		}
		else if (d == "elif" && !st.empty()) st.back().prev.push_back( st.back().cur ), st.back().cur = rest;
		else if (d == "else" && !st.empty()) st.back().prev.push_back( st.back().cur ), st.back().cur = "";
		else if (d == "endif" && !st.empty()) st.pop_back();
	}
	bool InImpl() const
	{
		for (const auto& f : st) if (f.prev.empty() && f.cur.find( "TINYBVH_IMPLEMENTATION" ) != std::string::npos && f.cur.find( '!' ) == std::string::npos) return true;
		return false;
	}
	std::string Condition() const // conjunction of the non-guard conditions; "" if none
	{
		std::string r;
		for (const auto& f : st)
		{
			if (f.guard) continue;
			std::string c;
			for (const auto& p : f.prev) c += (c.empty() ? "" : " && ") + std::string( "!(" ) + p + ")";
			if (!f.cur.empty()) c += (c.empty() ? "" : " && ") + std::string( "(" ) + f.cur + ")";
			if (!c.empty()) r += (r.empty() ? "" : " && ") + c;
		}
		return r;
	}
};

// ----------------------------------------------------------------------------
// Options
// ----------------------------------------------------------------------------

enum Toggle
{
	T_BVH_GPU, T_BVH4_GPU, T_BVH4_CPU, T_BVH8_CPU, T_BVH8_CWBVH, T_BVH4_AMD_HW, T_CWBVH8_AMD_HW, T_BVH_DOBB,
	T_MBVH, T_BVH_VERBOSE,			// intermediate layouts: forced on when something needs them
	T_VOXELSET,
	T_OPTIMIZER, T_MERGELEAFS, T_PRESPLITTING, T_FULLSWEEP, T_HPLOC, T_THREADING,
	T_X86, T_ARM, T_TEMPLATES,
	T_COUNT
};

struct ToggleInfo { const char* id; const char* label; const char* cls; };
static const ToggleInfo kToggles[T_COUNT] = {
	{ "bvh_gpu", "BVH_GPU (2-wide, GPU)", "BVH_GPU" },
	{ "bvh4_gpu", "BVH4_GPU (4-wide, GPU)", "BVH4_GPU" },
	{ "bvh4_cpu", "BVH4_CPU (4-wide, CPU SIMD)", "BVH4_CPU" },
	{ "bvh8_cpu", "BVH8_CPU (8-wide, CPU AVX2)", "BVH8_CPU" },
	{ "bvh8_cwbvh", "BVH8_CWBVH (compressed 8-wide, GPU)", "BVH8_CWBVH" },
	{ "bvh4_amd_hw", "BVH4_AMD_HW (AMD RT hardware)", "BVH4_AMD_HW" },
	{ "cwbvh8_amd_hw", "CWBVH8_AMD_HW (AMD RT hardware)", "CWBVH8_AMD_HW" },
	{ "bvh_dobb", "BVH_DOBB (8-wide, oriented boxes)", "BVH_DOBB" },
	{ "mbvh", "MBVH (intermediate)", "MBVH" },
	{ "bvh_verbose", "BVH_Verbose (intermediate)", "BVH_Verbose" },
	{ "voxelset", "VoxelSet (voxel grid BLAS)", "VoxelSet" },
	{ "optimizer", "Optimizer (Optimize, on every layout)", 0 },
	{ "mergeleafs", "BVH::MergeLeafs", 0 },
	{ "presplitting", "Presplitting (usePresplitting)", 0 },
	{ "fullsweep", "Full-sweep builder (useFullSweep)", 0 },
	{ "hploc", "H-PLOC builder (useHPLOC)", 0 },
	{ "threading", "Threading (thread pool, job hooks)", 0 },
	{ "x86", "x86 SIMD code (SSE, AVX, AVX2)", 0 },
	{ "arm", "ARM NEON code", 0 },
	{ "templates", "Templates (float and double precision)", 0 },
};

struct Options
{
	bool on[T_COUNT];
	// The intermediate layouts default to 'only when needed': Resolve turns them on when another
	// choice requires them, and they stay in otherwise only when asked for explicitly.
	Options() { for (bool& b : on) b = true; on[T_MBVH] = on[T_BVH_VERBOSE] = false; }
};

// Which intermediate layouts the other choices require: MBVH for the layouts converted through it,
// BVH_Verbose for the parts that work on it. Resolve, the Windows window and --describe read these.
static const Toggle kNeedsMBVH[] = { T_BVH4_GPU, T_BVH4_CPU, T_BVH8_CPU, T_BVH8_CWBVH, T_BVH4_AMD_HW, T_CWBVH8_AMD_HW, T_BVH_DOBB };
static const Toggle kNeedsVerbose[] = { T_OPTIMIZER, T_MERGELEAFS };
template <size_t N> static bool AnyOn( const Options& o, const Toggle( &list )[N] )
{
	for (const Toggle t : list) if (o.on[t]) return true;
	return false;
}
static bool RequiresMBVH( const Options& o ) { return AnyOn( o, kNeedsMBVH ); }
static bool RequiresVerbose( const Options& o ) { return AnyOn( o, kNeedsVerbose ); }

// Where each toggle goes, in the Windows window and on the web page: group (0: layouts, 1: builders,
// 2: code), column, row. The two intermediate layouts show as '<class>, when needed'.
struct Place { int t, group, col, row; };
static const Place kPlace[] = {
	{ T_BVH_GPU, 0, 0, 0 }, { T_BVH4_GPU, 0, 0, 1 }, { T_BVH8_CWBVH, 0, 0, 2 }, { T_BVH4_AMD_HW, 0, 0, 3 }, { T_CWBVH8_AMD_HW, 0, 0, 4 },
	{ T_BVH_DOBB, 0, 0, 5 },
	{ T_BVH4_CPU, 0, 1, 0 }, { T_BVH8_CPU, 0, 1, 1 }, { T_VOXELSET, 0, 1, 2 }, { T_MBVH, 0, 1, 3 }, { T_BVH_VERBOSE, 0, 1, 4 },
	{ T_PRESPLITTING, 1, 0, 0 }, { T_FULLSWEEP, 1, 0, 1 }, { T_HPLOC, 1, 0, 2 },
	{ T_OPTIMIZER, 1, 1, 0 }, { T_MERGELEAFS, 1, 1, 1 },
	{ T_THREADING, 2, 0, 0 }, { T_TEMPLATES, 2, 0, 1 },
	{ T_X86, 2, 1, 0 }, { T_ARM, 2, 1, 1 },
};
static const char* kGroupTitle[3] = { "Layouts (BVH is always included)", "Builders and optimization", "Code" };
static const int kRows[3] = { 6, 3, 2 };	// rows per group
static std::string DisplayLabel( const int t )
{
	return (t == T_MBVH || t == T_BVH_VERBOSE) ? std::string( kToggles[t].cls ) + ", when needed" : kToggles[t].label;
}

static Options Resolve( Options o )
{
	if (RequiresMBVH( o )) o.on[T_MBVH] = true;
	if (RequiresVerbose( o )) o.on[T_BVH_VERBOSE] = true;
	return o;
}

// ----------------------------------------------------------------------------
// Span deletion
// ----------------------------------------------------------------------------

struct Edit { size_t b, e; std::string with; };

static bool OnlyBlank( const std::string& t, size_t b, size_t e ) { for (size_t i = b; i < e; i++) if (!isspace( (unsigned char)t[i] )) return false; return true; }
static bool BlankOrComment( const std::string& t, size_t b, size_t e )
{
	size_t i = b;
	while (i < e && IsBlank( t[i] )) i++;
	return i >= e || t[i] == '\n' || t.compare( i, 2, "//" ) == 0;
}
static bool CommentLine( const std::string& t, size_t ls, size_t le )
{
	size_t i = ls;
	while (i < le && IsBlank( t[i] )) i++;
	return t.compare( i, 2, "//" ) == 0;
}
static bool BlankLine( const std::string& t, size_t ls, size_t le ) { return OnlyBlank( t, ls, le ); }

// Turns a deletion of [b,e) into a deletion of whole lines when nothing else shares them; a
// comment paragraph directly above goes too when the deleted code stood alone below it.
static Edit WidenDeletion( const std::string& t, size_t b, size_t e )
{
	const size_t ls = LineStart( t, b );
	const size_t le = LineEnd( t, e > b ? e - 1 : e );
	if (!OnlyBlank( t, ls, b ) || !BlankOrComment( t, e, le )) return { b, e, "" };
	size_t from = ls;
	// comment lines directly above, separated from what precedes them by a blank line
	size_t c = ls;
	while (c > 0)
	{
		const size_t ps = LineStart( t, c - 1 );
		if (!CommentLine( t, ps, c )) break;
		c = ps;
	}
	if (c < ls)
	{
		const bool blankAbove = c == 0 || BlankLine( t, LineStart( t, c - 1 ), c );
		const size_t ne = LineEnd( t, le );
		const bool blankBelow = le >= t.size() || BlankLine( t, le, ne ) || t.compare( le + (size_t)(std::find_if( t.begin() + le, t.end(), []( char ch ) { return !IsBlank( ch ); } ) - (t.begin() + le)), 1, "}" ) == 0;
		if (blankAbove && blankBelow) from = c;
	}
	return { from, le, "" };
}

static bool IsRuleLine( const std::string& l ) { const std::string x = Trim( l ); return StartsWith( x, "// ---" ) || StartsWith( x, "// ===" ); }
static bool IsCommentedDefine( const std::string& l ) { const std::string x = Trim( l ); return StartsWith( x, "// #define " ) || StartsWith( x, "//#define " ); }

// Start of the comment lines directly above line start 'ls': no blank line between, and not
// past a banner rule or a commented-out #define (which describe other things).
static size_t AttachedCommentStart( const std::string& t, size_t ls )
{
	size_t c = ls;
	while (c > 0)
	{
		const size_t ps = LineStart( t, c - 1 );
		if (!CommentLine( t, ps, c )) break;
		const std::string l = t.substr( ps, c - ps );
		if (IsRuleLine( l ) || IsCommentedDefine( l )) break;
		c = ps;
	}
	return c;
}

// Deletes whole lines [b,e) together with the comment lines directly above them.
static Edit DeleteWithComment( const std::string& t, size_t b, size_t e )
{
	const Edit w = WidenDeletion( t, b, e );
	if (w.b != LineStart( t, w.b )) return w; // the code shares its line: leave comments alone
	return { AttachedCommentStart( t, w.b ), w.e, "" };
}

static void ApplyEdits( std::string& t, std::vector<Edit> edits )
{
	// by position; at equal positions insertions first, in the order they were made
	std::stable_sort( edits.begin(), edits.end(), []( const Edit& a, const Edit& b )
	{
		if (a.b != b.b) return a.b < b.b;
		return a.e == a.b && b.e != b.b;
	} );
	std::string r;
	r.reserve( t.size() );
	size_t p = 0;
	for (const Edit& e : edits)
	{
		if (e.b < p)
		{
			// inside an earlier deletion: a longer deletion extends it, anything else is moot
			if (e.e > p && e.with.empty()) p = e.e;
			continue;
		}
		r.append( t, p, e.b - p ), r += e.with, p = e.e;
	}
	r.append( t, p, std::string::npos );
	t.swap( r );
}

// ----------------------------------------------------------------------------
// Removal of layouts and features
// ----------------------------------------------------------------------------

static bool RefsAny( const std::string& code, const std::set<std::string>& names, std::string* which = 0 )
{
	for (const auto& n : names) if (HasWord( code, n )) { if (which) *which = n; return true; }
	return false;
}

static int LineOf( const std::string& t, size_t p ) { return (int)std::count( t.begin(), t.begin() + p, '\n' ) + 1; }

// 'if (...) return ...;' on one line: a dispatch on the layout of a BLAS, removable on its own.
static bool IsDispatchLine( const std::string& code )
{
	const std::string c = Trim( code );
	return StartsWith( c, "if" ) && (c.size() > 2 && (c[2] == ' ' || c[2] == '(')) && HasWord( c, "return" ) && !c.empty() && c.back() == ';' &&
		std::count( c.begin(), c.end(), '{' ) == 0;
}

struct RemovalPlan
{
	std::set<std::string> classes;		// excluded classes and structs
	std::set<std::string> members;		// member names removed from every class (features)
	std::set<std::pair<std::string, std::string>> classMembers; // (class, member): removed from that class only
	std::set<std::string> macros;		// feature macros that announce a left-out part
	std::set<std::tuple<std::string, std::string, std::string>> killDecl, killDef; // (class, member, signature)
};

static void RemoveCode( std::string& t, RemovalPlan& plan, std::string& log )
{
	if (plan.classes.empty() && plan.members.empty() && plan.classMembers.empty() && plan.macros.empty()) return;
	int removedEnt = 0, removedMem = 0, removedLines = 0;
	for (int iter = 0; iter < 32; iter++)
	{
		std::vector<Item> items = Scan( t, 0, t.size(), false );
		std::vector<Edit> edits;
		for (const Item& it : items)
		{
			if (it.kind != IT_ENT) continue;
			Entity en;
			en.item = it;
			ClassifyEntity( en );
			const std::string text = t.substr( it.b, it.e - it.b );
			en.code = CodeOnly( text );
			std::string why;
			auto drop = [&]( const std::string& reason )
			{
				edits.push_back( DeleteWithComment( t, it.b, it.e ) ), removedEnt++;
				if (en.kind == EK_MEMBERDEF && en.owner == en.outer) plan.killDecl.insert( { en.owner, en.name, en.sig } );
				(void)reason;
			};
			// 1. code of an excluded class
			if ((en.kind == EK_CLASS || en.kind == EK_MEMBERDEF || en.kind == EK_INSTANTIATION) && (plan.classes.count( en.owner ) || plan.classes.count( en.outer ))) { drop( "class" ); continue; }
			if (en.kind == EK_ALIAS && (plan.classes.count( en.name ) || RefsAny( en.code, plan.classes ))) { drop( "alias" ); continue; }
			if ((en.kind == EK_FREEFUNC || en.kind == EK_OTHER) && !en.name.empty() && plan.classes.count( en.name )) { drop( "function" ); continue; }
			// 2. members removed by a feature, and definitions whose declaration went
			if (en.kind == EK_MEMBERDEF && (plan.members.count( en.name ) || plan.classMembers.count( { en.owner, en.name } ) || plan.killDef.count( { en.owner, en.name, en.sig } ))) { drop( "member" ); continue; }
			// 3. class bodies: members that name an excluded class, or that a feature removes
			if (en.kind == EK_CLASS && it.body)
			{
				std::vector<Item> mem = Scan( t, it.bodyB + 1, it.bodyE, true );
				for (const Item& mi : mem)
				{
					if (mi.kind != IT_ENT) continue;
					Member m = ClassifyMember( t, mi );
					const bool byName = !m.name.empty() && (plan.members.count( m.name ) || plan.classMembers.count( { en.owner, m.name } ) || plan.killDecl.count( { en.owner, m.name, m.sig } ));
					if (byName || RefsAny( m.code, plan.classes ))
					{
						edits.push_back( DeleteWithComment( t, mi.b, mi.e ) ), removedMem++;
						if (!m.name.empty() && !m.sig.empty()) plan.killDef.insert( { en.owner, m.name, m.sig } );
					}
				}
				continue;
			}
			// 4. anything else that names an excluded class: a dispatch line goes on its own,
			// otherwise the whole entity goes (and, for a member, its declaration)
			if (!RefsAny( en.code, plan.classes, &why )) continue;
			std::vector<Edit> lines;
			std::string rest;
			for (size_t p = it.b; p < it.e; )
			{
				const size_t le = std::min( LineEnd( t, p ), it.e );
				const std::string lc = CodeOnly( t.substr( p, le - p ) );
				if (IsDispatchLine( lc ) && RefsAny( lc, plan.classes )) lines.push_back( { LineStart( t, p ), LineEnd( t, p ), "" } );
				else rest += lc + "\n";
				p = le;
			}
			if (!lines.empty() && !RefsAny( rest, plan.classes ))
			{
				for (const Edit& l : lines) edits.push_back( l ), removedLines++;
				continue;
			}
			log += "  also removed " + (en.name.empty() ? "'" + en.item.head.substr( 0, 70 ) + "'" : (en.owner.empty() ? en.name : en.owner + "::" + en.name)) +
				" (line " + std::to_string( LineOf( t, it.b ) ) + "), which uses " + why + "\n";
			drop( "reference" );
		}
		if (edits.empty()) break;
		ApplyEdits( t, edits );
	}
	log += "removed " + std::to_string( removedEnt ) + " declarations and definitions, " + std::to_string( removedMem ) +
		" class members and " + std::to_string( removedLines ) + " dispatch lines\n";
}

// Feature macros of left-out parts ('#define TINYBVH_DOBB_BUILTIN'), with the comment above them.
static void RemoveMacros( std::string& t, const std::set<std::string>& macros, std::string& log )
{
	if (macros.empty()) return;
	std::vector<Edit> edits;
	for (const Item& it : Scan( t, 0, t.size(), false ))
	{
		std::string rest;
		if (it.kind != IT_PP || Directive( it.head, &rest ) != "define") continue;
		size_t q = 0;
		while (q < rest.size() && IsIdent( rest[q] )) q++;
		if (!macros.count( rest.substr( 0, q ) )) continue;
		const size_t ls = LineStart( t, it.b );
		edits.push_back( { AttachedCommentStart( t, ls ), LineEnd( t, it.e ), "" } );
	}
	if (!edits.empty()) log += "removed " + std::to_string( edits.size() ) + " feature macros of left-out parts\n";
	ApplyEdits( t, edits );
}

// Static functions nothing refers to any more (helpers of removed layouts).
static void RemoveUnusedStatics( std::string& t, std::string& log )
{
	int removed = 0;
	for (int iter = 0; iter < 16; iter++)
	{
		std::vector<Item> items = Scan( t, 0, t.size(), false );
		const std::string code = CodeOnly( t );
		std::vector<Edit> edits;
		for (const Item& it : items)
		{
			if (it.kind != IT_ENT) continue;
			Entity en;
			en.item = it;
			ClassifyEntity( en );
			// functions only: a static object can exist for its constructor or destructor alone
			// (tinybvh_poolAtExit shuts the thread pool down at exit), so it always stays
			if (!en.isStatic || en.kind != EK_FREEFUNC || !it.body) continue;
			std::string name = en.name;
			if (name.empty())
			{
				// a static variable: the identifier before '=' / '{' / ';' / '['
				const std::string& h = en.item.head;
				size_t q = h.find_first_of( "=[;{(" );
				if (q == std::string::npos) q = h.size();
				while (q > 0 && h[q - 1] == ' ') q--;
				size_t a = q;
				while (a > 0 && IsIdent( h[a - 1] )) a--;
				name = h.substr( a, q - a );
			}
			if (name.empty() || name == "operator") continue;
			// referenced anywhere but in its own text?
			const std::string own = CodeOnly( t.substr( it.b, it.e - it.b ) );
			size_t total = 0, inOwn = 0;
			for (size_t p = FindWord( code, name ); p != std::string::npos; p = FindWord( code, name, p + 1 )) total++;
			for (size_t p = FindWord( own, name ); p != std::string::npos; p = FindWord( own, name, p + 1 )) inOwn++;
			if (total > inOwn) continue;
			edits.push_back( WidenDeletion( t, it.b, it.e ) ), removed++;
		}
		if (edits.empty()) break;
		ApplyEdits( t, edits );
	}
	if (removed) log += "removed " + std::to_string( removed ) + " static helpers left without users\n";
}

// ----------------------------------------------------------------------------
// Orphans: helpers that only served code this configuration leaves out. Something is an
// orphan when the full source uses it outside its own definition and the result no longer
// does. Only internal code qualifies, so the public interface stays whole: functions and
// types of the implementation section, member functions that are not public, and macros.
// Data members stay, as they shape the objects.
// ----------------------------------------------------------------------------

static int CountWord( const std::string& code, const std::string& w )
{
	int n = 0;
	for (size_t p = FindWord( code, w ); p != std::string::npos; p = FindWord( code, w, p + 1 )) n++;
	return n;
}

static void RemoveOrphans( std::string& t, const std::string& full, std::string& log )
{
	const std::string fullCode = CodeOnly( full );
	std::vector<std::string> removed;
	for (int iter = 0; iter < 16; iter++)
	{
		const std::string code = CodeOnly( t );
		const std::vector<Item> items = Scan( t, 0, t.size(), false );
		struct Span { size_t b, e; bool comment; };
		struct Cand { std::vector<Span> spans, guards; bool eligible = true; };
		std::map<std::string, Cand> cand;
		std::map<std::pair<std::string, std::string>, bool> memberCand; // (class, name) -> non-public function
		std::map<std::string, std::vector<Span>> usingSpans;			// 'using Base::name;' lines
		CondTracker ct;
		// pass 1: class bodies, for the member functions that are not public
		for (const Item& it : items)
		{
			if (it.kind != IT_ENT) continue;
			Entity en;
			en.item = it;
			ClassifyEntity( en );
			if (en.kind != EK_CLASS || !it.body || en.specialization) continue;
			bool isPublic = StartsWith( it.head, "struct" ) || it.head.find( " struct " ) != std::string::npos;
			for (const Item& mi : Scan( t, it.bodyB + 1, it.bodyE, true ))
			{
				if (mi.kind == IT_ACCESS) { isPublic = mi.head == "public"; continue; }
				if (mi.kind != IT_ENT) continue;
				const Member m = ClassifyMember( t, mi );
				if (m.isUsing)
				{
					const size_t q = mi.head.rfind( "::" );
					if (q != std::string::npos && mi.head.find( "Base::" ) != std::string::npos)
					{
						std::string n = Trim( mi.head.substr( q + 2 ) );
						if (!n.empty() && n.back() == ';') n.pop_back();
						usingSpans[Trim( n )].push_back( { mi.b, mi.e, false } );
					}
					continue;
				}
				if (m.isFriend || m.arity < 0 || m.name.empty() || m.name == en.owner || m.name[0] == '~' || StartsWith( m.name, "operator" )) continue;
				if (isPublic) { cand[m.name].eligible = false; continue; }
				memberCand[{ en.owner, m.name }] = true;
				cand[m.name].spans.push_back( { mi.b, mi.e, false } );
			}
		}
		// pass 2: namespace-scope definitions and macros
		for (size_t k = 0; k < items.size(); k++)
		{
			const Item& it = items[k];
			if (it.kind == IT_PP)
			{
				ct.Feed( t.substr( it.b, it.e - it.b ) );
				std::string rest;
				const std::string d = Directive( it.head, &rest );
				if (d == "define" || d == "undef")
				{
					size_t q = 0;
					while (q < rest.size() && IsIdent( rest[q] )) q++;
					const std::string n = rest.substr( 0, q );
					if (n.empty()) continue;
					cand["#" + n].spans.push_back( { it.b, it.e, false } );
					// a default, '#ifndef X / #define X ... / #endif': the #ifndef is part of it, not a use
					std::string r0;
					if (d == "define" && k > 0 && k + 1 < items.size() && items[k - 1].kind == IT_PP && items[k + 1].kind == IT_PP &&
						Directive( items[k - 1].head, &r0 ) == "ifndef" && r0 == n && Directive( items[k + 1].head ) == "endif")
						cand["#" + n].guards.push_back( { items[k - 1].b, items[k - 1].e, false } );
				}
				continue;
			}
			if (it.kind != IT_ENT) continue;
			Entity en;
			en.item = it;
			ClassifyEntity( en );
			if (en.outer.empty()) en.outer = en.owner;
			if (en.kind == EK_MEMBERDEF)
			{
				if (memberCand.count( { en.owner, en.name } ) && en.owner == en.outer) cand[en.name].spans.push_back( { it.b, it.e, true } );
				continue;
			}
			std::string name = en.kind == EK_CLASS ? en.owner : en.name;
			// constant static data ('static const bvhvec3 proxyKDopAxes[13] = ...'): named before '[', '=' or '{'
			const bool constData = en.kind == EK_OTHER && (StartsWith( it.head, "static const " ) || StartsWith( it.head, "static constexpr " ));
			if (constData)
			{
				const std::string& h = it.head;
				size_t q = h.find_first_of( "[={;" );
				if (q == std::string::npos) q = h.size();
				while (q > 0 && h[q - 1] == ' ') q--;
				size_t a = q;
				while (a > 0 && IsIdent( h[a - 1] )) a--;
				name = h.substr( a, q - a );
			}
			if (name.empty() || name == "operator") continue;
			if ((en.kind != EK_FREEFUNC && en.kind != EK_CLASS && !constData) || en.specialization) { cand[name].eligible = false; continue; }
			// API: anything declared or defined outside the implementation section stays
			if (!ct.InImpl()) { cand[name].eligible = false; continue; }
			cand[name].spans.push_back( { it.b, it.e, true } );
		}
		// orphans: used by the full source beyond their own text, unused now
		std::vector<Edit> edits;
		for (auto& [key, c] : cand)
		{
			if (!c.eligible || c.spans.empty()) continue;
			const bool macro = key[0] == '#';
			const std::string name = macro ? key.substr( 1 ) : key;
			std::vector<Span> spans = c.spans;
			if (!macro && usingSpans.count( name )) for (const Span& u : usingSpans[name]) spans.push_back( u );
			int own = 0;
			for (const Span& sp : spans) own += CountWord( CodeOnly( t.substr( sp.b, sp.e - sp.b ) ), name );
			for (const Span& sp : c.guards) own += CountWord( CodeOnly( t.substr( sp.b, sp.e - sp.b ) ), name );
			if (CountWord( code, name ) > own || CountWord( fullCode, name ) <= own) continue;
			for (const Span& sp : spans)
			{
				if (macro)
				{
					// an '#ifndef X / #define X ... / #endif' default goes as a whole, with its description
					const size_t ls = LineStart( t, sp.b ), le = LineEnd( t, sp.e );
					size_t b = ls, e = le;
					if (ls > 0)
					{
						const size_t ps = LineStart( t, ls - 1 ), ne = LineEnd( t, le );
						std::string r1;
						if (Directive( t.substr( ps, ls - ps ), &r1 ) == "ifndef" && r1 == name && Directive( t.substr( le, ne - le ) ) == "endif") b = ps, e = ne;
					}
					edits.push_back( { AttachedCommentStart( t, b ), e, "" } );
				}
				else edits.push_back( sp.comment ? DeleteWithComment( t, sp.b, sp.e ) : WidenDeletion( t, sp.b, sp.e ) );
			}
			removed.push_back( name );
		}
		if (edits.empty()) break;
		ApplyEdits( t, edits );
	}
	// commented-out defines of switches that nothing tests any more
	{
		const std::string code = CodeOnly( t );
		std::vector<Edit> edits;
		for (size_t p = 0; p < t.size(); p = LineEnd( t, p ))
		{
			const size_t le = LineEnd( t, p );
			const std::string l = t.substr( p, le - p );
			if (!IsCommentedDefine( l )) continue;
			const std::string x = Trim( l );
			size_t q = x.find( "define" ) + 6;
			while (q < x.size() && x[q] == ' ') q++;
			size_t r = q;
			while (r < x.size() && IsIdent( x[r] )) r++;
			const std::string n = x.substr( q, r - q );
			if (n.empty() || CountWord( code, n ) > 0 || CountWord( fullCode, n ) == 0) continue;
			edits.push_back( { AttachedCommentStart( t, p ), le, "" } ), removed.push_back( n );
		}
		if (!edits.empty()) ApplyEdits( t, edits );
	}
	if (!removed.empty())
	{
		std::string list;
		for (size_t i = 0; i < removed.size() && i < 60; i++) list += (i ? ", " : "") + removed[i];
		if (removed.size() > 60) list += ", ...";
		log += "removed " + std::to_string( removed.size() ) + " helpers that only left-out code used: " + list + "\n";
	}
}

// Comments about what was left out: banner sections left without code, trailing comments
// and single comment lines that name a left-out class.
static void CleanComments( std::string& t, const std::set<std::string>& names, std::string& log )
{
	std::vector<std::string> lines = SplitLines( t );
	std::vector<bool> drop( lines.size(), false );
	auto isComment = [&]( size_t i ) { const std::string x = Trim( lines[i] ); return StartsWith( x, "//" ); };
	auto isBlank = [&]( size_t i ) { return Trim( lines[i] ).empty(); };
	auto namesLeftOut = [&]( const std::string& s ) { for (const auto& n : names) if (HasWord( s, n )) return true; return false; };
	int sections = 0, trailing = 0, single = 0;
	// banner sections: a title line followed by a rule; the section runs to the next banner title,
	// or to the end of the namespace or implementation block it is in
	auto isTitle = [&]( size_t i ) { return i + 1 < lines.size() && isComment( i ) && !IsRuleLine( lines[i] ) && IsRuleLine( lines[i + 1] ); };
	for (size_t i = 0; i < lines.size(); i++)
	{
		if (!isTitle( i )) continue;
		size_t j = i + 2;
		bool code = false;
		for (; j < lines.size(); j++)
		{
			if (isTitle( j )) break;
			const std::string x = Trim( lines[j] );
			if (StartsWith( x, "} // namespace" ) || StartsWith( x, "#endif // TINY" ) || StartsWith( x, "// ---- end " )) break;
			if (x.empty() || StartsWith( x, "//" )) continue;
			code = true;
			break;
		}
		if (code) continue;
		for (size_t k = i; k < j; k++) drop[k] = true;
		sections++;
	}
	for (size_t i = 0; i < lines.size(); i++)
	{
		if (drop[i]) continue;
		std::string& l = lines[i];
		const size_t c = l.find( "//" );
		if (c == std::string::npos) continue;
		const std::string before = Trim( l.substr( 0, c ) );
		const std::string comment = l.substr( c );
		if (!namesLeftOut( comment )) continue;
		if (!before.empty())
		{
			// a trailing comment, after code with no string literal (whose '//' could be text)
			if (before.find( '"' ) != std::string::npos) continue;
			size_t e = c;
			while (e > 0 && IsBlank( l[e - 1] )) e--;
			l.erase( e ), trailing++;
			continue;
		}
		// a comment line on its own: not part of a paragraph, and not a banner title
		const bool above = i > 0 && !drop[i - 1] && isComment( i - 1 ), below = i + 1 < lines.size() && isComment( i + 1 );
		if (above || below) continue;
		drop[i] = true, single++;
	}
	std::vector<std::string> out;
	for (size_t i = 0; i < lines.size(); i++) if (!drop[i]) out.push_back( lines[i] );
	t = JoinLines( out );
	(void)isBlank;
	if (sections + trailing + single)
		log += "removed comments on left-out code: " + std::to_string( sections ) + " empty sections, " + std::to_string( single ) +
		" comment lines and " + std::to_string( trailing ) + " trailing comments\n";
}

// Replaces one source line, found by a fragment, keeping its indentation.
static bool ReplaceLine( std::string& t, const std::string& fragment, const std::string& with )
{
	const size_t p = t.find( fragment );
	if (p == std::string::npos) return false;
	const size_t ls = LineStart( t, p ), le = LineEnd( t, p );
	size_t ind = ls;
	while (ind < le && IsBlank( t[ind] )) ind++;
	t = t.substr( 0, ls ) + t.substr( ls, ind - ls ) + with + "\n" + t.substr( le );
	return true;
}

// ----------------------------------------------------------------------------
// Hand-written rewrites, for the few places where a kept function shares code with an
// excluded layout in a way that no general rule can untangle. Lines are compared
// without their indentation. Each rule must match when its class is excluded; one
// that no longer does means the source changed, and the generator says so.
// ----------------------------------------------------------------------------

struct Rewrite { const char* when; const char* line; const char* with; bool optional; }; // with == 0: delete the line
static const Rewrite kRewrites[] = {
	// ray bundles: the packet tracers serve both a TLAS and a BVH4_CPU BLAS
	{ "BVH4_CPU", "const BVH4_CPU* blas4 = bvh->layout == LAYOUT_BVH4_CPU ? (const BVH4_CPU*)bvh : 0;", 0, false },
	{ "BVH4_CPU", "if (!tlas && !blas4) return tinybvh_packets_per_ray( bvh, b, pk, n );", "if (!tlas) return tinybvh_packets_per_ray( bvh, b, pk, n );", false },
	{ "BVH4_CPU", "if (!tlas && !blas4) return tinybvh_packets_occluded_per_ray( bvh, b, pk, n );", "if (!tlas) return tinybvh_packets_occluded_per_ray( bvh, b, pk, n );", false },
	{ "BVH4_CPU", "else OCTANT_DISPATCH_BUNDLE( tinybvh_bundle_bvh4, o, *blas4, b, bucket[o], bn[o] )", 0, false },
	{ "BVH4_CPU", "else OCTANT_DISPATCH_BUNDLE( tinybvh_occluded_bvh4, o, *blas4, b, bucket[o], bn[o] )", 0, false },
	// no threading: the job hooks leave BVHContext (see PlanFor), and the helpers run serially
	{ "-threading", "if (ctx.spawn) ctx.spawn( fn, payload, payloadSize, ctx.userdata ); else fn( (void*)payload );", "(void)ctx, (void)payloadSize, fn( (void*)payload );", false },
	{ "-threading", "inline void tinybvh_barrier( const BVHContext& ctx ) { if (ctx.barrier) ctx.barrier( ctx.userdata ); }", "inline void tinybvh_barrier( const BVHContext& ) {}", false },
	{ "-threading", "if (ctx.parallel_for) ctx.parallel_for( n, []( uint32_t i, void* p ) { (*(const F*)p)(i); },", 0, false },
	{ "-threading", "const_cast<void*>((const void*)&body), ctx.userdata );", 0, false },
	{ "-threading", "else for (uint32_t i = 0; i < n; i++) body( i );", "(void)ctx; for (uint32_t i = 0; i < n; i++) body( i );", false },
	// BVH::Load clears the full-sweep builder's buffers along with the others
	{ "-fullsweep", "scratchPad = 0, flag = 0, tmpBnds = 0, SARs = 0;", "scratchPad = 0;", false },
	{ "-fullsweep", "for (int32_t a = 0; a < 3; a++) sortedIdx[a] = 0, sortedBnds[a] = 0;", 0, false },
	// builders left out: asking for one is a fatal error, as for the optimizer
	{ "-fullsweep", "if (settings.useFullSweep) PrepareBuild( vertices, indices, prims ), BuildFullSweep();",
		"if (settings.useFullSweep) { BVH_FATAL_ERROR( \"BVH::Build( .. ), settings.useFullSweep: the full-sweep builder is not part of this build of TinyBVH.\" ); }", false },
	// Bonsai's top tree builders: one line each, which goes with its builder. Asking for a builder that
	// is left out is a fatal error, as for the others; BONSAI_TOP_CHEAPEST tries the ones there are.
	{ "-fullsweep", "if (settings.bonsaiTopTree == BONSAI_TOP_FULLSWEEP) top.BuildFullSweep( 0, 0 ), built = true;",
		"BVH_FATAL_ERROR_IF( settings.bonsaiTopTree == BONSAI_TOP_FULLSWEEP, \"BVH::Build( .. ), settings.bonsaiTopTree: the full-sweep builder is not part of this build of TinyBVH.\" );", false },
	{ "-fullsweep", "{ BVH t( context ); prepareTop( t ); t.BuildFullSweep( 0, 0 ); if (t.InteriorArea() < top.InteriorArea()) top = tinybvh_move( t ); }", 0, false },
	{ "-hploc", "if (settings.bonsaiTopTree == BONSAI_TOP_HPLOC) top.BuildHPLOC(), built = true;",
		"BVH_FATAL_ERROR_IF( settings.bonsaiTopTree == BONSAI_TOP_HPLOC, \"BVH::Build( .. ), settings.bonsaiTopTree: the H-PLOC builder is not part of this build of TinyBVH.\" );", false },
	{ "-hploc", "{ BVH t( context ); prepareTop( t ); t.BuildHPLOC(); if (t.InteriorArea() < top.InteriorArea()) top = tinybvh_move( t ); }", 0, false },
	{ "-hploc", "else if (settings.useHPLOC) PrepareBuild( vertices, indices, prims ), BuildHPLOC();",
		"else if (settings.useHPLOC) { BVH_FATAL_ERROR( \"BVH::Build( .. ), settings.useHPLOC: the H-PLOC builder is not part of this build of TinyBVH.\" ); }", false },
	{ "-presplitting", "// special builders: full-sweep, H-PLOC.",
		"BVH_FATAL_ERROR_IF( settings.usePresplitting, \"BVH::Build( .. ), settings.usePresplitting: presplitting is not part of this build of TinyBVH.\" );\n\t// special builders: full-sweep, H-PLOC.", false },
	// ... and the comments of the job helpers
	{ "-threading", "// The functions below serialize when no thread pool is available.", "// Without threading, the job helpers below run their work on the calling thread.", true },
	{ "-threading", "// tinybvh_parallel_for: run body( i ) for i in [0,n), in parallel when the context supplies a pool.", "// tinybvh_parallel_for: run body( i ) for i in [0,n).", true },
	{ "-threading", "// Parallel build hooks: 'spawn' runs fn(payload-copy) asynchronously", 0, true },
	{ "-threading", "// Wait for spawned tasks", 0, true },
	{ "-threading", "// Runs fn(i, payload) for i in [0,n) and block for the results", 0, true },
	{ "-threading", "#define TINYBVH_DEFAULT_SPAWN nullptr", 0, true },
	{ "-threading", "#define TINYBVH_DEFAULT_BARRIER nullptr", 0, true },
	{ "-threading", "#define TINYBVH_DEFAULT_PARALLEL_FOR nullptr", 0, true },
	{ "-threading", "#undef TINYBVH_DEFAULT_SPAWN", 0, true },
	{ "-threading", "#undef TINYBVH_DEFAULT_BARRIER", 0, true },
	{ "-threading", "#undef TINYBVH_DEFAULT_PARALLEL_FOR", 0, true },
	{ "-threading", "// Built-in std::thread pool backing the default BVHContext hooks. Define", 0, true },
	{ "-threading", "// TINYBVH_NO_BUILTIN_POOL to leave this out, in which case builds are serial", 0, true },
	{ "-threading", "// unless you hook your own.", 0, true },
	{ "-threading", "// Threading helpers. ParallelFor runs body( i ) for i in [0, n), on the pool only if 'threads';", "// ParallelFor runs body( i ) for i in [0, n), on the calling thread.", true },
	{ "-threading", "// enableThreading = false must keep a build on the calling thread.", 0, true },
	{ "-threading", "// Whether loops over the pool are allowed at all, and whether a build over 'prims' primitives uses it.", "// Without threading, neither loops nor builds use a pool.", true },
	{ "-threading", "// Spawn fn(payload) asynchronously", "// Run fn(payload) now: without threading there is nothing to run it on.", true },
	{ "-threading", "// Wait for spawned tasks to finish", "// Nothing to wait for: each task ran when it was spawned.", true },
	{ "-presplitting", "// presplitting", 0, true },
	// the SIMD detection's notes, for the platforms that are left
	{ "-x86&-arm", "// SSE/AVX/AVX2/NEON support.", 0, true },
	{ "-x86", "// SSE/AVX/AVX2/NEON support.", "// NEON support.", true },
	{ "-arm", "// SSE/AVX/AVX2/NEON support.", "// SSE/AVX/AVX2 support.", true },
	{ "-x86", "// SSE4.2 availability: Since Nehalem (2008)", 0, true },
	{ "-x86", "// AVX1 availability: Since Sandy Bridge (2011)", 0, true },
	{ "-x86", "// AVX2 availability: Since Haswell (2013)", 0, true },
	// a reduced build: the six-file layout described at the top no longer applies
	{ "+reduced", "// The library consists of this file plus the headers it includes:", "// This is a single-header build of TinyBVH; the configuration it holds is listed at the top.", true },
	{ "+reduced", "//   tiny_bvh_base.h        platform-neutral code: types, builders and traversal", 0, true },
	{ "+reduced", "//   tiny_bvh_x86_float.h   SSE / AVX / AVX2 kernels for the single precision layouts", 0, true },
	{ "+reduced", "//   tiny_bvh_x86_double.h  x86 kernels for the double precision layouts", 0, true },
	{ "+reduced", "//   tiny_bvh_arm_float.h   NEON kernels for the single precision layouts", 0, true },
	{ "+reduced", "//   tiny_bvh_arm_double.h  ARM kernels for the double precision layouts", 0, true },
	{ "+reduced", "// Keep them together; only tiny_bvh.h is included by application code.", 0, true },
};

// Phrases replaced inside lines, for comments that mostly still apply.
struct Phrase { const char* when; const char* phrase; const char* with; };
static const Phrase kPhrases[] = {
	{ "-threading", " via the thread pool", "" },
	{ "-threading", ", optionally sliced over the thread pool", "" },
	{ "-threading", ", scheduled via the threading hooks;", ";" },
};

// 'active': the excluded classes, plus "-<id>" for every toggle that is off. A rule that is not
// optional must match: one that no longer does means the source changed.
static void ApplyRewrites( std::string& t, const std::set<std::string>& active, std::string& log )
{
	std::vector<std::string> lines = SplitLines( t );
	for (const Rewrite& r : kRewrites)
	{
		if (!active.count( r.when )) continue;
		int hits = 0;
		for (size_t i = 0; i < lines.size(); i++)
		{
			if (Trim( lines[i] ) != r.line) continue;
			if (!r.with) { lines.erase( lines.begin() + i-- ), hits++; continue; }
			size_t ind = 0;
			while (ind < lines[i].size() && IsBlank( lines[i][ind] )) ind++;
			lines[i] = lines[i].substr( 0, ind ) + r.with, hits++;
		}
		if (!hits && !r.optional)
			log += std::string( "  warning: rewrite for " ) + (r.when[0] == '-' ? std::string( "leaving out " ) + (r.when + 1) : std::string( r.when )) + " no longer matches: '" + r.line + "'\n";
	}
	for (const Phrase& ph : kPhrases)
	{
		if (!active.count( ph.when )) continue;
		for (std::string& l : lines)
			for (size_t q = l.find( ph.phrase ); q != std::string::npos; q = l.find( ph.phrase, q + strlen( ph.with ) ))
				l.replace( q, strlen( ph.phrase ), ph.with );
	}
	t = JoinLines( lines );
}

// ----------------------------------------------------------------------------
// A left-out feature's condition is false. Its reads become 'false', and then
// 'if (false) S' goes (an 'else' branch stays), 'false ? a : b' becomes 'b', and
// '!false' becomes 'true'.
// ----------------------------------------------------------------------------

// Next occurrence of 'pat' in code, from 'from'; comments and literals are skipped.
static size_t FindInCode( const std::string& t, const std::string& pat, size_t from )
{
	for (size_t i = from; i < t.size(); )
	{
		const size_t j = SkipNonCode( t, i, t.size() );
		if (j != i) { i = j; continue; }
		if (t.compare( i, pat.size(), pat ) == 0) return i;
		i++;
	}
	return std::string::npos;
}

static size_t SkipSpaceAndComments( const std::string& t, size_t p )
{
	while (p < t.size())
	{
		if (isspace( (unsigned char)t[p] )) { p++; continue; }
		if (t.compare( p, 2, "//" ) == 0 || t.compare( p, 2, "/*" ) == 0) { p = SkipNonCode( t, p, t.size() ); continue; }
		break;
	}
	return p;
}

// End of the statement at p: a braced block, or up to its ';' at depth 0. npos if unclear.
static size_t StatementEnd( const std::string& t, size_t p )
{
	p = SkipSpaceAndComments( t, p );
	if (p >= t.size()) return std::string::npos;
	if (t.compare( p, 2, "if" ) == 0 && !IsIdent( t[p + 2] )) return std::string::npos; // a nested if: an 'else' would be ambiguous
	int depth = 0;
	const bool block = t[p] == '{';
	for (size_t k = p; k < t.size(); )
	{
		const size_t j = SkipNonCode( t, k, t.size() );
		if (j != k) { k = j; continue; }
		const char c = t[k];
		if (c == '(' || c == '[' || c == '{') depth++;
		else if (c == ')' || c == ']' || c == '}')
		{
			if (depth == 0) return std::string::npos;
			if (--depth == 0 && block && c == '}') return k + 1;
		}
		else if (c == ';' && depth == 0 && !block) return k + 1;
		k++;
	}
	return std::string::npos;
}

// The previous non-space character before p, or 0.
static char PrevCode( const std::string& t, size_t p ) { while (p > 0 && isspace( (unsigned char)t[p - 1] )) p--; return p > 0 ? t[p - 1] : 0; }

static void FoldFalse( std::string& t, const std::vector<std::string>& exprs, std::string& log )
{
	int reads = 0, ifs = 0, ternaries = 0, unclear = 0;
	// reads of the condition, not member accesses ('top.settings.x') nor declarations ('bool F()'),
	// nor the condition of a fatal error check: that asks whether the feature was requested, and
	// must keep refusing it ('BVH_FATAL_ERROR_IF( settings.usePresplitting, ... )')
	auto inFatalCheck = [&]( size_t p )
	{
		const size_t c = t.rfind( "BVH_FATAL_ERROR_IF", p );
		if (c == std::string::npos || t.find( ';', c ) < p) return false;
		int depth = 0;
		for (size_t k = t.find( '(', c ); k < p; k++) { if (t[k] == '(') depth++; else if (t[k] == ')') depth--; else if (t[k] == ',' && depth == 1) return false; }
		return depth > 0; // still inside the first argument
	};
	for (const std::string& e : exprs)
	{
		for (size_t p = FindInCode( t, e, 0 ); p != std::string::npos; p = FindInCode( t, e, p + 1 ))
		{
			const char before = PrevCode( t, p );
			if (IsIdent( before ) || before == '.' || before == '>' || before == ':') continue;
			if (inFatalCheck( p )) continue;
			if (p + e.size() < t.size() && IsIdent( t[p + e.size()] ) && IsIdent( e.back() )) continue;
			size_t q = p + e.size();
			while (q < t.size() && IsBlank( t[q] )) q++;
			if (q < t.size() && t[q] == '=' && t[q + 1] != '=') continue; // an assignment to the setting
			t.replace( p, e.size(), "false" ), reads++;
		}
	}
	for (bool changed = true; changed; )
	{
		changed = false;
		for (const char* n : { "!false", "!true" })
			for (size_t p = FindInCode( t, n, 0 ); p != std::string::npos; p = FindInCode( t, n, p + 1 ))
				if (!IsIdent( t[p + strlen( n )] )) t.replace( p, strlen( n ), n[1] == 'f' ? "true" : "false" ), changed = true;
		// 'false ? a : b' and 'true ? a : b', where the literal is the whole condition
		for (const char* lit : { "false", "true" })
		{
			for (size_t p = FindInCode( t, lit, 0 ); p != std::string::npos; p = FindInCode( t, lit, p + 1 ))
			{
				const size_t L = strlen( lit );
				if ((p > 0 && IsIdent( t[p - 1] )) || IsIdent( t[p + L] )) continue;
				size_t q = p + L;
				while (q < t.size() && IsBlank( t[q] )) q++;
				if (t[q] != '?') continue;
				const char before = PrevCode( t, p );
				if (!strchr( "=(,?:{;", before ) && !(p >= 7 && t.compare( LineStart( t, p ), 0, "" ) == 0 && Trim( t.substr( LineStart( t, p ), p - LineStart( t, p ) ) ) == "return")) continue;
				// a, up to the matching ':'; b, up to the end of the conditional expression
				size_t a0 = q + 1, k = a0, colon = std::string::npos, end = std::string::npos;
				int depth = 0, nested = 0;
				for (; k < t.size(); k++)
				{
					const char c = t[k];
					if (c == '(' || c == '[' || c == '{') depth++;
					else if (c == ')' || c == ']' || c == '}') { if (depth == 0) break; depth--; }
					else if (c == '?' && depth == 0) nested++;
					else if (c == ':' && depth == 0)
					{
						if (t[k + 1] == ':' || (k > 0 && t[k - 1] == ':')) { k += t[k + 1] == ':' ? 1 : 0; continue; }
						if (nested) { nested--; continue; }
						colon = k++;
						break;
					}
					else if (c == ';' && depth == 0) break;
				}
				if (colon == std::string::npos) { unclear++; continue; }
				for (depth = 0; k < t.size(); k++)
				{
					const char c = t[k];
					if (c == '(' || c == '[' || c == '{') depth++;
					else if (c == ')' || c == ']' || c == '}') { if (depth == 0) { end = k; break; } depth--; }
					else if ((c == ';' || c == ',') && depth == 0) { end = k; break; }
				}
				if (end == std::string::npos) { unclear++; continue; }
				const std::string keep = Trim( lit[0] == 'f' ? t.substr( colon + 1, end - colon - 1 ) : t.substr( a0, colon - a0 ) );
				t.replace( p, end - p, keep ), ternaries++, changed = true;
			}
		}
		// 'if (false) S [else S2]' and 'if (true) S [else S2]'
		for (const char* lit : { "if (false)", "if (true)" })
		{
			for (size_t p = FindInCode( t, lit, 0 ); p != std::string::npos; p = FindInCode( t, lit, p + 1 ))
			{
				if (p > 0 && IsIdent( t[p - 1] )) continue;
				const size_t s0 = p + strlen( lit ), s1 = StatementEnd( t, s0 );
				if (s1 == std::string::npos) { unclear++; continue; }
				size_t e = SkipSpaceAndComments( t, s1 );
				const bool hasElse = t.compare( e, 4, "else" ) == 0 && !IsIdent( t[e + 4] );
				if (lit[4] == 'f')
				{
					if (hasElse) t.erase( p, e + 4 - p ); // the else branch stays, as a plain statement
					else
					{
						const Edit d = WidenDeletion( t, p, s1 );
						t.erase( d.b, d.e - d.b );
					}
				}
				else
				{
					if (hasElse)
					{
						const size_t s2 = StatementEnd( t, e + 4 );
						if (s2 == std::string::npos) { unclear++; continue; }
						t.erase( s1, s2 - s1 );
					}
					t.erase( p, s0 - p );
				}
				ifs++, changed = true;
			}
		}
	}
	log += "folded " + std::to_string( reads ) + " reads of a left-out feature's condition: " + std::to_string( ifs ) + " if statements and " +
		std::to_string( ternaries ) + " conditional expressions resolved\n";
	if (unclear) log += "  warning: " + std::to_string( unclear ) + " uses of a left-out feature's condition were too complex to resolve\n";
}

// Empty #if/#endif pairs and long runs of blank lines, as left behind by the removals.
static void Tidy( std::string& t )
{
	for (bool changed = true; changed; )
	{
		changed = false;
		std::vector<std::string> lines = SplitLines( t ), out;
		out.reserve( lines.size() );
		for (size_t i = 0; i < lines.size(); i++)
		{
			const std::string d = Directive( lines[i] );
			if (IsIfLike( d ) || d == "else" || d == "elif")
			{
				size_t j = i + 1;
				while (j < lines.size() && BlankLine( lines[j], 0, lines[j].size() )) j++;
				if (j < lines.size() && Directive( lines[j] ) == "endif")
				{
					if (IsIfLike( d )) { i = j, changed = true; continue; }			// #if ... #endif with nothing inside
					if (d == "else") { i = j - 1, changed = true; continue; }	// an empty #else branch: keep the #endif
				}
			}
			out.push_back( lines[i] );
		}
		// at most one blank line in a row
		std::vector<std::string> squeezed;
		for (const auto& l : out)
		{
			const bool blank = BlankLine( l, 0, l.size() );
			if (blank && !squeezed.empty() && BlankLine( squeezed.back(), 0, squeezed.back().size() )) { changed = true; continue; }
			squeezed.push_back( l );
		}
		t = JoinLines( squeezed );
	}
}

// ----------------------------------------------------------------------------
// Templates off: the classes templated on <Float, Index> become plain classes for
// float / uint32_t, their explicit specializations become ordinary definitions,
// and the generic definitions they replace are compiled only where no
// specialization applies.
// ----------------------------------------------------------------------------

// Applies replacements to code only, leaving comments and literals alone.
static std::string ReplaceInCode( const std::string& t, const std::vector<std::pair<std::string, std::string>>& rules )
{
	std::string r, seg;
	r.reserve( t.size() );
	auto flush = [&]()
	{
		for (const auto& rule : rules)
		{
			if (seg.find( rule.first ) == std::string::npos) continue;
			std::string o;
			size_t p = 0;
			for (size_t q = seg.find( rule.first ); q != std::string::npos; q = seg.find( rule.first, p ))
			{
				// identifier rules match whole words only
				const bool word = IsIdent( rule.first[0] ) && IsIdent( rule.first.back() );
				if (word && ((q > 0 && IsIdent( seg[q - 1] )) || (q + rule.first.size() < seg.size() && IsIdent( seg[q + rule.first.size()] ))))
				{
					o.append( seg, p, q + 1 - p ), p = q + 1;
					continue;
				}
				// prefix rules ('TEMPLATED ') must start a word
				if (!word && IsIdent( rule.first[0] ) && q > 0 && IsIdent( seg[q - 1] )) { o.append( seg, p, q + 1 - p ), p = q + 1; continue; }
				o.append( seg, p, q - p ), o += rule.second, p = q + rule.first.size();
			}
			o.append( seg, p, std::string::npos );
			seg.swap( o );
		}
		r += seg, seg.clear();
	};
	for (size_t i = 0; i < t.size(); )
	{
		const size_t j = SkipNonCode( t, i, t.size() );
		if (j != i) { flush(); r.append( t, i, j - i ), i = j; continue; }
		seg += t[i++];
	}
	flush();
	return r;
}

static void DeTemplate( std::string& t, std::string& log )
{
	// --- pass 1: what is templated, and where the specializations are
	std::vector<Item> items = Scan( t, 0, t.size(), false );
	std::set<std::string> templated, templatedM; // classes declared TEMPLATED / TEMPLATED_M
	struct Spec { std::string owner, name, sig; std::string cond; bool body; bool isClass; size_t b, e; };
	std::vector<Spec> specs;
	std::vector<Entity> ents;
	{
		CondTracker ct;
		for (const Item& it : items)
		{
			if (it.kind == IT_PP) { ct.Feed( t.substr( it.b, it.e - it.b ) ); continue; }
			if (it.kind != IT_ENT) continue;
			Entity en;
			en.item = it;
			ClassifyEntity( en );
			en.cond = ct.Condition(), en.inImpl = ct.InImpl();
			if (en.kind == EK_CLASS && en.templated) (StartsWith( it.head, "TEMPLATED_M" ) ? templatedM : templated).insert( en.owner );
			if (en.outer.empty()) en.outer = en.owner;
			ents.push_back( en );
		}
	}
	// a member is identified by its qualified owner ('BVH::BVHNode'), name and signature
	auto qual = []( const Entity& en ) { return en.outer == en.owner ? en.owner : en.outer + "::" + en.owner; };
	for (const Entity& en : ents)
		if (en.specialization && templated.count( en.outer ) && (en.kind == EK_MEMBERDEF || en.kind == EK_CLASS))
			specs.push_back( { qual( en ), en.name, en.sig, en.cond, en.item.body, en.kind == EK_CLASS, en.item.b, en.item.e } );
	log += "templates off: " + std::to_string( templated.size() ) + " classes become plain classes, " + std::to_string( specs.size() ) + " specializations become ordinary code\n";

	// --- pass 2: structural edits
	std::vector<Edit> edits;
	auto condOf = [&]( const std::string& owner, const std::string& name, const std::string& sig, bool isClass, std::vector<const Spec*>* list )
	{
		std::string c;
		for (const Spec& s : specs)
		{
			if (s.owner != owner || s.isClass != isClass || !s.body) continue;
			if (!isClass && (s.name != name || s.sig != sig)) continue;
			c += (c.empty() ? "" : " || ") + std::string( "(" ) + (s.cond.empty() ? std::string( "1" ) : s.cond) + ")";
			if (list) list->push_back( &s );
		}
		return c;
	};
	int guarded = 0;
	for (const Entity& en : ents)
	{
		const Item& it = en.item;
		// explicit specializations: declarations go, definitions lose 'template <>'
		if (en.specialization && templated.count( en.outer ))
		{
			if (en.kind == EK_CLASS) { edits.push_back( WidenDeletion( t, it.b, it.e ) ); continue; } // moved to the generic, below
			if (!it.body) { edits.push_back( WidenDeletion( t, it.b, it.e ) ); continue; }
			const size_t p = t.find( "template", it.b ), q = t.find( '>', p );
			size_t r = q + 1;
			while (r < it.e && IsBlank( t[r] )) r++;
			edits.push_back( { p, r, "" } );
			continue;
		}
		// explicit instantiations: gone, except those of MBVH, which keeps its width parameter
		if (en.kind == EK_INSTANTIATION) { if (!templatedM.count( en.owner )) edits.push_back( WidenDeletion( t, it.b, it.e ) ); continue; }
		// a generic definition that a platform header replaces: compile it only where none does
		if (en.kind == EK_MEMBERDEF && en.templated && templated.count( en.outer ) && it.body)
		{
			const std::string c = condOf( qual( en ), en.name, en.sig, false, 0 );
			if (!c.empty())
			{
				const size_t ls = LineStart( t, it.b ), le = LineEnd( t, it.e - 1 );
				edits.push_back( { ls, ls, "#if !(" + c + ")\n" } );
				edits.push_back( { le, le, "#endif\n" } );
				guarded++;
			}
		}
		// a generic class with specializations: the specializations take its place, by condition
		if (en.kind == EK_CLASS && en.templated && templated.count( en.owner ) && it.body)
		{
			std::vector<const Spec*> list;
			const std::string c = condOf( en.owner, "", "", true, &list );
			if (!list.empty())
			{
				std::string rep;
				for (size_t k = 0; k < list.size(); k++)
				{
					std::string s = t.substr( list[k]->b, list[k]->e - list[k]->b );
					const size_t tp = s.find( "template" ), gt = s.find( '>', tp );
					size_t sp = gt + 1;
					while (sp < s.size() && IsBlank( s[sp] )) sp++;
					s = s.substr( 0, tp ) + s.substr( sp );
					const size_t im = s.find( "impl::" + en.owner );
					if (im != std::string::npos) s.erase( im, 6 );
					rep += std::string( k == 0 ? "#if " : "#elif " ) + (list[k]->cond.empty() ? std::string( "1" ) : list[k]->cond) + "\n" + s + "\n";
				}
				rep += "#else\n" + t.substr( it.b, it.e - it.b ) + "\n#endif";
				edits.push_back( { it.b, it.e, rep } );
			}
		}
		// member templates of plain classes exist for the double types: gone with them
		if (en.kind == EK_MEMBERDEF && en.templated && !templated.count( en.outer ) && !templatedM.count( en.outer )) { edits.push_back( WidenDeletion( t, it.b, it.e ) ); continue; }
		if (en.kind == EK_CLASS && !en.templated && it.body && !templated.count( en.owner ))
		{
			std::vector<Item> mem = Scan( t, it.bodyB + 1, it.bodyE, true );
			for (const Item& mi : mem) if (mi.kind == IT_ENT && StartsWith( mi.head, "TEMPLATED " )) edits.push_back( WidenDeletion( t, mi.b, mi.e ) );
		}
		// definitions outside the implementation section must be inline once they are not templates
		if (en.templated && !en.inImpl && it.body && (en.kind == EK_MEMBERDEF || en.kind == EK_FREEFUNC) && !StartsWith( en.item.head, "TEMPLATED_M" ) &&
			en.item.head.find( " inline " ) == std::string::npos)
		{
			const size_t p = t.find( "TEMPLATED", it.b );
			edits.push_back( { p + 9, p + 9, " inline" } );
		}
	}
	// the two TEMPLATED macros
	for (const Item& it : items)
	{
		if (it.kind != IT_PP) continue;
		std::string rest;
		if (Directive( it.head, &rest ) == "define" && (StartsWith( rest, "TEMPLATED " ) || StartsWith( rest, "TEMPLATED_M " )))
			edits.push_back( WidenDeletion( t, it.b, it.e ) );
	}
	// specializations without a generic counterpart are fine (the generic may not exist), but
	// report them: a signature that failed to match would otherwise show up as a redefinition
	int lonely = 0;
	for (const Spec& sp : specs)
	{
		if (sp.isClass || !sp.body) continue;
		bool found = false;
		for (const Entity& en : ents)
			if (en.kind == EK_MEMBERDEF && en.templated && !en.specialization && qual( en ) == sp.owner && en.name == sp.name && en.sig == sp.sig) found = true;
		if (!found && lonely++ < 40) log += "  note: " + sp.owner + "::" + sp.name + sp.sig + " has no generic definition\n";
	}
	ApplyEdits( t, edits );
	log += "  " + std::to_string( guarded ) + " generic definitions now compile only where no platform specialization exists\n";

	// --- pass 3: the packed instance index is decided by the preprocessor now, not by 'if constexpr',
	// because a discarded branch is only skipped in a template
	{
		const bool a = ReplaceLine( t, "if constexpr (!bvh_packed_inst<Index>) hit.inst = 0;", "#if INST_IDX_BITS == 32\n\t\thit.inst = 0;\n#endif" );
		const std::string p1 = "if constexpr (bvh_packed_inst<Index>) hit.prim = (prim & PRIM_IDX_MASK) + instIdx;";
		const size_t p = t.find( p1 );
		bool b = false;
		if (p != std::string::npos)
		{
			const size_t ls = LineStart( t, p ), le = LineEnd( t, p ), le2 = LineEnd( t, le );
			const std::string next = Trim( t.substr( le, le2 - le ) );
			if (next == "else hit.prim = prim, hit.inst = instIdx;")
			{
				t = t.substr( 0, ls ) + "#if INST_IDX_BITS != 32\n\t\thit.prim = (prim & PRIM_IDX_MASK) + instIdx;\n#else\n\t\thit.prim = prim, hit.inst = instIdx;\n#endif\n" + t.substr( le2 );
				b = true;
			}
		}
		if (!a || !b) log += "  warning: the packed instance index code in Ray has changed; check INST_IDX_BITS builds\n";
	}

	// --- pass 4: the template machinery itself
	t = ReplaceInCode( t, {
		{ "TEMPLATED_M ", "template <int M> " },
		{ "template <typename, typename> friend ", "friend " },
		{ "template <int, typename, typename> friend ", "template <int> friend " },
		{ "template <typename Float, typename Index, ", "template <" },
		{ ", typename Float, typename Index>", ">" },
		{ "template <typename Float, typename Index> ", "" },
		{ "TEMPLATED ", "" },
		{ "<Float, Index>", "" },
		{ "<float, uint32_t>", "" },
		{ ", Float, Index>", ">" },
		{ ", float, uint32_t>", ">" },
	} );

	// --- pass 5: Float and Index are float and uint32_t, except in templates that declare their own
	items = Scan( t, 0, t.size(), false );
	std::string out;
	out.reserve( t.size() );
	size_t p = 0;
	for (const Item& it : items)
	{
		if (it.kind != IT_ENT) continue;
		const bool ownF = it.head.find( "typename Float" ) != std::string::npos || it.head.find( "class Float" ) != std::string::npos;
		const bool ownI = it.head.find( "typename Index" ) != std::string::npos;
		std::vector<std::pair<std::string, std::string>> rules;
		if (!ownF) rules.push_back( { "Float", "float" } );
		if (!ownI) rules.push_back( { "Index", "uint32_t" } );
		out += ReplaceInCode( t.substr( p, it.b - p ), { { "Float", "float" }, { "Index", "uint32_t" } } );
		out += ReplaceInCode( t.substr( it.b, it.e - it.b ), rules );
		p = it.e;
	}
	out += ReplaceInCode( t.substr( p ), { { "Float", "float" }, { "Index", "uint32_t" } } );
	t.swap( out );
}

// ----------------------------------------------------------------------------
// The pipeline
// ----------------------------------------------------------------------------

struct GenResult
{
	bool ok = false;
	std::string text;	// the generated header
	std::string log;
};

static std::string ConfigSummary( const Options& o )
{
	std::string in, out;
	for (int i = 0; i < T_COUNT; i++)
	{
		std::string& s = o.on[i] ? in : out;
		s += (s.empty() ? "" : ", ") + std::string( kToggles[i].id );
	}
	return "included: " + (in.empty() ? std::string( "-" ) : in) + "\n//   excluded: " + (out.empty() ? std::string( "-" ) : out);
}

// Names of everything a configuration leaves out.
static RemovalPlan PlanFor( const Options& o, std::string& log )
{
	RemovalPlan plan;
	for (int i = 0; i < T_COUNT; i++) if (kToggles[i].cls && !o.on[i]) plan.classes.insert( kToggles[i].cls );
	// BVH_DOBB's lookup tables and conversion data; its constants go as orphans
	if (!o.on[T_BVH_DOBB]) plan.classes.insert( "DOBB_LUT" ), plan.classes.insert( "DOBBSidebandData" ), plan.macros.insert( "TINYBVH_DOBB_BUILTIN" );
	// the CPU layouts' leaf blocks
	if (!o.on[T_BVH4_CPU] && !o.on[T_BVH8_CPU]) plan.classes.insert( "BVHTri4Leaf" ), plan.classes.insert( "BVHTri1Leaf" );
	if (!o.on[T_OPTIMIZER]) for (const char* m : { "Optimize", "OptimizeSearch", "OptimizeNodes", "OptimizeReinsert" }) plan.members.insert( m );
	if (!o.on[T_MERGELEAFS]) for (const char* m : { "MergeLeafs", "mergeLeafCap" }) plan.members.insert( m );
	if (!o.on[T_PRESPLITTING]) for (const char* m : { "Presplit", "PresplitPostPass", "SIMDPresplit" }) plan.members.insert( m );
	if (!o.on[T_FULLSWEEP])
	{
		// the builder, its task trampoline, and the per-build buffers only it uses
		for (const char* m : { "BuildFullSweep", "GatherSweepBounds", "SweepBounds" }) plan.members.insert( m );
		// data members by class: their names are also used for locals and parameters elsewhere
		for (const char* m : { "sortedBnds", "tmpBnds", "sortedIdx", "SARs", "flag" }) plan.classMembers.insert( { "BVH", m } );
		plan.classes.insert( "BVHBuildFullSweepSubtree" );
	}
	if (!o.on[T_HPLOC]) plan.members.insert( "BuildHPLOC" );
	// without threading, BVHContext loses its job hooks; the built-in pool and the threaded build
	// paths go with ENABLE_THREADED_BUILDS
	if (!o.on[T_THREADING]) for (const char* m : { "spawn", "barrier", "parallel_for" }) plan.classMembers.insert( { "BVHContext", m } );
	std::string cls;
	for (const auto& c : plan.classes) cls += (cls.empty() ? "" : ", ") + c;
	if (!cls.empty()) log += "excluded classes: " + cls + "\n";
	return plan;
}

static GenResult Generate( const SourceSet& src, Options o )
{
	GenResult res;
	o = Resolve( o );
	try
	{
		for (const char* h : kHeaders) if (!src.files.count( h )) throw ScanError{ std::string( "missing " ) + h };
		std::string t = Merge( src, res.log );
		const std::string merged = t;
		RewritePreamble( t, o.on[T_X86], o.on[T_ARM], o.on[T_TEMPLATES], o.on[T_VOXELSET], o.on[T_THREADING], res.log );
		RemovalPlan plan = PlanFor( o, res.log );
		// the hand-written edits come first, while the code of every platform is still there: a
		// rule that does not match then means that the source changed, not that a platform's code
		// (the x86 ray bundles, for one) was left out
		if (!o.on[T_OPTIMIZER])
		{
			if (!ReplaceLine( t, "if (settings.postOptimize) Optimize(", "BVH_FATAL_ERROR_IF( settings.postOptimize, \"BVH::Build( .. ), settings.postOptimize: the optimizer is not part of this build of TinyBVH.\" );" ))
				res.log += "  warning: the postOptimize call in BVH::Build was not found\n";
			// the optimizer's own tuning constants
			for (const char* d : { "#define OPTIMIZE_TASKS ", "#define OPTIMIZE_NODES_PER_TASK " })
			{
				const size_t p = t.find( d );
				if (p != std::string::npos) t.erase( LineStart( t, p ), LineEnd( t, p ) - LineStart( t, p ) );
			}
		}
		std::set<std::string> active = plan.classes;
		for (int i = 0; i < T_COUNT; i++) if (!o.on[i]) active.insert( std::string( "-" ) + kToggles[i].id ), active.insert( "+reduced" );
		if (!o.on[T_X86] && !o.on[T_ARM]) active.insert( "-x86&-arm" );
		if (!o.on[T_PRESPLITTING]) FoldFalse( t, { "settings.usePresplitting", "SIMDPresplit()" }, res.log );
		ApplyRewrites( t, active, res.log );
		std::set<std::string> undef;
		if (!o.on[T_X86]) undef.insert( "BVH_USESSE" ), undef.insert( "BVH_USEAVX" ), undef.insert( "BVH_USEAVX2" );
		if (!o.on[T_ARM]) undef.insert( "BVH_USENEON" );
		if (!o.on[T_TEMPLATES]) undef.insert( "DOUBLE_PRECISION_SUPPORT" );
		if (!o.on[T_VOXELSET]) undef.insert( "ENABLE_VOXEL_SUPPORT" );
		if (!o.on[T_THREADING]) undef.insert( "ENABLE_THREADED_BUILDS" );
		t = Unifdef( t, undef, res.log );
		RemoveCode( t, plan, res.log );
		RemoveMacros( t, plan.macros, res.log );
		if (!plan.classes.empty() || !plan.members.empty()) RemoveUnusedStatics( t, res.log );
		if (t != merged)
		{
			RemoveOrphans( t, merged, res.log );
			CleanComments( t, plan.classes, res.log );
		}
		if (!o.on[T_TEMPLATES]) DeTemplate( t, res.log );
		if (t != merged) Tidy( t );
		// validation: nothing that was left out may still be named in code
		{
			const std::string code = CodeOnly( t );
			int problems = 0;
			std::set<std::string> names = plan.classes;
			for (const auto& m : plan.members) names.insert( m );
			if (!o.on[T_TEMPLATES]) for (const char* n : { "TEMPLATED", "TEMPLATED_M", "Float", "Index" }) names.insert( n );
			for (const auto& n : names)
			{
				for (size_t p = FindWord( code, n ); p != std::string::npos; p = FindWord( code, n, p + 1 ))
				{
					// Float / Index stay legitimate as template parameters of the remaining helpers
					if (n == "Float" || n == "Index") break;
					if (problems++ < 12) res.log += "error: '" + n + "' is still used at line " + std::to_string( LineOf( code, p ) ) + "\n";
				}
			}
			int depth = 0;
			for (const auto& l : SplitLines( t ))
			{
				const std::string d = Directive( l );
				if (IsIfLike( d )) depth++; else if (d == "endif") depth--;
			}
			if (depth != 0) res.log += "error: unbalanced #if / #endif in the result\n", problems++;
			// without threading, nothing may start, wait for or synchronize with a thread
			if (!o.on[T_THREADING])
			{
				for (const char* w : { "std::thread", "std::mutex", "std::condition_variable", "<thread>", "<mutex>", "<condition_variable>",
					".spawn", ".barrier", ".parallel_for", "->spawn", "->barrier", "->parallel_for", "tinybvh_builtin_", "ENABLE_THREADED_BUILDS" })
				{
					for (size_t p = code.find( w ); p != std::string::npos; p = code.find( w, p + 1 ))
					{
						const size_t e = p + strlen( w );
						if (IsIdent( w[strlen( w ) - 1] ) && e < code.size() && IsIdent( code[e] ) && w[strlen( w ) - 1] != '_') continue;
						if (problems++ < 12) res.log += std::string( "error: '" ) + w + "' is still used at line " + std::to_string( LineOf( code, p ) ) + "\n";
						break;
					}
				}
			}
			if (!o.on[T_VOXELSET] && code.find( "ENABLE_VOXEL_SUPPORT" ) != std::string::npos) res.log += "error: ENABLE_VOXEL_SUPPORT is still used\n", problems++;
			if (problems) throw ScanError{ std::to_string( problems ) + " problem(s); the generated header would not compile" };
		}
		// header comment
		std::string head = "// This file was generated by tinybvh_custom from tiny_bvh.h and its five platform\n"
			"// headers; edit those, not this. Configuration:\n//   " + ConfigSummary( o ) + "\n\n";
		res.text = head + t;
		if (src.crlf)
		{
			std::string c;
			c.reserve( res.text.size() + res.text.size() / 30 );
			for (const char ch : res.text) { if (ch == '\n') c += '\r'; c += ch; }
			res.text.swap( c );
		}
		res.log += "result: " + std::to_string( std::count( t.begin(), t.end(), '\n' ) ) + " lines\n";
		res.ok = true;
	}
	catch (const ScanError& e)
	{
		res.log += "error: " + e.what + "\n";
	}
	return res;
}

// ----------------------------------------------------------------------------
// File input / output
// ----------------------------------------------------------------------------

// Paths are UTF-8; on Windows they are converted for the wide file API.
static FILE* OpenPath( const std::string& path, const char* mode )
{
#ifdef _WIN32
	std::wstring w( MultiByteToWideChar( CP_UTF8, 0, path.c_str(), -1, 0, 0 ), 0 );
	MultiByteToWideChar( CP_UTF8, 0, path.c_str(), -1, &w[0], (int)w.size() );
	std::wstring m( mode, mode + strlen( mode ) );
	return _wfopen( w.c_str(), m.c_str() );
#else
	return fopen( path.c_str(), mode );
#endif
}

static bool ReadFile( const std::string& path, std::string& out )
{
	FILE* f = OpenPath( path, "rb" );
	if (!f) return false;
	std::string s;
	char buf[65536];
	size_t n;
	while ((n = fread( buf, 1, sizeof( buf ), f )) > 0) s.append( buf, n );
	fclose( f );
	out.swap( s );
	return true;
}

static bool WriteFile( const std::string& path, const std::string& data )
{
	FILE* f = OpenPath( path, "wb" );
	if (!f) return false;
	const bool ok = fwrite( data.data(), 1, data.size(), f ) == data.size();
	return fclose( f ) == 0 && ok;
}

static std::string JoinPath( const std::string& dir, const std::string& file )
{
	if (dir.empty()) return file;
	const char last = dir.back();
	return (last == '/' || last == '\\') ? dir + file : dir + "/" + file;
}

static bool LoadSources( const std::string& dir, SourceSet& src, std::string& log )
{
	bool ok = true;
	for (const char* h : kHeaders)
	{
		std::string s;
		if (!ReadFile( JoinPath( dir, h ), s )) { log += "error: cannot read " + JoinPath( dir, h ) + "\n"; ok = false; continue; }
		if (s.find( "\r\n" ) != std::string::npos) src.crlf = true;
		s.erase( std::remove( s.begin(), s.end(), '\r' ), s.end() );
		if (s.empty() || s.back() != '\n') s += '\n';
		src.files[h] = s;
	}
	return ok;
}

// Would writing 'file' overwrite one of the six headers in 'dir'?
static bool IsSourceFile( const std::string& dir, const std::string& file )
{
	namespace fs = std::filesystem;
	std::error_code ec;
	const fs::path out = fs::u8path( file );
	if (!fs::exists( out, ec )) return false;
	for (const char* h : kHeaders) if (fs::equivalent( out, fs::u8path( JoinPath( dir, h ) ), ec )) return true;
	return false;
}

} // namespace gen

// ============================================================================
// Command line: tinybvh_custom --in <folder> --out <file> [--no-<toggle>] [--with-<toggle>]
// ============================================================================

static void PrintUsage( FILE* f )
{
	fprintf( f, "usage: tinybvh_custom [--in <folder with the six headers>] [--out <header>] [options]\n"
		"  --no-<id> leaves a part out, --with-<id> puts it in; everything is in by default,\n"
		"  except mbvh and bvh_verbose, which are in only when something needs them.\n"
		"  --no-all-layouts leaves out every layout except BVH; --describe prints the ids, labels\n"
		"  and dependencies as JSON, for other front ends. Ids:\n" );
	for (int i = 0; i < gen::T_COUNT; i++) fprintf( f, "    %-14s %s\n", gen::kToggles[i].id, gen::kToggles[i].label );
}

// --describe: the toggles as JSON, for other front ends such as the web page. Per toggle: its id,
// the label to show, the class it leaves out (or null), where it goes, whether it is in by default,
// and which toggles need it (the two intermediate layouts only).
static std::string JsonString( const std::string& s )
{
	std::string r = "\"";
	for (const char c : s) { if (c == '"' || c == '\\') r += '\\'; r += c; }
	return r + "\"";
}
static void PrintDescription( FILE* f )
{
	using namespace gen;
	const Options defaults;
	fprintf( f, "{\n  \"groups\": [\n" );
	for (int g = 0; g < 3; g++) fprintf( f, "    { \"title\": %s, \"rows\": %d }%s\n", JsonString( kGroupTitle[g] ).c_str(), kRows[g], g < 2 ? "," : "" );
	fprintf( f, "  ],\n  \"toggles\": [\n" );
	int n = 0;
	for (const Place& p : kPlace)
	{
		std::string needed;
		auto list = [&]( const Toggle* b, const Toggle* e ) { for (; b != e; b++) needed += (needed.empty() ? "" : ", ") + JsonString( kToggles[*b].id ); };
		if (p.t == T_MBVH) list( std::begin( kNeedsMBVH ), std::end( kNeedsMBVH ) );
		if (p.t == T_BVH_VERBOSE) list( std::begin( kNeedsVerbose ), std::end( kNeedsVerbose ) );
		const ToggleInfo& t = kToggles[p.t];
		fprintf( f, "    { \"id\": %s, \"label\": %s, \"class\": %s, \"group\": %d, \"col\": %d, \"row\": %d, \"default\": %s, \"neededBy\": [%s] }%s\n",
			JsonString( t.id ).c_str(), JsonString( DisplayLabel( p.t ) ).c_str(), t.cls ? JsonString( t.cls ).c_str() : "null", p.group, p.col, p.row,
			defaults.on[p.t] ? "true" : "false", needed.c_str(), ++n < (int)(sizeof( kPlace ) / sizeof( kPlace[0] )) ? "," : "" );
	}
	fprintf( f, "  ]\n}\n" );
}

// Returns the process exit code.
static int RunCommandLine( const std::vector<std::string>& args, FILE* out, FILE* err )
{
	using namespace gen;
	std::string in = ".", file = "tiny_bvh_custom.h";
	Options o;
	for (size_t i = 0; i < args.size(); i++)
	{
		const std::string& a = args[i];
		if (a == "--in" && i + 1 < args.size()) in = args[++i];
		else if (a == "--out" && i + 1 < args.size()) file = args[++i];
		else if (a == "--help" || a == "-h" || a == "/?") { PrintUsage( out ); return 0; }
		else if (a == "--describe") { PrintDescription( out ); return 0; }
		else if (StartsWith( a, "--no-" ) || StartsWith( a, "--with-" ))
		{
			const bool on = StartsWith( a, "--with-" );
			const std::string id = a.substr( on ? 7 : 5 );
			bool found = false;
			for (int t = 0; t < T_COUNT; t++) if (id == kToggles[t].id) o.on[t] = on, found = true;
			if (id == "all-layouts") { for (int t = T_BVH_GPU; t <= T_BVH_VERBOSE; t++) o.on[t] = on; found = true; }
			if (!found) { fprintf( err, "unknown option %s\n", a.c_str() ); PrintUsage( err ); return 2; }
		}
		else { fprintf( err, "unknown argument %s\n", a.c_str() ); PrintUsage( err ); return 2; }
	}
	SourceSet src;
	std::string log;
	if (!LoadSources( in, src, log )) { fputs( log.c_str(), err ); return 1; }
	if (IsSourceFile( in, file )) { fprintf( err, "refusing to overwrite %s, one of the six source headers\n", file.c_str() ); return 1; }
	GenResult r = Generate( src, o );
	fputs( r.log.c_str(), out );
	if (!r.ok) return 1;
	if (!WriteFile( file, r.text )) { fprintf( err, "cannot write %s\n", file.c_str() ); return 1; }
	fprintf( out, "wrote %s\n", file.c_str() );
	return 0;
}

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
// The web page's entry: the command line's arguments, one per line. The page puts the six headers
// into the module's file system first, and reads the result from it afterwards.
extern "C" EMSCRIPTEN_KEEPALIVE int tinybvh_custom_run( const char* argLines )
{
	std::vector<std::string> args;
	std::string a;
	for (const char* c = argLines;; c++)
	{
		if (*c && *c != '\n') { a += *c; continue; }
		if (!a.empty()) args.push_back( a ), a.clear();
		if (!*c) break;
	}
	const int code = RunCommandLine( args, stdout, stderr );
	fflush( stdout ), fflush( stderr );
	return code;
}
#endif

#ifndef _WIN32

int main( int argc, char** argv )
{
	std::vector<std::string> args( argv + 1, argv + argc );
	return RunCommandLine( args, stdout, stderr );
}

#else

// ============================================================================
// Windows: a small window with a checkbox per toggle
// ============================================================================

enum
{
	IDC_SRC = 100, IDC_SRC_BROWSE, IDC_OUT, IDC_OUT_BROWSE, IDC_GENERATE, IDC_LOG, IDC_STATUS,
	IDC_TOGGLE = 200 // + toggle index
};

static HWND g_wnd, g_src, g_out, g_log, g_status, g_check[gen::T_COUNT];
static HWND g_srcLabel, g_srcBrowse, g_group[3], g_outLabel, g_outBrowse, g_generate;
static HFONT g_font, g_mono;
static int g_dpi = 96;
static bool g_userChoice[gen::T_COUNT]; // what the user ticked, for the two intermediate layouts

static int S( const int v ) { return MulDiv( v, g_dpi, 96 ); }

static std::wstring Wide( const std::string& s )
{
	if (s.empty()) return std::wstring();
	const int n = MultiByteToWideChar( CP_UTF8, 0, s.data(), (int)s.size(), 0, 0 );
	std::wstring w( n, 0 );
	MultiByteToWideChar( CP_UTF8, 0, s.data(), (int)s.size(), &w[0], n );
	return w;
}

static std::string Utf8( const std::wstring& w )
{
	if (w.empty()) return std::string();
	const int n = WideCharToMultiByte( CP_UTF8, 0, w.data(), (int)w.size(), 0, 0, 0, 0 );
	std::string s( n, 0 );
	WideCharToMultiByte( CP_UTF8, 0, w.data(), (int)w.size(), &s[0], n, 0, 0 );
	return s;
}

static std::wstring GetText( HWND h )
{
	const int n = GetWindowTextLengthW( h );
	std::wstring s( n + 1, 0 );
	GetWindowTextW( h, &s[0], n + 1 );
	s.resize( n );
	return s;
}

static bool Checked( int t ) { return SendMessageW( g_check[t], BM_GETCHECK, 0, 0 ) == BST_CHECKED; }
static void SetCheck( int t, bool on ) { SendMessageW( g_check[t], BM_SETCHECK, on ? BST_CHECKED : BST_UNCHECKED, 0 ); }

static gen::Options CurrentOptions()
{
	gen::Options o;
	for (int t = 0; t < gen::T_COUNT; t++) o.on[t] = Checked( t );
	// the intermediate layouts: what the user chose, the generator adds them where required
	o.on[gen::T_MBVH] = g_userChoice[gen::T_MBVH];
	o.on[gen::T_BVH_VERBOSE] = g_userChoice[gen::T_BVH_VERBOSE];
	return o;
}

// The two intermediate layouts are ticked and locked while something needs them.
static void UpdateDependencies()
{
	gen::Options o;
	for (int t = 0; t < gen::T_COUNT; t++) o.on[t] = Checked( t );
	const bool needMBVH = gen::RequiresMBVH( o ), needVerbose = gen::RequiresVerbose( o );
	SetCheck( gen::T_MBVH, needMBVH || g_userChoice[gen::T_MBVH] );
	EnableWindow( g_check[gen::T_MBVH], !needMBVH );
	SetCheck( gen::T_BVH_VERBOSE, needVerbose || g_userChoice[gen::T_BVH_VERBOSE] );
	EnableWindow( g_check[gen::T_BVH_VERBOSE], !needVerbose );
}

static void Log( const std::string& s )
{
	std::string crlf;
	for (const char c : s) { if (c == '\n') crlf += '\r'; crlf += c; }
	SetWindowTextW( g_log, Wide( crlf ).c_str() );
	SendMessageW( g_log, EM_SETSEL, 0, 0 ), SendMessageW( g_log, EM_SCROLLCARET, 0, 0 );
}

static std::wstring ExeFolder()
{
	wchar_t path[MAX_PATH];
	const DWORD n = GetModuleFileNameW( 0, path, MAX_PATH );
	std::wstring p( path, n );
	const size_t s = p.find_last_of( L"\\/" );
	return s == std::wstring::npos ? L"." : p.substr( 0, s );
}

static bool HasHeaders( const std::wstring& dir )
{
	return GetFileAttributesW( (dir + L"\\tiny_bvh.h").c_str() ) != INVALID_FILE_ATTRIBUTES &&
		GetFileAttributesW( (dir + L"\\tiny_bvh_base.h").c_str() ) != INVALID_FILE_ATTRIBUTES;
}

static void BrowseSource()
{
	BROWSEINFOW bi = {};
	bi.hwndOwner = g_wnd;
	bi.lpszTitle = L"Folder with tiny_bvh.h and its five platform headers";
	bi.ulFlags = BIF_RETURNONLYFSDIRS | BIF_NEWDIALOGSTYLE;
	PIDLIST_ABSOLUTE pidl = SHBrowseForFolderW( &bi );
	if (!pidl) return;
	wchar_t path[MAX_PATH];
	if (SHGetPathFromIDListW( pidl, path )) SetWindowTextW( g_src, path );
	CoTaskMemFree( pidl );
}

static void BrowseOutput()
{
	wchar_t file[MAX_PATH] = {};
	const std::wstring cur = GetText( g_out );
	wcsncpy( file, cur.c_str(), MAX_PATH - 1 );
	OPENFILENAMEW ofn = {};
	ofn.lStructSize = sizeof( ofn );
	ofn.hwndOwner = g_wnd;
	ofn.lpstrFilter = L"C++ headers (*.h)\0*.h\0All files (*.*)\0*.*\0";
	ofn.lpstrFile = file;
	ofn.nMaxFile = MAX_PATH;
	ofn.lpstrDefExt = L"h";
	ofn.Flags = OFN_OVERWRITEPROMPT | OFN_PATHMUSTEXIST;
	if (GetSaveFileNameW( &ofn )) SetWindowTextW( g_out, file );
}

static void GenerateHeader()
{
	const std::wstring srcDir = GetText( g_src );
	std::wstring outFile = GetText( g_out );
	if (outFile.empty()) { Log( "Enter a name for the generated header.\n" ); return; }
	// a bare file name goes next to the sources
	if (outFile.find_first_of( L"\\/:" ) == std::wstring::npos) outFile = srcDir + L"\\" + outFile;
	gen::SourceSet src;
	std::string log;
	if (!gen::LoadSources( Utf8( srcDir ), src, log )) { Log( log + "\nPick the folder that holds tiny_bvh.h and its five platform headers.\n" ); return; }
	if (gen::IsSourceFile( Utf8( srcDir ), Utf8( outFile ) )) { Log( "The output would overwrite one of the six source headers; choose another name.\n" ); return; }
	SetWindowTextW( g_status, L"Generating..." );
	UpdateWindow( g_status );
	gen::GenResult r = gen::Generate( src, CurrentOptions() );
	if (r.ok && !gen::WriteFile( Utf8( outFile ), r.text )) r.ok = false, r.log += "error: cannot write " + Utf8( outFile ) + "\n";
	if (r.ok) r.log = "Wrote " + Utf8( outFile ) + "\n\n" + r.log;
	SetWindowTextW( g_status, r.ok ? L"Done." : L"Failed; see the log." );
	Log( r.log );
}

static HWND Make( const wchar_t* cls, const wchar_t* text, DWORD style, int x, int y, int w, int h, int id, DWORD ex = 0 )
{
	HWND c = CreateWindowExW( ex, cls, text, WS_CHILD | WS_VISIBLE | style, S( x ), S( y ), S( w ), S( h ), g_wnd, (HMENU)(INT_PTR)id, GetModuleHandleW( 0 ), 0 );
	SendMessageW( c, WM_SETFONT, (WPARAM)g_font, TRUE );
	return c;
}

static const int kMinW = 560, kMinH = 600; // smallest client area, in 96 dpi units

// Places the controls for the current client size: the fields and the log stretch, the
// buttons stay right, and the two checkbox columns split the width.
static void LayoutControls()
{
	RECT rc;
	GetClientRect( g_wnd, &rc );
	const int W = std::max( kMinW, MulDiv( rc.right, 96, g_dpi ) ), H = std::max( kMinH, MulDiv( rc.bottom, 96, g_dpi ) );
	const int m = 12, bw = 92, rowH = 22;
	HDWP dwp = BeginDeferWindowPos( 40 );
	auto put = [&]( HWND h, int x, int y, int w, int hh ) { if (dwp) dwp = DeferWindowPos( dwp, h, 0, S( x ), S( y ), S( w ), S( hh ), SWP_NOZORDER | SWP_NOACTIVATE ); };
	put( g_srcLabel, m, 12, W - 2 * m, 18 );
	put( g_src, m, 32, W - 2 * m - bw - 12, 23 );
	put( g_srcBrowse, W - m - bw, 31, bw, 25 );
	int y = 66;
	int groupY[3];
	for (int g = 0; g < 3; g++)
	{
		const int h = 22 + gen::kRows[g] * rowH + 8;
		groupY[g] = y;
		put( g_group[g], m, y, W - 2 * m, h );
		y += h + 8;
	}
	const int colW = (W - 2 * m) / 2;
	for (const gen::Place& p : gen::kPlace) put( g_check[p.t], m + 12 + p.col * colW, groupY[p.group] + 22 + p.row * rowH, colW - 20, 20 );
	put( g_outLabel, m, y + 4, W - 2 * m, 18 );
	put( g_out, m, y + 24, W - 2 * m - bw - 12, 23 );
	put( g_outBrowse, W - m - bw, y + 23, bw, 25 );
	put( g_generate, m, y + 60, 120, 28 );
	put( g_status, m + 132, y + 66, W - 2 * m - 132, 18 );
	put( g_log, m, y + 98, W - 2 * m, std::max( 60, H - (y + 98) - m ) );
	if (dwp) EndDeferWindowPos( dwp );
	InvalidateRect( g_wnd, 0, TRUE ); // the group boxes do not repaint their insides
}

static void CreateControls()
{
	// created in tab order; LayoutControls places them
	g_srcLabel = Make( L"STATIC", L"Folder with tiny_bvh.h and its five platform headers:", 0, 0, 0, 10, 10, -1 );
	g_src = Make( L"EDIT", L"", WS_TABSTOP | ES_AUTOHSCROLL, 0, 0, 10, 10, IDC_SRC, WS_EX_CLIENTEDGE );
	g_srcBrowse = Make( L"BUTTON", L"Browse...", WS_TABSTOP | BS_PUSHBUTTON, 0, 0, 10, 10, IDC_SRC_BROWSE );
	for (int g = 0; g < 3; g++) g_group[g] = Make( L"BUTTON", Wide( gen::kGroupTitle[g] ).c_str(), BS_GROUPBOX, 0, 0, 10, 10, -1 );
	for (const gen::Place& p : gen::kPlace)
	{
		const std::wstring label = Wide( gen::DisplayLabel( p.t ) );
		g_check[p.t] = Make( L"BUTTON", label.c_str(), WS_TABSTOP | BS_AUTOCHECKBOX, 0, 0, 10, 10, IDC_TOGGLE + p.t );
		SetCheck( p.t, p.t != gen::T_MBVH && p.t != gen::T_BVH_VERBOSE );
	}
	g_outLabel = Make( L"STATIC", L"Generated header (a bare name is written next to the sources):", 0, 0, 0, 10, 10, -1 );
	g_out = Make( L"EDIT", L"tiny_bvh_custom.h", WS_TABSTOP | ES_AUTOHSCROLL, 0, 0, 10, 10, IDC_OUT, WS_EX_CLIENTEDGE );
	g_outBrowse = Make( L"BUTTON", L"Save as...", WS_TABSTOP | BS_PUSHBUTTON, 0, 0, 10, 10, IDC_OUT_BROWSE );
	g_generate = Make( L"BUTTON", L"Generate", WS_TABSTOP | BS_DEFPUSHBUTTON, 0, 0, 10, 10, IDC_GENERATE );
	g_status = Make( L"STATIC", L"", 0, 0, 0, 10, 10, IDC_STATUS );
	g_log = Make( L"EDIT", L"", ES_MULTILINE | ES_READONLY | ES_AUTOVSCROLL | ES_AUTOHSCROLL | WS_VSCROLL | WS_HSCROLL, 0, 0, 10, 10, IDC_LOG, WS_EX_CLIENTEDGE );
	SendMessageW( g_log, WM_SETFONT, (WPARAM)g_mono, TRUE );
	LayoutControls();
	// the sources: next to the tool, or in the current folder
	std::wstring dir = ExeFolder();
	if (!HasHeaders( dir ))
	{
		wchar_t cwd[MAX_PATH];
		GetCurrentDirectoryW( MAX_PATH, cwd );
		if (HasHeaders( cwd )) dir = cwd;
	}
	SetWindowTextW( g_src, dir.c_str() );
	UpdateDependencies();
	Log( "Untick what you do not need and press Generate. With everything ticked, the result holds\n"
		"all the code of the six headers. MBVH and BVH_Verbose are included whenever a ticked layout\n"
		"or feature needs them; tick them to keep them regardless.\n" );
}

// The smallest window that still fits the controls.
static void MinTrackSize( MINMAXINFO* mm )
{
	RECT r = { 0, 0, S( kMinW ), S( kMinH ) };
	AdjustWindowRect( &r, WS_OVERLAPPEDWINDOW, FALSE );
	mm->ptMinTrackSize.x = r.right - r.left, mm->ptMinTrackSize.y = r.bottom - r.top;
}

static LRESULT CALLBACK WndProc( HWND h, UINT msg, WPARAM wp, LPARAM lp )
{
	switch (msg)
	{
	case WM_CREATE:
		g_wnd = h;
		CreateControls();
		return 0;
	case WM_SIZE:
		if (g_log && wp != SIZE_MINIMIZED) LayoutControls();
		return 0;
	case WM_GETMINMAXINFO:
		MinTrackSize( (MINMAXINFO*)lp );
		return 0;
	case WM_COMMAND:
	{
		const int id = LOWORD( wp );
		if (id == IDC_SRC_BROWSE) BrowseSource();
		else if (id == IDC_OUT_BROWSE) BrowseOutput();
		else if (id == IDC_GENERATE || id == IDOK) GenerateHeader(); // IDOK: Enter
		else if (id >= IDC_TOGGLE && id < IDC_TOGGLE + gen::T_COUNT && HIWORD( wp ) == BN_CLICKED)
		{
			const int t = id - IDC_TOGGLE;
			if (t == gen::T_MBVH || t == gen::T_BVH_VERBOSE) g_userChoice[t] = Checked( t );
			UpdateDependencies();
		}
		return 0;
	}
	case WM_CTLCOLORSTATIC:
	{
		// the read-only log keeps a white background
		if ((HWND)lp == g_log) { SetBkColor( (HDC)wp, GetSysColor( COLOR_WINDOW ) ); return (LRESULT)GetSysColorBrush( COLOR_WINDOW ); }
		break;
	}
	case WM_DESTROY:
		PostQuitMessage( 0 );
		return 0;
	}
	return DefWindowProcW( h, msg, wp, lp );
}

static int RunConsole( int argc, wchar_t** argv )
{
	// started from a console: report there
	const bool attached = AttachConsole( ATTACH_PARENT_PROCESS ) != 0;
	FILE* out = stdout, * err = stderr;
	if (attached) { out = _wfopen( L"CONOUT$", L"w" ); err = out; }
	std::vector<std::string> args;
	for (int i = 1; i < argc; i++) args.push_back( Utf8( argv[i] ) );
	const int rc = RunCommandLine( args, out ? out : stdout, err ? err : stderr );
	if (attached && out) fclose( out );
	return rc;
}

int WINAPI wWinMain( HINSTANCE inst, HINSTANCE, PWSTR, int show )
{
	int argc = 0;
	wchar_t** argv = CommandLineToArgvW( GetCommandLineW(), &argc );
	if (argc > 1) { const int rc = RunConsole( argc, argv ); LocalFree( argv ); return rc; }
	LocalFree( argv );
	SetProcessDPIAware();
	CoInitializeEx( 0, COINIT_APARTMENTTHREADED );
	{
		HDC dc = GetDC( 0 );
		g_dpi = GetDeviceCaps( dc, LOGPIXELSY );
		ReleaseDC( 0, dc );
	}
	NONCLIENTMETRICSW ncm = {};
	ncm.cbSize = sizeof( ncm );
	SystemParametersInfoW( SPI_GETNONCLIENTMETRICS, sizeof( ncm ), &ncm, 0 );
	g_font = CreateFontIndirectW( &ncm.lfMessageFont );
	LOGFONTW mono = ncm.lfMessageFont;
	wcscpy( mono.lfFaceName, L"Consolas" );
	g_mono = CreateFontIndirectW( &mono );
	WNDCLASSEXW wc = {};
	wc.cbSize = sizeof( wc );
	wc.lpfnWndProc = WndProc;
	wc.hInstance = inst;
	wc.hCursor = LoadCursorW( 0, (LPCWSTR)IDC_ARROW );
	wc.hbrBackground = (HBRUSH)(COLOR_BTNFACE + 1);
	wc.lpszClassName = L"TinyBVHCustom";
	wc.hIcon = LoadIconW( 0, (LPCWSTR)IDI_APPLICATION );
	RegisterClassExW( &wc );
	RECT r = { 0, 0, S( 640 ), S( 740 ) };
	const DWORD style = WS_OVERLAPPEDWINDOW; // resizable; LayoutControls follows the size
	AdjustWindowRect( &r, style, FALSE );
	HWND w = CreateWindowExW( 0, wc.lpszClassName, L"TinyBVH: custom single header", style, CW_USEDEFAULT, CW_USEDEFAULT, r.right - r.left, r.bottom - r.top, 0, 0, inst, 0 );
	ShowWindow( w, show );
	MSG m;
	while (GetMessageW( &m, 0, 0, 0 ) > 0)
	{
		// Ctrl+A selects all text in an edit field, which the standard edit control does not do
		if (m.message == WM_KEYDOWN && m.wParam == 'A' && GetKeyState( VK_CONTROL ) < 0)
		{
			wchar_t cls[16] = {};
			GetClassNameW( m.hwnd, cls, 16 );
			if (_wcsicmp( cls, L"Edit" ) == 0) { SendMessageW( m.hwnd, EM_SETSEL, 0, -1 ); continue; }
		}
		if (IsDialogMessageW( w, &m )) continue; // tab between controls, Enter for Generate
		TranslateMessage( &m ), DispatchMessageW( &m );
	}
	DeleteObject( g_font ), DeleteObject( g_mono );
	CoUninitialize();
	return 0;
}

#endif
