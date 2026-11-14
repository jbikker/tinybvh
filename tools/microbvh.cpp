// microbvh.cpp - TinyBVH single-header condenser.
//
// Merges the six TinyBVH headers into one heavily condensed header, microbvh.h,
// that is a drop-in replacement for tiny_bvh.h: the public interface, the
// configuration macros and the observable behaviour are unchanged.
//
// What it does:
//  - splices tiny_bvh_base.h and the platform headers into tiny_bvh.h;
//  - regenerates the SIMD detection block for one target ISA and folds away
//    every preprocessor branch that the ISA choice decides;
//  - removes all comments except the license, and all layout whitespace;
//  - renames function parameters and local variables to one or two characters;
//  - factors repeating token sequences into generated object-like macros;
//  - repacks the result into lines of at most 100 characters.
//
// Build: cl /O2 /std:c++17 microbvh.cpp   or   g++ -O2 -std=c++17 -o microbvh microbvh.cpp
// Run:   microbvh --isa avx2 --out microbvh.h
//
// Jacco Bikker / TinyBVH.

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <climits>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace std;

// ============================================================================
//
//        T O K E N S   A N D   C H U N K S
//
// ============================================================================

enum TokKind : uint8_t { T_ID, T_NUM, T_STR, T_PUNCT };

struct Tok
{
	string s;
	uint8_t k = T_PUNCT;
	bool ws = false;			// whitespace preceded this token in the source
	Tok() = default;
	Tok( const string& a, uint8_t b, bool c = false ) : s( a ), k( b ), ws( c ) {}
	bool is( const char* p ) const { return s == p; }
	bool id() const { return k == T_ID; }
};

// A chunk is either one preprocessor directive (which must occupy its own
// output line) or a run of code tokens, which may be repacked freely.
struct Chunk
{
	bool dir = false;
	vector<Tok> t;
	string raw;					// verbatim tail, for #include
	bool rawTail = false;
};

static void fatal( const string& msg )
{
	fprintf( stderr, "microbvh: %s\n", msg.c_str() );
	exit( 1 );
}

static string readFile( const string& path )
{
	ifstream f( path, ios::binary );
	if (!f) fatal( "cannot open " + path );
	ostringstream ss;
	ss << f.rdbuf();
	return ss.str();
}

// ============================================================================
//
//        L E X E R
//
// ============================================================================

static bool idStart( char c ) { return isalpha( (unsigned char)c ) || c == '_' || c == '$'; }
static bool idChar( char c ) { return isalnum( (unsigned char)c ) || c == '_' || c == '$'; }

struct Lexer
{
	const string& src;
	size_t p = 0;
	bool lineStart = true;
	bool sawWs = false;
	explicit Lexer( const string& s ) : src( s ) {}

	bool eof() const { return p >= src.size(); }
	char cur() const { return p < src.size() ? src[p] : 0; }
	char at( size_t i ) const { return p + i < src.size() ? src[p + i] : 0; }

	// Skip whitespace, comments and line splices. Returns false at a newline,
	// which is consumed; the caller then knows a new source line started.
	bool skipSpace( bool& newline )
	{
		newline = false;
		for (; p < src.size(); )
		{
			char c = src[p];
			if (c == '\\' && (at( 1 ) == '\n' || (at( 1 ) == '\r' && at( 2 ) == '\n')))
			{
				p += at( 1 ) == '\r' ? 3 : 2, sawWs = true; // line splice: logical line continues
				continue;
			}
			if (c == '\n') { p++, newline = true; return false; }
			if (c == ' ' || c == '\t' || c == '\r' || c == '\f' || c == '\v') { p++, sawWs = true; continue; }
			if (c == '/' && at( 1 ) == '/')
			{
				while (p < src.size() && src[p] != '\n')
				{
					// a line comment can itself be spliced onto the next line
					if (src[p] == '\\' && (at( 1 ) == '\n' || (at( 1 ) == '\r' && at( 2 ) == '\n')))
						p += at( 1 ) == '\r' ? 3 : 2; else p++;
				}
				sawWs = true;
				continue;
			}
			if (c == '/' && at( 1 ) == '*')
			{
				p += 2;
				bool crossed = false;
				while (p < src.size() && !(src[p] == '*' && at( 1 ) == '/'))
				{
					if (src[p] == '\n') crossed = true;
					p++;
				}
				if (p < src.size()) p += 2;
				sawWs = true;
				if (crossed) { newline = true; return false; } // comment swallowed the line end
				continue;
			}
			return true;
		}
		return false;
	}

	// Read one token at the current position; assumes skipSpace returned true.
	Tok next()
	{
		bool ws = sawWs;
		sawWs = false;
		size_t s0 = p;
		char c = src[p];
		if (idStart( c ))
		{
			// string / char literal prefixes stay attached to the literal
			size_t q = p;
			while (q < src.size() && idChar( src[q] )) q++;
			string w = src.substr( p, q - p );
			if (q < src.size() && (src[q] == '"' || src[q] == '\'') &&
				(w == "L" || w == "u" || w == "U" || w == "u8" || w == "R" || w == "LR" || w == "uR" || w == "UR" || w == "u8R"))
			{
				p = q; Tok t = lexString(); t.s = w + t.s; t.ws = ws; return t;
			}
			p = q;
			return Tok( w, T_ID, ws );
		}
		if (isdigit( (unsigned char)c ) || (c == '.' && isdigit( (unsigned char)at( 1 ) )))
		{
			// pp-number: digits, letters, '_', '.', '\'' and exponent signs
			p++;
			while (p < src.size())
			{
				char d = src[p];
				if ((d == '+' || d == '-') && p > s0)
				{
					char e = src[p - 1];
					if (e == 'e' || e == 'E' || e == 'p' || e == 'P') { p++; continue; }
					break;
				}
				if (idChar( d ) || d == '.' || d == '\'') { p++; continue; }
				break;
			}
			return Tok( src.substr( s0, p - s0 ), T_NUM, ws );
		}
		if (c == '"' || c == '\'') { Tok t = lexString(); t.ws = ws; return t; }
		// punctuators, longest match first
		static const char* P3[] = { "<<=", ">>=", "->*", "...", "%:%" };
		static const char* P2[] = { "<<", ">>", "<=", ">=", "==", "!=", "&&", "||", "++", "--",
			"+=", "-=", "*=", "/=", "%=", "&=", "|=", "^=", "->", "::", ".*", "##", "<:", ":>", "<%", "%>", "%:" };
		for (const char* q : P3) if (!strncmp( &src[p], q, 3 )) { p += 3; return Tok( q, T_PUNCT, ws ); }
		for (const char* q : P2) if (p + 1 < src.size() && src[p] == q[0] && src[p + 1] == q[1] )
			{ p += 2; return Tok( q, T_PUNCT, ws ); }
		p++;
		return Tok( string( 1, c ), T_PUNCT, ws );
	}

	Tok lexString()
	{
		size_t s0 = p;
		char q = src[p++];
		if (q == '"' && p >= 2 && src[p - 2] == 'R') { /* raw strings are not used by TinyBVH */ }
		while (p < src.size() && src[p] != q)
		{
			if (src[p] == '\\') p++;
			if (src[p] == '\\' && (at( 1 ) == '\n')) p++;
			p++;
		}
		if (p < src.size()) p++;
		// user-defined literal suffix
		while (p < src.size() && idChar( src[p] )) p++;
		return Tok( src.substr( s0, p - s0 ), T_STR, false );
	}
};

// Turn source text into chunks. Directives become one chunk each; code tokens
// accumulate into a single chunk until the next directive.
static void lexToChunks( const string& src, vector<Chunk>& out )
{
	Lexer lx( src );
	bool atLineStart = true;
	Chunk code;
	auto flushCode = [&]() { if (!code.t.empty()) { out.push_back( code ); code.t.clear(); } };
	for (; ; )
	{
		bool nl = false;
		if (!lx.skipSpace( nl ))
		{
			if (nl) { atLineStart = true; continue; }
			break; // end of file
		}
		if (atLineStart && lx.cur() == '#')
		{
			lx.p++, lx.sawWs = false;
			Chunk d;
			d.dir = true;
			// directive name
			bool nl2 = false;
			if (!lx.skipSpace( nl2 )) { if (nl2) { atLineStart = true; continue; } break; }
			Tok name = lx.next();
			d.t.push_back( name );
			if (name.is( "include" ) || name.is( "include_next" ))
			{
				// keep the header-name verbatim: <a.h> would not tokenize sanely
				size_t s0 = lx.p;
				while (lx.p < src.size() && src[lx.p] != '\n') lx.p++;
				d.raw = src.substr( s0, lx.p - s0 );
				while (!d.raw.empty() && isspace( (unsigned char)d.raw.back() )) d.raw.pop_back();
				size_t b = d.raw.find_first_not_of( " \t" );
				d.raw = b == string::npos ? "" : d.raw.substr( b );
				d.rawTail = true;
				flushCode();
				out.push_back( d );
				atLineStart = true;
				continue;
			}
			for (; ; )
			{
				bool nl3 = false;
				if (!lx.skipSpace( nl3 )) break;
				d.t.push_back( lx.next() );
			}
			flushCode();
			out.push_back( d );
			atLineStart = true;
			continue;
		}
		atLineStart = false;
		code.t.push_back( lx.next() );
	}
	flushCode();
}

// ============================================================================
//
//        P R E P R O C E S S O R   F O L D I N G
//
// ============================================================================

// Values we assert for the selected ISA; every other symbol stays unknown and
// its conditionals are emitted unchanged.
struct SymTab { map<string, bool> known; map<string, long long> value; };

struct EvalRes { bool known; long long v; };
static const EvalRes UNK{ false, 0 };

struct CondEval
{
	const vector<Tok>& t;
	const SymTab& S;
	size_t i;
	CondEval( const vector<Tok>& tt, const SymTab& s ) : t( tt ), S( s ), i( 0 ) {}
	bool has() const { return i < t.size(); }
	const string& peek() const { static string e; return i < t.size() ? t[i].s : e; }
	bool eat( const char* p ) { if (i < t.size() && t[i].s == p) { i++; return true; } return false; }

	EvalRes primary()
	{
		if (!has()) return UNK;
		if (eat( "defined" ))
		{
			bool par = eat( "(" );
			if (!has()) return UNK;
			string n = t[i++].s;
			if (par && !eat( ")" )) return UNK;
			auto it = S.known.find( n );
			if (it == S.known.end()) return UNK;
			return { true, it->second ? 1 : 0 };
		}
		if (eat( "(" ))
		{
			EvalRes r = orExpr();
			if (!eat( ")" )) return UNK;
			return r;
		}
		if (t[i].k == T_NUM)
		{
			string n = t[i++].s;
			while (!n.empty() && (n.back() == 'u' || n.back() == 'U' || n.back() == 'l' || n.back() == 'L')) n.pop_back();
			if (n.empty()) return UNK;
			char* e = nullptr;
			long long v = strtoll( n.c_str(), &e, 0 );
			if (e && *e) return UNK;
			return { true, v };
		}
		if (t[i].k == T_ID)
		{
			string n = t[i++].s;
			auto it = S.known.find( n );
			if (it == S.known.end()) return UNK;
			if (!it->second) return { true, 0 };		// known undefined: evaluates to 0
			auto v = S.value.find( n );
			return v == S.value.end() ? UNK : EvalRes{ true, v->second };
		}
		return UNK;
	}
	EvalRes unary()
	{
		if (eat( "!" )) { EvalRes r = unary(); return r.known ? EvalRes{ true, !r.v } : UNK; }
		if (eat( "-" )) { EvalRes r = unary(); return r.known ? EvalRes{ true, -r.v } : UNK; }
		if (eat( "+" )) return unary();
		if (eat( "~" )) { EvalRes r = unary(); return r.known ? EvalRes{ true, ~r.v } : UNK; }
		return primary();
	}
	EvalRes mul()
	{
		EvalRes a = unary();
		for (; ; )
		{
			string op = peek();
			if (op != "*" && op != "/" && op != "%") return a;
			i++;
			EvalRes b = unary();
			if (!a.known || !b.known) { a = UNK; continue; }
			if (op == "*") a = { true, a.v * b.v };
			else if (b.v == 0) a = UNK;
			else a = { true, op == "/" ? a.v / b.v : a.v % b.v };
		}
	}
	EvalRes add()
	{
		EvalRes a = mul();
		for (; ; )
		{
			string op = peek();
			if (op != "+" && op != "-") return a;
			i++;
			EvalRes b = mul();
			a = (a.known && b.known) ? EvalRes{ true, op == "+" ? a.v + b.v : a.v - b.v } : UNK;
		}
	}
	EvalRes rel()
	{
		EvalRes a = add();
		for (; ; )
		{
			string op = peek();
			if (op != "<" && op != ">" && op != "<=" && op != ">=") return a;
			i++;
			EvalRes b = add();
			if (!a.known || !b.known) { a = UNK; continue; }
			a = { true, op == "<" ? a.v < b.v : op == ">" ? a.v > b.v : op == "<=" ? a.v <= b.v : a.v >= b.v };
		}
	}
	EvalRes eq()
	{
		EvalRes a = rel();
		for (; ; )
		{
			string op = peek();
			if (op != "==" && op != "!=") return a;
			i++;
			EvalRes b = rel();
			a = (a.known && b.known) ? EvalRes{ true, op == "==" ? a.v == b.v : a.v != b.v } : UNK;
		}
	}
	EvalRes andExpr()
	{
		EvalRes a = eq();
		for (; ; )
		{
			if (peek() != "&&") return a;
			i++;
			EvalRes b = eq();
			if ((a.known && !a.v) || (b.known && !b.v)) a = { true, 0 };	// one false operand decides
			else a = (a.known && b.known) ? EvalRes{ true, 1 } : UNK;
		}
	}
	EvalRes orExpr()
	{
		EvalRes a = andExpr();
		for (; ; )
		{
			if (peek() != "||") return a;
			i++;
			EvalRes b = andExpr();
			if ((a.known && a.v) || (b.known && b.v)) a = { true, 1 };	// one true operand decides
			else a = (a.known && b.known) ? EvalRes{ true, 0 } : UNK;
		}
	}
	EvalRes run() { EvalRes r = orExpr(); return i == t.size() ? r : UNK; }
};

static bool isCondOpen( const Chunk& c ) { return c.dir && (c.t[0].is( "if" ) || c.t[0].is( "ifdef" ) || c.t[0].is( "ifndef" )); }
static bool isCondMid( const Chunk& c ) { return c.dir && (c.t[0].is( "elif" ) || c.t[0].is( "else" )); }
static bool isCondEnd( const Chunk& c ) { return c.dir && c.t[0].is( "endif" ); }

static EvalRes evalCond( const Chunk& c, const SymTab& S )
{
	const string& n = c.t[0].s;
	if (n == "else") return { true, 1 };
	if (n == "ifdef" || n == "ifndef")
	{
		if (c.t.size() < 2 || !c.t[1].id()) return UNK;
		auto it = S.known.find( c.t[1].s );
		if (it == S.known.end()) return UNK;
		bool d = it->second;
		return { true, (n == "ifdef") == d ? 1 : 0 };
	}
	vector<Tok> e( c.t.begin() + 1, c.t.end() );
	CondEval ev( e, S );
	return ev.run();
}

// Fold conditionals whose every branch condition is decidable; recurse into the rest.
static size_t foldRegion( const vector<Chunk>& in, size_t i, size_t end, const SymTab& S, vector<Chunk>& out )
{
	while (i < end)
	{
		const Chunk& c = in[i];
		if (!isCondOpen( c )) { out.push_back( c ); i++; continue; }
		// map the chain: branch starts and the matching #endif
		vector<size_t> br{ i };
		size_t j = i + 1, depth = 1, endIdx = end;
		for (; j < end; j++)
		{
			const Chunk& d = in[j];
			if (isCondOpen( d )) { depth++; continue; }
			if (isCondEnd( d )) { if (--depth == 0) { endIdx = j; break; } continue; }
			if (depth == 1 && isCondMid( d )) br.push_back( j );
		}
		if (endIdx == end) fatal( "unterminated #if" );
		vector<EvalRes> val;
		bool allKnown = true;
		for (size_t b : br) { EvalRes r = evalCond( in[b], S ); val.push_back( r ); allKnown &= r.known; }
		br.push_back( endIdx );
		if (allKnown)
		{
			for (size_t k = 0; k + 1 < br.size(); k++) if (val[k].v)
			{
				foldRegion( in, br[k] + 1, br[k + 1], S, out );
				break;
			}
		}
		else
		{
			for (size_t k = 0; k + 1 < br.size(); k++)
			{
				out.push_back( in[br[k]] );
				foldRegion( in, br[k] + 1, br[k + 1], S, out );
			}
			out.push_back( in[endIdx] );
		}
		i = endIdx + 1;
	}
	return i;
}

// ============================================================================
//
//        E M I T T E R   P R I M I T I V E S
//
// ============================================================================

static bool isDigraph( char a, char b )
{
	static const char* D[] = { "++", "--", "+=", "-=", "*=", "/=", "%=", "^=", "&=", "|=", "==", "!=",
		"<=", ">=", "&&", "||", "<<", ">>", "->", "::", "##", "..", "//", "/*", "*/", "<:", ":>",
		"<%", "%>", "%:", ".*", "&&" };
	for (const char* d : D) if (d[0] == a && d[1] == b) return true;
	return false;
}

static bool needSpace( const Tok& a, const Tok& b )
{
	char x = a.s.back(), y = b.s.front();
	if (idChar( x ) && idChar( y )) return true;
	if (idChar( x ) && (y == '"' || y == '\'')) return true;			// would become a literal prefix
	if ((x == '"' || x == '\'') && idChar( y )) return true;			// would become a literal suffix
	if (isDigraph( x, y )) return true;
	if (a.s == "->" && y == '*') return true;							// ->*
	if (a.s == "<<" && y == '=') return true;
	if (a.s == ">>" && y == '=') return true;
	if (x == '.' && isdigit( (unsigned char)y )) return true;
	return false;
}

static size_t renderLen( const vector<Tok>& t, size_t from, size_t n )
{
	size_t len = 0;
	for (size_t i = 0; i < n; i++)
	{
		if (i) len += needSpace( t[from + i - 1], t[from + i] ) ? 1 : 0;
		len += t[from + i].s.size();
	}
	return len;
}

// ============================================================================
//
//        A S S E M B L Y
//
// ============================================================================

static vector<string> splitLines( const string& s )
{
	vector<string> L;
	string cur;
	for (char c : s)
	{
		if (c == '\n') { L.push_back( cur ); cur.clear(); }
		else if (c != '\r') cur += c;
	}
	if (!cur.empty()) L.push_back( cur );
	return L;
}

static string trimmed( const string& s )
{
	size_t a = s.find_first_not_of( " \t" );
	if (a == string::npos) return "";
	size_t b = s.find_last_not_of( " \t" );
	return s.substr( a, b - a + 1 );
}

static size_t findLine( const vector<string>& L, const string& what, size_t from = 0 )
{
	for (size_t i = from; i < L.size(); i++) if (trimmed( L[i] ) == what) return i;
	return string::npos;
}

// Drop the "#ifndef TINY_BVH_H_ / #error / #endif" preamble of a sub-header.
static string stripSubHeaderGuard( const string& text )
{
	vector<string> L = splitLines( text );
	size_t i = findLine( L, "#ifndef TINY_BVH_H_" );
	if (i == string::npos) return text;
	size_t j = i;
	int depth = 0;
	for (; j < L.size(); j++)
	{
		string t = trimmed( L[j] );
		if (t.rfind( "#if", 0 ) == 0) depth++;
		else if (t.rfind( "#endif", 0 ) == 0) { if (--depth == 0) break; }
	}
	string out;
	for (size_t k = 0; k < L.size(); k++) if (k < i || k > j) out += L[k] + "\n";
	return out;
}

enum Isa { ISA_NONE, ISA_SSE, ISA_AVX, ISA_AVX2, ISA_NEON };

static const char* isaName( Isa i )
{
	return i == ISA_NONE ? "scalar" : i == ISA_SSE ? "SSE" : i == ISA_AVX ? "AVX" :
		i == ISA_AVX2 ? "AVX2" : "NEON";
}

static const char* isaGuard( Isa i )
{
	return i == ISA_SSE ? "BVH_USESSE" : i == ISA_AVX ? "BVH_USEAVX" :
		i == ISA_AVX2 ? "BVH_USEAVX2" : i == ISA_NEON ? "BVH_USENEON" : "";
}

static string simdDetectBlock( Isa isa )
{
	const string x86 = "defined __x86_64__ || defined _M_X64 || defined __wasm_simd128__ || "
		"defined __wasm_relaxed_simd__";
	string s = "#ifndef TINYBVH_NO_SIMD\n";
	if (isa == ISA_NONE) return s + "#define TINYBVH_NO_SIMD\n#endif\n";
	if (isa == ISA_NEON)
	{
		s += "#if (defined __aarch64__ || defined _M_ARM64) && (defined __ARM_NEON || defined __APPLE__)\n";
		s += "#define BVH_USENEON\n#include \"arm_neon.h\"\n#else\n#define TINYBVH_NO_SIMD\n#endif\n#endif\n";
		return s;
	}
	if (isa == ISA_SSE) s += "#if (" + x86 + ") && (defined __SSE4_2__ || defined _MSC_VER)\n";
	else if (isa == ISA_AVX) s += "#if (" + x86 + ") && defined __AVX__\n";
	else s += "#if (" + x86 + ") && defined __AVX2__ && (defined __FMA__ || defined _MSC_VER)\n";
	s += "#define BVH_USESSE\n";
	if (isa >= ISA_AVX) s += "#define BVH_USEAVX\n";
	if (isa >= ISA_AVX2) s += "#define BVH_USEAVX2\n";
	s += "#include \"immintrin.h\"\n";
	// TINYBVH_NO_SIMD advertises 'no complete SIMD path'; only an AVX2 build clears it.
	if (isa == ISA_AVX2) s += "#else\n#define TINYBVH_NO_SIMD\n#endif\n#endif\n";
	else s += "#endif\n#define TINYBVH_NO_SIMD\n#endif\n";
	return s;
}

// Splice the six headers into one source text, with the SIMD detection block
// and the include block regenerated for the selected ISA.
static string assemble( const string& dir, Isa isa, string& license )
{
	auto path = [&]( const char* f ) { return dir.empty() ? string( f ) : dir + "/" + f; };
	string main = readFile( path( "tiny_bvh.h" ) );
	// the license comment is the only comment that survives
	size_t lb = main.find( "/*" ), le = main.find( "*/" );
	if (lb == string::npos || le == string::npos || lb > 8) fatal( "license block not found in tiny_bvh.h" );
	license = main.substr( lb, le + 2 - lb );
	{ string t; for (char c : license) if (c != '\r') t += c; license = t; }

	vector<string> L = splitLines( main );
	size_t simdA = findLine( L, "#ifndef TINYBVH_NO_SIMD" );
	size_t simdB = string::npos;
	for (size_t i = simdA; i < L.size(); i++) if (trimmed( L[i] ).rfind( "#endif // TINYBVH_NO_SIMD", 0 ) == 0) { simdB = i; break; }
	size_t incA = findLine( L, "#include \"tiny_bvh_base.h\"" );
	size_t incB = string::npos;
	for (size_t i = incA; i < L.size(); i++) if (trimmed( L[i] ).rfind( "#endif", 0 ) == 0) { incB = i; break; }
	if (simdA == string::npos || simdB == string::npos || incA == string::npos || incB == string::npos)
		fatal( "tiny_bvh.h does not have the expected layout (SIMD detect / include block)" );

	string out;
	for (size_t i = 0; i < simdA; i++) out += L[i] + "\n";
	out += simdDetectBlock( isa );
	for (size_t i = simdB + 1; i < incA; i++) out += L[i] + "\n";
	out += stripSubHeaderGuard( readFile( path( "tiny_bvh_base.h" ) ) );
	if (isa != ISA_NONE)
	{
		const char* f1 = isa == ISA_NEON ? "tiny_bvh_arm_float.h" : "tiny_bvh_x86_float.h";
		const char* f2 = isa == ISA_NEON ? "tiny_bvh_arm_double.h" : "tiny_bvh_x86_double.h";
		out += string( "#ifdef " ) + isaGuard( isa ) + "\n";
		out += stripSubHeaderGuard( readFile( path( f1 ) ) );
		out += stripSubHeaderGuard( readFile( path( f2 ) ) );
		out += "#endif\n";
	}
	for (size_t i = incB + 1; i < L.size(); i++) out += L[i] + "\n";
	return out;
}

// ============================================================================
//
//        L O C A L   I D E N T I F I E R   S H O R T E N I N G
//
// ============================================================================

static const char* KEYWORDS[] = { "alignas", "alignof", "and", "and_eq", "asm", "auto", "bitand",
	"bitor", "bool", "break", "case", "catch", "char", "char8_t", "char16_t", "char32_t", "class",
	"compl", "concept", "const", "consteval", "constexpr", "constinit", "const_cast", "continue",
	"co_await", "co_return", "co_yield", "decltype", "default", "delete", "do", "double",
	"dynamic_cast", "else", "enum", "explicit", "export", "extern", "false", "float", "for",
	"friend", "goto", "if", "inline", "int", "long", "mutable", "namespace", "new", "noexcept",
	"not", "not_eq", "nullptr", "operator", "or", "or_eq", "private", "protected", "public",
	"register", "reinterpret_cast", "requires", "return", "short", "signed", "sizeof", "static",
	"static_assert", "static_cast", "struct", "switch", "template", "this", "thread_local",
	"throw", "true", "try", "typedef", "typeid", "typename", "union", "unsigned", "using",
	"virtual", "void", "volatile", "wchar_t", "while", "xor", "xor_eq" };

struct Flat
{
	vector<Tok> F;					// all code tokens, in order
	vector<size_t> owner;			// code-chunk index per token
	vector<char> dead;				// token removed by macro factoring
	vector<size_t> chunkOf;			// index in C for each code chunk
};

static void flatten( const vector<Chunk>& C, Flat& f )
{
	for (size_t i = 0; i < C.size(); i++) if (!C[i].dir)
	{
		size_t cc = f.chunkOf.size();
		f.chunkOf.push_back( i );
		for (const Tok& t : C[i].t) { f.F.push_back( t ); f.owner.push_back( cc ); }
	}
	f.dead.assign( f.F.size(), 0 );
}

static void unflatten( vector<Chunk>& C, const Flat& f )
{
	for (size_t c : f.chunkOf) C[c].t.clear();
	for (size_t i = 0; i < f.F.size(); i++) if (!f.dead[i]) C[f.chunkOf[f.owner[i]]].t.push_back( f.F[i] );
}

struct Names
{
	unordered_set<string> keyword, type, ns, macroIdent, global, funcLikeMacro, allIdent, tparam;
	bool isType( const string& s ) const
	{
		if (type.count( s )) return true;
		// vendor SIMD types and the stdint family, which are never declared here
		if (s.size() > 3 && s[0] == '_' && s[1] == '_' && s[2] == 'm' && isdigit( (unsigned char)s[3] )) return true;
		size_t n = s.size();
		if (n > 4 && s.compare( n - 2, 2, "_t" ) == 0)
		{
			for (size_t i = 1; i + 2 < n; i++) if (s[i] == 'x' && isdigit( (unsigned char)s[i - 1] ) &&
				isdigit( (unsigned char)s[i + 1] )) return true;	// float32x4_t, uint8x16x2_t, ..
		}
		return false;
	}
};

static bool isSpec( const string& s )
{
	static const unordered_set<string> S = { "const", "volatile", "static", "constexpr", "mutable",
		"register", "thread_local", "inline", "typename", "struct", "class", "enum", "union",
		"explicit", "virtual", "friend" };
	return S.count( s ) != 0;
}
static bool isBuiltinType( const string& s )
{
	static const unordered_set<string> S = { "void", "bool", "char", "short", "int", "long",
		"float", "double", "signed", "unsigned", "auto", "wchar_t", "char8_t", "char16_t", "char32_t" };
	return S.count( s ) != 0;
}

// Collect type, namespace and macro names from the whole chunk list.
static void collectNames( const vector<Chunk>& C, Names& N )
{
	for (const char* k : KEYWORDS) N.keyword.insert( k );
	N.ns.insert( "std" );
	for (const char* t : { "size_t", "ptrdiff_t", "intptr_t", "uintptr_t", "FILE", "va_list",
		"int8_t", "int16_t", "int32_t", "int64_t", "uint8_t", "uint16_t", "uint32_t", "uint64_t",
		"intmax_t", "uintmax_t", "nullptr_t" } ) N.type.insert( t );
	for (const Chunk& c : C)
	{
		if (c.dir)
		{
			for (const Tok& t : c.t) if (t.id()) { N.macroIdent.insert( t.s ); N.allIdent.insert( t.s ); }
			if (c.t[0].is( "define" ) && c.t.size() > 2 && c.t[1].id() && c.t[2].is( "(" ) && !c.t[2].ws)
				N.funcLikeMacro.insert( c.t[1].s );
			// a macro body can declare types too: TEMPLATED is
			// 'template <typename Float, typename Index>', and without scanning it
			// neither Float nor Index would ever be recognized as a type name.
		}
		const vector<Tok>& T = c.t;
		for (size_t i = 0; i < T.size(); i++)
		{
			if (T[i].id()) N.allIdent.insert( T[i].s );
			if ((T[i].is( "struct" ) || T[i].is( "class" ) || T[i].is( "union" ) || T[i].is( "enum" )) &&
				i + 1 < T.size())
			{
				size_t j = i + 1;
				if (T[j].is( "class" )) j++;					// enum class
				while (j + 1 < T.size() && T[j].id() && T[j + 1].is( "(" ))
				{
					size_t d = 0, k = j + 1;					// skip ALIGNED( n ) and friends
					for (; k < T.size(); k++) { if (T[k].is( "(" )) d++; else if (T[k].is( ")" ) && --d == 0) break; }
					j = k + 1;
				}
				if (j < T.size() && T[j].id() && !N.keyword.count( T[j].s )) N.type.insert( T[j].s );
			}
			else if (T[i].is( "namespace" ) && i + 1 < T.size() && T[i + 1].id()) N.ns.insert( T[i + 1].s );
			else if (T[i].is( "using" ) && i + 2 < T.size() && T[i + 1].id() && T[i + 2].is( "=" )) N.type.insert( T[i + 1].s );
			else if (T[i].is( "using" ) && i + 4 < T.size() && T[i + 1].is( "typename" ))
			{
				size_t j = i + 2;							// using typename Base::Fragment;
				while (j + 2 < T.size() && T[j].id() && T[j + 1].is( "::" )) j += 2;
				if (j < T.size() && T[j].id()) N.type.insert( T[j].s );
			}
			else if ((T[i].is( "typename" ) || T[i].is( "class" )) && i + 1 < T.size() && T[i + 1].id())
			{
				N.type.insert( T[i + 1].s );					// template parameters
				if (i && (T[i - 1].is( "<" ) || T[i - 1].is( "," ))) N.tparam.insert( T[i + 1].s );
			}
			else if (T[i].is( "typedef" ))
			{
				size_t j = i + 1, last = string::npos, par = string::npos;
				for (; j < T.size() && !T[j].is( ";" ); j++)
				{
					if (T[j].id()) last = j;
					if (T[j].is( "*" ) && j + 1 < T.size() && T[j + 1].id() && j + 2 < T.size() && T[j + 2].is( ")" )) par = j + 1;
				}
				size_t n = par != string::npos ? par : last;
				if (n != string::npos && !N.keyword.count( T[n].s )) N.type.insert( T[n].s );
				i = j;
			}
		}
	}
}

// Skip a balanced bracket run starting at an opening bracket; returns the index
// of the matching close, or npos.
static size_t matchFwd( const vector<Tok>& F, size_t i, size_t hi )
{
	static const string O = "([{", Cl = ")]}";
	size_t k = O.find( F[i].s[0] );
	if (k == string::npos || F[i].s.size() != 1) return string::npos;
	int d = 0;
	for (size_t j = i; j < hi; j++)
	{
		if (F[j].s.size() != 1) continue;
		if (F[j].s[0] == O[k]) d++;
		else if (F[j].s[0] == Cl[k]) { if (--d == 0) return j; }
	}
	return string::npos;
}

static size_t matchBack( const vector<Tok>& F, size_t i )
{
	static const string O = "([{", Cl = ")]}";
	size_t k = Cl.find( F[i].s[0] );
	if (k == string::npos || F[i].s.size() != 1) return string::npos;
	int d = 0;
	for (size_t j = i + 1; j-- > 0; )
	{
		if (F[j].s.size() != 1) continue;
		if (F[j].s[0] == Cl[k]) d++;
		else if (F[j].s[0] == O[k]) { if (--d == 0) return j; }
		if (j == 0) break;
	}
	return string::npos;
}

// Skip a template argument list that starts at '<'. The lexer emits '>>' as one
// token, which closes two levels.
static size_t skipAngles( const vector<Tok>& F, size_t i, size_t hi )
{
	int d = 0;
	for (size_t j = i; j < hi; j++)
	{
		if (F[j].is( "<" )) d++;
		else if (F[j].is( ">" )) { if (--d == 0) return j; }
		else if (F[j].is( ">>" )) { d -= 2; if (d <= 0) return j; }
		else if (F[j].is( ";" ) || F[j].is( "{" )) return string::npos;
	}
	return string::npos;
}

// Parse one declaration. mode 0: statement, ends at ';'. mode 1: parameter,
// ends at ',' or ')'. Returns false if this is not (clearly) a declaration.
static bool parseDecl( const vector<Tok>& F, size_t i, size_t hi, const Names& N, int mode,
	vector<size_t>& names, size_t& after )
{
	size_t p = i;
	auto tok = [&]( size_t k ) -> const Tok& { return F[k]; };
	// leading specifiers, plus ALIGNED( n ) style attributes
	for (; p < hi; )
	{
		if (tok( p ).id() && isSpec( tok( p ).s ) && !isBuiltinType( tok( p ).s )) { p++; continue; }
		if (tok( p ).id() && (tok( p ).s == "ALIGNED" || tok( p ).s == "TINYBVH_ALIGNED" ||
			tok( p ).s == "alignas") && p + 1 < hi && tok( p + 1 ).is( "(" ))
		{
			size_t e = matchFwd( F, p + 1, hi );
			if (e == string::npos) return false;
			p = e + 1; continue;
		}
		break;
	}
	if (p >= hi) return false;
	// the type
	if (tok( p ).id() && isBuiltinType( tok( p ).s ))
	{
		while (p < hi && tok( p ).id() && isBuiltinType( tok( p ).s )) p++;
	}
	else
	{
		if (tok( p ).is( "::" )) p++;
		if (p >= hi || !tok( p ).id()) return false;
		if (!N.isType( tok( p ).s ) && !N.ns.count( tok( p ).s )) return false;
		p++;
		for (; p < hi; )
		{
			if (tok( p ).is( "<" ))
			{
				size_t e = skipAngles( F, p, hi );
				if (e == string::npos) return false;
				p = e + 1; continue;
			}
			if (tok( p ).is( "::" ) && p + 1 < hi && tok( p + 1 ).id()) { p += 2; continue; }
			break;
		}
	}
	// declarators
	bool any = false;
	for (; ; )
	{
		while (p < hi && (tok( p ).is( "*" ) || tok( p ).is( "&" ) || tok( p ).is( "&&" ) ||
			tok( p ).is( "const" ) || tok( p ).is( "volatile" ))) p++;
		if (p >= hi) return false;
		if (mode == 1 && (tok( p ).is( "," ) || tok( p ).is( ")" ))) { after = p; return true; } // unnamed
		if (!tok( p ).id() || N.keyword.count( tok( p ).s )) return false;
		names.push_back( p );
		any = true;
		p++;
		// array bounds
		while (p < hi && tok( p ).is( "[" ))
		{
			size_t e = matchFwd( F, p, hi );
			if (e == string::npos) return false;
			p = e + 1;
		}
		if (p >= hi) return false;
		if (tok( p ).is( "(" ) || tok( p ).is( "{" ))
		{
			size_t e = matchFwd( F, p, hi );
			if (e == string::npos) return false;
			p = e + 1;
		}
		else if (tok( p ).is( "=" ))
		{
			int d = 0;
			p++;
			for (; p < hi; p++)
			{
				const string& s = tok( p ).s;
				if (s == "(" || s == "[" || s == "{") d++;
				else if (s == ")" || s == "]" || s == "}") { if (d == 0) break; d--; }
				else if (d == 0 && (s == "," || s == ";")) break;
			}
			if (p >= hi) return false;
		}
		else if (tok( p ).is( ":" ) && mode == 0) { after = p; return any; }	// range-based for
		// in a parameter list a comma ends this parameter; in a declaration it
		// introduces another declarator sharing the same type
		if (mode == 1 && p < hi && (tok( p ).is( "," ) || tok( p ).is( ")" ))) { after = p; return any; }
		if (mode == 0 && p < hi && tok( p ).is( "," )) { p++; continue; }
		break;
	}
	if (mode == 0) { if (p < hi && tok( p ).is( ";" )) { after = p; return any; } return false; }
	if (p < hi && (tok( p ).is( "," ) || tok( p ).is( ")" ))) { after = p; return any; }
	return false;
}

struct Body { size_t paramOpen, brace, close; };

// Locate every function body. Scopes that are not function bodies (namespaces,
// class bodies, initializers) are descended into.
static void findBodies( const vector<Tok>& F, vector<Body>& out )
{
	size_t n = F.size();
	for (size_t i = 0; i < n; i++)
	{
		if (!F[i].is( "{" )) continue;
		if (i == 0) continue;
		const Tok& pv = F[i - 1];
		bool fn = pv.is( ")" ) || pv.is( "const" ) || pv.is( "noexcept" ) || pv.is( "override" ) ||
			pv.is( "final" ) || pv.is( "&" ) || pv.is( "&&" );
		if (!fn) continue;
		size_t e = matchFwd( F, i, n );
		if (e == string::npos) continue;
		// walk back past a member initializer list to the parameter list
		size_t j = i - 1, po = string::npos;
		for (; ; )
		{
			while (j > 0 && (F[j].is( "const" ) || F[j].is( "noexcept" ) || F[j].is( "override" ) ||
				F[j].is( "final" ) || F[j].is( "&" ) || F[j].is( "&&" ))) j--;
			if (!F[j].is( ")" )) break;
			size_t p = matchBack( F, j );
			if (p == string::npos || p < 2) break;
			if (F[p - 1].id() && (F[p - 2].is( "," ) || F[p - 2].is( ":" ))) { j = p - 3; continue; }
			po = p; break;
		}
		out.push_back( { po, i, e } );
		i = e;											// nested scopes belong to this body
	}
}

struct RenameStats { size_t vars = 0, occ = 0, chars = 0; };

static void renameLocals( Flat& f, Names& N, RenameStats& st )
{
	const bool debugMisses = getenv( "MICROBVH_DEBUG" ) != nullptr;
	vector<Tok>& F = f.F;
	size_t n = F.size();
	vector<Body> bodies;
	findBodies( F, bodies );
	// identifiers that live outside every function body may not be renamed
	vector<char> inBody( n, 0 );
	for (const Body& b : bodies)
	{
		size_t s = b.paramOpen == string::npos ? b.brace : b.paramOpen;
		for (size_t i = s; i <= b.close; i++) inBody[i] = 1;
	}
	unordered_set<string> global;
	for (size_t i = 0; i < n; i++)
	{
		if (inBody[i]) continue;
		if (F[i].is( "(" ))
		{
			// a parameter list of a prototype declares nothing at this scope;
			// its names are free for reuse as local names in the definitions
			size_t e = matchFwd( F, i, n );
			if (e != string::npos)
			{
				size_t j = e + 1;
				while (j < n && (F[j].is( "const" ) || F[j].is( "noexcept" ) || F[j].is( "override" ) ||
					F[j].is( "final" ) || F[j].is( "=" ) || F[j].is( "0" ) || F[j].is( "delete" ) ||
					F[j].is( "default" ))) j++;
				if (j < n && (F[j].is( ";" ) || F[j].is( "," ))) { i = e; continue; }
			}
		}
		if (F[i].id()) global.insert( F[i].s );
	}

	for (const Body& b : bodies)
	{
		size_t lo = b.paramOpen == string::npos ? b.brace : b.paramOpen, hi = b.close;
		vector<size_t> decl;
		size_t after;
		if (b.paramOpen != string::npos)
		{
			size_t pc = matchFwd( F, b.paramOpen, n );
			if (pc != string::npos) for (size_t i = b.paramOpen + 1; i < pc; )
			{
				vector<size_t> nm;
				if (parseDecl( F, i, pc + 1, N, 1, nm, after )) { for (size_t x : nm) decl.push_back( x ); i = after + 1; }
				else
				{
					if (debugMisses) fprintf( stderr, "param %s %s %s\n", F[i].s.c_str(),
						F[i + 1].s.c_str(), F[i + 2].s.c_str() );
					while (i < pc && !F[i].is( "," )) i++;
					i++;
				}
			}
		}
		for (size_t i = b.brace + 1; i < hi; i++)
		{
			bool stmtStart = F[i - 1].is( "{" ) || F[i - 1].is( "}" ) || F[i - 1].is( ";" );
			bool forInit = F[i - 1].is( "(" ) && i >= 2 && F[i - 2].is( "for" );
			bool lambdaParams = F[i].is( "(" ) && F[i - 1].is( "]" );
			if (lambdaParams)
			{
				size_t pc = matchFwd( F, i, hi );
				if (pc != string::npos) for (size_t j = i + 1; j < pc; )
				{
					vector<size_t> nm;
					if (parseDecl( F, j, pc + 1, N, 1, nm, after )) { for (size_t x : nm) decl.push_back( x ); j = after + 1; }
					else { while (j < pc && !F[j].is( "," )) j++; j++; }
				}
				continue;
			}
			if (!stmtStart && !forInit) continue;
			vector<size_t> nm;
			if (parseDecl( F, i, hi, N, 0, nm, after )) { for (size_t x : nm) decl.push_back( x ); i = after; }
			else if (stmtStart && F[i].id() && !N.keyword.count( F[i].s ) && i + 1 < hi &&
				F[i + 1].is( ":" )) decl.push_back( i );		// a goto label
			else if (debugMisses && F[i].id() && !N.keyword.count( F[i].s ))
			{
				// Set MICROBVH_DEBUG to list statements that were not recognized as
				// declarations; most are plain expressions, but a type name missing
				// from the collector shows up here as a repeated 'Type name =' miss.
				fprintf( stderr, "miss %s %s %s\n", F[i].s.c_str(), F[i + 1].s.c_str(), F[i + 2].s.c_str() );
			}
		}
		if (decl.empty()) continue;
		// A member initializer names members, not parameters: ': stride( stride )'.
		vector<char> skip( hi + 1, 0 );
		if (b.paramOpen != string::npos)
		{
			size_t pc = matchFwd( F, b.paramOpen, n );
			if (pc != string::npos) for (size_t i = pc + 1; i < b.brace; i++)
				if (F[i].id() && i + 1 < b.brace && (F[i + 1].is( "(" ) || F[i + 1].is( "{" )) &&
					i > pc + 1 && (F[i - 1].is( ":" ) || F[i - 1].is( "," ))) skip[i] = 1;
		}
		// brace depth relative to the function body, for the scope test below
		vector<uint16_t> depth( hi + 1, 0 );
		{
			int d = 0;
			for (size_t i = lo; i <= hi; i++)
			{
				if (F[i].is( "}" )) d--;
				depth[i] = (uint16_t)(i < b.brace ? 0 : d);
				if (F[i].is( "{" )) d++;
			}
		}
		// which spellings may be renamed, and which short names are free
		unordered_set<string> used;
		for (size_t i = lo; i <= hi; i++) if (F[i].id()) used.insert( F[i].s );
		map<string, size_t> count;
		map<string, size_t> firstUse;
		for (size_t i = lo; i <= hi; i++) if (F[i].id() && !skip[i] && !(i > 0 && (F[i - 1].is( "." ) ||
			F[i - 1].is( "->" ) || F[i - 1].is( "::" ))))
		{
			count[F[i].s]++;
			if (!firstUse.count( F[i].s )) firstUse[F[i].s] = i;
		}
		unordered_set<string> declaredHere;		// spellings whose declaration opens their scope
		for (size_t d : decl) if (firstUse.count( F[d].s ) && firstUse[F[d].s] == d && depth[d] <= 1)
			declaredHere.insert( F[d].s );
		vector<pair<size_t, string>> cand;		// (bytes saved, name)
		unordered_set<string> seen;
		for (size_t d : decl)
		{
			const string& s = F[d].s;
			if (s.size() < 2 || seen.count( s )) continue;
			if (N.keyword.count( s ) || N.macroIdent.count( s )) continue;
			// A name that also exists at namespace or class scope is safe to rename
			// only when its declaration here is its first appearance and stays in
			// scope for the rest of the function; nothing after it can mean the
			// outer entity, and nothing before it mentions the name at all.
			if (global.count( s ) && !declaredHere.count( s )) continue;
			seen.insert( s );
			cand.push_back( { (s.size() - 1) * count[s], s } );
		}
		sort( cand.begin(), cand.end(), []( const pair<size_t, string>& a, const pair<size_t, string>& b )
			{ return a.first != b.first ? a.first > b.first : a.second < b.second; } );
		// free short names, in increasing length
		vector<string> pool;
		const char* A = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";
		for (const char* p = A; *p; p++) pool.push_back( string( 1, *p ) );
		for (const char* p = A; *p; p++) for (const char* q = "0123456789"; *q; q++)
			pool.push_back( string( 1, *p ) + *q );
		for (const char* p = A; *p; p++) for (const char* q = A; *q; q++)
			pool.push_back( string( 1, *p ) + *q );
		size_t next = 0;
		unordered_map<string, string> map2;
		for (auto& c : cand)
		{
			while (next < pool.size() && (used.count( pool[next] ) || N.macroIdent.count( pool[next] ) ||
				N.keyword.count( pool[next] ))) next++;
			if (next >= pool.size()) break;
			if (pool[next].size() >= c.second.size()) continue;
			map2[c.second] = pool[next];
			used.insert( pool[next] );
			N.allIdent.insert( pool[next] );	// keep generated macro names off this spelling
			next++;
		}
		if (map2.empty()) continue;
		for (size_t i = lo; i <= hi; i++)
		{
			if (!F[i].id()) continue;
			if (i > 0 && (F[i - 1].is( "." ) || F[i - 1].is( "->" ) || F[i - 1].is( "::" ))) continue;
			if (skip[i]) continue;
			auto it = map2.find( F[i].s );
			if (it == map2.end()) continue;
			st.occ++;
			st.chars += F[i].s.size() - it->second.size();
			F[i].s = it->second;
		}
		st.vars += map2.size();
	}
}

// ============================================================================
//
//        I N T E R N A L   N A M E   S H O R T E N I N G
//
// ============================================================================

// Names that a class exposes stay as they are; names a class only uses
// internally (private / protected members, including the ones pulled in with
// 'using Base::x') are free to shrink. A name is only shortened when every
// declaration of it in the whole header is non-public.

struct ClassBody { size_t open, close; bool isClass; };

static void findClassBodies( const vector<Tok>& F, vector<ClassBody>& out )
{
	size_t n = F.size();
	for (size_t i = 0; i < n; i++)
	{
		if (!F[i].is( "class" ) && !F[i].is( "struct" ) && !F[i].is( "union" )) continue;
		if (i && (F[i - 1].is( "enum" ) || F[i - 1].is( "friend" ))) continue;
		size_t j = i + 1, body = string::npos;
		for (; j < n; )
		{
			if (F[j].is( "(" ) || F[j].is( "[" ))
			{
				size_t e = matchFwd( F, j, n );
				if (e == string::npos) break;
				j = e + 1; continue;
			}
			if (F[j].is( "<" ))
			{
				size_t e = skipAngles( F, j, n );
				if (e == string::npos) break;
				j = e + 1; continue;
			}
			// '>' ends a 'template <class T>' parameter, which is not a class definition
			if (F[j].is( ";" ) || F[j].is( ")" ) || F[j].is( "}" ) || F[j].is( ">" ) ||
				F[j].is( "," ) || F[j].is( "=" ) || F[j].is( "*" ) || F[j].is( "&" )) break;
			if (F[j].is( "{" )) { body = j; break; }
			j++;
		}
		if (body == string::npos) continue;
		size_t e = matchFwd( F, body, n );
		if (e == string::npos) continue;
		out.push_back( { body, e, F[i].is( "class" ) } );
	}
}

// Pull the declared name(s) out of one member declaration.
static void memberNames( const vector<Tok>& F, size_t a, size_t b, const Names& N,
	const string& ownName, vector<size_t>& names )
{
	if (a >= b) return;
	if (F[a].is( "template" ) || F[a].is( "friend" ) || F[a].is( "typedef" )) return;
	for (size_t i = a; i < b; i++) if (F[i].is( "friend" ) || F[i].is( "operator" )) return;
	if (F[a].is( "using" ))
	{
		size_t last = string::npos;
		for (size_t i = a + 1; i < b; i++) { if (F[i].is( "=" )) return; if (F[i].id()) last = i; }
		if (last != string::npos) names.push_back( last );
		return;
	}
	// a function: the identifier right before the first parameter list
	int d = 0;
	for (size_t i = a; i < b; i++)
	{
		if (F[i].is( "[" ) || F[i].is( "{" )) { size_t e = matchFwd( F, i, b + 1 ); if (e == string::npos) break; i = e; continue; }
		if (F[i].is( "<" )) { size_t e = skipAngles( F, i, b + 1 ); if (e == string::npos) break; i = e; continue; }
		if (!F[i].is( "(" )) continue;
		if (i > a && F[i - 1].id() && !N.keyword.count( F[i - 1].s ) && F[i - 1].s != ownName &&
			!N.macroIdent.count( F[i - 1].s )) names.push_back( i - 1 );
		return;
		(void)d;
	}
	// otherwise a data member
	size_t after;
	vector<size_t> nm;
	if (parseDecl( F, a, b + 1, N, 0, nm, after )) for (size_t x : nm) names.push_back( x );
}

// Walk a class body, tracking access specifiers, and record each member name.
static void scanClassBody( const vector<Tok>& F, const ClassBody& cb, const Names& N,
	unordered_set<string>& pub, unordered_set<string>& priv )
{
	bool isPublic = !cb.isClass;
	string ownName;
	{	// the class name sits just before the body, past any base clause
		size_t j = cb.open;
		while (j > 0 && !F[j - 1].is( "class" ) && !F[j - 1].is( "struct" ) && !F[j - 1].is( "union" ))
		{
			if (F[j - 1].id()) { ownName = F[j - 1].s; break; }
			j--;
		}
	}
	size_t stmt = cb.open + 1;
	for (size_t i = cb.open + 1; i < cb.close; i++)
	{
		if (F[i].is( "(" ) || F[i].is( "[" ))
		{
			size_t e = matchFwd( F, i, cb.close );
			if (e == string::npos) break;
			i = e; continue;
		}
		if (F[i].is( "{" ))
		{
			size_t e = matchFwd( F, i, cb.close );
			if (e == string::npos) break;
			if (e + 1 >= cb.close || !F[e + 1].is( ";" ))	// an inline body ends the declaration
			{
				vector<size_t> nm;
				memberNames( F, stmt, i, N, ownName, nm );
				for (size_t x : nm) (isPublic ? pub : priv).insert( F[x].s );
				stmt = e + 1;
			}
			i = e; continue;
		}
		if ((F[i].is( "public" ) || F[i].is( "private" ) || F[i].is( "protected" )) &&
			i + 1 < cb.close && F[i + 1].is( ":" ))
		{
			isPublic = F[i].is( "public" );
			i++; stmt = i + 1; continue;
		}
		if (!F[i].is( ";" )) continue;
		vector<size_t> nm;
		memberNames( F, stmt, i, N, ownName, nm );
		for (size_t x : nm) (isPublic ? pub : priv).insert( F[x].s );
		stmt = i + 1;
	}
}

static void renameMembers( Flat& f, Names& N, size_t& count, size_t& saved )
{
	vector<Tok>& F = f.F;
	size_t n = F.size();
	vector<ClassBody> classes;
	findClassBodies( F, classes );
	vector<char> inClass( n, 0 );
	for (const ClassBody& c : classes) for (size_t i = c.open; i <= c.close; i++) inClass[i] = 1;
	vector<Body> bodies;
	findBodies( F, bodies );
	vector<char> inBody( n, 0 );
	for (const Body& b : bodies)
	{
		size_t s = b.paramOpen == string::npos ? b.brace : b.paramOpen;
		for (size_t i = s; i <= b.close; i++) inBody[i] = 1;
	}
	unordered_set<string> pub, priv;
	for (const ClassBody& c : classes) scanClassBody( F, c, N, pub, priv );
	// anything named at namespace scope is reachable from application code
	for (size_t i = 0; i < n; i++)
	{
		if (inClass[i] || inBody[i]) continue;
		if (F[i].is( "(" ))
		{
			size_t e = matchFwd( F, i, n );
			if (e != string::npos)
			{
				size_t j = e + 1;
				while (j < n && (F[j].is( "const" ) || F[j].is( "noexcept" ) || F[j].is( "override" ) ||
					F[j].is( "final" ) || F[j].is( "=" ) || F[j].is( "0" ))) j++;
				if (j < n && (F[j].is( ";" ) || F[j].is( "," ))) { i = e; continue; }
			}
		}
		if (F[i].id() && !(i && F[i - 1].is( "::" ))) pub.insert( F[i].s );
	}
	// member names that std or the CRT might also use are left alone
	static const char* COMMON[] = { "size", "data", "begin", "end", "first", "second", "count",
		"clear", "reset", "swap", "empty", "value", "get", "set", "next", "prev", "free", "malloc",
		"now", "load", "store", "lock", "unlock", "wait", "join", "push", "pop", "back", "front",
		"notify_one", "notify_all", "exchange", "max_size", "resize", "at", "find", "insert" };
	for (const char* c : COMMON) pub.insert( c );
	// anything qualified with std:: belongs to the standard library, not to us
	for (size_t i = 2; i < n; i++) if (F[i].id() && F[i - 1].is( "::" ))
	{
		size_t j = i - 1;
		while (j >= 2 && F[j].is( "::" ) && F[j - 1].id() && F[j - 2].is( "::" )) j -= 2;
		if (j >= 1 && F[j - 1].id() && (F[j - 1].s == "std" || !N.allIdent.count( F[j - 1].s )))
			pub.insert( F[i].s );
	}

	map<string, size_t> use;
	for (size_t i = 0; i < n; i++) if (F[i].id()) use[F[i].s]++;
	vector<pair<size_t, string>> cand;
	const bool dbg = getenv( "MICROBVH_DEBUG" ) != nullptr;
	for (const string& s : priv)
	{
		if (s.size() < 3 || pub.count( s ) || N.macroIdent.count( s ) || N.keyword.count( s ) ||
			N.isType( s ) || N.ns.count( s ))
		{
			if (dbg && s.size() >= 3) fprintf( stderr, "keep %s (%s) x%zu\n", s.c_str(),
				pub.count( s ) ? "public" : N.macroIdent.count( s ) ? "macro" : "type", use[s] );
			continue;
		}
		cand.push_back( { (s.size() - 2) * use[s], s } );
	}
	sort( cand.begin(), cand.end(), []( const pair<size_t, string>& a, const pair<size_t, string>& b )
		{ return a.first != b.first ? a.first > b.first : a.second < b.second; } );
	// the most used names get the shortest replacements
	vector<string> pool;
	const char* A = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";
	for (const char* p = A; *p; p++) for (const char* q = "0123456789"; *q; q++) pool.push_back( string( 1, *p ) + *q );
	for (const char* p = A; *p; p++) for (const char* q = A; *q; q++) pool.push_back( string( 1, *p ) + *q );
	size_t next = 0;
	unordered_map<string, string> ren;
	for (auto& c : cand)
	{
		while (next < pool.size() && N.allIdent.count( pool[next] )) next++;
		if (next >= pool.size()) break;
		ren[c.second] = pool[next];
		N.allIdent.insert( pool[next] );
		next++;
	}
	if (ren.empty()) return;
	for (size_t i = 0; i < n; i++)
	{
		if (!F[i].id()) continue;
		auto it = ren.find( F[i].s );
		if (it == ren.end()) continue;
		saved += F[i].s.size() - it->second.size();
		F[i].s = it->second;
	}
	count = ren.size();
}

// Parameter names in a pure declaration are documentation; the language does
// not need them. 'void Build( const Vertex* v, Index n );' is identical to
// 'void Build( const Vertex*, Index );' to every caller.
static size_t stripPrototypeNames( Flat& f, const Names& N )
{
	vector<Tok>& F = f.F;
	size_t n = F.size(), removed = 0;
	vector<Body> bodies;
	findBodies( F, bodies );
	vector<char> inBody( n, 0 );
	for (const Body& b : bodies)
	{
		size_t s = b.paramOpen == string::npos ? b.brace : b.paramOpen;
		for (size_t i = s; i <= b.close; i++) inBody[i] = 1;
	}
	for (size_t i = 0; i < n; i++)
	{
		if (inBody[i] || !F[i].is( "(" )) continue;
		size_t e = matchFwd( F, i, n );
		if (e == string::npos) continue;
		size_t j = e + 1;
		while (j < n && (F[j].is( "const" ) || F[j].is( "noexcept" ) || F[j].is( "override" ) ||
			F[j].is( "final" ) || F[j].is( "=" ) || F[j].is( "0" ))) j++;
		if (j >= n || !F[j].is( ";" )) { i = e; continue; }		// not a plain declaration
		if (i == 0 || !F[i - 1].id() || N.keyword.count( F[i - 1].s )) { i = e; continue; }
		for (size_t k = i + 1; k < e; )
		{
			vector<size_t> nm;
			size_t after;
			if (parseDecl( F, k, e + 1, N, 1, nm, after ))
			{
				for (size_t x : nm) if (!F[x + 1].is( "[" )) { f.dead[x] = 1; removed++; }
				k = after + 1;
			}
			else { while (k < e && !F[k].is( "," )) k++; k++; }
		}
		i = e;
	}
	return removed;
}

// ============================================================================
//
//        S T A T E M E N T   S I M P L I F I C A T I O N
//
// ============================================================================

// Statement kinds, as far as this tool needs to tell them apart.
enum StKind { ST_EXPR, ST_DECL, ST_RETVAL, ST_OTHER, ST_LABEL, ST_MACRO, ST_BLOCK, ST_BAD };

static bool isScalarType( const string& s )
{
	static const unordered_set<string> S = { "bool", "char", "short", "int", "long", "float",
		"double", "signed", "unsigned", "int8_t", "int16_t", "int32_t", "int64_t", "uint8_t",
		"uint16_t", "uint32_t", "uint64_t", "size_t" };
	return S.count( s ) != 0;
}

struct Simplifier
{
	vector<Tok>& F;
	vector<char>& dead;
	const Names& N;
	size_t n;
	size_t blocks = 0, folds = 0, loops = 0, tried = 0;
	map<string, size_t> why;
	void note( const char* w ) { why[w]++; }
	unordered_set<size_t> doWhile;		// 'while' tokens that belong to a do-statement
	Simplifier( Flat& f, const Names& nn ) : F( f.F ), dead( f.dead ), N( nn ), n( f.F.size() ) {}

	size_t skipParens( size_t i, size_t hi ) const
	{
		size_t e = matchFwd( F, i, hi );
		return e == string::npos ? string::npos : e + 1;
	}
	size_t toSemi( size_t i, size_t hi ) const
	{
		int d = 0;
		for (; i < hi; i++)
		{
			const string& s = F[i].s;
			if (s == "(" || s == "[" || s == "{") d++;
			else if (s == ")" || s == "]" || s == "}") { if (d == 0) return string::npos; d--; }
			else if (s == ";" && d == 0) return i + 1;
		}
		return string::npos;
	}
	// End of the statement starting at i, with its kind.
	size_t stmt( size_t i, size_t hi, StKind& k ) const
	{
		if (i >= hi) { k = ST_BAD; return string::npos; }
		const string& s = F[i].s;
		if (s == "{") { k = ST_BLOCK; return skipParens( i, hi ); }
		if (s == ";") { k = ST_EXPR; return i + 1; }
		if (s == "if")
		{
			size_t j = i + 1;
			if (j < hi && F[j].is( "constexpr" )) j++;
			if (j >= hi || !F[j].is( "(" )) { k = ST_BAD; return string::npos; }
			j = skipParens( j, hi );
			if (j == string::npos) { k = ST_BAD; return string::npos; }
			StKind sub;
			j = stmt( j, hi, sub );
			if (j == string::npos) { k = ST_BAD; return string::npos; }
			if (j < hi && F[j].is( "else" )) j = stmt( j + 1, hi, sub );
			k = ST_OTHER;
			return j;
		}
		if (s == "for" || s == "while" || s == "switch")
		{
			size_t j = i + 1;
			if (j >= hi || !F[j].is( "(" )) { k = ST_BAD; return string::npos; }
			j = skipParens( j, hi );
			if (j == string::npos) { k = ST_BAD; return string::npos; }
			StKind sub;
			j = stmt( j, hi, sub );
			k = ST_OTHER;
			return j;
		}
		if (s == "do")
		{
			StKind sub;
			size_t j = stmt( i + 1, hi, sub );
			if (j == string::npos || j >= hi || !F[j].is( "while" )) { k = ST_BAD; return string::npos; }
			const_cast<Simplifier*>( this )->doWhile.insert( j );
			j = skipParens( j + 1, hi );
			if (j == string::npos || j >= hi || !F[j].is( ";" )) { k = ST_BAD; return string::npos; }
			k = ST_OTHER;
			return j + 1;
		}
		if (s == "case" || s == "default")
		{
			for (size_t j = i; j < hi; j++) if (F[j].is( ":" )) { k = ST_LABEL; return j + 1; }
			k = ST_BAD;
			return string::npos;
		}
		if (F[i].id() && i + 1 < hi && F[i + 1].is( ":" )) { k = ST_LABEL; return i + 2; }
		if (s == "return")
		{
			size_t e = toSemi( i, hi );
			k = (i + 1 < hi && F[i + 1].is( ";" )) ? ST_OTHER : ST_RETVAL;
			return e;
		}
		if (s == "break" || s == "continue" || s == "goto" || s == "throw")
		{
			k = ST_OTHER;
			return toSemi( i, hi );
		}
		size_t after;
		vector<size_t> nm;
		if (parseDecl( F, i, hi, N, 0, nm, after )) { k = ST_DECL; return after + 1; }
		size_t e = toSemi( i, hi );
		if (e == string::npos) { k = ST_BAD; return string::npos; }
		k = N.funcLikeMacro.count( s ) ? ST_MACRO : ST_EXPR;
		return e;
	}

	// 'while (1)' and 'while (true)' are shorter written as 'for (;;)'.
	void shortenLoop( size_t i )
	{
		if (!F[i].is( "while" ) || i + 3 >= n || doWhile.count( i )) return;
		if (!F[i + 1].is( "(" ) || !F[i + 3].is( ")" )) return;
		if (!F[i + 2].is( "1" ) && !F[i + 2].is( "true" )) return;
		F[i].s = "for";
		F[i + 2].s = ";;";
		F[i + 2].k = T_PUNCT;
		loops++;
	}

	// True when a trailing 'else' would bind to an 'if' inside this statement.
	bool absorbsElse( size_t i, size_t hi ) const
	{
		if (i >= hi) return false;
		if (F[i].is( "{" )) return false;							// its own braces catch the else
		if (F[i].is( "if" ))
		{
			size_t j = i + 1;
			if (j < hi && F[j].is( "constexpr" )) j++;
			j = skipParens( j, hi );
			if (j == string::npos) return true;
			StKind k;
			size_t e = stmt( j, hi, k );
			if (e == string::npos) return true;
			if (e < hi && F[e].is( "else" )) return absorbsElse( e + 1, hi );
			return true;										// an if without else
		}
		if (F[i].is( "for" ) || F[i].is( "while" ))
		{
			size_t j = skipParens( i + 1, hi );
			return j == string::npos ? true : absorbsElse( j, hi );
		}
		if (F[i].is( "case" ) || F[i].is( "default" ) || (F[i].id() && i + 1 < hi && F[i + 1].is( ":" )))
		{
			for (size_t j = i; j < hi; j++) if (F[j].is( ":" )) return absorbsElse( j + 1, hi );
		}
		return false;
	}

	// Drop the braces of a control statement body where the language allows it.
	void tryBlock( size_t b, size_t hi )
	{
		size_t e = matchFwd( F, b, hi );
		if (e == string::npos) return;
		tried++;
		vector<pair<size_t, StKind>> st;		// (start, kind)
		size_t i = b + 1;
		while (i < e)
		{
			StKind k;
			size_t j = stmt( i, e, k );
			if (j == string::npos || k == ST_BAD || j > e) { note( "parse" ); return; }
			st.push_back( { i, k } );
			i = j;
		}
		if (i != e || st.empty()) { note( "span" ); return; }
		// a trailing 'else' may not be allowed to bind to an inner 'if'
		if (e + 1 < n && F[e + 1].is( "else" ) && absorbsElse( st.back().first, e ))
			{ note( "else" ); return; }
		if (st.size() == 1)
		{
			StKind k = st[0].second;
			// an inner if/else reads ambiguously once the braces are gone
			for (size_t j = b + 1; j < e; j++) if (F[j].is( "else" )) { note( "inner-else" ); return; }
			if (k != ST_EXPR && k != ST_RETVAL && k != ST_OTHER) { note( k == ST_DECL ? "1decl" : k == ST_MACRO ? "1macro" : "1other" ); return; }
			dead[b] = dead[e] = 1;
			blocks++;
			return;
		}
		// several statements: the comma operator turns them into one
		for (auto& p : st) if (p.second != ST_EXPR) { note( p.second == ST_DECL ? "ndecl" : p.second == ST_MACRO ? "nmacro" : p.second == ST_RETVAL ? "nret" : p.second == ST_OTHER ? "nother" : "nblock" ); return; }
		for (size_t s2 = 0; s2 + 1 < st.size(); s2++)
		{
			size_t term = st[s2 + 1].first - 1;	// the ';' that ends this statement
			if (!F[term].is( ";" )) return;
		}
		for (size_t s2 = 0; s2 + 1 < st.size(); s2++) F[st[s2 + 1].first - 1].s = ",";
		dead[b] = dead[e] = 1;
		blocks++;
		folds += st.size() - 1;
	}

	void run()
	{
		vector<Body> bodies;
		findBodies( F, bodies );
		// find the 'while' of every do-statement first; those may not become 'for'
		for (const Body& bd : bodies) for (size_t i = bd.brace + 1; i < bd.close; i++) if (F[i].is( "do" ))
		{
			StKind k;
			stmt( i, bd.close + 1, k );
		}
		for (const Body& bd : bodies) for (size_t i = bd.brace + 1; i < bd.close; i++)
		{
			shortenLoop( i );
			if (!F[i].is( "{" ) || i == 0) continue;
			const Tok& pv = F[i - 1];
			bool ok = pv.is( "else" ) || pv.is( "do" );
			if (!ok && pv.is( ")" ))
			{
				size_t o = matchBack( F, i - 1 );
				if (o != string::npos && o > 0 && (F[o - 1].is( "if" ) || F[o - 1].is( "while" ) ||
					F[o - 1].is( "for" ) || (F[o - 1].is( "constexpr" ) && o > 1 && F[o - 2].is( "if" )))) ok = true;
			}
			if (ok) tryBlock( i, bd.close + 1 );
		}
	}
};

// Top-level 'const' on a by-value scalar parameter or local changes nothing the
// compiler reports; it is dropped unless the name is needed as a constant.
static size_t stripConst( Flat& f, const Names& N )
{
	vector<Tok>& F = f.F;
	size_t dropped = 0;
	vector<Body> bodies;
	findBodies( F, bodies );
	for (const Body& b : bodies)
	{
		size_t lo = b.paramOpen == string::npos ? b.brace : b.paramOpen, hi = b.close;
		// names that must stay constant expressions
		unordered_set<string> pinned;
		for (size_t i = lo; i <= hi; i++) if (F[i].id())
		{
			bool before = i > lo && (F[i - 1].is( "[" ) || F[i - 1].is( "<" ) || F[i - 1].is( "case" ));
			bool after = i + 1 <= hi && (F[i + 1].is( ">" ) || F[i + 1].is( ">>" ));
			if (before || after) pinned.insert( F[i].s );
		}
		for (size_t i = lo; i + 3 <= hi; i++)
		{
			if (!F[i].is( "const" ) || i == lo) continue;
			const Tok& pv = F[i - 1];
			if (!pv.is( "(" ) && !pv.is( "," ) && !pv.is( "{" ) && !pv.is( "}" ) && !pv.is( ";" )) continue;
			size_t j = i + 1;
			while (j <= hi && F[j].id() && (isScalarType( F[j].s ) || N.tparam.count( F[j].s ))) j++;
			if (j == i + 1 || j > hi) continue;					// no scalar type after the const
			if (!F[j].id() || N.keyword.count( F[j].s )) continue;
			if (j + 1 > hi) continue;
			const Tok& nx = F[j + 1];
			if (!nx.is( "=" ) && !nx.is( ";" ) && !nx.is( "," ) && !nx.is( ")" )) continue;
			if (pinned.count( F[j].s )) continue;
			f.dead[i] = 1;
			dropped++;
		}
	}
	return dropped;
}

// Adjacent declarations that share a type spell that type once.
// 'uint32_t a = 1; uint32_t b = 2;' becomes 'uint32_t a = 1, b = 2;'.
static size_t mergeDecls( Flat& f, const Names& N )
{
	vector<Tok>& F = f.F;
	size_t merged = 0;
	vector<Body> bodies;
	findBodies( F, bodies );
	auto typeEnd = [&]( size_t a, size_t name )
	{
		size_t j = a;
		while (j < name && !F[j].is( "*" ) && !F[j].is( "&" ) && !F[j].is( "&&" )) j++;
		return j;
	};
	for (const Body& b : bodies)
	{
		size_t hi = b.close;
		for (size_t i = b.brace + 1; i < hi; i++)
		{
			if (!F[i - 1].is( "{" ) && !F[i - 1].is( "}" ) && !F[i - 1].is( ";" )) continue;
			vector<size_t> nm;
			size_t after;
			if (!parseDecl( F, i, hi, N, 0, nm, after ) || nm.empty()) continue;
			if (!F[after].is( ";" )) continue;
			size_t aEnd = typeEnd( i, nm[0] ), semi = after;
			for (; ; )
			{
				size_t j = semi + 1;							// the statement right after
				if (j >= hi || f.owner[j] != f.owner[semi]) break;	// a directive sits between them
				vector<size_t> nm2;
				size_t after2;
				if (!parseDecl( F, j, hi, N, 0, nm2, after2 ) || nm2.empty()) break;
				if (!F[after2].is( ";" )) break;
				size_t bEnd = typeEnd( j, nm2[0] );
				if (bEnd - j != aEnd - i) break;
				bool same = true;
				for (size_t k = 0; k < aEnd - i; k++) if (F[i + k].s != F[j + k].s) { same = false; break; }
				if (!same) break;
				F[semi].s = ",";								// join the two declarations
				for (size_t k = j; k < bEnd; k++) f.dead[k] = 1;
				merged++;
				semi = after2;
			}
			i = semi;
		}
	}
	return merged;
}

// Shorter spellings for literals. Every rewrite keeps the value and the type.
//  - '0.5f' is '.5f', '1.0f' is '1.f'
//  - a hex constant is respelled in whichever base is shorter
//  - an all-ones mask is '~0u', a power of two is '1u<<k'
//  - 'true' and 'false' are 1 and 0
// The multi-token forms are only used where the neighbouring operators bind
// more loosely than a shift, so no parentheses are needed and no expression
// changes meaning.
struct LitStats { size_t floats = 0, ints = 0, bools = 0, bytes = 0; };

static bool shiftSafeLeft( const Tok& t )
{
	static const unordered_set<string> S = { "(", ",", "=", "return", ";", "{", "[", "&&", "||",
		"|", "&", "^", "?", ":", "+=", "-=", "|=", "&=", "^=" };
	return S.count( t.s ) != 0;
}
static bool shiftSafeRight( const Tok& t )
{
	static const unordered_set<string> S = { ")", ",", ";", "]", "}", "&&", "||", "|", "&", "^", ":" };
	return S.count( t.s ) != 0;
}

// Parse an integer literal into a value plus a type class. Returns false for
// anything whose type is not obvious: octal, digit separators, odd suffixes.
static bool intLiteral( const string& s, unsigned long long& v, bool& isU, bool& is64 )
{
	size_t i = 0, n = s.size();
	if (n == 0 || !isdigit( (unsigned char)s[0] )) return false;
	int base = 10;
	if (n > 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) { base = 16; i = 2; }
	else if (s[0] == '0' && n > 1) return false;					// octal: leave alone
	size_t digits = i;
	v = 0;
	for (; i < n; i++)
	{
		int d;
		char c = s[i];
		if (isdigit( (unsigned char)c )) d = c - '0';
		else if (base == 16 && isxdigit( (unsigned char)c )) d = (tolower( c ) - 'a') + 10;
		else break;
		unsigned long long nv = v * (unsigned)base + (unsigned)d;
		if (nv < v) return false;									// overflow
		v = nv;
	}
	if (i == digits) return false;
	string suf = s.substr( i );
	for (char& c : suf) c = (char)tolower( (unsigned char)c );
	if (suf != "" && suf != "u" && suf != "ull" && suf != "ul" && suf != "ll" && suf != "llu" &&
		suf != "l" && suf != "lu") return false;
	isU = suf.find( 'u' ) != string::npos;
	is64 = suf.find( "ll" ) != string::npos || v > 0xFFFFFFFFull;
	if (!isU && base == 16 && !is64 && v > 0x7FFFFFFFull) isU = true;	// hex picks unsigned int
	else if (!isU && v > 0x7FFFFFFFull && base != 16) return false;		// decimal would widen
	if (suf.find( 'l' ) != string::npos && suf.find( "ll" ) == string::npos) return false;
	return true;
}

static void shortenConstants( Flat& f, LitStats& st, bool doBool )
{
	vector<Tok>& F = f.F;
	for (size_t k = 0; k < F.size(); k++)
	{
		Tok& t = F[k];
		if (doBool && t.k == T_ID && (t.s == "true" || t.s == "false"))
		{
			st.bytes += t.s.size() - 1, st.bools++;
			t.s = t.s == "true" ? "1" : "0";
			t.k = T_NUM;
			continue;
		}
		if (t.k != T_NUM) continue;
		if (t.s.find( '.' ) != string::npos)
		{
			size_t suf = t.s.size();
			while (suf && (t.s[suf - 1] == 'f' || t.s[suf - 1] == 'F')) suf--;
			string body = t.s.substr( 0, suf ), tail = t.s.substr( suf );
			if (body.find_first_not_of( "0123456789." ) != string::npos) continue;
			size_t dot = body.find( '.' );
			if (dot == string::npos || body.find( '.', dot + 1 ) != string::npos) continue;
			string lead = body.substr( 0, dot ), frac = body.substr( dot + 1 );
			while (!frac.empty() && frac.back() == '0') frac.pop_back();
			if (lead == "0" && !frac.empty()) lead.clear();
			if (lead.empty() && frac.empty()) lead = "0";
			string out = lead + "." + frac + tail;
			if (out.size() < t.s.size()) { st.bytes += t.s.size() - out.size(), st.floats++; t.s = out; }
			continue;
		}
		unsigned long long v;
		bool isU, is64;
		if (!intLiteral( t.s, v, isU, is64 )) continue;
		string suf = string( isU ? "u" : "" ) + (is64 ? "ll" : "");
		if (is64 && isU) suf = "ull";
		vector<string> forms;
		{
			char dec[32], hex[32];
			snprintf( dec, sizeof( dec ), "%llu", v );
			snprintf( hex, sizeof( hex ), "0x%llX", v );
			forms.push_back( string( dec ) + suf );
			forms.push_back( string( hex ) + suf );
		}
		bool multi = false;
		unsigned long long allOnes = is64 ? ~0ull : 0xFFFFFFFFull;
		if (isU && v == allOnes) forms.push_back( "~0" + suf );			// unary: always safe
		size_t best = 0;
		for (size_t i = 1; i < forms.size(); i++) if (forms[i].size() < forms[best].size()) best = i;
		// a power of two needs a shift, which only fits without parentheses in a
		// context that binds more loosely than '<<'
		if (isU && v >= 16 && (v & (v - 1)) == 0 && k && k + 1 < F.size() &&
			shiftSafeLeft( F[k - 1] ) && shiftSafeRight( F[k + 1] ))
		{
			int sh = 0;
			for (unsigned long long w = v; w > 1; w >>= 1) sh++;
			char buf[32];
			snprintf( buf, sizeof( buf ), "1%s<<%d", suf.c_str(), sh );
			if (strlen( buf ) < forms[best].size()) { forms.push_back( buf ); best = forms.size() - 1; multi = true; }
		}
		(void)multi;
		if (forms[best].size() < t.s.size())
		{
			st.bytes += t.s.size() - forms[best].size(), st.ints++;
			t.s = forms[best];
		}
	}
}

// ============================================================================
//
//        M A C R O   F A C T O R I N G
//
// ============================================================================

static uint64_t hashSeq( const vector<Tok>& F, size_t i, size_t n )
{
	uint64_t h = 1469598103934665603ull;
	for (size_t k = 0; k < n; k++)
	{
		for (char c : F[i + k].s) { h ^= (unsigned char)c; h *= 1099511628211ull; }
		h ^= 1u; h *= 1099511628211ull;
	}
	return h;
}

static bool seqUsable( const vector<Tok>& F, size_t i, size_t n, const Names& N, bool* argUnsafe = nullptr )
{
	if (F[i].is( "(" )) return false;							// would hide a function-like macro call
	if (n > 1 && N.funcLikeMacro.count( F[i + n - 1].s )) return false;
	int d1 = 0, d2 = 0, d3 = 0;
	bool unsafe = false;
	for (size_t k = 0; k < n; k++)
	{
		const string& s = F[i + k].s;
		if (s == "#" || s == "##") return false;
		if (s == "(") d1++; else if (s == ")") d1--;
		else if (s == "[") d2++; else if (s == "]") d2--;
		else if (s == "{") d3++; else if (s == "}") d3--;
		else if (s == "," && d1 == 0 && d2 == 0 && d3 == 0) unsafe = true;
		// a run that closes a bracket it never opened is not self-contained,
		// even when the totals happen to balance again: ');if(' is not a unit
		if (d1 < 0 || d2 < 0 || d3 < 0) return false;
	}
	// A run must close what it opens. Leaving a bracket open is legal and was
	// measured: it is worth under a hundred bytes across the targets, which does
	// not justify moving brackets into macro bodies.
	if (d1 || d2 || d3) return false;
	if (argUnsafe) *argUnsafe = unsafe;
	return true;
}

static bool seqBalanced( const vector<Tok>& F, size_t i, size_t n )
{
	int d1 = 0, d2 = 0, d3 = 0;
	for (size_t k = 0; k < n; k++)
	{
		const string& s = F[i + k].s;
		if (s == "(") d1++; else if (s == ")") d1--;
		else if (s == "[") d2++; else if (s == "]") d2--;
		else if (s == "{") d3++; else if (s == "}") d3--;
		if (d1 < 0 || d2 < 0 || d3 < 0) return false;
	}
	return !d1 && !d2 && !d3;
}

// Positions inside any argument list, where a run carrying a top-level comma
// may not be replaced: if the callee turns out to be a function-like macro the
// comma count is its arity. The callee need not be one of ours - _MM_SHUFFLE
// comes from <xmmintrin.h> - so every 'name( ... )' is treated as suspect.
static void markMacroArgs( const Flat& f, const Names& N, vector<char>& in )
{
	const vector<Tok>& F = f.F;
	(void)N;
	in.assign( F.size(), 0 );
	for (size_t i = 0; i + 1 < F.size(); i++)
	{
		if (!F[i].id() || !F[i + 1].is( "(" )) continue;
		size_t e = matchFwd( F, i + 1, F.size() );
		if (e == string::npos) continue;
		for (size_t k = i + 1; k <= e; k++) in[k] = 1;
	}
}

struct MacroDef { string name, param; vector<string> params; vector<Tok> seq; };

static void compact( Flat& f )
{
	size_t w = 0;
	for (size_t i = 0; i < f.F.size(); i++) if (!f.dead[i])
	{
		f.F[w] = f.F[i]; f.owner[w] = f.owner[i]; w++;
	}
	f.F.resize( w ); f.owner.resize( w ); f.dead.assign( w, 0 );
}

// Hands out short macro names that do not collide with anything in the header.
// The prefixed names come first, so the hottest macros stay recognizable; after
// those 62 run out it falls back to any free two character spelling rather than
// going to three characters, which is worth several kilobytes. A collision with
// application code is a redefinition or a syntax error, never silent breakage.
struct NameGen
{
	Names& N;
	string prefix;
	vector<string> pool;
	size_t ctr = 0, ctrLong = 0, longStart = 0;
	NameGen( Names& n, const string& p ) : N( n ), prefix( p ) { build(); ctrLong = longStart; }
	void build()
	{
		// Uppercase first: 'eq', 'lt' and friends are real members of standard
		// library types, and this header is often included before <string>.
		static const char* A = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
		static const char* B = "abcdefghijklmnopqrstuvwxyz0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
		if (!prefix.empty()) for (const char* q = B; *q; q++) pool.push_back( prefix + *q );
		for (const char* q = A; *q; q++) for (const char* r = B; *r; r++)
		{
			string s2 = string( 1, *q ) + *r;
			if (!prefix.empty() && s2[0] == prefix[0]) continue;		// already emitted above
			pool.push_back( s2 );
		}
		longStart = pool.size();
		string pfx = prefix.empty() ? string( "Z" ) : prefix;
		for (const char* q = B; *q; q++) for (const char* r = B; *r; r++)
			pool.push_back( pfx + *q + *r );
	}
	string at( size_t& c, size_t limit )
	{
		for (; c < limit; c++) if (!N.allIdent.count( pool[c] ))
		{
			N.allIdent.insert( pool[c] );
			return pool[c++];
		}
		// exhausted: fall back to a longer spelling
		static size_t extra = 0;
		for (; ; extra++)
		{
			char buf[32];
			snprintf( buf, sizeof( buf ), "%s%zu", prefix.empty() ? "Z" : prefix.c_str(), extra );
			string s2 = buf;
			if (!N.allIdent.count( s2 )) { N.allIdent.insert( s2 ); extra++; return s2; }
		}
	}
	string operator()() { return at( ctr, longStart ); }
	// Names for things used a handful of times; the two character names belong to
	// the factored runs, which are referenced hundreds of times each.
	string longName() { return at( ctrLong, pool.size() ); }
	void undo() { if (ctr) ctr--; }
	void undoLong() { if (ctrLong > longStart) ctrLong--; }
};

static size_t g_macroTail = 0;		// bytes of cleanup each generated macro costs
static size_t g_minRun = 4;			// shortest run worth a macro; shorter ones crowd out better ones
static size_t g_perRound = 0;		// 0 = apply every candidate a round turns up

static void factorMacros( Flat& f, Names& N, vector<MacroDef>& defs, NameGen& gen,
	int rounds, size_t maxN, size_t maxCand, long long minGain )
{
	auto freshName = [&]() -> string { return gen(); };
	for (int round = 0; round < rounds; round++)
	{
		vector<Tok>& F = f.F;
		size_t n = F.size();
		if (n < 8) break;
		struct C { uint64_t h; uint32_t cnt; uint32_t pos; uint32_t len; long long est; bool unsafe; };
		vector<C> cands;
		// L == 1 captures a single long identifier or keyword. The macro expands
		// back to the original spelling, so public names stay intact after
		// preprocessing while the source text shrinks.
		for (size_t L = 1; L <= maxN; L++)
		{
			unordered_map<uint64_t, pair<uint32_t, uint32_t>> cnt;	// hash -> (count, first pos)
			cnt.reserve( n / 2 );
			for (size_t i = 0; i + L <= n; i++)
			{
				if (f.owner[i] != f.owner[i + L - 1]) continue;
				if (L == 1 && ((F[i].k != T_ID && F[i].k != T_NUM) || F[i].s.size() < 3)) continue;
				uint64_t h = hashSeq( F, i, L );
				auto& e = cnt[h];
				if (e.first++ == 0) e.second = (uint32_t)i;
			}
			for (auto& kv : cnt)
			{
				if (kv.second.first < 3) continue;
				size_t pos = kv.second.second;
				size_t rl = renderLen( F, pos, L );
				if (rl < g_minRun) continue;
				long long raw = (long long)kv.second.first * ((long long)rl - 2);
				if (raw <= minGain || raw - (long long)(12 + rl + g_macroTail) <= 0) continue;
				long long est = raw - (long long)(12 + rl + g_macroTail);
				bool unsafe = false;
				if (!seqUsable( F, pos, L, N, &unsafe )) continue;
				cands.push_back( { kv.first, kv.second.first, (uint32_t)pos, (uint32_t)L, est, unsafe } );
			}
		}
		if (cands.empty()) break;
		sort( cands.begin(), cands.end(), []( const C& a, const C& b )
			{ return a.est != b.est ? a.est > b.est : a.len > b.len; } );
		if (cands.size() > maxCand) cands.resize( maxCand );
		// index positions by first-token spelling so matching stays cheap
		unordered_map<string, vector<uint32_t>> firstPos;
		unordered_set<string> firsts;
		for (const C& c : cands) firsts.insert( F[c.pos].s );
		for (size_t i = 0; i < n; i++) if (firsts.count( F[i].s )) firstPos[F[i].s].push_back( (uint32_t)i );
		vector<char> taken( n, 0 ), inArgs;
		markMacroArgs( f, N, inArgs );
		int applied = 0;
		for (const C& c : cands)
		{
			vector<Tok> seq( F.begin() + c.pos, F.begin() + c.pos + c.len );
			bool multiStmt = false;						// does the body hold a statement break?
			{
				int d = 0;
				for (size_t k = 0; k + 1 < seq.size(); k++)
				{
					const string& t = seq[k].s;
					if (t == "(" || t == "[" || t == "{") d++;
					else if (t == ")" || t == "]" || t == "}") d--;
					else if (t == ";" && d == 0) { multiStmt = true; break; }
				}
			}
			// find all non-overlapping, untaken occurrences
			vector<uint32_t> hits;
			auto& pl = firstPos[seq[0].s];
			uint32_t last = 0;
			bool first = true;
			for (uint32_t p : pl)
			{
				if (!first && p < last) continue;
				if (p + c.len > n) break;
				if (f.owner[p] != f.owner[p + c.len - 1]) continue;
				bool ok = true;
				for (uint32_t k = 0; k < c.len; k++)
					if (taken[p + k] || F[p + k].s != seq[k].s || F[p + k].k != seq[k].k) { ok = false; break; }
				if (!ok) continue;
				if (c.unsafe)
				{
					// the whole run must stay clear of any argument list, not just
					// its first token: a run may start outside and reach inside
					bool touches = false;
					for (uint32_t k = 0; k < c.len && !touches; k++) touches = inArgs[p + k] != 0;
					if (touches) continue;
				}
				if (multiStmt && p > 0)
				{
					const Tok& pv = F[p - 1];
					bool subStmt = pv.is( "else" ) || pv.is( "do" );
					if (!subStmt && pv.is( ")" ))
					{
						size_t o = matchBack( F, p - 1 );
						if (o != string::npos && o > 0 && (F[o - 1].is( "if" ) || F[o - 1].is( "for" ) ||
							F[o - 1].is( "while" ))) subStmt = true;
					}
					if (subStmt) continue;
				}
				hits.push_back( p );
				last = p + c.len;
				first = false;
			}
			if (hits.size() < 3) continue;
			size_t rl = renderLen( F, c.pos, c.len );
			string name = freshName();
			long long raw = (long long)hits.size() * ((long long)rl - (long long)name.size());
			long long gain = raw - (long long)(10 + name.size() + rl) -
				(long long)(g_macroTail ? 8 + name.size() : 0);
			if (raw <= minGain || gain <= 0) { gen.undo(); continue; }
			for (uint32_t p : hits)
			{
				F[p] = Tok( name, T_ID, true );
				for (uint32_t k = 1; k < c.len; k++) { f.dead[p + k] = 1; taken[p + k] = 1; }
				taken[p] = 1;
			}
			defs.push_back( { name, string(), {}, seq } );
			applied++;
			if (g_perRound && (size_t)applied >= g_perRound) break;
		}
		compact( f );
		if (!applied) break;
	}
}

// ----------------------------------------------------------------------------
// Parameterized macros: near-repeats that differ in one token.
//
// Exact matching stops at the first operand that varies. A great deal of this
// code base repeats with a single token changed - a lane index, a child
// pointer, an axis. Capturing those as a one-argument macro is the source-level
// equivalent of factoring the snippet into a function, but the preprocessor
// pastes the tokens back, so the compiler sees exactly what it saw before and
// no inlining decision changes.
// ----------------------------------------------------------------------------

struct ParamCand { uint32_t pos, len, hole, cnt; long long est; bool unsafe; };

static uint64_t hashHoled( const vector<Tok>& F, size_t i, size_t n, size_t hole )
{
	uint64_t h = 1469598103934665603ull;
	for (size_t k = 0; k < n; k++)
	{
		if (k == hole) { h ^= 0xfeedu; h *= 1099511628211ull; continue; }
		for (char c : F[i + k].s) { h ^= (unsigned char)c; h *= 1099511628211ull; }
		h ^= 1u; h *= 1099511628211ull;
	}
	return h;
}

// Is this run safe to put behind a macro at the given occurrence?
static bool hitSafe( const vector<Tok>& F, size_t p, size_t len )
{
	bool multi = false;
	int d = 0;
	for (size_t k = 0; k + 1 < len; k++)
	{
		const string& t = F[p + k].s;
		if (t == "(" || t == "[" || t == "{") d++;
		else if (t == ")" || t == "]" || t == "}") d--;
		else if (t == ";" && d == 0) { multi = true; break; }
	}
	if (!multi || p == 0) return true;
	const Tok& pv = F[p - 1];
	if (pv.is( "else" ) || pv.is( "do" )) return false;
	if (pv.is( ")" ))
	{
		size_t o = matchBack( F, p - 1 );
		if (o != string::npos && o > 0 && (F[o - 1].is( "if" ) || F[o - 1].is( "for" ) ||
			F[o - 1].is( "while" ))) return false;
	}
	return true;
}

static void factorParamMacros( Flat& f, Names& N, vector<MacroDef>& defs, NameGen& gen,
	int rounds, size_t maxN, long long minGain, size_t& made )
{
	for (int round = 0; round < rounds; round++)
	{
		vector<Tok>& F = f.F;
		size_t n = F.size();
		if (n < 16) break;
		vector<ParamCand> cands;
		for (size_t L = 4; L <= maxN; L++)
		{
			for (size_t hole = 0; hole < L; hole++)
			{
				// hash -> (count, first position, hole token varies)
				unordered_map<uint64_t, pair<uint32_t, uint32_t>> cnt;
				unordered_map<uint64_t, char> varies;
				for (size_t i = 0; i + L <= n; i++)
				{
					const Tok& h = F[i + hole];
					if (h.k != T_ID && h.k != T_NUM) continue;
					if (N.funcLikeMacro.count( h.s )) continue;
					if (f.owner[i] != f.owner[i + L - 1]) continue;
					uint64_t k = hashHoled( F, i, L, hole );
					auto& e = cnt[k];
					if (e.first++ == 0) e.second = (uint32_t)i;
					else if (!varies[k] && F[e.second + hole].s != h.s) varies[k] = 1;
				}
				for (auto& kv : cnt)
				{
					if (kv.second.first < 3 || !varies[kv.first]) continue;
					size_t pos = kv.second.second;
					size_t rl = renderLen( F, pos, L ), al = F[pos + hole].s.size();
					if (rl < 12) continue;
					// name(arg) replaces the run; assume a 3 character macro name
					long long per = (long long)rl - (long long)(2 + 2 + al);
					long long raw = (long long)kv.second.first * per;
					if (raw <= minGain || raw - (long long)(15 + rl + g_macroTail) <= 0) continue;
					long long est = raw - (long long)(15 + rl + g_macroTail);
					bool unsafe = false;
					if (!seqUsable( F, pos, L, N, &unsafe ) || !seqBalanced( F, pos, L )) continue;
					cands.push_back( { (uint32_t)pos, (uint32_t)L, (uint32_t)hole, kv.second.first, est, unsafe } );
				}
			}
		}
		if (cands.empty()) break;
		sort( cands.begin(), cands.end(), []( const ParamCand& a, const ParamCand& b )
			{ return a.est != b.est ? a.est > b.est : a.len > b.len; } );
		if (cands.size() > 2000) cands.resize( 2000 );
		unordered_map<string, vector<uint32_t>> firstPos;
		unordered_set<string> firsts;
		for (const ParamCand& c : cands) firsts.insert( F[c.pos].s );
		for (size_t i = 0; i < n; i++) if (firsts.count( F[i].s )) firstPos[F[i].s].push_back( (uint32_t)i );
		vector<char> taken( n, 0 ), inArgs;
		markMacroArgs( f, N, inArgs );
		int applied = 0;
		for (const ParamCand& c : cands)
		{
			vector<Tok> seq( F.begin() + c.pos, F.begin() + c.pos + c.len );
			vector<uint32_t> hits;
			long long origBytes = 0, argBytes = 0;
			uint32_t last = 0;
			bool first = true;
			for (uint32_t p : firstPos[seq[0].s])
			{
				if (!first && p < last) continue;
				if (p + c.len > n) continue;
				if (f.owner[p] != f.owner[p + c.len - 1]) continue;
				bool ok = true;
				for (uint32_t k = 0; k < c.len && ok; k++)
				{
					if (taken[p + k]) ok = false;
					else if (k == c.hole) { if (F[p + k].k != T_ID && F[p + k].k != T_NUM) ok = false;
						else if (N.funcLikeMacro.count( F[p + k].s )) ok = false; }
					else if (F[p + k].s != seq[k].s || F[p + k].k != seq[k].k) ok = false;
				}
				if (!ok || !hitSafe( F, p, c.len )) continue;
				if (c.unsafe)
				{
					// collapsing a run that carries a top-level comma would drop an
					// argument from an enclosing function-like macro call
					bool touches = false;
					for (uint32_t k = 0; k < c.len && !touches; k++) touches = inArgs[p + k] != 0;
					if (touches) continue;
				}
				hits.push_back( p );
				origBytes += (long long)renderLen( F, p, c.len );
				argBytes += (long long)F[p + c.hole].s.size();
				last = p + c.len;
				first = false;
			}
			if (hits.size() < 3) continue;
			// the parameter must be a spelling the body does not already use
			string param;
			for (const char* q = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"; *q && param.empty(); q++)
			{
				string t( 1, *q );
				bool clash = false;
				for (size_t k = 0; k < c.len; k++) if (k != c.hole && F[c.pos + k].s == t) clash = true;
				if (!clash) param = t;
			}
			if (param.empty()) continue;
			string name = gen();
			size_t bodyLen = renderLen( F, c.pos, c.len );
			long long raw = origBytes - (long long)hits.size() * (long long)(name.size() + 2) - argBytes;
			long long gain = raw - (long long)(11 + name.size() + bodyLen) -
				(long long)(g_macroTail ? 8 + name.size() : 0);
			if (raw <= minGain || gain <= 0) { gen.undo(); continue; }
			MacroDef d;
			d.name = name;
			d.param = param;
			d.seq.assign( seq.begin(), seq.end() );
			d.seq[c.hole] = Tok( param, T_ID, true );
			for (uint32_t p : hits)
			{
				Tok arg = F[p + c.hole];
				for (uint32_t k = 0; k < c.len; k++) taken[p + k] = 1;
				F[p] = Tok( name, T_ID, true );
				F[p + 1] = Tok( "(", T_PUNCT );
				F[p + 2] = arg;
				F[p + 3] = Tok( ")", T_PUNCT );
				for (uint32_t k = 4; k < c.len; k++) f.dead[p + k] = 1;
			}
			defs.push_back( d );
			applied++, made++;
		}
		compact( f );
		if (!applied) break;
	}
}

// ----------------------------------------------------------------------------
// The library's own function-like macros.
//
// Object-like internal macros are already handled: the single token pass aliases
// TEMPLATED to a two character name and the preprocessor expands it straight
// back. A function-like macro cannot be aliased that way, because the alias
// would leave its '(' outside, so those are renamed outright - at the #define,
// at every #undef, and at every call site.
//
// A macro is treated as internal when its name never appears in a conditional
// directive: a configuration switch is always tested somewhere with #ifdef,
// #ifndef or defined(), so one that is never tested is not configuration.
// ----------------------------------------------------------------------------

static size_t shortenInternalMacros( vector<Chunk>& C, Names& N, NameGen& gen,
	const set<string>& keep, const set<string>& force, bool list )
{
	unordered_set<string> conditional, defined_;
	for (const Chunk& c : C)
	{
		if (!c.dir) continue;
		if (c.t[0].is( "if" ) || c.t[0].is( "ifdef" ) || c.t[0].is( "ifndef" ) || c.t[0].is( "elif" ))
			for (size_t i = 1; i < c.t.size(); i++) if (c.t[i].id()) conditional.insert( c.t[i].s );
		if (c.t[0].is( "define" ) && c.t.size() > 2 && c.t[1].id() && c.t[2].is( "(" ) && !c.t[2].ws)
			defined_.insert( c.t[1].s );
	}
	// names a caller is known to reach for, and the warning plumbing we may drop
	static const char* PUBLIC[] = { "ALIGNED", "TINYBVH_ALIGNED", "BVH_FATAL_ERROR",
		"BVH_FATAL_ERROR_IF", "WARNING" };
	map<string, size_t> use;
	for (const Chunk& c : C) for (const Tok& t : c.t) if (t.id()) use[t.s]++;
	vector<pair<size_t, string>> cand;
	for (const string& m : defined_)
	{
		if (force.count( m )) { cand.push_back( { (m.size() - 2) * use[m], m } ); continue; }
		if (keep.count( m ) || conditional.count( m )) continue;
		bool pub = false;
		for (const char* q : PUBLIC) if (m == q) pub = true;
		if (pub || m.size() < 4) continue;
		cand.push_back( { (m.size() - 2) * use[m], m } );
	}
	sort( cand.begin(), cand.end(), []( const pair<size_t, string>& a, const pair<size_t, string>& b )
		{ return a.first != b.first ? a.first > b.first : a.second < b.second; } );
	unordered_map<string, string> ren;
	for (auto& c : cand)
	{
		string n = gen.longName();
		if (n.size() >= c.second.size()) { gen.undoLong(); continue; }
		ren[c.second] = n;
		N.macroIdent.insert( n );
		N.funcLikeMacro.insert( n );
		if (list) fprintf( stderr, "macro %s -> %s (x%zu)\n", c.second.c_str(), n.c_str(), use[c.second] );
	}
	if (ren.empty()) return 0;
	for (Chunk& c : C) for (Tok& t : c.t)
	{
		if (!t.id()) continue;
		auto it = ren.find( t.s );
		if (it != ren.end()) t.s = it->second;
	}
	return ren.size();
}

// ----------------------------------------------------------------------------
// Multi-argument macros: runs that repeat with only their numeric literals
// changing. A 4x4 matrix inverse writes 'T[a]*T[b]*T[c]' ninety-six times with
// nothing but the three indices varying; one macro with three parameters
// collapses all of them. Holes are numeric literals only, which keeps the
// arguments single tokens, so no argument can change how the body parses.
// ----------------------------------------------------------------------------

struct NumCand { uint32_t pos, len, cnt; uint64_t mask; long long est; bool unsafe; };

// Is this token an operand that a macro could take as an argument?
static bool holeable( const Tok& t, const Names& N, bool idHoles )
{
	if (t.k == T_NUM) return true;
	if (!idHoles || t.k != T_ID) return false;
	return !N.keyword.count( t.s ) && !N.funcLikeMacro.count( t.s );
}

static uint64_t hashNumHoled( const vector<Tok>& F, size_t i, size_t n, const Names& N, bool idHoles )
{
	uint64_t h = 1469598103934665603ull;
	for (size_t k = 0; k < n; k++)
	{
		if (holeable( F[i + k], N, idHoles )) { h ^= 0xbeefu; h *= 1099511628211ull; continue; }
		for (char c : F[i + k].s) { h ^= (unsigned char)c; h *= 1099511628211ull; }
		h ^= 1u; h *= 1099511628211ull;
	}
	return h;
}

static void factorNumericMacros( Flat& f, Names& N, vector<MacroDef>& defs, NameGen& gen,
	int rounds, size_t maxN, long long minGain, size_t& made, bool idHoles = false )
{
	for (int round = 0; round < rounds; round++)
	{
		vector<Tok>& F = f.F;
		size_t n = F.size();
		if (n < 16) break;
		vector<NumCand> cands;
		for (size_t L = 4; L <= maxN && L <= 60; L++)
		{
			struct Ent { uint32_t cnt, pos; uint64_t mask; };
			unordered_map<uint64_t, Ent> grp;
			for (size_t i = 0; i + L <= n; i++)
			{
				if (f.owner[i] != f.owner[i + L - 1]) continue;
				uint64_t num = 0;
				int cnt = 0;
				for (size_t k = 0; k < L; k++) if (holeable( F[i + k], N, idHoles )) num |= 1ull << k, cnt++;
				if (cnt == 0 || cnt > 16) continue;
				uint64_t h = hashNumHoled( F, i, L, N, idHoles );
				auto it = grp.find( h );
				if (it == grp.end()) { grp[h] = { 1, (uint32_t)i, 0 }; continue; }
				it->second.cnt++;
				// remember which literal positions differ from the first occurrence
				for (size_t k = 0; k < L; k++) if ((num >> k) & 1)
					if (F[i + k].s != F[it->second.pos + k].s) it->second.mask |= 1ull << k;
			}
			for (auto& kv : grp)
			{
				Ent& e = kv.second;
				if (e.cnt < 3 || !e.mask) continue;
				size_t holes = 0;
				for (size_t k = 0; k < L; k++) if ((e.mask >> k) & 1) holes++;
				if (holes > (idHoles ? 4u : 8u) || L < 2 + 2 * holes) continue;
				size_t rl = renderLen( F, e.pos, L ), args = 0;
				for (size_t k = 0; k < L; k++) if ((e.mask >> k) & 1) args += F[e.pos + k].s.size();
				long long per = (long long)rl - (long long)(2 + 2 + args + holes - 1);
				long long raw = (long long)e.cnt * per;
				if (per <= 0 || raw <= minGain || raw - (long long)(19 + rl + g_macroTail) <= 0) continue;
				bool unsafe = false;
				if (!seqUsable( F, e.pos, L, N, &unsafe ) || !seqBalanced( F, e.pos, L )) continue;
				cands.push_back( { e.pos, (uint32_t)L, e.cnt, e.mask,
					raw - (long long)(19 + rl + g_macroTail), unsafe } );
			}
		}
		if (cands.empty()) break;
		sort( cands.begin(), cands.end(), []( const NumCand& a, const NumCand& b )
			{ return a.est != b.est ? a.est > b.est : a.len > b.len; } );
		if (cands.size() > 1500) cands.resize( 1500 );
		unordered_map<string, vector<uint32_t>> firstPos;
		unordered_set<string> firsts;
		for (const NumCand& c : cands) firsts.insert( F[c.pos].s );
		for (size_t i = 0; i < n; i++) if (firsts.count( F[i].s )) firstPos[F[i].s].push_back( (uint32_t)i );
		vector<char> taken( n, 0 ), inArgs;
		markMacroArgs( f, N, inArgs );
		int applied = 0;
		for (const NumCand& c : cands)
		{
			vector<Tok> seq( F.begin() + c.pos, F.begin() + c.pos + c.len );
			vector<uint32_t> hits;
			long long origBytes = 0, callBytes = 0;
			uint32_t last = 0;
			bool first = true;
			size_t holes = 0;
			for (uint32_t k = 0; k < c.len; k++) if ((c.mask >> k) & 1) holes++;
			for (uint32_t p : firstPos[seq[0].s])
			{
				if (!first && p < last) continue;
				if (p + c.len > n || f.owner[p] != f.owner[p + c.len - 1]) continue;
				bool ok = true;
				for (uint32_t k = 0; k < c.len && ok; k++)
				{
					if (taken[p + k]) ok = false;
					else if ((c.mask >> k) & 1) { if (!holeable( F[p + k], N, idHoles )) ok = false; }
					else if (F[p + k].s != seq[k].s || F[p + k].k != seq[k].k) ok = false;
				}
				if (!ok || !hitSafe( F, p, c.len )) continue;
				if (c.unsafe)
				{
					bool touches = false;
					for (uint32_t k = 0; k < c.len && !touches; k++) touches = inArgs[p + k] != 0;
					if (touches) continue;
				}
				hits.push_back( p );
				origBytes += (long long)renderLen( F, p, c.len );
				for (uint32_t k = 0; k < c.len; k++) if ((c.mask >> k) & 1) callBytes += F[p + k].s.size();
				last = p + c.len;
				first = false;
			}
			if (hits.size() < 3) continue;
			// parameter names the body does not already use
			vector<string> param;
			for (const char* q = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"; *q && param.size() < holes; q++)
			{
				string t( 1, *q );
				bool clash = false;
				for (uint32_t k = 0; k < c.len; k++) if (!((c.mask >> k) & 1) && F[c.pos + k].s == t) clash = true;
				if (!clash) param.push_back( t );
			}
			if (param.size() < holes) continue;
			string name = gen();
			size_t bodyLen = renderLen( F, c.pos, c.len );
			long long raw = origBytes - (long long)hits.size() * (long long)(name.size() + 2 + holes - 1) - callBytes;
			long long gain = raw - (long long)(10 + name.size() + 2 * holes + 1 + bodyLen) -
				(long long)(g_macroTail ? 8 + name.size() : 0);
			if (raw <= minGain || gain <= 0) { gen.undo(); continue; }
			MacroDef d;
			d.name = name;
			d.seq.assign( seq.begin(), seq.end() );
			{
				size_t h = 0;
				for (uint32_t k = 0; k < c.len; k++) if ((c.mask >> k) & 1)
					d.seq[k] = Tok( param[h], T_ID, true ), d.params.push_back( param[h] ), h++;
			}
			for (uint32_t p : hits)
			{
				vector<Tok> args;
				for (uint32_t k = 0; k < c.len; k++) if ((c.mask >> k) & 1) args.push_back( F[p + k] );
				for (uint32_t k = 0; k < c.len; k++) taken[p + k] = 1;
				size_t w = p;
				F[w++] = Tok( name, T_ID, true );
				F[w++] = Tok( "(", T_PUNCT );
				for (size_t a = 0; a < args.size(); a++)
				{
					if (a) F[w++] = Tok( ",", T_PUNCT );
					F[w++] = args[a];
				}
				F[w++] = Tok( ")", T_PUNCT );
				for (size_t k = w; k < (size_t)p + c.len; k++) f.dead[k] = 1;
			}
			defs.push_back( d );
			applied++, made++;
		}
		compact( f );
		if (!applied) break;
	}
}

// ============================================================================
//
//        O U T P U T
//
// ============================================================================

struct Writer
{
	string out, line;
	size_t width;
	explicit Writer( size_t w ) : width( w ) {}
	void flush() { if (!line.empty()) { out += line; out += '\n'; line.clear(); } }
	void raw( const string& s ) { flush(); out += s; if (s.empty() || s.back() != '\n') out += '\n'; }
	void code( const Tok& prev, bool havePrev, const Tok& t )
	{
		size_t sep = (havePrev && !line.empty() && needSpace( prev, t )) ? 1 : 0;
		if (t.k == T_STR && t.s.size() > width - 2 && t.s.front() == '"' && t.s.back() == '"')
		{
			// a literal longer than one line: split it into adjacent literals
			flush();
			string body = t.s.substr( 1, t.s.size() - 2 );
			size_t i = 0;
			while (i < body.size())
			{
				string part;
				while (i < body.size() && part.size() < width - 4)
				{
					if (body[i] == '\\' && i + 1 < body.size()) { part += body[i]; part += body[i + 1]; i += 2; }
					else part += body[i++];
				}
				out += "\"" + part + "\"\n";
			}
			return;
		}
		if (!line.empty() && line.size() + sep + t.s.size() > width) flush();
		if (!line.empty()) line.append( sep, ' ' );
		line += t.s;
	}
};

// Separator flags for a directive's tokens. A #define needs exactly the right
// spacing: none before a function-like macro's '(', one before the replacement
// list (ISO C++11 requires whitespace after the macro name).
static vector<bool> directiveSeps( const Chunk& c )
{
	vector<bool> sp( c.t.size(), false );
	size_t body = string::npos;
	if (c.t[0].is( "define" ) && c.t.size() > 1)
	{
		body = 2;
		if (c.t.size() > 2 && c.t[2].is( "(" ) && !c.t[2].ws)
		{
			size_t d = 0, k = 2;
			for (; k < c.t.size(); k++) { if (c.t[k].is( "(" )) d++; else if (c.t[k].is( ")" ) && --d == 0) break; }
			body = k + 1;
		}
	}
	for (size_t i = 1; i < c.t.size(); i++)
		sp[i] = i == 1 || i == body || needSpace( c.t[i - 1], c.t[i] );
	return sp;
}

static string directiveText( const Chunk& c )
{
	string s = "#" + c.t[0].s;
	if (c.rawTail) return s + " " + c.raw;
	vector<bool> sp = directiveSeps( c );
	for (size_t i = 1; i < c.t.size(); i++) { if (sp[i]) s += ' '; s += c.t[i].s; }
	return s;
}

static void emitDirective( Writer& w, const Chunk& c )
{
	w.flush();
	string s = directiveText( c );
	if (s.size() <= w.width || c.rawTail) { w.out += s; w.out += '\n'; return; }
	// wrap with line splices, at token boundaries
	string cur = "#" + c.t[0].s;
	vector<bool> seps = directiveSeps( c );
	for (size_t i = 1; i < c.t.size(); i++)
	{
		bool sp = seps[i];
		if (cur.size() + (sp ? 1 : 0) + c.t[i].s.size() + 2 > w.width && cur.size() > 1)
		{
			w.out += cur + " \\\n";
			cur.clear();
			sp = false;
		}
		if (sp) cur += ' ';
		cur += c.t[i].s;
	}
	w.out += cur;
	w.out += '\n';
}

static string emit( const vector<Chunk>& C, size_t width )
{
	Writer w( width );
	Tok prev;
	bool havePrev = false;
	for (const Chunk& c : C)
	{
		if (c.dir) { emitDirective( w, c ); havePrev = false; continue; }
		for (const Tok& t : c.t) { w.code( prev, havePrev, t ); prev = t; havePrev = true; }
	}
	w.flush();
	return w.out;
}

// ============================================================================
//
//        D R I V E R
//
// ============================================================================

// Internal plumbing that the regenerated SIMD detection block makes redundant.
static void dropUnusedInternalMacros( vector<Chunk>& C )
{
	static const char* INTERNAL[] = { "WARNING", "EMIT_COMPILER_WARNING_STRINGIFY0",
		"EMIT_COMPILER_WARNING_STRINGIFY1", "EMIT_COMPILER_WARNING_COMPOSE",
		"EMIT_COMPILER_MESSAGE_PREFACE" };
	for (int pass = 0; pass < 4; pass++)
	{
		unordered_map<string, int> use;
		for (const Chunk& c : C)
		{
			size_t skip = (c.dir && c.t[0].is( "define" )) ? 2 : 0;
			for (size_t i = skip; i < c.t.size(); i++) if (c.t[i].id()) use[c.t[i].s]++;
		}
		vector<Chunk> keep;
		bool changed = false;
		for (const Chunk& c : C)
		{
			bool drop = false;
			if (c.dir && c.t[0].is( "define" ) && c.t.size() > 1 && c.t[1].id())
				for (const char* m : INTERNAL) if (c.t[1].s == m && use[c.t[1].s] == 0) drop = true;
			if (drop) changed = true; else keep.push_back( c );
		}
		C.swap( keep );
		if (!changed) break;
	}
}

static void countShape( const vector<Chunk>& C, long long& par, long long& brc, long long& brk, size_t& toks )
{
	par = brc = brk = 0, toks = 0;
	for (const Chunk& c : C) if (!c.dir) for (const Tok& t : c.t)
	{
		toks++;
		if (t.s == "(") par++; else if (t.s == ")") par--;
		else if (t.s == "{") brc++; else if (t.s == "}") brc--;
		else if (t.s == "[") brk++; else if (t.s == "]") brk--;
	}
}

int main( int argc, char** argv )
{
	string dir = ".", outPath = "microbvh.h", prefix = "Z";
	Isa isa = ISA_NONE;
	size_t width = 100, maxSeq = 32, maxCand = 4000;
	int rounds = 10, paramRounds = 10;
	bool opHoles = true;
	size_t paramSeq = 32, minRun = 4, perRound = 0;
	long long minGain = -1;
	bool doRename = true, doMacros = true, doMembers = true, doBraces = true;
	set<string> keepMacro, forceMacro;
	bool doMacroNames = true, doBool = true, doLicense = false, doUndef = true;
	vector<pair<string, long long>> cliDefine;
	vector<string> cliUndef;
	int constMode = -1;				// -1 = auto: only worthwhile without macro factoring
	for (int i = 1; i < argc; i++)
	{
		string a = argv[i];
		auto val = [&]() -> string { if (i + 1 >= argc) fatal( "missing value for " + a ); return argv[++i]; };
		if (a == "--in") dir = val();
		else if (a == "--out") outPath = val();
		else if (a == "--width") width = (size_t)atoi( val().c_str() );
		else if (a == "--rounds") rounds = atoi( val().c_str() );
		else if (a == "--maxseq") maxSeq = (size_t)atoi( val().c_str() );
		else if (a == "--maxcand") maxCand = (size_t)atoi( val().c_str() );
		else if (a == "--param-rounds") paramRounds = atoi( val().c_str() );
		else if (a == "--param-seq") paramSeq = (size_t)atoi( val().c_str() );
		else if (a == "--minrun") minRun = (size_t)atoi( val().c_str() );
		else if (a == "--per-round") perRound = (size_t)atoi( val().c_str() );
		else if (a == "--no-operand-holes") opHoles = false;
		else if (a == "--gain") minGain = atoll( val().c_str() );
		else if (a == "--macro-prefix") prefix = val();
		else if (a == "--no-rename") doRename = false;
		else if (a == "--no-members") doMembers = false;
		else if (a == "--no-braces") doBraces = false;
		else if (a == "--no-const") constMode = 0;
		else if (a == "--const") constMode = 1;
		else if (a == "-D" || a == "--define")
		{
			string v = val(), name = v;
			long long num = LLONG_MIN;
			size_t eq = v.find( '=' );
			if (eq != string::npos) { name = v.substr( 0, eq ); num = atoll( v.c_str() + eq + 1 ); }
			cliDefine.push_back( { name, num } );
		}
		else if (a == "-U" || a == "--undefine") cliUndef.push_back( val() );
		else if (a == "--keep-macro") keepMacro.insert( val() );
		else if (a == "--shorten-macro") forceMacro.insert( val() );
		else if (a == "--no-macro-names") doMacroNames = false;
		else if (a == "--no-bool") doBool = false;
		else if (a == "--license") doLicense = true;
		else if (a == "--no-undef") doUndef = false;
		else if (a == "--no-macros") doMacros = false;
		else if (a == "--isa")
		{
			string v = val();
			for (char& c : v) c = (char)tolower( (unsigned char)c );
			if (v == "none" || v == "scalar") isa = ISA_NONE;
			else if (v == "sse") isa = ISA_SSE;
			else if (v == "avx") isa = ISA_AVX;
			else if (v == "avx2") isa = ISA_AVX2;
			else if (v == "neon") isa = ISA_NEON;
			else fatal( "unknown --isa " + v + " (none|sse|avx|avx2|neon)" );
		}
		else if (a == "-h" || a == "--help")
		{
			printf( "microbvh - condense the TinyBVH headers into one.\n\n"
				"  --in <dir>           folder holding the six tiny_bvh headers (default: .)\n"
				"  --out <file>         output header (default: microbvh.h)\n"
				"  --isa <name>         none|sse|avx|avx2|neon; default none, the scalar path only\n"
				"  --width <n>          maximum line length (default 100)\n"
				"  --macro-prefix <s>   prefix for generated macros (default Z)\n"
				"  --rounds <n>         macro factoring rounds (default 10)\n"
				"  --maxseq <n>         longest token run a macro may capture (default 32)\n"
				"  --maxcand <n>        macro candidates considered per round (default 4000)\n"
				"  --param-rounds <n>   rounds of one-argument macro factoring (default 10, 0 = off)\n"
				"  --param-seq <n>      longest run a one-argument macro may capture (default 32)\n"
				"  --minrun <n>         shortest run a macro may capture (default 4)\n"
				"  --gain <n>           text a macro must remove to be worth a line (default: --width)\n"
				"  --no-rename          keep original local variable names\n"
				"  --no-macros          do not factor repeated token runs into macros\n"
				"  --no-members         keep original internal (private/protected) member names\n"
				"  --no-macro-names     keep original names for the library's own macros\n"
				"  --keep-macro <name>  never rename this macro\n"
				"  --shorten-macro <n>  rename this macro even if the heuristic would keep it\n"
				"  --no-bool            keep 'true' and 'false' instead of 1 and 0\n"
				"  --license            emit the full MIT license block (default: one line)\n"
				"  --no-undef           leave the generated macros defined after the header\n"
				"  -D NAME[=n] / -U NAME  pin a configuration macro and fold its conditionals\n"
				"  --no-braces          keep every block brace; no comma folding\n"
				"  --const / --no-const drop top-level const on scalar parameters and locals;\n"
				"                       default: only when --no-macros, where it pays off\n\n"
				"Set MICROBVH_DEBUG to list statements the declaration parser did not\n"
				"recognize, and MICROBVH_ALIAS to report what per-function aliasing of\n"
				"long names could still buy; both are diagnostics, not transformations.\n" );
			return 0;
		}
		else fatal( "unknown option " + a );
	}
	if (width < 40) fatal( "--width below 40 is not workable" );

	string license;
	string src = assemble( dir, isa, license );
	vector<Chunk> raw;
	lexToChunks( src, raw );
	SymTab S;
	S.known["BVH_USESSE"] = isa == ISA_SSE || isa == ISA_AVX || isa == ISA_AVX2;
	S.known["BVH_USEAVX"] = isa == ISA_AVX || isa == ISA_AVX2;
	S.known["BVH_USEAVX2"] = isa == ISA_AVX2;
	S.known["BVH_USENEON"] = isa == ISA_NEON;
	for (auto& d : cliDefine) { S.known[d.first] = true; if (d.second != LLONG_MIN) S.value[d.first] = d.second; }
	for (const string& u : cliUndef) { S.known[u] = false; S.value.erase( u ); }
	vector<Chunk> C;
	foldRegion( raw, 0, raw.size(), S, C );
	dropUnusedInternalMacros( C );

	long long p0, b0, k0; size_t t0;
	countShape( C, p0, b0, k0, t0 );

	Names N;
	collectNames( C, N );
	NameGen gen( N, prefix );
	size_t macroRenamed = 0;
	if (doMacroNames)
		macroRenamed = shortenInternalMacros( C, N, gen, keepMacro, forceMacro,
			getenv( "MICROBVH_DEBUG" ) != nullptr );
	Flat f;
	flatten( C, f );
	RenameStats rs;
	size_t memberCount = 0, memberSaved = 0;
	if (doRename && doMembers) renameMembers( f, N, memberCount, memberSaved );
	if (doRename) renameLocals( f, N, rs );
	size_t constDropped = 0;
	if (constMode == 1 || (constMode < 0 && !doMacros)) constDropped = stripConst( f, N );
	size_t protoNames = doRename ? stripPrototypeNames( f, N ) : 0;
	compact( f );
	LitStats lit;
	shortenConstants( f, lit, doBool );
	size_t declMerged = 0;
	if (doBraces) declMerged = mergeDecls( f, N );
	compact( f );
	Simplifier simp( f, N );
	if (doBraces) simp.run();
	compact( f );
	vector<MacroDef> defs;
	{	// every pass up to here must leave the bracket structure untouched;
		// macro factoring may move brackets into bodies, and is covered by the
		// compile-and-compare verification instead
		unflatten( C, f );
		long long p1, b1, k1; size_t t1;
		countShape( C, p1, b1, k1, t1 );
		if (p0 != p1 || b0 != b1 || k0 != k1) fatal( "internal error: bracket balance changed" );
	}
	g_macroTail = doUndef ? 10 : 0;		// "#undef Xy\n"
	g_minRun = minRun;
	g_perRound = perRound;
	long long gainLimit = minGain < 0 ? (long long)width : minGain;
	size_t paramMade = 0;
	if (doMacros) for (int pass = 0; pass < (paramRounds > 0 ? 3 : 1); pass++)
	{
		size_t before = defs.size();
		factorMacros( f, N, defs, gen, rounds, maxSeq, maxCand, gainLimit );
		if (paramRounds > 0)
		{
			factorParamMacros( f, N, defs, gen, paramRounds, paramSeq, gainLimit, paramMade );
			factorNumericMacros( f, N, defs, gen, paramRounds, paramSeq, gainLimit, paramMade );
			if (opHoles) factorNumericMacros( f, N, defs, gen, paramRounds, paramSeq, gainLimit, paramMade, true );
		}
		if (defs.size() == before) break;
	}
	if (getenv( "MICROBVH_ALIAS" ))
	{
		// What a per-function 'auto a = X;' alias could still buy, on top of
		// everything the macro pass already took.
		vector<Body> bb;
		findBodies( f.F, bb );
		map<string, long long> top;
		long long gainLong = 0, gainMacro = 0;
		size_t siteLong = 0, siteMacro = 0;
		for (const Body& b : bb)
		{
			map<string, size_t> c;
			for (size_t i = b.brace + 1; i < b.close; i++) if (f.F[i].id() &&
				!(f.F[i - 1].is( "." ) || f.F[i - 1].is( "->" ) || f.F[i - 1].is( "::" ))) c[f.F[i].s]++;
			for (auto& kv : c)
			{
				size_t L = kv.first.size(), k = kv.second;
				if (L < 2) continue;
				long long g = (long long)((L - 1) * k) - (long long)(8 + L);	// "auto a=X;"
				if (g <= 0) continue;
				if (L <= 3) gainMacro += g, siteMacro++; else { gainLong += g, siteLong++; top[kv.first] += g; }
			}
		}
		fprintf( stderr, "alias ceiling: long names %lld bytes over %zu sites; "
			"already-macro names %lld bytes over %zu sites\n", gainLong, siteLong, gainMacro, siteMacro );
		vector<pair<long long, string>> v;
		for (auto& kv : top) v.push_back( { kv.second, kv.first } );
		sort( v.rbegin(), v.rend() );
		for (size_t i = 0; i < v.size() && i < 10; i++)
			fprintf( stderr, "   %-22s %lld\n", v[i].second.c_str(), v[i].first );
	}
	unflatten( C, f );

	string body = emit( C, width );
	// The full MIT block belongs with the readable sources; a condensed header
	// carries a one line notice instead, with --license to restore the original.
	string head = doLicense ? license + "\n" :
		"// TinyBVH by Jacco Bikker, MIT licensed - see tiny_bvh.h for the full notice.\n";
	head += "// microbvh.h - generated from the TinyBVH headers by microbvh.cpp.\n";
	head += string( "// Target ISA: " ) + isaName( isa ) + ". Drop-in replacement for tiny_bvh.h.\n";
	head += "// Do not edit; edit the TinyBVH sources and regenerate.\n";
	string tail;
	if (!defs.empty())
	{
		Writer w( width );
		for (const MacroDef& d : defs)
		{
			Chunk c;
			c.dir = true;
			c.t.push_back( Tok( "define", T_ID ) );
			c.t.push_back( Tok( d.name, T_ID ) );
			if (!d.param.empty() || !d.params.empty())
			{
				c.t.push_back( Tok( "(", T_PUNCT ) );		// function-like: no space here
				if (!d.param.empty()) c.t.push_back( Tok( d.param, T_ID ) );
				for (size_t q = 0; q < d.params.size(); q++)
				{
					if (q) c.t.push_back( Tok( ",", T_PUNCT ) );
					c.t.push_back( Tok( d.params[q], T_ID ) );
				}
				c.t.push_back( Tok( ")", T_PUNCT ) );
			}
			for (const Tok& t : d.seq) c.t.push_back( t );
			if (d.param.empty() && d.params.empty() && c.t[2].is( "(" )) c.t[2].ws = true;
			emitDirective( w, c );
		}
		head += w.out;
		if (doUndef)
		{
			// Nothing generated here escapes the header: each inclusion defines
			// these and takes them away again, so a second include for the
			// implementation still sees them and user code never does.
			for (const MacroDef& d : defs) tail += "#undef " + d.name + "\n";
		}
	}
	head += "#if defined __GNUC__ || defined __clang__\n#pragma GCC diagnostic push\n"
		"#pragma GCC diagnostic ignored \"-Wmisleading-indentation\"\n#endif\n";
	string result = head + body + tail +
		"#if defined __GNUC__ || defined __clang__\n#pragma GCC diagnostic pop\n#endif\n";

	ofstream o( outPath, ios::binary );
	if (!o) fatal( "cannot write " + outPath );
	o << result;
	o.close();

	size_t before = 0;
	for (const char* fn : { "tiny_bvh.h", "tiny_bvh_base.h", "tiny_bvh_x86_float.h",
		"tiny_bvh_x86_double.h", "tiny_bvh_arm_float.h", "tiny_bvh_arm_double.h" })
		before += readFile( dir.empty() ? string( fn ) : dir + "/" + fn ).size();
	size_t lines = 1, longest = 0, cur = 0;
	for (char c : result) { if (c == '\n') { lines++; longest = max( longest, cur ); cur = 0; } else cur++; }
	printf( "microbvh: %s, ISA=%s\n", outPath.c_str(), isaName( isa ) );
	printf( "  sources      %zu bytes (6 files)\n", before );
	printf( "  output       %zu bytes, %zu lines, longest %zu\n", result.size(), lines - 1, longest );
	printf( "  ratio        %.1f%% of the original\n", 100.0 * (double)result.size() / (double)before );
	printf( "  renamed      %zu locals, %zu references, %zu bytes\n", rs.vars, rs.occ, rs.chars );
	printf( "  internals    %zu names, %zu bytes; %zu prototype parameter names dropped\n",
		memberCount, memberSaved, protoNames );
	printf( "  own macros   %zu renamed\n", macroRenamed );
	printf( "  blocks       %zu braces removed, %zu statements comma-joined, %zu loops\n",
		simp.blocks, simp.folds, simp.loops );
	if (getenv( "MICROBVH_DEBUG" ))
	{
		fprintf( stderr, "blocks considered: %zu\n", simp.tried );
		for (auto& w : simp.why) fprintf( stderr, "  %-8s %zu\n", w.first.c_str(), w.second );
	}
	printf( "  constants    %zu float, %zu integer, %zu bool rewritten, %zu bytes\n",
		lit.floats, lit.ints, lit.bools, lit.bytes );
	printf( "  const        %zu dropped, %zu declarations merged\n", constDropped, declMerged );
	printf( "  macros       %zu generated (%zu with a parameter)\n", defs.size(), paramMade );
	return 0;
}
