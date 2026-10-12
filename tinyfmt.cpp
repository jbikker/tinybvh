// tinyfmt.cpp - applies the tinybvh coding standard to the tinybvh headers.
//
// Build:  g++ -O2 -std=c++17 tinyfmt.cpp -o tinyfmt
//         cl /O2 /EHsc /std:c++17 tinyfmt.cpp
//
// Usage:  tinyfmt [--check] [--verbose] [file.h ...]
//         Without file arguments, the six tinybvh headers in the current folder are processed.
//         --check      report only, write nothing; exit code 1 if any file would change.
//         --verbose    print every changed line (before / after).
//
// Rules, measured on the tinybvh headers (dominant form vs. exceptions):
//   - CRLF line endings, including after the last line (so git sees a properly terminated file);
//     no UTF-8 BOM, no trailing whitespace.
//   - Indentation with tabs (tab = 4 columns); leading spaces are converted.
//   - At most one consecutive blank line; no blank lines at the start or end of a file.
//   - Calls and declarations pad their parentheses: Foo( a, b ), sizeof( T ), [&]( int i ), Foo<T>( x ).
//     Empty argument lists are tight: Foo().
//   - Control statements: one space after the keyword, no padding inside: if (a), for (i = 0; i < n; i++);
//     an empty last for-clause keeps its space: for (i = n; i-- > 0; ), but for (;;).
//   - Expression parentheses and casts are left alone: (a + b), (float*)p, static_cast<T>(x), alignas(16).
//   - Brackets are tight: a[i]. Braces of one-liners are padded: { return x; }.
//   - A space follows every comma and semicolon (compact numeric tables such as { 0,1,2 } are kept).
//   - Spaces around = == != <= >= && || and compound assignments (operator declarations excluded).
//   - Allman braces: a '{' ending a function / control / lambda / class header goes on its own line;
//     "} else" is split. Namespaces, initializers ("= {") and constructor init lists keep '{' inline.
//   - A block whose '{' is indented deeper than its header is shifted back to the header's level, so
//     lambda bodies look like normal scopes, and Visual Studio's extra indent after "if (x) ISLIKELY"
//     is undone.
//   - #else, #elif and #endif are aligned with their matching #if.
//   - The line after a _Pragma( ... ) line (no semicolon) keeps the pragma's indentation, undoing
//     Visual Studio's continuation indent, so consecutive _Pragma lines line up.
// Comments, string/char literals (including raw strings) and preprocessor lines are never modified,
// apart from line endings, trailing whitespace and leading indentation. Multiple spaces are never
// collapsed, so tab/space alignment of columns and trailing comments survives.
// Safety: a file is only written when its token stream (code without whitespace and comments, plus
// all literals) is unchanged, and when formatting the result again is a no-op.

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

const int TAB_SIZE = 4;
const size_t npos = std::string::npos;

const char* const defaultFiles[] = {
	"tiny_bvh.h", "tiny_bvh_base.h", "tiny_bvh_x86_float.h",
	"tiny_bvh_x86_double.h", "tiny_bvh_arm_float.h", "tiny_bvh_arm_double.h"
};
const char* const ctrlWords[] = { "if", "for", "while", "switch", "catch", 0 };
const char* const noPadWords[] = { "alignas", "alignof", "decltype", "noexcept", "__attribute__", "__declspec",
	"defined", "return", "throw", "case", "new", "delete", "co_return", "co_yield", "co_await", "requires", 0 };
const char* const braceWords[] = { "else", "do", "try", "const", "noexcept", "override", "final", "mutable",
	"ISLIKELY", "ISUNLIKELY", 0 };
const char* const classKeys[] = { "struct", "class", "union", "enum", 0 };
const char* const opTokens[] = { "<<=", ">>=", "<=>", "->*", "==", "!=", "<=", ">=", "&&", "||", "+=", "-=",
	"*=", "/=", "%=", "&=", "|=", "^=", "<<", ">>", "++", "--", "->", 0 };
const char* const spacedOps[] = { "=", "==", "!=", "<=", ">=", "&&", "||", "+=", "-=", "*=", "/=", "%=",
	"&=", "|=", "^=", "<<=", ">>=", 0 };

bool IsWs( char c ) { return c == ' ' || c == '\t'; }
bool IsDigit( char c ) { return c >= '0' && c <= '9'; }
bool IsIdent( char c ) { return isalnum( (unsigned char)c ) || c == '_'; }
bool OnlyWs( const std::string& s ) { return s.find_first_not_of( " \t" ) == npos; }
bool InList( const std::string& w, const char* const* list )
{
	for (; *list; list++) if (w == *list) return true;
	return false;
}
bool EndsWith( const std::string& s, const char* t )
{
	const size_t n = strlen( t );
	return s.size() >= n && s.compare( s.size() - n, n, t ) == 0;
}
std::string WordBefore( const std::string& s, size_t end )
{
	size_t b = end;
	while (b > 0 && IsIdent( s[b - 1] )) b--;
	return s.substr( b, end - b );
}
bool EndsWithWord( const std::string& s, const char* w )
{
	const size_t n = strlen( w );
	return EndsWith( s, w ) && (s.size() == n || !IsIdent( s[s.size() - n - 1] ));
}

// Visual columns of the leading whitespace of s; a receives its length in characters.
int LeadCols( const std::string& s, size_t& a )
{
	int col = 0;
	for (a = 0; a < s.size() && IsWs( s[a] ); a++) col = s[a] == '\t' ? (col / TAB_SIZE + 1) * TAB_SIZE : col + 1;
	return col;
}
std::string MakeIndent( int col ) { return std::string( col / TAB_SIZE, '\t' ) + std::string( col % TAB_SIZE, ' ' ); }

// True if code in s[a, e) contains a lambda introducer followed by a parameter list: [&]( ... ) -> T
bool HasLambda( const std::string& s, const std::string& m, size_t a, size_t e )
{
	for (size_t q = a; q + 1 < e; q++) if (m[q] == 'c' && s[q] == ']' && s[q + 1] == '(') return true;
	return false;
}

// True if code in s[a, e) names a class key (struct, class, union, enum) and contains no assignment.
bool IsClassHeader( const std::string& s, const std::string& m, size_t a, size_t e )
{
	bool classKey = false;
	for (size_t q = a; q < e; q++)
	{
		if (m[q] != 'c') continue;
		if (s[q] == '=') return false;
		if (IsIdent( s[q] ) && (q == 0 || !IsIdent( s[q - 1] )))
		{
			size_t w = q;
			while (w < e && IsIdent( s[w] )) w++;
			if (InList( s.substr( q, w - q ), classKeys )) classKey = true;
		}
	}
	return classKey;
}

std::vector<std::string> SplitLines( const std::string& t )
{
	std::vector<std::string> lines;
	std::string cur;
	for (size_t i = 0; i < t.size(); i++)
	{
		if (t[i] == '\r' || t[i] == '\n')
		{
			lines.push_back( cur ), cur.clear();
			if (t[i] == '\r' && i + 1 < t.size() && t[i + 1] == '\n') i++;
		}
		else cur += t[i];
	}
	lines.push_back( cur ); // empty if the file ends with a line break
	return lines;
}

// ----------------------------------------------------------------------------
// Lexer: classifies every character of a line as code ('c'), comment ('m') or
// literal ('s'), carrying block comments, raw strings and continued string /
// preprocessor lines over to the next line.
// ----------------------------------------------------------------------------

struct LexState
{
	enum Mode { CODE, BLOCK, STRING, RAW } mode = CODE;
	char quote = 0;
	std::string rawEnd; // )delim"
	bool ppCont = false;
};

struct LexLine
{
	std::string mask;
	bool pp = false, startsInLit = false, endsInLit = false;
};

bool IsDigitSeparator( const std::string& l, size_t i )
{
	// C++14 digit separator: a quote inside a numeric literal such as 1'000'000.
	size_t s = i;
	while (s > 0 && (IsIdent( l[s - 1] ) || l[s - 1] == '\'')) s--;
	return s < i && IsDigit( l[s] );
}

bool IsRawStringStart( const std::string& l, size_t q )
{
	if (q == 0 || l[q - 1] != 'R') return false;
	size_t s = q - 1;
	if (s >= 2 && l.compare( s - 2, 2, "u8" ) == 0) s -= 2;
	else if (s >= 1 && strchr( "uUL", l[s - 1] )) s -= 1;
	return s == 0 || !IsIdent( l[s - 1] );
}

LexLine Lex( const std::string& l, LexState& st )
{
	LexLine r;
	const size_t n = l.size();
	r.mask.assign( n, 'c' );
	r.startsInLit = st.mode == LexState::STRING || st.mode == LexState::RAW;
	if (st.ppCont) r.pp = true;
	else if (st.mode == LexState::CODE)
	{
		const size_t f = l.find_first_not_of( " \t" );
		r.pp = f != npos && l[f] == '#';
	}
	size_t i = 0;
	while (i < n)
	{
		if (st.mode == LexState::BLOCK)
		{
			const size_t e = l.find( "*/", i ), end = e == npos ? n : e + 2;
			r.mask.replace( i, end - i, end - i, 'm' );
			if (e != npos) st.mode = LexState::CODE;
			i = end;
			continue;
		}
		if (st.mode == LexState::RAW)
		{
			const size_t e = l.find( st.rawEnd, i ), end = e == npos ? n : e + st.rawEnd.size();
			r.mask.replace( i, end - i, end - i, 's' );
			if (e != npos) st.mode = LexState::CODE;
			i = end;
			continue;
		}
		if (st.mode == LexState::STRING)
		{
			size_t j = i;
			while (j < n)
			{
				if (l[j] == '\\') j += 2;
				else if (l[j++] == st.quote) { st.mode = LexState::CODE; break; }
			}
			if (j > n) j = n;
			r.mask.replace( i, j - i, j - i, 's' );
			i = j;
			continue;
		}
		const char c = l[i];
		if (c == '/' && i + 1 < n && l[i + 1] == '/')
		{
			r.mask.replace( i, n - i, n - i, 'm' );
			break;
		}
		if (c == '/' && i + 1 < n && l[i + 1] == '*')
		{
			r.mask[i] = r.mask[i + 1] = 'm', st.mode = LexState::BLOCK, i += 2;
			continue;
		}
		if (c == '"' && IsRawStringStart( l, i ))
		{
			const size_t p = l.find( '(', i + 1 );
			if (p != npos && p - i - 1 <= 16)
			{
				st.rawEnd = ")" + l.substr( i + 1, p - i - 1 ) + "\"";
				st.mode = LexState::RAW;
				r.mask.replace( i, p + 1 - i, p + 1 - i, 's' );
				i = p + 1;
				continue;
			}
		}
		if (c == '"' || (c == '\'' && !IsDigitSeparator( l, i )))
		{
			st.mode = LexState::STRING, st.quote = c, r.mask[i] = 's', i++;
			continue;
		}
		i++;
	}
	const size_t last = l.find_last_not_of( " \t" );
	const bool continued = last != npos && l[last] == '\\';
	if (st.mode == LexState::STRING && !continued) st.mode = LexState::CODE; // unterminated: don't leak
	r.endsInLit = st.mode == LexState::STRING || st.mode == LexState::RAW;
	st.ppCont = r.pp && continued;
	return r;
}

// Token stream used for the safety check: code without whitespace and comments, plus literals.
std::string Canonical( const std::string& text )
{
	std::string r;
	LexState st;
	for (const std::string& l : SplitLines( text ))
	{
		const LexLine lx = Lex( l, st );
		for (size_t i = 0; i < l.size(); i++)
			if (lx.mask[i] == 's' || (lx.mask[i] == 'c' && !IsWs( l[i] ))) r += l[i];
		if (lx.endsInLit) r += '\n';
	}
	return r;
}

// ----------------------------------------------------------------------------
// Formatter
// ----------------------------------------------------------------------------

enum : char { P_OTHER, P_CALL, P_CTRL, P_FOR };

struct OutLine
{
	std::string text;
	bool keep;  // starts inside a literal: never treat as a removable blank line
	size_t src; // source line index, for reporting
};

class Formatter
{
public:
	bool finalEol = true;
	std::string Run( const std::string& text, std::vector<OutLine>* report = 0 );
private:
	std::vector<char> parens;               // kinds of the currently open parentheses
	std::vector<std::vector<char>> ppStack; // paren state at each #if, restored at #else / #elif
	std::vector<std::string> ppIndent;      // indentation of each open #if
	std::string o, om;                      // output line under construction and its classes
	void Put( char c, char k = 'c' ) { o += c, om += k; }
	void TrimRight() { while (!o.empty() && IsWs( o.back() )) o.pop_back(), om.pop_back(); }
	char ClassifyOpen() const;
	void Directive( std::string& l );
	void FormatCode( const std::string& l, const std::string& mask );
	void Finish( size_t src, bool keep, std::vector<OutLine>& out );
};

// Tracks #if nesting: restores the paren state per branch, and aligns #else / #elif / #endif
// with their #if.
void Formatter::Directive( std::string& l )
{
	size_t a = l.find( '#' ) + 1;
	while (a < l.size() && IsWs( l[a] )) a++;
	size_t b = a;
	while (b < l.size() && IsIdent( l[b] )) b++;
	const std::string w = l.substr( a, b - a );
	const size_t hash = l.find( '#' );
	if (w == "if" || w == "ifdef" || w == "ifndef")
	{
		ppStack.push_back( parens );
		ppIndent.push_back( l.substr( 0, hash ) );
		return;
	}
	const bool branch = w.compare( 0, 4, "elif" ) == 0 || w == "else", end = w == "endif";
	if (!branch && !end) return;
	if (!ppIndent.empty()) l = ppIndent.back() + l.substr( hash );
	if (branch && !ppStack.empty()) parens = ppStack.back();
	if (end && !ppStack.empty()) ppStack.pop_back(), ppIndent.pop_back();
}

// Decide how the '(' about to be emitted is formatted, based on what precedes it.
char Formatter::ClassifyOpen() const
{
	const std::string& s = o;
	size_t t = s.size();
	while (t > 0 && IsWs( s[t - 1] )) t--;
	if (t == 0) return P_OTHER;
	const bool gap = t != s.size();
	const char last = s[t - 1];
	if (IsIdent( last ))
	{
		const std::string w = WordBefore( s, t );
		if (w == "for") return P_FOR;
		if (InList( w, ctrlWords )) return P_CTRL;
		if (w == "constexpr")
		{
			size_t b = t - w.size();
			while (b > 0 && IsWs( s[b - 1] )) b--;
			if (WordBefore( s, b ) == "if") return P_CTRL;
		}
		if (gap || IsDigit( w[0] ) || InList( w, noPadWords )) return P_OTHER;
		return P_CALL;
	}
	// operator declarations, with or without a gap: operator==( ... ), operator [] ( ... ), operator()( ... )
	size_t b = t;
	while (b > 0 && strchr( "=+-*/%^&|<>!~,[]()", s[b - 1] )) b--;
	while (b > 0 && IsWs( s[b - 1] )) b--;
	if (b < t && WordBefore( s, b ) == "operator") return P_CALL;
	if (gap) return P_OTHER;
	if (last == ']') return P_CALL; // lambda
	if (last == ')') return P_OTHER; // function pointers: (*fn)(x)
	if (last == '>' && (t < 2 || s[t - 2] != '-'))
	{
		// template call Foo<T>( x ); casts keep their tight form: static_cast<T>(x)
		int depth = 0;
		size_t k = t;
		while (k > 0)
		{
			const char ch = s[--k];
			if (ch == '>') depth++;
			else if (ch == '<' && --depth == 0) break;
			else if (strchr( ";{}()", ch )) return P_OTHER;
		}
		if (depth != 0) return P_OTHER;
		const std::string w = WordBefore( s, k );
		if (w.empty() || IsDigit( w[0] ) || EndsWith( w, "_cast" ) || w == "template") return P_OTHER;
		return P_CALL;
	}
	return P_OTHER;
}

void Formatter::FormatCode( const std::string& l, const std::string& mask )
{
	const size_t n = l.size();
	size_t i = 0;
	while (i < n)
	{
		const char c = l[i];
		if (mask[i] != 'c' || IsWs( c ))
		{
			Put( c, mask[i] ), i++;
			continue;
		}
		const char next = i + 1 < n ? l[i + 1] : 0;
		if (c == '(')
		{
			const char kind = ClassifyOpen();
			size_t j = i + 1;
			while (j < n && IsWs( l[j] )) j++;
			if (kind == P_CTRL || kind == P_FOR)
			{
				TrimRight(), Put( ' ' ), Put( '(' );
				i = j < n ? j : i + 1; // no padding inside a condition
			}
			else if (kind == P_CALL && j < n && l[j] == ')' && mask[j] == 'c')
			{
				Put( '(' ), Put( ')' ), i = j + 1; // empty argument list
				continue;
			}
			else
			{
				Put( '(' ), i++;
				if (kind == P_CALL && next && !IsWs( next )) Put( ' ' );
			}
			parens.push_back( kind );
			continue;
		}
		if (c == ')')
		{
			const char kind = parens.empty() ? (char)P_OTHER : parens.back();
			if (!parens.empty()) parens.pop_back();
			if (!OnlyWs( o ))
			{
				if (kind == P_CALL && !IsWs( o.back() ) && o.back() != '(') Put( ' ' );
				if (kind == P_CTRL || kind == P_FOR) TrimRight();
				if (kind == P_FOR && o.back() == ';' && !EndsWith( o, "(;;" )) Put( ' ' ); // for (i = n; i-- > 0; )
			}
			Put( ')' ), i++;
			continue;
		}
		if (c == '[')
		{
			Put( '[' ), i++;
			size_t j = i;
			while (j < n && IsWs( l[j] )) j++;
			if (j < n) i = j;
			continue;
		}
		if (c == ']')
		{
			if (!OnlyWs( o )) TrimRight();
			Put( ']' ), i++;
			continue;
		}
		if (c == '{')
		{
			Put( '{' ), i++;
			if (next && !IsWs( next ) && next != '}') Put( ' ' );
			continue;
		}
		if (c == '}')
		{
			if (!OnlyWs( o ) && !IsWs( o.back() ) && o.back() != '{') Put( ' ' );
			Put( '}' ), i++;
			continue;
		}
		if (c == ',')
		{
			const bool table = !o.empty() && IsDigit( o.back() ) && IsDigit( next ); // { 0,1,2 } stays compact
			Put( ',' ), i++;
			if (next && !IsWs( next ) && !table) Put( ' ' );
			continue;
		}
		if (c == ';')
		{
			Put( ';' ), i++;
			if (next && !IsWs( next ) && next != ';' && next != ')') Put( ' ' );
			continue;
		}
		if (strchr( "=!<>&|+-*/%^", c ))
		{
			std::string tok( 1, c );
			for (const char* const* t = opTokens; *t; t++)
			{
				const size_t len = strlen( *t );
				if (l.compare( i, len, *t ) == 0 && mask.compare( i, len, std::string( len, 'c' ) ) == 0)
				{
					tok = *t;
					break;
				}
			}
			const size_t e = i + tok.size();
			std::string before = o;
			while (!before.empty() && IsWs( before.back() )) before.pop_back();
			const bool sb = OnlyWs( o ) || IsWs( o.back() ), sa = e >= n || IsWs( l[e] );
			bool spaced = InList( tok, spacedOps ) && !EndsWithWord( before, "operator" );
			if (spaced && tok == "&&") // binary only; leave rvalue references (T&&) alone
				spaced = !sb && !sa && (IsIdent( o.back() ) || o.back() == ')' || o.back() == ']') &&
				(IsIdent( l[e] ) || l[e] == '(' || l[e] == '!');
			if (spaced && tok == "=")
			{
				size_t f = e;
				while (f < n && IsWs( l[f] )) f++;
				if ((!before.empty() && before.back() == '[') || (f < n && l[f] == ']')) spaced = false; // [=]
			}
			if (spaced && !sb) Put( ' ' );
			for (const char ch : tok) Put( ch );
			if (spaced && !sa) Put( ' ' );
			i = e;
			continue;
		}
		Put( c ), i++;
	}
}

void Formatter::Finish( size_t src, bool keep, std::vector<OutLine>& out )
{
	std::string s = o, m = om;
	const size_t a = s.find_first_not_of( " \t" );
	if (keep || a == npos)
	{
		out.push_back( { keep ? s : std::string(), keep, src } );
		return;
	}
	const std::string indent = s.substr( 0, a );
	// "} else" -> "}" / "else"
	if (s[a] == '}' && m[a] == 'c')
	{
		size_t b = a + 1;
		while (b < s.size() && IsWs( s[b] )) b++;
		if (s.compare( b, 4, "else" ) == 0 && m[b] == 'c' && (b + 4 == s.size() || !IsIdent( s[b + 4] )))
		{
			out.push_back( { indent + "}", false, src } );
			s = indent + s.substr( b ), m = m.substr( 0, a ) + m.substr( b );
		}
	}
	// Allman: a '{' that ends a function / control / class header moves to its own line.
	size_t k = npos;
	for (size_t q = 0; q < s.size(); q++) if (m[q] == 'c' && !IsWs( s[q] )) k = q;
	if (k != npos && k > a && s[k] == '{')
	{
		size_t p = k;
		while (p > a && IsWs( s[p - 1] )) p--;
		const std::string tail = s.substr( k + 1 ); // whitespace and/or a comment
		const size_t tc = tail.find_first_not_of( " \t" );
		const bool tailOk = tc == npos || tail.compare( tc, 2, "//" ) == 0;
		size_t fw = a;
		while (fw < s.size() && IsIdent( s[fw] )) fw++;
		const std::string first = s.substr( a, fw - a ), w = WordBefore( s, p );
		bool move = s[p - 1] == ')' || s[p - 1] == ']' || InList( w, braceWords ) || HasLambda( s, m, a, p );
		if (!move && !strchr( "=,({[", s[p - 1] )) move = IsClassHeader( s, m, a, p );
		if (first == "namespace" || first == "extern") move = false;
		if (move && tailOk)
		{
			std::string head = s.substr( 0, p );
			if (tc != npos) head += IsWs( tail[0] ) ? tail : " " + tail;
			out.push_back( { head, false, src } );
			out.push_back( { indent + "{", false, src } );
			return;
		}
	}
	out.push_back( { s, false, src } );
}

// True if the code line s can own a block that opens on the next line.
bool IsBlockHeader( const std::string& s, const std::string& m )
{
	size_t k = npos;
	for (size_t q = 0; q < s.size(); q++) if (m[q] == 'c' && !IsWs( s[q] )) k = q;
	if (k == npos) return false;
	if (s[k] == ')' || s[k] == ']') return true;
	if (InList( WordBefore( s, k + 1 ), braceWords )) return true; // else, do, ISLIKELY, ...
	const size_t a = s.find_first_not_of( " \t" );
	if (strchr( "=,({[", s[k] )) return false;
	return HasLambda( s, m, a, k + 1 ) || IsClassHeader( s, m, a, k + 1 );
}

// Remove up to 'delta' columns of indentation from lines [from, to], never going below column 'floor'.
// Code lines strictly between 'open' and 'close' stay at least at column 'bodyMin'.
void Shift( std::vector<OutLine>& lines, std::vector<std::string>& mask, const std::vector<char>& lit,
	size_t from, size_t to, int delta, int floor, const std::vector<char>& pp, size_t open, size_t close, int bodyMin )
{
	for (size_t k = from; k <= to; k++)
	{
		std::string& t = lines[k].text;
		size_t la;
		const int col = LeadCols( t, la );
		int target = col - std::min( delta, std::max( 0, col - floor ) );
		if (k > open && k < close && !pp[k]) target = std::max( target, bodyMin );
		if (lit[k] || la == t.size() || target == col) continue;
		const std::string ind = MakeIndent( target );
		t = ind + t.substr( la ), mask[k] = std::string( ind.size(), mask[k][0] ) + mask[k].substr( la );
	}
}

// A _Pragma( ... ) line has no semicolon, so Visual Studio indents the next line as a continuation.
// The next code line (and comment lines before it) is pulled back to the pragma's indentation.
void FixPragmaIndent( std::vector<OutLine>& lines )
{
	const size_t n = lines.size();
	std::vector<std::string> mask( n );
	std::vector<char> pp( n ), lit( n );
	LexState st;
	for (size_t i = 0; i < n; i++)
	{
		const LexLine lx = Lex( lines[i].text, st );
		mask[i] = lx.mask, pp[i] = lx.pp, lit[i] = lx.startsInLit || lines[i].keep;
	}
	int pragmaCol = -1; // indentation of the preceding unterminated _Pragma line, or -1
	for (size_t i = 0; i < n; i++)
	{
		std::string& t = lines[i].text;
		const size_t a = t.find_first_not_of( " \t" );
		if (lit[i]) { pragmaCol = -1; continue; }
		if (pp[i] || a == npos) continue;
		size_t la;
		const int col = LeadCols( t, la );
		if (pragmaCol >= 0 && col > pragmaCol)
		{
			const std::string ind = MakeIndent( pragmaCol );
			t = ind + t.substr( la ), mask[i] = std::string( ind.size(), mask[i][0] ) + mask[i].substr( la );
		}
		if (mask[i][a] == 'm') continue; // comment-only line: keep looking for the next code line
		// is this line itself an unterminated _Pragma( ... )?
		const size_t b = t.find_first_not_of( " \t" );
		size_t k = npos;
		int depth = 0;
		for (size_t q = b; q < t.size(); q++) if (mask[i][q] == 'c' && !IsWs( t[q] ))
		{
			k = q;
			if (t[q] == '(') depth++; else if (t[q] == ')') depth--;
		}
		const bool pragma = t.compare( b, 7, "_Pragma" ) == 0 && (b + 7 == t.size() || !IsIdent( t[b + 7] ));
		pragmaCol = pragma && k != npos && t[k] == ')' && depth == 0 ? LeadCols( t, la ) : -1;
	}
}

// A block whose '{' is indented deeper than its header (lambdas, or Visual Studio's indenting after
// "if (x) ISLIKELY") is shifted back so that it looks like a normal scope.
void FixBlockIndent( std::vector<OutLine>& lines )
{
	const size_t n = lines.size();
	std::vector<std::string> mask( n );
	std::vector<char> pp( n ), lit( n );
	LexState st;
	for (size_t i = 0; i < n; i++)
	{
		const LexLine lx = Lex( lines[i].text, st );
		mask[i] = lx.mask, pp[i] = lx.pp, lit[i] = lx.startsInLit || lines[i].keep;
	}
	size_t header = npos; // last code line
	for (size_t i = 0; i < n; i++)
	{
		const std::string& s = lines[i].text;
		const size_t a = s.find_first_not_of( " \t" );
		if (lit[i] || pp[i] || a == npos || mask[i][a] == 'm') continue;
		if (s.compare( a, npos, "{" ) == 0)
		{
			// find the matching '}'
			size_t close = npos;
			for (size_t k = i, depth = 0; k < n && close == npos; k++)
			{
				if (pp[k]) continue;
				for (size_t q = 0; q < lines[k].text.size(); q++) if (mask[k][q] == 'c')
				{
					if (lines[k].text[q] == '{') depth++;
					else if (lines[k].text[q] == '}' && --depth == 0) { close = k; break; }
				}
			}
			if (close == npos) { header = i; continue; }
			size_t la;
			int braceCol = LeadCols( s, la );
			// 1. '{' deeper than its header: shift the whole block back to the header's level.
			if (header != npos && IsBlockHeader( lines[header].text, mask[header] ))
			{
				const int hc = LeadCols( lines[header].text, la ), delta = braceCol - hc;
				if (delta > 0)
				{
					Shift( lines, mask, lit, i, close, delta, hc, pp, i, close, hc + TAB_SIZE );
					braceCol = hc;
				}
			}
			// 2. body more than one level deeper than its '{': shift the body back.
			size_t first = i + 1;
			while (first < close && (pp[first] || OnlyWs( lines[first].text ))) first++;
			if (first < close && !lit[first])
			{
				const int excess = LeadCols( lines[first].text, la ) - (braceCol + TAB_SIZE);
				if (excess > 0) Shift( lines, mask, lit, i + 1, close - 1, excess, braceCol + TAB_SIZE, pp, i, close, 0 );
			}
			// 3. a '}' that starts its line aligns with its '{'.
			const size_t ca = lines[close].text.find_first_not_of( " \t" );
			if (close > i && !lit[close] && lines[close].text[ca] == '}' && LeadCols( lines[close].text, la ) > braceCol)
				Shift( lines, mask, lit, close, close, 1 << 20, braceCol, pp, i, close, 0 );
		}
		header = i;
	}
}

std::string Formatter::Run( const std::string& input, std::vector<OutLine>* report )
{
	std::string text = input;
	if (text.compare( 0, 3, "\xEF\xBB\xBF" ) == 0) text.erase( 0, 3 );
	const std::vector<std::string> src = SplitLines( text );
	std::vector<OutLine> out;
	LexState st;
	parens.clear(), ppStack.clear(), ppIndent.clear();
	for (size_t li = 0; li < src.size(); li++)
	{
		const bool continuation = st.ppCont;
		const LexLine lx = Lex( src[li], st );
		std::string l = src[li], mask = lx.mask;
		if (!lx.endsInLit)
		{
			size_t e = l.size();
			while (e > 0 && IsWs( l[e - 1] )) e--;
			l.resize( e ), mask.resize( e );
		}
		if (!lx.startsInLit)
		{
			size_t a = 0;
			int col = 0;
			while (a < l.size() && IsWs( l[a] )) col = l[a++] == '\t' ? (col / TAB_SIZE + 1) * TAB_SIZE : col + 1;
			std::string ind( col / TAB_SIZE, '\t' );
			ind.append( col % TAB_SIZE, ' ' );
			if (a == l.size()) l.clear(), mask.clear();
			else if (ind != l.substr( 0, a ))
			{
				const char k = mask[0];
				l = ind + l.substr( a ), mask = std::string( ind.size(), k ) + mask.substr( a );
			}
		}
		if (lx.pp)
		{
			if (!continuation) Directive( l );
			out.push_back( { l, false, li } );
			continue;
		}
		o.clear(), om.clear();
		FormatCode( l, mask );
		Finish( li, lx.startsInLit, out );
	}
	std::vector<OutLine> res;
	for (const OutLine& ln : out)
	{
		const bool blank = ln.text.empty() && !ln.keep;
		if (blank && (res.empty() || (res.back().text.empty() && !res.back().keep))) continue;
		res.push_back( ln );
	}
	while (!res.empty() && res.back().text.empty() && !res.back().keep) res.pop_back();
	FixPragmaIndent( res );
	FixBlockIndent( res );
	std::string r;
	for (size_t i = 0; i < res.size(); i++) r += (i ? "\r\n" : "") + res[i].text;
	if (finalEol && !res.empty()) r += "\r\n";
	if (report) *report = res;
	return r;
}

// ----------------------------------------------------------------------------
// File handling
// ----------------------------------------------------------------------------

bool ReadFile( const std::string& path, std::string& data )
{
	std::ifstream f( path, std::ios::binary );
	if (!f) return false;
	std::stringstream ss;
	ss << f.rdbuf();
	data = ss.str();
	return true;
}

bool WriteFile( const std::string& path, const std::string& data )
{
	const std::string tmp = path + ".tinyfmt.tmp";
	{
		std::ofstream f( tmp, std::ios::binary | std::ios::trunc );
		if (!f || !f.write( data.data(), (std::streamsize)data.size() )) return false;
	}
	std::error_code ec;
	std::filesystem::rename( tmp, path, ec );
	if (ec) std::filesystem::remove( tmp, ec );
	return !ec;
}

const char* usage =
	"tinyfmt - applies the tinybvh coding standard\n"
	"usage: tinyfmt [--check] [--verbose] [file.h ...]\n"
	"  no files     process the six tinybvh headers in the current folder\n"
	"  --check      write nothing; exit code 1 if any file would change\n"
	"  --verbose    print every changed line\n";

} // namespace

int main( int argc, char** argv )
{
	bool check = false, verbose = false;
	const bool finalEol = true;
	std::vector<std::string> files;
	for (int i = 1; i < argc; i++)
	{
		const std::string arg = argv[i];
		if (arg == "--check") check = true;
		else if (arg == "--verbose" || arg == "-v") verbose = true;
		else if (arg == "--help" || arg == "-h") { fputs( usage, stdout ); return 0; }
		else if (arg[0] == '-') { fprintf( stderr, "unknown option %s\n%s", arg.c_str(), usage ); return 2; }
		else files.push_back( arg );
	}
	const bool useDefaults = files.empty();
	if (useDefaults) for (const char* f : defaultFiles) files.push_back( f );
	int changedFiles = 0, errors = 0, found = 0;
	for (const std::string& path : files)
	{
		std::string input;
		if (!ReadFile( path, input ))
		{
			fprintf( stderr, "%s: cannot read%s\n", path.c_str(), useDefaults ? ", skipped" : "" );
			if (!useDefaults) errors++;
			continue;
		}
		found++;
		std::string text = input;
		if (text.compare( 0, 3, "\xEF\xBB\xBF" ) == 0) text.erase( 0, 3 );
		Formatter fmt;
		fmt.finalEol = finalEol;
		std::vector<OutLine> lines;
		const std::string result = fmt.Run( text, &lines );
		if (result == input)
		{
			printf( "%s: ok\n", path.c_str() );
			continue;
		}
		if (Canonical( result ) != Canonical( text ))
		{
			fprintf( stderr, "%s: ERROR: formatting would change the token stream; file left untouched\n", path.c_str() );
			errors++;
			continue;
		}
		Formatter again;
		again.finalEol = finalEol;
		if (again.Run( result ) != result)
		{
			fprintf( stderr, "%s: ERROR: formatting is not stable; file left untouched\n", path.c_str() );
			errors++;
			continue;
		}
		// report per source line
		const std::vector<std::string> src = SplitLines( text );
		std::vector<std::vector<std::string>> bySrc( src.size() );
		for (const OutLine& ln : lines) bySrc[ln.src].push_back( ln.text );
		int changedLines = 0;
		for (size_t i = 0; i < src.size(); i++)
		{
			if (bySrc[i].size() == 1 && bySrc[i][0] == src[i]) continue;
			if (i + 1 == src.size() && src[i].empty() && bySrc[i].empty()) continue; // final line break
			changedLines++;
			if (!verbose) continue;
			printf( "%s:%zu:\n  - %s\n", path.c_str(), i + 1, src[i].c_str() );
			for (const std::string& t : bySrc[i]) printf( "  + %s\n", t.c_str() );
		}
		std::string extra;
		if (input.compare( 0, 3, "\xEF\xBB\xBF" ) == 0) extra += ", BOM removed";
		bool pureCrlf = true;
		for (size_t i = 0; i < text.size(); i++)
		{
			if (text[i] == '\n' && (i == 0 || text[i - 1] != '\r')) pureCrlf = false;
			if (text[i] == '\r' && (i + 1 == text.size() || text[i + 1] != '\n')) pureCrlf = false;
		}
		if (!pureCrlf) extra += ", line endings -> CRLF";
		const bool hadEol = !text.empty() && (text.back() == '\n' || text.back() == '\r');
		if (hadEol != finalEol) extra += finalEol ? ", final line break added" : ", final line break removed";
		changedFiles++;
		if (check) printf( "%s: %d line(s) need fixing%s\n", path.c_str(), changedLines, extra.c_str() );
		else if (WriteFile( path, result )) printf( "%s: %d line(s) fixed%s\n", path.c_str(), changedLines, extra.c_str() );
		else
		{
			fprintf( stderr, "%s: ERROR: cannot write\n", path.c_str() );
			errors++;
		}
	}
	if (useDefaults && found == 0)
	{
		fprintf( stderr, "no tinybvh headers found; run tinyfmt in the tinybvh folder\n" );
		return 2;
	}
	if (errors) return 2;
	return check && changedFiles ? 1 : 0;
}
