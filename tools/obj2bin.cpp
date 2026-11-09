// obj2bin.cpp - converts Wavefront .obj geometry to the TinyBVH triangle blob,
// with a best-guess 24-bit vertex colour derived from the .mtl materials.
//
// File layout:
//   uint32_t triangleCount
//   triangleCount * 3 * { float x, y, z; uint32_t rgb; }
//
// i.e. 48 bytes per triangle, matching bvhvec4 vert0/vert1/vert2 as consumed by
// BVH::Build( const bvhvec4*, uint32_t ). The .w field holds 0x00RRGGBB.
//
// This version never holds the .obj or the triangle soup in memory: the input
// is read through a 1 MB sliding window (grown only if a single line does not
// fit) and triangles are streamed to the output through a small write cache,
// with the triangle count patched into the header on close. What is left in
// memory is only what faces can refer back to: the positions and texcoords,
// held in a ChunkedArray of fixed 1M-element blocks so the store never copies
// or reallocates itself as it grows - 12 bytes per 'v' line and 8 per 'vt',
// independent of file size. Offsets are 64-bit throughout, so files beyond
// 2 GB are fine on MSVC as well.
//
// Colour estimate, in order of decreasing confidence:
//   1. material has map_Kd and the corner has a 'vt': bilinear texel * Kd.
//   2. material has map_Kd but no 'vt' (or the texture failed to load): the
//      average colour of the texture * Kd.
//   3. material has Kd (or, if Kd is black, Ke; or, if neither, Ka): that.
//   4. no material / no .mtl found: white (0xffffff).
//
// Note that Kd is nominally linear while texels are sRGB bytes; they are simply
// multiplied here, which is the usual preview convention and is what you want
// for a colour you will display directly.
//
// Image decoding: TGA (24/32, raw + RLE), BMP (24/32, uncompressed) and PPM/P6
// are decoded natively. Drop a copy of stb_image.h next to this file (or in
// external/) and PNG, JPG and the rest are picked up automatically. Compile
// with -DNO_STB_IMAGE to force the built-in decoders.
//
// Textures are looked up next to the .obj/.mtl and in a 'textures' subdir.
// Materials are otherwise ignored; this remains a pure triangle soup exporter.
// N-gons are fan-triangulated, degenerate and out-of-range faces are dropped.
//
// Build:
//   g++ -O2 -D_FILE_OFFSET_BITS=64 -o obj2bin obj2bin.cpp
//   clang++ -O2 -D_FILE_OFFSET_BITS=64 -o obj2bin obj2bin.cpp
//   cl /O2 /EHsc obj2bin.cpp
//
// Usage:
//   obj2bin input.obj [output.bin] [--no-textures]

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <cmath>
#include <string>
#include <vector>

// Optional stb_image, for PNG/JPG/etc.
#if !defined(NO_STB_IMAGE) && defined(__has_include)
#if __has_include("stb_image.h")
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define USE_STB_IMAGE 1
#elif __has_include("external/stb_image.h")
#define STB_IMAGE_IMPLEMENTATION
#include "external/stb_image.h"
#define USE_STB_IMAGE 1
#endif
#endif

#ifndef OBJ2BIN_WINDOW
#define OBJ2BIN_WINDOW (1 << 20)	// input window, bytes; grows if a line exceeds it
#endif
#define OBJ2BIN_CACHE (3 * 65536)	// output cache, vertices

static const uint32_t WHITE = 0x00ffffff;

struct Vertex { float x, y, z; uint32_t rgb; };
struct Vec3 { float x, y, z; };
struct Vec2 { float u, v; };

// --------------------------------------------------------------------------
// chunked array
// --------------------------------------------------------------------------

// Append-only store in fixed 2^LOGN element blocks. Unlike std::vector it never
// copies or reallocates its contents, so growing to several GB does not need a
// 1.5x spike, and no single allocation has to be contiguous. Only the (tiny)
// block pointer table grows. Push reports allocation failure instead of
// throwing, so a file that does not fit can be rejected cleanly.
template <typename T, int LOGN = 20> class ChunkedArray
{
public:
	~ChunkedArray() { for (size_t i = 0; i < chunk.size(); i++) free( chunk[i] ); }
	bool Push( const T& v )
	{
		const size_t slot = count & MASK;
		if (!slot) // start a new block
		{
			T* c = (T*)malloc( sizeof( T ) << LOGN );
			if (!c) return false;
			chunk.push_back( c );
		}
		chunk[count >> LOGN][slot] = v, count++;
		return true;
	}
	const T& operator []( size_t i ) const { return chunk[i >> LOGN][i & MASK]; }
	size_t Size() const { return count; }
	uint64_t Bytes() const { return (uint64_t)chunk.size() * sizeof( T ) << LOGN; }
private:
	static const size_t MASK = ((size_t)1 << LOGN) - 1;
	std::vector<T*> chunk;
	size_t count = 0;
};

// --------------------------------------------------------------------------
// small helpers
// --------------------------------------------------------------------------

static inline bool IsSpace( char c ) { return c == ' ' || c == '\t' || c == '\r'; }
static inline void SkipSpace( const char*& p ) { while (IsSpace( *p )) p++; }
static inline int Wrap( int a, int n ) { a %= n; return a < 0 ? a + n : a; }

static inline uint32_t PackRGB( float r, float g, float b )
{
	const int ri = (int)(r * 255.0f + 0.5f), gi = (int)(g * 255.0f + 0.5f), bi = (int)(b * 255.0f + 0.5f);
	const uint32_t rc = (uint32_t)(ri < 0 ? 0 : (ri > 255 ? 255 : ri));
	const uint32_t gc = (uint32_t)(gi < 0 ? 0 : (gi > 255 ? 255 : gi));
	const uint32_t bc = (uint32_t)(bi < 0 ? 0 : (bi > 255 ? 255 : bi));
	return (rc << 16) | (gc << 8) | bc;
}

static int64_t SeekEnd( FILE* f )
{
#ifdef _MSC_VER
	if (_fseeki64( f, 0, SEEK_END )) return -1;
	const int64_t len = _ftelli64( f );
	_fseeki64( f, 0, SEEK_SET );
#else
	if (fseeko( f, 0, SEEK_END )) return -1;
	const int64_t len = (int64_t)ftello( f );
	fseeko( f, 0, SEEK_SET );
#endif
	return len;
}

// Whole-file read, for images only; text goes through LineReader below.
#ifndef USE_STB_IMAGE
static uint8_t* ReadFile( const char* path, size_t& size )
{
	FILE* f = fopen( path, "rb" );
	if (!f) return 0;
	const int64_t len = SeekEnd( f );
	if (len < 0 || (uint64_t)len + 1 > (uint64_t)SIZE_MAX) { fclose( f ); return 0; }
	uint8_t* buf = (uint8_t*)malloc( (size_t)len + 1 );
	if (!buf) { fclose( f ); return 0; }
	size = fread( buf, 1, (size_t)len, f );
	buf[size] = 0;
	fclose( f );
	return buf;
}

#endif

static bool FileExists( const std::string& p )
{
	FILE* f = fopen( p.c_str(), "rb" );
	if (!f) return false;
	fclose( f );
	return true;
}

static std::string DirOf( const std::string& path )
{
	const size_t slash = path.find_last_of( "/\\" );
	return slash == std::string::npos ? std::string( "" ) : path.substr( 0, slash + 1 );
}

static std::string BaseOf( const std::string& path )
{
	const size_t slash = path.find_last_of( "/\\" );
	return slash == std::string::npos ? path : path.substr( slash + 1 );
}

// --------------------------------------------------------------------------
// windowed line reader
// --------------------------------------------------------------------------

// Hands out one NUL-terminated line at a time from a fixed-size window. The
// returned pointer stays valid until the next Next() call. A line longer than
// the window doubles it; nothing else ever reallocates.
class LineReader
{
public:
	~LineReader() { if (f) fclose( f ); }
	bool Open( const char* path )
	{
		f = fopen( path, "rb" );
		if (!f) return false;
		size = SeekEnd( f );
		buf.resize( OBJ2BIN_WINDOW );
		return true;
	}
	// Logical line: physical lines joined where one ends in a backslash, as the
	// obj spec allows. The join is the slow path and costs nothing otherwise.
	const char* Next()
	{
		size_t len;
		const char* line = Raw( len );
		if (!line) return 0;
		while (len && IsSpace( line[len - 1] )) len--;
		if (!len || line[len - 1] != '\\') return line;
		joined.assign( line, len - 1 );
		continued++;
		while (1)
		{
			size_t n;
			const char* more = Raw( n );
			if (!more) break;
			while (n && IsSpace( more[n - 1] )) n--;
			joined += ' ';
			if (n && more[n - 1] == '\\') { joined.append( more, n - 1 ), continued++; continue; }
			joined.append( more, n );
			break;
		}
		return joined.c_str();
	}
	int64_t Size() const { return size; }
	uint64_t Read() const { return consumed; }
	bool Failed() const { return error; }			// read error, not end of file
	uint64_t Continued() const { return continued; }
	bool Unterminated() const { return unterminated; }	// no newline at end of file
private:
	const char* Raw( size_t& len )
	{
		size_t scan = head;
		while (1)
		{
			if (tail > scan)
			{
				char* nl = (char*)memchr( buf.data() + scan, '\n', tail - scan );
				if (nl)
				{
					*nl = 0;
					char* line = buf.data() + head;
					len = (size_t)(nl - buf.data()) - head;
					head = (size_t)(nl - buf.data()) + 1;
					return line;
				}
				scan = tail;
			}
			if (eof)
			{
				if (head == tail) return 0;
				if (tail == buf.size()) buf.push_back( 0 ); else buf[tail] = 0; // last line, no newline
				char* line = buf.data() + head;
				len = tail - head;
				head = tail;
				unterminated = true;
				return line;
			}
			Refill( scan );
		}
	}
	void Refill( size_t& scan )
	{
		if (head > 0) // slide the live range down to the front
		{
			memmove( buf.data(), buf.data() + head, tail - head );
			tail -= head, scan -= head, head = 0;
		}
		if (tail + 1 >= buf.size()) buf.resize( buf.size() * 2 ); // single line > window
		const size_t n = fread( buf.data() + tail, 1, buf.size() - tail - 1, f );
		tail += n, consumed += n;
		// a short read is only the end of the file if the stream is actually at
		// eof; anything else is an i/o error and must not pass as a clean finish.
		if (!n) eof = true, error = ferror( f ) != 0;
	}
	FILE* f = 0;
	std::vector<char> buf;
	std::string joined;			// scratch for continued lines only
	size_t head = 0, tail = 0;
	uint64_t consumed = 0, continued = 0;
	int64_t size = -1;
	bool eof = false, error = false, unterminated = false;
};

// --------------------------------------------------------------------------
// streaming triangle writer
// --------------------------------------------------------------------------

// Writes the header with a placeholder count, streams vertices through a cache
// and patches the count on close. Nothing beyond the cache is kept.
class TriWriter
{
public:
	bool Open( const char* path )
	{
		f = fopen( path, "wb+" );
		if (!f) return false;
		const uint32_t placeholder = 0;
		if (fwrite( &placeholder, sizeof( uint32_t ), 1, f ) != 1) { failed = true; return false; }
		cache.reserve( OBJ2BIN_CACHE );
		return true;
	}
	void Push( const Vertex& v )
	{
		cache.push_back( v ), verts++;
		if (cache.size() >= OBJ2BIN_CACHE) Flush();
	}
	uint64_t Triangles() const { return verts / 3; }
	bool Close()
	{
		if (!f) return false;
		Flush();
		const uint32_t count = (uint32_t)(verts / 3);
		if (!failed && (fseek( f, 0, SEEK_SET ) || fwrite( &count, sizeof( uint32_t ), 1, f ) != 1))
			failed = true;
		fclose( f ), f = 0;
		return !failed;
	}
private:
	void Flush()
	{
		if (!cache.empty() && !failed)
			if (fwrite( cache.data(), sizeof( Vertex ), cache.size(), f ) != cache.size()) failed = true;
		cache.clear();
	}
	FILE* f = 0;
	std::vector<Vertex> cache;
	uint64_t verts = 0;
	bool failed = false;
};

// --------------------------------------------------------------------------
// images: native TGA / BMP / PPM, or stb_image when available
// --------------------------------------------------------------------------

struct Image
{
	int w = 0, h = 0;
	std::vector<uint8_t> rgb;	// 3 bytes per texel, top row first
	float avg[3] = { 1, 1, 1 };	// average colour: the fallback for corners without a uv
};

#ifndef USE_STB_IMAGE

static bool DecodeTGA( const uint8_t* d, size_t size, Image& img )
{
	if (size < 18) return false;
	const int idLen = d[0], cmapType = d[1], type = d[2];
	const int w = d[12] | (d[13] << 8), h = d[14] | (d[15] << 8);
	const int bpp = d[16], desc = d[17];
	if (cmapType != 0 || (type != 2 && type != 10)) return false;
	if ((bpp != 24 && bpp != 32) || w <= 0 || h <= 0) return false;
	const int bypp = bpp / 8;
	const uint8_t* p = d + 18 + idLen, *end = d + size;
	std::vector<uint8_t> raw( (size_t)w * h * bypp );
	if (type == 2)
	{
		if (p + raw.size() > end) return false;
		memcpy( raw.data(), p, raw.size() );
	}
	else for (size_t o = 0; o < raw.size(); ) // RLE
	{
		if (p >= end) return false;
		const int hdr = *p++, count = (hdr & 127) + 1;
		if (hdr & 128)
		{
			if (p + bypp > end || o + (size_t)count * bypp > raw.size()) return false;
			for (int i = 0; i < count; i++, o += bypp) memcpy( &raw[o], p, bypp );
			p += bypp;
		}
		else
		{
			const size_t n = (size_t)count * bypp;
			if (p + n > end || o + n > raw.size()) return false;
			memcpy( &raw[o], p, n ), p += n, o += n;
		}
	}
	img.w = w, img.h = h, img.rgb.resize( (size_t)w * h * 3 );
	const bool topDown = (desc & 32) != 0;
	for (int y = 0; y < h; y++)
	{
		const uint8_t* src = &raw[(size_t)(topDown ? y : h - 1 - y) * w * bypp];
		uint8_t* dst = &img.rgb[(size_t)y * w * 3];
		for (int x = 0; x < w; x++, src += bypp, dst += 3)
			dst[0] = src[2], dst[1] = src[1], dst[2] = src[0]; // BGR -> RGB
	}
	return true;
}

static bool DecodeBMP( const uint8_t* d, size_t size, Image& img )
{
	if (size < 54 || d[0] != 'B' || d[1] != 'M') return false;
	const uint32_t dataOfs = d[10] | (d[11] << 8) | (d[12] << 16) | ((uint32_t)d[13] << 24);
	const int32_t w = (int32_t)(d[18] | (d[19] << 8) | (d[20] << 16) | ((uint32_t)d[21] << 24));
	const int32_t h = (int32_t)(d[22] | (d[23] << 8) | (d[24] << 16) | ((uint32_t)d[25] << 24));
	const int bpp = d[28] | (d[29] << 8);
	const uint32_t compr = d[30] | (d[31] << 8) | (d[32] << 16) | ((uint32_t)d[33] << 24);
	if ((bpp != 24 && bpp != 32) || (compr != 0 && compr != 3) || w <= 0 || h == 0) return false;
	const int height = h < 0 ? -h : h;
	const bool topDown = h < 0;
	const int bypp = bpp / 8, stride = ((w * bypp) + 3) & ~3;
	if (dataOfs + (size_t)stride * height > size) return false;
	img.w = w, img.h = height, img.rgb.resize( (size_t)w * height * 3 );
	for (int y = 0; y < height; y++)
	{
		const uint8_t* src = d + dataOfs + (size_t)(topDown ? y : height - 1 - y) * stride;
		uint8_t* dst = &img.rgb[(size_t)y * w * 3];
		for (int x = 0; x < w; x++, src += bypp, dst += 3)
			dst[0] = src[2], dst[1] = src[1], dst[2] = src[0];
	}
	return true;
}

static bool DecodePPM( const uint8_t* d, size_t size, Image& img )
{
	if (size < 10 || d[0] != 'P' || d[1] != '6') return false;
	const char* p = (const char*)d + 2, *end = (const char*)d + size;
	int field[3] = { 0, 0, 0 };
	for (int i = 0; i < 3; i++)
	{
		while (p < end && (IsSpace( *p ) || *p == '\n' || *p == '#'))
			if (*p == '#') { while (p < end && *p != '\n') p++; } else p++;
		if (p >= end) return false;
		char* next = 0;
		field[i] = (int)strtol( p, &next, 10 );
		if (next == p) return false;
		p = next;
	}
	if (field[0] <= 0 || field[1] <= 0 || field[2] != 255) return false;
	p++; // single whitespace character after maxval
	const size_t n = (size_t)field[0] * field[1] * 3;
	if (p + n > end) return false;
	img.w = field[0], img.h = field[1], img.rgb.assign( (const uint8_t*)p, (const uint8_t*)p + n );
	return true;
}

#endif // !USE_STB_IMAGE

static bool LoadImage( const std::string& path, Image& img )
{
#ifdef USE_STB_IMAGE
	int w = 0, h = 0, n = 0;
	uint8_t* pixels = stbi_load( path.c_str(), &w, &h, &n, 3 );
	if (!pixels) return false;
	img.w = w, img.h = h;
	img.rgb.assign( pixels, pixels + (size_t)w * h * 3 );
	stbi_image_free( pixels );
#else
	size_t size = 0;
	uint8_t* d = ReadFile( path.c_str(), size );
	if (!d) return false;
	const bool ok = DecodeTGA( d, size, img ) || DecodeBMP( d, size, img ) || DecodePPM( d, size, img );
	free( d );
	if (!ok) return false;
#endif
	double sum[3] = { 0, 0, 0 };
	const size_t texels = (size_t)img.w * img.h;
	if (!texels) return false;
	for (size_t i = 0; i < texels; i++)
		sum[0] += img.rgb[i * 3 + 0], sum[1] += img.rgb[i * 3 + 1], sum[2] += img.rgb[i * 3 + 2];
	for (int a = 0; a < 3; a++) img.avg[a] = (float)(sum[a] / (255.0 * (double)texels));
	return true;
}

// Bilinear sample, repeat wrapping, v flipped: OBJ has v=0 at the bottom.
static void Sample( const Image& img, float u, float v, float* out )
{
	const float x = u * img.w - 0.5f, y = (1.0f - v) * img.h - 0.5f;
	const int x0 = (int)floorf( x ), y0 = (int)floorf( y );
	const float fx = x - x0, fy = y - y0;
	const int xi[2] = { Wrap( x0, img.w ), Wrap( x0 + 1, img.w ) };
	const int yi[2] = { Wrap( y0, img.h ), Wrap( y0 + 1, img.h ) };
	for (int a = 0; a < 3; a++)
	{
		const float c00 = img.rgb[((size_t)yi[0] * img.w + xi[0]) * 3 + a];
		const float c10 = img.rgb[((size_t)yi[0] * img.w + xi[1]) * 3 + a];
		const float c01 = img.rgb[((size_t)yi[1] * img.w + xi[0]) * 3 + a];
		const float c11 = img.rgb[((size_t)yi[1] * img.w + xi[1]) * 3 + a];
		const float top = c00 + (c10 - c00) * fx, bottom = c01 + (c11 - c01) * fx;
		out[a] = (top + (bottom - top) * fy) * (1.0f / 255.0f);
	}
}

// --------------------------------------------------------------------------
// materials
// --------------------------------------------------------------------------

struct Material
{
	std::string name;
	float kd[3] = { 1, 1, 1 };
	int tex = -1;			// index into the texture table, -1 for none
	bool hasKd = false;
};

static std::vector<Material> materials;
static std::vector<Image> textures;
static std::vector<std::string> texPaths;
static uint64_t texMissing = 0, texFailed = 0;

static int FindMaterial( const std::string& name )
{
	for (size_t i = 0; i < materials.size(); i++) if (materials[i].name == name) return (int)i;
	return -1;
}

static bool ParseFloat3( const char* p, float* v )
{
	int read = 0;
	for (int i = 0; i < 3; i++)
	{
		SkipSpace( p );
		char* next = 0;
		const float f = strtof( p, &next );
		if (next == p) break;
		v[i] = f, p = next, read++;
	}
	if (read == 1) v[1] = v[2] = v[0]; // 'Kd 0.5' is legal
	return read > 0;
}

// Trailing filename of a map_* line, skipping the -o/-s/-mm/-bm/... options.
static std::string ParseMapName( const char* p )
{
	std::vector<std::string> tok;
	while (*p)
	{
		SkipSpace( p );
		if (!*p) break;
		const char* s = p;
		while (*p && !IsSpace( *p )) p++;
		tok.push_back( std::string( s, (size_t)(p - s) ) );
	}
	size_t i = 0;
	while (i < tok.size() && tok[i].size() > 1 && tok[i][0] == '-')
	{
		const std::string& o = tok[i];
		size_t args = 1;
		if (o == "-o" || o == "-s" || o == "-t") // 1..3 numeric arguments
		{
			args = 0;
			while (i + 1 + args < tok.size() && args < 3)
			{
				const char* s = tok[i + 1 + args].c_str();
				char* next = 0;
				strtod( s, &next );
				if (next == s || *next) break;
				args++;
			}
		}
		else if (o == "-mm") args = 2;
		i += 1 + args;
	}
	std::string name;
	for (; i < tok.size(); i++) name += (name.empty() ? "" : " ") + tok[i]; // names may contain spaces
	return name;
}

// Trailing name on a 'newmtl'/'usemtl' line, with the whitespace trimmed.
static std::string TrailingName( const char* p )
{
	SkipSpace( p );
	size_t len = strlen( p );
	while (len && IsSpace( p[len - 1] )) len--;
	return std::string( p, len );
}

static int LoadTexture( const std::string& baseDir, const std::string& rawName, bool enabled )
{
	if (!enabled || rawName.empty()) return -1;
	std::string name = rawName;
	for (size_t i = 0; i < name.size(); i++) if (name[i] == '\\') name[i] = '/';
	// try: as given, under 'textures/', then both again with the directories stripped.
	const std::string leaf = BaseOf( name );
	const std::string cand[5] = { baseDir + name, baseDir + "textures/" + name,
		baseDir + leaf, baseDir + "textures/" + leaf, name };
	std::string path;
	for (int i = 0; i < 5 && path.empty(); i++) if (FileExists( cand[i] )) path = cand[i];
	if (path.empty())
	{
		printf( "  texture not found: %s\n", rawName.c_str() );
		texMissing++;
		return -1;
	}
	for (size_t i = 0; i < texPaths.size(); i++) if (texPaths[i] == path) return (int)i;
	Image img;
	if (!LoadImage( path, img ))
	{
		printf( "  could not decode: %s\n", path.c_str() );
		texFailed++;
		return -1;
	}
	printf( "  texture: %s (%ix%i, avg %.2f %.2f %.2f)\n", path.c_str(), img.w, img.h,
		img.avg[0], img.avg[1], img.avg[2] );
	textures.push_back( std::move( img ) );
	texPaths.push_back( path );
	return (int)textures.size() - 1;
}

static void LoadMTL( const std::string& path, const std::string& baseDir, bool useTextures )
{
	LineReader mtl;
	if (!mtl.Open( path.c_str() )) { printf( "  no .mtl at %s\n", path.c_str() ); return; }
	printf( "  reading %s\n", path.c_str() );
	int cur = -1;
	float ke[3] = { 0, 0, 0 }, ka[3] = { 0, 0, 0 };
	bool hasKe = false, hasKa = false;
	// Kd wins; a black or absent Kd falls back to Ke, then Ka.
	auto finish = [&]() {
		if (cur < 0) return;
		Material& m = materials[cur];
		const bool black = m.kd[0] == 0 && m.kd[1] == 0 && m.kd[2] == 0;
		if (m.hasKd && !black) return;
		if (hasKe && (ke[0] + ke[1] + ke[2]) > 0) memcpy( m.kd, ke, 12 ), m.hasKd = true;
		else if (!m.hasKd && hasKa) memcpy( m.kd, ka, 12 ), m.hasKd = true;
	};
	const char* line;
	while ((line = mtl.Next()) != 0)
	{
		const char* p = line;
		SkipSpace( p );
		if (!strncmp( p, "newmtl", 6 ) && IsSpace( p[6] ))
		{
			finish();
			const std::string name = TrailingName( p + 6 );
			cur = FindMaterial( name );
			if (cur < 0)
			{
				Material m;
				m.name = name;
				materials.push_back( m );
				cur = (int)materials.size() - 1;
			}
			hasKe = hasKa = false;
			ke[0] = ke[1] = ke[2] = ka[0] = ka[1] = ka[2] = 0;
		}
		else if (cur >= 0 && p[0] == 'K' && (p[1] == 'd' || p[1] == 'e' || p[1] == 'a') && IsSpace( p[2] ))
		{
			const char which = p[1];
			const char* q = p + 2;
			SkipSpace( q );
			if (*q == '-' || *q == '.' || (*q >= '0' && *q <= '9')) // skip 'spectral'/'xyz' forms
			{
				float v[3] = { 0, 0, 0 };
				if (ParseFloat3( q, v ))
				{
					if (which == 'd') memcpy( materials[cur].kd, v, 12 ), materials[cur].hasKd = true;
					else if (which == 'e') memcpy( ke, v, 12 ), hasKe = true;
					else memcpy( ka, v, 12 ), hasKa = true;
				}
			}
		}
		else if (cur >= 0 && !strncmp( p, "map_Kd", 6 ) && IsSpace( p[6] ))
			materials[cur].tex = LoadTexture( baseDir, ParseMapName( p + 6 ), useTextures );
	}
	finish();
}

// --------------------------------------------------------------------------
// obj face tokens
// --------------------------------------------------------------------------

struct Corner { int v, vt; };

// One 'f' token: v, v/vt, v//vn or v/vt/vn. OBJ indices are 1-based; negative
// values are relative to the end of the list. Returns false if unusable.
static bool ParseCorner( const char*& p, int vCount, int vtCount, Corner& out )
{
	out.v = -1, out.vt = -1;
	char* next = 0;
	long vi = strtol( p, &next, 10 );
	bool ok = next != p;
	p = next;
	if (ok)
	{
		if (vi > 0) vi--; else if (vi < 0) vi += vCount; else ok = false;
		if (ok && (vi < 0 || vi >= vCount)) ok = false;
		if (ok) out.v = (int)vi;
	}
	if (*p == '/')
	{
		p++;
		if (*p != '/')
		{
			long ti = strtol( p, &next, 10 );
			if (next != p)
			{
				p = next;
				if (ti > 0) ti--; else if (ti < 0) ti += vtCount; else ti = -1;
				if (ti >= 0 && ti < vtCount) out.vt = (int)ti;
			}
		}
	}
	while (*p && !IsSpace( *p )) p++; // step over any /vn
	return ok;
}

// --------------------------------------------------------------------------
// main
// --------------------------------------------------------------------------

int main( int argc, char** argv )
{
	const char* inPath = 0;
	const char* outArg = 0;
	bool useTextures = true, badArgs = false;
	for (int i = 1; i < argc; i++)
	{
		if (!strcmp( argv[i], "--no-textures" ) || !strcmp( argv[i], "-nt" )) useTextures = false;
		else if (!inPath) inPath = argv[i];
		else if (!outArg) outArg = argv[i];
		else badArgs = true;
	}
	if (!inPath || badArgs)
	{
		fprintf( stderr, "usage: %s input.obj [output.bin] [--no-textures]\n", argv[0] );
		return 1;
	}
	std::string outPath;
	if (outArg) outPath = outArg; else
	{
		outPath = inPath;
		const size_t dot = outPath.find_last_of( '.' ), slash = outPath.find_last_of( "/\\" );
		if (dot != std::string::npos && (slash == std::string::npos || dot > slash)) outPath.erase( dot );
		outPath += ".bin";
	}
	const std::string baseDir = DirOf( inPath );
	LineReader obj;
	if (!obj.Open( inPath ))
	{
		fprintf( stderr, "error: could not read '%s': %s\n", inPath, strerror( errno ) );
		return 1;
	}
	TriWriter out;
	if (!out.Open( outPath.c_str() ))
	{
		fprintf( stderr, "error: could not write '%s': %s\n", outPath.c_str(), strerror( errno ) );
		return 1;
	}

	ChunkedArray<Vec3> pos;		// vertex positions, 1M per block
	ChunkedArray<Vec2> uv;		// texcoords, 1M per block
	std::vector<Corner> face;	// scratch: corners of the current face
	uint64_t badFaces = 0, degenerate = 0, quads = 0, ngons = 0, lines = 0, faceLines = 0;
	uint64_t fromTexel = 0, fromAvg = 0, fromKd = 0, fromDefault = 0;
	float bmin[3] = { 1e30f, 1e30f, 1e30f }, bmax[3] = { -1e30f, -1e30f, -1e30f };
	const bool progress = obj.Size() > (64 << 20);
	int curMat = -1;
	const char* fatal = 0;
	const char* line;
	while ((line = obj.Next()) != 0)
	{
		if (progress && !(++lines & 0xfffff))
			printf( "\r  parsing: %llu / %llu MB", (unsigned long long)(obj.Read() >> 20),
				(unsigned long long)(obj.Size() >> 20) ), fflush( stdout );
		const char* p = line;
		SkipSpace( p );
		if (p[0] == 'v' && IsSpace( p[1] ))
		{
			const char* q = p + 2;
			float v[3] = { 0, 0, 0 };
			for (int i = 0; i < 3; i++)
			{
				SkipSpace( q );
				char* next = 0;
				const float f = strtof( q, &next );
				if (next == q) break; // malformed; leave the rest at zero
				v[i] = f, q = next;
			}
			const Vec3 xyz = { v[0], v[1], v[2] };
			if (!pos.Push( xyz )) { fatal = "out of memory storing vertex positions"; break; }
			if (pos.Size() > 0x7fffffffull) { fatal = "more than 2^31 vertices; obj indices cannot address them"; break; }
		}
		else if (p[0] == 'v' && p[1] == 't' && IsSpace( p[2] ))
		{
			const char* q = p + 3;
			float t[2] = { 0, 0 };
			for (int i = 0; i < 2; i++)
			{
				SkipSpace( q );
				char* next = 0;
				const float f = strtof( q, &next );
				if (next == q) break;
				t[i] = f, q = next;
			}
			const Vec2 st = { t[0], t[1] };
			if (!uv.Push( st )) { fatal = "out of memory storing texcoords"; break; }
			if (uv.Size() > 0x7fffffffull) { fatal = "more than 2^31 texcoords"; break; }
		}
		else if (!strncmp( p, "mtllib", 6 ) && IsSpace( p[6] ))
		{
			const char* q = p + 6;
			while (*q) // one or more filenames on the line
			{
				SkipSpace( q );
				if (!*q) break;
				const char* s = q;
				while (*q && !IsSpace( *q )) q++;
				LoadMTL( baseDir + std::string( s, (size_t)(q - s) ), baseDir, useTextures );
			}
		}
		else if (!strncmp( p, "usemtl", 6 ) && IsSpace( p[6] ))
			curMat = FindMaterial( TrailingName( p + 6 ) );
		else if (p[0] == 'f' && IsSpace( p[1] ))
		{
			const char* q = p + 2;
			faceLines++;
			const int vCount = (int)pos.Size(), vtCount = (int)uv.Size();
			face.clear();
			bool malformed = false;
			while (1)
			{
				SkipSpace( q );
				if (!*q) break;
				Corner c;
				if (ParseCorner( q, vCount, vtCount, c )) face.push_back( c ); else malformed = true;
			}
			if (malformed || face.size() < 3) { badFaces++; continue; }
			if (face.size() == 4) quads++; else if (face.size() > 4) ngons++;
			const Material* mat = curMat >= 0 ? &materials[curMat] : 0;
			const Image* tex = (mat && mat->tex >= 0) ? &textures[mat->tex] : 0;
			for (size_t i = 2; i < face.size(); i++) // fan triangulation
			{
				const Corner t[3] = { face[0], face[i - 1], face[i] };
				if (t[0].v == t[1].v || t[1].v == t[2].v || t[0].v == t[2].v) { degenerate++; continue; }
				if (out.Triangles() == 0xffffffffull) { fatal = "triangle count exceeds 32 bits"; break; }
				for (int k = 0; k < 3; k++)
				{
					Vertex vert;
					const Vec3& src = pos[(size_t)t[k].v];
					vert.x = src.x, vert.y = src.y, vert.z = src.z;
					if (!mat) vert.rgb = WHITE, fromDefault++; else
					{
						float c[3] = { mat->kd[0], mat->kd[1], mat->kd[2] };
						if (tex)
						{
							float texel[3];
							if (t[k].vt >= 0)
							{
								const Vec2& st = uv[(size_t)t[k].vt];
								Sample( *tex, st.u, st.v, texel ), fromTexel++;
							}
							else
								texel[0] = tex->avg[0], texel[1] = tex->avg[1], texel[2] = tex->avg[2], fromAvg++;
							c[0] *= texel[0], c[1] *= texel[1], c[2] *= texel[2];
						}
						else if (mat->hasKd) fromKd++; else fromDefault++;
						vert.rgb = PackRGB( c[0], c[1], c[2] );
					}
					const float xyz[3] = { vert.x, vert.y, vert.z };
					for (int a = 0; a < 3; a++)
						bmin[a] = xyz[a] < bmin[a] ? xyz[a] : bmin[a],
						bmax[a] = xyz[a] > bmax[a] ? xyz[a] : bmax[a];
					out.Push( vert );
				}
			}
			if (fatal) break;
		}
	}
	if (progress) printf( "\r%*s\r", 40, "" ), fflush( stdout );

	if (obj.Failed() && !fatal) fatal = "read error partway through the input file";
	const uint64_t triangles = out.Triangles();
	if (fatal) fprintf( stderr, "error: %s; stopped at %llu triangles\n", fatal,
		(unsigned long long)triangles );
	if (!out.Close())
	{
		fprintf( stderr, "error: write failed (disk full?)\n" );
		return 1;
	}
	if (fatal)
	{
		remove( outPath.c_str() ); // never leave a plausible-looking partial file behind
		return 1;
	}
	if (!triangles)
	{
		fprintf( stderr, "error: no triangles in '%s'\n", inPath );
		remove( outPath.c_str() );
		return 1;
	}

	// report
	printf( "%s: %llu verts, %llu tris -> %s (%llu bytes)\n", inPath, (unsigned long long)pos.Size(),
		(unsigned long long)triangles, outPath.c_str(), (unsigned long long)(4 + triangles * 48) );
	if (pos.Bytes() + uv.Bytes() > (64 << 20)) printf( "vertex store: %llu MB resident\n",
		(unsigned long long)((pos.Bytes() + uv.Bytes()) >> 20) );
	printf( "bounds: (%.4f, %.4f, %.4f) .. (%.4f, %.4f, %.4f)\n",
		bmin[0], bmin[1], bmin[2], bmax[0], bmax[1], bmax[2] );
	printf( "read %llu of %lld bytes, %llu face lines -> %llu triangles\n",
		(unsigned long long)obj.Read(), (long long)obj.Size(),
		(unsigned long long)faceLines, (unsigned long long)triangles );
	printf( "materials: %zu, textures: %zu\n", materials.size(), textures.size() );
	printf( "colours: %llu sampled, %llu texture average, %llu flat Kd, %llu default white\n",
		(unsigned long long)fromTexel, (unsigned long long)fromAvg,
		(unsigned long long)fromKd, (unsigned long long)fromDefault );
	if (quads || ngons) printf( "triangulated %llu quads, %llu n-gons\n",
		(unsigned long long)quads, (unsigned long long)ngons );
	if (obj.Continued()) printf( "joined %llu continued line(s)\n", (unsigned long long)obj.Continued() );
	if (obj.Unterminated()) printf( "note: the input does not end in a newline; if that is unexpected,\n"
		"      the file itself may be truncated (a 4 GB copy or download limit, say)\n" );
	if (badFaces) printf( "warning: skipped %llu malformed face(s)\n", (unsigned long long)badFaces );
	if (degenerate) printf( "warning: skipped %llu degenerate triangle(s)\n", (unsigned long long)degenerate );
	if (texMissing || texFailed)
	{
		printf( "warning: %llu texture(s) not found, %llu not decoded",
			(unsigned long long)texMissing, (unsigned long long)texFailed );
#ifdef USE_STB_IMAGE
		printf( "\n" );
#else
		printf( " (no stb_image.h: only TGA/BMP/PPM are supported)\n" );
#endif
	}
	return 0;
}
