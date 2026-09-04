// tiny_bvh_base.h: platform-neutral part of TinyBVH. Do not include this file
// directly; include tiny_bvh.h, which selects the platform headers.

#ifndef TINY_BVH_H_
#error "Include tiny_bvh.h instead of tiny_bvh_base.h."
#endif

#ifndef TINY_BVH_BASE_H_
#define TINY_BVH_BASE_H_

namespace tinybvh {
// round 'x' up to a multiple of 'alignment', which must be a power of two.
inline size_t make_multiple_of( size_t x, size_t alignment )
{
	if (x > SIZE_MAX - (alignment - 1)) return 0; // would overflow
	return (x + (alignment - 1)) & ~(alignment - 1);
}
inline void* malloc64( size_t size, void* = nullptr )
{
	if (size == 0) return nullptr;
	size = make_multiple_of( size, 64 );
	if (size == 0) return nullptr; // overflowed in make_multiple_of
#ifdef TINYBVH_ALIGNED_ALLOC
	return TINYBVH_ALIGNED_ALLOC( 64, size );
#elif defined _WIN32 // MSVC / MinGW / clang-cl: the CRT provides _aligned_malloc.
	return _aligned_malloc( size, 64 );
#else // Linux, Apple, Android, Emscripten, other Unices; 32-bit and 64-bit.
	// posix_memalign rather than C11 aligned_alloc: the latter isn't declared by
	// glibc < 2.27 in strict C++ mode and is macOS 10.15+ / iOS 13+ only.
	void* ptr = nullptr;
	return posix_memalign( &ptr, 64, size ) == 0 ? ptr : nullptr;
#endif
}
inline void free64( void* ptr, void* = nullptr )
{
#ifdef TINYBVH_ALIGNED_FREE
	TINYBVH_ALIGNED_FREE( ptr );
#elif defined _WIN32
	_aligned_free( ptr );
#else
	free( ptr );
#endif
}
}; // namespace tinybvh

// error handling
#ifdef _WINDOWS_ // windows.h has been included
#define BVH_FATAL_ERROR_IF(c,s) if (c) { char t[512]; snprintf( t, 512, \
	"Fatal error in %s, line %i:\n%s\n", __FILE__, __LINE__, s ); \
	MessageBoxA( NULL, t, "Fatal error", MB_OK ); exit( 1 ); }
#else
#define BVH_FATAL_ERROR_IF(c,s) if (c) { fprintf( stderr, \
	"Fatal error in %s, line %i:\n%s\n", __FILE__, __LINE__, s ); exit( 1 ); }
#endif
#define BVH_FATAL_ERROR(s) BVH_FATAL_ERROR_IF(1,s)

namespace tinybvh {

// We'll use this whenever a layout has no specialized shadow ray query.
#define FALLBACK_SHADOW_QUERY( s ) { Ray r = s; float d = s.hit.t; Intersect( r ); return r.hit.t < d; }

#ifdef _MSC_VER
// Suppress a warning caused by the union of x,y,.. and cell[..] in vectors.
// We need this union to address vector components either by name or by index.
// The warning is re-enabled right after the definition of the data types.
#pragma warning ( push )
#pragma warning ( disable: 4201 /* nameless struct / union */ )
#endif

#ifndef TINYBVH_USE_CUSTOM_VECTOR_TYPES

struct bvhvec3;
struct ALIGNED( 16 ) bvhvec4
{
	// vector naming is designed to not cause any name clashes.
	bvhvec4() = default;
	constexpr bvhvec4( const float a, const float b, const float c, const float d ) : x( a ), y( b ), z( c ), w( d ) {}
	constexpr bvhvec4( const float a ) : x( a ), y( a ), z( a ), w( a ) {}
	bvhvec4( const bvhvec3 & a );
	bvhvec4( const bvhvec3 & a, float b );
	constexpr float& operator [] ( const int32_t i ) { return cell[i]; }
	const float& operator [] ( const int32_t i ) const { return cell[i]; }
	union { struct { float x, y, z, w; }; float cell[4]; };
};

struct ALIGNED( 8 ) bvhvec2
{
	bvhvec2() = default;
	constexpr bvhvec2( const float a, const float b ) : x( a ), y( b ) {}
	constexpr bvhvec2( const float a ) : x( a ), y( a ) {}
	constexpr bvhvec2( const bvhvec4 a ) : x( a.x ), y( a.y ) {}
	constexpr float& operator [] ( const int32_t i ) { return cell[i]; }
	const float& operator [] ( const int32_t i ) const { return cell[i]; }
	union { struct { float x, y; }; float cell[2]; };
};

struct bvhvec3
{
	bvhvec3() = default;
	constexpr bvhvec3( const float a, const float b, const float c ) : x( a ), y( b ), z( c ) {}
	constexpr bvhvec3( const float a ) : x( a ), y( a ), z( a ) {}
	constexpr bvhvec3( const bvhvec4 a ) : x( a.x ), y( a.y ), z( a.z ) {}
	constexpr float& operator [] ( const int32_t i ) { return cell[i]; }
	const float& operator [] ( const int32_t i ) const { return cell[i]; }
	union { struct { float x, y, z; }; float cell[3]; };
};

struct bvhint3
{
	bvhint3() = default;
	constexpr bvhint3( const int32_t a, const int32_t b, const int32_t c ) : x( a ), y( b ), z( c ) {}
	constexpr bvhint3( const int32_t a ) : x( a ), y( a ), z( a ) {}
	bvhint3( const bvhvec3& a ) { x = (int32_t)a.x, y = (int32_t)a.y, z = (int32_t)a.z; }
	constexpr int32_t& operator [] ( const int32_t i ) { return cell[i]; }
	const int32_t& operator [] ( const int32_t i ) const { return cell[i]; }
	union { struct { int32_t x, y, z; }; int32_t cell[3]; };
};

struct bvhint2
{
	bvhint2() = default;
	constexpr bvhint2( const int32_t a, const int32_t b ) : x( a ), y( b ) {}
	constexpr bvhint2( const int32_t a ) : x( a ), y( a ) {}
	constexpr int32_t& operator [] ( const int32_t i ) { return cell[i]; }
	const int32_t& operator [] ( const int32_t i ) const { return cell[i]; }
	union { struct { int32_t x, y; }; int32_t cell[2]; };
};

struct bvhuint2
{
	bvhuint2() = default;
	constexpr bvhuint2( const uint32_t a, const uint32_t b ) : x( a ), y( b ) {}
	constexpr bvhuint2( const uint32_t a ) : x( a ), y( a ) {}
	constexpr uint32_t& operator [] ( const int32_t i ) { return cell[i]; }
	const uint32_t& operator [] ( const int32_t i ) const { return cell[i]; }
	union { struct { uint32_t x, y; }; uint32_t cell[2]; };
};

struct bvhuint3
{
	bvhuint3() = default;
	constexpr bvhuint3( const uint32_t a, const uint32_t b, const uint32_t c ) : x( a ), y( b ), z( c ) {}
	constexpr bvhuint3( const uint32_t a ) : x( a ), y( a ), z( a ) {}
	constexpr uint32_t& operator [] ( const int32_t i ) { return cell[i]; }
	const uint32_t& operator [] ( const int32_t i ) const { return cell[i]; }
	union { struct { uint32_t x, y, z; }; uint32_t cell[3]; };
};

struct bvhuint4
{
	bvhuint4() = default;
	constexpr bvhuint4( const uint32_t a, const uint32_t b, const uint32_t c, const uint32_t d ) : x( a ), y( b ), z( c ), w( d ) {}
	constexpr bvhuint4( const uint32_t a ) : x( a ), y( a ), z( a ), w( a ) {}
	constexpr uint32_t& operator [] ( const int32_t i ) { return cell[i]; }
	const uint32_t& operator [] ( const int32_t i ) const { return cell[i]; }
	union { struct { uint32_t x, y, z, w; }; uint32_t cell[4]; };
};

struct bvhmat4 // exists only so we can use TinyBVH types conveniently in tinyscene.
{
	bvhmat4() = default;
	constexpr float& operator [] ( const int32_t i ) { return cell[i]; }
	constexpr const float& operator [] ( const int32_t i ) const { return cell[i]; }
	bvhmat4& operator += ( const bvhmat4& a ) { for (int i = 0; i < 16; i++) cell[i] += a.cell[i]; return *this; }
	float cell[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
};

#endif // TINYBVH_USE_CUSTOM_VECTOR_TYPES

struct ALIGNED( 32 ) bvhaabb
{
	bvhvec3 minBounds; uint32_t dummy1;
	bvhvec3 maxBounds; uint32_t dummy2;
};

struct bvhvec4slice
{
	bvhvec4slice() = default;
	bvhvec4slice( const bvhvec4* data, uint32_t count, uint32_t stride = sizeof( bvhvec4 ) );
	constexpr operator bool() const { return !!data; }
	const bvhvec4& operator [] ( size_t i ) const;
	const int8_t* data = nullptr;
	uint32_t count, stride;
};

// type punning helpers
template <typename D, typename S> TINYBVH_FORCEINLINE D tinybvh_bitcast( const S& s )
{
	D d; memcpy( &d, &s, sizeof( D ) ); return d;
}
TINYBVH_FORCEINLINE float tinybvh_as_float( const uint32_t x ) { return tinybvh_bitcast<float>( x ); }
TINYBVH_FORCEINLINE float tinybvh_as_float( const int32_t x ) { return tinybvh_bitcast<float>( x ); }
TINYBVH_FORCEINLINE uint32_t tinybvh_as_uint( const float x ) { return tinybvh_bitcast<uint32_t>( x ); }
TINYBVH_FORCEINLINE int32_t tinybvh_as_int( const float x ) { return tinybvh_bitcast<int32_t>( x ); }
TINYBVH_FORCEINLINE float tinybvh_getlane_f( const void* v, const size_t lane )
{
	float f; memcpy( &f, (const char*)v + lane * 4, 4 ); return f;
}
TINYBVH_FORCEINLINE void tinybvh_setlane_f( void* v, const size_t lane, const float f )
{
	memcpy( (char*)v + lane * 4, &f, 4 );
}
TINYBVH_FORCEINLINE uint32_t tinybvh_getlane_u( const void* v, const size_t lane )
{
	uint32_t u; memcpy( &u, (const char*)v + lane * 4, 4 ); return u;
}
TINYBVH_FORCEINLINE void tinybvh_setlane_u( void* v, const size_t lane, const uint32_t u )
{
	memcpy( (char*)v + lane * 4, &u, 4 );
}

// Math operations.
// Note: Since this header file is expected to be included in a source file
// of a separate project, the static keyword doesn't provide sufficient
// isolation; hence the tinybvh_ prefix.
TINYBVH_FORCEINLINE bool tinybvh_isfinite( float f )
{
	uint32_t i;
	memcpy( &i, &f, sizeof( i ) );
	return (i & 0x7F800000) != 0x7F800000; // ieee-754: finite if not all exponent bits are 1.
}
TINYBVH_FORCEINLINE bool tinybvh_isnan( float f )
{
	uint32_t i;
	memcpy( &i, &f, sizeof( i ) );
	return (i & 0x7F800000) == 0x7F800000 && (i & 0x007FFFFF) != 0; // ieee-754
}
TINYBVH_FORCEINLINE float tinybvh_safercp( const float x )
{
	const float r = 1 / x;
	if (!(fabsf( r ) <= BVH_RCP_FAR)) return copysignf( BVH_RCP_FAR, x );
	return r;
}
TINYBVH_FORCEINLINE double tinybvh_safercp( const double x )
{
	const double r = 1 / x;
	if (!(fabs( r ) <= BVH_DBL_RCP_FAR)) return copysign( BVH_DBL_RCP_FAR, x );
	return r;
}
TINYBVH_FORCEINLINE bvhvec3 tinybvh_safercp( const bvhvec3 a ) { return bvhvec3( tinybvh_safercp( a.x ), tinybvh_safercp( a.y ), tinybvh_safercp( a.z ) ); }
TINYBVH_FORCEINLINE bvhvec3 tinybvh_rcp( const bvhvec3 a ) { return tinybvh_safercp( a ); /* bvhvec3( 1.0f / a.x, 1.0f / a.y, 1.0f / a.z ); */ }
TINYBVH_FORCEINLINE float tinybvh_sqrf( const float x ) { return x * x; }
TINYBVH_FORCEINLINE float tinybvh_fma( const float a, const float b, const float c ) // Madmann91
{
#ifdef FP_FAST_FMAF
	return fmaf( a, b, c );
#elif defined(__clang__)
	_Pragma( "clang diagnostic push" )
		_Pragma( "clang diagnostic ignored \"-Wunknown-pragmas\"" )
		_Pragma( "STDC FP_CONTRACT ON" )
		_Pragma( "clang diagnostic pop" )
	#endif
		return a * b + c;
}
TINYBVH_FORCEINLINE float tinybvh_min( const float a, const float b ) { return a < b ? a : b; }
TINYBVH_FORCEINLINE float tinybvh_max( const float a, const float b ) { return a > b ? a : b; }
TINYBVH_FORCEINLINE double tinybvh_min( const double a, const double b ) { return a < b ? a : b; }
TINYBVH_FORCEINLINE double tinybvh_max( const double a, const double b ) { return a > b ? a : b; }
TINYBVH_FORCEINLINE int32_t tinybvh_min( const int32_t a, const int32_t b ) { return a < b ? a : b; }
TINYBVH_FORCEINLINE int32_t tinybvh_max( const int32_t a, const int32_t b ) { return a > b ? a : b; }
TINYBVH_FORCEINLINE uint32_t tinybvh_min( const uint32_t a, const uint32_t b ) { return a < b ? a : b; }
TINYBVH_FORCEINLINE uint64_t tinybvh_min( const uint64_t a, const uint64_t b ) { return a < b ? a : b; }
TINYBVH_FORCEINLINE uint32_t tinybvh_max( const uint32_t a, const uint32_t b ) { return a > b ? a : b; }
TINYBVH_FORCEINLINE uint64_t tinybvh_max( const uint64_t a, const uint64_t b ) { return a > b ? a : b; }
TINYBVH_FORCEINLINE bvhvec3 tinybvh_min( const bvhvec3& a, const bvhvec3& b ) { return bvhvec3( tinybvh_min( a.x, b.x ), tinybvh_min( a.y, b.y ), tinybvh_min( a.z, b.z ) ); }
TINYBVH_FORCEINLINE bvhvec4 tinybvh_min( const bvhvec4& a, const bvhvec4& b ) { return bvhvec4( tinybvh_min( a.x, b.x ), tinybvh_min( a.y, b.y ), tinybvh_min( a.z, b.z ), tinybvh_min( a.w, b.w ) ); }
TINYBVH_FORCEINLINE bvhvec3 tinybvh_max( const bvhvec3& a, const bvhvec3& b ) { return bvhvec3( tinybvh_max( a.x, b.x ), tinybvh_max( a.y, b.y ), tinybvh_max( a.z, b.z ) ); }
TINYBVH_FORCEINLINE bvhvec4 tinybvh_max( const bvhvec4& a, const bvhvec4& b ) { return bvhvec4( tinybvh_max( a.x, b.x ), tinybvh_max( a.y, b.y ), tinybvh_max( a.z, b.z ), tinybvh_max( a.w, b.w ) ); }
TINYBVH_FORCEINLINE float tinybvh_clamp( const float x, const float a, const float b ) { return x > a ? (x < b ? x : b) : a; /* NaN safe */ }
TINYBVH_FORCEINLINE int32_t tinybvh_clamp( const int32_t x, const int32_t a, const int32_t b ) { return x > a ? (x < b ? x : b) : a; /* NaN safe */ }
template <class T> inline static void tinybvh_swap( T& a, T& b ) { T t = a; a = b; b = t; }
TINYBVH_FORCEINLINE float tinybvh_halfarea( const bvhvec3& v ) { return v.x < -BVH_FAR ? 0 : (v.x * v.y + v.y * v.z + v.z * v.x); } // for SAH calculations
TINYBVH_FORCEINLINE uint32_t tinybvh_maxdim( const bvhvec3& v ) { uint32_t r = fabs( v.x ) > fabs( v.y ) ? 0 : 1; return fabs( v.z ) > fabs( v[r] ) ? 2 : r; }
TINYBVH_FORCEINLINE bool tinybvh_isfinite( double f )
{
	uint64_t i;
	memcpy( &i, &f, sizeof( i ) );
	return (i & 0x7FF0000000000000ull) != 0x7FF0000000000000ull;
}
TINYBVH_FORCEINLINE bool tinybvh_isnan( double f )
{
	uint64_t i;
	memcpy( &i, &f, sizeof( i ) );
	return (i & 0x7FF0000000000000ull) == 0x7FF0000000000000ull && (i & 0x000FFFFFFFFFFFFFull) != 0;
}
TINYBVH_FORCEINLINE double tinybvh_sqrf( const double x ) { return x * x; }
TINYBVH_FORCEINLINE double tinybvh_fma( const double a, const double b, const double c ) { return a * b + c; }
TINYBVH_FORCEINLINE double tinybvh_clamp( const double x, const double a, const double b ) { return x > a ? (x < b ? x : b) : a; /* NaN safe */ }

// Operator overloads.
// Only a minimal set is provided.
#ifndef TINYBVH_USE_CUSTOM_VECTOR_TYPES

TINYBVH_FORCEINLINE bvhvec2 operator-( const bvhvec2& a ) { return bvhvec2( -a.x, -a.y ); }
TINYBVH_FORCEINLINE bvhvec3 operator-( const bvhvec3& a ) { return bvhvec3( -a.x, -a.y, -a.z ); }
TINYBVH_FORCEINLINE bvhvec4 operator-( const bvhvec4& a ) { return bvhvec4( -a.x, -a.y, -a.z, -a.w ); }
TINYBVH_FORCEINLINE bvhvec2 operator+( const bvhvec2& a, const bvhvec2& b ) { return bvhvec2( a.x + b.x, a.y + b.y ); }
TINYBVH_FORCEINLINE bvhvec3 operator+( const bvhvec3& a, const bvhvec3& b ) { return bvhvec3( a.x + b.x, a.y + b.y, a.z + b.z ); }
TINYBVH_FORCEINLINE bvhvec4 operator+( const bvhvec4& a, const bvhvec4& b ) { return bvhvec4( a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w ); }
TINYBVH_FORCEINLINE bvhvec4 operator+( const bvhvec4& a, const bvhvec3& b ) { return bvhvec4( a.x + b.x, a.y + b.y, a.z + b.z, a.w ); }
TINYBVH_FORCEINLINE bvhvec2 operator-( const bvhvec2& a, const bvhvec2& b ) { return bvhvec2( a.x - b.x, a.y - b.y ); }
TINYBVH_FORCEINLINE bvhvec3 operator-( const bvhvec3& a, const bvhvec3& b ) { return bvhvec3( a.x - b.x, a.y - b.y, a.z - b.z ); }
TINYBVH_FORCEINLINE bvhvec4 operator-( const bvhvec4& a, const bvhvec4& b ) { return bvhvec4( a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w ); }
TINYBVH_FORCEINLINE void operator+=( bvhvec2& a, const bvhvec2& b ) { a.x += b.x; a.y += b.y; }
TINYBVH_FORCEINLINE void operator+=( bvhvec3& a, const bvhvec3& b ) { a.x += b.x; a.y += b.y; a.z += b.z; }
TINYBVH_FORCEINLINE void operator+=( bvhvec4& a, const bvhvec4& b ) { a.x += b.x; a.y += b.y; a.z += b.z; a.w += b.w; }
TINYBVH_FORCEINLINE bvhvec2 operator*( const bvhvec2& a, const bvhvec2& b ) { return bvhvec2( a.x * b.x, a.y * b.y ); }
TINYBVH_FORCEINLINE bvhvec3 operator*( const bvhvec3& a, const bvhvec3& b ) { return bvhvec3( a.x * b.x, a.y * b.y, a.z * b.z ); }
TINYBVH_FORCEINLINE bvhvec4 operator*( const bvhvec4& a, const bvhvec4& b ) { return bvhvec4( a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w ); }
TINYBVH_FORCEINLINE bvhvec2 operator*( const bvhvec2& a, float b ) { return bvhvec2( a.x * b, a.y * b ); }
TINYBVH_FORCEINLINE bvhvec3 operator*( const bvhvec3& a, float b ) { return bvhvec3( a.x * b, a.y * b, a.z * b ); }
TINYBVH_FORCEINLINE bvhvec4 operator*( const bvhvec4& a, float b ) { return bvhvec4( a.x * b, a.y * b, a.z * b, a.w * b ); }
TINYBVH_FORCEINLINE bvhvec2 operator*( float b, const bvhvec2& a ) { return bvhvec2( b * a.x, b * a.y ); }
TINYBVH_FORCEINLINE bvhvec3 operator*( float b, const bvhvec3& a ) { return bvhvec3( b * a.x, b * a.y, b * a.z ); }
TINYBVH_FORCEINLINE bvhvec4 operator*( float b, const bvhvec4& a ) { return bvhvec4( b * a.x, b * a.y, b * a.z, b * a.w ); }
TINYBVH_FORCEINLINE bvhvec2 operator/( float b, const bvhvec2& a ) { return bvhvec2( b / a.x, b / a.y ); }
TINYBVH_FORCEINLINE bvhvec3 operator/( float b, const bvhvec3& a ) { return bvhvec3( b / a.x, b / a.y, b / a.z ); }
TINYBVH_FORCEINLINE bvhvec4 operator/( float b, const bvhvec4& a ) { return bvhvec4( b / a.x, b / a.y, b / a.z, b / a.w ); }
TINYBVH_FORCEINLINE bvhvec3 operator/( bvhvec3 b, const bvhvec3& a ) { return bvhvec3( b.x / a.x, b.y / a.y, b.z / a.z ); }
TINYBVH_FORCEINLINE void operator*=( bvhvec3& a, const float b ) { a.x *= b; a.y *= b; a.z *= b; }

#endif // TINYBVH_USE_CUSTOM_VECTOR_TYPES

// Vector math: cross and dot.
TINYBVH_FORCEINLINE bvhvec3 tinybvh_cross( const bvhvec3& a, const bvhvec3& b )
{
	return bvhvec3( a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x );
}
TINYBVH_FORCEINLINE float tinybvh_dot( const bvhvec2& a, const bvhvec2& b ) { return a.x * b.x + a.y * b.y; }
TINYBVH_FORCEINLINE float tinybvh_dot( const bvhvec3& a, const bvhvec3& b ) { return a.x * b.x + a.y * b.y + a.z * b.z; }
TINYBVH_FORCEINLINE float tinybvh_dot( const bvhvec4& a, const bvhvec4& b ) { return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w; }

// Vector math: common operations.
TINYBVH_FORCEINLINE float tinybvh_length( const bvhvec3& a ) { return sqrtf( a.x * a.x + a.y * a.y + a.z * a.z ); }
TINYBVH_FORCEINLINE bvhvec3 tinybvh_normalize( const bvhvec3& a )
{
	float l = tinybvh_length( a ), rl = l == 0 ? 0 : (1.0f / l);
	return a * rl;
}
inline bvhvec3 tinybvh_transform_point( const bvhvec3& v, const bvhmat4& T )
{
	const float* Tcell = (const float*)&T;
	const bvhvec3 res(
		Tcell[0] * v.x + Tcell[1] * v.y + Tcell[2] * v.z + Tcell[3],
		Tcell[4] * v.x + Tcell[5] * v.y + Tcell[6] * v.z + Tcell[7],
		Tcell[8] * v.x + Tcell[9] * v.y + Tcell[10] * v.z + Tcell[11] );
	const float w = Tcell[12] * v.x + Tcell[13] * v.y + Tcell[14] * v.z + Tcell[15];
	if (w == 1) return res; else return res * (1.f / w);
}
inline bvhvec3 tinybvh_transform_vector( const bvhvec3& v, const bvhmat4& T )
{
	const float* Tcell = (const float*)&T;
	return bvhvec3( Tcell[0] * v.x + Tcell[1] * v.y + Tcell[2] * v.z, Tcell[4] * v.x +
		Tcell[5] * v.y + Tcell[6] * v.z, Tcell[8] * v.x + Tcell[9] * v.y + Tcell[10] * v.z );
}

#ifdef DOUBLE_PRECISION_SUPPORT
// Double-precision math

#ifndef TINYBVH_USE_CUSTOM_VECTOR_TYPES

struct bvhdbl3
{
	bvhdbl3() = default;
	constexpr bvhdbl3( const double a, const double b, const double c ) : x( a ), y( b ), z( c ) {}
	constexpr bvhdbl3( const double a ) : x( a ), y( a ), z( a ) {}
	bvhdbl3( const bvhvec3 a ) : x( (double)a.x ), y( (double)a.y ), z( (double)a.z ) {}
	constexpr double& operator [] ( const int32_t i ) { return cell[i]; }
	const double& operator [] ( const int32_t i ) const { return cell[i]; }
	union { struct { double x, y, z; }; double cell[3]; };
};

#endif // TINYBVH_USE_CUSTOM_VECTOR_TYPES

#ifdef _MSC_VER
#pragma warning ( pop )
#endif

TINYBVH_FORCEINLINE bvhdbl3 tinybvh_min( const bvhdbl3& a, const bvhdbl3& b ) { return bvhdbl3( tinybvh_min( a.x, b.x ), tinybvh_min( a.y, b.y ), tinybvh_min( a.z, b.z ) ); }
TINYBVH_FORCEINLINE bvhdbl3 tinybvh_max( const bvhdbl3& a, const bvhdbl3& b ) { return bvhdbl3( tinybvh_max( a.x, b.x ), tinybvh_max( a.y, b.y ), tinybvh_max( a.z, b.z ) ); }

#ifndef TINYBVH_USE_CUSTOM_VECTOR_TYPES

constexpr bvhdbl3 operator-( const bvhdbl3& a ) { return bvhdbl3( -a.x, -a.y, -a.z ); }
constexpr bvhdbl3 operator+( const bvhdbl3& a, const bvhdbl3& b ) { return bvhdbl3( a.x + b.x, a.y + b.y, a.z + b.z ); }
constexpr bvhdbl3 operator-( const bvhdbl3& a, const bvhdbl3& b ) { return bvhdbl3( a.x - b.x, a.y - b.y, a.z - b.z ); }
constexpr void operator+=( bvhdbl3& a, const bvhdbl3& b ) { a.x += b.x; a.y += b.y; a.z += b.z; }
constexpr bvhdbl3 operator*( const bvhdbl3& a, const bvhdbl3& b ) { return bvhdbl3( a.x * b.x, a.y * b.y, a.z * b.z ); }
constexpr bvhdbl3 operator*( const bvhdbl3& a, double b ) { return bvhdbl3( a.x * b, a.y * b, a.z * b ); }
constexpr bvhdbl3 operator*( double b, const bvhdbl3& a ) { return bvhdbl3( b * a.x, b * a.y, b * a.z ); }
constexpr bvhdbl3 operator/( double b, const bvhdbl3& a ) { return bvhdbl3( b / a.x, b / a.y, b / a.z ); }
constexpr bvhdbl3 operator/( bvhdbl3 b, const bvhdbl3& a ) { return bvhdbl3( b.x / a.x, b.y / a.y, b.z / a.z ); }
constexpr bvhdbl3& operator*=( bvhdbl3& a, const double b ) { a.x *= b, a.y *= b, a.z *= b; return a; }

#endif // TINYBVH_USE_CUSTOM_VECTOR_TYPES

struct bvhdbl3slice
{
	bvhdbl3slice() = default;
	bvhdbl3slice( const bvhdbl3* data, uint64_t count, uint64_t stride = sizeof( bvhdbl3 ) );
	constexpr operator bool() const { return !!data; }
	const bvhdbl3& operator [] ( size_t i ) const;
	const int8_t* data = nullptr;
	uint64_t count, stride;
};

TINYBVH_FORCEINLINE double tinybvh_length( const bvhdbl3& a ) { return sqrt( a.x * a.x + a.y * a.y + a.z * a.z ); }
TINYBVH_FORCEINLINE bvhdbl3 tinybvh_normalize( const bvhdbl3& a )
{
	double l = tinybvh_length( a ), rl = l == 0 ? 0 : (1.0 / l);
	return a * rl;
}
TINYBVH_FORCEINLINE bvhdbl3 tinybvh_safercp( const bvhdbl3 a ) { return bvhdbl3( tinybvh_safercp( a.x ), tinybvh_safercp( a.y ), tinybvh_safercp( a.z ) ); }
TINYBVH_FORCEINLINE bvhdbl3 tinybvh_rcp( const bvhdbl3 a ) { return tinybvh_safercp( a ); }
inline bvhdbl3 tinybvh_transform_point( const bvhdbl3& v, const double* T )
{
	const bvhdbl3 res(
		T[0] * v.x + T[1] * v.y + T[2] * v.z + T[3],
		T[4] * v.x + T[5] * v.y + T[6] * v.z + T[7],
		T[8] * v.x + T[9] * v.y + T[10] * v.z + T[11] );
	const double w = T[12] * v.x + T[13] * v.y + T[14] * v.z + T[15];
	if (w == 1) return res; else return res * (1. / w);
}
inline bvhdbl3 tinybvh_transform_vector( const bvhdbl3& v, const double* T )
{
	return bvhdbl3( T[0] * v.x + T[1] * v.y + T[2] * v.z, T[4] * v.x +
		T[5] * v.y + T[6] * v.z, T[8] * v.x + T[9] * v.y + T[10] * v.z );
}

TINYBVH_FORCEINLINE bvhdbl3 tinybvh_cross( const bvhdbl3& a, const bvhdbl3& b )
{
	return bvhdbl3( a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x );
}
TINYBVH_FORCEINLINE double tinybvh_dot( const bvhdbl3& a, const bvhdbl3& b ) { return a.x * b.x + a.y * b.y + a.z * b.z; }

TINYBVH_FORCEINLINE double tinybvh_halfarea( const bvhdbl3& v ) { return v.x < -BVH_DBL_FAR ? 0 : (v.x * v.y + v.y * v.z + v.z * v.x); } // for SAH calculations

TINYBVH_FORCEINLINE uint32_t tinybvh_maxdim( const bvhdbl3& v ) { uint32_t r = fabs( v.x ) > fabs( v.y ) ? 0 : 1; return fabs( v.z ) > fabs( v[r] ) ? 2 : r; }

// Double-precision 4x4 matrix, the counterpart of bvhmat4 for BLASInstanceEx.
struct bvhdblmat4
{
	bvhdblmat4() = default;
	constexpr double& operator [] ( const int32_t i ) { return cell[i]; }
	constexpr const double& operator [] ( const int32_t i ) const { return cell[i]; }
	double cell[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
};
inline bvhdbl3 tinybvh_transform_point( const bvhdbl3& v, const bvhdblmat4& T ) { return tinybvh_transform_point( v, T.cell ); }
inline bvhdbl3 tinybvh_transform_vector( const bvhdbl3& v, const bvhdblmat4& T ) { return tinybvh_transform_vector( v, T.cell ); }

#endif // DOUBLE_PRECISION_SUPPORT

// ============================================================================
//
//        T R A I T S
//
// ============================================================================

// bvh_traits maps the scalar type of a BVH instantiation to its vector types.
// Application code may specialize this for other vector libraries.
template <typename Float> struct bvh_traits;
template <> struct bvh_traits<float>
{
	using vec3 = bvhvec3;
	using vertex = bvhvec4;		// vertex storage: 16 bytes per vertex, w unused
	using slice = bvhvec4slice;
	using mat4 = bvhmat4;
	static constexpr bool wide_layouts = true;	// BVH4_CPU, BVH8_CPU and BVH_SoA exist for this scalar type.
};
#ifdef DOUBLE_PRECISION_SUPPORT
template <> struct bvh_traits<double>
{
	using vec3 = bvhdbl3;
	using vertex = bvhdbl3;		// vertex storage: 24 bytes per vertex
	using slice = bvhdbl3slice;
	using mat4 = bvhdblmat4;
	static constexpr bool wide_layouts = false;
};
#endif

// 'Far' sentinel per scalar type; BVH_FAR and BVH_DBL_FAR for templated code.
template <typename Float> constexpr Float bvh_far = Float( 0 );
template <> inline constexpr float bvh_far<float> = BVH_FAR;
template <> inline constexpr double bvh_far<double> = BVH_DBL_FAR;

// ============================================================================
//
//        T I N Y _ B V H   I N T E R F A C E
//
// ============================================================================

// The templated types live in tinybvh::impl; tinybvh provides aliases for the
// single precision (BVH, Ray, ..) and double precision (BVH_Double, RayEx, ..)
// instantiations.
namespace impl {

// With INST_IDX_BITS below 32, 32-bit indices store the instance index in the top bits of
// hit.prim, shifted by INST_IDX_SHFT. 64-bit indices always use a separate field.
template <typename Index> constexpr bool bvh_packed_inst = INST_IDX_BITS != 32 && sizeof( Index ) == 4;
template <typename Index> constexpr int bvh_inst_shift = bvh_packed_inst<Index> ? INST_IDX_SHFT : 0;
template <typename Index, bool packed = bvh_packed_inst<Index>> struct IntersectionInst { Index inst; };
template <typename Index> struct IntersectionInst<Index, true> { };

#if defined _MSC_VER || defined __GNUC__
#pragma pack(push, 4) // is there a good alternative for Clang / EMSCRIPTEN?
#endif
template <typename Float, typename Index> struct Intersection : IntersectionInst<Index>
{
	// An intersection result is designed to fit in no more than
	// four 32-bit values. This allows efficient storage of a result in
	// GPU code. The obvious missing result is an instance id; consider
	// squeezing this in the 'prim' field in some way.
	// Using this data and the original triangle data, all other info for
	// shading (such as normal, texture color etc.) can be reconstructed.
	Float t, u, v;	// distance along ray & barycentric coordinates of the intersection
	Index prim;		// primitive index
	// 64 byte of custom data - For the typical case, where Intersection is
	// a member of a cache-aligned Ray, this starts at a cache line boundary.
	void* auxData;
	union
	{
		unsigned char userChar[56];
		float userFloat[14];
		uint32_t userInt32[14];
		double userDouble[7];
		uint64_t userInt64[7];
	};
};
#if defined _MSC_VER || defined __GNUC__
#pragma pack(pop) // is there a good alternative for Clang / EMSCRIPTEN?
#endif

// 16 bits of mask to tell which BLASInstance to intersect or not.
// By default the mask will be initialized to intersect all instances.
#define RAY_MASK_INTERSECT_ALL 0xFFFF

template <typename Float, typename Index> struct ALIGNED( 64 ) Ray
{
	// Basic ray class. Note: For single blas traversal it is expected
	// that Ray::rD is properly initialized. For tlas/blas traversal this
	// field is typically updated for each blas.
	using Vec3 = typename bvh_traits<Float>::vec3;
	Ray() = default;
	Ray( Vec3 origin, Vec3 direction, Float t = bvh_far<Float>, uint32_t rayMask = RAY_MASK_INTERSECT_ALL )
	{
		O = origin, D = tinybvh_normalize( direction ), rD = tinybvh_rcp( D );
		hit.t = t, hit.u = hit.v = 0, hit.prim = 0;
		if constexpr (!bvh_packed_inst<Index>) hit.inst = 0;
		mask = rayMask & RAY_MASK_INTERSECT_ALL;
	}
	// Records the hit primitive together with the instance index of the current BLAS.
	void SetHitPrim( const Index prim )
	{
		if constexpr (bvh_packed_inst<Index>) hit.prim = (prim & PRIM_IDX_MASK) + instIdx;
		else hit.prim = prim, hit.inst = instIdx;
	}
	ALIGNED( 16 ) Vec3 O; uint32_t mask = RAY_MASK_INTERSECT_ALL;
	ALIGNED( 16 ) Vec3 D; Index instIdx = 0;
	ALIGNED( 16 ) Vec3 rD;
	// a packed 16-byte hit record starts on a 16-byte boundary
	alignas( bvh_packed_inst<Index> ? 16 : alignof( Intersection<Float, Index> ) ) Intersection<Float, Index> hit;
};

} // namespace impl

template <typename Float, typename Index, typename Vec3>
inline Float tinybvh_intersect_aabb( impl::Ray<Float, Index>& ray, const Vec3& aabbMin, const Vec3& aabbMax )
{
	// "slab test" ray/AABB intersection
	Float tx1 = (aabbMin.x - ray.O.x) * ray.rD.x, tx2 = (aabbMax.x - ray.O.x) * ray.rD.x;
	Float tmin = tinybvh_min( tx1, tx2 ), tmax = tinybvh_max( tx1, tx2 );
	Float ty1 = (aabbMin.y - ray.O.y) * ray.rD.y, ty2 = (aabbMax.y - ray.O.y) * ray.rD.y;
	tmin = tinybvh_max( tmin, tinybvh_min( ty1, ty2 ) );
	tmax = tinybvh_min( tmax, tinybvh_max( ty1, ty2 ) );
	Float tz1 = (aabbMin.z - ray.O.z) * ray.rD.z, tz2 = (aabbMax.z - ray.O.z) * ray.rD.z;
	tmin = tinybvh_max( tmin, tinybvh_min( tz1, tz2 ) );
	tmax = tinybvh_min( tmax, tinybvh_max( tz1, tz2 ) );
	if (tmax >= tmin && tmin < ray.hit.t && tmax >= 0) return tmin; else return bvh_far<Float>;
}

#if defined ENABLE_THREADED_BUILDS && !defined TINYBVH_NO_BUILTIN_POOL
// Default build hooks: a process-wide std::thread pool (tiny_bvh_threadpool.h).
// Define TINYBVH_NO_BUILTIN_POOL if you prefer to provide your own interface.
void tinybvh_builtin_spawn( void (*fn)(void* payload), const void* payload, uint32_t payload_size, void* userdata );
void tinybvh_builtin_barrier( void* userdata );
void tinybvh_builtin_parallel_for( uint32_t n, void (*fn)(uint32_t index, void* payload), void* payload, void* userdata );
// tinybvh_shutdown_builtin_pool: Releases the calling thread's build pool, joining its worker threads. 
// Optional; a thread's pool is released automatically when that thread exits. Call this explicitly if 
// you need teardown to happen at a controlled point rather than in an exit-time destructor.
void tinybvh_shutdown_builtin_pool();
#define TINYBVH_DEFAULT_SPAWN tinybvh_builtin_spawn
#define TINYBVH_DEFAULT_BARRIER tinybvh_builtin_barrier
#define TINYBVH_DEFAULT_PARALLEL_FOR tinybvh_builtin_parallel_for
#else
#define TINYBVH_DEFAULT_SPAWN nullptr
#define TINYBVH_DEFAULT_BARRIER nullptr
#define TINYBVH_DEFAULT_PARALLEL_FOR nullptr
#endif

struct BVHContext
{
	void* (*malloc)(size_t size, void* userdata) = malloc64;
	void (*free)(void* ptr, void* userdata) = free64;
	// Parallel build hooks: 'spawn' runs fn(payload-copy) asynchronously
	void (*spawn)(void (*fn)(void* payload), const void* payload, uint32_t payload_size, void* userdata) = TINYBVH_DEFAULT_SPAWN;
	// Wait for spawned tasks
	void (*barrier)(void* userdata) = TINYBVH_DEFAULT_BARRIER;
	// Runs fn(i, payload) for i in [0,n) and block for the results
	void (*parallel_for)(uint32_t n, void (*fn)(uint32_t index, void* payload), void* payload, void* userdata) = TINYBVH_DEFAULT_PARALLEL_FOR;
	void* userdata = nullptr;
};

#undef TINYBVH_DEFAULT_SPAWN
#undef TINYBVH_DEFAULT_BARRIER
#undef TINYBVH_DEFAULT_PARALLEL_FOR

// The functions below serialize when no thread pool is available.

// Spawn fn(payload) asynchronously
inline void tinybvh_spawn( const BVHContext& ctx, void (*fn)(void*), const void* payload, uint32_t payload_size )
{
	if (ctx.spawn) ctx.spawn( fn, payload, payload_size, ctx.userdata ); else fn( (void*)payload );
}
// Wait for spawned tasks to finish
inline void tinybvh_barrier( const BVHContext& ctx )
{
	if (ctx.barrier) ctx.barrier( ctx.userdata );
}
// Execute a parallel loop via the thread pool
inline void tinybvh_parallel_for( const BVHContext& ctx, uint32_t n, void (*fn)(uint32_t, void*), void* payload )
{
	if (ctx.parallel_for) ctx.parallel_for( n, fn, payload, ctx.userdata );
	else for (uint32_t i = 0; i < n; i++) fn( i, payload );
}

struct BVHBuildSettings
{
	bool usePresplitting = false;	// pre-split triangles before building the BVH.
	bool useSpatialSplits = false;	// consider spatial splits during construction (SBVH).
	bool presplitPostPass = true;	// attempt to un-split primitives in leafs after a presplit build.
	float presplitFactor = 0.3f;	// presplit budget relative to input data size.
	bool useFullSweep = false;		// for experiments only; full-sweep SAH builder.
	bool postOptimize = false;		// optimize generated BVH using tree rotations.
	int optimizeIterations = 25;	// default optimization iterations.
	bool useSIMDifavailable = true;	// set to false to use the scalar reference builder.
};

enum BVHType : uint32_t
{
	// Every BVH class is derived from BVHBase, but we don't use virtual functions, for
	// performance reasons. For a TLAS over a mix of BVH layouts we do however need this
	// kind of behavior when transitioning from a TLAS leaf to a BLAS root node.
	LAYOUT_UNDEFINED = 0,
	LAYOUT_BVH = 1,
	LAYOUT_BVH_VERBOSE,
	LAYOUT_BVH_GPU,
	LAYOUT_MBVH,
	LAYOUT_BVH4_CPU,
	LAYOUT_BVH4_GPU,
	LAYOUT_CWBVH,
	LAYOUT_BVH8_AVX2,
	LAYOUT_VOXELSET,
	LAYOUT_BVH_SOA
};

namespace impl {

template <typename Float, typename Index> class BVHBase;
template <typename Float, typename Index> class BVH;
template <typename Float, typename Index> class BVH_Verbose;
template <typename Float, typename Index> class BVH_GPU;
template <typename Float, typename Index> class BVH_SoA;
template <int M, typename Float, typename Index> class MBVH;
template <typename Float, typename Index> class BVH4_GPU;
template <typename Float, typename Index> class BVH4_CPU;
template <typename Float, typename Index> class BVH8_CWBVH;
template <typename Float, typename Index> class BVH8_CPU;
template <typename Float, typename Index> class BLASInstance;
template <typename Float, typename Index> struct BVHTri4Leaf;

// Set by the platform headers for the instantiations that have SIMD builders.
template <typename Float, typename Index> struct BVHSIMDBuilders { static constexpr bool avx = false; };

template <typename Float, typename Index> class BVHBase
{
public:
	using Traits = bvh_traits<Float>;
	using Vec3 = typename Traits::vec3;
	using Vertex = typename Traits::vertex;
	using Slice = typename Traits::slice;
	using Mat4 = typename Traits::mat4;
	using Ray = impl::Ray<Float, Index>;
	using Intersection = impl::Intersection<Float, Index>;
	using BVH = impl::BVH<Float, Index>;
	using BVH_Verbose = impl::BVH_Verbose<Float, Index>;
	using BLASInstance = impl::BLASInstance<Float, Index>;
	using BVHTri4Leaf = impl::BVHTri4Leaf<Float, Index>;
	struct ALIGNED( 32 ) Fragment
	{
		// A fragment stores the bounds of an input primitive. The name 'Fragment' is from
		// "Parallel Spatial Splits in Bounding Volume Hierarchies", 2016, Fuetterling et al.,
		// and refers to the potential splitting of these boxes for SBVH construction.
		Vec3 bmin;					// AABB min x, y and z
		Index primIdx;				// index of the original primitive
		Vec3 bmax;					// AABB max x, y and z
		uint32_t clipped = 0;		// Fragment is the result of clipping if > 0.
		void Extend( Vec3 p ) { bmin = tinybvh_min( p, bmin ); bmax = tinybvh_max( p, bmax ); }
	};
	// BVH flags, maintainted by tiny_bvh.
	bool rebuildable = true;		// rebuilds are safe only if a tree has not been converted.
	bool refittable = true;			// refits are safe only if the tree has no spatial splits.
	bool may_have_holes = false;	// threaded builds and MergeLeafs produce BVHs with unused nodes.
	bool bvh_over_aabbs = false;	// a BVH over AABBs is useful for e.g. TLAS traversal.
	bool bvh_over_indices = false;	// a BVH over indices cannot translate primitive index to vertex index.
	bool threadedBuild = true;		// will be disabled for small meshes.
	// BVH construction flags and settings.
	BVHBuildSettings settings;		// build settings: presplitting, full-sweep etc.
	float c_trav = C_TRAV;			// cost of a traversal step, used to steer SAH construction.
	float c_int = C_INT;			// cost of a primitive intersection, used to steer SAH construction.
	BVHContext context;				// context used to provide user-defined allocation functions.
	BVHType layout = LAYOUT_UNDEFINED; // BVH layout identifier.
	// Keep track of allocated buffer size to avoid repeated allocation during layout conversion.
	Index allocatedNodes = 0;		// number of nodes allocated for the BVH.
	Index usedNodes = 0;			// number of nodes used for the BVH.
	Index triCount = 0;				// number of primitives in the BVH.
	Index idxCount = 0;				// number of primitive indices; can exceed triCount for SBVH.
	bool l_quads = false;			// some layouts have 4 prims in each leaf; adjust SAH cost for this.
	uint32_t hqbvhbins = HQBVHBINS;	// number of bins to use in SBVH construction.
	bool hqbvhoddeven = false;		// if true, odd levels will use one extra bin during construction.
	Vec3 aabbMin, aabbMax;		// bounds of the root node of the BVH.
	// Opacity maps support.
	uint32_t opmapN = 0;			// opacity micro map subdivision: 0 = no maps.
	uint32_t* opmap = 0;			// opacity micro maps; opmapN^2 bits per triangle.
	bool hasOpacityMicroMaps() const { return opmapN > 0; }
	void SetOpacityMicroMaps( uint32_t* mapData, uint32_t N ) { opmap = mapData, opmapN = N; }
	// Custom memory allocation
	void* AlignedAlloc( size_t size ) const;
	void AlignedFree( void* ptr ) const;
	// Single-object allocation - used for the atomic build counters.
	template <class T, class V> T* ContextNew( const V& init )
	{
		void* mem = AlignedAlloc( sizeof( T ) );
		return mem ? new (mem) T( init ) : nullptr;
	}
	template <class T> void ContextDelete( T*& obj )
	{
		if (obj) obj->~T(), AlignedFree( obj ), obj = 0;
	}
	// Common methods
	void CopyBasePropertiesFrom( const BVHBase& original );	// copy flags from one BVH to another
	// First word of a BVH cache file: version, scalar and index width, and layout.
	uint32_t CacheHeader() const { return uint32_t( TINY_BVH_CACHE_VERSION + (sizeof( Float ) << 16) + (sizeof( Index ) << 20) + (layout << 24) ); }
protected:
	~BVHBase() {}
	TINYBVH_FORCEINLINE void IntersectTri( Ray& ray, const Index idx, const Slice& verts, const Index i0, const Index i1, const Index i2 ) const;
	TINYBVH_FORCEINLINE bool TriOccludes( const Ray& ray, const Slice& verts, const Index triIdx, const Index i0, const Index i1, const Index i2 ) const;
	static void PrecomputeTriangle( const Slice& vert, const uint32_t ti0, const uint32_t ti1, const uint32_t ti2, void* dst );
	static Float SA( const Vec3& aabbMin, const Vec3& aabbMax );
};

template <typename Float, typename Index> class BVH : public BVHBase<Float, Index>
{
public:
	using Base = BVHBase<Float, Index>;
	using typename Base::Vec3;
	using typename Base::Vertex;
	using typename Base::Slice;
	using typename Base::Fragment;
	using typename Base::Ray;
	using typename Base::BVH_Verbose;
	using typename Base::BLASInstance;
	using Base::rebuildable;
	using Base::refittable;
	using Base::may_have_holes;
	using Base::bvh_over_aabbs;
	using Base::bvh_over_indices;
	using Base::threadedBuild;
	using Base::settings;
	using Base::c_trav;
	using Base::c_int;
	using Base::context;
	using Base::layout;
	using Base::allocatedNodes;
	using Base::usedNodes;
	using Base::triCount;
	using Base::idxCount;
	using Base::l_quads;
	using Base::hqbvhbins;
	using Base::hqbvhoddeven;
	using Base::aabbMin;
	using Base::aabbMax;
	using Base::opmapN;
	using Base::opmap;
	using Base::AlignedAlloc;
	using Base::AlignedFree;
	using Base::ContextNew;
	using Base::ContextDelete;
	using Base::CopyBasePropertiesFrom;
protected:
	using Base::SA;
	using Base::IntersectTri;
	using Base::TriOccludes;
public:
	template <typename, typename> friend class BVH_GPU;
	template <typename, typename> friend class BVH4_CPU;
	template <typename, typename> friend class BVH8_CPU;
	template <typename, typename> friend class BVH8_CWBVH;
#ifdef ENABLE_BVH_SOA
	template <typename, typename> friend class BVH_SoA;
#endif
	template <int, typename, typename> friend class MBVH;
	struct SubdivTask { Index node, sliceStart, sliceEnd; uint32_t depth; };
	struct BVHNode
	{
		// 'Traditional' 32-byte BVH node layout, as proposed by Ingo Wald.
		// When aligned to a cache line boundary, two of these fit together.
		// The double precision version has 64-bit indices and is 64 bytes.
		Vec3 aabbMin; Index leftFirst; // 16 bytes
		Vec3 aabbMax; Index triCount;  // 16 bytes
		bool isLeaf() const { return triCount > 0; /* empty BVH leaves do not exist */ }
		bool Intersect( const Vec3& bmin, const Vec3& bmax ) const;
		Float SurfaceArea() const { return BVHBase<Float, Index>::SA( aabbMin, aabbMax ); }
	};
	BVH( BVHContext ctx = {} ) { layout = LAYOUT_BVH; context = ctx; }
	BVH( const BVH_Verbose& original ) { layout = LAYOUT_BVH; ConvertFrom( original ); }
	BVH( const Vertex* vertices, const Index primCount ) { layout = LAYOUT_BVH; Build( vertices, primCount ); }
	BVH( const Slice& vertices ) { layout = LAYOUT_BVH; Build( vertices ); }
	BVH( BVH&& ) noexcept;
	BVH& operator=( const BVH& other ) = default;
	~BVH();
	void Save( const char* fileName );
	bool Load( const char* fileName, const Vertex* vertices, const Index primCount );
	bool Load( const char* fileName, const Vertex* vertices, const uint32_t* indices, const Index primCount );
	bool Load( const char* fileName, const Slice& vertices, const uint32_t* indices = 0, const Index primCount = 0 );
	void BuildQuick( const Vertex* vertices, const Index primCount );
	void BuildQuick( const Slice& vertices );
	void Build( const Vertex* vertices, const Index primCount );
	void Build( const Slice& vertices );
	void Build( const Vertex* vertices, const uint32_t* indices, const Index primCount );
	void Build( const Slice& vertices, const uint32_t* indices, const Index primCount );
	void Build( BLASInstance* instances, const Index instCount, BVHBase<Float, Index>** blasses, const Index blasCount );
	void Build( BLASInstance* instances, const Index instCount, BVH** blasses, const Index blasCount );
	void Build( void (*customGetAABB)(const Index, Vec3&, Vec3&, void*), const Index primCount );
	void BuildAABB( const Vertex* aabbs, const Index primCount );
	void BuildHQ( const Vertex* vertices, const Index primCount );
	void BuildHQ( const Slice& vertices );
	void BuildHQ( const Vertex* vertices, const uint32_t* indices, const Index primCount );
	void BuildHQ( const Slice& vertices, const uint32_t* indices, const Index primCount );
	// SIMD builders. These exist for BVH<float, uint32_t> on the matching platform
	// (tiny_bvh_x86_float.h, tiny_bvh_arm_float.h); other instantiations raise an error.
	void BuildAVX( const Vertex* vertices, const Index primCount );
	void BuildAVX( const Slice& vertices );
	void BuildAVX( const Vertex* vertices, const uint32_t* indices, const Index primCount );
	void BuildAVX( const Slice& vertices, const uint32_t* indices, const Index primCount );
	void BuildNEON( const Vertex* vertices, const Index primCount );
	void BuildNEON( const Slice& vertices );
	void BuildNEON( const Vertex* vertices, const uint32_t* indices, const Index primCount );
	void BuildNEON( const Slice& vertices, const uint32_t* indices, const Index primCount );
	int32_t Intersect( Ray& ray ) const;
	bool IsOccluded( const Ray& ray ) const;
	bool IntersectSphere( const Vec3& pos, const Float r ) const;
	void Intersect256Rays( Ray* first ) const;
	void Intersect256RaysSSE( Ray* packet ) const; // requires BVH_USEAVX
	bool IsOccludedTLAS( const Ray& ray ) const;
	int32_t IntersectTLAS( Ray& ray ) const;
	void ConvertFrom( const BVH_Verbose& original, bool compact = true );
	void SplitLeafs( const Index maxPrims );
	void CombineLeafs( const Index nodeIdx = 0 );
	Float SAHCost( const Index nodeIdx = 0, uint32_t depth = 0 ) const;
	Float EPOCost( const Index nodeIdx = 0, uint32_t depth = 0 ) const;
	Index CollectNodes( const Index root, Index* list, const Index cap ) const;
	static void MetricTask( const uint32_t task, void* payload );
	void Refit( const Index nodeIdx = 0 );
	void Compact();
	void Optimize( const uint32_t iterations = 25, bool extreme = false, bool stochastic = false );
	int32_t NodeCount() const;
	int32_t LeafCount() const;
	int32_t PrimCount( const Index nodeIdx = 0 ) const;
	// BVH type identification
	bool isTLAS() const { return instList != 0; }
	bool isBLAS() const { return instList == 0; }
	bool isIndexed() const { return vertIdx != 0; }
	bool hasCustomGeom() const { return customIntersect != 0; }
	// internal methods that need to be public: these are scheduled via the threading hooks.
	void Build( Index nodeIdx = 0, uint32_t depth = 0 );
	void BuildFullSweep( Index nodeIdx = 0, uint32_t depth = 0 );
	void BuildHQTask( Index nodeIdx, uint32_t depth, Index sliceStart, Index sliceEnd, Index* triIdxB );
	void BuildSIMDSubtree( Index nodeIdx = 0, uint32_t depth = 0 );
	void PrepareSIMDBuildFragSlice( const Index first, const Index last, const uint32_t* indices,
		const int8_t* vertData, const uint32_t stride4, void* frags, Float* rootMin, Float* rootMax );
	void BuildSIMDBinTask( const Index first, const Index last, void* binbox,
		uint32_t* count, const Float* nmin4, const Float* rpd4 );
private:
	void PrepareSIMDBuild( const Slice& vertices, const uint32_t* indices, const Index primCount );
	void BuildSIMDFinalize();
	void PrepareHQBuild( const Slice& vertices, const uint32_t* indices, const Index prims );
	void BuildHQ();
	void PrepareBuild( const Slice& vertices, const uint32_t* indices, const Index primCount );
	bool ClipFrag( const Fragment& orig, Fragment& newFrag, Vec3 bmin, Vec3 bmax, const uint32_t splitAxis ) const;
	bool SplitFrag( const Fragment& orig, Fragment& left, Fragment& right, const uint32_t splitAxis, const Float splitPos ) const;
	Index CombineLeafs( const Index primCount, Index& firstIdx, Index nodeIdx = 0 );
	Float SplitPriority( const Fragment& f ) const;
	Index Presplit();
	static Float GetNodeSize( const Float extent, const Float globalSize );
	void PresplitPostPass();
	inline Float SplitCostSAH( const Float rAparent, const Float Aleft, const Index Nleft, const Float Aright, const Index Nright ) const;
	inline Float NoSplitCostSAH( const Index Nparent ) const;
	Float EPOArea( const Index subtreeRoot, const Index nodeIdx = 0 ) const;
	Float TriArea( const Index triIdx ) const;
	Float PrimArea( const Index slot ) const;
protected:
	template <bool posX, bool posY, bool posZ> int32_t IntersectOctant( Ray& ray ) const;
	template <bool posX, bool posY, bool posZ> int32_t IntersectTLASOctant( Ray& ray ) const;
	template <bool posX, bool posY, bool posZ> bool IsOccludedOctant( const Ray& ray ) const;
	template <bool posX, bool posY, bool posZ> bool IsOccludedTLASOctant( const Ray& ray ) const;
public:
	// Basic BVH data
	Slice verts = {};				// pointer to input primitive array: 3 vertices per tri.
	uint32_t* vertIdx = 0;			// vertex indices, only used in case the BVH is built over indexed prims.
	Index* primIdx = 0;				// primitive index array.
	uint32_t* rrsHits = 0;			// for RDH: ray hit count per triangle.
	BLASInstance* instList = 0;		// instance array, for top-level acceleration structure.
	BVHBase<Float, Index>** blasList = 0; // blas array, for TLAS traversal.
	Index blasCount = 0;			// number of blasses in blasList.
	BVHNode* bvhNode = 0;			// BVH node pool, Wald 32-byte format. Root is always in node 0.
	Index newNodePtr = 0;			// used during build to keep track of next free node in pool.
	Index nextFrag = 0;				// used during SBVH build to keep track of next free fragment.
	Fragment* fragment = 0;			// input primitive bounding boxes.
	// Custom geometry intersection callback
	bool (*customIntersect)(Ray&, const Index, void*) = 0;
	bool (*customIsOccluded)(const Ray&, const Index, void*) = 0;
	void* customUserdata = 0;
private:
#ifdef ENABLE_THREADED_BUILDS
	// Atomic counters for threaded builds
	std::atomic<Index>* atomicNewNodePtr = 0;
	std::atomic<Index>* atomicNextFrag = 0;
#endif
	BVHNode* leafNodes = 0; // will point inside node array
	Index* scratchPad = 0; // for sorting
	Index numLeafNodes = 0;
	Index numInternalNodes = 0;
	// data for full sweep builder
	struct SweepBounds { Vec3 bmin, bmax; };	// 24 bytes: fragment bounds, in sorted order.
	uint8_t* flag = 0;
	Index* sortedIdx[3] = { 0 };
	SweepBounds* sortedBnds[3] = { 0 };	// per-axis fragment bounds; keeps the sweeps linear.
	SweepBounds* tmpBnds = 0;			// scratch for the stable partition of sortedBnds.
	Float* SARs = 0;
	void GatherSweepBounds( uint32_t axis );
	static void SweepGatherTask( uint32_t axis, void* payload );
};

template <typename Float, typename Index> class BVH_GPU : public BVHBase<Float, Index>
{
public:
	using Base = BVHBase<Float, Index>;
	using typename Base::Vec3;
	using typename Base::Vertex;
	using typename Base::Slice;
	using typename Base::Ray;
	using typename Base::BVH;
	using typename Base::BLASInstance;
	using Base::may_have_holes;
	using Base::bvh_over_aabbs;
	using Base::settings;
	using Base::c_trav;
	using Base::c_int;
	using Base::context;
	using Base::layout;
	using Base::allocatedNodes;
	using Base::usedNodes;
	using Base::triCount;
	using Base::idxCount;
	using Base::aabbMin;
	using Base::aabbMax;
	using Base::AlignedAlloc;
	using Base::AlignedFree;
	using Base::CopyBasePropertiesFrom;
	struct BVHNode
	{
		// Alternative 64-byte BVH node layout, which specifies the bounds of
		// the children rather than the node itself. This layout is used by
		// Aila and Laine in their seminal GPU ray tracing paper.
		Vec3 lmin; uint32_t left;
		Vec3 lmax; uint32_t right;
		Vec3 rmin; uint32_t triCount;
		Vec3 rmax; uint32_t firstTri; // total: 64 bytes
		bool isLeaf() const { return triCount > 0; }
	};
	BVH_GPU( BVHContext ctx = {} ) { layout = LAYOUT_BVH_GPU; context = ctx; }
	BVH_GPU( const BVH& original ) { /* DEPRICATED */ ConvertFrom( original ); }
	BVH_GPU( BVH_GPU&& );
	BVH_GPU& operator=( const BVH_GPU& ) = default;
	~BVH_GPU();
	void Build( const bvhvec4* vertices, const Index primCount );
	void Build( const Slice& vertices );
	void Build( const bvhvec4* vertices, const uint32_t* indices, const Index primCount );
	void Build( const Slice& vertices, const uint32_t* indices, const Index primCount );
	void Build( BLASInstance* instances, const Index instCount, BVHBase<Float, Index>** blasses, const Index blasCount );
	void BuildAABB( const bvhvec4* aabbs, const Index primCount );
	void BuildHQ( const bvhvec4* vertices, const Index primCount );
	void BuildHQ( const Slice& vertices );
	void BuildHQ( const bvhvec4* vertices, const uint32_t* indices, const Index primCount );
	void BuildHQ( const Slice& vertices, const uint32_t* indices, const Index primCount );
	void Optimize( const uint32_t iterations = 25, bool extreme = false );
	Float SAHCost( const Index nodeIdx = 0 ) { return bvh.SAHCost( nodeIdx ); }
	void ConvertFrom( const BVH& original, bool compact = true );
	int32_t Intersect( Ray& ray ) const;
	bool IsOccluded( const Ray& ray ) const { FALLBACK_SHADOW_QUERY( ray ); }
	// BVH data
	BVHNode* bvhNode = 0;			// BVH node in Aila & Laine format.
	BVH bvh;						// BVH_GPU is created from BVH and uses its data.
	Slice orderedVerts = {};		// Vertex data ordered according to triIdx.
	bool ownBVH = true;				// False when ConvertFrom receives an external bvh.
};

#ifdef ENABLE_BVH_SOA

template <typename Float, typename Index> class BVH_SoA : public BVHBase<Float, Index>
{
public:
	using Base = BVHBase<Float, Index>;
	using typename Base::Vertex;
	using typename Base::Slice;
	using typename Base::Ray;
	using typename Base::BVH;
	using Base::settings;
	using Base::c_trav;
	using Base::c_int;
	using Base::context;
	using Base::layout;
	using Base::allocatedNodes;
	using Base::usedNodes;
	using Base::triCount;
	using Base::aabbMin;
	using Base::aabbMax;
	using Base::AlignedAlloc;
	using Base::AlignedFree;
	using Base::CopyBasePropertiesFrom;
protected:
	using Base::IntersectTri;
	using Base::TriOccludes;
public:
	struct BVHNode
	{
		// Second alternative 64-byte BVH node layout, same as BVHAilaLaine but
		// with child AABBs stored in SoA order: { left.min, left.max, right.min, right.max }.
		Float xxxx[4], yyyy[4], zzzz[4];
		Index left, right, triCount, firstTri; // total: 64 bytes
		bool isLeaf() const { return triCount > 0; }
	};
	BVH_SoA( BVHContext ctx = {} ) { layout = LAYOUT_BVH_SOA; context = ctx; }
	BVH_SoA( const BVH& original ) { /* DEPRICATED */ layout = LAYOUT_BVH_SOA; ConvertFrom( original ); }
	BVH_SoA( BVH_SoA&& );
	BVH_SoA& operator=( const BVH_SoA& ) = default;
	~BVH_SoA();
	void Build( const Vertex* vertices, const Index primCount );
	void Build( const Slice& vertices );
	void Build( const Vertex* vertices, const uint32_t* indices, const Index primCount );
	void Build( const Slice& vertices, const uint32_t* indices, const Index primCount );
	void BuildHQ( const Vertex* vertices, const Index primCount );
	void BuildHQ( const Slice& vertices );
	void BuildHQ( const Vertex* vertices, const uint32_t* indices, const Index primCount );
	void BuildHQ( const Slice& vertices, const uint32_t* indices, const Index primCount );
	void Optimize( const uint32_t iterations = 25, bool extreme = false );
	Float SAHCost( const Index nodeIdx = 0 ) { return bvh.SAHCost( nodeIdx ); }
	void Save( const char* fileName );
	bool Load( const char* fileName, const Vertex* vertices, const Index primCount );
	bool Load( const char* fileName, const Vertex* vertices, const uint32_t* indices, const Index primCount );
	bool Load( const char* fileName, const Slice& vertices, const uint32_t* indices = 0, const Index primCount = 0 );
	void ConvertFrom( const BVH& original, bool compact = true );
	int32_t Intersect( Ray& ray ) const;
	bool IsOccluded( const Ray& ray ) const;
	// BVH data
	BVHNode* bvhNode = 0;			// BVH node in 'structure of arrays' format.
	BVH bvh;						// BVH_SoA is created from BVH and uses its data.
	bool ownBVH = true;				// False when ConvertFrom receives an external bvh.
};

#endif

template <typename Float, typename Index> class BVH_Verbose : public BVHBase<Float, Index>
{
public:
	using Base = BVHBase<Float, Index>;
	using typename Base::Vec3;
	using typename Base::Slice;
	using typename Base::Fragment;
	using typename Base::BVH;
	using Base::refittable;
	using Base::may_have_holes;
	using Base::bvh_over_indices;
	using Base::c_trav;
	using Base::c_int;
	using Base::context;
	using Base::layout;
	using Base::allocatedNodes;
	using Base::usedNodes;
	using Base::triCount;
	using Base::idxCount;
	using Base::aabbMin;
	using Base::aabbMax;
	using Base::AlignedAlloc;
	using Base::AlignedFree;
	using Base::CopyBasePropertiesFrom;
protected:
	using Base::SA;
public:
	struct BVHNode
	{
		// This node layout has some extra data per node: It stores left and right
		// child node indices explicitly, and stores the index of the parent node.
		// This format exists primarily for the BVH optimizer.
		Vec3 aabbMin; Index left;
		Vec3 aabbMax; Index right;
		Index triCount, firstTri, parent;
		Float dummy[5]; // total: 64 bytes.
		bool isLeaf() const { return triCount > 0; }
		Float SA() const { return BVHBase<Float, Index>::SA( aabbMin, aabbMax ); }
	};
	BVH_Verbose( BVHContext ctx = {} ) { layout = LAYOUT_BVH_VERBOSE; context = ctx; }
	BVH_Verbose( const BVH& original ) { /* DEPRECATED */ layout = LAYOUT_BVH_VERBOSE; ConvertFrom( original ); }
	BVH_Verbose( BVH_Verbose&& );
	BVH_Verbose& operator=( const BVH_Verbose& ) = default;
	~BVH_Verbose();
	void ConvertFrom( const BVH& original, bool compact = true );
	Float SAHCost( const Index nodeIdx = 0 ) const;
	int32_t NodeCount() const;
	int32_t PrimCount( const Index nodeIdx = 0 ) const;
	void Refit( const Index nodeIdx = 0, bool skipLeafs = false );
	void CheckFit( const Index nodeIdx = 0, bool skipLeafs = false );
	void Compact();
	void SortIndices();
	void SplitLeafs( const Index maxPrims = 1 );
	void MergeLeafs();
	void Optimize( const uint32_t iterations = 25, bool extreme = false, bool stochastic = false );
private:
	struct SortItem { Index idx; Float cost; };
	struct RefitRecord { Vec3 bmin; Index node; Vec3 bmax; Index dummy; }; // 32 bytes
	double RefitUp( Index nodeIdx, RefitRecord* journal, uint32_t& journalPtr, const uint32_t journalCap );
	Float SAHCostUp( Index nodeIdx ) const;
	Index FindBestNewPosition( const Index Lid, Float& cost ) const;
	Index CountSubtreeTris( const Index nodeIdx, Index* counters );
	void MergeSubtree( const Index nodeIdx, Index* newIdx, Index& newIdxPtr );
public:
	// BVH data
	Slice verts = {};				// pointer to input primitive array: 3 vertices per tri.
	Fragment* fragment = 0;			// input primitive bounding boxes.
	Index* primIdx = 0;				// primitive index array - pointer copied from original.
	BVHNode* bvhNode = 0;			// BVH node with additional info, for BVH optimizer.
};

template <int M, typename Float, typename Index> class MBVH : public BVHBase<Float, Index>
{
public:
	using Base = BVHBase<Float, Index>;
	using typename Base::Vec3;
	using typename Base::Vertex;
	using typename Base::Slice;
	using typename Base::Ray;
	using typename Base::BVH;
	using Base::may_have_holes;
	using Base::settings;
	using Base::c_trav;
	using Base::c_int;
	using Base::context;
	using Base::layout;
	using Base::allocatedNodes;
	using Base::usedNodes;
	using Base::triCount;
	using Base::l_quads;
	using Base::aabbMin;
	using Base::aabbMax;
	using Base::AlignedAlloc;
	using Base::AlignedFree;
	using Base::CopyBasePropertiesFrom;
protected:
	using Base::SA;
public:
	struct MBVHNode
	{
		// M-wide (aka 'shallow') BVH layout.
		Vec3 aabbMin; Index firstTri;
		Vec3 aabbMax; Index triCount;
		Index child[M];
		uint32_t childCount;
		uint32_t dummy[((30 - M) & 3) + 1]; // dummies are for alignment.
		bool isLeaf() const { return triCount > 0; }
	};
	MBVH( BVHContext ctx = {} ) { layout = LAYOUT_MBVH; context = ctx; }
	MBVH( const BVH& original ) { /* DEPRECATED */ layout = LAYOUT_MBVH; ConvertFrom( original ); }
	MBVH( MBVH&& );
	MBVH& operator=( const MBVH& ) = default;
	~MBVH();
	void Build( const Vertex* vertices, const Index primCount );
	void Build( const Slice& vertices );
	void Build( const Vertex* vertices, const uint32_t* indices, const Index primCount );
	void Build( const Slice& vertices, const uint32_t* indices, const Index primCount );
	void BuildHQ( const Vertex* vertices, const Index primCount );
	void BuildHQ( const Slice& vertices );
	void BuildHQ( const Vertex* vertices, const uint32_t* indices, const Index primCount );
	void BuildHQ( const Slice& vertices, const uint32_t* indices, const Index primCount );
	void Optimize( const uint32_t iterations = 25, bool extreme = false );
	void Refit( const Index nodeIdx = 0 );
	Index LeafCount( const Index nodeIdx = 0 ) const;
	Float SAHCost( const Index nodeIdx = 0 ) const;
	void ConvertFrom( const BVH& original, bool compact = true );
	// BVH data
	MBVHNode* mbvhNode = 0;			// BVH node for M-wide BVH.
	BVH bvh;						// MBVH<M> is created from BVH and uses its data.
	bool ownBVH = true;				// False when ConvertFrom receives an external bvh.
	// collapse settings, consumed by ConvertFrom
	Index leafPrimLimit = 0;		// max prims the collapse may merge into one leaf; 0 disables merging
};

template <typename Float, typename Index> class BVH4_GPU : public BVHBase<Float, Index>
{
public:
	using Base = BVHBase<Float, Index>;
	using typename Base::Vec3;
	using typename Base::Slice;
	using typename Base::Ray;
	using typename Base::BVH;
	using Base::settings;
	using Base::c_trav;
	using Base::c_int;
	using Base::context;
	using Base::layout;
	using Base::allocatedNodes;
	using Base::usedNodes;
	using Base::triCount;
	using Base::aabbMin;
	using Base::aabbMax;
	using Base::AlignedAlloc;
	using Base::AlignedFree;
	using Base::CopyBasePropertiesFrom;
protected:
	using Base::PrecomputeTriangle;
public:
	struct BVHNode // actual struct is unused; left here to show structure of data in bvh4Data.
	{
		// 4-way BVH node, optimized for GPU rendering
		struct aabb8 { uint8_t xmin, ymin, zmin, xmax, ymax, zmax; }; // quantized
		Vec3 aabbMin; uint32_t c0Info;			// 16
		Vec3 aabbExt; uint32_t c1Info;			// 16
		aabb8 c0bounds, c1bounds; uint32_t c2Info;	// 16
		aabb8 c2bounds, c3bounds; uint32_t c3Info;	// 16; total: 64 bytes
		// childInfo, 32bit:
		// msb:        0=interior, 1=leaf
		// leaf:       16 bits: relative start of triangle data, 15 bits: triangle count.
		// interior:   31 bits: child node address, in float4s from BVH data start.
		// Triangle data: directly follows nodes with leaves. Per tri:
		// - bvhvec4 vert0, vert1, vert2
		// - uint vert0.w stores original triangle index.
		// We can make the node smaller by storing child nodes sequentially, but
		// there is no way we can shave off a full 16 bytes, unless aabbExt is stored
		// as chars as well, as in CWBVH.
	};
	BVH4_GPU( BVHContext ctx = {} ) { layout = LAYOUT_BVH4_GPU; context = ctx; }
	BVH4_GPU( const MBVH<4, Float, Index>& bvh4 ) { /* DEPRECATED */ layout = LAYOUT_BVH4_GPU; ConvertFrom( bvh4 ); }
	BVH4_GPU( BVH4_GPU&& );
	BVH4_GPU& operator=( const BVH4_GPU& ) = default;
	~BVH4_GPU();
	void Build( const bvhvec4* vertices, const Index primCount );
	void Build( const Slice& vertices );
	void Build( const bvhvec4* vertices, const uint32_t* indices, const Index primCount );
	void Build( const Slice& vertices, const uint32_t* indices, const Index primCount );
	void BuildHQ( const bvhvec4* vertices, const Index primCount );
	void BuildHQ( const Slice& vertices );
	void BuildHQ( const bvhvec4* vertices, const uint32_t* indices, const Index primCount );
	void BuildHQ( const Slice& vertices, const uint32_t* indices, const Index primCount );
	void Optimize( const uint32_t iterations = 25, bool extreme = false );
	void ConvertFrom( const MBVH<4, Float, Index>& original, bool compact = true );
	Float SAHCost( const Index nodeIdx = 0 ) const { return bvh4.SAHCost( nodeIdx ); }
	int32_t Intersect( Ray& ray ) const;
	bool IsOccluded( const Ray& ray ) const { FALLBACK_SHADOW_QUERY( ray ); }
	// BVH data
	bvhvec4* bvh4Data = 0;			// 64-byte 4-wide BVH node for efficient GPU rendering.
	uint32_t allocatedBlocks = 0;	// node data and triangles are stored in 16-byte blocks.
	uint32_t usedBlocks = 0;		// actually used storage.
	MBVH<4, Float, Index> bvh4;		// BVH4_GPU is created from BVH4 and uses its data.
	bool ownBVH4 = true;			// False when ConvertFrom receives an external bvh.
};

template <typename Float, typename Index> class BVH4_CPU : public BVHBase<Float, Index>
{
public:
	using Base = BVHBase<Float, Index>;
	using Base::opmap;
	using Base::opmapN;
	using typename Base::Vec3;
	using typename Base::Vertex;
	using typename Base::Slice;
	using typename Base::Ray;
	using typename Base::BVH;
	using typename Base::BVHTri4Leaf;
	using Base::settings;
	using Base::c_trav;
	using Base::c_int;
	using Base::context;
	using Base::layout;
	using Base::usedNodes;
	using Base::triCount;
	using Base::l_quads;
	using Base::aabbMin;
	using Base::aabbMax;
	using Base::AlignedAlloc;
	using Base::AlignedFree;
	using Base::CopyBasePropertiesFrom;
	enum { EMPTY_BIT = 1 << 31, LEAF_BIT = 1 << 30 };
	struct ALIGNED( 64 ) BVHNode
	{
		// 4-way BVH node, optimized for CPU rendering. The SSE and NEON kernels load the
		// bounds of the four children as vectors; the layout is 128 bytes for float.
		Float xmin[4], xmax[4];
		Float ymin[4], ymax[4];
		Float zmin[4], zmax[4];
		uint32_t child[4];		// bit 30: leaf flag, bit 31: empty slot, bits 28..0: block index.
		uint32_t perm[4];		// sorted lane order per octant, see ConvertFrom.
	};
	struct ALIGNED( 64 ) CacheLine { uint8_t data[64]; };
	BVH4_CPU( BVHContext ctx = {} ) { layout = LAYOUT_BVH4_CPU; context = ctx; c_int = 2; l_quads = true; }
	BVH4_CPU( BVH4_CPU&& );
	BVH4_CPU& operator=( const BVH4_CPU& ) = default;
	~BVH4_CPU();
	void Save( const char* fileName );
	bool Load( const char* fileName, const Index expectedTris );
	void Build( const Vertex* vertices, const Index primCount );
	void Build( const Slice& vertices );
	void Build( const Vertex* vertices, const uint32_t* indices, const Index prims );
	void Build( const Slice& vertices, const uint32_t* indices, Index prims );
	void BuildHQ( const Vertex* vertices, const Index primCount );
	void BuildHQ( const Slice& vertices );
	void BuildHQ( const Vertex* vertices, const uint32_t* indices, const Index prims );
	void BuildHQ( const Slice& vertices, const uint32_t* indices, Index prims );
	void Optimize( const uint32_t iterations, bool extreme );
	void Refit();
	Float SAHCost( const Index nodeIdx ) const;
	void ConvertFrom( MBVH<4, Float, Index>& original );
	int32_t Intersect( Ray& ray ) const;
	bool IsOccluded( const Ray& ray ) const;
	// Traversal kernels specialized for the ray octant; the platform headers provide these.
	template <bool posX, bool posY, bool posZ> int32_t IntersectOctant( Ray& ray ) const;
	template <bool posX, bool posY, bool posZ> bool IsOccludedOctant( const Ray& ray ) const;
	// BVH4 data
	CacheLine* bvh4Data = 0;		// Interleaved interior (128b) and leaf (192b) data.
	MBVH<4, Float, Index> bvh4;		// BVH4_CPU is created from BVH4 and uses its data.
	bool ownBVH4 = true;			// false when ConvertFrom receives an external bvh4.
	uint32_t allocatedBlocks = 0;	// node data and triangles are stored in 64-byte blocks.
	uint32_t usedBlocks = 0;		// the amount of data actually used.
};

template <typename Float, typename Index> class BVH8_CWBVH : public BVHBase<Float, Index>
{
public:
	using Base = BVHBase<Float, Index>;
	using typename Base::Vec3;
	using typename Base::Slice;
	using typename Base::Ray;
	using typename Base::BVH;
	using Base::settings;
	using Base::c_trav;
	using Base::c_int;
	using Base::context;
	using Base::layout;
	using Base::triCount;
	using Base::idxCount;
	using Base::aabbMin;
	using Base::aabbMax;
	using Base::AlignedAlloc;
	using Base::AlignedFree;
	using Base::CopyBasePropertiesFrom;
protected:
	using Base::PrecomputeTriangle;
public:
	BVH8_CWBVH( BVHContext ctx = {} ) { layout = LAYOUT_CWBVH; context = ctx; }
	BVH8_CWBVH( MBVH<8, Float, Index>& bvh8 ) { /* DEPRECATED */ layout = LAYOUT_CWBVH; ConvertFrom( bvh8 ); }
	BVH8_CWBVH( BVH8_CWBVH&& );
	BVH8_CWBVH& operator=( const BVH8_CWBVH& ) = default;
	~BVH8_CWBVH();
	void Save( const char* fileName );
	bool Load( const char* fileName, const Index expectedTris );
	void Build( const bvhvec4* vertices, const Index primCount );
	void Build( const Slice& vertices );
	void Build( const bvhvec4* vertices, const uint32_t* indices, const Index primCount );
	void Build( const Slice& vertices, const uint32_t* indices, const Index primCount );
	void BuildHQ( const bvhvec4* vertices, const Index primCount );
	void BuildHQ( const Slice& vertices );
	void BuildHQ( const bvhvec4* vertices, const uint32_t* indices, const Index primCount );
	void BuildHQ( const Slice& vertices, const uint32_t* indices, const Index primCount );
	void Optimize( const uint32_t iterations = 25, bool extreme = false );
	void ConvertFrom( const MBVH<8, Float, Index>& original, bool compact = true );
	Float SAHCost( const Index nodeIdx = 0 ) const;
	int32_t Intersect( Ray& ray ) const;
	bool IsOccluded( const Ray& ray ) const { FALLBACK_SHADOW_QUERY( ray ); }
	// BVH8 data
	bvhvec4* bvh8Data = 0;			// nodes in CWBVH format.
	bvhvec4* bvh8Tris = 0;			// triangle data for CWBVH nodes.
	uint32_t allocatedBlocks = 0;	// node data is stored in blocks of 16 byte.
	uint32_t usedBlocks = 0;		// actually used blocks.
	MBVH<8, Float, Index> bvh8;		// BVH8_CWBVH is created from BVH8 and uses its data.
	bool ownBVH8 = true;			// false when ConvertFrom receives an external bvh8.
	uint32_t usedTriBlocks = 0;		// 16-byte blocks actually used in bvh8Tris
	uint32_t allocatedTris = 0;		// 16-byte blocks allocated for bvh8Tris
};

template <typename Float, typename Index> struct ALIGNED( 64 ) BVHTri4Leaf
{
	using Vec3 = typename bvh_traits<Float>::vec3;
	Float v0x[4], v0y[4], v0z[4];
	Float e1x[4], e1y[4], e1z[4];
	Float e2x[4], e2y[4], e2z[4];
	Index primIdx[4];			// total: 160 bytes for float, padded to 3 full cachelines.
	inline void SetData( const Vec3& v0, const Vec3& e1, const Vec3& e2, const Index pidx, const uint32_t slot )
	{
		v0x[slot] = v0.x, v0y[slot] = v0.y, v0z[slot] = v0.z;
		e1x[slot] = e1.x, e1y[slot] = e1.y, e1z[slot] = e1.z;
		e2x[slot] = e2.x, e2y[slot] = e2.y, e2z[slot] = e2.z;
		primIdx[slot] = pidx;
	}
	// Scalar counterpart of the SIMD leaf tests: Moeller-Trumbore for triangle 'slot', with the
	// opacity map applied. Returns true for a hit in (0, tmax) and sets t, u and v.
	inline bool Intersect( const uint32_t slot, const Ray<Float, Index>& ray, const Float tmax, Float& t, Float& u, Float& v, const uint32_t* opmap, const uint32_t opmapN ) const
	{
		const Vec3 e1( e1x[slot], e1y[slot], e1z[slot] ), e2( e2x[slot], e2y[slot], e2z[slot] );
		const Vec3 h = tinybvh_cross( ray.D, e2 );
		const Float det = tinybvh_dot( e1, h );
		if (det == 0) return false;
		const Float f = 1 / det;
		const Vec3 s = ray.O - Vec3( v0x[slot], v0y[slot], v0z[slot] );
		u = f * tinybvh_dot( s, h );
		const Vec3 q = tinybvh_cross( s, e1 );
		v = f * tinybvh_dot( ray.D, q );
		if (!(u >= 0 && v >= 0 && u + v <= 1)) return false;
		t = f * tinybvh_dot( e2, q );
		if (!(t > 0 && t < tmax)) return false;
		if (opmap)
		{
			const Float fN = (Float)opmapN;
			const int row = int( (u + v) * fN ), diag = int( (1 - u) * fN );
			const int idx = (row * row) + int( v * fN ) + (diag - (opmapN - 1 - row));
			const uint32_t* om = opmap + primIdx[slot] * ((opmapN * opmapN + 31) >> 5);
			if (!(om[idx >> 5] & (1 << (idx & 31)))) return false;
		}
		return true;
	}
};

// Storage for a single triangle, for BVH8_CPU.
template <typename Float, typename Index> struct BVHTri1Leaf
{
	using Vec3 = typename bvh_traits<Float>::vec3;
	Vec3 v0, e1, e2;
	Index primIdx;				// total: 40 bytes
};

template <typename Float, typename Index> class BVH8_CPU : public BVHBase<Float, Index>
{
public:
	using Base = BVHBase<Float, Index>;
	using Base::opmap;
	using Base::opmapN;
	using typename Base::Vec3;
	using typename Base::Vertex;
	using typename Base::Slice;
	using typename Base::Ray;
	using typename Base::BVH;
	using typename Base::BVHTri4Leaf;
	using Base::settings;
	using Base::c_trav;
	using Base::c_int;
	using Base::context;
	using Base::layout;
	using Base::usedNodes;
	using Base::triCount;
	using Base::l_quads;
	using Base::aabbMin;
	using Base::aabbMax;
	using Base::AlignedAlloc;
	using Base::AlignedFree;
	using Base::CopyBasePropertiesFrom;
	enum { EMPTY_BIT = 1 << 31, LEAF_BIT = 1 << 30 };
	struct ALIGNED( 64 ) BVHNode
	{
		// 8-way BVH node, optimized for CPU rendering.
		// Based on: "Accelerated Single Ray Tracing for Wide Vector Units", Fuetterling et al., 2017,
		// and the implementation by Mathijs Molenaar, https://github.com/mathijs727/pandora
		Float xmin[8], xmax[8];
		Float ymin[8], ymax[8];
		Float zmin[8], zmax[8];
		uint32_t child[8];		// bit 30: leaf flag, bits 28..0: index of child node or leaf.
		uint32_t perm[8];		// sorted lane order per octant, see ConvertFrom.
	};
	struct BVHNodeCompact
	{
		// Novel 128-byte 8-way BVH node.
		uint32_t child[8], perm[8];									// 64
		float d0, bminx, bminy, bminz, d1, bextx, bexty, bextz;		// 32
		uint32_t cbminmaxx[4], cbminmaxy[4];						// 32, total: 128
	};
	struct ALIGNED( 64 ) CacheLine { uint8_t data[64]; };
	BVH8_CPU( BVHContext ctx = {} ) { layout = LAYOUT_BVH8_AVX2; context = ctx; c_int = 2; l_quads = true; }
	BVH8_CPU( BVH8_CPU&& );
	BVH8_CPU& operator=( const BVH8_CPU& ) = default;
	~BVH8_CPU();
	void Save( const char* fileName );
	bool Load( const char* fileName, const Index expectedTris );
	void Build( const Vertex* vertices, const Index primCount );
	void Build( const Slice& vertices );
	void Build( const Vertex* vertices, const uint32_t* indices, const Index prims );
	void Build( const Slice& vertices, const uint32_t* indices, Index prims );
	void BuildHQ( const Vertex* vertices, const Index primCount );
	void BuildHQ( const Slice& vertices );
	void BuildHQ( const Vertex* vertices, const uint32_t* indices, const Index prims );
	void BuildHQ( const Slice& vertices, const uint32_t* indices, Index prims );
	void Optimize( const uint32_t iterations, bool extreme );
	void Refit();
	Float SAHCost( const Index nodeIdx ) const;
	void ConvertFrom( MBVH<8, Float, Index>& original );
	int32_t Intersect( Ray& ray ) const;
	bool IsOccluded( const Ray& ray ) const;
	// Traversal kernels specialized for the ray octant; the platform headers provide these.
	template <bool posX, bool posY, bool posZ> int32_t IntersectOctant( Ray& ray ) const;
	template <bool posX, bool posY, bool posZ> bool IsOccludedOctant( const Ray& ray ) const;
	// BVH8 data
	CacheLine* bvh8Data = 0;		// Interleaved interior (256b) and leaf (192b) data.
	MBVH<8, Float, Index> bvh8;		// BVH8_CPU is created from BVH8 and uses its data.
	bool ownBVH8 = true;			// false when ConvertFrom receives an external bvh8.
	uint32_t allocatedBlocks = 0;	// node data and triangles are stored in 64-byte blocks.
	uint32_t usedBlocks = 0;		// the amount of data actually used.
};

// BLASInstance: A TLAS is built over BLAS instances, where a single BLAS can be
// used with multiple transforms, and multiple BLASses can be combined in a complex
// scene. The TLAS is built over the world-space AABBs of the BLAS root nodes.
template <typename Float, typename Index> class ALIGNED( 64 ) BLASInstance
{
public:
	using Vec3 = typename bvh_traits<Float>::vec3;
	using Mat4 = typename bvh_traits<Float>::mat4;
	BLASInstance() = default;
	BLASInstance( Index idx ) : blasIdx( idx ) {}
	Mat4 transform; // defaults to identity
	Mat4 invTransform; // defaults to identity
	Vec3 aabbMin = Vec3( bvh_far<Float> );
	Index blasIdx = 0;
	Vec3 aabbMax = Vec3( -bvh_far<Float> );
	uint32_t mask = (uint32_t)RAY_MASK_INTERSECT_ALL; // set to intersect with all rays by default
	void Update( BVHBase<Float, Index>* blas );
	void InvertTransform();
};

} // namespace impl

// Single precision, 32-bit indices: the default layouts.
using Intersection = impl::Intersection<float, uint32_t>;
using Ray = impl::Ray<float, uint32_t>;
using BVHBase = impl::BVHBase<float, uint32_t>;
using BVH = impl::BVH<float, uint32_t>;
using BVH_Verbose = impl::BVH_Verbose<float, uint32_t>;
using BVH_GPU = impl::BVH_GPU<float, uint32_t>;
#ifdef ENABLE_BVH_SOA
using BVH_SoA = impl::BVH_SoA<float, uint32_t>;
#endif
template <int M> using MBVH = impl::MBVH<M, float, uint32_t>;
using BVH4_GPU = impl::BVH4_GPU<float, uint32_t>;
using BVH4_CPU = impl::BVH4_CPU<float, uint32_t>;
using BVH8_CWBVH = impl::BVH8_CWBVH<float, uint32_t>;
using BVH8_CPU = impl::BVH8_CPU<float, uint32_t>;
using BVHTri4Leaf = impl::BVHTri4Leaf<float, uint32_t>;
using BLASInstance = impl::BLASInstance<float, uint32_t>;
#ifdef DOUBLE_PRECISION_SUPPORT
// Double precision, 64-bit indices.
using IntersectionEx = impl::Intersection<double, uint64_t>;
using RayEx = impl::Ray<double, uint64_t>;
using BVHBaseEx = impl::BVHBase<double, uint64_t>;
using BVH_Double = impl::BVH<double, uint64_t>;
using BLASInstanceEx = impl::BLASInstance<double, uint64_t>;
#endif

#ifdef ENABLE_VOXEL_SUPPORT

class VoxelSet : public BVHBase // just so it can be attached conveniently in a TLAS
{
public:
	struct DDAState
	{
		uint32_t X, Y, Z;		// 12 bytes
		float dummy1 = 0;		// 16 bytes
		bvhvec3 tmax;
		float dummy2 = 0;		// 32 bytes in total
	};
	VoxelSet();
	~VoxelSet();
	VoxelSet( const VoxelSet& ) = delete;		// owns raw allocations; copying would alias them
	VoxelSet& operator=( const VoxelSet& ) = delete;
	void Set( const uint32_t x, const uint32_t y, const uint32_t z, const uint32_t v );
	void UpdateTopGrid();
	bvhvec3 GetNormal( const Ray& ray ) const;
	int32_t Intersect( Ray& ray ) const;
	bool IsOccluded( const Ray& ray ) const;
	// A VoxelSet can only be a BLAS in a single precision TLAS.
	template <typename F, typename I> int32_t Intersect( impl::Ray<F, I>& ) const;
	template <typename F, typename I> bool IsOccluded( const impl::Ray<F, I>& ) const;
private:
	bool Setup3DDDA( const Ray& ray, const bvhvec3& Dsign, DDAState& state, const bvhint3& step, bvhvec3& tdelta, float& t ) const;
	// lowest level: 1 32-bit value per voxel
public:
	static constexpr int objectDim = 256; // 64, 128 or 256.
private:
	static constexpr int objectSize = objectDim * objectDim * objectDim;
	// grid level: collection of bricks
	static constexpr int brickDim = 8;
	static constexpr int brickSize = brickDim * brickDim * brickDim;
	static constexpr int gridDim = objectDim / brickDim;
	static constexpr int gridSize = gridDim * gridDim * gridDim;
	// topgrid level: 1 bit for each group of bricks
	static constexpr int groupDim = 4;
	static constexpr int groupSize = groupDim * groupDim * groupDim;
	static constexpr int topGridDim = gridDim / groupDim;
	static constexpr int topGridSize = topGridDim * topGridDim * topGridDim;
	static constexpr int topGridWords = (topGridSize + 31) / 32;	// topGrid is read/written as uint32_t
	static constexpr int topGridBytes = topGridWords * 4;
	static float NudgeScale( const bvhvec3& O )
	{
		// float resolution near the evaluated point scales with the larger of |O| and t;
		// for a ray that reaches the unit-sized object, |O| is a good proxy for both.
		static constexpr float voxelEps = 4.0f * objectDim * 1.1920929e-7f;
		const float m = tinybvh_max( fabsf( O.x ), tinybvh_max( fabsf( O.y ), fabsf( O.z ) ) );
		return voxelEps * tinybvh_max( 1.0f, m );
	}
	// masks
	static constexpr uint32_t topResMask = gridDim - groupDim;
	static constexpr uint32_t superMask = topResMask + topResMask * gridDim + topResMask * gridDim * gridDim;
	// non-static data
	uint32_t* grid = 0;
	uint32_t* brick = 0;
	uint32_t brickCount = (objectDim * objectDim) / 16; // will grow as needed; scales roughly quadratically with objectDim
	uint32_t freeBrickPtr = 1; // skip 1, as 0 denotes an empty brick in the topgrid.
	uint32_t* topGrid = 0;
};

#endif

} // namespace tinybvh

#endif // TINY_BVH_BASE_H_

// ============================================================================
//
//        I M P L E M E N T A T I O N
//
// ============================================================================

#ifdef TINYBVH_IMPLEMENTATION

#include <assert.h>			// for assert
#ifdef _MSC_VER
#include <intrin.h>			// for __lzcnt
#endif
#include <fstream>			// fstream
#include <algorithm>		// for std::swap
#if defined ENABLE_THREADED_BUILDS && !defined TINYBVH_NO_BUILTIN_POOL
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>
#endif

// Some constexpr stuff to produce nice-looking branches in
// *::Intersect with proper dead code elinimation.
#ifdef ENABLE_INDEXED_GEOMETRY
static constexpr bool indexedEnabled = true;
#else
static constexpr bool indexedEnabled = false;
#endif
#ifdef ENABLE_CUSTOM_GEOMETRY
static constexpr bool customEnabled = true;
#else
static constexpr bool customEnabled = false;
#endif

namespace tinybvh {

TINYBVH_FORCEINLINE uint32_t __bfind( uint32_t x ) // https://github.com/mackron/refcode/blob/master/lzcnt.c
{
#if defined _MSC_VER && !defined __clang__
	return 31 ^ __lzcnt( x );
#elif defined __EMSCRIPTEN__
	return 31 ^ __builtin_clz( x );
#elif defined __GNUC__ || defined __clang__
#if defined __APPLE__ || __has_builtin(__builtin_clz)
	return 31 ^ __builtin_clz( x );
#else
	uint32_t r;
	__asm__ __volatile__( "lzcnt{l %1, %0| %0, %1}" : "=r"(r) : "r"(x) : "cc" );
	return 31 ^ r;
#endif
#endif
}

// array element counting; https://stackoverflow.com/questions/12784136
#define BVH_NUM_ELEMS(a) (sizeof(a)/sizeof 0[a])

// random numbers
uint32_t tinybvh_rnduint( uint32_t& s ) { s ^= s << 13, s ^= s >> 17, s ^= s << 5; return s; }
float tinybvh_rndfloat( uint32_t& s ) { return tinybvh_rnduint( s ) * 2.3283064365387e-10f; }

// random unit vector
bvhvec3 tinybvh_rndvec3( uint32_t& s )
{
	bvhvec3 R;
loop: R = bvhvec3( tinybvh_rndfloat( s ) - 0.5f, tinybvh_rndfloat( s ) - 0.5f, tinybvh_rndfloat( s ) - 0.5f );
	if (tinybvh_dot( R, R ) > 0.25f) goto loop;
	return tinybvh_normalize( R );
}

static uint32_t __popc( uint32_t x )
{
#if defined _MSC_VER && !defined __clang__ && (defined _M_X64 || defined _M_IX86)
	return __popcnt( x );
#elif defined __GNUC__ || defined __clang__
	return __builtin_popcount( x );
#else
	x = x - ((x >> 1) & 0x55555555), x = (x & 0x33333333) + ((x >> 2) & 0x33333333);
	return (((x + (x >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24;
#endif
}

// code compaction: Moeller-Trumbore ray/tri test.
// Works for any precision: v0, e1 and e2 determine the vector type.
#define MOLLER_TRUMBORE_TEST( tmax, exit ) \
	const auto h = tinybvh_cross( ray.D, e2 );		\
	const auto a = tinybvh_dot( e1, h );			\
	if (a == 0) exit;								\
	const auto f = 1 / a;							\
	const auto s = ray.O - v0;						\
	const auto u = f * tinybvh_dot( s, h );			\
	const auto q = tinybvh_cross( s, e1 );			\
	const auto v = f * tinybvh_dot( ray.D, q );		\
	if (!(u >= 0 && v >= 0 && u + v <= 1)) exit;	\
	const auto t = f * tinybvh_dot( e2, q );		\
	if (!(t >= 0 && t <= tmax)) exit;

// code compaction: fetching triangle vertices, with or without indices.
#define GET_PRIM_INDICES_I0_I1_I2( bvh, idx ) if (indexedEnabled && bvh.vertIdx != 0) \
	i0 = bvh.vertIdx[idx * 3], i1 = bvh.vertIdx[idx * 3 + 1], i2 = bvh.vertIdx[idx * 3 + 2]; \
	else i0 = idx * 3, i1 = idx * 3 + 1, i2 = idx * 3 + 2;

// ray validation: throw an error if the input ray contains nans.
#define VALIDATE_RAY(r) { const auto test = (r).D.x + (r).D.y + (r).D.z + (r).hit.t + (r).O.x \
	+ (r).O.y + (r).O.z; BVH_FATAL_ERROR_IF( tinybvh_isnan( test ), "Input ray contains NaNs." ); }

#ifndef TINYBVH_USE_CUSTOM_VECTOR_TYPES

bvhmat4 operator*( const float s, const bvhmat4& a )
{
	bvhmat4 r;
	for (uint32_t i = 0; i < 16; i++) r[i] = a[i] * s;
	return r;
}
bvhmat4 operator*( const bvhmat4& a, const bvhmat4& b )
{
	bvhmat4 r;
	for (uint32_t i = 0; i < 16; i += 4) for (uint32_t j = 0; j < 4; ++j)
		r[i + j] = (a[i + 0] * b[j + 0]) + (a[i + 1] * b[j + 4]) +
		(a[i + 2] * b[j + 8]) + (a[i + 3] * b[j + 12]);
	return r;
}
bvhvec4::bvhvec4( const bvhvec3& a ) { x = a.x; y = a.y; z = a.z; w = 0; }
bvhvec4::bvhvec4( const bvhvec3& a, float b ) { x = a.x; y = a.y; z = a.z; w = b; }

#endif

bvhvec4slice::bvhvec4slice( const bvhvec4* data, uint32_t count, uint32_t stride ) :
	data{ reinterpret_cast<const int8_t*>(data) },
	count{ count }, stride{ stride } {
}

const bvhvec4& bvhvec4slice::operator[]( size_t i ) const
{
#ifdef PARANOID
	BVH_FATAL_ERROR_IF( i >= count, "bvhvec4slice::[..], Reading outside slice." );
#endif
	return *reinterpret_cast<const bvhvec4*>(data + stride * i);
}

#ifdef DOUBLE_PRECISION_SUPPORT

bvhdbl3slice::bvhdbl3slice( const bvhdbl3* data, uint64_t count, uint64_t stride ) :
	data{ reinterpret_cast<const int8_t*>(data) },
	count{ count }, stride{ stride } {
}

const bvhdbl3& bvhdbl3slice::operator[]( size_t i ) const
{
#ifdef PARANOID
	BVH_FATAL_ERROR_IF( i >= count, "bvhdbl3slice::[..], Reading outside slice." );
#endif
	return *reinterpret_cast<const bvhdbl3*>(data + stride * i);
}

#endif // DOUBLE_PRECISION_SUPPORT

namespace impl {

// BVHBase implementation
// ----------------------------------------------------------------------------

template <typename Float, typename Index> void* BVHBase<Float, Index>::AlignedAlloc( size_t size ) const
{
	return context.malloc ? context.malloc( size, context.userdata ) : nullptr;
}

template <typename Float, typename Index> void BVHBase<Float, Index>::AlignedFree( void* ptr ) const
{
	if (context.free && ptr)
		context.free( ptr, context.userdata );
}

template <typename Float, typename Index> void BVHBase<Float, Index>::CopyBasePropertiesFrom( const BVHBase& original )
{
	this->rebuildable = original.rebuildable;
	this->refittable = original.refittable;
	this->may_have_holes = original.may_have_holes;
	this->bvh_over_aabbs = original.bvh_over_aabbs;
	this->bvh_over_indices = original.bvh_over_indices;
	this->context = original.context;
	this->settings = original.settings;
	this->triCount = original.triCount;
	this->idxCount = original.idxCount;
	this->aabbMin = original.aabbMin, this->aabbMax = original.aabbMax;
	this->opmap = original.opmap, this->opmapN = original.opmapN;
}

// BVH implementation
// ----------------------------------------------------------------------------

template <typename Float, typename Index> BVH<Float, Index>::BVH( BVH&& other ) noexcept
{
	// shallow copy of parameters, options, and pointers
	*this = other;
	// mark 'other' as deleted to avoid double-free
	other.primIdx = 0;
	other.bvhNode = 0;
	other.fragment = 0;
}

template <typename Float, typename Index> BVH<Float, Index>::~BVH()
{
	AlignedFree( bvhNode );
	bvhNode = 0;
	AlignedFree( primIdx );
	primIdx = 0;
	AlignedFree( fragment );
	fragment = 0;
}

template <typename Float, typename Index> void BVH<Float, Index>::Save( const char* fileName )
{
	// saving is easy, it's the loadingn that will be complex.
	std::fstream s{ fileName, s.binary | s.out };
	const uint32_t header = this->CacheHeader();
	s.write( (char*)&header, sizeof( uint32_t ) );
	s.write( (char*)&triCount, sizeof( Index ) );
	s.write( (char*)this, sizeof( BVH ) );
	s.write( (char*)bvhNode, usedNodes * sizeof( BVHNode ) );
	s.write( (char*)primIdx, idxCount * sizeof( Index ) );
}

template <typename Float, typename Index> bool BVH<Float, Index>::Load( const char* f, const Vertex* v, const Index p ) { return Load( f, Slice( v, p * 3, sizeof( Vertex ) ) ); }
template <typename Float, typename Index> bool BVH<Float, Index>::Load( const char* f, const Vertex* v, const uint32_t* i, const Index p ) { return Load( f, Slice( v, p * 3, sizeof( Vertex ) ), i, p ); }
template <typename Float, typename Index> bool BVH<Float, Index>::Load( const char* fileName, const Slice& vertices, const uint32_t* indices, const Index primCount )
{
	// open file and check contents
	std::fstream s{ fileName, s.binary | s.in };
	if (!s) return false;
	BVHContext ctxBackup = context;
	uint32_t* opmapBackup = opmap, opmapNBackup = opmapN;
	bool expectIndexed = (indices != nullptr);
	uint32_t header;
	Index fileTriCount;
	s.read( (char*)&header, sizeof( uint32_t ) );
	if (header != this->CacheHeader()) return false;
	s.read( (char*)&fileTriCount, sizeof( Index ) );
	if (expectIndexed && fileTriCount != primCount) return false;
	if (!expectIndexed && fileTriCount != vertices.count / 3) return false;
	AlignedFree( bvhNode );
	AlignedFree( primIdx );
	AlignedFree( fragment );
	s.read( (char*)this, sizeof( BVH ) );
	const bool fileIsIndexed = (vertIdx != nullptr);
	const bool fileIsTLAS = (blasList != nullptr || instList != nullptr);
	bvhNode = 0, primIdx = 0, fragment = 0, rrsHits = 0;
	verts = Slice( nullptr, 0, 0 ), vertIdx = 0;
	instList = 0, blasList = 0, blasCount = 0;
	customIntersect = 0, customIsOccluded = 0, customUserdata = 0;
#ifdef ENABLE_THREADED_BUILDS
	atomicNewNodePtr = 0, atomicNextFrag = 0;
#endif
	leafNodes = 0, scratchPad = 0, flag = 0, tmpBnds = 0, SARs = 0;
	for (int32_t a = 0; a < 3; a++) sortedIdx[a] = 0, sortedBnds[a] = 0;
	context = ctxBackup; // can't load context; function pointers will differ.
	opmap = opmapBackup, opmapN = opmapNBackup; // opacity maps are caller-owned; keep what was set.
	if (expectIndexed != fileIsIndexed || fileIsTLAS)
	{
		// not the geometry we expected, or a TLAS, which we can't load/save;
		// leave *this as a valid but empty BVH rather than a half-loaded one.
		allocatedNodes = usedNodes = triCount = idxCount = 0;
		return false;
	}
	bvhNode = (BVHNode*)AlignedAlloc( allocatedNodes * sizeof( BVHNode ) );
	primIdx = (Index*)AlignedAlloc( idxCount * sizeof( Index ) );
	fragment = 0; // no need for this in a BVH that can't be rebuilt.
	s.read( (char*)bvhNode, usedNodes * sizeof( BVHNode ) );
	s.read( (char*)primIdx, idxCount * sizeof( Index ) );
	verts = vertices; // we can't load vertices since the BVH doesn't own this data.
	vertIdx = (uint32_t*)indices;
	// all ok.
	return true;
}

// BVH builder for triangle geometry.
// This code uses no SIMD instructions. Faster code, using SSE/AVX, is available for x64 CPUs.
template <typename Float, typename Index> void BVH<Float, Index>::Build( const Slice& v ) { Build( v, 0, 0 ); }
template <typename Float, typename Index> void BVH<Float, Index>::Build( const Vertex* v, const uint32_t* i, const Index p ) { Build( Slice( v, p * 3, sizeof( Vertex ) ), i, p ); }
template <typename Float, typename Index> void BVH<Float, Index>::Build( const Vertex* v, const Index p ) { Build( Slice( v, p * 3, sizeof( Vertex ) ) ); }
template <typename Float, typename Index> void BVH<Float, Index>::Build( const Slice& vertices, const uint32_t* indices, Index prims )
{
#ifdef SLICEDUMP
	// this code dumps the passed geometry data to a file - for debugging only.
	std::fstream df{ "dump.bin", df.binary | df.out };
	uint32_t vcount = (uint32_t)vertices.count, indexed = indices == 0 ? 0 : 1, stride = (uint32_t)vertices.stride;
	uint32_t pcount = (uint32_t)(indices ? prims : (vertices.count / 3));
	df.write( (char*)&pcount, 4 );
	df.write( (char*)&vcount, 4 );
	df.write( (char*)&stride, 4 );
	df.write( (char*)&indexed, sizeof( uint32_t ) );
	df.write( (char*)vertices.data, vertices.stride * vertices.count );
	if (indexed) df.write( (char*)indices, prims * 3 * 4 );
#endif
	if (settings.useSpatialSplits) // SBVH requested
	{
		PrepareHQBuild( vertices, indices, prims );
		BuildHQ();
	}
	else if (settings.useFullSweep) // Full-sweep requested
	{
		PrepareBuild( vertices, indices, prims );
		BuildFullSweep();
	}
	else if (BVHSIMDBuilders<Float, Index>::avx && settings.useSIMDifavailable) // No preference: use fast AVX builder
	{
		BuildAVX( vertices, indices, prims );
	}
	else
	{
		PrepareBuild( vertices, indices, prims ); // No preference, no AVX: use reference builder.
		Build();
	}
	if (settings.postOptimize) Optimize( settings.optimizeIterations );
}

template <typename Float, typename Index> void BVH<Float, Index>::BuildAABB( const Vertex* aabbs, const Index aabbCount )
{
	// BVH builder for a list of AABBs.
	BVH_FATAL_ERROR_IF( aabbCount == 0, "BVH::BuildAABB( .. ), aabbCount == 0." );
	triCount = idxCount = aabbCount;
	const Index spaceNeeded = aabbCount * 2; // upper limit
	if (allocatedNodes < spaceNeeded)
	{
		AlignedFree( bvhNode );
		AlignedFree( primIdx );
		AlignedFree( fragment );
		bvhNode = (BVHNode*)AlignedAlloc( spaceNeeded * sizeof( BVHNode ) );
		allocatedNodes = spaceNeeded;
		memset( &bvhNode[1], 0, sizeof( BVHNode ) ); // node 1 remains unused, for cache line alignment.
		primIdx = (Index*)AlignedAlloc( aabbCount * sizeof( Index ) );
		fragment = (Fragment*)AlignedAlloc( aabbCount * sizeof( Fragment ) );
	}
	// copy relevant data to the fragment array over which the BVH will be built.
	BVHNode& root = bvhNode[0];
	root.leftFirst = 0, root.triCount = aabbCount, root.aabbMin = Vec3( bvh_far<Float> ), root.aabbMax = Vec3( -bvh_far<Float> );
	for (Index i = 0; i < aabbCount; i++)
	{
		fragment[i].bmin = Vec3( aabbs[i * 2] ), fragment[i].bmax = Vec3( aabbs[i * 2 + 1] );
		fragment[i].primIdx = i, fragment[i].clipped = 0, primIdx[i] = i;
		root.aabbMin = tinybvh_min( root.aabbMin, fragment[i].bmin );
		root.aabbMax = tinybvh_max( root.aabbMax, fragment[i].bmax );
	}
	// start build
	newNodePtr = 2;
	Build();
}

template <typename Float, typename Index> void BVH<Float, Index>::Build( void (*customGetAABB)(const Index, Vec3&, Vec3&, void*), const Index primCount )
{
	// BVH builder for custom geometry; AABBs are obtained via a function pointer in context.
	BVH_FATAL_ERROR_IF( primCount == 0, "BVH::Build( void (*customGetAABB)( .. ), instCount ), instCount == 0." );
	triCount = idxCount = primCount;
	const Index spaceNeeded = primCount * 2; // upper limit
	if (allocatedNodes < spaceNeeded)
	{
		AlignedFree( bvhNode );
		AlignedFree( primIdx );
		AlignedFree( fragment );
		bvhNode = (BVHNode*)AlignedAlloc( spaceNeeded * sizeof( BVHNode ) );
		allocatedNodes = spaceNeeded;
		memset( &bvhNode[1], 0, sizeof( BVHNode ) ); // node 1 remains unused, for cache line alignment.
		primIdx = (Index*)AlignedAlloc( primCount * sizeof( Index ) );
		fragment = (Fragment*)AlignedAlloc( primCount * sizeof( Fragment ) );
	}
	// copy relevant data to the fragment array over which the BVH will be built.
	BVHNode& root = bvhNode[0];
	root.leftFirst = 0, root.triCount = primCount, root.aabbMin = Vec3( bvh_far<Float> ), root.aabbMax = Vec3( -bvh_far<Float> );
	for (Index i = 0; i < primCount; i++)
	{
		customGetAABB( i, fragment[i].bmin, fragment[i].bmax, customUserdata );
		fragment[i].primIdx = i, fragment[i].clipped = 0, primIdx[i] = i;
		root.aabbMin = tinybvh_min( root.aabbMin, fragment[i].bmin );
		root.aabbMax = tinybvh_max( root.aabbMax, fragment[i].bmax );
	}
	// start build
	newNodePtr = 2;
	Build();
}

template <typename Float, typename Index> void BVH<Float, Index>::Build( BLASInstance* instances, const Index instCount, BVHBase<Float, Index>** blasses, const Index bCount )
{
	// TLAS builder. Build a BVH over a list of BLAS instances.
	BVH_FATAL_ERROR_IF( instCount == 0, "BVH::Build( BLASInstance*, instCount ), instCount == 0." );
	triCount = idxCount = instCount;
	const Index spaceNeeded = instCount * 2; // upper limit
	if (allocatedNodes < spaceNeeded)
	{
		AlignedFree( bvhNode );
		AlignedFree( primIdx );
		AlignedFree( fragment );
		bvhNode = (BVHNode*)AlignedAlloc( spaceNeeded * sizeof( BVHNode ) );
		allocatedNodes = spaceNeeded;
		memset( &bvhNode[1], 0, sizeof( BVHNode ) ); // node 1 remains unused, for cache line alignment.
		primIdx = (Index*)AlignedAlloc( instCount * sizeof( Index ) );
		fragment = (Fragment*)AlignedAlloc( instCount * sizeof( Fragment ) );
	}
	instList = instances, blasList = blasses, blasCount = bCount;
	// copy relevant data to the fragment array over which the BVH will be built.
	BVHNode& root = bvhNode[0];
	root.leftFirst = 0, root.triCount = instCount, root.aabbMin = Vec3( bvh_far<Float> ), root.aabbMax = Vec3( -bvh_far<Float> );
	for (Index i = 0; i < instCount; i++)
	{
		if (blasList) // if a null pointer is passed, we'll assume the BLASInstances have been updated elsewhere.
			instList[i].Update( (BVH*)blasList[instList[i].blasIdx] );
		fragment[i].bmin = instList[i].aabbMin, fragment[i].primIdx = i;
		fragment[i].bmax = instList[i].aabbMax, fragment[i].clipped = 0;
		root.aabbMin = tinybvh_min( root.aabbMin, instList[i].aabbMin );
		root.aabbMax = tinybvh_max( root.aabbMax, instList[i].aabbMax ), primIdx[i] = i;
	}
	// start build
	newNodePtr = 2;
	Build(); // or BuildAVX, for large TLAS.
}

// TLAS over BLASses of the same layout. BVH derives from BVHBase without offset, which makes the cast of the array safe.
template <typename Float, typename Index> void BVH<Float, Index>::Build( BLASInstance* instances, const Index instCount, BVH** blasses, const Index bCount )
{
	Build( instances, instCount, (BVHBase<Float, Index>**)blasses, bCount );
}

template <typename Float, typename Index> void BVH<Float, Index>::BuildQuick( const Vertex* v, const Index p ) { BuildQuick( Slice( v, p * 3, sizeof( Vertex ) ) ); }
template <typename Float, typename Index> void BVH<Float, Index>::BuildQuick( const Slice& vertices )
{
	// Basic single-function BVH builder, using mid-point splits.
	BVH_FATAL_ERROR_IF( vertices.count < 3, "BVH::BuildQuick( .. ), primCount == 0." );
	// allocate on first build
	const Index primCount = vertices.count / 3;
	const Index spaceNeeded = primCount * 2; // upper limit
	// a rebuild is unsafe once the tree has been converted, whether or not we reallocate.
	BVH_FATAL_ERROR_IF( allocatedNodes > 0 && !rebuildable, "BVH::BuildQuick( .. ), bvh not rebuildable." );
	// (re)allocate if the node pool is too small, or if the scratch buffers are absent.
	if (allocatedNodes < spaceNeeded || primIdx == 0 || fragment == 0)
	{
		AlignedFree( bvhNode );
		AlignedFree( primIdx );
		AlignedFree( fragment );
		bvhNode = (BVHNode*)AlignedAlloc( spaceNeeded * sizeof( BVHNode ) );
		allocatedNodes = spaceNeeded;
		memset( &bvhNode[1], 0, sizeof( BVHNode ) ); // node 1 remains unused, for cache line alignment.
		primIdx = (Index*)AlignedAlloc( primCount * sizeof( Index ) );
		fragment = (Fragment*)AlignedAlloc( primCount * sizeof( Fragment ) );
	}
	verts = vertices; // note: we're not copying this data; don't delete.
	vertIdx = 0, bvh_over_indices = false, bvh_over_aabbs = false;
	instList = 0, blasList = 0, blasCount = 0;
	idxCount = triCount = primCount, newNodePtr = 2;
	// assign all triangles to the root node
	BVHNode& root = bvhNode[0];
	root.leftFirst = 0, root.triCount = triCount, root.aabbMin = Vec3( bvh_far<Float> ), root.aabbMax = Vec3( -bvh_far<Float> );
	// initialize fragments and initialize root node bounds
	for (Index i = 0; i < triCount; i++)
		fragment[i].bmin = tinybvh_min( tinybvh_min( verts[i * 3], verts[i * 3 + 1] ), verts[i * 3 + 2] ),
		fragment[i].bmax = tinybvh_max( tinybvh_max( verts[i * 3], verts[i * 3 + 1] ), verts[i * 3 + 2] ),
		fragment[i].primIdx = i, fragment[i].clipped = 0,
		root.aabbMin = tinybvh_min( root.aabbMin, fragment[i].bmin ),
		root.aabbMax = tinybvh_max( root.aabbMax, fragment[i].bmax ), primIdx[i] = i;
	// subdivide recursively
	Index task[512], taskCount = 0, nodeIdx = 0;
	while (1)
	{
		while (1)
		{
			BVHNode& node = bvhNode[nodeIdx];
			// in-place partition against midpoint on longest axis
			Index j = node.leftFirst + node.triCount, src = node.leftFirst;
			uint32_t axis = 0;
			Vec3 extent = node.aabbMax - node.aabbMin;
			if (extent.y > extent.x && extent.y > extent.z) axis = 1;
			if (extent.z > extent.x && extent.z > extent.y) axis = 2;
			Float splitPos = node.aabbMin[axis] + extent[axis] * 0.5f, centroid;
			Vec3 lbmin( bvh_far<Float> ), lbmax( -bvh_far<Float> ), rbmin( bvh_far<Float> ), rbmax( -bvh_far<Float> ), fmin, fmax;
			for (Index fi, i = 0; i < node.triCount; i++)
			{
				fi = primIdx[src], fmin = fragment[fi].bmin, fmax = fragment[fi].bmax;
				centroid = (fmin[axis] + fmax[axis]) * 0.5f;
				if (centroid < splitPos)
				{
					lbmin = tinybvh_min( lbmin, fmin ), lbmax = tinybvh_max( lbmax, fmax ), src++;
					continue;
				}
				rbmin = tinybvh_min( rbmin, fmin ), rbmax = tinybvh_max( rbmax, fmax );
				tinybvh_swap( primIdx[src], primIdx[--j] );
			}
			// create child nodes
			const Index leftCount = src - node.leftFirst, rightCount = node.triCount - leftCount;
			if (leftCount == 0 || rightCount == 0 || taskCount == BVH_NUM_ELEMS( task )) break; // split did not work out.
			const Index lci = newNodePtr++, rci = newNodePtr++;
			bvhNode[lci].aabbMin = lbmin, bvhNode[lci].aabbMax = lbmax;
			bvhNode[lci].leftFirst = node.leftFirst, bvhNode[lci].triCount = leftCount;
			bvhNode[rci].aabbMin = rbmin, bvhNode[rci].aabbMax = rbmax;
			bvhNode[rci].leftFirst = j, bvhNode[rci].triCount = rightCount;
			node.leftFirst = lci, node.triCount = 0;
			task[taskCount++] = rci, nodeIdx = lci;
		}
		// fetch subdivision task from stack
		if (taskCount == 0) break; else nodeIdx = task[--taskCount];
	}
	// all done.
	aabbMin = bvhNode[0].aabbMin, aabbMax = bvhNode[0].aabbMax, usedNodes = newNodePtr;
	refittable = true; // not using spatial splits: can refit this BVH
	may_have_holes = false; // the reference builder produces a continuous list of nodes
}

template <typename Float, typename Index> void BVH<Float, Index>::PrepareBuild( const Slice& vertices, const uint32_t* indices, const Index prims )
{
	// Allocate memory and prepare a list of fragments to build a BVH over.
	const Index primCount = prims > 0 ? prims : vertices.count / 3;
	const Index splitBudget = settings.usePresplitting ? ((int)(primCount * settings.presplitFactor)) : 0;
	const Index spaceNeeded = (primCount + splitBudget) * 2; // upper limit
	// allocate memory on first build
	if (allocatedNodes < spaceNeeded)
	{
		AlignedFree( bvhNode );
		AlignedFree( primIdx );
		AlignedFree( fragment );
		bvhNode = (BVHNode*)AlignedAlloc( spaceNeeded * sizeof( BVHNode ) );
		allocatedNodes = spaceNeeded;
		memset( &bvhNode[1], 0, sizeof( BVHNode ) );	// node 1 remains unused, for cache line alignment.
		primIdx = (Index*)AlignedAlloc( (primCount + splitBudget) * sizeof( Index ) );
		if (vertices) fragment = (Fragment*)AlignedAlloc( (primCount + splitBudget) * sizeof( Fragment ) );
		else BVH_FATAL_ERROR_IF( fragment == 0, "BVH::PrepareBuild( 0, .. ), not called from ::Build( aabb )." );
	}
	else BVH_FATAL_ERROR_IF( !rebuildable, "BVH::PrepareBuild( .. ), bvh not rebuildable." );
	// set verts, vertIdx
	triCount = primCount, verts = vertices, vertIdx = (uint32_t*)indices;
	// prepare root node
	BVHNode& root = bvhNode[0];
	root.aabbMin = Vec3( bvh_far<Float> ), root.aabbMax = Vec3( -bvh_far<Float> );
	// prepare fragments
	BVH_FATAL_ERROR_IF( vertices.count == 0, "BVH::PrepareBuild( .. ), empty vertex slice." );
	if (!indices)
	{
		BVH_FATAL_ERROR_IF( prims != 0, "BVH::PrepareBuild( .. ), indices == 0." );
		// building a BVH over triangles specified as three 16-byte vertices each.
		for (Index i = 0; i < primCount; i++)
		{
			const Vertex v0 = verts[i * 3], v1 = verts[i * 3 + 1], v2 = verts[i * 3 + 2];
			const Vertex fmin = tinybvh_min( v0, tinybvh_min( v1, v2 ) );
			const Vertex fmax = tinybvh_max( v0, tinybvh_max( v1, v2 ) );
			fragment[i].bmin = fmin, fragment[i].bmax = fmax, fragment[i].primIdx = i, fragment[i].clipped = 0;
			root.aabbMin = tinybvh_min( root.aabbMin, fragment[i].bmin );
			root.aabbMax = tinybvh_max( root.aabbMax, fragment[i].bmax ), primIdx[i] = i;
		}
	}
	else
	{
		BVH_FATAL_ERROR_IF( prims == 0, "BVH::PrepareBuild( .. ), prims == 0." );
		// building a BVH over triangles consisting of vertices indexed by 'indices'.
		for (Index i = 0; i < primCount; i++)
		{
			const uint32_t i0 = indices[i * 3], i1 = indices[i * 3 + 1], i2 = indices[i * 3 + 2];
			const Vertex v0 = verts[i0], v1 = verts[i1], v2 = verts[i2];
			const Vertex fmin = tinybvh_min( v0, tinybvh_min( v1, v2 ) );
			const Vertex fmax = tinybvh_max( v0, tinybvh_max( v1, v2 ) );
			fragment[i].bmin = fmin, fragment[i].bmax = fmax, fragment[i].primIdx = i, fragment[i].clipped = 0;
			root.aabbMin = tinybvh_min( root.aabbMin, fragment[i].bmin );
			root.aabbMax = tinybvh_max( root.aabbMax, fragment[i].bmax ), primIdx[i] = i;
		}
	}
	// presplitting
	Index fragCount = settings.usePresplitting ? Presplit() : primCount;
	// finalize root node
	root.leftFirst = 0, root.triCount = idxCount = triCount = fragCount;
	// reset node pool
	newNodePtr = 2;
	bvh_over_indices = indices != nullptr;
	// all set; actual build happens in BVH::Build.
}

// Helper function to build a subtree via the thread pool
template <typename Float, typename Index> struct BVHBuildSubtreeArgs { BVH<Float, Index>* bvh; Index node; uint32_t depth; };
template <typename Float, typename Index> static void BVHBuildSubtree( void* payload )
{
	BVHBuildSubtreeArgs<Float, Index>* a = (BVHBuildSubtreeArgs<Float, Index>*)payload;
	a->bvh->Build( a->node, a->depth );
}
template <typename Float, typename Index> void BVH<Float, Index>::Build( Index nodeIdx, uint32_t depth )
{
	// Reference builder: Binned, threaded SAH BVH builder. Not using SIMD.
	if (depth == 0)
	{
		threadedBuild = false;
	#ifdef ENABLE_THREADED_BUILDS
		// build in parallel when given a sufficiently large input
		if (triCount >= MT_BUILD_THRESHOLD && context.spawn && context.barrier)
			threadedBuild = true, atomicNewNodePtr = this->template ContextNew<std::atomic<Index>>( newNodePtr );
	#endif
	}
	// subdivide root node recursively
	Index task[512], taskCount = 0;
	BVHNode& root = bvhNode[0];
	Vec3 minDim = (root.aabbMax - root.aabbMin) * 1e-20f;
	Vec3 bestLMin( 0 ), bestLMax( 0 ), bestRMin( 0 ), bestRMax( 0 );
	while (1)
	{
		while (1)
		{
			BVHNode& node = bvhNode[nodeIdx];
			const Float SA = node.SurfaceArea();
			if (SA == 0) break; // can't split an infinitely small node.
			// find optimal object split
			Vec3 binMin[3][BVHBINS], binMax[3][BVHBINS];
			for (uint32_t a = 0; a < 3; a++) for (uint32_t i = 0; i < BVHBINS; i++)
				binMin[a][i] = Vec3( bvh_far<Float> ), binMax[a][i] = Vec3( -bvh_far<Float> );
			Index count[3][BVHBINS] = {};
			const Vec3 extent = node.aabbMax - node.aabbMin;
			const Vec3 nmin3 = node.aabbMin, rpd3 = Vec3(
				extent.x > minDim.x ? (BVHBINS / extent.x) : 0,
				extent.y > minDim.y ? (BVHBINS / extent.y) : 0,
				extent.z > minDim.z ? (BVHBINS / extent.z) : 0
			);
			const Index* nodeFragIdx = primIdx + node.leftFirst;
			for (Index i = 0; i < node.triCount; i++) // process all tris for x,y and z at once
			{
				const Fragment& frag = fragment[nodeFragIdx[i]];
				const Vec3 fbmin = frag.bmin, fbmax = frag.bmax;
				const Vec3 fbi = ((fbmin + fbmax) * 0.5f - nmin3) * rpd3;
			bvhint3 bi( (int32_t)fbi.x, (int32_t)fbi.y, (int32_t)fbi.z );
				bi.x = tinybvh_clamp( bi.x, 0, BVHBINS - 1 );
				bi.y = tinybvh_clamp( bi.y, 0, BVHBINS - 1 );
				bi.z = tinybvh_clamp( bi.z, 0, BVHBINS - 1 );
				binMin[0][bi.x] = tinybvh_min( binMin[0][bi.x], fbmin );
				binMax[0][bi.x] = tinybvh_max( binMax[0][bi.x], fbmax ), count[0][bi.x]++;
				binMin[1][bi.y] = tinybvh_min( binMin[1][bi.y], fbmin );
				binMax[1][bi.y] = tinybvh_max( binMax[1][bi.y], fbmax ), count[1][bi.y]++;
				binMin[2][bi.z] = tinybvh_min( binMin[2][bi.z], fbmin );
				binMax[2][bi.z] = tinybvh_max( binMax[2][bi.z], fbmax ), count[2][bi.z]++;
			}
			// calculate per-split totals
			Float splitCost = bvh_far<Float>;
			uint32_t bestAxis = 0, bestPos = 0;
			for (int32_t a = 0; a < 3; a++) if (extent[a] > minDim[a])
			{
				Vec3 lBMin[BVHBINS - 1], rBMin[BVHBINS - 1], l1( bvh_far<Float> ), l2( -bvh_far<Float> );
				Vec3 lBMax[BVHBINS - 1], rBMax[BVHBINS - 1], r1( bvh_far<Float> ), r2( -bvh_far<Float> );
				Float ANL[BVHBINS - 1], ANR[BVHBINS - 1];
				for (Index lN = 0, rN = 0, i = 0; i < BVHBINS - 1; i++)
				{
					lBMin[i] = l1 = tinybvh_min( l1, binMin[a][i] );
					rBMin[BVHBINS - 2 - i] = r1 = tinybvh_min( r1, binMin[a][BVHBINS - 1 - i] );
					lBMax[i] = l2 = tinybvh_max( l2, binMax[a][i] );
					rBMax[BVHBINS - 2 - i] = r2 = tinybvh_max( r2, binMax[a][BVHBINS - 1 - i] );
					lN += count[a][i], rN += count[a][BVHBINS - 1 - i];
					ANL[i] = lN == 0 ? bvh_far<Float> : (tinybvh_halfarea( l2 - l1 ) * (Float)lN);
					ANR[BVHBINS - 2 - i] = rN == 0 ? bvh_far<Float> : (tinybvh_halfarea( r2 - r1 ) * (Float)rN);
				}
				// evaluate bin totals to find best position for object split
				for (uint32_t i = 0; i < BVHBINS - 1; i++)
				{
					const Float C = ANL[i] + ANR[i];
					if (C < splitCost)
					{
						splitCost = C, bestAxis = a, bestPos = i;
						bestLMin = lBMin[i], bestRMin = rBMin[i], bestLMax = lBMax[i], bestRMax = rBMax[i];
					}
				}
			}
			splitCost = c_trav + c_int * splitCost / SA;
			const Float noSplitCost = (Float)node.triCount * c_int;
			if (splitCost >= noSplitCost)
			{
			#ifdef _DEBUG
				if (node.triCount > 512) fprintf( stderr, "failed to split large node (%llu tris).\n", (unsigned long long)node.triCount );
			#endif
				break; // not splitting is better.
			}
			// in-place partition
			Index j = node.leftFirst + node.triCount, src = node.leftFirst;
			const Float rpd = rpd3[bestAxis], nmin = nmin3[bestAxis];
			for (Index i = 0; i < node.triCount; i++)
			{
				const Index fi = primIdx[src];
				int32_t bi = (uint32_t)(((fragment[fi].bmin[bestAxis] + fragment[fi].bmax[bestAxis]) * 0.5f - nmin) * rpd);
				bi = tinybvh_clamp( bi, 0, BVHBINS - 1 );
				if ((uint32_t)bi <= bestPos) src++; else tinybvh_swap( primIdx[src], primIdx[--j] );
			}
			// create child nodes
			Index leftCount = src - node.leftFirst, rightCount = node.triCount - leftCount;
			if (leftCount == 0 || rightCount == 0 || taskCount == BVH_NUM_ELEMS( task )) break; // should not happen.
			Index n;
		#ifdef ENABLE_THREADED_BUILDS
			if (threadedBuild) n = atomicNewNodePtr->fetch_add( 2 ); else n = newNodePtr, newNodePtr += 2;
		#else
			n = newNodePtr, newNodePtr += 2;
		#endif
			bvhNode[n].aabbMin = bestLMin, bvhNode[n].aabbMax = bestLMax;
			bvhNode[n].leftFirst = node.leftFirst, bvhNode[n].triCount = leftCount;
			bvhNode[n + 1].aabbMin = bestRMin, bvhNode[n + 1].aabbMax = bestRMax;
			bvhNode[n + 1].leftFirst = j, bvhNode[n + 1].triCount = rightCount;
			node.leftFirst = n, node.triCount = 0;
		#ifdef ENABLE_THREADED_BUILDS
			// only hand work to the pool if the subtrees are large enough to pay for the task overhead.
			if (threadedBuild && depth < MT_SPAWN_DEPTH &&
				tinybvh_max( leftCount, rightCount ) > MT_SPAWN_MIN_PRIMS)
			{
				// spawn the larger subtree, continue with the small one; root barrier joins.
				BVHBuildSubtreeArgs<Float, Index> a = { this, leftCount > rightCount ? (Index)n : (Index)n + 1, depth + 1 };
				tinybvh_spawn( context, &BVHBuildSubtree<Float, Index>, &a, sizeof( a ) );
				nodeIdx = leftCount > rightCount ? ((Index)n + 1) : (Index)n;
				continue;
			}
		#endif
			task[taskCount++] = n + 1, nodeIdx = n;
		}
		// fetch subdivision task from stack
		if (taskCount == 0) break; else nodeIdx = task[--taskCount];
	}
	// all done.
	if (depth == 0)
	{
	#ifdef ENABLE_THREADED_BUILDS
		if (threadedBuild)
		{
			tinybvh_barrier( context );
			newNodePtr = atomicNewNodePtr->load();
			ContextDelete( atomicNewNodePtr );
		}
	#endif
		usedNodes = newNodePtr;
		aabbMin = bvhNode[0].aabbMin, aabbMax = bvhNode[0].aabbMax;
		refittable = settings.usePresplitting ? false : true; // only if not using spatial splits
		may_have_holes = false; // the reference builder produces a continuous list of nodes
		bvh_over_aabbs = (verts == 0); // bvh over aabbs is suitable as TLAS
		if (settings.usePresplitting)
		{
			// finalize indices in index array
			for (Index i = 0; i < triCount; i++) primIdx[i] = fragment[primIdx[i]].primIdx;
			if (settings.presplitPostPass) PresplitPostPass();
		}
	}
}

// radix sort
static TINYBVH_FORCEINLINE uint32_t FloatToKey( const float value )
{
	uint32_t f;
	memcpy( &f, &value, sizeof( f ) );
	uint32_t mask = (uint32_t)((int)f >> 31 | (1 << 31));
	return f ^ mask;
}
template <typename Index> static void RadixSort( Index* input, Index* output, const uint32_t* keys, int len )
{
	// http://stereopsis.com/radix.html - Beats std::sort unless for small inputs (say len <= ~64)
	const int binSize = 1 << 11, mask = binSize - 1;
	int prefixSum[binSize * 3] = { 0 };
	for (int i = 0; i < len; i++) // compute histogram for all passes
	{
		const uint32_t key = keys[input[i]];
		prefixSum[key & mask]++, prefixSum[((key >> 11) & mask) + binSize]++;
		prefixSum[((key >> 22) & mask) + 2 * binSize]++;
	}
	// compute prefix sum for all passes
	int sum0 = 0, sum1 = 0, sum2 = 0;
	for (int i = 0; i < binSize; i++)
	{
		const int temp0 = prefixSum[i], temp1 = prefixSum[i + binSize], temp2 = prefixSum[i + 2 * binSize];
		prefixSum[i] = sum0, sum0 += temp0, prefixSum[i + binSize] = sum1, sum1 += temp1;
		prefixSum[i + 2 * binSize] = sum2, sum2 += temp2;
	}
	for (int i = 0; i < 3; i++) // sort from LSB to MSB in radix-sized steps
	{
		for (int j = 0; j < len; j++)
		{
			const Index element = input[j];
			const uint32_t key = keys[element];
			output[prefixSum[((key >> (i * 11)) & mask) + i * binSize]++] = element;
		}
		tinybvh_swap( input, output );
	}
}

// static helper function to build a subtree via the thread pool
template <typename Float, typename Index> static void BVHBuildFullSweepSubtree( void* payload )
{
	BVHBuildSubtreeArgs<Float, Index>* a = (BVHBuildSubtreeArgs<Float, Index>*)payload;
	a->bvh->BuildFullSweep( a->node, a->depth );
}
#ifdef ENABLE_THREADED_BUILDS
// sort one axis' fragment keys; scheduled via the parallel_for hook.
template <typename Index> struct BVHRadixSortArgs { Index* input[3]; Index* output[3]; uint32_t* keys[3]; int len; };
template <typename Index> static void BVHRadixSortAxis( uint32_t a, void* payload )
{
	BVHRadixSortArgs<Index>* args = (BVHRadixSortArgs<Index>*)payload;
	RadixSort( args->input[a], args->output[a], args->keys[a], args->len );
}
#endif
// Gather the fragment bounds for one axis into sorted order. Scheduled via parallel_for.
template <typename Float, typename Index> void BVH<Float, Index>::SweepGatherTask( uint32_t axis, void* payload ) { ((BVH*)payload)->GatherSweepBounds( axis ); }
template <typename Float, typename Index> void BVH<Float, Index>::GatherSweepBounds( uint32_t axis )
{
	const Index* idx = sortedIdx[axis];
	SweepBounds* bnd = sortedBnds[axis];
	for (Index i = 0; i < triCount; i++)
	{
		const Fragment& f = fragment[idx[i]];
		bnd[i].bmin = f.bmin, bnd[i].bmax = f.bmax;
	}
}
template <typename Float, typename Index> void BVH<Float, Index>::BuildFullSweep( Index nodeIdx, uint32_t depth )
{
	// Full-sweep SAH builder. Instead of using binning, this builder evaluates all possible split
	// plane candidates for each axis. Works well with triangle presplitting.
	if (depth == 0)
	{
		if (triCount < 2)
		{
			// Trivial input: the root node is a leaf. Handled here separately.
			BVHNode& root = bvhNode[0];
			root.aabbMin = Vec3( bvh_far<Float> ), root.aabbMax = Vec3( -bvh_far<Float> );
			for (Index i = 0; i < triCount; i++)
				root.aabbMin = tinybvh_min( root.aabbMin, fragment[primIdx[i]].bmin ),
				root.aabbMax = tinybvh_max( root.aabbMax, fragment[primIdx[i]].bmax );
			root.leftFirst = 0, root.triCount = triCount;
			aabbMin = root.aabbMin, aabbMax = root.aabbMax, usedNodes = newNodePtr;
			refittable = true, may_have_holes = false, bvh_over_aabbs = (verts == 0);
			if (settings.usePresplitting)
			{
				for (Index i = 0; i < triCount; i++) primIdx[i] = fragment[primIdx[i]].primIdx;
				if (settings.presplitPostPass) PresplitPostPass();
			}
			return;
		}
		// prepare threading
		threadedBuild = false;
	#ifdef ENABLE_THREADED_BUILDS
		// build in parallel when given a sufficiently large input
		if (triCount >= MT_BUILD_THRESHOLD && context.spawn && context.barrier)
			threadedBuild = true, atomicNewNodePtr = this->template ContextNew<std::atomic<Index>>( newNodePtr );
	#endif
		// create 32-bit integer sorting keys from fragment centroids
		uint32_t* sortKey[3];
		for (int a = 0; a < 3; a++) sortKey[a] = (uint32_t*)(bvhNode + 2) + a * triCount;
		for (Index i = 0; i < triCount; i++)
			for (int a = 0; a < 3; a++) sortKey[a][i] = FloatToKey( fragment[i].bmin[a] + fragment[i].bmax[a] );
		// allocate data for O(N) stable partition
		flag = (uint8_t*)AlignedAlloc( triCount );
		for (int a = 0; a < 3; a++) sortedIdx[a] = (Index*)AlignedAlloc( triCount * sizeof( Index ) );
	#ifdef ENABLE_THREADED_BUILDS
		// the index scratch follows the three key arrays, aligned to the index size.
		Index* primTmp1 = (Index*)(bvhNode + 2) + (3 * triCount * sizeof( uint32_t ) + sizeof( Index ) - 1) / sizeof( Index );
		Index* primTmp2 = primTmp1 + triCount;
		memcpy( primTmp1, primIdx, triCount * sizeof( Index ) );
		memcpy( primTmp2, primIdx, triCount * sizeof( Index ) );
		BVHRadixSortArgs<Index> ra = { { primIdx, primTmp1, primTmp2 },
			{ sortedIdx[0], sortedIdx[1], sortedIdx[2] }, { sortKey[0], sortKey[1], sortKey[2] }, (int)triCount };
		tinybvh_parallel_for( context, 3, &BVHRadixSortAxis<Index>, &ra );
	#else
		for (uint32_t a = 0; a < 3; a++) RadixSort( primIdx, sortedIdx[a], sortKey[a], (int)triCount );
	#endif
		// allocate space for right sweep. The buffer doubles as index scratch for the
		// stable partition and is sized for the wider of the two types.
		SARs = (Float*)AlignedAlloc( triCount * (sizeof( Index ) > sizeof( Float ) ? sizeof( Index ) : sizeof( Float )) );
		// per-axis copies of the fragment bounds, in sorted order.
		// cost: 3x24 bytes per fragment, plus 24 for the partition scratch.
		for (int a = 0; a < 3; a++) sortedBnds[a] = (SweepBounds*)AlignedAlloc( triCount * sizeof( SweepBounds ) );
		tmpBnds = (SweepBounds*)AlignedAlloc( triCount * sizeof( SweepBounds ) );
		tinybvh_parallel_for( context, 3, &BVH::SweepGatherTask, this );
	}
	// subdivide root node recursively
	Index task[512], taskCount = 0;
	Vec3 minDim = (bvhNode->aabbMax - bvhNode->aabbMin) * 1e-20f;
	const Float cratio = c_int > 0 ? (c_trav / c_int) : 0;
	while (1)
	{
		while (1)
		{
			BVHNode& node = bvhNode[nodeIdx];
			// update node bounds
			node.aabbMin = Vec3( bvh_far<Float> ), node.aabbMax = Vec3( -bvh_far<Float> );
			const SweepBounds* nodeBnd = sortedBnds[0] + node.leftFirst;
			for (Index i = 0; i < node.triCount; i++)
			{
				node.aabbMin = tinybvh_min( node.aabbMin, nodeBnd[i].bmin );
				node.aabbMax = tinybvh_max( node.aabbMax, nodeBnd[i].bmax );
			}
			if (node.triCount == 1) break; // can't split one triangle.
			const Vec3 extent = node.aabbMax - node.aabbMin;
			Float splitCost = ((Float)node.triCount - cratio) * node.SurfaceArea();
			uint32_t splitAxis = 0;
			Index splitPos = 0;
			for (uint32_t a = 0; a < 3; a++) if (extent[a] > minDim[a])
			{
				const SweepBounds* bnd = sortedBnds[a] + node.leftFirst;
				Float* sar = SARs + node.leftFirst;
				Index firstRightTri = 1;
				Vec3 Rmin( bvh_far<Float> ), Rmax( -bvh_far<Float> );
				// sweep from right to left
				for (Index i = 0; i < node.triCount; i++)
				{
					const Index j = node.triCount - i - 1;
					const Float SAR = (Float)i * tinybvh_halfarea( Rmax - Rmin );
					sar[j] = SAR;
					Rmin = tinybvh_min( Rmin, bnd[j].bmin );
					Rmax = tinybvh_max( Rmax, bnd[j].bmax );
					if (SAR >= splitCost)
					{
						// right side's cost is already greater than lowest cost and will only increase. Stop early
						firstRightTri = j + 1;
						break;
					}
				}
				// sweep from left to right
				Vec3 Lmin( bvh_far<Float> ), Lmax( -bvh_far<Float> );
				for (Index i = 0; i < firstRightTri - 1; i++)
				{
					Lmin = tinybvh_min( Lmin, bnd[i].bmin );
					Lmax = tinybvh_max( Lmax, bnd[i].bmax );
				}
				for (Index i = firstRightTri - 1; i < node.triCount - 1; i++)
				{
					Lmin = tinybvh_min( Lmin, bnd[i].bmin );
					Lmax = tinybvh_max( Lmax, bnd[i].bmax );
					const Float SAL = (Float)(i + 1) * tinybvh_halfarea( Lmax - Lmin );
					const Float C = SAL + sar[i];
					if (C < splitCost) splitCost = C, splitPos = i + 1, splitAxis = a;
					else if (SAL >= splitCost) break;
				}
			}
			if (splitPos == 0) break; // no split beats not splitting; make this node a leaf.
			// partition
			for (Index i = 0; i < splitPos; i++) flag[sortedIdx[splitAxis][node.leftFirst + i]] = 0; // "left"
			for (Index i = splitPos; i < node.triCount; i++) flag[sortedIdx[splitAxis][node.leftFirst + i]] = 1; // "right"
			// stable partition needs temp buffer, let's reuse memory
			Index* tmp = (Index*)SARs;
			for (uint32_t a = 0; a < 3; a++) if (a != splitAxis)
			{
				// the bounds travel with the indices, so the sweeps stay linear at every level.
				Index* idx = sortedIdx[a] + node.leftFirst;
				SweepBounds* bnd = sortedBnds[a] + node.leftFirst;
				Index* tmpIdx = tmp + node.leftFirst;
				SweepBounds* tmpBnd = tmpBnds + node.leftFirst;
				Index p0 = 0, p1 = 0;
				for (Index i = 0; i < node.triCount; i++)
				{
					const Index fi = idx[i];
					if (flag[fi]) tmpIdx[p1] = fi, tmpBnd[p1] = bnd[i], p1++;
					else idx[p0] = fi, bnd[p0] = bnd[i], p0++;
				}
				memcpy( idx + p0, tmpIdx, p1 * sizeof( Index ) );
				memcpy( bnd + p0, tmpBnd, p1 * sizeof( SweepBounds ) );
			}
			// create child nodes
			Index leftCount = splitPos, rightCount = node.triCount - leftCount;
			if (leftCount >= node.triCount || rightCount >= node.triCount || taskCount == BVH_NUM_ELEMS( task )) break;
			memcpy( primIdx + node.leftFirst, sortedIdx[splitAxis] + node.leftFirst, node.triCount * sizeof( Index ) );
			Index n;
		#ifdef ENABLE_THREADED_BUILDS
			if (threadedBuild) n = atomicNewNodePtr->fetch_add( 2 ); else n = newNodePtr, newNodePtr += 2;
		#else
			n = newNodePtr, newNodePtr += 2;
		#endif
			bvhNode[n].leftFirst = node.leftFirst;
			bvhNode[n].triCount = leftCount;
			bvhNode[n + 1].leftFirst = node.leftFirst + leftCount;
			bvhNode[n + 1].triCount = rightCount;
			node.leftFirst = n, node.triCount = 0;
		#ifdef ENABLE_THREADED_BUILDS
			if (threadedBuild && depth < MT_SPAWN_DEPTH &&
				tinybvh_max( leftCount, rightCount ) > MT_SPAWN_MIN_PRIMS)
			{
				// spawn the larger subtree, continue with the smaller one; root barrier joins.
				BVHBuildSubtreeArgs<Float, Index> a = { this, leftCount > rightCount ? (Index)n : (Index)n + 1, depth + 1 };
				tinybvh_spawn( context, &BVHBuildFullSweepSubtree<Float, Index>, &a, sizeof( a ) );
				nodeIdx = leftCount > rightCount ? ((Index)n + 1) : (Index)n;
				continue;
			}
		#endif
			// recurse: push the larger child and continue with the smaller one.
			if (leftCount > rightCount) task[taskCount++] = n, nodeIdx = n + 1;
			else task[taskCount++] = n + 1, nodeIdx = n;
		}
		// fetch subdivision task from stack
		if (taskCount == 0) break; else nodeIdx = task[--taskCount];
	}
	// cleanup allocated buffers when done
	if (depth == 0)
	{
	#ifdef ENABLE_THREADED_BUILDS
		if (threadedBuild)
		{
			tinybvh_barrier( context ); // wait for all spawned subtrees
			newNodePtr = atomicNewNodePtr->load();
			ContextDelete( atomicNewNodePtr );
		}
	#endif
		for (int a = 0; a < 3; a++) AlignedFree( sortedIdx[a] ), sortedIdx[a] = 0;
		for (int a = 0; a < 3; a++) AlignedFree( sortedBnds[a] ), sortedBnds[a] = 0;
		AlignedFree( tmpBnds ), tmpBnds = 0;
		AlignedFree( SARs ), SARs = 0;
		AlignedFree( flag ), flag = 0;
		aabbMin = bvhNode[0].aabbMin, aabbMax = bvhNode[0].aabbMax;
		refittable = true; // not using spatial splits: can refit this BVH
		may_have_holes = false; // this builder produces a continuous list of nodes
		bvh_over_aabbs = (verts == 0); // bvh over aabbs is suitable as TLAS
		usedNodes = newNodePtr;
		if (settings.usePresplitting)
		{
			// finalize indices in index array
			for (Index i = 0; i < triCount; i++) primIdx[i] = fragment[primIdx[i]].primIdx;
			if (settings.presplitPostPass) PresplitPostPass();
		}
	}
}

// SBVH builder. This builder introduces spatial splits during construction,
// improving tree quality at the expense of construction time.
template <typename Float, typename Index> void BVH<Float, Index>::BuildHQ( const Vertex* v, const Index p ) { BuildHQ( Slice( v, p * 3, sizeof( Vertex ) ) ); }
template <typename Float, typename Index> void BVH<Float, Index>::BuildHQ( const Vertex* v, const uint32_t* i, const Index p ) { BuildHQ( Slice( v, p * 3, sizeof( Vertex ) ), i, p ); }
template <typename Float, typename Index> void BVH<Float, Index>::BuildHQ( const Slice& v ) { PrepareHQBuild( v, 0, 0 ); BuildHQ(); }
template <typename Float, typename Index> void BVH<Float, Index>::BuildHQ( const Slice& v, const uint32_t* i, Index p ) { PrepareHQBuild( v, i, p ); BuildHQ(); }
template <typename Float, typename Index> void BVH<Float, Index>::PrepareHQBuild( const Slice& vertices, const uint32_t* indices, const Index prims )
{
	BVH_FATAL_ERROR_IF( vertices.count == 0, "BVH::PrepareHQBuild( .. ), zero primitives." );
	Index primCount = prims > 0 ? prims : vertices.count / 3;
	const Index slack = primCount >> 1; // for split prims
	const Index spaceNeeded = primCount * 3;
	// a rebuild is unsafe once the tree has been converted, whether or not we reallocate.
	BVH_FATAL_ERROR_IF( allocatedNodes > 0 && !rebuildable, "BVH::PrepareHQBuild( .. ), bvh not rebuildable." );
	// allocate memory on first build
	if (allocatedNodes < spaceNeeded)
	{
		AlignedFree( bvhNode );
		AlignedFree( primIdx );
		AlignedFree( fragment );
		bvhNode = (BVHNode*)AlignedAlloc( spaceNeeded * sizeof( BVHNode ) );
		allocatedNodes = spaceNeeded;
		memset( &bvhNode[1], 0, sizeof( BVHNode ) ); // node 1 remains unused, for cache line alignment.
		primIdx = (Index*)AlignedAlloc( (primCount + slack) * sizeof( Index ) );
		fragment = (Fragment*)AlignedAlloc( (primCount + slack) * sizeof( Fragment ) );
	}
	verts = vertices; // note: we're not copying this data; don't delete.
	idxCount = primCount + slack, triCount = primCount, vertIdx = (uint32_t*)indices;
	// prepare fragments
	BVHNode& root = bvhNode[0];
	root.leftFirst = 0, root.triCount = triCount, root.aabbMin = Vec3( bvh_far<Float> ), root.aabbMax = Vec3( -bvh_far<Float> );
	if (!indices)
	{
		BVH_FATAL_ERROR_IF( vertices.count == 0, "BVH::PrepareHQBuild( .. ), primCount == 0." );
		BVH_FATAL_ERROR_IF( prims != 0, "BVH::PrepareHQBuild( .. ), indices == 0." );
		// building a BVH over triangles specified as three 16-byte vertices each.
		for (Index i = 0; i < triCount; i++)
		{
			const Vertex v0 = verts[i * 3], v1 = verts[i * 3 + 1], v2 = verts[i * 3 + 2];
			const Vertex fmin = tinybvh_min( v0, tinybvh_min( v1, v2 ) );
			const Vertex fmax = tinybvh_max( v0, tinybvh_max( v1, v2 ) );
			fragment[i].bmin = fmin, fragment[i].bmax = fmax, fragment[i].primIdx = i, fragment[i].clipped = 0;
			root.aabbMin = tinybvh_min( root.aabbMin, fragment[i].bmin );
			root.aabbMax = tinybvh_max( root.aabbMax, fragment[i].bmax ), primIdx[i] = i;
		}
	}
	else
	{
		BVH_FATAL_ERROR_IF( vertices.count == 0, "BVH::PrepareHQBuild( .. ), empty vertex slice." );
		BVH_FATAL_ERROR_IF( prims == 0, "BVH::PrepareHQBuild( .. ), prims == 0." );
		// building a BVH over triangles consisting of vertices indexed by 'indices'.
		for (Index i = 0; i < triCount; i++)
		{
			const uint32_t i0 = indices[i * 3], i1 = indices[i * 3 + 1], i2 = indices[i * 3 + 2];
			const Vertex v0 = verts[i0], v1 = verts[i1], v2 = verts[i2];
			const Vertex fmin = tinybvh_min( v0, tinybvh_min( v1, v2 ) );
			const Vertex fmax = tinybvh_max( v0, tinybvh_max( v1, v2 ) );
			fragment[i].bmin = fmin, fragment[i].bmax = fmax, fragment[i].primIdx = i, fragment[i].clipped = 0;
			root.aabbMin = tinybvh_min( root.aabbMin, fragment[i].bmin );
			root.aabbMax = tinybvh_max( root.aabbMax, fragment[i].bmax ), primIdx[i] = i;
		}
	}
	// clear remainder of index array
	memset( primIdx + triCount, 0, slack * sizeof( Index ) );
	bvh_over_indices = indices != nullptr;
	// threading
#ifndef ENABLE_THREADED_BUILDS
	threadedBuild = false;
#else
	// build in parallel when given a sufficiently large input
	if (primCount < MT_BUILD_THRESHOLD || !context.spawn || !context.barrier) threadedBuild = false;
#endif
	// all set; actual build happens in BVH::BuildHQ.
}

template <typename Float, typename Index> void BVH<Float, Index>::BuildHQ()
{
	// Threaded SBVH builder entry point.
	const Index slack = triCount >> 1; // for split prims
	Index* idxTmp = (Index*)AlignedAlloc( (triCount + slack) * sizeof( Index ) );
	memset( idxTmp, 0, (triCount + slack) * sizeof( Index ) );
	// reset node pool
	newNodePtr = 2, nextFrag = triCount;
#ifdef ENABLE_THREADED_BUILDS
	if (threadedBuild)
		atomicNewNodePtr = this->template ContextNew<std::atomic<Index>>( (Index)2 ),
		atomicNextFrag = this->template ContextNew<std::atomic<Index>>( triCount );
#endif
	// subdivide recursively
	Index nodeIdx = 0, sliceStart = 0, sliceEnd = triCount + slack;
	uint32_t depth = 0;
	BuildHQTask( nodeIdx, depth, sliceStart, sliceEnd, idxTmp );
	// all done.
	AlignedFree( idxTmp );
	aabbMin = bvhNode[0].aabbMin, aabbMax = bvhNode[0].aabbMax;
	refittable = false; // can't refit an SBVH
	may_have_holes = false; // there may be holes in the index list, but not in the node list
#ifdef ENABLE_THREADED_BUILDS
	if (threadedBuild) newNodePtr = atomicNewNodePtr->load(), nextFrag = atomicNextFrag->load();
	ContextDelete( atomicNewNodePtr );
	ContextDelete( atomicNextFrag );
#endif
	usedNodes = newNodePtr;
	Compact();
}

// Helper function to build a subtree via the thread pool
template <typename Float, typename Index> struct BVHBuildHQArgs { BVH<Float, Index>* bvh; Index node; uint32_t depth; Index sliceStart, sliceEnd; Index* idxTmp; };
template <typename Float, typename Index> static void BVHBuildHQSubtree( void* payload )
{
	BVHBuildHQArgs<Float, Index>* a = (BVHBuildHQArgs<Float, Index>*)payload;
	a->bvh->BuildHQTask( a->node, a->depth, a->sliceStart, a->sliceEnd, a->idxTmp );
}
template <typename Float, typename Index> void BVH<Float, Index>::BuildHQTask( Index nodeIdx, uint32_t depth, Index sliceStart, Index sliceEnd, Index* idxTmp )
{
	// prepare subdivision
	ALIGNED( 64 ) SubdivTask localTask[512];
	uint32_t localTasks = 0;
	Vec3 bestLMin( 0 ), bestLMax( 0 ), bestRMin( 0 ), bestRMax( 0 );
	BVHNode& root = bvhNode[0];
	const Float rootArea = tinybvh_halfarea( root.aabbMax - root.aabbMin );
	const Vec3 minDim = (root.aabbMax - root.aabbMin) * 1e-7f /* don't touch, carefully picked */;
	// subdivide
	uint32_t binCount = hqbvhbins;
	while (1)
	{
		while (1)
		{
			// fetch node to subdivide
			BVHNode& node = bvhNode[nodeIdx];
			// alternating bin counts for optimizer.
			if (hqbvhoddeven) binCount = hqbvhbins + (depth & 1); // odd levels get one more
			// find optimal object split
			Vec3 binMin[3][MAXHQBINS], binMax[3][MAXHQBINS];
			for (uint32_t a = 0; a < 3; a++) for (uint32_t i = 0; i < binCount; i++)
				binMin[a][i] = Vec3( bvh_far<Float> ), binMax[a][i] = Vec3( -bvh_far<Float> );
			Index count[3][MAXHQBINS];
			for (uint32_t i = 0; i < 3; i++) memset( count[i], 0, binCount * sizeof( Index ) );
			const Vec3 rpd3 = Vec3( Vec3( (Float)binCount ) / (node.aabbMax - node.aabbMin) ), nmin3 = node.aabbMin;
			for (Index i = 0; i < node.triCount; i++) // process all tris for x,y and z at once
			{
				const Index fi = primIdx[node.leftFirst + i];
				const Vec3 fbi = ((fragment[fi].bmin + fragment[fi].bmax) * 0.5f - nmin3) * rpd3;
			bvhint3 bi( (int32_t)fbi.x, (int32_t)fbi.y, (int32_t)fbi.z );
				bi.x = tinybvh_clamp( bi.x, 0, binCount - 1 );
				bi.y = tinybvh_clamp( bi.y, 0, binCount - 1 );
				bi.z = tinybvh_clamp( bi.z, 0, binCount - 1 );
				binMin[0][bi.x] = tinybvh_min( binMin[0][bi.x], fragment[fi].bmin );
				binMax[0][bi.x] = tinybvh_max( binMax[0][bi.x], fragment[fi].bmax ), count[0][bi.x]++;
				binMin[1][bi.y] = tinybvh_min( binMin[1][bi.y], fragment[fi].bmin );
				binMax[1][bi.y] = tinybvh_max( binMax[1][bi.y], fragment[fi].bmax ), count[1][bi.y]++;
				binMin[2][bi.z] = tinybvh_min( binMin[2][bi.z], fragment[fi].bmin );
				binMax[2][bi.z] = tinybvh_max( binMax[2][bi.z], fragment[fi].bmax ), count[2][bi.z]++;
			}
			// calculate per-split totals
			Float noSplitCost = NoSplitCostSAH( node.triCount );
			Float splitCost = noSplitCost, rSAV = 1.0f / node.SurfaceArea();
			uint32_t bestAxis = 0, bestPos = 0;
			for (int32_t a = 0; a < 3; a++) if ((node.aabbMax[a] - node.aabbMin[a]) > minDim[a])
			{
				Vec3 lBMin[MAXHQBINS - 1], rBMin[MAXHQBINS - 1], l1( bvh_far<Float> ), l2( -bvh_far<Float> );
				Vec3 lBMax[MAXHQBINS - 1], rBMax[MAXHQBINS - 1], r1( bvh_far<Float> ), r2( -bvh_far<Float> );
				Float AL[MAXHQBINS - 1], AR[MAXHQBINS - 1];		// left and right area per split plane
				Index NL[MAXHQBINS - 1], NR[MAXHQBINS - 1];	// summed left and right tricount
				for (Index lN = 0, rN = 0, i = 0; i < binCount - 1; i++)
				{
					lBMin[i] = l1 = tinybvh_min( l1, binMin[a][i] );
					rBMin[binCount - 2 - i] = r1 = tinybvh_min( r1, binMin[a][binCount - 1 - i] );
					lBMax[i] = l2 = tinybvh_max( l2, binMax[a][i] );
					rBMax[binCount - 2 - i] = r2 = tinybvh_max( r2, binMax[a][binCount - 1 - i] );
					lN += count[a][i], rN += count[a][binCount - 1 - i];
					NL[i] = lN, NR[binCount - 2 - i] = rN;
					AL[i] = lN == 0 ? bvh_far<Float> : tinybvh_halfarea( l2 - l1 );
					AR[binCount - 2 - i] = rN == 0 ? bvh_far<Float> : tinybvh_halfarea( r2 - r1 );
				}
				// evaluate bin totals to find best position for object split
				for (uint32_t i = 0; i < binCount - 1; i++)
				{
					const Float C = SplitCostSAH( rSAV, AL[i], NL[i], AR[i], NR[i] );
					if (C >= splitCost) continue;
					splitCost = C, bestAxis = a, bestPos = i;
					bestLMin = lBMin[i], bestRMin = rBMin[i], bestLMax = lBMax[i], bestRMax = rBMax[i];
				}
			}
			// consider a spatial split
			bool spatial = false;
			Index bestNL = 0, bestNR = 0, budget = sliceEnd - sliceStart;
			Vec3 spatialUnion = bestLMax - bestRMin;
			Float spatialOverlap = (tinybvh_halfarea( spatialUnion )) / rootArea;
			if (budget > node.triCount && (spatialOverlap > 1e-4f || splitCost >= noSplitCost))
			{
				Float minSplitCost = splitCost * 0.985f; // don't accept a spatial split for minimal gain
				for (int a = 0; a < 3; a++) if ((node.aabbMax[a] - node.aabbMin[a]) > minDim[a])
				{
					// setup bins
					Vec3 sbinMin[MAXHQBINS], sbinMax[MAXHQBINS];
					int countIn[MAXHQBINS], countOut[MAXHQBINS];
					memset( countIn, 0, binCount * 4 );
					memset( countOut, 0, binCount * 4 );
					for (uint32_t i = 0; i < binCount; i++) sbinMin[i] = Vec3( bvh_far<Float> ), sbinMax[i] = Vec3( -bvh_far<Float> );
					// populate bins with clipped fragments
					const Float planeDist = (node.aabbMax[a] - node.aabbMin[a]) / (binCount * 0.9999f);
					const Float rPlaneDist = 1.0f / planeDist, nodeMin = node.aabbMin[a];
					for (Index i = 0; i < node.triCount; i++)
					{
						const Index fi = primIdx[node.leftFirst + i];
						const int bin1 = tinybvh_clamp( (int32_t)((fragment[fi].bmin[a] - nodeMin) * rPlaneDist), 0, binCount - 1 );
						const int bin2 = tinybvh_clamp( (int32_t)((fragment[fi].bmax[a] - nodeMin) * rPlaneDist), 0, binCount - 1 );
						countIn[bin1]++, countOut[bin2]++;
						if (bin2 == bin1) // fragment fits in a single bin
							sbinMin[bin1] = tinybvh_min( sbinMin[bin1], fragment[fi].bmin ),
							sbinMax[bin1] = tinybvh_max( sbinMax[bin1], fragment[fi].bmax );
						else for (int j = bin1; j <= bin2; j++)
						{
							// clip fragment to each bin it overlaps
							Vec3 bmin = node.aabbMin, bmax = node.aabbMax;
							bmin[a] = nodeMin + planeDist * j;
							bmax[a] = j == (int)(binCount - 2) ? node.aabbMax[a] : (bmin[a] + planeDist);
							Fragment orig = fragment[fi];
							Fragment tmpFrag;
							if (!ClipFrag( orig, tmpFrag, bmin, bmax, a )) continue;
							sbinMin[j] = tinybvh_min( sbinMin[j], tmpFrag.bmin );
							sbinMax[j] = tinybvh_max( sbinMax[j], tmpFrag.bmax );
						}
					}
					// evaluate split candidates
					Vec3 lBMin[MAXHQBINS - 1], rBMin[MAXHQBINS - 1], l1( bvh_far<Float> ), l2( -bvh_far<Float> );
					Vec3 lBMax[MAXHQBINS - 1], rBMax[MAXHQBINS - 1], r1( bvh_far<Float> ), r2( -bvh_far<Float> );
					Float AL[MAXHQBINS], AR[MAXHQBINS];
					Index NL[MAXHQBINS], NR[MAXHQBINS];
					for (Index lN = 0, rN = 0, i = 0; i < binCount - 1; i++)
					{
						lBMin[i] = l1 = tinybvh_min( l1, sbinMin[i] ), rBMin[binCount - 2 - i] = r1 = tinybvh_min( r1, sbinMin[binCount - 1 - i] );
						lBMax[i] = l2 = tinybvh_max( l2, sbinMax[i] ), rBMax[binCount - 2 - i] = r2 = tinybvh_max( r2, sbinMax[binCount - 1 - i] );
						lN += countIn[i], rN += countOut[binCount - 1 - i];
						AL[i] = lN == 0 ? bvh_far<Float> : tinybvh_halfarea( l2 - l1 );
						AR[binCount - 2 - i] = rN == 0 ? bvh_far<Float> : tinybvh_halfarea( r2 - r1 );
						NL[i] = lN, NR[binCount - 2 - i] = rN;
					}
					// find best position for spatial split
					for (uint32_t i = 0; i < binCount - 1; i++)
					{
						const Float Cspatial = SplitCostSAH( rSAV, AL[i], NL[i], AR[i], NR[i] );
						if (Cspatial < minSplitCost && NL[i] + NR[i] < budget && NL[i] > 0 && NR[i] > 0)
						{
							spatial = true, minSplitCost = splitCost = Cspatial, bestAxis = a, bestPos = i;
							bestLMin = lBMin[i], bestLMax = lBMax[i], bestRMin = rBMin[i], bestRMax = rBMax[i];
							bestNL = NL[i], bestNR = NR[i]; // for unsplitting
							bestLMax[a] = bestRMin[a]; // accurate
						}
					}
				}
			}
			// evaluate best split cost
			if (splitCost >= noSplitCost)
			{
				for (Index i = 0; i < node.triCount; i++)
					primIdx[node.leftFirst + i] = fragment[primIdx[node.leftFirst + i]].primIdx;
				break; // not splitting is better.
			}
			// double-buffered partition
			Index A = sliceStart, B = sliceEnd, src = node.leftFirst;
			if (spatial)
			{
				// spatial partitioning
				const Float planeDist = (node.aabbMax[bestAxis] - node.aabbMin[bestAxis]) / (binCount * 0.9999f);
				const Float rPlaneDist = 1.0f / planeDist, nodeMin = node.aabbMin[bestAxis];
				for (Index i = 0; i < node.triCount; i++)
				{
					const Index fragIdx = primIdx[src++];
					const uint32_t bin1 = (uint32_t)tinybvh_max( (fragment[fragIdx].bmin[bestAxis] - nodeMin) * rPlaneDist, Float( 0 ) );
					const uint32_t bin2 = (uint32_t)tinybvh_max( (fragment[fragIdx].bmax[bestAxis] - nodeMin) * rPlaneDist, Float( 0 ) );
					if (bin2 <= bestPos) idxTmp[A++] = fragIdx; else if (bin1 > bestPos) idxTmp[--B] = fragIdx; else
					{
					#if defined SBVH_UNSPLITTING
						// unsplitting: 1. Calculate what happens if we add this primitive entirely to the left side
						if (bestNR > 1)
						{
							Vec3 unsplitLMin = tinybvh_min( bestLMin, fragment[fragIdx].bmin );
							Vec3 unsplitLMax = tinybvh_max( bestLMax, fragment[fragIdx].bmax );
							Float AL = tinybvh_halfarea( unsplitLMax - unsplitLMin );
							Float AR = tinybvh_halfarea( bestRMax - bestRMin );
							Float CunsplitLeft = SplitCostSAH( rSAV, AL, bestNL, AR, bestNR - 1 );
							if (CunsplitLeft <= splitCost)
							{
								bestNR--, splitCost = CunsplitLeft, idxTmp[A++] = fragIdx;
								bestLMin = unsplitLMin, bestLMax = unsplitLMax;
								continue;
							}
						}
						// 2. Calculate what happens if we add this primitive entirely to the right side
						if (bestNL > 1)
						{
							const Vec3 unsplitRMin = tinybvh_min( bestRMin, fragment[fragIdx].bmin );
							const Vec3 unsplitRMax = tinybvh_max( bestRMax, fragment[fragIdx].bmax );
							const Float AL = tinybvh_halfarea( bestLMax - bestLMin );
							const Float AR = tinybvh_halfarea( unsplitRMax - unsplitRMin );
							const Float CunsplitRight = SplitCostSAH( rSAV, AL, bestNL - 1, AR, bestNR );
							if (CunsplitRight <= splitCost)
							{
								bestNL--, splitCost = CunsplitRight, idxTmp[--B] = fragIdx;
								bestRMin = unsplitRMin, bestRMax = unsplitRMax;
								continue;
							}
						}
					#endif
						// split straddler
						ALIGNED( 64 ) Fragment part1, part2; // keep all clipping in a single cacheline.
						Float splitPos = bestLMax[bestAxis];
						if (SplitFrag( fragment[fragIdx], part1, part2, bestAxis, splitPos ))
						{
						#ifdef ENABLE_THREADED_BUILDS
							Index newFragIdx = threadedBuild ? atomicNextFrag->fetch_add( 1 ) : nextFrag++;
						#else
							Index newFragIdx = nextFrag++;
						#endif
							fragment[fragIdx] = part1, idxTmp[A++] = fragIdx;
							fragment[newFragIdx] = part2, idxTmp[--B] = newFragIdx;
						}
						else // didn't work out; see what we can do.
						{
							const Float sahLeft = tinybvh_halfarea( part1.bmax - part1.bmin );
							if (sahLeft > 0) idxTmp[A++] = fragIdx; else idxTmp[--B] = fragIdx;
						}
					}
				}
				// for spatial splits, we fully refresh the bounds: clipping is never fully stable..
				bestLMin = bestRMin = Vec3( bvh_far<Float> ), bestLMax = bestRMax = Vec3( -bvh_far<Float> );
				for (Index i = sliceStart; i < A; i++)
					bestLMin = tinybvh_min( bestLMin, fragment[idxTmp[i]].bmin ),
					bestLMax = tinybvh_max( bestLMax, fragment[idxTmp[i]].bmax );
				for (Index i = B; i < sliceEnd; i++)
					bestRMin = tinybvh_min( bestRMin, fragment[idxTmp[i]].bmin ),
					bestRMax = tinybvh_max( bestRMax, fragment[idxTmp[i]].bmax );
			}
			else
			{
				// object partitioning
				const Float rpd = rpd3[bestAxis], nmin = nmin3[bestAxis];
				for (Index i = 0; i < node.triCount; i++)
				{
					const Index fr = primIdx[src + i];
					int32_t bi = (int32_t)(((fragment[fr].bmin[bestAxis] + fragment[fr].bmax[bestAxis]) * 0.5f - nmin) * rpd);
					bi = tinybvh_clamp( bi, 0, binCount - 1 );
					if (bi <= (int32_t)bestPos) idxTmp[A++] = fr; else idxTmp[--B] = fr;
				}
			}
			// copy back slice data
			memcpy( primIdx + sliceStart, idxTmp + sliceStart, (sliceEnd - sliceStart) * sizeof( Index ) );
			// create child nodes
			Index leftCount = A - sliceStart, rightCount = sliceEnd - B;
			if (leftCount == 0 || rightCount == 0)
			{
				// spatial split failed. We shouldn't get here, but we do sometimes..
				for (Index i = 0; i < node.triCount; i++)
					primIdx[node.leftFirst + i] = fragment[primIdx[node.leftFirst + i]].primIdx;
				node.aabbMin = tinybvh_min( bestLMin, bestRMin );
				node.aabbMax = tinybvh_max( bestLMax, bestRMax );
				break;
			}
			Index leftChildIdx;
		#ifdef ENABLE_THREADED_BUILDS
			if (threadedBuild) leftChildIdx = atomicNewNodePtr->fetch_add( 2 ); else leftChildIdx = newNodePtr, newNodePtr += 2;
		#else
			leftChildIdx = newNodePtr, newNodePtr += 2;
		#endif
			const Index rightChildIdx = leftChildIdx + 1;
			bvhNode[leftChildIdx].aabbMin = bestLMin, bvhNode[leftChildIdx].aabbMax = bestLMax;
			bvhNode[leftChildIdx].leftFirst = sliceStart, bvhNode[leftChildIdx].triCount = leftCount;
			bvhNode[rightChildIdx].aabbMin = bestRMin, bvhNode[rightChildIdx].aabbMax = bestRMax;
			bvhNode[rightChildIdx].leftFirst = B, bvhNode[rightChildIdx].triCount = rightCount;
			node.leftFirst = leftChildIdx, node.triCount = 0;
			// recurse
			if (depth < MT_SPAWN_DEPTH && threadedBuild)
			{
				// spawn both child subtrees and return; the root barrier joins them.
				BVHBuildHQArgs<Float, Index> a0 = { this, (Index)leftChildIdx, depth + 1, sliceStart, (A + B) >> 1, idxTmp };
				BVHBuildHQArgs<Float, Index> a1 = { this, (Index)rightChildIdx, depth + 1, (A + B) >> 1, sliceEnd, idxTmp };
				tinybvh_spawn( context, &BVHBuildHQSubtree<Float, Index>, &a0, sizeof( a0 ) );
				tinybvh_spawn( context, &BVHBuildHQSubtree<Float, Index>, &a1, sizeof( a1 ) );
				break;
			}
			// proceed with left child, push right child on local stack
			localTask[localTasks].node = rightChildIdx, localTask[localTasks].depth = depth;
			localTask[localTasks].sliceStart = (A + B) >> 1, localTask[localTasks++].sliceEnd = sliceEnd;
			nodeIdx = leftChildIdx, sliceEnd = (A + B) >> 1;
		}
		// pop a local task, if any are left
		if (localTasks == 0) break;
		nodeIdx = localTask[--localTasks].node, depth = localTask[localTasks].depth;
		sliceStart = localTask[localTasks].sliceStart, sliceEnd = localTask[localTasks].sliceEnd;
	}
	// all done; wait for all spawned subtrees at the root.
	if (depth == 0 && threadedBuild) tinybvh_barrier( context );
}

template <typename Float, typename Index> Float BVH<Float, Index>::SplitCostSAH( const Float rAparent, const Float Aleft, const Index Nleft, const Float Aright, const Index Nright ) const
{
	const Index lN = l_quads ? (((Nleft + 3) >> 2) * 4) : Nleft;
	const Index rN = l_quads ? (((Nright + 3) >> 2) * 4) : Nright;
	return c_trav + c_int * rAparent * (Aleft * (Float)lN + Aright * (Float)rN);
}

template <typename Float, typename Index> Float BVH<Float, Index>::NoSplitCostSAH( const Index Nparent ) const
{
	return (Float)(l_quads ? (((Nparent + 3) >> 2) * 4) : Nparent) * c_int;
}

// BVH TOOLS

#define BVH_METRIC_TASKS 256 // reduction tasks for tree quality metrics.

template <typename Float, typename Index> struct BVHMetricArgs
{
	const BVH<Float, Index>* bvh;
	const Index* node;		// node indices to score
	Index nodeCount;
	uint32_t tasks;
	double* epo;				// [tasks] EPO accumulators, or 0 to skip
	double* sah;				// [tasks] SAH accumulators, or 0 to skip
	double* area;				// [tasks] primitive area accumulators, or 0 to skip
	Index primCount;
};

template <typename Float, typename Index> int32_t BVH<Float, Index>::PrimCount( const Index nodeIdx ) const
{
	// determine the total number of primitives / fragments in leaf nodes.
	const BVHNode& n = bvhNode[nodeIdx];
	return n.isLeaf() ? (int32_t)n.triCount : (PrimCount( n.leftFirst ) + PrimCount( n.leftFirst + 1 ));
}

template <typename Float, typename Index> Index BVH<Float, Index>::CollectNodes( const Index root, Index* list, const Index cap ) const
{
	// iterative pre-order walk, storing the indices of the nodes reachable from 'root'.
	Index count = 0, nodeIdx = root, stack[TINYBVH_STACK_SIZE], stackPtr = 0;
	while (1)
	{
		BVH_FATAL_ERROR_IF( count == cap, "BVH::CollectNodes, node list overflow; malformed tree?" );
		list[count++] = nodeIdx;
		const BVHNode& n = bvhNode[nodeIdx];
		if (!n.isLeaf())
		{
			BVH_FATAL_ERROR_IF( stackPtr == TINYBVH_STACK_SIZE, "BVH::CollectNodes, tree too deep." );
			stack[stackPtr++] = n.leftFirst + 1, nodeIdx = n.leftFirst;
			continue;
		}
		if (stackPtr == 0) break;
		nodeIdx = stack[--stackPtr];
	}
	return count;
}

template <typename Float, typename Index> void BVH<Float, Index>::MetricTask( const uint32_t task, void* payload )
{
	// score a contiguous slice of the node list
	const BVHMetricArgs<Float, Index>* a = (const BVHMetricArgs<Float, Index>*)payload;
	const BVH* bvh = a->bvh;
	double epo = 0, sah = 0;
	const Index first = (Index)(((uint64_t)a->nodeCount * task) / a->tasks);
	const Index last = (Index)(((uint64_t)a->nodeCount * (task + 1)) / a->tasks);
	for (Index i = first; i < last; i++)
	{
		const Index idx = a->node[i];
		const BVHNode& n = bvh->bvhNode[idx];
		const double w = n.isLeaf() ? ((double)bvh->c_int * n.triCount) : (double)bvh->c_trav;
		if (a->sah) sah += w * (double)n.SurfaceArea();
		if (a->epo) epo += w * (double)bvh->EPOArea( idx );
	}
	if (a->epo) a->epo[task] = epo;
	if (a->sah) a->sah[task] = sah;
	if (a->area)
	{
		double area = 0;
		const Index p0 = (Index)(((uint64_t)a->primCount * task) / a->tasks);
		const Index p1 = (Index)(((uint64_t)a->primCount * (task + 1)) / a->tasks);
		for (Index i = p0; i < p1; i++) area += (double)bvh->PrimArea( i );
		a->area[task] = area;
	}
}

template <typename Float, typename Index> Float BVH<Float, Index>::SAHCost( const Index nodeIdx, uint32_t ) const
{
	BVH_FATAL_ERROR_IF( bvhNode == 0, "BVH::SAHCost( .. ), bvhNode == 0." );
	const BVHNode& root = bvhNode[nodeIdx];
	if (root.isLeaf()) return c_int * root.SurfaceArea() * root.triCount;
	double cost = 0;
	Index idx = nodeIdx, stack[TINYBVH_STACK_SIZE], stackPtr = 0;
	while (1)
	{
		const BVHNode& n = bvhNode[idx];
		if (n.isLeaf()) cost += (double)c_int * n.triCount * n.SurfaceArea();
		else
		{
			cost += (double)c_trav * n.SurfaceArea();
			BVH_FATAL_ERROR_IF( stackPtr == TINYBVH_STACK_SIZE, "BVH::SAHCost, tree too deep." );
			stack[stackPtr++] = n.leftFirst + 1, idx = n.leftFirst;
			continue;
		}
		if (stackPtr == 0) break;
		idx = stack[--stackPtr];
	}
	if (nodeIdx != 0) return (Float)cost;
	const Float rootArea = root.SurfaceArea();
	return rootArea > 0 ? (Float)(cost / rootArea) : 0.0f;
}

template <typename Float, typename Index> void BVH<Float, Index>::ConvertFrom( const BVH_Verbose& original, bool compact )
{
	// allocate space
	const Index spaceNeeded = compact ? original.usedNodes : original.allocatedNodes;
	CopyBasePropertiesFrom( original );
	if (allocatedNodes < spaceNeeded)
	{
		AlignedFree( bvhNode );
		bvhNode = (BVHNode*)AlignedAlloc( spaceNeeded * sizeof( BVHNode ) );
		allocatedNodes = spaceNeeded;
	}
	memset( bvhNode, 0, sizeof( BVHNode ) * spaceNeeded );
	this->verts = original.verts;
	// correctly handle ownership of primIdx: make a fresh copy of the indices, unless
	// we're converting back our own data (when primIdx == original.primIdx).
	if (primIdx != original.primIdx)
	{
		AlignedFree( primIdx ); // whatever we owned before is about to be replaced
		primIdx = 0;
		if (original.primIdx != 0 && original.idxCount > 0)
		{
			primIdx = (Index*)AlignedAlloc( original.idxCount * sizeof( Index ) );
			memcpy( primIdx, original.primIdx, original.idxCount * sizeof( Index ) );
		}
	}
	// start conversion
	Index srcNodeIdx = 0, dstNodeIdx = 0;
	newNodePtr = 2;
	Index srcStack[1024], dstStack[1024], stackPtr = 0;
	while (1)
	{
		const typename BVH_Verbose::BVHNode& orig = original.bvhNode[srcNodeIdx];
		bvhNode[dstNodeIdx].aabbMin = orig.aabbMin;
		bvhNode[dstNodeIdx].aabbMax = orig.aabbMax;
		if (orig.isLeaf())
		{
			bvhNode[dstNodeIdx].triCount = orig.triCount;
			bvhNode[dstNodeIdx].leftFirst = orig.firstTri;
			if (stackPtr == 0) break;
			srcNodeIdx = srcStack[--stackPtr];
			dstNodeIdx = dstStack[stackPtr];
		}
		else
		{
			bvhNode[dstNodeIdx].leftFirst = newNodePtr;
			Index srcRightIdx = orig.right;
			srcNodeIdx = orig.left, dstNodeIdx = newNodePtr++;
			srcStack[stackPtr] = srcRightIdx;
			dstStack[stackPtr++] = newNodePtr++;
		}
	}
	usedNodes = original.usedNodes;
}

template <typename Float, typename Index> Float BVH<Float, Index>::TriArea( const Index triIdx ) const
{
	const Index vidx = triIdx * 3;
	Vec3 v0, v1, v2;
	if (vertIdx) v0 = verts[vertIdx[vidx]], v1 = verts[vertIdx[vidx + 1]], v2 = verts[vertIdx[vidx + 2]];
	else v0 = verts[vidx], v1 = verts[vidx + 1], v2 = verts[vidx + 2];
	return 0.5f * tinybvh_length( tinybvh_cross( v1 - v0, v2 - v0 ) );
}

template <typename Float, typename Index> Float BVH<Float, Index>::PrimArea( const Index slot ) const
{
	return TriArea( primIdx[slot] );
}

template <typename Node> inline bool tinybvh_aabbs_overlap( const Node& node1, const Node& node2 )
{
	const auto bmin1 = node1.aabbMin, bmin2 = node2.aabbMin;
	const auto bmax1 = node1.aabbMax, bmax2 = node2.aabbMax;
	return bmin1.x <= bmax2.x && bmax1.x >= bmin2.x && bmin1.y <= bmax2.y &&
		bmax1.y >= bmin2.y && bmin1.z <= bmax2.z && bmax1.z >= bmin2.z;
}

template <typename Vertex, typename Vec3> inline bool tinybvh_tri_inside_box( const Vertex& v0, const Vertex& v1, const Vertex& v2, const Vec3& bmin, const Vec3& bmax )
{
	bool allin = v0.x >= bmin.x && v0.x <= bmax.x && v0.y >= bmin.y && v0.y <= bmax.y && v0.z >= bmin.z && v0.z <= bmax.z;
	allin &= v1.x >= bmin.x && v1.x <= bmax.x && v1.y >= bmin.y && v1.y <= bmax.y && v1.z >= bmin.z && v1.z <= bmax.z;
	allin &= v2.x >= bmin.x && v2.x <= bmax.x && v2.y >= bmin.y && v2.y <= bmax.y && v2.z >= bmin.z && v2.z <= bmax.z;
	return allin;
}

// Area of the part of triangle v0, v1, v2 inside the box, by clipping against its six planes.
template <typename Float, typename Vec3> Float tinybvh_clipped_tri_area( const Vec3& v0, const Vec3& v1, const Vec3& v2, const Vec3& bmin, const Vec3& bmax )
{
	uint32_t Nin = 3;
	Vec3 vin[10] = { v0, v1, v2 }, vout[10], C;
	for (uint32_t a = 0; a < 3; a++)
	{
		uint32_t Nout = 0;
		const Float l = bmin[a], r = bmax[a];
		for (uint32_t v = 0; v < Nin; v++)
		{
			Vec3 vert0 = vin[v], vert1 = vin[(v + 1) % Nin];
			const bool v0in = vert0[a] >= l, v1in = vert1[a] >= l;
			if (!(v0in || v1in)) continue; else if (v0in ^ v1in)
				C = vert0 + (l - vert0[a]) / (vert1[a] - vert0[a]) * (vert1 - vert0),
				C[a] = l /* accurate */, vout[Nout++] = C;
			if (v1in) vout[Nout++] = vert1;
		}
		Nin = 0;
		for (uint32_t v = 0; v < Nout; v++)
		{
			Vec3 vert0 = vout[v], vert1 = vout[(v + 1) % Nout];
			const bool v0in = vert0[a] <= r, v1in = vert1[a] <= r;
			if (!(v0in || v1in)) continue; else if (v0in ^ v1in)
				C = vert0 + (r - vert0[a]) / (vert1[a] - vert0[a]) * (vert1 - vert0),
				C[a] = r /* accurate */, vin[Nin++] = C;
			if (v1in) vin[Nin++] = vert1;
		}
	}
	if (Nin < 3) return 0;
	// area of the remaining convex polygon
	Float area = 0;
	const Vec3 p0 = vin[0];
	for (uint32_t j = 0; j < Nin - 2; j++) area += Float( 0.5 ) * tinybvh_length( tinybvh_cross( vin[j + 1] - p0, vin[j + 2] - p0 ) );
	return area;
}

// Area of the primitives in the leafs below nodeIdx that lies inside the subtree's box.
// tiny_bvh_x86_float.h overloads the two box tests with SSE versions for float.
template <typename Float, typename Index> Float BVH<Float, Index>::EPOArea( const Index subtreeRoot, const Index nodeIdx ) const
{
	// abort if we reached the subtree
	if (nodeIdx == subtreeRoot) return 0;
	const BVHNode& n = bvhNode[nodeIdx];
	const BVHNode& subtree = bvhNode[subtreeRoot];
	// handle case where n is a leaf node
	Float area = 0;
	if (n.isLeaf())
	{
		const Vec3 bmin = subtree.aabbMin, bmax = subtree.aabbMax;
		for (Index i = 0; i < n.triCount; i++)
		{
			const Index vidx = primIdx[n.leftFirst + i] * 3;
			Vertex v0, v1, v2;
			if (vertIdx) v0 = verts[vertIdx[vidx]], v1 = verts[vertIdx[vidx + 1]], v2 = verts[vertIdx[vidx + 2]];
			else v0 = verts[vidx], v1 = verts[vidx + 1], v2 = verts[vidx + 2];
			// Early out: triangle fully inside the subtree AABB?
			if (tinybvh_tri_inside_box( v0, v1, v2, bmin, bmax )) area += Float( 0.5 ) * tinybvh_length( tinybvh_cross( Vec3( v1 - v0 ), Vec3( v2 - v0 ) ) );
			else area += tinybvh_clipped_tri_area<Float>( Vec3( v0 ), Vec3( v1 ), Vec3( v2 ), bmin, bmax );
		}
		return area;
	}
	// recurse if n is an inner node
	const BVHNode& left = bvhNode[n.leftFirst], & right = bvhNode[n.leftFirst + 1];
	if (tinybvh_aabbs_overlap( left, subtree )) area += EPOArea( subtreeRoot, n.leftFirst );
	if (tinybvh_aabbs_overlap( right, subtree )) area += EPOArea( subtreeRoot, n.leftFirst + 1 );
	return area;
}

template <typename Float, typename Index> Float BVH<Float, Index>::EPOCost( const Index nodeIdx, uint32_t ) const
{
	BVH_FATAL_ERROR_IF( bvhNode == 0, "BVH::EPOCost( .. ), bvhNode == 0." );
	BVH_FATAL_ERROR_IF( verts == 0, "BVH::EPOCost( .. ), bvh has no vertex data." );
	Index* list = (Index*)AlignedAlloc( (size_t)usedNodes * sizeof( Index ) );
	const Index count = CollectNodes( nodeIdx, list, usedNodes );
	const uint32_t tasks = (uint32_t)tinybvh_min( (Index)BVH_METRIC_TASKS, count );
	ALIGNED( 64 ) double epo[BVH_METRIC_TASKS] = {}, sah[BVH_METRIC_TASKS] = {}, area[BVH_METRIC_TASKS] = {};
	// the SAH term and the total primitive area are sums over the same index
	// ranges, so they ride along in the same pass instead of costing a second one.
	BVHMetricArgs<Float, Index> args = { this, list, count, tasks, epo, sah, area, triCount };
	tinybvh_parallel_for( context, tasks, &BVH::MetricTask, &args );
	double epoSum = 0, sahSum = 0, areaSum = 0;
	for (uint32_t i = 0; i < tasks; i++)
		epoSum += epo[i], sahSum += sah[i], areaSum += area[i];
	AlignedFree( list );
	// EPO is only defined for a whole tree - for a subtree we return the raw sum.
	if (nodeIdx != 0) return (Float)epoSum;
	const Float rootArea = bvhNode[0].SurfaceArea();
	const Float sahCost = rootArea > 0 ? (Float)(sahSum / rootArea) : 0.0f;
	if (!(areaSum > 0)) return sahCost; // degenerate geometry: no EPO term
	return (1.0f - W_EPO) * sahCost + W_EPO * (Float)(epoSum / areaSum);
}

template <typename Float, typename Index> void BVH<Float, Index>::SplitLeafs( const Index maxPrims )
{
	Index stack[64], stackPtr = 0, nodeIdx = 0;
	while (1)
	{
		BVHNode& node = bvhNode[nodeIdx];
		if (node.isLeaf())
		{
			if (node.triCount > maxPrims)
			{
				BVHNode& left = bvhNode[newNodePtr], & right = bvhNode[newNodePtr + 1];
				left = node, right = node;
				right.leftFirst = node.leftFirst + maxPrims;
				right.triCount = node.triCount - maxPrims;
				left.triCount = maxPrims, node.leftFirst = newNodePtr, node.triCount = 0, newNodePtr += 2;
			}
			else
			{
				if (!stackPtr) break;
				nodeIdx = stack[--stackPtr];
			}
		}
		else
		{
			nodeIdx = node.leftFirst;
			stack[stackPtr++] = node.leftFirst + 1;
		}
	}
	usedNodes = newNodePtr;
}

template <typename Float, typename Index> Float BVH<Float, Index>::SplitPriority( const Fragment& f ) const
{
	auto fastCbrt = []( float x ) {
		uint32_t i;
		memcpy( &i, &x, sizeof( i ) );
		i = 0x2A51067Fu + i / 3u;
		Float y = tinybvh_as_float( i );
		y = (2.0f * y + x / (y * y)) * (1.0f / 3.0f); // refine with Newton-Raphson iterations.
		return y;
		};
	const Vec3 extent = f.bmax - f.bmin;
	const Float extentPrio = tinybvh_sqrf( extent[tinybvh_maxdim( extent )] );
	const Float boxArea = tinybvh_halfarea( extent );
	const Float triArea = TriArea( f.primIdx );
	const Float emptyAreaPrio = tinybvh_max( boxArea - triArea, Float( 0 ) );
	return fastCbrt( (float)(extentPrio * emptyAreaPrio) );
}

template <typename Float, typename Index> Float BVH<Float, Index>::GetNodeSize( const Float extent, const Float globalSize )
{
	// transform into [0.0, 1.0]
	const float alpha = (float)(extent / globalSize);
	// compute 2^(floor(log2(alpha)))
	uint32_t exponentBits;
	std::memcpy( &exponentBits, &alpha, sizeof( exponentBits ) );
	exponentBits &= 255u << 23;
	float nodeSize;
	std::memcpy( &nodeSize, &exponentBits, sizeof( nodeSize ) );
	// transform back into global space
	nodeSize *= globalSize;
	return nodeSize > 0 ? nodeSize : extent * 0.5f;
}

template <typename Float, typename Index> Index BVH<Float, Index>::Presplit()
{
	// Based on Section 5 of "Fast Parallel Construction of High-Quality Bounding 
	// Volume Hierarchies", Karras and Aila, 2013, and BoyBaykiller's implementation,
	// which is in turn based on code by MadMann91.
	Index fragCount = triCount;
	const Float factor = settings.presplitFactor;
	const Index splitBudget = (Index)(triCount * settings.presplitFactor);
	Float* prio = (Float*)AlignedAlloc( triCount * sizeof( Float ) );
	int* splits = (int*)AlignedAlloc( (triCount + splitBudget) * sizeof( int ) );
	double summedPrio = 0;
	for (Index i = 0; i < triCount; i++)
		prio[i] = SplitPriority( fragment[i] ), summedPrio += prio[i];
	// hand out the budget in a single pass.
	if (summedPrio > 0)
	{
		const Float scale = (Float)((double)triCount * (double)factor / summedPrio);
		for (Index i = 0, remaining = splitBudget; i < triCount; i++)
		{
			const Float share = prio[i] * scale;
			// '> 0' rather than '>= 0' so that a NaN share yields no extra splits.
			Index extra = share > 0 ? (Index)tinybvh_min( share, (Float)remaining ) : 0;
			if (extra > remaining) extra = remaining; // (Float)remaining may round up
			splits[i] = 1 + (int)extra, remaining -= extra;
		}
	}
	else for (Index i = 0; i < triCount; i++) splits[i] = 1; // fully degenerate input
	// do actual splitting.
	const BVHNode& root = bvhNode[0];
	const Vec3 rootExtent = root.aabbMax - root.aabbMin;
	ALIGNED( 64 ) Fragment part1, part2; // keep all clipping in a single cacheline.
	for (Index i = 0; i < fragCount; ) if (splits[i] == 1) i++; else
	{
		const Fragment& f = fragment[i];
		const Vec3 extent = f.bmax - f.bmin;
		const uint32_t splitAxis = tinybvh_maxdim( extent );
		if (!(extent[splitAxis] > 0)) { splits[i] = 1; continue; }
		Float nodeSize = GetNodeSize( extent[splitAxis], rootExtent[splitAxis] );
		if (nodeSize > extent[splitAxis] * 0.9999f) nodeSize *= 0.5f;
		// snap mid position to nearest split plane
		const Float midPos = (f.bmin[splitAxis] + f.bmax[splitAxis]) * 0.5f;
		const Float index = roundf( (midPos - root.aabbMin[splitAxis]) / nodeSize );
		Float splitPos = root.aabbMin[splitAxis] + index * nodeSize;
		if (!(splitPos > f.bmin[splitAxis] && splitPos < f.bmax[splitAxis])) splitPos = midPos;
		if (!SplitFrag( fragment[i], part1, part2, splitAxis, splitPos )) { splits[i] = 1; continue; }
		fragment[i] = part1, fragment[fragCount] = part2;
		// distribute available splits over part1 and part2
		const int toDivide = splits[i];
		const Vec3 p1Extent = part1.bmax - part1.bmin, p2Extent = part2.bmax - part2.bmin;
		const Float p1Size = p1Extent[tinybvh_maxdim( p1Extent )];
		const Float p2Size = p2Extent[tinybvh_maxdim( p2Extent )];
		const Float sumSize = p1Size + p2Size;
		// sumSize is zero only if both halves are points; split the budget evenly.
		const int p1Count = sumSize > 0 ? (int)((Float)toDivide * p1Size / sumSize) : (toDivide / 2);
		splits[i] = tinybvh_clamp( p1Count, 1, toDivide - 1 );
		splits[fragCount] = toDivide - splits[i];
		primIdx[fragCount] = fragCount;
		fragCount++;
	}
	// cleanup
	AlignedFree( prio );
	AlignedFree( splits );
	// all done.
	return fragCount;
}

template <typename Float, typename Index> void BVH<Float, Index>::PresplitPostPass()
{
	// see if we have any leafs that reference the same primitive multiple times
	for (Index i = 2; i < usedNodes; i++) if (bvhNode[i].isLeaf())
	{
		Index first = bvhNode[i].leftFirst, & count = bvhNode[i].triCount;
		for (Index j = 0; j < count; j++) for (Index p0 = primIdx[first + j], k = j + 1; k < count; k++)
		{
			const Index p1 = primIdx[first + k];
			if (p0 == p1) primIdx[first + k] = primIdx[first + --count];
		}
	}
}

template <typename Float, typename Index> void BVH<Float, Index>::Optimize( const uint32_t iterations, bool extreme, bool stochastic )
{
	BVH_Verbose* verbose = new BVH_Verbose();
	verbose->ConvertFrom( *this );
	verbose->Optimize( iterations, extreme, stochastic );
	verbose->SortIndices();
	ConvertFrom( *verbose );
	delete verbose; // safe: ~BVH_Verbose only releases its own node pool.
}

// Refitting: For animated meshes, where the topology remains intact. This
// includes trees waving in the wind, or subsequent frames for skinned
// animations. Repeated refitting tends to lead to deteriorated BVHs and
// slower ray tracing. Rebuild when this happens.
template <typename Float, typename Index> void BVH<Float, Index>::Refit( const Index /* unused */ )
{
	BVH_FATAL_ERROR_IF( !refittable, "BVH::Refit( .. ), refitting an SBVH or pre-splitted BVH." );
	BVH_FATAL_ERROR_IF( bvhNode == 0, "BVH::Refit( .. ), bvhNode == 0." );
	BVH_FATAL_ERROR_IF( may_have_holes, "BVH::Refit( .. ), bvh may have holes." );
	BVH_FATAL_ERROR_IF( isTLAS(), "BVH::Refit( .. ), do not refit a TLAS, use Build(..)." );
	BVH_FATAL_ERROR_IF( !verts, "BVH::Refit( .. ), bvh has no vertex data." );
	for (int64_t i = (int64_t)usedNodes - 1; i >= 0; i--) if (i != 1)
	{
		BVHNode& node = bvhNode[i];
		if (node.isLeaf()) // leaf: adjust to current triangle vertex positions
		{
			Vertex bmin( bvh_far<Float> ), bmax( -bvh_far<Float> );
			if (vertIdx) for (Index first = node.leftFirst, j = 0; j < node.triCount; j++)
			{
				const Index vidx = primIdx[first + j] * 3;
				const uint32_t i0 = vertIdx[vidx], i1 = vertIdx[vidx + 1], i2 = vertIdx[vidx + 2];
				const Vertex v0 = verts[i0], v1 = verts[i1], v2 = verts[i2];
				const Vertex t1 = tinybvh_min( v0, bmin ), t2 = tinybvh_max( v0, bmax );
				const Vertex t3 = tinybvh_min( v1, v2 ), t4 = tinybvh_max( v1, v2 );
				bmin = tinybvh_min( t1, t3 ), bmax = tinybvh_max( t2, t4 );
			}
			else for (Index first = node.leftFirst, j = 0; j < node.triCount; j++)
			{
				const Index vidx = primIdx[first + j] * 3;
				const Vertex v0 = verts[vidx], v1 = verts[vidx + 1], v2 = verts[vidx + 2];
				const Vertex t1 = tinybvh_min( v0, bmin ), t2 = tinybvh_max( v0, bmax );
				const Vertex t3 = tinybvh_min( v1, v2 ), t4 = tinybvh_max( v1, v2 );
				bmin = tinybvh_min( t1, t3 ), bmax = tinybvh_max( t2, t4 );
			}
			node.aabbMin = bmin, node.aabbMax = bmax;
			continue;
		}
		// interior node: adjust to child bounds
		const BVHNode& left = bvhNode[node.leftFirst], & right = bvhNode[node.leftFirst + 1];
		node.aabbMin = tinybvh_min( left.aabbMin, right.aabbMin );
		node.aabbMax = tinybvh_max( left.aabbMax, right.aabbMax );
	}
	aabbMin = bvhNode[0].aabbMin, aabbMax = bvhNode[0].aabbMax;
}

// CombineLeafs: Collapse subtrees if the summed leaf prim count does not
// exceed the specified number. For BVH8_CPU construction.
template <typename Float, typename Index> Index BVH<Float, Index>::CombineLeafs( const Index primCount, Index& firstIdx, Index nodeIdx )
{
	BVHNode& node = bvhNode[nodeIdx];
	if (node.isLeaf()) { firstIdx = node.leftFirst; return node.triCount; }
	Index firstLeft = 0, leftCount = CombineLeafs( primCount, firstLeft, node.leftFirst );
	Index firstRight = 0, rightCount = CombineLeafs( primCount, firstRight, node.leftFirst + 1 );
	firstIdx = tinybvh_min( firstLeft, firstRight );
	if (leftCount + rightCount <= primCount)
		node.triCount = leftCount + rightCount, node.leftFirst = firstIdx;
	return leftCount + rightCount;
}

// CombineLeafs: Combine leaf nodes if this improves tree SAH cost. For HPLOC postprocessing.
template <typename Float, typename Index> void BVH<Float, Index>::CombineLeafs( const Index nodeIdx )
{
	BVHNode& node = bvhNode[nodeIdx];
	if (node.isLeaf()) return;
	BVHNode& left = bvhNode[node.leftFirst];
	BVHNode& right = bvhNode[node.leftFirst + 1];
	if (left.isLeaf() && right.isLeaf())
	{
		const Index combinedCount = left.triCount + right.triCount;
		Float rAnode = 1.0f / tinybvh_halfarea( node.aabbMax - node.aabbMin );
		Float Cnode = c_int * combinedCount;
		Float Cleft = c_int * left.triCount * tinybvh_halfarea( left.aabbMax - left.aabbMin ) * rAnode;
		Float Cright = c_int * right.triCount * tinybvh_halfarea( right.aabbMax - right.aabbMin ) * rAnode;
		Float Csplit = Cleft + Cright + c_trav;
		if (Cnode < Csplit) if (right.leftFirst == (left.leftFirst + left.triCount))
			node.leftFirst = left.leftFirst, node.triCount = combinedCount;
		return;
	}
	CombineLeafs( node.leftFirst );
	CombineLeafs( node.leftFirst + 1 );
}

template <typename Float, typename Index> bool BVH<Float, Index>::IntersectSphere( const Vec3& pos, const Float r ) const
{
	const Vec3 bmin = pos - Vec3( r ), bmax = pos + Vec3( r );
	BVHNode* node = &bvhNode[0], * stack[TINYBVH_STACK_SIZE];
	Index stackPtr = 0;
	const Float r2 = r * r;
	while (1)
	{
		if (node->isLeaf())
		{
			// check if the leaf aabb overlaps the sphere: https://gamedev.stackexchange.com/a/156877
			Float dist2 = 0;
			if (pos.x < node->aabbMin.x) dist2 += (node->aabbMin.x - pos.x) * (node->aabbMin.x - pos.x);
			if (pos.x > node->aabbMax.x) dist2 += (pos.x - node->aabbMax.x) * (pos.x - node->aabbMax.x);
			if (pos.y < node->aabbMin.y) dist2 += (node->aabbMin.y - pos.y) * (node->aabbMin.y - pos.y);
			if (pos.y > node->aabbMax.y) dist2 += (pos.y - node->aabbMax.y) * (pos.y - node->aabbMax.y);
			if (pos.z < node->aabbMin.z) dist2 += (node->aabbMin.z - pos.z) * (node->aabbMin.z - pos.z);
			if (pos.z > node->aabbMax.z) dist2 += (pos.z - node->aabbMax.z) * (pos.z - node->aabbMax.z);
			if (dist2 <= r2)
			{
				// tri/sphere test: https://gist.github.com/yomotsu/d845f21e2e1eb49f647f#file-gistfile1-js-L223
				for (Index i = 0; i < node->triCount; i++)
				{
					Index idx = primIdx[node->leftFirst + i];
					Vec3 a, b, c;
					if (!vertIdx) idx *= 3, a = verts[idx], b = verts[idx + 1], c = verts[idx + 2]; else
					{
						const uint32_t i0 = vertIdx[idx * 3], i1 = vertIdx[idx * 3 + 1], i2 = vertIdx[idx * 3 + 2];
						a = verts[i0], b = verts[i1], c = verts[i2];
					}
					const Vec3 A = a - pos, B = b - pos, C = c - pos;
					const Float rr = r * r;
					const Vec3 V = tinybvh_cross( B - A, C - A );
					const Float d = tinybvh_dot( A, V ), e = tinybvh_dot( V, V );
					if (d * d > rr * e) continue;
					const Float aa = tinybvh_dot( A, A ), ab = tinybvh_dot( A, B ), ac = tinybvh_dot( A, C );
					const Float bb = tinybvh_dot( B, B ), bc = tinybvh_dot( B, C ), cc = tinybvh_dot( C, C );
					if ((aa > rr && ab > aa && ac > aa) || (bb > rr && ab > bb && bc > bb) ||
						(cc > rr && ac > cc && bc > cc)) continue;
					const Vec3 AB = B - A, BC = C - B, CA = A - C;
					const Float d1 = ab - aa, d2 = bc - bb, d3 = ac - cc;
					const Float e1 = tinybvh_dot( AB, AB ), e2 = tinybvh_dot( BC, BC ), e3 = tinybvh_dot( CA, CA );
					const Vec3 Q1 = A * e1 - AB * d1, Q2 = B * e2 - BC * d2, Q3 = C * e3 - CA * d3;
					const Vec3 QC = C * e1 - Q1, QA = A * e2 - Q2, QB = B * e3 - Q3;
					if ((tinybvh_dot( Q1, Q1 ) > rr * e1 * e1 && tinybvh_dot( Q1, QC ) >= 0) ||
						(tinybvh_dot( Q2, Q2 ) > rr * e2 * e2 && tinybvh_dot( Q2, QA ) >= 0) ||
						(tinybvh_dot( Q3, Q3 ) > rr * e3 * e3 && tinybvh_dot( Q3, QB ) >= 0)) continue;
					// const Float dist = sqrtf( d * d / e ) - r; // we're not using this.
					return true;
				}
			}
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		BVHNode* child1 = &bvhNode[node->leftFirst], * child2 = &bvhNode[node->leftFirst + 1];
		bool hit1 = child1->Intersect( bmin, bmax ), hit2 = child2->Intersect( bmin, bmax );
		if (hit1 && hit2) stack[stackPtr++] = child2, node = child1;
		else if (hit1) node = child1; else if (hit2) node = child2;
		else { if (stackPtr == 0) break; node = stack[--stackPtr]; }
	}
	return false;
}

#define SLAB_TEST_TWO_NODES \
	Float tx1a = tinybvh_fma( posX ? child1->aabbMin.x : child1->aabbMax.x, ray.rD.x, -rox ); \
	Float ty1a = tinybvh_fma( posY ? child1->aabbMin.y : child1->aabbMax.y, ray.rD.y, -roy ); \
	Float tz1a = tinybvh_fma( posZ ? child1->aabbMin.z : child1->aabbMax.z, ray.rD.z, -roz ); \
	Float tx1b = tinybvh_fma( posX ? child2->aabbMin.x : child2->aabbMax.x, ray.rD.x, -rox ); \
	Float ty1b = tinybvh_fma( posY ? child2->aabbMin.y : child2->aabbMax.y, ray.rD.y, -roy ); \
	Float tz1b = tinybvh_fma( posZ ? child2->aabbMin.z : child2->aabbMax.z, ray.rD.z, -roz ); \
	Float tx2a = tinybvh_fma( posX ? child1->aabbMax.x : child1->aabbMin.x, ray.rD.x, -rox ); \
	Float ty2a = tinybvh_fma( posY ? child1->aabbMax.y : child1->aabbMin.y, ray.rD.y, -roy ); \
	Float tz2a = tinybvh_fma( posZ ? child1->aabbMax.z : child1->aabbMin.z, ray.rD.z, -roz ); \
	Float tx2b = tinybvh_fma( posX ? child2->aabbMax.x : child2->aabbMin.x, ray.rD.x, -rox ); \
	Float ty2b = tinybvh_fma( posY ? child2->aabbMax.y : child2->aabbMin.y, ray.rD.y, -roy ); \
	Float tz2b = tinybvh_fma( posZ ? child2->aabbMax.z : child2->aabbMin.z, ray.rD.z, -roz ); \
	Float tmina = tinybvh_max( tinybvh_max( tx1a, ty1a ), tinybvh_max( tz1a, Float( 0 ) ) ); \
	Float tminb = tinybvh_max( tinybvh_max( tx1b, ty1b ), tinybvh_max( tz1b, Float( 0 ) ) ); \
	Float tmaxa = tinybvh_min( tinybvh_min( tx2a, ty2a ), tinybvh_min( tz2a, ray.hit.t ) ); \
	Float tmaxb = tinybvh_min( tinybvh_min( tx2b, ty2b ), tinybvh_min( tz2b, ray.hit.t ) ); \
	if (tmaxa >= tmina) dist1 = tmina; \
	if (tmaxb >= tminb) dist2 = tminb;

// Calls the kernel specialized for the octant of the ray direction.
#define OCTANT_DISPATCH( kernel, ray ) \
	{ \
		const bool posX = ray.rD.x >= 0, posY = ray.rD.y >= 0, posZ = ray.rD.z >= 0; \
		if (posX) \
		{ \
			if (posY) return posZ ? kernel<true, true, true>( ray ) : kernel<true, true, false>( ray ); \
			return posZ ? kernel<true, false, true>( ray ) : kernel<true, false, false>( ray ); \
		} \
		if (posY) return posZ ? kernel<false, true, true>( ray ) : kernel<false, true, false>( ray ); \
		return posZ ? kernel<false, false, true>( ray ) : kernel<false, false, false>( ray ); \
	}

template <typename Float, typename Index> int32_t BVH<Float, Index>::Intersect( Ray& ray ) const
{
	VALIDATE_RAY( ray );
	if (isTLAS()) OCTANT_DISPATCH( IntersectTLASOctant, ray )
	else OCTANT_DISPATCH( IntersectOctant, ray )
}

template <typename Float, typename Index> template <bool posX, bool posY, bool posZ> int32_t BVH<Float, Index>::IntersectOctant( Ray& ray ) const
{
	BVHNode* node = &bvhNode[0], * stack[TINYBVH_STACK_SIZE];
	Index stackPtr = 0;
	Float cost = 0;
	const Float rox = ray.O.x * ray.rD.x;
	const Float roy = ray.O.y * ray.rD.y;
	const Float roz = ray.O.z * ray.rD.z;
	while (1)
	{
		cost += c_trav;
		if (node->isLeaf()) ISUNLIKELY
		{
			// Performance note: if indexed primitives (ENABLE_INDEXED_GEOMETRY) and custom
			// geometry (ENABLE_CUSTOM_GEOMETRY) are both disabled, this leaf code reduces
			// to a regular loop over triangles. Otherwise, the extra flexibility comes at
			// a small performance cost.
			if (indexedEnabled && vertIdx != 0) for (Index i = 0; i < node->triCount; i++, cost += c_int)
			{
				const Index pi = primIdx[node->leftFirst + i];
				const uint32_t i0 = vertIdx[pi * 3], i1 = vertIdx[pi * 3 + 1], i2 = vertIdx[pi * 3 + 2];
				IntersectTri( ray, pi, verts, i0, i1, i2 );
			}
			else if (customEnabled && customIntersect != 0) for (Index i = 0; i < node->triCount; i++, cost += c_int)
			{
				if ((*customIntersect)(ray, primIdx[node->leftFirst + i], customUserdata)) ray.SetHitPrim( ray.hit.prim );
			}
			else for (Index i = 0; i < node->triCount; i++, cost += c_int)
			{
				const Index pi = primIdx[node->leftFirst + i];
				IntersectTri( ray, pi, verts, pi * 3, pi * 3 + 1, pi * 3 + 2 );
			}
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		BVHNode* child1 = &bvhNode[node->leftFirst], * child2 = &bvhNode[node->leftFirst + 1];
		Float dist1 = bvh_far<Float>, dist2 = bvh_far<Float>;
		SLAB_TEST_TWO_NODES;
		if (dist1 > dist2) { tinybvh_swap( dist1, dist2 ); tinybvh_swap( child1, child2 ); }
		if (dist1 == bvh_far<Float> /* missed both child nodes */)
		{
			if (stackPtr == 0) break; else node = stack[--stackPtr];
		}
		else /* hit at least one node */
		{
			node = child1; /* continue with the nearest */
			if (dist2 != bvh_far<Float>) stack[stackPtr++] = child2; /* push far child */
		}
	}
	return (int32_t)cost; // cast to not break interface.
}

template <typename Float, typename Index> template <bool posX, bool posY, bool posZ> int32_t BVH<Float, Index>::IntersectTLASOctant( Ray& ray ) const
{
	BVHNode* node = &bvhNode[0], * stack[TINYBVH_STACK_SIZE];
	Index stackPtr = 0;
	Float cost = 0;
	const Float rox = ray.O.x * ray.rD.x;
	const Float roy = ray.O.y * ray.rD.y;
	const Float roz = ray.O.z * ray.rD.z;
	while (1)
	{
		cost += c_trav;
		if (node->isLeaf()) ISUNLIKELY
		{
			Ray tmpRay;
			for (Index i = 0; i < node->triCount; i++)
			{
				// BLAS traversal
				const Index instIdx = primIdx[node->leftFirst + i];
				const BLASInstance& inst = instList[instIdx];
				// Check if the ray should intersect this BLAS Instance, otherwise skip it
				if (!(inst.mask & ray.mask)) continue;
				const BVHBase<Float, Index>* blas = blasList[inst.blasIdx];
				// 1. Transform ray with the inverse of the instance transform
				tmpRay.O = tinybvh_transform_point( ray.O, inst.invTransform );
				tmpRay.D = tinybvh_transform_vector( ray.D, inst.invTransform );
				tmpRay.instIdx = instIdx << bvh_inst_shift<Index>;
				tmpRay.mask = ray.mask;
				tmpRay.hit = ray.hit;
				tmpRay.rD = tinybvh_rcp( tmpRay.D );
				// 2. Traverse BLAS with the transformed ray. When all BLASses are of the same
				// layout this reduces to nearly zero cost for a small set of predictable branches.
				if (blas->layout == LAYOUT_BVH) cost += ((BVH*)blas)->Intersect( tmpRay );
			#ifdef ENABLE_VOXEL_SUPPORT
				else if (blas->layout == LAYOUT_VOXELSET) cost += ((VoxelSet*)blas)->Intersect( tmpRay );
			#endif
				else if constexpr (bvh_traits<Float>::wide_layouts)
				{
					if (blas->layout == LAYOUT_BVH4_CPU) cost += ((BVH4_CPU<Float, Index>*)blas)->Intersect( tmpRay );
					else if (blas->layout == LAYOUT_BVH8_AVX2) cost += ((BVH8_CPU<Float, Index>*)blas)->Intersect( tmpRay );
				#ifdef ENABLE_BVH_SOA
					else if (blas->layout == LAYOUT_BVH_SOA) cost += ((BVH_SoA<Float, Index>*)blas)->Intersect( tmpRay );
				#endif
					else assert( !"unsupported BLAS layout" );
				}
				else assert( !"unsupported BLAS layout" );
				// 3. Restore ray
				ray.hit = tmpRay.hit;
			}
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		BVHNode* child1 = &bvhNode[node->leftFirst], * child2 = &bvhNode[node->leftFirst + 1];
		Float dist1 = bvh_far<Float>, dist2 = bvh_far<Float>;
		SLAB_TEST_TWO_NODES;
		if (dist1 > dist2) { tinybvh_swap( dist1, dist2 ); tinybvh_swap( child1, child2 ); }
		if (dist1 == bvh_far<Float> /* missed both child nodes */)
		{
			if (stackPtr == 0) break; else node = stack[--stackPtr];
		}
		else /* hit at least one node */
		{
			node = child1; /* continue with the nearest */
			if (dist2 != bvh_far<Float>) stack[stackPtr++] = child2; /* push far child */
		}
	}
	return (int32_t)cost;
}

template <typename Float, typename Index> bool BVH<Float, Index>::IsOccluded( const Ray& ray ) const
{
	VALIDATE_RAY( ray );
	if (isTLAS()) OCTANT_DISPATCH( IsOccludedTLASOctant, ray )
	else OCTANT_DISPATCH( IsOccludedOctant, ray )
}

template <typename Float, typename Index> template <bool posX, bool posY, bool posZ> bool BVH<Float, Index>::IsOccludedOctant( const Ray& ray ) const
{
	BVHNode* node = &bvhNode[0], * stack[TINYBVH_STACK_SIZE];
	Index stackPtr = 0;
	const Float rox = ray.O.x * ray.rD.x;
	const Float roy = ray.O.y * ray.rD.y;
	const Float roz = ray.O.z * ray.rD.z;
	while (1)
	{
		if (node->isLeaf()) ISUNLIKELY
		{
			if (indexedEnabled && vertIdx != 0) for (Index i = 0; i < node->triCount; i++)
			{
				const Index pi = primIdx[node->leftFirst + i], vi0 = pi * 3;
				const uint32_t i0 = vertIdx[vi0], i1 = vertIdx[vi0 + 1], i2 = vertIdx[vi0 + 2];
				if (TriOccludes( ray, verts, pi, i0, i1, i2 )) return true;
			}
			else if (customEnabled && customIsOccluded != 0)
			{
				for (Index i = 0; i < node->triCount; i++)
					if ((*customIsOccluded)(ray, primIdx[node->leftFirst + i], customUserdata)) return true;
			}
			else for (Index i = 0; i < node->triCount; i++)
			{
				const Index pi = primIdx[node->leftFirst + i], vi0 = pi * 3;
				if (TriOccludes( ray, verts, pi, vi0, vi0 + 1, vi0 + 2 )) return true;
			}
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		BVHNode* child1 = &bvhNode[node->leftFirst];
		BVHNode* child2 = &bvhNode[node->leftFirst + 1];
		Float dist1 = bvh_far<Float>, dist2 = bvh_far<Float>;
		SLAB_TEST_TWO_NODES;
		if (dist1 > dist2) { tinybvh_swap( dist1, dist2 ); tinybvh_swap( child1, child2 ); }
		if (dist1 == bvh_far<Float> /* missed both child nodes */)
		{
			if (stackPtr == 0) break; else node = stack[--stackPtr];
		}
		else /* hit at least one node */
		{
			node = child1; /* continue with the nearest */
			if (dist2 != bvh_far<Float>) stack[stackPtr++] = child2; /* push far child */
		}
	}
	return false;
}

template <typename Float, typename Index> template <bool posX, bool posY, bool posZ> bool BVH<Float, Index>::IsOccludedTLASOctant( const Ray& ray ) const
{
	BVHNode* node = &bvhNode[0], * stack[TINYBVH_STACK_SIZE];
	Index stackPtr = 0;
	Ray tmpRay;
	const Float rox = ray.O.x * ray.rD.x;
	const Float roy = ray.O.y * ray.rD.y;
	const Float roz = ray.O.z * ray.rD.z;
	while (1)
	{
		if (node->isLeaf()) ISUNLIKELY
		{
			for (Index i = 0; i < node->triCount; i++)
			{
				// BLAS traversal
				const Index instIdx = primIdx[node->leftFirst + i];
				BLASInstance& inst = instList[instIdx];
				// Check if the ray should intersect this BLAS Instance, otherwise skip it
				if (!(inst.mask & ray.mask)) continue;
				const BVHBase<Float, Index>* blas = blasList[inst.blasIdx];
				// 1. Transform ray with the inverse of the instance transform
				tmpRay.O = tinybvh_transform_point( ray.O, inst.invTransform );
				tmpRay.D = tinybvh_transform_vector( ray.D, inst.invTransform );
				tmpRay.instIdx = instIdx << bvh_inst_shift<Index>;
				tmpRay.mask = ray.mask;
				tmpRay.hit = ray.hit;
				tmpRay.rD = tinybvh_rcp( tmpRay.D );
				// 2. Traverse BLAS with the transformed ray
				bool occluded = false;
				if (blas->layout == LAYOUT_BVH) occluded = ((BVH*)blas)->IsOccluded( tmpRay );
			#ifdef ENABLE_VOXEL_SUPPORT
				else if (blas->layout == LAYOUT_VOXELSET) occluded = ((VoxelSet*)blas)->IsOccluded( tmpRay );
			#endif
				else if constexpr (bvh_traits<Float>::wide_layouts)
				{
					if (blas->layout == LAYOUT_BVH4_CPU) occluded = ((BVH4_CPU<Float, Index>*)blas)->IsOccluded( tmpRay );
					else if (blas->layout == LAYOUT_BVH8_AVX2) occluded = ((BVH8_CPU<Float, Index>*)blas)->IsOccluded( tmpRay );
				#ifdef ENABLE_BVH_SOA
					else if (blas->layout == LAYOUT_BVH_SOA) occluded = ((BVH_SoA<Float, Index>*)blas)->IsOccluded( tmpRay );
				#endif
					else assert( !"unsupported BLAS layout" );
				}
				else assert( !"unsupported BLAS layout" );
				if (occluded) return true;
			}
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		BVHNode* child1 = &bvhNode[node->leftFirst], * child2 = &bvhNode[node->leftFirst + 1];
		Float dist1 = bvh_far<Float>, dist2 = bvh_far<Float>;
		SLAB_TEST_TWO_NODES;
		if (dist1 > dist2) { tinybvh_swap( dist1, dist2 ); tinybvh_swap( child1, child2 ); }
		if (dist1 == bvh_far<Float> /* missed both child nodes */)
		{
			if (stackPtr == 0) break; else node = stack[--stackPtr];
		}
		else /* hit at least one node */
		{
			node = child1; /* continue with the nearest */
			if (dist2 != bvh_far<Float>) stack[stackPtr++] = child2; /* push far child */
		}
	}
	return false;
}

// Intersect a WALD_32BYTE BVH with a ray packet.
// The 256 rays travel together to better utilize the caches and to amortize the cost
// of memory transfers over the rays in the bundle.
// Note that this basic implementation assumes a specific layout of the rays. Provided
// as 'proof of concept', should not be used in production code.
// Based on Large Ray Packets for Real-time Whitted Ray Tracing, Overbeck et al., 2008,
// extended with sorted traversal and reduced stack traffic.
template <typename Float, typename Index> void BVH<Float, Index>::Intersect256Rays( Ray* packet ) const
{
	// convenience macro
#define CALC_TMIN_TMAX_WITH_SLABTEST_ON_RAY( r ) const Vec3 rD = packet[r].rD, t1 = o1 * rD, t2 = o2 * rD; \
	const Float tmin = tinybvh_max( tinybvh_max( tinybvh_min( t1.x, t2.x ), tinybvh_min( t1.y, t2.y ) ), tinybvh_min( t1.z, t2.z ) ); \
	const Float tmax = tinybvh_min( tinybvh_min( tinybvh_max( t1.x, t2.x ), tinybvh_max( t1.y, t2.y ) ), tinybvh_max( t1.z, t2.z ) );
	// Corner rays are: 0, 51, 204 and 255
	// Construct the bounding planes, with normals pointing outwards
	const Vec3 O = packet[0].O; // same for all rays in this case
	const Vec3 p0 = packet[0].O + packet[0].D; // top-left
	const Vec3 p1 = packet[51].O + packet[51].D; // top-right
	const Vec3 p2 = packet[204].O + packet[204].D; // bottom-left
	const Vec3 p3 = packet[255].O + packet[255].D; // bottom-right
	const Vec3 plane0 = tinybvh_normalize( tinybvh_cross( p0 - O, p0 - p2 ) ); // left plane
	const Vec3 plane1 = tinybvh_normalize( tinybvh_cross( p3 - O, p3 - p1 ) ); // right plane
	const Vec3 plane2 = tinybvh_normalize( tinybvh_cross( p1 - O, p1 - p0 ) ); // top plane
	const Vec3 plane3 = tinybvh_normalize( tinybvh_cross( p2 - O, p2 - p3 ) ); // bottom plane
	const Float d0 = tinybvh_dot( O, plane0 ), d1 = tinybvh_dot( O, plane1 );
	const Float d2 = tinybvh_dot( O, plane2 ), d3 = tinybvh_dot( O, plane3 );
	// The box corner with the smallest signed distance to a plane. The box lies outside if this corner does.
	auto corner = []( const BVHNode* n, const Vec3& plane )
	{
		return Vec3( plane.x < 0 ? n->aabbMax.x : n->aabbMin.x, plane.y < 0 ? n->aabbMax.y : n->aabbMin.y, plane.z < 0 ? n->aabbMax.z : n->aabbMin.z );
	};
	// Traverse the tree with the packet
	int32_t first = 0, last = 255; // first and last active ray in the packet
	const BVHNode* node = &bvhNode[0];
	ALIGNED( 64 ) Index stack[2 * TINYBVH_STACK_SIZE], stackPtr = 0;
	while (1)
	{
		if (node->isLeaf())
		{
			// handle leaf node
			for (Index j = 0; j < node->triCount; j++)
			{
				const Index idx = primIdx[node->leftFirst + j], vid = idx * 3;
				const Vec3 e1 = verts[vid + 1] - verts[vid], e2 = verts[vid + 2] - verts[vid];
				const Vec3 s = O - Vec3( verts[vid] );
				for (int32_t i = first; i <= last; i++)
				{
					Ray& ray = packet[i];
					const Vec3 h = tinybvh_cross( ray.D, e2 );
					const Float a = tinybvh_dot( e1, h );
					if (a == 0) continue; // ray parallel to triangle
					const Float f = 1 / a, u = f * tinybvh_dot( s, h );
					const Vec3 q = tinybvh_cross( s, e1 );
					const Float v = f * tinybvh_dot( ray.D, q );
					if (!(u >= 0 && v >= 0 && u + v <= 1)) continue;
					const Float t = f * tinybvh_dot( e2, q );
					if (!(t > 0 && t < ray.hit.t)) continue;
					ray.hit.t = t, ray.hit.u = u, ray.hit.v = v;
					ray.SetHitPrim( idx );
				}
			}
			if (stackPtr == 0) break; else // pop
				last = (int32_t)stack[--stackPtr], node = bvhNode + stack[--stackPtr],
				first = last >> 8, last &= 255;
		}
		else
		{
			// fetch pointers to child nodes
			const BVHNode* left = bvhNode + node->leftFirst;
			const BVHNode* right = bvhNode + node->leftFirst + 1;
			bool visitLeft = true, visitRight = true;
			int32_t leftFirst = first, leftLast = last, rightFirst = first, rightLast = last;
			Float distLeft, distRight;
			{
				// see if we want to intersect the left child
				const Vec3 o1( left->aabbMin.x - O.x, left->aabbMin.y - O.y, left->aabbMin.z - O.z );
				const Vec3 o2( left->aabbMax.x - O.x, left->aabbMax.y - O.y, left->aabbMax.z - O.z );
				// 1. Early-in test: if first ray hits the node, the packet visits the node
				bool earlyHit;
				{
					CALC_TMIN_TMAX_WITH_SLABTEST_ON_RAY( first );
					earlyHit = (tmax >= tmin && tmin < packet[first].hit.t && tmax >= 0);
					distLeft = tmin;
				}
				if (!earlyHit) // 2. Early-out test: if the node aabb is outside the four planes, we skip the node
				{
					if (tinybvh_dot( corner( left, plane0 ), plane0 ) > d0 || tinybvh_dot( corner( left, plane1 ), plane1 ) > d1 ||
						tinybvh_dot( corner( left, plane2 ), plane2 ) > d2 || tinybvh_dot( corner( left, plane3 ), plane3 ) > d3)
						visitLeft = false;
					else // 3. Last resort: update first and last, stay in node if first > last
					{
						for (; leftFirst <= leftLast; leftFirst++)
						{
							CALC_TMIN_TMAX_WITH_SLABTEST_ON_RAY( leftFirst );
							if (tmax >= tmin && tmin < packet[leftFirst].hit.t && tmax >= 0) { distLeft = tmin; break; }
						}
						for (; leftLast >= leftFirst; leftLast--)
						{
							CALC_TMIN_TMAX_WITH_SLABTEST_ON_RAY( leftLast );
							if (tmax >= tmin && tmin < packet[leftLast].hit.t && tmax >= 0) break;
						}
						visitLeft = leftLast >= leftFirst;
					}
				}
			}
			{
				// see if we want to intersect the right child
				const Vec3 o1( right->aabbMin.x - O.x, right->aabbMin.y - O.y, right->aabbMin.z - O.z );
				const Vec3 o2( right->aabbMax.x - O.x, right->aabbMax.y - O.y, right->aabbMax.z - O.z );
				// 1. Early-in test: if first ray hits the node, the packet visits the node
				bool earlyHit;
				{
					CALC_TMIN_TMAX_WITH_SLABTEST_ON_RAY( first );
					earlyHit = (tmax >= tmin && tmin < packet[first].hit.t && tmax >= 0);
					distRight = tmin;
				}
				if (!earlyHit) // 2. Early-out test: if the node aabb is outside the four planes, we skip the node
				{
					if (tinybvh_dot( corner( right, plane0 ), plane0 ) > d0 || tinybvh_dot( corner( right, plane1 ), plane1 ) > d1 ||
						tinybvh_dot( corner( right, plane2 ), plane2 ) > d2 || tinybvh_dot( corner( right, plane3 ), plane3 ) > d3)
						visitRight = false;
					else // 3. Last resort: update first and last, stay in node if first > last
					{
						for (; rightFirst <= rightLast; rightFirst++)
						{
							CALC_TMIN_TMAX_WITH_SLABTEST_ON_RAY( rightFirst );
							if (tmax >= tmin && tmin < packet[rightFirst].hit.t && tmax >= 0) { distRight = tmin; break; }
						}
						for (; rightLast >= first; rightLast--)
						{
							CALC_TMIN_TMAX_WITH_SLABTEST_ON_RAY( rightLast );
							if (tmax >= tmin && tmin < packet[rightLast].hit.t && tmax >= 0) break;
						}
						visitRight = rightLast >= rightFirst;
					}
				}
			}
			// process intersection result
			if (visitLeft && visitRight)
			{
				if (distLeft < distRight) // push right, continue with left
				{
					stack[stackPtr++] = node->leftFirst + 1;
					stack[stackPtr++] = (rightFirst << 8) + rightLast;
					node = left, first = leftFirst, last = leftLast;
				}
				else // push left, continue with right
				{
					stack[stackPtr++] = node->leftFirst;
					stack[stackPtr++] = (leftFirst << 8) + leftLast;
					node = right, first = rightFirst, last = rightLast;
				}
			}
			else if (visitLeft) // continue with left
				node = left, first = leftFirst, last = leftLast;
			else if (visitRight) // continue with right
				node = right, first = rightFirst, last = rightLast;
			else if (stackPtr == 0) break; else // pop
				last = (int32_t)stack[--stackPtr], node = bvhNode + stack[--stackPtr],
				first = last >> 8, last &= 255;
		}
	}
}

template <typename Float, typename Index> int32_t BVH<Float, Index>::NodeCount() const
{
	// Determine the number of nodes in the tree. Typically the result should
	// be usedNodes - 1 (second node is always unused), but some builders may
	// have unused nodes besides node 1. TODO: Support more layouts.
	Index retVal = 0, nodeIdx = 0, stack[64], stackPtr = 0;
	while (1)
	{
		const BVHNode& n = bvhNode[nodeIdx];
		retVal++;
		if (n.isLeaf()) { if (stackPtr == 0) break; else nodeIdx = stack[--stackPtr]; }
		else nodeIdx = n.leftFirst, stack[stackPtr++] = n.leftFirst + 1;
	}
	return (int32_t)retVal;
}

template <typename Float, typename Index> int32_t BVH<Float, Index>::LeafCount() const
{
	// Determine the number of nodes in the tree. Typically the result should
	// be usedNodes - 1 (second node is always unused), but some builders may
	// have unused nodes besides node 1. TODO: Support more layouts.
	Index retVal = 0, nodeIdx = 0, stack[64], stackPtr = 0;
	while (1)
	{
		const BVHNode& n = bvhNode[nodeIdx];
		if (n.isLeaf()) { retVal++; if (stackPtr == 0) break; else nodeIdx = stack[--stackPtr]; }
		else nodeIdx = n.leftFirst, stack[stackPtr++] = n.leftFirst + 1;
	}
	return (int32_t)retVal;
}

// Compact: Reduce the size of a BVH by removing any unused nodes.
// This is useful after an SBVH build or multi-threaded build, but also after
// calling MergeLeafs. Some operations, such as Optimize, *require* a
// compacted tree to work correctly.
template <typename Float, typename Index> void BVH<Float, Index>::Compact()
{
	BVH_FATAL_ERROR_IF( bvhNode == 0, "BVH::Compact(), bvhNode == 0." );
	if (bvhNode[0].isLeaf()) return; // nothing to compact.
	BVHNode* tmpNodes = (BVHNode*)AlignedAlloc( sizeof( BVHNode ) * allocatedNodes /* do *not* trim */ );
	Index* idx = (Index*)AlignedAlloc( sizeof( Index ) * idxCount );
	memcpy( tmpNodes, bvhNode, 2 * sizeof( BVHNode ) );
	newNodePtr = 2;
	Index newIdxPtr = 0, nodeIdx = 0, stack[TINYBVH_STACK_SIZE], stackPtr = 0;
	while (1)
	{
		BVHNode& node = tmpNodes[nodeIdx];
		if (node.isLeaf())
		{
			const Index leafStart = newIdxPtr;
			for (Index i = 0; i < node.triCount; i++) idx[newIdxPtr++] = primIdx[node.leftFirst + i];
			node.leftFirst = leafStart;
			if (!stackPtr) break;
			nodeIdx = stack[--stackPtr];
		}
		else
		{
			const BVHNode& left = bvhNode[node.leftFirst];
			const BVHNode& right = bvhNode[node.leftFirst + 1];
			tmpNodes[newNodePtr] = left, tmpNodes[newNodePtr + 1] = right;
			const Index todo1 = newNodePtr, todo2 = newNodePtr + 1;
			node.leftFirst = newNodePtr, newNodePtr += 2;
			nodeIdx = todo1, stack[stackPtr++] = todo2;
		}
	}
	AlignedFree( bvhNode );
	AlignedFree( primIdx );
	usedNodes = newNodePtr, bvhNode = tmpNodes, primIdx = idx;
}

// BVH_Verbose implementation
// ----------------------------------------------------------------------------

template <typename Float, typename Index> BVH_Verbose<Float, Index>::BVH_Verbose( BVH_Verbose&& other )
{
	*this = other;
	other.bvhNode = 0;
}

template <typename Float, typename Index> BVH_Verbose<Float, Index>::~BVH_Verbose()
{
	AlignedFree( bvhNode );
}

template <typename Float, typename Index> void BVH_Verbose<Float, Index>::ConvertFrom( const BVH& original, bool /* unused here */ )
{
	// allocate space
	Index spaceNeeded = original.triCount * (original.refittable ? 2 : 3);
	CopyBasePropertiesFrom( original );
	if (allocatedNodes < spaceNeeded)
	{
		AlignedFree( bvhNode );
		bvhNode = (BVHNode*)AlignedAlloc( sizeof( BVHNode ) * spaceNeeded );
		allocatedNodes = spaceNeeded;
	}
	memset( bvhNode, 0, sizeof( BVHNode ) * spaceNeeded );
	this->verts = original.verts;
	this->fragment = original.fragment;
	this->primIdx = original.primIdx;
	bvhNode[0].parent = (Index)-1; // root sentinel
	// convert
	Index nodeIdx = 0, parent = (Index)-1, stack[128], stackPtr = 0;
	while (1)
	{
		const typename BVH::BVHNode& orig = original.bvhNode[nodeIdx];
		bvhNode[nodeIdx].aabbMin = orig.aabbMin, bvhNode[nodeIdx].aabbMax = orig.aabbMax;
		bvhNode[nodeIdx].triCount = orig.triCount, bvhNode[nodeIdx].parent = parent;
		if (orig.isLeaf())
		{
			bvhNode[nodeIdx].firstTri = orig.leftFirst;
			if (stackPtr == 0) break;
			nodeIdx = stack[--stackPtr];
			parent = stack[--stackPtr];
		}
		else
		{
			bvhNode[nodeIdx].left = orig.leftFirst;
			bvhNode[nodeIdx].right = orig.leftFirst + 1;
			stack[stackPtr++] = nodeIdx;
			stack[stackPtr++] = orig.leftFirst + 1;
			parent = nodeIdx;
			nodeIdx = orig.leftFirst;
		}
	}
	usedNodes = original.usedNodes;
}

template <typename Float, typename Index> int32_t BVH_Verbose<Float, Index>::NodeCount() const
{
	Index retVal = 0, nodeIdx = 0, stack[64], stackPtr = 0;
	while (1)
	{
		const BVHNode& n = bvhNode[nodeIdx];
		retVal++;
		if (n.isLeaf()) { if (stackPtr == 0) break; else nodeIdx = stack[--stackPtr]; }
		else nodeIdx = n.left, stack[stackPtr++] = n.right;
	}
	return (int32_t)retVal;
}

template <typename Float, typename Index> Float BVH_Verbose<Float, Index>::SAHCost( const Index nodeIdx ) const
{
	const BVHNode& n = bvhNode[nodeIdx];
	const Float SAn = SA( n.aabbMin, n.aabbMax );
	if (n.isLeaf()) return c_int * SAn * n.triCount;
	Float cost = c_trav * SAn + SAHCost( n.left ) + SAHCost( n.right );
	return nodeIdx == 0 ? (cost / SAn) : cost;
}

template <typename Float, typename Index> void BVH_Verbose<Float, Index>::Refit( const Index nodeIdx, bool skipLeafs )
{
	BVH_FATAL_ERROR_IF( !refittable && !skipLeafs, "BVH_Verbose::Refit( .. ), refitting an SBVH." );
	BVH_FATAL_ERROR_IF( bvhNode == 0, "BVH_Verbose::Refit( .. ), bvhNode == 0." );
	BVH_FATAL_ERROR_IF( bvh_over_indices && !skipLeafs, "BVH_Verbose::Refit( .. ), bvh used indexed tris." );
	BVHNode& node = bvhNode[nodeIdx];
	if (node.isLeaf()) // leaf: adjust to current triangle vertex positions
	{
		if (skipLeafs) return;
		Vec3 bmin( bvh_far<Float> ), bmax( -bvh_far<Float> );
		for (Index first = node.firstTri, j = 0; j < node.triCount; j++)
		{
			const Index vertIdx = primIdx[first + j] * 3;
			const Vec3 v0 = verts[vertIdx];
			const Vec3 v1 = verts[vertIdx + 1];
			const Vec3 v2 = verts[vertIdx + 2];
			bmin = tinybvh_min( bmin, v0 ), bmax = tinybvh_max( bmax, v0 );
			bmin = tinybvh_min( bmin, v1 ), bmax = tinybvh_max( bmax, v1 );
			bmin = tinybvh_min( bmin, v2 ), bmax = tinybvh_max( bmax, v2 );
		}
		node.aabbMin = bmin, node.aabbMax = bmax;
	}
	else
	{
		Refit( node.left, skipLeafs );
		Refit( node.right, skipLeafs );
		node.aabbMin = tinybvh_min( bvhNode[node.left].aabbMin, bvhNode[node.right].aabbMin );
		node.aabbMax = tinybvh_max( bvhNode[node.left].aabbMax, bvhNode[node.right].aabbMax );
	}
	if (nodeIdx == 0) aabbMin = node.aabbMin, aabbMax = node.aabbMax;
}

template <typename Float, typename Index> void BVH_Verbose<Float, Index>::CheckFit( const Index nodeIdx, bool skipLeafs )
{
	BVHNode& node = bvhNode[nodeIdx];
	Vec3 bmin( bvh_far<Float> ), bmax( -bvh_far<Float> );
	if (node.isLeaf()) // leaf: adjust to current triangle vertex positions
	{
		if (skipLeafs) return;
		for (Index first = node.firstTri, j = 0; j < node.triCount; j++)
		{
			const Index vertIdx = primIdx[first + j] * 3;
			const Vec3 v0 = verts[vertIdx];
			const Vec3 v1 = verts[vertIdx + 1];
			const Vec3 v2 = verts[vertIdx + 2];
			bmin = tinybvh_min( bmin, v0 ), bmax = tinybvh_max( bmax, v0 );
			bmin = tinybvh_min( bmin, v1 ), bmax = tinybvh_max( bmax, v1 );
			bmin = tinybvh_min( bmin, v2 ), bmax = tinybvh_max( bmax, v2 );
		}
	}
	else
	{
		CheckFit( node.left, skipLeafs );
		CheckFit( node.right, skipLeafs );
		bmin = tinybvh_min( bvhNode[node.left].aabbMin, bvhNode[node.right].aabbMin );
		bmax = tinybvh_max( bvhNode[node.left].aabbMax, bvhNode[node.right].aabbMax );
	}
}

template <typename Float, typename Index> void BVH_Verbose<Float, Index>::Compact()
{
	BVH_FATAL_ERROR_IF( bvhNode == 0, "BVH_Verbose::Compact(), bvhNode == 0." );
	if (bvhNode[0].isLeaf()) return; // nothing to compact.
	BVHNode* tmp = (BVHNode*)AlignedAlloc( sizeof( BVHNode ) * usedNodes );
	memcpy( tmp, bvhNode, 2 * sizeof( BVHNode ) );
	Index newNodePtr = 2, nodeIdx = 0, stack[TINYBVH_STACK_SIZE], stackPtr = 0;
	while (1)
	{
		BVHNode& node = tmp[nodeIdx];
		const BVHNode& left = bvhNode[node.left];
		const BVHNode& right = bvhNode[node.right];
		const Index todo1 = newNodePtr, todo2 = newNodePtr + 1;
		tmp[todo1] = left, tmp[todo2] = right;
		tmp[todo1].parent = tmp[todo2].parent = nodeIdx;
		node.left = todo1, node.right = todo2, newNodePtr += 2;
		if (!left.isLeaf())
		{
			if (!right.isLeaf())
			{
				BVH_FATAL_ERROR_IF( stackPtr == TINYBVH_STACK_SIZE, "BVH_Verbose::Compact(), stack overflow." );
				stack[stackPtr++] = todo2;
			}
			nodeIdx = todo1;
			continue;
		}
		if (!right.isLeaf()) { nodeIdx = todo2; continue; }
		if (!stackPtr) break;
		nodeIdx = stack[--stackPtr];
	}
	usedNodes = newNodePtr;
	AlignedFree( bvhNode );
	bvhNode = tmp;
	may_have_holes = false; // the relocated node list is contiguous again.
}

template <typename Float, typename Index> void BVH_Verbose<Float, Index>::SortIndices()
{
	// create a new primIdx array which has the primitive indices sorted by depth-first traversal order.
	if (bvhNode == 0 || idxCount == 0) return;
	Index* tmp = (Index*)AlignedAlloc( idxCount * sizeof( Index ) );
	Index* stack = (Index*)AlignedAlloc( usedNodes * sizeof( Index ) );
	Index nodeIdx = 0, stackPtr = 0, nextIdx = 0;
	while (1)
	{
		BVHNode& node = bvhNode[nodeIdx];
		if (node.isLeaf())
		{
			const Index tmpFirst = nextIdx;
			for (Index i = 0; i < node.triCount; i++) tmp[nextIdx++] = primIdx[node.firstTri + i];
			node.firstTri = tmpFirst;
			if (stackPtr == 0) break; else nodeIdx = stack[--stackPtr];
			continue;
		}
		nodeIdx = node.left;
		stack[stackPtr++] = node.right;
	}
	memcpy( primIdx, tmp, nextIdx * sizeof( Index ) );
	AlignedFree( stack );
	AlignedFree( tmp );
}

template <typename Float, typename Index> void BVH_Verbose<Float, Index>::Optimize( const uint32_t iterations, const bool extreme, bool stochastic )
{
	BVH_FATAL_ERROR_IF( bvhNode == 0, "BVH_Verbose::Optimize( .. ), bvhNode == 0." );
	BVH_FATAL_ERROR_IF( may_have_holes, "BVH_Verbose::Optimize( .. ), bvh may have holes; Compact() first." );
	if (iterations == 0 || usedNodes < 6 || bvhNode[0].isLeaf()) return;
	// working memory, allocated once.
	SortItem* sortList = (SortItem*)AlignedAlloc( usedNodes * sizeof( SortItem ) );
	const uint32_t journalCap = 4096; // >> 3x max tree depth
	RefitRecord* journal = (RefitRecord*)AlignedAlloc( journalCap * sizeof( RefitRecord ) );
	const double minGain = -1e-7 * (double)bvhNode[0].SA();
	uint32_t seed = 0x12345678u;
	// optimize by reinserting subtrees with a high cost - Section 3.4 of the paper.
	for (uint32_t i = 0; i < iterations; i++)
	{
		// calculate combined cost for all nodes
		Index interiorNodes = 0;
		for (Index j = 2; j < usedNodes; j++)
		{
			const BVHNode& node = bvhNode[j];
			if (node.isLeaf()) continue;
			// skip the two children of the root: they have no grandparent to reattach to.
			if (node.parent == 0) continue;
			const Float A = node.SA(), AL = bvhNode[node.left].SA(), AR = bvhNode[node.right].SA();
			const Float Mmin = A / tinybvh_max( Float( 1e-10 ), tinybvh_min( AL, AR ) );
			const Float Msum = A / tinybvh_max( Float( 1e-10 ), Float( 0.5 ) * (AL + AR) );
			sortList[interiorNodes].idx = j, sortList[interiorNodes++].cost = A * Msum * Mmin;
		}
		if (interiorNodes == 0) break;
		// last couple of iterations we will process more nodes.
		const Float portion = stochastic ? 0.5f : (extreme ? (0.01f + (0.6f * (Float)i) / (Float)iterations) : 0.01f);
		const int limit = tinybvh_max( 1, tinybvh_min( (int)interiorNodes, (int)(portion * (Float)interiorNodes) ) );
		const int step = tinybvh_max( 1, (int)(portion / 0.02f) );
		// partial descending sort: only the first 'limit' entries need to end up in order.
		struct Task { int first, last; } stack[TINYBVH_STACK_SIZE];
		int first = 0, last = (int)interiorNodes - 1, stackPtr = 0;
		while (1)
		{
			if (first >= last)
			{
				if (stackPtr == 0) break; else first = stack[--stackPtr].first, last = stack[stackPtr].last;
				continue;
			}
			// median-of-three pivot
			SortItem t;
			const int mid = first + ((last - first) >> 1);
			if (sortList[mid].cost > sortList[first].cost) t = sortList[mid], sortList[mid] = sortList[first], sortList[first] = t;
			if (sortList[last].cost > sortList[first].cost) t = sortList[last], sortList[last] = sortList[first], sortList[first] = t;
			if (sortList[last].cost > sortList[mid].cost) t = sortList[last], sortList[last] = sortList[mid], sortList[mid] = t;
			t = sortList[mid], sortList[mid] = sortList[first], sortList[first] = t;
			int pivot = first;
			const SortItem e = sortList[first];
			for (int j = first + 1; j <= last; j++) if (sortList[j].cost > e.cost)
				t = sortList[j], sortList[j] = sortList[++pivot], sortList[pivot] = t;
			t = sortList[pivot], sortList[pivot] = sortList[first], sortList[first] = t;
			if (pivot + 1 < limit && stackPtr < TINYBVH_STACK_SIZE) stack[stackPtr].first = pivot + 1, stack[stackPtr++].last = last;
			last = pivot - 1;
		}
		// reinsert selected nodes
		BVHNode bckp[5];
		int start = 0;
		if (stochastic)
		{
			float r = tinybvh_rndfloat( seed );
			r = tinybvh_max( 0.0f, (r * 1.2f) - 0.3f ); // 0 .. 0.9f
			start = (int)((Float)limit * r);
		}
		Index accepted = 0;
		for (int j = start; j < limit; j += stochastic ? (int)((tinybvh_rnduint( seed ) & 63) + 1) : step)
		{
			// prepare change
			const Index Nid = sortList[j].idx;
			BVHNode& N = bvhNode[Nid]; // N must have a grandparent to reattach its sibling to.
			if (N.isLeaf() || N.parent == 0 || N.parent == (Index)-1) continue;
			const Index Pid = N.parent;
			BVHNode& P = bvhNode[Pid];
			const Index X1 = P.parent, X2 = (P.left == Nid ? P.right : P.left);
			uint32_t jPtr = 0;
			// execute the change, accumulating the exact SAH delta as we go.
			bckp[0] = bvhNode[X1];
			if (bvhNode[X1].left == Pid) bvhNode[X1].left = X2;
			else /* bvhNode[X1].right == Pid */ bvhNode[X1].right = X2;
			const Index p2 = bvhNode[X2].parent;
			bvhNode[X2].parent = X1;
			const Index Lid = N.left, Rid = N.right;
			double deltaArea = RefitUp( X2, journal, jPtr, journalCap );
			Float cost1, cost2;
			const Index Xbest1 = FindBestNewPosition( Lid, cost1 );
			if (Xbest1 == 0)
			{
				while (jPtr) --jPtr, bvhNode[journal[jPtr].node].aabbMin = journal[jPtr].bmin,
					bvhNode[journal[jPtr].node].aabbMax = journal[jPtr].bmax;
				bvhNode[X2].parent = p2, bvhNode[X1] = bckp[0];
				continue;
			}
			const Index XA = bvhNode[Xbest1].parent;
			bckp[1] = bvhNode[Nid];
			N.left = Xbest1, N.right = Lid, N.parent = XA;
			bckp[2] = bvhNode[XA];
			if (bvhNode[XA].left == Xbest1) bvhNode[XA].left = Nid; else bvhNode[XA].right = Nid;
			const Index p3 = bvhNode[Xbest1].parent, p4 = bvhNode[Lid].parent;
			bvhNode[Xbest1].parent = Nid, bvhNode[Lid].parent = Nid;
			deltaArea += RefitUp( Nid, journal, jPtr, journalCap );
			const Index Xbest2 = FindBestNewPosition( Rid, cost2 );
			if (Xbest2 == 0) // [10] defensive, as above.
			{
				bvhNode[Lid].parent = p4, bvhNode[Xbest1].parent = p3;
				bvhNode[XA] = bckp[2], bvhNode[Nid] = bckp[1];
				bvhNode[X2].parent = p2, bvhNode[X1] = bckp[0];
				while (jPtr) --jPtr, bvhNode[journal[jPtr].node].aabbMin = journal[jPtr].bmin,
					bvhNode[journal[jPtr].node].aabbMax = journal[jPtr].bmax;
				continue;
			}
			const Index XB = bvhNode[Xbest2].parent;
			bckp[3] = bvhNode[Pid];
			P.left = Xbest2, P.right = Rid, P.parent = XB;
			bckp[4] = bvhNode[XB];
			if (bvhNode[XB].left == Xbest2) bvhNode[XB].left = Pid; else bvhNode[XB].right = Pid;
			const Index p1 = bvhNode[Xbest2].parent, p0 = bvhNode[Rid].parent;
			bvhNode[Xbest2].parent = Pid, bvhNode[Rid].parent = Pid;
			deltaArea += RefitUp( Pid, journal, jPtr, journalCap );
			// keep the change if it meaningfully reduced the SAH cost of the tree.
			if (deltaArea < minGain) { accepted++; continue; }
			// undo change, mind the order.
			bvhNode[Rid].parent = p0, bvhNode[Xbest2].parent = p1, bvhNode[XB] = bckp[4];
			bvhNode[Pid] = bckp[3], bvhNode[Lid].parent = p4, bvhNode[Xbest1].parent = p3;
			bvhNode[XA] = bckp[2], bvhNode[Nid] = bckp[1], bvhNode[X2].parent = p2, bvhNode[X1] = bckp[0];
			// restore the boxes from the journal, newest first.
			while (jPtr) --jPtr, bvhNode[journal[jPtr].node].aabbMin = journal[jPtr].bmin,
				bvhNode[journal[jPtr].node].aabbMax = journal[jPtr].bmax;
		}
		if (accepted == 0 && !stochastic && !extreme) break;
	}
	Refit( 0, true );
	AlignedFree( journal );
	AlignedFree( sortList );
}

// Single-primitive leafs: Prepare the BVH for optimization. While it is not strictly
// necessary to have a single primitive per leaf, it will yield a slightly better
// optimized BVH. The leafs of the optimized BVH should be collapsed ('MergeLeafs')
// to obtain the final tree.
template <typename Float, typename Index> void BVH_Verbose<Float, Index>::SplitLeafs( const Index maxPrims )
{
	Index nodeIdx = 0, stack[TINYBVH_STACK_SIZE], stackPtr = 0;
	while (1)
	{
		BVHNode& node = bvhNode[nodeIdx];
		if (!node.isLeaf()) nodeIdx = node.left, stack[stackPtr++] = node.right; else
		{
			// split this leaf
			if (node.triCount > maxPrims)
			{
				const Index newIdx1 = usedNodes++, newIdx2 = usedNodes++;
				BVHNode& new1 = bvhNode[newIdx1], & new2 = bvhNode[newIdx2];
				new1.firstTri = node.firstTri, new1.triCount = node.triCount / 2;
				new1.parent = new2.parent = nodeIdx, new1.left = new1.right = 0;
				new2.firstTri = node.firstTri + new1.triCount;
				new2.triCount = node.triCount - new1.triCount, new2.left = new2.right = 0;
				node.left = newIdx1, node.right = newIdx2, node.triCount = 0;
				new1.aabbMin = new2.aabbMin = Vec3( bvh_far<Float> ), new1.aabbMax = new2.aabbMax = Vec3( -bvh_far<Float> );
				for (Index fi, i = 0; i < new1.triCount; i++)
					fi = primIdx[new1.firstTri + i],
					new1.aabbMin = tinybvh_min( new1.aabbMin, fragment[fi].bmin ),
					new1.aabbMax = tinybvh_max( new1.aabbMax, fragment[fi].bmax );
				for (Index fi, i = 0; i < new2.triCount; i++)
					fi = primIdx[new2.firstTri + i],
					new2.aabbMin = tinybvh_min( new2.aabbMin, fragment[fi].bmin ),
					new2.aabbMax = tinybvh_max( new2.aabbMax, fragment[fi].bmax );
				// recurse
				if (new1.triCount > 1) stack[stackPtr++] = newIdx1;
				if (new2.triCount > 1) stack[stackPtr++] = newIdx2;
			}
			if (stackPtr == 0) break; else nodeIdx = stack[--stackPtr];
		}
	}
}

// MergeLeafs: After optimizing a BVH, single-primitive leafs should be merged whenever
// SAH indicates this is an improvement.
template <typename Float, typename Index> void BVH_Verbose<Float, Index>::MergeLeafs()
{
	// allocate some working space
	if (bvhNode == 0 || idxCount == 0) return;
	Index* subtreeTriCount = (Index*)AlignedAlloc( usedNodes * sizeof( Index ) );
	Index* newIdx = (Index*)AlignedAlloc( idxCount * sizeof( Index ) );
	memset( subtreeTriCount, 0, usedNodes * sizeof( Index ) );
	CountSubtreeTris( 0, subtreeTriCount );
	Index stack[TINYBVH_STACK_SIZE], stackPtr = 0, nodeIdx = 0, newIdxPtr = 0;
	while (1)
	{
		BVHNode& node = bvhNode[nodeIdx];
		if (node.isLeaf())
		{
			Index start = newIdxPtr;
			MergeSubtree( nodeIdx, newIdx, newIdxPtr );
			node.firstTri = start;
			// pop new task
			if (stackPtr == 0) break;
			nodeIdx = stack[--stackPtr];
		}
		else
		{
			const Index leftCount = subtreeTriCount[node.left];
			const Index rightCount = subtreeTriCount[node.right];
			const Index mergedCount = leftCount + rightCount;
			// cost of unsplit
			Float Cunsplit = SA( node.aabbMin, node.aabbMax ) * mergedCount * c_int;
			// cost of leaving things as they are
			BVHNode& left = bvhNode[node.left];
			BVHNode& right = bvhNode[node.right];
			Float Ckeepsplit = c_trav + c_int * (left.SA() * leftCount + right.SA() * rightCount);
			if (Cunsplit <= Ckeepsplit)
			{
				// collapse the subtree
				Index start = newIdxPtr;
				MergeSubtree( nodeIdx, newIdx, newIdxPtr );
				node.firstTri = start, node.triCount = mergedCount;
				node.left = node.right = 0;
				// pop new task
				if (stackPtr == 0) break;
				nodeIdx = stack[--stackPtr];
			}
			else /* recurse */ nodeIdx = node.left, stack[stackPtr++] = node.right;
		}
	}
	// cleanup
	memcpy( primIdx, newIdx, newIdxPtr * sizeof( Index ) );
	AlignedFree( newIdx );
	AlignedFree( subtreeTriCount );
	may_have_holes = true; // all over the place, in fact
}

// BVH_GPU implementation
// ----------------------------------------------------------------------------

template <typename Float, typename Index> BVH_GPU<Float, Index>::BVH_GPU( BVH_GPU&& other )
{
	*this = other;
	// mark 'other' as deleted to avoid double-free
	other.bvhNode = 0;
	other.orderedVerts = Slice( nullptr, 0, 0 );
}

template <typename Float, typename Index> BVH_GPU<Float, Index>::~BVH_GPU()
{
	if (!ownBVH) bvh = BVH(); // clear out pointers we don't own.
	AlignedFree( bvhNode );
	bvhNode = 0;
	AlignedFree( (void*)orderedVerts.data ); // allocated by ConvertFrom; we own it.
	orderedVerts = Slice( nullptr, 0, 0 );
}

// forwarders
template <typename Float, typename Index> void BVH_GPU<Float, Index>::Build( const bvhvec4* v, const Index p ) { Build( Slice( v, p * 3, sizeof( bvhvec4 ) ) ); }
template <typename Float, typename Index> void BVH_GPU<Float, Index>::Build( const Slice& v ) { Build( v, 0, 0 ); }
template <typename Float, typename Index> void BVH_GPU<Float, Index>::Build( const bvhvec4* v, const uint32_t* i, const Index p ) { Build( Slice( v, p * 3, sizeof( bvhvec4 ) ), i, p ); }
template <typename Float, typename Index> void BVH_GPU<Float, Index>::BuildHQ( const bvhvec4* v, const Index p ) { BuildHQ( Slice( v, p * 3, sizeof( bvhvec4 ) ) ); }
template <typename Float, typename Index> void BVH_GPU<Float, Index>::BuildHQ( const bvhvec4* v, const uint32_t* i, const Index p ) { BuildHQ( Slice( v, p * 3, sizeof( bvhvec4 ) ), i, p ); }
template <typename Float, typename Index> void BVH_GPU<Float, Index>::BuildHQ( const Slice& v, const uint32_t* i, Index p ) { settings.useSpatialSplits = true; Build( v, i, p ); }
template <typename Float, typename Index> void BVH_GPU<Float, Index>::BuildHQ( const Slice& v ) { settings.useSpatialSplits = true; Build( v ); }

template <typename Float, typename Index> void BVH_GPU<Float, Index>::Build( const Slice& vertices, const uint32_t* indices, Index prims )
{
	// propagate settings for this layout to the underlying layout
	bvh.context = context, bvh.settings = settings;
	bvh.c_int = c_int, bvh.c_trav = c_trav;
	// build underlying layout
	if (indices) bvh.Build( vertices, indices, prims ); else bvh.Build( vertices );
	// convert to BVH_GPU layout
	ConvertFrom( bvh, false );
}

template <typename Float, typename Index> void BVH_GPU<Float, Index>::BuildAABB( const bvhvec4* aabbs, const Index primCount )
{
	// build a TLAS based on the array of BLASInstance records.
	bvh.context = context, bvh.settings = settings;
	bvh.c_int = c_int, bvh.c_trav = c_trav;
	// build underlying layout
	bvh.BuildAABB( aabbs, primCount );
	// convert to BVH_GPU layout
	ConvertFrom( bvh );
}

template <typename Float, typename Index> void BVH_GPU<Float, Index>::Build( BLASInstance* instances, const Index instCount, BVHBase<Float, Index>** blasses, const Index blasCount )
{
	// build a TLAS based on the array of BLASInstance records.
	bvh.context = context, bvh.settings = settings;
	bvh.c_int = c_int, bvh.c_trav = c_trav;
	// build underlying layout
	bvh.Build( instances, instCount, blasses, blasCount );
	// convert to BVH_GPU layout
	ConvertFrom( bvh, false );
}

template <typename Float, typename Index> void BVH_GPU<Float, Index>::Optimize( const uint32_t iterations, bool extreme )
{
	bvh.Optimize( iterations, extreme );
	ConvertFrom( bvh, false );
}

// Encode a BVH2 leaf as a compact child reference for the BVH_GPU layout:
// msb marks leaf, 24..30 hold triangle count and 0..23 the offset into primIdx.
template <typename Node> static uint32_t CompactLeafRef( const Node& leaf )
{
	return 0x80000000u | (tinybvh_min( 127u, (uint32_t)leaf.triCount ) << 24) | ((uint32_t)leaf.leftFirst & 0xffffff);
}

template <typename Float, typename Index> void BVH_GPU<Float, Index>::ConvertFrom( const BVH& original, bool compact )
{
	// get a copy of the original bvh
	if (&original != &bvh) ownBVH = false; // bvh isn't ours; don't delete in destructor.
	bvh = original;
	const uint32_t sourceNodes = (uint32_t)(compact ? original.usedNodes : original.allocatedNodes);
	const uint32_t spaceNeeded = (sourceNodes >> 1) + 1; // the +1 covers a leaf-only bvh
	CopyBasePropertiesFrom( original );
	if (allocatedNodes < spaceNeeded)
	{
		AlignedFree( bvhNode );
		bvhNode = (BVHNode*)AlignedAlloc( sizeof( BVHNode ) * spaceNeeded );
		allocatedNodes = spaceNeeded;
	}
	memset( bvhNode, 0, sizeof( BVHNode ) * spaceNeeded );
	this->may_have_holes = false;
	// a bvh consisting of a single leaf: traversal always starts at an interior
	// node, so wrap the leaf in one. The second child is an empty leaf, which is
	// harmless to 'hit' - it intersects nothing and pops immediately.
	if (original.bvhNode[0].isLeaf())
	{
		BVHNode& root = this->bvhNode[0];
		root.lmin = original.bvhNode[0].aabbMin, root.lmax = original.bvhNode[0].aabbMax;
		root.rmin = root.rmax = original.bvhNode[0].aabbMin;
		root.left = CompactLeafRef( original.bvhNode[0] );
		root.right = 0x80000000u; // triCount 0, firstTri 0
		usedNodes = 1;
		return;
	}
	// depth-first conversion
	uint32_t stack[TINYBVH_STACK_SIZE], stackPtr = 0, newNodePtr = 1, nodeIdx = 0, slot = 0;
	while (1)
	{
		const typename BVH::BVHNode& orig = original.bvhNode[nodeIdx];
		uint32_t leftIdx = (uint32_t)orig.leftFirst, rightIdx = leftIdx + 1;
		const typename BVH::BVHNode& c0 = original.bvhNode[leftIdx];
		const typename BVH::BVHNode& c1 = original.bvhNode[rightIdx];
		// put the larger node to the left to improve cache coherence during traversal
		if (tinybvh_halfarea( c0.aabbMax - c0.aabbMin ) <= tinybvh_halfarea( c1.aabbMax - c1.aabbMin ))
			tinybvh_swap( leftIdx, rightIdx );
		const typename BVH::BVHNode& left = original.bvhNode[leftIdx];
		const typename BVH::BVHNode& right = original.bvhNode[rightIdx];
		BVHNode& node = this->bvhNode[slot];
		node.lmin = left.aabbMin, node.lmax = left.aabbMax;
		node.rmin = right.aabbMin, node.rmax = right.aabbMax;
		node.triCount = 0, node.firstTri = 0; // unused: leafs live in left / right
		const bool leftLeaf = left.isLeaf(), rightLeaf = right.isLeaf();
		uint32_t leftSlot = 0, rightSlot = 0;
		if (leftLeaf) node.left = CompactLeafRef( left ); else node.left = leftSlot = newNodePtr++;
		if (rightLeaf) node.right = CompactLeafRef( right ); else node.right = rightSlot = newNodePtr++;
		// defer the right subtree, then descend into the left one
		if (!rightLeaf) stack[stackPtr++] = rightIdx, stack[stackPtr++] = rightSlot;
		if (!leftLeaf) { nodeIdx = leftIdx, slot = leftSlot; continue; }
		if (!stackPtr) break;
		slot = stack[--stackPtr], nodeIdx = stack[--stackPtr];
	}
	usedNodes = newNodePtr;
	// reorder triangle data to avoid the idx array indirection - release the buffer from a previous 
	// conversion; this object owns it.
	AlignedFree( (void*)orderedVerts.data );
	orderedVerts = Slice( nullptr, 0, 0 );
	if (bvh_over_aabbs) return; // skip for TLAS builds.
	bvhvec4* vertexData = (bvhvec4*)AlignedAlloc( sizeof( bvhvec4 ) * 3 * idxCount );
	orderedVerts = Slice( vertexData, 3 * idxCount );
	for (uint32_t i = 0; i < idxCount; i++)
	{
		uint32_t pidx = (uint32_t)bvh.primIdx[i];
		if (pidx < triCount)
		{
			vertexData[i * 3 + 0] = bvh.verts[pidx * 3 + 0];
			vertexData[i * 3 + 1] = bvh.verts[pidx * 3 + 1] - vertexData[i * 3 + 0];
			vertexData[i * 3 + 2] = bvh.verts[pidx * 3 + 2] - vertexData[i * 3 + 0];
			vertexData[i * 3 + 0].w = tinybvh_as_float( pidx ); // store original primitive index.
			vertexData[i * 3 + 1].w = bvh.verts[pidx * 3 + 1].w; // keep; may contain triangle color.
		}
	}
}

template <typename Float, typename Index> int32_t BVH_GPU<Float, Index>::Intersect( Ray& ray ) const
{
	VALIDATE_RAY( ray );
	uint32_t nodeIdx = 0, stack[TINYBVH_STACK_SIZE], stackPtr = 0;
	float cost = 0;
	while (1)
	{
		cost += c_trav;
		if (nodeIdx & 0x80000000u)
		{
			uint32_t primCount = (nodeIdx >> 24) & 127, firstTri = nodeIdx & 0xffffff;
			for (uint32_t i = 0; i < primCount; i++, cost += c_int)
			{
				const uint32_t pi = (firstTri + i) * 3;
				const bvhvec4 v0_ = orderedVerts[pi];
				const Vec3 v0 = v0_, e1 = orderedVerts[pi + 1], e2 = orderedVerts[pi + 2];
				MOLLER_TRUMBORE_TEST( ray.hit.t, continue );
				// register a hit: ray is shortened to t.
				ray.hit.t = t, ray.hit.u = u, ray.hit.v = v;
				ray.SetHitPrim( tinybvh_as_uint( v0_.w ) );
			}
			if (stackPtr == 0) break; else nodeIdx = stack[--stackPtr];
			continue;
		}
		// not a leaf; nodeIdx contains the index of an interior node.
		BVHNode& node = bvhNode[nodeIdx];
		float dist1 = bvh_far<Float>, dist2 = bvh_far<Float>;
		const Vec3 t1a = (node.lmin - ray.O) * ray.rD, t2a = (node.lmax - ray.O) * ray.rD;
		const Vec3 t1b = (node.rmin - ray.O) * ray.rD, t2b = (node.rmax - ray.O) * ray.rD;
		const float tmina = tinybvh_max( tinybvh_max( tinybvh_min( t1a.x, t2a.x ), tinybvh_min( t1a.y, t2a.y ) ), tinybvh_min( t1a.z, t2a.z ) );
		const float tmaxa = tinybvh_min( tinybvh_min( tinybvh_max( t1a.x, t2a.x ), tinybvh_max( t1a.y, t2a.y ) ), tinybvh_max( t1a.z, t2a.z ) );
		const float tminb = tinybvh_max( tinybvh_max( tinybvh_min( t1b.x, t2b.x ), tinybvh_min( t1b.y, t2b.y ) ), tinybvh_min( t1b.z, t2b.z ) );
		const float tmaxb = tinybvh_min( tinybvh_min( tinybvh_max( t1b.x, t2b.x ), tinybvh_max( t1b.y, t2b.y ) ), tinybvh_max( t1b.z, t2b.z ) );
		if (tmaxa >= tmina && tmina < ray.hit.t && tmaxa >= 0) dist1 = tmina;
		if (tmaxb >= tminb && tminb < ray.hit.t && tmaxb >= 0) dist2 = tminb;
		uint32_t lidx = node.left, ridx = node.right;
		if (dist1 > dist2)
		{
			float t = dist1; dist1 = dist2; dist2 = t;
			uint32_t i = lidx; lidx = ridx; ridx = i;
		}
		if (dist1 == bvh_far<Float>)
		{
			if (stackPtr == 0) break; else nodeIdx = stack[--stackPtr];
		}
		else
		{
			nodeIdx = lidx;
			if (dist2 != bvh_far<Float>) stack[stackPtr++] = ridx;
		}
	}
	return (int32_t)cost; // cast to not break interface.
}

#ifdef ENABLE_BVH_SOA

// BVH_SoA implementation
// ----------------------------------------------------------------------------

template <typename Float, typename Index> BVH_SoA<Float, Index>::BVH_SoA( BVH_SoA&& other )
{
	*this = other;
	other.bvhNode = 0;
}

template <typename Float, typename Index> BVH_SoA<Float, Index>::~BVH_SoA()
{
	if (!ownBVH) bvh = BVH(); // clear out pointers we don't own.
	AlignedFree( bvhNode );
}

// forwarders
template <typename Float, typename Index> void BVH_SoA<Float, Index>::Build( const Vertex* v, const Index p ) { Build( Slice( v, p * 3, sizeof( Vertex ) ) ); }
template <typename Float, typename Index> void BVH_SoA<Float, Index>::Build( const Vertex* v, const uint32_t* i, const Index p ) { Build( Slice( v, p * 3, sizeof( Vertex ) ), i, p ); }
template <typename Float, typename Index> void BVH_SoA<Float, Index>::Build( const Slice& v ) { Build( v, 0, 0 ); }
template <typename Float, typename Index> void BVH_SoA<Float, Index>::BuildHQ( const Vertex* v, const Index p ) { BuildHQ( Slice( v, p * 3, sizeof( Vertex ) ) ); }
template <typename Float, typename Index> void BVH_SoA<Float, Index>::BuildHQ( const Vertex* v, const uint32_t* indices, const Index p ) { BuildHQ( Slice( v, p * 3, sizeof( Vertex ) ), indices, p ); }
template <typename Float, typename Index> void BVH_SoA<Float, Index>::BuildHQ( const Slice& v ) { BuildHQ( v, 0, 0 ); }
template <typename Float, typename Index> void BVH_SoA<Float, Index>::BuildHQ( const Slice& v, const uint32_t* i, Index p ) { settings.useSpatialSplits = true; Build( v, i, p ); }

template <typename Float, typename Index> void BVH_SoA<Float, Index>::Build( const Slice& vertices, const uint32_t* indices, Index prims )
{
	// propagate settings for this layout to the underlying layout
	bvh.context = context, bvh.settings = settings;
	bvh.c_int = c_int, bvh.c_trav = c_trav;
	// build underlying layout
	bvh.Build( vertices, indices, prims );
	// convert to BVH_SoA layout
	ConvertFrom( bvh, false );
}

template <typename Float, typename Index> void BVH_SoA<Float, Index>::Optimize( const uint32_t iterations, bool extreme )
{
	bvh.Optimize( iterations, extreme );
	ConvertFrom( bvh, false );
}

template <typename Float, typename Index> void BVH_SoA<Float, Index>::Save( const char* fileName )
{
	bvh.Save( fileName );
}

template <typename Float, typename Index> bool BVH_SoA<Float, Index>::Load( const char* fileName, const Vertex* vertices, const Index primCount )
{
	return Load( fileName, Slice( vertices, primCount * 3, sizeof( Vertex ) ) );
}

template <typename Float, typename Index> bool BVH_SoA<Float, Index>::Load( const char* fileName, const Vertex* vertices, const uint32_t* indices, const Index primCount )
{
	return Load( fileName, Slice( vertices, primCount * 3, sizeof( Vertex ) ), indices, primCount );
}

template <typename Float, typename Index> bool BVH_SoA<Float, Index>::Load( const char* fileName, const Slice& vertices, const uint32_t* indices, const Index primCount )
{
	if (!bvh.Load( fileName, vertices, indices, primCount )) return false;
	ConvertFrom( bvh, false );
	return true;
}

template <typename Float, typename Index> void BVH_SoA<Float, Index>::ConvertFrom( const BVH& original, bool compact )
{
	// get a copy of the original bvh
	if (&original != &bvh) ownBVH = false; // bvh isn't ours; don't delete in destructor.
	bvh = original;
	// allocate space
	const Index spaceNeeded = compact ? bvh.usedNodes : bvh.allocatedNodes;
	if (allocatedNodes < spaceNeeded)
	{
		AlignedFree( bvhNode );
		bvhNode = (BVHNode*)AlignedAlloc( sizeof( BVHNode ) * spaceNeeded );
		allocatedNodes = spaceNeeded;
	}
	memset( bvhNode, 0, sizeof( BVHNode ) * spaceNeeded );
	CopyBasePropertiesFrom( bvh );
	// recursively convert nodes
	Index newAlt2Node = 0, nodeIdx = 0, stack[TINYBVH_STACK_SIZE], stackPtr = 0;
	while (1)
	{
		const typename BVH::BVHNode& node = bvh.bvhNode[nodeIdx];
		const Index idx = newAlt2Node++;
		if (node.isLeaf())
		{
			bvhNode[idx].triCount = node.triCount;
			bvhNode[idx].firstTri = node.leftFirst;
			if (!stackPtr) break;
			nodeIdx = stack[--stackPtr];
			Index newNodeParent = stack[--stackPtr];
			bvhNode[newNodeParent].right = newAlt2Node;
		}
		else
		{
			const typename BVH::BVHNode& left = bvh.bvhNode[node.leftFirst];
			const typename BVH::BVHNode& right = bvh.bvhNode[node.leftFirst + 1];
			Float* xx = bvhNode[idx].xxxx, * yy = bvhNode[idx].yyyy, * zz = bvhNode[idx].zzzz;
			xx[0] = left.aabbMin.x, xx[1] = left.aabbMax.x, xx[2] = right.aabbMin.x, xx[3] = right.aabbMax.x;
			yy[0] = left.aabbMin.y, yy[1] = left.aabbMax.y, yy[2] = right.aabbMin.y, yy[3] = right.aabbMax.y;
			zz[0] = left.aabbMin.z, zz[1] = left.aabbMax.z, zz[2] = right.aabbMin.z, zz[3] = right.aabbMax.z;
			bvhNode[idx].left = newAlt2Node; // right will be filled when popped
			stack[stackPtr++] = idx;
			stack[stackPtr++] = node.leftFirst + 1;
			nodeIdx = node.leftFirst;
		}
	}
	usedNodes = newAlt2Node;
}

#endif

// Generic (templated) MBVH implementation
// ----------------------------------------------------------------------------

template <int M, typename Float, typename Index> MBVH<M, Float, Index>::MBVH( MBVH<M, Float, Index>&& other )
{
	*this = other;
	other.mbvhNode = 0;
}

template <int M, typename Float, typename Index> MBVH<M, Float, Index>::~MBVH()
{
	if (!ownBVH) bvh = BVH(); // clear out pointers we don't own.
	AlignedFree( mbvhNode );
}

// forwarders
template <int M, typename Float, typename Index> void MBVH<M, Float, Index>::Build( const Vertex* v, const Index p ) { Build( Slice( v, p * 3, sizeof( Vertex ) ) ); }
template <int M, typename Float, typename Index> void MBVH<M, Float, Index>::Build( const Vertex* v, const uint32_t* i, const Index p ) { Build( Slice( v, p * 3, sizeof( Vertex ) ), i, p ); }
template <int M, typename Float, typename Index> void MBVH<M, Float, Index>::Build( const Slice& v ) { Build( v, 0, 0 ); }
template <int M, typename Float, typename Index> void MBVH<M, Float, Index>::BuildHQ( const Vertex* v, const Index p ) { BuildHQ( Slice( v, p * 3, sizeof( Vertex ) ) ); }
template <int M, typename Float, typename Index> void MBVH<M, Float, Index>::BuildHQ( const Vertex* v, const uint32_t* indices, const Index p ) { BuildHQ( Slice( v, p * 3, sizeof( Vertex ) ), indices, p ); }
template <int M, typename Float, typename Index> void MBVH<M, Float, Index>::BuildHQ( const Slice& v ) { BuildHQ( v, 0, 0 ); }
template <int M, typename Float, typename Index> void MBVH<M, Float, Index>::BuildHQ( const Slice& v, const uint32_t* i, Index p ) { settings.useSpatialSplits = true; Build( v, i, p ); }

template <int M, typename Float, typename Index> void MBVH<M, Float, Index>::Build( const Slice& vertices, const uint32_t* indices, Index prims )
{
	// propagate settings for this layout to the underlying layout
	bvh.context = context, bvh.settings = settings;
	bvh.c_int = c_int, bvh.c_trav = c_trav;
	// build underlying layout
	bvh.Build( vertices, indices, prims );
	// convert to MBVH layout
	ConvertFrom( bvh, true );
}

template <int M, typename Float, typename Index> void MBVH<M, Float, Index>::Optimize( const uint32_t iterations, bool extreme )
{
	bvh.Optimize( iterations, extreme );
	ConvertFrom( bvh, true );
}

template <int M, typename Float, typename Index> Index MBVH<M, Float, Index>::LeafCount( const Index nodeIdx ) const
{
	MBVHNode& node = mbvhNode[nodeIdx];
	if (node.isLeaf()) return 1;
	Index count = 0;
	for (Index i = 0; i < node.childCount; i++) count += LeafCount( node.child[i] );
	return count;
}

template <int M, typename Float, typename Index> void MBVH<M, Float, Index>::Refit( const Index nodeIdx )
{
	MBVHNode& node = mbvhNode[nodeIdx];
	if (node.isLeaf())
	{
		Vec3 bmin( bvh_far<Float> ), bmax( -bvh_far<Float> );
		if (bvh.vertIdx) for (Index first = node.firstTri, j = 0; j < node.triCount; j++)
		{
			const Index vidx = bvh.primIdx[first + j] * 3;
			const uint32_t i0 = bvh.vertIdx[vidx], i1 = bvh.vertIdx[vidx + 1], i2 = bvh.vertIdx[vidx + 2];
			const Vec3 v0 = bvh.verts[i0], v1 = bvh.verts[i1], v2 = bvh.verts[i2];
			bmin = tinybvh_min( bmin, tinybvh_min( tinybvh_min( v0, v1 ), v2 ) );
			bmax = tinybvh_max( bmax, tinybvh_max( tinybvh_max( v0, v1 ), v2 ) );
		}
		else for (Index first = node.firstTri, j = 0; j < node.triCount; j++)
		{
			const Index vidx = bvh.primIdx[first + j] * 3;
			const Vec3 v0 = bvh.verts[vidx], v1 = bvh.verts[vidx + 1], v2 = bvh.verts[vidx + 2];
			bmin = tinybvh_min( bmin, tinybvh_min( tinybvh_min( v0, v1 ), v2 ) );
			bmax = tinybvh_max( bmax, tinybvh_max( tinybvh_max( v0, v1 ), v2 ) );
		}
		node.aabbMin = bmin, node.aabbMax = bmax;
	}
	else
	{
		for (Index i = 0; i < node.childCount; i++) Refit( node.child[i] );
		MBVHNode& firstChild = mbvhNode[node.child[0]];
		Vec3 bmin = firstChild.aabbMin, bmax = firstChild.aabbMax;
		for (Index i = 1; i < node.childCount; i++)
		{
			MBVHNode& child = mbvhNode[node.child[i]];
			bmin = tinybvh_min( bmin, child.aabbMin );
			bmax = tinybvh_max( bmax, child.aabbMax );
		}
		node.aabbMin = bmin, node.aabbMax = bmax;
	}
	if (nodeIdx == 0) aabbMin = node.aabbMin, aabbMax = node.aabbMax;
}

template <int M, typename Float, typename Index> Float MBVH<M, Float, Index>::SAHCost( const Index nodeIdx ) const
{
	// Determine the SAH cost of the tree. This provides an indication
	// of the quality of the BVH: Lower is better.
	const MBVHNode& n = mbvhNode[nodeIdx];
	const Float sa = BVH::SA( n.aabbMin, n.aabbMax );
	if (n.isLeaf()) return c_int * sa * n.triCount;
	Float cost = c_trav * sa;
	for (Index i = 0; i < M; i++) if (n.child[i] != 0) cost += SAHCost( n.child[i] );
	return nodeIdx == 0 ? (cost / sa) : cost;
}

#if 1

// Collapse a BVH2 into an M-wide BVH. Based on "Efficient Incoherent Ray Traversal on GPUs 
// Through Compressed Wide BVHs", Ylitie et al. 2017, section 4.2.
static constexpr float c_leaf = C_INT * 0.8f;
template <int M, typename Float, typename Index> void MBVH<M, Float, Index>::ConvertFrom( const BVH& original, bool compact )
{
	// get a copy of the original bvh
	if (&original != &bvh) ownBVH = false; // bvh isn't ours; don't delete in destructor.
	bvh = original;
	// allocate space; the collapse emits at most one node per bvh2 node, and the
	// 'root is a leaf' case below needs one extra.
	const Index N = original.usedNodes;
	const Index spaceNeeded = (compact ? N : original.allocatedNodes) + 2;
	CopyBasePropertiesFrom( original );
	if (allocatedNodes < spaceNeeded)
	{
		AlignedFree( mbvhNode );
		mbvhNode = (MBVHNode*)AlignedAlloc( spaceNeeded * sizeof( MBVHNode ) );
		allocatedNodes = spaceNeeded;
	}
	memset( mbvhNode, 0, sizeof( MBVHNode ) * spaceNeeded );
	// special case where root is leaf: add extra level - cwbvh needs this.
	if (original.bvhNode[0].isLeaf())
	{
		MBVHNode& root = this->mbvhNode[0], & leaf = this->mbvhNode[1];
		root.aabbMin = leaf.aabbMin = original.bvhNode[0].aabbMin;
		root.aabbMax = leaf.aabbMax = original.bvhNode[0].aabbMax;
		leaf.firstTri = original.bvhNode[0].leftFirst, leaf.triCount = original.bvhNode[0].triCount;
		root.child[0] = 1, root.childCount = 1, root.triCount = 0;
		usedNodes = 2, this->may_have_holes = false;
		return;
	}
	// scratch data for the collapse
	Float* cost = (Float*)AlignedAlloc( (size_t)N * M * sizeof( Float ) );
	uint8_t* dist = (uint8_t*)AlignedAlloc( (size_t)N * M );
	Index* subFirst = (Index*)AlignedAlloc( (size_t)N * sizeof( Index ) );
	Index* subCount = (Index*)AlignedAlloc( (size_t)N * sizeof( Index ) );
	Index* srcNode = (Index*)AlignedAlloc( (size_t)spaceNeeded * sizeof( Index ) );
	uint8_t* flag = (uint8_t*)AlignedAlloc( (size_t)N );
	constexpr uint8_t USED = 1, CONTIG = 2, ASLEAF = 4;
	constexpr Index NOTANODE = (Index)-1;
	memset( flag, 0, N );
	// pass 1: mark the nodes that are actually part of the tree. A child always
	// has a higher index than its parent, so one forward sweep suffices.
	flag[0] = USED;
	for (Index n = 0; n < N; n++) if (flag[n] & USED)
	{
		const typename BVH::BVHNode& node = original.bvhNode[n];
		if (!node.isLeaf()) flag[node.leftFirst] |= USED, flag[node.leftFirst + 1] |= USED;
	}
	// pass 2: bottom-up dynamic programming; same ordering argument, reversed.
	for (int64_t n = (int64_t)N - 1; n >= 0; n--)
	{
		if (!(flag[n] & USED)) continue;
		const typename BVH::BVHNode& node = original.bvhNode[n];
		const size_t base = (size_t)n * M;
		const Float sa = SA( node.aabbMin, node.aabbMax );
		if (node.isLeaf())
		{
			// a leaf takes a single slot, no matter how many it is offered.
			subFirst[n] = node.leftFirst, subCount[n] = node.triCount;
			flag[n] |= CONTIG | ASLEAF;
			const Float c = sa * (c_leaf + c_int * (Float)(l_quads ? (((node.triCount + 3) >> 2) * 4) : node.triCount));
			for (int32_t i = 0; i < M; i++) cost[base + i] = c;
			continue;
		}
		const size_t L = (size_t)node.leftFirst * M, R = L + M;
		const Index li = node.leftFirst, ri = li + 1;
		subCount[n] = subCount[li] + subCount[ri];
		subFirst[n] = tinybvh_min( subFirst[li], subFirst[ri] );
		// a subtree can only become one leaf if it owns a contiguous index range.
		if ((flag[li] & flag[ri] & CONTIG) && subFirst[n] + subCount[n] ==
			tinybvh_max( subFirst[li] + subCount[li], subFirst[ri] + subCount[ri] )) flag[n] |= CONTIG;
		// cost of spreading this subtree over i slots: best split over the children.
		for (int32_t i = 2; i <= M; i++)
		{
			Float best = bvh_far<Float>;
			uint8_t bestK = 1;
			for (int32_t k = 1; k < i; k++)
			{
				const Float c = cost[L + k - 1] + cost[R + (i - k) - 1];
				if (c < best) best = c, bestK = (uint8_t)k;
			}
			cost[base + i - 1] = best, dist[base + i - 1] = bestK;
		}
		// cost in a single slot: a new wide node, or the whole subtree as one leaf.
		const Float cNode = sa * c_trav + cost[base + M - 1];
		Float cLeaf = bvh_far<Float>;
		if ((flag[n] & CONTIG) && subCount[n] <= leafPrimLimit)
			cLeaf = sa * (c_leaf + c_int * (Float)(l_quads ? (((subCount[n] + 3) >> 2) * 4) : subCount[n]));
		if (cLeaf < cNode) cost[base] = cLeaf, flag[n] |= ASLEAF; else cost[base] = cNode;
		// a subtree may always use fewer slots than it is offered; dist == 0 then
		// means 'this subtree occupies a single slot'.
		for (int32_t i = 2; i <= M; i++) if (cost[base] < cost[base + i - 1])
			cost[base + i - 1] = cost[base], dist[base + i - 1] = 0;
	}
	// pass 3: decode the solution, breadth-first, so the children of a node end
	// up in consecutive entries of the node pool.
	usedNodes = 1, srcNode[0] = 0;
	for (Index i = 0; i < usedNodes; i++)
	{
		if (srcNode[i] == NOTANODE) continue; // this one is a leaf; already final.
		MBVHNode& node = this->mbvhNode[i];
		const typename BVH::BVHNode& orig = original.bvhNode[srcNode[i]];
		node.aabbMin = orig.aabbMin, node.aabbMax = orig.aabbMax;
		node.triCount = 0, node.childCount = 0;
		// hand out the M child slots according to the tabulated distribution
		Index task[M + 2];
		int32_t taskSlots[M + 2], tasks = 1;
		task[0] = srcNode[i], taskSlots[0] = M;
		while (tasks > 0)
		{
			const Index c = task[--tasks];
			const int32_t slots = taskSlots[tasks];
			const typename BVH::BVHNode& child = original.bvhNode[c];
			if (slots > 1 && !child.isLeaf() && dist[(size_t)c * M + slots - 1] != 0)
			{
				const int32_t k = dist[(size_t)c * M + slots - 1];
				task[tasks] = child.leftFirst + 1, taskSlots[tasks++] = slots - k; // right first,
				task[tasks] = child.leftFirst, taskSlots[tasks++] = k; // so that left pops first.
				continue;
			}
			const Index childIdx = usedNodes++;
			node.child[node.childCount++] = childIdx;
			if (flag[c] & ASLEAF)
			{
				MBVHNode& leaf = this->mbvhNode[childIdx];
				leaf.aabbMin = child.aabbMin, leaf.aabbMax = child.aabbMax;
				leaf.firstTri = subFirst[c], leaf.triCount = subCount[c], leaf.childCount = 0;
				srcNode[childIdx] = NOTANODE;
			}
			else srcNode[childIdx] = c;
		}
	}
	// finalize
	AlignedFree( cost ), AlignedFree( dist ), AlignedFree( subFirst );
	AlignedFree( subCount ), AlignedFree( srcNode ), AlignedFree( flag );
	this->may_have_holes = false; // the collapse produces a continuous list of nodes.
}

#else

// Old collapse method, left in for reference.
template <int M, typename Float, typename Index> void MBVH<M, Float, Index>::ConvertFrom( const BVH& original, bool compact )
{
	// get a copy of the original bvh
	if (&original != &bvh) ownBVH = false; // bvh isn't ours; don't delete in destructor.
	bvh = original;
	// allocate space
	Index spaceNeeded = compact ? original.usedNodes : original.allocatedNodes;
	constexpr bool M8 = M == 8;
	if (M8) spaceNeeded += original.usedNodes >> 1; // cwbvh / SplitLeafs
	if (allocatedNodes < spaceNeeded)
	{
		AlignedFree( mbvhNode );
		mbvhNode = (MBVHNode*)AlignedAlloc( spaceNeeded * sizeof( MBVHNode ) );
		allocatedNodes = spaceNeeded;
	}
	memset( mbvhNode, 0, sizeof( MBVHNode ) * spaceNeeded );
	CopyBasePropertiesFrom( original );
	// create an mbvh node for each bvh2 node
	for (Index i = 0; i < original.usedNodes; i++) if (i != 1)
	{
		typename BVH::BVHNode& orig = original.bvhNode[i];
		MBVHNode& node = this->mbvhNode[i];
		node.aabbMin = orig.aabbMin, node.aabbMax = orig.aabbMax;
		if (orig.isLeaf()) node.triCount = orig.triCount, node.firstTri = orig.leftFirst;
		else node.child[0] = orig.leftFirst, node.child[1] = orig.leftFirst + 1, node.childCount = 2;
	}
	// collapse
	Index stack[TINYBVH_STACK_SIZE], stackPtr = 0, nodeIdx = 0; // i.e., root node
	while (1)
	{
		MBVHNode& node = this->mbvhNode[nodeIdx];
		while (node.childCount < M)
		{
			int32_t bestChild = -1;
			Float bestChildSA = 0;
			for (Index i = 0; i < node.childCount; i++)
			{
				// see if we can adopt child i
				const MBVHNode& child = this->mbvhNode[node.child[i]];
				if (!child.isLeaf() && node.childCount - 1 + child.childCount <= M)
				{
					const Float childSA = SA( child.aabbMin, child.aabbMax );
					if (childSA > bestChildSA) bestChild = i, bestChildSA = childSA;
				}
			}
			if (bestChild == -1) break; // could not adopt
			const MBVHNode& child = this->mbvhNode[node.child[bestChild]];
			node.child[bestChild] = child.child[0];
			for (Index i = 1; i < child.childCount; i++)
				node.child[node.childCount++] = child.child[i];
		}
		// we're done with the node; proceed with the children.
		for (Index i = 0; i < node.childCount; i++)
		{
			const Index childIdx = node.child[i];
			const MBVHNode& child = this->mbvhNode[childIdx];
			if (!child.isLeaf()) stack[stackPtr++] = childIdx;
		}
		if (stackPtr == 0) break;
		nodeIdx = stack[--stackPtr];
	}
	// special case where root is leaf: add extra level - cwbvh needs this.
	MBVHNode& root = this->mbvhNode[0];
	if (root.isLeaf())
	{
		mbvhNode[1] = root;
		root.childCount = 1;
		root.child[0] = 1;
		root.triCount = 0;
	}
	// finalize
	usedNodes = original.usedNodes;
	this->may_have_holes = true;
}

#endif

// BVH4_GPU implementation
// ----------------------------------------------------------------------------

template <typename Float, typename Index> BVH4_GPU<Float, Index>::BVH4_GPU( BVH4_GPU&& other )
{
	*this = other;
	other.bvh4Data = 0;
}

template <typename Float, typename Index> BVH4_GPU<Float, Index>::~BVH4_GPU()
{
	if (!ownBVH4) bvh4 = MBVH<4, Float, Index>(); // clear out pointers we don't own.
	AlignedFree( bvh4Data );
}

// forwarders
template <typename Float, typename Index> void BVH4_GPU<Float, Index>::Build( const bvhvec4* v, const Index p ) { Build( Slice( v, p * 3, sizeof( bvhvec4 ) ) ); }
template <typename Float, typename Index> void BVH4_GPU<Float, Index>::Build( const bvhvec4* v, const uint32_t* i, const Index p ) { Build( Slice( v, p * 3, sizeof( bvhvec4 ) ), i, p ); }
template <typename Float, typename Index> void BVH4_GPU<Float, Index>::Build( const Slice& v ) { Build( v, 0, 0 ); }
template <typename Float, typename Index> void BVH4_GPU<Float, Index>::BuildHQ( const bvhvec4* v, const Index p ) { BuildHQ( Slice( v, p * 3, sizeof( bvhvec4 ) ) ); }
template <typename Float, typename Index> void BVH4_GPU<Float, Index>::BuildHQ( const bvhvec4* v, const uint32_t* i, const Index p ) { BuildHQ( Slice( v, p * 3, sizeof( bvhvec4 ) ), i, p ); }
template <typename Float, typename Index> void BVH4_GPU<Float, Index>::BuildHQ( const Slice& v ) { BuildHQ( v, 0, 0 ); }
template <typename Float, typename Index> void BVH4_GPU<Float, Index>::BuildHQ( const Slice& v, const uint32_t* i, Index p ) { settings.useSpatialSplits = true; Build( v, i, p ); }

template <typename Float, typename Index> void BVH4_GPU<Float, Index>::Build( const Slice& vertices, const uint32_t* indices, Index prims )
{
	// propagate settings for this layout to the underlying layout
	bvh4.context = bvh4.bvh.context = context, bvh4.settings = bvh4.bvh.settings = settings;
	bvh4.c_int = bvh4.bvh.c_int = c_int, bvh4.c_trav = bvh4.bvh.c_trav = c_trav;
	// build underlying layout
	bvh4.Build( vertices, indices, prims );
	// convert to BVH4_GPU layout
	ConvertFrom( bvh4, true );
}

template <typename Float, typename Index> void BVH4_GPU<Float, Index>::Optimize( const uint32_t iterations, bool extreme )
{
	bvh4.Optimize( iterations, extreme );
	ConvertFrom( bvh4, true );
}

template <typename Float, typename Index> void BVH4_GPU<Float, Index>::ConvertFrom( const MBVH<4, Float, Index>& original, bool compact )
{
	// get a copy of the original bvh4
	if (&original != &bvh4) ownBVH4 = false; // bvh isn't ours; don't delete in destructor.
	bvh4 = original;
	// Convert a 4-wide BVH to a format suitable for GPU traversal. Layout:
	// offs 0:   aabbMin (12 bytes), 4x quantized child xmin (4 bytes)
	// offs 16:  aabbMax (12 bytes), 4x quantized child xmax (4 bytes)
	// offs 32:  4x child ymin, then ymax, zmax, zmax (total 16 bytes)
	// offs 48:  4x child node info: leaf if MSB set.
	//           Leaf: 15 bits for tri count, 16 for offset
	//           Interior: 32 bits for position of child node.
	// Triangle data ('by value') immediately follows each leaf node.
	uint32_t blocksNeeded = compact ? (bvh4.usedNodes * 4) : (bvh4.allocatedNodes * 4); // here, 'block' is 16 bytes.
	blocksNeeded += 6 * bvh4.triCount; // this layout stores tris in the same buffer.
	if (allocatedBlocks < blocksNeeded)
	{
		AlignedFree( bvh4Data );
		bvh4Data = (bvhvec4*)AlignedAlloc( blocksNeeded * 16 );
		allocatedBlocks = blocksNeeded;
	}
	memset( bvh4Data, 0, 16 * blocksNeeded );
	CopyBasePropertiesFrom( bvh4 );
	// start conversion
	uint32_t nodeIdx = 0, newAlt4Ptr = 0, stack[TINYBVH_STACK_SIZE], stackPtr = 0, retValPos = 0;
	while (1)
	{
		const typename MBVH<4, Float, Index>::MBVHNode& orig = bvh4.mbvhNode[nodeIdx];
		// convert BVH4 node - must be an interior node.
		assert( !orig.isLeaf() );
		bvhvec4* nodeBase = bvh4Data + newAlt4Ptr;
		uint32_t baseAlt4Ptr = newAlt4Ptr;
		newAlt4Ptr += 4;
		nodeBase[0] = bvhvec4( orig.aabbMin, 0 );
		nodeBase[1] = bvhvec4( (orig.aabbMax - orig.aabbMin) * (1.0f / 255.0f), 0 );
		typename MBVH<4, Float, Index>::MBVHNode* childNode[4] = {
			&bvh4.mbvhNode[orig.child[0]], &bvh4.mbvhNode[orig.child[1]],
			&bvh4.mbvhNode[orig.child[2]], &bvh4.mbvhNode[orig.child[3]]
		};
		// start with leaf child node conversion
		uint32_t childInfo[4] = { 0, 0, 0, 0 }; // will store in final fields later
		for (int32_t i = 0; i < 4; i++) if (childNode[i]->isLeaf())
		{
			childInfo[i] = newAlt4Ptr - baseAlt4Ptr;
			childInfo[i] |= childNode[i]->triCount << 16;
			childInfo[i] |= 0x80000000;
			for (uint32_t j = 0; j < childNode[i]->triCount; j++)
			{
				uint32_t t = bvh4.bvh.primIdx[childNode[i]->firstTri + j];
				uint32_t ti0, ti1, ti2;
				if (bvh4.bvh.vertIdx)
					ti0 = bvh4.bvh.vertIdx[t * 3],
					ti1 = bvh4.bvh.vertIdx[t * 3 + 1],
					ti2 = bvh4.bvh.vertIdx[t * 3 + 2];
				else
					ti0 = t * 3, ti1 = t * 3 + 1, ti2 = t * 3 + 2;
			#ifdef BVH4_GPU_COMPRESSED_TRIS
				PrecomputeTriangle( bvh4.bvh.verts, ti0, ti1, ti2, &bvh4Data[newAlt4Ptr] );
				bvh4Data[newAlt4Ptr + 3] = bvhvec4( 0, 0, 0, tinybvh_as_float( t ) );
				newAlt4Ptr += 4;
			#else
				bvhvec4 v0 = bvh4.bvh.verts[ti0];
				bvh4Data[newAlt4Ptr + 1] = bvh4.bvh.verts[ti1] - v0;
				bvh4Data[newAlt4Ptr + 2] = bvh4.bvh.verts[ti2] - v0;
				v0.w = tinybvh_as_float( t );
				bvh4Data[newAlt4Ptr + 0] = v0;
				newAlt4Ptr += 3;
			#endif
			}
		}
		// process interior nodes
		for (int32_t i = 0; i < 4; i++) if (!childNode[i]->isLeaf())
		{
			// childInfo[i] = node.child[i] == 0 ? 0 : GPUFormatBVH4( node.child[i] );
			if (orig.child[i] == 0) childInfo[i] = 0; else
			{
				stack[stackPtr++] = (uint32_t)(((const char*)&nodeBase[3] - (const char*)bvh4Data) / 4 + i);
				stack[stackPtr++] = orig.child[i];
			}
		}
		// store child node bounds, quantized
		const Vec3 extent = orig.aabbMax - orig.aabbMin;
		Vec3 scale;
		scale.x = extent.x > 1e-10f ? (254.999f / extent.x) : 0;
		scale.y = extent.y > 1e-10f ? (254.999f / extent.y) : 0;
		scale.z = extent.z > 1e-10f ? (254.999f / extent.z) : 0;
		uint8_t* slot0 = (uint8_t*)&nodeBase[0] + 12;	// 4 chars
		uint8_t* slot1 = (uint8_t*)&nodeBase[1] + 12;	// 4 chars
		uint8_t* slot2 = (uint8_t*)&nodeBase[2];		// 16 chars
		if (orig.child[0])
		{
			const Vec3 relBMin = childNode[0]->aabbMin - orig.aabbMin, relBMax = childNode[0]->aabbMax - orig.aabbMin;
			slot0[0] = (uint8_t)floorf( relBMin.x * scale.x ), slot1[0] = (uint8_t)ceilf( relBMax.x * scale.x );
			slot2[0] = (uint8_t)floorf( relBMin.y * scale.y ), slot2[4] = (uint8_t)ceilf( relBMax.y * scale.y );
			slot2[8] = (uint8_t)floorf( relBMin.z * scale.z ), slot2[12] = (uint8_t)ceilf( relBMax.z * scale.z );
		}
		if (orig.child[1])
		{
			const Vec3 relBMin = childNode[1]->aabbMin - orig.aabbMin, relBMax = childNode[1]->aabbMax - orig.aabbMin;
			slot0[1] = (uint8_t)floorf( relBMin.x * scale.x ), slot1[1] = (uint8_t)ceilf( relBMax.x * scale.x );
			slot2[1] = (uint8_t)floorf( relBMin.y * scale.y ), slot2[5] = (uint8_t)ceilf( relBMax.y * scale.y );
			slot2[9] = (uint8_t)floorf( relBMin.z * scale.z ), slot2[13] = (uint8_t)ceilf( relBMax.z * scale.z );
		}
		if (orig.child[2])
		{
			const Vec3 relBMin = childNode[2]->aabbMin - orig.aabbMin, relBMax = childNode[2]->aabbMax - orig.aabbMin;
			slot0[2] = (uint8_t)floorf( relBMin.x * scale.x ), slot1[2] = (uint8_t)ceilf( relBMax.x * scale.x );
			slot2[2] = (uint8_t)floorf( relBMin.y * scale.y ), slot2[6] = (uint8_t)ceilf( relBMax.y * scale.y );
			slot2[10] = (uint8_t)floorf( relBMin.z * scale.z ), slot2[14] = (uint8_t)ceilf( relBMax.z * scale.z );
		}
		if (orig.child[3])
		{
			const Vec3 relBMin = childNode[3]->aabbMin - orig.aabbMin, relBMax = childNode[3]->aabbMax - orig.aabbMin;
			slot0[3] = (uint8_t)floorf( relBMin.x * scale.x ), slot1[3] = (uint8_t)ceilf( relBMax.x * scale.x );
			slot2[3] = (uint8_t)floorf( relBMin.y * scale.y ), slot2[7] = (uint8_t)ceilf( relBMax.y * scale.y );
			slot2[11] = (uint8_t)floorf( relBMin.z * scale.z ), slot2[15] = (uint8_t)ceilf( relBMax.z * scale.z );
		}
		// finalize node
		nodeBase[3] = bvhvec4(
			tinybvh_as_float( childInfo[0] ), tinybvh_as_float( childInfo[1] ),
			tinybvh_as_float( childInfo[2] ), tinybvh_as_float( childInfo[3] )
		);
		// pop new work from the stack
		if (retValPos > 0) tinybvh_setlane_u( bvh4Data, retValPos, baseAlt4Ptr );
		if (stackPtr == 0) break;
		nodeIdx = stack[--stackPtr];
		retValPos = stack[--stackPtr];
	}
	usedBlocks = newAlt4Ptr;
}

// IntersectAlt4Nodes. For testing the converted data only; not efficient.
// This code replicates how traversal on GPU happens.
#define SWAP(A,B,C,D) tmp=A,A=B,B=tmp,tmp2=C,C=D,D=tmp2;
struct uchar4 { uint8_t x, y, z, w; };
static uchar4 as_uchar4( const float v ) { return tinybvh_bitcast<uchar4>( v ); }
static uint32_t as_uint( const float v ) { uint32_t t; memcpy( &t, &v, sizeof( t ) ); return t; }
template <typename Float, typename Index> int32_t BVH4_GPU<Float, Index>::Intersect( Ray& ray ) const
{
	// traverse a blas
	VALIDATE_RAY( ray );
	uint32_t offset = 0, stack[TINYBVH_STACK_SIZE], stackPtr = 0, tmp2 /* for SWAP macro */;
	float cost = 0;
	while (1)
	{
		cost += c_trav;
		// fetch the node
		const bvhvec4 data0 = bvh4Data[offset + 0], data1 = bvh4Data[offset + 1];
		const bvhvec4 data2 = bvh4Data[offset + 2], data3 = bvh4Data[offset + 3];
		// extract aabb
		const Vec3 bmin = data0, extent = data1; // pre-scaled by 1/255
		// reconstruct conservative child aabbs
		const uchar4 d0 = as_uchar4( data0.w ), d1 = as_uchar4( data1.w ), d2 = as_uchar4( data2.x );
		const uchar4 d3 = as_uchar4( data2.y ), d4 = as_uchar4( data2.z ), d5 = as_uchar4( data2.w );
		const Vec3 c0min = bmin + extent * Vec3( d0.x, d2.x, d4.x ), c0max = bmin + extent * Vec3( d1.x, d3.x, d5.x );
		const Vec3 c1min = bmin + extent * Vec3( d0.y, d2.y, d4.y ), c1max = bmin + extent * Vec3( d1.y, d3.y, d5.y );
		const Vec3 c2min = bmin + extent * Vec3( d0.z, d2.z, d4.z ), c2max = bmin + extent * Vec3( d1.z, d3.z, d5.z );
		const Vec3 c3min = bmin + extent * Vec3( d0.w, d2.w, d4.w ), c3max = bmin + extent * Vec3( d1.w, d3.w, d5.w );
		// intersect child aabbs
		const Vec3 t1a = (c0min - ray.O) * ray.rD, t2a = (c0max - ray.O) * ray.rD;
		const Vec3 t1b = (c1min - ray.O) * ray.rD, t2b = (c1max - ray.O) * ray.rD;
		const Vec3 t1c = (c2min - ray.O) * ray.rD, t2c = (c2max - ray.O) * ray.rD;
		const Vec3 t1d = (c3min - ray.O) * ray.rD, t2d = (c3max - ray.O) * ray.rD;
		const Vec3 minta = tinybvh_min( t1a, t2a ), maxta = tinybvh_max( t1a, t2a );
		const Vec3 mintb = tinybvh_min( t1b, t2b ), maxtb = tinybvh_max( t1b, t2b );
		const Vec3 mintc = tinybvh_min( t1c, t2c ), maxtc = tinybvh_max( t1c, t2c );
		const Vec3 mintd = tinybvh_min( t1d, t2d ), maxtd = tinybvh_max( t1d, t2d );
		const float tmina = tinybvh_max( tinybvh_max( tinybvh_max( minta.x, minta.y ), minta.z ), 0.0f );
		const float tminb = tinybvh_max( tinybvh_max( tinybvh_max( mintb.x, mintb.y ), mintb.z ), 0.0f );
		const float tminc = tinybvh_max( tinybvh_max( tinybvh_max( mintc.x, mintc.y ), mintc.z ), 0.0f );
		const float tmind = tinybvh_max( tinybvh_max( tinybvh_max( mintd.x, mintd.y ), mintd.z ), 0.0f );
		const float tmaxa = tinybvh_min( tinybvh_min( tinybvh_min( maxta.x, maxta.y ), maxta.z ), ray.hit.t );
		const float tmaxb = tinybvh_min( tinybvh_min( tinybvh_min( maxtb.x, maxtb.y ), maxtb.z ), ray.hit.t );
		const float tmaxc = tinybvh_min( tinybvh_min( tinybvh_min( maxtc.x, maxtc.y ), maxtc.z ), ray.hit.t );
		const float tmaxd = tinybvh_min( tinybvh_min( tinybvh_min( maxtd.x, maxtd.y ), maxtd.z ), ray.hit.t );
		float dist0 = tmina > tmaxa ? bvh_far<Float> : tmina, dist1 = tminb > tmaxb ? bvh_far<Float> : tminb;
		float dist2 = tminc > tmaxc ? bvh_far<Float> : tminc, dist3 = tmind > tmaxd ? bvh_far<Float> : tmind, tmp;
		// get child node info fields
		uint32_t c0info = as_uint( data3.x ), c1info = as_uint( data3.y );
		uint32_t c2info = as_uint( data3.z ), c3info = as_uint( data3.w );
		if (dist0 < dist2) SWAP( dist0, dist2, c0info, c2info );
		if (dist1 < dist3) SWAP( dist1, dist3, c1info, c3info );
		if (dist0 < dist1) SWAP( dist0, dist1, c0info, c1info );
		if (dist2 < dist3) SWAP( dist2, dist3, c2info, c3info );
		if (dist1 < dist2) SWAP( dist1, dist2, c1info, c2info );
		// process results, starting with farthest child, so nearest ends on top of stack
		uint32_t nextNode = 0;
		uint32_t leaf[4] = { 0, 0, 0, 0 }, leafs = 0;
		if (dist0 < bvh_far<Float>)
		{
			if (c0info & 0x80000000) leaf[leafs++] = c0info; else if (c0info) stack[stackPtr++] = c0info;
		}
		if (dist1 < bvh_far<Float>)
		{
			if (c1info & 0x80000000) leaf[leafs++] = c1info; else if (c1info) stack[stackPtr++] = c1info;
		}
		if (dist2 < bvh_far<Float>)
		{
			if (c2info & 0x80000000) leaf[leafs++] = c2info; else if (c2info) stack[stackPtr++] = c2info;
		}
		if (dist3 < bvh_far<Float>)
		{
			if (c3info & 0x80000000) leaf[leafs++] = c3info; else if (c3info) stack[stackPtr++] = c3info;
		}
		// process encountered leafs, if any
		for (uint32_t i = 0; i < leafs; i++)
		{
			const uint32_t N = (leaf[i] >> 16) & 0x7fff;
			uint32_t triStart = offset + (leaf[i] & 0xffff);
			for (uint32_t j = 0; j < N; j++, triStart += 3)
			{
				cost += c_int;
				const Vec3 e2 = Vec3( bvh4Data[triStart + 2] );
				const Vec3 e1 = Vec3( bvh4Data[triStart + 1] );
				const Vec3 v0 = bvh4Data[triStart + 0];
				MOLLER_TRUMBORE_TEST( ray.hit.t, continue );
				ray.hit.t = t, ray.hit.u = u, ray.hit.v = v;
				ray.hit.prim = as_uint( bvh4Data[triStart + 0].w );
			}
		}
		// continue with nearest node or first node on the stack
		if (nextNode) offset = nextNode; else
		{
			if (!stackPtr) break;
			offset = stack[--stackPtr];
		}
	}
	return (int32_t)cost; // cast to not break interface.
}

// BVH4_CPU implementation
// ----------------------------------------------------------------------------

template <typename Float, typename Index> BVH4_CPU<Float, Index>::BVH4_CPU( BVH4_CPU&& other )
{
	*this = other;
	other.bvh4Data = 0;
}

template <typename Float, typename Index> BVH4_CPU<Float, Index>::~BVH4_CPU()
{
	if (!ownBVH4) bvh4 = MBVH<4, Float, Index>(); // clear out pointers we don't own.
	AlignedFree( bvh4Data );
}

// forwarders
template <typename Float, typename Index> void BVH4_CPU<Float, Index>::Build( const Vertex* v, const uint32_t* i, const Index p ) { Build( Slice( v, p * 3, sizeof( Vertex ) ), i, p ); }
template <typename Float, typename Index> void BVH4_CPU<Float, Index>::Build( const Vertex* v, const Index p ) { Build( Slice( v, p * 3, sizeof( Vertex ) ) ); }
template <typename Float, typename Index> void BVH4_CPU<Float, Index>::Build( const Slice& v ) { Build( v, 0, 0 ); }
template <typename Float, typename Index> void BVH4_CPU<Float, Index>::BuildHQ( const Vertex* v, const Index p ) { BuildHQ( Slice( v, p * 3, sizeof( Vertex ) ) ); }
template <typename Float, typename Index> void BVH4_CPU<Float, Index>::BuildHQ( const Vertex* v, const uint32_t* i, const Index p ) { BuildHQ( Slice( v, p * 3, sizeof( Vertex ) ), i, p ); }
template <typename Float, typename Index> void BVH4_CPU<Float, Index>::BuildHQ( const Slice& v ) { BuildHQ( v, 0, 0 ); }
template <typename Float, typename Index> void BVH4_CPU<Float, Index>::BuildHQ( const Slice& v, const uint32_t* i, Index p ) { settings.useSpatialSplits = true; Build( v, i, p ); }

template <typename Float, typename Index> void BVH4_CPU<Float, Index>::Build( const Slice& vertices, const uint32_t* indices, Index prims )
{
	// propagate settings for this layout to the underlying layout
	bvh4.bvh.context = bvh4.context = context, bvh4.bvh.settings = bvh4.settings = settings;
	bvh4.bvh.c_int = bvh4.c_int = c_int, bvh4.bvh.c_trav = bvh4.c_trav = c_trav;
	// build underlying layout
	bvh4.bvh.Build( vertices, indices, prims );
	// convert to BVH4_CPU layout
	ConvertFrom( bvh4 );
}

template <typename Float, typename Index> void BVH4_CPU<Float, Index>::Save( const char* fileName )
{
	std::fstream s{ fileName, s.binary | s.out };
	const uint32_t header = this->CacheHeader();
	s.write( (char*)&header, sizeof( uint32_t ) );
	s.write( (char*)&triCount, sizeof( Index ) );
	s.write( (char*)this, sizeof( BVH4_CPU ) );
	s.write( (char*)bvh4Data, usedBlocks * 64 );
}

template <typename Float, typename Index> bool BVH4_CPU<Float, Index>::Load( const char* fileName, const Index expectedTris )
{
	// open file and check contents
	std::fstream s{ fileName, s.binary | s.in };
	if (!s) return false;
	BVHContext tmp = context;
	uint32_t header;
	Index fileTriCount;
	s.read( (char*)&header, sizeof( uint32_t ) );
	if (header != this->CacheHeader()) return false;
	s.read( (char*)&fileTriCount, sizeof( Index ) );
	if (fileTriCount != expectedTris) return false;
	// all checks passed; safe to overwrite *this
	s.read( (char*)this, sizeof( BVH4_CPU ) );
	context = tmp; // can't load context; function pointers will differ.
	bvh4Data = (CacheLine*)AlignedAlloc( usedBlocks * 64 );
	allocatedBlocks = usedBlocks;
	s.read( (char*)bvh4Data, usedBlocks * 64 );
	bvh4 = MBVH<4, Float, Index>();
	return true;
}

template <typename Float, typename Index> void BVH4_CPU<Float, Index>::Optimize( const uint32_t iterations, bool extreme )
{
	bvh4.Optimize( iterations, extreme );
	ConvertFrom( bvh4 );
}

template <typename Float, typename Index> void BVH4_CPU<Float, Index>::Refit()
{
	bvh4.Refit();
	ConvertFrom( bvh4 );
}

template <typename Float, typename Index> Float BVH4_CPU<Float, Index>::SAHCost( const Index nodeIdx ) const
{
	return bvh4.SAHCost( nodeIdx );
}

#define SORT(a,b) { if (tinybvh_as_float( idist[a] ) < tinybvh_as_float( idist[b] )) \
	{ const uint32_t h = idist[a]; idist[a] = idist[b], idist[b] = h; } }

template <typename Float, typename Index> void BVH4_CPU<Float, Index>::ConvertFrom( MBVH<4, Float, Index>& original )
{
	// Note: identical to BVH8_CPU version, just with fewer lanes.
	// get a copy of the input bvh4
	if (&original != &bvh4) ownBVH4 = false; // bvh isn't ours; don't delete in destructor.
	bvh4 = original;
	// prepare input bvh4
	Index firstIdx = 0;
	bvh4.bvh.CombineLeafs( 4, firstIdx, 0 );
	bvh4.bvh.SplitLeafs( 4 );
	bvh4.leafPrimLimit = 4, bvh4.l_quads = l_quads; // leafs in this layout hold 4 prims
	bvh4.c_int = c_int, bvh4.c_trav = c_trav;
	bvh4.ConvertFrom( bvh4.bvh, true );
	// allocate if needed
	const Index nodesNeeded = bvh4.usedNodes, leafsNeeded = bvh4.LeafCount();
	const Index blocksNeeded = nodesNeeded * (sizeof( BVHNode ) / 64) + leafsNeeded * (sizeof( BVHTri4Leaf ) / 64); // here, block = cacheline.
	// Child slots store the block index in 29 bits; refuse to build beyond that.
	BVH_FATAL_ERROR_IF( blocksNeeded > 0x1fffffff, "BVH4_CPU::ConvertFrom, BVH does not fit in 29-bit block indices." );
	if (allocatedBlocks < blocksNeeded)
	{
		AlignedFree( bvh4Data );
		void* (*allocator)(size_t, void*) = malloc64;
		bvh4Data = (CacheLine*)allocator( blocksNeeded * 64, 0 );
		allocatedBlocks = (uint32_t)blocksNeeded;
	}
	CopyBasePropertiesFrom( bvh4 );
	// start conversion
	uint32_t newBlockPtr = 0, stackPtr = 0;
	Index nodeIdx = 0, stack[TINYBVH_STACK_SIZE];
	while (1)
	{
		const typename MBVH<4, Float, Index>::MBVHNode& orig = bvh4.mbvhNode[nodeIdx];
		BVHNode* newNode = (BVHNode*)(bvh4Data + newBlockPtr);
		newBlockPtr += sizeof( BVHNode ) / 64;
		memset( newNode, 0, sizeof( BVHNode ) );
		// calculate the permutation offsets for the node
		for (uint32_t q = 0; q < 8; q++)
		{
			const Vec3 D( q & 1 ? 1.0f : -1.0f, q & 2 ? 1.0f : -1.0f, q & 4 ? 1.0f : -1.0f );
			uint32_t idist[4];
			for (int i = 0; i < 4; i++) if (orig.child[i] == 0)
				idist[i] = (tinybvh_as_uint( 1e30f ) & 0xfffffffc) + i;
			else
			{
				const typename MBVH<4, Float, Index>::MBVHNode& c = bvh4.mbvhNode[orig.child[i]];
				const Vec3 p( q & 1 ? c.aabbMin.x : c.aabbMax.x, q & 2 ? c.aabbMin.y : c.aabbMax.y, q & 4 ? c.aabbMin.z : c.aabbMax.z );
				idist[i] = (tinybvh_as_uint( (float)tinybvh_dot( D, p ) ) & 0xfffffff8) + i;
			}
			// apply sorting network - https://bertdobbelaere.github.io/sorting_networks.html#N4L5D3
			SORT( 0, 2 ); SORT( 1, 3 ); SORT( 0, 1 ); SORT( 2, 3 ); SORT( 1, 2 );
			for (int i = 0; i < 4; i++) newNode->perm[i] += (idist[i] & 3) << (q * 2);
		}
		// fill remaining fields
		int32_t cidx = 0;
		for (int32_t i = 0; i < 4; i++) if (orig.child[i])
		{
			const typename MBVH<4, Float, Index>::MBVHNode& child = bvh4.mbvhNode[orig.child[i]];
			newNode->xmin[cidx] = child.aabbMin.x, newNode->xmax[cidx] = child.aabbMax.x;
			newNode->ymin[cidx] = child.aabbMin.y, newNode->ymax[cidx] = child.aabbMax.y;
			newNode->zmin[cidx] = child.aabbMin.z, newNode->zmax[cidx] = child.aabbMax.z;
			if (child.isLeaf())
			{
				// emit leaf node: group of up to 4 triangles in AoS format.
				newNode->child[cidx] = newBlockPtr + LEAF_BIT;
				BVHTri4Leaf* leaf = (BVHTri4Leaf*)(bvh4Data + newBlockPtr);
				newBlockPtr += sizeof( BVHTri4Leaf ) / 64;
				for (uint32_t i0, i1, i2, l = 0; l < 4; l++)
				{
					const Index primIdx = bvh4.bvh.primIdx[child.firstTri + tinybvh_min( (Index)l, child.triCount - 1 )];
					GET_PRIM_INDICES_I0_I1_I2( bvh4.bvh, primIdx );
					const Vertex v0 = bvh4.bvh.verts[i0], e1 = bvh4.bvh.verts[i1] - v0, e2 = bvh4.bvh.verts[i2] - v0;
					leaf->SetData( v0, e1, e2, primIdx, l );
				}
			}
			else
			{
				const size_t slot = ((const char*)newNode->child - (const char*)bvh4Data) / 4 + cidx;
				stack[stackPtr++] = (Index)slot;
				stack[stackPtr++] = orig.child[i];
			}
			cidx++;
		}
		for (; cidx < 4; cidx++)
			newNode->xmin[cidx] = bvh_far<Float>, newNode->xmax[cidx] = -bvh_far<Float>,
			newNode->ymin[cidx] = bvh_far<Float>, newNode->ymax[cidx] = -bvh_far<Float>,
			newNode->zmin[cidx] = bvh_far<Float>, newNode->zmax[cidx] = -bvh_far<Float>,
			newNode->child[cidx] |= EMPTY_BIT;
		// pop next task
		if (!stackPtr) break;
		nodeIdx = stack[--stackPtr];
		const Index offset = stack[--stackPtr];
		tinybvh_setlane_u( bvh4Data, (size_t)offset, newBlockPtr );
	}
	usedBlocks = newBlockPtr;
}

// BVH8_CPU implementation
// ----------------------------------------------------------------------------

template <typename Float, typename Index> BVH8_CPU<Float, Index>::BVH8_CPU( BVH8_CPU&& other )
{
	*this = other;
	other.bvh8Data = 0;
}

template <typename Float, typename Index> BVH8_CPU<Float, Index>::~BVH8_CPU()
{
	if (!ownBVH8) bvh8 = MBVH<8, Float, Index>(); // clear out pointers we don't own.
	AlignedFree( bvh8Data );
}

// forwarders
template <typename Float, typename Index> void BVH8_CPU<Float, Index>::Build( const Vertex* v, const Index p ) { Build( Slice( v, p * 3, sizeof( Vertex ) ) ); }
template <typename Float, typename Index> void BVH8_CPU<Float, Index>::Build( const Slice& vertices ) { Build( vertices, 0, 0 ); }
template <typename Float, typename Index> void BVH8_CPU<Float, Index>::Build( const Vertex* v, const uint32_t* i, const Index p ) { Build( Slice( v, p * 3, sizeof( Vertex ) ), i, p ); }
template <typename Float, typename Index> void BVH8_CPU<Float, Index>::BuildHQ( const Vertex* v, const Index p ) { BuildHQ( Slice( v, p * 3, sizeof( Vertex ) ) ); }
template <typename Float, typename Index> void BVH8_CPU<Float, Index>::BuildHQ( const Slice& vertices ) { BuildHQ( vertices, 0, 0 ); }
template <typename Float, typename Index> void BVH8_CPU<Float, Index>::BuildHQ( const Vertex* v, const uint32_t* i, const Index p ) { BuildHQ( Slice( v, p * 3, sizeof( Vertex ) ), i, p ); }
template <typename Float, typename Index> void BVH8_CPU<Float, Index>::BuildHQ( const Slice& v, const uint32_t* i, Index p ) { settings.useSpatialSplits = true; Build( v, i, p ); }

template <typename Float, typename Index> void BVH8_CPU<Float, Index>::Build( const Slice& vertices, const uint32_t* indices, Index prims )
{
	// propagate settings for this layout to the underlying layout
	bvh8.bvh.context = bvh8.context = context, bvh8.bvh.settings = bvh8.settings = settings;
	bvh8.bvh.c_int = bvh8.c_int = c_int, bvh8.bvh.c_trav = bvh8.c_trav = c_trav;
	// build underlying layout
	bvh8.bvh.Build( vertices, indices, prims );
	bvh8.bvh.Compact();
	// convert to BVH4_CPU layout
	ConvertFrom( bvh8 );
}

template <typename Float, typename Index> void BVH8_CPU<Float, Index>::Save( const char* fileName )
{
	std::fstream s{ fileName, s.binary | s.out };
	const uint32_t header = this->CacheHeader();
	s.write( (char*)&header, sizeof( uint32_t ) );
	s.write( (char*)&triCount, sizeof( Index ) );
	s.write( (char*)this, sizeof( BVH8_CPU ) );
	s.write( (char*)bvh8Data, usedBlocks * 64 );
}

template <typename Float, typename Index> bool BVH8_CPU<Float, Index>::Load( const char* fileName, const Index expectedTris )
{
	// open file and check contents
	std::fstream s{ fileName, s.binary | s.in };
	if (!s) return false;
	BVHContext tmp = context;
	uint32_t header;
	Index fileTriCount;
	s.read( (char*)&header, sizeof( uint32_t ) );
	if (header != this->CacheHeader()) return false;
	s.read( (char*)&fileTriCount, sizeof( Index ) );
	if (fileTriCount != expectedTris) return false;
	// all checks passed; safe to overwrite *this
	s.read( (char*)this, sizeof( BVH8_CPU ) );
	context = tmp; // can't load context; function pointers will differ.
	bvh8Data = (CacheLine*)AlignedAlloc( usedBlocks * 64 );
	allocatedBlocks = usedBlocks;
	s.read( (char*)bvh8Data, usedBlocks * 64 );
	bvh8 = MBVH<8, Float, Index>();
	return true;
}

template <typename Float, typename Index> void BVH8_CPU<Float, Index>::Optimize( const uint32_t iterations, bool extreme )
{
	bvh8.Optimize( iterations, extreme );
	ConvertFrom( bvh8 );
}

template <typename Float, typename Index> void BVH8_CPU<Float, Index>::Refit()
{
	bvh8.Refit();
	ConvertFrom( bvh8 );
}

template <typename Float, typename Index> Float BVH8_CPU<Float, Index>::SAHCost( const Index nodeIdx ) const
{
	return bvh8.SAHCost( nodeIdx );
}

template <typename Float, typename Index> void BVH8_CPU<Float, Index>::ConvertFrom( MBVH<8, Float, Index>& original )
{
	// get a copy of the input bvh8
	if (&original != &bvh8) ownBVH8 = false; // bvh isn't ours; don't delete in destructor.
	bvh8 = original;
	// prepare input bvh8
	Index firstIdx = 0;
	bvh8.bvh.CombineLeafs( 4, firstIdx, 0 );
	bvh8.bvh.SplitLeafs( 4 );
	bvh8.leafPrimLimit = 4, bvh8.l_quads = l_quads; // leafs in this layout hold 4 prims
	bvh8.c_int = c_int, bvh8.c_trav = c_trav;
	bvh8.ConvertFrom( bvh8.bvh, true );
	// allocate if needed
	const Index nodesNeeded = bvh8.usedNodes, leafsNeeded = bvh8.LeafCount();
	Index blocksNeeded = nodesNeeded * (sizeof( BVHNode ) / 64) + leafsNeeded * (sizeof( BVHTri4Leaf ) / 64); // here, block = cacheline.
	// reserve one extra leaf at the end; a degenerate 'null leaf' that unused child slots point to.
	const uint32_t nullLeafBlock = (uint32_t)blocksNeeded;
	blocksNeeded += sizeof( BVHTri4Leaf ) / 64;
	// Child slots store the block index in 29 bits; refuse to build beyond that.
	BVH_FATAL_ERROR_IF( blocksNeeded > 0x1fffffff, "BVH8_CPU::ConvertFrom, BVH does not fit in 29-bit block indices." );
	if (allocatedBlocks < blocksNeeded)
	{
		AlignedFree( bvh8Data );
		void* (*allocator)(size_t, void*) = malloc64;
		bvh8Data = (CacheLine*)allocator( blocksNeeded * 64, 0 );
		allocatedBlocks = (uint32_t)blocksNeeded;
	}
	CopyBasePropertiesFrom( bvh8 );
	memset( bvh8Data + nullLeafBlock, 0, sizeof( BVHTri4Leaf ) );
	// start conversion
	uint32_t newBlockPtr = 0, stackPtr = 0;
	Index nodeIdx = 0, stack[TINYBVH_STACK_SIZE];
	while (1)
	{
		const typename MBVH<8, Float, Index>::MBVHNode& orig = bvh8.mbvhNode[nodeIdx];
		BVHNode* newNode = (BVHNode*)(bvh8Data + newBlockPtr);
		newBlockPtr += sizeof( BVHNode ) / 64;
		memset( newNode, 0, sizeof( BVHNode ) );
		// calculate the permutation offsets for the node
		for (uint32_t q = 0; q < 8; q++)
		{
			const Vec3 D( q & 1 ? 1.0f : -1.0f, q & 2 ? 1.0f : -1.0f, q & 4 ? 1.0f : -1.0f );
			uint32_t idist[8];
			for (int i = 0; i < 8; i++) if (orig.child[i] == 0)
				idist[i] = (tinybvh_as_uint( bvh_far<Float> ) & 0xfffffff8) + i;
			else
			{
				const typename MBVH<8, Float, Index>::MBVHNode& c = bvh8.mbvhNode[orig.child[i]];
				const Vec3 p( q & 1 ? c.aabbMin.x : c.aabbMax.x, q & 2 ? c.aabbMin.y : c.aabbMax.y, q & 4 ? c.aabbMin.z : c.aabbMax.z );
				idist[i] = (tinybvh_as_uint( (float)tinybvh_dot( D, p ) ) & 0xfffffff8) + i;
			}
			// apply sorting network - https://bertdobbelaere.github.io/sorting_networks.html#N8L19D6
			SORT( 0, 2 ); SORT( 1, 3 ); SORT( 4, 6 ); SORT( 5, 7 ); SORT( 0, 4 );
			SORT( 1, 5 ); SORT( 2, 6 ); SORT( 3, 7 ); SORT( 0, 1 ); SORT( 2, 3 );
			SORT( 4, 5 ); SORT( 6, 7 ); SORT( 2, 4 ); SORT( 3, 5 ); SORT( 1, 4 );
			SORT( 3, 6 ); SORT( 1, 2 ); SORT( 3, 4 ); SORT( 5, 6 );
			for (int i = 0; i < 8; i++) newNode->perm[i] += (idist[i] & 7) << (q * 3);
		}
		// fill remaining fields
		int32_t cidx = 0;
		for (int32_t i = 0; i < 8; i++) if (orig.child[i])
		{
			const typename MBVH<8, Float, Index>::MBVHNode& child = bvh8.mbvhNode[orig.child[i]];
			newNode->xmin[cidx] = child.aabbMin.x, newNode->xmax[cidx] = child.aabbMax.x;
			newNode->ymin[cidx] = child.aabbMin.y, newNode->ymax[cidx] = child.aabbMax.y;
			newNode->zmin[cidx] = child.aabbMin.z, newNode->zmax[cidx] = child.aabbMax.z;
			if (child.isLeaf())
			{
				// emit leaf node: group of up to 4 triangles in AoS format.
				newNode->child[cidx] = newBlockPtr + LEAF_BIT;
				BVHTri4Leaf* leaf = (BVHTri4Leaf*)(bvh8Data + newBlockPtr);
				newBlockPtr += sizeof( BVHTri4Leaf ) / 64;
				for (uint32_t i0, i1, i2, l = 0; l < 4; l++)
				{
					const Index primIdx = bvh8.bvh.primIdx[child.firstTri + tinybvh_min( (Index)l, child.triCount - 1 )];
					GET_PRIM_INDICES_I0_I1_I2( bvh8.bvh, primIdx );
					const Vertex v0 = bvh8.bvh.verts[i0], e1 = bvh8.bvh.verts[i1] - v0, e2 = bvh8.bvh.verts[i2] - v0;
					leaf->SetData( v0, e1, e2, primIdx, l );
				}
			}
			else
			{
				const size_t slot = ((const char*)newNode->child - (const char*)bvh8Data) / 4 + cidx;
				stack[stackPtr++] = (Index)slot;
				stack[stackPtr++] = orig.child[i];
			}
			cidx++;
		}
		for (; cidx < 8; cidx++)
		{
			newNode->xmin[cidx] = bvh_far<Float>, newNode->xmax[cidx] = -bvh_far<Float>;
			newNode->ymin[cidx] = bvh_far<Float>, newNode->ymax[cidx] = -bvh_far<Float>;
			newNode->zmin[cidx] = bvh_far<Float>, newNode->zmax[cidx] = -bvh_far<Float>;
			newNode->child[cidx] = nullLeafBlock + LEAF_BIT;
		}
		// pop next task
		if (!stackPtr) break;
		nodeIdx = stack[--stackPtr];
		const Index offset = stack[--stackPtr];
		tinybvh_setlane_u( bvh8Data, (size_t)offset, newBlockPtr );
	}
	BVH_FATAL_ERROR_IF( newBlockPtr > nullLeafBlock, "BVH8_CPU::ConvertFrom, block count underestimated." );
	usedBlocks = nullLeafBlock + sizeof( BVHTri4Leaf ) / 64;
}

// BVH8_CWBVH implementation
// ----------------------------------------------------------------------------

template <typename Float, typename Index> BVH8_CWBVH<Float, Index>::BVH8_CWBVH( BVH8_CWBVH&& other )
{
	*this = other;
	other.bvh8Data = 0;
	other.bvh8Tris = 0;
}

template <typename Float, typename Index> BVH8_CWBVH<Float, Index>::~BVH8_CWBVH()
{
	if (!ownBVH8) bvh8 = MBVH<8, Float, Index>(); // clear out pointers we don't own.
	AlignedFree( bvh8Data );
	AlignedFree( bvh8Tris );
}

// forwarders
template <typename Float, typename Index> void BVH8_CWBVH<Float, Index>::Build( const bvhvec4* v, const Index p ) { Build( Slice( v, p * 3, sizeof( bvhvec4 ) ) ); }
template <typename Float, typename Index> void BVH8_CWBVH<Float, Index>::Build( const Slice& v ) { Build( v, 0, 0 ); }
template <typename Float, typename Index> void BVH8_CWBVH<Float, Index>::Build( const bvhvec4* v, const uint32_t* i, const Index p ) { Build( Slice( v, p * 3, sizeof( bvhvec4 ) ), i, p ); }
template <typename Float, typename Index> void BVH8_CWBVH<Float, Index>::BuildHQ( const bvhvec4* v, const Index p ) { BuildHQ( Slice( v, p * 3, sizeof( bvhvec4 ) ) ); }
template <typename Float, typename Index> void BVH8_CWBVH<Float, Index>::BuildHQ( const Slice& vertices ) { BuildHQ( vertices, 0, 0 ); }
template <typename Float, typename Index> void BVH8_CWBVH<Float, Index>::BuildHQ( const bvhvec4* v, const uint32_t* i, const Index p ) { BuildHQ( Slice( v, p * 3, sizeof( bvhvec4 ) ), i, p ); }
template <typename Float, typename Index> void BVH8_CWBVH<Float, Index>::BuildHQ( const Slice& v, const uint32_t* i, Index p ) { settings.useSpatialSplits = true; Build( v, i, p ); }

template <typename Float, typename Index> void BVH8_CWBVH<Float, Index>::Build( const Slice& vertices, const uint32_t* indices, Index prims )
{
	// propagate settings for this layout to the underlying layout
	bvh8.bvh.context = bvh8.context = context;
	bvh8.bvh.settings = bvh8.settings = settings;
	bvh8.bvh.c_int = bvh8.c_int = c_int, bvh8.bvh.c_trav = bvh8.c_trav = c_trav;
	// build underlying layout
	bvh8.bvh.Build( vertices, indices, prims );
	bvh8.bvh.Compact();
	uint32_t firstIdx = 0;
	bvh8.bvh.CombineLeafs( 3, firstIdx, 0 ); // merge subtrees that fit in one leaf slot
	bvh8.bvh.SplitLeafs( 3 );
	// convert to BVH8_CPU layout
	bvh8.leafPrimLimit = 3; // a cwbvh leaf slot holds at most 3 prims
	bvh8.ConvertFrom( bvh8.bvh, true );
	ConvertFrom( bvh8, true );
}

template <typename Float, typename Index> void BVH8_CWBVH<Float, Index>::Optimize( const uint32_t iterations, bool extreme )
{
	bvh8.Optimize( iterations, extreme );
	ConvertFrom( bvh8, true );
}

template <typename Float, typename Index> Float BVH8_CWBVH<Float, Index>::SAHCost( const Index nodeIdx ) const
{
	return bvh8.SAHCost( nodeIdx );
}

template <typename Float, typename Index> void BVH8_CWBVH<Float, Index>::Save( const char* fileName )
{
	std::fstream s{ fileName, s.binary | s.out };
	const uint32_t header = this->CacheHeader();
	s.write( (char*)&header, sizeof( uint32_t ) );
	s.write( (char*)&triCount, sizeof( Index ) );
	s.write( (char*)this, sizeof( BVH8_CWBVH ) );
	s.write( (char*)bvh8Data, usedBlocks * 16 );
	s.write( (char*)bvh8Tris, usedTriBlocks * 16 );
}

template <typename Float, typename Index> bool BVH8_CWBVH<Float, Index>::Load( const char* fileName, const Index expectedTris )
{
	// open file and check contents
	std::fstream s{ fileName, s.binary | s.in };
	if (!s) return false;
	BVHContext tmp = context;
	uint32_t header;
	Index fileTriCount;
	s.read( (char*)&header, sizeof( uint32_t ) );
	if (header != this->CacheHeader()) return false;
	s.read( (char*)&fileTriCount, sizeof( Index ) );
	if (fileTriCount != expectedTris) return false;
	// all checks passed; safe to overwrite *this
	s.read( (char*)this, sizeof( BVH8_CWBVH ) );
	context = tmp; // can't load context; function pointers will differ.
	bvh8Data = (bvhvec4*)AlignedAlloc( usedBlocks * 16 );
	bvh8Tris = (bvhvec4*)AlignedAlloc( usedTriBlocks * 16 );
	allocatedBlocks = usedBlocks, allocatedTris = usedTriBlocks;
	s.read( (char*)bvh8Data, usedBlocks * 16 );
	s.read( (char*)bvh8Tris, usedTriBlocks * 16 );
	bvh8 = MBVH<8, Float, Index>();
	return true;
}

// quantization exponent, guarded against zero / denormal / NaN extents.
static int32_t CWBVHQuantExp( const float extent )
{
	if (!(extent > 0)) return -126; // zero, denormal or NaN extent
	const float e = ceilf( log2f( extent * (1.0f / 255.0f) ) );
	return (int32_t)tinybvh_clamp( e, -126.0f, 126.0f );
}

// Convert a BVH8 to the format specified in: "Efficient Incoherent Ray Traversal on GPUs Through
// Compressed Wide BVHs", Ylitie et al. 2017. Adapted from code by "AlanWBFT".
template <typename Float, typename Index> void BVH8_CWBVH<Float, Index>::ConvertFrom( const MBVH<8, Float, Index>& original, bool compact )
{
	// get a copy of the original bvh8
	if (&original != &bvh8) ownBVH8 = false; // bvh isn't ours; don't delete in destructor.
	bvh8 = original;
	BVH_FATAL_ERROR_IF( bvh8.mbvhNode[0].isLeaf(), "BVH8_CWBVH::ConvertFrom( .. ), converting a single-node bvh." );
	CopyBasePropertiesFrom( bvh8 );
	uint32_t nodeCap = tinybvh_max( 128u, bvh8.triCount >> 1 );
	if (bvh8Data == 0 || nodeCap > allocatedBlocks)
	{
		AlignedFree( bvh8Data );
		bvh8Data = (bvhvec4*)AlignedAlloc( nodeCap * 16 );
		allocatedBlocks = nodeCap;
	}
#ifdef CWBVH_COMPRESSED_TRIS
	const uint32_t triCap = bvh8.idxCount * 4;
#else
	const uint32_t triCap = bvh8.idxCount * 3;
#endif
	if (bvh8Tris == 0 || triCap > allocatedTris)
	{
		AlignedFree( bvh8Tris );
		bvh8Tris = (bvhvec4*)AlignedAlloc( triCap * 16 );
		allocatedTris = triCap;
	}
	uint32_t stackCap = 512, stackPtr = 1;
	uint32_t* stackNodeIdx = (uint32_t*)AlignedAlloc( stackCap * 4 );
	uint32_t* stackNodeAddr = (uint32_t*)AlignedAlloc( stackCap * 4 );
	stackNodeIdx[0] = 0, stackNodeAddr[0] = 0;
	uint32_t nodeDataPtr = 5, triDataPtr = 0;
	uint32_t slotsUsed = 0, nodeBlocks = 0, leafPrims = 0, leafBlocks = 0;
	while (stackPtr > 0)
	{
		const uint32_t origIdx = stackNodeIdx[--stackPtr];
		const typename MBVH<8, Float, Index>::MBVHNode* orig = &bvh8.mbvhNode[origIdx];
		const uint32_t currentNodeAddr = stackNodeAddr[stackPtr];
		if (nodeDataPtr + 45 > allocatedBlocks)
		{
			const uint32_t newCap = tinybvh_max( allocatedBlocks + (allocatedBlocks >> 1), nodeDataPtr + 45 );
			bvhvec4* newData = (bvhvec4*)AlignedAlloc( newCap * 16 );
			memcpy( newData, bvh8Data, nodeDataPtr * 16 );
			AlignedFree( bvh8Data );
			bvh8Data = newData, allocatedBlocks = newCap;
		}
		memset( &bvh8Data[currentNodeAddr], 0, 80 );
		const Vec3 nodeLo = orig->aabbMin, nodeHi = orig->aabbMax;
		// greedy child node ordering
		const Vec3 nodeCentroid = (nodeLo + nodeHi) * 0.5f;
		float cost[8][8];
		int32_t assignment[8];
		bool isSlotEmpty[8];
		for (int32_t s = 0; s < 8; s++) isSlotEmpty[s] = true, assignment[s] = -1;
		for (int32_t i = 0; i < 8; i++)
		{
			if (orig->child[i] == 0) { for (int32_t s = 0; s < 8; s++) cost[s][i] = bvh_far<Float>; continue; }
			const typename MBVH<8, Float, Index>::MBVHNode* const child = &bvh8.mbvhNode[orig->child[i]];
			const Vec3 d = (child->aabbMin + child->aabbMax) * 0.5f - nodeCentroid;
			for (int32_t s = 0; s < 8; s++)
				cost[s][i] = ((s & 4) ? -d.x : d.x) + ((s & 2) ? -d.y : d.y) + ((s & 1) ? -d.z : d.z);
		}
		while (1)
		{
			float minCost = bvh_far<Float>;
			int32_t minEntryx = -1, minEntryy = -1;
			for (int32_t s = 0; s < 8; s++) if (isSlotEmpty[s]) for (int32_t i = 0; i < 8; i++)
				if (assignment[i] == -1 && cost[s][i] < minCost)
					minCost = cost[s][i], minEntryx = s, minEntryy = i;
			if (minEntryx == -1) break;
			isSlotEmpty[minEntryx] = false, assignment[minEntryy] = minEntryx;
		}
		for (int32_t i = 0; i < 8; i++) if (assignment[i] == -1) for (int32_t s = 0; s < 8; s++) if (isSlotEmpty[s])
		{
			isSlotEmpty[s] = false, assignment[i] = s;
			break;
		}
		uint32_t slotChild[8] = {};
		for (int32_t i = 0; i < 8; i++) slotChild[assignment[i]] = orig->child[i];
		const int32_t ex = CWBVHQuantExp( nodeHi.x - nodeLo.x );
		const int32_t ey = CWBVHQuantExp( nodeHi.y - nodeLo.y );
		const int32_t ez = CWBVHQuantExp( nodeHi.z - nodeLo.z );
		const uint32_t bx = (uint32_t)(127 - ex) << 23, by = (uint32_t)(127 - ey) << 23, bz = (uint32_t)(127 - ez) << 23;
		const float rsx = tinybvh_as_float( bx ), rsy = tinybvh_as_float( by ), rsz = tinybvh_as_float( bz ); // 2^-e, exact
		// encode output
		int32_t internalChildCount = 0, leafChildTriCount = 0, childBaseIndex = 0, triangleBaseIndex = 0;
		uint8_t imask = 0;
		uint8_t* const qbase = (uint8_t*)&bvh8Data[currentNodeAddr + 2];
		uint8_t* const metaField = ((uint8_t*)&bvh8Data[currentNodeAddr + 1]) + 8;
		for (int32_t i = 0; i < 8; i++)
		{
			if (slotChild[i] == 0) continue;
			const typename MBVH<8, Float, Index>::MBVHNode* const child = &bvh8.mbvhNode[slotChild[i]];
			slotsUsed++;
			qbase[i + 0] = (uint8_t)tinybvh_clamp( (int32_t)floorf( (child->aabbMin.x - nodeLo.x) * rsx ), 0, 255 );
			qbase[i + 8] = (uint8_t)tinybvh_clamp( (int32_t)floorf( (child->aabbMin.y - nodeLo.y) * rsy ), 0, 255 );
			qbase[i + 16] = (uint8_t)tinybvh_clamp( (int32_t)floorf( (child->aabbMin.z - nodeLo.z) * rsz ), 0, 255 );
			qbase[i + 24] = (uint8_t)tinybvh_clamp( (int32_t)ceilf( (child->aabbMax.x - nodeLo.x) * rsx ), 0, 255 );
			qbase[i + 32] = (uint8_t)tinybvh_clamp( (int32_t)ceilf( (child->aabbMax.y - nodeLo.y) * rsy ), 0, 255 );
			qbase[i + 40] = (uint8_t)tinybvh_clamp( (int32_t)ceilf( (child->aabbMax.z - nodeLo.z) * rsz ), 0, 255 );
			if (!child->isLeaf())
			{
				// interior node, set params and push onto stack
				const uint32_t childNodeAddr = nodeDataPtr;
				if (internalChildCount++ == 0) childBaseIndex = childNodeAddr / 5;
				nodeDataPtr += 5, imask |= 1 << i;
				metaField[i] = (uint8_t)((1 << 5) | (24 + i));
				if (stackPtr == stackCap)
				{
					const uint32_t newCap = stackCap * 2;
					uint32_t* ni = (uint32_t*)AlignedAlloc( newCap * 4 );
					uint32_t* na = (uint32_t*)AlignedAlloc( newCap * 4 );
					memcpy( ni, stackNodeIdx, stackCap * 4 ), memcpy( na, stackNodeAddr, stackCap * 4 );
					AlignedFree( stackNodeIdx ), AlignedFree( stackNodeAddr );
					stackNodeIdx = ni, stackNodeAddr = na, stackCap = newCap;
				}
				stackNodeIdx[stackPtr] = slotChild[i], stackNodeAddr[stackPtr++] = childNodeAddr;
				continue;
			}
			// leaf node
			const uint32_t tcount = child->triCount;
			BVH_FATAL_ERROR_IF( tcount == 0 || tcount > 3,
				"BVH8_CWBVH::ConvertFrom( .. ), leaf with unsupported triangle count; call BVH::SplitLeafs( 3 ) first." );
			if (leafChildTriCount == 0) triangleBaseIndex = triDataPtr;
			const int32_t unaryEncodedTriCount = tcount == 1 ? 0b001 : tcount == 2 ? 0b011 : 0b111;
			metaField[i] = (uint8_t)((unaryEncodedTriCount << 5) | leafChildTriCount);
			leafChildTriCount += tcount;
			leafPrims += tcount, leafBlocks++;
			for (uint32_t j = 0; j < tcount; j++)
			{
				const uint32_t triIdx = (uint32_t)bvh8.bvh.primIdx[child->firstTri + j];
				uint32_t ti0, ti1, ti2;
				if (bvh8.bvh.vertIdx)
					ti0 = bvh8.bvh.vertIdx[triIdx * 3],
					ti1 = bvh8.bvh.vertIdx[triIdx * 3 + 1],
					ti2 = bvh8.bvh.vertIdx[triIdx * 3 + 2];
				else
					ti0 = triIdx * 3, ti1 = triIdx * 3 + 1, ti2 = triIdx * 3 + 2;
			#ifdef CWBVH_COMPRESSED_TRIS
				PrecomputeTriangle( bvh8.bvh.verts, ti0, ti1, ti2, &bvh8Tris[triDataPtr] );
				bvh8Tris[triDataPtr + 3] = bvhvec4( 0, 0, 0, tinybvh_as_float( triIdx ) );
				triDataPtr += 4;
			#else
				bvhvec4 t = bvh8.bvh.verts[ti0];
				bvh8Tris[triDataPtr + 0] = bvh8.bvh.verts[ti2] - t;
				bvh8Tris[triDataPtr + 1] = bvh8.bvh.verts[ti1] - t;
				t.w = tinybvh_as_float( triIdx );
				bvh8Tris[triDataPtr + 2] = t, triDataPtr += 3;
			#endif
			}
		}
		nodeBlocks++;
		const uint8_t exyzAndimask[4] = { (uint8_t)(ex + 127), (uint8_t)(ey + 127), (uint8_t)(ez + 127), imask };
		bvh8Data[currentNodeAddr + 0] = bvhvec4( nodeLo, tinybvh_bitcast<float>( exyzAndimask ) );
		bvh8Data[currentNodeAddr + 1].x = tinybvh_as_float( childBaseIndex );
		bvh8Data[currentNodeAddr + 1].y = tinybvh_as_float( triangleBaseIndex );
	}
	AlignedFree( stackNodeIdx ), AlignedFree( stackNodeAddr );
	usedBlocks = nodeDataPtr;
	usedTriBlocks = triDataPtr;
	if (compact && allocatedBlocks > usedBlocks)
	{
		bvhvec4* trimmed = (bvhvec4*)AlignedAlloc( usedBlocks * 16 );
		memcpy( trimmed, bvh8Data, usedBlocks * 16 );
		AlignedFree( bvh8Data );
		bvh8Data = trimmed, allocatedBlocks = usedBlocks;
	}
#ifdef CWBVH_REPORT_FULLNESS
	// The two fullness numbers Haydel et al. report for every build: how much of
	// each 8-slot node block and each 3-triangle leaf the collapse actually filled.
	printf( "CWBVH: %.2f children/node (max 8), %.2f prims/leaf (max 3), %.1f bytes/tri\n",
		nodeBlocks ? (float)slotsUsed / (float)nodeBlocks : 0.0f,
		leafBlocks ? (float)leafPrims / (float)leafBlocks : 0.0f,
		triCount ? (float)(usedBlocks + usedTriBlocks) * 16.0f / (float)triCount : 0.0f );
#else
	(void)slotsUsed, (void)nodeBlocks, (void)leafPrims, (void)leafBlocks;
#endif
}

template <typename Float, typename Index> int32_t BVH4_CPU<Float, Index>::Intersect( Ray& ray ) const
{
	VALIDATE_RAY( ray );
	OCTANT_DISPATCH( IntersectOctant, ray )
}

template <typename Float, typename Index> bool BVH4_CPU<Float, Index>::IsOccluded( const Ray& ray ) const
{
	VALIDATE_RAY( ray );
	OCTANT_DISPATCH( IsOccludedOctant, ray )
}

template <typename Float, typename Index> int32_t BVH8_CPU<Float, Index>::Intersect( Ray& ray ) const
{
	VALIDATE_RAY( ray );
	OCTANT_DISPATCH( IntersectOctant, ray )
}

template <typename Float, typename Index> bool BVH8_CPU<Float, Index>::IsOccluded( const Ray& ray ) const
{
	VALIDATE_RAY( ray );
	OCTANT_DISPATCH( IsOccludedOctant, ray )
}

// Intersect_CWBVH:
// Intersect a compressed 8-wide BVH with a ray. For debugging only, not efficient.
// This is here to test data before it goes to the GPU.
#define STACK_POP() { ngroup = traversalStack[--stackPtr]; }
#define STACK_PUSH() { traversalStack[stackPtr++] = ngroup; }
TINYBVH_FORCEINLINE uint32_t extract_byte( const uint32_t i, const uint32_t n ) { return (i >> (n * 8)) & 0xFF; }
TINYBVH_FORCEINLINE uint32_t sign_extend_s8x4( const uint32_t i )
{
	// asm("prmt.b32 %0, %1, 0x0, 0x0000BA98;" : "=r"(v) : "r"(i)); // BA98: 1011`1010`1001`1000
	// with the given parameters, prmt will extend the sign to all bits in a byte.
	uint32_t b0 = (i & 0b10000000000000000000000000000000) ? 0xff000000 : 0;
	uint32_t b1 = (i & 0b00000000100000000000000000000000) ? 0x00ff0000 : 0;
	uint32_t b2 = (i & 0b00000000000000001000000000000000) ? 0x0000ff00 : 0;
	uint32_t b3 = (i & 0b00000000000000000000000010000000) ? 0x000000ff : 0;
	return b0 + b1 + b2 + b3; // probably can do better than this.
}
template <typename Float, typename Index> int32_t BVH8_CWBVH<Float, Index>::Intersect( Ray& ray ) const
{
	bvhuint2 traversalStack[TINYBVH_STACK_SIZE * 4 /* wide trees push more nodes per step */];
	uint32_t hitAddr = 0, stackPtr = 0;
	bvhvec2 triangleuv( 0, 0 );
	const bvhvec4* blasNodes = bvh8Data, * blasTris = bvh8Tris;
	float tmin = 0, tmax = ray.hit.t;
	const uint32_t octinv = (7 - ((ray.D.x < 0 ? 4 : 0) | (ray.D.y < 0 ? 2 : 0) | (ray.D.z < 0 ? 1 : 0))) * 0x1010101;
	bvhuint2 ngroup = bvhuint2( 0, 0b10000000000000000000000000000000 ), tgroup = bvhuint2( 0 );
	do
	{
		if (ngroup.y > 0x00FFFFFF)
		{
			const uint32_t hits = ngroup.y, imask = ngroup.y;
			const uint32_t child_bit_index = __bfind( hits ), child_node_base_index = ngroup.x;
			ngroup.y &= ~(1 << child_bit_index);
			if (ngroup.y > 0x00FFFFFF) { STACK_PUSH( /* nodeGroup */ ); }
			{
				const uint32_t slot_index = (child_bit_index - 24) ^ (octinv & 255);
				const uint32_t relative_index = __popc( imask & ~(0xFFFFFFFF << slot_index) );
				const uint32_t child_node_index = child_node_base_index + relative_index;
				const bvhvec4 n0 = blasNodes[child_node_index * 5 + 0], n1 = blasNodes[child_node_index * 5 + 1];
				const bvhvec4 n2 = blasNodes[child_node_index * 5 + 2], n3 = blasNodes[child_node_index * 5 + 3];
				const bvhvec4 n4 = blasNodes[child_node_index * 5 + 4], p = n0;
				// n0.w holds three 127-biased quantization exponents plus imask.
				const uint32_t e4 = as_uint( n0.w );
				ngroup.x = as_uint( n1.x ), tgroup.x = as_uint( n1.y ), tgroup.y = 0;
				uint32_t hitmask = 0;
				const uint32_t vx = (e4 & 255) << 23u; const float adjusted_idirx = tinybvh_as_float( vx ) * ray.rD.x;
				const uint32_t vy = ((e4 >> 8) & 255) << 23u; const float adjusted_idiry = tinybvh_as_float( vy ) * ray.rD.y;
				const uint32_t vz = ((e4 >> 16) & 255) << 23u; const float adjusted_idirz = tinybvh_as_float( vz ) * ray.rD.z;
				const float origx = -(ray.O.x - p.x) * ray.rD.x;
				const float origy = -(ray.O.y - p.y) * ray.rD.y;
				const float origz = -(ray.O.z - p.z) * ray.rD.z;
				{	// First 4
					const uint32_t meta4 = tinybvh_as_uint( n1.z ), is_inner4 = (meta4 & (meta4 << 1)) & 0x10101010;
					const uint32_t inner_mask4 = sign_extend_s8x4( is_inner4 << 3 );
					const uint32_t bit_index4 = (meta4 ^ (octinv & inner_mask4)) & 0x1F1F1F1F;
					const uint32_t child_bits4 = (meta4 >> 5) & 0x07070707;
					uint32_t swizzledLox = (ray.rD.x < 0) ? tinybvh_as_uint( n3.z ) : tinybvh_as_uint( n2.x ), swizzledHix = (ray.rD.x < 0) ? tinybvh_as_uint( n2.x ) : tinybvh_as_uint( n3.z );
					uint32_t swizzledLoy = (ray.rD.y < 0) ? tinybvh_as_uint( n4.x ) : tinybvh_as_uint( n2.z ), swizzledHiy = (ray.rD.y < 0) ? tinybvh_as_uint( n2.z ) : tinybvh_as_uint( n4.x );
					uint32_t swizzledLoz = (ray.rD.z < 0) ? tinybvh_as_uint( n4.z ) : tinybvh_as_uint( n3.x ), swizzledHiz = (ray.rD.z < 0) ? tinybvh_as_uint( n3.x ) : tinybvh_as_uint( n4.z );
					float tminx[4], tminy[4], tminz[4], tmaxx[4], tmaxy[4], tmaxz[4];
					tminx[0] = ((swizzledLox >> 0) & 0xFF) * adjusted_idirx + origx, tminx[1] = ((swizzledLox >> 8) & 0xFF) * adjusted_idirx + origx, tminx[2] = ((swizzledLox >> 16) & 0xFF) * adjusted_idirx + origx;
					tminx[3] = ((swizzledLox >> 24) & 0xFF) * adjusted_idirx + origx, tminy[0] = ((swizzledLoy >> 0) & 0xFF) * adjusted_idiry + origy, tminy[1] = ((swizzledLoy >> 8) & 0xFF) * adjusted_idiry + origy;
					tminy[2] = ((swizzledLoy >> 16) & 0xFF) * adjusted_idiry + origy, tminy[3] = ((swizzledLoy >> 24) & 0xFF) * adjusted_idiry + origy, tminz[0] = ((swizzledLoz >> 0) & 0xFF) * adjusted_idirz + origz;
					tminz[1] = ((swizzledLoz >> 8) & 0xFF) * adjusted_idirz + origz, tminz[2] = ((swizzledLoz >> 16) & 0xFF) * adjusted_idirz + origz, tminz[3] = ((swizzledLoz >> 24) & 0xFF) * adjusted_idirz + origz;
					tmaxx[0] = ((swizzledHix >> 0) & 0xFF) * adjusted_idirx + origx, tmaxx[1] = ((swizzledHix >> 8) & 0xFF) * adjusted_idirx + origx, tmaxx[2] = ((swizzledHix >> 16) & 0xFF) * adjusted_idirx + origx;
					tmaxx[3] = ((swizzledHix >> 24) & 0xFF) * adjusted_idirx + origx, tmaxy[0] = ((swizzledHiy >> 0) & 0xFF) * adjusted_idiry + origy, tmaxy[1] = ((swizzledHiy >> 8) & 0xFF) * adjusted_idiry + origy;
					tmaxy[2] = ((swizzledHiy >> 16) & 0xFF) * adjusted_idiry + origy, tmaxy[3] = ((swizzledHiy >> 24) & 0xFF) * adjusted_idiry + origy, tmaxz[0] = ((swizzledHiz >> 0) & 0xFF) * adjusted_idirz + origz;
					tmaxz[1] = ((swizzledHiz >> 8) & 0xFF) * adjusted_idirz + origz, tmaxz[2] = ((swizzledHiz >> 16) & 0xFF) * adjusted_idirz + origz, tmaxz[3] = ((swizzledHiz >> 24) & 0xFF) * adjusted_idirz + origz;
					for (int32_t i = 0; i < 4; i++)
					{
						// Use VMIN, VMAX to compute the slabs
						const float cmin = tinybvh_max( tinybvh_max( tinybvh_max( tminx[i], tminy[i] ), tminz[i] ), tmin );
						const float cmax = tinybvh_min( tinybvh_min( tinybvh_min( tmaxx[i], tmaxy[i] ), tmaxz[i] ), tmax );
						if (cmin <= cmax) hitmask |= extract_byte( child_bits4, i ) << extract_byte( bit_index4, i );
					}
				}
				{	// Second 4
					const uint32_t meta4 = tinybvh_as_uint( n1.w ), is_inner4 = (meta4 & (meta4 << 1)) & 0x10101010;
					const uint32_t inner_mask4 = sign_extend_s8x4( is_inner4 << 3 );
					const uint32_t bit_index4 = (meta4 ^ (octinv & inner_mask4)) & 0x1F1F1F1F;
					const uint32_t child_bits4 = (meta4 >> 5) & 0x07070707;
					uint32_t swizzledLox = (ray.rD.x < 0) ? tinybvh_as_uint( n3.w ) : tinybvh_as_uint( n2.y ), swizzledHix = (ray.rD.x < 0) ? tinybvh_as_uint( n2.y ) : tinybvh_as_uint( n3.w );
					uint32_t swizzledLoy = (ray.rD.y < 0) ? tinybvh_as_uint( n4.y ) : tinybvh_as_uint( n2.w ), swizzledHiy = (ray.rD.y < 0) ? tinybvh_as_uint( n2.w ) : tinybvh_as_uint( n4.y );
					uint32_t swizzledLoz = (ray.rD.z < 0) ? tinybvh_as_uint( n4.w ) : tinybvh_as_uint( n3.y ), swizzledHiz = (ray.rD.z < 0) ? tinybvh_as_uint( n3.y ) : tinybvh_as_uint( n4.w );
					float tminx[4], tminy[4], tminz[4], tmaxx[4], tmaxy[4], tmaxz[4];
					tminx[0] = ((swizzledLox >> 0) & 0xFF) * adjusted_idirx + origx, tminx[1] = ((swizzledLox >> 8) & 0xFF) * adjusted_idirx + origx, tminx[2] = ((swizzledLox >> 16) & 0xFF) * adjusted_idirx + origx;
					tminx[3] = ((swizzledLox >> 24) & 0xFF) * adjusted_idirx + origx, tminy[0] = ((swizzledLoy >> 0) & 0xFF) * adjusted_idiry + origy, tminy[1] = ((swizzledLoy >> 8) & 0xFF) * adjusted_idiry + origy;
					tminy[2] = ((swizzledLoy >> 16) & 0xFF) * adjusted_idiry + origy, tminy[3] = ((swizzledLoy >> 24) & 0xFF) * adjusted_idiry + origy, tminz[0] = ((swizzledLoz >> 0) & 0xFF) * adjusted_idirz + origz;
					tminz[1] = ((swizzledLoz >> 8) & 0xFF) * adjusted_idirz + origz, tminz[2] = ((swizzledLoz >> 16) & 0xFF) * adjusted_idirz + origz, tminz[3] = ((swizzledLoz >> 24) & 0xFF) * adjusted_idirz + origz;
					tmaxx[0] = ((swizzledHix >> 0) & 0xFF) * adjusted_idirx + origx, tmaxx[1] = ((swizzledHix >> 8) & 0xFF) * adjusted_idirx + origx, tmaxx[2] = ((swizzledHix >> 16) & 0xFF) * adjusted_idirx + origx;
					tmaxx[3] = ((swizzledHix >> 24) & 0xFF) * adjusted_idirx + origx, tmaxy[0] = ((swizzledHiy >> 0) & 0xFF) * adjusted_idiry + origy, tmaxy[1] = ((swizzledHiy >> 8) & 0xFF) * adjusted_idiry + origy;
					tmaxy[2] = ((swizzledHiy >> 16) & 0xFF) * adjusted_idiry + origy, tmaxy[3] = ((swizzledHiy >> 24) & 0xFF) * adjusted_idiry + origy, tmaxz[0] = ((swizzledHiz >> 0) & 0xFF) * adjusted_idirz + origz;
					tmaxz[1] = ((swizzledHiz >> 8) & 0xFF) * adjusted_idirz + origz, tmaxz[2] = ((swizzledHiz >> 16) & 0xFF) * adjusted_idirz + origz, tmaxz[3] = ((swizzledHiz >> 24) & 0xFF) * adjusted_idirz + origz;
					for (int32_t i = 0; i < 4; i++)
					{
						const float cmin = tinybvh_max( tinybvh_max( tinybvh_max( tminx[i], tminy[i] ), tminz[i] ), tmin );
						const float cmax = tinybvh_min( tinybvh_min( tinybvh_min( tmaxx[i], tmaxy[i] ), tmaxz[i] ), tmax );
						if (cmin <= cmax) hitmask |= extract_byte( child_bits4, i ) << extract_byte( bit_index4, i );
					}
				}
				ngroup.y = (hitmask & 0xFF000000) | (as_uint( n0.w ) >> 24), tgroup.y = hitmask & 0x00FFFFFF;
			}
		}
		else tgroup = ngroup, ngroup = bvhuint2( 0 );
		while (tgroup.y != 0)
		{
		#ifdef CWBVH_COMPRESSED_TRIS
			// "Fast Ray-Triangle Intersections by Coordinate Transformation", Baldwin & Weber, 2016.
			const uint32_t triangleIndex = __bfind( tgroup.y );
			const int32_t triAddr = tgroup.x + triangleIndex * 4;
			tgroup.y -= 1 << triangleIndex;
			const bvhvec4 T2 = blasTris[triAddr + 2];
			const float transD = T2.x * ray.D.x + T2.y * ray.D.y + T2.z * ray.D.z;
			if (transD == 0) continue; // ray parallel to the triangle plane
			const float transS = T2.x * ray.O.x + T2.y * ray.O.y + T2.z * ray.O.z + T2.w;
			const float d = -transS / transD;
			if (!(d > 0 && d < tmax)) continue; // also rejects NaN
			const bvhvec4 T0 = blasTris[triAddr + 0], T1 = blasTris[triAddr + 1];
			const Vec3 I = ray.O + d * ray.D;
			const float u = T0.x * I.x + T0.y * I.y + T0.z * I.z + T0.w;
			const float v = T1.x * I.x + T1.y * I.y + T1.z * I.z + T1.w;
			if (!(u >= 0 && v >= 0 && u + v <= 1)) continue;
			triangleuv = bvhvec2( u, v ), tmax = d;
			hitAddr = as_uint( blasTris[triAddr + 3].w );
		#else
			// Moeller-Trumbore intersection.
			uint32_t triangleIndex = __bfind( tgroup.y );
			tgroup.y -= 1 << triangleIndex;
			int32_t triAddr = tgroup.x + triangleIndex * 3;
			const Vec3 e2 = Vec3( blasTris[triAddr + 0] ), e1 = Vec3( blasTris[triAddr + 1] );
			const Vec3 v0 = blasTris[triAddr + 2];
			MOLLER_TRUMBORE_TEST( tmax, continue );
			triangleuv = bvhvec2( u, v ), tmax = t;
			hitAddr = as_uint( blasTris[triAddr + 2].w );
		#endif
		}
		if (ngroup.y > 0x00FFFFFF) continue;
		if (stackPtr > 0) { STACK_POP( /* nodeGroup */ ); }
		else
		{
			ray.hit.t = tmax;
			if (tmax < bvh_far<Float>) ray.hit.u = triangleuv.x, ray.hit.v = triangleuv.y, ray.hit.prim = hitAddr;
			break;
		}
	} while (true);
	return 0;
}

// Generic definitions of the members that the platform headers specialize.
// ----------------------------------------------------------------------------

template <typename Float, typename Index> void BVH<Float, Index>::BuildAVX( const Vertex* v, const Index p ) { BuildAVX( Slice( v, p * 3, sizeof( Vertex ) ), 0, 0 ); }
template <typename Float, typename Index> void BVH<Float, Index>::BuildAVX( const Vertex* v, const uint32_t* i, const Index p ) { BuildAVX( Slice( v, p * 3, sizeof( Vertex ) ), i, p ); }
template <typename Float, typename Index> void BVH<Float, Index>::BuildAVX( const Slice& v ) { BuildAVX( v, 0, 0 ); }
template <typename Float, typename Index> void BVH<Float, Index>::BuildAVX( const Slice&, const uint32_t*, const Index )
{
	BVH_FATAL_ERROR( "BVH::BuildAVX requires AVX and single precision." );
}

template <typename Float, typename Index> void BVH<Float, Index>::BuildNEON( const Vertex* v, const Index p ) { BuildNEON( Slice( v, p * 3, sizeof( Vertex ) ), 0, 0 ); }
template <typename Float, typename Index> void BVH<Float, Index>::BuildNEON( const Vertex* v, const uint32_t* i, const Index p ) { BuildNEON( Slice( v, p * 3, sizeof( Vertex ) ), i, p ); }
template <typename Float, typename Index> void BVH<Float, Index>::BuildNEON( const Slice& v ) { BuildNEON( v, 0, 0 ); }
template <typename Float, typename Index> void BVH<Float, Index>::BuildNEON( const Slice&, const uint32_t*, const Index )
{
	BVH_FATAL_ERROR( "BVH::BuildNEON requires NEON and single precision." );
}

template <typename Float, typename Index> void BVH<Float, Index>::Intersect256RaysSSE( Ray* ) const
{
	BVH_FATAL_ERROR( "BVH::Intersect256RaysSSE requires AVX and single precision." );
}

// Scalar reference for the BVH4_CPU and BVH8_CPU kernels in tiny_bvh_x86_float.h and
// tiny_bvh_arm_float.h: the same node order and stack handling, without intrinsics. Also
// serves the instantiations that have no SIMD kernel. W is the node width; the perm entries
// hold log2(W) bits per sorted position.
#define WIDE_SLAB_TEST( i, tfar ) \
	const Float tx1 = (posX ? n->xmin[i] : n->xmax[i]) * ray.rD.x - rx, tx2 = (posX ? n->xmax[i] : n->xmin[i]) * ray.rD.x - rx; \
	const Float ty1 = (posY ? n->ymin[i] : n->ymax[i]) * ray.rD.y - ry, ty2 = (posY ? n->ymax[i] : n->ymin[i]) * ray.rD.y - ry; \
	const Float tz1 = (posZ ? n->zmin[i] : n->zmax[i]) * ray.rD.z - rz, tz2 = (posZ ? n->zmax[i] : n->zmin[i]) * ray.rD.z - rz; \
	const Float tmin = tinybvh_max( tinybvh_max( tx1, ty1 ), tinybvh_max( tz1, Float( 0 ) ) ); \
	const Float tmax = tinybvh_min( tinybvh_min( tx2, ty2 ), tinybvh_min( tz2, tfar ) );

template <typename Layout, int W, bool posX, bool posY, bool posZ, typename Float, typename Index>
int32_t tinybvh_wide_intersect( const typename Layout::CacheLine* data, Ray<Float, Index>& ray, const uint32_t* opmap, const uint32_t opmapN )
{
	using Node = typename Layout::BVHNode;
	using Leaf = typename Layout::BVHTri4Leaf;
	static_assert( sizeof( Node::child ) == W * sizeof( uint32_t ), "node width mismatch" );
	constexpr int signShift = (W == 4 ? 2 : 3) * ((posX ? 1 : 0) + (posY ? 2 : 0) + (posZ ? 4 : 0));
	uint32_t nodeStack[TINYBVH_STACK_SIZE * (W / 2) + W /* wide trees push more nodes per step */];
	Float distStack[TINYBVH_STACK_SIZE * (W / 2) + W];
	int32_t stackPtr = 0;
	uint32_t nodeIdx = 0, steps = 0;
	Float tcur = ray.hit.t;
	const Float rx = ray.O.x * ray.rD.x, ry = ray.O.y * ray.rD.y, rz = ray.O.z * ray.rD.z;
	while (1)
	{
		while (!(nodeIdx & Layout::LEAF_BIT))
		{
			steps++;
			const Node* n = (const Node*)(data + nodeIdx);
			Float tmins[W], tmaxs[W];
			for (int i = 0; i < W; i++) { WIDE_SLAB_TEST( i, tcur ); tmins[i] = tmin, tmaxs[i] = tmax; }
			// perm lists the lanes sorted for this octant, farthest first; continue with the
			// nearest valid child and push the others, farthest first.
			uint32_t next = W;
			for (int s = 0; s < W; s++)
			{
				const uint32_t lane = (n->perm[s] >> signShift) & (W - 1);
				if (tmins[lane] > tmaxs[lane]) continue;
				if (next != W) nodeStack[stackPtr] = n->child[next], distStack[stackPtr++] = tmins[next];
				next = lane;
			}
			if (next != W) { nodeIdx = n->child[next]; continue; }
			// skip entries behind the current hit
			do { if (!stackPtr) return (int32_t)steps; nodeIdx = nodeStack[--stackPtr]; } while (distStack[stackPtr] > tcur);
		}
		const Leaf* leaf = (const Leaf*)(data + (nodeIdx & 0x1fffffff));
		for (uint32_t i = 0; i < 4; i++)
		{
			Float t, u, v;
			if (!leaf->Intersect( i, ray, tcur, t, u, v, opmap, opmapN )) continue;
			ray.hit.t = tcur = t, ray.hit.u = u, ray.hit.v = v;
			ray.SetHitPrim( leaf->primIdx[i] );
		}
		do { if (!stackPtr) return (int32_t)steps; nodeIdx = nodeStack[--stackPtr]; } while (distStack[stackPtr] > tcur);
	}
}

template <typename Layout, int W, bool posX, bool posY, bool posZ, typename Float, typename Index>
bool tinybvh_wide_occluded( const typename Layout::CacheLine* data, const Ray<Float, Index>& ray, const uint32_t* opmap, const uint32_t opmapN )
{
	using Node = typename Layout::BVHNode;
	using Leaf = typename Layout::BVHTri4Leaf;
	uint32_t nodeStack[TINYBVH_STACK_SIZE * (W / 2) + W /* wide trees push more nodes per step */];
	int32_t stackPtr = 0;
	uint32_t nodeIdx = 0;
	const Float rx = ray.O.x * ray.rD.x, ry = ray.O.y * ray.rD.y, rz = ray.O.z * ray.rD.z;
	while (1)
	{
		while (!(nodeIdx & Layout::LEAF_BIT))
		{
			const Node* n = (const Node*)(data + nodeIdx);
			// the children can be visited in any order
			uint32_t next = W;
			for (uint32_t i = 0; i < W; i++)
			{
				WIDE_SLAB_TEST( i, ray.hit.t );
				if (tmin > tmax) continue;
				if (next != W) nodeStack[stackPtr++] = n->child[next];
				next = i;
			}
			if (next != W) { nodeIdx = n->child[next]; continue; }
			if (!stackPtr) return false;
			nodeIdx = nodeStack[--stackPtr];
		}
		const Leaf* leaf = (const Leaf*)(data + (nodeIdx & 0x1fffffff));
		for (uint32_t i = 0; i < 4; i++)
		{
			Float t, u, v;
			if (leaf->Intersect( i, ray, ray.hit.t, t, u, v, opmap, opmapN )) return true;
		}
		if (!stackPtr) return false;
		nodeIdx = nodeStack[--stackPtr];
	}
}

#undef WIDE_SLAB_TEST

template <typename Float, typename Index> template <bool posX, bool posY, bool posZ> int32_t BVH4_CPU<Float, Index>::IntersectOctant( Ray& ray ) const
{
	return tinybvh_wide_intersect<BVH4_CPU, 4, posX, posY, posZ>( bvh4Data, ray, opmap, opmapN );
}

template <typename Float, typename Index> template <bool posX, bool posY, bool posZ> bool BVH4_CPU<Float, Index>::IsOccludedOctant( const Ray& ray ) const
{
	return tinybvh_wide_occluded<BVH4_CPU, 4, posX, posY, posZ>( bvh4Data, ray, opmap, opmapN );
}

template <typename Float, typename Index> template <bool posX, bool posY, bool posZ> int32_t BVH8_CPU<Float, Index>::IntersectOctant( Ray& ray ) const
{
	return tinybvh_wide_intersect<BVH8_CPU, 8, posX, posY, posZ>( bvh8Data, ray, opmap, opmapN );
}

template <typename Float, typename Index> template <bool posX, bool posY, bool posZ> bool BVH8_CPU<Float, Index>::IsOccludedOctant( const Ray& ray ) const
{
	return tinybvh_wide_occluded<BVH8_CPU, 8, posX, posY, posZ>( bvh8Data, ray, opmap, opmapN );
}

#ifdef ENABLE_BVH_SOA

// Scalar reference for the BVH_SoA kernels in the platform headers.
#define BVH_SOA_SLAB_TESTS \
	const Float* x = node->xxxx, * y = node->yyyy, * z = node->zzzz; \
	const Float tx1a = (x[0] - ray.O.x) * ray.rD.x, tx2a = (x[1] - ray.O.x) * ray.rD.x; \
	const Float ty1a = (y[0] - ray.O.y) * ray.rD.y, ty2a = (y[1] - ray.O.y) * ray.rD.y; \
	const Float tz1a = (z[0] - ray.O.z) * ray.rD.z, tz2a = (z[1] - ray.O.z) * ray.rD.z; \
	const Float tx1b = (x[2] - ray.O.x) * ray.rD.x, tx2b = (x[3] - ray.O.x) * ray.rD.x; \
	const Float ty1b = (y[2] - ray.O.y) * ray.rD.y, ty2b = (y[3] - ray.O.y) * ray.rD.y; \
	const Float tz1b = (z[2] - ray.O.z) * ray.rD.z, tz2b = (z[3] - ray.O.z) * ray.rD.z; \
	const Float tmina = tinybvh_max( tinybvh_max( tinybvh_min( tx1a, tx2a ), tinybvh_min( ty1a, ty2a ) ), tinybvh_max( tinybvh_min( tz1a, tz2a ), Float( 0 ) ) ); \
	const Float tmaxa = tinybvh_min( tinybvh_min( tinybvh_max( tx1a, tx2a ), tinybvh_max( ty1a, ty2a ) ), tinybvh_min( tinybvh_max( tz1a, tz2a ), ray.hit.t ) ); \
	const Float tminb = tinybvh_max( tinybvh_max( tinybvh_min( tx1b, tx2b ), tinybvh_min( ty1b, ty2b ) ), tinybvh_max( tinybvh_min( tz1b, tz2b ), Float( 0 ) ) ); \
	const Float tmaxb = tinybvh_min( tinybvh_min( tinybvh_max( tx1b, tx2b ), tinybvh_max( ty1b, ty2b ) ), tinybvh_min( tinybvh_max( tz1b, tz2b ), ray.hit.t ) ); \
	Index lidx = node->left, ridx = node->right; \
	Float dist1 = tmaxa >= tmina ? tmina : bvh_far<Float>, dist2 = tmaxb >= tminb ? tminb : bvh_far<Float>; \
	if (dist1 > dist2) { tinybvh_swap( dist1, dist2 ); tinybvh_swap( lidx, ridx ); } \
	if (dist1 == bvh_far<Float>) { if (stackPtr == 0) break; else node = stack[--stackPtr]; } \
	else { node = bvhNode + lidx; if (dist2 != bvh_far<Float>) stack[stackPtr++] = bvhNode + ridx; }

template <typename Float, typename Index> int32_t BVH_SoA<Float, Index>::Intersect( Ray& ray ) const
{
	VALIDATE_RAY( ray );
	BVHNode* node = &bvhNode[0], * stack[TINYBVH_STACK_SIZE];
	const Slice& verts = bvh.verts;
	const Index* primIdx = bvh.primIdx;
	uint32_t stackPtr = 0;
	Float cost = 0;
	while (1)
	{
		cost += c_trav;
		if (node->isLeaf())
		{
			if (indexedEnabled && bvh.vertIdx != 0) for (Index i = 0; i < node->triCount; i++, cost += c_int)
			{
				const Index pi = primIdx[node->firstTri + i];
				const uint32_t i0 = bvh.vertIdx[pi * 3], i1 = bvh.vertIdx[pi * 3 + 1], i2 = bvh.vertIdx[pi * 3 + 2];
				IntersectTri( ray, pi, verts, i0, i1, i2 );
			}
			else for (Index i = 0; i < node->triCount; i++, cost += c_int)
			{
				const Index pi = primIdx[node->firstTri + i];
				IntersectTri( ray, pi, verts, pi * 3, pi * 3 + 1, pi * 3 + 2 );
			}
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		BVH_SOA_SLAB_TESTS;
	}
	return (int32_t)cost;
}

template <typename Float, typename Index> bool BVH_SoA<Float, Index>::IsOccluded( const Ray& ray ) const
{
	BVHNode* node = &bvhNode[0], * stack[TINYBVH_STACK_SIZE];
	const Slice& verts = bvh.verts;
	const Index* primIdx = bvh.primIdx;
	uint32_t stackPtr = 0;
	while (1)
	{
		if (node->isLeaf())
		{
			if (indexedEnabled && bvh.vertIdx != 0) for (Index i = 0; i < node->triCount; i++)
			{
				const Index pi = primIdx[node->firstTri + i], vi0 = pi * 3;
				const uint32_t i0 = bvh.vertIdx[vi0], i1 = bvh.vertIdx[vi0 + 1], i2 = bvh.vertIdx[vi0 + 2];
				if (TriOccludes( ray, verts, pi, i0, i1, i2 )) return true;
			}
			else for (Index i = 0; i < node->triCount; i++)
			{
				const Index pi = primIdx[node->firstTri + i], vi0 = pi * 3;
				if (TriOccludes( ray, verts, pi, vi0, vi0 + 1, vi0 + 2 )) return true;
			}
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		BVH_SOA_SLAB_TESTS;
	}
	return false;
}

#undef BVH_SOA_SLAB_TESTS

#endif

// ============================================================================
//
//        H E L P E R S
//
// ============================================================================

// Update
template <typename Float, typename Index> void BLASInstance<Float, Index>::Update( BVHBase<Float, Index>* blas )
{
	InvertTransform(); // TODO: done unconditionally; for a big TLAS this may be wasteful. Detect changes automatically?
	// transform the eight corners of the root node aabb using the
	// instance transform and calculate the worldspace aabb over those.
	aabbMin = Vec3( bvh_far<Float> ), aabbMax = Vec3( -bvh_far<Float> );
	Vec3 bmin = blas->aabbMin, bmax = blas->aabbMax;
	for (int32_t j = 0; j < 8; j++)
	{
		const Vec3 p( j & 1 ? bmax.x : bmin.x, j & 2 ? bmax.y : bmin.y, j & 4 ? bmax.z : bmin.z );
		const Vec3 t = tinybvh_transform_point( p, transform );
		aabbMin = tinybvh_min( aabbMin, t ), aabbMax = tinybvh_max( aabbMax, t );
	}
}

// InvertTransform - calculate the inverse of the matrix stored in 'transform'
template <typename Float, typename Index> void BLASInstance<Float, Index>::InvertTransform()
{
	// math from MESA, via http://stackoverflow.com/questions/1148309/inverting-a-4x4-matrix
	const Mat4& T = this->transform; // Mat4 indexes its 16 cells directly.
	Mat4& iT = this->invTransform;
	iT[0] = T[5] * T[10] * T[15] - T[5] * T[11] * T[14] - T[9] * T[6] * T[15] + T[9] * T[7] * T[14] + T[13] * T[6] * T[11] - T[13] * T[7] * T[10];
	iT[1] = -T[1] * T[10] * T[15] + T[1] * T[11] * T[14] + T[9] * T[2] * T[15] - T[9] * T[3] * T[14] - T[13] * T[2] * T[11] + T[13] * T[3] * T[10];
	iT[2] = T[1] * T[6] * T[15] - T[1] * T[7] * T[14] - T[5] * T[2] * T[15] + T[5] * T[3] * T[14] + T[13] * T[2] * T[7] - T[13] * T[3] * T[6];
	iT[3] = -T[1] * T[6] * T[11] + T[1] * T[7] * T[10] + T[5] * T[2] * T[11] - T[5] * T[3] * T[10] - T[9] * T[2] * T[7] + T[9] * T[3] * T[6];
	iT[4] = -T[4] * T[10] * T[15] + T[4] * T[11] * T[14] + T[8] * T[6] * T[15] - T[8] * T[7] * T[14] - T[12] * T[6] * T[11] + T[12] * T[7] * T[10];
	iT[5] = T[0] * T[10] * T[15] - T[0] * T[11] * T[14] - T[8] * T[2] * T[15] + T[8] * T[3] * T[14] + T[12] * T[2] * T[11] - T[12] * T[3] * T[10];
	iT[6] = -T[0] * T[6] * T[15] + T[0] * T[7] * T[14] + T[4] * T[2] * T[15] - T[4] * T[3] * T[14] - T[12] * T[2] * T[7] + T[12] * T[3] * T[6];
	iT[7] = T[0] * T[6] * T[11] - T[0] * T[7] * T[10] - T[4] * T[2] * T[11] + T[4] * T[3] * T[10] + T[8] * T[2] * T[7] - T[8] * T[3] * T[6];
	iT[8] = T[4] * T[9] * T[15] - T[4] * T[11] * T[13] - T[8] * T[5] * T[15] + T[8] * T[7] * T[13] + T[12] * T[5] * T[11] - T[12] * T[7] * T[9];
	iT[9] = -T[0] * T[9] * T[15] + T[0] * T[11] * T[13] + T[8] * T[1] * T[15] - T[8] * T[3] * T[13] - T[12] * T[1] * T[11] + T[12] * T[3] * T[9];
	iT[10] = T[0] * T[5] * T[15] - T[0] * T[7] * T[13] - T[4] * T[1] * T[15] + T[4] * T[3] * T[13] + T[12] * T[1] * T[7] - T[12] * T[3] * T[5];
	iT[11] = -T[0] * T[5] * T[11] + T[0] * T[7] * T[9] + T[4] * T[1] * T[11] - T[4] * T[3] * T[9] - T[8] * T[1] * T[7] + T[8] * T[3] * T[5];
	iT[12] = -T[4] * T[9] * T[14] + T[4] * T[10] * T[13] + T[8] * T[5] * T[14] - T[8] * T[6] * T[13] - T[12] * T[5] * T[10] + T[12] * T[6] * T[9];
	iT[13] = T[0] * T[9] * T[14] - T[0] * T[10] * T[13] - T[8] * T[1] * T[14] + T[8] * T[2] * T[13] + T[12] * T[1] * T[10] - T[12] * T[2] * T[9];
	iT[14] = -T[0] * T[5] * T[14] + T[0] * T[6] * T[13] + T[4] * T[1] * T[14] - T[4] * T[2] * T[13] - T[12] * T[1] * T[6] + T[12] * T[2] * T[5];
	iT[15] = T[0] * T[5] * T[10] - T[0] * T[6] * T[9] - T[4] * T[1] * T[10] + T[4] * T[2] * T[9] + T[8] * T[1] * T[6] - T[8] * T[2] * T[5];
	const Float det = T[0] * iT[0] + T[1] * iT[4] + T[2] * iT[8] + T[3] * iT[12];
	if (det == 0) return; // actually, invert failed. That's bad.
	const Float invdet = 1.0f / det;
	for (int i = 0; i < 16; i++) iT[i] *= invdet;
}

// SA
template <typename Float, typename Index> Float BVHBase<Float, Index>::SA( const Vec3& aabbMin, const Vec3& aabbMax )
{
	Vec3 e = aabbMax - aabbMin; // extent of the node
	return e.x * e.y + e.y * e.z + e.z * e.x;
}

// IntersectTri
template <typename Float, typename Index> void BVHBase<Float, Index>::IntersectTri( Ray& ray, const Index triIdx, const Slice& verts, const Index i0, const Index i1, const Index i2 ) const
{
#ifdef WATERTIGHT_TRITEST
	// Woop et al.'s Watertight intersection algorithm.
	// PART 1 - Precalculations
	uint32_t kz = tinybvh_maxdim( ray.D ), kx = (1 << kz) & 3, ky = (1 << kx) & 3;
	if (ray.D[kz] < 0) std::swap( kx, ky );
	const Float Sz = ray.rD[kz], Sx = ray.D[kx] * Sz, Sy = ray.D[ky] * Sz;
	// PART 2 - Intersection
	const Vec3 C = Vec3( verts[i0] ) - ray.O;
	const Vec3 A = Vec3( verts[i1] ) - ray.O;
	const Vec3 B = Vec3( verts[i2] ) - ray.O;
	const Float Ax = A[kx] - Sx * A[kz], Ay = A[ky] - Sy * A[kz];
	const Float Bx = B[kx] - Sx * B[kz], By = B[ky] - Sy * B[kz];
	const Float Cx = C[kx] - Sx * C[kz], Cy = C[ky] - Sy * C[kz];
	const Float U = Cx * By - Cy * Bx, V = Ax * Cy - Ay * Cx, W = Bx * Ay - By * Ax;
	if (!((U >= 0 && V >= 0 && W >= 0) || (U <= 0 && V <= 0 && W <= 0))) return;
	const Float det = U + V + W;
	if (det == 0) return;
	const Float Az = Sz * A[kz], Bz = Sz * B[kz], Cz = Sz * C[kz];
	const Float T = U * Az + V * Bz + W * Cz;
	const Float invDet = 1.0f / det, t = T * invDet;
	if (!(t >= 0 && t < ray.hit.t)) return;
	const Float u = U * invDet, v = V * invDet;
#else
	// Moeller-Trumbore ray/triangle intersection algorithm.
	const Vertex v0_ = verts[i0];
	const Vec3 v0 = v0_, e1 = verts[i1] - v0_, e2 = verts[i2] - v0_;
	MOLLER_TRUMBORE_TEST( ray.hit.t, return );
#endif
	// evaluate opacity map, if present.
	if (opmap)
	{
		const Float fN = (Float)opmapN;
		const int row = int( (u + v) * fN ), diag = int( (1 - u) * fN );
		const int idx = (row * row) + int( v * fN ) + (diag - (opmapN - 1 - row));
		uint32_t* om = opmap + triIdx * ((opmapN * opmapN + 31) >> 5);
		if (!(om[idx >> 5] & (1 << (idx & 31)))) return;
	}
	// register a hit: ray is shortened to t.
	ray.hit.t = t, ray.hit.u = u, ray.hit.v = v;
	ray.SetHitPrim( triIdx );
}

// TriOccludes
template <typename Float, typename Index> bool BVHBase<Float, Index>::TriOccludes( const Ray& ray, const Slice& verts, const Index triIdx, const Index i0, const Index i1, const Index i2 ) const
{
#ifdef WATERTIGHT_TRITEST
	// Woop et al.'s Watertight intersection algorithm.
	// PART 1 - Precalculations
	uint32_t kz = tinybvh_maxdim( ray.D ), kx = (1 << kz) & 3, ky = (1 << kx) & 3;
	if (ray.D[kz] < 0) std::swap( kx, ky );
	const Float Sz = ray.rD[kz], Sx = ray.D[kx] * Sz, Sy = ray.D[ky] * Sz;
	// PART 2 - Intersection
	const Vec3 C = Vec3( verts[i0] ) - ray.O;
	const Vec3 A = Vec3( verts[i1] ) - ray.O;
	const Vec3 B = Vec3( verts[i2] ) - ray.O;
	const Float Ax = A[kx] - Sx * A[kz], Ay = A[ky] - Sy * A[kz];
	const Float Bx = B[kx] - Sx * B[kz], By = B[ky] - Sy * B[kz];
	const Float Cx = C[kx] - Sx * C[kz], Cy = C[ky] - Sy * C[kz];
	const Float U = Cx * By - Cy * Bx, V = Ax * Cy - Ay * Cx, W = Bx * Ay - By * Ax;
	if (!((U >= 0 && V >= 0 && W >= 0) || (U <= 0 && V <= 0 && W <= 0))) return false;
	const Float det = U + V + W;
	if (det == 0) return false;
	const Float Az = Sz * A[kz], Bz = Sz * B[kz], Cz = Sz * C[kz];
	const Float T = U * Az + V * Bz + W * Cz;
	const Float invDet = 1.0f / det, t = T * invDet;
	if (!(t >= 0 && t <= ray.hit.t)) return false;
	const Float u = U * invDet, v = V * invDet;
#else
	// Moeller-Trumbore ray/triangle intersection algorithm
	const Vertex v0_ = verts[i0];
	const Vec3 v0 = v0_, e1 = verts[i1] - v0_, e2 = verts[i2] - v0_;
	MOLLER_TRUMBORE_TEST( ray.hit.t, return false );
#endif
	// evaluate opacity map, if present.
	if (opmap)
	{
		const Float fN = (Float)opmapN;
		const int row = int( (u + v) * fN ), diag = int( (1 - u) * fN );
		const int idx = (row * row) + int( v * fN ) + (diag - (opmapN - 1 - row));
		uint32_t* om = opmap + triIdx * ((opmapN * opmapN + 31) >> 5);
		if (!(om[idx >> 5] & (1 << (idx & 31)))) return false;
	}
	// occluded.
	return true;
}

// PrecomputeTriangle (helper), transforms a triangle to the format used in:
// Fast Ray-Triangle Intersections by Coordinate Transformation. Baldwin & Weber, 2016.
template <typename Float, typename Index> void BVHBase<Float, Index>::PrecomputeTriangle( const Slice& vert, const uint32_t ti0, const uint32_t ti1, const uint32_t ti2, void* dst )
{
	Float T[12];
	Vec3 v0 = vert[ti0], v1 = vert[ti1], v2 = vert[ti2];
	Vec3 e1 = v1 - v0, e2 = v2 - v0, N = tinybvh_cross( e1, e2 );
	Float x1, x2, n = tinybvh_dot( v0, N ), rN;
	if (fabs( N[0] ) > fabs( N[1] ) && fabs( N[0] ) > fabs( N[2] ))
	{
		x1 = v1.y * v0.z - v1.z * v0.y, x2 = v2.y * v0.z - v2.z * v0.y, rN = 1.0f / N.x;
		T[0] = 0, T[1] = e2.z * rN, T[2] = -e2.y * rN, T[3] = x2 * rN;
		T[4] = 0, T[5] = -e1.z * rN, T[6] = e1.y * rN, T[7] = -x1 * rN;
		T[8] = 1, T[9] = N.y * rN, T[10] = N.z * rN, T[11] = -n * rN;
	}
	else if (fabs( N.y ) > fabs( N.z ))
	{
		x1 = v1.z * v0.x - v1.x * v0.z, x2 = v2.z * v0.x - v2.x * v0.z, rN = 1.0f / N.y;
		T[0] = -e2.z * rN, T[1] = 0, T[2] = e2.x * rN, T[3] = x2 * rN;
		T[4] = e1.z * rN, T[5] = 0, T[6] = -e1.x * rN, T[7] = -x1 * rN;
		T[8] = N.x * rN, T[9] = 1, T[10] = N.z * rN, T[11] = -n * rN;
	}
	else if (fabs( N.z ) > 0)
	{
		x1 = v1.x * v0.y - v1.y * v0.x, x2 = v2.x * v0.y - v2.y * v0.x, rN = 1.0f / N.z;
		T[0] = e2.y * rN, T[1] = -e2.x * rN, T[2] = 0, T[3] = x2 * rN;
		T[4] = -e1.y * rN, T[5] = e1.x * rN, T[6] = 0, T[7] = -x1 * rN;
		T[8] = N.x * rN, T[9] = N.y * rN, T[10] = 1, T[11] = -n * rN;
	}
	else memset( T, 0, 12 * 4 );
	memcpy( dst, T, 12 * 4 );
}

template <typename Float, typename Index> bool BVH<Float, Index>::BVHNode::Intersect( const Vec3& bmin, const Vec3& bmax ) const
{
	return bmin.x < aabbMax.x && bmax.x > aabbMin.x &&
		bmin.y < aabbMax.y && bmax.y > aabbMin.y &&
		bmin.z < aabbMax.z && bmax.z > aabbMin.z;
}

// SplitFrag: cut a fragment in two new fragments. Based on madmann91 code.
template <typename Float, typename Index> bool BVH<Float, Index>::SplitFrag( const Fragment& orig, Fragment& left, Fragment& right, const uint32_t axis, const Float pos ) const
{
	left.bmin = Vec3( bvh_far<Float> ), left.bmax = Vec3( -bvh_far<Float> );
	left.primIdx = orig.primIdx, left.clipped = true, right = left;
	auto split_edge = [=]( const Vec3& a, const Vec3& b ) {
		Vec3 c = a + (pos - a[axis]) / (b[axis] - a[axis]) * (b - a);
		c[axis] = pos; /* exactly on split position */ return c; };
	Vec3 v0, v1, v2;
	const Index vidx = orig.primIdx * 3;
	if (!vertIdx) v0 = verts[vidx], v1 = verts[vidx + 1], v2 = verts[vidx + 2];
	else v0 = verts[vertIdx[vidx]], v1 = verts[vertIdx[vidx + 1]], v2 = verts[vertIdx[vidx + 2]];
	const bool l0 = v0[axis] <= pos, l1 = v1[axis] <= pos, l2 = v2[axis] <= pos;
	if (l0) left.Extend( v0 ); else right.Extend( v0 );
	if (l1) left.Extend( v1 ); else right.Extend( v1 );
	if (l2) left.Extend( v2 ); else right.Extend( v2 );
	if (l0 ^ l1) { const Vec3 c = split_edge( v0, v1 ); left.Extend( c ); right.Extend( c ); }
	if (l1 ^ l2) { const Vec3 c = split_edge( v1, v2 ); left.Extend( c ); right.Extend( c ); }
	if (l2 ^ l0) { const Vec3 c = split_edge( v2, v0 ); left.Extend( c ); right.Extend( c ); }
	if (orig.clipped) // clip against orig box
		left.bmin = tinybvh_max( orig.bmin, left.bmin ),
		left.bmax = tinybvh_min( orig.bmax, left.bmax ),
		right.bmin = tinybvh_max( orig.bmin, right.bmin ),
		right.bmax = tinybvh_min( orig.bmax, right.bmax );
	return tinybvh_halfarea( left.bmax - left.bmin ) > 0 && tinybvh_halfarea( right.bmax - right.bmin ) > 0;
}

// ClipFrag: clip a fragment for binning.
template <typename Float, typename Index> bool BVH<Float, Index>::ClipFrag( const Fragment& orig, Fragment& newFrag, Vec3 bmin, Vec3 bmax, const uint32_t axis ) const
{
	Fragment tmp1, tmp2;
	tmp1.bmin = Vec3( bvh_far<Float> ), tmp1.bmax = Vec3( -bvh_far<Float> );
	tmp1.primIdx = orig.primIdx, tmp1.clipped = true, tmp2 = tmp1;
	auto split_edge = [=]( const Vec3& a, const Vec3& b, const Float pos ) {
		Vec3 c = a + (pos - a[axis]) / (b[axis] - a[axis]) * (b - a);
		c[axis] = pos; /* exactly on split position */ return c; };
	Vec3 v0, v1, v2;
	const Index vidx = orig.primIdx * 3;
	if (!vertIdx) v0 = verts[vidx], v1 = verts[vidx + 1], v2 = verts[vidx + 2];
	else v0 = verts[vertIdx[vidx]], v1 = verts[vertIdx[vidx + 1]], v2 = verts[vertIdx[vidx + 2]];
	const Float left = bmin[axis], right = bmax[axis];
	// clip against min bounds
	bool in0 = v0[axis] >= left, in1 = v1[axis] >= left, in2 = v2[axis] >= left;
	if (in0) tmp1.Extend( v0 );
	if (in1) tmp1.Extend( v1 );
	if (in2) tmp1.Extend( v2 );
	if (in0 ^ in1) { const Vec3 c = split_edge( v0, v1, left ); tmp1.Extend( c ); }
	if (in1 ^ in2) { const Vec3 c = split_edge( v1, v2, left ); tmp1.Extend( c ); }
	if (in2 ^ in0) { const Vec3 c = split_edge( v2, v0, left ); tmp1.Extend( c ); }
	// clip against max bounds
	in0 = v0[axis] <= right, in1 = v1[axis] <= right, in2 = v2[axis] <= right;
	if (in0) tmp2.Extend( v0 );
	if (in1) tmp2.Extend( v1 );
	if (in2) tmp2.Extend( v2 );
	if (in0 ^ in1) { const Vec3 c = split_edge( v0, v1, right ); tmp2.Extend( c ); }
	if (in1 ^ in2) { const Vec3 c = split_edge( v1, v2, right ); tmp2.Extend( c ); }
	if (in2 ^ in0) { const Vec3 c = split_edge( v2, v0, right ); tmp2.Extend( c ); }
	newFrag.bmin = tinybvh_max( tmp1.bmin, tmp2.bmin );
	newFrag.bmax = tinybvh_min( tmp1.bmax, tmp2.bmax );
	if (orig.clipped) // clip against orig box
		newFrag.bmin = tinybvh_max( orig.bmin, newFrag.bmin ),
		newFrag.bmax = tinybvh_min( orig.bmax, newFrag.bmax );
	const Float sa = tinybvh_halfarea( newFrag.bmax - newFrag.bmin );
	return sa > 0;
}

// RefitUp: Update bounding boxes of ancestors of the specified node.
// Returns the change in the summed surface area of the nodes it updated.
template <typename Float, typename Index> double BVH_Verbose<Float, Index>::RefitUp( Index nodeIdx, RefitRecord* journal, uint32_t& journalPtr, const uint32_t journalCap )
{
	double deltaArea = 0;
	bool first = true;
	while (1)
	{
		BVHNode& node = bvhNode[nodeIdx];
		if (!node.isLeaf())
		{
			const BVHNode& left = bvhNode[node.left];
			const BVHNode& right = bvhNode[node.right];
			const Vec3 bmin = tinybvh_min( left.aabbMin, right.aabbMin );
			const Vec3 bmax = tinybvh_max( left.aabbMax, right.aabbMax );
			// early out: if this node's box is unchanged, no ancestor changes either.
			if (!first &&
				bmin.x == node.aabbMin.x && bmin.y == node.aabbMin.y && bmin.z == node.aabbMin.z &&
				bmax.x == node.aabbMax.x && bmax.y == node.aabbMax.y && bmax.z == node.aabbMax.z) break;
			if (journalPtr == journalCap) return bvh_far<Float>; // overflow: force a reject.
			journal[journalPtr].node = nodeIdx;
			journal[journalPtr].bmin = node.aabbMin;
			journal[journalPtr++].bmax = node.aabbMax;
			const Float areaBefore = node.SA();
			node.aabbMin = bmin, node.aabbMax = bmax;
			deltaArea += (double)node.SA() - (double)areaBefore;
		}
		first = false;
		if (nodeIdx == 0) break; else nodeIdx = node.parent;
	}
	return deltaArea;
}

// SAHCostUp: Calculate the SAH cost of a node and its ancestry
template <typename Float, typename Index> Float BVH_Verbose<Float, Index>::SAHCostUp( Index nodeIdx ) const
{
	Float sum = 0;
	while (nodeIdx != (Index)-1)
	{
		BVHNode& node = bvhNode[nodeIdx];
		sum += Base::SA( node.aabbMin, node.aabbMax );
		nodeIdx = node.parent;
	}
	return sum;
}

// FindBestNewPosition
// Part of "Fast Insertion-Based Optimization of Bounding Volume Hierarchies"
template <typename Float, typename Index> Index BVH_Verbose<Float, Index>::FindBestNewPosition( const Index Lid, Float& bestCost ) const
{
	struct Task { Float ci; Index node; };
	static const int maxTasks = 512;
	ALIGNED( 64 ) Task task[maxTasks];
	const BVHNode& L = bvhNode[Lid];
	const Vec3 Lmin = L.aabbMin, Lmax = L.aabbMax;
	const Float LSA = L.SA();
	Float Cbest = bvh_far<Float>;
	Index Xbest = 0;
	int tasks = 1;
	task[0].ci = 0, task[0].node = 0;
	while (tasks > 0)
	{
		// pop the task with the smallest induced cost
		const Index Xid = task[0].node;
		const Float CiLX = task[0].ci;
		if (--tasks > 0)
		{
			task[0] = task[tasks];
			for (int i = 0; ; )
			{
				const int l = 2 * i + 1, r = l + 1;
				int s = i;
				if (l < tasks && task[l].ci < task[s].ci) s = l;
				if (r < tasks && task[r].ci < task[s].ci) s = r;
				if (s == i) break;
				const Task t = task[i]; task[i] = task[s]; task[s] = t; i = s;
			}
		}
		// if the popped task has the lowest induced cost in the queue: done.
		if (CiLX + LSA >= Cbest) break;
		const BVHNode& X = bvhNode[Xid];
		const Float CdLX = SA( tinybvh_min( Lmin, X.aabbMin ), tinybvh_max( Lmax, X.aabbMax ) );
		const Float CLX = CiLX + CdLX;
		if (CLX < Cbest && Xid != 0) Cbest = CLX, Xbest = Xid;
		const Float Ci = CLX - X.SA();
		if (Ci + LSA >= Cbest || X.isLeaf() || tasks + 2 > maxTasks) continue;
		for (int c = 0; c < 2; c++)
		{
			int i = tasks++;
			task[i].ci = Ci, task[i].node = c ? X.right : X.left;
			while (i > 0)
			{
				const int p = (i - 1) >> 1;
				if (task[p].ci <= task[i].ci) break;
				const Task t = task[p]; task[p] = task[i]; task[i] = t; i = p;
			}
		}
	}
	bestCost = Cbest;
	return Xbest;
}

// Determine for each node in the tree the number of primitives
// stored in that subtree. Helper function for MergeLeafs.
template <typename Float, typename Index> Index BVH_Verbose<Float, Index>::CountSubtreeTris( const Index nodeIdx, Index* counters )
{
	BVHNode& node = bvhNode[nodeIdx];
	Index result = node.triCount;
	if (!result) result = CountSubtreeTris( node.left, counters ) + CountSubtreeTris( node.right, counters );
	counters[nodeIdx] = result;
	return result;
}

// Write the triangle indices stored in a subtree to a continuous
// slice in the 'newIdx' array. Helper function for MergeLeafs.
template <typename Float, typename Index> void BVH_Verbose<Float, Index>::MergeSubtree( const Index nodeIdx, Index* newIdx, Index& newIdxPtr )
{
	BVHNode& node = bvhNode[nodeIdx];
	if (node.isLeaf())
	{
		memcpy( newIdx + newIdxPtr, primIdx + node.firstTri, node.triCount * sizeof( Index ) );
		newIdxPtr += node.triCount;
		return;
	}
	MergeSubtree( node.left, newIdx, newIdxPtr );
	MergeSubtree( node.right, newIdx, newIdxPtr );
}

} // namespace impl

#ifdef ENABLE_VOXEL_SUPPORT

// VoxelSet implementation
// ----------------------------------------------------------------------------

template <typename F, typename I> int32_t VoxelSet::Intersect( impl::Ray<F, I>& ) const
{
	BVH_FATAL_ERROR( "VoxelSet::Intersect, a VoxelSet can only be traversed with a single precision ray." );
}

template <typename F, typename I> bool VoxelSet::IsOccluded( const impl::Ray<F, I>& ) const
{
	BVH_FATAL_ERROR( "VoxelSet::IsOccluded, a VoxelSet can only be traversed with a single precision ray." );
}

VoxelSet::VoxelSet()
{
	grid = (uint32_t*)AlignedAlloc( gridSize * sizeof( uint32_t ) );
	memset( grid, 0, gridSize * sizeof( uint32_t ) );
	brick = (uint32_t*)AlignedAlloc( brickSize * brickCount * sizeof( uint32_t ) );
	memset( brick, 0, brickSize * brickCount * sizeof( uint32_t ) );
	topGrid = (uint32_t*)AlignedAlloc( topGridBytes );
	memset( topGrid, 0, topGridBytes );
	freeBrickPtr = 1; // first available brick; we'll skip 0
	layout = LAYOUT_VOXELSET;
	aabbMin = bvhvec3( 0 ), aabbMax = bvhvec3( 1 ); // a voxel object is always (1,1,1) in object space.
}

VoxelSet::~VoxelSet()
{
	AlignedFree( grid );
	AlignedFree( brick );
	AlignedFree( topGrid );
	grid = brick = topGrid = 0;
}

void VoxelSet::Set( const uint32_t x, const uint32_t y, const uint32_t z, const uint32_t v )
{
	// note: not thread-safe.
	BVH_FATAL_ERROR_IF( x >= objectDim || y >= objectDim || z >= objectDim, "VoxelSet::Set( .. ), voxel coordinate out of range." );
	const uint32_t bx = x / brickDim, by = y / brickDim, bz = z / brickDim;
	const uint32_t gridIdx = bx + by * gridDim + bz * gridDim * gridDim;
	uint32_t brickIdx = grid[gridIdx];
	if (!brickIdx)
	{
		if (freeBrickPtr == brickCount) // we ran out; reallocate
		{
			uint32_t newBrickCount = brickCount + (brickCount >> 2);
			uint32_t* newBrickPool = (uint32_t*)AlignedAlloc( newBrickCount * brickSize * sizeof( uint32_t ) );
			memcpy( newBrickPool, brick, brickCount * brickSize * sizeof( uint32_t ) );
			memset( newBrickPool + brickCount * brickSize, 0, (newBrickCount - brickCount) * brickSize * sizeof( uint32_t ) );
			AlignedFree( brick );
			brick = newBrickPool, brickCount = newBrickCount;
		}
		brickIdx = grid[gridIdx] = freeBrickPtr++;
	}
	const uint32_t voxelIdx = (x & (brickDim - 1)) + (y & (brickDim - 1)) * brickDim + (z & (brickDim - 1)) * brickDim * brickDim;
	brick[brickIdx * brickSize + voxelIdx] = v;
}

void VoxelSet::UpdateTopGrid()
{
	memset( topGrid, 0, topGridBytes );
	for (int x = 0; x < topGridDim; x++) for (int y = 0; y < topGridDim; y++) for (int z = 0; z < topGridDim; z++)
	{
		uint32_t* gridBase = grid + x * groupDim + y * groupDim * gridDim + z * groupDim * gridDim * gridDim;
		bool hasContent = false;
		for (int u = 0; u < groupDim; u++) for (int v = 0; v < groupDim; v++) for (int w = 0; w < groupDim; w++)
			if (gridBase[u + v * gridDim + w * gridDim * gridDim])
			{
				hasContent = true;
				goto break3;
			}
	break3:
		if (!hasContent) continue;
		uint32_t topIdx = x + y * topGridDim + z * topGridDim * topGridDim;
		topGrid[topIdx >> 5] |= 1 << (topIdx & 31);
	}
}

bool VoxelSet::Setup3DDDA( const Ray& ray, const bvhvec3& Dsign, DDAState& state, const bvhint3& step, bvhvec3& tdelta, float& t ) const
{
	// if ray is not inside the object aabb: advance until it is
	if (!(ray.O.x >= 0 && ray.O.x <= 1 && ray.O.y >= 0 && ray.O.y <= 1 && ray.O.z >= 0 && ray.O.z <= 1))
	{
		float tx1 = -ray.O.x * ray.rD.x, tx2 = (1 - ray.O.x) * ray.rD.x;
		float tmin = tinybvh_min( tx1, tx2 ), tmax = tinybvh_max( tx1, tx2 );
		float ty1 = -ray.O.y * ray.rD.y, ty2 = (1 - ray.O.y) * ray.rD.y;
		tmin = tinybvh_max( tmin, tinybvh_min( ty1, ty2 ) );
		tmax = tinybvh_min( tmax, tinybvh_max( ty1, ty2 ) );
		float tz1 = -ray.O.z * ray.rD.z, tz2 = (1 - ray.O.z) * ray.rD.z;
		tmin = tinybvh_max( tmin, tinybvh_min( tz1, tz2 ) );
		tmax = tinybvh_min( tmax, tinybvh_max( tz1, tz2 ) );
		if (tmax < tmin || tmin > ray.hit.t || tmax < 0) return false; else t = tmin;
	}
	// setup amanatides & woo - assume object size is 1x1x1, from (0,0,0) to (1,1,1)
	constexpr float cellSize = 1.0f / topGridDim;
	const float eps = NudgeScale( ray.O ) * (1.0f / (brickDim * groupDim));	// voxel units -> top-cell units
	const bvhvec3 nudge( (float)step.x * eps, (float)step.y * eps, (float)step.z * eps );
	const bvhvec3 posInGrid = (ray.O + ray.D * t) * (float)topGridDim + nudge;
	const bvhvec3 gridPlanes = (bvhvec3( ceilf( posInGrid.x ), ceilf( posInGrid.y ), ceilf( posInGrid.z ) ) - Dsign) * cellSize;
	const bvhint3 P(
		tinybvh_clamp( (int)posInGrid.x, 0, topGridDim - 1 ),
		tinybvh_clamp( (int)posInGrid.y, 0, topGridDim - 1 ),
		tinybvh_clamp( (int)posInGrid.z, 0, topGridDim - 1 )
	);
	state.X = P.x, state.Y = P.y, state.Z = P.z;
	state.tmax = (gridPlanes - ray.O) * ray.rD;
	tdelta = bvhvec3( (float)step.x, (float)step.y, (float)step.z ) * cellSize * ray.rD;
	// proceed with traversal
	return true;
}

bvhvec3 VoxelSet::GetNormal( const Ray& ray ) const
{
	const bvhvec3 I1 = (ray.O + ray.hit.t * ray.D) * (float)objectDim; // our object is (1,1,1) in object space, so this scales each voxel to (1,1,1)
	const bvhvec3 fG( I1.x - floorf( I1.x ), I1.y - floorf( I1.y ), I1.z - floorf( I1.z ) );
	const bvhvec3 d = tinybvh_min( fG, 1.0f - fG );
	const float mind = tinybvh_min( tinybvh_min( d.x, d.y ), d.z );
	if (mind == d.x) return bvhvec3( ray.D.x > 0 ? -1.0f : 1.0f, 0, 0 );
	else if (mind == d.y) return bvhvec3( 0, ray.D.y > 0 ? -1.0f : 1.0f, 0 );
	else return bvhvec3( 0, 0, ray.D.z > 0 ? -1.0f : 1.0f );
}

int32_t VoxelSet::Intersect( Ray& ray ) const
{
	// setup Amanatides & Woo grid traversal
	ALIGNED( 64 ) DDAState l1_, l2_;
	uint32_t xsign, ysign, zsign;
	memcpy( &xsign, &ray.D.x, sizeof( xsign ) );
	memcpy( &ysign, &ray.D.y, sizeof( xsign ) );
	memcpy( &zsign, &ray.D.z, sizeof( xsign ) );
	xsign >>= 31, ysign >>= 31, zsign >>= 31;
	const bvhvec3 Dsign = bvhvec3( (float)xsign, (float)ysign, (float)zsign );
	const bvhint3 step( 1 - (int)xsign * 2, 1 - (int)ysign * 2, 1 - (int)zsign * 2 );
	const float eps = NudgeScale( ray.O );	// voxel units; scaled down per level below
	const bvhvec3 nudge( (float)step.x * eps, (float)step.y * eps, (float)step.z * eps );
	bvhvec3 tdelta;
	float t = 0;
	if (!Setup3DDDA( ray, Dsign, l1_, step, tdelta, t )) return 0;
	const bvhvec3 l2tdelta = tdelta * (1.0f / groupDim);
	const bvhvec3 l3tdelta = l2tdelta * (1.0f / brickDim);
	uint32_t* gridBase = 0;
	int32_t steps = 0;
	// start stepping:
	while (t < ray.hit.t)	// a voxel beyond the current hit distance cannot win
	{
		const uint32_t tidx = l1_.X + l1_.Y * topGridDim + l1_.Z * topGridDim * topGridDim;
		const uint32_t cell = topGrid[tidx >> 5] & (1 << (tidx & 31));
		if (cell)
		{
			// setup midlevel traversal
			const bvhvec3 posInGrid = (ray.O + t * ray.D) * (float)gridDim + nudge * (1.0f / brickDim);
			const bvhvec3 gridPlanes = (bvhvec3( ceilf( posInGrid.x ), ceilf( posInGrid.y ), ceilf( posInGrid.z ) ) - Dsign) * (1.0f / gridDim);
			l2_.X = tinybvh_clamp( (int)posInGrid.x, l1_.X * groupDim, l1_.X * groupDim + (groupDim - 1) );
			l2_.Y = tinybvh_clamp( (int)posInGrid.y, l1_.Y * groupDim, l1_.Y * groupDim + (groupDim - 1) );
			l2_.Z = tinybvh_clamp( (int)posInGrid.z, l1_.Z * groupDim, l1_.Z * groupDim + (groupDim - 1) );
			l2_.tmax = (gridPlanes - ray.O) * ray.rD;
			gridBase = grid + ((l2_.X + l2_.Y * gridDim + l2_.Z * gridDim * gridDim) & superMask);
			l2_.X &= groupDim - 1, l2_.Y &= groupDim - 1, l2_.Z &= groupDim - 1;
			// step through midlevel cells
			while (1)
			{
				const uint32_t brickCell = gridBase[l2_.X + l2_.Y * gridDim + l2_.Z * gridDim * gridDim];
				if (brickCell)
				{
					// setup 3DDDA for brick traversal
					uint32_t* brickData = brick + brickCell * brickSize;
					const bvhvec3 posInBrick = (ray.O + t * ray.D) * (float)objectDim + nudge;
					uint32_t X = tinybvh_clamp( (int)posInBrick.x - (l2_.X + l1_.X * groupDim) * brickDim, 0, brickDim - 1 );
					uint32_t Y = tinybvh_clamp( (int)posInBrick.y - (l2_.Y + l1_.Y * groupDim) * brickDim, 0, brickDim - 1 );
					uint32_t Z = tinybvh_clamp( (int)posInBrick.z - (l2_.Z + l1_.Z * groupDim) * brickDim, 0, brickDim - 1 );
					const bvhvec3 brickPlanes = (bvhvec3( ceilf( posInBrick.x ), ceilf( posInBrick.y ), ceilf( posInBrick.z ) ) - Dsign) * (1.0f / objectDim);
					bvhvec3 tmax = (brickPlanes - ray.O) * ray.rD;
					// step through brick
					while (1)
					{
						steps++;
						const uint32_t v = brickData[X + Y * brickDim + Z * brickDim * brickDim];
						if (v)
						{
							// traversal is front-to-back, so no closer voxel remains
							if (t >= ray.hit.t) return steps;
							ray.hit.t = t;
							ray.SetHitPrim( v );
							return steps;
						}
						if (tmax.x < tmax.y)
						{
							if (tmax.x < tmax.z)
							{
								if ((X += step.x) >= brickDim) break;
								t = tmax.x, tmax.x += l3tdelta.x;
							}
							else
							{
								if ((Z += step.z) >= brickDim) break;
								t = tmax.z, tmax.z += l3tdelta.z;
							}
						}
						else
						{
							if (tmax.y < tmax.z)
							{
								if ((Y += step.y) >= brickDim) break;
								t = tmax.y, tmax.y += l3tdelta.y;
							}
							else
							{
								if ((Z += step.z) >= brickDim) break;
								t = tmax.z, tmax.z += l3tdelta.z;
							}
						}
					}
				}
				if (l2_.tmax.x < l2_.tmax.y)
				{
					if (l2_.tmax.x < l2_.tmax.z)
					{
						if ((l2_.X += step.x) >= groupDim) break;
						t = l2_.tmax.x, l2_.tmax.x += l2tdelta.x;
					}
					else
					{
						if ((l2_.Z += step.z) >= groupDim) break;
						t = l2_.tmax.z, l2_.tmax.z += l2tdelta.z;
					}
				}
				else
				{
					if (l2_.tmax.y < l2_.tmax.z)
					{
						if ((l2_.Y += step.y) >= groupDim) break;
						t = l2_.tmax.y, l2_.tmax.y += l2tdelta.y;
					}
					else
					{
						if ((l2_.Z += step.z) >= groupDim) break;
						t = l2_.tmax.z, l2_.tmax.z += l2tdelta.z;
					}
				}
			}
		}
		if (l1_.tmax.x < l1_.tmax.y)
		{
			if (l1_.tmax.x < l1_.tmax.z)
			{
				if ((l1_.X += step.x) >= topGridDim) break;
				t = l1_.tmax.x, l1_.tmax.x += tdelta.x;
			}
			else
			{
				if ((l1_.Z += step.z) >= topGridDim) break;
				t = l1_.tmax.z, l1_.tmax.z += tdelta.z;
			}
		}
		else
		{
			if (l1_.tmax.y < l1_.tmax.z)
			{
				if ((l1_.Y += step.y) >= topGridDim) break;
				t = l1_.tmax.y, l1_.tmax.y += tdelta.y;
			}
			else
			{
				if ((l1_.Z += step.z) >= topGridDim) break;
				t = l1_.tmax.z, l1_.tmax.z += tdelta.z;
			}
		}
	}
	return 0;
}

bool VoxelSet::IsOccluded( const Ray& ray ) const
{
	// setup Amanatides & Woo grid traversal
	ALIGNED( 64 ) DDAState l1_, l2_;
	const uint32_t xsign = tinybvh_as_uint( ray.D.x ) >> 31;
	const uint32_t ysign = tinybvh_as_uint( ray.D.y ) >> 31;
	const uint32_t zsign = tinybvh_as_uint( ray.D.z ) >> 31;
	const bvhvec3 Dsign = bvhvec3( (float)xsign, (float)ysign, (float)zsign );
	const bvhint3 step( 1 - (int)xsign * 2, 1 - (int)ysign * 2, 1 - (int)zsign * 2 );
	const float eps = NudgeScale( ray.O );	// voxel units; scaled down per level below
	const bvhvec3 nudge( (float)step.x * eps, (float)step.y * eps, (float)step.z * eps );
	bvhvec3 tdelta;
	float t = 0;
	if (!Setup3DDDA( ray, Dsign, l1_, step, tdelta, t )) return false;
	const bvhvec3 l2tdelta = tdelta * (1.0f / groupDim);
	const bvhvec3 l3tdelta = l2tdelta * (1.0f / brickDim);
	uint32_t* gridBase = 0;
	// start stepping:
	while (t < ray.hit.t)
	{
		const uint32_t tidx = l1_.X + l1_.Y * topGridDim + l1_.Z * topGridDim * topGridDim;
		const uint32_t cell = topGrid[tidx >> 5] & (1 << (tidx & 31));
		if (cell)
		{
			// setup midlevel traversal
			const bvhvec3 posInGrid = (ray.O + t * ray.D) * (float)gridDim + nudge * (1.0f / brickDim);
			const bvhvec3 gridPlanes = (bvhvec3( ceilf( posInGrid.x ), ceilf( posInGrid.y ), ceilf( posInGrid.z ) ) - Dsign) * (1.0f / gridDim);
			l2_.X = tinybvh_clamp( (int)posInGrid.x, l1_.X * groupDim, l1_.X * groupDim + (groupDim - 1) );
			l2_.Y = tinybvh_clamp( (int)posInGrid.y, l1_.Y * groupDim, l1_.Y * groupDim + (groupDim - 1) );
			l2_.Z = tinybvh_clamp( (int)posInGrid.z, l1_.Z * groupDim, l1_.Z * groupDim + (groupDim - 1) );
			l2_.tmax = (gridPlanes - ray.O) * ray.rD;
			gridBase = grid + ((l2_.X + l2_.Y * gridDim + l2_.Z * gridDim * gridDim) & superMask);
			l2_.X &= groupDim - 1, l2_.Y &= groupDim - 1, l2_.Z &= groupDim - 1;
			// step through midlevel cells
			while (1)
			{
				const uint32_t brickCell = gridBase[l2_.X + l2_.Y * gridDim + l2_.Z * gridDim * gridDim];
				if (brickCell)
				{
					// setup 3DDDA for brick traversal
					uint32_t* brickData = brick + brickCell * brickSize;
					const bvhvec3 posInBrick = (ray.O + t * ray.D) * (float)objectDim + nudge;
					uint32_t X = tinybvh_clamp( (int)posInBrick.x - (l2_.X + l1_.X * groupDim) * brickDim, 0u, brickDim - 1 );
					uint32_t Y = tinybvh_clamp( (int)posInBrick.y - (l2_.Y + l1_.Y * groupDim) * brickDim, 0u, brickDim - 1 );
					uint32_t Z = tinybvh_clamp( (int)posInBrick.z - (l2_.Z + l1_.Z * groupDim) * brickDim, 0u, brickDim - 1 );
					const bvhvec3 brickPlanes = (bvhvec3( ceilf( posInBrick.x ), ceilf( posInBrick.y ), ceilf( posInBrick.z ) ) - Dsign) * (1.0f / objectDim);
					bvhvec3 tmax = (brickPlanes - ray.O) * ray.rD;
					// step through brick
					while (1)
					{
						const uint32_t v = brickData[X + Y * brickDim + Z * brickDim * brickDim];
						if (v) return t < ray.hit.t;
						if (tmax.x < tmax.y)
						{
							if (tmax.x < tmax.z)
							{
								if ((X += step.x) >= brickDim) break;
								t = tmax.x, tmax.x += l3tdelta.x;
							}
							else
							{
								if ((Z += step.z) >= brickDim) break;
								t = tmax.z, tmax.z += l3tdelta.z;
							}
						}
						else
						{
							if (tmax.y < tmax.z)
							{
								if ((Y += step.y) >= brickDim) break;
								t = tmax.y, tmax.y += l3tdelta.y;
							}
							else
							{
								if ((Z += step.z) >= brickDim) break;
								t = tmax.z, tmax.z += l3tdelta.z;
							}
						}
					}
				}
				if (l2_.tmax.x < l2_.tmax.y)
				{
					if (l2_.tmax.x < l2_.tmax.z)
					{
						if ((l2_.X += step.x) >= groupDim) break;
						t = l2_.tmax.x, l2_.tmax.x += l2tdelta.x;
					}
					else
					{
						if ((l2_.Z += step.z) >= groupDim) break;
						t = l2_.tmax.z, l2_.tmax.z += l2tdelta.z;
					}
				}
				else
				{
					if (l2_.tmax.y < l2_.tmax.z)
					{
						if ((l2_.Y += step.y) >= groupDim) break;
						t = l2_.tmax.y, l2_.tmax.y += l2tdelta.y;
					}
					else
					{
						if ((l2_.Z += step.z) >= groupDim) break;
						t = l2_.tmax.z, l2_.tmax.z += l2tdelta.z;
					}
				}
			}
		}
		if (l1_.tmax.x < l1_.tmax.y)
		{
			if (l1_.tmax.x < l1_.tmax.z)
			{
				if ((l1_.X += step.x) >= topGridDim) break;
				t = l1_.tmax.x, l1_.tmax.x += tdelta.x;
			}
			else
			{
				if ((l1_.Z += step.z) >= topGridDim) break;
				t = l1_.tmax.z, l1_.tmax.z += tdelta.z;
			}
		}
		else
		{
			if (l1_.tmax.y < l1_.tmax.z)
			{
				if ((l1_.Y += step.y) >= topGridDim) break;
				t = l1_.tmax.y, l1_.tmax.y += tdelta.y;
			}
			else
			{
				if ((l1_.Z += step.z) >= topGridDim) break;
				t = l1_.tmax.z, l1_.tmax.z += tdelta.z;
			}
		}
	}
	// we shouldn't get here
	return false;
}

#endif

// Built-in std::thread pool backing the default BVHContext hooks. Define
// TINYBVH_NO_BUILTIN_POOL to leave this out, in which case builds are serial
// unless you hook your own.
#if defined ENABLE_THREADED_BUILDS && !defined TINYBVH_NO_BUILTIN_POOL

#if defined _WIN32 && defined _MSC_VER
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
#endif // _WIN32 + _MSC_VER
#ifdef PLATFORM_LINUX
#include <pthread.h>
#ifdef __FREEBSD__
#include <pthread_np.h>
#endif
#endif // PLATFORM_LINUX

// Wicked job system, condensed. https://github.com/turanszkij/WickedEngine
// Removed: Thread priority, Dispatch, graceful shutdown; not needed in TinyBVH.
// A task is a raw function pointer plus an inline payload copy: pool-agnostic, with
// no per-task allocation.
class JobSystem
{
public:
	JobSystem() { Initialize(); }
	~JobSystem()
	{
		if (res.numThreads == 0) return;
		{ std::scoped_lock lock( res.sleepingMutex ); res.alive.store( false ); }
		res.sleepingCondition.notify_all();
		for (auto& thread : res.threads) if (thread.joinable()) thread.join();
		res.jobQueue.reset();
		res.threads.clear();
		res.numThreads = 0;
	}
	struct context { std::atomic<uint32_t> counter{ 0 }; } ctx;
	static const uint32_t JOB_PAYLOAD_MAX = 64; // builder payloads are <= 32B, + spawn header
	struct Job
	{
		void (*fn)(void*) = nullptr;
		uint8_t padding[8]; // avoid VS warning.
		alignas(16) uint8_t payload[JOB_PAYLOAD_MAX]; // inline copy; scheduling never allocates
		TINYBVH_FORCEINLINE uint32_t execute( context& ctx )
		{
			fn( payload );
			return ctx.counter.fetch_sub( 1 );
		}
	};
	struct JobQueue
	{
		std::deque<Job> queue;
		std::mutex locker;
		TINYBVH_FORCEINLINE void push_back( const Job& item ) { std::scoped_lock lock( locker ); queue.push_back( item ); }
		TINYBVH_FORCEINLINE bool pop_front( Job& item )
		{
			std::scoped_lock lock( locker );
			if (queue.empty()) return false; else item = std::move( queue.front() );
			queue.pop_front();
			return true;
		}
	};
	struct Resources
	{
		uint32_t numThreads = 0;
		std::vector<std::thread> threads;
		std::unique_ptr<JobQueue[]> jobQueue;
		std::atomic<uint32_t> nextQueue{ 0 };
		std::condition_variable sleepingCondition, waitingCondition;
		std::mutex sleepingMutex, waitingMutex;
		std::atomic_bool alive{ true };
		TINYBVH_FORCEINLINE void work( context& ctx, uint32_t startingQueue )
		{
			Job job;
			for (uint32_t i = 0; i < numThreads; ++i) while (jobQueue[startingQueue++ % numThreads].pop_front( job ))
				if (job.execute( ctx ) == 1) { std::unique_lock<std::mutex> lock( waitingMutex ); waitingCondition.notify_all(); }
		}
	} res;
	void Initialize()
	{
		res.numThreads = tinybvh_max( 1u, (uint32_t)std::thread::hardware_concurrency() );
		res.jobQueue.reset( new JobQueue[res.numThreads] );
		res.threads.reserve( res.numThreads );
		context& c = ctx;
		Resources& r = res;
		for (uint32_t threadID = 0; threadID < res.numThreads; threadID++)
		{
			std::thread& worker = res.threads.emplace_back( [&c, threadID, &r]
				{
					for (;;)
					{
						r.work( c, threadID );
						std::unique_lock<std::mutex> lock( r.sleepingMutex );
						if (!r.alive.load()) break;
						r.sleepingCondition.wait( lock );
					}
				} );
			auto handle = worker.native_handle();
			(void)handle;
		#if defined _WIN32 && defined _MSC_VER // TODO: get this working with gcc.
			// windows-specific thread setup
			SetThreadPriority( handle, 0 /* THREAD_PRIORITY_NORMAL */ );
			SetThreadDescription( handle, L"tinybvh::build" );
		#elif defined PLATFORM_LINUX
			// linux-specific thread setup
			char thread_name[16];
			snprintf( thread_name, sizeof( thread_name ), "tinybvh_%u", threadID );
			pthread_setname_np( handle, thread_name );
		#endif
		}
	}
	void Execute( void (*fn)(void*), const void* payload, uint32_t size )
	{
		assert( size <= JOB_PAYLOAD_MAX );
		ctx.counter.fetch_add( 1 );
		Job job;
		job.fn = fn;
		memcpy( job.payload, payload, size );
		res.jobQueue[res.nextQueue.fetch_add( 1 ) % res.numThreads].push_back( job );
		res.sleepingCondition.notify_one();
	}
	void Wait()
	{
		if (!IsBusy()) return;
		res.sleepingCondition.notify_all(); // wake any sleeping threads
		res.work( ctx, res.nextQueue.fetch_add( 1 ) % res.numThreads );
		while (IsBusy())
		{
			std::unique_lock<std::mutex> lock( res.waitingMutex );
			if (IsBusy()) res.waitingCondition.wait( lock, [this] { return !IsBusy(); } );
		}
	}
	bool IsBusy() { return ctx.counter.load( std::memory_order_relaxed ) > 0; }
};

// A build uses two job systems: 'subtree' for recursive fork/join (spawn/barrier)
// and 'binning' for the AVX builder's scoped leaf fan-out (parallel_for). One pair
// per host thread; 'binning' is created on demand (the scalar builders never bin).
struct PoolPair
{
	JobSystem subtree;
	std::unique_ptr<JobSystem> binningJobs;
	std::mutex binningMutex;
	JobSystem& binning()
	{
		std::scoped_lock lock( binningMutex );
		if (!binningJobs) binningJobs = std::make_unique<JobSystem>();
		return *binningJobs;
	}
};

// One pair per host thread, owned by that thread and freed when that thread exits.
static thread_local PoolPair* tinybvh_tl_pair = nullptr;

// Holds the pair this thread owns. Releasing clears tinybvh_tl_pair as well, so a
// later call can never be handed a pointer to an already-destroyed pool.
struct PoolPairOwner
{
	PoolPair* pair = nullptr;
	PoolPair* acquire() { if (!pair) pair = new PoolPair(); return pair; }
	void release()
	{
		if (!pair) return; // this thread never owned a pool; nothing to do
		PoolPair* p = pair;
		pair = nullptr;
		if (tinybvh_tl_pair == p) tinybvh_tl_pair = nullptr;
		delete p; // joins the worker threads of both job systems
	}
	~PoolPairOwner() { release(); }
};

static PoolPairOwner& tinybvh_pool_owner()
{
	static thread_local PoolPairOwner owner;
	return owner;
}
static PoolPair* tinybvh_pool_pair()
{
	if (!tinybvh_tl_pair) tinybvh_tl_pair = tinybvh_pool_owner().acquire();
	return tinybvh_tl_pair;
}
void tinybvh_shutdown_builtin_pool() { tinybvh_pool_owner().release(); }

// Spawned tasks run [pair][fn] ahead of the payload: the worker adopts 'pair' so its
// nested spawns reuse this host's pools. Keeps the pool/TLS logic out of JobSystem.
struct BVHSpawnEnvelope { PoolPair* pair; void (*fn)(void*); };
static void tinybvh_spawn_task( void* blob )
{
	BVHSpawnEnvelope* e = (BVHSpawnEnvelope*)blob;
	tinybvh_tl_pair = e->pair;
	e->fn( (uint8_t*)blob + sizeof( BVHSpawnEnvelope ) );
}
// Default BVHContext hooks (declared in tiny_bvh.h); userdata is unused.
void tinybvh_builtin_spawn( void (*fn)(void*), const void* payload, uint32_t payload_size, void* )
{
	// pack [pair][fn][payload]; Execute copies it inline (the caller's frame may unwind).
	PoolPair* pair = tinybvh_pool_pair();
	alignas(16) uint8_t blob[JobSystem::JOB_PAYLOAD_MAX];
	BVHSpawnEnvelope* e = (BVHSpawnEnvelope*)blob;
	e->pair = pair, e->fn = fn;
	memcpy( blob + sizeof( BVHSpawnEnvelope ), payload, payload_size );
	pair->subtree.Execute( &tinybvh_spawn_task, blob, sizeof( BVHSpawnEnvelope ) + payload_size );
}

void tinybvh_builtin_barrier( void* ) { tinybvh_pool_pair()->subtree.Wait(); }

struct BVHParallelForArgs { void (*fn)(uint32_t, void*); uint32_t index; void* payload; };
static void tinybvh_parallel_for_task( void* payload )
{
	BVHParallelForArgs* a = (BVHParallelForArgs*)payload;
	a->fn( a->index, a->payload );
}
void tinybvh_builtin_parallel_for( uint32_t n, void (*fn)(uint32_t, void*), void* payload, void* )
{
	if (n == 0) return;
	JobSystem& jobs = tinybvh_pool_pair()->binning();
	for (uint32_t i = 0; i < n; i++)
	{
		BVHParallelForArgs a = { fn, i, payload };
		jobs.Execute( &tinybvh_parallel_for_task, &a, sizeof( a ) );
	}
	jobs.Wait();
}

#endif

} // namespace tinybvh

#endif // TINYBVH_IMPLEMENTATION
