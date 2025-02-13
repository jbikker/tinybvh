/*
The MIT License (MIT)

Copyright (c) 2024, Jacco Bikker / Breda University of Applied Sciences.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

// How to use:
//
// Use this in *one* .c or .cpp
//   #define TINYBVH_IMPLEMENTATION
//   #include "tiny_bvh.h"
// Instantiate a BVH and build it for a list of triangles:
//   BVH bvh;
//   bvh.Build( (bvhvec4*)myVerts, numTriangles );
//   Ray ray( bvhvec3( 0, 0, 0 ), bvhvec3( 0, 0, 1 ), 1e30f );
//   bvh.Intersect( ray );
// After this, intersection information is in ray.hit.

// tinybvh can use custom vector types by defining TINYBVH_USE_CUSTOM_VECTOR_TYPES once before inclusion.
// To define custom vector types create a tinybvh namespace with the appropriate using directives, e.g.:
//	 namespace tinybvh
//   {
//     using bvhint2 = math::int2;
//     using bvhint3 = math::int3;
//     using bvhuint2 = math::uint2;
//     using bvhvec2 = math::float2;
//     using bvhvec3 = math::float3;
//     using bvhvec4 = math::float4;
//     using bvhdbl3 = math::double3;
//   }
//
//	 #define TINYBVH_USE_CUSTOM_VECTOR_TYPES
//   #include <tiny_bvh.h>

// tinybvh can be further configured using #defines, to be specified before the #include:
// #define BVHBINS 8        - the number of bins to use in regular BVH construction. Default is 8.
// #define HQBVHBINS 32     - the number of bins to use in SBVH construction. Default is 8.
// #define INST_IDX_BITS 10 - the number of bits to use for the instance index. Default is 32,
//                            which stores the bits in a separate field in tinybvh::Intersection.
// #define C_INT 1          - the estimated cost of a primitive intersection test. Default is 1.
// #define C_TRAV 1         - the estimated cost of a traversal step. Default is 1.

// See tiny_bvh_test.cpp for basic usage. In short:
// instantiate a BVH: tinybvh::BVH bvh;
// build it: bvh.Build( (tinybvh::bvhvec4*)triangleData, TRIANGLE_COUNT );
// ..where triangleData is an array of four-component float vectors:
// - For a single triangle, provide 3 vertices,
// - For each vertex provide x, y and z.
// The fourth float in each vertex is a dummy value and exists purely for
// a more efficient layout of the data in memory.

// More information about the BVH data structure:
// https://jacco.ompf2.com/2022/04/13/how-to-build-a-bvh-part-1-basics

// Further references: See README.md

// Author and contributors:
// Jacco Bikker: BVH code and examples
// Eddy L O Jansson: g++ / clang support
// Aras Pranckevičius: non-Intel architecture support
// Jefferson Amstutz: CMake support
// Christian Oliveros: WASM / EMSCRIPTEN support
// Thierry Cantenot: user-defined alloc & free
// David Peicho: slices & Rust bindings, API advice
// Aytek Aman: C++11 threading implementation

#ifndef TINY_BVH_H_
#define TINY_BVH_H_

// Run-time checks; disabled by default.
// #define PARANOID // checks out-of-bound access of slices
// #define SLICEDUMP // dumps the slice used for building to a file

// Binned BVH building: bin count.
#ifndef BVHBINS
#define BVHBINS 8
#endif
#ifndef HQBVHBINS
#define HQBVHBINS 8
#endif
#define AVXBINS 8 // must stay at 8.

// TLAS setting
// Note: Instance index is encoded in the top bits of the prim idx field.
// Max number of instances in TLAS: 2 ^ INST_IDX_BITS
// Max number of primitives per BLAS: 2 ^ (32 - INST_IDX_BITS)
#ifndef INST_IDX_BITS
#define INST_IDX_BITS 32 // Use 4..~12 to use prim field bits for instance id, or set to 32 to store index in separate field.
#endif
// Derived; for convenience:
#define INST_IDX_SHFT (32 - INST_IDX_BITS)
#if INST_IDX_BITS == 32
#define PRIM_IDX_MASK 0xffffffff // instance index stored separately.
#else
#define PRIM_IDX_MASK ((1 << INST_IDX_SHFT) - 1) // instance index stored in top bits of hit.prim.
#endif

// SAH BVH building: Heuristic parameters
// CPU traversal: C_INT = 1, C_TRAV = 1 seems optimal.
#ifndef C_INT
#define C_INT	1
#endif
#ifndef C_TRAV
#define C_TRAV	1
#endif

// SBVH: "Unsplitting"
#define SBVH_UNSPLITTING

// 'Infinity' values
#define BVH_FAR	1e30f		// actual valid ieee range: 3.40282347E+38
#define BVH_DBL_FAR 1e300	// actual valid ieee range: 1.797693134862315E+308

// Features
#ifndef NO_DOUBLE_PRECISION_SUPPORT
#define DOUBLE_PRECISION_SUPPORT
#endif
// #define TINYBVH_USE_CUSTOM_VECTOR_TYPES
// #define TINYBVH_NO_SIMD
#ifndef NO_INDEXED_GEOMETRY
#define ENABLE_INDEXED_GEOMETRY
#endif
#ifndef NO_CUSTOM_GEOMETRY
#define ENABLE_CUSTOM_GEOMETRY
#endif

// CWBVH triangle format: doesn't seem to help on GPU?
// #define CWBVH_COMPRESSED_TRIS
// BVH4 triangle format
// #define BVH4_GPU_COMPRESSED_TRIS

// We'll use this whenever a layout has no specialized shadow ray query.
#define FALLBACK_SHADOW_QUERY( s ) { Ray r = s; float d = s.hit.t; Intersect( r ); return r.hit.t < d; }

// include fast AVX BVH builder
#ifndef TINYBVH_NO_SIMD
#if defined(__x86_64__) || defined(_M_X64) || defined(__wasm_simd128__) || defined(__wasm_relaxed_simd__)
#define BVH_USEAVX
#include "immintrin.h" // for __m128 and __m256
#elif defined(__aarch64__) || defined(_M_ARM64)
#define BVH_USENEON
#include "arm_neon.h"
#endif
#endif // TINYBVH_NO_SIMD

// library version
#define TINY_BVH_VERSION_MAJOR	1
#define TINY_BVH_VERSION_MINOR	3
#define TINY_BVH_VERSION_SUB	6

// ============================================================================
//
//        P R E L I M I N A R I E S
//
// ============================================================================

// needful includes
#ifdef _MSC_VER // Visual Studio / C11
#include <malloc.h> // for alloc/free
#include <stdio.h> // for fprintf
#include <math.h> // for sqrtf, fabs
#include <string.h> // for memset
#include <stdlib.h> // for exit(1)
#else // Emscripten / gcc / clang
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <cstring>
#endif
#include <cstdint>

// aligned memory allocation
// note: formally size needs to be a multiple of 'alignment'. See:
// https://en.cppreference.com/w/c/memory/aligned_alloc
// EMSCRIPTEN enforces this.
// Copy of the same construct in tinyocl, different namespace.
namespace tinybvh {
inline size_t make_multiple_64( size_t x ) { return (x + 63) & ~0x3f; }
}
#ifdef _MSC_VER // Visual Studio / C11
#define ALIGNED( x ) __declspec( align( x ) )
namespace tinybvh {
inline void* malloc64( size_t size, void* = nullptr )
{
	return size == 0 ? 0 : _aligned_malloc( make_multiple_64( size ), 64 );
}
inline void free64( void* ptr, void* = nullptr ) { _aligned_free( ptr ); }
}
#else // EMSCRIPTEN / gcc / clang
#define ALIGNED( x ) __attribute__( ( aligned( x ) ) )
#if !defined(TINYBVH_NO_SIMD) && (defined(__x86_64__) || defined(_M_X64) || defined(__wasm_simd128__) || defined(__wasm_relaxed_simd__))
#include <xmmintrin.h>
namespace tinybvh {
inline void* malloc64( size_t size, void* = nullptr )
{
	return size == 0 ? 0 : _mm_malloc( make_multiple_64( size ), 64 );
}
inline void free64( void* ptr, void* = nullptr ) { _mm_free( ptr ); }
}
#else
namespace tinybvh {
inline void* malloc64( size_t size, void* = nullptr )
{
	return size == 0 ? 0 : aligned_alloc( 64, make_multiple_64( size ) );
}
inline void free64( void* ptr, void* = nullptr ) { free( ptr ); }
}
#endif
#endif

#ifdef _MSC_VER
#define __FORCEINLINE __forceinline
#else
#define __FORCEINLINE __attribute__((always_inline)) inline
#endif

namespace tinybvh {

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
	bvhvec4( const float a, const float b, const float c, const float d ) : x( a ), y( b ), z( c ), w( d ) {}
	bvhvec4( const float a ) : x( a ), y( a ), z( a ), w( a ) {}
	bvhvec4( const bvhvec3 & a );
	bvhvec4( const bvhvec3 & a, float b );
	float& operator [] ( const int32_t i ) { return cell[i]; }
	union { struct { float x, y, z, w; }; float cell[4]; };
};

struct ALIGNED( 8 ) bvhvec2
{
	bvhvec2() = default;
	bvhvec2( const float a, const float b ) : x( a ), y( b ) {}
	bvhvec2( const float a ) : x( a ), y( a ) {}
	bvhvec2( const bvhvec4 a ) : x( a.x ), y( a.y ) {}
	float& operator [] ( const int32_t i ) { return cell[i]; }
	union { struct { float x, y; }; float cell[2]; };
};

struct bvhvec3
{
	bvhvec3() = default;
	bvhvec3( const float a, const float b, const float c ) : x( a ), y( b ), z( c ) {}
	bvhvec3( const float a ) : x( a ), y( a ), z( a ) {}
	bvhvec3( const bvhvec4 a ) : x( a.x ), y( a.y ), z( a.z ) {}
	float halfArea() { return x < -BVH_FAR ? 0 : (x * y + y * z + z * x); } // for SAH calculations
	float& operator [] ( const int32_t i ) { return cell[i]; }
	union { struct { float x, y, z; }; float cell[3]; };
};

struct bvhint3
{
	bvhint3() = default;
	bvhint3( const int32_t a, const int32_t b, const int32_t c ) : x( a ), y( b ), z( c ) {}
	bvhint3( const int32_t a ) : x( a ), y( a ), z( a ) {}
	bvhint3( const bvhvec3& a ) { x = (int32_t)a.x, y = (int32_t)a.y, z = (int32_t)a.z; }
	int32_t& operator [] ( const int32_t i ) { return cell[i]; }
	union { struct { int32_t x, y, z; }; int32_t cell[3]; };
};

struct bvhint2
{
	bvhint2() = default;
	bvhint2( const int32_t a, const int32_t b ) : x( a ), y( b ) {}
	bvhint2( const int32_t a ) : x( a ), y( a ) {}
	int32_t x, y;
};

struct bvhuint2
{
	bvhuint2() = default;
	bvhuint2( const uint32_t a, const uint32_t b ) : x( a ), y( b ) {}
	bvhuint2( const uint32_t a ) : x( a ), y( a ) {}
	uint32_t x, y;
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
	operator bool() const { return !!data; }
	const bvhvec4& operator [] ( size_t i ) const;
	const int8_t* data = nullptr;
	uint32_t count, stride;
};

// Math operations.
// Note: Since this header file is expected to be included in a source file
// of a separate project, the static keyword doesn't provide sufficient
// isolation; hence the tinybvh_ prefix.
inline float tinybvh_safercp( const float x ) { return x > 1e-12f ? (1.0f / x) : (x < -1e-12f ? (1.0f / x) : BVH_FAR); }
inline bvhvec3 tinybvh_safercp( const bvhvec3 a ) { return bvhvec3( tinybvh_safercp( a.x ), tinybvh_safercp( a.y ), tinybvh_safercp( a.z ) ); }
inline float tinybvh_min( const float a, const float b ) { return a < b ? a : b; }
inline float tinybvh_max( const float a, const float b ) { return a > b ? a : b; }
inline double tinybvh_min( const double a, const double b ) { return a < b ? a : b; }
inline double tinybvh_max( const double a, const double b ) { return a > b ? a : b; }
inline int32_t tinybvh_min( const int32_t a, const int32_t b ) { return a < b ? a : b; }
inline int32_t tinybvh_max( const int32_t a, const int32_t b ) { return a > b ? a : b; }
inline uint32_t tinybvh_min( const uint32_t a, const uint32_t b ) { return a < b ? a : b; }
inline uint32_t tinybvh_max( const uint32_t a, const uint32_t b ) { return a > b ? a : b; }
inline bvhvec3 tinybvh_min( const bvhvec3& a, const bvhvec3& b ) { return bvhvec3( tinybvh_min( a.x, b.x ), tinybvh_min( a.y, b.y ), tinybvh_min( a.z, b.z ) ); }
inline bvhvec4 tinybvh_min( const bvhvec4& a, const bvhvec4& b ) { return bvhvec4( tinybvh_min( a.x, b.x ), tinybvh_min( a.y, b.y ), tinybvh_min( a.z, b.z ), tinybvh_min( a.w, b.w ) ); }
inline bvhvec3 tinybvh_max( const bvhvec3& a, const bvhvec3& b ) { return bvhvec3( tinybvh_max( a.x, b.x ), tinybvh_max( a.y, b.y ), tinybvh_max( a.z, b.z ) ); }
inline bvhvec4 tinybvh_max( const bvhvec4& a, const bvhvec4& b ) { return bvhvec4( tinybvh_max( a.x, b.x ), tinybvh_max( a.y, b.y ), tinybvh_max( a.z, b.z ), tinybvh_max( a.w, b.w ) ); }
inline float tinybvh_clamp( const float x, const float a, const float b ) { return x > a ? (x < b ? x : b) : a; /* NaN safe */ }
inline int32_t tinybvh_clamp( const int32_t x, const int32_t a, const int32_t b ) { return x > a ? (x < b ? x : b) : a; /* NaN safe */ }
template <class T> inline static void tinybvh_swap( T& a, T& b ) { T t = a; a = b; b = t; }

// Operator overloads.
// Only a minimal set is provided.
#ifndef TINYBVH_USE_CUSTOM_VECTOR_TYPES

inline bvhvec2 operator-( const bvhvec2& a ) { return bvhvec2( -a.x, -a.y ); }
inline bvhvec3 operator-( const bvhvec3& a ) { return bvhvec3( -a.x, -a.y, -a.z ); }
inline bvhvec4 operator-( const bvhvec4& a ) { return bvhvec4( -a.x, -a.y, -a.z, -a.w ); }
inline bvhvec2 operator+( const bvhvec2& a, const bvhvec2& b ) { return bvhvec2( a.x + b.x, a.y + b.y ); }
inline bvhvec3 operator+( const bvhvec3& a, const bvhvec3& b ) { return bvhvec3( a.x + b.x, a.y + b.y, a.z + b.z ); }
inline bvhvec4 operator+( const bvhvec4& a, const bvhvec4& b ) { return bvhvec4( a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w ); }
inline bvhvec4 operator+( const bvhvec4& a, const bvhvec3& b ) { return bvhvec4( a.x + b.x, a.y + b.y, a.z + b.z, a.w ); }
inline bvhvec2 operator-( const bvhvec2& a, const bvhvec2& b ) { return bvhvec2( a.x - b.x, a.y - b.y ); }
inline bvhvec3 operator-( const bvhvec3& a, const bvhvec3& b ) { return bvhvec3( a.x - b.x, a.y - b.y, a.z - b.z ); }
inline bvhvec4 operator-( const bvhvec4& a, const bvhvec4& b ) { return bvhvec4( a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w ); }
inline void operator+=( bvhvec2& a, const bvhvec2& b ) { a.x += b.x; a.y += b.y; }
inline void operator+=( bvhvec3& a, const bvhvec3& b ) { a.x += b.x; a.y += b.y; a.z += b.z; }
inline void operator+=( bvhvec4& a, const bvhvec4& b ) { a.x += b.x; a.y += b.y; a.z += b.z; a.w += b.w; }
inline bvhvec2 operator*( const bvhvec2& a, const bvhvec2& b ) { return bvhvec2( a.x * b.x, a.y * b.y ); }
inline bvhvec3 operator*( const bvhvec3& a, const bvhvec3& b ) { return bvhvec3( a.x * b.x, a.y * b.y, a.z * b.z ); }
inline bvhvec4 operator*( const bvhvec4& a, const bvhvec4& b ) { return bvhvec4( a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w ); }
inline bvhvec2 operator*( const bvhvec2& a, float b ) { return bvhvec2( a.x * b, a.y * b ); }
inline bvhvec3 operator*( const bvhvec3& a, float b ) { return bvhvec3( a.x * b, a.y * b, a.z * b ); }
inline bvhvec4 operator*( const bvhvec4& a, float b ) { return bvhvec4( a.x * b, a.y * b, a.z * b, a.w * b ); }
inline bvhvec2 operator*( float b, const bvhvec2& a ) { return bvhvec2( b * a.x, b * a.y ); }
inline bvhvec3 operator*( float b, const bvhvec3& a ) { return bvhvec3( b * a.x, b * a.y, b * a.z ); }
inline bvhvec4 operator*( float b, const bvhvec4& a ) { return bvhvec4( b * a.x, b * a.y, b * a.z, b * a.w ); }
inline bvhvec2 operator/( float b, const bvhvec2& a ) { return bvhvec2( b / a.x, b / a.y ); }
inline bvhvec3 operator/( float b, const bvhvec3& a ) { return bvhvec3( b / a.x, b / a.y, b / a.z ); }
inline bvhvec4 operator/( float b, const bvhvec4& a ) { return bvhvec4( b / a.x, b / a.y, b / a.z, b / a.w ); }
inline void operator*=( bvhvec3& a, const float b ) { a.x *= b; a.y *= b; a.z *= b; }

#endif // TINYBVH_USE_CUSTOM_VECTOR_TYPES

// Vector math: cross and dot.
inline bvhvec3 tinybvh_cross( const bvhvec3& a, const bvhvec3& b )
{
	return bvhvec3( a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x );
}
inline float tinybvh_dot( const bvhvec2& a, const bvhvec2& b ) { return a.x * b.x + a.y * b.y; }
inline float tinybvh_dot( const bvhvec3& a, const bvhvec3& b ) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline float tinybvh_dot( const bvhvec4& a, const bvhvec4& b ) { return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w; }

// Vector math: common operations.
inline float tinybvh_length( const bvhvec3& a ) { return sqrtf( a.x * a.x + a.y * a.y + a.z * a.z ); }
inline bvhvec3 tinybvh_normalize( const bvhvec3& a )
{
	float l = tinybvh_length( a ), rl = l == 0 ? 0 : (1.0f / l);
	return a * rl;
}
inline bvhvec3 tinybvh_transform_point( const bvhvec3& v, const float* T )
{
	const bvhvec3 res(
		T[0] * v.x + T[1] * v.y + T[2] * v.z + T[3],
		T[4] * v.x + T[5] * v.y + T[6] * v.z + T[7],
		T[8] * v.x + T[9] * v.y + T[10] * v.z + T[11] );
	const float w = T[12] * v.x + T[13] * v.y + T[14] * v.z + T[15];
	if (w == 1) return res; else return res * (1.f / w);
}
inline bvhvec3 tinybvh_transform_vector( const bvhvec3& v, const float* T )
{
	return bvhvec3( T[0] * v.x + T[1] * v.y + T[2] * v.z, T[4] * v.x +
		T[5] * v.y + T[6] * v.z, T[8] * v.x + T[9] * v.y + T[10] * v.z );
}

#ifdef DOUBLE_PRECISION_SUPPORT
// Double-precision math

#ifndef TINYBVH_USE_CUSTOM_VECTOR_TYPES

struct bvhdbl3
{
	bvhdbl3() = default;
	bvhdbl3( const double a, const double b, const double c ) : x( a ), y( b ), z( c ) {}
	bvhdbl3( const double a ) : x( a ), y( a ), z( a ) {}
	bvhdbl3( const bvhvec3 a ) : x( (double)a.x ), y( (double)a.y ), z( (double)a.z ) {}
	double halfArea() { return x < -BVH_DBL_FAR ? 0 : (x * y + y * z + z * x); } // for SAH calculations
	double& operator [] ( const int32_t i ) { return cell[i]; }
	union { struct { double x, y, z; }; double cell[3]; };
};

#endif // TINYBVH_USE_CUSTOM_VECTOR_TYPES

#ifdef _MSC_VER
#pragma warning ( pop )
#endif

inline bvhdbl3 tinybvh_min( const bvhdbl3& a, const bvhdbl3& b ) { return bvhdbl3( tinybvh_min( a.x, b.x ), tinybvh_min( a.y, b.y ), tinybvh_min( a.z, b.z ) ); }
inline bvhdbl3 tinybvh_max( const bvhdbl3& a, const bvhdbl3& b ) { return bvhdbl3( tinybvh_max( a.x, b.x ), tinybvh_max( a.y, b.y ), tinybvh_max( a.z, b.z ) ); }

#ifndef TINYBVH_USE_CUSTOM_VECTOR_TYPES

inline bvhdbl3 operator-( const bvhdbl3& a ) { return bvhdbl3( -a.x, -a.y, -a.z ); }
inline bvhdbl3 operator+( const bvhdbl3& a, const bvhdbl3& b ) { return bvhdbl3( a.x + b.x, a.y + b.y, a.z + b.z ); }
inline bvhdbl3 operator-( const bvhdbl3& a, const bvhdbl3& b ) { return bvhdbl3( a.x - b.x, a.y - b.y, a.z - b.z ); }
inline void operator+=( bvhdbl3& a, const bvhdbl3& b ) { a.x += b.x; a.y += b.y; a.z += b.z; }
inline bvhdbl3 operator*( const bvhdbl3& a, const bvhdbl3& b ) { return bvhdbl3( a.x * b.x, a.y * b.y, a.z * b.z ); }
inline bvhdbl3 operator*( const bvhdbl3& a, double b ) { return bvhdbl3( a.x * b, a.y * b, a.z * b ); }
inline bvhdbl3 operator*( double b, const bvhdbl3& a ) { return bvhdbl3( b * a.x, b * a.y, b * a.z ); }
inline bvhdbl3 operator/( double b, const bvhdbl3& a ) { return bvhdbl3( b / a.x, b / a.y, b / a.z ); }
inline bvhdbl3 operator*=( bvhdbl3& a, const double b ) { return bvhdbl3( a.x * b, a.y * b, a.z * b ); }

#endif // TINYBVH_USE_CUSTOM_VECTOR_TYPES

inline double tinybvh_length( const bvhdbl3& a ) { return sqrt( a.x * a.x + a.y * a.y + a.z * a.z ); }
inline bvhdbl3 tinybvh_normalize( const bvhdbl3& a )
{
	double l = tinybvh_length( a ), rl = l == 0 ? 0 : (1.0 / l);
	return a * rl;
}
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

inline bvhdbl3 tinybvh_cross( const bvhdbl3& a, const bvhdbl3& b )
{
	return bvhdbl3( a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x );
}
inline double tinybvh_dot( const bvhdbl3& a, const bvhdbl3& b ) { return a.x * b.x + a.y * b.y + a.z * b.z; }

#endif // DOUBLE_PRECISION_SUPPORT

// SIMD typedef, helps keeping the interface generic
#ifdef BVH_USEAVX
typedef __m128 SIMDVEC4;
typedef __m256 SIMDVEC8;
#define SIMD_SETVEC(a,b,c,d) _mm_set_ps( a, b, c, d )
#define SIMD_SETRVEC(a,b,c,d) _mm_set_ps( d, c, b, a )
#elif defined(BVH_USENEON)
typedef float32x4_t SIMDVEC4;
typedef float32x4x2_t SIMDVEC8;
inline float32x4_t SIMD_SETVEC( float w, float z, float y, float x )
{
	ALIGNED( 64 ) float data[4] = { x, y, z, w };
	return vld1q_f32( data );
}
inline float32x4_t SIMD_SETRVEC( float x, float y, float z, float w )
{
	ALIGNED( 64 ) float data[4] = { x, y, z, w };
	return vld1q_f32( data );
}
inline uint32x4_t SIMD_SETRVECU( uint32_t x, uint32_t y, uint32_t z, uint32_t w )
{
	ALIGNED( 64 ) uint32_t data[4] = { x, y, z, w };
	return vld1q_u32( data );
}
#else
typedef bvhvec4 SIMDVEC4;
#define SIMD_SETVEC(a,b,c,d) bvhvec4( d, c, b, a )
#define SIMD_SETRVEC(a,b,c,d) bvhvec4( a, b, c, d )
#endif

// error handling
#define FATAL_ERROR(s) FATAL_ERROR_IF(1,s)
#define FATAL_ERROR_IF(c,s) if (c) { fprintf( stderr, \
	"Fatal error in tiny_bvh.h, line %i:\n%s\n", __LINE__, s ); exit( 1 ); }

// ============================================================================
//
//        T I N Y _ B V H   I N T E R F A C E
//
// ============================================================================

#if defined(_MSC_VER) || defined(__GNUC__)
#pragma pack(push, 4) // is there a good alternative for Clang / EMSCRIPTEN?
#endif
struct Intersection
{
	// An intersection result is designed to fit in no more than
	// four 32-bit values. This allows efficient storage of a result in
	// GPU code. The obvious missing result is an instance id; consider
	// squeezing this in the 'prim' field in some way.
	// Using this data and the original triangle data, all other info for
	// shading (such as normal, texture color etc.) can be reconstructed.
#if INST_IDX_BITS == 32
	uint32_t inst;	// instance index. Stored in top bits of prim if INST_IDX_BITS != 32.
#endif
	float t, u, v;	// distance along ray & barycentric coordinates of the intersection
	uint32_t prim;	// primitive index
	// 64 byte of custom data -
	// assuming struct Ray is aligned, this starts at a cache line boundary.
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
#if defined(_MSC_VER) || defined(__GNUC__)
#pragma pack(pop) // is there a good alternative for Clang / EMSCRIPTEN?
#endif

struct ALIGNED( 64 ) Ray
{
	// Basic ray class. Note: For single blas traversal it is expected
	// that Ray::rD is properly initialized. For tlas/blas traversal this
	// field is typically updated for each blas.
	Ray() = default;
	Ray( bvhvec3 origin, bvhvec3 direction, float t = BVH_FAR )
	{
		memset( this, 0, sizeof( Ray ) );
		O = origin, D = tinybvh_normalize( direction ), rD = tinybvh_safercp( D );
		hit.t = t;
	}
	ALIGNED( 16 ) bvhvec3 O; uint32_t dummy1;
	ALIGNED( 16 ) bvhvec3 D; uint32_t instIdx = 0;
	ALIGNED( 16 ) bvhvec3 rD;
#if INST_IDX_BITS != 32
	uint32_t dummy2; // align to 16 bytes if field 'hit' is 16 bytes; otherwise don't.
#endif
	Intersection hit;
};

#ifdef DOUBLE_PRECISION_SUPPORT

struct IntersectionEx
{
	// Double-precision hit record.
	double t, u, v;	// distance along ray & barycentric coordinates of the intersection
	uint64_t inst, prim; // instance and primitive index
};

struct RayEx
{
	// Double-precision ray definition.
	RayEx() = default;
	RayEx( bvhdbl3 origin, bvhdbl3 direction, double tmax = BVH_DBL_FAR )
	{
		memset( this, 0, sizeof( RayEx ) );
		O = origin, D = direction;
		double rl = 1.0 / sqrt( D.x * D.x + D.y * D.y + D.z * D.z );
		D.x *= rl, D.y *= rl, D.z *= rl;
		rD.x = 1.0 / D.x, rD.y = 1.0 / D.y, rD.z = 1.0 / D.z;
		hit.u = hit.v = 0, hit.t = tmax;
	}
	bvhdbl3 O, D, rD;
	IntersectionEx hit;
	uint64_t instIdx = 0;
};

#endif

struct BVHContext
{
	void* (*malloc)(size_t size, void* userdata) = malloc64;
	void (*free)(void* ptr, void* userdata) = free64;
	void* userdata = nullptr;
};

enum TraceDevice : uint32_t { USE_CPU = 1, USE_GPU };

class BVHBase
{
public:
	enum BVHType : uint32_t
	{
		// Every BVHJ class is derived from BVHBase, but we don't use virtual functions, for
		// performance reasons. For a TLAS over a mix of BVH layouts we do however need this
		// kind of behavior when transitioning from a TLAS leaf to a BLAS root node.
		UNDEFINED = 0,
		LAYOUT_BVH = 1,
		LAYOUT_BVH_VERBOSE,
		LAYOUT_BVH_DOUBLE,
		LAYOUT_BVH_SOA,
		LAYOUT_BVH_GPU,
		LAYOUT_MBVH,
		LAYOUT_BVH4_CPU,
		LAYOUT_BVH4_GPU,
		LAYOUT_MBVH8,
		LAYOUT_CWBVH,
		LAYOUT_BVH4_AVX2
	};
	struct ALIGNED( 32 ) Fragment
	{
		// A fragment stores the bounds of an input primitive. The name 'Fragment' is from
		// "Parallel Spatial Splits in Bounding Volume Hierarchies", 2016, Fuetterling et al.,
		// and refers to the potential splitting of these boxes for SBVH construction.
		bvhvec3 bmin;				// AABB min x, y and z
		uint32_t primIdx;			// index of the original primitive
		bvhvec3 bmax;				// AABB max x, y and z
		uint32_t clipped = 0;		// Fragment is the result of clipping if > 0.
		bool validBox() { return bmin.x < BVH_FAR; }
	};
	// BVH flags, maintainted by tiny_bvh.
	bool rebuildable = true;		// rebuilds are safe only if a tree has not been converted.
	bool refittable = true;			// refits are safe only if the tree has no spatial splits.
	bool may_have_holes = false;	// threaded builds and MergeLeafs produce BVHs with unused nodes.
	bool bvh_over_aabbs = false;	// a BVH over AABBs is useful for e.g. TLAS traversal.
	bool bvh_over_indices = false;	// a BVH over indices cannot translate primitive index to vertex index.
	BVHContext context;				// context used to provide user-defined allocation functions
	BVHType layout = UNDEFINED;		// BVH layout identifier
	// Keep track of allocated buffer size to avoid repeated allocation during layout conversion.
	uint32_t allocatedNodes = 0;	// number of nodes allocated for the BVH.
	uint32_t usedNodes = 0;			// number of nodes used for the BVH.
	uint32_t triCount = 0;			// number of primitives in the BVH.
	uint32_t idxCount = 0;			// number of primitive indices; can exceed triCount for SBVH.
	bvhvec3 aabbMin, aabbMax;		// bounds of the root node of the BVH.
	// Custom memory allocation
	void* AlignedAlloc( size_t size );
	void AlignedFree( void* ptr );
	// Common methods
	void CopyBasePropertiesFrom( const BVHBase& original );	// copy flags from one BVH to another
protected:
	__FORCEINLINE void IntersectTri( Ray& ray, const bvhvec4slice& verts, const uint32_t primIdx ) const;
	__FORCEINLINE void IntersectTriIndexed( Ray& ray, const bvhvec4slice& verts, const uint32_t* indices, const uint32_t idx ) const;
	__FORCEINLINE bool TriOccludes( const Ray& ray, const bvhvec4slice& verts, const uint32_t idx ) const;
	__FORCEINLINE bool IndexedTriOccludes( const Ray& ray, const bvhvec4slice& verts, const uint32_t* indices, const uint32_t idx ) const;
	static float IntersectAABB( const Ray& ray, const bvhvec3& aabbMin, const bvhvec3& aabbMax );
	static void PrecomputeTriangle( const bvhvec4slice& vert, const uint32_t ti0, const uint32_t ti1, const uint32_t ti2, float* T );
	static float SA( const bvhvec3& aabbMin, const bvhvec3& aabbMax );
};

class BLASInstance;
class BVH_Verbose;
class BVH : public BVHBase
{
public:
	friend class BVH_GPU;
	friend class BVH_SoA;
	template <int M> friend class MBVH;
	enum BuildFlags : uint32_t
	{
		NONE = 0,			// Default building behavior (binned, SAH-driven).
		FULLSPLIT = 1		// Split as far as possible, even when SAH doesn't agree.
	};
	struct BVHNode
	{
		// 'Traditional' 32-byte BVH node layout, as proposed by Ingo Wald.
		// When aligned to a cache line boundary, two of these fit together.
		bvhvec3 aabbMin; uint32_t leftFirst; // 16 bytes
		bvhvec3 aabbMax; uint32_t triCount;	// 16 bytes, total: 32 bytes
		bool isLeaf() const { return triCount > 0; /* empty BVH leaves do not exist */ }
		float Intersect( const Ray& ray ) const { return BVH::IntersectAABB( ray, aabbMin, aabbMax ); }
		bool Intersect( const bvhvec3& bmin, const bvhvec3& bmax ) const;
		float SurfaceArea() const { return BVH::SA( aabbMin, aabbMax ); }
	};
	BVH( BVHContext ctx = {} ) { layout = LAYOUT_BVH; context = ctx; }
	BVH( const BVH_Verbose& original ) { layout = LAYOUT_BVH; ConvertFrom( original ); }
	BVH( const bvhvec4* vertices, const uint32_t primCount ) { layout = LAYOUT_BVH; Build( vertices, primCount ); }
	BVH( const bvhvec4slice& vertices ) { layout = LAYOUT_BVH; Build( vertices ); }
	~BVH();
	void ConvertFrom( const BVH_Verbose& original, bool compact = true );
	float SAHCost( const uint32_t nodeIdx = 0 ) const;
	int32_t NodeCount() const;
	int32_t PrimCount( const uint32_t nodeIdx = 0 ) const;
	void Compact();
	void Save( const char* fileName );
	bool Load( const char* fileName, const bvhvec4* vertices, const uint32_t primCount );
	bool Load( const char* fileName, const bvhvec4* vertices, const uint32_t* indices, const uint32_t primCount );
	bool Load( const char* fileName, const bvhvec4slice& vertices, const uint32_t* indices = 0, const uint32_t primCount = 0 );
	void BuildQuick( const bvhvec4* vertices, const uint32_t primCount );
	void BuildQuick( const bvhvec4slice& vertices );
	void Build( const bvhvec4* vertices, const uint32_t primCount );
	void Build( const bvhvec4slice& vertices );
	void Build( const bvhvec4* vertices, const uint32_t* indices, const uint32_t primCount );
	void Build( const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t primCount );
	void Build( BLASInstance* instances, const uint32_t instCount, BVHBase** blasses, const uint32_t blasCount );
	void Build( void (*customGetAABB)(const unsigned, bvhvec3&, bvhvec3&), const uint32_t primCount );
	void BuildHQ( const bvhvec4* vertices, const uint32_t primCount );
	void BuildHQ( const bvhvec4slice& vertices );
	void BuildHQ( const bvhvec4* vertices, const uint32_t* indices, const uint32_t primCount );
	void BuildHQ( const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t primCount );
#ifdef BVH_USEAVX
	void BuildAVX( const bvhvec4* vertices, const uint32_t primCount );
	void BuildAVX( const bvhvec4slice& vertices );
	void BuildAVX( const bvhvec4* vertices, const uint32_t* indices, const uint32_t primCount );
	void BuildAVX( const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t primCount );
#endif
	void Refit( const uint32_t nodeIdx = 0 );
	void Optimize( const uint32_t iterations = 25, bool extreme = false );
	int32_t Intersect( Ray& ray ) const;
	bool IntersectSphere( const bvhvec3& pos, const float r ) const;
	bool IsOccluded( const Ray& ray ) const;
	void Intersect256Rays( Ray* first ) const;
	void Intersect256RaysSSE( Ray* packet ) const; // requires BVH_USEAVX
	// private:
	void PrepareBuild( const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t primCount );
	void Build();
	bool IsOccludedTLAS( const Ray& ray ) const;
	int32_t IntersectTLAS( Ray& ray ) const;
	void PrepareAVXBuild( const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t primCount );
	void BuildAVX();
	void PrepareHQBuild( const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t prims );
	void BuildHQ();
	bool ClipFrag( const Fragment& orig, Fragment& newFrag, bvhvec3 bmin, bvhvec3 bmax, bvhvec3 minDim, const uint32_t splitAxis );
	void SplitFrag( const Fragment& orig, Fragment& left, Fragment& right, const bvhvec3& minDim, const uint32_t splitAxis, const float splitPos, bool& leftOK, bool& rightOK );
protected:
	void BuildDefault( const bvhvec4* vertices, const uint32_t primCount );
	void BuildDefault( const bvhvec4slice& vertices );
	void BuildDefault( const bvhvec4* vertices, const uint32_t* indices, const uint32_t primCount );
	void BuildDefault( const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t primCount );
public:
	// BVH type identification
	bool isTLAS() const { return instList != 0; }
	bool isBLAS() const { return instList == 0; }
	bool isIndexed() const { return vertIdx != 0; }
	bool hasCustomGeom() const { return customIntersect != 0; }
	// Basic BVH data
	bvhvec4slice verts = {};		// pointer to input primitive array: 3x16 bytes per tri.
	uint32_t* vertIdx = 0;			// vertex indices, only used in case the BVH is built over indexed prims.
	uint32_t* primIdx = 0;			// primitive index array.
	BLASInstance* instList = 0;		// instance array, for top-level acceleration structure.
	BVHBase** blasList = 0;			// blas array, for TLAS traversal.
	uint32_t blasCount = 0;			// number of blasses in blasList.
	BVHNode* bvhNode = 0;			// BVH node pool, Wald 32-byte format. Root is always in node 0.
	uint32_t newNodePtr = 0;		// used during build to keep track of next free node in pool.
	Fragment* fragment = 0;			// input primitive bounding boxes.
	// Custom geometry intersection callback
	bool (*customIntersect)(Ray&, const unsigned) = 0;
	bool (*customIsOccluded)(const Ray&, const unsigned) = 0;
};

#ifdef DOUBLE_PRECISION_SUPPORT

class BLASInstanceEx;
class BVH_Double : public BVHBase
{
public:
	struct BVHNode
	{
		// Double precision 'traditional' BVH node layout.
		// Compared to the default BVHNode, child node indices and triangle indices
		// are also expanded to 64bit values to support massive scenes.
		bvhdbl3 aabbMin, aabbMax; // 2x24 bytes
		uint64_t leftFirst; // 8 bytes
		uint64_t triCount; // 8 bytes, total: 64 bytes
		bool isLeaf() const { return triCount > 0; /* empty BVH leaves do not exist */ }
		double Intersect( const RayEx& ray ) const;
		double SurfaceArea() const;
	};
	struct Fragment
	{
		// Double-precision version of the fragment sruct.
		bvhdbl3 bmin, bmax;			// AABB
		uint64_t primIdx;			// index of the original primitive
	};
	BVH_Double( BVHContext ctx = {} ) { layout = LAYOUT_BVH_DOUBLE; context = ctx; }
	~BVH_Double();
	void Build( const bvhdbl3* vertices, const uint64_t primCount );
	void Build( BLASInstanceEx* bvhs, const uint64_t instCount, BVH_Double** blasses, const uint64_t blasCount );
	void Build( void (*customGetAABB)(const uint64_t, bvhdbl3&, bvhdbl3&), const uint64_t primCount );
	void PrepareBuild( const bvhdbl3* vertices, const uint64_t primCount );
	void Build();
	double SAHCost( const uint64_t nodeIdx = 0 ) const;
	int32_t Intersect( RayEx& ray ) const;
	bool IsOccluded( const RayEx& ray ) const;
	bool IsOccludedTLAS( const RayEx& ray ) const;
	int32_t IntersectTLAS( RayEx& ray ) const;
	bvhdbl3* verts = 0;				// pointer to input primitive array, double-precision, 3x24 bytes per tri.
	Fragment* fragment = 0;			// input primitive bounding boxes, double-precision.
	BVHNode* bvhNode = 0;			// BVH node, double precision format.
	uint64_t* primIdx = 0;			// primitive index array for double-precision bvh.
	BLASInstanceEx* instList = 0;	// instance array, for top-level acceleration structure.
	BVH_Double** blasList = 0;		// blas array, for TLAS traversal.
	uint64_t blasCount = 0;			// number of blasses in blasList.
	// 64-bit base overrides
	uint64_t newNodePtr = 0;		// next free bvh pool entry to allocate
	uint64_t usedNodes = 0;			// number of nodes used for the BVH.
	uint64_t allocatedNodes = 0;	// number of nodes allocated for the BVH.
	uint64_t triCount = 0;			// number of primitives in the BVH.
	uint64_t idxCount = 0;			// number of primitive indices.
	bvhdbl3 aabbMin, aabbMax;		// bounds of the root node of the BVH.
	// Custom geometry intersection callback
	bool (*customIntersect)(RayEx&, uint64_t) = 0;
	bool (*customIsOccluded)(const RayEx&, uint64_t) = 0;
};

#endif // DOUBLE_PRECISION_SUPPORT

class BVH_GPU : public BVHBase
{
public:
	struct BVHNode
	{
		// Alternative 64-byte BVH node layout, which specifies the bounds of
		// the children rather than the node itself. This layout is used by
		// Aila and Laine in their seminal GPU ray tracing paper.
		bvhvec3 lmin; uint32_t left;
		bvhvec3 lmax; uint32_t right;
		bvhvec3 rmin; uint32_t triCount;
		bvhvec3 rmax; uint32_t firstTri; // total: 64 bytes
		bool isLeaf() const { return triCount > 0; }
	};
	BVH_GPU( BVHContext ctx = {} ) { layout = LAYOUT_BVH_GPU; context = ctx; }
	BVH_GPU( const BVH& original ) { /* DEPRICATED */ ConvertFrom( original ); }
	~BVH_GPU();
	void Build( const bvhvec4* vertices, const uint32_t primCount );
	void Build( const bvhvec4slice& vertices );
	void Build( const bvhvec4* vertices, const uint32_t* indices, const uint32_t primCount );
	void Build( const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t primCount );
	void Build( BLASInstance* instances, const uint32_t instCount, BVHBase** blasses, const uint32_t blasCount );
	void BuildHQ( const bvhvec4* vertices, const uint32_t primCount );
	void BuildHQ( const bvhvec4slice& vertices );
	void BuildHQ( const bvhvec4* vertices, const uint32_t* indices, const uint32_t primCount );
	void BuildHQ( const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t primCount );
	void Optimize( const uint32_t iterations = 25, bool extreme = false );
	float SAHCost( const uint32_t nodeIdx = 0 ) const { return bvh.SAHCost( nodeIdx ); }
	void ConvertFrom( const BVH& original, bool compact = true );
	int32_t Intersect( Ray& ray ) const;
	bool IsOccluded( const Ray& ray ) const { FALLBACK_SHADOW_QUERY( ray ); }
	// BVH data
	BVHNode* bvhNode = 0;			// BVH node in Aila & Laine format.
	BVH bvh;						// BVH4 is created from BVH and uses its data.
	bool ownBVH = true;				// False when ConvertFrom receives an external bvh.
};

class BVH_SoA : public BVHBase
{
public:
	struct BVHNode
	{
		// Second alternative 64-byte BVH node layout, same as BVHAilaLaine but
		// with child AABBs stored in SoA order.
		SIMDVEC4 xxxx, yyyy, zzzz;
		uint32_t left, right, triCount, firstTri; // total: 64 bytes
		bool isLeaf() const { return triCount > 0; }
	};
	BVH_SoA( BVHContext ctx = {} ) { layout = LAYOUT_BVH_SOA; context = ctx; }
	BVH_SoA( const BVH& original ) { /* DEPRICATED */ layout = LAYOUT_BVH_SOA; ConvertFrom( original ); }
	~BVH_SoA();
	void Build( const bvhvec4* vertices, const uint32_t primCount );
	void Build( const bvhvec4slice& vertices );
	void Build( const bvhvec4* vertices, const uint32_t* indices, const uint32_t primCount );
	void Build( const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t primCount );
	void BuildHQ( const bvhvec4* vertices, const uint32_t primCount );
	void BuildHQ( const bvhvec4slice& vertices );
	void BuildHQ( const bvhvec4* vertices, const uint32_t* indices, const uint32_t primCount );
	void BuildHQ( const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t primCount );
	void Optimize( const uint32_t iterations = 25, bool extreme = false );
	float SAHCost( const uint32_t nodeIdx = 0 ) const { return bvh.SAHCost( nodeIdx ); }
	void Save( const char* fileName );
	bool Load( const char* fileName, const bvhvec4* vertices, const uint32_t primCount );
	bool Load( const char* fileName, const bvhvec4* vertices, const uint32_t* indices, const uint32_t primCount );
	bool Load( const char* fileName, const bvhvec4slice& vertices, const uint32_t* indices = 0, const uint32_t primCount = 0 );
	void ConvertFrom( const BVH& original, bool compact = true );
	int32_t Intersect( Ray& ray ) const;
	bool IsOccluded( const Ray& ray ) const;
	// BVH data
	BVHNode* bvhNode = 0;			// BVH node in 'structure of arrays' format.
	BVH bvh;						// BVH_SoA is created from BVH and uses its data.
	bool ownBVH = true;				// False when ConvertFrom receives an external bvh.
};

class BVH_Verbose : public BVHBase
{
public:
	struct BVHNode
	{
		// This node layout has some extra data per node: It stores left and right
		// child node indices explicitly, and stores the index of the parent node.
		// This format exists primarily for the BVH optimizer.
		bvhvec3 aabbMin; uint32_t left;
		bvhvec3 aabbMax; uint32_t right;
		uint32_t triCount, firstTri, parent;
		float dummy[5]; // total: 64 bytes.
		bool isLeaf() const { return triCount > 0; }
		float SA() const { return BVH::SA( aabbMin, aabbMax ); }
	};
	BVH_Verbose( BVHContext ctx = {} ) { layout = LAYOUT_BVH_VERBOSE; context = ctx; }
	BVH_Verbose( const BVH& original ) { /* DEPRECATED */ layout = LAYOUT_BVH_VERBOSE; ConvertFrom( original ); }
	~BVH_Verbose() { AlignedFree( bvhNode ); }
	void ConvertFrom( const BVH& original, bool compact = true );
	float SAHCost( const uint32_t nodeIdx = 0 ) const;
	int32_t NodeCount() const;
	int32_t PrimCount( const uint32_t nodeIdx = 0 ) const;
	void Refit( const uint32_t nodeIdx = 0, bool skipLeafs = false );
	void CheckFit( const uint32_t nodeIdx = 0, bool skipLeafs = false );
	void Compact();
	void SplitLeafs( const uint32_t maxPrims = 1 );
	void MergeLeafs();
	void Optimize( const uint32_t iterations = 25, bool extreme = false );
private:
	struct SortItem { uint32_t idx; float cost; };
	void RefitUp( uint32_t nodeIdx );
	float SAHCostUp( uint32_t nodeIdx ) const;
	uint32_t FindBestNewPosition( const uint32_t Lid ) const;
	uint32_t CountSubtreeTris( const uint32_t nodeIdx, uint32_t* counters );
	void MergeSubtree( const uint32_t nodeIdx, uint32_t* newIdx, uint32_t& newIdxPtr );
public:
	// BVH data
	bvhvec4slice verts = {};		// pointer to input primitive array: 3x16 bytes per tri.
	Fragment* fragment = 0;			// input primitive bounding boxes, double-precision.
	uint32_t* primIdx = 0;			// primitive index array - pointer copied from original.
	BVHNode* bvhNode = 0;			// BVH node with additional info, for BVH optimizer.
};

template <int M> class MBVH : public BVHBase
{
public:
	struct MBVHNode
	{
		// 4-wide (aka 'shallow') BVH layout.
		bvhvec3 aabbMin; uint32_t firstTri;
		bvhvec3 aabbMax; uint32_t triCount;
		uint32_t child[M];
		uint32_t childCount;
		uint32_t dummy[((30 - M) & 3) + 1]; // dummies are for alignment.
		bool isLeaf() const { return triCount > 0; }
	};
	MBVH( BVHContext ctx = {} ) { layout = LAYOUT_MBVH; context = ctx; }
	MBVH( const BVH& original ) { /* DEPRECATED */ layout = LAYOUT_MBVH; ConvertFrom( original ); }
	~MBVH();
	void Build( const bvhvec4* vertices, const uint32_t primCount );
	void Build( const bvhvec4slice& vertices );
	void Build( const bvhvec4* vertices, const uint32_t* indices, const uint32_t primCount );
	void Build( const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t primCount );
	void BuildHQ( const bvhvec4* vertices, const uint32_t primCount );
	void BuildHQ( const bvhvec4slice& vertices );
	void BuildHQ( const bvhvec4* vertices, const uint32_t* indices, const uint32_t primCount );
	void BuildHQ( const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t primCount );
	void Optimize( const uint32_t iterations = 25, bool extreme = false );
	void Refit( const uint32_t nodeIdx = 0 );
	float SAHCost( const uint32_t nodeIdx = 0 ) const;
	void ConvertFrom( const BVH& original, bool compact = true );
	void SplitBVHLeaf( const uint32_t nodeIdx, const uint32_t maxPrims );
	// BVH data
	MBVHNode* mbvhNode = 0;			// BVH node for M-wide BVH.
	BVH bvh;						// BVH4 is created from BVH and uses its data.
	bool ownBVH = true;				// False when ConvertFrom receives an external bvh.
};

class BVH4_GPU : public BVHBase
{
public:
	struct BVHNode // actual struct is unused; left here to show structure of data in bvh4Data.
	{
		// 4-way BVH node, optimized for GPU rendering
		struct aabb8 { uint8_t xmin, ymin, zmin, xmax, ymax, zmax; }; // quantized
		bvhvec3 aabbMin; uint32_t c0Info;			// 16
		bvhvec3 aabbExt; uint32_t c1Info;			// 16
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
	BVH4_GPU( const MBVH<4>& bvh4 ) { /* DEPRECATED */ layout = LAYOUT_BVH4_GPU; ConvertFrom( bvh4 ); }
	~BVH4_GPU();
	void Build( const bvhvec4* vertices, const uint32_t primCount );
	void Build( const bvhvec4slice& vertices );
	void Build( const bvhvec4* vertices, const uint32_t* indices, const uint32_t primCount );
	void Build( const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t primCount );
	void BuildHQ( const bvhvec4* vertices, const uint32_t primCount );
	void BuildHQ( const bvhvec4slice& vertices );
	void BuildHQ( const bvhvec4* vertices, const uint32_t* indices, const uint32_t primCount );
	void BuildHQ( const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t primCount );
	void Optimize( const uint32_t iterations = 25, bool extreme = false );
	void ConvertFrom( const MBVH<4>& original, bool compact = true );
	float SAHCost( const uint32_t nodeIdx = 0 ) const { return bvh4.SAHCost( nodeIdx ); }
	int32_t Intersect( Ray& ray ) const;
	bool IsOccluded( const Ray& ray ) const { FALLBACK_SHADOW_QUERY( ray ); }
	// BVH data
	bvhvec4* bvh4Data = 0;			// 64-byte 4-wide BVH node for efficient GPU rendering.
	uint32_t allocatedBlocks = 0;	// node data and triangles are stored in 16-byte blocks.
	uint32_t usedBlocks = 0;		// actually used storage.
	MBVH<4> bvh4;					// BVH4_CPU is created from BVH4 and uses its data.
	bool ownBVH4 = true;			// False when ConvertFrom receives an external bvh.
};

class BVH4_CPU : public BVHBase
{
public:
	struct BVHNode
	{
		// 4-way BVH node, optimized for CPU rendering.
		// Based on: "Faster Incoherent Ray Traversal Using 8-Wide AVX Instructions",
		// Áfra, 2013.
		SIMDVEC4 xmin4, ymin4, zmin4;
		SIMDVEC4 xmax4, ymax4, zmax4;
		uint32_t childFirst[4];
		uint32_t triCount[4];
	};
	BVH4_CPU( BVHContext ctx = {} ) { layout = LAYOUT_BVH4_CPU; context = ctx; }
	BVH4_CPU( const MBVH<4>& bvh4 ) { /* DEPRECATED */ layout = LAYOUT_BVH4_CPU; ConvertFrom( bvh4 ); }
	~BVH4_CPU();
	void Build( const bvhvec4* vertices, const uint32_t primCount );
	void Build( const bvhvec4slice& vertices );
	void Build( const bvhvec4* vertices, const uint32_t* indices, const uint32_t primCount );
	void Build( const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t primCount );
	void BuildHQ( const bvhvec4* vertices, const uint32_t primCount );
	void BuildHQ( const bvhvec4slice& vertices );
	void BuildHQ( const bvhvec4* vertices, const uint32_t* indices, const uint32_t primCount );
	void BuildHQ( const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t primCount );
	void Optimize( const uint32_t iterations = 25, bool extreme = false );
	void Save( const char* fileName );
	bool Load( const char* fileName, const uint32_t primCount );
	float SAHCost( const uint32_t nodeIdx = 0 ) const;
	void ConvertFrom( const MBVH<4>& original, bool compact = true );
	int32_t Intersect( Ray& ray ) const;
	bool IsOccluded( const Ray& ray ) const;
	// BVH data
	BVHNode* bvh4Node = 0;			// 128-byte 4-wide BVH node for efficient CPU rendering.
	bvhvec4* bvh4Tris = 0;			// triangle data for BVHNode4Alt2 nodes.
	MBVH<4> bvh4;					// BVH4_CPU is created from BVH4 and uses its data.
	bool ownBVH4 = true;			// False when ConvertFrom receives an external bvh4.
};

class BVH8_CWBVH : public BVHBase
{
public:
	BVH8_CWBVH( BVHContext ctx = {} ) { layout = LAYOUT_CWBVH; context = ctx; }
	BVH8_CWBVH( MBVH<8>& bvh8 ) { /* DEPRECATED */ layout = LAYOUT_CWBVH; ConvertFrom( bvh8 ); }
	~BVH8_CWBVH();
	void Save( const char* fileName );
	bool Load( const char* fileName, const uint32_t expectedTris );
	void Build( const bvhvec4* vertices, const uint32_t primCount );
	void Build( const bvhvec4slice& vertices );
	void Build( const bvhvec4* vertices, const uint32_t* indices, const uint32_t primCount );
	void Build( const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t primCount );
	void BuildHQ( const bvhvec4* vertices, const uint32_t primCount );
	void BuildHQ( const bvhvec4slice& vertices );
	void BuildHQ( const bvhvec4* vertices, const uint32_t* indices, const uint32_t primCount );
	void BuildHQ( const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t primCount );
	void Optimize( const uint32_t iterations = 25, bool extreme = false );
	void ConvertFrom( MBVH<8>& original, bool compact = true );
	float SAHCost( const uint32_t nodeIdx = 0 ) const;
	int32_t Intersect( Ray& ray ) const;
	bool IsOccluded( const Ray& ray ) const { FALLBACK_SHADOW_QUERY( ray ); }
	// BVH8 data
	bvhvec4* bvh8Data = 0;			// nodes in CWBVH format.
	bvhvec4* bvh8Tris = 0;			// triangle data for CWBVH nodes.
	uint32_t allocatedBlocks = 0;	// node data is stored in blocks of 16 byte.
	uint32_t usedBlocks = 0;		// actually used blocks.
	MBVH<8> bvh8;					// BVH8_CWBVH is created from BVH8 and uses its data.
	bool ownBVH8 = true;			// false when ConvertFrom receives an external bvh8.
};

// BLASInstance: A TLAS is built over BLAS instances, where a single BLAS can be
// used with multiple transforms, and multiple BLASses can be combined in a complex
// scene. The TLAS is built over the world-space AABBs of the BLAS root nodes.
class ALIGNED( 64 ) BLASInstance
{
public:
	BLASInstance() = default;
	BLASInstance( uint32_t idx ) : blasIdx( idx ) {}
	float transform[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 }; // identity
	float invTransform[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 }; // identity
	bvhvec3 aabbMin = bvhvec3( BVH_FAR );
	uint32_t blasIdx = 0;
	bvhvec3 aabbMax = bvhvec3( -BVH_FAR );
	uint32_t dummy[9]; // pad struct to 64 byte
	void Update( BVHBase * blas );
	void InvertTransform();
};

#ifdef DOUBLE_PRECISION_SUPPORT

// BLASInstanceEx: Double-precision version of BLASInstance.
class BLASInstanceEx
{
public:
	BLASInstanceEx() = default;
	BLASInstanceEx( uint64_t idx ) : blasIdx( idx ) {}
	double transform[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 }; // identity
	double invTransform[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 }; // identity
	bvhdbl3 aabbMin = bvhdbl3( BVH_DBL_FAR );
	uint64_t blasIdx = 0;
	bvhdbl3 aabbMax = bvhdbl3( -BVH_DBL_FAR );
	uint64_t dummy = 0;
	void Update( BVH_Double* blas );
	void InvertTransform();
};

#endif

// Experimental & 'under construction' structs

class BVH4_WiVe : public BVHBase
{
public:
	struct BVHNode
	{
		// 4-way BVH node, optimized for CPU rendering.
		// Based on: "Accelerated Single Ray Tracing for Wide Vector Units",
		// Fuetterling1 et al., 2017.
		union { SIMDVEC8 xminmax8; float xminmax[8]; }; // x0max,x1max,..,x3max; x0min,x1min,..,x3min
		union { SIMDVEC8 yminmax8; float yminmax[8]; }; // y0max,y1max,..,y3max; y0min,x1min,..,y3min
		union { SIMDVEC8 zminmax8; float zminmax[8]; }; // z0max,z1max,..,z3max; z0min,x1min,..,z3min
		union { SIMDVEC8 sn8; float sn[8]; }; // n0..3, s0..3; total size =  128 bytes.
		// n_i: inner node flag + child node cluster offset
	};
	BVH4_WiVe( BVHContext ctx = {} ) { layout = LAYOUT_BVH4_AVX2; context = ctx; }
	~BVH4_WiVe();
	void Build( const bvhvec4* vertices, const uint32_t primCount );
	void Build( const bvhvec4slice& vertices );
	void ConvertFrom( MBVH<4>& original, bool compact = true );
	int32_t Intersect( Ray& ray ) const;
	bool IsOccluded( const Ray& ray ) const;
	// BVH4 data
	bvhvec4slice verts = {};		// pointer to input primitive array: 3x16 bytes per tri.
	uint32_t* primIdx = 0;			// primitive index array - pointer copied from original.
	BVHNode* bvh4Node = 0;			// 128-byte 4-wide BVH node for efficient CPU rendering.
};

} // namespace tinybvh

#endif // TINY_BVH_H_

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

// We need quite a bit of type reinterpretation, so we'll
// turn off the gcc warning here until the end of the file.
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
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
#if defined BVH_USEAVX || defined BVH_USENEON

static uint32_t __bfind( uint32_t x ) // https://github.com/mackron/refcode/blob/master/lzcnt.c
{
#if defined(_MSC_VER) && !defined(__clang__)
	return 31 - __lzcnt( x );
#elif defined(__EMSCRIPTEN__)
	return 31 - __builtin_clz( x );
#elif defined(__GNUC__) || defined(__clang__)
#ifndef __APPLE__
	uint32_t r;
	__asm__ __volatile__( "lzcnt{l %1, %0| %0, %1}" : "=r"(r) : "r"(x) : "cc" );
	return 31 - r;
#else
	return 31 - __builtin_clz( x ); // TODO: unverified.
#endif
#endif
}

#endif

#ifndef TINYBVH_USE_CUSTOM_VECTOR_TYPES

bvhvec4::bvhvec4( const bvhvec3& a ) { x = a.x; y = a.y; z = a.z; w = 0; }
bvhvec4::bvhvec4( const bvhvec3& a, float b ) { x = a.x; y = a.y; z = a.z; w = b; }

#endif

bvhvec4slice::bvhvec4slice( const bvhvec4* data, uint32_t count, uint32_t stride ) :
	data{ reinterpret_cast<const int8_t*>(data) },
	count{ count }, stride{ stride } {}

const bvhvec4& bvhvec4slice::operator[]( size_t i ) const
{
#ifdef PARANOID
	FATAL_ERROR_IF( i >= count, "bvhvec4slice::[..], Reading outside slice." );
#endif
	return *reinterpret_cast<const bvhvec4*>(data + stride * i);
}

void* BVHBase::AlignedAlloc( size_t size )
{
	return context.malloc ? context.malloc( size, context.userdata ) : nullptr;
}

void BVHBase::AlignedFree( void* ptr )
{
	if (context.free)
		context.free( ptr, context.userdata );
}

void BVHBase::CopyBasePropertiesFrom( const BVHBase& original )
{
	this->rebuildable = original.rebuildable;
	this->refittable = original.refittable;
	this->may_have_holes = original.may_have_holes;
	this->bvh_over_aabbs = original.bvh_over_aabbs;
	this->bvh_over_indices = original.bvh_over_indices;
	this->context = original.context;
	this->triCount = original.triCount;
	this->idxCount = original.idxCount;
	this->aabbMin = original.aabbMin, this->aabbMax = original.aabbMax;
}

// BVH implementation
// ----------------------------------------------------------------------------

BVH::~BVH()
{
	AlignedFree( bvhNode );
	AlignedFree( primIdx );
	AlignedFree( fragment );
}

void BVH::Save( const char* fileName )
{
	// saving is easy, it's the loadingn that will be complex.
	std::fstream s{ fileName, s.binary | s.out };
	uint32_t header = TINY_BVH_VERSION_SUB + (TINY_BVH_VERSION_MINOR << 8) + (TINY_BVH_VERSION_MAJOR << 16) + (layout << 24);
	s.write( (char*)&header, sizeof( uint32_t ) );
	s.write( (char*)&triCount, sizeof( uint32_t ) );
	s.write( (char*)this, sizeof( BVH ) );
	s.write( (char*)bvhNode, usedNodes * sizeof( BVHNode ) );
	s.write( (char*)primIdx, idxCount * sizeof( uint32_t ) );
}

bool BVH::Load( const char* fileName, const bvhvec4* vertices, const uint32_t primCount )
{
	return Load( fileName, bvhvec4slice{ vertices, primCount * 3, sizeof( bvhvec4 ) } );
}

bool BVH::Load( const char* fileName, const bvhvec4* vertices, const uint32_t* indices, const uint32_t primCount )
{
	return Load( fileName, bvhvec4slice{ vertices, primCount * 3, sizeof( bvhvec4 ) }, indices, primCount );
}

bool BVH::Load( const char* fileName, const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t primCount )
{
	// open file and check contents
	std::fstream s{ fileName, s.binary | s.in };
	if (!s) return false;
	BVHContext tmp = context;
	bool expectIndexed = (indices != nullptr);
	uint32_t header, fileTriCount;
	s.read( (char*)&header, sizeof( uint32_t ) );
	if (((header >> 8) & 255) != TINY_BVH_VERSION_MINOR ||
		((header >> 16) & 255) != TINY_BVH_VERSION_MAJOR ||
		(header & 255) != TINY_BVH_VERSION_SUB || (header >> 24) != layout) return false;
	s.read( (char*)&fileTriCount, sizeof( uint32_t ) );
	if (expectIndexed && fileTriCount != primCount) return false;
	if (!expectIndexed && fileTriCount != vertices.count / 3) return false;
	// all checks passed; safe to overwrite *this
	s.read( (char*)this, sizeof( BVH ) );
	bool fileIsIndexed = vertIdx != nullptr;
	if (expectIndexed != fileIsIndexed) return false; // not what we expected.
	if (blasList != nullptr || instList != nullptr) return false; // can't load/save TLAS.
	context = tmp; // can't load context; function pointers will differ.
	bvhNode = (BVHNode*)AlignedAlloc( allocatedNodes * sizeof( BVHNode ) );
	primIdx = (uint32_t*)AlignedAlloc( idxCount * sizeof( uint32_t ) );
	fragment = 0; // no need for this in a BVH that can't be rebuilt.
	s.read( (char*)bvhNode, usedNodes * sizeof( BVHNode ) );
	s.read( (char*)primIdx, idxCount * sizeof( uint32_t ) );
	verts = vertices; // we can't load vertices since the BVH doesn't own this data.
	vertIdx = (uint32_t*)indices;
	// all ok.
	return true;
}

void BVH::BuildDefault( const bvhvec4* vertices, const uint32_t primCount )
{
	// access point for builds over a raw list of vertices. The stride of
	// the vertex data must be 16 bytes.
	// The list will be encapsulated in a 'slice'. The slice can also be used
	// directly to provide data with a different stride, which enables use of
	// vertex buffers created for rasterization.
	BuildDefault( bvhvec4slice{ vertices, primCount * 3, sizeof( bvhvec4 ) } );
}
void BVH::BuildDefault( const bvhvec4* vertices, const uint32_t* indices, const uint32_t primCount )
{
	// access point for builders over an indexed list of raw vertices.
	BuildDefault( bvhvec4slice{ vertices, primCount * 3, sizeof( bvhvec4 ) }, indices, primCount );
}
void BVH::BuildDefault( const bvhvec4slice& vertices )
{
	// default builder: used internally when constructing a BVH layout requires
	// a regular BVH. Currently, this is the case for all of them.
#if defined(BVH_USEAVX)
	// if AVX is supported, BuildAVX is the optimal option. Tree quality is
	// identical to the reference builder, but speed is much better.
	BuildAVX( vertices );
#else
	// fallback option, in case AVX is not supported: the reference builder, which
	// should work on all platforms.
	Build( vertices );
#endif
}
void BVH::BuildDefault( const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t primCount )
{
	// default builder for indexed vertices. See notes above.
#if defined(BVH_USEAVX)
	BuildAVX( vertices, indices, primCount );
#else
	Build( vertices, indices, primCount );
#endif
}

void BVH::ConvertFrom( const BVH_Verbose& original, bool compact )
{
	// allocate space
	const uint32_t spaceNeeded = compact ? original.usedNodes : original.allocatedNodes;
	if (allocatedNodes < spaceNeeded)
	{
		AlignedFree( bvhNode );
		bvhNode = (BVHNode*)AlignedAlloc( triCount * 2 * sizeof( BVHNode ) );
		allocatedNodes = spaceNeeded;
	}
	memset( bvhNode, 0, sizeof( BVHNode ) * spaceNeeded );
	CopyBasePropertiesFrom( original );
	this->verts = original.verts;
	this->primIdx = original.primIdx;
	// start conversion
	uint32_t srcNodeIdx = 0, dstNodeIdx = 0;
	newNodePtr = 2;
	uint32_t srcStack[1024], dstStack[1024], stackPtr = 0;
	while (1)
	{
		const BVH_Verbose::BVHNode& orig = original.bvhNode[srcNodeIdx];
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
			uint32_t srcRightIdx = orig.right;
			srcNodeIdx = orig.left, dstNodeIdx = newNodePtr++;
			srcStack[stackPtr] = srcRightIdx;
			dstStack[stackPtr++] = newNodePtr++;
		}
	}
	usedNodes = original.usedNodes;
}

float BVH::SAHCost( const uint32_t nodeIdx ) const
{
	// Determine the SAH cost of the tree. This provides an indication
	// of the quality of the BVH: Lower is better.
	const BVHNode& n = bvhNode[nodeIdx];
	if (n.isLeaf()) return C_INT * n.SurfaceArea() * n.triCount;
	float cost = C_TRAV * n.SurfaceArea() + SAHCost( n.leftFirst ) + SAHCost( n.leftFirst + 1 );
	return nodeIdx == 0 ? (cost / n.SurfaceArea()) : cost;
}

int32_t BVH::PrimCount( const uint32_t nodeIdx ) const
{
	// Determine the total number of primitives / fragments in leaf nodes.
	const BVHNode& n = bvhNode[nodeIdx];
	return n.isLeaf() ? n.triCount : (PrimCount( n.leftFirst ) + PrimCount( n.leftFirst + 1 ));
}

// Basic single-function BVH builder, using mid-point splits.
// This builder yields a correct BVH in little time, but the quality of the
// structure will be low. Use this only if build time is the bottleneck in
// your application (e.g., when you need to trace few rays).
void BVH::BuildQuick( const bvhvec4* vertices, const uint32_t primCount )
{
	// build the BVH with a continuous array of bvhvec4 vertices:
	// in this case, the stride for the slice is 16 bytes.
	BuildQuick( bvhvec4slice{ vertices, primCount * 3, sizeof( bvhvec4 ) } );
}
void BVH::BuildQuick( const bvhvec4slice& vertices )
{
	FATAL_ERROR_IF( vertices.count == 0, "BVH::BuildQuick( .. ), primCount == 0." );
	// allocate on first build
	const uint32_t primCount = vertices.count / 3;
	const uint32_t spaceNeeded = primCount * 2; // upper limit
	if (allocatedNodes < spaceNeeded)
	{
		AlignedFree( bvhNode );
		AlignedFree( primIdx );
		AlignedFree( fragment );
		bvhNode = (BVHNode*)AlignedAlloc( spaceNeeded * sizeof( BVHNode ) );
		allocatedNodes = spaceNeeded;
		memset( &bvhNode[1], 0, 32 );	// node 1 remains unused, for cache line alignment.
		primIdx = (uint32_t*)AlignedAlloc( primCount * sizeof( uint32_t ) );
		fragment = (Fragment*)AlignedAlloc( primCount * sizeof( Fragment ) );
	}
	else FATAL_ERROR_IF( !rebuildable, "BVH::BuildQuick( .. ), bvh not rebuildable." );
	verts = vertices; // note: we're not copying this data; don't delete.
	idxCount = triCount = primCount;
	// reset node pool
	newNodePtr = 2;
	// assign all triangles to the root node
	BVHNode& root = bvhNode[0];
	root.leftFirst = 0, root.triCount = triCount, root.aabbMin = bvhvec3( BVH_FAR ), root.aabbMax = bvhvec3( -BVH_FAR );
	// initialize fragments and initialize root node bounds
	for (uint32_t i = 0; i < triCount; i++)
	{
		fragment[i].bmin = tinybvh_min( tinybvh_min( verts[i * 3], verts[i * 3 + 1] ), verts[i * 3 + 2] );
		fragment[i].bmax = tinybvh_max( tinybvh_max( verts[i * 3], verts[i * 3 + 1] ), verts[i * 3 + 2] );
		root.aabbMin = tinybvh_min( root.aabbMin, fragment[i].bmin );
		root.aabbMax = tinybvh_max( root.aabbMax, fragment[i].bmax ), primIdx[i] = i;
	}
	// subdivide recursively
	uint32_t task[256], taskCount = 0, nodeIdx = 0;
	while (1)
	{
		while (1)
		{
			BVHNode& node = bvhNode[nodeIdx];
			// in-place partition against midpoint on longest axis
			uint32_t j = node.leftFirst + node.triCount, src = node.leftFirst;
			bvhvec3 extent = node.aabbMax - node.aabbMin;
			uint32_t axis = 0;
			if (extent.y > extent.x && extent.y > extent.z) axis = 1;
			if (extent.z > extent.x && extent.z > extent.y) axis = 2;
			float splitPos = node.aabbMin[axis] + extent[axis] * 0.5f, centroid;
			bvhvec3 lbmin( BVH_FAR ), lbmax( -BVH_FAR ), rbmin( BVH_FAR ), rbmax( -BVH_FAR ), fmin, fmax;
			for (uint32_t fi, i = 0; i < node.triCount; i++)
			{
				fi = primIdx[src], fmin = fragment[fi].bmin, fmax = fragment[fi].bmax;
				centroid = (fmin[axis] + fmax[axis]) * 0.5f;
				if (centroid < splitPos)
					lbmin = tinybvh_min( lbmin, fmin ), lbmax = tinybvh_max( lbmax, fmax ), src++;
				else
				{
					rbmin = tinybvh_min( rbmin, fmin ), rbmax = tinybvh_max( rbmax, fmax );
					tinybvh_swap( primIdx[src], primIdx[--j] );
				}
			}
			// create child nodes
			const uint32_t leftCount = src - node.leftFirst, rightCount = node.triCount - leftCount;
			if (leftCount == 0 || rightCount == 0) break; // split did not work out.
			const int32_t lci = newNodePtr++, rci = newNodePtr++;
			bvhNode[lci].aabbMin = lbmin, bvhNode[lci].aabbMax = lbmax;
			bvhNode[lci].leftFirst = node.leftFirst, bvhNode[lci].triCount = leftCount;
			bvhNode[rci].aabbMin = rbmin, bvhNode[rci].aabbMax = rbmax;
			bvhNode[rci].leftFirst = j, bvhNode[rci].triCount = rightCount;
			node.leftFirst = lci, node.triCount = 0;
			// recurse
			task[taskCount++] = rci, nodeIdx = lci;
		}
		// fetch subdivision task from stack
		if (taskCount == 0) break; else nodeIdx = task[--taskCount];
	}
	// all done.
	aabbMin = bvhNode[0].aabbMin, aabbMax = bvhNode[0].aabbMax;
	refittable = true; // not using spatial splits: can refit this BVH
	may_have_holes = false; // the reference builder produces a continuous list of nodes
	usedNodes = newNodePtr;
}

// Basic single-function binned-SAH-builder.
// This is the reference builder; it yields a decent tree suitable for ray tracing on the CPU.
// This code uses no SIMD instructions. Faster code, using SSE/AVX, is available for x64 CPUs.
// For GPU rendering: The resulting BVH should be converted to a more optimal
// format after construction, e.g. BVH_GPU, BVH4_GPU or BVH8_CWBVH.
void BVH::Build( const bvhvec4* vertices, const uint32_t prims )
{
	// build the BVH with a continuous array of bvhvec4 vertices:
	// in this case, the stride for the slice is 16 bytes.
	Build( bvhvec4slice{ vertices, prims * 3, sizeof( bvhvec4 ) } );
}
void BVH::Build( const bvhvec4slice& vertices )
{
	// build the BVH from vertices stored in a slice.
	PrepareBuild( vertices, 0, 0 /* empty index list; primcount is derived from slice */ );
	Build();
}
void BVH::Build( const bvhvec4* vertices, const uint32_t* indices, const uint32_t prims )
{
	// build the BVH with a continuous array of bvhvec4 vertices, indexed by 'indices'.
	Build( bvhvec4slice{ vertices, prims * 3, sizeof( bvhvec4 ) }, indices, prims );
}
void BVH::Build( const bvhvec4slice& vertices, const uint32_t* indices, uint32_t prims )
{
	// build the BVH from vertices stored in a slice, indexed by 'indices'.
	PrepareBuild( vertices, indices, prims );
	Build();
}

void BVH::Build( void (*customGetAABB)(const unsigned, bvhvec3&, bvhvec3&), const uint32_t primCount )
{
	FATAL_ERROR_IF( primCount == 0, "BVH::Build( void (*customGetAABB)( .. ), instCount ), instCount == 0." );
	triCount = idxCount = primCount;
	const uint32_t spaceNeeded = primCount * 2; // upper limit
	if (allocatedNodes < spaceNeeded)
	{
		AlignedFree( bvhNode );
		AlignedFree( primIdx );
		AlignedFree( fragment );
		bvhNode = (BVHNode*)AlignedAlloc( spaceNeeded * sizeof( BVHNode ) );
		allocatedNodes = spaceNeeded;
		memset( &bvhNode[1], 0, 32 );	// node 1 remains unused, for cache line alignment.
		primIdx = (uint32_t*)AlignedAlloc( primCount * sizeof( uint32_t ) );
		fragment = (Fragment*)AlignedAlloc( primCount * sizeof( Fragment ) );
	}
	// copy relevant data from instance array
	BVHNode& root = bvhNode[0];
	root.leftFirst = 0, root.triCount = primCount, root.aabbMin = bvhvec3( BVH_FAR ), root.aabbMax = bvhvec3( -BVH_FAR );
	for (uint32_t i = 0; i < primCount; i++)
	{
		customGetAABB( i, fragment[i].bmin, fragment[i].bmax );
		fragment[i].primIdx = i, fragment[i].clipped = 0, primIdx[i] = i;
		root.aabbMin = tinybvh_min( root.aabbMin, fragment[i].bmin );
		root.aabbMax = tinybvh_max( root.aabbMax, fragment[i].bmax );
	}
	// start build
	newNodePtr = 2;
	Build(); // or BuildAVX, for large TLAS.
}

void BVH::Build( BLASInstance* instances, const uint32_t instCount, BVHBase** blasses, const uint32_t bCount )
{
	FATAL_ERROR_IF( instCount == 0, "BVH::Build( BLASInstance*, instCount ), instCount == 0." );
	triCount = idxCount = instCount;
	const uint32_t spaceNeeded = instCount * 2; // upper limit
	if (allocatedNodes < spaceNeeded)
	{
		AlignedFree( bvhNode );
		AlignedFree( primIdx );
		AlignedFree( fragment );
		bvhNode = (BVHNode*)AlignedAlloc( spaceNeeded * sizeof( BVHNode ) );
		allocatedNodes = spaceNeeded;
		memset( &bvhNode[1], 0, 32 );	// node 1 remains unused, for cache line alignment.
		primIdx = (uint32_t*)AlignedAlloc( instCount * sizeof( uint32_t ) );
		fragment = (Fragment*)AlignedAlloc( instCount * sizeof( Fragment ) );
	}
	instList = instances;
	blasList = blasses;
	blasCount = bCount;
	// copy relevant data from instance array
	BVHNode& root = bvhNode[0];
	root.leftFirst = 0, root.triCount = instCount, root.aabbMin = bvhvec3( BVH_FAR ), root.aabbMax = bvhvec3( -BVH_FAR );
	for (uint32_t i = 0; i < instCount; i++)
	{
		if (blasList) // if a null pointer is passed, we'll assume the BLASInstances have been updated elsewhere.
		{
			uint32_t blasIdx = instList[i].blasIdx;
			BVH* blas = (BVH*)blasList[blasIdx];
			instList[i].Update( blas );
		}
		fragment[i].bmin = instList[i].aabbMin, fragment[i].primIdx = i;
		fragment[i].bmax = instList[i].aabbMax, fragment[i].clipped = 0;
		root.aabbMin = tinybvh_min( root.aabbMin, instList[i].aabbMin );
		root.aabbMax = tinybvh_max( root.aabbMax, instList[i].aabbMax ), primIdx[i] = i;
	}
	// start build
	newNodePtr = 2;
	Build(); // or BuildAVX, for large TLAS.
}

void BVH::PrepareBuild( const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t prims )
{
#ifdef SLICEDUMP
	// this code dumps the passed geometry data to a file - for debugging only.
	std::fstream df{ "dump.bin", df.binary | df.out };
	uint32_t vcount = vertices.count, indexed = indices == 0 ? 0 : 1, stride = vertices.stride;
	uint32_t pcount = indices ? prims : (vertices.count / 3);
	df.write( (char*)&pcount, 4 );
	df.write( (char*)&vcount, 4 );
	df.write( (char*)&stride, 4 );
	df.write( (char*)&indexed, sizeof( uint32_t ) );
	df.write( (char*)vertices.data, vertices.stride * vertices.count );
	if (indexed) df.write( (char*)indices, prims * 3 * 4 );
#endif
	uint32_t primCount = prims > 0 ? prims : vertices.count / 3;
	const uint32_t spaceNeeded = primCount * 2; // upper limit
	// allocate memory on first build
	if (allocatedNodes < spaceNeeded)
	{
		AlignedFree( bvhNode );
		AlignedFree( primIdx );
		AlignedFree( fragment );
		bvhNode = (BVHNode*)AlignedAlloc( spaceNeeded * sizeof( BVHNode ) );
		allocatedNodes = spaceNeeded;
		memset( &bvhNode[1], 0, 32 );	// node 1 remains unused, for cache line alignment.
		primIdx = (uint32_t*)AlignedAlloc( primCount * sizeof( uint32_t ) );
		if (vertices) fragment = (Fragment*)AlignedAlloc( primCount * sizeof( Fragment ) );
		else FATAL_ERROR_IF( fragment == 0, "BVH::PrepareBuild( 0, .. ), not called from ::Build( aabb )." );
	}
	else FATAL_ERROR_IF( !rebuildable, "BVH::PrepareBuild( .. ), bvh not rebuildable." );
	verts = vertices, idxCount = triCount = primCount, vertIdx = (uint32_t*)indices;
	// prepare fragments
	FATAL_ERROR_IF( vertices.count == 0, "BVH::PrepareBuild( .. ), empty vertex slice." );
	BVHNode& root = bvhNode[0];
	root.leftFirst = 0, root.triCount = triCount, root.aabbMin = bvhvec3( BVH_FAR ), root.aabbMax = bvhvec3( -BVH_FAR );
	if (!indices)
	{
		FATAL_ERROR_IF( prims != 0, "BVH::PrepareBuild( .. ), indices == 0." );
		// building a BVH over triangles specified as three 16-byte vertices each.
		for (uint32_t i = 0; i < triCount; i++)
		{
			const bvhvec4 v0 = verts[i * 3], v1 = verts[i * 3 + 1], v2 = verts[i * 3 + 2];
			const bvhvec4 fmin = tinybvh_min( v0, tinybvh_min( v1, v2 ) );
			const bvhvec4 fmax = tinybvh_max( v0, tinybvh_max( v1, v2 ) );
			fragment[i].bmin = fmin, fragment[i].bmax = fmax;
			root.aabbMin = tinybvh_min( root.aabbMin, fragment[i].bmin );
			root.aabbMax = tinybvh_max( root.aabbMax, fragment[i].bmax ), primIdx[i] = i;
		}
	}
	else
	{
		FATAL_ERROR_IF( prims == 0, "BVH::PrepareBuild( .. ), prims == 0." );
		// building a BVH over triangles consisting of vertices indexed by 'indices'.
		for (uint32_t i = 0; i < triCount; i++)
		{
			const uint32_t i0 = indices[i * 3], i1 = indices[i * 3 + 1], i2 = indices[i * 3 + 2];
			const bvhvec4 v0 = verts[i0], v1 = verts[i1], v2 = verts[i2];
			const bvhvec4 fmin = tinybvh_min( v0, tinybvh_min( v1, v2 ) );
			const bvhvec4 fmax = tinybvh_max( v0, tinybvh_max( v1, v2 ) );
			fragment[i].bmin = fmin, fragment[i].bmax = fmax;
			root.aabbMin = tinybvh_min( root.aabbMin, fragment[i].bmin );
			root.aabbMax = tinybvh_max( root.aabbMax, fragment[i].bmax ), primIdx[i] = i;
		}
	}
	// reset node pool
	newNodePtr = 2;
	bvh_over_indices = indices != nullptr;
	// all set; actual build happens in BVH::Build.
}
void BVH::Build()
{
	// subdivide root node recursively
	uint32_t task[256], taskCount = 0, nodeIdx = 0;
	BVHNode& root = bvhNode[0];
	bvhvec3 minDim = (root.aabbMax - root.aabbMin) * 1e-20f, bestLMin = bvhvec3( 0 ), bestLMax = bvhvec3( 0 ), bestRMin = bvhvec3( 0 ), bestRMax = bvhvec3( 0 );
	while (1)
	{
		while (1)
		{
			BVHNode& node = bvhNode[nodeIdx];
			// find optimal object split
			bvhvec3 binMin[3][BVHBINS], binMax[3][BVHBINS];
			for (uint32_t a = 0; a < 3; a++) for (uint32_t i = 0; i < BVHBINS; i++) binMin[a][i] = bvhvec3( BVH_FAR ), binMax[a][i] = bvhvec3( -BVH_FAR );
			uint32_t count[3][BVHBINS];
			memset( count, 0, BVHBINS * 3 * sizeof( uint32_t ) );
			const bvhvec3 rpd3 = bvhvec3( BVHBINS / (node.aabbMax - node.aabbMin) ), nmin3 = node.aabbMin;
			for (uint32_t i = 0; i < node.triCount; i++) // process all tris for x,y and z at once
			{
				const uint32_t fi = primIdx[node.leftFirst + i];
				bvhint3 bi = bvhint3( ((fragment[fi].bmin + fragment[fi].bmax) * 0.5f - nmin3) * rpd3 );
				bi.x = tinybvh_clamp( bi.x, 0, BVHBINS - 1 );
				bi.y = tinybvh_clamp( bi.y, 0, BVHBINS - 1 );
				bi.z = tinybvh_clamp( bi.z, 0, BVHBINS - 1 );
				binMin[0][bi.x] = tinybvh_min( binMin[0][bi.x], fragment[fi].bmin );
				binMax[0][bi.x] = tinybvh_max( binMax[0][bi.x], fragment[fi].bmax ), count[0][bi.x]++;
				binMin[1][bi.y] = tinybvh_min( binMin[1][bi.y], fragment[fi].bmin );
				binMax[1][bi.y] = tinybvh_max( binMax[1][bi.y], fragment[fi].bmax ), count[1][bi.y]++;
				binMin[2][bi.z] = tinybvh_min( binMin[2][bi.z], fragment[fi].bmin );
				binMax[2][bi.z] = tinybvh_max( binMax[2][bi.z], fragment[fi].bmax ), count[2][bi.z]++;
			}
			// calculate per-split totals
			float splitCost = BVH_FAR, rSAV = 1.0f / node.SurfaceArea();
			uint32_t bestAxis = 0, bestPos = 0;
			for (int32_t a = 0; a < 3; a++) if ((node.aabbMax[a] - node.aabbMin[a]) > minDim[a])
			{
				bvhvec3 lBMin[BVHBINS - 1], rBMin[BVHBINS - 1], l1 = bvhvec3( BVH_FAR ), l2 = bvhvec3( -BVH_FAR );
				bvhvec3 lBMax[BVHBINS - 1], rBMax[BVHBINS - 1], r1 = bvhvec3( BVH_FAR ), r2 = bvhvec3( -BVH_FAR );
				float ANL[BVHBINS - 1], ANR[BVHBINS - 1];
				for (uint32_t lN = 0, rN = 0, i = 0; i < BVHBINS - 1; i++)
				{
					lBMin[i] = l1 = tinybvh_min( l1, binMin[a][i] );
					rBMin[BVHBINS - 2 - i] = r1 = tinybvh_min( r1, binMin[a][BVHBINS - 1 - i] );
					lBMax[i] = l2 = tinybvh_max( l2, binMax[a][i] );
					rBMax[BVHBINS - 2 - i] = r2 = tinybvh_max( r2, binMax[a][BVHBINS - 1 - i] );
					lN += count[a][i], rN += count[a][BVHBINS - 1 - i];
					ANL[i] = lN == 0 ? BVH_FAR : ((l2 - l1).halfArea() * (float)lN);
					ANR[BVHBINS - 2 - i] = rN == 0 ? BVH_FAR : ((r2 - r1).halfArea() * (float)rN);
				}
				// evaluate bin totals to find best position for object split
				for (uint32_t i = 0; i < BVHBINS - 1; i++)
				{
					const float C = C_TRAV + rSAV * C_INT * (ANL[i] + ANR[i]);
					if (C < splitCost)
					{
						splitCost = C, bestAxis = a, bestPos = i;
						bestLMin = lBMin[i], bestRMin = rBMin[i], bestLMax = lBMax[i], bestRMax = rBMax[i];
					}
				}
			}
			float noSplitCost = (float)node.triCount * C_INT;
			if (splitCost >= noSplitCost) break; // not splitting is better.
			// in-place partition
			uint32_t j = node.leftFirst + node.triCount, src = node.leftFirst;
			const float rpd = rpd3.cell[bestAxis], nmin = nmin3.cell[bestAxis];
			for (uint32_t i = 0; i < node.triCount; i++)
			{
				const uint32_t fi = primIdx[src];
				int32_t bi = (uint32_t)(((fragment[fi].bmin[bestAxis] + fragment[fi].bmax[bestAxis]) * 0.5f - nmin) * rpd);
				bi = tinybvh_clamp( bi, 0, BVHBINS - 1 );
				if ((uint32_t)bi <= bestPos) src++; else tinybvh_swap( primIdx[src], primIdx[--j] );
			}
			// create child nodes
			uint32_t leftCount = src - node.leftFirst, rightCount = node.triCount - leftCount;
			if (leftCount == 0 || rightCount == 0) break; // should not happen.
			const int32_t lci = newNodePtr++, rci = newNodePtr++;
			bvhNode[lci].aabbMin = bestLMin, bvhNode[lci].aabbMax = bestLMax;
			bvhNode[lci].leftFirst = node.leftFirst, bvhNode[lci].triCount = leftCount;
			bvhNode[rci].aabbMin = bestRMin, bvhNode[rci].aabbMax = bestRMax;
			bvhNode[rci].leftFirst = j, bvhNode[rci].triCount = rightCount;
			node.leftFirst = lci, node.triCount = 0;
			// recurse
			task[taskCount++] = rci, nodeIdx = lci;
		}
		// fetch subdivision task from stack
		if (taskCount == 0) break; else nodeIdx = task[--taskCount];
	}
	// all done.
	aabbMin = bvhNode[0].aabbMin, aabbMax = bvhNode[0].aabbMax;
	refittable = true; // not using spatial splits: can refit this BVH
	may_have_holes = false; // the reference builder produces a continuous list of nodes
	bvh_over_aabbs = (verts == 0); // bvh over aabbs is suitable as TLAS
	usedNodes = newNodePtr;
}

// SBVH builder.
// Besides the regular object splits used in the reference builder, the SBVH
// algorithm also considers spatial splits, where primitives may be cut in
// multiple parts. This increases primitive count but may reduce overlap of
// BVH nodes. The cost of each option is considered per split.
// For typical geometry, SBVH yields a tree that can be traversed 25% faster.
// This comes at greatly increased construction cost, making the SBVH
// primarily useful for static geometry.
void BVH::BuildHQ( const bvhvec4* vertices, const uint32_t primCount )
{
	BuildHQ( bvhvec4slice{ vertices, primCount * 3, sizeof( bvhvec4 ) } );
}

void BVH::BuildHQ( const bvhvec4* vertices, const uint32_t* indices, const uint32_t prims )
{
	// build the BVH with a continuous array of bvhvec4 vertices, indexed by 'indices'.
	BuildHQ( bvhvec4slice{ vertices, prims * 3, sizeof( bvhvec4 ) }, indices, prims );
}

void BVH::BuildHQ( const bvhvec4slice& vertices )
{
	PrepareHQBuild( vertices, 0, 0 );
	BuildHQ();
}

void BVH::BuildHQ( const bvhvec4slice& vertices, const uint32_t* indices, uint32_t prims )
{
	// build the BVH from vertices stored in a slice, indexed by 'indices'.
	PrepareHQBuild( vertices, indices, prims );
	BuildHQ();
}

void BVH::PrepareHQBuild( const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t prims )
{
	uint32_t primCount = prims > 0 ? prims : vertices.count / 3;
	const uint32_t slack = primCount >> 1; // for split prims
	const uint32_t spaceNeeded = primCount * 3;
	// allocate memory on first build
	if (allocatedNodes < spaceNeeded)
	{
		AlignedFree( bvhNode );
		AlignedFree( primIdx );
		AlignedFree( fragment );
		bvhNode = (BVHNode*)AlignedAlloc( spaceNeeded * sizeof( BVHNode ) );
		allocatedNodes = spaceNeeded;
		memset( &bvhNode[1], 0, 32 );	// node 1 remains unused, for cache line alignment.
		primIdx = (uint32_t*)AlignedAlloc( (primCount + slack) * sizeof( uint32_t ) );
		fragment = (Fragment*)AlignedAlloc( (primCount + slack) * sizeof( Fragment ) );
	}
	else FATAL_ERROR_IF( !rebuildable, "BVH::PrepareHQBuild( .. ), bvh not rebuildable." );
	verts = vertices; // note: we're not copying this data; don't delete.
	idxCount = primCount + slack, triCount = primCount, vertIdx = (uint32_t*)indices;
	// prepare fragments
	BVHNode& root = bvhNode[0];
	root.leftFirst = 0, root.triCount = triCount, root.aabbMin = bvhvec3( BVH_FAR ), root.aabbMax = bvhvec3( -BVH_FAR );
	if (!indices)
	{
		FATAL_ERROR_IF( vertices.count == 0, "BVH::PrepareHQBuild( .. ), primCount == 0." );
		FATAL_ERROR_IF( prims != 0, "BVH::PrepareHQBuild( .. ), indices == 0." );
		// building a BVH over triangles specified as three 16-byte vertices each.
		for (uint32_t i = 0; i < triCount; i++)
		{
			const bvhvec4 v0 = verts[i * 3], v1 = verts[i * 3 + 1], v2 = verts[i * 3 + 2];
			const bvhvec4 fmin = tinybvh_min( v0, tinybvh_min( v1, v2 ) );
			const bvhvec4 fmax = tinybvh_max( v0, tinybvh_max( v1, v2 ) );
			fragment[i].bmin = fmin, fragment[i].bmax = fmax, fragment[i].primIdx = i, fragment[i].clipped = 0;
			root.aabbMin = tinybvh_min( root.aabbMin, fragment[i].bmin );
			root.aabbMax = tinybvh_max( root.aabbMax, fragment[i].bmax ), primIdx[i] = i;
		}
	}
	else
	{
		FATAL_ERROR_IF( vertices.count == 0, "BVH::PrepareHQBuild( .. ), empty vertex slice." );
		FATAL_ERROR_IF( prims == 0, "BVH::PrepareHQBuild( .. ), prims == 0." );
		// building a BVH over triangles consisting of vertices indexed by 'indices'.
		for (uint32_t i = 0; i < triCount; i++)
		{
			const uint32_t i0 = indices[i * 3], i1 = indices[i * 3 + 1], i2 = indices[i * 3 + 2];
			const bvhvec4 v0 = verts[i0], v1 = verts[i1], v2 = verts[i2];
			const bvhvec4 fmin = tinybvh_min( v0, tinybvh_min( v1, v2 ) );
			const bvhvec4 fmax = tinybvh_max( v0, tinybvh_max( v1, v2 ) );
			fragment[i].bmin = fmin, fragment[i].bmax = fmax, fragment[i].primIdx = i, fragment[i].clipped = 0;
			root.aabbMin = tinybvh_min( root.aabbMin, fragment[i].bmin );
			root.aabbMax = tinybvh_max( root.aabbMax, fragment[i].bmax ), primIdx[i] = i;
		}
	}
	// clear remainder of index array
	memset( primIdx + triCount, 0, slack * 4 );
	bvh_over_indices = indices != nullptr;
	// all set; actual build happens in BVH::Build.
}
void BVH::BuildHQ()
{
	const uint32_t slack = triCount >> 1; // for split prims
	uint32_t* triIdxA = primIdx;
	uint32_t* triIdxB = (uint32_t*)AlignedAlloc( (triCount + slack) * sizeof( uint32_t ) );
	memset( triIdxB, 0, (triCount + slack) * 4 );
	// reset node pool
	newNodePtr = 2;
	uint32_t nextFrag = triCount;
	// subdivide recursively
	BVHNode& root = bvhNode[0];
	const float rootArea = (root.aabbMax - root.aabbMin).halfArea();
	struct Task { uint32_t node, sliceStart, sliceEnd, dummy; };
	ALIGNED( 64 ) Task task[1024];
	uint32_t taskCount = 0, nodeIdx = 0, sliceStart = 0, sliceEnd = triCount + slack;
	const bvhvec3 minDim = (root.aabbMax - root.aabbMin) * 1e-7f /* don't touch, carefully picked */;
	bvhvec3 bestLMin = bvhvec3( 0 ), bestLMax = bvhvec3( 0 ), bestRMin = bvhvec3( 0 ), bestRMax = bvhvec3( 0 );
	while (1)
	{
		while (1)
		{
			BVHNode& node = bvhNode[nodeIdx];
			// find optimal object split
			bvhvec3 binMin[3][HQBVHBINS], binMax[3][HQBVHBINS];
			for (uint32_t a = 0; a < 3; a++) for (uint32_t i = 0; i < HQBVHBINS; i++) binMin[a][i] = bvhvec3( BVH_FAR ), binMax[a][i] = bvhvec3( -BVH_FAR );
			uint32_t count[3][HQBVHBINS];
			memset( count, 0, HQBVHBINS * 3 * sizeof( uint32_t ) );
			const bvhvec3 rpd3 = bvhvec3( HQBVHBINS / (node.aabbMax - node.aabbMin) ), nmin3 = node.aabbMin;
			for (uint32_t i = 0; i < node.triCount; i++) // process all tris for x,y and z at once
			{
				const uint32_t fi = primIdx[node.leftFirst + i];
				bvhint3 bi = bvhint3( ((fragment[fi].bmin + fragment[fi].bmax) * 0.5f - nmin3) * rpd3 );
				bi.x = tinybvh_clamp( bi.x, 0, HQBVHBINS - 1 );
				bi.y = tinybvh_clamp( bi.y, 0, HQBVHBINS - 1 );
				bi.z = tinybvh_clamp( bi.z, 0, HQBVHBINS - 1 );
				binMin[0][bi.x] = tinybvh_min( binMin[0][bi.x], fragment[fi].bmin );
				binMax[0][bi.x] = tinybvh_max( binMax[0][bi.x], fragment[fi].bmax ), count[0][bi.x]++;
				binMin[1][bi.y] = tinybvh_min( binMin[1][bi.y], fragment[fi].bmin );
				binMax[1][bi.y] = tinybvh_max( binMax[1][bi.y], fragment[fi].bmax ), count[1][bi.y]++;
				binMin[2][bi.z] = tinybvh_min( binMin[2][bi.z], fragment[fi].bmin );
				binMax[2][bi.z] = tinybvh_max( binMax[2][bi.z], fragment[fi].bmax ), count[2][bi.z]++;
			}
			// calculate per-split totals
			float splitCost = 1e30f, rSAV = 1.0f / node.SurfaceArea();
			uint32_t bestAxis = 0, bestPos = 0;
			for (int32_t a = 0; a < 3; a++) if ((node.aabbMax[a] - node.aabbMin[a]) > minDim.cell[a])
			{
				bvhvec3 lBMin[HQBVHBINS - 1], rBMin[HQBVHBINS - 1], l1 = bvhvec3( BVH_FAR ), l2 = bvhvec3( -BVH_FAR );
				bvhvec3 lBMax[HQBVHBINS - 1], rBMax[HQBVHBINS - 1], r1 = bvhvec3( BVH_FAR ), r2 = bvhvec3( -BVH_FAR );
				float ANL[HQBVHBINS - 1], ANR[HQBVHBINS - 1];
				for (uint32_t lN = 0, rN = 0, i = 0; i < HQBVHBINS - 1; i++)
				{
					lBMin[i] = l1 = tinybvh_min( l1, binMin[a][i] );
					rBMin[HQBVHBINS - 2 - i] = r1 = tinybvh_min( r1, binMin[a][HQBVHBINS - 1 - i] );
					lBMax[i] = l2 = tinybvh_max( l2, binMax[a][i] );
					rBMax[HQBVHBINS - 2 - i] = r2 = tinybvh_max( r2, binMax[a][HQBVHBINS - 1 - i] );
					lN += count[a][i], rN += count[a][HQBVHBINS - 1 - i];
					ANL[i] = lN == 0 ? BVH_FAR : ((l2 - l1).halfArea() * (float)lN);
					ANR[HQBVHBINS - 2 - i] = rN == 0 ? BVH_FAR : ((r2 - r1).halfArea() * (float)rN);
				}
				// evaluate bin totals to find best position for object split
				for (uint32_t i = 0; i < HQBVHBINS - 1; i++)
				{
					const float C = C_TRAV + C_INT * rSAV * (ANL[i] + ANR[i]);
					if (C >= splitCost) continue;
					splitCost = C, bestAxis = a, bestPos = i;
					bestLMin = lBMin[i], bestRMin = rBMin[i], bestLMax = lBMax[i], bestRMax = rBMax[i];
				}
			}
			// consider a spatial split
			bool spatial = false;
			uint32_t NL[HQBVHBINS - 1], NR[HQBVHBINS - 1], budget = sliceEnd - sliceStart, bestNL = 0, bestNR = 0;
			bvhvec3 spatialUnion = bestLMax - bestRMin;
			float spatialOverlap = (spatialUnion.halfArea()) / rootArea;
			if (budget > node.triCount && splitCost < 1e30f && spatialOverlap > 1e-5f)
			{
				for (uint32_t a = 0; a < 3; a++) if ((node.aabbMax[a] - node.aabbMin[a]) > minDim.cell[a])
				{
					// setup bins
					bvhvec3 binaMin[HQBVHBINS], binaMax[HQBVHBINS];
					for (uint32_t i = 0; i < HQBVHBINS; i++) binaMin[i] = bvhvec3( BVH_FAR ), binaMax[i] = bvhvec3( -BVH_FAR );
					uint32_t countIn[HQBVHBINS] = { 0 }, countOut[HQBVHBINS] = { 0 };
					// populate bins with clipped fragments
					const float planeDist = (node.aabbMax[a] - node.aabbMin[a]) / (HQBVHBINS * 0.9999f);
					const float rPlaneDist = 1.0f / planeDist, nodeMin = node.aabbMin[a];
					for (uint32_t i = 0; i < node.triCount; i++)
					{
						const uint32_t fragIdx = triIdxA[node.leftFirst + i];
						const int32_t bin1 = tinybvh_clamp( (int32_t)((fragment[fragIdx].bmin[a] - nodeMin) * rPlaneDist), 0, HQBVHBINS - 1 );
						const int32_t bin2 = tinybvh_clamp( (int32_t)((fragment[fragIdx].bmax[a] - nodeMin) * rPlaneDist), 0, HQBVHBINS - 1 );
						countIn[bin1]++, countOut[bin2]++;
						if (bin2 == bin1) // fragment fits in a single bin
							binaMin[bin1] = tinybvh_min( binaMin[bin1], fragment[fragIdx].bmin ),
							binaMax[bin1] = tinybvh_max( binaMax[bin1], fragment[fragIdx].bmax );
						else for (int32_t j = bin1; j <= bin2; j++)
						{
							// clip fragment to each bin it overlaps
							bvhvec3 bmin = node.aabbMin, bmax = node.aabbMax;
							bmin[a] = nodeMin + planeDist * j;
							bmax[a] = j == (HQBVHBINS - 2) ? node.aabbMax[a] : (bmin[a] + planeDist);
							Fragment orig = fragment[fragIdx];
							Fragment tmpFrag;
							if (!ClipFrag( orig, tmpFrag, bmin, bmax, minDim, a )) continue;
							binaMin[j] = tinybvh_min( binaMin[j], tmpFrag.bmin );
							binaMax[j] = tinybvh_max( binaMax[j], tmpFrag.bmax );
						}
					}
					// evaluate split candidates
					bvhvec3 lBMin[HQBVHBINS - 1], rBMin[HQBVHBINS - 1], l1 = bvhvec3( BVH_FAR ), l2 = bvhvec3( -BVH_FAR );
					bvhvec3 lBMax[HQBVHBINS - 1], rBMax[HQBVHBINS - 1], r1 = bvhvec3( BVH_FAR ), r2 = bvhvec3( -BVH_FAR );
					float ANL[HQBVHBINS], ANR[HQBVHBINS];
					for (uint32_t lN = 0, rN = 0, i = 0; i < HQBVHBINS - 1; i++)
					{
						lBMin[i] = l1 = tinybvh_min( l1, binaMin[i] ), rBMin[HQBVHBINS - 2 - i] = r1 = tinybvh_min( r1, binaMin[HQBVHBINS - 1 - i] );
						lBMax[i] = l2 = tinybvh_max( l2, binaMax[i] ), rBMax[HQBVHBINS - 2 - i] = r2 = tinybvh_max( r2, binaMax[HQBVHBINS - 1 - i] );
						lN += countIn[i], rN += countOut[HQBVHBINS - 1 - i], NL[i] = lN, NR[HQBVHBINS - 2 - i] = rN;
						ANL[i] = lN == 0 ? BVH_FAR : ((l2 - l1).halfArea() * (float)lN);
						ANR[HQBVHBINS - 2 - i] = rN == 0 ? BVH_FAR : ((r2 - r1).halfArea() * (float)rN);
					}
					// find best position for spatial split
					for (uint32_t i = 0; i < HQBVHBINS - 1; i++)
					{
						const float Cspatial = C_TRAV + C_INT * rSAV * (ANL[i] + ANR[i]);
						if (Cspatial < splitCost && NL[i] + NR[i] < budget)
						{
							spatial = true, splitCost = Cspatial, bestAxis = a, bestPos = i;
							bestLMin = lBMin[i], bestLMax = lBMax[i], bestRMin = rBMin[i], bestRMax = rBMax[i];
							bestNL = NL[i], bestNR = NR[i]; // for unsplitting
							bestLMax[a] = bestRMin[a]; // accurate
						}
					}
				}
			}
			// evaluate best split cost
			float noSplitCost = (float)node.triCount * C_INT;
			if (splitCost >= noSplitCost)
			{
				bvhvec3 nodeMin( BVH_FAR ), nodeMax( -BVH_FAR );
				for (uint32_t i = 0; i < node.triCount; i++)
					primIdx[node.leftFirst + i] = fragment[primIdx[node.leftFirst + i]].primIdx;
				break; // not splitting is better.
			}
			// double-buffered partition
			uint32_t A = sliceStart, B = sliceEnd, src = node.leftFirst;
			if (spatial)
			{
				// spatial partitioning
				const float planeDist = (node.aabbMax[bestAxis] - node.aabbMin[bestAxis]) / (HQBVHBINS * 0.9999f);
				const float rPlaneDist = 1.0f / planeDist, nodeMin = node.aabbMin[bestAxis];
				for (uint32_t i = 0; i < node.triCount; i++)
				{
					const uint32_t fragIdx = triIdxA[src++];
					const uint32_t bin1 = (uint32_t)tinybvh_max( (fragment[fragIdx].bmin[bestAxis] - nodeMin) * rPlaneDist, 0.0f );
					const uint32_t bin2 = (uint32_t)tinybvh_max( (fragment[fragIdx].bmax[bestAxis] - nodeMin) * rPlaneDist, 0.0f );
					if (bin2 <= bestPos) triIdxB[A++] = fragIdx; else if (bin1 > bestPos) triIdxB[--B] = fragIdx; else
					{
					#ifdef SBVH_UNSPLITTING
						// unsplitting: 1. Calculate what happens if we add this primitive entirely to the left side
						if (bestNR > 1)
						{
							bvhvec3 unsplitLMin = tinybvh_min( bestLMin, fragment[fragIdx].bmin );
							bvhvec3 unsplitLMax = tinybvh_max( bestLMax, fragment[fragIdx].bmax );
							float AL = (unsplitLMax - unsplitLMin).halfArea();
							float AR = (bestRMax - bestRMin).halfArea();
							float CunsplitLeft = C_TRAV + C_INT * rSAV * (AL * bestNL + AR * (bestNR - 1));
							if (CunsplitLeft < splitCost)
							{
								bestNR--, splitCost = CunsplitLeft, triIdxB[A++] = fragIdx;
								bestLMin = unsplitLMin, bestLMax = unsplitLMax;
								continue;
							}
						}
						// 2. Calculate what happens if we add this primitive entirely to the right side
						if (bestNL > 1)
						{
							const bvhvec3 unsplitRMin = tinybvh_min( bestRMin, fragment[fragIdx].bmin );
							const bvhvec3 unsplitRMax = tinybvh_max( bestRMax, fragment[fragIdx].bmax );
							const float AL = (bestLMax - bestLMin).halfArea();
							const float AR = (unsplitRMax - unsplitRMin).halfArea();
							const float CunsplitRight = C_TRAV + C_INT * rSAV * (AL * (bestNL - 1) + AR * bestNR);
							if (CunsplitRight < splitCost)
							{
								bestNL--, splitCost = CunsplitRight, triIdxB[--B] = fragIdx;
								bestRMin = unsplitRMin, bestRMax = unsplitRMax;
								continue;
							}
						}
					#endif
						// split straddler
						ALIGNED( 64 ) Fragment part1, part2; // keep all clipping in a single cacheline.
						bool leftOK = false, rightOK = false;
						float splitPos = bestLMax[bestAxis];
						SplitFrag( fragment[fragIdx], part1, part2, minDim, bestAxis, splitPos, leftOK, rightOK );
						if (leftOK && rightOK)
							fragment[fragIdx] = part1, triIdxB[A++] = fragIdx,
							fragment[nextFrag] = part2, triIdxB[--B] = nextFrag++;
						else // didn't work out; unsplit (rare)
							if (leftOK) triIdxB[A++] = fragIdx; else triIdxB[--B] = fragIdx;
					}
				}
				// for spatial splits, we fully refresh the bounds: clipping is never fully stable..
				bestLMin = bestRMin = bvhvec3( BVH_FAR ), bestLMax = bestRMax = bvhvec3( -BVH_FAR );
				for (uint32_t i = sliceStart; i < A; i++)
					bestLMin = tinybvh_min( bestLMin, fragment[triIdxB[i]].bmin ),
					bestLMax = tinybvh_max( bestLMax, fragment[triIdxB[i]].bmax );
				for (uint32_t i = B; i < sliceEnd; i++)
					bestRMin = tinybvh_min( bestRMin, fragment[triIdxB[i]].bmin ),
					bestRMax = tinybvh_max( bestRMax, fragment[triIdxB[i]].bmax );
			}
			else
			{
				// object partitioning
				const float rpd = rpd3.cell[bestAxis], nmin = nmin3.cell[bestAxis];
				for (uint32_t i = 0; i < node.triCount; i++)
				{
					const uint32_t fr = primIdx[src + i];
					int32_t bi = (int32_t)(((fragment[fr].bmin[bestAxis] + fragment[fr].bmax[bestAxis]) * 0.5f - nmin) * rpd);
					bi = tinybvh_clamp( bi, 0, HQBVHBINS - 1 );
					if (bi <= (int32_t)bestPos) triIdxB[A++] = fr; else triIdxB[--B] = fr;
				}
			}
			// copy back slice data
			memcpy( triIdxA + sliceStart, triIdxB + sliceStart, (sliceEnd - sliceStart) * 4 );
			// create child nodes
			uint32_t leftCount = A - sliceStart, rightCount = sliceEnd - B;
			if (leftCount == 0 || rightCount == 0) break;
			int32_t leftChildIdx = newNodePtr++, rightChildIdx = newNodePtr++;
			bvhNode[leftChildIdx].aabbMin = bestLMin, bvhNode[leftChildIdx].aabbMax = bestLMax;
			bvhNode[leftChildIdx].leftFirst = sliceStart, bvhNode[leftChildIdx].triCount = leftCount;
			bvhNode[rightChildIdx].aabbMin = bestRMin, bvhNode[rightChildIdx].aabbMax = bestRMax;
			bvhNode[rightChildIdx].leftFirst = B, bvhNode[rightChildIdx].triCount = rightCount;
			node.leftFirst = leftChildIdx, node.triCount = 0;
			// recurse
			task[taskCount].node = rightChildIdx, task[taskCount].sliceEnd = sliceEnd;
			task[taskCount++].sliceStart = sliceEnd = (A + B) >> 1, nodeIdx = leftChildIdx;
		}
		// fetch subdivision task from stack
		if (taskCount == 0) break; else
			nodeIdx = task[--taskCount].node,
			sliceStart = task[taskCount].sliceStart,
			sliceEnd = task[taskCount].sliceEnd;
	}
	// all done.
	AlignedFree( triIdxB );
	aabbMin = bvhNode[0].aabbMin, aabbMax = bvhNode[0].aabbMax;
	refittable = false; // can't refit an SBVH
	may_have_holes = false; // there may be holes in the index list, but not in the node list
	usedNodes = newNodePtr;
	Compact();
}

// Optimize: Will happen via BVH_Verbose.
void BVH::Optimize( const uint32_t iterations, bool extreme )
{
	BVH_Verbose* verbose = new BVH_Verbose();
	verbose->ConvertFrom( *this );
	verbose->Optimize( iterations, extreme );
	ConvertFrom( *verbose );
}

// Refitting: For animated meshes, where the topology remains intact. This
// includes trees waving in the wind, or subsequent frames for skinned
// animations. Repeated refitting tends to lead to deteriorated BVHs and
// slower ray tracing. Rebuild when this happens.
void BVH::Refit( const uint32_t /* unused */ )
{
	FATAL_ERROR_IF( !refittable, "BVH::Refit( .. ), refitting an SBVH." );
	FATAL_ERROR_IF( bvhNode == 0, "BVH::Refit( .. ), bvhNode == 0." );
	FATAL_ERROR_IF( may_have_holes, "BVH::Refit( .. ), bvh may have holes." );
	for (int32_t i = usedNodes - 1; i >= 0; i--)
	{
		BVHNode& node = bvhNode[i];
		if (node.isLeaf()) // leaf: adjust to current triangle vertex positions
		{
			bvhvec4 bmin( BVH_FAR ), bmax( -BVH_FAR );
			if (vertIdx)
			{
				for (uint32_t first = node.leftFirst, j = 0; j < node.triCount; j++)
				{
					const uint32_t vidx = primIdx[first + j] * 3;
					const uint32_t i0 = vertIdx[vidx], i1 = vertIdx[vidx + 1], i2 = vertIdx[vidx + 2];
					const bvhvec4 v0 = verts[i0], v1 = verts[i1], v2 = verts[i2];
					const bvhvec4 t1 = tinybvh_min( v0, bmin );
					const bvhvec4 t2 = tinybvh_max( v0, bmax );
					const bvhvec4 t3 = tinybvh_min( v1, v2 );
					const bvhvec4 t4 = tinybvh_max( v1, v2 );
					bmin = tinybvh_min( t1, t3 );
					bmax = tinybvh_max( t2, t4 );
				}
			}
			else
			{
				for (uint32_t first = node.leftFirst, j = 0; j < node.triCount; j++)
				{
					const uint32_t vidx = primIdx[first + j] * 3;
					const bvhvec4 v0 = verts[vidx], v1 = verts[vidx + 1], v2 = verts[vidx + 2];
					const bvhvec4 t1 = tinybvh_min( v0, bmin );
					const bvhvec4 t2 = tinybvh_max( v0, bmax );
					const bvhvec4 t3 = tinybvh_min( v1, v2 );
					const bvhvec4 t4 = tinybvh_max( v1, v2 );
					bmin = tinybvh_min( t1, t3 );
					bmax = tinybvh_max( t2, t4 );
				}
			}
			node.aabbMin = aabbMin, node.aabbMax = aabbMax;
			continue;
		}
		// interior node: adjust to child bounds
		const BVHNode& left = bvhNode[node.leftFirst], & right = bvhNode[node.leftFirst + 1];
		node.aabbMin = tinybvh_min( left.aabbMin, right.aabbMin );
		node.aabbMax = tinybvh_max( left.aabbMax, right.aabbMax );
	}
	aabbMin = bvhNode[0].aabbMin, aabbMax = bvhNode[0].aabbMax;
}

bool BVH::IntersectSphere( const bvhvec3& pos, const float r ) const
{
	const bvhvec3 bmin = pos - bvhvec3( r ), bmax = pos + bvhvec3( r );
	BVHNode* node = &bvhNode[0], * stack[64];
	uint32_t stackPtr = 0;
	const float r2 = r * r;
	while (1)
	{
		if (node->isLeaf())
		{
			// check if the leaf aabb overlaps the sphere: https://gamedev.stackexchange.com/a/156877
			float dist2 = 0;
			if (pos.x < bmin.x) dist2 += (bmin.x - pos.x) * (bmin.x - pos.x);
			if (pos.x > bmax.x) dist2 += (pos.x - bmax.x) * (pos.x - bmax.x);
			if (pos.y < bmin.y) dist2 += (bmin.y - pos.y) * (bmin.y - pos.y);
			if (pos.y > bmax.y) dist2 += (pos.y - bmax.y) * (pos.y - bmax.y);
			if (pos.z < bmin.z) dist2 += (bmin.z - pos.z) * (bmin.z - pos.z);
			if (pos.z > bmax.z) dist2 += (pos.z - bmax.z) * (pos.z - bmax.z);
			if (dist2 <= r2)
			{
				// tri/sphere test: https://gist.github.com/yomotsu/d845f21e2e1eb49f647f#file-gistfile1-js-L223
				for (uint32_t i = 0; i < node->triCount; i++)
				{
					uint32_t idx = primIdx[node->leftFirst + i];
					bvhvec3 a, b, c;
					if (!vertIdx) idx *= 3, a = verts[idx], b = verts[idx + 1], c = verts[idx + 2]; else
					{
						const uint32_t i0 = vertIdx[idx * 3], i1 = vertIdx[idx * 3 + 1], i2 = vertIdx[idx * 3 + 2];
						a = verts[i0], b = verts[i1], c = verts[i2];
					}
					const bvhvec3 A = a - pos, B = b - pos, C = c - pos;
					const float rr = r * r;
					const bvhvec3 V = tinybvh_cross( B - A, C - A );
					const float d = tinybvh_dot( A, V ), e = tinybvh_dot( V, V );
					if (d * d > rr * e) continue;
					const float aa = tinybvh_dot( A, A ), ab = tinybvh_dot( A, B ), ac = tinybvh_dot( A, C );
					const float bb = tinybvh_dot( B, B ), bc = tinybvh_dot( B, C ), cc = tinybvh_dot( C, C );
					if ((aa > rr && ab > aa && ac > aa) ||
						(bb > rr && ab > bb && bc > bb) ||
						(cc > rr && ac > cc && bc > cc)) continue;
					const bvhvec3 AB = B - A, BC = C - B, CA = A - C;
					const float d1 = ab - aa, d2 = bc - bb, d3 = ac - cc;
					const float e1 = tinybvh_dot( AB, AB ), e2 = tinybvh_dot( BC, BC ), e3 = tinybvh_dot( CA, CA );
					const bvhvec3 Q1 = A * e1 - AB * d1, Q2 = B * e2 - BC * d2, Q3 = C * e3 - CA * d3;
					const bvhvec3 QC = C * e1 - Q1, QA = A * e2 - Q2, QB = B * e3 - Q3;
					if ((tinybvh_dot( Q1, Q1 ) > rr * e1 * e1 && tinybvh_dot( Q1, QC ) >= 0) ||
						(tinybvh_dot( Q2, Q2 ) > rr * e2 * e2 && tinybvh_dot( Q2, QA ) >= 0) ||
						(tinybvh_dot( Q3, Q3 ) > rr * e3 * e3 && tinybvh_dot( Q3, QB ) >= 0)) continue;
					// const float dist = sqrtf( d * d / e ) - r; // we're not using this.
					return true;
				}
			}
			if (stackPtr == 0) break; else node = stack[--stackPtr];
		}
		BVHNode* child1 = &bvhNode[node->leftFirst], * child2 = &bvhNode[node->leftFirst + 1];
		bool hit1 = child1->Intersect( bmin, bmax ), hit2 = child2->Intersect( bmin, bmax );
		if (hit1 && hit2) stack[stackPtr++] = child2, node = child1;
		else if (hit1) node = child1; else if (hit2) node = child2;
		else { if (stackPtr == 0) break; node = stack[--stackPtr]; }
	}
	return false;
}

int32_t BVH::Intersect( Ray& ray ) const
{
	if (isTLAS()) return IntersectTLAS( ray );
	BVHNode* node = &bvhNode[0], * stack[64];
	uint32_t stackPtr = 0, cost = 0;
	while (1)
	{
		cost += C_TRAV;
		if (node->isLeaf())
		{
			// Performance note: if indexed primitives (ENABLE_INDEXED_GEOMETRY) and custom
			// geometry (ENABLE_CUSTOM_GEOMETRY) are both disabled, this leaf code reduces
			// to a regular loop over triangles. Otherwise, the extra flexibility comes at
			// a small performance cost.
			if (indexedEnabled && vertIdx != 0) for (uint32_t i = 0; i < node->triCount; i++, cost += C_INT)
				IntersectTriIndexed( ray, verts, vertIdx, primIdx[node->leftFirst + i] );
			else if (customEnabled && customIntersect != 0) for (uint32_t i = 0; i < node->triCount; i++, cost += C_INT)
			{
				if ((*customIntersect)(ray, primIdx[node->leftFirst + i]))
				{
				#if INST_IDX_BITS == 32
					ray.hit.inst = ray.instIdx;
				#else
					ray.hit.prim = (ray.hit.prim & PRIM_IDX_MASK) + ray.instIdx;
				#endif
			}
		}
			else for (uint32_t i = 0; i < node->triCount; i++, cost += C_INT)
				IntersectTri( ray, verts, primIdx[node->leftFirst + i] );
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
	}
		BVHNode* child1 = &bvhNode[node->leftFirst];
		BVHNode* child2 = &bvhNode[node->leftFirst + 1];
		float dist1 = child1->Intersect( ray ), dist2 = child2->Intersect( ray );
		if (dist1 > dist2) { tinybvh_swap( dist1, dist2 ); tinybvh_swap( child1, child2 ); }
		if (dist1 == BVH_FAR /* missed both child nodes */)
		{
			if (stackPtr == 0) break; else node = stack[--stackPtr];
		}
		else /* hit at least one node */
		{
			node = child1; /* continue with the nearest */
			if (dist2 != BVH_FAR) stack[stackPtr++] = child2; /* push far child */
		}
}
	return cost;
}

int32_t BVH::IntersectTLAS( Ray& ray ) const
{
	BVHNode* node = &bvhNode[0], * stack[64];
	uint32_t stackPtr = 0, cost = 0;
	while (1)
	{
		cost += C_TRAV;
		if (node->isLeaf())
		{
			Ray tmp;
			for (uint32_t i = 0; i < node->triCount; i++)
			{
				// BLAS traversal
				const uint32_t instIdx = primIdx[node->leftFirst + i];
				const BLASInstance& inst = instList[instIdx];
				const BVHBase* blas = blasList[inst.blasIdx];
				// 1. Transform ray with the inverse of the instance transform
				tmp.O = tinybvh_transform_point( ray.O, inst.invTransform );
				tmp.D = tinybvh_transform_vector( ray.D, inst.invTransform );
				tmp.instIdx = instIdx << (32 - INST_IDX_BITS);
				tmp.hit = ray.hit;
				tmp.rD = tinybvh_safercp( tmp.D );
				// 2. Traverse BLAS with the transformed ray
				// Note: Valid BVH layout options for BLASses are the regular BVH layout,
				// the AVX-optimized BVH_SOA layout and the wide BVH4_CPU layout. If all
				// BLASses are of the same layout this reduces to nearly zero cost for
				// a small set of predictable branches.
				assert( blas->layout == LAYOUT_BVH || blas->layout == LAYOUT_BVH4_CPU || blas->layout == LAYOUT_BVH_SOA );
				if (blas->layout == LAYOUT_BVH)
				{
					// regular (triangle) BVH traversal
					cost += ((BVH*)blas)->Intersect( tmp );
				}
				else
				{
					if (blas->layout == LAYOUT_BVH4_CPU) cost += ((BVH4_CPU*)blas)->Intersect( tmp );
					else if (blas->layout == LAYOUT_BVH_SOA) cost += ((BVH_SoA*)blas)->Intersect( tmp );
				}
				// 3. Restore ray
				ray.hit = tmp.hit;
			}
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		BVHNode* child1 = &bvhNode[node->leftFirst];
		BVHNode* child2 = &bvhNode[node->leftFirst + 1];
		float dist1 = child1->Intersect( ray ), dist2 = child2->Intersect( ray );
		if (dist1 > dist2) { tinybvh_swap( dist1, dist2 ); tinybvh_swap( child1, child2 ); }
		if (dist1 == BVH_FAR /* missed both child nodes */)
		{
			if (stackPtr == 0) break; else node = stack[--stackPtr];
		}
		else /* hit at least one node */
		{
			node = child1; /* continue with the nearest */
			if (dist2 != BVH_FAR) stack[stackPtr++] = child2; /* push far child */
		}
	}
	return cost;
}

bool BVH::IsOccluded( const Ray& ray ) const
{
	if (isTLAS()) return IsOccludedTLAS( ray );
	BVHNode* node = &bvhNode[0], * stack[64];
	uint32_t stackPtr = 0;
	while (1)
	{
		if (node->isLeaf())
		{
			if (indexedEnabled && vertIdx != 0)
			{
				for (uint32_t i = 0; i < node->triCount; i++)
					if (IndexedTriOccludes( ray, verts, vertIdx, primIdx[node->leftFirst + i] )) return true;
			}
			else if (customEnabled && customIsOccluded != 0)
			{
				for (uint32_t i = 0; i < node->triCount; i++)
					if ((*customIsOccluded)(ray, primIdx[node->leftFirst + i])) return true;
			}
			else
			{
				for (uint32_t i = 0; i < node->triCount; i++)
					if (TriOccludes( ray, verts, primIdx[node->leftFirst + i] )) return true;
			}
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		BVHNode* child1 = &bvhNode[node->leftFirst];
		BVHNode* child2 = &bvhNode[node->leftFirst + 1];
		float dist1 = child1->Intersect( ray ), dist2 = child2->Intersect( ray );
		if (dist1 > dist2) { tinybvh_swap( dist1, dist2 ); tinybvh_swap( child1, child2 ); }
		if (dist1 == BVH_FAR /* missed both child nodes */)
		{
			if (stackPtr == 0) break; else node = stack[--stackPtr];
		}
		else /* hit at least one node */
		{
			node = child1; /* continue with the nearest */
			if (dist2 != BVH_FAR) stack[stackPtr++] = child2; /* push far child */
		}
	}
	return false;
}

bool BVH::IsOccludedTLAS( const Ray& ray ) const
{
	BVHNode* node = &bvhNode[0], * stack[64];
	uint32_t stackPtr = 0;
	Ray tmp;
	tmp.hit = ray.hit;
	while (1)
	{
		if (node->isLeaf())
		{
			Ray tmp;
			for (uint32_t i = 0; i < node->triCount; i++)
			{
				// BLAS traversal
				BLASInstance& inst = instList[primIdx[node->leftFirst + i]];
				const BVHBase* blas = blasList[inst.blasIdx];
				// 1. Transform ray with the inverse of the instance transform
				tmp.O = tinybvh_transform_point( ray.O, inst.invTransform );
				tmp.D = tinybvh_transform_vector( ray.D, inst.invTransform );
				tmp.hit.t = ray.hit.t;
				tmp.rD = tinybvh_safercp( tmp.D );
				// 2. Traverse BLAS with the transformed ray
				assert( blas->layout == LAYOUT_BVH || blas->layout == LAYOUT_BVH4_CPU || blas->layout == LAYOUT_BVH_SOA );
				if (blas->layout == LAYOUT_BVH)
				{
					// regular (triangle) BVH traversal
					if (((BVH*)blas)->IsOccluded( tmp )) return true;
				}
				else
				{
					if (blas->layout == LAYOUT_BVH4_CPU) { if (((BVH4_CPU*)blas)->IsOccluded( tmp )) return true; }
					else if (blas->layout == LAYOUT_BVH_SOA) { if (((BVH_SoA*)blas)->IsOccluded( tmp )) return true; }
				}
			}
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		BVHNode* child1 = &bvhNode[node->leftFirst];
		BVHNode* child2 = &bvhNode[node->leftFirst + 1];
		float dist1 = child1->Intersect( ray ), dist2 = child2->Intersect( ray );
		if (dist1 > dist2) { tinybvh_swap( dist1, dist2 ); tinybvh_swap( child1, child2 ); }
		if (dist1 == BVH_FAR /* missed both child nodes */)
		{
			if (stackPtr == 0) break; else node = stack[--stackPtr];
		}
		else /* hit at least one node */
		{
			node = child1; /* continue with the nearest */
			if (dist2 != BVH_FAR) stack[stackPtr++] = child2; /* push far child */
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
void BVH::Intersect256Rays( Ray* packet ) const
{
	// convenience macro
#define CALC_TMIN_TMAX_WITH_SLABTEST_ON_RAY( r ) const bvhvec3 rD = packet[r].rD, t1 = o1 * rD, t2 = o2 * rD; \
	const float tmin = tinybvh_max( tinybvh_max( tinybvh_min( t1.x, t2.x ), tinybvh_min( t1.y, t2.y ) ), tinybvh_min( t1.z, t2.z ) ); \
	const float tmax = tinybvh_min( tinybvh_min( tinybvh_max( t1.x, t2.x ), tinybvh_max( t1.y, t2.y ) ), tinybvh_max( t1.z, t2.z ) );
	// Corner rays are: 0, 51, 204 and 255
	// Construct the bounding planes, with normals pointing outwards
	const bvhvec3 O = packet[0].O; // same for all rays in this case
	const bvhvec3 p0 = packet[0].O + packet[0].D; // top-left
	const bvhvec3 p1 = packet[51].O + packet[51].D; // top-right
	const bvhvec3 p2 = packet[204].O + packet[204].D; // bottom-left
	const bvhvec3 p3 = packet[255].O + packet[255].D; // bottom-right
	const bvhvec3 plane0 = tinybvh_normalize( tinybvh_cross( p0 - O, p0 - p2 ) ); // left plane
	const bvhvec3 plane1 = tinybvh_normalize( tinybvh_cross( p3 - O, p3 - p1 ) ); // right plane
	const bvhvec3 plane2 = tinybvh_normalize( tinybvh_cross( p1 - O, p1 - p0 ) ); // top plane
	const bvhvec3 plane3 = tinybvh_normalize( tinybvh_cross( p2 - O, p2 - p3 ) ); // bottom plane
	const int32_t sign0x = plane0.x < 0 ? 4 : 0, sign0y = plane0.y < 0 ? 5 : 1, sign0z = plane0.z < 0 ? 6 : 2;
	const int32_t sign1x = plane1.x < 0 ? 4 : 0, sign1y = plane1.y < 0 ? 5 : 1, sign1z = plane1.z < 0 ? 6 : 2;
	const int32_t sign2x = plane2.x < 0 ? 4 : 0, sign2y = plane2.y < 0 ? 5 : 1, sign2z = plane2.z < 0 ? 6 : 2;
	const int32_t sign3x = plane3.x < 0 ? 4 : 0, sign3y = plane3.y < 0 ? 5 : 1, sign3z = plane3.z < 0 ? 6 : 2;
	const float d0 = tinybvh_dot( O, plane0 ), d1 = tinybvh_dot( O, plane1 );
	const float d2 = tinybvh_dot( O, plane2 ), d3 = tinybvh_dot( O, plane3 );
	// Traverse the tree with the packet
	int32_t first = 0, last = 255; // first and last active ray in the packet
	const BVHNode* node = &bvhNode[0];
	ALIGNED( 64 ) uint32_t stack[64], stackPtr = 0;
	while (1)
	{
		if (node->isLeaf())
		{
			// handle leaf node
			for (uint32_t j = 0; j < node->triCount; j++)
			{
				const uint32_t idx = primIdx[node->leftFirst + j], vid = idx * 3;
				const bvhvec3 edge1 = verts[vid + 1] - verts[vid], edge2 = verts[vid + 2] - verts[vid];
				const bvhvec3 s = O - bvhvec3( verts[vid] );
				for (int32_t i = first; i <= last; i++)
				{
					Ray& ray = packet[i];
					const bvhvec3 h = tinybvh_cross( ray.D, edge2 );
					const float a = tinybvh_dot( edge1, h );
					if (fabs( a ) < 0.0000001f) continue; // ray parallel to triangle
					const float f = 1 / a, u = f * tinybvh_dot( s, h );
					if (u < 0 || u > 1) continue;
					const bvhvec3 q = tinybvh_cross( s, edge1 );
					const float v = f * tinybvh_dot( ray.D, q );
					if (v < 0 || u + v > 1) continue;
					const float t = f * tinybvh_dot( edge2, q );
					if (t <= 0 || t >= ray.hit.t) continue;
					ray.hit.t = t, ray.hit.u = u, ray.hit.v = v;
				#if INST_IDX_BITS == 32
					ray.hit.prim = idx;
					ray.hit.inst = ray.instIdx;
				#else
					ray.hit.prim = idx + ray.instIdx;
				#endif
				}
			}
			if (stackPtr == 0) break; else // pop
				last = stack[--stackPtr], node = bvhNode + stack[--stackPtr],
				first = last >> 8, last &= 255;
		}
		else
		{
			// fetch pointers to child nodes
			const BVHNode* left = bvhNode + node->leftFirst;
			const BVHNode* right = bvhNode + node->leftFirst + 1;
			bool visitLeft = true, visitRight = true;
			int32_t leftFirst = first, leftLast = last, rightFirst = first, rightLast = last;
			float distLeft, distRight;
			{
				// see if we want to intersect the left child
				const bvhvec3 o1( left->aabbMin.x - O.x, left->aabbMin.y - O.y, left->aabbMin.z - O.z );
				const bvhvec3 o2( left->aabbMax.x - O.x, left->aabbMax.y - O.y, left->aabbMax.z - O.z );
				// 1. Early-in test: if first ray hits the node, the packet visits the node
				bool earlyHit;
				{
					CALC_TMIN_TMAX_WITH_SLABTEST_ON_RAY( first );
					earlyHit = (tmax >= tmin && tmin < packet[first].hit.t && tmax >= 0);
					distLeft = tmin;
				}
				if (!earlyHit) // 2. Early-out test: if the node aabb is outside the four planes, we skip the node
				{
					float* minmax = (float*)left;
					bvhvec3 c0( minmax[sign0x], minmax[sign0y], minmax[sign0z] );
					bvhvec3 c1( minmax[sign1x], minmax[sign1y], minmax[sign1z] );
					bvhvec3 c2( minmax[sign2x], minmax[sign2y], minmax[sign2z] );
					bvhvec3 c3( minmax[sign3x], minmax[sign3y], minmax[sign3z] );
					if (tinybvh_dot( c0, plane0 ) > d0 || tinybvh_dot( c1, plane1 ) > d1 ||
						tinybvh_dot( c2, plane2 ) > d2 || tinybvh_dot( c3, plane3 ) > d3)
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
				const bvhvec3 o1( right->aabbMin.x - O.x, right->aabbMin.y - O.y, right->aabbMin.z - O.z );
				const bvhvec3 o2( right->aabbMax.x - O.x, right->aabbMax.y - O.y, right->aabbMax.z - O.z );
				// 1. Early-in test: if first ray hits the node, the packet visits the node
				bool earlyHit;
				{
					CALC_TMIN_TMAX_WITH_SLABTEST_ON_RAY( first );
					earlyHit = (tmax >= tmin && tmin < packet[first].hit.t && tmax >= 0);
					distRight = tmin;
				}
				if (!earlyHit) // 2. Early-out test: if the node aabb is outside the four planes, we skip the node
				{
					float* minmax = (float*)right;
					bvhvec3 c0( minmax[sign0x], minmax[sign0y], minmax[sign0z] );
					bvhvec3 c1( minmax[sign1x], minmax[sign1y], minmax[sign1z] );
					bvhvec3 c2( minmax[sign2x], minmax[sign2y], minmax[sign2z] );
					bvhvec3 c3( minmax[sign3x], minmax[sign3y], minmax[sign3z] );
					if (tinybvh_dot( c0, plane0 ) > d0 || tinybvh_dot( c1, plane1 ) > d1 ||
						tinybvh_dot( c2, plane2 ) > d2 || tinybvh_dot( c3, plane3 ) > d3)
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
				last = stack[--stackPtr], node = bvhNode + stack[--stackPtr],
				first = last >> 8, last &= 255;
		}
	}
}

int32_t BVH::NodeCount() const
{
	// Determine the number of nodes in the tree. Typically the result should
	// be usedNodes - 1 (second node is always unused), but some builders may
	// have unused nodes besides node 1. TODO: Support more layouts.
	uint32_t retVal = 0, nodeIdx = 0, stack[64], stackPtr = 0;
	while (1)
	{
		const BVHNode& n = bvhNode[nodeIdx];
		retVal++;
		if (n.isLeaf()) { if (stackPtr == 0) break; else nodeIdx = stack[--stackPtr]; }
		else nodeIdx = n.leftFirst, stack[stackPtr++] = n.leftFirst + 1;
	}
	return retVal;
}

// Compact: Reduce the size of a BVH by removing any unsed nodes.
// This is useful after an SBVH build or multi-threaded build, but also after
// calling MergeLeafs. Some operations, such as Optimize, *require* a
// compacted tree to work correctly.
void BVH::Compact()
{
	FATAL_ERROR_IF( bvhNode == 0, "BVH::Compact(), bvhNode == 0." );
	BVHNode* tmp = (BVHNode*)AlignedAlloc( sizeof( BVHNode ) * allocatedNodes /* do *not* trim */ );
	memcpy( tmp, bvhNode, 2 * sizeof( BVHNode ) );
	newNodePtr = 2;
	uint32_t nodeIdx = 0, stack[128], stackPtr = 0;
	while (1)
	{
		BVHNode& node = tmp[nodeIdx];
		const BVHNode& left = bvhNode[node.leftFirst];
		const BVHNode& right = bvhNode[node.leftFirst + 1];
		tmp[newNodePtr] = left, tmp[newNodePtr + 1] = right;
		const uint32_t todo1 = newNodePtr, todo2 = newNodePtr + 1;
		node.leftFirst = newNodePtr, newNodePtr += 2;
		if (!left.isLeaf()) stack[stackPtr++] = todo1;
		if (!right.isLeaf()) stack[stackPtr++] = todo2;
		if (!stackPtr) break;
		nodeIdx = stack[--stackPtr];
	}
	usedNodes = newNodePtr;
	AlignedFree( bvhNode );
	bvhNode = tmp;
}

// BVH_Verbose implementation
// ----------------------------------------------------------------------------

void BVH_Verbose::ConvertFrom( const BVH& original, bool /* unused here */ )
{
	// allocate space
	uint32_t spaceNeeded = original.triCount * (refittable ? 2 : 3);
	if (allocatedNodes < spaceNeeded)
	{
		AlignedFree( bvhNode );
		bvhNode = (BVHNode*)AlignedAlloc( sizeof( BVHNode ) * spaceNeeded );
		allocatedNodes = spaceNeeded;
	}
	memset( bvhNode, 0, sizeof( BVHNode ) * spaceNeeded );
	CopyBasePropertiesFrom( original );
	this->verts = original.verts;
	this->fragment = original.fragment;
	this->primIdx = original.primIdx;
	bvhNode[0].parent = 0xffffffff; // root sentinel
	// convert
	uint32_t nodeIdx = 0, parent = 0xffffffff, stack[128], stackPtr = 0;
	while (1)
	{
		const BVH::BVHNode& orig = original.bvhNode[nodeIdx];
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

int32_t BVH_Verbose::NodeCount() const
{
	// Determine the number of nodes in the tree. Typically the result should
	// be usedNodes - 1 (second node is always unused), but some builders may
	// have unused nodes besides node 1. TODO: Support more layouts.
	uint32_t retVal = 0, nodeIdx = 0, stack[64], stackPtr = 0;
	while (1)
	{
		const BVHNode& n = bvhNode[nodeIdx];
		retVal++;
		if (n.isLeaf()) { if (stackPtr == 0) break; else nodeIdx = stack[--stackPtr]; }
		else nodeIdx = n.left, stack[stackPtr++] = n.right;
	}
	return retVal;
}

float BVH_Verbose::SAHCost( const uint32_t nodeIdx ) const
{
	// Determine the SAH cost of the tree. This provides an indication
	// of the quality of the BVH: Lower is better.
	const BVHNode& n = bvhNode[nodeIdx];
	const float SAn = SA( n.aabbMin, n.aabbMax );
	if (n.isLeaf()) return C_INT * SAn * n.triCount;
	float cost = C_TRAV * SAn + SAHCost( n.left ) + SAHCost( n.right );
	return nodeIdx == 0 ? (cost / SAn) : cost;
}

void BVH_Verbose::Refit( const uint32_t nodeIdx, bool skipLeafs )
{
	FATAL_ERROR_IF( !refittable && !skipLeafs, "BVH_Verbose::Refit( .. ), refitting an SBVH." );
	FATAL_ERROR_IF( bvhNode == 0, "BVH_Verbose::Refit( .. ), bvhNode == 0." );
	FATAL_ERROR_IF( bvh_over_indices && !skipLeafs, "BVH_Verbose::Refit( .. ), bvh used indexed tris." );
	BVHNode& node = bvhNode[nodeIdx];
	if (node.isLeaf()) // leaf: adjust to current triangle vertex positions
	{
		if (skipLeafs) return;
		bvhvec3 bmin( BVH_FAR ), bmax( -BVH_FAR );
		for (uint32_t first = node.firstTri, j = 0; j < node.triCount; j++)
		{
			const uint32_t vertIdx = primIdx[first + j] * 3;
			const bvhvec3 v0 = verts[vertIdx];
			const bvhvec3 v1 = verts[vertIdx + 1];
			const bvhvec3 v2 = verts[vertIdx + 2];
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

void BVH_Verbose::CheckFit( const uint32_t nodeIdx, bool skipLeafs )
{
	BVHNode& node = bvhNode[nodeIdx];
	bvhvec3 bmin( BVH_FAR ), bmax( -BVH_FAR );
	if (node.isLeaf()) // leaf: adjust to current triangle vertex positions
	{
		if (skipLeafs) return;
		for (uint32_t first = node.firstTri, j = 0; j < node.triCount; j++)
		{
			const uint32_t vertIdx = primIdx[first + j] * 3;
			const bvhvec3 v0 = verts[vertIdx];
			const bvhvec3 v1 = verts[vertIdx + 1];
			const bvhvec3 v2 = verts[vertIdx + 2];
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

void BVH_Verbose::Compact()
{
	FATAL_ERROR_IF( bvhNode == 0, "BVH_Verbose::Compact(), bvhNode == 0." );
	BVHNode* tmp = (BVHNode*)AlignedAlloc( sizeof( BVHNode ) * usedNodes );
	memcpy( tmp, bvhNode, 2 * sizeof( BVHNode ) );
	uint32_t newNodePtr = 2, nodeIdx = 0, stack[64], stackPtr = 0;
	while (1)
	{
		BVHNode& node = tmp[nodeIdx];
		const BVHNode& left = bvhNode[node.left];
		const BVHNode& right = bvhNode[node.right];
		tmp[newNodePtr] = left, tmp[newNodePtr + 1] = right;
		const uint32_t todo1 = newNodePtr, todo2 = newNodePtr + 1;
		node.left = newNodePtr++, node.right = newNodePtr++;
		if (!left.isLeaf()) stack[stackPtr++] = todo1;
		if (!right.isLeaf()) stack[stackPtr++] = todo2;
		if (!stackPtr) break;
		nodeIdx = stack[--stackPtr];
	}
	usedNodes = newNodePtr;
	AlignedFree( bvhNode );
	bvhNode = tmp;
}

void BVH_Verbose::Optimize( const uint32_t iterations, const bool extreme )
{
	// allocate array for sorting; size is upper-bound.
	SortItem* sortList = (SortItem*)AlignedAlloc( usedNodes * sizeof( SortItem ) );
	// optimize by reinserting subtrees with a high cost - Section 3.4 of the paper.
	for (uint32_t i = 0; i < iterations; i++)
	{
		// calculate combined cost for all nodes
		uint32_t interiorNodes = 0;
		for (uint32_t j = 2; j < usedNodes; j++)
		{
			const BVHNode& node = bvhNode[j];
			if (node.isLeaf()) continue;
			if (node.parent == 0) continue;
			if (bvhNode[node.parent].parent == 0) continue;
			const float A = node.SA(), AL = bvhNode[node.left].SA(), AR = bvhNode[node.right].SA();
			float Mmin = A / tinybvh_min( 1e-10f, tinybvh_min( AL, AR ) );
			float Msum = A / tinybvh_min( 1e-10f, 0.5f * (AL + AR) );
			float Mcomb = A * Msum * Mmin;
			sortList[interiorNodes].idx = j, sortList[interiorNodes++].cost = Mcomb;
		}
		// last couple of iterations we will process more nodes.
		const float portion = extreme ? (0.01f + (0.6f * (float)i) / (float)iterations) : 0.01f;
		const int limit = (uint32_t)(portion * (float)interiorNodes);
		const int step = tinybvh_max( 1, (int)(portion / 0.02f) );
		// sort list - partial quick sort.
		struct Task { uint32_t first, last; } stack[64];
		int pivot, first = 0, last = (int)interiorNodes - 1, stackPtr = 0;
		while (1)
		{
			if (first >= last)
			{
				if (stackPtr == 0) break; else first = stack[--stackPtr].first, last = stack[stackPtr].last;
				continue;
			}
			pivot = first;
			SortItem t, e = sortList[first];
			for (int j = first + 1; j <= last; j++) if (sortList[j].cost > e.cost)
				t = sortList[j], sortList[j] = sortList[++pivot], sortList[pivot] = t;
			t = sortList[pivot], sortList[pivot] = sortList[first], sortList[first] = t;
			if (pivot < limit) stack[stackPtr].first = pivot + 1, stack[stackPtr++].last = last;
			last = pivot - 1;
		}
		// reinsert selected nodes
		BVHNode bckp[5];
		for (int j = 0; j < limit; j += step)
		{
			// prepare change
			const uint32_t Nid = sortList[j].idx;
			BVHNode& N = bvhNode[Nid];
			if (N.parent == 0) continue;
			const uint32_t Pid = N.parent;
			BVHNode& P = bvhNode[Pid];
			if (P.parent == 0) continue;
			const uint32_t X1 = P.parent, X2 = (P.left == Nid ? P.right : P.left);
			// compute SAH before change
			float sahBefore = SAHCostUp( Nid );
			// execute change
			bckp[0] = bvhNode[X1];
			if (bvhNode[X1].left == Pid) bvhNode[X1].left = X2;
			else /* verbose[X1].right == Pid */ bvhNode[X1].right = X2;
			const uint32_t p2 = bvhNode[X2].parent;
			bvhNode[X2].parent = X1;
			const uint32_t Lid = N.left, Rid = N.right;
			RefitUp( X2 );
			// ReinsertNode( L, Nid ); ReinsertNode( R, Pid );
			const uint32_t Xbest1 = FindBestNewPosition( Lid ), XA = bvhNode[Xbest1].parent;
			sahBefore += SAHCostUp( Xbest1 );
			bckp[1] = bvhNode[Nid];
			N.left = Xbest1, N.right = Lid, N.parent = XA;
			bckp[2] = bvhNode[XA];
			if (bvhNode[XA].left == Xbest1) bvhNode[XA].left = Nid; else bvhNode[XA].right = Nid;
			const uint32_t p3 = bvhNode[Xbest1].parent, p4 = bvhNode[Lid].parent;
			bvhNode[Xbest1].parent = Nid, bvhNode[Lid].parent = Nid;
			RefitUp( Nid );
			const uint32_t Xbest2 = FindBestNewPosition( Rid ), XB = bvhNode[Xbest2].parent;
			sahBefore += SAHCostUp( Xbest2 );
			bckp[3] = bvhNode[Pid];
			P.left = Xbest2, P.right = Rid, P.parent = XB;
			bckp[4] = bvhNode[XB];
			if (bvhNode[XB].left == Xbest2) bvhNode[XB].left = Pid; else bvhNode[XB].right = Pid;
			const uint32_t p1 = bvhNode[Xbest2].parent, p0 = bvhNode[Rid].parent;
			bvhNode[Xbest2].parent = Pid, bvhNode[Rid].parent = Pid;
			RefitUp( Pid );
			// compute SAH after change
			float sahAfter = SAHCostUp( X1 ) + SAHCostUp( Nid ) + SAHCostUp( Pid );
			if (sahAfter <= sahBefore) continue;
			// undo change, mind the order.
			bvhNode[Rid].parent = p0, bvhNode[Xbest2].parent = p1, bvhNode[XB] = bckp[4];
			bvhNode[Pid] = bckp[3], bvhNode[Lid].parent = p4, bvhNode[Xbest1].parent = p3;
			bvhNode[XA] = bckp[2], bvhNode[Nid] = bckp[1], bvhNode[X2].parent = p2, bvhNode[X1] = bckp[0];
			RefitUp( XB );
			RefitUp( XA );
			RefitUp( Nid );
		}
		Refit( 0, true );
	}
	AlignedFree( sortList );
}

// Single-primitive leafs: Prepare the BVH for optimization. While it is not strictly
// necessary to have a single primitive per leaf, it will yield a slightly better
// optimized BVH. The leafs of the optimized BVH should be collapsed ('MergeLeafs')
// to obtain the final tree.
void BVH_Verbose::SplitLeafs( const uint32_t maxPrims )
{
	uint32_t nodeIdx = 0, stack[64], stackPtr = 0;
	while (1)
	{
		BVHNode& node = bvhNode[nodeIdx];
		if (!node.isLeaf()) nodeIdx = node.left, stack[stackPtr++] = node.right; else
		{
			// split this leaf
			if (node.triCount > maxPrims)
			{
				const uint32_t newIdx1 = usedNodes++, newIdx2 = usedNodes++;
				BVHNode& new1 = bvhNode[newIdx1], & new2 = bvhNode[newIdx2];
				new1.firstTri = node.firstTri, new1.triCount = node.triCount / 2;
				new1.parent = new2.parent = nodeIdx, new1.left = new1.right = 0;
				new2.firstTri = node.firstTri + new1.triCount;
				new2.triCount = node.triCount - new1.triCount, new2.left = new2.right = 0;
				node.left = newIdx1, node.right = newIdx2, node.triCount = 0;
				new1.aabbMin = new2.aabbMin = bvhvec3( BVH_FAR ), new1.aabbMax = new2.aabbMax = bvhvec3( -BVH_FAR );
				for (uint32_t fi, i = 0; i < new1.triCount; i++)
					fi = primIdx[new1.firstTri + i],
					new1.aabbMin = tinybvh_min( new1.aabbMin, fragment[fi].bmin ),
					new1.aabbMax = tinybvh_max( new1.aabbMax, fragment[fi].bmax );
				for (uint32_t fi, i = 0; i < new2.triCount; i++)
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
void BVH_Verbose::MergeLeafs()
{
	// allocate some working space
	uint32_t* subtreeTriCount = (uint32_t*)AlignedAlloc( usedNodes * 4 );
	uint32_t* newIdx = (uint32_t*)AlignedAlloc( idxCount * 4 );
	memset( subtreeTriCount, 0, usedNodes * 4 );
	CountSubtreeTris( 0, subtreeTriCount );
	uint32_t stack[64], stackPtr = 0, nodeIdx = 0, newIdxPtr = 0;
	while (1)
	{
		BVHNode& node = bvhNode[nodeIdx];
		if (node.isLeaf())
		{
			uint32_t start = newIdxPtr;
			MergeSubtree( nodeIdx, newIdx, newIdxPtr );
			node.firstTri = start;
			// pop new task
			if (stackPtr == 0) break;
			nodeIdx = stack[--stackPtr];
		}
		else
		{
			const uint32_t leftCount = subtreeTriCount[node.left];
			const uint32_t rightCount = subtreeTriCount[node.right];
			const uint32_t mergedCount = leftCount + rightCount;
			// cost of unsplit
			float Cunsplit = SA( node.aabbMin, node.aabbMax ) * mergedCount * C_INT;
			// cost of leaving things as they are
			BVHNode& left = bvhNode[node.left];
			BVHNode& right = bvhNode[node.right];
			float Ckeepsplit = C_TRAV + C_INT * (left.SA() * leftCount + right.SA() * rightCount);
			if (Cunsplit <= Ckeepsplit)
			{
				// collapse the subtree
				uint32_t start = newIdxPtr;
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
	AlignedFree( subtreeTriCount );
	AlignedFree( primIdx );
	primIdx = newIdx, may_have_holes = true; // all over the place, in fact
}

// BVH_GPU implementation
// ----------------------------------------------------------------------------

BVH_GPU::~BVH_GPU()
{
	if (!ownBVH) bvh = BVH(); // clear out pointers we don't own.
	AlignedFree( bvhNode );
}

void BVH_GPU::Build( const bvhvec4* vertices, const uint32_t primCount )
{
	Build( bvhvec4slice( vertices, primCount * 3, sizeof( bvhvec4 ) ) );
}

void BVH_GPU::Build( const bvhvec4slice& vertices )
{
	bvh.context = context;
	bvh.BuildDefault( vertices );
	ConvertFrom( bvh, false );
}

void BVH_GPU::Build( const bvhvec4* vertices, const uint32_t* indices, const uint32_t prims )
{
	// build the BVH with a continuous array of bvhvec4 vertices, indexed by 'indices'.
	Build( bvhvec4slice{ vertices, prims * 3, sizeof( bvhvec4 ) }, indices, prims );
}

void BVH_GPU::Build( const bvhvec4slice& vertices, const uint32_t* indices, uint32_t prims )
{
	// build the BVH from vertices stored in a slice, indexed by 'indices'.
	bvh.context = context;
	bvh.BuildDefault( vertices, indices, prims );
	ConvertFrom( bvh, false );
}

void BVH_GPU::Build( BLASInstance* instances, const uint32_t instCount, BVHBase** blasses, const uint32_t blasCount )
{
	// build a TLAS based on the array of BLASInstance records.
	bvh.context = context;
	bvh.Build( instances, instCount, blasses, blasCount );
	ConvertFrom( bvh, false );
}

void BVH_GPU::BuildHQ( const bvhvec4* vertices, const uint32_t primCount )
{
	BuildHQ( bvhvec4slice( vertices, primCount * 3, sizeof( bvhvec4 ) ) );
}

void BVH_GPU::BuildHQ( const bvhvec4slice& vertices )
{
	bvh.BuildHQ( vertices );
	ConvertFrom( bvh, false );
}

void BVH_GPU::BuildHQ( const bvhvec4* vertices, const uint32_t* indices, const uint32_t prims )
{
	BuildHQ( bvhvec4slice{ vertices, prims * 3, sizeof( bvhvec4 ) }, indices, prims );
}

void BVH_GPU::BuildHQ( const bvhvec4slice& vertices, const uint32_t* indices, uint32_t prims )
{
	bvh.context = context;
	bvh.BuildHQ( vertices, indices, prims );
	ConvertFrom( bvh, false );
}

void BVH_GPU::Optimize( const uint32_t iterations, bool extreme )
{
	bvh.Optimize( iterations, extreme );
	ConvertFrom( bvh, false );
}

void BVH_GPU::ConvertFrom( const BVH& original, bool compact )
{
	// get a copy of the original bvh
	if (&original != &bvh) ownBVH = false; // bvh isn't ours; don't delete in destructor.
	bvh = original;
	// allocate space
	const uint32_t spaceNeeded = compact ? original.usedNodes : original.allocatedNodes;
	if (allocatedNodes < spaceNeeded)
	{
		AlignedFree( bvhNode );
		bvhNode = (BVHNode*)AlignedAlloc( sizeof( BVHNode ) * spaceNeeded );
		allocatedNodes = spaceNeeded;
	}
	memset( bvhNode, 0, sizeof( BVHNode ) * spaceNeeded );
	CopyBasePropertiesFrom( original );
	// recursively convert nodes
	uint32_t newNodePtr = 0, nodeIdx = 0, stack[128], stackPtr = 0;
	while (1)
	{
		const BVH::BVHNode& orig = original.bvhNode[nodeIdx];
		const uint32_t idx = newNodePtr++;
		if (orig.isLeaf())
		{
			this->bvhNode[idx].triCount = orig.triCount;
			this->bvhNode[idx].firstTri = orig.leftFirst;
			if (!stackPtr) break;
			nodeIdx = stack[--stackPtr];
			uint32_t newNodeParent = stack[--stackPtr];
			this->bvhNode[newNodeParent].right = newNodePtr;
		}
		else
		{
			const BVH::BVHNode& left = original.bvhNode[orig.leftFirst];
			const BVH::BVHNode& right = original.bvhNode[orig.leftFirst + 1];
			this->bvhNode[idx].lmin = left.aabbMin, this->bvhNode[idx].rmin = right.aabbMin;
			this->bvhNode[idx].lmax = left.aabbMax, this->bvhNode[idx].rmax = right.aabbMax;
			this->bvhNode[idx].left = newNodePtr; // right will be filled when popped
			stack[stackPtr++] = idx;
			stack[stackPtr++] = orig.leftFirst + 1;
			nodeIdx = orig.leftFirst;
		}
	}
	usedNodes = newNodePtr;
}

int32_t BVH_GPU::Intersect( Ray& ray ) const
{
	BVHNode* node = &bvhNode[0], * stack[64];
	const bvhvec4slice& verts = bvh.verts;
	const uint32_t* primIdx = bvh.primIdx;
	uint32_t stackPtr = 0, cost = 0;
	while (1)
	{
		cost += C_TRAV;
		if (node->isLeaf())
		{
			if (indexedEnabled && bvh.vertIdx != 0) for (uint32_t i = 0; i < node->triCount; i++, cost += C_INT)
				IntersectTriIndexed( ray, verts, bvh.vertIdx, primIdx[node->firstTri + i] );
			else if (customEnabled && bvh.customIntersect != 0) for (uint32_t i = 0; i < node->triCount; i++, cost += C_INT)
				(*bvh.customIntersect)(ray, primIdx[node->firstTri + i]);
			else for (uint32_t i = 0; i < node->triCount; i++, cost += C_INT)
				IntersectTri( ray, verts, primIdx[node->firstTri + i] );
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		const bvhvec3 lmin = node->lmin - ray.O, lmax = node->lmax - ray.O;
		const bvhvec3 rmin = node->rmin - ray.O, rmax = node->rmax - ray.O;
		float dist1 = BVH_FAR, dist2 = BVH_FAR;
		const bvhvec3 t1a = lmin * ray.rD, t2a = lmax * ray.rD;
		const bvhvec3 t1b = rmin * ray.rD, t2b = rmax * ray.rD;
		const float tmina = tinybvh_max( tinybvh_max( tinybvh_min( t1a.x, t2a.x ), tinybvh_min( t1a.y, t2a.y ) ), tinybvh_min( t1a.z, t2a.z ) );
		const float tmaxa = tinybvh_min( tinybvh_min( tinybvh_max( t1a.x, t2a.x ), tinybvh_max( t1a.y, t2a.y ) ), tinybvh_max( t1a.z, t2a.z ) );
		const float tminb = tinybvh_max( tinybvh_max( tinybvh_min( t1b.x, t2b.x ), tinybvh_min( t1b.y, t2b.y ) ), tinybvh_min( t1b.z, t2b.z ) );
		const float tmaxb = tinybvh_min( tinybvh_min( tinybvh_max( t1b.x, t2b.x ), tinybvh_max( t1b.y, t2b.y ) ), tinybvh_max( t1b.z, t2b.z ) );
		if (tmaxa >= tmina && tmina < ray.hit.t && tmaxa >= 0) dist1 = tmina;
		if (tmaxb >= tminb && tminb < ray.hit.t && tmaxb >= 0) dist2 = tminb;
		uint32_t lidx = node->left, ridx = node->right;
		if (dist1 > dist2)
		{
			float t = dist1; dist1 = dist2; dist2 = t;
			uint32_t i = lidx; lidx = ridx; ridx = i;
		}
		if (dist1 == BVH_FAR)
		{
			if (stackPtr == 0) break; else node = stack[--stackPtr];
		}
		else
		{
			node = bvhNode + lidx;
			if (dist2 != BVH_FAR) stack[stackPtr++] = bvhNode + ridx;
		}
	}
	return cost;
}

// BVH_SoA implementation
// ----------------------------------------------------------------------------

BVH_SoA::~BVH_SoA()
{
	if (!ownBVH) bvh = BVH(); // clear out pointers we don't own.
	AlignedFree( bvhNode );
}

void BVH_SoA::Build( const bvhvec4* vertices, const uint32_t primCount )
{
	Build( bvhvec4slice( vertices, primCount * 3, sizeof( bvhvec4 ) ) );
}

void BVH_SoA::Build( const bvhvec4slice& vertices )
{
	bvh.context = context; // properly propagate context to fix issue #66.
	bvh.BuildDefault( vertices );
	ConvertFrom( bvh, false );
}

void BVH_SoA::Build( const bvhvec4* vertices, const uint32_t* indices, const uint32_t prims )
{
	// build the BVH with a continuous array of bvhvec4 vertices, indexed by 'indices'.
	Build( bvhvec4slice{ vertices, prims * 3, sizeof( bvhvec4 ) }, indices, prims );
}

void BVH_SoA::Build( const bvhvec4slice& vertices, const uint32_t* indices, uint32_t prims )
{
	// build the BVH from vertices stored in a slice, indexed by 'indices'.
	bvh.context = context;
	bvh.BuildDefault( vertices, indices, prims );
	ConvertFrom( bvh, false );
}

void BVH_SoA::BuildHQ( const bvhvec4* vertices, const uint32_t primCount )
{
	BuildHQ( bvhvec4slice( vertices, primCount * 3, sizeof( bvhvec4 ) ) );
}

void BVH_SoA::BuildHQ( const bvhvec4slice& vertices )
{
	bvh.context = context; // properly propagate context to fix issue #66.
	bvh.BuildHQ( vertices );
	ConvertFrom( bvh, false );
}

void BVH_SoA::BuildHQ( const bvhvec4* vertices, const uint32_t* indices, const uint32_t prims )
{
	BuildHQ( bvhvec4slice{ vertices, prims * 3, sizeof( bvhvec4 ) }, indices, prims );
}

void BVH_SoA::BuildHQ( const bvhvec4slice& vertices, const uint32_t* indices, uint32_t prims )
{
	bvh.context = context;
	bvh.BuildHQ( vertices, indices, prims );
	ConvertFrom( bvh, false );
}

void BVH_SoA::Optimize( const uint32_t iterations, bool extreme )
{
	bvh.Optimize( iterations, extreme );
	ConvertFrom( bvh, false );
}

void BVH_SoA::Save( const char* fileName )
{
	bvh.Save( fileName );
}

bool BVH_SoA::Load( const char* fileName, const bvhvec4* vertices, const uint32_t primCount )
{
	return Load( fileName, bvhvec4slice{ vertices, primCount * 3, sizeof( bvhvec4 ) } );
}

bool BVH_SoA::Load( const char* fileName, const bvhvec4* vertices, const uint32_t* indices, const uint32_t primCount )
{
	return Load( fileName, bvhvec4slice{ vertices, primCount * 3, sizeof( bvhvec4 ) }, indices, primCount );
}

bool BVH_SoA::Load( const char* fileName, const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t primCount )
{
	if (!bvh.Load( fileName, vertices, indices, primCount )) return false;
	ConvertFrom( bvh, false );
	return true;
}

void BVH_SoA::ConvertFrom( const BVH& original, bool compact )
{
	// get a copy of the original bvh
	if (&original != &bvh) ownBVH = false; // bvh isn't ours; don't delete in destructor.
	bvh = original;
	// allocate space
	const uint32_t spaceNeeded = compact ? bvh.usedNodes : bvh.allocatedNodes;
	if (allocatedNodes < spaceNeeded)
	{
		AlignedFree( bvhNode );
		bvhNode = (BVHNode*)AlignedAlloc( sizeof( BVHNode ) * spaceNeeded );
		allocatedNodes = spaceNeeded;
	}
	memset( bvhNode, 0, sizeof( BVHNode ) * spaceNeeded );
	CopyBasePropertiesFrom( bvh );
	// recursively convert nodes
	uint32_t newAlt2Node = 0, nodeIdx = 0, stack[128], stackPtr = 0;
	while (1)
	{
		const BVH::BVHNode& node = bvh.bvhNode[nodeIdx];
		const uint32_t idx = newAlt2Node++;
		if (node.isLeaf())
		{
			bvhNode[idx].triCount = node.triCount;
			bvhNode[idx].firstTri = node.leftFirst;
			if (!stackPtr) break;
			nodeIdx = stack[--stackPtr];
			uint32_t newNodeParent = stack[--stackPtr];
			bvhNode[newNodeParent].right = newAlt2Node;
		}
		else
		{
			const BVH::BVHNode& left = bvh.bvhNode[node.leftFirst];
			const BVH::BVHNode& right = bvh.bvhNode[node.leftFirst + 1];
			// This BVH layout requires BVH_USEAVX/BVH_USENEON for traversal, but at least we
			// can convert to it without SSE/AVX/NEON support.
			bvhNode[idx].xxxx = SIMD_SETRVEC( left.aabbMin.x, left.aabbMax.x, right.aabbMin.x, right.aabbMax.x );
			bvhNode[idx].yyyy = SIMD_SETRVEC( left.aabbMin.y, left.aabbMax.y, right.aabbMin.y, right.aabbMax.y );
			bvhNode[idx].zzzz = SIMD_SETRVEC( left.aabbMin.z, left.aabbMax.z, right.aabbMin.z, right.aabbMax.z );
			bvhNode[idx].left = newAlt2Node; // right will be filled when popped
			stack[stackPtr++] = idx;
			stack[stackPtr++] = node.leftFirst + 1;
			nodeIdx = node.leftFirst;
		}
	}
	usedNodes = newAlt2Node;
}

// BVH_SoA::Intersect can be found in the BVH_USEAVX section later in this file.

// Generic (templated) MBVH implementation
// ----------------------------------------------------------------------------

template<int M> MBVH<M>::~MBVH()
{
	if (!ownBVH) bvh = BVH(); // clear out pointers we don't own.
	AlignedFree( mbvhNode );
}

template<int M> void MBVH<M>::Build( const bvhvec4* vertices, const uint32_t primCount )
{
	Build( bvhvec4slice( vertices, primCount * 3, sizeof( bvhvec4 ) ) );
}

template<int M> void MBVH<M>::Build( const bvhvec4slice& vertices )
{
	bvh.context = context; // properly propagate context to fix issue #66.
	bvh.BuildDefault( vertices );
	ConvertFrom( bvh, false );
}

template<int M> void MBVH<M>::Build( const bvhvec4* vertices, const uint32_t* indices, const uint32_t prims )
{
	// build the BVH with a continuous array of bvhvec4 vertices, indexed by 'indices'.
	Build( bvhvec4slice{ vertices, prims * 3, sizeof( bvhvec4 ) }, indices, prims );
}

template<int M> void MBVH<M>::Build( const bvhvec4slice& vertices, const uint32_t* indices, uint32_t prims )
{
	// build the BVH from vertices stored in a slice, indexed by 'indices'.
	bvh.context = context;
	bvh.BuildDefault( vertices, indices, prims );
	ConvertFrom( bvh, true );
}

template<int M> void MBVH<M>::BuildHQ( const bvhvec4* vertices, const uint32_t primCount )
{
	BuildHQ( bvhvec4slice( vertices, primCount * 3, sizeof( bvhvec4 ) ) );
}

template<int M> void MBVH<M>::BuildHQ( const bvhvec4slice& vertices )
{
	bvh.context = context;
	bvh.BuildHQ( vertices );
	ConvertFrom( bvh, true );
}

template<int M> void MBVH<M>::BuildHQ( const bvhvec4* vertices, const uint32_t* indices, const uint32_t prims )
{
	Build( bvhvec4slice{ vertices, prims * 3, sizeof( bvhvec4 ) }, indices, prims );
}

template<int M> void MBVH<M>::BuildHQ( const bvhvec4slice& vertices, const uint32_t* indices, uint32_t prims )
{
	bvh.context = context;
	bvh.BuildHQ( vertices, indices, prims );
	ConvertFrom( bvh, true );
}

template<int M> void MBVH<M>::Optimize( const uint32_t iterations, bool extreme )
{
	bvh.Optimize( iterations, extreme );
	ConvertFrom( bvh, true );
}

template<int M> void MBVH<M>::Refit( const uint32_t nodeIdx )
{
	MBVHNode& node = mbvhNode[nodeIdx];
	if (node.isLeaf())
	{
		bvhvec3 aabbMin( BVH_FAR ), aabbMax( -BVH_FAR );
		if (bvh.vertIdx)
		{
			for (uint32_t first = node.firstTri, j = 0; j < node.triCount; j++)
			{
				const uint32_t vidx = bvh.primIdx[first + j] * 3;
				const uint32_t i0 = bvh.vertIdx[vidx], i1 = bvh.vertIdx[vidx + 1], i2 = bvh.vertIdx[vidx + 2];
				const bvhvec3 v0 = bvh.verts[i0], v1 = bvh.verts[i1], v2 = bvh.verts[i2];
				aabbMin = tinybvh_min( aabbMin, tinybvh_min( tinybvh_min( v0, v1 ), v2 ) );
				aabbMax = tinybvh_max( aabbMax, tinybvh_max( tinybvh_max( v0, v1 ), v2 ) );
			}
		}
		else
		{
			for (uint32_t first = node.firstTri, j = 0; j < node.triCount; j++)
			{
				const uint32_t vidx = bvh.primIdx[first + j] * 3;
				const bvhvec3 v0 = bvh.verts[vidx], v1 = bvh.verts[vidx + 1], v2 = bvh.verts[vidx + 2];
				aabbMin = tinybvh_min( aabbMin, tinybvh_min( tinybvh_min( v0, v1 ), v2 ) );
				aabbMax = tinybvh_max( aabbMax, tinybvh_max( tinybvh_max( v0, v1 ), v2 ) );
			}
		}
		node.aabbMin = aabbMin, node.aabbMax = aabbMax;
	}
	else
	{
		for (unsigned i = 0; i < node.childCount; i++) Refit( node.child[i] );
		MBVHNode& firstChild = mbvhNode[node.child[0]];
		bvhvec3 bmin = firstChild.aabbMin, bmax = firstChild.aabbMax;
		for (unsigned i = 1; i < node.childCount; i++)
		{
			MBVHNode& child = mbvhNode[node.child[i]];
			bmin = tinybvh_min( bmin, child.aabbMin );
			bmax = tinybvh_max( bmax, child.aabbMax );
		}
	}
	if (nodeIdx == 0) aabbMin = node.aabbMin, aabbMax = node.aabbMax;
}

template<int M> float MBVH<M>::SAHCost( const uint32_t nodeIdx ) const
{
	// Determine the SAH cost of the tree. This provides an indication
	// of the quality of the BVH: Lower is better.
	const MBVHNode& n = mbvhNode[nodeIdx];
	const float sa = BVH::SA( n.aabbMin, n.aabbMax );
	if (n.isLeaf()) return C_INT * sa * n.triCount;
	float cost = C_TRAV * sa;
	for (unsigned i = 0; i < M; i++) if (n.child[i] != 0) cost += SAHCost( n.child[i] );
	return nodeIdx == 0 ? (cost / sa) : cost;
}

template<int M> void MBVH<M>::ConvertFrom( const BVH& original, bool compact )
{
	// get a copy of the original bvh
	if (&original != &bvh) ownBVH = false; // bvh isn't ours; don't delete in destructor.
	bvh = original;
	// allocate space
	uint32_t spaceNeeded = compact ? original.usedNodes : original.allocatedNodes;
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
	for (uint32_t i = 0; i < original.usedNodes; i++) if (i != 1)
	{
		BVH::BVHNode& orig = original.bvhNode[i];
		MBVHNode& node = this->mbvhNode[i];
		node.aabbMin = orig.aabbMin, node.aabbMax = orig.aabbMax;
		if (orig.isLeaf()) node.triCount = orig.triCount, node.firstTri = orig.leftFirst;
		else node.child[0] = orig.leftFirst, node.child[1] = orig.leftFirst + 1, node.childCount = 2;
	}
	// collapse
	uint32_t stack[128], stackPtr = 0, nodeIdx = 0; // i.e., root node
	while (1)
	{
		MBVHNode& node = this->mbvhNode[nodeIdx];
		while (node.childCount < M)
		{
			int32_t bestChild = -1;
			float bestChildSA = 0;
			for (uint32_t i = 0; i < node.childCount; i++)
			{
				// see if we can adopt child i
				const MBVHNode& child = this->mbvhNode[node.child[i]];
				if (!child.isLeaf() && node.childCount - 1 + child.childCount <= M)
				{
					const float childSA = SA( child.aabbMin, child.aabbMax );
					if (childSA > bestChildSA) bestChild = i, bestChildSA = childSA;
				}
			}
			if (bestChild == -1) break; // could not adopt
			const MBVHNode& child = this->mbvhNode[node.child[bestChild]];
			node.child[bestChild] = child.child[0];
			for (uint32_t i = 1; i < child.childCount; i++)
				node.child[node.childCount++] = child.child[i];
		}
		// we're done with the node; proceed with the children.
		for (uint32_t i = 0; i < node.childCount; i++)
		{
			const uint32_t childIdx = node.child[i];
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

// SplitBVH8Leaf: CWBVH requires that a leaf has no more than 3 primitives,
// but regular BVH construction does not guarantee this. So, here we split
// busy leafs recursively in multiple leaves, until the requirement is met.
template<int M> void MBVH<M>::SplitBVHLeaf( const uint32_t nodeIdx, const uint32_t maxPrims )
{
	const uint32_t* primIdx = bvh.primIdx;
	const Fragment* fragment = bvh.fragment;
	MBVHNode& node = mbvhNode[nodeIdx];
	if (node.triCount <= maxPrims) return; // also catches interior nodes
	// place all primitives in a new node and make this the first child of 'node'
	MBVHNode& firstChild = mbvhNode[node.child[0] = usedNodes++];
	firstChild.triCount = node.triCount;
	firstChild.firstTri = node.firstTri;
	uint32_t nextChild = 1;
	// share with new sibling nodes
	while (firstChild.triCount > maxPrims && nextChild < M)
	{
		MBVHNode& child = mbvhNode[node.child[nextChild] = usedNodes++];
		firstChild.triCount -= maxPrims, child.triCount = maxPrims;
		child.firstTri = firstChild.firstTri + firstChild.triCount;
		nextChild++;
	}
	for (uint32_t i = 0; i < nextChild; i++)
	{
		MBVHNode& child = mbvhNode[node.child[i]];
		if (!refittable) child.aabbMin = node.aabbMin, child.aabbMax = node.aabbMax; else
		{
			// TODO: why is this producing wrong aabbs for SBVH?
			child.aabbMin = bvhvec3( BVH_FAR ), child.aabbMax = bvhvec3( -BVH_FAR );
			for (uint32_t fi, j = 0; j < child.triCount; j++) fi = primIdx[child.firstTri + j],
				child.aabbMin = tinybvh_min( child.aabbMin, fragment[fi].bmin ),
				child.aabbMax = tinybvh_max( child.aabbMax, fragment[fi].bmax );
		}
	}
	node.triCount = 0;
	// recurse; should be rare
	if (firstChild.triCount > maxPrims) SplitBVHLeaf( node.child[0], maxPrims );
}

// BVH4_CPU implementation
// ----------------------------------------------------------------------------

BVH4_CPU::~BVH4_CPU()
{
	if (!ownBVH4) bvh4 = MBVH<4>(); // clear out pointers we don't own.
	AlignedFree( bvh4Node );
	AlignedFree( bvh4Tris );
}

void BVH4_CPU::Build( const bvhvec4* vertices, const uint32_t primCount )
{
	Build( bvhvec4slice( vertices, primCount * 3, sizeof( bvhvec4 ) ) );
}

void BVH4_CPU::Build( const bvhvec4slice& vertices )
{
	bvh4.context = context; // properly propagate context to fix issue #66.
	bvh4.Build( vertices );
	ConvertFrom( bvh4, true );
}

void BVH4_CPU::Build( const bvhvec4* vertices, const uint32_t* indices, const uint32_t prims )
{
	// build the BVH with a continuous array of bvhvec4 vertices, indexed by 'indices'.
	Build( bvhvec4slice{ vertices, prims * 3, sizeof( bvhvec4 ) }, indices, prims );
}

void BVH4_CPU::Build( const bvhvec4slice& vertices, const uint32_t* indices, uint32_t prims )
{
	// build the BVH from vertices stored in a slice, indexed by 'indices'.
	bvh4.context = context;
	bvh4.Build( vertices, indices, prims );
	ConvertFrom( bvh4, true );
}

void BVH4_CPU::BuildHQ( const bvhvec4* vertices, const uint32_t primCount )
{
	BuildHQ( bvhvec4slice( vertices, primCount * 3, sizeof( bvhvec4 ) ) );
}

void BVH4_CPU::BuildHQ( const bvhvec4slice& vertices )
{
	bvh4.context = context;
	bvh4.BuildHQ( vertices );
	ConvertFrom( bvh4, true );
}

void BVH4_CPU::BuildHQ( const bvhvec4* vertices, const uint32_t* indices, const uint32_t prims )
{
	Build( bvhvec4slice{ vertices, prims * 3, sizeof( bvhvec4 ) }, indices, prims );
}

void BVH4_CPU::BuildHQ( const bvhvec4slice& vertices, const uint32_t* indices, uint32_t prims )
{
	bvh4.context = context;
	bvh4.BuildHQ( vertices, indices, prims );
	ConvertFrom( bvh4, true );
}

void BVH4_CPU::Optimize( const uint32_t iterations, bool extreme )
{
	bvh4.Optimize( iterations, extreme );
	ConvertFrom( bvh4, true );
}

float BVH4_CPU::SAHCost( const uint32_t nodeIdx ) const
{
	return bvh4.SAHCost( nodeIdx );
}

void BVH4_CPU::Save( const char* fileName )
{
	// saving is easy, it's the loadingn that will be complex.
	std::fstream s{ fileName, s.binary | s.out };
	uint32_t header = TINY_BVH_VERSION_SUB + (TINY_BVH_VERSION_MINOR << 8) + (TINY_BVH_VERSION_MAJOR << 16) + (layout << 24);
	s.write( (char*)&header, sizeof( uint32_t ) );
	s.write( (char*)&triCount, sizeof( uint32_t ) );
	s.write( (char*)this, sizeof( BVH4_CPU ) );
	s.write( (char*)bvh4Node, usedNodes * sizeof( BVHNode ) );
	s.write( (char*)bvh4Tris, idxCount * 4 * sizeof( bvhvec4 ) );
}

bool BVH4_CPU::Load( const char* fileName, const uint32_t primCount )
{
	// open file and check contents
	std::fstream s{ fileName, s.binary | s.in };
	if (!s) return false;
	BVHContext tmp = context;
	uint32_t header, fileTriCount;
	s.read( (char*)&header, sizeof( uint32_t ) );
	if (((header >> 8) & 255) != TINY_BVH_VERSION_MINOR || ((header >> 16) & 255) != TINY_BVH_VERSION_MAJOR ||
		(header & 255) != TINY_BVH_VERSION_SUB || (header >> 24) != layout) return false;
	s.read( (char*)&fileTriCount, sizeof( uint32_t ) );
	if (fileTriCount != primCount) return false;
	// all checks passed; safe to overwrite *this
	s.read( (char*)this, sizeof( BVH4_CPU ) );
	context = tmp; // can't load context; function pointers will differ.
	bvh4 = MBVH<4>(); // this won't be valid; avoid deleting it's data later.
	bvh4Node = (BVHNode*)AlignedAlloc( usedNodes * sizeof( BVHNode ) );
	bvh4Tris = (bvhvec4*)AlignedAlloc( primCount * 4 * sizeof( bvhvec4 ) );
	s.read( (char*)bvh4Node, usedNodes * sizeof( BVHNode ) );
	s.read( (char*)bvh4Tris, primCount * 4 * sizeof( bvhvec4 ) );
	allocatedNodes = usedNodes, ownBVH4 = false;
	// all ok.
	return true;
}

void BVH4_CPU::ConvertFrom( const MBVH<4>& original, bool compact )
{
	// get a copy of the original bvh4
	if (&original != &bvh4) ownBVH4 = false; // bvh isn't ours; don't delete in destructor.
	bvh4 = original;
	// Convert a 4-wide BVH to a format suitable for CPU traversal.
	// See Faster Incoherent Ray Traversal Using 8-Wide AVX InstructionsLayout,
	// Atilla T. Áfra, 2013.
	uint32_t spaceNeeded = compact ? bvh4.usedNodes : bvh4.allocatedNodes;
	if (allocatedNodes < spaceNeeded)
	{
		AlignedFree( bvh4Node );
		AlignedFree( bvh4Tris );
		bvh4Node = (BVHNode*)AlignedAlloc( spaceNeeded * sizeof( BVHNode ) );
		bvh4Tris = (bvhvec4*)AlignedAlloc( bvh4.idxCount * 4 * sizeof( bvhvec4 ) );
		allocatedNodes = spaceNeeded;
	}
	memset( bvh4Node, 0, spaceNeeded * sizeof( BVHNode ) );
	CopyBasePropertiesFrom( bvh4 );
	// start conversion
	uint32_t newAlt4Ptr = 0, nodeIdx = 0, stack[128], stackPtr = 0;
	while (1)
	{
		const MBVH<4>::MBVHNode& orig = bvh4.mbvhNode[nodeIdx];
		BVHNode& newNode = bvh4Node[newAlt4Ptr++];
		int32_t cidx = 0;
		for (int32_t i = 0; i < 4; i++) if (orig.child[i])
		{
			const MBVH<4>::MBVHNode& child = bvh4.mbvhNode[orig.child[i]];
			((float*)&newNode.xmin4)[cidx] = child.aabbMin.x;
			((float*)&newNode.ymin4)[cidx] = child.aabbMin.y;
			((float*)&newNode.zmin4)[cidx] = child.aabbMin.z;
			((float*)&newNode.xmax4)[cidx] = child.aabbMax.x;
			((float*)&newNode.ymax4)[cidx] = child.aabbMax.y;
			((float*)&newNode.zmax4)[cidx] = child.aabbMax.z;
			if (child.isLeaf())
				newNode.childFirst[cidx] = child.firstTri,
				newNode.triCount[cidx] = child.triCount;
			else
				stack[stackPtr++] = (uint32_t)((uint32_t*)&newNode.childFirst[cidx] - (uint32_t*)bvh4Node),
				stack[stackPtr++] = orig.child[i];
			cidx++;
		}
		for (; cidx < 4; cidx++)
		{
			((float*)&newNode.xmin4)[cidx] = 1e30f, ((float*)&newNode.xmax4)[cidx] = 1.00001e30f;
			((float*)&newNode.ymin4)[cidx] = 1e30f, ((float*)&newNode.ymax4)[cidx] = 1.00001e30f;
			((float*)&newNode.zmin4)[cidx] = 1e30f, ((float*)&newNode.zmax4)[cidx] = 1.00001e30f;
		}
		// pop next task
		if (!stackPtr) break;
		nodeIdx = stack[--stackPtr];
		uint32_t offset = stack[--stackPtr];
		((uint32_t*)bvh4Node)[offset] = newAlt4Ptr;
	}
	// Convert index list: store primitives 'by value'.
	// This also allows us to compact and reorder them for best performance.
	stackPtr = 0, nodeIdx = 0;
	uint32_t triPtr = 0;
	while (1)
	{
		BVHNode& node = bvh4Node[nodeIdx];
		for (int32_t i = 0; i < 4; i++) if (node.triCount[i] + node.childFirst[i] > 0)
		{
			if (!node.triCount[i]) stack[stackPtr++] = node.childFirst[i]; else
			{
				uint32_t first = node.childFirst[i];
				uint32_t count = node.triCount[i];
				node.childFirst[i] = triPtr;
				// assign vertex data
				for (uint32_t j = 0; j < count; j++)
				{
					const uint32_t fi = bvh4.bvh.primIdx[first + j];
					uint32_t ti0, ti1, ti2;
					if (bvh4.bvh.vertIdx)
						ti0 = bvh4.bvh.vertIdx[fi * 3],
						ti1 = bvh4.bvh.vertIdx[fi * 3 + 1],
						ti2 = bvh4.bvh.vertIdx[fi * 3 + 2];
					else
						ti0 = fi * 3, ti1 = fi * 3 + 1, ti2 = fi * 3 + 2;
					PrecomputeTriangle( bvh4.bvh.verts, ti0, ti1, ti2, (float*)&bvh4Tris[triPtr] );
					bvh4Tris[triPtr + 3] = bvhvec4( 0, 0, 0, *(float*)&fi );
					triPtr += 4;
				}
			}
		}
		if (!stackPtr) break;
		nodeIdx = stack[--stackPtr];
	}
	usedNodes = newAlt4Ptr;
}

// BVH4_GPU implementation
// ----------------------------------------------------------------------------

BVH4_GPU::~BVH4_GPU()
{
	if (!ownBVH4) bvh4 = MBVH<4>(); // clear out pointers we don't own.
	AlignedFree( bvh4Data );
}

void BVH4_GPU::Build( const bvhvec4* vertices, const uint32_t primCount )
{
	Build( bvhvec4slice( vertices, primCount * 3, sizeof( bvhvec4 ) ) );
}

void BVH4_GPU::Build( const bvhvec4slice& vertices )
{
	bvh4.context = context;
	bvh4.Build( vertices );
	ConvertFrom( bvh4, true );
}

void BVH4_GPU::Build( const bvhvec4* vertices, const uint32_t* indices, const uint32_t prims )
{
	// build the BVH with a continuous array of bvhvec4 vertices, indexed by 'indices'.
	Build( bvhvec4slice{ vertices, prims * 3, sizeof( bvhvec4 ) }, indices, prims );
}

void BVH4_GPU::Build( const bvhvec4slice& vertices, const uint32_t* indices, uint32_t prims )
{
	// build the BVH from vertices stored in a slice, indexed by 'indices'.
	bvh4.context = context;
	bvh4.Build( vertices, indices, prims );
	ConvertFrom( bvh4, true );
}

void BVH4_GPU::BuildHQ( const bvhvec4* vertices, const uint32_t primCount )
{
	BuildHQ( bvhvec4slice( vertices, primCount * 3, sizeof( bvhvec4 ) ) );
}

void BVH4_GPU::BuildHQ( const bvhvec4slice& vertices )
{
	bvh4.context = context;
	bvh4.BuildHQ( vertices );
	ConvertFrom( bvh4, true );
}

void BVH4_GPU::BuildHQ( const bvhvec4* vertices, const uint32_t* indices, const uint32_t prims )
{
	Build( bvhvec4slice{ vertices, prims * 3, sizeof( bvhvec4 ) }, indices, prims );
}

void BVH4_GPU::BuildHQ( const bvhvec4slice& vertices, const uint32_t* indices, uint32_t prims )
{
	bvh4.context = context;
	bvh4.BuildHQ( vertices, indices, prims );
	ConvertFrom( bvh4, true );
}

void BVH4_GPU::Optimize( const uint32_t iterations, bool extreme )
{
	bvh4.Optimize( iterations, extreme );
	ConvertFrom( bvh4, true );
}

void BVH4_GPU::ConvertFrom( const MBVH<4>& original, bool compact )
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
	blocksNeeded += 6 * triCount; // this layout stores tris in the same buffer.
	if (allocatedBlocks < blocksNeeded)
	{
		AlignedFree( bvh4Data );
		bvh4Data = (bvhvec4*)AlignedAlloc( blocksNeeded * 16 );
		allocatedBlocks = blocksNeeded;
	}
	memset( bvh4Data, 0, 16 * blocksNeeded );
	CopyBasePropertiesFrom( bvh4 );
	// start conversion
	uint32_t nodeIdx = 0, newAlt4Ptr = 0, stack[128], stackPtr = 0, retValPos = 0;
	while (1)
	{
		const MBVH<4>::MBVHNode& orig = bvh4.mbvhNode[nodeIdx];
		// convert BVH4 node - must be an interior node.
		assert( !orig.isLeaf() );
		bvhvec4* nodeBase = bvh4Data + newAlt4Ptr;
		uint32_t baseAlt4Ptr = newAlt4Ptr;
		newAlt4Ptr += 4;
		nodeBase[0] = bvhvec4( orig.aabbMin, 0 );
		nodeBase[1] = bvhvec4( (orig.aabbMax - orig.aabbMin) * (1.0f / 255.0f), 0 );
		MBVH<4>::MBVHNode* childNode[4] = {
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
				PrecomputeTriangle( verts, ti0, ti1, ti2, (float*)&bvh4Alt[newAlt4Ptr] );
				bvh4Alt[newAlt4Ptr + 3] = bvhvec4( 0, 0, 0, *(float*)&t );
				newAlt4Ptr += 4;
			#else
				bvhvec4 v0 = bvh4.bvh.verts[ti0];
				bvh4Data[newAlt4Ptr + 1] = bvh4.bvh.verts[ti1] - v0;
				bvh4Data[newAlt4Ptr + 2] = bvh4.bvh.verts[ti2] - v0;
				v0.w = *(float*)&t; // as_float
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
				stack[stackPtr++] = (uint32_t)(((float*)&nodeBase[3] + i) - (float*)bvh4Data);
				stack[stackPtr++] = orig.child[i];
			}
		}
		// store child node bounds, quantized
		const bvhvec3 extent = orig.aabbMax - orig.aabbMin;
		bvhvec3 scale;
		scale.x = extent.x > 1e-10f ? (254.999f / extent.x) : 0;
		scale.y = extent.y > 1e-10f ? (254.999f / extent.y) : 0;
		scale.z = extent.z > 1e-10f ? (254.999f / extent.z) : 0;
		uint8_t* slot0 = (uint8_t*)&nodeBase[0] + 12;	// 4 chars
		uint8_t* slot1 = (uint8_t*)&nodeBase[1] + 12;	// 4 chars
		uint8_t* slot2 = (uint8_t*)&nodeBase[2];		// 16 chars
		if (orig.child[0])
		{
			const bvhvec3 relBMin = childNode[0]->aabbMin - orig.aabbMin, relBMax = childNode[0]->aabbMax - orig.aabbMin;
			slot0[0] = (uint8_t)floorf( relBMin.x * scale.x ), slot1[0] = (uint8_t)ceilf( relBMax.x * scale.x );
			slot2[0] = (uint8_t)floorf( relBMin.y * scale.y ), slot2[4] = (uint8_t)ceilf( relBMax.y * scale.y );
			slot2[8] = (uint8_t)floorf( relBMin.z * scale.z ), slot2[12] = (uint8_t)ceilf( relBMax.z * scale.z );
		}
		if (orig.child[1])
		{
			const bvhvec3 relBMin = childNode[1]->aabbMin - orig.aabbMin, relBMax = childNode[1]->aabbMax - orig.aabbMin;
			slot0[1] = (uint8_t)floorf( relBMin.x * scale.x ), slot1[1] = (uint8_t)ceilf( relBMax.x * scale.x );
			slot2[1] = (uint8_t)floorf( relBMin.y * scale.y ), slot2[5] = (uint8_t)ceilf( relBMax.y * scale.y );
			slot2[9] = (uint8_t)floorf( relBMin.z * scale.z ), slot2[13] = (uint8_t)ceilf( relBMax.z * scale.z );
		}
		if (orig.child[2])
		{
			const bvhvec3 relBMin = childNode[2]->aabbMin - orig.aabbMin, relBMax = childNode[2]->aabbMax - orig.aabbMin;
			slot0[2] = (uint8_t)floorf( relBMin.x * scale.x ), slot1[2] = (uint8_t)ceilf( relBMax.x * scale.x );
			slot2[2] = (uint8_t)floorf( relBMin.y * scale.y ), slot2[6] = (uint8_t)ceilf( relBMax.y * scale.y );
			slot2[10] = (uint8_t)floorf( relBMin.z * scale.z ), slot2[14] = (uint8_t)ceilf( relBMax.z * scale.z );
		}
		if (orig.child[3])
		{
			const bvhvec3 relBMin = childNode[3]->aabbMin - orig.aabbMin, relBMax = childNode[3]->aabbMax - orig.aabbMin;
			slot0[3] = (uint8_t)floorf( relBMin.x * scale.x ), slot1[3] = (uint8_t)ceilf( relBMax.x * scale.x );
			slot2[3] = (uint8_t)floorf( relBMin.y * scale.y ), slot2[7] = (uint8_t)ceilf( relBMax.y * scale.y );
			slot2[11] = (uint8_t)floorf( relBMin.z * scale.z ), slot2[15] = (uint8_t)ceilf( relBMax.z * scale.z );
		}
		// finalize node
		nodeBase[3] = bvhvec4(
			*(float*)&childInfo[0], *(float*)&childInfo[1],
			*(float*)&childInfo[2], *(float*)&childInfo[3]
		);
		// pop new work from the stack
		if (retValPos > 0) ((uint32_t*)bvh4Data)[retValPos] = baseAlt4Ptr;
		if (stackPtr == 0) break;
		nodeIdx = stack[--stackPtr];
		retValPos = stack[--stackPtr];
	}
	usedBlocks = newAlt4Ptr;
}

// IntersectAlt4Nodes. For testing the converted data only; not efficient.
// This code replicates how traversal on GPU happens.
#define SWAP(A,B,C,D) t=A,A=B,B=t,t2=C,C=D,D=t2;
struct uchar4 { uint8_t x, y, z, w; };
static uchar4 as_uchar4( const float v ) { union { float t; uchar4 t4; }; t = v; return t4; }
static uint32_t as_uint( const float v ) { return *(uint32_t*)&v; }
int32_t BVH4_GPU::Intersect( Ray& ray ) const
{
	// traverse a blas
	uint32_t offset = 0, stack[128], stackPtr = 0, t2 /* for SWAP macro */;
	uint32_t cost = 0;
	while (1)
	{
		cost += C_TRAV;
		// fetch the node
		const bvhvec4 data0 = bvh4Data[offset + 0], data1 = bvh4Data[offset + 1];
		const bvhvec4 data2 = bvh4Data[offset + 2], data3 = bvh4Data[offset + 3];
		// extract aabb
		const bvhvec3 bmin = data0, extent = data1; // pre-scaled by 1/255
		// reconstruct conservative child aabbs
		const uchar4 d0 = as_uchar4( data0.w ), d1 = as_uchar4( data1.w ), d2 = as_uchar4( data2.x );
		const uchar4 d3 = as_uchar4( data2.y ), d4 = as_uchar4( data2.z ), d5 = as_uchar4( data2.w );
		const bvhvec3 c0min = bmin + extent * bvhvec3( d0.x, d2.x, d4.x ), c0max = bmin + extent * bvhvec3( d1.x, d3.x, d5.x );
		const bvhvec3 c1min = bmin + extent * bvhvec3( d0.y, d2.y, d4.y ), c1max = bmin + extent * bvhvec3( d1.y, d3.y, d5.y );
		const bvhvec3 c2min = bmin + extent * bvhvec3( d0.z, d2.z, d4.z ), c2max = bmin + extent * bvhvec3( d1.z, d3.z, d5.z );
		const bvhvec3 c3min = bmin + extent * bvhvec3( d0.w, d2.w, d4.w ), c3max = bmin + extent * bvhvec3( d1.w, d3.w, d5.w );
		// intersect child aabbs
		const bvhvec3 t1a = (c0min - ray.O) * ray.rD, t2a = (c0max - ray.O) * ray.rD;
		const bvhvec3 t1b = (c1min - ray.O) * ray.rD, t2b = (c1max - ray.O) * ray.rD;
		const bvhvec3 t1c = (c2min - ray.O) * ray.rD, t2c = (c2max - ray.O) * ray.rD;
		const bvhvec3 t1d = (c3min - ray.O) * ray.rD, t2d = (c3max - ray.O) * ray.rD;
		const bvhvec3 minta = tinybvh_min( t1a, t2a ), maxta = tinybvh_max( t1a, t2a );
		const bvhvec3 mintb = tinybvh_min( t1b, t2b ), maxtb = tinybvh_max( t1b, t2b );
		const bvhvec3 mintc = tinybvh_min( t1c, t2c ), maxtc = tinybvh_max( t1c, t2c );
		const bvhvec3 mintd = tinybvh_min( t1d, t2d ), maxtd = tinybvh_max( t1d, t2d );
		const float tmina = tinybvh_max( tinybvh_max( tinybvh_max( minta.x, minta.y ), minta.z ), 0.0f );
		const float tminb = tinybvh_max( tinybvh_max( tinybvh_max( mintb.x, mintb.y ), mintb.z ), 0.0f );
		const float tminc = tinybvh_max( tinybvh_max( tinybvh_max( mintc.x, mintc.y ), mintc.z ), 0.0f );
		const float tmind = tinybvh_max( tinybvh_max( tinybvh_max( mintd.x, mintd.y ), mintd.z ), 0.0f );
		const float tmaxa = tinybvh_min( tinybvh_min( tinybvh_min( maxta.x, maxta.y ), maxta.z ), ray.hit.t );
		const float tmaxb = tinybvh_min( tinybvh_min( tinybvh_min( maxtb.x, maxtb.y ), maxtb.z ), ray.hit.t );
		const float tmaxc = tinybvh_min( tinybvh_min( tinybvh_min( maxtc.x, maxtc.y ), maxtc.z ), ray.hit.t );
		const float tmaxd = tinybvh_min( tinybvh_min( tinybvh_min( maxtd.x, maxtd.y ), maxtd.z ), ray.hit.t );
		float dist0 = tmina > tmaxa ? BVH_FAR : tmina, dist1 = tminb > tmaxb ? BVH_FAR : tminb;
		float dist2 = tminc > tmaxc ? BVH_FAR : tminc, dist3 = tmind > tmaxd ? BVH_FAR : tmind, t;
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
		if (dist0 < BVH_FAR)
		{
			if (c0info & 0x80000000) leaf[leafs++] = c0info; else if (c0info) stack[stackPtr++] = c0info;
		}
		if (dist1 < BVH_FAR)
		{
			if (c1info & 0x80000000) leaf[leafs++] = c1info; else if (c1info) stack[stackPtr++] = c1info;
		}
		if (dist2 < BVH_FAR)
		{
			if (c2info & 0x80000000) leaf[leafs++] = c2info; else if (c2info) stack[stackPtr++] = c2info;
		}
		if (dist3 < BVH_FAR)
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
				cost += C_INT;
				const bvhvec3 edge2 = bvhvec3( bvh4Data[triStart + 2] );
				const bvhvec3 edge1 = bvhvec3( bvh4Data[triStart + 1] );
				const bvhvec3 v0 = bvh4Data[triStart + 0];
				const bvhvec3 h = tinybvh_cross( ray.D, edge2 );
				const float a = tinybvh_dot( edge1, h );
				if (fabs( a ) < 0.0000001f) continue;
				const float f = 1 / a;
				const bvhvec3 s = ray.O - v0;
				const float u = f * tinybvh_dot( s, h );
				if (u < 0 || u > 1) continue;
				const bvhvec3 q = tinybvh_cross( s, edge1 );
				const float v = f * tinybvh_dot( ray.D, q );
				if (v < 0 || u + v > 1) continue;
				const float d = f * tinybvh_dot( edge2, q );
				if (d <= 0.0f || d >= ray.hit.t /* i.e., t */) continue;
				ray.hit.t = d, ray.hit.u = u, ray.hit.v = v;
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
	return cost;
}

// BVH8_CWBVH implementation
// ----------------------------------------------------------------------------

BVH8_CWBVH::~BVH8_CWBVH()
{
	if (!ownBVH8) bvh8 = MBVH<8>(); // clear out pointers we don't own.
	AlignedFree( bvh8Data );
	AlignedFree( bvh8Tris );
}

void BVH8_CWBVH::Optimize( const uint32_t iterations, bool extreme )
{
	bvh8.Optimize( iterations, extreme );
	ConvertFrom( bvh8, true );
}

float BVH8_CWBVH::SAHCost( const uint32_t nodeIdx ) const
{
	return bvh8.SAHCost( nodeIdx );
}

void BVH8_CWBVH::Save( const char* fileName )
{
	std::fstream s{ fileName, s.binary | s.out };
	uint32_t header = TINY_BVH_VERSION_SUB + (TINY_BVH_VERSION_MINOR << 8) + (TINY_BVH_VERSION_MAJOR << 16) + (layout << 24);
	s.write( (char*)&header, sizeof( uint32_t ) );
	s.write( (char*)&triCount, sizeof( uint32_t ) );
	s.write( (char*)this, sizeof( BVH8_CWBVH ) );
	s.write( (char*)bvh8Data, usedBlocks * 16 );
	s.write( (char*)bvh8Tris, bvh8.idxCount * 4 * 16 );
}

bool BVH8_CWBVH::Load( const char* fileName, const uint32_t expectedTris )
{
	// open file and check contents
	std::fstream s{ fileName, s.binary | s.in };
	if (!s) return false;
	BVHContext tmp = context;
	uint32_t header, fileTriCount;
	s.read( (char*)&header, sizeof( uint32_t ) );
	if (((header >> 8) & 255) != TINY_BVH_VERSION_MINOR ||
		((header >> 16) & 255) != TINY_BVH_VERSION_MAJOR ||
		(header & 255) != TINY_BVH_VERSION_SUB || (header >> 24) != layout) return false;
	s.read( (char*)&fileTriCount, sizeof( uint32_t ) );
	if (fileTriCount != expectedTris) return false;
	// all checks passed; safe to overwrite *this
	s.read( (char*)this, sizeof( BVH8_CWBVH ) );
	context = tmp; // can't load context; function pointers will differ.
	bvh8Data = (bvhvec4*)AlignedAlloc( usedBlocks * 16 );
	bvh8Tris = (bvhvec4*)AlignedAlloc( bvh8.idxCount * 4 * 16 );
	allocatedBlocks = usedBlocks;
	s.read( (char*)bvh8Data, usedBlocks * 16 );
	s.read( (char*)bvh8Tris, bvh8.idxCount * 4 * 16 );
	bvh8 = MBVH<8>();
	return true;
}

void BVH8_CWBVH::Build( const bvhvec4* vertices, const uint32_t primCount )
{
	Build( bvhvec4slice( vertices, primCount * 3, sizeof( bvhvec4 ) ) );
}

void BVH8_CWBVH::Build( const bvhvec4slice& vertices )
{
	bvh8.context = context; // properly propagate context to fix issue #66.
	bvh8.Build( vertices );
	ConvertFrom( bvh8, true );
}

void BVH8_CWBVH::Build( const bvhvec4* vertices, const uint32_t* indices, const uint32_t prims )
{
	// build the BVH with a continuous array of bvhvec4 vertices, indexed by 'indices'.
	Build( bvhvec4slice{ vertices, prims * 3, sizeof( bvhvec4 ) }, indices, prims );
}

void BVH8_CWBVH::Build( const bvhvec4slice& vertices, const uint32_t* indices, uint32_t prims )
{
	// build the BVH from vertices stored in a slice, indexed by 'indices'.
	bvh8.context = context;
	bvh8.Build( vertices, indices, prims );
	ConvertFrom( bvh8, true );
}

void BVH8_CWBVH::BuildHQ( const bvhvec4* vertices, const uint32_t primCount )
{
	BuildHQ( bvhvec4slice( vertices, primCount * 3, sizeof( bvhvec4 ) ) );
}

void BVH8_CWBVH::BuildHQ( const bvhvec4slice& vertices )
{
	bvh8.context = context; // properly propagate context to fix issue #66.
	bvh8.BuildHQ( vertices );
	ConvertFrom( bvh8, true );
}

void BVH8_CWBVH::BuildHQ( const bvhvec4* vertices, const uint32_t* indices, const uint32_t prims )
{
	Build( bvhvec4slice{ vertices, prims * 3, sizeof( bvhvec4 ) }, indices, prims );
}

void BVH8_CWBVH::BuildHQ( const bvhvec4slice& vertices, const uint32_t* indices, uint32_t prims )
{
	bvh8.context = context;
	bvh8.BuildHQ( vertices, indices, prims );
	ConvertFrom( bvh8, true );
}

void BVH8_CWBVH::ConvertFrom( MBVH<8>& original, bool )
{
	// get a copy of the original bvh8
	if (&original != &bvh8) ownBVH8 = false; // bvh isn't ours; don't delete in destructor.
	bvh8 = original;
	// Convert a BVH8 to the format specified in: "Efficient Incoherent Ray
	// Traversal on GPUs Through Compressed Wide BVHs", Ylitie et al. 2017.
	// Adapted from code by "AlanWBFT".
	FATAL_ERROR_IF( bvh8.mbvhNode[0].isLeaf(), "BVH8_CWBVH::ConvertFrom( .. ), converting a single-node bvh." );
	// allocate memory
	// Note: This can be far lower (specifically: usedNodes) if we know that
	// none of the BVH8 leafs has more than three primitives.
	// Without this guarantee, the only safe upper limit is triCount * 2, since
	// we will be splitting fat BVH8 leafs to as we go.
	uint32_t spaceNeeded = bvh8.triCount * 2 * 5; // CWBVH nodes use 80 bytes each.
	if (spaceNeeded > allocatedBlocks)
	{
		bvh8Data = (bvhvec4*)AlignedAlloc( spaceNeeded * 16 );
		bvh8Tris = (bvhvec4*)AlignedAlloc( bvh8.idxCount * 4 * 16 );
		allocatedBlocks = spaceNeeded;
	}
	memset( bvh8Data, 0, spaceNeeded * 16 );
	memset( bvh8Tris, 0, bvh8.idxCount * 3 * 16 );
	CopyBasePropertiesFrom( bvh8 );
	MBVH<8>::MBVHNode* stackNodePtr[256];
	uint32_t stackNodeAddr[256], stackPtr = 1, nodeDataPtr = 5, triDataPtr = 0;
	stackNodePtr[0] = &bvh8.mbvhNode[0], stackNodeAddr[0] = 0;
	// start conversion
	while (stackPtr > 0)
	{
		MBVH<8>::MBVHNode* orig = stackNodePtr[--stackPtr];
		const int32_t currentNodeAddr = stackNodeAddr[stackPtr];
		bvhvec3 nodeLo = orig->aabbMin, nodeHi = orig->aabbMax;
		// greedy child node ordering
		const bvhvec3 nodeCentroid = (nodeLo + nodeHi) * 0.5f;
		float cost[8][8];
		int32_t assignment[8];
		bool isSlotEmpty[8];
		for (int32_t s = 0; s < 8; s++)
		{
			isSlotEmpty[s] = true, assignment[s] = -1;
			bvhvec3 ds(
				(((s >> 2) & 1) == 1) ? -1.0f : 1.0f,
				(((s >> 1) & 1) == 1) ? -1.0f : 1.0f,
				(((s >> 0) & 1) == 1) ? -1.0f : 1.0f
			);
			for (int32_t i = 0; i < 8; i++) if (orig->child[i] == 0) cost[s][i] = BVH_FAR; else
			{
				MBVH<8>::MBVHNode* const child = &bvh8.mbvhNode[orig->child[i]];
				if (child->triCount > 3 /* must be leaf */) bvh8.SplitBVHLeaf( orig->child[i], 3 );
				bvhvec3 childCentroid = (child->aabbMin + child->aabbMax) * 0.5f;
				cost[s][i] = tinybvh_dot( childCentroid - nodeCentroid, ds );
			}
		}
		while (1)
		{
			float minCost = BVH_FAR;
			int32_t minEntryx = -1, minEntryy = -1;
			for (int32_t s = 0; s < 8; s++) for (int32_t i = 0; i < 8; i++)
				if (assignment[i] == -1 && isSlotEmpty[s] && cost[s][i] < minCost)
					minCost = cost[s][i], minEntryx = s, minEntryy = i;
			if (minEntryx == -1 && minEntryy == -1) break;
			isSlotEmpty[minEntryx] = false, assignment[minEntryy] = minEntryx;
		}
		for (int32_t i = 0; i < 8; i++) if (assignment[i] == -1) for (int32_t s = 0; s < 8; s++) if (isSlotEmpty[s])
		{
			isSlotEmpty[s] = false, assignment[i] = s;
			break;
		}
		const MBVH<8>::MBVHNode oldNode = *orig;
		for (int32_t i = 0; i < 8; i++) orig->child[assignment[i]] = oldNode.child[i];
		// calculate quantization parameters for each axis
		const int32_t ex = (int32_t)((int8_t)ceilf( log2f( (nodeHi.x - nodeLo.x) / 255.0f ) ));
		const int32_t ey = (int32_t)((int8_t)ceilf( log2f( (nodeHi.y - nodeLo.y) / 255.0f ) ));
		const int32_t ez = (int32_t)((int8_t)ceilf( log2f( (nodeHi.z - nodeLo.z) / 255.0f ) ));
		// encode output
		int32_t internalChildCount = 0, leafChildTriCount = 0, childBaseIndex = 0, triangleBaseIndex = 0;
		uint8_t imask = 0;
		for (int32_t i = 0; i < 8; i++)
		{
			if (orig->child[i] == 0) continue;
			MBVH<8>::MBVHNode* const child = &bvh8.mbvhNode[orig->child[i]];
			const int32_t qlox = (int32_t)floorf( (child->aabbMin.x - nodeLo.x) / powf( 2, (float)ex ) );
			const int32_t qloy = (int32_t)floorf( (child->aabbMin.y - nodeLo.y) / powf( 2, (float)ey ) );
			const int32_t qloz = (int32_t)floorf( (child->aabbMin.z - nodeLo.z) / powf( 2, (float)ez ) );
			const int32_t qhix = (int32_t)ceilf( (child->aabbMax.x - nodeLo.x) / powf( 2, (float)ex ) );
			const int32_t qhiy = (int32_t)ceilf( (child->aabbMax.y - nodeLo.y) / powf( 2, (float)ey ) );
			const int32_t qhiz = (int32_t)ceilf( (child->aabbMax.z - nodeLo.z) / powf( 2, (float)ez ) );
			uint8_t* const baseAddr = (uint8_t*)&bvh8Data[currentNodeAddr + 2];
			baseAddr[i + 0] = (uint8_t)qlox, baseAddr[i + 24] = (uint8_t)qhix;
			baseAddr[i + 8] = (uint8_t)qloy, baseAddr[i + 32] = (uint8_t)qhiy;
			baseAddr[i + 16] = (uint8_t)qloz, baseAddr[i + 40] = (uint8_t)qhiz;
			if (!child->isLeaf())
			{
				// interior node, set params and push onto stack
				const int32_t childNodeAddr = nodeDataPtr;
				if (internalChildCount++ == 0) childBaseIndex = childNodeAddr / 5;
				nodeDataPtr += 5, imask |= 1 << i;
				// set the meta field - This calculation assumes children are stored contiguously.
				uint8_t* const childMetaField = ((uint8_t*)&bvh8Data[currentNodeAddr + 1]) + 8;
				childMetaField[i] = (1 << 5) | (24 + (uint8_t)i); // I don't see how this accounts for empty children?
				stackNodePtr[stackPtr] = child, stackNodeAddr[stackPtr++] = childNodeAddr; // counted in float4s
				internalChildCount++;
				continue;
			}
			// leaf node
			const uint32_t tcount = tinybvh_min( child->triCount, 3u ); // TODO: ensure that's the case; clamping for now.
			if (leafChildTriCount == 0) triangleBaseIndex = triDataPtr;
			int32_t unaryEncodedTriCount = tcount == 1 ? 0b001 : tcount == 2 ? 0b011 : 0b111;
			// set the meta field - This calculation assumes children are stored contiguously.
			uint8_t* const childMetaField = ((uint8_t*)&bvh8Data[currentNodeAddr + 1]) + 8;
			childMetaField[i] = (uint8_t)((unaryEncodedTriCount << 5) | leafChildTriCount);
			leafChildTriCount += tcount;
			for (uint32_t j = 0; j < tcount; j++)
			{
				int32_t triIdx = bvh8.bvh.primIdx[child->firstTri + j];
				uint32_t ti0, ti1, ti2;
				if (bvh8.bvh.vertIdx)
					ti0 = bvh8.bvh.vertIdx[triIdx * 3],
					ti1 = bvh8.bvh.vertIdx[triIdx * 3 + 1],
					ti2 = bvh8.bvh.vertIdx[triIdx * 3 + 2];
				else
					ti0 = triIdx * 3, ti1 = triIdx * 3 + 1, ti2 = triIdx * 3 + 2;
			#ifdef CWBVH_COMPRESSED_TRIS
				PrecomputeTriangle( verts, ti0, ti1, ti2, (float*)&bvh8Tris[triDataPtr] );
				bvh8Tris[triDataPtr + 3] = bvhvec4( 0, 0, 0, *(float*)&triIdx );
				triDataPtr += 4;
			#else
				bvhvec4 t = bvh8.bvh.verts[ti0];
				bvh8Tris[triDataPtr + 0] = bvh8.bvh.verts[ti2] - t;
				bvh8Tris[triDataPtr + 1] = bvh8.bvh.verts[ti1] - t;
				t.w = *(float*)&triIdx;
				bvh8Tris[triDataPtr + 2] = t, triDataPtr += 3;
			#endif
			}
		}
		uint8_t exyzAndimask[4] = { *(uint8_t*)&ex, *(uint8_t*)&ey, *(uint8_t*)&ez, imask };
		bvh8Data[currentNodeAddr + 0] = bvhvec4( nodeLo, *(float*)&exyzAndimask );
		bvh8Data[currentNodeAddr + 1].x = *(float*)&childBaseIndex;
		bvh8Data[currentNodeAddr + 1].y = *(float*)&triangleBaseIndex;
	}
	usedBlocks = nodeDataPtr;
}

// ============================================================================
//
//        I M P L E M E N T A T I O N  -  A V X / S S E  C O D E
//
// ============================================================================

#ifdef BVH_USEAVX

// Ultra-fast single-threaded AVX binned-SAH-builder.
// This code produces BVHs nearly identical to reference, but much faster.
// On a 12th gen laptop i7 CPU, Sponza Crytek (~260k tris) is processed in 51ms.
// The code relies on the availability of AVX instructions. AVX2 is not needed.
#if defined(_MSC_VER) && !defined(__clang__)
#define LANE(a,b) a.m128_f32[b]
#define LANE8(a,b) a.m256_f32[b]
// Not using clang/g++ method under MSCC; compiler may benefit from .m128_i32.
#define ILANE(a,b) a.m128i_i32[b]
#else
#define LANE(a,b) a[b]
#define LANE8(a,b) a[b]
// Below method reduces to a single instruction.
#define ILANE(a,b) _mm_cvtsi128_si32(_mm_castps_si128( _mm_shuffle_ps(_mm_castsi128_ps( a ), _mm_castsi128_ps( a ), b)))
#endif
inline float halfArea( const __m128 a /* a contains extent of aabb */ )
{
	return LANE( a, 0 ) * LANE( a, 1 ) + LANE( a, 1 ) * LANE( a, 2 ) + LANE( a, 2 ) * LANE( a, 3 );
}
inline float halfArea( const __m256& a /* a contains aabb itself, with min.xyz negated */ )
{
#ifndef _MSC_VER
	// g++ doesn't seem to like the faster construct
	float* c = (float*)&a;
	float ex = c[4] + c[0], ey = c[5] + c[1], ez = c[6] + c[2];
	return ex * ey + ey * ez + ez * ex;
#else
	const __m128 q = _mm256_castps256_ps128( _mm256_add_ps( _mm256_permute2f128_ps( a, a, 5 ), a ) );
	const __m128 v = _mm_mul_ps( q, _mm_shuffle_ps( q, q, 9 ) );
	return LANE( v, 0 ) + LANE( v, 1 ) + LANE( v, 2 );
#endif
}
#define PROCESS_PLANE( a, pos, ANLR, lN, rN, lb, rb ) if (lN * rN != 0) { \
	ANLR = halfArea( lb ) * (float)lN + halfArea( rb ) * (float)rN; \
	const float C = C_TRAV + C_INT * rSAV * ANLR; if (C < splitCost) \
	splitCost = C, bestAxis = a, bestPos = pos, bestLBox = lb, bestRBox = rb; }
#if defined(_MSC_VER)
#pragma warning ( push )
#pragma warning( disable:4701 ) // "potentially uninitialized local variable 'bestLBox' used"
#elif defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif
void BVH::BuildAVX( const bvhvec4* vertices, const uint32_t primCount )
{
	// build the BVH with a continuous array of bvhvec4 vertices:
	// in this case, the stride for the slice is 16 bytes.
	BuildAVX( bvhvec4slice{ vertices, primCount * 3, sizeof( bvhvec4 ) } );
}
void BVH::BuildAVX( const bvhvec4slice& vertices )
{
	PrepareAVXBuild( vertices, 0, 0 );
	BuildAVX();
}
void BVH::BuildAVX( const bvhvec4* vertices, const uint32_t* indices, const uint32_t primCount )
{
	// build the BVH with an indexed array of bvhvec4 vertices.
	BuildAVX( bvhvec4slice{ vertices, primCount * 3, sizeof( bvhvec4 ) }, indices, primCount );
}
void BVH::BuildAVX( const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t primCount )
{
	PrepareAVXBuild( vertices, indices, primCount );
	BuildAVX();
}
void BVH::PrepareAVXBuild( const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t prims )
{
	FATAL_ERROR_IF( vertices.count == 0, "BVH::PrepareAVXBuild( .. ), primCount == 0." );
	FATAL_ERROR_IF( vertices.stride & 15, "BVH::PrepareAVXBuild( .. ), stride must be multiple of 16." );
	// some constants
	static const __m128 min4 = _mm_set1_ps( BVH_FAR ), max4 = _mm_set1_ps( -BVH_FAR );
	// reset node pool
	uint32_t primCount = prims > 0 ? prims : vertices.count / 3;
	const uint32_t spaceNeeded = primCount * 2;
	if (allocatedNodes < spaceNeeded)
	{
		AlignedFree( bvhNode );
		AlignedFree( primIdx );
		AlignedFree( fragment );
		primIdx = (uint32_t*)AlignedAlloc( primCount * sizeof( uint32_t ) );
		bvhNode = (BVHNode*)AlignedAlloc( spaceNeeded * sizeof( BVHNode ) );
		allocatedNodes = spaceNeeded;
		memset( &bvhNode[1], 0, 32 ); // avoid crash in refit.
		fragment = (Fragment*)AlignedAlloc( primCount * sizeof( Fragment ) );
	}
	else FATAL_ERROR_IF( !rebuildable, "BVH::BuildAVX( .. ), bvh not rebuildable." );
	verts = vertices; // note: we're not copying this data; don't delete.
	vertIdx = (uint32_t*)indices;
	triCount = idxCount = primCount;
	newNodePtr = 2;
	struct FragSSE { __m128 bmin4, bmax4; };
	FragSSE* frag4 = (FragSSE*)fragment;
	const __m128* verts4 = (__m128*)verts.data; // that's why it must be 16-byte aligned.
	// assign all triangles to the root node
	BVHNode& root = bvhNode[0];
	root.leftFirst = 0, root.triCount = triCount;
	// initialize fragments and update root bounds
	__m128 rootMin = min4, rootMax = max4;
	if (indices)
	{
		FATAL_ERROR_IF( vertices.count == 0, "BVH::PrepareAVXBuild( .. ), empty vertex slice." );
		FATAL_ERROR_IF( prims == 0, "BVH::PrepareAVXBuild( .. ), prims == 0." );
		// build the BVH over indexed triangles
		for (uint32_t i = 0; i < triCount; i++)
		{
			const uint32_t i0 = indices[i * 3], i1 = indices[i * 3 + 1], i2 = indices[i * 3 + 2];
			const __m128 v0 = verts4[i0], v1 = verts4[i1], v2 = verts4[i2];
			const __m128 t1 = _mm_min_ps( _mm_min_ps( v0, v1 ), v2 );
			const __m128 t2 = _mm_max_ps( _mm_max_ps( v0, v1 ), v2 );
			frag4[i].bmin4 = t1, frag4[i].bmax4 = t2, rootMin = _mm_min_ps( rootMin, t1 ), rootMax = _mm_max_ps( rootMax, t2 );
			primIdx[i] = i;
		}
	}
	else
	{
		FATAL_ERROR_IF( vertices.count == 0, "BVH::PrepareAVXBuild( .. ), empty vertex slice." );
		FATAL_ERROR_IF( prims != 0, "BVH::PrepareAVXBuild( .. ), indices == 0." );
		// build the BVH over a list of vertices: three per triangle
		for (uint32_t i = 0; i < triCount; i++)
		{
			const __m128 v0 = verts4[i * 3], v1 = verts4[i * 3 + 1], v2 = verts4[i * 3 + 2];
			const __m128 t1 = _mm_min_ps( _mm_min_ps( v0, v1 ), v2 );
			const __m128 t2 = _mm_max_ps( _mm_max_ps( v0, v1 ), v2 );
			frag4[i].bmin4 = t1, frag4[i].bmax4 = t2, rootMin = _mm_min_ps( rootMin, t1 ), rootMax = _mm_max_ps( rootMax, t2 );
			primIdx[i] = i;
		}
	}
	root.aabbMin = *(bvhvec3*)&rootMin, root.aabbMax = *(bvhvec3*)&rootMax;
	bvh_over_indices = indices != nullptr;
}
void BVH::BuildAVX()
{
	// aligned data
	ALIGNED( 64 ) __m256 binbox[3 * AVXBINS];			// 768 bytes
	ALIGNED( 64 ) __m256 binboxOrig[3 * AVXBINS];		// 768 bytes
	ALIGNED( 64 ) uint32_t count[3][AVXBINS]{};			// 96 bytes
	ALIGNED( 64 ) __m256 bestLBox, bestRBox;			// 64 bytes
	// some constants
	static const __m128 half4 = _mm_set1_ps( 0.5f );
	static const __m128 two4 = _mm_set1_ps( 2.0f ), min1 = _mm_set1_ps( -1 );
	static const __m128i maxbin4 = _mm_set1_epi32( 7 );
	static const __m128 mask3 = _mm_cmpeq_ps( _mm_setr_ps( 0, 0, 0, 1 ), _mm_setzero_ps() );
	static const __m128 binmul3 = _mm_set1_ps( AVXBINS * 0.49999f );
	static const __m256 max8 = _mm256_set1_ps( -BVH_FAR ), mask6 = _mm256_set_m128( mask3, mask3 );
	static const __m256 signFlip8 = _mm256_setr_ps( -0.0f, -0.0f, -0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f );
	for (uint32_t i = 0; i < 3 * AVXBINS; i++) binboxOrig[i] = max8; // binbox initialization template
	struct FragSSE { __m128 bmin4, bmax4; };
	FragSSE* frag4 = (FragSSE*)fragment;
	__m256* frag8 = (__m256*)fragment;
	// subdivide recursively
	ALIGNED( 64 ) uint32_t task[128], taskCount = 0, nodeIdx = 0;
	BVHNode& root = bvhNode[0];
	const bvhvec3 minDim = (root.aabbMax - root.aabbMin) * 1e-7f;
	while (1)
	{
		while (1)
		{
			BVHNode& node = bvhNode[nodeIdx];
			__m128* node4 = (__m128*) & bvhNode[nodeIdx];
			// find optimal object split
			const __m128 d4 = _mm_blendv_ps( min1, _mm_sub_ps( node4[1], node4[0] ), mask3 );
			const __m128 nmin4 = _mm_mul_ps( _mm_and_ps( node4[0], mask3 ), two4 );
			const __m128 rpd4 = _mm_and_ps( _mm_div_ps( binmul3, d4 ), _mm_cmpneq_ps( d4, _mm_setzero_ps() ) );
			// implementation of Section 4.1 of "Parallel Spatial Splits in Bounding Volume Hierarchies":
			// main loop operates on two fragments to minimize dependencies and maximize ILP.
			uint32_t fi = primIdx[node.leftFirst];
			memset( count, 0, sizeof( count ) );
			__m256 r0, r1, r2, f = _mm256_xor_ps( _mm256_and_ps( frag8[fi], mask6 ), signFlip8 );
			const __m128 fmin = _mm_and_ps( frag4[fi].bmin4, mask3 ), fmax = _mm_and_ps( frag4[fi].bmax4, mask3 );
			const __m128i bi4 = _mm_cvtps_epi32( _mm_sub_ps( _mm_mul_ps( _mm_sub_ps( _mm_add_ps( fmax, fmin ), nmin4 ), rpd4 ), half4 ) );
			const __m128i b4c = _mm_max_epi32( _mm_min_epi32( bi4, maxbin4 ), _mm_setzero_si128() ); // clamp needed after all
			memcpy( binbox, binboxOrig, sizeof( binbox ) );
			uint32_t i0 = ILANE( b4c, 0 ), i1 = ILANE( b4c, 1 ), i2 = ILANE( b4c, 2 ), * ti = primIdx + node.leftFirst + 1;
			for (uint32_t i = 0; i < node.triCount - 1; i++)
			{
				uint32_t fid = *ti++;
			#if defined __GNUC__ || _MSC_VER < 1920
				if (fid > triCount) fid = triCount - 1; // never happens but g++ *and* vs2017 need this to not crash...
			#endif
				const __m256 b0 = binbox[i0], b1 = binbox[AVXBINS + i1], b2 = binbox[2 * AVXBINS + i2];
				const __m128 frmin = _mm_and_ps( frag4[fid].bmin4, mask3 ), frmax = _mm_and_ps( frag4[fid].bmax4, mask3 );
				r0 = _mm256_max_ps( b0, f ), r1 = _mm256_max_ps( b1, f ), r2 = _mm256_max_ps( b2, f );
				const __m128i b4 = _mm_cvtps_epi32( _mm_sub_ps( _mm_mul_ps( _mm_sub_ps( _mm_add_ps( frmax, frmin ), nmin4 ), rpd4 ), half4 ) );
				const __m128i bc4 = _mm_max_epi32( _mm_min_epi32( b4, maxbin4 ), _mm_setzero_si128() ); // clamp needed after all
				f = _mm256_xor_ps( _mm256_and_ps( frag8[fid], mask6 ), signFlip8 ), count[0][i0]++, count[1][i1]++, count[2][i2]++;
				binbox[i0] = r0, i0 = ILANE( bc4, 0 );
				binbox[AVXBINS + i1] = r1, i1 = ILANE( bc4, 1 );
				binbox[2 * AVXBINS + i2] = r2, i2 = ILANE( bc4, 2 );
			}
			// final business for final fragment
			const __m256 b0 = binbox[i0], b1 = binbox[AVXBINS + i1], b2 = binbox[2 * AVXBINS + i2];
			count[0][i0]++, count[1][i1]++, count[2][i2]++;
			r0 = _mm256_max_ps( b0, f ), r1 = _mm256_max_ps( b1, f ), r2 = _mm256_max_ps( b2, f );
			binbox[i0] = r0, binbox[AVXBINS + i1] = r1, binbox[2 * AVXBINS + i2] = r2;
			// calculate per-split totals
			float splitCost = BVH_FAR, rSAV = 1.0f / node.SurfaceArea();
			uint32_t bestAxis = 0, bestPos = 0, n = newNodePtr, j = node.leftFirst + node.triCount, src = node.leftFirst;
			const __m256* bb = binbox;
			for (int32_t a = 0; a < 3; a++, bb += AVXBINS) if ((node.aabbMax[a] - node.aabbMin[a]) > minDim.cell[a])
			{
				// hardcoded bin processing for AVXBINS == 8
				assert( AVXBINS == 8 );
				const uint32_t lN0 = count[a][0], rN0 = count[a][7];
				const __m256 lb0 = bb[0], rb0 = bb[7];
				const uint32_t lN1 = lN0 + count[a][1], rN1 = rN0 + count[a][6], lN2 = lN1 + count[a][2];
				const uint32_t rN2 = rN1 + count[a][5], lN3 = lN2 + count[a][3], rN3 = rN2 + count[a][4];
				const __m256 lb1 = _mm256_max_ps( lb0, bb[1] ), rb1 = _mm256_max_ps( rb0, bb[6] );
				const __m256 lb2 = _mm256_max_ps( lb1, bb[2] ), rb2 = _mm256_max_ps( rb1, bb[5] );
				const __m256 lb3 = _mm256_max_ps( lb2, bb[3] ), rb3 = _mm256_max_ps( rb2, bb[4] );
				const uint32_t lN4 = lN3 + count[a][4], rN4 = rN3 + count[a][3], lN5 = lN4 + count[a][5];
				const uint32_t rN5 = rN4 + count[a][2], lN6 = lN5 + count[a][6], rN6 = rN5 + count[a][1];
				const __m256 lb4 = _mm256_max_ps( lb3, bb[4] ), rb4 = _mm256_max_ps( rb3, bb[3] );
				const __m256 lb5 = _mm256_max_ps( lb4, bb[5] ), rb5 = _mm256_max_ps( rb4, bb[2] );
				const __m256 lb6 = _mm256_max_ps( lb5, bb[6] ), rb6 = _mm256_max_ps( rb5, bb[1] );
				float ANLR3 = BVH_FAR; PROCESS_PLANE( a, 3, ANLR3, lN3, rN3, lb3, rb3 ); // most likely split
				float ANLR2 = BVH_FAR; PROCESS_PLANE( a, 2, ANLR2, lN2, rN4, lb2, rb4 );
				float ANLR4 = BVH_FAR; PROCESS_PLANE( a, 4, ANLR4, lN4, rN2, lb4, rb2 );
				float ANLR5 = BVH_FAR; PROCESS_PLANE( a, 5, ANLR5, lN5, rN1, lb5, rb1 );
				float ANLR1 = BVH_FAR; PROCESS_PLANE( a, 1, ANLR1, lN1, rN5, lb1, rb5 );
				float ANLR0 = BVH_FAR; PROCESS_PLANE( a, 0, ANLR0, lN0, rN6, lb0, rb6 );
				float ANLR6 = BVH_FAR; PROCESS_PLANE( a, 6, ANLR6, lN6, rN0, lb6, rb0 ); // least likely split
			}
			float noSplitCost = (float)node.triCount * C_INT;
			if (splitCost >= noSplitCost) break; // not splitting is better.
			// in-place partition
			const float rpd = (*(bvhvec3*)&rpd4)[bestAxis], nmin = (*(bvhvec3*)&nmin4)[bestAxis];
			uint32_t t, fr = primIdx[src];
			for (uint32_t i = 0; i < node.triCount; i++)
			{
				const uint32_t bi = (uint32_t)((fragment[fr].bmax[bestAxis] + fragment[fr].bmin[bestAxis] - nmin) * rpd);
				if (bi <= bestPos) fr = primIdx[++src]; else t = fr, fr = primIdx[src] = primIdx[--j], primIdx[j] = t;
			}
			// create child nodes and recurse
			const uint32_t leftCount = src - node.leftFirst, rightCount = node.triCount - leftCount;
			if (leftCount == 0 || rightCount == 0) break; // should not happen.
			*(__m256*)& bvhNode[n] = _mm256_xor_ps( bestLBox, signFlip8 );
			bvhNode[n].leftFirst = node.leftFirst, bvhNode[n].triCount = leftCount;
			node.leftFirst = n++, node.triCount = 0, newNodePtr += 2;
			*(__m256*)& bvhNode[n] = _mm256_xor_ps( bestRBox, signFlip8 );
			bvhNode[n].leftFirst = j, bvhNode[n].triCount = rightCount;
			task[taskCount++] = n, nodeIdx = n - 1;
		}
		// fetch subdivision task from stack
		if (taskCount == 0) break; else nodeIdx = task[--taskCount];
	}
	// all done.
	aabbMin = bvhNode[0].aabbMin, aabbMax = bvhNode[0].aabbMax;
	refittable = true; // not using spatial splits: can refit this BVH
	may_have_holes = false; // the AVX builder produces a continuous list of nodes
	usedNodes = newNodePtr;
}
#if defined(_MSC_VER)
#pragma warning ( pop ) // restore 4701
#elif defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop // restore -Wmaybe-uninitialized
#endif

// Intersect a BVH with a ray packet, basic SSE-optimized version.
// Note: This yields +10% on 10th gen Intel CPUs, but a small loss on
// more recent hardware. This function needs a full conversion to work
// with groups of 8 rays at a time - TODO.
void BVH::Intersect256RaysSSE( Ray* packet ) const
{
	// Corner rays are: 0, 51, 204 and 255
	// Construct the bounding planes, with normals pointing outwards
	bvhvec3 O = packet[0].O; // same for all rays in this case
	__m128 O4 = *(__m128*) & packet[0].O;
	__m128 mask4 = _mm_cmpeq_ps( _mm_setzero_ps(), _mm_set_ps( 1, 0, 0, 0 ) );
	bvhvec3 p0 = packet[0].O + packet[0].D; // top-left
	bvhvec3 p1 = packet[51].O + packet[51].D; // top-right
	bvhvec3 p2 = packet[204].O + packet[204].D; // bottom-left
	bvhvec3 p3 = packet[255].O + packet[255].D; // bottom-right
	bvhvec3 plane0 = tinybvh_normalize( tinybvh_cross( p0 - O, p0 - p2 ) ); // left plane
	bvhvec3 plane1 = tinybvh_normalize( tinybvh_cross( p3 - O, p3 - p1 ) ); // right plane
	bvhvec3 plane2 = tinybvh_normalize( tinybvh_cross( p1 - O, p1 - p0 ) ); // top plane
	bvhvec3 plane3 = tinybvh_normalize( tinybvh_cross( p2 - O, p2 - p3 ) ); // bottom plane
	int32_t sign0x = plane0.x < 0 ? 4 : 0, sign0y = plane0.y < 0 ? 5 : 1, sign0z = plane0.z < 0 ? 6 : 2;
	int32_t sign1x = plane1.x < 0 ? 4 : 0, sign1y = plane1.y < 0 ? 5 : 1, sign1z = plane1.z < 0 ? 6 : 2;
	int32_t sign2x = plane2.x < 0 ? 4 : 0, sign2y = plane2.y < 0 ? 5 : 1, sign2z = plane2.z < 0 ? 6 : 2;
	int32_t sign3x = plane3.x < 0 ? 4 : 0, sign3y = plane3.y < 0 ? 5 : 1, sign3z = plane3.z < 0 ? 6 : 2;
	float t0 = tinybvh_dot( O, plane0 ), t1 = tinybvh_dot( O, plane1 );
	float t2 = tinybvh_dot( O, plane2 ), t3 = tinybvh_dot( O, plane3 );
	// Traverse the tree with the packet
	int32_t first = 0, last = 255; // first and last active ray in the packet
	BVHNode* node = &bvhNode[0];
	ALIGNED( 64 ) uint32_t stack[64], stackPtr = 0;
	while (1)
	{
		if (node->isLeaf())
		{
			// handle leaf node
			for (uint32_t j = 0; j < node->triCount; j++)
			{
				const uint32_t idx = primIdx[node->leftFirst + j], vid = idx * 3;
				const bvhvec3 edge1 = verts[vid + 1] - verts[vid], edge2 = verts[vid + 2] - verts[vid];
				const bvhvec3 s = O - bvhvec3( verts[vid] );
				for (int32_t i = first; i <= last; i++)
				{
					Ray& ray = packet[i];
					const bvhvec3 h = tinybvh_cross( ray.D, edge2 );
					const float a = tinybvh_dot( edge1, h );
					if (fabs( a ) < 0.0000001f) continue; // ray parallel to triangle
					const float f = 1 / a, u = f * tinybvh_dot( s, h );
					if (u < 0 || u > 1) continue;
					const bvhvec3 q = tinybvh_cross( s, edge1 );
					const float v = f * tinybvh_dot( ray.D, q );
					if (v < 0 || u + v > 1) continue;
					const float t = f * tinybvh_dot( edge2, q );
					if (t <= 0 || t >= ray.hit.t) continue;
					ray.hit.t = t, ray.hit.u = u, ray.hit.v = v, ray.hit.prim = idx;
				}
			}
			if (stackPtr == 0) break; else // pop
				last = stack[--stackPtr], node = bvhNode + stack[--stackPtr],
				first = last >> 8, last &= 255;
		}
		else
		{
			// fetch pointers to child nodes
			BVHNode* left = bvhNode + node->leftFirst;
			BVHNode* right = bvhNode + node->leftFirst + 1;
			bool visitLeft = true, visitRight = true;
			int32_t leftFirst = first, leftLast = last, rightFirst = first, rightLast = last;
			float distLeft, distRight;
			{
				// see if we want to intersect the left child
				const __m128 minO4 = _mm_sub_ps( *(__m128*) & left->aabbMin, O4 );
				const __m128 maxO4 = _mm_sub_ps( *(__m128*) & left->aabbMax, O4 );
				// 1. Early-in test: if first ray hits the node, the packet visits the node
				bool earlyHit;
				{
					const __m128 rD4 = *(__m128*) & packet[first].rD;
					const __m128 st1 = _mm_mul_ps( _mm_and_ps( minO4, mask4 ), rD4 );
					const __m128 st2 = _mm_mul_ps( _mm_and_ps( maxO4, mask4 ), rD4 );
					const __m128 vmax4 = _mm_max_ps( st1, st2 ), vmin4 = _mm_min_ps( st1, st2 );
					const float tmax = tinybvh_min( LANE( vmax4, 0 ), tinybvh_min( LANE( vmax4, 1 ), LANE( vmax4, 2 ) ) );
					const float tmin = tinybvh_max( LANE( vmin4, 0 ), tinybvh_max( LANE( vmin4, 1 ), LANE( vmin4, 2 ) ) );
					earlyHit = (tmax >= tmin && tmin < packet[first].hit.t && tmax >= 0);
					distLeft = tmin;
				}
				// 2. Early-out test: if the node aabb is outside the four planes, we skip the node
				if (!earlyHit)
				{
					float* minmax = (float*)left;
					bvhvec3 c0( minmax[sign0x], minmax[sign0y], minmax[sign0z] );
					bvhvec3 c1( minmax[sign1x], minmax[sign1y], minmax[sign1z] );
					bvhvec3 c2( minmax[sign2x], minmax[sign2y], minmax[sign2z] );
					bvhvec3 c3( minmax[sign3x], minmax[sign3y], minmax[sign3z] );
					if (tinybvh_dot( c0, plane0 ) > t0 || tinybvh_dot( c1, plane1 ) > t1 ||
						tinybvh_dot( c2, plane2 ) > t2 || tinybvh_dot( c3, plane3 ) > t3)
						visitLeft = false;
					else
					{
						// 3. Last resort: update first and last, stay in node if first > last
						for (; leftFirst <= leftLast; leftFirst++)
						{
							const __m128 rD4 = *(__m128*) & packet[leftFirst].rD;
							const __m128 st1 = _mm_mul_ps( _mm_and_ps( minO4, mask4 ), rD4 );
							const __m128 st2 = _mm_mul_ps( _mm_and_ps( maxO4, mask4 ), rD4 );
							const __m128 vmax4 = _mm_max_ps( st1, st2 ), vmin4 = _mm_min_ps( st1, st2 );
							const float tmax = tinybvh_min( LANE( vmax4, 0 ), tinybvh_min( LANE( vmax4, 1 ), LANE( vmax4, 2 ) ) );
							const float tmin = tinybvh_max( LANE( vmin4, 0 ), tinybvh_max( LANE( vmin4, 1 ), LANE( vmin4, 2 ) ) );
							if (tmax >= tmin && tmin < packet[leftFirst].hit.t && tmax >= 0) { distLeft = tmin; break; }
						}
						for (; leftLast >= leftFirst; leftLast--)
						{
							const __m128 rD4 = *(__m128*) & packet[leftLast].rD;
							const __m128 st1 = _mm_mul_ps( _mm_and_ps( minO4, mask4 ), rD4 );
							const __m128 st2 = _mm_mul_ps( _mm_and_ps( maxO4, mask4 ), rD4 );
							const __m128 vmax4 = _mm_max_ps( st1, st2 ), vmin4 = _mm_min_ps( st1, st2 );
							const float tmax = tinybvh_min( LANE( vmax4, 0 ), tinybvh_min( LANE( vmax4, 1 ), LANE( vmax4, 2 ) ) );
							const float tmin = tinybvh_max( LANE( vmin4, 0 ), tinybvh_max( LANE( vmin4, 1 ), LANE( vmin4, 2 ) ) );
							if (tmax >= tmin && tmin < packet[leftLast].hit.t && tmax >= 0) break;
						}
						visitLeft = leftLast >= leftFirst;
					}
				}
			}
			{
				// see if we want to intersect the right child
				const __m128 minO4 = _mm_sub_ps( *(__m128*) & right->aabbMin, O4 );
				const __m128 maxO4 = _mm_sub_ps( *(__m128*) & right->aabbMax, O4 );
				// 1. Early-in test: if first ray hits the node, the packet visits the node
				bool earlyHit;
				{
					const __m128 rD4 = *(__m128*) & packet[first].rD;
					const __m128 st1 = _mm_mul_ps( minO4, rD4 ), st2 = _mm_mul_ps( maxO4, rD4 );
					const __m128 vmax4 = _mm_max_ps( st1, st2 ), vmin4 = _mm_min_ps( st1, st2 );
					const float tmax = tinybvh_min( LANE( vmax4, 0 ), tinybvh_min( LANE( vmax4, 1 ), LANE( vmax4, 2 ) ) );
					const float tmin = tinybvh_max( LANE( vmin4, 0 ), tinybvh_max( LANE( vmin4, 1 ), LANE( vmin4, 2 ) ) );
					earlyHit = (tmax >= tmin && tmin < packet[first].hit.t && tmax >= 0);
					distRight = tmin;
				}
				// 2. Early-out test: if the node aabb is outside the four planes, we skip the node
				if (!earlyHit)
				{
					float* minmax = (float*)right;
					bvhvec3 c0( minmax[sign0x], minmax[sign0y], minmax[sign0z] );
					bvhvec3 c1( minmax[sign1x], minmax[sign1y], minmax[sign1z] );
					bvhvec3 c2( minmax[sign2x], minmax[sign2y], minmax[sign2z] );
					bvhvec3 c3( minmax[sign3x], minmax[sign3y], minmax[sign3z] );
					if (tinybvh_dot( c0, plane0 ) > t0 || tinybvh_dot( c1, plane1 ) > t1 ||
						tinybvh_dot( c2, plane2 ) > t2 || tinybvh_dot( c3, plane3 ) > t3)
						visitRight = false;
					else
					{
						// 3. Last resort: update first and last, stay in node if first > last
						for (; rightFirst <= rightLast; rightFirst++)
						{
							const __m128 rD4 = *(__m128*) & packet[rightFirst].rD;
							const __m128 st1 = _mm_mul_ps( _mm_and_ps( minO4, mask4 ), rD4 );
							const __m128 st2 = _mm_mul_ps( _mm_and_ps( maxO4, mask4 ), rD4 );
							const __m128 vmax4 = _mm_max_ps( st1, st2 ), vmin4 = _mm_min_ps( st1, st2 );
							const float tmax1 = tinybvh_min( LANE( vmax4, 0 ), tinybvh_min( LANE( vmax4, 1 ), LANE( vmax4, 2 ) ) );
							const float tmin1 = tinybvh_max( LANE( vmin4, 0 ), tinybvh_max( LANE( vmin4, 1 ), LANE( vmin4, 2 ) ) );
							if (tmax1 >= tmin1 && tmin1 < packet[rightFirst].hit.t && tmax1 >= 0) { distRight = tmin1; break; }
						}
						for (; rightLast >= first; rightLast--)
						{
							const __m128 rD4 = *(__m128*) & packet[rightLast].rD;
							const __m128 st1 = _mm_mul_ps( _mm_and_ps( minO4, mask4 ), rD4 );
							const __m128 st2 = _mm_mul_ps( _mm_and_ps( maxO4, mask4 ), rD4 );
							const __m128 vmax4 = _mm_max_ps( st1, st2 ), vmin4 = _mm_min_ps( st1, st2 );
							const float tmax1 = tinybvh_min( LANE( vmax4, 0 ), tinybvh_min( LANE( vmax4, 1 ), LANE( vmax4, 2 ) ) );
							const float tmin1 = tinybvh_max( LANE( vmin4, 0 ), tinybvh_max( LANE( vmin4, 1 ), LANE( vmin4, 2 ) ) );
							if (tmax1 >= tmin1 && tmin1 < packet[rightLast].hit.t && tmax1 >= 0) break;
						}
						visitRight = rightLast >= rightFirst;
					}
				}
			}
			// process intersection result
			if (visitLeft && visitRight)
			{
				if (distLeft < distRight)
				{
					// push right, continue with left
					stack[stackPtr++] = node->leftFirst + 1;
					stack[stackPtr++] = (rightFirst << 8) + rightLast;
					node = left, first = leftFirst, last = leftLast;
				}
				else
				{
					// push left, continue with right
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
				last = stack[--stackPtr], node = bvhNode + stack[--stackPtr],
				first = last >> 8, last &= 255;
		}
	}
}

// Traverse the second alternative BVH layout (ALT_SOA).
int32_t BVH_SoA::Intersect( Ray& ray ) const
{
	BVHNode* node = &bvhNode[0], * stack[64];
	const bvhvec4slice& verts = bvh.verts;
	const uint32_t* primIdx = bvh.primIdx;
	uint32_t stackPtr = 0, cost = 0;
	const __m128 Ox4 = _mm_set1_ps( ray.O.x ), rDx4 = _mm_set1_ps( ray.rD.x );
	const __m128 Oy4 = _mm_set1_ps( ray.O.y ), rDy4 = _mm_set1_ps( ray.rD.y );
	const __m128 Oz4 = _mm_set1_ps( ray.O.z ), rDz4 = _mm_set1_ps( ray.rD.z );
	while (1)
	{
		cost += C_TRAV;
		if (node->isLeaf())
		{
			if (indexedEnabled && bvh.vertIdx != 0) for (uint32_t i = 0; i < node->triCount; i++, cost += C_INT)
				IntersectTriIndexed( ray, verts, bvh.vertIdx, primIdx[node->firstTri + i] );
			else if (customEnabled && bvh.customIntersect != 0) for (uint32_t i = 0; i < node->triCount; i++, cost += C_INT)
				(*bvh.customIntersect)(ray, primIdx[node->firstTri + i]);
			else for (uint32_t i = 0; i < node->triCount; i++, cost += C_INT)
				IntersectTri( ray, verts, primIdx[node->firstTri + i] );
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		__m128 x4 = _mm_mul_ps( _mm_sub_ps( node->xxxx, Ox4 ), rDx4 );
		__m128 y4 = _mm_mul_ps( _mm_sub_ps( node->yyyy, Oy4 ), rDy4 );
		__m128 z4 = _mm_mul_ps( _mm_sub_ps( node->zzzz, Oz4 ), rDz4 );
		// transpose
		__m128 t0 = _mm_unpacklo_ps( x4, y4 ), t2 = _mm_unpacklo_ps( z4, z4 );
		__m128 t1 = _mm_unpackhi_ps( x4, y4 ), t3 = _mm_unpackhi_ps( z4, z4 );
		__m128 xyzw1a = _mm_shuffle_ps( t0, t2, _MM_SHUFFLE( 1, 0, 1, 0 ) );
		__m128 xyzw2a = _mm_shuffle_ps( t0, t2, _MM_SHUFFLE( 3, 2, 3, 2 ) );
		__m128 xyzw1b = _mm_shuffle_ps( t1, t3, _MM_SHUFFLE( 1, 0, 1, 0 ) );
		__m128 xyzw2b = _mm_shuffle_ps( t1, t3, _MM_SHUFFLE( 3, 2, 3, 2 ) );
		// process
		__m128 tmina4 = _mm_min_ps( xyzw1a, xyzw2a ), tmaxa4 = _mm_max_ps( xyzw1a, xyzw2a );
		__m128 tminb4 = _mm_min_ps( xyzw1b, xyzw2b ), tmaxb4 = _mm_max_ps( xyzw1b, xyzw2b );
		// transpose back
		t0 = _mm_unpacklo_ps( tmina4, tmaxa4 ), t2 = _mm_unpacklo_ps( tminb4, tmaxb4 );
		t1 = _mm_unpackhi_ps( tmina4, tmaxa4 ), t3 = _mm_unpackhi_ps( tminb4, tmaxb4 );
		x4 = _mm_shuffle_ps( t0, t2, _MM_SHUFFLE( 1, 0, 1, 0 ) );
		y4 = _mm_shuffle_ps( t0, t2, _MM_SHUFFLE( 3, 2, 3, 2 ) );
		z4 = _mm_shuffle_ps( t1, t3, _MM_SHUFFLE( 1, 0, 1, 0 ) );
		uint32_t lidx = node->left, ridx = node->right;
		const __m128 min4 = _mm_max_ps( _mm_max_ps( _mm_max_ps( x4, y4 ), z4 ), _mm_setzero_ps() );
		const __m128 max4 = _mm_min_ps( _mm_min_ps( _mm_min_ps( x4, y4 ), z4 ), _mm_set1_ps( ray.hit.t ) );
	#if 0
		// TODO: why is this slower on gen14?
		const float tmina_0 = LANE( min4, 0 ), tmaxa_1 = LANE( max4, 1 );
		const float tminb_2 = LANE( min4, 2 ), tmaxb_3 = LANE( max4, 3 );
		t0 = _mm_shuffle_ps( max4, max4, _MM_SHUFFLE( 1, 3, 1, 3 ) );
		t1 = _mm_shuffle_ps( min4, min4, _MM_SHUFFLE( 0, 2, 0, 2 ) );
		t0 = _mm_blendv_ps( inf4, t1, _mm_cmpge_ps( t0, t1 ) );
		float dist1 = LANE( t0, 1 ), dist2 = LANE( t0, 0 );
	#else
		const float tmina_0 = LANE( min4, 0 ), tmaxa_1 = LANE( max4, 1 );
		const float tminb_2 = LANE( min4, 2 ), tmaxb_3 = LANE( max4, 3 );
		float dist1 = tmaxa_1 >= tmina_0 ? tmina_0 : BVH_FAR;
		float dist2 = tmaxb_3 >= tminb_2 ? tminb_2 : BVH_FAR;
	#endif
		if (dist1 > dist2)
		{
			float t = dist1; dist1 = dist2; dist2 = t;
			uint32_t i = lidx; lidx = ridx; ridx = i;
		}
		if (dist1 == BVH_FAR)
		{
			if (stackPtr == 0) break; else node = stack[--stackPtr];
		}
		else
		{
			node = bvhNode + lidx;
			if (dist2 != BVH_FAR) stack[stackPtr++] = bvhNode + ridx;
		}
	}
	return cost;
}

// Find occlusions in the second alternative BVH layout (ALT_SOA).
bool BVH_SoA::IsOccluded( const Ray& ray ) const
{
	BVHNode* node = &bvhNode[0], * stack[64];
	const bvhvec4slice& verts = bvh.verts;
	const uint32_t* primIdx = bvh.primIdx;
	uint32_t stackPtr = 0;
	const __m128 Ox4 = _mm_set1_ps( ray.O.x ), rDx4 = _mm_set1_ps( ray.rD.x );
	const __m128 Oy4 = _mm_set1_ps( ray.O.y ), rDy4 = _mm_set1_ps( ray.rD.y );
	const __m128 Oz4 = _mm_set1_ps( ray.O.z ), rDz4 = _mm_set1_ps( ray.rD.z );
	while (1)
	{
		if (node->isLeaf())
		{
			if (indexedEnabled && bvh.vertIdx != 0)
			{
				for (uint32_t i = 0; i < node->triCount; i++)
					if (IndexedTriOccludes( ray, verts, bvh.vertIdx, primIdx[node->firstTri + i] )) return true;
			}
			else if (customEnabled && bvh.customIsOccluded != 0)
			{
				for (uint32_t i = 0; i < node->triCount; i++)
					if ((*bvh.customIsOccluded)(ray, primIdx[node->firstTri + i])) return true;
			}
			else
			{
				for (uint32_t i = 0; i < node->triCount; i++)
					if (TriOccludes( ray, verts, primIdx[node->firstTri + i] )) return true;
			}
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		__m128 x4 = _mm_mul_ps( _mm_sub_ps( node->xxxx, Ox4 ), rDx4 );
		__m128 y4 = _mm_mul_ps( _mm_sub_ps( node->yyyy, Oy4 ), rDy4 );
		__m128 z4 = _mm_mul_ps( _mm_sub_ps( node->zzzz, Oz4 ), rDz4 );
		// transpose
		__m128 t0 = _mm_unpacklo_ps( x4, y4 ), t2 = _mm_unpacklo_ps( z4, z4 );
		__m128 t1 = _mm_unpackhi_ps( x4, y4 ), t3 = _mm_unpackhi_ps( z4, z4 );
		__m128 xyzw1a = _mm_shuffle_ps( t0, t2, _MM_SHUFFLE( 1, 0, 1, 0 ) );
		__m128 xyzw2a = _mm_shuffle_ps( t0, t2, _MM_SHUFFLE( 3, 2, 3, 2 ) );
		__m128 xyzw1b = _mm_shuffle_ps( t1, t3, _MM_SHUFFLE( 1, 0, 1, 0 ) );
		__m128 xyzw2b = _mm_shuffle_ps( t1, t3, _MM_SHUFFLE( 3, 2, 3, 2 ) );
		// process
		__m128 tmina4 = _mm_min_ps( xyzw1a, xyzw2a ), tmaxa4 = _mm_max_ps( xyzw1a, xyzw2a );
		__m128 tminb4 = _mm_min_ps( xyzw1b, xyzw2b ), tmaxb4 = _mm_max_ps( xyzw1b, xyzw2b );
		// transpose back
		t0 = _mm_unpacklo_ps( tmina4, tmaxa4 ), t2 = _mm_unpacklo_ps( tminb4, tmaxb4 );
		t1 = _mm_unpackhi_ps( tmina4, tmaxa4 ), t3 = _mm_unpackhi_ps( tminb4, tmaxb4 );
		x4 = _mm_shuffle_ps( t0, t2, _MM_SHUFFLE( 1, 0, 1, 0 ) );
		y4 = _mm_shuffle_ps( t0, t2, _MM_SHUFFLE( 3, 2, 3, 2 ) );
		z4 = _mm_shuffle_ps( t1, t3, _MM_SHUFFLE( 1, 0, 1, 0 ) );
		uint32_t lidx = node->left, ridx = node->right;
		const __m128 min4 = _mm_max_ps( _mm_max_ps( _mm_max_ps( x4, y4 ), z4 ), _mm_setzero_ps() );
		const __m128 max4 = _mm_min_ps( _mm_min_ps( _mm_min_ps( x4, y4 ), z4 ), _mm_set1_ps( ray.hit.t ) );
		const float tmina_0 = LANE( min4, 0 ), tmaxa_1 = LANE( max4, 1 );
		const float tminb_2 = LANE( min4, 2 ), tmaxb_3 = LANE( max4, 3 );
		float dist1 = tmaxa_1 >= tmina_0 ? tmina_0 : BVH_FAR;
		float dist2 = tmaxb_3 >= tminb_2 ? tminb_2 : BVH_FAR;
		if (dist1 > dist2)
		{
			float t = dist1; dist1 = dist2; dist2 = t;
			uint32_t i = lidx; lidx = ridx; ridx = i;
		}
		if (dist1 == BVH_FAR)
		{
			if (stackPtr == 0) break; else node = stack[--stackPtr];
		}
		else
		{
			node = bvhNode + lidx;
			if (dist2 != BVH_FAR) stack[stackPtr++] = bvhNode + ridx;
		}
	}
	return false;
}

// Intersect_CWBVH:
// Intersect a compressed 8-wide BVH with a ray. For debugging only, not efficient.
// Not technically limited to BVH_USEAVX, but __lzcnt and __popcnt will require
// exotic compiler flags (in combination with __builtin_ia32_lzcnt_u32), so... Since
// this is just here to test data before it goes to the GPU: MSVC-only for now.
static uint32_t __popc( uint32_t x )
{
#if defined(_MSC_VER) && !defined(__clang__)
	return __popcnt( x );
#elif defined(__GNUC__) || defined(__clang__)
	return __builtin_popcount( x );
#endif
}
#define STACK_POP() { ngroup = traversalStack[--stackPtr]; }
#define STACK_PUSH() { traversalStack[stackPtr++] = ngroup; }
inline uint32_t extract_byte( const uint32_t i, const uint32_t n ) { return (i >> (n * 8)) & 0xFF; }
inline uint32_t sign_extend_s8x4( const uint32_t i )
{
	// asm("prmt.b32 %0, %1, 0x0, 0x0000BA98;" : "=r"(v) : "r"(i)); // BA98: 1011`1010`1001`1000
	// with the given parameters, prmt will extend the sign to all bits in a byte.
	uint32_t b0 = (i & 0b10000000000000000000000000000000) ? 0xff000000 : 0;
	uint32_t b1 = (i & 0b00000000100000000000000000000000) ? 0x00ff0000 : 0;
	uint32_t b2 = (i & 0b00000000000000001000000000000000) ? 0x0000ff00 : 0;
	uint32_t b3 = (i & 0b00000000000000000000000010000000) ? 0x000000ff : 0;
	return b0 + b1 + b2 + b3; // probably can do better than this.
}
int32_t BVH8_CWBVH::Intersect( Ray& ray ) const
{
	bvhuint2 traversalStack[128];
	uint32_t hitAddr = 0, stackPtr = 0;
	bvhvec2 triangleuv( 0, 0 );
	const bvhvec4* blasNodes = bvh8Data;
	const bvhvec4* blasTris = bvh8Tris;
	float tmin = 0, tmax = ray.hit.t;
	const uint32_t octinv = (7 - ((ray.D.x < 0 ? 4 : 0) | (ray.D.y < 0 ? 2 : 0) | (ray.D.z < 0 ? 1 : 0))) * 0x1010101;
	bvhuint2 ngroup = bvhuint2( 0, 0b10000000000000000000000000000000 ), tgroup = bvhuint2( 0 );
	do
	{
		if (ngroup.y > 0x00FFFFFF)
		{
			const uint32_t hits = ngroup.y, imask = ngroup.y;
			const uint32_t child_bit_index = __bfind( hits );
			const uint32_t child_node_base_index = ngroup.x;
			ngroup.y &= ~(1 << child_bit_index);
			if (ngroup.y > 0x00FFFFFF) { STACK_PUSH( /* nodeGroup */ ); }
			{
				const uint32_t slot_index = (child_bit_index - 24) ^ (octinv & 255);
				const uint32_t relative_index = __popc( imask & ~(0xFFFFFFFF << slot_index) );
				const uint32_t child_node_index = child_node_base_index + relative_index;
				const bvhvec4 n0 = blasNodes[child_node_index * 5 + 0], n1 = blasNodes[child_node_index * 5 + 1];
				const bvhvec4 n2 = blasNodes[child_node_index * 5 + 2], n3 = blasNodes[child_node_index * 5 + 3];
				const bvhvec4 n4 = blasNodes[child_node_index * 5 + 4], p = n0;
				bvhint3 e;
				e.x = (int32_t) * ((int8_t*)&n0.w + 0), e.y = (int32_t) * ((int8_t*)&n0.w + 1), e.z = (int32_t) * ((int8_t*)&n0.w + 2);
				ngroup.x = as_uint( n1.x ), tgroup.x = as_uint( n1.y ), tgroup.y = 0;
				uint32_t hitmask = 0;
				const uint32_t vx = (e.x + 127) << 23u; const float adjusted_idirx = *(float*)&vx * ray.rD.x;
				const uint32_t vy = (e.y + 127) << 23u; const float adjusted_idiry = *(float*)&vy * ray.rD.y;
				const uint32_t vz = (e.z + 127) << 23u; const float adjusted_idirz = *(float*)&vz * ray.rD.z;
				const float origx = -(ray.O.x - p.x) * ray.rD.x;
				const float origy = -(ray.O.y - p.y) * ray.rD.y;
				const float origz = -(ray.O.z - p.z) * ray.rD.z;
				{	// First 4
					const uint32_t meta4 = *(uint32_t*)&n1.z, is_inner4 = (meta4 & (meta4 << 1)) & 0x10101010;
					const uint32_t inner_mask4 = sign_extend_s8x4( is_inner4 << 3 );
					const uint32_t bit_index4 = (meta4 ^ (octinv & inner_mask4)) & 0x1F1F1F1F;
					const uint32_t child_bits4 = (meta4 >> 5) & 0x07070707;
					uint32_t swizzledLox = (ray.rD.x < 0) ? *(uint32_t*)&n3.z : *(uint32_t*)&n2.x, swizzledHix = (ray.rD.x < 0) ? *(uint32_t*)&n2.x : *(uint32_t*)&n3.z;
					uint32_t swizzledLoy = (ray.rD.y < 0) ? *(uint32_t*)&n4.x : *(uint32_t*)&n2.z, swizzledHiy = (ray.rD.y < 0) ? *(uint32_t*)&n2.z : *(uint32_t*)&n4.x;
					uint32_t swizzledLoz = (ray.rD.z < 0) ? *(uint32_t*)&n4.z : *(uint32_t*)&n3.x, swizzledHiz = (ray.rD.z < 0) ? *(uint32_t*)&n3.x : *(uint32_t*)&n4.z;
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
					const uint32_t meta4 = *(uint32_t*)&n1.w, is_inner4 = (meta4 & (meta4 << 1)) & 0x10101010;
					const uint32_t inner_mask4 = sign_extend_s8x4( is_inner4 << 3 );
					const uint32_t bit_index4 = (meta4 ^ (octinv & inner_mask4)) & 0x1F1F1F1F;
					const uint32_t child_bits4 = (meta4 >> 5) & 0x07070707;
					uint32_t swizzledLox = (ray.rD.x < 0) ? *(uint32_t*)&n3.w : *(uint32_t*)&n2.y, swizzledHix = (ray.rD.x < 0) ? *(uint32_t*)&n2.y : *(uint32_t*)&n3.w;
					uint32_t swizzledLoy = (ray.rD.y < 0) ? *(uint32_t*)&n4.y : *(uint32_t*)&n2.w, swizzledHiy = (ray.rD.y < 0) ? *(uint32_t*)&n2.w : *(uint32_t*)&n4.y;
					uint32_t swizzledLoz = (ray.rD.z < 0) ? *(uint32_t*)&n4.w : *(uint32_t*)&n3.y, swizzledHiz = (ray.rD.z < 0) ? *(uint32_t*)&n3.y : *(uint32_t*)&n4.w;
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
			uint32_t triangleIndex = __bfind( tgroup.y );
		#ifdef CWBVH_COMPRESSED_TRIS
			const float* T = (float*)&blasTris[tgroup.x + triangleIndex * 4];
			const float transS = T[8] * ray.O.x + T[9] * ray.O.y + T[10] * ray.O.z + T[11];
			const float transD = T[8] * ray.D.x + T[9] * ray.D.y + T[10] * ray.D.z;
			const float ta = -transS / transD;
			if (ta > 0 && ta < ray.hit.t)
			{
				const bvhvec3 wr = ray.O + ta * ray.D;
				const float u = T[0] * wr.x + T[1] * wr.y + T[2] * wr.z + T[3];
				const float v = T[4] * wr.x + T[5] * wr.y + T[6] * wr.z + T[7];
				const bool hit = u >= 0 && v >= 0 && u + v < 1;
				if (hit) triangleuv = bvhvec2( u, v ), tmax = ta, hitAddr = *(uint32_t*)&T[15];
			}
		#else
			int32_t triAddr = tgroup.x + triangleIndex * 3;
			const bvhvec3 edge2 = bvhvec3( blasTris[triAddr + 0] );
			const bvhvec3 edge1 = bvhvec3( blasTris[triAddr + 1] );
			const bvhvec3 v0 = blasTris[triAddr + 2];
			const bvhvec3 h = tinybvh_cross( ray.D, edge2 );
			const float a = tinybvh_dot( edge1, h );
			if (fabs( a ) > 0.0000001f)
			{
				const float f = 1 / a;
				const bvhvec3 s = ray.O - v0;
				const float u = f * tinybvh_dot( s, h );
				if (u >= 0 && u <= 1)
				{
					const bvhvec3 q = tinybvh_cross( s, edge1 );
					const float v = f * tinybvh_dot( ray.D, q );
					if (v >= 0 && u + v <= 1)
					{
						const float d = f * tinybvh_dot( edge2, q );
						if (d > 0.0f && d < tmax)
						{
							triangleuv = bvhvec2( u, v ), tmax = d;
							hitAddr = as_uint( blasTris[triAddr + 2].w );
						}
					}
				}
			}
		#endif
			tgroup.y -= 1 << triangleIndex;
		}
		if (ngroup.y <= 0x00FFFFFF)
		{
			if (stackPtr > 0) { STACK_POP( /* nodeGroup */ ); }
			else
			{
				ray.hit.t = tmax;
				if (tmax < BVH_FAR)
					ray.hit.u = triangleuv.x, ray.hit.v = triangleuv.y;
				ray.hit.prim = hitAddr;
				break;
			}
		}
	} while (true);
	return 0;
}

// Traverse a 4-way BVH stored in 'Atilla Áfra' layout.
inline void IntersectCompactTri( Ray& r, __m128& t4, const float* T )
{
	const float transS = T[8] * r.O.x + T[9] * r.O.y + T[10] * r.O.z + T[11];
	const float transD = T[8] * r.D.x + T[9] * r.D.y + T[10] * r.D.z;
	const float ta = -transS / transD;
	if (ta <= 0 || ta >= r.hit.t) return;
	const bvhvec3 wr = r.O + ta * r.D;
	const float u = T[0] * wr.x + T[1] * wr.y + T[2] * wr.z + T[3];
	const float v = T[4] * wr.x + T[5] * wr.y + T[6] * wr.z + T[7];
	const bool hit = u >= 0 && v >= 0 && u + v < 1;
#if INST_IDX_BITS == 32
	if (hit) r.hit = { r.instIdx, ta, u, v, *(uint32_t*)&T[15] }, t4 = _mm_set1_ps( ta );
#else
	if (hit) r.hit = { ta, u, v, *(uint32_t*)&T[15] + r.instIdx }, t4 = _mm_set1_ps( ta );
#endif
}
int32_t BVH4_CPU::Intersect( Ray& ray ) const
{
	uint32_t nodeIdx = 0, stack[1024], stackPtr = 0, cost = 0;
	const __m128 ox4 = _mm_set1_ps( ray.O.x ), rdx4 = _mm_set1_ps( ray.rD.x );
	const __m128 oy4 = _mm_set1_ps( ray.O.y ), rdy4 = _mm_set1_ps( ray.rD.y );
	const __m128 oz4 = _mm_set1_ps( ray.O.z ), rdz4 = _mm_set1_ps( ray.rD.z );
	__m128 t4 = _mm_set1_ps( ray.hit.t ), zero4 = _mm_setzero_ps();
	const __m128 idx4 = _mm_castsi128_ps( _mm_setr_epi32( 0, 1, 2, 3 ) );
	const __m128 idxMask = _mm_castsi128_ps( _mm_set1_epi32( 0xfffffffc ) );
	const __m128 inf4 = _mm_set1_ps( BVH_FAR );
	while (1)
	{
		cost += C_TRAV;
		const BVHNode& node = bvh4Node[nodeIdx];
		// intersect the ray with four AABBs
		const __m128 xmin4 = node.xmin4, xmax4 = node.xmax4;
		const __m128 ymin4 = node.ymin4, ymax4 = node.ymax4;
		const __m128 zmin4 = node.zmin4, zmax4 = node.zmax4;
		const __m128 x0 = _mm_sub_ps( xmin4, ox4 ), x1 = _mm_sub_ps( xmax4, ox4 );
		const __m128 y0 = _mm_sub_ps( ymin4, oy4 ), y1 = _mm_sub_ps( ymax4, oy4 );
		const __m128 z0 = _mm_sub_ps( zmin4, oz4 ), z1 = _mm_sub_ps( zmax4, oz4 );
		const __m128 tx1 = _mm_mul_ps( x0, rdx4 ), tx2 = _mm_mul_ps( x1, rdx4 );
		const __m128 ty1 = _mm_mul_ps( y0, rdy4 ), ty2 = _mm_mul_ps( y1, rdy4 );
		const __m128 tz1 = _mm_mul_ps( z0, rdz4 ), tz2 = _mm_mul_ps( z1, rdz4 );
		const __m128 txmin = _mm_min_ps( tx1, tx2 ), tymin = _mm_min_ps( ty1, ty2 ), tzmin = _mm_min_ps( tz1, tz2 );
		const __m128 txmax = _mm_max_ps( tx1, tx2 ), tymax = _mm_max_ps( ty1, ty2 ), tzmax = _mm_max_ps( tz1, tz2 );
		const __m128 tmin = _mm_max_ps( _mm_max_ps( txmin, tymin ), tzmin );
		const __m128 tmax = _mm_min_ps( _mm_min_ps( txmax, tymax ), tzmax );
		const __m128 hit = _mm_and_ps( _mm_and_ps( _mm_cmpge_ps( tmax, tmin ), _mm_cmplt_ps( tmin, t4 ) ), _mm_cmpge_ps( tmax, zero4 ) );
		const int32_t hitBits = _mm_movemask_ps( hit ), hits = __popc( hitBits );
		if (hits == 1 /* 43% */)
		{
			// just one node was hit - no sorting needed.
			const uint32_t lane = __bfind( hitBits ), count = node.triCount[lane];
			if (count == 0) nodeIdx = node.childFirst[lane]; else
			{
				const uint32_t first = node.childFirst[lane];
				for (uint32_t j = 0; j < count; j++, cost += C_INT) // TODO: aim for 4 prims per leaf
					IntersectCompactTri( ray, t4, (float*)(bvh4Tris + first + j * 4) );
				if (stackPtr == 0) break;
				nodeIdx = stack[--stackPtr];
			}
			continue;
		}
		if (hits == 0 /* 29% */)
		{
			if (stackPtr == 0) break;
			nodeIdx = stack[--stackPtr];
			continue;
		}
		if (hits == 2 /* 16% */)
		{
			// two nodes hit
			uint32_t lane0 = __bfind( hitBits ), lane1 = __bfind( hitBits - (1 << lane0) );
			float dist0 = ((float*)&tmin)[lane0], dist1 = ((float*)&tmin)[lane1];
			if (dist1 < dist0)
			{
				uint32_t t = lane0; lane0 = lane1; lane1 = t;
				float ft = dist0; dist0 = dist1; dist1 = ft;
			}
			const uint32_t triCount0 = node.triCount[lane0], triCount1 = node.triCount[lane1];
			// process first lane
			if (triCount0 == 0) nodeIdx = node.childFirst[lane0]; else
			{
				const uint32_t first = node.childFirst[lane0];
				for (uint32_t j = 0; j < triCount0; j++, cost += C_INT) // TODO: aim for 4 prims per leaf
					IntersectCompactTri( ray, t4, (float*)(bvh4Tris + first + j * 4) );
				nodeIdx = 0;
			}
			// process second lane
			if (triCount1 == 0)
			{
				if (nodeIdx) stack[stackPtr++] = nodeIdx;
				nodeIdx = node.childFirst[lane1];
			}
			else
			{
				const uint32_t first = node.childFirst[lane1];
				for (uint32_t j = 0; j < triCount1; j++, cost += C_INT) // TODO: aim for 4 prims per leaf
					IntersectCompactTri( ray, t4, (float*)(bvh4Tris + first + j * 4) );
			}
		}
		else if (hits == 3 /* 8% */)
		{
			// blend in lane indices
			__m128 tm = _mm_or_ps( _mm_and_ps( _mm_blendv_ps( inf4, tmin, hit ), idxMask ), idx4 );
			// sort
			float tmp, d0 = LANE( tm, 0 ), d1 = LANE( tm, 1 ), d2 = LANE( tm, 2 ), d3 = LANE( tm, 3 );
			if (d0 < d2) tmp = d0, d0 = d2, d2 = tmp;
			if (d1 < d3) tmp = d1, d1 = d3, d3 = tmp;
			if (d0 < d1) tmp = d0, d0 = d1, d1 = tmp;
			if (d2 < d3) tmp = d2, d2 = d3, d3 = tmp;
			if (d1 < d2) tmp = d1, d1 = d2, d2 = tmp;
			// process hits
			float d[4] = { d0, d1, d2, d3 };
			nodeIdx = 0;
			for (int32_t i = 1; i < 4; i++)
			{
				uint32_t lane = *(uint32_t*)&d[i] & 3;
				if (node.triCount[lane] == 0)
				{
					const uint32_t childIdx = node.childFirst[lane];
					if (nodeIdx) stack[stackPtr++] = nodeIdx;
					nodeIdx = childIdx;
					continue;
				}
				const uint32_t first = node.childFirst[lane], count = node.triCount[lane];
				for (uint32_t j = 0; j < count; j++, cost += C_INT) // TODO: aim for 4 prims per leaf
					IntersectCompactTri( ray, t4, (float*)(bvh4Tris + first + j * 4) );
			}
		}
		else /* hits == 4, 2%: rare */
		{
			// blend in lane indices
			__m128 tm = _mm_or_ps( _mm_and_ps( _mm_blendv_ps( inf4, tmin, hit ), idxMask ), idx4 );
			// sort
			float tmp, d0 = LANE( tm, 0 ), d1 = LANE( tm, 1 ), d2 = LANE( tm, 2 ), d3 = LANE( tm, 3 );
			if (d0 < d2) tmp = d0, d0 = d2, d2 = tmp;
			if (d1 < d3) tmp = d1, d1 = d3, d3 = tmp;
			if (d0 < d1) tmp = d0, d0 = d1, d1 = tmp;
			if (d2 < d3) tmp = d2, d2 = d3, d3 = tmp;
			if (d1 < d2) tmp = d1, d1 = d2, d2 = tmp;
			// process hits
			float d[4] = { d0, d1, d2, d3 };
			nodeIdx = 0;
			for (int32_t i = 0; i < 4; i++)
			{
				uint32_t lane = *(uint32_t*)&d[i] & 3;
				if (node.triCount[lane] + node.childFirst[lane] == 0) continue; // TODO - never happens?
				if (node.triCount[lane] == 0)
				{
					const uint32_t childIdx = node.childFirst[lane];
					if (nodeIdx) stack[stackPtr++] = nodeIdx;
					nodeIdx = childIdx;
					continue;
				}
				const uint32_t first = node.childFirst[lane], count = node.triCount[lane];
				for (uint32_t j = 0; j < count; j++, cost += C_INT) // TODO: aim for 4 prims per leaf
					IntersectCompactTri( ray, t4, (float*)(bvh4Tris + first + j * 4) );
			}
		}
		// get next task
		if (nodeIdx) continue;
		if (stackPtr == 0) break; else nodeIdx = stack[--stackPtr];
	}
	return cost;
}

// Find occlusions in a 4-way BVH stored in 'Atilla Áfra' layout.
inline bool OccludedCompactTri( const Ray& r, const float* T )
{
	const float transS = T[8] * r.O.x + T[9] * r.O.y + T[10] * r.O.z + T[11];
	const float transD = T[8] * r.D.x + T[9] * r.D.y + T[10] * r.D.z;
	const float ta = -transS / transD;
	if (ta <= 0 || ta >= r.hit.t) return false;
	const bvhvec3 wr = r.O + ta * r.D;
	const float u = T[0] * wr.x + T[1] * wr.y + T[2] * wr.z + T[3];
	const float v = T[4] * wr.x + T[5] * wr.y + T[6] * wr.z + T[7];
	return u >= 0 && v >= 0 && u + v < 1;
}
#ifdef __GNUC__
#pragma GCC push_options
#pragma GCC optimize ("-O1") // TODO: I must be doing something wrong, figure out what.
#endif
bool BVH4_CPU::IsOccluded( const Ray& ray ) const
{
	uint32_t nodeIdx = 0, stack[1024], stackPtr = 0;
	const __m128 ox4 = _mm_set1_ps( ray.O.x ), rdx4 = _mm_set1_ps( ray.rD.x );
	const __m128 oy4 = _mm_set1_ps( ray.O.y ), rdy4 = _mm_set1_ps( ray.rD.y );
	const __m128 oz4 = _mm_set1_ps( ray.O.z ), rdz4 = _mm_set1_ps( ray.rD.z );
	__m128 t4 = _mm_set1_ps( ray.hit.t ), zero4 = _mm_setzero_ps();
	const __m128 idx4 = _mm_castsi128_ps( _mm_setr_epi32( 0, 1, 2, 3 ) );
	const __m128 idxMask = _mm_castsi128_ps( _mm_set1_epi32( 0xfffffffc ) );
	const __m128 inf4 = _mm_set1_ps( BVH_FAR );
	while (1)
	{
		const BVHNode& node = bvh4Node[nodeIdx];
		// intersect the ray with four AABBs
		const __m128 xmin4 = node.xmin4, xmax4 = node.xmax4;
		const __m128 ymin4 = node.ymin4, ymax4 = node.ymax4;
		const __m128 zmin4 = node.zmin4, zmax4 = node.zmax4;
		const __m128 x0 = _mm_sub_ps( xmin4, ox4 ), x1 = _mm_sub_ps( xmax4, ox4 );
		const __m128 y0 = _mm_sub_ps( ymin4, oy4 ), y1 = _mm_sub_ps( ymax4, oy4 );
		const __m128 z0 = _mm_sub_ps( zmin4, oz4 ), z1 = _mm_sub_ps( zmax4, oz4 );
		const __m128 tx1 = _mm_mul_ps( x0, rdx4 ), tx2 = _mm_mul_ps( x1, rdx4 );
		const __m128 ty1 = _mm_mul_ps( y0, rdy4 ), ty2 = _mm_mul_ps( y1, rdy4 );
		const __m128 tz1 = _mm_mul_ps( z0, rdz4 ), tz2 = _mm_mul_ps( z1, rdz4 );
		const __m128 txmin = _mm_min_ps( tx1, tx2 ), tymin = _mm_min_ps( ty1, ty2 ), tzmin = _mm_min_ps( tz1, tz2 );
		const __m128 txmax = _mm_max_ps( tx1, tx2 ), tymax = _mm_max_ps( ty1, ty2 ), tzmax = _mm_max_ps( tz1, tz2 );
		const __m128 tmin = _mm_max_ps( _mm_max_ps( txmin, tymin ), tzmin );
		const __m128 tmax = _mm_min_ps( _mm_min_ps( txmax, tymax ), tzmax );
		const __m128 hit = _mm_and_ps( _mm_and_ps( _mm_cmpge_ps( tmax, tmin ), _mm_cmplt_ps( tmin, t4 ) ), _mm_cmpge_ps( tmax, zero4 ) );
		const int32_t hitBits = _mm_movemask_ps( hit ), hits = __popc( hitBits );
		if (hits == 1 /* 43% */)
		{
			// just one node was hit - no sorting needed.
			const uint32_t lane = __bfind( hitBits ), count = node.triCount[lane];
			if (count == 0) nodeIdx = node.childFirst[lane]; else
			{
				const uint32_t first = node.childFirst[lane];
				for (uint32_t j = 0; j < count; j++) // TODO: aim for 4 prims per leaf
					if (OccludedCompactTri( ray, (float*)(bvh4Tris + first + j * 4) )) return true;
				if (stackPtr == 0) break;
				nodeIdx = stack[--stackPtr];
			}
			continue;
		}
		if (hits == 0 /* 29% */)
		{
			if (stackPtr == 0) break;
			nodeIdx = stack[--stackPtr];
			continue;
		}
		if (hits == 2 /* 16% */)
		{
			// two nodes hit
			uint32_t lane0 = __bfind( hitBits ), lane1 = __bfind( hitBits - (1 << lane0) );
			float dist0 = ((float*)&tmin)[lane0], dist1 = ((float*)&tmin)[lane1];
			if (dist1 < dist0)
			{
				uint32_t t = lane0; lane0 = lane1; lane1 = t;
				float ft = dist0; dist0 = dist1; dist1 = ft;
			}
			const uint32_t triCount0 = node.triCount[lane0], triCount1 = node.triCount[lane1];
			// process first lane
			if (triCount0 == 0) nodeIdx = node.childFirst[lane0]; else
			{
				const uint32_t first = node.childFirst[lane0];
				for (uint32_t j = 0; j < triCount0; j++) // TODO: aim for 4 prims per leaf
					if (OccludedCompactTri( ray, (float*)(bvh4Tris + first + j * 4) )) return true;
				nodeIdx = 0;
			}
			// process second lane
			if (triCount1 == 0)
			{
				if (nodeIdx) stack[stackPtr++] = nodeIdx;
				nodeIdx = node.childFirst[lane1];
			}
			else
			{
				const uint32_t first = node.childFirst[lane1];
				for (uint32_t j = 0; j < triCount1; j++) // TODO: aim for 4 prims per leaf
					if (OccludedCompactTri( ray, (float*)(bvh4Tris + first + j * 4) )) return true;
			}
		}
		else if (hits == 3 /* 8% */)
		{
			// blend in lane indices
			__m128 tm = _mm_or_ps( _mm_and_ps( _mm_blendv_ps( inf4, tmin, hit ), idxMask ), idx4 );
			// sort
			float tmp, d0 = LANE( tm, 0 ), d1 = LANE( tm, 1 ), d2 = LANE( tm, 2 ), d3 = LANE( tm, 3 );
			if (d0 < d2) tmp = d0, d0 = d2, d2 = tmp;
			if (d1 < d3) tmp = d1, d1 = d3, d3 = tmp;
			if (d0 < d1) tmp = d0, d0 = d1, d1 = tmp;
			if (d2 < d3) tmp = d2, d2 = d3, d3 = tmp;
			if (d1 < d2) tmp = d1, d1 = d2, d2 = tmp;
			// process hits
			float d[4] = { d0, d1, d2, d3 };
			nodeIdx = 0;
			for (int32_t i = 1; i < 4; i++)
			{
				uint32_t lane = *(uint32_t*)&d[i] & 3;
				if (node.triCount[lane] == 0)
				{
					const uint32_t childIdx = node.childFirst[lane];
					if (nodeIdx) stack[stackPtr++] = nodeIdx;
					nodeIdx = childIdx;
					continue;
				}
				const uint32_t first = node.childFirst[lane], count = node.triCount[lane];
				for (uint32_t j = 0; j < count; j++) // TODO: aim for 4 prims per leaf
					if (OccludedCompactTri( ray, (float*)(bvh4Tris + first + j * 4) )) return true;
			}
		}
		else /* hits == 4, 2%: rare */
		{
			// blend in lane indices
			__m128 tm = _mm_or_ps( _mm_and_ps( _mm_blendv_ps( inf4, tmin, hit ), idxMask ), idx4 );
			// sort
			float tmp, d0 = LANE( tm, 0 ), d1 = LANE( tm, 1 ), d2 = LANE( tm, 2 ), d3 = LANE( tm, 3 );
			if (d0 < d2) tmp = d0, d0 = d2, d2 = tmp;
			if (d1 < d3) tmp = d1, d1 = d3, d3 = tmp;
			if (d0 < d1) tmp = d0, d0 = d1, d1 = tmp;
			if (d2 < d3) tmp = d2, d2 = d3, d3 = tmp;
			if (d1 < d2) tmp = d1, d1 = d2, d2 = tmp;
			// process hits
			float d[4] = { d0, d1, d2, d3 };
			nodeIdx = 0;
			for (int32_t i = 0; i < 4; i++)
			{
				uint32_t lane = *(uint32_t*)&d[i] & 3;
				if (node.triCount[lane] + node.childFirst[lane] == 0) continue; // TODO - never happens?
				if (node.triCount[lane] == 0)
				{
					const uint32_t childIdx = node.childFirst[lane];
					if (nodeIdx) stack[stackPtr++] = nodeIdx;
					nodeIdx = childIdx;
					continue;
				}
				const uint32_t first = node.childFirst[lane], count = node.triCount[lane];
				for (uint32_t j = 0; j < count; j++) // TODO: aim for 4 prims per leaf
					if (OccludedCompactTri( ray, (float*)(bvh4Tris + first + j * 4) )) return true;
			}
		}
		// get next task
		if (nodeIdx) continue;
		if (stackPtr == 0) break; else nodeIdx = stack[--stackPtr];
	}
	return false;
}
#ifdef __GNUC__
#pragma GCC pop_options
#endif

#if 0 // under construction

uint32_t signShiftAmount( const bool positiveX, bool positiveY, bool positiveZ )
{
	return ((positiveX ? 0b001 : 0u) | (positiveY ? 0b010 : 0u) | (positiveZ ? 0b100 : 0u)) * 3;
}

int32_t BVH4_WiVe::Intersect( Ray& ray ) const
{
	__m256 ox8 = _mm256_set1_ps( ray.O.x ), rdx8 = _mm256_set1_ps( ray.rD.x );
	__m256 oy8 = _mm256_set1_ps( ray.O.y ), rdy8 = _mm256_set1_ps( ray.rD.y );
	__m256 oz8 = _mm256_set1_ps( ray.O.z ), rdz8 = _mm256_set1_ps( ray.rD.z );
	__m256 t8 = _mm256_set1_ps( ray.hit.t );
	__m256i signShift8 = _mm256_set1_epi32( (ray.D.x > 0 ? 3 : 0) + (ray.D.y > 0 ? 6 : 0) + (ray.D.z > 0 ? 12 : 0) );
	struct StackEntry { uint32_t node; float dist; };
	ALIGNED( 64 ) StackEntry stack[64];
	// std::fill( std::begin( stackDistances ), std::end( stackDistances ), std::numeric_limits<float>::max() );
	uint32_t stackPtr = 1;
	stack[0].node = 0, stack[0].dist = 0;
	while (stackPtr > 0)
	{
		stackPtr--;
		uint32_t nodeIdx = stack[--stackPtr].node; // mask with ((1u << 29) - 1) ?
		const BVHNode& node = bvh4Node[nodeIdx];
		if (!node.isLeaf())
		{
			__m256i child8;
			__m256 dist8;
			uint32_t numChildren = intersectInnerNode( node, simdRay, childrenSIMD, distancesSIMD );
			if (numChildren > 0)
			{
				_mm256_storeu_si256( child8, .. ); // childrenSIMD.store( std::span( stackCompressedNodeHandles.data() + stackPtr, 8 ) );
				_mm256_storeu_ps( dist8, .. ); // distancesSIMD.store( std::span( stackDistances.data() + stackPtr, 8 ) );
				stackPtr += numChildren;
			}
		}
		else // leaf
		{
			if (intersectLeaf( &m_leafIndexAllocator.get( handle ), leafNodePrimitiveCount( compressedNodeHandle ), ray, si ))
			{
				t8 = _mm256_set1_ps( ray.hit.t );
				// compress stack
				uint32_t outStackPtr = 0;
				for (size_t i = 0; i < stackPtr; i += 8)
				{
					__m256i node8 = _mm256_load_si256( .. ); // node8.loadAligned( std::span( stackCompressedNodeHandles.data() + i, 8 ) );
					__m256 dist8 = _mm256_load_ps( .. ); // dist8.loadAligned( std::span( stackDistances.data() + i, 8 ) );
					__m256 mask8 = _mm256_cmp_ps( _CMP_LT_OQ, dist8, t8 );
					__m256i cpi = mask8.computeCompressPermutation(); // ?
					dist8 = dist8.permute( cpi );
					node8 = node8.permute( cpi );
					_mm256_storeu_ps( dist8, .. ); // dist8.store( std::span( stackDistances.data() + outStackPtr, 8 ) );
					_mm256_storeu_si256( node8, .. ); // node8.store( std::span( stackCompressedNodeHandles.data() + outStackPtr, 8 ) );
					uint32_t numItems = tinybvh_min( 8, stackPtr - i );
					uint32_t validMask = (1 << numItems) - 1;
					outStackPtr += mask8.count( validMask ); // ?
				}
				stackPtr = outStackPtr;
			}
		}
	}
}

#endif

#endif // BVH_USEAVX

// ============================================================================
//
//        I M P L E M E N T A T I O N  -  A R M / N E O N  C O D E
//
// ============================================================================

#ifdef BVH_USENEON

// Traverse the second alternative BVH layout (ALT_SOA).
int32_t BVH_SoA::Intersect( Ray& ray ) const
{
	BVHNode* node = &bvhNode[0], * stack[64];
	const bvhvec4slice& verts = bvh.verts;
	const uint32_t* primIdx = bvh.primIdx;
	uint32_t stackPtr = 0, cost = 0;
	const float32x4_t Ox4 = vdupq_n_f32( ray.O.x ), rDx4 = vdupq_n_f32( ray.rD.x );
	const float32x4_t Oy4 = vdupq_n_f32( ray.O.y ), rDy4 = vdupq_n_f32( ray.rD.y );
	const float32x4_t Oz4 = vdupq_n_f32( ray.O.z ), rDz4 = vdupq_n_f32( ray.rD.z );
	// const float32x4_t inf4 = vdupq_n_f32( BVH_FAR );
	while (1)
	{
		cost += C_TRAV;
		if (node->isLeaf())
		{
			for (uint32_t i = 0; i < node->triCount; i++, cost += C_INT)
			{
				const uint32_t tidx = primIdx[node->firstTri + i], vertIdx = tidx * 3;
				const bvhvec3 edge1 = verts[vertIdx + 1] - verts[vertIdx];
				const bvhvec3 edge2 = verts[vertIdx + 2] - verts[vertIdx];
				const bvhvec3 h = tinybvh_cross( ray.D, edge2 );
				const float a = tinybvh_dot( edge1, h );
				if (fabs( a ) < 0.0000001f) continue; // ray parallel to triangle
				const float f = 1 / a;
				const bvhvec3 s = ray.O - bvhvec3( verts[vertIdx] );
				const float u = f * tinybvh_dot( s, h );
				if (u < 0 || u > 1) continue;
				const bvhvec3 q = tinybvh_cross( s, edge1 );
				const float v = f * tinybvh_dot( ray.D, q );
				if (v < 0 || u + v > 1) continue;
				const float t = f * tinybvh_dot( edge2, q );
				if (t < 0 || t > ray.hit.t) continue;
				ray.hit.t = t, ray.hit.u = u, ray.hit.v = v, ray.hit.prim = tidx;
			}
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		float32x4_t x4 = vmulq_f32( vsubq_f32( node->xxxx, Ox4 ), rDx4 );
		float32x4_t y4 = vmulq_f32( vsubq_f32( node->yyyy, Oy4 ), rDy4 );
		float32x4_t z4 = vmulq_f32( vsubq_f32( node->zzzz, Oz4 ), rDz4 );
		// transpose
		float32x4_t t0 = vzip1q_f32( x4, y4 ), t2 = vzip1q_f32( z4, z4 );
		float32x4_t t1 = vzip2q_f32( x4, y4 ), t3 = vzip2q_f32( z4, z4 );
		float32x4_t xyzw1a = vcombine_f32( vget_low_f32( t0 ), vget_low_f32( t2 ) );
		float32x4_t xyzw2a = vcombine_f32( vget_high_f32( t0 ), vget_high_f32( t2 ) );
		float32x4_t xyzw1b = vcombine_f32( vget_low_f32( t1 ), vget_low_f32( t3 ) );
		float32x4_t xyzw2b = vcombine_f32( vget_high_f32( t1 ), vget_high_f32( t3 ) );
		// process
		float32x4_t tmina4 = vminq_f32( xyzw1a, xyzw2a ), tmaxa4 = vmaxq_f32( xyzw1a, xyzw2a );
		float32x4_t tminb4 = vminq_f32( xyzw1b, xyzw2b ), tmaxb4 = vmaxq_f32( xyzw1b, xyzw2b );
		// transpose back
		t0 = vzip1q_f32( tmina4, tmaxa4 ), t2 = vzip1q_f32( tminb4, tmaxb4 );
		t1 = vzip2q_f32( tmina4, tmaxa4 ), t3 = vzip2q_f32( tminb4, tmaxb4 );
		x4 = vcombine_f32( vget_low_f32( t0 ), vget_low_f32( t2 ) );
		y4 = vcombine_f32( vget_high_f32( t0 ), vget_high_f32( t2 ) );
		z4 = vcombine_f32( vget_low_f32( t1 ), vget_low_f32( t3 ) );
		uint32_t lidx = node->left, ridx = node->right;
		const float32x4_t min4 = vmaxq_f32( vmaxq_f32( vmaxq_f32( x4, y4 ), z4 ), vdupq_n_f32( 0 ) );
		const float32x4_t max4 = vminq_f32( vminq_f32( vminq_f32( x4, y4 ), z4 ), vdupq_n_f32( ray.hit.t ) );
		const float tmina_0 = vgetq_lane_f32( min4, 0 ), tmaxa_1 = vgetq_lane_f32( max4, 1 );
		const float tminb_2 = vgetq_lane_f32( min4, 2 ), tmaxb_3 = vgetq_lane_f32( max4, 3 );
		float dist1 = tmaxa_1 >= tmina_0 ? tmina_0 : BVH_FAR;
		float dist2 = tmaxb_3 >= tminb_2 ? tminb_2 : BVH_FAR;
		if (dist1 > dist2)
		{
			float t = dist1; dist1 = dist2; dist2 = t;
			uint32_t i = lidx; lidx = ridx; ridx = i;
		}
		if (dist1 == BVH_FAR)
		{
			if (stackPtr == 0) break; else node = stack[--stackPtr];
		}
		else
		{
			node = bvhNode + lidx;
			if (dist2 != BVH_FAR) stack[stackPtr++] = bvhNode + ridx;
		}
	}
	return cost;
}

bool BVH_SoA::IsOccluded( const Ray& ray ) const
{
	BVHNode* node = &bvhNode[0], * stack[64];
	const bvhvec4slice& verts = bvh.verts;
	const uint32_t* primIdx = bvh.primIdx;
	uint32_t stackPtr = 0;
	const float32x4_t Ox4 = vdupq_n_f32( ray.O.x ), rDx4 = vdupq_n_f32( ray.rD.x );
	const float32x4_t Oy4 = vdupq_n_f32( ray.O.y ), rDy4 = vdupq_n_f32( ray.rD.y );
	const float32x4_t Oz4 = vdupq_n_f32( ray.O.z ), rDz4 = vdupq_n_f32( ray.rD.z );
	while (1)
	{
		if (node->isLeaf())
		{
			for (uint32_t i = 0; i < node->triCount; i++)
			{
				const uint32_t tidx = primIdx[node->firstTri + i], vertIdx = tidx * 3;
				const bvhvec3 edge1 = verts[vertIdx + 1] - verts[vertIdx];
				const bvhvec3 edge2 = verts[vertIdx + 2] - verts[vertIdx];
				const bvhvec3 h = tinybvh_cross( ray.D, edge2 );
				const float a = tinybvh_dot( edge1, h );
				if (fabs( a ) < 0.0000001f) continue; // ray parallel to triangle
				const float f = 1 / a;
				const bvhvec3 s = ray.O - bvhvec3( verts[vertIdx] );
				const float u = f * tinybvh_dot( s, h );
				if (u < 0 || u > 1) continue;
				const bvhvec3 q = tinybvh_cross( s, edge1 );
				const float v = f * tinybvh_dot( ray.D, q );
				if (v < 0 || u + v > 1) continue;
				const float t = f * tinybvh_dot( edge2, q );
				if (t > 0 && t <= ray.hit.t) return true;
			}
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		float32x4_t x4 = vmulq_f32( vsubq_f32( node->xxxx, Ox4 ), rDx4 );
		float32x4_t y4 = vmulq_f32( vsubq_f32( node->yyyy, Oy4 ), rDy4 );
		float32x4_t z4 = vmulq_f32( vsubq_f32( node->zzzz, Oz4 ), rDz4 );
		// transpose
		float32x4_t t0 = vzip1q_f32( x4, y4 ), t2 = vzip1q_f32( z4, z4 );
		float32x4_t t1 = vzip2q_f32( x4, y4 ), t3 = vzip2q_f32( z4, z4 );
		float32x4_t xyzw1a = vcombine_f32( vget_low_f32( t0 ), vget_low_f32( t2 ) );
		float32x4_t xyzw2a = vcombine_f32( vget_high_f32( t0 ), vget_high_f32( t2 ) );
		float32x4_t xyzw1b = vcombine_f32( vget_low_f32( t1 ), vget_low_f32( t3 ) );
		float32x4_t xyzw2b = vcombine_f32( vget_high_f32( t1 ), vget_high_f32( t3 ) );
		// process
		float32x4_t tmina4 = vminq_f32( xyzw1a, xyzw2a ), tmaxa4 = vmaxq_f32( xyzw1a, xyzw2a );
		float32x4_t tminb4 = vminq_f32( xyzw1b, xyzw2b ), tmaxb4 = vmaxq_f32( xyzw1b, xyzw2b );
		// transpose back
		t0 = vzip1q_f32( tmina4, tmaxa4 ), t2 = vzip1q_f32( tminb4, tmaxb4 );
		t1 = vzip2q_f32( tmina4, tmaxa4 ), t3 = vzip2q_f32( tminb4, tmaxb4 );
		x4 = vcombine_f32( vget_low_f32( t0 ), vget_low_f32( t2 ) );
		y4 = vcombine_f32( vget_high_f32( t0 ), vget_high_f32( t2 ) );
		z4 = vcombine_f32( vget_low_f32( t1 ), vget_low_f32( t3 ) );
		uint32_t lidx = node->left, ridx = node->right;
		const float32x4_t min4 = vmaxq_f32( vmaxq_f32( vmaxq_f32( x4, y4 ), z4 ), vdupq_n_f32( 0 ) );
		const float32x4_t max4 = vminq_f32( vminq_f32( vminq_f32( x4, y4 ), z4 ), vdupq_n_f32( ray.hit.t ) );
		const float tmina_0 = vgetq_lane_f32( min4, 0 ), tmaxa_1 = vgetq_lane_f32( max4, 1 );
		const float tminb_2 = vgetq_lane_f32( min4, 2 ), tmaxb_3 = vgetq_lane_f32( max4, 3 );
		float dist1 = tmaxa_1 >= tmina_0 ? tmina_0 : BVH_FAR;
		float dist2 = tmaxb_3 >= tminb_2 ? tminb_2 : BVH_FAR;
		if (dist1 > dist2)
		{
			float t = dist1; dist1 = dist2; dist2 = t;
			uint32_t i = lidx; lidx = ridx; ridx = i;
		}
		if (dist1 == BVH_FAR)
		{
			if (stackPtr == 0) break; else node = stack[--stackPtr];
		}
		else
		{
			node = bvhNode + lidx;
			if (dist2 != BVH_FAR) stack[stackPtr++] = bvhNode + ridx;
		}
	}
	return false;
}

// Traverse a 4-way BVH stored in 'Atilla Áfra' layout.
inline void IntersectCompactTri( Ray& r, float32x4_t& t4, const float* T )
{
	const float transS = T[8] * r.O.x + T[9] * r.O.y + T[10] * r.O.z + T[11];
	const float transD = T[8] * r.D.x + T[9] * r.D.y + T[10] * r.D.z;
	const float ta = -transS / transD;
	if (ta <= 0 || ta >= r.hit.t) return;
	const bvhvec3 wr = r.O + ta * r.D;
	const float u = T[0] * wr.x + T[1] * wr.y + T[2] * wr.z + T[3];
	const float v = T[4] * wr.x + T[5] * wr.y + T[6] * wr.z + T[7];
	const bool hit = u >= 0 && v >= 0 && u + v < 1;
#if INST_IDX_BITS == 32
	if (hit) r.hit = { 0, ta, u, v, *(uint32_t*)&T[15] }, t4 = vdupq_n_f32( ta );
#else
	if (hit) r.hit = { ta, u, v, *(uint32_t*)&T[15] }, t4 = vdupq_n_f32( ta );
#endif
}

inline int32_t ARMVecMovemask( uint32x4_t v ) {
	const int32_t shiftArr[4] = { 0, 1, 2, 3 };
	int32x4_t shift = vld1q_s32( shiftArr );
	return vaddvq_u32( vshlq_u32( vshrq_n_u32( v, 31 ), shift ) );
}

int32_t BVH4_CPU::Intersect( Ray& ray ) const
{
	uint32_t nodeIdx = 0, stack[1024], stackPtr = 0, cost = 0;
	const float32x4_t ox4 = vdupq_n_f32( ray.O.x ), rdx4 = vdupq_n_f32( ray.rD.x );
	const float32x4_t oy4 = vdupq_n_f32( ray.O.y ), rdy4 = vdupq_n_f32( ray.rD.y );
	const float32x4_t oz4 = vdupq_n_f32( ray.O.z ), rdz4 = vdupq_n_f32( ray.rD.z );
	float32x4_t t4 = vdupq_n_f32( ray.hit.t ), zero4 = vdupq_n_f32( 0.0f );
	const uint32x4_t idx4 = SIMD_SETRVECU( 0, 1, 2, 3 );
	const uint32x4_t idxMask = vdupq_n_u32( 0xfffffffc );
	const float32x4_t inf4 = vdupq_n_f32( BVH_FAR );
	while (1)
	{
		cost += C_TRAV;
		const BVHNode& node = bvh4Node[nodeIdx];
		// intersect the ray with four AABBs
		const float32x4_t xmin4 = node.xmin4, xmax4 = node.xmax4;
		const float32x4_t ymin4 = node.ymin4, ymax4 = node.ymax4;
		const float32x4_t zmin4 = node.zmin4, zmax4 = node.zmax4;
		const float32x4_t x0 = vsubq_f32( xmin4, ox4 ), x1 = vsubq_f32( xmax4, ox4 );
		const float32x4_t y0 = vsubq_f32( ymin4, oy4 ), y1 = vsubq_f32( ymax4, oy4 );
		const float32x4_t z0 = vsubq_f32( zmin4, oz4 ), z1 = vsubq_f32( zmax4, oz4 );
		const float32x4_t tx1 = vmulq_f32( x0, rdx4 ), tx2 = vmulq_f32( x1, rdx4 );
		const float32x4_t ty1 = vmulq_f32( y0, rdy4 ), ty2 = vmulq_f32( y1, rdy4 );
		const float32x4_t tz1 = vmulq_f32( z0, rdz4 ), tz2 = vmulq_f32( z1, rdz4 );
		const float32x4_t txmin = vminq_f32( tx1, tx2 ), tymin = vminq_f32( ty1, ty2 ), tzmin = vminq_f32( tz1, tz2 );
		const float32x4_t txmax = vmaxq_f32( tx1, tx2 ), tymax = vmaxq_f32( ty1, ty2 ), tzmax = vmaxq_f32( tz1, tz2 );
		const float32x4_t tmin = vmaxq_f32( vmaxq_f32( txmin, tymin ), tzmin );
		const float32x4_t tmax = vminq_f32( vminq_f32( txmax, tymax ), tzmax );
		uint32x4_t hit = vandq_u32( vandq_u32( vcgeq_f32( tmax, tmin ), vcltq_f32( tmin, t4 ) ), vcgeq_f32( tmax, zero4 ) );
		int32_t hitBits = ARMVecMovemask( hit ), hits = vcnt_s8( vreinterpret_s8_s32( vcreate_u32( hitBits ) ) )[0];
		if (hits == 1 /* 43% */)
		{
			// just one node was hit - no sorting needed.
			const uint32_t lane = __bfind( hitBits ), count = node.triCount[lane];
			if (count == 0) nodeIdx = node.childFirst[lane]; else
			{
				const uint32_t first = node.childFirst[lane];
				for (uint32_t j = 0; j < count; j++, cost += C_INT) // TODO: aim for 4 prims per leaf
					IntersectCompactTri( ray, t4, (float*)(bvh4Tris + first + j * 4) );
				if (stackPtr == 0) break;
				nodeIdx = stack[--stackPtr];
			}
			continue;
		}
		if (hits == 0 /* 29% */)
		{
			if (stackPtr == 0) break;
			nodeIdx = stack[--stackPtr];
			continue;
		}
		if (hits == 2 /* 16% */)
		{
			// two nodes hit
			uint32_t lane0 = __bfind( hitBits ), lane1 = __bfind( hitBits - (1 << lane0) );
			float dist0 = ((float*)&tmin)[lane0], dist1 = ((float*)&tmin)[lane1];
			if (dist1 < dist0)
			{
				uint32_t t = lane0; lane0 = lane1; lane1 = t;
				float ft = dist0; dist0 = dist1; dist1 = ft;
			}
			const uint32_t triCount0 = node.triCount[lane0], triCount1 = node.triCount[lane1];
			// process first lane
			if (triCount0 == 0) nodeIdx = node.childFirst[lane0]; else
			{
				const uint32_t first = node.childFirst[lane0];
				for (uint32_t j = 0; j < triCount0; j++, cost += C_INT) // TODO: aim for 4 prims per leaf
					IntersectCompactTri( ray, t4, (float*)(bvh4Tris + first + j * 4) );
				nodeIdx = 0;
			}
			// process second lane
			if (triCount1 == 0)
			{
				if (nodeIdx) stack[stackPtr++] = nodeIdx;
				nodeIdx = node.childFirst[lane1];
			}
			else
			{
				const uint32_t first = node.childFirst[lane1];
				for (uint32_t j = 0; j < triCount1; j++, cost += C_INT) // TODO: aim for 4 prims per leaf
					IntersectCompactTri( ray, t4, (float*)(bvh4Tris + first + j * 4) );
			}
		}
		else if (hits == 3 /* 8% */)
		{
			// blend in lane indices
			float32x4_t tm = vreinterpretq_f32_u32( vorrq_u32( vandq_u32( vreinterpretq_u32_f32( vbslq_f32( hit, tmin, inf4 ) ), idxMask ), idx4 ) );

			// sort
			float tmp, d0 = tm[0], d1 = tm[1], d2 = tm[2], d3 = tm[3];
			if (d0 < d2) tmp = d0, d0 = d2, d2 = tmp;
			if (d1 < d3) tmp = d1, d1 = d3, d3 = tmp;
			if (d0 < d1) tmp = d0, d0 = d1, d1 = tmp;
			if (d2 < d3) tmp = d2, d2 = d3, d3 = tmp;
			if (d1 < d2) tmp = d1, d1 = d2, d2 = tmp;
			// process hits
			float d[4] = { d0, d1, d2, d3 };
			nodeIdx = 0;
			for (int32_t i = 1; i < 4; i++)
			{
				uint32_t lane = *(uint32_t*)&d[i] & 3;
				if (node.triCount[lane] == 0)
				{
					const uint32_t childIdx = node.childFirst[lane];
					if (nodeIdx) stack[stackPtr++] = nodeIdx;
					nodeIdx = childIdx;
					continue;
				}
				const uint32_t first = node.childFirst[lane], count = node.triCount[lane];
				for (uint32_t j = 0; j < count; j++, cost += C_INT) // TODO: aim for 4 prims per leaf
					IntersectCompactTri( ray, t4, (float*)(bvh4Tris + first + j * 4) );
			}
		}
		else /* hits == 4, 2%: rare */
		{
			// blend in lane indices
			float32x4_t tm = vreinterpretq_f32_u32( vorrq_u32( vandq_u32( vreinterpretq_u32_f32( vbslq_f32( hit, tmin, inf4 ) ), idxMask ), idx4 ) );
			// sort
			float tmp, d0 = tm[0], d1 = tm[1], d2 = tm[2], d3 = tm[3];
			if (d0 < d2) tmp = d0, d0 = d2, d2 = tmp;
			if (d1 < d3) tmp = d1, d1 = d3, d3 = tmp;
			if (d0 < d1) tmp = d0, d0 = d1, d1 = tmp;
			if (d2 < d3) tmp = d2, d2 = d3, d3 = tmp;
			if (d1 < d2) tmp = d1, d1 = d2, d2 = tmp;
			// process hits
			float d[4] = { d0, d1, d2, d3 };
			nodeIdx = 0;
			for (int32_t i = 0; i < 4; i++)
			{
				uint32_t lane = *(uint32_t*)&d[i] & 3;
				if (node.triCount[lane] + node.childFirst[lane] == 0) continue; // TODO - never happens?
				if (node.triCount[lane] == 0)
				{
					const uint32_t childIdx = node.childFirst[lane];
					if (nodeIdx) stack[stackPtr++] = nodeIdx;
					nodeIdx = childIdx;
					continue;
				}
				const uint32_t first = node.childFirst[lane], count = node.triCount[lane];
				for (uint32_t j = 0; j < count; j++, cost += C_INT) // TODO: aim for 4 prims per leaf
					IntersectCompactTri( ray, t4, (float*)(bvh4Tris + first + j * 4) );
			}
		}
		// get next task
		if (nodeIdx) continue;
		if (stackPtr == 0) break; else nodeIdx = stack[--stackPtr];
	}
	return cost;
}

// Find occlusions in a 4-way BVH stored in 'Atilla Áfra' layout.
inline bool OccludedCompactTri( const Ray& r, const float* T )
{
	const float transS = T[8] * r.O.x + T[9] * r.O.y + T[10] * r.O.z + T[11];
	const float transD = T[8] * r.D.x + T[9] * r.D.y + T[10] * r.D.z;
	const float ta = -transS / transD;
	if (ta <= 0 || ta >= r.hit.t) return false;
	const bvhvec3 wr = r.O + ta * r.D;
	const float u = T[0] * wr.x + T[1] * wr.y + T[2] * wr.z + T[3];
	const float v = T[4] * wr.x + T[5] * wr.y + T[6] * wr.z + T[7];
	return u >= 0 && v >= 0 && u + v < 1;
}

bool BVH4_CPU::IsOccluded( const Ray& ray ) const
{
	uint32_t nodeIdx = 0, stack[1024], stackPtr = 0;
	const float32x4_t ox4 = vdupq_n_f32( ray.O.x ), rdx4 = vdupq_n_f32( ray.rD.x );
	const float32x4_t oy4 = vdupq_n_f32( ray.O.y ), rdy4 = vdupq_n_f32( ray.rD.y );
	const float32x4_t oz4 = vdupq_n_f32( ray.O.z ), rdz4 = vdupq_n_f32( ray.rD.z );
	float32x4_t t4 = vdupq_n_f32( ray.hit.t ), zero4 = vdupq_n_f32( 0.0f );
	const uint32x4_t idx4 = SIMD_SETRVECU( 0, 1, 2, 3 );
	const uint32x4_t idxMask = vdupq_n_u32( 0xfffffffc );
	const float32x4_t inf4 = vdupq_n_f32( BVH_FAR );
	while (1)
	{
		const BVHNode& node = bvh4Node[nodeIdx];
		// intersect the ray with four AABBs
		const float32x4_t xmin4 = node.xmin4, xmax4 = node.xmax4;
		const float32x4_t ymin4 = node.ymin4, ymax4 = node.ymax4;
		const float32x4_t zmin4 = node.zmin4, zmax4 = node.zmax4;
		const float32x4_t x0 = vsubq_f32( xmin4, ox4 ), x1 = vsubq_f32( xmax4, ox4 );
		const float32x4_t y0 = vsubq_f32( ymin4, oy4 ), y1 = vsubq_f32( ymax4, oy4 );
		const float32x4_t z0 = vsubq_f32( zmin4, oz4 ), z1 = vsubq_f32( zmax4, oz4 );
		const float32x4_t tx1 = vmulq_f32( x0, rdx4 ), tx2 = vmulq_f32( x1, rdx4 );
		const float32x4_t ty1 = vmulq_f32( y0, rdy4 ), ty2 = vmulq_f32( y1, rdy4 );
		const float32x4_t tz1 = vmulq_f32( z0, rdz4 ), tz2 = vmulq_f32( z1, rdz4 );
		const float32x4_t txmin = vminq_f32( tx1, tx2 ), tymin = vminq_f32( ty1, ty2 ), tzmin = vminq_f32( tz1, tz2 );
		const float32x4_t txmax = vmaxq_f32( tx1, tx2 ), tymax = vmaxq_f32( ty1, ty2 ), tzmax = vmaxq_f32( tz1, tz2 );
		const float32x4_t tmin = vmaxq_f32( vmaxq_f32( txmin, tymin ), tzmin );
		const float32x4_t tmax = vminq_f32( vminq_f32( txmax, tymax ), tzmax );
		uint32x4_t hit = vandq_u32( vandq_u32( vcgeq_f32( tmax, tmin ), vcltq_f32( tmin, t4 ) ), vcgeq_f32( tmax, zero4 ) );
		int32_t hitBits = ARMVecMovemask( hit ), hits = vcnt_s8( vreinterpret_s8_s32( vcreate_u32( hitBits ) ) )[0];
		if (hits == 1 /* 43% */)
		{
			// just one node was hit - no sorting needed.
			const uint32_t lane = __bfind( hitBits ), count = node.triCount[lane];
			if (count == 0) nodeIdx = node.childFirst[lane]; else
			{
				const uint32_t first = node.childFirst[lane];
				for (uint32_t j = 0; j < count; j++) // TODO: aim for 4 prims per leaf
					if (OccludedCompactTri( ray, (float*)(bvh4Tris + first + j * 4) )) return true;
				if (stackPtr == 0) break;
				nodeIdx = stack[--stackPtr];
			}
			continue;
		}
		if (hits == 0 /* 29% */)
		{
			if (stackPtr == 0) break;
			nodeIdx = stack[--stackPtr];
			continue;
		}
		if (hits == 2 /* 16% */)
		{
			// two nodes hit
			uint32_t lane0 = __bfind( hitBits ), lane1 = __bfind( hitBits - (1 << lane0) );
			float dist0 = ((float*)&tmin)[lane0], dist1 = ((float*)&tmin)[lane1];
			if (dist1 < dist0)
			{
				uint32_t t = lane0; lane0 = lane1; lane1 = t;
				float ft = dist0; dist0 = dist1; dist1 = ft;
			}
			const uint32_t triCount0 = node.triCount[lane0], triCount1 = node.triCount[lane1];
			// process first lane
			if (triCount0 == 0) nodeIdx = node.childFirst[lane0]; else
			{
				const uint32_t first = node.childFirst[lane0];
				for (uint32_t j = 0; j < triCount0; j++) // TODO: aim for 4 prims per leaf
					if (OccludedCompactTri( ray, (float*)(bvh4Tris + first + j * 4) )) return true;
				nodeIdx = 0;
			}
			// process second lane
			if (triCount1 == 0)
			{
				if (nodeIdx) stack[stackPtr++] = nodeIdx;
				nodeIdx = node.childFirst[lane1];
			}
			else
			{
				const uint32_t first = node.childFirst[lane1];
				for (uint32_t j = 0; j < triCount1; j++) // TODO: aim for 4 prims per leaf
					if (OccludedCompactTri( ray, (float*)(bvh4Tris + first + j * 4) )) return true;
			}
		}
		else if (hits == 3 /* 8% */)
		{
			// blend in lane indices
			float32x4_t tm = vreinterpretq_f32_u32( vorrq_u32( vandq_u32( vreinterpretq_u32_f32( vbslq_f32( hit, tmin, inf4 ) ), idxMask ), idx4 ) );
			// sort
			float tmp, d0 = tm[0], d1 = tm[1], d2 = tm[2], d3 = tm[3];
			if (d0 < d2) tmp = d0, d0 = d2, d2 = tmp;
			if (d1 < d3) tmp = d1, d1 = d3, d3 = tmp;
			if (d0 < d1) tmp = d0, d0 = d1, d1 = tmp;
			if (d2 < d3) tmp = d2, d2 = d3, d3 = tmp;
			if (d1 < d2) tmp = d1, d1 = d2, d2 = tmp;
			// process hits
			float d[4] = { d0, d1, d2, d3 };
			nodeIdx = 0;
			for (int32_t i = 1; i < 4; i++)
			{
				uint32_t lane = *(uint32_t*)&d[i] & 3;
				if (node.triCount[lane] == 0)
				{
					const uint32_t childIdx = node.childFirst[lane];
					if (nodeIdx) stack[stackPtr++] = nodeIdx;
					nodeIdx = childIdx;
					continue;
				}
				const uint32_t first = node.childFirst[lane], count = node.triCount[lane];
				for (uint32_t j = 0; j < count; j++) // TODO: aim for 4 prims per leaf
					if (OccludedCompactTri( ray, (float*)(bvh4Tris + first + j * 4) )) return true;
			}
		}
		else /* hits == 4, 2%: rare */
		{
			// blend in lane indices
			float32x4_t tm = vreinterpretq_f32_u32( vorrq_u32( vandq_u32( vreinterpretq_u32_f32( vbslq_f32( hit, tmin, inf4 ) ), idxMask ), idx4 ) );
			// sort
			float tmp, d0 = tm[0], d1 = tm[1], d2 = tm[2], d3 = tm[3];
			if (d0 < d2) tmp = d0, d0 = d2, d2 = tmp;
			if (d1 < d3) tmp = d1, d1 = d3, d3 = tmp;
			if (d0 < d1) tmp = d0, d0 = d1, d1 = tmp;
			if (d2 < d3) tmp = d2, d2 = d3, d3 = tmp;
			if (d1 < d2) tmp = d1, d1 = d2, d2 = tmp;
			// process hits
			float d[4] = { d0, d1, d2, d3 };
			nodeIdx = 0;
			for (int32_t i = 0; i < 4; i++)
			{
				uint32_t lane = *(uint32_t*)&d[i] & 3;
				if (node.triCount[lane] + node.childFirst[lane] == 0) continue; // TODO - never happens?
				if (node.triCount[lane] == 0)
				{
					const uint32_t childIdx = node.childFirst[lane];
					if (nodeIdx) stack[stackPtr++] = nodeIdx;
					nodeIdx = childIdx;
					continue;
				}
				const uint32_t first = node.childFirst[lane], count = node.triCount[lane];
				for (uint32_t j = 0; j < count; j++) // TODO: aim for 4 prims per leaf
					if (OccludedCompactTri( ray, (float*)(bvh4Tris + first + j * 4) )) return true;
			}
		}
		// get next task
		if (nodeIdx) continue;
		if (stackPtr == 0) break; else nodeIdx = stack[--stackPtr];
	}
	return false;
}

#endif // BVH_USENEON

#if !defined( BVH_USEAVX ) && !defined( BVH_USENEON )

int32_t BVH_SoA::Intersect( Ray& ray ) const
{
	FATAL_ERROR( "BVH_SoA::Intersect: Requires AVX or NEON." );
}

bool BVH_SoA::IsOccluded( const Ray& ray ) const
{
	FATAL_ERROR( "BVH_SoA::IsOccluded: Requires AVX or NEON." );
}

int32_t BVH4_CPU::Intersect( Ray& ray ) const
{
	FATAL_ERROR( "BVH4_CPU::Intersect: Requires AVX or NEON." );
}

bool BVH4_CPU::IsOccluded( const Ray& ray ) const
{
	FATAL_ERROR( "BVH4_CPU::IsOccluded: Requires AVX or NEON." );
}

#endif

// ============================================================================
//
//        D O U B L E   P R E C I S I O N   S U P P O R T
//
// ============================================================================

#ifdef DOUBLE_PRECISION_SUPPORT

// Destructor
BVH_Double::~BVH_Double()
{
	AlignedFree( fragment );
	AlignedFree( bvhNode );
	AlignedFree( primIdx );
}

void BVH_Double::Build( void (*customGetAABB)(const uint64_t, bvhdbl3&, bvhdbl3&), const uint64_t primCount )
{
	FATAL_ERROR_IF( primCount == 0, "BVH_Double::Build( void (*customGetAABB)( .. ), instCount ), instCount == 0." );
	triCount = idxCount = primCount;
	const uint64_t spaceNeeded = primCount * 2; // upper limit
	if (allocatedNodes < spaceNeeded)
	{
		AlignedFree( bvhNode );
		AlignedFree( primIdx );
		AlignedFree( fragment );
		bvhNode = (BVHNode*)AlignedAlloc( spaceNeeded * sizeof( BVHNode ) );
		allocatedNodes = spaceNeeded;
		primIdx = (uint64_t*)AlignedAlloc( primCount * sizeof( uint64_t ) );
		fragment = (Fragment*)AlignedAlloc( primCount * sizeof( Fragment ) );
	}
	// copy relevant data from instance array
	BVHNode& root = bvhNode[0];
	root.leftFirst = 0, root.triCount = primCount, root.aabbMin = bvhvec3( BVH_FAR ), root.aabbMax = bvhvec3( -BVH_FAR );
	for (uint32_t i = 0; i < primCount; i++)
	{
		customGetAABB( i, fragment[i].bmin, fragment[i].bmax );
		root.aabbMin = tinybvh_min( root.aabbMin, fragment[i].bmin ), fragment[i].primIdx = i;
		root.aabbMax = tinybvh_max( root.aabbMax, fragment[i].bmax ), primIdx[i] = i;
	}
	// start build
	newNodePtr = 1;
	Build(); // or BuildAVX, for large TLAS.
}

void BVH_Double::Build( BLASInstanceEx* bvhs, const uint64_t instCount, BVH_Double** blasses, const uint64_t bCount )
{
	FATAL_ERROR_IF( instCount == 0, "BVH_Double::Build( BLASInstanceEx*, instCount ), instCount == 0." );
	triCount = idxCount = instCount;
	const uint64_t spaceNeeded = instCount * 2; // upper limit
	if (allocatedNodes < spaceNeeded)
	{
		AlignedFree( bvhNode );
		AlignedFree( primIdx );
		AlignedFree( fragment );
		bvhNode = (BVHNode*)AlignedAlloc( spaceNeeded * sizeof( BVHNode ) );
		allocatedNodes = spaceNeeded;
		primIdx = (uint64_t*)AlignedAlloc( instCount * sizeof( uint64_t ) );
		fragment = (Fragment*)AlignedAlloc( instCount * sizeof( Fragment ) );
	}
	instList = (BLASInstanceEx*)bvhs;
	blasList = blasses;
	blasCount = bCount;
	// copy relevant data from instance array
	BVHNode& root = bvhNode[0];
	root.leftFirst = 0, root.triCount = instCount, root.aabbMin = bvhdbl3( BVH_DBL_FAR ), root.aabbMax = bvhdbl3( -BVH_DBL_FAR );
	for (uint64_t i = 0; i < instCount; i++)
	{
		if (blasList) // if a null pointer is passed, we'll assume the BLASInstances have been updated elsewhere.
		{
			uint64_t blasIdx = instList[i].blasIdx;
			BVH_Double* blas = blasList[blasIdx];
			instList[i].Update( blas );
		}
		fragment[i].bmin = instList[i].aabbMin, fragment[i].primIdx = i, fragment[i].bmax = instList[i].aabbMax;
		root.aabbMin = tinybvh_min( root.aabbMin, instList[i].aabbMin );
		root.aabbMax = tinybvh_max( root.aabbMax, instList[i].aabbMax ), primIdx[i] = i;
	}
	// start build
	newNodePtr = 1;
	Build(); // or BuildAVX, for large TLAS.
}

void BVH_Double::Build( const bvhdbl3* vertices, const uint64_t primCount )
{
	PrepareBuild( vertices, primCount );
	Build();
}
void BVH_Double::PrepareBuild( const bvhdbl3* vertices, const uint64_t primCount )
{
	FATAL_ERROR_IF( primCount == 0, "BVH_Double::PrepareBuild( .. ), primCount == 0." );
	const uint64_t spaceNeeded = primCount * 2; // upper limit
	// allocate memory on first build
	if (allocatedNodes < spaceNeeded)
	{
		AlignedFree( bvhNode );
		AlignedFree( primIdx );
		AlignedFree( fragment );
		bvhNode = (BVHNode*)AlignedAlloc( spaceNeeded * sizeof( BVHNode ) );
		allocatedNodes = spaceNeeded;
		primIdx = (uint64_t*)AlignedAlloc( primCount * sizeof( uint64_t ) );
		fragment = (Fragment*)AlignedAlloc( primCount * sizeof( Fragment ) );
	}
	verts = (bvhdbl3*)vertices; // note: we're not copying this data; don't delete.
	idxCount = triCount = primCount;
	// prepare fragments
	BVHNode& root = bvhNode[0];
	root.leftFirst = 0, root.triCount = triCount, root.aabbMin = bvhdbl3( BVH_DBL_FAR ), root.aabbMax = bvhdbl3( -BVH_DBL_FAR );
	for (uint32_t i = 0; i < triCount; i++)
	{
		const bvhdbl3 v0 = verts[i * 3], v1 = verts[i * 3 + 1], v2 = verts[i * 3 + 2];
		fragment[i].bmin = tinybvh_min( tinybvh_min( v0, v1 ), v2 );
		fragment[i].bmax = tinybvh_max( tinybvh_max( v0, v1 ), v2 );
		root.aabbMin = tinybvh_min( root.aabbMin, fragment[i].bmin );
		root.aabbMax = tinybvh_max( root.aabbMax, fragment[i].bmax ), primIdx[i] = i;
	}
	// reset node pool
	newNodePtr = 1;
	// all set; actual build happens in BVH_Double::Build.
}
void BVH_Double::Build()
{
	// subdivide root node recursively
	BVHNode& root = bvhNode[0];
	uint64_t task[256], taskCount = 0, nodeIdx = 0;
	bvhdbl3 minDim = (root.aabbMax - root.aabbMin) * 1e-20;
	bvhdbl3 bestLMin = bvhdbl3( 0 ), bestLMax = bvhdbl3( 0 ), bestRMin = bvhdbl3( 0 ), bestRMax = bvhdbl3( 0 );
	while (1)
	{
		while (1)
		{
			BVHNode& node = bvhNode[nodeIdx];
			// find optimal object split
			bvhdbl3 binMin[3][BVHBINS], binMax[3][BVHBINS];
			for (uint32_t a = 0; a < 3; a++) for (uint32_t i = 0; i < BVHBINS; i++) binMin[a][i] = bvhdbl3( BVH_DBL_FAR ), binMax[a][i] = bvhdbl3( -BVH_DBL_FAR );
			uint32_t count[3][BVHBINS];
			memset( count, 0, BVHBINS * 3 * sizeof( uint32_t ) );
			const bvhdbl3 rpd3 = bvhdbl3( BVHBINS / (node.aabbMax - node.aabbMin) ), nmin3 = node.aabbMin;
			for (uint32_t i = 0; i < node.triCount; i++) // process all tris for x,y and z at once
			{
				const uint64_t fi = primIdx[node.leftFirst + i];
				const bvhdbl3 fbi = ((fragment[fi].bmin + fragment[fi].bmax) * 0.5 - nmin3) * rpd3;
				bvhint3 bi( (int32_t)fbi.x, (int32_t)fbi.y, (int32_t)fbi.z );
				bi.x = tinybvh_clamp( bi.x, 0, BVHBINS - 1 );
				bi.y = tinybvh_clamp( bi.y, 0, BVHBINS - 1 );
				bi.z = tinybvh_clamp( bi.z, 0, BVHBINS - 1 );
				binMin[0][bi.x] = tinybvh_min( binMin[0][bi.x], fragment[fi].bmin );
				binMax[0][bi.x] = tinybvh_max( binMax[0][bi.x], fragment[fi].bmax ), count[0][bi.x]++;
				binMin[1][bi.y] = tinybvh_min( binMin[1][bi.y], fragment[fi].bmin );
				binMax[1][bi.y] = tinybvh_max( binMax[1][bi.y], fragment[fi].bmax ), count[1][bi.y]++;
				binMin[2][bi.z] = tinybvh_min( binMin[2][bi.z], fragment[fi].bmin );
				binMax[2][bi.z] = tinybvh_max( binMax[2][bi.z], fragment[fi].bmax ), count[2][bi.z]++;
			}
			// calculate per-split totals
			double splitCost = BVH_DBL_FAR, rSAV = 1.0 / node.SurfaceArea();
			uint32_t bestAxis = 0, bestPos = 0;
			for (int32_t a = 0; a < 3; a++) if ((node.aabbMax[a] - node.aabbMin[a]) > minDim[a])
			{
				bvhdbl3 lBMin[BVHBINS - 1], rBMin[BVHBINS - 1], l1 = bvhdbl3( BVH_DBL_FAR ), l2 = bvhdbl3( -BVH_DBL_FAR );
				bvhdbl3 lBMax[BVHBINS - 1], rBMax[BVHBINS - 1], r1 = bvhdbl3( BVH_DBL_FAR ), r2 = bvhdbl3( -BVH_DBL_FAR );
				double ANL[BVHBINS - 1], ANR[BVHBINS - 1];
				for (uint32_t lN = 0, rN = 0, i = 0; i < BVHBINS - 1; i++)
				{
					lBMin[i] = l1 = tinybvh_min( l1, binMin[a][i] );
					rBMin[BVHBINS - 2 - i] = r1 = tinybvh_min( r1, binMin[a][BVHBINS - 1 - i] );
					lBMax[i] = l2 = tinybvh_max( l2, binMax[a][i] );
					rBMax[BVHBINS - 2 - i] = r2 = tinybvh_max( r2, binMax[a][BVHBINS - 1 - i] );
					lN += count[a][i], rN += count[a][BVHBINS - 1 - i];
					ANL[i] = lN == 0 ? BVH_DBL_FAR : ((l2 - l1).halfArea() * (double)lN);
					ANR[BVHBINS - 2 - i] = rN == 0 ? BVH_DBL_FAR : ((r2 - r1).halfArea() * (double)rN);
				}
				// evaluate bin totals to find best position for object split
				for (uint32_t i = 0; i < BVHBINS - 1; i++)
				{
					const double C = C_TRAV + rSAV * C_INT * (ANL[i] + ANR[i]);
					if (C < splitCost)
					{
						splitCost = C, bestAxis = a, bestPos = i;
						bestLMin = lBMin[i], bestRMin = rBMin[i], bestLMax = lBMax[i], bestRMax = rBMax[i];
					}
				}
			}
			double noSplitCost = (double)node.triCount * C_INT;
			if (splitCost >= noSplitCost) break; // not splitting is better.
			// in-place partition
			uint64_t j = node.leftFirst + node.triCount, src = node.leftFirst;
			const double rpd = rpd3.cell[bestAxis], nmin = nmin3.cell[bestAxis];
			for (uint64_t i = 0; i < node.triCount; i++)
			{
				const uint64_t fi = primIdx[src];
				int32_t bi = (uint32_t)(((fragment[fi].bmin[bestAxis] + fragment[fi].bmax[bestAxis]) * 0.5 - nmin) * rpd);
				bi = tinybvh_clamp( bi, 0, BVHBINS - 1 );
				if ((uint32_t)bi <= bestPos) src++; else tinybvh_swap( primIdx[src], primIdx[--j] );
			}
			// create child nodes
			uint64_t leftCount = src - node.leftFirst, rightCount = node.triCount - leftCount;
			if (leftCount == 0 || rightCount == 0) break; // should not happen.
			const uint64_t lci = newNodePtr++, rci = newNodePtr++;
			bvhNode[lci].aabbMin = bestLMin, bvhNode[lci].aabbMax = bestLMax;
			bvhNode[lci].leftFirst = node.leftFirst, bvhNode[lci].triCount = leftCount;
			bvhNode[rci].aabbMin = bestRMin, bvhNode[rci].aabbMax = bestRMax;
			bvhNode[rci].leftFirst = j, bvhNode[rci].triCount = rightCount;
			node.leftFirst = lci, node.triCount = 0;
			// recurse
			task[taskCount++] = rci, nodeIdx = lci;
		}
		// fetch subdivision task from stack
		if (taskCount == 0) break; else nodeIdx = task[--taskCount];
	}
	// all done.
	aabbMin = bvhNode[0].aabbMin, aabbMax = bvhNode[0].aabbMax;
	refittable = true; // not using spatial splits: can refit this BVH
	may_have_holes = false; // the reference builder produces a continuous list of nodes
	bvh_over_aabbs = (verts == 0); // bvh over aabbs is suitable as TLAS
	usedNodes = newNodePtr;
}

double BVH_Double::BVHNode::SurfaceArea() const
{
	const bvhdbl3 e = aabbMax - aabbMin;
	return e.x * e.y + e.y * e.z + e.z * e.x;
}

double BVH_Double::SAHCost( const uint64_t nodeIdx ) const
{
	// Determine the SAH cost of a double-precision tree.
	const BVHNode& n = bvhNode[nodeIdx];
	if (n.isLeaf()) return C_INT * n.SurfaceArea() * n.triCount;
	double cost = C_TRAV * n.SurfaceArea() + SAHCost( n.leftFirst ) + SAHCost( n.leftFirst + 1 );
	return nodeIdx == 0 ? (cost / n.SurfaceArea()) : cost;
}

// Traverse the default BVH layout, double-precision.
int32_t BVH_Double::Intersect( RayEx& ray ) const
{
	if (instList) return IntersectTLAS( ray );
	BVHNode* node = &bvhNode[0], * stack[64];
	uint32_t stackPtr = 0, cost = 0;
	while (1)
	{
		cost += C_TRAV;
		if (node->isLeaf())
		{
			if (customEnabled && customIntersect != 0)
			{
				for (uint32_t i = 0; i < node->triCount; i++, cost += C_INT)
					if ((*customIntersect)(ray, primIdx[node->leftFirst + i]))
						ray.hit.inst = ray.instIdx;
			}
			else for (uint32_t i = 0; i < node->triCount; i++, cost += C_INT)
			{
				const uint64_t idx = primIdx[node->leftFirst + i];
				const uint64_t vertIdx = idx * 3;
				const bvhdbl3 edge1 = verts[vertIdx + 1] - verts[vertIdx];
				const bvhdbl3 edge2 = verts[vertIdx + 2] - verts[vertIdx];
				const bvhdbl3 h = tinybvh_cross( ray.D, edge2 );
				const double a = tinybvh_dot( edge1, h );
				if (fabs( a ) < 0.0000001) continue; // ray parallel to triangle
				const double f = 1 / a;
				const bvhdbl3 s = ray.O - bvhdbl3( verts[vertIdx] );
				const double u = f * tinybvh_dot( s, h );
				if (u < 0 || u > 1) continue;
				const bvhdbl3 q = tinybvh_cross( s, edge1 );
				const double v = f * tinybvh_dot( ray.D, q );
				if (v < 0 || u + v > 1) continue;
				const double t = f * tinybvh_dot( edge2, q );
				if (t > 0 && t < ray.hit.t)
				{
					// register a hit: ray is shortened to t
					ray.hit.t = t, ray.hit.u = u, ray.hit.v = v;
					ray.hit.prim = idx;
					ray.hit.inst = ray.instIdx;
				}
			}
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		BVHNode* child1 = &bvhNode[node->leftFirst];
		BVHNode* child2 = &bvhNode[node->leftFirst + 1];
		double dist1 = child1->Intersect( ray ), dist2 = child2->Intersect( ray );
		if (dist1 > dist2) { tinybvh_swap( dist1, dist2 ); tinybvh_swap( child1, child2 ); }
		if (dist1 == BVH_DBL_FAR /* missed both child nodes */)
		{
			if (stackPtr == 0) break; else node = stack[--stackPtr];
		}
		else /* hit at least one node */
		{
			node = child1; /* continue with the nearest */
			if (dist2 != BVH_DBL_FAR) stack[stackPtr++] = child2; /* push far child */
		}
	}
	return cost;
}

// Traverse a double-precision TLAS.
int32_t BVH_Double::IntersectTLAS( RayEx& ray ) const
{
	BVHNode* node = &bvhNode[0], * stack[64];
	uint32_t stackPtr = 0, cost = 0;
	while (1)
	{
		cost += C_TRAV;
		if (node->isLeaf())
		{
			RayEx tmp;
			for (uint32_t i = 0; i < node->triCount; i++)
			{
				// BLAS traversal
				const uint64_t instIdx = primIdx[node->leftFirst + i];
				BLASInstanceEx& inst = instList[instIdx];
				BVH_Double* blas = blasList[inst.blasIdx];
				// 1. Transform ray with the inverse of the instance transform
				tmp.O = tinybvh_transform_point( ray.O, inst.invTransform );
				tmp.D = tinybvh_transform_vector( ray.D, inst.invTransform );
				tmp.hit = ray.hit;
				tmp.rD.x = tmp.D.x > 1e-24 ? (1.0 / tmp.D.x) : (tmp.D.x < -1e-24 ? (1.0 / tmp.D.x) : BVH_DBL_FAR);
				tmp.rD.y = tmp.D.y > 1e-24 ? (1.0 / tmp.D.y) : (tmp.D.y < -1e-24 ? (1.0 / tmp.D.y) : BVH_DBL_FAR);
				tmp.rD.z = tmp.D.z > 1e-24 ? (1.0 / tmp.D.z) : (tmp.D.z < -1e-24 ? (1.0 / tmp.D.z) : BVH_DBL_FAR);
				// 2. Traverse BLAS with the transformed ray
				tmp.instIdx = instIdx;
				cost += blas->Intersect( tmp );
				// 3. Restore ray
				ray.hit = tmp.hit;
			}
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		BVHNode* child1 = &bvhNode[node->leftFirst];
		BVHNode* child2 = &bvhNode[node->leftFirst + 1];
		double dist1 = child1->Intersect( ray ), dist2 = child2->Intersect( ray );
		if (dist1 > dist2) { tinybvh_swap( dist1, dist2 ); tinybvh_swap( child1, child2 ); }
		if (dist1 == BVH_DBL_FAR /* missed both child nodes */)
		{
			if (stackPtr == 0) break; else node = stack[--stackPtr];
		}
		else /* hit at least one node */
		{
			node = child1; /* continue with the nearest */
			if (dist2 != BVH_DBL_FAR) stack[stackPtr++] = child2; /* push far child */
		}
	}
	return cost;
}

bool BVH_Double::IsOccluded( const RayEx& ray ) const
{
	if (instList) return IsOccludedTLAS( ray );
	BVHNode* node = &bvhNode[0], * stack[64];
	uint32_t stackPtr = 0;
	while (1)
	{
		if (node->isLeaf())
		{
			if (customEnabled && customIntersect != 0)
			{
				for (uint32_t i = 0; i < node->triCount; i++)
					(*customIsOccluded)(ray, primIdx[node->leftFirst + i]);
			}
			else for (uint32_t i = 0; i < node->triCount; i++)
			{
				const uint64_t idx = primIdx[node->leftFirst + i];
				const uint64_t vertIdx = idx * 3;
				const bvhdbl3 edge1 = verts[vertIdx + 1] - verts[vertIdx];
				const bvhdbl3 edge2 = verts[vertIdx + 2] - verts[vertIdx];
				const bvhdbl3 h = tinybvh_cross( ray.D, edge2 );
				const double a = tinybvh_dot( edge1, h );
				if (fabs( a ) < 0.0000001) continue; // ray parallel to triangle
				const double f = 1 / a;
				const bvhdbl3 s = ray.O - bvhdbl3( verts[vertIdx] );
				const double u = f * tinybvh_dot( s, h );
				if (u < 0 || u > 1) continue;
				const bvhdbl3 q = tinybvh_cross( s, edge1 );
				const double v = f * tinybvh_dot( ray.D, q );
				if (v < 0 || u + v > 1) continue;
				const double t = f * tinybvh_dot( edge2, q );
				if (t > 0 && t < ray.hit.t) return true;
			}
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		BVHNode* child1 = &bvhNode[node->leftFirst];
		BVHNode* child2 = &bvhNode[node->leftFirst + 1];
		double dist1 = child1->Intersect( ray ), dist2 = child2->Intersect( ray );
		if (dist1 > dist2) { tinybvh_swap( dist1, dist2 ); tinybvh_swap( child1, child2 ); }
		if (dist1 == BVH_DBL_FAR /* missed both child nodes */)
		{
			if (stackPtr == 0) break; else node = stack[--stackPtr];
		}
		else /* hit at least one node */
		{
			node = child1; /* continue with the nearest */
			if (dist2 != BVH_DBL_FAR) stack[stackPtr++] = child2; /* push far child */
		}
	}
	return false;
}

bool BVH_Double::IsOccludedTLAS( const RayEx& ray ) const
{
	BVHNode* node = &bvhNode[0], * stack[64];
	uint32_t stackPtr = 0;
	RayEx tmp;
	while (1)
	{
		if (node->isLeaf())
		{
			for (uint32_t i = 0; i < node->triCount; i++)
			{
				// BLAS traversal
				BLASInstanceEx& inst = instList[primIdx[node->leftFirst + i]];
				BVH_Double* blas = blasList[inst.blasIdx];
				// 1. Transform ray with the inverse of the instance transform
				tmp.O = tinybvh_transform_point( ray.O, inst.invTransform );
				tmp.D = tinybvh_transform_vector( ray.D, inst.invTransform );
				tmp.rD.x = tmp.D.x > 1e-24 ? (1.0 / tmp.D.x) : (tmp.D.x < -1e-24 ? (1.0 / tmp.D.x) : BVH_DBL_FAR);
				tmp.rD.y = tmp.D.y > 1e-24 ? (1.0 / tmp.D.y) : (tmp.D.y < -1e-24 ? (1.0 / tmp.D.y) : BVH_DBL_FAR);
				tmp.rD.z = tmp.D.z > 1e-24 ? (1.0 / tmp.D.z) : (tmp.D.z < -1e-24 ? (1.0 / tmp.D.z) : BVH_DBL_FAR);
				// 2. Traverse BLAS with the transformed ray
				if (blas->IsOccluded( tmp )) return true;
			}
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		BVHNode* child1 = &bvhNode[node->leftFirst];
		BVHNode* child2 = &bvhNode[node->leftFirst + 1];
		double dist1 = child1->Intersect( ray ), dist2 = child2->Intersect( ray );
		if (dist1 > dist2) { tinybvh_swap( dist1, dist2 ); tinybvh_swap( child1, child2 ); }
		if (dist1 == BVH_DBL_FAR /* missed both child nodes */)
		{
			if (stackPtr == 0) break; else node = stack[--stackPtr];
		}
		else /* hit at least one node */
		{
			node = child1; /* continue with the nearest */
			if (dist2 != BVH_DBL_FAR) stack[stackPtr++] = child2; /* push far child */
		}
	}
	return false;
}

// IntersectAABB, double precision
double BVH_Double::BVHNode::Intersect( const RayEx& ray ) const
{
	// double-precision "slab test" ray/AABB intersection
	double tx1 = (aabbMin.x - ray.O.x) * ray.rD.x, tx2 = (aabbMax.x - ray.O.x) * ray.rD.x;
	double tmin = tinybvh_min( tx1, tx2 ), tmax = tinybvh_max( tx1, tx2 );
	double ty1 = (aabbMin.y - ray.O.y) * ray.rD.y, ty2 = (aabbMax.y - ray.O.y) * ray.rD.y;
	tmin = tinybvh_max( tmin, tinybvh_min( ty1, ty2 ) );
	tmax = tinybvh_min( tmax, tinybvh_max( ty1, ty2 ) );
	double tz1 = (aabbMin.z - ray.O.z) * ray.rD.z, tz2 = (aabbMax.z - ray.O.z) * ray.rD.z;
	tmin = tinybvh_max( tmin, tinybvh_min( tz1, tz2 ) );
	tmax = tinybvh_min( tmax, tinybvh_max( tz1, tz2 ) );
	if (tmax >= tmin && tmin < ray.hit.t && tmax >= 0) return tmin; else return BVH_DBL_FAR;
}

#endif

// ============================================================================
//
//        H E L P E R S
//
// ============================================================================

// Update
void BLASInstance::Update( BVHBase* blas )
{
	InvertTransform(); // TODO: done unconditionally; for a big TLAS this may be wasteful. Detect changes automatically?
	// transform the eight corners of the root node aabb using the
	// instance transform and calculate the worldspace aabb over those.
	aabbMin = bvhvec3( BVH_FAR ), aabbMax = bvhvec3( -BVH_FAR );
	bvhvec3 bmin = blas->aabbMin, bmax = blas->aabbMax;
	for (int32_t j = 0; j < 8; j++)
	{
		const bvhvec3 p( j & 1 ? bmax.x : bmin.x, j & 2 ? bmax.y : bmin.y, j & 4 ? bmax.z : bmin.z );
		const bvhvec3 t = tinybvh_transform_point( p, transform );
		aabbMin = tinybvh_min( aabbMin, t ), aabbMax = tinybvh_max( aabbMax, t );
	}
}

// InvertTransform - calculate the inverse of the matrix stored in 'transform'
void BLASInstance::InvertTransform()
{
	// math from MESA, via http://stackoverflow.com/questions/1148309/inverting-a-4x4-matrix
	const float* T = this->transform;
	invTransform[0] = T[5] * T[10] * T[15] - T[5] * T[11] * T[14] - T[9] * T[6] * T[15] + T[9] * T[7] * T[14] + T[13] * T[6] * T[11] - T[13] * T[7] * T[10];
	invTransform[1] = -T[1] * T[10] * T[15] + T[1] * T[11] * T[14] + T[9] * T[2] * T[15] - T[9] * T[3] * T[14] - T[13] * T[2] * T[11] + T[13] * T[3] * T[10];
	invTransform[2] = T[1] * T[6] * T[15] - T[1] * T[7] * T[14] - T[5] * T[2] * T[15] + T[5] * T[3] * T[14] + T[13] * T[2] * T[7] - T[13] * T[3] * T[6];
	invTransform[3] = -T[1] * T[6] * T[11] + T[1] * T[7] * T[10] + T[5] * T[2] * T[11] - T[5] * T[3] * T[10] - T[9] * T[2] * T[7] + T[9] * T[3] * T[6];
	invTransform[4] = -T[4] * T[10] * T[15] + T[4] * T[11] * T[14] + T[8] * T[6] * T[15] - T[8] * T[7] * T[14] - T[12] * T[6] * T[11] + T[12] * T[7] * T[10];
	invTransform[5] = T[0] * T[10] * T[15] - T[0] * T[11] * T[14] - T[8] * T[2] * T[15] + T[8] * T[3] * T[14] + T[12] * T[2] * T[11] - T[12] * T[3] * T[10];
	invTransform[6] = -T[0] * T[6] * T[15] + T[0] * T[7] * T[14] + T[4] * T[2] * T[15] - T[4] * T[3] * T[14] - T[12] * T[2] * T[7] + T[12] * T[3] * T[6];
	invTransform[7] = T[0] * T[6] * T[11] - T[0] * T[7] * T[10] - T[4] * T[2] * T[11] + T[4] * T[3] * T[10] + T[8] * T[2] * T[7] - T[8] * T[3] * T[6];
	invTransform[8] = T[4] * T[9] * T[15] - T[4] * T[11] * T[13] - T[8] * T[5] * T[15] + T[8] * T[7] * T[13] + T[12] * T[5] * T[11] - T[12] * T[7] * T[9];
	invTransform[9] = -T[0] * T[9] * T[15] + T[0] * T[11] * T[13] + T[8] * T[1] * T[15] - T[8] * T[3] * T[13] - T[12] * T[1] * T[11] + T[12] * T[3] * T[9];
	invTransform[10] = T[0] * T[5] * T[15] - T[0] * T[7] * T[13] - T[4] * T[1] * T[15] + T[4] * T[3] * T[13] + T[12] * T[1] * T[7] - T[12] * T[3] * T[5];
	invTransform[11] = -T[0] * T[5] * T[11] + T[0] * T[7] * T[9] + T[4] * T[1] * T[11] - T[4] * T[3] * T[9] - T[8] * T[1] * T[7] + T[8] * T[3] * T[5];
	invTransform[12] = -T[4] * T[9] * T[14] + T[4] * T[10] * T[13] + T[8] * T[5] * T[14] - T[8] * T[6] * T[13] - T[12] * T[5] * T[10] + T[12] * T[6] * T[9];
	invTransform[13] = T[0] * T[9] * T[14] - T[0] * T[10] * T[13] - T[8] * T[1] * T[14] + T[8] * T[2] * T[13] + T[12] * T[1] * T[10] - T[12] * T[2] * T[9];
	invTransform[14] = -T[0] * T[5] * T[14] + T[0] * T[6] * T[13] + T[4] * T[1] * T[14] - T[4] * T[2] * T[13] - T[12] * T[1] * T[6] + T[12] * T[2] * T[5];
	invTransform[15] = T[0] * T[5] * T[10] - T[0] * T[6] * T[9] - T[4] * T[1] * T[10] + T[4] * T[2] * T[9] + T[8] * T[1] * T[6] - T[8] * T[2] * T[5];
	const float det = T[0] * invTransform[0] + T[1] * invTransform[4] + T[2] * invTransform[8] + T[3] * invTransform[12];
	if (det == 0) return; // actually, invert failed. That's bad.
	const float invdet = 1.0f / det;
	for (int i = 0; i < 16; i++) invTransform[i] *= invdet;
}

#ifdef DOUBLE_PRECISION_SUPPORT

// Update
void BLASInstanceEx::Update( BVH_Double* blas )
{
	InvertTransform(); // TODO: done unconditionally; for a big TLAS this may be wasteful. Detect changes automatically?
	// transform the eight corners of the root node aabb using the
	// instance transform and calculate the worldspace aabb over those.
	aabbMin = bvhdbl3( BVH_FAR ), aabbMax = bvhdbl3( -BVH_FAR );
	bvhdbl3 bmin = blas->aabbMin, bmax = blas->aabbMax;
	for (int32_t j = 0; j < 8; j++)
	{
		const bvhdbl3 p( j & 1 ? bmax.x : bmin.x, j & 2 ? bmax.y : bmin.y, j & 4 ? bmax.z : bmin.z );
		const bvhdbl3 t = tinybvh_transform_point( p, transform );
		aabbMin = tinybvh_min( aabbMin, t ), aabbMax = tinybvh_max( aabbMax, t );
	}
}

// InvertTransform - calculate the inverse of the matrix stored in 'transform'
void BLASInstanceEx::InvertTransform()
{
	// math from MESA, via http://stackoverflow.com/questions/1148309/inverting-a-4x4-matrix
	const double* T = this->transform;
	invTransform[0] = T[5] * T[10] * T[15] - T[5] * T[11] * T[14] - T[9] * T[6] * T[15] + T[9] * T[7] * T[14] + T[13] * T[6] * T[11] - T[13] * T[7] * T[10];
	invTransform[1] = -T[1] * T[10] * T[15] + T[1] * T[11] * T[14] + T[9] * T[2] * T[15] - T[9] * T[3] * T[14] - T[13] * T[2] * T[11] + T[13] * T[3] * T[10];
	invTransform[2] = T[1] * T[6] * T[15] - T[1] * T[7] * T[14] - T[5] * T[2] * T[15] + T[5] * T[3] * T[14] + T[13] * T[2] * T[7] - T[13] * T[3] * T[6];
	invTransform[3] = -T[1] * T[6] * T[11] + T[1] * T[7] * T[10] + T[5] * T[2] * T[11] - T[5] * T[3] * T[10] - T[9] * T[2] * T[7] + T[9] * T[3] * T[6];
	invTransform[4] = -T[4] * T[10] * T[15] + T[4] * T[11] * T[14] + T[8] * T[6] * T[15] - T[8] * T[7] * T[14] - T[12] * T[6] * T[11] + T[12] * T[7] * T[10];
	invTransform[5] = T[0] * T[10] * T[15] - T[0] * T[11] * T[14] - T[8] * T[2] * T[15] + T[8] * T[3] * T[14] + T[12] * T[2] * T[11] - T[12] * T[3] * T[10];
	invTransform[6] = -T[0] * T[6] * T[15] + T[0] * T[7] * T[14] + T[4] * T[2] * T[15] - T[4] * T[3] * T[14] - T[12] * T[2] * T[7] + T[12] * T[3] * T[6];
	invTransform[7] = T[0] * T[6] * T[11] - T[0] * T[7] * T[10] - T[4] * T[2] * T[11] + T[4] * T[3] * T[10] + T[8] * T[2] * T[7] - T[8] * T[3] * T[6];
	invTransform[8] = T[4] * T[9] * T[15] - T[4] * T[11] * T[13] - T[8] * T[5] * T[15] + T[8] * T[7] * T[13] + T[12] * T[5] * T[11] - T[12] * T[7] * T[9];
	invTransform[9] = -T[0] * T[9] * T[15] + T[0] * T[11] * T[13] + T[8] * T[1] * T[15] - T[8] * T[3] * T[13] - T[12] * T[1] * T[11] + T[12] * T[3] * T[9];
	invTransform[10] = T[0] * T[5] * T[15] - T[0] * T[7] * T[13] - T[4] * T[1] * T[15] + T[4] * T[3] * T[13] + T[12] * T[1] * T[7] - T[12] * T[3] * T[5];
	invTransform[11] = -T[0] * T[5] * T[11] + T[0] * T[7] * T[9] + T[4] * T[1] * T[11] - T[4] * T[3] * T[9] - T[8] * T[1] * T[7] + T[8] * T[3] * T[5];
	invTransform[12] = -T[4] * T[9] * T[14] + T[4] * T[10] * T[13] + T[8] * T[5] * T[14] - T[8] * T[6] * T[13] - T[12] * T[5] * T[10] + T[12] * T[6] * T[9];
	invTransform[13] = T[0] * T[9] * T[14] - T[0] * T[10] * T[13] - T[8] * T[1] * T[14] + T[8] * T[2] * T[13] + T[12] * T[1] * T[10] - T[12] * T[2] * T[9];
	invTransform[14] = -T[0] * T[5] * T[14] + T[0] * T[6] * T[13] + T[4] * T[1] * T[14] - T[4] * T[2] * T[13] - T[12] * T[1] * T[6] + T[12] * T[2] * T[5];
	invTransform[15] = T[0] * T[5] * T[10] - T[0] * T[6] * T[9] - T[4] * T[1] * T[10] + T[4] * T[2] * T[9] + T[8] * T[1] * T[6] - T[8] * T[2] * T[5];
	const double det = T[0] * invTransform[0] + T[1] * invTransform[4] + T[2] * invTransform[8] + T[3] * invTransform[12];
	if (det == 0) return; // actually, invert failed. That's bad.
	const double invdet = 1. / det;
	for (int i = 0; i < 16; i++) invTransform[i] *= invdet;
}

#endif

// SA
float BVHBase::SA( const bvhvec3& aabbMin, const bvhvec3& aabbMax )
{
	bvhvec3 e = aabbMax - aabbMin; // extent of the node
	return e.x * e.y + e.y * e.z + e.z * e.x;
}

// IntersectTri
void BVHBase::IntersectTri( Ray& ray, const bvhvec4slice& verts, const uint32_t idx ) const
{
	// Moeller-Trumbore ray/triangle intersection algorithm
	const uint32_t vertIdx = idx * 3;
	const bvhvec4 vert0 = verts[vertIdx];
	const bvhvec3 edge1 = verts[vertIdx + 1] - vert0;
	const bvhvec3 edge2 = verts[vertIdx + 2] - vert0;
	const bvhvec3 h = tinybvh_cross( ray.D, edge2 );
	const float a = tinybvh_dot( edge1, h );
	if (fabs( a ) < 0.0000001f) return; // ray parallel to triangle
	const float f = 1 / a;
	const bvhvec3 s = ray.O - bvhvec3( vert0 );
	const float u = f * tinybvh_dot( s, h );
	if (u < 0 || u > 1) return;
	const bvhvec3 q = tinybvh_cross( s, edge1 );
	const float v = f * tinybvh_dot( ray.D, q );
	if (v < 0 || u + v > 1) return;
	const float t = f * tinybvh_dot( edge2, q );
	if (t > 0 && t < ray.hit.t)
	{
		// register a hit: ray is shortened to t
		ray.hit.t = t, ray.hit.u = u, ray.hit.v = v;
	#if INST_IDX_BITS == 32
		ray.hit.prim = idx, ray.hit.inst = ray.instIdx;
	#else
		ray.hit.prim = idx + ray.instIdx;
	#endif
	}
}

// IntersectTriIndexed
void BVHBase::IntersectTriIndexed( Ray& ray, const bvhvec4slice& verts, const uint32_t* indices, const uint32_t idx ) const
{
	// Moeller-Trumbore ray/triangle intersection algorithm
	const uint32_t i0 = indices[idx * 3], i1 = indices[idx * 3 + 1], i2 = indices[idx * 3 + 2];
	const bvhvec4 vert0 = verts[i0];
	const bvhvec3 edge1 = verts[i1] - vert0;
	const bvhvec3 edge2 = verts[i2] - vert0;
	const bvhvec3 h = tinybvh_cross( ray.D, edge2 );
	const float a = tinybvh_dot( edge1, h );
	if (fabs( a ) < 0.0000001f) return; // ray parallel to triangle
	const float f = 1 / a;
	const bvhvec3 s = ray.O - bvhvec3( vert0 );
	const float u = f * tinybvh_dot( s, h );
	if (u < 0 || u > 1) return;
	const bvhvec3 q = tinybvh_cross( s, edge1 );
	const float v = f * tinybvh_dot( ray.D, q );
	if (v < 0 || u + v > 1) return;
	const float t = f * tinybvh_dot( edge2, q );
	if (t > 0 && t < ray.hit.t)
	{
		// register a hit: ray is shortened to t
		ray.hit.t = t, ray.hit.u = u, ray.hit.v = v;
	#if INST_IDX_BITS == 32
		ray.hit.prim = idx, ray.hit.inst = ray.instIdx;
	#else
		ray.hit.prim = idx + ray.instIdx;
	#endif
	}
}

// TriOccludes
bool BVHBase::TriOccludes( const Ray& ray, const bvhvec4slice& verts, const uint32_t idx ) const
{
	// Moeller-Trumbore ray/triangle intersection algorithm
	const uint32_t vertIdx = idx * 3;
	const bvhvec4 vert0 = verts[vertIdx];
	const bvhvec3 edge1 = verts[vertIdx + 1] - vert0;
	const bvhvec3 edge2 = verts[vertIdx + 2] - vert0;
	const bvhvec3 h = tinybvh_cross( ray.D, edge2 );
	const float a = tinybvh_dot( edge1, h );
	if (fabs( a ) < 0.0000001f) return false; // ray parallel to triangle
	const float f = 1 / a;
	const bvhvec3 s = ray.O - bvhvec3( vert0 );
	const float u = f * tinybvh_dot( s, h );
	if (u < 0 || u > 1) return false;
	const bvhvec3 q = tinybvh_cross( s, edge1 );
	const float v = f * tinybvh_dot( ray.D, q );
	if (v < 0 || u + v > 1) return false;
	const float t = f * tinybvh_dot( edge2, q );
	return t > 0 && t < ray.hit.t;
}

bool BVHBase::IndexedTriOccludes( const Ray& ray, const bvhvec4slice& verts, const uint32_t* indices, const uint32_t idx ) const
{
	// Moeller-Trumbore ray/triangle intersection algorithm
	const uint32_t i0 = indices[idx * 3], i1 = indices[idx * 3 + 1], i2 = indices[idx * 3 + 2];
	const bvhvec4 vert0 = verts[i0];
	const bvhvec3 edge1 = verts[i1] - vert0;
	const bvhvec3 edge2 = verts[i2] - vert0;
	const bvhvec3 h = tinybvh_cross( ray.D, edge2 );
	const float a = tinybvh_dot( edge1, h );
	if (fabs( a ) < 0.0000001f) return false; // ray parallel to triangle
	const float f = 1 / a;
	const bvhvec3 s = ray.O - bvhvec3( vert0 );
	const float u = f * tinybvh_dot( s, h );
	if (u < 0 || u > 1) return false;
	const bvhvec3 q = tinybvh_cross( s, edge1 );
	const float v = f * tinybvh_dot( ray.D, q );
	if (v < 0 || u + v > 1) return false;
	const float t = f * tinybvh_dot( edge2, q );
	return t > 0 && t < ray.hit.t;
}

// IntersectAABB
float BVHBase::IntersectAABB( const Ray& ray, const bvhvec3& aabbMin, const bvhvec3& aabbMax )
{
	// "slab test" ray/AABB intersection
	float tx1 = (aabbMin.x - ray.O.x) * ray.rD.x, tx2 = (aabbMax.x - ray.O.x) * ray.rD.x;
	float tmin = tinybvh_min( tx1, tx2 ), tmax = tinybvh_max( tx1, tx2 );
	float ty1 = (aabbMin.y - ray.O.y) * ray.rD.y, ty2 = (aabbMax.y - ray.O.y) * ray.rD.y;
	tmin = tinybvh_max( tmin, tinybvh_min( ty1, ty2 ) );
	tmax = tinybvh_min( tmax, tinybvh_max( ty1, ty2 ) );
	float tz1 = (aabbMin.z - ray.O.z) * ray.rD.z, tz2 = (aabbMax.z - ray.O.z) * ray.rD.z;
	tmin = tinybvh_max( tmin, tinybvh_min( tz1, tz2 ) );
	tmax = tinybvh_min( tmax, tinybvh_max( tz1, tz2 ) );
	if (tmax >= tmin && tmin < ray.hit.t && tmax >= 0) return tmin; else return BVH_FAR;
}

// PrecomputeTriangle (helper), transforms a triangle to the format used in:
// Fast Ray-Triangle Intersections by Coordinate Transformation. Baldwin & Weber, 2016.
void BVHBase::PrecomputeTriangle( const bvhvec4slice& vert, const uint32_t ti0, const uint32_t ti1, const uint32_t ti2, float* T )
{
	bvhvec3 v0 = vert[ti0], v1 = vert[ti1], v2 = vert[ti2];
	bvhvec3 e1 = v1 - v0, e2 = v2 - v0, N = tinybvh_cross( e1, e2 );
	float x1, x2, n = tinybvh_dot( v0, N ), rN;
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
	else memset( T, 0, 12 * 4 ); // cerr << "degenerate source " << endl;
}

bool BVH::BVHNode::Intersect( const bvhvec3& bmin, const bvhvec3& bmax ) const
{
	return bmin.x < aabbMax.x && bmax.x > aabbMin.x &&
		bmin.y < aabbMax.y && bmax.y > aabbMin.y &&
		bmin.z < aabbMax.z && bmax.z > aabbMin.z;
}

// Faster ClipFrag, which clips against only two planes if a tri wasn't clipped before.
bool BVH::ClipFrag( const Fragment& orig, Fragment& newFrag, bvhvec3 bmin, bvhvec3 bmax, bvhvec3 minDim, const uint32_t axis )
{
	// find intersection of bmin/bmax and orig bmin/bmax
	bmin = tinybvh_max( bmin, orig.bmin ), bmax = tinybvh_min( bmax, orig.bmax );
	const bvhvec3 extent = bmax - bmin;
	uint32_t Nin = 3, vidx = orig.primIdx * 3;
	if (orig.clipped)
	{
		// generic case: Sutherland-Hodgeman against six bounding planes
		bvhvec3 vin[16], vout[16];
		if (vertIdx)
			vin[0] = verts[vertIdx[vidx]], vin[1] = verts[vertIdx[vidx + 1]], vin[2] = verts[vertIdx[vidx + 2]];
		else
			vin[0] = verts[vidx], vin[1] = verts[vidx + 1], vin[2] = verts[vidx + 2];
		for (uint32_t a = 0; a < 3; a++)
		{
			const float eps = minDim.cell[a];
			if (extent.cell[a] > eps)
			{
				uint32_t Nout = 0;
				const float l = bmin[a], r = bmax[a];
				for (uint32_t v = 0; v < Nin; v++)
				{
					bvhvec3 v0 = vin[v], v1 = vin[(v + 1) % Nin];
					const bool v0in = v0[a] >= l - eps, v1in = v1[a] >= l - eps;
					if (!(v0in || v1in)) continue; else if (v0in ^ v1in)
					{
						bvhvec3 C = v0 + (l - v0[a]) / (v1[a] - v0[a]) * (v1 - v0);
						C[a] = l /* accurate */, vout[Nout++] = C;
					}
					if (v1in) vout[Nout++] = v1;
				}
				Nin = 0;
				for (uint32_t v = 0; v < Nout; v++)
				{
					bvhvec3 v0 = vout[v], v1 = vout[(v + 1) % Nout];
					const bool v0in = v0[a] <= r + eps, v1in = v1[a] <= r + eps;
					if (!(v0in || v1in)) continue; else if (v0in ^ v1in)
					{
						bvhvec3 C = v0 + (r - v0[a]) / (v1[a] - v0[a]) * (v1 - v0);
						C[a] = r /* accurate */, vin[Nin++] = C;
					}
					if (v1in) vin[Nin++] = v1;
				}
			}
		}
		bvhvec3 mn( BVH_FAR ), mx( -BVH_FAR );
		for (uint32_t i = 0; i < Nin; i++) mn = tinybvh_min( mn, vin[i] ), mx = tinybvh_max( mx, vin[i] );
		newFrag.primIdx = orig.primIdx;
		newFrag.bmin = tinybvh_max( mn, bmin ), newFrag.bmax = tinybvh_min( mx, bmax );
		newFrag.clipped = 1;
		return Nin > 0;
	}
	else
	{
		// special case: if this fragment has not been clipped before, only clip against planes on split axis.
		bool hasVerts = false;
		bvhvec3 mn( BVH_FAR ), mx( -BVH_FAR ), vout[4], C;
		if (extent.cell[axis] > minDim.cell[axis])
		{
			const float l = bmin[axis], r = bmax[axis];
			uint32_t Nout = 0;
			{
				bvhvec3 v0, v1, v2;
				if (vertIdx)
					v0 = verts[vertIdx[vidx]], v1 = verts[vertIdx[vidx + 1]], v2 = verts[vertIdx[vidx + 2]];
				else
					v0 = verts[vidx + 0], v1 = verts[vidx + 1], v2 = verts[vidx + 2];
				bool v0in = v0[axis] >= l, v1in = v1[axis] >= l, v2in = v2[axis] >= l;
				if (v0in || v1in)
				{
					if (v0in ^ v1in)
					{
						const float f = tinybvh_clamp( (l - v0[axis]) / (v1[axis] - v0[axis]), 0.0f, 1.0f );
						C = v0 + f * (v1 - v0), C[axis] = l /* accurate */, vout[Nout++] = C;
					}
					if (v1in) vout[Nout++] = v1;
				}
				if (v1in || v2in)
				{
					if (v1in ^ v2in)
					{
						const float f = tinybvh_clamp( (l - v1[axis]) / (v2[axis] - v1[axis]), 0.0f, 1.0f );
						C = v1 + f * (v2 - v1), C[axis] = l /* accurate */, vout[Nout++] = C;
					}
					if (v2in) vout[Nout++] = v2;
				}
				if (v2in || v0in)
				{
					if (v2in ^ v0in)
					{
						const float f = tinybvh_clamp( (l - v2[axis]) / (v0[axis] - v2[axis]), 0.0f, 1.0f );
						C = v2 + f * (v0 - v2), C[axis] = l /* accurate */, vout[Nout++] = C;
					}
					if (v0in) vout[Nout++] = v0;
				}
			}
			for (uint32_t v = 0; v < Nout; v++)
			{
				bvhvec3 v0 = vout[v], v1 = vout[(v + 1) % Nout];
				const bool v0in = v0[axis] <= r, v1in = v1[axis] <= r;
				if (!(v0in || v1in)) continue; else if (v0in ^ v1in)
				{
					const float f = tinybvh_clamp( (r - v0[axis]) / (v1[axis] - v0[axis]), 0.0f, 1.0f );
					C = v0 + f * (v1 - v0), C[axis] = r /* accurate */, hasVerts = true;
					mn = tinybvh_min( mn, C ), mx = tinybvh_max( mx, C );
				}
				if (v1in) hasVerts = true, mn = tinybvh_min( mn, v1 ), mx = tinybvh_max( mx, v1 );
			}
		}
		newFrag.bmin = tinybvh_max( mn, bmin ), newFrag.bmax = tinybvh_min( mx, bmax );
		newFrag.primIdx = orig.primIdx, newFrag.clipped = 1;
		return hasVerts;
	}
}

// SplitFrag: cut a fragment in two new fragments.
void BVH::SplitFrag( const Fragment& orig, Fragment& left, Fragment& right, const bvhvec3& minDim, const uint32_t splitAxis, const float splitPos, bool& leftOK, bool& rightOK )
{
	// method: we will split the fragment against the main split axis into two new fragments.
	// In case the original fragment was clipped before, we first clip to the AABB of 'orig'.
	bvhvec3 vin[16], vout[16], vleft[16], vright[16]; // occasionally exceeds 9, but never 12
	uint32_t vidx = orig.primIdx * 3, Nin = 3, Nout = 0, Nleft = 0, Nright = 0;
	if (!vertIdx) vin[0] = verts[vidx], vin[1] = verts[vidx + 1], vin[2] = verts[vidx + 2];
	else vin[0] = verts[vertIdx[vidx]], vin[1] = verts[vertIdx[vidx + 1]], vin[2] = verts[vertIdx[vidx + 2]];
	const bvhvec3 extent = orig.bmax - orig.bmin;
	if (orig.clipped) for (int a = 0; a < 3; a++) if (extent.cell[a] > minDim.cell[a])
	{
		float l = orig.bmin.cell[a], r = orig.bmax.cell[a];
		Nout = 0;
		for (uint32_t v = 0; v < Nin; v++)
		{
			bvhvec3 v0 = vin[v], v1 = vin[(v + 1) % Nin];
			const bool v0in = v0[a] >= l, v1in = v1[a] >= l;
			if (!(v0in || v1in)) continue; else if (v0in ^ v1in)
			{
				const float f = tinybvh_clamp( (l - v0[a]) / (v1[a] - v0[a]), 0.0f, 1.0f );
				bvhvec3 C = v0 + f * (v1 - v0);
				C[a] = l /* accurate */, vout[Nout++] = C;
			}
			if (v1in) vout[Nout++] = v1;
		}
		Nin = 0;
		for (uint32_t v = 0; v < Nout; v++)
		{
			bvhvec3 v0 = vout[v], v1 = vout[(v + 1) % Nout];
			const bool v0in = v0[a] <= r, v1in = v1[a] <= r;
			if (!(v0in || v1in)) continue; else if (v0in ^ v1in)
			{
				const float f = tinybvh_clamp( (r - v0[a]) / (v1[a] - v0[a]), 0.0f, 1.0f );
				bvhvec3 C = v0 + f * (v1 - v0);
				C[a] = r /* accurate */, vin[Nin++] = C;
			}
			if (v1in) vin[Nin++] = v1;
		}
	}
	for (uint32_t v = 0; v < Nin; v++)
	{
		bvhvec3 v0 = vin[v], v1 = vin[(v + 1) % Nin];
		bool v0left = v0[splitAxis] < splitPos, v1left = v1[splitAxis] < splitPos;
		if (v0left && v1left) vleft[Nleft++] = v1; else if (!v0left && !v1left) vright[Nright++] = v1; else
		{
			const float f = tinybvh_clamp( (splitPos - v0[splitAxis]) / (v1[splitAxis] - v0[splitAxis]), 0.0f, 1.0f );
			bvhvec3 C = v0 + f * (v1 - v0);
			C[splitAxis] = splitPos;
			if (v0left) vleft[Nleft++] = vright[Nright++] = C, vright[Nright++] = v1;
			else vright[Nright++] = vleft[Nleft++] = C, vleft[Nleft++] = v1;
		}
	}
	// calculate left and right fragments
	left.bmin = right.bmin = bvhvec3( BVH_FAR ), left.bmax = right.bmax = bvhvec3( -BVH_FAR );
	for (uint32_t i = 0; i < Nleft; i++)
		left.bmin = tinybvh_min( left.bmin, vleft[i] ),
		left.bmax = tinybvh_max( left.bmax, vleft[i] );
	for (uint32_t i = 0; i < Nright; i++)
		right.bmin = tinybvh_min( right.bmin, vright[i] ),
		right.bmax = tinybvh_max( right.bmax, vright[i] );
	left.clipped = right.clipped = 1, left.primIdx = right.primIdx = orig.primIdx;
	leftOK = Nleft > 0, rightOK = Nright > 0;
}

// RefitUp: Update bounding boxes of ancestors of the specified node.
void BVH_Verbose::RefitUp( uint32_t nodeIdx )
{
	while (1)
	{
		BVHNode& node = bvhNode[nodeIdx];
		if (!node.isLeaf())
		{
			const BVHNode& left = bvhNode[node.left];
			const BVHNode& right = bvhNode[node.right];
			node.aabbMin = tinybvh_min( left.aabbMin, right.aabbMin );
			node.aabbMax = tinybvh_max( left.aabbMax, right.aabbMax );
		}
		if (nodeIdx == 0) break; else nodeIdx = node.parent;
	}
}

// SAHCostUp: Calculate the SAH cost of a node and its ancestry
float BVH_Verbose::SAHCostUp( uint32_t nodeIdx ) const
{
	float sum = 0;
	while (nodeIdx != 0xffffffff)
	{
		BVHNode& node = bvhNode[nodeIdx];
		sum += BVH::SA( node.aabbMin, node.aabbMax );
		nodeIdx = node.parent;
	}
	return sum;
}

// FindBestNewPosition
// Part of "Fast Insertion-Based Optimization of Bounding Volume Hierarchies"
// K.I.S.S. version with brute-force array search.
uint32_t BVH_Verbose::FindBestNewPosition( const uint32_t Lid ) const
{
	struct Task { float ci; uint32_t node; };
	ALIGNED( 64 ) Task task[512];
	float Cbest = BVH_FAR;
	int tasks = 1 /* doesn't exceed 70 for Crytek Sponza */, Xbest = 0;
	const BVHNode& L = bvhNode[Lid];
	// reinsert L into BVH
	task[0].node = 0, task[0].ci = 0;
	while (tasks > 0)
	{
		// 'pop' task with smallest Ci
		uint32_t bestTask = 0;
		float minCi = task[0].ci; // tnx Brian
		for (int j = 1; j < tasks; j++) if (task[j].ci < minCi) minCi = task[j].ci, bestTask = j;
		const uint32_t Xid = task[bestTask].node;
		const float CiLX = task[bestTask].ci;
		if (--tasks > 0) task[bestTask] = task[tasks];
		// execute task
		const BVHNode& X = bvhNode[Xid];
		if (CiLX + L.SA() >= Cbest) break;
		const float CdLX = SA( tinybvh_min( L.aabbMin, X.aabbMin ), tinybvh_max( L.aabbMax, X.aabbMax ) );
		const float CLX = CiLX + CdLX;
		if (CLX < Cbest && Xid != 0) Cbest = CLX, Xbest = Xid;
		const float Ci = CLX - X.SA();
		if (Ci + L.SA() >= Cbest || X.isLeaf()) continue;
		task[tasks].node = X.left, task[tasks++].ci = Ci;
		task[tasks].node = X.right, task[tasks++].ci = Ci;
	}
	return Xbest;
}

// Determine for each node in the tree the number of primitives
// stored in that subtree. Helper function for MergeLeafs.
uint32_t BVH_Verbose::CountSubtreeTris( const uint32_t nodeIdx, uint32_t* counters )
{
	BVHNode& node = bvhNode[nodeIdx];
	uint32_t result = node.triCount;
	if (!result)
		result = CountSubtreeTris( node.left, counters ) + CountSubtreeTris( node.right, counters );
	counters[nodeIdx] = result;
	return result;
}

// Write the triangle indices stored in a subtree to a continuous
// slice in the 'newIdx' array. Helper function for MergeLeafs.
void BVH_Verbose::MergeSubtree( const uint32_t nodeIdx, uint32_t* newIdx, uint32_t& newIdxPtr )
{
	BVHNode& node = bvhNode[nodeIdx];
	if (node.isLeaf())
	{
		memcpy( newIdx + newIdxPtr, primIdx + node.firstTri, node.triCount * 4 );
		newIdxPtr += node.triCount;
	}
	else
	{
		MergeSubtree( node.left, newIdx, newIdxPtr );
		MergeSubtree( node.right, newIdx, newIdxPtr );
	}
}
} // namespace tinybvh

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

#endif // TINYBVH_IMPLEMENTATION