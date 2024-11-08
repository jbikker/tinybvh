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

// Nov 08, '24: version 0.3.0
// Oct 30, '24: version 0.1.0 : Initial release.
// Oct 29, '24: version 0.0.1 : Establishing interface.
//

//
// Use this in *one* .c or .cpp
//   #define TINYBVH_IMPLEMENTATION
//   #include "tiny_bvh.h"
//

// How to use:
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

// Further references:
// - Parallel Spatial Splits in Bounding Volume Hierarchies:
//   https://diglib.eg.org/items/f55715b1-9e56-4b40-af73-59d3dfba9fe7
// - Heuristics for ray tracing using space subdivision:
//   https://graphicsinterface.org/wp-content/uploads/gi1989-22.pdf
// - Heuristic Ray Shooting Algorithms:
//   https://dcgi.fel.cvut.cz/home/havran/DISSVH/phdthesis.html

// Author and contributors:
// Jacco Bikker: BVH code and examples
// Eddy L O Jansson: g++ / clang support
// Aras Pranckevičius: non-Intel architecture support
// Jefferson Amstutz: CMake surpport

#ifndef TINY_BVH_H_
#define TINY_BVH_H_

// binned BVH building: bin count
#define BVHBINS 8

// include fast AVX BVH builder
#if defined(__x86_64__) || defined(_M_X64)
#define BVH_USEAVX
#endif

// library version
#define TINY_BVH_VERSION_MAJOR	0
#define TINY_BVH_VERSION_MINOR	3
#define TINY_BVH_VERSION_SUB	1

// ============================================================================
//
//        P R E L I M I N A R I E S
// 
// ============================================================================

// aligned memory allocation
#ifdef _MSC_VER
// Visual Studio / C11
#include <malloc.h>
#include <math.h> // for sqrtf, fabs
#include <string.h> // for memset
#define ALIGNED( x ) __declspec( align( x ) )
#define ALIGNED_MALLOC( x ) ( ( x ) == 0 ? 0 : _aligned_malloc( ( x ), 64 ) )
#define ALIGNED_FREE( x ) _aligned_free( x )
#define OPERATOR_NEW_ALIGN(x)
#define OPERATOR_DELETE_ALIGN(x)
#else
// gcc / clang
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <new> // for std::align_val_t
#define ALIGNED( x ) __attribute__( ( aligned( x ) ) )
#if defined(__x86_64__) || defined(_M_X64)
// https://stackoverflow.com/questions/32612881/why-use-mm-malloc-as-opposed-to-aligned-malloc-alligned-alloc-or-posix-mem
#include <xmmintrin.h>
#define ALIGNED_MALLOC( x ) ( ( x ) == 0 ? 0 : _mm_malloc( ( x ), 64 ) )
#define ALIGNED_FREE( x ) _mm_free( x )
#else
#define ALIGNED_MALLOC( x ) ( ( x ) == 0 ? 0 : aligned_alloc( 64, ( x ) ) )
#define ALIGNED_FREE( x ) free( x )
#endif
#define OPERATOR_NEW_ALIGN(x) (std::align_val_t(x))
#define OPERATOR_DELETE_ALIGN(x) ,(std::align_val_t(x))
#endif
// generic
#ifdef BVH_USEAVX
#include "immintrin.h" // for __m128 and __m256
#endif

namespace tinybvh {

#ifdef _MSC_VER
// Suppress a warning caused by the union of x,y,.. and cell[..] in vectors.
// We need this union to address vector components either by name or by index.
// The warning is re-enabled right after the definition of the data types.
#pragma warning ( push )
#pragma warning ( disable: 4201 /* nameless struct / union */ )
#endif

struct bvhvec3;
struct ALIGNED( 16 ) bvhvec4
{
	// vector naming is designed to not cause any name clashes.
	bvhvec4() = default;
	bvhvec4( const float a, const float b, const float c, const float d ) : x( a ), y( b ), z( c ), w( d ) {}
	bvhvec4( const float a ) : x( a ), y( a ), z( a ), w( a ) {}
	bvhvec4( const bvhvec3 & a );
	bvhvec4( const bvhvec3 & a, float b );
	float& operator [] ( const int i ) { return cell[i]; }
	union { struct { float x, y, z, w; }; float cell[4]; };
};

struct ALIGNED( 8 ) bvhvec2
{
	bvhvec2() = default;
	bvhvec2( const float a, const float b ) : x( a ), y( b ) {}
	bvhvec2( const float a ) : x( a ), y( a ) {}
	bvhvec2( const bvhvec4 a ) : x( a.x ), y( a.y ) {}
	float& operator [] ( const int i ) { return cell[i]; }
	union { struct { float x, y; }; float cell[2]; };
};

struct bvhvec3
{
	bvhvec3() = default;
	bvhvec3( const float a, const float b, const float c ) : x( a ), y( b ), z( c ) {}
	bvhvec3( const float a ) : x( a ), y( a ), z( a ) {}
	bvhvec3( const bvhvec4 a ) : x( a.x ), y( a.y ), z( a.z ) {}
	float halfArea() { return x < -1e30f ? 0 : (x * y + y * z + z * x); } // for SAH calculations
	float& operator [] ( const int i ) { return cell[i]; }
	union { struct { float x, y, z; }; float cell[3]; };
};
struct bvhint3
{
	bvhint3() = default;
	bvhint3( const int a, const int b, const int c ) : x( a ), y( b ), z( c ) {}
	bvhint3( const int a ) : x( a ), y( a ), z( a ) {}
	bvhint3( const bvhvec3& a ) { x = (int)a.x, y = (int)a.y, z = (int)a.z; }
	int& operator [] ( const int i ) { return cell[i]; }
	union { struct { int x, y, z; }; int cell[3]; };
};

#ifdef TINYBVH_IMPLEMENTATION
bvhvec4::bvhvec4( const bvhvec3& a ) { x = a.x; y = a.y; z = a.z; w = 0; }
bvhvec4::bvhvec4( const bvhvec3& a, float b ) { x = a.x; y = a.y; z = a.z; w = b; }
#endif

#ifdef _MSC_VER
#pragma warning ( pop )
#endif

// Math operations.
// Note: Since this header file is expected to be included in a source file
// of a separate project, the static keyword doesn't provide sufficient
// isolation; hence the tinybvh_ prefix.
inline float tinybvh_safercp( const float x ) { return x > 1e-12f ? (1.0f / x) : (x < -1e-12f ? (1.0f / x) : 1e30f); }
inline bvhvec3 tinybvh_safercp( const bvhvec3 a ) { return bvhvec3( tinybvh_safercp( a.x ), tinybvh_safercp( a.y ), tinybvh_safercp( a.z ) ); }
static inline float tinybvh_min( const float a, const float b ) { return a < b ? a : b; }
static inline float tinybvh_max( const float a, const float b ) { return a > b ? a : b; }
static inline int tinybvh_min( const int a, const int b ) { return a < b ? a : b; }
static inline int tinybvh_max( const int a, const int b ) { return a > b ? a : b; }
static inline bvhvec2 tinybvh_min( const bvhvec2& a, const bvhvec2& b ) { return bvhvec2( tinybvh_min( a.x, b.x ), tinybvh_min( a.y, b.y ) ); }
static inline bvhvec3 tinybvh_min( const bvhvec3& a, const bvhvec3& b ) { return bvhvec3( tinybvh_min( a.x, b.x ), tinybvh_min( a.y, b.y ), tinybvh_min( a.z, b.z ) ); }
static inline bvhvec4 tinybvh_min( const bvhvec4& a, const bvhvec4& b ) { return bvhvec4( tinybvh_min( a.x, b.x ), tinybvh_min( a.y, b.y ), tinybvh_min( a.z, b.z ), tinybvh_min( a.w, b.w ) ); }
static inline bvhvec2 tinybvh_max( const bvhvec2& a, const bvhvec2& b ) { return bvhvec2( tinybvh_max( a.x, b.x ), tinybvh_max( a.y, b.y ) ); }
static inline bvhvec3 tinybvh_max( const bvhvec3& a, const bvhvec3& b ) { return bvhvec3( tinybvh_max( a.x, b.x ), tinybvh_max( a.y, b.y ), tinybvh_max( a.z, b.z ) ); }
static inline bvhvec4 tinybvh_max( const bvhvec4& a, const bvhvec4& b ) { return bvhvec4( tinybvh_max( a.x, b.x ), tinybvh_max( a.y, b.y ), tinybvh_max( a.z, b.z ), tinybvh_max( a.w, b.w ) ); }
static inline float tinybvh_clamp( const float x, const float a, const float b ) { return x < a ? a : (x > b ? b : x); }
static inline int tinybvh_clamp( const int x, const int a, const int b ) { return x < a ? a : (x > b ? b : x); }
template <class T> inline static void tinybvh_swap( T& a, T& b ) { T t = a; a = b; b = t; }

// Operator overloads.
// Only a minimal set is provided.
inline bvhvec2 operator-( const bvhvec2& a ) { return bvhvec2( -a.x, -a.y ); }
inline bvhvec3 operator-( const bvhvec3& a ) { return bvhvec3( -a.x, -a.y, -a.z ); }
inline bvhvec4 operator-( const bvhvec4& a ) { return bvhvec4( -a.x, -a.y, -a.z, -a.w ); }
inline bvhvec2 operator+( const bvhvec2& a, const bvhvec2& b ) { return bvhvec2( a.x + b.x, a.y + b.y ); }
inline bvhvec3 operator+( const bvhvec3& a, const bvhvec3& b ) { return bvhvec3( a.x + b.x, a.y + b.y, a.z + b.z ); }
inline bvhvec4 operator+( const bvhvec4& a, const bvhvec4& b ) { return bvhvec4( a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w ); }
inline bvhvec2 operator-( const bvhvec2& a, const bvhvec2& b ) { return bvhvec2( a.x - b.x, a.y - b.y ); }
inline bvhvec3 operator-( const bvhvec3& a, const bvhvec3& b ) { return bvhvec3( a.x - b.x, a.y - b.y, a.z - b.z ); }
inline bvhvec4 operator-( const bvhvec4& a, const bvhvec4& b ) { return bvhvec4( a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w ); }
inline void operator+=( bvhvec2& a, const bvhvec2& b ) { a.x += b.x;	a.y += b.y; }
inline void operator+=( bvhvec3& a, const bvhvec3& b ) { a.x += b.x;	a.y += b.y;	a.z += b.z; }
inline void operator+=( bvhvec4& a, const bvhvec4& b ) { a.x += b.x;	a.y += b.y;	a.z += b.z;	a.w += b.w; }
inline bvhvec2 operator*( const bvhvec2& a, const bvhvec2& b ) { return bvhvec2( a.x * b.x, a.y * b.y ); }
inline bvhvec3 operator*( const bvhvec3& a, const bvhvec3& b ) { return bvhvec3( a.x * b.x, a.y * b.y, a.z * b.z ); }
inline bvhvec4 operator*( const bvhvec4& a, const bvhvec4& b ) { return bvhvec4( a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w ); }
inline bvhvec2 operator*( const bvhvec2& a, float b ) { return bvhvec2( a.x * b, a.y * b ); }
inline bvhvec2 operator*( float b, const bvhvec2& a ) { return bvhvec2( b * a.x, b * a.y ); }
inline bvhvec3 operator*( const bvhvec3& a, float b ) { return bvhvec3( a.x * b, a.y * b, a.z * b ); }
inline bvhvec3 operator*( float b, const bvhvec3& a ) { return bvhvec3( b * a.x, b * a.y, b * a.z ); }
inline bvhvec4 operator*( const bvhvec4& a, float b ) { return bvhvec4( a.x * b, a.y * b, a.z * b, a.w * b ); }
inline bvhvec4 operator*( float b, const bvhvec4& a ) { return bvhvec4( b * a.x, b * a.y, b * a.z, b * a.w ); }
inline bvhvec2 operator/( float b, const bvhvec2& a ) { return bvhvec2( b / a.x, b / a.y ); }
inline bvhvec3 operator/( float b, const bvhvec3& a ) { return bvhvec3( b / a.x, b / a.y, b / a.z ); }
inline bvhvec4 operator/( float b, const bvhvec4& a ) { return bvhvec4( b / a.x, b / a.y, b / a.z, b / a.w ); }

// Vector math: cross and dot.
static inline bvhvec3 cross( const bvhvec3& a, const bvhvec3& b )
{
	return bvhvec3( a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x );
}
static inline float dot( const bvhvec2& a, const bvhvec2& b ) { return a.x * b.x + a.y * b.y; }
static inline float dot( const bvhvec3& a, const bvhvec3& b ) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline float dot( const bvhvec4& a, const bvhvec4& b ) { return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w; }

// Vector math: common operations.
static float length( const bvhvec3& a ) { return sqrtf( a.x * a.x + a.y * a.y + a.z * a.z ); }
static bvhvec3 normalize( const bvhvec3& a )
{
	float l = length( a ), rl = l == 0 ? 0 : (1.0f / l);
	return a * rl;
}

// SIMD typedef, helps keeping the interface generic
#ifdef BVH_USEAVX
typedef __m128 SIMDVEC4;
#define SIMD_SETVEC(a,b,c,d) _mm_set_ps( a, b, c, d )
#define SIMD_SETRVEC(a,b,c,d) _mm_set_ps( d, c, b, a )
#else
typedef bvhvec4 SIMDVEC4;
#define SIMD_SETVEC(a,b,c,d) bvhvec4( d, c, b, a )
#define SIMD_SETRVEC(a,b,c,d) bvhvec4( a, b, c, d )
#endif

// ============================================================================
//
//        R A Y   T R A C I N G   S T R U C T S  /  C L A S S E S
// 
// ============================================================================

struct Intersection
{
	// An intersection result is designed to fit in no more than
	// four 32-bit values. This allows efficient storage of a result in
	// GPU code. The obvious missing result is an instance id; consider
	// squeezing this in the 'prim' field in some way.
	// Using this data and the original triangle data, all other info for
	// shading (such as normal, texture color etc.) can be reconstructed.
	float t, u, v;		// distance along ray & barycentric coordinates of the intersection
	unsigned int prim;	// primitive index
};

struct Ray
{
	// Basic ray class. Note: For single blas traversal it is expected
	// that Ray::rD is properly initialized. For tlas/blas traversal this
	// field is typically updated for each blas.
	Ray() = default;
	Ray( bvhvec3 origin, bvhvec3 direction, float t = 1e30f )
	{
		memset( this, 0, sizeof( Ray ) );
		O = origin, D = normalize( direction ), rD = tinybvh_safercp( D );
		hit.t = t;
	}
	ALIGNED( 16 ) bvhvec3 O; unsigned int dummy1;
	ALIGNED( 16 ) bvhvec3 D; unsigned int dummy2;
	ALIGNED( 16 ) bvhvec3 rD; unsigned int dummy3;
	ALIGNED( 16 ) Intersection hit;
};

class BVH
{
public:
	enum BVHLayout { WALD_32BYTE = 1, AILA_LAINE, ALT_SOA };
	struct BVHNode
	{
		// 'Traditional' 32-byte BVH node layout, as proposed by Ingo Wald.
		// When aligned to a cache line boundary, two of these fit together.
		bvhvec3 aabbMin; unsigned int leftFirst; // 16 bytes
		bvhvec3 aabbMax; unsigned int triCount;	// 16 bytes, total: 32 bytes
		bool isLeaf() const { return triCount > 0; /* empty BVH leaves do not exist */ }
		float Intersect( const Ray& ray ) const { return BVH::IntersectAABB( ray, aabbMin, aabbMax ); }
		float SurfaceArea() const { return BVH::SA( aabbMin, aabbMax ); }
		float CalculateNodeCost() const { return SurfaceArea() * triCount; }
	};
	struct BVHNodeAlt
	{
		// Alternative 64-byte BVH node layout, which specifies the bounds of
		// the children rather than the node itself. This layout is used by
		// Aila and Laine in their seminal GPU ray tracing paper.
		bvhvec3 lmin; unsigned int left;
		bvhvec3 lmax; unsigned int right;
		bvhvec3 rmin; unsigned int triCount;
		bvhvec3 rmax; unsigned int firstTri; // total: 64 bytes
		bool isLeaf() const { return triCount > 0; }
	};
	struct BVHNodeAlt2
	{
		// Second alternative 64-byte BVH node layout, same as BVHNodeAlt but
		// with child AABBs stored in SoA order.
		SIMDVEC4 xxxx, yyyy, zzzz;
		unsigned int left, right, triCount, firstTri; // total: 64 bytes
		bool isLeaf() const { return triCount > 0; }
	};
	struct Fragment
	{
		// A fragment stores the bounds of an input primitive. The name 'Fragment' is from
		// "Parallel Spatial Splits in Bounding Volume Hierarchies", 2016, Fuetterling et al.,
		// and refers to the potential splitting of these boxes for SBVH construction.
		bvhvec3 bmin;				// AABB min x, y and z
		unsigned int primIdx;		// index of the original primitive
		bvhvec3 bmax;				// AABB max x, y and z
		unsigned int clipped = 0;	// Fragment is the result of clipping if > 0.
		bool validBox() { return bmin.x < 1e30f; }
	};
	BVH() = default;
	~BVH()
	{
		ALIGNED_FREE( bvhNode );
		delete[] triIdx;
		operator delete[](fragment OPERATOR_DELETE_ALIGN(32));
		bvhNode = 0, triIdx = 0, fragment = 0;
	}
	float SAHCost( const unsigned int nodeIdx = 0 ) const
	{
		// Determine the SAH cost of the tree. This provides an indication
		// of the quality of the BVH: Lower is better.
		const BVHNode& n = bvhNode[nodeIdx];
		if (n.isLeaf()) return 2.0f * n.SurfaceArea() * n.triCount;
		float cost = 3.0f * n.SurfaceArea() + SAHCost( n.leftFirst ) + SAHCost( n.leftFirst + 1 );
		return nodeIdx == 0 ? (cost / n.SurfaceArea()) : cost;
	}
	int NodeCount( const unsigned int nodeIdx = 0 ) const
	{
		// Determine the number of nodes in the tree. Typically the result should
		// be newNodePtr - 1 (second node is always unused), but some builders may 
		// have unused nodes besides node 1.
		const BVHNode& n = bvhNode[nodeIdx];
		unsigned int retVal = 1;
		if (!n.isLeaf()) retVal += NodeCount( n.leftFirst ) + NodeCount( n.leftFirst + 1 );
		return retVal;
	}
	void Build( const bvhvec4* vertices, const unsigned int primCount );
	void BuildAVX( const bvhvec4* vertices, const unsigned int primCount );
	void Convert( BVHLayout from, BVHLayout to, bool deleteOriginal = false );
	void Refit();
	int Intersect( Ray& ray, BVHLayout layout = WALD_32BYTE ) const;
	void Intersect256Rays( Ray* first ) const;
	void Intersect256RaysSSE( Ray* packet ) const; // requires BVH_USEAVX
private:
	int Intersect_Wald32Byte( Ray& ray ) const;
	int Intersect_AilaLaine( Ray& ray ) const;
	int Intersect_AltSoA( Ray& ray ) const; // requires BVH_USEAVX
	void IntersectTri( Ray& ray, const unsigned int triIdx ) const;
	static float IntersectAABB( const Ray& ray, const bvhvec3& aabbMin, const bvhvec3& aabbMax );
	static float SA( const bvhvec3& aabbMin, const bvhvec3& aabbMax )
	{
		bvhvec3 e = aabbMax - aabbMin; // extent of the node
		return e.x * e.y + e.y * e.z + e.z * e.x;
	}
public:
	bvhvec4* verts = 0;				// pointer to input primitive array: 3x16 bytes per tri
	unsigned int triCount = 0;		// number of primitives in tris
	Fragment* fragment = 0;			// input primitive bounding boxes
	unsigned int* triIdx = 0;		// primitive index array
	unsigned int idxCount = 0;		// number of indices in triIdx. May exceed triCount * 3 for SBVH.
	unsigned int newNodePtr = 0;	// number of reserved nodes
	BVHNode* bvhNode = 0;			// BVH node pool, Wald 32-byte format. Root is always in node 0.
	BVHNodeAlt* altNode = 0;		// BVH node in Aila & Laine format.
	BVHNodeAlt2* alt2Node = 0;		// BVH node in Aila & Laine (SoA version) format.
};

} // namespace tinybvh

// ============================================================================
//
//        I M P L E M E N T A T I O N
// 
// ============================================================================

#ifdef TINYBVH_IMPLEMENTATION

#include <assert.h>			// for assert

namespace tinybvh {

// Basic single-function binned-SAH-builder. 
// This is the reference builder; it yields a decent tree suitable for ray 
// tracing on the CPU. This code uses no SIMD instructions. 
// Faster code, using SSE/AVX, is available for x64 CPUs.
// For GPU rendering: The resulting BVH should be converted to a more optimal
// format after construction.
void BVH::Build( const bvhvec4* vertices, const unsigned int primCount )
{
	// allocate on first build
	if (!bvhNode)
	{
		idxCount = primCount;
		triCount = primCount;
		bvhNode = (BVHNode*)ALIGNED_MALLOC( triCount * 2 * sizeof( BVHNode ) );
		memset( &bvhNode[1], 0, 32 );	// node 1 remains unused, for cache line alignment.
		triIdx = new unsigned int[triCount];
		verts = (bvhvec4*)vertices;		// note: we're not copying this data; don't delete.
		fragment = new OPERATOR_NEW_ALIGN(32) Fragment[triCount];
	}
	else assert( triCount == primCount ); // don't change triangle count between builds.
	// reset node pool
	newNodePtr = 2;
	// assign all triangles to the root node
	BVHNode& root = bvhNode[0];
	root.leftFirst = 0, root.triCount = triCount, root.aabbMin = bvhvec3( 1e30f ), root.aabbMax = bvhvec3( -1e30f );
	// initialize fragments and initialize root node bounds
	for (unsigned int i = 0; i < triCount; i++)
	{
		fragment[i].bmin = tinybvh_min( tinybvh_min( verts[i * 3], verts[i * 3 + 1] ), verts[i * 3 + 2] );
		fragment[i].bmax = tinybvh_max( tinybvh_max( verts[i * 3], verts[i * 3 + 1] ), verts[i * 3 + 2] );
		root.aabbMin = tinybvh_min( root.aabbMin, fragment[i].bmin );
		root.aabbMax = tinybvh_max( root.aabbMax, fragment[i].bmax ), triIdx[i] = i;
	}
	// subdivide recursively
	unsigned int task[256], taskCount = 0, nodeIdx = 0;
	bvhvec3 minDim = (root.aabbMax - root.aabbMin) * 1e-20f, bestLMin = 0, bestLMax = 0, bestRMin = 0, bestRMax = 0;
	while (1)
	{
		while (1)
		{
			BVHNode& node = bvhNode[nodeIdx];
			// find optimal object split
			bvhvec3 binMin[3][BVHBINS], binMax[3][BVHBINS];
			for (unsigned int a = 0; a < 3; a++) for (unsigned int i = 0; i < BVHBINS; i++) binMin[a][i] = 1e30f, binMax[a][i] = -1e30f;
			unsigned int count[3][BVHBINS];
			memset( count, 0, BVHBINS * 3 * sizeof( unsigned int ) );
			const bvhvec3 rpd3 = bvhvec3( BVHBINS / (node.aabbMax - node.aabbMin) ), nmin3 = node.aabbMin;
			for (unsigned int i = 0; i < node.triCount; i++) // process all tris for x,y and z at once
			{
				const unsigned int fi = triIdx[node.leftFirst + i];
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
			float splitCost = 1e30f;
			unsigned int bestAxis = 0, bestPos = 0;
			for (int a = 0; a < 3; a++) if ((node.aabbMax[a] - node.aabbMin[a]) > minDim[a])
			{
				bvhvec3 lBMin[BVHBINS - 1], rBMin[BVHBINS - 1], l1 = 1e30f, l2 = -1e30f;
				bvhvec3 lBMax[BVHBINS - 1], rBMax[BVHBINS - 1], r1 = 1e30f, r2 = -1e30f;
				float ANL[BVHBINS - 1], ANR[BVHBINS - 1];
				for (unsigned int lN = 0, rN = 0, i = 0; i < BVHBINS - 1; i++)
				{
					lBMin[i] = l1 = tinybvh_min( l1, binMin[a][i] );
					rBMin[BVHBINS - 2 - i] = r1 = tinybvh_min( r1, binMin[a][BVHBINS - 1 - i] );
					lBMax[i] = l2 = tinybvh_max( l2, binMax[a][i] );
					rBMax[BVHBINS - 2 - i] = r2 = tinybvh_max( r2, binMax[a][BVHBINS - 1 - i] );
					lN += count[a][i], rN += count[a][BVHBINS - 1 - i];
					ANL[i] = lN == 0 ? 1e30f : ((l2 - l1).halfArea() * (float)lN);
					ANR[BVHBINS - 2 - i] = rN == 0 ? 1e30f : ((r2 - r1).halfArea() * (float)rN);
				}
				// evaluate bin totals to find best position for object split
				for (unsigned int i = 0; i < BVHBINS - 1; i++)
				{
					const float C = ANL[i] + ANR[i];
					if (C < splitCost)
					{
						splitCost = C, bestAxis = a, bestPos = i;
						bestLMin = lBMin[i], bestRMin = rBMin[i], bestLMax = lBMax[i], bestRMax = rBMax[i];
					}
				}
			}
			if (splitCost >= node.CalculateNodeCost()) break; // not splitting is better.
			// in-place partition
			unsigned int j = node.leftFirst + node.triCount, src = node.leftFirst;
			const float rpd = rpd3.cell[bestAxis], nmin = nmin3.cell[bestAxis];
			for (unsigned int i = 0; i < node.triCount; i++)
			{
				const unsigned int fi = triIdx[src];
				int bi = (unsigned int)(((fragment[fi].bmin[bestAxis] + fragment[fi].bmax[bestAxis]) * 0.5f - nmin) * rpd);
				bi = tinybvh_clamp( bi, 0, BVHBINS - 1 );
				if ((unsigned int)bi <= bestPos) src++; else tinybvh_swap( triIdx[src], triIdx[--j] );
			}
			// create child nodes
			unsigned int leftCount = src - node.leftFirst, rightCount = node.triCount - leftCount;
			if (leftCount == 0 || rightCount == 0) break; // should not happen.
			const int lci = newNodePtr++, rci = newNodePtr++;
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
}

void BVH::Convert( BVHLayout from, BVHLayout to, bool deleteOriginal )
{
	if (from == WALD_32BYTE && to == AILA_LAINE)
	{
		// allocate space
		altNode = (BVHNodeAlt*)ALIGNED_MALLOC( sizeof( BVHNodeAlt ) * newNodePtr );
		memset( altNode, 0, sizeof( BVHNodeAlt ) * newNodePtr );
		// recursively convert nodes
		unsigned int newAltNode = 0, nodeIdx = 0, stack[128], stackPtr = 0;
		while (1)
		{
			const BVHNode& node = bvhNode[nodeIdx];
			const unsigned int idx = newAltNode++;
			if (node.isLeaf())
			{
				altNode[idx].triCount = node.triCount;
				altNode[idx].firstTri = node.leftFirst;
				if (!stackPtr) break;
				nodeIdx = stack[--stackPtr];
				unsigned int newNodeParent = stack[--stackPtr];
				altNode[newNodeParent].right = newAltNode;
			}
			else
			{
				const BVHNode& left = bvhNode[node.leftFirst];
				const BVHNode& right = bvhNode[node.leftFirst + 1];
				altNode[idx].lmin = left.aabbMin, altNode[idx].rmin = right.aabbMin;
				altNode[idx].lmax = left.aabbMax, altNode[idx].rmax = right.aabbMax;
				altNode[idx].left = newAltNode; // right will be filled when popped
				stack[stackPtr++] = idx;
				stack[stackPtr++] = node.leftFirst + 1;
				nodeIdx = node.leftFirst;
			}
		}
	}
	else if (from == WALD_32BYTE && to == ALT_SOA)
	{
		// allocate space
		alt2Node = (BVHNodeAlt2*)ALIGNED_MALLOC( sizeof( BVHNodeAlt2 ) * newNodePtr );
		memset( alt2Node, 0, sizeof( BVHNodeAlt2 ) * newNodePtr );
		// recursively convert nodes
		unsigned int newAlt2Node = 0, nodeIdx = 0, stack[128], stackPtr = 0;
		while (1)
		{
			const BVHNode& node = bvhNode[nodeIdx];
			const unsigned int idx = newAlt2Node++;
			if (node.isLeaf())
			{
				alt2Node[idx].triCount = node.triCount;
				alt2Node[idx].firstTri = node.leftFirst;
				if (!stackPtr) break;
				nodeIdx = stack[--stackPtr];
				unsigned int newNodeParent = stack[--stackPtr];
				alt2Node[newNodeParent].right = newAlt2Node;
			}
			else
			{
				const BVHNode& left = bvhNode[node.leftFirst];
				const BVHNode& right = bvhNode[node.leftFirst + 1];
				// This BVH layout requires BVH_USEAVX for traversal, but at least we
				// can convert to it without SSE/AVX/NEON support.
				alt2Node[idx].xxxx = SIMD_SETRVEC( left.aabbMin.x, left.aabbMax.x, right.aabbMin.x, right.aabbMax.x );
				alt2Node[idx].yyyy = SIMD_SETRVEC( left.aabbMin.y, left.aabbMax.y, right.aabbMin.y, right.aabbMax.y );
				alt2Node[idx].zzzz = SIMD_SETRVEC( left.aabbMin.z, left.aabbMax.z, right.aabbMin.z, right.aabbMax.z );
				alt2Node[idx].left = newAlt2Node; // right will be filled when popped
				stack[stackPtr++] = idx;
				stack[stackPtr++] = node.leftFirst + 1;
				nodeIdx = node.leftFirst;
			}
		}
	}
	else
	{
		// For now all other conversions are invalid.
		assert( false );
	}
}

// Refitting: For animated meshes, where the topology remains intact. This
// includes trees waving in the wind, or subsequent frames for skinned
// animations. Repeated refitting tends to lead to deteriorated BVHs and
// slower ray tracing. Rebuild when this happens.
void BVH::Refit()
{
	for (int i = newNodePtr - 1; i >= 0; i--)
	{
		BVHNode& node = bvhNode[i];
		if (node.isLeaf()) // leaf: adjust to current triangle vertex positions
		{
			bvhvec4 aabbMin( 1e30f ), aabbMax( -1e30f );
			for (unsigned int first = node.leftFirst, j = 0; j < node.triCount; j++)
			{
				const unsigned int vertIdx = triIdx[first + j] * 3;
				aabbMin = tinybvh_min( aabbMin, verts[vertIdx] ), aabbMax = tinybvh_max( aabbMax, verts[vertIdx] );
				aabbMin = tinybvh_min( aabbMin, verts[vertIdx + 1] ), aabbMax = tinybvh_max( aabbMax, verts[vertIdx + 1] );
				aabbMin = tinybvh_min( aabbMin, verts[vertIdx + 2] ), aabbMax = tinybvh_max( aabbMax, verts[vertIdx + 2] );
			}
			node.aabbMin = aabbMin, node.aabbMax = aabbMax;
			continue;
		}
		// interior node: adjust to child bounds
		const BVHNode& left = bvhNode[node.leftFirst], & right = bvhNode[node.leftFirst + 1];
		node.aabbMin = tinybvh_min( left.aabbMin, right.aabbMin );
		node.aabbMax = tinybvh_max( left.aabbMax, right.aabbMax );
	}
}

// Intersect a BVH with a ray.
// This function returns the intersection details in Ray::hit. Additionally,
// the number of steps through the BVH is returned. Visualize this to get a
// visual impression of the structure of the BVH.
int BVH::Intersect( Ray& ray, BVHLayout layout ) const
{
	switch (layout)
	{
	case WALD_32BYTE:
		return Intersect_Wald32Byte( ray );
		break;
	case AILA_LAINE:
		return Intersect_AilaLaine( ray );
		break;
	case ALT_SOA:
		return Intersect_AltSoA( ray );
		break;
	default:
		assert( false );
	};
	return 0;
}

// Traverse the default BVH layout (WALD_32BYTE).
int BVH::Intersect_Wald32Byte( Ray& ray ) const
{
	assert( bvhNode != 0 );
	BVHNode* node = &bvhNode[0], * stack[64];
	unsigned int stackPtr = 0, steps = 0;
	while (1)
	{
		steps++;
		if (node->isLeaf())
		{
			for (unsigned int i = 0; i < node->triCount; i++) IntersectTri( ray, triIdx[node->leftFirst + i] );
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		BVHNode* child1 = &bvhNode[node->leftFirst];
		BVHNode* child2 = &bvhNode[node->leftFirst + 1];
		float dist1 = child1->Intersect( ray ), dist2 = child2->Intersect( ray );
		if (dist1 > dist2) { tinybvh_swap( dist1, dist2 ); tinybvh_swap( child1, child2 ); }
		if (dist1 == 1e30f /* missed both child nodes */)
		{
			if (stackPtr == 0) break; else node = stack[--stackPtr];
		}
		else /* hit at least one node */
		{
			node = child1; /* continue with the nearest */
			if (dist2 != 1e30f) stack[stackPtr++] = child2; /* push far child */
		}
	}
	return steps;
}

// Traverse the alternative BVH layout (AILA_LAINE).
int BVH::Intersect_AilaLaine( Ray& ray ) const
{
	assert( altNode != 0 );
	BVHNodeAlt* node = &altNode[0], * stack[64];
	unsigned int stackPtr = 0, steps = 0;
	while (1)
	{
		steps++;
		if (node->isLeaf())
		{
			for (unsigned int i = 0; i < node->triCount; i++) IntersectTri( ray, triIdx[node->firstTri + i] );
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		const bvhvec3 lmin = node->lmin - ray.O, lmax = node->lmax - ray.O;
		const bvhvec3 rmin = node->rmin - ray.O, rmax = node->rmax - ray.O;
		float dist1 = 1e30f, dist2 = 1e30f;
		const bvhvec3 t1a = lmin * ray.rD, t2a = lmax * ray.rD;
		const bvhvec3 t1b = rmin * ray.rD, t2b = rmax * ray.rD;
		const float tmina = tinybvh_max( tinybvh_max( tinybvh_min( t1a.x, t2a.x ), tinybvh_min( t1a.y, t2a.y ) ), tinybvh_min( t1a.z, t2a.z ) );
		const float tmaxa = tinybvh_min( tinybvh_min( tinybvh_max( t1a.x, t2a.x ), tinybvh_max( t1a.y, t2a.y ) ), tinybvh_max( t1a.z, t2a.z ) );
		const float tminb = tinybvh_max( tinybvh_max( tinybvh_min( t1b.x, t2b.x ), tinybvh_min( t1b.y, t2b.y ) ), tinybvh_min( t1b.z, t2b.z ) );
		const float tmaxb = tinybvh_min( tinybvh_min( tinybvh_max( t1b.x, t2b.x ), tinybvh_max( t1b.y, t2b.y ) ), tinybvh_max( t1b.z, t2b.z ) );
		if (tmaxa >= tmina && tmina < ray.hit.t && tmaxa >= 0) dist1 = tmina;
		if (tmaxb >= tminb && tminb < ray.hit.t && tmaxb >= 0) dist2 = tminb;
		unsigned int lidx = node->left, ridx = node->right;
		if (dist1 > dist2)
		{
			float t = dist1; dist1 = dist2; dist2 = t;
			unsigned int i = lidx; lidx = ridx; ridx = i;
		}
		if (dist1 == 1e30f)
		{
			if (stackPtr == 0) break; else node = stack[--stackPtr];
		}
		else
		{
			node = altNode + lidx;
			if (dist2 != 1e30f) stack[stackPtr++] = altNode + ridx;
		}
	}
	return steps;
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
	const bvhvec3 plane0 = normalize( cross( p0 - O, p0 - p2 ) ); // left plane
	const bvhvec3 plane1 = normalize( cross( p3 - O, p3 - p1 ) ); // right plane
	const bvhvec3 plane2 = normalize( cross( p1 - O, p1 - p0 ) ); // top plane
	const bvhvec3 plane3 = normalize( cross( p2 - O, p2 - p3 ) ); // bottom plane
	const int sign0x = plane0.x < 0 ? 4 : 0, sign0y = plane0.y < 0 ? 5 : 1, sign0z = plane0.z < 0 ? 6 : 2;
	const int sign1x = plane1.x < 0 ? 4 : 0, sign1y = plane1.y < 0 ? 5 : 1, sign1z = plane1.z < 0 ? 6 : 2;
	const int sign2x = plane2.x < 0 ? 4 : 0, sign2y = plane2.y < 0 ? 5 : 1, sign2z = plane2.z < 0 ? 6 : 2;
	const int sign3x = plane3.x < 0 ? 4 : 0, sign3y = plane3.y < 0 ? 5 : 1, sign3z = plane3.z < 0 ? 6 : 2;
	const float d0 = dot( O, plane0 ), d1 = dot( O, plane1 );
	const float d2 = dot( O, plane2 ), d3 = dot( O, plane3 );
	// Traverse the tree with the packet
	int first = 0, last = 255; // first and last active ray in the packet
	const BVHNode* node = &bvhNode[0];
	ALIGNED( 64 ) unsigned int stack[64], stackPtr = 0;
	while (1)
	{
		if (node->isLeaf())
		{
			// handle leaf node
			for (unsigned int j = 0; j < node->triCount; j++)
			{
				const unsigned int idx = triIdx[node->leftFirst + j], vid = idx * 3;
				const bvhvec3 edge1 = verts[vid + 1] - verts[vid], edge2 = verts[vid + 2] - verts[vid];
				const bvhvec3 s = O - bvhvec3( verts[vid] );
				for (int i = first; i <= last; i++)
				{
					Ray& ray = packet[i];
					const bvhvec3 h = cross( ray.D, edge2 );
					const float a = dot( edge1, h );
					if (fabs( a ) < 0.0000001f) continue; // ray parallel to triangle
					const float f = 1 / a, u = f * dot( s, h );
					if (u < 0 || u > 1) continue;
					const bvhvec3 q = cross( s, edge1 );
					const float v = f * dot( ray.D, q );
					if (v < 0 || u + v > 1) continue;
					const float t = f * dot( edge2, q );
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
			const BVHNode* left = bvhNode + node->leftFirst;
			const BVHNode* right = bvhNode + node->leftFirst + 1;
			bool visitLeft = true, visitRight = true;
			int leftFirst = first, leftLast = last, rightFirst = first, rightLast = last;
			float distLeft, distRight;
			{
				// see if we want to intersect the left child
				const bvhvec3 o1( left->aabbMin.x - O.x, left->aabbMin.y - O.y, left->aabbMin.z - O.z );
				const bvhvec3 o2( left->aabbMax.x - O.x, left->aabbMax.y - O.y, left->aabbMax.z - O.z );
				// 1. Early-in test: if first ray hits the node, the packet visits the node
				CALC_TMIN_TMAX_WITH_SLABTEST_ON_RAY( first );
				const bool earlyHit = (tmax >= tmin && tmin < packet[first].hit.t && tmax >= 0);
				distLeft = tmin;
				// 2. Early-out test: if the node aabb is outside the four planes, we skip the node
				if (!earlyHit)
				{
					float* minmax = (float*)left;
					bvhvec3 p0( minmax[sign0x], minmax[sign0y], minmax[sign0z] );
					bvhvec3 p1( minmax[sign1x], minmax[sign1y], minmax[sign1z] );
					bvhvec3 p2( minmax[sign2x], minmax[sign2y], minmax[sign2z] );
					bvhvec3 p3( minmax[sign3x], minmax[sign3y], minmax[sign3z] );
					if (dot( p0, plane0 ) > d0 || dot( p1, plane1 ) > d1 || dot( p2, plane2 ) > d2 || dot( p3, plane3 ) > d3)
						visitLeft = false;
					else
					{
						// 3. Last resort: update first and last, stay in node if first > last
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
				CALC_TMIN_TMAX_WITH_SLABTEST_ON_RAY( first );
				const bool earlyHit = (tmax >= tmin && tmin < packet[first].hit.t && tmax >= 0);
				distRight = tmin;
				// 2. Early-out test: if the node aabb is outside the four planes, we skip the node
				if (!earlyHit)
				{
					float* minmax = (float*)right;
					bvhvec3 p0( minmax[sign0x], minmax[sign0y], minmax[sign0z] );
					bvhvec3 p1( minmax[sign1x], minmax[sign1y], minmax[sign1z] );
					bvhvec3 p2( minmax[sign2x], minmax[sign2y], minmax[sign2z] );
					bvhvec3 p3( minmax[sign3x], minmax[sign3y], minmax[sign3z] );
					if (dot( p0, plane0 ) > d0 || dot( p1, plane1 ) > d1 || dot( p2, plane2 ) > d2 || dot( p3, plane3 ) > d3)
						visitRight = false;
					else
					{
						// 3. Last resort: update first and last, stay in node if first > last
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

// IntersectTri
void BVH::IntersectTri( Ray& ray, const unsigned int idx ) const
{
	// Moeller-Trumbore ray/triangle intersection algorithm
	const unsigned int vertIdx = idx * 3;
	const bvhvec3 edge1 = verts[vertIdx + 1] - verts[vertIdx];
	const bvhvec3 edge2 = verts[vertIdx + 2] - verts[vertIdx];
	const bvhvec3 h = cross( ray.D, edge2 );
	const float a = dot( edge1, h );
	if (fabs( a ) < 0.0000001f) return; // ray parallel to triangle
	const float f = 1 / a;
	const bvhvec3 s = ray.O - bvhvec3( verts[vertIdx] );
	const float u = f * dot( s, h );
	if (u < 0 || u > 1) return;
	const bvhvec3 q = cross( s, edge1 );
	const float v = f * dot( ray.D, q );
	if (v < 0 || u + v > 1) return;
	const float t = f * dot( edge2, q );
	if (t > 0 && t < ray.hit.t)
	{
		// register a hit: ray is shortened to t
		ray.hit.t = t, ray.hit.u = u, ray.hit.v = v, ray.hit.prim = idx;
	}
}

// IntersectAABB
float BVH::IntersectAABB( const Ray& ray, const bvhvec3& aabbMin, const bvhvec3& aabbMax )
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
	if (tmax >= tmin && tmin < ray.hit.t && tmax >= 0) return tmin; else return 1e30f;
}

// ============================================================================
//
//        I M P L E M E N T A T I O N  -  S I M D  C O D E
// 
// ============================================================================

#ifdef BVH_USEAVX

// Ultra-fast single-threaded AVX binned-SAH-builder.
// This code produces BVHs nearly identical to reference, but much faster.
// On a 12th gen laptop i7 CPU, Sponza Crytek (~260k tris) is processed in 51ms.
// The code relies on the availability of AVX instructions. AVX2 is not needed.
#ifdef _MSC_VER
#define LANE(a,b) a.m128_f32[b]
// Not using clang/g++ method under MSCC; compiler may benefit from .m128_i32.
#define ILANE(a,b) a.m128i_i32[b]
#else
#define LANE(a,b) a[b]
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
	ANLR = halfArea( lb ) * (float)lN + halfArea( rb ) * (float)rN; if (ANLR < splitCost) \
	splitCost = ANLR, bestAxis = a, bestPos = pos, bestLBox = lb, bestRBox = rb; }
#if defined(_MSC_VER)
#pragma warning ( push )
#pragma warning( disable:4701 ) // "potentially uninitialized local variable 'bestLBox' used"
#elif defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif
void BVH::BuildAVX( const bvhvec4* vertices, const unsigned int primCount )
{
	int test = BVHBINS;
	if (test != 8) assert( false ); // AVX builders require BVHBINS == 8.
	// aligned data
	ALIGNED( 64 ) __m256 binbox[3 * BVHBINS];				// 768 bytes
	ALIGNED( 64 ) __m256 binboxOrig[3 * BVHBINS];			// 768 bytes
	ALIGNED( 64 ) unsigned int count[3][BVHBINS]{};			// 96 bytes
	ALIGNED( 64 ) __m256 bestLBox, bestRBox;				// 64 bytes
	// some constants
	static const __m128 max4 = _mm_set1_ps( -1e30f ), half4 = _mm_set1_ps( 0.5f );
	static const __m128 two4 = _mm_set1_ps( 2.0f ), min1 = _mm_set1_ps( -1 );
	static const __m256 max8 = _mm256_set1_ps( -1e30f );
	static const __m256 signFlip8 = _mm256_setr_ps( -0.0f, -0.0f, -0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f );
	static const __m128 signFlip4 = _mm_setr_ps( -0.0f, -0.0f, -0.0f, 0.0f );
	static const __m128 mask3 = _mm_cmpeq_ps( _mm_setr_ps( 0, 0, 0, 1 ), _mm_setzero_ps() );
	static const __m128 binmul3 = _mm_set1_ps( BVHBINS * 0.49999f );
	for (unsigned int i = 0; i < 3 * BVHBINS; i++) binboxOrig[i] = max8; // binbox initialization template
	// reset node pool
	newNodePtr = 2;
	if (!bvhNode)
	{
		triCount = primCount;
		verts = (bvhvec4*)vertices;
		triIdx = new unsigned int[triCount];
		bvhNode = (BVHNode*)ALIGNED_MALLOC( triCount * 2 * sizeof( BVHNode ) );
		memset( &bvhNode[1], 0, 32 ); // avoid crash in refit.
		fragment = new OPERATOR_NEW_ALIGN(32) Fragment[triCount];
		idxCount = triCount;
	}
	else assert( triCount == primCount ); // don't change triangle count between builds.
	struct FragSSE { __m128 bmin4, bmax4; };
	FragSSE* frag4 = (FragSSE*)fragment;
	__m256* frag8 = (__m256*)fragment;
	const __m128* tris4 = (__m128*)verts;
	// assign all triangles to the root node
	BVHNode& root = bvhNode[0];
	root.leftFirst = 0, root.triCount = triCount;
	// initialize fragments and update root bounds
	__m128 rootMin = max4, rootMax = max4;
	for (unsigned int i = 0; i < triCount; i++)
	{
		const __m128 v1 = _mm_xor_ps( signFlip4, _mm_min_ps( _mm_min_ps( tris4[i * 3], tris4[i * 3 + 1] ), tris4[i * 3 + 2] ) );
		const __m128 v2 = _mm_max_ps( _mm_max_ps( tris4[i * 3], tris4[i * 3 + 1] ), tris4[i * 3 + 2] );
		frag4[i].bmin4 = v1, frag4[i].bmax4 = v2, rootMin = _mm_max_ps( rootMin, v1 ), rootMax = _mm_max_ps( rootMax, v2 ), triIdx[i] = i;
	}
	rootMin = _mm_xor_ps( rootMin, signFlip4 );
	root.aabbMin = *(bvhvec3*)&rootMin, root.aabbMax = *(bvhvec3*)&rootMax;
	// subdivide recursively
	ALIGNED( 64 ) unsigned int task[128], taskCount = 0, nodeIdx = 0;
	const bvhvec3 minDim = (root.aabbMax - root.aabbMin) * 1e-10f;
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
			unsigned int fi = triIdx[node.leftFirst];
			memset( count, 0, sizeof( count ) );
			__m256 r0, r1, r2, f = frag8[fi];
			__m128i bi4 = _mm_cvtps_epi32( _mm_sub_ps( _mm_mul_ps( _mm_sub_ps( _mm_sub_ps( frag4[fi].bmax4, frag4[fi].bmin4 ), nmin4 ), rpd4 ), half4 ) );
			memcpy( binbox, binboxOrig, sizeof( binbox ) );
			unsigned int i0 = ILANE( bi4, 0 ), i1 = ILANE( bi4, 1 ), i2 = ILANE( bi4, 2 ), * ti = triIdx + node.leftFirst + 1;
			for (unsigned int i = 0; i < node.triCount - 1; i++)
			{
				unsigned int fid = *ti++;
				const __m256 b0 = binbox[i0], b1 = binbox[BVHBINS + i1], b2 = binbox[2 * BVHBINS + i2];
				const __m128 fmin = frag4[fid].bmin4, fmax = frag4[fid].bmax4;
				r0 = _mm256_max_ps( b0, f ), r1 = _mm256_max_ps( b1, f ), r2 = _mm256_max_ps( b2, f );
				const __m128i b4 = _mm_cvtps_epi32( _mm_sub_ps( _mm_mul_ps( _mm_sub_ps( _mm_sub_ps( fmax, fmin ), nmin4 ), rpd4 ), half4 ) );
			#if _MSC_VER < 1920 
				// VS2017 needs a random check on fid to produce correct code... The condition is never true. 
				if (fid > triCount) fid = triCount - 1;
			#endif
				f = frag8[fid], count[0][i0]++, count[1][i1]++, count[2][i2]++;
				binbox[i0] = r0, i0 = ILANE( b4, 0 );
				binbox[BVHBINS + i1] = r1, i1 = ILANE( b4, 1 );
				binbox[2 * BVHBINS + i2] = r2, i2 = ILANE( b4, 2 );
			}
			// final business for final fragment
			const __m256 b0 = binbox[i0], b1 = binbox[BVHBINS + i1], b2 = binbox[2 * BVHBINS + i2];
			count[0][i0]++, count[1][i1]++, count[2][i2]++;
			r0 = _mm256_max_ps( b0, f ), r1 = _mm256_max_ps( b1, f ), r2 = _mm256_max_ps( b2, f );
			binbox[i0] = r0, binbox[BVHBINS + i1] = r1, binbox[2 * BVHBINS + i2] = r2;
			// calculate per-split totals
			float splitCost = 1e30f;
			unsigned int bestAxis = 0, bestPos = 0, n = newNodePtr, j = node.leftFirst + node.triCount, src = node.leftFirst;
			const __m256* bb = binbox;
			for (int a = 0; a < 3; a++, bb += BVHBINS) if ((node.aabbMax[a] - node.aabbMin[a]) > minDim.cell[a])
			{
				// hardcoded bin processing for BVHBINS == 8
				assert( BVHBINS == 8 );
				const unsigned int lN0 = count[a][0], rN0 = count[a][7];
				const __m256 lb0 = bb[0], rb0 = bb[7];
				const unsigned int lN1 = lN0 + count[a][1], rN1 = rN0 + count[a][6], lN2 = lN1 + count[a][2];
				const unsigned int rN2 = rN1 + count[a][5], lN3 = lN2 + count[a][3], rN3 = rN2 + count[a][4];
				const __m256 lb1 = _mm256_max_ps( lb0, bb[1] ), rb1 = _mm256_max_ps( rb0, bb[6] );
				const __m256 lb2 = _mm256_max_ps( lb1, bb[2] ), rb2 = _mm256_max_ps( rb1, bb[5] );
				const __m256 lb3 = _mm256_max_ps( lb2, bb[3] ), rb3 = _mm256_max_ps( rb2, bb[4] );
				const unsigned int lN4 = lN3 + count[a][4], rN4 = rN3 + count[a][3], lN5 = lN4 + count[a][5];
				const unsigned int rN5 = rN4 + count[a][2], lN6 = lN5 + count[a][6], rN6 = rN5 + count[a][1];
				const __m256 lb4 = _mm256_max_ps( lb3, bb[4] ), rb4 = _mm256_max_ps( rb3, bb[3] );
				const __m256 lb5 = _mm256_max_ps( lb4, bb[5] ), rb5 = _mm256_max_ps( rb4, bb[2] );
				const __m256 lb6 = _mm256_max_ps( lb5, bb[6] ), rb6 = _mm256_max_ps( rb5, bb[1] );
				float ANLR3 = 1e30f; PROCESS_PLANE( a, 3, ANLR3, lN3, rN3, lb3, rb3 ); // most likely split
				float ANLR2 = 1e30f; PROCESS_PLANE( a, 2, ANLR2, lN2, rN4, lb2, rb4 );
				float ANLR4 = 1e30f; PROCESS_PLANE( a, 4, ANLR4, lN4, rN2, lb4, rb2 );
				float ANLR5 = 1e30f; PROCESS_PLANE( a, 5, ANLR5, lN5, rN1, lb5, rb1 );
				float ANLR1 = 1e30f; PROCESS_PLANE( a, 1, ANLR1, lN1, rN5, lb1, rb5 );
				float ANLR0 = 1e30f; PROCESS_PLANE( a, 0, ANLR0, lN0, rN6, lb0, rb6 );
				float ANLR6 = 1e30f; PROCESS_PLANE( a, 6, ANLR6, lN6, rN0, lb6, rb0 ); // least likely split
			}
			if (splitCost >= node.CalculateNodeCost()) break; // not splitting is better.
			// in-place partition
			const float rpd = (*(bvhvec3*)&rpd4)[bestAxis], nmin = (*(bvhvec3*)&nmin4)[bestAxis];
			unsigned int t, fr = triIdx[src];
			for (unsigned int i = 0; i < node.triCount; i++)
			{
				const unsigned int bi = (unsigned int)((fragment[fr].bmax[bestAxis] - fragment[fr].bmin[bestAxis] - nmin) * rpd);
				if (bi <= bestPos) fr = triIdx[++src]; else t = fr, fr = triIdx[src] = triIdx[--j], triIdx[j] = t;
			}
			// create child nodes and recurse
			const unsigned int leftCount = src - node.leftFirst, rightCount = node.triCount - leftCount;
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
	bvhvec3 plane0 = normalize( cross( p0 - O, p0 - p2 ) ); // left plane
	bvhvec3 plane1 = normalize( cross( p3 - O, p3 - p1 ) ); // right plane
	bvhvec3 plane2 = normalize( cross( p1 - O, p1 - p0 ) ); // top plane
	bvhvec3 plane3 = normalize( cross( p2 - O, p2 - p3 ) ); // bottom plane
	int sign0x = plane0.x < 0 ? 4 : 0, sign0y = plane0.y < 0 ? 5 : 1, sign0z = plane0.z < 0 ? 6 : 2;
	int sign1x = plane1.x < 0 ? 4 : 0, sign1y = plane1.y < 0 ? 5 : 1, sign1z = plane1.z < 0 ? 6 : 2;
	int sign2x = plane2.x < 0 ? 4 : 0, sign2y = plane2.y < 0 ? 5 : 1, sign2z = plane2.z < 0 ? 6 : 2;
	int sign3x = plane3.x < 0 ? 4 : 0, sign3y = plane3.y < 0 ? 5 : 1, sign3z = plane3.z < 0 ? 6 : 2;
	float t0 = dot( O, plane0 ), t1 = dot( O, plane1 );
	float t2 = dot( O, plane2 ), t3 = dot( O, plane3 );
	// Traverse the tree with the packet
	int first = 0, last = 255; // first and last active ray in the packet
	BVHNode* node = &bvhNode[0];
	ALIGNED( 64 ) unsigned int stack[64], stackPtr = 0;
	while (1)
	{
		if (node->isLeaf())
		{
			// handle leaf node
			for (unsigned int j = 0; j < node->triCount; j++)
			{
				const unsigned int idx = triIdx[node->leftFirst + j], vid = idx * 3;
				const bvhvec3 edge1 = verts[vid + 1] - verts[vid], edge2 = verts[vid + 2] - verts[vid];
				const bvhvec3 s = O - bvhvec3( verts[vid] );
				for (int i = first; i <= last; i++)
				{
					Ray& ray = packet[i];
					const bvhvec3 h = cross( ray.D, edge2 );
					const float a = dot( edge1, h );
					if (fabs( a ) < 0.0000001f) continue; // ray parallel to triangle
					const float f = 1 / a, u = f * dot( s, h );
					if (u < 0 || u > 1) continue;
					const bvhvec3 q = cross( s, edge1 );
					const float v = f * dot( ray.D, q );
					if (v < 0 || u + v > 1) continue;
					const float t = f * dot( edge2, q );
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
			int leftFirst = first, leftLast = last, rightFirst = first, rightLast = last;
			float distLeft, distRight;
			{
				// see if we want to intersect the left child
				const __m128 minO4 = _mm_sub_ps( *(__m128*) & left->aabbMin, O4 );
				const __m128 maxO4 = _mm_sub_ps( *(__m128*) & left->aabbMax, O4 );
				// 1. Early-in test: if first ray hits the node, the packet visits the node
				const __m128 rD4 = *(__m128*) & packet[first].rD;
				const __m128 st1 = _mm_mul_ps( _mm_and_ps( minO4, mask4 ), rD4 );
				const __m128 st2 = _mm_mul_ps( _mm_and_ps( maxO4, mask4 ), rD4 );
				const __m128 vmax4 = _mm_max_ps( st1, st2 ), vmin4 = _mm_min_ps( st1, st2 );
				const float tmax = tinybvh_min( LANE( vmax4, 0 ), tinybvh_min( LANE( vmax4, 1 ), LANE( vmax4, 2 ) ) );
				const float tmin = tinybvh_max( LANE( vmin4, 0 ), tinybvh_max( LANE( vmin4, 1 ), LANE( vmin4, 2 ) ) );
				const bool earlyHit = (tmax >= tmin && tmin < packet[first].hit.t && tmax >= 0);
				distLeft = tmin;
				// 2. Early-out test: if the node aabb is outside the four planes, we skip the node
				if (!earlyHit)
				{
					float* minmax = (float*)left;
					bvhvec3 p0( minmax[sign0x], minmax[sign0y], minmax[sign0z] );
					bvhvec3 p1( minmax[sign1x], minmax[sign1y], minmax[sign1z] );
					bvhvec3 p2( minmax[sign2x], minmax[sign2y], minmax[sign2z] );
					bvhvec3 p3( minmax[sign3x], minmax[sign3y], minmax[sign3z] );
					if (dot( p0, plane0 ) > t0 || dot( p1, plane1 ) > t1 || dot( p2, plane2 ) > t2 || dot( p3, plane3 ) > t3)
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
				const __m128 rD4 = *(__m128*) & packet[first].rD;
				const __m128 st1 = _mm_mul_ps( minO4, rD4 ), st2 = _mm_mul_ps( maxO4, rD4 );
				const __m128 vmax4 = _mm_max_ps( st1, st2 ), vmin4 = _mm_min_ps( st1, st2 );
				const float tmax = tinybvh_min( LANE( vmax4, 0 ), tinybvh_min( LANE( vmax4, 1 ), LANE( vmax4, 2 ) ) );
				const float tmin = tinybvh_max( LANE( vmin4, 0 ), tinybvh_max( LANE( vmin4, 1 ), LANE( vmin4, 2 ) ) );
				const bool earlyHit = (tmax >= tmin && tmin < packet[first].hit.t && tmax >= 0);
				distRight = tmin;
				// 2. Early-out test: if the node aabb is outside the four planes, we skip the node
				if (!earlyHit)
				{
					float* minmax = (float*)right;
					bvhvec3 p0( minmax[sign0x], minmax[sign0y], minmax[sign0z] );
					bvhvec3 p1( minmax[sign1x], minmax[sign1y], minmax[sign1z] );
					bvhvec3 p2( minmax[sign2x], minmax[sign2y], minmax[sign2z] );
					bvhvec3 p3( minmax[sign3x], minmax[sign3y], minmax[sign3z] );
					if (dot( p0, plane0 ) > t0 || dot( p1, plane1 ) > t1 || dot( p2, plane2 ) > t2 || dot( p3, plane3 ) > t3)
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
							const float tmax = tinybvh_min( LANE( vmax4, 0 ), tinybvh_min( LANE( vmax4, 1 ), LANE( vmax4, 2 ) ) );
							const float tmin = tinybvh_max( LANE( vmin4, 0 ), tinybvh_max( LANE( vmin4, 1 ), LANE( vmin4, 2 ) ) );
							if (tmax >= tmin && tmin < packet[rightFirst].hit.t && tmax >= 0) { distRight = tmin; break; }
						}
						for (; rightLast >= first; rightLast--)
						{
							const __m128 rD4 = *(__m128*) & packet[rightLast].rD;
							const __m128 st1 = _mm_mul_ps( _mm_and_ps( minO4, mask4 ), rD4 );
							const __m128 st2 = _mm_mul_ps( _mm_and_ps( maxO4, mask4 ), rD4 );
							const __m128 vmax4 = _mm_max_ps( st1, st2 ), vmin4 = _mm_min_ps( st1, st2 );
							const float tmax = tinybvh_min( LANE( vmax4, 0 ), tinybvh_min( LANE( vmax4, 1 ), LANE( vmax4, 2 ) ) );
							const float tmin = tinybvh_max( LANE( vmin4, 0 ), tinybvh_max( LANE( vmin4, 1 ), LANE( vmin4, 2 ) ) );
							if (tmax >= tmin && tmin < packet[rightLast].hit.t && tmax >= 0) break;
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
int BVH::Intersect_AltSoA( Ray& ray ) const
{
	assert( alt2Node != 0 );
	BVHNodeAlt2* node = &alt2Node[0], * stack[64];
	unsigned int stackPtr = 0, steps = 0;
	const __m128 Ox4 = _mm_set1_ps( ray.O.x ), rDx4 = _mm_set1_ps( ray.rD.x );
	const __m128 Oy4 = _mm_set1_ps( ray.O.y ), rDy4 = _mm_set1_ps( ray.rD.y );
	const __m128 Oz4 = _mm_set1_ps( ray.O.z ), rDz4 = _mm_set1_ps( ray.rD.z );
	const __m128 inf4 = _mm_set1_ps( 1e30f );
	while (1)
	{
		steps++;
		if (node->isLeaf())
		{
			for (unsigned int i = 0; i < node->triCount; i++)
			{
				const unsigned int tidx = triIdx[node->firstTri + i], vertIdx = tidx * 3;
				const bvhvec3 edge1 = verts[vertIdx + 1] - verts[vertIdx];
				const bvhvec3 edge2 = verts[vertIdx + 2] - verts[vertIdx];
				const bvhvec3 h = cross( ray.D, edge2 );
				const float a = dot( edge1, h );
				if (fabs( a ) < 0.0000001f) continue; // ray parallel to triangle
				const float f = 1 / a;
				const bvhvec3 s = ray.O - bvhvec3( verts[vertIdx] );
				const float u = f * dot( s, h );
				if (u < 0 || u > 1) continue;
				const bvhvec3 q = cross( s, edge1 );
				const float v = f * dot( ray.D, q );
				if (v < 0 || u + v > 1) continue;
				const float t = f * dot( edge2, q );
				if (t < 0 || t > ray.hit.t) continue;
				ray.hit.t = t, ray.hit.u = u, ray.hit.v = v, ray.hit.prim = tidx;
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
		unsigned int lidx = node->left, ridx = node->right;
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
		float dist1 = tmaxa_1 >= tmina_0 ? tmina_0 : 1e30f;
		float dist2 = tmaxb_3 >= tminb_2 ? tminb_2 : 1e30f;
	#endif
		if (dist1 > dist2)
		{
			float t = dist1; dist1 = dist2; dist2 = t;
			unsigned int i = lidx; lidx = ridx; ridx = i;
		}
		if (dist1 == 1e30f)
		{
			if (stackPtr == 0) break; else node = stack[--stackPtr];
		}
		else
		{
			node = alt2Node + lidx;
			if (dist2 != 1e30f) stack[stackPtr++] = alt2Node + ridx;
		}
	}
	return steps;
}

#else

int BVH::Intersect_AltSoA( Ray& ray ) const
{
	// not implemented for platforms that do not support SSE/AVX.
	assert( false );
	return 0;
}

#endif // BVH_USEAVX

} // namespace tinybvh

#endif // TINYBVH_IMPLEMENTATION

#endif // TINY_BVH_H_
