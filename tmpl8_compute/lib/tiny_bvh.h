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

// version 0.0.1 : Establishing interface.
//

//
// Use this in *one* .c or .cpp
//   #define TINYBVH_IMPLEMENTATION
//   #include "tiny_bvh.h"
//

// References:
// - How to build a BVH series:
//   https://jacco.ompf2.com/2022/04/13/how-to-build-a-bvh-part-1-basics/
// - Parallel Spatial Splits in Bounding Volume Hierarchies:
//   https://diglib.eg.org/items/f55715b1-9e56-4b40-af73-59d3dfba9fe7
// - Heuristics for ray tracing using space subdivision:
//   https://graphicsinterface.org/wp-content/uploads/gi1989-22.pdf
// - Heuristic Ray Shooting Algorithms:
//   https://dcgi.fel.cvut.cz/home/havran/DISSVH/phdthesis.html

#ifndef TINY_BVH_H_
#define TINY_BVH_H_

// binned BVH building: bin count
#define BVHBINS 8

// include fast AVX BVH builder
#define BVH_USEAVX

// ============================================================================
//
//        P R E L I M I N A R I E S
// 
// ============================================================================

// aligned memory allocation
#ifdef _MSC_VER
// Visual Studio / C11
#define ALIGNED( x ) __declspec( align( x ) )
#define ALIGNED_MALLOC( x ) ( ( x ) == 0 ? 0 : _aligned_malloc( ( x ), 64 ) )
#define ALIGNED_FREE( x ) _aligned_free( x )
#else
// gcc
#define ALIGNED( x ) __attribute__( ( aligned( x ) ) )
#define ALIGNED_MALLOC( x ) ( ( x ) == 0 ? 0 : aligned_alloc( 64, ( x ) ) )
#define ALIGNED_FREE( x ) free( x )
// TODO: Intel, Posix; see: 
// https://stackoverflow.com/questions/32612881/why-use-mm-malloc-as-opposed-to-aligned-malloc-alligned-alloc-or-posix-mem
#endif

namespace tinybvh {

#ifdef _MSC_VER
// Suppress a warning caused by the union of x,y,.. and cell[..] in vectors.
// We need this union to address vector components either by name or by index.
// The warning is re-enabled right after the definition of the data types.
#pragma warning ( push )
#pragma warning ( disable: 4201 /* nameless struct / union */ )
#endif

struct ALIGNED( 16 ) bvhvec4
{
	// vector naming is designed to not cause any name clashes.
	bvhvec4() = default;
	bvhvec4( const float a, const float b, const float c, const float d ) : x( a ), y( b ), z( c ), w( d ) {}
	bvhvec4( const float a ) : x( a ), y( a ), z( a ), w( a) {}
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
static inline bvhvec2 tinybvh_min( const bvhvec2& a, const bvhvec2& b ) { return bvhvec2( tinybvh_min( a.x, b.x ), tinybvh_min( a.y, b.y ) ); }
static inline bvhvec3 tinybvh_min( const bvhvec3& a, const bvhvec3& b ) { return bvhvec3( tinybvh_min( a.x, b.x ), tinybvh_min( a.y, b.y ), tinybvh_min( a.z, b.z ) ); }
static inline bvhvec4 tinybvh_min( const bvhvec4& a, const bvhvec4& b ) { return bvhvec4( tinybvh_min( a.x, b.x ), tinybvh_min( a.y, b.y ), tinybvh_min( a.z, b.z ), tinybvh_min( a.w, b.w ) ); }
static inline bvhvec2 tinybvh_max( const bvhvec2& a, const bvhvec2& b ) { return bvhvec2( tinybvh_max( a.x, b.x ), tinybvh_max( a.y, b.y ) ); }
static inline bvhvec3 tinybvh_max( const bvhvec3& a, const bvhvec3& b ) { return bvhvec3( tinybvh_max( a.x, b.x ), tinybvh_max( a.y, b.y ), tinybvh_max( a.z, b.z ) ); }
static inline bvhvec4 tinybvh_max( const bvhvec4& a, const bvhvec4& b ) { return bvhvec4( tinybvh_max( a.x, b.x ), tinybvh_max( a.y, b.y ), tinybvh_max( a.z, b.z ), tinybvh_max( a.w, b.w ) ); }

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
	float t, u, v;	// distance along ray & barycentric coordinates of the intersection
	uint prim;		// primitive index
};

struct Ray
{
	// Basic ray class. Note: For single blas traversal it is expected
	// that Ray::rD is properly initialized. For tlas/blas traversal this
	// field is typically updated for each blas.
	Ray() = default;
	Ray( bvhvec3 origin, bvhvec3 direction, float t = 1e30f )
	{
		O = origin, D = direction, rD = tinybvh_safercp( D );
		hit.t = t;
	}
	bvhvec3 O, D, rD;
	Intersection hit; // total ray size: 64 bytes
};

class BVH
{
public:
	struct BVHNode
	{
		bvhvec3 aabbMin; uint leftFirst; // 16 bytes
		bvhvec3 aabbMax; uint triCount;	// 16 bytes, total: 32 bytes
		bool isLeaf() const { return triCount > 0; /* empty BVH leaves do not exist */ }
		float Intersect( const Ray& ray ) const { return BVH::IntersectAABB( ray, aabbMin, aabbMax ); }
		float SurfaceArea() const { return BVH::SA( aabbMin, aabbMax ); }
		float CalculateNodeCost() const { return SurfaceArea() * triCount; }
	};
	struct Fragment
	{
		// A fragment stores the bounds of an input primitive. The name 'Fragment' is from
		// "Parallel Spatial Splits in Bounding Volume Hierarchies", 2016, Fuetterling et al.,
		// and refers to the potential splitting of these boxes for SBVH construction.
		bvhvec3 bmin;			// AABB min x, y and z
		uint primIdx;			// index of the original primitive
		bvhvec3 bmax;			// AABB max x, y and z
		uint clipped = 0;		// Fragment is the result of clipping if > 0.
		bool validBox() { return bmin.x < 1e30f; }
	};
	BVH() = default;
	~BVH()
	{
		ALIGNED_FREE( bvhNode );
		delete triIdx;
		delete fragment;
		bvhNode = 0, triIdx = 0, fragment = 0;
	}
	float SAHCost( const uint nodeIdx = 0 ) const
	{
		// Determine the SAH cost of the tree. This provides an indication
		// of the quality of the BVH: Lower is better.
		const BVHNode& n = bvhNode[nodeIdx];
		if (n.isLeaf()) return 2.0f * n.SurfaceArea() * n.triCount;
		float cost = 3.0f * n.SurfaceArea() + SAHCost( n.leftFirst ) + SAHCost( n.leftFirst + 1 );
		return nodeIdx == 0 ? (cost / n.SurfaceArea()) : cost;
	}
	int NodeCount( const uint nodeIdx = 0 ) const
	{
		// Determine the number of nodes in the tree. Typically the result should
		// be newNodePtr - 1 (second node is always unused), but some builders may 
		// have unused nodes besides node 1.
		const BVHNode& n = bvhNode[nodeIdx];
		uint retVal = 1;
		if (!n.isLeaf()) retVal += NodeCount( n.leftFirst ) + NodeCount( n.leftFirst + 1 );
		return retVal;
	}
	void Build( const bvhvec4* vertices, const uint primCount );
	void BuildAVX( const bvhvec4* vertices, const uint primCount );
	void Refit();
	int Intersect( Ray& ray ) const;
private:
	void IntersectTri( Ray& ray, const uint triIdx ) const;
	static float IntersectAABB( const Ray& ray, const bvhvec3& aabbMin, const bvhvec3& aabbMax );
	static float SA( const bvhvec3& aabbMin, const bvhvec3& aabbMax )
	{
		bvhvec3 e = aabbMax - aabbMin; // extent of the node
		return e.x * e.y + e.y * e.z + e.z * e.x;
	}
public:
	bvhvec4* tris = 0;			// pointer to input primitive array: 3x16 bytes per tri
	uint triCount = 0;			// number of primitives in tris
	Fragment* fragment = 0;		// input primitive bounding boxes
	uint* triIdx = 0;			// primitive index array
	uint idxCount = 0;			// number of indices in triIdx. May exceed triCount * 3 for SBVH.
	uint newNodePtr = 0;		// number of reserved nodes
	BVHNode* bvhNode = 0;		// BVH node pool. Root is always in node 0.
};

// ============================================================================
//
//        I M P L E M E N T A T I O N
// 
// ============================================================================

#ifdef TINYBVH_IMPLEMENTATION

#include <assert.h>

// Basic binned-SAH-builder. This is the reference builder; it yields a decent
// tree suitable for ray tracing on the CPU. The code is platform-independent.
// Faster code, using SSE/AVX, is available for x64 CPUs.
// For GPU rendering the resulting BVH should be converted to a more optimal
// format after construction.
void BVH::Build( const bvhvec4* vertices, const uint primCount )
{
	// reset node pool
	newNodePtr = 2, triCount = primCount;
	if (!bvhNode)
	{
		bvhNode = (BVHNode*)ALIGNED_MALLOC( triCount * 2 * sizeof( BVHNode ) );
		memset( &bvhNode[1], 0, 32 );	// node 1 remains unused, for cache line alignment.
		triIdx = new uint[triCount];
		tris = (bvhvec4*)vertices;		// note: we're not copying this data; don't delete.
		fragment = new Fragment[triCount];
		idxCount = primCount;
		triCount = primCount;
	}
	else assert( triCount == primCount ); // don't change triangle count between builds.
	// assign all triangles to the root node
	BVHNode& root = bvhNode[0];
	root.leftFirst = 0, root.triCount = triCount, root.aabbMin = bvhvec3( 1e30f ), root.aabbMax = bvhvec3( -1e30f );
	// initialize fragments and initialize root node bounds
	for (uint i = 0; i < triCount; i++)
	{
		fragment[i].bmin = tinybvh_min( tinybvh_min( tris[i * 3], tris[i * 3 + 1] ), tris[i * 3 + 2] );
		fragment[i].bmax = tinybvh_max( tinybvh_max( tris[i * 3], tris[i * 3 + 1] ), tris[i * 3 + 2] );
		root.aabbMin = tinybvh_min( root.aabbMin, fragment[i].bmin );
		root.aabbMax = tinybvh_max( root.aabbMax, fragment[i].bmax ), triIdx[i] = i;
	}
	// subdivide recursively
	uint task[256], taskCount = 0, nodeIdx = 0;
	bvhvec3 minDim = (root.aabbMax - root.aabbMin) * 1e-20f, bestLMin = 0, bestLMax = 0, bestRMin = 0, bestRMax = 0;
	while (1)
	{
		while (1)
		{
			BVHNode& node = bvhNode[nodeIdx];
			// find optimal object split
			bvhvec3 binMin[3][BVHBINS], binMax[3][BVHBINS];
			for (uint a = 0; a < 3; a++) for (uint i = 0; i < BVHBINS; i++) binMin[a][i] = 1e30f, binMax[a][i] = -1e30f;
			uint count[3][BVHBINS] = { 0 };
			const bvhvec3 rpd3 = bvhvec3( BVHBINS / (node.aabbMax - node.aabbMin) ), nmin3 = node.aabbMin;
			for (uint i = 0; i < node.triCount; i++) // process all tris for x,y and z at once
			{
				const uint fi = triIdx[node.leftFirst + i];
				bvhint3 bi = bvhint3( ((fragment[fi].bmin + fragment[fi].bmax) * 0.5f - nmin3) * rpd3 );
				bi.x = clamp( bi.x, 0, BVHBINS - 1 ), bi.y = clamp( bi.y, 0, BVHBINS - 1 ), bi.z = clamp( bi.z, 0, BVHBINS - 1 );
				binMin[0][bi.x] = tinybvh_min( binMin[0][bi.x], fragment[fi].bmin );
				binMax[0][bi.x] = tinybvh_max( binMax[0][bi.x], fragment[fi].bmax ), count[0][bi.x]++;
				binMin[1][bi.y] = tinybvh_min( binMin[1][bi.y], fragment[fi].bmin );
				binMax[1][bi.y] = tinybvh_max( binMax[1][bi.y], fragment[fi].bmax ), count[1][bi.y]++;
				binMin[2][bi.z] = tinybvh_min( binMin[2][bi.z], fragment[fi].bmin );
				binMax[2][bi.z] = tinybvh_max( binMax[2][bi.z], fragment[fi].bmax ), count[2][bi.z]++;
			}
			// calculate per-split totals
			float splitCost = 1e30f;
			uint bestAxis = 0, bestPos = 0;
			for (int a = 0; a < 3; a++) if ((node.aabbMax[a] - node.aabbMin[a]) > minDim[a])
			{
				bvhvec3 lBMin[BVHBINS - 1], rBMin[BVHBINS - 1], l1 = 1e30f, l2 = -1e30f;
				bvhvec3 lBMax[BVHBINS - 1], rBMax[BVHBINS - 1], r1 = 1e30f, r2 = -1e30f;
				float ANL[BVHBINS - 1], ANR[BVHBINS - 1];
				for (uint lN = 0, rN = 0, i = 0; i < BVHBINS - 1; i++)
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
				for (uint i = 0; i < BVHBINS - 1; i++)
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
			uint j = node.leftFirst + node.triCount, src = node.leftFirst;
			const float rpd = rpd3.cell[bestAxis], nmin = nmin3.cell[bestAxis];
			for (uint i = 0; i < node.triCount; i++)
			{
				const uint fi = triIdx[src];
				int bi = (uint)(((fragment[fi].bmin[bestAxis] + fragment[fi].bmax[bestAxis]) * 0.5f - nmin) * rpd);
				bi = clamp( bi, 0, BVHBINS - 1 );
				if ((uint)bi <= bestPos) src++; else swap( triIdx[src], triIdx[--j] );
			}
			// create child nodes
			uint leftCount = src - node.leftFirst, rightCount = node.triCount - leftCount;
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

#ifdef BVH_USEAVX

// Ultra-fast single-threaded AVX binned-SAH-builder.
// This code produces BVHs nearly identical to reference, but much faster.
// On a 12th gen laptop i7 CPU, Sponza Crytek (~260k tris) is processed in 51ms.
// The code relies on the availability of AVX instructions. AVX2 is not needed.
__forceinline float halfArea( const __m128 a /* a contains extent of aabb */ )
{
	return a.m128_f32[0] * a.m128_f32[1] + a.m128_f32[1] * a.m128_f32[2] + a.m128_f32[2] * a.m128_f32[3];
}
__forceinline float halfArea( const __m256 a /* a contains aabb itself, with min.xyz negated */ )
{
	const __m128 q = _mm256_castps256_ps128( _mm256_add_ps( _mm256_permute2f128_ps( a, a, 5 ), a ) );
	const __m128 v = _mm_mul_ps( q, _mm_shuffle_ps( q, q, 9 ) );
	return v.m128_f32[0] + v.m128_f32[1] + v.m128_f32[2];
}
#define PROCESS_PLANE( a, pos, ANLR, lN, rN, lb, rb ) if (lN * rN != 0) { \
	ANLR = halfArea( lb ) * (float)lN + halfArea( rb ) * (float)rN; if (ANLR < splitCost) \
	splitCost = ANLR, bestAxis = a, bestPos = pos, bestLBox = lb, bestRBox = rb; }
#ifdef _MSC_VER
#pragma warning ( push )
#pragma warning( disable:4701 ) // "potentially uninitialized local variable 'bestLBox' used"
#endif
void BVH::BuildAVX( const bvhvec4* vertices, const uint primCount )
{
	if constexpr (BVHBINS != 8) FatalError( "AVX builders require BVHBINS == 8." );
	// aligned data
	__declspec(align(64)) __m256 binbox[3 * BVHBINS];				// 768 bytes
	__declspec(align(64)) __m256 binboxOrig[3 * BVHBINS];			// 768 bytes
	__declspec(align(64)) uint count[3][BVHBINS] = { 0 };			// 96 bytes
	__declspec(align(64)) __m256 bestLBox, bestRBox;				// 64 bytes
	// some constants
	static const __m128 min4 = _mm_set1_ps( 1e30f ), max4 = _mm_set1_ps( -1e30f ), half4 = _mm_set1_ps( 0.5f );
	static const __m128 two4 = _mm_set1_ps( 2.0f ), min1 = _mm_set1_ps( -1 );
	static const __m256 min8 = _mm256_set1_ps( 1e30f ), max8 = _mm256_set1_ps( -1e30f );
	static const __m256 signFlip8 = _mm256_setr_ps( -0.0f, -0.0f, -0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f );
	static const __m128 signFlip4 = _mm_setr_ps( -0.0f, -0.0f, -0.0f, 0.0f );
	static const __m128 mask3 = _mm_cmpeq_ps( _mm_setr_ps( 0, 0, 0, 1 ), _mm_setzero_ps() );
	static const __m128 binmul3 = _mm_set1_ps( BVHBINS * 0.49999f );
	for (uint i = 0; i < 3 * BVHBINS; i++) binboxOrig[i] = max8; // binbox initialization template
	// reset node pool
	newNodePtr = 2;
	if (!bvhNode)
	{
		triCount = primCount;
		tris = (bvhvec4*)vertices;
		triIdx = new uint[triCount];
		bvhNode = (BVHNode*)MALLOC64( triCount * 2 * sizeof( BVHNode ) );
		memset( &bvhNode[1], 0, 32 ); // avoid crash in refit.
		fragment = new Fragment[triCount];
		idxCount = triCount;
	}
	else assert( triCount == primCount ); // don't change triangle count between builds.
	struct FragSSE { __m128 bmin4, bmax4; };
	FragSSE* frag4 = (FragSSE*)fragment;
	__m256* frag8 = (__m256*)fragment;
	const __m128* tris4 = (__m128*)tris;
	// assign all triangles to the root node
	BVHNode& root = bvhNode[0];
	root.leftFirst = 0, root.triCount = triCount;
	// initialize fragments and update root bounds
	__m128 rootMin = max4, rootMax = max4;
	for (uint i = 0; i < triCount; i++)
	{
		const __m128 v1 = _mm_xor_ps( signFlip4, _mm_min_ps( _mm_min_ps( tris4[i * 3], tris4[i * 3 + 1] ), tris4[i * 3 + 2] ) );
		const __m128 v2 = _mm_max_ps( _mm_max_ps( tris4[i * 3], tris4[i * 3 + 1] ), tris4[i * 3 + 2] );
		frag4[i].bmin4 = v1, frag4[i].bmax4 = v2, rootMin = _mm_max_ps( rootMin, v1 ), rootMax = _mm_max_ps( rootMax, v2 ), triIdx[i] = i;
	}
	rootMin = _mm_xor_ps( rootMin, signFlip4 );
	root.aabbMin = *(bvhvec3*)&rootMin, root.aabbMax = *(bvhvec3*)&rootMax;
	// subdivide recursively
	__declspec(align(64)) uint task[128], taskCount = 0, nodeIdx = 0;
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
			uint fi = triIdx[node.leftFirst];
			memset( count, 0, sizeof( count ) );
			__m256 r0, r1, r2, f = frag8[fi];
			__m128i bi4 = _mm_cvtps_epi32( _mm_sub_ps( _mm_mul_ps( _mm_sub_ps( _mm_sub_ps( frag4[fi].bmax4, frag4[fi].bmin4 ), nmin4 ), rpd4 ), half4 ) );
			memcpy( binbox, binboxOrig, sizeof( binbox ) );
			uint i0 = bi4.m128i_i32[0], i1 = bi4.m128i_i32[1], i2 = bi4.m128i_i32[2], * ti = triIdx + node.leftFirst + 1;
			for (uint i = 0; i < node.triCount - 1; i++)
			{
				const uint fid = *ti++;
				const __m256 b0 = binbox[i0], b1 = binbox[BVHBINS + i1], b2 = binbox[2 * BVHBINS + i2];
				const __m128 fmin = frag4[fid].bmin4, fmax = frag4[fid].bmax4;
				r0 = _mm256_max_ps( b0, f ), r1 = _mm256_max_ps( b1, f ), r2 = _mm256_max_ps( b2, f );
				const __m128i b4 = _mm_cvtps_epi32( _mm_sub_ps( _mm_mul_ps( _mm_sub_ps( _mm_sub_ps( fmax, fmin ), nmin4 ), rpd4 ), half4 ) );
				f = frag8[fid], count[0][i0]++, count[1][i1]++, count[2][i2]++;
				binbox[i0] = r0, i0 = b4.m128i_i32[0];
				binbox[BVHBINS + i1] = r1, i1 = b4.m128i_i32[1];
				binbox[2 * BVHBINS + i2] = r2, i2 = b4.m128i_i32[2];
			}
			// final business for final fragment
			const __m256 b0 = binbox[i0], b1 = binbox[BVHBINS + i1], b2 = binbox[2 * BVHBINS + i2];
			count[0][i0]++, count[1][i1]++, count[2][i2]++;
			r0 = _mm256_max_ps( b0, f ), r1 = _mm256_max_ps( b1, f ), r2 = _mm256_max_ps( b2, f );
			binbox[i0] = r0, binbox[BVHBINS + i1] = r1, binbox[2 * BVHBINS + i2] = r2;
			// calculate per-split totals
			float splitCost = 1e30f;
			uint bestAxis = 0, bestPos = 0, n = newNodePtr, j = node.leftFirst + node.triCount, src = node.leftFirst;
			const __m256* bb = binbox;
			for (int a = 0; a < 3; a++, bb += BVHBINS) if ((node.aabbMax[a] - node.aabbMin[a]) > minDim.cell[a])
			{
				// hardcoded bin processing for BVHBINS == 8, see end of file for generic code.
				assert( BVHBINS == 8 );
				const uint lN0 = count[a][0], rN0 = count[a][7];
				const __m256 lb0 = bb[0], rb0 = bb[7];
				const uint lN1 = lN0 + count[a][1], rN1 = rN0 + count[a][6], lN2 = lN1 + count[a][2];
				const uint rN2 = rN1 + count[a][5], lN3 = lN2 + count[a][3], rN3 = rN2 + count[a][4];
				const __m256 lb1 = _mm256_max_ps( lb0, bb[1] ), rb1 = _mm256_max_ps( rb0, bb[6] );
				const __m256 lb2 = _mm256_max_ps( lb1, bb[2] ), rb2 = _mm256_max_ps( rb1, bb[5] );
				const __m256 lb3 = _mm256_max_ps( lb2, bb[3] ), rb3 = _mm256_max_ps( rb2, bb[4] );
				const uint lN4 = lN3 + count[a][4], rN4 = rN3 + count[a][3], lN5 = lN4 + count[a][5];
				const uint rN5 = rN4 + count[a][2], lN6 = lN5 + count[a][6], rN6 = rN5 + count[a][1];
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
			uint t, fr = triIdx[src];
			for (uint i = 0; i < node.triCount; i++)
			{
				const uint bi = (uint)((fragment[fr].bmax[bestAxis] - fragment[fr].bmin[bestAxis] - nmin) * rpd);
				if (bi <= bestPos) fr = triIdx[++src]; else t = fr, fr = triIdx[src] = triIdx[--j], triIdx[j] = t;
			}
			// create child nodes and recurse
			const uint leftCount = src - node.leftFirst, rightCount = node.triCount - leftCount;
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
#ifdef _MSC_VER
#pragma warning ( pop ) // restore 4701
#endif

#endif

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
			for (uint first = node.leftFirst, j = 0; j < node.triCount; j++)
			{
				const uint vertIdx = triIdx[first + j] * 3;
				aabbMin = tinybvh_min( aabbMin, tris[vertIdx] ), aabbMax = tinybvh_max( aabbMax, tris[vertIdx] );
				aabbMin = tinybvh_min( aabbMin, tris[vertIdx + 1] ), aabbMax = tinybvh_max( aabbMax, tris[vertIdx + 1] );
				aabbMin = tinybvh_min( aabbMin, tris[vertIdx + 2] ), aabbMax = tinybvh_max( aabbMax, tris[vertIdx + 2] );
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
int BVH::Intersect( Ray& ray ) const
{
	// traverse bvh
	BVHNode* node = &bvhNode[0], * stack[64];
	uint stackPtr = 0, steps = 0;
	while (1)
	{
		steps++;
		if (node->isLeaf())
		{
			for (uint i = 0; i < node->triCount; i++) IntersectTri( ray, triIdx[node->leftFirst + i] );
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		BVHNode* child1 = &bvhNode[node->leftFirst], * child2 = &bvhNode[node->leftFirst + 1];
		float dist1 = child1->Intersect( ray ), dist2 = child2->Intersect( ray );
		if (dist1 > dist2) { swap( dist1, dist2 ); swap( child1, child2 ); }
		if (dist1 == 1e30f)
		{
			if (stackPtr == 0) break; else node = stack[--stackPtr];
		}
		else
		{
			node = child1;
			if (dist2 != 1e30f) stack[stackPtr++] = child2;
		}
	}
	return steps;
}

// IntersectTri
void BVH::IntersectTri( Ray& ray, const uint idx ) const
{
	// Moeller-Trumbore ray/triangle intersection algorithm
	const uint vertIdx = idx * 3;
	const bvhvec3 edge1 = tris[vertIdx + 1] - tris[vertIdx];
	const bvhvec3 edge2 = tris[vertIdx + 2] - tris[vertIdx];
	const bvhvec3 h = cross( ray.D, edge2 );
	const float a = dot( edge1, h );
	if (fabs( a ) < 0.0000001f) return; // ray parallel to triangle
	const float f = 1 / a;
	const bvhvec3 s = ray.O - bvhvec3( tris[vertIdx] );
	const float u = f * dot( s, h );
	if (u < 0 || u > 1) return;
	const bvhvec3 q = cross( s, edge1 );
	const float v = f * dot( ray.D, q );
	if (v < 0 || u + v > 1) return;
	const float t = f * dot( edge2, q );
	if (t > 0 && t < ray.hit.t) ray.hit.t = t, ray.hit.u = u, ray.hit.v = v, ray.hit.prim = idx;
}

// IntersectAABB
float BVH::IntersectAABB( const Ray& ray, const bvhvec3& aabbMin, const bvhvec3& aabbMax )
{
	// "slab test" ray/AABB intersection
	float tx1 = (aabbMin.x - ray.O.x) * ray.rD.x, tx2 = (aabbMax.x - ray.O.x) * ray.rD.x;
	float tmin = min( tx1, tx2 ), tmax = max( tx1, tx2 );
	float ty1 = (aabbMin.y - ray.O.y) * ray.rD.y, ty2 = (aabbMax.y - ray.O.y) * ray.rD.y;
	tmin = max( tmin, min( ty1, ty2 ) ), tmax = min( tmax, max( ty1, ty2 ) );
	float tz1 = (aabbMin.z - ray.O.z) * ray.rD.z, tz2 = (aabbMax.z - ray.O.z) * ray.rD.z;
	tmin = max( tmin, min( tz1, tz2 ) ), tmax = min( tmax, max( tz1, tz2 ) );
	if (tmax >= tmin && tmin < ray.hit.t && tmax >= 0) return tmin; else return 1e30f;
}

#endif

}

#endif

// EOF