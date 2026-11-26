// gpu-side code for ray traversal

struct BVHNode
{
	float4 lmin; // unsigned left in w
	float4 lmax; // unsigned right in w
	float4 rmin; // unsigned triCount in w
	float4 rmax; // unsigned firstTri in w
};

struct Ray
{
	// data is defined here as 16-byte values to encourage the compilers
	// to fetch 16 bytes at a time: 12 (so, 8 + 4) will be slower.
	float4 O, D, rD; // 48 byte
	float4 hit; // 16 byte
};

// #define CWBVH_COMPRESSED_TRIS // sync with tiny_bvh.h
// #define BVH4_GPU_COMPRESSED_TRIS // sync with tiny_bvh.h

// BVH traversal stack size 
#define STACK_SIZE 32

// Low-level optimizations for specific platforms
#ifdef ISINTEL // Iris Xe, Arc, ..
// #define USE_VLOAD_VSTORE
#define SIMD_AABBTEST
#elif defined ISNVIDIA // 2080, 3080, 4080, ..
#define USE_VLOAD_VSTORE
// #define SIMD_AABBTEST
#elif defined ISAMD
#define USE_VLOAD_VSTORE
// #define SIMD_AABBTEST
#else // unkown GPU
// #define USE_VLOAD_VSTORE
#define SIMD_AABBTEST
#endif

// The _packed kernels take 32-byte rays (O.xyz, tmax; D.xyz, unused) and write 16-byte hits (t, u, v, prim), or
// one occlusion word (1: occluded) for _packed_any, to a separate array. Faster than the Ray variants: a 16-byte
// write into each 64-byte Ray can cost more than a coherent trace.
#ifndef TINYBVH_SAFE_RCP
#define TINYBVH_SAFE_RCP
// 1 / x, or +/-2^100 for a (near) zero x, as tinybvh_safercp on the host.
inline float safe_rcp( const float x ) { return fabs( x ) >= 0x1p-100f ? 1.0f / x : copysign( 0x1p100f, x ); }
inline float3 safe_rcp3( const float3 d ) { return (float3)( safe_rcp( d.x ), safe_rcp( d.y ), safe_rcp( d.z ) ); }
#endif

// Includes for traversal kernel implementations:
#include "traverse_bvh2.cl"
#include "traverse_bvh4.cl"
#include "traverse_cwbvh.cl"