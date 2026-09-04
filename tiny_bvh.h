/*
The MIT License (MIT)

Copyright (c) 2024-2026, Jacco Bikker / Breda University of Applied Sciences.

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
// The library consists of this file plus the headers it includes:
//   tiny_bvh_base.h        platform-neutral code: types, builders and traversal
//   tiny_bvh_x86_float.h   SSE / AVX / AVX2 kernels for the single precision layouts
//   tiny_bvh_x86_double.h  x86 kernels for the double precision layouts
//   tiny_bvh_arm_float.h   NEON kernels for the single precision layouts
//   tiny_bvh_arm_double.h  ARM kernels for the double precision layouts
// Keep them together; only tiny_bvh.h is included by application code.
// Instantiate a BVH and build it for a list of triangles:
//   BVH bvh;
//   bvh.Build( (bvhvec4*)myVerts, numTriangles );
//   Ray ray( bvhvec3( 0, 0, 0 ), bvhvec3( 0, 0, 1 ) );
//   bvh.Intersect( ray );
// After this, intersection information is in ray.hit.

// TinyBVH can use custom vector types by defining TINYBVH_USE_CUSTOM_VECTOR_TYPES once before inclusion.
// To define custom vector types create a tinybvh namespace with the appropriate using directives, e.g.:
//	 namespace tinybvh
//   {
//     using bvhint2 = math::int2;
//     using bvhint3 = math::int3;
//     using bvhuint2 = math::uint2;
//     using bvhuint3 = math::uint3;
//     using bvhuint4 = math::uint4;
//     using bvhvec2 = math::float2;
//     using bvhvec3 = math::float3;
//     using bvhvec4 = math::float4;
//     using bvhdbl3 = math::double3;
//     using bvhmat4 = math::mat4x4;
//   }
//
//	 #define TINYBVH_USE_CUSTOM_VECTOR_TYPES
//   #include <tiny_bvh.h>

// TinyBVH can be further configured using #defines, to be specified before the #include:
// #define BVHBINS 8        - the number of bins to use in regular BVH construction. Default is 8.
// #define HQBVHBINS 32     - the number of bins to use in SBVH construction. Default is 8.
// #define INST_IDX_BITS 10 - the number of bits to use for the instance index. Default is 32,
//                            which stores the bits in a separate field in tinybvh::Intersection.
// #define C_INT 1          - the estimated cost of a primitive intersection test. Default is 1.
// #define C_TRAV 1         - the estimated cost of a traversal step. Default is 1.

// See tiny_bvh_minimal.cpp for basic usage. In short:
// instantiate a BVH: tinybvh::BVH bvh;
// build it: bvh.Build( (tinybvh::bvhvec4*)triangleData, TRIANGLE_COUNT );
// ..where triangleData is an array of four-component float vectors:
// - For a single triangle, provide 3 vertices,
// - For each vertex provide x, y and z.
// The fourth float in each vertex is a dummy value and exists purely for
// a more efficient layout of the data in memory.

// A manual for TinyBVH can be found here:
// https://jacco.ompf2.com/2025/01/24/tinybvh-manual-basic-use
// https://jacco.ompf2.com/2025/01/25/tinybvh-manual-advanced

// More information about the BVH data structure:
// https://jacco.ompf2.com/2022/04/13/how-to-build-a-bvh-part-1-basics

// Further references: See README.md

// Get the latest version of TinyBVH from GitHub:
// github.com/jbikker/tinybvh

// Author and contributors:
// Jacco Bikker: library and examples
// Wenzel Jakob: NEON code, refactoring support
// Eddy L O Jansson: g++ / clang support
// Aras Pranckevičius: non-Intel architecture support
// Jefferson Amstutz: CMake support
// Christian Oliveros: WASM / EMSCRIPTEN support
// Thierry Cantenot: user-defined alloc & free
// David Peicho: slices & Rust bindings, API advice
// .. and other contributors: see GitHub page.

#ifndef TINY_BVH_H_
#define TINY_BVH_H_

// Library version:
#define TINY_BVH_VERSION_MAJOR	1
#define TINY_BVH_VERSION_MINOR	8
#define TINY_BVH_VERSION_SUB	2

// Cached BVH file version - increases only when file layout changes.
#define TINY_BVH_CACHE_VERSION	183

// Run-time checks / debugging.
// #define PARANOID // checks out-of-bound access of slices
// #define SLICEDUMP // dumps the slice used for building to a file - debug feature.

// Binned BVH building: bin count.
#ifndef BVHBINS
#define BVHBINS 8
#endif
#ifndef HQBVHBINS
#define HQBVHBINS 8
#define MAXHQBINS 128
#endif
#define AVXBINS 8 // must stay at 8.

// Stack size for all CPU-side traversal functions.
#ifndef TINYBVH_STACK_SIZE
#define TINYBVH_STACK_SIZE 128
#endif

// TLAS setting
// Note: Except when INST_IDX_BITS is set to 32, the instance index is encoded in
// the top bits of the prim idx field.
// Max number of instances in TLAS: 2 ^ INST_IDX_BITS
// Max number of primitives per BLAS: 2 ^ (32 - INST_IDX_BITS)
#ifndef INST_IDX_BITS
#define INST_IDX_BITS 32 // Use 4..~12 to use prim field bits for instance id, or set to 32 to store index in separate field.
#endif

// SAH BVH building: Heuristic parameters
// CPU traversal: C_INT = 1, C_TRAV = 1 seems optimal.
// These are defaults, which initialize the public members c_int and c_trav in
// BVHBase (and thus each BVH instance).
#ifndef C_INT
#define C_INT	1
#endif
#ifndef C_TRAV
#define C_TRAV	1
#endif
#ifndef W_EPO
#define W_EPO	0.71f
#endif

// SBVH: "Unsplitting"
#define SBVH_UNSPLITTING
#define RDH_MAX_WEIGHT 0.8f

// Triangle intersection: "Watertight"
// #define WATERTIGHT_TRITEST

// 'Infinity' values
#define BVH_FAR	1e30f		// actual valid ieee range: 3.40282347E+38
#define BVH_RCP_FAR	0x1p100f	// reciprocal of a zero direction component; see tinybvh_safercp
#define BVH_DBL_FAR 1e300	// actual valid ieee range: 1.797693134862315E+308
#define BVH_DBL_RCP_FAR 0x1p1000	// double precision counterpart of BVH_RCP_FAR

// Threaded builds: spawn subtree tasks down to this depth (up to 2^N tasks).
#ifndef MT_SPAWN_DEPTH
#define MT_SPAWN_DEPTH 9
#endif
// Threaded builds: only spawn a task if the larger child has at least this many primitives.
#ifndef MT_SPAWN_MIN_PRIMS
#define MT_SPAWN_MIN_PRIMS 5000
#endif
#ifndef MT_BUILD_THRESHOLD
#define MT_BUILD_THRESHOLD 50000 // single-threaded builds below this triangle count
#endif

// Features
#ifndef NO_DOUBLE_PRECISION_SUPPORT
#define DOUBLE_PRECISION_SUPPORT
#endif
#ifndef NO_INDEXED_GEOMETRY
#define ENABLE_INDEXED_GEOMETRY
#endif
#ifndef NO_CUSTOM_GEOMETRY
#define ENABLE_CUSTOM_GEOMETRY
#endif
#ifndef NO_THREADED_BUILDS // if defined, TinyBVH compiles using C++14.
#define ENABLE_THREADED_BUILDS 
#endif
#ifndef NO_VOXEL_SUPPORT
#define ENABLE_VOXEL_SUPPORT
#endif
#ifdef USE_DEPRECATED_LAYOUT
#define ENABLE_BVH_SOA 
#endif
// #define TINYBVH_USE_CUSTOM_VECTOR_TYPES
// #define TINYBVH_NO_SIMD

// C++ features
#if __cplusplus >= 202002L
#define ISLIKELY [[likely]]
#define ISUNLIKELY [[unlikely]]
#else
#define ISLIKELY
#define ISUNLIKELY
#endif

// Experimental / WIP features

// CWBVH triangle format - doesn't seem to help on GPU?
#define CWBVH_COMPRESSED_TRIS
// BVH4 triangle format
// #define BVH4_GPU_COMPRESSED_TRIS
// #define CWBVH_REPORT_FULLNESS

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
#ifdef _WIN32 // MinGW / clang-cl: no C11 aligned_alloc in the CRT, use _aligned_malloc
#include <malloc.h> // for alloc/free
#endif
#endif
#include <cstdint>
#include <atomic> // for SBVH builds
#include <new> // for placement new, in BVHBase::ContextNew

// Platform-independent compile-time warnings.
#define EMIT_COMPILER_WARNING_STRINGIFY0(x) #x
#define EMIT_COMPILER_WARNING_STRINGIFY1(x) EMIT_COMPILER_WARNING_STRINGIFY0(x)
#ifdef __GNUC__
#define EMIT_COMPILER_WARNING_COMPOSE(x) GCC warning x
#else
#define EMIT_COMPILER_MESSAGE_PREFACE(type) \
__FILE__ "(" EMIT_COMPILER_WARNING_STRINGIFY1(__LINE__) "): " type ": "
#define EMIT_COMPILER_WARNING_COMPOSE(x) message(EMIT_COMPILER_MESSAGE_PREFACE("warning C0000") x)
#endif
#define WARNING(x) _Pragma(EMIT_COMPILER_WARNING_STRINGIFY1(EMIT_COMPILER_WARNING_COMPOSE(x)))

// Inlining.
#if defined _MSC_VER
#define TINYBVH_FORCEINLINE __forceinline
#elif defined __GNUC__ || defined __clang__
#define TINYBVH_FORCEINLINE __attribute__((always_inline)) inline
#else
#define TINYBVH_FORCEINLINE inline
#endif

// SSE/AVX/AVX2/NEON support.
// SSE4.2 availability: Since Nehalem (2008)
// AVX1 availability: Since Sandy Bridge (2011)
// AVX2 availability: Since Haswell (2013)
#ifndef TINYBVH_NO_SIMD
#if defined __x86_64__ || defined _M_X64 || defined __wasm_simd128__ || defined __wasm_relaxed_simd__
#if !defined __SSE4_2__  && !defined _MSC_VER
WARNING( "SSE4.2 not enabled in compilation." )
#else
#define BVH_USESSE
#ifndef __SSE4_2__
#define __SSE4_2__		// msvc doesn't set the SSE flag
#endif
#endif
#if !defined __AVX__
WARNING( "AVX not enabled in compilation." )
#define TINYBVH_NO_SIMD
#else
#define BVH_USEAVX		// required for BuildAVX and others
#define BVH_USESSE
#endif
#if !defined __AVX2__ || (!defined __FMA__ && !defined _MSC_VER)
WARNING( "AVX2 and FMA not enabled in compilation." )
#define TINYBVH_NO_SIMD
#else
#define BVH_USEAVX2		// required for BVH8_CPU
#define BVH_USEAVX
#define BVH_USESSE
#endif
#include "immintrin.h"	// for __m128 and __m256
#elif defined __aarch64__ || defined _M_ARM64
#if !defined __ARM_NEON && !defined __APPLE__
WARNING( "NEON not enabled in compilation." )
#define TINYBVH_NO_SIMD
#else
#define BVH_USENEON
#include "arm_neon.h"
#endif
#else // 32-bit x86, PowerPC, RISC-V, ...: TinyBVH has no SIMD path for these.
#define TINYBVH_NO_SIMD
#endif
#endif // TINYBVH_NO_SIMD

// aligned memory allocation
// an application can override the allocator by defining both
// TINYBVH_ALIGNED_ALLOC(alignment,size) and TINYBVH_ALIGNED_FREE(ptr).
#if defined(TINYBVH_ALIGNED_ALLOC) != defined(TINYBVH_ALIGNED_FREE)
#error "Define both TINYBVH_ALIGNED_ALLOC and TINYBVH_ALIGNED_FREE, or neither."
#endif
#ifndef ALIGNED
#ifdef _MSC_VER
#define ALIGNED( x ) __declspec( align( x ) )
#else
#define ALIGNED( x ) __attribute__( ( aligned( x ) ) )
#endif
#endif
#define TINYBVH_ALIGNED( x ) ALIGNED( x ) // prefixed alias; 'ALIGNED' may collide.

// Derived TLAS things; for convenience.
#define INST_IDX_SHFT (32 - INST_IDX_BITS)
#if INST_IDX_BITS == 32
#define PRIM_IDX_MASK 0xffffffff // instance index stored separately.
#else
#define PRIM_IDX_MASK ((1 << INST_IDX_SHFT) - 1) // instance index stored in top bits of hit.prim.
#endif

#endif // TINY_BVH_H_

#include "tiny_bvh_base.h"
#if defined BVH_USESSE
#include "tiny_bvh_x86_float.h"
#include "tiny_bvh_x86_double.h"
#elif defined BVH_USENEON
#include "tiny_bvh_arm_float.h"
#include "tiny_bvh_arm_double.h"
#endif

#ifdef TINYBVH_IMPLEMENTATION

// Explicit instantiations of the templated layouts. Application code only sees
// the declarations in the interface part of the headers; the definitions are
// compiled once, in the translation unit that defines TINYBVH_IMPLEMENTATION.
namespace tinybvh {

template class impl::BVHBase<float, uint32_t>;
template class impl::BVH<float, uint32_t>;
template class impl::BVH_Verbose<float, uint32_t>;
template class impl::BVH_GPU<float, uint32_t>;
#ifdef ENABLE_BVH_SOA
template class impl::BVH_SoA<float, uint32_t>;
#endif
template class impl::MBVH<4, float, uint32_t>;
template class impl::MBVH<8, float, uint32_t>;
template class impl::BVH4_GPU<float, uint32_t>;
template class impl::BVH4_CPU<float, uint32_t>;
template class impl::BVH8_CWBVH<float, uint32_t>;
template class impl::BVH8_CPU<float, uint32_t>;
template class impl::BLASInstance<float, uint32_t>;
#ifdef DOUBLE_PRECISION_SUPPORT
template class impl::BVHBase<double, uint64_t>;
template class impl::BVH<double, uint64_t>;
template class impl::BVH_Verbose<double, uint64_t>;
template class impl::BLASInstance<double, uint64_t>;
#endif

} // namespace tinybvh

#endif // TINYBVH_IMPLEMENTATION
