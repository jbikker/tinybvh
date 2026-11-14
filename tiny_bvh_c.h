/*
tiny_bvh_c.h - C API for TinyBVH.

The MIT License (MIT)
Copyright (c) 2024-2026, Jacco Bikker / Breda University of Applied Sciences.
See tiny_bvh.h for the full license text.

This header is plain C99 (C11 preferred) and also compiles as C++. The implementation
is tiny_bvh_c.cpp, a C++17 translation unit that contains TinyBVH itself:

    c++ -std=c++17 -O3 -mavx2 -mfma -c tiny_bvh_c.cpp                   (static)
    c++ -std=c++17 -O3 -mavx2 -mfma -fPIC -shared -fvisibility=hidden \
        -Wl,-Bsymbolic -DTBVH_SHARED tiny_bvh_c.cpp -o libtinybvh_c.so    (shared)

Clients of a shared build define TBVH_SHARED as well. A shared build exports only the
tbvh_ functions, so it can share a process with C++ code that uses TinyBVH directly;
a static build cannot be linked into such a program. The SIMD paths are chosen when
tiny_bvh_c.cpp is compiled; tbvh_get_config reports what the library was built with.
Without AVX2 (x64) or NEON (arm64) every layout still works, on scalar code.

Conventions
- Every object is an opaque tbvh_bvh handle, created for one layout (tbvh_layout).
- Functions return tbvh_result (0 = TBVH_SUCCESS). Traversal functions return a
  value >= 0 on success (the traversal cost, or the occlusion result) or a negated
  tbvh_result (< 0), so the hot path needs no out-parameters. tbvh_get_last_error
  describes the most recent failure on the calling thread.
- TinyBVH does not copy geometry: vertex and index buffers must stay alive and in
  place while a BVH built over them is in use, and for tbvh_refit. Layouts that embed
  triangle data (BVH4_CPU, BVH8_CPU, the GPU layouts) do not read them during traversal.
- The same holds for the instance array passed to tbvh_build_tlas.
- Rays, hit records and instances are binary-identical to their C++ counterparts, so
  traversal involves no conversion. tbvh_abi_ok() checks this header against the
  library; binding generators should call it once.
- Fatal errors inside TinyBVH (bad input it detects, NaN rays, failed allocations, ..)
  are reported as TBVH_ERROR_FATAL or TBVH_ERROR_OUT_OF_MEMORY instead of terminating
  the process, unless the library is built with TBVH_NO_EXCEPTIONS. When such an error
  comes from a call that modifies a handle (build, refit, optimize, load), the handle
  becomes unusable: other calls refuse it with TBVH_ERROR_INVALID_HANDLE, and
  tbvh_destroy frees it without releasing the layout's buffers, which may be in an
  inconsistent state. A fatal error on a worker thread of a threaded build still
  terminates the process.
- Thread safety: traversal of one handle from many threads is safe; building,
  refitting or destroying a handle must not overlap with any other use of it.
*/

#ifndef TINY_BVH_C_H_
#define TINY_BVH_C_H_

#include <stddef.h>
#include <stdint.h>
#include <math.h>

#define TBVH_C_API_VERSION 1

#if defined(TBVH_SHARED) && defined(_WIN32)
#ifdef TBVH_BUILDING
#define TBVH_API __declspec(dllexport)
#else
#define TBVH_API __declspec(dllimport)
#endif
#elif defined(TBVH_SHARED) && (defined(__GNUC__) || defined(__clang__))
#define TBVH_API __attribute__((visibility("default")))
#else
#define TBVH_API
#endif

#if defined(__cplusplus)
#define TBVH_ALIGN(n) alignas(n)
#elif defined(_MSC_VER)
#define TBVH_ALIGN(n) __declspec(align(n))
#elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
#define TBVH_ALIGN(n) _Alignas(n)
#else
#define TBVH_ALIGN(n) __attribute__((aligned(n)))
#endif

#if defined(_MSC_VER) && !defined(__cplusplus)
#define TBVH_INLINE static __inline
#else
#define TBVH_INLINE static inline
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------- */
/* Results                                                                   */
/* ------------------------------------------------------------------------- */

typedef enum tbvh_result
{
	TBVH_SUCCESS = 0,
	TBVH_ERROR_INVALID_ARGUMENT = 1,	/* null pointer, zero count, misaligned data, .. */
	TBVH_ERROR_INVALID_HANDLE = 2,		/* null, destroyed or foreign handle */
	TBVH_ERROR_UNSUPPORTED = 3,			/* the layout does not offer this operation */
	TBVH_ERROR_WRONG_PRECISION = 4,		/* float call on a double layout, or vice versa */
	TBVH_ERROR_NOT_BUILT = 5,			/* traversal or query on an empty handle */
	TBVH_ERROR_NOT_REFITTABLE = 6,		/* spatial splits or presplitting were used */
	TBVH_ERROR_OUT_OF_MEMORY = 7,		/* an allocation failed */
	TBVH_ERROR_IO = 8,					/* file could not be read or written, or does not match */
	TBVH_ERROR_FATAL = 9,				/* TinyBVH raised a fatal error; see tbvh_get_last_error */
	TBVH_ERROR_INTERNAL = 10			/* unexpected C++ exception */
} tbvh_result;

/* Short constant description of a result code. */
TBVH_API const char* tbvh_result_string( tbvh_result result );
/* Description of the most recent failure on the calling thread. Only failing calls
   write it; the pointer stays valid until the next failure on this thread. */
TBVH_API const char* tbvh_get_last_error( void );
/* Optional process-wide callback, invoked on every failure before the call returns.
   Set it once, before other threads use the library. */
typedef void (*tbvh_error_fn)(tbvh_result result, const char* message, void* userdata);
TBVH_API void tbvh_set_error_callback( tbvh_error_fn fn, void* userdata );

/* ------------------------------------------------------------------------- */
/* Layouts                                                                   */
/* ------------------------------------------------------------------------- */

typedef enum tbvh_layout
{
	/* single precision, 32-bit indices */
	TBVH_LAYOUT_BVH = 1,				/* binary BVH, Wald layout: reference / TLAS / custom geometry */
	TBVH_LAYOUT_BVH_GPU = 2,			/* binary BVH, Aila & Laine layout */
	TBVH_LAYOUT_BVH4_CPU = 3,			/* 4-wide, CPU traversal (SSE / NEON) */
	TBVH_LAYOUT_BVH4_GPU = 4,			/* 4-wide, quantized, GPU traversal */
	TBVH_LAYOUT_BVH8_CPU = 5,			/* 8-wide, CPU traversal (AVX2) - fastest on x64 */
	TBVH_LAYOUT_BVH8_CWBVH = 6,			/* compressed wide BVH (Ylitie et al.), GPU traversal */
	TBVH_LAYOUT_BVH4_AMD_HW = 7,		/* AMD ray tracing hardware, BVH4 (buffers only) */
	TBVH_LAYOUT_CWBVH8_AMD_HW = 8,		/* AMD ray tracing hardware, BVH8 (buffers only) */
	TBVH_LAYOUT_BVH_DOBB = 9,			/* DOBB-BVH (Kern et al., 2025); experimental */
	/* double precision, 64-bit indices */
	TBVH_LAYOUT_BVH_DOUBLE = 101,
	TBVH_LAYOUT_BVH4_CPU_DOUBLE = 102
} tbvh_layout;

/* Nonzero if 'layout' is compiled into the library. */
TBVH_API int tbvh_layout_supported( tbvh_layout layout );

/* ------------------------------------------------------------------------- */
/* Library configuration                                                     */
/* ------------------------------------------------------------------------- */

#define TBVH_SIMD_SSE		(1u << 0)
#define TBVH_SIMD_AVX		(1u << 1)
#define TBVH_SIMD_AVX2		(1u << 2)	/* AVX2 + FMA: SIMD builder, fast BVH8_CPU */
#define TBVH_SIMD_NEON		(1u << 3)

#define TBVH_FEATURE_DOUBLE				(1u << 0)
#define TBVH_FEATURE_THREADED_BUILDS	(1u << 1)
#define TBVH_FEATURE_BUILTIN_POOL		(1u << 2)	/* tbvh_context_default provides a thread pool */
#define TBVH_FEATURE_INDEXED			(1u << 3)
#define TBVH_FEATURE_CUSTOM_GEOMETRY	(1u << 4)
#define TBVH_FEATURE_RECOVERABLE_ERRORS	(1u << 5)	/* fatal errors become TBVH_ERROR_FATAL */

typedef struct tbvh_config
{
	uint32_t api_version;		/* TBVH_C_API_VERSION of the library */
	uint32_t version_major, version_minor, version_sub; /* TinyBVH version */
	uint32_t cache_version;		/* file format version of tbvh_save */
	uint32_t simd;				/* TBVH_SIMD_* */
	uint32_t features;			/* TBVH_FEATURE_* */
	uint32_t bvh_bins;			/* BVHBINS */
	uint32_t hq_bins_default;	/* HQBVHBINS */
	uint32_t hq_bins_max;		/* MAXHQBINS */
	uint32_t bundle_rays;		/* rays per tbvh_intersect_bundle call */
	uint32_t stack_size;		/* TINYBVH_STACK_SIZE */
	uint32_t sizeof_ray, sizeof_rayd;
	uint32_t sizeof_instance, sizeof_instanced;
	uint32_t sizeof_context, sizeof_build_settings;
} tbvh_config;

TBVH_API void tbvh_get_config( tbvh_config* config );

/* Joins the worker threads of the built-in build pool. Optional; the pool is
   released at exit, and re-created by the next threaded build. */
TBVH_API void tbvh_shutdown_thread_pool( void );

/* ------------------------------------------------------------------------- */
/* Rays and hits                                                             */
/* ------------------------------------------------------------------------- */

#define TBVH_FAR			1e30f
#define TBVH_DBL_FAR		1e300
#define TBVH_RCP_FAR		1.2676506002282294e30f		/* 2^100: rD for a zero D component */
#define TBVH_DBL_RCP_FAR	1.0715086071862673e301		/* 2^1000 */
#define TBVH_MASK_ALL		0xFFFFu						/* ray mask: intersect all instances */

#pragma pack(push, 4)
typedef struct tbvh_hit
{
	uint32_t inst;				/* instance index after TLAS traversal; 0 otherwise */
	float t, u, v;				/* distance along the ray, barycentrics */
	uint32_t prim;				/* primitive index */
	void* auxData;				/* free for application use; starts the ray's second cache line */
	union { uint8_t u8[56]; float f32[14]; uint32_t u32[14]; double f64[7]; uint64_t u64[7]; } user;
} tbvh_hit;

typedef struct tbvh_hitd
{
	uint64_t inst;
	double t, u, v;
	uint64_t prim;
	void* auxData;
	union { uint8_t u8[56]; float f32[14]; uint32_t u32[14]; double f64[7]; uint64_t u64[7]; } user;
} tbvh_hitd;
#pragma pack(pop)

/* A ray. rD must hold the reciprocal of D: use tbvh_ray_init, or tbvh_ray_set_dir
   when changing D. The library reads rD, not D, to pick a traversal kernel. */
typedef struct tbvh_ray
{
	TBVH_ALIGN(64) float O[3];
	uint32_t mask;				/* ANDed with tbvh_instance.mask during TLAS traversal */
	float D[3];
	uint32_t instIdx;			/* used internally during TLAS traversal */
	float rD[3];
	tbvh_hit hit;				/* closest hit so far; hit.t is the ray extent on input */
} tbvh_ray;

typedef struct tbvh_rayd
{
	TBVH_ALIGN(64) double O[3];
	uint32_t mask;
	double D[3];
	uint64_t instIdx;
	double rD[3];
	tbvh_hitd hit;
} tbvh_rayd;

TBVH_INLINE float tbvh_safe_rcp( const float x )
{
	const float r = 1.0f / x;
	return fabsf( r ) <= TBVH_RCP_FAR ? r : copysignf( TBVH_RCP_FAR, x );
}
TBVH_INLINE double tbvh_safe_rcpd( const double x )
{
	const double r = 1.0 / x;
	return fabs( r ) <= TBVH_DBL_RCP_FAR ? r : copysign( TBVH_DBL_RCP_FAR, x );
}
TBVH_INLINE void tbvh_ray_set_dir( tbvh_ray* ray, const float dx, const float dy, const float dz )
{
	ray->D[0] = dx, ray->D[1] = dy, ray->D[2] = dz;
	ray->rD[0] = tbvh_safe_rcp( dx ), ray->rD[1] = tbvh_safe_rcp( dy ), ray->rD[2] = tbvh_safe_rcp( dz );
}
TBVH_INLINE void tbvh_rayd_set_dir( tbvh_rayd* ray, const double dx, const double dy, const double dz )
{
	ray->D[0] = dx, ray->D[1] = dy, ray->D[2] = dz;
	ray->rD[0] = tbvh_safe_rcpd( dx ), ray->rD[1] = tbvh_safe_rcpd( dy ), ray->rD[2] = tbvh_safe_rcpd( dz );
}
/* Same as the C++ Ray constructor: D is not normalized; tmax is usually TBVH_FAR. */
TBVH_INLINE void tbvh_ray_init( tbvh_ray* ray, const float ox, const float oy, const float oz,
	const float dx, const float dy, const float dz, const float tmax )
{
	ray->O[0] = ox, ray->O[1] = oy, ray->O[2] = oz;
	tbvh_ray_set_dir( ray, dx, dy, dz );
	ray->mask = TBVH_MASK_ALL, ray->instIdx = 0;
	ray->hit.t = tmax, ray->hit.u = ray->hit.v = 0, ray->hit.prim = 0, ray->hit.inst = 0;
}
TBVH_INLINE void tbvh_rayd_init( tbvh_rayd* ray, const double ox, const double oy, const double oz,
	const double dx, const double dy, const double dz, const double tmax )
{
	ray->O[0] = ox, ray->O[1] = oy, ray->O[2] = oz;
	tbvh_rayd_set_dir( ray, dx, dy, dz );
	ray->mask = TBVH_MASK_ALL, ray->instIdx = 0;
	ray->hit.t = tmax, ray->hit.u = ray->hit.v = 0, ray->hit.prim = 0, ray->hit.inst = 0;
}

/* ------------------------------------------------------------------------- */
/* Instances (TLAS)                                                          */
/* ------------------------------------------------------------------------- */

/* transform is row-major, with the translation in elements 3, 7 and 11.
   invTransform, aabbMin and aabbMax are written by tbvh_build_tlas. */
typedef struct tbvh_instance
{
	TBVH_ALIGN(64) float transform[16];
	float invTransform[16];
	float aabbMin[3];
	uint32_t blasIdx;			/* index into the BLAS array passed to tbvh_build_tlas */
	float aabbMax[3];
	uint32_t mask;				/* tested against tbvh_ray.mask */
} tbvh_instance;

typedef struct tbvh_instanced
{
	TBVH_ALIGN(64) double transform[16];
	double invTransform[16];
	double aabbMin[3];
	uint64_t blasIdx;
	double aabbMax[3];
	uint32_t mask;
} tbvh_instanced;

TBVH_INLINE void tbvh_instance_init( tbvh_instance* inst, const uint32_t blasIdx )
{
	int i;
	for (i = 0; i < 16; i++) inst->transform[i] = inst->invTransform[i] = (i % 5 == 0) ? 1.0f : 0.0f;
	for (i = 0; i < 3; i++) inst->aabbMin[i] = TBVH_FAR, inst->aabbMax[i] = -TBVH_FAR;
	inst->blasIdx = blasIdx, inst->mask = TBVH_MASK_ALL;
}
TBVH_INLINE void tbvh_instanced_init( tbvh_instanced* inst, const uint64_t blasIdx )
{
	int i;
	for (i = 0; i < 16; i++) inst->transform[i] = inst->invTransform[i] = (i % 5 == 0) ? 1.0 : 0.0;
	for (i = 0; i < 3; i++) inst->aabbMin[i] = TBVH_DBL_FAR, inst->aabbMax[i] = -TBVH_DBL_FAR;
	inst->blasIdx = blasIdx, inst->mask = TBVH_MASK_ALL;
}

/* ------------------------------------------------------------------------- */
/* Context: allocation and threading hooks                                   */
/* ------------------------------------------------------------------------- */

/* Same layout as tinybvh::BVHContext. alloc must return 64-byte aligned memory.
   spawn copies payload_size bytes of payload before returning and runs fn on the copy;
   barrier waits for everything spawned; parallel_for runs fn( i, payload ) for i in
   [0, n) and returns when all calls completed. spawn / barrier / parallel_for may be
   null: builds then run on the calling thread. */
typedef struct tbvh_context
{
	void* (*alloc)(size_t size, void* userdata);
	void (*release)(void* ptr, void* userdata);
	void (*spawn)(void (*fn)(void* payload), const void* payload, uint32_t payload_size, void* userdata);
	void (*barrier)(void* userdata);
	void (*parallel_for)(uint32_t n, void (*fn)(uint32_t index, void* payload), void* payload, void* userdata);
	void* userdata;
} tbvh_context;

/* The context a handle gets when tbvh_create receives null: 64-byte aligned
   malloc / free, and the built-in thread pool if TBVH_FEATURE_BUILTIN_POOL. */
TBVH_API void tbvh_context_default( tbvh_context* context );

/* ------------------------------------------------------------------------- */
/* Build settings                                                            */
/* ------------------------------------------------------------------------- */

#define TBVH_BUILD_PRESPLIT					(1u << 0)	/* pre-split triangles before building */
#define TBVH_BUILD_SPATIAL_SPLITS			(1u << 1)	/* Bonsai HQ, or SBVH with TBVH_BUILD_BINNED */
#define TBVH_BUILD_SPATIAL_SKIP_SMALLEST	(1u << 2)	/* SBVH: no spatial splits along the smallest axis */
#define TBVH_BUILD_PRESPLIT_POST_PASS		(1u << 3)	/* un-split primitives in leaves after presplitting */
#define TBVH_BUILD_FULL_SWEEP				(1u << 4)	/* full-sweep SAH builder; for experiments */
#define TBVH_BUILD_BINNED					(1u << 5)	/* binned SAH builders instead of Bonsai */
#define TBVH_BUILD_HPLOC					(1u << 6)	/* H-PLOC builder */
#define TBVH_BUILD_KEEP_SPLITTING			(1u << 7)	/* subdivide regardless of SAH */
#define TBVH_BUILD_POST_OPTIMIZE			(1u << 8)	/* optimize by parallel reinsertion after the build */
#define TBVH_BUILD_USE_SIMD					(1u << 9)	/* use the SIMD builder when available */
#define TBVH_BUILD_THREADED					(1u << 10)	/* use the context's threading hooks */
#define TBVH_BUILD_DISCARD_INTERMEDIATES	(1u << 11)	/* free the intermediate BVH after conversion; no refit */
#define TBVH_BUILD_MINIMIZE_MEM_USE			(1u << 12)	/* size intermediate data exactly */
#define TBVH_BUILD_HQ_ODD_EVEN				(1u << 13)	/* SBVH: one extra bin on odd levels */

typedef enum tbvh_bonsai_top
{
	TBVH_BONSAI_TOP_BINNED = 0,
	TBVH_BONSAI_TOP_FULLSWEEP = 1,
	TBVH_BONSAI_TOP_HPLOC = 2,
	TBVH_BONSAI_TOP_CHEAPEST = 3
} tbvh_bonsai_top;

/* Versioned: struct_size must be sizeof( tbvh_build_settings ); tbvh_build_settings_default
   sets it. Fields map to tinybvh::BVHBuildSettings plus the per-BVH SAH parameters. */
typedef struct tbvh_build_settings
{
	uint32_t struct_size;
	uint32_t flags;					/* TBVH_BUILD_* */
	float min_spatial_overlap;
	float presplit_factor;
	uint32_t bonsai_grid_prims;
	float bonsai_prune_factor;
	uint32_t bonsai_top_tree;		/* tbvh_bonsai_top */
	uint32_t optimize_iterations;
	float c_int, c_trav;			/* SAH costs of a primitive test and of a traversal step */
	uint32_t hq_bins;				/* SBVH bin count, at most tbvh_config.hq_bins_max */
	uint32_t hq_leaf_prims;			/* SBVH: nodes this small become leaves; 0 = SAH decides */
} tbvh_build_settings;

TBVH_API void tbvh_build_settings_default( tbvh_build_settings* settings );

/* ------------------------------------------------------------------------- */
/* Handles                                                                   */
/* ------------------------------------------------------------------------- */

typedef struct tbvh_bvh tbvh_bvh;

/* context may be null; it is copied. The handle allocates through it. */
TBVH_API tbvh_result tbvh_create( tbvh_layout layout, const tbvh_context* context, tbvh_bvh** out );
/* Null is a no-op. */
TBVH_API void tbvh_destroy( tbvh_bvh* bvh );
TBVH_API tbvh_layout tbvh_get_layout( const tbvh_bvh* bvh );

/* Settings apply to subsequent builds of this handle. */
TBVH_API tbvh_result tbvh_set_build_settings( tbvh_bvh* bvh, const tbvh_build_settings* settings );
TBVH_API tbvh_result tbvh_get_build_settings( const tbvh_bvh* bvh, tbvh_build_settings* settings );

/* ------------------------------------------------------------------------- */
/* Building                                                                  */
/* ------------------------------------------------------------------------- */

/* Triangle geometry. Vertices are four floats (x, y, z, w; w is ignored, but some
   layouts carry it along), start 16-byte aligned, and are 'vertex_stride' bytes
   apart: a multiple of 16, or 0 for 16. Without indices, triangle i is vertices
   3i .. 3i+2; prim_count may be 0 for vertex_count / 3. With indices, prim_count
   triangles use 3 indices each, all below vertex_count. */
typedef struct tbvh_geometry
{
	const float* vertices;
	uint32_t vertex_count;
	uint32_t vertex_stride;
	const uint32_t* indices;
	uint32_t prim_count;
} tbvh_geometry;

/* Double precision triangle geometry: three doubles per vertex, 8-byte aligned;
   vertex_stride is a multiple of 8, at least 24, or 0 for 24. */
typedef struct tbvh_geometryd
{
	const double* vertices;
	uint64_t vertex_count;
	uint64_t vertex_stride;
	const uint64_t* indices;
	uint64_t prim_count;
} tbvh_geometryd;

typedef enum tbvh_quality
{
	TBVH_QUALITY_DEFAULT = 0,		/* Build(): Bonsai with the SIMD builder; see the settings */
	TBVH_QUALITY_HIGH = 1,			/* BuildHQ(): spatial splits, for this build only */
	TBVH_QUALITY_QUICK = 2			/* BuildQuick(): TBVH_LAYOUT_BVH only, no indices */
} tbvh_quality;

TBVH_API tbvh_result tbvh_build( tbvh_bvh* bvh, const tbvh_geometry* geometry, tbvh_quality quality );
TBVH_API tbvh_result tbvh_buildd( tbvh_bvh* bvh, const tbvh_geometryd* geometry, tbvh_quality quality );

/* Custom geometry, TBVH_LAYOUT_BVH only. The intersect callback tests primitive
   'prim' and, if it finds a hit closer than ray->hit.t, writes ray->hit (t, u, v,
   prim) and returns nonzero. The occluded callback returns nonzero for any hit
   closer than ray->hit.t. Rays passed to the callbacks are in BLAS space. A BVH over
   custom geometry or boxes must not be traversed without both callbacks; TinyBVH
   checks this only when built with PARANOID. */
typedef int (*tbvh_intersect_fn)(tbvh_ray* ray, uint32_t prim, void* userdata);
typedef int (*tbvh_occluded_fn)(const tbvh_ray* ray, uint32_t prim, void* userdata);
typedef void (*tbvh_aabb_fn)(uint32_t prim, float bmin[3], float bmax[3], void* userdata);

/* Sets both callbacks, or clears both when both are null. A triangle build clears them;
   tbvh_build_custom, tbvh_build_aabbs and tbvh_build_tlas keep them, so they may be
   set before those builds. */
TBVH_API tbvh_result tbvh_set_custom_geometry( tbvh_bvh* bvh, tbvh_intersect_fn intersect,
	tbvh_occluded_fn occluded, void* userdata );
/* Builds over prim_count primitives whose bounds come from aabb_fn. */
TBVH_API tbvh_result tbvh_build_custom( tbvh_bvh* bvh, tbvh_aabb_fn aabb_fn, uint32_t prim_count );
/* Builds over prim_count boxes: 8 floats each, min x, y, z, (pad), max x, y, z, (pad).
   Traversal requires custom geometry callbacks; 16-byte aligned. */
TBVH_API tbvh_result tbvh_build_aabbs( tbvh_bvh* bvh, const float* aabbs, uint32_t prim_count );

/* Top-level BVH over instances, TBVH_LAYOUT_BVH (tbvh_build_tlasd: TBVH_LAYOUT_BVH_DOUBLE).
   BLASes are BVH, BVH4_CPU or BVH8_CPU handles of the same precision, already built.
   'instances' is referenced, not copied; rebuild after changing transforms. The BLAS
   array is copied; the BLAS handles must outlive the TLAS. */
TBVH_API tbvh_result tbvh_build_tlas( tbvh_bvh* tlas, tbvh_instance* instances, uint32_t instance_count,
	tbvh_bvh* const* blases, uint32_t blas_count );
TBVH_API tbvh_result tbvh_build_tlasd( tbvh_bvh* tlas, tbvh_instanced* instances, uint64_t instance_count,
	tbvh_bvh* const* blases, uint64_t blas_count );

/* Refit after moving vertices in place. BVH, BVH4_CPU and BVH8_CPU; not after spatial
   splits or presplitting, and not with TBVH_BUILD_DISCARD_INTERMEDIATES. */
TBVH_API tbvh_result tbvh_refit( tbvh_bvh* bvh );
/* Tree optimization by reinsertion; 0 iterations uses the settings' optimize_iterations. */
TBVH_API tbvh_result tbvh_optimize( tbvh_bvh* bvh, uint32_t iterations );

/* ------------------------------------------------------------------------- */
/* Traversal                                                                 */
/* ------------------------------------------------------------------------- */

/* Closest hit: updates ray->hit if a closer hit exists. Returns the traversal cost
   as the layout counts it, or a negated tbvh_result. BVH4_CPU, BVH8_CPU and
   BVH8_CWBVH do not count, and return 0. Not available for the AMD hardware layouts. */
TBVH_API int32_t tbvh_intersect( const tbvh_bvh* bvh, tbvh_ray* ray );
TBVH_API int32_t tbvh_intersectd( const tbvh_bvh* bvh, tbvh_rayd* ray );
/* Any hit closer than ray->hit.t: returns 1 or 0, or a negated tbvh_result. */
TBVH_API int32_t tbvh_is_occluded( const tbvh_bvh* bvh, const tbvh_ray* ray );
TBVH_API int32_t tbvh_is_occludedd( const tbvh_bvh* bvh, const tbvh_rayd* ray );
/* tbvh_config.bundle_rays rays at once, 64-byte aligned: BVH4_CPU, or a TLAS.
   occluded receives one byte (0 / 1) per ray. Return the traversal cost or a
   negated tbvh_result. */
TBVH_API int32_t tbvh_intersect_bundle( const tbvh_bvh* bvh, tbvh_ray* rays );
TBVH_API int32_t tbvh_is_occluded_bundle( const tbvh_bvh* bvh, tbvh_ray* rays, uint8_t* occluded );

/* ------------------------------------------------------------------------- */
/* Queries                                                                   */
/* ------------------------------------------------------------------------- */

#define TBVH_INFO_BUILT			(1u << 0)
#define TBVH_INFO_REFITTABLE	(1u << 1)
#define TBVH_INFO_TLAS			(1u << 2)
#define TBVH_INFO_INDEXED		(1u << 3)
#define TBVH_INFO_CUSTOM		(1u << 4)
#define TBVH_INFO_DOUBLE		(1u << 5)

typedef struct tbvh_info
{
	uint32_t layout;			/* tbvh_layout */
	uint32_t flags;				/* TBVH_INFO_* */
	uint64_t prim_count;		/* primitives, or instances for a TLAS */
	uint64_t index_count;		/* primitive references; above prim_count after spatial splits */
	uint64_t node_count;		/* nodes; for the wide and GPU layouts, nodes of the wide tree they
								   were converted from (0 after tbvh_load) */
	double aabb_min[3], aabb_max[3];
} tbvh_info;

TBVH_API tbvh_result tbvh_get_info( const tbvh_bvh* bvh, tbvh_info* info );
/* SAH cost of the tree; walks the whole tree. */
TBVH_API tbvh_result tbvh_get_sah_cost( const tbvh_bvh* bvh, double* cost );

/* Raw buffers, e.g. for GPU upload. The pointers stay valid until the next build,
   load or destroy of the handle.
   TBVH_LAYOUT_BVH:            NODES (32-byte nodes; 64 for double), PRIM_INDICES
   TBVH_LAYOUT_BVH_GPU:        NODES (64-byte nodes), TRIANGLES (v0, v1 - v0, v2 - v0 as
                               float4; v0.w holds the primitive index), PRIM_INDICES
   TBVH_LAYOUT_BVH4_CPU/8_CPU: NODES (64-byte blocks: interior nodes and leaf triangles)
   TBVH_LAYOUT_BVH4_GPU:       NODES (16-byte blocks: nodes and leaf triangles)
   TBVH_LAYOUT_BVH8_CWBVH:     NODES, TRIANGLES (16-byte blocks)
   TBVH_LAYOUT_*_AMD_HW:       NODES (16-byte blocks, hardware node format)
   TBVH_LAYOUT_BVH_DOBB:       NODES (80-byte nodes), PRIM_INDICES */
typedef enum tbvh_buffer
{
	TBVH_BUFFER_NODES = 0,
	TBVH_BUFFER_TRIANGLES = 1,
	TBVH_BUFFER_PRIM_INDICES = 2
} tbvh_buffer;

TBVH_API tbvh_result tbvh_get_buffer( const tbvh_bvh* bvh, tbvh_buffer buffer, const void** data, size_t* size );

/* ------------------------------------------------------------------------- */
/* Serialization                                                             */
/* ------------------------------------------------------------------------- */

/* Writes the layout's cache file. Files are tied to the TinyBVH cache version, the
   precision and the layout. Not available for BVH_GPU and BVH_DOBB, or for a TLAS or
   custom geometry. TBVH_ERROR_IO if the file could not be written completely. */
TBVH_API tbvh_result tbvh_save( const tbvh_bvh* bvh, const char* file_name );
/* Replaces the handle's tree with one read from a file. Every layout but BVH_DOBB reads
   files that tbvh_save wrote for it; BVH_GPU, BVH4_CPU, BVH8_CPU and the other wide
   layouts also read a TBVH_LAYOUT_BVH file saved with its geometry, and convert it.
   geometry may be null. For TBVH_LAYOUT_BVH, non-null geometry is used instead of the
   geometry in the file; for the other layouts, which store their triangles, it is only
   checked against the file's primitive count. A failed load leaves the tree as it was
   (TBVH_LAYOUT_BVH: possibly empty). Allocations use the handle's context. */
TBVH_API tbvh_result tbvh_load( tbvh_bvh* bvh, const char* file_name, const tbvh_geometry* geometry );
TBVH_API tbvh_result tbvh_loadd( tbvh_bvh* bvh, const char* file_name, const tbvh_geometryd* geometry );

/* ------------------------------------------------------------------------- */
/* ABI check                                                                 */
/* ------------------------------------------------------------------------- */

/* Nonzero if this header matches the library. */
TBVH_INLINE int tbvh_abi_ok( void )
{
	tbvh_config c;
	tbvh_get_config( &c );
	return c.api_version == TBVH_C_API_VERSION &&
		c.sizeof_ray == sizeof( tbvh_ray ) && c.sizeof_rayd == sizeof( tbvh_rayd ) &&
		c.sizeof_instance == sizeof( tbvh_instance ) && c.sizeof_instanced == sizeof( tbvh_instanced ) &&
		c.sizeof_context == sizeof( tbvh_context ) && c.sizeof_build_settings == sizeof( tbvh_build_settings );
}

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* TINY_BVH_C_H_ */
