// tiny_bvh_c.cpp - implementation of the TinyBVH C API (tiny_bvh_c.h).
// This translation unit contains TinyBVH itself; compile it as C++17 or later.
// Define TBVH_NO_EXCEPTIONS to keep TinyBVH's terminate-on-fatal-error behavior.

#define TBVH_BUILDING
#include "tiny_bvh_c.h"

#if defined INST_IDX_BITS && INST_IDX_BITS != 32
#error "The C API stores the instance index in tbvh_hit.inst; INST_IDX_BITS must be 32."
#endif
#ifdef TINYBVH_USE_CUSTOM_VECTOR_TYPES
#error "The C API requires TinyBVH's own vector types."
#endif
#ifdef NORMALIZE_RAY_DIR
#error "tbvh_ray_init follows the 1.9 API: rays are not normalized."
#endif

// TinyBVH's own symbols stay out of the library's interface, so a shared build can coexist
// with a C++ TinyBVH in the same process. System headers come first: they must keep their
// default visibility.
#if (defined __GNUC__ || defined __clang__) && !defined _WIN32
#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <mutex>
#include <new>
#include <thread>
#include <vector>
#include <pthread.h>
#if __has_include(<pthread_np.h>)
#include <pthread_np.h>
#endif
#if defined __x86_64__
#include <immintrin.h>
#elif defined __aarch64__
#include <arm_neon.h>
#endif
#pragma GCC visibility push(hidden)
#define TBVH_HIDDEN_PUSHED
#endif

#define TINYBVH_IMPLEMENTATION
#include "tiny_bvh.h"

#ifdef TBVH_HIDDEN_PUSHED
#pragma GCC visibility pop
#define TBVH_INTERNAL __attribute__((visibility("hidden")))
#else
#define TBVH_INTERNAL
#endif

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <new>
#include <type_traits>
#include <utility>

using namespace tinybvh;

// ============================================================================
// Binary compatibility of the shared types
// ============================================================================

#if defined __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-offsetof" // Intersection is not standard-layout
#endif

#define TBVH_SAME_OFFSET( C, CPP, cm, cppm ) \
	static_assert( offsetof( C, cm ) == offsetof( CPP, cppm ), #C "::" #cm " does not match " #CPP "::" #cppm )
#define TBVH_SAME_SIZE( C, CPP ) \
	static_assert( sizeof( C ) == sizeof( CPP ) && alignof( C ) == alignof( CPP ), #C " does not match " #CPP )

TBVH_SAME_SIZE( tbvh_hit, Intersection );
TBVH_SAME_OFFSET( tbvh_hit, Intersection, inst, inst );
TBVH_SAME_OFFSET( tbvh_hit, Intersection, t, t );
TBVH_SAME_OFFSET( tbvh_hit, Intersection, u, u );
TBVH_SAME_OFFSET( tbvh_hit, Intersection, v, v );
TBVH_SAME_OFFSET( tbvh_hit, Intersection, prim, prim );
TBVH_SAME_OFFSET( tbvh_hit, Intersection, auxData, auxData );
TBVH_SAME_OFFSET( tbvh_hit, Intersection, user, userChar );
TBVH_SAME_SIZE( tbvh_ray, Ray );
TBVH_SAME_OFFSET( tbvh_ray, Ray, O, O );
TBVH_SAME_OFFSET( tbvh_ray, Ray, mask, mask );
TBVH_SAME_OFFSET( tbvh_ray, Ray, D, D );
TBVH_SAME_OFFSET( tbvh_ray, Ray, instIdx, instIdx );
TBVH_SAME_OFFSET( tbvh_ray, Ray, rD, rD );
TBVH_SAME_OFFSET( tbvh_ray, Ray, hit, hit );
TBVH_SAME_SIZE( tbvh_instance, BLASInstance );
TBVH_SAME_OFFSET( tbvh_instance, BLASInstance, transform, transform );
TBVH_SAME_OFFSET( tbvh_instance, BLASInstance, invTransform, invTransform );
TBVH_SAME_OFFSET( tbvh_instance, BLASInstance, aabbMin, aabbMin );
TBVH_SAME_OFFSET( tbvh_instance, BLASInstance, blasIdx, blasIdx );
TBVH_SAME_OFFSET( tbvh_instance, BLASInstance, aabbMax, aabbMax );
TBVH_SAME_OFFSET( tbvh_instance, BLASInstance, mask, mask );
#ifdef DOUBLE_PRECISION_SUPPORT
TBVH_SAME_SIZE( tbvh_hitd, IntersectionEx );
TBVH_SAME_OFFSET( tbvh_hitd, IntersectionEx, inst, inst );
TBVH_SAME_OFFSET( tbvh_hitd, IntersectionEx, t, t );
TBVH_SAME_OFFSET( tbvh_hitd, IntersectionEx, prim, prim );
TBVH_SAME_OFFSET( tbvh_hitd, IntersectionEx, auxData, auxData );
TBVH_SAME_OFFSET( tbvh_hitd, IntersectionEx, user, userChar );
TBVH_SAME_SIZE( tbvh_rayd, RayEx );
TBVH_SAME_OFFSET( tbvh_rayd, RayEx, O, O );
TBVH_SAME_OFFSET( tbvh_rayd, RayEx, mask, mask );
TBVH_SAME_OFFSET( tbvh_rayd, RayEx, D, D );
TBVH_SAME_OFFSET( tbvh_rayd, RayEx, instIdx, instIdx );
TBVH_SAME_OFFSET( tbvh_rayd, RayEx, rD, rD );
TBVH_SAME_OFFSET( tbvh_rayd, RayEx, hit, hit );
TBVH_SAME_SIZE( tbvh_instanced, BLASInstanceEx );
TBVH_SAME_OFFSET( tbvh_instanced, BLASInstanceEx, invTransform, invTransform );
TBVH_SAME_OFFSET( tbvh_instanced, BLASInstanceEx, aabbMin, aabbMin );
TBVH_SAME_OFFSET( tbvh_instanced, BLASInstanceEx, blasIdx, blasIdx );
TBVH_SAME_OFFSET( tbvh_instanced, BLASInstanceEx, aabbMax, aabbMax );
TBVH_SAME_OFFSET( tbvh_instanced, BLASInstanceEx, mask, mask );
#endif
TBVH_SAME_SIZE( tbvh_context, BVHContext );
TBVH_SAME_OFFSET( tbvh_context, BVHContext, alloc, malloc );
TBVH_SAME_OFFSET( tbvh_context, BVHContext, release, free );
TBVH_SAME_OFFSET( tbvh_context, BVHContext, spawn, spawn );
TBVH_SAME_OFFSET( tbvh_context, BVHContext, barrier, barrier );
TBVH_SAME_OFFSET( tbvh_context, BVHContext, parallel_for, parallel_for );
TBVH_SAME_OFFSET( tbvh_context, BVHContext, userdata, userdata );

#if defined __GNUC__
#pragma GCC diagnostic pop
#endif

static_assert( sizeof( bool ) == 1, "tbvh_is_occluded_bundle passes uint8_t* as bool*" );
static_assert( TBVH_RCP_FAR == BVH_RCP_FAR && TBVH_DBL_RCP_FAR == BVH_DBL_RCP_FAR, "tbvh_safe_rcp mismatch" );
static_assert( TBVH_FAR == BVH_FAR && TBVH_DBL_FAR == BVH_DBL_FAR, "far plane mismatch" );
static_assert( TBVH_MASK_ALL == RAY_MASK_INTERSECT_ALL, "ray mask mismatch" );

// ============================================================================
// Errors
// ============================================================================

namespace {

thread_local char tl_lastError[512] = "";
tbvh_error_fn g_errorFn = nullptr;
void* g_errorUser = nullptr;

tbvh_result Fail( const tbvh_result r, const char* fmt, ... )
{
	va_list args;
	va_start( args, fmt );
	vsnprintf( tl_lastError, sizeof( tl_lastError ), fmt, args );
	va_end( args );
	if (g_errorFn) g_errorFn( r, tl_lastError, g_errorUser );
	return r;
}

#ifndef TBVH_NO_EXCEPTIONS
// TinyBVH's fatal errors arrive here; we unwind to the C API entry point.
struct FatalError { char message[512]; };
void FatalHandler( const char* message )
{
	FatalError e;
	snprintf( e.message, sizeof( e.message ), "%s", message );
	throw e;
}
// AlignedAlloc reports a failed allocation as a fatal error; give it its own result code.
tbvh_result FatalResult( const FatalError& e )
{
	const bool oom = strstr( e.message, "AlignedAlloc( .. ), out of memory" ) != nullptr;
	return Fail( oom ? TBVH_ERROR_OUT_OF_MEMORY : TBVH_ERROR_FATAL, "%s", e.message );
}
#define TBVH_TRY try {
#define TBVH_CATCH( RET ) } \
	catch (const FatalError& e) { return RET( FatalResult( e ) ); } \
	catch (const std::bad_alloc&) { return RET( Fail( TBVH_ERROR_OUT_OF_MEMORY, "out of memory" ) ); } \
	catch (...) { return RET( Fail( TBVH_ERROR_INTERNAL, "unexpected C++ exception" ) ); }
// Mutating calls: after an error raised inside TinyBVH the object may hold freed pointers, so
// the handle is marked broken. tbvh_destroy then frees the handle without running the layout's
// destructor (a leak instead of a double free), and every other call refuses it.
#define TBVH_CATCH_MUTATING( H ) } \
	catch (const FatalError& e) { (H)->broken = true; return FatalResult( e ); } \
	catch (const std::bad_alloc&) { (H)->broken = true; return Fail( TBVH_ERROR_OUT_OF_MEMORY, "out of memory" ); } \
	catch (...) { (H)->broken = true; return Fail( TBVH_ERROR_INTERNAL, "unexpected C++ exception" ); }
#else
// The handler reports; TinyBVH terminates the process when it returns.
void FatalHandler( const char* message ) { Fail( TBVH_ERROR_FATAL, "%s", message ); }
#define TBVH_TRY {
#define TBVH_CATCH( RET ) }
#define TBVH_CATCH_MUTATING( H ) }
#endif
#define AS_RESULT( r ) (r)
#define AS_INT( r ) (-(int32_t)(r))

struct InstallFatalHandler { InstallFatalHandler() { tinybvh_set_fatal_error_handler( FatalHandler ); } } g_installFatalHandler;

} // namespace

// ============================================================================
// Handles and dispatch
// ============================================================================

struct TBVH_INTERNAL tbvh_bvh
{
	static constexpr uint32_t MAGIC = 0x48564254u; // 'TBVH'
	uint32_t magic;
	tbvh_layout layout;
	BVHContext ctx;					// for the handle itself and its side allocations
	void* obj;						// the C++ layout object
	void** blasList;				// TLAS: BVHBase<Real, Index>* per BLAS
	uint64_t blasCount;
	tbvh_intersect_fn customIntersect;
	tbvh_occluded_fn customOccluded;
	tbvh_aabb_fn customAabb;
	void* customUser;
	bool broken;					// a mutating call raised an error inside TinyBVH; see TBVH_CATCH_MUTATING
};

namespace {

inline bool Live( const tbvh_bvh* h ) { return h && h->magic == tbvh_bvh::MAGIC; }
inline bool Valid( const tbvh_bvh* h ) { return Live( h ) && !h->broken; }
tbvh_result BadHandle( const tbvh_bvh* h, const char* fn )
{
	if (Live( h )) return Fail( TBVH_ERROR_INVALID_HANDLE, "%s: handle is unusable after an error inside TinyBVH; destroy it", fn );
	return Fail( TBVH_ERROR_INVALID_HANDLE, "%s: invalid handle", fn );
}
inline bool IsDouble( const tbvh_layout l ) { return l == TBVH_LAYOUT_BVH_DOUBLE || l == TBVH_LAYOUT_BVH4_CPU_DOUBLE; }

void* CtxAlloc( const BVHContext& c, const size_t n ) { return c.malloc ? c.malloc( n, c.userdata ) : nullptr; }
void CtxFree( const BVHContext& c, void* p ) { if (p && c.free) c.free( p, c.userdata ); }

// Scalar and index type of a layout class impl::X<Real, Index>.
template <class T> struct LayoutTypes;
template <template <class, class> class L, class R, class I> struct LayoutTypes<L<R, I>> { using Real = R; using Index = I; };
template <class T> using RealOf = typename LayoutTypes<T>::Real;
template <class T> using IndexOf = typename LayoutTypes<T>::Index;
template <class T> using BaseOf = impl::BVHBase<RealOf<T>, IndexOf<T>>;
template <class T> using RayOf = impl::Ray<RealOf<T>, IndexOf<T>>;
template <class T> using SliceOf = typename bvh_traits<RealOf<T>>::slice;
template <class T> constexpr bool IsBVH = std::is_same_v<T, impl::BVH<RealOf<T>, IndexOf<T>>>;

// Capability detection; overload sets differ per layout.
#define TBVH_DETECT( NAME, EXPR ) \
	template <class T, class = void> struct NAME : std::false_type {}; \
	template <class T> struct NAME<T, std::void_t<decltype( EXPR )>> : std::true_type {};
TBVH_DETECT( HasIntersect, std::declval<const T&>().Intersect( std::declval<RayOf<T>&>() ) )
TBVH_DETECT( HasOccluded, std::declval<const T&>().IsOccluded( std::declval<const RayOf<T>&>() ) )
TBVH_DETECT( HasBundle, std::declval<const T&>().IntersectBundle( std::declval<RayOf<T>*>() ) )
TBVH_DETECT( HasBuildHQ, std::declval<T&>().BuildHQ( std::declval<const SliceOf<T>&>(), std::declval<const IndexOf<T>*>(), IndexOf<T>() ) )
TBVH_DETECT( HasRefit, std::declval<T&>().Refit() )
TBVH_DETECT( HasOptimize, std::declval<T&>().Optimize( 1u, 8u ) )
TBVH_DETECT( HasSave, std::declval<T&>().Save( (const char*)0 ) )
TBVH_DETECT( HasStaticLoad, T::Load( (const char*)0, BVHContext() ) )
TBVH_DETECT( HasSAH, std::declval<T&>().SAHCost( IndexOf<T>() ) )
TBVH_DETECT( HasInnerBVH, std::declval<T&>().bvh.verts )
TBVH_DETECT( HasInnerBVH4, std::declval<T&>().bvh4.bvh.verts )
TBVH_DETECT( HasInnerBVH8, std::declval<T&>().bvh8.bvh.verts )
TBVH_DETECT( HasInnerMBVH4, std::declval<T&>().bvh4.mbvhNode )
TBVH_DETECT( HasInnerMBVH8, std::declval<T&>().bvh8.mbvhNode )

// Node count: wide and GPU layouts count blocks, not nodes; report their wide source tree.
template <class T> uint64_t NodeCount( const T& o )
{
	const uint64_t own = static_cast<const BaseOf<T>&>(o).usedNodes;
	if (own) return own;
	if constexpr (HasInnerMBVH4<T>::value) return o.bvh4.usedNodes;
	else if constexpr (HasInnerMBVH8<T>::value) return o.bvh8.usedNodes;
	else return 0;
}

// The binary BVH a layout is built from; it holds the input references and flags.
template <class T> impl::BVH<RealOf<T>, IndexOf<T>>* InnerBVH( T& o )
{
	if constexpr (IsBVH<T>) return &o;
	else if constexpr (HasInnerBVH<T>::value) return &o.bvh;
	else if constexpr (HasInnerBVH4<T>::value) return &o.bvh4.bvh;
	else if constexpr (HasInnerBVH8<T>::value) return &o.bvh8.bvh;
	else return nullptr;
}

#ifdef DOUBLE_PRECISION_SUPPORT
#define TBVH_DOUBLE_CASES \
	case TBVH_LAYOUT_BVH_DOUBLE: return fn( *(BVH_Double*)h->obj ); \
	case TBVH_LAYOUT_BVH4_CPU_DOUBLE: return fn( *(BVH4_Double*)h->obj );
#else
#define TBVH_DOUBLE_CASES
#endif

// Calls fn( layout object ) with the concrete type. tbvh_create guarantees a known layout.
template <class Fn> decltype(auto) Visit( const tbvh_bvh* h, Fn&& fn )
{
	switch (h->layout)
	{
	case TBVH_LAYOUT_BVH_GPU: return fn( *(BVH_GPU*)h->obj );
	case TBVH_LAYOUT_BVH4_CPU: return fn( *(BVH4_CPU*)h->obj );
	case TBVH_LAYOUT_BVH4_GPU: return fn( *(BVH4_GPU*)h->obj );
	case TBVH_LAYOUT_BVH8_CPU: return fn( *(BVH8_CPU*)h->obj );
	case TBVH_LAYOUT_BVH8_CWBVH: return fn( *(BVH8_CWBVH*)h->obj );
	case TBVH_LAYOUT_BVH4_AMD_HW: return fn( *(BVH4_AMD_HW*)h->obj );
	case TBVH_LAYOUT_CWBVH8_AMD_HW: return fn( *(CWBVH8_AMD_HW*)h->obj );
	case TBVH_LAYOUT_BVH_DOBB: return fn( *(BVH_DOBB*)h->obj );
	TBVH_DOUBLE_CASES
	default: return fn( *(BVH*)h->obj ); // TBVH_LAYOUT_BVH
	}
}

template <class T> void* NewLayout( const BVHContext& ctx )
{
	static_assert( alignof(T) <= 64, "layout object needs more than 64-byte alignment" );
	void* mem = CtxAlloc( ctx, sizeof( T ) );
	return mem ? new (mem) T( ctx ) : nullptr;
}

// Builders: precision of the geometry has been checked against the handle.
template <class T> using GeometryOf = std::conditional_t<std::is_same_v<RealOf<T>, float>, tbvh_geometry, tbvh_geometryd>;

template <class T> tbvh_result BuildTriangles( T& o, const GeometryOf<T>& g, const tbvh_quality q )
{
	using Index = IndexOf<T>;
	using Slice = SliceOf<T>;
	using Vertex = typename bvh_traits<RealOf<T>>::vertex;
	const auto stride = g.vertex_stride ? g.vertex_stride : sizeof( Vertex );
	const Index prims = g.indices ? (Index)g.prim_count : (Index)(g.prim_count ? g.prim_count : g.vertex_count / 3);
	const Slice slice( (const Vertex*)g.vertices, g.indices ? g.vertex_count : prims * 3, stride );
	const Index* idx = (const Index*)g.indices;
	if (q == TBVH_QUALITY_QUICK)
	{
		if constexpr (IsBVH<T>)
		{
			if (idx) return Fail( TBVH_ERROR_UNSUPPORTED, "tbvh_build: TBVH_QUALITY_QUICK does not take indices" );
			o.BuildQuick( slice );
			return TBVH_SUCCESS;
		}
		else return Fail( TBVH_ERROR_UNSUPPORTED, "tbvh_build: TBVH_QUALITY_QUICK is TBVH_LAYOUT_BVH only" );
	}
	if (q == TBVH_QUALITY_HIGH)
	{
		if constexpr (HasBuildHQ<T>::value)
		{
			o.BuildHQ( slice, idx, idx ? prims : 0 );
			return TBVH_SUCCESS;
		}
		else return Fail( TBVH_ERROR_UNSUPPORTED, "tbvh_build: this layout has no high quality builder" );
	}
	o.Build( slice, idx, idx ? prims : 0 );
	return TBVH_SUCCESS;
}

template <class G, class Real> tbvh_result CheckGeometry( const G* g, const char* fn )
{
	if (!g || !g->vertices) return Fail( TBVH_ERROR_INVALID_ARGUMENT, "%s: no geometry", fn );
	const uint64_t defStride = std::is_same_v<Real, float> ? 16 : 24, align = std::is_same_v<Real, float> ? 16 : 8;
	const uint64_t stride = g->vertex_stride ? g->vertex_stride : defStride;
	if ((uintptr_t)g->vertices % align)
		return Fail( TBVH_ERROR_INVALID_ARGUMENT, "%s: vertices must be %u-byte aligned", fn, (unsigned)align );
	if (stride < defStride || stride % align)
		return Fail( TBVH_ERROR_INVALID_ARGUMENT, "%s: vertex_stride must be a multiple of %u, at least %u", fn, (unsigned)align, (unsigned)defStride );
	if (g->indices)
	{
		if (g->prim_count == 0 || g->vertex_count == 0)
			return Fail( TBVH_ERROR_INVALID_ARGUMENT, "%s: indexed geometry needs prim_count and vertex_count", fn );
		for (uint64_t i = 0; i < (uint64_t)g->prim_count * 3; i++) if ((uint64_t)g->indices[i] >= (uint64_t)g->vertex_count)
			return Fail( TBVH_ERROR_INVALID_ARGUMENT, "%s: index %llu (%llu) out of range", fn, (unsigned long long)i, (unsigned long long)g->indices[i] );
	}
	else
	{
		if (g->vertex_count < 3) return Fail( TBVH_ERROR_INVALID_ARGUMENT, "%s: fewer than 3 vertices", fn );
		if ((uint64_t)g->prim_count * 3 > (uint64_t)g->vertex_count)
			return Fail( TBVH_ERROR_INVALID_ARGUMENT, "%s: prim_count exceeds vertex_count / 3", fn );
	}
	return TBVH_SUCCESS;
}

template <class T> bool Built( const T& o ) { return static_cast<const BaseOf<T>&>(o).triCount > 0; }

// Custom geometry trampolines: TinyBVH passes the handle as userdata.
bool CustomIntersect( Ray& ray, const uint32_t prim, void* user )
{
	const tbvh_bvh* h = (const tbvh_bvh*)user;
	return h->customIntersect( reinterpret_cast<tbvh_ray*>(&ray), prim, h->customUser ) != 0;
}
bool CustomOccluded( const Ray& ray, const uint32_t prim, void* user )
{
	const tbvh_bvh* h = (const tbvh_bvh*)user;
	return h->customOccluded( reinterpret_cast<const tbvh_ray*>(&ray), prim, h->customUser ) != 0;
}
void CustomAabb( const uint32_t prim, bvhvec3& bmin, bvhvec3& bmax, void* user )
{
	const tbvh_bvh* h = (const tbvh_bvh*)user;
	h->customAabb( prim, &bmin.x, &bmax.x, h->customUser );
}

} // namespace

// ============================================================================
// API: library
// ============================================================================

const char* tbvh_result_string( const tbvh_result r )
{
	switch (r)
	{
	case TBVH_SUCCESS: return "success";
	case TBVH_ERROR_INVALID_ARGUMENT: return "invalid argument";
	case TBVH_ERROR_INVALID_HANDLE: return "invalid handle";
	case TBVH_ERROR_UNSUPPORTED: return "not supported by this layout";
	case TBVH_ERROR_WRONG_PRECISION: return "wrong precision for this layout";
	case TBVH_ERROR_NOT_BUILT: return "not built";
	case TBVH_ERROR_NOT_REFITTABLE: return "not refittable";
	case TBVH_ERROR_OUT_OF_MEMORY: return "out of memory";
	case TBVH_ERROR_IO: return "file error";
	case TBVH_ERROR_FATAL: return "fatal error in TinyBVH";
	case TBVH_ERROR_INTERNAL: return "internal error";
	}
	return "unknown result";
}

const char* tbvh_get_last_error( void ) { return tl_lastError; }

void tbvh_set_error_callback( tbvh_error_fn fn, void* userdata ) { g_errorFn = fn, g_errorUser = userdata; }

int tbvh_layout_supported( const tbvh_layout layout )
{
	switch (layout)
	{
	case TBVH_LAYOUT_BVH: case TBVH_LAYOUT_BVH_GPU: case TBVH_LAYOUT_BVH4_CPU: case TBVH_LAYOUT_BVH4_GPU:
	case TBVH_LAYOUT_BVH8_CPU: case TBVH_LAYOUT_BVH8_CWBVH: case TBVH_LAYOUT_BVH4_AMD_HW:
	case TBVH_LAYOUT_CWBVH8_AMD_HW: case TBVH_LAYOUT_BVH_DOBB: return 1;
#ifdef DOUBLE_PRECISION_SUPPORT
	case TBVH_LAYOUT_BVH_DOUBLE: case TBVH_LAYOUT_BVH4_CPU_DOUBLE: return 1;
#endif
	default: return 0;
	}
}

void tbvh_get_config( tbvh_config* c )
{
	if (!c) return;
	memset( c, 0, sizeof( *c ) );
	c->api_version = TBVH_C_API_VERSION;
	c->version_major = TINY_BVH_VERSION_MAJOR, c->version_minor = TINY_BVH_VERSION_MINOR, c->version_sub = TINY_BVH_VERSION_SUB;
	c->cache_version = TINY_BVH_CACHE_VERSION;
#ifdef BVH_USESSE
	c->simd |= TBVH_SIMD_SSE;
#endif
#ifdef BVH_USEAVX
	c->simd |= TBVH_SIMD_AVX;
#endif
#ifdef BVH_USEAVX2
	c->simd |= TBVH_SIMD_AVX2;
#endif
#ifdef BVH_USENEON
	c->simd |= TBVH_SIMD_NEON;
#endif
#ifdef DOUBLE_PRECISION_SUPPORT
	c->features |= TBVH_FEATURE_DOUBLE;
#endif
#ifdef ENABLE_THREADED_BUILDS
	c->features |= TBVH_FEATURE_THREADED_BUILDS;
#ifndef TINYBVH_NO_BUILTIN_POOL
	c->features |= TBVH_FEATURE_BUILTIN_POOL;
#endif
#endif
#ifdef ENABLE_INDEXED_GEOMETRY
	c->features |= TBVH_FEATURE_INDEXED;
#endif
#ifdef ENABLE_CUSTOM_GEOMETRY
	c->features |= TBVH_FEATURE_CUSTOM_GEOMETRY;
#endif
#ifndef TBVH_NO_EXCEPTIONS
	c->features |= TBVH_FEATURE_RECOVERABLE_ERRORS;
#endif
	c->bvh_bins = BVHBINS, c->hq_bins_default = HQBVHBINS, c->hq_bins_max = MAXHQBINS;
	c->bundle_rays = TINYBVH_BUNDLE_RAYS, c->stack_size = TINYBVH_STACK_SIZE;
	c->sizeof_ray = sizeof( tbvh_ray ), c->sizeof_rayd = sizeof( tbvh_rayd );
	c->sizeof_instance = sizeof( tbvh_instance ), c->sizeof_instanced = sizeof( tbvh_instanced );
	c->sizeof_context = sizeof( tbvh_context ), c->sizeof_build_settings = sizeof( tbvh_build_settings );
}

void tbvh_shutdown_thread_pool( void )
{
#if defined ENABLE_THREADED_BUILDS && !defined TINYBVH_NO_BUILTIN_POOL
	tinybvh_shutdown_builtin_pool();
#endif
}

void tbvh_context_default( tbvh_context* context )
{
	if (!context) return;
	const BVHContext ctx{};
	memcpy( context, &ctx, sizeof( ctx ) );
}

void tbvh_build_settings_default( tbvh_build_settings* s )
{
	if (!s) return;
	const BVHBuildSettings d{};
	memset( s, 0, sizeof( *s ) );
	s->struct_size = sizeof( *s );
	s->flags = (d.usePresplitting ? TBVH_BUILD_PRESPLIT : 0) | (d.useSpatialSplits ? TBVH_BUILD_SPATIAL_SPLITS : 0) |
		(d.spatialSkipSmallestAxis ? TBVH_BUILD_SPATIAL_SKIP_SMALLEST : 0) | (d.presplitPostPass ? TBVH_BUILD_PRESPLIT_POST_PASS : 0) |
		(d.useFullSweep ? TBVH_BUILD_FULL_SWEEP : 0) | (d.binnedBVH ? TBVH_BUILD_BINNED : 0) | (d.useHPLOC ? TBVH_BUILD_HPLOC : 0) |
		(d.keepSplitting ? TBVH_BUILD_KEEP_SPLITTING : 0) | (d.postOptimize ? TBVH_BUILD_POST_OPTIMIZE : 0) |
		(d.useSIMDifavailable ? TBVH_BUILD_USE_SIMD : 0) | (d.enableThreading ? TBVH_BUILD_THREADED : 0) |
		(d.discardIntermediates ? TBVH_BUILD_DISCARD_INTERMEDIATES : 0) | (d.minimizeMemUse ? TBVH_BUILD_MINIMIZE_MEM_USE : 0);
	s->min_spatial_overlap = d.minSpatialOverlap, s->presplit_factor = d.presplitFactor;
	s->bonsai_grid_prims = d.bonsaiGridPrims, s->bonsai_prune_factor = d.bonsaiPruneFactor;
	s->bonsai_top_tree = d.bonsaiTopTree, s->optimize_iterations = (uint32_t)d.optimizeIterations;
	s->c_int = C_INT, s->c_trav = C_TRAV, s->hq_bins = HQBVHBINS, s->hq_leaf_prims = 0;
}

// ============================================================================
// API: handles
// ============================================================================

tbvh_result tbvh_create( const tbvh_layout layout, const tbvh_context* context, tbvh_bvh** out )
{
	if (!out) return Fail( TBVH_ERROR_INVALID_ARGUMENT, "tbvh_create: out is null" );
	*out = nullptr;
	if (!tbvh_layout_supported( layout )) return Fail( TBVH_ERROR_UNSUPPORTED, "tbvh_create: layout %d is not available", (int)layout );
	BVHContext ctx{};
	if (context)
	{
		memcpy( (void*)&ctx, context, sizeof( ctx ) );
		if (!ctx.malloc || !ctx.free) return Fail( TBVH_ERROR_INVALID_ARGUMENT, "tbvh_create: context needs alloc and release" );
	}
	TBVH_TRY
	void* mem = CtxAlloc( ctx, sizeof( tbvh_bvh ) );
	if (!mem) return Fail( TBVH_ERROR_OUT_OF_MEMORY, "tbvh_create: out of memory" );
	tbvh_bvh* h = new (mem) tbvh_bvh();
	h->layout = layout, h->ctx = ctx;
	switch (layout)
	{
	case TBVH_LAYOUT_BVH: h->obj = NewLayout<BVH>( ctx ); break;
	case TBVH_LAYOUT_BVH_GPU: h->obj = NewLayout<BVH_GPU>( ctx ); break;
	case TBVH_LAYOUT_BVH4_CPU: h->obj = NewLayout<BVH4_CPU>( ctx ); break;
	case TBVH_LAYOUT_BVH4_GPU: h->obj = NewLayout<BVH4_GPU>( ctx ); break;
	case TBVH_LAYOUT_BVH8_CPU: h->obj = NewLayout<BVH8_CPU>( ctx ); break;
	case TBVH_LAYOUT_BVH8_CWBVH: h->obj = NewLayout<BVH8_CWBVH>( ctx ); break;
	case TBVH_LAYOUT_BVH4_AMD_HW: h->obj = NewLayout<BVH4_AMD_HW>( ctx ); break;
	case TBVH_LAYOUT_CWBVH8_AMD_HW: h->obj = NewLayout<CWBVH8_AMD_HW>( ctx ); break;
	case TBVH_LAYOUT_BVH_DOBB: h->obj = NewLayout<BVH_DOBB>( ctx ); break;
#ifdef DOUBLE_PRECISION_SUPPORT
	case TBVH_LAYOUT_BVH_DOUBLE: h->obj = NewLayout<BVH_Double>( ctx ); break;
	case TBVH_LAYOUT_BVH4_CPU_DOUBLE: h->obj = NewLayout<BVH4_Double>( ctx ); break;
#endif
	default: break;
	}
	if (!h->obj) { CtxFree( ctx, h ); return Fail( TBVH_ERROR_OUT_OF_MEMORY, "tbvh_create: out of memory" ); }
	h->magic = tbvh_bvh::MAGIC;
	*out = h;
	return TBVH_SUCCESS;
	TBVH_CATCH( AS_RESULT )
}

void tbvh_destroy( tbvh_bvh* h )
{
	if (!Live( h )) return;
	if (!h->broken)
	{
		TBVH_TRY
		Visit( h, []( auto& o ) { using T = std::decay_t<decltype(o)>; o.~T(); } );
		TBVH_CATCH( (void) )
	}
	const BVHContext ctx = h->ctx;
	CtxFree( ctx, h->obj );
	CtxFree( ctx, h->blasList );
	h->magic = 0;
	CtxFree( ctx, h );
}

tbvh_layout tbvh_get_layout( const tbvh_bvh* h ) { return Valid( h ) ? h->layout : (tbvh_layout)0; }

tbvh_result tbvh_set_build_settings( tbvh_bvh* h, const tbvh_build_settings* s )
{
	if (!Valid( h )) return BadHandle( h, "tbvh_set_build_settings" );
	if (!s || s->struct_size < sizeof( tbvh_build_settings ))
		return Fail( TBVH_ERROR_INVALID_ARGUMENT, "tbvh_set_build_settings: settings missing or struct_size too small" );
	if (s->hq_bins < 2 || s->hq_bins > MAXHQBINS)
		return Fail( TBVH_ERROR_INVALID_ARGUMENT, "tbvh_set_build_settings: hq_bins must be in [2, %u]", (unsigned)MAXHQBINS );
	if (s->bonsai_top_tree > TBVH_BONSAI_TOP_CHEAPEST)
		return Fail( TBVH_ERROR_INVALID_ARGUMENT, "tbvh_set_build_settings: unknown bonsai_top_tree" );
	Visit( h, [&]( auto& o )
	{
		auto& b = static_cast<BaseOf<std::decay_t<decltype(o)>>&>(o);
		BVHBuildSettings& d = b.settings;
		const uint32_t f = s->flags;
		d.usePresplitting = f & TBVH_BUILD_PRESPLIT, d.useSpatialSplits = f & TBVH_BUILD_SPATIAL_SPLITS;
		d.spatialSkipSmallestAxis = f & TBVH_BUILD_SPATIAL_SKIP_SMALLEST, d.presplitPostPass = f & TBVH_BUILD_PRESPLIT_POST_PASS;
		d.useFullSweep = f & TBVH_BUILD_FULL_SWEEP, d.binnedBVH = f & TBVH_BUILD_BINNED, d.useHPLOC = f & TBVH_BUILD_HPLOC;
		d.keepSplitting = f & TBVH_BUILD_KEEP_SPLITTING, d.postOptimize = f & TBVH_BUILD_POST_OPTIMIZE;
		d.useSIMDifavailable = f & TBVH_BUILD_USE_SIMD, d.discardIntermediates = f & TBVH_BUILD_DISCARD_INTERMEDIATES;
		d.minimizeMemUse = f & TBVH_BUILD_MINIMIZE_MEM_USE;
	#ifdef ENABLE_THREADED_BUILDS
		d.enableThreading = f & TBVH_BUILD_THREADED;
	#endif
		d.minSpatialOverlap = s->min_spatial_overlap, d.presplitFactor = s->presplit_factor;
		d.bonsaiGridPrims = s->bonsai_grid_prims, d.bonsaiPruneFactor = s->bonsai_prune_factor;
		d.bonsaiTopTree = s->bonsai_top_tree, d.optimizeIterations = (int)s->optimize_iterations;
		b.c_int = s->c_int, b.c_trav = s->c_trav;
		// SBVH parameters are not passed down by PassSettingsTo: set them on the inner BVH too.
		auto apply = [&]( auto& t ) { t.hqbvhbins = s->hq_bins, t.hqbvhoddeven = (f & TBVH_BUILD_HQ_ODD_EVEN) != 0, t.hqLeafPrims = s->hq_leaf_prims; };
		apply( b );
		if (auto* inner = InnerBVH( o )) apply( *inner );
	} );
	return TBVH_SUCCESS;
}

tbvh_result tbvh_get_build_settings( const tbvh_bvh* h, tbvh_build_settings* s )
{
	if (!Valid( h )) return BadHandle( h, "tbvh_get_build_settings" );
	if (!s) return Fail( TBVH_ERROR_INVALID_ARGUMENT, "tbvh_get_build_settings: settings is null" );
	Visit( h, [&]( auto& o )
	{
		const auto& b = static_cast<const BaseOf<std::decay_t<decltype(o)>>&>(o);
		const BVHBuildSettings& d = b.settings;
		memset( s, 0, sizeof( *s ) );
		s->struct_size = sizeof( *s );
		s->flags = (d.usePresplitting ? TBVH_BUILD_PRESPLIT : 0) | (d.useSpatialSplits ? TBVH_BUILD_SPATIAL_SPLITS : 0) |
			(d.spatialSkipSmallestAxis ? TBVH_BUILD_SPATIAL_SKIP_SMALLEST : 0) | (d.presplitPostPass ? TBVH_BUILD_PRESPLIT_POST_PASS : 0) |
			(d.useFullSweep ? TBVH_BUILD_FULL_SWEEP : 0) | (d.binnedBVH ? TBVH_BUILD_BINNED : 0) | (d.useHPLOC ? TBVH_BUILD_HPLOC : 0) |
			(d.keepSplitting ? TBVH_BUILD_KEEP_SPLITTING : 0) | (d.postOptimize ? TBVH_BUILD_POST_OPTIMIZE : 0) |
			(d.useSIMDifavailable ? TBVH_BUILD_USE_SIMD : 0) | (d.enableThreading ? TBVH_BUILD_THREADED : 0) |
			(d.discardIntermediates ? TBVH_BUILD_DISCARD_INTERMEDIATES : 0) | (d.minimizeMemUse ? TBVH_BUILD_MINIMIZE_MEM_USE : 0) |
			(b.hqbvhoddeven ? TBVH_BUILD_HQ_ODD_EVEN : 0);
		s->min_spatial_overlap = d.minSpatialOverlap, s->presplit_factor = d.presplitFactor;
		s->bonsai_grid_prims = d.bonsaiGridPrims, s->bonsai_prune_factor = d.bonsaiPruneFactor;
		s->bonsai_top_tree = d.bonsaiTopTree, s->optimize_iterations = (uint32_t)d.optimizeIterations;
		s->c_int = b.c_int, s->c_trav = b.c_trav, s->hq_bins = b.hqbvhbins, s->hq_leaf_prims = b.hqLeafPrims;
	} );
	return TBVH_SUCCESS;
}

// ============================================================================
// API: building
// ============================================================================

tbvh_result tbvh_build( tbvh_bvh* h, const tbvh_geometry* g, const tbvh_quality q )
{
	if (!Valid( h )) return BadHandle( h, "tbvh_build" );
	if (IsDouble( h->layout )) return Fail( TBVH_ERROR_WRONG_PRECISION, "tbvh_build: double precision layout; use tbvh_buildd" );
	if (const tbvh_result r = CheckGeometry<tbvh_geometry, float>( g, "tbvh_build" )) return r;
	TBVH_TRY
	return Visit( h, [&]( auto& o ) -> tbvh_result
	{
		using T = std::decay_t<decltype(o)>;
		if constexpr (std::is_same_v<RealOf<T>, float>) return BuildTriangles( o, *g, q );
		else return TBVH_ERROR_WRONG_PRECISION;
	} );
	TBVH_CATCH_MUTATING( h )
}

tbvh_result tbvh_buildd( tbvh_bvh* h, const tbvh_geometryd* g, const tbvh_quality q )
{
	if (!Valid( h )) return BadHandle( h, "tbvh_buildd" );
	if (!IsDouble( h->layout )) return Fail( TBVH_ERROR_WRONG_PRECISION, "tbvh_buildd: single precision layout; use tbvh_build" );
	if (const tbvh_result r = CheckGeometry<tbvh_geometryd, double>( g, "tbvh_buildd" )) return r;
	TBVH_TRY
	return Visit( h, [&]( auto& o ) -> tbvh_result
	{
		using T = std::decay_t<decltype(o)>;
		if constexpr (std::is_same_v<RealOf<T>, double>) return BuildTriangles( o, *g, q );
		else return TBVH_ERROR_WRONG_PRECISION;
	} );
	TBVH_CATCH_MUTATING( h )
}

tbvh_result tbvh_set_custom_geometry( tbvh_bvh* h, tbvh_intersect_fn intersect, tbvh_occluded_fn occluded, void* userdata )
{
	if (!Valid( h )) return BadHandle( h, "tbvh_set_custom_geometry" );
	if (h->layout != TBVH_LAYOUT_BVH) return Fail( TBVH_ERROR_UNSUPPORTED, "tbvh_set_custom_geometry: TBVH_LAYOUT_BVH only" );
	if (!intersect != !occluded) return Fail( TBVH_ERROR_INVALID_ARGUMENT, "tbvh_set_custom_geometry: set both callbacks, or neither" );
	BVH& o = *(BVH*)h->obj;
	h->customIntersect = intersect, h->customOccluded = occluded, h->customUser = userdata;
	o.customIntersect = intersect ? CustomIntersect : nullptr;
	o.customIsOccluded = occluded ? CustomOccluded : nullptr;
	o.customUserdata = h;
	return TBVH_SUCCESS;
}

tbvh_result tbvh_build_custom( tbvh_bvh* h, tbvh_aabb_fn aabb_fn, const uint32_t prim_count )
{
	if (!Valid( h )) return BadHandle( h, "tbvh_build_custom" );
	if (h->layout != TBVH_LAYOUT_BVH) return Fail( TBVH_ERROR_UNSUPPORTED, "tbvh_build_custom: TBVH_LAYOUT_BVH only" );
	if (!aabb_fn || prim_count == 0) return Fail( TBVH_ERROR_INVALID_ARGUMENT, "tbvh_build_custom: need aabb_fn and prim_count" );
	TBVH_TRY
	BVH& o = *(BVH*)h->obj;
	h->customAabb = aabb_fn;
	o.customUserdata = h;
	o.Build( CustomAabb, prim_count );
	return TBVH_SUCCESS;
	TBVH_CATCH_MUTATING( h )
}

tbvh_result tbvh_build_aabbs( tbvh_bvh* h, const float* aabbs, const uint32_t prim_count )
{
	if (!Valid( h )) return BadHandle( h, "tbvh_build_aabbs" );
	if (h->layout != TBVH_LAYOUT_BVH) return Fail( TBVH_ERROR_UNSUPPORTED, "tbvh_build_aabbs: TBVH_LAYOUT_BVH only" );
	if (!aabbs || prim_count == 0) return Fail( TBVH_ERROR_INVALID_ARGUMENT, "tbvh_build_aabbs: need aabbs and prim_count" );
	if ((uintptr_t)aabbs & 15) return Fail( TBVH_ERROR_INVALID_ARGUMENT, "tbvh_build_aabbs: aabbs must be 16-byte aligned" );
	TBVH_TRY
	((BVH*)h->obj)->BuildAABB( (const bvhvec4*)aabbs, prim_count );
	return TBVH_SUCCESS;
	TBVH_CATCH_MUTATING( h )
}

namespace {
template <class TLAS, class Inst> tbvh_result BuildTLAS( tbvh_bvh* h, Inst* instances, const uint64_t instCount,
	tbvh_bvh* const* blases, const uint64_t blasCount, const char* fn )
{
	using Base = BaseOf<TLAS>;
	using Index = IndexOf<TLAS>;
	using BLASInst = impl::BLASInstance<RealOf<TLAS>, Index>;
	if (!instances || instCount == 0 || !blases || blasCount == 0)
		return Fail( TBVH_ERROR_INVALID_ARGUMENT, "%s: need instances and BLASes", fn );
	if ((uintptr_t)instances & 63) return Fail( TBVH_ERROR_INVALID_ARGUMENT, "%s: instances must be 64-byte aligned", fn );
	const bool dbl = IsDouble( h->layout );
	for (uint64_t i = 0; i < blasCount; i++)
	{
		const tbvh_bvh* b = blases[i];
		if (!Valid( b )) return Fail( TBVH_ERROR_INVALID_HANDLE, "%s: BLAS %llu is not a valid handle", fn, (unsigned long long)i );
		if (b == h) return Fail( TBVH_ERROR_INVALID_ARGUMENT, "%s: a TLAS cannot be its own BLAS", fn );
		const tbvh_layout l = b->layout;
		const bool ok = dbl ? (l == TBVH_LAYOUT_BVH_DOUBLE || l == TBVH_LAYOUT_BVH4_CPU_DOUBLE) :
			(l == TBVH_LAYOUT_BVH || l == TBVH_LAYOUT_BVH4_CPU || l == TBVH_LAYOUT_BVH8_CPU);
		if (!ok) return Fail( TBVH_ERROR_UNSUPPORTED, "%s: BLAS %llu has a layout that a TLAS cannot traverse", fn, (unsigned long long)i );
		if (!Visit( b, []( auto& o ) { return Built( o ); } )) return Fail( TBVH_ERROR_NOT_BUILT, "%s: BLAS %llu is not built", fn, (unsigned long long)i );
	}
	for (uint64_t i = 0; i < instCount; i++) if ((uint64_t)instances[i].blasIdx >= blasCount)
		return Fail( TBVH_ERROR_INVALID_ARGUMENT, "%s: instance %llu refers to BLAS %llu of %llu", fn,
			(unsigned long long)i, (unsigned long long)instances[i].blasIdx, (unsigned long long)blasCount );
	// TinyBVH keeps the BLAS pointer array; own a copy so the caller's array may go away.
	void** list = (void**)CtxAlloc( h->ctx, (size_t)blasCount * sizeof( void* ) );
	if (!list) return Fail( TBVH_ERROR_OUT_OF_MEMORY, "%s: out of memory", fn );
	for (uint64_t i = 0; i < blasCount; i++)
		list[i] = Visit( blases[i], []( auto& o ) -> void*
		{
			using U = std::decay_t<decltype(o)>;
			if constexpr (std::is_same_v<BaseOf<U>, Base>) return static_cast<Base*>(&o); else return nullptr;
		} );
	TLAS& o = *(TLAS*)h->obj;
	o.Build( reinterpret_cast<BLASInst*>(instances), (Index)instCount, (Base**)list, (Index)blasCount );
	CtxFree( h->ctx, h->blasList );
	h->blasList = list, h->blasCount = blasCount;
	return TBVH_SUCCESS;
}
} // namespace

tbvh_result tbvh_build_tlas( tbvh_bvh* h, tbvh_instance* instances, const uint32_t instCount, tbvh_bvh* const* blases, const uint32_t blasCount )
{
	if (!Valid( h )) return BadHandle( h, "tbvh_build_tlas" );
	if (h->layout != TBVH_LAYOUT_BVH) return Fail( TBVH_ERROR_UNSUPPORTED, "tbvh_build_tlas: the TLAS must be TBVH_LAYOUT_BVH" );
	TBVH_TRY
	return BuildTLAS<BVH>( h, instances, instCount, blases, blasCount, "tbvh_build_tlas" );
	TBVH_CATCH_MUTATING( h )
}

tbvh_result tbvh_build_tlasd( tbvh_bvh* h, tbvh_instanced* instances, const uint64_t instCount, tbvh_bvh* const* blases, const uint64_t blasCount )
{
	if (!Valid( h )) return BadHandle( h, "tbvh_build_tlasd" );
#ifdef DOUBLE_PRECISION_SUPPORT
	if (h->layout != TBVH_LAYOUT_BVH_DOUBLE) return Fail( TBVH_ERROR_UNSUPPORTED, "tbvh_build_tlasd: the TLAS must be TBVH_LAYOUT_BVH_DOUBLE" );
	TBVH_TRY
	return BuildTLAS<BVH_Double>( h, instances, instCount, blases, blasCount, "tbvh_build_tlasd" );
	TBVH_CATCH_MUTATING( h )
#else
	(void)instances, (void)instCount, (void)blases, (void)blasCount;
	return Fail( TBVH_ERROR_UNSUPPORTED, "tbvh_build_tlasd: built without double precision support" );
#endif
}

tbvh_result tbvh_refit( tbvh_bvh* h )
{
	if (!Valid( h )) return BadHandle( h, "tbvh_refit" );
	TBVH_TRY
	return Visit( h, [&]( auto& o ) -> tbvh_result
	{
		using T = std::decay_t<decltype(o)>;
		if constexpr (HasRefit<T>::value)
		{
			if (!Built( o )) return Fail( TBVH_ERROR_NOT_BUILT, "tbvh_refit: not built" );
			const auto* inner = InnerBVH( o );
			if (!static_cast<const BaseOf<T>&>(o).isRefittable())
				return Fail( TBVH_ERROR_NOT_REFITTABLE, "tbvh_refit: built with spatial splits or presplitting" );
			if (inner->isTLAS()) return Fail( TBVH_ERROR_UNSUPPORTED, "tbvh_refit: rebuild a TLAS instead" );
			if (!inner->verts || inner->bvhNode == 0)
				return Fail( TBVH_ERROR_NOT_REFITTABLE, "tbvh_refit: the intermediate BVH was discarded or never built over triangles" );
			o.Refit();
			return TBVH_SUCCESS;
		}
		else return Fail( TBVH_ERROR_UNSUPPORTED, "tbvh_refit: this layout cannot be refitted" );
	} );
	TBVH_CATCH_MUTATING( h )
}

tbvh_result tbvh_optimize( tbvh_bvh* h, uint32_t iterations )
{
	if (!Valid( h )) return BadHandle( h, "tbvh_optimize" );
	TBVH_TRY
	return Visit( h, [&]( auto& o ) -> tbvh_result
	{
		using T = std::decay_t<decltype(o)>;
		if constexpr (HasOptimize<T>::value)
		{
			if (!Built( o )) return Fail( TBVH_ERROR_NOT_BUILT, "tbvh_optimize: not built" );
			if (InnerBVH( o )->isTLAS()) return Fail( TBVH_ERROR_UNSUPPORTED, "tbvh_optimize: not for a TLAS" );
			if (iterations == 0) iterations = (uint32_t)static_cast<BaseOf<T>&>(o).settings.optimizeIterations;
			o.Optimize( iterations, 8u );
			return TBVH_SUCCESS;
		}
		else return Fail( TBVH_ERROR_UNSUPPORTED, "tbvh_optimize: this layout cannot be optimized" );
	} );
	TBVH_CATCH_MUTATING( h )
}

// ============================================================================
// API: traversal
// ============================================================================

namespace {
template <class T> tbvh_result CheckTraversable( const T& o )
{
	if (!Built( o )) return Fail( TBVH_ERROR_NOT_BUILT, "traversal: not built" );
	return TBVH_SUCCESS;
}
template <class Real, class CRay, bool OCCLUSION> int32_t Trace( const tbvh_bvh* h, CRay* ray, const char* fn )
{
	if (!Valid( h )) return AS_INT( BadHandle( h, fn ) );
	if (!ray) return AS_INT( Fail( TBVH_ERROR_INVALID_ARGUMENT, "%s: ray is null", fn ) );
	TBVH_TRY
	return Visit( h, [&]( auto& o ) -> int32_t
	{
		using T = std::decay_t<decltype(o)>;
		if constexpr (!std::is_same_v<RealOf<T>, Real>) return AS_INT( Fail( TBVH_ERROR_WRONG_PRECISION, "%s: wrong precision", fn ) );
		else if constexpr (!HasIntersect<T>::value) return AS_INT( Fail( TBVH_ERROR_UNSUPPORTED, "%s: no CPU traversal for this layout", fn ) );
		else
		{
			if (const tbvh_result r = CheckTraversable( o )) return AS_INT( r );
			using R = RayOf<T>;
			if constexpr (OCCLUSION) return o.IsOccluded( *reinterpret_cast<const R*>(ray) ) ? 1 : 0;
			else return o.Intersect( *reinterpret_cast<R*>(ray) );
		}
	} );
	TBVH_CATCH( AS_INT )
}
} // namespace

int32_t tbvh_intersect( const tbvh_bvh* h, tbvh_ray* ray ) { return Trace<float, tbvh_ray, false>( h, ray, "tbvh_intersect" ); }
int32_t tbvh_is_occluded( const tbvh_bvh* h, const tbvh_ray* ray ) { return Trace<float, const tbvh_ray, true>( h, ray, "tbvh_is_occluded" ); }
int32_t tbvh_intersectd( const tbvh_bvh* h, tbvh_rayd* ray ) { return Trace<double, tbvh_rayd, false>( h, ray, "tbvh_intersectd" ); }
int32_t tbvh_is_occludedd( const tbvh_bvh* h, const tbvh_rayd* ray ) { return Trace<double, const tbvh_rayd, true>( h, ray, "tbvh_is_occludedd" ); }

namespace {
int32_t Bundle( const tbvh_bvh* h, tbvh_ray* rays, uint8_t* occluded, const char* fn )
{
	if (!Valid( h )) return AS_INT( BadHandle( h, fn ) );
	if (!rays || ((uintptr_t)rays & 63)) return AS_INT( Fail( TBVH_ERROR_INVALID_ARGUMENT, "%s: rays missing or not 64-byte aligned", fn ) );
	TBVH_TRY
	return Visit( h, [&]( auto& o ) -> int32_t
	{
		using T = std::decay_t<decltype(o)>;
		if constexpr (!std::is_same_v<RealOf<T>, float> || !HasBundle<T>::value)
			return AS_INT( Fail( TBVH_ERROR_UNSUPPORTED, "%s: BVH4_CPU or a TLAS only", fn ) );
		else
		{
			if (const tbvh_result r = CheckTraversable( o )) return AS_INT( r );
			if constexpr (IsBVH<T>) if (!o.isTLAS()) return AS_INT( Fail( TBVH_ERROR_UNSUPPORTED, "%s: a BVH must be a TLAS", fn ) );
			if (occluded) return o.IsOccludedBundle( reinterpret_cast<Ray*>(rays), reinterpret_cast<bool*>(occluded) );
			return o.IntersectBundle( reinterpret_cast<Ray*>(rays) );
		}
	} );
	TBVH_CATCH( AS_INT )
}
} // namespace

int32_t tbvh_intersect_bundle( const tbvh_bvh* h, tbvh_ray* rays ) { return Bundle( h, rays, nullptr, "tbvh_intersect_bundle" ); }
int32_t tbvh_is_occluded_bundle( const tbvh_bvh* h, tbvh_ray* rays, uint8_t* occluded )
{
	if (!occluded) return AS_INT( Fail( TBVH_ERROR_INVALID_ARGUMENT, "tbvh_is_occluded_bundle: occluded is null" ) );
	return Bundle( h, rays, occluded, "tbvh_is_occluded_bundle" );
}

// ============================================================================
// API: queries
// ============================================================================

tbvh_result tbvh_get_info( const tbvh_bvh* h, tbvh_info* info )
{
	if (!Valid( h )) return BadHandle( h, "tbvh_get_info" );
	if (!info) return Fail( TBVH_ERROR_INVALID_ARGUMENT, "tbvh_get_info: info is null" );
	Visit( h, [&]( auto& o )
	{
		using T = std::decay_t<decltype(o)>;
		const auto& b = static_cast<const BaseOf<T>&>(o);
		const auto* inner = InnerBVH( o );
		memset( info, 0, sizeof( *info ) );
		info->layout = (uint32_t)h->layout;
		info->flags = (b.triCount > 0 ? TBVH_INFO_BUILT : 0) | (b.isRefittable() ? TBVH_INFO_REFITTABLE : 0) |
			(IsDouble( h->layout ) ? TBVH_INFO_DOUBLE : 0);
		if (inner) info->flags |= (inner->isTLAS() ? TBVH_INFO_TLAS : 0) | (inner->isIndexed() ? TBVH_INFO_INDEXED : 0) |
			(inner->hasCustomGeom() ? TBVH_INFO_CUSTOM : 0);
		info->prim_count = b.triCount, info->index_count = b.idxCount, info->node_count = NodeCount( o );
		for (int i = 0; i < 3; i++) info->aabb_min[i] = b.aabbMin[i], info->aabb_max[i] = b.aabbMax[i];
	} );
	return TBVH_SUCCESS;
}

tbvh_result tbvh_get_sah_cost( const tbvh_bvh* h, double* cost )
{
	if (!Valid( h )) return BadHandle( h, "tbvh_get_sah_cost" );
	if (!cost) return Fail( TBVH_ERROR_INVALID_ARGUMENT, "tbvh_get_sah_cost: cost is null" );
	TBVH_TRY
	return Visit( h, [&]( auto& o ) -> tbvh_result
	{
		using T = std::decay_t<decltype(o)>;
		if constexpr (HasSAH<T>::value)
		{
			if (!Built( o )) return Fail( TBVH_ERROR_NOT_BUILT, "tbvh_get_sah_cost: not built" );
			*cost = (double)o.SAHCost( IndexOf<T>( 0 ) );
			return TBVH_SUCCESS;
		}
		else return Fail( TBVH_ERROR_UNSUPPORTED, "tbvh_get_sah_cost: not available for this layout" );
	} );
	TBVH_CATCH( AS_RESULT )
}

tbvh_result tbvh_get_buffer( const tbvh_bvh* h, const tbvh_buffer which, const void** data, size_t* size )
{
	if (!Valid( h )) return BadHandle( h, "tbvh_get_buffer" );
	if (!data || !size) return Fail( TBVH_ERROR_INVALID_ARGUMENT, "tbvh_get_buffer: data and size are required" );
	*data = nullptr, *size = 0;
	const void* p = nullptr;
	size_t n = 0;
	auto pick = [&]( const tbvh_buffer b, const void* ptr, const size_t bytes ) { if (which == b) p = ptr, n = bytes; };
	switch (h->layout)
	{
	case TBVH_LAYOUT_BVH: { const BVH& o = *(const BVH*)h->obj;
		pick( TBVH_BUFFER_NODES, o.bvhNode, (size_t)o.usedNodes * sizeof( BVH::BVHNode ) );
		pick( TBVH_BUFFER_PRIM_INDICES, o.primIdx, (size_t)o.idxCount * sizeof( uint32_t ) ); break; }
	case TBVH_LAYOUT_BVH_GPU: { const BVH_GPU& o = *(const BVH_GPU*)h->obj;
		pick( TBVH_BUFFER_NODES, o.bvhNode, (size_t)o.usedNodes * sizeof( BVH_GPU::BVHNode ) );
		pick( TBVH_BUFFER_TRIANGLES, o.orderedVerts.data, (size_t)o.orderedVerts.count * o.orderedVerts.stride );
		pick( TBVH_BUFFER_PRIM_INDICES, o.bvh.primIdx, o.bvh.primIdx ? (size_t)o.bvh.idxCount * sizeof( uint32_t ) : 0 ); break; }
	case TBVH_LAYOUT_BVH4_CPU: { const BVH4_CPU& o = *(const BVH4_CPU*)h->obj;
		pick( TBVH_BUFFER_NODES, o.bvh4Data, (size_t)o.usedBlocks * 64 ); break; }
	case TBVH_LAYOUT_BVH8_CPU: { const BVH8_CPU& o = *(const BVH8_CPU*)h->obj;
		pick( TBVH_BUFFER_NODES, o.bvh8Data, (size_t)o.usedBlocks * 64 ); break; }
	case TBVH_LAYOUT_BVH4_GPU: { const BVH4_GPU& o = *(const BVH4_GPU*)h->obj;
		pick( TBVH_BUFFER_NODES, o.bvh4Data, (size_t)o.usedBlocks * 16 ); break; }
	case TBVH_LAYOUT_BVH8_CWBVH: { const BVH8_CWBVH& o = *(const BVH8_CWBVH*)h->obj;
		pick( TBVH_BUFFER_NODES, o.bvh8Data, (size_t)o.usedBlocks * 16 );
		pick( TBVH_BUFFER_TRIANGLES, o.bvh8Tris, (size_t)o.usedTriBlocks * 16 ); break; }
	case TBVH_LAYOUT_BVH4_AMD_HW: { const BVH4_AMD_HW& o = *(const BVH4_AMD_HW*)h->obj;
		pick( TBVH_BUFFER_NODES, o.bvh4Data, (size_t)o.usedBlocks * 16 ); break; }
	case TBVH_LAYOUT_CWBVH8_AMD_HW: { const CWBVH8_AMD_HW& o = *(const CWBVH8_AMD_HW*)h->obj;
		pick( TBVH_BUFFER_NODES, o.bvh8Data, (size_t)o.usedBlocks * 16 ); break; }
	case TBVH_LAYOUT_BVH_DOBB: { const BVH_DOBB& o = *(const BVH_DOBB*)h->obj;
		pick( TBVH_BUFFER_NODES, o.dobbNode, (size_t)o.usedNodes * sizeof( BVH_DOBB::BVHNode ) );
		pick( TBVH_BUFFER_PRIM_INDICES, o.dobbPrimIdx, (size_t)o.idxCount * sizeof( uint32_t ) ); break; }
#ifdef DOUBLE_PRECISION_SUPPORT
	case TBVH_LAYOUT_BVH_DOUBLE: { const BVH_Double& o = *(const BVH_Double*)h->obj;
		pick( TBVH_BUFFER_NODES, o.bvhNode, (size_t)o.usedNodes * sizeof( BVH_Double::BVHNode ) );
		pick( TBVH_BUFFER_PRIM_INDICES, o.primIdx, (size_t)o.idxCount * sizeof( uint64_t ) ); break; }
	case TBVH_LAYOUT_BVH4_CPU_DOUBLE: { const BVH4_Double& o = *(const BVH4_Double*)h->obj;
		pick( TBVH_BUFFER_NODES, o.bvh4Data, (size_t)o.usedBlocks * 64 ); break; }
#endif
	default: break;
	}
	if (!p || !n) return Fail( TBVH_ERROR_UNSUPPORTED, "tbvh_get_buffer: layout has no such buffer, or it is empty" );
	*data = p, *size = n;
	return TBVH_SUCCESS;
}

// ============================================================================
// API: serialization
// ============================================================================

tbvh_result tbvh_save( const tbvh_bvh* h, const char* fileName )
{
	if (!Valid( h )) return BadHandle( h, "tbvh_save" );
	if (!fileName) return Fail( TBVH_ERROR_INVALID_ARGUMENT, "tbvh_save: no file name" );
	TBVH_TRY
	return Visit( h, [&]( auto& o ) -> tbvh_result
	{
		using T = std::decay_t<decltype(o)>;
		if constexpr (HasSave<T>::value)
		{
			if (!Built( o )) return Fail( TBVH_ERROR_NOT_BUILT, "tbvh_save: not built" );
			if (InnerBVH( o )->isTLAS() || InnerBVH( o )->hasCustomGeom())
				return Fail( TBVH_ERROR_UNSUPPORTED, "tbvh_save: a TLAS or custom geometry cannot be saved" );
			if (!o.Save( fileName )) return Fail( TBVH_ERROR_IO, "tbvh_save: cannot write '%s'", fileName );
			return TBVH_SUCCESS;
		}
		else return Fail( TBVH_ERROR_UNSUPPORTED, "tbvh_save: this layout cannot be saved" );
	} );
	TBVH_CATCH( AS_RESULT )
}

namespace {
template <class G> tbvh_result Load( tbvh_bvh* h, const char* fileName, const G* g, const char* fn )
{
	if (!fileName) return Fail( TBVH_ERROR_INVALID_ARGUMENT, "%s: no file name", fn );
	return Visit( h, [&]( auto& o ) -> tbvh_result
	{
		using T = std::decay_t<decltype(o)>;
		using Index = IndexOf<T>;
		if constexpr (!std::is_same_v<GeometryOf<T>, G>) return Fail( TBVH_ERROR_WRONG_PRECISION, "%s: wrong precision", fn );
		else if constexpr (IsBVH<T>)
		{
			using Vertex = typename bvh_traits<RealOf<T>>::vertex;
			bool ok;
			if (!g) ok = o.Load( fileName, SliceOf<T>( nullptr, 0, 0 ), nullptr, 0 );
			else
			{
				if (const tbvh_result r = CheckGeometry<G, RealOf<T>>( g, fn )) return r;
				const auto stride = g->vertex_stride ? g->vertex_stride : sizeof( Vertex );
				const Index prims = g->indices ? (Index)g->prim_count : (Index)(g->prim_count ? g->prim_count : g->vertex_count / 3);
				const SliceOf<T> slice( (const Vertex*)g->vertices, g->indices ? g->vertex_count : prims * 3, stride );
				ok = o.Load( fileName, slice, (const Index*)g->indices, g->indices ? prims : 0 );
			}
			if (!ok) return Fail( TBVH_ERROR_IO, "%s: '%s' is missing, or does not match this layout, version or geometry", fn, fileName );
			return TBVH_SUCCESS;
		}
		else if constexpr (HasStaticLoad<T>::value)
		{
			// The static Load reads this layout's file, or converts a BVH file saved with its
			// geometry. Load into a temporary, so that a failed load leaves the handle's tree as is.
			T loaded = T::Load( fileName, h->ctx );
			if (!Built( loaded ))
				return Fail( TBVH_ERROR_IO, "%s: '%s' is missing, or is not a file of this layout or a BVH file with geometry, for this version", fn, fileName );
			if (g)
			{
				const Index prims = (Index)(g->prim_count ? g->prim_count : g->vertex_count / 3);
				if (static_cast<const BaseOf<T>&>(loaded).triCount != prims)
					return Fail( TBVH_ERROR_IO, "%s: '%s' holds %llu primitives, the geometry %llu", fn, fileName,
						(unsigned long long)static_cast<const BaseOf<T>&>(loaded).triCount, (unsigned long long)prims );
			}
			o = std::move( loaded );
			return TBVH_SUCCESS;
		}
		else return Fail( TBVH_ERROR_UNSUPPORTED, "%s: this layout cannot be loaded", fn );
	} );
}
} // namespace

tbvh_result tbvh_load( tbvh_bvh* h, const char* fileName, const tbvh_geometry* g )
{
	if (!Valid( h )) return BadHandle( h, "tbvh_load" );
	TBVH_TRY
	return Load( h, fileName, g, "tbvh_load" );
	TBVH_CATCH_MUTATING( h )
}

tbvh_result tbvh_loadd( tbvh_bvh* h, const char* fileName, const tbvh_geometryd* g )
{
	if (!Valid( h )) return BadHandle( h, "tbvh_loadd" );
	TBVH_TRY
	return Load( h, fileName, g, "tbvh_loadd" );
	TBVH_CATCH_MUTATING( h )
}
