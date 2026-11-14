/* test_tiny_bvh_c.c - exercises tiny_bvh_c.h from plain C.
   Every traversal result is checked against brute-force ray / triangle tests. */

#include "tiny_bvh_c.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

static int failures = 0, checks = 0;
#define CHECK( c, ... ) do { checks++; if (!(c)) { failures++; printf( "FAIL %s:%d: ", __FILE__, __LINE__ ); printf( __VA_ARGS__ ); printf( "\n" ); } } while (0)
#define OK( call ) do { const tbvh_result r_ = (call); CHECK( r_ == TBVH_SUCCESS, "%s -> %s (%s)", #call, tbvh_result_string( r_ ), tbvh_get_last_error() ); } while (0)

/* deterministic random numbers */
static uint32_t seed = 0x12345678;
static float rnd( void ) { seed ^= seed << 13, seed ^= seed >> 17, seed ^= seed << 5; return (seed & 0xffffff) / 16777216.0f; }

/* --- geometry -------------------------------------------------------------- */

#define TRIS 6000
#define RAYS 3000
static float* soup;				/* TRIS * 3 vertices, 4 floats each (16-byte aligned) */

static void make_soup( void )
{
	soup = (float*)aligned_alloc( 64, TRIS * 3 * 4 * sizeof( float ) );
	for (int i = 0; i < TRIS; i++)
	{
		const float cx = rnd() * 10 - 5, cy = rnd() * 10 - 5, cz = rnd() * 10 - 5;
		for (int v = 0; v < 3; v++)
		{
			float* p = soup + (i * 3 + v) * 4;
			p[0] = cx + rnd() * 0.6f - 0.3f, p[1] = cy + rnd() * 0.6f - 0.3f, p[2] = cz + rnd() * 0.6f - 0.3f, p[3] = 0;
		}
	}
}

/* Indexed version of the same triangles: vertices shuffled, indices pointing back. */
static float* ivert;
static uint32_t* iidx;
static void make_indexed( void )
{
	ivert = (float*)aligned_alloc( 64, TRIS * 3 * 4 * sizeof( float ) );
	iidx = (uint32_t*)malloc( TRIS * 3 * sizeof( uint32_t ) );
	uint32_t* perm = (uint32_t*)malloc( TRIS * 3 * sizeof( uint32_t ) );
	for (uint32_t i = 0; i < TRIS * 3; i++) perm[i] = i;
	for (uint32_t i = TRIS * 3 - 1; i > 0; i--) { uint32_t j = (uint32_t)(rnd() * (i + 1)) % (i + 1), t = perm[i]; perm[i] = perm[j], perm[j] = t; }
	for (uint32_t i = 0; i < TRIS * 3; i++) memcpy( ivert + perm[i] * 4, soup + i * 4, 16 ), iidx[i] = perm[i];
	free( perm );
}

/* --- brute force oracle ---------------------------------------------------- */

static float tri_hit( const float* o, const float* d, const float* a, const float* b, const float* c )
{
	const float e1[3] = { b[0] - a[0], b[1] - a[1], b[2] - a[2] }, e2[3] = { c[0] - a[0], c[1] - a[1], c[2] - a[2] };
	const float h[3] = { d[1] * e2[2] - d[2] * e2[1], d[2] * e2[0] - d[0] * e2[2], d[0] * e2[1] - d[1] * e2[0] };
	const float det = e1[0] * h[0] + e1[1] * h[1] + e1[2] * h[2];
	if (fabsf( det ) < 1e-12f) return 1e30f;
	const float f = 1 / det, s[3] = { o[0] - a[0], o[1] - a[1], o[2] - a[2] };
	const float u = f * (s[0] * h[0] + s[1] * h[1] + s[2] * h[2]);
	if (u < 0 || u > 1) return 1e30f;
	const float q[3] = { s[1] * e1[2] - s[2] * e1[1], s[2] * e1[0] - s[0] * e1[2], s[0] * e1[1] - s[1] * e1[0] };
	const float v = f * (d[0] * q[0] + d[1] * q[1] + d[2] * q[2]);
	if (v < 0 || u + v > 1) return 1e30f;
	const float t = f * (e2[0] * q[0] + e2[1] * q[1] + e2[2] * q[2]);
	return t > 0 ? t : 1e30f;
}

static tbvh_ray* rays;			/* the test rays */
static float* ref_t;			/* brute force distance per ray */
static uint32_t* ref_prim;

static void make_rays( void )
{
	rays = (tbvh_ray*)aligned_alloc( 64, RAYS * sizeof( tbvh_ray ) );
	ref_t = (float*)malloc( RAYS * sizeof( float ) );
	ref_prim = (uint32_t*)malloc( RAYS * sizeof( uint32_t ) );
	for (int i = 0; i < RAYS; i++)
	{
		float o[3] = { rnd() * 14 - 7, rnd() * 14 - 7, rnd() * 14 - 7 }, d[3] = { rnd() - 0.5f, rnd() - 0.5f, rnd() - 0.5f };
		if (i % 97 == 0) d[i % 3] = 0; /* axis-parallel components exercise the safe reciprocal */
		tbvh_ray_init( rays + i, o[0], o[1], o[2], d[0], d[1], d[2], TBVH_FAR );
		ref_t[i] = 1e30f, ref_prim[i] = ~0u;
		for (int t = 0; t < TRIS; t++)
		{
			const float* v = soup + t * 12;
			const float dist = tri_hit( o, d, v, v + 4, v + 8 );
			if (dist < ref_t[i]) ref_t[i] = dist, ref_prim[i] = (uint32_t)t;
		}
	}
}

static int same_t( const float a, const float b ) { return (a >= 1e29f && b >= 1e29f) || fabsf( a - b ) <= 1e-3f * fmaxf( 1, fabsf( b ) ); }

/* Traces all rays; returns the number of rays that disagree with the oracle. */
static int check_against_oracle( const tbvh_bvh* bvh, const char* label )
{
	int bad = 0, errors = 0, occBad = 0;
	for (int i = 0; i < RAYS; i++)
	{
		tbvh_ray r = rays[i];
		const int32_t cost = tbvh_intersect( bvh, &r );
		if (cost < 0) { errors++; continue; }
		if (!same_t( r.hit.t, ref_t[i] )) bad++;
		else if (ref_t[i] < 1e29f && r.hit.prim != ref_prim[i])
		{
			/* a different primitive at the same distance is fine (shared edge, overlap) */
			const float* v = soup + r.hit.prim * 12;
			if (!same_t( tri_hit( rays[i].O, rays[i].D, v, v + 4, v + 8 ), ref_t[i] )) bad++;
		}
		tbvh_ray s = rays[i];
		const int32_t occ = tbvh_is_occluded( bvh, &s );
		if (occ < 0) errors++;
		else if (occ != (ref_t[i] < 1e29f)) occBad++;
	}
	CHECK( errors == 0, "%s: %d traversal errors (%s)", label, errors, tbvh_get_last_error() );
	CHECK( bad <= RAYS / 1000, "%s: %d of %d closest hits disagree with brute force", label, bad, RAYS );
	CHECK( occBad <= RAYS / 1000, "%s: %d of %d occlusion results disagree", label, occBad, RAYS );
	return bad;
}

/* --- counting allocator ---------------------------------------------------- */

static long live_allocs = 0, total_allocs = 0;
static void* count_alloc( size_t size, void* user )
{
	(void)user;
	live_allocs++, total_allocs++;
	return aligned_alloc( 64, (size + 63) & ~(size_t)63 );
}
static void count_free( void* p, void* user ) { (void)user; if (p) live_allocs--; free( p ); }

/* --- arena allocator that fails on request ---------------------------------- */
/* Blocks a failed build leaves behind are released by arena_reset, so that leak
   checkers stay quiet about the deliberate failures. */

#define ARENA_MAX 8192
static void* arena[ARENA_MAX];
static int arena_used = 0;
static long arena_fail_in = -1;			/* allocations until one fails; -1: never */
static void* arena_alloc( size_t size, void* user )
{
	(void)user;
	if (arena_fail_in == 0) { arena_fail_in = -1; return NULL; }
	if (arena_fail_in > 0) arena_fail_in--;
	if (arena_used == ARENA_MAX) return NULL;
	return arena[arena_used++] = aligned_alloc( 64, (size + 63) & ~(size_t)63 );
}
static void arena_release( void* p, void* user )
{
	(void)user;
	if (!p) return;
	for (int i = 0; i < arena_used; i++) if (arena[i] == p) { free( p ); arena[i] = arena[--arena_used]; return; }
	printf( "FAIL: arena_release of unknown block %p (double free?)\n" , p ), failures++;
}
static int arena_reset( void ) { const int n = arena_used; while (arena_used) free( arena[--arena_used] ); return n; }

/* --- custom geometry: spheres ---------------------------------------------- */

typedef struct sphere { float c[3], r; } sphere;
static sphere spheres[64];

static float sphere_hit( const sphere* s, const float* O, const float* D )
{
	const float oc[3] = { O[0] - s->c[0], O[1] - s->c[1], O[2] - s->c[2] };
	const float a = D[0] * D[0] + D[1] * D[1] + D[2] * D[2];
	const float b = oc[0] * D[0] + oc[1] * D[1] + oc[2] * D[2];
	const float c = oc[0] * oc[0] + oc[1] * oc[1] + oc[2] * oc[2] - s->r * s->r;
	const float disc = b * b - a * c;
	if (disc < 0) return 1e30f;
	const float t = (-b - sqrtf( disc )) / a;
	return t > 0 ? t : 1e30f;
}
static int sphere_intersect( tbvh_ray* ray, uint32_t prim, void* user )
{
	const sphere* s = (const sphere*)user + prim;
	const float t = sphere_hit( s, ray->O, ray->D );
	if (t >= ray->hit.t) return 0;
	ray->hit.t = t, ray->hit.u = ray->hit.v = 0, ray->hit.prim = prim;
	return 1;
}
static int sphere_occluded( const tbvh_ray* ray, uint32_t prim, void* user )
{
	return sphere_hit( (const sphere*)user + prim, ray->O, ray->D ) < ray->hit.t;
}
static void sphere_aabb( uint32_t prim, float bmin[3], float bmax[3], void* user )
{
	const sphere* s = (const sphere*)user + prim;
	for (int a = 0; a < 3; a++) bmin[a] = s->c[a] - s->r, bmax[a] = s->c[a] + s->r;
}

static int error_callback_calls = 0;
static void on_error( tbvh_result r, const char* msg, void* user ) { (void)r, (void)msg, (void)user; error_callback_calls++; }

/* --- tests ----------------------------------------------------------------- */

static const tbvh_layout traversable[] = { TBVH_LAYOUT_BVH, TBVH_LAYOUT_BVH_GPU, TBVH_LAYOUT_BVH4_CPU, TBVH_LAYOUT_BVH4_GPU,
	TBVH_LAYOUT_BVH8_CPU, TBVH_LAYOUT_BVH8_CWBVH, TBVH_LAYOUT_BVH_DOBB };
static const char* names[] = { "BVH", "BVH_GPU", "BVH4_CPU", "BVH4_GPU", "BVH8_CPU", "BVH8_CWBVH", "BVH_DOBB" };

static void test_layouts( void )
{
	const tbvh_geometry soupGeom = { soup, TRIS * 3, 0, NULL, 0 };
	const tbvh_geometry indexedGeom = { ivert, TRIS * 3, 16, iidx, TRIS };
	for (int l = 0; l < (int)(sizeof( traversable ) / sizeof( traversable[0] )); l++)
	{
		for (int pass = 0; pass < 3; pass++)
		{
			char label[64];
			const tbvh_quality q = pass == 1 ? TBVH_QUALITY_HIGH : TBVH_QUALITY_DEFAULT;
			snprintf( label, sizeof( label ), "%s %s", names[l], pass == 0 ? "default" : pass == 1 ? "HQ" : "indexed" );
			tbvh_bvh* bvh = NULL;
			OK( tbvh_create( traversable[l], NULL, &bvh ) );
			const tbvh_result r = tbvh_build( bvh, pass == 2 ? &indexedGeom : &soupGeom, q );
			if (traversable[l] == TBVH_LAYOUT_BVH_DOBB && pass == 1)
				CHECK( r == TBVH_ERROR_UNSUPPORTED, "%s: expected unsupported, got %s", label, tbvh_result_string( r ) );
			else
			{
				CHECK( r == TBVH_SUCCESS, "%s build: %s (%s)", label, tbvh_result_string( r ), tbvh_get_last_error() );
				if (r == TBVH_SUCCESS) check_against_oracle( bvh, label );
				tbvh_info info;
				OK( tbvh_get_info( bvh, &info ) );
				CHECK( info.prim_count == TRIS && (info.flags & TBVH_INFO_BUILT), "%s: info prim_count %llu", label, (unsigned long long)info.prim_count );
				CHECK( !!(info.flags & TBVH_INFO_INDEXED) == (pass == 2), "%s: indexed flag", label );
				CHECK( info.node_count > 0, "%s: node_count is 0", label );
				const void* data; size_t size;
				OK( tbvh_get_buffer( bvh, TBVH_BUFFER_NODES, &data, &size ) );
				CHECK( data && size > 0, "%s: node buffer", label );
			}
			tbvh_destroy( bvh );
		}
	}
	/* hardware layouts: buffers, no CPU traversal */
	const tbvh_layout hw[] = { TBVH_LAYOUT_BVH4_AMD_HW, TBVH_LAYOUT_CWBVH8_AMD_HW };
	for (int l = 0; l < 2; l++)
	{
		tbvh_bvh* bvh;
		OK( tbvh_create( hw[l], NULL, &bvh ) );
		OK( tbvh_build( bvh, &soupGeom, TBVH_QUALITY_DEFAULT ) );
		const void* data; size_t size;
		OK( tbvh_get_buffer( bvh, TBVH_BUFFER_NODES, &data, &size ) );
		CHECK( size >= TRIS * 64 / 2, "AMD layout %d: node buffer of %zu bytes", (int)hw[l], size );
		tbvh_ray r = rays[0];
		CHECK( tbvh_intersect( bvh, &r ) == -TBVH_ERROR_UNSUPPORTED, "AMD layout: intersect should be unsupported" );
		tbvh_destroy( bvh );
	}
}

static void test_refit( void )
{
	const size_t bytes = TRIS * 3 * 16;
	float* verts = (float*)aligned_alloc( 64, bytes );
	memcpy( verts, soup, bytes );
	const tbvh_geometry g = { verts, TRIS * 3, 0, NULL, 0 };
	const tbvh_layout ls[] = { TBVH_LAYOUT_BVH, TBVH_LAYOUT_BVH4_CPU, TBVH_LAYOUT_BVH8_CPU };
	for (int l = 0; l < 3; l++)
	{
		memcpy( verts, soup, bytes );
		tbvh_bvh* bvh;
		OK( tbvh_create( ls[l], NULL, &bvh ) );
		OK( tbvh_build( bvh, &g, TBVH_QUALITY_DEFAULT ) );
		/* move everything by +1 in x, refit, and trace shifted rays: must match the oracle */
		for (int i = 0; i < TRIS * 3; i++) verts[i * 4] += 1;
		OK( tbvh_refit( bvh ) );
		int bad = 0;
		for (int i = 0; i < RAYS; i++)
		{
			tbvh_ray r = rays[i];
			r.O[0] += 1;
			if (tbvh_intersect( bvh, &r ) < 0 || !same_t( r.hit.t, ref_t[i] )) bad++;
		}
		CHECK( bad <= RAYS / 1000, "refit layout %d: %d mismatches", (int)ls[l], bad );
		tbvh_destroy( bvh );
		/* an HQ build has spatial splits: refit must be refused, not crash */
		OK( tbvh_create( ls[l], NULL, &bvh ) );
		OK( tbvh_build( bvh, &g, TBVH_QUALITY_HIGH ) );
		tbvh_info info;
		OK( tbvh_get_info( bvh, &info ) );
		const tbvh_result r = tbvh_refit( bvh );
		CHECK( (info.flags & TBVH_INFO_REFITTABLE) ? r == TBVH_SUCCESS : r == TBVH_ERROR_NOT_REFITTABLE,
			"HQ refit layout %d: %s, refittable flag %d", (int)ls[l], tbvh_result_string( r ), !!(info.flags & TBVH_INFO_REFITTABLE) );
		/* the HQ build must not leave spatial splits enabled for the next default build */
		tbvh_build_settings s;
		OK( tbvh_get_build_settings( bvh, &s ) );
		CHECK( !(s.flags & TBVH_BUILD_SPATIAL_SPLITS), "HQ quality leaked into the settings" );
		tbvh_destroy( bvh );
	}
	free( verts );
}

static void test_tlas( void )
{
	const tbvh_geometry g = { soup, TRIS * 3, 0, NULL, 0 };
	tbvh_bvh *blasA, *blasB, *tlas;
	OK( tbvh_create( TBVH_LAYOUT_BVH8_CPU, NULL, &blasA ) );
	OK( tbvh_create( TBVH_LAYOUT_BVH, NULL, &blasB ) );
	OK( tbvh_create( TBVH_LAYOUT_BVH, NULL, &tlas ) );
	OK( tbvh_build( blasA, &g, TBVH_QUALITY_DEFAULT ) );
	OK( tbvh_build( blasB, &g, TBVH_QUALITY_DEFAULT ) );
	tbvh_bvh* blases[2] = { blasA, blasB };
	/* instance 0: blasA at +100 x; instance 1: blasB at -100 x */
	tbvh_instance* inst = (tbvh_instance*)aligned_alloc( 64, 2 * sizeof( tbvh_instance ) );
	tbvh_instance_init( inst + 0, 0 ), inst[0].transform[3] = 100;
	tbvh_instance_init( inst + 1, 1 ), inst[1].transform[3] = -100;
	OK( tbvh_build_tlas( tlas, inst, 2, blases, 2 ) );
	tbvh_info info;
	OK( tbvh_get_info( tlas, &info ) );
	CHECK( (info.flags & TBVH_INFO_TLAS) && info.prim_count == 2, "TLAS info" );
	int bad = 0;
	for (int i = 0; i < RAYS; i++)
	{
		for (int k = 0; k < 2; k++)
		{
			tbvh_ray r = rays[i];
			r.O[0] += k == 0 ? 100.0f : -100.0f;
			if (tbvh_intersect( tlas, &r ) < 0 || !same_t( r.hit.t, ref_t[i] )) bad++;
			else if (ref_t[i] < 1e29f && r.hit.inst != (uint32_t)k) bad++;
		}
	}
	CHECK( bad <= RAYS / 500, "TLAS: %d mismatches", bad );
	/* bundle traversal over the TLAS */
	tbvh_config cfg;
	tbvh_get_config( &cfg );
	tbvh_ray* bundle = (tbvh_ray*)aligned_alloc( 64, cfg.bundle_rays * sizeof( tbvh_ray ) );
	uint8_t* occ = (uint8_t*)malloc( cfg.bundle_rays );
	for (uint32_t i = 0; i < cfg.bundle_rays; i++) bundle[i] = rays[i], bundle[i].O[0] += 100;
	CHECK( tbvh_intersect_bundle( tlas, bundle ) >= 0, "TLAS bundle: %s", tbvh_get_last_error() );
	bad = 0;
	for (uint32_t i = 0; i < cfg.bundle_rays; i++) if (!same_t( bundle[i].hit.t, ref_t[i] )) bad++;
	CHECK( bad == 0, "TLAS bundle: %d mismatches", bad );
	for (uint32_t i = 0; i < cfg.bundle_rays; i++) bundle[i] = rays[i], bundle[i].O[0] -= 100;
	CHECK( tbvh_is_occluded_bundle( tlas, bundle, occ ) >= 0, "TLAS occluded bundle" );
	bad = 0;
	for (uint32_t i = 0; i < cfg.bundle_rays; i++) if (occ[i] != (ref_t[i] < 1e29f)) bad++;
	CHECK( bad == 0, "TLAS occluded bundle: %d mismatches", bad );
	/* misuse */
	inst[1].blasIdx = 7;
	CHECK( tbvh_build_tlas( tlas, inst, 2, blases, 2 ) == TBVH_ERROR_INVALID_ARGUMENT, "TLAS: bad blasIdx accepted" );
	tbvh_bvh* gpu;
	OK( tbvh_create( TBVH_LAYOUT_BVH_GPU, NULL, &gpu ) );
	OK( tbvh_build( gpu, &g, TBVH_QUALITY_DEFAULT ) );
	inst[1].blasIdx = 1, blases[1] = gpu;
	CHECK( tbvh_build_tlas( tlas, inst, 2, blases, 2 ) == TBVH_ERROR_UNSUPPORTED, "TLAS: BVH_GPU BLAS accepted" );
	CHECK( tbvh_refit( tlas ) == TBVH_ERROR_UNSUPPORTED || tbvh_refit( tlas ) == TBVH_ERROR_NOT_REFITTABLE, "TLAS refit not refused" );
	tbvh_destroy( gpu );
	free( bundle ), free( occ ), free( inst );
	tbvh_destroy( tlas ), tbvh_destroy( blasA ), tbvh_destroy( blasB );
}

static void test_bundle_bvh4( void )
{
	const tbvh_geometry g = { soup, TRIS * 3, 0, NULL, 0 };
	tbvh_bvh* bvh;
	OK( tbvh_create( TBVH_LAYOUT_BVH4_CPU, NULL, &bvh ) );
	OK( tbvh_build( bvh, &g, TBVH_QUALITY_DEFAULT ) );
	tbvh_config cfg;
	tbvh_get_config( &cfg );
	tbvh_ray* bundle = (tbvh_ray*)aligned_alloc( 64, cfg.bundle_rays * sizeof( tbvh_ray ) );
	memcpy( bundle, rays, cfg.bundle_rays * sizeof( tbvh_ray ) );
	CHECK( tbvh_intersect_bundle( bvh, bundle ) >= 0, "BVH4 bundle: %s", tbvh_get_last_error() );
	int bad = 0;
	for (uint32_t i = 0; i < cfg.bundle_rays; i++) if (!same_t( bundle[i].hit.t, ref_t[i] )) bad++;
	CHECK( bad == 0, "BVH4 bundle: %d mismatches", bad );
	tbvh_bvh* b8;
	OK( tbvh_create( TBVH_LAYOUT_BVH8_CPU, NULL, &b8 ) );
	OK( tbvh_build( b8, &g, TBVH_QUALITY_DEFAULT ) );
	CHECK( tbvh_intersect_bundle( b8, bundle ) == -TBVH_ERROR_UNSUPPORTED, "BVH8_CPU bundle should be unsupported" );
	tbvh_destroy( b8 ), tbvh_destroy( bvh ), free( bundle );
}

static void test_custom( void )
{
	for (int i = 0; i < 64; i++)
		spheres[i].c[0] = rnd() * 10 - 5, spheres[i].c[1] = rnd() * 10 - 5, spheres[i].c[2] = rnd() * 10 - 5, spheres[i].r = 0.2f + rnd() * 0.5f;
	tbvh_bvh* bvh;
	OK( tbvh_create( TBVH_LAYOUT_BVH, NULL, &bvh ) );
	CHECK( tbvh_set_custom_geometry( bvh, sphere_intersect, NULL, spheres ) == TBVH_ERROR_INVALID_ARGUMENT, "intersect without occluded accepted" );
	OK( tbvh_set_custom_geometry( bvh, sphere_intersect, sphere_occluded, spheres ) ); /* before the build: kept */
	OK( tbvh_build_custom( bvh, sphere_aabb, 64 ) );
	int bad = 0, occBad = 0;
	for (int i = 0; i < RAYS; i++)
	{
		float best = 1e30f;
		for (int s = 0; s < 64; s++) { const float t = sphere_hit( spheres + s, rays[i].O, rays[i].D ); if (t < best) best = t; }
		tbvh_ray r = rays[i];
		if (tbvh_intersect( bvh, &r ) < 0 || !same_t( r.hit.t, best )) bad++;
		r = rays[i];
		if (tbvh_is_occluded( bvh, &r ) != (best < 1e29f)) occBad++;
	}
	CHECK( bad == 0 && occBad == 0, "custom spheres: %d hit / %d occlusion mismatches", bad, occBad );
	/* same spheres through tbvh_build_aabbs */
	float* boxes = (float*)aligned_alloc( 64, 64 * 8 * sizeof( float ) );
	for (int i = 0; i < 64; i++) { sphere_aabb( (uint32_t)i, boxes + i * 8, boxes + i * 8 + 4, spheres ); boxes[i * 8 + 3] = boxes[i * 8 + 7] = 0; }
	OK( tbvh_build_aabbs( bvh, boxes, 64 ) );
	tbvh_ray r = rays[1];
	CHECK( tbvh_intersect( bvh, &r ) >= 0, "AABB build traversal" );
	tbvh_info info;
	OK( tbvh_get_info( bvh, &info ) );
	CHECK( info.flags & TBVH_INFO_CUSTOM, "AABB build dropped the callbacks" );
	/* a triangle build clears the callbacks (in TinyBVH itself) */
	const tbvh_geometry g = { soup, TRIS * 3, 0, NULL, 0 };
	OK( tbvh_build( bvh, &g, TBVH_QUALITY_DEFAULT ) );
	OK( tbvh_get_info( bvh, &info ) );
	CHECK( !(info.flags & TBVH_INFO_CUSTOM), "triangle build kept the callbacks" );
	check_against_oracle( bvh, "BVH after custom" );
	free( boxes );
	tbvh_destroy( bvh );
}

static void test_errors( void )
{
	tbvh_bvh* bvh;
	tbvh_set_error_callback( on_error, NULL );
	OK( tbvh_create( TBVH_LAYOUT_BVH8_CPU, NULL, &bvh ) );
	/* validation */
	tbvh_geometry g = { soup + 1, TRIS * 3, 0, NULL, 0 };
	CHECK( tbvh_build( bvh, &g, TBVH_QUALITY_DEFAULT ) == TBVH_ERROR_INVALID_ARGUMENT, "misaligned vertices accepted" );
	g.vertices = soup, g.vertex_stride = 12;
	CHECK( tbvh_build( bvh, &g, TBVH_QUALITY_DEFAULT ) == TBVH_ERROR_INVALID_ARGUMENT, "stride 12 accepted" );
	uint32_t badIdx[3] = { 0, 1, 99999 };
	const tbvh_geometry gi = { soup, 3, 0, badIdx, 1 };
	CHECK( tbvh_build( bvh, &gi, TBVH_QUALITY_DEFAULT ) == TBVH_ERROR_INVALID_ARGUMENT, "out of range index accepted" );
	CHECK( strstr( tbvh_get_last_error(), "out of range" ) != NULL, "last error: %s", tbvh_get_last_error() );
	CHECK( tbvh_build( bvh, &g, TBVH_QUALITY_QUICK ) != TBVH_SUCCESS, "quick build on BVH8_CPU" );
	tbvh_ray r = rays[0];
	CHECK( tbvh_intersect( bvh, &r ) == -TBVH_ERROR_NOT_BUILT, "traversal of an unbuilt handle" );
	tbvh_rayd rd;
	tbvh_rayd_init( &rd, 0, 0, 0, 1, 0, 0, TBVH_DBL_FAR );
	CHECK( tbvh_intersectd( bvh, &rd ) == -TBVH_ERROR_WRONG_PRECISION, "double ray on float layout" );
	CHECK( tbvh_intersect( NULL, &r ) == -TBVH_ERROR_INVALID_HANDLE, "null handle" );
	/* a fatal error inside TinyBVH: NaN ray. Must come back as an error code. */
	g.vertex_stride = 0;
	OK( tbvh_build( bvh, &g, TBVH_QUALITY_DEFAULT ) );
	r = rays[0];
	r.D[1] = NAN;
	const int32_t res = tbvh_intersect( bvh, &r );
	tbvh_config cfg;
	tbvh_get_config( &cfg );
	if (cfg.features & TBVH_FEATURE_RECOVERABLE_ERRORS)
	{
		CHECK( res == -TBVH_ERROR_FATAL, "NaN ray: got %d", res );
		CHECK( strstr( tbvh_get_last_error(), "NaN" ) != NULL, "NaN ray message: %s", tbvh_get_last_error() );
	}
	/* a fatal error during a build: DOBB refuses spatial splits in its settings */
	tbvh_bvh* dobb;
	OK( tbvh_create( TBVH_LAYOUT_BVH_DOBB, NULL, &dobb ) );
	tbvh_build_settings s;
	tbvh_build_settings_default( &s );
	s.flags |= TBVH_BUILD_SPATIAL_SPLITS;
	OK( tbvh_set_build_settings( dobb, &s ) );
	const tbvh_result br = tbvh_build( dobb, &g, TBVH_QUALITY_DEFAULT );
	CHECK( br == TBVH_ERROR_FATAL, "DOBB with spatial splits: %s", tbvh_result_string( br ) );
	if (br == TBVH_ERROR_FATAL)
	{
		CHECK( tbvh_build( dobb, &g, TBVH_QUALITY_DEFAULT ) == TBVH_ERROR_INVALID_HANDLE, "broken handle accepted" );
		CHECK( strstr( tbvh_get_last_error(), "unusable" ) != NULL, "broken handle message: %s", tbvh_get_last_error() );
	}
	tbvh_destroy( dobb );
	/* the library is still usable */
	check_against_oracle( bvh, "BVH8_CPU after fatal errors" );
	s.struct_size = 8;
	CHECK( tbvh_set_build_settings( bvh, &s ) == TBVH_ERROR_INVALID_ARGUMENT, "short settings struct accepted" );
	CHECK( error_callback_calls > 0, "error callback not called" );
	tbvh_set_error_callback( NULL, NULL );
	tbvh_destroy( bvh );
	tbvh_destroy( NULL );
}

/* Out of memory, also during a rebuild that has to grow its buffers: reported as such, the
   handle is refused afterwards, and destroying it must not touch freed memory. */
static void test_out_of_memory( void )
{
	tbvh_config cfg;
	tbvh_get_config( &cfg );
	if (!(cfg.features & TBVH_FEATURE_RECOVERABLE_ERRORS)) return;
	tbvh_context ctx;
	tbvh_context_default( &ctx );
	ctx.alloc = arena_alloc, ctx.release = arena_release;
	const tbvh_geometry small = { soup, 300, 0, NULL, 0 }, big = { soup, TRIS * 3, 0, NULL, 0 };
	const tbvh_layout ls[] = { TBVH_LAYOUT_BVH, TBVH_LAYOUT_BVH4_CPU, TBVH_LAYOUT_BVH8_CPU, TBVH_LAYOUT_BVH_GPU };
	int oom = 0, ok = 0;
	for (int l = 0; l < 4; l++) for (int q = 0; q < 2; q++) for (long k = 0; k < 40; k++)
	{
		tbvh_bvh* bvh;
		arena_fail_in = -1;
		OK( tbvh_create( ls[l], &ctx, &bvh ) );
		OK( tbvh_build( bvh, &small, TBVH_QUALITY_DEFAULT ) );
		arena_fail_in = k;
		const tbvh_result r = tbvh_build( bvh, &big, q ? TBVH_QUALITY_HIGH : TBVH_QUALITY_DEFAULT );
		arena_fail_in = -1;
		if (r == TBVH_SUCCESS) ok++;
		else
		{
			oom++;
			CHECK( r == TBVH_ERROR_OUT_OF_MEMORY, "layout %d k %ld: %s (%s)", (int)ls[l], k, tbvh_result_string( r ), tbvh_get_last_error() );
			tbvh_ray ray = rays[0];
			CHECK( tbvh_intersect( bvh, &ray ) == -TBVH_ERROR_INVALID_HANDLE, "traversal of a broken handle" );
		}
		tbvh_destroy( bvh );
		arena_reset();
	}
	CHECK( oom > 0 && ok > 0, "out of memory sweep: %d failed, %d succeeded", oom, ok );
	arena_fail_in = 0;
	tbvh_bvh* none = NULL;
	CHECK( tbvh_create( TBVH_LAYOUT_BVH, &ctx, &none ) == TBVH_ERROR_OUT_OF_MEMORY && none == NULL, "create without memory" );
	arena_fail_in = -1;
	arena_reset();
}

static void test_save_load( void )
{
	const tbvh_geometry g = { soup, TRIS * 3, 0, NULL, 0 };
	const tbvh_layout ls[] = { TBVH_LAYOUT_BVH, TBVH_LAYOUT_BVH4_CPU, TBVH_LAYOUT_BVH8_CPU, TBVH_LAYOUT_BVH8_CWBVH };
	for (int l = 0; l < 4; l++)
	{
		tbvh_bvh *a, *b;
		OK( tbvh_create( ls[l], NULL, &a ) );
		OK( tbvh_build( a, &g, TBVH_QUALITY_DEFAULT ) );
		OK( tbvh_save( a, "test.bvh" ) );
		OK( tbvh_create( ls[l], NULL, &b ) );
		OK( tbvh_build( b, &g, TBVH_QUALITY_HIGH ) ); /* loading must replace this, without leaks */
		OK( tbvh_load( b, "test.bvh", &g ) );
		double ca = 0, cb = 1;
		if (tbvh_get_sah_cost( a, &ca ) == TBVH_SUCCESS && tbvh_get_sah_cost( b, &cb ) == TBVH_SUCCESS)
			CHECK( fabs( ca - cb ) < 1e-6 * ca, "layout %d: SAH after load %f vs %f", (int)ls[l], ca, cb );
		check_against_oracle( b, "loaded" );
		const tbvh_geometry wrong = { soup, 300, 0, NULL, 0 };
		CHECK( tbvh_load( b, "test.bvh", &wrong ) == TBVH_ERROR_IO, "layout %d: load with wrong geometry", (int)ls[l] );
		tbvh_destroy( a ), tbvh_destroy( b );
	}
	/* write failures are reported */
	tbvh_bvh* w;
	OK( tbvh_create( TBVH_LAYOUT_BVH8_CPU, NULL, &w ) );
	OK( tbvh_build( w, &g, TBVH_QUALITY_DEFAULT ) );
	CHECK( tbvh_save( w, "/nonexistent-dir/test.bvh" ) == TBVH_ERROR_IO, "save to a missing directory" );
	CHECK( tbvh_save( w, "/dev/full" ) == TBVH_ERROR_IO, "save to a full device" );
	/* a failed load leaves the tree in place */
	CHECK( tbvh_load( w, "no-such-file.bvh", NULL ) == TBVH_ERROR_IO, "load of a missing file" );
	check_against_oracle( w, "BVH8_CPU after a failed load" );
	tbvh_destroy( w );
	/* BVH file with geometry: read by BVH_GPU and the wide layouts, through the handle's context */
	tbvh_bvh* src;
	OK( tbvh_create( TBVH_LAYOUT_BVH, NULL, &src ) );
	OK( tbvh_build( src, &g, TBVH_QUALITY_DEFAULT ) );
	OK( tbvh_save( src, "test.bvh" ) );
	tbvh_destroy( src );
	tbvh_context ctx;
	tbvh_context_default( &ctx );
	ctx.alloc = count_alloc, ctx.release = count_free;
	const tbvh_layout conv[] = { TBVH_LAYOUT_BVH_GPU, TBVH_LAYOUT_BVH4_CPU, TBVH_LAYOUT_BVH8_CPU, TBVH_LAYOUT_BVH4_GPU, TBVH_LAYOUT_BVH8_CWBVH };
	for (int l = 0; l < 5; l++)
	{
		tbvh_bvh* d;
		live_allocs = 0, total_allocs = 0;
		OK( tbvh_create( conv[l], &ctx, &d ) );
		const tbvh_result r = tbvh_load( d, "test.bvh", NULL );
		CHECK( r == TBVH_SUCCESS, "layout %d from a BVH file: %s (%s)", (int)conv[l], tbvh_result_string( r ), tbvh_get_last_error() );
		if (r == TBVH_SUCCESS) check_against_oracle( d, "converted on load" );
		CHECK( total_allocs > 1, "layout %d: load did not allocate through the context", (int)conv[l] );
		tbvh_destroy( d );
		CHECK( live_allocs == 0, "layout %d: %ld allocations leaked by load", (int)conv[l], live_allocs );
	}
	/* BVH: geometry from the file */
	tbvh_bvh* c;
	OK( tbvh_create( TBVH_LAYOUT_BVH, NULL, &c ) );
	OK( tbvh_build( c, &g, TBVH_QUALITY_DEFAULT ) );
	OK( tbvh_save( c, "test.bvh" ) );
	tbvh_destroy( c );
	OK( tbvh_create( TBVH_LAYOUT_BVH, NULL, &c ) );
	OK( tbvh_load( c, "test.bvh", NULL ) );
	check_against_oracle( c, "BVH with file geometry" );
	tbvh_destroy( c );
	remove( "test.bvh" );
}

static void test_double( void )
{
	if (!tbvh_layout_supported( TBVH_LAYOUT_BVH_DOUBLE )) { printf( "double precision not compiled in\n" ); return; }
	double* dv = (double*)malloc( TRIS * 3 * 3 * sizeof( double ) );
	for (int i = 0; i < TRIS * 3; i++) for (int a = 0; a < 3; a++) dv[i * 3 + a] = soup[i * 4 + a];
	const tbvh_geometryd g = { dv, TRIS * 3, 0, NULL, 0 };
	const tbvh_layout ls[] = { TBVH_LAYOUT_BVH_DOUBLE, TBVH_LAYOUT_BVH4_CPU_DOUBLE };
	tbvh_bvh* blas[2];
	for (int l = 0; l < 2; l++)
	{
		OK( tbvh_create( ls[l], NULL, &blas[l] ) );
		const tbvh_geometry fg = { soup, TRIS * 3, 0, NULL, 0 };
		CHECK( tbvh_build( blas[l], &fg, TBVH_QUALITY_DEFAULT ) == TBVH_ERROR_WRONG_PRECISION, "float geometry on double layout" );
		OK( tbvh_buildd( blas[l], &g, TBVH_QUALITY_DEFAULT ) );
		int bad = 0;
		for (int i = 0; i < RAYS; i++)
		{
			tbvh_rayd r;
			tbvh_rayd_init( &r, rays[i].O[0], rays[i].O[1], rays[i].O[2], rays[i].D[0], rays[i].D[1], rays[i].D[2], TBVH_DBL_FAR );
			if (tbvh_intersectd( blas[l], &r ) < 0) bad++;
			else if (!same_t( r.hit.t >= 1e29 ? 1e30f : (float)r.hit.t, ref_t[i] )) bad++;
		}
		CHECK( bad <= RAYS / 1000, "double layout %d: %d mismatches", (int)ls[l], bad );
	}
	/* double TLAS over both */
	tbvh_bvh* tlas;
	OK( tbvh_create( TBVH_LAYOUT_BVH_DOUBLE, NULL, &tlas ) );
	tbvh_instanced* inst = (tbvh_instanced*)aligned_alloc( 64, 2 * sizeof( tbvh_instanced ) );
	tbvh_instanced_init( inst, 0 ), inst[0].transform[7] = 1e6;	/* far away: double precision territory */
	tbvh_instanced_init( inst + 1, 1 ), inst[1].transform[7] = -1e6;
	OK( tbvh_build_tlasd( tlas, inst, 2, blas, 2 ) );
	int bad = 0;
	for (int i = 0; i < RAYS; i++) for (int k = 0; k < 2; k++)
	{
		tbvh_rayd r;
		tbvh_rayd_init( &r, rays[i].O[0], rays[i].O[1] + (k ? -1e6 : 1e6), rays[i].O[2], rays[i].D[0], rays[i].D[1], rays[i].D[2], TBVH_DBL_FAR );
		if (tbvh_intersectd( tlas, &r ) < 0) bad++;
		else if (!same_t( r.hit.t >= 1e29 ? 1e30f : (float)r.hit.t, ref_t[i] )) bad++;
		else if (ref_t[i] < 1e29f && r.hit.inst != (uint64_t)k) bad++;
	}
	CHECK( bad <= RAYS / 500, "double TLAS: %d mismatches", bad );
	free( inst ), free( dv );
	tbvh_destroy( tlas ), tbvh_destroy( blas[0] ), tbvh_destroy( blas[1] );
}

static void test_context( void )
{
	tbvh_context ctx;
	tbvh_context_default( &ctx );
	ctx.alloc = count_alloc, ctx.release = count_free;
	const tbvh_geometry g = { soup, TRIS * 3, 0, NULL, 0 };
	for (int l = 0; l < (int)(sizeof( traversable ) / sizeof( traversable[0] )); l++)
	{
		tbvh_bvh* bvh;
		OK( tbvh_create( traversable[l], &ctx, &bvh ) );
		OK( tbvh_build( bvh, &g, TBVH_QUALITY_DEFAULT ) );
		OK( tbvh_build( bvh, &g, TBVH_QUALITY_DEFAULT ) ); /* rebuild reuses or frees */
		tbvh_destroy( bvh );
		CHECK( live_allocs == 0, "%s: %ld allocations leaked", names[l], live_allocs );
		live_allocs = 0;
	}
	CHECK( total_allocs > 0, "custom allocator was not used" );
	/* single threaded build: no pool hooks */
	ctx.spawn = NULL, ctx.barrier = NULL, ctx.parallel_for = NULL;
	tbvh_bvh* bvh;
	OK( tbvh_create( TBVH_LAYOUT_BVH8_CPU, &ctx, &bvh ) );
	OK( tbvh_build( bvh, &g, TBVH_QUALITY_DEFAULT ) );
	check_against_oracle( bvh, "BVH8_CPU, no thread pool" );
	tbvh_destroy( bvh );
}

/* Above MT_BUILD_THRESHOLD, so builds use the context's pool hooks. */
static void test_threaded( void )
{
	enum { BIG = 120000, BIGRAYS = 300 };
	float* v = (float*)aligned_alloc( 64, BIG * 3 * 16 );
	for (int i = 0; i < BIG; i++)
	{
		const float c[3] = { rnd() * 20, rnd() * 20, rnd() * 20 };
		for (int k = 0; k < 3; k++) { float* p = v + (i * 3 + k) * 4; for (int a = 0; a < 3; a++) p[a] = c[a] + rnd() * 0.3f; p[3] = 0; }
	}
	tbvh_ray* br = (tbvh_ray*)aligned_alloc( 64, BIGRAYS * sizeof( tbvh_ray ) );
	float* bt = (float*)malloc( BIGRAYS * sizeof( float ) );
	for (int i = 0; i < BIGRAYS; i++)
	{
		tbvh_ray_init( br + i, rnd() * 20, rnd() * 20, -1, rnd() - 0.5f, rnd() - 0.5f, 1, TBVH_FAR );
		bt[i] = 1e30f;
		for (int t = 0; t < BIG; t++) { const float d = tri_hit( br[i].O, br[i].D, v + t * 12, v + t * 12 + 4, v + t * 12 + 8 ); if (d < bt[i]) bt[i] = d; }
	}
	tbvh_context counting;
	tbvh_context_default( &counting );
	counting.alloc = count_alloc, counting.release = count_free;
	const tbvh_geometry g = { v, BIG * 3, 0, NULL, 0 };
	const tbvh_layout ls[] = { TBVH_LAYOUT_BVH, TBVH_LAYOUT_BVH4_CPU, TBVH_LAYOUT_BVH8_CPU };
	for (int c = 0; c < 2; c++) for (int l = 0; l < 3; l++) for (int q = 0; q < 2; q++)
	{
		tbvh_bvh* bvh;
		live_allocs = 0;
		OK( tbvh_create( ls[l], c ? &counting : NULL, &bvh ) );
		OK( tbvh_build( bvh, &g, q ? TBVH_QUALITY_HIGH : TBVH_QUALITY_DEFAULT ) );
		int bad = 0;
		for (int i = 0; i < BIGRAYS; i++) { tbvh_ray r = br[i]; if (tbvh_intersect( bvh, &r ) < 0 || !same_t( r.hit.t, bt[i] )) bad++; }
		CHECK( bad == 0, "threaded layout %d quality %d context %d: %d mismatches", (int)ls[l], q, c, bad );
		if (!q)
		{
			for (int i = 0; i < BIG * 3; i++) v[i * 4 + 2] += 2;
			OK( tbvh_refit( bvh ) );
			bad = 0;
			for (int i = 0; i < BIGRAYS; i++) { tbvh_ray r = br[i]; r.O[2] += 2; if (tbvh_intersect( bvh, &r ) < 0 || !same_t( r.hit.t, bt[i] )) bad++; }
			for (int i = 0; i < BIG * 3; i++) v[i * 4 + 2] -= 2;
			CHECK( bad == 0, "threaded refit layout %d context %d: %d mismatches", (int)ls[l], c, bad );
		}
		tbvh_destroy( bvh );
		if (c) CHECK( live_allocs == 0, "threaded layout %d: %ld allocations leaked", (int)ls[l], live_allocs );
	}
	free( v ), free( br ), free( bt );
}

int main( void )
{
	tbvh_config cfg;
	tbvh_get_config( &cfg );
	printf( "TinyBVH %u.%u.%u, C API %u, simd 0x%x, features 0x%x, bundle %u rays\n", cfg.version_major, cfg.version_minor,
		cfg.version_sub, cfg.api_version, cfg.simd, cfg.features, cfg.bundle_rays );
	CHECK( tbvh_abi_ok(), "ABI mismatch between header and library" );
	make_soup(), make_indexed(), make_rays();
	int hits = 0;
	for (int i = 0; i < RAYS; i++) hits += ref_t[i] < 1e29f;
	printf( "%d triangles, %d rays, %d hit\n", TRIS, RAYS, hits );
	test_layouts();
	test_refit();
	test_tlas();
	test_bundle_bvh4();
	test_custom();
	test_errors();
	test_out_of_memory();
	test_save_load();
	test_double();
	test_context();
	test_threaded();
	tbvh_shutdown_thread_pool();
	printf( "%d checks, %d failures\n", checks, failures );
	return failures ? 1 : 0;
}
