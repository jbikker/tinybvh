// Correctness tests for the double-precision BVH (BVH_Double / RayEx).
// These run as part of CTest and return a non-zero exit code on failure.

#define TINYBVH_IMPLEMENTATION
#define TINYBVH_NO_SIMD
#include "tiny_bvh.h"
#include <cstdio>
#include <cstdlib>

using namespace tinybvh;

static int g_failures = 0;

#define CHECK( cond ) do { \
	if (!(cond)) { printf( "FAIL: %s (line %i)\n", #cond, __LINE__ ); g_failures++; } \
} while (0)

// ----------------------------------------------------------------------------
// Custom geometry: a small set of spheres, intersected analytically.
// ----------------------------------------------------------------------------

struct Sphere { bvhdbl3 pos; double r; };
static Sphere* g_spheres = 0;

static bool sphereIntersect( RayEx& ray, const uint64_t primID )
{
	const bvhdbl3 oc = ray.O - g_spheres[primID].pos;
	const double b = tinybvh_dot( oc, ray.D );
	const double r = g_spheres[primID].r;
	const double c = tinybvh_dot( oc, oc ) - r * r;
	double d = b * b - c;
	if (d <= 0) return false;
	d = sqrt( d );
	const double t = -b - d;
	const bool hit = t < ray.hit.t && t > 0;
	if (hit) ray.hit.t = t, ray.hit.prim = primID;
	return hit;
}

static bool sphereIsOccluded( const RayEx& ray, const uint64_t primID )
{
	const bvhdbl3 oc = ray.O - g_spheres[primID].pos;
	const double b = tinybvh_dot( oc, ray.D );
	const double r = g_spheres[primID].r;
	const double c = tinybvh_dot( oc, oc ) - r * r;
	double d = b * b - c;
	if (d <= 0) return false;
	d = sqrt( d );
	const double t = -b - d;
	return t < ray.hit.t && t > 0;
}

static void sphereAABB( const uint64_t primID, bvhdbl3& bmin, bvhdbl3& bmax )
{
	bmin = g_spheres[primID].pos - bvhdbl3( g_spheres[primID].r );
	bmax = g_spheres[primID].pos + bvhdbl3( g_spheres[primID].r );
}

static void TestCustomShadowRays()
{
	printf( "Test: double-precision custom-geometry shadow rays...\n" );
	const int N = 3;
	g_spheres = new Sphere[N];
	g_spheres[0].pos = bvhdbl3( 0, 0, 5 ), g_spheres[0].r = 1.0;
	g_spheres[1].pos = bvhdbl3( 8, 0, 0 ), g_spheres[1].r = 1.0;
	g_spheres[2].pos = bvhdbl3( 0, -7, 0 ), g_spheres[2].r = 1.0;

	BVH_Double bvh;
	bvh.Build( &sphereAABB, N );
	bvh.customIntersect = &sphereIntersect;
	bvh.customIsOccluded = &sphereIsOccluded;

	// Ray straight at sphere 0 (front face at z=4); should be occluded and hit.
	{
		RayEx ray( bvhdbl3( 0, 0, 0 ), bvhdbl3( 0, 0, 1 ), 1e30 );
		CHECK( bvh.IsOccluded( ray ) == true );
		RayEx ray2( bvhdbl3( 0, 0, 0 ), bvhdbl3( 0, 0, 1 ), 1e30 );
		bvh.Intersect( ray2 );
		CHECK( ray2.hit.t < 1e30 ); // a hit was found
		CHECK( ray2.hit.prim == 0 );
	}
	// Ray into empty space; should not be occluded and should not hit.
	{
		RayEx ray( bvhdbl3( 0, 0, 0 ), bvhdbl3( 0, 1, 1 ), 1e30 );
		CHECK( bvh.IsOccluded( ray ) == false );
		RayEx ray2( bvhdbl3( 0, 0, 0 ), bvhdbl3( 0, 1, 1 ), 1e30 );
		bvh.Intersect( ray2 );
		CHECK( ray2.hit.t == 1e30 ); // no hit
	}
	// Ray toward sphere 0 but with tmax shorter than the hit distance (t~4).
	// The sphere is beyond the ray, so it must not occlude.
	{
		RayEx ray( bvhdbl3( 0, 0, 0 ), bvhdbl3( 0, 0, 1 ), 3.0 );
		CHECK( bvh.IsOccluded( ray ) == false );
	}
	// Occlusion result must agree with Intersect for a batch of directions.
	for (int i = 0; i < N; i++)
	{
		const bvhdbl3 dir = tinybvh_normalize( g_spheres[i].pos );
		RayEx shadow( bvhdbl3( 0, 0, 0 ), dir, 1e30 );
		RayEx probe( bvhdbl3( 0, 0, 0 ), dir, 1e30 );
		bvh.Intersect( probe );
		const bool hit = probe.hit.t < 1e30;
		CHECK( bvh.IsOccluded( shadow ) == hit );
	}

	delete[] g_spheres, g_spheres = 0;
}

int main()
{
	TestCustomShadowRays();

	if (g_failures == 0) { printf( "All double-precision tests passed.\n" ); return 0; }
	printf( "%i double-precision test(s) FAILED.\n", g_failures );
	return 1;
}
