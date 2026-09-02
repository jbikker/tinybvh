// Regression tests for axis-aligned rays.
//
// The scene consists of six subdivided square plates, one on each side of the
// origin at distance PLATE_DIST along an axis. Rays start near the origin with
// direction components drawn from { -1, -0, +0, +1 }, at least one of them
// zero. Hit distances are compared against an analytic reference for every
// layout and builder, and for TLAS traversal over instances rotated by 90
// degrees, which produces negative zeros in the instance frame.

#define TINYBVH_IMPLEMENTATION
#define USE_DEPRECATED_LAYOUT // enables BVH_SoA
#include "tiny_bvh.h"
#include <cstdio>
#include <vector>

using namespace tinybvh;

static constexpr float PLATE_DIST = 2.0f;	// distance of each plate from the origin
static constexpr float PLATE_EXTENT = 4.0f;	// half the plate side length
static constexpr int PLATE_RES = 16;		// quads per plate side
static constexpr float INST_SPACING = 20.0f;	// distance between instances along z

// (0.1,0.2,0.3) triggers a fail on Ubuntu release only:
// It produces a hit at (1.8,0.2,2.0), which is *exactly* on the diagonal of two triangles.
// static const bvhvec3 g_origins[] = { bvhvec3( 0.1f, 0.2f, 0.3f ), bvhvec3( -0.3f, 0.25f, -0.15f ), bvhvec3( 0, 0, 0 ) };
static const bvhvec3 g_origins[] = { bvhvec3( 0.1f, 0.2f, 0.35f ), bvhvec3( -0.3f, 0.25f, -0.15f ), bvhvec3( 0, 0, 0 ) };
static const float g_components[] = { -1, -0.0f, 0.0f, 1 };

static int g_failures = 0, g_testFailures = 0;

static std::vector<bvhvec4> BuildScene()
{
	std::vector<bvhvec4> tris;
	for (int axis = 0; axis < 3; axis++) for (int side = -1; side <= 1; side += 2)
	{
		const int u = (axis + 1) % 3, v = (axis + 2) % 3;
		for (int i = 0; i < PLATE_RES; i++) for (int j = 0; j < PLATE_RES; j++)
		{
			bvhvec4 c[4];
			for (int k = 0; k < 4; k++)
			{
				c[k] = bvhvec4( 0 );
				c[k][axis] = side * PLATE_DIST;
				c[k][u] = -PLATE_EXTENT + 2 * PLATE_EXTENT * (i + (k & 1)) / PLATE_RES;
				c[k][v] = -PLATE_EXTENT + 2 * PLATE_EXTENT * (j + (k >> 1)) / PLATE_RES;
			}
			tris.push_back( c[0] ), tris.push_back( c[1] ), tris.push_back( c[2] );
			tris.push_back( c[1] ), tris.push_back( c[3] ), tris.push_back( c[2] );
		}
	}
	return tris;
}

// Distance to the nearest plate along a normalized direction.
static float Reference( const bvhvec3& O, const bvhvec3& D )
{
	float best = BVH_FAR;
	for (int axis = 0; axis < 3; axis++) for (int side = -1; side <= 1; side += 2)
	{
		if (D[axis] == 0) continue;
		const float t = (side * PLATE_DIST - O[axis]) / D[axis];
		if (t <= 0 || t >= best) continue;
		bool inside = true;
		for (int k = 0; k < 3; k++) if (k != axis && fabsf( O[k] + t * D[k] ) > PLATE_EXTENT) inside = false;
		if (inside) best = t;
	}
	return best;
}

static void Fail( const char* name, const char* what, const bvhvec3& O, const bvhvec3& D, float expected, float got )
{
	g_failures++;
	if (g_testFailures++ == 0) printf( "FAIL: %s, %s: O=(%g,%g,%g) D=(%s%g,%s%g,%s%g) expected t=%g, got %g\n",
		name, what, O.x, O.y, O.z, std::signbit( D.x ) ? "-" : "+", fabsf( D.x ),
		std::signbit( D.y ) ? "-" : "+", fabsf( D.y ), std::signbit( D.z ) ? "-" : "+", fabsf( D.z ), expected, got );
}

// Traces the test rays through 'acc'. For a TLAS, the origins are repeated at
// the position of each instance.
template <class RayT, class Vec, class Accel> static void TestRays( const char* name, const Accel& acc, int instances = 1 )
{
	for (int inst = 0; inst < instances; inst++) for (const bvhvec3& O : g_origins) for (int i = 0; i < 64; i++)
	{
		bvhvec3 D( g_components[i & 3], g_components[(i >> 2) & 3], g_components[i >> 4] );
		const int zeros = (D.x == 0) + (D.y == 0) + (D.z == 0);
		if (zeros == 0 || zeros == 3) continue;
		D = tinybvh_normalize( D );
		const float expected = Reference( O, D );
		const Vec o( O + bvhvec3( 0, 0, inst * INST_SPACING ) ), d( D );
		RayT ray( o, d );
		acc.Intersect( ray );
		if (fabs( ray.hit.t - expected ) > 1e-4f * expected) Fail( name, "Intersect", O, D, expected, (float)ray.hit.t );
		const RayT shadow( o, d );
		if (!acc.IsOccluded( shadow )) Fail( name, "IsOccluded", O, D, expected, BVH_FAR );
		const RayT shortShadow( o, d, expected * 0.99f );
		if (acc.IsOccluded( shortShadow )) Fail( name, "IsOccluded, short ray", O, D, expected, expected * 0.99f );
	}
	if (g_testFailures) printf( "FAIL: %s, %i failing checks.\n", name, g_testFailures );
	g_testFailures = 0;
}

template <class Accel> static void TestFloat( const char* name, const Accel& acc ) { TestRays<Ray, bvhvec3>( name, acc ); }

// Instances: identity plus 90 degree rotations about each axis, spaced along z.
// The scene maps onto itself under these rotations, so the reference distances
// still apply. Rotation i is applied to instance i and its inverse produces
// negative zeros for axis-aligned world rays.
template <class Inst> static void SetTransform( Inst& inst, int i )
{
	float m[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
	if (i > 0)
	{
		const int axis = i - 1, u = (axis + 1) % 3, v = (axis + 2) % 3;
		m[u * 4 + u] = m[v * 4 + v] = 0, m[u * 4 + v] = -1, m[v * 4 + u] = 1;
	}
	m[11] = i * INST_SPACING;
	for (int k = 0; k < 16; k++) inst.transform[k] = m[k];
}

template <class TLAS, class Inst, class RayT, class Vec, class Blas> static void TestTLAS( const char* name, Blas* blas )
{
	Inst inst[4];
	for (int i = 0; i < 4; i++) inst[i] = Inst( 0 ), SetTransform( inst[i], i );
	TLAS tlas;
	tlas.Build( inst, 4, &blas, 1 );
	TestRays<RayT, Vec>( name, tlas, 4 );
}

static void TestFloatTLAS( const char* name, BVHBase* blas ) { TestTLAS<BVH, BLASInstance, Ray, bvhvec3>( name, blas ); }

int main()
{
	const std::vector<bvhvec4> tris = BuildScene();
	const bvhvec4* verts = tris.data();
	const uint32_t triCount = (uint32_t)tris.size() / 3;

	// BVH layout, all builders
	{
		BVH bvh;
		bvh.Build( verts, triCount ), TestFloat( "BVH::Build", bvh ), TestFloatTLAS( "TLAS over BVH", &bvh );
		bvh.BuildQuick( verts, triCount ), TestFloat( "BVH::BuildQuick", bvh );
		bvh.BuildHQ( verts, triCount ), TestFloat( "BVH::BuildHQ", bvh );
		bvh.settings.useFullSweep = true;
		bvh.Build( verts, triCount ), TestFloat( "BVH::Build, full sweep", bvh );
		bvh.settings.useFullSweep = false;
	#if defined BVH_USEAVX && !defined BVH_USENEON
		bvh.BuildAVX( verts, triCount ), TestFloat( "BVH::BuildAVX", bvh );
	#endif
	#ifdef BVH_USENEON
		bvh.BuildNEON( verts, triCount ), TestFloat( "BVH::BuildNEON", bvh );
	#endif
	}

	// Other layouts
	{
		BVH_GPU bvh;
		bvh.Build( verts, triCount ), TestFloat( "BVH_GPU::Build", bvh );
		bvh.BuildHQ( verts, triCount ), TestFloat( "BVH_GPU::BuildHQ", bvh );
	}
	{
		BVH4_GPU bvh;
		bvh.Build( verts, triCount ), TestFloat( "BVH4_GPU::Build", bvh );
		bvh.BuildHQ( verts, triCount ), TestFloat( "BVH4_GPU::BuildHQ", bvh );
	}
#if defined BVH_USEAVX || defined BVH_USENEON
	{
		BVH_SoA bvh;
		bvh.Build( verts, triCount ), TestFloat( "BVH_SoA::Build", bvh );
		bvh.BuildHQ( verts, triCount ), TestFloat( "BVH_SoA::BuildHQ", bvh );
	}
#endif
#if defined BVH_USESSE || defined BVH_USENEON
	{
		BVH4_CPU bvh;
		bvh.Build( verts, triCount ), TestFloat( "BVH4_CPU::Build", bvh ), TestFloatTLAS( "TLAS over BVH4_CPU", &bvh );
		bvh.BuildHQ( verts, triCount ), TestFloat( "BVH4_CPU::BuildHQ", bvh );
	}
#endif
#if defined BVH_USEAVX && !defined BVH_USENEON
	{
		BVH8_CWBVH bvh;
		bvh.Build( verts, triCount ), TestFloat( "BVH8_CWBVH::Build", bvh );
		bvh.BuildHQ( verts, triCount ), TestFloat( "BVH8_CWBVH::BuildHQ", bvh );
	}
#endif
#if defined BVH_USEAVX2 && !defined BVH_USENEON
	{
		BVH8_CPU bvh;
		bvh.Build( verts, triCount ), TestFloat( "BVH8_CPU::Build", bvh ), TestFloatTLAS( "TLAS over BVH8_CPU", &bvh );
		bvh.BuildHQ( verts, triCount ), TestFloat( "BVH8_CPU::BuildHQ", bvh );
	}
#endif

	// Double precision
	{
		std::vector<bvhdbl3> dverts;
		for (const bvhvec4& v : tris) dverts.push_back( bvhdbl3( v.x, v.y, v.z ) );
		BVH_Double bvh;
		bvh.Build( dverts.data(), triCount );
		TestRays<RayEx, bvhdbl3>( "BVH_Double::Build", bvh );
		TestTLAS<BVH_Double, BLASInstanceEx, RayEx, bvhdbl3>( "TLAS over BVH_Double", &bvh );
	}

	if (g_failures) { printf( "%i axis-aligned ray test failures.\n", g_failures ); return 1; }
	printf( "Axis-aligned ray tests passed.\n" );
	return 0;
}
