// Compare the SIMD kernels and builders against the scalar reference.
//
// This file instantiates SIMD and scalar BVH implementations side by side and
// compares them against each other. The scalar side uses the fact that the
// platform headers specialize BVH<float, uint32_t> but not BVH<float, uint64_t>,
// which therefore falls back to the scalar code.
//
// The trees are built with the reference builder on both sides, so that the
// comparison is between kernels and not builders. The builders also have their
// own comparison.

#define TINYBVH_IMPLEMENTATION
#define USE_DEPRECATED_LAYOUT // enables BVH_SoA
#include "tiny_bvh.h"
#include <cstdio>
#include <vector>

using namespace tinybvh;

using RayS = impl::Ray<float, uint64_t>;
using BVHS = impl::BVH<float, uint64_t>;
using BVHBaseS = impl::BVHBase<float, uint64_t>;
using BLASInstanceS = impl::BLASInstance<float, uint64_t>;
using BVH4_CPUS = impl::BVH4_CPU<float, uint64_t>;
using BVH8_CPUS = impl::BVH8_CPU<float, uint64_t>;
using BVH_SoAS = impl::BVH_SoA<float, uint64_t>;

static constexpr float SCENE_SIZE = 10.0f;		// triangles and ray origins live in [0, SCENE_SIZE]^3
static constexpr float T_TOLERANCE = 2e-5f;		// FMA versus separate multiply and add
static constexpr int RAYS = 20000;
static constexpr int INSTANCES = 4;
static constexpr float INST_SPACING = 20.0f;

static int g_failures = 0, g_testFailures = 0;

struct Scene
{
	std::vector<bvhvec4> verts;		// three vertices per triangle, or the shared vertices of the grid
	std::vector<uint32_t> indices;	// empty for a triangle soup
	std::vector<uint32_t> opmap;	// opacity micro maps, opmapN^2 bits per triangle
	uint32_t triCount = 0, opmapN = 0;
};

// Random triangle soup: small triangles scattered through the scene box.
static Scene MakeSoup( const uint32_t count, uint32_t seed )
{
	Scene s;
	s.triCount = count;
	for (uint32_t i = 0; i < count; i++)
	{
		const bvhvec3 c( tinybvh_rndfloat( seed ) * SCENE_SIZE, tinybvh_rndfloat( seed ) * SCENE_SIZE, tinybvh_rndfloat( seed ) * SCENE_SIZE );
		for (int k = 0; k < 3; k++) s.verts.push_back( bvhvec4( c + tinybvh_rndvec3( seed ) * 0.5f, 0 ) );
	}
	return s;
}

// Indexed height field with shared vertices. This exercises the indexed leaf conversion.
static Scene MakeGrid( const int res, uint32_t seed )
{
	Scene s;
	for (int j = 0; j <= res; j++) for (int i = 0; i <= res; i++)
		s.verts.push_back( bvhvec4( SCENE_SIZE * i / res, SCENE_SIZE * j / res, SCENE_SIZE * 0.5f + tinybvh_rndfloat( seed ), 0 ) );
	for (int j = 0; j < res; j++) for (int i = 0; i < res; i++)
	{
		const uint32_t a = j * (res + 1) + i, b = a + 1, c = a + res + 1, d = c + 1;
		const uint32_t tris[6] = { a, b, c, b, d, c };
		for (uint32_t t : tris) s.indices.push_back( t );
	}
	s.triCount = res * res * 2;
	return s;
}

// Random opacity micro maps: about half of the micro triangles are transparent.
static void AddOpacityMaps( Scene& s, const uint32_t N, uint32_t seed )
{
	s.opmapN = N;
	s.opmap.resize( (size_t)s.triCount * ((N * N + 31) >> 5) );
	for (uint32_t& w : s.opmap) w = tinybvh_rnduint( seed );
}

template <class Acc> static void Build( Acc& acc, const Scene& s )
{
	acc.settings.useSIMDifavailable = false; // the reference builder gives both sides the same tree
	if (s.indices.empty()) acc.Build( s.verts.data(), s.triCount );
	else acc.Build( s.verts.data(), s.indices.data(), s.triCount );
	if (s.opmapN) acc.SetOpacityMicroMaps( (uint32_t*)s.opmap.data(), s.opmapN );
}

static void Fail( const char* name, const char* what, const int ray, const float ta, const float tb, const uint64_t pa, const uint64_t pb )
{
	g_failures++;
	if (g_testFailures++ == 0)
		printf( "FAIL: %s, %s, ray %i: SIMD t=%.7g prim=%llu, scalar t=%.7g prim=%llu\n", name, what, ray, ta, (unsigned long long)pa, tb, (unsigned long long)pb );
}

static bool SameHit( const float ta, const float tb, const uint64_t pa, const uint64_t pb )
{
	// same distance, or the same primitive with a rounding difference in t.
	if (fabsf( ta - tb ) > T_TOLERANCE) return false;
	return pa == pb || (ta < BVH_FAR && tb < BVH_FAR);
}

// Traces the same random rays through 'a' (SIMD side) and 'b' (scalar side), comparing
// the closest hit, occlusion, and occlusion by a ray that ends just before the hit.
template <class RayA, class RayB, class AccA, class AccB>
static void CompareRays( const char* name, const AccA& a, const AccB& b, const int instances = 1 )
{
	uint32_t seed = 0x9E3779B9u;
	for (int i = 0; i < RAYS; i++)
	{
		// origins in and around the scene box, of the instance selected for this ray.
		const float span = SCENE_SIZE * 1.4f, margin = SCENE_SIZE * 0.2f;
		bvhvec3 O( tinybvh_rndfloat( seed ) * span - margin, tinybvh_rndfloat( seed ) * span - margin, tinybvh_rndfloat( seed ) * span - margin );
		O.z += (i % instances) * INST_SPACING;
		bvhvec3 D = tinybvh_rndvec3( seed );
		if ((i & 7) == 0) D[i % 3] = 0, D = tinybvh_normalize( D ); // some rays with a zero component
		RayA ra( O, D );
		RayB rb( O, D );
		a.Intersect( ra ), b.Intersect( rb );
		if (!SameHit( ra.hit.t, rb.hit.t, ra.hit.prim, rb.hit.prim )) Fail( name, "Intersect", i, ra.hit.t, rb.hit.t, ra.hit.prim, rb.hit.prim );
		else if (ra.hit.t < BVH_FAR && (fabsf( ra.hit.u - rb.hit.u ) > 1e-4f || fabsf( ra.hit.v - rb.hit.v ) > 1e-4f) && ra.hit.prim == rb.hit.prim)
			Fail( name, "Intersect barycentrics", i, ra.hit.u, rb.hit.u, ra.hit.prim, rb.hit.prim );
		const RayA sa( O, D );
		const RayB sb( O, D );
		if (a.IsOccluded( sa ) != b.IsOccluded( sb )) Fail( name, "IsOccluded", i, ra.hit.t, rb.hit.t, ra.hit.prim, rb.hit.prim );
		if (ra.hit.t < BVH_FAR)
		{
			const RayA la( O, D, ra.hit.t * 0.99f );
			const RayB lb( O, D, ra.hit.t * 0.99f );
			if (a.IsOccluded( la ) != b.IsOccluded( lb )) Fail( name, "IsOccluded, short ray", i, ra.hit.t, rb.hit.t, ra.hit.prim, rb.hit.prim );
		}
	}
	if (g_testFailures) printf( "FAIL: %s, %i failing checks.\n", name, g_testFailures );
	g_testFailures = 0;
}

template <class AccA, class AccB> static void CompareLayouts( const char* name, const Scene& s )
{
	AccA a;
	AccB b;
	Build( a, s ), Build( b, s );
	CompareRays<Ray, RayS>( name, a, b );
}

// Instances: identity plus 90 degree rotations about each axis, spaced along z, as in the
// axis-aligned ray test. Compares the TLAS traversal, which dispatches to the BLAS kernel.
template <class Inst> static void SetTransform( Inst& inst, int i )
{
	float m[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
	if (i > 0)
	{
		const int axis = i - 1, u = (axis + 1) % 3, v = (axis + 2) % 3;
		m[u * 4 + u] = m[v * 4 + v] = 0, m[u * 4 + v] = -1, m[v * 4 + u] = 1;
		// keep the rotated scene box in place: rotate about the box center.
		const float c = SCENE_SIZE * 0.5f;
		m[u * 4 + 3] = c - m[u * 4 + u] * c - m[u * 4 + v] * c;
		m[v * 4 + 3] = c - m[v * 4 + u] * c - m[v * 4 + v] * c;
	}
	m[11] += i * INST_SPACING;
	for (int k = 0; k < 16; k++) inst.transform[k] = m[k];
}

template <class AccA, class AccB> static void CompareTLAS( const char* name, const Scene& s )
{
	AccA a;
	AccB b;
	Build( a, s ), Build( b, s );
	BLASInstance instA[INSTANCES];
	BLASInstanceS instB[INSTANCES];
	for (int i = 0; i < INSTANCES; i++) instA[i] = BLASInstance( 0 ), instB[i] = BLASInstanceS( 0 ), SetTransform( instA[i], i ), SetTransform( instB[i], i );
	BVHBase* blasA = &a;
	BVHBaseS* blasB = &b;
	BVH tlasA;
	BVHS tlasB;
	tlasA.Build( instA, INSTANCES, &blasA, 1 ), tlasB.Build( instB, INSTANCES, &blasB, 1 );
	CompareRays<Ray, RayS>( name, tlasA, tlasB, INSTANCES );
}

// The SIMD builders against the reference builder: different trees, same closest hits.
static void CompareBuilders( const Scene& s )
{
	BVH reference;
	Build( reference, s );
#if defined BVH_USEAVX && !defined BVH_USENEON
	{
		BVH avx;
		avx.BuildAVX( s.verts.data(), s.triCount );
		CompareRays<Ray, Ray>( "BuildAVX versus reference builder", avx, reference );
	}
#endif
#ifdef BVH_USENEON
	{
		BVH neon;
		neon.BuildNEON( s.verts.data(), s.triCount );
		CompareRays<Ray, Ray>( "BuildNEON versus reference builder", neon, reference );
	}
#endif
	// The SBVH builder clips fragments with SSE code on x86 and scalar code elsewhere.
	BVH hq;
	BVHS hqS;
	hq.BuildHQ( s.verts.data(), s.triCount ), hqS.BuildHQ( s.verts.data(), s.triCount );
	if (hq.usedNodes != hqS.usedNodes || fabsf( hq.SAHCost() - hqS.SAHCost() ) > 1e-3f * hq.SAHCost())
		g_failures++, printf( "FAIL: BuildHQ, SIMD tree has %u nodes, SAH %.4f; scalar tree has %llu nodes, SAH %.4f\n",
			hq.usedNodes, hq.SAHCost(), (unsigned long long)hqS.usedNodes, hqS.SAHCost() );
	CompareRays<Ray, RayS>( "BuildHQ, SIMD versus scalar fragment clipping", hq, hqS );
}

#if defined BVH_USEAVX && !defined BVH_USENEON
// The SSE packet traversal against the scalar packet traversal. A packet is a 16x16 block
// of rays from one origin through a square on a plane; the corner rays bound the frustum.
static void ComparePackets( const Scene& s )
{
	BVH bvh;
	Build( bvh, s );
	uint32_t seed = 0x1234567u;
	ALIGNED( 64 ) Ray packetA[256], packetB[256];
	for (int p = 0; p < 32; p++)
	{
		const bvhvec3 O( tinybvh_rndfloat( seed ) * SCENE_SIZE, tinybvh_rndfloat( seed ) * SCENE_SIZE, -1.0f - tinybvh_rndfloat( seed ) * 3 );
		const bvhvec3 corner( tinybvh_rndfloat( seed ) * SCENE_SIZE * 0.8f, tinybvh_rndfloat( seed ) * SCENE_SIZE * 0.8f, 0 );
		const float size = 0.5f + tinybvh_rndfloat( seed ) * 2;
		for (int y = 0; y < 16; y++) for (int x = 0; x < 16; x++)
		{
			const bvhvec3 target = corner + bvhvec3( size * x / 15.0f, size * y / 15.0f, 0 );
			packetA[y * 16 + x] = packetB[y * 16 + x] = Ray( O, target - O );
		}
		bvh.Intersect256RaysSSE( packetA ), bvh.Intersect256Rays( packetB );
		for (int i = 0; i < 256; i++) if (!SameHit( packetA[i].hit.t, packetB[i].hit.t, packetA[i].hit.prim, packetB[i].hit.prim ))
			Fail( "Intersect256RaysSSE versus Intersect256Rays", "packet", p * 256 + i, packetA[i].hit.t, packetB[i].hit.t, packetA[i].hit.prim, packetB[i].hit.prim );
	}
	if (g_testFailures) printf( "FAIL: Intersect256RaysSSE, %i failing checks.\n", g_testFailures );
	g_testFailures = 0;
}
#endif

int main()
{
	Scene soup = MakeSoup( 4000, 12345 ), grid = MakeGrid( 48, 777 ), mapped = MakeSoup( 4000, 12345 );
	AddOpacityMaps( mapped, 4, 4321 );

	// traversal kernels
	CompareLayouts<BVH4_CPU, BVH4_CPUS>( "BVH4_CPU, soup", soup );
	CompareLayouts<BVH4_CPU, BVH4_CPUS>( "BVH4_CPU, indexed grid", grid );
	CompareLayouts<BVH4_CPU, BVH4_CPUS>( "BVH4_CPU, opacity maps", mapped );
	CompareTLAS<BVH4_CPU, BVH4_CPUS>( "TLAS over BVH4_CPU", soup );
	CompareLayouts<BVH8_CPU, BVH8_CPUS>( "BVH8_CPU, soup", soup );
	CompareLayouts<BVH8_CPU, BVH8_CPUS>( "BVH8_CPU, indexed grid", grid );
	CompareLayouts<BVH8_CPU, BVH8_CPUS>( "BVH8_CPU, opacity maps", mapped );
	CompareTLAS<BVH8_CPU, BVH8_CPUS>( "TLAS over BVH8_CPU", soup );
	CompareLayouts<BVH_SoA, BVH_SoAS>( "BVH_SoA, soup", soup );
	CompareLayouts<BVH_SoA, BVH_SoAS>( "BVH_SoA, indexed grid", grid );

	// builders and packets
	CompareBuilders( soup );
#if defined BVH_USEAVX && !defined BVH_USENEON
	ComparePackets( soup );
#endif

	if (g_failures) { printf( "%i SIMD reference test failures.\n", g_failures ); return 1; }
	printf( "SIMD reference tests passed.\n" );
	return 0;
}
