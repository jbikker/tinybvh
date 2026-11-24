// Validation of motion blur: BVH_Motion, and moving instances in a TLAS.
//
// The references are a static BVH built over the geometry at the time of the rays, and brute
// force over all triangles at that time. Also checked: the bounds invariants at random times,
// occlusion against intersection, indexed input, Refit, move semantics, IntersectBundle on a
// TLAS with motion, and the double precision instantiation.
//
// Usage: tiny_bvh_motion_test [scene] - default ./testdata/cryteksponza.bin. Without a scene
// file a procedural triangle soup is used, so the test also runs without the test data.

#define TINYBVH_IMPLEMENTATION
#include "tiny_bvh.h"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <vector>
#include <chrono>
#include <unordered_map>

using namespace tinybvh;

static int failures = 0;
#define CHECK( c, ... ) { if (!(c)) { failures++; printf( "  FAIL (line %i): ", __LINE__ ); printf( __VA_ARGS__ ); printf( "\n" ); } }

// xorshift32
static uint32_t seed = 0x2545f491;
static uint32_t RandomUInt() { seed ^= seed << 13, seed ^= seed >> 17, seed ^= seed << 5; return seed; }
static float RandomFloat() { return (float)(RandomUInt() >> 8) * (1.0f / 16777216.0f); }
static bvhvec3 RandomDir()
{
	while (1)
	{
		const bvhvec3 d( RandomFloat() * 2 - 1, RandomFloat() * 2 - 1, RandomFloat() * 2 - 1 );
		const float l2 = tinybvh_dot( d, d );
		if (l2 > 0.01f && l2 <= 1) return d * (1.0f / sqrtf( l2 ));
	}
}

// Scene: a triangle soup, three vertices per triangle.
static bool LoadScene( const char* fileName, std::vector<bvhvec4>& v )
{
	FILE* f = fopen( fileName, "rb" );
	if (!f) return false;
	uint32_t n = 0;
	bool ok = fread( &n, 4, 1, f ) == 1 && n > 0;
	if (ok) v.resize( (size_t)n * 3 ), ok = fread( v.data(), 48, n, f ) == n;
	fclose( f );
	return ok;
}
static void ProceduralScene( std::vector<bvhvec4>& v, const uint32_t n )
{
	v.resize( (size_t)n * 3 );
	for (uint32_t i = 0; i < n; i++)
	{
		const bvhvec3 p( RandomFloat() * 100 - 50, RandomFloat() * 40 - 3, RandomFloat() * 60 - 30 );
		for (int k = 0; k < 3; k++) v[i * 3 + k] = bvhvec4( p + RandomDir() * 0.5f, 1.0f );
	}
}

// The end-of-shutter key: a smooth deformation, which keeps shared vertices together, and a
// few triangles that fly off on their own.
static std::vector<bvhvec4> Animate( const std::vector<bvhvec4>& v0, const float amplitude )
{
	std::vector<bvhvec4> v1( v0.size() );
	for (size_t i = 0; i < v0.size(); i++)
	{
		const bvhvec4 p = v0[i];
		v1[i] = bvhvec4( p.x + amplitude * sinf( p.y * 0.3f ), p.y + amplitude * 0.5f * cosf( p.x * 0.2f ),
			p.z + amplitude * sinf( p.x * 0.25f + p.z * 0.1f ), p.w );
	}
	uint32_t s = 0x9e3779b9u;
	for (size_t t = 0; t < v0.size() / 3; t++)
	{
		s ^= s << 13, s ^= s >> 17, s ^= s << 5;
		if (s & 31) continue;
		const bvhvec3 d( sinf( (float)s ), cosf( (float)(s >> 3) ), sinf( (float)(s >> 7) ) );
		for (int k = 0; k < 3; k++) v1[t * 3 + k] = bvhvec4( bvhvec3( v1[t * 3 + k] ) + d * (4 * amplitude), v1[t * 3 + k].w );
	}
	return v1;
}

// The geometry at time t1, interpolated as tiny_bvh does.
static bvhvec3 At( const bvhvec4& a, const bvhvec4& b, const float t1 )
{
	return tinybvh_motion_lerp( bvhvec3( a ), bvhvec3( b ), 1 - t1, t1 );
}
static std::vector<bvhvec4> Snapshot( const std::vector<bvhvec4>& v0, const std::vector<bvhvec4>& v1, const float t1 )
{
	std::vector<bvhvec4> v( v0.size() );
	for (size_t i = 0; i < v.size(); i++) v[i] = bvhvec4( At( v0[i], v1[i], t1 ), 0 );
	return v;
}

// Moeller-Trumbore, as in tiny_bvh, for float and double.
template <typename V3, typename F> static bool TriTest( const V3& O, const V3& D, const V3& v0, const V3& v1, const V3& v2, const F tmax, F& tHit, F* uHit = 0, F* vHit = 0 )
{
	const V3 e1 = v1 - v0, e2 = v2 - v0, h = tinybvh_cross( D, e2 );
	const F a = tinybvh_dot( e1, h );
	if (a == 0) return false;
	const F f = 1 / a;
	const V3 s = O - v0, q = tinybvh_cross( s, e1 );
	const F u = f * tinybvh_dot( s, h ), v = f * tinybvh_dot( D, q );
	if (!(u >= 0 && v >= 0 && u + v <= 1)) return false;
	const F t = f * tinybvh_dot( e2, q );
	if (!(t >= 0 && t < tmax)) return false;
	tHit = t;
	if (uHit) *uHit = u, *vHit = v;
	return true;
}
// Nearest hit over all triangles; u and v of that hit if asked for.
static float BruteForce( const bvhvec3& O, const bvhvec3& D, const std::vector<bvhvec4>& v, float tmax = BVH_FAR, float* u = 0, float* w = 0 )
{
	for (size_t i = 0; i < v.size(); i += 3) TriTest( O, D, bvhvec3( v[i] ), bvhvec3( v[i + 1] ), bvhvec3( v[i + 2] ), tmax, tmax, u, w );
	return tmax;
}
// A hit this close to an edge of its triangle can go either way under rounding: two evaluations
// of the same ray, in different spaces or under different FMA contraction, may disagree on it.
static bool NearEdge( const float u, const float v, const float eps ) { return tinybvh_min( tinybvh_min( u, v ), 1 - u - v ) < eps; }

// Agreement of two traversals: the same distance, up to rounding, or both a miss. 'scale': the
// magnitude of the coordinates, for results computed in different spaces.
template <typename F> static bool SameHit( const F a, const F b, const F scale = 0 )
{
	if (a >= bvh_far<F> || b >= bvh_far<F>) return a == b;
	return fabs( a - b ) <= F( 1e-4 ) * (1 + fabs( a )) + F( 1e-5 ) * scale;
}
static float Scale( const bvhvec3& p ) { return tinybvh_max( fabsf( p.x ), tinybvh_max( fabsf( p.y ), fabsf( p.z ) ) ); }

// Rays from random points in the scene bounds, in random directions, at random times.
static Ray RandomRay( const bvhvec3& bmin, const bvhvec3& bmax, const float time )
{
	const bvhvec3 O( bmin.x + RandomFloat() * (bmax.x - bmin.x), bmin.y + RandomFloat() * (bmax.y - bmin.y), bmin.z + RandomFloat() * (bmax.z - bmin.z) );
	return Ray( O, RandomDir(), BVH_FAR, RAY_MASK_INTERSECT_ALL, time );
}

// The invariants of a BVH_Motion: the keys of every node contain those of its children, every
// triangle is inside its leaf and inside the bounds over the shutter interval at random times,
// and every primitive is in exactly one leaf.
static void CheckBounds( const BVH_Motion& m, const int times )
{
	uint64_t badNodes = 0, badTris = 0, badRoot = 0;
	std::vector<uint32_t> seen( m.triCount, 0 );
	auto vertex = [&]( const uint32_t prim, const int k ) { return m.vertIdx ? m.vertIdx[prim * 3 + k] : prim * 3 + k; };
	auto contains = []( const bvhvec3& pmin, const bvhvec3& pmax, const bvhvec3& cmin, const bvhvec3& cmax )
	{
		return pmin.x <= cmin.x && pmin.y <= cmin.y && pmin.z <= cmin.z && pmax.x >= cmax.x && pmax.y >= cmax.y && pmax.z >= cmax.z;
	};
	for (int s = 0; s < times; s++)
	{
		const float t1 = s == 0 ? 0 : s == 1 ? 1 : RandomFloat(), t0 = 1 - t1;
		for (uint32_t i = 0; i < m.usedNodes; i++) if (i != 1)
		{
			const BVH_Motion::BVHNode& n = m.bvhNode[i];
			const bvhvec3 bmin = tinybvh_motion_lerp( n.aabbMin0, n.aabbMin1, t0, t1 );
			const bvhvec3 bmax = tinybvh_motion_lerp( n.aabbMax0, n.aabbMax1, t0, t1 );
			auto inside = [&]( const bvhvec3& a, const bvhvec3& b ) { return a.x >= bmin.x && a.y >= bmin.y && a.z >= bmin.z && b.x <= bmax.x && b.y <= bmax.y && b.z <= bmax.z; };
			// interior nodes, on the keys: the monotone lerp carries this over to every time. Two
			// lerps evaluated here may round differently under FMA contraction; in traversal,
			// the leaf padding absorbs that.
			if (!n.isLeaf())
			{
				for (uint32_t c = 0; s == 0 && c < 2; c++)
				{
					const BVH_Motion::BVHNode& child = m.bvhNode[n.leftFirst + c];
					if (!contains( n.aabbMin0, n.aabbMax0, child.aabbMin0, child.aabbMax0 ) ||
						!contains( n.aabbMin1, n.aabbMax1, child.aabbMin1, child.aabbMax1 )) badNodes++;
				}
			}
			else for (uint32_t j = 0; j < n.triCount; j++)
			{
				const uint32_t prim = m.primIdx[n.leftFirst + j];
				if (s == 0) seen[prim]++;
				for (int k = 0; k < 3; k++)
				{
					const uint32_t vi = vertex( prim, k );
					const bvhvec3 p = tinybvh_motion_lerp( bvhvec3( m.verts0[vi] ), bvhvec3( m.verts1[vi] ), t0, t1 );
					if (!inside( p, p )) badTris++;
					if (p.x < m.aabbMin.x || p.y < m.aabbMin.y || p.z < m.aabbMin.z || p.x > m.aabbMax.x || p.y > m.aabbMax.y || p.z > m.aabbMax.z) badRoot++;
				}
			}
		}
	}
	uint32_t notOnce = 0;
	for (uint32_t i = 0; i < m.triCount; i++) if (seen[i] != 1) notOnce++;
	CHECK( badNodes == 0, "%llu children outside their parent", (unsigned long long)badNodes );
	CHECK( badTris == 0, "%llu vertices outside their leaf", (unsigned long long)badTris );
	CHECK( badRoot == 0, "%llu vertices outside the shutter interval bounds", (unsigned long long)badRoot );
	CHECK( notOnce == 0, "%u primitives not in exactly one leaf", notOnce );
	CHECK( m.idxCount == m.triCount, "idxCount %u, triCount %u", m.idxCount, m.triCount );
}

// BVH_Motion against static BVHs over the geometry at fixed times, and against brute force at
// a few of those, for intersection and occlusion.
static void CheckTraversal( const BVH_Motion& m, const std::vector<bvhvec4>& v0, const std::vector<bvhvec4>& v1, const uint32_t raysPerTime, const uint32_t bruteRays )
{
	const bvhvec3 bmin = m.aabbMin, bmax = m.aabbMax;
	uint32_t mismatches = 0, bruteMismatches = 0, occlusionErrors = 0, hits = 0, rays = 0;
	for (int s = 0; s < 10; s++)
	{
		const float time = s == 0 ? 0 : s == 1 ? 1 : RandomFloat();
		const std::vector<bvhvec4> snap = Snapshot( v0, v1, time );
		BVH reference;
		reference.Build( snap.data(), (uint32_t)(snap.size() / 3) );
		for (uint32_t i = 0; i < raysPerTime; i++, rays++)
		{
			Ray ray = RandomRay( bmin, bmax, time ), ref = ray;
			m.Intersect( ray );
			reference.Intersect( ref );
			if (!SameHit( ray.hit.t, ref.hit.t )) mismatches++;
			if (ray.hit.t < BVH_FAR) hits++;
			if (i < bruteRays && !SameHit( ray.hit.t, BruteForce( ray.O, ray.D, snap ) )) bruteMismatches++;
			// occlusion: blocked just beyond the hit, free just before it.
			Ray shadow( ray.O, ray.D, ray.hit.t < BVH_FAR ? ray.hit.t * 1.001f + 1e-4f : BVH_FAR, RAY_MASK_INTERSECT_ALL, time );
			if (m.IsOccluded( shadow ) != (ray.hit.t < BVH_FAR)) occlusionErrors++;
			if (ray.hit.t < BVH_FAR)
			{
				shadow.hit.t = ray.hit.t * 0.999f;
				if (m.IsOccluded( shadow )) occlusionErrors++;
			}
		}
	}
	printf( "  %u rays, %.1f%% hit; %u mismatches vs static BVH, %u vs brute force, %u occlusion errors\n",
		rays, 100.0f * hits / rays, mismatches, bruteMismatches, occlusionErrors );
	CHECK( mismatches == 0, "BVH_Motion and static BVH disagree on %u rays", mismatches );
	CHECK( bruteMismatches == 0, "BVH_Motion and brute force disagree on %u rays", bruteMismatches );
	CHECK( occlusionErrors == 0, "IsOccluded disagrees with Intersect on %u rays", occlusionErrors );
}

// Geometry that does not move: BVH_Motion should find what BVH finds, at any time, at about the
// same SAH cost. BVH's default builder (Bonsai) builds slightly better trees than a binned builder
// such as BVH_Motion's, so BVH_Motion may cost a little more.
static void TestStatic( const std::vector<bvhvec4>& v )
{
	printf( "static geometry:\n" );
	const uint32_t n = (uint32_t)(v.size() / 3);
	BVH bvh;
	BVH_Motion m;
	bvh.Build( v.data(), n );
	m.Build( v.data(), v.data(), n );
	uint32_t mismatches = 0, edgeGrazes = 0;
	for (uint32_t i = 0; i < 100000; i++)
	{
		Ray a = RandomRay( bvh.aabbMin, bvh.aabbMax, RandomFloat() ), b = a;
		bvh.Intersect( a ), m.Intersect( b );
		if (SameHit( a.hit.t, b.hit.t )) continue;
		// BVH and BVH_Motion test triangles in different code, which under -ffast-math may round a
		// hit this close to an edge either way; the nearer of the two hits is the one the other missed.
		const Ray& nearer = a.hit.t < b.hit.t ? a : b;
		if (NearEdge( nearer.hit.u, nearer.hit.v, 1e-5f )) edgeGrazes++; else mismatches++;
	}
	const float sah = bvh.SAHCost(), motionSah = m.SAHCost();
	printf( "  SAH cost: BVH %.3f, BVH_Motion %.3f; %u mismatches in 100000 rays (%u edge grazes)\n", sah, motionSah, mismatches, edgeGrazes );
	CHECK( mismatches == 0, "BVH_Motion over static geometry disagrees with BVH on %u rays", mismatches );
	CHECK( edgeGrazes <= 10, "%u edge grazes in 100000 rays: too many to be rounding", edgeGrazes );
	CHECK( motionSah <= 1.03f * sah, "SAH cost %f vs %f", sah, motionSah );
}

static void TestMotion( const std::vector<bvhvec4>& v0, const float amplitude, const uint32_t raysPerTime, const uint32_t bruteRays )
{
	printf( "moving geometry, amplitude %.2f:\n", amplitude );
	const std::vector<bvhvec4> v1 = Animate( v0, amplitude );
	const uint32_t n = (uint32_t)(v0.size() / 3);
	BVH_Motion m;
	m.Build( v0.data(), v1.data(), n );
	const std::vector<bvhvec4> mid = Snapshot( v0, v1, 0.5f );
	BVH midBVH;
	midBVH.Build( mid.data(), n );
	printf( "  %u nodes; SAH cost %.3f over the shutter interval (mid-shutter static BVH: %.3f)\n", m.usedNodes, m.SAHCost(), midBVH.SAHCost() );
	CheckBounds( m, 8 );
	CheckTraversal( m, v0, v1, raysPerTime, bruteRays );
	// times outside the shutter interval are clamped to it.
	uint32_t clampErrors = 0;
	for (uint32_t i = 0; i < 10000; i++)
	{
		const bool early = i & 1;
		Ray a = RandomRay( m.aabbMin, m.aabbMax, early ? -0.5f : 1.5f ), b = a;
		b.time = early ? 0.0f : 1.0f;
		m.Intersect( a ), m.Intersect( b );
		if (a.hit.t != b.hit.t) clampErrors++;
	}
	CHECK( clampErrors == 0, "%u rays outside the shutter interval not clamped", clampErrors );
}

// A threaded build makes the tree of a single-threaded build; only the order of the nodes in memory differs.
static void TestThreading( const std::vector<bvhvec4>& v0 )
{
	printf( "threaded build:\n" );
	const std::vector<bvhvec4> v1 = Animate( v0, 0.5f );
	const uint32_t n = (uint32_t)(v0.size() / 3);
	BVH_Motion threaded, single;
	single.settings.enableThreading = false;
	threaded.Build( v0.data(), v1.data(), n );
	single.Build( v0.data(), v1.data(), n );
	uint32_t mismatches = 0;
	for (uint32_t i = 0; i < 20000; i++)
	{
		Ray a = RandomRay( threaded.aabbMin, threaded.aabbMax, RandomFloat() ), b = a;
		threaded.Intersect( a ), single.Intersect( b );
		if (a.hit.t != b.hit.t || (a.hit.t < BVH_FAR && a.hit.prim != b.hit.prim)) mismatches++;
	}
	const float sah = threaded.SAHCost(), singleSah = single.SAHCost();
	printf( "  %u and %u nodes, SAH cost %.3f and %.3f; %u mismatches in 20000 rays\n", threaded.usedNodes, single.usedNodes, sah, singleSah, mismatches );
	CHECK( threaded.usedNodes == single.usedNodes && fabsf( sah - singleSah ) <= 1e-6f * sah, "threaded and single-threaded builds differ" );
	CHECK( mismatches == 0, "threaded and single-threaded builds disagree on %u rays", mismatches );
}

// Indexed input: the same triangles through a vertex buffer without duplicates.
static void TestIndexed( const std::vector<bvhvec4>& v0 )
{
	printf( "indexed geometry:\n" );
	const std::vector<bvhvec4> v1 = Animate( v0, 0.5f );
	struct Key { float x[6]; bool operator==( const Key& o ) const { for (int i = 0; i < 6; i++) if (x[i] != o.x[i]) return false; return true; } };
	struct KeyHash { size_t operator()( const Key& k ) const { size_t h = 0; for (int i = 0; i < 6; i++) { uint32_t u; memcpy( &u, &k.x[i], 4 ); h = h * 0x9e3779b97f4a7c15ull + u; } return h; } };
	std::unordered_map<Key, uint32_t, KeyHash> unique;
	std::vector<bvhvec4> u0, u1;
	std::vector<uint32_t> idx( v0.size() );
	for (size_t i = 0; i < v0.size(); i++)
	{
		const Key k = { { v0[i].x, v0[i].y, v0[i].z, v1[i].x, v1[i].y, v1[i].z } };
		auto it = unique.find( k );
		if (it == unique.end()) it = unique.emplace( k, (uint32_t)u0.size() ).first, u0.push_back( v0[i] ), u1.push_back( v1[i] );
		idx[i] = it->second;
	}
	const uint32_t n = (uint32_t)(v0.size() / 3);
	BVH_Motion soup, indexed;
	soup.Build( v0.data(), v1.data(), n );
	indexed.Build( bvhvec4slice( u0.data(), (uint32_t)u0.size() ), bvhvec4slice( u1.data(), (uint32_t)u1.size() ), idx.data(), n );
	CheckBounds( indexed, 4 );
	uint32_t mismatches = 0;
	for (uint32_t i = 0; i < 100000; i++)
	{
		Ray a = RandomRay( soup.aabbMin, soup.aabbMax, RandomFloat() ), b = a;
		soup.Intersect( a ), indexed.Intersect( b );
		if (a.hit.t != b.hit.t || (a.hit.t < BVH_FAR && a.hit.prim != b.hit.prim)) mismatches++;
	}
	printf( "  %zu unique vertices for %u triangles; %u mismatches in 100000 rays\n", u0.size(), n, mismatches );
	CHECK( mismatches == 0, "indexed and non-indexed BVH_Motion disagree on %u rays", mismatches );
}

// Refit: new end-of-shutter positions in the same buffer.
static void TestRefit( const std::vector<bvhvec4>& v0 )
{
	printf( "refit:\n" );
	std::vector<bvhvec4> v1 = Animate( v0, 0.25f );
	const uint32_t n = (uint32_t)(v0.size() / 3);
	BVH_Motion m;
	m.Build( v0.data(), v1.data(), n );
	const float before = m.SAHCost();
	const std::vector<bvhvec4> larger = Animate( v0, 1.0f );
	memcpy( v1.data(), larger.data(), larger.size() * sizeof( bvhvec4 ) ); // in place, as for an animation
	m.Refit();
	printf( "  SAH cost %.3f before, %.3f after refitting to larger motion\n", before, m.SAHCost() );
	CheckBounds( m, 4 );
	CheckTraversal( m, v0, v1, 2000, 0 );
	// a rebuild of the same object, over other keys, reuses its allocations.
	const std::vector<bvhvec4> other = Animate( v0, 2.0f );
	m.Build( v0.data(), other.data(), n );
	printf( "  rebuilt: SAH cost %.3f\n", m.SAHCost() );
	CheckBounds( m, 2 );
	CheckTraversal( m, v0, other, 1000, 0 );
}

// Degenerate input for the builder: a single triangle, and a stack of identical triangles, half
// of which stay put while the others move away; with and without keepSplitting, which splits
// down to leaves of one triangle.
static void TestDegenerate()
{
	printf( "degenerate input:\n" );
	const bvhvec4 tri[3] = { bvhvec4( 0, 0, 0, 0 ), bvhvec4( 1, 0, 0, 0 ), bvhvec4( 0, 1, 0, 0 ) };
	std::vector<bvhvec4> v0, v1;
	for (int i = 0; i < 64; i++) for (int k = 0; k < 3; k++)
		v0.push_back( tri[k] ), v1.push_back( tri[k] + bvhvec4( 0, 0, i < 32 ? 0.0f : 2.0f, 0 ) );
	uint32_t mismatches = 0, badLeaves = 0;
	BVH_Motion m; // one object for all builds: its buffers grow, and are reused at a smaller size.
	for (int keepSplitting = 0; keepSplitting < 2; keepSplitting++) for (uint32_t n : { 1u, 64u, 32u })
	{
		m.settings.keepSplitting = keepSplitting == 1;
		m.Build( v0.data(), v1.data(), n );
		CheckBounds( m, 4 );
		for (uint32_t i = 0; i < m.usedNodes; i++) if (i != 1 && m.bvhNode[i].isLeaf() && keepSplitting && m.bvhNode[i].triCount != 1) badLeaves++;
		const std::vector<bvhvec4> keys0( v0.begin(), v0.begin() + n * 3 ), keys1( v1.begin(), v1.begin() + n * 3 );
		for (int i = 0; i < 200; i++)
		{
			const float time = RandomFloat();
			const std::vector<bvhvec4> snap = Snapshot( keys0, keys1, time );
			Ray ray( bvhvec3( RandomFloat() * 0.6f, RandomFloat() * 0.6f, -1 ), tinybvh_normalize( bvhvec3( RandomFloat() * 0.1f, RandomFloat() * 0.1f, 1 ) ), BVH_FAR, RAY_MASK_INTERSECT_ALL, time );
			m.Intersect( ray );
			if (!SameHit( ray.hit.t, BruteForce( ray.O, ray.D, snap ) )) mismatches++;
		}
	}
	printf( "  %u mismatches vs brute force; %u leaves of more than one triangle with keepSplitting\n", mismatches, badLeaves );
	CHECK( mismatches == 0, "BVH_Motion over degenerate input disagrees with brute force on %u rays", mismatches );
	CHECK( badLeaves == 0, "keepSplitting left %u leaves of more than one triangle", badLeaves );
}

// Move construction and assignment hand over the buffers.
static void TestMove( const std::vector<bvhvec4>& v0 )
{
	printf( "move semantics:\n" );
	const std::vector<bvhvec4> v1 = Animate( v0, 0.5f );
	const uint32_t n = (uint32_t)(v0.size() / 3);
	BVH_Motion a;
	a.Build( v0.data(), v1.data(), n );
	Ray probe = RandomRay( a.aabbMin, a.aabbMax, 0.3f ), probe2 = probe, probe3 = probe;
	a.Intersect( probe );
	BVH_Motion b( std::move( a ) );
	CHECK( a.bvhNode == 0 && a.primIdx == 0, "moved-from BVH_Motion still holds buffers" );
	b.Intersect( probe2 );
	BVH_Motion c;
	c.Build( v0.data(), v0.data(), n );
	c = std::move( b );
	c.Intersect( probe3 );
	CHECK( probe.hit.t == probe2.hit.t && probe.hit.t == probe3.hit.t, "results changed after a move" );
	a.Build( v0.data(), v1.data(), n ); // the moved-from object can be rebuilt
	CHECK( a.bvhNode != 0, "moved-from BVH_Motion did not rebuild" );
}

// TLAS: static and moving instances of every BLAS layout the TLAS supports on the CPU, against
// brute force over the world-space triangles at the time of the rays.
static bvhmat4 Transform( const float angle, const bvhvec3& pos, const float scale )
{
	bvhmat4 m;
	const float c = cosf( angle ) * scale, s = sinf( angle ) * scale;
	m[0] = c, m[1] = 0, m[2] = s, m[3] = pos.x;
	m[4] = 0, m[5] = scale, m[6] = 0, m[7] = pos.y;
	m[8] = -s, m[9] = 0, m[10] = c, m[11] = pos.z;
	return m;
}
static bvhmat4 LerpMat( const bvhmat4& a, const bvhmat4& b, const float t1 )
{
	bvhmat4 m;
	for (int i = 0; i < 16; i++) m[i] = tinybvh_motion_lerp( a[i], b[i], 1 - t1, t1 );
	return m;
}

static void TestTLAS( const std::vector<bvhvec4>& scene )
{
	printf( "TLAS:\n" );
	// BLASses over a part of the scene: static BVH, BVH4_CPU and BVH8_CPU, and a BVH_Motion.
	const uint32_t n = tinybvh_min( (uint32_t)(scene.size() / 3), 20000u );
	const std::vector<bvhvec4> v0( scene.begin(), scene.begin() + n * 3 ), v1 = Animate( v0, 0.5f );
	BVH blas0;
	BVH4_CPU blas1;
	BVH8_CPU blas2;
	BVH_Motion blas3;
	blas0.Build( v0.data(), n );
	blas1.Build( v0.data(), n );
	blas2.Build( v0.data(), n );
	blas3.Build( v0.data(), v1.data(), n );
	BVHBase* blases[4] = { &blas0, &blas1, &blas2, &blas3 };
	const bool moves[4] = { false, false, false, true };
	// instances: rotated and translated copies along x; every other one moves.
	const uint32_t instCount = 8;
	BLASInstance inst[instCount];
	bvhmat4 end[instCount];
	for (uint32_t i = 0; i < instCount; i++)
	{
		inst[i] = BLASInstance( i & 3 );
		inst[i].transform = Transform( 0.3f * i, bvhvec3( 120.0f * i, 0, 0 ), 1 + 0.1f * i );
		if (i & 4) end[i] = Transform( 0.3f * i + 0.4f, bvhvec3( 120.0f * i + 6, 3, -2 ), 1.1f + 0.1f * i ), inst[i].transform1 = &end[i];
	}
	BVH tlas;
	tlas.Build( inst, instCount, blases, 4 );
	CHECK( tlas.hasMotion, "TLAS with moving instances not flagged" );
	// rays and two references at a few fixed times. Object space: brute force through the inverse
	// of each instance at the ray time, as the TLAS does. World space: brute force over triangles
	// moved by the test's own interpolated transforms, independent of tiny_bvh; the two spaces
	// round differently, so there a disagreement on a hit grazing an edge is no error.
	uint32_t rays = 0, hits = 0, mismatches = 0, worldMismatches = 0, edgeCases = 0, occlusionErrors = 0, bundleMismatches = 0;
	auto agree = [&]( const float tlasT, const float tlasU, const float tlasV, const float refT, const float refU, const float refV, const float scale, const float eps )
	{
		if (SameHit( tlasT, refT, scale )) return true;
		// the nearer of the two hits is the one the other side missed.
		const bool tlasNearer = tlasT < refT;
		if (NearEdge( tlasNearer ? tlasU : refU, tlasNearer ? tlasV : refV, eps )) return edgeCases++, true;
		return false;
	};
	for (int s = 0; s < 6; s++)
	{
		const float time = s == 0 ? 0 : s == 1 ? 1 : RandomFloat();
		const std::vector<bvhvec4> moved = Snapshot( v0, v1, time );
		std::vector<bvhvec4> world;
		for (uint32_t i = 0; i < instCount; i++)
		{
			const bvhmat4 M = inst[i].transform1 ? LerpMat( inst[i].transform, *inst[i].transform1, time ) : inst[i].transform;
			for (uint32_t j = 0; j < n * 3; j++)
			{
				const bvhvec3 p = moves[inst[i].blasIdx] ? At( v0[j], v1[j], time ) : bvhvec3( v0[j] );
				world.push_back( bvhvec4( tinybvh_transform_point( p, M ), 0 ) );
			}
		}
		Ray bundle[TINYBVH_BUNDLE_RAYS], shadows[TINYBVH_BUNDLE_RAYS];
		for (uint32_t i = 0; i < 400; i++, rays++)
		{
			Ray ray = RandomRay( tlas.aabbMin, tlas.aabbMax, time );
			ray.hit.t = 1e4f;
			if (i < TINYBVH_BUNDLE_RAYS) bundle[i] = shadows[i] = ray;
			tlas.Intersect( ray );
			float ref = 1e4f, refU = 0, refV = 0;
			for (uint32_t k = 0; k < instCount; k++)
			{
				const bvhmat4 inv = inst[k].InvTransformAt( time );
				ref = BruteForce( tinybvh_transform_point( ray.O, inv ), tinybvh_transform_vector( ray.D, inv ),
					moves[inst[k].blasIdx] ? moved : v0, ref, &refU, &refV );
			}
			if (!agree( ray.hit.t, ray.hit.u, ray.hit.v, ref, refU, refV, 0, 1e-5f )) mismatches++;
			float worldU = 0, worldV = 0;
			const float worldRef = BruteForce( ray.O, ray.D, world, 1e4f, &worldU, &worldV );
			if (!agree( ray.hit.t, ray.hit.u, ray.hit.v, worldRef, worldU, worldV, Scale( ray.O ) + worldRef, 1e-3f )) worldMismatches++;
			if (ray.hit.t < 1e4f) hits++;
			Ray shadow( ray.O, ray.D, ray.hit.t < 1e4f ? ray.hit.t * 1.001f + 1e-4f : 1e4f, RAY_MASK_INTERSECT_ALL, time );
			if (tlas.IsOccluded( shadow ) != (ray.hit.t < 1e4f)) occlusionErrors++;
			if (i == TINYBVH_BUNDLE_RAYS - 1)
			{
				// IntersectBundle traces these one by one; the results are those of Intersect.
				Ray single[TINYBVH_BUNDLE_RAYS];
				for (uint32_t k = 0; k < TINYBVH_BUNDLE_RAYS; k++) single[k] = bundle[k], tlas.Intersect( single[k] );
				tlas.IntersectBundle( bundle );
				bool occluded[TINYBVH_BUNDLE_RAYS];
				tlas.IsOccludedBundle( shadows, occluded );
				for (uint32_t k = 0; k < TINYBVH_BUNDLE_RAYS; k++)
				{
					if (bundle[k].hit.t != single[k].hit.t) bundleMismatches++;
					if (occluded[k] != (single[k].hit.t < 1e4f)) bundleMismatches++;
				}
			}
		}
	}
	printf( "  %u instances, %u rays, %.1f%% hit; mismatches vs brute force: %u in object space, %u in world space"
		" (%u edge grazes); %u occlusion errors, %u bundle mismatches\n", instCount, rays, 100.0f * hits / rays,
		mismatches, worldMismatches, edgeCases, occlusionErrors, bundleMismatches );
	CHECK( mismatches == 0, "TLAS and object space brute force disagree on %u rays", mismatches );
	CHECK( worldMismatches == 0, "TLAS and world space brute force disagree on %u rays", worldMismatches );
	CHECK( edgeCases * 100 <= rays, "%u edge grazes in %u rays: too many to be rounding", edgeCases, rays );
	CHECK( occlusionErrors == 0, "TLAS IsOccluded disagrees with Intersect on %u rays", occlusionErrors );
	CHECK( bundleMismatches == 0, "IntersectBundle / IsOccludedBundle differ from Intersect on %u rays", bundleMismatches );
	// a TLAS over this TLAS sees its motion too, and traces the same.
	BVH outer;
	BVHBase* inner[1] = { &tlas };
	BLASInstance whole( 0 );
	outer.Build( &whole, 1, inner, 1 );
	CHECK( outer.hasMotion, "TLAS over a moving TLAS not flagged" );
	uint32_t nestedMismatches = 0;
	for (uint32_t i = 0; i < 1000; i++)
	{
		Ray a = RandomRay( tlas.aabbMin, tlas.aabbMax, RandomFloat() ), b = a;
		tlas.Intersect( a ), outer.Intersect( b );
		if (!SameHit( a.hit.t, b.hit.t )) nestedMismatches++;
	}
	CHECK( nestedMismatches == 0, "nested TLAS disagrees on %u rays", nestedMismatches );
	// static instances of a motion blur BLAS: flagged; nothing that moves: not flagged.
	for (uint32_t i = 0; i < instCount; i++) inst[i].transform1 = 0;
	tlas.Build( inst, instCount, blases, 4 );
	CHECK( tlas.hasMotion, "TLAS with a BVH_Motion BLAS not flagged" );
	for (uint32_t i = 0; i < instCount; i++) inst[i].blasIdx = i % 3;
	tlas.Build( inst, instCount, blases, 4 );
	CHECK( !tlas.hasMotion, "TLAS without motion flagged as moving" );
}

#ifdef DOUBLE_PRECISION_SUPPORT

// SRT instance motion. The reference is independent of tiny_bvh: double precision, and the vector form of the
// quaternion rotation, v + 2 w (q x v) + 2 q x (q x v), instead of a matrix.
template <typename K> static void RefSRTPoint( const K* k, const double time, const double p[3], double out[3] )
{
	const double t = time, s = 1 - t;
	auto mix = [&]( const double a, const double b ) { return s * a + t * b; };
	const double sx = mix( k[0].sx, k[1].sx ), a = mix( k[0].a, k[1].a ), b = mix( k[0].b, k[1].b ), sy = mix( k[0].sy, k[1].sy );
	const double c = mix( k[0].c, k[1].c ), sz = mix( k[0].sz, k[1].sz );
	const double pv[3] = { mix( k[0].pvx, k[1].pvx ), mix( k[0].pvy, k[1].pvy ), mix( k[0].pvz, k[1].pvz ) };
	const double tr[3] = { mix( k[0].tx, k[1].tx ), mix( k[0].ty, k[1].ty ), mix( k[0].tz, k[1].tz ) };
	double q0[4] = { k[0].qx, k[0].qy, k[0].qz, k[0].qw }, q1[4] = { k[1].qx, k[1].qy, k[1].qz, k[1].qw };
	const double n0 = sqrt( q0[0] * q0[0] + q0[1] * q0[1] + q0[2] * q0[2] + q0[3] * q0[3] ), n1 = sqrt( q1[0] * q1[0] + q1[1] * q1[1] + q1[2] * q1[2] + q1[3] * q1[3] );
	for (int i = 0; i < 4; i++) q0[i] /= n0, q1[i] /= n1;
	const double d = tinybvh_clamp( q0[0] * q1[0] + q0[1] * q1[1] + q0[2] * q1[2] + q0[3] * q1[3], -1.0, 1.0 ), om = acos( d );
	const double w0 = sin( om ) > 1e-9 ? sin( s * om ) / sin( om ) : s, w1 = sin( om ) > 1e-9 ? sin( t * om ) / sin( om ) : t;
	double q[4];
	for (int i = 0; i < 4; i++) q[i] = w0 * q0[i] + w1 * q1[i];
	const double nq = sqrt( q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3] );
	for (int i = 0; i < 4; i++) q[i] /= nq;
	const double v[3] = { sx * p[0] + a * p[1] + b * p[2] + pv[0], sy * p[1] + c * p[2] + pv[1], sz * p[2] + pv[2] };
	const double u[3] = { q[1] * v[2] - q[2] * v[1], q[2] * v[0] - q[0] * v[2], q[0] * v[1] - q[1] * v[0] }; // q x v
	const double uu[3] = { q[1] * u[2] - q[2] * u[1], q[2] * u[0] - q[0] * u[2], q[0] * u[1] - q[1] * u[0] }; // q x (q x v)
	for (int i = 0; i < 3; i++) out[i] = v[i] + 2 * q[3] * u[i] + 2 * uu[i] + tr[i];
}
static bvhvec3 RefSRT( const BLASInstance::SRT* k, const float time, const bvhvec3& p )
{
	const double pd[3] = { p.x, p.y, p.z };
	double r[3];
	RefSRTPoint( k, time, pd, r );
	return bvhvec3( (float)r[0], (float)r[1], (float)r[2] );
}
// A key rotating by 'degrees' around 'axis', with scale, shear, pivot and translation.
static BLASInstance::SRT Key( const bvhvec3& axis, const float degrees, const bvhvec3& scale, const bvhvec3& shear, const bvhvec3& pivot, const bvhvec3& pos )
{
	BLASInstance::SRT k;
	const bvhvec3 n = tinybvh_normalize( axis );
	const float h = degrees * 3.14159265f / 360;
	k.sx = scale.x, k.sy = scale.y, k.sz = scale.z, k.a = shear.x, k.b = shear.y, k.c = shear.z;
	k.pvx = pivot.x, k.pvy = pivot.y, k.pvz = pivot.z, k.tx = pos.x, k.ty = pos.y, k.tz = pos.z;
	k.qx = n.x * sinf( h ), k.qy = n.y * sinf( h ), k.qz = n.z * sinf( h ), k.qw = cosf( h );
	return k;
}
// Triangles in a ball of radius 'r' around 'center'.
static std::vector<bvhvec4> Cluster( const uint32_t n, const bvhvec3& center, const float r )
{
	std::vector<bvhvec4> v;
	for (uint32_t i = 0; i < n; i++)
	{
		const bvhvec3 p = center + RandomDir() * (r * cbrtf( RandomFloat() ));
		for (int k = 0; k < 3; k++) v.push_back( bvhvec4( p + RandomDir() * (r * 0.08f), 0 ) );
	}
	return v;
}
static void TestSRT()
{
	printf( "SRT instance motion:\n" );
	// the key evaluation against the reference, its inverse, and the arc: a quarter turn keeps a point on its circle,
	// where interpolating the matrices would cut the corner to cos( 45 ) of the radius.
	const BLASInstance::SRT turn[2] = { Key( bvhvec3( 0, 1, 0 ), 0, bvhvec3( 1 ), bvhvec3( 0 ), bvhvec3( 0 ), bvhvec3( 0 ) ),
		Key( bvhvec3( 0, 1, 0 ), 90, bvhvec3( 1 ), bvhvec3( 0 ), bvhvec3( 0 ), bvhvec3( 0 ) ) };
	const BLASInstance::SRT odd[2] = { Key( bvhvec3( 1, 2, 3 ), 20, bvhvec3( 1, 2, 0.5f ), bvhvec3( 0.3f, -0.2f, 0.1f ), bvhvec3( 1, 0, -2 ), bvhvec3( 5, 6, 7 ) ),
		Key( bvhvec3( 1, 2, 3 ), 290, bvhvec3( 2, 1, 1.5f ), bvhvec3( -0.1f, 0.4f, 0 ), bvhvec3( 0, 3, 1 ), bvhvec3( -4, 2, 9 ) ) };
	BLASInstance probe( 0 );
	float keyError = 0, inverseError = 0;
	for (int i = 0; i < 1000; i++)
	{
		probe.srt = (i & 1) ? odd : turn;
		const float time = RandomFloat();
		const bvhvec3 p( RandomFloat() * 4 - 2, RandomFloat() * 4 - 2, RandomFloat() * 4 - 2 );
		const bvhvec3 a = tinybvh_transform_point( p, probe.TransformAt( time ) ), r = RefSRT( probe.srt, time, p );
		keyError = tinybvh_max( keyError, tinybvh_length( a - r ) / (1 + tinybvh_length( r )) );
		const bvhvec3 back = tinybvh_transform_point( a, probe.InvTransformAt( time ) );
		inverseError = tinybvh_max( inverseError, tinybvh_length( back - p ) / (1 + tinybvh_length( p )) );
	}
	probe.srt = turn;
	const float radius = tinybvh_length( tinybvh_transform_point( bvhvec3( 2, 0, 0 ), probe.TransformAt( 0.5f ) ) ) / 2;
	printf( "  keys vs reference: error %.2e; inverse: %.2e; radius halfway a quarter turn: %.6f\n", keyError, inverseError, radius );
	CHECK( keyError < 1e-5f, "SRT keys evaluate %e away from the reference", keyError );
	CHECK( inverseError < 1e-5f, "InvTransformAt does not invert TransformAt (%e)", inverseError );
	CHECK( fabsf( radius - 1 ) < 1e-5f, "a quarter turn left its circle: radius %f", radius );
	// a TLAS over a static BVH and a BVH_Motion, with SRT instances, against brute force in world space.
	const std::vector<bvhvec4> c0 = Cluster( 1500, bvhvec3( 0.5f, 0, 0 ), 1 ), c1 = Animate( c0, 0.05f );
	BVH still;
	BVH_Motion moving;
	still.Build( c0.data(), 1500 );
	moving.Build( c0.data(), c1.data(), 1500 );
	BVHBase* blases[2] = { &still, &moving };
	const BLASInstance::SRT keys[6][2] = {
		{ Key( bvhvec3( 0, 1, 0 ), 0, bvhvec3( 1 ), bvhvec3( 0 ), bvhvec3( 0 ), bvhvec3( 0, 0, 0 ) ), Key( bvhvec3( 0, 1, 0 ), 90, bvhvec3( 1 ), bvhvec3( 0 ), bvhvec3( 0 ), bvhvec3( 0, 0, 0 ) ) },
		{ Key( bvhvec3( 1, 1, 0 ), 10, bvhvec3( 1 ), bvhvec3( 0 ), bvhvec3( -0.5f, 0, 0 ), bvhvec3( 6, 0, 0 ) ), Key( bvhvec3( 1, 1, 0 ), 280, bvhvec3( 1.3f ), bvhvec3( 0 ), bvhvec3( -0.5f, 0, 0 ), bvhvec3( 6, 1, 0 ) ) },
		{ Key( bvhvec3( 0, 0, 1 ), 0, bvhvec3( 1, 2, 1 ), bvhvec3( 0.4f, 0, 0.2f ), bvhvec3( 0 ), bvhvec3( 0, 6, 0 ) ), Key( bvhvec3( 0, 0, 1 ), 45, bvhvec3( 2, 1, 1 ), bvhvec3( -0.3f, 0.2f, 0 ), bvhvec3( 0 ), bvhvec3( 0, 6, 1 ) ) },
		{ Key( bvhvec3( 0, 1, 0 ), 0, bvhvec3( 1 ), bvhvec3( 0 ), bvhvec3( 0 ), bvhvec3( -6, 0, 0 ) ), Key( bvhvec3( 0, 1, 0 ), 0, bvhvec3( 1 ), bvhvec3( 0 ), bvhvec3( 0 ), bvhvec3( -4, 0, 2 ) ) },
		{ Key( bvhvec3( 3, 1, 2 ), 30, bvhvec3( 1 ), bvhvec3( 0 ), bvhvec3( 0 ), bvhvec3( 0, -6, 0 ) ), Key( bvhvec3( 3, 1, 2 ), 30, bvhvec3( 1 ), bvhvec3( 0 ), bvhvec3( 0 ), bvhvec3( 0, -6, 0 ) ) },
		{ Key( bvhvec3( 0, 1, 1 ), -40, bvhvec3( 1 ), bvhvec3( 0 ), bvhvec3( 0.5f, 0.5f, 0 ), bvhvec3( 6, 6, 0 ) ), Key( bvhvec3( 0, 1, 1 ), 120, bvhvec3( 0.7f ), bvhvec3( 0 ), bvhvec3( 0.5f, 0.5f, 0 ), bvhvec3( 6, 6, 3 ) ) } };
	BLASInstance inst[12];
	for (uint32_t i = 0; i < 12; i++) inst[i] = BLASInstance( i & 1 ), inst[i].srt = keys[i >> 1];
	BVH tlas;
	tlas.Build( inst, 12, blases, 2 );
	CHECK( tlas.hasMotion, "TLAS with SRT instances not flagged" );
	// the bounds of every instance hold its triangles at any time; the slack is the margin over the swept extent.
	uint32_t outside = 0;
	float slack = 0, padSlack = 0;
	for (uint32_t i = 0; i < 12; i++)
	{
		bvhvec3 smin( BVH_FAR ), smax( -BVH_FAR );
		for (int s = 0; s < 64; s++)
		{
			const float time = s == 0 ? 0 : s == 1 ? 1 : RandomFloat();
			for (size_t j = 0; j < c0.size(); j++)
			{
				const bvhvec3 p = RefSRT( inst[i].srt, time, (i & 1) ? At( c0[j], c1[j], time ) : bvhvec3( c0[j] ) );
				const float eps = 1e-5f * (1 + Scale( p ));
				if (p.x < inst[i].aabbMin.x - eps || p.y < inst[i].aabbMin.y - eps || p.z < inst[i].aabbMin.z - eps ||
					p.x > inst[i].aabbMax.x + eps || p.y > inst[i].aabbMax.y + eps || p.z > inst[i].aabbMax.z + eps) outside++;
				smin = tinybvh_min( smin, p ), smax = tinybvh_max( smax, p );
			}
		}
		const bvhvec3 e = smax - smin, b = inst[i].aabbMax - inst[i].aabbMin;
		slack = tinybvh_max( slack, tinybvh_max( b.x / e.x, tinybvh_max( b.y / e.y, b.z / e.z ) ) - 1 );
		// what the padding adds: the margin over the paths of the corners of the BLAS box, which the bounds follow.
		const BVHBase* blas = blases[inst[i].blasIdx];
		bvhvec3 cmin( BVH_FAR ), cmax( -BVH_FAR );
		for (int s = 0; s <= 4096; s++) for (int j = 0; j < 8; j++)
		{
			const bvhvec3 p( j & 1 ? blas->aabbMax.x : blas->aabbMin.x, j & 2 ? blas->aabbMax.y : blas->aabbMin.y, j & 4 ? blas->aabbMax.z : blas->aabbMin.z );
			const bvhvec3 w = RefSRT( inst[i].srt, s / 4096.0f, p );
			cmin = tinybvh_min( cmin, w ), cmax = tinybvh_max( cmax, w );
		}
		const bvhvec3 ce = cmax - cmin;
		padSlack = tinybvh_max( padSlack, tinybvh_max( b.x / ce.x, tinybvh_max( b.y / ce.y, b.z / ce.z ) ) - 1 );
	}
	uint32_t rays = 0, hits = 0, mismatches = 0, grazes = 0, bundleMismatches = 0;
	for (int s = 0; s < 8; s++)
	{
		const float time = s == 0 ? 0 : s == 1 ? 1 : RandomFloat();
		std::vector<bvhvec4> world;
		for (uint32_t i = 0; i < 12; i++) for (size_t j = 0; j < c0.size(); j++)
			world.push_back( bvhvec4( RefSRT( inst[i].srt, time, (i & 1) ? At( c0[j], c1[j], time ) : bvhvec3( c0[j] ) ), 0 ) );
		Ray bundle[TINYBVH_BUNDLE_RAYS];
		for (uint32_t i = 0; i < 300; i++, rays++)
		{
			Ray ray = RandomRay( tlas.aabbMin, tlas.aabbMax, time );
			if (i & 1) // half of the rays aim at a vertex of the scene at this time.
			{
				const bvhvec3 target = bvhvec3( world[RandomUInt() % world.size()] ) + RandomDir() * 0.02f;
				ray = Ray( ray.O, tinybvh_normalize( target - ray.O ), BVH_FAR, RAY_MASK_INTERSECT_ALL, time );
			}
			ray.hit.t = 1e4f;
			if (i < TINYBVH_BUNDLE_RAYS) bundle[i] = ray;
			tlas.Intersect( ray );
			float u = 0, v = 0;
			const float ref = BruteForce( ray.O, ray.D, world, 1e4f, &u, &v );
			if (ray.hit.t < 1e4f) hits++;
			if (!SameHit( ray.hit.t, ref, Scale( ray.O ) + ref ))
			{
				const bool tlasNearer = ray.hit.t < ref;
				if (NearEdge( tlasNearer ? ray.hit.u : u, tlasNearer ? ray.hit.v : v, 1e-3f )) grazes++; else mismatches++;
			}
		}
		Ray single[TINYBVH_BUNDLE_RAYS];
		for (uint32_t k = 0; k < TINYBVH_BUNDLE_RAYS; k++) single[k] = bundle[k], tlas.Intersect( single[k] );
		tlas.IntersectBundle( bundle );
		for (uint32_t k = 0; k < TINYBVH_BUNDLE_RAYS; k++) if (bundle[k].hit.t != single[k].hit.t) bundleMismatches++;
	}
	printf( "  12 instances, %u rays, %.1f%% hit; %u mismatches vs brute force (%u edge grazes), %u bundle mismatches\n",
		rays, 100.0f * hits / rays, mismatches, grazes, bundleMismatches );
	printf( "  bounds: %u vertices outside; at most %.1f%% wider than the swept triangles, %.2f%% wider than the swept BLAS box\n",
		outside, slack * 100, padSlack * 100 );
	CHECK( padSlack < 0.02f, "SRT bounds %.2f%% wider than the swept BLAS box", padSlack * 100 );
	CHECK( mismatches == 0, "SRT TLAS and brute force disagree on %u rays", mismatches );
	CHECK( grazes * 100 <= rays, "%u edge grazes in %u rays: too many to be rounding", grazes, rays );
	CHECK( bundleMismatches == 0, "IntersectBundle differs from Intersect on %u rays", bundleMismatches );
	CHECK( outside == 0, "%u vertices outside the bounds of their instance", outside );
	// translation only: the same motion as two matrices.
	const BLASInstance::SRT slide[2] = { Key( bvhvec3( 0, 1, 0 ), 25, bvhvec3( 1 ), bvhvec3( 0 ), bvhvec3( 0 ), bvhvec3( 1, 2, 3 ) ),
		Key( bvhvec3( 0, 1, 0 ), 25, bvhvec3( 1 ), bvhvec3( 0 ), bvhvec3( 0 ), bvhvec3( 4, 1, -2 ) ) };
	const bvhmat4 end = slide[1].Matrix();
	BLASInstance bySRT( 0 ), byMatrix( 0 );
	bySRT.srt = slide, byMatrix.transform = slide[0].Matrix(), byMatrix.transform1 = &end;
	BVH srtTLAS, matrixTLAS;
	srtTLAS.Build( &bySRT, 1, blases, 2 ), matrixTLAS.Build( &byMatrix, 1, blases, 2 );
	uint32_t different = 0;
	for (uint32_t i = 0; i < 2000; i++)
	{
		Ray a = RandomRay( srtTLAS.aabbMin, srtTLAS.aabbMax, RandomFloat() ), b = a;
		srtTLAS.Intersect( a ), matrixTLAS.Intersect( b );
		if (!SameHit( a.hit.t, b.hit.t ) && !NearEdge( a.hit.t < b.hit.t ? a.hit.u : b.hit.u, a.hit.t < b.hit.t ? a.hit.v : b.hit.v, 1e-4f )) different++;
	}
	printf( "  translation only: %u of 2000 rays differ from the same motion as matrices\n", different );
	CHECK( different == 0, "SRT translation and matrix motion disagree on %u rays", different );
#ifdef DOUBLE_PRECISION_SUPPORT
	// double precision: a TLAS of one turning BLASInstanceEx against brute force.
	std::vector<bvhdbl3> d0;
	for (size_t j = 0; j < c0.size(); j++) d0.push_back( bvhdbl3( c0[j].x, c0[j].y, c0[j].z ) );
	BVH_Double dblas;
	dblas.Build( d0.data(), 1500 );
	BLASInstanceEx::SRT dk[2];
	for (int k = 0; k < 2; k++)
	{
		const BLASInstance::SRT& f = keys[1][k];
		dk[k].sx = f.sx, dk[k].a = f.a, dk[k].b = f.b, dk[k].pvx = f.pvx, dk[k].sy = f.sy, dk[k].c = f.c, dk[k].pvy = f.pvy, dk[k].sz = f.sz;
		dk[k].pvz = f.pvz, dk[k].qx = f.qx, dk[k].qy = f.qy, dk[k].qz = f.qz, dk[k].qw = f.qw, dk[k].tx = f.tx, dk[k].ty = f.ty, dk[k].tz = f.tz;
	}
	BLASInstanceEx dinst( 0 );
	dinst.srt = dk;
	BVHBaseEx* dblases[1] = { &dblas };
	BVH_Double dtlas;
	dtlas.Build( &dinst, 1, dblases, 1 );
	uint32_t dmismatches = 0;
	for (int i = 0; i < 400; i++)
	{
		const float time = RandomFloat();
		const Ray r = RandomRay( bvhvec3( (float)dtlas.aabbMin.x, (float)dtlas.aabbMin.y, (float)dtlas.aabbMin.z ), bvhvec3( (float)dtlas.aabbMax.x, (float)dtlas.aabbMax.y, (float)dtlas.aabbMax.z ), time );
		RayEx ray( bvhdbl3( r.O.x, r.O.y, r.O.z ), bvhdbl3( r.D.x, r.D.y, r.D.z ), 1e4, RAY_MASK_INTERSECT_ALL, time );
		dtlas.Intersect( ray );
		double ref = 1e4;
		for (size_t j = 0; j < d0.size(); j += 3)
		{
			bvhdbl3 w[3];
			for (int k = 0; k < 3; k++)
			{
				const double p[3] = { d0[j + k].x, d0[j + k].y, d0[j + k].z };
				double o[3];
				RefSRTPoint( dk, time, p, o );
				w[k] = bvhdbl3( o[0], o[1], o[2] );
			}
			TriTest( ray.O, ray.D, w[0], w[1], w[2], ref, ref );
		}
		if (!SameHit( ray.hit.t, ref )) dmismatches++;
	}
	printf( "  double precision: %u mismatches in 400 rays\n", dmismatches );
	CHECK( dmismatches == 0, "double SRT TLAS and brute force disagree on %u rays", dmismatches );
#endif
}

// Double precision: BVH_Motion_Double, alone and in a TLAS of BLASInstanceEx.
static void TestDouble( const std::vector<bvhvec4>& scene )
{
	printf( "double precision:\n" );
	const uint32_t n = tinybvh_min( (uint32_t)(scene.size() / 3), 20000u );
	const std::vector<bvhvec4> f1 = Animate( std::vector<bvhvec4>( scene.begin(), scene.begin() + n * 3 ), 0.5f );
	std::vector<bvhdbl3> v0( n * 3 ), v1( n * 3 );
	for (uint32_t i = 0; i < n * 3; i++) v0[i] = bvhdbl3( scene[i].x, scene[i].y, scene[i].z ), v1[i] = bvhdbl3( f1[i].x, f1[i].y, f1[i].z );
	BVH_Motion_Double m;
	BVH_Double still;
	m.Build( v0.data(), v1.data(), n );
	still.Build( v0.data(), n );
	BVHBaseEx* blases[2] = { &still, &m };
	BLASInstanceEx inst[4];
	bvhdblmat4 end[4];
	for (uint32_t i = 0; i < 4; i++)
	{
		inst[i] = BLASInstanceEx( i & 1 );
		inst[i].transform[3] = 100.0 * i;
		if (i >= 2) end[i] = inst[i].transform, end[i][3] += 5, end[i][7] += 2, inst[i].transform1 = &end[i];
	}
	BVH_Double tlas;
	tlas.Build( inst, 4, blases, 2 );
	uint32_t rays = 0, hits = 0, mismatches = 0, tlasMismatches = 0;
	for (int s = 0; s < 4; s++)
	{
		const double t1 = s == 0 ? 0 : s == 1 ? 1 : (double)RandomFloat(), t0 = 1 - t1;
		std::vector<bvhdbl3> snap( n * 3 ), world;
		for (uint32_t i = 0; i < n * 3; i++) snap[i] = tinybvh_motion_lerp( v0[i], v1[i], t0, t1 );
		for (uint32_t i = 0; i < 4; i++)
		{
			bvhdblmat4 M = inst[i].transform;
			if (inst[i].transform1) for (int k = 0; k < 16; k++) M[k] = tinybvh_motion_lerp( inst[i].transform[k], (*inst[i].transform1)[k], t0, t1 );
			for (uint32_t j = 0; j < n * 3; j++) world.push_back( tinybvh_transform_point( (i & 1) ? snap[j] : v0[j], M ) );
		}
		for (uint32_t i = 0; i < 300; i++, rays++)
		{
			const Ray r = RandomRay( bvhvec3( (float)m.aabbMin.x, (float)m.aabbMin.y, (float)m.aabbMin.z ), bvhvec3( (float)m.aabbMax.x, (float)m.aabbMax.y, (float)m.aabbMax.z ), (float)t1 );
			RayEx ray( bvhdbl3( r.O.x, r.O.y, r.O.z ), bvhdbl3( r.D.x, r.D.y, r.D.z ), 1e4, RAY_MASK_INTERSECT_ALL, (float)t1 );
			RayEx world_ray = ray;
			m.Intersect( ray );
			double ref = 1e4;
			for (uint32_t j = 0; j < n * 3; j += 3) TriTest( ray.O, ray.D, snap[j], snap[j + 1], snap[j + 2], ref, ref );
			if (!SameHit( ray.hit.t, ref )) mismatches++;
			if (ray.hit.t < 1e4) hits++;
			tlas.Intersect( world_ray );
			double worldRef = 1e4;
			for (size_t j = 0; j < world.size(); j += 3) TriTest( world_ray.O, world_ray.D, world[j], world[j + 1], world[j + 2], worldRef, worldRef );
			if (!SameHit( world_ray.hit.t, worldRef )) tlasMismatches++;
		}
	}
	printf( "  %u rays, %.1f%% hit; %u mismatches vs brute force, %u through the TLAS\n", rays, 100.0f * hits / rays, mismatches, tlasMismatches );
	CHECK( mismatches == 0, "BVH_Motion_Double and brute force disagree on %u rays", mismatches );
	CHECK( tlasMismatches == 0, "double TLAS and brute force disagree on %u rays", tlasMismatches );
}

#endif

// The Ray layout of option 1: time in the last four bytes, the user union four bytes shorter.
static void TestLayout()
{
	const Ray r( bvhvec3( 0 ), bvhvec3( 0, 0, 1 ) );
	const size_t timeAt = (size_t)((const char*)&r.time - (const char*)&r);
	printf( "layout: sizeof( Ray ) %zu, time at %zu; sizeof( Intersection ) %zu; sizeof( BVH_Motion::BVHNode ) %zu\n",
		sizeof( Ray ), timeAt, sizeof( Intersection ), sizeof( BVH_Motion::BVHNode ) );
	CHECK( sizeof( Ray ) == 128 && timeAt == 124, "unexpected Ray layout" );
	CHECK( sizeof( BVH_Motion::BVHNode ) == 64, "unexpected BVH_Motion node size" );
	CHECK( sizeof( BLASInstance ) == 192, "unexpected BLASInstance size" );
	CHECK( r.time == 0, "default ray time is not 0" );
#ifdef DOUBLE_PRECISION_SUPPORT
	const RayEx rx( bvhdbl3( 0 ), bvhdbl3( 0, 0, 1 ) );
	CHECK( sizeof( RayEx ) == 192 && (size_t)((const char*)&rx.time - (const char*)&rx) == 188, "unexpected RayEx layout" );
#endif
}

// Build times, and the throughput of the same rays through BVH and BVH_Motion, and through a
// TLAS over a static and a moving instance of the BVH; single-threaded traversal. Primary rays
// use the camera of the speedtest.
static void Benchmark( const std::vector<bvhvec4>& v0 )
{
	const uint32_t n = (uint32_t)(v0.size() / 3), count = 480 * 320 * 4;
	const std::vector<bvhvec4> v1 = Animate( v0, 0.5f ), v3 = Animate( v0, 3.0f );
	BVH bvh, staticTLAS, movingTLAS;
	BVH_Motion still, moving, fast;
	// build times: the best of three, in milliseconds. BVH uses its default builder.
	auto timed = [&]( auto build )
	{
		double best = 1e30;
		for (int i = 0; i < 3; i++)
		{
			const auto t0 = std::chrono::high_resolution_clock::now();
			build();
			best = tinybvh_min( best, std::chrono::duration<double>( std::chrono::high_resolution_clock::now() - t0 ).count() * 1e3 );
		}
		return best;
	};
	const double buildBVH = timed( [&]() { bvh.Build( v0.data(), n ); } );
	const double buildStill = timed( [&]() { still.Build( v0.data(), v0.data(), n ); } );
	const double buildMoving = timed( [&]() { moving.Build( v0.data(), v1.data(), n ); } );
	const double buildFast = timed( [&]() { fast.Build( v0.data(), v3.data(), n ); } );
	BVH_Motion single;
	single.settings.enableThreading = false;
	const double buildSingle = timed( [&]() { single.Build( v0.data(), v1.data(), n ); } );
	printf( "build (ms): BVH %.1f; BVH_Motion: static keys %.1f, amplitude 0.5 %.1f (single-threaded %.1f), amplitude 3.0 %.1f\n",
		buildBVH, buildStill, buildMoving, buildSingle, buildFast );
	printf( "performance (single thread, Mrays/s):         primary  incoherent\n" );
	BVHBase* blases[1] = { &bvh };
	BLASInstance staticInst( 0 ), movingInst( 0 );
	bvhmat4 shifted;
	shifted[3] = 0.5f, shifted[7] = 0.2f;
	movingInst.transform1 = &shifted;
	staticTLAS.Build( &staticInst, 1, blases, 1 );
	movingTLAS.Build( &movingInst, 1, blases, 1 );
	// the same shift, with a small turn around the scene's vertical axis, as SRT keys.
	BLASInstance srtInst( 0 );
	BLASInstance::SRT turn[2];
	turn[1].tx = 0.5f, turn[1].ty = 0.2f, turn[1].qy = sinf( 0.01f ), turn[1].qw = cosf( 0.01f );
	srtInst.srt = turn;
	BVH srtTLAS;
	srtTLAS.Build( &srtInst, 1, blases, 1 );
	std::vector<Ray> primary( count ), incoherent( count );
	const bvhvec3 eye( -15.24f, 21.5f, 2.54f ), view = tinybvh_normalize( bvhvec3( 0.826f, -0.438f, -0.356f ) );
	const bvhvec3 right = tinybvh_normalize( tinybvh_cross( bvhvec3( 0, 1, 0 ), view ) ), up = 0.8f * tinybvh_cross( view, right );
	const bvhvec3 C = eye + 2 * view, p1 = C - right + up, p2 = C + right + up, p3 = C - right - up;
	for (uint32_t i = 0; i < count; i++)
	{
		const uint32_t x = (i >> 2) % 480, y = (i >> 2) / 480;
		const float u = (x * 2 + (i & 1)) / 960.0f, v = (y * 2 + ((i >> 1) & 1)) / 640.0f;
		const bvhvec3 P = p1 + u * (p2 - p1) + v * (p3 - p1);
		primary[i] = Ray( eye, tinybvh_normalize( P - eye ), BVH_FAR, RAY_MASK_INTERSECT_ALL, RandomFloat() );
		incoherent[i] = RandomRay( bvh.aabbMin, bvh.aabbMax, RandomFloat() );
	}
	auto run = [&]( const char* name, auto& b )
	{
		double mrays[2];
		for (int set = 0; set < 2; set++)
		{
			std::vector<Ray> r = set ? incoherent : primary;
			const auto t0 = std::chrono::high_resolution_clock::now();
			for (uint32_t i = 0; i < count; i++) b.Intersect( r[i] );
			mrays[set] = count / std::chrono::duration<double>( std::chrono::high_resolution_clock::now() - t0 ).count() * 1e-6;
		}
		printf( "  %-42s %7.2f %10.2f\n", name, mrays[0], mrays[1] );
	};
	run( "BVH", bvh );
	run( "BVH_Motion, static keys", still );
	run( "BVH_Motion, moving (amplitude 0.5)", moving );
	run( "BVH_Motion, moving (amplitude 3.0)", fast );
	run( "TLAS over the BVH, static instance", staticTLAS );
	run( "TLAS over the BVH, moving instance", movingTLAS );
	run( "TLAS over the BVH, SRT instance", srtTLAS );
}

int main( int argc, char** argv )
{
	const char* fileName = argc > 1 ? argv[1] : "./testdata/cryteksponza.bin";
	std::vector<bvhvec4> scene;
	if (LoadScene( fileName, scene )) printf( "scene: %s, %zu triangles\n", fileName, scene.size() / 3 );
	else ProceduralScene( scene, 100000 ), printf( "scene: %s not found, using %zu procedural triangles\n", fileName, scene.size() / 3 );
	TestLayout();
	TestStatic( scene );
	TestMotion( scene, 0.5f, 20000, 40 );
	TestMotion( scene, 3.0f, 5000, 10 );
	TestThreading( scene );
	TestIndexed( scene );
	TestRefit( scene );
	TestDegenerate();
	TestMove( scene );
	TestTLAS( scene );
	TestSRT();
#ifdef DOUBLE_PRECISION_SUPPORT
	TestDouble( scene );
#endif
	if (argc < 3) Benchmark( scene );
	printf( failures ? "%i check(s) FAILED.\n" : "All checks passed.\n", failures );
	return failures ? 1 : 0;
}
