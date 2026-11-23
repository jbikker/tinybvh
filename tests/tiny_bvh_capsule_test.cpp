// Validation of BVH::IntersectCapsule.
//
// A capsule is the set of points within r of segment p0-p1. Each result is
// checked against spheres placed along the segment with IntersectSphere:
// if a sphere of radius r hits, the capsule must hit; if the capsule hits,
// a sphere grown by half the sample spacing must hit. Degenerate capsules
// (p0 == p1) must agree with IntersectSphere exactly.
//
// Usage: tiny_bvh_capsule_test [scene] - default ./testdata/cryteksponza.bin.

#define TINYBVH_IMPLEMENTATION
#include "tiny_bvh.h"
#include <cstdio>
#include <vector>
#include <chrono>

using namespace tinybvh;

static uint32_t seed = 0x12345678;
static float Rand() { seed ^= seed << 13, seed ^= seed >> 17, seed ^= seed << 5; return (seed >> 8) * (1.0f / 16777216.0f); }

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

int main( int argc, char** argv )
{
	const char* fileName = argc > 1 ? argv[1] : "./testdata/cryteksponza.bin";
	std::vector<bvhvec4> scene;
	if (!LoadScene( fileName, scene )) { printf( "could not load %s\n", fileName ); return 1; }
	printf( "scene: %s, %zu triangles\n", fileName, scene.size() / 3 );
	BVH bvh;
	bvh.Build( scene.data(), (uint32_t)scene.size() / 3 );
	const bvhvec3 bmin = bvh.bvhNode[0].aabbMin, ext = bvh.bvhNode[0].aabbMax - bmin;
	const float diag = tinybvh_length( ext );
	int failures = 0, hits = 0;
	const int N = 20000, S = 256;
	for (int i = 0; i < N; i++)
	{
		const bvhvec3 p0 = bmin + bvhvec3( Rand() * ext.x, Rand() * ext.y, Rand() * ext.z );
		const bvhvec3 dir = tinybvh_normalize( bvhvec3( Rand() - 0.5f, Rand() - 0.5f, Rand() - 0.5f ) );
		const float len = diag * (i & 1 ? 0.002f : 0.03f) * Rand(), r = diag * 0.004f * Rand() + 1e-4f;
		const bvhvec3 p1 = p0 + dir * len;
		const bool hit = bvh.IntersectCapsule( p0, p1, r ), point = i % 11 == 0;
		if (point)
		{
			if (bvh.IntersectCapsule( p0, p0, r ) != bvh.IntersectSphere( p0, r )) failures++, printf( "degenerate mismatch %i\n", i );
			continue;
		}
		const float halfStep = 0.5f * len / (S - 1);
		bool lower = false, upper = false;
		for (int s = 0; s < S && !lower; s++)
		{
			const bvhvec3 c = p0 + (p1 - p0) * (s / (float)(S - 1));
			if (bvh.IntersectSphere( c, r )) lower = upper = true;
			else if (!upper && bvh.IntersectSphere( c, r + halfStep * 1.001f + 1e-5f )) upper = true;
		}
		if (lower && !hit) failures++, printf( "missed hit %i\n", i );
		if (hit && !upper) failures++, printf( "false hit %i\n", i );
		hits += hit;
	}
	printf( "%i capsules, %i hits\n", N, hits );
	// timing
	std::vector<bvhvec3> P0( N ), P1( N );
	std::vector<float> R( N );
	for (int i = 0; i < N; i++)
	{
		P0[i] = bmin + bvhvec3( Rand() * ext.x, Rand() * ext.y, Rand() * ext.z );
		P1[i] = P0[i] + tinybvh_normalize( bvhvec3( Rand() - 0.5f, Rand() - 0.5f, Rand() - 0.5f ) ) * diag * 0.01f;
		R[i] = diag * 0.003f;
	}
	auto t0 = std::chrono::high_resolution_clock::now();
	int c1 = 0, c2 = 0;
	for (int i = 0; i < N; i++) c1 += bvh.IntersectCapsule( P0[i], P1[i], R[i] );
	auto t1 = std::chrono::high_resolution_clock::now();
	for (int i = 0; i < N; i++) c2 += bvh.IntersectSphere( P0[i], R[i] );
	auto t2 = std::chrono::high_resolution_clock::now();
	printf( "capsule: %.3f us/query (%i hits), sphere: %.3f us/query (%i hits)\n",
		std::chrono::duration<double, std::micro>( t1 - t0 ).count() / N, c1,
		std::chrono::duration<double, std::micro>( t2 - t1 ).count() / N, c2 );
#ifdef DOUBLE_PRECISION_SUPPORT
	{
		std::vector<bvhdbl3> dv( scene.size() );
		for (size_t i = 0; i < scene.size(); i++) dv[i] = bvhdbl3( scene[i].x, scene[i].y, scene[i].z );
		BVH_Double dbvh;
		dbvh.Build( dv.data(), (uint64_t)dv.size() / 3 );
		int mismatches = 0;
		for (int i = 0; i < 2000; i++)
		{
			const bvhdbl3 a( P0[i].x, P0[i].y, P0[i].z ), b( P1[i].x, P1[i].y, P1[i].z );
			if (dbvh.IntersectCapsule( a, b, R[i] ) != bvh.IntersectCapsule( P0[i], P1[i], R[i] )) mismatches++;
		}
		printf( "double vs float mismatches: %i / 2000 (boundary cases only expected)\n", mismatches );
	}
#endif
	printf( failures ? "%i check(s) FAILED.\n" : "All checks passed.\n", failures );
	return failures ? 1 : 0;
}
