// BVH_GPU against BVH: trees whose root is a leaf (a fresh object, a rebuild, ConvertFrom), and leaves of more
// than 127 triangles, the most a BVH_GPU child reference holds.
// Usage: tiny_bvh_gpu_test [scene] - default ./testdata/cryteksponza.bin.

#define TINYBVH_IMPLEMENTATION
#include "tiny_bvh.h"
#include <cstdio>
#include <cstdint>
#include <vector>

using namespace tinybvh;

static int failures = 0;
static uint32_t seed = 0x2468ace;
static float Rand() { seed ^= seed << 13, seed ^= seed >> 17, seed ^= seed << 5; return (seed >> 8) * (1.0f / 16777216.0f); }

// rays from around 'from' toward random points in the box [lo, hi]
static std::vector<Ray> Rays( const bvhvec3 from, const bvhvec3 lo, const bvhvec3 hi, const uint32_t count )
{
	std::vector<Ray> rays;
	for (uint32_t i = 0; i < count; i++)
	{
		const bvhvec3 O = from + bvhvec3( Rand() - 0.5f, Rand() - 0.5f, Rand() - 0.5f ) * 0.1f;
		const bvhvec3 P = lo + (hi - lo) * bvhvec3( Rand(), Rand(), Rand() );
		rays.push_back( Ray( O, tinybvh_normalize( P - O ) ) );
	}
	return rays;
}

// the same hit distance (up to rounding) or the same miss, for every ray
static void Compare( const char* label, const BVH& ref, const BVH_GPU& gpu, const std::vector<Ray>& rays, const bool expectHits = true )
{
	int hits = 0, differ = 0;
	for (const Ray& r : rays)
	{
		Ray a = r, b = r;
		ref.Intersect( a ), gpu.Intersect( b );
		const bool hit = a.hit.t < BVH_FAR;
		hits += hit ? 1 : 0;
		if (hit ? !(b.hit.t <= a.hit.t * 1.00001f + 1e-6f && b.hit.t >= a.hit.t * 0.99999f - 1e-6f) : b.hit.t < BVH_FAR) differ++;
	}
	const bool ok = differ == 0 && (hits > 0) == expectHits;
	printf( "%s: %i of %zu rays hit, %i differ%s\n", label, hits, rays.size(), differ, ok ? "" : " - FAILED" );
	if (!ok) failures++;
}

static uint32_t LargestLeaf( const BVH& b )
{
	uint32_t largest = 0;
	for (uint32_t i = 0; i < b.usedNodes; i++) if (i != 1 && b.bvhNode[i].isLeaf()) largest = tinybvh_max( largest, (uint32_t)b.bvhNode[i].triCount );
	return largest;
}

int main( int argc, char** argv )
{
	const bvhvec4 tri[3] = { bvhvec4( 0, 0, 0, 0 ), bvhvec4( 1, 0, 0, 0 ), bvhvec4( 0, 1, 0, 0 ) };
	const std::vector<Ray> triRays = Rays( bvhvec3( 0.3f, 0.3f, -2 ), bvhvec3( -0.5f, -0.5f, 0 ), bvhvec3( 1.5f, 1.5f, 0 ), 2000 );
	// one triangle: the root is a leaf
	{
		BVH_GPU gpu;
		gpu.Build( tri, 1 );
		Compare( "Build, 1 triangle", gpu.bvh, gpu, triRays );
		Compare( "Build, 1 triangle, rays that miss its box", gpu.bvh, gpu, Rays( bvhvec3( 5, 5, -2 ), bvhvec3( 4, 4, 0 ), bvhvec3( 6, 6, 0 ), 100 ), false );
	}
	// a quad of two triangles: a root leaf too
	{
		const bvhvec4 quad[6] = { bvhvec4( -1, 0, -1, 0 ), bvhvec4( 1, 0, -1, 0 ), bvhvec4( 1, 0, 1, 0 ),
			bvhvec4( -1, 0, -1, 0 ), bvhvec4( 1, 0, 1, 0 ), bvhvec4( -1, 0, 1, 0 ) };
		BVH_GPU gpu;
		gpu.Build( quad, 2 );
		Compare( "Build, quad", gpu.bvh, gpu, Rays( bvhvec3( 0, 3, 0 ), bvhvec3( -1.5f, 0, -1.5f ), bvhvec3( 1.5f, 0, 1.5f ), 2000 ) );
	}
	// a rebuild into a root leaf must not keep the vertices of the first build
	{
		const bvhvec4 two[6] = { bvhvec4( 10, 0, 0, 0 ), bvhvec4( 11, 0, 0, 0 ), bvhvec4( 10, 1, 0, 0 ),
			bvhvec4( -10, 0, 0, 0 ), bvhvec4( -9, 0, 0, 0 ), bvhvec4( -10, 1, 0, 0 ) };
		const bvhvec4 one[3] = { bvhvec4( 0, 0, 3, 0 ), bvhvec4( 1, 0, 3, 0 ), bvhvec4( 0, 1, 3, 0 ) };
		BVH_GPU gpu;
		gpu.Build( two, 2 );
		gpu.Build( one, 1 );
		Compare( "Build of 2 triangles, then of 1", gpu.bvh, gpu, Rays( bvhvec3( 0.3f, 0.3f, -1 ), bvhvec3( -0.5f, -0.5f, 3 ), bvhvec3( 1.5f, 1.5f, 3 ), 2000 ) );
	}
	// ConvertFrom of a BVH that is a single leaf
	{
		BVH bvh;
		bvh.Build( tri, 1 );
		BVH_GPU gpu;
		gpu.ConvertFrom( bvh );
		Compare( "ConvertFrom, 1 triangle", bvh, gpu, triRays );
	}
	// 200 triangles sharing one box: one leaf of 200, of which the last ones alone cover a corner
	{
		std::vector<bvhvec4> v;
		const int n = 200;
		for (int k = 0; k < n; k++)
		{
			const float x = 1.0f - 2.0f * (float)(k + 1) / n;
			v.push_back( bvhvec4( -1, -1, 0, 0 ) ), v.push_back( bvhvec4( 1, 1, 0, 0 ) ), v.push_back( bvhvec4( x, 1, 0, 0 ) );
		}
		for (int k = 0; k < 2; k++) v.push_back( bvhvec4( 100.0f + k, 0, 0, 0 ) ), v.push_back( bvhvec4( 101.0f + k, 0, 0, 0 ) ), v.push_back( bvhvec4( 100.0f + k, 1, 0, 0 ) );
		// rays that stay clear of the shared diagonal and the outer edges, where rounding decides hit or miss
		std::vector<Ray> fanRays;
		for (const Ray& r : Rays( bvhvec3( 0, 0, -2 ), bvhvec3( -0.98f, -0.98f, 0 ), bvhvec3( 0.98f, 0.98f, 0 ), 4000 ))
		{
			const bvhvec3 P = r.O + r.D * (-r.O.z / r.D.z);
			if (fabsf( P.y - P.x ) > 0.01f && fanRays.size() < 2000) fanRays.push_back( r );
		}
		for (int hq = 0; hq < 2; hq++)
		{
			BVH_GPU gpu;
			if (hq) gpu.settings.binnedBVH = true, gpu.BuildHQ( v.data(), n + 2 ); else gpu.Build( v.data(), n + 2 );
			char label[96];
			snprintf( label, sizeof( label ), "%s, 200 triangles in one box (leaf of %u)", hq ? "SBVH" : "Build", LargestLeaf( gpu.bvh ) );
			Compare( label, gpu.bvh, gpu, fanRays );
		}
		// the same leaf alone, as the root
		BVH bvh;
		bvh.Build( v.data(), n );
		BVH_GPU gpu;
		gpu.ConvertFrom( bvh );
		Compare( "ConvertFrom, 200 triangles in one box, root leaf", bvh, gpu, fanRays );
	}
	// a scene, built as usual and with BuildQuick (large leaves)
	const char* fileName = argc > 1 ? argv[1] : "./testdata/cryteksponza.bin";
	FILE* f = fopen( fileName, "rb" );
	uint32_t triCount = 0;
	std::vector<bvhvec4> scene;
	if (f && fread( &triCount, 4, 1, f ) == 1 && triCount > 0)
	{
		scene.resize( (size_t)triCount * 3 );
		if (fread( scene.data(), 48, triCount, f ) != triCount) scene.clear();
	}
	if (f) fclose( f );
	if (scene.empty()) printf( "could not read %s - FAILED\n", fileName ), failures++;
	else
	{
		BVH quick;
		quick.BuildQuick( scene.data(), triCount );
		const bvhvec3 lo = quick.bvhNode[0].aabbMin, hi = quick.bvhNode[0].aabbMax, mid = (lo + hi) * 0.5f;
		const std::vector<Ray> rays = Rays( mid, lo, hi, 20000 );
		BVH_GPU gpu;
		gpu.Build( scene.data(), triCount );
		Compare( "Build, scene", gpu.bvh, gpu, rays );
		BVH_GPU gpuQuick;
		gpuQuick.ConvertFrom( quick );
		char label[96];
		snprintf( label, sizeof( label ), "ConvertFrom of BuildQuick, scene (leaves up to %u)", LargestLeaf( quick ) );
		Compare( label, quick, gpuQuick, rays );
	}
	if (failures) printf( "%i tests FAILED\n", failures ); else printf( "all tests passed\n" );
	return failures ? 1 : 0;
}
