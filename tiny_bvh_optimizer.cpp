#define TINYBVH_IMPLEMENTATION
#include "tiny_bvh.h"

// SCENES:
// 1: Crytek Sponza
// 2: Conference Room
// 3: Stanford Dragon
// 4: Bistro
// 5: Legocar
// 6: San Miguel
#define SCENE	5
#define VERIFY_OPTIMIZED_BVH

// Includes, needful things
#ifdef _MSC_VER
#include "windows.h"	// for message window
#include "stdio.h"		// for printf
#else
#include <cstdio>
#endif
#include <fstream>
#include <chrono>
#include <thread>
#include <vector>

using namespace tinybvh;

// Global data
bvhvec4* tris = 0;	// triangles to build a BVH over.
int triCount = 0;	// number of triangles in the scene.

// Convenient timer class, for reporting
struct Timer
{
	Timer() { reset(); }
	float elapsed() const
	{
		auto t2 = std::chrono::high_resolution_clock::now();
		return (float)std::chrono::duration_cast<std::chrono::duration<double>>(t2 - start).count();
	}
	void reset() { start = std::chrono::high_resolution_clock::now(); }
	std::chrono::high_resolution_clock::time_point start;
};

// "Representative Ray Set" generator, INTERIOR version:
// Spawns random paths from 8 x 8 x 8 points in the scene to create a final selection of 1M rays,
// in four equally sized groups: 'primary rays', 'short diffuse rays', 'long diffuse rays', and
// rays to the sky. The set will be used to somewhat objectively measure the traversal cost of a BVH.
#define RRS_SIZE 5'000'000
Ray* rayset = new Ray[RRS_SIZE];
void RepresentativeRaysInterior()
{
	// Build an intermedite BVH.
	BVH tmp;
	tmp.Build( tris, triCount );
	// Place path spawn points in the scene on an 8x8x8 grid.
	bvhvec3 S[8 * 8 * 8], bmin = tmp.aabbMin, bext = tmp.aabbMax - tmp.aabbMin;
	for (int x = 0; x < 8; x++) for (int y = 0; y < 8; y++) for (int z = 0; z < 8; z++)
		S[x + y * 8 + z * 64] = bmin + (bvhvec3( (float)x, (float)y, (float)z ) + 1) * (1.0f / 9.0f) * bext;
	// Scene-size dependent values
	const float sceneSize = tinybvh_max( tinybvh_max( bext.x, bext.y ), bext.z );
	const float shortRay = sceneSize * 0.03f, longRay = sceneSize * 10;
	const float epsilon = sceneSize * 0.00001f, tooShort = 50 * epsilon;
	// Create random paths
	printf( "Generating representative ray set" );
	uint32_t seed = 0x123456, progress = 0, spawnIdx = 0;
	uint32_t Ngroup1 = 0;		// primary ray ending on surface
	uint32_t Ngroup2 = 0;		// from prim to prim, short distance
	uint32_t Ngroup3 = 0;		// from prim to prim, long distance
	uint32_t Ngroup4 = 0;		// from prim to nothing
	while (Ngroup1 + Ngroup2 + Ngroup3 + Ngroup4 < RRS_SIZE)
	{
		if (++progress == RRS_SIZE / 10) { printf( "." ); progress = 0; }
		// Random walk
		bvhvec3 P = S[spawnIdx++ & 511], R = tinybvh_rndvec3( seed );
		for (int j = 0; j < 8; j++)
		{
			Ray ray( P + R * epsilon, R ), r = ray /* copy with pristine hit record */;
			tmp.Intersect( ray );
			// Classify and store ray.
			if (j == 0 && ray.hit.t < longRay && Ngroup1 < RRS_SIZE / 4) rayset[Ngroup1++] = r;
			else if (j > 0 && ray.hit.t < shortRay && ray.hit.t > tooShort && Ngroup2 < RRS_SIZE / 4)
				rayset[Ngroup2++ + RRS_SIZE / 4] = r;
			else if (j > 0 && ray.hit.t < longRay && ray.hit.t > shortRay && Ngroup3 < RRS_SIZE / 4)
				rayset[Ngroup3++ + RRS_SIZE / 2] = r;
			else if (j > 0 && ray.hit.t == BVH_FAR && Ngroup4 < RRS_SIZE / 4)
				rayset[Ngroup4++ + 3 * (RRS_SIZE / 4)] = r;
			// Random bounce.
			if (ray.hit.t == BVH_FAR) break;
			uint32_t i0, i1, i2, triIdx = ray.hit.prim;
			GET_PRIM_INDICES_I0_I1_I2( tmp, triIdx );
			const bvhvec4 v0 = tmp.verts[i0], v1 = tmp.verts[i1], v2 = tmp.verts[i2];
			bvhvec3 N = tinybvh_normalize( tinybvh_cross( v1 - v0, v2 - v0 ) );
			if (tinybvh_dot( N, ray.D ) > 0) N *= -1.0f;
			R = tinybvh_rndvec3( seed );
			if (tinybvh_dot( R, N ) < 0) R *= -1.0f;
			P = P + ray.hit.t * R;
		}
	}
	printf( " done.\n" );
}

// "Representative Ray Set" generator, EXTERIOR version:
// Spawns random paths from a sphere surrounding the scene, towards a smaller sphere on the scene
// origin. Compared to the 'interior' version, this approach avoids paths that start inside objects.
void RepresentativeRaysExterior()
{
	// Build an intermedite BVH.
	BVH tmp;
	tmp.Build( tris, triCount );
	// Calculate path spawn points on a elipsoid.
	bvhvec3 S[512], bmin = tmp.aabbMin, bext = tmp.aabbMax - tmp.aabbMin;
	uint32_t seed = 0x123456;
	for (int i = 0; i < 512; i++) S[i] = tinybvh_rndvec3( seed ) * bext * 2;
	// Scene-size dependent values
	const float sceneSize = tinybvh_max( tinybvh_max( bext.x, bext.y ), bext.z );
	const float shortRay = sceneSize * 0.03f, longRay = sceneSize * 10;
	const float epsilon = sceneSize * 0.00001f, tooShort = 50 * epsilon;
	// Create random paths
	printf( "Generating representative ray set" );
	uint32_t progress = 0, spawnIdx = 0;
	uint32_t Ngroup1 = 0;		// primary ray ending on surface, 50%
	uint32_t Ngroup2 = 0;		// from prim to prim, 25%
	uint32_t Ngroup3 = 0;		// from prim to nothing, 25%
	while (Ngroup1 + Ngroup2 + Ngroup3 < RRS_SIZE)
	{
		if (++progress == RRS_SIZE / 10) { printf( "." ); progress = 0; }
		// Random walk
		bvhvec3 P = S[spawnIdx++ & 511], P2 = S[(spawnIdx * 13) & 511] * 0.1f;
		bvhvec3 R = tinybvh_normalize( P2 - P );
		for (int j = 0; j < 8; j++)
		{
			Ray ray( P + R * epsilon, R ), r = ray /* copy with pristine hit record */;
			tmp.Intersect( ray );
			// Classify and store ray.
			if (j == 0 && ray.hit.t < longRay && Ngroup1 < RRS_SIZE / 2) rayset[Ngroup1++] = r;
			else if (j > 0 && ray.hit.t > tooShort && Ngroup2 < RRS_SIZE / 4)
				rayset[Ngroup2++ + RRS_SIZE / 2] = r;
			else if (j > 0 && ray.hit.t == BVH_FAR && Ngroup3 < RRS_SIZE / 4)
				rayset[Ngroup3++ + 3 * (RRS_SIZE / 4)] = r;
			// Random bounce.
			if (ray.hit.t == BVH_FAR) break;
			uint32_t i0, i1, i2, triIdx = ray.hit.prim;
			GET_PRIM_INDICES_I0_I1_I2( tmp, triIdx );
			const bvhvec4 v0 = tmp.verts[i0], v1 = tmp.verts[i1], v2 = tmp.verts[i2];
			bvhvec3 N = tinybvh_normalize( tinybvh_cross( v1 - v0, v2 - v0 ) );
			if (tinybvh_dot( N, ray.D ) > 0) N *= -1.0f;
			R = tinybvh_rndvec3( seed );
			if (tinybvh_dot( R, N ) < 0) R *= -1.0f;
			P = P + ray.hit.t * R;
		}
	}
	printf( " done.\n" );
}

// Evaluate traversal cost using "Representative Ray Set"
uint32_t splitSum[256];
void TraceCostThread( const BVH* bvh, const BVH* refBVH, const int set, const int sets )
{
	uint32_t sum = 0, setSize = RRS_SIZE / sets, start = setSize * set;
	const uint32_t end = (set == (sets - 1)) ? RRS_SIZE : (start + setSize);
	Ray r, r2;
	if (refBVH) for (uint32_t i = start; i < end; i++)
	{
		r = r2 = rayset[i];
		sum += bvh->Intersect( r );
		refBVH->Intersect( r2 );
		if (r.hit.t != r2.hit.t)
		{
			printf( "damaged BVH.\n" );
			r2 = rayset[i];
			refBVH->Intersect( r2 );
		}
	}
	else for (uint32_t i = start; i < end; i++) r = rayset[i], sum += bvh->Intersect( r );
	splitSum[set] = sum;
}
float RRSTraceCost( const BVH* bvh, const BVH* refBVH = 0 )
{
#ifdef _DEBUG
	TraceCostThread( bvh, refBVH, 0, 1 );
	uint32_t sum = splitSum[0];
#else
	std::vector<std::thread> threads;
	for (uint32_t i = 0; i < 8; i++) threads.emplace_back( &TraceCostThread, bvh, refBVH, i, 8 );
	for (auto& thread : threads) thread.join();
	uint32_t sum = 0;
	for (int i = 0; i < 8; i++) sum += splitSum[i];
#endif
	return (float)sum / RRS_SIZE;
}

// Scene management - Append a file, with optional position, scale and color override, tinyfied
void AddMesh( const char* file, float scale = 1, bvhvec3 pos = {}, int c = 0, int N = 0 )
{
	std::fstream s{ file, s.binary | s.in };
	s.read( (char*)&N, 4 );
	bvhvec4* data = (bvhvec4*)malloc64( (N + triCount) * 48 );
	if (tris) memcpy( data, tris, triCount * 48 ), free64( tris );
	tris = data, s.read( (char*)tris + triCount * 48, N * 48 ), triCount += N;
	for (int* b = (int*)tris + (triCount - N) * 12, i = 0; i < N * 3; i++)
		*(bvhvec3*)b = *(bvhvec3*)b * scale + pos, b[3] = c ? c : b[3], b += 4;
}

// Scene management - Load specified scene
void LoadScene()
{
#if SCENE == 1
	AddMesh( "./testdata/cryteksponza.bin" );
	printf( "Results for Crytek Sponza (%i tris)\n------------------------------\n", triCount );
	RepresentativeRaysInterior();
#elif SCENE == 2
	AddMesh( "./testdata/conference.bin" );
	printf( "Results for Conference (%i tris)\n------------------------------\n", triCount );
	RepresentativeRaysInterior();
#elif SCENE == 3
	AddMesh( "./testdata/dragon.bin" );
	printf( "Results for Stanford Dragon (%i tris)\n------------------------------\n", triCount );
	RepresentativeRaysExterior();
#elif SCENE == 4
	AddMesh( "./testdata/bistro_ext_part1.bin", 1 );
	AddMesh( "./testdata/bistro_ext_part2.bin", 1 );
	printf( "Results for Bistro (%i tris)\n------------------------------\n", triCount );
	RepresentativeRaysExterior();
#elif SCENE == 5
	AddMesh( "./testdata/legocar.bin", 1 );
	printf( "Results for Legocar (%i tris)\n------------------------------\n", triCount );
	RepresentativeRaysExterior();
#elif SCENE == 6
	AddMesh( "./testdata/sanmiguel.bin", 1 );
	printf( "Results for SanMiguel (%i tris)\n------------------------------\n", triCount );
#ifndef _DEBUG
	RepresentativeRaysInterior();
#endif
#endif
}

#if 1

int main()
{
	// Initialize
	int minor = TINY_BVH_VERSION_MINOR, major = TINY_BVH_VERSION_MAJOR, sub = TINY_BVH_VERSION_SUB;
	printf( "TinyBVH v%i.%i.%i Optimizing Tool\n", major, minor, sub );
	printf( "----------------------------------------------------------------\n" );
	LoadScene();

	// STAGE 1: Find optimal bin count between 8 and 99.

	int bins = 8, bestCostBins = -1, first = 1;
	float bestSAH = 1e30f, bestRRSCost = 1e30f, baseCost = 0;
	BVH bvh;
	std::fstream test{ "sbvh.bin", test.binary | test.in }; // skip if sbvh.bin already exists
	if (!test) while (1)
	{
		Timer t;
		bvh.hqbvhbins = bins;
		bvh.BuildHQ( tris, triCount );
		float buildTime = t.elapsed();
		// Evaluate traversal cost using RRS
		float sah = bvh.SAHCost();
		float RRScost = RRSTraceCost( &bvh );
		if (RRScost < bestRRSCost) // we optimize for RRS cost, not SAH.
		{
			bestRRSCost = RRScost, bestCostBins = bins, bestSAH = sah;
			bvh.Save( "sbvh.bin" ); // overwrites previous best
		}
		// Report
		if (first) first = 0, baseCost = RRScost;
		float percentage = (bestRRSCost * 100) / baseCost;
		printf( "SBVH, %2u|127 bins (%.1fs): SAH=%5.1f, RRS %.2f", bins, buildTime, sah, RRScost );
		if (bestCostBins == bins) printf( " ==> saved to ""sbvh.bin"".\n" ); else
			printf( " (best: %2u bins, RRS %.2f [%.2f%%])\n", bestCostBins, bestRRSCost, percentage );
		if (++bins == 128) break; // searching beyond this point doesn't seem to make sense.
	}

	// STAGE 2: Optimize bvh with optimal bin count

	// Obtain reference SBVH stats
	BVH refbvh;
	refbvh.hqbvhbins = HQBVHBINS;
	refbvh.BuildHQ( tris, triCount );
	float refCost = RRSTraceCost( &refbvh );
	// Load SBVH with best split plane count
	bvh.Load( "sbvh.bin", tris, triCount ); // generated in STAGE 1
	BVH::BVHNode* backup = (BVH::BVHNode*)malloc64( bvh.allocatedNodes * sizeof( BVH::BVHNode ) );
	BVH_Verbose* verbose = new BVH_Verbose();
	// Optimize
	while (1)
	{
		float SAHBefore = bvh.SAHCost();
		float costBefore = RRSTraceCost( &bvh );
		memcpy( backup, bvh.bvhNode, bvh.allocatedNodes * sizeof( BVH::BVHNode ) );
		uint32_t usedBackup = bvh.usedNodes, allocBackup = bvh.allocatedNodes;
		verbose->ConvertFrom( bvh );
		verbose->Optimize( 1, false, true );
		bvh.ConvertFrom( *verbose, false );
		float SAHafter = bvh.SAHCost();
	#ifdef VERIFY_OPTIMIZED_BVH
		float costAfter = RRSTraceCost( &bvh, &refbvh );
	#else
		float costAfter = RRSTraceCost( &bvh, 0 );
	#endif
		printf( "Optimizer: SAH from %.2f to %.2f, cost from %.3f to %.3f", SAHBefore, SAHafter, costBefore, costAfter );
		if (costAfter >= costBefore)
		{
			printf( " - REJECTED\n" );
			memcpy( bvh.bvhNode, backup, bvh.allocatedNodes * sizeof( BVH::BVHNode ) );
			bvh.usedNodes = usedBackup, bvh.allocatedNodes = allocBackup;
		}
		else
		{
			float percentage = (costAfter * 100) / refCost;
			printf( " - %.2f%%, saved to sbvh_opt.bin\n", percentage );
		#if SCENE == 1
			bvh.Save( "sbvh_sponza_opt.bin" );
		#elif SCENE == 2
			bvh.Save( "sbvh_conference_opt.bin" );
		#elif SCENE == 3
			bvh.Save( "sbvh_dragon_opt.bin" );
		#elif SCENE == 4
			bvh.Save( "sbvh_bistro_opt.bin" );
		#elif SCENE == 5
			bvh.Save( "sbvh_legocar_opt.bin" );
		#elif SCENE == 6
			bvh.Save( "sbvh_sanmiguel_opt.bin" );
		#endif
		}
	}

	printf( "all done." );
	return 0;
}

#else

// Report on BVH quality
int main()
{
	// Initialize
	LoadScene();

	// Prepare and evaluate several BVHs
	float sah, rrs;

	BVH bvhBinned;
	bvhBinned.Build( tris, triCount );
	sah = bvhBinned.SAHCost(), rrs = RRSTraceCost( &bvhBinned );
	printf( "SAH BVH Binned (8) - SAH: %.3f, RRS: %.3f\n", sah, rrs );

#if SCENE != 2

	BVH bvhSweep;
	bvhSweep.useFullSweep = true;
	bvhSweep.Build( tris, triCount );
	sah = bvhSweep.SAHCost(), rrs = RRSTraceCost( &bvhSweep );
	printf( "Full-sweep SAH BVH - SAH: %.3f, RRS: %.3f\n", sah, rrs );

#endif

	BVH bvhSpatial;
	bvhSpatial.BuildHQ( tris, triCount );
	sah = bvhSpatial.SAHCost(), rrs = RRSTraceCost( &bvhSpatial );
	printf( "Spatial split BVH  - SAH: %.3f, RRS: %.3f\n", sah, rrs );

	BVH bvhOptimized;
	bvhOptimized.Build( tris, triCount );
	bvhOptimized.Optimize( 250, true );
	sah = bvhOptimized.SAHCost(), rrs = RRSTraceCost( &bvhOptimized );
	printf( "Optimized binned   - SAH: %.3f, RRS: %.3f\n", sah, rrs );
#if 0
	bvhOptimized.Optimize( 1000, true );
	sah = bvhOptimized.SAHCost(), rrs = RRSTraceCost( &bvhOptimized );
	printf( "Optimized+ binned  - SAH: %.3f, RRS: %.3f\n", sah, rrs );
#endif

	BVH bvhSpatialOptimized;
	bvhSpatialOptimized.BuildHQ( tris, triCount );
	bvhSpatialOptimized.Optimize( 250, true );
	sah = bvhSpatialOptimized.SAHCost(), rrs = RRSTraceCost( &bvhSpatialOptimized );
	printf( "Optimized SBVH     - SAH: %.3f, RRS: %.3f\n", sah, rrs );
#if 0
	bvhSpatialOptimized.Optimize( 1000, true );
	sah = bvhSpatialOptimized.SAHCost(), rrs = RRSTraceCost( &bvhSpatialOptimized );
	printf( "Optimized+ SBVH    - SAH: %.3f, RRS: %.3f\n", sah, rrs );
#endif

	BVH bvhExtreme;
#if SCENE == 1
	bvhExtreme.Load( "sbvh_sponza_opt.bin", tris, triCount );
#elif SCENE == 2
	bvhExtreme.Load( "sbvh_conference_opt.bin", tris, triCount );
#elif SCENE == 3
	bvhExtreme.Load( "sbvh_dragon_opt.bin", tris, triCount );
#elif SCENE == 4
	bvhExtreme.Load( "sbvh_bistro_opt.bin", tris, triCount );
#elif SCENE == 5
	bvhExtreme.Load( "sbvh_legocar_opt.bin", tris, triCount );
#elif SCENE == 6
	bvhExtreme.Load( "sbvh_sanmiguel_opt.bin", tris, triCount );
#endif
	sah = bvhExtreme.SAHCost(), rrs = RRSTraceCost( &bvhExtreme );
	printf( "Ours               - SAH: %.3f, RRS: %.3f\n", sah, rrs );
}

#endif