// EXPERIMENTAL CODE - PLEASE IGNORE FOR NOW.

#define TINYBVH_IMPLEMENTATION
#include "tiny_bvh.h"

// SCENES:
// --------------------------------------------------
// 1: Crytek Sponza
// 2: Conference Room
// 3: Stanford Dragon
// 4: Bistro
// 5: Legocar
// 6: San Miguel
#define SCENE	6

// STAGES:
// --------------------------------------------------
// 1: Determine best bin count
// 2: Optimize using reinsertion & RRS
// 3: Report
#define STAGE	1

// EXPERIMENT SETTINGS:
// --------------------------------------------------
// #define VERIFY_OPTIMIZED_BVH
// #define RANDOM_BIN_COUNT

// RAY SETS:
// --------------------------------------------------
#define RRS_INTERIOR	1	// 8x8x8 grid of spherical path sources
#define RRS_OBJECT		2	// scene-surrounding sphere of path sources

// FILES:
// --------------------------------------------------
#if SCENE == 1
#define SCENE_NAME		"Crytek Sponza"
#define RAYSET_TYPE		RRS_INTERIOR
#define GEOM_FILE		"./testdata/cryteksponza.bin"
#define STAT_FILE		"sbvh_cryteksponza.csv"
#define HPLOC_FILE		"cryteksponza.hploc"
#define OPTIMIZED_BVH	"sbvh_cryteksponza_opt.bin"
#define RRS_SIZE		2'000'000
#define BEST_BINCOUNT	27.5f
#define BEST_BINNED_BVH	"sbvh_cryteksponza_27.5bins.bin"
#elif SCENE == 2
#define SCENE_NAME		"Conference Room"
#define RAYSET_TYPE		RRS_INTERIOR
#define GEOM_FILE		"./testdata/conference.bin"
#define STAT_FILE		"sbvh_conference.csv"
#define HPLOC_FILE		"conference.hploc"
#define OPTIMIZED_BVH	"sbvh_conference_opt.bin"
#define RRS_SIZE		1'000'000
#define BEST_BINCOUNT	31.5f // UPDATED
#define BEST_BINNED_BVH	"sbvh_conference_31.5bins.bin"
#elif SCENE == 3
#define SCENE_NAME		"Stanford Dragon"
#define RAYSET_TYPE		RRS_OBJECT
#define GEOM_FILE		"./testdata/dragon.bin"
#define STAT_FILE		"sbvh_dragon.csv"
#define HPLOC_FILE		"dragon.hploc"
#define OPTIMIZED_BVH	"sbvh_sponza_opt.bin"
#define RRS_SIZE		1'000'000
#define BEST_BINCOUNT	123.0f // UPDATED
#define BEST_BINNED_BVH	"sbvh_dragon_123bins.bin"
#elif SCENE == 4
#define SCENE_NAME		"Amazon Lumberyard Bistro"
#define RAYSET_TYPE		RRS_OBJECT
#define GEOM_FILE		"./testdata/bistro_ext_part1.bin"
#define STAT_FILE		"sbvh_bistro_ext.csv"
#define HPLOC_FILE		"bistro_ext.hploc"
#define OPTIMIZED_BVH	"sbvh_bistro_opt.bin"
#define RRS_SIZE		2'500'000
#define BEST_BINCOUNT	105.0f // UPDATED
#define BEST_BINNED_BVH	"sbvh_bistro_105bins.bin"
#elif SCENE == 5
#define SCENE_NAME		"Lego Car"
#define RAYSET_TYPE		RRS_OBJECT
#define GEOM_FILE		"./testdata/legocar.bin"
#define STAT_FILE		"sbvh_legocar.csv"
#define HPLOC_FILE		"legocar.hploc"
#define OPTIMIZED_BVH	"sbvh_legocar_opt.bin"
#define RRS_SIZE		500'000
#define BEST_BINCOUNT	56.5f // UPDATED
#define BEST_BINNED_BVH	"sbvh_legocar_56.5bins.bin"
#elif SCENE == 6
#define SCENE_NAME		"San Miguel"
#define RAYSET_TYPE		RRS_INTERIOR
#define GEOM_FILE		"./testdata/sanmiguel.bin"
#define STAT_FILE		"sbvh_sanmiguel.csv"
#define HPLOC_FILE		"sanmiguel.hploc"
#define OPTIMIZED_BVH	"sbvh_sanmiguel_opt.bin"
#define RRS_SIZE		2'500'000
#define BEST_BINCOUNT	27.0f
#define BEST_BINNED_BVH	"sbvh_sanmiguel_27bins.bin"
#endif

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

// "Representative Ray Set" generators.
// INTERIOR version:
// Spawns random paths from 8 x 8 x 8 points in the scene to create a final selection of 1M rays,
// in four equally sized groups: 'primary rays', 'short diffuse rays', 'long diffuse rays', and
// rays to the sky. The set will be used to somewhat objectively measure the traversal cost of a BVH.
// OBJECT version:
// Spawns random paths from a sphere surrounding the scene, towards a smaller sphere on the scene
// origin. Compared to the 'interior' version, this approach avoids paths that start inside objects.
Ray* rayset = new Ray[RRS_SIZE];
void RepresentativeRays( const uint32_t setType )
{
	// Build an intermedite BVH.
	BVH tmp;
	tmp.Build( tris, triCount );
	// Common preparations:
	bvhvec3 S[512], bmin = tmp.aabbMin, bext = tmp.aabbMax - tmp.aabbMin;
	const float sceneSize = tinybvh_max( tinybvh_max( bext.x, bext.y ), bext.z );
	const float shortRay = sceneSize * 0.03f, longRay = sceneSize * 10;
	const float epsilon = sceneSize * 0.00001f, tooShort = 50 * epsilon;
	uint32_t seed = 0x123456, progress = 0, spawnIdx = 0;
	uint32_t Ngroup1 = 0;		// primary ray ending on surface
	uint32_t Ngroup2 = 0;		// from prim to prim, short distance
	uint32_t Ngroup3 = 0;		// from prim to prim, long distance
	uint32_t Ngroup4 = 0;		// from prim to nothing
	// Produce the set.
	printf( "Generating representative ray set" );
	if (setType == RRS_INTERIOR)
	{
		// Place path spawn points in the scene on an 8x8x8 grid.
		for (int x = 0; x < 8; x++) for (int y = 0; y < 8; y++) for (int z = 0; z < 8; z++)
			S[x + y * 8 + z * 64] = bmin + (bvhvec3( (float)x, (float)y, (float)z ) + 1) * (1.0f / 9.0f) * bext;
		// Create random paths
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
	}
	else
	{
		// Calculate path spawn points on a elipsoid.
		for (int i = 0; i < 512; i++) S[i] = tinybvh_rndvec3( seed ) * bext * 2;
		// Create random paths
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
		r = r2 = rayset[i], sum += bvh->Intersect( r );
		refBVH->Intersect( r2 );
		if (r.hit.t != r2.hit.t) printf( "damaged BVH.\n" );
	}
	else for (uint32_t i = start; i < end; i++) r = rayset[i], sum += bvh->Intersect( r );
	splitSum[set] = sum;
}
float RRSTraceCost( const BVH* bvh, const BVH* refBVH = 0 )
{
	std::vector<std::thread> threads;
	for (uint32_t i = 0; i < 8; i++) threads.emplace_back( &TraceCostThread, bvh, refBVH, i, 8 );
	for (auto& thread : threads) thread.join();
	uint32_t sum = 0;
	for (int i = 0; i < 8; i++) sum += splitSum[i];
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

int main()
{
	// Initialize
	int minor = TINY_BVH_VERSION_MINOR, major = TINY_BVH_VERSION_MAJOR, sub = TINY_BVH_VERSION_SUB;
	printf( "TinyBVH v%i.%i.%i Optimizing Tool\n", major, minor, sub );
	printf( "----------------------------------------------------------------\n" );
	printf( "Loading... " );
	AddMesh( GEOM_FILE );
#if SCENE == 4
	AddMesh( "./testdata/bistro_ext_part2.bin", 1 ); // only scene with two files
#endif
	char n[] = SCENE_NAME;
	printf( "done. Results for %s (%i tris)\n-----------------------\n", n, triCount );
	RepresentativeRays( RAYSET_TYPE );

#if STAGE == 1 // STAGE 1: Find optimal bin count between 8 and 99, also try 'odd/even' counts.

	int bins = 8, bestCostBins = -1;
	float bestSAH = 1e30f, bestRRSCost, baseCost = 0;
	// reference: 8 bins
	printf( "Building reference BVH (8 bins)... " );
	BVH bvh;
	bvh.hqbvhbins = 8;
	bvh.BuildHQ( tris, triCount );
	printf( "done.\n" );
	baseCost = bestRRSCost = RRSTraceCost( &bvh );
	bool odd = false;
	// find optimal bin count by minimizing RRS cost.
	char t[] = STAT_FILE;
	FILE* f = fopen( t, "w" );
	while (1)
	{
		Timer t;
	#ifdef RANDOM_BIN_COUNT
		// split with random bin count between 35 and 97. Deprecated, not helping.
		bvh.hqbvhbins = bins;
		bvh.hqbvhbinseed = ((rand() & 8191) + 1) * 13;
		bvh.hqbvhrndbins = true;
	#else
		// use 'bins' splits, with one extra for odd tree levels.
		bvh.hqbvhbins = bins;
		bvh.hqbvhoddeven = odd;
		odd = !odd;
	#endif
		bvh.BuildHQ( tris, triCount );
		float buildTime = t.elapsed();
		// Evaluate traversal cost using RRS
		float sah = bvh.SAHCost();
		float RRScost = RRSTraceCost( &bvh );
		float percentage = baseCost * 100 / RRScost;
		printf( "SBVH, %i.%i bins (%.1fs): SAH=%5.1f, RRS %.2f [%.2f%%] ",
			bins, (bvh.hqbvhoddeven ? 5 : 0), buildTime, sah, RRScost, percentage );
		fprintf( f, "bins,%i.%i,time,%f,SAH,%f,RRS,%f\n", bins, (bvh.hqbvhoddeven ? 5 : 0), buildTime, sah, RRScost );
		if (RRScost < bestRRSCost) // we optimize for RRS cost, not SAH.
		{
			bestRRSCost = RRScost, bestCostBins = bins, bestSAH = sah;
			char t[] = BEST_BINNED_BVH;
			bvh.Save( t ); // overwrites previous best
			printf( " ==> saved to %s.\n", t );
		}
		else printf( "\n" );
		if (!odd) bins++;
		if (bins == 128) break; // searching beyond this point doesn't seem to make sense.
	}
	fclose( f );
	printf( "All done.\n" );

#elif STAGE == 2 // STAGE 2: Optimize bvh with optimal bin count

	// Obtain reference SBVH stats
	BVH refbvh;
	refbvh.hqbvhbins = HQBVHBINS;
	printf( "Building reference BVH (8 bins)... " );
	refbvh.BuildHQ( tris, triCount );
	printf( "done.\n" );
	float refCost = RRSTraceCost( &refbvh );
	// Load SBVH with best split plane count
	char t[] = BEST_BINNED_BVH; // generated in STAGE 1
	BVH bvh;
	bvh.Load( t, tris, triCount );
	float startCost = RRSTraceCost( &bvh );
	printf( "BVH in %s: SAH=%.2f, cost=%.2f (%.2f%%).\n", t, bvh.SAHCost(), startCost, 100 * refCost / startCost );
	BVH::BVHNode* backup = (BVH::BVHNode*)malloc64( bvh.allocatedNodes * sizeof( BVH::BVHNode ) );
	BVH_Verbose* verbose = new BVH_Verbose();
	uint32_t iteration = 0;
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
		printf( "Iteration %05i: SAH from %.2f to %.2f, cost from %.3f to %.3f", iteration++, SAHBefore, SAHafter, costBefore, costAfter );
		if (costAfter >= costBefore)
		{
			printf( " - REJECTED\n" );
			memcpy( bvh.bvhNode, backup, bvh.allocatedNodes * sizeof( BVH::BVHNode ) );
			bvh.usedNodes = usedBackup, bvh.allocatedNodes = allocBackup;
		}
		else
		{
			char t[] = OPTIMIZED_BVH;
			printf( " - %.2f%%, saved to %s\n", refCost * 100 / costAfter, t );
			bvh.Save( t );
		}
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

		BVH_Verbose verbose( bvhBinned );
		FILE* f = 0;
	#if SCENE == 1
		f = fopen( "cryteksponza.hploc", "rb" );
	#elif SCENE == 3
		f = fopen( "dragon.hploc", "rb" );
	#elif SCENE == 4
		f = fopen( "bistro_ext.hploc", "rb" );
	#elif SCENE == 5
		f = fopen( "legocar.hploc", "rb" );
	#endif
		if (f)
		{
			bvhvec3 bmin, bmax;
			fread( &bmin, 1, 12, f );
			fread( &bmax, 1, 12, f );
			uint32_t triCount, nodeCount;
			fread( &triCount, 1, 4, f );
			nodeCount = triCount * 2 - 1;
			fread( verbose.bvhNode, sizeof( BVH_Verbose::BVHNode ), nodeCount, f );
			verbose.usedNodes = nodeCount;
			BVH ploc;
			ploc.ConvertFrom( verbose );
			sah = ploc.SAHCost(), rrs = RRSTraceCost( &ploc );
			printf( "H-PLOC build   - SAH: %.3f, RRS: %.3f\n", sah, rrs );
		}

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

	return 0;
}