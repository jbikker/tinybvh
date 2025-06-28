// Code for SBVH optimization experiments.
// Usage:
// - Specify the scene using #define SCENE
// - Start with STAGE 1 to determine an optimized bin count. This also
//   produces a precalculated BVH on disk which will be used in stage 2.
// - Set STAGE to 2 to optimize the BVH. A new precalculated BVH will
//   be saved to disk. The process takes several hours for most scenes.
// - Get detailed statistics on the results by setting STAGE to 3.

// set C_INT and C_TRAV to match the paper
// "On Quality Metrics of Bounding Volume Hierarchies",
// Aila et al., 20213
#define C_INT	1.0f
#define C_TRAV	1.2f

// SCENES:
// --------------------------------------------------
// 1: Crytek Sponza
// 2: Conference Room
// 3: Stanford Dragon
// 4: Bistro
// 5: Legocar
// 6: San Miguel
// 7: Living Room
// 8: Living Room, rotated
#define SCENE	9

// STAGES:
// --------------------------------------------------
// 1: Determine best bin count
// 2: Optimize using reinsertion & RRS
// 3: Report
#define STAGE	2

// EXPERIMENT SETTINGS:
// --------------------------------------------------
// #define VERIFY_OPTIMIZED_BVH
// #define CALCULATE_EPO

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
#define STAT_FILE		"./testdata/opt_rrs/sbvh_cryteksponza.csv"
#define HPLOC_FILE		"./testdata/hploc/cryteksponza.hploc"
#define RESULTS_FILE	"./testdata/sponza_results.csv"
#define OPTIMIZED_BVH	"./testdata/opt_rrs/sbvh_cryteksponza_opt.bin"
#define RRS_SIZE		2'000'000 // must be a multiple of 64 for NVIDIA OpenCL
#define BEST_BINCOUNT	33.5f
#define BEST_BINNED_BVH	"./testdata/opt_rrs/sbvh_cryteksponza_33.5bins.bin"
#elif SCENE == 2
#define SCENE_NAME		"Conference Room"
#define RAYSET_TYPE		RRS_INTERIOR
#define GEOM_FILE		"./testdata/conference.bin"
#define STAT_FILE		"./testdata/opt_rrs/sbvh_conference.csv"
#define HPLOC_FILE		"./testdata/hploc/conference.hploc"
#define RESULTS_FILE	"./testdata/conference_results.csv"
#define OPTIMIZED_BVH	"./testdata/opt_rrs/sbvh_conference_opt.bin"
#define RRS_SIZE		1'000'000 // must be a multiple of 64 for NVIDIA OpenCL
#define BEST_BINCOUNT	31.5f
#define BEST_BINNED_BVH	"./testdata/opt_rrs/sbvh_conference_31.5bins.bin"
#define	W_EPO			0.41f // as specified in paper, overriding default 0.71
#elif SCENE == 3
#define SCENE_NAME		"Stanford Dragon"
#define RAYSET_TYPE		RRS_OBJECT
#define GEOM_FILE		"./testdata/dragon.bin"
#define STAT_FILE		"./testdata/opt_rrs/sbvh_dragon.csv"
#define HPLOC_FILE		"./testdata/hploc/dragon.hploc"
#define RESULTS_FILE	"./testdata/dragon_results.csv"
#define OPTIMIZED_BVH	"./testdata/opt_rrs/sbvh_dragon_opt.bin"
#define RRS_SIZE		1'000'000 // must be a multiple of 64 for NVIDIA OpenCL
#define BEST_BINCOUNT	93.0f
#define BEST_BINNED_BVH	"./testdata/opt_rrs/sbvh_dragon_93bins.bin"
#define	W_EPO			0.61f // as specified in paper, overriding default 0.71
#elif SCENE == 4
#define SCENE_NAME		"Amazon Lumberyard Bistro"
#define RAYSET_TYPE		RRS_OBJECT
#define GEOM_FILE		"./testdata/bistro_ext_part1.bin"
#define STAT_FILE		"./testdata/opt_rrs/sbvh_bistro_ext.csv"
#define HPLOC_FILE		"./testdata/hploc/bistro.hploc"
#define RESULTS_FILE	"./testdata/bistro_results.csv"
#define OPTIMIZED_BVH	"./testdata/opt_rrs/sbvh_bistro_opt.bin"
#define RRS_SIZE		2'500'032 // must be a multiple of 64 for NVIDIA OpenCL
#define BEST_BINCOUNT	105.0f
#define BEST_BINNED_BVH	"./testdata/opt_rrs/sbvh_bistro_105bins.bin"
#elif SCENE == 5
#define SCENE_NAME		"Lego Car"
#define RAYSET_TYPE		RRS_OBJECT
#define GEOM_FILE		"./testdata/legocar.bin"
#define STAT_FILE		"./testdata/opt_rrs/sbvh_legocar.csv"
#define HPLOC_FILE		"./testdata/hploc/legocar.hploc"
#define RESULTS_FILE	"./testdata/lego_results.csv"
#define OPTIMIZED_BVH	"./testdata/opt_rrs/sbvh_legocar_opt.bin"
#define RRS_SIZE		500'032 // must be a multiple of 64 for NVIDIA OpenCL
#define BEST_BINCOUNT	38.5f
#define BEST_BINNED_BVH	"./testdata/opt_rrs/sbvh_legocar_38.5bins.bin"
#elif SCENE == 6
#define SCENE_NAME		"San Miguel"
#define RAYSET_TYPE		RRS_INTERIOR
#define GEOM_FILE		"./testdata/sanmiguel.bin"
#define STAT_FILE		"./testdata/opt_rrs/sbvh_sanmiguel.csv"
#define HPLOC_FILE		"./testdata/hploc/sanmiguel.hploc"
#define RESULTS_FILE	"./testdata/sanmiguel_results.csv"
#define OPTIMIZED_BVH	"./testdata/opt_rrs/sbvh_sanmiguel_opt.bin"
#define RRS_SIZE		2'500'032 // must be a multiple of 64 for NVIDIA OpenCL
#define BEST_BINCOUNT	27.0f
#define BEST_BINNED_BVH	"./testdata/opt_rrs/sbvh_sanmiguel_27bins.bin"
#define	W_EPO			0.72f // as specified in paper, overriding default 0.71
#elif SCENE == 7
#define SCENE_NAME		"Living Room"
#define RAYSET_TYPE		RRS_INTERIOR
#define GEOM_FILE		"./testdata/living.bin"
#define STAT_FILE		"./testdata/opt_rrs/sbvh_living.csv"
#define HPLOC_FILE		"./testdata/hploc/living.hploc"
#define RESULTS_FILE	"./testdata/living_results.csv"
#define OPTIMIZED_BVH	"./testdata/opt_rrs/sbvh_living_opt.bin"
#define RRS_SIZE		2'500'032
#define BEST_BINCOUNT	124.5f
#define BEST_BINNED_BVH	"./testdata/opt_rrs/sbvh_living_124.5bins.bin"
#elif SCENE == 8
#define SCENE_NAME		"Living Room (rotated)"
#define RAYSET_TYPE		RRS_INTERIOR
#define GEOM_FILE		"./testdata/living_rotated.bin"
#define STAT_FILE		"./testdata/opt_rrs/sbvh_living_rotated.csv"
#define HPLOC_FILE		"./testdata/hploc/living_rotated.hploc"
#define RESULTS_FILE	"./testdata/living_rotated_results.csv"
#define OPTIMIZED_BVH	"./testdata/opt_rrs/sbvh_living_rotated_opt.bin"
#define RRS_SIZE		2'500'032
#define BEST_BINCOUNT	23.0f
#define BEST_BINNED_BVH	"./testdata/opt_rrs/sbvh_living_rotated_23bins.bin"
#elif SCENE == 9
#define SCENE_NAME		"Cached BVH"
#define RAYSET_TYPE		RRS_OBJECT
#define GEOM_FILE		"./scene_0012.tri"
#define STAT_FILE		"./scene_0012_stats.csv"
#define HPLOC_FILE		""
#define RESULTS_FILE	"./scene_0012_results.csv"
#define OPTIMIZED_BVH	"./scene_0012_opt.bin"
#define RRS_SIZE		2'500'032
#define BEST_BINCOUNT	93.0f
#define BEST_BINNED_BVH	"./scene_0012_opt_93bins.bin"
#endif

// TinyBVH, TinyOCL
#define TINYBVH_IMPLEMENTATION
#include "tiny_bvh.h"

#ifdef _MSC_VER
#include "Windows.h"
#endif

#define TINY_OCL_IMPLEMENTATION
#include "tiny_ocl.h"

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
tinyocl::Buffer* gpuNodes = 0;
tinyocl::Buffer* idxData = 0;
tinyocl::Buffer* triData = 0;
tinyocl::Buffer* rayData = 0;
tinyocl::Buffer* rrsResult = 0;
BVH_GPU bvhgpu;
tinyocl::Kernel* ailalaine_kernel = 0;
tinyocl::Kernel* ailalaine_rrs_kernel = 0;
uint32_t* rawRays = 0;

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
	tmp.BuildHQ( tris, triCount );
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

// RRS cost and trace time is evaluated on the GPU.
void UpdateBVHOnGPU( const BVH* bvh )
{
	bvhgpu.context = bvh->context;
	bvhgpu.ConvertFrom( *bvh );
	if (!ailalaine_kernel)
	{
		ailalaine_kernel = new tinyocl::Kernel( "traverse.cl", "batch_ailalaine" );
		ailalaine_rrs_kernel = new tinyocl::Kernel( "traverse.cl", "batch_ailalaine_rrs" );
		gpuNodes = new tinyocl::Buffer( bvh->allocatedNodes * 2 /* room for growth */ * sizeof( BVH_GPU::BVHNode ) );
		idxData = new tinyocl::Buffer( bvh->idxCount * 2 /* room for growth */ * sizeof( unsigned ) );
		rayData = new tinyocl::Buffer( RRS_SIZE * 64 /* sizeof( tinybvh::Ray ) */ );
		triData = new tinyocl::Buffer( bvh->triCount * 3 * sizeof( tinybvh::bvhvec4 ), tris );
		rrsResult = new tinyocl::Buffer( RRS_SIZE * 4 );
		triData->CopyToDevice();
		rrsResult->CopyToDevice();
		rawRays = new uint32_t[16 * RRS_SIZE];
		for (unsigned i = 0; i < RRS_SIZE; i++) memcpy( rawRays + i * 16, &rayset[i], 64 );
		ailalaine_kernel->SetArguments( gpuNodes, idxData, triData, rayData );
		ailalaine_rrs_kernel->SetArguments( gpuNodes, idxData, triData, rayData, rrsResult );
	};
	memcpy( gpuNodes->GetHostPtr(), bvhgpu.bvhNode, bvhgpu.usedNodes * sizeof( BVH_GPU::BVHNode ) );
	memcpy( idxData->GetHostPtr(), bvhgpu.bvh.primIdx, bvh->idxCount * sizeof( unsigned ) );
	gpuNodes->CopyToDevice();
	idxData->CopyToDevice();
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
#if 0
	// calculate RRS cost on CPU
	std::vector<std::thread> threads;
	for (uint32_t i = 0; i < 8; i++) threads.emplace_back( &TraceCostThread, bvh, refBVH, i, 8 );
	for (auto& thread : threads) thread.join();
	uint32_t sum = 0;
	for (int i = 0; i < 8; i++) sum += splitSum[i];
	return (float)sum / RRS_SIZE;
#else
	// calculate RRS cost on GPU
	UpdateBVHOnGPU( bvh ); // TODO: once per iteration
	memcpy( rayData->GetHostPtr(), rawRays, 64 * RRS_SIZE );
	rayData->CopyToDevice();
	ailalaine_rrs_kernel->Run( RRS_SIZE, 64, 0 );
	rrsResult->CopyFromDevice();
	uint32_t sum = 0, * r = rrsResult->GetHostPtr();
	for (int i = 0; i < RRS_SIZE; i++) sum += r[i];
	return (float)sum / RRS_SIZE;
#endif
}
float RRSTraceTimeCPU( const BVH* bvh )
{
	BVH8_CPU fastbvh;
	fastbvh.bvh8.bvh.context = fastbvh.bvh8.context = bvh->context;
	fastbvh.bvh8.bvh = *bvh;
	fastbvh.ConvertFrom( fastbvh.bvh8 );
	Timer t;
	uint32_t sum = 0;
	for (int i = 0; i <= 10; i++)
	{
		if (i == 1) t.reset(); // first one is for cache warming
		Ray r;
		for (int j = 0; j < RRS_SIZE; j++) r = rayset[j], sum += fastbvh.Intersect( r );
	}
	float runtime = t.elapsed() * 0.1f;
	fastbvh.bvh8.triCount = sum; // dummy operation to avoid dead code elimination
	fastbvh.bvh8 = MBVH<8>();
	return runtime; // average of 10 runs
}
float RRSTraceTimeGPU( const BVH* bvh )
{
	// create rays and send them to the gpu side
	UpdateBVHOnGPU( bvh );
	memcpy( rayData->GetHostPtr(), rawRays, 64 * RRS_SIZE );
	rayData->CopyToDevice();
	// start timer and start kernel on gpu
	cl_event event;
	cl_ulong startTime, endTime;
	float traceTime = 0;
	for (int pass = 0; pass <= 50; pass++)
	{
		ailalaine_kernel->Run( RRS_SIZE, 64, 0, &event );
		clWaitForEvents( 1, &event ); // OpenCL kernsl run asynchronously
		clGetEventProfilingInfo( event, CL_PROFILING_COMMAND_START, sizeof( cl_ulong ), &startTime, 0 );
		clGetEventProfilingInfo( event, CL_PROFILING_COMMAND_END, sizeof( cl_ulong ), &endTime, 0 );
		if (pass == 0) continue; // first pass is for cache warming
		traceTime += (endTime - startTime) * 1e-9f; // event timing is in nanoseconds
	}
	return traceTime * 0.02f;
}

// Scene management - Append a file, with optional position, scale and color override, tinyfied
void AddMesh( const char* file, float scale = 1, bvhvec3 pos = {}, int c = 0, int N = 0 )
{
	std::fstream s{ file, s.binary | s.in };
	s.read( (char*)&N, 4 );
	bvhvec4* data = (bvhvec4*)tinybvh::malloc64( (N + triCount) * 48 );
	if (tris) memcpy( data, tris, triCount * 48 ), tinybvh::free64( tris );
	tris = data, s.read( (char*)tris + triCount * 48, N * 48 ), triCount += N;
	for (int* b = (int*)tris + (triCount - N) * 12, i = 0; i < N * 3; i++)
		*(bvhvec3*)b = *(bvhvec3*)b * scale + pos, b[3] = c ? c : b[3], b += 4;
}

// BVH quality evaluation: EPO or RRS
float bvhcost( const BVH& bvh )
{
#ifdef CALCULATE_EPO
	return bvh.EPOCost();
#else
	return RRSTraceCost( &bvh );
#endif
}

float refsah = 0, refrrs = 0, refepo = 0, refcpu = 0, refgpu = 0;
void printstat( float sah, float rrs, float epo, float cpu, float gpu )
{
	printf( "%.3f (%+6.2f%%)  ", sah, 100 * refsah / sah - 100 );
	printf( "%.3f (%+6.2f%%)  ", rrs, 100 * refrrs / rrs - 100 );
#ifdef CALCULATE_EPO
	printf( "%.3f (%+6.2f%%)  ", epo, 100 * refepo / epo - 100 );
#endif
	printf( "%.3f (%+6.2f%%)  ", cpu, 100 * refcpu / cpu - 100 );
	printf( "%.3f (%+6.2f%%)\n", gpu, 100 * refgpu / gpu - 100 );
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
	float bestSAH = 1e30f, bestRRSCost, baseCost = 0, baseEpo, bestEpo;
	// reference: 8 bins
	printf( "Building reference BVH (8 bins)... " );
	BVH bvh;
	bvh.hqbvhbins = 8;
	bvh.BuildHQ( tris, triCount );
	printf( "done.\n" );
	baseCost = bestRRSCost = RRSTraceCost( &bvh );
#ifdef CALCULATE_EPO
	baseEpo = bestEpo = bvh.EPOCost();
#else
	baseEpo = bestEpo = 0;
#endif
	bool odd = false;
	// find optimal bin count by minimizing RRS cost.
	char t[] = STAT_FILE;
	FILE* f = fopen( t, "w" );
	while (1)
	{
		Timer t;
		// use 'bins' splits, with one extra for odd tree levels.
		bvh.hqbvhbins = bins;
		bvh.hqbvhoddeven = odd;
		odd = !odd;
		bvh.BuildHQ( tris, triCount );
		float buildTime = t.elapsed();
		// Evaluate traversal cost using RRS
		float sah = bvh.SAHCost(), epo = 0;
		float RRScost = RRSTraceCost( &bvh );
		float RRSpercentage = baseCost * 100 / RRScost;
	#ifdef CALCULATE_EPO
		epo = bvh.EPOCost();
		float EPOpercentage = baseEpo * 100 / epo;
		printf( "SBVH, %i.%i bins (%.1fs): SAH=%5.1f, RRS %.2f [%.2f%%], EPO %.2f [%.2f%%] ",
			bins, (bvh.hqbvhoddeven ? 5 : 0), buildTime, sah, RRScost, RRSpercentage, epo, EPOpercentage );
	#else
		printf( "SBVH, %i.%i bins (%.1fs): SAH=%5.1f, RRS %.2f [%.2f%%] ",
			bins, (bvh.hqbvhoddeven ? 5 : 0), buildTime, sah, RRScost, RRSpercentage );
	#endif
		fprintf( f, "bins,%i.%i,time,%f,SAH,%f,RRS,%f,EPO,%f\n", bins, (bvh.hqbvhoddeven ? 5 : 0), buildTime, sah, RRScost, epo );
		// if (epo < bestEpo)		// we optimize for EPO cost
		if (RRScost < bestRRSCost)	// we optimize for RRS cost
		{
			// bestSAH = sah;
			bestRRSCost = RRScost;
			// bestEpo = epo;
			bestCostBins = bins;
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
	float refCost = bvhcost( refbvh );
	BVH bvh;
	// Try to continue where we left off
	char b[] = OPTIMIZED_BVH;
	float startCost;
	if (!bvh.Load( b, tris, triCount ))
	{
		// Load SBVH with best split plane count
		char t[] = BEST_BINNED_BVH; // generated in STAGE 1
		bvh.Load( t, tris, triCount );
		startCost = bvhcost( bvh );
		printf( "BVH in %s: SAH=%.2f, cost=%.2f (%.2f%%).\n", t, bvh.SAHCost(), startCost, 100 * refCost / startCost );
	}
	else
	{
		startCost = bvhcost( bvh );
		printf( "BVH in %s: SAH=%.2f, cost=%.2f (%.2f%%).\n", b, bvh.SAHCost(), startCost, 100 * refCost / startCost );
	}
	BVH::BVHNode* backup = (BVH::BVHNode*)tinybvh::malloc64( bvh.allocatedNodes * sizeof( BVH::BVHNode ) );
	BVH_Verbose* verbose = new BVH_Verbose();
	uint32_t iteration = 0;
	// Optimize
	float sahBefore = bvh.SAHCost();
	float costBefore = bvhcost( bvh );
	while (1)
	{
		memcpy( backup, bvh.bvhNode, bvh.allocatedNodes * sizeof( BVH::BVHNode ) );
		uint32_t usedBackup = bvh.usedNodes, allocBackup = bvh.allocatedNodes;
		verbose->ConvertFrom( bvh );
		verbose->Optimize( 1, false, true );
		bvh.ConvertFrom( *verbose, false );
		float sahAfter = bvh.SAHCost();
	#if defined VERIFY_OPTIMIZED_BVH && !defined CALCULATE_EPO
		float costAfter = RRSTraceCost( &bvh, &refbvh );
	#else
		float costAfter = bvhcost( bvh );
	#endif
		printf( "Iteration %05i: SAH from %.2f to %.2f, cost from %.3f to %.3f", iteration++, sahBefore, sahAfter, costBefore, costAfter );
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
			sahBefore = sahAfter;
			costBefore = costAfter;
		}
	}

#elif STAGE == 3

	// Prepare and evaluate several BVHs
	FILE* c = fopen( RESULTS_FILE, "w" );
	{
		BVH bvh;
		bvh.useFullSweep = true;
		bvh.Build( tris, triCount );
		float sah = bvh.SAHCost(), rrs = RRSTraceCost( &bvh ), epo = 0;
	#ifdef CALCULATE_EPO
		epo = bvh.EPOCost();
	#endif
		float cpu = RRSTraceTimeCPU( &bvh ), gpu = RRSTraceTimeGPU( &bvh );
	#ifdef CALCULATE_EPO
		printf( "                     SAH               RRS               EPO               CPU time         GPU time\n" );
		printf( "                     ----------------------------------------------------------------------------------------\n" );
		printf( "SAH (full sweep)     %.3f ( 100.0%%)  %.3f ( 100.0%%)  %.3f ( 100.0%%)  %.3f ( 100.0%%)  %.3f ( 100.0%%)\n", sah, rrs, epo, cpu, gpu );
	#else
		printf( "                     SAH               RRS               CPU time         GPU time\n" );
		printf( "                     -----------------------------------------------------------------------\n" );
		printf( "SAH (full sweep)     %.3f ( 100.0%%)  %.3f ( 100.0%%)  %.3f ( 100.0%%)  %.3f ( 100.0%%)\n", sah, rrs, cpu, gpu );
	#endif
		fprintf( c, ",sah,rrs,epo,cpu,gpu\n" );
		fprintf( c, "full sweep,%f,%f,%f,%f,%f\n", sah, rrs, epo, cpu, gpu );
		refsah = sah, refrrs = rrs, refepo = epo, refcpu = cpu, refgpu = gpu;
		bvh.Optimize( 50 );
		sah = bvh.SAHCost(), rrs = RRSTraceCost( &bvh );
	#ifdef CALCULATE_EPO
		epo = bvh.EPOCost();
	#endif
		cpu = RRSTraceTimeCPU( &bvh ), gpu = RRSTraceTimeGPU( &bvh );
		printf( "Optimized f.sweep    " );
		printstat( sah, rrs, epo, cpu, gpu );
	}
	{
		FILE* f = fopen( HPLOC_FILE, "rb" );
		if (f)
		{
			BVH bvh;
			bvh.Build( tris, triCount );
			BVH_Verbose verbose( bvh );
			bvhvec3 bmin, bmax;
			fread( &bmin, 1, 12, f );
			fread( &bmax, 1, 12, f );
			uint32_t nodeCount;
			fread( &nodeCount, 1, 4, f );
			fread( verbose.bvhNode, sizeof( BVH_Verbose::BVHNode ), nodeCount, f );
			verbose.usedNodes = nodeCount;
			for (int i = 0; i < triCount; i++) verbose.primIdx[i] = i;
			// verbose.Refit();
			verbose.SortIndices();
			bvh.ConvertFrom( verbose );
			bvh.CombineLeafs();
			bvh.CombineLeafs();
			verbose.Refit();
			float sah = bvh.SAHCost(), rrs = RRSTraceCost( &bvh ), epo = 0;
		#ifdef CALCULATE_EPO
			epo = bvh.EPOCost();
		#endif
			float cpu = RRSTraceTimeCPU( &bvh ), gpu = RRSTraceTimeGPU( &bvh );
			fprintf( c, "hploc,%f,%f,%f,%f,%f\n", sah, rrs, epo, cpu, gpu );
			printf( "H-PLOC build         " );
			printstat( sah, rrs, epo, cpu, gpu );
		}
		else printf( "H-PLOC build         MISSING FILE, SKIPPED\n" );
	}
#if SCENE != 8
	{
		BVH bvh; // defaults to 8 bins
		bvh.Build( tris, triCount );
		float sah = bvh.SAHCost(), rrs = RRSTraceCost( &bvh ), epo = 0;
	#ifdef CALCULATE_EPO
		epo = bvh.EPOCost();
	#endif
		float cpu = RRSTraceTimeCPU( &bvh ), gpu = RRSTraceTimeGPU( &bvh );
		printf( "SAH BVH Binned (8)   " );
		fprintf( c, "binned[8],%f,%f,%f,%f,%f\n", sah, rrs, epo, cpu, gpu );
		printstat( sah, rrs, epo, cpu, gpu );
		bvh.Optimize( 50 );
		sah = bvh.SAHCost(), rrs = RRSTraceCost( &bvh );
	#ifdef CALCULATE_EPO
		epo = bvh.EPOCost();
	#endif
		cpu = RRSTraceTimeCPU( &bvh ), gpu = RRSTraceTimeGPU( &bvh );
		printf( "Optimized BVH        " );
		fprintf( c, "binned[8] optimized,%f,%f,%f,%f,%f\n", sah, rrs, epo, cpu, gpu );
		printstat( sah, rrs, epo, cpu, gpu );
	}
#else
	printf( "SAH BVH Binned (8)   SKIPPED\n" );
	printf( "Optimized BVH        SKIPPED\n" );
#endif
	{
		BVH bvh;
		bvh.hqbvhbins = 8;
		bvh.BuildHQ( tris, triCount );
		float sah = bvh.SAHCost(), rrs = RRSTraceCost( &bvh ), epo = 0;
	#ifdef CALCULATE_EPO
		epo = bvh.EPOCost();
	#endif
		float cpu = RRSTraceTimeCPU( &bvh ), gpu = RRSTraceTimeGPU( &bvh );
		printf( "SBVH, 8 bins         " );
		fprintf( c, "sbvh[8],%f,%f,%f,%f,%f\n", sah, rrs, epo, cpu, gpu );
		printstat( sah, rrs, epo, cpu, gpu );
	}
	{
		BVH bvh;
		bvh.hqbvhbins = 32;
		bvh.BuildHQ( tris, triCount );
		float sah = bvh.SAHCost(), rrs = RRSTraceCost( &bvh ), epo = bvh.EPOCost();
		float cpu = RRSTraceTimeCPU( &bvh ), gpu = RRSTraceTimeGPU( &bvh );
		printf( "SBVH, 32 bins        " );
		fprintf( c, "sbvh[32],%f,%f,%f,%f,%f\n", sah, rrs, epo, cpu, gpu );
		printstat( sah, rrs, epo, cpu, gpu );
		bvh.Optimize( 50 );
		sah = bvh.SAHCost(), rrs = RRSTraceCost( &bvh ), epo = 0;
	#ifdef CALCULATE_EPO
		epo = bvh.EPOCost();
	#endif
		cpu = RRSTraceTimeCPU( &bvh ), gpu = RRSTraceTimeGPU( &bvh );
		printf( "SBVH optimized       " );
		fprintf( c, "sbvh[32] optimized,%f,%f,%f,%f,%f\n", sah, rrs, epo, cpu, gpu );
		printstat( sah, rrs, epo, cpu, gpu );
		bvh.Optimize( 50 );
	}
	{
		BVH bvh;
		char t[] = BEST_BINNED_BVH;
		printf( "SBVH, optimal bins   " );
		if (!bvh.Load( t, tris, triCount )) printf( "FILE NOT FOUND.\n" ); else
		{
			float sah = bvh.SAHCost(), rrs = RRSTraceCost( &bvh ), epo = 0;
		#ifdef CALCULATE_EPO
			epo = bvh.EPOCost();
		#endif
			float cpu = RRSTraceTimeCPU( &bvh ), gpu = RRSTraceTimeGPU( &bvh );
			fprintf( c, "sbvh best bins,%f,%f,%f,%f,%f\n", sah, rrs, epo, cpu, gpu );
			printstat( sah, rrs, epo, cpu, gpu );
		}
	}
	{
		BVH bvh;
		char t[] = OPTIMIZED_BVH;
		printf( "SBVH RRSopt (ours)   " );
		if (!bvh.Load( t, tris, triCount )) printf( "FILE NOT FOUND.\n" ); else
		{
			float sah = bvh.SAHCost(), rrs = RRSTraceCost( &bvh ), epo = 0;
		#ifdef CALCULATE_EPO
			epo = bvh.EPOCost();
		#endif
			float cpu = RRSTraceTimeCPU( &bvh ), gpu = RRSTraceTimeGPU( &bvh );
			fprintf( c, "sbvh (ours),%f,%f,%f,%f,%f\n", sah, rrs, epo, cpu, gpu );
			printstat( sah, rrs, epo, cpu, gpu );
		}
	}
	fclose( c );

	// TODO:
	// Check if other binned builders show similar behavior for bin count

#endif

	return 0;
}