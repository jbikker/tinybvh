#define TINYBVH_IMPLEMENTATION
#include "tiny_bvh.h"

// 'screen resolution': see tiny_bvh_fenster.cpp; this program traces the
// same rays, but without visualization - just performance statistics.
#define SCRWIDTH	480
#define SCRHEIGHT	320

// GPU ray tracing
#define ENABLE_OPENCL

// tests to perform
// #define BUILD_MIDPOINT
#define BUILD_REFERENCE
#define BUILD_DOUBLE
#define BUILD_AVX
#define BUILD_NEON
#define BUILD_SBVH
// #define REFIT_BVH2
#define REFIT_MBVH4
#define REFIT_MBVH8
// #define BUILD_AVX_SBVH
#define TRAVERSE_2WAY_ST
#define TRAVERSE_ALT2WAY_ST
#define TRAVERSE_SOA2WAY_ST
#define TRAVERSE_4WAY
#define TRAVERSE_2WAY_DBL
#define TRAVERSE_CWBVH
#define TRAVERSE_2WAY_MT
#define TRAVERSE_2WAY_MT_PACKET
#define TRAVERSE_OPTIMIZED_ST
#define TRAVERSE_4WAY_OPTIMIZED
// #define EMBREE_BUILD // win64-only for now.
// #define EMBREE_TRAVERSE // win64-only for now.

// GPU rays: only if ENABLE_OPENCL is defined.
#define GPU_2WAY
#define GPU_4WAY
#define GPU_CWBVH

using namespace tinybvh;

#ifdef _MSC_VER
#include "stdio.h"		// for printf
#include "stdlib.h"		// for rand
#else
#include <cstdio>
#endif
#ifdef _WIN32
#include <intrin.h>		// for __cpuidex
#elif defined ENABLE_OPENCL
#undef ENABLE_OPENCL
#endif
#if defined(__GNUC__) && defined(__x86_64__)
#include <cpuid.h>
#endif
#ifdef __EMSCRIPTEN__
#include <emscripten/version.h> // for __EMSCRIPTEN_major__, __EMSCRIPTEN_minor__
#endif

bvhvec4* triangles = 0;
#include <fstream>
int verts = 0;
float traceTime, buildTime, refitTime, * refDist = 0, * refDistFull = 0;
unsigned refOccluded[3] = {}, * refOccl[3] = {};
unsigned Nfull, Nsmall;
Ray* fullBatch[3], * smallBatch[3];
Ray* shadowBatch[3];
#ifdef DOUBLE_PRECISION_SUPPORT
RayEx* doubleBatch[3];
#endif

// bvh layouts
BVH* bvh = new BVH();
BVH* ref_bvh = new BVH();
BVH_Verbose* bvh_verbose = 0;
BVH_Double* bvh_double = new BVH_Double();
BVH_SoA* bvh_soa = 0;
BVH_GPU* bvh_gpu = 0;
MBVH<4>* bvh4 = 0;
BVH4_CPU* bvh4_cpu = 0;
BVH4_GPU* bvh4_gpu = 0;
BVH8_CWBVH* cwbvh = 0;
enum { _DEFAULT = 1, _BVH, _VERBOSE, _DOUBLE, _SOA, _GPU2, _BVH4, _CPU4, _GPU4, _BVH8, _CWBVH };

#if defined EMBREE_BUILD || defined EMBREE_TRAVERSE
#include "embree4/rtcore.h"
static RTCScene embreeScene;
void embreeError( void* userPtr, enum RTCError error, const char* str )
{
	printf( "error %d: %s\n", error, str );
}
#endif

#ifdef ENABLE_OPENCL
#define TINY_OCL_IMPLEMENTATION
#include "tiny_ocl.h"
#endif

float uniform_rand() { return (float)rand() / (float)RAND_MAX; }

#include <chrono>
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

float TestPrimaryRays( uint32_t layout, unsigned N, unsigned passes, float* avgCost = 0 )
{
	// Primary rays: coherent batch of rays from a pinhole camera. One ray per
	// pixel, organized in tiles to further increase coherence.
	Timer t;
	for (int view = 0; view < 3; view++)
	{
		Ray* batch = N == Nsmall ? smallBatch[view] : fullBatch[view];
		for (unsigned i = 0; i < N; i++) batch[i].hit.t = 1e30f;
	}
	uint32_t travCost = 0;
	for (unsigned pass = 0; pass < passes + 1; pass++)
	{
		uint32_t view = pass == 0 ? 0 : (3 - pass); // 0, 2, 1, 0
		Ray* batch = N == Nsmall ? smallBatch[view] : fullBatch[view];
		if (pass == 1) t.reset(); // first pass is cache warming
		switch (layout)
		{
		case _BVH: for (unsigned i = 0; i < N; i++) travCost += bvh->Intersect( batch[i] ); break;
		case _DEFAULT: for (unsigned i = 0; i < N; i++) travCost += ref_bvh->Intersect( batch[i] ); break;
		case _GPU2: for (unsigned i = 0; i < N; i++) travCost += bvh_gpu->Intersect( batch[i] ); break;
		case _CPU4: for (unsigned i = 0; i < N; i++) travCost += bvh4_cpu->Intersect( batch[i] ); break;
		case _GPU4: for (unsigned i = 0; i < N; i++) travCost += bvh4_gpu->Intersect( batch[i] ); break;
		#ifdef BVH_USEAVX
		case _CWBVH: for (unsigned i = 0; i < N; i++) travCost += cwbvh->Intersect( batch[i] ); break;
		case _SOA: for (unsigned i = 0; i < N; i++) travCost += bvh_soa->Intersect( batch[i] ); break;
		#endif
		default: break;
		};
	}
	if (avgCost) *avgCost = travCost / (float)(3 * N);
	return t.elapsed() / passes;
}

#ifdef DOUBLE_PRECISION_SUPPORT

float TestPrimaryRaysEx( unsigned N, unsigned passes, float* avgCost = 0 )
{
	// Primary rays: coherent batch of rays from a pinhole camera.
	// Double-precision version.
	Timer t;
	for (int view = 0; view < 3; view++)
	{
		RayEx* batch = doubleBatch[view];
		for (unsigned i = 0; i < N; i++) batch[i].hit.t = 1e30f;
	}
	uint32_t travCost = 0;
	for (unsigned pass = 0; pass < passes + 1; pass++)
	{
		uint32_t view = pass == 0 ? 0 : (3 - pass); // 0, 2, 1, 0
		RayEx* batch = doubleBatch[view];
		if (pass == 1) t.reset(); // first pass is cache warming
		for (unsigned i = 0; i < N; i++) travCost += bvh_double->Intersect( batch[i] );
	}
	if (avgCost) *avgCost = travCost / (float)(3 * N);
	return t.elapsed() / passes;
}

void ValidateTraceResultEx( float* ref, unsigned N, unsigned line )
{
	float refSum = 0;
	double batchSum = 0;
	for (unsigned i = 0; i < N; i += 4)
		refSum += ref[i] == 1e30f ? 100 : ref[i],
		batchSum += doubleBatch[0][i].hit.t == 1e300 ? 100 : doubleBatch[0][i].hit.t;
	if (fabs( refSum - (float)batchSum ) / refSum < 0.0001f) return;
	fprintf( stderr, "Validation failed on line %i.\n", line );
	exit( 1 );
}

#endif

float TestShadowRays( uint32_t layout, unsigned N, unsigned passes )
{
	// Shadow rays: coherent batch of rays from a single point to 'far away'. Shadow
	// rays terminate on the first hit, and don't need sorted order. They also don't
	// store intersection information, and are therefore expected to be faster than
	// primary rays.
	Timer t;
	unsigned occluded = 0;
	for (unsigned pass = 0; pass < passes + 1; pass++)
	{
		uint32_t view = pass == 0 ? 0 : (3 - pass); // 0, 2, 1, 0
		Ray* batch = shadowBatch[view];
		if (pass == 1) t.reset(); // first pass is cache warming
		occluded = 0;
		switch (layout)
		{
		case _DEFAULT: for (unsigned i = 0; i < N; i++) occluded += bvh->IsOccluded( batch[i] ); break;
		#ifdef BVH_USEAVX
		case _SOA: for (unsigned i = 0; i < N; i++) occluded += bvh_soa->IsOccluded( batch[i] ); break;
		#endif
		case _GPU2: for (unsigned i = 0; i < N; i++) occluded += bvh_gpu->IsOccluded( batch[i] ); break;
		case _CPU4: for (unsigned i = 0; i < N; i++) occluded += bvh4_cpu->IsOccluded( batch[i] ); break;
		default: break;
		}
	}
	// Shadow ray validation: The compacted triangle format used by some intersection
	// kernels will lead to some diverging results. We check if no more than about
	// 1/1000 checks differ. Shadow rays also use an origin offset, based on scene
	// extend, to account for limited floating point accuracy.
	if (abs( (int)occluded - (int)refOccluded[0] ) > 500) // allow some slack, we're using various tri intersectors
	{
		fprintf( stderr, "\nValidation for shadow rays failed (%i != %i).\n", (int)occluded, (int)refOccluded[0] );
		exit( 1 );
	}
	return t.elapsed() / passes;
}

void ValidateTraceResult( float* ref, unsigned N, unsigned line )
{
	float refSum = 0, batchSum = 0;
	Ray* batch = N == Nsmall ? smallBatch[0] : fullBatch[0];
	for (unsigned i = 0; i < N; i += 4)
		refSum += ref[i] == 1e30f ? 100 : ref[i],
		batchSum += batch[i].hit.t == 1e30f ? 100 : batch[i].hit.t;
	float diff = fabs( refSum - batchSum );
	if (diff / refSum > 0.01f)
	{
	#if 1
		printf( "!! Validation failed on line %i: %.1f != %.1f\n", line, refSum, batchSum );
	#else
		fprintf( stderr, "Validation failed on line %i - dumping img.raw.\n", line );
		int step = (N == SCRWIDTH * SCRHEIGHT ? 1 : 16);
		unsigned char pixel[SCRWIDTH * SCRHEIGHT];
		for (unsigned i = 0, ty = 0; ty < SCRHEIGHT / 4; ty++) for (unsigned tx = 0; tx < SCRWIDTH / 4; tx++)
		{
			for (unsigned y = 0; y < 4; y++) for (unsigned x = 0; x < 4; x++, i += step)
			{
				float col = batch[i].hit.t == 1e30f ? 0 : batch[i].hit.t;
				pixel[tx * 4 + x + (ty * 4 + y) * SCRWIDTH] = (unsigned char)((int)(col * 0.1f) & 255);
			}
		}
		std::fstream s{ "img.raw", s.binary | s.out };
		s.seekp( 0 );
		s.write( (char*)&pixel, SCRWIDTH * SCRHEIGHT );
		s.close();
		exit( 1 );
	#endif
	}
}

// Multi-threading
#include <atomic>
#include <thread>
#include <vector>

static unsigned threadCount = std::thread::hardware_concurrency();
static std::atomic<int> batchIdx( 0 );

#if defined(TRAVERSE_2WAY_MT) || defined(ENABLE_OPENCL)

void IntersectBvhWorkerThread( int batchCount, Ray* fullBatch, int threadIdx )
{
	int batch = threadIdx;
	while (batch < batchCount)
	{
		const int batchStart = batch * 10000;
		for (int i = 0; i < 10000; i++) bvh->Intersect( fullBatch[batchStart + i] );
		batch = batchIdx++;
	}
}

#endif

#ifdef TRAVERSE_2WAY_MT_PACKET

void IntersectBvh256WorkerThread( int batchCount, Ray* fullBatch, int threadIdx )
{
	int batch = threadIdx;
	while (batch < batchCount)
	{
		const int batchStart = batch * 30 * 256;
		for (int i = 0; i < 30; i++) bvh->Intersect256Rays( fullBatch + batchStart + i * 256 );

		batch = batchIdx++;
	}
}

#endif

#ifdef BVH_USEAVX

void IntersectBvh256SSEWorkerThread( int batchCount, Ray* fullBatch, int threadIdx )
{
	int batch = threadIdx;
	while (batch < batchCount)
	{
		const int batchStart = batch * 30 * 256;
		for (int i = 0; i < 30; i++) bvh->Intersect256RaysSSE( fullBatch + batchStart + i * 256 );

		batch = batchIdx++;
	}
}

#endif

int main()
{
	int minor = TINY_BVH_VERSION_MINOR;
	int major = TINY_BVH_VERSION_MAJOR;
	int sub = TINY_BVH_VERSION_SUB;
	printf( "tiny_bvh version %i.%i.%i performance statistics ", major, minor, sub );

	// determine compiler
#ifdef _MSC_VER
	printf( "(MSVC %i build)\n", _MSC_VER );
#elif defined __EMSCRIPTEN__
	// EMSCRIPTEN needs to be before clang or gcc
	printf( "(emcc %i.%i build)\n", __EMSCRIPTEN_major__, __EMSCRIPTEN_minor__ );
#elif defined __clang__
	printf( "(clang %i.%i build)\n", __clang_major__, __clang_minor__ );
#elif defined __GNUC__
	printf( "(gcc %i.%i build)\n", __GNUC__, __GNUC_MINOR__ );
#else
	printf( "\n" );
#endif

	// determine what CPU is running the tests.
#if (defined(__x86_64__) || defined(_M_X64)) && (defined (_WIN32) || defined(__GNUC__))
	char model[64]{};
	for (unsigned i = 0; i < 3; ++i)
	{
	#ifdef _WIN32
		__cpuidex( (int*)(model + i * 16), i + 0x80000002, 0 );
	#elif defined(__GNUC__)
		__get_cpuid( i + 0x80000002,
			(unsigned*)model + i * 4 + 0, (unsigned*)model + i * 4 + 1,
			(unsigned*)model + i * 4 + 2, (unsigned*)model + i * 4 + 3 );
	#endif
	}
	printf( "running on %s\n", model );
#endif
	printf( "----------------------------------------------------------------\n" );

#ifdef ENABLE_OPENCL

	// load and compile the OpenCL kernel code
	// This also triggers OpenCL init and device identification.
	tinyocl::Kernel ailalaine_kernel( "traverse.cl", "batch_ailalaine" );
	tinyocl::Kernel gpu4way_kernel( "traverse.cl", "batch_gpu4way" );
	tinyocl::Kernel cwbvh_kernel( "traverse.cl", "batch_cwbvh" );
	printf( "----------------------------------------------------------------\n" );

#endif

	// load raw vertex data for Crytek's Sponza
	const std::string scene = "cryteksponza.bin";
	std::string filename{ "./testdata/" };
	filename += scene;
	std::fstream s{ filename, s.binary | s.in };
	s.seekp( 0 );
	s.read( (char*)&verts, 4 );
	printf( "Loading triangle data (%i tris).\n", verts );
	verts *= 3, triangles = (bvhvec4*)tinybvh::malloc64( verts * sizeof( bvhvec4 ) );
	s.read( (char*)triangles, verts * 16 );

	// setup view pyramid for a pinhole camera:
	// eye, p1 (top-left), p2 (top-right) and p3 (bottom-left)
	bvhvec3 eyes[3] = {
		bvhvec3( -15.24f, 21.5f, 2.54f ),
		bvhvec3( -34, 5, 11.26f ),
		bvhvec3( -1.3, 4.96, 12.28 )
	}, eye = eyes[0];
	bvhvec3 views[3] = {
		tinybvh_normalize( bvhvec3( 0.826f, -0.438f, -0.356f ) ),
		tinybvh_normalize( bvhvec3( 0.9427, 0.0292, -0.3324 ) ),
		tinybvh_normalize( bvhvec3( -0.9886, 0.0507, -0.1419 ) )
	}, view = views[0];
	bvhvec3 right = tinybvh_normalize( tinybvh_cross( bvhvec3( 0, 1, 0 ), view ) );
	bvhvec3 up = 0.8f * tinybvh_cross( view, right ), C = eye + 2 * view;
	bvhvec3 p1 = C - right + up, p2 = C + right + up, p3 = C - right - up;

	// generate primary rays in a cacheline-aligned buffer - and, for data locality:
	// organized in 4x4 pixel tiles, 16 samples per pixel, so 256 rays per tile.
	// one set for each camera position / direction.

	for (int i = 0; i < 3; i++)
	{
		Nfull = Nsmall = 0;
		fullBatch[i] = (Ray*)tinybvh::malloc64( SCRWIDTH * SCRHEIGHT * 16 * sizeof( Ray ) );
		smallBatch[i] = (Ray*)tinybvh::malloc64( SCRWIDTH * SCRHEIGHT * 2 * sizeof( Ray ) );
	#ifdef DOUBLE_PRECISION_SUPPORT
		doubleBatch[i] = (RayEx*)tinybvh::malloc64( SCRWIDTH * SCRHEIGHT * 2 * sizeof( RayEx ) );
	#endif
		for (int ty = 0; ty < SCRHEIGHT / 4; ty++) for (int tx = 0; tx < SCRWIDTH / 4; tx++)
		{
			for (int y = 0; y < 4; y++) for (int x = 0; x < 4; x++)
			{
				int pixel_x = tx * 4 + x;
				int pixel_y = ty * 4 + y;
				for (int s = 0; s < 16; s++) // 16 samples per pixel
				{
					float u = (float)(pixel_x * 4 + (s & 3)) / (SCRWIDTH * 4);
					float v = (float)(pixel_y * 4 + (s >> 2)) / (SCRHEIGHT * 4);
					bvhvec3 P = p1 + u * (p2 - p1) + v * (p3 - p1);
					fullBatch[i][Nfull++] = Ray( eye, tinybvh_normalize( P - eye ) );
					if ((s & 7) == 0)
					{
						smallBatch[i][Nsmall] = fullBatch[i][Nfull - 1];
					#ifdef DOUBLE_PRECISION_SUPPORT
						tinybvh::bvhdbl3 O = smallBatch[i][Nsmall].O;
						tinybvh::bvhdbl3 D = smallBatch[i][Nsmall].D;
						doubleBatch[i][Nsmall] = RayEx( O, D );
					#endif
						Nsmall++;
					}
				}
			}
		}
	}

	//  T I N Y _ B V H   P E R F O R M A N C E   M E A S U R E M E N T S

	Timer t;

	// measure single-core bvh construction time - warming caches
	printf( "BVH construction speed\n" );
	printf( "warming caches...\n" );
	bvh->Build( triangles, verts / 3 );

#ifdef BUILD_MIDPOINT

	// measure single-core bvh construction time - quick bvh builder
	printf( "- quick bvh builder: " );
	t.reset();
	for (int pass = 0; pass < 3; pass++) bvh->BuildQuick( triangles, verts / 3 );
	buildTime = t.elapsed() / 3.0f;
	printf( "%7.2fms for %7i triangles ", buildTime * 1000.0f, verts / 3 );
	printf( "- %6i nodes, SAH=%.2f\n", bvh->usedNodes, bvh->SAHCost() );

#endif

	float avgCost;

#ifdef BUILD_REFERENCE

	// measure single-core bvh construction time - reference builder
	printf( "- reference builder: " );
	t.reset();
	for (int pass = 0; pass < 3; pass++) bvh->Build( triangles, verts / 3 );
	buildTime = t.elapsed() / 3.0f;
	TestPrimaryRays( _BVH, Nsmall, 3, &avgCost );
	printf( "%7.2fms for %7i triangles ", buildTime * 1000.0f, verts / 3 );
	printf( "- %6i nodes, SAH=%.2f, rayCost=%.2f\n", bvh->usedNodes, bvh->SAHCost(), avgCost );

#endif

#if defined BUILD_DOUBLE && defined DOUBLE_PRECISION_SUPPORT

	// measure single-core bvh construction time - double-precision builder
	printf( "- 'double' builder:  " );
	t.reset();
	tinybvh::bvhdbl3* triEx = (tinybvh::bvhdbl3*)tinybvh::malloc64( verts * sizeof( tinybvh::bvhdbl3 ) );
	for (int i = 0; i < verts; i++)
		triEx[i].x = (double)triangles[i].x,
		triEx[i].y = (double)triangles[i].y,
		triEx[i].z = (double)triangles[i].z;
	bvh_double->Build( triEx, verts / 3 );
	buildTime = t.elapsed();
	TestPrimaryRaysEx( Nsmall, 3, &avgCost );
	printf( "%7.2fms for %7i triangles ", buildTime * 1000.0f, verts / 3 );
	printf( "- %6i nodes, SAH=%.2f, rayCost=%.2f\n", bvh->usedNodes, bvh->SAHCost(), avgCost );

#endif

#if defined BUILD_AVX && defined BVH_USEAVX

	// measure single-core bvh construction time - AVX builder
	printf( "- fast AVX builder:  " );
	t.reset();
	for (int pass = 0; pass < 3; pass++) bvh->BuildAVX( triangles, verts / 3 );
	buildTime = t.elapsed() / 3.0f;
	TestPrimaryRays( _BVH, Nsmall, 3, &avgCost );
	printf( "%7.2fms for %7i triangles ", buildTime * 1000.0f, verts / 3 );
	printf( "- %6i nodes, SAH=%.2f, rayCost=%.2f\n", bvh->usedNodes, bvh->SAHCost(), avgCost );

#endif
    
#if defined BUILD_NEON && defined BVH_USENEON

    // measure single-core bvh construction time - NEON builder
    printf( "- fast NEON builder:  " );
    t.reset();
    for (int pass = 0; pass < 3; pass++) bvh->BuildNEON( triangles, verts / 3 );
    buildTime = t.elapsed() / 3.0f;
    TestPrimaryRays( _BVH, Nsmall, 3, &avgCost );
    printf( "%7.2fms for %7i triangles ", buildTime * 1000.0f, verts / 3 );
    printf( "- %6i nodes, SAH=%.2f, rayCost=%.2f\n", bvh->usedNodes, bvh->SAHCost(), avgCost );

#endif

#ifdef BUILD_SBVH

	// measure single-core bvh construction time - AVX builder
	printf( "- HQ (SBVH) builder: " );
	t.reset();
	for (int pass = 0; pass < 2; pass++) bvh->BuildHQ( triangles, verts / 3 );
	buildTime = t.elapsed() / 2.0f;
	TestPrimaryRays( _BVH, Nsmall, 3, &avgCost );
	printf( "%7.2fms for %7i triangles ", buildTime * 1000.0f, verts / 3 );
	printf( "- %6i nodes, SAH=%.2f, rayCost=%.2f\n", bvh->usedNodes, bvh->SAHCost(), avgCost );

#endif

	// measure single-core bvh construction time - warming caches
	printf( "BVH refitting speed\n" );

#ifdef REFIT_BVH2

	// measure single-core bvh refit time
	printf( "- BVH2 refitting: " );
	{
		BVH tmpBVH;
		tmpBVH.Build( triangles, verts / 3 );
		for (int pass = 0; pass < 10; pass++)
		{
			if (pass == 1) t.reset();
			tmpBVH.Refit();
		}
		refitTime = t.elapsed() / 9.0f;
	}
	printf( "%7.2fms for %7i triangles ", refitTime * 1000.0f, verts / 3 );
	printf( "- SAH=%.2f\n", bvh->SAHCost() );

#endif

#ifdef REFIT_MBVH4

	// measure single-core mbvh refit time
	printf( "- BVH4 refitting: " );
	{
		MBVH<4> tmpBVH4;
		tmpBVH4.Build( triangles, verts / 3 );
		for (int pass = 0; pass < 10; pass++)
		{
			if (pass == 1) t.reset();
			tmpBVH4.Refit();
		}
		refitTime = t.elapsed() / 9.0f;
	}
	printf( "%7.2fms for %7i triangles ", refitTime * 1000.0f, verts / 3 );
	printf( "- SAH=%.2f\n", bvh->SAHCost() );

#endif

#ifdef REFIT_MBVH8

	// measure single-core mbvh refit time
	printf( "- BVH8 refitting: " );
	{
		MBVH<8> tmpBVH8;
		tmpBVH8.Build( triangles, verts / 3 );
		for (int pass = 0; pass < 10; pass++)
		{
			if (pass == 1) t.reset();
			tmpBVH8.Refit();
		}
		refitTime = t.elapsed() / 9.0f;
	}
	printf( "%7.2fms for %7i triangles ", refitTime * 1000.0f, verts / 3 );
	printf( "- SAH=%.2f\n", bvh->SAHCost() );

#endif

#if defined BUILD_AVX_SBVH && defined BVH_USEAVX

	// measure single-core bvh construction time - AVX builder
	printf( "- AVX SBVH builder:  " );
	t.reset();
	for (int pass = 0; pass < 3; pass++) bvh->BuildHQAVX( triangles, verts / 3 );
	buildTime = t.elapsed() / 3.0f;
	TestPrimaryRays( _BVH, Nsmall, 3, &avgCost );
	printf( "%7.2fms for %7i triangles ", buildTime * 1000.0f, verts / 3 );
	printf( "- %6i nodes, SAH=%.2f, rayCost=%.2f\n", bvh->usedNodes, bvh->SAHCost(), avgCost );

#endif

#if defined EMBREE_BUILD || defined EMBREE_TRAVERSE

	// convert data to correct format for Embree and build a BVH
	printf( "- Embree builder:    " );
	RTCDevice embreeDevice = rtcNewDevice( NULL );
	rtcSetDeviceErrorFunction( embreeDevice, embreeError, NULL );
	embreeScene = rtcNewScene( embreeDevice );
	RTCGeometry embreeGeom = rtcNewGeometry( embreeDevice, RTC_GEOMETRY_TYPE_TRIANGLE );
	float* vertices = (float*)rtcSetNewGeometryBuffer( embreeGeom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3 * sizeof( float ), verts );
	unsigned* indices = (unsigned*)rtcSetNewGeometryBuffer( embreeGeom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, 3 * sizeof( unsigned ), verts / 3 );
	for (int i = 0; i < verts; i++)
	{
		vertices[i * 3 + 0] = triangles[i].x, vertices[i * 3 + 1] = triangles[i].y;
		vertices[i * 3 + 2] = triangles[i].z, indices[i] = i; // Note: not using shared vertices.
	}
	rtcSetGeometryBuildQuality( embreeGeom, RTC_BUILD_QUALITY_HIGH ); // max quality
	rtcCommitGeometry( embreeGeom );
	rtcAttachGeometry( embreeScene, embreeGeom );
	rtcReleaseGeometry( embreeGeom );
	rtcSetSceneBuildQuality( embreeScene, RTC_BUILD_QUALITY_HIGH );
	t.reset();
	rtcCommitScene( embreeScene ); // assuming this is where (supposedly threaded) BVH build happens.
	buildTime = t.elapsed();
	printf( "%7.2fms for %7i triangles\n", buildTime * 1000.0f, verts / 3 );

#endif

	// report CPU single ray, single-core performance
	printf( "BVH traversal speed - single-threaded\n" );
	ref_bvh->Build( triangles, verts / 3 );

	// estimate correct shadow ray epsilon based on scene extends
	tinybvh::bvhvec4 bmin( 1e30f ), bmax( -1e30f );
	for (int i = 0; i < verts; i++)
		bmin = tinybvh::tinybvh_min( bmin, triangles[i] ),
		bmax = tinybvh::tinybvh_max( bmax, triangles[i] );
	tinybvh::bvhvec3 e = bmax - bmin;
	float maxExtent = tinybvh::tinybvh_max( tinybvh::tinybvh_max( e.x, e.y ), e.z );
	float shadowEpsilon = maxExtent * 5e-7f;

	// setup proper shadow ray batch
	traceTime = TestPrimaryRays( _DEFAULT, Nsmall, 1 ); // just to generate intersection points
	for (int view = 0; view < 3; view++)
	{
		shadowBatch[view] = (Ray*)tinybvh::malloc64( sizeof( Ray ) * Nsmall );
		const tinybvh::bvhvec3 lightPos( 0, 0, 0 );
		for (unsigned i = 0; i < Nsmall; i++)
		{
			float t = tinybvh::tinybvh_min( 1000.0f, smallBatch[view][i].hit.t );
			bvhvec3 I = smallBatch[view][i].O + t * smallBatch[view][i].D;
			bvhvec3 D = tinybvh_normalize( lightPos - I );
			shadowBatch[view][i] = Ray( I + D * shadowEpsilon, D, tinybvh_length( lightPos - I ) - shadowEpsilon );
		}
		// get reference shadow ray query result
		refOccluded[view] = 0, refOccl[view] = new unsigned[Nsmall];
		for (unsigned i = 0; i < Nsmall; i++)
			refOccluded[view] += (refOccl[view][i] = ref_bvh->IsOccluded( shadowBatch[view][i] ) ? 1 : 0);
	}

#ifdef TRAVERSE_2WAY_ST

	// WALD_32BYTE - Have this enabled at all times if validation is desired.
	printf( "- WALD_32BYTE - primary: " );
	traceTime = TestPrimaryRays( _DEFAULT, Nsmall, 3 );
	refDist = new float[Nsmall];
	for (unsigned i = 0; i < Nsmall; i++) refDist[i] = smallBatch[0][i].hit.t;
	printf( "%4.2fM rays in %5.1fms (%7.2fMRays/s), ", (float)Nsmall * 1e-6f, traceTime * 1000, (float)Nsmall / traceTime * 1e-6f );
	traceTime = TestShadowRays( _DEFAULT, Nsmall, 3 );
	printf( "shadow: %5.1fms (%7.2fMRays/s)\n", traceTime * 1000, (float)Nsmall / traceTime * 1e-6f );

#endif

#ifdef TRAVERSE_ALT2WAY_ST

	// GPU
	if (!bvh_gpu)
	{
		bvh_gpu = new BVH_GPU();
		bvh_gpu->BuildHQ( triangles, verts / 3 );
	}
	printf( "- AILA_LAINE  - primary: " );
	traceTime = TestPrimaryRays( _GPU2, Nsmall, 3 );
	ValidateTraceResult( refDist, Nsmall, __LINE__ );
	printf( "%4.2fM rays in %5.1fms (%7.2fMRays/s), ", (float)Nsmall * 1e-6f, traceTime * 1000, (float)Nsmall / traceTime * 1e-6f );
	traceTime = TestShadowRays( _GPU2, Nsmall, 3 );
	printf( "shadow: %5.1fms (%7.2fMRays/s)\n", traceTime * 1000, (float)Nsmall / traceTime * 1e-6f );

#endif

#if defined TRAVERSE_SOA2WAY_ST && defined BVH_USEAVX // BVH_SoA::IsOccluded is not available for NEON yet.

	// SOA
	if (!bvh_soa)
	{
		bvh_soa = new BVH_SoA();
		bvh_soa->BuildHQ( triangles, verts / 3 );
	}
	printf( "- ALT_SOA     - primary: " );
	traceTime = TestPrimaryRays( _SOA, Nsmall, 3 );
	ValidateTraceResult( refDist, Nsmall, __LINE__ );
	printf( "%4.2fM rays in %5.1fms (%7.2fMRays/s), ", (float)Nsmall * 1e-6f, traceTime * 1000, (float)Nsmall / traceTime * 1e-6f );
	traceTime = TestShadowRays( _SOA, Nsmall, 3 );
	printf( "shadow: %5.1fms (%7.2fMRays/s)\n", traceTime * 1000, (float)Nsmall / traceTime * 1e-6f );

#endif

#ifdef TRAVERSE_4WAY

	// BVH4_CPU
	if (!bvh4_cpu)
	{
		bvh4_cpu = new BVH4_CPU();
		bvh4_cpu->BuildHQ( triangles, verts / 3 );
	}
	printf( "- BVH4_AFRA   - primary: " );
	traceTime = TestPrimaryRays( _CPU4, Nsmall, 3 );
	ValidateTraceResult( refDist, Nsmall, __LINE__ );
	printf( "%4.2fM rays in %5.1fms (%7.2fMRays/s), ", (float)Nsmall * 1e-6f, traceTime * 1000, (float)Nsmall / traceTime * 1e-6f );
	traceTime = TestShadowRays( _CPU4, Nsmall, 3 );
	printf( "shadow: %5.1fms (%7.2fMRays/s)\n", traceTime * 1000, (float)Nsmall / traceTime * 1e-6f );

#endif

#if defined TRAVERSE_2WAY_DBL && defined BUILD_DOUBLE && defined DOUBLE_PRECISION_SUPPORT

	// double-precision Rays/BVH
	printf( "- WALD_DOUBLE - primary: " );
	traceTime = TestPrimaryRaysEx( Nsmall, 3 );
	ValidateTraceResultEx( refDist, Nsmall, __LINE__ );
	printf( "%4.2fM rays in %5.1fms (%7.2fMRays/s)\n", (float)Nsmall * 1e-6f, traceTime * 1000, (float)Nsmall / traceTime * 1e-6f );

#endif

#ifdef TRAVERSE_CWBVH

	// CWBVH - Not efficient on CPU.
	if (!cwbvh)
	{
		cwbvh = new BVH8_CWBVH();
		cwbvh->BuildHQ( triangles, verts / 3 );
	}
	printf( "- BVH8/CWBVH  - primary: " );
	traceTime = TestPrimaryRays( _CWBVH, Nsmall, 3 );
	ValidateTraceResult( refDist, Nsmall, __LINE__ );
	printf( "%4.2fM rays in %5.1fms (%7.2fMRays/s)\n", (float)Nsmall * 1e-6f, traceTime * 1000, (float)Nsmall / traceTime * 1e-6f );

#endif

#if defined TRAVERSE_OPTIMIZED_ST || defined TRAVERSE_4WAY_OPTIMIZED

	printf( "Optimized BVH performance - Optimizing... " );
	bvh->Build( triangles, verts / 3 );
	float prevSAH = bvh->SAHCost();
	if (!bvh_verbose)
	{
		bvh_verbose = new BVH_Verbose();
		bvh_verbose->ConvertFrom( *bvh );
	}
	t.reset();
	bvh_verbose->Optimize( 50, true );
	bvh->ConvertFrom( *bvh_verbose );
	TestPrimaryRays( _BVH, Nsmall, 3, &avgCost );
	printf( "done (%.2fs). New: %i nodes, SAH=%.2f to %.2f, rayCost=%.2f\n", t.elapsed(), bvh->NodeCount(), prevSAH, bvh->SAHCost(), avgCost );

#endif

#ifdef TRAVERSE_OPTIMIZED_ST

	// ALT_SOA
	delete bvh_soa;
	// Building a BVH_SoA over an optimized BVH: Careful, do not delete the
	// passed BVH; we use some of its data in the BVH_SoA.
	bvh_soa = new BVH_SoA();
	bvh_soa->ConvertFrom( *bvh );
	printf( "- ALT_SOA     - primary: " );
	traceTime = TestPrimaryRays( _SOA, Nsmall, 3 );
	ValidateTraceResult( refDist, Nsmall, __LINE__ );
	printf( "%4.2fM rays in %5.1fms (%7.2fMRays/s), ", (float)Nsmall * 1e-6f, traceTime * 1000, (float)Nsmall / traceTime * 1e-6f );
	traceTime = TestShadowRays( _SOA, Nsmall, 3 );
	printf( "shadow: %5.1fms (%7.2fMRays/s)\n", traceTime * 1000, (float)Nsmall / traceTime * 1e-6f );

#endif

#ifdef TRAVERSE_4WAY_OPTIMIZED

	// BVH4_AFRA
	delete bvh4;
	delete bvh4_cpu;
	// Building a BVH4_CPU over an optimized BVH: Careful, do not delete the
	// passed BVH; we use some of its data in the BVH4_CPU.
	bvh4 = new MBVH<4>();
	bvh4_cpu = new BVH4_CPU();
	bvh4->ConvertFrom( *bvh );
	bvh4_cpu->ConvertFrom( *bvh4 );
	printf( "- BVH4_AFRA   - primary: " );
	traceTime = TestPrimaryRays( _CPU4, Nsmall, 3 );
	ValidateTraceResult( refDist, Nsmall, __LINE__ );
	printf( "%4.2fM rays in %5.1fms (%7.2fMRays/s), ", (float)Nsmall * 1e-6f, traceTime * 1000, (float)Nsmall / traceTime * 1e-6f );
	traceTime = TestShadowRays( _CPU4, Nsmall, 3 );
	printf( "shadow: %5.1fms (%7.2fMRays/s)\n", traceTime * 1000, (float)Nsmall / traceTime * 1e-6f );

#endif

#ifdef ENABLE_OPENCL

	// report GPU performance
	printf( "BVH traversal speed - GPU (OpenCL)\n" );

	// calculate full res reference distances using threaded traversal on CPU.
	const int batchCount = Nfull / 10000;
	batchIdx = threadCount;
	std::vector<std::thread> threads;
	for (unsigned i = 0; i < Nfull; i++) fullBatch[0][i].hit.t = 1e30f;
	for (uint32_t i = 0; i < threadCount; i++)
		threads.emplace_back( &IntersectBvhWorkerThread, batchCount, fullBatch[0], i );
	for (auto& thread : threads) thread.join();
	refDistFull = new float[Nfull];
	for (unsigned i = 0; i < Nfull; i++) refDistFull[i] = fullBatch[0][i].hit.t;

#ifdef GPU_2WAY

	// trace the rays on GPU using OpenCL
	printf( "- AILA_LAINE  - primary: " );
	if (!bvh_gpu)
	{
		bvh_gpu = new BVH_GPU();
		bvh_gpu->Build( triangles, verts / 3 );
	}
	// create OpenCL buffers for the BVH data calculated by tiny_bvh.h
	tinyocl::Buffer gpuNodes( bvh_gpu->usedNodes * sizeof( BVH_GPU::BVHNode ), bvh_gpu->bvhNode );
	tinyocl::Buffer idxData( bvh_gpu->idxCount * sizeof( unsigned ), bvh_gpu->bvh.primIdx );
	tinyocl::Buffer triData( bvh_gpu->triCount * 3 * sizeof( tinybvh::bvhvec4 ), triangles );
	// synchronize the host-side data to the gpu side
	gpuNodes.CopyToDevice();
	idxData.CopyToDevice();
	triData.CopyToDevice();
	// create rays and send them to the gpu side
	tinyocl::Buffer rayData( Nfull * 64 /* sizeof( tinybvh::Ray ) */ );
	// the size of the ray struct exceeds 64 bytes because of the large Intersection struct.
	// Here we chop this off, since on the GPU side, the ray is precisely 64 bytes.
	for (unsigned i = 0; i < Nfull; i++)
		memcpy( (unsigned char*)rayData.GetHostPtr() + 64 * i, &fullBatch[0][i], 64 );
	rayData.CopyToDevice();
	// create an event to time the OpenCL kernel
	cl_event event;
	cl_ulong startTime, endTime;
	// start timer and start kernel on gpu
	t.reset();
	traceTime = 0;
	ailalaine_kernel.SetArguments( &gpuNodes, &idxData, &triData, &rayData );
	for (int pass = 0; pass < 9; pass++)
	{
		ailalaine_kernel.Run( Nfull, 64, 0, &event ); // for now, todo.
		clWaitForEvents( 1, &event ); // OpenCL kernsl run asynchronously
		clGetEventProfilingInfo( event, CL_PROFILING_COMMAND_START, sizeof( cl_ulong ), &startTime, 0 );
		clGetEventProfilingInfo( event, CL_PROFILING_COMMAND_END, sizeof( cl_ulong ), &endTime, 0 );
		if (pass == 0) continue; // first pass is for cache warming
		traceTime += (endTime - startTime) * 1e-9f; // event timing is in nanoseconds
	}
	// get results from GPU - this also syncs the queue.
	rayData.CopyFromDevice();
	// report on timing
	traceTime /= 8.0f;
	printf( "%4.2fM rays in %5.1fms (%7.2fMRays/s)\n", (float)Nfull * 1e-6f, traceTime * 1000, (float)Nfull / traceTime * 1e-6f );
	// validate GPU ray tracing result
	ValidateTraceResult( refDistFull, Nfull, __LINE__ );

#endif

#ifdef GPU_4WAY

	// trace the rays on GPU using OpenCL
	printf( "- BVH4_GPU    - primary: " );
	if (!bvh4_gpu)
	{
		bvh4_gpu = new BVH4_GPU();
		bvh4_gpu->Build( triangles, verts / 3 );
	}
	// create OpenCL buffers for the BVH data calculated by tiny_bvh.h
	tinyocl::Buffer gpu4Nodes( bvh4_gpu->usedBlocks * sizeof( tinybvh::bvhvec4 ), bvh4_gpu->bvh4Data );
	// synchronize the host-side data to the gpu side
	gpu4Nodes.CopyToDevice();
#ifndef GPU_2WAY // otherwise these already exist.
	// create an event to time the OpenCL kernel
	cl_event event;
	cl_ulong startTime, endTime;
	// create rays and send them to the gpu side
	tinyocl::Buffer rayData( Nfull * 64 /* sizeof( tinybvh::Ray ) */, 0 );
	for (unsigned i = 0; i < Nfull; i++)
		memcpy( (unsigned char*)rayData.GetHostPtr() + 64 * i, &fullBatch[0][i], 64 );
	rayData.CopyToDevice();
#endif
	// start timer and start kernel on gpu
	t.reset();
	traceTime = 0;
	gpu4way_kernel.SetArguments( &gpu4Nodes, &rayData );
	for (int pass = 0; pass < 9; pass++)
	{
		gpu4way_kernel.Run( Nfull, 64, 0, &event ); // for now, todo.
		clWaitForEvents( 1, &event ); // OpenCL kernsl run asynchronously
		clGetEventProfilingInfo( event, CL_PROFILING_COMMAND_START, sizeof( cl_ulong ), &startTime, 0 );
		clGetEventProfilingInfo( event, CL_PROFILING_COMMAND_END, sizeof( cl_ulong ), &endTime, 0 );
		if (pass == 0) continue; // first pass is for cache warming
		traceTime += (endTime - startTime) * 1e-9f; // event timing is in nanoseconds
	}
	// get results from GPU - this also syncs the queue.
	rayData.CopyFromDevice();
	// report on timing
	traceTime /= 8.0f;
	printf( "%4.2fM rays in %5.1fms (%7.2fMRays/s)\n", (float)Nfull * 1e-6f, traceTime * 1000, (float)Nfull / traceTime * 1e-6f );
	// validate GPU ray tracing result
	ValidateTraceResult( refDistFull, Nfull, __LINE__ );

#endif

#ifdef GPU_CWBVH

	// trace the rays on GPU using OpenCL
	printf( "- BVH8/CWBVH  - primary: " );
	if (!cwbvh)
	{
		cwbvh = new BVH8_CWBVH();
		cwbvh->Build( triangles, verts / 3 );
	}
	// create OpenCL buffers for the BVH data calculated by tiny_bvh.h
	tinyocl::Buffer cwbvhNodes( cwbvh->usedBlocks * sizeof( tinybvh::bvhvec4 ), cwbvh->bvh8Data );
#ifdef CWBVH_COMPRESSED_TRIS
	tinyocl::Buffer cwbvhTris( cwbvh->idxCount * 4 * sizeof( tinybvh::bvhvec4 ), cwbvh->bvh8Tris );
#else
	tinyocl::Buffer cwbvhTris( cwbvh->idxCount * 3 * sizeof( tinybvh::bvhvec4 ), cwbvh->bvh8Tris );
#endif
	// synchronize the host-side data to the gpu side
	cwbvhNodes.CopyToDevice();
	cwbvhTris.CopyToDevice();
#if !defined GPU_2WAY && !defined GPU_4WAY // otherwise these already exist.
	// create an event to time the OpenCL kernel
	cl_event event;
	cl_ulong startTime, endTime;
	// create rays and send them to the gpu side
	tinyocl::Buffer rayData( Nfull * 64 /* sizeof( tinybvh::Ray ) */, 0 );
	for (unsigned i = 0; i < Nfull; i++)
		memcpy( (unsigned char*)rayData.GetHostPtr() + 64 * i, &fullBatch[0][i], 64 );
	rayData.CopyToDevice();
#endif
	// start timer and start kernel on gpu
	t.reset();
	traceTime = 0;
	cwbvh_kernel.SetArguments( &cwbvhNodes, &cwbvhTris, &rayData );
	for (int pass = 0; pass < 9; pass++)
	{
		cwbvh_kernel.Run( Nfull, 64, 0, &event ); // for now, todo.
		clWaitForEvents( 1, &event ); // OpenCL kernsl run asynchronously
		clGetEventProfilingInfo( event, CL_PROFILING_COMMAND_START, sizeof( cl_ulong ), &startTime, 0 );
		clGetEventProfilingInfo( event, CL_PROFILING_COMMAND_END, sizeof( cl_ulong ), &endTime, 0 );
		if (pass == 0) continue; // first pass is for cache warming
		traceTime += (endTime - startTime) * 1e-9f; // event timing is in nanoseconds
	}
	// get results from GPU - this also syncs the queue.
	rayData.CopyFromDevice();
	// report on timing
	traceTime /= 8.0f;
	printf( "%4.2fM rays in %5.1fms (%7.2fMRays/s)\n", (float)Nfull * 1e-6f, traceTime * 1000, (float)Nfull / traceTime * 1e-6f );
	// validate GPU ray tracing result
	ValidateTraceResult( refDistFull, Nfull, __LINE__ );

#endif

#endif

	// report threaded CPU performance
	printf( "BVH traversal speed - CPU multi-core\n" );

#ifdef TRAVERSE_2WAY_MT

	// using OpenMP and batches of 10,000 rays
	printf( "- WALD_32BYTE - primary: " );
	for (int pass = 0; pass < 4; pass++)
	{
		if (pass == 1) t.reset(); // first pass is cache warming
		const int batchCount = Nfull / 10000;

		batchIdx = threadCount;
		std::vector<std::thread> threads;
		for (uint32_t i = 0; i < threadCount; i++)
			threads.emplace_back( &IntersectBvhWorkerThread, batchCount, fullBatch[0], i );
		for (auto& thread : threads) thread.join();
	}
	traceTime = t.elapsed() / 3.0f;
	printf( "%4.2fM rays in %5.1fms (%7.2fMRays/s)\n", (float)Nfull * 1e-6f, traceTime * 1000, (float)Nfull / traceTime * 1e-6f );

#endif

#ifdef TRAVERSE_2WAY_MT_PACKET

	// multi-core packet traversal
	printf( "- RayPacket   - primary: " );
	for (int pass = 0; pass < 4; pass++)
	{
		if (pass == 1) t.reset(); // first pass is cache warming
		const int batchCount = Nfull / (30 * 256); // batches of 30 packets of 256 rays

		batchIdx = threadCount;
		std::vector<std::thread> threads;
		for (uint32_t i = 0; i < threadCount; i++)
			threads.emplace_back( &IntersectBvh256WorkerThread, batchCount, fullBatch[0], i );
		for (auto& thread : threads) thread.join();
	}
	traceTime = t.elapsed() / 3.0f;
	printf( "%4.2fM rays in %5.1fms (%7.2fMRays/s)\n", (float)Nfull * 1e-6f, traceTime * 1000, (float)Nfull / traceTime * 1e-6f );

#ifdef BVH_USEAVX

	// trace all rays three times to estimate average performance
	// - coherent distribution, multi-core, packet traversal, SSE version
	printf( "- Packet,SSE  - primary: " );
	for (int pass = 0; pass < 4; pass++)
	{
		if (pass == 1) t.reset(); // first pass is cache warming
		const int batchCount = Nfull / (30 * 256); // batches of 30 packets of 256 rays

		batchIdx = threadCount;
		std::vector<std::thread> threads;
		for (uint32_t i = 0; i < threadCount; i++)
			threads.emplace_back( &IntersectBvh256SSEWorkerThread, batchCount, fullBatch[0], i );
		for (auto& thread : threads) thread.join();
	}
	traceTime = t.elapsed() / 3.0f;
	printf( "%4.2fM rays in %5.1fms (%7.2fMRays/s)\n", (float)Nfull * 1e-6f, traceTime * 1000, (float)Nfull / traceTime * 1e-6f );

#endif

#endif

#if defined EMBREE_TRAVERSE && defined EMBREE_BUILD

	// report threaded CPU performance
	printf( "BVH traversal speed - EMBREE reference\n" );

	// trace all rays three times to estimate average performance
	// - coherent, Embree, single-threaded
	printf( "- Default BVH - primary: " );
	struct RTCRayHit* rayhits = (RTCRayHit*)tinybvh::malloc64( SCRWIDTH * SCRHEIGHT * 16 * sizeof( RTCRayHit ) );
	// copy our rays to Embree format
	for (unsigned i = 0; i < Nfull; i++)
	{
		rayhits[i].ray.org_x = fullBatch[0][i].O.x, rayhits[i].ray.org_y = fullBatch[0][i].O.y, rayhits[i].ray.org_z = fullBatch[0][i].O.z;
		rayhits[i].ray.dir_x = fullBatch[0][i].D.x, rayhits[i].ray.dir_y = fullBatch[0][i].D.y, rayhits[i].ray.dir_z = fullBatch[0][i].D.z;
		rayhits[i].ray.tnear = 0, rayhits[i].ray.tfar = fullBatch[0][i].hit.t;
		rayhits[i].ray.mask = -1, rayhits[i].ray.flags = 0;
		rayhits[i].hit.geomID = RTC_INVALID_GEOMETRY_ID;
		rayhits[i].hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
	}
	for (int pass = 0; pass < 4; pass++)
	{
		if (pass == 1) t.reset(); // first pass is cache warming
		for (int i = 0; i < Nsmall; i++) rtcIntersect1( embreeScene, rayhits + i );
	}
	traceTime = t.elapsed() / 3.0f;
	// retrieve intersection results
	for (unsigned i = 0; i < Nfull; i++)
	{
		fullBatch[0][i].hit.t = rayhits[i].ray.tfar;
		fullBatch[0][i].hit.u = rayhits[i].hit.u, fullBatch[0][i].hit.u = rayhits[i].hit.v;
		fullBatch[0][i].hit.prim = rayhits[i].hit.primID;
	}
	printf( "%4.2fM rays in %5.1fms (%7.2fMRays/s)\n", (float)Nsmall * 1e-6f, traceTime * 1000, (float)Nsmall / traceTime * 1e-6f );
	tinybvh::free64( rayhits );

#endif

	// verify memory management
	delete bvh;
	delete bvh_verbose;
	delete bvh_double;
	delete bvh_soa;
	delete bvh_gpu;
	delete bvh4;
	delete bvh4_cpu;
	delete bvh4_gpu;
	delete cwbvh;

	printf( "all done." );
	return 0;
}