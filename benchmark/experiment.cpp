#include "headers.h"

#if defined _WIN32 && defined _MSC_VER

#define WIN32_LEAN_AND_MEAN
#include "windows.h"

#endif

#ifdef ENABLE_OPENCL
extern tinyocl::Kernel* kernel_nearest;
extern tinyocl::Kernel* kernel_any;
extern tinyocl::Kernel* gpu4way_kernel;
extern tinyocl::Kernel* gpu4way_kernel_any;
extern tinyocl::Kernel* amd4way_kernel;
extern tinyocl::Kernel* amd4way_kernel_any;
extern tinyocl::Kernel* amd8way_kernel;
extern tinyocl::Kernel* amd8way_kernel_any;
extern tinyocl::Kernel* cwbvh_kernel;
extern tinyocl::Kernel* cwbvh_kernel_any;
cl_ulong startTime, endTime;
tinyocl::Buffer* gpuRayData = 0;
tinyocl::Buffer* gpuOrigRays = 0;
#endif

void InitDXR();

extern FILE* csv;

namespace tinybvh
{

extern RTCScene embreeScene;

void Experiment::Init( BVHLayout layout, BuildFlags buildFlags, Scene prims, RaySet rays, ExperimentFlags expFlags, const char* view, const char* rayFile )
{
	if (rays == RaySet::UNSPECIFIED)
	{
		// Accstruc build experiment.
		primSet = prims;
		flags = expFlags;
		if (!cachedPrimSet[primSet]) cachedPrimSet[primSet] = new PrimitiveSet( primSet );
		bvh = new AccStruc( layout, buildFlags ); // actual construction is postponed until ::Run.
		// Create description.
		title = new char[1024];
		snprintf( title, 1024, "BVH BUILD%s - %s - %s (%ik tris)",
			(flags & MULTICORE) ? " (MT)" : ((flags & USE_GPU) ? " (GPU)" : ""),
			bvh->GetDescription(),
			cachedPrimSet[primSet]->GetDescription(),
			cachedPrimSet[primSet]->primCount / 1000 );
	}
	else
	{
		// Accstruc traversal experiment.
		primSet = prims;
		raySet = rays;
		flags = expFlags;
		if (!cachedPrimSet[primSet]) cachedPrimSet[primSet] = new PrimitiveSet( primSet );
		if (!cachedRaySet[raySet]) cachedRaySet[raySet] = new RayDistribution( (RaySet)raySet, cachedPrimSet[primSet] );
		bvh = new AccStruc( layout, buildFlags ); // actual construction is postponed until ::Run.
		// Create description.
		title = new char[1024];
		char features[128] = "";
		if (flags & MULTICORE) if (flags & PACKETS) strncpy( features, " (MT/packets)", 100 ); else strncpy( features, " (MT)", 100 );
		else if (flags & USE_GPU) strncpy( features, " (GPU)", 100 );
		else if (flags & PACKETS) strncpy( features, " (packets)", 100 );
		snprintf( title, 1024, "BVH TRACE%s - %s - %s (%ik tris) - %s",
			features, bvh->GetDescription(),
			cachedPrimSet[primSet]->GetDescription(),
			cachedPrimSet[primSet]->primCount / 1000,
			cachedRaySet[raySet]->GetDescription() );
		// Save a thumbnail of the scene / rayset if requested. 
		// Must be .tga, only supported for single-threaded builds.
		if (view)
		{
			tgaFile = new char[512];
			strncpy( tgaFile, view, 500 );
		}
		// Save the set of rays if requested.
		if (rayFile)
		{
			cachedRaySet[raySet]->WriteToFile( rayFile );
		}
	}
}

Experiment::Experiment( BVHLayout layout, BuildFlags buildFlags, Scene prims, RaySet rays, ExperimentFlags expFlags, const char* view, const char* rayFile )
{
	Init( layout, buildFlags, prims, rays, expFlags, view, rayFile );
}

Experiment::Experiment( BVHLayout layout, const char* baked, Scene prims, RaySet rays, ExperimentFlags expFlags, const char* view, const char* rayFile )
{
	Init(  layout, BAKED, prims, rays, expFlags, view, rayFile );
	bakedBVH = new char[strlen( baked ) + 1];
	strncpy( bakedBVH, baked, strlen( baked ) + 1 );
}

static bool lastWasTrav = false, firstExperiment = true;
static constexpr uint32_t batchAmplifier = 2;
static constexpr int ocl_runs = 200;

void Experiment::Run()
{
#if defined _WIN32 && defined _MSC_VER
	// force processing to one core to improve cache coherence - TODO: gcc.
	const uint32_t hw = (uint32_t)std::thread::hardware_concurrency();
	if (hw >= 4) SetThreadAffinityMask( GetCurrentThread(), 4 );
#endif
	// run actual experiment
	if (raySet != UNSPECIFIED) RunTraceExperiment(); else RunBuildExperiment();
}

void Experiment::RunTraceExperiment()
{
	// Traversal experiment.
	// - Build once - time is irrelevant;
	// - Trace on a single core;
	// - Trace one million rays several times for average trace time.
	if (bakedBVH)
	{
		bvh->LoadBVH( bakedBVH );
	}
	else
	{
		bvh->PrepareBuild();
		bvh->Build( cachedPrimSet[primSet] );
		bvh->PostBuild();
	}
	// emit header
	if (firstExperiment || !lastWasTrav)
	{
		if (csv) fprintf( csv, "ray traversal measurements\n" );
		if (csv) fprintf( csv, "device,scene,tris,bvh,flags,ray set,nearest,runs,any hit,runs,unit\n" );
	}
	lastWasTrav = true;
	firstExperiment = false;
	// write experiment settings to csv
	if (csv)
	{
		if (flags & USE_GPU) fprintf( csv, "gpu," );
		else if (flags & MULTICORE) fprintf( csv, (flags & PACKETS) ? "cpu (MT/packets)" : "cpu (MT)" );
		else fprintf( csv, (flags & PACKETS) ? "cpu (packets)" : "cpu," );
		fprintf( csv, "%s,%i,", cachedPrimSet[primSet]->shrt, cachedPrimSet[primSet]->primCount );
		fprintf( csv, "%s,%s,", bvh->shrt, bvh->flagShrt );
		fprintf( csv, "%s,", cachedRaySet[raySet]->shrt );
		if (csv) fflush( csv );
	}
	// setup rays
	int N = cachedRaySet[raySet]->rayCount;
	char* extensionRays = (char*)malloc64( N * 64 );
	char* shadowRays = (char*)malloc64( N * 64 );
	bvhvec3* O = cachedRaySet[raySet]->O, * D = cachedRaySet[raySet]->D;
	float* tmin = cachedRaySet[raySet]->tmin, * tmax = cachedRaySet[raySet]->tmax;
	for (int i = 0; i < N; i++)
	{
		Ray r( O[i] + D[i] * tmin[i], D[i], tmax[i] - tmin[i] );
		Ray s( O[i] + D[i] * tmin[i], D[i], tmax[i] - tmin[i] );
		memcpy( extensionRays + 64 * i, &r, 64 );
		memcpy( shadowRays + 64 * i, &s, 64 );
	}
	// trace extension rays
	float traceTime, raysPerSecond;
	Timer t;
	int runs = 0;
#ifdef ENABLE_OPENCL
	if (flags & USE_GPU && bvh->layout == GPU_BVH)
	{
		traceTime = RunGPU_BVH2( extensionRays, N, false );
		raysPerSecond = (float)(N * batchAmplifier) / traceTime;
		runs = ocl_runs;
		if (tgaFile) WriteImage( 0, (char*)gpuRayData->GetHostPtr() );
	}
	else if (flags & USE_GPU && bvh->layout == GPU_BVH4)
	{
		traceTime = RunGPU_BVH4( extensionRays, N, false );
		raysPerSecond = (float)(N * batchAmplifier) / traceTime;
		runs = ocl_runs;
		if (tgaFile) WriteImage( 0, (char*)gpuRayData->GetHostPtr() );
	}
	else if (flags & USE_GPU && bvh->layout == CWBVH)
	{
		traceTime = RunGPU_CWBVH( extensionRays, N, false );
		raysPerSecond = (float)(N * batchAmplifier) / traceTime;
		runs = ocl_runs;
		if (tgaFile) WriteImage( 0, (char*)gpuRayData->GetHostPtr() );
	}
	else if (flags & USE_GPU && bvh->layout == AMD_HWRT)
	{
		traceTime = RunGPU_AMD4( extensionRays, N, false );
		raysPerSecond = (float)(N * batchAmplifier) / traceTime;
		runs = ocl_runs;
		if (tgaFile) WriteImage( 0, (char*)gpuRayData->GetHostPtr() );
	}
	else if (flags & USE_GPU && bvh->layout == AMD_HWRT8)
	{
		traceTime = RunGPU_AMD8( extensionRays, N, false );
		raysPerSecond = (float)(N * batchAmplifier) / traceTime;
		runs = ocl_runs;
		if (tgaFile) WriteImage( 0, (char*)gpuRayData->GetHostPtr() );
	}
	else
	#endif
	{
		if (bvh->layout == EMBREE)
		{
			// Embree needs to be handled separately: 
			// we don't want to include ray translation in the measurement.
			RTCRayHit* embreeRay = (RTCRayHit*)malloc64( N * sizeof( RTCRayHit ) );
			for (int i = 0; i < N; i++)
			{
				const Ray& r = *(Ray*)(shadowRays + i * 64);
				embreeRay[i].ray.org_x = r.O.x, embreeRay[i].ray.org_y = r.O.y, embreeRay[i].ray.org_z = r.O.z;
				embreeRay[i].ray.dir_x = r.D.x, embreeRay[i].ray.dir_y = r.D.y, embreeRay[i].ray.dir_z = r.D.z;
				embreeRay[i].ray.tnear = 0, embreeRay[i].ray.tfar = r.hit.t;
				embreeRay[i].ray.mask = -1, embreeRay[i].ray.flags = 0;
				embreeRay[i].hit.geomID = RTC_INVALID_GEOMETRY_ID;
				embreeRay[i].hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
			}
			t.reset();
			while (runs < 5 || t.elapsed() < 1.5f /* at least 5, or whatever fits in a 1.5 seconds. */)
			{
				for (int i = 0; i < N; i++)
				{
					embreeRay[i].ray.tfar = BVH_FAR; // reset
					rtcIntersect1( embreeScene, &embreeRay[i] );
				}
				runs++;
			}
			traceTime = t.elapsed() * (1.0f / runs); // average of runs.
			raysPerSecond = (float)N / traceTime;
			free64( embreeRay );
		}
		else
		{
			// dump image for the ray set, if requested
			if (tgaFile) WriteImage( extensionRays );
			// trace 'first hit' rays on CPU
			bvh->IntersectBatch( extensionRays, N ); // warm caches, precompute data
			t.reset();
			while (runs < 5 || t.elapsed() < 1.5f /* at least 5, or whatever fits in a 1.5 seconds. */)
			{
				if (flags & MULTICORE)
				{
					if (flags & PACKETS) bvh->IntersectBatchMTPackets( extensionRays, N );
					else bvh->IntersectBatchMT( extensionRays, N );
				}
				else
				{
					if (flags & PACKETS) bvh->IntersectBatchPackets( extensionRays, N );
					else bvh->IntersectBatch( extensionRays, N );
				}
				runs++;
			}
			traceTime = t.elapsed() * (1.0f / runs); // average of runs.
			raysPerSecond = (float)N / traceTime;
		}
	}
	float mraysPerSecond = raysPerSecond / RAY_BATCH_SIZE;
	printf( "%s\nfind nearest: %.2fM, ", title, mraysPerSecond );
	int mag = 6; // use the same scale for shadows rays.
	if (csv)
	{
		if (raysPerSecond < 99000) { mag = 3; fprintf( csv, "%.2f,", raysPerSecond / 1000 ); } // report in KRays/s
		else /* if (raysPerSecond < 99000000) */ fprintf( csv, "%.2f,", raysPerSecond / 1000000 ); // report in MRays/s
		// else { mag = 9; fprintf( csv, "%.2f,", raysPerSecond / 1000000000 ); } // report in BRays/s
		fprintf( csv, "%i,", runs );
		fflush( csv );
	}
	// trace 'any hit' rays
	runs = 0;
	if (bvh->layout == MADMANN91)
	{
		printf( "any hit: n/a\n" );
		if (csv) fprintf( csv, "n/a\n" );
		if (csv) fflush( csv );
	}
	else
	{
	#ifdef ENABLE_OPENCL
		if (flags & USE_GPU && bvh->layout == GPU_BVH)
		{
			traceTime = RunGPU_BVH2( extensionRays, N, true );
			raysPerSecond = (float)(N * batchAmplifier) / traceTime;
			runs = 40;
		}
		else if (flags & USE_GPU && bvh->layout == GPU_BVH4)
		{
			traceTime = RunGPU_BVH4( extensionRays, N, true );
			raysPerSecond = (float)(N * batchAmplifier) / traceTime;
			runs = 40;
		}
		else if (flags & USE_GPU && bvh->layout == CWBVH)
		{
			traceTime = RunGPU_CWBVH( extensionRays, N, true );
			raysPerSecond = (float)(N * batchAmplifier) / traceTime;
			runs = 40;
		}
		else if (flags & USE_GPU && bvh->layout == AMD_HWRT)
		{
			traceTime = RunGPU_AMD4( extensionRays, N, true );
			raysPerSecond = (float)(N * batchAmplifier) / traceTime;
			runs = 40;
		}
		else if (flags & USE_GPU && bvh->layout == AMD_HWRT8)
		{
			traceTime = RunGPU_AMD8( extensionRays, N, true );
			raysPerSecond = (float)(N * batchAmplifier) / traceTime;
			runs = 40;
		}
		else
		#endif
			if (bvh->layout == EMBREE)
			{
				// Embree needs to be handled separately: 
				// we don't want to include ray translation in the measurement.
				RTCRay* embreeRay = (RTCRay*)malloc64( N * sizeof( RTCRay ) );
				for (int i = 0; i < N; i++)
				{
					const Ray& r = *(Ray*)(shadowRays + i * 64);
					embreeRay[i].org_x = r.O.x, embreeRay[i].org_y = r.O.y, embreeRay[i].org_z = r.O.z;
					embreeRay[i].dir_x = r.D.x, embreeRay[i].dir_y = r.D.y, embreeRay[i].dir_z = r.D.z;
					embreeRay[i].tnear = 0, embreeRay[i].tfar = r.hit.t;
					embreeRay[i].mask = -1, embreeRay[i].flags = RTC_RAY_QUERY_FLAG_COHERENT;
				}
				t.reset();
				while (runs < 5 || t.elapsed() < 1.5f /* at least 5, or whatever fits in a 1.5 seconds. */)
				{
					for (int i = 0; i < N; i++)
					{
						embreeRay[i].tfar = BVH_FAR; // reset
						rtcOccluded1( embreeScene, &embreeRay[i] );
					}
					runs++;
				}
				traceTime = t.elapsed() * (1.0f / runs); // average of runs.
				raysPerSecond = (float)N / traceTime;
				free64( embreeRay );
			}
			else
			{
				// trace 'any hit' rays on CPU
				bvh->OcclusionBatch( shadowRays, N ); // warm caches
				t.reset();
				while (runs < 5 || t.elapsed() < 1.5f /* at least 5, or whatever fits in a 1.5 seconds. */)
				{
					if (flags & MULTICORE)
					{
						if (flags & PACKETS) bvh->OcclusionBatchMTPackets( shadowRays, N );
						else bvh->OcclusionBatchMT( shadowRays, N );
					}
					else
					{
						if (flags & PACKETS) bvh->OcclusionBatchPackets( shadowRays, N );
						else bvh->OcclusionBatch( shadowRays, N );
					}
					runs++;
				}
				traceTime = t.elapsed() * (1.0f / runs); // average of runs.
				raysPerSecond = (float)N / traceTime;
			}
		mraysPerSecond = raysPerSecond / RAY_BATCH_SIZE;
		printf( "any hit: %.2fM\n", mraysPerSecond );
		if (csv)
		{
			if (mag == 3) fprintf( csv, "%.2f,%i,K\n", raysPerSecond / 1000, runs );
			else /* if (mag == 6) */ fprintf( csv, "%.2f,%i,M\n", raysPerSecond / 1000000, runs );
			// else if (mag == 9) fprintf( csv, "%.2f,%i,B\n", raysPerSecond / 1000000000, runs );
			fflush( csv );
		}
	}
	// cleanup
	free64( extensionRays );
	free64( shadowRays );
}

void Experiment::RunBuildExperiment()
{
	// Accstruc build experiment.
	// - Build several times for average build time;
	// - Assess SAH and EPO.
	if (bakedBVH)
	{
		bvh->LoadBVH( bakedBVH );
	}
	else
	{
		bvh->PrepareBuild();
		bvh->Build( cachedPrimSet[primSet] ); // warm caches
		bvh->PostBuild();
	}
	// emit header
	if (firstExperiment || lastWasTrav)
	{
		if (csv) fprintf( csv, "bvh build measurements\n" );
		if (csv) fprintf( csv, "device,scene,tris,bvh,flags,time (ms),sah,epo,runs\n" );
	}
	lastWasTrav = false;
	firstExperiment = false;
	// write experiment settings to csv
	if (csv)
	{
		if (flags & USE_GPU) fprintf( csv, "gpu," ); else if (flags & MULTICORE) fprintf( csv, "cpu (MT)," ); else fprintf( csv, "cpu," );
		fprintf( csv, "%s,%i,", cachedPrimSet[primSet]->shrt, cachedPrimSet[primSet]->primCount );
		fprintf( csv, "%s,%s,", bvh->shrt, bvh->flagShrt );
		if (csv) fflush( csv );
	}
	bvh->PrepareBuild();
	Timer t;
	int minruns = 5;
	int runs = 0;
	while (runs < minruns || t.elapsed() < 1.5f /* at least 'minruns', or whatever fits in a 1.5 seconds. */)
	{
		bvh->Build( cachedPrimSet[primSet] );
		runs++;
		if (t.elapsed() > 5.0f) break; // let's stop if builds take very long.
	}
	buildTime = t.elapsed() * (1.0f / runs); // average of runs.
	bvh->PostBuild();
	// report
	printf( "%s\nbuild time: %.2fms ", title, buildTime * 1000.0f );
	float sahCost = bvh->SAHCost();
	float epoCost = bvh->EPOCost();
	if (bvh->layout == EMBREE) printf( "SAH: n/a, EPO: n/a\n" );
	else if (bvh->layout == MADMANN91) printf( "SAH: %.3f, EPO: n/a\n", sahCost );
	else printf( "SAH: %.3f, EPO: %.2f\n", sahCost, epoCost );
	if (csv)
	{
		if (bvh->layout == EMBREE) fprintf( csv, "%.2f,n/a,n/a,%i\n", buildTime * 1000.0f, runs );
		else if (bvh->layout == MADMANN91) fprintf( csv, "%.2f,%.3f,n/a,%i\n", buildTime * 1000.0f, sahCost, runs );
		else fprintf( csv, "%.2f,%.3f,%.3f,%i\n", buildTime * 1000.0f, sahCost, epoCost, runs );
		fflush( csv );
	}
}

void Experiment::WriteImage( char* raySet, const char* tracedRays )
{
	const int imgSize = (int)sqrtf( RAY_BATCH_SIZE );
	const int h[] = { 3 << 16, 8 << 24, 0, imgSize + (imgSize << 16), 8 };
	FILE* f = fopen( tgaFile, "wb" );
	fwrite( h, 2, 9, f ); // minimalist greyscale tga header
	const bvhvec3 sceneExtent = bvh->SceneExtent();
	float distScale = 384.0f / sceneExtent[tinybvh_maxdim( sceneExtent )], t;
	for (int i = 0; i < RAY_BATCH_SIZE; i++)
	{
		// de-tile and flip image for tga file.
		int x = i % imgSize, y = imgSize - 1 - (i / imgSize);
		int tx = x / 4, ty = y / 4, tile = tx + ty * (imgSize / 4);
		int posInTile = (x & 3) + (y & 3) * 4;
		int rayIdx = tile * 16 + posInTile;
		// trace ray for pixel (x,y) - or use provided trace results
		if (tracedRays) t = ((Ray*)(tracedRays + rayIdx * 64))->hit.t; else t = bvh->IntersectBatch( raySet + rayIdx * 64, 1 );
		int c = (int)(255 - tinybvh_min( 255.0f, t * distScale ));
		fputc( c, f );
	}
	fclose( f );
}

#ifdef ENABLE_OPENCL

void PrepareRayData( char* raySet, const int N )
{
	if (!gpuRayData) 
	{
		gpuRayData = new tinyocl::Buffer( N * 64 * batchAmplifier /* size of Ray on GPU */ );
		gpuOrigRays = new tinyocl::Buffer( N * 64 * batchAmplifier /* size of Ray on GPU */, 0, tinyocl::Buffer::DEVICEONLY );
	}
	for (int o = 0, j = 0; j < batchAmplifier; j++) for (int i = 0; i < N; i++, o += 64)
		memcpy( (unsigned char*)gpuRayData->GetHostPtr() + o, raySet + i * 64, 64 );
}

int ExperimentCore( tinyocl::Kernel* kernel, const int N, uint64_t& traceTime )
{
	gpuRayData->CopyToDevice();
	gpuRayData->CopyTo( gpuOrigRays );
	cl_event events[ocl_runs] = {};
	for (int pass = -20 /* warm up */; pass < ocl_runs; pass++)
	{
		gpuOrigRays->CopyTo( gpuRayData );
		kernel->Run( N * batchAmplifier, 64, 0, pass >= 0 ? &events[pass] : 0 );
	}
	clFinish( kernel->GetQueue() );
	traceTime = 0x3fffffffffffffffu;
	for (int pass = 0; pass < ocl_runs; pass++)
	{
		clGetEventProfilingInfo( events[pass], CL_PROFILING_COMMAND_START, sizeof( cl_ulong ), &startTime, 0 );
		clGetEventProfilingInfo( events[pass], CL_PROFILING_COMMAND_END, sizeof( cl_ulong ), &endTime, 0 );
		uint64_t elapsed = endTime - startTime;
		if (elapsed < traceTime) traceTime = elapsed;
		clReleaseEvent( events[pass] );
	}
	return ocl_runs;
}

float Experiment::RunGPU_BVH2( char* raySet, const int N, bool anyHit )
{
	// trace 'first hit' rays on GPU
	BVH_GPU* bvh_gpu = (BVH_GPU*)bvh->GetBVH();
	// create OpenCL buffers for the BVH data calculated by tiny_bvh.h
	tinyocl::Buffer gpuNodes( bvh_gpu->usedNodes * sizeof( BVH_GPU::BVHNode ), bvh_gpu->bvhNode, tinyocl::Buffer::READONLY );
	tinyocl::Buffer triData( bvh_gpu->idxCount * sizeof( tinybvh::bvhvec4 ) * 3, (bvhvec4*)bvh_gpu->orderedVerts.data, tinyocl::Buffer::READONLY );
	gpuNodes.CopyToDevice();
	triData.CopyToDevice();
	PrepareRayData( raySet, N );
	// start kernel on gpu
	uint64_t traceTime = 0;
	tinyocl::Kernel* kernel = anyHit ? kernel_any : kernel_nearest;
	kernel->SetArguments( &gpuNodes, &triData, gpuRayData );
	int runs = ExperimentCore( kernel, N, traceTime );
	// get results from GPU for verification
	if (!anyHit) gpuRayData->CopyFromDevice();
	return (float)traceTime * 1e-9f;
}

float Experiment::RunGPU_BVH4( char* raySet, const int N, bool anyHit )
{
	// trace 'first hit' rays on GPU using a 4-wide BVH
	BVH4_GPU* bvh4_gpu = (BVH4_GPU*)bvh->GetBVH();
	tinyocl::Buffer gpuNodes( bvh4_gpu->usedBlocks * sizeof( tinybvh::bvhvec4 ), bvh4_gpu->bvh4Data, tinyocl::Buffer::READONLY );
	gpuNodes.CopyToDevice();
	PrepareRayData( raySet, N );
	// start timer and start kernel on gpu
	uint64_t traceTime = 0;
	tinyocl::Kernel* kernel = anyHit ? gpu4way_kernel_any : gpu4way_kernel;
	kernel->SetArguments( &gpuNodes, gpuRayData );
	int runs = ExperimentCore( kernel, N, traceTime );
	// get results from GPU for verification.
	if (!anyHit) gpuRayData->CopyFromDevice();
	return (float)traceTime * 1e-9f;
}

float Experiment::RunGPU_CWBVH( char* raySet, const int N, bool anyHit )
{
	// trace 'first hit' rays on GPU using CWBVH
	BVH8_CWBVH* cwbvh = (BVH8_CWBVH*)bvh->GetBVH();
	tinyocl::Buffer cwbvhNodes( cwbvh->usedBlocks * sizeof( tinybvh::bvhvec4 ), cwbvh->bvh8Data, tinyocl::Buffer::READONLY );
#ifdef CWBVH_COMPRESSED_TRIS
	tinyocl::Buffer cwbvhTris( cwbvh->idxCount * 4 * sizeof( tinybvh::bvhvec4 ), cwbvh->bvh8Tris, tinyocl::Buffer::READONLY );
#else
	tinyocl::Buffer cwbvhTris( cwbvh->idxCount * 3 * sizeof( tinybvh::bvhvec4 ), cwbvh->bvh8Tris, tinyocl::Buffer::READONLY );
#endif
	cwbvhNodes.CopyToDevice();
	cwbvhTris.CopyToDevice();
	PrepareRayData( raySet, N );
	// start timer and start kernel on gpu
	uint64_t traceTime = 0;
	tinyocl::Kernel* kernel = anyHit ? cwbvh_kernel_any : cwbvh_kernel;
	kernel->SetArguments( &cwbvhNodes, &cwbvhTris, gpuRayData );
	int runs = ExperimentCore( kernel, N, traceTime );
	// get results from GPU for verification.
	if (!anyHit) gpuRayData->CopyFromDevice();
	return (float)traceTime * 1e-9f;
}

float Experiment::RunGPU_AMD4( char* raySet, const int N, bool anyHit )
{
	// load AMD-specific kernel
	if (!amd4way_kernel) amd4way_kernel = new tinyocl::Kernel( "kernels/traverse_amd_hw.cl", "batch_gpu4wayHW" );
	if (!amd4way_kernel_any) amd4way_kernel_any = new tinyocl::Kernel( "kernels/traverse_amd_hw.cl", "batch_gpu4wayHW_any" );
	// trace 'first hit' rays on GPU using a 4-wide BVH
	BVH4_AMD_HW* bvh4_gpu = (BVH4_AMD_HW*)bvh->GetBVH();
	tinyocl::Buffer gpuNodes( bvh4_gpu->usedBlocks * sizeof( tinybvh::bvhvec4 ), bvh4_gpu->bvh4Data, tinyocl::Buffer::READONLY );
	gpuNodes.CopyToDevice();
	PrepareRayData( raySet, N );
	// start timer and start kernel on gpu
	uint64_t traceTime = 0;
	tinyocl::Kernel* kernel = anyHit ? amd4way_kernel_any : amd4way_kernel;
	kernel->SetArguments( &gpuNodes, gpuRayData );
	int runs = ExperimentCore( kernel, N, traceTime );
	// get results from GPU for verification.
	if (!anyHit) gpuRayData->CopyFromDevice();
	return (float)traceTime * 1e-9f;
}

float Experiment::RunGPU_AMD8( char* raySet, const int N, bool anyHit )
{
	// load AMD-specific kernel
	if (!amd8way_kernel) amd8way_kernel = new tinyocl::Kernel( "kernels/traverse_amd_hw.cl", "batch_gpu8wayHW" );
	if (!amd8way_kernel_any) amd8way_kernel_any = new tinyocl::Kernel( "kernels/traverse_amd_hw.cl", "batch_gpu8wayHW_any" );
	// trace 'first hit' rays on GPU using an 8-wide BVH
	CWBVH8_AMD_HW* bvh8_gpu = (CWBVH8_AMD_HW*)bvh->GetBVH();
	tinyocl::Buffer gpuNodes( bvh8_gpu->usedBlocks * sizeof( tinybvh::bvhvec4 ), bvh8_gpu->bvh8Data, tinyocl::Buffer::READONLY );
	gpuNodes.CopyToDevice();
	PrepareRayData( raySet, N );
	// start timer and start kernel on gpu
	uint64_t traceTime = 0;
	tinyocl::Kernel* kernel = anyHit ? amd8way_kernel_any : amd8way_kernel;
	kernel->SetArguments( &gpuNodes, gpuRayData );
	int runs = ExperimentCore( kernel, N, traceTime );
	// get results from GPU for verification.
	if (!anyHit) gpuRayData->CopyFromDevice();
	return (float)traceTime * 1e-9f;
}

#endif

}; // namespace tinybvh