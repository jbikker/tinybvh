#include "headers.h"

#define WIN32_LEAN_AND_MEAN
#include "windows.h"

#ifdef ENABLE_OPENCL
extern tinyocl::Kernel* ailalaine_kernel;
extern tinyocl::Kernel* ailalaine_kernel_any;
extern tinyocl::Kernel* gpu4way_kernel;
extern tinyocl::Kernel* gpu4way_kernel_any;
extern tinyocl::Kernel* cwbvh_kernel;
extern tinyocl::Kernel* cwbvh_kernel_any;
cl_event event;
cl_ulong startTime, endTime;
tinyocl::Buffer* gpuRayData = 0;
#endif

extern FILE* csv;

namespace tinybvh
{

Experiment::Experiment( BVHLayout layout, BuildFlags buildFlags, Scene prims, RaySet rays, ExperimentFlags expFlags, const char* view )
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
		snprintf( title, 1024, "BVH TRACE%s - %s - %s (%ik tris) - %s",
			(flags & MULTICORE) ? " (MT)" : ((flags & USE_GPU) ? " (GPU)" : ""),
			bvh->GetDescription(),
			cachedPrimSet[primSet]->GetDescription(),
			cachedPrimSet[primSet]->primCount / 1000,
			cachedRaySet[raySet]->GetDescription() );
		// Save a thumbnail of the scene / rayset if requested. 
		// Must be .tga, only supported for single-threaded builds.
		if (view)
		{
			tgaFile = new char[512];
			strncpy( tgaFile, view, 512 );
		}
	}
}

static bool lastWasTrav = false, firstExperiment = true;

void Experiment::Run()
{
#if defined _WIN32 && defined _MSC_VER
	// force processing to one core to improve cache coherence - TODO: gcc.
	SetThreadAffinityMask( GetCurrentProcess(), 1 );
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
	bvh->PrepareBuild();
	bvh->Build( cachedPrimSet[primSet] );
	bvh->PostBuild();
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
		if (flags & USE_GPU) fprintf( csv, "gpu," ); else if (flags & MULTICORE) fprintf( csv, "cpu (MT)," ); else fprintf( csv, "cpu," );
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
		memcpy( shadowRays + 64 * i, &r, 64 );
	}
	// trace extension rays
	float traceTime, raysPerSecond;
	Timer t;
	int runs = 0;
#ifdef ENABLE_OPENCL
	if (flags & USE_GPU && bvh->layout == GPU_BVH)
	{
		traceTime = RunGPU_BVH2( extensionRays, N, tgaFile );
		raysPerSecond = (float)(N * 8) / traceTime;
		runs = 40;
		WriteImage( 0, (char*)gpuRayData->GetHostPtr() );
	}
	else if (flags & USE_GPU && bvh->layout == GPU_BVH4)
	{
		traceTime = RunGPU_BVH4( extensionRays, N, tgaFile );
		raysPerSecond = (float)(N * 8) / traceTime;
		runs = 40;
		WriteImage( 0, (char*)gpuRayData->GetHostPtr() );
	}
	else if (flags & USE_GPU && bvh->layout == CWBVH)
	{
		traceTime = RunGPU_CWBVH( extensionRays, N, tgaFile );
		raysPerSecond = (float)(N * 8) / traceTime;
		runs = 40;
		WriteImage( 0, (char*)gpuRayData->GetHostPtr() );
	}
	else
	#endif
	{
		// dump image for the ray set, if requested
		if (tgaFile) WriteImage( extensionRays );
		// trace 'first hit' rays on CPU
		bvh->IntersectBatch( extensionRays, N ); // warm caches, precompute data
		t.reset();
		while (runs < 5 || t.elapsed() < 1.5f /* at least 5, or whatever fits in a 1.5 seconds. */)
		{
			if (flags & MULTICORE) bvh->IntersectBatchMT( extensionRays, N ); else bvh->IntersectBatch( extensionRays, N );
			runs++;
		}
		traceTime = t.elapsed() * (1.0f / runs); // average of runs.
		raysPerSecond = (float)N / traceTime;
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
			traceTime = RunGPU_BVH2_Any( extensionRays, N );
			raysPerSecond = (float)(N * 8) / traceTime;
			runs = 40;
		}
		else if (flags & USE_GPU && bvh->layout == GPU_BVH4)
		{
			traceTime = RunGPU_BVH4_Any( extensionRays, N );
			raysPerSecond = (float)(N * 8) / traceTime;
			runs = 40;
		}
		else if (flags & USE_GPU && bvh->layout == CWBVH)
		{
			traceTime = RunGPU_CWBVH_Any( extensionRays, N );
			raysPerSecond = (float)(N * 8) / traceTime;
			runs = 40;
		}
		else
		#endif
		{
			// trace 'any hit' rays on CPU
			bvh->OcclusionBatch( shadowRays, N ); // warm caches
			t.reset();
			while (runs < 5 || t.elapsed() < 1.5f /* at least 5, or whatever fits in a 1.5 seconds. */)
			{
				if (flags & MULTICORE) bvh->OcclusionBatchMT( extensionRays, N ); else bvh->OcclusionBatch( shadowRays, N );
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
	bvh->PrepareBuild();
	bvh->Build( cachedPrimSet[primSet] ); // warm caches
	bvh->PostBuild();
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
	int runs = 0;
	while (runs < 5 || t.elapsed() < 1.5f /* at least 5, or whatever fits in a 1.5 seconds. */)
	{
		bvh->Build( cachedPrimSet[primSet] );
		runs++;
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

float Experiment::RunGPU_BVH2( char* raySet, const int N, const char* tgaFile )
{
	// trace 'first hit' rays on GPU
	BVH_GPU* bvh_gpu = (BVH_GPU*)bvh->GetBVH();
	// create OpenCL buffers for the BVH data calculated by tiny_bvh.h
	tinyocl::Buffer gpuNodes( bvh_gpu->usedNodes * sizeof( BVH_GPU::BVHNode ), bvh_gpu->bvhNode );
	tinyocl::Buffer idxData( bvh_gpu->idxCount * sizeof( unsigned ), bvh_gpu->bvh.primIdx );
	tinyocl::Buffer triData( cachedPrimSet[primSet]->primCount * sizeof( tinybvh::bvhvec4 ) * 3, cachedPrimSet[primSet]->verts );
	gpuNodes.CopyToDevice();
	idxData.CopyToDevice();
	triData.CopyToDevice();
	// create rays and send them to the gpu side
	if (!gpuRayData) gpuRayData = new tinyocl::Buffer( N * 64 * 8 /* size of Ray on GPU */ );
	for (int o = 0, j = 0; j < 8; j++) for (int i = 0; i < N; i++, o += 64)
		memcpy( (unsigned char*)gpuRayData->GetHostPtr() + o, raySet + i * 64, 64 );
	// start timer and start kernel on gpu
	uint64_t traceTime = 0;
	ailalaine_kernel->SetArguments( &gpuNodes, &idxData, &triData, gpuRayData );
	int runs = 0;
	for (int pass = 0; pass < 50; pass++)
	{
		gpuRayData->CopyToDevice();
		ailalaine_kernel->Run( N * 8, 64, 0, &event );
		clWaitForEvents( 1, &event ); // OpenCL kernels run asynchronously
		clGetEventProfilingInfo( event, CL_PROFILING_COMMAND_START, sizeof( cl_ulong ), &startTime, 0 );
		clGetEventProfilingInfo( event, CL_PROFILING_COMMAND_END, sizeof( cl_ulong ), &endTime, 0 );
		if (pass < 10) continue; else runs++; // encourage the GPU to run at full speed
		traceTime += endTime - startTime;
	}
	// get results from GPU for verification.
	gpuRayData->CopyFromDevice();
	return (traceTime / runs) * 1e-9f;
}

float Experiment::RunGPU_BVH2_Any( char* raySet, const int N )
{
	// trace 'any hit' rays on GPU
	BVH_GPU* bvh_gpu = (BVH_GPU*)bvh->GetBVH();
	// create OpenCL buffers for the BVH data calculated by tiny_bvh.h
	tinyocl::Buffer gpuNodes( bvh_gpu->usedNodes * sizeof( BVH_GPU::BVHNode ), bvh_gpu->bvhNode );
	tinyocl::Buffer idxData( bvh_gpu->idxCount * sizeof( unsigned ), bvh_gpu->bvh.primIdx );
	tinyocl::Buffer triData( cachedPrimSet[primSet]->primCount * 3 * sizeof( tinybvh::bvhvec4 ), cachedPrimSet[primSet]->verts );
	gpuNodes.CopyToDevice();
	idxData.CopyToDevice();
	triData.CopyToDevice();
	// create rays and send them to the gpu side
	if (!gpuRayData) gpuRayData = new tinyocl::Buffer( N * 64 * 8 /* size of Ray on GPU */ );
	for (int o = 0, j = 0; j < 8; j++) for (int i = 0; i < N; i++, o += 64)
		memcpy( (unsigned char*)gpuRayData->GetHostPtr() + o, raySet + i * 64, 64 );
	// start timer and start kernel on gpu
	uint64_t traceTime = 0;
	ailalaine_kernel_any->SetArguments( &gpuNodes, &idxData, &triData, gpuRayData );
	int runs = 0;
	for (int pass = 0; pass < 50; pass++)
	{
		gpuRayData->CopyToDevice();
		ailalaine_kernel_any->Run( N * 8, 64, 0, &event );
		clWaitForEvents( 1, &event ); // OpenCL kernels run asynchronously
		clGetEventProfilingInfo( event, CL_PROFILING_COMMAND_START, sizeof( cl_ulong ), &startTime, 0 );
		clGetEventProfilingInfo( event, CL_PROFILING_COMMAND_END, sizeof( cl_ulong ), &endTime, 0 );
		if (pass < 10) continue; else runs++; // encourage the GPU to run at full speed
		traceTime += endTime - startTime;
	}
	// get results from GPU for verification.
	// gpuRayData->CopyFromDevice();
	return (traceTime / runs) * 1e-9f;
}

float Experiment::RunGPU_BVH4( char* raySet, const int N, const char* tgaFile )
{
	// trace 'first hit' rays on GPU using a 4-wide BVH
	BVH4_GPU* bvh4_gpu = (BVH4_GPU*)bvh->GetBVH();
	tinyocl::Buffer gpuNodes( bvh4_gpu->usedBlocks * sizeof( tinybvh::bvhvec4 ), bvh4_gpu->bvh4Data );
	gpuNodes.CopyToDevice();
	// create rays and send them to the gpu side
	if (!gpuRayData) gpuRayData = new tinyocl::Buffer( N * 64 * 8 /* size of Ray on GPU */ );
	for (int o = 0, j = 0; j < 8; j++) for (int i = 0; i < N; i++, o += 64)
		memcpy( (unsigned char*)gpuRayData->GetHostPtr() + o, raySet + i * 64, 64 );
	// start timer and start kernel on gpu
	uint64_t traceTime = 0;
	gpu4way_kernel->SetArguments( &gpuNodes, gpuRayData );
	int runs = 0;
	for (int pass = 0; pass < 50; pass++)
	{
		gpuRayData->CopyToDevice();
		gpu4way_kernel->Run( N * 8, 64, 0, &event );
		clWaitForEvents( 1, &event ); // OpenCL kernels run asynchronously
		clGetEventProfilingInfo( event, CL_PROFILING_COMMAND_START, sizeof( cl_ulong ), &startTime, 0 );
		clGetEventProfilingInfo( event, CL_PROFILING_COMMAND_END, sizeof( cl_ulong ), &endTime, 0 );
		if (pass < 10) continue; else runs++; // encourage the GPU to run at full speed
		traceTime += endTime - startTime;
	}
	// get results from GPU for verification.
	gpuRayData->CopyFromDevice();
	return (traceTime / runs) * 1e-9f;
}

float Experiment::RunGPU_BVH4_Any( char* raySet, const int N )
{
	// trace 'any hit' rays on GPU, using a 4-way BVH
	BVH4_GPU* bvh4_gpu = (BVH4_GPU*)bvh->GetBVH();
	tinyocl::Buffer gpuNodes( bvh4_gpu->usedBlocks * sizeof( tinybvh::bvhvec4 ), bvh4_gpu->bvh4Data );
	gpuNodes.CopyToDevice();
	// create rays and send them to the gpu side
	if (!gpuRayData) gpuRayData = new tinyocl::Buffer( N * 64 * 8 /* size of Ray on GPU */ );
	for (int o = 0, j = 0; j < 8; j++) for (int i = 0; i < N; i++, o += 64)
		memcpy( (unsigned char*)gpuRayData->GetHostPtr() + o, raySet + i * 64, 64 );
	// start timer and start kernel on gpu
	uint64_t traceTime = 0;
	gpu4way_kernel_any->SetArguments( &gpuNodes, gpuRayData );
	int runs = 0;
	for (int pass = 0; pass < 50; pass++)
	{
		gpuRayData->CopyToDevice();
		gpu4way_kernel_any->Run( N * 8, 64, 0, &event );
		clWaitForEvents( 1, &event ); // OpenCL kernels run asynchronously
		clGetEventProfilingInfo( event, CL_PROFILING_COMMAND_START, sizeof( cl_ulong ), &startTime, 0 );
		clGetEventProfilingInfo( event, CL_PROFILING_COMMAND_END, sizeof( cl_ulong ), &endTime, 0 );
		if (pass < 10) continue; else runs++; // encourage the GPU to run at full speed
		traceTime += endTime - startTime;
	}
	// get results from GPU for verification.
	// gpuRayData->CopyFromDevice();
	return (traceTime / runs) * 1e-9f;
}

float Experiment::RunGPU_CWBVH( char* raySet, const int N, const char* tgaFile )
{
	// trace 'first hit' rays on GPU using CWBVH
	BVH8_CWBVH* cwbvh = (BVH8_CWBVH*)bvh->GetBVH();
	tinyocl::Buffer cwbvhNodes( cwbvh->usedBlocks * sizeof( tinybvh::bvhvec4 ), cwbvh->bvh8Data );
#ifdef CWBVH_COMPRESSED_TRIS
	tinyocl::Buffer cwbvhTris( cwbvh->idxCount * 4 * sizeof( tinybvh::bvhvec4 ), cwbvh->bvh8Tris );
#else
	tinyocl::Buffer cwbvhTris( cwbvh->idxCount * 3 * sizeof( tinybvh::bvhvec4 ), cwbvh->bvh8Tris );
#endif
	cwbvhNodes.CopyToDevice();
	cwbvhTris.CopyToDevice();
	// create rays (duplicated 8x) and send them to the gpu side
	if (!gpuRayData) gpuRayData = new tinyocl::Buffer( N * 64 * 8 /* size of Ray on GPU */ );
	for (int o = 0, j = 0; j < 8; j++) for (int i = 0; i < N; i++, o += 64)
		memcpy( (unsigned char*)gpuRayData->GetHostPtr() + o, raySet + i * 64, 64 );
	// start timer and start kernel on gpu
	uint64_t traceTime = 0;
	cwbvh_kernel->SetArguments( &cwbvhNodes, &cwbvhTris, gpuRayData );
	int runs = 0;
	for (int pass = 0; pass < 50; pass++)
	{
		gpuRayData->CopyToDevice();
		cwbvh_kernel->Run( N * 8, 64, 0, &event ); // for now, todo.
		clWaitForEvents( 1, &event ); // OpenCL kernels run asynchronously
		clGetEventProfilingInfo( event, CL_PROFILING_COMMAND_START, sizeof( cl_ulong ), &startTime, 0 );
		clGetEventProfilingInfo( event, CL_PROFILING_COMMAND_END, sizeof( cl_ulong ), &endTime, 0 );
		if (pass < 10) continue; else runs++; // encourage the GPU to run at full speed
		traceTime += endTime - startTime;
	}
	// get results from GPU for verification.
	gpuRayData->CopyFromDevice();
	return (traceTime / runs) * 1e-9f;
}

float Experiment::RunGPU_CWBVH_Any( char* raySet, const int N )
{
	// trace 'first hit' rays on GPU using CWBVH
	BVH8_CWBVH* cwbvh = (BVH8_CWBVH*)bvh->GetBVH();
	tinyocl::Buffer cwbvhNodes( cwbvh->usedBlocks * sizeof( tinybvh::bvhvec4 ), cwbvh->bvh8Data );
#ifdef CWBVH_COMPRESSED_TRIS
	tinyocl::Buffer cwbvhTris( cwbvh->idxCount * 4 * sizeof( tinybvh::bvhvec4 ), cwbvh->bvh8Tris );
#else
	tinyocl::Buffer cwbvhTris( cwbvh->idxCount * 3 * sizeof( tinybvh::bvhvec4 ), cwbvh->bvh8Tris );
#endif
	cwbvhNodes.CopyToDevice();
	cwbvhTris.CopyToDevice();
	// create rays and send them to the gpu side
	if (!gpuRayData) gpuRayData = new tinyocl::Buffer( N * 64 * 8 /* size of Ray on GPU */ );
	for (int o = 0, j = 0; j < 8; j++) for (int i = 0; i < N; i++, o += 64)
		memcpy( (unsigned char*)gpuRayData->GetHostPtr() + o, raySet + i * 64, 64 );
	// start timer and start kernel on gpu
	uint64_t traceTime = 0;
	cwbvh_kernel_any->SetArguments( &cwbvhNodes, &cwbvhTris, gpuRayData );
	int runs = 0;
	for (int pass = 0; pass < 50; pass++)
	{
		gpuRayData->CopyToDevice();
		cwbvh_kernel_any->Run( N * 8, 64, 0, &event ); // for now, todo.
		clWaitForEvents( 1, &event ); // OpenCL kernels run asynchronously
		clGetEventProfilingInfo( event, CL_PROFILING_COMMAND_START, sizeof( cl_ulong ), &startTime, 0 );
		clGetEventProfilingInfo( event, CL_PROFILING_COMMAND_END, sizeof( cl_ulong ), &endTime, 0 );
		if (pass < 10) continue; else runs++; // encourage the GPU to run at full speed
		traceTime += endTime - startTime;
	}
	// get results from GPU for verification.
	// gpuRayData->CopyFromDevice();
	return (traceTime / runs) * 1e-9f;
}

#endif

}; // namespace tinybvh