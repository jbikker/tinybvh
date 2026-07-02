#include "headers.h"

#ifdef ENABLE_OPENCL
extern tinyocl::Kernel* ailalaine_kernel;
extern tinyocl::Kernel* ailalaine_kernel_any;
extern tinyocl::Kernel* gpu4way_kernel;
extern tinyocl::Kernel* gpu4way_kernel_any;
extern tinyocl::Kernel* cwbvh_kernel;
extern tinyocl::Kernel* cwbvh_kernel_any;
cl_event event;
cl_ulong startTime, endTime;
#endif

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

void Experiment::Run()
{
	if (raySet != UNSPECIFIED)
	{
		// Traversal experiment.
		// - Build once, time is irrelevant;
		// - Trace on a single core;
		// - Trace one million rays several times for average trace time.
		bvh->Build( cachedPrimSet[primSet] );
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
		// dump image for the ray set, if requested
		if (tgaFile) WriteImage();
		// trace extension rays
		float traceTime;
		Timer t;
	#ifdef ENABLE_OPENCL
		if (flags & USE_GPU && bvh->layout == GPU_BVH) traceTime = RunGPU_BVH2( extensionRays, N );
		else if (flags & USE_GPU && bvh->layout == GPU_BVH4) traceTime = RunGPU_BVH4( extensionRays, N );
		else if (flags & USE_GPU && bvh->layout == CWBVH) traceTime = RunGPU_CWBVH( extensionRays, N );
		else
		#endif
		{
			// trace 'first hit' rays on CPU
			bvh->IntersectBatch( extensionRays, N ); // warm caches, precompute data
			int runs = 0;
			t.reset();
			while (runs < 5 || t.elapsed() < 1.5f /* at least 5, or whatever fits in a 1.5 seconds. */)
			{
				if (flags & MULTICORE) bvh->IntersectBatchMT( extensionRays, N ); else bvh->IntersectBatch( extensionRays, N );
				runs++;
			}
			traceTime = t.elapsed() * (1.0f / runs); // average of runs.
		}
		float raysPerSecond = (float)N / traceTime;
		float mraysPerSecond = raysPerSecond / RAY_BATCH_SIZE;
		printf( "%s\nfind nearest: %.2fM, ", title, mraysPerSecond );
		// trace 'any hit' rays
		if (bvh->layout == MADMANN91)
		{
			printf( "any hit: n/a\n" );
		}
		else
		{
		#ifdef ENABLE_OPENCL
			if (flags & USE_GPU && bvh->layout == GPU_BVH) traceTime = RunGPU_BVH2_Any( extensionRays, N );
			else if (flags & USE_GPU && bvh->layout == GPU_BVH4) traceTime = RunGPU_BVH4_Any( extensionRays, N );
			else if (flags & USE_GPU && bvh->layout == CWBVH) traceTime = RunGPU_CWBVH_Any( extensionRays, N );
			else
			#endif
			{
				// trace 'any hit' rays on CPU
				int runs = 0;
				bvh->OcclusionBatch( shadowRays, N ); // warm caches
				t.reset();
				while (runs < 5 || t.elapsed() < 1.5f /* at least 5, or whatever fits in a 1.5 seconds. */)
				{
					if (flags & MULTICORE) bvh->OcclusionBatchMT( extensionRays, N ); else bvh->OcclusionBatch( shadowRays, N );
					runs++;
				}
				traceTime = t.elapsed() * (1.0f / runs); // average of runs.
			}
			raysPerSecond = (float)N / traceTime;
			mraysPerSecond = raysPerSecond / RAY_BATCH_SIZE;
			printf( "any hit: %.2fM\n", mraysPerSecond );
		}
		// cleanup
		free64( extensionRays );
		free64( shadowRays );
	}
	else
	{
		// Accstruc build experiment.
		// - Build several times for average build time;
		// - Assess SAH and EPO.
		BVH* accstruc = (BVH*)bvh->Build( cachedPrimSet[primSet] ); // warm caches
		Timer t;
		int runs = 0;
		while (runs < 5 || t.elapsed() < 1.5f /* at least 5, or whatever fits in a 1.5 seconds. */)
		{
			bvh->Build( cachedPrimSet[primSet] );
			runs++;
		}
		buildTime = t.elapsed() * (1.0f / runs); // average of runs.
		// report
		printf( "%s\nbuild time: %.2fms ", title, buildTime * 1000.0f );
		float sahCost = bvh->SAHCost();
		float epoCost = bvh->EPOCost();
		printf( "SAH: %.3f, EPO: %.2f\n", sahCost, epoCost );
	}
}

void Experiment::WriteImage( char* raySet )
{
	const int imgSize = (int)sqrtf( RAY_BATCH_SIZE );
	const int h[] = { 3 << 16, 8 << 24, 0, imgSize + (imgSize << 16), 8 };
	FILE* f = fopen( tgaFile, "wb" );
	fwrite( h, 2, 9, f ); // minimalist greyscale tga header
	const bvhvec3 sceneExtent = bvh->SceneExtent();
	const float distScale = 384.0f / sceneExtent[tinybvh_maxdim( sceneExtent )];
	for (int i = 0; i < RAY_BATCH_SIZE; i++)
	{
		// de-tile and flip image for tga file.
		int x = i % imgSize, y = imgSize - 1 - (i / imgSize);
		int tx = x / 4, ty = y / 4, tile = tx + ty * (imgSize / 4);
		int posInTile = (x & 3) + (y & 3) * 4;
		int rayIdx = tile * 16 + posInTile;
		// trace ray for pixel (x,y)
		float t = bvh->IntersectBatch( raySet + rayIdx * 64, 1 );
		int c = (int)(255 - tinybvh_min( 255.0f, t * distScale ));
		fputc( c, f );
	}
	fclose( f );
}

#ifdef ENABLE_OPENCL

float Experiment::RunGPU_BVH2( char* raySet, const int N )
{
	// trace 'first hit' rays on GPU
	BVH_GPU* bvh_gpu = (BVH_GPU*)bvh->GetBVH();
	// create OpenCL buffers for the BVH data calculated by tiny_bvh.h
	tinyocl::Buffer gpuNodes( bvh_gpu->usedNodes * sizeof( BVH_GPU::BVHNode ), bvh_gpu->bvhNode );
	tinyocl::Buffer idxData( bvh_gpu->idxCount * sizeof( unsigned ), bvh_gpu->bvh.primIdx );
	tinyocl::Buffer triData( bvh_gpu->triCount * 3 * sizeof( tinybvh::bvhvec4 ), cachedPrimSet[primSet]->verts );
	gpuNodes.CopyToDevice();
	idxData.CopyToDevice();
	triData.CopyToDevice();
	// create rays and send them to the gpu side
	tinyocl::Buffer rayData( N * 64 /* size of Ray on GPU */ );
	for (int i = 0; i < N; i++)
		memcpy( (unsigned char*)rayData.GetHostPtr() + 64 * i, raySet + i * 64, 64 );
	rayData.CopyToDevice();
	// start timer and start kernel on gpu
	float traceTime = 0;
	ailalaine_kernel->SetArguments( &gpuNodes, &idxData, &triData, &rayData );
	for (int pass = 0; pass < 25; pass++)
	{
		ailalaine_kernel->Run( N, 64, 0, &event );
		clWaitForEvents( 1, &event ); // OpenCL kernsl run asynchronously
		clGetEventProfilingInfo( event, CL_PROFILING_COMMAND_START, sizeof( cl_ulong ), &startTime, 0 );
		clGetEventProfilingInfo( event, CL_PROFILING_COMMAND_END, sizeof( cl_ulong ), &endTime, 0 );
		if (pass == 0) continue; // first pass is for cache warming
		traceTime += (endTime - startTime) * 1e-9f; // event timing is in nanoseconds
	}
	// get results from GPU for verification.
	// rayData.CopyFromDevice();
	return traceTime / 24.0f;
}

float Experiment::RunGPU_BVH2_Any( char* raySet, const int N )
{
	// trace 'any hit' rays on GPU
	BVH_GPU* bvh_gpu = (BVH_GPU*)bvh->GetBVH();
	// create OpenCL buffers for the BVH data calculated by tiny_bvh.h
	tinyocl::Buffer gpuNodes( bvh_gpu->usedNodes * sizeof( BVH_GPU::BVHNode ), bvh_gpu->bvhNode );
	tinyocl::Buffer idxData( bvh_gpu->idxCount * sizeof( unsigned ), bvh_gpu->bvh.primIdx );
	tinyocl::Buffer triData( bvh_gpu->triCount * 3 * sizeof( tinybvh::bvhvec4 ), cachedPrimSet[primSet]->verts );
	gpuNodes.CopyToDevice();
	idxData.CopyToDevice();
	triData.CopyToDevice();
	// create rays and send them to the gpu side
	tinyocl::Buffer rayData( N * 64 /* size of Ray on GPU */ );
	for (int i = 0; i < N; i++)
		memcpy( (unsigned char*)rayData.GetHostPtr() + 64 * i, raySet + i * 64, 64 );
	rayData.CopyToDevice();
	// start timer and start kernel on gpu
	float traceTime = 0;
	ailalaine_kernel_any->SetArguments( &gpuNodes, &idxData, &triData, &rayData );
	for (int pass = 0; pass < 25; pass++)
	{
		ailalaine_kernel_any->Run( N, 64, 0, &event );
		clWaitForEvents( 1, &event ); // OpenCL kernsl run asynchronously
		clGetEventProfilingInfo( event, CL_PROFILING_COMMAND_START, sizeof( cl_ulong ), &startTime, 0 );
		clGetEventProfilingInfo( event, CL_PROFILING_COMMAND_END, sizeof( cl_ulong ), &endTime, 0 );
		if (pass == 0) continue; // first pass is for cache warming
		traceTime += (endTime - startTime) * 1e-9f; // event timing is in nanoseconds
	}
	// get results from GPU for verification.
	// rayData.CopyFromDevice();
	return traceTime / 24.0f;
}

float Experiment::RunGPU_BVH4( char* raySet, const int N )
{
	// trace 'first hit' rays on GPU using a 4-wide BVH
	BVH4_GPU* bvh4_gpu = (BVH4_GPU*)bvh->GetBVH();
	tinyocl::Buffer gpuNodes( bvh4_gpu->usedBlocks * sizeof( tinybvh::bvhvec4 ), bvh4_gpu->bvh4Data );
	gpuNodes.CopyToDevice();
	// create rays and send them to the gpu side
	tinyocl::Buffer rayData( N * 64 /* size of Ray on GPU */ );
	for (int i = 0; i < N; i++)
		memcpy( (unsigned char*)rayData.GetHostPtr() + 64 * i, raySet + i * 64, 64 );
	rayData.CopyToDevice();
	// start timer and start kernel on gpu
	float traceTime = 0;
	gpu4way_kernel->SetArguments( &gpuNodes, &rayData );
	for (int pass = 0; pass < 25; pass++)
	{
		gpu4way_kernel->Run( N, 64, 0, &event );
		clWaitForEvents( 1, &event ); // OpenCL kernsl run asynchronously
		clGetEventProfilingInfo( event, CL_PROFILING_COMMAND_START, sizeof( cl_ulong ), &startTime, 0 );
		clGetEventProfilingInfo( event, CL_PROFILING_COMMAND_END, sizeof( cl_ulong ), &endTime, 0 );
		if (pass == 0) continue; // first pass is for cache warming
		traceTime += (endTime - startTime) * 1e-9f; // event timing is in nanoseconds
	}
	// get results from GPU for verification.
	// rayData.CopyFromDevice();
	return traceTime / 24.0f;
}

float Experiment::RunGPU_BVH4_Any( char* raySet, const int N )
{
	// trace 'any hit' rays on GPU, using a 4-way BVH
	BVH4_GPU* bvh4_gpu = (BVH4_GPU*)bvh->GetBVH();
	tinyocl::Buffer gpuNodes( bvh4_gpu->usedBlocks * sizeof( tinybvh::bvhvec4 ), bvh4_gpu->bvh4Data );
	gpuNodes.CopyToDevice();
	// create rays and send them to the gpu side
	tinyocl::Buffer rayData( N * 64 /* size of Ray on GPU */ );
	for (int i = 0; i < N; i++)
		memcpy( (unsigned char*)rayData.GetHostPtr() + 64 * i, raySet + i * 64, 64 );
	rayData.CopyToDevice();
	// start timer and start kernel on gpu
	float traceTime = 0;
	gpu4way_kernel_any->SetArguments( &gpuNodes, &rayData );
	for (int pass = 0; pass < 25; pass++)
	{
		gpu4way_kernel_any->Run( N, 64, 0, &event );
		clWaitForEvents( 1, &event ); // OpenCL kernsl run asynchronously
		clGetEventProfilingInfo( event, CL_PROFILING_COMMAND_START, sizeof( cl_ulong ), &startTime, 0 );
		clGetEventProfilingInfo( event, CL_PROFILING_COMMAND_END, sizeof( cl_ulong ), &endTime, 0 );
		if (pass == 0) continue; // first pass is for cache warming
		traceTime += (endTime - startTime) * 1e-9f; // event timing is in nanoseconds
	}
	// get results from GPU for verification.
	// rayData.CopyFromDevice();
	return traceTime / 24.0f;
}

float Experiment::RunGPU_CWBVH( char* raySet, const int N )
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
	tinyocl::Buffer rayData( N * 64 /* size of Ray on GPU */ );
	for (int i = 0; i < N; i++)
		memcpy( (unsigned char*)rayData.GetHostPtr() + 64 * i, raySet + i * 64, 64 );
	rayData.CopyToDevice();
	// start timer and start kernel on gpu
	float traceTime = 0;
	cwbvh_kernel->SetArguments( &cwbvhNodes, &cwbvhTris, &rayData );
	for (int pass = 0; pass < 25; pass++)
	{
		cwbvh_kernel->Run( N, 64, 0, &event ); // for now, todo.
		clWaitForEvents( 1, &event ); // OpenCL kernsl run asynchronously
		clGetEventProfilingInfo( event, CL_PROFILING_COMMAND_START, sizeof( cl_ulong ), &startTime, 0 );
		clGetEventProfilingInfo( event, CL_PROFILING_COMMAND_END, sizeof( cl_ulong ), &endTime, 0 );
		if (pass == 0) continue; // first pass is for cache warming
		traceTime += (endTime - startTime) * 1e-9f; // event timing is in nanoseconds
	}
	// get results from GPU for verification.
	// rayData.CopyFromDevice();
	return traceTime / 24.0f;
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
	tinyocl::Buffer rayData( N * 64 /* size of Ray on GPU */ );
	for (int i = 0; i < N; i++)
		memcpy( (unsigned char*)rayData.GetHostPtr() + 64 * i, raySet + i * 64, 64 );
	rayData.CopyToDevice();
	// start timer and start kernel on gpu
	float traceTime = 0;
	cwbvh_kernel_any->SetArguments( &cwbvhNodes, &cwbvhTris, &rayData );
	for (int pass = 0; pass < 25; pass++)
	{
		cwbvh_kernel_any->Run( N, 64, 0, &event ); // for now, todo.
		clWaitForEvents( 1, &event ); // OpenCL kernsl run asynchronously
		clGetEventProfilingInfo( event, CL_PROFILING_COMMAND_START, sizeof( cl_ulong ), &startTime, 0 );
		clGetEventProfilingInfo( event, CL_PROFILING_COMMAND_END, sizeof( cl_ulong ), &endTime, 0 );
		if (pass == 0) continue; // first pass is for cache warming
		traceTime += (endTime - startTime) * 1e-9f; // event timing is in nanoseconds
	}
	// get results from GPU for verification.
	// rayData.CopyFromDevice();
	return traceTime / 24.0f;
}

#endif

}; // namespace tinybvh