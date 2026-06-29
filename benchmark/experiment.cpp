#include "headers.h"

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
		snprintf( title, 1024, "BVH build measurement - Layout: %s - Scene: %s (%i tris)",
			bvh->GetDescription(),
			cachedPrimSet[primSet]->GetDescription(),
			cachedPrimSet[primSet]->primCount );
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
		snprintf( title, 1024, "BVH traversal measurement - Layout: %s - Scene: %s (%i tris) - %s",
			bvh->GetDescription(),
			cachedPrimSet[primSet]->GetDescription(),
			cachedPrimSet[primSet]->primCount,
			cachedRaySet[raySet]->GetDescription() );
		// Save a thumbnail of the scene / rayset if requested. Must be .tga.
		if (view)
		{
			FILE* f = fopen( view, "wb" );
			const int imgSize = (int)sqrtf( RAY_BATCH_SIZE );
			const int h[] = { 3 << 16, 8 << 24, 0, imgSize + (imgSize << 16), 8 };
			fwrite( h, 2, 9, f ); // minimalist greyscale tga header
			bvh->Build( cachedPrimSet[primSet] );
			const bvhvec3 sceneExtent = bvh->SceneExtent();
			const float distScale = 384.0f / sceneExtent[tinybvh_maxdim( sceneExtent )];
			for (int i = 0; i < RAY_BATCH_SIZE; i++)
			{
				int x = i % imgSize, y = i / imgSize, p = x + (imgSize - 1 - y) * imgSize; // flip over y
				Ray r( cachedRaySet[raySet]->O[p], cachedRaySet[raySet]->D[p], 1e34f );
				bvh->IntersectBatch( (__m256*)&r, 1 );
				int c = (int)(255 - tinybvh_min( 255.0f, r.hit.t * distScale ) );
				fputc( c, f );
			}
			fclose( f );
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
		__m256* extensionRays = (__m256*)malloc64( N * sizeof( Ray ) );
		__m256* shadowRays = (__m256*)malloc64( N * sizeof( Ray ) );
		bvhvec3* O = cachedRaySet[raySet]->O, *D = cachedRaySet[raySet]->D;
		float* tmin = cachedRaySet[raySet]->tmin, *tmax = cachedRaySet[raySet]->tmax;
		for (int i = 0; i < N; i++)
		{
			Ray r( O[i] + D[i] * tmin[i], D[i], tmax[i] - tmin[i] );
			Ray s( O[i] + D[i] * tmin[i], D[i], tmax[i] - tmin[i] );
			memcpy( extensionRays + 2 * i, &r, 64 );
			memcpy( shadowRays + 2 * i, &r, 64 );
		}
		// trace extension rays
		bvh->IntersectBatch( extensionRays, N ); // warm caches, precompute data
		int runs = 0;
		Timer t;
		while (runs < 5 || t.elapsed() < 1.5f /* at least 5, or whatever fits in a 1.5 seconds. */)
		{
			if (flags & MULTICORE) bvh->IntersectBatchMT( extensionRays, N ); else bvh->IntersectBatch( extensionRays, N );
			runs++;
		}
		float traceTime = t.elapsed() * (1.0f / runs); // average of runs.
		float raysPerSecond = (float)N / traceTime;
		float mraysPerSecond = raysPerSecond / RAY_BATCH_SIZE;
		printf( "%s\nfind nearest: %.2fM, ", title, mraysPerSecond );
		// trace shadow rays
		if (bvh->layout == MADMANN91)
		{
			printf( "any hit: n/a\n" );
		}
		else
		{
			runs = 0;
			bvh->OcclusionBatch( shadowRays, N ); // warm caches
			t.reset();
			while (runs < 5 || t.elapsed() < 1.5f /* at least 5, or whatever fits in a 1.5 seconds. */)
			{
				if (flags & MULTICORE) bvh->OcclusionBatchMT( extensionRays, N ); else bvh->OcclusionBatch( shadowRays, N );
				runs++;
			}
			traceTime = t.elapsed() * (1.0f / runs); // average of runs.
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

}; // namespace tinybvh