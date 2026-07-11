// Benchmark Suite for TinyBVH. Work in Progress.
// Construct experiments using:
// - BVH layout:  
//   { BVH2, BVH4_WIVE, BVH8_WIVE, GPU_BVH, GPU_BVH4, CWBVH, MADDMAN91, EMBREE } a.k.a. { 0 .. 7 };
// - Build flags: NO_FLAGS or 
//   { INDEXED, AVXBUILD, FULLSWEEP, PRESPLIT, SPATIALSPLITS, OPTIMIZE } and combinations thereof;
// - Scene: 
//   { CRYTEK_SPONZA, BISTRO_EXTERIOR, CONFERENCE_ROOM, BUNNY_10K, STANFORD_DRAGON } a.k.a. { 0..4 };
// - Ray distribution: 
//   { PRIMARY_VIEW1/2/3, FIRST_BOUNCE, SECOND_BOUNCE, AO_RAYS  } a.k.a. { 0..5 }.
// Specify a valid tga file name as the last parameter to render scene/view to a file.
// An unspecified ray distribution will analyze the specified BVH builder:
// - Construction time (average over several runs);
// - SAH and EPO cost.
// TODO:
// - Embree BVH build and traversal for comparison
// - Madmann91 BVH build and traversal for comparison
// - GPU traversal for GPU_BVH, GPU_BVH4 and CWBVH
// - Double-precision BVH experiments
// - Export to .csv

#include "headers.h"

int main()
{
	PrintHeader(); // see tools.cpp
	InitOpenCL(); // does nothing if disabled; see tools.cpp
	vector<Experiment*> experiment;
	// construct list of experiments
	Scene scene = CRYTEK_SPONZA;

#if 1 // run one block at a time to reduce throttling effects.

	// 1. BVH construction
	// PART 1 - TinyBVH, from ultra-fast to ultra-quality
	experiment.push_back( new Experiment( BVH2, AVXBUILD, scene ) );
	experiment.push_back( new Experiment( BVH2, PRESPLIT, scene ) );
	experiment.push_back( new Experiment( BVH2, FULLSWEEP, scene ) );
	experiment.push_back( new Experiment( BVH2, SPATIALSPLITS, scene ) );
	experiment.push_back( new Experiment( BVH2, PRESPLIT|SPATIALSPLITS|OPTIMIZE, scene ) );
	// PART 2 - Embree, low-medium-high
	experiment.push_back( new Experiment( EMBREE, LOW, scene ) );
	experiment.push_back( new Experiment( EMBREE, MEDIUM, scene ) );
	experiment.push_back( new Experiment( EMBREE, HIGH, scene ) );
	// PART 3 - Madmann91, low-medium-high
	experiment.push_back( new Experiment( MADMANN91, LOW, scene ) );
	experiment.push_back( new Experiment( MADMANN91, MEDIUM, scene ) );
	experiment.push_back( new Experiment( MADMANN91, HIGH, scene ) );

#endif

#if 0
	// 2. CPU BVH traversal
	// PART 1 - TinyBVH, from default bvh via quick bvh builds to ultra-quality
	experiment.push_back( new Experiment( BVH2, NO_FLAGS, scene, PRIMARY_VIEW1 ) ); // basic BVH
	experiment.push_back( new Experiment( BVH2, NO_FLAGS, scene, PRIMARY_VIEW2 ) );
	experiment.push_back( new Experiment( BVH2, NO_FLAGS, scene, PRIMARY_VIEW3 ) );
	experiment.push_back( new Experiment( BVH2, NO_FLAGS, scene, FIRST_BOUNCE ) );
	experiment.push_back( new Experiment( BVH2, NO_FLAGS, scene, AO_RAYS ) );
	experiment.push_back( new Experiment( BVH2, AVXBUILD, scene, PRIMARY_VIEW1 ) ); // fast build, good quality
	experiment.push_back( new Experiment( BVH2, AVXBUILD, scene, PRIMARY_VIEW2 ) );
	experiment.push_back( new Experiment( BVH2, AVXBUILD, scene, PRIMARY_VIEW3 ) );
	experiment.push_back( new Experiment( BVH2, AVXBUILD, scene, FIRST_BOUNCE ) );
	experiment.push_back( new Experiment( BVH2, AVXBUILD, scene, AO_RAYS ) );
	experiment.push_back( new Experiment( BVH4_WIVE, PRESPLIT, scene, PRIMARY_VIEW1 ) ); // BVH for fast traversal
	experiment.push_back( new Experiment( BVH4_WIVE, PRESPLIT, scene, PRIMARY_VIEW2 ) );
	experiment.push_back( new Experiment( BVH4_WIVE, PRESPLIT, scene, PRIMARY_VIEW3 ) );
	experiment.push_back( new Experiment( BVH4_WIVE, PRESPLIT, scene, FIRST_BOUNCE ) );
	experiment.push_back( new Experiment( BVH4_WIVE, PRESPLIT, scene, AO_RAYS ) );
	experiment.push_back( new Experiment( BVH8_WIVE, PRESPLIT|SPATIALSPLITS|OPTIMIZE, scene, PRIMARY_VIEW1 ) ); // HQ BVH
	experiment.push_back( new Experiment( BVH8_WIVE, PRESPLIT|SPATIALSPLITS|OPTIMIZE, scene, PRIMARY_VIEW2 ) );
	experiment.push_back( new Experiment( BVH8_WIVE, PRESPLIT|SPATIALSPLITS|OPTIMIZE, scene, PRIMARY_VIEW3 ) );
	experiment.push_back( new Experiment( BVH8_WIVE, PRESPLIT|SPATIALSPLITS|OPTIMIZE, scene, FIRST_BOUNCE ) );
	experiment.push_back( new Experiment( BVH8_WIVE, PRESPLIT|SPATIALSPLITS|OPTIMIZE, scene, AO_RAYS ) );
#endif
#if 0
	// PART 2 - Embree: low, medium, high
	experiment.push_back( new Experiment( EMBREE, LOW, scene, PRIMARY_VIEW1 ) );
	experiment.push_back( new Experiment( EMBREE, LOW, scene, PRIMARY_VIEW2 ) );
	experiment.push_back( new Experiment( EMBREE, LOW, scene, PRIMARY_VIEW3 ) );
	experiment.push_back( new Experiment( EMBREE, LOW, scene, FIRST_BOUNCE ) );
	experiment.push_back( new Experiment( EMBREE, LOW, scene, AO_RAYS ) );
	experiment.push_back( new Experiment( EMBREE, MEDIUM, scene, PRIMARY_VIEW1 ) );
	experiment.push_back( new Experiment( EMBREE, MEDIUM, scene, PRIMARY_VIEW2 ) );
	experiment.push_back( new Experiment( EMBREE, MEDIUM, scene, PRIMARY_VIEW3 ) );
	experiment.push_back( new Experiment( EMBREE, MEDIUM, scene, FIRST_BOUNCE ) );
	experiment.push_back( new Experiment( EMBREE, MEDIUM, scene, AO_RAYS ) );
	experiment.push_back( new Experiment( EMBREE, HIGH, scene, PRIMARY_VIEW1 ) );
	experiment.push_back( new Experiment( EMBREE, HIGH, scene, PRIMARY_VIEW2 ) );
	experiment.push_back( new Experiment( EMBREE, HIGH, scene, PRIMARY_VIEW3 ) );
	experiment.push_back( new Experiment( EMBREE, HIGH, scene, FIRST_BOUNCE ) );
	experiment.push_back( new Experiment( EMBREE, HIGH, scene, AO_RAYS ) );
#endif
#if 0
	// PART 3 - Madmann91: low, medium, high
	experiment.push_back( new Experiment( MADMANN91, LOW, scene, PRIMARY_VIEW1 ) );
	experiment.push_back( new Experiment( MADMANN91, LOW, scene, PRIMARY_VIEW2 ) );
	experiment.push_back( new Experiment( MADMANN91, LOW, scene, PRIMARY_VIEW3 ) );
	experiment.push_back( new Experiment( MADMANN91, LOW, scene, FIRST_BOUNCE ) );
	experiment.push_back( new Experiment( MADMANN91, LOW, scene, AO_RAYS ) );
	experiment.push_back( new Experiment( MADMANN91, MEDIUM, scene, PRIMARY_VIEW1 ) );
	experiment.push_back( new Experiment( MADMANN91, MEDIUM, scene, PRIMARY_VIEW2 ) );
	experiment.push_back( new Experiment( MADMANN91, MEDIUM, scene, PRIMARY_VIEW3 ) );
	experiment.push_back( new Experiment( MADMANN91, MEDIUM, scene, FIRST_BOUNCE ) );
	experiment.push_back( new Experiment( MADMANN91, MEDIUM, scene, AO_RAYS ) );
	experiment.push_back( new Experiment( MADMANN91, HIGH, scene, PRIMARY_VIEW1 ) );
	experiment.push_back( new Experiment( MADMANN91, HIGH, scene, PRIMARY_VIEW2 ) );
	experiment.push_back( new Experiment( MADMANN91, HIGH, scene, PRIMARY_VIEW3 ) );
	experiment.push_back( new Experiment( MADMANN91, HIGH, scene, FIRST_BOUNCE ) );
	experiment.push_back( new Experiment( MADMANN91, HIGH, scene, AO_RAYS ) );
#endif

#if 0

	// 3. MULTI-CORE CPU TRAVERSAL
	experiment.push_back( new Experiment( BVH2, NO_FLAGS, scene, PRIMARY_VIEW1, MULTICORE ) );
	experiment.push_back( new Experiment( BVH2, NO_FLAGS, scene, PRIMARY_VIEW2, MULTICORE ) );
	experiment.push_back( new Experiment( BVH2, NO_FLAGS, scene, PRIMARY_VIEW3, MULTICORE ) );
	experiment.push_back( new Experiment( BVH2, NO_FLAGS, scene, FIRST_BOUNCE, MULTICORE ) );
	experiment.push_back( new Experiment( BVH2, NO_FLAGS, scene, AO_RAYS, MULTICORE ) );
	experiment.push_back( new Experiment( BVH4_WIVE, PRESPLIT, scene, PRIMARY_VIEW1, MULTICORE ) );
	experiment.push_back( new Experiment( BVH4_WIVE, PRESPLIT, scene, PRIMARY_VIEW2, MULTICORE ) );
	experiment.push_back( new Experiment( BVH4_WIVE, PRESPLIT, scene, PRIMARY_VIEW3, MULTICORE ) );
	experiment.push_back( new Experiment( BVH4_WIVE, PRESPLIT, scene, FIRST_BOUNCE, MULTICORE ) );
	experiment.push_back( new Experiment( BVH4_WIVE, PRESPLIT, scene, AO_RAYS, MULTICORE ) );
	experiment.push_back( new Experiment( BVH8_WIVE, PRESPLIT|SPATIALSPLITS|OPTIMIZE, scene, PRIMARY_VIEW1, MULTICORE ) );
	experiment.push_back( new Experiment( BVH8_WIVE, PRESPLIT|SPATIALSPLITS|OPTIMIZE, scene, PRIMARY_VIEW2, MULTICORE ) );
	experiment.push_back( new Experiment( BVH8_WIVE, PRESPLIT|SPATIALSPLITS|OPTIMIZE, scene, PRIMARY_VIEW3, MULTICORE ) );
	experiment.push_back( new Experiment( BVH8_WIVE, PRESPLIT|SPATIALSPLITS|OPTIMIZE, scene, FIRST_BOUNCE, MULTICORE ) );
	experiment.push_back( new Experiment( BVH8_WIVE, PRESPLIT|SPATIALSPLITS|OPTIMIZE, scene, AO_RAYS, MULTICORE ) );

#endif

#if 0

	// 4. GPU TRAVERSAL
	experiment.push_back( new Experiment( GPU_BVH, PRESPLIT|SPATIALSPLITS|OPTIMIZE, scene, PRIMARY_VIEW1, USE_GPU ) );
	experiment.push_back( new Experiment( GPU_BVH, PRESPLIT|SPATIALSPLITS|OPTIMIZE, scene, PRIMARY_VIEW2, USE_GPU ) );
	experiment.push_back( new Experiment( GPU_BVH, PRESPLIT|SPATIALSPLITS|OPTIMIZE, scene, PRIMARY_VIEW3, USE_GPU ) );
	experiment.push_back( new Experiment( GPU_BVH, PRESPLIT|SPATIALSPLITS|OPTIMIZE, scene, FIRST_BOUNCE, USE_GPU ) );
	experiment.push_back( new Experiment( GPU_BVH, PRESPLIT|SPATIALSPLITS|OPTIMIZE, scene, AO_RAYS, USE_GPU ) );
	experiment.push_back( new Experiment( GPU_BVH4, PRESPLIT|SPATIALSPLITS|OPTIMIZE, scene, PRIMARY_VIEW1, USE_GPU ) );
	experiment.push_back( new Experiment( GPU_BVH4, PRESPLIT|SPATIALSPLITS|OPTIMIZE, scene, PRIMARY_VIEW2, USE_GPU ) );
	experiment.push_back( new Experiment( GPU_BVH4, PRESPLIT|SPATIALSPLITS|OPTIMIZE, scene, PRIMARY_VIEW3, USE_GPU ) );
	experiment.push_back( new Experiment( GPU_BVH4, PRESPLIT|SPATIALSPLITS|OPTIMIZE, scene, FIRST_BOUNCE, USE_GPU ) );
	experiment.push_back( new Experiment( GPU_BVH4, PRESPLIT|SPATIALSPLITS|OPTIMIZE, scene, AO_RAYS, USE_GPU ) );
	experiment.push_back( new Experiment( CWBVH, PRESPLIT|SPATIALSPLITS|OPTIMIZE, scene, PRIMARY_VIEW1, USE_GPU ) );
	experiment.push_back( new Experiment( CWBVH, PRESPLIT|SPATIALSPLITS|OPTIMIZE, scene, PRIMARY_VIEW2, USE_GPU ) );
	experiment.push_back( new Experiment( CWBVH, PRESPLIT|SPATIALSPLITS|OPTIMIZE, scene, PRIMARY_VIEW3, USE_GPU ) );
	experiment.push_back( new Experiment( CWBVH, PRESPLIT|SPATIALSPLITS|OPTIMIZE, scene, FIRST_BOUNCE, USE_GPU ) );
	experiment.push_back( new Experiment( CWBVH, PRESPLIT|SPATIALSPLITS|OPTIMIZE, scene, AO_RAYS, USE_GPU ) );

#endif

	// run experiments
	for( int i = 0; i < experiment.size(); i++ ) experiment[i]->Run();
	// all done.
	return 0;
}