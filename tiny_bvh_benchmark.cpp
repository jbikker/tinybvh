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
#if 0
	// 1. BVH construction
	experiment.push_back( new Experiment( BVH2, AVXBUILD, scene ) );
	experiment.push_back( new Experiment( EMBREE, NO_FLAGS, scene ) );
	experiment.push_back( new Experiment( BVH2, FULLSWEEP, scene ) );
	experiment.push_back( new Experiment( BVH2, FULLSWEEP|PRESPLIT, scene ) );
	experiment.push_back( new Experiment( BVH2, SPATIALSPLITS, scene ) );
	experiment.push_back( new Experiment( MADMANN91, NO_FLAGS, scene ) );
#else

	// 2. BVH traversal

#if 0

	experiment.push_back( new Experiment( GPU_BVH, NO_FLAGS, scene, PRIMARY_VIEW1, USE_GPU ) );
	experiment.push_back( new Experiment( GPU_BVH4, NO_FLAGS, scene, PRIMARY_VIEW1, USE_GPU ) );
	experiment.push_back( new Experiment( CWBVH, NO_FLAGS, scene, PRIMARY_VIEW1, USE_GPU ) );
	experiment.push_back( new Experiment( GPU_BVH, SPATIALSPLITS, scene, PRIMARY_VIEW1, USE_GPU ) );
	experiment.push_back( new Experiment( GPU_BVH4, SPATIALSPLITS|OPTIMIZE, scene, PRIMARY_VIEW1, USE_GPU ) );

	experiment.push_back( new Experiment( GPU_BVH, NO_FLAGS, scene, FIRST_BOUNCE, USE_GPU ) );
	experiment.push_back( new Experiment( GPU_BVH4, NO_FLAGS, scene, FIRST_BOUNCE, USE_GPU ) );
	experiment.push_back( new Experiment( CWBVH, NO_FLAGS, scene, FIRST_BOUNCE, USE_GPU ) );
	experiment.push_back( new Experiment( GPU_BVH, SPATIALSPLITS, scene, FIRST_BOUNCE, USE_GPU ) );
	experiment.push_back( new Experiment( GPU_BVH4, SPATIALSPLITS|OPTIMIZE, scene, FIRST_BOUNCE, USE_GPU ) );

	experiment.push_back( new Experiment( GPU_BVH, NO_FLAGS, scene, AO_RAYS, USE_GPU ) );
	experiment.push_back( new Experiment( GPU_BVH4, NO_FLAGS, scene, AO_RAYS, USE_GPU ) );
	experiment.push_back( new Experiment( CWBVH, NO_FLAGS, scene, AO_RAYS, USE_GPU ) );
	experiment.push_back( new Experiment( GPU_BVH, SPATIALSPLITS, scene, AO_RAYS, USE_GPU ) );
	experiment.push_back( new Experiment( GPU_BVH4, SPATIALSPLITS|OPTIMIZE, scene, AO_RAYS, USE_GPU ) );

#else

	experiment.push_back( new Experiment( BVH2, NO_FLAGS, scene, PRIMARY_VIEW1, DEFAULT, "dbg_bvh2.tga" ) );
	experiment.push_back( new Experiment( BVH2, SPATIALSPLITS, scene, PRIMARY_VIEW1 ) );
	experiment.push_back( new Experiment( BVH2, SPATIALSPLITS, scene, PRIMARY_VIEW1, MULTICORE ) );
	experiment.push_back( new Experiment( BVH4_WIVE, NO_FLAGS, scene, PRIMARY_VIEW1, DEFAULT, "dbg_bvh4wive.tga" ) );
	experiment.push_back( new Experiment( BVH4_WIVE, SPATIALSPLITS, scene, PRIMARY_VIEW1, DEFAULT, "dbg_bvh4wive_spatial.tga" ) );
	experiment.push_back( new Experiment( BVH8_WIVE, SPATIALSPLITS|OPTIMIZE, scene, PRIMARY_VIEW1, DEFAULT, "dbg_bvh8wive_spatial.tga" ) );
	experiment.push_back( new Experiment( BVH8_WIVE, SPATIALSPLITS|OPTIMIZE, scene, PRIMARY_VIEW1, MULTICORE ) );
	experiment.push_back( new Experiment( MADMANN91, NO_FLAGS, scene, PRIMARY_VIEW1, DEFAULT, "dbg_madmann91.tga" ) );
	experiment.push_back( new Experiment( MADMANN91, SPATIALSPLITS, scene, PRIMARY_VIEW1 ) );
	experiment.push_back( new Experiment( MADMANN91, SPATIALSPLITS, scene, PRIMARY_VIEW1, MULTICORE ) );
	experiment.push_back( new Experiment( EMBREE, NO_FLAGS, scene, PRIMARY_VIEW1, DEFAULT, "dbg_embree.tga" ) );
	experiment.push_back( new Experiment( EMBREE, NO_FLAGS, scene, PRIMARY_VIEW1, MULTICORE  ) );

#endif

#endif
	// run experiments
	for( int i = 0; i < experiment.size(); i++ ) experiment[i]->Run();
	// all done.
	return 0;
}