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

#if 1

	experiment.push_back( new Experiment( GPU_BVH, NO_FLAGS, scene, PRIMARY_VIEW1, USE_GPU ) );
	experiment.push_back( new Experiment( GPU_BVH4, NO_FLAGS, scene, PRIMARY_VIEW1, USE_GPU ) );
	experiment.push_back( new Experiment( GPU_BVH, OPTIMIZE, scene, PRIMARY_VIEW1, USE_GPU ) );
	experiment.push_back( new Experiment( GPU_BVH4, OPTIMIZE, scene, PRIMARY_VIEW1, USE_GPU ) );

#else

	// traversal performance for 'low', 'medium' and 'high', in TinyBVH, Madmann91 and Embree.
	experiment.push_back( new Experiment( BVH2, NO_FLAGS, scene, PRIMARY_VIEW1 ) );
	experiment.push_back( new Experiment( BVH4_WIVE, PRESPLIT, scene, PRIMARY_VIEW1 ) );
	experiment.push_back( new Experiment( BVH8_WIVE, PRESPLIT|SPATIALSPLITS|OPTIMIZE, scene, PRIMARY_VIEW1 ) );
	experiment.push_back( new Experiment( MADMANN91, LOW, scene, PRIMARY_VIEW1 ) );
	experiment.push_back( new Experiment( MADMANN91, MEDIUM, scene, PRIMARY_VIEW1 ) );
	experiment.push_back( new Experiment( MADMANN91, HIGH, scene, PRIMARY_VIEW1 ) );
	experiment.push_back( new Experiment( EMBREE, LOW, scene, PRIMARY_VIEW1 ) );
	experiment.push_back( new Experiment( EMBREE, MEDIUM, scene, PRIMARY_VIEW1 ) );
	experiment.push_back( new Experiment( EMBREE, HIGH, scene, PRIMARY_VIEW1 ) );
	
	// traversal performance for first-bounce diffuse rays.
	experiment.push_back( new Experiment( BVH8_WIVE, PRESPLIT|SPATIALSPLITS|OPTIMIZE, scene, FIRST_BOUNCE, DEFAULT, "noise_tinybvh.tga" ) );
	experiment.push_back( new Experiment( MADMANN91, HIGH, scene, FIRST_BOUNCE, DEFAULT, "noise_madmann.tga" ) );
	experiment.push_back( new Experiment( EMBREE, HIGH, scene, FIRST_BOUNCE, DEFAULT, "noise_embree.tga" ) );
	
	// traversal performance for ambient occlusion rays.
	experiment.push_back( new Experiment( BVH8_WIVE, PRESPLIT|SPATIALSPLITS|OPTIMIZE, scene, AO_RAYS ) );
	experiment.push_back( new Experiment( MADMANN91, HIGH, scene, AO_RAYS ) );
	experiment.push_back( new Experiment( EMBREE, HIGH, scene, AO_RAYS ) );

#endif

#endif

	// run experiments
	for( int i = 0; i < experiment.size(); i++ ) experiment[i]->Run();
	// all done.
	return 0;
}