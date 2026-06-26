// Benchmark Suite for TinyBVH. Work in Progress.
// Construct experiments using:
// - BVH layout:  
//   { BVH2, BVH4, BVH4_WIVE, BVH8_WIVE, GPU_BVH, GPU_BVH4, CWBVH } a.k.a. { 0 .. 7 };
// - Build flags: NO_FLAGS or 
//   { AVXBUILD, FULLSWEEP, PRESPLIT, SPATIALSPLITS, OPTIMIZE } and combinations thereof;
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
	vector<Experiment*> experiment;
	// construct list of experiments
	Scene scene = CRYTEK_SPONZA;
#if 1
	// 1. BVH construction
	experiment.push_back( new Experiment( BVH2, AVXBUILD, scene ) );
	experiment.push_back( new Experiment( BVH2, FULLSWEEP, scene ) );
	experiment.push_back( new Experiment( BVH2, FULLSWEEP|PRESPLIT, scene ) );
	experiment.push_back( new Experiment( BVH2, SPATIALSPLITS, scene ) );
	experiment.push_back( new Experiment( MADMANN91, NO_FLAGS, scene ) );
#else
	// 2. BVH traversal
	experiment.push_back( new Experiment( BVH2, NO_FLAGS, scene, PRIMARY_VIEW1, "view1.tga" ) );
	experiment.push_back( new Experiment( BVH2, NO_FLAGS, scene, PRIMARY_VIEW2, "view2.tga" ) );
	experiment.push_back( new Experiment( BVH2, NO_FLAGS, scene, PRIMARY_VIEW3, "view3.tga" ) );
#endif
	// run experiments
	for( int i = 0; i < experiment.size(); i++ ) experiment[i]->Run();
	// all done.
	return 0;
}