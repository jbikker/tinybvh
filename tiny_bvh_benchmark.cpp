// Benchmark Suite for TinyBVH. EARLY CODE - UNDER CONSTRUCTION.
// Combine:
// 1. Scene (Sponza, Bistro, ..)
// 2. BVH type (basic, SBVH, BVH8, ..)
// 3. BVH options (full sweep, presplit, optimize, ..)
// 4. Ray distribution (primary, shadow, AO, ..)
// 5. Processor (CPU, GPU)
// And analyze BVH quality according to various measures:
// * SAH
// * EPO
// * rays/s
// * single-threaded build time
// * multi-threaded build time
// Data is printed to the screen and/or exported to file.

#define TINYBVH_IMPLEMENTATION
#include "tiny_bvh.h"
#include <cstdlib>
#include <cstdio>

using namespace tinybvh;
using namespace std;

#include "experiment.h"

vector<Experiment*> experiment;

int main()
{
	uint32_t scene = 0;
	// construct list of experiments
	// 1. BVH construction
#if 0
	experiment.push_back( new Experiment( BVHBase::BVHType::LAYOUT_BVH, BuildFlags::NONE, scene ) );
	experiment.push_back( new Experiment( BVHBase::BVHType::LAYOUT_BVH, BuildFlags::AVXBUILD, scene ) );
	experiment.push_back( new Experiment( BVHBase::BVHType::LAYOUT_BVH, BuildFlags::FULLSWEEP, scene ) );
	experiment.push_back( new Experiment( BVHBase::BVHType::LAYOUT_BVH, BuildFlags::FULLSWEEP | BuildFlags::PRESPLIT, scene ) );
	experiment.push_back( new Experiment( BVHBase::BVHType::LAYOUT_BVH, BuildFlags::SPATIALSPLITS, scene ) );
#endif
	// 2. BVH traversal
	experiment.push_back( new Experiment( BVHBase::BVHType::LAYOUT_BVH, NO_FLAGS, CRYTEK_SPONZA, PRIMARY_VIEW1, "view1.tga" ) );
	experiment.push_back( new Experiment( BVHBase::BVHType::LAYOUT_BVH, NO_FLAGS, CRYTEK_SPONZA, PRIMARY_VIEW2, "view2.tga" ) );
	experiment.push_back( new Experiment( BVHBase::BVHType::LAYOUT_BVH, NO_FLAGS, CRYTEK_SPONZA, PRIMARY_VIEW3, "view3.tga" ) );
	experiment.push_back( new Experiment( BVHBase::BVHType::LAYOUT_BVH, NO_FLAGS, CRYTEK_SPONZA, FIRST_BOUNCE, "view1.tga" ) );
	// run experiments
	for (auto ex : experiment) ex->Run();
	// all done.
	return 0;
}