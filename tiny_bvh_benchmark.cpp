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
#include <chrono>

using namespace tinybvh;
using namespace std;

#include "ray_distribution.h"
#include "primitive_set.h"
#include "acc_struc.h"

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

class Experiment
{
public:
	Experiment( BVHBase::BVHType layout, AccStruc::BuildFlags flags, uint32_t primSet, uint32_t raySet )
	{
		// Accstruc traversal experiment.
		if (!cachedPrimSet[primSet]) cachedPrimSet[primSet] = new PrimitiveSet( primSet );
		if (!cachedRaySet[raySet]) cachedRaySet[raySet] = new RayDistribution( raySet );
		bvh = new AccStruc( layout, flags ); // actual construction is postponed until ::Run.
	}
	Experiment( const char* desc, BVHBase::BVHType layout, AccStruc::BuildFlags flags, uint32_t prims )
	{
		// Accstruc build experiment.
		primSet = prims;
		title = new char[strlen( desc ) + 1];
		strncpy( title, desc, strlen( desc ) + 1 );
		if (!cachedPrimSet[primSet]) cachedPrimSet[primSet] = new PrimitiveSet( primSet );
		bvh = new AccStruc( layout, flags ); // actual construction is postponed until ::Run.
	}
	void Run()
	{
		if (raySet > -1)
		{
			// this is a traversal experiment.
			// - Build once, time is irrelevant;
			// - Trace one million rays several times for average trace time.
			Timer t;
			bvh->Build( cachedPrimSet[primSet] );
			buildTime = t.elapsed(); // warning: single build; not accurate.
			// TODO: trace rays.
		}
		else
		{
			// this is an accstruc build experiment.
			// - Build several times for average build time;
			// - Assess SAH and EPO.
			bvh->Build( cachedPrimSet[primSet] ); // warm caches
			Timer t;
			int runs = 0;
			while (runs < 5 && t.elapsed() < 1.5f /* at least 5, or whatever fits in a 1.5 seconds. */)
			{
				bvh->Build( cachedPrimSet[primSet] );
				runs++;
			}
			buildTime = t.elapsed() * (1.0f / runs ); // average of runs.
			// report
			printf( "%s\nbuild time: %.1fms ", title, buildTime * 1000.0f );
			t.reset();
			float sahCost = bvh->SAHCost();
			printf( "SAH: %.3f (%.1fms) ", sahCost, t.elapsed() * 1000.0f );
			t.reset();
			float epoCost = bvh->EPOCost();
			printf( "EPO: %.3f (%.1fms)\n", epoCost, t.elapsed() * 1000.0f );
		}
	}
private:
	int raySet = -1, primSet = -1;
	AccStruc* bvh = 0;
	char* title = 0;
	float buildTime = 0; // in seconds.
	inline static PrimitiveSet* cachedPrimSet[99] = { 0 };
	inline static RayDistribution* cachedRaySet[99] = { 0 };
};

vector<Experiment*> experiment;

int main()
{
	uint32_t scene = 1;
	// construct list of experiments
	experiment.push_back( new Experiment(
		"Default BVH - Reference Builder - Crytek Sponza",
		BVHBase::BVHType::LAYOUT_BVH,
		AccStruc::BuildFlags::NONE,
		scene
	) );
	experiment.push_back( new Experiment(
		"Default BVH - AVX Builder - Crytek Sponza",
		BVHBase::BVHType::LAYOUT_BVH,
		AccStruc::BuildFlags::AVXBUILD,
		scene
	) );
	experiment.push_back( new Experiment(
		"Default BVH - Full-sweep Builder - Crytek Sponza",
		BVHBase::BVHType::LAYOUT_BVH,
		AccStruc::BuildFlags::FULLSWEEP,
		scene
	) );
	experiment.push_back( new Experiment(
		"Default BVH - Full-sweep with presplits - Crytek Sponza",
		BVHBase::BVHType::LAYOUT_BVH,
		AccStruc::BuildFlags::FULLSWEEP | AccStruc::BuildFlags::PRESPLIT,
		scene
	) );
	// run experiments
	for (auto ex : experiment) ex->Run();
	// all done.
	return 0;
}