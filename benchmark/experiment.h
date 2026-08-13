#pragma once

#include <chrono>
#include "ray_distribution.h"
#include "primitive_set.h"
#include "acc_struc.h"

namespace tinybvh
{

enum ExperimentFlags : int {
	DEFAULT = 0,
	MULTICORE = 1,
	USE_GPU = 2,
	STABLE_POWER = 4 // windows only; requires 'Developer Mode'; opens a DX12 window.
};

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
	Experiment( BVHLayout layout, BuildFlags buildFlags, Scene prims, RaySet rays = RaySet::UNSPECIFIED, ExperimentFlags = DEFAULT, const char* view = 0 );
	void Run();
private:
	void RunBuildExperiment();
	void RunTraceExperiment();
	void WriteImage( char* raySet, const char* tracedRays = 0 );
	float RunGPU_BVH2( char* raySet, const int N, const char* tgaFile );
	float RunGPU_BVH2_Any( char* raySet, const int N );
	float RunGPU_BVH4( char* raySet, const int N, const char* tgaFile );
	float RunGPU_BVH4_Any( char* raySet, const int N );
	float RunGPU_CWBVH( char* raySet, const int N, const char* tgaFile );
	float RunGPU_CWBVH_Any( char* raySet, const int N );
	RaySet raySet = UNSPECIFIED;
	Scene primSet = (Scene)0;
	AccStruc* bvh = 0;
	ExperimentFlags flags;
	char* title = 0, *tgaFile = 0;
	float buildTime = 0; // in seconds.
	inline static PrimitiveSet* cachedPrimSet[99] = { 0 };
	inline static RayDistribution* cachedRaySet[99] = { 0 };
};

inline ExperimentFlags operator|( ExperimentFlags a, ExperimentFlags b )
{
	return static_cast<ExperimentFlags>(static_cast<int>(a) | static_cast<int>(b));
}

};

// EOF