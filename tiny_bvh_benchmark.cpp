// Benchmark Suite for TinyBVH. Work in Progress.
// Construct experiments using:
// - BVH layout:  
//   { BVH2, BVH4, BVH4_WIVE, BVH8_WIVE, GPU_BVH, GPU_BVH4, CWBVH, AMD_HWRT, MADMANN91, EMBREE, AMD_HWRT8 } a.k.a. { 0 .. 10 };
// - Build flags: NO_FLAGS or 
//   { INDEXED, SIMDBUILD, FULLSWEEP, PRESPLIT, SPATIALSPLITS, OPTIMIZE } and combinations thereof;
// - Scene: 
//   { CRYTEK_SPONZA, BISTRO_EXTERIOR, CONFERENCE_ROOM, BUNNY_10K, STANFORD_DRAGON } a.k.a. { 0..4 };
// - Ray distribution: 
//   { PRIMARY_VIEW1/2/3, FIRST_BOUNCE, SECOND_BOUNCE, AO_RAYS  } a.k.a. { 0..5 }.
// Specify a valid tga file name as the last parameter to render scene/view to a file.
// An unspecified ray distribution will analyze the specified BVH builder:
// - Construction time (average over several runs);
// - SAH and EPO cost.

#include "headers.h"

Scene scene = CONFERENCE_ROOM;

int main()
{
	PrintHeader(); // see tools.cpp
	InitOpenCL( 0 /* 0, or "AMD" or "NVIDIA" or "Intel" */ );
	InitVulkan( 0 /* same choices; 0 prefers a discrete GPU. Pick the OpenCL GPU to compare the two. */ );
	vector<Experiment*> experiment;
	// construct list of experiments

#if 0 // run one block at a time to reduce throttling effects.

	// 1. BVH construction
	// PART 1 - TinyBVH, from ultra-fast to ultra-quality
	scene = CRYTEK_SPONZA;
	experiment.push_back( new Experiment( BVH2, HPLOC, scene ) );
	experiment.push_back( new Experiment( BVH2, HPLOC | OPTIMIZE, scene ) );
	experiment.push_back( new Experiment( BVH2, SIMDBUILD, scene ) );
	experiment.push_back( new Experiment( BVH2, BONSAI, scene ) );
	experiment.push_back( new Experiment( BVH2, BONSAI | SPATIALSPLITS, scene ) );
	experiment.push_back( new Experiment( BVH2, BONSAI | SPATIALSPLITS | OPTIMIZE, scene ) );
	// experiment.push_back( new Experiment( BVH2, PRESPLIT, scene ) );
	experiment.push_back( new Experiment( BVH2, FULLSWEEP, scene ) );
	experiment.push_back( new Experiment( BVH2, SPATIALSPLITS, scene ) );
	experiment.push_back( new Experiment( BVH2, SPATIALSPLITS | OPTIMIZE, scene ) );
	// PART 2 - Embree, low-medium-high
	experiment.push_back( new Experiment( EMBREE, LOW, scene ) );
	experiment.push_back( new Experiment( EMBREE, MEDIUM, scene ) );
	experiment.push_back( new Experiment( EMBREE, HIGH, scene ) );
	// PART 3 - Madmann91, low-medium-high
	experiment.push_back( new Experiment( MADMANN91, LOW, scene ) );
	experiment.push_back( new Experiment( MADMANN91, MEDIUM, scene ) );
	experiment.push_back( new Experiment( MADMANN91, HIGH, scene ) );
	// PART 4 - Derived layouts
	experiment.push_back( new Experiment( BVH4_WIVE, SIMDBUILD, scene ) );
	experiment.push_back( new Experiment( BVH4_WIVE, SPATIALSPLITS, scene ) );
	experiment.push_back( new Experiment( BVH8_WIVE, SIMDBUILD, scene ) );
	experiment.push_back( new Experiment( BVH8_WIVE, SPATIALSPLITS, scene ) );

#endif

#if 0

	// 2. CPU BVH traversal
	// PART 1 - TinyBVH, from default bvh via quick bvh builds to ultra-quality
	scene = CRYTEK_SPONZA;
	experiment.push_back( new Experiment( BVH2, SIMDBUILD, scene, PRIMARY_VIEW1 ) );
	experiment.push_back( new Experiment( BVH2, SIMDBUILD, scene, PRIMARY_VIEW2 ) );
	experiment.push_back( new Experiment( BVH2, SIMDBUILD, scene, PRIMARY_VIEW3 ) );
	experiment.push_back( new Experiment( BVH2, SIMDBUILD, scene, FIRST_BOUNCE ) );
	experiment.push_back( new Experiment( BVH2, SIMDBUILD, scene, AO_RAYS ) );
	experiment.push_back( new Experiment( BVH2, SIMDBUILD | SPATIALSPLITS, scene, PRIMARY_VIEW1 ) );
	experiment.push_back( new Experiment( BVH2, SIMDBUILD | SPATIALSPLITS, scene, PRIMARY_VIEW2 ) );
	experiment.push_back( new Experiment( BVH2, SIMDBUILD | SPATIALSPLITS, scene, PRIMARY_VIEW3 ) );
	experiment.push_back( new Experiment( BVH2, SIMDBUILD | SPATIALSPLITS, scene, FIRST_BOUNCE ) );
	experiment.push_back( new Experiment( BVH2, SIMDBUILD | SPATIALSPLITS, scene, AO_RAYS ) );
	experiment.push_back( new Experiment( BVH4_WIVE, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW1 ) ); // BVH for fast traversal
	experiment.push_back( new Experiment( BVH4_WIVE, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW2 ) );
	experiment.push_back( new Experiment( BVH4_WIVE, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW3 ) );
	experiment.push_back( new Experiment( BVH4_WIVE, SPATIALSPLITS | OPTIMIZE, scene, FIRST_BOUNCE ) );
	experiment.push_back( new Experiment( BVH4_WIVE, SPATIALSPLITS | OPTIMIZE, scene, AO_RAYS ) );
	experiment.push_back( new Experiment( BVH8_WIVE, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW1 ) ); // HQ BVH
	experiment.push_back( new Experiment( BVH8_WIVE, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW2 ) );
	experiment.push_back( new Experiment( BVH8_WIVE, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW3 ) );
	experiment.push_back( new Experiment( BVH8_WIVE, SPATIALSPLITS | OPTIMIZE, scene, FIRST_BOUNCE ) );
	experiment.push_back( new Experiment( BVH8_WIVE, SPATIALSPLITS | OPTIMIZE, scene, AO_RAYS ) );
	experiment.push_back( new Experiment( BVH4_WIVE, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW1, PACKETS ) );
	experiment.push_back( new Experiment( BVH4_WIVE, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW2, PACKETS ) );
	experiment.push_back( new Experiment( BVH4_WIVE, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW3, PACKETS ) );
	experiment.push_back( new Experiment( BVH4_WIVE, SPATIALSPLITS | OPTIMIZE, scene, FIRST_BOUNCE, PACKETS ) );
	experiment.push_back( new Experiment( BVH4_WIVE, SPATIALSPLITS | OPTIMIZE, scene, AO_RAYS, PACKETS ) );

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
	experiment.push_back( new Experiment( BVH2, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW1, MULTICORE ) );
	experiment.push_back( new Experiment( BVH2, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW2, MULTICORE ) );
	experiment.push_back( new Experiment( BVH2, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW3, MULTICORE ) );
	experiment.push_back( new Experiment( BVH2, SPATIALSPLITS | OPTIMIZE, scene, FIRST_BOUNCE, MULTICORE ) );
	experiment.push_back( new Experiment( BVH2, SPATIALSPLITS | OPTIMIZE, scene, AO_RAYS, MULTICORE ) );
	experiment.push_back( new Experiment( BVH4_WIVE, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW1, MULTICORE ) );
	experiment.push_back( new Experiment( BVH4_WIVE, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW2, MULTICORE ) );
	experiment.push_back( new Experiment( BVH4_WIVE, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW3, MULTICORE ) );
	experiment.push_back( new Experiment( BVH4_WIVE, SPATIALSPLITS | OPTIMIZE, scene, FIRST_BOUNCE, MULTICORE ) );
	experiment.push_back( new Experiment( BVH4_WIVE, SPATIALSPLITS | OPTIMIZE, scene, AO_RAYS, MULTICORE ) );
	experiment.push_back( new Experiment( BVH8_WIVE, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW1, MULTICORE ) );
	experiment.push_back( new Experiment( BVH8_WIVE, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW2, MULTICORE ) );
	experiment.push_back( new Experiment( BVH8_WIVE, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW3, MULTICORE ) );
	experiment.push_back( new Experiment( BVH8_WIVE, SPATIALSPLITS | OPTIMIZE, scene, FIRST_BOUNCE, MULTICORE ) );
	experiment.push_back( new Experiment( BVH8_WIVE, SPATIALSPLITS | OPTIMIZE, scene, AO_RAYS, MULTICORE ) );
	experiment.push_back( new Experiment( BVH4_WIVE, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW1, MULTICORE | PACKETS ) );
	experiment.push_back( new Experiment( BVH4_WIVE, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW2, MULTICORE | PACKETS ) );
	experiment.push_back( new Experiment( BVH4_WIVE, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW3, MULTICORE | PACKETS ) );

#endif

#if 0

#ifdef ENABLE_OPENCL
	// 4. GPU TRAVERSAL
	scene = CRYTEK_SPONZA;
	experiment.push_back( new Experiment( GPU_BVH, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW1, USE_GPU ) );
	experiment.push_back( new Experiment( GPU_BVH, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW2, USE_GPU ) );
	experiment.push_back( new Experiment( GPU_BVH, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW3, USE_GPU ) );
	experiment.push_back( new Experiment( GPU_BVH, SPATIALSPLITS | OPTIMIZE, scene, FIRST_BOUNCE, USE_GPU ) );
	experiment.push_back( new Experiment( GPU_BVH, SPATIALSPLITS | OPTIMIZE, scene, AO_RAYS, USE_GPU ) );
	experiment.push_back( new Experiment( GPU_BVH4, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW1, USE_GPU ) );
	experiment.push_back( new Experiment( GPU_BVH4, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW2, USE_GPU ) );
	experiment.push_back( new Experiment( GPU_BVH4, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW3, USE_GPU ) );
	experiment.push_back( new Experiment( GPU_BVH4, SPATIALSPLITS | OPTIMIZE, scene, FIRST_BOUNCE, USE_GPU ) );
	experiment.push_back( new Experiment( GPU_BVH4, SPATIALSPLITS | OPTIMIZE, scene, AO_RAYS, USE_GPU ) );
	experiment.push_back( new Experiment( CWBVH, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW1, USE_GPU ) );
	experiment.push_back( new Experiment( CWBVH, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW2, USE_GPU ) );
	experiment.push_back( new Experiment( CWBVH, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW3, USE_GPU ) );
	experiment.push_back( new Experiment( CWBVH, SPATIALSPLITS | OPTIMIZE, scene, FIRST_BOUNCE, USE_GPU ) );
	experiment.push_back( new Experiment( CWBVH, SPATIALSPLITS | OPTIMIZE, scene, AO_RAYS, USE_GPU ) );
	if (tinyocl::Kernel::isRDNA3 || tinyocl::Kernel::isRDNA35 || tinyocl::Kernel::isRDNA4)
	{
		experiment.push_back( new Experiment( AMD_HWRT, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW1, USE_GPU ) );
		experiment.push_back( new Experiment( AMD_HWRT, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW2, USE_GPU ) );
		experiment.push_back( new Experiment( AMD_HWRT, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW3, USE_GPU ) );
		experiment.push_back( new Experiment( AMD_HWRT, SPATIALSPLITS | OPTIMIZE, scene, FIRST_BOUNCE, USE_GPU ) );
		experiment.push_back( new Experiment( AMD_HWRT, SPATIALSPLITS | OPTIMIZE, scene, AO_RAYS, USE_GPU ) );
	}
	if (tinyocl::Kernel::isRDNA4)
	{
		experiment.push_back( new Experiment( AMD_HWRT8, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW1, USE_GPU ) );
		experiment.push_back( new Experiment( AMD_HWRT8, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW2, USE_GPU ) );
		experiment.push_back( new Experiment( AMD_HWRT8, SPATIALSPLITS | OPTIMIZE, scene, PRIMARY_VIEW3, USE_GPU ) );
		experiment.push_back( new Experiment( AMD_HWRT8, SPATIALSPLITS | OPTIMIZE, scene, FIRST_BOUNCE, USE_GPU ) );
		experiment.push_back( new Experiment( AMD_HWRT8, SPATIALSPLITS | OPTIMIZE, scene, AO_RAYS, USE_GPU ) );
	}
#endif

#endif

#if 1

	// 5. GPU TRAVERSAL, REFERENCE: the driver's own BVH through Vulkan ray queries, low-medium-high
	// ROTATE: the BLAS in the frame BVH::Orient picks, the TLAS instance turns it back (Bistro: about 23 degrees).
	scene = BISTRO_EXTERIOR;
	experiment.push_back( new Experiment( VULKAN_RT, LOW, scene ) );
	experiment.push_back( new Experiment( VULKAN_RT, MEDIUM, scene ) );
	experiment.push_back( new Experiment( VULKAN_RT, HIGH, scene ) );
	experiment.push_back( new Experiment( VULKAN_RT, HIGH, scene, PRIMARY_VIEW1 ) );
	experiment.push_back( new Experiment( VULKAN_RT, HIGH, scene, PRIMARY_VIEW2 ) );
	experiment.push_back( new Experiment( VULKAN_RT, HIGH, scene, PRIMARY_VIEW3 ) );
	experiment.push_back( new Experiment( VULKAN_RT, HIGH, scene, FIRST_BOUNCE ) );
	experiment.push_back( new Experiment( VULKAN_RT, HIGH, scene, AO_RAYS ) );
	experiment.push_back( new Experiment( VULKAN_RT, HIGH | ROTATE, scene, PRIMARY_VIEW1 ) );
	experiment.push_back( new Experiment( VULKAN_RT, HIGH | ROTATE, scene, PRIMARY_VIEW2 ) );
	experiment.push_back( new Experiment( VULKAN_RT, HIGH | ROTATE, scene, PRIMARY_VIEW3 ) );
	experiment.push_back( new Experiment( VULKAN_RT, HIGH | ROTATE, scene, FIRST_BOUNCE ) );
	experiment.push_back( new Experiment( VULKAN_RT, HIGH | ROTATE, scene, AO_RAYS ) );

#endif

	// run experiments
	for (size_t i = 0; i < experiment.size(); i++) experiment[i]->Run();
	// all done.
	return 0;
}