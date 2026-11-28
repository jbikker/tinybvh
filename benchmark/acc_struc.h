#pragma once

namespace tinybvh
{

enum BuildFlags : int {
	NO_FLAGS = 0,
	INDEXED = 1,
	SIMDBUILD = 2,
	FULLSWEEP = 4,
	LBVH = 8,
	BONSAI = 16,
	HPLOC = 32,
	PRESPLIT = 64,
	SPATIALSPLITS = 128,
	OPTIMIZE = 256,
	BAKED = 512,
	LOW = 1024, // low, medium, high: For Embree and Madmann91.
	MEDIUM = 2048,
	HIGH = 4096,
	ROTATE = 8192 // pick a rotation that tightens the boxes (BVH::orientation), where the layout can carry it
};

enum BVHLayout
{
	BVH2 = 0,	// default 2-wide BVH
	BVH4,		// 4-wide BVH (a.k.a. MBVH)
	BVH4_WIVE,	// 4-wide BVH with optimized SSE traversal
	BVH8_WIVE,	// 4-wide BVH with optimized AVX2 traversal
	GPU_BVH,	// BVH2 optimized for GPU traversal
	GPU_BVH4,	// BVH4 optimized for GPU traversal
	CWBVH,		// 'Compressed Wide BVH'
	AMD_HWRT,	// Native AMD format
	MADMANN91,	// for reference: latest madmann91 BVH lib
	EMBREE,		// for reference: latest Embree lib.
	AMD_HWRT8,	// Native AMD 8-wide format
	VULKAN_RT,	// for reference: the GPU driver's own acceleration structure, through Vulkan ray queries
};

class PrimitiveSet;
class AccStruc
{
public:
	AccStruc( BVHLayout bvhLayout, BuildFlags bvhFlags );
	char* GetDescription() { return desc; }
	void PrepareBuild();
	BVHBase* Build( PrimitiveSet* primSet );
	BVHBase* LoadBVH( const char* bakedBVH );
	void PostBuild();
	BVHBase* GetBVH() { return bvh; }
	float SAHCost();
	float EPOCost();
	float IntersectBatch( char* rayData, const int rayCount );
	void IntersectBatchMT( char* rayData, const int rayCount );
	void OcclusionBatch( char* rayData, const int rayCount );
	void OcclusionBatchMT( char* rayData, const int rayCount );
	float IntersectBatchPackets( char* rayData, const int rayCount );
	void IntersectBatchMTPackets( char* rayData, const int rayCount );
	void OcclusionBatchPackets( char* rayData, const int rayCount );
	void OcclusionBatchMTPackets( char* rayData, const int rayCount );
	bvhvec3 SceneExtent();
	int NodeCount();
	BVHLayout layout = BVH2;
	char desc[128], shrt[64], flagShrt[64];
	VulkanRT* GetVulkan() { return vulkan; }
	void ReleaseGPU() { delete vulkan; vulkan = 0; vulkanPrims = 0; } // experiments are kept: free their GPU memory
private:
	void ReportRotation( const bool oriented, const float* orientation );
	VulkanRT* vulkan = 0;
	PrimitiveSet* vulkanPrims = 0; // geometry uploaded for 'vulkan'
	bool rotationReported = false;
	PrimitiveSet* primSet = 0;
	BVHBase* bvh = 0;
	BuildFlags flags = NO_FLAGS;
	float sah = 0, epo = 0;
	bvh::v2::Bvh<bvh::v2::Node<float,3>> madmannbvh;
	std::vector<PrecomputedTri> precomputed_tris;
	bool mmTrisPrecomputed = false;
};

inline BuildFlags operator|( BuildFlags a, BuildFlags b )
{
	return static_cast<BuildFlags>(static_cast<int>(a) | static_cast<int>(b));
}

};

// EOF