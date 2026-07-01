#pragma once

namespace tinybvh
{

enum BuildFlags : int {
	NO_FLAGS = 0,
	INDEXED = 1,
	AVXBUILD = 2,
	FULLSWEEP = 4,
	PRESPLIT = 8,
	SPATIALSPLITS = 16,
	OPTIMIZE = 32,
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
	MADMANN91,	// for reference: latest madmann91 BVH lib
	EMBREE		// for reference: latest Embree lib.
};

class PrimitiveSet;
class AccStruc
{
public:
	AccStruc( BVHLayout bvhLayout, BuildFlags bvhFlags );
	char* GetDescription() { return desc; }
	BVHBase* Build( PrimitiveSet* primSet );
	BVHBase* GetBVH() { return bvh; }
	float SAHCost();
	float EPOCost();
	void IntersectBatch( __m256* r256, const int rayCount );
	void IntersectBatchMT( __m256* r256, const int rayCount );
	void OcclusionBatch( __m256* r256, const int rayCount );
	void OcclusionBatchMT( __m256* r256, const int rayCount );
	bvhvec3 SceneExtent() { return bvh->aabbMax - bvh->aabbMin; }
	int NodeCount();
	BVHLayout layout = BVH2;
private:
	PrimitiveSet* primSet = 0;
	BVHBase* bvh = 0;
	BuildFlags flags = NO_FLAGS;
	float sah = 0, epo = 0;
	char desc[256];
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