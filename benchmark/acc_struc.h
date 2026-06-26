#pragma once

namespace tinybvh
{

enum BuildFlags : int {
	NO_FLAGS = 0,
	AVXBUILD = 1,
	FULLSWEEP = 2,
	PRESPLIT = 4,
	SPATIALSPLITS = 8,
	OPTIMIZE = 16
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
	MADMANN91,	// for reference: Madmann91's BVH lib
};

class PrimitiveSet;
class AccStruc
{
public:
	AccStruc( BVHLayout bvhLayout, BuildFlags bvhFlags );
	char* GetDescription() { return desc; }
	BVHBase* Build( PrimitiveSet* primSet );
	float SAHCost();
	float EPOCost();
	void IntersectBatch( Ray* raySet, int rayCount );
	void OcclusionBatch( Ray* raySet, int rayCount );
	bvhvec3 SceneExtent() { return bvh->aabbMax - bvh->aabbMin; }
	int NodeCount();
private:
	BVHLayout layout = BVH2;
	PrimitiveSet* primSet = 0;
	BVHBase* bvh = 0;
	bvh::v2::Bvh<bvh::v2::Node<float,3>> madmannbvh;
	BuildFlags flags = NO_FLAGS;
	float sah = 0, epo = 0;
	char desc[256];
};

inline BuildFlags operator|( BuildFlags a, BuildFlags b )
{
	return static_cast<BuildFlags>(static_cast<int>(a) | static_cast<int>(b));
}

};

// EOF