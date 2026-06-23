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

class PrimitiveSet;
class AccStruc
{
public:
	AccStruc( BVHBase::BVHType bvhLayout, BuildFlags bvhFlags );
	char* GetDescription() { return desc; }
	BVHBase* Build( PrimitiveSet* primSet );
	float SAHCost();
	float EPOCost();
	void IntersectBatch( Ray* raySet, int rayCount );
	void OcclusionBatch( Ray* raySet, int rayCount );
	bvhvec3 SceneExtent() { return bvh->aabbMax - bvh->aabbMin; }
	int NodeCount();
private:
	BVHBase::BVHType layout = BVHBase::BVHType::LAYOUT_BVH;
	PrimitiveSet* primSet = 0;
	BVHBase* bvh = 0;
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