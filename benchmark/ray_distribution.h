#pragma once

namespace tinybvh
{

enum RaySet : int { 
	UNSPECIFIED = -1,
	PRIMARY_VIEW1 = 0, 
	PRIMARY_VIEW2, 
	PRIMARY_VIEW3,
};

class PrimitiveSet;
class RayDistribution
{
public:
	RayDistribution( RaySet r, PrimitiveSet* p );
	~RayDistribution();
	char* GetDescription() { return desc; }
	RaySet raySet = RaySet::UNSPECIFIED;
	uint32_t rayCount = 0;
	bvhvec3* O = 0;			// ray origins
	bvhvec3* D = 0;			// ray directions
	float* tmin, *tmax = 0;	// ray start and end
	char desc[256];
};

}; // namespace tinybvh

// EOF