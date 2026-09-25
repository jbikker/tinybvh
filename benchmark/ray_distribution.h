#pragma once

namespace tinybvh
{

#define RAY_BATCH_SIZE	(1024*1024)

enum RaySet : int { 
	UNSPECIFIED = -1,
	PRIMARY_VIEW1 = 0, 
	PRIMARY_VIEW2, 
	PRIMARY_VIEW3,
	FIRST_BOUNCE,
	SECOND_BOUNCE,
	AO_RAYS
};

class PrimitiveSet;
class RayDistribution
{
public:
	RayDistribution( RaySet r, PrimitiveSet* p );
	~RayDistribution();
	char* GetDescription() { return desc; }
	void WriteToFile( const char* fileName );
	RaySet raySet = RaySet::UNSPECIFIED;
	uint32_t rayCount = 0;
	bvhvec3* O = 0;			// ray origins
	bvhvec3* D = 0;			// ray directions
	float* tmin, *tmax = 0;	// ray start and end
private:
	void UpdateViewPyramid( const bvhvec3 camPos, const bvhvec3 camDir );
	bvhvec3 eye, p1, p2, p3;
public:
	char desc[128], shrt[64];
};

}; // namespace tinybvh

// EOF