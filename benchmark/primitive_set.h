#pragma once

namespace tinybvh
{

enum Scene : uint32_t { 
	CRYTEK_SPONZA, 
	BISTRO_EXTERIOR,
	CONFERENCE_ROOM,
	BUNNY_10K,
	STANFORD_DRAGON
};

class PrimitiveSet
{
public:
	PrimitiveSet( uint32_t scene );
	char* GetDescription() { return desc; }
	bvhvec3 GetCameraPos( uint32_t i ) const { return camPos[i % 3]; }
	bvhvec3 GetCameraDir( uint32_t i ) const { return camDir[i % 3]; }
	Scene scene = CRYTEK_SPONZA;
	uint32_t primCount;
	bvhvec4* verts = 0;
	uint32_t* indices = 0;
	char desc[256];
private:
	void AddMesh( const char* file, float scale = 1, bvhvec3 pos = {}, int c = 0, int N = 0 );
	bvhvec3 camPos[3], camDir[3];
public:
	// madmann91 data
	std::vector<bvh::v2::Tri<float, 3>> tris;
	std::vector<bvh::v2::BBox<float, 3>> bboxes;
	std::vector<bvh::v2::Vec<float, 3>> centers;
};

}; // namespace tinybvh

// EOF