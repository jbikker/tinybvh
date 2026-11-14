#include "headers.h"

namespace tinybvh
{

bvh::v2::ThreadPool thread_pool;
bvh::v2::ParallelExecutor executor( thread_pool );
RTCScene embreeScene;
RTCDevice embreeDevice;
RTCGeometry embreeGeom;
void embreeError( void* userPtr, enum RTCError error, const char* str ) { printf( "error %d: %s\n", error, str ); }

PrimitiveSet::PrimitiveSet( uint32_t scene )
{
	if (scene == Scene::CRYTEK_SPONZA)
	{
		// load raw vertex data for Crytek's Sponza
		AddMesh( "./testdata/cryteksponza.bin" );
		camPos[0] = bvhvec3( -15.24f, 21.5f, 2.54f ), camDir[0] = tinybvh_normalize( bvhvec3( 0.826f, -0.438f, -0.356f ) );
		camPos[1] = bvhvec3( -34, 5, 11.26f ), camDir[1] = tinybvh_normalize( bvhvec3( 0.9427f, 0.0292f, -0.3324f ) );
		camPos[2] = bvhvec3( -1.3, 4.96, 12.28 ), camDir[2] = tinybvh_normalize( bvhvec3( -0.9886f, 0.0507f, -0.1419f ) );
	}
	else if (scene == Scene::BISTRO_EXTERIOR)
	{
		// load the Bistro Exterior
		AddMesh( "./testdata/bistro_ext_part1.bin" );
		AddMesh( "./testdata/bistro_ext_part2.bin" );
		camPos[0] = bvhvec3( -24.2f, -8.2f, -2.0f ), camDir[0] = tinybvh_normalize( bvhvec3( -13.55f, -8.26f, -1.74f ) - camPos[0] );
		camPos[1] = bvhvec3( -19.8f, 16.4f, -9.3f ), camDir[1] = tinybvh_normalize( bvhvec3( -15.17f, -9.57f, -11.34f ) - camPos[1] );
		camPos[2] = bvhvec3( 30.9f, 1.4f, 26.2f ), camDir[2] = tinybvh_normalize( bvhvec3( 26.74f, -3.48f, 21.52f ) - camPos[2] );
	}
	else if (scene == Scene::SAN_MIGUEL)
	{
		// load 30 million triangles for San Miguel
		AddMesh( "./testdata/sanmiguel.bin" );
		camPos[0] = bvhvec3( 20.89f, 1.25f, 7.13f ), camDir[0] = tinybvh_normalize( bvhvec3( -1, 0, -0.07f ) );
		camPos[1] = bvhvec3( 9.73f, 2.05f, -1.04f ), camDir[1] = tinybvh_normalize( bvhvec3( -0.69f, -0.67f, -0.28f ) );
		camPos[2] = bvhvec3( 18.83f, 8.02f, -2.84f ), camDir[2] = tinybvh_normalize( bvhvec3( -0.26f, -0.18f, 0.95f ) );
	}
	else if (scene == CONFERENCE_ROOM)
	{
		// load the classic Conference Room
		AddMesh( "./testdata/conference.bin" );
		camPos[0] = bvhvec3( -18.0f, -1.3f, -6.7f ), camDir[0] = tinybvh_normalize( bvhvec3( -9.93f, -2.66f, -2.09f ) - camPos[0] );
		camPos[1] = bvhvec3( -1.4f, -3.4f, 10.7f ), camDir[1] = tinybvh_normalize( bvhvec3( -4.13f, -4.03f, 15.14f ) - camPos[1] );
		camPos[2] = bvhvec3( 21.5f, 1.8f, -10.2f ), camDir[2] = tinybvh_normalize( bvhvec3( -0.59f, -2.68f, -2.39f ) - camPos[2] );
	}
	else if (scene == BUNNY_10K)
	{
		// load a 10k triangle version of the Stanford Bunny
		AddMesh( "./testdata/bunny10k.bin" );
		camPos[0] = bvhvec3( -14.3f, 15.1f, 12.6f ), camDir[0] = tinybvh_normalize( bvhvec3( -7.63f, 12.0f, 5.26f ) - camPos[0] );
		camPos[1] = bvhvec3( -9.3f, 25.4f, 3.7f ), camDir[1] = tinybvh_normalize( bvhvec3( -5.34f, 15.95f, 0.79f ) - camPos[1] );
		camPos[2] = bvhvec3( -2.2f, 6.4f, 1.7f ), camDir[2] = tinybvh_normalize( bvhvec3( -8.98f, 13.42f, 3.33f ) - camPos[2] );
	}
	else if (scene == STANFORD_DRAGON)
	{
		// load the Stanford Dragon model
		AddMesh( "./testdata/dragon.bin" );
		camPos[0] = bvhvec3( -4.3f, 3.2f, -5.8f ), camDir[0] = tinybvh_normalize( bvhvec3( -0.39f, 2.09f, -2.48f ) - camPos[0] );
		camPos[1] = bvhvec3( -4.8f, 2.1f, -7.2f ), camDir[1] = tinybvh_normalize( bvhvec3( -0.53f, 0.80f, -1.88f ) - camPos[1] );
		camPos[2] = bvhvec3( -0.7f, 4.9f, 0.4f ), camDir[2] = tinybvh_normalize( bvhvec3( 0.13f, 3.28f, -1.36f ) - camPos[2] );
	}
	else if (scene == LEGO_PORSCHE)
	{
		// load the Lego Porsche model
		AddMesh( "./testdata/legoporsche.bin" );
		camPos[0] = bvhvec3( 0.33f, -2.31f, 0.30f ), camDir[0] = tinybvh_normalize( bvhvec3( -0.11f, 0.72f, 0.69f ) );
		camPos[1] = bvhvec3( 0.29f, 0.55f, -2.02f ), camDir[1] = tinybvh_normalize( bvhvec3( -0.24f, -0.28f, 0.93f ) );
		camPos[2] = bvhvec3( 3.98f, 1.67f, 4.28f ), camDir[2] = tinybvh_normalize( bvhvec3( -0.70f, -0.48f, -0.53f ) );
	}
	else if (scene == VICTORIAN_HOUSE)
	{
		// load the Victorian House
		AddMesh( "./testdata/victorianhouse.bin" );
		camPos[0] = bvhvec3( 0.16f, -0.68f, 1.23f ), camDir[0] = tinybvh_normalize( bvhvec3( -0.20f, 0.12f, -0.97f ) );
		camPos[1] = bvhvec3( -2.33f, 1.36f, -0.39f ), camDir[1] = tinybvh_normalize( bvhvec3( 0.72f, -0.66f, 0.21f ) );
		camPos[2] = bvhvec3( -0.82f, -0.66f, 2.33f ), camDir[2] = tinybvh_normalize( bvhvec3( 0.35f, 0.16f, -0.92f ) );
	}
	else if (scene == HAIRBALL)
	{
		// load the Victorian House
		AddMesh( "./testdata/hairball.bin" );
		camPos[0] = bvhvec3( -1.04f, -6.64f, -1.79f ), camDir[0] = tinybvh_normalize( bvhvec3( 0.12f, 0.95f, 0.28f ) );
		camPos[1] = bvhvec3( -3.05f, -1.61f, 0.60f ), camDir[1] = tinybvh_normalize( bvhvec3( 0.96f, 0.24f, 0.13f ) );
		camPos[2] = bvhvec3( -4.61f, -2.00f, 0.38f ), camDir[2] = tinybvh_normalize( bvhvec3( 0.41f, 0.89f, 0.21f ) );
	}
	else
	{
		exit( 0 ); // invalid scene.
	}
	// build dummy indices: e.g. tri 0 is defined by vertices 0, 1 and 2.
	indices = new uint32_t[primCount * 3];
	for( uint32_t i = 0; i < primCount * 3; i++ ) indices[i] = i;
	// construct description
	switch (scene)
	{
	case Scene::CRYTEK_SPONZA: 
	{
		strncpy( desc, "Crytek Sponza", 128 );
		strncpy( shrt, "sponza", 64 );
		break;
	}
	case Scene::BISTRO_EXTERIOR: 
	{
		strncpy( desc, "Bistro Exterior", 128 ); 
		strncpy( shrt, "bistro", 64 ); 
		break;
	}
	case Scene::SAN_MIGUEL: 
	{
		strncpy( desc, "San Miguel", 128 ); 
		strncpy( shrt, "sanmiguel", 64 ); 
		break;
	}
	case Scene::CONFERENCE_ROOM: 
	{
		strncpy( desc, "Conference Room", 128 ); 
		strncpy( shrt, "confroom", 64 ); 
		break;
	}
	case Scene::BUNNY_10K: 
	{
		strncpy( desc, "Stanford Bunny (10k)", 128 ); 
		strncpy( shrt, "bunny", 64 ); 
		break;
	}
	case Scene::STANFORD_DRAGON: 
	{
		strncpy( desc, "Stanford Dragon", 128 ); 
		strncpy( shrt, "dragon", 64 ); 
		break;
	}
	case Scene::VICTORIAN_HOUSE: 
	{
		strncpy( desc, "Victorian House", 128 ); 
		strncpy( shrt, "house", 64 ); 
		break;
	}
	case Scene::HAIRBALL: 
	{
		strncpy( desc, "Hairball", 128 ); 
		strncpy( shrt, "hairball", 64 ); 
		break;
	}
	case Scene::LEGO_PORSCHE: 
	{
		strncpy( desc, "Lego Porsche", 128 ); 
		strncpy( shrt, "porsche", 64 ); 
		break;
	}
	default: 
	{
		strncpy( desc, "UNKNWON SCENE", 128 ); 
		strncpy( shrt, "unkown", 64 ); 
		break;
	}
	};
	// convert to madmann91 data
	bvhvec4* v = verts;
	for (int N = (int)primCount * 3, i = 0; i < N; i += 3) tris.emplace_back(
		_Vec3( v[i].x, v[i].y, v[i].z ), _Vec3( v[i + 1].x, v[i + 1].y, v[i + 1].z ),
		_Vec3( v[i + 2].x, v[i + 2].y, v[i + 2].z ) );
	bboxes.resize( tris.size() );
	centers.resize( tris.size() );
	executor.for_each( 0, tris.size(), [&]( size_t begin, size_t end )
		{ for (size_t i = begin; i < end; ++i) bboxes[i] = tris[i].get_bbox(),
		centers[i] = tris[i].get_center(); } );
	// convert to Embree data
	embreeDevice = rtcNewDevice( NULL );
	rtcSetDeviceErrorFunction( embreeDevice, embreeError, NULL );
	embreeScene = rtcNewScene( embreeDevice );
	embreeGeom = rtcNewGeometry( embreeDevice, RTC_GEOMETRY_TYPE_TRIANGLE );
	float* embVertices = (float*)rtcSetNewGeometryBuffer( embreeGeom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 4 * sizeof( float ), primCount * 3 );
	unsigned* embIndices = (unsigned*)rtcSetNewGeometryBuffer( embreeGeom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, 3 * sizeof( unsigned ), primCount );
	for (uint32_t i = 0; i < primCount * 3; i++)
	{
		embVertices[i * 4 + 0] = verts[i].x;
		embVertices[i * 4 + 1] = verts[i].y;
		embVertices[i * 4 + 2] = verts[i].z, embIndices[i] = i; // Note: not using shared vertices.
		embVertices[i * 4 + 3] = 0;
	}
}

void PrimitiveSet::AddMesh( const char* file, float scale, bvhvec3 pos, int c, int N )
{
	std::fstream s{ file, s.binary | s.in }; s.read( (char*)&N, 4 );
	bvhvec4* data = (bvhvec4*)tinybvh::malloc64( (N + primCount) * 48 );
	if (verts) memcpy( data, verts, primCount * 48 ), tinybvh::free64( verts );
	verts = data, s.read( (char*)verts + primCount * 48, N * 48 ), primCount += N;
	for (int* b = (int*)verts + (primCount - N) * 12, i = 0; i < N * 3; i++)
		*(bvhvec3*)b = *(bvhvec3*)b * scale + pos, b[3] = c ? c : b[3], b += 4;
}

};