#define FENSTER_APP_IMPLEMENTATION
#define SCRWIDTH 800
#define SCRHEIGHT 600
#include "external/fenster.h" // https://github.com/zserge/fenster

#define TINYBVH_IMPLEMENTATION
#include "tiny_bvh.h"
using namespace tinybvh;
#include <fstream>
#include <thread>

#define TINYSCENE_IMPLEMENTATION
#define TINYSCENE_USE_CUSTOM_VECTOR_TYPES
namespace tinyscene // override tinyscene's vector types with tinybvh's for easier interop
{
using ts_int2 = tinybvh::bvhint2;
using ts_int3 = tinybvh::bvhint3;
using ts_uint2 = tinybvh::bvhuint2;
using ts_uint3 = tinybvh::bvhuint3;
using ts_uint4 = tinybvh::bvhuint4;
using ts_vec2 = tinybvh::bvhvec2;
using ts_vec3 = tinybvh::bvhvec3;
using ts_vec4 = tinybvh::bvhvec4;
}
#include "tiny_scene.h" // very much in beta.
using namespace tinyscene;

// scene data
Scene scene;
BVH8_CPU legocar;

// view pyramid for a pinhole camera
static bvhvec3 eye( -15.24f, 21.5f, 2.54f ), p1, p2, p3;
static bvhvec3 view = tinybvh_normalize( bvhvec3( 0.826f, -0.438f, -0.356f ) );

// Application start: Initialize scene.
void Init()
{
	// Load a scene from a GLTF file using tinyscene.
	scene.AddScene( "./testdata/drone/scene.gltf", ts_mat4::scale( 0.1f ) );
	// Load camera position / direction from file.
	std::fstream t = std::fstream{ "camera.bin", t.binary | t.in };
	if (!t.is_open()) return;
	t.read( (char*)&eye, sizeof( eye ) );
	t.read( (char*)&view, sizeof( view ) );
	t.close();
}

// Camera interaction: WASD+RF for translation; cursor keys for rotation.
bool UpdateCamera( float delta_time_s, fenster& f )
{
	bvhvec3 right = tinybvh_normalize( tinybvh_cross( bvhvec3( 0, 1, 0 ), view ) ), up = 0.8f * tinybvh_cross( view, right );
	float moved = 0, spd = 10.0f * delta_time_s;
	if (f.keys['A'] || f.keys['D']) eye += right * (f.keys['D'] ? spd : -spd), moved = 1;
	if (f.keys['W'] || f.keys['S']) eye += view * (f.keys['W'] ? spd : -spd), moved = 1;
	if (f.keys['R'] || f.keys['F']) eye += up * 2.0f * (f.keys['R'] ? spd : -spd), moved = 1;
	if (f.keys[20]) view = tinybvh_normalize( view + right * -0.1f * spd ), moved = 1;
	if (f.keys[19]) view = tinybvh_normalize( view + right * 0.1f * spd ), moved = 1;
	if (f.keys[17]) view = tinybvh_normalize( view + up * -0.1f * spd ), moved = 1;
	if (f.keys[18]) view = tinybvh_normalize( view + up * 0.1f * spd ), moved = 1;
	// recalculate right, up
	right = tinybvh_normalize( tinybvh_cross( bvhvec3( 0, 1, 0 ), view ) ), up = 0.8f * tinybvh_cross( view, right );
	bvhvec3 C = eye + 1.2f * view;
	p1 = C - right + up, p2 = C + right + up, p3 = C - right - up;
	return moved > 0;
}

// Main ray tracing function: Calculates the (floating point) color for a pixel.
bvhvec3 Trace( Ray& ray )
{
	Scene::tlas->Intersect( ray );
	if (ray.hit.t >= 10000) return 0;
	return ray.hit.t * 0.03f;
	const uint32_t primIdx = ray.hit.prim & PRIM_IDX_MASK;
	const FatTri& tri = scene.meshPool[0]->triangles[primIdx];
	bvhvec3 N( tri.Nx, tri.Ny, tri.Nz );
	if (tinybvh_dot( N, ray.D ) > 0) N = -N;
	return (N + 1) * 0.5f;
}

// Render 20x20 pixel tiles using all cores.
static std::atomic<int> jobCount( 0 );
void WorkerThread( uint32_t* buf )
{
	int xtiles = SCRWIDTH / 20, ytiles = SCRHEIGHT / 20, tile;
tileloop:
	if ((tile = --jobCount) < 0) return; else tile = (xtiles * ytiles - 1) - tile;
	const int tx = tile % xtiles, ty = tile / xtiles;
	for (int y = 0; y < 20; y++) for (int x = 0; x < 20; x++) // trace 400 primary rays
	{
		const int pixelx = tx * 20 + x, pixely = ty * 20 + y;
		const float u = (float)pixelx / SCRWIDTH, v = (float)pixely / SCRHEIGHT;
		const bvhvec3 D = tinybvh_normalize( p1 + u * (p2 - p1) + v * (p3 - p1) - eye );
		Ray ray( eye, D );
		const bvhvec3 E = tinybvh_min( Trace( ray ), bvhvec3( 1 ) ) * 255.0f;
		buf[pixelx + pixely * SCRWIDTH] = (int)E.x + ((int)E.y << 8) + ((int)E.z << 16);
	}
	goto tileloop;
}

// Application Tick, exectuted once per frame.
void Tick( float delta_time_s, fenster& f, uint32_t* buf )
{
	static unsigned threadCount = std::thread::hardware_concurrency();
	UpdateCamera( delta_time_s, f );
	scene.UpdateSceneGraph( delta_time_s );
	jobCount = SCRWIDTH * SCRHEIGHT / 400;
	std::vector<std::thread> threads;
#ifdef _DEBUG
	for (unsigned i = 0; i < threadCount; i++) WorkerThread( buf ); // single thread in debug.
#else
	for (unsigned i = 0; i < threadCount; i++) threads.emplace_back( &WorkerThread, buf );
#endif
	for (auto& thread : threads) thread.join();
}

// Application Shutdown.
void Shutdown()
{
	// save camera position / direction to file
	std::fstream s = std::fstream{ "camera.bin", s.binary | s.out };
	s.write( (char*)&eye, sizeof( eye ) );
	s.write( (char*)&view, sizeof( view ) );
	s.close();
}