#define FENSTER_APP_IMPLEMENTATION
#define SCRWIDTH 800
#define SCRHEIGHT 600
#include "external/fenster.h" // https://github.com/zserge/fenster

#define NO_DOUBLE_PRECISION_SUPPORT
#define TINYBVH_IMPLEMENTATION
#define INST_IDX_BITS 8 // override default; space for 256 instances.
#include "tiny_bvh.h"
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

using namespace tinybvh;

struct Sphere { bvhvec3 pos; float r; };

tinyscene::Scene scene;
BVH8_CPU legocar;
static std::atomic<int> tileIdx( 0 );
static unsigned threadCount = std::thread::hardware_concurrency();

// setup view pyramid for a pinhole camera
static bvhvec3 eye( -15.24f, 21.5f, 2.54f ), p1, p2, p3;
static bvhvec3 view = tinybvh_normalize( bvhvec3( 0.826f, -0.438f, -0.356f ) );

void Init()
{
	int carid = scene.AddMesh( "./testdata/legocar.obj", 20 );
	tinyscene::Mesh* mesh = scene.meshPool[carid];
	legocar.Build( mesh->vertices.data(), (unsigned)mesh->triangles.size() );
}

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

void TraceWorkerThread( uint32_t* buf, int threadIdx )
{
	const int xtiles = SCRWIDTH / 20, ytiles = SCRHEIGHT / 20, tiles = xtiles * ytiles;
	int tile = threadIdx;
	while (tile < tiles)
	{
		const int tx = tile % xtiles, ty = tile / xtiles;
		const bvhvec3 L = tinybvh_normalize( bvhvec3( 1, 2, 3 ) );
		for (int y = 0; y < 20; y++) for (int x = 0; x < 20; x++)
		{
			// setup primary ray
			const int pixel_x = tx * 20 + x, pixel_y = ty * 20 + y;
			const float u = (float)pixel_x / SCRWIDTH, v = (float)pixel_y / SCRHEIGHT;
			const bvhvec3 D = tinybvh_normalize( p1 + u * (p2 - p1) + v * (p3 - p1) - eye );
			Ray ray( eye, D );
			legocar.Intersect( ray );
			if (ray.hit.t >= 10000) continue;
			uint32_t primIdx = ray.hit.prim & PRIM_IDX_MASK;
			tinyscene::FatTri& tri = scene.meshPool[0]->triangles[primIdx];
			bvhvec3 N( tri.Nx, tri.Ny, tri.Nz );
			if (tinybvh_dot( N, ray.D ) > 0) N = -N;
			int c = (int)(255.0f * fabs( N.y ));
			buf[pixel_x + pixel_y * SCRWIDTH] = c + (c << 8) + (c << 16);
		}
		tile = tileIdx++;
	}
}

void Tick( float delta_time_s, fenster& f, uint32_t* buf )
{
	UpdateCamera( delta_time_s, f );
	for (int i = 0; i < SCRWIDTH * SCRHEIGHT; i++) buf[i] = 0xaaaaff;
	tileIdx = threadCount;
	std::vector<std::thread> threads;
#ifdef _DEBUG
	for (uint32_t i = 0; i < threadCount; i++) TraceWorkerThread( buf, i );
#else
	for (uint32_t i = 0; i < threadCount; i++) threads.emplace_back( &TraceWorkerThread, buf, i );
#endif
	for (auto& thread : threads) thread.join();
}

void Shutdown() { /* nothing here. */ }