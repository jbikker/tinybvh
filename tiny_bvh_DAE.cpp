// This example is the starting point for the workshop at DAE / HOWEST

#define FENSTER_APP_IMPLEMENTATION
#define SCRWIDTH 800
#define SCRHEIGHT 600
#include "external/fenster.h" // https://github.com/zserge/fenster

#define TINYBVH_IMPLEMENTATION
#include "tiny_bvh.h"
#include <atomic>
#include <fstream>
#include <thread>
#include <vector>

using namespace tinybvh;

// application variables

static BVH bvh;
static bvhvec4* tris = 0;
static int triCount = 0, frameIdx = 0;
static std::atomic<int> tileIdx( 0 );

// setup view pyramid for a pinhole camera
static bvhvec3 view = tinybvh_normalize( bvhvec3( -1, 0, 0 ) );
static bvhvec3 eye( -15, 10, 0 ), C = eye + view;
static bvhvec3 p1 = C + bvhvec3( 0, 1, -1 ), p2 = C + bvhvec3( 0, 1, 1 ), p3 = C + bvhvec3( 0, -1, -1 );

// scene management - Append a file, with optional position, scale and color override, tinyfied
void AddMesh( const char* file, float scale = 1, bvhvec3 pos = {}, int c = 0, int N = 0 )
{
	std::fstream s{ file, s.binary | s.in };
	s.read( (char*)&N, 4 );
	bvhvec4* data = (bvhvec4*)malloc64( (N + triCount) * 48 );
	if (tris) memcpy( data, tris, triCount * 48 ), free64( tris );
	tris = data, s.read( (char*)tris + triCount * 48, N * 48 ), triCount += N;
	for (int* b = (int*)tris + (triCount - N) * 12, i = 0; i < N * 3; i++)
		*(bvhvec3*)b = *(bvhvec3*)b * scale + pos, b[3] = c ? c : b[3], b += 4;
}

// application init
void Init()
{
	// load raw vertex data
#if 1
	tris = (bvhvec4*)malloc64( 3 * sizeof( bvhvec4 ) );
	tris[0] = bvhvec4( -32, 3, -5, 0 );
	tris[1] = bvhvec4( -32, 20, 0, 0 );
	tris[2] = bvhvec4( -32, 3, 5, 0 );
	triCount = 1;
#else
	AddMesh( "./testdata/cryteksponza.bin", 1, bvhvec3( 0 ), 0xffffff );
#endif
	bvh.Build( tris, triCount );
}

// light transport calculation
bvhvec3 Trace( Ray ray, unsigned& seed, unsigned depth = 0 )
{
	// find primary intersection
	bvh.Intersect( ray );
	return ray.hit.t;
}

void TraceWorkerThread( uint32_t* buf, int tile )
{
	const int xtiles = SCRWIDTH / 20, ytiles = SCRHEIGHT / 20, tiles = xtiles * ytiles;
	while (tile < tiles)
	{
		unsigned tx = tile % xtiles, ty = tile / xtiles, seed = (tile + 1) * 17;
		for (int y = 0; y < 20; y++) for (int x = 0; x < 20; x++)
		{
			// setup primary ray
			const int pixel_x = tx * 20 + x, pixel_y = ty * 20 + y;
			const float u = (float)pixel_x / SCRWIDTH, v = (float)pixel_y / SCRHEIGHT;
			const bvhvec3 D = tinybvh_normalize( p1 + u * (p2 - p1) + v * (p3 - p1) - eye );
			// trace
			bvhvec3 color = Trace( Ray( eye, D ), seed );
			int c = (int)(tinybvh_min( 1.0f, color.x * 0.03f ) * 255.0f);
			buf[pixel_x + pixel_y * SCRWIDTH] = c + (c << 8) + (c << 16);
		}
		tile = tileIdx++;
	}
}

// Application Tick
void Tick( float delta_time_s, fenster& f, uint32_t* buf )
{
	// render tiles
	static unsigned threadCount = std::thread::hardware_concurrency();
	tileIdx = threadCount;
	std::vector<std::thread> threads;
	for (uint32_t i = 0; i < threadCount; i++) threads.emplace_back( &TraceWorkerThread, buf, i );
	for (auto& thread : threads) thread.join();
}

// Application Shutdown
void Shutdown() {}