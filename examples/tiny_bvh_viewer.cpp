// Minimalistic object viewer application.
// Useful for inspecting BVHs and for finding camera views for the benchmark app.

#define FENSTER_APP_IMPLEMENTATION
#define SCRWIDTH 800
#define SCRHEIGHT 800
#define TILESIZE 20
#include "external/fenster.h" // https://github.com/zserge/fenster

#define TINYBVH_IMPLEMENTATION
#include "tiny_bvh.h"
#include <atomic>
#include <fstream>
#include <thread>
#include <vector>

using namespace tinybvh;

// Application variables

static BVH_GPU bvh;
static bvhvec4* tris = 0;
static int triCount = 0;
static std::atomic<int> tileIdx( 0 );
static unsigned threadCount = std::thread::hardware_concurrency();

// View pyramid for a pinhole camera
static bvhvec3 eye( 0, 30, 0 ), p1, p2, p3;
static bvhvec3 view = tinybvh_normalize( bvhvec3( -1, 0, 0 ) );

// Geometry access
bvhvec3 rgb32_to_vec3( const unsigned c ) { return bvhvec3( (float)(c >> 16), (float)((c >> 8) & 255), (float)(c & 255) ) * (1 / 255.f); }
bvhvec3 TriangleColor( const unsigned idx ) { return rgb32_to_vec3( *(unsigned*)&tris[idx * 3].w ); }
bvhvec3 TriangleNormal( const unsigned idx )
{
	bvhvec3 a = tris[idx * 3], b = tris[idx * 3 + 1], c = tris[idx * 3 + 2];
	return tinybvh_normalize( tinybvh_cross( b - a, a - c ) );
}

// Scene management - Append a file, with optional position, scale and color override, tinyfied
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

// Application init
void Init()
{
	// prepare scene
#if 0
	AddMesh( "./testdata/bistro_ext_part1.bin" );
	AddMesh( "./testdata/bistro_ext_part2.bin" );
#else
	AddMesh( "./testdata/cryteksponza.bin" );
#endif
	bvh.BuildHQ( tris, triCount );
	// load camera position / direction from file
	std::fstream t = std::fstream{ "camera.bin", t.binary | t.in };
	if (!t.is_open()) return;
	t.read( (char*)&eye, sizeof( eye ) );
	t.read( (char*)&view, sizeof( view ) );
}

// Keyboard handling
void UpdateCamera( float dt, fenster& f )
{
	bvhvec3 right = tinybvh_normalize( tinybvh_cross( bvhvec3( 0, 1, 0 ), view ) ), up = 0.8f * tinybvh_cross( view, right );
	// get camera controls.
	if (f.keys['A']) eye += right * -1.0f * dt * 10; else if (f.keys['D']) eye += right * dt * 10;
	if (f.keys['W']) eye += view * dt * 10; else if (f.keys['S']) eye += view * -1.0f * dt * 10;
	if (f.keys['R']) eye += up * dt * 20; else if (f.keys['F']) eye += up * -1.0f * dt * 20;
	if (f.keys[20]) view = tinybvh_normalize( view + right * -1.0f * dt );
	if (f.keys[19]) view = tinybvh_normalize( view + right * dt );
	if (f.keys[17]) view = tinybvh_normalize( view + up * -1.0f * dt );
	if (f.keys[18]) view = tinybvh_normalize( view + up * dt );
	// save camera position
	static bool sdown = true;
	if (!f.keys['P']) sdown = false; else if (!sdown)
	{
		sdown = true;
		FILE* f = fopen( "cam.txt", "a" );
		fprintf( f, "camPos[0] = bvhvec3( %.2f, %.2f, %.2f ), camDir[0] = tinybvh_normalize( bvhvec3( %.2f, %.2f, %.2f ) );\n",
			eye.x, eye.y, eye.z, view.x, view.y, view.z ); 
		fclose( f );
	}
	// recalculate right, up
	right = tinybvh_normalize( tinybvh_cross( bvhvec3( 0, 1, 0 ), view ) ), up = 0.8f * tinybvh_cross( view, right );
	bvhvec3 C = eye + 1.2f * view;
	p1 = C - right + up, p2 = C + right + up, p3 = C - right - up;
}

void TraceWorkerThread( uint32_t* buf, int threadIdx )
{
	const int xtiles = SCRWIDTH / TILESIZE, ytiles = SCRHEIGHT / TILESIZE;
	const int tiles = xtiles * ytiles;
	int tile = threadIdx;
	while (tile < tiles)
	{
		const int tx = tile % xtiles, ty = tile / xtiles;
		for (int y = 0; y < TILESIZE; y++) for (int x = 0; x < TILESIZE; x++)
		{
			const int pixel_x = tx * TILESIZE + x, pixel_y = ty * TILESIZE + y;
			const int pixelIdx = pixel_x + pixel_y * SCRWIDTH;
			// setup primary ray
			const float u = (float)pixel_x / SCRWIDTH, v = (float)pixel_y / SCRHEIGHT;
			const bvhvec3 D = tinybvh_normalize( p1 + u * (p2 - p1) + v * (p3 - p1) - eye );
			// trace
			Ray ray( eye, D );
			bvh.Intersect( ray );
			bvhvec3 E( 0.6f, 0.7f, 1 );
			if (ray.hit.t < 1e30f)
			{	
				bvhvec3 N = TriangleNormal( ray.hit.prim );
				if (tinybvh_dot( N, ray.D ) > 0) N *= -1.0f;
				E = (N + 1) * 0.5f;
			}
			const int r = (int)tinybvh_min( 255.0f, sqrtf( E.x ) * 255.0f );
			const int g = (int)tinybvh_min( 255.0f, sqrtf( E.y ) * 255.0f );
			const int b = (int)tinybvh_min( 255.0f, sqrtf( E.z ) * 255.0f );
			buf[pixelIdx] = b + (g << 8) + (r << 16);
		}
		tile = tileIdx++;
	}
}

// Application Tick - render and update window contents.
void Tick( float delta_time_s, fenster& f, uint32_t* buf )
{
	UpdateCamera( delta_time_s, f );
	tileIdx = threadCount;
	std::vector<std::thread> threads;
	for (uint32_t i = 0; i < threadCount; i++) threads.emplace_back( &TraceWorkerThread, buf, i );
	for (auto& thread : threads) thread.join();
	char title[256];
	snprintf( title, 256, "cam (%.2f,%.2f,%.2f) => (%.2f,%.2f,%.2f) - press P to save to cam.txt",
		eye.x, eye.y, eye.z, view.x, view.y, view.z );
	fenster_update_title( &f, title );
}

// Application Shutdown - save camera to file for next run.
void Shutdown()
{
	std::fstream s = std::fstream{ "camera.bin", s.binary | s.out };
	s.write( (char*)&eye, sizeof( eye ) );
	s.write( (char*)&view, sizeof( view ) );
}