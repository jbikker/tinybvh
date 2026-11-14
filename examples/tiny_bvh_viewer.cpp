// Minimalistic object viewer application:
// Renders using CPU ray tracing.
// Uses 'Fenster' for cross-platform windowed graphics.
// Any of the BVH layouts with an Intersect method can be used.

#define FENSTER_APP_IMPLEMENTATION
#define SCRWIDTH 1600
#define SCRHEIGHT 900
#include "external/fenster.h" // https://github.com/zserge/fenster

#define TINYBVH_IMPLEMENTATION
#include "tiny_bvh.h"
#include <atomic>
#include <fstream>
#include <thread>
#include <vector>

using namespace tinybvh;

// Application variables

BVH bvh;								// BVH layout to use. Can be: BVH, BVH4_CPU, BVH8_CPU, BVH8_CWBVH.
bvhvec4* verts = 0;						// Triangle data. Three bvhvec4 vertices per triangle.
int triCount = 0;						// Triangle count.
std::atomic<int> tileIdx( 0 );			// Atomic counter for tile-based threaded rendering.

// View pyramid for a pinhole camera
bvhvec3 eye( 0, 30, 0 ), p1, p2, p3;	// View pyramid; p1 is top-left, p2 is top-right, p3 is bottom-left.
bvhvec3 view( -1, 0, 0 );				// Initial view direction.

// Scene management - Append a file, with optional position, scale and color override, 'tinyfied'
void AddMesh( const char* file, float scale = 1, bvhvec3 pos = {}, int c = 0, int N = 0 )
{
	std::fstream s{ file, s.binary | s.in };
	s.read( (char*)&N, 4 );
	size_t bytes = ((size_t)N + (size_t)triCount) * 48;
	bvhvec4* data = (bvhvec4*)malloc64( bytes );
	if (verts) memcpy( data, verts, (size_t)triCount * 48 ), free64( verts );
	verts = data, s.read( (char*)verts + (size_t)triCount * 48, N * 48 ), triCount += N;
	for (int* b = (int*)verts + (triCount - N) * 12, i = 0; i < N * 3; i++)
		*(bvhvec3*)b = *(bvhvec3*)b * scale + pos, b[3] = c ? c : b[3], b += 4;
}

// Geometry access
bvhvec3 rgb32_to_vec3( const unsigned c ) { return bvhvec3( (float)((c >> 16) & 255), (float)((c >> 8) & 255), (float)(c & 255) ) * (1 / 255.f); }
bvhvec3 TriangleColor( const unsigned idx ) { return rgb32_to_vec3( *(unsigned*)&verts[idx * 3].w ); }
bvhvec3 TriangleNormal( const unsigned idx )
{
	bvhvec3 a = verts[idx * 3], b = verts[idx * 3 + 1], c = verts[idx * 3 + 2];
	return tinybvh_normalize( tinybvh_cross( b - a, a - c ) );
}

// Application initialization
void Init()
{
	// have a console window
	CONSOLE_SCREEN_BUFFER_INFO coninfo;
	AllocConsole();
	GetConsoleScreenBufferInfo( GetStdHandle( STD_OUTPUT_HANDLE ), &coninfo );
	coninfo.dwSize.X = 1280;
	coninfo.dwSize.Y = 800;
	SetConsoleScreenBufferSize( GetStdHandle( STD_OUTPUT_HANDLE ), coninfo.dwSize );
	FILE* file = nullptr;
	freopen_s( &file, "CON", "w", stdout );
	freopen_s( &file, "CON", "w", stderr );
	SetWindowPos( GetConsoleWindow(), HWND_BOTTOM, 0, 0, 800, 600, 0 );
	// prepare scene
	AddMesh( "./testdata/cryteksponza.bin" );
	printf( "triangles: %i\n", triCount );
	bvh.Build( verts, triCount );
	bvh.Save( "./testdata/bistro_opt.bvh" );
	// load serialized camera position / direction from file
	std::fstream s = std::fstream{ "camera.bin", s.binary | s.in };
	if (!s.is_open()) return;
	s.read( (char*)&eye, sizeof( eye ) );
	s.read( (char*)&view, sizeof( view ) );
}

// Keyboard handling: WASD for movement, R,F: up/down, cursors: rotate.
void UpdateCamera( float dt, fenster& f )
{
	// get camera controls.
	static bvhvec3 right, up;
	if (f.keys['A']) eye += right * -1.0f * dt * 10; else if (f.keys['D']) eye += right * dt * 10;
	if (f.keys['W']) eye += view * dt * 10; else if (f.keys['S']) eye += view * -1.0f * dt * 10;
	if (f.keys['R']) eye += up * dt * 20; else if (f.keys['F']) eye += up * -1.0f * dt * 20;
	if (f.keys[20]) view = tinybvh_normalize( view + right * -1.0f * dt );
	if (f.keys[19]) view = tinybvh_normalize( view + right * dt );
	if (f.keys[17]) view = tinybvh_normalize( view + up * -1.0f * dt );
	if (f.keys[18]) view = tinybvh_normalize( view + up * dt );
	// recalculate right, up
	right = tinybvh_normalize( tinybvh_cross( bvhvec3( 0, 1, 0 ), view ) ), up = 0.8f * tinybvh_cross( view, right );
	bvhvec3 center = eye + 1.2f * view;
	constexpr static float aspect = (float)SCRWIDTH / (float)SCRHEIGHT;
	p1 = center - aspect * right + up, p2 = center + aspect * right + up, p3 = center - aspect * right - up;
}

bvhvec3 heat( float t )
{
    float r = 0.1357f + t * (4.5974f + t * (-42.3277f + t * (130.5887f + t * (-150.5666f + t * 58.1375f))));
    float g = 0.0914f + t * (2.1856f + t * (4.8052f + t * (-14.0195f + t * (4.2109f + t * 2.7747f))));
    float b = 0.1067f + t * (12.5925f + t * (-60.1097f + t * (109.0745f + t * (-88.5066f + t * 26.8183f))));
    r = r < 0 ? 0 : (r > 1 ? 1 : r); g = g < 0 ? 0 : (g > 1 ? 1 : g); b = b < 0 ? 0 : (b > 1 ? 1 : b);
	return bvhvec3( r, g, b );
}

// Worker thread renders tiles until no unclaimed tiles are left.
void TraceWorkerThread( uint32_t* buf, int threadIdx )
{
	static constexpr uint32_t TILESIZE = 20;
	const int xtiles = SCRWIDTH / TILESIZE, ytiles = SCRHEIGHT / TILESIZE;
	for (int tiles = xtiles * ytiles, tile = threadIdx; tile < tiles; tile = tileIdx++)
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
		#if 1
			// heatmap viewer
			static float rms = 1.0f / 256;
			bvhvec3 E = heat( tinybvh_min( 1.0f, (float)bvh.Intersect( ray ) * rms ) );
			if (ray.hit.t == BVH_FAR) E = bvhvec3( 1 );
		#else
			// regular viewer
			bvhvec3 E( 0.6f, 0.7f, 1 ); // sky color
			if (ray.hit.t < BVH_FAR)
			{
				bvhvec3 N = TriangleNormal( ray.hit.prim & PRIM_IDX_MASK );
				if (tinybvh_dot( N, ray.D ) > 0) N *= -1.0f;
				static bvhvec3 L = tinybvh_normalize( bvhvec3( 1, 1, 0 ) );
				float light = 1; // tinybvh_clamp( tinybvh_dot( N, L ), 0.5f, 0.999f );
				// E = TriangleColor( ray.hit.prim & PRIM_IDX_MASK ) * light;
				E = (N + 1) * 0.5f; // encode normal as rgb color
			}
		#endif
			float r = 255.0f * E.x, g = 255.0f * E.y, b = 255.0f * E.z;
			// store color in Fenster pixel buffer
			buf[pixelIdx] = (int)b + ((int)g << 8) + ((int)r << 16);
		}
	}
}

// Application Tick - render and update window contents.
void Tick( float delta_time_s, fenster& f, uint32_t* buf )
{
	UpdateCamera( delta_time_s, f );
	const uint32_t cores = tinybvh_max( 1u, std::thread::hardware_concurrency() );
	tileIdx = cores;
	std::vector<std::thread> threads;
	for (uint32_t i = 0; i < cores; i++) threads.emplace_back( &TraceWorkerThread, buf, i );
	for (auto& thread : threads) thread.join();
	// use Fenster window title to display camera parameters and fps
	char title[256];
	snprintf( title, sizeof( title ), "cam (%.2f,%.2f,%.2f) => (%.2f,%.2f,%.2f)",
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