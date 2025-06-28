#define FENSTER_APP_IMPLEMENTATION
#define SCRWIDTH 800
#define SCRHEIGHT 600
#include "external/fenster.h" // https://github.com/zserge/fenster

#define TINYBVH_NO_SIMD
#define TINYBVH_IMPLEMENTATION
#include "tiny_bvh.h"
#include <fstream>
#include "zlib.h"

using namespace tinybvh;

VoxelSet voxels;
int frameIdx = 0;
Ray* rays = 0;
#ifdef COLOR_DEPTH
int* depths = 0;
#endif

bvhvec4* vertices = 0;
uint32_t* indices = 0;
const char scene[] = "cryteksponza.bin";
int verts = 0, inds = 0;

// setup view pyramid for a pinhole camera:
// eye, p1 (top-left), p2 (top-right) and p3 (bottom-left)
static bvhvec3 eye( 0, 1.154f, -0.375f ), p1, p2, p3;
static bvhvec3 view( 0.350f, -0.602f, 0.717f );

void Init()
{
#if 0
	// load raw vertex data for Crytek's Sponza
	std::string filename{ "./testdata/" };
	filename += scene;
	std::fstream s{ filename, s.binary | s.in };
	s.seekp( 0 );
	s.read( (char*)&verts, 4 );
	printf( "Loading triangle data (%i tris).\n", verts );
	verts *= 3, vertices = (bvhvec4*)malloc64( verts * 16 );
	s.read( (char*)vertices, verts * 16 );
	s.close();
#endif
	// load voxel object
	uint32_t* grid = new uint32_t[128 * 128 * 128];
	gzFile f = gzopen( "./testdata/voxels/legocar.bin", "rb" );
	bvhint3 size;
	gzread( f, &size, sizeof( bvhint3 ) );
	gzread( f, grid, size.x * size.y * size.z * 4 );
	gzclose( f );
	// store in VoxelSet instance
	for (int x = 0; x < 128; x++) for (int y = 0; y < 128; y++) for (int z = 0; z < 128; z++)
	{
		uint32_t v = grid[x + y * size.x + z * size.x * size.y];
		if (v) voxels.Set( x, y, z, v );
	}
	voxels.UpdateTopGrid();
	int w = 0;
	// allocate buffers
	rays = (Ray*)tinybvh::malloc64( SCRWIDTH * SCRHEIGHT * 16 * sizeof( Ray ) );
}

bool UpdateCamera( float delta_time_s, fenster& f )
{
	bvhvec3 right = tinybvh_normalize( tinybvh_cross( bvhvec3( 0, 1, 0 ), view ) );
	bvhvec3 up = 0.8f * tinybvh_cross( view, right );

	// get camera controls.
	bool moved = false;
	if (f.keys['A']) eye += right * -1.0f * delta_time_s * 2, moved = true;
	if (f.keys['D']) eye += right * delta_time_s * 2, moved = true;
	if (f.keys['W']) eye += view * delta_time_s * 2, moved = true;
	if (f.keys['S']) eye += view * -1.0f * delta_time_s * 2, moved = true;
	if (f.keys['R']) eye += up * delta_time_s * 2, moved = true;
	if (f.keys['F']) eye += up * -1.0f * delta_time_s * 2, moved = true;
	if (f.keys[20]) view = tinybvh_normalize( view + right * -1.0f * delta_time_s ), moved = true;
	if (f.keys[19]) view = tinybvh_normalize( view + right * delta_time_s ), moved = true;
	if (f.keys[17]) view = tinybvh_normalize( view + up * -1.0f * delta_time_s ), moved = true;
	if (f.keys[18]) view = tinybvh_normalize( view + up * delta_time_s ), moved = true;

	// recalculate right, up
	right = tinybvh_normalize( tinybvh_cross( bvhvec3( 0, 1, 0 ), view ) );
	up = 0.8f * tinybvh_cross( view, right );
	bvhvec3 C = eye + 2 * view;
	p1 = C - right + up, p2 = C + right + up, p3 = C - right - up;
	return moved;
}

void Tick( float delta_time_s, fenster& f, uint32_t* buf )
{
	// handle user input and update camera
	bool moved = UpdateCamera( delta_time_s, f ) || frameIdx++ == 0;

	// handle user input and update camera
	UpdateCamera( delta_time_s, f );
	int mx = tinybvh_clamp( f.x, 0, SCRWIDTH - 1 );
	int my = tinybvh_clamp( f.y, 0, SCRHEIGHT - 1 );

	// clear the screen with a debug-friendly color
	for (int i = 0; i < SCRWIDTH * SCRHEIGHT; i++) buf[i] = 0xff00ff;

	// generate primary rays in a cacheline-aligned buffer
	int N = 0;
	for (int ty = 0; ty < SCRHEIGHT; ty += 4) for (int tx = 0; tx < SCRWIDTH; tx += 4)
	{
		for (int y = 0; y < 4; y++) for (int x = 0; x < 4; x++)
		{
			float u = (float)(tx + x) / SCRWIDTH, v = (float)(ty + y) / SCRHEIGHT;
			bvhvec3 D = tinybvh_normalize( p1 + u * (p2 - p1) + v * (p3 - p1) - eye );
			rays[N++] = Ray( eye, D, 1e30f );
		}
	}

	// trace primary rays
	for (int i = 0; i < N; i++) voxels.Intersect( rays[i] );

	// visualize result
	const bvhvec3 L = tinybvh_normalize( bvhvec3( 1, 2, 3 ) );
	for (int i = 0, ty = 0; ty < SCRHEIGHT / 4; ty++) for (int tx = 0; tx < SCRWIDTH / 4; tx++)
	{
		for (int y = 0; y < 4; y++) for (int x = 0; x < 4; x++, i++) if (rays[i].hit.t < 10000)
		{
			int pixel_x = tx * 4 + x, pixel_y = ty * 4 + y, primIdx = rays[i].hit.prim;
			// get voxel normal
			bvhvec3 N = voxels.GetNormal( rays[i] );
			// get voxel color
			uint32_t color = rays[i].hit.prim;
			float r = (float)((color >> 16) & 255);
			float g = (float)((color >> 8) & 255);
			float b = (float)(color & 255);
			bvhvec3 shaded = bvhvec3( r, g, b ) * (1.0f / 255.0f) * fabs( tinybvh_dot( N, L ) );
			// final plot
			int ir = (int)(shaded.x * 255.0f);
			int ig = (int)(shaded.y * 255.0f);
			int ib = (int)(shaded.z * 255.0f);
			buf[pixel_x + pixel_y * SCRWIDTH] = ib + (ig << 8) + (ir << 16);
		}
	}

	// crosshair
	for (int x = 0; x < SCRWIDTH; x += 2) buf[x + my * SCRWIDTH] ^= 0xAAAAAA;
	for (int y = 0; y < SCRHEIGHT; y += 2) buf[mx + y * SCRWIDTH] ^= 0xAAAAAA;

	// print frame time / rate in window title
	char title[50];
	sprintf( title, "tiny_bvh %.2f s %.2f Hz", delta_time_s, 1.0f / delta_time_s );
	fenster_update_title( &f, title );
}

void Shutdown()
{
	// delete allocated buffers
	tinybvh::free64( rays );
}