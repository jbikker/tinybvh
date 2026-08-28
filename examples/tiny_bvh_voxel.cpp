// Example of voxel traversal using TinyBVH.

#define FENSTER_APP_IMPLEMENTATION
#define SCRWIDTH 800
#define SCRHEIGHT 600
#include "external/fenster.h" // https://github.com/zserge/fenster

#define TINYBVH_IMPLEMENTATION
#include "tiny_bvh.h"
#include <zlib.h>

using namespace tinybvh;

VoxelSet voxels;

// setup view pyramid for a pinhole camera:
// eye, p1 (top-left), p2 (top-right) and p3 (bottom-left)
static bvhvec3 eye( 0.15f, 0.354f, -0.105f ), p1, p2, p3;
static bvhvec3 view = tinybvh_normalize( bvhvec3( 0.350f, -0.402f, 0.717f ) );

void Init()
{
	// load voxel object
	gzFile f = gzopen( "./testdata/voxels/legocar.bin", "rb" );
	bvhint3 size, clamped;
	gzread( f, &size, sizeof( bvhint3 ) );
	uint32_t* grid = new uint32_t[size.x * size.y * size.z];
	gzread( f, grid, size.x * size.y * size.z * 4 );
	gzclose( f );
	// store in VoxelSet instance
	clamped.x = tinybvh_min( size.x, VoxelSet::objectDim );
	clamped.y = tinybvh_min( size.y, VoxelSet::objectDim );
	clamped.z = tinybvh_min( size.z, VoxelSet::objectDim );
	for (int z = 0; z < clamped.z; z++) for (int y = 0; y < clamped.y; y++) for (int x = 0; x < clamped.x; x++)
	{
		uint32_t v = grid[x + y * size.x + z * size.x * size.y];
		if (v) voxels.Set( x, y, z, v );
	}
	voxels.UpdateTopGrid();
	delete[] grid;
}

// Keyboard handling: WASD for movement, R,F: up/down, cursors: rotate.
void UpdateCamera( float delta_time_s, fenster& f )
{
	static bvhvec3 right, up;
	float spd = 10.0f * delta_time_s;
	if (f.keys['A'] || f.keys['D']) eye += right * (f.keys['D'] ? spd : -spd);
	if (f.keys['W'] || f.keys['S']) eye += view * (f.keys['W'] ? spd : -spd);
	if (f.keys['R'] || f.keys['F']) eye += up * 2.0f * (f.keys['R'] ? spd : -spd);
	if (f.keys[20]) view = tinybvh_normalize( view + right * -0.1f * spd );
	if (f.keys[19]) view = tinybvh_normalize( view + right * 0.1f * spd );
	if (f.keys[17]) view = tinybvh_normalize( view + up * -0.1f * spd );
	if (f.keys[18]) view = tinybvh_normalize( view + up * 0.1f * spd );
	// recalculate right, up
	right = tinybvh_normalize( tinybvh_cross( bvhvec3( 0, 1, 0 ), view ) ), up = 0.8f * tinybvh_cross( view, right );
	bvhvec3 center = eye + 1.2f * view;
	p1 = center - right + up, p2 = center + right + up, p3 = center - right - up;
}

void Tick( float delta_time_s, fenster& f, uint32_t* buf )
{
	// handle user input and update camera
	UpdateCamera( delta_time_s, f );

	// visualize result
	const bvhvec3 L = tinybvh_normalize( bvhvec3( 1, 2, -3 ) );
	for (int y = 0; y < SCRHEIGHT; y++) for (int x = 0; x < SCRWIDTH; x++)
	{
		float u = (float)x / SCRWIDTH, v = (float)y / SCRHEIGHT;
		bvhvec3 D = tinybvh_normalize( p1 + u * (p2 - p1) + v * (p3 - p1) - eye );
		Ray ray( eye, D );
		voxels.Intersect( ray );
		uint32_t pixelColor = 0xff00ff;
		if (ray.hit.t < BVH_FAR)
		{
			// get voxel normal
			bvhvec3 N = voxels.GetNormal( ray );
			// get voxel color
			uint32_t color = ray.hit.prim;
			float r = (float)((color >> 16) & 255);
			float g = (float)((color >> 8) & 255);
			float b = (float)(color & 255);
			bvhvec3 shaded = bvhvec3( r, g, b ) * (1.0f / 255.0f) * tinybvh_max( 0.f, tinybvh_dot( N, L ) );
			// final plot
			int ir = (int)(shaded.x * 255.0f);
			int ig = (int)(shaded.y * 255.0f);
			int ib = (int)(shaded.z * 255.0f);
			pixelColor = ib + (ig << 8) + (ir << 16);
		}
		buf[x + y * SCRWIDTH] = pixelColor;
	}
	// print frame time / rate in window title
	char title[99];
	snprintf( title, sizeof( title ), "tiny_bvh %.2f s %.2f Hz", delta_time_s, 1.0f / delta_time_s );
	fenster_update_title( &f, title );
}

void Shutdown() { /* nothing here */ }