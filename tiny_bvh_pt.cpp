// This example shows how to build a basic CPU path tracer using
// tinybvh. Function Tick uses OpenMP to render tiles of pixels
// in parallel. For each pixel, function Trace recursively evaluates 
// light. The scene is here a single array of triangles, which
// function AddMesh (repeatedly) adds to.

#define FENSTER_APP_IMPLEMENTATION
#define SCRWIDTH 800
#define SCRHEIGHT 600
#define TILESIZE 8
#include "external/fenster.h" // https://github.com/zserge/fenster

#define TINYBVH_IMPLEMENTATION
#include "tiny_bvh.h"
#include <fstream>

using namespace tinybvh;

// Application variables

#if defined BVH_USEAVX || defined BVH_USENEON
static BVH bvh_build;
static BVH4 bvh4;
static BVH4_Afra bvh_trace; // this is the one we will use for tracing
#else
static BVH bvh_build, bvh_trace;
#endif
static bvhvec4* tris = 0;
static int triCount = 0, frameIdx = 0, spp = 0;
static bvhvec3 accumulator[SCRWIDTH * SCRHEIGHT];

// Setup view pyramid for a pinhole camera: 
// eye, p1 (top-left), p2 (top-right) and p3 (bottom-left)
static bvhvec3 eye( 0, 30, 0 ), p1, p2, p3;
static bvhvec3 view = normalize( bvhvec3( -1, 0, 0 ) );

// Xor32 RNG
static unsigned RandomUInt( unsigned& seed ) { seed ^= seed << 13, seed ^= seed >> 17, seed ^= seed << 5; return seed; }
static float RandomFloat( unsigned& seed ) { return RandomUInt( seed ) * 2.3283064365387e-10f; }

// Ray tracing math
bvhvec3 DiffuseReflection( const bvhvec3 N, unsigned& seed )
{
	bvhvec3 R;
	do
	{
		R = bvhvec3( RandomFloat( seed ) * 2 - 1, RandomFloat( seed ) * 2 - 1, RandomFloat( seed ) * 2 - 1 );
	} while (dot( R, R ) > 1);
	return normalize( dot( R, N ) < 0 ? R : -R );
}
bvhvec3 CosWeightedDiffReflection( const bvhvec3 N, unsigned& seed )
{
	bvhvec3 R = DiffuseReflection( N, seed );
	return normalize( N + R );
}

// Color conversion
bvhvec3 rgb32_to_vec3( const unsigned c )
{
	return bvhvec3( (float)(c >> 16), (float)((c >> 8) & 255), (float)(c & 255) ) * (1 / 255.f);
}

// Geometry access
bvhvec3 TriangleColor( const unsigned idx ) { return rgb32_to_vec3( *(unsigned*)&tris[idx * 3].w ); }
bvhvec3 TriangleNormal( const unsigned idx )
{
	bvhvec3 a = tris[idx * 3], b = tris[idx * 3 + 1], c = tris[idx * 3 + 2];
	return normalize( cross( b - a, a - c ) );
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
	// load raw vertex data
	AddMesh( "./testdata/cryteksponza.bin", 1, bvhvec3( 0 ), 0xffffff );
	AddMesh( "./testdata/dragon.bin", 1.1f, bvhvec3( 29, 3.01f, 0 ), 0xffbb88 );
	AddMesh( "./testdata/lucy.bin", 1.1f, bvhvec3( -2, 4.1f, -3 ), 0xaaaaff );
	AddMesh( "./testdata/bunny.bin", 0.2f, bvhvec3( -7, 0.13f, 0 ), 0x333333 );
	AddMesh( "./testdata/legocar.bin", 0.3f, bvhvec3( -12, 0.8f, -5 ) );
	AddMesh( "./testdata/armadillo.bin", 0.3f, bvhvec3( 7, 1, 3 ), 0xff2020 );
	AddMesh( "./testdata/xyzrgb_dragon.bin", 0.5f, bvhvec3( -22, 0.95f, 0 ), 0xffffaa );
	AddMesh( "./testdata/suzanne.bin", 0.2f, bvhvec3( -18, 0.95f, -16 ), 0x90ff90 );
	AddMesh( "./testdata/head.bin", 0.5f, bvhvec3( 0, 3, 9 ) );
	// build bvh
	bvh_build.BuildAVX( tris, triCount );
#if defined BVH_USEAVX || defined BVH_USENEON
	bvh4.ConvertFrom( bvh_build );
	bvh_trace.ConvertFrom( bvh4 );
#else
	bvh_trace = bvh;
#endif
	// load camera position / direction from file
	std::fstream t = std::fstream{ "camera.bin", t.binary | t.in };
	if (!t.is_open()) return;
	t.read( (char*)&eye, sizeof( eye ) );
	t.read( (char*)&view, sizeof( view ) );
	t.close();
}

// Keyboard handling
bool UpdateCamera( float delta_time_s, fenster& f )
{
	bvhvec3 right = normalize( cross( bvhvec3( 0, 1, 0 ), view ) ), up = 0.8f * cross( view, right );
	// get camera controls.
	bool moved = false;
	if (f.keys['A']) eye += right * -1.0f * delta_time_s * 10, moved = true;
	if (f.keys['D']) eye += right * delta_time_s * 10, moved = true;
	if (f.keys['W']) eye += view * delta_time_s * 10, moved = true;
	if (f.keys['S']) eye += view * -1.0f * delta_time_s * 10, moved = true;
	if (f.keys['R']) eye += up * delta_time_s * 20, moved = true;
	if (f.keys['F']) eye += up * -1.0f * delta_time_s * 20, moved = true;
	if (f.keys[20]) view = normalize( view + right * -1.0f * delta_time_s ), moved = true;
	if (f.keys[19]) view = normalize( view + right * delta_time_s ), moved = true;
	if (f.keys[17]) view = normalize( view + up * -1.0f * delta_time_s ), moved = true;
	if (f.keys[18]) view = normalize( view + up * delta_time_s ), moved = true;
	// recalculate right, up
	right = normalize( cross( bvhvec3( 0, 1, 0 ), view ) ), up = 0.8f * cross( view, right );
	bvhvec3 C = eye + 1.2f * view;
	p1 = C - right + up, p2 = C + right + up, p3 = C - right - up;
	return moved;
}

// Light transport calculation - Basic recursive Path Tracer with IS and Next Event Estimation
bvhvec3 Trace( Ray ray, unsigned& seed, unsigned depth = 0 )
{
	// find primary intersection
	bvh_trace.Intersect( ray );
	// shade
	if (ray.hit.t == 1e30f) return bvhvec3( 0.6f, 0.7f, 1 ); // hit nothing
	bvhvec3 I = ray.O + ray.hit.t * ray.D;
	bvhvec3 N = TriangleNormal( ray.hit.prim );
	if (dot( N, ray.D ) > 0) N = -N;
	bvhvec3 BRDF = TriangleColor( ray.hit.prim ) * (1.0f / 3.14159f);
	bvhvec3 Lpos( RandomFloat( seed ) * 30 - 15, 40, RandomFloat( seed ) * 6 - 3 ); // virtual
	float dist = length( Lpos - I );
	bvhvec3 L = (Lpos - I) * (1.0f / dist); // normalize
	bvhvec3 direct = {}, indirect = {};
	float NdotL = dot( N, L ), NLdotL = fabs( dot( L, bvhvec3( 0, 1, 0 ) ) );
	if (NdotL > 0)
		if (!bvh_trace.IsOccluded( Ray( I + L * 0.001f, L, dist ) ) )
			direct = BRDF * NdotL * NLdotL * bvhvec3( 9, 9, 8 ) * 500 * (1.0f / (dist * dist));
	// random bounce
	if (depth < 4)
	{
		bvhvec3 R = CosWeightedDiffReflection( N, seed );
		float pdf = 1.0f / dot( N, R );
		bvhvec3 irradiance = Trace( Ray( I + R * 0.001f, R ), seed, depth + 1 );
		indirect = BRDF * irradiance * (1.0f / pdf);
	}
	// finalize
	return direct + indirect;
}

// Application Tick
void Tick( float delta_time_s, fenster& f, uint32_t* buf )
{
	// handle user input and update camera
	if (UpdateCamera( delta_time_s, f ) || frameIdx++ == 0 )
	{
		memset( accumulator, 0, SCRWIDTH * SCRHEIGHT * sizeof( bvhvec3 ) );
		spp = 1;
	}

	// render tiles
	const int xtiles = SCRWIDTH / TILESIZE, ytiles = SCRHEIGHT / TILESIZE;
	const int tiles = xtiles * ytiles;
	const float scale = 1.0f / spp++;
#pragma omp parallel for schedule(dynamic)
	for (int tile = 0; tile < tiles; tile++)
	{
		const int tx = tile % xtiles, ty = tile / xtiles;
		unsigned seed = (tile + 17) * 171717 + frameIdx * 1023;
		for (int y = 0; y < TILESIZE; y++) for (int x = 0; x < TILESIZE; x++)
		{
			const int pixel_x = tx * TILESIZE + x, pixel_y = ty * TILESIZE + y;
			const int pixelIdx = pixel_x + pixel_y * SCRWIDTH;
			// setup primary ray
			const float u = (float)pixel_x / SCRWIDTH, v = (float)pixel_y / SCRHEIGHT;
			const bvhvec3 D = normalize( p1 + u * (p2 - p1) + v * (p3 - p1) - eye );
			// trace
			accumulator[pixelIdx] += Trace( Ray( eye, D ), seed );
			const bvhvec3 E = accumulator[pixelIdx] * scale;
			// visualize, with a poor man's gamma correct
			const int r = (int)tinybvh_min( 255.0f, sqrtf( E.x ) * 255.0f );
			const int g = (int)tinybvh_min( 255.0f, sqrtf( E.y ) * 255.0f );
			const int b = (int)tinybvh_min( 255.0f, sqrtf( E.z ) * 255.0f );
			buf[pixelIdx] = b + (g << 8) + (r << 16);
		}
	}
}

// Application Shutdown
void Shutdown()
{
	// save camera position / direction to file
	std::fstream s = std::fstream{ "camera.bin", s.binary | s.out };
	s.write( (char*)&eye, sizeof( eye ) );
	s.write( (char*)&view, sizeof( view ) );
	s.close();
}