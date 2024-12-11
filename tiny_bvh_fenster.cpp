#define FENSTER_APP_IMPLEMENTATION
#define SCRWIDTH 800
#define SCRHEIGHT 600
#include "external/fenster.h" // https://github.com/zserge/fenster

#define LOADSCENE

#define TINYBVH_IMPLEMENTATION
#include "tiny_bvh.h"
#include <fstream>

using namespace tinybvh;

BVH bvh;
BVH_SoA bvh2;

#ifdef LOADSCENE
bvhvec4* triangles = 0;
const char scene[] = "cryteksponza.bin";
#else
ALIGNED( 16 ) bvhvec4 triangles[259 /* level 3 */ * 6 * 2 * 49 * 3]{};
#endif
int verts = 0;

// setup view pyramid for a pinhole camera: 
// eye, p1 (top-left), p2 (top-right) and p3 (bottom-left)
#ifdef LOADSCENE
static bvhvec3 eye( 0, 30, 0 ), p1, p2, p3;
static bvhvec3 view = normalize( bvhvec3( -8, 2, -1.7f ) );
#else
static bvhvec3 eye( -3.5f, -1.5f, -6.5f ), p1, p2, p3;
static bvhvec3 view = normalize( bvhvec3( 3, 1.5f, 5 ) );
#endif

void sphere_flake( float x, float y, float z, float s, int d = 0 )
{
	// procedural tesselated sphere flake object
#define P(F,a,b,c) p[i+F*64]={(float)a ,(float)b,(float)c}
	bvhvec3 p[384], pos( x, y, z ), ofs( 3.5 );
	for (int i = 0, u = 0; u < 8; u++) for (int v = 0; v < 8; v++, i++)
		P( 0, u, v, 0 ), P( 1, u, 0, v ), P( 2, 0, u, v ),
		P( 3, u, v, 7 ), P( 4, u, 7, v ), P( 5, 7, u, v );
	for (int i = 0; i < 384; i++) p[i] = normalize( p[i] - ofs ) * s + pos;
	for (int i = 0, side = 0; side < 6; side++, i += 8)
		for (int u = 0; u < 7; u++, i++) for (int v = 0; v < 7; v++, i++)
			triangles[verts++] = p[i], triangles[verts++] = p[i + 8],
			triangles[verts++] = p[i + 1], triangles[verts++] = p[i + 1],
			triangles[verts++] = p[i + 9], triangles[verts++] = p[i + 8];
	if (d < 3) sphere_flake( x + s * 1.55f, y, z, s * 0.5f, d + 1 );
	if (d < 3) sphere_flake( x - s * 1.5f, y, z, s * 0.5f, d + 1 );
	if (d < 3) sphere_flake( x, y + s * 1.5f, z, s * 0.5f, d + 1 );
	if (d < 3) sphere_flake( x, x - s * 1.5f, z, s * 0.5f, d + 1 );
	if (d < 3) sphere_flake( x, y, z + s * 1.5f, s * 0.5f, d + 1 );
	if (d < 3) sphere_flake( x, y, z - s * 1.5f, s * 0.5f, d + 1 );
}

void Init()
{
#ifdef LOADSCENE
	// load raw vertex data for Crytek's Sponza
	std::string filename{ "./testdata/" };
	filename += scene;
	std::fstream s{ filename, s.binary | s.in };
	s.seekp( 0 );
	s.read( (char*)&verts, 4 );
	printf( "Loading triangle data (%i tris).\n", verts );
	verts *= 3, triangles = (bvhvec4*)malloc64( verts * 16 );
	s.read( (char*)triangles, verts * 16 );
	s.close();
#else
	// generate a sphere flake scene
	sphere_flake( 0, 0, 0, 1.5f );
#endif

	// build a BVH over the scene
#if defined(BVH_USEAVX)
	bvh.BuildAVX( triangles, verts / 3 );
#elif defined(BVH_USENEON)
	bvh.BuildNEON( triangles, verts / 3 );
#else
	bvh.Build( triangles, verts / 3 );
#endif
	bvh2.ConvertFrom( bvh );

	// load camera position / direction from file
	std::fstream t = std::fstream{ "camera.bin", t.binary | t.in };
	if (!t.is_open()) return;
	t.seekp( 0 );
	t.read( (char*)&eye, sizeof( eye ) );
	t.read( (char*)&view, sizeof( view ) );
	t.close();
}

void UpdateCamera(float delta_time_s, fenster& f)
{
	bvhvec3 right = normalize( cross( bvhvec3( 0, 1, 0 ), view ) );
	bvhvec3 up = 0.8f * cross( view, right );

	// get camera controls.
	if (f.keys['A']) eye += right * -1.0f * delta_time_s * 10;
	if (f.keys['D']) eye += right * delta_time_s * 10;
	if (f.keys['W']) eye += view * delta_time_s * 10;
	if (f.keys['S']) eye += view * -1.0f * delta_time_s * 10;
	if (f.keys['R']) eye += up * delta_time_s * 10;
	if (f.keys['F']) eye += up * -1.0f * delta_time_s * 10;

	// recalculate right, up
	right = normalize( cross( bvhvec3( 0, 1, 0 ), view ) );
	up = 0.8f * cross( view, right );
	bvhvec3 C = eye + 2 * view;
	p1 = C - right + up, p2 = C + right + up, p3 = C - right - up;
}

void Tick(float delta_time_s, fenster & f, uint32_t* buf)
{
	// handle user input and update camera
	UpdateCamera(delta_time_s, f);

	// clear the screen with a debug-friendly color
	for (int i = 0; i < SCRWIDTH * SCRHEIGHT; i++) buf[i] = 0xff00ff;

	// generate primary rays in a cacheline-aligned buffer - and, for data locality:
	// organized in 4x4 pixel tiles, 16 samples per pixel, so 256 rays per tile.
	int N = 0;
	Ray* rays = (Ray*)tinybvh::malloc64( SCRWIDTH * SCRHEIGHT * 16 * sizeof( Ray ) );
	int * depths = (int *)tinybvh::malloc64(SCRWIDTH * SCRHEIGHT * sizeof (int));
	for (int ty = 0; ty < SCRHEIGHT; ty += 4) for (int tx = 0; tx < SCRWIDTH; tx += 4 )
	{
		for (int y = 0; y < 4; y++) for (int x = 0; x < 4; x++)
		{
			float u = (float)(tx + x) / SCRWIDTH, v = (float)(ty + y) / SCRHEIGHT;
			bvhvec3 D = normalize( p1 + u * (p2 - p1) + v * (p3 - p1) - eye );
			rays[N++] = Ray( eye, D, 1e30f );
		}
	}

	// trace primary rays
	for (int i = 0; i < N; i++) depths[i] = bvh2.Intersect( rays[i] );

	// visualize result
	const bvhvec3 L = normalize( bvhvec3( 1, 2, 3 ) );
	for (int i = 0, ty = 0; ty < SCRHEIGHT / 4; ty++) for (int tx = 0; tx < SCRWIDTH / 4; tx++)
	{
		for (int y = 0; y < 4; y++) for (int x = 0; x < 4; x++, i++) if (rays[i].hit.t < 10000)
		{
			int pixel_x = tx * 4 + x, pixel_y = ty * 4 + y, primIdx = rays[i].hit.prim;
			bvhvec3 v0 = triangles[primIdx * 3 + 0];
			bvhvec3 v1 = triangles[primIdx * 3 + 1];
			bvhvec3 v2 = triangles[primIdx * 3 + 2];
			bvhvec3 N = normalize( cross( v1 - v0, v2 - v0 ) );
			int c = (int)(255.9f * fabs( dot( N, L ) ));
			buf[pixel_x + pixel_y * SCRWIDTH] = c + (c << 8) + (c << 16);
			// buf[pixel_x + pixel_y * SCRWIDTH] = (primIdx * 0xdeece66d + 0xb) & 0xFFFFFF; // color is hashed primitive index
			// buf[pixel_x + pixel_y * SCRWIDTH] = depths[i] << 17; // render depth as red
		}
	}
	tinybvh::free64( rays );
}

void Shutdown()
{
	// save camera position / direction to file
	std::fstream s = std::fstream{ "camera.bin", s.binary | s.out };
	s.seekp( 0 );
	s.write( (char*)&eye, sizeof( eye ) );
	s.write( (char*)&view, sizeof( view ) );
	s.close();
}