#define FENSTER_APP_IMPLEMENTATION
#define SCRWIDTH 800
#define SCRHEIGHT 600
#include "external/fenster.h" // https://github.com/zserge/fenster

#define TINYBVH_IMPLEMENTATION
#include "tiny_bvh.h"
#include <fstream>

using namespace tinybvh;

struct Sphere
{
	bvhvec3 pos;
	float r;
};

BVH bvh;
int frameIdx = 0;
bvhvec4* triangles = 0;
Sphere* spheres = 0;
int verts = 0;

// setup view pyramid for a pinhole camera
static bvhvec3 eye( -15.24f, 21.5f, 2.54f ), p1, p2, p3;
static bvhvec3 view = tinybvh_normalize( bvhvec3( 0.826f, -0.438f, -0.356f ) );

// callback for custom geometry: ray/sphere intersection
bool sphereIntersect( tinybvh::Ray& ray, const unsigned primID )
{
	bvhvec3 oc = ray.O - spheres[primID].pos;
	float b = tinybvh_dot( oc, ray.D );
	float r = spheres[primID].r;
	float c = tinybvh_dot( oc, oc ) - r * r;
	float t, d = b * b - c;
	if (d <= 0) return false;
	d = sqrtf( d ), t = -b - d;
	bool hit = t < ray.hit.t && t > 0;
	if (hit) ray.hit.t = t, ray.hit.prim = primID;
	return hit;
}

bool sphereIsOccluded( const tinybvh::Ray& ray, const unsigned primID )
{
	bvhvec3 oc = ray.O - spheres[primID].pos;
	float b = tinybvh_dot( oc, ray.D );
	float r = spheres[primID].r;
	float c = tinybvh_dot( oc, oc ) - r * r;
	float t, d = b * b - c;
	if (d <= 0) return false;
	d = sqrtf( d ), t = -b - d;
	return t < ray.hit.t && t > 0;
}

void sphereAABB( const unsigned primID, bvhvec3& boundsMin, bvhvec3& boundsMax )
{
	boundsMin = spheres[primID].pos - bvhvec3( spheres[primID].r );
	boundsMax = spheres[primID].pos + bvhvec3( spheres[primID].r );
}

void Init()
{
	// load raw vertex data for Crytek's Sponza
	std::fstream s{ "./testdata/cryteksponza.bin", s.binary | s.in };
	s.seekp( 0 );
	s.read( (char*)&verts, 4 );
	printf( "Loading triangle data (%i tris).\n", verts );
	verts *= 3, triangles = (bvhvec4*)malloc64( verts * 16 );
	s.read( (char*)triangles, verts * 16 );
	s.close();

	// turn the array of triangles into an array of spheres
	spheres = new Sphere[verts / 3];
	for (int i = 0; i < verts / 3; i++)
	{
		bvhvec3 v0 = triangles[i * 3], v1 = triangles[i * 3 + 1], v2 = triangles[i * 3 + 2];
		spheres[i].r = tinybvh_min( 0.35f, 0.25f * tinybvh_min( tinybvh_length( v1 - v0 ), tinybvh_length( v2 - v0 ) ) );
		spheres[i].pos = (v0 + v1 + v2) * 0.33333f;
	}

	// build the BVH over the aabbs
	bvh.Build( &sphereAABB, verts / 3 );

	// set custom intersection callbacks
	bvh.customIntersect = &sphereIntersect;
	bvh.customIsOccluded = &sphereIsOccluded;
}

bool UpdateCamera( float delta_time_s, fenster& f )
{
	bvhvec3 right = tinybvh_normalize( tinybvh_cross( bvhvec3( 0, 1, 0 ), view ) );
	bvhvec3 up = 0.8f * tinybvh_cross( view, right );

	// get camera controls.
	bool moved = false;
	if (f.keys['A']) eye += right * -1.0f * delta_time_s * 10, moved = true;
	if (f.keys['D']) eye += right * delta_time_s * 10, moved = true;
	if (f.keys['W']) eye += view * delta_time_s * 10, moved = true;
	if (f.keys['S']) eye += view * -1.0f * delta_time_s * 10, moved = true;
	if (f.keys['R']) eye += up * delta_time_s * 10, moved = true;
	if (f.keys['F']) eye += up * -1.0f * delta_time_s * 10, moved = true;
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

	// clear the screen with a debug-friendly color
	for (int i = 0; i < SCRWIDTH * SCRHEIGHT; i++) buf[i] = 0xaaaaff;

	// trace rays
	const bvhvec3 L = tinybvh_normalize( bvhvec3( 1, 2, 3 ) );
	for (int ty = 0; ty < SCRHEIGHT / 4; ty++) for (int tx = 0; tx < SCRWIDTH / 4; tx++)
	{
		for (int y = 0; y < 4; y++) for (int x = 0; x < 4; x++)
		{
			float u = (float)(tx * 4 + x) / SCRWIDTH, v = (float)(ty * 4 + y) / SCRHEIGHT;
			bvhvec3 D = tinybvh_normalize( p1 + u * (p2 - p1) + v * (p3 - p1) - eye );
			Ray ray( eye, D, 1e30f );
			bvh.Intersect( ray );
			if (ray.hit.t < 10000)
			{
				int pixel_x = tx * 4 + x, pixel_y = ty * 4 + y, primIdx = ray.hit.prim;
				bvhvec3 I = ray.O + ray.hit.t * ray.D;
				bvhvec3 N = tinybvh_normalize( I - spheres[primIdx].pos );
				int c = (int)(255.9f * fabs( tinybvh_dot( N, L ) ));
				buf[pixel_x + pixel_y * SCRWIDTH] = c + (c << 8) + (c << 16);
			}
		}
	}
}

void Shutdown() { /* nothing here. */ }