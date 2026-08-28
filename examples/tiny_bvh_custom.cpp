// Example showing the use of custom primitives in a BVH.
// Ray tracing in this example is CPU-only and single-threaded.

#define FENSTER_APP_IMPLEMENTATION
#define SCRWIDTH 800
#define SCRHEIGHT 600
#include "external/fenster.h" // https://github.com/zserge/fenster

#define TINYBVH_IMPLEMENTATION
#include "tiny_bvh.h"
#include <fstream>

using namespace tinybvh;

struct Sphere { bvhvec3 pos; float r; };

BVH bvh;
bvhvec4* verts = 0;
Sphere* spheres = 0;

// setup view pyramid for a pinhole camera
bvhvec3 eye( -15.24f, 21.5f, 2.54f ), p1, p2, p3;
bvhvec3 view = tinybvh_normalize( bvhvec3( 0.826f, -0.438f, -0.356f ) );

// directional light vector
const bvhvec3 L = tinybvh_normalize( bvhvec3( 1, 2, 3 ) );

// callback for custom geometry: ray/sphere intersection - "find nearest"
bool sphereIntersect( Ray& ray, const uint32_t primID, void* )
{
	bvhvec3 oc = ray.O - spheres[primID].pos;
	float b = tinybvh_dot( oc, ray.D ), r = spheres[primID].r;
	float c = tinybvh_dot( oc, oc ) - r * r, t, d = b * b - c;
	if (d <= 0) return false;
	d = sqrtf( d ), t = -b - d; // skipping +d; take only the near root.
	bool hit = t < ray.hit.t && t > 0;
	if (hit) ray.hit.t = t, ray.hit.prim = primID;
	return hit;
}

// callback for custom geometry: ray/sphere intersection - "any hit"
bool sphereIsOccluded( const Ray& ray, const uint32_t primID, void* )
{
	bvhvec3 oc = ray.O - spheres[primID].pos;
	float b = tinybvh_dot( oc, ray.D ), r = spheres[primID].r;
	float c = tinybvh_dot( oc, oc ) - r * r, t, d = b * b - c;
	if (d <= 0) return false;
	d = sqrtf( d ), t = -b - d; // skipping +d; take only the near root.
	return t < ray.hit.t && t > 0;
}

// callback for custom geometry: sphere AABB
void sphereAABB( const uint32_t primID, bvhvec3& boundsMin, bvhvec3& boundsMax, void* )
{
	boundsMin = spheres[primID].pos - bvhvec3( spheres[primID].r );
	boundsMax = spheres[primID].pos + bvhvec3( spheres[primID].r );
}

void Init()
{
	// load raw vertex data for Crytek's Sponza
	std::fstream s{ "./testdata/cryteksponza.bin", s.binary | s.in };
	uint32_t triCount, vertCount;
	s.read( (char*)&triCount, 4 );
	printf( "Loading triangle data (%u tris).\n", triCount );
	vertCount = 3 * triCount, verts = (bvhvec4*)malloc64( vertCount * 16 );
	s.read( (char*)verts, vertCount * 16 );

	// turn the array of triangles into an array of spheres
	spheres = new Sphere[triCount];
	for (uint32_t i = 0; i < triCount; i++)
	{
		bvhvec3 v0 = verts[i * 3], v1 = verts[i * 3 + 1], v2 = verts[i * 3 + 2];
		spheres[i].r = tinybvh_min( 0.35f, 0.25f * tinybvh_min( tinybvh_length( v1 - v0 ), tinybvh_length( v2 - v0 ) ) );
		spheres[i].pos = (v0 + v1 + v2) / 3.0f;
	}

	// build the BVH over the aabbs
	bvh.Build( &sphereAABB, triCount );

	// set custom intersection callbacks
	bvh.customIntersect = &sphereIntersect;
	bvh.customIsOccluded = &sphereIsOccluded;
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

	// trace rays
	for (uint32_t y = 0; y < SCRHEIGHT; y++) for (uint32_t x = 0; x < SCRWIDTH; x++)
	{
		float u = (float)x / SCRWIDTH, v = (float)y / SCRHEIGHT;
		bvhvec3 D = tinybvh_normalize( p1 + u * (p2 - p1) + v * (p3 - p1) - eye );
		Ray ray( eye, D );
		bvh.Intersect( ray );
		uint32_t pixelColor = 0xaaaaff; // background / sky color
		if (ray.hit.t < BVH_FAR)
		{
			// calculate intersection point and normal
			bvhvec3 I = ray.O + ray.hit.t * ray.D;
			bvhvec3 N = tinybvh_normalize( I - spheres[ray.hit.prim].pos );
			int c = (int)(255.9f * tinybvh_max( 0.f, tinybvh_dot( N, L ) ));
			// cast a shadow ray
			Ray shadowRay( I + N * 0.001f, L );
			if (bvh.IsOccluded( shadowRay )) c /= 4;
			pixelColor = c + (c << 8) + (c << 16);
		}
		buf[x + y * SCRWIDTH] = pixelColor;
	}
}

void Shutdown()
{
	delete[] spheres;
	free64( verts );
}