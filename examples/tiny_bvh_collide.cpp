// Example combining three concepts:
// 1. Custom geometry: A sphere is intersected using custom callbacks;
// 2. A TLAS: Sponza and the sphere are each in their own BVH;
// 3. Sphere collision detection using BVH::IntersectSphere.
// Ray tracing in this example happens on the (multi-core) CPU.

#define FENSTER_APP_IMPLEMENTATION
#define SCRWIDTH 800
#define SCRHEIGHT 600
#include "external/fenster.h" // https://github.com/zserge/fenster

#define TINYBVH_IMPLEMENTATION
#define INST_IDX_BITS 8 // override default; space for 256 instances.
#include "tiny_bvh.h"
#include <fstream>
#include <thread>

using namespace tinybvh;

struct Sphere { bvhvec3 pos; float r; };

// setup scene for TLAS traversal
BVH sponza, obj, tlas;
BVHBase* bvhList[] = { &sponza, &obj };
BLASInstance inst[2];

// application variables
bvhvec4* verts = 0;
Sphere* spheres = 0;

// threading variables
std::atomic<uint32_t> tileIdx( 0 );
uint32_t threadCount = tinybvh_max( 1u, std::thread::hardware_concurrency() );

// setup view pyramid for a pinhole camera
bvhvec3 eye( -15.24f, 21.5f, 2.54f ), p1, p2, p3;
bvhvec3 view = tinybvh_normalize( bvhvec3( 0.83f, -0.44f, -0.36f ) );

// directional light direction
const bvhvec3 L = tinybvh_normalize( bvhvec3( 1, 2, 3 ) );

// callback for custom geometry: ray/sphere intersection - "find nearest"
bool sphereIntersect( tinybvh::Ray& ray, const uint32_t primID, void* )
{
	bvhvec3 oc = ray.O - spheres[primID].pos;
	float b = tinybvh_dot( oc, ray.D ), r = spheres[primID].r;
	float c = tinybvh_dot( oc, oc ) - r * r, t, d = b * b - c;
	if (d <= 0) return false;
	d = sqrtf( d ), t = -b - d; // skipping +d; take only the near root.
	bool hit = t < ray.hit.t && t > 0;
	// note: we store the bare primitive index; BVH::Intersect adds the
	// instance index to hit.prim once we return true.
	if (hit) ray.hit.t = t, ray.hit.prim = primID;
	return hit;
}

// callback for custom geometry: ray/sphere intersection - "any hit"
bool sphereIsOccluded( const tinybvh::Ray& ray, const uint32_t primID, void* )
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
	sponza.Build( verts, triCount );

	// create a blas for a single sphere
	spheres = new Sphere[1];
	spheres[0].pos = bvhvec3( 0 ), spheres[0].r = 2;
	obj.Build( &sphereAABB, 1 );
	obj.customIntersect = &sphereIntersect;
	obj.customIsOccluded = &sphereIsOccluded;

	// create instance list
	inst[0] = BLASInstance( 0 /* sponza */ );
	inst[1] = BLASInstance( 1 /* sphere */ );

	// actual TLAS construction happens each frame in Tick.
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
	bvhvec3 C = eye + 1.2f * view;
	p1 = C - right + up, p2 = C + right + up, p3 = C - right - up;
}

// Worker thread renders tiles until no unclaimed tiles are left.
void TraceWorkerThread( uint32_t* buf, uint32_t threadIdx )
{
	constexpr uint32_t TILESIZE = 20;
	constexpr uint32_t xtiles = SCRWIDTH / TILESIZE, ytiles = SCRHEIGHT / TILESIZE;
	constexpr uint32_t tiles = xtiles * ytiles;
	for( uint32_t tile = threadIdx; tile < tiles; tile = tileIdx++ )
	{
		const uint32_t tx = tile % xtiles, ty = tile / xtiles;
		for (uint32_t y = 0; y < TILESIZE; y++) for (uint32_t x = 0; x < TILESIZE; x++)
		{
			const uint32_t pixel_x = tx * TILESIZE + x, pixel_y = ty * TILESIZE + y;
			const uint32_t pixelIdx = pixel_x + pixel_y * SCRWIDTH;
			// setup primary ray
			const float u = (float)pixel_x / SCRWIDTH, v = (float)pixel_y / SCRHEIGHT;
			const bvhvec3 D = tinybvh_normalize( p1 + u * (p2 - p1) + v * (p3 - p1) - eye );
			// trace
			Ray ray( eye, D );
			tlas.Intersect( ray );
			if (ray.hit.t < BVH_FAR)
			{
				// instance and primitive index are stored together for compactness
				uint32_t primIdx = ray.hit.prim & PRIM_IDX_MASK;
				uint32_t instIdx = (uint32_t)ray.hit.prim >> INST_IDX_SHFT;
				BLASInstance& instance = inst[instIdx];
				uint32_t blasIdx = instance.blasIdx;
				bvhvec3 N;
				if (blasIdx == 0)
				{
					// we hit the Sponza mesh, which consists of triangles
					bvhvec3 v0 = verts[primIdx * 3];
					bvhvec3 v1 = verts[primIdx * 3 + 1];
					bvhvec3 v2 = verts[primIdx * 3 + 2];
					N = tinybvh_normalize( tinybvh_cross( v1 - v0, v2 - v0 ) );
					// the next line is disabled because we know Sponza is used with an identity transform.
					// N = tinybvh_transform_vector( N, instance.transform );
				}
				else
				{
					// we hit a sphere
					bvhvec3 C = tinybvh_transform_point( spheres[primIdx].pos, instance.transform );
					bvhvec3 I = ray.O + ray.hit.t * ray.D;
					N = tinybvh_normalize( I - C );
				}
				int c = (int)(255.9f * fabs( tinybvh_dot( N, L ) ));
				buf[pixelIdx] = c + (c << 8) + (c << 16);
			}
		}
	}
}

void Tick( float delta_time_s, fenster& f, uint32_t* buf )
{
	// handle user input and update camera
	UpdateCamera( delta_time_s, f );

	// clear the screen with a debug-friendly color
	for (uint32_t i = 0; i < SCRWIDTH * SCRHEIGHT; i++) buf[i] = 0xaaaaff;

	// position the sphere
	static float bally = 20, ballv = 0;
	ballv -= 0.05f;
	bally += ballv; // note: for this demo, physics ignore delta_time_s.
	constexpr uint32_t TY = 7; // fields 3, 7, 11 in a 4x4 matrix hold the translation.
	inst[1].transform[TY] = bally;
	if (sponza.IntersectSphere( bvhvec3( 0, bally, 0 ), spheres[0].r )) ballv = -ballv;

	// build the tlas
	tlas.Build( inst, 2, bvhList, 2 );

	// render tiles
	tileIdx = threadCount;
	std::vector<std::thread> threads;
	for (uint32_t i = 0; i < threadCount; i++)
		threads.emplace_back( &TraceWorkerThread, buf, i );
	for (auto& thread : threads) thread.join();
}

void Shutdown() 
{
	delete [] spheres;
	free64( verts );
}