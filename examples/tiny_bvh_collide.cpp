#define FENSTER_APP_IMPLEMENTATION
#define SCRWIDTH 800
#define SCRHEIGHT 600
#define TILESIZE 20
#include "external/fenster.h" // https://github.com/zserge/fenster

#define TINYBVH_IMPLEMENTATION
#define INST_IDX_BITS 8 // override default; space for 256 instances.
#include "tiny_bvh.h"
#include <fstream>
#include <thread>

using namespace tinybvh;

struct Sphere { bvhvec3 pos; float r; };

BVH sponza, obj, tlas;
BVHBase* bvhList[] = { &sponza, &obj };
BLASInstance inst[2];
int frameIdx = 0, verts = 0;
bvhvec4* triangles = 0;
Sphere* spheres = 0;
static std::atomic<int> tileIdx( 0 );
static unsigned threadCount = std::thread::hardware_concurrency();

// setup view pyramid for a pinhole camera
static bvhvec3 eye( -15.24f, 21.5f, 2.54f ), p1, p2, p3;
static bvhvec3 view = tinybvh_normalize( bvhvec3( 0.826f, -0.438f, -0.356f ) );

// callback for custom geometry: ray/sphere intersection
bool sphereIntersect( tinybvh::Ray& ray, const unsigned primID )
{
	bvhvec3 oc = ray.O - spheres[primID].pos;
	float b = tinybvh_dot( oc, ray.D ), r = spheres[primID].r;
	float c = tinybvh_dot( oc, oc ) - r * r, t, d = b * b - c;
	if (d <= 0) return false;
	d = sqrtf( d ), t = -b - d;
	bool hit = t < ray.hit.t && t > 0;
	if (hit) ray.hit.t = t, ray.hit.prim = primID;
	return hit;
}

bool sphereIsOccluded( const tinybvh::Ray& ray, const unsigned primID )
{
	bvhvec3 oc = ray.O - spheres[primID].pos;
	float b = tinybvh_dot( oc, ray.D ), r = spheres[primID].r;
	float c = tinybvh_dot( oc, oc ) - r * r, t, d = b * b - c;
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
	s.read( (char*)&verts, 4 );
	printf( "Loading triangle data (%i tris).\n", verts );
	verts *= 3, triangles = (bvhvec4*)malloc64( verts * 16 );
	s.read( (char*)triangles, verts * 16 );
	sponza.Build( triangles, verts / 3 );

	// create a blas for a single sphere
	spheres = new Sphere[1];
	spheres[0].pos = bvhvec3( 0 ), spheres[0].r = 2;
	obj.Build( &sphereAABB, 1 );
	obj.customIntersect = &sphereIntersect;
	obj.customIsOccluded = &sphereIsOccluded;

	// create instance list
	inst[0] = BLASInstance( 0 /* sponza */ );
	inst[1] = BLASInstance( 1 /* sphere */ );
}

bool UpdateCamera( float delta_time_s, fenster& f )
{
	bvhvec3 right = tinybvh_normalize( tinybvh_cross( bvhvec3( 0, 1, 0 ), view ) ), up = 0.8f * tinybvh_cross( view, right );
	float moved = 0, spd = 10.0f * delta_time_s;
	if (f.keys['A'] || f.keys['D']) eye += right * (f.keys['D'] ? spd : -spd), moved = 1;
	if (f.keys['W'] || f.keys['S']) eye += view * (f.keys['W'] ? spd : -spd), moved = 1;
	if (f.keys['R'] || f.keys['F']) eye += up * 2.0f * (f.keys['R'] ? spd : -spd), moved = 1;
	if (f.keys[20]) view = tinybvh_normalize( view + right * -0.1f * spd ), moved = 1;
	if (f.keys[19]) view = tinybvh_normalize( view + right * 0.1f * spd ), moved = 1;
	if (f.keys[17]) view = tinybvh_normalize( view + up * -0.1f * spd ), moved = 1;
	if (f.keys[18]) view = tinybvh_normalize( view + up * 0.1f * spd ), moved = 1;
	// recalculate right, up
	right = tinybvh_normalize( tinybvh_cross( bvhvec3( 0, 1, 0 ), view ) ), up = 0.8f * tinybvh_cross( view, right );
	bvhvec3 C = eye + 1.2f * view;
	p1 = C - right + up, p2 = C + right + up, p3 = C - right - up;
	return moved > 0;
}

void TraceWorkerThread( uint32_t* buf, int threadIdx )
{
	const int xtiles = SCRWIDTH / TILESIZE, ytiles = SCRHEIGHT / TILESIZE;
	const int tiles = xtiles * ytiles;
	int tile = threadIdx;
	while (tile < tiles)
	{
		const int tx = tile % xtiles, ty = tile / xtiles;
		const bvhvec3 L = tinybvh_normalize( bvhvec3( 1, 2, 3 ) );
		for (int y = 0; y < TILESIZE; y++) for (int x = 0; x < TILESIZE; x++)
		{
			const int pixel_x = tx * TILESIZE + x, pixel_y = ty * TILESIZE + y;
			const int pixelIdx = pixel_x + pixel_y * SCRWIDTH;
			// setup primary ray
			const float u = (float)pixel_x / SCRWIDTH, v = (float)pixel_y / SCRHEIGHT;
			const bvhvec3 D = tinybvh_normalize( p1 + u * (p2 - p1) + v * (p3 - p1) - eye );
			Ray ray( eye, D, 1e30f );
			tlas.Intersect( ray );
			if (ray.hit.t < 10000)
			{
			#if INST_IDX_BITS == 32
				// instance and primitive index are stored in separate fields
				uint32_t primIdx = ray.hit.prim;
				uint32_t instIdx = ray.hit.inst;
			#else
				// instance and primitive index are stored together for compactness
				uint32_t primIdx = ray.hit.prim & PRIM_IDX_MASK;
				uint32_t instIdx = (uint32_t)ray.hit.prim >> INST_IDX_SHFT;
			#endif
				BLASInstance& instance = inst[instIdx];
				uint32_t blasIdx = instance.blasIdx;
				bvhvec3 N;
				if (blasIdx == 0)
				{
					// we hit the Sponza mesh, which consists of triangles
					bvhvec3 v0 = triangles[primIdx * 3];
					bvhvec3 v1 = triangles[primIdx * 3 + 1];
					bvhvec3 v2 = triangles[primIdx * 3 + 2];
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
		tile = tileIdx++;
	}
}

void Tick( float delta_time_s, fenster& f, uint32_t* buf )
{
	// handle user input and update camera
	UpdateCamera( delta_time_s, f );

	// clear the screen with a debug-friendly color
	for (int i = 0; i < SCRWIDTH * SCRHEIGHT; i++) buf[i] = 0xaaaaff;

	// position the sphere
	static float bally = 20, ballv = 0;
	ballv -= 0.05f;
	bally += ballv;
	inst[1].transform[7] = bally;
	if (sponza.IntersectSphere( bvhvec3( 0, bally, 0 ), 2 )) ballv = -ballv;

	// build the tlas
	tlas.Build( inst, 2, bvhList, 2 );

	// render tiles
	tileIdx = threadCount;
	std::vector<std::thread> threads;
	for (uint32_t i = 0; i < threadCount; i++)
		threads.emplace_back( &TraceWorkerThread, buf, i );
	for (auto& thread : threads) thread.join();
}

void Shutdown() { /* nothing here. */ }