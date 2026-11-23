#define FENSTER_APP_IMPLEMENTATION
#define SCRWIDTH 800
#define SCRHEIGHT 600
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

struct CarSetup { float x, z, heading, speed; };
static const CarSetup carSetup[] = {
	{ -7.4f, -1.2f, 100, 0 },
	{ -9.4f, 0.9f, -135, 50 },
	{ -13.0f, -2.4f, -116, 20 },
	{ -15.2f, -1.7f, 180, 5 },
	{ -10.4f, 3.1f, -45, 8 },
};
static const int carCount = sizeof( carSetup ) / sizeof( carSetup[0] );
static const float carScale = 0.3f, shutter = 0.030f; // shutter interval in seconds
static BVH sponza, car, tlas;
static BVHBase* blasList[2] = { &sponza, &car };
static BLASInstance instances[1 + carCount]; // Sponza, then the cars
static bvhmat4 carEnd[carCount]; // the car transforms at the end of the shutter interval
static bool motion = true;
static bvhvec4* sponzaTris = 0, * carTris = 0;
static int sponzaTriCount = 0, carTriCount = 0, frameIdx = 0, spp = 0;
static bvhvec3 accumulator[SCRWIDTH * SCRHEIGHT];
static std::atomic<int> tileIdx( 0 );

// Multi-threading
static unsigned threadCount = std::thread::hardware_concurrency();

// Setup view pyramid for a pinhole camera:
// eye, p1 (top-left), p2 (top-right) and p3 (bottom-left)
static bvhvec3 eye( -11, 7, 1.4f ), p1, p2, p3;
static bvhvec3 view = tinybvh_normalize( bvhvec3( 0, -7, -1 ) );

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
	} while (tinybvh_dot( R, R ) > 1);
	return tinybvh_normalize( tinybvh_dot( R, N ) < 0 ? R : -R );
}
bvhvec3 CosWeightedDiffReflection( const bvhvec3 N, unsigned& seed )
{
	bvhvec3 R = DiffuseReflection( N, seed );
	return tinybvh_normalize( N + R );
}

// Color conversion
bvhvec3 rgb32_to_vec3( const unsigned c )
{
	return bvhvec3( (float)(c >> 16), (float)((c >> 8) & 255), (float)(c & 255) ) * (1 / 255.f);
}

// Geometry access: the hit instance tells us which mesh we hit.
const bvhvec4* InstanceTris( const unsigned instIdx ) { return instIdx == 0 ? sponzaTris : carTris; }
bvhvec3 TriangleColor( const unsigned instIdx, const unsigned prim )
{
	return rgb32_to_vec3( *(unsigned*)&InstanceTris( instIdx )[prim * 3].w );
}
bvhvec3 TriangleNormal( const unsigned instIdx, const unsigned prim )
{
	const bvhvec4* tris = InstanceTris( instIdx );
	bvhvec3 a = tris[prim * 3], b = tris[prim * 3 + 1], c = tris[prim * 3 + 2];
	// object space to world space; a car only changes position over time, not orientation,
	// so the transform at the start of the shutter interval has the right rotation.
	return tinybvh_normalize( tinybvh_transform_vector( tinybvh_cross( b - a, a - c ), instances[instIdx].transform ) );
}

// Load a mesh (raw triangle data) with an optional color override.
bvhvec4* LoadMesh( const char* file, int& N, const int c = 0 )
{
	std::fstream s{ file, s.binary | s.in };
	s.read( (char*)&N, 4 );
	bvhvec4* data = (bvhvec4*)malloc64( N * 48 );
	s.read( (char*)data, N * 48 );
	if (c) for (int i = 0; i < N * 3; i++) *(int*)&data[i].w = c;
	return data;
}

// Direction of a heading in degrees, on the floor plane.
bvhvec3 HeadingDir( const float heading )
{
	const float a = heading * 3.14159265f / 180;
	return bvhvec3( cosf( a ), 0, sinf( a ) );
}

// A car transform: rotated around y so that the front of the car (object +z, the blue
// hood) faces 'heading', scaled, placed at 'pos'.
bvhmat4 CarTransform( const bvhvec3 pos, const float heading )
{
	const bvhvec3 f = HeadingDir( heading ) * carScale; // world forward; object +x maps to (f.z, 0, -f.x)
	bvhmat4 m;
	m[0] = f.z, m[1] = 0, m[2] = f.x, m[3] = pos.x;
	m[4] = 0, m[5] = carScale, m[6] = 0, m[7] = pos.y;
	m[8] = -f.x, m[9] = 0, m[10] = f.z, m[11] = pos.z;
	return m;
}

// (Re)build the TLAS; cheap, there are only six instances.
void BuildTLAS()
{
	for (int i = 0; i < carCount; i++)
		instances[1 + i].transform1 = motion && carSetup[i].speed > 0 ? &carEnd[i] : 0;
	tlas.Build( instances, 1 + carCount, blasList, 2 );
}

// Application init
void Init()
{
	// load raw vertex data
	sponzaTris = LoadMesh( "./testdata/cryteksponza.bin", sponzaTriCount, 0xffffff );
	carTris = LoadMesh( "./testdata/legocar.bin", carTriCount );
	// build the BLASses
	sponza.Build( sponzaTris, sponzaTriCount );
	car.Build( carTris, carTriCount );
	// instances: Sponza, and the cars, which move forward during the shutter interval.
	instances[0] = BLASInstance( 0 );
	for (int i = 0; i < carCount; i++)
	{
		const CarSetup& c = carSetup[i];
		// put the car on the floor: find the floor below it, and lift the car by half its height.
		Ray down( bvhvec3( c.x, 20, c.z ), bvhvec3( 0, -1, 0 ) );
		sponza.Intersect( down );
		const bvhvec3 pos( c.x, 20 - down.hit.t + car.aabbMax.y * carScale, c.z );
		instances[1 + i] = BLASInstance( 1 );
		instances[1 + i].transform = CarTransform( pos, c.heading );
		carEnd[i] = CarTransform( pos + HeadingDir( c.heading ) * (c.speed * shutter), c.heading );
	}
	BuildTLAS();
}

// Keyboard handling
bool UpdateCamera( float delta_time_s, fenster& f )
{
	bvhvec3 right = tinybvh_normalize( tinybvh_cross( bvhvec3( 0, 1, 0 ), view ) ), up = 0.8f * tinybvh_cross( view, right );
	// get camera controls.
	bool moved = false;
	if (f.keys['A']) eye += right * -1.0f * delta_time_s * 10, moved = true;
	if (f.keys['D']) eye += right * delta_time_s * 10, moved = true;
	if (f.keys['W']) eye += view * delta_time_s * 10, moved = true;
	if (f.keys['S']) eye += view * -1.0f * delta_time_s * 10, moved = true;
	if (f.keys['R']) eye += up * delta_time_s * 20, moved = true;
	if (f.keys['F']) eye += up * -1.0f * delta_time_s * 20, moved = true;
	if (f.keys[20]) view = tinybvh_normalize( view + right * -1.0f * delta_time_s ), moved = true;
	if (f.keys[19]) view = tinybvh_normalize( view + right * delta_time_s ), moved = true;
	if (f.keys[17]) view = tinybvh_normalize( view + up * -1.0f * delta_time_s ), moved = true;
	if (f.keys[18]) view = tinybvh_normalize( view + up * delta_time_s ), moved = true;
	// toggle motion blur
	static bool mWasDown = false;
	if (f.keys['M'] && !mWasDown) motion = !motion, BuildTLAS(), moved = true;
	mWasDown = f.keys['M'] != 0;
	// recalculate right, up
	right = tinybvh_normalize( tinybvh_cross( bvhvec3( 0, 1, 0 ), view ) ), up = 0.8f * tinybvh_cross( view, right );
	bvhvec3 C = eye + 1.2f * view;
	p1 = C - right + up, p2 = C + right + up, p3 = C - right - up;
	return moved;
}

// Light transport calculation - Basic recursive Path Tracer with IS and Next Event Estimation.
// All rays of a path use the time of the primary ray.
bvhvec3 Trace( Ray ray, unsigned& seed, unsigned depth = 0 )
{
	// find primary intersection
	tlas.Intersect( ray );
	// shade
	if (ray.hit.t == 1e30f) return bvhvec3( 0.6f, 0.7f, 1 ); // hit nothing
#if INST_IDX_BITS == 32
	const unsigned prim = ray.hit.prim, instIdx = ray.hit.inst;
#else
	const unsigned prim = ray.hit.prim & PRIM_IDX_MASK, instIdx = ray.hit.prim >> INST_IDX_SHFT;
#endif
	const float time = ray.time;
	bvhvec3 I = ray.O + ray.hit.t * ray.D;
	bvhvec3 N = TriangleNormal( instIdx, prim );
	if (tinybvh_dot( N, ray.D ) > 0) N = -N;
	bvhvec3 BRDF = TriangleColor( instIdx, prim ) * (1.0f / 3.14159f);
	bvhvec3 Lpos( RandomFloat( seed ) * 30 - 15, 40, RandomFloat( seed ) * 6 - 3 ); // virtual
	float dist = tinybvh_length( Lpos - I );
	bvhvec3 L = (Lpos - I) * (1.0f / dist); // normalize
	bvhvec3 direct = {}, indirect = {};
	float NdotL = tinybvh_dot( N, L ), NLdotL = fabs( tinybvh_dot( L, bvhvec3( 0, 1, 0 ) ) );
	if (NdotL > 0)
		if (!tlas.IsOccluded( Ray( I + L * 0.001f, L, dist, RAY_MASK_INTERSECT_ALL, time ) ))
			direct = BRDF * NdotL * NLdotL * bvhvec3( 9, 9, 8 ) * 500 * (1.0f / (dist * dist));
	// random bounce
	if (depth < 2)
	{
		bvhvec3 R = CosWeightedDiffReflection( N, seed );
		float pdf = 1.0f / tinybvh_dot( N, R );
		bvhvec3 irradiance = Trace( Ray( I + R * 0.001f, R, 1e30f, RAY_MASK_INTERSECT_ALL, time ), seed, depth + 1 );
		indirect = BRDF * irradiance * (1.0f / pdf);
	}
	// finalize
	return direct + indirect;
}

void TraceWorkerThread( uint32_t* buf, float scale, int threadIdx )
{
	const int xtiles = SCRWIDTH / TILESIZE, ytiles = SCRHEIGHT / TILESIZE;
	const int tiles = xtiles * ytiles;
	int tile = threadIdx;
	while (tile < tiles)
	{
		const int tx = tile % xtiles, ty = tile / xtiles;
		unsigned seed = (tile + 17) * 171717 + frameIdx * 1023;
		for (int y = 0; y < TILESIZE; y++) for (int x = 0; x < TILESIZE; x++)
		{
			const int pixel_x = tx * TILESIZE + x, pixel_y = ty * TILESIZE + y;
			const int pixelIdx = pixel_x + pixel_y * SCRWIDTH;
			// setup primary ray, at a random time in the shutter interval
			const float u = (float)pixel_x / SCRWIDTH, v = (float)pixel_y / SCRHEIGHT;
			const bvhvec3 D = tinybvh_normalize( p1 + u * (p2 - p1) + v * (p3 - p1) - eye );
			const float time = RandomFloat( seed );
			// trace
			accumulator[pixelIdx] += Trace( Ray( eye, D, 1e30f, RAY_MASK_INTERSECT_ALL, time ), seed );
			const bvhvec3 E = accumulator[pixelIdx] * scale;
			// visualize, with a poor man's gamma correct
			const int r = (int)tinybvh_min( 255.0f, sqrtf( E.x ) * 255.0f );
			const int g = (int)tinybvh_min( 255.0f, sqrtf( E.y ) * 255.0f );
			const int b = (int)tinybvh_min( 255.0f, sqrtf( E.z ) * 255.0f );
			buf[pixelIdx] = b + (g << 8) + (r << 16);
		}
		tile = tileIdx++;
	}
}

// Application Tick
void Tick( float delta_time_s, fenster& f, uint32_t* buf )
{
	// handle user input and update camera
	if (UpdateCamera( delta_time_s, f ) || frameIdx++ == 0)
	{
		memset( accumulator, 0, SCRWIDTH * SCRHEIGHT * sizeof( bvhvec3 ) );
		spp = 1;
	}
	// render tiles
	const float scale = 1.0f / spp++;
	tileIdx = threadCount;
	std::vector<std::thread> threads;
	for (uint32_t i = 0; i < threadCount; i++)
		threads.emplace_back( &TraceWorkerThread, buf, scale, i );
	for (auto& thread : threads) thread.join();
	// print frame time / rate in window title
	char title[80];
	sprintf( title, "tiny_bvh motion blur %s - %i spp, %.2f s", motion ? "on" : "off", spp - 1, delta_time_s );
	fenster_update_title( &f, title );
}

// Application Shutdown
void Shutdown()
{
	free64( sponzaTris );
	free64( carTris );
}
