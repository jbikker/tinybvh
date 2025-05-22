// This example shows how to build a GPU path tracer with instancing
// using a TLAS. TinyOCL is used to render on the GPU using OpenCL.
// A more elaborate (and efficient) example is in tiny_bvh_interop.

#define FENSTER_APP_IMPLEMENTATION
#define SCRWIDTH 1280
#define SCRHEIGHT 720
#include "external/fenster.h" // https://github.com/zserge/fenster

#define GRIDSIZE 15
#define DRAGONS (GRIDSIZE * GRIDSIZE * GRIDSIZE)

// This application uses tinybvh - And this file will include the implementation.
#define TINYBVH_IMPLEMENTATION
#define INST_IDX_BITS 12
#include "tiny_bvh.h"
using namespace tinybvh;

// This application uses tinyocl - And this file will include the implementation.
#define TINY_OCL_IMPLEMENTATION
#include "tiny_ocl.h"

// Other includes
#include <fstream>
#include <cstdlib>
#include <cstdio>

// Application variables

static BVH8_CWBVH dragon;
static BVHBase* blasList[] = { &dragon, &dragon };
static BVH_GPU tlas;
static BLASInstance instance[DRAGONS + 1];
static bvhvec4* verts = 0;
static int triCount = 0, frameIdx = 0, spp = 0;
static Kernel* init, * clear, * rayGen, * extend, * shade;
static Kernel* updateCounters1, * updateCounters2, * traceShadows, * finalize;
static Buffer* pixels, * accumulator, * raysIn, * raysOut, * connections;
static Buffer* dragonNodes = 0, * dragonTris = 0, * drVerts, * noise = 0;
static Buffer* tlasNodes = 0, * tlasIndices = 0, * blasInstances = 0;
static size_t computeUnits;
static uint32_t* blueNoise = new uint32_t[128 * 128 * 8];

// View pyramid for a pinhole camera
struct RenderData
{
	bvhvec4 eye = bvhvec4( 0, 30, 0, 0 ), view = bvhvec4( -1, 0, 0, 0 ), C, p0, p1, p2;
	uint32_t frameIdx, dummy1, dummy2, dummy3;
} rd;

static float uniform_rand() { return (float)rand() / (float)RAND_MAX; }

// Scene management - Append a file, with optional position, scale and color override, tinyfied
void AddMesh( const char* file, float scale = 1, bvhvec3 pos = {}, int c = 0, int N = 0 )
{
	std::fstream s{ file, s.binary | s.in }; s.read( (char*)&N, 4 );
	bvhvec4* data = (bvhvec4*)tinybvh::malloc64( (N + triCount) * 48 );
	if (verts) memcpy( data, verts, triCount * 48 ), tinybvh::free64( verts );
	verts = data, s.read( (char*)verts + triCount * 48, N * 48 ), triCount += N;
	for (int* b = (int*)verts + (triCount - N) * 12, i = 0; i < N * 3; i++)
		*(bvhvec3*)b = *(bvhvec3*)b * scale + pos, b[3] = c ? c : b[3], b += 4;
}
void AddQuad( const bvhvec3 pos, const float w, const float d, int c )
{
	bvhvec4* data = (bvhvec4*)tinybvh::malloc64( (triCount + 2) * 48 );
	if (verts) memcpy( data + 6, verts, triCount * 48 ), tinybvh::free64( verts );
	data[0] = bvhvec3( -w, 0, -d ), data[1] = bvhvec3( w, 0, -d );
	data[2] = bvhvec3( w, 0, d ), data[3] = bvhvec3( -w, 0, -d ), verts = data;
	data[4] = bvhvec3( w, 0, d ), data[5] = bvhvec3( -w, 0, d ), triCount += 2;
	for (int i = 0; i < 6; i++) data[i] = 0.5f * data[i] + pos, data[i].w = *(float*)&c;
}

// Blue noise from file
void LoadBlueNoise()
{
	std::fstream s{ "./testdata/blue_noise_128x128x8_2d.raw", s.binary | s.in };
	s.read( (char*)blueNoise, 128 * 128 * 8 * 4 );
}

// Application init
void Init()
{
	// create OpenCL kernels
	init = new Kernel( "wavefront2.cl", "SetRenderData" );
	clear = new Kernel( "wavefront2.cl", "Clear" );
	rayGen = new Kernel( "wavefront2.cl", "Generate" );
	extend = new Kernel( "wavefront2.cl", "Extend" );
	shade = new Kernel( "wavefront2.cl", "Shade" );
	updateCounters1 = new Kernel( "wavefront2.cl", "UpdateCounters1" );
	updateCounters2 = new Kernel( "wavefront2.cl", "UpdateCounters2" );
	traceShadows = new Kernel( "wavefront2.cl", "Connect" );
	finalize = new Kernel( "wavefront2.cl", "Finalize" );

	// we need the 'compute unit' or 'SM' count for wavefront rendering; ask OpenCL for it.
	clGetDeviceInfo( init->GetDeviceID(), CL_DEVICE_MAX_COMPUTE_UNITS, sizeof( size_t ), &computeUnits, NULL );

	// create OpenCL buffers for wavefront path tracing
	int N = SCRWIDTH * SCRHEIGHT;
	raysIn = new Buffer( N * sizeof( bvhvec4 ) * 4 );
	raysOut = new Buffer( N * sizeof( bvhvec4 ) * 4 );
	connections = new Buffer( N * 3 * sizeof( bvhvec4 ) * 3 );
	accumulator = new Buffer( N * sizeof( bvhvec4 ) );
	pixels = new Buffer( N * sizeof( uint32_t ) );
	LoadBlueNoise();
	noise = new Buffer( 128 * 128 * 8 * sizeof( uint32_t ), blueNoise );
	noise->CopyToDevice();

	// load dragon mesh
	AddMesh( "./testdata/dragon.bin", 1, bvhvec3( 0 ) );
	dragon.BuildHQ( verts, triCount );

	// dragon grid
	for (int b = 0, x = 0; x < GRIDSIZE; x++) for (int y = 0; y < GRIDSIZE; y++) for (int z = 0; z < GRIDSIZE; z++, b++)
	{
		instance[b] = BLASInstance( 1 /* dragon */ );
		BLASInstance& i = instance[b];
		i.transform[0] = i.transform[5] = i.transform[10] = 0.07f;
		i.transform[3] = (float)x, i.transform[7] = (float)y, i.transform[11] = (float)z;
	}

	// finalize scene: build tlas
	tlas.Build( instance, DRAGONS, blasList, 2 );

	// create OpenCL buffers for BVH data
	tlasNodes = new Buffer( tlas.allocatedNodes /* could change! */ * sizeof( BVH_GPU::BVHNode ), tlas.bvhNode );
	tlasIndices = new Buffer( tlas.bvh.idxCount * sizeof( uint32_t ), tlas.bvh.primIdx );
	tlasNodes->CopyToDevice();
	tlasIndices->CopyToDevice();
	blasInstances = new Buffer( (DRAGONS + 1) * sizeof( BLASInstance ), instance );
	blasInstances->CopyToDevice();
	dragonNodes = new Buffer( dragon.usedBlocks * sizeof( bvhvec4 ), dragon.bvh8Data );
	dragonTris = new Buffer( dragon.idxCount * 3 * sizeof( bvhvec4 ), dragon.bvh8Tris );
	drVerts = new Buffer( triCount * 3 * sizeof( bvhvec4 ), verts );
	dragonNodes->CopyToDevice();
	dragonTris->CopyToDevice();
	drVerts->CopyToDevice();

	// load camera position / direction from file
	std::fstream t = std::fstream{ "camera_gpu.bin", t.binary | t.in };
	if (!t.is_open()) return;
	t.read( (char*)&rd, sizeof( rd ) );
}

// Keyboard handling
bool UpdateCamera( float delta_time_s, fenster& f )
{
	bvhvec3 right = tinybvh_normalize( tinybvh_cross( bvhvec3( 0, 1, 0 ), rd.view ) );
	bvhvec3 up = 0.8f * tinybvh_cross( rd.view, right );
	// get camera controls
	float moved = 0, spd = 10.0f * delta_time_s;
	if (f.keys['A'] || f.keys['D']) rd.eye += right * (f.keys['D'] ? spd : -spd), moved = 1;
	if (f.keys['W'] || f.keys['S']) rd.eye += rd.view * (f.keys['W'] ? spd : -spd), moved = 1;
	if (f.keys['R'] || f.keys['F']) rd.eye += up * 2.0f * (f.keys['R'] ? spd : -spd), moved = 1;
	if (f.keys[20]) rd.view = tinybvh_normalize( rd.view + right * -0.1f * spd ), moved = 1;
	if (f.keys[19]) rd.view = tinybvh_normalize( rd.view + right * 0.1f * spd ), moved = 1;
	if (f.keys[17]) rd.view = tinybvh_normalize( rd.view + up * -0.1f * spd ), moved = 1;
	if (f.keys[18]) rd.view = tinybvh_normalize( rd.view + up * 0.1f * spd ), moved = 1;
	// recalculate right, up
	right = tinybvh_normalize( tinybvh_cross( bvhvec3( 0, 1, 0 ), rd.view ) );
	up = 0.8f * tinybvh_cross( rd.view, right );
	bvhvec3 C = rd.eye + 1.2f * rd.view;
	rd.p0 = C - right + up, rd.p1 = C + right + up, rd.p2 = C - right - up;
	return moved > 0;
}

// Application Tick
void Tick( float delta_time_s, fenster& f, uint32_t* buf )
{
	// handle user input and update camera
	int N = SCRWIDTH * SCRHEIGHT;
	if (UpdateCamera( delta_time_s, f ) || frameIdx++ == 0)
	{
		clear->SetArguments( accumulator );
		clear->Run( N );
		spp = 1;
	}
	// wavefront step 0: render on the GPU
	init->SetArguments( N, rd.eye, rd.p0, rd.p1, rd.p2,
		frameIdx, SCRWIDTH, SCRHEIGHT,
		dragonNodes, dragonTris, drVerts, dragonNodes, dragonTris, drVerts,
		tlasNodes, tlasIndices, blasInstances,
		noise
	);
	init->Run( 1 ); // init atomic counters, set buffer ptrs etc.
	rayGen->SetArguments( raysOut, spp * 19191 );
	rayGen->Run2D( oclint2( SCRWIDTH, SCRHEIGHT ) );
	for (int i = 0; i < 3; i++)
	{
		swap( raysOut, raysIn );
		extend->SetArguments( raysIn );
		extend->Run( computeUnits * 64 * 16, 64 );
		updateCounters1->Run( 1 );
		shade->SetArguments( accumulator, raysIn, raysOut, connections, spp - 1 );
		shade->Run( computeUnits * 64 * 16, 64 );
		updateCounters2->Run( 1 );
	}
	traceShadows->SetArguments( accumulator, connections );
	traceShadows->Run( computeUnits * 64 * 8, 64 );
	finalize->SetArguments( accumulator, 1.0f / (float)spp++, pixels );
	finalize->Run2D( oclint2( SCRWIDTH, SCRHEIGHT ) );
	pixels->CopyFromDevice();
	memcpy( buf, pixels->GetHostPtr(), N * sizeof( uint32_t ) );
	// print frame time / rate in window title
	char title[50];
	sprintf( title, "tiny_bvh %.2f s %.2f Hz", delta_time_s, 1.0f / delta_time_s );
	fenster_update_title( &f, title );
}

// Application Shutdown
void Shutdown() { /* nothing here */ }