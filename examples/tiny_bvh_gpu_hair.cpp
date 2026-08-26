// This example shows how to render hair strands using the method described in:
// Modeling Hair Strands with Roving Capsules, Reshetov & Hart, 2024.
// Also see: https://www.shadertoy.com/view/4ffXWs - And:
// https://mmzala.github.io/blog/hair-geometry / https://github.com/mmzala/vkhrt

#define FENSTER_APP_IMPLEMENTATION
#define SCRWIDTH 1280
#define SCRHEIGHT 720
#include "external/fenster.h" // https://github.com/zserge/fenster

// This application uses tinybvh - And this file will include the implementation.
#define TINYBVH_IMPLEMENTATION
#include "tiny_bvh.h"
using namespace tinybvh;

// This application uses tinyocl - And this file will include the implementation.
#define TINY_OCL_IMPLEMENTATION
#include "tiny_ocl.h"

// Other includes
#include <fstream>

// Application variables
static BVH_GPU bvh;
static Kernel* init, * trace;
static Buffer* pixels, * bvhNodes = 0, * bvhIndices = 0, * hairs = 0, * hairVerts = 0;

// View pyramid for a pinhole camera
struct RenderData
{
	bvhvec4 eye = bvhvec4( 35, 5, -20, 0 );
	bvhvec4 view = tinybvh_normalize( bvhvec4( -1, -0.1f, 0.5f, 0 ) );
	bvhvec4 C, p0, p1, p2;
} rd;

// Hair data
struct Strand { uint32_t offset, N; };
constexpr float strandRadius = 0.2f;
uint32_t strandCount = 0, offset = 0;
Strand* strands = 0;
bvhaabb* strandBox = 0;
bvhvec4* vertPool = 0, * vertPtr = 0;
void LoadHair( const char* file )
{
	delete[] vertPool;
	delete[] strands;
	delete[] strandBox;
	std::fstream s{ file, s.binary | s.in };
	s.read( (char*)&strandCount, 4 );
	vertPool = vertPtr = new bvhvec4[68 * strandCount]; // sufficient for straight & curly.
	memset( vertPool, 0, sizeof( bvhvec4 ) * 68 * strandCount );
	strands = new Strand[strandCount];
	strandBox = new bvhaabb[strandCount];
	uint32_t strnd = 0;
	for (uint32_t i = 0; i < strandCount; i += 100, strnd++ )
	{
		s.read( (char*)&strands[strnd].N, 4 );
		strands[strnd].offset = offset;
		strandBox[strnd].minBounds = bvhvec3( BVH_FAR );
		strandBox[strnd].maxBounds = bvhvec3( -BVH_FAR );
		for (uint32_t j = 0; j < strands[strnd].N; j++, vertPtr++)
		{
			s.read( (char*)vertPtr, 12 );
			vertPtr->w = strandRadius;
			strandBox[strnd].minBounds = tinybvh_min( strandBox[strnd].minBounds, bvhvec3( *vertPtr ) );
			strandBox[strnd].maxBounds = tinybvh_max( strandBox[strnd].maxBounds, bvhvec3( *vertPtr ) );
		}
		strandBox[strnd].minBounds = strandBox[strnd].minBounds - bvhvec3( strandRadius );
		strandBox[strnd].maxBounds = strandBox[strnd].maxBounds + bvhvec3( strandRadius );
		offset += strands[strnd].N;
	}
	strandCount = strnd;
}

// Application init
void Init()
{
	// create OpenCL kernels
	init = new Kernel( "kernels/hairviewer.cl", "SetRenderData" );
	trace = new Kernel( "kernels/hairviewer.cl", "Render" );
	// create OpenCL buffers for wavefront path tracing
	int N = SCRWIDTH * SCRHEIGHT;
	pixels = new Buffer( N * sizeof( uint32_t ) );
	// load raw vertex data
	LoadHair( "./testdata/strands_S_curly.data" );
	hairs = new Buffer( strandCount * sizeof( Strand ), strands );
	hairVerts = new Buffer( offset * sizeof( bvhvec4 ), vertPool );
	hairs->CopyToDevice();
	hairVerts->CopyToDevice();
	// build bvh over hair AABBs
	bvh.BuildAABB( (bvhvec4*)strandBox, strandCount );
	// create OpenCL buffers for BVH data
	bvhNodes = new Buffer( bvh.usedNodes * sizeof( BVH_GPU::BVHNode ), bvh.bvhNode );
	bvhIndices = new Buffer( bvh.idxCount * sizeof( uint32_t ), bvh.bvh.primIdx );
	bvhNodes->CopyToDevice();
	bvhIndices->CopyToDevice();
}

// Keyboard handling
bool UpdateCamera( float delta_time_s, fenster& f )
{
	bvhvec3 right = tinybvh_normalize( tinybvh_cross( bvhvec3( 0, 1, 0 ), rd.view ) ), up = 0.8f * tinybvh_cross( rd.view, right );
	// get camera controls.
	float moved = 0, spd = 10.0f * delta_time_s;
	if (f.keys['A'] || f.keys['D']) rd.eye += right * (f.keys['D'] ? spd : -spd), moved = 1;
	if (f.keys['W'] || f.keys['S']) rd.eye += rd.view * (f.keys['W'] ? spd : -spd), moved = 1;
	if (f.keys['R'] || f.keys['F']) rd.eye += up * 2.0f * (f.keys['R'] ? spd : -spd), moved = 1;
	if (f.keys[20]) rd.view = tinybvh_normalize( rd.view + right * -0.1f * spd ), moved = 1;
	if (f.keys[19]) rd.view = tinybvh_normalize( rd.view + right * 0.1f * spd ), moved = 1;
	if (f.keys[17]) rd.view = tinybvh_normalize( rd.view + up * -0.1f * spd ), moved = 1;
	if (f.keys[18]) rd.view = tinybvh_normalize( rd.view + up * 0.1f * spd ), moved = 1;
	// recalculate right, up
	right = tinybvh_normalize( tinybvh_cross( bvhvec3( 0, 1, 0 ), rd.view ) ), up = 0.8f * tinybvh_cross( rd.view, right );
	bvhvec3 C = rd.eye + 1.2f * rd.view;
	rd.p0 = C - right + up, rd.p1 = C + right + up, rd.p2 = C - right - up;
	return moved > 0;
}

// Application Tick
void Tick( float delta_time_s, fenster& f, uint32_t* buf )
{
	// handle user input and update camera
	int N = SCRWIDTH * SCRHEIGHT;
	UpdateCamera( delta_time_s, f );
	init->SetArguments( rd.eye, rd.p0, rd.p1, rd.p2 );
	init->Run( 1 );
	trace->SetArguments( pixels, bvhNodes, bvhIndices, hairs, hairVerts );
	trace->Run2D( oclint2( SCRWIDTH, SCRHEIGHT ) );
	pixels->CopyFromDevice();
	memcpy( buf, pixels->GetHostPtr(), N * sizeof( uint32_t ) );
}

// Application Shutdown
void Shutdown() { /* nothing here */ }