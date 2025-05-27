// Template, 2024 IGAD Edition
// Get the latest version from: https://github.com/jbikker/tmpl8
// IGAD/NHTV/BUAS/UU - Jacco Bikker - 2006-2024

#include "precomp.h"
using namespace tinybvh;
using namespace tinyocl;
using namespace tinyscene;
#include "gltfdemo.h"

#define AUTOCAM

// -----------------------------------------------------------
// Initialize the application
// -----------------------------------------------------------
void GLTFDemo::Init()
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
	finalize = new Kernel( "wavefront2.cl", "FinalizeGL" );
	screen = 0; // this tells the template to not overwrite the render target.

	// we need the 'compute unit' or 'SM' count for wavefront rendering; ask OpenCL for it.
	clGetDeviceInfo( init->GetDeviceID(), CL_DEVICE_MAX_COMPUTE_UNITS, sizeof( size_t ), &computeUnits, NULL );

	// create OpenCL buffers for wavefront path tracing
	int N = SCRWIDTH * SCRHEIGHT;
	pixels = new Buffer( GetRenderTarget()->ID, 0, Buffer::TARGET );
	raysIn = new Buffer( N * sizeof( float4 ) * 4 );
	raysOut = new Buffer( N * sizeof( float4 ) * 4 );
	connections = new Buffer( N * 3 * sizeof( float4 ) * 3 );
	accumulator = new Buffer( N * sizeof( float4 ) );
	LoadBlueNoise();
	noise = new Buffer( 128 * 128 * 8 * sizeof( uint32_t ), blueNoise );
	noise->CopyToDevice();

	// load dragon mesh
	AddMesh( "./testdata/dragon.bin", 1, float3( 0 ) );
	swap( verts, dragonVerts );
	swap( triCount, dragonTriCount );
	if (!dragon.Load( "dragon.bvh", dragonTriCount ))
	{
		dragon.BuildHQ( dragonVerts, dragonTriCount );
		dragon.Save( "dragon.bvh" );
	}

	// create dragon instances
	for (int d = 0; d < DRAGONS; d++)
	{
		instance[d + 1] = BLASInstance( 1 /* dragon */ );
		BLASInstance& i = instance[d + 1];
		float t = (float)d * 0.17f + 1.0f;
		float size = 0.1f + 0.075f * RandomFloat();
		float3 pos = splinePos( t );
		float3 D = -size * normalize( splinePos( t + 0.01f ) - pos );
		float3 U( 0, size, 0 );
		float3 R( -D.z, 0, D.x );
		pos += R * 20.0f * (RandomFloat() - 0.5f);
		i.transform[0] = R.x, i.transform[1] = R.y, i.transform[2] = R.z;
		i.transform[4] = U.x, i.transform[5] = U.y, i.transform[6] = U.z;
		i.transform[8] = D.x, i.transform[9] = D.y, i.transform[10] = D.z;
		i.transform[3] = pos.x;
		i.transform[7] = -9.2f;
		i.transform[11] = pos.z;
	}
	// load vertex data for static scenery
	AddQuad( float3( -22, 12, 2 ), 9, 5, 0x1ffffff ); // hard-coded light source
	AddMesh( "./testdata/bistro_ext_part1.bin", 1, float3( 0 ) );
	AddMesh( "./testdata/bistro_ext_part2.bin", 1, float3( 0 ) );

	// build bvh (here: 'compressed wide bvh', for efficient GPU rendering)
	if (!bistro.Load( "bistro.bvh", triCount ))
	{
		bistro.BuildHQ( verts, triCount );
		bistro.Save( "bistro.bvh" );
	}

	instance[0] = BLASInstance( 0 /* static geometry */ );
	tlas.Build( instance, DRAGONS + 1, blasList, 2 );

	// create OpenCL buffers for BVH data
	tlasNodes = new Buffer( tlas.allocatedNodes /* could change! */ * sizeof( BVH_GPU::BVHNode ), tlas.bvhNode );
	tlasIndices = new Buffer( tlas.bvh.idxCount * sizeof( uint32_t ), tlas.bvh.primIdx );
	tlasNodes->CopyToDevice();
	tlasIndices->CopyToDevice();
	blasInstances = new Buffer( (DRAGONS + 1) * sizeof( BLASInstance ), instance );
	blasInstances->CopyToDevice();
	bistroNodes = new Buffer( bistro.usedBlocks * sizeof( float4 ), bistro.bvh8Data );
	bistroTris = new Buffer( bistro.idxCount * 3 * sizeof( float4 ), bistro.bvh8Tris );
	bistroVerts = new Buffer( triCount * 3 * sizeof( float4 ), verts );
	dragonNodes = new Buffer( dragon.usedBlocks * sizeof( float4 ), dragon.bvh8Data );
	dragonTris = new Buffer( dragon.idxCount * 3 * sizeof( float4 ), dragon.bvh8Tris );
	drVerts = new Buffer( dragonTriCount * 3 * sizeof( float4 ), dragonVerts );
	dragonNodes->CopyToDevice();
	dragonTris->CopyToDevice();
	drVerts->CopyToDevice();
	bistroNodes->CopyToDevice();
	bistroTris->CopyToDevice();
	bistroVerts->CopyToDevice();
}

// -----------------------------------------------------------
// Update camera
// -----------------------------------------------------------
bool GLTFDemo::UpdateCamera( float delta_time_s )
{
	// playback camera spline
	static float ct = 0, moved = 1;
	ct += delta_time_s * 0.25f;
	if (ct > 10) ct -= 10;
	splineCam( ct + 1 );
	float3 right = normalize( cross( float3( 0, 1, 0 ), rd.view ) );
	float3 up = 0.8f * cross( rd.view, right ), C = rd.eye + 1.2f * rd.view;
	rd.p0 = C - right + up, rd.p1 = C + right + up, rd.p2 = C - right - up;
	return moved > 0;
}

// -----------------------------------------------------------
// Main application tick function - Executed once per frame
// -----------------------------------------------------------
void GLTFDemo::Tick( float delta_time )
{
	// handle user input and update camera
	int N = SCRWIDTH * SCRHEIGHT;
	UpdateCamera( delta_time * 0.001f );
	clear->SetArguments( accumulator );
	clear->Run( N );
	spp = 1;
	// wavefront step 0: render on the GPU
	init->SetArguments( N, rd.eye, rd.p0, rd.p1, rd.p2,
		frameIdx, SCRWIDTH, SCRHEIGHT,
		bistroNodes, bistroTris, bistroVerts, dragonNodes, dragonTris, drVerts,
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
}