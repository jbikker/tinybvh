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
	// load gltf scene
	scene.SetBVHDefault( GPU_DYNAMIC );
	scene.AddScene( "./testdata/drone/scene.gltf", ts_mat4::scale( 0.1f ) );
	scene.SetSkyDome( new SkyDome( "./testdata/sky_15.hdr" ) );
	scene.UpdateSceneGraph( 0 ); // this will build the BVHs.
	// create OpenCL kernels
	init = new Kernel( "raytracer.cl", "SetRenderData" );
	render = new Kernel( "raytracer.cl", "Render" );
	screen = 0; // this tells the template to not overwrite the render target.
	// create OpenCL buffers 
	// 1. pixel buffer: We will render to this. Synced with template OpenGL render target.
	int N = SCRWIDTH * SCRHEIGHT;
	pixels = new Buffer( GetRenderTarget()->ID, 0, Buffer::TARGET );
	// 2. instance buffer: For the BLASInstances.
	instances = new Buffer( (unsigned)Scene::instPool.size() * sizeof( BLASInstance ), Scene::instPool.data() );
	// 3. BLAS buffers, one per tinyscene mesh.
	for( int i = 0; i < Scene::meshPool.size(); i++ )
	{
		// a BLAS needs BVH nodes, triangle indices and the actual triangle data.
		BVH_GPU* gpubvh = Scene::meshPool[i]->blas.dynamicGPU;
		blasNode[i] = new Buffer( gpubvh->usedNodes * sizeof( BVH_GPU::BVHNode ), gpubvh->bvhNode );
		blasIdx[i] = new Buffer( gpubvh->idxCount * sizeof( uint32_t ), gpubvh->bvh.primIdx );
		blasTri[i] = new Buffer( gpubvh->triCount * sizeof( bvhvec4 ) * 3, (void*)gpubvh->bvh.verts.data );
	}
	// 4. TLAS buffer. The TLAS references the BLASInstances.
	BVH_GPU* tlas = Scene::gpuTlas;
	tlasNode = new Buffer( tlas->usedNodes * sizeof( BVH_GPU::BVHNode ), tlas->bvhNode );
	tlasIdx = new Buffer( tlas->idxCount * sizeof( uint32_t ), tlas->bvh.primIdx );
	// 5. materials buffer.

	// 6. texture descriptors buffer.

	// 7. texture bitmap buffer.

}

// -----------------------------------------------------------
// Update camera
// -----------------------------------------------------------
bool GLTFDemo::UpdateCamera( float /* delta_time_s */ )
{
	return true; // TODO
}

// -----------------------------------------------------------
// Main application tick function - Executed once per frame
// -----------------------------------------------------------
void GLTFDemo::Tick( float delta_time )
{
	// handle user input and update camera
	int N = SCRWIDTH * SCRHEIGHT;
	UpdateCamera( delta_time * 0.001f );
}