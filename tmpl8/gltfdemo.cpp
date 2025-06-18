// Template, 2024 IGAD Edition
// Get the latest version from: https://github.com/jbikker/tmpl8
// IGAD/NHTV/BUAS/UU - Jacco Bikker - 2006-2024

#include "precomp.h"
using namespace tinybvh;
using namespace tinyocl;
using namespace tinyscene;
#include "gltfdemo.h"

// The material description in tinyscene is very elaborate. Instead of sending it
// directly to the GPU we convert it to a limited and more convenient layout.
// Later this will also allow us to optimize it for a specific device.
// Note: Make sure to make the size of this struct a multiple of 16 bytes.
struct GPUMaterial
{
	enum { HAS_TEXTURE = 1, HAS_NMAP = 2 };
	float4 albedo;				// beware: OpenCL 3-component vectors align to 16 byte.
	uint flags;					// material flags.
	uint offset;				// start of texture data in the texture bitmap buffer.
	uint width, height;			// size of the texture.
	uint normalOffset;			// start of the normal map data in the texture bitmap buffer.
	uint wnormal, hnormal;		// normal map size.
	uint dummy;					// padding; struct size must be a multiple of 16 bytes.
};

// -----------------------------------------------------------
// Initialize the application
// -----------------------------------------------------------
void GLTFDemo::Init()
{
	// load gltf scene
	scene.SetBVHDefault( GPU_DYNAMIC );
	scene.AddScene( "./testdata/cratercity/scene.gltf", mat4::Translate( 0, -18.9f, 0 ) );
	scene.AddScene( "./testdata/mangotree/scene.gltf", mat4::Translate( 5, -2.9f, 0 ) * mat4::Scale( 2 ) );
	scene.AddScene( "./testdata/drone/scene.gltf", mat4::Translate( 13.9f, -1.9f, 11.8f ) * mat4::Scale( 0.03f ) );
	scene.SetSkyDome( new SkyDome( "./testdata/sky_15.hdr" ) );
	int leaves = scene.FindNode( "leaves" );
	scene.CreateOpacityMicroMaps( leaves );
	scene.UpdateSceneGraph( 0 ); // this will build the BLASses and TLAS.

	// create OpenCL kernels
	init = new Kernel( "raytracer.cl", "SetRenderData" );
	render = new Kernel( "raytracer.cl", "Render" );
	screen = 0; // this tells the template to not overwrite the render target.

	// create OpenCL buffers

	// 1. pixel buffer: We will render to this. Synced with template OpenGL render target.
	pixels = new Buffer( GetRenderTarget()->ID, 0, Buffer::TARGET );

	// 2. instance buffer: For the BLASInstances.
	instances = new Buffer( (unsigned)Scene::instPool.size() * sizeof( BLASInstance ), Scene::instPool.data() );
	instances->CopyToDevice();

	// 3. BLAS buffers, one per tinyscene mesh.
	// Complication: OpenCL does not let us pass device-side pointers, let alone arrays
	// of them. So, we make a single large buffer for all BLAS nodes, and use offsets
	// within it for the individual BLASses. Same for indices and triangles.
	uint nodeCount = 0, indexCount = 0, triCount = 0, opmapOffset = 0;
	blasDesc = new Buffer( (int)Scene::meshPool.size() * 32 );
	for (int i = 0; i < Scene::meshPool.size(); i++)
	{
		BVH_GPU* gpubvh = Scene::meshPool[i]->blas.dynamicGPU;
		blasDesc->GetHostPtr()[i * 8 + 0] = nodeCount;
		blasDesc->GetHostPtr()[i * 8 + 1] = indexCount;
		blasDesc->GetHostPtr()[i * 8 + 2] = triCount;
		blasDesc->GetHostPtr()[i * 8 + 3] = gpubvh->opmap ? opmapOffset : 0x99999999;
		nodeCount += gpubvh->usedNodes, indexCount += gpubvh->idxCount, triCount += gpubvh->triCount;
		if (gpubvh->opmap) opmapOffset += gpubvh->triCount * 32; // for N=32: 128 bytes = 32uints
	}
	blasNode = new Buffer( nodeCount * sizeof( BVH_GPU::BVHNode ) );
	blasIdx = new Buffer( indexCount * sizeof( uint ) );
	blasTri = new Buffer( triCount * sizeof( float4 ) * 3 );
	blasOpMap = new Buffer( (opmapOffset > 0 ? opmapOffset : 32) * 4 );
	blasFatTri = new Buffer( triCount * sizeof( FatTri ) );
	nodeCount = 0, indexCount = 0, triCount = 0, opmapOffset = 0;
	for (int i = 0; i < Scene::meshPool.size(); i++)
	{
		// a BLAS needs BVH nodes, triangle indices and the actual triangle data.
		BVH_GPU* gpubvh = Scene::meshPool[i]->blas.dynamicGPU;
		memcpy( (BVH_GPU::BVHNode*)blasNode->GetHostPtr() + nodeCount, gpubvh->bvhNode, gpubvh->usedNodes * sizeof( BVH_GPU::BVHNode ) );
		memcpy( (uint*)blasIdx->GetHostPtr() + indexCount, gpubvh->bvh.primIdx, gpubvh->idxCount * sizeof( uint ) );
		memcpy( (float4*)blasTri->GetHostPtr() + triCount * 3, gpubvh->bvh.verts.data, gpubvh->triCount * sizeof( float4 ) * 3 );
		if (gpubvh->opmap) memcpy( (uint32_t*)blasOpMap->GetHostPtr() + opmapOffset * 32, gpubvh->opmap, gpubvh->triCount * 128 );
		memcpy( (FatTri*)blasFatTri->GetHostPtr() + triCount, Scene::meshPool[i]->triangles.data(), gpubvh->triCount * sizeof( FatTri ) );
		nodeCount += gpubvh->usedNodes, indexCount += gpubvh->idxCount, triCount += gpubvh->triCount;
		if (gpubvh->opmap) opmapOffset += gpubvh->triCount * 32; // for N=32: 128 bytes = 32uints
	}
	blasNode->CopyToDevice();
	blasIdx->CopyToDevice();
	blasTri->CopyToDevice();
	blasOpMap->CopyToDevice();
	blasFatTri->CopyToDevice();
	blasDesc->CopyToDevice();

	// 4. TLAS buffer. The TLAS references the BLASInstances.
	// Note: The TLAS is rebuilt per frame. By syncing 'allocatedNodes' rather than
	// 'usedNodes', we account for fluctuating node counts. This is similar to RTX, where
	// static meshes can be compacted, but dynamic ones can't.
	tlasNode = new Buffer( Scene::gpuTlas->allocatedNodes * sizeof( BVH_GPU::BVHNode ), Scene::gpuTlas->bvhNode );
	tlasIdx = new Buffer( Scene::gpuTlas->idxCount * sizeof( uint ), Scene::gpuTlas->bvh.primIdx );

	// 5. texture pixel buffer.
	int textureCount = (int)Scene::textures.size(), texturePixelCount = 0;
	int* texOffset = new int[textureCount];
	for (int i = 0; i < textureCount; i++)
	{
		Texture* t = Scene::textures[i];
		if (!t->idata) continue; // TODO: floating point textures.
		texOffset[i] = texturePixelCount;
		texturePixelCount += t->width * t->height;
	}
	texels = new Buffer( texturePixelCount * sizeof( uint ) );
	texturePixelCount = 0;
	for (int i = 0; i < textureCount; i++)
	{
		Texture* t = Scene::textures[i];
		if (!t->idata) continue; // TODO: floating point textures.
		memcpy( texels->GetHostPtr() + texturePixelCount, t->idata, t->width * t->height * 4 );
		texturePixelCount += t->width * t->height;
	}
	texels->CopyToDevice();

	// 6. materials buffer.
	// We can't just use Scene::Material as it is too complex so we convert what we need.
	int materialCount = (int)Scene::materials.size();
	materials = new Buffer( materialCount * sizeof( GPUMaterial ) );
	for (int i = 0; i < materialCount; i++)
	{
		Material* original = Scene::materials[i];
		GPUMaterial* for_gpu = (GPUMaterial*)materials->GetHostPtr() + i;
		memset( for_gpu, 0, sizeof( GPUMaterial ) );
		for_gpu->albedo = float4( original->color.value, 0 );
		for_gpu->flags = 0;
		int textureID = original->color.textureID;
		if (textureID > -1)
		{
			Texture* t = Scene::textures[textureID];
			if (t->idata)
			{
				for_gpu->flags |= GPUMaterial::HAS_TEXTURE;
				for_gpu->width = t->width, for_gpu->height = t->height;
				for_gpu->offset = texOffset[textureID];
			}
		}
		textureID = original->normals.textureID;
		if (textureID > -1)
		{
			Texture* t = Scene::textures[textureID];
			if (t->idata)
			{
				for_gpu->flags |= GPUMaterial::HAS_NMAP;
				for_gpu->wnormal = t->width, for_gpu->hnormal = t->height;
				for_gpu->normalOffset = texOffset[textureID];
			}
		}
	}
	materials->CopyToDevice();

	// 7. sky bitmap (HDR).
	skyPixels = new Buffer( Scene::sky->width * Scene::sky->height * 12, Scene::sky->pixels );
	skyPixels->CopyToDevice();

	// pass all static data to the Init function.
	init->SetArguments( tlasNode, tlasIdx, instances, blasNode, blasIdx, blasTri, blasOpMap, blasFatTri, blasDesc,
		materials, texels, Scene::sky->width, Scene::sky->height, skyPixels );
	init->Run( 1 );

	// set a suitable camera
	eye = float3( -4.31f, 2.63f, 19.44f );
	view = float3( 0.16f, -0.16f, -0.97f );
}

// -----------------------------------------------------------
// Update camera
// -----------------------------------------------------------
bool GLTFDemo::UpdateCamera( float delta_time_s )
{
	bvhvec3 right = tinybvh_normalize( tinybvh_cross( bvhvec3( 0, 1, 0 ), view ) );
	bvhvec3 up = 0.8f * tinybvh_cross( view, right );

	// get camera controls.
	bool moved = false;
	if (GetAsyncKeyState( 'A' )) eye += right * -1.0f * delta_time_s * 10.0f, moved = true;
	if (GetAsyncKeyState( 'D' )) eye += right * delta_time_s * 10.0f, moved = true;
	if (GetAsyncKeyState( 'W' )) eye += view * delta_time_s * 10.0f, moved = true;
	if (GetAsyncKeyState( 'S' )) eye += view * -1.0f * delta_time_s * 10.0f, moved = true;
	if (GetAsyncKeyState( 'R' )) eye += up * delta_time_s * 10.0f, moved = true;
	if (GetAsyncKeyState( 'F' )) eye += up * -1.0f * delta_time_s * 10.0f, moved = true;
	if (GetAsyncKeyState( VK_LEFT )) view = tinybvh_normalize( view + right * -1.0f * delta_time_s ), moved = true;
	if (GetAsyncKeyState( VK_RIGHT )) view = tinybvh_normalize( view + right * delta_time_s ), moved = true;
	if (GetAsyncKeyState( VK_UP )) view = tinybvh_normalize( view + up * -1.0f * delta_time_s ), moved = true;
	if (GetAsyncKeyState( VK_DOWN )) view = tinybvh_normalize( view + up * delta_time_s ), moved = true;

	// recalculate right, up
	right = tinybvh_normalize( tinybvh_cross( bvhvec3( 0, 1, 0 ), view ) );
	up = 0.8f * tinybvh_cross( view, right );
	bvhvec3 C = eye + 2.0f * view;
	float aspect = 1.5f;
	p1 = C - right * aspect + up, p2 = C + right * aspect + up, p3 = C - right * aspect - up;
	return moved;
}

// -----------------------------------------------------------
// Main application tick function - Executed once per frame
// -----------------------------------------------------------
void GLTFDemo::Tick( float delta_time )
{
	// handle user input and update camera.
	UpdateCamera( delta_time * 0.001f );
	scene.UpdateSceneGraph( delta_time * 0.001f );

	// sync tlas - TODO: would be better if this could be a single copy.
	tlasNode->CopyToDevice();
	tlasIdx->CopyToDevice();
	instances->CopyToDevice();

	// start device-side rendering.
	render->SetArguments( pixels, SCRWIDTH, SCRHEIGHT, eye, p1, p2, p3 );
	render->Run2D( oclint2( SCRWIDTH, SCRHEIGHT ) );

	// print camera aim pos
	static int delay = 10;
	if (--delay == 0)
	{
		printf( "x = %.2f, y = %.2f, z = %.2f)\n", eye.x, eye.y, eye.z );
		delay = 10;
	}
}