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

// BLAS descriptor.
struct BLASDesc
{
	uint nodeOffset;			// position of blas node data in global blasNodes array
	uint indexOffset;			// position of blas index data in global blasIdx array
	uint triOffset;				// position of blas triangle and FatTri data in global arrays
	uint opmapOffset;			// position of opacity micromap data in global arrays
	uint node8Offset;			// position of CWBVH nodes in global array
	uint tri8Offset;			// position of CWBVH triangle data in global array
	uint blasType;				// blas type: 0 = BVH_GPU, 1 = BVH8_CWBVH
	uint dummy;					// padding
};

// -----------------------------------------------------------
// Initialize the application
// -----------------------------------------------------------
void GLTFDemo::Init()
{
	// load gltf scene
	scene.SetBVHDefault( GPU_DYNAMIC );
	int terrain = scene.AddScene( "./testdata/cratercity/scene.gltf", mat4::Translate( 0, -18.9f, 0 ) * mat4::RotateY( 1 ) );
	scene.AddScene( "./testdata/mangotree/scene.gltf", mat4::Translate( 5, -3.5f, 0 ) * mat4::Scale( 2 ) );
	scene.AddScene( "./testdata/drone/scene.gltf", mat4::Translate( 21.5f, -1.75f, -7 ) * mat4::Scale( 0.03f ) * mat4::RotateY( PI * 1.5f ) );
	scene.SetSkyDome( new SkyDome( "./testdata/sky_15.hdr" ) );
	int leaves = scene.FindNode( "leaves" );
	if (leaves > -1) scene.CreateOpacityMicroMaps( leaves );
	// scene.meshPool[0]->blas.bvhType = GPU_RIGID; // ->SetBVHType( terrain, GPU_RIGID );
	scene.CollapseMeshes( terrain ); // combine the meshes into a single mesh; may yield a better BVH.
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
	uint node2Count = 0, node8Count = 0, indexCount = 0, triCount = 0, tri8Count = 0, opmapOffset = 0;
	blasDesc = new Buffer( (int)Scene::meshPool.size() * sizeof( BLASDesc ) );
	for (int i = 0; i < Scene::meshPool.size(); i++)
	{
		Mesh* mesh = Scene::meshPool[i];
		if (!mesh) continue;
		BLASDesc& desc = ((BLASDesc*)blasDesc->GetHostPtr())[i];
		desc.indexOffset = indexCount;
		desc.triOffset = triCount;
		desc.blasType = GPU_DYNAMIC;
		if (mesh->blas.bvhType == GPU_DYNAMIC)
		{
			BVH_GPU* gpubvh2 = mesh->blas.dynamicGPU;
			desc.nodeOffset = node2Count;
			desc.opmapOffset = gpubvh2->opmap ? opmapOffset : 0x99999999;
			node2Count += gpubvh2->usedNodes;
			indexCount += gpubvh2->idxCount;
			triCount += gpubvh2->triCount;
			if (gpubvh2->opmap) opmapOffset += gpubvh2->triCount * 32; // for N=32: 128 bytes = 32uints
		}
		else
		{
			BVH8_CWBVH* gpubvh8 = mesh->blas.rigidGPU;
			desc.node8Offset = node8Count;
			desc.tri8Offset = tri8Count;
			desc.opmapOffset = gpubvh8->opmap ? opmapOffset : 0x99999999;
			desc.blasType = GPU_RIGID;
			node8Count += gpubvh8->usedNodes;
			tri8Count += gpubvh8->idxCount; // not triCount; we must account for spatial splits.
			if (gpubvh8->opmap) opmapOffset += gpubvh8->triCount * 32; // for N=32: 128 bytes = 32uints
		}
	}
	blasNode2 = new Buffer( node2Count * sizeof( BVH_GPU::BVHNode ) );
	blasIdx = new Buffer( indexCount * sizeof( uint ) );
	blasTri = new Buffer( triCount * sizeof( float4 ) * 3 );
	blasNode8 = new Buffer( node8Count * 80 );
	blasTri8 = new Buffer( tri8Count * 64 );
	blasOpMap = new Buffer( (opmapOffset > 0 ? opmapOffset : 32) * 4 );
	blasFatTri = new Buffer( triCount * sizeof( FatTri ) );
	node2Count = 0, node8Count = 0, indexCount = 0, triCount = 0, tri8Count = 0, opmapOffset = 0;
	for (int i = 0; i < Scene::meshPool.size(); i++)
	{
		// a BLAS needs BVH nodes, triangle indices and the actual triangle data.
		Mesh* mesh = Scene::meshPool[i];
		if (!mesh) continue;
		if (mesh->blas.bvhType == GPU_DYNAMIC)
		{
			BVH_GPU* gpubvh2 = mesh->blas.dynamicGPU;
			memcpy( (BVH_GPU::BVHNode*)blasNode2->GetHostPtr() + node2Count, gpubvh2->bvhNode, gpubvh2->usedNodes * sizeof( BVH_GPU::BVHNode ) );
			memcpy( (uint*)blasIdx->GetHostPtr() + indexCount, gpubvh2->bvh.primIdx, gpubvh2->idxCount * sizeof( uint ) );
			memcpy( (float4*)blasTri->GetHostPtr() + triCount * 3, gpubvh2->bvh.verts.data, gpubvh2->triCount * sizeof( float4 ) * 3 );
			memcpy( (FatTri*)blasFatTri->GetHostPtr() + triCount, mesh->triangles.data(), gpubvh2->triCount * sizeof( FatTri ) );
			if (gpubvh2->opmap) memcpy( (uint32_t*)blasOpMap->GetHostPtr() + opmapOffset * 32, gpubvh2->opmap, gpubvh2->triCount * 128 );
			node2Count += gpubvh2->usedNodes, indexCount += gpubvh2->idxCount, triCount += gpubvh2->triCount;
			if (gpubvh2->opmap) opmapOffset += gpubvh2->triCount * 32; // for N=32: 128 bytes = 32uints
		}
		else
		{
			BVH8_CWBVH* gpubvh8 = mesh->blas.rigidGPU;
			memcpy( (bvhvec4*)blasNode8->GetHostPtr() + node8Count * 5, gpubvh8->bvh8Data, gpubvh8->usedNodes * sizeof( BVH_GPU::BVHNode ) );
			memcpy( (float*)blasTri8->GetHostPtr() + tri8Count * 16, gpubvh8->bvh8Tris, gpubvh8->idxCount * 64 );
			memcpy( (FatTri*)blasFatTri->GetHostPtr() + triCount, mesh->triangles.data(), gpubvh8->triCount * sizeof( FatTri ) );
			if (gpubvh8->opmap) memcpy( (uint32_t*)blasOpMap->GetHostPtr() + opmapOffset * 32, gpubvh8->opmap, gpubvh8->triCount * 128 );
			node8Count += gpubvh8->usedNodes, tri8Count + gpubvh8->idxCount;
			if (gpubvh8->opmap) opmapOffset += gpubvh8->triCount * 32; // for N=32: 128 bytes = 32uints
		}
	}
	blasNode2->CopyToDevice();
	blasIdx->CopyToDevice();
	blasTri->CopyToDevice();
	blasNode8->CopyToDevice();
	blasTri8->CopyToDevice();
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
	init->SetArguments( tlasNode, tlasIdx, instances, blasNode2, blasIdx, blasTri, blasNode8, blasTri8, blasOpMap,
		blasFatTri, blasDesc, materials, texels, Scene::sky->width, Scene::sky->height, skyPixels );
	init->Run( 1 );

	// set a suitable camera
	eye = float3( -3.55f, 13.89f, -21.51f );
	view = normalize( float3( 1, -1, 1 ) );
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