// Template, 2024 IGAD Edition
// Get the latest version from: https://github.com/jbikker/tmpl8
// IGAD/NHTV/BUAS/UU - Jacco Bikker - 2006-2024

#include "precomp.h"
using namespace tinybvh;
using namespace tinyocl;
using namespace tinyscene;
#include "gltfdemo.h"

float3 DiffuseReflection( float3 N )
{
	float3 R;
	do { R = (float3)(RandomFloat() * 2 - 1, RandomFloat() * 2 - 1, RandomFloat() * 2 - 1); } while (dot( R, R ) > 1);
	return normalize( dot( R, N ) > 0 ? R : -R );
}
float3 CosWeightedDiffReflection( const float3 N )
{
	float r1 = RandomFloat(), r0 = RandomFloat();
	const float r = sqrt( 1 - r1 * r1 ), phi = 4 * PI * r0;
	const float3 R = (float3)(cos( phi ) * r, sin( phi ) * r, r1);
	return normalize( N + R );
}
float3 SampleSky( const float3 D )
{
	float p = atan2( D.z, D.x );
	uint u = (uint)(Scene::sky->width * (p + (p < 0 ? PI * 2 : 0)) * INV2PI - 0.5f);
	uint v = (uint)(Scene::sky->height * acos( D.y ) * INVPI - 0.5f);
	uint idx = min( u + v * Scene::sky->width, (uint)(Scene::sky->width * Scene::sky->height - 1) );
	return Scene::sky->pixels[idx];
}

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
	uint triOffset;				// position of blas triangle data in global array
	uint fatTriOffset;			// position of blas FatTri data in global array
	uint opmapOffset;			// position of opacity micromap data in global arrays
	uint node8Offset;			// position of CWBVH nodes in global array
	uint tri8Offset;			// position of CWBVH triangle data in global array
	uint blasType;				// blas type: 0 = BVH_GPU, 1 = BVH8_CWBVH
};

// camera path.
static float3 cam[] = {
	float3( 48.42, 3.94, -11.66 ), float3( 47.43, 3.79, -11.55 ),
	float3( 36.81, 2.14, -12.36 ), float3( 35.84, 1.99, -12.17 ),
	float3( 19.47, 0.67, -12.98 ), float3( 18.50, 0.51, -12.78 ),
	float3( -1.78, -0.80, -8.88 ), float3( -2.76, -0.93, -8.76 ),
	float3( -33.32, 2.16, -2.20 ), float3( -32.43, 1.93, -1.80 ),
	float3( -33.22, 4.81, 9.82 ), float3( -32.26, 4.58, 9.66 ),
	float3( -26.91, 4.81, 17.93 ), float3( -26.55, 4.59, 17.02 ),
	float3( -2.19, 4.81, 19.07 ), float3( -2.19, 4.59, 18.10 ),
	float3( 21.82, 4.81, 15.33 ), float3( 21.57, 4.59, 14.39 ),
	float3( 40.44, 4.81, -5.08 ), float3( 39.57, 4.34, -5.02 ),
	float3( 33.39, 4.81, -17.78 ), float3( 32.86, 4.29, -17.11 ),
	float3( 28.13, 3.45, -18.17 ), float3( 27.95, 2.93, -17.33 ),
	float3( 20.24, 3.45, -17.62 ), float3( 20.39, 2.94, -16.78 ),
	float3( 15.61, 1.06, -8.83 ), float3( 16.44, 0.60, -8.50 ),
	float3( 17.39, 0.52, -3.10 ), float3( 18.07, 0.12, -3.72 ),
	float3( 29.60, 2.18, -4.33 ), float3( 30.20, 1.71, -4.98 ),
	float3( 32.83, 18.25, -5.91 ), float3( 33.32, 17.55, -6.44 ),
	float3( 62.90, 28.35, -21.68 ), float3( 62.26, 27.67, -21.32 ),
	float3( 40.17, 34.49, -53.01 ), float3( 39.84, 33.82, -52.35 ),
	float3( -1.97, 31.37, -52.33 ), float3( -1.89, 30.71, -51.59 ),
	float3( -32.71, 22.84, -23.49 ), float3( -32.05, 22.21, -23.10 ),
	float3( -30.08, 9.19, -3.13 ), float3( -29.19, 8.74, -3.28 ),
};
static float st = 1;
static int mode = 0;

// -----------------------------------------------------------
// Initialize the application
// -----------------------------------------------------------
void GLTFDemo::Init()
{

}

// -----------------------------------------------------------
// Init stage 2: Scene loading
// -----------------------------------------------------------
void GLTFDemo::InitScene1()
{
	// load gltf scene
	scene.SetBVHDefault( GPU_RIGID ); // even the drone does not use BVH rebuilds.
	scene.CacheBVHs(); // BVHs will be saved to disk for faster loading and optimization.
	terrain = scene.AddScene( "./testdata/cratercity/scene.gltf", mat4::Translate( 0, -18.9f, 0 ) * mat4::RotateY( 1 ) );
	tree1 = scene.AddScene( "./testdata/mangotree/scene.gltf", mat4::Translate( 5, -3.5f, 0 ) * mat4::Scale( 2 ) );
	tree2 = scene.AddScene( "./testdata/smallpine/scene.gltf", mat4::Translate( 0, 0, 0 ) * mat4::Scale( 0.03f ) );
	balloon = scene.AddScene( "./testdata/balloon/scene.gltf", mat4::Translate( 10, 10, 10 ) * mat4::Scale( 18 ) );
	drone = scene.AddScene( "./testdata/drone/scene.gltf", mat4::Translate( 21.5f, -1.87f, -7 ) * mat4::Scale( 0.03f ) * mat4::RotateY( PI * 1.5f ) );
	scene.SetSkyDome( new SkyDome( "./testdata/sky_15.hdr" ) );
};

// -----------------------------------------------------------
// Init stage 2: Scene processing
// -----------------------------------------------------------
void GLTFDemo::InitScene2()
{
	scene.CollapseMeshes( tree1 );
	scene.CollapseMeshes( balloon );
	scene.CreateOpacityMicroMaps( tree1 );
	int treeMesh = scene.CollapseMeshes( tree2 );
	scene.CreateOpacityMicroMaps( tree2 );
	int terrainMesh = scene.CollapseMeshes( terrain ); // combine the meshes into a single mesh; may yield a better BVH.
	int terrainNode = scene.FindMeshNode( terrain /* possibly a hierarchy */, terrainMesh );
	scene.SetBVHType( drone, GPU_DYNAMIC );
	scene.SetBVHType( terrain, GPU_STATIC );
	printf( "building BVHs...\n" );
	scene.UpdateSceneGraph( 0 ); // this will build the BLASses and TLAS.
	printf( "all done.\n" );
	// place a rough circle of trees
	printf( "populating terrain... " );
	mat4 invTerrain = scene.nodePool[terrainNode]->combinedTransform.Inverted();
	for (float a = 0; a < 2 * PI; a += 0.045f + RandomFloat() * 0.03f)
	{
		int nodeId = scene.AddNode( new tinyscene::Node( treeMesh, mat4::Identity() ) );
		mat4 T1 = mat4::Translate( (30 + 2 * RandomFloat()) * sinf( a ), 0, (30 + 2 * RandomFloat()) * cosf( a ) );
		float3 O( T1[3], 20, T1[11] ), D( 0, -1, 0 );
		Ray r( float4( O, 1 ) * invTerrain, float4( D, 0 ) * invTerrain );
		scene.meshPool[terrainMesh]->blas.staticGPU->bvh8.bvh.Intersect( r );
		T1[7] = (O + D * r.hit.t * 0.1f /* erm... why 0.1? */).y - 0.2f;
		float hsize = 0.03f + RandomFloat() * 0.025f, vsize = 0.03f + RandomFloat() * 0.015f;
		mat4 T2 = mat4::Scale( float3( hsize, vsize, hsize ) );
		mat4 T3 = mat4::RotateY( RandomFloat() * TWOPI ) * mat4::RotateX( PI * 1.5f );
		scene.SetNodeTransform( nodeId, T1 * T2 * T3 );
		scene.AddInstance( nodeId );
	}
	printf( "done. Updating TLAS... " );
	scene.UpdateSceneGraph( 0 ); // this will build the BLASses and TLAS.
	printf( "done.\n" );

	// create OpenCL kernels
	init = new Kernel( "raytracer.cl", "SetRenderData" );
	render = new Kernel( "raytracer.cl", "Render" );
	renderNormals = new Kernel( "raytracer.cl", "RenderNormals" );
	renderDepth = new Kernel( "raytracer.cl", "RenderDepth" );

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
	uint node2Count = 0, block8Count = 0, indexCount = 0, triCount = 0, fatTriCount = 0, tri8Count = 0, opmapOffset = 0;
	blasDesc = new Buffer( (int)Scene::meshPool.size() * sizeof( BLASDesc ) );
	for (int i = 0; i < Scene::meshPool.size(); i++)
	{
		Mesh* mesh = Scene::meshPool[i];
		if (!mesh) continue;
		BLASDesc& desc = ((BLASDesc*)blasDesc->GetHostPtr())[i];
		desc.indexOffset = indexCount;
		desc.triOffset = triCount;
		desc.fatTriOffset = fatTriCount;
		desc.blasType = mesh->blas.bvhType;
		if (mesh->blas.bvhType == GPU_DYNAMIC || mesh->blas.bvhType == GPU_RIGID)
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
			BVH8_CWBVH* gpubvh8 = mesh->blas.staticGPU;
			desc.node8Offset = block8Count;
			desc.tri8Offset = tri8Count;
			desc.opmapOffset = gpubvh8->opmap ? opmapOffset : 0x99999999;
			block8Count += gpubvh8->usedBlocks;
			tri8Count += gpubvh8->idxCount; // not triCount; leafs store tris by value and we may have spatial splits.
			// if (gpubvh8->opmap) opmapOffset += gpubvh8->triCount * 32; // for N=32: 128 bytes = 32uints
		}
		fatTriCount += (int)mesh->triangles.size();
	}
	blasNode2 = new Buffer( node2Count * sizeof( BVH_GPU::BVHNode ) );
	blasIdx = new Buffer( indexCount * sizeof( uint ) );
	blasTri = new Buffer( triCount * sizeof( float4 ) * 3 );
	blasNode8 = new Buffer( block8Count * 16 );
	blasTri8 = new Buffer( tri8Count * 64 );
	blasOpMap = new Buffer( (opmapOffset > 0 ? opmapOffset : 32) * 4 );
	blasFatTri = new Buffer( fatTriCount * sizeof( FatTri ) );
	node2Count = 0, block8Count = 0, indexCount = 0, triCount = 0, tri8Count = 0, fatTriCount = 0, opmapOffset = 0;
	for (int i = 0; i < Scene::meshPool.size(); i++)
	{
		// a BLAS needs BVH nodes, triangle indices and the actual triangle data.
		Mesh* mesh = Scene::meshPool[i];
		if (!mesh) continue;
		if (mesh->blas.bvhType == GPU_DYNAMIC || mesh->blas.bvhType == GPU_RIGID)
		{
			BVH_GPU* gpubvh2 = mesh->blas.dynamicGPU;
			memcpy( (BVH_GPU::BVHNode*)blasNode2->GetHostPtr() + node2Count, gpubvh2->bvhNode, gpubvh2->usedNodes * sizeof( BVH_GPU::BVHNode ) );
			memcpy( (uint*)blasIdx->GetHostPtr() + indexCount, gpubvh2->bvh.primIdx, gpubvh2->idxCount * sizeof( uint ) );
			memcpy( (float4*)blasTri->GetHostPtr() + triCount * 3, gpubvh2->bvh.verts.data, gpubvh2->triCount * sizeof( float4 ) * 3 );
			memcpy( (FatTri*)blasFatTri->GetHostPtr() + fatTriCount, mesh->triangles.data(), mesh->triangles.size() * sizeof( FatTri ) );
			if (gpubvh2->opmap) memcpy( (uint32_t*)blasOpMap->GetHostPtr() + opmapOffset, gpubvh2->opmap, gpubvh2->triCount * 128 );
			node2Count += gpubvh2->usedNodes, indexCount += gpubvh2->idxCount, triCount += gpubvh2->triCount;
			if (gpubvh2->opmap) opmapOffset += gpubvh2->triCount * 32; // for N=32: 128 bytes = 32uints
		}
		else
		{
			BVH8_CWBVH* gpubvh8 = mesh->blas.staticGPU;
			memcpy( (bvhvec4*)blasNode8->GetHostPtr() + block8Count, gpubvh8->bvh8Data, gpubvh8->usedBlocks * 16 );
			memcpy( (float*)blasTri8->GetHostPtr() + tri8Count * 16, gpubvh8->bvh8Tris, gpubvh8->idxCount * 64 );
			memcpy( (FatTri*)blasFatTri->GetHostPtr() + fatTriCount, mesh->triangles.data(), mesh->triangles.size() * sizeof( FatTri ) );
			// if (gpubvh8->opmap) memcpy( (uint32_t*)blasOpMap->GetHostPtr() + opmapOffset, gpubvh8->opmap, gpubvh8->triCount * 128 );
			block8Count += gpubvh8->usedBlocks, tri8Count + gpubvh8->idxCount;
			// if (gpubvh8->opmap) opmapOffset += gpubvh8->triCount * 32; // for N=32: 128 bytes = 32uints
		}
		fatTriCount += (int)mesh->triangles.size();
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

	// 8. IBL data.
	IBL = new Buffer( 16 * 16 * 16 * sizeof( float4 ) );
	FILE* f = fopen( "./cache/ibl.dat", "rb" );
	if (f) fread( IBL->GetHostPtr(), 1, 16 * 16 * 16 * 16, f ); else
	{
		for (int z = 0; z < 16; z++)
		{
			float fz = z - 7.5f;
			for (int y = 0; y < 16; y++)
			{
				float fy = y - 7.5f;
				for (int x = 0; x < 16; x++)
				{
					float fx = x - 7.5f;
					float3 D = normalize( float3( fx, fy, fz ) );
					float3 sum( 0 );
					for (int i = 0; i < 128; i++)
					{
						float3 R = CosWeightedDiffReflection( D );
						sum += SampleSky( R );
					}
					((float4*)IBL->GetHostPtr())[x + y * 16 + z * 256] = float4( sum * 1.0f / 128, 1 );
				}
			}
		}
		f = fopen( "./cache/ibl.dat", "wb" );
		fwrite( IBL->GetHostPtr(), 1, 16 * 16 * 16 * 16, f );
	}
	fclose( f );
	IBL->CopyToDevice();

	// pass all static data to the Init function.
	init->SetArguments( tlasNode, tlasIdx, instances, blasNode2, blasIdx, blasTri, blasNode8, blasTri8, blasOpMap,
		blasFatTri, blasDesc, materials, texels, Scene::sky->width, Scene::sky->height, skyPixels, IBL );
	init->Run( 1 );

	// set a suitable camera
	eye = float3( -6.55f, 13.89f, -18.51f );
	view = normalize( float3( 1, -1, 1 ) );
}

// -----------------------------------------------------------
// Init stage 2: Scene finalizing
// -----------------------------------------------------------
void GLTFDemo::InitScene3()
{
}

// -----------------------------------------------------------
// Update camera
// -----------------------------------------------------------
bool GLTFDemo::UpdateCamera( float delta_time_s )
{
	bvhvec3 right = tinybvh_normalize( tinybvh_cross( bvhvec3( 0, 1, 0 ), view ) );
	bvhvec3 up = 0.8f * tinybvh_cross( view, right );
	bool moved = false;
#if 0
	// get camera controls.
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
	static bool Pdown = false;
	if (!GetAsyncKeyState( 'P' )) Pdown = false; else
	{
		if (!Pdown) // save point for path.
		{
			FILE* f = fopen( "spline.txt", "a" );
			float3 P = eye + view;
			fprintf( f, "	float3( %.2f, %.2f, %.2f ), float3( %.2f, %.2f, %.2f ),\n", eye.x, eye.y, eye.z, P.x, P.y, P.z );
			fclose( f );
		}
		Pdown = true;
	}
#else
	// update position on spline path.
	st += delta_time_s * 0.2f;
	if (st > sizeof( cam ) / 24 - 3) st = 1, mode = 0;
	float t = st;
	static int ps = 0;
	int s = (uint)t;
	if (s != ps) printf( "%i\n", s );
	ps = s;
	t -= (float)s, s *= 2;
	const float3 Pp = cam[s - 2], Qp = cam[s], Rp = cam[s + 2], Sp = cam[s + 4];
	const float3 Pt = cam[s - 1], Qt = cam[s + 1], Rt = cam[s + 3], St = cam[s + 5];
	float3 a = 2.0f * Qp, b = Rp - Pp, c = 2.0f * Pp - 5.0f * Qp + 4.0f * Rp - Sp;
	eye = 0.5f * (a + (b * t) + (c * t * t) + ((3.0f * Qp - 3.0f * Rp + Sp - Pp) * t * t * t));
	a = 2.0f * Qt, b = Rt - Pt, c = 2.0f * Pt - 5.0f * Qt + 4.0f * Rt - St;
	float3 target = 0.5f * (a + (b * t) + (c * t * t) + ((3.0f * Qt - 3.0f * Rt + St - Pt) * t * t * t));
	view = normalize( target - float3( eye ) );
#endif
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
	// initialize: show logo first
	static int initseq = 0;
	if (initseq < 5)
	{
		screen->Clear( 0 );
		initseq++;
		return;
	}
	else if (initseq < 8)
	{
		static Surface s( "testdata/tinybvh.png" );
		switch (initseq)
		{
		case 5:
			InitScene1();
			break;
		case 6:
			s.CopyTo( screen, (SCRWIDTH - s.width) / 2, (SCRHEIGHT - s.height) / 2 - 20 );
			break;
		case 7:
			screen = 0; // this tells the template to not overwrite the render target.
			InitScene2();
			break;
		}
		initseq++;
		return;
	}

	// handle user input and update camera.
	UpdateCamera( delta_time * 0.001f );
	static float a = 2;
	scene.SetNodeTransform( balloon, mat4::Translate( 17 * sinf( a ), 2 - 2 * sinf( a ), 21 * cosf( a ) ) * mat4::Scale( 18 ) ); // mat4::Translate( float3( sinf( a ), 10, cosf( a ) ) * 20.f ) * mat4::Scale( 18 ) );
	a += 0.00003f * delta_time;
	if (a > TWOPI) a -= TWOPI;
	scene.UpdateSceneGraph( delta_time * 0.001f );

	// reset animation at keyframe 7.
	static int rt = 0, mt = 0;
	int it = (int)st;
	if (it != 7 && it != 16 && it != 1) rt = 0; else { if (rt != it) scene.ResetAnimations(); rt = it; }
	if (it != 8 && it != 9 && it != 10 && it != 17 && it != 18 && it != 19) mt = 0; else if (mt != it)
	{
		mode = (mode + 1) % 3;
		if (it == 10) scene.ResetAnimations();
		mt = it;
	}

	// sync tlas - TODO: would be better if this could be a single copy.
	tlasNode->CopyToDevice();
	tlasIdx->CopyToDevice();
	instances->CopyToDevice();

	// start device-side rendering.
	static bool altDown = false;
	if (!GetAsyncKeyState( VK_TAB )) altDown = false; else
	{
		if (!altDown) altDown = false, mode = (mode + 1) % 3;
		altDown = true;
	}
	if (mode == 0)
	{
		// full rendering
		render->SetArguments( pixels, SCRWIDTH, SCRHEIGHT, eye, p1, p2, p3 );
		render->Run2D( oclint2( SCRWIDTH, SCRHEIGHT ) );
	}
	else if (mode == 1)
	{
		// BVH structure
		renderDepth->SetArguments( pixels, SCRWIDTH, SCRHEIGHT, eye, p1, p2, p3 );
		renderDepth->Run2D( oclint2( SCRWIDTH, SCRHEIGHT ) );
	}
	else // if (mode == 2)
	{
		// only normals
		renderNormals->SetArguments( pixels, SCRWIDTH, SCRHEIGHT, eye, p1, p2, p3 );
		renderNormals->Run2D( oclint2( SCRWIDTH, SCRHEIGHT ) );
	}

#if 0
	// print camera aim pos
	static int delay = 10;
	if (--delay == 0)
	{
		printf( "x = %.2f, y = %.2f, z = %.2f)\n", eye.x, eye.y, eye.z );
		delay = 10;
	}
#endif
}