struct Ray
{
	float4 O, D, rD; // 48 byte
	float4 hit; // 16 byte
};

struct BVHNode
{
	float4 lmin; // unsigned left in w
	float4 lmax; // unsigned right in w
	float4 rmin; // unsigned triCount in w
	float4 rmax; // unsigned firstTri in w
};

struct Instance
{
	float transform[16];
	float invTransform[16];
	float4 aabbMin;					// w: uint blasIdx
	float4 aabbMax;					// w: uint mask
	uint dummy[8];					// padding to 64 bytes
};

struct FatTri
{
	float u0, u1, u2;				// 12 bytes for layer 0 texture coordinates
	int ltriIdx;					// 4, set only for emissive triangles, used for MIS
	float v0, v1, v2;				// 12 bytes for layer 0 texture coordinates
	uint material;					// 4 bytes for triangle material index
	float4 vN0;						// 12 bytes for vertex0 normal, Nx in w
	float4 vN1;						// 12 bytes for vertex1 normal, Ny in w
	float4 vN2;						// 12 bytes for vertex2 normal, Nz in w
	float4 vertex0;					// vertex 0 position + second layer u0 in w
	float4 vertex1;					// vertex 1 position + second layer u1 in w
	float4 vertex2;					// vertex 2 position + second layer u2 in w
	float4 T;						// 12 bytes for tangent vector, triangle area in w
	float4 B;						// 12 bytes for bitangent vector, inverse area in w
	float4 alpha;					// 'consistent normal interpolation', LOD in w
	float v0_2, v1_2, v2_2;			// 12 bytes for second layer texture coordinates
	float dummy4;					// padding. Total FatTri size: 192 bytes.
};

#define HAS_TEXTURE		1
#define HAS_NMAP		2
struct Material
{
	float4 albedo;					// beware: OpenCL 3-component vectors align to 16 byte.
	uint flags;						// material flags.
	uint offset;					// start of texture data in the texture bitmap buffer.
	uint width, height;				// size of the texture.
	uint normalOffset;				// start of the normal map data in the texture bitmap buffer.
	uint wnormal, hnormal;			// normal map size.
	uint dummy;						// padding; struct size must be a multiple of 16 bytes.
};

// buffers - most data will be accessed as 128-bit values for efficiency.
global struct BVHNode* tlasNodes;	// top-level acceleration structure node data
global uint* tlasIdx;				// tlas index data
global struct Instance* instances;	// instances
global struct BVHNode* blasNodes;	// bottom-level acceleration structure node data
global uint* blasIdx;				// blas index data
global float4* blasTris;			// blas primitive data for intersection: vertices only
global struct FatTri* blasFatTris;	// blas primitive data for shading: full data
global uint* blasOffsets;			// position of individual blas chunks in larger arrays
global struct Material* materials;	// GPUMaterial data, referenced from FatTris
global uint* texels;				// texture data
global uint2 skySize;				// sky dome image data size
global float* skyPixels;			// HDR sky image data, 12 bytes per pixel

// traversal kernels.
#define STACK_SIZE 32
#include "tools.cl"
#include "traverse_bvh2.cl"
#include "traverse_tlas.cl"

void kernel SetRenderData( 
	global struct BVHNode* tlasNodeData, global uint* tlasIdxData,
	global struct Instance* instanceData,
	global struct BVHNode* blasNodeData, global uint* blasIdxData, 
	global float4* blasTriData, global struct FatTri* blasFatTriData, global uint* blasOffsetData,
	global struct Material* materialData, global uint* texelData, 
	uint skyWidth, uint skyHeight, global float* skyData
)
{
	tlasNodes = tlasNodeData;
	tlasIdx = tlasIdxData;
	instances = instanceData;
	blasNodes = blasNodeData;
	blasIdx = blasIdxData;
	blasTris = blasTriData;
	blasFatTris = blasFatTriData;
	blasOffsets = blasOffsetData;
	materials = materialData;
	texels = texelData;
	skySize.x = skyWidth;
	skySize.y = skyHeight;
	skyPixels = skyData;
}

float3 SampleSky( const float3 D )
{
	float p = atan2( D.z, D.x );
	uint u = (uint)(skySize.x * (p + (p < 0 ? PI * 2 : 0)) * INV2PI - 0.5f);
	uint v = (uint)(skySize.y * acos( D.y ) * INVPI - 0.5f);
	uint idx = min( u + v * skySize.x, skySize.x * skySize.y - 1);
	return (float3)( skyPixels[idx * 3], skyPixels[idx * 3 + 1], skyPixels[idx * 3 + 2] );
}

float4 Trace( struct Ray ray )
{
	float3 radiance = (float3)( 0 );
	float3 throughput = (float3)( 1 );
	float3 L = normalize( (float3)( 2, 4, 5 ) );
	for( int depth = 0; depth < 2; depth++ )
	{
		// extend path
		float4 hit = traverse_tlas( ray.O, ray.D, ray.rD, 1e30f );
		if (hit.x == 1e30f) 
		{
			float3 sky = SampleSky( ray.D.xyz );
			radiance += throughput * sky;
			break;
		}

		// gather shading information
		uint idxData = as_uint( hit.w ), primIdx = idxData & 0xffffff, instIdx = idxData >> 24;
		struct Instance* inst = instances + instIdx;
		uint blasIdx = as_uint( inst->aabbMin.w );
		struct FatTri* tri = blasFatTris + blasOffsets[blasIdx * 4 + 2] + primIdx;
		float3 iN = (hit.y * tri->vN1 + hit.z * tri->vN2 + (1 - hit.y - hit.z) * tri->vN0).xyz;
		float3 N = (float3)( tri->vN0.w, tri->vN1.w, tri->vN2.w );
		struct Material* material = materials + tri->material;
		float3 albedo = (float3)(1);
		float tu = hit.y * tri->u1 + hit.z * tri->u2 + (1 - hit.y - hit.z) * tri->u0;
		float tv = hit.y * tri->v1 + hit.z * tri->v2 + (1 - hit.y - hit.z) * tri->v0;
		tu -= floor( tu ), tv -= floor( tv );
		if (material->flags & HAS_TEXTURE)
		{
			int iu = (int)(tu * material->width);
			int iv = (int)(tv * material->height);
			uint pixel = texels[material->offset + iu + iv * material->width];
			albedo = (float3)( (float)((pixel >> 16) & 255), (float)((pixel >> 8) & 255), (float)(pixel & 255) ) * (1.0f / 256.0f);
		}
		else albedo = material->albedo.xyz;
		if (material->flags & HAS_NMAP)
		{
			int iu = (int)(tu * material->wnormal);
			int iv = (int)(tv * material->hnormal);
			uint pixel = texels[material->normalOffset + iu + iv * material->wnormal];
			float3 mN = (float3)( (float)(pixel & 255), (float)((pixel >> 8) & 255), (float)((pixel >> 24) & 255) );
			mN *= 1.0f / 128.0f, mN += -1.0f;
			iN = mN.x * tri->T.xyz + mN.y * tri->B.xyz + mN.z * iN;
		}
		iN = normalize( TransformVector( iN, inst->transform ) );
		N = normalize( TransformVector( N, inst->transform ) );
		if (dot( iN, N ) < 0) iN *= -1;

		// direct light
		radiance += albedo * max( 0.2f, dot( iN, L ) );
		
		// indirect light
		if (depth == 1) break;
		float3 R = ray.D.xyz - 2 * dot( iN, ray.D.xyz ) * iN;
		float3 I = ray.O.xyz + ray.D.xyz * ray.hit.x;
		ray.O = (float4)( I + R * 0.0001f, 1 );
		ray.D = (float4)( R, 0 );
		ray.rD = (float4)(1.0f / R.x, 1.0f / R.y, 1.0f / R.z, 1 );
		throughput *= albedo * dot( R, iN ) * 0.5f;
	}
	return (float4)( radiance, 1.0f );
}

void kernel Render( 
	write_only image2d_t pixels, const uint width, const uint height,
	const float4 eye, const float4 p1, const float4 p2, const float4 p3
)
{
	// extract pixel coordinates from thread id
	const uint x = get_global_id( 0 ), y = get_global_id( 1 );
	const uint pixelIdx = x + y * get_global_size( 0 );
	
	// create primary ray
	struct Ray ray;
	float u = (float)x / width, v = (float)y / height;
	ray.O = eye;
	ray.D = (float4)( normalize( (p1 + u * (p2 - p1) + v * (p3 - p1) - eye).xyz ), 1 );
	ray.rD = (float4)( 1.0f / ray.D.x, 1.0f / ray.D.y, 1.0f / ray.D.z, 1 );

	// evaluate light transport
	write_imagef( pixels, (int2)(x, y), Trace( ray ) );
}