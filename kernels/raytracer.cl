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

struct BLASDesc
{
	uint nodeOffset;				// position of blas node data in global blasNodes array
	uint indexOffset;				// position of blas index data in global blasIdx array
	uint triOffset;					// position of blas triangle data in global array
	uint fatTriOffset;				// position of blas FatTri data in global array
	uint opmapOffset;				// position of opacity micromap data in global arrays
	uint node8Offset;				// position of CWBVH nodes in global array
	uint tri8Offset;				// position of CWBVH triangle data in global array
	uint blasType;					// blas type: 0 = BVH_GPU, 1 = BVH8_CWBVH
};

// buffers - most data will be accessed as 128-bit values for efficiency.
global struct BVHNode* tlasNodes;	// top-level acceleration structure node data
global uint* tlasIdx;				// tlas index data
global struct Instance* instances;	// instances
global struct BVHNode* blasNodes;	// bottom-level acceleration structure node data
global uint* blasIdx;				// blas index data
global float4* blasTris;			// blas primitive data for intersection: vertices only
global float4* blasCWNodes;			// CWBVH blas node data
global float4* blasTri8;			// CWBVH triangle data
global uint* blasOpMap;				// opacity maps for BVHs with alpha mapped textures
global struct FatTri* blasFatTris;	// blas primitive data for shading: full data
global struct BLASDesc* blasDesc;	// blas descriptor data: blas type & position of chunks in larger arrays
global struct Material* materials;	// GPUMaterial data, referenced from FatTris
global uint* texels;				// texture data
global uint2 skySize;				// sky dome image data size
global float* skyPixels;			// HDR sky image data, 12 bytes per pixel
global float4* IBL;					// IBL data

// Low-level CWBVH traversal optimizations for specific platforms
#ifdef ISINTEL // Iris Xe, Arc, ..
// #define USE_VLOAD_VSTORE
#define SIMD_AABBTEST
#elif defined ISNVIDIA // 2080, 3080, 4080, ..
#define USE_VLOAD_VSTORE
// #define SIMD_AABBTEST
#elif defined ISAMD
#define USE_VLOAD_VSTORE
// #define SIMD_AABBTEST
#else // unkown GPU
// #define USE_VLOAD_VSTORE
#define SIMD_AABBTEST
#endif

// traversal kernels.
#define STACK_SIZE 32
#include "tools.cl"
#include "traverse_bvh2.cl"
#include "traverse_cwbvh.cl"
 #include "traverse_tlas.cl"

void kernel SetRenderData(
	global struct BVHNode* tlasNodeData, global uint* tlasIdxData,
	global struct Instance* instanceData,
	global struct BVHNode* blasNodeData, global uint* blasIdxData, global float4* blasTriData, 
	global float4* blasCWNodeData, global float4* blasTri8Data,
	global uint* blasOpMapData, global struct FatTri* blasFatTriData, global struct BLASDesc* blasDescData,
	global struct Material* materialData, global uint* texelData,
	uint skyWidth, uint skyHeight, global float* skyData, global float4* IBLData
)
{
	tlasNodes = tlasNodeData;
	tlasIdx = tlasIdxData;
	instances = instanceData;
	blasNodes = blasNodeData;
	blasIdx = blasIdxData;
	blasTris = blasTriData;
	blasCWNodes = blasCWNodeData;
	blasTri8 = blasTri8Data;
	blasOpMap = blasOpMapData;
	blasFatTris = blasFatTriData;
	blasDesc = blasDescData;
	materials = materialData;
	texels = texelData;
	skySize.x = skyWidth;
	skySize.y = skyHeight;
	skyPixels = skyData;
	IBL = IBLData;
}

float3 SampleSky( const float3 D )
{
	float p = atan2( D.z, D.x );
	uint u = (uint)(skySize.x * (p + (p < 0 ? PI * 2 : 0)) * INV2PI - 0.5f);
	uint v = (uint)(skySize.y * acos( D.y ) * INVPI - 0.5f);
	uint idx = min( u + v * skySize.x, skySize.x * skySize.y - 1 );
	return (float3)(skyPixels[idx * 3], skyPixels[idx * 3 + 1], skyPixels[idx * 3 + 2]);
}

float3 SampleIBL( const float3 D )
{
	// diffuse IBL: sample precalculated cosine-weighted hemisphere integral as an 
	// approximation of the light from the skydome reflected by a fragment with normal D.
	float x = (D.x + 1) * 7.49f, fx = x - floor( x );
	float y = (D.y + 1) * 7.49f, fy = y - floor( y );
	float z = (D.z + 1) * 7.49f, fz = z - floor( z );
	int ix0 = (int)x, iy0 = (int)y, iz0 = (int)z;
	int ix1 = (ix0 + 1) & 15, iy1 = (iy0 + 1) & 15, iz1 = (iz0 + 1) & 15;
	float4 p0 = IBL[ix0 + iy0 * 16 + iz0 * 16 * 16], p1 = IBL[ix1 + iy0 * 16 + iz0 * 16 * 16];
	float4 p2 = IBL[ix0 + iy1 * 16 + iz0 * 16 * 16], p3 = IBL[ix1 + iy1 * 16 + iz0 * 16 * 16];
	float4 p4 = IBL[ix0 + iy0 * 16 + iz1 * 16 * 16], p5 = IBL[ix1 + iy0 * 16 + iz1 * 16 * 16];
	float4 p6 = IBL[ix0 + iy1 * 16 + iz1 * 16 * 16], p7 = IBL[ix1 + iy1 * 16 + iz1 * 16 * 16];
	float w0 = (1 - fx) * (1 - fy), w1 = fx * (1 - fy), w2 = (1 - fx) * fy, w3 = 1 - (w0 + w1 + w2);
	float4 pa = p0 * w0 + p1 * w1 + p2 * w2 + p3 * w3;
	float4 pb = p4 * w0 + p5 * w1 + p6 * w2 + p7 * w3;
	return ((1 - fz) * pa + fz * pb).xyz;
}

float3 RayTarget( const float u, const float v, const float distortion,
	const float3 p1, const float3 p2, const float3 p3,
	const uint width, const uint height ) // From LH2
{
	float3 posOnPixel;
#if 0
	// no Panini
	posOnPixel = p1 + u * (p2 - p1) + v * (p3 - p1);
#else
	// Panini projection, from: https://www.shadertoy.com/view/Wt3fzB
	const float tx = u - 0.499f, ty = v - 0.499f;
	const float rr = tx * tx + ty * ty;
	const float rq = sqrt( rr ) * (1.0f + distortion * rr + distortion * rr * rr);
	const float theta = atan2( tx, ty );
	const float bx = (sin( theta ) * rq + 0.5f) * width;
	const float by = (cos( theta ) * rq + 0.5f) * height;
	// posOnPixel = p1 + (bx + r0) * (right / (float)scrSize.x) + (by + r1) * (up / (float)scrSize.y);
	float r0 = 0, r1 = 0;
	float3 right = p2 - p1;
	float3 up = p3 - p1;
	posOnPixel = p1 + (bx + r0) * (right / width) + (by + r1) * (up / height);
#endif
	return posOnPixel;
}

float4 Trace( struct Ray ray )
{
	// hardcoded directional light.
	float3 L = normalize( (float3)(-5, 1.5f, -1) );
	float3 suncol = (float3)( 1.1f, 1.1f, 1.0f );
	float3 rL = (float3)(1.0f / L.x, 1.0f / L.y, 1.0f / L.z);

	// shading data
	float tu, tv;
	float4 hit;
	float3 N, iN, albedo, I;
	global struct Material* material;
	global struct FatTri* tri;
	global struct Instance* inst;
	uint blasIdx, idxData, primIdx, instIdx;

	// find a non-translucent texel in max. 2 steps.
	for (int step = 0; step < 2; step++)
	{
		// extend path
		hit = traverse_tlas( ray.O, ray.D, ray.rD, 1000.0f, 0 );
		if (hit.x > 999) return (float4)( SampleSky( ray.D.xyz ), 1 );

		// gather shading information
		idxData = as_uint( hit.w ), primIdx = idxData & 0xffffff, instIdx = idxData >> 24;
		inst = instances + instIdx;
		blasIdx = as_uint( inst->aabbMin.w );
		tri = blasFatTris + blasDesc[blasIdx].fatTriOffset + primIdx;
		iN = (hit.y * tri->vN1 + hit.z * tri->vN2 + (1 - hit.y - hit.z) * tri->vN0).xyz;
		N = (float3)(tri->vN0.w, tri->vN1.w, tri->vN2.w);
		I = ray.O.xyz + ray.D.xyz * hit.x;
		material = materials + tri->material;
		albedo = (float3)(1);
		tu = hit.y * tri->u1 + hit.z * tri->u2 + (1 - hit.y - hit.z) * tri->u0;
		tv = hit.y * tri->v1 + hit.z * tri->v2 + (1 - hit.y - hit.z) * tri->v0;
		tu -= floor( tu ), tv -= floor( tv );
		bool validAlbedo = true;
		if (material->flags & HAS_TEXTURE)
		{
			int iu = (int)(tu * material->width);
			int iv = (int)(tv * material->height);
			uint pixel = texels[material->offset + iu + iv * material->width];
			albedo = (float3)((float)(pixel & 255), (float)((pixel >> 8) & 255), (float)((pixel >> 16) & 255)) * (1.0f / 256.0f);
			if ((pixel >> 24) < 2) validAlbedo = false;
		}
		else albedo = material->albedo.xyz;
		if (validAlbedo) break;

		// texture pixel was translucent; extend ray.
		ray.O = (float4)(I, 1) + ray.D * 0.001f;
	}

	// fog layer
	float3 fogColor = (float3)( 0.6f, 0.52f, 0.35f );
	float fog = 0, layer = -1.5f;
	float t = (layer - ray.O.y) * ray.rD.y;
	if (ray.O.y > layer)
	{
		if (t > 0 && t < hit.x) fog = 1 - exp( (t - hit.x) * 0.04f );
	}
	else
	{
		if (t > 0) t = min( hit.x, t ); else t = hit.x;
		fog = 1 - exp( -t * 0.03f );
	}
	
	// finalize shading
	if (material->flags & HAS_NMAP)
	{
		int iu = (int)(tu * material->wnormal);
		int iv = (int)(tv * material->hnormal);
		uint pixel = texels[material->normalOffset + iu + iv * material->wnormal];
		float3 mN = (float3)((float)(pixel & 255), (float)((pixel >> 8) & 255), (float)((pixel >> 24) & 255));
		mN *= 1.0f / 128.0f, mN += -1.0f;
		iN = mN.x * tri->T.xyz + mN.y * tri->B.xyz + mN.z * iN;
	}
	N = normalize( TransformVector( N, inst->transform ) );
	if (dot( N, ray.D.xyz ) > 0) N *= -1;
	iN = normalize( TransformVector( iN, inst->transform ) );
	if (dot( iN, N ) < 0) iN *= -1;

	// direct light
	bool shaded = false;
	if (dot( rL, iN ) > 0) 
		shaded = isoccluded_tlas( (float4)(I + L * 0.001f, 1), (float4)(L, 1), (float4)(rL, 1), 1000 );
	float3 ibl = SampleIBL( iN );
	float3 irr = ibl * 0.7f;
	if (!shaded) irr += 0.7f * max( 0.0f, dot( iN, L ) ) * suncol;
	float3 radiance = (1 - fog) * albedo * irr + fog * fogColor;		
	return (float4)(radiance, 1.0f);
}

float4 TraceNormals( struct Ray ray )
{
	// shading data
	float tu, tv;
	float4 hit;
	float3 N, iN, albedo, I;
	global struct Material* material;
	global struct FatTri* tri;
	global struct Instance* inst;
	uint blasIdx, idxData, primIdx, instIdx;

	// find a non-translucent texel in max. 2 steps.
	for (int step = 0; step < 2; step++)
	{
		// extend path
		hit = traverse_tlas( ray.O, ray.D, ray.rD, 1000.0f, 0 );
		if (hit.x > 999) return (float4)( 0 );

		// gather shading information
		idxData = as_uint( hit.w ), primIdx = idxData & 0xffffff, instIdx = idxData >> 24;
		inst = instances + instIdx;
		blasIdx = as_uint( inst->aabbMin.w );
		tri = blasFatTris + blasDesc[blasIdx].fatTriOffset + primIdx;
		iN = (hit.y * tri->vN1 + hit.z * tri->vN2 + (1 - hit.y - hit.z) * tri->vN0).xyz;
		N = (float3)(tri->vN0.w, tri->vN1.w, tri->vN2.w);
		I = ray.O.xyz + ray.D.xyz * hit.x;
		material = materials + tri->material;
		albedo = (float3)(1);
		tu = hit.y * tri->u1 + hit.z * tri->u2 + (1 - hit.y - hit.z) * tri->u0;
		tv = hit.y * tri->v1 + hit.z * tri->v2 + (1 - hit.y - hit.z) * tri->v0;
		tu -= floor( tu ), tv -= floor( tv );
		bool validAlbedo = true;
		if (material->flags & HAS_TEXTURE)
		{
			int iu = (int)(tu * material->width);
			int iv = (int)(tv * material->height);
			uint pixel = texels[material->offset + iu + iv * material->width];
			albedo = (float3)((float)(pixel & 255), (float)((pixel >> 8) & 255), (float)((pixel >> 16) & 255)) * (1.0f / 256.0f);
			if ((pixel >> 24) < 2) validAlbedo = false;
		}
		else albedo = material->albedo.xyz;
		if (validAlbedo) break;

		// texture pixel was translucent; extend ray.
		ray.O = (float4)(I, 1) + ray.D * 0.001f;
	}

	// finalize shading
	if (material->flags & HAS_NMAP)
	{
		int iu = (int)(tu * material->wnormal);
		int iv = (int)(tv * material->hnormal);
		uint pixel = texels[material->normalOffset + iu + iv * material->wnormal];
		float3 mN = (float3)((float)(pixel & 255), (float)((pixel >> 8) & 255), (float)((pixel >> 24) & 255));
		mN *= 1.0f / 128.0f, mN += -1.0f;
		iN = mN.x * tri->T.xyz + mN.y * tri->B.xyz + mN.z * iN;
	}
	N = normalize( TransformVector( N, inst->transform ) );
	if (dot( N, ray.D.xyz ) > 0) N *= -1;
	iN = normalize( TransformVector( iN, inst->transform ) );
	if (dot( iN, N ) < 0) iN *= -1;

	return (float4)( iN, 1 );
}

uint TraceDepth( struct Ray ray )
{
	uint steps;
	float4 hit = traverse_tlas( ray.O, ray.D, ray.rD, 1000.0f, &steps );
	return steps;
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
	ray.D = (float4)( RayTarget( u, v, 0.15f, p1.xyz, p2.xyz, p3.xyz, width, height ) - eye.xyz, 1 );
	ray.rD = (float4)(1.0f / ray.D.x, 1.0f / ray.D.y, 1.0f / ray.D.z, 1);

	// evaluate light transport
	float4 E = Trace( ray );

	// postprocessing
	float3 A = aces( E.xyz );
	float2 uv = (float2)( u * (1 - u), v * (1 - v) );
	float vig = 0.75f + 0.25f * pow( uv.x * uv.y * 15, 0.15f );

	// write pixel
	write_imagef( pixels, (int2)(x, y), (float4)( vig * A, 1 ) );
}

void kernel RenderNormals(
	write_only image2d_t pixels, const uint width, const uint height,
	const float4 eye, const float4 p1, const float4 p2, const float4 p3
)
{
	// extract pixel coordinates from thread id
	const uint x = get_global_id( 0 ), y = get_global_id( 1 );
	const uint pixelIdx = x + y * get_global_size( 0 );

	// create primary ray and trace
	struct Ray ray;
	float u = (float)x / width, v = (float)y / height;
	ray.O = eye;
	ray.D = (float4)( RayTarget( u, v, 0.15f, p1.xyz, p2.xyz, p3.xyz, width, height ) - eye.xyz, 1 );
	ray.rD = (float4)(1.0f / ray.D.x, 1.0f / ray.D.y, 1.0f / ray.D.z, 1);
	write_imagef( pixels, (int2)(x, y), TraceNormals( ray ) );
}

void kernel RenderDepth(
	write_only image2d_t pixels, const uint width, const uint height,
	const float4 eye, const float4 p1, const float4 p2, const float4 p3
)
{
	// extract pixel coordinates from thread id
	const uint x = get_global_id( 0 ), y = get_global_id( 1 );
	const uint pixelIdx = x + y * get_global_size( 0 );

	// create primary ray and trace
	struct Ray ray;
	float u = (float)x / width, v = (float)y / height;
	ray.O = eye;
	ray.D = (float4)( RayTarget( u, v, 0.15f, p1.xyz, p2.xyz, p3.xyz, width, height ) - eye.xyz, 1 );
	ray.rD = (float4)(1.0f / ray.D.x, 1.0f / ray.D.y, 1.0f / ray.D.z, 1);
	uint steps = TraceDepth( ray );
	write_imagef( pixels, (int2)(x, y), (float4)( plasma_quintic( (float)steps * 0.006f ), 1 ) );
}