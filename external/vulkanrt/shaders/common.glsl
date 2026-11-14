// Shared resource declarations for the Vulkan tinybvh benchmark.
//
// D3D12 hands buffers to shaders through root descriptors, which have no Vulkan
// equivalent; every resource therefore lives in a single descriptor set that is
// written once at init time. Bindings unused by a given shader are simply left
// undeclared - a shader is not required to consume every binding in the layout.
//
//   binding 0 : render target        (was u0)
//   binding 1 : TLAS                 (was t0 space0)
//   binding 2 : ray buffer           (was t1 space0)
//   binding 3 : BVH_GPU nodes        (was t0 space1)
//   binding 4 : BVH_GPU primIdx      (was t1 space1)
//   binding 5 : BVH_GPU triangles    (was t2 space1)
//   binding 6 : BVH4_GPU blob        (was t3 space1)
//   binding 7 : CWBVH nodes          (was t4 space1)
//   binding 8 : CWBVH triangles      (was t5 space1)

#ifndef COMMON_GLSL
#define COMMON_GLSL

// 32 bytes per ray, matching the D3D12 StructuredBuffer<RayData> stride: the
// pad lanes of the HLSL struct are folded into the vec4s here.
struct RayData
{
	vec4 origin;    // .xyz = origin,    .w = unused
	vec4 direction; // .xyz = direction, .w = unused
};

layout( set = 0, binding = 2, std430 ) readonly buffer RayBuffer { RayData rayBuffer[]; };

// robust reciprocal calculation

#define BVH_DIR_MIN 7.88860905e-31f // 0x1p-100; 1 / BVH_DIR_MIN == BVH_RCP_FAR

float safe_rcp( in float x )
{
	const uint s = floatBitsToUint( x ) & 0x80000000u;
	return 1.0f / uintBitsToFloat( floatBitsToUint( max( abs( x ), BVH_DIR_MIN ) ) | s );
}

vec3 safe_rcp3( in vec3 d )
{
	const uvec3 s = floatBitsToUint( d ) & 0x80000000u;
	return 1.0f / uintBitsToFloat( floatBitsToUint( max( abs( d ), vec3( BVH_DIR_MIN ) ) ) | s );
}

#endif // COMMON_GLSL
