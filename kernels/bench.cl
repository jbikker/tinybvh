// Entry points for the interleaved Vulkan / OpenCL benchmark in program_vk.cpp.
//
// The traversal code itself is the stock tinybvh code from traverse.cl; all this
// file adds is a set of kernels that do exactly what the GLSL compute shaders
// (tiny.comp, tiny4.comp, tiny8.comp) do, so the two sides can be compared:
//
// - tmax is always 1e30, never taken from the ray. The stock batch_* kernels read
//   it back from the hit record, which would make every pass after the first one
//   cheaper than the first - fine for a one-shot trace, wrong for a benchmark that
//   dispatches the same rays twenty times in a row.
// - the only thing written per ray is one RGBA8 pixel, matching the imageStore in
//   the GLSL, instead of a 16-byte hit record.
//
// The ray buffer is the usual 64-byte tinybvh GPU ray, filled in by InitOpenCL().

#include "traverse.cl"

// the depth visualization from the GLSL shaders: 1 - min( 1, t * 0.01 ), grey.
uint depth_to_pixel( const float t )
{
	const float g = t >= 1e30f ? 0.0f : (1.0f - fmin( 1.0f, t * 0.01f ));
	const uint v = (uint)(g * 255.0f + 0.5f);
	return 0xff000000 | (v << 16) | (v << 8) | v; // R in the low byte: R8G8B8A8_UNORM
}

void kernel bench_bvh2( const global struct BVHNode* bvhNode, const global float4* orderedVerts,
	const global struct Ray* rayData, global uint* pixels )
{
	const uint threadId = get_global_id( 0 );
	const float3 O = rayData[threadId].O.xyz;
	const float3 D = rayData[threadId].D.xyz;
	const float3 rD = rayData[threadId].rD.xyz;
	const float4 hit = traverse( bvhNode, orderedVerts, 0, O, D, rD, 1e30f, 0 );
	pixels[threadId] = depth_to_pixel( hit.x );
}

void kernel bench_bvh4( const global float4* alt4Node, const global struct Ray* rayData, global uint* pixels )
{
	const uint threadId = get_global_id( 0 );
	const float3 O = rayData[threadId].O.xyz;
	const float3 D = rayData[threadId].D.xyz;
	const float3 rD = rayData[threadId].rD.xyz;
	const float4 hit = traverse_gpu4way( alt4Node, O, D, rD, 1e30f );
	pixels[threadId] = depth_to_pixel( hit.x );
}

void kernel bench_cwbvh( global const float4* cwbvhNodes, global const float4* cwbvhTris,
	const global struct Ray* rayData, global uint* pixels )
{
	const uint threadId = get_global_id( 0 );
#ifdef SIMD_AABBTEST
	float4 O4 = rayData[threadId].O; O4.w = 1;
	float4 D4 = rayData[threadId].D; D4.w = 0;
	float4 rD4 = rayData[threadId].rD; rD4.w = 1;
	const float4 hit = traverse_cwbvh( cwbvhNodes, cwbvhTris, O4, D4, rD4, 1e30f );
#else
	const float4 O4 = rayData[threadId].O;
	const float4 D4 = rayData[threadId].D;
	const float4 rD4 = rayData[threadId].rD;
	const float4 hit = traverse_cwbvh( cwbvhNodes, cwbvhTris, O4.xyz, D4.xyz, rD4.xyz, 1e30f );
#endif
	pixels[threadId] = depth_to_pixel( hit.x );
}
