// ============================================================================
//
//        T R A V E R S E _ C W B V H   ( S L A N G )
//
// ============================================================================

// A Slang port of traverse_cwbvh.cl, for compute and other shaders on any
// target Slang compiles for (SPIR-V for Vulkan, Metal, HLSL/DXIL, ...).
// It traverses a BVH8_CWBVH: bvh8Data (5 float4 per node) and bvh8Tris
// (4 float4 per triangle with CWBVH_COMPRESSED_TRIS, else 3), uploaded
// as they are to two StructuredBuffer<float4>.
// Define CWBVH_COMPRESSED_TRIS here when tiny_bvh.h defines it: the triangle
// format must match the build.
// Usage:
//   #include "traverse_cwbvh.slang"
//   StructuredBuffer<float4> cwbvhNodes, cwbvhTris;
//   ...
//   const float4 hit = traverse_cwbvh( cwbvhNodes, cwbvhTris, O, D, 1.0f / D, 1e30f );
//   // hit.x: distance, hit.yz: barycentrics, asuint( hit.w ): primitive index
//   const bool occluded = isoccluded_cwbvh( cwbvhNodes, cwbvhTris, O, D, 1.0f / D, distance );

#ifndef CWBVH_STACK
#define CWBVH_STACK 24
#endif

#ifdef CWBVH_COMPRESSED_TRIS
#define CWBVH_TRI_STRIDE 4 // must match the tiny_bvh.h build define
#else
#define CWBVH_TRI_STRIDE 3
#endif

// 4 quantized child bounds (one byte each) as floats in [1, 2): see QUNPACK_BITTRICK
// in traverse_cwbvh.cl. The node test compensates with a scale of 256 and a bias of 1.
float4 cwbvh_unpack_q8x4( const uint w )
{
	const uint m = 0x007F8000u, b = 0x3F800000u; // mantissa slot 22..15, exponent 0
	return asfloat( (uint4( w << 15, w << 7, w >> 1, w >> 9 ) & m) | b );
}

// Slab test of 4 of the 8 children of a node; returns their bits in the hit mask.
uint cwbvh_child_hits( const uint meta4, const uint octinv4,
	const float4 lox4, const float4 hix4, const float4 loy4, const float4 hiy4, const float4 loz4, const float4 hiz4,
	const float3 idir, const float3 orig, const float tmax )
{
	const uint is_inner4 = (meta4 & (meta4 << 1)) & 0x10101010u;
	// expand the 0x10 marker in each byte to 0xFF: (x << 4) - (x >> 4) is
	// 0x100 - 0x01 per marked byte, with no carry into the neighbours.
	const uint inner_mask4 = (is_inner4 << 4) - (is_inner4 >> 4);
	const uint bit_index4 = (meta4 ^ (octinv4 & inner_mask4)) & 0x1F1F1F1Fu;
	const uint child_bits4 = (meta4 >> 5) & 0x07070707u;
	const float4 tmin4 = max( max( max( mad( lox4, idir.x, orig.x ), mad( loy4, idir.y, orig.y ) ), mad( loz4, idir.z, orig.z ) ), 0.0f );
	const float4 tmax4 = min( min( min( mad( hix4, idir.x, orig.x ), mad( hiy4, idir.y, orig.y ) ), mad( hiz4, idir.z, orig.z ) ), tmax );
	uint hitmask = 0;
	[unroll] for (uint i = 0; i < 4; i++) if (tmin4[i] <= tmax4[i])
		hitmask |= ((child_bits4 >> (i * 8)) & 255u) << ((bit_index4 >> (i * 8)) & 31u);
	return hitmask;
}

// Shared traversal: nearest hit, or any hit for anyHit (a literal at the call
// sites, so each specializes). Returns (t, u, v, primitive index as float bits);
// t is the input t and the primitive index 0 when nothing was hit.
float4 cwbvh_traverse( StructuredBuffer<float4> cwbvhNodes, StructuredBuffer<float4> cwbvhTris,
	const float3 O, const float3 D, const float3 rD, const float t, const bool anyHit )
{
	// prepare traversal
	uint stack[CWBVH_STACK];
	uint hitAddr = 0, stackPtr = 0;
	float2 uv = float2( 0, 0 );
	float tmax = t;
	const uint octinv4 = (7 - ((D.x < 0 ? 4 : 0) | (D.y < 0 ? 2 : 0) | (D.z < 0 ? 1 : 0))) * 0x1010101u;
	const float3 rDs = rD * 256.0f; // per-ray, hoisted out of the node loop.
	uint2 ngroup = uint2( 0, 0x80000000u ), tgroup = uint2( 0, 0 );
	uint parentNode = 0;
	while (true)
	{
		const uint imask = ngroup.y;
		const uint child_bit_index = firstbithigh( ngroup.y );
		const uint child_node_base_index = ngroup.x;
		ngroup.y &= ~(1u << child_bit_index);
		// a stack entry is the parent node and the hit bits left: the node gives back the rest
		if (ngroup.y > 0x00FFFFFFu && stackPtr < CWBVH_STACK) stack[stackPtr++] = (parentNode << 8) | (ngroup.y >> 24);
		{
			const uint slot_index = (child_bit_index - 24) ^ (octinv4 & 255);
			const uint relative_index = countbits( imask & ~(0xFFFFFFFFu << slot_index) );
			const uint nodeIndex = child_node_base_index + relative_index;
			const uint child_node_index = nodeIndex * 5;
			const float4 n0 = cwbvhNodes[child_node_index + 0], n1 = cwbvhNodes[child_node_index + 1];
			const float4 n2 = cwbvhNodes[child_node_index + 2], n3 = cwbvhNodes[child_node_index + 3];
			const float4 n4 = cwbvhNodes[child_node_index + 4];
			// n0.w packs three 127-biased quantization exponents plus imask, so each
			// byte is already an IEEE-754 exponent field: (b << 23) is 2^e directly.
			const uint e4 = asuint( n0.w );
			parentNode = nodeIndex;
			ngroup.x = asuint( n1.x );
			tgroup = uint2( asuint( n1.y ), 0 );
			const float3 idir = float3( asfloat( (e4 & 255) << 23 ) * rDs.x,
				asfloat( ((e4 >> 8) & 255) << 23 ) * rDs.y, asfloat( ((e4 >> 16) & 255) << 23 ) * rDs.z );
			const float3 orig = (n0.xyz - O) * rD - idir;
			uint hitmask = cwbvh_child_hits( asuint( n1.z ), octinv4, // first 4
				cwbvh_unpack_q8x4( asuint( rD.x < 0 ? n3.z : n2.x ) ), cwbvh_unpack_q8x4( asuint( rD.x < 0 ? n2.x : n3.z ) ),
				cwbvh_unpack_q8x4( asuint( rD.y < 0 ? n4.x : n2.z ) ), cwbvh_unpack_q8x4( asuint( rD.y < 0 ? n2.z : n4.x ) ),
				cwbvh_unpack_q8x4( asuint( rD.z < 0 ? n4.z : n3.x ) ), cwbvh_unpack_q8x4( asuint( rD.z < 0 ? n3.x : n4.z ) ),
				idir, orig, tmax );
			hitmask |= cwbvh_child_hits( asuint( n1.w ), octinv4, // second 4
				cwbvh_unpack_q8x4( asuint( rD.x < 0 ? n3.w : n2.y ) ), cwbvh_unpack_q8x4( asuint( rD.x < 0 ? n2.y : n3.w ) ),
				cwbvh_unpack_q8x4( asuint( rD.y < 0 ? n4.y : n2.w ) ), cwbvh_unpack_q8x4( asuint( rD.y < 0 ? n2.w : n4.y ) ),
				cwbvh_unpack_q8x4( asuint( rD.z < 0 ? n4.w : n3.y ) ), cwbvh_unpack_q8x4( asuint( rD.z < 0 ? n3.y : n4.w ) ),
				idir, orig, tmax );
			ngroup.y = (hitmask & 0xFF000000u) | (e4 >> 24);
			tgroup.y = hitmask & 0x00FFFFFFu;
		}
		// process the triangles referenced by the current triangle group
		while (tgroup.y != 0)
		{
			const uint triangleIndex = firstbithigh( tgroup.y );
			const uint triAddr = tgroup.x + triangleIndex * CWBVH_TRI_STRIDE;
			tgroup.y -= 1u << triangleIndex;
		#ifdef CWBVH_COMPRESSED_TRIS
			// Fast intersection of triangle data for the algorithm in:
			// "Fast Ray-Triangle Intersections by Coordinate Transformation"
			// Baldwin & Weber, 2016.
			const float4 T2 = cwbvhTris[triAddr + 2];
			const float transS = dot( T2.xyz, O ) + T2.w;
			const float transD = dot( T2.xyz, D );
			const float s = -transS;
			if (!(s * transD > 0)) continue; // d <= 0, transD == 0, or NaN
			if (!(abs( s ) < tmax * abs( transD ))) continue; // d >= tmax, or NaN
			const float d = s / transD; // only for survivors
			const float4 T0 = cwbvhTris[triAddr + 0];
			const float4 T1 = cwbvhTris[triAddr + 1];
			const float3 I = O + d * D;
			const float u = dot( T0.xyz, I ) + T0.w;
			const float v = dot( T1.xyz, I ) + T1.w;
			if (!(u >= 0 && v >= 0 && u + v < 1)) continue;
			if (anyHit) return float4( d, u, v, cwbvhTris[triAddr + 3].w );
			uv = float2( u, v );
			tmax = d;
			hitAddr = asuint( cwbvhTris[triAddr + 3].w );
		#else
			// Moeller-Trumbore, iquilezles.org version. Tris are stored as 3x16 bytes:
			// two edges plus vertex 0, whose w holds the original primitive index.
			const float3 e1 = cwbvhTris[triAddr].xyz;
			const float3 e2 = cwbvhTris[triAddr + 1].xyz;
			const float4 v0 = cwbvhTris[triAddr + 2];
			const float3 r = cross( D, e1 );
			const float a = dot( e2, r );
			const float f = 1 / a;
			const float3 s = O - v0.xyz;
			const float u = f * dot( s, r );
			const float3 q = cross( s, e2 );
			const float v = f * dot( D, q );
			if (!(u >= 0 && v >= 0 && u + v <= 1)) continue;
			const float d = f * dot( e1, q );
			if (!(d > 0 && d < tmax)) continue;
			if (anyHit) return float4( d, u, v, v0.w );
			uv = float2( u, v );
			tmax = d;
			hitAddr = asuint( v0.w );
		#endif
		}
		if (ngroup.y <= 0x00FFFFFFu)
		{
			if (stackPtr == 0) break;
			// unpack: the node index gives back both missing fields
			const uint entry = stack[--stackPtr];
			parentNode = entry >> 8;
			const uint parentAddr = parentNode * 5;
			ngroup.x = asuint( cwbvhNodes[parentAddr + 1].x );
			ngroup.y = ((entry & 255) << 24) | (asuint( cwbvhNodes[parentAddr].w ) >> 24);
		}
	}
	return float4( tmax, uv.x, uv.y, asfloat( hitAddr ) );
}

// "find nearest" ray query: (t, u, v, primitive index as float bits), t = 'tmax' if nothing was hit.
float4 traverse_cwbvh( StructuredBuffer<float4> cwbvhNodes, StructuredBuffer<float4> cwbvhTris,
	const float3 O, const float3 D, const float3 rD, const float tmax )
{
	return cwbvh_traverse( cwbvhNodes, cwbvhTris, O, D, rD, tmax, false );
}

// "any hit" ray query: whether anything is hit closer than 'tmax'.
bool isoccluded_cwbvh( StructuredBuffer<float4> cwbvhNodes, StructuredBuffer<float4> cwbvhTris,
	const float3 O, const float3 D, const float3 rD, const float tmax )
{
	return cwbvh_traverse( cwbvhNodes, cwbvhTris, O, D, rD, tmax, true ).x < tmax;
}

#ifdef CWBVH_BATCH_KERNELS
// Batch kernels, as batch_cwbvh / batch_cwbvh_any in traverse_cwbvh.cl: a ray per
// thread, its hit record written back. Define CWBVH_BATCH_KERNELS to compile them.
struct Ray { float4 O, D, rD, hit; };
StructuredBuffer<float4> cwbvhNodes;
StructuredBuffer<float4> cwbvhTris;
RWStructuredBuffer<Ray> rayData;

[shader( "compute" )]
[numthreads( 64, 1, 1 )]
void batch_cwbvh( uint3 threadId : SV_DispatchThreadID )
{
	const Ray ray = rayData[threadId.x];
	rayData[threadId.x].hit = traverse_cwbvh( cwbvhNodes, cwbvhTris, ray.O.xyz, ray.D.xyz, ray.rD.xyz, 1e30f );
}

[shader( "compute" )]
[numthreads( 64, 1, 1 )]
void batch_cwbvh_any( uint3 threadId : SV_DispatchThreadID )
{
	const Ray ray = rayData[threadId.x];
	const bool occluded = isoccluded_cwbvh( cwbvhNodes, cwbvhTris, ray.O.xyz, ray.D.xyz, ray.rD.xyz, 1e30f );
	rayData[threadId.x].hit = float4( 0, 0, 0, occluded ? 1 : 0 );
}
#endif
