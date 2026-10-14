struct BVHNode
{
	float4 lmin; // unsigned left in w
	float4 lmax; // unsigned right in w
	float4 rmin; // unsigned triCount in w
	float4 rmax; // unsigned firstTri in w
};

struct RayData
{
	// 16-byte values, to encourage the compilers to fetch 16 bytes at a time:
	// 12 (so, 8 + 4) will be slower.
	float4 O; // .xyz = origin,    .w unused
	float4 D; // .xyz = direction, .w unused
};

#define GROUP_X    8
#define GROUP_Y    8
#define GROUP_SIZE (GROUP_X * GROUP_Y)

// BVH traversal stack size 
#define STACK_SIZE 32

#define STACK_AT(i) stack[(i) * GROUP_SIZE + lane]

float4 traverse( const global struct BVHNode* bvhNode, 
	const global float4* orderedVerts, const global uint* opmap,
	const float3 O, const float3 D, const float3 rD, const float tmax,
	local uint* stack, const uint lane, uint* stepCount )
{
	// "find nearest" ray query using the BVH_GPU format.
	const float3 rO = O * -rD; // precalculate for efficient slab test
	float4 hit = (float4)( tmax, 0, 0, 0 );
	uint nodeIdx = 0, stackPtr = 0;
	while (1)
	{
		if (nodeIdx & 0x80000000)
		{
			const uint triCount = (nodeIdx >> 24) & 127;
			uint firstVert = (nodeIdx & 0xffffff) * 3;
			for (uint i = 0; i < triCount; i++, firstVert += 3)
			{
				const float4 vert0 = orderedVerts[firstVert];
				const float4 edge1 = orderedVerts[firstVert + 1];
				const float4 edge2 = orderedVerts[firstVert + 2];
				const float3 h = cross( D, edge2.xyz );
				const float a = dot( edge1.xyz, h );
				const float f = native_recip( a );
				const float3 s = O - vert0.xyz;
				const float u = f * dot( s, h );
				const float3 q = cross( s, edge1.xyz );
				const float v = f * dot( D, q );
				const float d = f * dot( edge2.xyz, q );
				const bool valid = u >= 0 && v >= 0 && u + v <= 1 && d > 0 && d < hit.x;
				hit = valid ? (float4)( d, u, v, vert0.w ) : hit;
			}
			if (stackPtr == 0) break;
			nodeIdx = STACK_AT( --stackPtr );
			continue;
		}
		const float4 lmin = bvhNode[nodeIdx].lmin, lmax = bvhNode[nodeIdx].lmax;
		const float4 rmin = bvhNode[nodeIdx].rmin, rmax = bvhNode[nodeIdx].rmax;
		uint left = as_uint( lmin.w ), right = as_uint( lmax.w );
		const float3 t1a = fma( lmin.xyz, rD, rO ), t2a = fma( lmax.xyz, rD, rO );
		const float3 t1b = fma( rmin.xyz, rD, rO ), t2b = fma( rmax.xyz, rD, rO );
		const float3 minta = fmin( t1a, t2a ), maxta = fmax( t1a, t2a );
		const float3 mintb = fmin( t1b, t2b ), maxtb = fmax( t1b, t2b );
		const float tmina = fmax( fmax( fmax( minta.x, minta.y ), minta.z ), 0 );
		const float tminb = fmax( fmax( fmax( mintb.x, mintb.y ), mintb.z ), 0 );
		const float tmaxa = fmin( fmin( fmin( maxta.x, maxta.y ), maxta.z ), hit.x );
		const float tmaxb = fmin( fmin( fmin( maxtb.x, maxtb.y ), maxtb.z ), hit.x );
		const bool hitA = tmina <= tmaxa, hitB = tminb <= tmaxb;
		if (hitA && hitB)
		{
			uint near = left, far = right;
			if (tminb < tmina) near = right, far = left;
			STACK_AT( stackPtr++ ) = far, nodeIdx = near;
		}
		else if (hitA) nodeIdx = left;
		else if (hitB) nodeIdx = right;
		else { if (stackPtr == 0) break; nodeIdx = STACK_AT( --stackPtr ); }
	}
	return hit;
}

// the depth visualization from the GLSL shaders: 1 - min( 1, t * 0.01 ), grey.
uint depth_to_pixel( const float t )
{
	const float g = t >= 1e30f ? 0.0f : (1.0f - fmin( 1.0f, t * 0.01f ));
	const uint v = (uint)(g * 255.0f + 0.5f);
	return 0xff000000 | (v << 16) | (v << 8) | v; // R in the low byte: R8G8B8A8_UNORM
}

#define BVH_DIR_MIN 0x1p-100f

float safe_rcp( float x )
{
	const uint s = as_uint( x ) & 0x80000000u;
	return 1.0f / as_float( as_uint( fmax( fabs( x ), BVH_DIR_MIN ) ) | s );
}

float3 safe_rcp3( float3 d )
{
	const uint3 s = as_uint3( d ) & 0x80000000u;
	return 1.0f / as_float3( as_uint3( fmax( fabs( d ), (float3)BVH_DIR_MIN ) ) | s );
}

// Launched 2D over the image: global size rtWidth x rtHeight, local size
// GROUP_X x GROUP_Y. The ray and pixel buffers stay in scanline order, exactly as
// the Vulkan side has them.
__attribute__((reqd_work_group_size( GROUP_X, GROUP_Y, 1 )))
void kernel bench_bvh2( const global struct BVHNode* bvhNode, const global float4* orderedVerts,
	const global struct RayData* rayData, global uint* pixels )
{
	local uint stack[STACK_SIZE * GROUP_SIZE];
	const uint lane = get_local_id( 1 ) * GROUP_X + get_local_id( 0 );
	const uint threadId = get_global_id( 1 ) * get_global_size( 0 ) + get_global_id( 0 );
	const float3 O = rayData[threadId].O.xyz;
	const float3 D = rayData[threadId].D.xyz;
	const float3 rD = safe_rcp3( D );
	const float4 hit = traverse( bvhNode, orderedVerts, 0, O, D, rD, 1e30f, stack, lane, 0 );
	pixels[threadId] = depth_to_pixel( hit.x );
}