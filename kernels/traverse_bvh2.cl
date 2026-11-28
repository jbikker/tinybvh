// ============================================================================
//
//        T R A V E R S E _ A I L A L A I N E
// 
// ============================================================================

#ifndef TINYBVH_SAFE_RCP
#define TINYBVH_SAFE_RCP
// 1 / x, or +/-2^100 for a (near) zero x, as tinybvh_safercp on the host.
inline float safe_rcp( const float x ) { return fabs( x ) >= 0x1p-100f ? 1.0f / x : copysign( 0x1p100f, x ); }
inline float3 safe_rcp3( const float3 d ) { return (float3)( safe_rcp( d.x ), safe_rcp( d.y ), safe_rcp( d.z ) ); }
#endif

// Per-vendor choices, timed on a Radeon 890M (RDNA3.5) and an RTX 5080 (Blackwell):
// AMD: the batch kernels keep the first BVH2_LOCAL_STACK stack entries of each lane in local memory
// (private memory is slow scratch), for 1D workgroups of up to BVH2_LOCAL_LANES lanes.
// NVIDIA: one set of loads for leaf and interior lanes, and a branchless triangle test.
#if defined ISAMD && STACK_SIZE > 16
#define BVH2_LOCAL_STACK 16
#define BVH2_LOCAL_LANES 64
#elif defined ISNVIDIA
#define BVH2_UNIFIED_FETCH
#define BVH2_BRANCHLESS_TRI
#endif
#define BVH2_DONE 0x7fffffffu // no node: interior nodes are below it, leaf references have the msb set

// Moller-Trumbore in accept form; with IEEE compares this also rejects the NaNs of degenerate triangles.
inline bool intersect_bvh2_tri( const float4 vert0, const float4 edge1, const float4 edge2,
	const float3 O, const float3 D, const global uint* opmap, float4* hit )
{
				const float3 h = cross( D, edge2.xyz );
	const float f = native_recip( dot( edge1.xyz, h ) );
				const float3 s = O - vert0.xyz;
				const float u = f * dot( s, h );
				const float3 q = cross( s, edge1.xyz );
				const float v = f * dot( D, q );
#ifdef BVH2_BRANCHLESS_TRI
				const float d = f * dot( edge2.xyz, q );
	bool valid = u >= 0 && v >= 0 && u + v <= 1 && d > 0 && d < (*hit).x;
#else
	if (!(u >= 0 && v >= 0 && u + v <= 1)) return false;
	const float d = f * dot( edge2.xyz, q );
	if (!(d > 0 && d < (*hit).x)) return false;
	bool valid = true;
#endif
	if (opmap && valid)
				{
					const uint triIdx = as_uint( vert0.w );
					const int row = (int)( (u + v) * 32.0f ), diag = (int)( (1 - u) * 32.0f );
					const int idx = (row * row) + (int)( v * 32.0f ) + (diag - (31 - row));
		valid = (opmap[triIdx * 32 + (idx >> 5)] & (1 << (idx & 31))) != 0;
				}
#ifdef BVH2_BRANCHLESS_TRI
	*hit = valid ? (float4)( d, u, v, vert0.w ) : *hit;
#else
	if (valid) *hit = (float4)( d, u, v, vert0.w );
#endif
	return valid;
}

// Stack: private, or with localStack the first BVH2_LOCAL_STACK entries at lstack, BVH2_LOCAL_LANES apart.
#ifdef BVH2_LOCAL_STACK
#define BVH2_PUSH( x ) { if (!localStack) stack[stackPtr] = (x); else if (stackPtr < BVH2_LOCAL_STACK) \
	lstack[stackPtr * BVH2_LOCAL_LANES] = (x); else deep[stackPtr - BVH2_LOCAL_STACK] = (x); stackPtr++; }
#define BVH2_POP() ( --stackPtr, !localStack ? stack[stackPtr] : stackPtr < BVH2_LOCAL_STACK ? \
	lstack[stackPtr * BVH2_LOCAL_LANES] : deep[stackPtr - BVH2_LOCAL_STACK] )
#else
#define BVH2_PUSH( x ) { stack[stackPtr++] = (x); }
#define BVH2_POP() stack[--stackPtr]
#endif

// One interior node or one triangle per iteration (a leaf reference counts down in place), so lanes in a
// leaf do not hold up the others. anyHit: return at the first hit closer than tmax.
float4 traverse_bvh2( const global struct BVHNode* bvhNode, const global float4* orderedVerts,
	const global uint* opmap, const float3 O, const float3 D, const float3 rD, const float tmax,
	const bool anyHit, local uint* lstack, const bool localStack, uint* stepCount )
{
	const float3 rO = O * -rD; // precalculate for efficient slab test
	float4 hit = (float4)( tmax, 0, 0, 0 );
	uint nodeIdx = 0, stack[STACK_SIZE], stackPtr = 0, steps = 0;
#ifdef BVH2_LOCAL_STACK
	uint deep[STACK_SIZE - BVH2_LOCAL_STACK]; // local stack: the entries beyond the local ones
#endif
	while (nodeIdx != BVH2_DONE)
	{
		bool pop;
		const bool leaf = (nodeIdx & 0x80000000u) != 0;
	#ifdef BVH2_UNIFIED_FETCH
		const global float4* p = leaf ? orderedVerts + (size_t)(nodeIdx & 0xffffff) * 3 : (const global float4*)(bvhNode + nodeIdx);
		const float4 f0 = p[0], f1 = p[1], f2 = p[2], f3 = p[leaf ? 2 : 3]; // a triangle is 3 float4s
	#endif
		if (leaf)
		{
			const uint triCount = (nodeIdx >> 24) & 127;
			if (triCount)
			{
			#ifdef BVH2_UNIFIED_FETCH
				const bool found = intersect_bvh2_tri( f0, f1, f2, O, D, opmap, &hit );
			#else
				const global float4* tri = orderedVerts + (size_t)(nodeIdx & 0xffffff) * 3;
				const bool found = intersect_bvh2_tri( tri[0], tri[1], tri[2], O, D, opmap, &hit );
			#endif
				if (anyHit && found) { if (stepCount) *stepCount += steps + 1; return hit; }
			}
			nodeIdx += 1u - (1u << 24); // next triangle: first + 1, count - 1
			pop = triCount <= 1, steps += pop;
		}
		else
		{
		#ifdef BVH2_UNIFIED_FETCH
			const float4 lmin = f0, lmax = f1, rmin = f2, rmax = f3;
		#else
		const float4 lmin = bvhNode[nodeIdx].lmin, lmax = bvhNode[nodeIdx].lmax;
		const float4 rmin = bvhNode[nodeIdx].rmin, rmax = bvhNode[nodeIdx].rmax;
		#endif
			const uint left = as_uint( lmin.w ), right = as_uint( lmax.w );
		const float3 t1a = fma( lmin.xyz, rD, rO ), t2a = fma( lmax.xyz, rD, rO );
		const float3 t1b = fma( rmin.xyz, rD, rO ), t2b = fma( rmax.xyz, rD, rO );
		const float3 minta = fmin( t1a, t2a ), maxta = fmax( t1a, t2a );
		const float3 mintb = fmin( t1b, t2b ), maxtb = fmax( t1b, t2b );
		const float tmina = fmax( fmax( fmax( minta.x, minta.y ), minta.z ), 0 );
		const float tminb = fmax( fmax( fmax( mintb.x, mintb.y ), mintb.z ), 0 );
		const float tmaxa = fmin( fmin( fmin( maxta.x, maxta.y ), maxta.z ), hit.x );
		const float tmaxb = fmin( fmin( fmin( maxtb.x, maxtb.y ), maxtb.z ), hit.x );
		const bool hitA = tmina <= tmaxa, hitB = tminb <= tmaxb;
			pop = !(hitA || hitB), steps++;
		if (hitA && hitB)
		{
				const bool swap = tminb < tmina; // visit the nearest child first
				BVH2_PUSH( swap ? left : right );
				nodeIdx = swap ? right : left;
		}
			else nodeIdx = hitA ? left : right;
		}
		if (pop) nodeIdx = stackPtr == 0 ? BVH2_DONE : BVH2_POP();
	}
	if (stepCount) *stepCount += steps;
	return hit;
}
#undef BVH2_PUSH
#undef BVH2_POP

float4 traverse( const global struct BVHNode* bvhNode, 
	const global float4* orderedVerts, const global uint* opmap,
	const float3 O, const float3 D, const float3 rD, const float tmax, uint* stepCount )
{
	// "find nearest" ray query using the BVH_GPU format.
	return traverse_bvh2( bvhNode, orderedVerts, opmap, O, D, rD, tmax, false, 0, false, stepCount );
}

bool isoccluded( 
	const global struct BVHNode* bvhNode, 
	const global float4* orderedVerts, const global uint* opmap, 
	const float3 O, const float3 D, const float3 rD, const float tmax )
{
	// "any hit" ray query using the BVH_GPU format.
	return traverse_bvh2( bvhNode, orderedVerts, opmap, O, D, rD, tmax, true, 0, false, 0 ).x < tmax;
}

// The batch kernels below: on AMD with the local stack, when the workgroup fits it.
#ifdef BVH2_LOCAL_STACK
#define BVH2_LOCAL_STACK_ARRAY local uint lstack[BVH2_LOCAL_STACK * BVH2_LOCAL_LANES]
#define BVH2_LOCAL_STACK_PTR lstack
#else
#define BVH2_LOCAL_STACK_ARRAY
#define BVH2_LOCAL_STACK_PTR 0
#endif
inline float4 batch_traverse_bvh2( const global struct BVHNode* bvhNode, const global float4* orderedVerts,
	const float3 O, const float3 D, const float3 rD, const float tmax, const bool anyHit, local uint* lstack )
{
#ifdef BVH2_LOCAL_STACK
	if (get_local_size( 0 ) <= BVH2_LOCAL_LANES && get_local_size( 1 ) * get_local_size( 2 ) == 1)
		return traverse_bvh2( bvhNode, orderedVerts, 0, O, D, rD, tmax, anyHit, lstack + get_local_id( 0 ), true, 0 );
#endif
	return traverse_bvh2( bvhNode, orderedVerts, 0, O, D, rD, tmax, anyHit, 0, false, 0 );
}

void kernel batch_nearest( const global struct BVHNode* bvhNode, const global float4* orderedVerts, global struct Ray* rayData )
{
	// fetch ray
	const uint threadId = get_global_id( 0 );
	if (threadId >= get_global_size( 0 )) return;
	BVH2_LOCAL_STACK_ARRAY;
	const float3 O = rayData[threadId].O.xyz;
	const float3 D = rayData[threadId].D.xyz;
	const float3 rD = rayData[threadId].rD.xyz;
	const float tmax = rayData[threadId].hit.x;
	float4 hit = batch_traverse_bvh2( bvhNode, orderedVerts, O, D, rD, tmax, false, BVH2_LOCAL_STACK_PTR );
	rayData[threadId].hit = hit;
}

void kernel batch_any( const global struct BVHNode* bvhNode, const global float4* orderedVerts, global struct Ray* rayData )
{
	// fetch ray
	const uint threadId = get_global_id( 0 );
	if (threadId >= get_global_size( 0 )) return;
	BVH2_LOCAL_STACK_ARRAY;
	const float3 O = rayData[threadId].O.xyz;
	const float3 D = rayData[threadId].D.xyz;
	const float3 rD = rayData[threadId].rD.xyz;
	const float tmax = rayData[threadId].hit.x;
	float4 hit = 0;
	if (batch_traverse_bvh2( bvhNode, orderedVerts, O, D, rD, tmax, true, BVH2_LOCAL_STACK_PTR ).x < tmax) hit.w = as_float( 1 );
	rayData[threadId].hit = hit;
}

void kernel batch_nearest_packed( const global struct BVHNode* bvhNode, const global float4* orderedVerts, const global float4* rays, global float4* hits )
{
	const uint threadId = get_global_id( 0 );
	BVH2_LOCAL_STACK_ARRAY;
	const float4 O = rays[threadId * 2], D = rays[threadId * 2 + 1];
	hits[threadId] = batch_traverse_bvh2( bvhNode, orderedVerts, O.xyz, D.xyz, safe_rcp3( D.xyz ), O.w, false, BVH2_LOCAL_STACK_PTR );
}

void kernel batch_any_packed( const global struct BVHNode* bvhNode, const global float4* orderedVerts, const global float4* rays, global uint* occluded )
{
	const uint threadId = get_global_id( 0 );
	BVH2_LOCAL_STACK_ARRAY;
	const float4 O = rays[threadId * 2], D = rays[threadId * 2 + 1];
	occluded[threadId] = batch_traverse_bvh2( bvhNode, orderedVerts, O.xyz, D.xyz, safe_rcp3( D.xyz ), O.w, true, BVH2_LOCAL_STACK_PTR ).x < O.w ? 1 : 0;
}

void kernel reset_rays( global struct Ray* rayData, float d )
{
	const uint threadId = get_global_id( 0 );
	if (threadId >= get_global_size( 0 )) return;
	rayData[threadId].hit.x = d;
}

#undef BVH2_LOCAL_STACK
#undef BVH2_LOCAL_LANES
#undef BVH2_UNIFIED_FETCH
#undef BVH2_BRANCHLESS_TRI
#undef BVH2_DONE
#undef BVH2_LOCAL_STACK_ARRAY
#undef BVH2_LOCAL_STACK_PTR
