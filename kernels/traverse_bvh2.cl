// ============================================================================
//
//        T R A V E R S E _ A I L A L A I N E
// 
// ============================================================================

uint RRScost_ailalaine( const global struct BVHNode* bvhNode, const global uint* idx, const global float4* orderedVerts, const float3 O, const float3 D, const float3 rD, const float tmax )
{
	// traverse BVH
	float4 hit;
	hit.x = tmax;
	uint nodeIdx = 0, stack[STACK_SIZE], stackPtr = 0;
	float cost = 0;
	while (1)
	{
		// process node
		cost += 1.2f; // TODO: obtain somehow via tiny_bvh.h?
		if (nodeIdx & 0x80000000)
		{
			const uint triCount = (nodeIdx >> 24) & 127;
			uint firstVert = (nodeIdx & 0xffffff) * 3;
			for (uint i = 0; i < triCount; i++, firstVert += 3)
			{
				cost += 1.0f; // TODO: obtain somehow via tiny_bvh.h?
				const float4 vert0 = orderedVerts[firstVert];
				const float4 edge1 = orderedVerts[firstVert + 1];
				const float4 edge2 = orderedVerts[firstVert + 2];
				const float3 h = cross( D, edge2.xyz );
				const float a = dot( edge1.xyz, h );
				const float f = 1 / a;
				const float3 s = O - vert0.xyz;
				const float u = f * dot( s, h );
				const float3 q = cross( s, edge1.xyz );
				const float v = f * dot( D, q );
				if (u < 0 || v < 0 || u + v > 1) continue;
				const float d = f * dot( edge2.xyz, q );
				if (d > 0.0f && d < hit.x) hit = (float4)(d, u, v, vert0.w);
			}
			if (stackPtr == 0) break;
			nodeIdx = stack[--stackPtr];
			continue;
		}
		const float4 lmin = bvhNode[nodeIdx].lmin, lmax = bvhNode[nodeIdx].lmax;
		const float4 rmin = bvhNode[nodeIdx].rmin, rmax = bvhNode[nodeIdx].rmax;
		uint left = as_uint( lmin.w ), right = as_uint( lmax.w );
		// child AABB intersection tests
		const float3 t1a = (lmin.xyz - O) * rD, t2a = (lmax.xyz - O) * rD;
		const float3 t1b = (rmin.xyz - O) * rD, t2b = (rmax.xyz - O) * rD;
		const float3 minta = fmin( t1a, t2a ), maxta = fmax( t1a, t2a );
		const float3 mintb = fmin( t1b, t2b ), maxtb = fmax( t1b, t2b );
		const float tmina = fmax( fmax( fmax( minta.x, minta.y ), minta.z ), 0 );
		const float tminb = fmax( fmax( fmax( mintb.x, mintb.y ), mintb.z ), 0 );
		const float tmaxa = fmin( fmin( fmin( maxta.x, maxta.y ), maxta.z ), hit.x );
		const float tmaxb = fmin( fmin( fmin( maxtb.x, maxtb.y ), maxtb.z ), hit.x );
		const float dist1 = tmina > tmaxa ? 1e30f : tmina;
		const float dist2 = tminb > tmaxb ? 1e30f : tminb;
		if (dist1 > dist2)
		{
			if (dist2 == 1e30f) { if (stackPtr == 0) break; else nodeIdx = stack[--stackPtr]; }
			else { nodeIdx = right; if (dist1 < 1e30f) stack[stackPtr++] = left; }
		}
		else
		{
			if (dist1 == 1e30f) { if (stackPtr == 0) break; else nodeIdx = stack[--stackPtr]; }
			else { nodeIdx = left; if (dist2 < 1e30f) stack[stackPtr++] = right; }
		}

	}
	// write back intersection result
	return (uint)cost;
}

float4 traverse_ailalaine( const global struct BVHNode* bvhNode, 
	const global uint* idx, const global float4* orderedVerts, const global uint* opmap,
	const float3 O, const float3 D, const float3 rD, const float tmax, uint* stepCount )
{
	// prepare slab test
	const float3 rO = O * -rD;
	// traverse BVH
	float4 hit = (float4)( tmax, 0, 0, 0 );
	uint nodeIdx = 0, stack[STACK_SIZE], stackPtr = 0, steps = 0;
	while (1)
	{
		steps++;
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
				if (u < 0 || v < 0 || u + v > 1) continue;
				const float d = f * dot( edge2.xyz, q );
				if (d <= 0.0f || d >= hit.x) continue;
				if (opmap)
				{
					const uint triIdx = as_uint( vert0.w );
					const int row = (int)( (u + v) * 32.0f ), diag = (int)( (1 - u) * 32.0f );
					const int idx = (row * row) + (int)( v * 32.0f ) + (diag - (31 - row));
					if (!(opmap[triIdx * 32 + (idx >> 5)] & (1 << (idx & 31)))) continue;
				}
				hit = (float4)(d, u, v, vert0.w);
			}
			if (stackPtr == 0) break;
			nodeIdx = stack[--stackPtr];
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
		const float dist1 = tmina > tmaxa ? 1e30f : tmina;
		const float dist2 = tminb > tmaxb ? 1e30f : tminb;
		if (dist1 > dist2)
		{
			if (dist2 == 1e30f) { if (stackPtr == 0) break; else nodeIdx = stack[--stackPtr]; }
			else { nodeIdx = right; if (dist1 < 1e30f) stack[stackPtr++] = left; }
		}
		else
		{
			if (dist1 == 1e30f) { if (stackPtr == 0) break; else nodeIdx = stack[--stackPtr]; }
			else { nodeIdx = left; if (dist2 < 1e30f) stack[stackPtr++] = right; }
		}
	}
	if (stepCount) *stepCount += steps;
	return hit;
}

bool isoccluded_ailalaine( 
	const global struct BVHNode* bvhNode, 
	const global uint* idx, const global float4* orderedVerts, const global uint* opmap, 
	const float3 O, const float3 D, const float3 rD, const float tmax )
{
	// prepare slab test
	const float3 rO = O * -rD;
	// traverse BVH
	uint nodeIdx = 0, stack[STACK_SIZE], stackPtr = 0;
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
				const float f = 1 / dot( edge1.xyz, h );
				const float3 s = O - vert0.xyz;
				const float u = f * dot( s, h );
				const float3 q = cross( s, edge1.xyz );
				const float v = f * dot( D, q );
				if (u < 0 || v < 0 || u + v > 1) continue;
				const float d = f * dot( edge2.xyz, q );
				if (d <= 0.0f || d >= tmax) continue;
				if (opmap)
				{
					const uint triIdx = as_uint( vert0.w );
					const int row = (int)( (u + v) * 32.0f ), diag = (int)( (1 - u) * 32.0f );
					const int idx = (row * row) + (int)( v * 32.0f ) + (diag - (31 - row));
					if (!(opmap[triIdx * 32 + (idx >> 5)] & (1 << (idx & 31)))) continue;
				}
				return true;
			}
			if (stackPtr == 0) break;
			nodeIdx = stack[--stackPtr];
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
		const float tmaxa = fmin( fmin( fmin( maxta.x, maxta.y ), maxta.z ), tmax );
		const float tmaxb = fmin( fmin( fmin( maxtb.x, maxtb.y ), maxtb.z ), tmax );
		const float dist1 = tmina > tmaxa ? 1e30f : tmina;
		const float dist2 = tminb > tmaxb ? 1e30f : tminb;
		if (dist1 > dist2)
		{
			if (dist2 == 1e30f) { if (stackPtr == 0) break; else nodeIdx = stack[--stackPtr]; }
			else { nodeIdx = right; if (dist1 < 1e30f) stack[stackPtr++] = left; }
		}
		else
		{
			if (dist1 == 1e30f) { if (stackPtr == 0) break; else nodeIdx = stack[--stackPtr]; }
			else { nodeIdx = left; if (dist2 < 1e30f) stack[stackPtr++] = right; }
		}
	}
	return false;
}

void kernel batch_ailalaine( const global struct BVHNode* bvhNode, const global uint* idx, const global float4* orderedVerts, global struct Ray* rayData )
{
	// fetch ray
	const uint threadId = get_global_id( 0 );
	if (threadId >= get_global_size( 0 )) return;
	const float3 O = rayData[threadId].O.xyz;
	const float3 D = rayData[threadId].D.xyz;
	const float3 rD = rayData[threadId].rD.xyz;
	float4 hit = traverse_ailalaine( bvhNode, idx, orderedVerts, 0, O, D, rD, 1e30f, 0 );
	rayData[threadId].hit = hit;
}

void kernel batch_ailalaine_any( const global struct BVHNode* bvhNode, const global uint* idx, const global float4* orderedVerts, global struct Ray* rayData )
{
	// fetch ray
	const uint threadId = get_global_id( 0 );
	if (threadId >= get_global_size( 0 )) return;
	const float3 O = rayData[threadId].O.xyz;
	const float3 D = rayData[threadId].D.xyz;
	const float3 rD = rayData[threadId].rD.xyz;
	const float tmax = 1e30f; // TODO: get this from the ray.
	float4 hit = 0;
	if (isoccluded_ailalaine( bvhNode, idx, orderedVerts, 0, O, D, rD, tmax )) hit.w = as_float( 1 );
	rayData[threadId].hit = hit;
}

void kernel batch_ailalaine_rrs( const global struct BVHNode* bvhNode, const global uint* idx, const global float4* orderedVerts, global struct Ray* rayData, global uint* rrsResult )
{
	// fetch ray
	const uint threadId = get_global_id( 0 );
	if (threadId >= get_global_size( 0 )) return;
	const float3 O = rayData[threadId].O.xyz;
	const float3 D = rayData[threadId].D.xyz;
	const float3 rD = rayData[threadId].rD.xyz;
	rrsResult[threadId] = RRScost_ailalaine( bvhNode, idx, orderedVerts, O, D, rD, 1e30f );
}