// ============================================================================
//
//        T R A V E R S E _ A I L A L A I N E
// 
// ============================================================================

uint RRScost_ailalaine( const global struct BVHNode* bvhNode, const global uint* idx, const global float4* verts, const float3 O, const float3 D, const float3 rD, const float tmax )
{
	// traverse BVH
	float4 hit;
	hit.x = tmax;
	uint node = 0, stack[STACK_SIZE], stackPtr = 0;
	float cost = 0;
	while (1)
	{
		// fetch the node
		cost += 1.2f; // TODO: obtain somehow via tiny_bvh.h?
		const float4 lmin = bvhNode[node].lmin, lmax = bvhNode[node].lmax;
		const float4 rmin = bvhNode[node].rmin, rmax = bvhNode[node].rmax;
		const uint triCount = as_uint( rmin.w );
		if (triCount > 0)
		{
			// process leaf node
			const uint firstTri = as_uint( rmax.w );
			for (uint i = 0; i < triCount; i++)
			{
				cost += 1.0f; // TODO: obtain somehow via tiny_bvh.h?
				const uint triIdx = idx[firstTri + i];
				const global float4* tri = verts + 3 * triIdx;
				// triangle intersection - M�ller-Trumbore
				const float4 edge1 = tri[1] - tri[0], edge2 = tri[2] - tri[0];
				const float3 h = cross( D, edge2.xyz );
				const float a = dot( edge1.xyz, h );
				if (fabs( a ) < 0.0000001f) continue;
				const float f = 1 / a;
				const float3 s = O - tri[0].xyz;
				const float u = f * dot( s, h );
				const float3 q = cross( s, edge1.xyz );
				const float v = f * dot( D, q );
				if (u < 0 || v < 0 || u + v > 1) continue;
				const float d = f * dot( edge2.xyz, q );
				if (d > 0.0f && d < hit.x) hit = (float4)(d, u, v, as_float( triIdx ));
			}
			if (stackPtr == 0) break;
			node = stack[--stackPtr];
			continue;
		}
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
		float dist1 = tmina > tmaxa ? 1e30f : tmina;
		float dist2 = tminb > tmaxb ? 1e30f : tminb;
		// traverse nearest child first
		if (dist1 > dist2)
		{
			float h = dist1; dist1 = dist2; dist2 = h;
			uint t = left; left = right; right = t;
		}
		if (dist1 == 1e30f)
		{
			if (stackPtr == 0) break; else node = stack[--stackPtr];
		}
		else
		{
			node = left;
			if (dist2 != 1e30f) stack[stackPtr++] = right;
		}
	}
	// write back intersection result
	return (uint)cost;
}

float4 traverse_ailalaine( const global struct BVHNode* bvhNode, 
	const global uint* idx, const global float4* verts, const global uint* opmap,
	const float3 O, const float3 D, const float3 rD, const float tmax, uint* stepCount )
{
	// prepare slab test
	const float3 rO = O * -rD;
	// traverse BVH
	float4 hit = (float4)( tmax, 0, 0, 0 );
	uint node = 0, stack[STACK_SIZE], stackPtr = 0, steps = 0;
	while (1)
	{
		steps++;
		const float4 lmin = bvhNode[node].lmin, lmax = bvhNode[node].lmax;
		const float4 rmin = bvhNode[node].rmin, rmax = bvhNode[node].rmax;
		const uint triCount = as_uint( rmin.w );
		if (triCount > 0 /* leaf */)
		{
			const uint firstTri = as_uint( rmax.w );
			for (uint i = 0; i < triCount; i++)
			{
				const uint triIdx = idx[firstTri + i];
				const global float4* tri = verts + 3 * triIdx;
				const float4 edge1 = tri[1] - tri[0], edge2 = tri[2] - tri[0];
				const float3 h = cross( D, edge2.xyz );
				const float f = 1 / dot( edge1.xyz, h );
				const float3 s = O - tri[0].xyz;
				const float u = f * dot( s, h );
				const float3 q = cross( s, edge1.xyz );
				const float v = f * dot( D, q );
				if (u < 0 || v < 0 || u + v > 1) continue;
				const float d = f * dot( edge2.xyz, q );
				if (d <= 0.0f || d >= hit.x) continue;
				if (opmap)
				{
					const int row = (int)( (u + v) * 32.0f ), diag = (int)( (1 - u) * 32.0f );
					const int idx = (row * row) + (int)( v * 32.0f ) + (diag - (31 - row));
					if (!(opmap[triIdx * 32 + (idx >> 5)] & (1 << (idx & 31)))) continue;
				}
				hit = (float4)(d, u, v, as_float( triIdx ));
			}
			if (stackPtr == 0) break;
			node = stack[--stackPtr];
			continue;
		}
		uint left = as_uint( lmin.w ), right = as_uint( lmax.w );
		const float3 t1a = fma( lmin.xyz, rD, rO ), t2a = fma( lmax.xyz, rD, rO );
		const float3 t1b = fma( rmin.xyz, rD, rO ), t2b = fma( rmax.xyz, rD, rO );
		const float3 minta = fmin( t1a, t2a ), maxta = fmax( t1a, t2a );
		const float3 mintb = fmin( t1b, t2b ), maxtb = fmax( t1b, t2b );
		const float tmina = fmax( fmax( fmax( minta.x, minta.y ), minta.z ), 0 );
		const float tminb = fmax( fmax( fmax( mintb.x, mintb.y ), mintb.z ), 0 );
		const float tmaxa = fmin( fmin( fmin( maxta.x, maxta.y ), maxta.z ), hit.x );
		const float tmaxb = fmin( fmin( fmin( maxtb.x, maxtb.y ), maxtb.z ), hit.x );
		float dist1 = tmina > tmaxa ? 1e30f : tmina;
		float dist2 = tminb > tmaxb ? 1e30f : tminb;
		if (dist1 > dist2)
		{
			float h = dist1; dist1 = dist2; dist2 = h;
			uint t = left; left = right; right = t;
		}
		if (dist1 == 1e30f) { if (stackPtr == 0) break; else node = stack[--stackPtr]; }
		else { node = left; if (dist2 != 1e30f) stack[stackPtr++] = right; }
	}
	if (stepCount) *stepCount += steps;
	return hit;
}

bool isoccluded_ailalaine( 
	const global struct BVHNode* bvhNode, 
	const global uint* idx, const global float4* verts, const global uint* opmap, 
	const float3 O, const float3 D, const float3 rD, const float tmax )
{
	uint node = 0, stack[STACK_SIZE], stackPtr = 0;
	while (1)
	{
		const float4 lmin = bvhNode[node].lmin, lmax = bvhNode[node].lmax;
		const float4 rmin = bvhNode[node].rmin, rmax = bvhNode[node].rmax;
		const uint triCount = as_uint( rmin.w );
		if (triCount > 0 /* leaf */)
		{
			const uint firstTri = as_uint( rmax.w );
			for (uint i = 0; i < triCount; i++)
			{
				const uint triIdx = idx[firstTri + i];
				const global float4* tri = verts + 3 * triIdx;
				const float4 edge1 = tri[1] - tri[0], edge2 = tri[2] - tri[0];
				const float3 h = cross( D, edge2.xyz );
				const float f = 1 / dot( edge1.xyz, h );
				const float3 s = O - tri[0].xyz;
				const float u = f * dot( s, h );
				const float3 q = cross( s, edge1.xyz );
				const float v = f * dot( D, q );
				if (u < 0 || v < 0 || u + v > 1) continue;
				const float d = f * dot( edge2.xyz, q );
				if (d <= 0.0f || d >= tmax) continue;
				if (opmap)
				{
					const int row = (int)( (u + v) * 32.0f ), diag = (int)( (1 - u) * 32.0f );
					const int idx = (row * row) + (int)( v * 32.0f ) + (diag - (31 - row));
					if (!(opmap[triIdx * 32 + (idx >> 5)] & (1 << (idx & 31)))) continue;
				}
				return true;
			}
			if (stackPtr == 0) break;
			node = stack[--stackPtr];
			continue;
		}
		uint left = as_uint( lmin.w ), right = as_uint( lmax.w );
		const float3 t1a = (lmin.xyz - O) * rD, t2a = (lmax.xyz - O) * rD;
		const float3 t1b = (rmin.xyz - O) * rD, t2b = (rmax.xyz - O) * rD;
		const float3 minta = fmin( t1a, t2a ), maxta = fmax( t1a, t2a );
		const float3 mintb = fmin( t1b, t2b ), maxtb = fmax( t1b, t2b );
		const float tmina = fmax( fmax( fmax( minta.x, minta.y ), minta.z ), 0 );
		const float tminb = fmax( fmax( fmax( mintb.x, mintb.y ), mintb.z ), 0 );
		const float tmaxa = fmin( fmin( fmin( maxta.x, maxta.y ), maxta.z ), tmax );
		const float tmaxb = fmin( fmin( fmin( maxtb.x, maxtb.y ), maxtb.z ), tmax );
		float dist1 = tmina > tmaxa ? 1e30f : tmina;
		float dist2 = tminb > tmaxb ? 1e30f : tminb;
		if (dist1 > dist2)
		{
			float h = dist1; dist1 = dist2; dist2 = h;
			uint t = left; left = right; right = t;
		}
		if (dist1 == 1e30f) { if (stackPtr == 0) break; else node = stack[--stackPtr]; }
		else { node = left; if (dist2 != 1e30f) stack[stackPtr++] = right; }
	}
	return false;
}

void kernel batch_ailalaine( const global struct BVHNode* bvhNode, const global uint* idx, const global float4* verts, global struct Ray* rayData )
{
	// fetch ray
	const uint threadId = get_global_id( 0 );
	const float3 O = rayData[threadId].O.xyz;
	const float3 D = rayData[threadId].D.xyz;
	const float3 rD = rayData[threadId].rD.xyz;
	float4 hit = traverse_ailalaine( bvhNode, idx, 0, verts, O, D, rD, 1e30f, 0 );
	rayData[threadId].hit = hit;
}

void kernel batch_ailalaine_rrs( const global struct BVHNode* bvhNode, const global uint* idx, const global float4* verts, global struct Ray* rayData, global uint* rrsResult )
{
	// fetch ray
	const uint threadId = get_global_id( 0 );
	const float3 O = rayData[threadId].O.xyz;
	const float3 D = rayData[threadId].D.xyz;
	const float3 rD = rayData[threadId].rD.xyz;
	rrsResult[threadId] = RRScost_ailalaine( bvhNode, idx, verts, O, D, rD, 1e30f );
}