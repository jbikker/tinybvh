// ============================================================================
//
//        T R A V E R S E _ A I L A L A I N E
// 
// ============================================================================

struct BVHNodeAlt
{
	float4 lmin; // unsigned left in w
	float4 lmax; // unsigned right in w
	float4 rmin; // unsigned triCount in w
	float4 rmax; // unsigned firstTri in w
};

uint RRScost_ailalaine( global struct BVHNodeAlt* altNode, global unsigned* idx, global float4* verts, const float3 O, const float3 D, const float3 rD, const float tmax )
{
	// traverse BVH
	float4 hit;
	hit.x = tmax;
	unsigned node = 0, stack[STACK_SIZE], stackPtr = 0;
	float cost = 0;
	while (1)
	{
		// fetch the node
		cost += 1.2f; // TODO: obtain somehow via tiny_bvh.h?
		const float4 lmin = altNode[node].lmin, lmax = altNode[node].lmax;
		const float4 rmin = altNode[node].rmin, rmax = altNode[node].rmax;
		const unsigned triCount = as_uint( rmin.w );
		if (triCount > 0)
		{
			// process leaf node
			const unsigned firstTri = as_uint( rmax.w );
			for (unsigned i = 0; i < triCount; i++)
			{
				cost += 1.0f; // TODO: obtain somehow via tiny_bvh.h?
				const unsigned triIdx = idx[firstTri + i];
#ifdef ISAPPLE
				// FIX error: initializing 'const __private float4 *__private' with an expression of type '__global float4 *' changes address space of pointer
				const float4 tri[3] = 
				{
					verts[3 * triIdx],
					verts[3 * triIdx + 1],
					verts[3 * triIdx + 2],
				};
#else
				const float4* tri = verts + 3 * triIdx;
#endif
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
		unsigned left = as_uint( lmin.w ), right = as_uint( lmax.w );
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
			unsigned t = left; left = right; right = t;
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

float4 traverse_ailalaine( global struct BVHNodeAlt* altNode, global unsigned* idx, global float4* verts, const float3 O, const float3 D, const float3 rD, const float tmax )
{
	// traverse BVH
	float4 hit;
	hit.x = tmax;
	unsigned node = 0, stack[STACK_SIZE], stackPtr = 0;
	while (1)
	{
		// fetch the node
		const float4 lmin = altNode[node].lmin, lmax = altNode[node].lmax;
		const float4 rmin = altNode[node].rmin, rmax = altNode[node].rmax;
		const unsigned triCount = as_uint( rmin.w );
		if (triCount > 0)
		{
			// process leaf node
			const unsigned firstTri = as_uint( rmax.w );
			for (unsigned i = 0; i < triCount; i++)
			{
				const unsigned triIdx = idx[firstTri + i];
#ifdef ISAPPLE
				// FIX error: initializing 'const __private float4 *__private' with an expression of type '__global float4 *' changes address space of pointer
				const float4 tri[3] = 
				{
					verts[3 * triIdx],
					verts[3 * triIdx + 1],
					verts[3 * triIdx + 2],
				};
#else
				const float4* tri = verts + 3 * triIdx;
#endif
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
		unsigned left = as_uint( lmin.w ), right = as_uint( lmax.w );
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
			unsigned t = left; left = right; right = t;
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
	return hit;
}

bool isoccluded_ailalaine( global struct BVHNodeAlt* altNode, global unsigned* idx, global float4* verts, const float3 O, const float3 D, const float3 rD, const float tmax )
{
	// traverse BVH
	unsigned node = 0, stack[STACK_SIZE], stackPtr = 0;
	while (1)
	{
		// fetch the node
		const float4 lmin = altNode[node].lmin, lmax = altNode[node].lmax;
		const float4 rmin = altNode[node].rmin, rmax = altNode[node].rmax;
		const unsigned triCount = as_uint( rmin.w );
		if (triCount > 0)
		{
			// process leaf node
			const unsigned firstTri = as_uint( rmax.w );
			for (unsigned i = 0; i < triCount; i++)
			{
				const unsigned triIdx = idx[firstTri + i];
#ifdef ISAPPLE
				// FIX error: initializing 'const __private float4 *__private' with an expression of type '__global float4 *' changes address space of pointer
				const float4 tri[3] = 
				{
					verts[3 * triIdx],
					verts[3 * triIdx + 1],
					verts[3 * triIdx + 2],
				};
#else
				const float4* tri = verts + 3 * triIdx;
#endif
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
				if (d > 0.0f && d < tmax) return true;
			}
			if (stackPtr == 0) break;
			node = stack[--stackPtr];
			continue;
		}
		unsigned left = as_uint( lmin.w ), right = as_uint( lmax.w );
		// child AABB intersection tests
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
		// traverse nearest child first
		if (dist1 > dist2)
		{
			float h = dist1; dist1 = dist2; dist2 = h;
			unsigned t = left; left = right; right = t;
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
	// no hit found
	return false;
}

void kernel batch_ailalaine( global struct BVHNodeAlt* altNode, global unsigned* idx, global float4* verts, global struct Ray* rayData )
{
	// fetch ray
	const unsigned threadId = get_global_id( 0 );
	const float3 O = rayData[threadId].O.xyz;
	const float3 D = rayData[threadId].D.xyz;
	const float3 rD = rayData[threadId].rD.xyz;
	float4 hit = traverse_ailalaine( altNode, idx, verts, O, D, rD, 1e30f );
	rayData[threadId].hit = hit;
}

void kernel batch_ailalaine_rrs( global struct BVHNodeAlt* altNode, global unsigned* idx, global float4* verts, global struct Ray* rayData, global uint* rrsResult )
{
	// fetch ray
	const unsigned threadId = get_global_id( 0 );
	const float3 O = rayData[threadId].O.xyz;
	const float3 D = rayData[threadId].D.xyz;
	const float3 rD = rayData[threadId].rD.xyz;
	rrsResult[threadId] = RRScost_ailalaine( altNode, idx, verts, O, D, rD, 1e30f );
}