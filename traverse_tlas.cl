// ============================================================================
//
//        T R A V E R S E _ T L A S
// 
// ============================================================================

// Here, the TLAS is expected to be in 'BVH_GPU' format, i.e. a 2-wide BVH
// in the layout proposed by Aila & Laine. This layout is chosen here for its 
// balance between efficient construction and traversal.
// For scenes with lots of BLAS nodes you may want to use a wider BVH, e.g.
// BVH4_GPU.

float4 traverse_tlas( const float4 O4, const float4 D4, const float4 rD4, const float tmax, uint* stepCount )
{
	// initialize return data
	float4 hit = (float4)(tmax, 0, 0, 0);
	// safety net
	if (isnan( O4.x + O4.y + O4.z + D4.x + D4.y + D4.z )) return hit;
	// traverse BVH
	unsigned node = 0, stack[STACK_SIZE], stackPtr = 0, steps = 0;
	if (stepCount) *stepCount = 0;
	while (1)
	{
		steps++;
		// fetch the node
		const float4 lmin = tlasNodes[node].lmin, lmax = tlasNodes[node].lmax;
		const float4 rmin = tlasNodes[node].rmin, rmax = tlasNodes[node].rmax;
		const unsigned triCount = as_uint( rmin.w );
		if (triCount > 0)
		{
			// process leaf node
			const unsigned firstTri = as_uint( rmax.w );
			for (unsigned i = 0; i < triCount; i++)
			{
				const uint instIdx = tlasIdx[firstTri + i];
				const struct Instance* inst = instances + instIdx;
				const float3 Oblas = TransformPoint( O4.xyz, inst->invTransform );
				const float3 Dblas = TransformVector( D4.xyz, inst->invTransform );
				const float3 rDblas = (float3)(1 / Dblas.x, 1 / Dblas.y, 1 / Dblas.z);
			#ifdef DEPRECATED_TLAS_PATH
				// this code exists only for the tiny_bvh_interop project and will go away.
				const global float4* blasNodes = instIdx == 0 ? bistroNodes : dragonNodes;
				const global float4* blasTris = instIdx == 0 ? bistroTris : dragonTris;
			#ifdef SIMD_AABBTEST
				const float4 blasHit = traverse_cwbvh( blasNodes, blasTris, (float4)(Oblas, 1), (float4)(Dblas, 0), (float4)(rDblas, 1), hit.x, stepCount );
			#else
				const float4 blasHit = traverse_cwbvh( blasNodes, blasTris, Oblas, Dblas, rDblas, hit.x, stepCount );
			#endif
			#else
				// this code handles arbitrary tlas/blas scenes.
				const uint blas = as_uint( inst->aabbMin.w );
				const uint opmapOffs = blasDesc[blas].opmapOffset;
				const global uint* opmap = opmapOffs == 0x99999999 ? 0 : (blasOpMap + opmapOffs);
				const uint blasType = blasDesc[blas].blasType;
				float4 blasHit;
				if (blasType == 4 /* GPU_STATIC */)
				{
					const global float4* nodes = blasCWNodes + blasDesc[blas].node8Offset * 5;
					const global float4* tris = blasTri8 + blasDesc[blas].tri8Offset * 4;
				#ifdef SIMD_AABBTEST
					blasHit = traverse_cwbvh( nodes, tris, (float4)(Oblas, 1), (float4)(Dblas, 0), (float4)(rDblas, 1), hit.x, stepCount );
				#else
					blasHit = traverse_cwbvh( nodes, tris, Oblas, Dblas, rDblas, hit.x, stepCount );
				#endif
				}
				else // if (blasType == 2 || blasType == 3 /* GPU_DYNAMIC or GPU_RIGID */)
				{
					const global struct BVHNode* nodes = blasNodes + blasDesc[blas].nodeOffset; // TODO: read offset data as uint4
					const global uint* idx = blasIdx + blasDesc[blas].indexOffset;
					const global float4* tris = blasTris + blasDesc[blas].triOffset * 3;
					blasHit = traverse_ailalaine( nodes, idx, tris, opmap, Oblas, Dblas, rDblas, hit.x, stepCount );
				}
			#endif
				if (blasHit.x < hit.x)
				{
					hit = blasHit;
					hit.w = as_float( as_uint( hit.w ) + (instIdx << 24) );
				}
			}
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		unsigned left = as_uint( lmin.w ), right = as_uint( lmax.w );
		// child AABB intersection tests
		const float3 t1a = (lmin.xyz - O4.xyz) * rD4.xyz, t2a = (lmax.xyz - O4.xyz) * rD4.xyz;
		const float3 t1b = (rmin.xyz - O4.xyz) * rD4.xyz, t2b = (rmax.xyz - O4.xyz) * rD4.xyz;
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
		if (dist1 == 1e30f) { if (stackPtr == 0) break; else node = stack[--stackPtr]; }
		else { node = left; if (dist2 != 1e30f) stack[stackPtr++] = right; }
	}
	// write back intersection result
	if (stepCount) *stepCount += steps;
	return hit;
}

bool isoccluded_tlas( const float4 O4, const float4 D4, const float4 rD4, const float tmax )
{
	// traverse BVH
	unsigned node = 0, stack[STACK_SIZE], stackPtr = 0;
	// safety net
	if (isnan( O4.x + O4.y + O4.z + D4.x + D4.y + D4.z )) return true;
	// traverse
	while (1)
	{
		// fetch the node
		const float4 lmin = tlasNodes[node].lmin, lmax = tlasNodes[node].lmax;
		const float4 rmin = tlasNodes[node].rmin, rmax = tlasNodes[node].rmax;
		const unsigned triCount = as_uint( rmin.w );
		if (triCount > 0)
		{
			// process leaf node
			const unsigned firstTri = as_uint( rmax.w );
			for (unsigned i = 0; i < triCount; i++)
			{
				const uint instIdx = tlasIdx[firstTri + i];
				const struct Instance* inst = instances + instIdx;
				const float3 Oblas = TransformPoint( O4.xyz, inst->invTransform );
				const float3 Dblas = TransformVector( D4.xyz, inst->invTransform );
				const float3 rDblas = (float3)(1 / Dblas.x, 1 / Dblas.y, 1 / Dblas.z);
			#ifdef DEPRECATED_TLAS_PATH
				// this code exists only for the tiny_bvh_interop project and will go away.
				const global float4* blasNodes = instIdx == 0 ? bistroNodes : dragonNodes;
				const global float4* blasTris = instIdx == 0 ? bistroTris : dragonTris;
			#ifdef SIMD_AABBTEST
				if (isoccluded_cwbvh( blasNodes, blasTris, (float4)(Oblas, 1), (float4)(Dblas, 0), (float4)(rDblas, 1), D4.w )) return true;
			#else
				if (isoccluded_cwbvh( blasNodes, blasTris, Oblas, Dblas, rDblas, D4.w )) return true;
			#endif
			#else
				// this code handles arbitrary tlas/blas scenes.
				const uint blas = as_uint( inst->aabbMin.w );
				const uint opmapOffs = blasDesc[blas].opmapOffset;
				const global uint* opmap = opmapOffs == 0x99999999 ? 0 : (blasOpMap + opmapOffs);
				const uint blasType = blasDesc[blas].blasType;
				if (blasType == 4 /* GPU_STATIC */)
				{
					const global float4* nodes = blasCWNodes + blasDesc[blas].node8Offset * 5;
					const global float4* tris = blasTri8 + blasDesc[blas].tri8Offset * 4;
				#ifdef SIMD_AABBTEST
					if (isoccluded_cwbvh( nodes, tris, (float4)(Oblas, 1), (float4)(Dblas, 0), (float4)(rDblas, 1), tmax )) return true;
				#else
					if (isoccluded_cwbvh( nodes, tris, Oblas, Dblas, rDblas, tmax )) return true;
				#endif
				}
				else // if (blasType == 2 || blasType == 3 /* GPU_DYNAMIC or GPU_RIGID */)
				{
					const global struct BVHNode* nodes = blasNodes + blasDesc[blas].nodeOffset; // TODO: read offset data as uint4
					const global uint* idx = blasIdx + blasDesc[blas].indexOffset;
					const global float4* tris = blasTris + blasDesc[blas].triOffset * 3;
					if (isoccluded_ailalaine( nodes, idx, tris, opmap, Oblas, Dblas, rDblas, tmax )) return true;
				}
			#endif
			}
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		unsigned left = as_uint( lmin.w ), right = as_uint( lmax.w );
		// child AABB intersection tests
		const float3 t1a = (lmin.xyz - O4.xyz) * rD4.xyz, t2a = (lmax.xyz - O4.xyz) * rD4.xyz;
		const float3 t1b = (rmin.xyz - O4.xyz) * rD4.xyz, t2b = (rmax.xyz - O4.xyz) * rD4.xyz;
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
		if (dist1 == 1e30f) { if (stackPtr == 0) break; else node = stack[--stackPtr]; }
		else { node = left; if (dist2 != 1e30f) stack[stackPtr++] = right; }
	}
	// no hit found
	return false;
}