	const global struct BVHNode* bvhNode, 
	const global unsigned* idx, const global float4* verts, const global uint* opmap,
	const float3 O, const float3 D, const float3 rD, const float tmax, uint* stepCount )
{
	// prepare slab test
#ifdef NO_TEMPLATED_INTERSECT
	const float3 rO = O * -rD;
#else
	const float rox = O.x * -rD.x;
	const float roy = O.y * -rD.y;
	const float roz = O.z * -rD.z;
#endif
	// traverse BVH
	float4 hit = (float4)( tmax, 0, 0, 0 );
	unsigned node = 0, stack[STACK_SIZE], stackPtr = 0, steps = 0;
	while (1)
	{
		steps++;
		const float4 lmin = bvhNode[node].lmin, lmax = bvhNode[node].lmax;
		const float4 rmin = bvhNode[node].rmin, rmax = bvhNode[node].rmax;
		const unsigned triCount = as_uint( rmin.w );
		if (triCount > 0 /* leaf */)
		{
			const unsigned firstTri = as_uint( rmax.w );
			for (unsigned i = 0; i < triCount; i++)
			{
				const unsigned triIdx = idx[firstTri + i];
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
		unsigned left = as_uint( lmin.w ), right = as_uint( lmax.w );

	#ifndef NO_TEMPLATED_INTERSECT

		// "templated"
	#if XYZDIR & 1
		float tx1a = fma( lmin.x, rD.x, rox ), tx1b = fma( rmin.x, rD.x, rox );
		float tx2a = fma( lmax.x, rD.x, rox ), tx2b = fma( rmax.x, rD.x, rox );
	#else
		float tx1a = fma( lmax.x, rD.x, rox ), tx1b = fma( rmax.x, rD.x, rox );
		float tx2a = fma( lmin.x, rD.x, rox ), tx2b = fma( rmin.x, rD.x, rox );
	#endif
	#if XYZDIR & 2
		float ty1a = fma( lmin.y, rD.y, roy ), ty1b = fma( rmin.y, rD.y, roy );
		float ty2a = fma( lmax.y, rD.y, roy ), ty2b = fma( rmax.y, rD.y, roy );
	#else
		float ty1a = fma( lmax.y, rD.y, roy ), ty1b = fma( rmax.y, rD.y, roy );
		float ty2a = fma( lmin.y, rD.y, roy ), ty2b = fma( rmin.y, rD.y, roy );
	#endif
	#if XYZDIR & 4
		float tz1a = fma( lmin.z, rD.z, roz ), tz1b = fma( rmin.z, rD.z, roz );
		float tz2a = fma( lmax.z, rD.z, roz ), tz2b = fma( rmax.z, rD.z, roz );
	#else
		float tz1a = fma( lmax.z, rD.z, roz ), tz1b = fma( rmax.z, rD.z, roz );
		float tz2a = fma( lmin.z, rD.z, roz ), tz2b = fma( rmin.z, rD.z, roz );
	#endif
		float tmina = max( max( tx1a, ty1a ), max( tz1a, 0.0f ) );
		float tminb = max( max( tx1b, ty1b ), max( tz1b, 0.0f ) );
		float tmaxa = min( min( tx2a, ty2a ), min( tz2a, hit.x ) );
		float tmaxb = min( min( tx2b, ty2b ), min( tz2b, hit.x ) );
		float dist1 = tmina > tmaxa ? 1e30f : tmina;
		float dist2 = tminb > tmaxb ? 1e30f : tminb;

	#else

		// conventional
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

	#endif

		if (dist1 > dist2)
		{
			float h = dist1; dist1 = dist2; dist2 = h;
			unsigned t = left; left = right; right = t;
		}
		if (dist1 == 1e30f) { if (stackPtr == 0) break; else node = stack[--stackPtr]; }
		else { node = left; if (dist2 != 1e30f) stack[stackPtr++] = right; }
	}
	if (stepCount) *stepCount += steps;
	return hit;
}