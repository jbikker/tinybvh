// ============================================================================
//
//        T R A V E R S E _ G P U 4 W A Y
// 
// ============================================================================

#ifdef BVH4_GPU_COMPRESSED_TRIS
#define STRIDE 4
#else
#define STRIDE 3
#endif

void IntersectTri( const unsigned vertIdx, const float3* O, const float3* D, float4* hit, const global float4* alt4Node ) 
{
#ifdef BVH4_GPU_COMPRESSED_TRIS
	const float4 T2 = alt4Node[vertIdx + 2];
	const float transS = T2.x * O->x + T2.y * O->y + T2.z * O->z + T2.w;
	const float transD = T2.x * D->x + T2.y * D->y + T2.z * D->z, d = -transS / transD;
	if (d <= 0 || d >= hit->x) return;
	const float4 T0 = alt4Node[vertIdx + 0], T1 = alt4Node[vertIdx + 1];
	const float3 I = *O + d * *D;
	const float u = T0.x * I.x + T0.y * I.y + T0.z * I.z + T0.w;
	const float v = T1.x * I.x + T1.y * I.y + T1.z * I.z + T1.w;
	const bool trihit = u >= 0 && v >= 0 && u + v < 1;
	if (trihit) *hit = (float4)(d, u, v, as_uint( alt4Node[vertIdx + 3].w ) );
#else
	const float4 edge2 = alt4Node[vertIdx + 2];
	const float4 edge1 = alt4Node[vertIdx + 1];
	const float4 v0 = alt4Node[vertIdx];
	const float3 h = cross( *D, edge2.xyz );
	const float a = dot( edge1.xyz, h );
	if (fabs( a ) < 0.0000001f) return;
	const float f = native_recip( a );
	const float3 s = *O - v0.xyz;
	const float u = f * dot( s, h );
	const float3 q = cross( s, edge1.xyz );
	const float v = f * dot( *D, q );
	if (u < 0 || v < 0 || u + v > 1) return;
	const float d = f * dot( edge2.xyz, q );
	if (d > 0.0f && d < hit->x) *hit = (float4)(d, u, v, v0.w);
#endif
}

bool TriOccluded( const unsigned vertIdx, const float3* O, const float3* D, float tmax, const global float4* alt4Node ) 
{
#ifdef BVH4_GPU_COMPRESSED_TRIS
	const float4 T2 = alt4Node[vertIdx + 2];
	const float transS = T2.x * O->x + T2.y * O->y + T2.z * O->z + T2.w;
	const float transD = T2.x * D->x + T2.y * D->y + T2.z * D->z, d = -transS / transD;
	if (d <= 0 || d >= tmax) return false;
	const float4 T0 = alt4Node[vertIdx + 0], T1 = alt4Node[vertIdx + 1];
	const float3 I = *O + d * *D;
	const float u = T0.x * I.x + T0.y * I.y + T0.z * I.z + T0.w;
	const float v = T1.x * I.x + T1.y * I.y + T1.z * I.z + T1.w;
	return u >= 0 && v >= 0 && u + v < 1;
#else
	const float4 edge2 = alt4Node[vertIdx + 2];
	const float4 edge1 = alt4Node[vertIdx + 1];
	const float4 v0 = alt4Node[vertIdx];
	const float3 h = cross( *D, edge2.xyz );
	const float a = dot( edge1.xyz, h );
	if (fabs( a ) < 0.0000001f) return false;
	const float f = native_recip( a );
	const float3 s = *O - v0.xyz;
	const float u = f * dot( s, h );
	const float3 q = cross( s, edge1.xyz );
	const float v = f * dot( *D, q );
	if (u < 0 || v < 0 || u + v > 1) return false;
	const float d = f * dot( edge2.xyz, q );
	return d > 0.0f && d < tmax;
#endif
}

float4 traverse_gpu4way( const global float4* alt4Node, const float3 O, const float3 D, const float3 rD, const float tmax )
{
	float4 hit;
	hit.x = tmax;
	// traverse the BVH
	const float4 zero4 = (float4)(0);
	unsigned offset = 0, stack[STACK_SIZE], stackPtr = 0;
	const unsigned smBase = get_local_id( 0 ) * 4;
	while (1)
	{
		// vectorized 4-wide quantized aabb intersection
		const float4 data0 = alt4Node[offset];
		const float4 data1 = alt4Node[offset + 1];
		const float4 data2 = alt4Node[offset + 2];
		const float4 cminx4 = convert_float4( as_uchar4( data0.w ) );
		const float4 cmaxx4 = convert_float4( as_uchar4( data1.w ) );
		const float4 cminy4 = convert_float4( as_uchar4( data2.x ) );
		const float3 bminO = (O - data0.xyz) * rD, rDe = rD * data1.xyz;
		const float4 cmaxy4 = convert_float4( as_uchar4( data2.y ) );
		const float4 cminz4 = convert_float4( as_uchar4( data2.z ) );
		const float4 cmaxz4 = convert_float4( as_uchar4( data2.w ) );
		const float4 t1x4 = cminx4 * rDe.xxxx - bminO.xxxx, t2x4 = cmaxx4 * rDe.xxxx - bminO.xxxx;
		const float4 t1y4 = cminy4 * rDe.yyyy - bminO.yyyy, t2y4 = cmaxy4 * rDe.yyyy - bminO.yyyy;
		const float4 t1z4 = cminz4 * rDe.zzzz - bminO.zzzz, t2z4 = cmaxz4 * rDe.zzzz - bminO.zzzz;
		uint4 data3 = as_uint4( alt4Node[offset + 3] );
		const float4 mintx4 = fmin( t1x4, t2x4 ), maxtx4 = fmax( t1x4, t2x4 );
		const float4 minty4 = fmin( t1y4, t2y4 ), maxty4 = fmax( t1y4, t2y4 );
		const float4 mintz4 = fmin( t1z4, t2z4 ), maxtz4 = fmax( t1z4, t2z4 );
		const float4 maxxy4 = select( mintx4, minty4, isless( mintx4, minty4 ) );
		const float4 maxyz4 = select( maxxy4, mintz4, isless( maxxy4, mintz4 ) );
		float4 dst4 = select( maxyz4, zero4, isless( maxyz4, zero4 ) );
		const float4 minxy4 = select( maxtx4, maxty4, isgreater( maxtx4, maxty4 ) );
		const float4 minyz4 = select( minxy4, maxtz4, isgreater( minxy4, maxtz4 ) );
		const float4 tmax4 = select( minyz4, hit.xxxx, isgreater( minyz4, hit.xxxx ) );
		dst4 = select( dst4, (float4)(1e30f), isgreater( dst4, tmax4 ) );
		// sort intersection distances - TODO: handle single-intersection case separately.
		if (dst4.x < dst4.z) dst4 = dst4.zyxw, data3 = data3.zyxw; // bertdobbelaere.github.io/sorting_networks.html
		if (dst4.y < dst4.w) dst4 = dst4.xwzy, data3 = data3.xwzy;
		if (dst4.x < dst4.y) dst4 = dst4.yxzw, data3 = data3.yxzw;
		if (dst4.z < dst4.w) dst4 = dst4.xywz, data3 = data3.xywz;
		if (dst4.y < dst4.z) dst4 = dst4.xzyw, data3 = data3.xzyw;
		// process results, starting with farthest child, so nearest ends on top of stack
		unsigned nextNode = 0;
		if (dst4.x < 1e30f) 
		{
			if ((data3.x >> 31) == 0) nextNode = data3.x; else
			{
				const unsigned triCount = (data3.x >> 16) & 0x7fff;
				for( int i = 0; i < triCount; i++ ) IntersectTri( (data3.x & 0xffff) + offset + i * STRIDE, &O, &D, &hit, alt4Node );
			}
		}
		if (dst4.y < 1e30f) 
		{
			if (data3.y >> 31)
			{
				const unsigned triCount = (data3.y >> 16) & 0x7fff;
				for( int i = 0; i < triCount; i++ ) IntersectTri( (data3.y & 0xffff) + offset + i * STRIDE, &O, &D, &hit, alt4Node );
			}
			else
			{
				if (nextNode) stack[stackPtr++] = nextNode;
				nextNode = data3.y;
			}
		}
		if (dst4.z < 1e30f) 
		{
			if (data3.z >> 31) 
			{
				const unsigned triCount = (data3.z >> 16) & 0x7fff;
				for( int i = 0; i < triCount; i++ ) IntersectTri( (data3.z & 0xffff) + offset + i * STRIDE, &O, &D, &hit, alt4Node );
			}
			else
			{
				if (nextNode) stack[stackPtr++] = nextNode;
				nextNode = data3.z;
			}
		}
		if (dst4.w < 1e30f) 
		{
			if (data3.w >> 31) 
			{
				const unsigned triCount = (data3.w >> 16) & 0x7fff;
				for( int i = 0; i < triCount; i++ ) IntersectTri( (data3.w & 0xffff) + offset + i * STRIDE, &O, &D, &hit, alt4Node );
			}
			else
			{
				if (nextNode) stack[stackPtr++] = nextNode;
				nextNode = data3.w;
			}
		}
		// continue with nearest node or first node on the stack
		if (nextNode) offset = nextNode; else
		{
			if (!stackPtr) break;
			offset = stack[--stackPtr];
		}
	}
	return hit;
}

bool isoccluded_gpu4way( const global float4* alt4Node, const float3 O, const float3 D, const float3 rD, const float tmax )
{
	// traverse the BVH
	const float4 zero4 = (float4)(0), t4 = (float4)(tmax);
	unsigned offset = 0, stack[STACK_SIZE], stackPtr = 0;
	const unsigned smBase = get_local_id( 0 ) * 4;
	while (1)
	{
		// vectorized 4-wide quantized aabb intersection
		const float4 data0 = alt4Node[offset];
		const float4 data1 = alt4Node[offset + 1];
		const float4 data2 = alt4Node[offset + 2];
		const float4 cminx4 = convert_float4( as_uchar4( data0.w ) );
		const float4 cmaxx4 = convert_float4( as_uchar4( data1.w ) );
		const float4 cminy4 = convert_float4( as_uchar4( data2.x ) );
		const float3 bminO = (O - data0.xyz) * rD, rDe = rD * data1.xyz;
		const float4 cmaxy4 = convert_float4( as_uchar4( data2.y ) );
		const float4 cminz4 = convert_float4( as_uchar4( data2.z ) );
		const float4 cmaxz4 = convert_float4( as_uchar4( data2.w ) );
		const float4 t1x4 = cminx4 * rDe.xxxx - bminO.xxxx, t2x4 = cmaxx4 * rDe.xxxx - bminO.xxxx;
		const float4 t1y4 = cminy4 * rDe.yyyy - bminO.yyyy, t2y4 = cmaxy4 * rDe.yyyy - bminO.yyyy;
		const float4 t1z4 = cminz4 * rDe.zzzz - bminO.zzzz, t2z4 = cmaxz4 * rDe.zzzz - bminO.zzzz;
		uint4 data3 = as_uint4( alt4Node[offset + 3] );
		const float4 mintx4 = fmin( t1x4, t2x4 ), maxtx4 = fmax( t1x4, t2x4 );
		const float4 minty4 = fmin( t1y4, t2y4 ), maxty4 = fmax( t1y4, t2y4 );
		const float4 mintz4 = fmin( t1z4, t2z4 ), maxtz4 = fmax( t1z4, t2z4 );
		const float4 maxxy4 = select( mintx4, minty4, isless( mintx4, minty4 ) );
		const float4 maxyz4 = select( maxxy4, mintz4, isless( maxxy4, mintz4 ) );
		float4 dst4 = select( maxyz4, zero4, isless( maxyz4, zero4 ) );
		const float4 minxy4 = select( maxtx4, maxty4, isgreater( maxtx4, maxty4 ) );
		const float4 minyz4 = select( minxy4, maxtz4, isgreater( minxy4, maxtz4 ) );
		const float4 tmax4 = select( minyz4, t4, isgreater( minyz4, t4 ) );
		dst4 = select( dst4, (float4)(1e30f), isgreater( dst4, tmax4 ) );
		// sort intersection distances - TODO: handle single-intersection case separately.
		if (dst4.x < dst4.z) dst4 = dst4.zyxw, data3 = data3.zyxw; // bertdobbelaere.github.io/sorting_networks.html
		if (dst4.y < dst4.w) dst4 = dst4.xwzy, data3 = data3.xwzy;
		if (dst4.x < dst4.y) dst4 = dst4.yxzw, data3 = data3.yxzw;
		if (dst4.z < dst4.w) dst4 = dst4.xywz, data3 = data3.xywz;
		if (dst4.y < dst4.z) dst4 = dst4.xzyw, data3 = data3.xzyw;
		// process results, starting with farthest child, so nearest ends on top of stack
		unsigned nextNode = 0;
		if (dst4.x < 1e30f) 
		{
			if ((data3.x >> 31) == 0) nextNode = data3.x; else
			{
				const unsigned triCount = (data3.x >> 16) & 0x7fff;
				for( int i = 0; i < triCount; i++ ) 
					if (TriOccluded( (data3.x & 0xffff) + offset + i * STRIDE, &O, &D, tmax, alt4Node )) return true;
			}
		}
		if (dst4.y < 1e30f) 
		{
			if (data3.y >> 31)
			{
				const unsigned triCount = (data3.y >> 16) & 0x7fff;
				for( int i = 0; i < triCount; i++ ) 
					if (TriOccluded( (data3.y & 0xffff) + offset + i * STRIDE, &O, &D, tmax, alt4Node )) return true;
			}
			else
			{
				if (nextNode) stack[stackPtr++] = nextNode;
				nextNode = data3.y;
			}
		}
		if (dst4.z < 1e30f) 
		{
			if (data3.z >> 31) 
			{
				const unsigned triCount = (data3.z >> 16) & 0x7fff;
				for( int i = 0; i < triCount; i++ ) 
					if (TriOccluded( (data3.z & 0xffff) + offset + i * STRIDE, &O, &D, tmax, alt4Node )) return true;
			}
			else
			{
				if (nextNode) stack[stackPtr++] = nextNode;
				nextNode = data3.z;
			}
		}
		if (dst4.w < 1e30f) 
		{
			if (data3.w >> 31) 
			{
				const unsigned triCount = (data3.w >> 16) & 0x7fff;
				for( int i = 0; i < triCount; i++ ) 
					if (TriOccluded( (data3.w & 0xffff) + offset + i * STRIDE, &O, &D, tmax, alt4Node )) return true;
			}
			else
			{
				if (nextNode) stack[stackPtr++] = nextNode;
				nextNode = data3.w;
			}
		}
		// continue with nearest node or first node on the stack
		if (nextNode) offset = nextNode; else
		{
			if (!stackPtr) break;
			offset = stack[--stackPtr];
		}
	}
	// no occlusion found.
	return false;
}

void kernel batch_gpu4way( const global float4* alt4Node, global struct Ray* rayData )
{
	// fetch ray
	const unsigned threadId = get_global_id( 0 );
	const float3 O = rayData[threadId].O.xyz;
	const float3 D = rayData[threadId].D.xyz;
	const float3 rD = rayData[threadId].rD.xyz;
	float4 hit = traverse_gpu4way( alt4Node, O, D, rD, 1e30f );
	rayData[threadId].hit = hit;
}

void kernel batch_gpu4way_any( const global float4* alt4Node, global struct Ray* rayData )
{
	// fetch ray
	const unsigned threadId = get_global_id( 0 );
	const float3 O = rayData[threadId].O.xyz;
	const float3 D = rayData[threadId].D.xyz;
	const float3 rD = rayData[threadId].rD.xyz;
	const float tmax = 1e30f; // TODO: get this from the ray.
	float4 hit = 0;
	if (isoccluded_gpu4way( alt4Node, O, D, rD, tmax )) hit.w = as_float( 1 );
	rayData[threadId].hit = hit;
}