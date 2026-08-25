// ============================================================================
//
//        T R A V E R S E _ C W B V H
// 
// ============================================================================

// preliminaries

#define CWBVH_STACK 24

inline uint __bfind( const uint v ) // OpenCL alternative for CUDA's native __bfind
{
	// see https://docs.nvidia.com/cuda/parallel-thread-execution/#integer-arithmetic-instructions-bfind
#ifdef ISNVIDIA
	uint b;
	asm ("bfind.u32 %0, %1; " : "=r"(b) : "r"(v));
	return b;
#else
	return 31 ^ clz( v ); // only correct if v cannot be zero, which is the case in traverse_cwbvh.
#endif
}

inline uint __popc( const uint v ) // OpenCL alternative for CUDA's native __popc
{
	// CUDA documentation: "Count the number of bits that are set to 1 in an integer."
#ifdef ISNVIDIA
	int p;
	asm ("popc.b32 %0, %1; " : "=r"(p) : "r"(v));
	return p; // note: identical performance to OpenCL popcount?
#else
	return popcount( v );
#endif
}

inline float _native_fma( const float a, const float b, const float c )
{
#ifdef ISNVIDIA
	float d;
	asm ("fma.rz.f32 %0, %1, %2, %3;" : "=f"(d) : "f"(a), "f"(b), "f"(c));
	return d;
#else
#ifdef FP_FAST_FMAF // https://registry.khronos.org/OpenCL/specs/3.0-unified/html/OpenCL_C.html
	return fma( a, b, c );
#else
	return a * b + c;
#endif
#endif
}

inline float fmin_fmin( const float a, const float b, const float c )
{
#if defined( ISNVIDIA ) && defined( ISPASCAL )
	// not a win on Turing
	return as_float( min_min( as_int( a ), as_int( b ), as_int( c ) ) );
#else
	return fmin( fmin( a, b ), c );
#endif
}

inline float fmax_fmax( const float a, const float b, const float c )
{
#if defined( ISNVIDIA ) && defined( ISPASCAL )
	return as_float( max_max( as_int( a ), as_int( b ), as_int( c ) ) );
#else
	return fmax( fmax( a, b ), c );
#endif
}

#ifdef USE_VLOAD_VSTORE
#define STACK_POP(X) { unsigned* a = (unsigned*)&stack[--stackPtr]; X = vload2( 0, a ); }
#define STACK_PUSH(X) { unsigned* a = (unsigned*)&stack[stackPtr++]; vstore2( X, 0, a ); }
#else
#define STACK_POP(X) { X = stack[--stackPtr]; }
#define STACK_PUSH(X) { stack[stackPtr++] = X; }
#endif
#ifdef ISNVIDIA
#define UPDATE_HITMASK asm( "vshl.u32.u32.u32.wrap.add %0,%1.b0, %2.b0, %3;" : "=r"(hitmask) : "r"(child_bits4), "r"(bit_index4), "r"(hitmask) );
#define UPDATE_HITMASK0 asm( "vshl.u32.u32.u32.wrap.add %0,%1.b0, %2.b0, %3;" : "=r"(hitmask) : "r"(child_bits4), "r"(bit_index4), "r"(hitmask) );
#define UPDATE_HITMASK1 asm( "vshl.u32.u32.u32.wrap.add %0,%1.b1, %2.b1, %3;" : "=r"(hitmask) : "r"(child_bits4), "r"(bit_index4), "r"(hitmask) );
#define UPDATE_HITMASK2 asm( "vshl.u32.u32.u32.wrap.add %0,%1.b2, %2.b2, %3;" : "=r"(hitmask) : "r"(child_bits4), "r"(bit_index4), "r"(hitmask) );
#define UPDATE_HITMASK3 asm( "vshl.u32.u32.u32.wrap.add %0,%1.b3, %2.b3, %3;" : "=r"(hitmask) : "r"(child_bits4), "r"(bit_index4), "r"(hitmask) );
#else
#define UPDATE_HITMASK hitmask = (child_bits4 & 255) << (bit_index4 & 31)
#define UPDATE_HITMASK0 hitmask |= (child_bits4 & 255) << (bit_index4 & 31)
#define UPDATE_HITMASK1 hitmask |= ((child_bits4 >> 8) & 255) << ((bit_index4 >> 8) & 31);
#define UPDATE_HITMASK2 hitmask |= ((child_bits4 >> 16) & 255) << ((bit_index4 >> 16) & 31);
#define UPDATE_HITMASK3 hitmask |= (child_bits4 >> 24) << (bit_index4 >> 24);
#endif

#define QUNPACK_SCALE	256.0f
#define QUNPACK_BIAS	1.0f
inline float4 unpack_q8x4( const uint w )
{
	const uint m = 0x007F8000u, b = 0x3F800000u; // mantissa slot 22..15, exponent 0
#ifndef ISNVIDIA
	return as_float4( ((uint4)(w << 15, w << 7, w >> 1, w >> 9) & m) | b );
#else
	return (float4)(
		as_float( ((w << 15) & m) | b ), as_float( ((w <<  7) & m) | b ),
		as_float( ((w >>  1) & m) | b ), as_float( ((w >>  9) & m) | b ) );
#endif
}

#ifdef SIMD_AABBTEST
#define float3or4 float4
#else
#define float3or4 float3
#endif

// cwbvh kernel - based on CUDA code by AlanWBFT https://github.com/AlanIWBFT

#ifdef SIMD_AABBTEST
float4 traverse_cwbvh( global const float4* cwbvhNodes, global const float4* cwbvhTris, const float4 O, const float4 D, const float4 rD, const float t )
#else
float4 traverse_cwbvh( global const float4* cwbvhNodes, global const float4* cwbvhTris, const float3 O, const float3 D, const float3 rD, const float t )
#endif
{
	// prepare traversal
	uint2 stack[CWBVH_STACK];
	uint hitAddr, stackPtr = 0;
	float2 uv;
	float4 hit;
	float tmax = t;
	const uint octinv4 = (7 - ((D.x < 0 ? 4 : 0) | (D.y < 0 ? 2 : 0) | (D.z < 0 ? 1 : 0))) * 0x1010101;
	const float3or4 rDs = rD * QUNPACK_SCALE; // per-ray, hoisted out of the node loop.
	uint2 ngroup = (uint2)(0, 0b10000000000000000000000000000000), tgroup = (uint2)(0);
	do
	{
		if (ngroup.y > 0x00FFFFFF)
		{
			const unsigned hits = ngroup.y, imask = ngroup.y;
			const unsigned child_bit_index = __bfind( hits );
			const unsigned child_node_base_index = ngroup.x;
			ngroup.y &= ~(1 << child_bit_index);
			if (ngroup.y > 0x00FFFFFF) { STACK_PUSH( ngroup ); }
			{
				const unsigned slot_index = (child_bit_index - 24) ^ (octinv4 & 255);
				const unsigned relative_index = __popc( imask & ~(0xFFFFFFFF << slot_index) );
				const unsigned child_node_index = (child_node_base_index + relative_index) * 5;
			#ifdef USE_VLOAD_VSTORE
				const float* p = (float*)&cwbvhNodes[child_node_index + 0];
				float4 n0 = vload4( 0, p ), n1 = vload4( 1, p ), n2 = vload4( 2, p );
				float4 n3 = vload4( 3, p ), n4 = vload4( 4, p );
			#else
				float4 n0 = cwbvhNodes[child_node_index + 0], n1 = cwbvhNodes[child_node_index + 1];
				float4 n2 = cwbvhNodes[child_node_index + 2], n3 = cwbvhNodes[child_node_index + 3];
				float4 n4 = cwbvhNodes[child_node_index + 4];
			#endif
				// n0.w packs three 127-biased quantization exponents plus imask, so each
				// byte is already an IEEE-754 exponent field: (b << 23) is 2^e directly.
				const uint e4 = as_uint( n0.w );
				ngroup.x = as_uint( n1.x ), tgroup = (uint2)(as_uint( n1.y ), 0);
				unsigned hitmask = 0;
			#ifdef SIMD_AABBTEST
				const float4 idir4 = (float4)( as_float( (e4 & 255) << 23 ) * rDs.x, 
					as_float( ((e4 >> 8) & 255) << 23 ) * rDs.y, as_float( ((e4 >> 16) & 255) << 23 ) * rDs.z, 1 );
				const float4 orig4 = (n0 - O) * rD - QUNPACK_BIAS * idir4;
			#else
				const float idirx = as_float( (e4 & 255) << 23 ) * rDs.x;
				const float idiry = as_float( ((e4 >> 8) & 255) << 23 ) * rDs.y;
				const float idirz = as_float( ((e4 >> 16) & 255) << 23 ) * rDs.z;
				const float origx = (n0.x - O.x) * rD.x - QUNPACK_BIAS * idirx;
				const float origy = (n0.y - O.y) * rD.y - QUNPACK_BIAS * idiry;
				const float origz = (n0.z - O.z) * rD.z - QUNPACK_BIAS * idirz;
			#endif
				{	// first 4
					const unsigned meta4 = as_uint( n1.z ), is_inner4 = (meta4 & (meta4 << 1)) & 0x10101010;
					const unsigned inner_mask4 = (is_inner4 << 4) - (is_inner4 >> 4);
					const unsigned bit_index4 = (meta4 ^ (octinv4 & inner_mask4)) & 0x1F1F1F1F;
					const unsigned child_bits4 = (meta4 >> 5) & 0x07070707;
					const float4 lox4 = unpack_q8x4( as_uint( rD.x < 0 ? n3.z : n2.x ) ), hix4 = unpack_q8x4( as_uint( rD.x < 0 ? n2.x : n3.z ) );
					const float4 loy4 = unpack_q8x4( as_uint( rD.y < 0 ? n4.x : n2.z ) ), hiy4 = unpack_q8x4( as_uint( rD.y < 0 ? n2.z : n4.x ) );
					const float4 loz4 = unpack_q8x4( as_uint( rD.z < 0 ? n4.z : n3.x ) ), hiz4 = unpack_q8x4( as_uint( rD.z < 0 ? n3.x : n4.z ) );
					{
					#ifdef SIMD_AABBTEST
						const float4 tminx4 = lox4 * idir4.xxxx + orig4.xxxx, tmaxx4 = hix4 * idir4.xxxx + orig4.xxxx;
						const float4 tminy4 = loy4 * idir4.yyyy + orig4.yyyy, tmaxy4 = hiy4 * idir4.yyyy + orig4.yyyy;
						const float4 tminz4 = loz4 * idir4.zzzz + orig4.zzzz, tmaxz4 = hiz4 * idir4.zzzz + orig4.zzzz;
						const float cmina = fmax( fmax( fmax( tminx4.x, tminy4.x ), tminz4.x ), 0 );
						const float cmaxa = fmin( fmin( fmin( tmaxx4.x, tmaxy4.x ), tmaxz4.x ), tmax );
						const float cminb = fmax( fmax( fmax( tminx4.y, tminy4.y ), tminz4.y ), 0 );
						const float cmaxb = fmin( fmin( fmin( tmaxx4.y, tmaxy4.y ), tmaxz4.y ), tmax );
						if (cmina <= cmaxa) UPDATE_HITMASK;
						if (cminb <= cmaxb) UPDATE_HITMASK1;
						const float cminc = fmax( fmax( fmax( tminx4.z, tminy4.z ), tminz4.z ), 0 );
						const float cmaxc = fmin( fmin( fmin( tmaxx4.z, tmaxy4.z ), tmaxz4.z ), tmax );
						const float cmind = fmax( fmax( fmax( tminx4.w, tminy4.w ), tminz4.w ), 0 );
						const float cmaxd = fmin( fmin( fmin( tmaxx4.w, tmaxy4.w ), tmaxz4.w ), tmax );
						if (cminc <= cmaxc) UPDATE_HITMASK2;
						if (cmind <= cmaxd) UPDATE_HITMASK3;
					#else
						float tminx0 = _native_fma( lox4.x, idirx, origx ), tminx1 = _native_fma( lox4.y, idirx, origx );
						float tminy0 = _native_fma( loy4.x, idiry, origy ), tminy1 = _native_fma( loy4.y, idiry, origy );
						float tminz0 = _native_fma( loz4.x, idirz, origz ), tminz1 = _native_fma( loz4.y, idirz, origz );
						float tmaxx0 = _native_fma( hix4.x, idirx, origx ), tmaxx1 = _native_fma( hix4.y, idirx, origx );
						float tmaxy0 = _native_fma( hiy4.x, idiry, origy ), tmaxy1 = _native_fma( hiy4.y, idiry, origy );
						float tmaxz0 = _native_fma( hiz4.x, idirz, origz ), tmaxz1 = _native_fma( hiz4.y, idirz, origz );
						n0.x = fmax( fmax_fmax( tminx0, tminy0, tminz0 ), 0 );
						n0.y = fmin( fmin_fmin( tmaxx0, tmaxy0, tmaxz0 ), tmax );
						n1.x = fmax( fmax_fmax( tminx1, tminy1, tminz1 ), 0 );
						n1.y = fmin( fmin_fmin( tmaxx1, tmaxy1, tmaxz1 ), tmax );
						if (n0.x <= n0.y) UPDATE_HITMASK;
						if (n1.x <= n1.y) UPDATE_HITMASK1;
						tminx0 = _native_fma( lox4.z, idirx, origx ), tminx1 = _native_fma( lox4.w, idirx, origx );
						tminy0 = _native_fma( loy4.z, idiry, origy ), tminy1 = _native_fma( loy4.w, idiry, origy );
						tminz0 = _native_fma( loz4.z, idirz, origz ), tminz1 = _native_fma( loz4.w, idirz, origz );
						tmaxx0 = _native_fma( hix4.z, idirx, origx ), tmaxx1 = _native_fma( hix4.w, idirx, origx );
						tmaxy0 = _native_fma( hiy4.z, idiry, origy ), tmaxy1 = _native_fma( hiy4.w, idiry, origy );
						tmaxz0 = _native_fma( hiz4.z, idirz, origz ), tmaxz1 = _native_fma( hiz4.w, idirz, origz );
						n0.x = fmax( fmax_fmax( tminx0, tminy0, tminz0 ), 0 );
						n0.y = fmin( fmin_fmin( tmaxx0, tmaxy0, tmaxz0 ), tmax );
						n1.x = fmax( fmax_fmax( tminx1, tminy1, tminz1 ), 0 );
						n1.y = fmin( fmin_fmin( tmaxx1, tmaxy1, tmaxz1 ), tmax );
						if (n0.x <= n0.y) UPDATE_HITMASK2;
						if (n1.x <= n1.y) UPDATE_HITMASK3;
					#endif
					}
				}
				{	// second 4
					const unsigned meta4 = as_uint( n1.w ), is_inner4 = (meta4 & (meta4 << 1)) & 0x10101010;
					const unsigned inner_mask4 = (is_inner4 << 4) - (is_inner4 >> 4);
					const unsigned bit_index4 = (meta4 ^ (octinv4 & inner_mask4)) & 0x1F1F1F1F;
					const unsigned child_bits4 = (meta4 >> 5) & 0x07070707;
					const float4 lox4 = unpack_q8x4( as_uint( rD.x < 0 ? n3.w : n2.y ) ), hix4 = unpack_q8x4( as_uint( rD.x < 0 ? n2.y : n3.w ) );
					const float4 loy4 = unpack_q8x4( as_uint( rD.y < 0 ? n4.y : n2.w ) ), hiy4 = unpack_q8x4( as_uint( rD.y < 0 ? n2.w : n4.y ) );
					const float4 loz4 = unpack_q8x4( as_uint( rD.z < 0 ? n4.w : n3.y ) ), hiz4 = unpack_q8x4( as_uint( rD.z < 0 ? n3.y : n4.w ) );
					{
					#ifdef SIMD_AABBTEST
						const float4 tminx4 = lox4 * idir4.xxxx + orig4.xxxx, tmaxx4 = hix4 * idir4.xxxx + orig4.xxxx;
						const float4 tminy4 = loy4 * idir4.yyyy + orig4.yyyy, tmaxy4 = hiy4 * idir4.yyyy + orig4.yyyy;
						const float4 tminz4 = loz4 * idir4.zzzz + orig4.zzzz, tmaxz4 = hiz4 * idir4.zzzz + orig4.zzzz;
						const float cmina = fmax( fmax( fmax( tminx4.x, tminy4.x ), tminz4.x ), 0 );
						const float cmaxa = fmin( fmin( fmin( tmaxx4.x, tmaxy4.x ), tmaxz4.x ), tmax );
						const float cminb = fmax( fmax( fmax( tminx4.y, tminy4.y ), tminz4.y ), 0 );
						const float cmaxb = fmin( fmin( fmin( tmaxx4.y, tmaxy4.y ), tmaxz4.y ), tmax );
						if (cmina <= cmaxa) UPDATE_HITMASK0;
						if (cminb <= cmaxb) UPDATE_HITMASK1;
						const float cminc = fmax( fmax( fmax( tminx4.z, tminy4.z ), tminz4.z ), 0 );
						const float cmaxc = fmin( fmin( fmin( tmaxx4.z, tmaxy4.z ), tmaxz4.z ), tmax );
						const float cmind = fmax( fmax( fmax( tminx4.w, tminy4.w ), tminz4.w ), 0 );
						const float cmaxd = fmin( fmin( fmin( tmaxx4.w, tmaxy4.w ), tmaxz4.w ), tmax );
						if (cminc <= cmaxc) UPDATE_HITMASK2;
						if (cmind <= cmaxd) UPDATE_HITMASK3;
					#else
						float tminx0 = _native_fma( lox4.x, idirx, origx ), tminx1 = _native_fma( lox4.y, idirx, origx );
						float tminy0 = _native_fma( loy4.x, idiry, origy ), tminy1 = _native_fma( loy4.y, idiry, origy );
						float tminz0 = _native_fma( loz4.x, idirz, origz ), tminz1 = _native_fma( loz4.y, idirz, origz );
						float tmaxx0 = _native_fma( hix4.x, idirx, origx ), tmaxx1 = _native_fma( hix4.y, idirx, origx );
						float tmaxy0 = _native_fma( hiy4.x, idiry, origy ), tmaxy1 = _native_fma( hiy4.y, idiry, origy );
						float tmaxz0 = _native_fma( hiz4.x, idirz, origz ), tmaxz1 = _native_fma( hiz4.y, idirz, origz );
						n0.x = fmax( fmax_fmax( tminx0, tminy0, tminz0 ), 0 );
						n0.y = fmin( fmin_fmin( tmaxx0, tmaxy0, tmaxz0 ), tmax );
						n1.x = fmax( fmax_fmax( tminx1, tminy1, tminz1 ), 0 );
						n1.y = fmin( fmin_fmin( tmaxx1, tmaxy1, tmaxz1 ), tmax );
						if (n0.x <= n0.y) UPDATE_HITMASK0;
						if (n1.x <= n1.y) UPDATE_HITMASK1;
						tminx0 = _native_fma( lox4.z, idirx, origx ), tminx1 = _native_fma( lox4.w, idirx, origx );
						tminy0 = _native_fma( loy4.z, idiry, origy ), tminy1 = _native_fma( loy4.w, idiry, origy );
						tminz0 = _native_fma( loz4.z, idirz, origz ), tminz1 = _native_fma( loz4.w, idirz, origz );
						tmaxx0 = _native_fma( hix4.z, idirx, origx ), tmaxx1 = _native_fma( hix4.w, idirx, origx );
						tmaxy0 = _native_fma( hiy4.z, idiry, origy ), tmaxy1 = _native_fma( hiy4.w, idiry, origy );
						tmaxz0 = _native_fma( hiz4.z, idirz, origz ), tmaxz1 = _native_fma( hiz4.w, idirz, origz );
						n0.x = fmax( fmax_fmax( tminx0, tminy0, tminz0 ), 0 );
						n0.y = fmin( fmin_fmin( tmaxx0, tmaxy0, tmaxz0 ), tmax );
						n1.x = fmax( fmax_fmax( tminx1, tminy1, tminz1 ), 0 );
						n1.y = fmin( fmin_fmin( tmaxx1, tmaxy1, tmaxz1 ), tmax );
						if (n0.x <= n0.y) UPDATE_HITMASK2;
						if (n1.x <= n1.y) UPDATE_HITMASK3;
					#endif
					}
				}
				ngroup.y = (hitmask & 0xFF000000) | (e4 >> 24), tgroup.y = hitmask & 0x00FFFFFF;
			}
		}
		else tgroup = ngroup, ngroup = (uint2)(0);
		while (tgroup.y != 0)
		{
		#ifdef CWBVH_COMPRESSED_TRIS
			// Fast intersection of triangle data for the algorithm in:
			// "Fast Ray-Triangle Intersections by Coordinate Transformation"
			// Baldwin & Weber, 2016.
			const unsigned triangleIndex = __bfind( tgroup.y ), triAddr = tgroup.x + triangleIndex * 4;
			const float4 T2 = cwbvhTris[triAddr + 2];
			const float transS = T2.x * O.x + T2.y * O.y + T2.z * O.z + T2.w;
			const float transD = T2.x * D.x + T2.y * D.y + T2.z * D.z;
			const float d = -transS / transD;
			tgroup.y -= 1 << triangleIndex;
			if (d <= 0 || d >= tmax) continue;
			const float4 T0 = cwbvhTris[triAddr + 0];
			const float4 T1 = cwbvhTris[triAddr + 1];
			const float3or4 I = O + d * D;
			const float u = T0.x * I.x + T0.y * I.y + T0.z * I.z + T0.w;
			const float v = T1.x * I.x + T1.y * I.y + T1.z * I.z + T1.w;
			const bool hit = u >= 0 && v >= 0 && u + v < 1;
			if (hit) uv = (float2)( u, v ), tmax = d, hitAddr = as_uint( cwbvhTris[triAddr + 3].w );
		#else
			// Möller-Trumbore intersection; triangles are stored as 3x16 bytes,
			// with the original primitive index in the (otherwise unused) w 
			// component of vertex 0. iquilezles.org version.
			const int triangleIndex = __bfind( tgroup.y ), triAddr = tgroup.x + triangleIndex * 3;
			const float3 e1 = cwbvhTris[triAddr].xyz;
			const float3 e2 = cwbvhTris[triAddr + 1].xyz;
			const float4 v0 = cwbvhTris[triAddr + 2];
			tgroup.y -= 1 << triangleIndex;
			const float3 r = cross( D.xyz, e1 );
			const float a = dot( e2, r );
			const float f = 1 / a;
			const float3 s = O.xyz - v0.xyz;
			const float u = f * dot( s, r );
			const float3 q = cross( s, e2 );
			const float v = f * dot( D.xyz, q );
			if (u < 0 || v < 0 || u + v > 1) continue;
			const float d = f * dot( e1, q );
			if (d <= 0.0f || d >= tmax) continue;
			uv = (float2)(u, v), tmax = d;
			hitAddr = as_uint( v0.w );
		#endif
		}
		if (ngroup.y <= 0x00FFFFFF)
		{
			if (stackPtr > 0) { STACK_POP( ngroup ); } else
			{
				hit = (float4)(tmax, uv.x, uv.y, as_float( hitAddr ));
				break;
			}
		}
	} while (true);
	return hit;
}

#ifdef SIMD_AABBTEST
bool isoccluded_cwbvh( global const float4* cwbvhNodes, global const float4* cwbvhTris, const float4 O, const float4 D, const float4 rD, const float t )
#else
bool isoccluded_cwbvh( global const float4* cwbvhNodes, global const float4* cwbvhTris, const float3 O, const float3 D, const float3 rD, const float t )
#endif
{
	// prepare traversal
	uint2 stack[CWBVH_STACK];
	uint stackPtr = 0;
	float tmax = t;
	const uint octinv4 = (7 - ((D.x < 0 ? 4 : 0) | (D.y < 0 ? 2 : 0) | (D.z < 0 ? 1 : 0))) * 0x1010101;
	const float3or4 rDs = rD * QUNPACK_SCALE; // per-ray, hoisted out of the node loop.
	uint2 ngroup = (uint2)(0, 0b10000000000000000000000000000000), tgroup = (uint2)(0);
	do
	{
		if (ngroup.y > 0x00FFFFFF)
		{
			const unsigned hits = ngroup.y, imask = ngroup.y;
			const unsigned child_bit_index = __bfind( hits );
			const unsigned child_node_base_index = ngroup.x;
			ngroup.y &= ~(1 << child_bit_index);
			if (ngroup.y > 0x00FFFFFF) { STACK_PUSH( ngroup ); }
			{
				const unsigned slot_index = (child_bit_index - 24) ^ (octinv4 & 255);
				const unsigned relative_index = __popc( imask & ~(0xFFFFFFFF << slot_index) );
				const unsigned child_node_index = (child_node_base_index + relative_index) * 5;
			#ifdef USE_VLOAD_VSTORE
				const float* p = (float*)&cwbvhNodes[child_node_index + 0];
				float4 n0 = vload4( 0, p ), n1 = vload4( 1, p ), n2 = vload4( 2, p );
				float4 n3 = vload4( 3, p ), n4 = vload4( 4, p );
			#else
				float4 n0 = cwbvhNodes[child_node_index + 0], n1 = cwbvhNodes[child_node_index + 1];
				float4 n2 = cwbvhNodes[child_node_index + 2], n3 = cwbvhNodes[child_node_index + 3];
				float4 n4 = cwbvhNodes[child_node_index + 4];
			#endif
				// n0.w packs three 127-biased quantization exponents plus imask, so each
				// byte is already an IEEE-754 exponent field: (b << 23) is 2^e directly.
				const uint e4 = as_uint( n0.w );
				ngroup.x = as_uint( n1.x ), tgroup = (uint2)(as_uint( n1.y ), 0);
				unsigned hitmask = 0;
			#ifdef SIMD_AABBTEST
				const float4 idir4 = (float4)(
					as_float( (e4 & 255) << 23 ) * rDs.x, 
					as_float( ((e4 >> 8) & 255) << 23 ) * rDs.y,
					as_float( ((e4 >> 16) & 255) << 23 ) * rDs.z, 1
				);
				const float4 orig4 = (n0 - O) * rD - QUNPACK_BIAS * idir4;
			#else
				const float idirx = as_float( (e4 & 255) << 23 ) * rDs.x;
				const float idiry = as_float( ((e4 >> 8) & 255) << 23 ) * rDs.y;
				const float idirz = as_float( ((e4 >> 16) & 255) << 23 ) * rDs.z;
				const float origx = (n0.x - O.x) * rD.x - QUNPACK_BIAS * idirx;
				const float origy = (n0.y - O.y) * rD.y - QUNPACK_BIAS * idiry;
				const float origz = (n0.z - O.z) * rD.z - QUNPACK_BIAS * idirz;
			#endif
				{	// first 4
					const unsigned meta4 = as_uint( n1.z ), is_inner4 = (meta4 & (meta4 << 1)) & 0x10101010;
					const unsigned inner_mask4 = (is_inner4 << 4) - (is_inner4 >> 4);
					const unsigned bit_index4 = (meta4 ^ (octinv4 & inner_mask4)) & 0x1F1F1F1F;
					const unsigned child_bits4 = (meta4 >> 5) & 0x07070707;
					const float4 lox4 = unpack_q8x4( as_uint( rD.x < 0 ? n3.z : n2.x ) ), hix4 = unpack_q8x4( as_uint( rD.x < 0 ? n2.x : n3.z ) );
					const float4 loy4 = unpack_q8x4( as_uint( rD.y < 0 ? n4.x : n2.z ) ), hiy4 = unpack_q8x4( as_uint( rD.y < 0 ? n2.z : n4.x ) );
					const float4 loz4 = unpack_q8x4( as_uint( rD.z < 0 ? n4.z : n3.x ) ), hiz4 = unpack_q8x4( as_uint( rD.z < 0 ? n3.x : n4.z ) );
					{
					#ifdef SIMD_AABBTEST
						const float4 tminx4 = lox4 * idir4.xxxx + orig4.xxxx, tmaxx4 = hix4 * idir4.xxxx + orig4.xxxx;
						const float4 tminy4 = loy4 * idir4.yyyy + orig4.yyyy, tmaxy4 = hiy4 * idir4.yyyy + orig4.yyyy;
						const float4 tminz4 = loz4 * idir4.zzzz + orig4.zzzz, tmaxz4 = hiz4 * idir4.zzzz + orig4.zzzz;
						const float cmina = fmax( fmax( fmax( tminx4.x, tminy4.x ), tminz4.x ), 0 );
						const float cmaxa = fmin( fmin( fmin( tmaxx4.x, tmaxy4.x ), tmaxz4.x ), tmax );
						const float cminb = fmax( fmax( fmax( tminx4.y, tminy4.y ), tminz4.y ), 0 );
						const float cmaxb = fmin( fmin( fmin( tmaxx4.y, tmaxy4.y ), tmaxz4.y ), tmax );
						if (cmina <= cmaxa) UPDATE_HITMASK;
						if (cminb <= cmaxb) UPDATE_HITMASK1;
						const float cminc = fmax( fmax( fmax( tminx4.z, tminy4.z ), tminz4.z ), 0 );
						const float cmaxc = fmin( fmin( fmin( tmaxx4.z, tmaxy4.z ), tmaxz4.z ), tmax );
						const float cmind = fmax( fmax( fmax( tminx4.w, tminy4.w ), tminz4.w ), 0 );
						const float cmaxd = fmin( fmin( fmin( tmaxx4.w, tmaxy4.w ), tmaxz4.w ), tmax );
						if (cminc <= cmaxc) UPDATE_HITMASK2;
						if (cmind <= cmaxd) UPDATE_HITMASK3;
					#else
						float tminx0 = _native_fma( lox4.x, idirx, origx ), tminx1 = _native_fma( lox4.y, idirx, origx );
						float tminy0 = _native_fma( loy4.x, idiry, origy ), tminy1 = _native_fma( loy4.y, idiry, origy );
						float tminz0 = _native_fma( loz4.x, idirz, origz ), tminz1 = _native_fma( loz4.y, idirz, origz );
						float tmaxx0 = _native_fma( hix4.x, idirx, origx ), tmaxx1 = _native_fma( hix4.y, idirx, origx );
						float tmaxy0 = _native_fma( hiy4.x, idiry, origy ), tmaxy1 = _native_fma( hiy4.y, idiry, origy );
						float tmaxz0 = _native_fma( hiz4.x, idirz, origz ), tmaxz1 = _native_fma( hiz4.y, idirz, origz );
						n0.x = fmax( fmax_fmax( tminx0, tminy0, tminz0 ), 0 );
						n0.y = fmin( fmin_fmin( tmaxx0, tmaxy0, tmaxz0 ), tmax );
						n1.x = fmax( fmax_fmax( tminx1, tminy1, tminz1 ), 0 );
						n1.y = fmin( fmin_fmin( tmaxx1, tmaxy1, tmaxz1 ), tmax );
						if (n0.x <= n0.y) UPDATE_HITMASK;
						if (n1.x <= n1.y) UPDATE_HITMASK1;
						tminx0 = _native_fma( lox4.z, idirx, origx ), tminx1 = _native_fma( lox4.w, idirx, origx );
						tminy0 = _native_fma( loy4.z, idiry, origy ), tminy1 = _native_fma( loy4.w, idiry, origy );
						tminz0 = _native_fma( loz4.z, idirz, origz ), tminz1 = _native_fma( loz4.w, idirz, origz );
						tmaxx0 = _native_fma( hix4.z, idirx, origx ), tmaxx1 = _native_fma( hix4.w, idirx, origx );
						tmaxy0 = _native_fma( hiy4.z, idiry, origy ), tmaxy1 = _native_fma( hiy4.w, idiry, origy );
						tmaxz0 = _native_fma( hiz4.z, idirz, origz ), tmaxz1 = _native_fma( hiz4.w, idirz, origz );
						n0.x = fmax( fmax_fmax( tminx0, tminy0, tminz0 ), 0 );
						n0.y = fmin( fmin_fmin( tmaxx0, tmaxy0, tmaxz0 ), tmax );
						n1.x = fmax( fmax_fmax( tminx1, tminy1, tminz1 ), 0 );
						n1.y = fmin( fmin_fmin( tmaxx1, tmaxy1, tmaxz1 ), tmax );
						if (n0.x <= n0.y) UPDATE_HITMASK2;
						if (n1.x <= n1.y) UPDATE_HITMASK3;
					#endif
					}
				}
				{	// second 4
					const unsigned meta4 = as_uint( n1.w ), is_inner4 = (meta4 & (meta4 << 1)) & 0x10101010;
					const unsigned inner_mask4 = (is_inner4 << 4) - (is_inner4 >> 4);
					const unsigned bit_index4 = (meta4 ^ (octinv4 & inner_mask4)) & 0x1F1F1F1F;
					const unsigned child_bits4 = (meta4 >> 5) & 0x07070707;
					const float4 lox4 = unpack_q8x4( as_uint( rD.x < 0 ? n3.w : n2.y ) ), hix4 = unpack_q8x4( as_uint( rD.x < 0 ? n2.y : n3.w ) );
					const float4 loy4 = unpack_q8x4( as_uint( rD.y < 0 ? n4.y : n2.w ) ), hiy4 = unpack_q8x4( as_uint( rD.y < 0 ? n2.w : n4.y ) );
					const float4 loz4 = unpack_q8x4( as_uint( rD.z < 0 ? n4.w : n3.y ) ), hiz4 = unpack_q8x4( as_uint( rD.z < 0 ? n3.y : n4.w ) );
					{
					#ifdef SIMD_AABBTEST
						const float4 tminx4 = lox4 * idir4.xxxx + orig4.xxxx, tmaxx4 = hix4 * idir4.xxxx + orig4.xxxx;
						const float4 tminy4 = loy4 * idir4.yyyy + orig4.yyyy, tmaxy4 = hiy4 * idir4.yyyy + orig4.yyyy;
						const float4 tminz4 = loz4 * idir4.zzzz + orig4.zzzz, tmaxz4 = hiz4 * idir4.zzzz + orig4.zzzz;
						const float cmina = fmax( fmax( fmax( tminx4.x, tminy4.x ), tminz4.x ), 0 );
						const float cmaxa = fmin( fmin( fmin( tmaxx4.x, tmaxy4.x ), tmaxz4.x ), tmax );
						const float cminb = fmax( fmax( fmax( tminx4.y, tminy4.y ), tminz4.y ), 0 );
						const float cmaxb = fmin( fmin( fmin( tmaxx4.y, tmaxy4.y ), tmaxz4.y ), tmax );
						if (cmina <= cmaxa) UPDATE_HITMASK0;
						if (cminb <= cmaxb) UPDATE_HITMASK1;
						const float cminc = fmax( fmax( fmax( tminx4.z, tminy4.z ), tminz4.z ), 0 );
						const float cmaxc = fmin( fmin( fmin( tmaxx4.z, tmaxy4.z ), tmaxz4.z ), tmax );
						const float cmind = fmax( fmax( fmax( tminx4.w, tminy4.w ), tminz4.w ), 0 );
						const float cmaxd = fmin( fmin( fmin( tmaxx4.w, tmaxy4.w ), tmaxz4.w ), tmax );
						if (cminc <= cmaxc) UPDATE_HITMASK2;
						if (cmind <= cmaxd) UPDATE_HITMASK3;
					#else
						float tminx0 = _native_fma( lox4.x, idirx, origx ), tminx1 = _native_fma( lox4.y, idirx, origx );
						float tminy0 = _native_fma( loy4.x, idiry, origy ), tminy1 = _native_fma( loy4.y, idiry, origy );
						float tminz0 = _native_fma( loz4.x, idirz, origz ), tminz1 = _native_fma( loz4.y, idirz, origz );
						float tmaxx0 = _native_fma( hix4.x, idirx, origx ), tmaxx1 = _native_fma( hix4.y, idirx, origx );
						float tmaxy0 = _native_fma( hiy4.x, idiry, origy ), tmaxy1 = _native_fma( hiy4.y, idiry, origy );
						float tmaxz0 = _native_fma( hiz4.x, idirz, origz ), tmaxz1 = _native_fma( hiz4.y, idirz, origz );
						n0.x = fmax( fmax_fmax( tminx0, tminy0, tminz0 ), 0 );
						n0.y = fmin( fmin_fmin( tmaxx0, tmaxy0, tmaxz0 ), tmax );
						n1.x = fmax( fmax_fmax( tminx1, tminy1, tminz1 ), 0 );
						n1.y = fmin( fmin_fmin( tmaxx1, tmaxy1, tmaxz1 ), tmax );
						if (n0.x <= n0.y) UPDATE_HITMASK0;
						if (n1.x <= n1.y) UPDATE_HITMASK1;
						tminx0 = _native_fma( lox4.z, idirx, origx ), tminx1 = _native_fma( lox4.w, idirx, origx );
						tminy0 = _native_fma( loy4.z, idiry, origy ), tminy1 = _native_fma( loy4.w, idiry, origy );
						tminz0 = _native_fma( loz4.z, idirz, origz ), tminz1 = _native_fma( loz4.w, idirz, origz );
						tmaxx0 = _native_fma( hix4.z, idirx, origx ), tmaxx1 = _native_fma( hix4.w, idirx, origx );
						tmaxy0 = _native_fma( hiy4.z, idiry, origy ), tmaxy1 = _native_fma( hiy4.w, idiry, origy );
						tmaxz0 = _native_fma( hiz4.z, idirz, origz ), tmaxz1 = _native_fma( hiz4.w, idirz, origz );
						n0.x = fmax( fmax_fmax( tminx0, tminy0, tminz0 ), 0 );
						n0.y = fmin( fmin_fmin( tmaxx0, tmaxy0, tmaxz0 ), tmax );
						n1.x = fmax( fmax_fmax( tminx1, tminy1, tminz1 ), 0 );
						n1.y = fmin( fmin_fmin( tmaxx1, tmaxy1, tmaxz1 ), tmax );
						if (n0.x <= n0.y) UPDATE_HITMASK2;
						if (n1.x <= n1.y) UPDATE_HITMASK3;
					#endif
					}
				}
				ngroup.y = (hitmask & 0xFF000000) | (e4 >> 24), tgroup.y = hitmask & 0x00FFFFFF;
			}
		}
		else tgroup = ngroup, ngroup = (uint2)(0);
		while (tgroup.y != 0)
		{
		#ifdef CWBVH_COMPRESSED_TRIS
			// Fast intersection of triangle data for the algorithm in:
			// "Fast Ray-Triangle Intersections by Coordinate Transformation"
			// Baldwin & Weber, 2016.
			const unsigned triangleIndex = __bfind( tgroup.y ), triAddr = tgroup.x + triangleIndex * 4;
			const float4 T2 = cwbvhTris[triAddr + 2];
			const float transS = T2.x * O.x + T2.y * O.y + T2.z * O.z + T2.w;
			const float transD = T2.x * D.x + T2.y * D.y + T2.z * D.z;
			const float d = -transS / transD;
			tgroup.y -= 1 << triangleIndex;
			if (d <= 0 || d >= tmax) continue;
			const float4 T0 = cwbvhTris[triAddr + 0];
			const float4 T1 = cwbvhTris[triAddr + 1];
			const float3or4 I = O + d * D;
			const float u = T0.x * I.x + T0.y * I.y + T0.z * I.z + T0.w;
			const float v = T1.x * I.x + T1.y * I.y + T1.z * I.z + T1.w;
			const bool hit = u >= 0 && v >= 0 && u + v < 1;
			if (hit) return true;
		#else
			// Möller-Trumbore intersection; triangles are stored as 3x16 bytes,
			// with the original primitive index in the (otherwise unused) w 
			// component of vertex 0.
			const int triangleIndex = __bfind( tgroup.y ), triAddr = tgroup.x + triangleIndex * 3;
			const float3 e1 = cwbvhTris[triAddr].xyz;
			const float3 e2 = cwbvhTris[triAddr + 1].xyz;
			const float3 v0 = cwbvhTris[triAddr + 2].xyz;
			tgroup.y -= 1 << triangleIndex;
			const float3 r = cross( D.xyz, e1 );
			const float a = dot( e2, r );
			if (fabs( a ) < 0.0000001f) continue;
			const float f = 1 / a;
			const float3 s = O.xyz - v0;
			const float u = f * dot( s, r );
			const float3 q = cross( s, e2 );
			const float v = f * dot( D.xyz, q );
			if (u < 0 || v < 0 || u + v > 1) continue;
			const float d = f * dot( e1, q );
			if (d > 0.0f && d < tmax) return true;
		#endif
		}
		if (ngroup.y <= 0x00FFFFFF) { if (stackPtr == 0) break; STACK_POP( ngroup ); }
	} while (true);
	return false; // no occlusion found.
}

void kernel batch_cwbvh( global const float4* cwbvhNodes, global const float4* cwbvhTris, global struct Ray* rayData )
{
	// initialize ray
	const unsigned threadId = get_global_id( 0 );
#ifdef SIMD_AABBTEST
	float4 O4 = rayData[threadId].O; O4.w = 1;
	float4 D4 = rayData[threadId].D; D4.w = 0;
	float4 rD4 = rayData[threadId].rD; rD4.w = 1;
	float4 hit = traverse_cwbvh( cwbvhNodes, cwbvhTris, O4, D4, rD4, 1e30f );
#else
	const float4 O4 = rayData[threadId].O;
	const float4 D4 = rayData[threadId].D;
	const float4 rD4 = rayData[threadId].rD;
	float4 hit = traverse_cwbvh( cwbvhNodes, cwbvhTris, O4.xyz, D4.xyz, rD4.xyz, 1e30f );
#endif
	rayData[threadId].hit = hit;
}

void kernel batch_cwbvh_any( global const float4* cwbvhNodes, global const float4* cwbvhTris, global struct Ray* rayData )
{
	// initialize ray
	const unsigned threadId = get_global_id( 0 );
#ifdef SIMD_AABBTEST
	float4 O4 = rayData[threadId].O; O4.w = 1;
	float4 D4 = rayData[threadId].D; D4.w = 0;
	float4 rD4 = rayData[threadId].rD; rD4.w = 1;
#else
	const float4 O4 = rayData[threadId].O;
	const float4 D4 = rayData[threadId].D;
	const float4 rD4 = rayData[threadId].rD;
#endif
	float tmax = 1e30f;
	float4 hit = 0;
	if (isoccluded_cwbvh( cwbvhNodes, cwbvhTris, O4.xyz, D4.xyz, rD4.xyz, tmax )) hit.w = 1;
	rayData[threadId].hit = hit;
}