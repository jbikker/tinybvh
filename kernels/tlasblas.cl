// ============================================================================
//
//        T L A S _ B A T C H . C L
//
// Self-contained batched TLAS/BLAS ray queries for TinyBVH. This file has no
// includes; drop it next to your application and build it as-is.
//
// Kernels:
//
//   batch_nearest_tlas - 'find nearest' query for a batch of rays.
//   batch_any_tlas     - 'any hit' query for a batch of rays (shadow rays).
//
// Ray layout (48 bytes, see struct Ray below):
//
//   O.xyz  ray origin,         O.w  ray mask, as uint bits.
//   D.xyz  ray direction,      D.w  unused.
//   hit.x  tmax on input; distance (nearest) or zero (any hit) on output.
//   hit.y  barycentric u,      hit.z  barycentric v.
//   hit.w  on output: primitive index with the instance index in the high
//          bits, as uint bits; for 'any hit', 1 when occluded, 0 otherwise.
//
// The reciprocal of the ray direction is computed in the kernel, so the host
// does not have to supply it.
//
// ============================================================================

// ----------------------------------------------------------------------------
// Configuration. Override any of these from the host with -D<NAME>=<VALUE>.
// ----------------------------------------------------------------------------

// Number of bits reserved for the instance index in hit.w. Must match the
// INST_IDX_BITS the host was built with: INST_IDX_SHFT == 32 - INST_IDX_BITS.
#ifndef INST_IDX_SHFT
#define INST_IDX_SHFT 24
#endif
#define PRIM_IDX_MASK ((1u << INST_IDX_SHFT) - 1u)

// Supported BLAS layouts. Each one costs code size and, when more than one is
// enabled, a branch per TLAS leaf; switch off what the scene does not use.
#ifndef BLAS_AILALAINE
#define BLAS_AILALAINE 1 // tinybvh::BVH_GPU
#endif
#ifndef BLAS_CWBVH
#define BLAS_CWBVH 1 // tinybvh::BVH8_CWBVH
#endif
#ifndef BLAS_AMD_HW_BVH4
#define BLAS_AMD_HW_BVH4 0 // tinybvh::BVH4_AMD_HW; needs RDNA3/RDNA4.
#endif
#ifndef BLAS_AMD_HW_CWBVH8
#define BLAS_AMD_HW_CWBVH8 0 // tinybvh::CWBVH8_AMD_HW; needs RDNA4.
#endif

// Software traversal stack size, used by the TLAS and by the BVH_GPU BLAS.
#ifndef STACK_SIZE
#define STACK_SIZE 32
#endif

// Set when a BLAS layout traverses on the hardware BVH unit, which needs an
// LDS stack and a fixed work group size of HW_BVH_WORKGROUP_SIZE.
#if BLAS_AMD_HW_BVH4 || BLAS_AMD_HW_CWBVH8
#define BLAS_HW_STACK 1
#else
#define BLAS_HW_STACK 0
#endif

// BLAS layout identifiers, stored in BLASDesc::blasType. Keep these in sync
// with the host; values 4 and up are free for new layouts.
#define BLASTYPE_AILALAINE		0
#define BLASTYPE_CWBVH			1
#define BLASTYPE_AMD_HW_BVH4	2
#define BLASTYPE_AMD_HW_CWBVH8	3

// Low-level optimizations for specific platforms.
#ifdef ISINTEL // Iris Xe, Arc, ..
// #define USE_VLOAD_VSTORE
#define SIMD_AABBTEST
#elif defined ISNVIDIA // 2080, 3080, 4080, ..
#define USE_VLOAD_VSTORE
// #define SIMD_AABBTEST
#elif defined ISAMD
#define USE_VLOAD_VSTORE
// #define SIMD_AABBTEST
#else // unknown GPU
// #define USE_VLOAD_VSTORE
#define SIMD_AABBTEST
#endif

// ----------------------------------------------------------------------------
// Data layout. These must match the host structs byte for byte; the host
// asserts this.
// ----------------------------------------------------------------------------

struct Ray
{
	// 48 bytes, laid out as three 16-byte values to encourage the compiler to
	// fetch 16 bytes at a time.
	float4 O; // w: ray mask, as uint bits.
	float4 D; // w: unused.
	float4 hit; // x: tmax in, t out. y, z: barycentrics. w: prim + inst.
};

struct BVHNode
{
	// Aila & Laine node layout, used for the TLAS and for BVH_GPU BLASses.
	float4 lmin; // uint left in w
	float4 lmax; // uint right in w
	float4 rmin; // uint triCount in w
	float4 rmax; // uint firstTri in w
};

struct Instance
{
	// Matches tinybvh::BLASInstance; 192 bytes.
	float transform[16];
	float invTransform[16];
	float4 aabbMin; // w: uint blasIdx
	float4 aabbMax; // w: uint mask
	uint dummy[8]; // padding to 192 bytes
};

struct BLASDesc
{
	// Describes where one BLAS lives inside the pools for its layout. Offsets
	// are in 16-byte blocks from the start of the pool, whatever the layout,
	// so a new layout needs no new fields here - only a new BLASTYPE and a
	// case in IntersectBLAS / IsOccludedBLAS below.
	uint blasType; // one of BLASTYPE_*
	uint nodeOffset; // first node block in the node pool for this layout
	uint primOffset; // first primitive block in the primitive pool, if any
	uint dummy[5]; // padding to 32 bytes; room for future layouts
};

// ----------------------------------------------------------------------------
// Helpers.
// ----------------------------------------------------------------------------

// TransformPoint - apply a 4x4 matrix transform to a 3D position. The matrix
// is explicitly in global memory, so this compiles without the generic address
// space, i.e. on OpenCL 1.2.
float3 TransformPoint( const float3 v, const global float* T )
{
	const float3 res = (float3)(
		T[0] * v.x + T[1] * v.y + T[2] * v.z + T[3],
		T[4] * v.x + T[5] * v.y + T[6] * v.z + T[7],
		T[8] * v.x + T[9] * v.y + T[10] * v.z + T[11]
	);
	const float w = T[12] * v.x + T[13] * v.y + T[14] * v.z + T[15];
	if (w == 1.0f) return res; else return res * (1.0f / w);
}

// TransformVector - apply a 4x4 matrix transform to a 3D vector.
float3 TransformVector( const float3 v, const global float* T )
{
	return (float3)( T[0] * v.x + T[1] * v.y + T[2] * v.z, T[4] * v.x +
		T[5] * v.y + T[6] * v.z, T[8] * v.x + T[9] * v.y + T[10] * v.z );
}
#if BLAS_AILALAINE
// ============================================================================
//
//        T R A V E R S E _ A I L A L A I N E
// 
// ============================================================================

float4 traverse( const global struct BVHNode* bvhNode, 
	const global float4* orderedVerts,
	const float3 O, const float3 D, const float3 rD, const float tmax, uint* stepCount )
{
	// "find nearest" ray query using the BVH_GPU format.
	const float3 rO = O * -rD; // precalculate for efficient slab test
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
		const bool hitA = tmina <= tmaxa, hitB = tminb <= tmaxb;
		if (hitA && hitB)
		{
			uint near = left, far = right;
			if (tminb < tmina) near = right, far = left;
			stack[stackPtr++] = far, nodeIdx = near;
		}
		else if (hitA) nodeIdx = left;
		else if (hitB) nodeIdx = right;
		else { if (stackPtr == 0) break; nodeIdx = stack[--stackPtr]; }
	}
	if (stepCount) *stepCount += steps;
	return hit;
}

bool isoccluded( 
	const global struct BVHNode* bvhNode, 
	const global float4* orderedVerts,
	const float3 O, const float3 D, const float3 rD, const float tmax )
{
	// "any hit" ray query using the BVH_GPU format.
	const float3 rO = O * -rD;
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
				const float f = native_recip( dot( edge1.xyz, h ) );
				const float3 s = O - vert0.xyz;
				const float u = f * dot( s, h );
				const float3 q = cross( s, edge1.xyz );
				const float v = f * dot( D, q );
				if (u < 0 || v < 0 || u + v > 1) continue;
				const float d = f * dot( edge2.xyz, q );
				if (d <= 0.0f || d >= tmax) continue;
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
		const bool hitA = tmina <= tmaxa, hitB = tminb <= tmaxb;
		if (hitA && hitB)
		{
			uint near = left, far = right;
			if (tminb < tmina) near = right, far = left;
			stack[stackPtr++] = far, nodeIdx = near;
		}
		else if (hitA) nodeIdx = left;
		else if (hitB) nodeIdx = right;
		else { if (stackPtr == 0) break; nodeIdx = stack[--stackPtr]; }
	}
	return false;
}
#endif // BLAS_AILALAINE
#if BLAS_CWBVH
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

#ifdef CWBVH_COMPRESSED_TRIS
#define TRI_STRIDE 4 // must match the tiny_bvh.h build define
#else
#define TRI_STRIDE 3
#endif

#define STACK_POP(X) { X = stack[--stackPtr]; }
#define STACK_PUSH(X) { stack[stackPtr++] = X; }

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

#define QUNPACK_BITTRICK 1
#if QUNPACK_BITTRICK
#define QUNPACK_SCALE	256.0f
#define QUNPACK_BIAS	1.0f
#else
#define QUNPACK_SCALE	1.0f
#define QUNPACK_BIAS	0.0f
#endif

inline float4 unpack_q8x4( const uint w )
{
#if QUNPACK_BITTRICK
	const uint m = 0x007F8000u, b = 0x3F800000u; // mantissa slot 22..15, exponent 0
#ifndef ISNVIDIA
	return as_float4( ((uint4)(w << 15, w << 7, w >> 1, w >> 9) & m) | b );
#else
	return (float4)(
		as_float( ((w << 15) & m) | b ), as_float( ((w <<  7) & m) | b ),
		as_float( ((w >>  1) & m) | b ), as_float( ((w >>  9) & m) | b ) );
#endif
#else
	return convert_float4( (uint4)(w, w >> 8, w >> 16, w >> 24) & 255u );
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
	uint stack[CWBVH_STACK];
	uint hitAddr = 0, stackPtr = 0;
	float2 uv = (float2)(0);
	float tmax = t;
	const uint octinv4 = (7 - ((D.x < 0 ? 4 : 0) | (D.y < 0 ? 2 : 0) | (D.z < 0 ? 1 : 0))) * 0x1010101;
	const float3or4 rDs = rD * QUNPACK_SCALE; // per-ray, hoisted out of the node loop.
	uint2 ngroup = (uint2)(0, 0x80000000), tgroup = (uint2)(0);
	uint parentNode = 0;
	do
	{
		const unsigned imask = ngroup.y;
		const unsigned child_bit_index = __bfind( ngroup.y );
		const unsigned child_node_base_index = ngroup.x;
		ngroup.y &= ~(1 << child_bit_index);
		if (ngroup.y > 0x00FFFFFF && stackPtr < CWBVH_STACK)
		{
			STACK_PUSH( (parentNode << 8) | (ngroup.y >> 24) );
		}
		{
			const unsigned slot_index = (child_bit_index - 24) ^ (octinv4 & 255);
			const unsigned relative_index = __popc( imask & ~(0xFFFFFFFF << slot_index) );
			const unsigned nodeIndex = child_node_base_index + relative_index;
			const unsigned child_node_index = nodeIndex * 5;
		#ifdef USE_VLOAD_VSTORE
			const global float* p = (const global float*)&cwbvhNodes[child_node_index + 0];
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
			parentNode = nodeIndex;
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
				// expand the 0x10 marker in each byte to 0xFF: (x << 4) - (x >> 4) is
				// 0x100 - 0x01 per marked byte, with no carry into the neighbours.
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
		// process the triangles referenced by the current triangle group
		while (tgroup.y != 0)
		{
			const unsigned triangleIndex = __bfind( tgroup.y );
			const unsigned triAddr = tgroup.x + triangleIndex * TRI_STRIDE;
			tgroup.y -= 1 << triangleIndex;
		#ifdef CWBVH_COMPRESSED_TRIS
			// Fast intersection of triangle data for the algorithm in:
			// "Fast Ray-Triangle Intersections by Coordinate Transformation"
			// Baldwin & Weber, 2016.
			const float4 T2 = cwbvhTris[triAddr + 2];
			const float transS = T2.x * O.x + T2.y * O.y + T2.z * O.z + T2.w;
			const float transD = T2.x * D.x + T2.y * D.y + T2.z * D.z;
			const float s = -transS;
			if (!(s * transD > 0)) continue; // d <= 0, transD == 0, or NaN
			if (!(fabs( s ) < tmax * fabs( transD ))) continue; // d >= tmax, or NaN
			const float d = s / transD; // only for survivors
			const float4 T0 = cwbvhTris[triAddr + 0];
			const float4 T1 = cwbvhTris[triAddr + 1];
			const float3or4 I = O + d * D;
			const float u = T0.x * I.x + T0.y * I.y + T0.z * I.z + T0.w;
			const float v = T1.x * I.x + T1.y * I.y + T1.z * I.z + T1.w;
			if (!(u >= 0 && v >= 0 && u + v < 1)) continue;
			uv = (float2)(u, v), tmax = d;
			hitAddr = as_uint( cwbvhTris[triAddr + 3].w );
		#else
			// Moeller-Trumbore, iquilezles.org version. Tris are stored as 3x16 bytes:
			// two edges plus vertex 0, whose w holds the original primitive index.
			const float3 e1 = cwbvhTris[triAddr].xyz;
			const float3 e2 = cwbvhTris[triAddr + 1].xyz;
			const float4 v0 = cwbvhTris[triAddr + 2];
			const float3 r = cross( D.xyz, e1 );
			const float a = dot( e2, r );
			const float f = 1 / a;
			const float3 s = O.xyz - v0.xyz;
			const float u = f * dot( s, r );
			const float3 q = cross( s, e2 );
			const float v = f * dot( D.xyz, q );
			if (!(u >= 0 && v >= 0 && u + v <= 1)) continue;
			const float d = f * dot( e1, q );
			if (!(d > 0 && d < tmax)) continue;
			uv = (float2)(u, v), tmax = d;
			hitAddr = as_uint( v0.w );
		#endif
		}
		if (ngroup.y <= 0x00FFFFFF)
		{
			if (stackPtr == 0) break;
			// unpack: the node index gives back both missing fields
			uint entry; STACK_POP( entry );
			parentNode = entry >> 8;
			const uint parentAddr = parentNode * 5;
			ngroup.x = as_uint( cwbvhNodes[parentAddr + 1].x );
			ngroup.y = ((entry & 255) << 24) | (as_uint( cwbvhNodes[parentAddr].w ) >> 24);
		}
	} while (true);
	return (float4)(tmax, uv.x, uv.y, as_float( hitAddr ));
}

#ifdef SIMD_AABBTEST
bool isoccluded_cwbvh( global const float4* cwbvhNodes, global const float4* cwbvhTris, const float4 O, const float4 D, const float4 rD, const float t )
#else
bool isoccluded_cwbvh( global const float4* cwbvhNodes, global const float4* cwbvhTris, const float3 O, const float3 D, const float3 rD, const float t )
#endif
{
	// prepare traversal
	uint stack[CWBVH_STACK];
	uint stackPtr = 0;
	const float tmax = t;
	const uint octinv4 = (7 - ((D.x < 0 ? 4 : 0) | (D.y < 0 ? 2 : 0) | (D.z < 0 ? 1 : 0))) * 0x1010101;
	const float3or4 rDs = rD * QUNPACK_SCALE; // per-ray, hoisted out of the node loop.
	uint2 ngroup = (uint2)(0, 0x80000000), tgroup = (uint2)(0);
	uint parentNode = 0;
	do
	{
		const unsigned imask = ngroup.y;
		const unsigned child_bit_index = __bfind( ngroup.y );
		const unsigned child_node_base_index = ngroup.x;
		ngroup.y &= ~(1 << child_bit_index);
		if (ngroup.y > 0x00FFFFFF && stackPtr < CWBVH_STACK)
		{
			STACK_PUSH( (parentNode << 8) | (ngroup.y >> 24) );
		}
		{
			const unsigned slot_index = (child_bit_index - 24) ^ (octinv4 & 255);
			const unsigned relative_index = __popc( imask & ~(0xFFFFFFFF << slot_index) );
			const unsigned nodeIndex = child_node_base_index + relative_index;
			const unsigned child_node_index = nodeIndex * 5;
		#ifdef USE_VLOAD_VSTORE
			const global float* p = (const global float*)&cwbvhNodes[child_node_index + 0];
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
		parentNode = nodeIndex;
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
		while (tgroup.y != 0)
		{
			const unsigned triangleIndex = __bfind( tgroup.y );
			const unsigned triAddr = tgroup.x + triangleIndex * TRI_STRIDE;
			tgroup.y -= 1 << triangleIndex;
		#ifdef CWBVH_COMPRESSED_TRIS
			// Fast intersection of triangle data for the algorithm in:
			// "Fast Ray-Triangle Intersections by Coordinate Transformation"
			// Baldwin & Weber, 2016.
			const float4 T2 = cwbvhTris[triAddr + 2];
			const float transS = T2.x * O.x + T2.y * O.y + T2.z * O.z + T2.w;
			const float transD = T2.x * D.x + T2.y * D.y + T2.z * D.z;
			const float s = -transS;
			if (!(s * transD > 0)) continue; // d <= 0, transD == 0, or NaN
			if (!(fabs( s ) < tmax * fabs( transD ))) continue; // d >= tmax, or NaN
			const float d = s / transD; // only for survivors
			const float4 T0 = cwbvhTris[triAddr + 0];
			const float4 T1 = cwbvhTris[triAddr + 1];
			const float3or4 I = O + d * D;
			const float u = T0.x * I.x + T0.y * I.y + T0.z * I.z + T0.w;
			const float v = T1.x * I.x + T1.y * I.y + T1.z * I.z + T1.w;
			if (u >= 0 && v >= 0 && u + v < 1) return true;
		#else
			// Moeller-Trumbore, iquilezles.org version. Tris are stored as 3x16 bytes:
			// two edges plus vertex 0, whose w holds the original primitive index.
			const float3 e1 = cwbvhTris[triAddr].xyz;
			const float3 e2 = cwbvhTris[triAddr + 1].xyz;
			const float3 v0 = cwbvhTris[triAddr + 2].xyz;
			const float3 r = cross( D.xyz, e1 );
			const float a = dot( e2, r );
			const float f = 1 / a;
			const float3 s = O.xyz - v0;
			const float u = f * dot( s, r );
			const float3 q = cross( s, e2 );
			const float v = f * dot( D.xyz, q );
			if (!(u >= 0 && v >= 0 && u + v <= 1)) continue;
			const float d = f * dot( e1, q );
			if (d > 0 && d < tmax) return true;
		#endif
		}
		if (ngroup.y <= 0x00FFFFFF)
		{
			if (stackPtr == 0) break;
			// unpack: the node index gives back both missing fields
			uint entry; STACK_POP( entry );
			parentNode = entry >> 8;
			const uint parentAddr = parentNode * 5;
			ngroup.x = as_uint( cwbvhNodes[parentAddr + 1].x );
			ngroup.y = ((entry & 255) << 24) | (as_uint( cwbvhNodes[parentAddr].w ) >> 24);
		}
	} while (true);
	return false; // no occlusion found.
}
#endif // BLAS_CWBVH
#if BLAS_HW_STACK
// Shared AMD hardware-traversal definitions.

inline uint id_to_offset( uint id ) { return ( id & ( ~7u ) ) << 3; }
inline uint id_to_type( uint id ) { return id & 7u; }

#define TRIANGLE 0u
#define INVALID_NODE 0xffffffffu

#define BVH_STACK_TERMINAL_NODE 0xfffffffeu
#ifndef HW_BVH_STACK_ENTRIES
#define HW_BVH_STACK_ENTRIES 16
#endif
#if HW_BVH_STACK_ENTRIES != 8 && HW_BVH_STACK_ENTRIES != 16 &&                 \
    HW_BVH_STACK_ENTRIES != 32 && HW_BVH_STACK_ENTRIES != 64
#error "AMD hardware BVH stacks require 8, 16, 32 or 64 entries per ray."
#endif
#ifdef ISRDNA4
// GFX12 stack encoding.
#define BVH_STACK_CONTROLS HW_BVH_STACK_ENTRIES
#define HW_BVH_STACK_BASE_SHIFT 15
#else
#if HW_BVH_STACK_ENTRIES == 8
#define BVH_STACK_CONTROLS 0x0000
#elif HW_BVH_STACK_ENTRIES == 16
#define BVH_STACK_CONTROLS 0x1000
#elif HW_BVH_STACK_ENTRIES == 32
#define BVH_STACK_CONTROLS 0x2000
#else
#define BVH_STACK_CONTROLS 0x3000
#endif
#define HW_BVH_STACK_BASE_SHIFT 18
#endif
// Hardware-stack kernels require 64 work-items and reserve one stack per ray.
#define HW_BVH_WORKGROUP_SIZE 64
#endif // BLAS_HW_STACK
#if BLAS_AMD_HW_BVH4
// ============================================================================
//
//        T R A V E R S E _ G P U 4 W A Y
//
// ============================================================================

// Node pointer encoding follows Mesa RADV (src/amd/vulkan/nir/radv_nir_rt_common.c).

// BVH descriptor: zero base, distance sorting, 42-bit size, pointer flags enabled.
// Type 8; triangle results are (t numerator, denominator, u numerator, v numerator).
// Reference: AMD RDNA3 ISA, section 10.9, BVH Resource Descriptor.
#define BVH_DESCRIPTOR                        \
	( (uint4)( 0u, ( 1u << 31 ), 0xffffffffu, \
			   ( 1u << 31 ) | ( 1u << 24 ) | ( 1u << 23 ) | 0x3ffu ) )

#define PAIR 1u // a triangle node with two triangles: intersected as type 1, then as type 0
#define BOX16 4u

// TinyBVH parent ID of a node, for recovery from a hardware stack overflow
inline uint parent_id( const global uint* words, const uint nodeID, const uint box16Start, const uint parentTable )
{
	const uint type = id_to_type( nodeID ), offset = id_to_offset( nodeID );
	return words[type == BOX16 ? parentTable + ( offset >> 6 ) - box16Start : ( offset >> 2 ) + ( type <= PAIR ? 14u : 31u )];
}

float4 traverse_gpu4wayHW( const global float4* alt4Node, const float4 O, const float4 D, const float4 rD, const float tmax, __local uint* hwStack )
{
	const ulong encoded_base = ( (ulong)alt4Node >> 3 ) & ( ( ( 1UL << 42 ) - 1UL ) << 3 );
	uint prevNode = INVALID_NODE;
	// LDS stack entries have a fixed stride of 32 words, even with wave64.
	uint lane = get_local_id( 1 ) * get_local_size( 0 ) + get_local_id( 0 );
	uint stackAddr = ( ( (uint)(size_t)hwStack >> 2 ) + ( lane & 31u ) +
					  ( lane >> 5 ) * ( 32u * HW_BVH_STACK_ENTRIES ) ) << HW_BVH_STACK_BASE_SHIFT;
	float4 hit = (float4)( tmax, 0.0f, 0.0f, as_float( INVALID_NODE ) );

	const global uint* words = (const global uint*)alt4Node;
	const uint box16Start = words[29], parentTable = words[30]; // for parent_id
	uint currentNodeID = 5; // start at root BOX32 node at byte offset zero.
	uint pairID = INVALID_NODE; // the pair node whose second test (as type 0) is in progress
	while (1)
	{
		uint2 stackResults;
		ulong currentPointer = encoded_base + (ulong)currentNodeID;
		uint4 dst4 = __builtin_amdgcn_image_bvh_intersect_ray_l(
			currentPointer, hit.x, O, D, rD, BVH_DESCRIPTOR );

		uint lastVisited = BVH_STACK_TERMINAL_NODE;
		uint type = id_to_type( currentNodeID );
		if (type <= PAIR)
		{
			float4 triData = as_float4( dst4 );
			float invDet = native_recip( triData[1] );
			float t = triData[0] * invDet;

			const global uint* triWords =
				(const global uint*)( (const global uchar*)alt4Node +
									  id_to_offset( currentNodeID ) );
			uint id = triWords[12 + type]; // primitive of tri0 at byte 48, of tri1 at byte 52

			if (t >= 0.0f && t <= hit.x)
			{
				float u = triData[2] * invDet;
				float v = triData[3] * invDet;
			#ifdef ISRDNA4
				// GFX12 uses the barycentric signs for metadata.
				u = fabs( u ), v = fabs( v );
			#endif
				hit = (float4)( t, u, v, as_float( id ) );
			}
			if (type == PAIR)
			{
				// test tri0 of the same node next, without a stack operation in between.
				pairID = currentNodeID, currentNodeID &= ~7u;
				continue;
			}
			// the parent refers to a pair by its type 1 ID; a stack recovery below must see that.
			if (pairID != INVALID_NODE) currentNodeID = pairID, pairID = INVALID_NODE;
			stackResults = __builtin_amdgcn_ds_bvh_stack_push4_pop1_rtn(
				stackAddr, BVH_STACK_TERMINAL_NODE, (uint4)INVALID_NODE, BVH_STACK_CONTROLS );
		}
		else
		{
			stackResults = __builtin_amdgcn_ds_bvh_stack_push4_pop1_rtn(
				stackAddr, prevNode, dst4, BVH_STACK_CONTROLS );
		}

		uint nextNodeID = stackResults.x;
		stackAddr = stackResults.y;
		prevNode = INVALID_NODE;

		if (nextNodeID != INVALID_NODE && nextNodeID != BVH_STACK_TERMINAL_NODE)
		{
			currentNodeID = nextNodeID;
			continue;
		}
		else if (nextNodeID == BVH_STACK_TERMINAL_NODE)
		{
			break;
		}

		// continue with nearest node or first node on the stack
	#ifndef ISRDNA4
		// Restore the GFX11 stack index after an empty-stack pop.
		stackAddr += 1u;
	#endif
		prevNode = currentNodeID;
		currentNodeID = parent_id( words, currentNodeID, box16Start, parentTable );

		if (currentNodeID == INVALID_NODE)
		{
			break;
		}
	}
	return hit;
}

bool isoccluded_gpu4wayHW( const global float4* alt4Node, const float4 O, const float4 D, const float4 rD, const float tmax, __local uint* hwStack )
{
	const ulong encoded_base = ( (ulong)alt4Node >> 3 ) & ( ( ( 1UL << 42 ) - 1UL ) << 3 );
	uint prevNode = INVALID_NODE;
	// LDS stack entries have a fixed stride of 32 words, even with wave64.
	uint lane = get_local_id( 1 ) * get_local_size( 0 ) + get_local_id( 0 );
	uint stackAddr = ( ( (uint)(size_t)hwStack >> 2 ) + ( lane & 31u ) +
					  ( lane >> 5 ) * ( 32u * HW_BVH_STACK_ENTRIES ) ) << HW_BVH_STACK_BASE_SHIFT;

	const global uint* words = (const global uint*)alt4Node;
	const uint box16Start = words[29], parentTable = words[30]; // for parent_id
	uint currentNodeID = 5; // start at root BOX32 node at byte offset zero.
	uint pairID = INVALID_NODE; // the pair node whose second test (as type 0) is in progress
	while (1)
	{
		uint2 stackResults;
		ulong currentPointer = encoded_base + (ulong)currentNodeID;
		uint4 dst4 = __builtin_amdgcn_image_bvh_intersect_ray_l(
			currentPointer, tmax, O, D, rD, BVH_DESCRIPTOR );

		uint type = id_to_type( currentNodeID );
		if (type <= PAIR)
		{
			const float4 triData = as_float4( dst4 );
			// Check the ray interval explicitly, using the closest-hit result format.
			const float t = triData.x * native_recip( triData.y );
			if (isfinite( t ) && t >= 0.0f && t < tmax) return true;
			if (type == PAIR)
			{
				pairID = currentNodeID, currentNodeID &= ~7u; // tri0 of the same node next
				continue;
			}
			if (pairID != INVALID_NODE) currentNodeID = pairID, pairID = INVALID_NODE;
			stackResults = __builtin_amdgcn_ds_bvh_stack_push4_pop1_rtn(
				stackAddr, BVH_STACK_TERMINAL_NODE, (uint4)INVALID_NODE, BVH_STACK_CONTROLS );
		}
		else
		{
			stackResults = __builtin_amdgcn_ds_bvh_stack_push4_pop1_rtn(
				stackAddr, prevNode, dst4, BVH_STACK_CONTROLS );
		}

		uint nextNodeID = stackResults.x;
		stackAddr = stackResults.y;
		prevNode = INVALID_NODE;

		if (nextNodeID != INVALID_NODE && nextNodeID != BVH_STACK_TERMINAL_NODE)
		{
			currentNodeID = nextNodeID;
			continue;
		}
		else if (nextNodeID == BVH_STACK_TERMINAL_NODE)
		{
			break;
		}

		// continue with nearest node or first node on the stack
		#ifndef ISRDNA4
		// Restore the GFX11 stack index after an empty-stack pop.
		stackAddr += 1u;
		#endif
		prevNode = currentNodeID;
		currentNodeID = parent_id( words, currentNodeID, box16Start, parentTable );

		if (currentNodeID == INVALID_NODE)
		{
			break;
		}
	}
	return false;
}
#endif // BLAS_AMD_HW_BVH4
#if BLAS_AMD_HW_CWBVH8
// ============================================================================
//
//        T R A V E R S E _ G P U 8 W A Y
//
// ============================================================================

// Node pointer encoding follows Mesa RADV (src/amd/vulkan/nir/radv_nir_rt_common.c).

inline uint bvh8_id_to_offset( uint id ) { return ( id & ( ~7u ) ) << 3; }
inline uint bvh8_id_to_type( uint id ) { return id & 7u; }
// Types 0..3 are a triangle pair; the type is the pair's index in its record (ID bit 3 adds 4).
inline bool bvh8_is_triangle( uint type ) { return type < 4u; }

#define BVH_STACK_SKIP_0_TO_7 0xfffffff9u
#if HW_BVH_STACK_ENTRIES != 8 && HW_BVH_STACK_ENTRIES != 16
#error "GFX12 hardware BVH stacks support 8 or 16 entries per ray in this kernel."
#endif
// BVH descriptor: zero base, distance sorting, 42-bit size, pointer flags enabled.
// Type 8; GFX12 triangle results contain t, barycentrics and primitive IDs.
#define BVH8_DESCRIPTOR                                      \
    ((uint4)(                                               \
        0u,                                                 \
        (1u << 31) | /* box sorting */                       \
        (1u << 20),  /* triangles first */                   \
        0xffffffffu, /* address-range low bits */            \
        (8u << 28) | /* BVH resource type */                 \
        (1u << 24) | /* triangle IJ return */                \
        (1u << 23) | /* pointer flags */                     \
        (1u << 22) | /* instance enable */                   \
        (1u << 21) | /* wide sorting */                      \
        (1u << 19) | /* compressed format */                 \
        0x3ffu       /* address-range high bits */           \
    ))

typedef uint uint10 __attribute__((ext_vector_type(10)));
typedef uint uint8 __attribute__((ext_vector_type(8)));

float4 traverse_gpu8wayHW( const global float4* alt4Node, const float4 O, const float4 D, const float4 rD, const float tmax, __local uint* hwStack )
{
	const ulong encoded_base = ( (ulong)alt4Node >> 3 ) & ( ( ( 1UL << 42 ) - 1UL ) << 3 );
	uint prevNode = INVALID_NODE;
	uint parentBoxID = INVALID_NODE;
	// LDS stack entries have a fixed stride of 32 words, even with wave64.
	uint lane = get_local_id( 1 ) * get_local_size( 0 ) + get_local_id( 0 );
	uint stackAddr = ( ( (uint)(size_t)hwStack >> 2 ) + ( lane & 31u ) +
					  ( lane >> 5 ) * ( 32u * HW_BVH_STACK_ENTRIES ) ) << HW_BVH_STACK_BASE_SHIFT;
	float4 hit = (float4)( tmax, 0.0f, 0.0f, as_float( INVALID_NODE ) );

	uint currentNodeID = 5; // start at root BOX32 node at byte offset zero.
	while (1)
	{
		uint2 stackResults;
		float3 outOrigin, outDirection;
		uint10 result = __builtin_amdgcn_image_bvh8_intersect_ray(
			encoded_base, prevNode != INVALID_NODE ? tmax : hit.x, (uchar)0xff, O.xyz, D.xyz, currentNodeID, BVH8_DESCRIPTOR, &outOrigin, &outDirection);

		uint type = bvh8_id_to_type( currentNodeID );
		if (bvh8_is_triangle( type ))
		{
			for (uint i = 0; i < 2; ++i)
			{
				const uint k = i * 4;
				const float t = as_float( result[k] );
				if (isfinite( t ) && t >= 0.0f && t <= hit.x)
				{
					const float u = fabs( as_float( result[k + 1] ) );
					const float v = fabs( as_float( result[k + 2] ) );
					const uint id = result[k + 3] >> 1;
					hit = (float4)( t, u, v, as_float( id ) );
				}
			}
			uint8 children = (uint8)INVALID_NODE;
			children.s7 = BVH_STACK_SKIP_0_TO_7;
			stackResults = __builtin_amdgcn_ds_bvh_stack_push8_pop1_rtn(
				stackAddr, INVALID_NODE, children, BVH_STACK_CONTROLS );
		}
		else
		{
			parentBoxID = currentNodeID;
			uint8 children;
			for (uint i = 0; i < 8; ++i) children[i] = result[i];
			stackResults = __builtin_amdgcn_ds_bvh_stack_push8_pop1_rtn(
				stackAddr, prevNode, children, BVH_STACK_CONTROLS );
		}

		uint nextNodeID = stackResults.x;
		stackAddr = stackResults.y;
		prevNode = INVALID_NODE;

		if (nextNodeID != INVALID_NODE && nextNodeID != BVH_STACK_TERMINAL_NODE)
		{
			currentNodeID = nextNodeID;
			continue;
		}
		else if (nextNodeID == BVH_STACK_TERMINAL_NODE)
		{
			break;
		}

		// continue with nearest node or first node on the stack
		prevNode = currentNodeID;
		uint parentID;
		const global uint* words =
			(const global uint*)( (const global uchar*)alt4Node +
								  bvh8_id_to_offset( currentNodeID ) );
		if (bvh8_is_triangle( type ))
		{
			parentID = parentBoxID;
		}
		else
		{
			parentID = words[2];
		}
		currentNodeID = parentID;

		if (currentNodeID == INVALID_NODE)
		{
			break;
		}
	}
	return hit;
}

bool isoccluded_gpu8wayHW( const global float4* alt4Node, const float4 O, const float4 D, const float4 rD, const float tmax, __local uint* hwStack )
{
	const ulong encoded_base = ( (ulong)alt4Node >> 3 ) & ( ( ( 1UL << 42 ) - 1UL ) << 3 );
	uint prevNode = INVALID_NODE;
	uint parentBoxID = INVALID_NODE;
	// LDS stack entries have a fixed stride of 32 words, even with wave64.
	uint lane = get_local_id( 1 ) * get_local_size( 0 ) + get_local_id( 0 );
	uint stackAddr = ( ( (uint)(size_t)hwStack >> 2 ) + ( lane & 31u ) +
					  ( lane >> 5 ) * ( 32u * HW_BVH_STACK_ENTRIES ) ) << HW_BVH_STACK_BASE_SHIFT;

	uint currentNodeID = 5; // start at root BOX32 node at byte offset zero.
	while (1)
	{
		uint2 stackResults;
		float3 outOrigin, outDirection;
		uint10 result = __builtin_amdgcn_image_bvh8_intersect_ray(
			encoded_base, tmax, (uchar)0xff, O.xyz, D.xyz, currentNodeID, BVH8_DESCRIPTOR, &outOrigin, &outDirection);

		uint type = bvh8_id_to_type( currentNodeID );
		if (bvh8_is_triangle( type ))
		{
			for (uint i = 0; i < 2; ++i)
			{
				const float t = as_float( result[i * 4] );
				if (isfinite( t ) && t >= 0.0f && t < tmax) return true;
			}
			uint8 children = (uint8)INVALID_NODE;
			children.s7 = BVH_STACK_SKIP_0_TO_7;
			stackResults = __builtin_amdgcn_ds_bvh_stack_push8_pop1_rtn(
				stackAddr, INVALID_NODE, children, BVH_STACK_CONTROLS );
		}
		else
		{
			parentBoxID = currentNodeID;
			uint8 children;
			for (uint i = 0; i < 8; ++i) children[i] = result[i];
			stackResults = __builtin_amdgcn_ds_bvh_stack_push8_pop1_rtn(
				stackAddr, prevNode, children, BVH_STACK_CONTROLS );
		}

		uint nextNodeID = stackResults.x;
		stackAddr = stackResults.y;
		prevNode = INVALID_NODE;

		if (nextNodeID != INVALID_NODE && nextNodeID != BVH_STACK_TERMINAL_NODE)
		{
			currentNodeID = nextNodeID;
			continue;
		}
		else if (nextNodeID == BVH_STACK_TERMINAL_NODE)
		{
			break;
		}

		// continue with nearest node or first node on the stack
		prevNode = currentNodeID;
		uint parentID;
		const global uint* words =
			(const global uint*)( (const global uchar*)alt4Node +
								  bvh8_id_to_offset( currentNodeID ) );
		if (bvh8_is_triangle( type ))
		{
			parentID = parentBoxID;
		}
		else
		{
			parentID = words[2];
		}
		currentNodeID = parentID;

		if (currentNodeID == INVALID_NODE)
		{
			break;
		}
	}
	return false;
}
#endif // BLAS_AMD_HW_CWBVH8
// ============================================================================
//
//        B L A S   D I S P A T C H
//
// ============================================================================

// Adding a BLAS layout takes five steps:
// 1. add a BLASTYPE_ constant and a BLAS_<NAME> switch at the top of this file;
// 2. paste its traversal functions above, between #if BLAS_<NAME> guards;
// 3. add its pool pointer(s) to BLAS_POOL_PARAMS / BLAS_POOL_ARGS below;
// 4. add a case to IntersectBLAS and IsOccludedBLAS;
// 5. mirror steps 1 and 3 on the host, in GPUTLAS.
// Pool offsets in BLASDesc are in 16-byte blocks, which is the common unit of
// every layout TinyBVH produces, so BLASDesc itself does not have to change.

#define BLAS_POOL_PARAMS \
	const global float4* alNodes, const global float4* alTris, \
	const global float4* cwNodes, const global float4* cwTris, \
	const global float4* amd4Nodes, const global float4* amd8Nodes

#define BLAS_POOL_ARGS alNodes, alTris, cwNodes, cwTris, amd4Nodes, amd8Nodes

#if BLAS_HW_STACK
#define BLAS_STACK_PARAM , __local uint* hwStack
#define BLAS_STACK_ARG , hwStack
#else
#define BLAS_STACK_PARAM
#define BLAS_STACK_ARG
#endif

// IntersectBLAS: 'find nearest' against a single BLAS, in BLAS space.
// Returns (t, u, v, as_float( primIdx )); t is tmax when nothing was hit.
float4 IntersectBLAS( const global struct BLASDesc* desc, BLAS_POOL_PARAMS,
	const float3 O, const float3 D, const float3 rD, const float tmax BLAS_STACK_PARAM )
{
	const uint blasType = desc->blasType;
#if BLAS_CWBVH || BLAS_AMD_HW_BVH4 || BLAS_AMD_HW_CWBVH8
	const float4 O4 = (float4)(O, 1), D4 = (float4)(D, 0), rD4 = (float4)(rD, 1);
#endif
#if BLAS_AILALAINE
	if (blasType == BLASTYPE_AILALAINE)
	{
		const global struct BVHNode* nodes = (const global struct BVHNode*)(alNodes + desc->nodeOffset);
		return traverse( nodes, alTris + desc->primOffset, O, D, rD, tmax, 0 );
	}
#endif
#if BLAS_CWBVH
	if (blasType == BLASTYPE_CWBVH)
	{
		const global float4* nodes = cwNodes + desc->nodeOffset;
		const global float4* tris = cwTris + desc->primOffset;
	#ifdef SIMD_AABBTEST
		return traverse_cwbvh( nodes, tris, O4, D4, rD4, tmax );
	#else
		return traverse_cwbvh( nodes, tris, O, D, rD, tmax );
	#endif
	}
#endif
#if BLAS_AMD_HW_BVH4
	if (blasType == BLASTYPE_AMD_HW_BVH4)
		return traverse_gpu4wayHW( amd4Nodes + desc->nodeOffset, O4, D4, rD4, tmax, hwStack );
#endif
#if BLAS_AMD_HW_CWBVH8
	if (blasType == BLASTYPE_AMD_HW_CWBVH8)
		return traverse_gpu8wayHW( amd8Nodes + desc->nodeOffset, O4, D4, rD4, tmax, hwStack );
#endif
	return (float4)(tmax, 0, 0, 0); // unsupported layout: report a miss.
}

// IsOccludedBLAS: 'any hit' against a single BLAS, in BLAS space.
bool IsOccludedBLAS( const global struct BLASDesc* desc, BLAS_POOL_PARAMS,
	const float3 O, const float3 D, const float3 rD, const float tmax BLAS_STACK_PARAM )
{
	const uint blasType = desc->blasType;
#if BLAS_CWBVH || BLAS_AMD_HW_BVH4 || BLAS_AMD_HW_CWBVH8
	const float4 O4 = (float4)(O, 1), D4 = (float4)(D, 0), rD4 = (float4)(rD, 1);
#endif
#if BLAS_AILALAINE
	if (blasType == BLASTYPE_AILALAINE)
	{
		const global struct BVHNode* nodes = (const global struct BVHNode*)(alNodes + desc->nodeOffset);
		return isoccluded( nodes, alTris + desc->primOffset, O, D, rD, tmax );
	}
#endif
#if BLAS_CWBVH
	if (blasType == BLASTYPE_CWBVH)
	{
		const global float4* nodes = cwNodes + desc->nodeOffset;
		const global float4* tris = cwTris + desc->primOffset;
	#ifdef SIMD_AABBTEST
		return isoccluded_cwbvh( nodes, tris, O4, D4, rD4, tmax );
	#else
		return isoccluded_cwbvh( nodes, tris, O, D, rD, tmax );
	#endif
	}
#endif
#if BLAS_AMD_HW_BVH4
	if (blasType == BLASTYPE_AMD_HW_BVH4)
		return isoccluded_gpu4wayHW( amd4Nodes + desc->nodeOffset, O4, D4, rD4, tmax, hwStack );
#endif
#if BLAS_AMD_HW_CWBVH8
	if (blasType == BLASTYPE_AMD_HW_CWBVH8)
		return isoccluded_gpu8wayHW( amd8Nodes + desc->nodeOffset, O4, D4, rD4, tmax, hwStack );
#endif
	return false; // unsupported layout: report a miss.
}

// ============================================================================
//
//        T R A V E R S E _ T L A S
//
// ============================================================================

// The TLAS is expected to be in BVH_GPU format: a 2-wide BVH in the layout
// proposed by Aila & Laine, chosen for its balance between efficient
// construction - it is rebuilt whenever an instance moves - and traversal.

float4 traverse_tlas(
	const global struct BVHNode* tlasNodes, const global uint* tlasIdx,
	const global struct Instance* instances, const global struct BLASDesc* blasDesc,
	BLAS_POOL_PARAMS, const float3 O, const float3 D, const float3 rD,
	const uint rayMask, const float tmax BLAS_STACK_PARAM )
{
	float4 hit = (float4)(tmax, 0, 0, 0);
	// safety net
	if (isnan( O.x + O.y + O.z + D.x + D.y + D.z )) return hit;
	// traverse the TLAS
	const float3 rO = O * -rD;
	uint nodeIdx = 0, stack[STACK_SIZE], stackPtr = 0;
	while (1)
	{
		if (nodeIdx & 0x80000000)
		{
			// leaf node: intersect the instances it holds
			const uint instCount = (nodeIdx >> 24) & 127;
			uint firstInst = nodeIdx & 0xffffff;
			for (uint i = 0; i < instCount; i++)
			{
				const uint instIdx = tlasIdx[firstInst + i];
				const global struct Instance* inst = instances + instIdx;
				if (!(rayMask & as_uint( inst->aabbMax.w ))) continue;
				const global struct BLASDesc* desc = blasDesc + as_uint( inst->aabbMin.w );
				const float3 Oblas = TransformPoint( O, inst->invTransform );
				const float3 Dblas = TransformVector( D, inst->invTransform );
				const float3 rDblas = native_recip( Dblas );
				const float4 blasHit = IntersectBLAS( desc, BLAS_POOL_ARGS,
					Oblas, Dblas, rDblas, hit.x BLAS_STACK_ARG );
				if (blasHit.x < hit.x)
				{
					hit = blasHit;
					hit.w = as_float( (as_uint( hit.w ) & PRIM_IDX_MASK) + (instIdx << INST_IDX_SHFT) );
				}
			}
			if (stackPtr == 0) break; else nodeIdx = stack[--stackPtr];
			continue;
		}
		const float4 lmin = tlasNodes[nodeIdx].lmin, lmax = tlasNodes[nodeIdx].lmax;
		const float4 rmin = tlasNodes[nodeIdx].rmin, rmax = tlasNodes[nodeIdx].rmax;
		uint left = as_uint( lmin.w ), right = as_uint( lmax.w );
		// child AABB intersection tests
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
		// traverse nearest child first
		if (dist1 > dist2)
		{
			float h = dist1; dist1 = dist2; dist2 = h;
			uint t = left; left = right; right = t;
		}
		if (dist1 == 1e30f) { if (stackPtr == 0) break; else nodeIdx = stack[--stackPtr]; }
		else { nodeIdx = left; if (dist2 != 1e30f) stack[stackPtr++] = right; }
	}
	return hit;
}

bool isoccluded_tlas(
	const global struct BVHNode* tlasNodes, const global uint* tlasIdx,
	const global struct Instance* instances, const global struct BLASDesc* blasDesc,
	BLAS_POOL_PARAMS, const float3 O, const float3 D, const float3 rD,
	const uint rayMask, const float tmax BLAS_STACK_PARAM )
{
	// safety net
	if (isnan( O.x + O.y + O.z + D.x + D.y + D.z )) return true;
	// traverse the TLAS
	const float3 rO = O * -rD;
	uint nodeIdx = 0, stack[STACK_SIZE], stackPtr = 0;
	while (1)
	{
		if (nodeIdx & 0x80000000)
		{
			// leaf node: test the instances it holds
			const uint instCount = (nodeIdx >> 24) & 127;
			uint firstInst = nodeIdx & 0xffffff;
			for (uint i = 0; i < instCount; i++)
			{
				const uint instIdx = tlasIdx[firstInst + i];
				const global struct Instance* inst = instances + instIdx;
				if (!(rayMask & as_uint( inst->aabbMax.w ))) continue;
				const global struct BLASDesc* desc = blasDesc + as_uint( inst->aabbMin.w );
				const float3 Oblas = TransformPoint( O, inst->invTransform );
				const float3 Dblas = TransformVector( D, inst->invTransform );
				const float3 rDblas = native_recip( Dblas );
				if (IsOccludedBLAS( desc, BLAS_POOL_ARGS,
					Oblas, Dblas, rDblas, tmax BLAS_STACK_ARG )) return true;
			}
			if (stackPtr == 0) break; else nodeIdx = stack[--stackPtr];
			continue;
		}
		const float4 lmin = tlasNodes[nodeIdx].lmin, lmax = tlasNodes[nodeIdx].lmax;
		const float4 rmin = tlasNodes[nodeIdx].rmin, rmax = tlasNodes[nodeIdx].rmax;
		uint left = as_uint( lmin.w ), right = as_uint( lmax.w );
		// child AABB intersection tests
		const float3 t1a = fma( lmin.xyz, rD, rO ), t2a = fma( lmax.xyz, rD, rO );
		const float3 t1b = fma( rmin.xyz, rD, rO ), t2b = fma( rmax.xyz, rD, rO );
		const float3 minta = fmin( t1a, t2a ), maxta = fmax( t1a, t2a );
		const float3 mintb = fmin( t1b, t2b ), maxtb = fmax( t1b, t2b );
		const float tmina = fmax( fmax( fmax( minta.x, minta.y ), minta.z ), 0 );
		const float tminb = fmax( fmax( fmax( mintb.x, mintb.y ), mintb.z ), 0 );
		const float tmaxa = fmin( fmin( fmin( maxta.x, maxta.y ), maxta.z ), tmax );
		const float tmaxb = fmin( fmin( fmin( maxtb.x, maxtb.y ), maxtb.z ), tmax );
		const bool hitA = tmina <= tmaxa, hitB = tminb <= tmaxb;
		if (hitA && hitB)
		{
			uint nearChild = left, farChild = right;
			if (tminb < tmina) nearChild = right, farChild = left;
			stack[stackPtr++] = farChild, nodeIdx = nearChild;
		}
		else if (hitA) nodeIdx = left;
		else if (hitB) nodeIdx = right;
		else { if (stackPtr == 0) break; nodeIdx = stack[--stackPtr]; }
	}
	return false;
}

// ============================================================================
//
//        K E R N E L S
//
// ============================================================================

#if BLAS_HW_STACK
__attribute__(( reqd_work_group_size( HW_BVH_WORKGROUP_SIZE, 1, 1 ) ))
#endif
void kernel batch_nearest_tlas(
	const global struct BVHNode* tlasNodes, const global uint* tlasIdx,
	const global struct Instance* instances, const global struct BLASDesc* blasDesc,
	BLAS_POOL_PARAMS, global struct Ray* rayData, const uint rayCount )
{
	const uint threadId = get_global_id( 0 );
	if (threadId >= rayCount) return;
#if BLAS_HW_STACK
	__local uint hwStack[HW_BVH_WORKGROUP_SIZE * HW_BVH_STACK_ENTRIES];
#endif
	const float4 O4 = rayData[threadId].O, D4 = rayData[threadId].D;
	const float3 rD = native_recip( D4.xyz );
	const float4 hit = traverse_tlas( tlasNodes, tlasIdx, instances, blasDesc,
		BLAS_POOL_ARGS, O4.xyz, D4.xyz, rD, as_uint( O4.w ),
		rayData[threadId].hit.x BLAS_STACK_ARG );
	rayData[threadId].hit = hit;
}

#if BLAS_HW_STACK
__attribute__(( reqd_work_group_size( HW_BVH_WORKGROUP_SIZE, 1, 1 ) ))
#endif
void kernel batch_any_tlas(
	const global struct BVHNode* tlasNodes, const global uint* tlasIdx,
	const global struct Instance* instances, const global struct BLASDesc* blasDesc,
	BLAS_POOL_PARAMS, global struct Ray* rayData, const uint rayCount )
{
	const uint threadId = get_global_id( 0 );
	if (threadId >= rayCount) return;
#if BLAS_HW_STACK
	__local uint hwStack[HW_BVH_WORKGROUP_SIZE * HW_BVH_STACK_ENTRIES];
#endif
	const float4 O4 = rayData[threadId].O, D4 = rayData[threadId].D;
	const float3 rD = native_recip( D4.xyz );
	float4 hit = (float4)(0, 0, 0, 0);
	if (isoccluded_tlas( tlasNodes, tlasIdx, instances, blasDesc,
		BLAS_POOL_ARGS, O4.xyz, D4.xyz, rD, as_uint( O4.w ),
		rayData[threadId].hit.x BLAS_STACK_ARG )) hit.w = as_float( 1u );
	rayData[threadId].hit = hit;
}

// Packed variants: 32-byte rays (O.xyz, tmax; D.xyz, ray mask as uint bits) in; 16-byte hits, or one occlusion
// word (1: occluded) for _any, out to a separate array. Faster than writing hits into 64-byte Rays.
#if BLAS_HW_STACK
__attribute__(( reqd_work_group_size( HW_BVH_WORKGROUP_SIZE, 1, 1 ) ))
#endif
void kernel batch_nearest_tlas_packed(
	const global struct BVHNode* tlasNodes, const global uint* tlasIdx,
	const global struct Instance* instances, const global struct BLASDesc* blasDesc,
	BLAS_POOL_PARAMS, const global float4* rays, global float4* hits, const uint rayCount )
{
	const uint threadId = get_global_id( 0 );
	if (threadId >= rayCount) return;
#if BLAS_HW_STACK
	__local uint hwStack[HW_BVH_WORKGROUP_SIZE * HW_BVH_STACK_ENTRIES];
#endif
	const float4 O4 = rays[threadId * 2], D4 = rays[threadId * 2 + 1];
	const float3 rD = native_recip( D4.xyz );
	hits[threadId] = traverse_tlas( tlasNodes, tlasIdx, instances, blasDesc,
		BLAS_POOL_ARGS, O4.xyz, D4.xyz, rD, as_uint( D4.w ), O4.w BLAS_STACK_ARG );
}

#if BLAS_HW_STACK
__attribute__(( reqd_work_group_size( HW_BVH_WORKGROUP_SIZE, 1, 1 ) ))
#endif
void kernel batch_any_tlas_packed(
	const global struct BVHNode* tlasNodes, const global uint* tlasIdx,
	const global struct Instance* instances, const global struct BLASDesc* blasDesc,
	BLAS_POOL_PARAMS, const global float4* rays, global uint* occluded, const uint rayCount )
{
	const uint threadId = get_global_id( 0 );
	if (threadId >= rayCount) return;
#if BLAS_HW_STACK
	__local uint hwStack[HW_BVH_WORKGROUP_SIZE * HW_BVH_STACK_ENTRIES];
#endif
	const float4 O4 = rays[threadId * 2], D4 = rays[threadId * 2 + 1];
	const float3 rD = native_recip( D4.xyz );
	occluded[threadId] = isoccluded_tlas( tlasNodes, tlasIdx, instances, blasDesc,
		BLAS_POOL_ARGS, O4.xyz, D4.xyz, rD, as_uint( D4.w ), O4.w BLAS_STACK_ARG ) ? 1 : 0;
}
