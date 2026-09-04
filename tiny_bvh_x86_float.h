// tiny_bvh_x86_float.h: SSE / AVX / AVX2 specializations for the single
// precision layouts. Included by tiny_bvh.h; do not include directly.

#ifndef TINY_BVH_H_
#error "Include tiny_bvh.h instead of tiny_bvh_x86_float.h."
#endif

#ifndef TINY_BVH_X86_FLOAT_H_
#define TINY_BVH_X86_FLOAT_H_

namespace tinybvh {

#ifdef BVH_USESSE
TINYBVH_FORCEINLINE __m128 tinybvh_load4( const void* p ) { __m128 r; memcpy( &r, p, 16 ); return r; }
TINYBVH_FORCEINLINE __m128i tinybvh_load4i( const void* p ) { __m128i r; memcpy( &r, p, 16 ); return r; }
TINYBVH_FORCEINLINE void tinybvh_store4( void* p, const __m128 v ) { memcpy( p, &v, 16 ); }
TINYBVH_FORCEINLINE void tinybvh_store4i( void* p, const __m128i v ) { memcpy( p, &v, 16 ); }
// extract a single 32-bit lane straight out of a register
#define TINYBVH_LANE0(V) ((uint32_t)_mm_cvtsi128_si32( V ))
#define TINYBVH_LANE1(V) ((uint32_t)_mm_extract_epi32( V, 1 ))
#define TINYBVH_LANE2(V) ((uint32_t)_mm_extract_epi32( V, 2 ))
#define TINYBVH_LANE3(V) ((uint32_t)_mm_extract_epi32( V, 3 ))
#endif
#ifdef BVH_USEAVX
TINYBVH_FORCEINLINE __m256 tinybvh_load8( const void* p ) { __m256 r; memcpy( &r, p, 32 ); return r; }
TINYBVH_FORCEINLINE __m256i tinybvh_load8i( const void* p ) { __m256i r; memcpy( &r, p, 32 ); return r; }
TINYBVH_FORCEINLINE void tinybvh_store8( void* p, const __m256 v ) { memcpy( p, &v, 32 ); }
TINYBVH_FORCEINLINE void tinybvh_store8i( void* p, const __m256i v ) { memcpy( p, &v, 32 ); }
#endif

// Specializations provided by this header.
#ifdef BVH_USESSE
template <> bool impl::BVH<float, uint32_t>::SplitFrag( const Fragment& orig, Fragment& left, Fragment& right, const uint32_t axis, const float pos ) const;
template <> bool impl::BVH<float, uint32_t>::ClipFrag( const Fragment& orig, Fragment& newFrag, bvhvec3 bmin, bvhvec3 bmax, const uint32_t axis ) const;
template <> template <bool posX, bool posY, bool posZ> int32_t impl::BVH4_CPU<float, uint32_t>::IntersectOctant( Ray& ray ) const;
template <> template <bool posX, bool posY, bool posZ> bool impl::BVH4_CPU<float, uint32_t>::IsOccludedOctant( const Ray& ray ) const;
#endif
#ifdef BVH_USEAVX
template <> struct impl::BVHSIMDBuilders<float, uint32_t> { static constexpr bool avx = true; };
template <> void impl::BVH<float, uint32_t>::BuildAVX( const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t primCount );
template <> void impl::BVH<float, uint32_t>::PrepareSIMDBuild( const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t primCount );
template <> void impl::BVH<float, uint32_t>::PrepareSIMDBuildFragSlice( const uint32_t first, const uint32_t last, const uint32_t* indices, const int8_t* vertData, const uint32_t stride4, void* frags, float* rootMin, float* rootMax );
template <> void impl::BVH<float, uint32_t>::BuildSIMDBinTask( const uint32_t first, const uint32_t last, void* binbox, uint32_t* count, const float* nmin4, const float* rpd4 );
template <> void impl::BVH<float, uint32_t>::BuildSIMDSubtree( uint32_t nodeIdx, uint32_t depth );
template <> void impl::BVH<float, uint32_t>::BuildSIMDFinalize();
template <> void impl::BVH<float, uint32_t>::Intersect256RaysSSE( Ray* packet ) const;
#ifdef ENABLE_BVH_SOA
template <> int32_t impl::BVH_SoA<float, uint32_t>::Intersect( Ray& ray ) const;
template <> bool impl::BVH_SoA<float, uint32_t>::IsOccluded( const Ray& ray ) const;
#endif
#endif
#ifdef BVH_USEAVX2
template <> template <bool posX, bool posY, bool posZ> int32_t impl::BVH8_CPU<float, uint32_t>::IntersectOctant( Ray& ray ) const;
template <> template <bool posX, bool posY, bool posZ> bool impl::BVH8_CPU<float, uint32_t>::IsOccludedOctant( const Ray& ray ) const;
#endif

} // namespace tinybvh

#endif // TINY_BVH_X86_FLOAT_H_

// ============================================================================
//
//        I M P L E M E N T A T I O N  -  A V X / S S E  C O D E
//
// ============================================================================

#ifdef TINYBVH_IMPLEMENTATION

namespace tinybvh {

#ifdef BVH_USESSE

// SIMD constants - Functions rather than mutable statics. Calls fold to a constant or a single broadcast.
TINYBVH_FORCEINLINE __m128 bvhc_min1() { return _mm_set1_ps( -1.0f ); }
TINYBVH_FORCEINLINE __m128 bvhc_binmul3() { return _mm_set1_ps( AVXBINS * 0.49999f ); }
TINYBVH_FORCEINLINE __m128 bvhc_mask3() { return _mm_cmpeq_ps( _mm_setr_ps( 0, 0, 0, 1 ), _mm_setzero_ps() ); }
// SIMD lane access
#if defined _MSC_VER && !defined __clang__
#define LANE(a,b) a.m128_f32[b]
// Not using clang/g++ method under MSCC; compiler may benefit from .m128_i32.
#define ILANE(a,b) a.m128i_i32[b]
#else
#define LANE(a,b) a[b]
// Below method reduces to a single instruction.
#define ILANE(a,b) _mm_cvtsi128_si32(_mm_castps_si128( _mm_shuffle_ps(_mm_castsi128_ps( a ), _mm_castsi128_ps( a ), b)))
#endif
// AABB halfarea calculation
TINYBVH_FORCEINLINE float halfArea( const __m128 a /* a contains extent of aabb */ )
{
	return LANE( a, 0 ) * LANE( a, 1 ) + LANE( a, 1 ) * LANE( a, 2 ) + LANE( a, 2 ) * LANE( a, 3 );
}

// SplitFrag: cut a fragment in two new fragments. Based on madmann91 code.
template <> bool impl::BVH<float, uint32_t>::SplitFrag( const Fragment& orig, Fragment& left, Fragment& right, const uint32_t axis, const float pos ) const
{
	__m128 lbmin4, lbmax4, rbmin4, rbmax4;
	lbmin4 = _mm_set1_ps( BVH_FAR ), rbmin4 = lbmin4;
	lbmax4 = _mm_set1_ps( -BVH_FAR ), rbmax4 = lbmax4;
	bvhvec4 v0, v1, v2;
	const uint32_t vidx = orig.primIdx * 3;
	if (!vertIdx) v0 = verts[vidx], v1 = verts[vidx + 1], v2 = verts[vidx + 2];
	else v0 = verts[vertIdx[vidx]], v1 = verts[vertIdx[vidx + 1]], v2 = verts[vertIdx[vidx + 2]];
	const __m128 v0_4 = tinybvh_load4( &v0 ), v1_4 = tinybvh_load4( &v1 ), v2_4 = tinybvh_load4( &v2 );
	const bool l0 = v0[axis] <= pos, l1 = v1[axis] <= pos, l2 = v2[axis] <= pos;
	if (l0) lbmin4 = _mm_min_ps( lbmin4, v0_4 ), lbmax4 = _mm_max_ps( lbmax4, v0_4 );
	else rbmin4 = _mm_min_ps( rbmin4, v0_4 ), rbmax4 = _mm_max_ps( rbmax4, v0_4 );
	if (l1) lbmin4 = _mm_min_ps( lbmin4, v1_4 ), lbmax4 = _mm_max_ps( lbmax4, v1_4 );
	else rbmin4 = _mm_min_ps( rbmin4, v1_4 ), rbmax4 = _mm_max_ps( rbmax4, v1_4 );
	if (l2) lbmin4 = _mm_min_ps( lbmin4, v2_4 ), lbmax4 = _mm_max_ps( lbmax4, v2_4 );
	else rbmin4 = _mm_min_ps( rbmin4, v2_4 ), rbmax4 = _mm_max_ps( rbmax4, v2_4 );
	bvhvec4 c; __m128 c4;
	if (l0 ^ l1)
		c = v0 + (pos - v0[axis]) / (v1[axis] - v0[axis]) * (v1 - v0), c[axis] = pos, c4 = tinybvh_load4( &c ),
		lbmin4 = _mm_min_ps( lbmin4, c4 ), lbmax4 = _mm_max_ps( lbmax4, c4 ),
		rbmin4 = _mm_min_ps( rbmin4, c4 ), rbmax4 = _mm_max_ps( rbmax4, c4 );
	if (l1 ^ l2)
		c = v1 + (pos - v1[axis]) / (v2[axis] - v1[axis]) * (v2 - v1), c[axis] = pos, c4 = tinybvh_load4( &c ),
		lbmin4 = _mm_min_ps( lbmin4, c4 ), lbmax4 = _mm_max_ps( lbmax4, c4 ),
		rbmin4 = _mm_min_ps( rbmin4, c4 ), rbmax4 = _mm_max_ps( rbmax4, c4 );
	if (l2 ^ l0)
		c = v2 + (pos - v2[axis]) / (v0[axis] - v2[axis]) * (v0 - v2), c[axis] = pos, c4 = tinybvh_load4( &c ),
		lbmin4 = _mm_min_ps( lbmin4, c4 ), lbmax4 = _mm_max_ps( lbmax4, c4 ),
		rbmin4 = _mm_min_ps( rbmin4, c4 ), rbmax4 = _mm_max_ps( rbmax4, c4 );
	if (orig.clipped) // clip against orig box
		lbmin4 = _mm_max_ps( lbmin4, _mm_and_ps( tinybvh_load4( &orig.bmin ), bvhc_mask3() ) ),
		lbmax4 = _mm_min_ps( lbmax4, _mm_and_ps( tinybvh_load4( &orig.bmax ), bvhc_mask3() ) ),
		rbmin4 = _mm_max_ps( rbmin4, _mm_and_ps( tinybvh_load4( &orig.bmin ), bvhc_mask3() ) ),
		rbmax4 = _mm_min_ps( rbmax4, _mm_and_ps( tinybvh_load4( &orig.bmax ), bvhc_mask3() ) );
	tinybvh_store4( &left.bmin, lbmin4 ), tinybvh_store4( &right.bmin, rbmin4 );
	tinybvh_store4( &left.bmax, lbmax4 ), tinybvh_store4( &right.bmax, rbmax4 );
	left.primIdx = right.primIdx = orig.primIdx;
	left.clipped = right.clipped = true;
	return tinybvh_halfarea( left.bmax - left.bmin ) > 0 && tinybvh_halfarea( right.bmax - right.bmin ) > 0;
}

// ClipFrag: clip a fragment for binning.
template <> bool impl::BVH<float, uint32_t>::ClipFrag( const Fragment& orig, Fragment& newFrag, bvhvec3 bmin, bvhvec3 bmax, const uint32_t axis ) const
{
	__m128 t1min4, t1max4, t2min4, t2max4;
	t1min4 = t2min4 = _mm_set1_ps( BVH_FAR );
	t1max4 = t2max4 = _mm_set1_ps( -BVH_FAR );
	bvhvec4 v0, v1, v2;
	const uint32_t vidx = orig.primIdx * 3;
	if (!vertIdx) v0 = verts[vidx], v1 = verts[vidx + 1], v2 = verts[vidx + 2];
	else v0 = verts[vertIdx[vidx]], v1 = verts[vertIdx[vidx + 1]], v2 = verts[vertIdx[vidx + 2]];
	const __m128 v0_4 = tinybvh_load4( &v0 ), v1_4 = tinybvh_load4( &v1 ), v2_4 = tinybvh_load4( &v2 );
	const float left = bmin[axis], right = bmax[axis];
	// clip against min bounds
	bool in0 = v0[axis] >= left, in1 = v1[axis] >= left, in2 = v2[axis] >= left;
	if (in0) t1min4 = _mm_min_ps( t1min4, v0_4 ), t1max4 = _mm_max_ps( t1max4, v0_4 );
	if (in1) t1min4 = _mm_min_ps( t1min4, v1_4 ), t1max4 = _mm_max_ps( t1max4, v1_4 );
	if (in2) t1min4 = _mm_min_ps( t1min4, v2_4 ), t1max4 = _mm_max_ps( t1max4, v2_4 );
	bvhvec4 c; __m128 c4;
	if (in0 ^ in1)
		c = v0 + (left - v0[axis]) / (v1[axis] - v0[axis]) * (v1 - v0), c[axis] = left, c4 = tinybvh_load4( &c ),
		t1min4 = _mm_min_ps( t1min4, c4 ), t1max4 = _mm_max_ps( t1max4, c4 ),
		t1min4 = _mm_min_ps( t1min4, c4 ), t1max4 = _mm_max_ps( t1max4, c4 );
	if (in1 ^ in2)
		c = v1 + (left - v1[axis]) / (v2[axis] - v1[axis]) * (v2 - v1), c[axis] = left, c4 = tinybvh_load4( &c ),
		t1min4 = _mm_min_ps( t1min4, c4 ), t1max4 = _mm_max_ps( t1max4, c4 ),
		t1min4 = _mm_min_ps( t1min4, c4 ), t1max4 = _mm_max_ps( t1max4, c4 );
	if (in2 ^ in0)
		c = v2 + (left - v2[axis]) / (v0[axis] - v2[axis]) * (v0 - v2), c[axis] = left, c4 = tinybvh_load4( &c ),
		t1min4 = _mm_min_ps( t1min4, c4 ), t1max4 = _mm_max_ps( t1max4, c4 ),
		t1min4 = _mm_min_ps( t1min4, c4 ), t1max4 = _mm_max_ps( t1max4, c4 );
	// clip against max bounds
	in0 = v0[axis] <= right, in1 = v1[axis] <= right, in2 = v2[axis] <= right;
	if (in0) t2min4 = _mm_min_ps( t2min4, v0_4 ), t2max4 = _mm_max_ps( t2max4, v0_4 );
	if (in1) t2min4 = _mm_min_ps( t2min4, v1_4 ), t2max4 = _mm_max_ps( t2max4, v1_4 );
	if (in2) t2min4 = _mm_min_ps( t2min4, v2_4 ), t2max4 = _mm_max_ps( t2max4, v2_4 );
	if (in0 ^ in1)
		c = v0 + (right - v0[axis]) / (v1[axis] - v0[axis]) * (v1 - v0), c[axis] = right, c4 = tinybvh_load4( &c ),
		t2min4 = _mm_min_ps( t2min4, c4 ), t2max4 = _mm_max_ps( t2max4, c4 ),
		t2min4 = _mm_min_ps( t2min4, c4 ), t2max4 = _mm_max_ps( t2max4, c4 );
	if (in1 ^ in2)
		c = v1 + (right - v1[axis]) / (v2[axis] - v1[axis]) * (v2 - v1), c[axis] = right, c4 = tinybvh_load4( &c ),
		t2min4 = _mm_min_ps( t2min4, c4 ), t2max4 = _mm_max_ps( t2max4, c4 ),
		t2min4 = _mm_min_ps( t2min4, c4 ), t2max4 = _mm_max_ps( t2max4, c4 );
	if (in2 ^ in0)
		c = v2 + (right - v2[axis]) / (v0[axis] - v2[axis]) * (v0 - v2), c[axis] = right, c4 = tinybvh_load4( &c ),
		t2min4 = _mm_min_ps( t2min4, c4 ), t2max4 = _mm_max_ps( t2max4, c4 ),
		t2min4 = _mm_min_ps( t2min4, c4 ), t2max4 = _mm_max_ps( t2max4, c4 );
	__m128 finalmin4, finalmax4;
	if (orig.clipped) // clip against orig box
		finalmin4 = _mm_max_ps( _mm_max_ps( t1min4, t2min4 ), _mm_and_ps( tinybvh_load4( &orig.bmin ), bvhc_mask3() ) ),
		finalmax4 = _mm_min_ps( _mm_min_ps( t1max4, t2max4 ), _mm_and_ps( tinybvh_load4( &orig.bmax ), bvhc_mask3() ) );
	else
		finalmin4 = _mm_max_ps( t1min4, t2min4 ),
		finalmax4 = _mm_min_ps( t1max4, t2max4 );
	tinybvh_store4( &newFrag.bmin, finalmin4 );
	tinybvh_store4( &newFrag.bmax, finalmax4 );
	newFrag.primIdx = orig.primIdx;
	newFrag.clipped = true;
	const float sa = tinybvh_halfarea( newFrag.bmax - newFrag.bmin );
	return sa > 0;
}

// SSE box tests for BVH::EPOArea.
namespace impl {
inline bool tinybvh_aabbs_overlap( const BVH<float, uint32_t>::BVHNode& node1, const BVH<float, uint32_t>::BVHNode& node2 )
{
	const __m128 n1min4 = tinybvh_load4( &node1.aabbMin ), n1max4 = tinybvh_load4( &node1.aabbMax );
	const __m128 n2min4 = tinybvh_load4( &node2.aabbMin ), n2max4 = tinybvh_load4( &node2.aabbMax );
	return (_mm_movemask_ps( _mm_and_ps( _mm_cmple_ps( n1min4, n2max4 ), _mm_cmpge_ps( n1max4, n2min4 ) ) ) & 7) == 7;
}
} // namespace impl

inline bool tinybvh_tri_inside_box( const bvhvec4& v0, const bvhvec4& v1, const bvhvec4& v2, const bvhvec3& bmin, const bvhvec3& bmax )
{
	const __m128 bmin4 = _mm_setr_ps( bmin.x, bmin.y, bmin.z, 0 ), bmax4 = _mm_setr_ps( bmax.x, bmax.y, bmax.z, 0 );
	const __m128 v04 = tinybvh_load4( &v0 ), v14 = tinybvh_load4( &v1 ), v24 = tinybvh_load4( &v2 );
	const __m128 vmin4 = _mm_min_ps( _mm_min_ps( v04, v14 ), v24 ), vmax4 = _mm_max_ps( _mm_max_ps( v04, v14 ), v24 );
	return (_mm_movemask_ps( _mm_and_ps( _mm_cmpge_ps( vmin4, bmin4 ), _mm_cmple_ps( vmax4, bmax4 ) ) ) & 7) == 7;
}

#define SSE_HIT( s ) ((m >> s) & 1)
#define SSE_PUSH( c, s ) { nodeStack[stackPtr] = c; distStack[stackPtr] = tminSorted[s]; stackPtr++; }

template <> template <bool posX, bool posY, bool posZ> int32_t impl::BVH4_CPU<float, uint32_t>::IntersectOctant( Ray& ray ) const
{
	ALIGNED( 64 ) uint32_t nodeStack[TINYBVH_STACK_SIZE * 2 /* wide trees push more nodes per step */];
	ALIGNED( 64 ) float distStack[TINYBVH_STACK_SIZE * 2];
	ALIGNED( 16 ) float tminSorted[4];
	const __m128 zero4 = _mm_setzero_ps();
	__m128 t4 = _mm_set1_ps( ray.hit.t );
	int32_t stackPtr = 0;
	uint32_t nodeIdx = 0;
	float tcur = ray.hit.t;
	constexpr int signShift = (posX ? 2 : 0) + (posY ? 4 : 0) + (posZ ? 8 : 0);
	const __m128 rx4 = _mm_set1_ps( ray.O.x * ray.rD.x ), rdx4 = _mm_set1_ps( ray.rD.x );
	const __m128 ry4 = _mm_set1_ps( ray.O.y * ray.rD.y ), rdy4 = _mm_set1_ps( ray.rD.y );
	const __m128 rz4 = _mm_set1_ps( ray.O.z * ray.rD.z ), rdz4 = _mm_set1_ps( ray.rD.z );
	const __m128 ox4 = _mm_set1_ps( ray.O.x ), oy4 = _mm_set1_ps( ray.O.y ), oz4 = _mm_set1_ps( ray.O.z );
	const __m128 dx4 = _mm_set1_ps( ray.D.x ), dy4 = _mm_set1_ps( ray.D.y ), dz4 = _mm_set1_ps( ray.D.z );
	const __m128 one4 = _mm_set1_ps( 1 ), inf4 = _mm_set1_ps( 1e34f );
#ifndef BVH_USEAVX
	const __m128i shftmsk4 = _mm_set1_epi32( 3 ), mul4 = _mm_set1_epi32( 0x04040404 ), add4 = _mm_set1_epi32( 0x03020100 );
#endif
#ifdef _DEBUG
	// sorry, not even this can be tolerated in this function. Only in debug.
	uint32_t steps = 0;
#endif
	while (1)
	{
	#ifdef _DEBUG
		steps++;
	#endif
		while (!(nodeIdx & LEAF_BIT))
		{
			const BVHNode* n = (BVHNode*)(bvh4Data + nodeIdx);
			const uint32_t* child = n->child, * perm = n->perm;
		#ifdef BVH_USEAVX2
			const __m128 tx1 = _mm_fmsub_ps( _mm_load_ps( posX ? n->xmin : n->xmax ), rdx4, rx4 );
			const __m128 ty1 = _mm_fmsub_ps( _mm_load_ps( posY ? n->ymin : n->ymax ), rdy4, ry4 );
			const __m128 tz1 = _mm_fmsub_ps( _mm_load_ps( posZ ? n->zmin : n->zmax ), rdz4, rz4 );
			const __m128 tx2 = _mm_fmsub_ps( _mm_load_ps( posX ? n->xmax : n->xmin ), rdx4, rx4 );
			const __m128 ty2 = _mm_fmsub_ps( _mm_load_ps( posY ? n->ymax : n->ymin ), rdy4, ry4 );
			const __m128 tz2 = _mm_fmsub_ps( _mm_load_ps( posZ ? n->zmax : n->zmin ), rdz4, rz4 );
		#else
			const __m128 tx1 = _mm_sub_ps( _mm_mul_ps( _mm_load_ps( posX ? n->xmin : n->xmax ), rdx4 ), rx4 );
			const __m128 ty1 = _mm_sub_ps( _mm_mul_ps( _mm_load_ps( posY ? n->ymin : n->ymax ), rdy4 ), ry4 );
			const __m128 tz1 = _mm_sub_ps( _mm_mul_ps( _mm_load_ps( posZ ? n->zmin : n->zmax ), rdz4 ), rz4 );
			const __m128 tx2 = _mm_sub_ps( _mm_mul_ps( _mm_load_ps( posX ? n->xmax : n->xmin ), rdx4 ), rx4 );
			const __m128 ty2 = _mm_sub_ps( _mm_mul_ps( _mm_load_ps( posY ? n->ymax : n->ymin ), rdy4 ), ry4 );
			const __m128 tz2 = _mm_sub_ps( _mm_mul_ps( _mm_load_ps( posZ ? n->zmax : n->zmin ), rdz4 ), rz4 );
		#endif
			const __m128 tmin = _mm_max_ps( _mm_max_ps( zero4, tx1 ), _mm_max_ps( ty1, tz1 ) );
			const __m128 tmax = _mm_min_ps( _mm_min_ps( tx2, t4 ), _mm_min_ps( ty2, tz2 ) );
			const __m128 mask4 = _mm_cmple_ps( tmin, tmax );
			// child index at each sorted position, loaded independently of the slab test
			const uint32_t c3 = child[(perm[3] >> signShift) & 3], c2 = child[(perm[2] >> signShift) & 3];
			const uint32_t c1 = child[(perm[1] >> signShift) & 3], c0 = child[(perm[0] >> signShift) & 3];
			// slab mask and entry distances in sorted order
		#ifdef BVH_USEAVX
			const __m128i index = _mm_srli_epi32( _mm_load_si128( (const __m128i*)perm ), signShift );
			const uint32_t m = _mm_movemask_ps( _mm_permutevar_ps( mask4, index ) );
			_mm_store_ps( tminSorted, _mm_permutevar_ps( tmin, index ) );
		#else
			const __m128i raw4 = _mm_and_si128( _mm_srli_epi32( _mm_load_si128( (const __m128i*)perm ), signShift ), shftmsk4 );
			const __m128i shfl16 = _mm_add_epi32( _mm_mullo_epi32( raw4, mul4 ), add4 );
			const uint32_t m = _mm_movemask_ps( _mm_castsi128_ps( _mm_shuffle_epi8( _mm_castps_si128( mask4 ), shfl16 ) ) );
			_mm_store_ps( tminSorted, _mm_castsi128_ps( _mm_shuffle_epi8( _mm_castps_si128( tmin ), shfl16 ) ) );
		#endif
			// continue with the nearest valid child and push the others, farthest first
			if (SSE_HIT( 3 ))
			{
				if (SSE_HIT( 0 )) SSE_PUSH( c0, 0 );
				if (SSE_HIT( 1 )) SSE_PUSH( c1, 1 );
				if (SSE_HIT( 2 )) SSE_PUSH( c2, 2 );
				nodeIdx = c3;
			}
			else if (SSE_HIT( 2 ))
			{
				if (SSE_HIT( 0 )) SSE_PUSH( c0, 0 );
				if (SSE_HIT( 1 )) SSE_PUSH( c1, 1 );
				nodeIdx = c2;
			}
			else if (SSE_HIT( 1 ))
			{
				if (SSE_HIT( 0 )) SSE_PUSH( c0, 0 );
				nodeIdx = c1;
			}
			else if (SSE_HIT( 0 )) nodeIdx = c0;
			else
			{
				// skip entries behind the current hit
				do { if (!stackPtr) goto the_end; nodeIdx = nodeStack[--stackPtr]; } while (distStack[stackPtr] > tcur);
			}
		}
		// Moeller-Trumbore ray/triangle intersection algorithm for four triangles
		const BVHTri4Leaf* leaf = (BVHTri4Leaf*)(bvh4Data + (nodeIdx & 0x1fffffff));
	#ifdef BVH_USEAVX2
		const __m128 hx4 = _mm_fmsub_ps( dy4, _mm_load_ps( leaf->e2z ), _mm_mul_ps( dz4, _mm_load_ps( leaf->e2y ) ) );
		const __m128 hy4 = _mm_fmsub_ps( dz4, _mm_load_ps( leaf->e2x ), _mm_mul_ps( dx4, _mm_load_ps( leaf->e2z ) ) );
		const __m128 hz4 = _mm_fmsub_ps( dx4, _mm_load_ps( leaf->e2y ), _mm_mul_ps( dy4, _mm_load_ps( leaf->e2x ) ) );
		const __m128 sx4 = _mm_sub_ps( ox4, _mm_load_ps( leaf->v0x ) ), sy4 = _mm_sub_ps( oy4, _mm_load_ps( leaf->v0y ) ), sz4 = _mm_sub_ps( oz4, _mm_load_ps( leaf->v0z ) );
		const __m128 det4 = _mm_fmadd_ps( _mm_load_ps( leaf->e1z ), hz4, _mm_fmadd_ps( _mm_load_ps( leaf->e1x ), hx4, _mm_mul_ps( _mm_load_ps( leaf->e1y ), hy4 ) ) );
		const __m128 qz4 = _mm_fmsub_ps( sx4, _mm_load_ps( leaf->e1y ), _mm_mul_ps( sy4, _mm_load_ps( leaf->e1x ) ) );
		const __m128 qx4 = _mm_fmsub_ps( sy4, _mm_load_ps( leaf->e1z ), _mm_mul_ps( sz4, _mm_load_ps( leaf->e1y ) ) );
		const __m128 qy4 = _mm_fmsub_ps( sz4, _mm_load_ps( leaf->e1x ), _mm_mul_ps( sx4, _mm_load_ps( leaf->e1z ) ) );
		const __m128 inv_det4 = _mm_div_ps( one4, det4 );
		const __m128 u4 = _mm_mul_ps( _mm_fmadd_ps( sz4, hz4, _mm_fmadd_ps( sx4, hx4, _mm_mul_ps( sy4, hy4 ) ) ), inv_det4 );
		const __m128 v4 = _mm_mul_ps( _mm_fmadd_ps( dz4, qz4, _mm_fmadd_ps( dx4, qx4, _mm_mul_ps( dy4, qy4 ) ) ), inv_det4 );
		const __m128 ta4 = _mm_mul_ps( _mm_fmadd_ps( _mm_load_ps( leaf->e2z ), qz4, _mm_fmadd_ps( _mm_load_ps( leaf->e2x ), qx4, _mm_mul_ps( _mm_load_ps( leaf->e2y ), qy4 ) ) ), inv_det4 );
	#else
		const __m128 hx4 = _mm_sub_ps( _mm_mul_ps( dy4, _mm_load_ps( leaf->e2z ) ), _mm_mul_ps( dz4, _mm_load_ps( leaf->e2y ) ) );
		const __m128 hy4 = _mm_sub_ps( _mm_mul_ps( dz4, _mm_load_ps( leaf->e2x ) ), _mm_mul_ps( dx4, _mm_load_ps( leaf->e2z ) ) );
		const __m128 hz4 = _mm_sub_ps( _mm_mul_ps( dx4, _mm_load_ps( leaf->e2y ) ), _mm_mul_ps( dy4, _mm_load_ps( leaf->e2x ) ) );
		const __m128 sx4 = _mm_sub_ps( ox4, _mm_load_ps( leaf->v0x ) ), sy4 = _mm_sub_ps( oy4, _mm_load_ps( leaf->v0y ) ), sz4 = _mm_sub_ps( oz4, _mm_load_ps( leaf->v0z ) );
		const __m128 det4 = _mm_add_ps( _mm_mul_ps( _mm_load_ps( leaf->e1z ), hz4 ), _mm_add_ps( _mm_mul_ps( _mm_load_ps( leaf->e1x ), hx4 ), _mm_mul_ps( _mm_load_ps( leaf->e1y ), hy4 ) ) );
		const __m128 qz4 = _mm_sub_ps( _mm_mul_ps( sx4, _mm_load_ps( leaf->e1y ) ), _mm_mul_ps( sy4, _mm_load_ps( leaf->e1x ) ) );
		const __m128 qx4 = _mm_sub_ps( _mm_mul_ps( sy4, _mm_load_ps( leaf->e1z ) ), _mm_mul_ps( sz4, _mm_load_ps( leaf->e1y ) ) );
		const __m128 qy4 = _mm_sub_ps( _mm_mul_ps( sz4, _mm_load_ps( leaf->e1x ) ), _mm_mul_ps( sx4, _mm_load_ps( leaf->e1z ) ) );
		const __m128 inv_det4 = _mm_div_ps( one4, det4 );
		const __m128 u4 = _mm_mul_ps( _mm_add_ps( _mm_mul_ps( sz4, hz4 ), _mm_add_ps( _mm_mul_ps( sx4, hx4 ), _mm_mul_ps( sy4, hy4 ) ) ), inv_det4 );
		const __m128 v4 = _mm_mul_ps( _mm_add_ps( _mm_mul_ps( dz4, qz4 ), _mm_add_ps( _mm_mul_ps( dx4, qx4 ), _mm_mul_ps( dy4, qy4 ) ) ), inv_det4 );
		const __m128 ta4 = _mm_mul_ps( _mm_add_ps( _mm_mul_ps( _mm_load_ps( leaf->e2z ), qz4 ), _mm_add_ps( _mm_mul_ps( _mm_load_ps( leaf->e2x ), qx4 ), _mm_mul_ps( _mm_load_ps( leaf->e2y ), qy4 ) ) ), inv_det4 );
	#endif
		const __m128 mask1 = _mm_and_ps( _mm_cmpge_ps( u4, zero4 ), _mm_cmpge_ps( v4, zero4 ) );
		const __m128 mask2 = _mm_cmple_ps( _mm_add_ps( u4, v4 ), one4 );
		const __m128 mask3 = _mm_and_ps( _mm_cmplt_ps( ta4, t4 ), _mm_cmpgt_ps( ta4, zero4 ) );
		__m128 combined = _mm_and_ps( _mm_and_ps( mask1, mask2 ), mask3 );
		uint32_t imask = _mm_movemask_ps( combined );
		// evaluate opacity map, if present (SSE version).
		if (opmap) if (imask)
		{
			const __m128 fN4 = _mm_set1_ps( (float)opmapN );
			const __m128i row4 = _mm_cvttps_epi32( _mm_mul_ps( _mm_add_ps( u4, v4 ), fN4 ) );
			const __m128i dia4 = _mm_cvttps_epi32( _mm_mul_ps( _mm_sub_ps( one4, u4 ), fN4 ) );
			const __m128i v0 = _mm_mullo_epi32( row4, row4 );
			const __m128i v1 = _mm_cvttps_epi32( _mm_mul_ps( v4, fN4 ) );
			const __m128i v2 = _mm_sub_epi32( dia4, _mm_sub_epi32( _mm_set1_epi32( opmapN - 1 ), row4 ) );
			uint32_t idx[4], omask[4] = { 0, 0, 0, 0 };
			tinybvh_store4i( idx, _mm_add_epi32( _mm_add_epi32( v0, v1 ), v2 ) );
			// proceed with scalar code for gather operation - TODO: better approach?
			for (int i = 0; i < 4; i++) if (imask & (1 << i))
			{
				uint32_t* om = opmap + leaf->primIdx[i] * ((opmapN * opmapN + 31) >> 5);
				if (om[idx[i] >> 5] & (1 << (idx[i] & 31))) omask[i] = 0xffffffff;
			}
			// combine
			combined = _mm_and_ps( combined, tinybvh_load4( omask ) );
			imask = _mm_movemask_ps( combined );
		}
		if (imask)
		{
			const __m128 dist4 = _mm_blendv_ps( inf4, ta4, combined );
			// compute broadcasted horizontal minimum of dist4
			const __m128 a = _mm_min_ps( dist4, _mm_shuffle_ps( dist4, dist4, _MM_SHUFFLE( 2, 1, 0, 3 ) ) );
			const __m128 c = _mm_min_ps( a, _mm_shuffle_ps( a, a, _MM_SHUFFLE( 1, 0, 3, 2 ) ) );
			const uint32_t lane = __bfind( _mm_movemask_ps( _mm_cmpeq_ps( c, dist4 ) ) );
			// update hit record
			const __m128 _d4 = dist4;
			const float t = tinybvh_getlane_f( &_d4, lane );
			const __m128 _u4 = u4, _v4 = v4;
			ray.hit.t = t, ray.hit.u = tinybvh_getlane_f( &_u4, lane ), ray.hit.v = tinybvh_getlane_f( &_v4, lane );
		#if INST_IDX_BITS == 32
			ray.hit.prim = leaf->primIdx[lane], ray.hit.inst = ray.instIdx;
		#else
			ray.hit.prim = leaf->primIdx[lane] + ray.instIdx;
		#endif
			t4 = _mm_set1_ps( t ), tcur = t;
		}
		// skip entries behind the current hit
		do { if (!stackPtr) goto the_end; nodeIdx = nodeStack[--stackPtr]; } while (distStack[stackPtr] > tcur);
	}
the_end:
#ifdef _DEBUG
	return steps;
#else
	return 0;
#endif
}

#undef SSE_PUSH

template <> template <bool posX, bool posY, bool posZ> bool impl::BVH4_CPU<float, uint32_t>::IsOccludedOctant( const Ray& ray ) const
{
	ALIGNED( 64 ) uint32_t nodeStack[TINYBVH_STACK_SIZE * 2 /* wide trees push more nodes per step */];
	int32_t stackPtr = 0;
	uint32_t nodeIdx = 0;
	const __m128 t4 = _mm_set1_ps( ray.hit.t );
	const __m128 rx4 = _mm_set1_ps( ray.O.x * ray.rD.x ), rdx4 = _mm_set1_ps( ray.rD.x );
	const __m128 ry4 = _mm_set1_ps( ray.O.y * ray.rD.y ), rdy4 = _mm_set1_ps( ray.rD.y );
	const __m128 rz4 = _mm_set1_ps( ray.O.z * ray.rD.z ), rdz4 = _mm_set1_ps( ray.rD.z );
	const __m128 ox4 = _mm_set1_ps( ray.O.x ), oy4 = _mm_set1_ps( ray.O.y ), oz4 = _mm_set1_ps( ray.O.z );
	const __m128 dx4 = _mm_set1_ps( ray.D.x ), dy4 = _mm_set1_ps( ray.D.y ), dz4 = _mm_set1_ps( ray.D.z );
	const __m128 one4 = _mm_set1_ps( 1.0f ), zero4 = _mm_setzero_ps();
	while (1)
	{
		while (!(nodeIdx & LEAF_BIT))
		{
			const BVHNode* n = (BVHNode*)(bvh4Data + nodeIdx);
			const uint32_t* child = n->child;
		#ifdef BVH_USEAVX2
			const __m128 tx1 = _mm_fmsub_ps( _mm_load_ps( posX ? n->xmin : n->xmax ), rdx4, rx4 );
			const __m128 ty1 = _mm_fmsub_ps( _mm_load_ps( posY ? n->ymin : n->ymax ), rdy4, ry4 );
			const __m128 tz1 = _mm_fmsub_ps( _mm_load_ps( posZ ? n->zmin : n->zmax ), rdz4, rz4 );
			const __m128 tx2 = _mm_fmsub_ps( _mm_load_ps( posX ? n->xmax : n->xmin ), rdx4, rx4 );
			const __m128 ty2 = _mm_fmsub_ps( _mm_load_ps( posY ? n->ymax : n->ymin ), rdy4, ry4 );
			const __m128 tz2 = _mm_fmsub_ps( _mm_load_ps( posZ ? n->zmax : n->zmin ), rdz4, rz4 );
		#else
			const __m128 tx1 = _mm_sub_ps( _mm_mul_ps( _mm_load_ps( posX ? n->xmin : n->xmax ), rdx4 ), rx4 );
			const __m128 ty1 = _mm_sub_ps( _mm_mul_ps( _mm_load_ps( posY ? n->ymin : n->ymax ), rdy4 ), ry4 );
			const __m128 tz1 = _mm_sub_ps( _mm_mul_ps( _mm_load_ps( posZ ? n->zmin : n->zmax ), rdz4 ), rz4 );
			const __m128 tx2 = _mm_sub_ps( _mm_mul_ps( _mm_load_ps( posX ? n->xmax : n->xmin ), rdx4 ), rx4 );
			const __m128 ty2 = _mm_sub_ps( _mm_mul_ps( _mm_load_ps( posY ? n->ymax : n->ymin ), rdy4 ), ry4 );
			const __m128 tz2 = _mm_sub_ps( _mm_mul_ps( _mm_load_ps( posZ ? n->zmax : n->zmin ), rdz4 ), rz4 );
		#endif
			const __m128 tmin = _mm_max_ps( _mm_max_ps( zero4, tx1 ), _mm_max_ps( ty1, tz1 ) );
			const __m128 tmax = _mm_min_ps( _mm_min_ps( tx2, t4 ), _mm_min_ps( ty2, tz2 ) );
			// slab mask in lane order, the children are visited in any order
			const uint32_t m = _mm_movemask_ps( _mm_cmple_ps( tmin, tmax ) );
			const uint32_t c0 = child[0], c1 = child[1], c2 = child[2], c3 = child[3];
			if (SSE_HIT( 0 ))
			{
				if (SSE_HIT( 3 )) nodeStack[stackPtr++] = c3;
				if (SSE_HIT( 2 )) nodeStack[stackPtr++] = c2;
				if (SSE_HIT( 1 )) nodeStack[stackPtr++] = c1;
				nodeIdx = c0;
			}
			else if (SSE_HIT( 1 ))
			{
				if (SSE_HIT( 3 )) nodeStack[stackPtr++] = c3;
				if (SSE_HIT( 2 )) nodeStack[stackPtr++] = c2;
				nodeIdx = c1;
			}
			else if (SSE_HIT( 2 ))
			{
				if (SSE_HIT( 3 )) nodeStack[stackPtr++] = c3;
				nodeIdx = c2;
			}
			else if (SSE_HIT( 3 )) nodeIdx = c3; else
			{
				if (!stackPtr) return false;
				nodeIdx = nodeStack[--stackPtr];
			}
		}
		// Moeller-Trumbore ray/triangle intersection algorithm for four triangles
		const BVHTri4Leaf* leaf = (BVHTri4Leaf*)(bvh4Data + (nodeIdx & 0x1fffffff));
	#ifdef BVH_USEAVX2
		const __m128 hx4 = _mm_fmsub_ps( dy4, _mm_load_ps( leaf->e2z ), _mm_mul_ps( dz4, _mm_load_ps( leaf->e2y ) ) );
		const __m128 hy4 = _mm_fmsub_ps( dz4, _mm_load_ps( leaf->e2x ), _mm_mul_ps( dx4, _mm_load_ps( leaf->e2z ) ) );
		const __m128 hz4 = _mm_fmsub_ps( dx4, _mm_load_ps( leaf->e2y ), _mm_mul_ps( dy4, _mm_load_ps( leaf->e2x ) ) );
		const __m128 sx4 = _mm_sub_ps( ox4, _mm_load_ps( leaf->v0x ) ), sy4 = _mm_sub_ps( oy4, _mm_load_ps( leaf->v0y ) ), sz4 = _mm_sub_ps( oz4, _mm_load_ps( leaf->v0z ) );
		const __m128 det4 = _mm_fmadd_ps( _mm_load_ps( leaf->e1z ), hz4, _mm_fmadd_ps( _mm_load_ps( leaf->e1x ), hx4, _mm_mul_ps( _mm_load_ps( leaf->e1y ), hy4 ) ) );
		const __m128 qz4 = _mm_fmsub_ps( sx4, _mm_load_ps( leaf->e1y ), _mm_mul_ps( sy4, _mm_load_ps( leaf->e1x ) ) );
		const __m128 qx4 = _mm_fmsub_ps( sy4, _mm_load_ps( leaf->e1z ), _mm_mul_ps( sz4, _mm_load_ps( leaf->e1y ) ) );
		const __m128 qy4 = _mm_fmsub_ps( sz4, _mm_load_ps( leaf->e1x ), _mm_mul_ps( sx4, _mm_load_ps( leaf->e1z ) ) );
		const __m128 inv_det4 = _mm_div_ps( one4, det4 );
		const __m128 u4 = _mm_mul_ps( _mm_fmadd_ps( sz4, hz4, _mm_fmadd_ps( sx4, hx4, _mm_mul_ps( sy4, hy4 ) ) ), inv_det4 );
		const __m128 v4 = _mm_mul_ps( _mm_fmadd_ps( dz4, qz4, _mm_fmadd_ps( dx4, qx4, _mm_mul_ps( dy4, qy4 ) ) ), inv_det4 );
		const __m128 ta4 = _mm_mul_ps( _mm_fmadd_ps( _mm_load_ps( leaf->e2z ), qz4, _mm_fmadd_ps( _mm_load_ps( leaf->e2x ), qx4, _mm_mul_ps( _mm_load_ps( leaf->e2y ), qy4 ) ) ), inv_det4 );
	#else
		const __m128 hx4 = _mm_sub_ps( _mm_mul_ps( dy4, _mm_load_ps( leaf->e2z ) ), _mm_mul_ps( dz4, _mm_load_ps( leaf->e2y ) ) );
		const __m128 hy4 = _mm_sub_ps( _mm_mul_ps( dz4, _mm_load_ps( leaf->e2x ) ), _mm_mul_ps( dx4, _mm_load_ps( leaf->e2z ) ) );
		const __m128 hz4 = _mm_sub_ps( _mm_mul_ps( dx4, _mm_load_ps( leaf->e2y ) ), _mm_mul_ps( dy4, _mm_load_ps( leaf->e2x ) ) );
		const __m128 sx4 = _mm_sub_ps( ox4, _mm_load_ps( leaf->v0x ) ), sy4 = _mm_sub_ps( oy4, _mm_load_ps( leaf->v0y ) ), sz4 = _mm_sub_ps( oz4, _mm_load_ps( leaf->v0z ) );
		const __m128 det4 = _mm_add_ps( _mm_mul_ps( _mm_load_ps( leaf->e1z ), hz4 ), _mm_add_ps( _mm_mul_ps( _mm_load_ps( leaf->e1x ), hx4 ), _mm_mul_ps( _mm_load_ps( leaf->e1y ), hy4 ) ) );
		const __m128 qz4 = _mm_sub_ps( _mm_mul_ps( sx4, _mm_load_ps( leaf->e1y ) ), _mm_mul_ps( sy4, _mm_load_ps( leaf->e1x ) ) );
		const __m128 qx4 = _mm_sub_ps( _mm_mul_ps( sy4, _mm_load_ps( leaf->e1z ) ), _mm_mul_ps( sz4, _mm_load_ps( leaf->e1y ) ) );
		const __m128 qy4 = _mm_sub_ps( _mm_mul_ps( sz4, _mm_load_ps( leaf->e1x ) ), _mm_mul_ps( sx4, _mm_load_ps( leaf->e1z ) ) );
		const __m128 inv_det4 = _mm_div_ps( one4, det4 );
		const __m128 u4 = _mm_mul_ps( _mm_add_ps( _mm_mul_ps( sz4, hz4 ), _mm_add_ps( _mm_mul_ps( sx4, hx4 ), _mm_mul_ps( sy4, hy4 ) ) ), inv_det4 );
		const __m128 v4 = _mm_mul_ps( _mm_add_ps( _mm_mul_ps( dz4, qz4 ), _mm_add_ps( _mm_mul_ps( dx4, qx4 ), _mm_mul_ps( dy4, qy4 ) ) ), inv_det4 );
		const __m128 ta4 = _mm_mul_ps( _mm_add_ps( _mm_mul_ps( _mm_load_ps( leaf->e2z ), qz4 ), _mm_add_ps( _mm_mul_ps( _mm_load_ps( leaf->e2x ), qx4 ), _mm_mul_ps( _mm_load_ps( leaf->e2y ), qy4 ) ) ), inv_det4 );
	#endif
		const __m128 mask1 = _mm_and_ps( _mm_cmpge_ps( u4, zero4 ), _mm_cmpge_ps( v4, zero4 ) );
		const __m128 mask2 = _mm_cmple_ps( _mm_add_ps( u4, v4 ), one4 );
		const __m128 mask3 = _mm_and_ps( _mm_cmplt_ps( ta4, t4 ), _mm_cmpgt_ps( ta4, zero4 ) );
		const __m128 combined = _mm_and_ps( _mm_and_ps( mask1, mask2 ), mask3 );
		const uint32_t imask = _mm_movemask_ps( combined );
		if (imask)
		{
			if (!opmap) return true;
			// evaluate opacity map, SSE version.
			const __m128 fN4 = _mm_set1_ps( (float)opmapN );
			const __m128i row4 = _mm_cvttps_epi32( _mm_mul_ps( _mm_add_ps( u4, v4 ), fN4 ) );
			const __m128i dia4 = _mm_cvttps_epi32( _mm_mul_ps( _mm_sub_ps( one4, u4 ), fN4 ) );
			const __m128i v0 = _mm_mullo_epi32( row4, row4 );
			const __m128i v1 = _mm_cvttps_epi32( _mm_mul_ps( v4, fN4 ) );
			const __m128i v2 = _mm_sub_epi32( dia4, _mm_sub_epi32( _mm_set1_epi32( opmapN - 1 ), row4 ) );
			uint32_t idx[4];
			tinybvh_store4i( idx, _mm_add_epi32( _mm_add_epi32( v0, v1 ), v2 ) );
			// proceed with scalar code for gather operation - TODO: better approach?
			for (int i = 0; i < 4; i++) if (imask & (1 << i))
			{
				uint32_t* om = opmap + leaf->primIdx[i] * ((opmapN * opmapN + 31) >> 5);
				if (om[idx[i] >> 5] & (1 << (idx[i] & 31))) return true;
			}
		}
		// we continue.
		if (!stackPtr) return false;
		nodeIdx = nodeStack[--stackPtr];
	}
}

#undef SSE_HIT

#endif // BVH_USESSE
#ifdef BVH_USEAVX

TINYBVH_FORCEINLINE __m256 bvhc_max8() { return _mm256_set1_ps( -BVH_FAR ); }
TINYBVH_FORCEINLINE __m256 bvhc_signFlip8() { return _mm256_setr_ps( -0.0f, -0.0f, -0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f ); }

// Fast threaded AVX binned-SAH-builder.
// This code produces BVHs nearly identical to reference, but much faster.
// The code relies on the availability of AVX instructions. AVX2 is not needed.
TINYBVH_FORCEINLINE float halfArea( const __m256& a /* a contains aabb itself, with min.xyz negated */ )
{
#ifndef _MSC_VER
	// g++ doesn't seem to like the faster construct
	float ex = tinybvh_getlane_f( &a, 4 ) + tinybvh_getlane_f( &a, 0 );
	float ey = tinybvh_getlane_f( &a, 5 ) + tinybvh_getlane_f( &a, 1 );
	float ez = tinybvh_getlane_f( &a, 6 ) + tinybvh_getlane_f( &a, 2 );
	return ex * ey + ey * ez + ez * ex;
#else
	const __m128 q = _mm256_castps256_ps128( _mm256_add_ps( _mm256_permute2f128_ps( a, a, 5 ), a ) );
	const __m128 v = _mm_mul_ps( q, _mm_shuffle_ps( q, q, 9 ) );
	return LANE( v, 0 ) + LANE( v, 1 ) + LANE( v, 2 );
#endif
}

#define PROCESS_PLANE( a, pos, ANLR, lN, rN, lb, rb ) if (lN != 0 && rN != 0) { \
	ANLR = halfArea( lb ) * (float)lN + halfArea( rb ) * (float)rN; if (ANLR < splitCost) \
	splitCost = ANLR, bestAxis = a, bestPos = pos, bestLBox = lb, bestRBox = rb; }
#if defined _MSC_VER
#pragma warning ( push )
#pragma warning( disable:4701 ) // "potentially uninitialized local variable 'bestLBox' used"
#pragma warning (disable:4324) // "lambda structure was padded due to alignment specifier"
#elif defined __GNUC__ && !defined __clang__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif

template <> void impl::BVH<float, uint32_t>::BuildAVX( const bvhvec4slice& v, const uint32_t* i, const uint32_t p )
{
	PrepareSIMDBuild( v, i, p );
	BuildSIMDSubtree( 0u, 0u );
	BuildSIMDFinalize();
}

// bin one slice of a node's fragment range; scheduled via the parallel_for hook.
static constexpr uint32_t AVXCOUNTSTRIDE = 32; // 32 * 4 bytes = 128 bytes.
struct ALIGNED( 64 ) SliceBounds { float bmin[4], bmax[4]; char pad[32]; };
struct BuildAVXFragSliceArgs
{
	BVH* bvh;
	const uint32_t triCount, sliceSize, slices, * indices, stride4;
	const int8_t* vertData;
	SliceBounds* slice;
	void* frags;
};
static void BuildAVXFragSlice( uint32_t i, void* payload )
{
	BuildAVXFragSliceArgs* a = (BuildAVXFragSliceArgs*)payload;
	const uint32_t first = a->sliceSize * i, last = i == (a->slices - 1) ? a->triCount : (first + a->sliceSize);
	a->bvh->PrepareSIMDBuildFragSlice( first, last, a->indices, a->vertData, a->stride4, a->frags, a->slice[i].bmin, a->slice[i].bmax );
}
template <> void impl::BVH<float, uint32_t>::PrepareSIMDBuildFragSlice( const uint32_t first, const uint32_t last,
	const uint32_t* indices, const int8_t* vertData, const uint32_t stride4, void* frags, float* rootMin, float* rootMax )
{
	// 'frags' is really a Fragment*; it is passed as void* so that the threading hook does not need to see the nested type.
	Fragment* frag = (Fragment*)frags;
	__m128 rmin = _mm_set1_ps( BVH_FAR ), rmax = _mm_set1_ps( -BVH_FAR );
	if (indices) for (uint32_t i = first; i < last; i++)
	{
		const uint32_t i0 = indices[i * 3], i1 = indices[i * 3 + 1], i2 = indices[i * 3 + 2];
		const __m128 v0 = tinybvh_load4( vertData + (size_t)(i0 * stride4) * 16 );
		const __m128 v1 = tinybvh_load4( vertData + (size_t)(i1 * stride4) * 16 );
		const __m128 v2 = tinybvh_load4( vertData + (size_t)(i2 * stride4) * 16 );
		const __m128 t1 = _mm_min_ps( _mm_min_ps( v0, v1 ), v2 ), t2 = _mm_max_ps( _mm_max_ps( v0, v1 ), v2 );
		tinybvh_store4( &frag[i].bmin, t1 ), tinybvh_store4( &frag[i].bmax, t2 );
		rmin = _mm_min_ps( rmin, t1 ), rmax = _mm_max_ps( rmax, t2 );
		primIdx[i] = i;
	}
	else for (uint32_t i = first; i < last; i++)
	{
		const __m128 v0 = tinybvh_load4( vertData + (size_t)((i * 3) * stride4) * 16 );
		const __m128 v1 = tinybvh_load4( vertData + (size_t)((i * 3 + 1) * stride4) * 16 );
		const __m128 v2 = tinybvh_load4( vertData + (size_t)((i * 3 + 2) * stride4) * 16 );
		const __m128 t1 = _mm_min_ps( _mm_min_ps( v0, v1 ), v2 ), t2 = _mm_max_ps( _mm_max_ps( v0, v1 ), v2 );
		tinybvh_store4( &frag[i].bmin, t1 ), tinybvh_store4( &frag[i].bmax, t2 );
		rmin = _mm_min_ps( rmin, t1 ), rmax = _mm_max_ps( rmax, t2 );
		primIdx[i] = i;
	}
	tinybvh_store4( rootMin, rmin ), tinybvh_store4( rootMax, rmax ); // slices are cache line separated; no false sharing.
}

template <> void impl::BVH<float, uint32_t>::PrepareSIMDBuild( const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t prims )
{
	BVH_FATAL_ERROR_IF( vertices.count == 0, "BVH::PrepareSIMDBuild( .. ), primCount == 0." );
	BVH_FATAL_ERROR_IF( vertices.stride & 15, "BVH::PrepareSIMDBuild( .. ), stride must be multiple of 16." );
	// some constants
	static const __m128 min4 = _mm_set1_ps( BVH_FAR ), max4 = _mm_set1_ps( -BVH_FAR );
	// reset node pool
	const uint32_t primCount = prims > 0 ? prims : vertices.count / 3;
	const uint32_t splitBudget = settings.usePresplitting ? ((int)(primCount * settings.presplitFactor)) : 0;
	const uint32_t spaceNeeded = (primCount + splitBudget) * 2; // upper limit
	// a rebuild is unsafe once the tree has been converted, whether or not we reallocate.
	BVH_FATAL_ERROR_IF( allocatedNodes > 0 && !rebuildable, "BVH::PrepareSIMDBuild( .. ), bvh not rebuildable." );
	if (allocatedNodes < spaceNeeded)
	{
		AlignedFree( bvhNode );
		AlignedFree( primIdx );
		AlignedFree( fragment );
		bvhNode = (BVHNode*)AlignedAlloc( spaceNeeded * sizeof( BVHNode ) );
		allocatedNodes = spaceNeeded;
		primIdx = (uint32_t*)AlignedAlloc( (primCount + splitBudget) * sizeof( uint32_t ) );
		memset( &bvhNode[1], 0, sizeof( BVHNode ) ); // avoid crash in refit.
		fragment = (Fragment*)AlignedAlloc( (primCount + splitBudget) * sizeof( Fragment ) );
	}
	triCount = primCount;
	verts = vertices; // note: we're not copying this data; don't delete.
	vertIdx = (uint32_t*)indices;
	const int8_t* vertData = verts.data;
	// prepare threading
	threadedBuild = false;
#ifdef ENABLE_THREADED_BUILDS
	if (triCount >= MT_BUILD_THRESHOLD && context.spawn && context.barrier)
		threadedBuild = true, atomicNewNodePtr = ContextNew<std::atomic<uint32_t>>( 2u );
#endif
	// initialize fragments
	__m128 rootMin = min4, rootMax = max4;
	uint32_t stride4 = verts.stride / 16;
	BVH_FATAL_ERROR_IF( primCount == 0, "BVH::PrepareSIMDBuild( .. ), primCount == 0." );
	// build the BVH over indexed triangles
	if (threadedBuild)
	{
		constexpr int slices = 4;
		ALIGNED( 64 ) SliceBounds slice[slices]; // one cache line per slice; no false sharing.
		BuildAVXFragSliceArgs args = { this, triCount, triCount / slices, slices, indices, stride4, vertData, slice, fragment };
		tinybvh_parallel_for( context, slices, &BuildAVXFragSlice, &args );
		rootMin = tinybvh_load4( slice[0].bmin ), rootMax = tinybvh_load4( slice[0].bmax );
		for (int i = 1; i < slices; i++)
			rootMin = _mm_min_ps( rootMin, tinybvh_load4( slice[i].bmin ) ), rootMax = _mm_max_ps( rootMax, tinybvh_load4( slice[i].bmax ) );
	}
	else
	{
		ALIGNED( 16 ) float rmin[4], rmax[4];
		PrepareSIMDBuildFragSlice( 0, triCount, indices, vertData, stride4, (void*)fragment, rmin, rmax );
		rootMin = tinybvh_load4( rmin ), rootMax = tinybvh_load4( rmax );
	}
	BVHNode& root = bvhNode[0];
	root.aabbMin = tinybvh_bitcast<bvhvec4>( rootMin ), root.aabbMax = tinybvh_bitcast<bvhvec4>( rootMax );
	// presplitting
	uint32_t fragCount = primCount;
	if (settings.usePresplitting)
	{
		for (uint32_t i = 0; i < primCount; i++) fragment[i].primIdx = i, fragment[i].clipped = 0;
		fragCount = Presplit();
	}
	// finalize root node
	root.leftFirst = 0, root.triCount = idxCount = triCount = fragCount;
	// reset node pool
	newNodePtr = 2, bvh_over_indices = indices != nullptr;
	// all set; actual build happens in BVH::BuildSIMDSubtree.
}

template <> void impl::BVH<float, uint32_t>::BuildSIMDBinTask( const uint32_t first, const uint32_t last, void* binboxes,
	uint32_t* count, const float* nmin, const float* rpd )
{
	__m256* binbox = (__m256*)binboxes;
	const __m128 nmin4 = tinybvh_load4( nmin ), rpd4 = tinybvh_load4( rpd );
	// A Fragment is 32 bytes and holds bmin/primIdx followed by bmax/clipped, so
	// it can be read as a single 8-wide vector, or as two 4-wide bounds.
	memset( count, 0, 3 * AVXBINS * 4 ); // exactly 96 bytes
	for (uint32_t i = 0; i < 3 * AVXBINS; i++) binbox[i] = bvhc_max8();
	if (first >= last) return; // empty slice; 'last - 1' below would wrap.
	uint32_t fi = primIdx[first];
	__m256 r0, r1, r2, f = _mm256_xor_ps( tinybvh_load8( fragment + fi ), bvhc_signFlip8() );
	const __m128i zero4i = _mm_setzero_si128();
	__m128i bc4 = _mm_max_epi32( _mm_cvttps_epi32( _mm_mul_ps( _mm_sub_ps( _mm_add_ps(
		tinybvh_load4( &fragment[fi].bmax ), tinybvh_load4( &fragment[fi].bmin ) ), nmin4 ), rpd4 ) ), zero4i );
	uint32_t i0 = TINYBVH_LANE0( bc4 ), i1 = TINYBVH_LANE1( bc4 ), i2 = TINYBVH_LANE2( bc4 ), * ti = primIdx + first + 1;
	for (uint32_t i = first; i < last - 1; i++)
	{
		uint32_t fid = *ti++;
	#if defined __GNUC__ || _MSC_VER < 1920
		if (fid >= triCount) fid = triCount - 1; // never happens but g++ *and* vs2017 need this to not crash...
	#endif
		const __m256 b0 = binbox[i0], b1 = binbox[AVXBINS + i1], b2 = binbox[2 * AVXBINS + i2];
		const __m128 frmin = tinybvh_load4( &fragment[fid].bmin ), frmax = tinybvh_load4( &fragment[fid].bmax );
		r0 = _mm256_max_ps( b0, f ), r1 = _mm256_max_ps( b1, f ), r2 = _mm256_max_ps( b2, f );
		bc4 = _mm_max_epi32( _mm_cvttps_epi32( _mm_mul_ps( _mm_sub_ps( _mm_add_ps( frmax, frmin ), nmin4 ), rpd4 ) ), zero4i );
		f = _mm256_xor_ps( tinybvh_load8( fragment + fid ), bvhc_signFlip8() );
		count[i0]++, count[AVXBINS + i1]++, count[AVXBINS * 2 + i2]++;
		binbox[i0] = r0, i0 = TINYBVH_LANE0( bc4 );
		binbox[AVXBINS + i1] = r1, i1 = TINYBVH_LANE1( bc4 );
		binbox[2 * AVXBINS + i2] = r2, i2 = TINYBVH_LANE2( bc4 );
	}
	// final business for final fragment
	const __m256 b0 = binbox[i0], b1 = binbox[AVXBINS + i1], b2 = binbox[2 * AVXBINS + i2];
	count[i0]++, count[AVXBINS + i1]++, count[AVXBINS * 2 + i2]++;
	r0 = _mm256_max_ps( b0, f ), r1 = _mm256_max_ps( b1, f ), r2 = _mm256_max_ps( b2, f );
	binbox[i0] = r0, binbox[AVXBINS + i1] = r1, binbox[2 * AVXBINS + i2] = r2;
}

// Helper function to build a subtree via the thread pool
static void BVHBuildAVXSubtree( void* payload )
{
	impl::BVHBuildSubtreeArgs<float, uint32_t>* a = (impl::BVHBuildSubtreeArgs<float, uint32_t>*)payload;
	a->bvh->BuildSIMDSubtree( a->node, a->depth );
}
// bin one slice of a node's fragment range; scheduled via the parallel_for hook.
struct BVHBuildAVXBinSliceArgs
{
	BVH* bvh;
	uint32_t leftFirst, triCount, sliceSize, slices;
	__m256* slicebinbox;				// base of slices x (3*AVXBINS) bin boxes
	uint32_t* slicecount;				// base of slices x AVXCOUNTSTRIDE counts
	__m128 nmin4, rpd4;
};
static void BVHBuildAVXBinSlice( uint32_t i, void* payload )
{
	BVHBuildAVXBinSliceArgs* a = (BVHBuildAVXBinSliceArgs*)payload;
	const uint32_t first = a->leftFirst + a->sliceSize * i;
	const uint32_t last = i == (a->slices - 1) ? (a->leftFirst + a->triCount) : (first + a->sliceSize);
	a->bvh->BuildSIMDBinTask( first, last, a->slicebinbox + i * 3 * AVXBINS,
		a->slicecount + i * AVXCOUNTSTRIDE, (const float*)&a->nmin4, (const float*)&a->rpd4 );
}
template <> void impl::BVH<float, uint32_t>::BuildSIMDSubtree( uint32_t nodeIdx, uint32_t depth )
{
	// aligned data
	constexpr uint32_t maxSlices = 24;
	const uint32_t slices = maxSlices - 2 * depth;
	ALIGNED( 64 ) __m256 slicebinbox[maxSlices][3 * AVXBINS];
	ALIGNED( 64 ) uint32_t slicecount[maxSlices][AVXCOUNTSTRIDE]; // padded: see AVXCOUNTSTRIDE
	ALIGNED( 64 ) __m256 bestLBox, bestRBox;			// 64 bytes
	__m256* binbox = slicebinbox[0];					// slot 0 doubles as the reduce target
	uint32_t* count = slicecount[0];
	// subdivide recursively
	ALIGNED( 64 ) uint32_t task[512], taskCount = 0;
	BVHNode& root = bvhNode[0];
	const bvhvec3 minDim = (root.aabbMax - root.aabbMin) * 1e-7f;
	while (1)
	{
		while (1)
		{
			BVHNode& node = bvhNode[nodeIdx];
			const float SAV = node.SurfaceArea();
			if (SAV == 0) break; // can't split an infinitely small node.
			const __m128 nodeMin4 = tinybvh_load4( &bvhNode[nodeIdx].aabbMin );
			const __m128 nodeMax4 = tinybvh_load4( &bvhNode[nodeIdx].aabbMax );
			// find optimal object split
			const __m128 d4 = _mm_blendv_ps( bvhc_min1(), _mm_sub_ps( nodeMax4, nodeMin4 ), bvhc_mask3() );
			const __m128 nmin4 = _mm_add_ps( nodeMin4, nodeMin4 );
			const __m128 rpd4 = _mm_and_ps( _mm_div_ps( bvhc_binmul3(), d4 ), _mm_cmpneq_ps( d4, _mm_setzero_ps() ) );
			// implementation of Section 4.1 of "Parallel Spatial Splits in Bounding Volume Hierarchies":
			// main loop operates on two fragments to minimize dependencies and maximize ILP.
			if (threadedBuild && node.triCount > MT_BUILD_THRESHOLD)
			{
				const uint32_t sliceSize = node.triCount / slices;
				BVHBuildAVXBinSliceArgs args = { this, node.leftFirst, node.triCount, sliceSize, slices,
					slicebinbox[0], slicecount[0], nmin4, rpd4 };
				tinybvh_parallel_for( context, slices, &BVHBuildAVXBinSlice, &args );
				// combine results from slices; slice-major, so each slice is a linear sweep.
				for (uint32_t slice = 1; slice < slices; slice++)
				{
					const __m256* sbb = slicebinbox[slice];
					const uint32_t* sc = slicecount[slice];
					for (uint32_t ai = 0; ai < 3 * AVXBINS; ai++)
						count[ai] += sc[ai], binbox[ai] = _mm256_max_ps( binbox[ai], sbb[ai] );
				}
			}
			else
				// binning runs serially; threading comes from the subtree spawns below.
				BuildSIMDBinTask( node.leftFirst, node.leftFirst + node.triCount, binbox, count, (const float*)&nmin4, (const float*)&rpd4 );
			// calculate per-split totals
			float splitCost = BVH_FAR;
			const float rSAV = 1.0f / SAV;
			uint32_t bestAxis = 0, bestPos = 0;
			const __m256* bb = binbox;
			for (int32_t a = 0; a < 3; a++, bb += AVXBINS) if ((node.aabbMax[a] - node.aabbMin[a]) > minDim[a])
			{
				// hardcoded bin processing for AVXBINS == 8
				assert( AVXBINS == 8 );
				const uint32_t* cnt = count + a * AVXBINS;
				const uint32_t lN0 = cnt[0], rN0 = cnt[7];
				const __m256 lb0 = bb[0], rb0 = bb[7];
				const uint32_t lN1 = lN0 + cnt[1], rN1 = rN0 + cnt[6], lN2 = lN1 + cnt[2];
				const uint32_t rN2 = rN1 + cnt[5], lN3 = lN2 + cnt[3], rN3 = rN2 + cnt[4];
				const __m256 lb1 = _mm256_max_ps( lb0, bb[1] ), rb1 = _mm256_max_ps( rb0, bb[6] );
				const __m256 lb2 = _mm256_max_ps( lb1, bb[2] ), rb2 = _mm256_max_ps( rb1, bb[5] );
				const __m256 lb3 = _mm256_max_ps( lb2, bb[3] ), rb3 = _mm256_max_ps( rb2, bb[4] );
				const uint32_t lN4 = lN3 + cnt[4], rN4 = rN3 + cnt[3], lN5 = lN4 + cnt[5];
				const uint32_t rN5 = rN4 + cnt[2], lN6 = lN5 + cnt[6], rN6 = rN5 + cnt[1];
				const __m256 lb4 = _mm256_max_ps( lb3, bb[4] ), rb4 = _mm256_max_ps( rb3, bb[3] );
				const __m256 lb5 = _mm256_max_ps( lb4, bb[5] ), rb5 = _mm256_max_ps( rb4, bb[2] );
				const __m256 lb6 = _mm256_max_ps( lb5, bb[6] ), rb6 = _mm256_max_ps( rb5, bb[1] );
				float ANLR3 = BVH_FAR; PROCESS_PLANE( a, 3, ANLR3, lN3, rN3, lb3, rb3 ); // most likely split
				float ANLR2 = BVH_FAR; PROCESS_PLANE( a, 2, ANLR2, lN2, rN4, lb2, rb4 );
				float ANLR4 = BVH_FAR; PROCESS_PLANE( a, 4, ANLR4, lN4, rN2, lb4, rb2 );
				float ANLR5 = BVH_FAR; PROCESS_PLANE( a, 5, ANLR5, lN5, rN1, lb5, rb1 );
				float ANLR1 = BVH_FAR; PROCESS_PLANE( a, 1, ANLR1, lN1, rN5, lb1, rb5 );
				float ANLR0 = BVH_FAR; PROCESS_PLANE( a, 0, ANLR0, lN0, rN6, lb0, rb6 );
				float ANLR6 = BVH_FAR; PROCESS_PLANE( a, 6, ANLR6, lN6, rN0, lb6, rb0 ); // least likely split
			}
			splitCost = c_trav + c_int * rSAV * splitCost;
			const float noSplitCost = (float)node.triCount * c_int;
			if (splitCost >= noSplitCost) break; // not splitting is better.
			const float rpd = tinybvh_getlane_f( &rpd4, bestAxis ), nmin = tinybvh_getlane_f( &nmin4, bestAxis );
			uint32_t i = node.leftFirst, j = node.leftFirst + node.triCount;
			for (uint32_t k = 0; k < node.triCount; k++)
			{
				const uint32_t fr = primIdx[i];
				const int32_t bi = tinybvh_max( 0, (int32_t)((fragment[fr].bmax[bestAxis] + fragment[fr].bmin[bestAxis] - nmin) * rpd) );
				if ((uint32_t)bi <= bestPos) i++; else
				{
					const uint32_t t = primIdx[--j];
					primIdx[j] = fr, primIdx[i] = t;
				}
			}
			// create child nodes and recurse
			const uint32_t leftCount = i - node.leftFirst, rightCount = node.triCount - leftCount;
			if (leftCount == 0 || rightCount == 0 || taskCount == BVH_NUM_ELEMS( task )) break; // should not happen.
			uint32_t n;
		#ifdef ENABLE_THREADED_BUILDS
			if (threadedBuild) n = atomicNewNodePtr->fetch_add( 2 ); else n = newNodePtr, newNodePtr += 2;
		#else
			n = newNodePtr, newNodePtr += 2;
		#endif
			tinybvh_store8( &bvhNode[n], _mm256_xor_ps( bestLBox, bvhc_signFlip8() ) );
			bvhNode[n].leftFirst = node.leftFirst, bvhNode[n].triCount = leftCount;
			node.leftFirst = n, node.triCount = 0;
			tinybvh_store8( &bvhNode[n + 1], _mm256_xor_ps( bestRBox, bvhc_signFlip8() ) );
			bvhNode[n + 1].leftFirst = i, bvhNode[n + 1].triCount = rightCount;
			const bool spawnThreads = tinybvh_max( leftCount, rightCount ) > MT_SPAWN_MIN_PRIMS && depth < MT_SPAWN_DEPTH && threadedBuild;
			if (!spawnThreads) task[taskCount++] = n + 1, nodeIdx = n; else
			{
				// spawn the larger subtree, continue with the small one; root barrier joins.
				impl::BVHBuildSubtreeArgs<float, uint32_t> a = { this, leftCount > rightCount ? n : (n + 1), depth + 1 };
				tinybvh_spawn( context, &BVHBuildAVXSubtree, &a, sizeof( a ) );
				nodeIdx = leftCount > rightCount ? (n + 1) : n;
			}
		}
		// fetch subdivision task from stack
		if (taskCount == 0) break;
		nodeIdx = task[--taskCount];
	}
}

template <> void impl::BVH<float, uint32_t>::BuildSIMDFinalize()
{
#ifdef ENABLE_THREADED_BUILDS
	if (threadedBuild)
	{
		tinybvh_barrier( context ); // wait for all spawned subtrees
		newNodePtr = atomicNewNodePtr->load();
		ContextDelete( atomicNewNodePtr );
	}
#endif
	// tree has been built.
	aabbMin = bvhNode[0].aabbMin, aabbMax = bvhNode[0].aabbMax;
	refittable = settings.usePresplitting ? false : true; // only if not using spatial splits
	may_have_holes = false; // there are no holes in the list of nodes.
	usedNodes = newNodePtr;
	if (settings.usePresplitting) // finalize indices in index array
	{
		for (uint32_t i = 0; i < triCount; i++) primIdx[i] = fragment[primIdx[i]].primIdx;
		if (settings.presplitPostPass) PresplitPostPass();
	}
}

#if defined _MSC_VER
#pragma warning ( pop ) // restore 4701
#elif defined __GNUC__ && !defined __clang__
#pragma GCC diagnostic pop // restore -Wmaybe-uninitialized
#endif

// Intersect a BVH with a ray packet, basic SSE-optimized version.
// Note: This yields +10% on 10th gen Intel CPUs, but a small loss on
// more recent hardware. This function needs a full conversion to work
// with groups of 8 rays at a time - TODO.
template <> void impl::BVH<float, uint32_t>::Intersect256RaysSSE( Ray* packet ) const
{
	// Corner rays are: 0, 51, 204 and 255
	// Construct the bounding planes, with normals pointing outwards
	bvhvec3 O = packet[0].O; // same for all rays in this case
	__m128 O4 = tinybvh_load4( &packet[0].O );
	__m128 mask4 = _mm_cmpeq_ps( _mm_setzero_ps(), _mm_set_ps( 1, 0, 0, 0 ) );
	bvhvec3 p0 = packet[0].O + packet[0].D; // top-left
	bvhvec3 p1 = packet[51].O + packet[51].D; // top-right
	bvhvec3 p2 = packet[204].O + packet[204].D; // bottom-left
	bvhvec3 p3 = packet[255].O + packet[255].D; // bottom-right
	bvhvec3 plane0 = tinybvh_normalize( tinybvh_cross( p0 - O, p0 - p2 ) ); // left plane
	bvhvec3 plane1 = tinybvh_normalize( tinybvh_cross( p3 - O, p3 - p1 ) ); // right plane
	bvhvec3 plane2 = tinybvh_normalize( tinybvh_cross( p1 - O, p1 - p0 ) ); // top plane
	bvhvec3 plane3 = tinybvh_normalize( tinybvh_cross( p2 - O, p2 - p3 ) ); // bottom plane
	int32_t sign0x = plane0.x < 0 ? 4 : 0, sign0y = plane0.y < 0 ? 5 : 1, sign0z = plane0.z < 0 ? 6 : 2;
	int32_t sign1x = plane1.x < 0 ? 4 : 0, sign1y = plane1.y < 0 ? 5 : 1, sign1z = plane1.z < 0 ? 6 : 2;
	int32_t sign2x = plane2.x < 0 ? 4 : 0, sign2y = plane2.y < 0 ? 5 : 1, sign2z = plane2.z < 0 ? 6 : 2;
	int32_t sign3x = plane3.x < 0 ? 4 : 0, sign3y = plane3.y < 0 ? 5 : 1, sign3z = plane3.z < 0 ? 6 : 2;
	float t0 = tinybvh_dot( O, plane0 ), t1 = tinybvh_dot( O, plane1 );
	float t2 = tinybvh_dot( O, plane2 ), t3 = tinybvh_dot( O, plane3 );
	// Traverse the tree with the packet
	int32_t first = 0, last = 255; // first and last active ray in the packet
	BVHNode* node = &bvhNode[0];
	ALIGNED( 64 ) uint32_t stack[2 * TINYBVH_STACK_SIZE], stackPtr = 0;
	while (1)
	{
		if (node->isLeaf())
		{
			// handle leaf node
			for (uint32_t j = 0; j < node->triCount; j++)
			{
				const uint32_t idx = primIdx[node->leftFirst + j], vid = idx * 3;
				const bvhvec3 e1 = verts[vid + 1] - verts[vid], e2 = verts[vid + 2] - verts[vid];
				const bvhvec3 s = O - bvhvec3( verts[vid] );
				for (int32_t i = first; i <= last; i++)
				{
					Ray& ray = packet[i];
					const bvhvec3 h = tinybvh_cross( ray.D, e2 );
					const float a = tinybvh_dot( e1, h );
					if (a == 0) continue; // ray parallel to triangle
					const float f = 1 / a, u = f * tinybvh_dot( s, h );
					const bvhvec3 q = tinybvh_cross( s, e1 );
					const float v = f * tinybvh_dot( ray.D, q );
					if (!(u >= 0 && v >= 0 && u + v <= 1)) continue;
					const float t = f * tinybvh_dot( e2, q );
					if (!(t > 0 && t < ray.hit.t)) continue;
					ray.hit.t = t, ray.hit.u = u, ray.hit.v = v, ray.hit.prim = idx;
				}
			}
			if (stackPtr == 0) break; else // pop
				last = stack[--stackPtr], node = bvhNode + stack[--stackPtr],
				first = last >> 8, last &= 255;
		}
		else
		{
			// fetch pointers to child nodes
			BVHNode* left = bvhNode + node->leftFirst;
			BVHNode* right = bvhNode + node->leftFirst + 1;
			bool visitLeft = true, visitRight = true;
			int32_t leftFirst = first, leftLast = last, rightFirst = first, rightLast = last;
			float distLeft, distRight;
			{
				// see if we want to intersect the left child
				const __m128 minO4 = _mm_sub_ps( tinybvh_load4( &left->aabbMin ), O4 );
				const __m128 maxO4 = _mm_sub_ps( tinybvh_load4( &left->aabbMax ), O4 );
				// 1. Early-in test: if first ray hits the node, the packet visits the node
				bool earlyHit;
				{
					const __m128 rD4 = tinybvh_load4( &packet[first].rD );
					const __m128 st1 = _mm_mul_ps( _mm_and_ps( minO4, mask4 ), rD4 );
					const __m128 st2 = _mm_mul_ps( _mm_and_ps( maxO4, mask4 ), rD4 );
					const __m128 vmax4 = _mm_max_ps( st1, st2 ), vmin4 = _mm_min_ps( st1, st2 );
					const float tmax = tinybvh_min( LANE( vmax4, 0 ), tinybvh_min( LANE( vmax4, 1 ), LANE( vmax4, 2 ) ) );
					const float tmin = tinybvh_max( LANE( vmin4, 0 ), tinybvh_max( LANE( vmin4, 1 ), LANE( vmin4, 2 ) ) );
					earlyHit = (tmax >= tmin && tmin < packet[first].hit.t && tmax >= 0);
					distLeft = tmin;
				}
				// 2. Early-out test: if the node aabb is outside the four planes, we skip the node
				if (!earlyHit)
				{
					const void* mm = left;
					bvhvec3 c0( tinybvh_getlane_f( mm, sign0x ), tinybvh_getlane_f( mm, sign0y ), tinybvh_getlane_f( mm, sign0z ) );
					bvhvec3 c1( tinybvh_getlane_f( mm, sign1x ), tinybvh_getlane_f( mm, sign1y ), tinybvh_getlane_f( mm, sign1z ) );
					bvhvec3 c2( tinybvh_getlane_f( mm, sign2x ), tinybvh_getlane_f( mm, sign2y ), tinybvh_getlane_f( mm, sign2z ) );
					bvhvec3 c3( tinybvh_getlane_f( mm, sign3x ), tinybvh_getlane_f( mm, sign3y ), tinybvh_getlane_f( mm, sign3z ) );
					if (tinybvh_dot( c0, plane0 ) > t0 || tinybvh_dot( c1, plane1 ) > t1 ||
						tinybvh_dot( c2, plane2 ) > t2 || tinybvh_dot( c3, plane3 ) > t3)
						visitLeft = false;
					else
					{
						// 3. Last resort: update first and last, stay in node if first > last
						for (; leftFirst <= leftLast; leftFirst++)
						{
							const __m128 rD4 = tinybvh_load4( &packet[leftFirst].rD );
							const __m128 st1 = _mm_mul_ps( _mm_and_ps( minO4, mask4 ), rD4 );
							const __m128 st2 = _mm_mul_ps( _mm_and_ps( maxO4, mask4 ), rD4 );
							const __m128 vmax4 = _mm_max_ps( st1, st2 ), vmin4 = _mm_min_ps( st1, st2 );
							const float tmax = tinybvh_min( LANE( vmax4, 0 ), tinybvh_min( LANE( vmax4, 1 ), LANE( vmax4, 2 ) ) );
							const float tmin = tinybvh_max( LANE( vmin4, 0 ), tinybvh_max( LANE( vmin4, 1 ), LANE( vmin4, 2 ) ) );
							if (tmax >= tmin && tmin < packet[leftFirst].hit.t && tmax >= 0) { distLeft = tmin; break; }
						}
						for (; leftLast >= leftFirst; leftLast--)
						{
							const __m128 rD4 = tinybvh_load4( &packet[leftLast].rD );
							const __m128 st1 = _mm_mul_ps( _mm_and_ps( minO4, mask4 ), rD4 );
							const __m128 st2 = _mm_mul_ps( _mm_and_ps( maxO4, mask4 ), rD4 );
							const __m128 vmax4 = _mm_max_ps( st1, st2 ), vmin4 = _mm_min_ps( st1, st2 );
							const float tmax = tinybvh_min( LANE( vmax4, 0 ), tinybvh_min( LANE( vmax4, 1 ), LANE( vmax4, 2 ) ) );
							const float tmin = tinybvh_max( LANE( vmin4, 0 ), tinybvh_max( LANE( vmin4, 1 ), LANE( vmin4, 2 ) ) );
							if (tmax >= tmin && tmin < packet[leftLast].hit.t && tmax >= 0) break;
						}
						visitLeft = leftLast >= leftFirst;
					}
				}
			}
			{
				// see if we want to intersect the right child
				const __m128 minO4 = _mm_sub_ps( tinybvh_load4( &right->aabbMin ), O4 );
				const __m128 maxO4 = _mm_sub_ps( tinybvh_load4( &right->aabbMax ), O4 );
				// 1. Early-in test: if first ray hits the node, the packet visits the node
				bool earlyHit;
				{
					const __m128 rD4 = tinybvh_load4( &packet[first].rD );
					const __m128 st1 = _mm_mul_ps( minO4, rD4 ), st2 = _mm_mul_ps( maxO4, rD4 );
					const __m128 vmax4 = _mm_max_ps( st1, st2 ), vmin4 = _mm_min_ps( st1, st2 );
					const float tmax = tinybvh_min( LANE( vmax4, 0 ), tinybvh_min( LANE( vmax4, 1 ), LANE( vmax4, 2 ) ) );
					const float tmin = tinybvh_max( LANE( vmin4, 0 ), tinybvh_max( LANE( vmin4, 1 ), LANE( vmin4, 2 ) ) );
					earlyHit = (tmax >= tmin && tmin < packet[first].hit.t && tmax >= 0);
					distRight = tmin;
				}
				// 2. Early-out test: if the node aabb is outside the four planes, we skip the node
				if (!earlyHit)
				{
					const void* mm = right;
					bvhvec3 c0( tinybvh_getlane_f( mm, sign0x ), tinybvh_getlane_f( mm, sign0y ), tinybvh_getlane_f( mm, sign0z ) );
					bvhvec3 c1( tinybvh_getlane_f( mm, sign1x ), tinybvh_getlane_f( mm, sign1y ), tinybvh_getlane_f( mm, sign1z ) );
					bvhvec3 c2( tinybvh_getlane_f( mm, sign2x ), tinybvh_getlane_f( mm, sign2y ), tinybvh_getlane_f( mm, sign2z ) );
					bvhvec3 c3( tinybvh_getlane_f( mm, sign3x ), tinybvh_getlane_f( mm, sign3y ), tinybvh_getlane_f( mm, sign3z ) );
					if (tinybvh_dot( c0, plane0 ) > t0 || tinybvh_dot( c1, plane1 ) > t1 ||
						tinybvh_dot( c2, plane2 ) > t2 || tinybvh_dot( c3, plane3 ) > t3)
						visitRight = false;
					else
					{
						// 3. Last resort: update first and last, stay in node if first > last
						for (; rightFirst <= rightLast; rightFirst++)
						{
							const __m128 rD4 = tinybvh_load4( &packet[rightFirst].rD );
							const __m128 st1 = _mm_mul_ps( _mm_and_ps( minO4, mask4 ), rD4 );
							const __m128 st2 = _mm_mul_ps( _mm_and_ps( maxO4, mask4 ), rD4 );
							const __m128 vmax4 = _mm_max_ps( st1, st2 ), vmin4 = _mm_min_ps( st1, st2 );
							const float tmax1 = tinybvh_min( LANE( vmax4, 0 ), tinybvh_min( LANE( vmax4, 1 ), LANE( vmax4, 2 ) ) );
							const float tmin1 = tinybvh_max( LANE( vmin4, 0 ), tinybvh_max( LANE( vmin4, 1 ), LANE( vmin4, 2 ) ) );
							if (tmax1 >= tmin1 && tmin1 < packet[rightFirst].hit.t && tmax1 >= 0) { distRight = tmin1; break; }
						}
						for (; rightLast >= first; rightLast--)
						{
							const __m128 rD4 = tinybvh_load4( &packet[rightLast].rD );
							const __m128 st1 = _mm_mul_ps( _mm_and_ps( minO4, mask4 ), rD4 );
							const __m128 st2 = _mm_mul_ps( _mm_and_ps( maxO4, mask4 ), rD4 );
							const __m128 vmax4 = _mm_max_ps( st1, st2 ), vmin4 = _mm_min_ps( st1, st2 );
							const float tmax1 = tinybvh_min( LANE( vmax4, 0 ), tinybvh_min( LANE( vmax4, 1 ), LANE( vmax4, 2 ) ) );
							const float tmin1 = tinybvh_max( LANE( vmin4, 0 ), tinybvh_max( LANE( vmin4, 1 ), LANE( vmin4, 2 ) ) );
							if (tmax1 >= tmin1 && tmin1 < packet[rightLast].hit.t && tmax1 >= 0) break;
						}
						visitRight = rightLast >= rightFirst;
					}
				}
			}
			// process intersection result
			if (visitLeft && visitRight)
			{
				if (distLeft < distRight)
				{
					// push right, continue with left
					stack[stackPtr++] = node->leftFirst + 1;
					stack[stackPtr++] = (rightFirst << 8) + rightLast;
					node = left, first = leftFirst, last = leftLast;
				}
				else
				{
					// push left, continue with right
					stack[stackPtr++] = node->leftFirst;
					stack[stackPtr++] = (leftFirst << 8) + leftLast;
					node = right, first = rightFirst, last = rightLast;
				}
			}
			else if (visitLeft) // continue with left
				node = left, first = leftFirst, last = leftLast;
			else if (visitRight) // continue with right
				node = right, first = rightFirst, last = rightLast;
			else if (stackPtr == 0) break; else // pop
				last = stack[--stackPtr], node = bvhNode + stack[--stackPtr],
				first = last >> 8, last &= 255;
		}
	}
}

#ifdef ENABLE_BVH_SOA

// Traverse the 'structure of arrays' BVH layout.
template <> int32_t impl::BVH_SoA<float, uint32_t>::Intersect( Ray& ray ) const
{
	VALIDATE_RAY( ray );
	BVHNode* node = &bvhNode[0], * stack[TINYBVH_STACK_SIZE];
	const bvhvec4slice& verts = bvh.verts;
	const uint32_t* primIdx = bvh.primIdx;
	uint32_t stackPtr = 0;
	float cost = 0;
	const __m128 Ox4 = _mm_set1_ps( ray.O.x ), rDx4 = _mm_set1_ps( ray.rD.x );
	const __m128 Oy4 = _mm_set1_ps( ray.O.y ), rDy4 = _mm_set1_ps( ray.rD.y );
	const __m128 Oz4 = _mm_set1_ps( ray.O.z ), rDz4 = _mm_set1_ps( ray.rD.z );
	while (1)
	{
		cost += c_trav;
		if (node->isLeaf())
		{
			if (indexedEnabled && bvh.vertIdx != 0) for (uint32_t i = 0; i < node->triCount; i++, cost += c_int)
			{
				const uint32_t pi = primIdx[node->firstTri + i];
				const uint32_t i0 = bvh.vertIdx[pi * 3], i1 = bvh.vertIdx[pi * 3 + 1], i2 = bvh.vertIdx[pi * 3 + 2];
				IntersectTri( ray, pi, verts, i0, i1, i2 );
			}
			else for (uint32_t i = 0; i < node->triCount; i++, cost += c_int)
			{
				const uint32_t pi = primIdx[node->firstTri + i];
				IntersectTri( ray, pi, verts, pi * 3, pi * 3 + 1, pi * 3 + 2 );
			}
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		__m128 x4 = _mm_mul_ps( _mm_sub_ps( _mm_load_ps( node->xxxx ), Ox4 ), rDx4 );
		__m128 y4 = _mm_mul_ps( _mm_sub_ps( _mm_load_ps( node->yyyy ), Oy4 ), rDy4 );
		__m128 z4 = _mm_mul_ps( _mm_sub_ps( _mm_load_ps( node->zzzz ), Oz4 ), rDz4 );
		// transpose
		__m128 t0 = _mm_unpacklo_ps( x4, y4 ), t2 = _mm_unpacklo_ps( z4, z4 );
		__m128 t1 = _mm_unpackhi_ps( x4, y4 ), t3 = _mm_unpackhi_ps( z4, z4 );
		const __m128 xyzw1a = _mm_shuffle_ps( t0, t2, _MM_SHUFFLE( 1, 0, 1, 0 ) );
		const __m128 xyzw2a = _mm_shuffle_ps( t0, t2, _MM_SHUFFLE( 3, 2, 3, 2 ) );
		const __m128 xyzw1b = _mm_shuffle_ps( t1, t3, _MM_SHUFFLE( 1, 0, 1, 0 ) );
		const __m128 xyzw2b = _mm_shuffle_ps( t1, t3, _MM_SHUFFLE( 3, 2, 3, 2 ) );
		// process
		const __m128 tmina4 = _mm_min_ps( xyzw1a, xyzw2a ), tmaxa4 = _mm_max_ps( xyzw1a, xyzw2a );
		const __m128 tminb4 = _mm_min_ps( xyzw1b, xyzw2b ), tmaxb4 = _mm_max_ps( xyzw1b, xyzw2b );
		// transpose back
		t0 = _mm_unpacklo_ps( tmina4, tmaxa4 ), t2 = _mm_unpacklo_ps( tminb4, tmaxb4 );
		t1 = _mm_unpackhi_ps( tmina4, tmaxa4 ), t3 = _mm_unpackhi_ps( tminb4, tmaxb4 );
		x4 = _mm_shuffle_ps( t0, t2, _MM_SHUFFLE( 1, 0, 1, 0 ) );
		y4 = _mm_shuffle_ps( t0, t2, _MM_SHUFFLE( 3, 2, 3, 2 ) );
		z4 = _mm_shuffle_ps( t1, t3, _MM_SHUFFLE( 1, 0, 1, 0 ) );
		uint32_t lidx = node->left, ridx = node->right;
		const __m128 min4 = _mm_max_ps( _mm_max_ps( _mm_max_ps( x4, y4 ), z4 ), _mm_setzero_ps() );
		const __m128 max4 = _mm_min_ps( _mm_min_ps( _mm_min_ps( x4, y4 ), z4 ), _mm_set1_ps( ray.hit.t ) );
		const float tmina_0 = LANE( min4, 0 ), tmaxa_1 = LANE( max4, 1 );
		const float tminb_2 = LANE( min4, 2 ), tmaxb_3 = LANE( max4, 3 );
		float dist1 = tmaxa_1 >= tmina_0 ? tmina_0 : BVH_FAR;
		float dist2 = tmaxb_3 >= tminb_2 ? tminb_2 : BVH_FAR;
		if (dist1 > dist2)
		{
			const float t = dist1; dist1 = dist2; dist2 = t;
			const uint32_t i = lidx; lidx = ridx; ridx = i;
		}
		if (dist1 == BVH_FAR)
		{
			if (stackPtr == 0) break; else node = stack[--stackPtr];
		}
		else
		{
			node = bvhNode + lidx;
			if (dist2 != BVH_FAR) stack[stackPtr++] = bvhNode + ridx;
		}
	}
	return (int32_t)cost;
}

// Find occlusions in the second alternative BVH layout (ALT_SOA).
template <> bool impl::BVH_SoA<float, uint32_t>::IsOccluded( const Ray& ray ) const
{
	BVHNode* node = &bvhNode[0], * stack[TINYBVH_STACK_SIZE];
	const bvhvec4slice& verts = bvh.verts;
	const uint32_t* primIdx = bvh.primIdx;
	uint32_t stackPtr = 0;
	const __m128 Ox4 = _mm_set1_ps( ray.O.x ), rDx4 = _mm_set1_ps( ray.rD.x );
	const __m128 Oy4 = _mm_set1_ps( ray.O.y ), rDy4 = _mm_set1_ps( ray.rD.y );
	const __m128 Oz4 = _mm_set1_ps( ray.O.z ), rDz4 = _mm_set1_ps( ray.rD.z );
	while (1)
	{
		if (node->isLeaf())
		{
			if (indexedEnabled && bvh.vertIdx != 0) for (uint32_t i = 0; i < node->triCount; i++)
			{
				const uint32_t pi = primIdx[node->firstTri + i], vi0 = pi * 3;
				const uint32_t i0 = bvh.vertIdx[vi0], i1 = bvh.vertIdx[vi0 + 1], i2 = bvh.vertIdx[vi0 + 2];
				if (TriOccludes( ray, verts, pi, i0, i1, i2 )) return true;
			}
			else for (uint32_t i = 0; i < node->triCount; i++)
			{
				const uint32_t pi = primIdx[node->firstTri + i], vi0 = pi * 3;
				if (TriOccludes( ray, verts, pi, vi0, vi0 + 1, vi0 + 2 )) return true;
			}
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		__m128 x4 = _mm_mul_ps( _mm_sub_ps( _mm_load_ps( node->xxxx ), Ox4 ), rDx4 );
		__m128 y4 = _mm_mul_ps( _mm_sub_ps( _mm_load_ps( node->yyyy ), Oy4 ), rDy4 );
		__m128 z4 = _mm_mul_ps( _mm_sub_ps( _mm_load_ps( node->zzzz ), Oz4 ), rDz4 );
		// transpose
		__m128 t0 = _mm_unpacklo_ps( x4, y4 ), t2 = _mm_unpacklo_ps( z4, z4 );
		__m128 t1 = _mm_unpackhi_ps( x4, y4 ), t3 = _mm_unpackhi_ps( z4, z4 );
		__m128 xyzw1a = _mm_shuffle_ps( t0, t2, _MM_SHUFFLE( 1, 0, 1, 0 ) );
		__m128 xyzw2a = _mm_shuffle_ps( t0, t2, _MM_SHUFFLE( 3, 2, 3, 2 ) );
		__m128 xyzw1b = _mm_shuffle_ps( t1, t3, _MM_SHUFFLE( 1, 0, 1, 0 ) );
		__m128 xyzw2b = _mm_shuffle_ps( t1, t3, _MM_SHUFFLE( 3, 2, 3, 2 ) );
		// process
		__m128 tmina4 = _mm_min_ps( xyzw1a, xyzw2a ), tmaxa4 = _mm_max_ps( xyzw1a, xyzw2a );
		__m128 tminb4 = _mm_min_ps( xyzw1b, xyzw2b ), tmaxb4 = _mm_max_ps( xyzw1b, xyzw2b );
		// transpose back
		t0 = _mm_unpacklo_ps( tmina4, tmaxa4 ), t2 = _mm_unpacklo_ps( tminb4, tmaxb4 );
		t1 = _mm_unpackhi_ps( tmina4, tmaxa4 ), t3 = _mm_unpackhi_ps( tminb4, tmaxb4 );
		x4 = _mm_shuffle_ps( t0, t2, _MM_SHUFFLE( 1, 0, 1, 0 ) );
		y4 = _mm_shuffle_ps( t0, t2, _MM_SHUFFLE( 3, 2, 3, 2 ) );
		z4 = _mm_shuffle_ps( t1, t3, _MM_SHUFFLE( 1, 0, 1, 0 ) );
		uint32_t lidx = node->left, ridx = node->right;
		const __m128 min4 = _mm_max_ps( _mm_max_ps( _mm_max_ps( x4, y4 ), z4 ), _mm_setzero_ps() );
		const __m128 max4 = _mm_min_ps( _mm_min_ps( _mm_min_ps( x4, y4 ), z4 ), _mm_set1_ps( ray.hit.t ) );
		const float tmina_0 = LANE( min4, 0 ), tmaxa_1 = LANE( max4, 1 );
		const float tminb_2 = LANE( min4, 2 ), tmaxb_3 = LANE( max4, 3 );
		float dist1 = tmaxa_1 >= tmina_0 ? tmina_0 : BVH_FAR;
		float dist2 = tmaxb_3 >= tminb_2 ? tminb_2 : BVH_FAR;
		if (dist1 > dist2)
		{
			float t = dist1; dist1 = dist2; dist2 = t;
			uint32_t i = lidx; lidx = ridx; ridx = i;
		}
		if (dist1 == BVH_FAR)
		{
			if (stackPtr == 0) break; else node = stack[--stackPtr];
		}
		else
		{
			node = bvhNode + lidx;
			if (dist2 != BVH_FAR) stack[stackPtr++] = bvhNode + ridx;
		}
	}
	return false;
}

#endif
#ifdef BVH_USEAVX2

// lane compaction table for the traversal stack: idxLUT256[255 - mask].
ALIGNED( 64 ) static const uint32_t idxLUT256[256][8] = {
	{ 0,1,2,3,4,5,6,7 }, { 1,2,3,4,5,6,7,0 }, { 0,2,3,4,5,6,7,0 }, { 2,3,4,5,6,7,0,0 },
	{ 0,1,3,4,5,6,7,0 }, { 1,3,4,5,6,7,0,0 }, { 0,3,4,5,6,7,0,0 }, { 3,4,5,6,7,0,0,0 },
	{ 0,1,2,4,5,6,7,0 }, { 1,2,4,5,6,7,0,0 }, { 0,2,4,5,6,7,0,0 }, { 2,4,5,6,7,0,0,0 },
	{ 0,1,4,5,6,7,0,0 }, { 1,4,5,6,7,0,0,0 }, { 0,4,5,6,7,0,0,0 }, { 4,5,6,7,0,0,0,0 },
	{ 0,1,2,3,5,6,7,0 }, { 1,2,3,5,6,7,0,0 }, { 0,2,3,5,6,7,0,0 }, { 2,3,5,6,7,0,0,0 },
	{ 0,1,3,5,6,7,0,0 }, { 1,3,5,6,7,0,0,0 }, { 0,3,5,6,7,0,0,0 }, { 3,5,6,7,0,0,0,0 },
	{ 0,1,2,5,6,7,0,0 }, { 1,2,5,6,7,0,0,0 }, { 0,2,5,6,7,0,0,0 }, { 2,5,6,7,0,0,0,0 },
	{ 0,1,5,6,7,0,0,0 }, { 1,5,6,7,0,0,0,0 }, { 0,5,6,7,0,0,0,0 }, { 5,6,7,0,0,0,0,0 },
	{ 0,1,2,3,4,6,7,0 }, { 1,2,3,4,6,7,0,0 }, { 0,2,3,4,6,7,0,0 }, { 2,3,4,6,7,0,0,0 },
	{ 0,1,3,4,6,7,0,0 }, { 1,3,4,6,7,0,0,0 }, { 0,3,4,6,7,0,0,0 }, { 3,4,6,7,0,0,0,0 },
	{ 0,1,2,4,6,7,0,0 }, { 1,2,4,6,7,0,0,0 }, { 0,2,4,6,7,0,0,0 }, { 2,4,6,7,0,0,0,0 },
	{ 0,1,4,6,7,0,0,0 }, { 1,4,6,7,0,0,0,0 }, { 0,4,6,7,0,0,0,0 }, { 4,6,7,0,0,0,0,0 },
	{ 0,1,2,3,6,7,0,0 }, { 1,2,3,6,7,0,0,0 }, { 0,2,3,6,7,0,0,0 }, { 2,3,6,7,0,0,0,0 },
	{ 0,1,3,6,7,0,0,0 }, { 1,3,6,7,0,0,0,0 }, { 0,3,6,7,0,0,0,0 }, { 3,6,7,0,0,0,0,0 },
	{ 0,1,2,6,7,0,0,0 }, { 1,2,6,7,0,0,0,0 }, { 0,2,6,7,0,0,0,0 }, { 2,6,7,0,0,0,0,0 },
	{ 0,1,6,7,0,0,0,0 }, { 1,6,7,0,0,0,0,0 }, { 0,6,7,0,0,0,0,0 }, { 6,7,0,0,0,0,0,0 },
	{ 0,1,2,3,4,5,7,0 }, { 1,2,3,4,5,7,0,0 }, { 0,2,3,4,5,7,0,0 }, { 2,3,4,5,7,0,0,0 },
	{ 0,1,3,4,5,7,0,0 }, { 1,3,4,5,7,0,0,0 }, { 0,3,4,5,7,0,0,0 }, { 3,4,5,7,0,0,0,0 },
	{ 0,1,2,4,5,7,0,0 }, { 1,2,4,5,7,0,0,0 }, { 0,2,4,5,7,0,0,0 }, { 2,4,5,7,0,0,0,0 },
	{ 0,1,4,5,7,0,0,0 }, { 1,4,5,7,0,0,0,0 }, { 0,4,5,7,0,0,0,0 }, { 4,5,7,0,0,0,0,0 },
	{ 0,1,2,3,5,7,0,0 }, { 1,2,3,5,7,0,0,0 }, { 0,2,3,5,7,0,0,0 }, { 2,3,5,7,0,0,0,0 },
	{ 0,1,3,5,7,0,0,0 }, { 1,3,5,7,0,0,0,0 }, { 0,3,5,7,0,0,0,0 }, { 3,5,7,0,0,0,0,0 },
	{ 0,1,2,5,7,0,0,0 }, { 1,2,5,7,0,0,0,0 }, { 0,2,5,7,0,0,0,0 }, { 2,5,7,0,0,0,0,0 },
	{ 0,1,5,7,0,0,0,0 }, { 1,5,7,0,0,0,0,0 }, { 0,5,7,0,0,0,0,0 }, { 5,7,0,0,0,0,0,0 },
	{ 0,1,2,3,4,7,0,0 }, { 1,2,3,4,7,0,0,0 }, { 0,2,3,4,7,0,0,0 }, { 2,3,4,7,0,0,0,0 },
	{ 0,1,3,4,7,0,0,0 }, { 1,3,4,7,0,0,0,0 }, { 0,3,4,7,0,0,0,0 }, { 3,4,7,0,0,0,0,0 },
	{ 0,1,2,4,7,0,0,0 }, { 1,2,4,7,0,0,0,0 }, { 0,2,4,7,0,0,0,0 }, { 2,4,7,0,0,0,0,0 },
	{ 0,1,4,7,0,0,0,0 }, { 1,4,7,0,0,0,0,0 }, { 0,4,7,0,0,0,0,0 }, { 4,7,0,0,0,0,0,0 },
	{ 0,1,2,3,7,0,0,0 }, { 1,2,3,7,0,0,0,0 }, { 0,2,3,7,0,0,0,0 }, { 2,3,7,0,0,0,0,0 },
	{ 0,1,3,7,0,0,0,0 }, { 1,3,7,0,0,0,0,0 }, { 0,3,7,0,0,0,0,0 }, { 3,7,0,0,0,0,0,0 },
	{ 0,1,2,7,0,0,0,0 }, { 1,2,7,0,0,0,0,0 }, { 0,2,7,0,0,0,0,0 }, { 2,7,0,0,0,0,0,0 },
	{ 0,1,7,0,0,0,0,0 }, { 1,7,0,0,0,0,0,0 }, { 0,7,0,0,0,0,0,0 }, { 7,0,0,0,0,0,0,0 },
	{ 0,1,2,3,4,5,6,0 }, { 1,2,3,4,5,6,0,0 }, { 0,2,3,4,5,6,0,0 }, { 2,3,4,5,6,0,0,0 },
	{ 0,1,3,4,5,6,0,0 }, { 1,3,4,5,6,0,0,0 }, { 0,3,4,5,6,0,0,0 }, { 3,4,5,6,0,0,0,0 },
	{ 0,1,2,4,5,6,0,0 }, { 1,2,4,5,6,0,0,0 }, { 0,2,4,5,6,0,0,0 }, { 2,4,5,6,0,0,0,0 },
	{ 0,1,4,5,6,0,0,0 }, { 1,4,5,6,0,0,0,0 }, { 0,4,5,6,0,0,0,0 }, { 4,5,6,0,0,0,0,0 },
	{ 0,1,2,3,5,6,0,0 }, { 1,2,3,5,6,0,0,0 }, { 0,2,3,5,6,0,0,0 }, { 2,3,5,6,0,0,0,0 },
	{ 0,1,3,5,6,0,0,0 }, { 1,3,5,6,0,0,0,0 }, { 0,3,5,6,0,0,0,0 }, { 3,5,6,0,0,0,0,0 },
	{ 0,1,2,5,6,0,0,0 }, { 1,2,5,6,0,0,0,0 }, { 0,2,5,6,0,0,0,0 }, { 2,5,6,0,0,0,0,0 },
	{ 0,1,5,6,0,0,0,0 }, { 1,5,6,0,0,0,0,0 }, { 0,5,6,0,0,0,0,0 }, { 5,6,0,0,0,0,0,0 },
	{ 0,1,2,3,4,6,0,0 }, { 1,2,3,4,6,0,0,0 }, { 0,2,3,4,6,0,0,0 }, { 2,3,4,6,0,0,0,0 },
	{ 0,1,3,4,6,0,0,0 }, { 1,3,4,6,0,0,0,0 }, { 0,3,4,6,0,0,0,0 }, { 3,4,6,0,0,0,0,0 },
	{ 0,1,2,4,6,0,0,0 }, { 1,2,4,6,0,0,0,0 }, { 0,2,4,6,0,0,0,0 }, { 2,4,6,0,0,0,0,0 },
	{ 0,1,4,6,0,0,0,0 }, { 1,4,6,0,0,0,0,0 }, { 0,4,6,0,0,0,0,0 }, { 4,6,0,0,0,0,0,0 },
	{ 0,1,2,3,6,0,0,0 }, { 1,2,3,6,0,0,0,0 }, { 0,2,3,6,0,0,0,0 }, { 2,3,6,0,0,0,0,0 },
	{ 0,1,3,6,0,0,0,0 }, { 1,3,6,0,0,0,0,0 }, { 0,3,6,0,0,0,0,0 }, { 3,6,0,0,0,0,0,0 },
	{ 0,1,2,6,0,0,0,0 }, { 1,2,6,0,0,0,0,0 }, { 0,2,6,0,0,0,0,0 }, { 2,6,0,0,0,0,0,0 },
	{ 0,1,6,0,0,0,0,0 }, { 1,6,0,0,0,0,0,0 }, { 0,6,0,0,0,0,0,0 }, { 6,0,0,0,0,0,0,0 },
	{ 0,1,2,3,4,5,0,0 }, { 1,2,3,4,5,0,0,0 }, { 0,2,3,4,5,0,0,0 }, { 2,3,4,5,0,0,0,0 },
	{ 0,1,3,4,5,0,0,0 }, { 1,3,4,5,0,0,0,0 }, { 0,3,4,5,0,0,0,0 }, { 3,4,5,0,0,0,0,0 },
	{ 0,1,2,4,5,0,0,0 }, { 1,2,4,5,0,0,0,0 }, { 0,2,4,5,0,0,0,0 }, { 2,4,5,0,0,0,0,0 },
	{ 0,1,4,5,0,0,0,0 }, { 1,4,5,0,0,0,0,0 }, { 0,4,5,0,0,0,0,0 }, { 4,5,0,0,0,0,0,0 },
	{ 0,1,2,3,5,0,0,0 }, { 1,2,3,5,0,0,0,0 }, { 0,2,3,5,0,0,0,0 }, { 2,3,5,0,0,0,0,0 },
	{ 0,1,3,5,0,0,0,0 }, { 1,3,5,0,0,0,0,0 }, { 0,3,5,0,0,0,0,0 }, { 3,5,0,0,0,0,0,0 },
	{ 0,1,2,5,0,0,0,0 }, { 1,2,5,0,0,0,0,0 }, { 0,2,5,0,0,0,0,0 }, { 2,5,0,0,0,0,0,0 },
	{ 0,1,5,0,0,0,0,0 }, { 1,5,0,0,0,0,0,0 }, { 0,5,0,0,0,0,0,0 }, { 5,0,0,0,0,0,0,0 },
	{ 0,1,2,3,4,0,0,0 }, { 1,2,3,4,0,0,0,0 }, { 0,2,3,4,0,0,0,0 }, { 2,3,4,0,0,0,0,0 },
	{ 0,1,3,4,0,0,0,0 }, { 1,3,4,0,0,0,0,0 }, { 0,3,4,0,0,0,0,0 }, { 3,4,0,0,0,0,0,0 },
	{ 0,1,2,4,0,0,0,0 }, { 1,2,4,0,0,0,0,0 }, { 0,2,4,0,0,0,0,0 }, { 2,4,0,0,0,0,0,0 },
	{ 0,1,4,0,0,0,0,0 }, { 1,4,0,0,0,0,0,0 }, { 0,4,0,0,0,0,0,0 }, { 4,0,0,0,0,0,0,0 },
	{ 0,1,2,3,0,0,0,0 }, { 1,2,3,0,0,0,0,0 }, { 0,2,3,0,0,0,0,0 }, { 2,3,0,0,0,0,0,0 },
	{ 0,1,3,0,0,0,0,0 }, { 1,3,0,0,0,0,0,0 }, { 0,3,0,0,0,0,0,0 }, { 3,0,0,0,0,0,0,0 },
	{ 0,1,2,0,0,0,0,0 }, { 1,2,0,0,0,0,0,0 }, { 0,2,0,0,0,0,0,0 }, { 2,0,0,0,0,0,0,0 },
	{ 0,1,0,0,0,0,0,0 }, { 1,0,0,0,0,0,0,0 }, { 0,0,0,0,0,0,0,0 }, { 0,0,0,0,0,0,0,0 }
};

template <> template <bool posX, bool posY, bool posZ> int32_t impl::BVH8_CPU<float, uint32_t>::IntersectOctant( Ray& ray ) const
{
	ALIGNED( 64 ) uint32_t nodeStack[TINYBVH_STACK_SIZE * 4 /* wide trees push more nodes per step */ + 8];
	ALIGNED( 64 ) float distStack[TINYBVH_STACK_SIZE * 4 + 8];
	const __m256 zero8 = _mm256_setzero_ps();
	__m256 t8 = _mm256_set1_ps( ray.hit.t );
	int32_t stackPtr = 0;
	uint32_t nodeIdx = 0;
	constexpr int signShift = (posX ? 3 : 0) + (posY ? 6 : 0) + (posZ ? 12 : 0);
	const __m256 rx8 = _mm256_set1_ps( ray.O.x * ray.rD.x ), rdx8 = _mm256_set1_ps( ray.rD.x );
	const __m256 ry8 = _mm256_set1_ps( ray.O.y * ray.rD.y ), rdy8 = _mm256_set1_ps( ray.rD.y );
	const __m256 rz8 = _mm256_set1_ps( ray.O.z * ray.rD.z ), rdz8 = _mm256_set1_ps( ray.rD.z );
	const __m256i lane8 = _mm256_setr_epi32( 0, 1, 2, 3, 4, 5, 6, 7 );
	const __m128 ox4 = _mm_set1_ps( ray.O.x ), oy4 = _mm_set1_ps( ray.O.y ), oz4 = _mm_set1_ps( ray.O.z );
	const __m128 dx4 = _mm_set1_ps( ray.D.x ), dy4 = _mm_set1_ps( ray.D.y ), dz4 = _mm_set1_ps( ray.D.z );
	const __m128 one4 = _mm_set1_ps( 1 ), inf4 = _mm_set1_ps( 1e34f );
#ifdef _DEBUG
	// sorry, not even this can be tolerated in this function. Only in debug.
	uint32_t steps = 0;
#endif
	while (1)
	{
		while (!(nodeIdx & LEAF_BIT)) ISLIKELY
		{
		#ifdef _DEBUG
			steps++;
		#endif
			const BVHNode* n = (BVHNode*)(bvh8Data + nodeIdx);
			const __m256 tx1 = _mm256_fmsub_ps( _mm256_load_ps( posX ? n->xmin : n->xmax ), rdx8, rx8 );
			const __m256 ty1 = _mm256_fmsub_ps( _mm256_load_ps( posY ? n->ymin : n->ymax ), rdy8, ry8 );
			const __m256 tz1 = _mm256_fmsub_ps( _mm256_load_ps( posZ ? n->zmin : n->zmax ), rdz8, rz8 );
			const __m256 tx2 = _mm256_fmsub_ps( _mm256_load_ps( posX ? n->xmax : n->xmin ), rdx8, rx8 );
			const __m256 ty2 = _mm256_fmsub_ps( _mm256_load_ps( posY ? n->ymax : n->ymin ), rdy8, ry8 );
			const __m256 tz2 = _mm256_fmsub_ps( _mm256_load_ps( posZ ? n->zmax : n->zmin ), rdz8, rz8 );
			const __m256 tmin = _mm256_max_ps( _mm256_max_ps( tx1, ty1 ), _mm256_max_ps( tz1, zero8 ) );
			const __m256 tmax = _mm256_min_ps( _mm256_min_ps( tx2, ty2 ), _mm256_min_ps( tz2, t8 ) );
			const __m256 mask8 = _mm256_cmp_ps( tmin, tmax, _CMP_LE_OQ );
			const uint32_t mask = _mm256_movemask_ps( mask8 );
			const uint32_t validNodes = __popc( mask );
			if (validNodes == 1)
			{
				const uint32_t lane = __bfind( mask );
				nodeIdx = n->child[lane];
			}
			else if (validNodes > 0)
			{
				const __m256i index = _mm256_srli_epi32( _mm256_load_si256( (const __m256i*)n->perm ), signShift );
				const uint32_t m = _mm256_movemask_ps( _mm256_permutevar8x32_ps( mask8, index ) );
				const __m256i c8 = _mm256_permutevar8x32_epi32( _mm256_load_si256( (const __m256i*)n->child ), index );
				nodeIdx = (uint32_t)_mm_cvtsi128_si32( _mm256_castsi256_si128(
					_mm256_permutevar8x32_epi32( c8, _mm256_set1_epi32( (int32_t)__bfind( m ) ) ) ) );
				// start the fill for the child we are about to descend into; the LUT
				// load, two permutes and two stores below hide the L1/L2 latency.
				_mm_prefetch( (const char*)(bvh8Data + (nodeIdx & 0x1fffffff)), _MM_HINT_T0 );
				const __m256i cpi = _mm256_load_si256( (const __m256i*)idxLUT256[255 - m] );
				const __m256 dist8 = _mm256_permutevar8x32_ps( _mm256_permutevar8x32_ps( tmin, index ), cpi );
				const __m256i child8 = _mm256_permutevar8x32_epi32( c8, cpi );
				_mm256_storeu_si256( (__m256i*)(nodeStack + stackPtr), child8 );
				_mm256_storeu_ps( distStack + stackPtr, dist8 );
				stackPtr += validNodes - 1;
			#ifdef _DEBUG
				BVH_FATAL_ERROR_IF( stackPtr > TINYBVH_STACK_SIZE * 4 - 8, "BVH8_CPU::Intersect, traversal stack overflow." );
			#endif
			}
			else
			{
				if (!stackPtr) ISUNLIKELY goto the_end;
				nodeIdx = nodeStack[--stackPtr];
			}
		}
			if (stackPtr) ISLIKELY
			{
				// An interior node is 256 bytes: x planes, y planes, z planes, child8+perm8 -
				// all four cachelines are read by the node test. A leaf is 192 bytes (3 lines).
				// Nodes sit at arbitrary 64-byte multiples (leafs are 3 blocks, nodes 4), so the
				// adjacent-line prefetcher cannot be relied on to fill in the gaps: issue all four.
				const char* next = (const char*)(bvh8Data + (nodeStack[stackPtr - 1] & 0x1fffffff));
				_mm_prefetch( next, _MM_HINT_T0 ), _mm_prefetch( next + 64, _MM_HINT_T0 );
				_mm_prefetch( next + 128, _MM_HINT_T0 ), _mm_prefetch( next + 192, _MM_HINT_T0 );
			}
				// Moeller-Trumbore ray/triangle intersection algorithm for four triangles
		const BVHTri4Leaf* leaf = (BVHTri4Leaf*)(bvh8Data + (nodeIdx & 0x1fffffff));
		const __m128 hx4 = _mm_fmsub_ps( dy4, _mm_load_ps( leaf->e2z ), _mm_mul_ps( dz4, _mm_load_ps( leaf->e2y ) ) );
		const __m128 hy4 = _mm_fmsub_ps( dz4, _mm_load_ps( leaf->e2x ), _mm_mul_ps( dx4, _mm_load_ps( leaf->e2z ) ) );
		const __m128 hz4 = _mm_fmsub_ps( dx4, _mm_load_ps( leaf->e2y ), _mm_mul_ps( dy4, _mm_load_ps( leaf->e2x ) ) );
		const __m128 sx4 = _mm_sub_ps( ox4, _mm_load_ps( leaf->v0x ) ), sy4 = _mm_sub_ps( oy4, _mm_load_ps( leaf->v0y ) );
		const __m128 sz4 = _mm_sub_ps( oz4, _mm_load_ps( leaf->v0z ) );
		const __m128 det4 = _mm_fmadd_ps( _mm_load_ps( leaf->e1z ), hz4, _mm_fmadd_ps( _mm_load_ps( leaf->e1x ), hx4, _mm_mul_ps( _mm_load_ps( leaf->e1y ), hy4 ) ) );
		const __m128 qz4 = _mm_fmsub_ps( sx4, _mm_load_ps( leaf->e1y ), _mm_mul_ps( sy4, _mm_load_ps( leaf->e1x ) ) );
		const __m128 qx4 = _mm_fmsub_ps( sy4, _mm_load_ps( leaf->e1z ), _mm_mul_ps( sz4, _mm_load_ps( leaf->e1y ) ) );
		const __m128 qy4 = _mm_fmsub_ps( sz4, _mm_load_ps( leaf->e1x ), _mm_mul_ps( sx4, _mm_load_ps( leaf->e1z ) ) );
		const __m128 inv_det4 = _mm_div_ps( one4, det4 );
		const __m128 u4 = _mm_mul_ps( _mm_fmadd_ps( sz4, hz4, _mm_fmadd_ps( sx4, hx4, _mm_mul_ps( sy4, hy4 ) ) ), inv_det4 );
		const __m128 v4 = _mm_mul_ps( _mm_fmadd_ps( dz4, qz4, _mm_fmadd_ps( dx4, qx4, _mm_mul_ps( dy4, qy4 ) ) ), inv_det4 );
		const __m128 ta4 = _mm_mul_ps( _mm_fmadd_ps( _mm_load_ps( leaf->e2z ), qz4, _mm_fmadd_ps( _mm_load_ps( leaf->e2x ), qx4, _mm_mul_ps( _mm_load_ps( leaf->e2y ), qy4 ) ) ), inv_det4 );
		const __m128 mask1 = _mm_cmpge_ps( u4, _mm_setzero_ps() ), mask2 = _mm_cmpge_ps( v4, _mm_setzero_ps() );
		const __m128 mask3 = _mm_cmple_ps( _mm_add_ps( u4, v4 ), one4 );
		const __m128 mask4 = _mm_cmpgt_ps( ta4, _mm_setzero_ps() );
		const __m128 mask5 = _mm_cmplt_ps( ta4, _mm256_castps256_ps128( t8 ) );
		__m128 combined = _mm_and_ps( _mm_and_ps( _mm_and_ps( mask1, mask2 ), _mm_and_ps( mask3, mask4 ) ), mask5 );
		uint32_t imask = _mm_movemask_ps( combined );
		if (imask)
		{
			// evaluate opacity map, if present (SSE version).
			if (opmap) ISUNLIKELY
			{
				const __m128 fN4 = _mm_set1_ps( (float)opmapN );
				const __m128i row4 = _mm_cvttps_epi32( _mm_mul_ps( _mm_add_ps( u4, v4 ), fN4 ) );
				const __m128i dia4 = _mm_cvttps_epi32( _mm_mul_ps( _mm_sub_ps( one4, u4 ), fN4 ) );
				const __m128i v0 = _mm_mullo_epi32( row4, row4 );
				const __m128i v1 = _mm_cvttps_epi32( _mm_mul_ps( v4, fN4 ) );
				const __m128i v2 = _mm_sub_epi32( dia4, _mm_sub_epi32( _mm_set1_epi32( opmapN - 1 ), row4 ) );
				uint32_t idx[4], omask[4] = { 0, 0, 0, 0 };
				tinybvh_store4i( idx, _mm_add_epi32( _mm_add_epi32( v0, v1 ), v2 ) );
				// proceed with scalar code for gather operation - TODO: better approach?
				for (int i = 0; i < 4; i++) if (imask & (1 << i))
				{
					uint32_t* om = opmap + leaf->primIdx[i] * ((opmapN * opmapN + 31) >> 5);
					if (om[idx[i] >> 5] & (1 << (idx[i] & 31))) omask[i] = 0xffffffff;
				}
				// combine
				combined = _mm_and_ps( combined, tinybvh_load4( omask ) );
				imask = _mm_movemask_ps( combined );
			}
				if (imask)
				{
					// compute broadcasted horizontal minimum of dist4
					const __m128 dist4 = _mm_blendv_ps( inf4, ta4, combined );
					const __m128 a = _mm_min_ps( dist4, _mm_shuffle_ps( dist4, dist4, _MM_SHUFFLE( 2, 1, 0, 3 ) ) );
					const __m128 c = _mm_min_ps( a, _mm_shuffle_ps( a, a, _MM_SHUFFLE( 1, 0, 3, 2 ) ) );
					const uint32_t lane = __bfind( _mm_movemask_ps( _mm_cmpeq_ps( c, dist4 ) ) );
					// update hit record.
					const __m128i lane4 = _mm_set1_epi32( (int32_t)lane );
					const float t = _mm_cvtss_f32( _mm_permutevar_ps( dist4, lane4 ) );
					ray.hit.t = t;
					ray.hit.u = _mm_cvtss_f32( _mm_permutevar_ps( u4, lane4 ) );
					ray.hit.v = _mm_cvtss_f32( _mm_permutevar_ps( v4, lane4 ) );
				#if INST_IDX_BITS == 32
					ray.hit.prim = leaf->primIdx[lane], ray.hit.inst = ray.instIdx;
				#else
					ray.hit.prim = leaf->primIdx[lane] + ray.instIdx;
				#endif
					t8 = _mm256_set1_ps( t );
					// compress stack
					int32_t outStackPtr = 0;
					for (int32_t i = 0; i < stackPtr; i += 8)
					{
						const int32_t numItems = tinybvh_min( 8, stackPtr - i );
						const __m256i valid8 = _mm256_cmpgt_epi32( _mm256_set1_epi32( numItems ), lane8 );
						__m256i node8 = _mm256_maskload_epi32( (const int32_t*)(nodeStack + i), valid8 );
						__m256 dist8 = _mm256_maskload_ps( distStack + i, valid8 );
						const uint32_t mask = _mm256_movemask_ps( _mm256_cmp_ps( dist8, t8, _CMP_LE_OQ ) ) & ((1u << numItems) - 1);
						const __m256i cpi = _mm256_load_si256( (const __m256i*)idxLUT256[255 - mask] );
						dist8 = _mm256_permutevar8x32_ps( dist8, cpi ), node8 = _mm256_permutevar8x32_epi32( node8, cpi );
						_mm256_storeu_ps( distStack + outStackPtr, dist8 );
						_mm256_storeu_si256( (__m256i*)(nodeStack + outStackPtr), node8 );
						outStackPtr += __popc( mask );
					}
					stackPtr = outStackPtr;
				}
		}
		if (!stackPtr) ISUNLIKELY break;
		nodeIdx = nodeStack[--stackPtr];
	}
the_end:
#ifdef _DEBUG
	return steps;
#else
	return 0;
#endif
}
template <> template <bool posX, bool posY, bool posZ> bool impl::BVH8_CPU<float, uint32_t>::IsOccludedOctant( const Ray& ray ) const
{
	ALIGNED( 64 ) uint32_t nodeStack[TINYBVH_STACK_SIZE * 4 /* wide trees push more nodes per step */ + 8];
	int32_t stackPtr = 0;
	uint32_t nodeIdx = 0;
	const __m256 zero8 = _mm256_setzero_ps();
	const __m256 t8 = _mm256_set1_ps( ray.hit.t );
	const __m256 rx8 = _mm256_set1_ps( ray.O.x * ray.rD.x ), rdx8 = _mm256_set1_ps( ray.rD.x );
	const __m256 ry8 = _mm256_set1_ps( ray.O.y * ray.rD.y ), rdy8 = _mm256_set1_ps( ray.rD.y );
	const __m256 rz8 = _mm256_set1_ps( ray.O.z * ray.rD.z ), rdz8 = _mm256_set1_ps( ray.rD.z );
	const __m128 ox4 = _mm_set1_ps( ray.O.x ), oy4 = _mm_set1_ps( ray.O.y ), oz4 = _mm_set1_ps( ray.O.z );
	const __m128 dx4 = _mm_set1_ps( ray.D.x ), dy4 = _mm_set1_ps( ray.D.y ), dz4 = _mm_set1_ps( ray.D.z );
	const __m128 t4 = _mm_set1_ps( ray.hit.t );
	const __m128 one4 = _mm_set1_ps( 1.0f ), zero4 = _mm_setzero_ps(), sign4 = _mm_set1_ps( -0.0f );
	while (1)
	{
		while (!(nodeIdx & LEAF_BIT)) ISLIKELY
		{
			const BVHNode * n = (BVHNode*)(bvh8Data + nodeIdx);
			const __m256i c8 = _mm256_load_si256( (const __m256i*)n->child );
			const __m256 tx1 = _mm256_fmsub_ps( _mm256_load_ps( posX ? n->xmin : n->xmax ), rdx8, rx8 );
			const __m256 ty1 = _mm256_fmsub_ps( _mm256_load_ps( posY ? n->ymin : n->ymax ), rdy8, ry8 );
			const __m256 tz1 = _mm256_fmsub_ps( _mm256_load_ps( posZ ? n->zmin : n->zmax ), rdz8, rz8 );
			const __m256 tx2 = _mm256_fmsub_ps( _mm256_load_ps( posX ? n->xmax : n->xmin ), rdx8, rx8 );
			const __m256 ty2 = _mm256_fmsub_ps( _mm256_load_ps( posY ? n->ymax : n->ymin ), rdy8, ry8 );
			const __m256 tz2 = _mm256_fmsub_ps( _mm256_load_ps( posZ ? n->zmax : n->zmin ), rdz8, rz8 );
			const __m256 tmin = _mm256_max_ps( _mm256_max_ps( tx1, ty1 ), _mm256_max_ps( tz1, zero8 ) );
			const __m256 tmax = _mm256_min_ps( _mm256_min_ps( tx2, ty2 ), _mm256_min_ps( tz2, t8 ) );
			const __m256 mask8 = _mm256_cmp_ps( tmin, tmax, _CMP_LE_OQ );
			const uint32_t mask = _mm256_movemask_ps( mask8 );
			const uint32_t validNodes = __popc( mask );
			if (validNodes == 1)
			{
				const uint32_t lane = __bfind( mask );
				nodeIdx = n->child[lane];
			}
			else if (validNodes > 0)
			{
				nodeIdx = (uint32_t)_mm_cvtsi128_si32( _mm256_castsi256_si128(
					_mm256_permutevar8x32_epi32( c8, _mm256_set1_epi32( (int32_t)__bfind( mask ) ) ) ) );
				const __m256i cpi = _mm256_load_si256( (const __m256i*)idxLUT256[255 - mask] );
				const __m256i child8 = _mm256_permutevar8x32_epi32( c8, cpi );
				_mm256_storeu_si256( (__m256i*)(nodeStack + stackPtr), child8 );
				stackPtr += validNodes - 1;
			#ifdef _DEBUG
				BVH_FATAL_ERROR_IF( stackPtr > TINYBVH_STACK_SIZE * 4 - 8, "BVH8_CPU::IsOccluded, traversal stack overflow." );
			#endif
			}
			else
			{
				if (!stackPtr) ISUNLIKELY return false;
				nodeIdx = nodeStack[--stackPtr];
			}
		}
			if (stackPtr) ISLIKELY
			{
				const char* next = (const char*)(bvh8Data + (nodeStack[stackPtr - 1] & 0x1fffffff));
				_mm_prefetch( next, _MM_HINT_T0 ), _mm_prefetch( next + 128, _MM_HINT_T0 );
			}
				// Moeller-Trumbore ray/triangle intersection algorithm for four triangles.
		const BVHTri4Leaf* leaf = (BVHTri4Leaf*)(bvh8Data + (nodeIdx & 0x1fffffff));
		const __m128 hx4 = _mm_fmsub_ps( dy4, _mm_load_ps( leaf->e2z ), _mm_mul_ps( dz4, _mm_load_ps( leaf->e2y ) ) );
		const __m128 hy4 = _mm_fmsub_ps( dz4, _mm_load_ps( leaf->e2x ), _mm_mul_ps( dx4, _mm_load_ps( leaf->e2z ) ) );
		const __m128 hz4 = _mm_fmsub_ps( dx4, _mm_load_ps( leaf->e2y ), _mm_mul_ps( dy4, _mm_load_ps( leaf->e2x ) ) );
		const __m128 sx4 = _mm_sub_ps( ox4, _mm_load_ps( leaf->v0x ) );
		const __m128 sy4 = _mm_sub_ps( oy4, _mm_load_ps( leaf->v0y ) );
		const __m128 sz4 = _mm_sub_ps( oz4, _mm_load_ps( leaf->v0z ) );
		const __m128 det4 = _mm_fmadd_ps( _mm_load_ps( leaf->e1z ), hz4, _mm_fmadd_ps( _mm_load_ps( leaf->e1x ), hx4, _mm_mul_ps( _mm_load_ps( leaf->e1y ), hy4 ) ) );
		const __m128 qz4 = _mm_fmsub_ps( sx4, _mm_load_ps( leaf->e1y ), _mm_mul_ps( sy4, _mm_load_ps( leaf->e1x ) ) );
		const __m128 qx4 = _mm_fmsub_ps( sy4, _mm_load_ps( leaf->e1z ), _mm_mul_ps( sz4, _mm_load_ps( leaf->e1y ) ) );
		const __m128 qy4 = _mm_fmsub_ps( sz4, _mm_load_ps( leaf->e1x ), _mm_mul_ps( sx4, _mm_load_ps( leaf->e1z ) ) );
		const __m128 nu4 = _mm_fmadd_ps( sz4, hz4, _mm_fmadd_ps( sx4, hx4, _mm_mul_ps( sy4, hy4 ) ) );
		const __m128 nv4 = _mm_fmadd_ps( dz4, qz4, _mm_fmadd_ps( dx4, qx4, _mm_mul_ps( dy4, qy4 ) ) );
		const __m128 nt4 = _mm_fmadd_ps( _mm_load_ps( leaf->e2z ), qz4, _mm_fmadd_ps( _mm_load_ps( leaf->e2x ), qx4, _mm_mul_ps( _mm_load_ps( leaf->e2y ), qy4 ) ) );
		const __m128 dsign4 = _mm_and_ps( det4, sign4 ), adet4 = _mm_andnot_ps( sign4, det4 );
		const __m128 u4 = _mm_xor_ps( nu4, dsign4 ), v4 = _mm_xor_ps( nv4, dsign4 );
		const __m128 ta4 = _mm_xor_ps( nt4, dsign4 );
		const __m128 mask1 = _mm_cmpge_ps( u4, zero4 );
		const __m128 mask2 = _mm_cmpge_ps( v4, zero4 );
		const __m128 mask3 = _mm_cmple_ps( _mm_add_ps( u4, v4 ), adet4 );
		const __m128 mask4 = _mm_cmplt_ps( ta4, _mm_mul_ps( t4, adet4 ) );
		const __m128 mask5 = _mm_cmpgt_ps( ta4, zero4 );
		const __m128 combined = _mm_and_ps( _mm_and_ps( _mm_and_ps( mask1, mask2 ), _mm_and_ps( mask3, mask4 ) ), mask5 );
		if (_mm_movemask_ps( combined ))
		{
			if (!opmap) return true;
			// evaluate opacity map, SSE version.
			const __m128 inv_det4 = _mm_div_ps( one4, det4 );
			const __m128 bu4 = _mm_mul_ps( nu4, inv_det4 ), bv4 = _mm_mul_ps( nv4, inv_det4 );
			const __m128 fN4 = _mm_set1_ps( (float)opmapN );
			const __m128i row4 = _mm_cvttps_epi32( _mm_mul_ps( _mm_add_ps( bu4, bv4 ), fN4 ) );
			const __m128i dia4 = _mm_cvttps_epi32( _mm_mul_ps( _mm_sub_ps( one4, bu4 ), fN4 ) );
			const __m128i v0 = _mm_mullo_epi32( row4, row4 );
			const __m128i v1 = _mm_cvttps_epi32( _mm_mul_ps( bv4, fN4 ) );
			const __m128i v2 = _mm_sub_epi32( dia4, _mm_sub_epi32( _mm_set1_epi32( opmapN - 1 ), row4 ) );
			uint32_t idx[4];
			tinybvh_store4i( idx, _mm_add_epi32( _mm_add_epi32( v0, v1 ), v2 ) );
			// proceed with scalar code for gather operation - TODO: better approach?
			const uint32_t imask = _mm_movemask_ps( combined );
			for (int i = 0; i < 4; i++) if (imask & (1 << i))
			{
				uint32_t* om = opmap + leaf->primIdx[i] * ((opmapN * opmapN + 31) >> 5);
				if (om[idx[i] >> 5] & (1 << (idx[i] & 31))) return true;
			}
		}
		// continue
		if (!stackPtr) ISUNLIKELY return false;
		nodeIdx = nodeStack[--stackPtr];
	}
}

#endif // BVH_USEAVX2

#endif // BVH_USEAVX

} // namespace tinybvh

#endif // TINYBVH_IMPLEMENTATION
