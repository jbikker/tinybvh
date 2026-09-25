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
template <> int32_t impl::BVH4_CPU<float, uint32_t>::IntersectBundle( Ray* rays ) const;
template <> int32_t impl::BVH<float, uint32_t>::IntersectBundle( Ray* rays ) const;
template <> int32_t impl::BVH4_CPU<float, uint32_t>::IsOccludedBundle( Ray* rays, bool* occluded ) const;
template <> int32_t impl::BVH<float, uint32_t>::IsOccludedBundle( Ray* rays, bool* occluded ) const;
#endif
#ifdef BVH_USEAVX
template <> struct impl::BVHSIMDBuilders<float, uint32_t> { static constexpr bool available = true; };
template <> void impl::BVH<float, uint32_t>::PrepareSIMDBuild( const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t primCount );
template <> void impl::BVH<float, uint32_t>::PrepareSIMDBuildFragSlice( const uint32_t first, const uint32_t last, const uint32_t* indices, const int8_t* vertData, const uint32_t stride4, void* frags, float* rootMin, float* rootMax );
template <> void impl::BVH<float, uint32_t>::BuildSIMDBinTask( const uint32_t first, const uint32_t last, void* binbox, uint32_t* count, const float* nmin4, const float* rpd4 );
template <> void impl::BVH<float, uint32_t>::BuildSIMDSubtree( uint32_t nodeIdx, uint32_t depth );
template <> void impl::BVH<float, uint32_t>::BuildSIMDFinalize();
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
#ifndef TINY_BVH_X86_FLOAT_H_IMPL
#define TINY_BVH_X86_FLOAT_H_IMPL

namespace tinybvh {

#ifdef BVH_USESSE

#define AVXBINS 8 // must stay at 8.

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
void impl::BuildAVXFragSlice( uint32_t i, void* payload )
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
void impl::BVHBuildAVXSubtree( void* payload )
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
void impl::BVHBuildAVXBinSlice( uint32_t i, void* payload )
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
	ALIGNED( 64 ) uint32_t task[TINYBVH_STACK_SIZE], taskCount = 0;
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

#if 0

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

#else

template <> template <bool posX, bool posY, bool posZ> int32_t impl::BVH8_CPU<float, uint32_t>::IntersectOctant( Ray& ray ) const
{
	ALIGNED( 64 ) uint32_t nodeStack[TINYBVH_STACK_SIZE * 4 /* wide trees push more nodes per step */ + 8];
	ALIGNED( 64 ) float distStack[TINYBVH_STACK_SIZE * 4 + 8];
	const __m256 zero8 = _mm256_setzero_ps();
	__m256 t8 = _mm256_set1_ps( ray.hit.t );
	int32_t stackPtr = 0;
	uint32_t nodeIdx = 0;
	const __m256 rx8 = _mm256_set1_ps( ray.O.x * ray.rD.x ), rdx8 = _mm256_set1_ps( ray.rD.x );
	const __m256 ry8 = _mm256_set1_ps( ray.O.y * ray.rD.y ), rdy8 = _mm256_set1_ps( ray.rD.y );
	const __m256 rz8 = _mm256_set1_ps( ray.O.z * ray.rD.z ), rdz8 = _mm256_set1_ps( ray.rD.z );
	const __m256i lane8 = _mm256_setr_epi32( 0, 1, 2, 3, 4, 5, 6, 7 );
#ifdef BVH8_SORTING_NETWORK
	const __m256i sentinel8 = _mm256_setr_epi32( (int32_t)0x80000000, (int32_t)0x80000001,
		(int32_t)0x80000002, (int32_t)0x80000003, (int32_t)0x80000004, (int32_t)0x80000005,
		(int32_t)0x80000006, (int32_t)0x80000007 );
#endif
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
		#ifdef BVH8_USE_PREFETCHING
			// prefetch child data. only incoherent rays benefit.
			for (uint32_t m = mask; m; m &= m - 1)
			{
			#if defined _MSC_VER && !defined __clang__
				unsigned long lane;
				_BitScanForward( &lane, m );
			#else
				const uint32_t lane = __builtin_ctz( m );
			#endif
				const char* p = (const char*)(bvh8Data + (n->child[lane] & 0x1fffffff));
				_mm_prefetch( p, _MM_HINT_T0 ), _mm_prefetch( p + 64, _MM_HINT_T0 );
				_mm_prefetch( p + 128, _MM_HINT_T0 ), _mm_prefetch( p + 192, _MM_HINT_T0 );
			}
		#endif
			if (validNodes == 1)
			{
				const uint32_t lane = __bfind( mask );
				nodeIdx = n->child[lane];
			}
		#ifdef BVH8_2VALIDNODES
			else if (validNodes == 2)
			{
				// The highest and lowest set bits identify the two children independently.
				// This avoids both the sorting network and dependent bit scans.
				const uint32_t lane0 = __bfind( mask );
			#if defined _MSC_VER && !defined __clang__
				unsigned long lane1;
				_BitScanForward( &lane1, mask );
			#else
				const uint32_t lane1 = __builtin_ctz( mask );
			#endif
				const float dist0 = _mm_cvtss_f32( _mm256_castps256_ps128( _mm256_permutevar8x32_ps( tmin, _mm256_set1_epi32( lane0 ) ) ) );
				const float dist1 = _mm_cvtss_f32( _mm256_castps256_ps128( _mm256_permutevar8x32_ps( tmin, _mm256_set1_epi32( lane1 ) ) ) );
				const bool first = dist0 < dist1;
				nodeIdx = n->child[first ? lane0 : lane1];
				nodeStack[stackPtr] = n->child[first ? lane1 : lane0];
				distStack[stackPtr++] = first ? dist1 : dist0;
			}
		#endif
			else if (validNodes > 0)
			{
			#ifndef BVH8_SORTING_NETWORK
				constexpr int signShift = (posX ? 3 : 0) + (posY ? 6 : 0) + (posZ ? 12 : 0);
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
			#else
				// use a sorting network to sort by entry distance.
				__m256i d = _mm256_or_si256( _mm256_and_si256( _mm256_castps_si256( tmin ), _mm256_set1_epi32( -8 ) ), lane8 );
				d = _mm256_blendv_epi8( _mm256_set1_epi32( 0x7fffffff ), d, _mm256_castps_si256( mask8 ) );
				#define TINYBVH_SORT8( shuffle, blend ) { const __m256i other = shuffle; \
					d = _mm256_blend_epi32( _mm256_min_epi32( d, other ), _mm256_max_epi32( d, other ), blend ); }
				TINYBVH_SORT8( _mm256_shuffle_epi32( d, _MM_SHUFFLE( 2, 3, 0, 1 ) ), 0x66 );
				TINYBVH_SORT8( _mm256_shuffle_epi32( d, _MM_SHUFFLE( 1, 0, 3, 2 ) ), 0x3c );
				TINYBVH_SORT8( _mm256_shuffle_epi32( d, _MM_SHUFFLE( 2, 3, 0, 1 ) ), 0x5a );
				// The lower half is ascending and the upper half descending. Fetch
				// their minimum now so the descent can start during the remaining merge.
				const __m128i otherMin = _mm_shuffle_epi32( _mm256_extracti128_si256( d, 1 ), _MM_SHUFFLE( 3, 3, 3, 3 ) );
				const uint32_t nearest = _mm_cvtsi128_si32( _mm_min_epi32( _mm256_castsi256_si128( d ), otherMin ) ) & 7;
				nodeIdx = n->child[nearest];
				TINYBVH_SORT8( _mm256_permute2x128_si256( d, d, 1 ), 0xf0 );
				TINYBVH_SORT8( _mm256_shuffle_epi32( d, _MM_SHUFFLE( 1, 0, 3, 2 ) ), 0xcc );
				TINYBVH_SORT8( _mm256_shuffle_epi32( d, _MM_SHUFFLE( 2, 3, 0, 1 ) ), 0xaa );
				// Reverse to put the closest child at the top of the stack.
				const __m256i order = _mm256_permutevar8x32_epi32( d, _mm256_sub_epi32( _mm256_set1_epi32( validNodes - 1 ), lane8 ) );
				_mm256_storeu_si256( (__m256i*)(nodeStack + stackPtr), _mm256_permutevar8x32_epi32( _mm256_load_si256( (const __m256i*)n->child ), order ) );
				_mm256_storeu_ps( distStack + stackPtr, _mm256_permutevar8x32_ps( tmin, order ) );
			#endif
				stackPtr += validNodes - 1;
			}
			else
			{
				if (!stackPtr) ISUNLIKELY goto the_end;
				nodeIdx = nodeStack[--stackPtr];
			}
		#ifdef _DEBUG
			BVH_FATAL_ERROR_IF( stackPtr > TINYBVH_STACK_SIZE * 4 - 8, "BVH8_CPU::Intersect, traversal stack overflow." );
		#endif
		}
	#ifdef BVH8_USE_PREFETCHING
		if (stackPtr) ISLIKELY
		{
			// prefetch the next node - only divergent rays benefit.
			const char* next = (const char*)(bvh8Data + (nodeStack[stackPtr - 1] & 0x1fffffff));
			_mm_prefetch( next, _MM_HINT_T0 ), _mm_prefetch( next + 64, _MM_HINT_T0 );
			_mm_prefetch( next + 128, _MM_HINT_T0 ), _mm_prefetch( next + 192, _MM_HINT_T0 );
		}
	#endif
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
		const __m128 signMask4 = _mm_set1_ps( -0.0f );
		const __m128 detSign4 = _mm_and_ps( det4, signMask4 );
		const __m128 absDet4 = _mm_andnot_ps( signMask4, det4 );
		const __m128 U4 = _mm_xor_ps( _mm_fmadd_ps( sz4, hz4, _mm_fmadd_ps( sx4, hx4, _mm_mul_ps( sy4, hy4 ) ) ), detSign4 );
		const __m128 V4 = _mm_xor_ps( _mm_fmadd_ps( dz4, qz4, _mm_fmadd_ps( dx4, qx4, _mm_mul_ps( dy4, qy4 ) ) ), detSign4 );
		const __m128 T4 = _mm_xor_ps( _mm_fmadd_ps( _mm_load_ps( leaf->e2z ), qz4, _mm_fmadd_ps( _mm_load_ps( leaf->e2x ), qx4, _mm_mul_ps( _mm_load_ps( leaf->e2y ), qy4 ) ) ), detSign4 );
		const __m128 mask1 = _mm_cmpge_ps( U4, _mm_setzero_ps() ), mask2 = _mm_cmpge_ps( V4, _mm_setzero_ps() );
		const __m128 mask3 = _mm_cmple_ps( _mm_add_ps( U4, V4 ), absDet4 );
		const __m128 mask4 = _mm_cmpgt_ps( T4, _mm_setzero_ps() );
		const __m128 mask5 = _mm_cmplt_ps( T4, _mm_mul_ps( _mm256_castps256_ps128( t8 ), absDet4 ) );
		__m128 combined = _mm_and_ps( _mm_and_ps( _mm_and_ps( mask1, mask2 ), _mm_and_ps( mask3, mask4 ) ), mask5 );
		uint32_t imask = _mm_movemask_ps( combined );
		if (imask)
		{
			const __m128 inv_det4 = _mm_div_ps( one4, absDet4 );
			const __m128 u4 = _mm_mul_ps( U4, inv_det4 );
			const __m128 v4 = _mm_mul_ps( V4, inv_det4 );
			const __m128 ta4 = _mm_mul_ps( T4, inv_det4 );
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

#endif

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

// WiVeC bundle traversal, AVX2. Fuetterling et al., HPG 2017, section 4, and
// rend.c's intersect_pckts_blas / intersect_pckts_tlas. Closest hit only.

static TINYBVH_FORCEINLINE uint32_t tinybvh_ctz( uint32_t x ) // lowest set bit
{
#if defined _MSC_VER && !defined __clang__
	unsigned long i;
	_BitScanForward( &i, x );
	return (uint32_t)i;
#else
	return (uint32_t)__builtin_ctz( x );
#endif
}

// RayBundle: the rays of one IntersectBundle call, in SoA form.
struct ALIGNED( 64 ) RayBundle
{
	float ox[TINYBVH_BUNDLE_RAYS], oy[TINYBVH_BUNDLE_RAYS], oz[TINYBVH_BUNDLE_RAYS];
	float dx[TINYBVH_BUNDLE_RAYS], dy[TINYBVH_BUNDLE_RAYS], dz[TINYBVH_BUNDLE_RAYS];
	float rdx[TINYBVH_BUNDLE_RAYS], rdy[TINYBVH_BUNDLE_RAYS], rdz[TINYBVH_BUNDLE_RAYS];
	float t[TINYBVH_BUNDLE_RAYS], u[TINYBVH_BUNDLE_RAYS], v[TINYBVH_BUNDLE_RAYS];
	uint32_t prim[TINYBVH_BUNDLE_RAYS];			// hit primitive, without instance bits
	uint32_t inst[TINYBVH_BUNDLE_RAYS];			// hit instance, pre-shifted
	uint8_t occluded[TINYBVH_BUNDLE_PACKETS];	// per-packet lane mask, occlusion only
	uint16_t src[TINYBVH_BUNDLE_RAYS];			// slot -> slot in the bundle this came from
	uint32_t packets = 0, instIdx = 0;			// packets in use; instance whose space this is
	uint32_t mask = RAY_MASK_INTERSECT_ALL;		// taken from ray 0 of the bundle
	int32_t Octant( const uint32_t p ) const
	{
		const uint32_t l = p * 8;
		const int32_t o = (rdx[l] >= 0 ? 1 : 0) + (rdy[l] >= 0 ? 2 : 0) + (rdz[l] >= 0 ? 4 : 0);
		for (uint32_t i = 1; i < 8; i++) if (o != ((rdx[l + i] >= 0 ? 1 : 0) +
			(rdy[l + i] >= 0 ? 2 : 0) + (rdz[l + i] >= 0 ? 4 : 0))) return -1;
		return o;
	}
	void SetRay( const uint32_t s, const Ray& ray )
	{
		ox[s] = ray.O.x, oy[s] = ray.O.y, oz[s] = ray.O.z;
		dx[s] = ray.D.x, dy[s] = ray.D.y, dz[s] = ray.D.z;
		rdx[s] = ray.rD.x, rdy[s] = ray.rD.y, rdz[s] = ray.rD.z;
		t[s] = ray.hit.t, u[s] = ray.hit.u, v[s] = ray.hit.v, src[s] = (uint16_t)s;
	#if INST_IDX_BITS == 32
		prim[s] = ray.hit.prim, inst[s] = ray.hit.inst;
	#else
		prim[s] = ray.hit.prim & PRIM_IDX_MASK, inst[s] = ray.hit.prim & ~PRIM_IDX_MASK;
	#endif
	}
	void GetHit( const uint32_t s, Ray& ray ) const
	{
		ray.hit.t = t[s], ray.hit.u = u[s], ray.hit.v = v[s];
	#if INST_IDX_BITS == 32
		ray.hit.prim = prim[s], ray.hit.inst = inst[s];
	#else
		ray.hit.prim = (prim[s] & PRIM_IDX_MASK) + inst[s];
	#endif
	}
	// Build the object space counterpart of the listed packets for one instance.
	void TransformTo( RayBundle& dst, const uint32_t* pk, const uint32_t n,
		const bvhmat4& inv, const uint32_t instance ) const
	{
		for (uint32_t i = 0; i < n; i++) for (uint32_t k = 0; k < 8; k++)
		{
			const uint32_t s = pk[i] * 8 + k, d = i * 8 + k;
			const bvhvec3 O = tinybvh_transform_point( bvhvec3( ox[s], oy[s], oz[s] ), inv );
			const bvhvec3 D = tinybvh_transform_vector( bvhvec3( dx[s], dy[s], dz[s] ), inv );
			dst.ox[d] = O.x, dst.oy[d] = O.y, dst.oz[d] = O.z;
			dst.dx[d] = D.x, dst.dy[d] = D.y, dst.dz[d] = D.z;
			dst.rdx[d] = tinybvh_safercp( D.x ), dst.rdy[d] = tinybvh_safercp( D.y ), dst.rdz[d] = tinybvh_safercp( D.z );
			dst.t[d] = t[s], dst.u[d] = u[s], dst.v[d] = v[s];
			dst.prim[d] = prim[s], dst.inst[d] = inst[s], dst.src[d] = (uint16_t)s;
		}
		for (uint32_t i = 0; i < n; i++) dst.occluded[i] = occluded[pk[i]];
		dst.packets = n, dst.instIdx = instance, dst.mask = mask;
	}
	// Take back whatever the object space bundle found; true if anything moved.
	bool MergeHits( const RayBundle& o )
	{
		bool improved = false;
		for (uint32_t d = 0, e = o.packets * 8; d < e; d++)
		{
			const uint32_t s = o.src[d];
			if (o.t[d] < t[s]) t[s] = o.t[d], u[s] = o.u[d], v[s] = o.v[d],
				prim[s] = o.prim[d], inst[s] = o.inst[d], improved = true;
		}
		return improved;
	}
};

static int32_t tinybvh_trace_packets( const BVHBase* bvh, RayBundle& b, const uint32_t* pk, const uint32_t n );

template <bool posX, bool posY, bool posZ>
static TINYBVH_FORCEINLINE float tinybvh_interval_slab( const bvhvec3& bmin, const bvhvec3& bmax,
	const float* rdMin, const float* rdMax, const float* roMin, const float* roMax, const float tfar )
{
	const float ex = posX ? bmin.x : bmax.x, ey = posY ? bmin.y : bmax.y, ez = posZ ? bmin.z : bmax.z;
	const float fx = posX ? bmax.x : bmin.x, fy = posY ? bmax.y : bmin.y, fz = posZ ? bmax.z : bmin.z;
	const float x1 = tinybvh_min( ex * rdMin[0], ex * rdMax[0] ) - roMax[0];
	const float y1 = tinybvh_min( ey * rdMin[1], ey * rdMax[1] ) - roMax[1];
	const float z1 = tinybvh_min( ez * rdMin[2], ez * rdMax[2] ) - roMax[2];
	const float x2 = tinybvh_max( fx * rdMin[0], fx * rdMax[0] ) - roMin[0];
	const float y2 = tinybvh_max( fy * rdMin[1], fy * rdMax[1] ) - roMin[1];
	const float z2 = tinybvh_max( fz * rdMin[2], fz * rdMax[2] ) - roMin[2];
	const float tmin = tinybvh_max( tinybvh_max( x1, y1 ), tinybvh_max( z1, 0.0f ) );
	const float tmax = tinybvh_min( tinybvh_min( x2, y2 ), tinybvh_min( z2, tfar ) );
	return tmin <= tmax ? tmin : BVH_FAR;
}

// Trace one ray through a blas of any layout.
static int32_t tinybvh_blas_intersect( const BVHBase* blas, Ray& ray )
{
	if (blas->layout == LAYOUT_BVH) return ((BVH*)blas)->Intersect( ray );
#ifdef ENABLE_VOXEL_SUPPORT
	if (blas->layout == LAYOUT_VOXELSET) return ((VoxelSet*)blas)->Intersect( ray );
#endif
	if (blas->layout == LAYOUT_BVH4_CPU) return ((BVH4_CPU*)blas)->Intersect( ray );
	if (blas->layout == LAYOUT_BVH8_AVX2) return ((BVH8_CPU*)blas)->Intersect( ray );
	assert( !"unsupported BLAS layout" );
	return 0;
}

// Trace selected packets one ray at a time - whenever we can't do packet traversal.
static int32_t tinybvh_packets_per_ray( const BVHBase* bvh, RayBundle& b, const uint32_t* pk, const uint32_t n )
{
	int32_t cost = 0;
	for (uint32_t i = 0; i < n; i++) for (uint32_t l = pk[i] * 8, e = l + 8; l < e; l++)
	{
		Ray ray( bvhvec3( b.ox[l], b.oy[l], b.oz[l] ), bvhvec3( b.dx[l], b.dy[l], b.dz[l] ), b.t[l] );
		ray.instIdx = b.instIdx, ray.mask = b.mask;
		cost += tinybvh_blas_intersect( bvh, ray );
		if (!(ray.hit.t < b.t[l])) continue;
		b.t[l] = ray.hit.t, b.u[l] = ray.hit.u, b.v[l] = ray.hit.v;
	#if INST_IDX_BITS == 32
		b.prim[l] = ray.hit.prim, b.inst[l] = ray.hit.inst;
	#else
		b.prim[l] = ray.hit.prim & PRIM_IDX_MASK, b.inst[l] = ray.hit.prim & ~PRIM_IDX_MASK;
	#endif
		if (b.instIdx) b.inst[l] = b.instIdx;
	}
	return cost;
}

// Broadcasted horizontal minimum and maximum of eight floats.
static TINYBVH_FORCEINLINE __m256 tinybvh_hmin8( const __m256 a )
{
	__m256 x = _mm256_min_ps( a, _mm256_permute_ps( a, _MM_SHUFFLE( 2, 3, 0, 1 ) ) );
	x = _mm256_min_ps( x, _mm256_permute_ps( x, _MM_SHUFFLE( 1, 0, 3, 2 ) ) );
	return _mm256_min_ps( x, _mm256_permute2f128_ps( x, x, 1 ) );
}
static TINYBVH_FORCEINLINE __m256 tinybvh_hmax8( const __m256 a )
{
	__m256 x = _mm256_max_ps( a, _mm256_permute_ps( a, _MM_SHUFFLE( 2, 3, 0, 1 ) ) );
	x = _mm256_max_ps( x, _mm256_permute_ps( x, _MM_SHUFFLE( 1, 0, 3, 2 ) ) );
	return _mm256_max_ps( x, _mm256_permute2f128_ps( x, x, 1 ) );
}
static TINYBVH_FORCEINLINE __m128 tinybvh_lo4( const __m256 a ) { return _mm256_castps256_ps128( a ); }

// Extract actual prim count from a leaf; ConvertFrom pads with repetitions.
static TINYBVH_FORCEINLINE uint32_t tinybvh_leaftris( const uint32_t* primIdx )
{
	uint32_t n = 4;
	while (n > 1 && primIdx[n - 1] == primIdx[n - 2]) n--;
	return n;
}

// Interval slab test for four child boxes of one node.
#define TINYBVH_INTERVAL_SLAB_TEST \
	const __m128 bx1 = _mm_load_ps( posX ? n->xmin : n->xmax ), by1 = _mm_load_ps( posY ? n->ymin : n->ymax ); \
	const __m128 bz1 = _mm_load_ps( posZ ? n->zmin : n->zmax ), bx2 = _mm_load_ps( posX ? n->xmax : n->xmin ); \
	const __m128 by2 = _mm_load_ps( posY ? n->ymax : n->ymin ), bz2 = _mm_load_ps( posZ ? n->zmax : n->zmin ); \
	const __m128 tx1 = _mm_min_ps( _mm_fmsub_ps( bx1, rdxMin4, rxMax4 ), _mm_fmsub_ps( bx1, rdxMax4, rxMax4 ) ); \
	const __m128 ty1 = _mm_min_ps( _mm_fmsub_ps( by1, rdyMin4, ryMax4 ), _mm_fmsub_ps( by1, rdyMax4, ryMax4 ) ); \
	const __m128 tz1 = _mm_min_ps( _mm_fmsub_ps( bz1, rdzMin4, rzMax4 ), _mm_fmsub_ps( bz1, rdzMax4, rzMax4 ) ); \
	const __m128 tx2 = _mm_max_ps( _mm_fmsub_ps( bx2, rdxMin4, rxMin4 ), _mm_fmsub_ps( bx2, rdxMax4, rxMin4 ) ); \
	const __m128 ty2 = _mm_max_ps( _mm_fmsub_ps( by2, rdyMin4, ryMin4 ), _mm_fmsub_ps( by2, rdyMax4, ryMin4 ) ); \
	const __m128 tz2 = _mm_max_ps( _mm_fmsub_ps( bz2, rdzMin4, rzMin4 ), _mm_fmsub_ps( bz2, rdzMax4, rzMin4 ) ); \
	const __m128 itmin = _mm_max_ps( _mm_max_ps( tx1, ty1 ), _mm_max_ps( tz1, zero4 ) ); \
	const __m128 itmax = _mm_min_ps( _mm_min_ps( tx2, ty2 ), _mm_min_ps( tz2, far4 ) ); \
	const uint32_t mask = (uint32_t)_mm_movemask_ps( _mm_cmple_ps( itmin, itmax ) );

// Test one packet against the box in px1..pz2.
#define TINYBVH_PACKET_BOX_TEST( l ) _mm256_movemask_ps( _mm256_cmp_ps( _mm256_max_ps( _mm256_max_ps( \
	_mm256_fmsub_ps( px1, _mm256_load_ps( b.rdx + (l) ), _mm256_load_ps( rx + (l) ) ), _mm256_fmsub_ps( py1, \
	_mm256_load_ps( b.rdy + (l) ), _mm256_load_ps( ry + (l) ) ) ), _mm256_max_ps( _mm256_fmsub_ps( pz1, \
	_mm256_load_ps( b.rdz + (l) ), _mm256_load_ps( rz + (l) ) ), zero8 ) ), _mm256_min_ps( _mm256_min_ps( \
	_mm256_fmsub_ps( px2, _mm256_load_ps( b.rdx + (l) ), _mm256_load_ps( rx + (l) ) ), _mm256_fmsub_ps( py2, \
	_mm256_load_ps( b.rdy + (l) ), _mm256_load_ps( ry + (l) ) ) ), _mm256_min_ps( _mm256_fmsub_ps( pz2, \
	_mm256_load_ps( b.rdz + (l) ), _mm256_load_ps( rz + (l) ) ), _mm256_load_ps( b.t + (l) ) ) ), _CMP_LE_OQ ) )

// Fold the interval ray over the listed packets, and cache O * rD per slot.
#define TINYBVH_BUNDLE_SETUP \
	__m256 rdxMin, rdxMax, rdyMin, rdyMax, rdzMin, rdzMax; \
	__m256 rxMin, rxMax, ryMin, ryMax, rzMin, rzMax, tmax8; \
	for (uint32_t i = 0; i < n; i++) { \
		const uint32_t l = pk[i] * 8; \
		const __m256 rdx = _mm256_load_ps( b.rdx + l ), rdy = _mm256_load_ps( b.rdy + l ), rdz = _mm256_load_ps( b.rdz + l ); \
		const __m256 qx = _mm256_mul_ps( _mm256_load_ps( b.ox + l ), rdx ); \
		const __m256 qy = _mm256_mul_ps( _mm256_load_ps( b.oy + l ), rdy ); \
		const __m256 qz = _mm256_mul_ps( _mm256_load_ps( b.oz + l ), rdz ); \
		_mm256_store_ps( rx + l, qx ), _mm256_store_ps( ry + l, qy ), _mm256_store_ps( rz + l, qz ); \
		if (i == 0) rdxMin = rdxMax = rdx, rdyMin = rdyMax = rdy, rdzMin = rdzMax = rdz, \
			rxMin = rxMax = qx, ryMin = ryMax = qy, rzMin = rzMax = qz, tmax8 = _mm256_load_ps( b.t + l ); \
		else rdxMin = _mm256_min_ps( rdxMin, rdx ), rdxMax = _mm256_max_ps( rdxMax, rdx ), \
			rdyMin = _mm256_min_ps( rdyMin, rdy ), rdyMax = _mm256_max_ps( rdyMax, rdy ), \
			rdzMin = _mm256_min_ps( rdzMin, rdz ), rdzMax = _mm256_max_ps( rdzMax, rdz ), \
			rxMin = _mm256_min_ps( rxMin, qx ), rxMax = _mm256_max_ps( rxMax, qx ), \
			ryMin = _mm256_min_ps( ryMin, qy ), ryMax = _mm256_max_ps( ryMax, qy ), \
			rzMin = _mm256_min_ps( rzMin, qz ), rzMax = _mm256_max_ps( rzMax, qz ), \
			tmax8 = _mm256_max_ps( tmax8, _mm256_load_ps( b.t + l ) ); }

// Moeller-Trumbore, eight rays against one triangle. Mirrors the operation
// order of the four-triangle version in the single ray kernel, so the two agree
// bit for bit.
#define TINYBVH_PACKET_TRI( i ) \
	const __m256 v0x8 = _mm256_broadcast_ss( leaf->v0x + (i) ), v0y8 = _mm256_broadcast_ss( leaf->v0y + (i) ); \
	const __m256 v0z8 = _mm256_broadcast_ss( leaf->v0z + (i) ), e1x8 = _mm256_broadcast_ss( leaf->e1x + (i) ); \
	const __m256 e1y8 = _mm256_broadcast_ss( leaf->e1y + (i) ), e1z8 = _mm256_broadcast_ss( leaf->e1z + (i) ); \
	const __m256 e2x8 = _mm256_broadcast_ss( leaf->e2x + (i) ), e2y8 = _mm256_broadcast_ss( leaf->e2y + (i) ); \
	const __m256 e2z8 = _mm256_broadcast_ss( leaf->e2z + (i) ); \
	const __m256 hx8 = _mm256_fmsub_ps( dy8, e2z8, _mm256_mul_ps( dz8, e2y8 ) ); \
	const __m256 hy8 = _mm256_fmsub_ps( dz8, e2x8, _mm256_mul_ps( dx8, e2z8 ) ); \
	const __m256 hz8 = _mm256_fmsub_ps( dx8, e2y8, _mm256_mul_ps( dy8, e2x8 ) ); \
	const __m256 sx8 = _mm256_sub_ps( ox8, v0x8 ), sy8 = _mm256_sub_ps( oy8, v0y8 ), sz8 = _mm256_sub_ps( oz8, v0z8 ); \
	const __m256 det8 = _mm256_fmadd_ps( e1z8, hz8, _mm256_fmadd_ps( e1x8, hx8, _mm256_mul_ps( e1y8, hy8 ) ) ); \
	const __m256 qz8 = _mm256_fmsub_ps( sx8, e1y8, _mm256_mul_ps( sy8, e1x8 ) ); \
	const __m256 qx8 = _mm256_fmsub_ps( sy8, e1z8, _mm256_mul_ps( sz8, e1y8 ) ); \
	const __m256 qy8 = _mm256_fmsub_ps( sz8, e1x8, _mm256_mul_ps( sx8, e1z8 ) ); \
	const __m256 invDet8 = _mm256_div_ps( one8, det8 ); \
	const __m256 bu8 = _mm256_mul_ps( _mm256_fmadd_ps( sz8, hz8, _mm256_fmadd_ps( sx8, hx8, _mm256_mul_ps( sy8, hy8 ) ) ), invDet8 ); \
	const __m256 bv8 = _mm256_mul_ps( _mm256_fmadd_ps( dz8, qz8, _mm256_fmadd_ps( dx8, qx8, _mm256_mul_ps( dy8, qy8 ) ) ), invDet8 ); \
	const __m256 bt8 = _mm256_mul_ps( _mm256_fmadd_ps( e2z8, qz8, _mm256_fmadd_ps( e2x8, qx8, _mm256_mul_ps( e2y8, qy8 ) ) ), invDet8 ); \
	const __m256 hit8 = _mm256_and_ps( \
		_mm256_and_ps( _mm256_cmp_ps( bu8, zero8, _CMP_GE_OQ ), _mm256_cmp_ps( bv8, zero8, _CMP_GE_OQ ) ), \
		_mm256_and_ps( _mm256_cmp_ps( _mm256_add_ps( bu8, bv8 ), one8, _CMP_LE_OQ ), \
		_mm256_and_ps( _mm256_cmp_ps( bt8, zero8, _CMP_GT_OQ ), _mm256_cmp_ps( bt8, tcur8, _CMP_LT_OQ ) ) ) );

template <bool posX, bool posY, bool posZ>
static int32_t tinybvh_bundle_bvh4( const BVH4_CPU& bvh, RayBundle& b, const uint32_t* pk, const uint32_t n )
{
	using BVHNode = BVH4_CPU::BVHNode;
	// an opacity map needs a per-lane gather in the leaf; not worth vectorizing.
	if (bvh.opmap) ISUNLIKELY return tinybvh_packets_per_ray( &bvh, b, pk, n );
	ALIGNED( 64 ) uint32_t nodeStack[TINYBVH_STACK_SIZE * 3 + 4];
	ALIGNED( 64 ) uint32_t infoStack[TINYBVH_STACK_SIZE * 3 + 4];	// parent << 2 | lane
	ALIGNED( 64 ) uint32_t fpiStack[TINYBVH_STACK_SIZE * 3 + 4];	// packet to test first
	ALIGNED( 64 ) float rx[TINYBVH_BUNDLE_RAYS], ry[TINYBVH_BUNDLE_RAYS], rz[TINYBVH_BUNDLE_RAYS];
	int32_t stackPtr = 0, steps = 0;
	uint32_t nodeIdx = 0, fpi = 0, active = (1u << n) - 1;
	const __m256 zero8 = _mm256_setzero_ps(), one8 = _mm256_set1_ps( 1.0f );
	const __m256i instIdx8 = _mm256_set1_epi32( (int32_t)b.instIdx );
	TINYBVH_BUNDLE_SETUP
	const __m128 rdxMin4 = tinybvh_lo4( tinybvh_hmin8( rdxMin ) ), rdxMax4 = tinybvh_lo4( tinybvh_hmax8( rdxMax ) );
	const __m128 rdyMin4 = tinybvh_lo4( tinybvh_hmin8( rdyMin ) ), rdyMax4 = tinybvh_lo4( tinybvh_hmax8( rdyMax ) );
	const __m128 rdzMin4 = tinybvh_lo4( tinybvh_hmin8( rdzMin ) ), rdzMax4 = tinybvh_lo4( tinybvh_hmax8( rdzMax ) );
	const __m128 rxMin4 = tinybvh_lo4( tinybvh_hmin8( rxMin ) ), rxMax4 = tinybvh_lo4( tinybvh_hmax8( rxMax ) );
	const __m128 ryMin4 = tinybvh_lo4( tinybvh_hmin8( ryMin ) ), ryMax4 = tinybvh_lo4( tinybvh_hmax8( ryMax ) );
	const __m128 rzMin4 = tinybvh_lo4( tinybvh_hmin8( rzMin ) ), rzMax4 = tinybvh_lo4( tinybvh_hmax8( rzMax ) );
	const __m128 zero4 = _mm_setzero_ps();
	__m128 far4 = tinybvh_lo4( tinybvh_hmax8( tmax8 ) );
	while (1)
	{
		if (!(nodeIdx & BVH4_CPU::LEAF_BIT)) ISLIKELY
		{
			steps++;
			const BVHNode* n = (const BVHNode*)(bvh.bvh4Data + nodeIdx);
			TINYBVH_INTERVAL_SLAB_TEST
			if (mask)
			{
				constexpr int signShift = 2 * ((posX ? 1 : 0) + (posY ? 2 : 0) + (posZ ? 4 : 0));
				for (uint32_t s = 0; s < 4; s++)
				{
					const uint32_t lane = (n->perm[s] >> signShift) & 3;
					if (!((mask >> lane) & 1)) continue;
					nodeStack[stackPtr] = n->child[lane];
					infoStack[stackPtr] = (nodeIdx << 2) | lane;
					fpiStack[stackPtr++] = fpi;
				}
				BVH_FATAL_ERROR_IF( stackPtr > TINYBVH_STACK_SIZE * 3, "BVH4_CPU::IntersectPackets, traversal stack overflow." );
			}
		}
		else
		{
			// 'active' holds the packets that entered this leaf's box; skip the rest.
			const BVHTri4Leaf* leaf = (const BVHTri4Leaf*)(bvh.bvh4Data + (nodeIdx & 0x1fffffff));
			const uint32_t triCount = tinybvh_leaftris( leaf->primIdx );
			bool anyHit = false;
			for (uint32_t a = active; a; a &= a - 1)
			{
				const uint32_t l = pk[tinybvh_ctz( a )] * 8;
				const __m256 ox8 = _mm256_load_ps( b.ox + l ), oy8 = _mm256_load_ps( b.oy + l ), oz8 = _mm256_load_ps( b.oz + l );
				const __m256 dx8 = _mm256_load_ps( b.dx + l ), dy8 = _mm256_load_ps( b.dy + l ), dz8 = _mm256_load_ps( b.dz + l );
				__m256 tcur8 = _mm256_load_ps( b.t + l );
				__m256 u8 = _mm256_undefined_ps(), v8 = _mm256_undefined_ps();
				__m256 prim8 = _mm256_undefined_ps(), inst8 = _mm256_undefined_ps();
				bool packetHit = false;
				for (uint32_t i = 0; i < triCount; i++)
				{
					TINYBVH_PACKET_TRI( i )
					if (!_mm256_movemask_ps( hit8 )) continue;
					if (!packetHit)
					{
						u8 = _mm256_load_ps( b.u + l ), v8 = _mm256_load_ps( b.v + l );
						prim8 = _mm256_castsi256_ps( _mm256_load_si256( (const __m256i*)(b.prim + l) ) );
						inst8 = _mm256_castsi256_ps( _mm256_load_si256( (const __m256i*)(b.inst + l) ) );
						packetHit = true;
					}
					tcur8 = _mm256_blendv_ps( tcur8, bt8, hit8 );
					u8 = _mm256_blendv_ps( u8, bu8, hit8 ), v8 = _mm256_blendv_ps( v8, bv8, hit8 );
					prim8 = _mm256_blendv_ps( prim8, _mm256_castsi256_ps( _mm256_set1_epi32( (int32_t)leaf->primIdx[i] ) ), hit8 );
					inst8 = _mm256_blendv_ps( inst8, _mm256_castsi256_ps( instIdx8 ), hit8 );
				}
				if (packetHit)
				{
					_mm256_store_ps( b.t + l, tcur8 );
					_mm256_store_ps( b.u + l, u8 ), _mm256_store_ps( b.v + l, v8 );
					_mm256_store_si256( (__m256i*)(b.prim + l), _mm256_castps_si256( prim8 ) );
					_mm256_store_si256( (__m256i*)(b.inst + l), _mm256_castps_si256( inst8 ) );
					anyHit = true;
				}
			}
			// the interval ray only has to reach the farthest ray still looking.
			if (anyHit)
			{
				__m256 m8 = _mm256_load_ps( b.t + pk[0] * 8 );
				for (uint32_t i = 1; i < n; i++) m8 = _mm256_max_ps( m8, _mm256_load_ps( b.t + pk[i] * 8 ) );
				far4 = tinybvh_lo4( tinybvh_hmax8( m8 ) );
			}
		}
		// Pop until some ray of some packet enters the child.
		while (1)
		{
			if (!stackPtr) ISUNLIKELY return steps;
			const uint32_t info = infoStack[--stackPtr];
			const uint32_t child = nodeStack[stackPtr], first = fpiStack[stackPtr];
			const bool isLeaf = (child & BVH4_CPU::LEAF_BIT) != 0;
			const BVHNode* p = (const BVHNode*)(bvh.bvh4Data + (info >> 2));
			const uint32_t sel = info & 3;
			const __m256 px1 = _mm256_broadcast_ss( (posX ? p->xmin : p->xmax) + sel );
			const __m256 py1 = _mm256_broadcast_ss( (posY ? p->ymin : p->ymax) + sel );
			const __m256 pz1 = _mm256_broadcast_ss( (posZ ? p->zmin : p->zmax) + sel );
			const __m256 px2 = _mm256_broadcast_ss( (posX ? p->xmax : p->xmin) + sel );
			const __m256 py2 = _mm256_broadcast_ss( (posY ? p->ymax : p->ymin) + sel );
			const __m256 pz2 = _mm256_broadcast_ss( (posZ ? p->zmax : p->zmin) + sel );
			uint32_t entering = 0, firstHit = 0, i = first;
			do
			{
				if (TINYBVH_PACKET_BOX_TEST( pk[i] * 8 ))
				{
					if (!entering) firstHit = i;
					entering |= 1u << i;
					if (!isLeaf) break;
				}
				if (++i == n) i = 0;
			}
			while (i != first);
			if (!entering) continue;
			nodeIdx = child, fpi = firstHit, active = entering;
			break;
		}
	}
}

// Ray bundle traversal - TLAS level.
template <bool posX, bool posY, bool posZ>
static int32_t tinybvh_bundle_tlas( const BVH& bvh, RayBundle& b, const uint32_t* pk, const uint32_t n )
{
	using BVHNode = BVH::BVHNode;
	uint32_t nodeStack[TINYBVH_STACK_SIZE], fpiStack[TINYBVH_STACK_SIZE];
	ALIGNED( 64 ) float rx[TINYBVH_BUNDLE_RAYS], ry[TINYBVH_BUNDLE_RAYS], rz[TINYBVH_BUNDLE_RAYS];
	RayBundle obj;
	uint32_t visiting[TINYBVH_BUNDLE_PACKETS], objPk[TINYBVH_BUNDLE_PACKETS];
	int32_t stackPtr = 0, steps = 0;
	uint32_t nodeIdx = 0, fpi = 0, active = (1u << n) - 1;
	const __m256 zero8 = _mm256_setzero_ps();
	TINYBVH_BUNDLE_SETUP
	// the TLAS node test is scalar, so reduce the interval ray all the way down.
	float rdMin[3], rdMax[3], roMin[3], roMax[3], tfar;
	_mm_store_ss( rdMin + 0, tinybvh_lo4( tinybvh_hmin8( rdxMin ) ) ), _mm_store_ss( rdMax + 0, tinybvh_lo4( tinybvh_hmax8( rdxMax ) ) );
	_mm_store_ss( rdMin + 1, tinybvh_lo4( tinybvh_hmin8( rdyMin ) ) ), _mm_store_ss( rdMax + 1, tinybvh_lo4( tinybvh_hmax8( rdyMax ) ) );
	_mm_store_ss( rdMin + 2, tinybvh_lo4( tinybvh_hmin8( rdzMin ) ) ), _mm_store_ss( rdMax + 2, tinybvh_lo4( tinybvh_hmax8( rdzMax ) ) );
	_mm_store_ss( roMin + 0, tinybvh_lo4( tinybvh_hmin8( rxMin ) ) ), _mm_store_ss( roMax + 0, tinybvh_lo4( tinybvh_hmax8( rxMax ) ) );
	_mm_store_ss( roMin + 1, tinybvh_lo4( tinybvh_hmin8( ryMin ) ) ), _mm_store_ss( roMax + 1, tinybvh_lo4( tinybvh_hmax8( ryMax ) ) );
	_mm_store_ss( roMin + 2, tinybvh_lo4( tinybvh_hmin8( rzMin ) ) ), _mm_store_ss( roMax + 2, tinybvh_lo4( tinybvh_hmax8( rzMax ) ) );
	_mm_store_ss( &tfar, tinybvh_lo4( tinybvh_hmax8( tmax8 ) ) );
	while (1)
	{
		const BVHNode& node = bvh.bvhNode[nodeIdx];
		steps++;
		if (!node.isLeaf())
		{
			const uint32_t c1 = (uint32_t)node.leftFirst, c2 = c1 + 1;
			float d1 = tinybvh_interval_slab<posX, posY, posZ>( bvh.bvhNode[c1].aabbMin, bvh.bvhNode[c1].aabbMax, rdMin, rdMax, roMin, roMax, tfar );
			float d2 = tinybvh_interval_slab<posX, posY, posZ>( bvh.bvhNode[c2].aabbMin, bvh.bvhNode[c2].aabbMax, rdMin, rdMax, roMin, roMax, tfar );
			uint32_t n1 = c1, n2 = c2;
			if (d1 < d2) tinybvh_swap( d1, d2 ), tinybvh_swap( n1, n2 );
			if (d1 < BVH_FAR) nodeStack[stackPtr] = n1, fpiStack[stackPtr++] = fpi;	// farthest first
			if (d2 < BVH_FAR) nodeStack[stackPtr] = n2, fpiStack[stackPtr++] = fpi;
			BVH_FATAL_ERROR_IF( stackPtr > TINYBVH_STACK_SIZE - 2, "BVH::IntersectPacketsTLAS, traversal stack overflow." );
		}
		else
		{
			bool anyHit = false;
			for (uint32_t i = 0; i < node.triCount; i++)
			{
				const uint32_t instIdx = bvh.primIdx[node.leftFirst + i];
				const BLASInstance& inst = bvh.instList[instIdx];
				if (!(inst.mask & b.mask)) continue;
				uint32_t vn = 0;
				for (uint32_t a = active; a; a &= a - 1) visiting[vn++] = pk[tinybvh_ctz( a )];
				b.TransformTo( obj, visiting, vn, inst.invTransform, instIdx << impl::bvh_inst_shift<uint32_t> );
				for (uint32_t j = 0; j < vn; j++) objPk[j] = j;
				steps += tinybvh_trace_packets( bvh.blasList[inst.blasIdx], obj, objPk, vn );
				anyHit |= b.MergeHits( obj );
			}
			if (anyHit)
			{
				__m256 m8 = _mm256_load_ps( b.t + pk[0] * 8 );
				for (uint32_t i = 1; i < n; i++) m8 = _mm256_max_ps( m8, _mm256_load_ps( b.t + pk[i] * 8 ) );
				_mm_store_ss( &tfar, tinybvh_lo4( tinybvh_hmax8( m8 ) ) );
			}
		}
		while (1)
		{
			if (!stackPtr) ISUNLIKELY return steps;
			const uint32_t entry = nodeStack[--stackPtr], first = fpiStack[stackPtr];
			const BVHNode& cand = bvh.bvhNode[entry];
			const bool isLeaf = cand.isLeaf();
			const __m256 px1 = _mm256_set1_ps( posX ? cand.aabbMin.x : cand.aabbMax.x );
			const __m256 py1 = _mm256_set1_ps( posY ? cand.aabbMin.y : cand.aabbMax.y );
			const __m256 pz1 = _mm256_set1_ps( posZ ? cand.aabbMin.z : cand.aabbMax.z );
			const __m256 px2 = _mm256_set1_ps( posX ? cand.aabbMax.x : cand.aabbMin.x );
			const __m256 py2 = _mm256_set1_ps( posY ? cand.aabbMax.y : cand.aabbMin.y );
			const __m256 pz2 = _mm256_set1_ps( posZ ? cand.aabbMax.z : cand.aabbMin.z );
			uint32_t entering = 0, firstHit = 0, i = first;
			do
			{
				if (TINYBVH_PACKET_BOX_TEST( pk[i] * 8 ))
				{
					if (!entering) firstHit = i;
					entering |= 1u << i;
					// a leaf decides which packets get transformed and pushed
					// through a blas, which is far more expensive than the scan.
					if (!isLeaf) break;
				}
				if (++i == n) i = 0;
			}
			while (i != first);
			if (!entering) continue;
			nodeIdx = entry, fpi = firstHit, active = entering;
			break;
		}
	}
}

// Octant dispatch for the bundle kernels.
#define OCTANT_DISPATCH_BUNDLE( kernel, o, ... ) \
	switch (o) { \
	case 1: cost += kernel<true, false, false>( __VA_ARGS__ ); break; \
	case 2: cost += kernel<false, true, false>( __VA_ARGS__ ); break; \
	case 3: cost += kernel<true, true, false>( __VA_ARGS__ ); break; \
	case 4: cost += kernel<false, false, true>( __VA_ARGS__ ); break; \
	case 5: cost += kernel<true, false, true>( __VA_ARGS__ ); break; \
	case 6: cost += kernel<false, true, true>( __VA_ARGS__ ); break; \
	case 7: cost += kernel<true, true, true>( __VA_ARGS__ ); break; \
	default: cost += kernel<false, false, false>( __VA_ARGS__ ); break; }

// Group packets pk[0..n) by octant and hand each group to 'bvh'.
static int32_t tinybvh_trace_packets( const BVHBase* bvh, RayBundle& b, const uint32_t* pk, const uint32_t n )
{
	const BVH* tlas = (bvh->layout == LAYOUT_BVH && ((const BVH*)bvh)->isTLAS()) ? (const BVH*)bvh : 0;
	const BVH4_CPU* blas4 = bvh->layout == LAYOUT_BVH4_CPU ? (const BVH4_CPU*)bvh : 0;
	if (!tlas && !blas4) return tinybvh_packets_per_ray( bvh, b, pk, n );
	uint32_t bucket[8][TINYBVH_BUNDLE_PACKETS], bn[8] = {}, loose[TINYBVH_BUNDLE_PACKETS], ln = 0;
	for (uint32_t i = 0; i < n; i++)
	{
		const int32_t o = b.Octant( pk[i] );
		if (o < 0) loose[ln++] = pk[i]; else bucket[o][bn[o]++] = pk[i];
	}
	int32_t cost = 0;
	for (uint32_t o = 0; o < 8; o++) if (bn[o])
	{
		if (tlas) OCTANT_DISPATCH_BUNDLE( tinybvh_bundle_tlas, o, *tlas, b, bucket[o], bn[o] )
		else OCTANT_DISPATCH_BUNDLE( tinybvh_bundle_bvh4, o, *blas4, b, bucket[o], bn[o] )
	}
	if (ln) cost += tinybvh_packets_per_ray( bvh, b, loose, ln );
	return cost;
}

// Trace TINYBVH_BUNDLE_RAYS rays as one WiVeC bundle.
static int32_t tinybvh_intersect_bundle( const BVHBase* bvh, Ray* rays )
{
	RayBundle b;
	for (uint32_t i = 0; i < TINYBVH_BUNDLE_RAYS; i++) b.SetRay( i, rays[i] );
	b.packets = TINYBVH_BUNDLE_PACKETS, b.instIdx = 0, b.mask = rays[0].mask;
	uint32_t pk[TINYBVH_BUNDLE_PACKETS];
	for (uint32_t p = 0; p < TINYBVH_BUNDLE_PACKETS; p++) pk[p] = p;
	const int32_t cost = tinybvh_trace_packets( bvh, b, pk, TINYBVH_BUNDLE_PACKETS );
	for (uint32_t i = 0; i < TINYBVH_BUNDLE_RAYS; i++) b.GetHit( i, rays[i] );
	return cost;
}

template <> int32_t impl::BVH4_CPU<float, uint32_t>::IntersectBundle( Ray* rays ) const
{
	return tinybvh_intersect_bundle( this, rays );
}

template <> int32_t impl::BVH<float, uint32_t>::IntersectBundle( Ray* rays ) const
{
	BVH_FATAL_ERROR_IF( !isTLAS(), "BVH::IntersectBundle, not a TLAS." );
	return tinybvh_intersect_bundle( this, rays );
}

// Bundle traversal for occlusion rays.

static int32_t tinybvh_occlude_packets( const BVHBase* bvh, RayBundle& b, const uint32_t* pk, const uint32_t n );

static bool tinybvh_blas_occluded( const BVHBase* blas, const Ray& ray )
{
	if (blas->layout == LAYOUT_BVH) return ((BVH*)blas)->IsOccluded( ray );
#ifdef ENABLE_VOXEL_SUPPORT
	if (blas->layout == LAYOUT_VOXELSET) return ((VoxelSet*)blas)->IsOccluded( ray );
#endif
	if (blas->layout == LAYOUT_BVH4_CPU) return ((BVH4_CPU*)blas)->IsOccluded( ray );
	if (blas->layout == LAYOUT_BVH8_AVX2) return ((BVH8_CPU*)blas)->IsOccluded( ray );
	assert( !"unsupported BLAS layout" );
	return false;
}

static int32_t tinybvh_packets_occluded_per_ray( const BVHBase* bvh, RayBundle& b, const uint32_t* pk, const uint32_t n )
{
	int32_t cost = 0;
	for (uint32_t i = 0; i < n; i++)
	{
		const uint32_t p = pk[i];
		for (uint32_t k = 0; k < 8; k++)
		{
			if (b.occluded[p] & (1u << k)) continue;
			const uint32_t l = p * 8 + k;
			Ray ray( bvhvec3( b.ox[l], b.oy[l], b.oz[l] ), bvhvec3( b.dx[l], b.dy[l], b.dz[l] ), b.t[l] );
			ray.instIdx = b.instIdx, ray.mask = b.mask;
			cost++;
			if (tinybvh_blas_occluded( bvh, ray )) b.occluded[p] |= (uint8_t)(1u << k);
		}
	}
	return cost;
}

// Eight lanes of an 8-bit mask, as a blend mask.
static TINYBVH_FORCEINLINE __m256 tinybvh_lanemask8( const uint32_t m )
{
	const __m256i bits = _mm256_setr_epi32( 1, 2, 4, 8, 16, 32, 64, 128 );
	return _mm256_castsi256_ps( _mm256_cmpeq_epi32( _mm256_and_si256( _mm256_set1_epi32( (int32_t)m ), bits ), bits ) );
}

// Moeller-Trumbore for eight rays and one triangle, without the reciprocal.
#define TINYBVH_PACKET_TRI_ANY( i ) \
	const __m256 v0x8 = _mm256_broadcast_ss( leaf->v0x + (i) ), v0y8 = _mm256_broadcast_ss( leaf->v0y + (i) ); \
	const __m256 v0z8 = _mm256_broadcast_ss( leaf->v0z + (i) ), e1x8 = _mm256_broadcast_ss( leaf->e1x + (i) ); \
	const __m256 e1y8 = _mm256_broadcast_ss( leaf->e1y + (i) ), e1z8 = _mm256_broadcast_ss( leaf->e1z + (i) ); \
	const __m256 e2x8 = _mm256_broadcast_ss( leaf->e2x + (i) ), e2y8 = _mm256_broadcast_ss( leaf->e2y + (i) ); \
	const __m256 e2z8 = _mm256_broadcast_ss( leaf->e2z + (i) ); \
	const __m256 hx8 = _mm256_fmsub_ps( dy8, e2z8, _mm256_mul_ps( dz8, e2y8 ) ); \
	const __m256 hy8 = _mm256_fmsub_ps( dz8, e2x8, _mm256_mul_ps( dx8, e2z8 ) ); \
	const __m256 hz8 = _mm256_fmsub_ps( dx8, e2y8, _mm256_mul_ps( dy8, e2x8 ) ); \
	const __m256 sx8 = _mm256_sub_ps( ox8, v0x8 ), sy8 = _mm256_sub_ps( oy8, v0y8 ), sz8 = _mm256_sub_ps( oz8, v0z8 ); \
	const __m256 det8 = _mm256_fmadd_ps( e1z8, hz8, _mm256_fmadd_ps( e1x8, hx8, _mm256_mul_ps( e1y8, hy8 ) ) ); \
	const __m256 qz8 = _mm256_fmsub_ps( sx8, e1y8, _mm256_mul_ps( sy8, e1x8 ) ); \
	const __m256 qx8 = _mm256_fmsub_ps( sy8, e1z8, _mm256_mul_ps( sz8, e1y8 ) ); \
	const __m256 qy8 = _mm256_fmsub_ps( sz8, e1x8, _mm256_mul_ps( sx8, e1z8 ) ); \
	const __m256 nu8 = _mm256_fmadd_ps( sz8, hz8, _mm256_fmadd_ps( sx8, hx8, _mm256_mul_ps( sy8, hy8 ) ) ); \
	const __m256 nv8 = _mm256_fmadd_ps( dz8, qz8, _mm256_fmadd_ps( dx8, qx8, _mm256_mul_ps( dy8, qy8 ) ) ); \
	const __m256 nt8 = _mm256_fmadd_ps( e2z8, qz8, _mm256_fmadd_ps( e2x8, qx8, _mm256_mul_ps( e2y8, qy8 ) ) ); \
	const __m256 dsign8 = _mm256_and_ps( det8, sign8 ), adet8 = _mm256_andnot_ps( sign8, det8 ); \
	const __m256 bu8 = _mm256_xor_ps( nu8, dsign8 ), bv8 = _mm256_xor_ps( nv8, dsign8 ), ta8 = _mm256_xor_ps( nt8, dsign8 ); \
	const __m256 hit8 = _mm256_and_ps( \
		_mm256_and_ps( _mm256_cmp_ps( bu8, zero8, _CMP_GE_OQ ), _mm256_cmp_ps( bv8, zero8, _CMP_GE_OQ ) ), \
		_mm256_and_ps( _mm256_cmp_ps( _mm256_add_ps( bu8, bv8 ), adet8, _CMP_LE_OQ ), _mm256_and_ps( \
		_mm256_cmp_ps( ta8, zero8, _CMP_GT_OQ ), _mm256_cmp_ps( ta8, _mm256_mul_ps( tcur8, adet8 ), _CMP_LT_OQ ) ) ) );

// Deferred box test against the working tmax rather than the bundle's.
#define TINYBVH_PACKET_BOX_TEST_ANY( i ) _mm256_movemask_ps( _mm256_cmp_ps( _mm256_max_ps( _mm256_max_ps( \
	_mm256_fmsub_ps( px1, _mm256_load_ps( b.rdx + pk[i] * 8 ), _mm256_load_ps( rx + pk[i] * 8 ) ), _mm256_fmsub_ps( py1, \
	_mm256_load_ps( b.rdy + pk[i] * 8 ), _mm256_load_ps( ry + pk[i] * 8 ) ) ), _mm256_max_ps( _mm256_fmsub_ps( pz1, \
	_mm256_load_ps( b.rdz + pk[i] * 8 ), _mm256_load_ps( rz + pk[i] * 8 ) ), zero8 ) ), _mm256_min_ps( _mm256_min_ps( \
	_mm256_fmsub_ps( px2, _mm256_load_ps( b.rdx + pk[i] * 8 ), _mm256_load_ps( rx + pk[i] * 8 ) ), _mm256_fmsub_ps( py2, \
	_mm256_load_ps( b.rdy + pk[i] * 8 ), _mm256_load_ps( ry + pk[i] * 8 ) ) ), _mm256_min_ps( _mm256_fmsub_ps( pz2, \
	_mm256_load_ps( b.rdz + pk[i] * 8 ), _mm256_load_ps( rz + pk[i] * 8 ) ), tcur[i] ) ), _CMP_LE_OQ ) )

// Retire the lanes an earlier instance already resolved, and fold the interval
// ray over what is left.
#define TINYBVH_BUNDLE_SETUP_ANY \
	__m256 rdxMin, rdxMax, rdyMin, rdyMax, rdzMin, rdzMax; \
	__m256 rxMin, rxMax, ryMin, ryMax, rzMin, rzMax, tmax8 = minusOne8; \
	for (uint32_t i = 0; i < n; i++) { \
		const uint32_t l = pk[i] * 8; \
		const __m256 rdx = _mm256_load_ps( b.rdx + l ), rdy = _mm256_load_ps( b.rdy + l ), rdz = _mm256_load_ps( b.rdz + l ); \
		const __m256 qx = _mm256_mul_ps( _mm256_load_ps( b.ox + l ), rdx ); \
		const __m256 qy = _mm256_mul_ps( _mm256_load_ps( b.oy + l ), rdy ); \
		const __m256 qz = _mm256_mul_ps( _mm256_load_ps( b.oz + l ), rdz ); \
		_mm256_store_ps( rx + l, qx ), _mm256_store_ps( ry + l, qy ), _mm256_store_ps( rz + l, qz ); \
		tcur[i] = _mm256_blendv_ps( _mm256_load_ps( b.t + l ), minusOne8, tinybvh_lanemask8( b.occluded[pk[i]] ) ); \
		if (b.occluded[pk[i]] != 0xff) alive |= 1u << i; \
		if (i == 0) rdxMin = rdxMax = rdx, rdyMin = rdyMax = rdy, rdzMin = rdzMax = rdz, \
			rxMin = rxMax = qx, ryMin = ryMax = qy, rzMin = rzMax = qz; \
		else rdxMin = _mm256_min_ps( rdxMin, rdx ), rdxMax = _mm256_max_ps( rdxMax, rdx ), \
			rdyMin = _mm256_min_ps( rdyMin, rdy ), rdyMax = _mm256_max_ps( rdyMax, rdy ), \
			rdzMin = _mm256_min_ps( rdzMin, rdz ), rdzMax = _mm256_max_ps( rdzMax, rdz ), \
			rxMin = _mm256_min_ps( rxMin, qx ), rxMax = _mm256_max_ps( rxMax, qx ), \
			ryMin = _mm256_min_ps( ryMin, qy ), ryMax = _mm256_max_ps( ryMax, qy ), \
			rzMin = _mm256_min_ps( rzMin, qz ), rzMax = _mm256_max_ps( rzMax, qz ); \
		tmax8 = _mm256_max_ps( tmax8, tcur[i] ); }

template <bool posX, bool posY, bool posZ>
static int32_t tinybvh_occluded_bvh4( const BVH4_CPU& bvh, RayBundle& b, const uint32_t* pk, const uint32_t n )
{
	using BVHNode = BVH4_CPU::BVHNode;
	if (bvh.opmap) ISUNLIKELY return tinybvh_packets_occluded_per_ray( &bvh, b, pk, n );
	ALIGNED( 64 ) uint32_t nodeStack[TINYBVH_STACK_SIZE * 3 + 4];
	ALIGNED( 64 ) uint32_t infoStack[TINYBVH_STACK_SIZE * 3 + 4];
	ALIGNED( 64 ) uint32_t fpiStack[TINYBVH_STACK_SIZE * 3 + 4];
	ALIGNED( 64 ) float rx[TINYBVH_BUNDLE_RAYS], ry[TINYBVH_BUNDLE_RAYS], rz[TINYBVH_BUNDLE_RAYS];
	__m256 tcur[TINYBVH_BUNDLE_PACKETS];	// working tmax, -1 for a retired lane
	int32_t stackPtr = 0, steps = 0;
	uint32_t nodeIdx = 0, fpi = 0, alive = 0, active;
	const __m256 zero8 = _mm256_setzero_ps(), minusOne8 = _mm256_set1_ps( -1.0f );
	const __m256 sign8 = _mm256_set1_ps( -0.0f );
	TINYBVH_BUNDLE_SETUP_ANY
	if (!alive) return 0;
	active = alive;
	const __m128 rdxMin4 = tinybvh_lo4( tinybvh_hmin8( rdxMin ) ), rdxMax4 = tinybvh_lo4( tinybvh_hmax8( rdxMax ) );
	const __m128 rdyMin4 = tinybvh_lo4( tinybvh_hmin8( rdyMin ) ), rdyMax4 = tinybvh_lo4( tinybvh_hmax8( rdyMax ) );
	const __m128 rdzMin4 = tinybvh_lo4( tinybvh_hmin8( rdzMin ) ), rdzMax4 = tinybvh_lo4( tinybvh_hmax8( rdzMax ) );
	const __m128 rxMin4 = tinybvh_lo4( tinybvh_hmin8( rxMin ) ), rxMax4 = tinybvh_lo4( tinybvh_hmax8( rxMax ) );
	const __m128 ryMin4 = tinybvh_lo4( tinybvh_hmin8( ryMin ) ), ryMax4 = tinybvh_lo4( tinybvh_hmax8( ryMax ) );
	const __m128 rzMin4 = tinybvh_lo4( tinybvh_hmin8( rzMin ) ), rzMax4 = tinybvh_lo4( tinybvh_hmax8( rzMax ) );
	const __m128 zero4 = _mm_setzero_ps();
	__m128 far4 = tinybvh_lo4( tinybvh_hmax8( tmax8 ) );
	while (1)
	{
		if (!(nodeIdx & BVH4_CPU::LEAF_BIT)) ISLIKELY
		{
			steps++;
			const BVHNode* n = (const BVHNode*)(bvh.bvh4Data + nodeIdx);
			TINYBVH_INTERVAL_SLAB_TEST
			if (mask)
			{
				// no ordering: any occluder will do, so perm goes unread.
				for (uint32_t lane = 0; lane < 4; lane++)
				{
					if (!((mask >> lane) & 1)) continue;
					nodeStack[stackPtr] = n->child[lane];
					infoStack[stackPtr] = (nodeIdx << 2) | lane;
					fpiStack[stackPtr++] = fpi;
				}
				BVH_FATAL_ERROR_IF( stackPtr > TINYBVH_STACK_SIZE * 3,
					"BVH4_CPU::IsOccludedBundle, traversal stack overflow." );
			}
		}
		else
		{
			const BVHTri4Leaf* leaf = (const BVHTri4Leaf*)(bvh.bvh4Data + (nodeIdx & 0x1fffffff));
			const uint32_t triCount = tinybvh_leaftris( leaf->primIdx );
			bool retired = false;
			// 'active' was computed at the pop; a packet may have finished since.
			for (uint32_t a = active & alive; a; a &= a - 1)
			{
				const uint32_t i = tinybvh_ctz( a ), l = pk[i] * 8;
				const __m256 ox8 = _mm256_load_ps( b.ox + l ), oy8 = _mm256_load_ps( b.oy + l ), oz8 = _mm256_load_ps( b.oz + l );
				const __m256 dx8 = _mm256_load_ps( b.dx + l ), dy8 = _mm256_load_ps( b.dy + l ), dz8 = _mm256_load_ps( b.dz + l );
				__m256 tcur8 = tcur[i];
				for (uint32_t j = 0; j < triCount; j++)
				{
					TINYBVH_PACKET_TRI_ANY( j )
					const uint32_t hit = (uint32_t)_mm256_movemask_ps( hit8 );
					if (!hit) continue;
					b.occluded[pk[i]] |= (uint8_t)hit, retired = true;
					tcur8 = _mm256_blendv_ps( tcur8, minusOne8, hit8 );
					if (b.occluded[pk[i]] == 0xff) { alive &= ~(1u << i); break; }
				}
				tcur[i] = tcur8;
			}
			if (!alive) ISUNLIKELY return steps;
			// the interval ray only has to reach the farthest live ray.
			if (retired)
			{
				__m256 m8 = minusOne8;
				for (uint32_t a = alive; a; a &= a - 1) m8 = _mm256_max_ps( m8, tcur[tinybvh_ctz( a )] );
				far4 = tinybvh_lo4( tinybvh_hmax8( m8 ) );
			}
		}
		while (1)
		{
			if (!stackPtr) ISUNLIKELY return steps;
			const uint32_t info = infoStack[--stackPtr];
			const uint32_t child = nodeStack[stackPtr], first = fpiStack[stackPtr];
			const bool isLeaf = (child & BVH4_CPU::LEAF_BIT) != 0;
			const BVHNode* p = (const BVHNode*)(bvh.bvh4Data + (info >> 2));
			const uint32_t sel = info & 3;
			const __m256 px1 = _mm256_broadcast_ss( (posX ? p->xmin : p->xmax) + sel );
			const __m256 py1 = _mm256_broadcast_ss( (posY ? p->ymin : p->ymax) + sel );
			const __m256 pz1 = _mm256_broadcast_ss( (posZ ? p->zmin : p->zmax) + sel );
			const __m256 px2 = _mm256_broadcast_ss( (posX ? p->xmax : p->xmin) + sel );
			const __m256 py2 = _mm256_broadcast_ss( (posY ? p->ymax : p->ymin) + sel );
			const __m256 pz2 = _mm256_broadcast_ss( (posZ ? p->zmax : p->zmin) + sel );
			uint32_t entering = 0, firstHit = 0, i = first;
			do
			{
				// a packet that finished is not tested again.
				if ((alive >> i) & 1) if (TINYBVH_PACKET_BOX_TEST_ANY( i ))
				{
					if (!entering) firstHit = i;
					entering |= 1u << i;
					if (!isLeaf) break;
				}
				if (++i == n) i = 0;
			}
			while (i != first);
			if (!entering) continue;
			nodeIdx = child, fpi = firstHit, active = entering;
			break;
		}
	}
}

template <bool posX, bool posY, bool posZ>
static int32_t tinybvh_occluded_tlas( const BVH& bvh, RayBundle& b, const uint32_t* pk, const uint32_t n )
{
	using BVHNode = BVH::BVHNode;
	uint32_t nodeStack[TINYBVH_STACK_SIZE], fpiStack[TINYBVH_STACK_SIZE];
	ALIGNED( 64 ) float rx[TINYBVH_BUNDLE_RAYS], ry[TINYBVH_BUNDLE_RAYS], rz[TINYBVH_BUNDLE_RAYS];
	__m256 tcur[TINYBVH_BUNDLE_PACKETS];
	RayBundle obj;
	uint32_t visiting[TINYBVH_BUNDLE_PACKETS], objPk[TINYBVH_BUNDLE_PACKETS];
	int32_t stackPtr = 0, steps = 0;
	uint32_t nodeIdx = 0, fpi = 0, alive = 0, active;
	const __m256 zero8 = _mm256_setzero_ps(), minusOne8 = _mm256_set1_ps( -1.0f );
	TINYBVH_BUNDLE_SETUP_ANY
	if (!alive) return 0;
	active = alive;
	float rdMin[3], rdMax[3], roMin[3], roMax[3], tfar;
	_mm_store_ss( rdMin + 0, tinybvh_lo4( tinybvh_hmin8( rdxMin ) ) ), _mm_store_ss( rdMax + 0, tinybvh_lo4( tinybvh_hmax8( rdxMax ) ) );
	_mm_store_ss( rdMin + 1, tinybvh_lo4( tinybvh_hmin8( rdyMin ) ) ), _mm_store_ss( rdMax + 1, tinybvh_lo4( tinybvh_hmax8( rdyMax ) ) );
	_mm_store_ss( rdMin + 2, tinybvh_lo4( tinybvh_hmin8( rdzMin ) ) ), _mm_store_ss( rdMax + 2, tinybvh_lo4( tinybvh_hmax8( rdzMax ) ) );
	_mm_store_ss( roMin + 0, tinybvh_lo4( tinybvh_hmin8( rxMin ) ) ), _mm_store_ss( roMax + 0, tinybvh_lo4( tinybvh_hmax8( rxMax ) ) );
	_mm_store_ss( roMin + 1, tinybvh_lo4( tinybvh_hmin8( ryMin ) ) ), _mm_store_ss( roMax + 1, tinybvh_lo4( tinybvh_hmax8( ryMax ) ) );
	_mm_store_ss( roMin + 2, tinybvh_lo4( tinybvh_hmin8( rzMin ) ) ), _mm_store_ss( roMax + 2, tinybvh_lo4( tinybvh_hmax8( rzMax ) ) );
	_mm_store_ss( &tfar, tinybvh_lo4( tinybvh_hmax8( tmax8 ) ) );
	while (1)
	{
		const BVHNode& node = bvh.bvhNode[nodeIdx];
		steps++;
		if (!node.isLeaf())
		{
			const uint32_t c1 = (uint32_t)node.leftFirst, c2 = c1 + 1;
			const float d1 = tinybvh_interval_slab<posX, posY, posZ>( bvh.bvhNode[c1].aabbMin, bvh.bvhNode[c1].aabbMax, rdMin, rdMax, roMin, roMax, tfar );
			const float d2 = tinybvh_interval_slab<posX, posY, posZ>( bvh.bvhNode[c2].aabbMin, bvh.bvhNode[c2].aabbMax, rdMin, rdMax, roMin, roMax, tfar );
			if (d1 < BVH_FAR) nodeStack[stackPtr] = c1, fpiStack[stackPtr++] = fpi;	// no ordering
			if (d2 < BVH_FAR) nodeStack[stackPtr] = c2, fpiStack[stackPtr++] = fpi;
			BVH_FATAL_ERROR_IF( stackPtr > TINYBVH_STACK_SIZE - 2, "BVH::IsOccludedBundle, traversal stack overflow." );
		}
		else
		{
			bool retired = false;
			for (uint32_t i = 0; i < node.triCount; i++)
			{
				const uint32_t instIdx = bvh.primIdx[node.leftFirst + i];
				const BLASInstance& inst = bvh.instList[instIdx];
				if (!(inst.mask & b.mask)) continue;
				uint32_t vn = 0;
				for (uint32_t a = active & alive; a; a &= a - 1) visiting[vn++] = pk[tinybvh_ctz( a )];
				if (!vn) continue;
				b.TransformTo( obj, visiting, vn, inst.invTransform, instIdx << impl::bvh_inst_shift<uint32_t> );
				for (uint32_t j = 0; j < vn; j++) objPk[j] = j;
				steps += tinybvh_occlude_packets( bvh.blasList[inst.blasIdx], obj, objPk, vn );
				// union the instance's result back, and drop packets that finished.
				for (uint32_t j = 0; j < vn; j++)
				{
					const uint32_t wp = obj.src[j * 8] / 8;
					const uint8_t before = b.occluded[wp];
					b.occluded[wp] |= obj.occluded[j];
					if (b.occluded[wp] == before) continue;
					retired = true;
					for (uint32_t s = 0; s < n; s++) if (pk[s] == wp)
					{
						tcur[s] = _mm256_blendv_ps( _mm256_load_ps( b.t + wp * 8 ), minusOne8,
							tinybvh_lanemask8( b.occluded[wp] ) );
						if (b.occluded[wp] == 0xff) alive &= ~(1u << s);
						break;
					}
				}
				if (!alive) ISUNLIKELY return steps;
			}
			if (retired)
			{
				__m256 m8 = minusOne8;
				for (uint32_t a = alive; a; a &= a - 1) m8 = _mm256_max_ps( m8, tcur[tinybvh_ctz( a )] );
				_mm_store_ss( &tfar, tinybvh_lo4( tinybvh_hmax8( m8 ) ) );
			}
		}
		while (1)
		{
			if (!stackPtr) ISUNLIKELY return steps;
			const uint32_t entry = nodeStack[--stackPtr], first = fpiStack[stackPtr];
			const BVHNode& cand = bvh.bvhNode[entry];
			const bool isLeaf = cand.isLeaf();
			const __m256 px1 = _mm256_set1_ps( posX ? cand.aabbMin.x : cand.aabbMax.x );
			const __m256 py1 = _mm256_set1_ps( posY ? cand.aabbMin.y : cand.aabbMax.y );
			const __m256 pz1 = _mm256_set1_ps( posZ ? cand.aabbMin.z : cand.aabbMax.z );
			const __m256 px2 = _mm256_set1_ps( posX ? cand.aabbMax.x : cand.aabbMin.x );
			const __m256 py2 = _mm256_set1_ps( posY ? cand.aabbMax.y : cand.aabbMin.y );
			const __m256 pz2 = _mm256_set1_ps( posZ ? cand.aabbMax.z : cand.aabbMin.z );
			uint32_t entering = 0, firstHit = 0, i = first;
			do
			{
				if ((alive >> i) & 1) if (TINYBVH_PACKET_BOX_TEST_ANY( i ))
				{
					if (!entering) firstHit = i;
					entering |= 1u << i;
					if (!isLeaf) break;
				}
				if (++i == n) i = 0;
			}
			while (i != first);
			if (!entering) continue;
			nodeIdx = entry, fpi = firstHit, active = entering;
			break;
		}
	}
}

// Group packets by octant and trace.
static int32_t tinybvh_occlude_packets( const BVHBase* bvh, RayBundle& b, const uint32_t* pk, const uint32_t n )
{
	const BVH* tlas = (bvh->layout == LAYOUT_BVH && ((const BVH*)bvh)->isTLAS()) ? (const BVH*)bvh : 0;
	const BVH4_CPU* blas4 = bvh->layout == LAYOUT_BVH4_CPU ? (const BVH4_CPU*)bvh : 0;
	if (!tlas && !blas4) return tinybvh_packets_occluded_per_ray( bvh, b, pk, n );
	uint32_t bucket[8][TINYBVH_BUNDLE_PACKETS], bn[8] = {}, loose[TINYBVH_BUNDLE_PACKETS], ln = 0;
	for (uint32_t i = 0; i < n; i++)
	{
		if (b.occluded[pk[i]] == 0xff) continue;
		const int32_t o = b.Octant( pk[i] );
		if (o < 0) loose[ln++] = pk[i]; else bucket[o][bn[o]++] = pk[i];
	}
	int32_t cost = 0;
	for (uint32_t o = 0; o < 8; o++) if (bn[o])
	{
		if (tlas) OCTANT_DISPATCH_BUNDLE( tinybvh_occluded_tlas, o, *tlas, b, bucket[o], bn[o] )
		else OCTANT_DISPATCH_BUNDLE( tinybvh_occluded_bvh4, o, *blas4, b, bucket[o], bn[o] )
	}
	if (ln) cost += tinybvh_packets_occluded_per_ray( bvh, b, loose, ln );
	return cost;
}

static int32_t tinybvh_isoccluded_bundle( const BVHBase* bvh, Ray* rays, bool* occluded )
{
	RayBundle b;
	for (uint32_t i = 0; i < TINYBVH_BUNDLE_RAYS; i++) b.SetRay( i, rays[i] );
	for (uint32_t p = 0; p < TINYBVH_BUNDLE_PACKETS; p++) b.occluded[p] = 0;
	b.packets = TINYBVH_BUNDLE_PACKETS, b.instIdx = 0, b.mask = rays[0].mask;
	uint32_t pk[TINYBVH_BUNDLE_PACKETS];
	for (uint32_t p = 0; p < TINYBVH_BUNDLE_PACKETS; p++) pk[p] = p;
	const int32_t cost = tinybvh_occlude_packets( bvh, b, pk, TINYBVH_BUNDLE_PACKETS );
	for (uint32_t i = 0; i < TINYBVH_BUNDLE_RAYS; i++)
		occluded[i] = (b.occluded[i >> 3] >> (i & 7)) & 1;
	return cost;
}

template <> int32_t impl::BVH4_CPU<float, uint32_t>::IsOccludedBundle( Ray* rays, bool* occluded ) const
{
	return tinybvh_isoccluded_bundle( this, rays, occluded );
}

template <> int32_t impl::BVH<float, uint32_t>::IsOccludedBundle( Ray* rays, bool* occluded ) const
{
	BVH_FATAL_ERROR_IF( !isTLAS(), "BVH::IsOccludedBundle, not a TLAS." );
	return tinybvh_isoccluded_bundle( this, rays, occluded );
}

#undef OCTANT_DISPATCH_BUNDLE

#endif // BVH_USEAVX2

#endif // BVH_USEAVX

} // namespace tinybvh

#endif // TINY_BVH_X86_FLOAT_H_IMPL
#endif // TINYBVH_IMPLEMENTATION