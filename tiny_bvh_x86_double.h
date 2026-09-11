// tiny_bvh_x86_double.h: AVX2 specializations for the double precision layouts.
// Included by tiny_bvh.h; do not include directly.

#ifndef TINY_BVH_H_
#error "Include tiny_bvh.h instead of tiny_bvh_x86_double.h."
#endif

#ifndef TINY_BVH_X86_DOUBLE_H_
#define TINY_BVH_X86_DOUBLE_H_

#if defined DOUBLE_PRECISION_SUPPORT && defined BVH_USEAVX2

namespace tinybvh {

// Specializations provided by this header.
template <> template <bool posX, bool posY, bool posZ> int32_t impl::BVH4_CPU<double, uint64_t>::IntersectOctant( Ray& ray ) const;
template <> template <bool posX, bool posY, bool posZ> bool impl::BVH4_CPU<double, uint64_t>::IsOccludedOctant( const Ray& ray ) const;

} // namespace tinybvh

#endif // DOUBLE_PRECISION_SUPPORT && BVH_USEAVX2

#endif // TINY_BVH_X86_DOUBLE_H_

// ============================================================================
//
//        I M P L E M E N T A T I O N  -  A V X 2  C O D E
//
// ============================================================================

#if defined TINYBVH_IMPLEMENTATION && defined DOUBLE_PRECISION_SUPPORT && defined BVH_USEAVX2

namespace tinybvh {

// BVH4_CPU<double> traversal, AVX2 version.
// ----------------------------------------------------------------------------
// Direct translation of the single precision SSE kernel to 256-bit registers.

#define AVX_HIT( l ) ((m >> l) & 1)
#define AVX_PUSH( c, l ) { nodeStack[stackPtr] = c; distStack[stackPtr] = tmin4[l]; stackPtr++; }

template <> template <bool posX, bool posY, bool posZ> int32_t impl::BVH4_CPU<double, uint64_t>::IntersectOctant( Ray& ray ) const
{
	ALIGNED( 64 ) uint32_t nodeStack[TINYBVH_STACK_SIZE * 2 /* wide trees push more nodes per step */];
	ALIGNED( 64 ) double distStack[TINYBVH_STACK_SIZE * 2];
	ALIGNED( 32 ) double tmin4[4];
	const __m256d zero4 = _mm256_setzero_pd();
	__m256d t4 = _mm256_set1_pd( ray.hit.t );
	int32_t stackPtr = 0;
	uint32_t nodeIdx = 0;
	double tcur = ray.hit.t;
	constexpr int signShift = (posX ? 2 : 0) + (posY ? 4 : 0) + (posZ ? 8 : 0);
	const __m256d rx4 = _mm256_set1_pd( ray.O.x * ray.rD.x ), rdx4 = _mm256_set1_pd( ray.rD.x );
	const __m256d ry4 = _mm256_set1_pd( ray.O.y * ray.rD.y ), rdy4 = _mm256_set1_pd( ray.rD.y );
	const __m256d rz4 = _mm256_set1_pd( ray.O.z * ray.rD.z ), rdz4 = _mm256_set1_pd( ray.rD.z );
	const __m256d ox4 = _mm256_set1_pd( ray.O.x ), oy4 = _mm256_set1_pd( ray.O.y ), oz4 = _mm256_set1_pd( ray.O.z );
	const __m256d dx4 = _mm256_set1_pd( ray.D.x ), dy4 = _mm256_set1_pd( ray.D.y ), dz4 = _mm256_set1_pd( ray.D.z );
	const __m256d one4 = _mm256_set1_pd( 1 ), inf4 = _mm256_set1_pd( BVH_DBL_FAR );
#ifdef _DEBUG
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
			const __m256d tx1 = _mm256_fmsub_pd( _mm256_load_pd( posX ? n->xmin : n->xmax ), rdx4, rx4 );
			const __m256d ty1 = _mm256_fmsub_pd( _mm256_load_pd( posY ? n->ymin : n->ymax ), rdy4, ry4 );
			const __m256d tz1 = _mm256_fmsub_pd( _mm256_load_pd( posZ ? n->zmin : n->zmax ), rdz4, rz4 );
			const __m256d tx2 = _mm256_fmsub_pd( _mm256_load_pd( posX ? n->xmax : n->xmin ), rdx4, rx4 );
			const __m256d ty2 = _mm256_fmsub_pd( _mm256_load_pd( posY ? n->ymax : n->ymin ), rdy4, ry4 );
			const __m256d tz2 = _mm256_fmsub_pd( _mm256_load_pd( posZ ? n->zmax : n->zmin ), rdz4, rz4 );
			const __m256d tmin = _mm256_max_pd( _mm256_max_pd( zero4, tx1 ), _mm256_max_pd( ty1, tz1 ) );
			const __m256d tmax = _mm256_min_pd( _mm256_min_pd( tx2, t4 ), _mm256_min_pd( ty2, tz2 ) );
			// The SSE kernel permutes the slab mask and tmin into sorted order, so that the
			// ladder below tests bit s and pushes lane s. AVX2 has no variable cross-lane
			// permute for 64-bit lanes, so both stay in lane order here and the ladder uses
			// the lane number of each sorted position, which it needs for the child index anyway.
			const uint32_t m = _mm256_movemask_pd( _mm256_cmp_pd( tmin, tmax, _CMP_LE_OQ ) );
			_mm256_store_pd( tmin4, tmin );
			// lane at each sorted position, and the child index stored in that lane
			const uint32_t l3 = (perm[3] >> signShift) & 3, l2 = (perm[2] >> signShift) & 3;
			const uint32_t l1 = (perm[1] >> signShift) & 3, l0 = (perm[0] >> signShift) & 3;
			const uint32_t c3 = child[l3], c2 = child[l2], c1 = child[l1], c0 = child[l0];
			// continue with the nearest valid child and push the others, farthest first
			if (AVX_HIT( l3 ))
			{
				if (AVX_HIT( l0 )) AVX_PUSH( c0, l0 );
				if (AVX_HIT( l1 )) AVX_PUSH( c1, l1 );
				if (AVX_HIT( l2 )) AVX_PUSH( c2, l2 );
				nodeIdx = c3;
			}
			else if (AVX_HIT( l2 ))
			{
				if (AVX_HIT( l0 )) AVX_PUSH( c0, l0 );
				if (AVX_HIT( l1 )) AVX_PUSH( c1, l1 );
				nodeIdx = c2;
			}
			else if (AVX_HIT( l1 ))
			{
				if (AVX_HIT( l0 )) AVX_PUSH( c0, l0 );
				nodeIdx = c1;
			}
			else if (AVX_HIT( l0 )) nodeIdx = c0;
			else
			{
				// skip entries behind the current hit
				do { if (!stackPtr) goto the_end; nodeIdx = nodeStack[--stackPtr]; } while (distStack[stackPtr] > tcur);
			}
		}
		// Moeller-Trumbore ray/triangle intersection algorithm for four triangles
		const BVHTri4Leaf* leaf = (BVHTri4Leaf*)(bvh4Data + (nodeIdx & 0x1fffffff));
		const __m256d hx4 = _mm256_fmsub_pd( dy4, _mm256_load_pd( leaf->e2z ), _mm256_mul_pd( dz4, _mm256_load_pd( leaf->e2y ) ) );
		const __m256d hy4 = _mm256_fmsub_pd( dz4, _mm256_load_pd( leaf->e2x ), _mm256_mul_pd( dx4, _mm256_load_pd( leaf->e2z ) ) );
		const __m256d hz4 = _mm256_fmsub_pd( dx4, _mm256_load_pd( leaf->e2y ), _mm256_mul_pd( dy4, _mm256_load_pd( leaf->e2x ) ) );
		const __m256d sx4 = _mm256_sub_pd( ox4, _mm256_load_pd( leaf->v0x ) ), sy4 = _mm256_sub_pd( oy4, _mm256_load_pd( leaf->v0y ) ), sz4 = _mm256_sub_pd( oz4, _mm256_load_pd( leaf->v0z ) );
		const __m256d det4 = _mm256_fmadd_pd( _mm256_load_pd( leaf->e1z ), hz4, _mm256_fmadd_pd( _mm256_load_pd( leaf->e1x ), hx4, _mm256_mul_pd( _mm256_load_pd( leaf->e1y ), hy4 ) ) );
		const __m256d qz4 = _mm256_fmsub_pd( sx4, _mm256_load_pd( leaf->e1y ), _mm256_mul_pd( sy4, _mm256_load_pd( leaf->e1x ) ) );
		const __m256d qx4 = _mm256_fmsub_pd( sy4, _mm256_load_pd( leaf->e1z ), _mm256_mul_pd( sz4, _mm256_load_pd( leaf->e1y ) ) );
		const __m256d qy4 = _mm256_fmsub_pd( sz4, _mm256_load_pd( leaf->e1x ), _mm256_mul_pd( sx4, _mm256_load_pd( leaf->e1z ) ) );
		const __m256d inv_det4 = _mm256_div_pd( one4, det4 );
		const __m256d u4 = _mm256_mul_pd( _mm256_fmadd_pd( sz4, hz4, _mm256_fmadd_pd( sx4, hx4, _mm256_mul_pd( sy4, hy4 ) ) ), inv_det4 );
		const __m256d v4 = _mm256_mul_pd( _mm256_fmadd_pd( dz4, qz4, _mm256_fmadd_pd( dx4, qx4, _mm256_mul_pd( dy4, qy4 ) ) ), inv_det4 );
		const __m256d ta4 = _mm256_mul_pd( _mm256_fmadd_pd( _mm256_load_pd( leaf->e2z ), qz4, _mm256_fmadd_pd( _mm256_load_pd( leaf->e2x ), qx4, _mm256_mul_pd( _mm256_load_pd( leaf->e2y ), qy4 ) ) ), inv_det4 );
		const __m256d mask1 = _mm256_and_pd( _mm256_cmp_pd( u4, zero4, _CMP_GE_OQ ), _mm256_cmp_pd( v4, zero4, _CMP_GE_OQ ) );
		const __m256d mask2 = _mm256_cmp_pd( _mm256_add_pd( u4, v4 ), one4, _CMP_LE_OQ );
		const __m256d mask3 = _mm256_and_pd( _mm256_cmp_pd( ta4, t4, _CMP_LT_OQ ), _mm256_cmp_pd( ta4, zero4, _CMP_GT_OQ ) );
		__m256d combined = _mm256_and_pd( _mm256_and_pd( mask1, mask2 ), mask3 );
		uint32_t imask = _mm256_movemask_pd( combined );
		// evaluate opacity map, if present (AVX2 version).
		if (opmap) if (imask)
		{
			const __m256d fN4 = _mm256_set1_pd( (double)opmapN );
			const __m128i row4 = _mm256_cvttpd_epi32( _mm256_mul_pd( _mm256_add_pd( u4, v4 ), fN4 ) );
			const __m128i dia4 = _mm256_cvttpd_epi32( _mm256_mul_pd( _mm256_sub_pd( one4, u4 ), fN4 ) );
			const __m128i v0 = _mm_mullo_epi32( row4, row4 );
			const __m128i v1 = _mm256_cvttpd_epi32( _mm256_mul_pd( v4, fN4 ) );
			const __m128i v2 = _mm_sub_epi32( dia4, _mm_sub_epi32( _mm_set1_epi32( opmapN - 1 ), row4 ) );
			uint32_t idx[4];
			uint64_t omask[4] = { 0, 0, 0, 0 };
			tinybvh_store4i( idx, _mm_add_epi32( _mm_add_epi32( v0, v1 ), v2 ) );
			// gather the opacity bits with scalar loads
			for (int i = 0; i < 4; i++) if (imask & (1 << i))
			{
				uint32_t* om = opmap + leaf->primIdx[i] * ((opmapN * opmapN + 31) >> 5);
				if (om[idx[i] >> 5] & (1 << (idx[i] & 31))) omask[i] = ~0ull;
			}
			// combine
			combined = _mm256_and_pd( combined, _mm256_castsi256_pd( tinybvh_load8i( omask ) ) );
			imask = _mm256_movemask_pd( combined );
		}
		if (imask)
		{
			const __m256d dist4 = _mm256_blendv_pd( inf4, ta4, combined );
			// compute broadcasted horizontal minimum of dist4
			const __m256d a = _mm256_min_pd( dist4, _mm256_permute2f128_pd( dist4, dist4, 1 ) );
			const __m256d c = _mm256_min_pd( a, _mm256_permute_pd( a, 5 ) );
			const uint32_t lane = __bfind( _mm256_movemask_pd( _mm256_cmp_pd( c, dist4, _CMP_EQ_OQ ) ) );
			// update hit record
			const double t = _mm256_cvtsd_f64( c );
			ray.hit.t = t, ray.hit.u = tinybvh_getlane_d( &u4, lane ), ray.hit.v = tinybvh_getlane_d( &v4, lane );
			ray.SetHitPrim( leaf->primIdx[lane] );
			t4 = c, tcur = t;
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

#undef AVX_PUSH

template <> template <bool posX, bool posY, bool posZ> bool impl::BVH4_CPU<double, uint64_t>::IsOccludedOctant( const Ray& ray ) const
{
	ALIGNED( 64 ) uint32_t nodeStack[TINYBVH_STACK_SIZE * 2 /* wide trees push more nodes per step */];
	int32_t stackPtr = 0;
	uint32_t nodeIdx = 0;
	const __m256d t4 = _mm256_set1_pd( ray.hit.t );
	const __m256d rx4 = _mm256_set1_pd( ray.O.x * ray.rD.x ), rdx4 = _mm256_set1_pd( ray.rD.x );
	const __m256d ry4 = _mm256_set1_pd( ray.O.y * ray.rD.y ), rdy4 = _mm256_set1_pd( ray.rD.y );
	const __m256d rz4 = _mm256_set1_pd( ray.O.z * ray.rD.z ), rdz4 = _mm256_set1_pd( ray.rD.z );
	const __m256d ox4 = _mm256_set1_pd( ray.O.x ), oy4 = _mm256_set1_pd( ray.O.y ), oz4 = _mm256_set1_pd( ray.O.z );
	const __m256d dx4 = _mm256_set1_pd( ray.D.x ), dy4 = _mm256_set1_pd( ray.D.y ), dz4 = _mm256_set1_pd( ray.D.z );
	const __m256d one4 = _mm256_set1_pd( 1 ), zero4 = _mm256_setzero_pd();
	while (1)
	{
		while (!(nodeIdx & LEAF_BIT))
		{
			const BVHNode* n = (BVHNode*)(bvh4Data + nodeIdx);
			const uint32_t* child = n->child;
			const __m256d tx1 = _mm256_fmsub_pd( _mm256_load_pd( posX ? n->xmin : n->xmax ), rdx4, rx4 );
			const __m256d ty1 = _mm256_fmsub_pd( _mm256_load_pd( posY ? n->ymin : n->ymax ), rdy4, ry4 );
			const __m256d tz1 = _mm256_fmsub_pd( _mm256_load_pd( posZ ? n->zmin : n->zmax ), rdz4, rz4 );
			const __m256d tx2 = _mm256_fmsub_pd( _mm256_load_pd( posX ? n->xmax : n->xmin ), rdx4, rx4 );
			const __m256d ty2 = _mm256_fmsub_pd( _mm256_load_pd( posY ? n->ymax : n->ymin ), rdy4, ry4 );
			const __m256d tz2 = _mm256_fmsub_pd( _mm256_load_pd( posZ ? n->zmax : n->zmin ), rdz4, rz4 );
			const __m256d tmin = _mm256_max_pd( _mm256_max_pd( zero4, tx1 ), _mm256_max_pd( ty1, tz1 ) );
			const __m256d tmax = _mm256_min_pd( _mm256_min_pd( tx2, t4 ), _mm256_min_pd( ty2, tz2 ) );
			// slab mask in lane order, the children are visited in any order
			const uint32_t m = _mm256_movemask_pd( _mm256_cmp_pd( tmin, tmax, _CMP_LE_OQ ) );
			const uint32_t c0 = child[0], c1 = child[1], c2 = child[2], c3 = child[3];
			if (AVX_HIT( 0 ))
			{
				if (AVX_HIT( 3 )) nodeStack[stackPtr++] = c3;
				if (AVX_HIT( 2 )) nodeStack[stackPtr++] = c2;
				if (AVX_HIT( 1 )) nodeStack[stackPtr++] = c1;
				nodeIdx = c0;
			}
			else if (AVX_HIT( 1 ))
			{
				if (AVX_HIT( 3 )) nodeStack[stackPtr++] = c3;
				if (AVX_HIT( 2 )) nodeStack[stackPtr++] = c2;
				nodeIdx = c1;
			}
			else if (AVX_HIT( 2 ))
			{
				if (AVX_HIT( 3 )) nodeStack[stackPtr++] = c3;
				nodeIdx = c2;
			}
			else if (AVX_HIT( 3 )) nodeIdx = c3; else
			{
				if (!stackPtr) return false;
				nodeIdx = nodeStack[--stackPtr];
			}
		}
		// Moeller-Trumbore ray/triangle intersection algorithm for four triangles
		const BVHTri4Leaf* leaf = (BVHTri4Leaf*)(bvh4Data + (nodeIdx & 0x1fffffff));
		const __m256d hx4 = _mm256_fmsub_pd( dy4, _mm256_load_pd( leaf->e2z ), _mm256_mul_pd( dz4, _mm256_load_pd( leaf->e2y ) ) );
		const __m256d hy4 = _mm256_fmsub_pd( dz4, _mm256_load_pd( leaf->e2x ), _mm256_mul_pd( dx4, _mm256_load_pd( leaf->e2z ) ) );
		const __m256d hz4 = _mm256_fmsub_pd( dx4, _mm256_load_pd( leaf->e2y ), _mm256_mul_pd( dy4, _mm256_load_pd( leaf->e2x ) ) );
		const __m256d sx4 = _mm256_sub_pd( ox4, _mm256_load_pd( leaf->v0x ) ), sy4 = _mm256_sub_pd( oy4, _mm256_load_pd( leaf->v0y ) ), sz4 = _mm256_sub_pd( oz4, _mm256_load_pd( leaf->v0z ) );
		const __m256d det4 = _mm256_fmadd_pd( _mm256_load_pd( leaf->e1z ), hz4, _mm256_fmadd_pd( _mm256_load_pd( leaf->e1x ), hx4, _mm256_mul_pd( _mm256_load_pd( leaf->e1y ), hy4 ) ) );
		const __m256d qz4 = _mm256_fmsub_pd( sx4, _mm256_load_pd( leaf->e1y ), _mm256_mul_pd( sy4, _mm256_load_pd( leaf->e1x ) ) );
		const __m256d qx4 = _mm256_fmsub_pd( sy4, _mm256_load_pd( leaf->e1z ), _mm256_mul_pd( sz4, _mm256_load_pd( leaf->e1y ) ) );
		const __m256d qy4 = _mm256_fmsub_pd( sz4, _mm256_load_pd( leaf->e1x ), _mm256_mul_pd( sx4, _mm256_load_pd( leaf->e1z ) ) );
		const __m256d inv_det4 = _mm256_div_pd( one4, det4 );
		const __m256d u4 = _mm256_mul_pd( _mm256_fmadd_pd( sz4, hz4, _mm256_fmadd_pd( sx4, hx4, _mm256_mul_pd( sy4, hy4 ) ) ), inv_det4 );
		const __m256d v4 = _mm256_mul_pd( _mm256_fmadd_pd( dz4, qz4, _mm256_fmadd_pd( dx4, qx4, _mm256_mul_pd( dy4, qy4 ) ) ), inv_det4 );
		const __m256d ta4 = _mm256_mul_pd( _mm256_fmadd_pd( _mm256_load_pd( leaf->e2z ), qz4, _mm256_fmadd_pd( _mm256_load_pd( leaf->e2x ), qx4, _mm256_mul_pd( _mm256_load_pd( leaf->e2y ), qy4 ) ) ), inv_det4 );
		const __m256d mask1 = _mm256_and_pd( _mm256_cmp_pd( u4, zero4, _CMP_GE_OQ ), _mm256_cmp_pd( v4, zero4, _CMP_GE_OQ ) );
		const __m256d mask2 = _mm256_cmp_pd( _mm256_add_pd( u4, v4 ), one4, _CMP_LE_OQ );
		const __m256d mask3 = _mm256_and_pd( _mm256_cmp_pd( ta4, t4, _CMP_LT_OQ ), _mm256_cmp_pd( ta4, zero4, _CMP_GT_OQ ) );
		const __m256d combined = _mm256_and_pd( _mm256_and_pd( mask1, mask2 ), mask3 );
		const uint32_t imask = _mm256_movemask_pd( combined );
		if (imask)
		{
			if (!opmap) return true;
			// evaluate opacity map, AVX2 version.
			const __m256d fN4 = _mm256_set1_pd( (double)opmapN );
			const __m128i row4 = _mm256_cvttpd_epi32( _mm256_mul_pd( _mm256_add_pd( u4, v4 ), fN4 ) );
			const __m128i dia4 = _mm256_cvttpd_epi32( _mm256_mul_pd( _mm256_sub_pd( one4, u4 ), fN4 ) );
			const __m128i v0 = _mm_mullo_epi32( row4, row4 );
			const __m128i v1 = _mm256_cvttpd_epi32( _mm256_mul_pd( v4, fN4 ) );
			const __m128i v2 = _mm_sub_epi32( dia4, _mm_sub_epi32( _mm_set1_epi32( opmapN - 1 ), row4 ) );
			uint32_t idx[4];
			tinybvh_store4i( idx, _mm_add_epi32( _mm_add_epi32( v0, v1 ), v2 ) );
			// gather the opacity bits with scalar loads
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

#undef AVX_HIT

} // namespace tinybvh

#endif // TINYBVH_IMPLEMENTATION && DOUBLE_PRECISION_SUPPORT && BVH_USEAVX2
