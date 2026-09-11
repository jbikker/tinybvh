// tiny_bvh_arm_double.h: NEON specializations for the double precision layouts.
// Included by tiny_bvh.h; do not include directly.

#ifndef TINY_BVH_H_
#error "Include tiny_bvh.h instead of tiny_bvh_arm_double.h."
#endif

#ifndef TINY_BVH_ARM_DOUBLE_H_
#define TINY_BVH_ARM_DOUBLE_H_

#ifdef DOUBLE_PRECISION_SUPPORT

namespace tinybvh {

// Specializations provided by this header.
template <> template <bool posX, bool posY, bool posZ> int32_t impl::BVH4_CPU<double, uint64_t>::IntersectOctant( Ray& ray ) const;
template <> template <bool posX, bool posY, bool posZ> bool impl::BVH4_CPU<double, uint64_t>::IsOccludedOctant( const Ray& ray ) const;

} // namespace tinybvh

#endif // DOUBLE_PRECISION_SUPPORT

#endif // TINY_BVH_ARM_DOUBLE_H_

// ============================================================================
//
//        I M P L E M E N T A T I O N  -  A R M / N E O N  C O D E
//
// ============================================================================

#if defined TINYBVH_IMPLEMENTATION && defined DOUBLE_PRECISION_SUPPORT

namespace tinybvh {

// BVH4_CPU<double> traversal, NEON version.
// ----------------------------------------------------------------------------
// Direct translation of the NEON single precision kernel. Every 4xFP32 operation
// turns into a pair of 2xFP64 ones.

// Narrow two 64-bit lane masks to one 32-bit lane mask.
TINYBVH_FORCEINLINE uint32x4_t neon_narrow_mask( const uint64x2_t a, const uint64x2_t b )
{
	return vuzp1q_u32( vreinterpretq_u32_u64( a ), vreinterpretq_u32_u64( b ) );
}

// Truncating conversion of two double pairs to four 32-bit integers.
TINYBVH_FORCEINLINE int32x4_t neon_cvt4_s32_f64( const float64x2_t a, const float64x2_t b )
{
	return vuzp1q_s32( vreinterpretq_s32_s64( vcvtq_s64_f64( a ) ), vreinterpretq_s32_s64( vcvtq_s64_f64( b ) ) );
}

#define NEON_HIT( s ) ((m64 >> (16 * s)) & 1)
#define NEON_PUSH( c, l ) { nodeStack[stackPtr] = c; distStack[stackPtr] = tmin4[l]; stackPtr++; }

template <> template <bool posX, bool posY, bool posZ> int32_t impl::BVH4_CPU<double, uint64_t>::IntersectOctant( Ray& ray ) const
{
	ALIGNED( 64 ) uint32_t nodeStack[TINYBVH_STACK_SIZE * 2 /* wide trees push more nodes per step */];
	ALIGNED( 64 ) double distStack[TINYBVH_STACK_SIZE * 2];
	ALIGNED( 16 ) double tmin4[4];
	const float64x2_t zero2 = vdupq_n_f64( 0 ), one2 = vdupq_n_f64( 1 ), inf2 = vdupq_n_f64( BVH_DBL_FAR );
	float64x2_t t2 = vdupq_n_f64( ray.hit.t );
	int32_t stackPtr = 0;
	uint32_t nodeIdx = 0;
	double tcur = ray.hit.t;
	constexpr int signShift = (posX ? 2 : 0) + (posY ? 4 : 0) + (posZ ? 8 : 0);
	// the slab test computes bound * rD - O * rD using a fused multiply-add.
	const float64x2_t rx2 = vdupq_n_f64( -ray.O.x * ray.rD.x ), rdx2 = vdupq_n_f64( ray.rD.x );
	const float64x2_t ry2 = vdupq_n_f64( -ray.O.y * ray.rD.y ), rdy2 = vdupq_n_f64( ray.rD.y );
	const float64x2_t rz2 = vdupq_n_f64( -ray.O.z * ray.rD.z ), rdz2 = vdupq_n_f64( ray.rD.z );
	const float64x2_t ox2 = vdupq_n_f64( ray.O.x ), oy2 = vdupq_n_f64( ray.O.y ), oz2 = vdupq_n_f64( ray.O.z );
	const float64x2_t dx2 = vdupq_n_f64( ray.D.x ), dy2 = vdupq_n_f64( ray.D.y ), dz2 = vdupq_n_f64( ray.D.z );
	// constants that turn the 2-bit lane indices in perm4 into a byte shuffle
	const uint32x4_t mul4 = vdupq_n_u32( 0x04040404 ), add4 = vdupq_n_u32( 0x03020100 );
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
			const double* xn = posX ? n->xmin : n->xmax, * xf = posX ? n->xmax : n->xmin;
			const double* yn = posY ? n->ymin : n->ymax, * yf = posY ? n->ymax : n->ymin;
			const double* zn = posZ ? n->zmin : n->zmax, * zf = posZ ? n->zmax : n->zmin;
			const float64x2_t txa1 = vfmaq_f64( rx2, vld1q_f64( xn ), rdx2 ), txb1 = vfmaq_f64( rx2, vld1q_f64( xn + 2 ), rdx2 );
			const float64x2_t tya1 = vfmaq_f64( ry2, vld1q_f64( yn ), rdy2 ), tyb1 = vfmaq_f64( ry2, vld1q_f64( yn + 2 ), rdy2 );
			const float64x2_t tza1 = vfmaq_f64( rz2, vld1q_f64( zn ), rdz2 ), tzb1 = vfmaq_f64( rz2, vld1q_f64( zn + 2 ), rdz2 );
			const float64x2_t txa2 = vfmaq_f64( rx2, vld1q_f64( xf ), rdx2 ), txb2 = vfmaq_f64( rx2, vld1q_f64( xf + 2 ), rdx2 );
			const float64x2_t tya2 = vfmaq_f64( ry2, vld1q_f64( yf ), rdy2 ), tyb2 = vfmaq_f64( ry2, vld1q_f64( yf + 2 ), rdy2 );
			const float64x2_t tza2 = vfmaq_f64( rz2, vld1q_f64( zf ), rdz2 ), tzb2 = vfmaq_f64( rz2, vld1q_f64( zf + 2 ), rdz2 );
			const float64x2_t tmina = vmaxq_f64( vmaxq_f64( txa1, tya1 ), vmaxq_f64( tza1, zero2 ) );
			const float64x2_t tminb = vmaxq_f64( vmaxq_f64( txb1, tyb1 ), vmaxq_f64( tzb1, zero2 ) );
			const float64x2_t tmaxa = vminq_f64( vminq_f64( txa2, tya2 ), vminq_f64( tza2, t2 ) );
			const float64x2_t tmaxb = vminq_f64( vminq_f64( txb2, tyb2 ), vminq_f64( tzb2, t2 ) );
			const uint32x4_t mask4 = neon_narrow_mask( vcleq_f64( tmina, tmaxa ), vcleq_f64( tminb, tmaxb ) );
			// The float kernel shuffles tmin into sorted order and pushes lane s of the result.
			// A 64-bit shuffle across two registers is not worth it: tmin is stored as is, and
			// a push reads the entry of the lane that holds its child.
			vst1q_f64( tmin4, tmina ), vst1q_f64( tmin4 + 2, tminb );
			// lane at each sorted position, and the child index stored in that lane
			const uint32_t l3 = (perm[3] >> signShift) & 3, l2 = (perm[2] >> signShift) & 3;
			const uint32_t l1 = (perm[1] >> signShift) & 3, l0 = (perm[0] >> signShift) & 3;
			const uint32_t c3 = child[l3], c2 = child[l2], c1 = child[l1], c0 = child[l0];
			// byte shuffle that brings the lanes into sorted order
			const uint32x4_t perm4 = vld1q_u32( perm );
			const uint32x4_t order4 = vshrq_n_u32( vshlq_n_u32( perm4, 30 - signShift ), 30 );
			const uint8x16_t shfl16 = vreinterpretq_u8_u32( vmlaq_u32( add4, order4, mul4 ) );
			// sorted slab mask, 16 bits per position
			const uint32x4_t sorted4 = vreinterpretq_u32_u8( vqtbl1q_u8( vreinterpretq_u8_u32( mask4 ), shfl16 ) );
			const uint64_t m64 = vget_lane_u64( vreinterpret_u64_u16( vshrn_n_u32( sorted4, 16 ) ), 0 );
			// continue with the nearest valid child and push the others, farthest first
			if (NEON_HIT( 3 ))
			{
				if (NEON_HIT( 0 )) NEON_PUSH( c0, l0 );
				if (NEON_HIT( 1 )) NEON_PUSH( c1, l1 );
				if (NEON_HIT( 2 )) NEON_PUSH( c2, l2 );
				nodeIdx = c3;
			}
			else if (NEON_HIT( 2 ))
			{
				if (NEON_HIT( 0 )) NEON_PUSH( c0, l0 );
				if (NEON_HIT( 1 )) NEON_PUSH( c1, l1 );
				nodeIdx = c2;
			}
			else if (NEON_HIT( 1 ))
			{
				if (NEON_HIT( 0 )) NEON_PUSH( c0, l0 );
				nodeIdx = c1;
			}
			else if (NEON_HIT( 0 )) nodeIdx = c0;
			else
			{
				// skip entries behind the current hit
				do { if (!stackPtr) goto the_end; nodeIdx = nodeStack[--stackPtr]; } while (distStack[stackPtr] > tcur);
			}
		}
		// Moeller-Trumbore ray/triangle intersection algorithm for four triangles
		const BVHTri4Leaf* leaf = (BVHTri4Leaf*)(bvh4Data + (nodeIdx & 0x1fffffff));
		const float64x2_t e1xa = vld1q_f64( leaf->e1x ), e1ya = vld1q_f64( leaf->e1y ), e1za = vld1q_f64( leaf->e1z );
		const float64x2_t e1xb = vld1q_f64( leaf->e1x + 2 ), e1yb = vld1q_f64( leaf->e1y + 2 ), e1zb = vld1q_f64( leaf->e1z + 2 );
		const float64x2_t e2xa = vld1q_f64( leaf->e2x ), e2ya = vld1q_f64( leaf->e2y ), e2za = vld1q_f64( leaf->e2z );
		const float64x2_t e2xb = vld1q_f64( leaf->e2x + 2 ), e2yb = vld1q_f64( leaf->e2y + 2 ), e2zb = vld1q_f64( leaf->e2z + 2 );
		const float64x2_t hxa = vfmsq_f64( vmulq_f64( dy2, e2za ), dz2, e2ya ), hxb = vfmsq_f64( vmulq_f64( dy2, e2zb ), dz2, e2yb );
		const float64x2_t hya = vfmsq_f64( vmulq_f64( dz2, e2xa ), dx2, e2za ), hyb = vfmsq_f64( vmulq_f64( dz2, e2xb ), dx2, e2zb );
		const float64x2_t hza = vfmsq_f64( vmulq_f64( dx2, e2ya ), dy2, e2xa ), hzb = vfmsq_f64( vmulq_f64( dx2, e2yb ), dy2, e2xb );
		const float64x2_t sxa = vsubq_f64( ox2, vld1q_f64( leaf->v0x ) ), sya = vsubq_f64( oy2, vld1q_f64( leaf->v0y ) ), sza = vsubq_f64( oz2, vld1q_f64( leaf->v0z ) );
		const float64x2_t sxb = vsubq_f64( ox2, vld1q_f64( leaf->v0x + 2 ) ), syb = vsubq_f64( oy2, vld1q_f64( leaf->v0y + 2 ) ), szb = vsubq_f64( oz2, vld1q_f64( leaf->v0z + 2 ) );
		const float64x2_t deta = vfmaq_f64( vfmaq_f64( vmulq_f64( e1ya, hya ), e1xa, hxa ), e1za, hza ), detb = vfmaq_f64( vfmaq_f64( vmulq_f64( e1yb, hyb ), e1xb, hxb ), e1zb, hzb );
		const float64x2_t qza = vfmsq_f64( vmulq_f64( sxa, e1ya ), sya, e1xa ), qzb = vfmsq_f64( vmulq_f64( sxb, e1yb ), syb, e1xb );
		const float64x2_t qxa = vfmsq_f64( vmulq_f64( sya, e1za ), sza, e1ya ), qxb = vfmsq_f64( vmulq_f64( syb, e1zb ), szb, e1yb );
		const float64x2_t qya = vfmsq_f64( vmulq_f64( sza, e1xa ), sxa, e1za ), qyb = vfmsq_f64( vmulq_f64( szb, e1xb ), sxb, e1zb );
		const float64x2_t inv_deta = vdivq_f64( one2, deta ), inv_detb = vdivq_f64( one2, detb );
		const float64x2_t ua = vmulq_f64( vfmaq_f64( vfmaq_f64( vmulq_f64( sya, hya ), sxa, hxa ), sza, hza ), inv_deta );
		const float64x2_t ub = vmulq_f64( vfmaq_f64( vfmaq_f64( vmulq_f64( syb, hyb ), sxb, hxb ), szb, hzb ), inv_detb );
		const float64x2_t va = vmulq_f64( vfmaq_f64( vfmaq_f64( vmulq_f64( dy2, qya ), dx2, qxa ), dz2, qza ), inv_deta );
		const float64x2_t vb = vmulq_f64( vfmaq_f64( vfmaq_f64( vmulq_f64( dy2, qyb ), dx2, qxb ), dz2, qzb ), inv_detb );
		const float64x2_t taa = vmulq_f64( vfmaq_f64( vfmaq_f64( vmulq_f64( e2ya, qya ), e2xa, qxa ), e2za, qza ), inv_deta );
		const float64x2_t tab = vmulq_f64( vfmaq_f64( vfmaq_f64( vmulq_f64( e2yb, qyb ), e2xb, qxb ), e2zb, qzb ), inv_detb );
		const uint64x2_t mask1a = vandq_u64( vcgeq_f64( ua, zero2 ), vcgeq_f64( va, zero2 ) ), mask1b = vandq_u64( vcgeq_f64( ub, zero2 ), vcgeq_f64( vb, zero2 ) );
		const uint64x2_t mask2a = vcleq_f64( vaddq_f64( ua, va ), one2 ), mask2b = vcleq_f64( vaddq_f64( ub, vb ), one2 );
		const uint64x2_t mask3a = vandq_u64( vcltq_f64( taa, t2 ), vcgtq_f64( taa, zero2 ) ), mask3b = vandq_u64( vcltq_f64( tab, t2 ), vcgtq_f64( tab, zero2 ) );
		uint64x2_t combineda = vandq_u64( vandq_u64( mask1a, mask2a ), mask3a ), combinedb = vandq_u64( vandq_u64( mask1b, mask2b ), mask3b );
		uint32_t imask = neon_movemask_popc( neon_narrow_mask( combineda, combinedb ) ) & 15;
		// evaluate opacity map, if present (NEON version).
		if (opmap) if (imask)
		{
			const float64x2_t fN2 = vdupq_n_f64( (double)opmapN );
			const int32x4_t row4 = neon_cvt4_s32_f64( vmulq_f64( vaddq_f64( ua, va ), fN2 ), vmulq_f64( vaddq_f64( ub, vb ), fN2 ) );
			const int32x4_t dia4 = neon_cvt4_s32_f64( vmulq_f64( vsubq_f64( one2, ua ), fN2 ), vmulq_f64( vsubq_f64( one2, ub ), fN2 ) );
			const int32x4_t v0 = vmulq_s32( row4, row4 );
			const int32x4_t v1 = neon_cvt4_s32_f64( vmulq_f64( va, fN2 ), vmulq_f64( vb, fN2 ) );
			const int32x4_t v2 = vsubq_s32( dia4, vsubq_s32( vdupq_n_s32( opmapN - 1 ), row4 ) );
			uint32_t idx[4];
			uint64_t omask[4] = { 0, 0, 0, 0 };
			vst1q_u32( idx, vreinterpretq_u32_s32( vaddq_s32( vaddq_s32( v0, v1 ), v2 ) ) );
			// gather the opacity bits with scalar loads
			for (int i = 0; i < 4; i++) if (imask & (1 << i))
			{
				uint32_t* om = opmap + leaf->primIdx[i] * ((opmapN * opmapN + 31) >> 5);
				if (om[idx[i] >> 5] & (1 << (idx[i] & 31))) omask[i] = ~0ull;
			}
			// combine
			combineda = vandq_u64( combineda, vld1q_u64( omask ) ), combinedb = vandq_u64( combinedb, vld1q_u64( omask + 2 ) );
			imask = neon_movemask_popc( neon_narrow_mask( combineda, combinedb ) ) & 15;
		}
		if (imask)
		{
			const float64x2_t dista = vbslq_f64( combineda, taa, inf2 ), distb = vbslq_f64( combinedb, tab, inf2 );
			const double t = vminvq_f64( vminq_f64( dista, distb ) );
			const uint32x4_t eq4 = neon_narrow_mask( vceqq_f64( dista, vdupq_n_f64( t ) ), vceqq_f64( distb, vdupq_n_f64( t ) ) );
			const uint32_t lane = __bfind( neon_movemask_popc( eq4 ) & 15 );
			// update hit record
			ray.hit.t = t, ray.hit.u = tinybvh_getlane_d( lane < 2 ? &ua : &ub, lane & 1 ), ray.hit.v = tinybvh_getlane_d( lane < 2 ? &va : &vb, lane & 1 );
			ray.SetHitPrim( leaf->primIdx[lane] );
			t2 = vdupq_n_f64( t ), tcur = t;
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

#undef NEON_PUSH
#undef NEON_HIT
#define NEON_HIT( l ) ((m64 >> (16 * l)) & 1)

template <> template <bool posX, bool posY, bool posZ> bool impl::BVH4_CPU<double, uint64_t>::IsOccludedOctant( const Ray& ray ) const
{
	ALIGNED( 64 ) uint32_t nodeStack[TINYBVH_STACK_SIZE * 2 /* wide trees push more nodes per step */];
	int32_t stackPtr = 0;
	uint32_t nodeIdx = 0;
	const float64x2_t zero2 = vdupq_n_f64( 0 ), one2 = vdupq_n_f64( 1 ), t2 = vdupq_n_f64( ray.hit.t );
	const float64x2_t rx2 = vdupq_n_f64( -ray.O.x * ray.rD.x ), rdx2 = vdupq_n_f64( ray.rD.x );
	const float64x2_t ry2 = vdupq_n_f64( -ray.O.y * ray.rD.y ), rdy2 = vdupq_n_f64( ray.rD.y );
	const float64x2_t rz2 = vdupq_n_f64( -ray.O.z * ray.rD.z ), rdz2 = vdupq_n_f64( ray.rD.z );
	const float64x2_t ox2 = vdupq_n_f64( ray.O.x ), oy2 = vdupq_n_f64( ray.O.y ), oz2 = vdupq_n_f64( ray.O.z );
	const float64x2_t dx2 = vdupq_n_f64( ray.D.x ), dy2 = vdupq_n_f64( ray.D.y ), dz2 = vdupq_n_f64( ray.D.z );
	while (1)
	{
		while (!(nodeIdx & LEAF_BIT))
		{
			const BVHNode* n = (BVHNode*)(bvh4Data + nodeIdx);
			const uint32_t* child = n->child;
			const double* xn = posX ? n->xmin : n->xmax, * xf = posX ? n->xmax : n->xmin;
			const double* yn = posY ? n->ymin : n->ymax, * yf = posY ? n->ymax : n->ymin;
			const double* zn = posZ ? n->zmin : n->zmax, * zf = posZ ? n->zmax : n->zmin;
			const float64x2_t txa1 = vfmaq_f64( rx2, vld1q_f64( xn ), rdx2 ), txb1 = vfmaq_f64( rx2, vld1q_f64( xn + 2 ), rdx2 );
			const float64x2_t tya1 = vfmaq_f64( ry2, vld1q_f64( yn ), rdy2 ), tyb1 = vfmaq_f64( ry2, vld1q_f64( yn + 2 ), rdy2 );
			const float64x2_t tza1 = vfmaq_f64( rz2, vld1q_f64( zn ), rdz2 ), tzb1 = vfmaq_f64( rz2, vld1q_f64( zn + 2 ), rdz2 );
			const float64x2_t txa2 = vfmaq_f64( rx2, vld1q_f64( xf ), rdx2 ), txb2 = vfmaq_f64( rx2, vld1q_f64( xf + 2 ), rdx2 );
			const float64x2_t tya2 = vfmaq_f64( ry2, vld1q_f64( yf ), rdy2 ), tyb2 = vfmaq_f64( ry2, vld1q_f64( yf + 2 ), rdy2 );
			const float64x2_t tza2 = vfmaq_f64( rz2, vld1q_f64( zf ), rdz2 ), tzb2 = vfmaq_f64( rz2, vld1q_f64( zf + 2 ), rdz2 );
			const float64x2_t tmina = vmaxq_f64( vmaxq_f64( txa1, tya1 ), vmaxq_f64( tza1, zero2 ) );
			const float64x2_t tminb = vmaxq_f64( vmaxq_f64( txb1, tyb1 ), vmaxq_f64( tzb1, zero2 ) );
			const float64x2_t tmaxa = vminq_f64( vminq_f64( txa2, tya2 ), vminq_f64( tza2, t2 ) );
			const float64x2_t tmaxb = vminq_f64( vminq_f64( txb2, tyb2 ), vminq_f64( tzb2, t2 ) );
			const uint32x4_t mask4 = neon_narrow_mask( vcleq_f64( tmina, tmaxa ), vcleq_f64( tminb, tmaxb ) );
			// slab mask, 16 bits per lane; the traversal order does not matter here
			const uint64_t m64 = vget_lane_u64( vreinterpret_u64_u16( vshrn_n_u32( mask4, 16 ) ), 0 );
			const uint32_t c0 = child[0], c1 = child[1], c2 = child[2], c3 = child[3];
			if (NEON_HIT( 0 ))
			{
				if (NEON_HIT( 3 )) nodeStack[stackPtr++] = c3;
				if (NEON_HIT( 2 )) nodeStack[stackPtr++] = c2;
				if (NEON_HIT( 1 )) nodeStack[stackPtr++] = c1;
				nodeIdx = c0;
			}
			else if (NEON_HIT( 1 ))
			{
				if (NEON_HIT( 3 )) nodeStack[stackPtr++] = c3;
				if (NEON_HIT( 2 )) nodeStack[stackPtr++] = c2;
				nodeIdx = c1;
			}
			else if (NEON_HIT( 2 ))
			{
				if (NEON_HIT( 3 )) nodeStack[stackPtr++] = c3;
				nodeIdx = c2;
			}
			else if (NEON_HIT( 3 )) nodeIdx = c3;
			else
			{
				if (!stackPtr) return false;
				nodeIdx = nodeStack[--stackPtr];
			}
		}
		// Moeller-Trumbore ray/triangle intersection algorithm for four triangles
		const BVHTri4Leaf* leaf = (BVHTri4Leaf*)(bvh4Data + (nodeIdx & 0x1fffffff));
		const float64x2_t e1xa = vld1q_f64( leaf->e1x ), e1ya = vld1q_f64( leaf->e1y ), e1za = vld1q_f64( leaf->e1z );
		const float64x2_t e1xb = vld1q_f64( leaf->e1x + 2 ), e1yb = vld1q_f64( leaf->e1y + 2 ), e1zb = vld1q_f64( leaf->e1z + 2 );
		const float64x2_t e2xa = vld1q_f64( leaf->e2x ), e2ya = vld1q_f64( leaf->e2y ), e2za = vld1q_f64( leaf->e2z );
		const float64x2_t e2xb = vld1q_f64( leaf->e2x + 2 ), e2yb = vld1q_f64( leaf->e2y + 2 ), e2zb = vld1q_f64( leaf->e2z + 2 );
		const float64x2_t hxa = vfmsq_f64( vmulq_f64( dy2, e2za ), dz2, e2ya ), hxb = vfmsq_f64( vmulq_f64( dy2, e2zb ), dz2, e2yb );
		const float64x2_t hya = vfmsq_f64( vmulq_f64( dz2, e2xa ), dx2, e2za ), hyb = vfmsq_f64( vmulq_f64( dz2, e2xb ), dx2, e2zb );
		const float64x2_t hza = vfmsq_f64( vmulq_f64( dx2, e2ya ), dy2, e2xa ), hzb = vfmsq_f64( vmulq_f64( dx2, e2yb ), dy2, e2xb );
		const float64x2_t sxa = vsubq_f64( ox2, vld1q_f64( leaf->v0x ) ), sya = vsubq_f64( oy2, vld1q_f64( leaf->v0y ) ), sza = vsubq_f64( oz2, vld1q_f64( leaf->v0z ) );
		const float64x2_t sxb = vsubq_f64( ox2, vld1q_f64( leaf->v0x + 2 ) ), syb = vsubq_f64( oy2, vld1q_f64( leaf->v0y + 2 ) ), szb = vsubq_f64( oz2, vld1q_f64( leaf->v0z + 2 ) );
		const float64x2_t deta = vfmaq_f64( vfmaq_f64( vmulq_f64( e1ya, hya ), e1xa, hxa ), e1za, hza ), detb = vfmaq_f64( vfmaq_f64( vmulq_f64( e1yb, hyb ), e1xb, hxb ), e1zb, hzb );
		const float64x2_t qza = vfmsq_f64( vmulq_f64( sxa, e1ya ), sya, e1xa ), qzb = vfmsq_f64( vmulq_f64( sxb, e1yb ), syb, e1xb );
		const float64x2_t qxa = vfmsq_f64( vmulq_f64( sya, e1za ), sza, e1ya ), qxb = vfmsq_f64( vmulq_f64( syb, e1zb ), szb, e1yb );
		const float64x2_t qya = vfmsq_f64( vmulq_f64( sza, e1xa ), sxa, e1za ), qyb = vfmsq_f64( vmulq_f64( szb, e1xb ), sxb, e1zb );
		const float64x2_t inv_deta = vdivq_f64( one2, deta ), inv_detb = vdivq_f64( one2, detb );
		const float64x2_t ua = vmulq_f64( vfmaq_f64( vfmaq_f64( vmulq_f64( sya, hya ), sxa, hxa ), sza, hza ), inv_deta );
		const float64x2_t ub = vmulq_f64( vfmaq_f64( vfmaq_f64( vmulq_f64( syb, hyb ), sxb, hxb ), szb, hzb ), inv_detb );
		const float64x2_t va = vmulq_f64( vfmaq_f64( vfmaq_f64( vmulq_f64( dy2, qya ), dx2, qxa ), dz2, qza ), inv_deta );
		const float64x2_t vb = vmulq_f64( vfmaq_f64( vfmaq_f64( vmulq_f64( dy2, qyb ), dx2, qxb ), dz2, qzb ), inv_detb );
		const float64x2_t taa = vmulq_f64( vfmaq_f64( vfmaq_f64( vmulq_f64( e2ya, qya ), e2xa, qxa ), e2za, qza ), inv_deta );
		const float64x2_t tab = vmulq_f64( vfmaq_f64( vfmaq_f64( vmulq_f64( e2yb, qyb ), e2xb, qxb ), e2zb, qzb ), inv_detb );
		const uint64x2_t mask1a = vandq_u64( vcgeq_f64( ua, zero2 ), vcgeq_f64( va, zero2 ) ), mask1b = vandq_u64( vcgeq_f64( ub, zero2 ), vcgeq_f64( vb, zero2 ) );
		const uint64x2_t mask2a = vcleq_f64( vaddq_f64( ua, va ), one2 ), mask2b = vcleq_f64( vaddq_f64( ub, vb ), one2 );
		const uint64x2_t mask3a = vandq_u64( vcltq_f64( taa, t2 ), vcgtq_f64( taa, zero2 ) ), mask3b = vandq_u64( vcltq_f64( tab, t2 ), vcgtq_f64( tab, zero2 ) );
		const uint64x2_t combineda = vandq_u64( vandq_u64( mask1a, mask2a ), mask3a ), combinedb = vandq_u64( vandq_u64( mask1b, mask2b ), mask3b );
		const uint32_t imask = neon_movemask_popc( neon_narrow_mask( combineda, combinedb ) ) & 15;
		if (imask)
		{
			if (!opmap) return true;
			// evaluate opacity map, NEON version.
			const float64x2_t fN2 = vdupq_n_f64( (double)opmapN );
			const int32x4_t row4 = neon_cvt4_s32_f64( vmulq_f64( vaddq_f64( ua, va ), fN2 ), vmulq_f64( vaddq_f64( ub, vb ), fN2 ) );
			const int32x4_t dia4 = neon_cvt4_s32_f64( vmulq_f64( vsubq_f64( one2, ua ), fN2 ), vmulq_f64( vsubq_f64( one2, ub ), fN2 ) );
			const int32x4_t v0 = vmulq_s32( row4, row4 );
			const int32x4_t v1 = neon_cvt4_s32_f64( vmulq_f64( va, fN2 ), vmulq_f64( vb, fN2 ) );
			const int32x4_t v2 = vsubq_s32( dia4, vsubq_s32( vdupq_n_s32( opmapN - 1 ), row4 ) );
			uint32_t idx[4];
			vst1q_u32( idx, vreinterpretq_u32_s32( vaddq_s32( vaddq_s32( v0, v1 ), v2 ) ) );
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

#undef NEON_HIT

} // namespace tinybvh

#endif // TINYBVH_IMPLEMENTATION && DOUBLE_PRECISION_SUPPORT
