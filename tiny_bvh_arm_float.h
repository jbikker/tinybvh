// tiny_bvh_arm_float.h: NEON specializations for the single precision layouts.
// Included by tiny_bvh.h; do not include directly.

#ifndef TINY_BVH_H_
#error "Include tiny_bvh.h instead of tiny_bvh_arm_float.h."
#endif

#ifndef TINY_BVH_ARM_FLOAT_H_
#define TINY_BVH_ARM_FLOAT_H_

namespace tinybvh {

#ifdef BVH_USENEON
TINYBVH_FORCEINLINE float32x4_t tinybvh_load4( const void* p ) { float32x4_t r; memcpy( &r, p, 16 ); return r; }
TINYBVH_FORCEINLINE int32x4_t tinybvh_load4i( const void* p ) { int32x4_t r; memcpy( &r, p, 16 ); return r; }
TINYBVH_FORCEINLINE void tinybvh_store4( void* p, const float32x4_t v ) { memcpy( p, &v, 16 ); }
TINYBVH_FORCEINLINE void tinybvh_store4i( void* p, const int32x4_t v ) { memcpy( p, &v, 16 ); }
// 32-byte pair load / store; the NEON counterpart of the AVX tinybvh_load8.
TINYBVH_FORCEINLINE float32x4x2_t tinybvh_load8( const void* p ) { float32x4x2_t r; memcpy( &r, p, 32 ); return r; }
TINYBVH_FORCEINLINE void tinybvh_store8( void* p, const float32x4x2_t v ) { memcpy( p, &v, 32 ); }
#endif
TINYBVH_FORCEINLINE float32x4_t SIMD_SETVEC( float w, float z, float y, float x )
{
	ALIGNED( 64 ) float data[4] = { x, y, z, w };
	return vld1q_f32( data );
}
TINYBVH_FORCEINLINE float32x4_t SIMD_SETRVEC( float x, float y, float z, float w )
{
	ALIGNED( 64 ) float data[4] = { x, y, z, w };
	return vld1q_f32( data );
}
TINYBVH_FORCEINLINE uint32x4_t SIMD_SETRVECU( uint32_t x, uint32_t y, uint32_t z, uint32_t w )
{
	ALIGNED( 64 ) uint32_t data[4] = { x, y, z, w };
	return vld1q_u32( data );
}
TINYBVH_FORCEINLINE int32x4_t SIMD_SETRVECS( int32_t x, int32_t y, int32_t z, int32_t w )
{
	ALIGNED( 64 ) int32_t data[4] = { x, y, z, w };
	return vld1q_s32( data );
}

// Specializations provided by this header.
template <> void impl::BVH<float, uint32_t>::BuildNEON( const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t primCount );
template <> void impl::BVH<float, uint32_t>::PrepareSIMDBuild( const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t primCount );
template <> void impl::BVH<float, uint32_t>::PrepareSIMDBuildFragSlice( const uint32_t first, const uint32_t last, const uint32_t* indices, const int8_t* vertData, const uint32_t stride4, void* frags, float* rootMin, float* rootMax );
template <> void impl::BVH<float, uint32_t>::BuildSIMDBinTask( const uint32_t first, const uint32_t last, void* binbox, uint32_t* count, const float* nmin4, const float* rpd4 );
template <> void impl::BVH<float, uint32_t>::BuildSIMDSubtree( uint32_t nodeIdx, uint32_t depth );
template <> void impl::BVH<float, uint32_t>::BuildSIMDFinalize();
template <> template <bool posX, bool posY, bool posZ> int32_t impl::BVH4_CPU<float, uint32_t>::IntersectOctant( Ray& ray ) const;
template <> template <bool posX, bool posY, bool posZ> bool impl::BVH4_CPU<float, uint32_t>::IsOccludedOctant( const Ray& ray ) const;
#ifdef ENABLE_BVH_SOA
template <> int32_t impl::BVH_SoA<float, uint32_t>::Intersect( Ray& ray ) const;
template <> bool impl::BVH_SoA<float, uint32_t>::IsOccluded( const Ray& ray ) const;
#endif

} // namespace tinybvh

#endif // TINY_BVH_ARM_FLOAT_H_

// ============================================================================
//
//        I M P L E M E N T A T I O N  -  A R M / N E O N  C O D E
//
// ============================================================================

#ifdef TINYBVH_IMPLEMENTATION

namespace tinybvh {

#define ILANE(a,b) vgetq_lane_s32( a, b )

// AABB halfarea calculation
TINYBVH_FORCEINLINE float halfArea( const float32x4_t a /* a contains extent of aabb */ )
{
	const float ex = vgetq_lane_f32( a, 0 ), ey = vgetq_lane_f32( a, 1 );
	const float ez = vgetq_lane_f32( a, 2 ), ew = vgetq_lane_f32( a, 3 );
	return ex * ey + ey * ez + ez * ew;
}
TINYBVH_FORCEINLINE float halfArea( const float32x4x2_t& a /* a contains aabb itself, with min.xyz negated */ )
{
	const float32x4_t e = vaddq_f32( a.val[1], a.val[0] ); // max + (-min)
	const float ex = vgetq_lane_f32( e, 0 ), ey = vgetq_lane_f32( e, 1 ), ez = vgetq_lane_f32( e, 2 );
	return ex * ey + ey * ez + ez * ex;
}

// Eight-wide helpers.
TINYBVH_FORCEINLINE float32x4x2_t vdupq_n_f32x2( const float v )
{
	const float32x4_t v4 = vdupq_n_f32( v );
	return float32x4x2_t{ v4, v4 };
}
TINYBVH_FORCEINLINE float32x4x2_t vmaxq_f32x2( const float32x4x2_t& a, const float32x4x2_t& b )
{
	return float32x4x2_t{ vmaxq_f32( a.val[0], b.val[0] ), vmaxq_f32( a.val[1], b.val[1] ) };
}
TINYBVH_FORCEINLINE float32x4x2_t veorq_f32x2( const float32x4x2_t& a, const float32x4x2_t& b )
{
	const uint32x4_t r0 = veorq_u32( vreinterpretq_u32_f32( a.val[0] ), vreinterpretq_u32_f32( b.val[0] ) );
	const uint32x4_t r1 = veorq_u32( vreinterpretq_u32_f32( a.val[1] ), vreinterpretq_u32_f32( b.val[1] ) );
	return float32x4x2_t{ vreinterpretq_f32_u32( r0 ), vreinterpretq_f32_u32( r1 ) };
}

// Constants for the NEON builder.
static const float32x4_t neon_min1 = vdupq_n_f32( -1.0f );
static const float32x4_t neon_zero4 = vdupq_n_f32( 0.0f );
static const float32x4_t neon_binmul3 = vdupq_n_f32( AVXBINS * 0.49999f );
static const uint32x4_t neon_mask3 = SIMD_SETRVECU( ~0u, ~0u, ~0u, 0u );
static const int32x4_t neon_zero4i = vdupq_n_s32( 0 );
static const int32x4_t neon_maxbin4 = vdupq_n_s32( AVXBINS - 1 );
static const float32x4x2_t neon_max8 = { vdupq_n_f32( -BVH_FAR ), vdupq_n_f32( -BVH_FAR ) };
static const float32x4x2_t neon_signFlip8 = { SIMD_SETRVEC( -0.0f, -0.0f, -0.0f, 0.0f ), vdupq_n_f32( 0.0f ) };

// Bin index for a fragment, three axes at a time.
TINYBVH_FORCEINLINE int32x4_t neon_binIdx( const float32x4_t& fmin, const float32x4_t& fmax,
	const float32x4_t& nmin4, const float32x4_t& rpd4 )
{
	const float32x4_t v = vmulq_f32( vsubq_f32( vaddq_f32( fmax, fmin ), nmin4 ), rpd4 );
	return vmaxq_s32( vminq_s32( vcvtq_s32_f32( v ), neon_maxbin4 ), neon_zero4i );
}

#define PROCESS_PLANE( a, pos, ANLR, lN, rN, lb, rb ) if (lN != 0 && rN != 0) { \
	ANLR = halfArea( lb ) * (float)lN + halfArea( rb ) * (float)rN; if (ANLR < splitCost) \
	splitCost = ANLR, bestAxis = a, bestPos = pos, bestLBox = lb, bestRBox = rb; }
#if defined _MSC_VER
#pragma warning ( push )
#pragma warning( disable:4701 ) // "potentially uninitialized local variable 'bestLBox' used"
#pragma warning (disable:4324) // "structure was padded due to alignment specifier"
#elif defined __GNUC__ && !defined __clang__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif

template <> void impl::BVH<float, uint32_t>::BuildNEON( const bvhvec4slice& v, const uint32_t* i, const uint32_t p )
{
	PrepareSIMDBuild( v, i, p );
	BuildSIMDSubtree( 0u, 0u );
	BuildSIMDFinalize();
}

// Fragment setup, optionally sliced over the thread pool.
static constexpr uint32_t NEONCOUNTSTRIDE = 32; // 32 * 4 bytes = 128 bytes.
struct ALIGNED( 64 ) NEONSliceBounds { float bmin[4], bmax[4]; char pad[32]; };
struct BuildNEONFragSliceArgs
{
	BVH* bvh;
	const uint32_t triCount, sliceSize, slices, * indices, stride4;
	const int8_t* vertData;
	NEONSliceBounds* slice;
	void* frags;
};
static void BuildNEONFragSlice( uint32_t i, void* payload )
{
	BuildNEONFragSliceArgs* a = (BuildNEONFragSliceArgs*)payload;
	const uint32_t first = a->sliceSize * i, last = i == (a->slices - 1) ? a->triCount : (first + a->sliceSize);
	a->bvh->PrepareSIMDBuildFragSlice( first, last, a->indices, a->vertData, a->stride4, a->frags, a->slice[i].bmin, a->slice[i].bmax );
}
template <> void impl::BVH<float, uint32_t>::PrepareSIMDBuildFragSlice( const uint32_t first, const uint32_t last,
	const uint32_t* indices, const int8_t* vertData, const uint32_t stride4, void* frags,
	float* rootMin, float* rootMax )
{
	Fragment* frag = (Fragment*)frags;
	float32x4_t rmin = vdupq_n_f32( BVH_FAR ), rmax = vdupq_n_f32( -BVH_FAR );
	if (indices) for (uint32_t i = first; i < last; i++)
	{
		const uint32_t i0 = indices[i * 3], i1 = indices[i * 3 + 1], i2 = indices[i * 3 + 2];
		const float32x4_t v0 = tinybvh_load4( vertData + (size_t)(i0 * stride4) * 16 );
		const float32x4_t v1 = tinybvh_load4( vertData + (size_t)(i1 * stride4) * 16 );
		const float32x4_t v2 = tinybvh_load4( vertData + (size_t)(i2 * stride4) * 16 );
		const float32x4_t t1 = vminq_f32( vminq_f32( v0, v1 ), v2 ), t2 = vmaxq_f32( vmaxq_f32( v0, v1 ), v2 );
		tinybvh_store4( &frag[i].bmin, t1 ), tinybvh_store4( &frag[i].bmax, t2 );
		rmin = vminq_f32( rmin, t1 ), rmax = vmaxq_f32( rmax, t2 );
		primIdx[i] = i;
	}
	else for (uint32_t i = first; i < last; i++)
	{
		const float32x4_t v0 = tinybvh_load4( vertData + (size_t)((i * 3) * stride4) * 16 );
		const float32x4_t v1 = tinybvh_load4( vertData + (size_t)((i * 3 + 1) * stride4) * 16 );
		const float32x4_t v2 = tinybvh_load4( vertData + (size_t)((i * 3 + 2) * stride4) * 16 );
		const float32x4_t t1 = vminq_f32( vminq_f32( v0, v1 ), v2 ), t2 = vmaxq_f32( vmaxq_f32( v0, v1 ), v2 );
		tinybvh_store4( &frag[i].bmin, t1 ), tinybvh_store4( &frag[i].bmax, t2 );
		rmin = vminq_f32( rmin, t1 ), rmax = vmaxq_f32( rmax, t2 );
		primIdx[i] = i;
	}
	vst1q_f32( rootMin, rmin ), vst1q_f32( rootMax, rmax ); // slices are cache line separated; no false sharing.
}

template <> void impl::BVH<float, uint32_t>::PrepareSIMDBuild( const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t prims )
{
	BVH_FATAL_ERROR_IF( vertices.count == 0, "BVH::PrepareSIMDBuild( .. ), primCount == 0." );
	BVH_FATAL_ERROR_IF( vertices.stride & 15, "BVH::PrepareSIMDBuild( .. ), stride must be multiple of 16." );
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
	// prepare threading; the atomic node counter is claimed in BuildSIMDSubtree.
	threadedBuild = false;
#ifdef ENABLE_THREADED_BUILDS
	if (triCount >= MT_BUILD_THRESHOLD && context.spawn && context.barrier) threadedBuild = true;
#endif
	// initialize fragments
	float32x4_t rootMin = vdupq_n_f32( BVH_FAR ), rootMax = vdupq_n_f32( -BVH_FAR );
	const uint32_t stride4 = verts.stride / 16;
	BVH_FATAL_ERROR_IF( primCount == 0, "BVH::PrepareSIMDBuild( .. ), primCount == 0." );
	if (threadedBuild)
	{
		constexpr int slices = 4;
		ALIGNED( 64 ) NEONSliceBounds slice[slices]; // one cache line per slice; no false sharing.
		BuildNEONFragSliceArgs args = { this, triCount, triCount / slices, slices, indices, stride4, vertData, slice, fragment };
		tinybvh_parallel_for( context, slices, &BuildNEONFragSlice, &args );
		rootMin = vld1q_f32( slice[0].bmin ), rootMax = vld1q_f32( slice[0].bmax );
		for (int i = 1; i < slices; i++)
			rootMin = vminq_f32( rootMin, vld1q_f32( slice[i].bmin ) ), rootMax = vmaxq_f32( rootMax, vld1q_f32( slice[i].bmax ) );
	}
	else
	{
		ALIGNED( 16 ) float rmin[4], rmax[4];
		PrepareSIMDBuildFragSlice( 0, triCount, indices, vertData, stride4, (void*)fragment, rmin, rmax );
		rootMin = vld1q_f32( rmin ), rootMax = vld1q_f32( rmax );
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
	float32x4x2_t* binbox = (float32x4x2_t*)binboxes;
	const float32x4_t nmin4 = vld1q_f32( nmin ), rpd4 = vld1q_f32( rpd );
	memset( count, 0, 3 * AVXBINS * 4 ); // exactly 96 bytes
	for (uint32_t i = 0; i < 3 * AVXBINS; i++) binbox[i] = neon_max8;
	if (first >= last) return; // empty slice; 'last - 1' below would wrap.
	// implementation of Section 4.1 of "Parallel Spatial Splits in Bounding Volume
	// Hierarchies": the loop keeps one fragment in flight to break the dependency
	// between the bin read and the bin write.
	const uint32_t fi = primIdx[first];
	float32x4x2_t r0, r1, r2, f = veorq_f32x2( tinybvh_load8( fragment + fi ), neon_signFlip8 );
	int32x4_t bc4 = neon_binIdx( tinybvh_load4( &fragment[fi].bmin ), tinybvh_load4( &fragment[fi].bmax ), nmin4, rpd4 );
	uint32_t i0 = (uint32_t)ILANE( bc4, 0 ), i1 = (uint32_t)ILANE( bc4, 1 );
	uint32_t i2 = (uint32_t)ILANE( bc4, 2 ), * ti = primIdx + first + 1;
	for (uint32_t i = first; i < last - 1; i++)
	{
		const uint32_t fid = *ti++;
		const float32x4x2_t b0 = binbox[i0], b1 = binbox[AVXBINS + i1], b2 = binbox[2 * AVXBINS + i2];
		const float32x4_t frmin = tinybvh_load4( &fragment[fid].bmin ), frmax = tinybvh_load4( &fragment[fid].bmax );
		r0 = vmaxq_f32x2( b0, f ), r1 = vmaxq_f32x2( b1, f ), r2 = vmaxq_f32x2( b2, f );
		bc4 = neon_binIdx( frmin, frmax, nmin4, rpd4 );
		f = veorq_f32x2( tinybvh_load8( fragment + fid ), neon_signFlip8 );
		count[i0]++, count[AVXBINS + i1]++, count[AVXBINS * 2 + i2]++;
		binbox[i0] = r0, i0 = (uint32_t)ILANE( bc4, 0 );
		binbox[AVXBINS + i1] = r1, i1 = (uint32_t)ILANE( bc4, 1 );
		binbox[2 * AVXBINS + i2] = r2, i2 = (uint32_t)ILANE( bc4, 2 );
	}
	// final business for final fragment
	const float32x4x2_t b0 = binbox[i0], b1 = binbox[AVXBINS + i1], b2 = binbox[2 * AVXBINS + i2];
	count[i0]++, count[AVXBINS + i1]++, count[AVXBINS * 2 + i2]++;
	r0 = vmaxq_f32x2( b0, f ), r1 = vmaxq_f32x2( b1, f ), r2 = vmaxq_f32x2( b2, f );
	binbox[i0] = r0, binbox[AVXBINS + i1] = r1, binbox[2 * AVXBINS + i2] = r2;
}

// Helper function to build a subtree via the thread pool
static void BVHBuildNEONSubtree( void* payload )
{
	impl::BVHBuildSubtreeArgs<float, uint32_t>* a = (impl::BVHBuildSubtreeArgs<float, uint32_t>*)payload;
	a->bvh->BuildSIMDSubtree( a->node, a->depth );
}
// bin one slice of a node's fragment range; scheduled via the parallel_for hook.
struct BVHBuildNEONBinSliceArgs
{
	BVH* bvh;
	uint32_t leftFirst, triCount, sliceSize, slices;
	float32x4x2_t* slicebinbox;			// base of slices x (3*AVXBINS) bin boxes
	uint32_t* slicecount;				// base of slices x NEONCOUNTSTRIDE counts
	float32x4_t nmin4, rpd4;
};
static void BVHBuildNEONBinSlice( uint32_t i, void* payload )
{
	BVHBuildNEONBinSliceArgs* a = (BVHBuildNEONBinSliceArgs*)payload;
	const uint32_t first = a->leftFirst + a->sliceSize * i;
	const uint32_t last = i == (a->slices - 1) ? (a->leftFirst + a->triCount) : (first + a->sliceSize);
	a->bvh->BuildSIMDBinTask( first, last, a->slicebinbox + i * 3 * AVXBINS,
		a->slicecount + i * NEONCOUNTSTRIDE, (const float*)&a->nmin4, (const float*)&a->rpd4 );
}
template <> void impl::BVH<float, uint32_t>::BuildSIMDSubtree( uint32_t nodeIdx, uint32_t depth )
{
	if (depth == 0)
	{
		threadedBuild = false;
	#ifdef ENABLE_THREADED_BUILDS
		// build in parallel when given a sufficiently large input
		if (triCount >= MT_BUILD_THRESHOLD && context.spawn && context.barrier)
			threadedBuild = true, atomicNewNodePtr = ContextNew<std::atomic<uint32_t>>( newNodePtr );
	#endif
	}
	// aligned data
	constexpr uint32_t maxSlices = 24;
	const uint32_t slices = maxSlices - 2 * depth;
	ALIGNED( 64 ) float32x4x2_t slicebinbox[maxSlices][3 * AVXBINS];
	ALIGNED( 64 ) uint32_t slicecount[maxSlices][NEONCOUNTSTRIDE]; // padded: see NEONCOUNTSTRIDE
	ALIGNED( 64 ) float32x4x2_t bestLBox, bestRBox;            // 64 bytes
	float32x4x2_t* binbox = slicebinbox[0];				// slot 0 doubles as the reduce target
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
			const float32x4_t nodeMin4 = tinybvh_load4( &bvhNode[nodeIdx].aabbMin );
			const float32x4_t nodeMax4 = tinybvh_load4( &bvhNode[nodeIdx].aabbMax );
			// find optimal object split
			const float32x4_t d4 = vbslq_f32( neon_mask3, vsubq_f32( nodeMax4, nodeMin4 ), neon_min1 );
			const float32x4_t nmin4 = vaddq_f32( nodeMin4, nodeMin4 );
			const uint32x4_t nonzero = vmvnq_u32( vceqq_f32( d4, neon_zero4 ) );
			const float32x4_t rpd4 = vreinterpretq_f32_u32( vandq_u32(
				vreinterpretq_u32_f32( vdivq_f32( neon_binmul3, d4 ) ), nonzero ) );
			if (threadedBuild && node.triCount > MT_BUILD_THRESHOLD)
			{
				const uint32_t sliceSize = node.triCount / slices;
				BVHBuildNEONBinSliceArgs args = { this, node.leftFirst, node.triCount, sliceSize, slices,
					slicebinbox[0], slicecount[0], nmin4, rpd4 };
				tinybvh_parallel_for( context, slices, &BVHBuildNEONBinSlice, &args );
				// combine results from slices; slice-major, so each slice is a linear sweep.
				for (uint32_t slice = 1; slice < slices; slice++)
				{
					const float32x4x2_t* sbb = slicebinbox[slice];
					const uint32_t* sc = slicecount[slice];
					for (uint32_t ai = 0; ai < 3 * AVXBINS; ai++)
						count[ai] += sc[ai], binbox[ai] = vmaxq_f32x2( binbox[ai], sbb[ai] );
				}
			}
			else
				// binning runs serially; threading comes from the subtree spawns below.
				BuildSIMDBinTask( node.leftFirst, node.leftFirst + node.triCount, binbox, count, (const float*)&nmin4, (const float*)&rpd4 );
			// calculate per-split totals
			float splitCost = BVH_FAR;
			const float rSAV = 1.0f / SAV;
			uint32_t bestAxis = 0, bestPos = 0;
			const float32x4x2_t* bb = binbox;
			for (int32_t a = 0; a < 3; a++, bb += AVXBINS) if ((node.aabbMax[a] - node.aabbMin[a]) > minDim[a])
			{
				// hardcoded bin processing for AVXBINS == 8
				assert( AVXBINS == 8 );
				const uint32_t* cnt = count + a * AVXBINS;
				const uint32_t lN0 = cnt[0], rN0 = cnt[7];
				const float32x4x2_t lb0 = bb[0], rb0 = bb[7];
				const uint32_t lN1 = lN0 + cnt[1], rN1 = rN0 + cnt[6], lN2 = lN1 + cnt[2];
				const uint32_t rN2 = rN1 + cnt[5], lN3 = lN2 + cnt[3], rN3 = rN2 + cnt[4];
				const float32x4x2_t lb1 = vmaxq_f32x2( lb0, bb[1] ), rb1 = vmaxq_f32x2( rb0, bb[6] );
				const float32x4x2_t lb2 = vmaxq_f32x2( lb1, bb[2] ), rb2 = vmaxq_f32x2( rb1, bb[5] );
				const float32x4x2_t lb3 = vmaxq_f32x2( lb2, bb[3] ), rb3 = vmaxq_f32x2( rb2, bb[4] );
				const uint32_t lN4 = lN3 + cnt[4], rN4 = rN3 + cnt[3], lN5 = lN4 + cnt[5];
				const uint32_t rN5 = rN4 + cnt[2], lN6 = lN5 + cnt[6], rN6 = rN5 + cnt[1];
				const float32x4x2_t lb4 = vmaxq_f32x2( lb3, bb[4] ), rb4 = vmaxq_f32x2( rb3, bb[3] );
				const float32x4x2_t lb5 = vmaxq_f32x2( lb4, bb[5] ), rb5 = vmaxq_f32x2( rb4, bb[2] );
				const float32x4x2_t lb6 = vmaxq_f32x2( lb5, bb[6] ), rb6 = vmaxq_f32x2( rb5, bb[1] );
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
			// in-place partition; must reproduce the binning arithmetic exactly.
			const float rpd = tinybvh_getlane_f( &rpd4, bestAxis ), nmin = tinybvh_getlane_f( &nmin4, bestAxis );
			uint32_t i = node.leftFirst, j = node.leftFirst + node.triCount;
			for (uint32_t k = 0; k < node.triCount; k++)
			{
				const uint32_t fr = primIdx[i];
				const int32_t bi = tinybvh_clamp( (int32_t)((fragment[fr].bmax[bestAxis] +
					fragment[fr].bmin[bestAxis] - nmin) * rpd), 0, AVXBINS - 1 );
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
			tinybvh_store8( &bvhNode[n], veorq_f32x2( bestLBox, neon_signFlip8 ) );
			bvhNode[n].leftFirst = node.leftFirst, bvhNode[n].triCount = leftCount;
			node.leftFirst = n, node.triCount = 0;
			tinybvh_store8( &bvhNode[n + 1], veorq_f32x2( bestRBox, neon_signFlip8 ) );
			bvhNode[n + 1].leftFirst = i, bvhNode[n + 1].triCount = rightCount;
			const bool spawnThreads = tinybvh_max( leftCount, rightCount ) > MT_SPAWN_MIN_PRIMS &&
				depth < MT_SPAWN_DEPTH && threadedBuild;
			if (!spawnThreads) task[taskCount++] = n + 1, nodeIdx = n; else
			{
				// spawn the larger subtree, continue with the small one; root barrier joins.
				impl::BVHBuildSubtreeArgs<float, uint32_t> a = { this, leftCount > rightCount ? n : (n + 1), depth + 1 };
				tinybvh_spawn( context, &BVHBuildNEONSubtree, &a, sizeof( a ) );
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

#ifdef ENABLE_BVH_SOA

// Traverse the second alternative BVH layout (BVH_SoA).
template <> int32_t impl::BVH_SoA<float, uint32_t>::Intersect( Ray& ray ) const
{
	VALIDATE_RAY( ray );
	BVHNode* node = &bvhNode[0], * stack[TINYBVH_STACK_SIZE];
	const bvhvec4slice& verts = bvh.verts;
	const uint32_t* primIdx = bvh.primIdx;
	uint32_t stackPtr = 0;
	float cost = 0;
	const float32x4_t Ox4 = vdupq_n_f32( ray.O.x ), rDx4 = vdupq_n_f32( ray.rD.x );
	const float32x4_t Oy4 = vdupq_n_f32( ray.O.y ), rDy4 = vdupq_n_f32( ray.rD.y );
	const float32x4_t Oz4 = vdupq_n_f32( ray.O.z ), rDz4 = vdupq_n_f32( ray.rD.z );
	// const float32x4_t inf4 = vdupq_n_f32( BVH_FAR );
	while (1)
	{
		cost += c_trav;
		if (node->isLeaf())
		{
			for (uint32_t i = 0; i < node->triCount; i++, cost += c_int)
			{
				const uint32_t tidx = primIdx[node->firstTri + i], vertIdx = tidx * 3;
				const bvhvec3 v0 = verts[vertIdx];
				const bvhvec3 e1 = bvhvec3( verts[vertIdx + 1] ) - v0;
				const bvhvec3 e2 = bvhvec3( verts[vertIdx + 2] ) - v0;
				MOLLER_TRUMBORE_TEST( ray.hit.t, continue );
				ray.hit.t = t, ray.hit.u = u, ray.hit.v = v, ray.hit.prim = tidx;
			}
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		float32x4_t x4 = vmulq_f32( vsubq_f32( vld1q_f32( node->xxxx ), Ox4 ), rDx4 );
		float32x4_t y4 = vmulq_f32( vsubq_f32( vld1q_f32( node->yyyy ), Oy4 ), rDy4 );
		float32x4_t z4 = vmulq_f32( vsubq_f32( vld1q_f32( node->zzzz ), Oz4 ), rDz4 );
		// transpose
		float32x4_t t0 = vzip1q_f32( x4, y4 ), t2 = vzip1q_f32( z4, z4 );
		float32x4_t t1 = vzip2q_f32( x4, y4 ), t3 = vzip2q_f32( z4, z4 );
		float32x4_t xyzw1a = vcombine_f32( vget_low_f32( t0 ), vget_low_f32( t2 ) );
		float32x4_t xyzw2a = vcombine_f32( vget_high_f32( t0 ), vget_high_f32( t2 ) );
		float32x4_t xyzw1b = vcombine_f32( vget_low_f32( t1 ), vget_low_f32( t3 ) );
		float32x4_t xyzw2b = vcombine_f32( vget_high_f32( t1 ), vget_high_f32( t3 ) );
		// process
		float32x4_t tmina4 = vminq_f32( xyzw1a, xyzw2a ), tmaxa4 = vmaxq_f32( xyzw1a, xyzw2a );
		float32x4_t tminb4 = vminq_f32( xyzw1b, xyzw2b ), tmaxb4 = vmaxq_f32( xyzw1b, xyzw2b );
		// transpose back
		t0 = vzip1q_f32( tmina4, tmaxa4 ), t2 = vzip1q_f32( tminb4, tmaxb4 );
		t1 = vzip2q_f32( tmina4, tmaxa4 ), t3 = vzip2q_f32( tminb4, tmaxb4 );
		x4 = vcombine_f32( vget_low_f32( t0 ), vget_low_f32( t2 ) );
		y4 = vcombine_f32( vget_high_f32( t0 ), vget_high_f32( t2 ) );
		z4 = vcombine_f32( vget_low_f32( t1 ), vget_low_f32( t3 ) );
		uint32_t lidx = node->left, ridx = node->right;
		const float32x4_t min4 = vmaxq_f32( vmaxq_f32( vmaxq_f32( x4, y4 ), z4 ), vdupq_n_f32( 0 ) );
		const float32x4_t max4 = vminq_f32( vminq_f32( vminq_f32( x4, y4 ), z4 ), vdupq_n_f32( ray.hit.t ) );
		const float tmina_0 = vgetq_lane_f32( min4, 0 ), tmaxa_1 = vgetq_lane_f32( max4, 1 );
		const float tminb_2 = vgetq_lane_f32( min4, 2 ), tmaxb_3 = vgetq_lane_f32( max4, 3 );
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
	return (int32_t)cost;
}

template <> bool impl::BVH_SoA<float, uint32_t>::IsOccluded( const Ray& ray ) const
{
	BVHNode* node = &bvhNode[0], * stack[TINYBVH_STACK_SIZE];
	const bvhvec4slice& verts = bvh.verts;
	const uint32_t* primIdx = bvh.primIdx;
	uint32_t stackPtr = 0;
	const float32x4_t Ox4 = vdupq_n_f32( ray.O.x ), rDx4 = vdupq_n_f32( ray.rD.x );
	const float32x4_t Oy4 = vdupq_n_f32( ray.O.y ), rDy4 = vdupq_n_f32( ray.rD.y );
	const float32x4_t Oz4 = vdupq_n_f32( ray.O.z ), rDz4 = vdupq_n_f32( ray.rD.z );
	while (1)
	{
		if (node->isLeaf())
		{
			for (uint32_t i = 0; i < node->triCount; i++)
			{
				const uint32_t tidx = primIdx[node->firstTri + i], vertIdx = tidx * 3;
				const bvhvec3 v0 = verts[vertIdx];
				const bvhvec3 e1 = bvhvec3( verts[vertIdx + 1] ) - v0;
				const bvhvec3 e2 = bvhvec3( verts[vertIdx + 2] ) - v0;
				MOLLER_TRUMBORE_TEST( ray.hit.t, continue );
				return true;
			}
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		float32x4_t x4 = vmulq_f32( vsubq_f32( vld1q_f32( node->xxxx ), Ox4 ), rDx4 );
		float32x4_t y4 = vmulq_f32( vsubq_f32( vld1q_f32( node->yyyy ), Oy4 ), rDy4 );
		float32x4_t z4 = vmulq_f32( vsubq_f32( vld1q_f32( node->zzzz ), Oz4 ), rDz4 );
		// transpose
		float32x4_t t0 = vzip1q_f32( x4, y4 ), t2 = vzip1q_f32( z4, z4 );
		float32x4_t t1 = vzip2q_f32( x4, y4 ), t3 = vzip2q_f32( z4, z4 );
		float32x4_t xyzw1a = vcombine_f32( vget_low_f32( t0 ), vget_low_f32( t2 ) );
		float32x4_t xyzw2a = vcombine_f32( vget_high_f32( t0 ), vget_high_f32( t2 ) );
		float32x4_t xyzw1b = vcombine_f32( vget_low_f32( t1 ), vget_low_f32( t3 ) );
		float32x4_t xyzw2b = vcombine_f32( vget_high_f32( t1 ), vget_high_f32( t3 ) );
		// process
		float32x4_t tmina4 = vminq_f32( xyzw1a, xyzw2a ), tmaxa4 = vmaxq_f32( xyzw1a, xyzw2a );
		float32x4_t tminb4 = vminq_f32( xyzw1b, xyzw2b ), tmaxb4 = vmaxq_f32( xyzw1b, xyzw2b );
		// transpose back
		t0 = vzip1q_f32( tmina4, tmaxa4 ), t2 = vzip1q_f32( tminb4, tmaxb4 );
		t1 = vzip2q_f32( tmina4, tmaxa4 ), t3 = vzip2q_f32( tminb4, tmaxb4 );
		x4 = vcombine_f32( vget_low_f32( t0 ), vget_low_f32( t2 ) );
		y4 = vcombine_f32( vget_high_f32( t0 ), vget_high_f32( t2 ) );
		z4 = vcombine_f32( vget_low_f32( t1 ), vget_low_f32( t3 ) );
		uint32_t lidx = node->left, ridx = node->right;
		const float32x4_t min4 = vmaxq_f32( vmaxq_f32( vmaxq_f32( x4, y4 ), z4 ), vdupq_n_f32( 0 ) );
		const float32x4_t max4 = vminq_f32( vminq_f32( vminq_f32( x4, y4 ), z4 ), vdupq_n_f32( ray.hit.t ) );
		const float tmina_0 = vgetq_lane_f32( min4, 0 ), tmaxa_1 = vgetq_lane_f32( max4, 1 );
		const float tminb_2 = vgetq_lane_f32( min4, 2 ), tmaxb_3 = vgetq_lane_f32( max4, 3 );
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

// BVH4_CPU traversal, NEON version.
// ----------------------------------------------------------------------------
// The next node is selected with predicted branches on the slab mask.

// Substitute for _mm_movemask_ps. The lane weights place the lane mask in the
// low nibble of the horizontal sum and the number of set lanes in the high nibble.
static const uint32x4_t neon_laneBits = SIMD_SETRVECU( 0x11, 0x12, 0x14, 0x18 );
TINYBVH_FORCEINLINE uint32_t neon_movemask_popc( const uint32x4_t mask )
{
	return vaddvq_u32( vandq_u32( mask, neon_laneBits ) );
}

#define NEON_HIT( s ) ((m64 >> (16 * s)) & 1)
#define NEON_PUSH( c, s ) { nodeStack[stackPtr] = c; vst1q_lane_f32( distStack + stackPtr, tminSorted, s ); stackPtr++; }

template <> template <bool posX, bool posY, bool posZ> int32_t impl::BVH4_CPU<float, uint32_t>::IntersectOctant( Ray& ray ) const
{
	ALIGNED( 64 ) uint32_t nodeStack[TINYBVH_STACK_SIZE * 2 /* wide trees push more nodes per step */];
	ALIGNED( 64 ) float distStack[TINYBVH_STACK_SIZE * 2];
	const float32x4_t zero4 = vdupq_n_f32( 0 ), one4 = vdupq_n_f32( 1 ), inf4 = vdupq_n_f32( 1e34f );
	float32x4_t t4 = vdupq_n_f32( ray.hit.t );
	int32_t stackPtr = 0;
	uint32_t nodeIdx = 0;
	float tcur = ray.hit.t;
	constexpr int signShift = (posX ? 2 : 0) + (posY ? 4 : 0) + (posZ ? 8 : 0);
	// the slab test computes bound * rD - O * rD using a fused multiply-add.
	const float32x4_t rx4 = vdupq_n_f32( -ray.O.x * ray.rD.x ), rdx4 = vdupq_n_f32( ray.rD.x );
	const float32x4_t ry4 = vdupq_n_f32( -ray.O.y * ray.rD.y ), rdy4 = vdupq_n_f32( ray.rD.y );
	const float32x4_t rz4 = vdupq_n_f32( -ray.O.z * ray.rD.z ), rdz4 = vdupq_n_f32( ray.rD.z );
	const float32x4_t ox4 = vdupq_n_f32( ray.O.x ), oy4 = vdupq_n_f32( ray.O.y ), oz4 = vdupq_n_f32( ray.O.z );
	const float32x4_t dx4 = vdupq_n_f32( ray.D.x ), dy4 = vdupq_n_f32( ray.D.y ), dz4 = vdupq_n_f32( ray.D.z );
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
			const float32x4_t tx1 = vfmaq_f32( rx4, vld1q_f32( posX ? n->xmin : n->xmax ), rdx4 );
			const float32x4_t ty1 = vfmaq_f32( ry4, vld1q_f32( posY ? n->ymin : n->ymax ), rdy4 );
			const float32x4_t tz1 = vfmaq_f32( rz4, vld1q_f32( posZ ? n->zmin : n->zmax ), rdz4 );
			const float32x4_t tx2 = vfmaq_f32( rx4, vld1q_f32( posX ? n->xmax : n->xmin ), rdx4 );
			const float32x4_t ty2 = vfmaq_f32( ry4, vld1q_f32( posY ? n->ymax : n->ymin ), rdy4 );
			const float32x4_t tz2 = vfmaq_f32( rz4, vld1q_f32( posZ ? n->zmax : n->zmin ), rdz4 );
			const float32x4_t tmin = vmaxq_f32( vmaxq_f32( tx1, ty1 ), vmaxq_f32( tz1, zero4 ) );
			const float32x4_t tmax = vminq_f32( vminq_f32( tx2, ty2 ), vminq_f32( tz2, t4 ) );
			const uint32x4_t mask4 = vcleq_f32( tmin, tmax );
			// child index at each sorted position
			const uint32_t c3 = child[(perm[3] >> signShift) & 3], c2 = child[(perm[2] >> signShift) & 3];
			const uint32_t c1 = child[(perm[1] >> signShift) & 3], c0 = child[(perm[0] >> signShift) & 3];
			// byte shuffle that brings the lanes into sorted order
			const uint32x4_t perm4 = vld1q_u32( perm );
			const uint32x4_t order4 = vshrq_n_u32( vshlq_n_u32( perm4, 30 - signShift ), 30 );
			const uint8x16_t shfl16 = vreinterpretq_u8_u32( vmlaq_u32( add4, order4, mul4 ) );
			// sorted slab mask, 16 bits per position
			const uint32x4_t sorted4 = vreinterpretq_u32_u8( vqtbl1q_u8( vreinterpretq_u8_u32( mask4 ), shfl16 ) );
			const uint64_t m64 = vget_lane_u64( vreinterpret_u64_u16( vshrn_n_u32( sorted4, 16 ) ), 0 );
			const float32x4_t tminSorted = vreinterpretq_f32_u8( vqtbl1q_u8( vreinterpretq_u8_f32( tmin ), shfl16 ) );
			// continue with the nearest valid child and push the others, farthest first
			if (NEON_HIT( 3 ))
			{
				if (NEON_HIT( 0 )) NEON_PUSH( c0, 0 );
				if (NEON_HIT( 1 )) NEON_PUSH( c1, 1 );
				if (NEON_HIT( 2 )) NEON_PUSH( c2, 2 );
				nodeIdx = c3;
			}
			else if (NEON_HIT( 2 ))
			{
				if (NEON_HIT( 0 )) NEON_PUSH( c0, 0 );
				if (NEON_HIT( 1 )) NEON_PUSH( c1, 1 );
				nodeIdx = c2;
			}
			else if (NEON_HIT( 1 ))
			{
				if (NEON_HIT( 0 )) NEON_PUSH( c0, 0 );
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
		const float32x4_t hx4 = vfmsq_f32( vmulq_f32( dy4, vld1q_f32( leaf->e2z ) ), dz4, vld1q_f32( leaf->e2y ) );
		const float32x4_t hy4 = vfmsq_f32( vmulq_f32( dz4, vld1q_f32( leaf->e2x ) ), dx4, vld1q_f32( leaf->e2z ) );
		const float32x4_t hz4 = vfmsq_f32( vmulq_f32( dx4, vld1q_f32( leaf->e2y ) ), dy4, vld1q_f32( leaf->e2x ) );
		const float32x4_t sx4 = vsubq_f32( ox4, vld1q_f32( leaf->v0x ) ), sy4 = vsubq_f32( oy4, vld1q_f32( leaf->v0y ) ), sz4 = vsubq_f32( oz4, vld1q_f32( leaf->v0z ) );
		const float32x4_t det4 = vfmaq_f32( vfmaq_f32( vmulq_f32( vld1q_f32( leaf->e1y ), hy4 ), vld1q_f32( leaf->e1x ), hx4 ), vld1q_f32( leaf->e1z ), hz4 );
		const float32x4_t qz4 = vfmsq_f32( vmulq_f32( sx4, vld1q_f32( leaf->e1y ) ), sy4, vld1q_f32( leaf->e1x ) );
		const float32x4_t qx4 = vfmsq_f32( vmulq_f32( sy4, vld1q_f32( leaf->e1z ) ), sz4, vld1q_f32( leaf->e1y ) );
		const float32x4_t qy4 = vfmsq_f32( vmulq_f32( sz4, vld1q_f32( leaf->e1x ) ), sx4, vld1q_f32( leaf->e1z ) );
		const float32x4_t inv_det4 = vdivq_f32( one4, det4 );
		const float32x4_t u4 = vmulq_f32( vfmaq_f32( vfmaq_f32( vmulq_f32( sy4, hy4 ), sx4, hx4 ), sz4, hz4 ), inv_det4 );
		const float32x4_t v4 = vmulq_f32( vfmaq_f32( vfmaq_f32( vmulq_f32( dy4, qy4 ), dx4, qx4 ), dz4, qz4 ), inv_det4 );
		const float32x4_t ta4 = vmulq_f32( vfmaq_f32( vfmaq_f32( vmulq_f32( vld1q_f32( leaf->e2y ), qy4 ), vld1q_f32( leaf->e2x ), qx4 ), vld1q_f32( leaf->e2z ), qz4 ), inv_det4 );
		const uint32x4_t mask1 = vandq_u32( vcgeq_f32( u4, zero4 ), vcgeq_f32( v4, zero4 ) );
		const uint32x4_t mask2 = vcleq_f32( vaddq_f32( u4, v4 ), one4 );
		const uint32x4_t mask3 = vandq_u32( vcltq_f32( ta4, t4 ), vcgtq_f32( ta4, zero4 ) );
		uint32x4_t combined = vandq_u32( vandq_u32( mask1, mask2 ), mask3 );
		uint32_t imask = neon_movemask_popc( combined ) & 15;
		// evaluate opacity map, if present (NEON version).
		if (opmap) if (imask)
		{
			const float32x4_t fN4 = vdupq_n_f32( (float)opmapN );
			const int32x4_t row4 = vcvtq_s32_f32( vmulq_f32( vaddq_f32( u4, v4 ), fN4 ) );
			const int32x4_t dia4 = vcvtq_s32_f32( vmulq_f32( vsubq_f32( one4, u4 ), fN4 ) );
			const int32x4_t v0 = vmulq_s32( row4, row4 );
			const int32x4_t v1 = vcvtq_s32_f32( vmulq_f32( v4, fN4 ) );
			const int32x4_t v2 = vsubq_s32( dia4, vsubq_s32( vdupq_n_s32( opmapN - 1 ), row4 ) );
			uint32_t idx[4], omask[4] = { 0, 0, 0, 0 };
			vst1q_u32( idx, vreinterpretq_u32_s32( vaddq_s32( vaddq_s32( v0, v1 ), v2 ) ) );
			// gather the opacity bits with scalar loads
			for (int i = 0; i < 4; i++) if (imask & (1 << i))
			{
				uint32_t* om = opmap + leaf->primIdx[i] * ((opmapN * opmapN + 31) >> 5);
				if (om[idx[i] >> 5] & (1 << (idx[i] & 31))) omask[i] = 0xffffffff;
			}
			// combine
			combined = vandq_u32( combined, vld1q_u32( omask ) );
			imask = neon_movemask_popc( combined ) & 15;
		}
		if (imask)
		{
			const float32x4_t dist4 = vbslq_f32( combined, ta4, inf4 );
			const float t = vminvq_f32( dist4 );
			const uint32_t lane = __bfind( neon_movemask_popc( vceqq_f32( dist4, vdupq_n_f32( t ) ) ) & 15 );
			// update hit record
			ray.hit.t = t, ray.hit.u = tinybvh_getlane_f( &u4, lane ), ray.hit.v = tinybvh_getlane_f( &v4, lane );
		#if INST_IDX_BITS == 32
			ray.hit.prim = leaf->primIdx[lane], ray.hit.inst = ray.instIdx;
		#else
			ray.hit.prim = leaf->primIdx[lane] + ray.instIdx;
		#endif
			t4 = vdupq_n_f32( t ), tcur = t;
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

template <> template <bool posX, bool posY, bool posZ> bool impl::BVH4_CPU<float, uint32_t>::IsOccludedOctant( const Ray& ray ) const
{
	ALIGNED( 64 ) uint32_t nodeStack[TINYBVH_STACK_SIZE * 2 /* wide trees push more nodes per step */];
	int32_t stackPtr = 0;
	uint32_t nodeIdx = 0;
	const float32x4_t zero4 = vdupq_n_f32( 0 ), one4 = vdupq_n_f32( 1 ), t4 = vdupq_n_f32( ray.hit.t );
	const float32x4_t rx4 = vdupq_n_f32( -ray.O.x * ray.rD.x ), rdx4 = vdupq_n_f32( ray.rD.x );
	const float32x4_t ry4 = vdupq_n_f32( -ray.O.y * ray.rD.y ), rdy4 = vdupq_n_f32( ray.rD.y );
	const float32x4_t rz4 = vdupq_n_f32( -ray.O.z * ray.rD.z ), rdz4 = vdupq_n_f32( ray.rD.z );
	const float32x4_t ox4 = vdupq_n_f32( ray.O.x ), oy4 = vdupq_n_f32( ray.O.y ), oz4 = vdupq_n_f32( ray.O.z );
	const float32x4_t dx4 = vdupq_n_f32( ray.D.x ), dy4 = vdupq_n_f32( ray.D.y ), dz4 = vdupq_n_f32( ray.D.z );
	while (1)
	{
		while (!(nodeIdx & LEAF_BIT))
		{
			const BVHNode* n = (BVHNode*)(bvh4Data + nodeIdx);
			const uint32_t* child = n->child;
			const float32x4_t tx1 = vfmaq_f32( rx4, vld1q_f32( posX ? n->xmin : n->xmax ), rdx4 );
			const float32x4_t ty1 = vfmaq_f32( ry4, vld1q_f32( posY ? n->ymin : n->ymax ), rdy4 );
			const float32x4_t tz1 = vfmaq_f32( rz4, vld1q_f32( posZ ? n->zmin : n->zmax ), rdz4 );
			const float32x4_t tx2 = vfmaq_f32( rx4, vld1q_f32( posX ? n->xmax : n->xmin ), rdx4 );
			const float32x4_t ty2 = vfmaq_f32( ry4, vld1q_f32( posY ? n->ymax : n->ymin ), rdy4 );
			const float32x4_t tz2 = vfmaq_f32( rz4, vld1q_f32( posZ ? n->zmax : n->zmin ), rdz4 );
			const float32x4_t tmin = vmaxq_f32( vmaxq_f32( tx1, ty1 ), vmaxq_f32( tz1, zero4 ) );
			const float32x4_t tmax = vminq_f32( vminq_f32( tx2, ty2 ), vminq_f32( tz2, t4 ) );
			const uint32x4_t mask4 = vcleq_f32( tmin, tmax );
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
		const float32x4_t hx4 = vfmsq_f32( vmulq_f32( dy4, vld1q_f32( leaf->e2z ) ), dz4, vld1q_f32( leaf->e2y ) );
		const float32x4_t hy4 = vfmsq_f32( vmulq_f32( dz4, vld1q_f32( leaf->e2x ) ), dx4, vld1q_f32( leaf->e2z ) );
		const float32x4_t hz4 = vfmsq_f32( vmulq_f32( dx4, vld1q_f32( leaf->e2y ) ), dy4, vld1q_f32( leaf->e2x ) );
		const float32x4_t sx4 = vsubq_f32( ox4, vld1q_f32( leaf->v0x ) ), sy4 = vsubq_f32( oy4, vld1q_f32( leaf->v0y ) ), sz4 = vsubq_f32( oz4, vld1q_f32( leaf->v0z ) );
		const float32x4_t det4 = vfmaq_f32( vfmaq_f32( vmulq_f32( vld1q_f32( leaf->e1y ), hy4 ), vld1q_f32( leaf->e1x ), hx4 ), vld1q_f32( leaf->e1z ), hz4 );
		const float32x4_t qz4 = vfmsq_f32( vmulq_f32( sx4, vld1q_f32( leaf->e1y ) ), sy4, vld1q_f32( leaf->e1x ) );
		const float32x4_t qx4 = vfmsq_f32( vmulq_f32( sy4, vld1q_f32( leaf->e1z ) ), sz4, vld1q_f32( leaf->e1y ) );
		const float32x4_t qy4 = vfmsq_f32( vmulq_f32( sz4, vld1q_f32( leaf->e1x ) ), sx4, vld1q_f32( leaf->e1z ) );
		const float32x4_t inv_det4 = vdivq_f32( one4, det4 );
		const float32x4_t u4 = vmulq_f32( vfmaq_f32( vfmaq_f32( vmulq_f32( sy4, hy4 ), sx4, hx4 ), sz4, hz4 ), inv_det4 );
		const float32x4_t v4 = vmulq_f32( vfmaq_f32( vfmaq_f32( vmulq_f32( dy4, qy4 ), dx4, qx4 ), dz4, qz4 ), inv_det4 );
		const float32x4_t ta4 = vmulq_f32( vfmaq_f32( vfmaq_f32( vmulq_f32( vld1q_f32( leaf->e2y ), qy4 ), vld1q_f32( leaf->e2x ), qx4 ), vld1q_f32( leaf->e2z ), qz4 ), inv_det4 );
		const uint32x4_t mask1 = vandq_u32( vcgeq_f32( u4, zero4 ), vcgeq_f32( v4, zero4 ) );
		const uint32x4_t mask2 = vcleq_f32( vaddq_f32( u4, v4 ), one4 );
		const uint32x4_t mask3 = vandq_u32( vcltq_f32( ta4, t4 ), vcgtq_f32( ta4, zero4 ) );
		const uint32x4_t combined = vandq_u32( vandq_u32( mask1, mask2 ), mask3 );
		const uint32_t imask = neon_movemask_popc( combined ) & 15;
		if (imask)
		{
			if (!opmap) return true;
			// evaluate opacity map, NEON version.
			const float32x4_t fN4 = vdupq_n_f32( (float)opmapN );
			const int32x4_t row4 = vcvtq_s32_f32( vmulq_f32( vaddq_f32( u4, v4 ), fN4 ) );
			const int32x4_t dia4 = vcvtq_s32_f32( vmulq_f32( vsubq_f32( one4, u4 ), fN4 ) );
			const int32x4_t v0 = vmulq_s32( row4, row4 );
			const int32x4_t v1 = vcvtq_s32_f32( vmulq_f32( v4, fN4 ) );
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

#endif // TINYBVH_IMPLEMENTATION
