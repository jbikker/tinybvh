// ---------------------------------------------------------------------------
// Geometric triangle pre-splitting, for the DXR BLAS.
//
// BVH::Presplit() in tiny_bvh.h splits *references*, not geometry: it cuts a
// Fragment (an AABB that points back at one triangle) in two and lets the
// builder use the two tighter boxes. The triangle itself is untouched; a leaf
// just stores the original primitive index, and PresplitPostPass() cleans up
// leafs that ended up referencing the same triangle twice.
//
// A DXR BLAS cannot be handed a fragment - it is built from a vertex buffer -
// so the only way to give the hardware builder tighter leaf bounds is to
// actually cut the triangle into smaller triangles. That is what this does.
// The split *decisions* (priority, budget, and the power-of-two split-plane
// snapping) are lifted from BVH::Presplit(); only the payload differs: a real
// polygon instead of a box.
//
// The surface is unchanged, so the traced depth image must be bit-identical
// before and after. What changes is BLAS quality, BLAS size and build time.
// ---------------------------------------------------------------------------

// A triangle clipped by k axis-aligned planes has at most 3 + k vertices. 12
// covers a piece that survived nine cuts; deeper than that we stop splitting
// it, which costs a bit of budget but never correctness.
static constexpr int MAX_POLY_VERTS = 12;

struct SplitPoly
{
	bvhvec3 v[MAX_POLY_VERTS];
	bvhvec3 bmin, bmax;
	int n;
	void CalcBounds()
	{
		bmin = bvhvec3( BVH_FAR ), bmax = bvhvec3( -BVH_FAR );
		for (int i = 0; i < n; i++) bmin = tinybvh_min( bmin, v[i] ), bmax = tinybvh_max( bmax, v[i] );
	}
};

// Sutherland-Hodgman clip of a convex polygon against an axis-aligned plane,
// keeping *both* halves. Each crossing point is computed once and handed to
// both outputs, so the two halves share bit-identical vertices along the seam
// and the cut introduces no crack. Returns false if either half came out
// degenerate (plane missed the polygon) or a half would overflow.
static bool SplitPolyAt( const SplitPoly& in, SplitPoly& lo, SplitPoly& hi, const int axis, const float pos )
{
	lo.n = hi.n = 0;
	for (int i = 0; i < in.n; i++)
	{
		if (lo.n + 2 > MAX_POLY_VERTS || hi.n + 2 > MAX_POLY_VERTS) return false;
		const bvhvec3& a = in.v[i], & b = in.v[(i + 1) % in.n];
		const bool aLo = a[axis] <= pos, bLo = b[axis] <= pos;
		if (aLo) lo.v[lo.n++] = a; else hi.v[hi.n++] = a;
		if (aLo != bLo) // edge crosses the plane: emit the crossing to both sides
		{
			bvhvec3 c = a + (pos - a[axis]) / (b[axis] - a[axis]) * (b - a);
			c[axis] = pos; // exactly on the split plane
			lo.v[lo.n++] = c, hi.v[hi.n++] = c;
		}
	}
	if (lo.n < 3 || hi.n < 3) return false; // everything landed on one side
	lo.CalcBounds(), hi.CalcBounds();
	return true;
}

// BVH::SplitPriority: big *and* mostly empty boxes are worth cutting.
static float PolySplitPriority( const SplitPoly& p, const float triArea )
{
	const bvhvec3 extent = p.bmax - p.bmin;
	const float extentPrio = tinybvh_sqrf( extent[tinybvh_maxdim( extent )] );
	const float boxArea = 2 * tinybvh_halfarea( extent );
	return cbrtf( extentPrio * tinybvh_max( boxArea - triArea, 0.0f ) );
}

// BVH::GetNodeSize: round the extent down to a power-of-two fraction of the
// scene extent, so split planes from different triangles line up on a grid.
static float PresplitNodeSize( const float extent, const float globalSize )
{
	float alpha = extent / globalSize;
	uint32_t exponentBits = (*(uint32_t*)&alpha) & (255u << 23);
	return *(float*)&exponentBits * globalSize;
}

// Cut 'srcTris' triangles into a denser soup, spending a budget of
// factor * srcTris extra pieces on the triangles that need it most. Returns a
// _aligned_malloc'd triangle soup (3 bvhvec4 per triangle); caller owns it.
static bvhvec4* PresplitTriangles( const bvhvec4* src, const int srcTris, const float factor, int& dstTris )
{
	const int splitBudget = (int)(srcTris * factor), maxPieces = srcTris + splitBudget;
	SplitPoly* piece = new SplitPoly[maxPieces];
	int* splits = new int[maxPieces];
	float* prio = new float[srcTris];
	float* area = new float[srcTris];
	// one polygon per input triangle, plus the scene bounds
	bvhvec3 rootMin( BVH_FAR ), rootMax( -BVH_FAR );
	for (int i = 0; i < srcTris; i++)
	{
		SplitPoly& p = piece[i];
		p.n = 3, p.v[0] = src[i * 3], p.v[1] = src[i * 3 + 1], p.v[2] = src[i * 3 + 2];
		p.CalcBounds();
		rootMin = tinybvh_min( rootMin, p.bmin ), rootMax = tinybvh_max( rootMax, p.bmax );
		area[i] = 0.5f * tinybvh_length( tinybvh_cross( p.v[1] - p.v[0], p.v[2] - p.v[0] ) );
	}
	const bvhvec3 rootExtent = rootMax - rootMin;
	// hand every input triangle a split count, backing off until it fits the budget
	float f = factor;
	while (1)
	{
		double summedPrio = 0;
		for (int i = 0; i < srcTris; i++) prio[i] = PolySplitPriority( piece[i], area[i] ), summedPrio += prio[i];
		int64_t total = 0;
		if (summedPrio <= 0) { for (int i = 0; i < srcTris; i++) splits[i] = 1; break; } // degenerate input
		for (int i = 0; i < srcTris; i++)
		{
			const float shareOfTris = (float)(prio[i] / summedPrio) * srcTris;
			splits[i] = 1 + (int)(shareOfTris * f), total += splits[i];
		}
		if (total <= maxPieces) break;
		f *= 0.95f;
	}
	// A piece with splits > 1 gets cut in two; the remaining budget goes to the
	// halves in proportion to their size. Both counters strictly decrease, so
	// this terminates, and the total number of pieces is the budgeted total.
	int pieces = srcTris;
	for (int i = 0; i < pieces; ) if (splits[i] == 1) i++; else
	{
		const SplitPoly& p = piece[i];
		const bvhvec3 extent = p.bmax - p.bmin;
		const int axis = (int)tinybvh_maxdim( extent );
		float nodeSize = PresplitNodeSize( extent[axis], rootExtent[axis] );
		if (nodeSize >= extent[axis] - 0.0001f) nodeSize *= 0.5f;
		const float midPos = (p.bmin[axis] + p.bmax[axis]) * 0.5f;
		const float index = roundf( (midPos - rootMin[axis]) / nodeSize );
		const float splitPos = rootMin[axis] + index * nodeSize;
		SplitPoly lo, hi;
		// the snapped plane can fall outside the piece; a median cut then a
		// give-up are the fallbacks, so a piece is never lost.
		if (!SplitPolyAt( p, lo, hi, axis, splitPos ) && !SplitPolyAt( p, lo, hi, axis, midPos ))
		{
			splits[i] = 1;
			continue;
		}
		const int toDivide = splits[i];
		const bvhvec3 loExt = lo.bmax - lo.bmin, hiExt = hi.bmax - hi.bmin;
		const float loSize = loExt[tinybvh_maxdim( loExt )], hiSize = hiExt[tinybvh_maxdim( hiExt )];
		const int loCount = (int)((float)toDivide * loSize / (loSize + hiSize));
		splits[i] = tinybvh_clamp( loCount, 1, toDivide - 1 );
		splits[pieces] = toDivide - splits[i];
		piece[i] = lo, piece[pieces] = hi;
		pieces++;
	}
	// fan-triangulate the pieces into a plain soup
	int outTris = 0;
	for (int i = 0; i < pieces; i++) outTris += piece[i].n - 2;
	bvhvec4* out = (bvhvec4*)_aligned_malloc( (size_t)outTris * 48, 64 );
	int w = 0;
	for (int i = 0; i < pieces; i++) for (int j = 1; j < piece[i].n - 1; j++)
		out[w++] = bvhvec4( piece[i].v[0], 0 ),
		out[w++] = bvhvec4( piece[i].v[j], 0 ),
		out[w++] = bvhvec4( piece[i].v[j + 1], 0 );
	delete[] piece, delete[] splits, delete[] prio, delete[] area;
	dstTris = outTris;
	return out;
}
