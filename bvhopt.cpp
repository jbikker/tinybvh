// bvhopt - offline BVH quality optimizer for TinyBVH.
//
// Reads a BVH saved with BVH::Save, spends an unbounded amount of time lowering its SAH and
// EPO cost, and writes the result next to the input with "_opt" appended to the stem.
//
// The optimizer is deliberately outside the library: it is slow, it is only useful offline,
// and none of it belongs in a header that people include to build a BVH in 90 ms. It touches
// TinyBVH only through public members of BVH_Verbose, plus BVH_Verbose::Optimize for the
// insertion-based stage, which is the one piece of the pipeline the library already has.
//
// Usage: bvhopt <file.bvh> [mesh.bin] [options]
//
// A saved BVH does not carry geometry - BVH::Load takes the vertices from the caller - so the
// mesh is needed too, both to reload the BVH and because EPO is defined over the triangles.
// It defaults to the BVH path with its extension replaced by .bin. The mesh format is the
// obvious one: a uint32 triangle count followed by three float4 vertices per triangle.
//
// Build: g++ -O2 -mavx2 -mfma -std=c++17 -o bvhopt bvhopt.cpp

#define TINYBVH_IMPLEMENTATION
#include "tiny_bvh.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>
#include <vector>
#include <algorithm>

using tinybvh::bvhvec3;
using tinybvh::bvhvec4;

static const uint32_t MAX_TREELET = 12;
// MAX_EXACT bounds the exact bottom-subtree DP. The recurrence enumerates 3^n subset pairs and
// holds 2^n costs plus 2^n boxes: at 14 that is 4.8M partition evaluations and ~520 KB of tables
// per subtree, the last size that stays inside a core's L2 and keeps a whole-scene pass in the
// minutes rather than the hours. Returns die well before it; 10 is the useful setting.
static const uint32_t MAX_EXACT = 14;
static const uint32_t STACK_SIZE = TINYBVH_STACK_SIZE;
static const float wEPO = W_EPO;

typedef std::chrono::high_resolution_clock Clock;
static double MilliSec( Clock::time_point a, Clock::time_point b )
{
	return std::chrono::duration<double, std::milli>( b - a ).count();
}

static float SurfaceArea( const bvhvec3& bmin, const bvhvec3& bmax )
{
	const bvhvec3 e = bmax - bmin;
	return e.x * e.y + e.y * e.z + e.z * e.x;
}

class Optimizer
{
public:
	typedef tinybvh::BVH_Verbose::BVHNode Node;

	// One reference: a primitive, clipped to the box it occupies. A spatial-split build has
	// several of these per primitive; the library discards them once the build is done, so
	// they are recovered here by clipping each primitive to the leaf that holds it.
	struct Ref
	{
		bvhvec3 bmin; uint32_t primIdx;
		bvhvec3 bmax; uint32_t clipped;
		void Extend( const bvhvec3& p ) { bmin = tinybvh::tinybvh_min( bmin, p ), bmax = tinybvh::tinybvh_max( bmax, p ); }
	};

	struct Settings
	{
		uint32_t rounds = 16;			// outer rounds; stops earlier once a round no longer pays.
		float epsilon = 1e-4f;			// relative gain a round must deliver to justify another.
		uint32_t seed = 0x12345678u;	// all randomization derives from this; runs are reproducible.
		uint32_t reinsertIters = 100;	// BVH_Verbose::Optimize iterations per round, 'extreme' mode; 0 disables.
		uint32_t treeletSize = 7;		// treelet leaves for the bulk of the tree; DP is O(3^n) per treelet.
		uint32_t treeletSizeTop = 10;	// ..and for the nodes that carry most of the cost. At most MAX_TREELET.
		uint32_t treeletSweeps = 2;		// bottom-up treelet sweeps per round; 0 disables.
		uint32_t rebuildSubtrees = 0;	// worst subtrees rebuilt from scratch per round; 0 disables.
		uint32_t rebuildMin = 64;		// ..considering only subtrees with at least this many references,
		uint32_t rebuildMax = 8192;		// ..and at most this many.
		uint32_t rebuildAttempts = 8;	// seeded attempts per subtree; attempt 0 is the plain greedy sweep.
		float rebuildJitter = 0.05f;	// cost perturbation for attempts 1..n: turbulence, not bias.
		bool unsplitRefs = true;		// merge spatial split references that no longer pay for themselves.
		bool explodeLeaves = true;		// break the input's leaves down to one reference each first, so that
										// no leaf is an immovable atom; maxLeafPrims restores them at the end.
		uint32_t exactBottomSize = 10;	// replace maximal subtrees of at most this many references by the
										// provably optimal BVH over them. 0 disables; at most MAX_EXACT.
		uint32_t maxLeafPrims = 4;		// final leaf collapse budget. 1 for a pure BVH2; 4 for BVH4_CPU.
		// EPO predicts traversal cost better than SAH but costs three orders of magnitude more to
		// evaluate, so it enters at two granularities only: an exact local test at the top of the
		// tree, where one node's box covers enough of the scene for the trade to matter, and an
		// exact whole-tree gate once per round.
		float epoProxy = 0.0f;			// sibling-overlap term in the inner objective. Measured harmful
										// as a global objective; it exists to generate candidates.
		uint32_t epoTreeletTop = 1024;	// exact EPO acceptance for interior nodes holding at least
										// refCount/epoTreeletTop references; 0 disables.
		uint32_t epoCandidates = 4;		// topologies offered to that test, generated by re-running the DP
										// under different overlap weights. 1 = veto only, no alternatives.
		bool epoGate = false;			// evaluate EPO once per round and undo a round that did not pay.
		bool verbose = true;			// report progress per round.
	};

	Optimizer( tinybvh::BVH_Verbose& bvh, const Settings& set ) : b( bvh ), s( set ) {}
	void Run( const Ref* refBox, const uint32_t* vertIdx );

private:
	struct State
	{
		float* subCost;			// unnormalized SAH cost of each node's subtree
		uint32_t* refCount;		// references below each node
		uint32_t* order;		// interior nodes in pre-order; reverse it to walk bottom-up
		uint32_t orderCount;
		uint32_t* work;			// scratch: node index lists
		float* workF;			// scratch: suffix areas
		uint32_t* workI;		// scratch: suffix reference counts
		float* tCost;			// treelet DP: cost per subset
		uint32_t* tPart;		// treelet DP: winning partition per subset
		bvhvec3* tMin, * tMax;	// treelet DP: bounds per subset
		uint32_t seed;
		const uint32_t* vertIdx;
		float epoProxy, primArea, rootArea;
		uint32_t epoTopRefs, epoCandidates;
	};
	void OptExRefitUp( uint32_t nodeIdx );
	void OptExComputeCosts( State& st );
	uint32_t OptExSweepSplit( uint32_t* leaf, const uint32_t count, const float jitter, uint32_t& seed, State& st );
	uint32_t OptExBuildTopology( uint32_t* leaf, const uint32_t count, uint32_t* slots, uint32_t& slotPtr,
		const uint32_t parent, const float jitter, uint32_t& seed, float& interiorCost, State& st );
	void OptExExplode( const Ref* refBox, State& st );
	bool OptExTreelet( const uint32_t nodeIdx, const uint32_t maxL, const bool epoCheck, State& st );
	void OptExTreeletSweep( const Settings& s, State& st );
	void OptExRebuildSubtrees( const Settings& s, State& st );
	void OptExUnsplit( State& st );
	void OptExCompactRefs();
	void OptExCollapse( const uint32_t maxLeafPrims, State& st );
	void OptExExactBottom( const Settings& s, State& st );
	float OptExOverlap( const uint32_t na, const uint32_t nb ) const;
	float OptExEPOArea( const uint32_t subtreeRoot, const uint32_t nodeIdx, const uint32_t* vertIdx ) const;
	float OptExPrimAreaSum( const uint32_t* vertIdx ) const;
	float OptExCombined( const uint32_t* vertIdx, const float primArea, float& sahOut ) const;
	float OptExLocalEPO( const uint32_t* nodes, const uint32_t count, const uint32_t* vertIdx,
		const float primArea, const float rootArea ) const;

	void Progress( const uint32_t round, const Clock::time_point& t0, const char* stage );

	tinybvh::BVH_Verbose& b;
	Settings s;
	const uint32_t* epoVertIdx = 0;
	float epoPrimArea = 1;
	bool holes = false;		// nodes made unreachable by a stage; cleared by the next Compact()
	float lastEPO = 0;
};

void Optimizer::OptExRefitUp( uint32_t nodeIdx )
{
	while (b.bvhNode[nodeIdx].parent != (uint32_t)-1)
	{
		const uint32_t p = b.bvhNode[nodeIdx].parent;
		Node& P = b.bvhNode[p];
		const bvhvec3 bmin = tinybvh::tinybvh_min( b.bvhNode[P.left].aabbMin, b.bvhNode[P.right].aabbMin );
		const bvhvec3 bmax = tinybvh::tinybvh_max( b.bvhNode[P.left].aabbMax, b.bvhNode[P.right].aabbMax );
		if (bmin.x == P.aabbMin.x && bmin.y == P.aabbMin.y && bmin.z == P.aabbMin.z &&
			bmax.x == P.aabbMax.x && bmax.y == P.aabbMax.y && bmax.z == P.aabbMax.z) return;
		P.aabbMin = bmin, P.aabbMax = bmax, nodeIdx = p;
	}
}

// OptExComputeCosts: subtree cost and reference count for every node, plus a pre-order list
// of the interior nodes. Walking that list backwards visits children before their parents.
void Optimizer::OptExComputeCosts( State& st )
{
	st.orderCount = 0;
	uint32_t stack[STACK_SIZE], stackPtr = 0, nodeIdx = 0;
	while (1)
	{
		const Node& n = b.bvhNode[nodeIdx];
		if (n.isLeaf())
		{
			st.subCost[nodeIdx] = b.c_int * SurfaceArea( n.aabbMin, n.aabbMax ) * (float)n.triCount;
			st.refCount[nodeIdx] = n.triCount;
			if (stackPtr == 0) break;
			nodeIdx = stack[--stackPtr];
			continue;
		}
		st.order[st.orderCount++] = nodeIdx;
		BVH_FATAL_ERROR_IF( stackPtr == STACK_SIZE, "BVH_Verbose::OptimizeEx, tree too deep." );
		stack[stackPtr++] = n.right, nodeIdx = n.left;
	}
	for (uint32_t i = st.orderCount; i-- > 0; )
	{
		const uint32_t n = st.order[i];
		const Node& node = b.bvhNode[n];
		st.subCost[n] = b.c_trav * SurfaceArea( node.aabbMin, node.aabbMax ) + st.epoProxy * OptExOverlap( node.left, node.right )
			+ st.subCost[node.left] + st.subCost[node.right];
		st.refCount[n] = st.refCount[node.left] + st.refCount[node.right];
	}
}

// OptExSweepSplit: exact SAH sweep over all three axes for a set of child nodes, partitioned
// in place. 'jitter' scales each axis' score by a hashed factor: the candidate set stays the
// same and the decision stays cost-driven, so repeated attempts sample a plateau rather than
// drift off it. jitter 0 is the plain greedy choice.
uint32_t Optimizer::OptExSweepSplit( uint32_t* leaf, const uint32_t count,
	const float jitter, uint32_t& seed, State& st )
{
	Node* nodes = b.bvhNode;
	float bestScore = BVH_FAR;
	uint32_t bestAxis = 3;
	uint32_t bestK = count / 2;
	for (uint32_t axis = 0; axis < 3; axis++)
	{
		std::sort( leaf, leaf + count, [nodes, axis]( const uint32_t p, const uint32_t q )
		{
			const float ca = nodes[p].aabbMin[axis] + nodes[p].aabbMax[axis];
			const float cb = nodes[q].aabbMin[axis] + nodes[q].aabbMax[axis];
			return ca != cb ? (ca < cb) : (p < q); // stable without needing stable_sort
		} );
		// suffix areas and reference counts
		bvhvec3 bmin( BVH_FAR ), bmax( -BVH_FAR );
		uint32_t n = 0;
		for (uint32_t i = count; i-- > 1; )
		{
			bmin = tinybvh::tinybvh_min( bmin, nodes[leaf[i]].aabbMin ), bmax = tinybvh::tinybvh_max( bmax, nodes[leaf[i]].aabbMax );
			n += st.refCount[leaf[i]];
			st.workF[i] = SurfaceArea( bmin, bmax ), st.workI[i] = n;
		}
		bmin = bvhvec3( BVH_FAR ), bmax = bvhvec3( -BVH_FAR ), n = 0;
		float axisBest = BVH_FAR;
		uint32_t axisK = 1;
		for (uint32_t i = 0; i < count - 1; i++)
		{
			bmin = tinybvh::tinybvh_min( bmin, nodes[leaf[i]].aabbMin ), bmax = tinybvh::tinybvh_max( bmax, nodes[leaf[i]].aabbMax );
			n += st.refCount[leaf[i]];
			const float score = SurfaceArea( bmin, bmax ) * (float)n + st.workF[i + 1] * (float)st.workI[i + 1];
			if (score < axisBest) axisBest = score, axisK = i + 1;
		}
		if (jitter > 0) axisBest *= float( 1 ) + jitter * tinybvh::tinybvh_rndfloat( seed );
		if (axisBest < bestScore) bestScore = axisBest, bestAxis = axis, bestK = axisK;
	}
	if (bestAxis == 3) return count / 2; // all candidates coincide; fall back to a median split
	if (bestAxis != 2) std::sort( leaf, leaf + count, [nodes, bestAxis]( const uint32_t p, const uint32_t q )
	{
		const float ca = nodes[p].aabbMin[bestAxis] + nodes[p].aabbMax[bestAxis];
		const float cb = nodes[q].aabbMin[bestAxis] + nodes[q].aabbMax[bestAxis];
		return ca != cb ? (ca < cb) : (p < q);
	} );
	return bestK;
}

// OptExBuildTopology: build a subtree over 'count' existing nodes, consuming interior node
// slots from 'slots'. Returns the subtree root and accumulates the interior cost it creates;
// the leaves' own cost is invariant, so that accumulator is all a comparison needs.
uint32_t Optimizer::OptExBuildTopology( uint32_t* leaf, const uint32_t count,
	uint32_t* slots, uint32_t& slotPtr, const uint32_t parent, const float jitter, uint32_t& seed,
	float& interiorCost, State& st )
{
	if (count == 1) { b.bvhNode[leaf[0]].parent = parent; return leaf[0]; }
	const uint32_t self = slots[slotPtr++];
	uint32_t k = count == 2 ? 1 : OptExSweepSplit( leaf, count, jitter, seed, st );
	if (k == 0 || k >= count) k = count / 2;
	const uint32_t l = OptExBuildTopology( leaf, k, slots, slotPtr, self, jitter, seed, interiorCost, st );
	const uint32_t r = OptExBuildTopology( leaf + k, count - k, slots, slotPtr, self, jitter, seed, interiorCost, st );
	Node& n = b.bvhNode[self];
	n.left = l, n.right = r, n.triCount = 0, n.firstTri = 0, n.parent = parent;
	n.aabbMin = tinybvh::tinybvh_min( b.bvhNode[l].aabbMin, b.bvhNode[r].aabbMin );
	n.aabbMax = tinybvh::tinybvh_max( b.bvhNode[l].aabbMax, b.bvhNode[r].aabbMax );
	st.refCount[self] = st.refCount[l] + st.refCount[r];
	interiorCost += b.c_trav * SurfaceArea( n.aabbMin, n.aabbMax ) + st.epoProxy * OptExOverlap( l, r );
	return self;
}

// OptExExplode: one reference per leaf. Multi-reference leaves become small subtrees over the
// recovered reference bounds; single-reference leaves just take their reference's (tighter) box.
void Optimizer::OptExExplode( const Ref* refBox, State& st )
{
	const uint32_t origNodes = b.usedNodes;
	for (uint32_t n = 0; n < origNodes; n++)
	{
		Node& node = b.bvhNode[n];
		if (!node.isLeaf() || node.triCount == 0) continue;
		const uint32_t first = node.firstTri, m = node.triCount;
		if (m == 1) { node.aabbMin = refBox[first].bmin, node.aabbMax = refBox[first].bmax; continue; }
		// turn the m references into m single-reference leaves, then a subtree over them.
		uint32_t* leaf = st.work;
		for (uint32_t i = 0; i < m; i++)
		{
			const uint32_t child = i == 0 ? n : b.usedNodes++; // slot 'n' is reused as a leaf below
			leaf[i] = child;
		}
		// the root slot must stay 'n', so allocate the interior slots first and shuffle.
		uint32_t* pool = (uint32_t*)tinybvh::malloc64( m * sizeof( uint32_t ) );
		pool[0] = n;
		for (uint32_t i = 1; i < m - 1; i++) pool[i] = b.usedNodes++;
		leaf[0] = b.usedNodes++; // the first reference needs a leaf slot of its own after all
		for (uint32_t i = 0; i < m; i++)
		{
			Node& lf = b.bvhNode[leaf[i]];
			lf.aabbMin = refBox[first + i].bmin, lf.aabbMax = refBox[first + i].bmax;
			lf.firstTri = first + i, lf.triCount = 1, lf.left = lf.right = 0;
			st.refCount[leaf[i]] = 1;
		}
		uint32_t slotPtr = 0;
		uint32_t seed = st.seed + (uint32_t)n * 2654435761u;
		float dummy = 0;
		OptExBuildTopology( leaf, m, pool, slotPtr, node.parent, 0, seed, dummy, st );
		tinybvh::free64( pool );
	}
	b.Refit( 0, true );
}

// OptExTreelet: form a treelet of up to maxL leaves below 'nodeIdx' by repeatedly opening up
// the treelet leaf with the largest surface area, then solve its optimal binary topology by
// exact DP over all 2^maxL subsets. The current topology is one of the candidates, so the
// result is never worse. Reuses the treelet's own interior slots: no allocation, no holes.
bool Optimizer::OptExTreelet( const uint32_t nodeIdx, const uint32_t maxL, const bool epoCheck, State& st )
{
	Node& root = b.bvhNode[nodeIdx];
	if (root.isLeaf()) return false;
	uint32_t leaf[MAX_TREELET], interior[MAX_TREELET];
	uint32_t L = 0, I = 0;
	interior[I++] = nodeIdx, leaf[L++] = root.left, leaf[L++] = root.right;
	while (L < maxL)
	{
		uint32_t pick = (uint32_t)-1;
		float bestArea = 0;
		for (uint32_t i = 0; i < L; i++) if (!b.bvhNode[leaf[i]].isLeaf())
		{
			const float a = SurfaceArea( b.bvhNode[leaf[i]].aabbMin, b.bvhNode[leaf[i]].aabbMax );
			if (pick == (uint32_t)-1 || a > bestArea) bestArea = a, pick = i;
		}
		if (pick == (uint32_t)-1) break; // the treelet is all tree leaves
		const uint32_t e = leaf[pick];
		interior[I++] = e, leaf[pick] = b.bvhNode[e].left, leaf[L++] = b.bvhNode[e].right;
	}
	if (L < 3) return false;
	const uint32_t full = (1u << L) - 1;
	float* cost = st.tCost;
	uint32_t* part = st.tPart;
	bvhvec3* tMin = st.tMin, * tMax = st.tMax;
	// Near the root a single node's box covers much of the scene, and the SAH-optimal topology
	// can give away more EPO than it wins. For those nodes the DP is re-run under a few sibling
	// overlap weights - useless as a global objective, but a cheap generator of topologies that
	// differ in the way EPO cares about - and the alternatives are measured exactly. Only the
	// treelet's own interior nodes change, so each measurement is 2(L-1) clipping traversals and
	// nothing outside the treelet needs visiting.
	static const float epoWeight[4] = { float( 0 ), float( 0.25 ), float( 0.75 ), float( 2 ) };
	const uint32_t candidates = epoCheck ? tinybvh::tinybvh_min( st.epoCandidates, 4u ) : 1u;
	Node bckInterior[MAX_TREELET], bestInterior[MAX_TREELET];
	uint32_t bckLeafParent[MAX_TREELET], bestLeafParent[MAX_TREELET];
	float bestEPO = 0;
	bool haveBest = false;
	if (epoCheck)
	{
		bestEPO = OptExLocalEPO( interior, I, st.vertIdx, st.primArea, st.rootArea );
		for (uint32_t i = 0; i < I; i++) bckInterior[i] = b.bvhNode[interior[i]];
		for (uint32_t i = 0; i < L; i++) bckLeafParent[i] = b.bvhNode[leaf[i]].parent;
	}
	for (uint32_t cand = 0; cand < candidates; cand++)
	{
		const float w = epoCheck ? epoWeight[cand] : st.epoProxy;
		for (uint32_t S = 1; S <= full; S++)
		{
			if (tinybvh::tinybvh_popc( S ) == 1)
			{
				const uint32_t n = leaf[tinybvh::tinybvh_ctz( S )];
				tMin[S] = b.bvhNode[n].aabbMin, tMax[S] = b.bvhNode[n].aabbMax, cost[S] = st.subCost[n];
				continue;
			}
			const uint32_t low = S & (uint32_t)(-(int32_t)S), rest = S ^ low;
			tMin[S] = tinybvh::tinybvh_min( tMin[low], tMin[rest] ), tMax[S] = tinybvh::tinybvh_max( tMax[low], tMax[rest] );
			float best = BVH_FAR;
			uint32_t bestP = 0;
			for (uint32_t P = (S - 1) & S; P; P = (P - 1) & S)
			{
				if (P < (S ^ P)) continue; // each unordered partition once
				float c = cost[P] + cost[S ^ P];
				if (w > 0)
				{
					// the two halves become siblings, so their box overlap is charged here.
					const bvhvec3 omin = tinybvh::tinybvh_max( tMin[P], tMin[S ^ P] ), omax = tinybvh::tinybvh_min( tMax[P], tMax[S ^ P] );
					const bvhvec3 e = omax - omin;
					if (e.x >= 0 && e.y >= 0 && e.z >= 0) c += w * SurfaceArea( omin, omax );
				}
				if (c < best) best = c, bestP = P;
			}
			cost[S] = best + b.c_trav * SurfaceArea( tMin[S], tMax[S] ), part[S] = bestP;
		}
		// without the exact test the DP must beat the current topology on its own terms.
		if (!epoCheck && !(cost[full] < st.subCost[nodeIdx])) return false;
		// write the winning topology back, root first so it lands in 'nodeIdx'.
		const uint32_t rootParent = b.bvhNode[nodeIdx].parent;
		uint32_t slotPtr = 0, depth = 0;
		struct Item { uint32_t set; uint32_t slot, parent; } todo[2 * MAX_TREELET];
		todo[depth].set = full, todo[depth].slot = interior[slotPtr++], todo[depth].parent = rootParent, depth++;
		while (depth)
		{
			const Item it = todo[--depth];
			const uint32_t P = part[it.set], Q = it.set ^ P;
			uint32_t child[2];
			const uint32_t sub[2] = { P, Q };
			for (uint32_t k = 0; k < 2; k++)
			{
				if (tinybvh::tinybvh_popc( sub[k] ) == 1) { child[k] = leaf[tinybvh::tinybvh_ctz( sub[k] )], b.bvhNode[child[k]].parent = it.slot; }
				else
				{
					child[k] = interior[slotPtr++];
					todo[depth].set = sub[k], todo[depth].slot = child[k], todo[depth].parent = it.slot, depth++;
				}
			}
			Node& n = b.bvhNode[it.slot];
			n.left = child[0], n.right = child[1], n.triCount = 0, n.firstTri = 0, n.parent = it.parent;
			n.aabbMin = tMin[it.set], n.aabbMax = tMax[it.set];
			st.subCost[it.slot] = cost[it.set], st.refCount[it.slot] = 0;
		}
		if (!epoCheck) break; // the SAH winner is the only candidate; keep it
		const float local = OptExLocalEPO( interior, I, st.vertIdx, st.primArea, st.rootArea );
		if (local < bestEPO)
		{
			bestEPO = local, haveBest = true;
			for (uint32_t i = 0; i < I; i++) bestInterior[i] = b.bvhNode[interior[i]];
			for (uint32_t i = 0; i < L; i++) bestLeafParent[i] = b.bvhNode[leaf[i]].parent;
		}
	}
	if (epoCheck)
	{
		// restore the winner, or the original topology when no candidate beat it.
		const Node* src = haveBest ? bestInterior : bckInterior;
		const uint32_t* srcParent = haveBest ? bestLeafParent : bckLeafParent;
		for (uint32_t i = 0; i < I; i++) b.bvhNode[interior[i]] = src[i];
		for (uint32_t i = 0; i < L; i++) b.bvhNode[leaf[i]].parent = srcParent[i];
		// the DP's own cost numbers carry the overlap weight of whichever candidate produced
		// them, so recompute this treelet's costs under the optimizer's normal convention.
		for (uint32_t i = I; i-- > 0; )
		{
			const Node& n = b.bvhNode[interior[i]];
			st.subCost[interior[i]] = b.c_trav * SurfaceArea( n.aabbMin, n.aabbMax ) + st.epoProxy * OptExOverlap( n.left, n.right )
				+ st.subCost[n.left] + st.subCost[n.right];
			st.refCount[interior[i]] = st.refCount[n.left] + st.refCount[n.right];
		}
		return haveBest;
	}
	// refCount of the rewritten interior nodes, bottom-up over the treelet only.
	for (uint32_t i = I; i-- > 0; )
	{
		const Node& n = b.bvhNode[interior[i]];
		st.refCount[interior[i]] = st.refCount[n.left] + st.refCount[n.right];
	}
	return true;
}

void Optimizer::OptExTreeletSweep( const Settings& s, State& st )
{
	OptExComputeCosts( st );
	const uint32_t tiny = (uint32_t)tinybvh::tinybvh_clamp( (int32_t)s.treeletSize, 3, (int32_t)MAX_TREELET );
	const uint32_t large = (uint32_t)tinybvh::tinybvh_clamp( (int32_t)s.treeletSizeTop, (int32_t)tiny, (int32_t)MAX_TREELET );
	const uint32_t topRefs = tinybvh::tinybvh_max( (uint32_t)1, (uint32_t)(st.refCount[0] / 64) );
	for (uint32_t i = st.orderCount; i-- > 0; )
	{
		const uint32_t n = st.order[i];
		if (b.bvhNode[n].isLeaf()) continue; // a sweep may have turned it into one
		OptExTreelet( n, st.refCount[n] >= topRefs ? large : tiny,
			st.epoTopRefs && st.refCount[n] >= (uint32_t)st.epoTopRefs, st );
	}
	b.Refit( 0, true );
}

// OptExRebuildSubtrees: rebuild the subtrees that carry the most cost from scratch. The root
// box of a subtree is the union of its references and cannot change, so a rebuild is a purely
// local move: nothing above it is touched, and the comparison is exact.
void Optimizer::OptExRebuildSubtrees( const Settings& s, State& st )
{
	OptExComputeCosts( st );
	if (st.orderCount == 0) return;
	// rank interior nodes of a workable size by the cost their subtree carries.
	struct Cand { float cost; uint32_t node; };
	Cand* cand = (Cand*)tinybvh::malloc64( st.orderCount * sizeof( Cand ) );
	uint32_t candCount = 0;
	for (uint32_t i = 0; i < st.orderCount; i++)
	{
		const uint32_t n = st.order[i];
		if (st.refCount[n] < (uint32_t)s.rebuildMin || st.refCount[n] > (uint32_t)s.rebuildMax) continue;
		cand[candCount].cost = st.subCost[n], cand[candCount++].node = n;
	}
	std::sort( cand, cand + candCount, []( const Cand& a, const Cand& b ) { return a.cost > b.cost; } );
	uint8_t* taken = (uint8_t*)tinybvh::malloc64( b.usedNodes );
	memset( taken, 0, b.usedNodes );
	Node* backup = (Node*)tinybvh::malloc64( (size_t)s.rebuildMax * sizeof( Node ) );
	uint32_t* pool = (uint32_t*)tinybvh::malloc64( (size_t)s.rebuildMax * sizeof( uint32_t ) );
	uint32_t* leaf = (uint32_t*)tinybvh::malloc64( (size_t)s.rebuildMax * sizeof( uint32_t ) );
	uint32_t* leaf0 = (uint32_t*)tinybvh::malloc64( (size_t)s.rebuildMax * sizeof( uint32_t ) );
	uint32_t stack[STACK_SIZE];
	uint32_t done = 0;
	for (uint32_t c = 0; c < candCount && done < s.rebuildSubtrees; c++)
	{
		const uint32_t R = cand[c].node;
		if (taken[R]) continue;
		// collect the subtree: leaves become the rebuild input, interior nodes the slot pool.
		uint32_t leafCount = 0, poolCount = 0, stackPtr = 0, nodeIdx = R;
		bool overflow = false;
		while (1)
		{
			const Node& n = b.bvhNode[nodeIdx];
			taken[nodeIdx] = 1;
			if (n.isLeaf()) leaf[leafCount++] = nodeIdx;
			else
			{
				pool[poolCount++] = nodeIdx;
				if (stackPtr == STACK_SIZE) { overflow = true; break; }
				stack[stackPtr++] = n.right, nodeIdx = n.left;
				continue;
			}
			if (stackPtr == 0) break;
			nodeIdx = stack[--stackPtr];
		}
		if (overflow || leafCount < 4 || poolCount != leafCount - 1) continue;
		pool[0] = R; // the root slot must stay put; the order of the others is free
		for (uint32_t i = 1; i < poolCount; i++) if (pool[i] == R) { pool[i] = pool[0]; break; }
		memcpy( leaf0, leaf, leafCount * sizeof( uint32_t ) );
		for (uint32_t i = 0; i < poolCount; i++) backup[i] = b.bvhNode[pool[i]];
		const uint32_t rootParent = b.bvhNode[R].parent;
		float original = 0;
		for (uint32_t i = 0; i < poolCount; i++) original += b.c_trav * SurfaceArea( backup[i].aabbMin, backup[i].aabbMax );
		float best = original;
		uint32_t bestAttempt = (uint32_t)-1;
		for (uint32_t attempt = 0; attempt < tinybvh::tinybvh_max( 1u, s.rebuildAttempts ); attempt++)
		{
			uint32_t slotPtr = 0;
			float interior = 0;
			uint32_t seed = st.seed + (uint32_t)R * 2654435761u + attempt * 40503u + 1;
			memcpy( leaf, leaf0, leafCount * sizeof( uint32_t ) );
			OptExBuildTopology( leaf, leafCount, pool, slotPtr, rootParent,
				attempt == 0 ? float( 0 ) : float( s.rebuildJitter ), seed, interior, st );
			if (interior < best) best = interior, bestAttempt = attempt;
		}
		if (bestAttempt == (uint32_t)-1)
		{
			// nothing beat the original: put it back, including the leaves' parents.
			for (uint32_t i = 0; i < poolCount; i++) b.bvhNode[pool[i]] = backup[i];
			for (uint32_t i = 0; i < poolCount; i++)
				b.bvhNode[b.bvhNode[pool[i]].left].parent = pool[i], b.bvhNode[b.bvhNode[pool[i]].right].parent = pool[i];
		}
		else
		{
			uint32_t slotPtr = 0;
			float interior = 0;
			uint32_t seed = st.seed + (uint32_t)R * 2654435761u + bestAttempt * 40503u + 1;
			memcpy( leaf, leaf0, leafCount * sizeof( uint32_t ) );
			OptExBuildTopology( leaf, leafCount, pool, slotPtr, rootParent,
				bestAttempt == 0 ? float( 0 ) : float( s.rebuildJitter ), seed, interior, st );
			done++;
		}
	}
	tinybvh::free64( leaf0 ), tinybvh::free64( leaf ), tinybvh::free64( pool );
	tinybvh::free64( backup ), tinybvh::free64( taken ), tinybvh::free64( cand );
}

// OptExUnsplit: a primitive that a spatial split duplicated may no longer be worth duplicating
// once the topology around it has moved. Merging two of its references deletes one leaf and its
// parent, and grows the survivor to the union; both sides of that trade are evaluated exactly
// along the paths they touch. Shrinkage above the deleted node is ignored, which only ever
// makes the estimate pessimistic.
void Optimizer::OptExUnsplit( State& st )
{
	OptExComputeCosts( st );
	// group the single-reference leaves by primitive.
	struct RefEntry { uint32_t prim, node; };
	RefEntry* ref = (RefEntry*)tinybvh::malloc64( b.idxCount * sizeof( RefEntry ) );
	uint32_t refs = 0;
	for (uint32_t n = 0; n < b.usedNodes; n++)
	{
		const Node& node = b.bvhNode[n];
		if (!node.isLeaf() || node.triCount != 1) continue;
		ref[refs].prim = b.primIdx[node.firstTri], ref[refs++].node = n;
	}
	std::sort( ref, ref + refs, []( const RefEntry& a, const RefEntry& b )
	{ return a.prim != b.prim ? (a.prim < b.prim) : (a.node < b.node); } );
	for (uint32_t i = 0; i < refs; )
	{
		uint32_t j = i;
		while (j < refs && ref[j].prim == ref[i].prim) j++;
		for (uint32_t x = i; x < j; x++) for (uint32_t y = x + 1; y < j; y++)
		{
			const uint32_t A = ref[x].node, B = ref[y].node;
			if (b.bvhNode[A].triCount != 1 || b.bvhNode[B].triCount != 1) continue; // already merged away
			const uint32_t P = b.bvhNode[B].parent;
			if (P == (uint32_t)-1 || b.bvhNode[A].parent == (uint32_t)-1) continue;
			const bvhvec3 uMin = tinybvh::tinybvh_min( b.bvhNode[A].aabbMin, b.bvhNode[B].aabbMin );
			const bvhvec3 uMax = tinybvh::tinybvh_max( b.bvhNode[A].aabbMax, b.bvhNode[B].aabbMax );
			const float uArea = SurfaceArea( uMin, uMax );
			if (b.bvhNode[A].parent == P)
			{
				// siblings: the parent becomes the merged leaf. Its box does not change.
				const float gain = b.c_trav * SurfaceArea( b.bvhNode[P].aabbMin, b.bvhNode[P].aabbMax ) +
					b.c_int * SurfaceArea( b.bvhNode[A].aabbMin, b.bvhNode[A].aabbMax ) + b.c_int * SurfaceArea( b.bvhNode[B].aabbMin, b.bvhNode[B].aabbMax ) -
					b.c_int * SurfaceArea( b.bvhNode[P].aabbMin, b.bvhNode[P].aabbMax );
				if (!(gain > 0)) continue;
				b.bvhNode[P].triCount = 1, b.bvhNode[P].firstTri = b.bvhNode[A].firstTri;
				b.bvhNode[P].left = b.bvhNode[P].right = 0;
				b.bvhNode[A].triCount = b.bvhNode[B].triCount = 0, b.bvhNode[A].parent = b.bvhNode[B].parent = (uint32_t)-1;
				holes = true;
				continue;
			}
			// P must not sit above A: removing it would change A's own path.
			bool nested = false;
			for (uint32_t w = b.bvhNode[A].parent; w != (uint32_t)-1; w = b.bvhNode[w].parent) if (w == P) { nested = true; break; }
			if (nested) continue;
			// removing B drops its leaf and its parent; its sibling takes the parent's place.
			float delta = -(b.c_trav * SurfaceArea( b.bvhNode[P].aabbMin, b.bvhNode[P].aabbMax ) +
				b.c_int * SurfaceArea( b.bvhNode[B].aabbMin, b.bvhNode[B].aabbMax ));
			// growing A to the union inflates A and, possibly, its ancestors.
			delta += b.c_int * (uArea - SurfaceArea( b.bvhNode[A].aabbMin, b.bvhNode[A].aabbMax ));
			for (uint32_t w = b.bvhNode[A].parent; w != (uint32_t)-1; w = b.bvhNode[w].parent)
			{
				const bvhvec3 nMin = tinybvh::tinybvh_min( b.bvhNode[w].aabbMin, uMin ), nMax = tinybvh::tinybvh_max( b.bvhNode[w].aabbMax, uMax );
				const float before = SurfaceArea( b.bvhNode[w].aabbMin, b.bvhNode[w].aabbMax ), after = SurfaceArea( nMin, nMax );
				if (!(after > before)) break; // and no ancestor above it grows either
				delta += b.c_trav * (after - before);
			}
			if (!(delta < 0)) continue;
			// apply: grow A, then splice out B and P.
			b.bvhNode[A].aabbMin = uMin, b.bvhNode[A].aabbMax = uMax;
			OptExRefitUp( A );
			const uint32_t S = b.bvhNode[P].left == B ? b.bvhNode[P].right : b.bvhNode[P].left;
			const uint32_t keepParent = b.bvhNode[P].parent;
			b.bvhNode[P] = b.bvhNode[S];
			b.bvhNode[P].parent = keepParent;
			if (!b.bvhNode[P].isLeaf()) b.bvhNode[b.bvhNode[P].left].parent = b.bvhNode[b.bvhNode[P].right].parent = P;
			b.bvhNode[B].triCount = b.bvhNode[S].triCount = 0;
			b.bvhNode[B].parent = b.bvhNode[S].parent = (uint32_t)-1;
			OptExRefitUp( P );
			holes = true;
		}
		i = j;
	}
	tinybvh::free64( ref );
}

// OptExCompactRefs: rewrite b.primIdx in depth-first order so that every subtree owns a
// contiguous range, which is what makes the collapse below a constant-time operation per
// node. Drops the slots of references that unsplitting removed.
void Optimizer::OptExCompactRefs()
{
	if (b.bvhNode == 0 || b.idxCount == 0) return;
	uint32_t* tmp = (uint32_t*)tinybvh::malloc64( b.idxCount * sizeof( uint32_t ) );
	uint32_t stack[STACK_SIZE], stackPtr = 0, nodeIdx = 0, next = 0;
	while (1)
	{
		Node& node = b.bvhNode[nodeIdx];
		if (node.isLeaf())
		{
			const uint32_t first = next;
			for (uint32_t i = 0; i < node.triCount; i++) tmp[next++] = b.primIdx[node.firstTri + i];
			node.firstTri = first;
			if (stackPtr == 0) break;
			nodeIdx = stack[--stackPtr];
			continue;
		}
		BVH_FATAL_ERROR_IF( stackPtr == STACK_SIZE, "BVH_Verbose::OptimizeEx, tree too deep." );
		stack[stackPtr++] = node.right, nodeIdx = node.left;
	}
	memcpy( b.primIdx, tmp, next * sizeof( uint32_t ) );
	b.idxCount = next;
	tinybvh::free64( tmp );
}

// OptExCollapse: exact bottom-up leaf sizing for the final topology. Run once, at the end:
// a multi-reference leaf is an atom that every other stage would have to work around.
void Optimizer::OptExCollapse( const uint32_t maxLeafPrims, State& st )
{
	if (maxLeafPrims < 2) return;
	OptExComputeCosts( st );
	// st.subCost currently holds the cost of the subtree as built; recompute it as the best
	// achievable cost under collapsing, bottom-up, and remember the first reference of each node.
	uint32_t* firstRef = st.work;
	for (uint32_t i = st.orderCount; i-- > 0; )
	{
		const uint32_t n = st.order[i];
		const Node& node = b.bvhNode[n];
		const float area = SurfaceArea( node.aabbMin, node.aabbMax );
		const float split = b.c_trav * area + st.subCost[node.left] + st.subCost[node.right];
		const float asLeaf = st.refCount[n] <= (uint32_t)maxLeafPrims ? b.c_int * area * (float)st.refCount[n] : BVH_FAR;
		st.subCost[n] = tinybvh::tinybvh_min( split, asLeaf );
	}
	for (uint32_t i = st.orderCount; i-- > 0; ) // children first: a parent reads its left child's value
	{
		const uint32_t n = st.order[i];
		const Node& node = b.bvhNode[n];
		firstRef[n] = b.bvhNode[node.left].isLeaf() ? b.bvhNode[node.left].firstTri : firstRef[node.left];
	}
	// top-down: the first node whose leaf form wins becomes a leaf; its subtree is abandoned
	// and disappears in the b.Compact() that follows.
	uint32_t stack[STACK_SIZE], stackPtr = 0, nodeIdx = 0;
	while (1)
	{
		Node& node = b.bvhNode[nodeIdx];
		if (!node.isLeaf())
		{
			const float area = SurfaceArea( node.aabbMin, node.aabbMax );
			if (st.refCount[nodeIdx] <= (uint32_t)maxLeafPrims &&
				b.c_int * area * (float)st.refCount[nodeIdx] <= b.c_trav * area + st.subCost[node.left] + st.subCost[node.right])
			{
				node.firstTri = firstRef[nodeIdx], node.triCount = st.refCount[nodeIdx];
				node.left = node.right = 0, holes = true;
			}
			else
			{
				BVH_FATAL_ERROR_IF( stackPtr == STACK_SIZE, "BVH_Verbose::OptimizeEx, tree too deep." );
				stack[stackPtr++] = node.right, nodeIdx = node.left;
				continue;
			}
		}
		if (stackPtr == 0) break;
		nodeIdx = stack[--stackPtr];
	}
}


// OptExOverlap: surface area of the intersection of two boxes, zero if they are disjoint. This
// is the free stand-in for EPO used inside the optimizer: foreign geometry inside a node's box
// arrives overwhelmingly through its sibling, and the overlap of the two child boxes is the
// part of that which can be read off the boxes alone, at no geometric cost at all.
float Optimizer::OptExOverlap( const uint32_t na, const uint32_t nb ) const
{
	const bvhvec3 bmin = tinybvh::tinybvh_max( b.bvhNode[na].aabbMin, b.bvhNode[nb].aabbMin );
	const bvhvec3 bmax = tinybvh::tinybvh_min( b.bvhNode[na].aabbMax, b.bvhNode[nb].aabbMax );
	const bvhvec3 e = bmax - bmin;
	if (e.x < 0 || e.y < 0 || e.z < 0) return 0;
	return SurfaceArea( bmin, bmax );
}

// OptExEPOArea: area of the primitives *not* below 'subtreeRoot' that falls inside its box.
// Mirrors BVH::EPOArea for the verbose layout.
float Optimizer::OptExEPOArea( const uint32_t subtreeRoot, const uint32_t nodeIdx,
	const uint32_t* vertIdx ) const
{
	if (nodeIdx == subtreeRoot) return 0; // everything below the subtree is its own, not foreign
	const Node& n = b.bvhNode[nodeIdx];
	const Node& subtree = b.bvhNode[subtreeRoot];
	float area = 0;
	if (n.isLeaf())
	{
		const bvhvec3 bmin = subtree.aabbMin, bmax = subtree.aabbMax;
		for (uint32_t i = 0; i < n.triCount; i++)
		{
			const uint32_t vidx = b.primIdx[n.firstTri + i] * 3;
			tinybvh::bvhvec4 v0, v1, v2;
			if (vertIdx) v0 = b.verts[vertIdx[vidx]], v1 = b.verts[vertIdx[vidx + 1]], v2 = b.verts[vertIdx[vidx + 2]];
			else v0 = b.verts[vidx], v1 = b.verts[vidx + 1], v2 = b.verts[vidx + 2];
			if (tinybvh::impl::tinybvh_tri_inside_box( v0, v1, v2, bmin, bmax ))
				area += float( 0.5 ) * tinybvh::tinybvh_length( tinybvh::tinybvh_cross( bvhvec3( v1 - v0 ), bvhvec3( v2 - v0 ) ) );
			else area += tinybvh::impl::tinybvh_clipped_tri_area<float>( bvhvec3( v0 ), bvhvec3( v1 ), bvhvec3( v2 ), bmin, bmax );
		}
		return area;
	}
	if (tinybvh::impl::tinybvh_aabbs_overlap( b.bvhNode[n.left], subtree )) area += OptExEPOArea( subtreeRoot, n.left, vertIdx );
	if (tinybvh::impl::tinybvh_aabbs_overlap( b.bvhNode[n.right], subtree )) area += OptExEPOArea( subtreeRoot, n.right, vertIdx );
	return area;
}

// OptExPrimAreaSum: total primitive area, the normalizer of the EPO term. Computed over the
// distinct primitives rather than over b.primIdx slots, so that it stays constant while the
// optimizer reorders and duplicates references.
float Optimizer::OptExPrimAreaSum( const uint32_t* vertIdx ) const
{
	double sum = 0;
	for (uint32_t i = 0; i < b.triCount; i++)
	{
		const uint32_t vidx = i * 3;
		tinybvh::bvhvec4 v0, v1, v2;
		if (vertIdx) v0 = b.verts[vertIdx[vidx]], v1 = b.verts[vertIdx[vidx + 1]], v2 = b.verts[vertIdx[vidx + 2]];
		else v0 = b.verts[vidx], v1 = b.verts[vidx + 1], v2 = b.verts[vidx + 2];
		sum += (double)(float( 0.5 ) * tinybvh::tinybvh_length( tinybvh::tinybvh_cross( bvhvec3( v1 - v0 ), bvhvec3( v2 - v0 ) ) ));
	}
	return (float)sum;
}

// OptExCombined: the full EPO-weighted quality metric, same definition as BVH::EPOCost. One
// call walks every node and clips geometry against each of their boxes: on Sponza that is
// three orders of magnitude slower than SAHCost, which is why the optimizer only ever calls
// this at round granularity.
float Optimizer::OptExCombined( const uint32_t* vertIdx, const float primArea, float& sahOut ) const
{
	double epoSum = 0, sahSum = 0;
	uint32_t stack[STACK_SIZE], stackPtr = 0, nodeIdx = 0;
	while (1)
	{
		const Node& n = b.bvhNode[nodeIdx];
		const double w = n.isLeaf() ? ((double)b.c_int * (double)n.triCount) : (double)b.c_trav;
		sahSum += w * (double)SurfaceArea( n.aabbMin, n.aabbMax );
		epoSum += w * (double)OptExEPOArea( nodeIdx, 0, vertIdx );
		if (!n.isLeaf())
		{
			if (stackPtr == STACK_SIZE) break;
			stack[stackPtr++] = n.right, nodeIdx = n.left;
			continue;
		}
		if (stackPtr == 0) break;
		nodeIdx = stack[--stackPtr];
	}
	const float rootArea = SurfaceArea( b.bvhNode[0].aabbMin, b.bvhNode[0].aabbMax );
	sahOut = rootArea > 0 ? (float)(sahSum / (double)rootArea) : float( 0 );
	if (!(primArea > 0)) return sahOut;
	return (float( 1 ) - float( wEPO )) * sahOut + float( wEPO ) * (float)(epoSum / (double)primArea);
}

// OptExLocalEPO: the combined cost of one set of interior nodes, in the units of the global
// metric. A treelet rewrite leaves the boxes and the primitive sets of every node outside the
// treelet untouched - its root box is the union of its leaves either way - so the change in
// the global metric is exactly the change over these nodes, and nothing else needs visiting.
float Optimizer::OptExLocalEPO( const uint32_t* nodes, const uint32_t count,
	const uint32_t* vertIdx, const float primArea, const float rootArea ) const
{
	double sah = 0, epo = 0;
	for (uint32_t i = 0; i < count; i++)
	{
		const uint32_t n = nodes[i];
		sah += (double)b.c_trav * (double)SurfaceArea( b.bvhNode[n].aabbMin, b.bvhNode[n].aabbMax );
		epo += (double)b.c_trav * (double)OptExEPOArea( n, 0, vertIdx );
	}
	return (float)((1.0 - (double)wEPO) * sah / (double)rootArea + (double)wEPO * epo / (double)primArea);
}


// OptExExactBottom: replace every maximal subtree holding at most 'exactBottomSize' references
// with the provably optimal BVH over those references - topology and leaf formation decided
// jointly, which is what separates this from running the treelet pass and then collapsing.
// The DP is the standard subset recurrence: for every subset S of the m references, the best
// cost is the cheaper of making S a leaf and splitting it into two subsets, and the table is
// filled in ascending numeric order because every proper subset of S precedes it.
void Optimizer::OptExExactBottom( const Settings& s, State& st )
{
	const uint32_t maxN = (uint32_t)tinybvh::tinybvh_clamp( (int32_t)s.exactBottomSize, 2, (int32_t)MAX_EXACT );
	OptExCompactRefs(); // every subtree now owns a contiguous slice of b.primIdx
	OptExComputeCosts( st );
	const uint32_t tableSize = 1u << maxN;
	float* cost = (float*)tinybvh::malloc64( tableSize * sizeof( float ) );
	uint32_t* part = (uint32_t*)tinybvh::malloc64( tableSize * sizeof( uint32_t ) );
	bvhvec3* tMin = (bvhvec3*)tinybvh::malloc64( tableSize * sizeof( bvhvec3 ) );
	bvhvec3* tMax = (bvhvec3*)tinybvh::malloc64( tableSize * sizeof( bvhvec3 ) );
	uint32_t leaf0[MAX_EXACT], pool[MAX_EXACT], prims[MAX_EXACT], tmpPrim[MAX_EXACT];
	uint32_t stack[STACK_SIZE], stackPtr = 0, nodeIdx = 0;
	while (1)
	{
		const Node& node = b.bvhNode[nodeIdx];
		bool descend = !node.isLeaf();
		if (descend && st.refCount[nodeIdx] <= (uint32_t)maxN)
		{
			// a maximal subtree of workable size: collect its leaves and interior slots.
			const uint32_t R = nodeIdx;
			uint32_t m = 0, slots = 0, s2 = 0, sub[STACK_SIZE], cur = R;
			bool usable = true;
			while (1)
			{
				const Node& n = b.bvhNode[cur];
				if (n.isLeaf())
				{
					if (n.triCount != 1 || m == maxN) { usable = false; break; }
					leaf0[m++] = cur;
				}
				else
				{
					pool[slots++] = cur;
					if (s2 == STACK_SIZE) { usable = false; break; }
					sub[s2++] = n.right, cur = n.left;
					continue;
				}
				if (s2 == 0) break;
				cur = sub[--s2];
			}
			if (usable && m >= 3 && slots == m - 1)
			{
				const uint32_t firstRef = b.bvhNode[leaf0[0]].firstTri;
				float current = 0;
				for (uint32_t i = 0; i < slots; i++) current += b.c_trav * SurfaceArea( b.bvhNode[pool[i]].aabbMin, b.bvhNode[pool[i]].aabbMax );
				for (uint32_t i = 0; i < m; i++)
				{
					current += b.c_int * SurfaceArea( b.bvhNode[leaf0[i]].aabbMin, b.bvhNode[leaf0[i]].aabbMax );
					prims[i] = b.primIdx[b.bvhNode[leaf0[i]].firstTri];
				}
				const uint32_t full = (1u << m) - 1;
				for (uint32_t S = 1; S <= full; S++)
				{
					const uint32_t bits = tinybvh::tinybvh_popc( S );
					if (bits == 1)
					{
						const uint32_t n = leaf0[tinybvh::tinybvh_ctz( S )];
						tMin[S] = b.bvhNode[n].aabbMin, tMax[S] = b.bvhNode[n].aabbMax;
						cost[S] = b.c_int * SurfaceArea( tMin[S], tMax[S] ), part[S] = 0; // a single reference is a leaf
						continue;
					}
					const uint32_t low = S & (uint32_t)(-(int32_t)S), rest = S ^ low;
					tMin[S] = tinybvh::tinybvh_min( tMin[low], tMin[rest] ), tMax[S] = tinybvh::tinybvh_max( tMax[low], tMax[rest] );
					const float area = SurfaceArea( tMin[S], tMax[S] );
					float best = BVH_FAR;
					uint32_t bestP = 0;
					for (uint32_t P = (S - 1) & S; P; P = (P - 1) & S)
					{
						if (P < (S ^ P)) continue; // each unordered partition once
						const float c = cost[P] + cost[S ^ P];
						if (c < best) best = c, bestP = P;
					}
					const float asLeaf = bits <= s.maxLeafPrims ? b.c_int * area * (float)bits : BVH_FAR;
					const float asSplit = b.c_trav * area + best;
					if (asLeaf <= asSplit) cost[S] = asLeaf, part[S] = 0; else cost[S] = asSplit, part[S] = bestP;
				}
				if (cost[full] < current)
				{
					// write the optimal tree back. Leaves get contiguous slices of the same
					// b.primIdx range, in whatever order the walk emits them; the compaction that
					// follows puts the range back in depth-first order.
					uint32_t slotPtr = 0, leafPtr = 0, refPtr = 0, depth = 0;
					struct Item { uint32_t set; uint32_t slot, parent; } todo[2 * MAX_EXACT];
					auto emitLeaf = [&]( const uint32_t S, const uint32_t parent ) -> uint32_t
					{
						const uint32_t n = leaf0[leafPtr++];
						b.bvhNode[n].aabbMin = tMin[S], b.bvhNode[n].aabbMax = tMax[S];
						b.bvhNode[n].firstTri = firstRef + refPtr, b.bvhNode[n].triCount = tinybvh::tinybvh_popc( S );
						b.bvhNode[n].left = b.bvhNode[n].right = 0, b.bvhNode[n].parent = parent;
						for (uint32_t b = 0; b < m; b++) if (S & (1u << b)) tmpPrim[refPtr++] = prims[b];
						return n;
					};
					if (part[full] == 0)
					{
						// the whole subtree collapses into a single leaf; R becomes that leaf.
						b.bvhNode[R].aabbMin = tMin[full], b.bvhNode[R].aabbMax = tMax[full];
						b.bvhNode[R].firstTri = firstRef, b.bvhNode[R].triCount = (uint32_t)m;
						b.bvhNode[R].left = b.bvhNode[R].right = 0;
						for (uint32_t b = 0; b < m; b++) tmpPrim[refPtr++] = prims[b];
						for (uint32_t i = 0; i < m; i++) b.bvhNode[leaf0[i]].triCount = 0, b.bvhNode[leaf0[i]].parent = (uint32_t)-1;
						for (uint32_t i = 1; i < slots; i++) b.bvhNode[pool[i]].triCount = 0, b.bvhNode[pool[i]].parent = (uint32_t)-1;
					}
					else
					{
						todo[depth].set = full, todo[depth].slot = pool[slotPtr++], todo[depth].parent = b.bvhNode[R].parent, depth++;
						while (depth)
						{
							const Item it = todo[--depth];
							const uint32_t P = part[it.set], Q = it.set ^ P;
							const uint32_t half[2] = { P, Q };
							uint32_t child[2];
							for (uint32_t k = 0; k < 2; k++)
							{
								if (part[half[k]] == 0) child[k] = emitLeaf( half[k], it.slot );
								else
								{
									child[k] = pool[slotPtr++];
									todo[depth].set = half[k], todo[depth].slot = child[k], todo[depth].parent = it.slot, depth++;
								}
							}
							Node& n = b.bvhNode[it.slot];
							n.left = child[0], n.right = child[1], n.triCount = 0, n.firstTri = 0, n.parent = it.parent;
							n.aabbMin = tMin[it.set], n.aabbMax = tMax[it.set];
						}
						// slots and leaves the optimal tree did not need become unreachable.
						for (uint32_t i = slotPtr; i < slots; i++) b.bvhNode[pool[i]].triCount = 0, b.bvhNode[pool[i]].parent = (uint32_t)-1;
						for (uint32_t i = leafPtr; i < m; i++) b.bvhNode[leaf0[i]].triCount = 0, b.bvhNode[leaf0[i]].parent = (uint32_t)-1;
					}
					if (slotPtr < slots || leafPtr < m || part[full] == 0) holes = true;
					memcpy( b.primIdx + firstRef, tmpPrim, m * sizeof( uint32_t ) );
				}
			}
			descend = false; // handled, optimally or not; never descend into it again
		}
		if (descend)
		{
			if (stackPtr == STACK_SIZE) break;
			stack[stackPtr++] = b.bvhNode[nodeIdx].right, nodeIdx = b.bvhNode[nodeIdx].left;
			continue;
		}
		if (stackPtr == 0) break;
		nodeIdx = stack[--stackPtr];
	}
	tinybvh::free64( tMax ), tinybvh::free64( tMin ), tinybvh::free64( part ), tinybvh::free64( cost );
}

void Optimizer::Run( const Ref* refBox, const uint32_t* vertIdx )
{
	BVH_FATAL_ERROR_IF( b.bvhNode == 0, "Optimizer::Run, no nodes." );
	
	if (b.usedNodes < 6 || b.idxCount == 0 || b.bvhNode[0].isLeaf()) return;
	// the exploded tree needs 2F-1 nodes; the pool was sized for the input tree.
	const uint32_t maxRefsForSnapshot = b.idxCount;
	const uint32_t spaceNeeded = 2 * b.idxCount + 8;
	if (b.allocatedNodes < spaceNeeded)
	{
		Node* grown = (Node*)tinybvh::malloc64( spaceNeeded * sizeof( Node ) );
		memset( grown, 0, spaceNeeded * sizeof( Node ) );
		memcpy( grown, b.bvhNode, b.usedNodes * sizeof( Node ) );
		tinybvh::free64( b.bvhNode ), b.bvhNode = grown, b.allocatedNodes = spaceNeeded;
	}
	State st;
	st.subCost = (float*)tinybvh::malloc64( b.allocatedNodes * sizeof( float ) );
	st.refCount = (uint32_t*)tinybvh::malloc64( b.allocatedNodes * sizeof( uint32_t ) );
	st.order = (uint32_t*)tinybvh::malloc64( b.allocatedNodes * sizeof( uint32_t ) );
	st.work = (uint32_t*)tinybvh::malloc64( b.allocatedNodes * sizeof( uint32_t ) );
	st.workF = (float*)tinybvh::malloc64( (b.idxCount + 2) * sizeof( float ) );
	st.workI = (uint32_t*)tinybvh::malloc64( (b.idxCount + 2) * sizeof( uint32_t ) );
	st.tCost = (float*)tinybvh::malloc64( (1u << MAX_TREELET) * sizeof( float ) );
	st.tPart = (uint32_t*)tinybvh::malloc64( (1u << MAX_TREELET) * sizeof( uint32_t ) );
	st.tMin = (bvhvec3*)tinybvh::malloc64( (1u << MAX_TREELET) * sizeof( bvhvec3 ) );
	st.tMax = (bvhvec3*)tinybvh::malloc64( (1u << MAX_TREELET) * sizeof( bvhvec3 ) );
	st.orderCount = 0, st.seed = s.seed ? s.seed : 1u;
	st.vertIdx = vertIdx, st.epoProxy = float( s.epoProxy );
	epoVertIdx = vertIdx;
	st.epoCandidates = tinybvh::tinybvh_max( 1u, s.epoCandidates );
	const bool epoExact = s.epoGate || s.epoTreeletTop > 0;
	st.primArea = epoExact ? OptExPrimAreaSum( vertIdx ) : float( 1 );
	epoPrimArea = st.primArea;
	st.rootArea = SurfaceArea( b.bvhNode[0].aabbMin, b.bvhNode[0].aabbMax );
	st.epoTopRefs = 0;
	Node* snapNode = 0;
	uint32_t* snapPrim = 0;
	if (s.epoGate)
	{
		snapNode = (Node*)tinybvh::malloc64( b.allocatedNodes * sizeof( Node ) );
		snapPrim = (uint32_t*)tinybvh::malloc64( maxRefsForSnapshot * sizeof( uint32_t ) );
	}
	// stage 0: one reference per leaf. This raises SAHCost; stage 5 takes it back.
	OptExComputeCosts( st );
	if (s.explodeLeaves) OptExExplode( refBox, st );
	OptExComputeCosts( st );
	float cost = b.SAHCost(), sahPart = 0;
	if (s.epoGate) cost = OptExCombined( vertIdx, st.primArea, sahPart );
	if (s.verbose)
	{
		if (s.explodeLeaves) printf( "  per-round SAH is measured on the exploded tree, one reference per leaf;\n"
			"  the final collapse takes that back. EPO is only shown when -epogate asks for it.\n" );
		printf( "  %-5s %10s %10s %7s  %s\n", "round", "SAH", "EPO", "sec", "stage" ), fflush( stdout );
	}
	for (uint32_t round = 0; round < s.rounds; round++)
	{
		const float before = cost;
		const Clock::time_point roundStart = Clock::now();
		if (holes) { b.Compact(), holes = false; }
		uint32_t snapNodes = 0, snapRefs = 0;
		if (s.epoGate)
		{
			snapNodes = b.usedNodes, snapRefs = b.idxCount;
			memcpy( snapNode, b.bvhNode, b.usedNodes * sizeof( Node ) );
			memcpy( snapPrim, b.primIdx, b.idxCount * sizeof( uint32_t ) );
		}
		st.epoTopRefs = s.epoTreeletTop ? tinybvh::tinybvh_max( (uint32_t)1, (uint32_t)(st.refCount[0] / s.epoTreeletTop) ) : (uint32_t)0;
		if (s.reinsertIters) b.Optimize( s.reinsertIters, true, false ), Progress( round, roundStart, "reinsertion" ); // stage 1
		for (uint32_t k = 0; k < s.treeletSweeps; k++) OptExTreeletSweep( s, st ), Progress( round, roundStart, "treelets" ); // stage 2
		if (s.rebuildSubtrees) OptExRebuildSubtrees( s, st ), Progress( round, roundStart, "subtree rebuild" ); // stage 3
		if (s.unsplitRefs) { OptExUnsplit( st ); OptExCompactRefs(); Progress( round, roundStart, "unsplitting" ); } // stage 4
		cost = s.epoGate ? OptExCombined( vertIdx, st.primArea, sahPart ) : b.SAHCost();
		if (s.epoGate && !(cost < before))
		{
			// the round did not pay for itself under the EPO-weighted metric: undo it whole.
			memcpy( b.bvhNode, snapNode, snapNodes * sizeof( Node ) );
			memcpy( b.primIdx, snapPrim, snapRefs * sizeof( uint32_t ) );
			b.usedNodes = snapNodes, b.idxCount = snapRefs, holes = false, cost = before;
			if (s.verbose) printf( "  round %u undone: no gain under the EPO-weighted metric\n", round ), fflush( stdout );
			break;
		}
		if (!(before - cost > s.epsilon * before)) break;
	}
	if (holes) { b.Compact(), holes = false; }
	OptExCompactRefs();
	if (s.exactBottomSize > 1)
	{
		const Clock::time_point t0 = Clock::now();
		OptExExactBottom( s, st ); // stage 5a: optimal bottoms, topology and leaves decided together
		if (holes) { b.Compact(), holes = false; }
		OptExCompactRefs();
		Progress( (uint32_t)-1, t0, "exact bottoms" );
	}
	const Clock::time_point tc = Clock::now();
	OptExCollapse( s.maxLeafPrims, st ); // stage 5b: exact collapse for everything above them
	OptExCompactRefs();
	Progress( (uint32_t)-1, tc, "leaf collapse" );
	if (holes) { b.Compact(), holes = false; }
	if (snapPrim) tinybvh::free64( snapPrim );
	if (snapNode) tinybvh::free64( snapNode );
	tinybvh::free64( st.tMax ), tinybvh::free64( st.tMin ), tinybvh::free64( st.tPart ), tinybvh::free64( st.tCost );
	tinybvh::free64( st.workI ), tinybvh::free64( st.workF ), tinybvh::free64( st.work );
	tinybvh::free64( st.order ), tinybvh::free64( st.refCount ), tinybvh::free64( st.subCost );
}
void Optimizer::Progress( const uint32_t round, const Clock::time_point& t0, const char* stage )
{
	if (!s.verbose) return;
	const float sah = b.SAHCost();
	if (round == (uint32_t)-1) printf( "  %-5s %10.4f %10s %7.1f  %s\n", "final", sah, "-", MilliSec( t0, Clock::now() ) / 1000.0, stage );
	else if (s.epoGate)
	{
		float sahPart = 0;
		lastEPO = OptExCombined( epoVertIdx, epoPrimArea, sahPart );
		printf( "  %-5u %10.4f %10.4f %7.1f  %s\n", round, sah, lastEPO, MilliSec( t0, Clock::now() ) / 1000.0, stage );
	}
	else printf( "  %-5u %10.4f %10s %7.1f  %s\n", round, sah, "-", MilliSec( t0, Clock::now() ) / 1000.0, stage );
	fflush( stdout );
}

// ----------------------------------------------------------------------------------------
// reference recovery
//
// A finished spatial-split build has remapped primIdx to primitive indices and dropped its
// fragment array, so the per-reference bounds every stage needs have to be reconstructed. The
// bound used here is the primitive's own box intersected with the box of the leaf that holds
// it, which provably contains the builder's fragment: the fragment's geometry lies inside the
// leaf box by construction and inside the primitive by definition.
//
// Clipping the triangle polygon against the leaf's six planes gives a tighter bound - worth
// about 0.14 of SAH and 0.9 of EPO on Sponza - but doing it as six successive half-space clips
// is not numerically safe: with the split positions sitting exactly on fragment extremes, an
// interpolated vertex can land inside where it belongs outside, and the box then fails to
// contain its own geometry. That showed up as 125 rays in 187k returning a different hit.
// A proper simultaneous polygon clip would recover the tightness; this is not that, and a
// bound that is slightly loose costs quality, while one that is slightly tight costs hits.
// ----------------------------------------------------------------------------------------

static Optimizer::Ref RecoverRef( const tinybvh::bvhvec4slice& verts, const uint32_t* vertIdx, const uint32_t prim,
	const bvhvec3& bmin, const bvhvec3& bmax )
{
	bvhvec3 v0, v1, v2;
	const uint32_t vidx = prim * 3;
	if (!vertIdx) v0 = verts[vidx], v1 = verts[vidx + 1], v2 = verts[vidx + 2];
	else v0 = verts[vertIdx[vidx]], v1 = verts[vertIdx[vidx + 1]], v2 = verts[vertIdx[vidx + 2]];
	Optimizer::Ref r;
	r.primIdx = prim, r.clipped = 1;
	r.bmin = tinybvh::tinybvh_min( v0, tinybvh::tinybvh_min( v1, v2 ) );
	r.bmax = tinybvh::tinybvh_max( v0, tinybvh::tinybvh_max( v1, v2 ) );
	r.bmin = tinybvh::tinybvh_max( r.bmin, bmin ), r.bmax = tinybvh::tinybvh_min( r.bmax, bmax );
	for (uint32_t k = 0; k < 3; k++) if (!(r.bmax[k] >= r.bmin[k])) { r.bmin = bmin, r.bmax = bmax; break; }
	return r;
}

// ----------------------------------------------------------------------------------------
// driver
// ----------------------------------------------------------------------------------------

static std::vector<bvhvec4> LoadMesh( const char* path, uint32_t& triCount )
{
	FILE* f = fopen( path, "rb" );
	if (!f) { printf( "error: cannot open mesh '%s'\n", path ); exit( 1 ); }
	if (fread( &triCount, 1, 4, f ) != 4) { printf( "error: '%s' is not a mesh file\n", path ); exit( 1 ); }
	std::vector<bvhvec4> v( (size_t)triCount * 3 );
	if (fread( v.data(), 16, v.size(), f ) != v.size()) { printf( "error: '%s' is truncated\n", path ); exit( 1 ); }
	fclose( f );
	return v;
}

static std::string ReplaceExt( const std::string& path, const char* ext )
{
	const size_t dot = path.find_last_of( '.' ), slash = path.find_last_of( "/\\" );
	if (dot == std::string::npos || (slash != std::string::npos && dot < slash)) return path + ext;
	return path.substr( 0, dot ) + ext;
}

static std::string OutputName( const std::string& path )
{
	const size_t dot = path.find_last_of( '.' ), slash = path.find_last_of( "/\\" );
	if (dot == std::string::npos || (slash != std::string::npos && dot < slash)) return path + "_opt";
	return path.substr( 0, dot ) + "_opt" + path.substr( dot );
}

static void Usage()
{
	printf(
		"usage: bvhopt <file.bvh> [mesh.bin] [options]\n"
		"\n"
		"  A BVH saved by a current TinyBVH carries its own geometry and needs nothing else.\n"
		"  For an older file, pass the mesh it was built from, or leave it to default to\n"
		"  <file>.bin. The geometry is needed either way: EPO is defined over the triangles.\n"
		"\n"
		"  -rounds N          outer rounds (default 16; stops early on convergence)\n"
		"  -reinsert N        reinsertion iterations per round, 0 disables (default 100)\n"
		"  -treelet N[,M]     treelet size, and size near the root (default 7,10; max 12)\n"
		"  -sweeps N          treelet sweeps per round, 0 disables (default 2)\n"
		"  -rebuild N         worst subtrees rebuilt per round, 0 disables (default 0)\n"
		"  -nounsplit         do not merge spatial split references\n"
		"  -noexplode         keep the input's leaves instead of one reference per leaf\n"
		"  -exact N           exact optimal subtrees up to N references, 0 disables (default 10, max 14)\n"
		"  -leaf N            final leaf budget: 1 for a pure BVH2, 4 for BVH4_CPU (default 4)\n"
		"  -epotop N          exact EPO test for nodes holding >= refs/N references, 0 disables (default 1024)\n"
		"  -epocand N         topologies offered to that test, 1..4 (default 4)\n"
		"  -epogate           evaluate EPO every round and undo a round that did not pay\n"
		"  -seed N            randomization seed\n"
		"  -quiet             no per-round progress\n" );
}

int main( int argc, char** argv )
{
	if (argc < 2) { Usage(); return 1; }
	std::string bvhPath = argv[1], meshPath;
	Optimizer::Settings set;
	for (int i = 2; i < argc; i++)
	{
		const std::string a = argv[i];
		auto num = [&]( uint32_t def ) { return i + 1 < argc ? (uint32_t)atoi( argv[++i] ) : def; };
		if (a == "-rounds") set.rounds = num( 16 );
		else if (a == "-reinsert") set.reinsertIters = num( 100 );
		else if (a == "-sweeps") set.treeletSweeps = num( 2 );
		else if (a == "-rebuild") set.rebuildSubtrees = num( 0 );
		else if (a == "-exact") set.exactBottomSize = num( 10 );
		else if (a == "-leaf") set.maxLeafPrims = num( 4 );
		else if (a == "-epotop") set.epoTreeletTop = num( 1024 );
		else if (a == "-epocand") set.epoCandidates = num( 4 );
		else if (a == "-seed") set.seed = num( 0x12345678u );
		else if (a == "-epogate") set.epoGate = true;
		else if (a == "-nounsplit") set.unsplitRefs = false;
		else if (a == "-noexplode") set.explodeLeaves = false;
		else if (a == "-quiet") set.verbose = false;
		else if (a == "-treelet")
		{
			if (i + 1 < argc)
			{
				const std::string v = argv[++i];
				const size_t comma = v.find( ',' );
				set.treeletSize = (uint32_t)atoi( v.c_str() );
				set.treeletSizeTop = comma == std::string::npos ? set.treeletSize : (uint32_t)atoi( v.c_str() + comma + 1 );
			}
		}
		else if (a == "-h" || a == "--help") { Usage(); return 0; }
		else if (a[0] == '-') { printf( "error: unknown option '%s'\n", a.c_str() ); return 1; }
		else meshPath = a;
	}
	set.treeletSize = (uint32_t)tinybvh::tinybvh_clamp( (int32_t)set.treeletSize, 3, (int32_t)MAX_TREELET );
	set.treeletSizeTop = (uint32_t)tinybvh::tinybvh_clamp( (int32_t)set.treeletSizeTop, (int32_t)set.treeletSize, (int32_t)MAX_TREELET );
	if (set.exactBottomSize > MAX_EXACT) set.exactBottomSize = MAX_EXACT;

	tinybvh::BVH bvh;
	std::vector<bvhvec4> mesh;
	bool standalone = meshPath.empty() && bvh.Load( bvhPath.c_str() );
	if (!standalone)
	{
		// an older file, or one the caller wants to pair with a specific mesh.
		const std::string path = meshPath.empty() ? ReplaceExt( bvhPath, ".bin" ) : meshPath;
		uint32_t triCount = 0;
		mesh = LoadMesh( path.c_str(), triCount );
		if (!bvh.Load( bvhPath.c_str(), mesh.data(), triCount ))
		{
			printf( "error: '%s' is not a BVH for this mesh, or was written by another version.\n", bvhPath.c_str() );
			return 1;
		}
		meshPath = path;
	}
	if (bvh.vertIdx) { printf( "error: indexed BVHs are not supported yet.\n" ); return 1; }

	if (standalone) printf( "input  : %s (geometry embedded)\n", bvhPath.c_str() );
	else printf( "input  : %s (+ %s)\n", bvhPath.c_str(), meshPath.c_str() );
	Clock::time_point t0 = Clock::now();
	const float sah0 = bvh.SAHCost();
	const double sahMs = MilliSec( t0, Clock::now() );
	t0 = Clock::now();
	const float epo0 = bvh.EPOCost();
	const double epoMs = MilliSec( t0, Clock::now() );
	printf( "         %u triangles, %u references, %u nodes\n", bvh.triCount, bvh.idxCount, bvh.usedNodes );
	printf( "         SAH %10.4f (%.1f ms)   EPO %10.4f (%.1f ms)\n", sah0, sahMs, epo0, epoMs );

	// recover one clipped box per reference, then hand the tree to the optimizer.
	std::vector<Optimizer::Ref> refBox( bvh.idxCount );
	for (uint32_t n = 0; n < bvh.usedNodes; n++)
	{
		const tinybvh::BVH::BVHNode& node = bvh.bvhNode[n];
		if (!node.isLeaf() || node.triCount == 0) continue;
		for (uint32_t i = 0; i < node.triCount; i++)
			refBox[node.leftFirst + i] = RecoverRef( bvh.verts, bvh.vertIdx,
				bvh.primIdx[node.leftFirst + i], node.aabbMin, node.aabbMax );
	}
	tinybvh::BVH_Verbose verbose;
	verbose.ConvertFrom( bvh );
	printf( "optimizing:\n" );
	t0 = Clock::now();
	Optimizer opt( verbose, set );
	opt.Run( refBox.data(), bvh.vertIdx );
	const double optMs = MilliSec( t0, Clock::now() );
	bvh.ConvertFrom( verbose );

	const float sah1 = bvh.SAHCost(), epo1 = bvh.EPOCost();
	printf( "result : %u references, %u nodes, %.1f s\n", bvh.idxCount, bvh.usedNodes, optMs / 1000.0 );
	printf( "         SAH %10.4f -> %10.4f (%+6.2f%%)\n", sah0, sah1, 100.0f * (sah1 - sah0) / sah0 );
	printf( "         EPO %10.4f -> %10.4f (%+6.2f%%)\n", epo0, epo1, 100.0f * (epo1 - epo0) / epo0 );
	const std::string outPath = OutputName( bvhPath );
	bvh.Save( outPath.c_str() );
	printf( "output : %s\n", outPath.c_str() );
	return 0;
}
