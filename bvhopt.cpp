// bvhopt - offline BVH quality optimizer for TinyBVH.
//
// Reads a BVH saved with BVH::Save, spends an unbounded amount of time lowering its SAH and
// EPO cost, and writes the result next to the input with "_opt" appended to the stem.
//
// Usage: bvhopt <file.bvh> [options]
//
// Build: g++ -O2 -mavx2 -mfma -std=c++17 -o bvhopt bvhopt.cpp
//
// Results:
// CRYTEKSPONZA   SAH: 61.6687 -> 56.7408 ( -7.99%), EPO: 20.9516 -> 19.2926 ( -7.92%)
// CONFERENCE     SAH: 36.3071 -> 30.9185 (-14.84%), EPO: 12.7778 -> 10.9949 (-13.95%)
// BISTRO         SAH: 64.6207 -> 58.1694 ( -9.98%), EPO: 32.8089 -> 27.2498 (-16.94%)

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

// DensityField: total primitive surface area per cell of a regular grid over the scene, summed
// into a 3D prefix table so the area inside any axis-aligned box is eight lookups.
//
// This is what makes EPO usable as a guide rather than only as a verdict. The foreign area a
// node sees is (all geometry inside its box) minus (its own geometry), and the second term is
// free: a node's references lie inside its own box by construction, so the geometry under a set
// of nodes is just the sum of their own areas. Only the first term needs the scene, and that is
// a function of the box alone - exactly what a density field answers.
struct DensityField
{
	uint32_t N = 0;
	bvhvec3 bmin, scale;
	float cellDiag = 0;		// world-space diagonal of one cell: below this the estimate is noise
	std::vector<double> sat; // (N+1)^3 prefix sums
	std::vector<double> cell;	// per-cell accumulator, released once the table is summed
	bvhvec3 cellSize;
	void BeginBuild( const bvhvec3& lo, const bvhvec3& hi, uint32_t res );
	void Splat( const bvhvec3& p, double amount );
	void EndBuild();
	void Build( const tinybvh::bvhvec4slice& verts, const uint32_t* vertIdx, uint32_t triCount,
		const bvhvec3& lo, const bvhvec3& hi, uint32_t res );
	double Sample( double x, double y, double z ) const;   // trilinear, in grid coordinates
	float AreaInside( const bvhvec3& lo, const bvhvec3& hi ) const;
};

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
		// The reinsertion stage is most of the run, and the serial version kept here converges
		// lower than the library's parallel one. For a mesh large enough that the wall clock is
		// the thing that hurts, 'fast' hands the stage back to the library.
		bool fast = false;				// use BVH_Verbose::Optimize, which runs on the pool.
		uint32_t fastMu = 8;			// its search density: higher searches fewer nodes per round.
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
		// EPO predicts traversal cost better than SAH but is three orders of magnitude more
		// expensive to evaluate, so it enters as an estimate read from a density field, with an
		// optional exact whole-tree gate once per round.
		float epoGuide = 0.5f;			// 1 = steer every decision by the EPO-weighted metric, using the
										// density field as the estimator; 0 = SAH only. Values in between
										// scale the EPO term, since the estimator is not the real thing.
		uint32_t epoGuideRes = 128;		// density field resolution per axis.
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
		float* workO;			// scratch: suffix own areas
		uint32_t* workI;		// scratch: suffix reference counts
		float* tCost;			// treelet DP: cost per subset
		uint32_t* tPart;		// treelet DP: winning partition per subset
		bvhvec3* tMin, * tMax;	// treelet DP: bounds per subset
		float* tOwn;			// treelet DP: own primitive area per subset
		uint32_t seed;
		const uint32_t* vertIdx;
		float primArea, rootArea;
		float* ownArea;			// primitive area below each node, which its own box fully contains
		float lambda;			// converts estimated foreign area into the units of surface area
	};
	void OptExRefitUp( uint32_t nodeIdx );
	void OptExComputeCosts( State& st );
	void RefreshOwnArea( State& st );
	struct SortItem { uint32_t idx; float cost; };
	struct RefitRecord { bvhvec3 bmin; uint32_t node; bvhvec3 bmax; uint32_t dummy; }; // 32 bytes
	double RefitUp( uint32_t nodeIdx, RefitRecord* journal, uint32_t& journalPtr, const uint32_t journalCap );
	float SAHCostUp( uint32_t nodeIdx ) const;
	uint32_t FindBestNewPosition( const uint32_t Lid, float& bestCost ) const;
	void OptimizeSerial( const uint32_t iterations, const bool extreme, bool stochastic );
	uint32_t OptExSweepSplit( uint32_t* leaf, const uint32_t count, const float jitter, uint32_t& seed, State& st );
	uint32_t OptExBuildTopology( uint32_t* leaf, const uint32_t count, uint32_t* slots, uint32_t& slotPtr,
		const uint32_t parent, const float jitter, uint32_t& seed, float& interiorCost, State& st );
	void OptExExplode( const Ref* refBox, State& st );
	bool OptExTreelet( const uint32_t nodeIdx, const uint32_t maxL, State& st );
	void OptExTreeletSweep( const Settings& s, State& st );
	void OptExRebuildSubtrees( const Settings& s, State& st );
	void OptExUnsplit( State& st );
	void OptExCompactRefs();
	void OptExCollapse( const uint32_t maxLeafPrims, State& st );
	void OptExExactBottom( const Settings& s, State& st );
	float OptExEPOArea( const uint32_t subtreeRoot, const uint32_t nodeIdx, const uint32_t* vertIdx ) const;
	float OptExPrimAreaSum( const uint32_t* vertIdx ) const;
	float OptExCombined( const uint32_t* vertIdx, const float primArea, float& sahOut ) const;

	void Progress( const uint32_t round, const Clock::time_point& t0, const char* stage );

	tinybvh::BVH_Verbose& b;
	Settings s;
	const uint32_t* epoVertIdx = 0;
	float epoPrimArea = 1;
	bool holes = false;		// nodes made unreachable by a stage; cleared by the next Compact()
	float lastEPO = 0;
	DensityField field;
	float LeafOwnArea( const uint32_t n, const uint32_t* vertIdx ) const;
	float EffArea( const bvhvec3& lo, const bvhvec3& hi, const float own, const State& st ) const
	{
		const float sa = SurfaceArea( lo, hi );
		if (st.lambda <= 0) return sa;
		// A box smaller than a cell gets its density by interpolation, and the error is then
		// comparable to the answer. Below that scale the estimate is dropped rather than trusted.
		const bvhvec3 e = hi - lo;
		if (e.x * e.x + e.y * e.y + e.z * e.z < field.cellDiag * field.cellDiag) return sa;
		const float foreign = field.AreaInside( lo, hi ) - own;
		return sa + st.lambda * tinybvh::tinybvh_max( 0.0f, foreign );
	}
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
// RefreshOwnArea: the primitive area a leaf's box actually contains. Only leaves need this from
// the geometry; interior nodes sum their children, which OptExComputeCosts does. Leaf boxes only
// move between rounds, so once per round is enough.
void Optimizer::RefreshOwnArea( State& st )
{
	if (st.lambda <= 0) return;
	uint32_t stack[STACK_SIZE], stackPtr = 0, nodeIdx = 0;
	while (1)
	{
		const Node& n = b.bvhNode[nodeIdx];
		if (n.isLeaf())
		{
			st.ownArea[nodeIdx] = LeafOwnArea( nodeIdx, st.vertIdx );
			if (stackPtr == 0) break;
			nodeIdx = stack[--stackPtr];
			continue;
		}
		BVH_FATAL_ERROR_IF( stackPtr == STACK_SIZE, "Optimizer::RefreshOwnArea, tree too deep." );
		stack[stackPtr++] = n.right, nodeIdx = n.left;
	}
}

void Optimizer::OptExComputeCosts( State& st )
{
	st.orderCount = 0;
	uint32_t stack[STACK_SIZE], stackPtr = 0, nodeIdx = 0;
	while (1)
	{
		const Node& n = b.bvhNode[nodeIdx];
		if (n.isLeaf())
		{
			st.subCost[nodeIdx] = b.c_int * EffArea( n.aabbMin, n.aabbMax, st.ownArea[nodeIdx], st ) * (float)n.triCount;
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
		st.ownArea[n] = st.ownArea[node.left] + st.ownArea[node.right];
		st.subCost[n] = b.c_trav * EffArea( node.aabbMin, node.aabbMax, st.ownArea[n], st )
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
		float own = 0;
		for (uint32_t i = count; i-- > 1; )
		{
			bmin = tinybvh::tinybvh_min( bmin, nodes[leaf[i]].aabbMin ), bmax = tinybvh::tinybvh_max( bmax, nodes[leaf[i]].aabbMax );
			n += st.refCount[leaf[i]], own += st.ownArea[leaf[i]];
			st.workF[i] = EffArea( bmin, bmax, own, st ), st.workI[i] = n, st.workO[i] = own;
		}
		bmin = bvhvec3( BVH_FAR ), bmax = bvhvec3( -BVH_FAR ), n = 0, own = 0;
		float axisBest = BVH_FAR;
		uint32_t axisK = 1;
		for (uint32_t i = 0; i < count - 1; i++)
		{
			bmin = tinybvh::tinybvh_min( bmin, nodes[leaf[i]].aabbMin ), bmax = tinybvh::tinybvh_max( bmax, nodes[leaf[i]].aabbMax );
			n += st.refCount[leaf[i]], own += st.ownArea[leaf[i]];
			const float score = EffArea( bmin, bmax, own, st ) * (float)n + st.workF[i + 1] * (float)st.workI[i + 1];
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
	st.ownArea[self] = st.ownArea[l] + st.ownArea[r];
	interiorCost += b.c_trav * EffArea( n.aabbMin, n.aabbMax, st.ownArea[self], st );
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
bool Optimizer::OptExTreelet( const uint32_t nodeIdx, const uint32_t maxL, State& st )
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
	for (uint32_t S = 1; S <= full; S++)
	{
		if (tinybvh::tinybvh_popc( S ) == 1)
		{
			const uint32_t n = leaf[tinybvh::tinybvh_ctz( S )];
			tMin[S] = b.bvhNode[n].aabbMin, tMax[S] = b.bvhNode[n].aabbMax;
			st.tOwn[S] = st.ownArea[n], cost[S] = st.subCost[n];
			continue;
		}
		const uint32_t low = S & (uint32_t)(-(int32_t)S), rest = S ^ low;
		tMin[S] = tinybvh::tinybvh_min( tMin[low], tMin[rest] ), tMax[S] = tinybvh::tinybvh_max( tMax[low], tMax[rest] );
		st.tOwn[S] = st.tOwn[low] + st.tOwn[rest]; // members lie inside their own boxes, so this is exact
		float best = BVH_FAR;
		uint32_t bestP = 0;
		for (uint32_t P = (S - 1) & S; P; P = (P - 1) & S)
		{
			if (P < (S ^ P)) continue; // each unordered partition once
			const float c = cost[P] + cost[S ^ P];
			if (c < best) best = c, bestP = P;
		}
		cost[S] = best + b.c_trav * EffArea( tMin[S], tMax[S], st.tOwn[S], st ), part[S] = bestP;
	}
	// without the exact test the DP must beat the current topology on its own terms.
	if (!(cost[full] < st.subCost[nodeIdx])) return false;
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
	// refCount of the rewritten interior nodes, bottom-up over the treelet only.
	for (uint32_t i = I; i-- > 0; )
	{
		const Node& n = b.bvhNode[interior[i]];
		st.refCount[interior[i]] = st.refCount[n.left] + st.refCount[n.right];
	}
	return true;
}


// ----------------------------------------------------------------------------------------
// reinsertion
//
// Bittner's insertion-based optimization, moved here verbatim from BVH_Verbose::Optimize. The
// library runs the parallel method of Meister & Bittner instead, which does less serial work
// but converges to a slightly higher SAH cost; this one stays for the offline case, where the
// quality matters and the wall clock does not.
// ----------------------------------------------------------------------------------------
void Optimizer::OptimizeSerial( const uint32_t iterations, const bool extreme, bool stochastic )
{
	BVH_FATAL_ERROR_IF( b.bvhNode == 0, "Optimizer::OptimizeSerial, no nodes." );
	BVH_FATAL_ERROR_IF( holes, "Optimizer::OptimizeSerial, tree has holes; compact it first." );
	if (iterations == 0 || b.usedNodes < 6 || b.bvhNode[0].isLeaf()) return;
	// working memory, allocated once.
	SortItem* sortList = (SortItem*)tinybvh::malloc64( b.usedNodes * sizeof( SortItem ) );
	const uint32_t journalCap = 4096; // >> 3x max tree depth
	RefitRecord* journal = (RefitRecord*)tinybvh::malloc64( journalCap * sizeof( RefitRecord ) );
	const double minGain = -1e-7 * (double)b.bvhNode[0].SA();
	uint32_t seed = 0x12345678u;
	// optimize by reinserting subtrees with a high cost - Section 3.4 of the paper.
	for (uint32_t i = 0; i < iterations; i++)
	{
		// calculate combined cost for all nodes
		uint32_t interiorNodes = 0;
		for (uint32_t j = 2; j < b.usedNodes; j++)
		{
			const Node& node = b.bvhNode[j];
			if (node.isLeaf()) continue;
			// skip the two children of the root: they have no grandparent to reattach to.
			if (node.parent == 0) continue;
			const float A = node.SA(), AL = b.bvhNode[node.left].SA(), AR = b.bvhNode[node.right].SA();
			const float Mmin = A / tinybvh::tinybvh_max( float( 1e-10 ), tinybvh::tinybvh_min( AL, AR ) );
			const float Msum = A / tinybvh::tinybvh_max( float( 1e-10 ), float( 0.5 ) * (AL + AR) );
			sortList[interiorNodes].idx = j, sortList[interiorNodes++].cost = A * Msum * Mmin;
		}
		if (interiorNodes == 0) break;
		// last couple of iterations we will process more nodes.
		const float portion = stochastic ? 0.5f : (extreme ? (0.01f + (0.6f * (float)i) / (float)iterations) : 0.01f);
		const uint32_t limit = tinybvh::tinybvh_max( (uint32_t)1, tinybvh::tinybvh_min( interiorNodes, (uint32_t)(portion * (float)interiorNodes) ) );
		const uint32_t step = tinybvh::tinybvh_max( (uint32_t)1, (uint32_t)(portion / 0.02f) );
		// partial descending sort: only the first 'limit' entries need to end up in order.
		struct Task { uint32_t first, last; } stack[STACK_SIZE];
		uint32_t first = 0, last = interiorNodes - 1;
		uint32_t stackPtr = 0;
		while (1)
		{
			if (first >= last)
			{
				if (stackPtr == 0) break; else first = stack[--stackPtr].first, last = stack[stackPtr].last;
				continue;
			}
			// median-of-three pivot
			SortItem t;
			const uint32_t mid = first + ((last - first) >> 1);
			if (sortList[mid].cost > sortList[first].cost) t = sortList[mid], sortList[mid] = sortList[first], sortList[first] = t;
			if (sortList[last].cost > sortList[first].cost) t = sortList[last], sortList[last] = sortList[first], sortList[first] = t;
			if (sortList[last].cost > sortList[mid].cost) t = sortList[last], sortList[last] = sortList[mid], sortList[mid] = t;
			t = sortList[mid], sortList[mid] = sortList[first], sortList[first] = t;
			uint32_t pivot = first;
			const SortItem e = sortList[first];
			for (uint32_t j = first + 1; j <= last; j++) if (sortList[j].cost > e.cost)
				t = sortList[j], sortList[j] = sortList[++pivot], sortList[pivot] = t;
			t = sortList[pivot], sortList[pivot] = sortList[first], sortList[first] = t;
			if (pivot + 1 < limit && stackPtr < STACK_SIZE) stack[stackPtr].first = pivot + 1, stack[stackPtr++].last = last;
			if (pivot == 0)
			{
				if (stackPtr == 0) break; else first = stack[--stackPtr].first, last = stack[stackPtr].last;
				continue;
			}
			last = pivot - 1;
		}
		// reinsert selected nodes
		Node bckp[5];
		uint32_t start = 0;
		if (stochastic)
		{
			float r = tinybvh::tinybvh_rndfloat( seed );
			r = tinybvh::tinybvh_max( 0.0f, (r * 1.2f) - 0.3f ); // 0 .. 0.9f
			start = (uint32_t)((float)limit * r);
		}
		uint32_t accepted = 0;
		for (uint32_t j = start; j < limit; j += stochastic ? (uint32_t)((tinybvh::tinybvh_rnduint( seed ) & 63) + 1) : step)
		{
			// prepare change
			const uint32_t Nid = sortList[j].idx;
			Node& N = b.bvhNode[Nid]; // N must have a grandparent to reattach its sibling to.
			if (N.isLeaf() || N.parent == 0 || N.parent == (uint32_t)-1) continue;
			const uint32_t Pid = N.parent;
			Node& P = b.bvhNode[Pid];
			const uint32_t X1 = P.parent, X2 = (P.left == Nid ? P.right : P.left);
			uint32_t jPtr = 0;
			// execute the change, accumulating the exact SAH delta as we go.
			bckp[0] = b.bvhNode[X1];
			if (b.bvhNode[X1].left == Pid) b.bvhNode[X1].left = X2;
			else /* b.bvhNode[X1].right == Pid */ b.bvhNode[X1].right = X2;
			const uint32_t p2 = b.bvhNode[X2].parent;
			b.bvhNode[X2].parent = X1;
			const uint32_t Lid = N.left, Rid = N.right;
			double deltaArea = RefitUp( X2, journal, jPtr, journalCap );
			float cost1, cost2;
			const uint32_t Xbest1 = FindBestNewPosition( Lid, cost1 );
			if (Xbest1 == 0)
			{
				while (jPtr) --jPtr, b.bvhNode[journal[jPtr].node].aabbMin = journal[jPtr].bmin,
					b.bvhNode[journal[jPtr].node].aabbMax = journal[jPtr].bmax;
				b.bvhNode[X2].parent = p2, b.bvhNode[X1] = bckp[0];
				continue;
			}
			const uint32_t XA = b.bvhNode[Xbest1].parent;
			bckp[1] = b.bvhNode[Nid];
			N.left = Xbest1, N.right = Lid, N.parent = XA;
			bckp[2] = b.bvhNode[XA];
			if (b.bvhNode[XA].left == Xbest1) b.bvhNode[XA].left = Nid; else b.bvhNode[XA].right = Nid;
			const uint32_t p3 = b.bvhNode[Xbest1].parent, p4 = b.bvhNode[Lid].parent;
			b.bvhNode[Xbest1].parent = Nid, b.bvhNode[Lid].parent = Nid;
			deltaArea += RefitUp( Nid, journal, jPtr, journalCap );
			const uint32_t Xbest2 = FindBestNewPosition( Rid, cost2 );
			if (Xbest2 == 0) // [10] defensive, as above.
			{
				b.bvhNode[Lid].parent = p4, b.bvhNode[Xbest1].parent = p3;
				b.bvhNode[XA] = bckp[2], b.bvhNode[Nid] = bckp[1];
				b.bvhNode[X2].parent = p2, b.bvhNode[X1] = bckp[0];
				while (jPtr) --jPtr, b.bvhNode[journal[jPtr].node].aabbMin = journal[jPtr].bmin,
					b.bvhNode[journal[jPtr].node].aabbMax = journal[jPtr].bmax;
				continue;
			}
			const uint32_t XB = b.bvhNode[Xbest2].parent;
			bckp[3] = b.bvhNode[Pid];
			P.left = Xbest2, P.right = Rid, P.parent = XB;
			bckp[4] = b.bvhNode[XB];
			if (b.bvhNode[XB].left == Xbest2) b.bvhNode[XB].left = Pid; else b.bvhNode[XB].right = Pid;
			const uint32_t p1 = b.bvhNode[Xbest2].parent, p0 = b.bvhNode[Rid].parent;
			b.bvhNode[Xbest2].parent = Pid, b.bvhNode[Rid].parent = Pid;
			deltaArea += RefitUp( Pid, journal, jPtr, journalCap );
			// keep the change if it meaningfully reduced the SAH cost of the tree.
			if (deltaArea < minGain) { accepted++; continue; }
			// undo change, mind the order.
			b.bvhNode[Rid].parent = p0, b.bvhNode[Xbest2].parent = p1, b.bvhNode[XB] = bckp[4];
			b.bvhNode[Pid] = bckp[3], b.bvhNode[Lid].parent = p4, b.bvhNode[Xbest1].parent = p3;
			b.bvhNode[XA] = bckp[2], b.bvhNode[Nid] = bckp[1], b.bvhNode[X2].parent = p2, b.bvhNode[X1] = bckp[0];
			// restore the boxes from the journal, newest first.
			while (jPtr) --jPtr, b.bvhNode[journal[jPtr].node].aabbMin = journal[jPtr].bmin,
				b.bvhNode[journal[jPtr].node].aabbMax = journal[jPtr].bmax;
		}
		if (accepted == 0 && !stochastic && !extreme) break;
	}
	b.Refit( 0, true );
	tinybvh::free64( journal );
	tinybvh::free64( sortList );
}

double Optimizer::RefitUp( uint32_t nodeIdx, RefitRecord* journal, uint32_t& journalPtr, const uint32_t journalCap )
{
	double deltaArea = 0;
	bool first = true;
	while (1)
	{
		Node& node = b.bvhNode[nodeIdx];
		if (!node.isLeaf())
		{
			const Node& left = b.bvhNode[node.left];
			const Node& right = b.bvhNode[node.right];
			const bvhvec3 bmin = tinybvh::tinybvh_min( left.aabbMin, right.aabbMin );
			const bvhvec3 bmax = tinybvh::tinybvh_max( left.aabbMax, right.aabbMax );
			// early out: if this node's box is unchanged, no ancestor changes either.
			if (!first &&
				bmin.x == node.aabbMin.x && bmin.y == node.aabbMin.y && bmin.z == node.aabbMin.z &&
				bmax.x == node.aabbMax.x && bmax.y == node.aabbMax.y && bmax.z == node.aabbMax.z) break;
			if (journalPtr == journalCap) return BVH_FAR; // overflow: force a reject.
			journal[journalPtr].node = nodeIdx;
			journal[journalPtr].bmin = node.aabbMin;
			journal[journalPtr++].bmax = node.aabbMax;
			const float areaBefore = node.SA();
			node.aabbMin = bmin, node.aabbMax = bmax;
			deltaArea += (double)node.SA() - (double)areaBefore;
		}
		first = false;
		if (nodeIdx == 0) break; else nodeIdx = node.parent;
	}
	return deltaArea;
}

// SAHCostUp: Calculate the SAH cost of a node and its ancestry
float Optimizer::SAHCostUp( uint32_t nodeIdx ) const
{
	float sum = 0;
	while (nodeIdx != (uint32_t)-1)
	{
		Node& node = b.bvhNode[nodeIdx];
		sum += SurfaceArea( node.aabbMin, node.aabbMax );
		nodeIdx = node.parent;
	}
	return sum;
}

// FindBestNewPosition
// Part of "Fast Insertion-Based Optimization of Bounding Volume Hierarchies"
uint32_t Optimizer::FindBestNewPosition( const uint32_t Lid, float& bestCost ) const
{
	struct Task { float ci; uint32_t node; };
	static const int maxTasks = STACK_SIZE;
	ALIGNED( 64 ) Task task[maxTasks];
	const Node& L = b.bvhNode[Lid];
	const bvhvec3 Lmin = L.aabbMin, Lmax = L.aabbMax;
	const float LSA = L.SA();
	float Cbest = BVH_FAR;
	uint32_t Xbest = 0;
	int tasks = 1;
	task[0].ci = 0, task[0].node = 0;
	while (tasks > 0)
	{
		// pop the task with the smallest induced cost
		const uint32_t Xid = task[0].node;
		const float CiLX = task[0].ci;
		if (--tasks > 0)
		{
			task[0] = task[tasks];
			for (int i = 0; ; )
			{
				const int l = 2 * i + 1, r = l + 1;
				int s = i;
				if (l < tasks && task[l].ci < task[s].ci) s = l;
				if (r < tasks && task[r].ci < task[s].ci) s = r;
				if (s == i) break;
				const Task t = task[i]; task[i] = task[s]; task[s] = t; i = s;
			}
		}
		// if the popped task has the lowest induced cost in the queue: done.
		if (CiLX + LSA >= Cbest) break;
		const Node& X = b.bvhNode[Xid];
		const float CdLX = SurfaceArea( tinybvh::tinybvh_min( Lmin, X.aabbMin ), tinybvh::tinybvh_max( Lmax, X.aabbMax ) );
		const float CLX = CiLX + CdLX;
		if (CLX < Cbest && Xid != 0) Cbest = CLX, Xbest = Xid;
		const float Ci = CLX - X.SA();
		if (Ci + LSA >= Cbest || X.isLeaf() || tasks + 2 > maxTasks) continue;
		for (int c = 0; c < 2; c++)
		{
			int i = tasks++;
			task[i].ci = Ci, task[i].node = c ? X.right : X.left;
			while (i > 0)
			{
				const int p = (i - 1) >> 1;
				if (task[p].ci <= task[i].ci) break;
				const Task t = task[p]; task[p] = task[i]; task[i] = t; i = p;
			}
		}
	}
	bestCost = Cbest;
	return Xbest;
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
		OptExTreelet( n, st.refCount[n] >= topRefs ? large : tiny, st );
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
	st.workO = (float*)tinybvh::malloc64( (b.idxCount + 2) * sizeof( float ) );
	st.tCost = (float*)tinybvh::malloc64( (1u << MAX_TREELET) * sizeof( float ) );
	st.tPart = (uint32_t*)tinybvh::malloc64( (1u << MAX_TREELET) * sizeof( uint32_t ) );
	st.tMin = (bvhvec3*)tinybvh::malloc64( (1u << MAX_TREELET) * sizeof( bvhvec3 ) );
	st.tMax = (bvhvec3*)tinybvh::malloc64( (1u << MAX_TREELET) * sizeof( bvhvec3 ) );
	st.orderCount = 0, st.seed = s.seed ? s.seed : 1u;
	// BVH::Save writes settings into the file and Load restores them, so a BVH saved with
	// threading disabled would run the library's parallel optimizer on the calling thread.
	if (s.fast) b.settings.enableThreading = true;
	st.vertIdx = vertIdx;
	st.ownArea = (float*)tinybvh::malloc64( b.allocatedNodes * sizeof( float ) );
	memset( st.ownArea, 0, b.allocatedNodes * sizeof( float ) );
	st.tOwn = (float*)tinybvh::malloc64( (1u << MAX_TREELET) * sizeof( float ) );
	st.lambda = 0;
	epoVertIdx = vertIdx;
	const bool epoExact = s.epoGate;
	st.primArea = epoExact ? OptExPrimAreaSum( vertIdx ) : float( 1 );
	epoPrimArea = st.primArea;
	st.rootArea = SurfaceArea( b.bvhNode[0].aabbMin, b.bvhNode[0].aabbMax );
	if (s.epoGuide > 0)
	{
		if (st.primArea <= 1) st.primArea = OptExPrimAreaSum( vertIdx );
		field.Build( b.verts, vertIdx, (uint32_t)b.triCount, b.bvhNode[0].aabbMin, b.bvhNode[0].aabbMax, s.epoGuideRes );
		// one scalar puts foreign area into surface-area units: the whole objective is then the
		// EPO-weighted metric up to a constant factor, and nothing else has to change.
		st.lambda = s.epoGuide * (wEPO * st.rootArea) / ((1.0f - wEPO) * st.primArea);
	}
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
	RefreshOwnArea( st );
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
		RefreshOwnArea( st );
		if (s.reinsertIters)
		{
			if (s.fast) b.Optimize( s.reinsertIters, s.fastMu ); else OptimizeSerial( s.reinsertIters, true, false );
			Progress( round, roundStart, "reinsertion" ); // stage 1
		}
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
	tinybvh::free64( st.tOwn ), tinybvh::free64( st.ownArea );
	tinybvh::free64( st.tMax ), tinybvh::free64( st.tMin ), tinybvh::free64( st.tPart ), tinybvh::free64( st.tCost );
	tinybvh::free64( st.workO ), tinybvh::free64( st.workI ), tinybvh::free64( st.workF ), tinybvh::free64( st.work );
	tinybvh::free64( st.order ), tinybvh::free64( st.refCount ), tinybvh::free64( st.subCost );
}
void DensityField::BeginBuild( const bvhvec3& lo, const bvhvec3& hi, uint32_t res )
{
	N = res;
	const bvhvec3 e = hi - lo;
	const bvhvec3 pad( e.x * 1e-4f + 1e-6f, e.y * 1e-4f + 1e-6f, e.z * 1e-4f + 1e-6f );
	bmin = lo - pad;
	const bvhvec3 span = (hi + pad) - bmin;
	scale = bvhvec3( (float)N / span.x, (float)N / span.y, (float)N / span.z );
	cellSize = bvhvec3( span.x / (float)N, span.y / (float)N, span.z / (float)N );
	cellDiag = tinybvh::tinybvh_length( span ) / (float)N;
	cell.assign( (size_t)N * N * N, 0.0 );
}

void DensityField::Splat( const bvhvec3& p, double amount )
{
	int32_t x = (int32_t)((p.x - bmin.x) * scale.x), y = (int32_t)((p.y - bmin.y) * scale.y), z = (int32_t)((p.z - bmin.z) * scale.z);
	x = tinybvh::tinybvh_clamp( x, 0, (int32_t)N - 1 );
	y = tinybvh::tinybvh_clamp( y, 0, (int32_t)N - 1 );
	z = tinybvh::tinybvh_clamp( z, 0, (int32_t)N - 1 );
	cell[((size_t)z * N + y) * N + x] += amount;
}

void DensityField::EndBuild()
{
	const size_t S = N + 1;
	double total = 0;
	sat.assign( S * S * S, 0.0 );
	for (uint32_t z = 0; z < N; z++) for (uint32_t y = 0; y < N; y++) for (uint32_t x = 0; x < N; x++)
	{
		const double v = cell[((size_t)z * N + y) * N + x];
		sat[((size_t)(z + 1) * S + (y + 1)) * S + (x + 1)] = v, total += v;
	}
	for (size_t z = 1; z < S; z++) for (size_t y = 1; y < S; y++) for (size_t x = 1; x < S; x++)
		sat[(z * S + y) * S + x] += sat[(z * S + y) * S + x - 1];
	for (size_t z = 1; z < S; z++) for (size_t x = 0; x < S; x++) for (size_t y = 1; y < S; y++)
		sat[(z * S + y) * S + x] += sat[(z * S + y - 1) * S + x];
	for (size_t y = 0; y < S; y++) for (size_t x = 0; x < S; x++) for (size_t z = 1; z < S; z++)
		sat[(z * S + y) * S + x] += sat[((z - 1) * S + y) * S + x];
	(void)total;
	cell.clear(), cell.shrink_to_fit();
}

void DensityField::Build( const tinybvh::bvhvec4slice& verts, const uint32_t* vertIdx, uint32_t triCount,
	const bvhvec3& lo, const bvhvec3& hi, uint32_t res )
{
	BeginBuild( lo, hi, res );
	const float cellMin = tinybvh::tinybvh_min( cellSize.x, tinybvh::tinybvh_min( cellSize.y, cellSize.z ) );
	for (uint32_t i = 0; i < triCount; i++)
	{
		const uint32_t vidx = i * 3;
		bvhvec3 v0, v1, v2;
		if (vertIdx) v0 = verts[vertIdx[vidx]], v1 = verts[vertIdx[vidx + 1]], v2 = verts[vertIdx[vidx + 2]];
		else v0 = verts[vidx], v1 = verts[vidx + 1], v2 = verts[vidx + 2];
		const float area = 0.5f * tinybvh::tinybvh_length( tinybvh::tinybvh_cross( v1 - v0, v2 - v0 ) );
		if (!(area > 0)) continue;
		// splat the area over a barycentric lattice fine enough to resolve the grid.
		const float extent = tinybvh::tinybvh_max( tinybvh::tinybvh_length( v1 - v0 ),
			tinybvh::tinybvh_max( tinybvh::tinybvh_length( v2 - v0 ), tinybvh::tinybvh_length( v2 - v1 ) ) );
		uint32_t k = (uint32_t)(extent / tinybvh::tinybvh_max( cellMin, 1e-9f )) + 1;
		k = tinybvh::tinybvh_min( k, 64u );
		const uint32_t samples = k * k;
		const double share = (double)area / (double)samples;
		for (uint32_t a = 0; a < k; a++) for (uint32_t c = 0; c < k; c++)
		{
			float u = ((float)a + 0.5f) / (float)k, v = ((float)c + 0.5f) / (float)k;
			if (u + v > 1.0f) u = 1.0f - u, v = 1.0f - v;
			Splat( v0 + u * (v1 - v0) + v * (v2 - v0), share );
		}
	}
	EndBuild();
}

double DensityField::Sample( double x, double y, double z ) const
{
	// trilinear read of the prefix table, so that boxes smaller than a cell still resolve.
	const size_t S = N + 1;
	x = tinybvh::tinybvh_clamp( x, 0.0, (double)N ), y = tinybvh::tinybvh_clamp( y, 0.0, (double)N );
	z = tinybvh::tinybvh_clamp( z, 0.0, (double)N );
	const size_t x0 = (size_t)x, y0 = (size_t)y, z0 = (size_t)z;
	const size_t x1 = tinybvh::tinybvh_min( x0 + 1, S - 1 ), y1 = tinybvh::tinybvh_min( y0 + 1, S - 1 ), z1 = tinybvh::tinybvh_min( z0 + 1, S - 1 );
	const double fx = x - (double)x0, fy = y - (double)y0, fz = z - (double)z0;
	auto at = [&]( size_t xi, size_t yi, size_t zi ) { return sat[(zi * S + yi) * S + xi]; };
	const double c00 = at( x0, y0, z0 ) * (1 - fx) + at( x1, y0, z0 ) * fx;
	const double c10 = at( x0, y1, z0 ) * (1 - fx) + at( x1, y1, z0 ) * fx;
	const double c01 = at( x0, y0, z1 ) * (1 - fx) + at( x1, y0, z1 ) * fx;
	const double c11 = at( x0, y1, z1 ) * (1 - fx) + at( x1, y1, z1 ) * fx;
	return (c00 * (1 - fy) + c10 * fy) * (1 - fz) + (c01 * (1 - fy) + c11 * fy) * fz;
}

float DensityField::AreaInside( const bvhvec3& lo, const bvhvec3& hi ) const
{
	if (N == 0) return 0;
	const double x0 = (lo.x - bmin.x) * scale.x, y0 = (lo.y - bmin.y) * scale.y, z0 = (lo.z - bmin.z) * scale.z;
	const double x1 = (hi.x - bmin.x) * scale.x, y1 = (hi.y - bmin.y) * scale.y, z1 = (hi.z - bmin.z) * scale.z;
	const double v = Sample( x1, y1, z1 ) - Sample( x0, y1, z1 ) - Sample( x1, y0, z1 ) - Sample( x1, y1, z0 )
		+ Sample( x0, y0, z1 ) + Sample( x0, y1, z0 ) + Sample( x1, y0, z0 ) - Sample( x0, y0, z0 );
	return (float)tinybvh::tinybvh_max( 0.0, v );
}

float Optimizer::LeafOwnArea( const uint32_t n, const uint32_t* vertIdx ) const
{
	const Node& node = b.bvhNode[n];
	float area = 0;
	for (uint32_t i = 0; i < node.triCount; i++)
	{
		const uint32_t vidx = b.primIdx[node.firstTri + i] * 3;
		tinybvh::bvhvec4 v0, v1, v2;
		if (vertIdx) v0 = b.verts[vertIdx[vidx]], v1 = b.verts[vertIdx[vidx + 1]], v2 = b.verts[vertIdx[vidx + 2]];
		else v0 = b.verts[vidx], v1 = b.verts[vidx + 1], v2 = b.verts[vidx + 2];
		if (tinybvh::impl::tinybvh_tri_inside_box( v0, v1, v2, node.aabbMin, node.aabbMax ))
			area += 0.5f * tinybvh::tinybvh_length( tinybvh::tinybvh_cross( bvhvec3( v1 - v0 ), bvhvec3( v2 - v0 ) ) );
		else area += tinybvh::impl::tinybvh_clipped_tri_area<float>( bvhvec3( v0 ), bvhvec3( v1 ), bvhvec3( v2 ), node.aabbMin, node.aabbMax );
	}
	return area;
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
// ray sampling
//
// A stand-in for a renderer's ray distribution: primaries aimed into the scene from outside,
// plus one scattered bounce from every hit. Used twice, with different seeds - once to build
// the ray density field the optimizer steers by, once to measure what the result actually
// costs to traverse. Sharing a sample between the two would only measure overfitting.
// ----------------------------------------------------------------------------------------

struct SampleRNG
{
	uint32_t s;
	SampleRNG( uint32_t x ) : s( x ? x : 1 ) {}
	uint32_t n() { s ^= s << 13, s ^= s >> 17, s ^= s << 5; return s; }
	float f() { return (float)(n() >> 8) * (1.0f / 16777216.0f); }
	float f( float a, float b ) { return a + (b - a) * f(); }
	bvhvec3 dir()
	{
		for (;;)
		{
			const bvhvec3 d( f( -1, 1 ), f( -1, 1 ), f( -1, 1 ) );
			const float l = tinybvh::tinybvh_length( d );
			if (l > 1e-3f && l <= 1.0f) return d * (1.0f / l);
		}
	}
};

// A fixed viewpoint makes the ray distribution strongly non-uniform, which is the only case in
// which weighting the cost model by it can say anything SAH does not already assume.
static bool g_cameraMode = false;

template <typename F> static void ForEachSampleRay( const tinybvh::BVH& bvh, uint32_t count, uint32_t seed, F&& fn )
{
	SampleRNG r( seed );
	const bvhvec3 lo = bvh.aabbMin, hi = bvh.aabbMax, mid = (lo + hi) * 0.5f;
	const float radius = 0.5f * tinybvh::tinybvh_length( hi - lo );
	// one fixed eye point for camera mode, derived from the scene box so it is reproducible.
	const bvhvec3 eye = mid + bvhvec3( 0.72f, 0.28f, 0.63f ) * (radius * 1.6f);
	for (uint32_t i = 0; i < count; i++)
	{
		const bvhvec3 o = g_cameraMode ? eye : (mid + r.dir() * (radius * 1.5f));
		const bvhvec3 target( r.f( lo.x, hi.x ), r.f( lo.y, hi.y ), r.f( lo.z, hi.z ) );
		bvhvec3 d = target - o;
		const float l = tinybvh::tinybvh_length( d );
		if (!(l > 0)) continue;
		d = d * (1.0f / l);
		tinybvh::Ray ray( o, d );
		const int32_t cost = bvh.Intersect( ray );
		fn( o, d, ray.hit.t, cost );
		if (ray.hit.t >= BVH_FAR) continue;
		const bvhvec3 p = o + d * ray.hit.t;
		const bvhvec3 d2 = r.dir();
		tinybvh::Ray bounce( p + d2 * (radius * 1e-4f), d2 );
		const int32_t cost2 = bvh.Intersect( bounce );
		fn( p, d2, bounce.hit.t, cost2 );
	}
}

static double MeasureTraversal( const tinybvh::BVH& bvh, uint32_t count, uint32_t seed )
{
	double total = 0;
	uint32_t n = 0;
	ForEachSampleRay( bvh, count, seed, [&]( const bvhvec3&, const bvhvec3&, float, int32_t cost )
	{
		total += (double)cost, n++;
	} );
	return n ? total / (double)n : 0.0;
}
// ----------------------------------------------------------------------------------------
// driver
// ----------------------------------------------------------------------------------------

static std::string OutputName( const std::string& path )
{
	const size_t dot = path.find_last_of( '.' ), slash = path.find_last_of( "/\\" );
	if (dot == std::string::npos || (slash != std::string::npos && dot < slash)) return path + "_opt";
	return path.substr( 0, dot ) + "_opt" + path.substr( dot );
}

static void Usage()
{
	printf(
		"usage: bvhopt <file.bvh> [options]\n"
		"\n"
		"  The BVH must have been saved with its geometry, which is what BVH::Save does by\n"
		"  default. EPO is defined over the triangles, so a file without them is of no use here.\n"
		"\n"
		"  -rounds N          outer rounds (default 16; stops early on convergence)\n"
		"  -reinsert N        reinsertion iterations per round, 0 disables (default 100)\n"
		"  -treelet N[,M]     treelet size, and size near the root (default 7,10; max 12)\n"
		"  -sweeps N          treelet sweeps per round, 0 disables (default 2)\n"
		"  -rebuild N         worst subtrees rebuilt per round, 0 disables (default 0)\n"
		"  -nounsplit         do not merge spatial split references\n"
		"  -noexplode         keep the input's leaves instead of one reference per leaf\n"
		"  -fast              reinsert with the library's parallel optimizer: quicker, converges higher\n"
		"  -fastmu N          its search density, higher is sparser and quicker (default 8)\n"
		"  -exact N           exact optimal subtrees up to N references, 0 disables (default 10, max 14)\n"
		"  -leaf N            final leaf budget: 1 for a pure BVH2, 4 for BVH4_CPU (default 4)\n"
		"  -epoguide W        steer decisions by estimated EPO, weight W (default 0.5, 0 = off)\n"
		"  -epores N          density field resolution per axis (default 128)\n"
		"  -epogate           evaluate EPO every round and undo a round that did not pay\n"
		"  -camera            measure traversal from one viewpoint instead of all around the scene\n"
		"  -seed N            randomization seed\n"
		"  -quiet             no per-round progress\n" );
}

int main( int argc, char** argv )
{
	if (argc < 2) { Usage(); return 1; }
	std::string bvhPath = argv[1];
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
		else if (a == "-seed") set.seed = num( 0x12345678u );
		else if (a == "-epogate") set.epoGate = true;
		else if (a == "-epoguide") set.epoGuide = i + 1 < argc ? (float)atof( argv[++i] ) : 1.0f;
		else if (a == "-epores") set.epoGuideRes = num( 128 );
		else if (a == "-camera") g_cameraMode = true;
		else if (a == "-nounsplit") set.unsplitRefs = false;
		else if (a == "-noexplode") set.explodeLeaves = false;
		else if (a == "-fast") set.fast = true;
		else if (a == "-fastmu") set.fastMu = num( 8 );
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
		else { printf( "error: unexpected argument '%s'\n", a.c_str() ); return 1; }
	}
	set.treeletSize = (uint32_t)tinybvh::tinybvh_clamp( (int32_t)set.treeletSize, 3, (int32_t)MAX_TREELET );
	set.treeletSizeTop = (uint32_t)tinybvh::tinybvh_clamp( (int32_t)set.treeletSizeTop, (int32_t)set.treeletSize, (int32_t)MAX_TREELET );
	if (set.exactBottomSize > MAX_EXACT) set.exactBottomSize = MAX_EXACT;

	tinybvh::BVH bvh = tinybvh::BVH::Load( bvhPath.c_str() );
	if (bvh.usedNodes == 0)
	{
		printf( "error: cannot read '%s'. It must be a BVH saved by this version of TinyBVH,\n"
			"       with its geometry included.\n", bvhPath.c_str() );
		return 1;
	}
	if (bvh.vertIdx) { printf( "error: indexed BVHs are not supported yet.\n" ); return 1; }

	printf( "input  : %s\n", bvhPath.c_str() );
	Clock::time_point t0 = Clock::now();
	const float sah0 = bvh.SAHCost();
	const double sahMs = MilliSec( t0, Clock::now() );
	t0 = Clock::now();
	const float epo0 = bvh.EPOCost();
	const double epoMs = MilliSec( t0, Clock::now() );
	printf( "         %u triangles, %u references, %u nodes\n", bvh.triCount, bvh.idxCount, bvh.usedNodes );
	const double trav0 = MeasureTraversal( bvh, 50000, 0xBEEF );
	printf( "         SAH %10.4f (%.1f ms)   EPO %10.4f (%.1f ms)   traversal %8.3f\n", sah0, sahMs, epo0, epoMs, trav0 );

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
	const double trav1 = MeasureTraversal( bvh, 50000, 0xBEEF );
	printf( "         traversal %8.3f -> %8.3f (%+6.2f%%)\n", trav0, trav1, 100.0 * (trav1 - trav0) / trav0 );
	const std::string outPath = OutputName( bvhPath );
	bvh.Save( outPath.c_str() );
	printf( "output : %s\n", outPath.c_str() );
	return 0;
}
