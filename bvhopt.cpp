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

static std::string StripExtension( const std::string& path )
{
	const size_t dot = path.find_last_of( '.' ), slash = path.find_last_of( "/\\" );
	if (dot == std::string::npos || (slash != std::string::npos && dot < slash)) return path;
	return path.substr( 0, dot );
}

static std::string OutputName( const std::string& path )
{
	const size_t dot = path.find_last_of( '.' ), slash = path.find_last_of( "/\\" );
	if (dot == std::string::npos || (slash != std::string::npos && dot < slash)) return path + "_opt";
	return path.substr( 0, dot ) + "_opt" + path.substr( dot );
}

// ----------------------------------------------------------------------------------------
// wide targets
//
// With --target, the input is a mesh rather than a BVH, and the output is a finished
// BVH4_CPU or BVH8_CPU. The binary tree is an intermediate here, so there is no reason to
// commit to one builder: both an SBVH and a Bonsai with spatial splits are built, optimized,
// collapsed and measured, and the cheapest one wins. The collapse itself is run twice per
// candidate as well - once weighted by surface area as usual, once by a visit probability
// measured with a sample ray set, after Gu et al. [PG 2015]. All four results are reported.
//
// 'cheapest' is the measured traversal work priced with the cost model the optimizer and the
// collapse already use: a node test at c_trav, a primitive test at c_int. Counting box tests
// alone - one per occupied child slot - prices a node by how full it is, which is backwards,
// since filling nodes is what the collapse is for. The weights only have to be right up to a
// constant, because candidates are ranked within one target, where the node width is fixed.
// Nothing here is selected by SAH on the tree, which ranks wide trees poorly.
// ----------------------------------------------------------------------------------------

// A ray set generated once and reused by every candidate, so that the comparison is between
// trees and nothing else. Two disjoint sets are drawn: one to fit the visit field, one to
// score the result. Fitting and scoring on the same rays would only measure overfitting.
struct RaySet
{
	std::vector<bvhvec3> O, D;
	size_t Count() const { return O.size(); }
};

static bool LoadMesh( const char* path, std::vector<bvhvec4>& tris )
{
	FILE* f = fopen( path, "rb" );
	if (!f) return false;
	uint32_t triCount = 0;
	if (fread( &triCount, sizeof( uint32_t ), 1, f ) != 1 || triCount == 0) { fclose( f ); return false; }
	tris.resize( (size_t)triCount * 3 );
	const size_t read = fread( tris.data(), sizeof( bvhvec4 ), tris.size(), f );
	fclose( f );
	return read == tris.size();
}

static void GenerateRays( const tinybvh::BVH& ref, const uint32_t count, const uint32_t seed, RaySet& set )
{
	set.O.clear(), set.D.clear();
	set.O.reserve( count * 2 ), set.D.reserve( count * 2 );
	ForEachSampleRay( ref, count, seed, [&]( const bvhvec3& o, const bvhvec3& d, float, int32_t )
	{
		set.O.push_back( o ), set.D.push_back( d );
	} );
}

static float BoxDistance( const bvhvec3& O, const bvhvec3& rD, const bvhvec3& bmin, const bvhvec3& bmax, const float tmax )
{
	const float tx1 = (bmin.x - O.x) * rD.x, tx2 = (bmax.x - O.x) * rD.x;
	float tmin = tinybvh::tinybvh_min( tx1, tx2 ), t = tinybvh::tinybvh_max( tx1, tx2 );
	const float ty1 = (bmin.y - O.y) * rD.y, ty2 = (bmax.y - O.y) * rD.y;
	tmin = tinybvh::tinybvh_max( tmin, tinybvh::tinybvh_min( ty1, ty2 ) );
	t = tinybvh::tinybvh_min( t, tinybvh::tinybvh_max( ty1, ty2 ) );
	const float tz1 = (bmin.z - O.z) * rD.z, tz2 = (bmax.z - O.z) * rD.z;
	tmin = tinybvh::tinybvh_max( tmin, tinybvh::tinybvh_min( tz1, tz2 ) );
	t = tinybvh::tinybvh_min( t, tinybvh::tinybvh_max( tz1, tz2 ) );
	if (t >= tmin && tmin < tmax && t >= 0) return tinybvh::tinybvh_max( 0.0f, tmin );
	return BVH_FAR;
}

static float TriDistance( const bvhvec3& O, const bvhvec3& D, const tinybvh::bvhvec4slice& verts, const uint32_t prim )
{
	const bvhvec3 v0( verts[prim * 3] ), v1( verts[prim * 3 + 1] ), v2( verts[prim * 3 + 2] );
	const bvhvec3 e1 = v1 - v0, e2 = v2 - v0, h = tinybvh::tinybvh_cross( D, e2 );
	const float a = tinybvh::tinybvh_dot( e1, h );
	if (fabsf( a ) < 1e-12f) return BVH_FAR;
	const float f = 1.0f / a;
	const bvhvec3 s = O - v0;
	const float u = f * tinybvh::tinybvh_dot( s, h );
	if (u < 0 || u > 1) return BVH_FAR;
	const bvhvec3 q = tinybvh::tinybvh_cross( s, e1 );
	const float v = f * tinybvh::tinybvh_dot( D, q );
	if (v < 0 || u + v > 1) return BVH_FAR;
	const float t = f * tinybvh::tinybvh_dot( e2, q );
	return t > 1e-5f ? t : BVH_FAR;
}

// CountPassTests: Gu et al.'s statistic. A node is counted when its box test passed and the
// traversal went on to its children or its primitives. A prune-test ends the walk there and
// says nothing about how the subtree is balanced, which is the thing the collapse needs to
// know; counting those too would just recover the surface area ratio.
static void CountPassTests( const tinybvh::BVH& bvh, const RaySet& rays, std::vector<uint32_t>& visit )
{
	visit.assign( bvh.usedNodes, 0u );
	struct Entry { uint32_t node; float dist; };
	Entry stack[STACK_SIZE];
	for (size_t r = 0; r < rays.Count(); r++)
	{
		const bvhvec3 O = rays.O[r], D = rays.D[r];
		const bvhvec3 rD( D.x == 0 ? BVH_FAR : 1.0f / D.x, D.y == 0 ? BVH_FAR : 1.0f / D.y, D.z == 0 ? BVH_FAR : 1.0f / D.z );
		float best = BVH_FAR;
		uint32_t stackPtr = 0, node = 0;
		if (BoxDistance( O, rD, bvh.bvhNode[0].aabbMin, bvh.bvhNode[0].aabbMax, best ) == BVH_FAR) continue;
		for (;;)
		{
			visit[node]++;
			const tinybvh::BVH::BVHNode& n = bvh.bvhNode[node];
			if (n.isLeaf())
			{
				for (uint32_t i = 0; i < n.triCount; i++)
					best = tinybvh::tinybvh_min( best, TriDistance( O, D, bvh.verts, bvh.primIdx[n.leftFirst + i] ) );
			}
			else
			{
				uint32_t l = n.leftFirst, rgt = l + 1;
				float dl = BoxDistance( O, rD, bvh.bvhNode[l].aabbMin, bvh.bvhNode[l].aabbMax, best );
				float dr = BoxDistance( O, rD, bvh.bvhNode[rgt].aabbMin, bvh.bvhNode[rgt].aabbMax, best );
				if (dl > dr) { const float t = dl; dl = dr, dr = t; const uint32_t c = l; l = rgt, rgt = c; }
				if (dl != BVH_FAR)
				{
					if (dr != BVH_FAR && stackPtr < STACK_SIZE) stack[stackPtr].node = rgt, stack[stackPtr++].dist = dr;
					node = l;
					continue;
				}
			}
			// pop the next node whose entry distance still beats the closest hit so far.
			for (;;)
			{
				if (stackPtr == 0) { node = (uint32_t)-1; break; }
				const Entry e = stack[--stackPtr];
				if (e.dist >= best) continue;	// prune-test: not counted.
				node = e.node;
				break;
			}
			if (node == (uint32_t)-1) break;
		}
	}
}

// BuildVisitField: pass-test counts become the probability of visiting each node, relative to
// the root. Below 'minVisits' the sample says nothing, so the node falls back to its parent's
// probability scaled by the surface area ratio - which is exactly what the collapse assumes
// everywhere today. The field therefore only differs from the default where it has evidence.
static void BuildVisitField( const tinybvh::BVH& bvh, const std::vector<uint32_t>& visit,
	const uint32_t minVisits, std::vector<float>& prob )
{
	prob.assign( bvh.usedNodes, 0.0f );
	const double root = visit[0] > 0 ? (double)visit[0] : 1.0;
	prob[0] = 1.0f;
	for (uint32_t n = 0; n < bvh.usedNodes; n++)
	{
		const tinybvh::BVH::BVHNode& node = bvh.bvhNode[n];
		if (node.isLeaf() || node.leftFirst + 1 >= bvh.usedNodes) continue;
		const float sa = SurfaceArea( node.aabbMin, node.aabbMax );
		for (uint32_t k = 0; k < 2; k++)
		{
			const uint32_t c = node.leftFirst + k;
			const tinybvh::BVH::BVHNode& child = bvh.bvhNode[c];
			if (visit[c] >= minVisits) prob[c] = (float)((double)visit[c] / root);
			else prob[c] = sa > 0 ? prob[n] * SurfaceArea( child.aabbMin, child.aabbMax ) / sa : prob[n];
		}
	}
}

// The SIMD kernels only return a step count in a debug build, and switching that on would
// change the very thing we are timing. So the structural metric is taken from the collapsed
// MBVH instead, with a plain scalar walk: child box tests and triangle tests per ray, which
// is what the collapse decides and is identical on every machine. Time is measured apart,
// and three of the six layouts have no CPU traversal at all, so for those it is all there is.
//
// boxTests and primTests are the raw structural counts. nodeTests and payPrims are the same
// walk costed the way the layout pays for it: a node is one box test over all of its slots at
// once, whatever their number, and a leaf of a quad layout costs a full quad even when it
// holds one primitive - BVH::LeafPrims applies the same rounding to the SAH it steers with.
template <typename M> static void CountWideSteps( const M& m, const RaySet& rays, const bool quadLeaves,
	double& boxTests, double& primTests, double& nodeTests, double& payPrims )
{
	double boxes = 0, prims = 0, nodes = 0, pay = 0;
	struct Entry { uint32_t node; float dist; };
	Entry stack[STACK_SIZE * 2];
	const tinybvh::BVH& src = m.bvh;
	for (size_t r = 0; r < rays.Count(); r++)
	{
		const bvhvec3 O = rays.O[r], D = rays.D[r];
		const bvhvec3 rD( D.x == 0 ? BVH_FAR : 1.0f / D.x, D.y == 0 ? BVH_FAR : 1.0f / D.y, D.z == 0 ? BVH_FAR : 1.0f / D.z );
		float best = BVH_FAR;
		uint32_t stackPtr = 0, node = 0;
		boxes += 1, nodes += 1; // the root box
		if (BoxDistance( O, rD, m.mbvhNode[0].aabbMin, m.mbvhNode[0].aabbMax, best ) == BVH_FAR) continue;
		for (;;)
		{
			const typename M::MBVHNode& n = m.mbvhNode[node];
			if (n.isLeaf())
			{
				prims += (double)n.triCount;
				pay += quadLeaves ? (double)((n.triCount + 3u) & ~3u) : (double)n.triCount;
				for (uint32_t i = 0; i < n.triCount; i++)
					best = tinybvh::tinybvh_min( best, TriDistance( O, D, src.verts, src.primIdx[n.firstTri + i] ) );
			}
			else
			{
				// one box test per occupied slot, the whole node at once in the real kernel.
				Entry hit[8];
				uint32_t hits = 0;
				nodes += 1;
				for (uint32_t k = 0; k < n.childCount; k++)
				{
					const uint32_t c = n.child[k];
					boxes += 1;
					const float d = BoxDistance( O, rD, m.mbvhNode[c].aabbMin, m.mbvhNode[c].aabbMax, best );
					if (d != BVH_FAR) hit[hits].node = c, hit[hits++].dist = d;
				}
				for (uint32_t a = 1; a < hits; a++) for (uint32_t b = a; b > 0 && hit[b - 1].dist < hit[b].dist; b--)
				{
					const Entry t = hit[b - 1]; hit[b - 1] = hit[b], hit[b] = t;
				}
				if (hits > 0)
				{
					for (uint32_t a = 0; a + 1 < hits && stackPtr < STACK_SIZE * 2; a++) stack[stackPtr++] = hit[a];
					node = hit[hits - 1].node;
					continue;
				}
			}
			for (;;)
			{
				if (stackPtr == 0) { node = (uint32_t)-1; break; }
				const Entry e = stack[--stackPtr];
				if (e.dist >= best) continue;
				node = e.node;
				break;
			}
			if (node == (uint32_t)-1) break;
		}
	}
	const double n = (double)rays.Count();
	boxTests = boxes / n, primTests = prims / n, nodeTests = nodes / n, payPrims = pay / n;
}

// The same walk over a binary tree, for the two 2-wide targets: one node test covers both
// children, and a BVH2 tests its primitives one at a time, so nothing is rounded here.
// rootTest tells the walk whether the kernel tests the root box before descending. BVH::
// Intersect does, and a ray that misses the scene pays one box test and stops. BVH_GPU
// cannot: its nodes hold the bounds of their children, so the root's own box is nowhere in
// the tree, and a ray that misses the scene pays for both of the root's children instead.
static void CountBinarySteps( const tinybvh::BVH& b, const RaySet& rays, const bool rootTest,
	double& boxTests, double& primTests, double& nodeTests )
{
	double boxes = 0, prims = 0, nodes = 0;
	struct Entry { uint32_t node; float dist; };
	Entry stack[STACK_SIZE * 2];
	for (size_t r = 0; r < rays.Count(); r++)
	{
		const bvhvec3 O = rays.O[r], D = rays.D[r];
		const bvhvec3 rD( D.x == 0 ? BVH_FAR : 1.0f / D.x, D.y == 0 ? BVH_FAR : 1.0f / D.y, D.z == 0 ? BVH_FAR : 1.0f / D.z );
		float best = BVH_FAR;
		uint32_t stackPtr = 0, node = 0;
		if (rootTest)
		{
			boxes += 1, nodes += 1; // the root box
			if (BoxDistance( O, rD, b.bvhNode[0].aabbMin, b.bvhNode[0].aabbMax, best ) == BVH_FAR) continue;
		}
		for (;;)
		{
			const tinybvh::BVH::BVHNode& n = b.bvhNode[node];
			if (n.isLeaf())
			{
				prims += (double)n.triCount;
				for (uint32_t i = 0; i < n.triCount; i++)
					best = tinybvh::tinybvh_min( best, TriDistance( O, D, b.verts, b.primIdx[n.leftFirst + i] ) );
			}
			else
			{
				nodes += 1, boxes += 2; // both children, one node test in the kernel
				uint32_t nearIdx = n.leftFirst, farIdx = n.leftFirst + 1;
				float nearD = BoxDistance( O, rD, b.bvhNode[nearIdx].aabbMin, b.bvhNode[nearIdx].aabbMax, best );
				float farD = BoxDistance( O, rD, b.bvhNode[farIdx].aabbMin, b.bvhNode[farIdx].aabbMax, best );
				if (nearD > farD) tinybvh::tinybvh_swap( nearD, farD ), tinybvh::tinybvh_swap( nearIdx, farIdx );
				if (nearD != BVH_FAR)
				{
					if (farD != BVH_FAR && stackPtr < STACK_SIZE * 2) stack[stackPtr].node = farIdx, stack[stackPtr++].dist = farD;
					node = nearIdx;
					continue;
				}
			}
			for (;;)
			{
				if (stackPtr == 0) { node = (uint32_t)-1; break; }
				const Entry e = stack[--stackPtr];
				if (e.dist >= best) continue;
				node = e.node;
				break;
			}
			if (node == (uint32_t)-1) break;
		}
	}
	const double n = (double)rays.Count();
	boxTests = boxes / n, primTests = prims / n, nodeTests = nodes / n;
}

// ----------------------------------------------------------------------------------------
// per-layout plumbing
//
// Six of the seven wide layouts are encodings of an MBVH<4> or an MBVH<8>, which is why one
// collapse and one structural metric serve all six. (BVH_GPU is the odd one out: it is
// 2-wide and encodes the binary tree directly, so it has no collapse to steer and is handled
// by EvaluateGPUTarget rather than by any of this.) They differ in three ways that matter: how
// many primitives a leaf slot holds, whether the collapse runs inside their ConvertFrom or
// has to be run first, and whether they have a file format of their own. The CPU layouts
// collapse from the binary tree themselves; the four GPU layouts take an MBVH that is
// already collapsed. WideBuild returns the MBVH the result was encoded from, which is what
// gets measured.
// ----------------------------------------------------------------------------------------

static const tinybvh::MBVH<4>* WideBuild( tinybvh::BVH4_CPU& o, tinybvh::MBVH<4>& m )
{
	o.bvh4.bvh.ReferenceFrom( m.bvh ), o.bvh4.ownBVH = false, o.bvh4.nodeProb = m.nodeProb;
	o.ConvertFrom( o.bvh4, true );	// leaves were merged by the optimizer's collapse
	return &o.bvh4;
}

static const tinybvh::MBVH<8>* WideBuild( tinybvh::BVH8_CPU& o, tinybvh::MBVH<8>& m )
{
	o.bvh8.bvh.ReferenceFrom( m.bvh ), o.bvh8.ownBVH = false, o.bvh8.nodeProb = m.nodeProb;
	o.ConvertFrom( o.bvh8, true );
	return &o.bvh8;
}

static const tinybvh::MBVH<4>* WideBuild( tinybvh::BVH4_GPU& o, tinybvh::MBVH<4>& m )
{
	m.ConvertFrom( m.bvh, true ), o.ConvertFrom( m, true );
	return &m;
}

static const tinybvh::MBVH<8>* WideBuild( tinybvh::BVH8_CWBVH& o, tinybvh::MBVH<8>& m )
{
	m.ConvertFrom( m.bvh, true ), o.ConvertFrom( m, true );
	return &m;
}

static const tinybvh::MBVH<4>* WideBuild( tinybvh::BVH4_AMD_HW& o, tinybvh::MBVH<4>& m )
{
	m.ConvertFrom( m.bvh, true ), o.ConvertFrom( m );
	return &m;
}

static const tinybvh::MBVH<8>* WideBuild( tinybvh::CWBVH8_AMD_HW& o, tinybvh::MBVH<8>& m )
{
	m.ConvertFrom( m.bvh, true ), o.ConvertFrom( m, true );
	return &m;
}

// Wall clock, best of a few runs: the differences between these trees are small enough that
// a single timed pass mostly measures the machine. The hardware formats have no traversal
// kernel on the CPU, so they report no time at all rather than a misleading one.
template <typename T> static double WideTime( const T& tree, const RaySet& rays, const uint32_t runs )
{
	double best = 1e30;
	for (uint32_t run = 0; run < runs; run++)
	{
		const Clock::time_point t0 = Clock::now();
		float sink = 0;
		for (size_t i = 0; i < rays.Count(); i++)
		{
			tinybvh::Ray ray( rays.O[i], rays.D[i] );
			tree.Intersect( ray );
			sink += ray.hit.t == BVH_FAR ? 0 : ray.hit.t;
		}
		const double ns = MilliSec( t0, Clock::now() ) * 1e6 / (double)rays.Count();
		if (sink >= 0 && ns < best) best = ns;
	}
	return best;
}

static double WideTime( const tinybvh::BVH4_AMD_HW&, const RaySet&, const uint32_t ) { return -1; }
static double WideTime( const tinybvh::CWBVH8_AMD_HW&, const RaySet&, const uint32_t ) { return -1; }

// Every layout now has a cache format of its own, so the collapse that won is what gets
// written, weighting included. The template is the fallback for anything added later that
// does not: for those the artifact would have to be the binary tree instead.
static bool WideSave( tinybvh::BVH4_CPU& o, const std::string& path ) { o.Save( path.c_str() ); return true; }
static bool WideSave( tinybvh::BVH8_CPU& o, const std::string& path ) { o.Save( path.c_str() ); return true; }
static bool WideSave( tinybvh::BVH8_CWBVH& o, const std::string& path ) { o.Save( path.c_str() ); return true; }
static bool WideSave( tinybvh::BVH4_GPU& o, const std::string& path ) { o.Save( path.c_str() ); return true; }
static bool WideSave( tinybvh::BVH4_AMD_HW& o, const std::string& path ) { o.Save( path.c_str() ); return true; }
static bool WideSave( tinybvh::CWBVH8_AMD_HW& o, const std::string& path ) { o.Save( path.c_str() ); return true; }
template <typename T> static bool WideSave( T&, const std::string& ) { return false; }

// Block sizes differ: the CPU layouts count 64-byte cache lines, the GPU ones 16-byte
// quadwords, and CWBVH keeps its triangles in a second array.
static double WideBytes( const tinybvh::BVH4_CPU& o ) { return (double)o.usedBlocks * 64; }
static double WideBytes( const tinybvh::BVH8_CPU& o ) { return (double)o.usedBlocks * 64; }
static double WideBytes( const tinybvh::BVH8_CWBVH& o ) { return (double)(o.usedBlocks + o.usedTriBlocks) * 16; }
template <typename T> static double WideBytes( const T& o ) { return (double)o.usedBlocks * 16; }

enum WideTargetId { TGT_BVH = 0, TGT_BVHGPU, TGT_BVH4CPU, TGT_BVH8CPU, TGT_BVH4GPU, TGT_BVH8CWBVH, TGT_BVH4AMDHW, TGT_CWBVH8AMDHW, TGT_COUNT };

struct TargetInfo
{
	const char* name;
	uint32_t leafPrims;		// primitives per leaf this layout is built for; 0: no cap, SAH decides
	uint32_t collapseLimit;	// leafPrimLimit for the collapse; 0 means it may not merge leaves
	bool fitLeaves;			// FitLeafs rather than SplitLeafs, as the layout's own Load does
	bool quadLeaves;		// leaf of four tested as one unit, so a partial leaf costs a full quad
};

static const TargetInfo g_target[TGT_COUNT] =
{
	{ "bvh",         0, 0, false, false },	// the plain binary tree: no collapse, no quads, SAH-sized leaves
	{ "bvhgpu",      0, 0, false, false },	// same tree, re-encoded; shares the binary trees with 'bvh'
	{ "bvh4cpu",     4, 4, false, true  },
	{ "bvh8cpu",     4, 4, false, true  },
	{ "bvh4gpu",     4, 0, false, true  },
	{ "bvh8cwbvh",   3, 3, true,  false },
	{ "bvh4amdhw",   1, 0, false, false },
	{ "cwbvh8amdhw", 2, 2, false, false }
};

// The candidates. Two builders with spatial splits, and the knobs of each that were measured
// to matter: the SBVH's bin count, and whether Bonsai HQ prunes its mini trees. Neither knob
// has a setting that is best everywhere - the best bin count moves with how much work the
// spatial splitter has to do - which is why they are sampled rather than tuned. Everything
// else that can be set on these builders was measured to change the final tree by nothing at
// all, or by less than the two ray sets disagree. The first CAND_DEFAULT entries are the set
// built without -sweep, so they must stay first.
struct CandidateInfo
{
	const char* name;
	bool binned;			// true: SBVH (binnedBVH). false: Bonsai HQ.
	uint32_t bins;			// SBVH: hqbvhbins. Ignored by Bonsai HQ.
	float prune;			// Bonsai HQ: bonsaiPruneFactor. Ignored by the SBVH.
};
static const CandidateInfo g_candidate[] = {
	{ "sbvh b8",   true,   8, 0.1f },	// the two defaults
	{ "bonsai-hq", false,  8, 0.1f },
	{ "sbvh b4",   true,   4, 0.1f },	// -sweep adds these
	{ "sbvh b16",  true,  16, 0.1f },
	{ "sbvh b32",  true,  32, 0.1f },
	{ "bonsai np", false,  8, 0.0f }
};
#define CAND_DEFAULT	2
#define CAND_MAX		(uint32_t)(sizeof( g_candidate ) / sizeof( CandidateInfo ))

static const char* CandidateName( const uint32_t c ) { return g_candidate[c].name; }

struct WideSettings
{
	bool target[TGT_COUNT] = {};
	uint32_t rays = 100000;			// sample rays per set; each yields a primary and a bounce.
	uint32_t minVisits = 8;			// below this a node's visit count is noise; use its area instead.
	bool weighted = true;			// also collapse steered by the measured visit field
	bool plain = true;				// also collapse steered by surface area, as the library does
	uint32_t timingRuns = 3;		// timed passes per tree; the best one is reported
	uint32_t candidates = CAND_DEFAULT;	// -sweep raises this to the full set; see g_candidate
	std::string outStem;
	bool Any() const { for (uint32_t t = 0; t < TGT_COUNT; t++) if (target[t]) return true; return false; }
};


struct WideResult
{
	uint32_t candidate;		// 0 = SBVH, 1 = Bonsai HQ
	bool weighted;			// collapse steered by the measured visit field
	double boxTests, primTests, ns;
	double cost;			// node tests at c_trav plus primitive tests at c_int; see below
};

// What the optimizer did to a candidate, kept so the winner can report it. The 'pre' pair is
// measured on the tree as the builder left it, before the references are recovered; both pairs
// use the same c_int and lQuads, so the two are comparable.
struct CandidateStats
{
	float sahPre, epoPre, sahPost, epoPost;
	double seconds;			// build plus optimize, without the measurements themselves
};

// Build one candidate binary tree and run the optimizer over it, with the leaf budget of the
// layouts it will feed. The layout's own leaf pass is run here as well, so that the node list
// is final before the visit field is measured on it: the collapse must see the same node
// indices the field was built on.
static void PrepareCandidate( tinybvh::BVH& bvh, const std::vector<bvhvec4>& tris, const uint32_t candidate,
	Optimizer::Settings set, const TargetInfo& info, CandidateStats& st )
{
	const Clock::time_point t0 = Clock::now();
	const CandidateInfo& cand = g_candidate[candidate];
	// A wide layout dictates its leaf size, and the builder may as well stop there, since the
	// optimizer explodes the leaves anyway and rebuilds them to that budget at the end. The plain
	// BVH dictates nothing: SAH decides, so the builder is left alone and the collapse is given a
	// budget that cannot bind.
	const bool capped = info.leafPrims > 0;
	set.maxLeafPrims = capped ? info.leafPrims : ~0u;
	bvh.settings.useSpatialSplits = true;
	bvh.settings.binnedBVH = cand.binned;		// SBVH with, Bonsai HQ without
	bvh.hqbvhbins = cand.bins;
	bvh.settings.bonsaiPruneFactor = cand.prune;
	bvh.hqLeafPrims = capped ? info.leafPrims : 0, bvh.c_int = 2, bvh.lQuads = info.quadLeaves;
	// RecoverRef below rebuilds each reference box by clipping its triangle to the box of the leaf
	// holding it, so smaller leaves recover the spatial splits more exactly - and for a leaf of one
	// they recover them exactly. Which is only worth doing if the optimizer is going to take the
	// leaves apart anyway; with -noexplode it works with the ones the builder produced.
	bvh.settings.keepSplitting = set.explodeLeaves;
	bvh.Build( tris.data(), (uint32_t)tris.size() / 3 );
	st.seconds = MilliSec( t0, Clock::now() ) / 1000.0;
	st.sahPre = bvh.SAHCost(), st.epoPre = bvh.EPOCost();
	const Clock::time_point t1 = Clock::now();
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
	Optimizer opt( verbose, set );
	opt.Run( refBox.data(), bvh.vertIdx );
	bvh.ConvertFrom( verbose );
	bvh.Compact();
	if (capped) { if (info.fitLeaves) bvh.FitLeafs( info.leafPrims ); else bvh.SplitLeafs( info.leafPrims ); }
	st.seconds += MilliSec( t1, Clock::now() ) / 1000.0;
	st.sahPost = bvh.SAHCost(), st.epoPost = bvh.EPOCost();
}

// What the optimizer achieved on the tree that won, which with several candidates in play is
// not the one any single line above reports.
static void ReportWinner( const CandidateStats& st )
{
	printf( "  SAH %10.4f -> %10.4f (%+6.2f%%)\n", st.sahPre, st.sahPost, 100.0f * (st.sahPost - st.sahPre) / st.sahPre );
	printf( "  EPO %10.4f -> %10.4f (%+6.2f%%)\n", st.epoPre, st.epoPost, 100.0f * (st.epoPost - st.epoPre) / st.epoPre );
}

// BVH_GPU packs a leaf into a single child slot: seven bits of triangle count, twenty-four
// bits of offset into primIdx. Both are clamped on encode rather than checked, so a tree
// that does not fit is encoded quietly and traverses into the wrong triangles. Neither limit
// is reachable with SAH-sized leaves on a sane mesh; 'quietly' is why they are tested anyway.
// The fat-leaf case is repaired in place, which is safe only because the plain BVH target is
// evaluated before this one - it sits earlier in WideTargetId, and the dispatch loop runs in
// enum order. The reference count is not repairable: at that size the layout is simply out.
static bool FitsGPULayout( tinybvh::BVH& bvh )
{
	if (bvh.idxCount > 0x1000000)
	{
		printf( "  skipped: %u references exceeds the 24-bit leaf offset this layout encodes.\n", bvh.idxCount );
		return false;
	}
	uint32_t fattest = 0;
	for (uint32_t n = 0; n < bvh.usedNodes; n++)
	{
		const tinybvh::BVH::BVHNode& node = bvh.bvhNode[n];
		if (node.isLeaf()) fattest = tinybvh::tinybvh_max( fattest, (uint32_t)node.triCount );
	}
	if (fattest > 127)
	{
		printf( "  note: leaf of %u, above the 127 this layout encodes; split to 127.\n", fattest );
		bvh.SplitLeafs( 127 );
	}
	return true;
}

// The BVH_GPU target: the same binary tree in the Aila & Laine node layout, where a node
// carries the bounds of its children and a leaf lives in a child slot rather than in a node
// of its own. Nothing about the tree changes, so like the plain BVH target there is no
// collapse to pick, no visit field, and one row per candidate. What does change is the cost:
// the missing root test (see CountBinarySteps) and the triangles, which this layout copies
// into an ordered array of its own and which dominate its size.
//
// The clock is the library's scalar reference kernel, not the GPU this layout is meant for.
// It is a real traversal of the real encoding and comparable with the 'bvh' row above it -
// 64-byte nodes against 32, no indirection through primIdx - but it says nothing about what
// the tree would do on a GPU.
static void EvaluateGPUTarget( const TargetInfo& info, tinybvh::BVH* bvh, const uint32_t candidates,
	const CandidateStats* stat, const RaySet& score, const WideSettings& w, const std::string& stem )
{
	printf( "\n%s:\n", info.name );
	printf( "  %-10s %-9s %10s %10s %9s %9s %10s\n", "tree", "collapse", "box/ray", "prim/ray", "cost/ray", "ns/ray", "size" );
	uint32_t bestIdx = 0, fastestIdx = 0;
	double bestCost = 1e30, bestNs = 1e30, winNs = 0;
	for (uint32_t c = 0; c < candidates; c++)
	{
		if (!FitsGPULayout( bvh[c] )) return;
		double box = 0, prim = 0, nodes = 0;
		CountBinarySteps( bvh[c], score, false, box, prim, nodes );
		const double cost = nodes * bvh[c].c_trav + prim * bvh[c].c_int;
		tinybvh::BVH_GPU tree;
		tree.ConvertFrom( bvh[c], true );
		const double ns = WideTime( tree, score, w.timingRuns );
		// nodes plus the ordered triangles, which this layout allocates and owns.
		const double mb = (double)tree.usedNodes * sizeof( tinybvh::BVH_GPU::BVHNode )
			+ (double)tree.idxCount * 3 * sizeof( bvhvec4 );
		printf( "  %-10s %-9s %10.2f %10.2f %9.2f%9.1f %7.1f MB\n", CandidateName( c ), "-", box, prim, cost, ns, mb / (1024 * 1024) );
		if (ns < bestNs) bestNs = ns, fastestIdx = c;
		if (cost < bestCost) bestCost = cost, bestIdx = c, winNs = ns;
	}
	if (candidates == 0) return;
	printf( "  winner: %s\n", CandidateName( bestIdx ) );
	if (fastestIdx != bestIdx) printf( "  note: %s was faster on the clock (%.1f vs %.1f ns); cost decided.\n",
		CandidateName( fastestIdx ), bestNs, winNs );
	ReportWinner( stat[bestIdx] );
	// No format of its own: BVH_GPU::Load reads a plain tree and converts it, and the
	// conversion is lossless, so the binary tree is the whole artifact.
	const std::string path = stem + "_" + info.name + ".bvh";
	bvh[bestIdx].Save( path.c_str() );
	printf( "  output: %s (binary tree; %s converts it on load)\n", path.c_str(), info.name );
}

// The plain BVH target. The optimized binary tree is the artifact, so there is no collapse to
// pick and one row per candidate rather than two - and no visit field, which only steers a
// collapse. Scoring and selection are otherwise identical to the wide targets.
static void EvaluateBinaryTarget( const TargetInfo& info, tinybvh::BVH* bvh, const uint32_t candidates,
	const CandidateStats* stat, const RaySet& score, const WideSettings& w, const std::string& stem )
{
	printf( "\n%s:\n", info.name );
	printf( "  %-10s %-9s %10s %10s %9s %9s %10s\n", "tree", "collapse", "box/ray", "prim/ray", "cost/ray", "ns/ray", "size" );
	uint32_t bestIdx = 0, fastestIdx = 0;
	double bestCost = 1e30, bestNs = 1e30, winNs = 0;
	for (uint32_t c = 0; c < candidates; c++)
	{
		double box = 0, prim = 0, nodes = 0;
		CountBinarySteps( bvh[c], score, true, box, prim, nodes );
		const double cost = nodes * bvh[c].c_trav + prim * bvh[c].c_int;
		const double ns = WideTime( bvh[c], score, w.timingRuns );
		const double mb = (double)(bvh[c].usedNodes * sizeof( tinybvh::BVH::BVHNode ) + bvh[c].idxCount * sizeof( uint32_t ));
		printf( "  %-10s %-9s %10.2f %10.2f %9.2f%9.1f %7.1f MB\n", CandidateName( c ), "-", box, prim, cost, ns, mb / (1024 * 1024) );
		if (ns < bestNs) bestNs = ns, fastestIdx = c;
		if (cost < bestCost) bestCost = cost, bestIdx = c, winNs = ns;
	}
	if (candidates == 0) return;
	printf( "  winner: %s\n", CandidateName( bestIdx ) );
	if (fastestIdx != bestIdx) printf( "  note: %s was faster on the clock (%.1f vs %.1f ns); cost decided.\n",
		CandidateName( fastestIdx ), bestNs, winNs );
	ReportWinner( stat[bestIdx] );
	const std::string path = stem + "_" + info.name + ".bvh";
	bvh[bestIdx].Save( path.c_str() );
	printf( "  output: %s\n", path.c_str() );
}

template <typename T, int M> static void EvaluateTarget( const TargetInfo& info, tinybvh::BVH* bvh,
	const uint32_t candidates, const CandidateStats* stat, const std::vector<float>* prob, const RaySet& score,
	const WideSettings& w, const std::string& stem )
{
	printf( "\n%s:\n", info.name );
	printf( "  %-10s %-9s %10s %10s %9s %9s %10s\n", "tree", "collapse", "box/ray", "prim/ray", "cost/ray", "ns/ray", "size" );
	std::vector<WideResult> results;
	tinybvh::MBVH<M> keep[CAND_MAX * 2];	// the GPU layouts reference their source MBVH; it has to outlive them.
	T best;
	uint32_t bestIdx = 0;
	double bestScore = 1e30;
	for (uint32_t c = 0; c < candidates; c++) for (uint32_t pass = 0; pass < 2; pass++)
	{
		const bool weighted = pass == 1;
		if (weighted && !w.weighted) continue;
		if (!weighted && !w.plain) continue;
		tinybvh::MBVH<M>& m = keep[c * 2 + pass];
		m.bvh.ReferenceFrom( bvh[c] ), m.ownBVH = false;
		m.leafPrimLimit = info.collapseLimit;
		m.nodeProb = weighted ? prob[c].data() : 0;
		T tree;
		const tinybvh::MBVH<M>* source = WideBuild( tree, m );
		WideResult r;
		r.candidate = c, r.weighted = weighted;
		double nodeTests = 0, payPrims = 0;
		CountWideSteps( *source, score, info.quadLeaves, r.boxTests, r.primTests, nodeTests, payPrims );
		r.cost = nodeTests * bvh[c].c_trav + payPrims * bvh[c].c_int;
		r.ns = WideTime( tree, score, w.timingRuns );
		results.push_back( r );
		printf( "  %-10s %-9s %10.2f %10.2f %9.2f", CandidateName( c ), weighted ? "visits" : "area", r.boxTests, r.primTests, r.cost );
		if (r.ns < 0) printf( "%9s", "-" ); else printf( "%9.1f", r.ns );
		printf( " %7.1f MB\n", WideBytes( tree ) / (1024 * 1024) );
		if (r.cost < bestScore) bestScore = r.cost, bestIdx = (uint32_t)results.size() - 1, best = tinybvh::tinybvh_move( tree );
	}
	if (results.empty()) return;
	const WideResult& win = results[bestIdx];
	const WideResult* fastest = &results[0];
	for (const WideResult& r : results) if (r.ns >= 0 && (fastest->ns < 0 || r.ns < fastest->ns)) fastest = &r;
	printf( "  winner: %s, %s collapse\n", CandidateName( win.candidate ), win.weighted ? "visit-weighted" : "area-weighted" );
	if (fastest->ns >= 0 && fastest != &win) printf( "  note: %s / %s was faster on the clock (%.1f vs %.1f ns); cost decided.\n",
		CandidateName( fastest->candidate ), fastest->weighted ? "visits" : "area", fastest->ns, win.ns );
	ReportWinner( stat[win.candidate] );
	const std::string path = stem + "_" + info.name + ".bvh";
	if (WideSave( best, path )) printf( "  output: %s\n", path.c_str() );
	else
	{
		// No format of its own: the artifact is the binary tree, which this layout's Load
		// converts. The collapse is redone there, so a weighted winner only survives if the
		// field goes with it - hence the sidecar.
		bvh[win.candidate].Save( path.c_str() );
		printf( "  output: %s (binary tree; %s converts it on load)\n", path.c_str(), info.name );
		if (win.weighted)
		{
			const std::string probPath = stem + "_" + info.name + ".prob";
			FILE* f = fopen( probPath.c_str(), "wb" );
			if (f)
			{
				const uint32_t n = (uint32_t)prob[win.candidate].size();
				fwrite( &n, sizeof( uint32_t ), 1, f );
				fwrite( prob[win.candidate].data(), sizeof( float ), n, f );
				fclose( f );
				printf( "  output: %s (visit field; feed it to MBVH::nodeProb or the weighting is lost)\n", probPath.c_str() );
			}
		}
	}
}

static int RunWideTargets( const std::string& meshPath, const Optimizer::Settings& set, const WideSettings& w )
{
	std::vector<bvhvec4> tris;
	if (!LoadMesh( meshPath.c_str(), tris ))
	{
		printf( "error: cannot read '%s' as a mesh (uint32 triangle count, then three float4 per triangle).\n", meshPath.c_str() );
		return 1;
	}
	printf( "input  : %s\n         %u triangles\n", meshPath.c_str(), (uint32_t)tris.size() / 3 );
	const std::string stem = w.outStem.empty() ? StripExtension( meshPath ) : w.outStem;
	// Targets that want the same leaf budget share their binary trees; the rest get their own,
	// since the optimizer's final collapse is what produces leaves of the right size.
	RaySet fit, score;
	bool done[TGT_COUNT] = {};
	for (uint32_t t = 0; t < TGT_COUNT; t++)
	{
		if (!w.target[t] || done[t]) continue;
		const TargetInfo& regime = g_target[t];
		if (regime.leafPrims == 0) printf( "\n=== binary trees, SAH-sized leaves ===\n" );
		else printf( "\n=== binary trees, %u primitive%s per leaf%s ===\n", regime.leafPrims,
			regime.leafPrims == 1 ? "" : "s", regime.quadLeaves ? ", quad-packed" : "" );
		tinybvh::BVH bvh[CAND_MAX];
		CandidateStats stat[CAND_MAX] = {};
		for (uint32_t c = 0; c < w.candidates; c++)
		{
			PrepareCandidate( bvh[c], tris, c, set, regime, stat[c] );
			printf( "  %-10s %u references, %u nodes, SAH %.4f, EPO %.4f, %.1f s\n", CandidateName( c ),
				bvh[c].idxCount, bvh[c].usedNodes, stat[c].sahPost, stat[c].epoPost, stat[c].seconds );
		}
		if (fit.Count() == 0)
		{
			// One ray distribution for every target and every candidate, drawn once.
			GenerateRays( bvh[0], w.rays, set.seed ^ 0x9E3779B9u, fit );
			GenerateRays( bvh[0], w.rays, set.seed ^ 0x85EBCA6Bu, score );
			printf( "  ray sets: %u to fit the visit field, %u to score, from disjoint seeds\n",
				(uint32_t)fit.Count(), (uint32_t)score.Count() );
		}
		std::vector<float> prob[CAND_MAX];
		if (w.weighted) for (uint32_t c = 0; c < w.candidates; c++)
		{
			std::vector<uint32_t> visit;
			CountPassTests( bvh[c], fit, visit );
			BuildVisitField( bvh[c], visit, w.minVisits, prob[c] );
		}
		for (uint32_t u = t; u < TGT_COUNT; u++)
		{
			if (!w.target[u] || g_target[u].leafPrims != regime.leafPrims) continue;
			if (g_target[u].quadLeaves != regime.quadLeaves) continue;	// different leaf cost, different tree
			done[u] = true;
			switch (u)
			{
			case TGT_BVH: EvaluateBinaryTarget( g_target[u], bvh, w.candidates, stat, score, w, stem ); break;
			case TGT_BVHGPU: EvaluateGPUTarget( g_target[u], bvh, w.candidates, stat, score, w, stem ); break;
			case TGT_BVH4CPU: EvaluateTarget<tinybvh::BVH4_CPU, 4>( g_target[u], bvh, w.candidates, stat, prob, score, w, stem ); break;
			case TGT_BVH8CPU: EvaluateTarget<tinybvh::BVH8_CPU, 8>( g_target[u], bvh, w.candidates, stat, prob, score, w, stem ); break;
			case TGT_BVH4GPU: EvaluateTarget<tinybvh::BVH4_GPU, 4>( g_target[u], bvh, w.candidates, stat, prob, score, w, stem ); break;
			case TGT_BVH8CWBVH: EvaluateTarget<tinybvh::BVH8_CWBVH, 8>( g_target[u], bvh, w.candidates, stat, prob, score, w, stem ); break;
			case TGT_BVH4AMDHW: EvaluateTarget<tinybvh::BVH4_AMD_HW, 4>( g_target[u], bvh, w.candidates, stat, prob, score, w, stem ); break;
			case TGT_CWBVH8AMDHW: EvaluateTarget<tinybvh::CWBVH8_AMD_HW, 8>( g_target[u], bvh, w.candidates, stat, prob, score, w, stem ); break;
			}
		}
	}
	return 0;
}

static void Usage()
{
	printf(
		"usage: bvhopt <file.bvh> [options]                     optimize a binary BVH\n"
		"       bvhopt <mesh.bin> --target <list> [options]     build an optimized wide BVH\n"
		"\n"
		"  A .bvh must have been saved with its geometry, which is what BVH::Save does by\n"
		"  default. EPO is defined over the triangles, so a file without them is of no use here.\n"
		"\n"
		"  With --target the input is a mesh instead (uint32 triangle count, then three float4\n"
		"  per triangle) and the output is a finished wide BVH. Both an SBVH and a Bonsai with\n"
		"  spatial splits are built, optimized and collapsed, each with and without the measured\n"
		"  visit weighting, and the variant with the fewest box tests is written out. Targets\n"
		"  that want the same leaf size share their binary trees; the others get their own.\n"
		"  Every layout with a cache format of its own gets the collapsed tree, which its Load\n"
		"  reads back directly; the rest get the binary tree their Load converts.\n"
		"\n"
		"  --target L         comma-separated, or 'all': bvh, bvhgpu, bvh4cpu, bvh8cpu,\n"
		"                     bvh4gpu, bvh8cwbvh, bvh4amdhw, cwbvh8amdhw\n"
		"  -rays N            sample rays per set (default 100000; each adds one bounce)\n"
		"  -visitmin N        visit count below which a node falls back to surface area (default 8)\n"
		"  -timingruns N      timed passes per tree, best reported (default 3)\n"
		"  -noweight          skip the visit-weighted collapse\n"
		"  -noplain           skip the surface-area collapse\n"
		"  -sweep             try four SBVH bin counts and both Bonsai prune settings, not just\n"
		"                     the defaults: six binary trees per leaf budget instead of two.\n"
		"                     Costs three times the build time and about 40%% more memory\n"
		"  -out STEM          output name stem (default: the mesh name without its extension)\n"
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
	WideSettings wide;
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
		else if (a == "-rays") wide.rays = num( 100000 );
		else if (a == "-visitmin") wide.minVisits = num( 8 );
		else if (a == "-timingruns") wide.timingRuns = num( 3 );
		else if (a == "-noweight") wide.weighted = false;
		else if (a == "-noplain") wide.plain = false;
		else if (a == "-sweep") wide.candidates = CAND_MAX;
		else if (a == "-out") { if (i + 1 < argc) wide.outStem = argv[++i]; }
		else if (a == "--target" || a == "-target")
		{
			const std::string v = i + 1 < argc ? argv[++i] : "";
			for (size_t p = 0; p < v.size(); )
			{
				const size_t comma = v.find( ',', p );
				const std::string t = v.substr( p, comma == std::string::npos ? comma : comma - p );
				if (t == "all") for (uint32_t k = 0; k < TGT_COUNT; k++) wide.target[k] = true;
				else
				{
					uint32_t k = 0;
					while (k < TGT_COUNT && t != g_target[k].name) k++;
					if (k == TGT_COUNT) { printf( "error: unknown target '%s'.\n", t.c_str() ); return 1; }
					wide.target[k] = true;
				}
				if (comma == std::string::npos) break;
				p = comma + 1;
			}
		}
		else if (a == "-out") { if (i + 1 < argc) wide.outStem = argv[++i]; }
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
	if (!wide.weighted && !wide.plain) { printf( "error: -noweight and -noplain leave nothing to build.\n" ); return 1; }
	if (wide.Any()) return RunWideTargets( bvhPath, set, wide );

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
