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
#include <memory>

using tinybvh::bvhvec3;
using tinybvh::bvhvec4;

static const uint32_t MAX_TREELET = 12;
// The agglomerative sweep is O(n^3) in the treelet size rather than O(3^n), so it reaches
// sizes the exact DP cannot: restructurings that need thirty leaves to express are invisible
// to a twelve-leaf treelet no matter how many times it is swept.
static const uint32_t MAX_AGGLOM = 48;

// What the optimizer is asked to minimize. Every stage in this file ultimately answers to one
// of these, and they do not agree: SAH is a model, EPO corrects the part of that model which
// ignores where rays stop, and ray cost is the thing itself, counted on real rays through the
// tree the run would actually deliver, in the layout it would deliver it in.
enum Metric : uint32_t { METRIC_SAH = 0, METRIC_EPO, METRIC_RAYCOST };
static const char* g_metricName[3] = { "sah", "epo", "raycost" };

// How hard to look. These are rungs of a ladder, not three separate configurations: hq starts
// from the tree fast produced and extreme starts from the tree hq produced, and the best tree
// by the chosen metric over the whole climb is what comes out. That is what makes "extreme is
// never worse than hq" true rather than merely likely - more search can land in a worse basin,
// which is exactly what annealing is for, so the guarantee has to come from keeping the best.
enum ProfileLevel : uint32_t { PROF_FAST = 0, PROF_HQ, PROF_EXTREME };
static const char* g_profileName[3] = { "fast", "hq", "extreme" };

// AnnealAccept: the acceptance rule, in one place, for every descent in this file. A move that
// lowers the objective is always taken; one that raises it by 'delta' is taken with probability
// exp( -delta / T ), which is zero when the temperature is. Every objective here is in units of
// area times cost, and so is T, which makes the ratio scale free.
static bool AnnealAccept( const float proposed, const float current, const float T, uint32_t& seed )
{
	if (proposed < current) return true;
	if (!(T > 0)) return false;
	const float delta = proposed - current;
	if (!(delta < 20.0f * T)) return false;	// also catches the infinities a failed move returns
	return tinybvh::tinybvh_rndfloat( seed ) < expf( -delta / T );
}

// AnnealTemp: a cooling schedule along a pass of 'total' steps, required to finish cold - the
// last third runs at zero, so whatever a pass touches last is left in a minimum rather than in
// a sample. The treelet sweep walks bottom-up and so ends at the root; the rebuild pass is
// ordered by cost and is handed the schedule reversed, which leaves the costliest subtrees to
// the strict descent and anneals only the cheap tail.
static float AnnealTemp( const float T0, const uint32_t done, const uint32_t total )
{
	if (!(T0 > 0) || total == 0) return 0;
	const uint32_t cold = total - total / 3;
	if (done >= cold || cold == 0) return 0;
	return T0 * expf( -3.0f * (float)done / (float)cold );
}
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

// RoundGate: a score the optimizer consults at the end of each round. SAH is a model of what
// a traversal costs; this is a measurement of it, taken on the tree the run would produce if
// it stopped here - collapsed to the final leaf budget, encoded in the target's own layout,
// and walked with a ray set. A round that does not lower it is undone.
//
// The optimizer knows nothing about ray sets or layouts and should not: the measurement lives
// outside it and enters through this one virtual call.
struct RoundGate
{
	virtual ~RoundGate() {}
	virtual double Score( const tinybvh::BVH_Verbose& tree ) const = 0;
	virtual const char* Unit() const = 0;
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
		uint32_t agglomSize = 0;		// agglomerative treelet leaves; 0 disables. At most MAX_AGGLOM.
		// Starting temperature of the annealed reinsertion, as a fraction of the root box area.
		// 0 keeps the strictly descending walk. The schedule cools to zero inside the call, so a
		// call still ends in a local minimum and the round above it sees an ordinary improvement.
		float anneal = 0;
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
	// A round that does not lower the measured score is undone, but one such round is not a
	// floor: the stages are deterministic, so the retry is given a different seed and a
	// stochastic reinsertion stride, which samples a different part of the tree. Give up after
	// this many undone rounds in a row. 0 restores the stop-on-first-failure rule.
	uint32_t gatePatience = 3;
		bool verbose = true;			// report progress per round.
	};

	Optimizer( tinybvh::BVH_Verbose& bvh, const Settings& set ) : b( bvh ), s( set ) {}
	void Run( Ref* refBox, const uint32_t* vertIdx );
	// Null by default, in which case rounds are judged by SAH, or by the EPO-weighted metric
	// when -epogate asks for it. Set it and both are replaced by the gate's own score.
	void SetGate( const RoundGate* g ) { gate = g; }

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
	void OptimizeSerial( const uint32_t iterations, const bool extreme, bool stochastic, uint32_t seed, float temp0 );
	uint32_t OptExSweepSplit( uint32_t* leaf, const uint32_t count, const float jitter, uint32_t& seed, State& st );
	uint32_t OptExBuildTopology( uint32_t* leaf, const uint32_t count, uint32_t* slots, uint32_t& slotPtr,
		const uint32_t parent, const float jitter, uint32_t& seed, float& interiorCost, State& st );
	void OptExExplode( const Ref* refBox, State& st );
	bool OptExTreelet( const uint32_t nodeIdx, const uint32_t maxL, State& st, float T );
	bool OptExTreeletAgglom( const uint32_t nodeIdx, const uint32_t maxL, State& st, float T );
	void OptExTreeletSweep( const Settings& s, State& st, float T0 );
	void OptExRebuildSubtrees( const Settings& s, State& st, float T0 );
	void OptExUnsplit( State& st );
	void OptExCompactRefs();
	void OptExCollapse( const uint32_t maxLeafPrims, State& st );
	void OptExExactBottom( const Settings& s, State& st );
	float OptExEPOArea( const uint32_t subtreeRoot, const uint32_t nodeIdx, const uint32_t* vertIdx ) const;
	float OptExPrimAreaSum( const uint32_t* vertIdx ) const;
	float OptExCombined( const uint32_t* vertIdx, const float primArea, float& sahOut ) const;
	double GateScore( State& st, Node* scratchNode, uint32_t* scratchPrim );

	void Progress( const uint32_t round, const Clock::time_point& t0, const char* stage );

	tinybvh::BVH_Verbose& b;
	Settings s;
	const uint32_t* epoVertIdx = 0;
	float epoPrimArea = 1;
	bool holes = false;		// nodes made unreachable by a stage; cleared by the next Compact()
	Ref* refs = 0;			// the live reference boxes; OptExCompactRefs keeps them in step
	uint32_t lastAnnealed = 0;	// uphill reinsertions accepted by the last OptimizeSerial call
	float lastUphill = 0;		// ..and the total area they cost, for the verbose log
	const RoundGate* gate = 0;
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
bool Optimizer::OptExTreelet( const uint32_t nodeIdx, const uint32_t maxL, State& st, float T )
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
	// Without the exact test the DP must beat the current topology on its own terms - or, while
	// the temperature is above zero, lose to it by a small enough margin. The DP's optimum is
	// optimal only over this treelet; letting a worse one through occasionally is how the sweep
	// reaches arrangements that no single treelet can see from here.
	uint32_t seed = st.seed + nodeIdx * 2654435761u + 7u;
	if (!AnnealAccept( cost[full], st.subCost[nodeIdx], T, seed )) return false;
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

// OptExTreeletAgglom: the same treelet, restructured by agglomerative clustering instead of by
// exact DP. The DP is optimal but costs O(3^n) in the number of treelet leaves, which caps it
// near twelve; clustering is O(n^3) and gives up optimality, so it can be handed thirty or
// fifty leaves instead. The trade is worth making because a treelet that is too small cannot
// express the restructuring at all, however optimally it is solved.
//
// Walter et al.'s rule, through this tool's own area term: repeatedly merge the pair of live
// clusters whose union has the smallest effective area, until one is left. The merge list is
// built first and written back only if it beats the topology that is already there, so a
// treelet the clustering cannot improve is left exactly as it was.
bool Optimizer::OptExTreeletAgglom( const uint32_t nodeIdx, const uint32_t maxL, State& st, float T )
{
	Node& root = b.bvhNode[nodeIdx];
	if (root.isLeaf()) return false;
	uint32_t leaf[MAX_AGGLOM], interior[MAX_AGGLOM];
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
	if (L < 4) return false;	// below this the DP sweep already solves it exactly
	// every open adds one interior node and one leaf, so there are exactly L-1 slots to fill.
	struct Cluster { bvhvec3 bmin, bmax; float cost, own; uint32_t node; bool live; };
	Cluster cl[MAX_AGGLOM];
	for (uint32_t i = 0; i < L; i++)
	{
		const uint32_t n = leaf[i];
		cl[i].bmin = b.bvhNode[n].aabbMin, cl[i].bmax = b.bvhNode[n].aabbMax;
		cl[i].cost = st.subCost[n], cl[i].own = st.ownArea[n], cl[i].node = n, cl[i].live = true;
	}
	struct Merge { uint32_t a, b; bvhvec3 bmin, bmax; float cost, own; };
	Merge mg[MAX_AGGLOM];
	uint32_t merges = 0;
	for (uint32_t live = L; live > 1; live--)
	{
		uint32_t bi = (uint32_t)-1, bj = 0;
		float bestD = BVH_FAR;
		bvhvec3 bestMin = cl[0].bmin, bestMax = cl[0].bmax;
		float bestOwn = 0;
		for (uint32_t i = 0; i < L; i++) if (cl[i].live) for (uint32_t j = i + 1; j < L; j++) if (cl[j].live)
		{
			const bvhvec3 mn = tinybvh::tinybvh_min( cl[i].bmin, cl[j].bmin );
			const bvhvec3 mx = tinybvh::tinybvh_max( cl[i].bmax, cl[j].bmax );
			// members lie inside their own boxes, so the own-area sum is exact, as in the DP.
			const float own = cl[i].own + cl[j].own;
			const float d = EffArea( mn, mx, own, st );
			if (d < bestD) bestD = d, bi = i, bj = j, bestMin = mn, bestMax = mx, bestOwn = own;
		}
		if (bi == (uint32_t)-1) return false;	// cannot happen with live > 1, but do not write back on it
		Merge& m = mg[merges++];
		m.a = cl[bi].node, m.b = cl[bj].node, m.bmin = bestMin, m.bmax = bestMax, m.own = bestOwn;
		m.cost = cl[bi].cost + cl[bj].cost + b.c_trav * bestD;
		cl[bi].bmin = bestMin, cl[bi].bmax = bestMax, cl[bi].own = bestOwn, cl[bi].cost = m.cost;
		// tag the surviving cluster as "the result of merge m"; the slot it will occupy is only
		// decided at write-back, so nothing is claimed until the whole clustering has won.
		cl[bi].node = 0x80000000u | (merges - 1);
		cl[bj].live = false;
	}
	if (merges != L - 1) return false;
	uint32_t aseed = st.seed + nodeIdx * 2654435761u + 13u;
	if (!AnnealAccept( mg[merges - 1].cost, st.subCost[nodeIdx], T, aseed )) return false;
	// Write back. Merge m takes slot interior[L-2-m], so the last merge lands in nodeIdx and a
	// merge's children, being earlier merges, always sit at higher interior indices than it does.
	const uint32_t rootParent = b.bvhNode[nodeIdx].parent;
	for (uint32_t m = 0; m < merges; m++)
	{
		const uint32_t slot = interior[L - 2 - m];
		// a tagged child is an earlier merge; resolve it to the slot that merge was given.
		uint32_t ca = mg[m].a, cb = mg[m].b;
		if (ca & 0x80000000u) ca = interior[L - 2 - (ca & 0x7fffffffu)];
		if (cb & 0x80000000u) cb = interior[L - 2 - (cb & 0x7fffffffu)];
		Node& n = b.bvhNode[slot];
		n.left = ca, n.right = cb, n.triCount = 0, n.firstTri = 0;
		n.parent = (m == merges - 1) ? rootParent : 0;	// fixed up by the parent pass below
		n.aabbMin = mg[m].bmin, n.aabbMax = mg[m].bmax;
		st.subCost[slot] = mg[m].cost, st.refCount[slot] = 0;
		b.bvhNode[ca].parent = slot, b.bvhNode[cb].parent = slot;
	}
	b.bvhNode[nodeIdx].parent = rootParent;
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
void Optimizer::OptimizeSerial( const uint32_t iterations, const bool extreme, bool stochastic, uint32_t seed, float temp0 )
{
	BVH_FATAL_ERROR_IF( b.bvhNode == 0, "Optimizer::OptimizeSerial, no nodes." );
	BVH_FATAL_ERROR_IF( holes, "Optimizer::OptimizeSerial, tree has holes; compact it first." );
	if (iterations == 0 || b.usedNodes < 6 || b.bvhNode[0].isLeaf()) return;
	// working memory, allocated once.
	SortItem* sortList = (SortItem*)tinybvh::malloc64( b.usedNodes * sizeof( SortItem ) );
	const uint32_t journalCap = 4096; // >> 3x max tree depth
	RefitRecord* journal = (RefitRecord*)tinybvh::malloc64( journalCap * sizeof( RefitRecord ) );
	const double minGain = -1e-7 * (double)b.bvhNode[0].SA();
	// optimize by reinserting subtrees with a high cost - Section 3.4 of the paper.
	// Annealing schedule. The walk below is a strict descent: a reinsertion is kept only if it
	// lowers the tree's total area, so it stops at the first arrangement from which every single
	// move is uphill. Accepting an uphill move with probability exp( -delta / T ) lets it leave
	// that basin, and cooling T to zero before the call returns means it still ends in a local
	// minimum - so the round above sees an ordinary improvement and needs to know nothing of this.
	// The last third runs cold, which is what makes the end state a minimum rather than a sample.
	const uint32_t cold = iterations - iterations / 3;
	float annealed = 0, uphill = 0;
	for (uint32_t i = 0; i < iterations; i++)
	{
		float T = 0;
		if (temp0 > 0 && i < cold) T = temp0 * expf( -3.0f * (float)i / (float)tinybvh::tinybvh_max( 1u, cold ) );
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
			// keep the change if it meaningfully reduced the SAH cost of the tree, or - while the
			// temperature is above zero - if the coin says to keep it anyway. deltaArea is in area
			// units and so is T, so the ratio is scale free; the guard also rejects the BVH_FAR
			// that RefitUp returns on journal overflow without calling expf on it.
			if (deltaArea < minGain) { accepted++; continue; }
			if (T > 0 && deltaArea < 20.0 * (double)T &&
				tinybvh::tinybvh_rndfloat( seed ) < expf( -(float)deltaArea / T ))
			{
				accepted++, annealed++, uphill += (float)deltaArea;
				continue;
			}
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
	lastAnnealed = (uint32_t)annealed, lastUphill = uphill;
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


void Optimizer::OptExTreeletSweep( const Settings& s, State& st, float T0 )
{
	OptExComputeCosts( st );
	const uint32_t tiny = (uint32_t)tinybvh::tinybvh_clamp( (int32_t)s.treeletSize, 3, (int32_t)MAX_TREELET );
	const uint32_t large = (uint32_t)tinybvh::tinybvh_clamp( (int32_t)s.treeletSizeTop, (int32_t)tiny, (int32_t)MAX_TREELET );
	const uint32_t topRefs = tinybvh::tinybvh_max( (uint32_t)1, (uint32_t)(st.refCount[0] / 64) );
	const uint32_t agglom = (uint32_t)tinybvh::tinybvh_clamp( (int32_t)s.agglomSize, 0, (int32_t)MAX_AGGLOM );
	uint32_t tried = 0, won = 0;
	for (uint32_t i = st.orderCount; i-- > 0; )
	{
		const uint32_t n = st.order[i];
		if (b.bvhNode[n].isLeaf()) continue; // a sweep may have turned it into one
		// the walk ends at the root, so the schedule leaves the top of the tree to a cold pass.
		const float T = AnnealTemp( T0, st.orderCount - 1 - i, st.orderCount );
		OptExTreelet( n, st.refCount[n] >= topRefs ? large : tiny, st, T );
		// The clustering is cubic in the treelet size, so it is not spent on nodes too small to
		// fill one: below a few times the treelet size there is nothing for it to rearrange that
		// the DP sweep has not already solved exactly. Both passes refuse a treelet they cannot
		// improve, so running them one after the other on the same node is safe and cumulative.
		if (agglom > large && st.refCount[n] >= agglom * 4 && !b.bvhNode[n].isLeaf())
		{
			tried++;
			if (OptExTreeletAgglom( n, agglom, st, T )) won++;
		}
	}
	if (s.verbose && tried) printf( "  agglomerative treelets: %u of %u nodes restructured\n", won, tried ), fflush( stdout );
	b.Refit( 0, true );
}

// OptExRebuildSubtrees: rebuild the subtrees that carry the most cost from scratch. The root
// box of a subtree is the union of its references and cannot change, so a rebuild is a purely
// local move: nothing above it is touched, and the comparison is exact.
void Optimizer::OptExRebuildSubtrees( const Settings& s, State& st, float T0 )
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
		// the best attempt whether or not it beat the original, so the annealed branch below has
		// something to offer: without it a pass at temperature would only ever see its own losers.
		float bestAny = BVH_FAR;
		uint32_t bestAnyAttempt = (uint32_t)-1;
		for (uint32_t attempt = 0; attempt < tinybvh::tinybvh_max( 1u, s.rebuildAttempts ); attempt++)
		{
			uint32_t slotPtr = 0;
			float interior = 0;
			uint32_t seed = st.seed + (uint32_t)R * 2654435761u + attempt * 40503u + 1;
			memcpy( leaf, leaf0, leafCount * sizeof( uint32_t ) );
			OptExBuildTopology( leaf, leafCount, pool, slotPtr, rootParent,
				attempt == 0 ? float( 0 ) : float( s.rebuildJitter ), seed, interior, st );
			if (interior < best) best = interior, bestAttempt = attempt;
			if (interior < bestAny) bestAny = interior, bestAnyAttempt = attempt;
		}
		if (bestAttempt == (uint32_t)-1 && bestAnyAttempt != (uint32_t)-1)
		{
			// Candidates are ordered by the cost they carry, so the schedule is reversed here: the
			// costliest subtrees are rebuilt at zero temperature and only the cheap tail is annealed.
			const float T = AnnealTemp( T0, candCount - 1 - c, candCount );
			uint32_t aseed = st.seed + (uint32_t)R * 2654435761u + 29u;
			if (AnnealAccept( bestAny, original, T, aseed )) bestAttempt = bestAnyAttempt;
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
	// The reference boxes are a parallel array to primIdx, so a permutation of one is a
	// permutation of the other. Leaving them behind hands the next caller boxes belonging to
	// other primitives - silently, because every index still points at a real triangle.
	Ref* tmpRef = refs ? (Ref*)tinybvh::malloc64( b.idxCount * sizeof( Ref ) ) : 0;
	uint32_t stack[STACK_SIZE], stackPtr = 0, nodeIdx = 0, next = 0;
	while (1)
	{
		Node& node = b.bvhNode[nodeIdx];
		if (node.isLeaf())
		{
			const uint32_t first = next;
			for (uint32_t i = 0; i < node.triCount; i++)
			{
				if (tmpRef) tmpRef[next] = refs[node.firstTri + i];
				tmp[next++] = b.primIdx[node.firstTri + i];
			}
			node.firstTri = first;
			if (stackPtr == 0) break;
			nodeIdx = stack[--stackPtr];
			continue;
		}
		BVH_FATAL_ERROR_IF( stackPtr == STACK_SIZE, "BVH_Verbose::OptimizeEx, tree too deep." );
		stack[stackPtr++] = node.right, nodeIdx = node.left;
	}
	memcpy( b.primIdx, tmp, next * sizeof( uint32_t ) );
	if (tmpRef) memcpy( refs, tmpRef, next * sizeof( Ref ) ), tinybvh::free64( tmpRef );
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


// GateScore: what the run would deliver if it stopped here. The per-round tree is exploded -
// one reference per leaf - and is therefore not a tree anyone traverses, so the gate has to be
// shown the collapsed one. Collapsing in place and putting the round back afterwards is both
// cheaper and safer than deep-copying a BVH_Verbose, which borrows its primitive array from
// the builder and cannot be copied at all.
double Optimizer::GateScore( State& st, Node* scratchNode, uint32_t* scratchPrim )
{
	const uint32_t keepNodes = b.usedNodes, keepRefs = b.idxCount;
	const bool keepHoles = holes;
	memcpy( scratchNode, b.bvhNode, keepNodes * sizeof( Node ) );
	memcpy( scratchPrim, b.primIdx, keepRefs * sizeof( uint32_t ) );
	OptExCompactRefs();
	OptExCollapse( s.maxLeafPrims, st );
	OptExCompactRefs();
	const double score = gate->Score( b );
	memcpy( b.bvhNode, scratchNode, keepNodes * sizeof( Node ) );
	memcpy( b.primIdx, scratchPrim, keepRefs * sizeof( uint32_t ) );
	b.usedNodes = keepNodes, b.idxCount = keepRefs, holes = keepHoles;
	return score;
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

void Optimizer::Run( Ref* refBox, const uint32_t* vertIdx )
{
	BVH_FATAL_ERROR_IF( b.bvhNode == 0, "Optimizer::Run, no nodes." );
	refs = refBox;	// so the ref compaction can keep the boxes in step with the indices
	
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
	// A gated run keeps the tree as it stood at the start of the round so that a round which
	// does not pay can be undone whole. The measured gate needs a second pair as well: a place
	// to park the finished round while the tree is collapsed for scoring.
	const bool gated = s.epoGate || gate != 0;
	Node* snapNode = 0, * scratchNode = 0;
	uint32_t* snapPrim = 0, * scratchPrim = 0;
	if (gated)
	{
		snapNode = (Node*)tinybvh::malloc64( b.allocatedNodes * sizeof( Node ) );
		snapPrim = (uint32_t*)tinybvh::malloc64( maxRefsForSnapshot * sizeof( uint32_t ) );
	}
	if (gate)
	{
		scratchNode = (Node*)tinybvh::malloc64( b.allocatedNodes * sizeof( Node ) );
		scratchPrim = (uint32_t*)tinybvh::malloc64( maxRefsForSnapshot * sizeof( uint32_t ) );
	}
	// stage 0: one reference per leaf. This raises SAHCost; stage 5 takes it back.
	OptExComputeCosts( st );
	if (s.explodeLeaves) OptExExplode( refBox, st );
	RefreshOwnArea( st );
	OptExComputeCosts( st );
	float cost = b.SAHCost(), sahPart = 0;
	double gateCost = 0;
	if (gate) cost = (float)(gateCost = GateScore( st, scratchNode, scratchPrim ));
	else if (s.epoGate) cost = OptExCombined( vertIdx, st.primArea, sahPart );
	if (s.verbose)
	{
		if (s.explodeLeaves) printf( "  per-round SAH is measured on the exploded tree, one reference per leaf;\n"
			"  the final collapse takes that back. EPO is only shown when -epogate asks for it.\n" );
		if (gate) printf( "  rounds are judged by %s, not by SAH; the gate column is that score.\n"
			"  gate at entry: %.4f\n", gate->Unit(), gateCost );
		printf( "  %-5s %10s %10s %7s  %s\n", "round", "SAH", gate ? "gate" : "EPO", "sec", "stage" ), fflush( stdout );
	}
	float best = cost;
	uint32_t stale = 0;
	bool stochastic = false;
	for (uint32_t round = 0; round < s.rounds; round++)
	{
		const float before = cost;
		const Clock::time_point roundStart = Clock::now();
		if (holes) { b.Compact(), holes = false; }
		uint32_t snapNodes = 0, snapRefs = 0;
		if (gated)
		{
			snapNodes = b.usedNodes, snapRefs = b.idxCount;
			memcpy( snapNode, b.bvhNode, b.usedNodes * sizeof( Node ) );
			memcpy( snapPrim, b.primIdx, b.idxCount * sizeof( uint32_t ) );
		}
		RefreshOwnArea( st );
		if (s.reinsertIters)
		{
			if (s.fast) b.Optimize( s.reinsertIters, s.fastMu );
			else OptimizeSerial( s.reinsertIters, true, stochastic, st.seed,
				s.anneal > 0 ? s.anneal * b.bvhNode[0].SA() : 0.0f );
			Progress( round, roundStart, "reinsertion" ); // stage 1
		}
		// One temperature for every descent in the round, in the objective's own units. Each pass
		// cools to zero before it ends, so the round as a whole still finishes in a local minimum
		// and the gate above it sees an ordinary improvement.
		const float T0 = s.anneal > 0 ? s.anneal * b.bvhNode[0].SA() : 0.0f;
		for (uint32_t k = 0; k < s.treeletSweeps; k++)
			OptExTreeletSweep( s, st, k + 1 == s.treeletSweeps ? 0.0f : T0 ),
			Progress( round, roundStart, "treelets" ); // stage 2
		if (s.rebuildSubtrees) OptExRebuildSubtrees( s, st, T0 ), Progress( round, roundStart, "subtree rebuild" ); // stage 3
		if (s.unsplitRefs) { OptExUnsplit( st ); OptExCompactRefs(); Progress( round, roundStart, "unsplitting" ); } // stage 4
		if (gate)
		{
			const Clock::time_point g0 = Clock::now();
			cost = (float)(gateCost = GateScore( st, scratchNode, scratchPrim ));
			if (s.verbose) printf( "  %-5u %10.4f %10.4f %7.1f  gate\n", round, b.SAHCost(), gateCost,
				MilliSec( g0, Clock::now() ) / 1000.0 ), fflush( stdout );
		}
		else cost = s.epoGate ? OptExCombined( vertIdx, st.primArea, sahPart ) : b.SAHCost();
		if (gated)
		{
			if (cost < best) { best = cost, stale = 0; }
			else
			{
				// the round did not pay for itself under the gating metric: undo it whole.
				memcpy( b.bvhNode, snapNode, snapNodes * sizeof( Node ) );
				memcpy( b.primIdx, snapPrim, snapRefs * sizeof( uint32_t ) );
				b.usedNodes = snapNodes, b.idxCount = snapRefs, holes = false, cost = best;
				stale++;
				if (stale > s.gatePatience)
				{
					if (s.verbose) printf( "  round %u undone: %u rounds in a row bought nothing under %s\n",
						round, stale, gate ? gate->Unit() : "the EPO-weighted metric" ), fflush( stdout );
					break;
				}
				// Deterministic stages reproduce an undone round exactly, so the retry has to be
				// given something different to work with: a new seed, and a reinsertion stage that
				// walks the node list in random strides rather than in fixed ones.
				st.seed = st.seed * 1664525u + 1013904223u;
				stochastic = true;
				if (s.verbose) printf( "  round %u undone (%u of %u); retrying on a different sample\n",
					round, stale, s.gatePatience + 1 ), fflush( stdout );
				continue;
			}
			continue;	// patience governs when a gated run stops, not the relative gain below.
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
	if (scratchPrim) tinybvh::free64( scratchPrim );
	if (scratchNode) tinybvh::free64( scratchNode );
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

// HotChildFirst: order every pair of siblings so that the one rays actually enter is the left
// child. Left and right carry no meaning in a BVH2 - both children are tested, and every
// traversal here sorts them by distance at runtime - so this is a relabeling of the tree and
// not a change to it. What it changes is where each node ends up in memory.
//
// BVH::Compact lays the tree out depth-first, descending into left first and placing siblings
// adjacent; MBVH::BlockLayout does the same for the wide layouts, giving each child's subtree a
// contiguous range in child-slot order. Under either, the second child's subtree begins only
// after the whole of the first one. Putting the hot child first therefore lands the nodes rays
// actually reach next to their parent, instead of behind a cold sibling's entire subtree.
//
// Nothing is rebuilt and no bound moves, so the tree answers every query exactly as before.
static uint32_t HotChildFirst( tinybvh::BVH& bvh, std::vector<uint32_t>& visit, uint32_t& ties, uint32_t& pairs )
{
	// Walk from the root rather than over the node array. A BVH keeps node 1 as a hole and an
	// unreachable record can hold anything; reading one as a node and following its child index
	// corrupts the tree. That cost a segfault to find, which is why the check below exists.
	uint32_t swaps = 0;
	ties = 0, pairs = 0;
	if (bvh.usedNodes < 3 || bvh.bvhNode[0].isLeaf()) return 0;
	std::vector<uint32_t> stack;
	stack.push_back( 0 );
	while (!stack.empty())
	{
		const uint32_t n = stack.back(); stack.pop_back();
		tinybvh::BVH::BVHNode& node = bvh.bvhNode[n];
		if (node.isLeaf()) continue;
		const uint32_t l = node.leftFirst, r = l + 1;
		if (r >= bvh.usedNodes) continue;
		stack.push_back( l ), stack.push_back( r );
		pairs++;
		// Counts alone decide almost nothing: with a sample of this size against a tree of this
		// size, 95% of sibling pairs are visited an equal number of times, nearly all of them 0 or
		// 1, and the pass leaves the tree as it found it. Surface area breaks those ties and is the
		// right quantity to break them with: it is the probability a random ray enters the child at
		// all, which is what the counts were sampling in the first place.
		const bool tied = visit[l] == visit[r];
		if (tied) ties++;
		const bool hotter = tied
			? SurfaceArea( bvh.bvhNode[r].aabbMin, bvh.bvhNode[r].aabbMax ) >
			  SurfaceArea( bvh.bvhNode[l].aabbMin, bvh.bvhNode[l].aabbMax )
			: visit[r] > visit[l];
		if (!hotter) continue;
		// the records carry their own child indices, so exchanging the pair is the whole swap; the
		// visit counts move with them so nodes examined later still read the right ones.
		const tinybvh::BVH::BVHNode tmp = bvh.bvhNode[l];
		bvh.bvhNode[l] = bvh.bvhNode[r], bvh.bvhNode[r] = tmp;
		const uint32_t v = visit[l]; visit[l] = visit[r], visit[r] = v;
		swaps++;
	}
	return swaps;
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

// ----------------------------------------------------------------------------------------
// the measured round gate
// ----------------------------------------------------------------------------------------
//
// Between rounds the optimizer asks whether it is making progress, and the answer it has had
// so far is SAH - a model that assumes uniformly distributed, non-terminating rays and a flat
// memory system, none of which is true. The gate replaces the model with the measurement this
// tool already trusts enough to pick a winner with: node tests and primitive tests counted on
// a real ray set, over the collapsed tree, in the target's own layout, priced at c_trav and
// c_int. Two properties make it a better judge than wall clock: it is deterministic, so the
// same tree always scores the same and a round is undone for a reason rather than for noise,
// and it exists for all eight targets, three of which have no CPU traversal at all.
//
// -gatetime asks for wall clock instead, where there is one. It costs nothing extra to offer
// and it is the honest control for the paragraph above.
//
// The rays are the fit set, which is disjoint from the one the result is scored with. Gating
// on the scoring set would optimize the report rather than the tree.

template <typename T, int M> struct WideGate : public RoundGate
{
	const TargetInfo* info = 0;
	const RaySet* rays = 0;
	float cTrav = 1, cInt = 2;
	bool useTime = false;
	uint32_t timingRuns = 3;
	// Its own tree and its own primitive array. The leaf passes below rewrite both, and the
	// arrays they would otherwise rewrite are the ones the optimizer is still working in.
	mutable tinybvh::BVH work;
	const char* Unit() const override { return useTime ? "measured traversal time" : "measured traversal work"; }
	double Score( const tinybvh::BVH_Verbose& tree ) const override
	{
		work.ConvertFrom( tree );
		if (info->leafPrims) { if (info->fitLeaves) work.FitLeafs( info->leafPrims ); else work.SplitLeafs( info->leafPrims ); }
		tinybvh::MBVH<M> m;
		m.bvh.ReferenceFrom( work ), m.ownBVH = false;
		m.leafPrimLimit = info->collapseLimit, m.nodeProb = 0;
		T wide;
		const tinybvh::MBVH<M>* source = WideBuild( wide, m );
		double box, prim, nodes, pay;
		CountWideSteps( *source, *rays, info->quadLeaves, box, prim, nodes, pay );
		if (!useTime) return nodes * cTrav + pay * cInt;
		const double ns = WideTime( wide, *rays, timingRuns );
		return ns > 0 ? ns : nodes * cTrav + pay * cInt;
	}
};

// The two 2-wide targets have no collapse to judge, so the gate is the binary walk instead.
struct BinaryGate : public RoundGate
{
	const RaySet* rays = 0;
	float cTrav = 1, cInt = 1;
	bool rootTest = true, useTime = false;
	uint32_t timingRuns = 3;
	mutable tinybvh::BVH work;
	const char* Unit() const override { return useTime ? "measured traversal time" : "measured traversal work"; }
	double Score( const tinybvh::BVH_Verbose& tree ) const override
	{
		work.ConvertFrom( tree );
		double box, prim, nodes;
		CountBinarySteps( work, *rays, rootTest, box, prim, nodes );
		if (!useTime) return nodes * cTrav + prim * cInt;
		const double ns = WideTime( work, *rays, timingRuns );
		return ns > 0 ? ns : nodes * cTrav + prim * cInt;
	}
};

template <typename T, int M> static RoundGate* NewWideGate( const TargetInfo& info, const RaySet& rays,
	const float cTrav, const float cInt, const bool useTime, const uint32_t runs )
{
	WideGate<T, M>* g = new WideGate<T, M>();
	g->info = &info, g->rays = &rays, g->cTrav = cTrav, g->cInt = cInt, g->useTime = useTime, g->timingRuns = runs;
	return g;
}

static RoundGate* MakeGate( const uint32_t targetId, const TargetInfo& info, const RaySet& rays,
	const float cTrav, const float cInt, const bool useTime, const uint32_t runs )
{
	switch (targetId)
	{
	case TGT_BVH4CPU: return NewWideGate<tinybvh::BVH4_CPU, 4>( info, rays, cTrav, cInt, useTime, runs );
	case TGT_BVH8CPU: return NewWideGate<tinybvh::BVH8_CPU, 8>( info, rays, cTrav, cInt, useTime, runs );
	case TGT_BVH4GPU: return NewWideGate<tinybvh::BVH4_GPU, 4>( info, rays, cTrav, cInt, useTime, runs );
	case TGT_BVH8CWBVH: return NewWideGate<tinybvh::BVH8_CWBVH, 8>( info, rays, cTrav, cInt, useTime, runs );
	case TGT_BVH4AMDHW: return NewWideGate<tinybvh::BVH4_AMD_HW, 4>( info, rays, cTrav, cInt, false, runs );
	case TGT_CWBVH8AMDHW: return NewWideGate<tinybvh::CWBVH8_AMD_HW, 8>( info, rays, cTrav, cInt, false, runs );
	default:
		{
			BinaryGate* g = new BinaryGate();
			g->rays = &rays, g->cTrav = cTrav, g->cInt = cInt, g->useTime = useTime, g->timingRuns = runs;
			g->rootTest = targetId != TGT_BVHGPU;	// see CountBinarySteps on why BVH_GPU cannot
			return g;
		}
	}
}

// ApplyProfileRung: the effort settings for one rung of the ladder. Everything here is search
// effort; nothing here changes what is being minimized, which is the metric's job.
static void ApplyProfileRung( Optimizer::Settings& set, const uint32_t rung )
{
	if (rung == PROF_FAST)
	{
		set.rounds = 2, set.reinsertIters = 10, set.treeletSweeps = 1;
		set.treeletSize = 7, set.treeletSizeTop = 7, set.exactBottomSize = 0;
		set.rebuildSubtrees = 0, set.agglomSize = 0, set.anneal = 0;
	}
	else if (rung == PROF_HQ)
	{
		set.rounds = 8, set.reinsertIters = 25, set.treeletSweeps = 2;
		set.treeletSize = 7, set.treeletSizeTop = 10, set.exactBottomSize = 12;
		set.rebuildSubtrees = 0, set.agglomSize = 24, set.anneal = 0.02f;
	}
	else
	{
		set.rounds = 24, set.reinsertIters = 60, set.treeletSweeps = 3;
		set.treeletSize = 9, set.treeletSizeTop = 12, set.exactBottomSize = 14;
		set.rebuildSubtrees = 64, set.rebuildAttempts = 8, set.agglomSize = 48, set.anneal = 0.05f;
	}
}

// ----------------------------------------------------------------------------------------
// cost model calibration
// ----------------------------------------------------------------------------------------
//
// c_trav and c_int are the two numbers every SAH decision in this tool is made with, and they
// are guesses: 1 and 2, the same on every machine, in every layout and for every scene. They
// are also a claim that can be checked, because a traversal is nothing but node tests and
// primitive tests and this tool counts both. Build a spread of trees over the mesh, encode
// each in the target layout, count its tests and time it, and the pair that explains the
// timings is a least squares fit away.
//
// The spread has to come from somewhere. Two trees that differ only in quality move both
// counts the same way and say nothing about their ratio, which is the only thing a cost model
// is: four builders of deliberately different quality and a range of leaf budgets move them
// apart, and that is what makes the system solvable. The fit is reported with the spread it
// had and the r-squared it reached, and it is refused rather than used when either is poor -
// a badly conditioned fit is worse than the guess it would replace.
//
// What comes out is a ratio, so c_trav is pinned at 1 and only c_int moves. The intercept
// absorbs the per-ray overhead that belongs to neither count.

struct CostFit
{
	float cTrav = 1, cInt = 2;
	double r2 = 0;				// how much of the within-population variance the two counts explain
	double r2Pooled = 0;		// ..and how much they explain with the populations thrown together
	double corr = 0;			// correlation of the two counts over the sample: 1 means unsolvable
	double residual = 0;		// RMS of what the model leaves unexplained, in ns per ray
	double noise = 0;			// RMS of what the clock leaves uncertain, in ns per ray
	double r2WithSize = 0;		// the same fit with the tree's footprint allowed in as a third term
	double nsPerMB = 0;			// ..and what that term came out at
	double nsPerNode = 0, nsPerPrim = 0;
	uint32_t points = 0, groups = 0;
	bool usable = false;
	const char* why = "not attempted";
};

struct CalPoint { double n, p, ns, err, mb, leaves; const char* builder; uint32_t bucket; };

// The sample axis, and the two that were tried before it.
//
// Varying the leaf budget does nothing: every CPU layout repacks leaves to its slot size, so
// FitLeafs( 1 ) and FitLeafs( 4 ) hand it the same tree. Varying the builder does too much of
// the wrong thing: a worse builder changes depth and locality as well as the counts, and on
// Sponza an H-PLOC tree is slower than a binned one while performing fewer tests of both kinds,
// which no cost model in the two counts can represent. Either way the fit returns nonsense.
//
// Splitting is the axis that works. One builder throughout, so tree quality is held still, and
// the amount of primitive splitting varied instead: splits buy tighter boxes with more
// references, which moves node tests down while moving primitive tests up. That is the only
// knob found that moves the two counts in opposite directions, which is what separating their
// costs requires. minSpatialOverlap gates the SBVH's spatial splits; presplitFactor sizes the
// pre-split pass; together they span a primitive-per-node ratio of about 1.5x.
struct CalConfig { const char* name; bool spatial; float overlap; bool presplit; float factor; };
static const CalConfig g_calConfig[8] = {
	{ "nosplit", false, 1e-3f, false, 0.00f },
	{ "ovl.50",  true,  0.50f, false, 0.00f },
	{ "ovl.20",  true,  0.20f, false, 0.00f },
	{ "ovl.05",  true,  0.05f, false, 0.00f },
	{ "ovl1e-2", true,  1e-2f, false, 0.00f },
	{ "ovl1e-3", true,  1e-3f, false, 0.00f },
	{ "pre.15",  false, 1e-3f, true,  0.15f },
	{ "pre.60",  false, 1e-3f, true,  0.60f }
};

static void ApplyCalConfig( tinybvh::BVH& cal, const CalConfig& c )
{
	cal.settings.binnedBVH = true;
	cal.settings.useSpatialSplits = c.spatial;
	cal.settings.minSpatialOverlap = c.overlap;
	cal.settings.usePresplitting = c.presplit;
	cal.settings.presplitFactor = c.factor;
}

static uint32_t SplitRaysByWork( const tinybvh::BVH& ref, const RaySet& in, RaySet* out,
	const uint32_t maxBuckets, const uint32_t minRays )
{
	std::vector<float> hitT;
	hitT.reserve( in.Count() );
	std::vector<uint8_t> missed( in.Count(), 0 );
	std::vector<float> dist( in.Count(), 0.0f );
	for (size_t i = 0; i < in.Count(); i++)
	{
		tinybvh::Ray ray( in.O[i], in.D[i] );
		ref.Intersect( ray );
		if (ray.hit.t >= BVH_FAR) missed[i] = 1; else dist[i] = ray.hit.t, hitT.push_back( ray.hit.t );
	}
	if (hitT.empty()) return 0;
	std::sort( hitT.begin(), hitT.end() );
	// bucket 0 is the misses; the hits are split into equal-count distance bands, which orders
	// them by how much tree a ray had to cross before it was allowed to stop.
	const uint32_t bands = maxBuckets > 1 ? maxBuckets - 1 : 1;
	float edge[8];
	for (uint32_t k = 1; k < bands; k++) edge[k - 1] = hitT[(size_t)(hitT.size() * k / bands)];
	uint32_t used = 0;
	for (uint32_t b = 0; b < maxBuckets; b++) out[b].O.clear(), out[b].D.clear();
	for (size_t i = 0; i < in.Count(); i++)
	{
		uint32_t b = 0;
		if (!missed[i]) { b = 1; while (b - 1 < bands - 1 && dist[i] >= edge[b - 1]) b++; }
		out[b].O.push_back( in.O[i] ), out[b].D.push_back( in.D[i] );
	}
	// drop buckets too small to time without measuring the clock instead of the tree.
	for (uint32_t b = 0; b < maxBuckets; b++) if (out[b].Count() >= minRays)
	{
		if (b != used) out[used].O.swap( out[b].O ), out[used].D.swap( out[b].D );
		used++;
	}
	return used;
}

// The fit, and the correction the first version of it needed. Regressing time on the two
// counts across the whole sample finds that a bucket of short, coherent rays is much faster
// per ray than a bucket of long ones at the same counts: where the tests land matters as much
// as how many there are, and bucket membership is therefore a term in its own right. Left in,
// it is attributed to the counts and poisons both coefficients.
//
// Removing it costs nothing: subtract each bucket's own means from its points and the per-
// bucket term is gone exactly, leaving the costs identified by how time moves between trees
// measured on the same rays - which is the comparison the tool actually makes when it ranks
// candidates. The slopes are then a two by two solve, and the difference between the within-
// bucket and pooled r-squared is a free measurement of how much of traversal time the counts
// do not explain at all.
static bool Solve3x3( double A[3][3], double rhs[3], double out[3] )
{
	for (int32_t c = 0; c < 3; c++)
	{
		int32_t piv = c;
		for (int32_t r = c + 1; r < 3; r++) if (fabs( A[r][c] ) > fabs( A[piv][c] )) piv = r;
		if (fabs( A[piv][c] ) < 1e-12) return false;
		if (piv != c)
		{
			for (int32_t k = 0; k < 3; k++) { const double t = A[c][k]; A[c][k] = A[piv][k], A[piv][k] = t; }
			const double t = rhs[c]; rhs[c] = rhs[piv], rhs[piv] = t;
		}
		for (int32_t r = c + 1; r < 3; r++)
		{
			const double f = A[r][c] / A[c][c];
			for (int32_t k = c; k < 3; k++) A[r][k] -= f * A[c][k];
			rhs[r] -= f * rhs[c];
		}
	}
	for (int32_t r = 2; r >= 0; r--)
	{
		double v = rhs[r];
		for (int32_t k = r + 1; k < 3; k++) v -= A[r][k] * out[k];
		out[r] = v / A[r][r];
	}
	return true;
}

// Acceptance is against the machine rather than against a constant. Every point is measured
// twice, so the sample carries its own error bar, and the question is not whether the model
// fits well but whether it fits as well as the clock allows: a residual at the noise floor is
// the best any model of these two counts could do, and a residual far above it says the counts
// are not what the time is going into. A fixed r-squared threshold would have answered a
// different question on every machine.
static CostFit SolveCostFit( const std::vector<CalPoint>& pt, const uint32_t buckets )
{
	CostFit fit;
	fit.points = (uint32_t)pt.size(), fit.groups = buckets;
	if (pt.size() < 8 || buckets == 0) { fit.why = "too few usable samples"; return fit; }
	std::vector<double> mN( buckets, 0 ), mP( buckets, 0 ), mT( buckets, 0 );
	std::vector<double> count( buckets, 0 );
	for (size_t i = 0; i < pt.size(); i++)
	{
		const CalPoint& q = pt[i];
		mN[q.bucket] += q.n, mP[q.bucket] += q.p, mT[q.bucket] += q.ns, count[q.bucket] += 1;
	}
	for (uint32_t b = 0; b < buckets; b++) if (count[b] > 0) mN[b] /= count[b], mP[b] /= count[b], mT[b] /= count[b];
	double Snn = 0, Snp = 0, Spp = 0, Rn = 0, Rp = 0, Stt = 0;
	for (size_t i = 0; i < pt.size(); i++)
	{
		const CalPoint& q = pt[i];
		const double dn = q.n - mN[q.bucket], dp = q.p - mP[q.bucket], dt = q.ns - mT[q.bucket];
		Snn += dn * dn, Snp += dn * dp, Spp += dp * dp;
		Rn += dn * dt, Rp += dp * dt, Stt += dt * dt;
	}
	const double det = Snn * Spp - Snp * Snp;
	fit.corr = (Snn > 0 && Spp > 0) ? Snp / sqrt( Snn * Spp ) : 1.0;
	if (!(fabs( det ) > 1e-12) || !(Stt > 0)) { fit.why = "the sample carries no usable variation"; return fit; }
	const double a = (Spp * Rn - Snp * Rp) / det, b = (Snn * Rp - Snp * Rn) / det;
	fit.nsPerNode = a, fit.nsPerPrim = b;
	double ssr = 0;
	for (size_t i = 0; i < pt.size(); i++)
	{
		const CalPoint& q = pt[i];
		const double dn = q.n - mN[q.bucket], dp = q.p - mP[q.bucket], dt = q.ns - mT[q.bucket];
		const double e = dt - (a * dn + b * dp);
		ssr += e * e;
	}
	fit.r2 = 1.0 - ssr / Stt;
	fit.residual = sqrt( ssr / (double)pt.size() );
	{
		// two independent estimates of the same quantity differ by sqrt(2) times the error of
		// one; their mean, which is what was fitted, carries half of that.
		double e2 = 0;
		for (size_t i = 0; i < pt.size(); i++) e2 += pt[i].err * pt[i].err;
		fit.noise = 0.5 * sqrt( e2 / (double)pt.size() );
	}
	// the same model with the buckets pooled, reported only as the diagnostic it is.
	{
		double gn = 0, gp = 0, gt = 0, n = (double)pt.size();
		for (size_t i = 0; i < pt.size(); i++) gn += pt[i].n, gp += pt[i].p, gt += pt[i].ns;
		gn /= n, gp /= n, gt /= n;
		double sst = 0, res = 0;
		for (size_t i = 0; i < pt.size(); i++)
		{
			const double pred = a * (pt[i].n - gn) + b * (pt[i].p - gp);
			const double dt = pt[i].ns - gt;
			sst += dt * dt, res += (dt - pred) * (dt - pred);
		}
		fit.r2Pooled = sst > 0 ? 1.0 - res / sst : 0;
	}
	// The diagnostic. If the counts leave something unexplained, the first suspect is the one
	// neither of them mentions: how much memory the tree occupies. Same within-bucket centering,
	// one more column. This never sets a cost - c_trav and c_int have nowhere to put it - it is
	// here to say whether the two-term model is the wrong model or just a noisy one.
	{
		std::vector<double> mM( buckets, 0 );
		for (size_t i = 0; i < pt.size(); i++) mM[pt[i].bucket] += pt[i].mb;
		for (uint32_t g = 0; g < buckets; g++) if (count[g] > 0) mM[g] /= count[g];
		double A[3][3] = {}, rhs[3] = {}, out[3] = {};
		for (size_t i = 0; i < pt.size(); i++)
		{
			const CalPoint& q = pt[i];
			const double dn = q.n - mN[q.bucket], dp = q.p - mP[q.bucket];
			const double dm = q.mb - mM[q.bucket], dt = q.ns - mT[q.bucket];
			A[0][0] += dn * dn, A[0][1] += dn * dp, A[0][2] += dn * dm;
			A[1][1] += dp * dp, A[1][2] += dp * dm;
			A[2][2] += dm * dm;
			rhs[0] += dn * dt, rhs[1] += dp * dt, rhs[2] += dm * dt;
		}
		A[1][0] = A[0][1], A[2][0] = A[0][2], A[2][1] = A[1][2];
		if (Solve3x3( A, rhs, out ))
		{
			double res = 0;
			for (size_t i = 0; i < pt.size(); i++)
			{
				const CalPoint& q = pt[i];
				const double dn = q.n - mN[q.bucket], dp = q.p - mP[q.bucket];
				const double dm = q.mb - mM[q.bucket], dt = q.ns - mT[q.bucket];
				const double e = dt - (out[0] * dn + out[1] * dp + out[2] * dm);
				res += e * e;
			}
			fit.r2WithSize = 1.0 - res / Stt, fit.nsPerMB = out[2];
		}
	}
	if (fabs( fit.corr ) > 0.98) { fit.why = "the two counts move together; the sample cannot separate them"; return fit; }
	if (!(a > 0) || !(b > 0))
	{
		// A layout that pads every leaf to a full slot has prim == slots * leaves exactly, so the
		// primitive column carries nothing the leaf column does not, and its coefficient is free to
		// absorb whatever the node column could not. Name that case rather than reporting a
		// negative cost per primitive as though it were a measurement.
		bool padded = true;
		for (size_t i = 0; i < pt.size() && padded; i++)
			if (pt[i].leaves > 0 && fabs( pt[i].p / pt[i].leaves - pt[0].p / pt[0].leaves ) > 1e-3) padded = false;
		fit.why = (padded && b <= 0) ? "every leaf is padded to a full slot here, so c_int is not identifiable"
			: "a coefficient came out negative";
		return fit;
	}
	double meanNs = 0;
	for (size_t i = 0; i < pt.size(); i++) meanNs += pt[i].ns;
	meanNs /= (double)pt.size();
	if (fit.noise > 0.10 * meanNs) { fit.why = "the clock on this machine is too unsteady to calibrate against"; return fit; }
	// Two ways to pass. Either the model is as good as the clock allows, or it is imperfect but
	// explains almost all of the variance anyway - at which point the ratio is pinned even though
	// something outside the two counts is still moving the time. Refusing the second case would
	// throw away a measurement that repeats to within a few percent across runs.
	const double allowed = fit.noise * 2.0 > meanNs * 0.02 ? fit.noise * 2.0 : meanNs * 0.02;
	if (fit.residual > allowed && fit.r2 < 0.90)
	{
		fit.why = "what the counts leave unexplained is well above the measurement error";
		return fit;
	}
	double ratio = b / a;
	if (ratio < 0.05) ratio = 0.05; else if (ratio > 50.0) ratio = 50.0;
	fit.cTrav = 1.0f, fit.cInt = (float)ratio;
	fit.usable = true, fit.why = "ok";
	return fit;
}

static void ReportCostFit( const CostFit& fit, const char* layout )
{
	printf( "  fit on %s: %u samples over %u ray populations, count correlation %+.3f\n",
		layout, fit.points, fit.groups, fit.corr );
	printf( "           %.4f ns per node test, %.4f ns per primitive test\n", fit.nsPerNode, fit.nsPerPrim );
	printf( "           r2 %.3f within a population, %.3f with them pooled\n", fit.r2, fit.r2Pooled );
	printf( "           residual %.1f ns per ray against a measurement error of %.1f ns\n", fit.residual, fit.noise );
	printf( "           r2 %.3f if the tree's footprint is allowed in as well, at %+.2f ns per MB\n",
		fit.r2WithSize, fit.nsPerMB );
	if (fit.usable) printf( "           c_trav 1, c_int %.3f (default is %g, %g)\n", fit.cInt,
		(double)C_TRAV, (double)C_INT );
	else printf( "           refused (%s); keeping c_trav 1, c_int 2\n", fit.why );
	fflush( stdout );
}

static const uint32_t CAL_BUCKETS = 4;
static const uint32_t CAL_MIN_RAYS = 4000;
static const uint32_t CAL_MAX_TREES = 8;

// Timing one tree to exhaustion and then the next attaches whatever the machine was doing at
// the time to that tree, and a drift of a few percent over a calibration pass is larger than
// the differences being measured. So every tree is built first and kept, and the measurements
// are then interleaved: each pass visits every tree and every ray bucket, starting at a
// different tree, and each pair keeps its best over all passes. Odd and even passes are kept
// apart, which costs nothing and yields two independent estimates of every point - the error
// bar that acceptance is judged against.
template <typename T> static void TimeInterleaved( const T* tree, const uint32_t trees,
	const RaySet* bucket, const uint32_t buckets, const uint32_t passes, double* estA, double* estB )
{
	for (uint32_t i = 0; i < trees * CAL_BUCKETS; i++) estA[i] = estB[i] = 1e30;
	for (uint32_t pass = 0; pass < passes * 2; pass++)
	{
		double* dst = (pass & 1) ? estB : estA;
		for (uint32_t k = 0; k < trees; k++)
		{
			const uint32_t ti = (k + pass) % trees;
			for (uint32_t bi = 0; bi < buckets; bi++)
			{
				const double ns = WideTime( tree[ti], bucket[bi], 1 );
				double& slot = dst[ti * CAL_BUCKETS + bi];
				if (ns > 0 && ns < slot) slot = ns;
			}
		}
	}
}

static void ReportBuckets( const RaySet* bucket, const uint32_t buckets )
{
	printf( "  ray buckets:" );
	for (uint32_t b = 0; b < buckets; b++) printf( " %u", (uint32_t)bucket[b].Count() );
	printf( "  (misses first, then hits by distance)\n" );
}

template <typename T, int M> static CostFit FitWideCostModel( const std::vector<bvhvec4>& tris,
	const TargetInfo& info, const RaySet& rays, const uint32_t runs, const bool verbose )
{
	RaySet bucket[CAL_BUCKETS];
	uint32_t buckets = 0;
	{
		tinybvh::BVH ref;
		ref.Build( tris.data(), (uint32_t)tris.size() / 3 );
		buckets = SplitRaysByWork( ref, rays, bucket, CAL_BUCKETS, CAL_MIN_RAYS );
	}
	if (buckets < 2) { CostFit f; f.why = "the ray set does not split into usable buckets"; return f; }
	if (verbose) ReportBuckets( bucket, buckets );
	const uint32_t trees = CAL_MAX_TREES;
	std::unique_ptr<tinybvh::BVH[]> calBuf( new tinybvh::BVH[CAL_MAX_TREES] );
	std::unique_ptr<tinybvh::MBVH<M>[]> keepBuf( new tinybvh::MBVH<M>[CAL_MAX_TREES] );
	std::unique_ptr<T[]> wideBuf( new T[CAL_MAX_TREES] );
	tinybvh::BVH* cal = calBuf.get();
	tinybvh::MBVH<M>* keep = keepBuf.get();
	T* wide = wideBuf.get();
	double n[CAL_MAX_TREES * CAL_BUCKETS], p[CAL_MAX_TREES * CAL_BUCKETS], lf[CAL_MAX_TREES * CAL_BUCKETS];
	double estA[CAL_MAX_TREES * CAL_BUCKETS], estB[CAL_MAX_TREES * CAL_BUCKETS], mb[CAL_MAX_TREES];
	for (uint32_t ti = 0; ti < trees; ti++)
	{
		ApplyCalConfig( cal[ti], g_calConfig[ti] );
		cal[ti].c_trav = 1, cal[ti].c_int = 2, cal[ti].lQuads = info.quadLeaves;
		cal[ti].Build( tris.data(), (uint32_t)tris.size() / 3 );
		if (info.leafPrims) cal[ti].FitLeafs( info.leafPrims );
		keep[ti].bvh.ReferenceFrom( cal[ti] ), keep[ti].ownBVH = false;
		keep[ti].leafPrimLimit = info.collapseLimit, keep[ti].nodeProb = 0;
		const tinybvh::MBVH<M>* source = WideBuild( wide[ti], keep[ti] );
		mb[ti] = WideBytes( wide[ti] ) / (1024 * 1024);
		for (uint32_t b = 0; b < buckets; b++)
		{
			double box, prim, nodes, pay;
			CountWideSteps( *source, bucket[b], info.quadLeaves, box, prim, nodes, pay );
			n[ti * CAL_BUCKETS + b] = nodes, p[ti * CAL_BUCKETS + b] = pay;
			lf[ti * CAL_BUCKETS + b] = info.quadLeaves ? pay * 0.25 : prim;
		}
	}
	if (!(WideTime( wide[0], bucket[0], 1 ) > 0)) { CostFit f; f.why = "this layout has no CPU traversal to time"; return f; }
	TimeInterleaved<T>( wide, trees, bucket, buckets, runs, estA, estB );
	std::vector<CalPoint> pt;
	if (verbose) printf( "  %-8s %7s %10s %10s %10s %10s %8s %7s\n",
		"config", "bucket", "node/ray", "leaf/ray", "prim/ray", "ns/ray", "+/-", "MB" );
	for (uint32_t ti = 0; ti < trees; ti++) for (uint32_t b = 0; b < buckets; b++)
	{
		const uint32_t i = ti * CAL_BUCKETS + b;
		CalPoint q;
		q.n = n[i], q.p = p[i], q.leaves = lf[i];
		q.ns = 0.5 * (estA[i] + estB[i]), q.err = fabs( estA[i] - estB[i] );
		q.mb = mb[ti], q.builder = g_calConfig[ti].name, q.bucket = b;
		pt.push_back( q );
		if (verbose) printf( "  %-8s %7u %10.2f %10.2f %10.2f %10.1f %8.1f %7.1f\n",
			g_calConfig[ti].name, b, q.n, q.leaves, q.p, q.ns, q.err, q.mb );
	}
	return SolveCostFit( pt, buckets );
}

static bool FitsGPULayout( tinybvh::BVH& bvh );	// defined with the BVH_GPU target below

static CostFit FitBinaryCostModel( const std::vector<bvhvec4>& tris, const bool gpuLayout,
	const RaySet& rays, const uint32_t runs, const bool verbose )
{
	RaySet bucket[CAL_BUCKETS];
	uint32_t buckets = 0;
	{
		tinybvh::BVH ref;
		ref.Build( tris.data(), (uint32_t)tris.size() / 3 );
		buckets = SplitRaysByWork( ref, rays, bucket, CAL_BUCKETS, CAL_MIN_RAYS );
	}
	if (buckets < 2) { CostFit f; f.why = "the ray set does not split into usable buckets"; return f; }
	if (verbose) ReportBuckets( bucket, buckets );
	const uint32_t trees = CAL_MAX_TREES;
	std::unique_ptr<tinybvh::BVH[]> calBuf( new tinybvh::BVH[CAL_MAX_TREES] );
	std::unique_ptr<tinybvh::BVH_GPU[]> gpuBuf( new tinybvh::BVH_GPU[CAL_MAX_TREES] );
	tinybvh::BVH* cal = calBuf.get();
	tinybvh::BVH_GPU* gpu = gpuBuf.get();
	double n[CAL_MAX_TREES * CAL_BUCKETS], p[CAL_MAX_TREES * CAL_BUCKETS];
	double estA[CAL_MAX_TREES * CAL_BUCKETS], estB[CAL_MAX_TREES * CAL_BUCKETS], mb[CAL_MAX_TREES];
	for (uint32_t ti = 0; ti < trees; ti++)
	{
		ApplyCalConfig( cal[ti], g_calConfig[ti] );
		cal[ti].c_trav = 1, cal[ti].c_int = 1;
		cal[ti].Build( tris.data(), (uint32_t)tris.size() / 3 );
		if (gpuLayout)
		{
			if (!FitsGPULayout( cal[ti] )) { CostFit f; f.why = "a calibration tree does not fit the layout"; return f; }
			gpu[ti].ConvertFrom( cal[ti], true );
		}
		mb[ti] = (double)(cal[ti].usedNodes * sizeof( tinybvh::BVH::BVHNode )
			+ cal[ti].idxCount * sizeof( uint32_t )) / (1024 * 1024);
		for (uint32_t b = 0; b < buckets; b++)
		{
			double box, prim, nodes;
			CountBinarySteps( cal[ti], bucket[b], !gpuLayout, box, prim, nodes );
			n[ti * CAL_BUCKETS + b] = nodes, p[ti * CAL_BUCKETS + b] = prim;
		}
	}
	if (gpuLayout) TimeInterleaved( gpu, trees, bucket, buckets, runs, estA, estB );
	else TimeInterleaved( cal, trees, bucket, buckets, runs, estA, estB );
	std::vector<CalPoint> pt;
	if (verbose) printf( "  %-8s %7s %10s %10s %10s %8s %7s\n",
		"config", "bucket", "node/ray", "prim/ray", "ns/ray", "+/-", "MB" );
	for (uint32_t ti = 0; ti < trees; ti++) for (uint32_t b = 0; b < buckets; b++)
	{
		const uint32_t i = ti * CAL_BUCKETS + b;
		CalPoint q;
		q.n = n[i], q.p = p[i], q.leaves = p[i];
		q.ns = 0.5 * (estA[i] + estB[i]), q.err = fabs( estA[i] - estB[i] );
		q.mb = mb[ti], q.builder = g_calConfig[ti].name, q.bucket = b;
		pt.push_back( q );
		if (verbose) printf( "  %-8s %7u %10.2f %10.2f %10.1f %8.1f %7.1f\n",
			g_calConfig[ti].name, b, q.n, q.p, q.ns, q.err, q.mb );
	}
	return SolveCostFit( pt, buckets );
}

static CostFit CalibrateCostModel( const uint32_t targetId, const TargetInfo& info,
	const std::vector<bvhvec4>& tris, const RaySet& rays, const uint32_t runs, const bool verbose )
{
	switch (targetId)
	{
	case TGT_BVH4CPU: return FitWideCostModel<tinybvh::BVH4_CPU, 4>( tris, info, rays, runs, verbose );
	case TGT_BVH8CPU: return FitWideCostModel<tinybvh::BVH8_CPU, 8>( tris, info, rays, runs, verbose );
	case TGT_BVH4GPU: return FitWideCostModel<tinybvh::BVH4_GPU, 4>( tris, info, rays, runs, verbose );
	case TGT_BVH8CWBVH: return FitWideCostModel<tinybvh::BVH8_CWBVH, 8>( tris, info, rays, runs, verbose );
	case TGT_BVH: return FitBinaryCostModel( tris, false, rays, runs, verbose );
	case TGT_BVHGPU: return FitBinaryCostModel( tris, true, rays, runs, verbose );
	default: { CostFit f; f.why = "this layout has no CPU traversal to time"; return f; }
	}
}

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

// ----------------------------------------------------------------------------------------
// subtree rebuild pre-pass
// ----------------------------------------------------------------------------------------
//
// A finished subtree is not the best tree over its own references. The fragments it holds were
// shaped by split planes chosen higher up, and the splits inside it were chosen by a greedy that
// had already committed to those. Rebuilding the subtree standalone lets both be re-decided
// against nothing but the box it has to fit in, which is fixed and therefore makes the exchange
// exactly local: the replacement's root box is clipped to the original's, so no node above it
// sees a different box and no cost above it changes.
//
// Measured on 370k triangles of San Miguel, over subtrees of 16 to 128 references: standalone,
// the restart alone is worth 0.20% of whole-tree cost, spatial splits add 0.19% for 1% more
// references, and sweeping the bin count adds a further 0.27%. Inside the pipeline the object
// split variant is worth almost nothing - bvhopt builds with keepSplitting, so the surrounding
// tree already has one reference per leaf and the leaf sizing freedom that produced most of that
// 0.20% does not exist - which is why only the spatial variant survives here. The bin sweep only
// exists when spatial splits are on: hqbvhbins is read in BuildHQ and nowhere in the binned
// object-split path.
//
// The search is exhaustive rather than sampled: every bin count in [minBins, maxBins], each with
// and without hqbvhoddeven, which gives odd levels one bin more. 50 builds per subtree at the
// defaults. Nothing here is cheap and nothing here is meant to be.
//
// Size matters more than any of that. At 20k references and up the greedy reproduces what it
// already built whatever the bin count, and 2 of 13 subtrees improved at all; the gain lives
// between 16 and a few hundred references, which is the band that treelets (7 to 10 leaves) and
// exact bottoms (14 references) cannot reach and the cost-carrying rebuild does not target.
//
// This runs before everything else in Optimizer::Run, on the tree as built.

struct PrePass
{
	bool on = false;			// -prepass
	uint32_t minRefs = 16;		// the band; see the note above on where the gain lives
	uint32_t maxRefs = 128;
	uint32_t minBins = 8;		// every bin count in [minBins, maxBins], each with and without
	uint32_t maxBins = 32;		// hqbvhoddeven, which gives odd levels one bin more
	uint32_t Configs() const { return (maxBins - minBins + 1) * 2; }
};

// Half the surface area of a box, clipped to [lo,hi]; zero if the clip is empty.
static float ClippedArea( const bvhvec3& bmin, const bvhvec3& bmax, const bvhvec3& lo, const bvhvec3& hi )
{
	const bvhvec3 a = tinybvh_max( bmin, lo ), b = tinybvh_min( bmax, hi ), e = b - a;
	if (e.x < 0 || e.y < 0 || e.z < 0) return 0;
	return e.x * e.y + e.y * e.z + e.z * e.x;
}

// Cost of a subtree with every box clipped to [lo,hi]. Unnormalized - the division by root area
// cancels when two subtrees over the same box are compared, and leaving it out lets costs add.
static double SubtreeCost( const tinybvh::BVH& bvh, const uint32_t root, const bvhvec3& lo, const bvhvec3& hi,
	uint32_t& refs, uint32_t& nodes )
{
	std::vector<uint32_t> stack;
	uint32_t node = root;
	double trav = 0, isect = 0;
	for (;;)
	{
		const tinybvh::BVH::BVHNode& n = bvh.bvhNode[node];
		const float a = ClippedArea( n.aabbMin, n.aabbMax, lo, hi );
		if (n.isLeaf()) isect += (double)a * n.triCount, refs += n.triCount;
		else
		{
			trav += a, nodes++;
			stack.push_back( (uint32_t)n.leftFirst + 1 ), stack.push_back( (uint32_t)n.leftFirst );
		}
		if (stack.empty()) break;
		node = stack.back(), stack.pop_back();
	}
	return trav * bvh.c_trav + isect * bvh.c_int;
}

static void CollectPrims( const tinybvh::BVH& bvh, const uint32_t root, std::vector<uint32_t>& out )
{
	std::vector<uint32_t> stack;
	uint32_t node = root;
	for (;;)
	{
		const tinybvh::BVH::BVHNode& n = bvh.bvhNode[node];
		if (n.isLeaf()) for (uint32_t i = 0; i < n.triCount; i++) out.push_back( bvh.primIdx[n.leftFirst + i] );
		else stack.push_back( (uint32_t)n.leftFirst + 1 ), stack.push_back( (uint32_t)n.leftFirst );
		if (stack.empty()) break;
		node = stack.back(), stack.pop_back();
	}
}

// One accepted replacement: the tree to splice in and the triangle id behind each of its prims.
struct Replacement
{
	tinybvh::BVH tree;
	std::vector<uint32_t> prim;
	uint32_t nodes = 0, refs = 0;
};

// Copy a replacement's subtree into the new node list, clipping every box to the box of the node
// it replaces, which only shrinks them and keeps the root box exactly equal to the original's.
static void EmitReplacement( const Replacement& r, const uint32_t src, const uint32_t dst,
	const bvhvec3& lo, const bvhvec3& hi, std::vector<tinybvh::BVH::BVHNode>& node, std::vector<uint32_t>& prim )
{
	std::vector<std::pair<uint32_t, uint32_t>> stack;
	stack.push_back( std::make_pair( src, dst ) );
	while (!stack.empty())
	{
		const uint32_t s = stack.back().first, d = stack.back().second;
		stack.pop_back();
		const tinybvh::BVH::BVHNode& in = r.tree.bvhNode[s];
		tinybvh::BVH::BVHNode& out = node[d];
		out.aabbMin = tinybvh_max( in.aabbMin, lo ), out.aabbMax = tinybvh_min( in.aabbMax, hi );
		if (in.isLeaf())
		{
			out.leftFirst = (uint32_t)prim.size(), out.triCount = in.triCount;
			for (uint32_t i = 0; i < in.triCount; i++) prim.push_back( r.prim[r.tree.primIdx[in.leftFirst + i]] );
		}
		else
		{
			const uint32_t slot = (uint32_t)node.size();
			node.resize( slot + 2 );
			node[d].leftFirst = slot, node[d].triCount = 0;
			stack.push_back( std::make_pair( (uint32_t)in.leftFirst, slot ) );
			stack.push_back( std::make_pair( (uint32_t)in.leftFirst + 1, slot + 1 ) );
		}
	}
}

// Returns the cost removed, as a share of the whole tree, or 0 if nothing was spliced.
// Geometry comes from the tree's own vertex slice rather than from a caller's array, so this
// serves both entry points: a mesh built with --target, and a .bvh loaded with its geometry.
static double RebuildSubtrees( tinybvh::BVH& bvh, const PrePass& pre )
{
	if (!bvh.hasOrderedNodes()) bvh.Compact();
	uint32_t wholeRefs = 0, wholeNodes = 0;
	const double whole = SubtreeCost( bvh, 0, bvh.bvhNode[0].aabbMin, bvh.bvhNode[0].aabbMax, wholeRefs, wholeNodes );
	const uint32_t total = (uint32_t)bvh.usedNodes;

	// Children sit above their parent only in a depth-first tree, which is what the descending
	// pass below relies on - and a Bonsai build is not one (BuildSIMD leaves orderedNodes false),
	// so its node list has parents above children and the pass would read counts that were never
	// written. Compacting first costs one pass and is what the optimizer does later anyway.
	// References owned per node.
	std::vector<uint32_t> owned( total, 0 );
	for (int32_t i = (int32_t)total - 1; i >= 0; i--)
	{
		const tinybvh::BVH::BVHNode& n = bvh.bvhNode[i];
		owned[i] = n.isLeaf() ? n.triCount : owned[n.leftFirst] + owned[n.leftFirst + 1];
	}
	// Candidates: the first node on each path whose reference count falls in the band. Disjoint by
	// construction, so their costs add and they can be replaced independently.
	std::vector<uint32_t> cand;
	{
		std::vector<uint32_t> stack;
		uint32_t node = 0;
		for (;;)
		{
			const tinybvh::BVH::BVHNode& n = bvh.bvhNode[node];
			if (!n.isLeaf())
			{
				if (owned[node] <= pre.maxRefs && owned[node] >= pre.minRefs) cand.push_back( node );
				else if (owned[node] > pre.maxRefs)
					stack.push_back( (uint32_t)n.leftFirst + 1 ), stack.push_back( (uint32_t)n.leftFirst );
			}
			if (stack.empty()) break;
			node = stack.back(), stack.pop_back();
		}
	}
	std::vector<Replacement> repl;
	std::vector<int32_t> replOf( total, -1 );
	double gained = 0;
	int64_t refDelta = 0, nodeDelta = 0;
	uint64_t winBins = 0, winOddEven = 0;	// which configuration actually won, over all accepted
	for (size_t c = 0; c < cand.size(); c++)
	{
		const tinybvh::BVH::BVHNode& nd = bvh.bvhNode[cand[c]];
		const bvhvec3 lo = nd.aabbMin, hi = nd.aabbMax;
		uint32_t refs = 0, nodes = 0;
		const double was = SubtreeCost( bvh, cand[c], lo, hi, refs, nodes );
		std::vector<uint32_t> prim;
		CollectPrims( bvh, cand[c], prim );
		std::sort( prim.begin(), prim.end() );
		prim.erase( std::unique( prim.begin(), prim.end() ), prim.end() );
		std::vector<bvhvec4> sub( prim.size() * 3 );
		for (size_t t = 0; t < prim.size(); t++) for (uint32_t k = 0; k < 3; k++)
			sub[t * 3 + k] = bvh.verts[(size_t)prim[t] * 3 + k];
		Replacement best;
		double bestCost = was;
		uint32_t bestBins = 0;
		bool bestOddEven = false;
		for (uint32_t b = pre.minBins; b <= pre.maxBins; b++) for (uint32_t oe = 0; oe < 2; oe++)
		{
			Replacement cur;
			// Inherit the surrounding build's settings before overriding the ones this pass is
			// about. Leaf sizing in particular: bvhopt builds with keepSplitting, so the tree
			// around the subtree has one reference per leaf, and a rebuild at default leaf sizing
			// is priced against it as a huge win that the optimizer's own explode step then
			// throws away. Comparing two trees built under different rules measures the rules.
			cur.tree.settings = bvh.settings;
			cur.tree.settings.useSpatialSplits = true;
			cur.tree.settings.binnedBVH = true;
			cur.tree.hqbvhbins = b, cur.tree.hqbvhoddeven = oe != 0;
			cur.tree.c_trav = bvh.c_trav, cur.tree.c_int = bvh.c_int;
			cur.tree.Build( sub.data(), (uint32_t)prim.size() );
			uint32_t rrefs = 0, rnodes = 0;
			const double now = SubtreeCost( cur.tree, 0, lo, hi, rrefs, rnodes );
			if (now >= bestCost) continue;
			cur.prim = prim, cur.nodes = rnodes, cur.refs = rrefs;
			best = std::move( cur ), bestCost = now, bestBins = b, bestOddEven = oe != 0;
		}
		if (bestCost >= was) continue;
		// The new arrays are written into the ones the BVH already owns, so a replacement is taken
		// only while the running totals stay inside what was allocated for them. Object-split
		// rebuilds never grow the reference list, so this only ever binds on the spatial variant.
		if ((int64_t)wholeRefs + refDelta + (int64_t)best.refs - (int64_t)refs > (int64_t)bvh.idxCount) continue;
		if ((int64_t)total + nodeDelta + 2 * ((int64_t)best.nodes - (int64_t)nodes) > (int64_t)bvh.allocatedNodes) continue;
		refDelta += (int64_t)best.refs - (int64_t)refs;
		nodeDelta += 2 * ((int64_t)best.nodes - (int64_t)nodes);
		gained += was - bestCost;
		winBins += bestBins, winOddEven += bestOddEven ? 1 : 0;
		replOf[cand[c]] = (int32_t)repl.size();
		repl.push_back( std::move( best ) );
	}
	if (repl.empty()) return 0;

	// One depth-first pass writes the whole tree afresh: untouched nodes copied, replaced ones
	// emitted from their rebuild. The result is compacted and depth-first by construction.
	std::vector<tinybvh::BVH::BVHNode> node( 1 );
	std::vector<uint32_t> prim;
	prim.reserve( wholeRefs + (size_t)(refDelta > 0 ? refDelta : 0) );
	std::vector<std::pair<uint32_t, uint32_t>> stack;
	stack.push_back( std::make_pair( 0u, 0u ) );
	while (!stack.empty())
	{
		const uint32_t src = stack.back().first, dst = stack.back().second;
		stack.pop_back();
		const tinybvh::BVH::BVHNode& in = bvh.bvhNode[src];
		if (replOf[src] >= 0)
		{
			EmitReplacement( repl[replOf[src]], 0, dst, in.aabbMin, in.aabbMax, node, prim );
			continue;
		}
		node[dst].aabbMin = in.aabbMin, node[dst].aabbMax = in.aabbMax;
		if (in.isLeaf())
		{
			node[dst].leftFirst = (uint32_t)prim.size(), node[dst].triCount = in.triCount;
			for (uint32_t i = 0; i < in.triCount; i++) prim.push_back( bvh.primIdx[in.leftFirst + i] );
		}
		else
		{
			const uint32_t slot = (uint32_t)node.size();
			node.resize( slot + 2 );
			node[dst].leftFirst = slot, node[dst].triCount = 0;
			stack.push_back( std::make_pair( (uint32_t)in.leftFirst, slot ) );
			stack.push_back( std::make_pair( (uint32_t)in.leftFirst + 1, slot + 1 ) );
		}
	}
	memcpy( bvh.bvhNode, node.data(), node.size() * sizeof( tinybvh::BVH::BVHNode ) );
	memcpy( bvh.primIdx, prim.data(), prim.size() * sizeof( uint32_t ) );
	bvh.usedNodes = (uint32_t)node.size(), bvh.idxCount = (uint32_t)prim.size();
	// The rewrite is depth-first and gap-free by construction, so no hole or ordering flag needs
	// clearing; refittability is the builder's to decide and a spatial-split rebuild only adds
	// the duplicate references an SBVH build would already have produced inside this box.
	printf( "prepass: %u of %u subtrees (%u..%u refs) rebuilt from %u configurations, SAH -%.2f%%, refs %+.2f%%\n",
		(uint32_t)repl.size(), (uint32_t)cand.size(), pre.minRefs, pre.maxRefs, pre.Configs(),
		100.0 * gained / whole, 100.0 * refDelta / (double)wholeRefs );
	printf( "         winning config: %.1f bins mean, odd/even in %.0f%% of them\n",
		(double)winBins / (double)repl.size(), 100.0 * (double)winOddEven / (double)repl.size() );
	return gained / whole;
}

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
	PrePass pre;				// -prepass: rebuild small subtrees before optimizing
	bool audit = false;			// -audit: report what is wrong with the mesh
	bool fix = false;			// -fix: and keep it out of the tree
	bool gate = false;			// -gate: judge each round by measured traversal, not by SAH
	bool gateTime = false;		// -gatetime: and measure it with the clock rather than by counting
	bool fitCost = false;		// -fitcost: measure c_trav and c_int rather than assume 1 and 2
	bool fitDetail = false;		// -fitdetail: show every calibration sample, not just the verdict
	bool calibrateOnly = false;	// -calibrate: fit every target and report, build nothing
	bool hotOrder = false;		// -hotorder: lay the tree out with the visited child first
	uint32_t profile = PROF_HQ;	// -profile fast|hq|extreme: how hard to look
	uint32_t metric = METRIC_SAH;	// -metric sah|epo|raycost: what to minimize
	bool Any() const { for (uint32_t t = 0; t < TGT_COUNT; t++) if (target[t]) return true; return false; }
};

// ----------------------------------------------------------------------------------------
// geometry audit
// ----------------------------------------------------------------------------------------
//
// Three defects make a region of a mesh cost far more to traverse than its triangle count
// suggests, and no builder can do anything about any of them:
//
//   Exact duplicates. Coincident triangles have coincident centroids and coincident boxes, so
//   no object split separates them - every candidate plane puts all of them on one side, and
//   the builder falls through to halving a range, which gives two children with the parent's
//   box and no SAH improvement at all, level after level. Spatial splits do not help either:
//   each copy spans the whole box, so any plane cuts all of them and both sides keep all of
//   them. Every ray entering the region then tests every copy. This is normally an authoring
//   accident - an object instanced several times at one transform and flattened on export.
//
//   Zero-area triangles. They carry a box but can never be hit, so they are pure cost.
//
//   Non-finite vertices. One NaN or infinity in one vertex propagates into the root box and
//   ruins the whole tree, not just its own neighbourhood.
//
// The audit is -audit and reports only; acting on it is -fix. Both are off by default: the
// audit costs a sort over the whole mesh, and dropping a triangle changes
// what a renderer can report as hit: the surviving copy wins, and if the copies carried
// different materials the choice between them was arbitrary to begin with, since they occupy
// the same space. Nothing is renumbered either way - see GeometryFilter.

struct GeometryIssues
{
	uint32_t nonFinite = 0;		// triangles with a NaN or an infinity in a vertex
	uint32_t degenerate = 0;	// zero area, hence unhittable
	uint32_t redundant = 0;		// copies of a triangle beyond the first
	uint32_t worstCopies = 0;	// copies of the single most duplicated triangle
	float worstAt[3] = { 0, 0, 0 };
	bool clean() const { return nonFinite + degenerate + redundant == 0; }
};

// Canonical form of a triangle: the rotation starting at the lexicographically smallest vertex,
// compared as raw bytes. Rotation only, never reflection - a triangle and its flipped copy
// occupy the same space but face opposite ways, and a renderer that culls backfaces can tell
// them apart, so they are not treated as the same triangle here.
static void CanonicalTri( const bvhvec4* tri, float* out )
{
	uint32_t first = 0;
	for (uint32_t i = 1; i < 3; i++)
	{
		const float* a = (const float*)&tri[i];
		const float* b = (const float*)&tri[first];
		if (a[0] < b[0] || (a[0] == b[0] && (a[1] < b[1] || (a[1] == b[1] && a[2] < b[2])))) first = i;
	}
	for (uint32_t i = 0; i < 3; i++)
	{
		const float* v = (const float*)&tri[(first + i) % 3];
		out[i * 3] = v[0], out[i * 3 + 1] = v[1], out[i * 3 + 2] = v[2];
	}
}

static uint64_t HashTri( const float* c )
{
	uint64_t h = 0xcbf29ce484222325ull;
	const uint32_t* w = (const uint32_t*)c;
	for (uint32_t i = 0; i < 9; i++) h = (h ^ w[i]) * 0x100000001b3ull;
	return h;
}

// Which triangles the BVH should index, and the original id of each one it does. The filter is
// applied by building over an index list rather than by compacting the mesh, and the primitive
// indices are mapped back to original triangle ids immediately afterwards - so the finished tree
// is an ordinary tree over the original mesh that simply never references what was dropped.
// Triangle ids are what a renderer looks materials up by, and they do not move.
struct GeometryFilter
{
	std::vector<uint32_t> keep;		// original triangle id, per kept triangle
	std::vector<uint32_t> vertIdx;	// three vertex indices per kept triangle, into the original array
	bool active() const { return !keep.empty(); }
};

// One pass for the pointwise defects, then a sort to group identical triangles. The sort is on a
// hash, so equal hashes are compared as bytes before anything is called a duplicate.
static GeometryIssues AuditGeometry( const std::vector<bvhvec4>& tris, const bool fix, GeometryFilter& filter )
{
	GeometryIssues issue;
	const uint32_t n = (uint32_t)(tris.size() / 3);
	std::vector<uint8_t> drop( n, 0 );
	for (uint32_t t = 0; t < n; t++)
	{
		const bvhvec4* v = &tris[(size_t)t * 3];
		bool finite = true;
		for (uint32_t i = 0; i < 3; i++) for (uint32_t a = 0; a < 3; a++)
		{
			const float f = ((const float*)&v[i])[a];
			if (f != f || f > 3.0e38f || f < -3.0e38f) finite = false;
		}
		if (!finite) { issue.nonFinite++, drop[t] = 1; continue; }
		const bvhvec3 e1 = bvhvec3( v[1] ) - bvhvec3( v[0] ), e2 = bvhvec3( v[2] ) - bvhvec3( v[0] );
		if (tinybvh::tinybvh_length( tinybvh::tinybvh_cross( e1, e2 ) ) == 0) issue.degenerate++, drop[t] = 1;
	}
	std::vector<std::pair<uint64_t, uint32_t>> byHash( n );
	std::vector<float> canon( (size_t)n * 9 );
	for (uint32_t t = 0; t < n; t++)
	{
		CanonicalTri( &tris[(size_t)t * 3], &canon[(size_t)t * 9] );
		byHash[t] = std::make_pair( HashTri( &canon[(size_t)t * 9] ), t );
	}
	std::sort( byHash.begin(), byHash.end() );
	for (uint32_t i = 0; i < n; )
	{
		uint32_t j = i + 1;
		while (j < n && byHash[j].first == byHash[i].first) j++;
		// One hash bucket. Compare members as bytes, since a hash collision between different
		// triangles would otherwise delete real geometry.
		for (uint32_t a = i; a < j; a++)
		{
			if (drop[byHash[a].second]) continue;
			uint32_t copies = 1;
			for (uint32_t b = a + 1; b < j; b++)
			{
				if (drop[byHash[b].second]) continue;
				if (memcmp( &canon[(size_t)byHash[a].second * 9], &canon[(size_t)byHash[b].second * 9], 9 * sizeof( float ) ) != 0) continue;
				copies++, issue.redundant++, drop[byHash[b].second] = 1;
			}
			if (copies > issue.worstCopies)
			{
				issue.worstCopies = copies;
				const bvhvec4* v = &tris[(size_t)byHash[a].second * 3];
				for (uint32_t k = 0; k < 3; k++)
					issue.worstAt[k] = (((const float*)&v[0])[k] + ((const float*)&v[1])[k] + ((const float*)&v[2])[k]) / 3.0f;
			}
		}
		i = j;
	}
	if (fix && !issue.clean())
	{
		filter.keep.reserve( n ), filter.vertIdx.reserve( (size_t)n * 3 );
		for (uint32_t t = 0; t < n; t++)
		{
			if (drop[t]) continue;
			filter.keep.push_back( t );
			for (uint32_t k = 0; k < 3; k++) filter.vertIdx.push_back( t * 3 + k );
		}
	}
	return issue;
}

static void ReportGeometry( const GeometryIssues& issue, const uint32_t n, const bool fix )
{
	if (issue.clean())
	{
		printf( "audit  : no duplicate, degenerate or non-finite triangles.\n" );
		return;
	}
	printf( "audit  :" );
	if (issue.nonFinite) printf( " %u non-finite,", issue.nonFinite );
	if (issue.degenerate) printf( " %u zero-area,", issue.degenerate );
	if (issue.redundant) printf( " %u redundant copies,", issue.redundant );
	printf( " of %u triangles (%.2f%%)\n", n, 100.0 * (issue.nonFinite + issue.degenerate + issue.redundant) / n );
	if (issue.worstCopies > 1) printf( "         worst: one triangle present %u times, around %g %g %g\n",
		issue.worstCopies, issue.worstAt[0], issue.worstAt[1], issue.worstAt[2] );
	if (fix) printf( "         dropped from the tree; triangle ids are unchanged.\n" );
	else printf( "         reported only; -fix drops them from the tree without renumbering.\n" );
}

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
	uint32_t rung;			// the profile rung whose tree was kept; see the ladder in PrepareCandidate
	double seconds;			// build plus optimize, without the measurements themselves
};

// MetricRank: which of two finished candidates the user asked for. Ray cost is the measured
// walk and is already what the tables below report; sah and epo are properties of the binary
// tree and were recorded when the candidate was built. Ranking by measured cost under -metric
// sah would quietly hand back a different tree than the one the optimizer spent its time on.
static float MetricRank( const CandidateStats& st, const uint32_t metric, const double measured )
{
	if (metric == METRIC_SAH) return st.sahPost;
	if (metric == METRIC_EPO) return st.epoPost;
	return (float)measured;
}


// Build one candidate binary tree and run the optimizer over it, with the leaf budget of the
// layouts it will feed. The layout's own leaf pass is run here as well, so that the node list
// is final before the visit field is measured on it: the collapse must see the same node
// indices the field was built on.
static void PrepareCandidate( tinybvh::BVH& bvh, const std::vector<bvhvec4>& tris, const uint32_t candidate,
	Optimizer::Settings set, const TargetInfo& info, CandidateStats& st, const GeometryFilter& filter,
	const PrePass& pre, const float cTrav, const float cInt, const RoundGate* gate,
	const uint32_t profile, const uint32_t metric )
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
	bvh.hqLeafPrims = capped ? info.leafPrims : 0, bvh.lQuads = info.quadLeaves;
	bvh.c_trav = cTrav, bvh.c_int = cInt;
	// RecoverRef below rebuilds each reference box by clipping its triangle to the box of the leaf
	// holding it, so smaller leaves recover the spatial splits more exactly - and for a leaf of one
	// they recover them exactly. Which is only worth doing if the optimizer is going to take the
	// leaves apart anyway; with -noexplode it works with the ones the builder produced.
	bvh.settings.keepSplitting = set.explodeLeaves;
	if (filter.active())
	{
		// Build over the kept triangles only, as an indexed mesh. The tree stays indexed for the
		// whole of the optimizer: its primitive indices are then positions in the kept list, which
		// is what the builder's own fragment array is sized for, and triangle lookups go through
		// vertIdx and land on the right vertices. Unindexing happens at the end, once nothing is
		// reading those arrays any more - see the bottom of this function.
		bvh.Build( tinybvh::bvhvec4slice( tris.data(), (uint32_t)tris.size(), sizeof( bvhvec4 ) ),
			filter.vertIdx.data(), (uint32_t)filter.keep.size() );
	}
	else bvh.Build( tris.data(), (uint32_t)tris.size() / 3 );
	// Before anything else touches the tree: see RebuildSubtrees.
	if (pre.on) RebuildSubtrees( bvh, pre );
	st.seconds = MilliSec( t0, Clock::now() ) / 1000.0;
	st.sahPre = bvh.SAHCost(), st.epoPre = bvh.EPOCost();
	const Clock::time_point t1 = Clock::now();
	// The reference boxes, recovered from the tree that holds them. This has to be redone for
	// every rung and not computed once: a run merges split references and compacts what is left,
	// so by the second rung an entry no longer describes the primitive sitting at that slot. The
	// next explode then hands a leaf a box that does not bound its own triangle, and the tree
	// quietly stops finding a third of the geometry.
	std::vector<Optimizer::Ref> refBox;
	auto RecoverRefBoxes = [&]()
	{
		refBox.clear(), refBox.resize( bvh.idxCount );
		for (uint32_t n = 0; n < bvh.usedNodes; n++)
		{
			const tinybvh::BVH::BVHNode& node = bvh.bvhNode[n];
			if (!node.isLeaf() || node.triCount == 0) continue;
			for (uint32_t i = 0; i < node.triCount; i++)
				refBox[node.leftFirst + i] = RecoverRef( bvh.verts, bvh.vertIdx,
					bvh.primIdx[node.leftFirst + i], node.aabbMin, node.aabbMax );
		}
	};
	// The ladder. Each rung runs on the tree the rung below it left behind, so the search for
	// 'extreme' contains the search for 'hq', which contains the search for 'fast'. Between rungs
	// the tree is scored by the metric and the best one seen is kept, which is what makes a higher
	// profile a guarantee rather than a hope: a rung that wanders off - and annealing exists to
	// make it wander - cannot cost anything, because its result is simply not the one returned.
	//
	// Each rung gets a fresh BVH_Verbose, rebuilt from the tree the last rung produced. Run ends
	// by compacting, which shrinks the node array, and the next rung starts by exploding every
	// leaf, which needs two nodes per reference; starting from a clean object is the simplest way
	// to be sure the room is there. Nothing between rungs may reorder primIdx either, since
	// refBox runs parallel to it - which is why the leaf passes below stay where they are, after
	// the climb rather than inside it.
	const uint32_t topRung = tinybvh::tinybvh_min( profile, (uint32_t)PROF_EXTREME );
	Optimizer::Settings rung = set;
	float bestScore = BVH_FAR;
	uint32_t bestRung = 0, keepUsed = 0, keepIdx = 0;
	std::vector<tinybvh::BVH::BVHNode> keepNode;
	std::vector<uint32_t> keepPrim;
	for (uint32_t r = 0; r <= topRung; r++)
	{
		ApplyProfileRung( rung, r );
		RecoverRefBoxes();
		float score = BVH_FAR;
		{
			tinybvh::BVH_Verbose verbose;
			verbose.ConvertFrom( bvh );
			// ConvertFrom copies flags, not costs, so without this the optimizer spends its whole
			// run minimizing a different SAH than the one the builder used and the one the result
			// is judged by.
			verbose.c_trav = bvh.c_trav, verbose.c_int = bvh.c_int, verbose.lQuads = bvh.lQuads;
			Optimizer opt( verbose, rung );
			opt.SetGate( gate );
			opt.Run( refBox.data(), bvh.vertIdx );
			// ray cost is the gate's to answer and it wants the verbose tree, so it is asked here;
			// the other two read the binary tree and are asked once it is back.
			if (metric == METRIC_RAYCOST && gate) score = (float)gate->Score( verbose );
			bvh.ConvertFrom( verbose );
		}
		if (metric == METRIC_SAH) score = bvh.SAHCost();
		else if (metric == METRIC_EPO) score = bvh.EPOCost();
		if (r == 0 || score < bestScore)
		{
			bestScore = score, bestRung = r;
			keepUsed = bvh.usedNodes, keepIdx = bvh.idxCount;
			keepNode.assign( bvh.bvhNode, bvh.bvhNode + keepUsed );
			keepPrim.assign( bvh.primIdx, bvh.primIdx + keepIdx );
		}
		if (set.verbose) printf( "  %-10s rung %s: %s %.4f%s\n", CandidateName( candidate ),
			g_profileName[r], g_metricName[metric], score, bestRung == r ? "" : " (kept the rung below)" );
	}
	if (bestRung != topRung)
	{
		memcpy( bvh.bvhNode, keepNode.data(), keepUsed * sizeof( tinybvh::BVH::BVHNode ) );
		memcpy( bvh.primIdx, keepPrim.data(), keepIdx * sizeof( uint32_t ) );
		bvh.usedNodes = keepUsed, bvh.idxCount = keepIdx;
	}
	st.rung = bestRung;
	bvh.Compact();
	if (capped) { if (info.fitLeaves) bvh.FitLeafs( info.leafPrims ); else bvh.SplitLeafs( info.leafPrims ); }
	st.seconds += MilliSec( t1, Clock::now() ) / 1000.0;
	st.sahPost = bvh.SAHCost(), st.epoPost = bvh.EPOCost();
	if (filter.active())
	{
		// Unindex: map each primitive index from a position in the kept list to the triangle id it
		// came from, drop the index array, and restore the mesh's primitive count. What is left is
		// an ordinary non-indexed tree over the original mesh - leaf fetch reads verts[prim*3+k],
		// exactly as it would have without the filter - that happens not to reference what the
		// audit dropped, and that Load accepts against the original geometry.
		//
		// This has to happen here and not a line earlier. Until the optimizer is finished, the
		// builder's fragment array is live and is indexed BY these primitive indices while being
		// sized for the kept count; renumbering them into the mesh's range reads past its end, for
		// a crash that depends on where the allocation happens to sit.
		for (uint32_t i = 0; i < bvh.idxCount; i++) bvh.primIdx[i] = filter.keep[bvh.primIdx[i]];
		bvh.vertIdx = 0, bvh.triCount = (uint32_t)(tris.size() / 3);
	}
}

// The baseline every result is quoted against: a stock Bonsai build with spatial splits, left
// exactly as the builder produced it. Not a candidate and never optimized - the candidates are
// built with keepSplitting on, a prepass over their small subtrees and whatever the profile
// asks for, so quoting one of them as 'before' would be measuring against a tree that already
// had work done to it, and would move whenever the winning candidate changed.
//
// It is capped and leaf-fitted the same way the finished tree is, so the two numbers describe
// trees of the same shape and the ratio between them means something.
struct Baseline { float sah = 0, epo = 0; bool fromInput = false; };

static Baseline BuildBaseline( const std::vector<bvhvec4>& tris, const GeometryFilter& filter,
	const TargetInfo& info, const float cTrav, const float cInt )
{
	tinybvh::BVH ref;
	const bool capped = info.leafPrims > 0;
	ref.settings.useSpatialSplits = true;		// BonsaiS: Bonsai, with spatial splits
	ref.settings.binnedBVH = false;
	ref.hqLeafPrims = capped ? info.leafPrims : 0, ref.lQuads = info.quadLeaves;
	ref.c_trav = cTrav, ref.c_int = cInt;
	if (filter.active()) ref.Build( tinybvh::bvhvec4slice( tris.data(), (uint32_t)tris.size(), sizeof( bvhvec4 ) ),
		filter.vertIdx.data(), (uint32_t)filter.keep.size() );
	else ref.Build( tris.data(), (uint32_t)tris.size() / 3 );
	if (capped) { if (info.fitLeaves) ref.FitLeafs( info.leafPrims ); else ref.SplitLeafs( info.leafPrims ); }
	Baseline b;
	b.sah = ref.SAHCost(), b.epo = ref.EPOCost();
	return b;
}

// What the optimizer achieved on the tree that won, which with several candidates in play is
// not the one any single line above reports, against the baseline above.
static void ReportWinner( const CandidateStats& st, const Baseline& base )
{
	printf( "  SAH %10.4f -> %10.4f (%+6.2f%%)\n", base.sah, st.sahPost, 100.0f * (st.sahPost - base.sah) / base.sah );
	printf( "  EPO %10.4f -> %10.4f (%+6.2f%%)\n", base.epo, st.epoPost, 100.0f * (st.epoPost - base.epo) / base.epo );
	printf( "         'before' is %s, not the winning candidate's own starting point\n",
		base.fromInput ? "the tree that was handed in" : "a stock BonsaiS build" );
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
	const CandidateStats* stat, const RaySet& score, const WideSettings& w, const std::string& stem, const Baseline& base )
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
		const float rank = MetricRank( stat[c], w.metric, cost );
		if (rank < bestCost) bestCost = rank, bestIdx = c, winNs = ns;
	}
	if (candidates == 0) return;
	printf( "  winner: %s\n", CandidateName( bestIdx ) );
	if (fastestIdx != bestIdx) printf( "  note: %s was faster on the clock (%.1f vs %.1f ns); cost decided.\n",
		CandidateName( fastestIdx ), bestNs, winNs );
	ReportWinner( stat[bestIdx], base );
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
	const CandidateStats* stat, const RaySet& score, const WideSettings& w, const std::string& stem, const Baseline& base )
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
		const float rank = MetricRank( stat[c], w.metric, cost );
		if (rank < bestCost) bestCost = rank, bestIdx = c, winNs = ns;
	}
	if (candidates == 0) return;
	printf( "  winner: %s\n", CandidateName( bestIdx ) );
	if (fastestIdx != bestIdx) printf( "  note: %s was faster on the clock (%.1f vs %.1f ns); cost decided.\n",
		CandidateName( fastestIdx ), bestNs, winNs );
	ReportWinner( stat[bestIdx], base );
	const std::string path = stem + "_" + info.name + ".bvh";
	bvh[bestIdx].Save( path.c_str() );
	printf( "  output: %s\n", path.c_str() );
}

template <typename T, int M> static void EvaluateTarget( const TargetInfo& info, tinybvh::BVH* bvh,
	const uint32_t candidates, const CandidateStats* stat, const std::vector<float>* prob, const RaySet& score,
	const WideSettings& w, const std::string& stem, const Baseline& base )
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
		const float rank = MetricRank( stat[c], w.metric, r.cost );
		if (rank < bestScore) bestScore = rank, bestIdx = (uint32_t)results.size() - 1, best = tinybvh::tinybvh_move( tree );
	}
	if (results.empty()) return;
	const WideResult& win = results[bestIdx];
	const WideResult* fastest = &results[0];
	for (const WideResult& r : results) if (r.ns >= 0 && (fastest->ns < 0 || r.ns < fastest->ns)) fastest = &r;
	printf( "  winner: %s, %s collapse\n", CandidateName( win.candidate ), win.weighted ? "visit-weighted" : "area-weighted" );
	if (fastest->ns >= 0 && fastest != &win) printf( "  note: %s / %s was faster on the clock (%.1f vs %.1f ns); cost decided.\n",
		CandidateName( fastest->candidate ), fastest->weighted ? "visits" : "area", fastest->ns, win.ns );
	ReportWinner( stat[win.candidate], base );
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

// PeekLayout: the first word of a saved BVH carries its layout in the top byte, so a .bvh can
// be asked which target produced it without being loaded. That is what lets -profile and
// -metric be applied to a .bvh without the user having to repeat --target.
static uint32_t PeekLayout( const std::string& path )
{
	FILE* f = fopen( path.c_str(), "rb" );
	if (!f) return 0;
	uint32_t header = 0;
	const bool ok = fread( &header, sizeof( uint32_t ), 1, f ) == 1;
	fclose( f );
	return ok ? (header >> 24) : 0;
}

static uint32_t TargetForLayout( const uint32_t layout )
{
	switch (layout)
	{
	case tinybvh::LAYOUT_BVH: return TGT_BVH;
	case tinybvh::LAYOUT_BVH_GPU: return TGT_BVHGPU;
	case tinybvh::LAYOUT_BVH4_CPU: return TGT_BVH4CPU;
	case tinybvh::LAYOUT_BVH8_SIMD: return TGT_BVH8CPU;
	case tinybvh::LAYOUT_BVH4_GPU: return TGT_BVH4GPU;
	case tinybvh::LAYOUT_CWBVH: return TGT_BVH8CWBVH;
	case tinybvh::LAYOUT_BVH4_AMD_HW: return TGT_BVH4AMDHW;
	case tinybvh::LAYOUT_CWBVH8_AMD_HW: return TGT_CWBVH8AMDHW;
	}
	return (uint32_t)-1;
}

// The geometry, from a mesh or from a .bvh that carries its own. A wide layout stores its
// triangles inside its leaf blocks rather than as a soup, so those files give up their layout
// but not their geometry; the caller is told to bring the mesh.
// The geometry, and - when the input was a tree rather than a mesh - that tree, kept so the
// results can be quoted against what the user actually handed over. 'input' is left untouched
// when the path is a mesh, and the caller falls back to building its own baseline.
static bool LoadGeometry( const std::string& path, std::vector<bvhvec4>& tris,
	tinybvh::BVH& input, bool& haveInput )
{
	haveInput = false;
	// Ask what the file is before trying to read it as a mesh: a saved BVH begins with a header
	// word, and a mesh loader reading that as a triangle count asks for gigabytes.
	const uint32_t layout = PeekLayout( path );
	if (TargetForLayout( layout ) == (uint32_t)-1) return LoadMesh( path.c_str(), tris );
	if (layout != tinybvh::LAYOUT_BVH) return false;
	input = tinybvh::BVH::Load( path.c_str() );
	if (input.usedNodes == 0 || !input.verts || input.vertIdx) return false;
	tris.resize( input.verts.count );
	for (uint32_t i = 0; i < (uint32_t)input.verts.count; i++) tris[i] = input.verts[i];
	haveInput = true;
	return true;
}

static int RunWideTargets( const std::string& meshPath, const Optimizer::Settings& set, const WideSettings& w )
{
	std::vector<bvhvec4> tris;
	tinybvh::BVH input;
	bool haveInput = false;
	if (!LoadGeometry( meshPath, tris, input, haveInput ))
	{
		const uint32_t layout = PeekLayout( meshPath );
		const uint32_t derived = TargetForLayout( layout );
		if (derived != (uint32_t)-1) printf(
			"error: '%s' is a %s tree, which keeps its triangles inside its own leaves rather\n"
			"       than as a mesh. Pass the mesh it was built from; its layout is already known,\n"
			"       so --target %s is implied.\n", meshPath.c_str(), g_target[derived].name, g_target[derived].name );
		else printf( "error: cannot read '%s' as a mesh (uint32 triangle count, then three float4 per triangle).\n", meshPath.c_str() );
		return 1;
	}
	printf( "input  : %s\n         %u triangles\n", meshPath.c_str(), (uint32_t)tris.size() / 3 );
	GeometryFilter filter;
	if (w.audit || w.fix) ReportGeometry( AuditGeometry( tris, w.fix, filter ), (uint32_t)tris.size() / 3, w.fix );
	const std::string stem = w.outStem.empty() ? StripExtension( meshPath ) : w.outStem;
	// Targets that want the same leaf budget share their binary trees; the rest get their own,
	// since the optimizer's final collapse is what produces leaves of the right size.
	RaySet fit, score;
	{
		// The ray sets used to be drawn from the first optimized candidate. The calibration and
		// the round gate both need them before any candidate exists, so they are drawn from a
		// throwaway tree instead: the distribution is a property of where the geometry is and
		// what it occludes, not of how it happens to be organized, so the sets are the same.
		tinybvh::BVH seed;
		seed.Build( tris.data(), (uint32_t)tris.size() / 3 );
		GenerateRays( seed, w.rays, set.seed ^ 0x9E3779B9u, fit );
		GenerateRays( seed, w.rays, set.seed ^ 0x85EBCA6Bu, score );
		printf( "         ray sets: %u to fit, %u to score, from disjoint seeds\n",
			(uint32_t)fit.Count(), (uint32_t)score.Count() );
	}
	const bool gateable = !filter.active();	// see the note where the gate is created
	printf( "         profile: %s, metric: %s\n", g_profileName[w.profile], g_metricName[w.metric] );
	if (w.metric == METRIC_RAYCOST && !gateable)
		printf( "         note: ray cost needs the round gate, which -fix disables; falling back to sah\n" );
	if (w.calibrateOnly)
	{
		// Every target fitted on its own, which normal operation cannot do: targets that share a
		// leaf budget share their binary trees, and a binary tree can only be built with one cost
		// model, so there the fit is per group. Nothing is built here, so each layout answers for
		// itself - which is what deriving a default for a layout needs.
		printf( "\n=== cost model calibration ===\n" );
		printf( "  %-13s %9s %9s %8s %7s %7s %7s %8s %7s  %s\n", "target", "ns/node", "ns/prim",
			"c_int", "r2", "r2+MB", "corr", "residual", "noise", "verdict" );
		const uint32_t runs = w.timingRuns > 5 ? w.timingRuns : 5;
		for (uint32_t t = 0; t < TGT_COUNT; t++)
		{
			if (!w.target[t]) continue;
			const CostFit cf = CalibrateCostModel( t, g_target[t], tris, fit, runs, w.fitDetail );
			printf( "  %-13s %9.3f %9.3f ", g_target[t].name, cf.nsPerNode, cf.nsPerPrim );
			if (cf.usable) printf( "%8.3f", cf.cInt ); else printf( "%8s", "-" );
			printf( " %7.3f %7.3f %+7.3f %8.1f %7.1f  %s\n", cf.r2, cf.r2WithSize, cf.corr,
				cf.residual, cf.noise, cf.usable ? "ok" : cf.why );
			fflush( stdout );
		}
		printf( "\n  c_int is the ratio of the two, with c_trav pinned at 1, and is only reported when\n"
			"  the fit survives every check. Reading a refusal:\n"
			"    corr near +/-1   the two counts move together and no sample can separate them\n"
			"    residual >> noise  the counts are not where the time goes, unless r2 is above 0.90\n"
			"    noise large      the machine, not the model; retry quieter or with more timingruns\n"
			"  A layout whose r2 rises sharply once the tree's footprint is allowed in is a layout\n"
			"  whose traversal time is not two counts, and no value of c_int will capture it.\n" );
		return 0;
	}
	bool done[TGT_COUNT] = {};
	for (uint32_t t = 0; t < TGT_COUNT; t++)
	{
		if (!w.target[t] || done[t]) continue;
		const TargetInfo& regime = g_target[t];
		if (regime.leafPrims == 0) printf( "\n=== binary trees, SAH-sized leaves ===\n" );
		else printf( "\n=== binary trees, %u primitive%s per leaf%s ===\n", regime.leafPrims,
			regime.leafPrims == 1 ? "" : "s", regime.quadLeaves ? ", quad-packed" : "" );
		// One representative target decides the cost model and the gate for the whole group,
		// since the group shares its binary trees. Prefer one with a CPU traversal kernel: the
		// two hardware formats have none, so neither the fit nor -gatetime can measure them.
		uint32_t rep = t;
		for (uint32_t u = t; u < TGT_COUNT; u++)
		{
			if (!w.target[u] || g_target[u].leafPrims != regime.leafPrims) continue;
			if (g_target[u].quadLeaves != regime.quadLeaves) continue;
			if (u != TGT_BVH4AMDHW && u != TGT_CWBVH8AMDHW) { rep = u; break; }
		}
		// The library's own defaults, not a local guess: a hard-coded 2 here was measured to cost
		// 30% more nodes and 30% more memory on Sponza for no speed, and it disagreed with both
		// the builder and the calibration.
		float cTrav = (float)C_TRAV, cInt = (float)C_INT;
		if (w.fitCost)
		{
			printf( "  calibrating the cost model against measured traversal:\n" );
			const uint32_t runs = w.timingRuns > 5 ? w.timingRuns : 5;
			const CostFit cf = CalibrateCostModel( rep, g_target[rep], tris, fit, runs, w.fitDetail );
			ReportCostFit( cf, g_target[rep].name );
			if (cf.usable) cTrav = cf.cTrav, cInt = cf.cInt;
		}
		// Everything downstream is priced with these two, the cost/ray column included, so a run
		// with a measured pair is not comparable to one without. Say which is in force.
		printf( "  cost model: c_trav %.3f, c_int %.3f (%s)\n", cTrav, cInt,
			w.fitCost ? "measured where the fit held, C_TRAV / C_INT otherwise" : "C_TRAV / C_INT" );
		// The gate prices primitives by walking them, and with -fix the primitive indices are
		// positions in the kept list for the whole of the optimizer while the walk reads the mesh
		// directly. Rather than renumber a live array mid-run, the gate stands down.
		const RoundGate* gate = 0;
		if (w.gate && gateable)
		{
			gate = MakeGate( rep, g_target[rep], fit, cTrav, cInt, w.gateTime, w.timingRuns );
			printf( "  round gate: %s, measured on %s with %u rays\n", gate->Unit(),
				g_target[rep].name, (uint32_t)fit.Count() );
		}
		else if (w.gate) printf( "  round gate: unavailable with -fix; rounds are judged by SAH\n" );
		// The baseline the result is quoted against. Given a tree to improve, that is the tree
		// itself - the user wants to know what this run bought over what they handed in, and
		// re-capping it would be answering a different question. Given a mesh, there is nothing
		// to compare with, so a stock BonsaiS build stands in: one per group, capped to the
		// group's leaf budget, since that is the only thing about a target that changes a tree's
		// shape. Either way the costs are the group's own, so both numbers are in the same units.
		Baseline base;
		if (haveInput)
		{
			input.c_trav = cTrav, input.c_int = cInt, input.lQuads = regime.quadLeaves;
			base.sah = input.SAHCost(), base.epo = input.EPOCost(), base.fromInput = true;
			printf( "  baseline:  the input tree, SAH %.4f, EPO %.4f\n", base.sah, base.epo );
		}
		else
		{
			base = BuildBaseline( tris, filter, regime, cTrav, cInt );
			printf( "  baseline:  stock BonsaiS build, SAH %.4f, EPO %.4f\n", base.sah, base.epo );
		}
		tinybvh::BVH bvh[CAND_MAX];
		CandidateStats stat[CAND_MAX] = {};
		for (uint32_t c = 0; c < w.candidates; c++)
		{
			PrepareCandidate( bvh[c], tris, c, set, regime, stat[c], filter, w.pre, cTrav, cInt, gate,
				w.profile, (w.metric == METRIC_RAYCOST && !gateable) ? METRIC_SAH : w.metric );
			printf( "  %-10s %u references, %u nodes, SAH %.4f, EPO %.4f, %.1f s\n", CandidateName( c ),
				bvh[c].idxCount, bvh[c].usedNodes, stat[c].sahPost, stat[c].epoPost, stat[c].seconds );
		}
		delete gate;
		if (w.hotOrder) for (uint32_t c = 0; c < w.candidates; c++)
		{
			std::vector<uint32_t> visit;
			CountPassTests( bvh[c], fit, visit );
			uint32_t ties = 0, pairs = 0;
			const uint32_t swaps = HotChildFirst( bvh[c], visit, ties, pairs );
			bvh[c].Compact();	// the relabeling only becomes a layout once the tree is rewritten
			printf( "  %-10s hot-child order: %u of %u pairs swapped, %u tied (%.1f%%)\n", CandidateName( c ),
				swaps, pairs, ties, pairs ? 100.0 * ties / pairs : 0.0 );
		}
		std::vector<float> prob[CAND_MAX];
		if (w.weighted) for (uint32_t c = 0; c < w.candidates; c++)
		{
			// after Compact every node has moved, so the counts are taken again rather than carried.
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
			case TGT_BVH: EvaluateBinaryTarget( g_target[u], bvh, w.candidates, stat, score, w, stem, base ); break;
			case TGT_BVHGPU: EvaluateGPUTarget( g_target[u], bvh, w.candidates, stat, score, w, stem, base ); break;
			case TGT_BVH4CPU: EvaluateTarget<tinybvh::BVH4_CPU, 4>( g_target[u], bvh, w.candidates, stat, prob, score, w, stem, base ); break;
			case TGT_BVH8CPU: EvaluateTarget<tinybvh::BVH8_CPU, 8>( g_target[u], bvh, w.candidates, stat, prob, score, w, stem, base ); break;
			case TGT_BVH4GPU: EvaluateTarget<tinybvh::BVH4_GPU, 4>( g_target[u], bvh, w.candidates, stat, prob, score, w, stem, base ); break;
			case TGT_BVH8CWBVH: EvaluateTarget<tinybvh::BVH8_CWBVH, 8>( g_target[u], bvh, w.candidates, stat, prob, score, w, stem, base ); break;
			case TGT_BVH4AMDHW: EvaluateTarget<tinybvh::BVH4_AMD_HW, 4>( g_target[u], bvh, w.candidates, stat, prob, score, w, stem, base ); break;
			case TGT_CWBVH8AMDHW: EvaluateTarget<tinybvh::CWBVH8_AMD_HW, 8>( g_target[u], bvh, w.candidates, stat, prob, score, w, stem, base ); break;
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
		"  -prepass           before optimizing, rebuild every subtree of 16..128 references\n"
		"                     standalone with spatial splits and keep the cheapest result over\n"
		"                     every bin count in 8..32, with and without odd/even. 50 builds\n"
		"                     per subtree; costs about 1%% more references\n"
		"  -audit             report duplicate, zero-area and non-finite triangles, and where\n"
		"                     the worst of them are; off by default, and changes nothing\n"
		"  -fix               audit, and keep what it finds out of the tree. Triangle ids are\n"
		"                     not renumbered, so a renderer indexing materials by primitive id\n"
		"                     is unaffected; see AuditGeometry for what this does and does not\n"
		"                     justify doing to a mesh\n"
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
		"\n"
		"  -fitcost           measure c_trav and c_int rather than assuming 1 and 2: a spread of\n"
		"                     trees is built over the mesh, timed in the target layout, and the two\n"
		"                     costs fitted to the timings. A fit that does not hold up is reported\n"
		"                     and discarded, not used. Off by default; see -calibrate first, since\n"
		"                     whether a fit holds at all is scene and layout dependent\n"
		"  -fitdetail         show every calibration sample rather than the verdict alone\n"
		"  -agglom N          restructure the costly nodes with agglomerative treelets of N leaves\n"
		"                     as well as the exact DP ones (default 24, max 48). Gives up optimality\n"
		"                     per treelet to reach sizes the O(3^n) DP cannot\n"
		"  -anneal F          let reinsertion accept uphill moves, starting at a temperature of F\n"
		"                     times the root box area and cooling to zero within each call, so the\n"
		"                     walk can leave a local minimum and still end in one (try 0.02)\n"
		"  -profile P         how hard to look: fast, hq (default) or extreme. The three are rungs\n"
		"                     of one ladder - hq continues from the tree fast produced, extreme from\n"
		"                     the tree hq produced, and the best by -metric is kept - so a higher\n"
		"                     profile cannot return a worse tree, only a slower answer\n"
		"  -metric M          what to minimize: sah (default), epo, or raycost. sah is the model,\n"
		"                     epo corrects for where rays stop, raycost counts node and primitive\n"
		"                     tests on real rays through the collapsed tree in the target layout.\n"
		"                     Sets the per-round test and the local guidance to match; both can\n"
		"                     still be overridden by flags given after it\n"
		"  -hotorder          lay each tree out with the more visited sibling first, so the nodes\n"
		"                     rays reach sit next to their parent. A relabeling of the tree, not a\n"
		"                     change to it: the same queries return the same answers\n"
		"  -calibrate         fit every target listed in --target, report, and build nothing. Each\n"
		"                     target is fitted on its own here, which a normal run cannot do: there\n"
		"                     targets sharing a leaf budget share a binary tree and so one fit\n"
		"  -gate              judge each round by measured traversal work on the collapsed tree in\n"
		"                     the target layout rather than by SAH, and undo a round that does not\n"
		"                     lower it. Uses the fit ray set, not the scoring one\n"
		"  -gatetime          -gate, but measured with the clock. Noisier, and only where there is\n"
		"                     a CPU kernel to run\n"
		"  -gatepatience N    undone rounds in a row before a gated run gives up (default 3). Each\n"
		"                     retry reseeds and samples the tree differently, so a flat round is\n"
		"                     not mistaken for a floor\n"
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
		else if (a == "-audit") wide.audit = true;
		else if (a == "-fitcost") wide.fitCost = true;
		else if (a == "-nofitcost") wide.fitCost = false;
		else if (a == "-fitdetail") wide.fitDetail = true;
		else if (a == "-calibrate") wide.calibrateOnly = true;
		else if (a == "-hotorder") wide.hotOrder = true;
		else if (a == "-profile" && i + 1 < argc)
		{
			const std::string v = argv[++i];
			if (v == "fast") wide.profile = PROF_FAST;
			else if (v == "hq") wide.profile = PROF_HQ;
			else if (v == "extreme") wide.profile = PROF_EXTREME;
			else { printf( "error: -profile wants fast, hq or extreme.\n" ); return 1; }
			// Effort that is not per-candidate: how many starting trees, and whether to spend the
			// prepass. Candidates 0 and 1 are the ones the lower profiles use, so a higher
			// profile's set contains a lower one's and the ladder's guarantee survives it.
			wide.candidates = wide.profile == PROF_FAST ? 1u : wide.profile == PROF_HQ ? 2u : CAND_MAX;
			wide.pre.on = wide.profile != PROF_FAST;
		}
		else if (a == "-metric" && i + 1 < argc)
		{
			const std::string v = argv[++i];
			if (v == "sah") wide.metric = METRIC_SAH;
			else if (v == "epo") wide.metric = METRIC_EPO;
			else if (v == "raycost") wide.metric = METRIC_RAYCOST;
			else { printf( "error: -metric wants sah, epo or raycost.\n" ); return 1; }
			// A stage that optimizes one thing while the round is judged by another is a slower
			// way of reaching the wrong tree, so the per-round test and the local guidance term
			// both follow the metric. Either can still be overridden after this on the line.
			wide.gate = wide.metric == METRIC_RAYCOST;
			set.epoGate = wide.metric == METRIC_EPO;
			set.epoGuide = wide.metric == METRIC_SAH ? 0.0f : wide.metric == METRIC_EPO ? 1.0f : 0.5f;
		}
		else if (a == "-agglom") set.agglomSize = num( 24 );
		else if (a == "-anneal") set.anneal = i + 1 < argc ? (float)atof( argv[++i] ) : 0.02f;
		else if (a == "-gate") wide.gate = true;
		else if (a == "-gatetime") wide.gate = true, wide.gateTime = true;
		else if (a == "-gatepatience") set.gatePatience = num( 3 );
		else if (a == "-prepass") wide.pre.on = true;
		else if (a == "-fix") wide.fix = true;
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
	if (wide.calibrateOnly && !wide.Any()) { printf( "error: -calibrate needs --target and a mesh; there is no layout to time otherwise.\n" ); return 1; }
	if (wide.calibrateOnly && wide.fix) printf( "note: -calibrate builds its own trees over the whole mesh; -fix does not apply to them.\n" );
	// A .bvh says which layout it is, so -profile and -metric can be pointed at one without the
	// user repeating --target. An explicit --target still wins; this only fills in a blank.
	{
		const uint32_t layout = PeekLayout( bvhPath );
		const uint32_t derived = TargetForLayout( layout );
		if (derived != (uint32_t)-1)
		{
			printf( "input  : %s was saved as %s\n", bvhPath.c_str(), g_target[derived].name );
			if (!wide.Any()) wide.target[derived] = true;
		}
	}
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
	// Before anything else touches the tree, and before the figures it is judged against are
	// taken: see RebuildSubtrees. It rewrites nodes and references, so it has to precede both
	// RecoverRef, which reads the leaf boxes, and the conversion to BVH_Verbose.
	if (wide.pre.on) RebuildSubtrees( bvh, wide.pre );
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
	verbose.c_trav = bvh.c_trav, verbose.c_int = bvh.c_int, verbose.lQuads = bvh.lQuads;
	// Without a target the tree stays binary, so the gate is the binary walk, measured with a
	// ray set drawn from the input. -fitcost needs a layout to time and has nothing to do here.
	RaySet gateRays;
	const RoundGate* gate = 0;
	if (wide.gate)
	{
		GenerateRays( bvh, wide.rays, set.seed ^ 0x9E3779B9u, gateRays );
		gate = MakeGate( TGT_BVH, g_target[TGT_BVH], gateRays, bvh.c_trav, bvh.c_int, wide.gateTime, wide.timingRuns );
		printf( "         round gate: %s, measured on %u rays\n", gate->Unit(), (uint32_t)gateRays.Count() );
	}
	printf( "optimizing:\n" );
	t0 = Clock::now();
	Optimizer opt( verbose, set );
	opt.SetGate( gate );
	opt.Run( refBox.data(), bvh.vertIdx );
	const double optMs = MilliSec( t0, Clock::now() );
	delete gate;
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
