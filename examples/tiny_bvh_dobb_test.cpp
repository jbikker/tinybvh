#define FENSTER_APP_IMPLEMENTATION
#define SCRWIDTH 400 // 1280
#define SCRHEIGHT 400 // 720
#define TILESIZE 20
#include "external/fenster.h" // https://github.com/zserge/fenster

#define TINYBVH_IMPLEMENTATION
#include "tiny_bvh.h"
#define TINYBVH_DOBB_IMPLEMENTATION
#include "tiny_bvh_dobb.h"
using namespace tinybvh;

// Other includes
#include <cstdlib>
#include <cstdio>

#ifdef _WIN32
#include <windows.h>
#endif
#include <iostream>
#include <iomanip>
#include <string>

#define NUMBEROFVIEWPOINTS 25

enum Phase {
	BUILD,
	RENDERMBVH,
	RENDERDOBB,
	END
};

// For instrumentation for experiment
struct TraversalStats {
	uint64_t rayCount = 0;
	uint64_t nodeVisits = 0;
	uint64_t boxTests = 0;
	uint64_t triTests = 0;

	void add(const TraversalStats& other) {
		rayCount += other.rayCount;
		nodeVisits += other.nodeVisits;
		boxTests += other.boxTests;
		triTests += other.triTests;
	}

	void reset() {
		rayCount = 0; nodeVisits = 0; boxTests = 0; triTests = 0;
	}

	void print(const char* statName) {
		// Guards against division by 0
		const double guardedRayCount = rayCount ? (double)rayCount : 1.0f;
		std::cout << std::fixed << std::setprecision(3) <<
			statName << " " << rayCount << std::endl <<
			"Avg nodes visited per ray: " << nodeVisits / guardedRayCount << std::endl <<
			"Avg boxes intersected per ray: " << boxTests / guardedRayCount << std::endl <<
			"Avg triangles intersected per ray: " << triTests / guardedRayCount << std::endl;
	}
};

// Setup view pyramid for a pinhole camera:
// eye, p1 (top-left), p2 (top-right), and p3 (bottom-left)
struct camera {
	bvhvec3 eye;
	bvhvec3 view;
	bvhvec3 p1, p2, p3;

	void initCamera(bvhvec3 _eye, bvhvec3 _view) {
		eye = _eye; view = tinybvh_normalize(_view);

		// Update camera vector once
		bvhvec3 right = tinybvh_normalize(tinybvh_cross(bvhvec3(0, 1, 0), view));
		bvhvec3 up = 0.8f * tinybvh_cross(view, right);
		bvhvec3 C = eye + 1.2f * view;
		p1 = C - right + up;   // top-left
		p2 = C + right + up;   // top-right
		p3 = C - right - up;   // bottom-left
	}
};

camera cam;

static camera viewPointCameras[NUMBEROFVIEWPOINTS];
static int viewPointIdx = 0;

struct SceneSetup {
	const char* sceneName;
	float viewPointScale = 1.0f;
	const char* meshPath[2];
	bvhvec3 eye, view;
};

static const SceneSetup scenes[] = {
	{"HAIRBALL",		1.0f,  {"./testdata/hairball.bin", nullptr}, bvhvec3(0.0f, 0.0f, -10.0f),    bvhvec3(0.0f, 0.0f, 1.0f)},
	{"BISTRO_EXTERIOR", 0.2f,  {"./testdata/bistro_ext_part1.bin", "./testdata/bistro_ext_part2.bin"}, bvhvec3(-24.2f, -8.2f, -2.0f),  bvhvec3(10.65f, -0.06f, 0.26f)},
	{"WHITE_OAK",		1.0f,  {"./testdata/white_oak.bin", nullptr},		  bvhvec3(0.34f, 3.3f, -7.0f),   bvhvec3(0.0f, 0.0f, 1.0f)},
	{"POWERPLANT",		0.15f, {"./testdata/powerplant.bin", nullptr}, bvhvec3(-57000.0f, 103000.0f, -40000.0f), bvhvec3(0.26f, -0.6f, 0.76f)},
	{"DRAGON",			1.0f,  {"./testdata/dragon_scene.bin", nullptr}, bvhvec3(15.7f, 6.2f, 14.5f), bvhvec3(-0.731f, -0.105f, -0.674f)},
	{"CRYTEKSPONZA",	0.15f, {"./testdata/cryteksponza.bin", nullptr}, bvhvec3(-15.24f, 21.5f, 2.54f), bvhvec3(0.826f, -0.438f, -0.356f)}
};
static const int sceneCount = sizeof(scenes) / sizeof(scenes[0]);
static int sceneIdx = 0;

// Helpers for experimental setup
static Phase phase = BUILD;

// Application variables
static TraversalStats primHitStats, secondHitStats, shadowStats;
static const int targetSpp = 8;
static const float nodeVisitScale = 1.0f / 250.0f;
static const float triTestScale = 1.0f / 1000.0f;
static const float boxTestScale = 1.0f / 2000.0f;
static BVH_DOBB dobbBvh;
static bvhvec4* tris = 0;
static int triCount = 0, spp = 0;
static bvhvec3 accumulator[SCRWIDTH * SCRHEIGHT];
static std::atomic<int> tileIdx(0);

// ---------------------- INSTRUMENTATION ----------------------

static float BoxSurfaceArea(const bvhvec3& min, const bvhvec3& max) {
	const bvhvec3 extent = max - min;
	return 2.0f * (extent.x * extent.y + extent.y * extent.z + extent.z * extent.x);
}

struct StackEntryDepth { uint32_t nodeIdx; uint32_t depth; };

void printMBVHSAH(const char* tableName) {
	float internalSAs[128] = { 0.0f }, leafSAs[128] = { 0.0f };
	const float rootSA = BoxSurfaceArea(dobbBvh.bvh8.mbvhNode[0].aabbMin, dobbBvh.bvh8.mbvhNode[0].aabbMax);

	float totalInternalSA = 0.0f, totalLeafSA = 0.0f;
	StackEntryDepth nodeStack[512];
	uint32_t stackIdx = 0;
	nodeStack[stackIdx++] = { 0, 0 };

	uint32_t maxDepth = 0;

	while (stackIdx > 0) {
		const MBVH<8>::MBVHNode& currNode = dobbBvh.bvh8.mbvhNode[nodeStack[--stackIdx].nodeIdx];
		const float nodeSA = BoxSurfaceArea(currNode.aabbMin, currNode.aabbMax);

		maxDepth = tinybvh_max(maxDepth, nodeStack[stackIdx].depth);

		if (currNode.isLeaf()) {
			totalLeafSA += nodeSA;
			leafSAs[nodeStack[stackIdx].depth] += nodeSA;
		}
		else {
			totalInternalSA += nodeSA;
			internalSAs[nodeStack[stackIdx].depth] += nodeSA;

			const uint32_t nextDepth = nodeStack[stackIdx].depth + 1;

			for (uint32_t childIdx = 0; childIdx < currNode.childCount; childIdx++) {
				nodeStack[stackIdx++] = { currNode.child[childIdx], nextDepth };
			}
		}
	}

	static const int firstColWidth = 16, colWidth = 8;

	std::cout << std::fixed << std::setprecision(3);

	std::cout << std::fixed << std::setprecision(3);

	std::cout << std::left << std::setw(firstColWidth) << tableName << std::right
		<< " | " << std::setw(colWidth) << "Internal"
		<< " | " << std::setw(colWidth) << "Leaf"
		<< " | " << std::setw(colWidth) << "All" << " | " << std::endl;
	std::cout << std::left << std::setw(firstColWidth) << "Total SAH:" << std::right
		<< " | " << std::setw(colWidth) << totalInternalSA / rootSA
		<< " | " << std::setw(colWidth) << totalLeafSA / rootSA
		<< " | " << std::setw(colWidth) << (totalInternalSA + totalLeafSA) / rootSA << " | " << std::endl;

	for (uint32_t depth = 0; depth <= maxDepth; depth++) {
		const std::string depthLabel = "SAH at depth " + std::to_string(depth) + ":";
		std::cout << std::left << std::setw(firstColWidth) << depthLabel << std::right
			<< " | " << std::setw(colWidth) << internalSAs[depth] / rootSA
			<< " | " << std::setw(colWidth) << leafSAs[depth] / rootSA
			<< " | " << std::setw(colWidth) << (internalSAs[depth] + leafSAs[depth]) / rootSA << " | " << std::endl;
	}
	std::cout << std::endl;
}

void printDOBBSAHAndStats(const char* tableName) {
	static constexpr uint32_t MAX_DEPTH = 128;
	float internalSAs[MAX_DEPTH] = { 0.0f }, leafSAs[MAX_DEPTH] = { 0.0f };
	uint32_t internalBoxCount[MAX_DEPTH] = { 0 }, leafBoxCount[MAX_DEPTH] = { 0 };
	uint32_t rotatedInternalBoxCount[MAX_DEPTH] = { 0 }, rotatedLeafBoxCount[MAX_DEPTH] = { 0 };

	// Root: its own world-space box, internal, never rotated
	const float rootSA = BoxSurfaceArea(dobbBvh.aabbMin, dobbBvh.aabbMax);
	internalSAs[0] = rootSA;
	internalBoxCount[0] = 1;

	StackEntryDepth nodeStack[512];
	uint32_t stackIdx = 0;
	nodeStack[stackIdx++] = { 0, 0 };

	uint32_t maxDepth = 0;

	while (stackIdx > 0) {
		const StackEntryDepth currEntry = nodeStack[--stackIdx];
		const BVH_DOBB::BVHNode& currNode = dobbBvh.dobbNode[currEntry.nodeIdx];
		const bool isRotated = currNode.Rotation() != 255;
		const uint32_t childDepth = tinybvh_min(currEntry.depth + 1, MAX_DEPTH - 1);
		maxDepth = tinybvh_max(maxDepth, childDepth);

		const float stepSize[3] = {
			ldexpf(1.0f, currNode.exponent[0]),
			ldexpf(1.0f, currNode.exponent[1]),
			ldexpf(1.0f, currNode.exponent[2])
		};

		for (uint32_t slot = 0; slot < 8; slot++) {
			if (currNode.meta[slot] == 0) continue;   // empty slot

			// Decode the quantized child box (in this node's frame; rotation keeps the area)
			const bvhvec3 slotMin = currNode.nodeMinPoint + bvhvec3(
				currNode.qLo[0][slot] * stepSize[0], currNode.qLo[1][slot] * stepSize[1], currNode.qLo[2][slot] * stepSize[2]);
			const bvhvec3 slotMax = currNode.nodeMinPoint + bvhvec3(
				currNode.qHi[0][slot] * stepSize[0], currNode.qHi[1][slot] * stepSize[1], currNode.qHi[2][slot] * stepSize[2]);
			const float slotSA = BoxSurfaceArea(slotMin, slotMax);

			if (currNode.iMask & (1u << slot)) {
				internalSAs[childDepth] += slotSA;
				internalBoxCount[childDepth]++;
				if (isRotated) rotatedInternalBoxCount[childDepth]++;

				const uint32_t childNodeIdx = currNode.ChildBaseIndex() +
					(uint32_t)std::popcount((uint32_t)(currNode.iMask & ((1u << slot) - 1)));
				nodeStack[stackIdx++] = { childNodeIdx, currEntry.depth + 1 };
			}
			else {
				leafSAs[childDepth] += slotSA;
				leafBoxCount[childDepth]++;
				if (isRotated) rotatedLeafBoxCount[childDepth]++;
			}
		}
	}

	// Totals over all depths
	float totalInternalSA = 0.0f, totalLeafSA = 0.0f;
	uint32_t totalInternalBoxes = 0, totalLeafBoxes = 0, totalRotatedInternal = 0, totalRotatedLeaf = 0;
	for (uint32_t depth = 0; depth <= maxDepth; depth++) {
		totalInternalSA += internalSAs[depth];
		totalLeafSA += leafSAs[depth];
		totalInternalBoxes += internalBoxCount[depth];
		totalLeafBoxes += leafBoxCount[depth];
		totalRotatedInternal += rotatedInternalBoxCount[depth];
		totalRotatedLeaf += rotatedLeafBoxCount[depth];
	}

	// Percentage with a guard for levels without boxes of that kind
	auto percentage = [](const uint32_t part, const uint32_t whole) {
		return whole > 0 ? 100.0f * part / whole : 0.0f;
		};

	static const int firstColWidth = 12, colWidth = 12;

	const std::ios_base::fmtflags previousFlags = std::cout.flags();
	const std::streamsize previousPrecision = std::cout.precision();
	std::cout << std::fixed << std::setprecision(3);

	std::cout << std::left << std::setw(firstColWidth) << tableName << std::right
		<< " | " << std::setw(colWidth) << "Internal SAH"
		<< " | " << std::setw(colWidth) << "Leaf SAH"
		<< " | " << std::setw(colWidth) << "All SAH"
		<< " | " << std::setw(colWidth) << "Int. rot. %"
		<< " | " << std::setw(colWidth) << "Leaf rot. %"
		<< " | " << std::setw(colWidth) << "All rot. %" << " | " << std::endl;
	std::cout << std::left << std::setw(firstColWidth) << "Total:" << std::right
		<< " | " << std::setw(colWidth) << totalInternalSA / rootSA
		<< " | " << std::setw(colWidth) << totalLeafSA / rootSA
		<< " | " << std::setw(colWidth) << (totalInternalSA + totalLeafSA) / rootSA
		<< " | " << std::setw(colWidth) << percentage(totalRotatedInternal, totalInternalBoxes)
		<< " | " << std::setw(colWidth) << percentage(totalRotatedLeaf, totalLeafBoxes)
		<< " | " << std::setw(colWidth) << percentage(totalRotatedInternal + totalRotatedLeaf, totalInternalBoxes + totalLeafBoxes)
		<< " | " << std::endl;

	for (uint32_t depth = 0; depth <= maxDepth; depth++) {
		const std::string depthLabel = "Depth " + std::to_string(depth) + ":";
		std::cout << std::left << std::setw(firstColWidth) << depthLabel << std::right
			<< " | " << std::setw(colWidth) << internalSAs[depth] / rootSA
			<< " | " << std::setw(colWidth) << leafSAs[depth] / rootSA
			<< " | " << std::setw(colWidth) << (internalSAs[depth] + leafSAs[depth]) / rootSA
			<< " | " << std::setw(colWidth) << percentage(rotatedInternalBoxCount[depth], internalBoxCount[depth])
			<< " | " << std::setw(colWidth) << percentage(rotatedLeafBoxCount[depth], leafBoxCount[depth])
			<< " | " << std::setw(colWidth) << percentage(rotatedInternalBoxCount[depth] + rotatedLeafBoxCount[depth],
				internalBoxCount[depth] + leafBoxCount[depth])
			<< " | " << std::endl;
	}
	std::cout << std::endl;

	std::cout.flags(previousFlags);
	std::cout.precision(previousPrecision);
}

// Same signature and behaviour as BVHBase::IntersectTri (Möller–Trumbore, no opacity maps)
static void IntersectTri(Ray& ray, const uint32_t triIdx, const bvhvec4slice& verts,
	const uint32_t i0, const uint32_t i1, const uint32_t i2)
{
	const bvhvec3 vertex0 = bvhvec3(verts[i0]);
	const bvhvec3 edge1 = bvhvec3(verts[i1]) - vertex0, edge2 = bvhvec3(verts[i2]) - vertex0;
	const bvhvec3 directionCrossEdge2 = tinybvh_cross(ray.D, edge2);
	const float determinant = tinybvh_dot(edge1, directionCrossEdge2);
	if (determinant == 0.0f) return;
	const float inverseDeterminant = 1.0f / determinant;
	const bvhvec3 originOffset = ray.O - vertex0;
	const float baryU = inverseDeterminant * tinybvh_dot(originOffset, directionCrossEdge2);
	const bvhvec3 offsetCrossEdge1 = tinybvh_cross(originOffset, edge1);
	const float baryV = inverseDeterminant * tinybvh_dot(ray.D, offsetCrossEdge1);
	if (!(baryU >= 0.0f && baryV >= 0.0f && baryU + baryV <= 1.0f)) return;
	const float distance = inverseDeterminant * tinybvh_dot(edge2, offsetCrossEdge1);
	if (!(distance >= 0.0f && distance <= ray.hit.t)) return;
	ray.hit.t = distance, ray.hit.u = baryU, ray.hit.v = baryV;
	ray.SetHitPrim(triIdx);
}

// Same signature and behaviour as BVHBase::TriOccludes
static bool TriOccludes(const Ray& ray, const bvhvec4slice& verts, const uint32_t triIdx,
	const uint32_t i0, const uint32_t i1, const uint32_t i2)
{
	(void)triIdx; // only needed for opacity maps
	const bvhvec3 vertex0 = bvhvec3(verts[i0]);
	const bvhvec3 edge1 = bvhvec3(verts[i1]) - vertex0, edge2 = bvhvec3(verts[i2]) - vertex0;
	const bvhvec3 directionCrossEdge2 = tinybvh_cross(ray.D, edge2);
	const float determinant = tinybvh_dot(edge1, directionCrossEdge2);
	if (determinant == 0.0f) return false;
	const float inverseDeterminant = 1.0f / determinant;
	const bvhvec3 originOffset = ray.O - vertex0;
	const float baryU = inverseDeterminant * tinybvh_dot(originOffset, directionCrossEdge2);
	const bvhvec3 offsetCrossEdge1 = tinybvh_cross(originOffset, edge1);
	const float baryV = inverseDeterminant * tinybvh_dot(ray.D, offsetCrossEdge1);
	if (!(baryU >= 0.0f && baryV >= 0.0f && baryU + baryV <= 1.0f)) return false;
	const float distance = inverseDeterminant * tinybvh_dot(edge2, offsetCrossEdge1);
	return distance >= 0.0f && distance <= ray.hit.t;
}

// Copy of DOBB Intersect with instrumentation
void IntersectionInstrumented(const BVH_DOBB& bvh, Ray& ray, TraversalStats* stats = nullptr) {
	if (stats) stats->rayCount++;

	BVH_DOBB::StackEntry nodeStack[512]; uint32_t stackIdx = 0;
	nodeStack[stackIdx++] = { 0, -BVH_FAR };

	while (stackIdx > 0) {
		// Early exit if a closer hit was already found
		const BVH_DOBB::StackEntry entry = nodeStack[--stackIdx];
		if (entry.entryDistance >= ray.hit.t) continue;

		if (stats) stats->nodeVisits++;

		const BVH_DOBB::BVHNode& currNode = bvh.dobbNode[entry.nodeIdx];

		Ray localRay;
		localRay.hit.t = ray.hit.t;
		const uint32_t rotationIdx = currNode.Rotation();

		if (rotationIdx != 255) {
			const DOBB_LUT::Mat3f rotation = bvh.getLutEntry(rotationIdx);
			const bvhvec3 rotationX = bvhvec3(rotation.m[0][0], rotation.m[1][0], rotation.m[2][0]);
			const bvhvec3 rotationY = bvhvec3(rotation.m[0][1], rotation.m[1][1], rotation.m[2][1]);
			const bvhvec3 rotationZ = bvhvec3(rotation.m[0][2], rotation.m[1][2], rotation.m[2][2]);

			localRay.O = bvhvec3(tinybvh_dot(rotationX, ray.O), tinybvh_dot(rotationY, ray.O), tinybvh_dot(rotationZ, ray.O));
			localRay.D = bvhvec3(tinybvh_dot(rotationX, ray.D), tinybvh_dot(rotationY, ray.D), tinybvh_dot(rotationZ, ray.D));
		}
		else {
			localRay.O = ray.O; localRay.D = ray.D;
		}
		localRay.rD = tinybvh_rcp(localRay.D);

		float stepSize[3] = {ldexpf(1.0f, currNode.exponent[0]), ldexpf(1.0f, currNode.exponent[1]), ldexpf(1.0f, currNode.exponent[2]),};

		float slotDistance[8]; uint32_t sortedSlots[8]; uint32_t sortedCount = 0;
		uint32_t remainingSlots = currNode.OccupiedMask();

		while (remainingSlots) {
			if (stats) stats->boxTests++;

			// Determine which is the next slot to process
			uint32_t slot = (uint32_t)std::countr_zero(remainingSlots);
			remainingSlots &= remainingSlots - 1;

			// Determine lower and higher extents of the BB
			bvhvec3 slotMinPoint = currNode.nodeMinPoint + bvhvec3(currNode.qLo[0][slot] * stepSize[0], currNode.qLo[1][slot] * stepSize[1], currNode.qLo[2][slot] * stepSize[2]);
			bvhvec3 slotMaxPoint = currNode.nodeMinPoint + bvhvec3(currNode.qHi[0][slot] * stepSize[0], currNode.qHi[1][slot] * stepSize[1], currNode.qHi[2][slot] * stepSize[2]);

			float entryDistance = tinybvh_intersect_aabb(localRay, slotMinPoint, slotMaxPoint);
			if (entryDistance < BVH_FAR) {
				slotDistance[slot] = entryDistance;
				// Insertion sort
				uint32_t insertPos = sortedCount++;
				while (insertPos > 0 && slotDistance[sortedSlots[insertPos - 1]] > entryDistance) {
					sortedSlots[insertPos] = sortedSlots[insertPos - 1];
					insertPos--;
				}
				sortedSlots[insertPos] = slot;
			}
		}

		// Process leaf hits
		for (uint32_t sortedIdx = 0; sortedIdx < sortedCount; sortedIdx++) {
			const uint32_t slot = sortedSlots[sortedIdx];
			if (currNode.iMask & (1u << slot)) continue;
			if (slotDistance[slot] >= ray.hit.t) break;   // sorted: all remaining are farther

			// Intersect triangles
			const uint32_t firstTri = currNode.triBaseIndex + (currNode.meta[slot] & 31);
			const uint32_t triCount = __popc(currNode.meta[slot] >> 5);

			uint32_t i0, i1, i2;
			for (uint32_t triIdx = 0; triIdx < triCount; triIdx++) {
				if (stats) stats->triTests++;

				const uint32_t primIdx = bvh.dobbPrimIdx[firstTri + triIdx];
				GET_PRIM_INDICES_I0_I1_I2(bvh.bvh8.bvh, primIdx);
				IntersectTri(ray, primIdx, bvh.bvh8.bvh.verts, i0, i1, i2);
			}
		}

		// Process internal hits
		for (uint32_t sortedIdx = sortedCount; sortedIdx-- > 0;) {
			const uint32_t slot = sortedSlots[sortedIdx];
			if (!(currNode.iMask & (1u << slot))) continue;
			if (slotDistance[slot] >= ray.hit.t) continue;   // behind a leaf hit from this node

			// Push internal node
			nodeStack[stackIdx++] = {
					currNode.ChildBaseIndex() + __popc(currNode.iMask & ((1u << slot) - 1)),
					slotDistance[slot]
			};
		}
	}
}

// Copy of DOBB isOccluded with instrumentation
bool isOccludedInstrumented(const BVH_DOBB& bvh, const Ray& ray, TraversalStats* stats) {
	if (stats) stats->rayCount++;
	
	uint32_t nodeStack[512];
	uint32_t stackIdx = 0;
	nodeStack[stackIdx++] = 0;

	while (stackIdx > 0) {
		if (stats) stats->nodeVisits++;

		const uint32_t currNodeIdx = nodeStack[--stackIdx];
		const BVH_DOBB::BVHNode& currNode = bvh.dobbNode[currNodeIdx];

		Ray localRay;
		localRay.hit.t = ray.hit.t;
		const uint32_t rotationIdx = currNode.Rotation();

		if (rotationIdx != 255) {
			const DOBB_LUT::Mat3f& rotation = bvh.getLutEntry(rotationIdx);

			const bvhvec3 rotationX = bvhvec3(rotation.m[0][0], rotation.m[1][0], rotation.m[2][0]);
			const bvhvec3 rotationY = bvhvec3(rotation.m[0][1], rotation.m[1][1], rotation.m[2][1]);
			const bvhvec3 rotationZ = bvhvec3(rotation.m[0][2], rotation.m[1][2], rotation.m[2][2]);

			localRay.O = bvhvec3(tinybvh_dot(rotationX, ray.O), tinybvh_dot(rotationY, ray.O), tinybvh_dot(rotationZ, ray.O));
			localRay.D = bvhvec3(tinybvh_dot(rotationX, ray.D), tinybvh_dot(rotationY, ray.D), tinybvh_dot(rotationZ, ray.D));
		}
		else {
			localRay.O = ray.O; localRay.D = ray.D;
		}
		localRay.rD = tinybvh_rcp(localRay.D);

		float stepSize[3] = {ldexpf(1.0f, currNode.exponent[0]), ldexpf(1.0f, currNode.exponent[1]), ldexpf(1.0f, currNode.exponent[2])};

		uint32_t hitMask = 0;
		uint32_t remainingSlots = currNode.OccupiedMask();

		while (remainingSlots) {
			if (stats) stats->boxTests++;

			// Determine which is the next slot to process
			uint32_t slot = (uint32_t)std::countr_zero(remainingSlots);
			remainingSlots &= remainingSlots - 1;

			// Determine lower and higher extents of the BB
			bvhvec3 slotMinPoint = currNode.nodeMinPoint + bvhvec3(currNode.qLo[0][slot] * stepSize[0], currNode.qLo[1][slot] * stepSize[1], currNode.qLo[2][slot] * stepSize[2]);
			bvhvec3 slotMaxPoint = currNode.nodeMinPoint + bvhvec3(currNode.qHi[0][slot] * stepSize[0], currNode.qHi[1][slot] * stepSize[1], currNode.qHi[2][slot] * stepSize[2]);

			float entryDistance = tinybvh_intersect_aabb(localRay, slotMinPoint, slotMaxPoint);
			if (entryDistance < BVH_FAR) 
				hitMask |= 1 << slot;
		}

		uint8_t leafHitMask = hitMask & ~currNode.iMask;
		uint8_t internalHitMask = hitMask & currNode.iMask;

		// Process leaf hits
		while (leafHitMask) {
			const uint32_t slot = (uint32_t)std::countr_zero(leafHitMask);
			leafHitMask &= leafHitMask - 1;

			// Intersect triangles
			const uint32_t firstTri = currNode.triBaseIndex + (currNode.meta[slot] & 31);
			const uint32_t triCount = __popc(currNode.meta[slot] >> 5);

			uint32_t i0, i1, i2;
			for (uint32_t triIdx = 0; triIdx < triCount; triIdx++) {
				if (stats) stats->triTests++;

				const uint32_t primIdx = bvh.dobbPrimIdx[firstTri + triIdx];
				GET_PRIM_INDICES_I0_I1_I2(bvh.bvh8.bvh, primIdx);
				if (TriOccludes(ray, bvh.bvh8.bvh.verts, primIdx, i0, i1, i2))
					return true;
			}
		}

		// Process internal hits
		while (internalHitMask) {
			const uint32_t slot = (uint32_t)std::countr_zero(internalHitMask);
			internalHitMask &= internalHitMask - 1;

			// Push internal node
			nodeStack[stackIdx++] = currNode.ChildBaseIndex() +
				__popc(currNode.iMask & ((1u << slot) - 1));
		}
	}

	return false;
}

// MBVH Intersect matching with DOBB processing order
void IntersectionInstrumented(const MBVH<8>& bvh8, Ray& ray, TraversalStats* stats) {
	if (stats) stats->rayCount++;

	BVH_DOBB::StackEntry nodeStack[512];
	uint32_t stackIdx = 0;
	nodeStack[stackIdx++] = { 0, -BVH_FAR };

	while (stackIdx > 0) {
		const BVH_DOBB::StackEntry entry = nodeStack[--stackIdx];
		if (entry.entryDistance >= ray.hit.t) continue;

		const MBVH<8>::MBVHNode& currNode = bvh8.mbvhNode[entry.nodeIdx];

		if (currNode.isLeaf()) {
			uint32_t i0, i1, i2;
			for (uint32_t triOffset = 0; triOffset < currNode.triCount; triOffset++) {
				if (stats) stats->triTests++;

				const uint32_t primIdx = bvh8.bvh.primIdx[currNode.firstTri + triOffset];
				GET_PRIM_INDICES_I0_I1_I2(bvh8.bvh, primIdx);
				IntersectTri(ray, primIdx, bvh8.bvh.verts, i0, i1, i2);
			}
		}
		else {
			// Leaves are not nodes in DOBB
			if (stats) stats->nodeVisits++;

			float slotDistance[8]; uint32_t sortedSlots[8]; uint32_t sortedCount = 0;

			for (uint32_t slot = 0; slot < 8; slot++) {
				if (currNode.child[slot] == 0) continue;
				if (stats) stats->boxTests++;

				const MBVH<8>::MBVHNode& child = bvh8.mbvhNode[currNode.child[slot]];
				const float entryDistance = tinybvh_intersect_aabb(ray, child.aabbMin, child.aabbMax);
				if (entryDistance < BVH_FAR) {
					slotDistance[slot] = entryDistance;
					// Insertion sort
					uint32_t insertPos = sortedCount++;
					while (insertPos > 0 && slotDistance[sortedSlots[insertPos - 1]] > entryDistance) {
						sortedSlots[insertPos] = sortedSlots[insertPos - 1];
						insertPos--;
					}
					sortedSlots[insertPos] = slot;
				}
			}

			// Push internal children first, farthest first
			for (uint32_t sortedIdx = sortedCount; sortedIdx-- > 0;) {
				const uint32_t slot = sortedSlots[sortedIdx];
				if (bvh8.mbvhNode[currNode.child[slot]].isLeaf()) continue;
				nodeStack[stackIdx++] = { currNode.child[slot], slotDistance[slot] };
			}

			// Push leaves on top, farthest first, so they pop before the internal children, nearest first
			for (uint32_t sortedIdx = sortedCount; sortedIdx-- > 0;) {
				const uint32_t slot = sortedSlots[sortedIdx];
				if (!bvh8.mbvhNode[currNode.child[slot]].isLeaf()) continue;
				nodeStack[stackIdx++] = { currNode.child[slot], slotDistance[slot] };
			}
		}
	}
}

// MBVH isOccluded matching with DOBB processing order
bool isOccludedInstrumented(const MBVH <8>& bvh8, const Ray& ray, TraversalStats* stats) {
	if (stats) stats->rayCount++;

	Ray boxRay = ray;

	uint32_t nodeStack[512];
	uint32_t stackIdx = 0;
	nodeStack[stackIdx++] = 0;

	while (stackIdx > 0) {
		const MBVH<8>::MBVHNode& currNode = bvh8.mbvhNode[nodeStack[--stackIdx]];

		if (currNode.isLeaf()) {
			uint32_t i0, i1, i2;
			for (uint32_t triOffset = 0; triOffset < currNode.triCount; triOffset++) {
				if (stats) stats->triTests++;

				const uint32_t primIdx = bvh8.bvh.primIdx[currNode.firstTri + triOffset];
				GET_PRIM_INDICES_I0_I1_I2(bvh8.bvh, primIdx);
				if (TriOccludes(ray, bvh8.bvh.verts, primIdx, i0, i1, i2))
					return true;
			}
		}
		else {
			// Leaves are not nodes in DOBB
			if (stats) stats->nodeVisits++;

			// Make sure nodes are traversed w the same logic as DOBB
			bool childHit[8] = {};
			for (uint32_t slot = 0; slot < 8; slot++) {
				if (currNode.child[slot] != 0) {
					if (stats) stats->boxTests++;

					const MBVH<8>::MBVHNode& child = bvh8.mbvhNode[currNode.child[slot]];
					childHit[slot] = tinybvh_intersect_aabb(boxRay, child.aabbMin, child.aabbMax) < BVH_FAR;;
				}
			}

			for (uint32_t slot = 0; slot < 8; slot++)
				if (childHit[slot] && !bvh8.mbvhNode[currNode.child[slot]].isLeaf())
					nodeStack[stackIdx++] = currNode.child[slot];

			for (uint32_t slot = 0; slot < 8; slot++)
				if (childHit[slot] && bvh8.mbvhNode[currNode.child[slot]].isLeaf())
					nodeStack[stackIdx++] = currNode.child[slot];
		}
	}

	return false;
}

void printStats(const char* tableName) {
	std::cout << std::fixed << std::setprecision(3);

	TraversalStats allStats;
	allStats.add(primHitStats); allStats.add(secondHitStats); allStats.add(shadowStats);

	// Guards against division by 0
	const double guardedPrimRayCount = primHitStats.rayCount ? (double)primHitStats.rayCount : 1.0f;
	const double guardedSecondRayCount = secondHitStats.rayCount ? (double)secondHitStats.rayCount : 1.0f;
	const double guardedShadowRayCount = shadowStats.rayCount ? (double)shadowStats.rayCount : 1.0f;
	const double guardedAllRayCount = allStats.rayCount ? (double)allStats.rayCount : 1.0f;

	const int firstColWidth = 35, colWidth = 15;

	std::cout << std::left << std::setw(firstColWidth) << tableName << std::right
		<< " | " << std::setw(colWidth) << "Primary Rays"
		<< " | " << std::setw(colWidth) << "Secondary Rays"
		<< " | " << std::setw(colWidth) << "Shadow Rays"
		<< " | " << std::setw(colWidth) << "All Rays" << " | " << std::endl;
	std::cout << std::left << std::setw(firstColWidth) << "Number of rays:" << std::right
		<< " | " << std::setw(colWidth) << primHitStats.rayCount
		<< " | " << std::setw(colWidth) << secondHitStats.rayCount
		<< " | " << std::setw(colWidth) << shadowStats.rayCount
		<< " | " << std::setw(colWidth) << allStats.rayCount << " | " << std::endl;
	std::cout << std::left << std::setw(firstColWidth) << "Avg nodes visited per ray:" << std::right
		<< " | " << std::setw(colWidth) << primHitStats.nodeVisits / guardedPrimRayCount
		<< " | " << std::setw(colWidth) << secondHitStats.nodeVisits / guardedSecondRayCount
		<< " | " << std::setw(colWidth) << shadowStats.nodeVisits / guardedShadowRayCount
		<< " | " << std::setw(colWidth) << allStats.nodeVisits / guardedAllRayCount << " | " << std::endl;
	std::cout << std::left << std::setw(firstColWidth) << "Avg boxes intersected per ray:" << std::right
		<< " | " << std::setw(colWidth) << primHitStats.boxTests / guardedPrimRayCount
		<< " | " << std::setw(colWidth) << secondHitStats.boxTests / guardedSecondRayCount
		<< " | " << std::setw(colWidth) << shadowStats.boxTests / guardedShadowRayCount
		<< " | " << std::setw(colWidth) << allStats.boxTests / guardedAllRayCount << " | " << std::endl;
	std::cout << std::left << std::setw(firstColWidth) << "Avg triangles intersected per ray:" << std::right
		<< " | " << std::setw(colWidth) << primHitStats.triTests / guardedPrimRayCount
		<< " | " << std::setw(colWidth) << secondHitStats.triTests / guardedSecondRayCount
		<< " | " << std::setw(colWidth) << shadowStats.triTests / guardedShadowRayCount
		<< " | " << std::setw(colWidth) << allStats.triTests / guardedAllRayCount << " | " << std::endl << std::endl;
}

// ---------------------- RENDERING ----------------------
// Multi-threading
static unsigned threadCount = std::thread::hardware_concurrency();

// Xor32 RNG
static unsigned RandomUInt(unsigned& seed) { seed ^= seed << 13, seed ^= seed >> 17, seed ^= seed << 5; return seed; }
static float RandomFloat(unsigned& seed) { return RandomUInt(seed) * 2.3283064365387e-10f; }

// Ray tracing math
bvhvec3 DiffuseReflection(const bvhvec3 N, unsigned& seed)
{
	bvhvec3 R;
	do
	{
		R = bvhvec3(RandomFloat(seed) * 2 - 1, RandomFloat(seed) * 2 - 1, RandomFloat(seed) * 2 - 1);
	} while (tinybvh_dot(R, R) > 1);
	return tinybvh_normalize(tinybvh_dot(R, N) < 0 ? R : -R);
}
bvhvec3 CosWeightedDiffReflection(const bvhvec3 N, unsigned& seed)
{
	bvhvec3 R = DiffuseReflection(N, seed);
	return tinybvh_normalize(N + R);
}

// Color conversion
bvhvec3 rgb32_to_vec3(const unsigned c)
{
	return bvhvec3((float)(c >> 16), (float)((c >> 8) & 255), (float)(c & 255)) * (1 / 255.f);
}

// Geometry access
bvhvec3 TriangleColor(const unsigned idx) { return rgb32_to_vec3(*(unsigned*)&tris[idx * 3].w); }
bvhvec3 TriangleNormal(const unsigned idx)
{
	bvhvec3 a = tris[idx * 3], b = tris[idx * 3 + 1], c = tris[idx * 3 + 2];
	return tinybvh_normalize(tinybvh_cross(b - a, a - c));
}

// Light transport calculation - Basic recursive Path Tracer with IS and Next Event Estimation
template <class BVHType>
bvhvec3 Trace(const BVHType& bvh, Ray ray, unsigned& seed, uint64_t& traceCost, 
	TraversalStats& threadPrimHitStats, TraversalStats& threadSecondHitStats, TraversalStats& threadShadowStats, unsigned depth = 0)
{
	// find primary intersection
	if (depth == 0) IntersectionInstrumented(bvh, ray, &threadPrimHitStats);
	else			IntersectionInstrumented(bvh, ray, &threadSecondHitStats);
	// shade
	if (ray.hit.t == 1e30f)
		return bvhvec3(0.6f, 0.7f, 1); // hit nothing
	bvhvec3 I = ray.O + ray.hit.t * ray.D;
	bvhvec3 N = TriangleNormal(ray.hit.prim);
	if (tinybvh_dot(N, ray.D) > 0) N = -N;
	bvhvec3 BRDF = TriangleColor(ray.hit.prim) * (1.0f / 3.14159f);
	bvhvec3 Lpos(RandomFloat(seed) * 30 - 15, 40, RandomFloat(seed) * 6 - 3); // virtual
	float dist = tinybvh_length(Lpos - I);
	bvhvec3 L = (Lpos - I) * (1.0f / dist); // normalize
	bvhvec3 direct = {}, indirect = {};
	float NdotL = tinybvh_dot(N, L), NLdotL = fabs(tinybvh_dot(L, bvhvec3(0, 1, 0)));
	if (NdotL > 0)
		if (!isOccludedInstrumented(bvh, Ray(I + L * 0.001f, L, dist), &threadShadowStats))
			direct = BRDF * NdotL * NLdotL * bvhvec3(9, 9, 8) * 500 * (1.0f / (dist * dist));
	// random bounce
	if (depth < 2)
	{
		bvhvec3 R = CosWeightedDiffReflection(N, seed);
		float pdf = 1.0f / tinybvh_dot(N, R);
		bvhvec3 irradiance;
		irradiance = Trace(bvh, Ray(I + R * 0.001f, R), seed, traceCost, threadPrimHitStats, threadSecondHitStats, threadShadowStats, depth + 1);
		indirect = BRDF * irradiance * (1.0f / pdf);
	}
	// finalize
	return direct + indirect;
}

template <class BVHType>
void TraceWorkerThread(const BVHType& bvh, uint32_t* buf, float scale, int threadIdx,
	TraversalStats& threadPrimHitStats, TraversalStats& threadSecondHitStats, TraversalStats& threadShadowStats)
{
	uint64_t localCost = 0;
	const int xtiles = SCRWIDTH / TILESIZE, ytiles = SCRHEIGHT / TILESIZE;
	const int tiles = xtiles * ytiles;
	int tile = threadIdx;
	while (tile < tiles)
	{
		const int tx = tile % xtiles, ty = tile / xtiles;
		unsigned seed = (tile + 17) * 171717 + spp * 1023;
		for (int y = 0; y < TILESIZE; y++) for (int x = 0; x < TILESIZE; x++)
		{
			const int pixel_x = tx * TILESIZE + x, pixel_y = ty * TILESIZE + y;
			const int pixelIdx = pixel_x + pixel_y * SCRWIDTH;
			// setup primary ray
			const float u = (float)pixel_x / SCRWIDTH, v = (float)pixel_y / SCRHEIGHT;
			const bvhvec3 D = tinybvh_normalize(cam.p1 + u * (cam.p2 - cam.p1) + v * (cam.p3 - cam.p1) - cam.eye);
			// trace
			const TraversalStats primHitBefore = threadPrimHitStats, secondHitBefore = threadSecondHitStats, shadowBefore = threadShadowStats;
			Trace(bvh, Ray(cam.eye, D), seed, localCost, threadPrimHitStats, threadSecondHitStats, threadShadowStats);
			// red: total node visits; green: total triangle tests; blue: total box tests
			accumulator[pixelIdx] += bvhvec3(
				nodeVisitScale * 
					((threadPrimHitStats.nodeVisits - primHitBefore.nodeVisits) + 
					(threadSecondHitStats.nodeVisits - secondHitBefore.nodeVisits) + 
					(threadShadowStats.nodeVisits - shadowBefore.nodeVisits)),
				triTestScale * 
					((threadPrimHitStats.triTests - primHitBefore.triTests) + 
					(threadSecondHitStats.triTests - secondHitBefore.triTests) +
					(threadShadowStats.triTests - shadowBefore.triTests)),
				boxTestScale *
					((threadPrimHitStats.boxTests - primHitBefore.boxTests) +
					(threadSecondHitStats.boxTests - secondHitBefore.boxTests) +
					(threadShadowStats.boxTests - shadowBefore.boxTests))
			);
			const bvhvec3 E = accumulator[pixelIdx] * scale;
			// visualize, with a poor man's gamma correct
			const int r = (int)tinybvh_min(255.0f, sqrtf(E.x) * 255.0f);
			const int g = (int)tinybvh_min(255.0f, sqrtf(E.y) * 255.0f);
			const int b = (int)tinybvh_min(255.0f, sqrtf(E.z) * 255.0f);
			buf[pixelIdx] = b + (g << 8) + (r << 16);
		}
		tile = tileIdx++;
	}
}

// ---------------------- LOAD DATA ----------------------

// Scene management - Append a file, with optional position, scale and color override, tinyfied
void AddMesh(const char* file, float scale = 1, bvhvec3 pos = {}, int c = 0,
	bvhvec3 rotationAxis = bvhvec3(0, 1, 0), float rotationDegrees = 0.0f)
{
	std::fstream s{ file, s.binary | s.in };
	int N = 0;
	s.read((char*)&N, 4);
	bvhvec4* data = (bvhvec4*)malloc64((N + triCount) * 48);
	if (tris) memcpy(data, tris, triCount * 48), free64(tris);
	tris = data, s.read((char*)tris + triCount * 48, N * 48), triCount += N;
	
	rotationAxis = tinybvh_normalize(rotationAxis);
	const float angle = rotationDegrees * (3.14159265f / 180.0f);
	const float cosAngle = cosf(angle);
	const float sinAngle = sinf(angle);

	for (int vertexIdx = (triCount - N) * 3; vertexIdx < triCount * 3; vertexIdx++) {
		bvhvec4& vertex = tris[vertexIdx];
		// Scale vertex position
		bvhvec3 vertPos = bvhvec3(vertex) * scale;
		// Rotate vertex position 
		vertPos = vertPos * cosAngle
			+ tinybvh_cross(rotationAxis, vertPos) * sinAngle
			+ rotationAxis * (tinybvh_dot(rotationAxis, vertPos) * (1.0f - cosAngle));
		// Add pos, to get world position
		vertPos = vertPos + pos;

		vertex.x = vertPos.x;
		vertex.y = vertPos.y;
		vertex.z = vertPos.z;
		if (c) *(int*)&vertex.w = c;
	}
}

void ClearMesh()
{
	if (tris)
	{
		free64(tris);
		tris = 0;
	}

	triCount = 0;
}

// Places cameras on a sphere centered in the middle of a mesh
void GenerateViewPoints(float viewPointRadiusScale = 1.0f) {
	// Find Bounding box of all vertices (DOBB only fits loosely) for the radius
	bvhvec3 minBounds(BVH_FAR, BVH_FAR, BVH_FAR), maxBounds(-BVH_FAR, -BVH_FAR, -BVH_FAR);
	// Get average of vertices for center of "viewpoint sphere"
	double vertexSumX = 0.0f, vertexSumY = 0.0f, vertexSumZ = 0.0f;
	const int vertCount = triCount * 3;
	for (int vertIdx = 0; vertIdx < vertCount; vertIdx++) {
		const bvhvec3 vert = bvhvec3(tris[vertIdx]);

		minBounds = tinybvh_min(minBounds, vert);
		maxBounds = tinybvh_max(maxBounds, vert);

		vertexSumX += vert.x;
		vertexSumY += vert.y;
		vertexSumZ += vert.z;
	}

	const float camDist = tinybvh_length(maxBounds - minBounds) * 0.5f * viewPointRadiusScale;
	const bvhvec3 center(
		(float)(vertexSumX / vertCount),
		(float)(vertexSumY / vertCount),
		(float)(vertexSumZ / vertCount)
	);

	// Evenly spaced cameras (fibonacci sphere) via Claude
	for (uint32_t pointIdx = 0; pointIdx < NUMBEROFVIEWPOINTS; pointIdx++) {
		const float height = 1.0f - 2.0f * (pointIdx + 0.5f) / NUMBEROFVIEWPOINTS;
		const float ringRadius = sqrtf(1.0f - height * height);
		const float azimuth = pointIdx * 3.14159265f * (3.0f - sqrtf(5.0f)); // golden angle
		const bvhvec3 direction(cosf(azimuth) * ringRadius, height, sinf(azimuth) * ringRadius);
	
		viewPointCameras[pointIdx].initCamera(center + (direction * camDist), -direction);
	}
}

// ---------------------- BUILD BVH ----------------------

// Convert MBVH8 so it mimics compressed node behavior
void convertMBVHToCompressedBounds() {
	for (uint32_t nodeIdx = 0; nodeIdx < dobbBvh.bvh8.usedNodes; nodeIdx++) {
		const MBVH<8>::MBVHNode& currNode = dobbBvh.bvh8.mbvhNode[nodeIdx];

		if (!currNode.isLeaf()) {
			// Find min and max points for parent node
			bvhvec3 minPoint{ BVH_FAR, BVH_FAR, BVH_FAR };
			bvhvec3 maxPoint{ -BVH_FAR, -BVH_FAR, -BVH_FAR };

			for (uint32_t childIdx = 0; childIdx < currNode.childCount; childIdx++) {
				const MBVH <8>::MBVHNode& childNode = dobbBvh.bvh8.mbvhNode[currNode.child[childIdx]];

				minPoint = tinybvh_min(minPoint, childNode.aabbMin);
				maxPoint = tinybvh_max(maxPoint, childNode.aabbMax);
			}

			// Quantize extent encompassed by min and max points
			float stepSize[3], inverseScale[3];
			for (uint32_t axisIdx = 0; axisIdx < 3; axisIdx++) {
				// Using same code as in ConvertFrom
				const float extent = maxPoint[axisIdx] - minPoint[axisIdx];

				int32_t exponent = extent > 0.0f ? (int32_t)ceilf(log2f(extent / 255.0f)) : -126;
				exponent = tinybvh_clamp(exponent, -126, 127);

				if (exponent < 127 && ldexpf(255.0f, exponent) < extent)
					exponent++;

				// Calculate stepsize and inverse stepsize
				stepSize[axisIdx] = ldexpf(1.0, exponent);
				inverseScale[axisIdx] = ldexpf(1.0f, -exponent);
			}

			// Calculate quantized AABBs for MBVH
			for (uint32_t childIdx = 0; childIdx < currNode.childCount; childIdx++) {
				MBVH<8>::MBVHNode& childNode = dobbBvh.bvh8.mbvhNode[currNode.child[childIdx]];

				for (uint32_t axisIdx = 0; axisIdx < 3; axisIdx++) {
					const uint8_t low = (uint8_t)tinybvh_clamp(
						floorf((childNode.aabbMin[axisIdx] - minPoint[axisIdx]) * inverseScale[axisIdx]),
						0.0f, 255.0f);
					const uint8_t high = (uint8_t)tinybvh_clamp(
						ceilf((childNode.aabbMax[axisIdx] - minPoint[axisIdx]) * inverseScale[axisIdx]),
						0.0f, 255.0f);

					childNode.aabbMin[axisIdx] = minPoint[axisIdx] + low * stepSize[axisIdx];
					childNode.aabbMax[axisIdx] = minPoint[axisIdx] + high * stepSize[axisIdx];
				}
			}
		}
	}
}

// Build BVHs and then set their c_trav and c_int values to 1.0
// to measure BVH quality
void InitBvhs() 
{
	dobbBvh.Build(tris, triCount);
	convertMBVHToCompressedBounds();
}

// ---------------------- MISC ----------------------

void Init() {
#ifdef _WIN32
	AllocConsole();
	FILE* dummy;
	freopen_s(&dummy, "CONOUT$", "w", stdout);
	freopen_s(&dummy, "CONOUT$", "w", stderr);
	std::cout.clear();
	std::cerr.clear();
#endif
}

template <class BVHType>
void RenderTick(const BVHType& bvh, float delta_time_s, fenster& f, uint32_t* buf)
{
	// render tiles
	const float scale = 1.0f / ++spp;
	tileIdx = threadCount;
	std::vector<TraversalStats> threadPrimHitStats(threadCount), threadSecondHitStats(threadCount), threadShadowStats(threadCount);
	std::vector<std::thread> threads;
	for (uint32_t i = 0; i < threadCount; i++)
		threads.emplace_back(TraceWorkerThread<BVHType>, std::cref(bvh), buf, scale, i,
			std::ref(threadPrimHitStats[i]), std::ref(threadSecondHitStats[i]), std::ref(threadShadowStats[i]));
	for (auto& thread : threads) thread.join();
	for (uint32_t i = 0; i < threadCount; i++)
		primHitStats.add(threadPrimHitStats[i]), secondHitStats.add(threadSecondHitStats[i]), shadowStats.add(threadShadowStats[i]);
	// Create title
	const char* bvhName = "-";
	if (phase == RENDERDOBB) bvhName = "DOBB BVH";
	if (phase == RENDERMBVH) bvhName = "MBVH 8";
	char title[80];
	snprintf(
		title, sizeof(title), 
		"%s, %s, view %d/%d spp: %d", 
		bvhName, scenes[sceneIdx].sceneName, 
		viewPointIdx + 1, NUMBEROFVIEWPOINTS, spp
	);
	fenster_update_title(&f, title);
}

void resetStats() {
	primHitStats.reset(); secondHitStats.reset(); shadowStats.reset();
}

void initViewPoint() {
	cam = viewPointCameras[viewPointIdx];
	
	memset(accumulator, 0, sizeof(accumulator));
	spp = 0;
}

void Tick(float delta_time_s, fenster& f, uint32_t* buf)
{
	switch (phase) {
	case BUILD: {
		const SceneSetup& scene = scenes[sceneIdx];

		// Reset and load next mesh to be tested
		std::cout << "-------------------------------------------------" << std::endl;
		std::cout << "Loading scene: " << scene.sceneName << std::endl;
		ClearMesh();
		if (scene.meshPath[0]) AddMesh(scene.meshPath[0], 1, bvhvec3(0), 0xffffff);
		if (scene.meshPath[1]) AddMesh(scene.meshPath[1], 1, bvhvec3(0), 0xffffff);
		GenerateViewPoints(scene.viewPointScale);

		// Build BVHs 
		std::cout << "Building BVHs for " << triCount << " triangles" << std::endl;
		InitBvhs();

		// Print stats of BVHs
		printMBVHSAH("MBVH8 Nodes");
		printDOBBSAHAndStats("DOBB Nodes");

		// Init and swap to the next phase
		viewPointIdx = 0;
		resetStats();
		initViewPoint();

		phase = RENDERMBVH;
		break;
	}
	case RENDERMBVH: {
		RenderTick(dobbBvh.bvh8, delta_time_s, f, buf);
		if (spp >= targetSpp)
		{
			if (++viewPointIdx < NUMBEROFVIEWPOINTS) {
				initViewPoint();
			}
			else {
				printStats((std::string("MBVH8 BVH, scene: ") + scenes[sceneIdx].sceneName).c_str());
				
				// Init and swap to the next phase
				viewPointIdx = 0;
				resetStats();
				initViewPoint();

				phase = RENDERDOBB;
			}
		}
		break;
	}
	case RENDERDOBB: {
		RenderTick(dobbBvh, delta_time_s, f, buf);
		if (spp >= targetSpp)
		{
			if (++viewPointIdx < NUMBEROFVIEWPOINTS) {
				initViewPoint();
			}
			else {
				printStats((std::string("DOBB BVH, scene: ") + scenes[sceneIdx].sceneName).c_str());

				phase = BUILD;
				sceneIdx++;
				if (sceneIdx >= sceneCount)
				{
					phase = END;
					std::cout << "END" << std::endl;
				}
			}
		}
		break;
	}
	case END: { break; }
	default: { break; }
	}
}

// Application Shutdown
void Shutdown() {
#ifdef _WIN32
	FreeConsole();
#endif
}