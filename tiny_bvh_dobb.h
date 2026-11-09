#ifndef TINY_BVH_DOBB_H_
#define TINY_BVH_DOBB_H_

#ifndef TINY_BVH_H_
#include "tiny_bvh.h"
#endif

#define AXESNO 13
#define SMALLM 4
#define MAXROTATIONVALUES 44
#define ALPHA 0.9f

#ifndef FATAL_ERROR_IF
#define FATAL_ERROR_IF(condition, message) if (condition) { fprintf( stderr, "Fatal error in tiny_bvh_dobb.h, line %i:\n%s\n", __LINE__, message ); exit( 1 ); }
#endif

#include <iostream>
#include <stdio.h>

namespace tinybvh {
	static const bvhvec3 proxyKDopAxes[AXESNO] = {
		// 3 Euclidian axes
		bvhvec3(1.0f, 0.0f, 0.0f),
		bvhvec3(0.0f, 1.0f, 0.0f),
		bvhvec3(0.0f, 0.0f, 1.0f),
		// 6 diagonal axes
		tinybvh_normalize(bvhvec3(1.0f, 1.0f, 0.0f)),
		tinybvh_normalize(bvhvec3(1.0f, -1.0f, 0.0f)),
		tinybvh_normalize(bvhvec3(1.0f, 0.0f, 1.0f)),
		tinybvh_normalize(bvhvec3(1.0f, 0.0f, -1.0f)),
		tinybvh_normalize(bvhvec3(0.0f, 1.0f, 1.0f)),
		tinybvh_normalize(bvhvec3(0.0f, 1.0f, -1.0f)),
		// 4 "axes in 3D space"
		// pointing towards the vertices of the basis cube
		tinybvh_normalize(bvhvec3(1.0f, 1.0f, 1.0f)),
		tinybvh_normalize(bvhvec3(1.0f, 1.0f, -1.0f)),
		tinybvh_normalize(bvhvec3(1.0f, -1.0f, 1.0f)),
		tinybvh_normalize(bvhvec3(1.0f, -1.0f, -1.0f)),
	};

	// Only implementing |K| = 13 case
	struct dobbSidebandData {
		// Index corresponds to axes
		bvhvec2 kDopExtents[AXESNO];
		uint8_t rotation;
		float AABBSASum;
	};

	class DOBB_LUT {
	public: 
		static constexpr float PI = 3.14159265358979f;
		static constexpr uint32_t AZIMUTH_BINS = 32, POLAR_BINS = 16, ANGLE_BINS = 16;

		struct Mat3f { float m[3][3]; };
		struct Mat3i { uint8_t m[3][3]; };

		struct reprojectionEntry {
			uint8_t axisIdx[3];
			int sign[3];
		};

		float rotationValues[MAXROTATIONVALUES];
		Mat3i rotations[AXESNO * SMALLM * 2];
		reprojectionEntry reprojections[AXESNO * SMALLM * 2][6];
		uint8_t azimuthAngles[AZIMUTH_BINS * POLAR_BINS * ANGLE_BINS];

		DOBB_LUT() { 
			buildLUT(); 
			buildReprojectionLUT();
			buildAxisAzimuthLUT();
		}

		Mat3f getRawEntry(uint32_t rotIdx) {
			Mat3f ret;
			Mat3i rotMat = rotations[rotIdx];

			ret.m[0][0] = rotationValues[rotMat.m[0][0]];
			ret.m[0][1] = rotationValues[rotMat.m[0][1]];
			ret.m[0][2] = rotationValues[rotMat.m[0][2]];

			ret.m[1][0] = rotationValues[rotMat.m[1][0]];
			ret.m[1][1] = rotationValues[rotMat.m[1][1]];
			ret.m[1][2] = rotationValues[rotMat.m[1][2]];

			ret.m[2][0] = rotationValues[rotMat.m[2][0]];
			ret.m[2][1] = rotationValues[rotMat.m[2][1]];
			ret.m[2][2] = rotationValues[rotMat.m[2][2]];

			return ret;
		}

		uint8_t getRotationForBasis(const bvhvec3(&basis)[3]) {
			bvhvec3 axis; float angle;
			basisToAxisAngle(basis, axis, angle);
			return azimuthAngles[axisAngleToCell(axis, angle)];
		}

	private:
		// https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
		// axis must be normalized
		static Mat3f rodriguesRotMat(bvhvec3 axis, float angle) {
			const float cs = cosf(angle);
			const float oneMinusCos = 1 - cs;
			const float sn = sinf(angle);
			Mat3f ret;

			ret.m[0][0] = cs + oneMinusCos * axis.x * axis.x;
			ret.m[0][1] = oneMinusCos * axis.x * axis.y - sn * axis.z;
			ret.m[0][2] = oneMinusCos * axis.x * axis.z + sn * axis.y;

			ret.m[1][0] = oneMinusCos * axis.x * axis.y + sn * axis.z;
			ret.m[1][1] = cs + oneMinusCos * axis.y * axis.y;
			ret.m[1][2] = oneMinusCos * axis.y * axis.z - sn * axis.x;

			ret.m[2][0] = oneMinusCos * axis.x * axis.z - sn * axis.y;
			ret.m[2][1] = oneMinusCos * axis.y * axis.z + sn * axis.x;
			ret.m[2][2] = cs + oneMinusCos * axis.z * axis.z;

			return ret;
		}

		void buildLUT() {
			const float delta = 3.14159265358979f / (2.0f * SMALLM);
			uint32_t rotationsIdx = 0;

			Mat3f rawRotations[AXESNO * SMALLM * 2];

			// Calculate raw rotation matrix values
			for (uint32_t axisIdx = 0; axisIdx < AXESNO; axisIdx++) {
				for (int i = 1; i <= SMALLM; i++) {
					rawRotations[rotationsIdx++] = rodriguesRotMat(
						proxyKDopAxes[axisIdx], i * delta);

					rawRotations[rotationsIdx++] = rodriguesRotMat(
						proxyKDopAxes[axisIdx], i * -delta);
				}
			}

			uint8_t rotValUsed = 0;
			for (uint32_t rotIdx = 0; rotIdx < AXESNO * SMALLM * 2; rotIdx++) {
				for (uint32_t rowIdx = 0; rowIdx < 3; rowIdx++) {
					for (uint32_t colIdx = 0; colIdx < 3; colIdx++) {
						bool found = false;
						const float currRawValue = rawRotations[rotIdx].m[rowIdx][colIdx];

						for (uint8_t rotValIdx = 0; rotValIdx < rotValUsed; rotValIdx++) {
							if (fabsf(rotationValues[rotValIdx] - currRawValue) < 0.000001f) {
								found = true;
								rotations[rotIdx].m[rowIdx][colIdx] = rotValIdx;
								break;
							}
						}

						if (!found) {
							rotationValues[rotValUsed] = currRawValue;
							rotations[rotIdx].m[rowIdx][colIdx] = rotValUsed++;
						}
					}
				}
			}
		}

		void buildReprojectionLUT() {
			for (uint32_t rotationIdx = 0; rotationIdx < AXESNO * SMALLM * 2; rotationIdx++) {
				const Mat3f& rotation = getRawEntry(rotationIdx);
				for (uint32_t candidateRotAxisIdx = 0; candidateRotAxisIdx < 3; candidateRotAxisIdx++) {
					const bvhvec3 candidateAxis(
						rotation.m[0][candidateRotAxisIdx],
						rotation.m[1][candidateRotAxisIdx],
						rotation.m[2][candidateRotAxisIdx]
					);

					findQuadSphereTriangle(
						candidateAxis, 
						reprojections[rotationIdx][candidateRotAxisIdx * 2]
					);
					findQuadSphereTriangle(
						-candidateAxis,
						reprojections[rotationIdx][candidateRotAxisIdx * 2 + 1]
					);
				}
			}
		}

		static void basisToAxisAngle(const bvhvec3(&basis)[3], bvhvec3& axis, float& angle) {
			// Find theta angle by "reverse-rodriguez"
			const float trace = basis[0].x + basis[1].y + basis[2].z;
			angle = acosf(tinybvh_clamp((trace - 1.0f) * 0.5f, -1.0f, 1.0f));
			const float sinAngle = sinf(angle);

			if (sinAngle > 1e-4f) {
				// General case: find axis by "reverse-rodriguez"
				axis = tinybvh_normalize(bvhvec3(
					basis[1].z - basis[2].y,
					basis[2].x - basis[0].z,
					basis[0].y - basis[1].x
				));
			}
			else if (angle < 1.0f) {
				// Angle is roughly 0, axis is arbitrary
				axis = bvhvec3(0.0f, 0.0f, 1.0f);
			}
			else {
				// Angle is roughly PI non-general "reverse-rodriguez"
				const float diagonal[3] = { basis[0].x, basis[1].y, basis[2].z };
				uint32_t largestIdx = 0;
				if (diagonal[1] > diagonal[largestIdx]) largestIdx = 1;
				if (diagonal[2] > diagonal[largestIdx]) largestIdx = 2;

				float components[3];
				components[largestIdx] = sqrtf(tinybvh_max((diagonal[largestIdx] + 1.0f) * 0.5f, 0.0f));
				for (uint32_t componentIdx = 0; componentIdx < 3; componentIdx++) {
					if (componentIdx == largestIdx) continue;
					// m[largest][k] + m[k][largest]
					const float symmetricSum = basis[componentIdx][largestIdx] + basis[largestIdx][componentIdx];
					components[componentIdx] = symmetricSum / (4.0f * components[largestIdx]);
				}
				axis = tinybvh_normalize(bvhvec3(components[0], components[1], components[2]));
			}
		}

		static uint32_t toBin(const float fraction, const uint32_t binCount) {
			return (uint32_t)tinybvh_clamp(fraction * binCount, 0.0f, (float)(binCount - 1));
		}

		// Slot an axis, angle combination into our qunatized bins
		static uint32_t axisAngleToCell(const bvhvec3& axis, const float angle) {
			const float azimuth = atan2f(axis.y, axis.x);
			const float polar = acosf(tinybvh_clamp(axis.z, -1.0f, 1.0f));

			const uint32_t azimuthBin = toBin((azimuth + PI) / (2.0f * PI), AZIMUTH_BINS);
			const uint32_t polarBin = toBin(polar / PI, POLAR_BINS);
			const uint32_t angleBin = toBin(angle / PI, ANGLE_BINS);

			return (angleBin * POLAR_BINS + polarBin) * AZIMUTH_BINS + azimuthBin;
		}

		const float similarity(const bvhvec3(&capitalB)[3], const uint32_t rotIdx) {
			const Mat3f rotMat = getRawEntry(rotIdx);
			// Using dot product, set up matrix that we can use to compare
			// all possible permutations of bases
			Mat3f dotMat;

			for (uint32_t i = 0; i < 3; i++) {
				const bvhvec3 r(rotMat.m[0][i], rotMat.m[1][i], rotMat.m[2][i]);

				for (uint32_t j = 0; j < 3; j++)
					dotMat.m[i][j] = fabsf(tinybvh_dot(r, capitalB[j]));
			}

			static const int Perms[6][3] = { {0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0} };
			float maxSim = 0.0f;
			for (uint32_t i = 0; i < 6; i++)
				maxSim = tinybvh_max(
					maxSim,
					dotMat.m[0][Perms[i][0]] +
					dotMat.m[1][Perms[i][1]] +
					dotMat.m[2][Perms[i][2]]
				);

			return maxSim;
		}

		uint8_t bruteForceBestRot(const bvhvec3(&basis)[3]) {
			float bestSim = -1.0f;
			uint8_t bestRot = 0;

			for (uint8_t rotationIdx = 0; rotationIdx < AXESNO * SMALLM * 2; rotationIdx++) {
				float sim = similarity(basis, rotationIdx);
				if (sim > bestSim) {
					bestSim = sim;
					bestRot = rotationIdx;
				}
			}

			return bestRot;
		}

		void buildAxisAzimuthLUT() {
			for (uint32_t angleBin = 0; angleBin < ANGLE_BINS; angleBin++) {
				for (uint32_t polarBin = 0; polarBin < POLAR_BINS; polarBin++) {
					for (uint32_t azimuthBin = 0; azimuthBin < AZIMUTH_BINS; azimuthBin++) {
						// Identify cell
						const float angle = (angleBin + 0.5f) / ANGLE_BINS * PI;
						const float polar = (polarBin + 0.5f) / POLAR_BINS * PI;
						const float azimuth = (azimuthBin + 0.5f) / AZIMUTH_BINS * 2.0f * PI - PI;

						const bvhvec3 axis(sinf(polar) * cosf(azimuth), sinf(polar) * sinf(azimuth), cosf(polar));
						const Mat3f cellRotation = rodriguesRotMat(axis, angle);
						const bvhvec3 basis[3] = {
							bvhvec3(cellRotation.m[0][0], cellRotation.m[1][0], cellRotation.m[2][0]),
							bvhvec3(cellRotation.m[0][1], cellRotation.m[1][1], cellRotation.m[2][1]),
							bvhvec3(cellRotation.m[0][2], cellRotation.m[1][2], cellRotation.m[2][2])
						};

						const uint32_t cellIdx = (angleBin * POLAR_BINS + polarBin) * AZIMUTH_BINS + azimuthBin;
						azimuthAngles[cellIdx] = bruteForceBestRot(basis);
					}
				}
			}
		}

		// Each triangle of the quad sphere for out purposes is 
		// corresponds to a face center,an edge middle point and a 
		// corner (of the unit cube)
		void findQuadSphereTriangle(const bvhvec3& direction, reprojectionEntry& ret) {
			const float components[3] = { direction.x, direction.y, direction.z };

			// Find face center
			float faceCenter[3] = {0.0f, 0.0f, 0.0f};

			uint8_t majorAxis = 0;
			if (fabsf(components[1]) > fabsf(components[majorAxis])) 
				majorAxis = 1;
			if (fabsf(components[2]) > fabsf(components[majorAxis]))
				majorAxis = 2;

			faceCenter[majorAxis] = components[majorAxis] >= 0.0f ? 1.0f : -1.0f;
			quadSpherePointToProxyAxis(faceCenter, ret.axisIdx[0], ret.sign[0]);

			// Find cube corner
			float cubeCorner[3] = {faceCenter[0], faceCenter[1], faceCenter[2]};

			int minorAxisFirst = (majorAxis + 1) % 3;
			int minorAxisSecond = (majorAxis + 2) % 3;
			const float minorAxisFirstSign = components[minorAxisFirst] >= 0.0f ? 1.0f : -1.0f;
			const float minorAxisSecondSign = components[minorAxisSecond] >= 0.0f ? 1.0f : -1.0f;
			
			cubeCorner[minorAxisFirst] = minorAxisFirstSign;
			cubeCorner[minorAxisSecond] = minorAxisSecondSign;
			quadSpherePointToProxyAxis(cubeCorner, ret.axisIdx[1], ret.sign[1]);

			// Find edge midpoint
			float edgeMidPoint[3] = {faceCenter[0], faceCenter[1], faceCenter[2]};
			
			const bool minorAxisFirstIsLarger = fabsf(components[minorAxisFirst]) >= fabsf(components[minorAxisSecond]);
			if (minorAxisFirstIsLarger)
				edgeMidPoint[minorAxisFirst] = minorAxisFirstSign;
			else
				edgeMidPoint[minorAxisSecond] = minorAxisSecondSign;
			quadSpherePointToProxyAxis(edgeMidPoint, ret.axisIdx[2], ret.sign[2]);
		}

		void quadSpherePointToProxyAxis(const float quadSpherePoint[3], uint8_t& axisIdx, int& sign) {
			bvhvec3 quadSpherePointVec = {quadSpherePoint[0], quadSpherePoint[1], quadSpherePoint[2] };
			quadSpherePointVec = tinybvh_normalize(quadSpherePointVec);
			
			for (uint8_t proxyAxisIdx = 0; proxyAxisIdx < AXESNO; proxyAxisIdx++) {
				float alignment = tinybvh_dot(
						proxyKDopAxes[proxyAxisIdx],
						quadSpherePointVec);

				if (alignment > 0.999f) { axisIdx = proxyAxisIdx; sign = 1; return; }
				if (alignment < -0.999f) { axisIdx = proxyAxisIdx; sign = -1; return; }
			}
		}
	};

	class BVH_DOBB : public BVHBase {
	public:
		struct ALIGNED(16) BVHNode
		{
			// Block 0
			bvhvec3 nodeMinPoint;
			int8_t exponent[3];
			uint8_t iMask;
			// Block 1
			uint32_t childBaseIndexRotation;	// 0 - 23: childBaseIndex, 24 - 31: rotation
			uint32_t triBaseIndex;
			uint8_t meta[8];
			// Block 2 - 4
			uint8_t qLo[3][8];
			uint8_t qHi[3][8];

			static constexpr uint32_t CHILD_INDEX_BITS = 24;
			static constexpr uint32_t CHILD_INDEX_MASK = (1u << CHILD_INDEX_BITS) - 1u;

			uint32_t ChildBaseIndex() const { return childBaseIndexRotation & CHILD_INDEX_MASK; }
			uint8_t Rotation() const { return (uint8_t)(childBaseIndexRotation >> CHILD_INDEX_BITS); }
			void SetChildBaseIndexRotation(const uint32_t childBaseIndex, const uint8_t rotationIndex) {
				childBaseIndexRotation = (childBaseIndex & CHILD_INDEX_MASK) | ((uint32_t)rotationIndex << CHILD_INDEX_BITS);
			}

			uint8_t OccupiedMask() const {
				uint8_t occupied = 0;
				for (uint8_t slot = 0; slot < 8; slot++)
					occupied |= (uint8_t)((meta[slot] != 0) << slot);
				return occupied;
			}

			uint8_t LeafMask() const { return OccupiedMask() & ~iMask; }
		};

		// For traversal of the bvh
		struct StackEntry { uint32_t nodeIdx; float entryDistance; };

		// TODO: add layout
		BVH_DOBB(BVHContext ctx = {}) { context = ctx; layout = LAYOUT_DOBB; }
		~BVH_DOBB();

		void Build(const bvhvec4* vertices, const uint32_t primCount);
		void Build(const bvhvec4slice& vertices);
		void Build(const bvhvec4* vertices, const uint32_t* indices, const uint32_t primCount);
		void Build(const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t primCount);

		// Leaves can have at most two triangles in them!
		void ConvertFrom(const MBVH<8>& original);

		int32_t Intersect(Ray& ray) const;
		bool IsOccluded(const Ray& ray) const;

		static DOBB_LUT getLut() { return lut; }
		static DOBB_LUT::Mat3f getLutEntry(uint32_t idx) { return lut.getRawEntry(idx); }

		// BVH data
		BVHNode* dobbNode = 0;
		uint32_t* dobbPrimIdx = 0;
		uint32_t allocatedPrimIndices = 0;
		MBVH<8> bvh8;
		bool ownBVH8 = true;
		
		bool forceInternalAABB = false;

	private:
		// SAH optimal widening, via Claude
		struct WideningData {
			float cost[8];              // cost[slots - 1] = C(n, slots)
			uint8_t leftSlots[8];       // slots >= 2: winning split of Distribute(n, slots), 0 = fewer slots was cheaper
			uint8_t internalLeftSlots;  // split of Distribute(n, 8), used when n becomes a wide interior node
			bool asLeaf;                // decision for C(n, 1)
			uint32_t firstTri, triCount; // subtree's contiguous range in primIdx
		};

		// SAH optimal widening, via Claude
		void widenSAHOptimal();
		void gatherSlots(const WideningData* table, const uint32_t binaryIdx, const uint32_t slots,
			uint32_t* slotNodes, uint32_t& slotCount) const;

		static DOBB_LUT lut;

		void calcBasisVectorsForBFromTri(const bvhvec3* triVerts, bvhvec3& a0, bvhvec3& a1, bvhvec3& a2);

		void quadify();
};

} // namespace tinybvh
#endif // TINY_BVH_DOBB_H_


#if defined( TINYBVH_DOBB_IMPLEMENTATION ) && !defined( TINY_BVH_DOBB_IMPL_DONE )
#define TINY_BVH_DOBB_IMPL_DONE

namespace tinybvh {
	DOBB_LUT BVH_DOBB::lut;

	BVH_DOBB::~BVH_DOBB()
	{
		if (!ownBVH8) bvh8.ReleaseOwnership();
		AlignedFree(dobbNode);
		AlignedFree(dobbPrimIdx);
	}

	void BVH_DOBB::Build(const bvhvec4* vertices, const uint32_t primCount) {
		Build(bvhvec4slice(vertices, primCount * 3, sizeof(bvhvec4)), 0, 0);
	}

	void BVH_DOBB::Build(const bvhvec4slice& vertices) {
		Build(vertices, 0, 0);
	}

	void BVH_DOBB::Build(const bvhvec4* vertices, const uint32_t* indices, const uint32_t primCount) {
		Build(bvhvec4slice(vertices, primCount * 3, sizeof(bvhvec4)), indices, primCount);
	}

	void BVH_DOBB::Build(const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t primCount) {
		// make sure we own bvh8 (a previous ConvertFrom may have left it referencing someone else's)
		if (!ownBVH8) bvh8.DropReference(context), ownBVH8 = true;
		bvh8.bvh.context = context;
		// Build binary BVH
		bvh8.bvh.settings.useFullSweep = true;
		bvh8.bvh.Build(vertices, indices, primCount);
		// Quadify the BVH for next steps
		quadify();
		// SAH optimal widening
		widenSAHOptimal();
		// Create oriented tree
		ConvertFrom(bvh8);
	}

	void BVH_DOBB::quadify() {
		BVH& bvh = bvh8.bvh;

		uint32_t stack[512]; 
		uint32_t stackIdx = 0;
		stack[stackIdx++] = 0;

		while (stackIdx > 0) {
			BVH::BVHNode& currNode = bvh.bvhNode[stack[--stackIdx]];

			if (currNode.triCount <= 2) {
				// Internal node OR leaf node with at most 2 triangles, nothing to "quadify"
				if (!currNode.isLeaf()) {
					// In case it is an internal node, add the two children to the stack
					stack[stackIdx++] = currNode.leftFirst;
					stack[stackIdx++] = currNode.leftFirst + 1;
				}
			}
			else {
				// In case it is leaf node with triCount > 2 we need to quadify
				const uint32_t firstTri = currNode.leftFirst;

				// Extract the triangle
				uint32_t i0, i1, i2;
				GET_PRIM_INDICES_I0_I1_I2(bvh, bvh.primIdx[firstTri]);
				bvhvec3 vert0 = bvh.verts[i0], vert1 = bvh.verts[i1], vert2 = bvh.verts[i2];
				const bvhvec3 firstTriMin = tinybvh_min(vert0, tinybvh_min(vert1, vert2));
				const bvhvec3 firstTriMax = tinybvh_max(vert0, tinybvh_max(vert1, vert2));

				// Find the best pairing
				float minSA = BVH_FAR;
				uint32_t minPairIdx = firstTri + 1;
				for (uint32_t candidateIdx = firstTri + 1; candidateIdx < firstTri + currNode.triCount; candidateIdx++) {
					GET_PRIM_INDICES_I0_I1_I2(bvh, bvh.primIdx[candidateIdx]);
					bvhvec3 vert0 = bvh.verts[i0], vert1 = bvh.verts[i1], vert2 = bvh.verts[i2];

					bvhvec3 extent = 
						tinybvh_max(firstTriMax, tinybvh_max(vert0, tinybvh_max(vert1, vert2)))
						- tinybvh_min(firstTriMin, tinybvh_min(vert0, tinybvh_min(vert1, vert2)));

					const float candidateSA = extent.x * extent.y + extent.y * extent.z + extent.z * extent.x;
					if (candidateSA < minSA) {
						minSA = candidateSA;
						minPairIdx = candidateIdx;
					}
				}

				// Swap triangles
				const uint32_t swappedPrim = bvh.primIdx[firstTri + 1];
				bvh.primIdx[firstTri + 1] = bvh.primIdx[minPairIdx];
				bvh.primIdx[minPairIdx] = swappedPrim;

				// Split leaves
				// Create and init new nodes
				const uint32_t leftIdx = bvh.newNodePtr;
				BVH::BVHNode& left = bvh.bvhNode[leftIdx];		left = currNode;
				BVH::BVHNode& right = bvh.bvhNode[leftIdx + 1]; right = currNode;
				// Correct wrong values in new nodes
				left.triCount = 2; right.triCount = currNode.triCount - 2;
				right.leftFirst = currNode.leftFirst + 2;
				// Update currNode
				currNode.leftFirst = leftIdx;
				currNode.triCount = 0;

				bvh.newNodePtr += 2;

				// Push new (right) node onto the stack
				stack[stackIdx++] = leftIdx + 1;
			}
		}

		bvh.usedNodes = bvh.newNodePtr;
		bvh.Refit();
	}

	// SAH optimal widening via Claude
	void BVH_DOBB::gatherSlots(const WideningData* table, const uint32_t binaryIdx, const uint32_t slots,
		uint32_t* slotNodes, uint32_t& slotCount) const
	{
		// One slot: this binary node becomes a child of the wide node
		if (slots == 1) { slotNodes[slotCount++] = binaryIdx; return; }

		const uint8_t leftSlots = table[binaryIdx].leftSlots[slots - 1];
		// Fewer slots was cheaper
		if (leftSlots == 0) { gatherSlots(table, binaryIdx, slots - 1, slotNodes, slotCount); return; }

		// Share the slots between the two binary children
		const uint32_t leftChildIdx = bvh8.bvh.bvhNode[binaryIdx].leftFirst;
		gatherSlots(table, leftChildIdx, leftSlots, slotNodes, slotCount);
		gatherSlots(table, leftChildIdx + 1, slots - leftSlots, slotNodes, slotCount);
	}

	// SAH optimal widening via Claude
	void BVH_DOBB::widenSAHOptimal() {
		BVH& bvh = bvh8.bvh;
		FATAL_ERROR_IF(bvh.bvhNode[0].isLeaf(), "BVH_DOBB::widenSAHOptimal(), binary root is a leaf.");

		const float traversalCost = bvh.c_trav, intersectionCost = bvh.c_int;
		WideningData* table = (WideningData*)AlignedAlloc(bvh.usedNodes * sizeof(WideningData));

		// Pass 1: bottom-up costs (children have larger indices than parents)
		for (int32_t nodeIdx = (int32_t)bvh.usedNodes - 1; nodeIdx >= 0; nodeIdx--) {
			if (nodeIdx == 1) continue;   // unused node

			const BVH::BVHNode& node = bvh.bvhNode[nodeIdx];
			WideningData& entry = table[nodeIdx];
			const bvhvec3 extent = node.aabbMax - node.aabbMin;
			const float area = 2.0f * (extent.x * extent.y + extent.y * extent.z + extent.z * extent.x);

			if (node.isLeaf()) {
				// Binary leaf: can't be split, same cost for every slot budget
				const float leafCost = area * node.triCount * intersectionCost;
				for (uint32_t slots = 1; slots <= 8; slots++) {
					entry.cost[slots - 1] = leafCost;
					entry.leftSlots[slots - 1] = 0;
				}
				entry.internalLeftSlots = 0;
				entry.asLeaf = true;
				entry.firstTri = node.leftFirst;
				entry.triCount = node.triCount;
				continue;
			}

			const WideningData& leftEntry = table[node.leftFirst];
			const WideningData& rightEntry = table[node.leftFirst + 1];
			entry.firstTri = tinybvh_min(leftEntry.firstTri, rightEntry.firstTri);
			entry.triCount = leftEntry.triCount + rightEntry.triCount;

			// Distribute(n, slots): share the slots between the two children
			float distributeCost[9];
			uint8_t distributeLeftSlots[9];
			for (uint32_t slots = 2; slots <= 8; slots++) {
				distributeCost[slots] = BVH_FAR;
				distributeLeftSlots[slots] = 1;
				for (uint32_t leftSlots = 1; leftSlots < slots; leftSlots++) {
					const float splitCost = leftEntry.cost[leftSlots - 1] + rightEntry.cost[slots - leftSlots - 1];
					if (splitCost < distributeCost[slots]) {
						distributeCost[slots] = splitCost;
						distributeLeftSlots[slots] = (uint8_t)leftSlots;
					}
				}
			}

			// C(n, 1): one MBVH leaf, or one wide interior node
			const float leafCost = entry.triCount <= 2 ? area * entry.triCount * intersectionCost : BVH_FAR;
			const float internalCost = area * traversalCost + distributeCost[8];
			entry.asLeaf = leafCost <= internalCost;
			entry.cost[0] = entry.asLeaf ? leafCost : internalCost;
			entry.leftSlots[0] = 0;
			entry.internalLeftSlots = distributeLeftSlots[8];

			// C(n, slots): split across the children, or use fewer slots
			for (uint32_t slots = 2; slots <= 8; slots++) {
				if (distributeCost[slots] < entry.cost[slots - 2]) {
					entry.cost[slots - 1] = distributeCost[slots];
					entry.leftSlots[slots - 1] = distributeLeftSlots[slots];
				}
				else {
					entry.cost[slots - 1] = entry.cost[slots - 2];
					entry.leftSlots[slots - 1] = 0;
				}
			}
		}

		// Allocate the wide nodes (each corresponds to a distinct binary node, so usedNodes is enough)
		if (bvh8.allocatedNodes < bvh.usedNodes) {
			AlignedFree(bvh8.mbvhNode);
			bvh8.mbvhNode = (MBVH<8>::MBVHNode*)AlignedAlloc(bvh.usedNodes * sizeof(MBVH<8>::MBVHNode));
			bvh8.allocatedNodes = bvh.usedNodes;
		}

		// Pass 2: top-down, breadth-first; the mbvhNode array doubles as the queue
		uint32_t* wideToBinary = (uint32_t*)AlignedAlloc(bvh.usedNodes * sizeof(uint32_t));
		wideToBinary[0] = 0;
		uint32_t wideNodeCount = 1;

		for (uint32_t wideIdx = 0; wideIdx < wideNodeCount; wideIdx++) {
			const uint32_t binaryIdx = wideToBinary[wideIdx];
			const BVH::BVHNode& binaryNode = bvh.bvhNode[binaryIdx];
			const WideningData& entry = table[binaryIdx];

			MBVH<8>::MBVHNode& wideNode = bvh8.mbvhNode[wideIdx];
			memset(&wideNode, 0, sizeof(wideNode));
			wideNode.aabbMin = binaryNode.aabbMin;
			wideNode.aabbMax = binaryNode.aabbMax;

			// MBVH leaf (the root is always interior)
			if (wideIdx != 0 && entry.asLeaf) {
				wideNode.firstTri = entry.firstTri;
				wideNode.triCount = entry.triCount;
				continue;
			}

			// Wide interior node: its slots come from Distribute(n, 8)
			uint32_t slotNodes[8];
			uint32_t slotCount = 0;
			gatherSlots(table, binaryNode.leftFirst, entry.internalLeftSlots, slotNodes, slotCount);
			gatherSlots(table, binaryNode.leftFirst + 1, 8 - entry.internalLeftSlots, slotNodes, slotCount);

			// Children get the next indices: breadth-first, child > parent, no holes
			wideNode.childCount = slotCount;
			for (uint32_t slot = 0; slot < slotCount; slot++) {
				wideToBinary[wideNodeCount] = slotNodes[slot];
				wideNode.child[slot] = wideNodeCount++;
			}
		}

		// Base properties the DOBB conversion copies from bvh8
		bvh8.usedNodes = wideNodeCount;
		bvh8.triCount = bvh.triCount;
		bvh8.idxCount = bvh.idxCount;
		bvh8.aabbMin = bvh.aabbMin;
		bvh8.aabbMax = bvh.aabbMax;

		AlignedFree(wideToBinary);
		AlignedFree(table);
	}

	void BVH_DOBB::ConvertFrom(const MBVH<8>& original)
	{
		// copy source
		if (&original != &bvh8)
		{
			if (ownBVH8) bvh8 = MBVH<8>(context);
			else bvh8.DropReference(context);
			ownBVH8 = false;
		}
		// ConvertFrom does not handle cases where there are 2 or less primitives
		FATAL_ERROR_IF(bvh8.usedNodes == 0, "BVH_DOBB::ConvertFrom( .. ), source MBVH is empty.");
		FATAL_ERROR_IF(bvh8.mbvhNode[0].isLeaf(), "BVH_DOBB::ConvertFrom( .. ), root is a leaf (scene has 2 triangles or fewer).");
		bvh8.ReferenceFrom(original);
		// get base properties
		CopyBasePropertiesFrom(bvh8);

		// Allocate proxy k-DOPs and rotation sideband data
		dobbSidebandData* sideband;
		sideband = (dobbSidebandData*)AlignedAlloc(bvh8.usedNodes * sizeof(dobbSidebandData));

		// Count the number of interior nodes in bvh8, DOBB BVH needs this
		// amount of nodes in total
		uint32_t nodesNeeded = 0;

		// Iterate through nodes backwards (children are processed
		// implicitly before parents)
		for (int nodeIdx = bvh8.usedNodes - 1; nodeIdx >= 0; nodeIdx--)
		{
			// Reset proxy k-DOP for min/max selection
			for (int axisIdx = 0; axisIdx < AXESNO; axisIdx++)
			{
				sideband[nodeIdx].kDopExtents[axisIdx] = bvhvec2(BVH_FAR, -BVH_FAR);
			}

			const auto& currNode = bvh8.mbvhNode[nodeIdx];

			// Decide if leaf or not
			if (currNode.isLeaf())
			{
				// Process leaf nodes
				bvhvec3 leafVerts[6];
				uint32_t noLeafVerts = 0;
				// used for GET_PRIM_INDICES_I0_I1_I2 macro
				uint32_t i0, i1, i2;

				// For determining B;
				bvhvec3 a0, a1, a2;

				if (currNode.triCount == 1) {
					noLeafVerts = 3;

					// Gather vertices in leaf
					const uint32_t prim = bvh8.bvh.primIdx[currNode.firstTri];
					GET_PRIM_INDICES_I0_I1_I2(bvh8.bvh, prim);

					leafVerts[0] = bvh8.bvh.verts[i0];
					leafVerts[1] = bvh8.bvh.verts[i1];
					leafVerts[2] = bvh8.bvh.verts[i2];

					// Determine B
					calcBasisVectorsForBFromTri(leafVerts, a0, a1, a2);
				}
				// Implicitly, leaf has 2 triangles
				else
				{
					const uint32_t prim0 = bvh8.bvh.primIdx[currNode.firstTri];
					GET_PRIM_INDICES_I0_I1_I2(bvh8.bvh, prim0);

					leafVerts[0] = bvh8.bvh.verts[i0];
					leafVerts[1] = bvh8.bvh.verts[i1];
					leafVerts[2] = bvh8.bvh.verts[i2];

					const uint32_t prim1 = bvh8.bvh.primIdx[currNode.firstTri + 1];
					GET_PRIM_INDICES_I0_I1_I2(bvh8.bvh, prim1);

					leafVerts[3] = bvh8.bvh.verts[i0];
					leafVerts[4] = bvh8.bvh.verts[i1];
					leafVerts[5] = bvh8.bvh.verts[i2];

					// Determine shared vertices
					uint32_t sharedVert1[2] = { 6, 6 };
					uint32_t sharedVert2[2] = { 6, 6 };

					for (uint32_t tri1VertIdx = 0; tri1VertIdx < 3; tri1VertIdx++) {
						for (uint32_t tri2VertIdx = 3; tri2VertIdx < 6; tri2VertIdx++) {
							if (leafVerts[tri1VertIdx].x == leafVerts[tri2VertIdx].x &&
								leafVerts[tri1VertIdx].y == leafVerts[tri2VertIdx].y &&
								leafVerts[tri1VertIdx].z == leafVerts[tri2VertIdx].z)
							{
								if (sharedVert1[0] == 6) {
									sharedVert1[0] = tri1VertIdx;
									sharedVert1[1] = tri2VertIdx;
								}
								else {
									sharedVert2[0] = tri1VertIdx;
									sharedVert2[1] = tri2VertIdx;
								}
							}

						}
					}

					// Check if triangles share an edge 
					if (sharedVert2[0] == 6) {
						calcBasisVectorsForBFromTri(leafVerts, a0, a1, a2);

						// Skip the extra branch by setting a default value
						noLeafVerts = 6;
						// Determine if the two triangles consist of 5 or 6 vertices
						if (sharedVert1[0] != 6) {
							leafVerts[sharedVert1[1]] = leafVerts[5];
							noLeafVerts = 5;
						}
					}
					else {
						noLeafVerts = 4;

						// Init tmpleagVerts for swaps
						bvhvec3 tmpleafVerts[6];
						for (uint32_t i = 0; i < 6; i++)
							tmpleafVerts[i] = leafVerts[i];

						// Insert first three vertex, making sure the shared
						// edge is in slots 1 and 2
						for (uint32_t i = 0; i < 3; i++) {
							if (i != sharedVert1[0] && i != sharedVert2[0])
								leafVerts[0] = tmpleafVerts[i];
							if (i == sharedVert1[0])
								leafVerts[1] = tmpleafVerts[i];
							if (i == sharedVert2[0])
								leafVerts[2] = tmpleafVerts[i];
						}

						// Insert final vertex
						for (uint32_t i = 3; i < 6; i++)
							if (i != sharedVert1[1] && i != sharedVert2[1])
								leafVerts[3] = tmpleafVerts[i];

						// Determine a0 and a1 for B
						// Length does not matter in this case, only that
						// the "middle" vertex is in the shared edge
						a0 = leafVerts[0] - leafVerts[1];
						a1 = leafVerts[3] - leafVerts[1];

						// Calculate B
						a0 = tinybvh_normalize(a0);
						a2 = tinybvh_normalize(tinybvh_cross(a0, a1));
						a1 = tinybvh_normalize(tinybvh_cross(a2, a0)); // normalized for safety
					}
				}

				// find best rotation candidate for leaf node with B
				sideband[nodeIdx].rotation = lut.getRotationForBasis({ a0, a1, a2 });

				// Create proxy KDop to propagate up
				for (uint32_t vertIdx = 0; vertIdx < noLeafVerts; vertIdx++) {
					for (uint32_t axisIdx = 0; axisIdx < AXESNO; axisIdx++) {
						sideband[nodeIdx].kDopExtents[axisIdx].x = tinybvh_min(
							tinybvh_dot(proxyKDopAxes[axisIdx], leafVerts[vertIdx]),
							sideband[nodeIdx].kDopExtents[axisIdx].x);

						sideband[nodeIdx].kDopExtents[axisIdx].y = tinybvh_max(
							tinybvh_dot(proxyKDopAxes[axisIdx], leafVerts[vertIdx]),
							sideband[nodeIdx].kDopExtents[axisIdx].y);
					}
				}
			}
			else
			{
				// Increment nodes needed for DOBB_BVH
				nodesNeeded++;

				// Process internal nodes
				for (uint32_t childIdx = 0; childIdx < currNode.childCount; childIdx++)
				{
					uint32_t currChildIdx = currNode.child[childIdx];

					// Determine k-DOP extents
					for (uint32_t axisIdx = 0; axisIdx < AXESNO; axisIdx++)
					{
						sideband[nodeIdx].kDopExtents[axisIdx].x = tinybvh_min(
							sideband[currChildIdx].kDopExtents[axisIdx].x,
							sideband[nodeIdx].kDopExtents[axisIdx].x);

						sideband[nodeIdx].kDopExtents[axisIdx].y = tinybvh_max(
							sideband[currChildIdx].kDopExtents[axisIdx].y,
							sideband[nodeIdx].kDopExtents[axisIdx].y);
					}
				}

				float AABBSAMax = 0.0f;
				uint8_t candidateRotationIdx = 0;
				sideband[nodeIdx].AABBSASum = 0.0f;

				// Process internal nodes
				for (uint32_t childIdx = 0; childIdx < currNode.childCount; childIdx++)
				{
					uint32_t currChildIdx = currNode.child[childIdx];

					// Gather extents from euclidean axes from axes collection
					float a = fabsf(sideband[currChildIdx].kDopExtents[0].x - sideband[currChildIdx].kDopExtents[0].y);
					float b = fabsf(sideband[currChildIdx].kDopExtents[1].x - sideband[currChildIdx].kDopExtents[1].y);
					float c = fabsf(sideband[currChildIdx].kDopExtents[2].x - sideband[currChildIdx].kDopExtents[2].y);

					// Select rotation by maxselecting on AABB surface area
					float childAABBSA = 2.0f * (a * b + b * c + c * a);
					sideband[nodeIdx].AABBSASum += childAABBSA;

					if (childAABBSA > AABBSAMax) {
						AABBSAMax = childAABBSA;
						candidateRotationIdx = sideband[currChildIdx].rotation;
					}
				}

				sideband[nodeIdx].rotation = candidateRotationIdx;
			}
		}

		// Allocate nodes and tri indices for DOBB_BVH
		if (allocatedNodes < nodesNeeded) {
			AlignedFree(dobbNode);
			dobbNode = (BVHNode*)AlignedAlloc(nodesNeeded * sizeof(BVHNode));

			allocatedNodes = nodesNeeded;
		}

		if (allocatedPrimIndices < idxCount) {
			AlignedFree(dobbPrimIdx);
			dobbPrimIdx = (uint32_t*)AlignedAlloc(idxCount * sizeof(uint32_t));
		
			allocatedPrimIndices = idxCount;
		}
	
		uint32_t stackBvhNodes[512], stackDobbNodes[512];
		stackBvhNodes[0] = 0; stackDobbNodes[0] = 0;
		uint32_t stackIdx = 1;
		uint32_t dobbNodeIdx = 1, dobbTriIdx = 0;

		while (stackIdx > 0) {
			// Pop top member of stack
			stackIdx--;
			uint32_t currBvhIdx = stackBvhNodes[stackIdx];
			uint32_t currDobbIdx = stackDobbNodes[stackIdx];
			const auto& currBvhNode = bvh8.mbvhNode[currBvhIdx];
			BVHNode& currDobbNode = dobbNode[currDobbIdx];
			memset(&currDobbNode, 0, sizeof(BVHNode));

			uint8_t candidateRotationIdx = sideband[currBvhIdx].rotation;
			DOBB_LUT::Mat3f candidateRotation = lut.getRawEntry(candidateRotationIdx);

			// Collect basis vectors of current rotation
			bvhvec3 rotatedAxes[6] = {
				bvhvec3(candidateRotation.m[0][0], candidateRotation.m[1][0], candidateRotation.m[2][0]),
				bvhvec3(-candidateRotation.m[0][0], -candidateRotation.m[1][0], -candidateRotation.m[2][0]),
				bvhvec3(candidateRotation.m[0][1], candidateRotation.m[1][1], candidateRotation.m[2][1]),
				bvhvec3(-candidateRotation.m[0][1], -candidateRotation.m[1][1], -candidateRotation.m[2][1]),
				bvhvec3(candidateRotation.m[0][2], candidateRotation.m[1][2], candidateRotation.m[2][2]),
				bvhvec3(-candidateRotation.m[0][2], -candidateRotation.m[1][2], -candidateRotation.m[2][2])
			};

			float rotatedAxesExtents[6];

			bool hasLeafSlot = false;

			// Values for e
			float DOBBSASum = 0.0f;
			bvhvec3 childMinPoint[8], childMaxPoint[8];

			for (uint32_t childIdx = 0; childIdx < currBvhNode.childCount; childIdx++)
			{
				uint32_t currChildIdx = currBvhNode.child[childIdx];

				// Do reprojection based on the selected rotation
				for (uint32_t rotatedAxesIdx = 0; rotatedAxesIdx < 6; rotatedAxesIdx++)
				{
					bvhvec3 apexPoint;

					// Do reprojection for the given axis
					// Reprojection entry corresponding to current rotation and axis
					const DOBB_LUT::reprojectionEntry& reprojection = lut.reprojections[candidateRotationIdx][rotatedAxesIdx];
					// Normals of selected planes
					bvhvec3 v0 = proxyKDopAxes[reprojection.axisIdx[0]] * (float)reprojection.sign[0];
					bvhvec3 v1 = proxyKDopAxes[reprojection.axisIdx[1]] * (float)reprojection.sign[1];
					bvhvec3 v2 = proxyKDopAxes[reprojection.axisIdx[2]] * (float)reprojection.sign[2];
					// Extents of selected planes
					float h0, h1, h2;
					if (reprojection.sign[0] == 1)
						h0 = sideband[currChildIdx].kDopExtents[reprojection.axisIdx[0]].y;
					else h0 = -sideband[currChildIdx].kDopExtents[reprojection.axisIdx[0]].x;
					if (reprojection.sign[1] == 1)
						h1 = sideband[currChildIdx].kDopExtents[reprojection.axisIdx[1]].y;
					else h1 = -sideband[currChildIdx].kDopExtents[reprojection.axisIdx[1]].x;
					if (reprojection.sign[2] == 1)
						h2 = sideband[currChildIdx].kDopExtents[reprojection.axisIdx[2]].y;
					else h2 = -sideband[currChildIdx].kDopExtents[reprojection.axisIdx[2]].x;
					// Determine apex point by finding intersection of three planes
					// Find vectors colinear with the intersection of pairwise planes
					bvhvec3 c0 = tinybvh_cross(v1, v2);
					apexPoint = (c0 * h0 + tinybvh_cross(v2, v0) * h1 + tinybvh_cross(v0, v1) * h2) / tinybvh_dot(v0, c0);
					// Determine axis extent by projecting apex point onto the axis
					rotatedAxesExtents[rotatedAxesIdx] = tinybvh_dot(rotatedAxes[rotatedAxesIdx], apexPoint);
				}

				childMinPoint[childIdx].x = -rotatedAxesExtents[1];
				childMinPoint[childIdx].y = -rotatedAxesExtents[3];
				childMinPoint[childIdx].z = -rotatedAxesExtents[5];

				childMaxPoint[childIdx].x = rotatedAxesExtents[0];
				childMaxPoint[childIdx].y = rotatedAxesExtents[2];
				childMaxPoint[childIdx].z = rotatedAxesExtents[4];

				float a = childMaxPoint[childIdx].x - childMinPoint[childIdx].x;
				float b = childMaxPoint[childIdx].y - childMinPoint[childIdx].y;
				float c = childMaxPoint[childIdx].z - childMinPoint[childIdx].z;
				DOBBSASum += 2 * (a * b + a * c + b * c);

				// Check if child is a leaf slot
				if (bvh8.mbvhNode[currChildIdx].isLeaf()) 
					hasLeafSlot = true;
			}

			// Determine chosen rotation, record extents if no rotation
			uint8_t chosenRotation;

			// Helper for forcing AABB
			bool shouldBeAABB = forceInternalAABB && !hasLeafSlot;

			if (!shouldBeAABB && DOBBSASum < ALPHA * sideband[currBvhIdx].AABBSASum) {
				chosenRotation = candidateRotationIdx;
			}
			else {
				chosenRotation = 255;

				for (uint32_t childIdx = 0; childIdx < currBvhNode.childCount; childIdx++) {
					uint32_t currChildIdx = currBvhNode.child[childIdx];

					childMinPoint[childIdx] = bvhvec3(
						sideband[currChildIdx].kDopExtents[0].x,
						sideband[currChildIdx].kDopExtents[1].x,
						sideband[currChildIdx].kDopExtents[2].x);
					childMaxPoint[childIdx] = bvhvec3(
						sideband[currChildIdx].kDopExtents[0].y,
						sideband[currChildIdx].kDopExtents[1].y,
						sideband[currChildIdx].kDopExtents[2].y);
				}
			}

			// Determine min and max points of bounding boxes
			bvhvec3 minPoint = { BVH_FAR, BVH_FAR, BVH_FAR };
			bvhvec3 maxPoint = { -BVH_FAR, -BVH_FAR, -BVH_FAR };

			for (uint32_t childIdx = 0; childIdx < currBvhNode.childCount; childIdx++) {
				minPoint = tinybvh_min(minPoint, childMinPoint[childIdx]);
				maxPoint = tinybvh_max(maxPoint, childMaxPoint[childIdx]);
			}

			currDobbNode.nodeMinPoint = minPoint;

			// To be used to calculate qLo-s and qHi-s
			float inverseScale[3];

			// Quantize extent of current bounding box
			for (uint32_t axisIdx = 0; axisIdx < 3; axisIdx++) {
				const float extent = maxPoint[axisIdx] - minPoint[axisIdx];
				// guard against in-plain BBs
				int32_t exponent = extent > 0.0f ? (int32_t)ceilf(log2f(extent / 255.0f)) : -126;
				// clamp into int8_t range
				exponent = tinybvh_clamp(exponent, -126, 127);
				// guard against ceil rounding "extent" below the real extent
				// indirectly by giving exponent too small of a value
				if (exponent < 127 && ldexpf(255.0f, exponent) < extent)
					exponent++;
				currDobbNode.exponent[axisIdx] = (int8_t)exponent;
				// calculate inverseScale here, once
				inverseScale[axisIdx] = ldexpf(1.0f, -exponent);
			}

			uint32_t currDobbNodeChildBaseIdx = 0;
			uint32_t currDobbNodeInternalChildCount = 0;
			uint32_t currDobbNodeTriCount = 0;

			// Fill rest of the data for the compressed nodes
			for (uint32_t childIdx = 0; childIdx < currBvhNode.childCount; childIdx++) {
				// Fill qLo and qHi
				// Fill qLo 
				currDobbNode.qLo[0][childIdx] = (uint8_t)tinybvh_clamp(
					floorf((childMinPoint[childIdx][0] - minPoint[0]) * inverseScale[0]),
					0.0f, 255.0f);
				currDobbNode.qLo[1][childIdx] = (uint8_t)tinybvh_clamp(
					floorf((childMinPoint[childIdx][1] - minPoint[1]) * inverseScale[1]),
					0.0f, 255.0f);
				currDobbNode.qLo[2][childIdx] = (uint8_t)tinybvh_clamp(
					floorf((childMinPoint[childIdx][2] - minPoint[2]) * inverseScale[2]),
					0.0f, 255.0f);
				// Fill qHi
				currDobbNode.qHi[0][childIdx] = (uint8_t)tinybvh_clamp(
					ceilf((childMaxPoint[childIdx][0] - minPoint[0]) * inverseScale[0]),
					0.0f, 255.0f);
				currDobbNode.qHi[1][childIdx] = (uint8_t)tinybvh_clamp(
					ceilf((childMaxPoint[childIdx][1] - minPoint[1]) * inverseScale[1]),
					0.0f, 255.0f);
				currDobbNode.qHi[2][childIdx] = (uint8_t)tinybvh_clamp(
					ceilf((childMaxPoint[childIdx][2] - minPoint[2]) * inverseScale[2]),
					0.0f, 255.0f);

				uint32_t currChildIdx = currBvhNode.child[childIdx];
				const auto& currChild = bvh8.mbvhNode[currChildIdx];

				// Pack internal child nodes
				if (!currChild.isLeaf())
				{
					// Set base index if its the first internal child
					// dobbNodeIdx holds the next available dobb node
					if (currDobbNodeInternalChildCount++ == 0)
						currDobbNodeChildBaseIdx = dobbNodeIdx;

					// Add next index pair to stack
					stackBvhNodes[stackIdx] = currChildIdx;
					stackDobbNodes[stackIdx++] = dobbNodeIdx++;

					currDobbNode.iMask |= (uint8_t)(1 << childIdx);
					currDobbNode.meta[childIdx] = (uint8_t)(0b00100000 | (24 + childIdx));
				}
				// Pack leaf child nodes
				else
				{
					// Set base index if it is the first leaf child
					// dobbTriIdx holds the next available triangle
					if (currDobbNodeTriCount == 0)
						currDobbNode.triBaseIndex = dobbTriIdx;

					// Pack meta
					const uint8_t unaryTriCount = currChild.triCount == 1 ? 0b001 : 0b011;
					currDobbNode.meta[childIdx] = (uint8_t)((unaryTriCount << 5) | currDobbNodeTriCount);

					// Increment tri count
					currDobbNodeTriCount += currChild.triCount;

					// Copy triangle indices into dobbPrimIdx in the correct
					// order 
					for (uint32_t triIdx = 0; triIdx < currChild.triCount; triIdx++)
						dobbPrimIdx[dobbTriIdx++] = bvh8.bvh.primIdx[currChild.firstTri + triIdx];
				}
			}

			// Set child base index and rotation in dobb node
			currDobbNode.SetChildBaseIndexRotation(currDobbNodeChildBaseIdx, chosenRotation);
		}

		usedNodes = dobbNodeIdx;

		AlignedFree(sideband);
		sideband = 0;
	}

	void BVH_DOBB::calcBasisVectorsForBFromTri(const bvhvec3* triVerts, bvhvec3& a0, bvhvec3& a1, bvhvec3& a2) {
		// Determine longest edge of triangle => find 
		// "middle point" of shortest edges
		const bvhvec3 edges[3] = {
						triVerts[1] - triVerts[0], // (v0, v1) = e0
						triVerts[2] - triVerts[1], // (v1, v2) = e1
						triVerts[0] - triVerts[2]  // (v2, v0) = e2
		};
		// Get lengths
		const float lengthsSqd[3] = {
			tinybvh_dot(edges[0], edges[0]),
			tinybvh_dot(edges[1], edges[1]),
			tinybvh_dot(edges[2], edges[2])
		};

		uint32_t longestIdx = lengthsSqd[0] > lengthsSqd[1] ? 0 : 1;
		if (lengthsSqd[2] > lengthsSqd[longestIdx]) longestIdx = 2;

		uint32_t midPointIdx = (longestIdx + 2) % 3;

		if (lengthsSqd[midPointIdx] < lengthsSqd[(midPointIdx + 2) % 3]) {
			a0 = edges[midPointIdx];
			a1 = -edges[(midPointIdx + 2) % 3];
		}
		else {
			a0 = -edges[(midPointIdx + 2) % 3];
			a1 = edges[midPointIdx];
		}

		// Calculate B
		a0 = tinybvh_normalize(a0);
		a2 = tinybvh_normalize(tinybvh_cross(a0, a1));
		a1 = tinybvh_normalize(tinybvh_cross(a2, a0)); // normalized for safety
	}

	int32_t BVH_DOBB::Intersect(Ray& ray) const {
		int32_t traversalSteps = 0;

		StackEntry nodeStack[512];
		uint32_t stackIdx = 0;
		nodeStack[stackIdx++] = {0, -BVH_FAR};

		while (stackIdx > 0) {
			// Early exit if a closer hit was already found
			const StackEntry entry = nodeStack[--stackIdx];
			if (entry.entryDistance >= ray.hit.t) continue;

			traversalSteps += (uint32_t)c_trav;
			const BVHNode& currNode = dobbNode[entry.nodeIdx];

			Ray localRay;
			localRay.hit.t = ray.hit.t;
			const uint32_t rotationIdx = currNode.Rotation();

			if (rotationIdx != 255) {
				const DOBB_LUT::Mat3f& rotation = lut.getRawEntry(rotationIdx);

				const bvhvec3 rotationX = bvhvec3(rotation.m[0][0], rotation.m[1][0], rotation.m[2][0]);
				const bvhvec3 rotationY = bvhvec3(rotation.m[0][1], rotation.m[1][1], rotation.m[2][1]);
				const bvhvec3 rotationZ = bvhvec3(rotation.m[0][2], rotation.m[1][2], rotation.m[2][2]);
			
				localRay.O = bvhvec3(
					tinybvh_dot(rotationX, ray.O),
					tinybvh_dot(rotationY, ray.O),
					tinybvh_dot(rotationZ, ray.O));

				localRay.D = bvhvec3(
					tinybvh_dot(rotationX, ray.D),
					tinybvh_dot(rotationY, ray.D),
					tinybvh_dot(rotationZ, ray.D));
			}
			else {
				localRay.O = ray.O;
				localRay.D = ray.D;
			}
			localRay.rD = tinybvh_rcp(localRay.D);

			float stepSize[3] = {
				ldexpf(1.0f, currNode.exponent[0]),
				ldexpf(1.0f, currNode.exponent[1]),
				ldexpf(1.0f, currNode.exponent[2]),
			};

			float slotDistance[8];
			uint32_t sortedSlots[8];
			uint32_t sortedCount = 0;

			uint32_t remainingSlots = currNode.OccupiedMask();

			while (remainingSlots) {
				// Determine which is the next slot to process
				uint32_t slot = (uint32_t)std::countr_zero(remainingSlots);
				remainingSlots &= remainingSlots - 1;

				// Determine lower and higher extents of the BB
				bvhvec3 slotMinPoint = currNode.nodeMinPoint + bvhvec3(
					currNode.qLo[0][slot] * stepSize[0],
					currNode.qLo[1][slot] * stepSize[1],
					currNode.qLo[2][slot] * stepSize[2]
				);

				bvhvec3 slotMaxPoint = currNode.nodeMinPoint + bvhvec3(
					currNode.qHi[0][slot] * stepSize[0],
					currNode.qHi[1][slot] * stepSize[1],
					currNode.qHi[2][slot] * stepSize[2]
				);

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
					const uint32_t primIdx = dobbPrimIdx[firstTri + triIdx];
					GET_PRIM_INDICES_I0_I1_I2(bvh8.bvh, primIdx);
					IntersectTri(ray, primIdx, bvh8.bvh.verts, i0, i1, i2);
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

		return traversalSteps;
	}

	bool BVH_DOBB::IsOccluded(const Ray& ray) const {
		uint32_t nodeStack[512];
		uint32_t stackIdx = 0;
		nodeStack[stackIdx++] = 0;

		while (stackIdx > 0) {
			const uint32_t currNodeIdx = nodeStack[--stackIdx];
			const BVHNode& currNode = dobbNode[currNodeIdx];

			Ray localRay;
			localRay.hit.t = ray.hit.t;
			const uint32_t rotationIdx = currNode.Rotation();

			if (rotationIdx != 255) {
				const DOBB_LUT::Mat3f& rotation = lut.getRawEntry(rotationIdx);

				const bvhvec3 rotationX = bvhvec3(rotation.m[0][0], rotation.m[1][0], rotation.m[2][0]);
				const bvhvec3 rotationY = bvhvec3(rotation.m[0][1], rotation.m[1][1], rotation.m[2][1]);
				const bvhvec3 rotationZ = bvhvec3(rotation.m[0][2], rotation.m[1][2], rotation.m[2][2]);

				localRay.O = bvhvec3(
					tinybvh_dot(rotationX, ray.O),
					tinybvh_dot(rotationY, ray.O),
					tinybvh_dot(rotationZ, ray.O));

				localRay.D = bvhvec3(
					tinybvh_dot(rotationX, ray.D),
					tinybvh_dot(rotationY, ray.D),
					tinybvh_dot(rotationZ, ray.D));
			}
			else {
				localRay.O = ray.O;
				localRay.D = ray.D;
			}
			localRay.rD = tinybvh_rcp(localRay.D);

			float stepSize[3] = {
				ldexpf(1.0f, currNode.exponent[0]),
				ldexpf(1.0f, currNode.exponent[1]),
				ldexpf(1.0f, currNode.exponent[2]),
			};

			uint32_t hitMask = 0;
			uint32_t remainingSlots = currNode.OccupiedMask();

			while (remainingSlots) {
				// Determine which is the next slot to process
				uint32_t slot = (uint32_t)std::countr_zero(remainingSlots);
				remainingSlots &= remainingSlots - 1;

				// Determine lower and higher extents of the BB
				bvhvec3 slotMinPoint = currNode.nodeMinPoint + bvhvec3(
					currNode.qLo[0][slot] * stepSize[0],
					currNode.qLo[1][slot] * stepSize[1],
					currNode.qLo[2][slot] * stepSize[2]
				);

				bvhvec3 slotMaxPoint = currNode.nodeMinPoint + bvhvec3(
					currNode.qHi[0][slot] * stepSize[0],
					currNode.qHi[1][slot] * stepSize[1],
					currNode.qHi[2][slot] * stepSize[2]
				);

				float entryDistance = tinybvh_intersect_aabb(localRay, slotMinPoint, slotMaxPoint);
				if (entryDistance < BVH_FAR) {
					hitMask |= 1 << slot;
				}
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
					const uint32_t primIdx = dobbPrimIdx[firstTri + triIdx];
					GET_PRIM_INDICES_I0_I1_I2(bvh8.bvh, primIdx);
					if(TriOccludes(ray, bvh8.bvh.verts, primIdx, i0, i1, i2))
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
} // namespace tinybvh
#endif // TINYBVH_DOBB_IMPLEMENTATION