// tiny_bvh_gpu.h: the GPU layouts of TinyBVH (BVH_GPU, BVH4_GPU, BVH8_CWBVH) and the
// layouts for ray tracing hardware (BVH4_AMD_HW, CWBVH8_AMD_HW, BVH6_INTEL_HW),
// with their CPU-side traversal. Do not include this file directly:
// tiny_bvh.h includes it, unless TINYBVH_NO_GPU is defined.

#ifndef TINY_BVH_H_
#error "Include tiny_bvh.h instead of tiny_bvh_gpu.h."
#endif

#ifndef TINY_BVH_GPU_H_
#define TINY_BVH_GPU_H_

#ifdef _MSC_VER
#pragma warning ( push )
#pragma warning ( disable: 4201 /* nameless struct / union */ )
#pragma warning ( disable: 4324 /* structure padding */ )
#endif

namespace tinybvh {
namespace impl {

TEMPLATED class BVH_GPU : public BVHBase<Float, Index>
{
public:
	using Base = BVHBase<Float, Index>;
	using typename Base::Vec3;
	using typename Base::Vertex;
	using typename Base::Slice;
	using typename Base::Ray;
	using typename Base::BVH;
	using typename Base::BLASInstance;
	using Base::settings;
	using Base::c_trav;
	using Base::c_int;
	using Base::context;
	using Base::layout;
	using Base::allocatedNodes;
	using Base::usedNodes;
	using Base::triCount;
	using Base::idxCount;
	using Base::aabbMin;
	using Base::aabbMax;
protected:
	using Base::hasHoles;
	using Base::orderedNodes;
	using Base::bvhOverAabbs;
	using Base::AlignedAlloc;
	using Base::AlignedFree;
	using Base::CopyBasePropertiesFrom;
public:
	struct BVHNode
	{
		// Alternative 64-byte BVH node layout, which specifies the bounds of
		// the children rather than the node itself. This layout is used by
		// Aila and Laine in their seminal GPU ray tracing paper.
		Vec3 lmin; uint32_t left;
		Vec3 lmax; uint32_t right;
		Vec3 rmin; uint32_t triCount;
		Vec3 rmax; uint32_t firstTri; // total: 64 bytes
		bool isLeaf() const { return triCount > 0; }
	};
	BVH_GPU( BVHContext ctx = {} ) { layout = LAYOUT_BVH_GPU; context = ctx; }
	BVH_GPU( const BVH& original ) { /* DEPRECATED */ ConvertFrom( original ); }
	BVH_GPU( const BVH_GPU& ) = delete; // owns its allocations, so it cannot be copied
	BVH_GPU( BVH_GPU&& ) noexcept;
	BVH_GPU& operator=( BVH_GPU&& ) noexcept;
	~BVH_GPU();
	void Build( const bvhvec4* vertices, const Index primCount );
	void Build( const Slice& vertices );
	void Build( const bvhvec4* vertices, const Index* indices, const Index primCount );
	void Build( const Slice& vertices, const Index* indices, const Index primCount );
	void Build( BLASInstance* instances, const Index instCount, BVHBase<Float, Index>** blasses, const Index blasCount );
	void BuildAABB( const bvhvec4* aabbs, const Index primCount );
	void BuildHQ( const bvhvec4* vertices, const Index primCount );
	void BuildHQ( const Slice& vertices );
	void BuildHQ( const bvhvec4* vertices, const Index* indices, const Index primCount );
	void BuildHQ( const Slice& vertices, const Index* indices, const Index primCount );
	void Optimize( const uint32_t iterations = 25, uint32_t mu = 8 );
	static BVH_GPU Load( const char* fileName, BVHContext ctx = {} );
	Float SAHCost( const Index nodeIdx = 0 ) { return bvh.SAHCost( nodeIdx ); }
	void ConvertFrom( const BVH& original, bool compact = true );
	int32_t Intersect( Ray& ray ) const;
	bool IsOccluded( const Ray& ray ) const { FALLBACK_SHADOW_QUERY( ray ); }
	// BVH data
	BVHNode* bvhNode = 0;			// BVH node in Aila & Laine format.
	BVH bvh;						// BVH_GPU is created from BVH and uses its data.
	Slice orderedVerts = {};		// Vertex data ordered according to triIdx.
	bool ownBVH = true;				// False when ConvertFrom receives an external bvh.
private:
	BVH_GPU& operator=( const BVH_GPU& ) = default;
};

TEMPLATED class BVH4_GPU : public BVHBase<Float, Index>
{
public:
	using Base = BVHBase<Float, Index>;
	using typename Base::BVH;
	using typename Base::Vec3;
	using typename Base::Slice;
	using typename Base::Ray;
	using Base::settings;
	using Base::c_trav;
	using Base::c_int;
	using Base::context;
	using Base::layout;
	using Base::allocatedNodes;
	using Base::usedNodes;
	using Base::triCount;
protected:
	using Base::AlignedAlloc;
	using Base::AlignedFree;
	using Base::CopyBasePropertiesFrom;
	using Base::PrecomputeTriangle;
public:
	struct BVHNode // actual struct is unused; left here to show structure of data in bvh4Data.
	{
		// 4-way BVH node, optimized for GPU rendering
		struct aabb8 { uint8_t xmin, ymin, zmin, xmax, ymax, zmax; }; // quantized
		Vec3 aabbMin; uint32_t c0Info;			// 16
		Vec3 aabbExt; uint32_t c1Info;			// 16
		aabb8 c0bounds, c1bounds; uint32_t c2Info;	// 16
		aabb8 c2bounds, c3bounds; uint32_t c3Info;	// 16; total: 64 bytes
		// childInfo, 32bit:
		// msb:        0=interior, 1=leaf
		// leaf:       16 bits: relative start of triangle data, 15 bits: triangle count.
		// interior:   31 bits: child node address, in float4s from BVH data start.
		// Triangle data: directly follows nodes with leaves. Per tri:
		// - bvhvec4 vert0, vert1, vert2
		// - uint vert0.w stores original triangle index.
		// We can make the node smaller by storing child nodes sequentially, but
		// there is no way we can shave off a full 16 bytes, unless aabbExt is stored
		// as chars as well, as in CWBVH.
	};
	BVH4_GPU( BVHContext ctx = {} ) { layout = LAYOUT_BVH4_GPU; context = ctx; }
	BVH4_GPU( const MBVH<4, Float, Index>& bvh4 ) { /* DEPRECATED */ layout = LAYOUT_BVH4_GPU; ConvertFrom( bvh4 ); }
	BVH4_GPU( const BVH4_GPU& ) = delete; // owns its allocations, so it cannot be copied
	BVH4_GPU( BVH4_GPU&& ) noexcept;
	BVH4_GPU& operator=( BVH4_GPU&& ) noexcept;
	~BVH4_GPU();
	void Build( const bvhvec4* vertices, const Index primCount );
	void Build( const Slice& vertices );
	void Build( const bvhvec4* vertices, const Index* indices, const Index primCount );
	void Build( const Slice& vertices, const Index* indices, const Index primCount );
	void BuildHQ( const bvhvec4* vertices, const Index primCount );
	void BuildHQ( const Slice& vertices );
	void BuildHQ( const bvhvec4* vertices, const Index* indices, const Index primCount );
	void BuildHQ( const Slice& vertices, const Index* indices, const Index primCount );
	bool Save( const char* fileName );
	bool Load( const char* fileName, const Index expectedTris );
	static BVH4_GPU Load( const char* fileName, BVHContext ctx = {} );
	void Optimize( const uint32_t iterations = 25, uint32_t mu = 8 );
	void ConvertFrom( const MBVH<4, Float, Index>& original, bool compact = true );
	Float SAHCost( const Index nodeIdx = 0 ) const { return bvh4.SAHCost( nodeIdx ); }
	int32_t Intersect( Ray& ray ) const;
	bool IsOccluded( const Ray& ray ) const { FALLBACK_SHADOW_QUERY( ray ); }
	// BVH data
	bvhvec4* bvh4Data = 0;			// 64-byte 4-wide BVH node for efficient GPU rendering.
	uint32_t allocatedBlocks = 0;	// node data and triangles are stored in 16-byte blocks.
	uint32_t usedBlocks = 0;		// actually used storage.
	MBVH<4, Float, Index> bvh4;		// BVH4_GPU is created from BVH4 and uses its data.
	bool ownBVH4 = true;			// False when ConvertFrom receives an external bvh.
private:
	BVH4_GPU& operator=( const BVH4_GPU& ) = default;
};

TEMPLATED class BVH4_AMD_HW : public BVHBase<Float, Index>
{
public:
	using Base = BVHBase<Float, Index>;
	using typename Base::BVH;
	using Base::allocatedNodes;
	using Base::c_int;
	using Base::c_trav;
	using Base::context;
	using Base::layout;
	using Base::settings;
	using Base::triCount;
	using Base::usedNodes;
	using typename Base::Ray;
	using typename Base::Slice;
	using typename Base::Vec3;
protected:
	using Base::AlignedAlloc;
	using Base::AlignedFree;
	using Base::CopyBasePropertiesFrom;
public:
	// Layout reference: GPURT RTIP2.0 / Mesa src/amd/vulkan/bvh/bvh.h (BOX32, BOX16 and triangle nodes).
	// These structs document the packed buffer; ConvertFrom writes bvhvec4 blocks. Regions: fp32 boxes from
	// offset 0 (the root), fp16 boxes, triangle nodes, then the parent IDs of the fp16 boxes.
	struct AMDBox32Node
	{
		struct aabb32 { float xmin, ymin, zmin, xmax, ymax, zmax; };
		// Child ID = (byteOffset >> 3) | type, with 64-byte aligned offsets.
		// The low three bits hold the type: 0 = triangle, 1 = triangle pair, 4 = BOX16, 5 = BOX32.
		// Empty children use UINT32_MAX and inverted bounds (+FLT_MAX, -FLT_MAX).
		uint32_t cPointers[4];
		aabb32 c0bounds, c1bounds, c2bounds, c3bounds; // bounds in child pointer order.
		// Byte 112: box flags (zero). Root only: byte 116 the fp16 region in 64-byte units, byte 120 the
		// fp16 parent ID table in 4-byte units. Byte 124: TinyBVH parent ID; UINT32_MAX at the root.
		uint32_t reserved[4];
	}; // 128 bytes.
	struct AMDBox16Node
	{
		uint32_t cPointers[4];
		uint32_t bounds[4][3]; // per child: x | y << 16, z | X << 16, Y | Z << 16 as halfs; minima rounded down, maxima up.
	}; // 64 bytes.
	struct AMDTriangleNode
	{
		struct vertVec3 { float x, y, z; };
		vertVec3 vertices[4]; // bytes 0..47: v0..v3. A pair (type 1): tri0 over (v0, v1, v2), tri1 over (v1, v3, v2).
		uint32_t triangle_id[2]; // bytes 48, 52: primitive index of tri0 and tri1.
		uint32_t reserved1; // byte 56: TinyBVH parent ID for traversal.
		uint32_t id; // byte 60: hardware ID, one byte per triangle: I and J vertex selectors (bits 0..3), opaque (bit 7).
	}; // 64 bytes.
	BVH4_AMD_HW( BVHContext ctx = {} ) { layout = LAYOUT_BVH4_AMD_HW; context = ctx; }
	BVH4_AMD_HW( const MBVH<4, Float, Index>& bvh4 ) { /* DEPRECATED */ layout = LAYOUT_BVH4_AMD_HW; ConvertFrom( bvh4 ); }
	BVH4_AMD_HW( const BVH4_AMD_HW& ) = delete; // owns its allocations, so it cannot be copied
	BVH4_AMD_HW( BVH4_AMD_HW&& ) noexcept;
	BVH4_AMD_HW& operator=( BVH4_AMD_HW&& ) noexcept;
	~BVH4_AMD_HW();
	void Build( const bvhvec4* vertices, const Index primCount );
	void Build( const Slice& vertices );
	void Build( const bvhvec4* vertices, const Index* indices, const Index primCount );
	void Build( const Slice& vertices, const Index* indices, const Index primCount );
	void BuildHQ( const bvhvec4* vertices, const Index primCount );
	void BuildHQ( const Slice& vertices );
	void BuildHQ( const bvhvec4* vertices, const Index* indices, const Index primCount );
	void BuildHQ( const Slice& vertices, const Index* indices, const Index primCount );
	bool Save( const char* fileName );
	bool Load( const char* fileName, const Index expectedTris );
	static BVH4_AMD_HW Load( const char* fileName, BVHContext ctx = {} );
	void Optimize( const uint32_t iterations = 25, uint32_t mu = 8 );
	void ConvertFrom( const MBVH<4, Float, Index>& original );
	Float SAHCost( const Index nodeIdx = 0 ) const { return bvh4.SAHCost( nodeIdx ); }
	// CPU traversal of the packed buffer, decoding the nodes as the hardware does; for validation.
	int32_t Intersect( Ray& ray ) const;
	bool IsOccluded( const Ray& ray ) const { FALLBACK_SHADOW_QUERY( ray ); }
	// Triangle k of a triangle node (1 for the second triangle of a pair), in hardware vertex order.
	static void NodeTriangle( const void* node, const uint32_t k, Vec3& v0, Vec3& v1, Vec3& v2, uint32_t& prim );
	// BVH data in 16-byte blocks; all nodes are 64-byte aligned.
	// The kernel uses a zero descriptor base and encodes the device address in the node pointer.
	bvhvec4* bvh4Data = 0;
	uint32_t allocatedBlocks = 0; // node data and triangles are stored in 16-byte blocks.
	uint32_t usedBlocks = 0; // actually used storage.
	MBVH<4, Float, Index> bvh4; // BVH4_AMD_HW is created from BVH4 and uses its data.
	bool ownBVH4 = true; // False when ConvertFrom receives an external bvh.
	bool pairEdges = true; // Build: pair triangles that share an edge before building (BVH::BuildPairs).
	float pairGrowth = 1.5f; // Build: maximum union surface area of a pair, relative to the mean of its triangles.
	float fp16Growth = 1.1f; // ConvertFrom: fp16 box node when its child boxes grow by at most this (in surface area).
private:
	static constexpr uint32_t NODE_TRIANGLE = 0, NODE_PAIR = 1, NODE_BOX16 = 4, NODE_BOX32 = 5;
	static uint32_t PackNodeID( uint32_t offset, uint32_t type ) { return (offset >> 3) | type; }
	uint32_t EncodeLeaf( const uint32_t nodeIdx, uint32_t* dst, const uint32_t parentID ) const;
	BVH4_AMD_HW& operator=( const BVH4_AMD_HW& ) = default;
};

TEMPLATED class CWBVH8_AMD_HW : public BVHBase<Float, Index>
{
public:
	using Base = BVHBase<Float, Index>;
	using Base::allocatedNodes;
	using Base::c_int;
	using Base::c_trav;
	using Base::context;
	using Base::layout;
	using Base::settings;
	using Base::triCount;
	using Base::usedNodes;
	using typename Base::BVH;
	using typename Base::Ray;
	using typename Base::Slice;
	using typename Base::Vec3;
protected:
	using Base::AlignedAlloc;
	using Base::AlignedFree;
	using Base::CopyBasePropertiesFrom;
public:
	// Layout reference: Mesa, src/amd/vulkan/bvh/bvh_defines.h (GFX12 nodes).
	// These structs document the packed buffer; ConvertFrom writes bvhvec4 blocks.
	struct AMDBoxChild
	{
		uint32_t dword0; // 12-bit xmin, 12-bit ymin, 4-bit cull flags, 4 unused bits.
		uint32_t dword1; // 12-bit zmin, 12-bit xmax, 8-bit ray mask.
		uint32_t dword2; // 12-bit ymax, 12-bit zmax, 4-bit type (box, or pair index), 4-bit range in 128-byte records.
	}; // 12 bytes.
	struct AMDBoxNode
	{
		uint32_t internal_base_id; // byte 0: first internal child, in 8-byte units.
		uint32_t primitive_base_id; // byte 4: record of the first primitive child, in 8-byte units.
		uint32_t parent_id; // byte 8: packed parent ID; UINT32_MAX at the root.
		bvhvec3 origin; // bytes 12..23: origin of the quantized child bounds.
		uint32_t child_count_exponents; // byte 24: three biased exponents and (child count - 1) in bits 28..30.
		uint32_t obb_matrix_index; // byte 28: 0x7f disables oriented bounding boxes.
		AMDBoxChild children[8]; // bytes 32..127: internal children first, primitive children next.
	}; // 128 bytes.
	struct AMDTriangleNode
	{
		uint32_t words[32]; // 52-bit header, compressed vertices and primitive IDs, 29-bit pair descriptors.
	}; // 128 bytes; up to 8 triangle pairs, possibly shared by several box nodes.
	CWBVH8_AMD_HW( BVHContext ctx = {} ) { layout = LAYOUT_CWBVH8_AMD_HW; context = ctx; }
	CWBVH8_AMD_HW( const MBVH<8, Float, Index>& bvh8 ) { /* DEPRECATED */ layout = LAYOUT_CWBVH8_AMD_HW; ConvertFrom( bvh8 ); }
	CWBVH8_AMD_HW( const CWBVH8_AMD_HW& ) = delete; // owns its allocations, so it cannot be copied
	CWBVH8_AMD_HW( CWBVH8_AMD_HW&& ) noexcept;
	CWBVH8_AMD_HW& operator=( CWBVH8_AMD_HW&& ) noexcept;
	~CWBVH8_AMD_HW();
	void Build( const bvhvec4* vertices, const Index primCount );
	void Build( const Slice& vertices );
	void Build( const bvhvec4* vertices, const uint32_t* indices, const Index primCount );
	void Build( const Slice& vertices, const uint32_t* indices, const Index primCount );
	void BuildHQ( const bvhvec4* vertices, const Index primCount );
	void BuildHQ( const Slice& vertices );
	void BuildHQ( const bvhvec4* vertices, const uint32_t* indices, const Index primCount );
	void BuildHQ( const Slice& vertices, const uint32_t* indices, const Index primCount );
	void Optimize( const uint32_t iterations = 25, uint32_t mu = 8 );
	bool Save( const char* fileName );
	bool Load( const char* fileName, const Index expectedTris );
	static CWBVH8_AMD_HW Load( const char* fileName, BVHContext ctx = {} );
	void ConvertFrom( const MBVH<8, Float, Index>& original, bool compact = true );
	Float SAHCost( const Index nodeIdx = 0 ) const { return bvh8.SAHCost( nodeIdx ); }
	// CPU traversal of the packed buffer, decoding the nodes as the hardware does; for validation.
	int32_t Intersect( Ray& ray ) const;
	bool IsOccluded( const Ray& ray ) const { FALLBACK_SHADOW_QUERY( ray ); }
	// Triangle pair p of a primitive record: returns the triangle count (1 or 2), vertices in v[0..5].
	static uint32_t RecordPair( const AMDTriangleNode& record, const uint32_t p, Vec3* v, uint32_t* prim );
	// BVH data in 16-byte blocks: box nodes, then primitive records; all 128-byte aligned.
	// The kernel uses a zero descriptor base and encodes the device address in the node pointer.
	bvhvec4* bvh8Data = 0;
	uint32_t allocatedBlocks = 0; // node data and triangles are stored in 16-byte blocks.
	uint32_t usedBlocks = 0; // actually used storage.
	MBVH<8, Float, Index> bvh8; // CWBVH8_AMD_HW is created from BVH8 and uses its data.
	bool ownBVH8 = true; // False when ConvertFrom receives an external bvh.
	bool pairEdges = true; // Build: pair triangles that share an edge before building (BVH::BuildPairs).
	float pairGrowth = 1.5f; // Build: maximum union surface area of a pair, relative to the mean of its triangles.
	uint32_t recordLookahead = 16; // ConvertFrom: box nodes considered for continuing an open primitive record.
private:
	static constexpr uint32_t NODE_TRIANGLE = 0, NODE_BOX32 = 5;
	static uint32_t PackNodeID( uint32_t offset, uint32_t type ) { return (offset >> 3) | type; }
	// A primitive record being filled by ConvertFrom. The bit fields are masks; computeNewNodeSize turns them into widths.
	struct encodingInfo
	{
		uint32_t xPrefixBits = 0, yPrefixBits = 0, zPrefixBits = 0; // OR of (vert ^ uniqueVerts[0]), per axis
		uint32_t trailingZeroBits = 0; // OR of all vertex components
		uint32_t baseIdx = 0; // primitive ID of the record's first triangle // TODO : Can be improved by choosing best PrimIdx
		uint32_t idxDiffBits = 0; // OR of (primitive ID ^ baseIdx)
		uint32_t uniqueVertCount = 0, pairCount = 0;
		bvhuint3 uniqueVerts[15]; // 4-bit vertex indices with 0xF reserved for procedural nodes: at most 15.
	};
	bool getVertsInfo( encodingInfo& info, const uint32_t leafIdx ) const;
	static int32_t computeNewNodeSize( const encodingInfo& info );
	void EncodePrimNode( const uint32_t* leaves, const uint32_t pairCount, uint32_t& nodeDataPtr );
	CWBVH8_AMD_HW& operator=( const CWBVH8_AMD_HW& ) = default;
};

TEMPLATED class BVH6_INTEL_HW : public BVHBase<Float, Index>
{
public:
	using Base = BVHBase<Float, Index>;
	using Base::allocatedNodes;
	using Base::c_int;
	using Base::c_trav;
	using Base::context;
	using Base::layout;
	using Base::settings;
	using Base::triCount;
	using Base::usedNodes;
	using typename Base::BVH;
	using typename Base::Ray;
	using typename Base::Slice;
	using typename Base::Vec3;
protected:
	using Base::AlignedAlloc;
	using Base::AlignedFree;
	using Base::CopyBasePropertiesFrom;
public:
	// Layout reference: Intel level-zero-raytracing-support, rtbuild/qbvh6.h, qnode.h, leaf.h (format DG2:
	// Alchemist, Battlemage, Meteor Lake, Lunar Lake). All data is in 64-byte blocks; the root is at byte 128.
	struct IntelHeader
	{
		uint32_t rtasFormat, reserved1; // ZE_RTAS_FORMAT_DG2 = 1.
		float bounds[6]; // scene bounds: lower xyz, upper xyz.
		uint32_t nodeDataStart, nodeDataCur, leafDataStart, leafDataCur; // allocator cursors, in 64-byte blocks.
		uint32_t proceduralDataStart, proceduralDataCur, backPointerDataStart, backPointerDataEnd;
		uint32_t numTimeSegments, numPrims, reserved[12];
		uint64_t dispatchGlobalsPtr;
	}; // 128 bytes.
	struct IntelNode6
	{
		float lower[3]; // origin of the quantization grid.
		int32_t childOffset; // first child, in 64-byte blocks relative to this node.
		uint8_t nodeType, pad; // 0 = mixed: child types in childData; 4 = fat leaf: all children are quads.
		int8_t exp[3]; // grid size per axis is 2^exp.
		uint8_t nodeMask;
		uint8_t childData[6]; // bits 0..1: size in blocks (0 for the last child), bits 2..5: child type / start prim.
		uint8_t q[6][6]; // lower_x, upper_x, lower_y, upper_y, lower_z, upper_z; lower 0x80, upper 0 = empty.
	}; // 64 bytes.
	struct IntelQuadLeaf
	{
		uint32_t shaderGeomMask; // 24-bit shader index, 8-bit geometry mask.
		uint32_t geomIndexFlags; // 29-bit geometry index, 1-bit type, 2-bit geometry flags (1 = opaque).
		uint32_t primIndex0;
		uint32_t pairInfo; // bits 0..4: primIndex1 - primIndex0, 16..21: j0, j1, j2, 22: last quad of a child.
		float v[4][3]; // triangle 0: v0, v1, v2; triangle 1: v[j0], v[j1], v[j2].
	}; // 64 bytes; stores one or two triangles.
	BVH6_INTEL_HW( BVHContext ctx = {} ) { layout = LAYOUT_BVH6_INTEL_HW; context = ctx; }
	BVH6_INTEL_HW( const BVH6_INTEL_HW& ) = delete; // owns its allocations, so it cannot be copied
	BVH6_INTEL_HW( BVH6_INTEL_HW&& ) noexcept;
	BVH6_INTEL_HW& operator=( BVH6_INTEL_HW&& ) noexcept;
	~BVH6_INTEL_HW();
	void Build( const bvhvec4* vertices, const Index primCount );
	void Build( const Slice& vertices );
	void Build( const bvhvec4* vertices, const Index* indices, const Index primCount );
	void Build( const Slice& vertices, const Index* indices, const Index primCount );
	void BuildHQ( const bvhvec4* vertices, const Index primCount );
	void BuildHQ( const Slice& vertices );
	void BuildHQ( const bvhvec4* vertices, const Index* indices, const Index primCount );
	void BuildHQ( const Slice& vertices, const Index* indices, const Index primCount );
	bool Save( const char* fileName );
	bool Load( const char* fileName, const Index expectedTris );
	void Optimize( const uint32_t iterations = 25, uint32_t mu = 8 );
	void ConvertFrom( const MBVH<6, Float, Index>& original );
	Float SAHCost( const Index nodeIdx = 0 ) const { return bvh6.SAHCost( nodeIdx ); }
	// CPU traversal of the packed buffer, for validation; the hardware does not use this.
	int32_t Intersect( Ray& ray ) const;
	bvhvec4* rtasData = 0; // the acceleration structure, ready for upload; 64-byte aligned.
	uint32_t allocatedBlocks = 0; // in 64-byte blocks.
	uint32_t usedBlocks = 0; // actually used storage.
	MBVH<6, Float, Index> bvh6; // BVH6_INTEL_HW is created from BVH6 and uses its data.
	bool ownBVH6 = true; // False when ConvertFrom receives an external bvh.
	static constexpr uint32_t LEAF_TRIS = 9; // triangles per fat leaf, as in Intel's builder.
private:
	static constexpr uint8_t NODE_INTERNAL = 0, NODE_QUAD = 4;
	static void InitNode( IntelNode6& node, bvhvec3 bmin, bvhvec3 bmax, const uint8_t type );
	static void SetChildBounds( IntelNode6& node, const uint32_t i, bvhvec3 bmin, bvhvec3 bmax );
	static void ChildBounds( const IntelNode6& node, const uint32_t i, bvhvec3& bmin, bvhvec3& bmax );
	static bool ChildValid( const IntelNode6& node, const uint32_t i ) { return !(node.q[0][i] & 0x80) || (node.q[1][i] & 0x80); }
	uint32_t EmitFatLeaf( const uint32_t srcNode, const uint32_t dstBlock, uint32_t blockPtr );
	BVH6_INTEL_HW& operator=( const BVH6_INTEL_HW& ) = default;
};

TEMPLATED class BVH8_CWBVH : public BVHBase<Float, Index>
{
public:
	using Base = BVHBase<Float, Index>;
	using typename Base::BVH;
	using typename Base::Vec3;
	using typename Base::Slice;
	using typename Base::Ray;
	using Base::settings;
	using Base::c_trav;
	using Base::c_int;
	using Base::context;
	using Base::layout;
	using Base::triCount;
	using Base::idxCount;
protected:
	using Base::AlignedAlloc;
	using Base::AlignedFree;
	using Base::CopyBasePropertiesFrom;
	using Base::PrecomputeTriangle;
public:
	BVH8_CWBVH( BVHContext ctx = {} ) { layout = LAYOUT_CWBVH; context = ctx; }
	BVH8_CWBVH( MBVH<8, Float, Index>& bvh8 ) { /* DEPRECATED */ layout = LAYOUT_CWBVH; ConvertFrom( bvh8 ); }
	BVH8_CWBVH( const BVH8_CWBVH& ) = delete; // owns its allocations, so it cannot be copied
	BVH8_CWBVH( BVH8_CWBVH&& ) noexcept;
	BVH8_CWBVH& operator=( BVH8_CWBVH&& ) noexcept;
	~BVH8_CWBVH();
	bool Save( const char* fileName );
	bool Load( const char* fileName, const Index expectedTris );
	void Build( const bvhvec4* vertices, const Index primCount );
	void Build( const Slice& vertices );
	void Build( const bvhvec4* vertices, const Index* indices, const Index primCount );
	void Build( const Slice& vertices, const Index* indices, const Index primCount );
	void BuildHQ( const bvhvec4* vertices, const Index primCount );
	void BuildHQ( const Slice& vertices );
	void BuildHQ( const bvhvec4* vertices, const Index* indices, const Index primCount );
	void BuildHQ( const Slice& vertices, const Index* indices, const Index primCount );
	static BVH8_CWBVH Load( const char* fileName, BVHContext ctx = {} );
	void Optimize( const uint32_t iterations = 25, uint32_t mu = 8 );
	void ConvertFrom( const MBVH<8, Float, Index>& original, bool compact = true );
	// Greedy octant assignment for the children of one node, used by ConvertFrom.
	// An AVX2 version of this function is in tiny_bvh_x86_float.h.
	uint32_t SlotPermutation( const MBVH<8, Float, Index>& bvh8, const Index nodeIdx,
		const Vec3& nodeLo, const float rsx, const float rsy, const float rsz, uint8_t* const qbase ) const;
	Float SAHCost( const Index nodeIdx = 0 ) const;
	int32_t Intersect( Ray& ray ) const;
	bool IsOccluded( const Ray& ray ) const { FALLBACK_SHADOW_QUERY( ray ); }
	// BVH8 data
	bvhvec4* bvh8Data = 0;			// nodes in CWBVH format.
	bvhvec4* bvh8Tris = 0;			// triangle data for CWBVH nodes.
	uint32_t allocatedBlocks = 0;	// node data is stored in blocks of 16 byte.
	uint32_t usedBlocks = 0;		// actually used blocks.
	MBVH<8, Float, Index> bvh8;		// BVH8_CWBVH is created from BVH8 and uses its data.
	bool ownBVH8 = true;			// false when ConvertFrom receives an external bvh8.
	uint32_t usedTriBlocks = 0;		// 16-byte blocks actually used in bvh8Tris
	uint32_t allocatedTris = 0;		// 16-byte blocks allocated for bvh8Tris
private:
	BVH8_CWBVH& operator=( const BVH8_CWBVH& ) = default;
};

} // namespace impl

// Single precision, 32-bit indices.
using BVH_GPU = impl::BVH_GPU<float, uint32_t>;
using BVH4_GPU = impl::BVH4_GPU<float, uint32_t>;
using BVH4_AMD_HW = impl::BVH4_AMD_HW<float, uint32_t>;
using CWBVH8_AMD_HW = impl::CWBVH8_AMD_HW<float, uint32_t>;
using BVH6_INTEL_HW = impl::BVH6_INTEL_HW<float, uint32_t>;
using BVH8_CWBVH = impl::BVH8_CWBVH<float, uint32_t>;

#ifdef BVH_USEAVX2
template <> uint32_t impl::BVH8_CWBVH<float, uint32_t>::SlotPermutation( const impl::MBVH<8, float, uint32_t>& bvh8, const uint32_t nodeIdx,
	const bvhvec3& nodeLo, const float rsx, const float rsy, const float rsz, uint8_t* const qbase ) const;
#endif

} // namespace tinybvh

#ifdef _MSC_VER
#pragma warning ( pop )
#endif

#endif // TINY_BVH_GPU_H_

// ============================================================================
//
//        I M P L E M E N T A T I O N
//
// ============================================================================

#ifdef TINYBVH_IMPLEMENTATION
#ifndef TINY_BVH_GPU_H_IMPL
#define TINY_BVH_GPU_H_IMPL

#ifdef _MSC_VER
#pragma warning ( push )
#pragma warning ( disable: 4324 /* structure padding */ )
#pragma warning ( disable: 4701 /* using possibly uninitialized variable */ )
#endif

namespace tinybvh {
namespace impl {

// BVH_GPU implementation
// ----------------------------------------------------------------------------

TEMPLATED BVH_GPU<Float, Index>::BVH_GPU( BVH_GPU&& other ) noexcept
{
	*this = other;
	// The embedded bvh must let go too: owned or not, this object now refers to
	// the same buffers, and only one of the two may release them.
	other.bvh.ReleaseOwnership();
	other.bvhNode = 0;
	other.orderedVerts = Slice( nullptr, 0, 0 );
}

// move assignment: release what we hold, then take over 'other'.
TEMPLATED BVH_GPU<Float, Index>& BVH_GPU<Float, Index>::operator=( BVH_GPU&& other ) noexcept
{
	if (this != &other) this->~BVH_GPU(), new (this) BVH_GPU( tinybvh_move( other ) );
	return *this;
}

TEMPLATED BVH_GPU<Float, Index>::~BVH_GPU()
{
	if (!ownBVH) bvh.ReleaseOwnership(); // clear out pointers we don't own.
	AlignedFree( bvhNode );
	bvhNode = 0;
	AlignedFree( (void*)orderedVerts.data ); // allocated by ConvertFrom; we own it.
	orderedVerts = Slice( nullptr, 0, 0 );
}

// forwarders
TEMPLATED void BVH_GPU<Float, Index>::Build( const bvhvec4* v, const Index p ) { Build( Slice( v, p * 3, sizeof( bvhvec4 ) ) ); }
TEMPLATED void BVH_GPU<Float, Index>::Build( const Slice& v ) { Build( v, 0, 0 ); }
TEMPLATED void BVH_GPU<Float, Index>::Build( const bvhvec4* v, const Index* i, const Index p ) { Build( Slice( v, p * 3, sizeof( bvhvec4 ) ), i, p ); }
TEMPLATED void BVH_GPU<Float, Index>::BuildHQ( const bvhvec4* v, const Index p ) { BuildHQ( Slice( v, p * 3, sizeof( bvhvec4 ) ) ); }
TEMPLATED void BVH_GPU<Float, Index>::BuildHQ( const bvhvec4* v, const Index* i, const Index p ) { BuildHQ( Slice( v, p * 3, sizeof( bvhvec4 ) ), i, p ); }
TEMPLATED void BVH_GPU<Float, Index>::BuildHQ( const Slice& v, const Index* i, Index p ) { ScopedFlag hq( settings.useSpatialSplits, true ); Build( v, i, p ); }
TEMPLATED void BVH_GPU<Float, Index>::BuildHQ( const Slice& v ) { ScopedFlag hq( settings.useSpatialSplits, true ); Build( v ); }

TEMPLATED void BVH_GPU<Float, Index>::Build( const Slice& vertices, const Index* indices, Index prims )
{
	// a tree received through ConvertFrom is not ours to rebuild: drop the reference, start afresh.
	if (!ownBVH) bvh.DropReference( context ), ownBVH = true;
	// propagate settings for this layout to the underlying layout
	this->PassSettingsTo( bvh );
	// build underlying layout
	bvh.Build( vertices, indices, prims );
	// convert to BVH_GPU layout
	ConvertFrom( bvh, false );
}

TEMPLATED void BVH_GPU<Float, Index>::BuildAABB( const bvhvec4* aabbs, const Index primCount )
{
	// a tree received through ConvertFrom is not ours to rebuild: drop the reference, start afresh.
	if (!ownBVH) bvh.DropReference( context ), ownBVH = true;
	// build a TLAS based on the array of BLASInstance records.
	this->PassSettingsTo( bvh );
	// build underlying layout
	bvh.BuildAABB( aabbs, primCount );
	// convert to BVH_GPU layout
	ConvertFrom( bvh );
}

TEMPLATED void BVH_GPU<Float, Index>::Build( BLASInstance* instances, const Index instCount, BVHBase<Float, Index>** blasses, const Index blasCount )
{
	// a tree received through ConvertFrom is not ours to rebuild: drop the reference, start afresh.
	if (!ownBVH) bvh.DropReference( context ), ownBVH = true;
	// build a TLAS based on the array of BLASInstance records.
	this->PassSettingsTo( bvh );
	// build underlying layout
	bvh.Build( instances, instCount, blasses, blasCount );
	// the GPU kernels see the instances through invTransform only.
	BVH_FATAL_ERROR_IF( bvh.hasMotion, "BVH_GPU::Build( BLASInstance*, .. ), no motion blur on the GPU yet." );
	// convert to BVH_GPU layout
	ConvertFrom( bvh, false );
}

TEMPLATED void BVH_GPU<Float, Index>::Optimize( const uint32_t iterations, uint32_t mu )
{
	bvh.Optimize( iterations, mu );
	ConvertFrom( bvh, false );
}

TEMPLATED void BVH_GPU<Float, Index>::ConvertFrom( const BVH& original, bool compact )
{
	// get a copy of the original bvh
	if (&original != &bvh) ownBVH = false; // bvh isn't ours; don't delete in destructor.
	bvh.ReferenceFrom( original );
	const uint32_t sourceNodes = (uint32_t)(compact ? original.usedNodes : original.allocatedNodes);
	// a child reference holds at most 127 triangles: larger leaves become subtrees, one node per extra group.
	uint32_t extraNodes = 0, stack[TINYBVH_STACK_SIZE], stackPtr = 0, nodeIdx = 0;
	while (1)
	{
		const typename BVH::BVHNode& n = original.bvhNode[nodeIdx];
		if (n.isLeaf()) extraNodes += ((uint32_t)n.triCount + 126) / 127 - 1;
		else { BVH_FATAL_ERROR_IF( stackPtr == TINYBVH_STACK_SIZE, "BVH_GPU::ConvertFrom( .. ), the tree is too deep." ); stack[stackPtr++] = (uint32_t)n.leftFirst + 1, nodeIdx = (uint32_t)n.leftFirst; continue; }
		if (!stackPtr) break;
		nodeIdx = stack[--stackPtr];
	}
	const uint32_t spaceNeeded = (sourceNodes >> 1) + 1 + extraNodes; // the +1 covers a leaf-only bvh
	CopyBasePropertiesFrom( original );
	if (allocatedNodes < spaceNeeded)
	{
		AlignedFree( bvhNode );
		bvhNode = (BVHNode*)AlignedAlloc( sizeof( BVHNode ) * spaceNeeded );
		allocatedNodes = spaceNeeded;
	}
	memset( bvhNode, 0, sizeof( BVHNode ) * spaceNeeded );
	this->hasHoles = false;
	// a leaf as a child reference: msb marks a leaf, bits 24..30 hold its triangle count, 0..23 its first.
	// A larger leaf is halved in whole groups of 127; both halves keep its box.
	uint32_t newNodePtr = 1, slot = 0;
	nodeIdx = 0;
	auto LeafRef = [&]( auto&& self, const uint32_t first, const uint32_t count, const Vec3& bmin, const Vec3& bmax ) -> uint32_t
	{
		if (count <= 127) return 0x80000000u | (count << 24) | (first & 0xffffff);
		const uint32_t leftCount = (count + 126) / 127 / 2 * 127, at = newNodePtr++;
		BVHNode& node = this->bvhNode[at];
		node.lmin = node.rmin = bmin, node.lmax = node.rmax = bmax;
		node.left = self( self, first, leftCount, bmin, bmax );
		node.right = self( self, first + leftCount, count - leftCount, bmin, bmax );
		return at;
	};
	// a bvh consisting of a single leaf: traversal always starts at an interior
	// node, so wrap the leaf in one. The second child is an empty leaf, which is
	// harmless to 'hit' - it intersects nothing and pops immediately.
	if (original.bvhNode[0].isLeaf())
	{
		const typename BVH::BVHNode& leaf = original.bvhNode[0];
		BVHNode& root = this->bvhNode[0];
		root.lmin = leaf.aabbMin, root.lmax = leaf.aabbMax;
		root.rmin = root.rmax = leaf.aabbMin;
		root.left = LeafRef( LeafRef, (uint32_t)leaf.leftFirst, (uint32_t)leaf.triCount, leaf.aabbMin, leaf.aabbMax );
		root.right = 0x80000000u; // triCount 0, firstTri 0
	}
	// depth-first conversion
	else while (1)
	{
		const typename BVH::BVHNode& orig = original.bvhNode[nodeIdx];
		uint32_t leftIdx = (uint32_t)orig.leftFirst, rightIdx = leftIdx + 1;
		const typename BVH::BVHNode& c0 = original.bvhNode[leftIdx];
		const typename BVH::BVHNode& c1 = original.bvhNode[rightIdx];
		// put the larger node to the left to improve cache coherence during traversal
		if (tinybvh_halfarea( c0.aabbMax - c0.aabbMin ) <= tinybvh_halfarea( c1.aabbMax - c1.aabbMin ))
			tinybvh_swap( leftIdx, rightIdx );
		const typename BVH::BVHNode& left = original.bvhNode[leftIdx];
		const typename BVH::BVHNode& right = original.bvhNode[rightIdx];
		BVHNode& node = this->bvhNode[slot];
		node.lmin = left.aabbMin, node.lmax = left.aabbMax;
		node.rmin = right.aabbMin, node.rmax = right.aabbMax;
		node.triCount = 0, node.firstTri = 0; // unused: leaves live in left / right
		const bool leftLeaf = left.isLeaf(), rightLeaf = right.isLeaf();
		uint32_t leftSlot = 0, rightSlot = 0;
		if (leftLeaf) node.left = LeafRef( LeafRef, (uint32_t)left.leftFirst, (uint32_t)left.triCount, left.aabbMin, left.aabbMax );
		else node.left = leftSlot = newNodePtr++;
		if (rightLeaf) node.right = LeafRef( LeafRef, (uint32_t)right.leftFirst, (uint32_t)right.triCount, right.aabbMin, right.aabbMax );
		else node.right = rightSlot = newNodePtr++;
		// defer the right subtree, then descend into the left one
		if (!rightLeaf) stack[stackPtr++] = rightIdx, stack[stackPtr++] = rightSlot;
		if (!leftLeaf) { nodeIdx = leftIdx, slot = leftSlot; continue; }
		if (!stackPtr) break;
		slot = stack[--stackPtr], nodeIdx = stack[--stackPtr];
	}
	usedNodes = newNodePtr;
	// reorder triangle data to avoid the idx array indirection - release the buffer from a previous
	// conversion; this object owns it.
	AlignedFree( (void*)orderedVerts.data );
	orderedVerts = Slice( nullptr, 0, 0 );
	if (bvhOverAabbs) return; // skip for TLAS builds.
	bvhvec4* vertexData = (bvhvec4*)AlignedAlloc( sizeof( bvhvec4 ) * 3 * idxCount );
	orderedVerts = Slice( vertexData, 3 * idxCount );
	for (Index i = 0; i < idxCount; i++)
	{
		Index primIdx = bvh.primIdx[i], i0, i1, i2;
		if (primIdx >= triCount) continue; // TODO: can this actually happen?
		GET_PRIM_INDICES_I0_I1_I2( bvh, primIdx );
		vertexData[i * 3 + 0] = bvh.verts[i0];
		vertexData[i * 3 + 1] = bvh.verts[i1] - vertexData[i * 3 + 0];
		vertexData[i * 3 + 2] = bvh.verts[i2] - vertexData[i * 3 + 0];
		vertexData[i * 3 + 0].w = tinybvh_as_float( primIdx ); // store original primitive index.
		vertexData[i * 3 + 1].w = bvh.verts[i1].w; // keep; may contain triangle color.
	}
}

TEMPLATED int32_t BVH_GPU<Float, Index>::Intersect( Ray& ray ) const
{
	VALIDATE_RAY( ray );
	uint32_t nodeIdx = 0, stack[TINYBVH_STACK_SIZE], stackPtr = 0;
	float cost = 0;
	while (1)
	{
		cost += c_trav;
		if (nodeIdx & 0x80000000u)
		{
			uint32_t primCount = (nodeIdx >> 24) & 127, firstTri = nodeIdx & 0xffffff;
			for (uint32_t i = 0; i < primCount; i++, cost += c_int)
			{
				const uint32_t pi = (firstTri + i) * 3;
				const bvhvec4 v0_ = orderedVerts[pi];
				const Vec3 v0 = v0_, e1 = orderedVerts[pi + 1], e2 = orderedVerts[pi + 2];
				MOLLER_TRUMBORE_TEST( ray.hit.t, continue );
				// register a hit: ray is shortened to t.
				ray.hit.t = t, ray.hit.u = u, ray.hit.v = v;
				ray.SetHitPrim( tinybvh_as_uint( v0_.w ) );
			}
			if (stackPtr == 0) break; else nodeIdx = stack[--stackPtr];
			continue;
		}
		// not a leaf; nodeIdx contains the index of an interior node.
		BVHNode& node = bvhNode[nodeIdx];
		float dist1 = bvh_far<Float>, dist2 = bvh_far<Float>;
		const Vec3 t1a = (node.lmin - ray.O) * ray.rD, t2a = (node.lmax - ray.O) * ray.rD;
		const Vec3 t1b = (node.rmin - ray.O) * ray.rD, t2b = (node.rmax - ray.O) * ray.rD;
		const float tmina = tinybvh_max( tinybvh_max( tinybvh_min( t1a.x, t2a.x ), tinybvh_min( t1a.y, t2a.y ) ), tinybvh_min( t1a.z, t2a.z ) );
		const float tmaxa = tinybvh_min( tinybvh_min( tinybvh_max( t1a.x, t2a.x ), tinybvh_max( t1a.y, t2a.y ) ), tinybvh_max( t1a.z, t2a.z ) );
		const float tminb = tinybvh_max( tinybvh_max( tinybvh_min( t1b.x, t2b.x ), tinybvh_min( t1b.y, t2b.y ) ), tinybvh_min( t1b.z, t2b.z ) );
		const float tmaxb = tinybvh_min( tinybvh_min( tinybvh_max( t1b.x, t2b.x ), tinybvh_max( t1b.y, t2b.y ) ), tinybvh_max( t1b.z, t2b.z ) );
		if (tmaxa >= tmina && tmina < ray.hit.t && tmaxa >= 0) dist1 = tmina;
		if (tmaxb >= tminb && tminb < ray.hit.t && tmaxb >= 0) dist2 = tminb;
		uint32_t lidx = node.left, ridx = node.right;
		if (dist1 > dist2)
		{
			float t = dist1; dist1 = dist2; dist2 = t;
			uint32_t i = lidx; lidx = ridx; ridx = i;
		}
		if (dist1 == bvh_far<Float>)
		{
			if (stackPtr == 0) break; else nodeIdx = stack[--stackPtr];
		}
		else
		{
			nodeIdx = lidx;
			if (dist2 != bvh_far<Float>) stack[stackPtr++] = ridx;
		}
	}
	return (int32_t)cost; // cast to not break interface.
}

// BVH4_GPU implementation
// ----------------------------------------------------------------------------

TEMPLATED BVH4_GPU<Float, Index>::BVH4_GPU( BVH4_GPU&& other ) noexcept
{
	*this = other;
	// The embedded bvh4 must let go too: owned or not, this object now refers to
	// the same buffers, and only one of the two may release them.
	other.bvh4.ReleaseOwnership();
	other.bvh4Data = 0;
}

// move assignment: release what we hold, then take over 'other'.
TEMPLATED BVH4_GPU<Float, Index>& BVH4_GPU<Float, Index>::operator=( BVH4_GPU&& other ) noexcept
{
	if (this != &other) this->~BVH4_GPU(), new (this) BVH4_GPU( tinybvh_move( other ) );
	return *this;
}

TEMPLATED BVH4_GPU<Float, Index>::~BVH4_GPU()
{
	if (!ownBVH4) bvh4.ReleaseOwnership(); // clear out pointers we don't own.
	AlignedFree( bvh4Data );
}

// forwarders
TEMPLATED void BVH4_GPU<Float, Index>::Build( const bvhvec4* v, const Index p ) { Build( Slice( v, p * 3, sizeof( bvhvec4 ) ) ); }
TEMPLATED void BVH4_GPU<Float, Index>::Build( const bvhvec4* v, const Index* i, const Index p ) { Build( Slice( v, p * 3, sizeof( bvhvec4 ) ), i, p ); }
TEMPLATED void BVH4_GPU<Float, Index>::Build( const Slice& v ) { Build( v, 0, 0 ); }
TEMPLATED void BVH4_GPU<Float, Index>::BuildHQ( const bvhvec4* v, const Index p ) { BuildHQ( Slice( v, p * 3, sizeof( bvhvec4 ) ) ); }
TEMPLATED void BVH4_GPU<Float, Index>::BuildHQ( const bvhvec4* v, const Index* i, const Index p ) { BuildHQ( Slice( v, p * 3, sizeof( bvhvec4 ) ), i, p ); }
TEMPLATED void BVH4_GPU<Float, Index>::BuildHQ( const Slice& v ) { BuildHQ( v, 0, 0 ); }
TEMPLATED void BVH4_GPU<Float, Index>::BuildHQ( const Slice& v, const Index* i, Index p ) { ScopedFlag hq( settings.useSpatialSplits, true ); Build( v, i, p ); }

TEMPLATED void BVH4_GPU<Float, Index>::Build( const Slice& vertices, const Index* indices, Index prims )
{
	// a tree received through ConvertFrom is not ours to rebuild: drop the reference, start afresh.
	if (!ownBVH4) bvh4.DropReference( context ), ownBVH4 = true;
	// propagate settings for this layout to the underlying layout
	this->PassSettingsTo( bvh4 ), this->PassSettingsTo( bvh4.bvh );
	// build underlying layout
	bvh4.Build( vertices, indices, prims );
	// convert to BVH4_GPU layout
	ConvertFrom( bvh4, true );
}

TEMPLATED void BVH4_GPU<Float, Index>::Optimize( const uint32_t iterations, uint32_t mu )
{
	bvh4.Optimize( iterations, mu );
	ConvertFrom( bvh4, true );
}

TEMPLATED bool BVH4_GPU<Float, Index>::Save( const char* fileName )
{
	std::fstream s{ fileName, s.binary | s.out };
	if (!s) return false;
	const uint32_t header = this->CacheHeader();
	s.write( (char*)&header, sizeof( uint32_t ) );
	s.write( (char*)&triCount, sizeof( Index ) );
	s.write( (char*)this, sizeof( BVH4_GPU ) );
	s.write( (char*)bvh4Data, (size_t)usedBlocks * 16 );
	// write errors may only surface when the stream is flushed
	s.close();
	return !s.fail();
}

TEMPLATED bool BVH4_GPU<Float, Index>::Load( const char* fileName, const Index expectedTris )
{
	// open file and check contents
	std::fstream s{ fileName, s.binary | s.in };
	if (!s) return false;
	BVHContext tmp = context;
	uint32_t header;
	Index fileTriCount;
	s.read( (char*)&header, sizeof( uint32_t ) );
	if (header != this->CacheHeader()) return false;
	s.read( (char*)&fileTriCount, sizeof( Index ) );
	if (fileTriCount != expectedTris) return false;
	// all checks passed: release what *this holds, then overwrite it with the file's image.
	uint32_t* opmapKeep = this->opmap, opmapNKeep = this->opmapN; // opacity maps are caller-owned; keep what was set.
	*this = BVH4_GPU( tmp );
	s.read( (char*)this, sizeof( BVH4_GPU ) );
	context = tmp; // can't load context; function pointers will differ.
	this->opmap = opmapKeep, this->opmapN = opmapNKeep;
	bvh4Data = (bvhvec4*)AlignedAlloc( (size_t)usedBlocks * 16 );
	allocatedBlocks = usedBlocks;
	s.read( (char*)bvh4Data, (size_t)usedBlocks * 16 );
	bvh4.DropReference( context ); // holds raw file data: reset, don't free.
	return true;
}

TEMPLATED void BVH4_GPU<Float, Index>::ConvertFrom( const MBVH<4, Float, Index>& original, bool compact )
{
	// get a copy of the original bvh4
	if (&original != &bvh4) ownBVH4 = false; // bvh isn't ours; don't delete in destructor.
	bvh4.ReferenceFrom( original );
	// Convert a 4-wide BVH to a format suitable for GPU traversal. Layout:
	// offs 0:   aabbMin (12 bytes), 4x quantized child xmin (4 bytes)
	// offs 16:  aabbMax (12 bytes), 4x quantized child xmax (4 bytes)
	// offs 32:  4x child ymin, then ymax, zmax, zmax (total 16 bytes)
	// offs 48:  4x child node info: leaf if MSB set.
	//           Leaf: 15 bits for tri count, 16 for offset
	//           Interior: 32 bits for position of child node.
	// Triangle data ('by value') immediately follows each leaf node.
	Index blocksNeeded = compact ? (bvh4.usedNodes * 4) : (bvh4.allocatedNodes * 4); // here, 'block' is 16 bytes.
	blocksNeeded += 6 * bvh4.triCount; // this layout stores tris in the same buffer.
	if (allocatedBlocks < blocksNeeded)
	{
		AlignedFree( bvh4Data );
		bvh4Data = (bvhvec4*)AlignedAlloc( blocksNeeded * 16 );
		allocatedBlocks = blocksNeeded;
	}
	memset( bvh4Data, 0, 16 * blocksNeeded );
	CopyBasePropertiesFrom( bvh4 );
	// start conversion
	uint32_t nodeIdx = 0, newAlt4Ptr = 0, stack[TINYBVH_STACK_SIZE], stackPtr = 0, retValPos = 0;
	while (1)
	{
		const typename MBVH<4, Float, Index>::MBVHNode& orig = bvh4.mbvhNode[nodeIdx];
		// convert BVH4 node - must be an interior node.
		assert( !orig.isLeaf() );
		bvhvec4* nodeBase = bvh4Data + newAlt4Ptr;
		uint32_t baseAlt4Ptr = newAlt4Ptr;
		newAlt4Ptr += 4;
		nodeBase[0] = bvhvec4( orig.aabbMin, 0 );
		nodeBase[1] = bvhvec4( (orig.aabbMax - orig.aabbMin) * (1.0f / 255.0f), 0 );
		typename MBVH<4, Float, Index>::MBVHNode* childNode[4] = {
			&bvh4.mbvhNode[orig.child[0]], &bvh4.mbvhNode[orig.child[1]],
			&bvh4.mbvhNode[orig.child[2]], &bvh4.mbvhNode[orig.child[3]]
		};
		// start with leaf child node conversion
		uint32_t childInfo[4] = { 0, 0, 0, 0 }; // will store in final fields later
		for (int32_t i = 0; i < 4; i++) if (childNode[i]->isLeaf())
		{
			childInfo[i] = newAlt4Ptr - baseAlt4Ptr;
			childInfo[i] |= childNode[i]->triCount << 16;
			childInfo[i] |= 0x80000000;
			for (Index j = 0; j < childNode[i]->triCount; j++)
			{
				Index primIdx = bvh4.bvh.primIdx[childNode[i]->firstTri + j], i0, i1, i2;
				GET_PRIM_INDICES_I0_I1_I2( bvh4.bvh, primIdx );
			#ifdef BVH4_GPU_COMPRESSED_TRIS
				PrecomputeTriangle( bvh4.bvh.verts, i0, i1, i2, &bvh4Data[newAlt4Ptr] );
				bvh4Data[newAlt4Ptr + 3] = bvhvec4( 0, 0, 0, tinybvh_as_float( t ) );
				newAlt4Ptr += 4;
			#else
				bvhvec4 v0 = bvh4.bvh.verts[i0];
				bvh4Data[newAlt4Ptr + 1] = bvh4.bvh.verts[i1] - v0;
				bvh4Data[newAlt4Ptr + 2] = bvh4.bvh.verts[i2] - v0;
				v0.w = tinybvh_as_float( primIdx );
				bvh4Data[newAlt4Ptr + 0] = v0;
				newAlt4Ptr += 3;
			#endif
			}
		}
		// process interior nodes
		for (int32_t i = 0; i < 4; i++) if (!childNode[i]->isLeaf())
		{
			// childInfo[i] = node.child[i] == 0 ? 0 : GPUFormatBVH4( node.child[i] );
			if (orig.child[i] == 0) childInfo[i] = 0; else
			{
				stack[stackPtr++] = (uint32_t)(((const char*)&nodeBase[3] - (const char*)bvh4Data) / 4 + i);
				stack[stackPtr++] = orig.child[i];
			}
		}
		// store child node bounds, quantized
		const Vec3 extent = orig.aabbMax - orig.aabbMin;
		Vec3 scale;
		scale.x = extent.x > 1e-10f ? (254.999f / extent.x) : 0;
		scale.y = extent.y > 1e-10f ? (254.999f / extent.y) : 0;
		scale.z = extent.z > 1e-10f ? (254.999f / extent.z) : 0;
		uint8_t* slot0 = (uint8_t*)&nodeBase[0] + 12;	// 4 chars
		uint8_t* slot1 = (uint8_t*)&nodeBase[1] + 12;	// 4 chars
		uint8_t* slot2 = (uint8_t*)&nodeBase[2];		// 16 chars
		if (orig.child[0])
		{
			const Vec3 relBMin = childNode[0]->aabbMin - orig.aabbMin, relBMax = childNode[0]->aabbMax - orig.aabbMin;
			slot0[0] = (uint8_t)floorf( relBMin.x * scale.x ), slot1[0] = (uint8_t)ceilf( relBMax.x * scale.x );
			slot2[0] = (uint8_t)floorf( relBMin.y * scale.y ), slot2[4] = (uint8_t)ceilf( relBMax.y * scale.y );
			slot2[8] = (uint8_t)floorf( relBMin.z * scale.z ), slot2[12] = (uint8_t)ceilf( relBMax.z * scale.z );
		}
		if (orig.child[1])
		{
			const Vec3 relBMin = childNode[1]->aabbMin - orig.aabbMin, relBMax = childNode[1]->aabbMax - orig.aabbMin;
			slot0[1] = (uint8_t)floorf( relBMin.x * scale.x ), slot1[1] = (uint8_t)ceilf( relBMax.x * scale.x );
			slot2[1] = (uint8_t)floorf( relBMin.y * scale.y ), slot2[5] = (uint8_t)ceilf( relBMax.y * scale.y );
			slot2[9] = (uint8_t)floorf( relBMin.z * scale.z ), slot2[13] = (uint8_t)ceilf( relBMax.z * scale.z );
		}
		if (orig.child[2])
		{
			const Vec3 relBMin = childNode[2]->aabbMin - orig.aabbMin, relBMax = childNode[2]->aabbMax - orig.aabbMin;
			slot0[2] = (uint8_t)floorf( relBMin.x * scale.x ), slot1[2] = (uint8_t)ceilf( relBMax.x * scale.x );
			slot2[2] = (uint8_t)floorf( relBMin.y * scale.y ), slot2[6] = (uint8_t)ceilf( relBMax.y * scale.y );
			slot2[10] = (uint8_t)floorf( relBMin.z * scale.z ), slot2[14] = (uint8_t)ceilf( relBMax.z * scale.z );
		}
		if (orig.child[3])
		{
			const Vec3 relBMin = childNode[3]->aabbMin - orig.aabbMin, relBMax = childNode[3]->aabbMax - orig.aabbMin;
			slot0[3] = (uint8_t)floorf( relBMin.x * scale.x ), slot1[3] = (uint8_t)ceilf( relBMax.x * scale.x );
			slot2[3] = (uint8_t)floorf( relBMin.y * scale.y ), slot2[7] = (uint8_t)ceilf( relBMax.y * scale.y );
			slot2[11] = (uint8_t)floorf( relBMin.z * scale.z ), slot2[15] = (uint8_t)ceilf( relBMax.z * scale.z );
		}
		// finalize node
		nodeBase[3] = bvhvec4(
			tinybvh_as_float( childInfo[0] ), tinybvh_as_float( childInfo[1] ),
			tinybvh_as_float( childInfo[2] ), tinybvh_as_float( childInfo[3] )
		);
		// pop new work from the stack
		if (retValPos > 0) tinybvh_setlane_u( bvh4Data, retValPos, baseAlt4Ptr );
		if (stackPtr == 0) break;
		nodeIdx = stack[--stackPtr];
		retValPos = stack[--stackPtr];
	}
	usedBlocks = newAlt4Ptr;
}

// IntersectAlt4Nodes. For testing the converted data only; not efficient.
// This code replicates how traversal on GPU happens.
#define SWAP(A,B,C,D) tmp=A,A=B,B=tmp,tmp2=C,C=D,D=tmp2;
struct uchar4 { uint8_t x, y, z, w; };
static uchar4 as_uchar4( const float v ) { return tinybvh_bitcast<uchar4>( v ); }
static uint32_t as_uint( const float v ) { uint32_t t; memcpy( &t, &v, sizeof( t ) ); return t; }
TEMPLATED int32_t BVH4_GPU<Float, Index>::Intersect( Ray& ray ) const
{
	// traverse a blas
	VALIDATE_RAY( ray );
	uint32_t offset = 0, stack[TINYBVH_STACK_SIZE], stackPtr = 0, tmp2 /* for SWAP macro */;
	float cost = 0;
	while (1)
	{
		cost += c_trav;
		// fetch the node
		const bvhvec4 data0 = bvh4Data[offset + 0], data1 = bvh4Data[offset + 1];
		const bvhvec4 data2 = bvh4Data[offset + 2], data3 = bvh4Data[offset + 3];
		// extract aabb
		const Vec3 bmin = data0, extent = data1; // pre-scaled by 1/255
		// reconstruct conservative child aabbs
		const uchar4 d0 = as_uchar4( data0.w ), d1 = as_uchar4( data1.w ), d2 = as_uchar4( data2.x );
		const uchar4 d3 = as_uchar4( data2.y ), d4 = as_uchar4( data2.z ), d5 = as_uchar4( data2.w );
		const Vec3 c0min = bmin + extent * Vec3( d0.x, d2.x, d4.x ), c0max = bmin + extent * Vec3( d1.x, d3.x, d5.x );
		const Vec3 c1min = bmin + extent * Vec3( d0.y, d2.y, d4.y ), c1max = bmin + extent * Vec3( d1.y, d3.y, d5.y );
		const Vec3 c2min = bmin + extent * Vec3( d0.z, d2.z, d4.z ), c2max = bmin + extent * Vec3( d1.z, d3.z, d5.z );
		const Vec3 c3min = bmin + extent * Vec3( d0.w, d2.w, d4.w ), c3max = bmin + extent * Vec3( d1.w, d3.w, d5.w );
		// intersect child aabbs
		const Vec3 t1a = (c0min - ray.O) * ray.rD, t2a = (c0max - ray.O) * ray.rD;
		const Vec3 t1b = (c1min - ray.O) * ray.rD, t2b = (c1max - ray.O) * ray.rD;
		const Vec3 t1c = (c2min - ray.O) * ray.rD, t2c = (c2max - ray.O) * ray.rD;
		const Vec3 t1d = (c3min - ray.O) * ray.rD, t2d = (c3max - ray.O) * ray.rD;
		const Vec3 minta = tinybvh_min( t1a, t2a ), maxta = tinybvh_max( t1a, t2a );
		const Vec3 mintb = tinybvh_min( t1b, t2b ), maxtb = tinybvh_max( t1b, t2b );
		const Vec3 mintc = tinybvh_min( t1c, t2c ), maxtc = tinybvh_max( t1c, t2c );
		const Vec3 mintd = tinybvh_min( t1d, t2d ), maxtd = tinybvh_max( t1d, t2d );
		const float tmina = tinybvh_max( tinybvh_max( tinybvh_max( minta.x, minta.y ), minta.z ), 0.0f );
		const float tminb = tinybvh_max( tinybvh_max( tinybvh_max( mintb.x, mintb.y ), mintb.z ), 0.0f );
		const float tminc = tinybvh_max( tinybvh_max( tinybvh_max( mintc.x, mintc.y ), mintc.z ), 0.0f );
		const float tmind = tinybvh_max( tinybvh_max( tinybvh_max( mintd.x, mintd.y ), mintd.z ), 0.0f );
		const float tmaxa = tinybvh_min( tinybvh_min( tinybvh_min( maxta.x, maxta.y ), maxta.z ), ray.hit.t );
		const float tmaxb = tinybvh_min( tinybvh_min( tinybvh_min( maxtb.x, maxtb.y ), maxtb.z ), ray.hit.t );
		const float tmaxc = tinybvh_min( tinybvh_min( tinybvh_min( maxtc.x, maxtc.y ), maxtc.z ), ray.hit.t );
		const float tmaxd = tinybvh_min( tinybvh_min( tinybvh_min( maxtd.x, maxtd.y ), maxtd.z ), ray.hit.t );
		float dist0 = tmina > tmaxa ? bvh_far<Float> : tmina, dist1 = tminb > tmaxb ? bvh_far<Float> : tminb;
		float dist2 = tminc > tmaxc ? bvh_far<Float> : tminc, dist3 = tmind > tmaxd ? bvh_far<Float> : tmind, tmp;
		// get child node info fields
		uint32_t c0info = as_uint( data3.x ), c1info = as_uint( data3.y );
		uint32_t c2info = as_uint( data3.z ), c3info = as_uint( data3.w );
		if (dist0 < dist2) SWAP( dist0, dist2, c0info, c2info );
		if (dist1 < dist3) SWAP( dist1, dist3, c1info, c3info );
		if (dist0 < dist1) SWAP( dist0, dist1, c0info, c1info );
		if (dist2 < dist3) SWAP( dist2, dist3, c2info, c3info );
		if (dist1 < dist2) SWAP( dist1, dist2, c1info, c2info );
		// process results, starting with farthest child, so nearest ends on top of stack
		uint32_t nextNode = 0;
		uint32_t leaf[4] = { 0, 0, 0, 0 }, leaves = 0;
		if (dist0 < bvh_far<Float>)
		{
			if (c0info & 0x80000000) leaf[leaves++] = c0info; else if (c0info) stack[stackPtr++] = c0info;
		}
		if (dist1 < bvh_far<Float>)
		{
			if (c1info & 0x80000000) leaf[leaves++] = c1info; else if (c1info) stack[stackPtr++] = c1info;
		}
		if (dist2 < bvh_far<Float>)
		{
			if (c2info & 0x80000000) leaf[leaves++] = c2info; else if (c2info) stack[stackPtr++] = c2info;
		}
		if (dist3 < bvh_far<Float>)
		{
			if (c3info & 0x80000000) leaf[leaves++] = c3info; else if (c3info) stack[stackPtr++] = c3info;
		}
		// process encountered leaves, if any
		for (uint32_t i = 0; i < leaves; i++)
		{
			const uint32_t N = (leaf[i] >> 16) & 0x7fff;
			uint32_t triStart = offset + (leaf[i] & 0xffff);
			for (uint32_t j = 0; j < N; j++, triStart += 3)
			{
				cost += c_int;
				const Vec3 e2 = Vec3( bvh4Data[triStart + 2] );
				const Vec3 e1 = Vec3( bvh4Data[triStart + 1] );
				const Vec3 v0 = bvh4Data[triStart + 0];
				MOLLER_TRUMBORE_TEST( ray.hit.t, continue );
				ray.hit.t = t, ray.hit.u = u, ray.hit.v = v;
				ray.hit.prim = as_uint( bvh4Data[triStart + 0].w );
			}
		}
		// continue with nearest node or first node on the stack
		if (nextNode) offset = nextNode; else
		{
			if (!stackPtr) break;
			offset = stack[--stackPtr];
		}
	}
	return (int32_t)cost; // cast to not break interface.
}

// BVH4_AMD_HW implementation
// ----------------------------------------------------------------------------

TEMPLATED BVH4_AMD_HW<Float, Index>::BVH4_AMD_HW( BVH4_AMD_HW&& other ) noexcept
{
	*this = other;
	// The embedded bvh4 must let go too: owned or not, this object now refers to
	// the same buffers, and only one of the two may release them.
	other.bvh4.ReleaseOwnership();
	other.bvh4Data = 0;
	other.allocatedBlocks = other.usedBlocks = 0;
}

// move assignment: release what we hold, then take over 'other'.
TEMPLATED BVH4_AMD_HW<Float, Index>& BVH4_AMD_HW<Float, Index>::operator=( BVH4_AMD_HW&& other ) noexcept
{
	if (this != &other) this->~BVH4_AMD_HW(), new (this) BVH4_AMD_HW( tinybvh_move( other ) );
	return *this;
}

TEMPLATED BVH4_AMD_HW<Float, Index>::~BVH4_AMD_HW()
{
	if (!ownBVH4) bvh4.ReleaseOwnership(); // clear out pointers we don't own.
	AlignedFree( bvh4Data );
}

// forwarders
TEMPLATED void BVH4_AMD_HW<Float, Index>::Build( const bvhvec4* v, const Index p ) { Build( Slice( v, p * 3, sizeof( bvhvec4 ) ) ); }
TEMPLATED void BVH4_AMD_HW<Float, Index>::Build( const bvhvec4* v, const Index* i, const Index p ) { Build( Slice( v, p * 3, sizeof( bvhvec4 ) ), i, p ); }
TEMPLATED void BVH4_AMD_HW<Float, Index>::Build( const Slice& v ) { Build( v, 0, 0 ); }
TEMPLATED void BVH4_AMD_HW<Float, Index>::BuildHQ( const bvhvec4* v, const Index p ) { BuildHQ( Slice( v, p * 3, sizeof( bvhvec4 ) ) ); }
TEMPLATED void BVH4_AMD_HW<Float, Index>::BuildHQ( const bvhvec4* v, const Index* i, const Index p ) { BuildHQ( Slice( v, p * 3, sizeof( bvhvec4 ) ), i, p ); }
TEMPLATED void BVH4_AMD_HW<Float, Index>::BuildHQ( const Slice& v ) { BuildHQ( v, 0, 0 ); }
TEMPLATED void BVH4_AMD_HW<Float, Index>::BuildHQ( const Slice& v, const Index* i, Index p ) { ScopedFlag hq( settings.useSpatialSplits, true ); Build( v, i, p ); }

TEMPLATED void BVH4_AMD_HW<Float, Index>::Build( const Slice& vertices, const Index* indices, Index prims )
{
	// a tree received through ConvertFrom is not ours to rebuild: drop the reference, start afresh.
	if (!ownBVH4) bvh4.DropReference( context ), ownBVH4 = true;
	// propagate settings for this layout to the underlying layout
	this->PassSettingsTo( bvh4 ), this->PassSettingsTo( bvh4.bvh );
	// leaves of one triangle, or of two that share an edge (one triangle node holds such a pair). Spatial
	// splits cannot clip a pair: that build keeps one triangle per leaf.
	if (pairEdges && !settings.useSpatialSplits) bvh4.bvh.BuildPairs( vertices, indices, prims, pairGrowth ); else
	{
		bvh4.bvh.settings.keepSplitting = true; // one triangle per leaf; the build delivers it
		bvh4.bvh.Build( vertices, indices, prims );
		bvh4.bvh.SplitLeafs( 1 ); // a no-op after the build above; returns at once when leaves are this small
	}
	bvh4.leafPrimLimit = 0; // the collapse keeps the leaves as they are
	bvh4.ConvertFrom( bvh4.bvh ); // Convert to 4 Wide
	// convert to AMD format
	ConvertFrom( bvh4 );
}

TEMPLATED void BVH4_AMD_HW<Float, Index>::Optimize( const uint32_t iterations, uint32_t mu )
{
	// a tree received through ConvertFrom is optimized where it is, as in every other layout: the
	// source BVH is the basis for this layout, not a product in its own right.
	bvh4.Optimize( iterations, mu );
	ConvertFrom( bvh4 );
}

TEMPLATED bool BVH4_AMD_HW<Float, Index>::Save( const char* fileName )
{
	std::fstream s{ fileName, s.binary | s.out };
	if (!s) return false;
	const uint32_t header = this->CacheHeader();
	s.write( (char*)&header, sizeof( uint32_t ) );
	s.write( (char*)&triCount, sizeof( Index ) );
	s.write( (char*)this, sizeof( BVH4_AMD_HW ) );
	s.write( (char*)bvh4Data, (size_t)usedBlocks * 16 );
	// write errors may only surface when the stream is flushed
	s.close();
	return !s.fail();
}

TEMPLATED bool BVH4_AMD_HW<Float, Index>::Load( const char* fileName, const Index expectedTris )
{
	// open file and check contents
	std::fstream s{ fileName, s.binary | s.in };
	if (!s) return false;
	BVHContext tmp = context;
	uint32_t header;
	Index fileTriCount;
	s.read( (char*)&header, sizeof( uint32_t ) );
	if (header != this->CacheHeader()) return false;
	s.read( (char*)&fileTriCount, sizeof( Index ) );
	if (fileTriCount != expectedTris) return false;
	// all checks passed: release what *this holds, then overwrite it with the file's image.
	uint32_t* opmapKeep = this->opmap, opmapNKeep = this->opmapN; // opacity maps are caller-owned; keep what was set.
	*this = BVH4_AMD_HW( tmp );
	s.read( (char*)this, sizeof( BVH4_AMD_HW ) );
	context = tmp; // can't load context; function pointers will differ.
	this->opmap = opmapKeep, this->opmapN = opmapNKeep;
	bvh4Data = (bvhvec4*)AlignedAlloc( (size_t)usedBlocks * 16 );
	allocatedBlocks = usedBlocks;
	s.read( (char*)bvh4Data, (size_t)usedBlocks * 16 );
	bvh4.DropReference( context ); // holds raw file data: reset, don't free.
	return true;
}

static inline float tinybvh_half_to_float( const uint32_t h )
{
	const uint32_t s = (h & 0x8000u) << 16, e = (h >> 10) & 31, m = h & 1023;
	if (e == 31) return tinybvh_as_float( s | 0x7f800000u | (m << 13) );
	if (e == 0) return (s ? -5.9604644775390625e-8f : 5.9604644775390625e-8f) * (float)m; // m * 2^-24
	return tinybvh_as_float( s | ((e + 112) << 23) | (m << 13) );
}

// A half within 1 ulp of f, |f| <= 65504; then one step outwards if needed, so the bound stays conservative.
static inline uint32_t tinybvh_half_bound( const float f, const bool up )
{
	const uint32_t b = tinybvh_as_uint( f ), s = (b >> 16) & 0x8000u, a = b & 0x7fffffffu;
	uint32_t h = s;
	if (a >= 0x38800000u) h |= (((a >> 23) - 112) << 10) + ((a >> 13) & 1023) + ((a >> 12) & 1); // normal
	else h |= (uint32_t)(tinybvh_as_float( a ) * 16777216.0f + 0.5f); // denormal: |f| / 2^-24
	const float back = tinybvh_half_to_float( h );
	const bool neg = s != 0, zero = (h & 0x7fffu) == 0;
	if (up && back < f) h = neg ? (zero ? 0x0001u : h - 1) : h + 1;
	if (!up && back > f) h = neg ? h + 1 : (zero ? 0x8001u : h - 1);
	return h;
}

// Triangle node for a leaf: one triangle (type 0), or two that share an edge (type 1). For a pair, v1 and v2
// are the shared vertices; the I and J selectors rotate each triangle back to its original vertex order.
TEMPLATED uint32_t BVH4_AMD_HW<Float, Index>::EncodeLeaf( const uint32_t nodeIdx, uint32_t* dst, const uint32_t parentID ) const
{
	const typename MBVH<4, Float, Index>::MBVHNode& leaf = bvh4.mbvhNode[nodeIdx];
	const BVH& src = bvh4.bvh;
	BVH_FATAL_ERROR_IF( leaf.triCount > 2, "BVH4_AMD_HW::ConvertFrom expected one triangle or one pair per leaf." );
	uint32_t tri[2] = { (uint32_t)src.primIdx[leaf.firstTri], 0 };
	if (leaf.triCount == 2) tri[1] = (uint32_t)src.primIdx[leaf.firstTri + 1];
	Vec3 v[2][3];
	for (uint32_t t = 0; t < leaf.triCount; t++) for (uint32_t k = 0; k < 3; k++)
		v[t][k] = src.verts[src.vertIdx ? src.vertIdx[tri[t] * 3 + k] : tri[t] * 3 + k];
	auto same = []( const Vec3& p, const Vec3& q )
	{
		return tinybvh_as_uint( p.x ) == tinybvh_as_uint( q.x ) && tinybvh_as_uint( p.y ) == tinybvh_as_uint( q.y ) && tinybvh_as_uint( p.z ) == tinybvh_as_uint( q.z );
	};
	auto store = [&]( const uint32_t slot, const Vec3& p ) { memcpy( dst + slot * 3, &p.x, 4 ), memcpy( dst + slot * 3 + 1, &p.y, 4 ), memcpy( dst + slot * 3 + 2, &p.z, 4 ); };
	memset( dst, 0, 64 );
	dst[12] = tri[0], dst[14] = parentID;
	if (leaf.triCount == 1)
	{
		for (uint32_t k = 0; k < 3; k++) store( k, v[0][k] );
		dst[15] = 0x89; // I = 1, J = 2: the stored order; opaque
		return NODE_TRIANGLE;
	}
	for (uint32_t i0 = 0; i0 < 3; i0++) for (uint32_t j0 = 0; j0 < 3; j0++)
	{
		const Vec3& e1 = v[0][(i0 + 1) % 3], & e2 = v[0][(i0 + 2) % 3], & f1 = v[1][(j0 + 1) % 3], & f2 = v[1][(j0 + 2) % 3];
		const bool direct = same( f1, e1 ) && same( f2, e2 );
		if (!direct && !(same( f1, e2 ) && same( f2, e1 ))) continue;
		store( 0, v[0][i0] ), store( 1, e1 ), store( 2, e2 ), store( 3, v[1][j0] );
		// tri0 over (v0, v1, v2): vertex x of tri0 is in slot (x - i0) mod 3. The hardware order is (slot 3 - I - J, slot I, slot J).
		const uint32_t i0Sel = (4 - i0) % 3, j0Sel = (5 - i0) % 3;
		// tri1 over (v1, v3, v2): its vertex j0 is v3 (selector 1); the other two are v1 (0) and v2 (2).
		uint32_t sel[3];
		sel[j0] = 1, sel[(j0 + 1) % 3] = direct ? 0 : 2, sel[(j0 + 2) % 3] = direct ? 2 : 0;
		dst[13] = tri[1];
		dst[15] = (i0Sel | (j0Sel << 2) | 0x80) | ((sel[1] | (sel[2] << 2) | 0x80) << 8);
		return NODE_PAIR;
	}
	BVH_FATAL_ERROR_IF( true, "BVH4_AMD_HW::ConvertFrom, a leaf holds two triangles that share no edge." );
	return NODE_TRIANGLE;
}

TEMPLATED void BVH4_AMD_HW<Float, Index>::ConvertFrom( const MBVH<4, Float, Index>& original )
{
	// get a copy of the original bvh4
	if (&original != &bvh4)
	{
		if (ownBVH4) bvh4 = MBVH<4, Float, Index>( context ); else bvh4.DropReference( context );
		ownBVH4 = false; // bvh isn't ours; don't delete in destructor.
	}
	bvh4.ReferenceFrom( original );
	CopyBasePropertiesFrom( bvh4 );
	typedef typename MBVH<4, Float, Index>::MBVHNode Node;
	const Node* node = bvh4.mbvhNode;
	const uint32_t nodeCount = (uint32_t)bvh4.usedNodes, NONE = 0xffffffffu;
	BVH_FATAL_ERROR_IF( node[0].isLeaf(), "BVH4_AMD_HW::ConvertFrom expects an interior root." );
	// 1. Depth first: each node's kind and address. An interior node becomes an fp16 box node (64 B) when
	// half precision barely grows its child boxes, else an fp32 box node (128 B; the root always); a leaf
	// becomes a triangle node (64 B). Each kind has its own region, so nothing needs padding.
	uint32_t* addr = (uint32_t*)AlignedAlloc( (size_t)nodeCount * 4 ); // in 16-byte blocks
	uint32_t* parent = (uint32_t*)AlignedAlloc( (size_t)nodeCount * 4 );
	uint32_t* order = (uint32_t*)AlignedAlloc( (size_t)nodeCount * 4 ), orderCount = 0;
	uint32_t* stack = (uint32_t*)AlignedAlloc( (size_t)nodeCount * 4 ), stackPtr = 1;
	uint8_t* kind = (uint8_t*)AlignedAlloc( nodeCount );
	uint32_t count[6] = {};
	stack[0] = 0, parent[0] = NONE;
	while (stackPtr > 0)
	{
		const uint32_t idx = stack[--stackPtr];
		order[orderCount++] = idx;
		const Node& n = node[idx];
		if (n.isLeaf()) { kind[idx] = n.triCount == 2 ? NODE_PAIR : NODE_TRIANGLE, count[kind[idx]]++; continue; }
		bool half = idx != 0; // each child box may grow by fp16Growth at most: a small one hides in a sum
		for (int i = 0; i < 4; i++) if (n.child[i] != 0)
		{
			parent[n.child[i]] = idx, stack[stackPtr++] = (uint32_t)n.child[i];
			const Vec3 lo = node[n.child[i]].aabbMin, hi = node[n.child[i]].aabbMax;
			for (int a = 0; a < 3; a++) half = half && lo[a] >= -65504 && hi[a] <= 65504;
			if (!half) continue;
			Vec3 lo16, hi16;
			for (int a = 0; a < 3; a++)
				lo16[a] = tinybvh_half_to_float( tinybvh_half_bound( (float)lo[a], false ) ), hi16[a] = tinybvh_half_to_float( tinybvh_half_bound( (float)hi[a], true ) );
			half = Base::SA( lo16, hi16 ) <= fp16Growth * Base::SA( lo, hi );
		}
		kind[idx] = half ? NODE_BOX16 : NODE_BOX32, count[kind[idx]]++;
	}
	AlignedFree( stack );
	const uint32_t box16Base = count[NODE_BOX32] * 8, triBase = box16Base + count[NODE_BOX16] * 4;
	const uint32_t tableBase = triBase + (count[NODE_TRIANGLE] + count[NODE_PAIR]) * 4;
	const uint64_t blocks64 = (uint64_t)tableBase + (count[NODE_BOX16] + 3) / 4;
	// Node IDs hold 32-bit byte offsets, so the tree must stay below 4 GB (2^28 blocks).
	BVH_FATAL_ERROR_IF( blocks64 > (1ull << 28), "BVH4_AMD_HW::ConvertFrom, BVH exceeds the 4 GB addressable by node IDs." );
	uint32_t next[6] = { triBase, 0, 0, 0, box16Base, 0 };
	for (uint32_t i = 0; i < orderCount; i++)
	{
		const uint32_t k = kind[order[i]], region = k == NODE_PAIR ? NODE_TRIANGLE : k;
		addr[order[i]] = next[region], next[region] += k == NODE_BOX32 ? 8 : 4;
	}
	if (allocatedBlocks < (uint32_t)blocks64)
	{
		AlignedFree( bvh4Data );
		bvh4Data = (bvhvec4*)AlignedAlloc( (size_t)blocks64 * 16 );
		allocatedBlocks = (uint32_t)blocks64;
	}
	usedBlocks = (uint32_t)blocks64;
	memset( bvh4Data + tableBase, 0, ((size_t)usedBlocks - tableBase) * 16 );
	// 2. Write the nodes. An fp16 box node has no spare bytes for TinyBVH's parent ID, used by the kernels to
	// recover from a stack overflow: those go in a table after the triangle nodes, found through the root.
	uint32_t* table = (uint32_t*)(bvh4Data + tableBase);
	const uint32_t tasks = Base::TaskCount( this->PoolAvailable(), orderCount, 4096, 256 );
	this->ParallelFor( tasks > 1, tasks, [&]( const uint32_t s )
	{
		for (uint32_t i = (uint32_t)Base::SliceStart( orderCount, s, tasks ); i < (uint32_t)Base::SliceStart( orderCount, s + 1, tasks ); i++)
		{
			const uint32_t idx = order[i], k = kind[idx];
			uint32_t* w = (uint32_t*)(bvh4Data + addr[idx]);
			const uint32_t parentID = idx == 0 ? NONE : PackNodeID( addr[parent[idx]] * 16, kind[parent[idx]] );
			if (k == NODE_TRIANGLE || k == NODE_PAIR) { EncodeLeaf( idx, w, parentID ); continue; }
			const Node& n = node[idx];
			for (int c = 0; c < 4; c++)
			{
				const uint32_t child = (uint32_t)n.child[c];
				w[c] = child == 0 ? NONE : PackNodeID( addr[child] * 16, kind[child] );
				const Vec3 lo = child == 0 ? Vec3( 3.4028235e38f ) : Vec3( node[child].aabbMin ); // empty: inverted
				const Vec3 hi = child == 0 ? Vec3( -3.4028235e38f ) : Vec3( node[child].aabbMax );
				if (k == NODE_BOX32) for (int a = 0; a < 3; a++) memcpy( w + 4 + c * 6 + a, &lo[a], 4 ), memcpy( w + 7 + c * 6 + a, &hi[a], 4 );
				else
				{
					uint32_t h[6] = { 0x7bffu, 0x7bffu, 0x7bffu, 0xfbffu, 0xfbffu, 0xfbffu }; // empty: +65504, -65504
					if (child != 0) for (int a = 0; a < 3; a++) h[a] = tinybvh_half_bound( (float)lo[a], false ), h[a + 3] = tinybvh_half_bound( (float)hi[a], true );
					w[4 + c * 3] = h[0] | (h[1] << 16), w[5 + c * 3] = h[2] | (h[3] << 16), w[6 + c * 3] = h[4] | (h[5] << 16);
				}
			}
			if (k == NODE_BOX16) { table[(addr[idx] - box16Base) / 4] = parentID; continue; }
			w[28] = 0, w[29] = idx == 0 ? box16Base / 4 : 0, w[30] = idx == 0 ? tableBase * 4 : 0, w[31] = parentID;
		}
	} );
	AlignedFree( addr ), AlignedFree( parent ), AlignedFree( order ), AlignedFree( kind );
}

// Vertex order per GPURT: type k selects the slots (v0, v1, v2) or (v1, v3, v2); the I and J selectors
// in byte k of the hardware ID at byte 60 rotate them. The primitive ID of triangle k is at byte 48 + 4k.
TEMPLATED void BVH4_AMD_HW<Float, Index>::NodeTriangle( const void* node, const uint32_t k, Vec3& v0, Vec3& v1, Vec3& v2, uint32_t& prim )
{
	const float* f = (const float*)node;
	const uint32_t* w = (const uint32_t*)node, slots = k ? 0x231u : 0x210u;
	const uint32_t i = (w[15] >> (8 * k)) & 3, j = (w[15] >> (8 * k + 2)) & 3, order[3] = { 3 - i - j, i, j };
	Vec3* out[3] = { &v0, &v1, &v2 };
	for (int c = 0; c < 3; c++)
	{
		const uint32_t s = (slots >> (order[c] * 4)) & 15;
		*out[c] = Vec3( f[s * 3], f[s * 3 + 1], f[s * 3 + 2] );
	}
	prim = w[12 + k];
}

TEMPLATED int32_t BVH4_AMD_HW<Float, Index>::Intersect( Ray& ray ) const
{
	struct Entry { uint32_t id; float t; } stack[TINYBVH_STACK_SIZE * 4], hit[4];
	const uint8_t* base = (const uint8_t*)bvh4Data;
	uint32_t stackPtr = 0, id = PackNodeID( 0, NODE_BOX32 ), steps = 0;
	while (1)
	{
		const uint32_t type = id & 7;
		const uint8_t* node = base + ((size_t)(id & ~7u) << 3);
		steps++;
		if (type == NODE_BOX32 || type == NODE_BOX16)
		{
			const uint32_t* w = (const uint32_t*)node;
			uint32_t hits = 0;
			for (uint32_t c = 0; c < 4; c++)
			{
				if (w[c] == 0xffffffffu) continue;
				Vec3 bmin, bmax;
				if (type == NODE_BOX32)
				{
					const float* b = (const float*)(node + 16 + c * 24);
					bmin = Vec3( b[0], b[1], b[2] ), bmax = Vec3( b[3], b[4], b[5] );
				}
				else
				{
					const uint32_t* h = w + 4 + c * 3; // x, y, z min, then x, y, z max, as halfs
					bmin = Vec3( tinybvh_half_to_float( h[0] & 0xffff ), tinybvh_half_to_float( h[0] >> 16 ), tinybvh_half_to_float( h[1] & 0xffff ) );
					bmax = Vec3( tinybvh_half_to_float( h[1] >> 16 ), tinybvh_half_to_float( h[2] & 0xffff ), tinybvh_half_to_float( h[2] >> 16 ) );
				}
				const float t = tinybvh_intersect_aabb( ray, bmin, bmax );
				if (t >= BVH_FAR) continue;
				uint32_t j = hits++;
				for (; j > 0 && hit[j - 1].t > t; j--) hit[j] = hit[j - 1];
				hit[j] = { w[c], t };
			}
			BVH_FATAL_ERROR_IF( stackPtr + hits > TINYBVH_STACK_SIZE * 4, "BVH4_AMD_HW::Intersect, stack overflow." );
			while (hits > 0) stack[stackPtr++] = hit[--hits]; // nearest on top
		}
		else for (uint32_t k = 0; k <= type; k++) // a pair (type 1) is tested as type 1, then as type 0
		{
			Vec3 v0, v1, v2;
			uint32_t prim;
			NodeTriangle( node, k, v0, v1, v2, prim );
			this->IntersectTri( ray, (Index)prim, v0, v1, v2 );
		}
		while (stackPtr > 0 && stack[stackPtr - 1].t >= ray.hit.t) stackPtr--;
		if (stackPtr == 0) break;
		id = stack[--stackPtr].id;
	}
	return (int32_t)steps;
}

// BVH6_INTEL_HW implementation
// ----------------------------------------------------------------------------

TEMPLATED BVH6_INTEL_HW<Float, Index>::BVH6_INTEL_HW( BVH6_INTEL_HW&& other ) noexcept
{
	*this = other;
	other.bvh6.ReleaseOwnership();
	other.rtasData = 0;
	other.allocatedBlocks = other.usedBlocks = 0;
}

TEMPLATED BVH6_INTEL_HW<Float, Index>& BVH6_INTEL_HW<Float, Index>::operator=( BVH6_INTEL_HW&& other ) noexcept
{
	if (this != &other) this->~BVH6_INTEL_HW(), new (this) BVH6_INTEL_HW( tinybvh_move( other ) );
	return *this;
}

TEMPLATED BVH6_INTEL_HW<Float, Index>::~BVH6_INTEL_HW()
{
	if (!ownBVH6) bvh6.ReleaseOwnership(); // clear out pointers we don't own.
	AlignedFree( rtasData );
}

// forwarders
TEMPLATED void BVH6_INTEL_HW<Float, Index>::Build( const bvhvec4* v, const Index p ) { Build( Slice( v, p * 3, sizeof( bvhvec4 ) ) ); }
TEMPLATED void BVH6_INTEL_HW<Float, Index>::Build( const bvhvec4* v, const Index* i, const Index p ) { Build( Slice( v, p * 3, sizeof( bvhvec4 ) ), i, p ); }
TEMPLATED void BVH6_INTEL_HW<Float, Index>::Build( const Slice& v ) { Build( v, 0, 0 ); }
TEMPLATED void BVH6_INTEL_HW<Float, Index>::BuildHQ( const bvhvec4* v, const Index p ) { BuildHQ( Slice( v, p * 3, sizeof( bvhvec4 ) ) ); }
TEMPLATED void BVH6_INTEL_HW<Float, Index>::BuildHQ( const bvhvec4* v, const Index* i, const Index p ) { BuildHQ( Slice( v, p * 3, sizeof( bvhvec4 ) ), i, p ); }
TEMPLATED void BVH6_INTEL_HW<Float, Index>::BuildHQ( const Slice& v ) { BuildHQ( v, 0, 0 ); }
TEMPLATED void BVH6_INTEL_HW<Float, Index>::BuildHQ( const Slice& v, const Index* i, Index p ) { ScopedFlag hq( settings.useSpatialSplits, true ); Build( v, i, p ); }

TEMPLATED void BVH6_INTEL_HW<Float, Index>::Build( const Slice& vertices, const Index* indices, Index prims )
{
	if (!ownBVH6) bvh6.DropReference( context ), ownBVH6 = true;
	this->PassSettingsTo( bvh6 ), this->PassSettingsTo( bvh6.bvh );
	bvh6.bvh.Build( vertices, indices, prims );
	bvh6.bvh.SplitLeafs( LEAF_TRIS );
	bvh6.leafPrimLimit = LEAF_TRIS; // the collapse may merge small subtrees into one fat leaf,
	bvh6.leafCostScale = (Float)0.1; // which is cheap: its children are culled per quad group
	bvh6.ConvertFrom( bvh6.bvh );
	ConvertFrom( bvh6 );
}

TEMPLATED void BVH6_INTEL_HW<Float, Index>::Optimize( const uint32_t iterations, uint32_t mu )
{
	bvh6.Optimize( iterations, mu );
	ConvertFrom( bvh6 );
}

TEMPLATED bool BVH6_INTEL_HW<Float, Index>::Save( const char* fileName )
{
	std::fstream s{ fileName, s.binary | s.out };
	if (!s) return false;
	const uint32_t header = this->CacheHeader();
	s.write( (char*)&header, sizeof( uint32_t ) );
	s.write( (char*)&triCount, sizeof( Index ) );
	s.write( (char*)this, sizeof( BVH6_INTEL_HW ) );
	s.write( (char*)rtasData, (size_t)usedBlocks * 64 );
	s.close();
	return !s.fail();
}

TEMPLATED bool BVH6_INTEL_HW<Float, Index>::Load( const char* fileName, const Index expectedTris )
{
	std::fstream s{ fileName, s.binary | s.in };
	if (!s) return false;
	BVHContext tmp = context;
	uint32_t header;
	Index fileTriCount;
	s.read( (char*)&header, sizeof( uint32_t ) );
	if (header != this->CacheHeader()) return false;
	s.read( (char*)&fileTriCount, sizeof( Index ) );
	if (fileTriCount != expectedTris) return false;
	uint32_t* opmapKeep = this->opmap, opmapNKeep = this->opmapN;
	*this = BVH6_INTEL_HW( tmp );
	s.read( (char*)this, sizeof( BVH6_INTEL_HW ) );
	context = tmp;
	this->opmap = opmapKeep, this->opmapN = opmapNKeep;
	rtasData = (bvhvec4*)AlignedAlloc( (size_t)usedBlocks * 64 );
	allocatedBlocks = usedBlocks;
	s.read( (char*)rtasData, (size_t)usedBlocks * 64 );
	bvh6.DropReference( context ); // holds raw file data: reset, don't free.
	return true;
}

// Quantization as in Intel's InternalNode6: grow the box by an ulp of its largest coordinate,
// then fit a grid of 256 steps of 2^(exp-8) per axis.
static inline void tinybvh_intel_conservative( bvhvec3& bmin, bvhvec3& bmax )
{
	const float a = tinybvh_max( tinybvh_max( fabsf( bmin.x ), fabsf( bmin.y ) ), fabsf( bmin.z ) );
	const float b = tinybvh_max( tinybvh_max( fabsf( bmax.x ), fabsf( bmax.y ) ), fabsf( bmax.z ) );
	const float err = 1.1920929e-7f * tinybvh_max( a, b );
	bmin = bmin - bvhvec3( err ), bmax = bmax + bvhvec3( err );
}

TEMPLATED void BVH6_INTEL_HW<Float, Index>::InitNode( IntelNode6& node, bvhvec3 bmin, bvhvec3 bmax, const uint8_t type )
{
	memset( &node, 0, sizeof( IntelNode6 ) );
	node.nodeType = type, node.nodeMask = 0xff;
	for (int i = 0; i < 6; i++) node.q[0][i] = node.q[2][i] = node.q[4][i] = 0x80; // all children empty
	tinybvh_intel_conservative( bmin, bmax );
	const bvhvec3 len = (bmax - bmin) * (1.0f + 1.1920929e-7f);
	for (int a = 0; a < 3; a++)
	{
		int e;
		const float m = frexpf( len[a], &e );
		e += m > 255.0f / 256.0f;
		node.exp[a] = (int8_t)tinybvh_clamp( e, -128, 127 );
		node.lower[a] = bmin[a];
	}
}

TEMPLATED void BVH6_INTEL_HW<Float, Index>::SetChildBounds( IntelNode6& node, const uint32_t i, bvhvec3 bmin, bvhvec3 bmax )
{
	tinybvh_intel_conservative( bmin, bmax );
	for (int a = 0; a < 3; a++)
	{
		const float lo = ldexpf( bmin[a] - node.lower[a], 8 - node.exp[a] );
		const float hi = ldexpf( bmax[a] - node.lower[a], 8 - node.exp[a] );
		node.q[a * 2][i] = (uint8_t)tinybvh_clamp( floorf( lo ), 0.0f, 255.0f );
		node.q[a * 2 + 1][i] = (uint8_t)tinybvh_clamp( ceilf( hi ), 0.0f, 255.0f );
	}
}

TEMPLATED void BVH6_INTEL_HW<Float, Index>::ChildBounds( const IntelNode6& node, const uint32_t i, bvhvec3& bmin, bvhvec3& bmax )
{
	for (int a = 0; a < 3; a++)
		bmin[a] = node.lower[a] + ldexpf( (float)node.q[a * 2][i], node.exp[a] - 8 ),
		bmax[a] = node.lower[a] + ldexpf( (float)node.q[a * 2 + 1][i], node.exp[a] - 8 );
}

// Writes a fat-leaf node at dstBlock: triangles are paired into quads, which are spread over
// up to 6 children of at most 3 contiguous quads each. Returns the next free block.
TEMPLATED uint32_t BVH6_INTEL_HW<Float, Index>::EmitFatLeaf( const uint32_t srcNode, const uint32_t dstBlock, uint32_t blockPtr )
{
	const typename MBVH<6, Float, Index>::MBVHNode& leaf = bvh6.mbvhNode[srcNode];
	const BVH& src = bvh6.bvh;
	const uint32_t n = (uint32_t)leaf.triCount;
	BVH_FATAL_ERROR_IF( n > 18, "BVH6_INTEL_HW::ConvertFrom, leaf exceeds 18 triangles." );
	uint32_t prim[18], vi[18][3];
	for (uint32_t i = 0; i < n; i++)
	{
		uint32_t p = (uint32_t)src.primIdx[leaf.firstTri + i], j = i;
		for (; j > 0 && prim[j - 1] > p; j--) prim[j] = prim[j - 1];
		prim[j] = p; // ascending, so the second triangle of a pair has the larger index
	}
	for (uint32_t i = 0; i < n; i++) for (uint32_t k = 0; k < 3; k++)
		vi[i][k] = src.vertIdx ? (uint32_t)src.vertIdx[prim[i] * 3 + k] : prim[i] * 3 + k;
	auto same = [&]( const uint32_t x, const uint32_t y )
	{
		if (src.vertIdx) return x == y;
		const bvhvec4 p = src.verts[x], q = src.verts[y];
		return tinybvh_as_uint( p.x ) == tinybvh_as_uint( q.x ) && tinybvh_as_uint( p.y ) == tinybvh_as_uint( q.y ) &&
			tinybvh_as_uint( p.z ) == tinybvh_as_uint( q.z );
	};
	// pair triangles sharing an edge; the hardware stores the second index as a 5-bit delta.
	struct Quad { uint32_t a, b, fresh; uint8_t j[3]; } quad[18];
	bool used[18] = {};
	uint32_t Q = 0;
	for (uint32_t i = 0; i < n; i++) if (!used[i])
	{
		Quad& qd = quad[Q++];
		qd.a = qd.b = i, qd.fresh = 0, qd.j[0] = qd.j[1] = qd.j[2] = 0, used[i] = true;
		for (uint32_t k = i + 1; k < n; k++)
		{
			if (used[k] || prim[k] == prim[i] || prim[k] - prim[i] > 31) continue;
			uint8_t j[3];
			uint32_t freshCount = 0, fresh = 0;
			for (uint32_t c = 0; c < 3; c++)
			{
				j[c] = 3;
				for (uint8_t d = 0; d < 3; d++) if (same( vi[k][c], vi[i][d] )) { j[c] = d; break; }
				if (j[c] == 3) freshCount++, fresh = c;
			}
			if (freshCount != 1) continue;
			qd.b = k, qd.fresh = fresh, qd.j[0] = j[0], qd.j[1] = j[1], qd.j[2] = j[2], used[k] = true;
			break;
		}
	}
	// quad bounds, clipped to the leaf: with spatial splits the leaf box covers part of a triangle.
	bvhvec3 qmin[18], qmax[18], qc[18];
	const bvhvec3 lmin = leaf.aabbMin, lmax = leaf.aabbMax;
	for (uint32_t i = 0; i < Q; i++)
	{
		bvhvec3 bmin( BVH_FAR ), bmax( -BVH_FAR );
		for (uint32_t k = 0; k < 3; k++) bmin = tinybvh_min( bmin, bvhvec3( src.verts[vi[quad[i].a][k]] ) ), bmax = tinybvh_max( bmax, bvhvec3( src.verts[vi[quad[i].a][k]] ) );
		if (quad[i].b != quad[i].a)
		{
			const bvhvec3 v3 = src.verts[vi[quad[i].b][quad[i].fresh]];
			bmin = tinybvh_min( bmin, v3 ), bmax = tinybvh_max( bmax, v3 );
		}
		const bvhvec3 cmin = tinybvh_max( bmin, lmin ), cmax = tinybvh_min( bmax, lmax );
		const bool empty = cmin.x > cmax.x || cmin.y > cmax.y || cmin.z > cmax.z;
		qmin[i] = empty ? lmin : cmin, qmax[i] = empty ? lmax : cmax, qc[i] = (qmin[i] + qmax[i]) * 0.5f;
	}
	// more quads than children: group them along the longest axis.
	uint32_t order[18];
	for (uint32_t i = 0; i < Q; i++) order[i] = i;
	if (Q > 6)
	{
		const bvhvec3 e = lmax - lmin;
		const int axis = e.x > e.y && e.x > e.z ? 0 : (e.y > e.z ? 1 : 2);
		for (uint32_t i = 1; i < Q; i++)
		{
			const uint32_t o = order[i];
			uint32_t j = i;
			for (; j > 0 && qc[order[j - 1]][axis] > qc[o][axis]; j--) order[j] = order[j - 1];
			order[j] = o;
		}
	}
	const uint32_t groups = tinybvh_min( Q, 6u );
	uint32_t gFirst[6], gCount[6], gOrder[6];
	bvhvec3 gmin[6], gmax[6], nmin( BVH_FAR ), nmax( -BVH_FAR );
	float gArea[6];
	for (uint32_t g = 0, first = 0; g < groups; g++)
	{
		gFirst[g] = first, gCount[g] = Q / groups + (g < Q % groups ? 1 : 0), first += gCount[g];
		gmin[g] = bvhvec3( BVH_FAR ), gmax[g] = bvhvec3( -BVH_FAR );
		for (uint32_t k = 0; k < gCount[g]; k++)
			gmin[g] = tinybvh_min( gmin[g], qmin[order[gFirst[g] + k]] ), gmax[g] = tinybvh_max( gmax[g], qmax[order[gFirst[g] + k]] );
		nmin = tinybvh_min( nmin, gmin[g] ), nmax = tinybvh_max( nmax, gmax[g] );
		gArea[g] = tinybvh_halfarea( gmax[g] - gmin[g] );
		uint32_t j = g; // largest child first, as in Intel's builder
		for (; j > 0 && gArea[gOrder[j - 1]] < gArea[g]; j--) gOrder[j] = gOrder[j - 1];
		gOrder[j] = g;
	}
	IntelNode6& node = *(IntelNode6*)(rtasData + (size_t)dstBlock * 4);
	InitNode( node, nmin, nmax, NODE_QUAD );
	node.childOffset = (int32_t)(blockPtr - dstBlock);
	for (uint32_t c = 0; c < groups; c++)
	{
		const uint32_t g = gOrder[c];
		node.childData[c] = (uint8_t)(c == groups - 1 ? 0 : gCount[g]);
		SetChildBounds( node, c, gmin[g], gmax[g] );
		for (uint32_t k = 0; k < gCount[g]; k++)
		{
			const Quad& qd = quad[order[gFirst[g] + k]];
			IntelQuadLeaf& ql = *(IntelQuadLeaf*)(rtasData + (size_t)blockPtr++ * 4);
			const bool paired = qd.b != qd.a, last = k == gCount[g] - 1;
			ql.shaderGeomMask = 0xffu << 24;
			ql.geomIndexFlags = 1u << 30; // opaque
			ql.primIndex0 = prim[qd.a];
			// an unpaired triangle uses delta 1: delta 0 selects the hardware's quad UV mode.
			ql.pairInfo = (paired ? prim[qd.b] - prim[qd.a] : 1u) | ((uint32_t)qd.j[0] << 16) |
				((uint32_t)qd.j[1] << 18) | ((uint32_t)qd.j[2] << 20) | ((uint32_t)last << 22);
			for (uint32_t v = 0; v < 4; v++)
			{
				const bvhvec4 p = src.verts[v < 3 ? vi[qd.a][v] : (paired ? vi[qd.b][qd.fresh] : vi[qd.a][2])];
				ql.v[v][0] = p.x, ql.v[v][1] = p.y, ql.v[v][2] = p.z;
			}
		}
	}
	return blockPtr;
}

TEMPLATED void BVH6_INTEL_HW<Float, Index>::ConvertFrom( const MBVH<6, Float, Index>& original )
{
	if (&original != &bvh6)
	{
		if (ownBVH6) bvh6 = MBVH<6, Float, Index>( context ); else bvh6.DropReference( context );
		ownBVH6 = false; // bvh isn't ours; don't delete in destructor.
	}
	bvh6.ReferenceFrom( original );
	CopyBasePropertiesFrom( bvh6 );
	// one block per node, one per quad at most, plus the 128-byte header.
	uint64_t blocks64 = 3 + (uint64_t)bvh6.usedNodes;
	for (Index i = 0; i < bvh6.usedNodes; i++) if (bvh6.mbvhNode[i].isLeaf()) blocks64 += bvh6.mbvhNode[i].triCount;
	BVH_FATAL_ERROR_IF( blocks64 > (1ull << 31), "BVH6_INTEL_HW::ConvertFrom, BVH exceeds the range of child offsets." );
	const uint32_t blocksNeeded = (uint32_t)blocks64;
	if (allocatedBlocks < blocksNeeded)
	{
		AlignedFree( rtasData );
		rtasData = (bvhvec4*)AlignedAlloc( (size_t)blocksNeeded * 64 );
		allocatedBlocks = blocksNeeded;
	}
	memset( rtasData, 0, (size_t)blocksNeeded * 64 );
	IntelHeader& header = *(IntelHeader*)rtasData;
	header.rtasFormat = 1; // ZE_RTAS_FORMAT_DG2
	header.nodeDataStart = header.nodeDataCur = header.leafDataStart = header.leafDataCur = 2;
	header.proceduralDataStart = header.proceduralDataCur = header.backPointerDataStart = header.backPointerDataEnd = 2;
	header.numTimeSegments = 1;
	if (bvh6.usedNodes == 0 || bvh6.triCount == 0)
	{
		InitNode( *(IntelNode6*)(rtasData + 8), bvhvec3( 0 ), bvhvec3( 0 ), NODE_INTERNAL );
		usedBlocks = 3;
		return;
	}
	const bvhvec3 rmin = bvh6.mbvhNode[0].aabbMin, rmax = bvh6.mbvhNode[0].aabbMax;
	for (int a = 0; a < 3; a++) header.bounds[a] = rmin[a], header.bounds[a + 3] = rmax[a];
	// depth-first; the children of a node are written as one contiguous run.
	uint32_t stack[1024], stackPtr = 0, blockPtr = 3;
	stack[stackPtr++] = 0, stack[stackPtr++] = 2;
	while (stackPtr > 0)
	{
		const uint32_t dstBlock = stack[--stackPtr], srcNode = stack[--stackPtr];
		const typename MBVH<6, Float, Index>::MBVHNode& orig = bvh6.mbvhNode[srcNode];
		if (orig.isLeaf()) { blockPtr = EmitFatLeaf( srcNode, dstBlock, blockPtr ); continue; }
		uint32_t child[6], k = 0;
		bvhvec3 bmin( BVH_FAR ), bmax( -BVH_FAR );
		for (uint32_t i = 0; i < 6; i++) if (orig.child[i] != 0)
		{
			const typename MBVH<6, Float, Index>::MBVHNode& c = bvh6.mbvhNode[orig.child[i]];
			child[k++] = (uint32_t)orig.child[i];
			bmin = tinybvh_min( bmin, bvhvec3( c.aabbMin ) ), bmax = tinybvh_max( bmax, bvhvec3( c.aabbMax ) );
		}
		IntelNode6& node = *(IntelNode6*)(rtasData + (size_t)dstBlock * 4);
		InitNode( node, bmin, bmax, NODE_INTERNAL );
		node.childOffset = (int32_t)(blockPtr - dstBlock);
		BVH_FATAL_ERROR_IF( stackPtr + 2 * k > 1024, "BVH6_INTEL_HW::ConvertFrom, conversion stack overflow." );
		for (uint32_t j = 0; j < k; j++)
		{
			const typename MBVH<6, Float, Index>::MBVHNode& c = bvh6.mbvhNode[child[j]];
			node.childData[j] = (uint8_t)(j == k - 1 ? 0 : 1); // child type 0: internal
			SetChildBounds( node, j, c.aabbMin, c.aabbMax );
			stack[stackPtr++] = child[j], stack[stackPtr++] = blockPtr + j;
		}
		blockPtr += k;
	}
	usedBlocks = blockPtr;
}

TEMPLATED int32_t BVH6_INTEL_HW<Float, Index>::Intersect( Ray& ray ) const
{
	struct Entry { uint32_t block; float t; } stack[256], hit[6];
	uint32_t stackPtr = 0, block = 2, steps = 0;
	while (1)
	{
		const IntelNode6& node = *(const IntelNode6*)(rtasData + (size_t)block * 4);
		uint32_t childBlock = (uint32_t)((int64_t)block + node.childOffset), hits = 0;
		steps++;
		for (uint32_t i = 0; i < 6; childBlock += node.childData[i++] & 3)
		{
			if (!ChildValid( node, i )) continue;
			bvhvec3 bmin, bmax;
			ChildBounds( node, i, bmin, bmax );
			const float t = tinybvh_intersect_aabb( ray, bmin, bmax );
			if (t >= BVH_FAR) continue;
			const uint32_t type = node.nodeType == NODE_INTERNAL ? (node.childData[i] >> 2) & 15 : node.nodeType;
			if (type != NODE_QUAD)
			{
				uint32_t j = hits++;
				for (; j > 0 && hit[j - 1].t > t; j--) hit[j] = hit[j - 1];
				hit[j] = { childBlock, t };
				continue;
			}
			for (const IntelQuadLeaf* ql = (const IntelQuadLeaf*)(rtasData + (size_t)childBlock * 4);; ql++)
			{
				const Vec3 v[4] = { Vec3( ql->v[0][0], ql->v[0][1], ql->v[0][2] ), Vec3( ql->v[1][0], ql->v[1][1], ql->v[1][2] ),
					Vec3( ql->v[2][0], ql->v[2][1], ql->v[2][2] ), Vec3( ql->v[3][0], ql->v[3][1], ql->v[3][2] ) };
				const uint32_t info = ql->pairInfo, j0 = (info >> 16) & 3, j1 = (info >> 18) & 3, j2 = (info >> 20) & 3;
				this->IntersectTri( ray, (Index)ql->primIndex0, v[0], v[1], v[2] );
				if (j0 | j1 | j2) this->IntersectTri( ray, (Index)(ql->primIndex0 + (info & 31)), v[j0], v[j1], v[j2] );
				if (info & (1u << 22)) break;
			}
		}
		BVH_FATAL_ERROR_IF( stackPtr + hits > 256, "BVH6_INTEL_HW::Intersect, stack overflow." );
		while (hits > 0) stack[stackPtr++] = hit[--hits]; // nearest on top
		while (stackPtr > 0 && stack[stackPtr - 1].t >= ray.hit.t) stackPtr--;
		if (stackPtr == 0) break;
		block = stack[--stackPtr].block;
	}
	return (int32_t)steps;
}

// CWBVH8_AMD_HW implementation
// ----------------------------------------------------------------------------

// Two single-triangle leaves under one node become one leaf of two triangles, so they share a pair slot.
TEMPLATED static void tinybvh_pair_siblings( BVH<Float, Index>& bvh )
{
	typename BVH<Float, Index>::BVHNode* node = bvh.bvhNode;
	Index stack[TINYBVH_STACK_SIZE * 2], stackPtr = 0, nodeIdx = 0;
	bool merged = false;
	while (1)
	{
		typename BVH<Float, Index>::BVHNode& n = node[nodeIdx];
		if (!n.isLeaf())
		{
			const typename BVH<Float, Index>::BVHNode& l = node[n.leftFirst], & r = node[n.leftFirst + 1];
			if (l.triCount == 1 && r.triCount == 1 && (r.leftFirst == l.leftFirst + 1 || l.leftFirst == r.leftFirst + 1))
				n.leftFirst = tinybvh_min( l.leftFirst, r.leftFirst ), n.triCount = 2, merged = true;
			else
			{
				BVH_FATAL_ERROR_IF( stackPtr >= TINYBVH_STACK_SIZE * 2, "CWBVH8_AMD_HW::Build, stack overflow." );
				stack[stackPtr++] = n.leftFirst + 1, nodeIdx = n.leftFirst;
				continue;
			}
		}
		if (stackPtr == 0) break;
		nodeIdx = stack[--stackPtr];
	}
	if (merged) bvh.Compact(); // drop the orphaned leaves
}

TEMPLATED CWBVH8_AMD_HW<Float, Index>::CWBVH8_AMD_HW( CWBVH8_AMD_HW&& other ) noexcept
{
	*this = other;
	// The embedded bvh8 must let go too: owned or not, this object now refers to
	// the same buffers, and only one of the two may release them.
	other.bvh8.ReleaseOwnership();
	other.bvh8Data = 0;
	other.allocatedBlocks = other.usedBlocks = 0;
}

// move assignment: release what we hold, then take over 'other'.
TEMPLATED CWBVH8_AMD_HW<Float, Index>& CWBVH8_AMD_HW<Float, Index>::operator=( CWBVH8_AMD_HW&& other ) noexcept
{
	if (this != &other) this->~CWBVH8_AMD_HW(), new (this) CWBVH8_AMD_HW( tinybvh_move( other ) );
	return *this;
}

TEMPLATED CWBVH8_AMD_HW<Float, Index>::~CWBVH8_AMD_HW()
{
	if (!ownBVH8) bvh8.ReleaseOwnership(); // clear out pointers we don't own.
	AlignedFree( bvh8Data );
}

// forwarders
TEMPLATED void CWBVH8_AMD_HW<Float, Index>::Build( const bvhvec4* v, const Index p ) { Build( Slice( v, p * 3, sizeof( bvhvec4 ) ) ); }
TEMPLATED void CWBVH8_AMD_HW<Float, Index>::Build( const bvhvec4* v, const uint32_t* i, const Index p ) { Build( Slice( v, p * 3, sizeof( bvhvec4 ) ), i, p ); }
TEMPLATED void CWBVH8_AMD_HW<Float, Index>::Build( const Slice& v ) { Build( v, 0, 0 ); }
TEMPLATED void CWBVH8_AMD_HW<Float, Index>::BuildHQ( const bvhvec4* v, const Index p ) { BuildHQ( Slice( v, p * 3, sizeof( bvhvec4 ) ) ); }
TEMPLATED void CWBVH8_AMD_HW<Float, Index>::BuildHQ( const bvhvec4* v, const uint32_t* i, const Index p ) { BuildHQ( Slice( v, p * 3, sizeof( bvhvec4 ) ), i, p ); }
TEMPLATED void CWBVH8_AMD_HW<Float, Index>::BuildHQ( const Slice& v ) { BuildHQ( v, 0, 0 ); }
TEMPLATED void CWBVH8_AMD_HW<Float, Index>::BuildHQ( const Slice& v, const uint32_t* i, Index p ) { ScopedFlag hq( settings.useSpatialSplits, true ); Build( v, i, p ); }

TEMPLATED void CWBVH8_AMD_HW<Float, Index>::Build( const Slice& vertices, const uint32_t* indices, Index prims )
{
	if (!ownBVH8)
	{
		bvh8.DropReference( context );
		ownBVH8 = true;
	}
	// propagate settings for this layout to the underlying layout
	bvh8.context = bvh8.bvh.context = context, bvh8.settings = bvh8.bvh.settings = settings;
	bvh8.c_int = bvh8.bvh.c_int = c_int, bvh8.c_trav = bvh8.bvh.c_trav = c_trav;
	// leaves of one triangle pair: edge-sharing pairs found before the build, or (spatial splits, which
	// cannot clip a pair) leaves of one triangle with exact boxes. Sibling triangles become pairs as well.
	if (pairEdges && !settings.useSpatialSplits) bvh8.bvh.BuildPairs( vertices, indices, prims, pairGrowth );
	else
	{
		ScopedFlag keep( bvh8.bvh.settings.keepSplitting, true );
		bvh8.bvh.Build( vertices, indices, prims );
	}
	tinybvh_pair_siblings( bvh8.bvh );
	bvh8.leafPrimLimit = 2; // the collapse may merge single triangles into a pair leaf
	bvh8.ConvertFrom( bvh8.bvh );
	// convert to AMD format
	ConvertFrom( bvh8 );
}

TEMPLATED void CWBVH8_AMD_HW<Float, Index>::Optimize( const uint32_t iterations, uint32_t mu )
{
	// a tree received through ConvertFrom is optimized where it is, as in every other layout: the
	// source BVH is the basis for this layout, not a product in its own right.
	bvh8.Optimize( iterations, mu );
	ConvertFrom( bvh8 );
}

// Sequential bit writer for GFX12 primitive records, following Mesa's
struct AMDBitWriter
{
	uint8_t* data;
	uint32_t capacityBits;
	uint32_t position = 0;
	explicit AMDBitWriter( void* destination, uint32_t bytes = 128 ) : data( (uint8_t*)destination ), capacityBits( bytes * 8 )
	{
		memset( data, 0, bytes );
	}
	void Write( uint32_t value, uint32_t bitCount )
	{
		assert( bitCount <= 32 );
		assert( position <= capacityBits );
		assert( bitCount <= capacityBits - position );
		// A 64-bit temporary makes shifting by 32 safe.
		uint64_t remaining = value;
		while (bitCount)
		{
			const uint32_t shift = position & 31, available = 32 - shift;
			const uint32_t count = tinybvh_min( bitCount, available );
			const uint32_t mask = count == 32 ? UINT32_MAX : ((1u << count) - 1u);
			uint8_t* destination = data + (position / 32) * 4;
			uint32_t word;
			memcpy( &word, destination, sizeof( word ) );
			word = (word & ~(mask << shift)) | ((uint32_t( remaining ) & mask) << shift);
			memcpy( destination, &word, sizeof( word ) );
			remaining >>= count, position += count, bitCount -= count;
		}
	}
	void SkipTo( uint32_t targetBit )
	{
		assert( targetBit >= position );
		assert( targetBit <= capacityBits );
		position = targetBit;
	}
};

// Round the extent up to a power of two for 12-bit conservative child bounds.
static int32_t AMDQuantExp( const double extent )
{
	if (!(extent > 0)) return -115;
	uint32_t roundedBits = (tinybvh_as_uint( float( extent ) ) + 0x7fffffu) & 0x7f800000u;
	if (double( tinybvh_as_float( roundedBits ) ) < extent) roundedBits += 1u << 23;
	return int32_t( roundedBits >> 23 ) - 127;
}

// Encode leaves (one triangle pair each, grouped by ConvertFrom) as one primitive record at nodeDataPtr.
TEMPLATED void CWBVH8_AMD_HW<Float, Index>::EncodePrimNode( const uint32_t* leaves, const uint32_t pairCount, uint32_t& nodeDataPtr )
{
	const uint32_t recTris = pairCount * 2; // triangle slots: pair i's tri0 at i * 2, tri1 at i * 2 + 1
	const uint32_t leafNodeAddr = nodeDataPtr;
	// Primitive IDs per slot. A single triangle's missing tri1 repeats tri0's ID, as Mesa does (always
	// 2 * pairCount - 1 payloads); its vertices then deduplicate onto tri0's.
	uint32_t recIds[16];
	bool hasTri1[8];
	for (uint32_t i = 0; i < pairCount; i++)
	{
		const typename MBVH<8, Float, Index>::MBVHNode* const leaf = &bvh8.mbvhNode[leaves[i]];
		hasTri1[i] = leaf->triCount == 2;
		recIds[i * 2] = (uint32_t)bvh8.bvh.primIdx[leaf->firstTri];
		recIds[i * 2 + 1] = (uint32_t)bvh8.bvh.primIdx[leaf->firstTri + (hasTri1[i] ? 1 : 0)];
	}
	// TODO :: Not well optimized
	// Vertex soup and deduplication
	bvhuint3 vertSoup[16 * 3]; // Set to max
	uint32_t idxAssign[16 * 3];
	for (uint32_t i = 0; i < recTris; i++)
	{
		const uint32_t triIdx = recIds[i];
		uint32_t ti0, ti1, ti2;
		if (bvh8.bvh.vertIdx)
			ti0 = bvh8.bvh.vertIdx[triIdx * 3],
			ti1 = bvh8.bvh.vertIdx[triIdx * 3 + 1],
			ti2 = bvh8.bvh.vertIdx[triIdx * 3 + 2];
		else
			ti0 = triIdx * 3, ti1 = triIdx * 3 + 1, ti2 = triIdx * 3 + 2;
		int v = i * 3;
		bvhvec3 currentVert = bvh8.bvh.verts[ti0];
		vertSoup[v] = bvhuint3( tinybvh_as_uint( currentVert.x ), tinybvh_as_uint( currentVert.y ), tinybvh_as_uint( currentVert.z ) );
		currentVert = bvh8.bvh.verts[ti1];
		vertSoup[v + 1] = bvhuint3( tinybvh_as_uint( currentVert.x ), tinybvh_as_uint( currentVert.y ), tinybvh_as_uint( currentVert.z ) );
		currentVert = bvh8.bvh.verts[ti2];
		vertSoup[v + 2] = bvhuint3( tinybvh_as_uint( currentVert.x ), tinybvh_as_uint( currentVert.y ), tinybvh_as_uint( currentVert.z ) );
	}
	// Dedup
	bvhuint3 uniqueVerts[16 * 3];
	uint32_t uniqueVertCount = 0;
	bool found = false;
	for (uint32_t i = 0; i < recTris * 3; i++)
	{
		found = false;
		bvhuint3 currentVert = vertSoup[i];
		for (uint32_t j = 0; j < uniqueVertCount; j++)
		{
			if (currentVert.x == uniqueVerts[j].x && currentVert.y == uniqueVerts[j].y && currentVert.z == uniqueVerts[j].z)
			{
				idxAssign[i] = j, found = true;
				break;
			}
		}
		if (!found)
		{
			idxAssign[i] = uniqueVertCount;
			uniqueVerts[uniqueVertCount++] = currentVert;
		}
	}
	// 4-bit vertex indices with 0xF reserved for procedural nodes: at most 15 unique vertices. ConvertFrom checked this.
	BVH_FATAL_ERROR_IF( uniqueVertCount > 15, "CWBVH8_AMD_HW::ConvertFrom( .. ), triangle pairs do not fit in a primitive record." );
	// Compute Header Bit Widths and trailing zeroes
	uint32_t xPrefixBits = 0, yPrefixBits = 0, zPrefixBits = 0;
	const bvhuint3 refVert = uniqueVerts[0];
	uint32_t trailingZeroBits = refVert.x | refVert.y | refVert.z;
	for (uint32_t i = 1; i < uniqueVertCount; i++)
	{
		const bvhuint3 testVert = uniqueVerts[i];
		xPrefixBits |= refVert.x ^ testVert.x;
		yPrefixBits |= refVert.y ^ testVert.y;
		zPrefixBits |= refVert.z ^ testVert.z;
		// Trailing zeroes ( shared across all axis)
		trailingZeroBits |= testVert.x | testVert.y | testVert.z;
	}
	uint32_t trailingZeroes = trailingZeroBits != 0 ? tinybvh_ctz( trailingZeroBits ) : 31;
	// Axis where all verts are equal: everything goes in the prefix, leaving a 1-bit payload.
	xPrefixBits = xPrefixBits != 0 ? (31 - tinybvh_bfind( xPrefixBits )) : 31 - trailingZeroes;
	yPrefixBits = yPrefixBits != 0 ? (31 - tinybvh_bfind( yPrefixBits )) : 31 - trailingZeroes;
	zPrefixBits = zPrefixBits != 0 ? (31 - tinybvh_bfind( zPrefixBits )) : 31 - trailingZeroes;
	// Payload Widths
	const uint32_t xBitWidth = 31 - trailingZeroes - xPrefixBits;
	const uint32_t yBitWidth = 31 - trailingZeroes - yPrefixBits;
	const uint32_t zBitWidth = 31 - trailingZeroes - zPrefixBits;
	const uint32_t geomBaseIdx = 0;
	const uint32_t geomIdx = 0; // TODO : Currently No TLAS Support.
	// Find prim idx widths
	const uint32_t baseIdx = recIds[0];
	uint32_t primBaseIdx = baseIdx ? tinybvh_bfind( baseIdx ) + 1 : 0; // TODO : Can be improved by choosing best PrimIdx
	uint32_t primIdx = 0;
	for (uint32_t i = 1; i < recTris; i++)
	{
		const uint32_t testIdx = recIds[i];
		uint32_t testIdxWidth = baseIdx != testIdx ? tinybvh_bfind( baseIdx ^ testIdx ) + 1 : 0;
		if (testIdxWidth > primIdx) primIdx = testIdxWidth;
	}
	int dataBits = 1024 - 52 - xPrefixBits - yPrefixBits - zPrefixBits - 29 * pairCount - uniqueVertCount * (xBitWidth + yBitWidth + zBitWidth + 3) - primBaseIdx - primIdx * (2 * pairCount - 1);
	BVH_FATAL_ERROR_IF( dataBits < 0, "CWBVH8_AMD_HW::ConvertFrom( .. ), triangle pairs do not fit in a primitive record." ); // ConvertFrom checked this.
	const uint32_t midpointOffset = 52 + (xPrefixBits + yPrefixBits + zPrefixBits) + uniqueVertCount * (xBitWidth + yBitWidth + zBitWidth + 3) + (geomBaseIdx << 1) + (geomIdx << 1) * (2 * pairCount - 1);
	AMDBitWriter writer( &bvh8Data[leafNodeAddr] );
	// x bits | y bits | z bits | trailing bits | geom base | geom | pair count | vertex type
	writer.Write( xBitWidth, 5 );
	writer.Write( yBitWidth, 5 );
	writer.Write( zBitWidth, 5 );
	writer.Write( trailingZeroes, 5 );
	writer.Write( geomBaseIdx, 4 );
	writer.Write( geomIdx, 4 );
	writer.Write( pairCount - 1, 3 );
	writer.Write( 0, 1 );
	// Primitive ID widths and bit offset of the first primitive ID.
	writer.Write( primBaseIdx, 5 );
	writer.Write( primIdx, 5 );
	writer.Write( midpointOffset, 10 );
	// Write prefixes
	if (xPrefixBits) writer.Write( refVert.x >> (xBitWidth + 1 + trailingZeroes), xPrefixBits );
	if (yPrefixBits) writer.Write( refVert.y >> (yBitWidth + 1 + trailingZeroes), yPrefixBits );
	if (zPrefixBits) writer.Write( refVert.z >> (zBitWidth + 1 + trailingZeroes), zPrefixBits );
	for (uint32_t i = 0; i < uniqueVertCount; i++) // Write verts
	{
		bvhuint3 testVert = uniqueVerts[i];
		writer.Write( testVert.x >> trailingZeroes, xBitWidth + 1 );
		writer.Write( testVert.y >> trailingZeroes, yBitWidth + 1 );
		writer.Write( testVert.z >> trailingZeroes, zBitWidth + 1 );

	}
	// Geometry ID fields have zero width; primitive IDs follow the vertices.
	writer.SkipTo( midpointOffset );
	writer.Write( recIds[0], primBaseIdx ); // Original ID of the first triangle.
	for (uint32_t j = 1; j < recTris; j++) writer.Write( recIds[j], primIdx );
	writer.SkipTo( 1024 - 29 * pairCount );
	for (int i = pairCount - 1; i >= 0; i--)
	{
		writer.Write( 1, 1 ); // prim_range_stop
		writer.Write( 0, 1 ); // tri1_double_sided
		writer.Write( hasTri1[i] ? 1 : 0, 1 ); // tri1_opaque; no geometry flags in tinybvh, so all opaque.
		for (int j = 0; j < 3; j++) writer.Write( hasTri1[i] ? idxAssign[(i * 2 + 1) * 3 + j] : 0, 4 );
		writer.Write( 0, 1 ); // tri0_double_sided
		writer.Write( 1, 1 ); // tri0_opaque
		for (int j = 0; j < 3; j++) writer.Write( idxAssign[(i * 2) * 3 + j], 4 );
	}
	nodeDataPtr += 8;
}

// Add a leaf (one triangle pair) to a record: deduplicate its vertices and update the masks.
// Returns false if the record would exceed 15 unique vertices, leaving info half-updated.
TEMPLATED bool CWBVH8_AMD_HW<Float, Index>::getVertsInfo( encodingInfo& info, const uint32_t leafIdx ) const
{
	const typename MBVH<8, Float, Index>::MBVHNode* const leaf = &bvh8.mbvhNode[leafIdx];
	BVH_FATAL_ERROR_IF( leaf->triCount == 0 || leaf->triCount > 2,
		"CWBVH8_AMD_HW::ConvertFrom( .. ), leaf is not a triangle pair; set triangle limit to 2 first." );
	// A single triangle's missing tri1 repeats tri0's ID: no new ID bits.
	for (uint32_t tri = 0; tri < leaf->triCount; tri++)
	{
		const uint32_t triIdx = (uint32_t)bvh8.bvh.primIdx[leaf->firstTri + tri];
		if (info.pairCount == 0 && tri == 0) info.baseIdx = triIdx;
		info.idxDiffBits |= info.baseIdx ^ triIdx;
		for (uint32_t i = 0; i < 3; i++)
		{
			uint32_t ti;
			if (bvh8.bvh.vertIdx) ti = bvh8.bvh.vertIdx[triIdx * 3 + i]; else ti = triIdx * 3 + i;
			const bvhvec3 vF = bvh8.bvh.verts[ti];
			const bvhuint3 currentVert( tinybvh_as_uint( vF.x ), tinybvh_as_uint( vF.y ), tinybvh_as_uint( vF.z ) );
			bool found = false;
			for (uint32_t j = 0; j < info.uniqueVertCount && !found; j++)
				found = currentVert.x == info.uniqueVerts[j].x && currentVert.y == info.uniqueVerts[j].y && currentVert.z == info.uniqueVerts[j].z;
			if (found) continue;
			if (info.uniqueVertCount == 15) return false;
			info.uniqueVerts[info.uniqueVertCount++] = currentVert;
			const bvhuint3 refVert = info.uniqueVerts[0];
			info.xPrefixBits |= refVert.x ^ currentVert.x;
			info.yPrefixBits |= refVert.y ^ currentVert.y;
			info.zPrefixBits |= refVert.z ^ currentVert.z;
			// Trailing zeroes ( shared across all axis)
			info.trailingZeroBits |= currentVert.x | currentVert.y | currentVert.z;
		}
	}
	info.pairCount++;
	return true;
}

// Bits left in a record holding info's pairs, negative if they do not fit; same layout as EncodePrimNode.
TEMPLATED int32_t CWBVH8_AMD_HW<Float, Index>::computeNewNodeSize( const encodingInfo& info )
{
	const uint32_t trailingZeroes = info.trailingZeroBits != 0 ? tinybvh_ctz( info.trailingZeroBits ) : 31;
	// Axis where all verts are equal: everything goes in the prefix, leaving a 1-bit payload.
	const uint32_t xPrefixBits = info.xPrefixBits != 0 ? (31 - tinybvh_bfind( info.xPrefixBits )) : 31 - trailingZeroes;
	const uint32_t yPrefixBits = info.yPrefixBits != 0 ? (31 - tinybvh_bfind( info.yPrefixBits )) : 31 - trailingZeroes;
	const uint32_t zPrefixBits = info.zPrefixBits != 0 ? (31 - tinybvh_bfind( info.zPrefixBits )) : 31 - trailingZeroes;
	const uint32_t xBitWidth = 31 - trailingZeroes - xPrefixBits;
	const uint32_t yBitWidth = 31 - trailingZeroes - yPrefixBits;
	const uint32_t zBitWidth = 31 - trailingZeroes - zPrefixBits;
	const uint32_t primBaseIdx = info.baseIdx ? tinybvh_bfind( info.baseIdx ) + 1 : 0;
	const uint32_t primIdx = info.idxDiffBits ? tinybvh_bfind( info.idxDiffBits ) + 1 : 0;
	const int32_t pairCount = (int32_t)info.pairCount;
	return 1024 - 52 - (int32_t)(xPrefixBits + yPrefixBits + zPrefixBits) - 29 * pairCount - (int32_t)(info.uniqueVertCount * (xBitWidth + yBitWidth + zBitWidth + 3)) - (int32_t)primBaseIdx - (int32_t)primIdx * (2 * pairCount - 1);
}

TEMPLATED bool CWBVH8_AMD_HW<Float, Index>::Save( const char* fileName )
{
	std::fstream s{ fileName, s.binary | s.out };
	if (!s) return false;
	const uint32_t header = this->CacheHeader();
	s.write( (char*)&header, sizeof( uint32_t ) );
	s.write( (char*)&triCount, sizeof( Index ) );
	s.write( (char*)this, sizeof( CWBVH8_AMD_HW ) );
	s.write( (char*)bvh8Data, (size_t)usedBlocks * 16 );
	// write errors may only surface when the stream is flushed
	s.close();
	return !s.fail();
}

TEMPLATED bool CWBVH8_AMD_HW<Float, Index>::Load( const char* fileName, const Index expectedTris )
{
	// open file and check contents
	std::fstream s{ fileName, s.binary | s.in };
	if (!s) return false;
	BVHContext tmp = context;
	uint32_t header;
	Index fileTriCount;
	s.read( (char*)&header, sizeof( uint32_t ) );
	if (header != this->CacheHeader()) return false;
	s.read( (char*)&fileTriCount, sizeof( Index ) );
	if (fileTriCount != expectedTris) return false;
	// all checks passed: release what *this holds, then overwrite it with the file's image.
	uint32_t* opmapKeep = this->opmap, opmapNKeep = this->opmapN; // opacity maps are caller-owned; keep what was set.
	*this = CWBVH8_AMD_HW( tmp );
	s.read( (char*)this, sizeof( CWBVH8_AMD_HW ) );
	context = tmp; // can't load context; function pointers will differ.
	this->opmap = opmapKeep, this->opmapN = opmapNKeep;
	bvh8Data = (bvhvec4*)AlignedAlloc( (size_t)usedBlocks * 16 );
	allocatedBlocks = usedBlocks;
	s.read( (char*)bvh8Data, (size_t)usedBlocks * 16 );
	bvh8.DropReference( context ); // holds raw file data: reset, don't free.
	return true;
}

TEMPLATED void CWBVH8_AMD_HW<Float, Index>::ConvertFrom( const MBVH<8, Float, Index>& original, bool /* compact: allocations are exact */ )
{
	// get a copy of the original bvh8
	if (&original != &bvh8)
	{
		if (ownBVH8) bvh8 = MBVH<8, Float, Index>( context ); else bvh8.DropReference( context );
		ownBVH8 = false; // bvh isn't ours; don't delete in destructor.
	}
	bvh8.ReferenceFrom( original );
	CopyBasePropertiesFrom( bvh8 );
	typedef typename MBVH<8, Float, Index>::MBVHNode Node;
	const Node* node = bvh8.mbvhNode;
	const uint32_t NONE = 0xffffffffu;
	// the leaf children of a box node (one triangle pair each); a root that is a leaf is its own box's child.
	auto leavesOf = [&]( const uint32_t idx, uint32_t* leaf )
	{
		if (node[idx].isLeaf()) { leaf[0] = idx; return 1u; }
		uint32_t n = 0;
		for (int i = 0; i < 8; i++) if (node[idx].child[i] != 0 && node[node[idx].child[i]].isLeaf()) leaf[n++] = (uint32_t)node[idx].child[i];
		return n;
	};
	// 1. Box nodes, depth first. The box children of a node are consecutive: they share internal_base_id.
	uint32_t boxCount = 1;
	{
		uint32_t* stack = (uint32_t*)AlignedAlloc( (size_t)tinybvh_max( (uint32_t)bvh8.usedNodes, 1u ) * 4 ), stackPtr = 0;
		if (!node[0].isLeaf()) stack[stackPtr++] = 0, boxCount = 0;
		while (stackPtr > 0)
		{
			const Node& n = node[stack[--stackPtr]];
			boxCount++;
			for (int i = 0; i < 8; i++) if (n.child[i] != 0 && !node[n.child[i]].isLeaf()) stack[stackPtr++] = (uint32_t)n.child[i];
		}
		AlignedFree( stack );
	}
	uint32_t* boxSrc = (uint32_t*)AlignedAlloc( (size_t)boxCount * 4 ), * boxFirst = (uint32_t*)AlignedAlloc( (size_t)boxCount * 4 );
	uint32_t* boxParent = (uint32_t*)AlignedAlloc( (size_t)boxCount * 4 ), * boxPlaced = (uint32_t*)AlignedAlloc( (size_t)boxCount * 4 );
	uint32_t* boxLeafBase = (uint32_t*)AlignedAlloc( ((size_t)boxCount + 1) * 4 ); // first leaf of each box in the stream
	uint32_t* stack = (uint32_t*)AlignedAlloc( (size_t)boxCount * 4 ), stackPtr = 1, allocated = 1;
	boxSrc[0] = 0, boxParent[0] = NONE, stack[0] = 0, boxLeafBase[0] = 0;
	while (stackPtr > 0)
	{
		const uint32_t b = stack[--stackPtr];
		const Node& n = node[boxSrc[b]];
		uint32_t leaf[8];
		boxLeafBase[b + 1] = leavesOf( boxSrc[b], leaf );
		boxFirst[b] = NONE;
		if (!n.isLeaf()) for (int i = 0; i < 8; i++) if (n.child[i] != 0 && !node[n.child[i]].isLeaf())
		{
			if (boxFirst[b] == NONE) boxFirst[b] = allocated;
			boxSrc[allocated] = (uint32_t)n.child[i], boxParent[allocated] = PackNodeID( b * 128, NODE_BOX32 );
			stack[stackPtr++] = allocated++;
		}
	}
	AlignedFree( stack );
	for (uint32_t b = 0; b < boxCount; b++) boxLeafBase[b + 1] += boxLeafBase[b];
	const uint32_t leafTotal = boxLeafBase[boxCount];
	// 2. Records. The leaves go into one stream of records, box by box: the record a box leaves open is
	// continued by the next box, picked from the next few as the one with the best fitting leaf. A box's
	// leaves stay consecutive in the stream, so its primitive children step through them from one base,
	// starting at any pair of a record (GPURT DecodeChildrenOffsets: base | pair type, plus range steps).
	// Groups of boxes are packed in parallel, each into its own stream; their number depends on the box
	// count only, so the result does not depend on the thread count.
	uint32_t* placed = (uint32_t*)AlignedAlloc( (size_t)leafTotal * 4 ); // leaves in stream order
	uint32_t* placedRecord = (uint32_t*)AlignedAlloc( (size_t)leafTotal * 4 );
	uint32_t* groupStart = (uint32_t*)AlignedAlloc( ((size_t)leafTotal + 1) * 4 ); // record starts, per group from its first leaf
	uint8_t* done = (uint8_t*)AlignedAlloc( boxCount );
	memset( done, 0, boxCount );
	const uint32_t groups = tinybvh_min( tinybvh_max( boxCount / 4096u, 1u ), 256u );
	uint32_t groupRecords[256];
	// free bits after adding a leaf to info, negative if it does not fit
	auto fit = [&]( const encodingInfo& info, const uint32_t leaf, encodingInfo& out )
	{
		if (info.pairCount == 8) return -1; // 3-bit pair count
		out = info;
		return getVertsInfo( out, leaf ) ? computeNewNodeSize( out ) : -1;
	};
	this->ParallelFor( groups > 1 && this->PoolAvailable(), groups, [&]( const uint32_t g )
	{
		const uint32_t lastBox = (uint32_t)Base::SliceStart( boxCount, g + 1, groups );
		uint32_t placedCount = boxLeafBase[Base::SliceStart( boxCount, g, groups )], recordCount = 0, * recordStart = groupStart + placedCount;
		encodingInfo open;
		bool isOpen = false;
		for (uint32_t nextBox = (uint32_t)Base::SliceStart( boxCount, g, groups );; )
		{
			while (nextBox < lastBox && done[nextBox]) nextBox++;
			if (nextBox == lastBox) break;
			uint32_t pick = nextBox, left[8];
			if (isOpen) for (uint32_t b = nextBox, seen = 0, bestBits = 0; b < lastBox && seen < recordLookahead; b++) if (!done[b])
			{
				const uint32_t n = leavesOf( boxSrc[b], left );
				if (n > 0) seen++;
				for (uint32_t i = 0; i < n; i++)
				{
					encodingInfo tmp;
					const int32_t bits = fit( open, left[i], tmp );
					if (bits >= (int32_t)bestBits) bestBits = (uint32_t)bits + 1, pick = b; // + 1: zero bits left still fits
				}
			}
			done[pick] = 1, boxPlaced[pick] = placedCount;
			uint32_t leftCount = leavesOf( boxSrc[pick], left );
			while (leftCount > 0)
			{
				uint32_t best = 0;
				if (!isOpen)
				{
					open = encodingInfo();
					BVH_FATAL_ERROR_IF( !getVertsInfo( open, left[0] ) || computeNewNodeSize( open ) < 0,
						"CWBVH8_AMD_HW::ConvertFrom( .. ), triangle pair does not fit in a primitive record." );
					recordStart[recordCount++] = placedCount, isOpen = true;
				}
				else
				{
					int32_t bestBits = -1;
					encodingInfo bestInfo, tmp;
					for (uint32_t i = 0; i < leftCount; i++)
					{
						const int32_t bits = fit( open, left[i], tmp );
						if (bits > bestBits) bestBits = bits, best = i, bestInfo = tmp;
					}
					if (bestBits < 0) { isOpen = false; continue; } // nothing fits: close the record
					open = bestInfo;
				}
				placedRecord[placedCount] = recordCount - 1, placed[placedCount++] = left[best];
				left[best] = left[--leftCount];
			}
		}
		groupRecords[g] = recordCount;
	} );
	AlignedFree( done );
	// number the records of all groups in one sequence
	uint32_t recordCount = 0;
	for (uint32_t g = 0; g < groups; g++) recordCount += groupRecords[g];
	uint32_t* recordStart = (uint32_t*)AlignedAlloc( ((size_t)recordCount + 1) * 4 ); // first placed leaf per record
	for (uint32_t g = 0, firstRecord = 0; g < groups; firstRecord += groupRecords[g++])
	{
		const uint32_t firstLeaf = boxLeafBase[Base::SliceStart( boxCount, g, groups )], endLeaf = boxLeafBase[Base::SliceStart( boxCount, g + 1, groups )];
		for (uint32_t r = 0; r < groupRecords[g]; r++) recordStart[firstRecord + r] = groupStart[firstLeaf + r];
		for (uint32_t p = firstLeaf; p < endLeaf; p++) placedRecord[p] += firstRecord;
	}
	recordStart[recordCount] = leafTotal;
	AlignedFree( groupStart ), AlignedFree( boxLeafBase );
	// 3. Write: box nodes first, then the record stream. Node IDs hold 32-bit byte offsets: below 4 GB.
	const uint64_t blocks64 = (uint64_t)(boxCount + recordCount) * 8;
	BVH_FATAL_ERROR_IF( blocks64 > (1ull << 28), "CWBVH8_AMD_HW::ConvertFrom, BVH exceeds the 4 GB addressable by node IDs." );
	if (allocatedBlocks != (uint32_t)blocks64)
	{
		AlignedFree( bvh8Data );
		bvh8Data = (bvhvec4*)AlignedAlloc( (size_t)blocks64 * 16 );
		allocatedBlocks = (uint32_t)blocks64;
	}
	usedBlocks = (uint32_t)blocks64;
	const uint32_t recordBase = boxCount * 8, recordTasks = Base::TaskCount( this->PoolAvailable(), recordCount, 2048, 256 );
	this->ParallelFor( recordTasks > 1, recordTasks, [&]( const uint32_t s )
	{
		for (uint32_t r = (uint32_t)Base::SliceStart( recordCount, s, recordTasks ); r < (uint32_t)Base::SliceStart( recordCount, s + 1, recordTasks ); r++)
		{
			uint32_t ptr = recordBase + r * 8;
			EncodePrimNode( placed + recordStart[r], recordStart[r + 1] - recordStart[r], ptr );
		}
	} );
	const uint32_t boxTasks = Base::TaskCount( this->PoolAvailable(), boxCount, 2048, 256 );
	this->ParallelFor( boxTasks > 1, boxTasks, [&]( const uint32_t s )
	{
		for (uint32_t b = (uint32_t)Base::SliceStart( boxCount, s, boxTasks ); b < (uint32_t)Base::SliceStart( boxCount, s + 1, boxTasks ); b++)
		{
			const Node& orig = node[boxSrc[b]];
			const uint32_t addr = b * 8;
			memset( &bvh8Data[addr], 0, 128 );
			const Vec3 nodeLo = orig.aabbMin, nodeHi = orig.aabbMax;
			const int32_t ex = AMDQuantExp( double( nodeHi.x ) - double( nodeLo.x ) );
			const int32_t ey = AMDQuantExp( double( nodeHi.y ) - double( nodeLo.y ) );
			const int32_t ez = AMDQuantExp( double( nodeHi.z ) - double( nodeLo.z ) );
			const double extent[3] = { tinybvh_as_float( uint32_t( ex + 127 ) << 23 ), tinybvh_as_float( uint32_t( ey + 127 ) << 23 ), tinybvh_as_float( uint32_t( ez + 127 ) << 23 ) };
			// 12-bit child bounds: minima rounded down, maxima rounded up (stored minus one, clamped at zero).
			uint32_t* childInfo = (uint32_t*)&bvh8Data[addr] + 8, slotsUsed = 0;
			auto quantize = [&]( const bvhvec3& bmin, const bvhvec3& bmax, const uint32_t typeAndSize )
			{
				uint32_t q[6];
				for (int a = 0; a < 3; a++)
				{
					q[a] = tinybvh_min( (uint32_t)tinybvh_max( floor( (double( bmin[a] ) - double( nodeLo[a] )) / extent[a] * 4096.0 ), 0.0 ), 4095u );
					q[a + 3] = tinybvh_min( (uint32_t)tinybvh_max( ceil( (double( bmax[a] ) - double( nodeLo[a] )) / extent[a] * 4096.0 ) - 1.0, 0.0 ), 4095u );
				}
				childInfo[0] = q[0] | (q[1] << 12);
				childInfo[1] = q[2] | (q[3] << 12) | (0xffu << 24); // all ray mask bits enabled
				childInfo[2] = q[4] | (q[5] << 12) | typeAndSize;
				childInfo += 3, slotsUsed++;
			};
			if (!orig.isLeaf()) for (int i = 0; i < 8; i++) if (orig.child[i] != 0 && !node[orig.child[i]].isLeaf())
				quantize( node[orig.child[i]].aabbMin, node[orig.child[i]].aabbMax, (NODE_BOX32 << 24) | (1u << 28) ); // one 128-byte box node
			uint32_t leaf[8];
			const uint32_t leafCount = leavesOf( boxSrc[b], leaf );
			for (uint32_t j = 0; j < leafCount; j++)
			{
				const uint32_t p = boxPlaced[b] + j, r = placedRecord[p], pair = p - recordStart[r];
				const Node& child = node[placed[p]];
				// the pair's box comes from its triangles: a leaf box can be looser.
				bvhvec3 pairMin( BVH_FAR ), pairMax( -BVH_FAR );
				for (uint32_t t = 0; t < child.triCount; t++)
				{
					const uint32_t triIdx = (uint32_t)bvh8.bvh.primIdx[child.firstTri + t];
					for (uint32_t k = 0; k < 3; k++)
					{
						const bvhvec3 v = bvh8.bvh.verts[bvh8.bvh.vertIdx ? bvh8.bvh.vertIdx[triIdx * 3 + k] : triIdx * 3 + k];
						pairMin = tinybvh_min( pairMin, v ), pairMax = tinybvh_max( pairMax, v );
					}
				}
				// With spatial splits a triangle can reach beyond its leaf; the leaf box clips it.
				pairMin = tinybvh_max( pairMin, bvhvec3( child.aabbMin ) ), pairMax = tinybvh_min( pairMax, bvhvec3( child.aabbMax ) );
				// range: records to step to the next child of this box; 1 for the last child, as GPURT writes it.
				const uint32_t range = j + 1 < leafCount ? placedRecord[p + 1] - r : 1u;
				quantize( pairMin, pairMax, (((pair & 3) | (pair >= 4 ? 8 : 0)) << 24) | (range << 28) ); // pair index as the type
			}
			for (uint32_t i = slotsUsed; i < 8; i++, childInfo += 3) childInfo[0] = 0xffffffffu, childInfo[1] = 0xfffu, childInfo[2] = 0; // empty
			uint32_t* header = (uint32_t*)&bvh8Data[addr];
			header[0] = boxFirst[b] == NONE ? 0 : boxFirst[b] * 16; // internal_base_id: 8-byte units
			header[1] = leafCount == 0 ? 0 : (recordBase + placedRecord[boxPlaced[b]] * 8) * 2; // primitive_base_id
			header[2] = boxParent[b]; // packed parent ID, for traversal stack overflow recovery
			memcpy( header + 3, &nodeLo.x, 4 ), memcpy( header + 4, &nodeLo.y, 4 ), memcpy( header + 5, &nodeLo.z, 4 ); // origin
			header[6] = uint32_t( ex + 127 ) | (uint32_t( ey + 127 ) << 8) | (uint32_t( ez + 127 ) << 16) | ((slotsUsed - 1) << 28); // exponents, child count - 1
			header[7] = 0x7f; // disable oriented bounding boxes
		}
	} );
	AlignedFree( boxSrc ), AlignedFree( boxFirst ), AlignedFree( boxParent ), AlignedFree( boxPlaced );
	AlignedFree( placed ), AlignedFree( placedRecord ), AlignedFree( recordStart );
}

// n <= 32 bits at bit pos of a 1024-bit record, LSB first.
static inline uint32_t tinybvh_record_bits( const uint32_t* w, const uint32_t pos, const uint32_t n )
{
	if (n == 0) return 0;
	const uint64_t lo = w[pos >> 5], hi = (pos >> 5) + 1 < 32 ? w[(pos >> 5) + 1] : 0;
	return (uint32_t)(((lo | (hi << 32)) >> (pos & 31)) & (n == 32 ? 0xffffffffull : (1ull << n) - 1));
}

// Decoding per GPURT primitiveNode.hlsli: ReadVertex, ReadPrimIdPayload, ReadTrianglePairDesc, Tri1Valid.
TEMPLATED uint32_t CWBVH8_AMD_HW<Float, Index>::RecordPair( const AMDTriangleNode& record, const uint32_t p, Vec3* v, uint32_t* prim )
{
	const uint32_t* w = record.words;
	const uint32_t tz = tinybvh_record_bits( w, 15, 5 ), anchorBits = tinybvh_record_bits( w, 32, 5 );
	const uint32_t payloadBits = tinybvh_record_bits( w, 37, 5 ), mid = tinybvh_record_bits( w, 42, 10 );
	uint32_t width[3], prefix[3], pos = 52;
	for (int a = 0; a < 3; a++)
	{
		width[a] = tinybvh_record_bits( w, a * 5, 5 ) + 1;
		const uint32_t prefixBits = 32 - tz - width[a];
		prefix[a] = prefixBits ? tinybvh_record_bits( w, pos, prefixBits ) << (32 - prefixBits) : 0;
		pos += prefixBits;
	}
	const uint32_t stride = width[0] + width[1] + width[2];
	const uint32_t desc = tinybvh_record_bits( w, 1024 - 29 * (p + 1), 29 );
	const uint32_t idx[2][3] = { { (desc >> 17) & 15, (desc >> 21) & 15, (desc >> 25) & 15 }, { (desc >> 3) & 15, (desc >> 7) & 15, (desc >> 11) & 15 } };
	const uint32_t tris = (idx[1][0] | idx[1][1] | idx[1][2]) ? 2 : 1;
	const uint32_t anchor = tinybvh_record_bits( w, mid, anchorBits );
	for (uint32_t t = 0; t < tris; t++)
	{
		for (int c = 0; c < 3; c++)
		{
			uint32_t vpos = pos + idx[t][c] * stride, bits[3];
			for (int a = 0; a < 3; a++) bits[a] = prefix[a] | (tinybvh_record_bits( w, vpos, width[a] ) << tz), vpos += width[a];
			v[t * 3 + c] = Vec3( tinybvh_as_float( bits[0] ), tinybvh_as_float( bits[1] ), tinybvh_as_float( bits[2] ) );
		}
		const uint32_t slot = p * 2 + t;
		if (slot == 0) prim[t] = anchor; else
		{
			const uint32_t payload = tinybvh_record_bits( w, mid + anchorBits + (slot - 1) * payloadBits, payloadBits );
			prim[t] = payloadBits >= anchorBits ? payload : (anchor & ~((1u << payloadBits) - 1u)) | payload;
		}
	}
	return tris;
}

// Child pointers per GPURT QuantizedBVH8BoxNode::DecodeChildrenOffsets: box children and primitive children
// have separate running offsets, each advanced by the child's range (in 128-byte units); a primitive child's
// pointer holds the 4-bit pair type.
TEMPLATED int32_t CWBVH8_AMD_HW<Float, Index>::Intersect( Ray& ray ) const
{
	struct Entry { uint32_t id; float t; } stack[TINYBVH_STACK_SIZE * 8], hit[8];
	const uint8_t* base = (const uint8_t*)bvh8Data;
	uint32_t stackPtr = 0, id = PackNodeID( 0, NODE_BOX32 ), steps = 0;
	while (1)
	{
		const uint32_t* w = (const uint32_t*)(base + ((size_t)(id & ~15u) << 3));
		steps++;
		if ((id & 15) == NODE_BOX32)
		{
			const uint32_t childCount = ((w[6] >> 28) & 7) + 1;
			const double origin[3] = { tinybvh_as_float( w[3] ), tinybvh_as_float( w[4] ), tinybvh_as_float( w[5] ) };
			double step[3];
			for (int a = 0; a < 3; a++) step[a] = double( tinybvh_as_float( ((w[6] >> (a * 8)) & 255) << 23 ) ) / 4096.0;
			uint32_t internalOffset = w[0], primitiveOffset = w[1], hits = 0;
			for (uint32_t c = 0; c < childCount; c++)
			{
				const uint32_t* ci = w + 8 + c * 3, type = (ci[2] >> 24) & 15, range = ci[2] >> 28;
				const uint32_t child = type == NODE_BOX32 ? (internalOffset | NODE_BOX32) : (primitiveOffset | type);
				(type == NODE_BOX32 ? internalOffset : primitiveOffset) += range << 4;
				const uint32_t q[6] = { ci[0] & 4095, (ci[0] >> 12) & 4095, ci[1] & 4095, (ci[1] >> 12) & 4095, ci[2] & 4095, (ci[2] >> 12) & 4095 };
				const double lo[3] = { origin[0] + q[0] * step[0], origin[1] + q[1] * step[1], origin[2] + q[2] * step[2] };
				const double hi[3] = { origin[0] + (q[3] + 1) * step[0], origin[1] + (q[4] + 1) * step[1], origin[2] + (q[5] + 1) * step[2] };
				Vec3 bmin, bmax;
				for (int a = 0; a < 3; a++) // round outwards, so the decoded box stays conservative
				{
					bmin[a] = (float)lo[a], bmax[a] = (float)hi[a];
					if ((double)bmin[a] > lo[a]) bmin[a] = nextafterf( (float)bmin[a], -BVH_FAR );
					if ((double)bmax[a] < hi[a]) bmax[a] = nextafterf( (float)bmax[a], BVH_FAR );
				}
				const float t = tinybvh_intersect_aabb( ray, bmin, bmax );
				if (t >= BVH_FAR) continue;
				uint32_t j = hits++;
				for (; j > 0 && hit[j - 1].t > t; j--) hit[j] = hit[j - 1];
				hit[j] = { child, t };
			}
			BVH_FATAL_ERROR_IF( stackPtr + hits > TINYBVH_STACK_SIZE * 8, "CWBVH8_AMD_HW::Intersect, stack overflow." );
			while (hits > 0) stack[stackPtr++] = hit[--hits]; // nearest on top
		}
		else
		{
			Vec3 v[6];
			uint32_t prim[2];
			const uint32_t tris = RecordPair( *(const AMDTriangleNode*)w, (id & 3) | ((id & 8) ? 4 : 0), v, prim );
			for (uint32_t t = 0; t < tris; t++) this->IntersectTri( ray, (Index)prim[t], v[t * 3], v[t * 3 + 1], v[t * 3 + 2] );
		}
		while (stackPtr > 0 && stack[stackPtr - 1].t >= ray.hit.t) stackPtr--;
		if (stackPtr == 0) break;
		id = stack[--stackPtr].id;
	}
	return (int32_t)steps;
}

// BVH8_CWBVH implementation
// ----------------------------------------------------------------------------

TEMPLATED BVH8_CWBVH<Float, Index>::BVH8_CWBVH( BVH8_CWBVH&& other ) noexcept
{
	*this = other;
	// The embedded bvh8 must let go too: owned or not, this object now refers to
	// the same buffers, and only one of the two may release them.
	other.bvh8.ReleaseOwnership();
	other.bvh8Data = 0;
	other.bvh8Tris = 0;
}

// move assignment: release what we hold, then take over 'other'.
TEMPLATED BVH8_CWBVH<Float, Index>& BVH8_CWBVH<Float, Index>::operator=( BVH8_CWBVH&& other ) noexcept
{
	if (this != &other) this->~BVH8_CWBVH(), new (this) BVH8_CWBVH( tinybvh_move( other ) );
	return *this;
}

TEMPLATED BVH8_CWBVH<Float, Index>::~BVH8_CWBVH()
{
	if (!ownBVH8) bvh8.ReleaseOwnership(); // clear out pointers we don't own.
	AlignedFree( bvh8Data );
	AlignedFree( bvh8Tris );
}

// forwarders
TEMPLATED void BVH8_CWBVH<Float, Index>::Build( const bvhvec4* v, const Index p ) { Build( Slice( v, p * 3, sizeof( bvhvec4 ) ) ); }
TEMPLATED void BVH8_CWBVH<Float, Index>::Build( const Slice& v ) { Build( v, 0, 0 ); }
TEMPLATED void BVH8_CWBVH<Float, Index>::Build( const bvhvec4* v, const Index* i, const Index p ) { Build( Slice( v, p * 3, sizeof( bvhvec4 ) ), i, p ); }
TEMPLATED void BVH8_CWBVH<Float, Index>::BuildHQ( const bvhvec4* v, const Index p ) { BuildHQ( Slice( v, p * 3, sizeof( bvhvec4 ) ) ); }
TEMPLATED void BVH8_CWBVH<Float, Index>::BuildHQ( const Slice& vertices ) { BuildHQ( vertices, 0, 0 ); }
TEMPLATED void BVH8_CWBVH<Float, Index>::BuildHQ( const bvhvec4* v, const Index* i, const Index p ) { BuildHQ( Slice( v, p * 3, sizeof( bvhvec4 ) ), i, p ); }
TEMPLATED void BVH8_CWBVH<Float, Index>::BuildHQ( const Slice& v, const Index* i, Index p ) { ScopedFlag hq( settings.useSpatialSplits, true ); Build( v, i, p ); }

TEMPLATED void BVH8_CWBVH<Float, Index>::Build( const Slice& vertices, const Index* indices, Index prims )
{
	// a tree received through ConvertFrom is not ours to rebuild: drop the reference, start afresh.
	if (!ownBVH8) bvh8.DropReference( context ), ownBVH8 = true;
	// propagate settings for this layout to the underlying layout
	this->PassSettingsTo( bvh8 ), this->PassSettingsTo( bvh8.bvh );
	// build underlying layout
	bvh8.bvh.Build( vertices, indices, prims );
	// split leaves larger than a leaf slot; the collapse decides which small subtrees become leaves,
	// rather than FitLeafs taking each one that fits. Not keepSplitting, unlike BVH4_CPU: leaves
	// here are priced per prim, and from one-prim leaves the collapse makes leaves too small to pay.
	bvh8.bvh.SplitLeafs( 3 );
	// convert to BVH8_CWBVH layout
	bvh8.leafPrimLimit = 3; // a cwbvh leaf slot holds at most 3 prims
	bvh8.ConvertFrom( bvh8.bvh, true );
	ConvertFrom( bvh8, true );
}

TEMPLATED void BVH8_CWBVH<Float, Index>::Optimize( const uint32_t iterations, uint32_t mu )
{
	bvh8.Optimize( iterations, mu );
	ConvertFrom( bvh8, true );
}

TEMPLATED Float BVH8_CWBVH<Float, Index>::SAHCost( const Index nodeIdx ) const
{
	return bvh8.SAHCost( nodeIdx );
}

TEMPLATED bool BVH8_CWBVH<Float, Index>::Save( const char* fileName )
{
	std::fstream s{ fileName, s.binary | s.out };
	if (!s) return false;
	const uint32_t header = this->CacheHeader();
	s.write( (char*)&header, sizeof( uint32_t ) );
	s.write( (char*)&triCount, sizeof( Index ) );
	s.write( (char*)this, sizeof( BVH8_CWBVH ) );
	s.write( (char*)bvh8Data, (size_t)usedBlocks * 16 );
	s.write( (char*)bvh8Tris, (size_t)usedTriBlocks * 16 );
	// write errors may only surface when the stream is flushed
	s.close();
	return !s.fail();
}

TEMPLATED bool BVH8_CWBVH<Float, Index>::Load( const char* fileName, const Index expectedTris )
{
	// open file and check contents
	std::fstream s{ fileName, s.binary | s.in };
	if (!s) return false;
	BVHContext tmp = context;
	uint32_t header;
	Index fileTriCount;
	s.read( (char*)&header, sizeof( uint32_t ) );
	if (header != this->CacheHeader()) return false;
	s.read( (char*)&fileTriCount, sizeof( Index ) );
	if (fileTriCount != expectedTris) return false;
	// all checks passed: release what *this holds, then overwrite it with the file's image.
	uint32_t* opmapKeep = this->opmap, opmapNKeep = this->opmapN; // opacity maps are caller-owned; keep what was set.
	*this = BVH8_CWBVH( tmp );
	s.read( (char*)this, sizeof( BVH8_CWBVH ) );
	context = tmp; // can't load context; function pointers will differ.
	this->opmap = opmapKeep, this->opmapN = opmapNKeep;
	bvh8Data = (bvhvec4*)AlignedAlloc( (size_t)usedBlocks * 16 );
	bvh8Tris = (bvhvec4*)AlignedAlloc( (size_t)usedTriBlocks * 16 );
	allocatedBlocks = usedBlocks, allocatedTris = usedTriBlocks;
	s.read( (char*)bvh8Data, (size_t)usedBlocks * 16 );
	s.read( (char*)bvh8Tris, (size_t)usedTriBlocks * 16 );
	bvh8.DropReference( context ); // holds raw file data: reset, don't free.
	return true;
}

// quantization exponent, guarded against zero / denormal / NaN extents.
static int32_t CWBVHQuantExp( const float extent )
{
	if (!(extent > 0)) return -126; // zero, denormal or NaN extent
	// ceil( log2( x ) ) for positive x is its unbiased exponent, plus one unless
	// the mantissa is exactly 1; no need to call out to log2f for that.
	const uint32_t b = tinybvh_as_uint( extent * (1.0f / 255.0f) );
	const int32_t e = (int32_t)((b >> 23) & 255) - 127;
	if (e < -126) return -126;	// subnormal product
	if (e > 125) return 126;	// inf, or an exponent that would clamp anyway
	return (b & 0x7FFFFF) ? e + 1 : e;
}

// SlotPermutation: greedy child node ordering from "Efficient Incoherent Ray
// Traversal on GPUs Through Compressed Wide BVHs", Ylitie et al. 2017.
TEMPLATED uint32_t BVH8_CWBVH<Float, Index>::SlotPermutation( const MBVH<8, Float, Index>& bvh8, const Index nodeIdx,
	const Vec3& nodeLo, const float rsx, const float rsy, const float rsz, uint8_t* const qbase ) const
{
	const typename MBVH<8, Float, Index>::MBVHNode* const orig = &bvh8.mbvhNode[nodeIdx];
	const Vec3 nodeCentroid = (orig->aabbMin + orig->aabbMax) * 0.5f;
	float cost[8][8];
	int32_t assignment[8];
	bool isSlotEmpty[8];
	for (int32_t s = 0; s < 8; s++) isSlotEmpty[s] = true, assignment[s] = -1;
	for (int32_t i = 0; i < 8; i++)
	{
		if (orig->child[i] == 0) { for (int32_t s = 0; s < 8; s++) cost[s][i] = bvh_far<float>; continue; }
		const typename MBVH<8, Float, Index>::MBVHNode* const child = &bvh8.mbvhNode[orig->child[i]];
		const Vec3 d = (child->aabbMin + child->aabbMax) * 0.5f - nodeCentroid;
		for (int32_t s = 0; s < 8; s++)
			cost[s][i] = (float)(((s & 4) ? -d.x : d.x) + ((s & 2) ? -d.y : d.y) + ((s & 1) ? -d.z : d.z));
	}
	while (1)
	{
		float minCost = bvh_far<float>;
		int32_t minEntryx = -1, minEntryy = -1;
		for (int32_t s = 0; s < 8; s++) if (isSlotEmpty[s]) for (int32_t i = 0; i < 8; i++)
			if (assignment[i] == -1 && cost[s][i] < minCost)
				minCost = cost[s][i], minEntryx = s, minEntryy = i;
		if (minEntryx == -1) break;
		isSlotEmpty[minEntryx] = false, assignment[minEntryy] = minEntryx;
	}
	// An absent child is never assigned: the nibble of a slot it would have taken
	// stays 0xF, which the caller reads as an empty slot.
	uint32_t perm = 0xFFFFFFFF;
	for (uint32_t i = 0; i < 8; i++) if (assignment[i] != -1) perm ^= (0xF ^ i) << (assignment[i] * 4);
	// quantize the child bounds into the slots they were just assigned to. An
	// empty slot gets an inverted box, which no ray can enter.
	for (uint32_t i = 0; i < 8; i++)
	{
		const uint32_t c = (perm >> (i * 4)) & 15;
		if (c == 15)
		{
			qbase[i + 0] = qbase[i + 8] = qbase[i + 16] = 255;
			qbase[i + 24] = qbase[i + 32] = qbase[i + 40] = 0;
			continue;
		}
		const typename MBVH<8, Float, Index>::MBVHNode* const child = &bvh8.mbvhNode[orig->child[c]];
		qbase[i + 0] = (uint8_t)tinybvh_clamp( (int32_t)floorf( (float)(child->aabbMin.x - nodeLo.x) * rsx ), 0, 255 );
		qbase[i + 8] = (uint8_t)tinybvh_clamp( (int32_t)floorf( (float)(child->aabbMin.y - nodeLo.y) * rsy ), 0, 255 );
		qbase[i + 16] = (uint8_t)tinybvh_clamp( (int32_t)floorf( (float)(child->aabbMin.z - nodeLo.z) * rsz ), 0, 255 );
		qbase[i + 24] = (uint8_t)tinybvh_clamp( (int32_t)ceilf( (float)(child->aabbMax.x - nodeLo.x) * rsx ), 0, 255 );
		qbase[i + 32] = (uint8_t)tinybvh_clamp( (int32_t)ceilf( (float)(child->aabbMax.y - nodeLo.y) * rsy ), 0, 255 );
		qbase[i + 40] = (uint8_t)tinybvh_clamp( (int32_t)ceilf( (float)(child->aabbMax.z - nodeLo.z) * rsz ), 0, 255 );
	}
	return perm;
}

// Convert a BVH8 to the format specified in: "Efficient Incoherent Ray Traversal on GPUs Through
// Compressed Wide BVHs", Ylitie et al. 2017. Adapted from code by "AlanWBFT".
TEMPLATED void BVH8_CWBVH<Float, Index>::ConvertFrom( const MBVH<8, Float, Index>& original, bool compact )
{
	// get a copy of the original bvh8
	if (&original != &bvh8) ownBVH8 = false; // bvh isn't ours; don't delete in destructor.
	bvh8.ReferenceFrom( original );
	BVH_FATAL_ERROR_IF( bvh8.mbvhNode[0].isLeaf(), "BVH8_CWBVH::ConvertFrom( .. ), converting a single-node bvh." );
	CopyBasePropertiesFrom( bvh8 );
	uint32_t nodeCap = tinybvh_max( 128u, bvh8.triCount >> 1 );
	if (bvh8Data == 0 || nodeCap > allocatedBlocks)
	{
		AlignedFree( bvh8Data );
		bvh8Data = (bvhvec4*)AlignedAlloc( nodeCap * 16 );
		allocatedBlocks = nodeCap;
	}
#ifdef CWBVH_COMPRESSED_TRIS
	const uint32_t triCap = bvh8.idxCount * 4;
#else
	const uint32_t triCap = bvh8.idxCount * 3;
#endif
	if (bvh8Tris == 0 || triCap > allocatedTris)
	{
		AlignedFree( bvh8Tris );
		bvh8Tris = (bvhvec4*)AlignedAlloc( triCap * 16 );
		allocatedTris = triCap;
	}
	uint32_t stackCap = TINYBVH_STACK_SIZE, stackPtr = 1;
	uint32_t* stackNodeIdx = (uint32_t*)AlignedAlloc( stackCap * 4 );
	uint32_t* stackNodeAddr = (uint32_t*)AlignedAlloc( stackCap * 4 );
	stackNodeIdx[0] = 0, stackNodeAddr[0] = 0;
	uint32_t nodeDataPtr = 5, triDataPtr = 0;
	uint32_t slotsUsed = 0, nodeBlocks = 0, leafPrims = 0, leafBlocks = 0;
	// prim indices in output order; the triangle data itself is written after the walk.
	uint32_t* triPrim = (uint32_t*)AlignedAlloc( (size_t)tinybvh_max( 1u, (uint32_t)bvh8.idxCount ) * 4 );
	uint32_t nTris = 0;
	while (stackPtr > 0)
	{
		const uint32_t origIdx = stackNodeIdx[--stackPtr];
		const typename MBVH<8, Float, Index>::MBVHNode* orig = &bvh8.mbvhNode[origIdx];
		const uint32_t currentNodeAddr = stackNodeAddr[stackPtr];
		if (nodeDataPtr + 45 > allocatedBlocks)
		{
			const uint32_t newCap = tinybvh_max( allocatedBlocks + (allocatedBlocks >> 1), nodeDataPtr + 45 );
			bvhvec4* newData = (bvhvec4*)AlignedAlloc( newCap * 16 );
			memcpy( newData, bvh8Data, nodeDataPtr * 16 );
			AlignedFree( bvh8Data );
			bvh8Data = newData, allocatedBlocks = newCap;
		}
		const Vec3 nodeLo = orig->aabbMin, nodeHi = orig->aabbMax;
		const int32_t ex = CWBVHQuantExp( nodeHi.x - nodeLo.x );
		const int32_t ey = CWBVHQuantExp( nodeHi.y - nodeLo.y );
		const int32_t ez = CWBVHQuantExp( nodeHi.z - nodeLo.z );
		const uint32_t bx = (uint32_t)(127 - ex) << 23, by = (uint32_t)(127 - ey) << 23, bz = (uint32_t)(127 - ez) << 23;
		const float rsx = tinybvh_as_float( bx ), rsy = tinybvh_as_float( by ), rsz = tinybvh_as_float( bz ); // 2^-e, exact
		uint8_t* const qbase = (uint8_t*)&bvh8Data[currentNodeAddr + 2];
		uint8_t* const metaField = ((uint8_t*)&bvh8Data[currentNodeAddr + 1]) + 8;
		// greedy child node ordering, with the child bounds quantized into their
		// slots while they are still in registers.
		const uint32_t perm = SlotPermutation( bvh8, origIdx, nodeLo, rsx, rsy, rsz, qbase );
		uint32_t slotChild[8] = {};
		for (uint32_t i = 0; i < 8; i++)
		{
			const uint32_t c = (perm >> (i * 4)) & 15;
			if (c != 15) slotChild[i] = orig->child[c];
		}
		// every byte of the record is written below, so no clearing is needed.
		*(uint64_t*)metaField = 0;
		int32_t internalChildCount = 0, leafChildTriCount = 0, childBaseIndex = 0, triangleBaseIndex = 0;
		uint8_t imask = 0;
		for (int32_t i = 0; i < 8; i++)
		{
			if (slotChild[i] == 0) continue;
			const typename MBVH<8, Float, Index>::MBVHNode* const child = &bvh8.mbvhNode[slotChild[i]];
			slotsUsed++;
			if (!child->isLeaf())
			{
				// interior node, set params and push onto stack
				const uint32_t childNodeAddr = nodeDataPtr;
				if (internalChildCount++ == 0) childBaseIndex = childNodeAddr / 5;
				nodeDataPtr += 5, imask |= 1 << i;
				metaField[i] = (uint8_t)((1 << 5) | (24 + i));
				if (stackPtr == stackCap)
				{
					const uint32_t newCap = stackCap * 2;
					uint32_t* ni = (uint32_t*)AlignedAlloc( newCap * 4 );
					uint32_t* na = (uint32_t*)AlignedAlloc( newCap * 4 );
					memcpy( ni, stackNodeIdx, stackCap * 4 ), memcpy( na, stackNodeAddr, stackCap * 4 );
					AlignedFree( stackNodeIdx ), AlignedFree( stackNodeAddr );
					stackNodeIdx = ni, stackNodeAddr = na, stackCap = newCap;
				}
				stackNodeIdx[stackPtr] = slotChild[i], stackNodeAddr[stackPtr++] = childNodeAddr;
				continue;
			}
			// leaf node
			const uint32_t tcount = child->triCount;
			BVH_FATAL_ERROR_IF( tcount == 0 || tcount > 3,
				"BVH8_CWBVH::ConvertFrom( .. ), leaf with unsupported triangle count; call BVH::SplitLeafs( 3 ) first." );
			if (leafChildTriCount == 0) triangleBaseIndex = triDataPtr;
			const int32_t unaryEncodedTriCount = tcount == 1 ? 0b001 : tcount == 2 ? 0b011 : 0b111;
			metaField[i] = (uint8_t)((unaryEncodedTriCount << 5) | leafChildTriCount);
			leafChildTriCount += tcount;
			leafPrims += tcount, leafBlocks++;
			for (uint32_t j = 0; j < tcount; j++)
				triPrim[nTris + j] = (uint32_t)bvh8.bvh.primIdx[child->firstTri + j];
		#ifdef CWBVH_COMPRESSED_TRIS
			nTris += tcount, triDataPtr += tcount * 4;
		#else
			nTris += tcount, triDataPtr += tcount * 3;
		#endif
		}
		nodeBlocks++;
		const uint8_t exyzAndimask[4] = { (uint8_t)(ex + 127), (uint8_t)(ey + 127), (uint8_t)(ez + 127), imask };
		bvh8Data[currentNodeAddr + 0] = bvhvec4( nodeLo, tinybvh_bitcast<float>( exyzAndimask ) );
		bvh8Data[currentNodeAddr + 1].x = tinybvh_as_float( childBaseIndex );
		bvh8Data[currentNodeAddr + 1].y = tinybvh_as_float( triangleBaseIndex );
	}
	AlignedFree( stackNodeIdx ), AlignedFree( stackNodeAddr );
	// Triangle data, in output order. Pulled out of the walk: there the vertex
	// fetch sits behind a second scattered load and has nothing to overlap with,
	// here it is one flat loop that keeps many misses in flight.
	{
		const int8_t* const vbase = bvh8.bvh.verts.data;
		const size_t vstride = bvh8.bvh.verts.stride;
		bvhvec4* const out = bvh8Tris;
		for (uint32_t k = 0; k < nTris; k++)
		{
			const uint32_t pf = k + 64 < nTris ? k + 64 : k;
			TINYBVH_PREFETCH( vbase + (size_t)triPrim[pf] * 3 * vstride );
			const uint32_t primIdx = triPrim[k];
			Index i0, i1, i2;
			GET_PRIM_INDICES_I0_I1_I2( bvh8.bvh, primIdx );
		#ifdef CWBVH_COMPRESSED_TRIS
			PrecomputeTriangle( bvh8.bvh.verts, i0, i1, i2, &out[k * 4] );
			out[k * 4 + 3] = bvhvec4( 0, 0, 0, tinybvh_as_float( primIdx ) );
		#else
			bvhvec4 v0, v1, v2;
			memcpy( &v0, vbase + (size_t)i0 * vstride, 16 );
			memcpy( &v1, vbase + (size_t)i1 * vstride, 16 );
			memcpy( &v2, vbase + (size_t)i2 * vstride, 16 );
			out[k * 3 + 0] = v2 - v0, out[k * 3 + 1] = v1 - v0;
			v0.w = tinybvh_as_float( primIdx );
			out[k * 3 + 2] = v0;
		#endif
		}
	}
	AlignedFree( triPrim );
	usedBlocks = nodeDataPtr;
	usedTriBlocks = triDataPtr;
	if (compact && allocatedBlocks > usedBlocks)
	{
		bvhvec4* trimmed = (bvhvec4*)AlignedAlloc( (size_t)usedBlocks * 16 );
		memcpy( trimmed, bvh8Data, (size_t)usedBlocks * 16 );
		AlignedFree( bvh8Data );
		bvh8Data = trimmed, allocatedBlocks = usedBlocks;
	}
#ifdef CWBVH_REPORT_FULLNESS
	// The two fullness numbers Haydel et al. report for every build: how much of
	// each 8-slot node block and each 3-triangle leaf the collapse actually filled.
	printf( "CWBVH: %.2f children/node (max 8), %.2f prims/leaf (max 3), %.1f bytes/tri\n",
		nodeBlocks ? (float)slotsUsed / (float)nodeBlocks : 0.0f,
		leafBlocks ? (float)leafPrims / (float)leafBlocks : 0.0f,
		triCount ? (float)(usedBlocks + usedTriBlocks) * 16.0f / (float)triCount : 0.0f );
#else
	(void)slotsUsed, (void)nodeBlocks, (void)leafPrims, (void)leafBlocks;
#endif
}

// Intersect_CWBVH:
// Intersect a compressed 8-wide BVH with a ray. For debugging only, not efficient.
// This is here to test data before it goes to the GPU.
#define STACK_POP() { ngroup = traversalStack[--stackPtr]; }
#define STACK_PUSH() { traversalStack[stackPtr++] = ngroup; }
TINYBVH_FORCEINLINE uint32_t extract_byte( const uint32_t i, const uint32_t n ) { return (i >> (n * 8)) & 0xFF; }
TINYBVH_FORCEINLINE uint32_t sign_extend_s8x4( const uint32_t i )
{
	// asm("prmt.b32 %0, %1, 0x0, 0x0000BA98;" : "=r"(v) : "r"(i)); // BA98: 1011`1010`1001`1000
	// with the given parameters, prmt will extend the sign to all bits in a byte.
	uint32_t b0 = (i & 0b10000000000000000000000000000000) ? 0xff000000 : 0;
	uint32_t b1 = (i & 0b00000000100000000000000000000000) ? 0x00ff0000 : 0;
	uint32_t b2 = (i & 0b00000000000000001000000000000000) ? 0x0000ff00 : 0;
	uint32_t b3 = (i & 0b00000000000000000000000010000000) ? 0x000000ff : 0;
	return b0 + b1 + b2 + b3; // probably can do better than this.
}
TEMPLATED int32_t BVH8_CWBVH<Float, Index>::Intersect( Ray& ray ) const
{
	bvhuint2 traversalStack[TINYBVH_STACK_SIZE * 4 /* wide trees push more nodes per step */];
	uint32_t hitAddr = 0, stackPtr = 0;
	bvhvec2 triangleuv( 0, 0 );
	const bvhvec4* blasNodes = bvh8Data, * blasTris = bvh8Tris;
	float tmin = 0, tmax = ray.hit.t;
	const uint32_t octinv = (7 - ((ray.D.x < 0 ? 4 : 0) | (ray.D.y < 0 ? 2 : 0) | (ray.D.z < 0 ? 1 : 0))) * 0x1010101;
	bvhuint2 ngroup = bvhuint2( 0, 0b10000000000000000000000000000000 ), tgroup = bvhuint2( 0 );
	do
	{
		if (ngroup.y > 0x00FFFFFF)
		{
			const uint32_t hits = ngroup.y, imask = ngroup.y;
			const uint32_t child_bit_index = tinybvh_bfind( hits ), child_node_base_index = ngroup.x;
			ngroup.y &= ~(1 << child_bit_index);
			if (ngroup.y > 0x00FFFFFF) { STACK_PUSH( /* nodeGroup */ ); }
			{
				const uint32_t slot_index = (child_bit_index - 24) ^ (octinv & 255);
				const uint32_t relative_index = tinybvh_popc( imask & ~(0xFFFFFFFF << slot_index) );
				const uint32_t child_node_index = child_node_base_index + relative_index;
				const bvhvec4 n0 = blasNodes[child_node_index * 5 + 0], n1 = blasNodes[child_node_index * 5 + 1];
				const bvhvec4 n2 = blasNodes[child_node_index * 5 + 2], n3 = blasNodes[child_node_index * 5 + 3];
				const bvhvec4 n4 = blasNodes[child_node_index * 5 + 4], p = n0;
				// n0.w holds three 127-biased quantization exponents plus imask.
				const uint32_t e4 = as_uint( n0.w );
				ngroup.x = as_uint( n1.x ), tgroup.x = as_uint( n1.y ), tgroup.y = 0;
				uint32_t hitmask = 0;
				const uint32_t vx = (e4 & 255) << 23u; const float adjusted_idirx = tinybvh_as_float( vx ) * ray.rD.x;
				const uint32_t vy = ((e4 >> 8) & 255) << 23u; const float adjusted_idiry = tinybvh_as_float( vy ) * ray.rD.y;
				const uint32_t vz = ((e4 >> 16) & 255) << 23u; const float adjusted_idirz = tinybvh_as_float( vz ) * ray.rD.z;
				const float origx = -(ray.O.x - p.x) * ray.rD.x;
				const float origy = -(ray.O.y - p.y) * ray.rD.y;
				const float origz = -(ray.O.z - p.z) * ray.rD.z;
				{	// First 4
					const uint32_t meta4 = tinybvh_as_uint( n1.z ), is_inner4 = (meta4 & (meta4 << 1)) & 0x10101010;
					const uint32_t inner_mask4 = sign_extend_s8x4( is_inner4 << 3 );
					const uint32_t bit_index4 = (meta4 ^ (octinv & inner_mask4)) & 0x1F1F1F1F;
					const uint32_t child_bits4 = (meta4 >> 5) & 0x07070707;
					uint32_t swizzledLox = (ray.rD.x < 0) ? tinybvh_as_uint( n3.z ) : tinybvh_as_uint( n2.x ), swizzledHix = (ray.rD.x < 0) ? tinybvh_as_uint( n2.x ) : tinybvh_as_uint( n3.z );
					uint32_t swizzledLoy = (ray.rD.y < 0) ? tinybvh_as_uint( n4.x ) : tinybvh_as_uint( n2.z ), swizzledHiy = (ray.rD.y < 0) ? tinybvh_as_uint( n2.z ) : tinybvh_as_uint( n4.x );
					uint32_t swizzledLoz = (ray.rD.z < 0) ? tinybvh_as_uint( n4.z ) : tinybvh_as_uint( n3.x ), swizzledHiz = (ray.rD.z < 0) ? tinybvh_as_uint( n3.x ) : tinybvh_as_uint( n4.z );
					float tminx[4], tminy[4], tminz[4], tmaxx[4], tmaxy[4], tmaxz[4];
					tminx[0] = ((swizzledLox >> 0) & 0xFF) * adjusted_idirx + origx, tminx[1] = ((swizzledLox >> 8) & 0xFF) * adjusted_idirx + origx, tminx[2] = ((swizzledLox >> 16) & 0xFF) * adjusted_idirx + origx;
					tminx[3] = ((swizzledLox >> 24) & 0xFF) * adjusted_idirx + origx, tminy[0] = ((swizzledLoy >> 0) & 0xFF) * adjusted_idiry + origy, tminy[1] = ((swizzledLoy >> 8) & 0xFF) * adjusted_idiry + origy;
					tminy[2] = ((swizzledLoy >> 16) & 0xFF) * adjusted_idiry + origy, tminy[3] = ((swizzledLoy >> 24) & 0xFF) * adjusted_idiry + origy, tminz[0] = ((swizzledLoz >> 0) & 0xFF) * adjusted_idirz + origz;
					tminz[1] = ((swizzledLoz >> 8) & 0xFF) * adjusted_idirz + origz, tminz[2] = ((swizzledLoz >> 16) & 0xFF) * adjusted_idirz + origz, tminz[3] = ((swizzledLoz >> 24) & 0xFF) * adjusted_idirz + origz;
					tmaxx[0] = ((swizzledHix >> 0) & 0xFF) * adjusted_idirx + origx, tmaxx[1] = ((swizzledHix >> 8) & 0xFF) * adjusted_idirx + origx, tmaxx[2] = ((swizzledHix >> 16) & 0xFF) * adjusted_idirx + origx;
					tmaxx[3] = ((swizzledHix >> 24) & 0xFF) * adjusted_idirx + origx, tmaxy[0] = ((swizzledHiy >> 0) & 0xFF) * adjusted_idiry + origy, tmaxy[1] = ((swizzledHiy >> 8) & 0xFF) * adjusted_idiry + origy;
					tmaxy[2] = ((swizzledHiy >> 16) & 0xFF) * adjusted_idiry + origy, tmaxy[3] = ((swizzledHiy >> 24) & 0xFF) * adjusted_idiry + origy, tmaxz[0] = ((swizzledHiz >> 0) & 0xFF) * adjusted_idirz + origz;
					tmaxz[1] = ((swizzledHiz >> 8) & 0xFF) * adjusted_idirz + origz, tmaxz[2] = ((swizzledHiz >> 16) & 0xFF) * adjusted_idirz + origz, tmaxz[3] = ((swizzledHiz >> 24) & 0xFF) * adjusted_idirz + origz;
					for (int32_t i = 0; i < 4; i++)
					{
						// Use VMIN, VMAX to compute the slabs
						const float cmin = tinybvh_max( tinybvh_max( tinybvh_max( tminx[i], tminy[i] ), tminz[i] ), tmin );
						const float cmax = tinybvh_min( tinybvh_min( tinybvh_min( tmaxx[i], tmaxy[i] ), tmaxz[i] ), tmax );
						if (cmin <= cmax) hitmask |= extract_byte( child_bits4, i ) << extract_byte( bit_index4, i );
					}
				}
				{	// Second 4
					const uint32_t meta4 = tinybvh_as_uint( n1.w ), is_inner4 = (meta4 & (meta4 << 1)) & 0x10101010;
					const uint32_t inner_mask4 = sign_extend_s8x4( is_inner4 << 3 );
					const uint32_t bit_index4 = (meta4 ^ (octinv & inner_mask4)) & 0x1F1F1F1F;
					const uint32_t child_bits4 = (meta4 >> 5) & 0x07070707;
					uint32_t swizzledLox = (ray.rD.x < 0) ? tinybvh_as_uint( n3.w ) : tinybvh_as_uint( n2.y ), swizzledHix = (ray.rD.x < 0) ? tinybvh_as_uint( n2.y ) : tinybvh_as_uint( n3.w );
					uint32_t swizzledLoy = (ray.rD.y < 0) ? tinybvh_as_uint( n4.y ) : tinybvh_as_uint( n2.w ), swizzledHiy = (ray.rD.y < 0) ? tinybvh_as_uint( n2.w ) : tinybvh_as_uint( n4.y );
					uint32_t swizzledLoz = (ray.rD.z < 0) ? tinybvh_as_uint( n4.w ) : tinybvh_as_uint( n3.y ), swizzledHiz = (ray.rD.z < 0) ? tinybvh_as_uint( n3.y ) : tinybvh_as_uint( n4.w );
					float tminx[4], tminy[4], tminz[4], tmaxx[4], tmaxy[4], tmaxz[4];
					tminx[0] = ((swizzledLox >> 0) & 0xFF) * adjusted_idirx + origx, tminx[1] = ((swizzledLox >> 8) & 0xFF) * adjusted_idirx + origx, tminx[2] = ((swizzledLox >> 16) & 0xFF) * adjusted_idirx + origx;
					tminx[3] = ((swizzledLox >> 24) & 0xFF) * adjusted_idirx + origx, tminy[0] = ((swizzledLoy >> 0) & 0xFF) * adjusted_idiry + origy, tminy[1] = ((swizzledLoy >> 8) & 0xFF) * adjusted_idiry + origy;
					tminy[2] = ((swizzledLoy >> 16) & 0xFF) * adjusted_idiry + origy, tminy[3] = ((swizzledLoy >> 24) & 0xFF) * adjusted_idiry + origy, tminz[0] = ((swizzledLoz >> 0) & 0xFF) * adjusted_idirz + origz;
					tminz[1] = ((swizzledLoz >> 8) & 0xFF) * adjusted_idirz + origz, tminz[2] = ((swizzledLoz >> 16) & 0xFF) * adjusted_idirz + origz, tminz[3] = ((swizzledLoz >> 24) & 0xFF) * adjusted_idirz + origz;
					tmaxx[0] = ((swizzledHix >> 0) & 0xFF) * adjusted_idirx + origx, tmaxx[1] = ((swizzledHix >> 8) & 0xFF) * adjusted_idirx + origx, tmaxx[2] = ((swizzledHix >> 16) & 0xFF) * adjusted_idirx + origx;
					tmaxx[3] = ((swizzledHix >> 24) & 0xFF) * adjusted_idirx + origx, tmaxy[0] = ((swizzledHiy >> 0) & 0xFF) * adjusted_idiry + origy, tmaxy[1] = ((swizzledHiy >> 8) & 0xFF) * adjusted_idiry + origy;
					tmaxy[2] = ((swizzledHiy >> 16) & 0xFF) * adjusted_idiry + origy, tmaxy[3] = ((swizzledHiy >> 24) & 0xFF) * adjusted_idiry + origy, tmaxz[0] = ((swizzledHiz >> 0) & 0xFF) * adjusted_idirz + origz;
					tmaxz[1] = ((swizzledHiz >> 8) & 0xFF) * adjusted_idirz + origz, tmaxz[2] = ((swizzledHiz >> 16) & 0xFF) * adjusted_idirz + origz, tmaxz[3] = ((swizzledHiz >> 24) & 0xFF) * adjusted_idirz + origz;
					for (int32_t i = 0; i < 4; i++)
					{
						const float cmin = tinybvh_max( tinybvh_max( tinybvh_max( tminx[i], tminy[i] ), tminz[i] ), tmin );
						const float cmax = tinybvh_min( tinybvh_min( tinybvh_min( tmaxx[i], tmaxy[i] ), tmaxz[i] ), tmax );
						if (cmin <= cmax) hitmask |= extract_byte( child_bits4, i ) << extract_byte( bit_index4, i );
					}
				}
				ngroup.y = (hitmask & 0xFF000000) | (as_uint( n0.w ) >> 24), tgroup.y = hitmask & 0x00FFFFFF;
			}
		}
		else tgroup = ngroup, ngroup = bvhuint2( 0 );
		while (tgroup.y != 0)
		{
		#ifdef CWBVH_COMPRESSED_TRIS
			// "Fast Ray-Triangle Intersections by Coordinate Transformation", Baldwin & Weber, 2016.
			const uint32_t triangleIndex = tinybvh_bfind( tgroup.y );
			const int32_t triAddr = tgroup.x + triangleIndex * 4;
			tgroup.y -= 1 << triangleIndex;
			const bvhvec4 T2 = blasTris[triAddr + 2];
			const float transD = T2.x * ray.D.x + T2.y * ray.D.y + T2.z * ray.D.z;
			if (transD == 0) continue; // ray parallel to the triangle plane
			const float transS = T2.x * ray.O.x + T2.y * ray.O.y + T2.z * ray.O.z + T2.w;
			const float d = -transS / transD;
			if (!(d > 0 && d < tmax)) continue; // also rejects NaN
			const bvhvec4 T0 = blasTris[triAddr + 0], T1 = blasTris[triAddr + 1];
			const Vec3 I = ray.O + d * ray.D;
			const float u = T0.x * I.x + T0.y * I.y + T0.z * I.z + T0.w;
			const float v = T1.x * I.x + T1.y * I.y + T1.z * I.z + T1.w;
			if (!(u >= 0 && v >= 0 && u + v <= 1)) continue;
			triangleuv = bvhvec2( u, v ), tmax = d;
			hitAddr = as_uint( blasTris[triAddr + 3].w );
		#else
			// Moeller-Trumbore intersection.
			uint32_t triangleIndex = tinybvh_bfind( tgroup.y );
			tgroup.y -= 1 << triangleIndex;
			int32_t triAddr = tgroup.x + triangleIndex * 3;
			const Vec3 e2 = Vec3( blasTris[triAddr + 0] ), e1 = Vec3( blasTris[triAddr + 1] );
			const Vec3 v0 = blasTris[triAddr + 2];
			MOLLER_TRUMBORE_TEST( tmax, continue );
			triangleuv = bvhvec2( u, v ), tmax = t;
			hitAddr = as_uint( blasTris[triAddr + 2].w );
		#endif
		}
		if (ngroup.y > 0x00FFFFFF) continue;
		if (stackPtr > 0) { STACK_POP( /* nodeGroup */ ); }
		else
		{
			ray.hit.t = tmax;
			if (tmax < bvh_far<Float>) ray.hit.u = triangleuv.x, ray.hit.v = triangleuv.y, ray.hit.prim = hitAddr;
			break;
		}
	} while (true);
	return 0;
}

// Static Load: a finished object from a cache file, converting a plain-BVH file where needed.
// ----------------------------------------------------------------------------

TEMPLATED BVH_GPU<Float, Index> BVH_GPU<Float, Index>::Load( const char* fileName, BVHContext ctx )
{
	BVH_GPU result( ctx );
	BVH src = BVH::Load( fileName, ctx );
	if (src.usedNodes == 0) return result;
	result.settings = src.settings; // as saved
	result.bvh = tinybvh_move( src );
	result.ConvertFrom( result.bvh, false );
	result.bvh = BVH( result.context ); // traversal runs on orderedVerts; the tree is spent
	return result;
}

TEMPLATED BVH4_GPU<Float, Index> BVH4_GPU<Float, Index>::Load( const char* fileName, BVHContext ctx )
{
	BVH4_GPU result( ctx );
	uint32_t header = 0;
	Index tris = 0;
	if (!tinybvh_peek_cache( fileName, header, tris )) return result;
	if (header == result.CacheHeader()) { result.Load( fileName, tris ); return result; }
	BVH src = BVH::Load( fileName, ctx );
	if (src.usedNodes == 0) return result;
	result.settings = src.settings; // as saved
	result.PassSettingsTo( result.bvh4 );
	result.bvh4.bvh = tinybvh_move( src );
	result.bvh4.ConvertFrom( result.bvh4.bvh );
	result.ConvertFrom( result.bvh4, true );
	result.bvh4 = MBVH<4, Float, Index>( result.context );
	return result;
}

TEMPLATED BVH4_AMD_HW<Float, Index> BVH4_AMD_HW<Float, Index>::Load( const char* fileName, BVHContext ctx )
{
	BVH4_AMD_HW result( ctx );
	uint32_t header = 0;
	Index tris = 0;
	if (!tinybvh_peek_cache( fileName, header, tris )) return result;
	if (header == result.CacheHeader()) { result.Load( fileName, tris ); return result; }
	BVH src = BVH::Load( fileName, ctx );
	if (src.usedNodes == 0) return result;
	result.settings = src.settings; // as saved
	result.PassSettingsTo( result.bvh4 );
	result.bvh4.bvh = tinybvh_move( src );
	result.bvh4.bvh.SplitLeafs( 1 );
	result.bvh4.leafPrimLimit = 0;
	result.bvh4.ConvertFrom( result.bvh4.bvh );
	result.ConvertFrom( result.bvh4 );
	result.bvh4 = MBVH<4, Float, Index>( result.context );
	return result;
}

TEMPLATED CWBVH8_AMD_HW<Float, Index> CWBVH8_AMD_HW<Float, Index>::Load( const char* fileName, BVHContext ctx )
{
	CWBVH8_AMD_HW result( ctx );
	uint32_t header = 0;
	Index tris = 0;
	if (!tinybvh_peek_cache( fileName, header, tris )) return result;
	if (header == result.CacheHeader()) { result.Load( fileName, tris ); return result; }
	BVH src = BVH::Load( fileName, ctx );
	if (src.usedNodes == 0) return result;
	result.settings = src.settings; // as saved
	result.PassSettingsTo( result.bvh8 );
	result.bvh8.bvh = tinybvh_move( src );
	result.bvh8.bvh.SplitLeafs( 2 ); // at most one triangle pair per leaf; Build also pairs edge-sharing triangles
	result.bvh8.leafPrimLimit = 2;
	result.bvh8.ConvertFrom( result.bvh8.bvh );
	result.ConvertFrom( result.bvh8 );
	result.bvh8 = MBVH<8, Float, Index>( result.context );
	return result;
}

TEMPLATED BVH8_CWBVH<Float, Index> BVH8_CWBVH<Float, Index>::Load( const char* fileName, BVHContext ctx )
{
	BVH8_CWBVH result( ctx );
	uint32_t header = 0;
	Index tris = 0;
	if (!tinybvh_peek_cache( fileName, header, tris )) return result;
	if (header == result.CacheHeader()) { result.Load( fileName, tris ); return result; }
	BVH src = BVH::Load( fileName, ctx );
	if (src.usedNodes == 0) return result;
	result.settings = src.settings; // as saved
	result.PassSettingsTo( result.bvh8 );
	result.bvh8.bvh = tinybvh_move( src );
	result.bvh8.bvh.SplitLeafs( 3 ); // a cwbvh leaf slot holds at most 3 prims; as in Build
	result.bvh8.leafPrimLimit = 3;
	result.bvh8.ConvertFrom( result.bvh8.bvh, true );
	result.ConvertFrom( result.bvh8, true );
	result.bvh8 = MBVH<8, Float, Index>( result.context );
	return result;
}

} // namespace impl

#undef SWAP
#undef STACK_POP
#undef STACK_PUSH

#ifdef BVH_USEAVX2

// SlotPermutation, AVX2.
template <> uint32_t impl::BVH8_CWBVH<float, uint32_t>::SlotPermutation( const impl::MBVH<8, float, uint32_t>& mbvh8, const uint32_t nodeIdx,
	const bvhvec3& nodeLo, const float rsx, const float rsy, const float rsz, uint8_t* const qbase ) const
{
	const MBVH<8, float, uint32_t>::MBVHNode* const orig = &mbvh8.mbvhNode[nodeIdx];
	// An absent child reads node 0; its lane is masked out of the assignment and
	// its slot is filled with an inverted box at the end.
	const __m256i childIdx = tinybvh_load8i( orig->child );
	__m128 lo[8], hi[8];
	for (uint32_t i = 0; i < 8; i++)
	{
		const __m256 v = tinybvh_load8( &mbvh8.mbvhNode[orig->child[i]] );	// min.xyz, firstTri, max.xyz, triCount
		lo[i] = _mm256_castps256_ps128( v ), hi[i] = _mm256_extractf128_ps( v, 1 );
	}
	_MM_TRANSPOSE4_PS( lo[0], lo[1], lo[2], lo[3] );
	_MM_TRANSPOSE4_PS( lo[4], lo[5], lo[6], lo[7] );
	_MM_TRANSPOSE4_PS( hi[0], hi[1], hi[2], hi[3] );
	_MM_TRANSPOSE4_PS( hi[4], hi[5], hi[6], hi[7] );
	const __m256 minx = _mm256_set_m128( lo[4], lo[0] ), miny = _mm256_set_m128( lo[5], lo[1] ), minz = _mm256_set_m128( lo[6], lo[2] );
	const __m256 maxx = _mm256_set_m128( hi[4], hi[0] ), maxy = _mm256_set_m128( hi[5], hi[1] ), maxz = _mm256_set_m128( hi[6], hi[2] );
	// centroid offsets, unhalved: a common factor of two does not change the order.
	const __m256 nodeSum = _mm256_add_ps( _mm256_broadcast_ps( (const __m128*) & orig->aabbMin ),
		_mm256_broadcast_ps( (const __m128*) & orig->aabbMax ) );
	const __m256 dx = _mm256_sub_ps( _mm256_add_ps( minx, maxx ), _mm256_permute_ps( nodeSum, 0x00 ) );
	const __m256 dy = _mm256_sub_ps( _mm256_add_ps( miny, maxy ), _mm256_permute_ps( nodeSum, 0x55 ) );
	const __m256 dz = _mm256_sub_ps( _mm256_add_ps( minz, maxz ), _mm256_permute_ps( nodeSum, 0xAA ) );
	// slot s negates x for bit 2, y for bit 1, z for bit 0; row[s ^ 7] is -row[s].
	const __m256 t0 = _mm256_add_ps( dx, dy ), t1 = _mm256_sub_ps( dx, dy ), SIGN = _mm256_set1_ps( -0.0f );
	__m256 row[8];
	row[0] = _mm256_add_ps( t0, dz ), row[1] = _mm256_sub_ps( t0, dz );
	row[2] = _mm256_add_ps( t1, dz ), row[3] = _mm256_sub_ps( t1, dz );
	row[4] = _mm256_xor_ps( row[3], SIGN ), row[5] = _mm256_xor_ps( row[2], SIGN );
	row[6] = _mm256_xor_ps( row[1], SIGN ), row[7] = _mm256_xor_ps( row[0], SIGN );
	const __m256i LOW3 = _mm256_set1_epi32( 7 ), FULL = _mm256_set1_epi32( -1 );
	__m256i key[8];
	for (uint32_t s = 0; s < 8; s++)
	{
		// float order -> uint32 order: flip the sign bit, or all bits if negative.
		const __m256i c = _mm256_castps_si256( row[s] );
		const __m256i m = _mm256_or_si256( _mm256_srai_epi32( c, 31 ), _mm256_set1_epi32( (int)0x80000000 ) );
		key[s] = _mm256_or_si256( _mm256_andnot_si256( LOW3, _mm256_xor_si256( c, m ) ), _mm256_set1_epi32( (int)s ) );
	}
	const __m256i lane = _mm256_setr_epi32( 0, 1, 2, 3, 4, 5, 6, 7 );
	__m256i busy = _mm256_cmpeq_epi32( childIdx, _mm256_setzero_si256() );	// -1 where there is no child
	const uint32_t children = tinybvh_popc( 255u & ~(uint32_t)_mm256_movemask_ps( _mm256_castsi256_ps( busy ) ) );
	uint32_t perm = 0xFFFFFFFF;
	for (uint32_t r = 0; r < children; r++)
	{
		const __m256i k0 = _mm256_min_epu32( _mm256_max_epu32( key[0], busy ), _mm256_max_epu32( key[1], busy ) );
		const __m256i k1 = _mm256_min_epu32( _mm256_max_epu32( key[2], busy ), _mm256_max_epu32( key[3], busy ) );
		const __m256i k2 = _mm256_min_epu32( _mm256_max_epu32( key[4], busy ), _mm256_max_epu32( key[5], busy ) );
		const __m256i k3 = _mm256_min_epu32( _mm256_max_epu32( key[6], busy ), _mm256_max_epu32( key[7], busy ) );
		const __m256i k = _mm256_min_epu32( _mm256_min_epu32( k0, k1 ), _mm256_min_epu32( k2, k3 ) );
		__m256i h = _mm256_min_epu32( k, _mm256_permute2x128_si256( k, k, 1 ) );
		h = _mm256_min_epu32( h, _mm256_shuffle_epi32( h, _MM_SHUFFLE( 1, 0, 3, 2 ) ) );
		h = _mm256_min_epu32( h, _mm256_shuffle_epi32( h, _MM_SHUFFLE( 2, 3, 0, 1 ) ) );
		const uint32_t hits = (uint32_t)_mm256_movemask_ps( _mm256_castsi256_ps( _mm256_cmpeq_epi32( k, h ) ) );
		const uint32_t child = tinybvh_ctz( hits );								// ties go to the lowest child,
		const uint32_t slot = TINYBVH_LANE0( _mm256_castsi256_si128( h ) ) & 7;	// and before that the lowest slot
		perm ^= (0xF ^ child) << (slot * 4);
		key[slot] = FULL;
		busy = _mm256_or_si256( busy, _mm256_cmpeq_epi32( lane, _mm256_set1_epi32( (int)child ) ) );
	}
	// quantization: gather the eight nibbles of 'perm' into lane indices, move the
	// bounds into slot order with one permute each, then scale and pack.
	const __m256i nib = _mm256_and_si256( _mm256_srlv_epi32( _mm256_set1_epi32( (int)perm ),
		_mm256_setr_epi32( 0, 4, 8, 12, 16, 20, 24, 28 ) ), _mm256_set1_epi32( 15 ) );
	const __m256i empty = _mm256_cmpeq_epi32( nib, _mm256_set1_epi32( 15 ) );
	const __m256i pick = _mm256_and_si256( nib, LOW3 );
	const __m256 vlo[3] = { _mm256_permutevar8x32_ps( minx, pick ), _mm256_permutevar8x32_ps( miny, pick ), _mm256_permutevar8x32_ps( minz, pick ) };
	const __m256 vhi[3] = { _mm256_permutevar8x32_ps( maxx, pick ), _mm256_permutevar8x32_ps( maxy, pick ), _mm256_permutevar8x32_ps( maxz, pick ) };
	const float base[3] = { nodeLo.x, nodeLo.y, nodeLo.z }, scale[3] = { rsx, rsy, rsz };
	const __m256i Z = _mm256_setzero_si256(), M255 = _mm256_set1_epi32( 255 );
	const __m256i PACK = _mm256_setr_epi8( 0, 4, 8, 12, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
		0, 4, 8, 12, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 );
	for (uint32_t a = 0; a < 3; a++)
	{
		const __m256 b = _mm256_set1_ps( base[a] ), sc = _mm256_set1_ps( scale[a] );
		__m256 qa = _mm256_mul_ps( _mm256_sub_ps( vlo[a], b ), sc );
		__m256 qb = _mm256_mul_ps( _mm256_sub_ps( vhi[a], b ), sc );
		qa = _mm256_round_ps( qa, _MM_FROUND_TO_NEG_INF | _MM_FROUND_NO_EXC );
		qb = _mm256_round_ps( qb, _MM_FROUND_TO_POS_INF | _MM_FROUND_NO_EXC );
		__m256i ia = _mm256_max_epi32( _mm256_min_epi32( _mm256_cvttps_epi32( qa ), M255 ), Z );
		__m256i ib = _mm256_max_epi32( _mm256_min_epi32( _mm256_cvttps_epi32( qb ), M255 ), Z );
		// an empty slot gets lo 255, hi 0: a box no ray can enter.
		ia = _mm256_blendv_epi8( ia, M255, empty ), ib = _mm256_blendv_epi8( ib, Z, empty );
		ia = _mm256_shuffle_epi8( ia, PACK ), ib = _mm256_shuffle_epi8( ib, PACK );
		const uint32_t al = (uint32_t)TINYBVH_LANE0( _mm256_castsi256_si128( ia ) );
		const uint32_t ah = (uint32_t)TINYBVH_LANE0( _mm256_extracti128_si256( ia, 1 ) );
		const uint32_t bl = (uint32_t)TINYBVH_LANE0( _mm256_castsi256_si128( ib ) );
		const uint32_t bh = (uint32_t)TINYBVH_LANE0( _mm256_extracti128_si256( ib, 1 ) );
		*(uint64_t*)(qbase + a * 8) = (uint64_t)al | ((uint64_t)ah << 32);
		*(uint64_t*)(qbase + 24 + a * 8) = (uint64_t)bl | ((uint64_t)bh << 32);
	}
	return perm;
}

#endif // BVH_USEAVX2

// Explicit instantiations; see tiny_bvh.h.
template class impl::BVH_GPU<float, uint32_t>;
template class impl::BVH4_GPU<float, uint32_t>;
template class impl::BVH4_AMD_HW<float, uint32_t>;
template class impl::CWBVH8_AMD_HW<float, uint32_t>;
template class impl::BVH6_INTEL_HW<float, uint32_t>;
template class impl::BVH8_CWBVH<float, uint32_t>;

} // namespace tinybvh

#ifdef _MSC_VER
#pragma warning ( pop )
#endif

#endif // TINY_BVH_GPU_H_IMPL
#endif // TINYBVH_IMPLEMENTATION
