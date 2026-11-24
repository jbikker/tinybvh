// Validation of BVH12_NVIDIA_HW: structural checks on the packed buffer, and a CPU traversal
// of that buffer compared against BVH::Intersect.
//
// Usage: tiny_bvh_nvidia_hw_test [scene] - default ./testdata/cryteksponza.bin.

#define TINYBVH_IMPLEMENTATION
#include "tiny_bvh.h"
#include <cstdio>
#include <cstddef>
#include <cmath>
#include <vector>

using namespace tinybvh;

static uint32_t seed = 0x12345678;
static float Rand() { seed ^= seed << 13, seed ^= seed >> 17, seed ^= seed << 5; return (seed >> 8) * (1.0f / 16777216.0f); }

static bool LoadScene( const char* fileName, std::vector<bvhvec4>& v )
{
	FILE* f = fopen( fileName, "rb" );
	if (!f) return false;
	uint32_t n = 0;
	bool ok = fread( &n, 4, 1, f ) == 1 && n > 0;
	if (ok) v.resize( (size_t)n * 3 ), ok = fread( v.data(), 48, n, f ) == n;
	fclose( f );
	return ok;
}

using Header = BVH12_NVIDIA_HW::NVHeader;
using Complet = BVH12_NVIDIA_HW::NVComplet;
using TriBlock = BVH12_NVIDIA_HW::NVTriBlock;

struct Stats
{
	uint32_t complets = 0, children = 0, leaves = 0, blocks = 0, tris = 0, vertCount = 0, errors = 0, refs = 0;
	std::vector<uint32_t> triSeen;
	const bvhvec4* verts = 0;
	uint64_t cursorBlock = 0; uint32_t cursorIdx = 0; bool cursorSet = false; // blocks are consumed in preorder
	bool clipped = false; // spatial splits: leaf boxes hold clipped pieces
};

static uint32_t TrailerBits( const TriBlock& b, const uint32_t i, const uint32_t n ) // n <= 32, may cross a word
{
	const uint64_t lo = b.bits[i >> 5], hi = (i >> 5) + 1 < 32 ? b.bits[(i >> 5) + 1] : 0;
	return (uint32_t)(((lo | hi << 32) >> (i & 31)) & ((1ull << n) - 1));
}

// Walks the complets from the root, following the decoded rules: parent and slot back links, children at
// firstChild + j, leaves as a running (block, index) cursor that continues from complet to complet in preorder.
static void Walk( const BVH12_NVIDIA_HW& hw, const uint64_t offset, const uint64_t parent, const uint32_t slot, Stats& s, const uint32_t depth )
{
	const uint8_t* base = (const uint8_t*)hw.rtasData;
	const Header& h = *(const Header*)base;
	const uint64_t completEnd = h.firstComplet + (uint64_t)h.complets * 128;
	if (offset < h.firstComplet || offset >= completEnd || depth > 64) { s.errors++, printf( "complet out of range\n" ); return; }
	const Complet& n = *(const Complet*)(base + offset);
	s.complets++;
	if (parent == 0 ? (n.parent != 0 || (n.startSlot >> 28) != 15) : ((int64_t)offset + (int64_t)n.parent * 128 != (int64_t)parent || (n.startSlot >> 28) != slot))
		s.errors++, printf( "bad parent link\n" );
	uint64_t block = (uint64_t)((int64_t)offset + n.firstBlock);
	uint32_t idx = (n.startSlot >> 24) & 15;
	bool hasLeaf = false, empty = false;
	// the complet's own leaves first, then its child complets: the block stream's order
	for (uint32_t i = 0; i < 12; i++)
	{
		const auto& c = n.child[i];
		if (!c.used) { empty = true; if (c.q[0] | c.q[1] | c.q[2] | c.q[3] | c.q[5] | c.leaf || c.q[4] != 0xff) s.errors++, printf( "bad empty slot\n" ); continue; }
		if (empty) { s.errors++, printf( "used slot after an empty one\n" ); continue; }
		s.children++;
		if (c.leaf == 0) continue;
		s.leaves++, hasLeaf = true;
		if (!s.cursorSet) s.cursorSet = true; else if (block != s.cursorBlock || idx != s.cursorIdx) s.errors++, printf( "leaf does not continue the block stream\n" );
		const uint64_t endBlock = block + (uint64_t)(c.leaf & 7) * 128;
		const uint32_t endIdx = (c.leaf >> 3) & 15;
		bvhvec3 bmin, bmax;
		BVH12_NVIDIA_HW::ChildBounds( n, i, bmin, bmax );
		uint32_t count = 0;
		while (block <= endBlock && (endIdx == 15 || block != endBlock || idx != endIdx))
		{
			if (block < h.firstBlock || block >= h.size) { s.errors++, printf( "leaf out of range\n" ); break; }
			const TriBlock& tb = *(const TriBlock*)(base + block);
			if (idx >= BVH12_NVIDIA_HW::BlockTris( tb )) { block += 128, idx = 0; continue; }
			bvhvec3 v[3];
			uint32_t prim;
			BVH12_NVIDIA_HW::BlockTri( tb, idx++, v[0], v[1], v[2], prim );
			count++, s.refs++;
			if (prim >= s.triSeen.size()) { s.errors++; continue; }
			s.triSeen[prim]++;
			// lossless: the decoded vertices are the input vertices, bit for bit; the leaf box holds them
			for (int k = 0; k < 3; k++) for (int a = 0; a < 3; a++)
			{
				if (s.verts && memcmp( &v[k][a], &s.verts[prim * 3 + k][a], 4 )) { if (s.errors++ < 5) printf( "vertex mismatch, prim %u\n", prim ); }
				if (!s.clipped && (v[k][a] < bmin[a] || v[k][a] > bmax[a])) { if (s.errors++ < 5) printf( "vertex outside its leaf box, prim %u\n", prim ); }
			}
		}
		if (count == 0 || count > 16) s.errors++, printf( "leaf with %u triangles\n", count );
		s.cursorBlock = block, s.cursorIdx = idx;
	}
	if ((n.flags != 0x40 && n.flags != 0x60) || ((n.flags & 0x20) != 0) != hasLeaf) s.errors++, printf( "bad complet flags\n" );
	if (!hasLeaf && n.firstBlock != 0) s.errors++, printf( "first block set without leaves\n" );
	for (uint32_t i = 0, inner = 0; i < 12 && n.child[i].used; i++) if (n.child[i].leaf == 0)
		Walk( hw, (uint64_t)((int64_t)offset + ((int64_t)n.firstChild + inner++) * 128), offset, i, s, depth + 1 );
}

// Every block: the driver's rules (top bits 001, at most 16 triangles and 16 vertices, vertices numbered in
// first-use order, triangle 0 = 0, 1, 2, free bits between vertices and records zero, used bits <= 1024).
static void BlockStats( const BVH12_NVIDIA_HW& hw, Stats& s )
{
	const Header& h = *(const Header*)hw.rtasData;
	for (uint32_t l = 0; l < h.blockLines; l++)
	{
		const TriBlock& tb = *(const TriBlock*)((const uint8_t*)hw.rtasData + h.firstBlock + (size_t)l * 128);
		const uint32_t t = tb.bits[31], T = BVH12_NVIDIA_HW::BlockTris( tb ), V = BVH12_NVIDIA_HW::BlockVerts( tb );
		const uint32_t w = (t & 31) + ((t >> 5) & 31) + ((t >> 10) & 31) + 3, pw = ((t >> 15) & 31) + 1;
		const uint32_t vertEnd = 96 + (V - 1) * w, recEnd = 958 - (T - 1) * (14 + pw);
		s.blocks++, s.tris += T, s.vertCount += V;
		bool ok = (t >> 29) == 1 && V <= BVH12_NVIDIA_HW::BLOCK_VERTS && vertEnd <= recEnd;
		uint32_t next = 3;
		for (uint32_t k = 1; k < T && ok; k++) for (uint32_t c = 0; c < 3; c++)
		{
			const uint32_t i = TrailerBits( tb, 958 - 14 * k + 2 + 4 * c, 4 );
			if (i > next) ok = false; else if (i == next) next++;
		}
		for (uint32_t b = vertEnd; b < recEnd && ok; b++) if (TrailerBits( tb, b, 1 )) ok = false;
		if (!ok && s.errors++ < 5) printf( "bad block %u\n", l );
	}
}

static int Compare( const BVH& ref, const BVH12_NVIDIA_HW& hw, const bvhvec3& bmin, const bvhvec3& ext, const char* label )
{
	int mismatches = 0, hits = 0;
	const int N = 200000;
	const float diag = tinybvh_length( ext );
	for (int i = 0; i < N; i++)
	{
		const bvhvec3 O = bmin + bvhvec3( Rand() * ext.x, Rand() * ext.y, Rand() * ext.z );
		bvhvec3 D = tinybvh_normalize( bvhvec3( Rand() - 0.5f, Rand() - 0.5f, Rand() - 0.5f ) );
		if (i % 7 == 0) D = bvhvec3( (float)(i % 3 == 0), (float)(i % 3 == 1), (float)(i % 3 == 2) ); // axis-aligned rays
		Ray a( O, D ), b( O, D );
		ref.Intersect( a );
		hw.Intersect( b );
		if (a.hit.t < BVH_FAR) hits++;
		if (a.hit.t >= BVH_FAR && b.hit.t >= BVH_FAR) continue;
		if (a.hit.prim == b.hit.prim && fabsf( a.hit.t - b.hit.t ) <= 1e-3f * a.hit.t) continue; // same triangle: rounding (grazing rays, fast math)
		if (fabsf( a.hit.t - b.hit.t ) <= 1e-6f * diag) continue; // shared edge
		if (mismatches++ < 10) printf( "  [%s] ray %i: ref t=%f prim=%u, hw t=%f prim=%u\n", label, i, a.hit.t, a.hit.prim, b.hit.t, b.hit.prim );
	}
	printf( "%s: %i rays, %i hits, %i mismatches\n", label, N, hits, mismatches );
	return mismatches;
}

static int Check( const BVH12_NVIDIA_HW& hw, const uint32_t triCount, const char* label, const bvhvec4* verts, const bool clipped = false )
{
	Stats s;
	s.triSeen.resize( triCount ), s.verts = verts, s.clipped = clipped;
	const Header& h = *(const Header*)hw.rtasData;
	if (h.word0 != 0x00110003 || h.size != hw.usedLines * 128 || h.address != (uint64_t)(uintptr_t)hw.rtasData || h.prims != triCount ||
		h.firstComplet != BVH12_NVIDIA_HW::FIRST_COMPLET * 128 || h.firstBlock != h.firstComplet + h.complets * 128 || h.firstBlock + h.blockLines * 128 != h.size)
		s.errors++, printf( "bad header\n" );
	Walk( hw, h.firstComplet, 0, 15, s, 0 );
	BlockStats( hw, s );
	if (s.complets != h.complets || s.refs != h.refs) s.errors++, printf( "header counts differ: %u/%u complets, %u/%u references\n", s.complets, h.complets, s.refs, h.refs );
	uint32_t missing = 0;
	for (uint32_t t = 0; t < triCount; t++) missing += s.triSeen[t] == 0;
	printf( "%s: %u lines (%.2f MB, %.1f B/tri), %u complets (%.2f children avg), %u leaves, %u blocks (%.2f tris, %.2f verts avg), %u missing, %u errors\n",
		label, hw.usedLines, hw.usedLines * 128.0 / (1 << 20), hw.usedLines * 128.0 / triCount, s.complets, (double)s.children / s.complets,
		s.leaves, s.blocks, (double)s.tris / s.blocks, (double)s.vertCount / s.blocks, missing, s.errors );
	return (s.errors || missing) ? 1 : 0;
}

int main( int argc, char** argv )
{
	const char* fileName = argc > 1 ? argv[1] : "./testdata/cryteksponza.bin";
	std::vector<bvhvec4> scene;
	if (!LoadScene( fileName, scene )) { printf( "could not load %s\n", fileName ); return 1; }
	const uint32_t triCount = (uint32_t)scene.size() / 3;
	printf( "scene: %s, %u triangles\n", fileName, triCount );
	static_assert(sizeof( Header ) == 384 && offsetof( Header, address ) == 0xd0 && offsetof( Header, moments ) == 0x150, "header layout");
	static_assert(sizeof( BVH12_NVIDIA_HW::NVDirTable ) == 512, "direction table size");
	static_assert(sizeof( Complet ) == 128, "complet size");
	static_assert(sizeof( TriBlock ) == 128, "triangle block size");
	BVH ref;
	ref.Build( scene.data(), triCount );
	const bvhvec3 bmin = ref.bvhNode[0].aabbMin, ext = ref.bvhNode[0].aabbMax - bmin;
	int failures = 0;
	for (int hq = 0; hq < 2; hq++)
	{
		BVH12_NVIDIA_HW hw;
		if (hq) hw.BuildHQ( scene.data(), triCount ); else hw.Build( scene.data(), triCount );
		const char* label = hq ? "BuildHQ" : "Build";
		failures += Check( hw, triCount, label, scene.data(), hq != 0 );
		failures += Compare( ref, hw, bmin, ext, label ) ? 1 : 0;
	}
	{
		BVH8_CWBVH cw;
		cw.Build( scene.data(), triCount );
		const double bytes = (cw.usedBlocks + cw.usedTriBlocks) * 16.0;
		printf( "for reference, BVH8_CWBVH: %.2f MB (%.1f B/tri)\n", bytes / (1 << 20), bytes / triCount );
	}
	// tiny mesh: a single complet with one leaf.
	{
		BVH12_NVIDIA_HW hw;
		hw.Build( scene.data(), 3 );
		failures += Check( hw, 3, "Tiny", scene.data() );
		BVH tiny3;
		tiny3.Build( scene.data(), 3 );
		const bvhvec3 m = tiny3.bvhNode[0].aabbMin, e = tiny3.bvhNode[0].aabbMax - m;
		failures += Compare( tiny3, hw, m - e, e * 3, "Tiny" ) ? 1 : 0;
	}
	// TLAS: rotated, scaled and masked instances of two BLASses, against a BVH TLAS over BVH BLASses.
	{
		const uint32_t smallTris = 500;
		BVH12_NVIDIA_HW nvBlas[2];
		nvBlas[0].Build( scene.data(), triCount );
		nvBlas[1].Build( scene.data(), smallTris );
		BVH refBlas[2];
		refBlas[0].Build( scene.data(), triCount );
		refBlas[1].Build( scene.data(), smallTris );
		const uint32_t instCount = 40;
		std::vector<BLASInstance> nvInst( instCount ), refInst( instCount );
		for (uint32_t i = 0; i < instCount; i++)
		{
			BLASInstance& b = nvInst[i];
			b.blasIdx = i % 5 == 0 ? 0 : 1;
			const float a = Rand() * 6.2831853f, s = 0.05f + Rand() * (b.blasIdx ? 2.0f : 0.1f);
			const float c = cosf( a ) * s, d = sinf( a ) * s;
			const float T[16] = { c, 0, d, (Rand() - 0.5f) * 400, 0, s, 0, (Rand() - 0.5f) * 100, -d, 0, c, (Rand() - 0.5f) * 400, 0, 0, 0, 1 };
			for (int k = 0; k < 16; k++) b.transform[k] = T[k];
			if (i % 7 == 3) b.mask = 2; // skipped by rays with mask 1
			refInst[i] = b;
		}
		BVH12_NVIDIA_HW* nvList[2] = { &nvBlas[0], &nvBlas[1] };
		BVHBase* refList[2] = { &refBlas[0], &refBlas[1] };
		BVH12_NVIDIA_HW tlas;
		tlas.Build( nvInst.data(), instCount, nvList, 2 );
		BVH refTlas;
		refTlas.Build( refInst.data(), instCount, refList, 2 );
		const Header& h = *(const Header*)tlas.rtasData;
		printf( "TLAS: %u lines, %u complets, %u instance nodes, type %u\n", tlas.usedLines, h.complets, h.blockLines, h.tinybvhType );
		if (h.tinybvhType != 1 || h.blockLines != instCount) failures++, printf( "TLAS: unexpected structure\n" );
		const bvhvec3 tmin = refTlas.bvhNode[0].aabbMin, text = refTlas.bvhNode[0].aabbMax - tmin;
		const float tol = 1e-6f * tinybvh_length( text );
		auto instOf = []( const auto& r ) -> uint32_t { if constexpr (impl::bvh_packed_inst<uint32_t>) return r.hit.prim >> INST_IDX_SHFT; else return r.hit.inst; };
		auto primOf = []( const auto& r ) -> uint32_t { if constexpr (impl::bvh_packed_inst<uint32_t>) return r.hit.prim & PRIM_IDX_MASK; else return r.hit.prim; };
		// a float traversal hit, verified in double precision: its distance, or -1 if not a hit.
		// eps > 0 accepts hits just outside the triangle; eps < 0 rejects hits within -eps of an edge.
		// 'margin' receives the distance to the nearest edge in units of the expected float error of
		// Moeller-Trumbore, about FLT_EPSILON * |O - v0| * |e| / |det|: large for a far origin and a small triangle.
		auto doubleHit = [&]( const bvhvec3& O, const bvhvec3& D, const Ray& r, const double eps = 1e-5, double* margin = 0 ) -> double {
			const float* M = (const float*)&refInst[instOf( r )].invTransform;
			const bvhvec4* v = &scene[(size_t)primOf( r ) * 3];
			double o[3], d[3], e1[3], e2[3], s[3], h[3], q[3];
			for (int k = 0; k < 3; k++)
				o[k] = M[k * 4] * (double)O.x + M[k * 4 + 1] * (double)O.y + M[k * 4 + 2] * (double)O.z + M[k * 4 + 3],
				d[k] = M[k * 4] * (double)D.x + M[k * 4 + 1] * (double)D.y + M[k * 4 + 2] * (double)D.z,
				e1[k] = (double)v[1][k] - v[0][k], e2[k] = (double)v[2][k] - v[0][k], s[k] = o[k] - v[0][k];
			h[0] = d[1] * e2[2] - d[2] * e2[1], h[1] = d[2] * e2[0] - d[0] * e2[2], h[2] = d[0] * e2[1] - d[1] * e2[0];
			q[0] = s[1] * e1[2] - s[2] * e1[1], q[1] = s[2] * e1[0] - s[0] * e1[2], q[2] = s[0] * e1[1] - s[1] * e1[0];
			const double f = 1 / (e1[0] * h[0] + e1[1] * h[1] + e1[2] * h[2]);
			const double u = f * (s[0] * h[0] + s[1] * h[1] + s[2] * h[2]), w = f * (d[0] * q[0] + d[1] * q[1] + d[2] * q[2]);
			const double t = f * (e2[0] * q[0] + e2[1] * q[1] + e2[2] * q[2]);
			if (margin)
			{
				auto len = []( const double* x ) { return sqrt( x[0] * x[0] + x[1] * x[1] + x[2] * x[2] ); };
				const double err = 1.1920929e-7 * len( s ) * tinybvh_max( len( e1 ), len( e2 ) ) * fabs( f );
				*margin = tinybvh_min( tinybvh_min( u, w ), 1 - u - w ) / err;
			}
			return (u >= -eps && w >= -eps && u + w <= 1 + eps && t > 0) ? t : -1;
		};
		int mismatches = 0, hits = 0, instMismatches = 0, closer = 0, edge = 0;
		for (int i = 0; i < 100000; i++)
		{
			const bvhvec3 O = tmin + bvhvec3( Rand() * text.x, Rand() * text.y, Rand() * text.z );
			bvhvec3 D = tinybvh_normalize( bvhvec3( Rand() - 0.5f, Rand() - 0.5f, Rand() - 0.5f ) );
			if (i & 2) // aim at a random point in a random instance
			{
				const BLASInstance& t = refInst[(uint32_t)(Rand() * instCount) % instCount];
				const bvhvec3 e = t.aabbMax - t.aabbMin;
				D = tinybvh_normalize( t.aabbMin + bvhvec3( Rand() * e.x, Rand() * e.y, Rand() * e.z ) - O );
			}
			Ray a( O, D ), b( O, D );
			if (i & 1) a.mask = b.mask = 1;
			refTlas.Intersect( a );
			tlas.Intersect( b );
			if (a.hit.t < BVH_FAR) hits++;
			if (a.hit.t >= BVH_FAR && b.hit.t >= BVH_FAR) continue;
			if (fabsf( a.hit.t - b.hit.t ) <= tol + 1e-5f * a.hit.t)
			{
				// same hit: the instance must match too (packed in prim, or in hit.inst).
				bool sameInst = (a.hit.prim >> INST_IDX_SHFT) == (b.hit.prim >> INST_IDX_SHFT);
				if constexpr (!impl::bvh_packed_inst<uint32_t>) sameInst = a.hit.inst == b.hit.inst;
				if (!sameInst && instMismatches++ < 5) printf( "  [TLAS] ray %i: instance differs\n", i );
				continue;
			}
			// different hits from rounding in the instance transform: accept a closer hw hit that holds in double.
			if (b.hit.t < a.hit.t && (refInst[instOf( b )].mask & b.mask) && fabs( doubleHit( O, D, b ) - b.hit.t ) <= tol + 1e-5 * b.hit.t) { closer++; continue; }
			// a hw miss where the reference hit is within rounding of an edge: in the transform, or in the
			// float triangle test (which rounds differently per compiler and inlining context)
			double margin = 1e30;
			if (b.hit.t >= BVH_FAR && doubleHit( O, D, a, 1e-5, &margin ) > 0 && (doubleHit( O, D, a, -1e-5 ) < 0 || margin < 16)) { edge++; continue; }
			if (mismatches++ < 10) printf( "  [TLAS] ray %i: ref t=%f prim=%08x, hw t=%f prim=%08x\n", i, a.hit.t, a.hit.prim, b.hit.t, b.hit.prim );
		}
		printf( "TLAS: 100000 rays, %i hits, %i mismatches, %i instance mismatches, %i closer verified hits, %i edge misses\n", hits, mismatches, instMismatches, closer, edge );
		if (mismatches || instMismatches) failures++;
		// a TLAS holds BLAS addresses: Save refuses it
		if (tlas.Save( "nvtlas.bin" )) failures++, printf( "TLAS: Save should fail\n" );
		// opacity maps belong to the BLAS: odd primitives of the small BLAS transparent (1x1 maps)
		std::vector<uint32_t> om( smallTris );
		for (uint32_t p = 0; p < smallTris; p++) om[p] = (p & 1) ? 0 : 1;
		nvBlas[1].opmap = refBlas[1].opmap = om.data(), nvBlas[1].opmapN = refBlas[1].opmapN = 1;
		int seeThrough = 0, opaque = 0, refOpaque = 0;
		for (int i = 0; i < 20000; i++)
		{
			const BLASInstance& t = refInst[1 + (uint32_t)(Rand() * (instCount - 1)) % (instCount - 1)];
			const bvhvec3 O = tmin + bvhvec3( Rand() * text.x, Rand() * text.y, Rand() * text.z ), e = t.aabbMax - t.aabbMin;
			const bvhvec3 D = tinybvh_normalize( t.aabbMin + bvhvec3( Rand() * e.x, Rand() * e.y, Rand() * e.z ) - O );
			Ray a( O, D ), b( O, D );
			refTlas.Intersect( a ), tlas.Intersect( b );
			if (a.hit.t < BVH_FAR && refInst[instOf( a )].blasIdx == 1) refOpaque++;
			if (b.hit.t >= BVH_FAR || refInst[instOf( b )].blasIdx != 1) continue;
			if (primOf( b ) & 1) seeThrough++; else opaque++;
		}
		printf( "TLAS opmap: %i hits on opaque prims (reference %i), %i on transparent prims\n", opaque, refOpaque, seeThrough );
		if (seeThrough || opaque == 0 || abs( opaque - refOpaque ) > refOpaque / 50 + 2) failures++;
		nvBlas[1].opmap = refBlas[1].opmap = 0, nvBlas[1].opmapN = refBlas[1].opmapN = 0;
	}
	// Save / Load: a BLAS round trip traces like the original; an empty object can't be saved
	{
		BVH12_NVIDIA_HW hw, back, empty;
		hw.Build( scene.data(), triCount );
		const bool saved = hw.Save( "nvblas.bin" ), loaded = saved && back.Load( "nvblas.bin", triCount );
		int diff = 0;
		if (loaded) for (int i = 0; i < 20000; i++)
		{
			const bvhvec3 O = bmin + bvhvec3( Rand() * ext.x, Rand() * ext.y, Rand() * ext.z );
			const bvhvec3 D = tinybvh_normalize( bvhvec3( Rand() - 0.5f, Rand() - 0.5f, Rand() - 0.5f ) );
			Ray a( O, D ), b( O, D );
			hw.Intersect( a ), back.Intersect( b );
			if (a.hit.t != b.hit.t || a.hit.prim != b.hit.prim) diff++;
		}
		const bool addressOk = loaded && ((const Header*)back.rtasData)->address == (uint64_t)(uintptr_t)back.rtasData;
		const bool emptyRefused = !empty.Save( "nvempty.bin" ) && !back.Load( "nvmissing.bin", triCount );
		printf( "Save/Load: saved %i, loaded %i, %i differing rays, address %s, empty refused %i\n", (int)saved, (int)loaded, diff, addressOk ? "ok" : "stale", (int)emptyRefused );
		if (!loaded || diff || !addressOk || !emptyRefused) failures++;
		remove( "nvblas.bin" );
	}
	printf( failures ? "FAILED\n" : "all tests passed\n" );
	return failures ? 1 : 0;
}
