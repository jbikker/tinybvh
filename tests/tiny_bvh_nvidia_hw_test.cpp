// Validation of BVH12_NVIDIA_HW: structural checks on the packed buffer, and a CPU traversal
// of that buffer compared against BVH::Intersect.
//
// Usage: tiny_bvh_nvidia_hw_test [scene] - default ./testdata/cryteksponza.bin.

#define TINYBVH_IMPLEMENTATION
#include "tiny_bvh.h"
#include <cstdio>
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

struct Stats { uint32_t complets = 0, children = 0, leaves = 0, blocks = 0, tris = 0, vertCount = 0, errors = 0; std::vector<uint32_t> triSeen; const bvhvec4* verts = 0; };

static void Walk( const BVH12_NVIDIA_HW& hw, const uint32_t line, const uint32_t parent, Stats& s, const uint32_t depth )
{
	const uint8_t* base = (const uint8_t*)hw.rtasData;
	const Header& h = *(const Header*)base;
	if (line < 1 || line > h.completLines || depth > 64) { s.errors++, printf( "complet out of range\n" ); return; }
	const Complet& n = *(const Complet*)(base + (size_t)line * 128);
	s.complets++, s.children += n.childCount;
	if ((int64_t)line + n.parent != parent) s.errors++, printf( "bad parent offset\n" );
	if (n.childCount > 12) { s.errors++; return; }
	for (uint32_t i = 0; i < n.childCount; i++)
	{
		const uint32_t meta = n.child[i].meta;
		if (n.innerMask & (1u << i)) { Walk( hw, (uint32_t)((int64_t)line + n.childBase + meta), line, s, depth + 1 ); continue; }
		s.leaves++;
		int64_t blk = (int64_t)line + n.leafBase + (meta & 255);
		uint32_t t = (meta >> 8) & 15, count = (meta >> 12) + 1;
		for (; count > 0; count--, t++)
		{
			if (blk <= h.completLines || blk >= hw.usedLines) { s.errors++, printf( "leaf out of range\n" ); break; }
			const TriBlock* tb = (const TriBlock*)(base + (size_t)blk * 128);
			if (t == BVH12_NVIDIA_HW::BlockTris( *tb )) tb++, blk++, t = 0;
			if (blk >= hw.usedLines || t >= BVH12_NVIDIA_HW::BlockTris( *tb )) { s.errors++, printf( "bad triangle range\n" ); break; }
			bvhvec3 v[3];
			uint32_t prim;
			BVH12_NVIDIA_HW::BlockTri( *tb, t, v[0], v[1], v[2], prim );
			if (prim >= s.triSeen.size()) { s.errors++; continue; }
			s.triSeen[prim]++;
			// lossless: the decoded vertices are the input vertices, bit for bit
			for (int k = 0; k < 3; k++) for (int a = 0; a < 3; a++)
				if (s.verts && memcmp( &v[k][a], &s.verts[prim * 3 + k][a], 4 )) { if (s.errors++ < 5) printf( "vertex mismatch, prim %u\n", prim ); }
		}
	}
}

// block fill, over the whole triangle-block region.
static void BlockStats( const BVH12_NVIDIA_HW& hw, Stats& s )
{
	const Header& h = *(const Header*)hw.rtasData;
	for (uint32_t l = 1 + h.completLines; l < hw.usedLines; l++)
	{
		const TriBlock& tb = *(const TriBlock*)((const uint8_t*)hw.rtasData + (size_t)l * 128);
		const uint32_t T = BVH12_NVIDIA_HW::BlockTris( tb ), V = BVH12_NVIDIA_HW::BlockVerts( tb );
		s.blocks++, s.tris += T, s.vertCount += V;
		if (T > BVH12_NVIDIA_HW::BLOCK_TRIS || V > BVH12_NVIDIA_HW::BLOCK_VERTS || (tb.bits[0] & 7) != 1) s.errors++, printf( "bad block\n" );
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
		if (a.hit.prim == b.hit.prim && a.hit.t == b.hit.t) continue;
		if (fabsf( a.hit.t - b.hit.t ) <= 1e-6f * diag) continue; // shared edge
		if (mismatches++ < 10) printf( "  [%s] ray %i: ref t=%f prim=%u, hw t=%f prim=%u\n", label, i, a.hit.t, a.hit.prim, b.hit.t, b.hit.prim );
	}
	printf( "%s: %i rays, %i hits, %i mismatches\n", label, N, hits, mismatches );
	return mismatches;
}

static int Check( const BVH12_NVIDIA_HW& hw, const uint32_t triCount, const char* label, const bvhvec4* verts )
{
	Stats s;
	s.triSeen.resize( triCount ), s.verts = verts;
	Walk( hw, 1, 1, s, 0 );
	BlockStats( hw, s );
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
	static_assert(sizeof( Header ) == 128, "header size");
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
		failures += Check( hw, triCount, label, scene.data() );
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
		printf( "TLAS: %u lines, %u complets, %u instance nodes, type %u\n", tlas.usedLines, h.completLines, h.triBlockLines, h.type );
		if (h.type != 1 || h.triBlockLines != instCount) failures++, printf( "TLAS: unexpected structure\n" );
		const bvhvec3 tmin = refTlas.bvhNode[0].aabbMin, text = refTlas.bvhNode[0].aabbMax - tmin;
		const float tol = 1e-6f * tinybvh_length( text );
		auto instOf = []( const auto& r ) -> uint32_t { if constexpr (impl::bvh_packed_inst<uint32_t>) return r.hit.prim >> INST_IDX_SHFT; else return r.hit.inst; };
		auto primOf = []( const auto& r ) -> uint32_t { if constexpr (impl::bvh_packed_inst<uint32_t>) return r.hit.prim & PRIM_IDX_MASK; else return r.hit.prim; };
		// a float traversal hit, verified in double precision: its distance, or -1 if not a hit.
		auto doubleHit = [&]( const bvhvec3& O, const bvhvec3& D, const Ray& r ) -> double {
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
			const double t = f * (e2[0] * q[0] + e2[1] * q[1] + e2[2] * q[2]), eps = 1e-5;
			return (u >= -eps && w >= -eps && u + w <= 1 + eps && t > 0) ? t : -1;
		};
		int mismatches = 0, hits = 0, instMismatches = 0, closer = 0;
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
			if (mismatches++ < 10) printf( "  [TLAS] ray %i: ref t=%f prim=%08x, hw t=%f prim=%08x\n", i, a.hit.t, a.hit.prim, b.hit.t, b.hit.prim );
		}
		printf( "TLAS: 100000 rays, %i hits, %i mismatches, %i instance mismatches, %i closer verified hits\n", hits, mismatches, instMismatches, closer );
		if (mismatches || instMismatches) failures++;
	}
	printf( failures ? "FAILED\n" : "all tests passed\n" );
	return failures ? 1 : 0;
}
