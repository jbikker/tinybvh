// Validation of BVH6_INTEL_HW: structural checks on the packed QBVH6 buffer, and a CPU traversal
// of that buffer compared against BVH::Intersect.
//
// Usage: tiny_bvh_intel_hw_test [scene] - default ./testdata/cryteksponza.bin.

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

using Node6 = BVH6_INTEL_HW::IntelNode6;
using Quad = BVH6_INTEL_HW::IntelQuadLeaf;

struct Stats { uint32_t nodes = 0, fatLeaves = 0, quads = 0, pairs = 0, errors = 0; std::vector<uint32_t> triSeen; };

// walk the buffer as the hardware would, checking that every child box contains its content.
static void Walk( const BVH6_INTEL_HW& hw, const uint32_t block, Stats& s, const uint32_t depth )
{
	const uint8_t* base = (const uint8_t*)hw.rtasData;
	const Node6& n = *(const Node6*)(base + (size_t)block * 64);
	s.nodes++;
	if (block >= hw.usedBlocks) { s.errors++; return; }
	int64_t cb = (int64_t)block + n.childOffset;
	for (int i = 0; i < 6; cb += n.childData[i++] & 3)
	{
		const bool valid = !(n.q[0][i] & 0x80) || (n.q[1][i] & 0x80);
		if (!valid) continue;
		if (cb < 3 || cb >= hw.usedBlocks) { s.errors++, printf( "child out of range\n" ); continue; }
		if (n.nodeType == 0)
		{
			if (depth > 64) { s.errors++; return; }
			Walk( hw, (uint32_t)cb, s, depth + 1 );
			continue;
		}
		if (n.nodeType != 4) { s.errors++, printf( "unexpected node type %i\n", n.nodeType ); continue; }
		if (i == 0) s.fatLeaves++;
		for (const Quad* q = (const Quad*)(base + (size_t)cb * 64);; q++)
		{
			s.quads++;
			const uint32_t info = q->pairInfo, j0 = (info >> 16) & 3, j1 = (info >> 18) & 3, j2 = (info >> 20) & 3;
			const bool paired = j0 | j1 | j2;
			if ((info & 31) == 0) s.errors++, printf( "quad UV mode set\n" );
			if (paired) s.pairs++;
			const uint32_t pc = paired ? 2 : 1;
			for (uint32_t t = 0; t < pc; t++)
			{
				const uint32_t prim = q->primIndex0 + t * (info & 31);
				if (prim < s.triSeen.size()) s.triSeen[prim]++; else s.errors++;
			}
			if (info & (1u << 22)) break;
			if ((const uint8_t*)(q + 1) >= base + (size_t)hw.usedBlocks * 64) { s.errors++; break; }
		}
	}
}

static int Compare( const BVH& ref, const BVH6_INTEL_HW& hw, const bvhvec3& bmin, const bvhvec3& ext, const char* label )
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
		const bool bothMiss = a.hit.t >= BVH_FAR && b.hit.t >= BVH_FAR;
		if (bothMiss) continue;
		// same triangle: fast-math may evaluate it slightly differently for grazing rays
		if (a.hit.prim == b.hit.prim && fabsf( a.hit.t - b.hit.t ) <= 1e-3f * (1 + a.hit.t)) continue;
		// equal distance on a shared edge may pick another triangle
		if (fabsf( a.hit.t - b.hit.t ) <= 1e-6f * diag) continue;
		if (mismatches++ < 10) printf( "  [%s] ray %i: ref t=%f prim=%u, hw t=%f prim=%u\n", label, i, a.hit.t, a.hit.prim, b.hit.t, b.hit.prim );
	}
	printf( "%s: %i rays, %i hits, %i mismatches\n", label, N, hits, mismatches );
	return mismatches;
}

int main( int argc, char** argv )
{
	const char* fileName = argc > 1 ? argv[1] : "./testdata/cryteksponza.bin";
	std::vector<bvhvec4> scene;
	if (!LoadScene( fileName, scene )) { printf( "could not load %s\n", fileName ); return 1; }
	const uint32_t triCount = (uint32_t)scene.size() / 3;
	printf( "scene: %s, %u triangles\n", fileName, triCount );
	static_assert(sizeof( BVH6_INTEL_HW::IntelHeader ) == 128, "header size");
	static_assert(sizeof( Node6 ) == 64, "node size");
	static_assert(sizeof( Quad ) == 64, "quad size");
	BVH ref;
	ref.Build( scene.data(), triCount );
	const bvhvec3 bmin = ref.bvhNode[0].aabbMin, ext = ref.bvhNode[0].aabbMax - bmin;
	int failures = 0;
	for (int hq = 0; hq < 2; hq++)
	{
		BVH6_INTEL_HW hw;
		if (hq) hw.BuildHQ( scene.data(), triCount ); else hw.Build( scene.data(), triCount );
		const char* label = hq ? "BuildHQ" : "Build";
		Stats s;
		s.triSeen.resize( triCount );
		Walk( hw, 2, s, 0 );
		uint32_t missing = 0;
		for (uint32_t t = 0; t < triCount; t++) missing += s.triSeen[t] == 0;
		printf( "%s: %u blocks (%.1f MB), %u nodes, %u fat leaves, %u quads (%u paired, %.1f%%), %u missing tris, %u errors\n",
			label, hw.usedBlocks, hw.usedBlocks * 64.0 / (1 << 20), s.nodes, s.fatLeaves, s.quads, s.pairs,
			100.0 * s.pairs / (s.quads ? s.quads : 1), missing, s.errors );
		if (s.errors || missing) failures++;
		if (Compare( ref, hw, bmin, ext, label )) failures++;
	}
	// indexed mesh: weld identical vertices, so pairs are found through shared indices.
	{
		std::vector<bvhvec4> verts;
		std::vector<uint32_t> idx( triCount * 3 );
		for (uint32_t i = 0; i < triCount * 3; i++)
		{
			// cheap weld: only against the previous triangle's vertices.
			uint32_t found = UINT32_MAX;
			for (uint32_t k = (i / 3) * 3 >= 3 ? (i / 3) * 3 - 3 : 0; k < i && found == UINT32_MAX; k++)
				if (scene[k].x == scene[i].x && scene[k].y == scene[i].y && scene[k].z == scene[i].z) found = idx[k];
			if (found == UINT32_MAX) found = (uint32_t)verts.size(), verts.push_back( scene[i] );
			idx[i] = found;
		}
		BVH6_INTEL_HW hw;
		hw.Build( verts.data(), idx.data(), triCount );
		Stats s;
		s.triSeen.resize( triCount );
		Walk( hw, 2, s, 0 );
		printf( "Indexed: %u quads (%u paired), %u errors\n", s.quads, s.pairs, s.errors );
		if (s.errors) failures++;
		if (Compare( ref, hw, bmin, ext, "Indexed" )) failures++;
	}
	// tiny mesh: the root is a fat leaf.
	{
		BVH6_INTEL_HW hw;
		hw.Build( scene.data(), 3 );
		BVH tiny3;
		tiny3.Build( scene.data(), 3 );
		const bvhvec3 m = tiny3.bvhNode[0].aabbMin, e = tiny3.bvhNode[0].aabbMax - m;
		if (Compare( tiny3, hw, m - e, e * 3, "Tiny" )) failures++;
	}
	printf( failures ? "FAILED\n" : "all tests passed\n" );
	return failures ? 1 : 0;
}
