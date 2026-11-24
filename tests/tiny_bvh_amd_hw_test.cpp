// Validation of BVH4_AMD_HW (RDNA2/3) and CWBVH8_AMD_HW (RDNA4): a walk over the packed buffers that
// addresses nodes the way the hardware does (every triangle once, vertices bit-exact), and the CPU
// traversal of the packed buffers compared against BVH::Intersect.
//
// Usage: tiny_bvh_amd_hw_test [scene] - default ./testdata/cryteksponza.bin.

#define TINYBVH_IMPLEMENTATION
#include "tiny_bvh.h"
#include <cstdio>
#include <cmath>
#include <cstring>
#include <vector>
#include <set>

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

struct Stats
{
	uint64_t boxes = 0, boxes16 = 0, leafNodes = 0, pairNodes = 0, records = 0, errors = 0;
	std::vector<uint32_t> triSeen;
	const bvhvec4* verts = 0;
	const uint32_t* indices = 0;
	void Triangle( const uint32_t prim, const bvhvec3& a, const bvhvec3& b, const bvhvec3& c )
	{
		if (prim >= triSeen.size()) { errors++; return; }
		triSeen[prim]++;
		const bvhvec3 v[3] = { a, b, c };
		for (int k = 0; k < 3; k++)
		{
			const bvhvec4& r = verts[indices ? indices[prim * 3 + k] : prim * 3 + k];
			if (memcmp( &v[k].x, &r.x, 12 ) && errors++ < 5) printf( "  vertex mismatch, prim %u\n", prim );
		}
	}
};

static float Half( uint32_t h )
{
	const uint32_t s = (h & 0x8000u) << 16, e = (h >> 10) & 31, m = h & 1023;
	uint32_t x = e == 31 ? (s | 0x7f800000u | (m << 13)) : e == 0 ? s : (s | ((e + 112) << 23) | (m << 13));
	float f;
	memcpy( &f, &x, 4 );
	if (e == 0) f = (s ? -1.0f : 1.0f) * (float)m * 5.9604644775390625e-8f;
	return f;
}

// RDNA2/3: walk from the root box at offset 0; children are (offset >> 3) | type.
static void WalkRDNA3( const BVH4_AMD_HW& hw, Stats& s )
{
	const uint8_t* base = (const uint8_t*)hw.bvh4Data;
	const uint64_t size = (uint64_t)hw.usedBlocks * 16;
	std::vector<uint32_t> todo = { 5 };
	while (!todo.empty())
	{
		const uint32_t id = todo.back(), type = id & 7;
		todo.pop_back();
		const uint64_t offset = (uint64_t)(id & ~7u) << 3;
		if (offset + 64 > size || (type == 5 && offset + 128 > size)) { s.errors++, printf( "  node out of range\n" ); continue; }
		const uint8_t* node = base + offset;
		if (type == 5 || type == 4)
		{
			type == 5 ? s.boxes++ : s.boxes16++;
			const uint32_t* w = (const uint32_t*)node;
			for (int c = 0; c < 4; c++) if (w[c] != 0xffffffffu)
			{
				// the child's content must lie within its box
				bvhvec3 bmin, bmax;
				if (type == 5) { const float* b = (const float*)(node + 16 + c * 24); bmin = bvhvec3( b[0], b[1], b[2] ), bmax = bvhvec3( b[3], b[4], b[5] ); }
				else { const uint32_t* h = w + 4 + c * 3; bmin = bvhvec3( Half( h[0] & 0xffff ), Half( h[0] >> 16 ), Half( h[1] & 0xffff ) ), bmax = bvhvec3( Half( h[1] >> 16 ), Half( h[2] & 0xffff ), Half( h[2] >> 16 ) ); }
				if (!(bmin.x <= bmax.x && bmin.y <= bmax.y && bmin.z <= bmax.z)) s.errors++, printf( "  inverted child box\n" );
				todo.push_back( w[c] );
			}
			continue;
		}
		if (type > 1) { s.errors++, printf( "  unexpected node type %u\n", type ); continue; }
		type ? s.pairNodes++ : s.leafNodes++;
		for (uint32_t k = 0; k <= type; k++)
		{
			bvhvec3 a, b, c;
			uint32_t prim;
			BVH4_AMD_HW::NodeTriangle( node, k, a, b, c, prim );
			s.Triangle( prim, a, b, c );
		}
	}
}

// RDNA4: child pointers per GPURT DecodeChildrenOffsets (separate running offsets for box and primitive children).
static void WalkRDNA4( const CWBVH8_AMD_HW& hw, Stats& s )
{
	const uint8_t* base = (const uint8_t*)hw.bvh8Data;
	const uint64_t size = (uint64_t)hw.usedBlocks * 16;
	std::set<uint64_t> records;
	std::vector<uint32_t> todo = { 5 };
	while (!todo.empty())
	{
		const uint32_t id = todo.back();
		todo.pop_back();
		const uint64_t offset = (uint64_t)(id & ~15u) << 3;
		if (offset + 128 > size) { s.errors++, printf( "  node out of range\n" ); continue; }
		const uint32_t* w = (const uint32_t*)(base + offset);
		if ((id & 15) == 5)
		{
			s.boxes++;
			const uint32_t count = ((w[6] >> 28) & 7) + 1;
			uint32_t internalOffset = w[0], primitiveOffset = w[1];
			for (uint32_t c = 0; c < count; c++)
			{
				const uint32_t* ci = w + 8 + c * 3, type = (ci[2] >> 24) & 15, range = ci[2] >> 28;
				if (type == 5) todo.push_back( internalOffset | 5 ), internalOffset += range << 4;
				else if ((type & 4) == 0) todo.push_back( primitiveOffset | type ), primitiveOffset += range << 4;
				else s.errors++, printf( "  unexpected child type %u\n", type );
			}
			for (uint32_t c = count; c < 8; c++) if (w[8 + c * 3] != 0xffffffffu) s.errors++, printf( "  empty slot not marked\n" );
			continue;
		}
		records.insert( offset );
		bvhvec3 v[6];
		uint32_t prim[2];
		const uint32_t pair = (id & 3) | ((id & 8) ? 4 : 0);
		if (pair > (((w[0] >> 28) & 7))) { s.errors++, printf( "  pair %u beyond the record's pair count\n", pair ); continue; }
		const uint32_t tris = CWBVH8_AMD_HW::RecordPair( *(const CWBVH8_AMD_HW::AMDTriangleNode*)w, pair, v, prim );
		tris == 2 ? s.pairNodes++ : s.leafNodes++;
		for (uint32_t t = 0; t < tris; t++) s.Triangle( prim[t], v[t * 3], v[t * 3 + 1], v[t * 3 + 2] );
	}
	s.records = records.size();
}

template <class T> static int Compare( const BVH& ref, const T& hw, const bvhvec3& bmin, const bvhvec3& ext, const char* label, double& steps )
{
	int mismatches = 0, hits = 0;
	const int N = 200000;
	const float diag = tinybvh_length( ext );
	steps = 0;
	for (int i = 0; i < N; i++)
	{
		const bvhvec3 O = bmin + bvhvec3( Rand() * ext.x, Rand() * ext.y, Rand() * ext.z );
		bvhvec3 D = tinybvh_normalize( bvhvec3( Rand() - 0.5f, Rand() - 0.5f, Rand() - 0.5f ) );
		if (i % 7 == 0) D = bvhvec3( (float)(i % 3 == 0), (float)(i % 3 == 1), (float)(i % 3 == 2) ); // axis-aligned rays
		Ray a( O, D ), b( O, D );
		ref.Intersect( a );
		steps += hw.Intersect( b );
		if (a.hit.t < BVH_FAR) hits++;
		if (a.hit.t >= BVH_FAR && b.hit.t >= BVH_FAR) continue;
		if (a.hit.prim == b.hit.prim && fabsf( a.hit.t - b.hit.t ) <= 1e-3f * a.hit.t) continue; // same triangle: rounding (grazing rays, fast math)
		if (fabsf( a.hit.t - b.hit.t ) <= 1e-6f * diag) continue; // shared edge
		if (mismatches++ < 10) printf( "  [%s] ray %i: ref t=%f prim=%u, hw t=%f prim=%u\n", label, i, a.hit.t, a.hit.prim, b.hit.t, b.hit.prim );
	}
	steps /= N;
	printf( "%s: %i rays, %i hits, %i mismatches, %.2f nodes visited per ray\n", label, N, hits, mismatches, steps );
	return mismatches;
}

static int Check( Stats& s, const uint32_t triCount, const char* label, const bool spatialSplits = false )
{
	uint32_t missing = 0, dup = 0;
	for (uint32_t t = 0; t < triCount; t++) missing += s.triSeen[t] == 0, dup += s.triSeen[t] > 1 && !spatialSplits; // spatial splits reference a triangle from several leaves
	if (missing || dup || s.errors) printf( "  [%s] %u missing, %u duplicated, %llu errors\n", label, missing, dup, (unsigned long long)s.errors );
	return (missing || dup || s.errors) ? 1 : 0;
}

int main( int argc, char** argv )
{
	const char* fileName = argc > 1 ? argv[1] : "./testdata/cryteksponza.bin";
	std::vector<bvhvec4> scene;
	if (!LoadScene( fileName, scene )) { printf( "could not load %s\n", fileName ); return 1; }
	const uint32_t triCount = (uint32_t)scene.size() / 3;
	printf( "scene: %s, %u triangles\n", fileName, triCount );
	BVH ref;
	ref.Build( scene.data(), triCount );
	const bvhvec3 bmin = ref.bvhNode[0].aabbMin, ext = ref.bvhNode[0].aabbMax - bmin;
	int failures = 0;
	double steps;
	for (int hq = 0; hq < 2; hq++)
	{
		char label[64];
		{
			BVH4_AMD_HW hw;
			if (hq) hw.BuildHQ( scene.data(), triCount ); else hw.Build( scene.data(), triCount );
			Stats s;
			s.triSeen.assign( triCount, 0 ), s.verts = scene.data();
			WalkRDNA3( hw, s );
			snprintf( label, 64, "RDNA3 %s", hq ? "BuildHQ" : "Build" );
			const uint64_t tris = s.leafNodes + 2 * s.pairNodes;
			printf( "%s: %.2f B/tri, %llu fp32 + %llu fp16 boxes, %llu triangle nodes (%.1f%% of triangles paired), SAH %.2f\n", label,
				hw.usedBlocks * 16.0 / triCount, (unsigned long long)s.boxes, (unsigned long long)s.boxes16, (unsigned long long)(s.leafNodes + s.pairNodes),
				100.0 * 2 * s.pairNodes / (tris ? tris : 1), (double)hw.SAHCost() );
			failures += Check( s, triCount, label, hq );
			failures += Compare( ref, hw, bmin, ext, label, steps ) ? 1 : 0;
		}
		{
			CWBVH8_AMD_HW hw;
			if (hq) hw.BuildHQ( scene.data(), triCount ); else hw.Build( scene.data(), triCount );
			Stats s;
			s.triSeen.assign( triCount, 0 ), s.verts = scene.data();
			WalkRDNA4( hw, s );
			snprintf( label, 64, "RDNA4 %s", hq ? "BuildHQ" : "Build" );
			const uint64_t tris = s.leafNodes + 2 * s.pairNodes;
			printf( "%s: %.2f B/tri, %llu boxes, %llu records (%.2f tris/record), %.1f%% of triangles paired, SAH %.2f\n", label,
				hw.usedBlocks * 16.0 / triCount, (unsigned long long)s.boxes, (unsigned long long)s.records, (double)tris / (s.records ? s.records : 1),
				100.0 * 2 * s.pairNodes / (tris ? tris : 1), (double)hw.SAHCost() );
			failures += Check( s, triCount, label, hq );
			failures += Compare( ref, hw, bmin, ext, label, steps ) ? 1 : 0;
		}
	}
	// indexed mesh: identical vertices welded into one index
	{
		std::vector<bvhvec4> verts;
		std::vector<uint32_t> idx( triCount * 3 );
		for (uint32_t i = 0; i < triCount * 3; i++)
		{
			uint32_t found = UINT32_MAX;
			for (uint32_t k = (i / 3) * 3 >= 3 ? (i / 3) * 3 - 3 : 0; k < i && found == UINT32_MAX; k++)
				if (scene[k].x == scene[i].x && scene[k].y == scene[i].y && scene[k].z == scene[i].z) found = idx[k];
			if (found == UINT32_MAX) found = (uint32_t)verts.size(), verts.push_back( scene[i] );
			idx[i] = found;
		}
		BVH4_AMD_HW a;
		a.Build( verts.data(), idx.data(), triCount );
		Stats s3;
		s3.triSeen.assign( triCount, 0 ), s3.verts = verts.data(), s3.indices = idx.data();
		WalkRDNA3( a, s3 );
		failures += Check( s3, triCount, "RDNA3 indexed" );
		failures += Compare( ref, a, bmin, ext, "RDNA3 indexed", steps ) ? 1 : 0;
		CWBVH8_AMD_HW b;
		b.Build( verts.data(), idx.data(), triCount );
		Stats s4;
		s4.triSeen.assign( triCount, 0 ), s4.verts = verts.data(), s4.indices = idx.data();
		WalkRDNA4( b, s4 );
		failures += Check( s4, triCount, "RDNA4 indexed" );
		failures += Compare( ref, b, bmin, ext, "RDNA4 indexed", steps ) ? 1 : 0;
	}
	// tiny meshes: the root holds the only leaves
	for (uint32_t n = 1; n <= 3; n++)
	{
		BVH tiny;
		tiny.Build( scene.data(), n );
		const bvhvec3 m = tiny.bvhNode[0].aabbMin, e = tiny.bvhNode[0].aabbMax - m;
		BVH4_AMD_HW a;
		a.Build( scene.data(), n );
		CWBVH8_AMD_HW b;
		b.Build( scene.data(), n );
		Stats s3, s4;
		s3.triSeen.assign( n, 0 ), s3.verts = scene.data(), s4.triSeen.assign( n, 0 ), s4.verts = scene.data();
		WalkRDNA3( a, s3 ), WalkRDNA4( b, s4 );
		failures += Check( s3, n, "RDNA3 tiny" ) + Check( s4, n, "RDNA4 tiny" );
		failures += Compare( tiny, a, m - e, e * 3, "RDNA3 tiny", steps ) ? 1 : 0;
		failures += Compare( tiny, b, m - e, e * 3, "RDNA4 tiny", steps ) ? 1 : 0;
	}
	printf( failures ? "FAILED\n" : "all tests passed\n" );
	return failures ? 1 : 0;
}
