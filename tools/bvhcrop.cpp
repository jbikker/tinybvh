// bvhcrop - cut a tinybvh .bin mesh down to a smaller one, by region or by decimation.
//
// Cropping to a box changes the statistics a BVH builder reacts to: the scene bounds shrink,
// the ratio of occupied volume to bounding volume changes, and so does the triangle count that
// several builders key their own parameters off. A build problem that depends on any of those
// will not survive the crop. Decimation keeps every one of them roughly intact - the bounds to
// within a triangle, the spatial density contrast exactly, since removing a fixed fraction
// everywhere leaves relative density unchanged - while cutting the count enough to move the
// file. It is the right tool for shrinking a scene that is misbehaving, rather than for
// looking at part of one.
//
// The .bin format is a uint32 triangle count followed by three vertices per triangle, each a
// float4. The fourth component carries whatever the producer put there - a color, a material
// index, a 1.0 - so it is copied through untouched rather than normalized.
//
// Usage: bvhcrop <in.bin> <out.bin> -box <minx> <miny> <minz> <maxx> <maxy> <maxz>
//        bvhcrop <in.bin> -info
//
// Build: g++ -O2 -std=c++17 -o bvhcrop bvhcrop.cpp

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <string>
#include <vector>
#include <algorithm>
#include <utility>

struct Vec3
{
	float x = 0, y = 0, z = 0;
	Vec3() = default;
	Vec3( float a, float b, float c ) { x = a, y = b, z = c; }
};

// One vertex as the format stores it: position plus the pass-through component.
struct Vertex
{
	float x, y, z, w;
};

static Vec3 Minimum( const Vec3& a, const Vec3& b )
{
	return Vec3( a.x < b.x ? a.x : b.x, a.y < b.y ? a.y : b.y, a.z < b.z ? a.z : b.z );
}

static Vec3 Maximum( const Vec3& a, const Vec3& b )
{
	return Vec3( a.x > b.x ? a.x : b.x, a.y > b.y ? a.y : b.y, a.z > b.z ? a.z : b.z );
}

// ----------------------------------------------------------------------------------------
// file i/o
// ----------------------------------------------------------------------------------------

// The header count is trusted only as far as the file backs it up: a truncated or mislabelled
// file would otherwise read past the end, and these files are big enough that a wrong count is
// easier to produce than to notice.
static bool LoadMesh( const char* path, std::vector<Vertex>& verts )
{
	FILE* f = fopen( path, "rb" );
	if (!f)
	{
		printf( "error: cannot open '%s'.\n", path );
		return false;
	}
	uint32_t triCount = 0;
	if (fread( &triCount, sizeof( uint32_t ), 1, f ) != 1)
	{
		printf( "error: '%s' is too short to hold a triangle count.\n", path );
		fclose( f );
		return false;
	}
	fseek( f, 0, SEEK_END );
	const long bytes = ftell( f );
	const long expected = (long)sizeof( uint32_t ) + (long)triCount * 3 * (long)sizeof( Vertex );
	if (bytes != expected)
	{
		printf( "error: '%s' claims %u triangles, which needs %ld bytes; the file is %ld.\n",
			path, triCount, expected, bytes );
		fclose( f );
		return false;
	}
	fseek( f, sizeof( uint32_t ), SEEK_SET );
	verts.resize( (size_t)triCount * 3 );
	const size_t read = triCount == 0 ? 0 : fread( verts.data(), sizeof( Vertex ), verts.size(), f );
	fclose( f );
	if (read != verts.size())
	{
		printf( "error: '%s' ended after %zu of %zu vertices.\n", path, read, verts.size() );
		return false;
	}
	return true;
}

static bool SaveMesh( const char* path, const std::vector<Vertex>& verts )
{
	FILE* f = fopen( path, "wb" );
	if (!f)
	{
		printf( "error: cannot write '%s'.\n", path );
		return false;
	}
	const uint32_t triCount = (uint32_t)(verts.size() / 3);
	bool ok = fwrite( &triCount, sizeof( uint32_t ), 1, f ) == 1;
	if (ok && triCount > 0) ok = fwrite( verts.data(), sizeof( Vertex ), verts.size(), f ) == verts.size();
	fclose( f );
	if (!ok) printf( "error: writing '%s' failed.\n", path );
	return ok;
}

// ----------------------------------------------------------------------------------------
// the crop itself
// ----------------------------------------------------------------------------------------

static void SceneBounds( const std::vector<Vertex>& verts, Vec3& bmin, Vec3& bmax )
{
	bmin = Vec3( 1e30f, 1e30f, 1e30f ), bmax = Vec3( -1e30f, -1e30f, -1e30f );
	for (size_t i = 0; i < verts.size(); i++)
	{
		const Vec3 v( verts[i].x, verts[i].y, verts[i].z );
		bmin = Minimum( bmin, v ), bmax = Maximum( bmax, v );
	}
}

// Containment: every vertex inside the box. Overlap: the triangle's own box meets the crop
// box. The second is the cheap test, not the exact one - a triangle whose box overlaps may
// still miss the box itself - so it errs towards keeping, which is the right direction for a
// tool whose output is meant to be a superset of what is visible in the region.
// ----------------------------------------------------------------------------------------
// decimation
// ----------------------------------------------------------------------------------------

// Keep or drop from a hash of the triangle index, not from a running generator: the decision
// for a triangle is then independent of every other, so the same seed and fraction select the
// same subset whatever else changes, and two decimations at different fractions nest.
static bool KeepTriangle( const uint32_t index, const uint32_t seed, const uint32_t threshold )
{
	uint64_t h = (uint64_t)index * 0x9E3779B97F4A7C15ull + (uint64_t)seed * 0xBF58476D1CE4E5B9ull;
	h ^= h >> 30, h *= 0xBF58476D1CE4E5B9ull;
	h ^= h >> 27, h *= 0x94D049BB133111EBull;
	h ^= h >> 31;
	return (uint32_t)(h >> 32) < threshold;
}

// The six triangles that define the scene bounds, one per box plane. Keeping these unconditionally
// is what holds the bounds fixed; without them a decimated scene shrinks slightly on every axis,
// which is exactly the property the decimation is meant to preserve.
static void FindExtremes( const std::vector<Vertex>& verts, uint32_t* extreme )
{
	float best[6] = { 1e30f, 1e30f, 1e30f, -1e30f, -1e30f, -1e30f };
	for (uint32_t i = 0; i < 6; i++) extreme[i] = 0;
	for (size_t v = 0; v < verts.size(); v++)
	{
		const float p[3] = { verts[v].x, verts[v].y, verts[v].z };
		const uint32_t tri = (uint32_t)(v / 3);
		for (uint32_t a = 0; a < 3; a++)
		{
			if (p[a] < best[a]) best[a] = p[a], extreme[a] = tri;
			if (p[a] > best[3 + a]) best[3 + a] = p[a], extreme[3 + a] = tri;
		}
	}
}

// Half the surface area of a triangle's AABB - the quantity a SAH builder actually sees, which
// is why it is the one to rank by when deciding which triangles are too important to drop.
static float BoxArea( const Vertex* tri )
{
	Vec3 bmin( tri[0].x, tri[0].y, tri[0].z ), bmax = bmin;
	for (uint32_t i = 1; i < 3; i++)
	{
		const Vec3 v( tri[i].x, tri[i].y, tri[i].z );
		bmin = Minimum( bmin, v ), bmax = Maximum( bmax, v );
	}
	const Vec3 e( bmax.x - bmin.x, bmax.y - bmin.y, bmax.z - bmin.z );
	return e.x * e.y + e.y * e.z + e.z * e.x;
}

static bool TriangleInside( const Vertex* tri, const Vec3& bmin, const Vec3& bmax, const bool overlap )
{
	if (!overlap)
	{
		for (uint32_t i = 0; i < 3; i++)
		{
			if (tri[i].x < bmin.x || tri[i].y < bmin.y || tri[i].z < bmin.z) return false;
			if (tri[i].x > bmax.x || tri[i].y > bmax.y || tri[i].z > bmax.z) return false;
		}
		return true;
	}
	Vec3 tmin( tri[0].x, tri[0].y, tri[0].z ), tmax = tmin;
	for (uint32_t i = 1; i < 3; i++)
	{
		const Vec3 v( tri[i].x, tri[i].y, tri[i].z );
		tmin = Minimum( tmin, v ), tmax = Maximum( tmax, v );
	}
	if (tmax.x < bmin.x || tmin.x > bmax.x) return false;
	if (tmax.y < bmin.y || tmin.y > bmax.y) return false;
	if (tmax.z < bmin.z || tmin.z > bmax.z) return false;
	return true;
}

static void Usage()
{
	printf(
		"usage: bvhcrop <in.bin> <out.bin> -decimate <fraction>\n"
		"       bvhcrop <in.bin> <out.bin> -box <minx> <miny> <minz> <maxx> <maxy> <maxz>\n"
		"       bvhcrop <in.bin> -info\n"
		"\n"
		"  Writes a subset of a tinybvh .bin mesh to a second .bin of the same format.\n"
		"  Triangles are kept or dropped whole; nothing is clipped or merged.\n"
		"\n"
		"  Decimation thins the whole scene and leaves its shape alone - bounds, aspect ratio\n"
		"  and relative density all survive. Cropping cuts a region out and changes all three,\n"
		"  so a build problem that depends on them will not survive it.\n"
		"\n"
		"  -decimate F        keep fraction F of the triangles, chosen by a hash of the index\n"
		"  -target N          the same, with F chosen to land near N triangles\n"
		"  -seed S            the hash seed; a different one gives a different subset (0)\n"
		"  -keeplarge N       never drop the N triangles with the largest bounding boxes,\n"
		"                     which are few, carry most of a SAH builder's difficulty, and are\n"
		"                     the ones uniform decimation is most likely to take out\n"
		"\n"
		"  -box X Y Z X Y Z   a box, as min corner then max corner\n"
		"  -rel  X Y Z X Y Z  the same, as fractions of the scene bounds: 0 0 0 1 1 1 is the\n"
		"                     whole scene, 0 0 0 0.5 1 1 the half with the lower x\n"
		"  -overlap           for a box, keep every triangle whose own box meets it, not only\n"
		"                     those with all three vertices inside\n"
		"\n"
		"  Given both, the box is kept whole and only what falls outside it is decimated: full\n"
		"  detail where you are looking, thinned surroundings, bounds unchanged.\n"
		"\n"
		"  -info              report triangle count and bounds, and write nothing\n"
		"\n"
		"  The scene bounds are held exactly: the six triangles that define them are never\n"
		"  dropped. Note that a builder keying parameters off the triangle count will still\n"
		"  see a different scene - Bonsai HQ sizes its cells from it, 512 fragments below 100k\n"
		"  primitives rising to 8192 above 4M - so a count-dependent effect can still move.\n" );
}

int main( int argc, char** argv )
{
	if (argc < 3)
	{
		Usage();
		return argc < 2 ? 1 : 0;
	}
	const std::string inPath = argv[1];
	std::string outPath;
	Vec3 bmin( 0, 0, 0 ), bmax( 0, 0, 0 );
	bool haveBox = false, relative = false, overlap = false, info = false;
	double fraction = 1.0;
	uint32_t seed = 0, keepLarge = 0, targetCount = 0;
	bool decimate = false;
	for (int i = 2; i < argc; i++)
	{
		const std::string a = argv[i];
		if (a == "-box" || a == "-rel")
		{
			if (i + 6 >= argc)
			{
				printf( "error: %s needs six numbers.\n", a.c_str() );
				return 1;
			}
			bmin = Vec3( (float)atof( argv[i + 1] ), (float)atof( argv[i + 2] ), (float)atof( argv[i + 3] ) );
			bmax = Vec3( (float)atof( argv[i + 4] ), (float)atof( argv[i + 5] ), (float)atof( argv[i + 6] ) );
			i += 6, haveBox = true, relative = a == "-rel";
		}
		else if (a == "-decimate" || a == "-target" || a == "-seed" || a == "-keeplarge")
		{
			if (i + 1 >= argc)
			{
				printf( "error: %s needs a value.\n", a.c_str() );
				return 1;
			}
			const std::string v = argv[++i];
			if (a == "-decimate") fraction = atof( v.c_str() ), decimate = true;
			else if (a == "-target") targetCount = (uint32_t)atoi( v.c_str() ), decimate = true;
			else if (a == "-seed") seed = (uint32_t)atoi( v.c_str() );
			else keepLarge = (uint32_t)atoi( v.c_str() );
		}
		else if (a == "-overlap") overlap = true;
		else if (a == "-info") info = true;
		else if (a == "-h" || a == "--help")
		{
			Usage();
			return 0;
		}
		else if (!a.empty() && a[0] == '-')
		{
			printf( "error: unknown option '%s'.\n", a.c_str() );
			return 1;
		}
		else if (outPath.empty()) outPath = a;
		else
		{
			printf( "error: unexpected argument '%s'.\n", a.c_str() );
			return 1;
		}
	}
	std::vector<Vertex> verts;
	if (!LoadMesh( inPath.c_str(), verts )) return 1;
	Vec3 sceneMin, sceneMax;
	SceneBounds( verts, sceneMin, sceneMax );
	printf( "input  : %s\n         %u triangles\n", inPath.c_str(), (uint32_t)(verts.size() / 3) );
	if (verts.empty())
	{
		printf( "         empty mesh; nothing to crop.\n" );
		return info ? 0 : 1;
	}
	printf( "bounds : %g %g %g .. %g %g %g\n", sceneMin.x, sceneMin.y, sceneMin.z, sceneMax.x, sceneMax.y, sceneMax.z );
	if (info) return 0;
	if (outPath.empty() || (!haveBox && !decimate))
	{
		printf( "error: an output file and a -decimate, -target, -box or -rel are needed.\n" );
		return 1;
	}
	const uint32_t inCount = (uint32_t)(verts.size() / 3);
	if (targetCount > 0) fraction = (double)targetCount / (double)inCount;
	if (decimate && (fraction <= 0 || fraction > 1))
	{
		printf( "error: the keep fraction is %g; it has to be above 0 and at most 1.\n", fraction );
		return 1;
	}
	// A box given as fractions is resolved against the bounds just reported, so the numbers on
	// the command line stay the same when the scene behind them changes.
	if (relative)
	{
		const Vec3 e( sceneMax.x - sceneMin.x, sceneMax.y - sceneMin.y, sceneMax.z - sceneMin.z );
		bmin = Vec3( sceneMin.x + bmin.x * e.x, sceneMin.y + bmin.y * e.y, sceneMin.z + bmin.z * e.z );
		bmax = Vec3( sceneMin.x + bmax.x * e.x, sceneMin.y + bmax.y * e.y, sceneMin.z + bmax.z * e.z );
	}
	if (bmax.x < bmin.x || bmax.y < bmin.y || bmax.z < bmin.z)
	{
		printf( "error: the crop box is inside out; give the min corner first.\n" );
		return 1;
	}
	if (haveBox) printf( "box    : %g %g %g .. %g %g %g (%s%s)\n", bmin.x, bmin.y, bmin.z, bmax.x, bmax.y, bmax.z,
		overlap ? "triangles overlapping it" : "triangles fully inside it",
		decimate ? ", kept whole" : "" );
	if (decimate) printf( "thin   : keeping %.4f%s, seed %u\n", fraction,
		haveBox ? " of what falls outside the box" : " of the scene", seed );
	// Triangles kept regardless of the decimation: the six that define the scene bounds, and the
	// largest by bounding box if asked for. Marked up front so the main pass stays a single test.
	std::vector<uint8_t> forced( inCount, 0 );
	if (decimate)
	{
		uint32_t extreme[6];
		FindExtremes( verts, extreme );
		for (uint32_t k = 0; k < 6; k++) forced[extreme[k]] = 1;
		if (keepLarge > 0)
		{
			std::vector<std::pair<float, uint32_t>> byArea( inCount );
			for (uint32_t t = 0; t < inCount; t++) byArea[t] = std::make_pair( BoxArea( &verts[(size_t)t * 3] ), t );
			const uint32_t n = keepLarge < inCount ? keepLarge : inCount;
			std::nth_element( byArea.begin(), byArea.begin() + n, byArea.end(),
				[]( const std::pair<float, uint32_t>& a, const std::pair<float, uint32_t>& b ) { return a.first > b.first; } );
			for (uint32_t k = 0; k < n; k++) forced[byArea[k].second] = 1;
			printf( "keep   : %u largest triangles, down to box area %g\n", n, byArea[n > 0 ? n - 1 : 0].first );
		}
	}
	const uint32_t threshold = fraction >= 1.0 ? 0xffffffffu : (uint32_t)(fraction * 4294967296.0);
	std::vector<Vertex> kept;
	for (uint32_t t = 0; t < inCount; t++)
	{
		const size_t v = (size_t)t * 3;
		// With a box and a decimation both given, the box selects what is kept whole and the
		// decimation applies to the rest; with only one of the two, the other test is absent.
		const bool inBox = haveBox && TriangleInside( &verts[v], bmin, bmax, overlap );
		if (haveBox && !decimate && !inBox) continue;
		if (decimate && !inBox && !forced[t] && !KeepTriangle( t, seed, threshold )) continue;
		kept.push_back( verts[v] ), kept.push_back( verts[v + 1] ), kept.push_back( verts[v + 2] );
	}
	const uint32_t outCount = (uint32_t)(kept.size() / 3);
	printf( "kept   : %u of %u triangles (%.1f%%)\n", outCount, inCount, 100.0 * outCount / inCount );
	if (outCount == 0) printf( "warning: the output has no triangles; check the box against the bounds above.\n" );
	else
	{
		Vec3 keptMin, keptMax;
		SceneBounds( kept, keptMin, keptMax );
		printf( "         bounds %g %g %g .. %g %g %g%s\n", keptMin.x, keptMin.y, keptMin.z, keptMax.x, keptMax.y, keptMax.z,
			(keptMin.x == sceneMin.x && keptMin.y == sceneMin.y && keptMin.z == sceneMin.z &&
			 keptMax.x == sceneMax.x && keptMax.y == sceneMax.y && keptMax.z == sceneMax.z) ? " (unchanged)" : "" );
	}
	if (!SaveMesh( outPath.c_str(), kept )) return 1;
	printf( "output : %s\n", outPath.c_str() );
	return 0;
}
