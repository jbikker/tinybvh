// settings.autoOrient: a BVH built in a rotated frame must answer every query as the unrotated BVH does.
// Scenes: a grid of box 'buildings' rotated 30 degrees about y (plain and indexed), and Sponza, which is
// axis aligned and must stay unrotated.
// Usage: tiny_bvh_orient_test [scene] - default ./testdata/cryteksponza.bin.

#define TINYBVH_IMPLEMENTATION
#include "tiny_bvh.h"
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <vector>

using namespace tinybvh;

static int failures = 0;
static uint32_t seed = 0x13579bd;
static float Rand() { seed ^= seed << 13, seed ^= seed >> 17, seed ^= seed << 5; return (seed >> 8) * (1.0f / 16777216.0f); }

static void Check( const bool ok, const char* what ) { if (!ok) printf( "FAIL: %s\n", what ), failures++; }

// boxes on a grid, the whole grid rotated by 'deg' about y
static std::vector<bvhvec4> City( const float deg, const int grid )
{
	std::vector<bvhvec4> v;
	const float c = cosf( deg * 3.14159265f / 180 ), s = sinf( deg * 3.14159265f / 180 );
	auto put = [&]( float x, float y, float z ) { v.push_back( bvhvec4( c * x - s * z, y, s * x + c * z, 0 ) ); };
	for (int i = 0; i < grid; i++) for (int j = 0; j < grid; j++)
	{
		const float x0 = i * 3.0f, z0 = j * 3.0f, x1 = x0 + 2, z1 = z0 + 2, h = 1 + Rand() * 6;
		const float p[8][3] = { { x0, 0, z0 }, { x1, 0, z0 }, { x1, 0, z1 }, { x0, 0, z1 }, { x0, h, z0 }, { x1, h, z0 }, { x1, h, z1 }, { x0, h, z1 } };
		const int f[12][3] = { { 0, 1, 5 }, { 0, 5, 4 }, { 1, 2, 6 }, { 1, 6, 5 }, { 2, 3, 7 }, { 2, 7, 6 }, { 3, 0, 4 }, { 3, 4, 7 }, { 4, 5, 6 }, { 4, 6, 7 }, { 0, 2, 1 }, { 0, 3, 2 } };
		for (const auto& t : f) for (int k = 0; k < 3; k++) put( p[t[k]][0], p[t[k]][1], p[t[k]][2] );
	}
	return v;
}

// rays between random points of the scene box, and from outside it
static std::vector<Ray> Rays( const bvhvec3 lo, const bvhvec3 hi, const uint32_t count )
{
	std::vector<Ray> rays;
	const bvhvec3 e = hi - lo;
	for (uint32_t i = 0; i < count; i++)
	{
		const bvhvec3 O = lo - e * 0.2f + e * 1.4f * bvhvec3( Rand(), Rand(), Rand() );
		const bvhvec3 P = lo + e * bvhvec3( Rand(), Rand(), Rand() );
		rays.push_back( Ray( O, tinybvh_normalize( P - O ) ) );
	}
	return rays;
}

// nearest and any-hit queries of 'b' against the reference 'a'. Rotating rays and vertices costs about an ulp
// of the scene's coordinates, as any instance transform does: a hit counts as the same when t agrees within
// that (absTol, set from the scene size), and a hit/miss flip only at an edge
static float absTol = 0;
template <typename A, typename B> static void Compare( const char* label, const A& a, const B& b, const std::vector<Ray>& rays, const float tol = 1e-4f )
{
	int hits = 0, differ = 0, interior = 0, occl = 0;
	// at an edge, or at the ray origin (a ray starting on a surface; the triangle test accepts t >= 0)
	auto edge = []( const Ray& r ) { return r.hit.t < BVH_FAR && (tinybvh_min( tinybvh_min( r.hit.u, r.hit.v ), 1 - r.hit.u - r.hit.v ) < 1e-3f || r.hit.t <= absTol); };
	for (const Ray& r : rays)
	{
		Ray ra = r, rb = r;
		a.Intersect( ra ), b.Intersect( rb );
		const bool ha = ra.hit.t < BVH_FAR, hb = rb.hit.t < BVH_FAR;
		hits += ha;
		if (ha != hb || (ha && fabsf( ra.hit.t - rb.hit.t ) > tol * ra.hit.t + absTol))
		{
			differ++;
			if (edge( ra ) || edge( rb )) continue;
			if (interior++ < 3) printf( "    t %g prim %u uv %g %g vs t %g prim %u uv %g %g\n", ra.hit.t, (uint32_t)ra.hit.prim, ra.hit.u, ra.hit.v, rb.hit.t, (uint32_t)rb.hit.prim, rb.hit.u, rb.hit.v );
		}
		occl += a.IsOccluded( r ) != b.IsOccluded( r );
	}
	// every difference must be an edge hit on one side; occlusion may flip on such edges as well
	const bool ok = interior == 0 && differ <= (int)rays.size() / 1000 && occl <= (int)rays.size() / 1000;
	printf( "%-44s %6d hits, %d differ (%d not at an edge), %d occlusion differ%s\n", label, hits, differ, interior, occl, ok ? "" : "  <-- FAIL" );
	if (!ok) failures++;
}

static bool IsIdentity( const bvhmat4& m ) { for (int i = 0; i < 16; i++) if (m[i] != (i % 5 == 0 ? 1.0f : 0.0f)) return false; return true; }

int main( int argc, char** argv )
{
	const char* sceneFile = argc > 1 ? argv[1] : "./testdata/cryteksponza.bin";
	std::vector<bvhvec4> city = City( 30, 40 );
	const uint32_t cityTris = (uint32_t)city.size() / 3;
	bvhvec3 lo( 1e30f ), hi( -1e30f );
	for (const bvhvec4& p : city) lo = tinybvh_min( lo, bvhvec3( p ) ), hi = tinybvh_max( hi, bvhvec3( p ) );
	const std::vector<Ray> rays = Rays( lo, hi, 100000 );
	absTol = 1e-5f * tinybvh_length( hi - lo );

	// the default: no rotation
	BVH plain;
	plain.Build( city.data(), cityTris );
	Check( !plain.isOriented() && IsIdentity( plain.orientation ), "default build is not the identity" );

	// the option: a rotation about y, and the same answers from every builder
	struct Variant { const char* name; bool hq, binned, simd, quick; };
	const Variant variants[] = { { "Build (default)", false, false, true, false }, { "Build, binned", false, true, true, false },
		{ "Build, reference builder", false, true, false, false }, { "BuildHQ (Bonsai HQ)", true, false, true, false },
		{ "BuildHQ, SBVH", true, true, false, false }, { "BuildQuick", false, false, true, true } };
	for (const Variant& vr : variants)
	{
		BVH o;
		o.settings.autoOrient = true, o.settings.binnedBVH = vr.binned, o.settings.useSIMDifavailable = vr.simd;
		if (vr.quick) o.BuildQuick( city.data(), cityTris ); else if (vr.hq) o.BuildHQ( city.data(), cityTris ); else o.Build( city.data(), cityTris );
		char label[128];
		const float yaw = atan2f( o.orientation[2], o.orientation[0] ) * 57.2958f;
		snprintf( label, sizeof( label ), "%s: oriented %d, yaw %.0f", vr.name, (int)o.isOriented(), yaw );
		Check( o.isOriented(), "city not oriented" );
		Check( o.orientation[5] == 1 && o.orientation[1] == 0 && o.orientation[4] == 0, "rotation is not about y" );
		Compare( label, plain, o, rays );
	}

	// indexed input
	{
		std::vector<uint32_t> idx( city.size() );
		for (uint32_t i = 0; i < idx.size(); i++) idx[i] = i;
		BVH o;
		o.settings.autoOrient = true;
		o.Build( city.data(), idx.data(), cityTris );
		Check( o.isOriented(), "indexed city not oriented" );
		Compare( "indexed Build", plain, o, rays );
	}

	// Refit after moving the vertices, Optimize, and the sphere and capsule queries
	{
		std::vector<bvhvec4> moved = city;
		BVH o, ref;
		o.settings.autoOrient = true;
		o.Build( moved.data(), cityTris ), ref.Build( moved.data(), cityTris );
		for (bvhvec4& p : moved) p.y += 0.25f * sinf( p.x * 0.1f ), p.x += 0.5f;
		o.Refit(), ref.Refit();
		Compare( "Refit after moving the vertices", ref, o, rays );
		o.Optimize( 5 );
		Check( o.isOriented(), "Optimize lost the orientation" );
		Compare( "Optimize", ref, o, rays );
		int differ = 0;
		for (int i = 0; i < 2000; i++)
		{
			const bvhvec3 p = lo + (hi - lo) * bvhvec3( Rand(), Rand(), Rand() ), q = p + bvhvec3( Rand() - 0.5f, Rand() - 0.5f, Rand() - 0.5f ) * 4;
			const float r = 0.05f + Rand() * 0.5f;
			differ += o.IntersectSphere( p, r ) != ref.IntersectSphere( p, r );
			differ += o.IntersectCapsule( p, q, r ) != ref.IntersectCapsule( p, q, r );
		}
		printf( "%-44s %d of 4000 differ\n", "IntersectSphere / IntersectCapsule", differ );
		Check( differ <= 2, "sphere / capsule queries differ" );
	}

	// Save and Load: with the caller's geometry, and with the geometry in the file
	{
		BVH o;
		o.settings.autoOrient = true;
		o.Build( city.data(), cityTris );
		Check( o.Save( "orient_test.bvh", false ), "Save" );
		BVH l;
		Check( l.Load( "orient_test.bvh", city.data(), cityTris ), "Load with caller geometry" );
		Check( l.isOriented(), "Load lost the orientation" );
		Compare( "Load (caller geometry)", plain, l, rays );
		Check( o.Save( "orient_test_geom.bvh", true ), "Save with geometry" );
		BVH g = BVH::Load( "orient_test_geom.bvh" );
		Compare( "Load (geometry from the file)", plain, g, rays );
		remove( "orient_test.bvh" ), remove( "orient_test_geom.bvh" );
	}

	// BVH_GPU carries the rotation: Build, BuildHQ, indexed, Optimize, ConvertFrom and Load
	{
		BVH_GPU g, h, gi;
		g.settings.autoOrient = h.settings.autoOrient = gi.settings.autoOrient = true;
		g.Build( city.data(), cityTris ), h.BuildHQ( city.data(), cityTris );
		std::vector<uint32_t> idx( city.size() );
		for (uint32_t i = 0; i < idx.size(); i++) idx[i] = i;
		gi.Build( city.data(), idx.data(), cityTris );
		Check( g.isOriented() && h.isOriented() && gi.isOriented(), "BVH_GPU not oriented" );
		Check( g.orientation[5] == 1 && g.orientation[1] == 0 && g.orientation[4] == 0, "BVH_GPU rotation is not about y" );
		Compare( "BVH_GPU Build", plain, g, rays );
		Compare( "BVH_GPU BuildHQ", plain, h, rays );
		Compare( "BVH_GPU indexed Build", plain, gi, rays );
		h.Optimize( 5 );
		Check( h.isOriented(), "BVH_GPU Optimize lost the orientation" );
		Compare( "BVH_GPU BuildHQ, Optimize", plain, h, rays );
		BVH o;
		o.settings.autoOrient = true;
		o.Build( city.data(), cityTris );
		BVH_GPU c;
		c.ConvertFrom( o );
		Check( c.isOriented(), "BVH_GPU ConvertFrom lost the orientation" );
		Compare( "BVH_GPU ConvertFrom an oriented BVH", plain, c, rays );
		Check( o.Save( "orient_test_gpu.bvh", true ), "Save for BVH_GPU" );
		BVH_GPU l = BVH_GPU::Load( "orient_test_gpu.bvh" );
		Check( l.isOriented(), "BVH_GPU Load lost the orientation" );
		Compare( "BVH_GPU Load", plain, l, rays );
		remove( "orient_test_gpu.bvh" );
		// default: no rotation, and the identity
		BVH_GPU d;
		d.Build( city.data(), cityTris );
		Check( !d.isOriented() && IsIdentity( d.orientation ), "BVH_GPU default build is not the identity" );
		// rebuilt without the option: back to the identity
		g.settings.autoOrient = false;
		g.Build( city.data(), cityTris );
		Check( !g.isOriented() && IsIdentity( g.orientation ), "BVH_GPU rebuild kept a stale rotation" );
		Compare( "BVH_GPU rebuilt without autoOrient", plain, g, rays );
		// as a BLAS: the rotation goes in the instance (transform * transpose( orientation ))
		BVH_GPU ob;
		ob.settings.autoOrient = true;
		ob.Build( city.data(), cityTris );
		BLASInstance inst( 0 );
		for (int r = 0; r < 3; r++) for (int k = 0; k < 3; k++) inst.transform[r * 4 + k] = ob.orientation[k * 4 + r];
		BVHBase* bl[1] = { &ob };
		BVH_GPU tl;
		tl.Build( &inst, 1, bl, 1 );
		Check( inst.aabbMin.x <= lo.x + 1e-3f && inst.aabbMax.x >= hi.x - 1e-3f && inst.aabbMin.z <= lo.z + 1e-3f && inst.aabbMax.z >= hi.z - 1e-3f, "BVH_GPU instance box misses the geometry" );
		// a GPU TLAS kernel takes rays through invTransform: that must map the world geometry onto orderedVerts
		float worst = 0;
		for (uint32_t i = 0; i < (uint32_t)ob.idxCount; i += 97)
		{
			const uint32_t prim = tinybvh_as_uint( ob.orderedVerts[i * 3].w );
			const bvhvec3 p = tinybvh_transform_point( bvhvec3( city[prim * 3] ), inst.invTransform );
			worst = tinybvh_max( worst, tinybvh_length( p - bvhvec3( ob.orderedVerts[i * 3] ) ) );
		}
		Check( worst <= absTol, "BVH_GPU instance invTransform does not map the world onto the tree" );
	}

	// a TLAS over an oriented BLAS, instanced with a rotation and translation
	{
		BVH ob, pb;
		ob.settings.autoOrient = true;
		ob.Build( city.data(), cityTris ), pb.Build( city.data(), cityTris );
		BLASInstance io( 0 ), ip( 0 );
		const float c = cosf( 0.7f ), s = sinf( 0.7f );
		for (BLASInstance* inst : { &io, &ip })
			inst->transform[0] = c, inst->transform[1] = -s, inst->transform[4] = s, inst->transform[5] = c, inst->transform[3] = 5, inst->transform[11] = -3;
		BVHBase* lo1[1] = { &ob }, * lo2[1] = { &pb };
		BVH to, tp;
		to.Build( &io, 1, lo1, 1 ), tp.Build( &ip, 1, lo2, 1 );
		bvhvec3 wlo = tp.aabbMin, whi = tp.aabbMax;
		const std::vector<Ray> wrays = Rays( wlo, whi, 100000 );
		Compare( "TLAS over an oriented BLAS", tp, to, wrays );
		// the instance box must hold the oriented BLAS
		Check( io.aabbMin.x <= ip.aabbMin.x + 1e-3f && io.aabbMax.x >= ip.aabbMax.x - 1e-3f, "instance box of the oriented BLAS too small" );
	}

	// double precision
	{
		std::vector<bvhdbl3> dv( city.size() );
		for (size_t i = 0; i < city.size(); i++) dv[i] = bvhdbl3( city[i].x, city[i].y, city[i].z );
		BVH_Double dp, dor;
		dor.settings.autoOrient = true;
		dp.Build( dv.data(), cityTris ), dor.Build( dv.data(), cityTris );
		Check( dor.isOriented(), "double city not oriented" );
		int differ = 0, hits = 0;
		for (const Ray& r : rays)
		{
			RayEx a( bvhdbl3( r.O.x, r.O.y, r.O.z ), bvhdbl3( r.D.x, r.D.y, r.D.z ) ), b = a;
			dp.Intersect( a ), dor.Intersect( b );
			hits += a.hit.t < BVH_DBL_FAR;
			differ += (a.hit.t < BVH_DBL_FAR) != (b.hit.t < BVH_DBL_FAR) || fabs( a.hit.t - b.hit.t ) > 1e-9 * (a.hit.t < BVH_DBL_FAR ? a.hit.t : 1);
		}
		printf( "%-44s %6d hits, %d differ\n", "BVH_Double", hits, differ );
		Check( differ <= (int)rays.size() / 5000, "double results differ" );
	}

	// an axis-aligned scene stays unrotated
	{
		FILE* f = fopen( sceneFile, "rb" );
		uint32_t n = 0;
		if (f && fread( &n, 4, 1, f ) == 1)
		{
			std::vector<bvhvec4> v( (size_t)n * 3 );
			if (fread( v.data(), 48, n, f ) == n)
			{
				BVH o;
				o.settings.autoOrient = true;
				o.Build( v.data(), n );
				printf( "%-44s oriented %d\n", sceneFile, (int)o.isOriented() );
				Check( !o.isOriented(), "axis-aligned scene was rotated" );
			}
			fclose( f );
		}
		else printf( "(%s not found: skipped)\n", sceneFile );
	}

	printf( failures ? "%d FAILURES\n" : "all tests passed\n", failures );
	return failures ? 1 : 0;
}
