// Regression tests for state propagation from a TLAS to a custom BLAS.
//
// Float and double variants intentionally perform the same setup and checks:
// a ray with mask 2 skips instance 0, selects instance 1, and must reach the
// BLAS callback with the selected instance ID, mask, and tmax. Each variant
// checks both occlusion and intersection traversal; the latter also verifies
// that the BLAS hit record is returned to the caller.

#define TINYBVH_IMPLEMENTATION
#define TINYBVH_NO_SIMD
#include "tiny_bvh.h"
#include <cstdio>

using namespace tinybvh;

static bool floatStateValid = false, doubleStateValid = false;
static int floatCalls = 0, doubleCalls = 0;

static void FloatAABB( const unsigned, bvhvec3& bmin, bvhvec3& bmax )
{
	bmin = bvhvec3( -1, -1, 4 );
	bmax = bvhvec3( 1, 1, 6 );
}

static void DoubleAABB( const uint64_t, bvhdbl3& bmin, bvhdbl3& bmax )
{
	bmin = bvhdbl3( -1, -1, 4 );
	bmax = bvhdbl3( 1, 1, 6 );
}

static bool FloatOccludes( const Ray& ray, const unsigned )
{
	floatCalls++;
	return floatStateValid = ray.instIdx == (1u << INST_IDX_SHFT) && ray.mask == 2 && ray.hit.t == 3.0f;
}

static bool DoubleOccludes( const RayEx& ray, const uint64_t )
{
	doubleCalls++;
	return doubleStateValid = ray.instIdx == 1 && ray.mask == 2 && ray.hit.t == 3.0;
}

static bool FloatIntersects( Ray& ray, const unsigned )
{
	floatCalls++;
	if (!(floatStateValid = ray.instIdx == (1u << INST_IDX_SHFT) && ray.mask == 2 && ray.hit.t == 3.0f)) return false;
	ray.hit.t = 2.0f;
	return true;
}

static bool DoubleIntersects( RayEx& ray, const uint64_t )
{
	doubleCalls++;
	if (!(doubleStateValid = ray.instIdx == 1 && ray.mask == 2 && ray.hit.t == 3.0)) return false;
	ray.hit.t = 2.0;
	return true;
}

static bool TestFloat()
{
	BVH blas, tlas;
	blas.Build( &FloatAABB, 1 );
	blas.customIsOccluded = &FloatOccludes;
	blas.customIntersect = &FloatIntersects;
	BVHBase* blases[] = { &blas };
	BLASInstance instances[] = { BLASInstance( 0 ), BLASInstance( 0 ) };
	instances[0].mask = 1;
	instances[1].mask = 2;
	tlas.Build( instances, 2, blases, 1 );

	floatCalls = 0;
	floatStateValid = false;
	const Ray shadow( bvhvec3( 0, 0, 0 ), bvhvec3( 0, 0, 1 ), 3.0f, 2 );
	if (!tlas.IsOccluded( shadow ) || floatCalls != 1 || !floatStateValid) return false;
	floatCalls = 0;
	floatStateValid = false;
	Ray probe( bvhvec3( 0, 0, 0 ), bvhvec3( 0, 0, 1 ), 3.0f, 2 );
	tlas.Intersect( probe );
	return floatCalls == 1 && floatStateValid && probe.hit.t == 2.0f;
}

static bool TestDouble()
{
	BVH_Double blas, tlas;
	blas.Build( &DoubleAABB, 1 );
	blas.customIsOccluded = &DoubleOccludes;
	blas.customIntersect = &DoubleIntersects;
	BVH_Double* blases[] = { &blas };
	BLASInstanceEx instances[] = { BLASInstanceEx( 0 ), BLASInstanceEx( 0 ) };
	instances[0].mask = 1;
	instances[1].mask = 2;
	tlas.Build( instances, 2, blases, 1 );

	doubleCalls = 0;
	doubleStateValid = false;
	const RayEx shadow( bvhdbl3( 0, 0, 0 ), bvhdbl3( 0, 0, 1 ), 3.0, 2 );
	if (!tlas.IsOccluded( shadow ) || doubleCalls != 1 || !doubleStateValid) return false;
	doubleCalls = 0;
	doubleStateValid = false;
	RayEx probe( bvhdbl3( 0, 0, 0 ), bvhdbl3( 0, 0, 1 ), 3.0, 2 );
	tlas.Intersect( probe );
	return doubleCalls == 1 && doubleStateValid && probe.hit.t == 2.0;
}

int main()
{
	if (!TestFloat()) { printf( "FAIL: float TLAS state propagation.\n" ); return 1; }
	if (!TestDouble()) { printf( "FAIL: double TLAS state propagation.\n" ); return 1; }
	printf( "TLAS state tests passed.\n" );
	return 0;
}
