#include "tiny_bvh.h"

using namespace std;
using namespace tinybvh;

#include "primitive_set.h"
#include "ray_distribution.h"

namespace tinybvh
{

// Xor32 RNG
static uint32_t seed = 0x12345678;
static uint32_t RandomUInt() { seed ^= seed << 13, seed ^= seed >> 17, seed ^= seed << 5; return seed; }
static float RandomFloat() { return RandomUInt() * 2.3283064365387e-10f; }

// uniform diffuse bounce
bvhvec3 DiffuseReflection( const bvhvec3 N )
{
	bvhvec3 R;
	do
	{
		R = bvhvec3( RandomFloat() * 2 - 1, RandomFloat() * 2 - 1, RandomFloat() * 2 - 1 );
	} while (tinybvh_dot( R, R ) > 1);
	return tinybvh_normalize( tinybvh_dot( R, N ) < 0 ? R : -R );
}

RayDistribution::RayDistribution( RaySet r, PrimitiveSet* p )
{
	// allocate room for ray batches
	O = new bvhvec3[RAY_BATCH_SIZE];
	D = new bvhvec3[RAY_BATCH_SIZE];
	tmin = new float[RAY_BATCH_SIZE];
	tmax = new float[RAY_BATCH_SIZE];
	// common calculations
	const int scrWidth = (int)sqrtf( RAY_BATCH_SIZE );
	const int tilesPerRow = scrWidth / 4;
	const float uvScale = 1.0f / scrWidth;
	// create ray distribution
	raySet = r;
	if (raySet == PRIMARY_VIEW1 || raySet == PRIMARY_VIEW2 || raySet == PRIMARY_VIEW3)
	{
		// create RAY_BATCH_SIZE rays in tiles of 4x4
		UpdateViewPyramid( p->GetCameraPos( raySet ), p->GetCameraDir( raySet ) );
		for (int i = 0, ty = 0; ty < tilesPerRow; ty++ ) for( int tx = 0; tx < tilesPerRow; tx++ )
		{
			for( int y = 0; y < 4; y++ ) for( int x = 0; x < 4; x++, i++ ) 
			{
				float u = (float)(x + tx * 4) * uvScale, v = (float)(y + ty * 4) * uvScale;
				bvhvec3 P = p1 + u * (p2 - p1) + v * (p3 - p1);
				O[i] = eye, D[i] = tinybvh_normalize( P - eye );
				tmin[i] = 0, tmax[i] = BVH_FAR;
			}
		}
		// all done.
		rayCount = RAY_BATCH_SIZE;
	}
	else if (raySet == FIRST_BOUNCE)
	{
		// we need a BVH to generate the rays.
		BVH bvh;
		bvh.settings.useSIMDifavailable = true; // fast build is preferred here.
		bvh.Build( p->verts, p->primCount );
		// fire rays from the first camera; use a diffuse bounce at the first hit
		UpdateViewPyramid( p->GetCameraPos( raySet ), p->GetCameraDir( raySet ) );
		const bvhvec3 sceneExtent = bvh.aabbMax - bvh.aabbMin;
		const float epsilon = sceneExtent[tinybvh_maxdim( sceneExtent )] * 1e-6f;
		int i = 0;
		while (1) for( int y = 0; y < scrWidth; y += 3 ) for( int x = 0; x < scrWidth; x += 3 )
		{
			const float u = (float)x * uvScale, v = (float)y * uvScale;
			const bvhvec3 P = p1 + u * (p2 - p1) + v * (p3 - p1);
			Ray ray( eye, tinybvh_normalize( P - eye ) );
			bvh.Intersect( ray );
			if (ray.hit.t > 1e20f) continue;
			const bvhvec3 I = eye + ray.hit.t * ray.D;
			const uint32_t t = ray.hit.prim * 3;
			const bvhvec3 a = p->verts[t], b = p->verts[t + 1], c = p->verts[t + 2];
			bvhvec3 N = tinybvh_normalize( tinybvh_cross( b - a, a - c ) );
			if (tinybvh_dot( ray.D, N ) > 1) N *= -1.0f;
			const bvhvec3 R = DiffuseReflection( N );
			O[i] = I + epsilon * N, D[i] = R, tmin[i] = 0, tmax[i] = BVH_FAR;
			if (++i == RAY_BATCH_SIZE) goto batch_full;
		}
	batch_full:
		// all done.
		rayCount = RAY_BATCH_SIZE;
	}
	else
	{
		exit( 0 ); // distribution does not exist (yet).
	}
	// construct description
	switch (raySet)
	{
	case PRIMARY_VIEW1: strncpy( desc, "primary rays cam1", 256 ); break;
	case PRIMARY_VIEW2: strncpy( desc, "primary rays cam2", 256 ); break;
	case PRIMARY_VIEW3: strncpy( desc, "primary rays cam3", 256 ); break;
	case FIRST_BOUNCE: strncpy( desc, "first bounce rays", 256 ); break;
	default: strncpy( desc, "UNKNOWN RAY SET", 256 ); break;
	};
}

RayDistribution::~RayDistribution()
{
	delete O;
	delete D;
	delete tmin;
	delete tmax;
	O = 0, D = 0, tmin = 0, tmax = 0;
}

void RayDistribution::UpdateViewPyramid( const bvhvec3 camPos, const bvhvec3 camDir )
{
	eye = camPos;
	bvhvec3 right = tinybvh_normalize( tinybvh_cross( bvhvec3( 0, 1, 0 ), camDir ) );
	bvhvec3 up = 0.8f * tinybvh_cross( camDir, right ), C = eye + 2 * camDir;
	p1 = C - right + up, p2 = C + right + up, p3 = C - right - up;
}

};