#include "tiny_bvh.h"

using namespace std;
using namespace tinybvh;

#include "primitive_set.h"
#include "ray_distribution.h"

namespace tinybvh
{

RayDistribution::RayDistribution( RaySet r, PrimitiveSet* p )
{
	// create ray distribution
	raySet = r;
	if (raySet == PRIMARY_VIEW1 || raySet == PRIMARY_VIEW2 || raySet == PRIMARY_VIEW3)
	{
		bvhvec3 eye = p->GetCameraPos( raySet );
		bvhvec3 view = p->GetCameraDir( raySet );
		bvhvec3 right = tinybvh_normalize( tinybvh_cross( bvhvec3( 0, 1, 0 ), view ) );
		bvhvec3 up = 0.8f * tinybvh_cross( view, right ), C = eye + 2 * view;
		bvhvec3 p1 = C - right + up, p2 = C + right + up, p3 = C - right - up;
		// create 1M rays
		O = new bvhvec3[1'000'000], D = new bvhvec3[1'000'000];
		tmin = new float[1'000'000], tmax = new float[1'000'000];
		for (int i = 0, y = 0; y < 1000; y++) for (int x = 0; x < 1000; x++, i++)
		{
			float u = (float)x * 0.001f, v = (float)y * 0.001f;
			bvhvec3 P = p1 + u * (p2 - p1) + v * (p3 - p1);
			O[i] = eye, D[i] = tinybvh_normalize( P - eye );
			tmin[i] = 0, tmax[i] = BVH_FAR;
		}
		rayCount = 1'000'000;
	}
	else
	{
		exit( 0 ); // distribution does not exist (yet).
	}
	// construct description
	switch (raySet)
	{
	case RaySet::PRIMARY_VIEW1: strncpy( desc, "primary rays cam1", 256 ); break;
	case RaySet::PRIMARY_VIEW2: strncpy( desc, "primary rays cam2", 256 ); break;
	case RaySet::PRIMARY_VIEW3: strncpy( desc, "primary rays cam3", 256 ); break;
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

};