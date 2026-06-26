#pragma once

#include "tiny_bvh.h"

using namespace std;
using namespace tinybvh;

#include <fstream>
#include <cstdlib>
#include <cstdio>

// madmann91's BVH traversal
#include "bvh/v2/bvh.h"
#include "bvh/v2/vec.h"
#include "bvh/v2/ray.h"
#include "bvh/v2/node.h"
#include "bvh/v2/default_builder.h"
#include "bvh/v2/thread_pool.h"
#include "bvh/v2/stack.h"
#include "bvh/v2/tri.h"
#include "bvh/v2/sphere.h"
using _Vec3 = bvh::v2::Vec<float, 3>;
using _BBox = bvh::v2::BBox<float, 3>;
using _Tri = bvh::v2::Tri<float, 3>;
using _Node = bvh::v2::Node<float, 3>;
using _Bvh = bvh::v2::Bvh<_Node>;
using _Ray = bvh::v2::Ray<float, 3>;
using PrecomputedTri = bvh::v2::PrecomputedTri<float>;

#include "primitive_set.h"
#include "ray_distribution.h"
#include "acc_struc.h"
#include "experiment.h"