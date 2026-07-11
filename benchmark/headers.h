#pragma once

#include "tiny_bvh.h"

// Settings
#define ENABLE_OPENCL // required for GPU traversal experiments.

using namespace std;
using namespace tinybvh;

#include <fstream>
#include <cstdlib>
#include <cstdio>

// Madmann91's BVH traversal
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

// Embree
#include "embree4/rtcore.h"

// Benchmark components
#include "primitive_set.h"
#include "ray_distribution.h"
#include "acc_struc.h"
#include "experiment.h"

// Low-level
#ifdef _WIN32
#include <intrin.h>		// for __cpuidex
#elif defined(__APPLE__) && defined(__MACH__)
// Keep ENABLE_OPENCL for APPLE
#elif defined ENABLE_OPENCL
#undef ENABLE_OPENCL
#endif
#if defined(__GNUC__) && defined(__x86_64__)
#include <cpuid.h>
#endif
#ifdef __EMSCRIPTEN__
#include <emscripten/version.h> // for __EMSCRIPTEN_major__, __EMSCRIPTEN_minor__
#endif
#ifdef ENABLE_OPENCL
#include "tiny_ocl.h"
#endif

// Forward declarations
void PrintHeader();
void InitOpenCL();