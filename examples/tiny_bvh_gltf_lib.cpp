// This file includes the implementations of the various libraries
// and exists purely to speedup compilation.

#define FENSTER_APP_IMPLEMENTATION
#define SCRWIDTH 800
#define SCRHEIGHT 600
#include "external/fenster.h" // https://github.com/zserge/fenster

#define TINYBVH_IMPLEMENTATION
#include "tiny_bvh.h"

#define TINYSCENE_IMPLEMENTATION
#define TINYSCENE_USE_CUSTOM_VECTOR_TYPES
namespace tinyscene // override tinyscene's vector types with tinybvh's for easier interop
{
using ts_int2 = tinybvh::bvhint2;
using ts_int3 = tinybvh::bvhint3;
using ts_uint2 = tinybvh::bvhuint2;
using ts_uint3 = tinybvh::bvhuint3;
using ts_uint4 = tinybvh::bvhuint4;
using ts_vec2 = tinybvh::bvhvec2;
using ts_vec3 = tinybvh::bvhvec3;
using ts_vec4 = tinybvh::bvhvec4;
using ts_mat4 = tinybvh::bvhmat4;
}
#include "tiny_scene.h" // very much in beta.