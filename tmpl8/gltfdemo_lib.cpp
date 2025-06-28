#include "precomp.h"

#define TINY_OCL_IMPLEMENTATION
#define TINY_OCL_GLINTEROP
#include "tiny_ocl.h"

#define TINYBVH_IMPLEMENTATION
#define TINYBVH_USE_CUSTOM_VECTOR_TYPES
#define NO_DOUBLE_PRECISION_SUPPORT
namespace tinybvh
{
using bvhint2 = int2;
using bvhint3 = int3;
using bvhuint2 = uint2;
using bvhuint3 = uint3;
using bvhuint4 = uint4;
using bvhvec2 = float2;
using bvhvec3 = float3;
using bvhvec4 = float4;
using bvhmat4 = mat4;
}
#include "tiny_bvh.h"

#define TINYSCENE_IMPLEMENTATION
#define TINYSCENE_USE_CUSTOM_VECTOR_TYPES
#define TINYSCENE_STBIMAGE_ALREADY_IMPLEMENTED
namespace tinyscene
{
using ts_int2 = int2;
using ts_int3 = int3;
using ts_uint2 = uint2;
using ts_uint3 = uint3;
using ts_uint4 = uint4;
using ts_vec2 = float2;
using ts_vec3 = float3;
using ts_vec4 = float4;
using ts_mat = mat4;
}
#include "tiny_scene.h"