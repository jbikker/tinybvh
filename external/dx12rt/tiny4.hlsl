// Software ray tracing compute shader, for BVH4_GPU layout.
// Converted from OpenCL code by Claude.

struct RayData
{
    float3 origin;
    float pad0;
    float3 direction;
    float pad1;
};

// --- resources -------------------------------------------------------------
StructuredBuffer<RayData> rayBuffer : register(t1); // space0: shared with the DXR path
StructuredBuffer<float4> bvh4Data : register(t3, space1); // tinybvh BVH4_GPU blob (nodes + tris)
RWTexture2D<float4> uav : register(u0); // render target

#ifdef BVH4_GPU_COMPRESSED_TRIS
#define STRIDE 4 // must match the tiny_bvh.h build define
#else
#define STRIDE 3
#endif

#define BVH4_STACK_SIZE 64 // need larger stack than BVH2.

// robust reciprocal calculation
float safe_rcp(float x)
{
    return 1.0f / (sign(x) * max(abs(x), 1e-5f));
}

// OpenCL as_uchar4( float ): the four bytes of the bit pattern, low byte first.
float4 as_uchar4(const float v)
{
    const uint u = asuint(v);
    return float4(uint4(u, u >> 8, u >> 16, u >> 24) & 255);
}

// Triangles are stored by value in the BVH4_GPU blob, one 'vertIdx' being an
// index in float4s into that blob.
void IntersectTri4(const uint vertIdx, const float3 O, const float3 D, inout float4 hit)
{
#ifdef BVH4_GPU_COMPRESSED_TRIS
    const float4 T2 = bvh4Data[vertIdx + 2];
    const float transS = T2.x * O.x + T2.y * O.y + T2.z * O.z + T2.w;
    const float transD = T2.x * D.x + T2.y * D.y + T2.z * D.z;
    const float d = -transS / transD;
    if (d <= 0 || d >= hit.x)
        return;
    const float4 T0 = bvh4Data[vertIdx + 0], T1 = bvh4Data[vertIdx + 1];
    const float3 I = O + d * D;
    const float u = T0.x * I.x + T0.y * I.y + T0.z * I.z + T0.w;
    const float v = T1.x * I.x + T1.y * I.y + T1.z * I.z + T1.w;
    if (u >= 0 && v >= 0 && u + v < 1)
        hit = float4(d, u, v, bvh4Data[vertIdx + 3].w);
#else
    const float4 vert0 = bvh4Data[vertIdx]; // .w = original triangle index
    const float4 edge1 = bvh4Data[vertIdx + 1]; // pre-subtracted by ConvertFrom
    const float4 edge2 = bvh4Data[vertIdx + 2];
    const float3 h = cross(D, edge2.xyz);
    const float a = dot(edge1.xyz, h);
    const float f = 1.0f / a;
    const float3 s = O - vert0.xyz;
    const float u = f * dot(s, h);
    const float3 q = cross(s, edge1.xyz);
    const float v = f * dot(D, q);
    const float d = f * dot(edge2.xyz, q);
    const bool valid = (u >= 0.0f) && (v >= 0.0f) && (u + v <= 1.0f) &&
        (d > 0.0f) && (d < hit.x);
    hit = valid ? float4(d, u, v, vert0.w) : hit;
#endif
}

float4 traverse_gpu4way(const float3 O, const float3 D, const float3 rD, const float tmax)
{
    float4 hit = float4(tmax, 0, 0, 0);
    uint offset = 0, stack[BVH4_STACK_SIZE], stackPtr = 0;
    while (true) // for (uint guard = 0; guard < BVH4_MAX_STEPS; guard++)
    {
        // vectorized 4-wide quantized aabb intersection
        const float4 data0 = bvh4Data[offset], data1 = bvh4Data[offset + 1];
        const float4 data2 = bvh4Data[offset + 2];
        uint4 data3 = asuint(bvh4Data[offset + 3]);
        const float3 bminO = (O - data0.xyz) * rD, rDe = rD * data1.xyz;
        float4 dst4 = float4(0, 0, 0, 0), tmax4 = hit.xxxx;
        // x axis - decode, use, discard
        {
            const float4 t1 = as_uchar4(data0.w) * rDe.xxxx - bminO.xxxx;
            const float4 t2 = as_uchar4(data1.w) * rDe.xxxx - bminO.xxxx;
            dst4 = max(dst4, min(t1, t2)), tmax4 = min(tmax4, max(t1, t2));
        }
        // y axis
        {
            const float4 t1 = as_uchar4(data2.x) * rDe.yyyy - bminO.yyyy;
            const float4 t2 = as_uchar4(data2.y) * rDe.yyyy - bminO.yyyy;
            dst4 = max(dst4, min(t1, t2)), tmax4 = min(tmax4, max(t1, t2));
        }
        // z axis
        {
            const float4 t1 = as_uchar4(data2.z) * rDe.zzzz - bminO.zzzz;
            const float4 t2 = as_uchar4(data2.w) * rDe.zzzz - bminO.zzzz;
            dst4 = max(dst4, min(t1, t2)), tmax4 = min(tmax4, max(t1, t2));
        }
        // 1e30 == 'child not intersected'. select() needs HLSL 2021; both operands
        // are spelled out as float4 so no scalar promotion is required.
        dst4 = select(dst4 > tmax4, float4(1e30f, 1e30f, 1e30f, 1e30f), dst4);
        // sort intersection distances, farthest first
        // bertdobbelaere.github.io/sorting_networks.html
        if (dst4.x < dst4.z)
        {
            dst4 = dst4.zyxw;
            data3 = data3.zyxw;
        }
        if (dst4.y < dst4.w)
        {
            dst4 = dst4.xwzy;
            data3 = data3.xwzy;
        }
        if (dst4.x < dst4.y)
        {
            dst4 = dst4.yxzw;
            data3 = data3.yxzw;
        }
        if (dst4.z < dst4.w)
        {
            dst4 = dst4.xywz;
            data3 = data3.xywz;
        }
        if (dst4.y < dst4.z)
        {
            dst4 = dst4.xzyw;
            data3 = data3.xzyw;
        }
        uint nextNode = 0;
        if (dst4.x < hit.x)
        {
            if ((data3.x >> 31) == 0)
                nextNode = data3.x; // nextNode is still 0 here, so nothing to push
            else
            {
                // leaf: tri data offset is relative to THIS node, hence '+ offset'
                const uint triCount = (data3.x >> 16) & 0x7fff;
                for (uint i = 0; i < triCount; i++)
                    IntersectTri4((data3.x & 0xffff) + offset + i * STRIDE, O, D, hit);
            }
        }
        if (dst4.y < hit.x)
        {
            if (data3.y >> 31)
            {
                const uint triCount = (data3.y >> 16) & 0x7fff;
                for (uint i = 0; i < triCount; i++)
                    IntersectTri4((data3.y & 0xffff) + offset + i * STRIDE, O, D, hit);
            }
            else
            {
                if (nextNode && stackPtr < BVH4_STACK_SIZE)
                    stack[stackPtr++] = nextNode;
                nextNode = data3.y;
            }
        }
        if (dst4.z < hit.x)
        {
            if (data3.z >> 31)
            {
                const uint triCount = (data3.z >> 16) & 0x7fff;
                for (uint i = 0; i < triCount; i++)
                    IntersectTri4((data3.z & 0xffff) + offset + i * STRIDE, O, D, hit);
            }
            else
            {
                if (nextNode && stackPtr < BVH4_STACK_SIZE)
                    stack[stackPtr++] = nextNode;
                nextNode = data3.z;
            }
        }
        if (dst4.w < hit.x)
        {
            if (data3.w >> 31)
            {
                const uint triCount = (data3.w >> 16) & 0x7fff;
                for (uint i = 0; i < triCount; i++)
                    IntersectTri4((data3.w & 0xffff) + offset + i * STRIDE, O, D, hit);
            }
            else
            {
                if (nextNode && stackPtr < BVH4_STACK_SIZE)
                    stack[stackPtr++] = nextNode;
                nextNode = data3.w;
            }
        }
        // continue with the nearest node, or the first node on the stack
        if (nextNode)
            offset = nextNode;
        else
        {
            if (stackPtr == 0)
                break;
            offset = stack[--stackPtr];
        }
    }
    return hit;
}

[numthreads(8, 8, 1)]void TraceRays_4_wide(uint3 tid : SV_DispatchThreadID)
{
    uint w, h;
    uav.GetDimensions(w, h);
    if (tid.x >= w || tid.y >= h)
        return; // out of bounds
    // get primary ray
    const uint id = tid.y * w + tid.x;
    const RayData r = rayBuffer[id];
    const float3 O = r.origin;
    const float3 D = r.direction;
    const float3 rD = float3(safe_rcp(D.x), safe_rcp(D.y), safe_rcp(D.z));
    // trace
    const float4 hit = traverse_gpu4way(O, D, rD, 1e30f);
    // visualize depth
    const float t = (hit.x >= 1e30f) ? 0.0f : (1.0f - min(1.0f, hit.x * 0.01f));
    uav[tid.xy] = float4(t, t, t, 1);
}
