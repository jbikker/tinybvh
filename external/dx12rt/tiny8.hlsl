// Software ray tracing compute shader, for BVH8_CWBVH layout.
// Converted from OpenCL code (traverse_cwbvh.cl) by Claude.
// Original CUDA code by AlanWBFT, https://github.com/AlanIWBFT

struct RayData
{
    float3 origin;
    float pad0;
    float3 direction;
    float pad1;
};

// --- resources -------------------------------------------------------------
StructuredBuffer<RayData> rayBuffer : register(t1); // space0: shared with the DXR path
StructuredBuffer<float4> cwbvhNodes : register(t4, space1); // 5x float4 per CWBVH node
StructuredBuffer<float4> cwbvhTris : register(t5, space1); // 3x (or 4x) float4 per triangle
RWTexture2D<float4> uav : register(u0); // render target

#define CWBVH_COMPRESSED_TRIS

#ifdef CWBVH_COMPRESSED_TRIS
#define TRI_STRIDE 4 // must match the tiny_bvh.h build define
#else
#define TRI_STRIDE 3
#endif

#define QUNPACK_BITTRICK 1
#if QUNPACK_BITTRICK
#define QUNPACK_SCALE 256.0f
#define QUNPACK_BIAS  1.0f
#else
#define QUNPACK_SCALE 1.0f
#define QUNPACK_BIAS  0.0f
#endif

#define GROUP_X    8
#define GROUP_Y    8
#define GROUP_SIZE (GROUP_X * GROUP_Y)

// Stack entries are node groups, not nodes. At most one group is pushed per level, 
// so this only has to cover tree depth. Measured peak: 6 at 450k primitives.
#define CWBVH_STACK_SIZE 8

// Depth-major, and split into two uint arrays rather than one uint2 array: a
// uint2 would give an 8-byte stride, landing all lanes at a given depth on
// every other LDS bank. This way consecutive lanes hit consecutive banks.
// 16 * 64 * 8 = 8KB per group, the same budget tiny.hlsl already uses.
groupshared uint g_stackX[CWBVH_STACK_SIZE * GROUP_SIZE];
groupshared uint g_stackY[CWBVH_STACK_SIZE * GROUP_SIZE];
#define STACK_PUSH(g) { const uint s_ = stackPtr++ * GROUP_SIZE + lane; \
                        g_stackX[s_] = (g).x, g_stackY[s_] = (g).y; }
#define STACK_POP(g)  { const uint s_ = --stackPtr * GROUP_SIZE + lane; \
                        (g) = uint2(g_stackX[s_], g_stackY[s_]); }

float safe_rcp(float x)
{
    return 1.0f / (sign(x) * max(abs(x), 1e-5f));
}

// Unpack four quantized child bounds.
float4 unpack_q8x4(const uint w)
{
#if QUNPACK_BITTRICK
    const uint m = 0x007F8000u, b = 0x3F800000u; // mantissa slot 22..15, exponent 0
    return asfloat((uint4(w << 15, w << 7, w >> 1, w >> 9) & m) | b);
#else
    return float4(uint4(w, w >> 8, w >> 16, w >> 24) & 255);
#endif
}

#define bfind(x) firstbithigh(x)

// slab test for one group of four children
uint intersect_quad(const uint meta4, const uint octinv4,
                    const float4 lox4, const float4 hix4,
                    const float4 loy4, const float4 hiy4,
                    const float4 loz4, const float4 hiz4,
                    const float3 idir, const float3 orig, const float tmax)
{
    const uint is_inner4 = (meta4 & (meta4 << 1)) & 0x10101010;
    // expand the 0x10 marker in each byte to 0xFF: (x << 4) - (x >> 4) is
    // 0x100 - 0x01 per marked byte, with no carry into the neighbours.
    const uint inner_mask4 = (is_inner4 << 4) - (is_inner4 >> 4);
    const uint bit_index4 = (meta4 ^ (octinv4 & inner_mask4)) & 0x1F1F1F1F;
    const uint child_bits4 = (meta4 >> 5) & 0x07070707;
    // intersect all four children at once
    const float4 tminx = lox4 * idir.xxxx + orig.xxxx, tmaxx = hix4 * idir.xxxx + orig.xxxx;
    const float4 tminy = loy4 * idir.yyyy + orig.yyyy, tmaxy = hiy4 * idir.yyyy + orig.yyyy;
    const float4 tminz = loz4 * idir.zzzz + orig.zzzz, tmaxz = hiz4 * idir.zzzz + orig.zzzz;
    const float4 cmin = max(max(max(tminx, tminy), tminz), 0.0f);
    const float4 cmax = min(min(min(tmaxx, tmaxy), tmaxz), tmax);
    // each surviving child contributes its child_bits, shifted to its slot
    uint hitmask = 0;
    if (cmin.x <= cmax.x)
        hitmask |= (child_bits4 & 255) << (bit_index4 & 31);
    if (cmin.y <= cmax.y)
        hitmask |= ((child_bits4 >> 8) & 255) << ((bit_index4 >> 8) & 31);
    if (cmin.z <= cmax.z)
        hitmask |= ((child_bits4 >> 16) & 255) << ((bit_index4 >> 16) & 31);
    if (cmin.w <= cmax.w)
        hitmask |= (child_bits4 >> 24) << (bit_index4 >> 24);
    return hitmask;
}

float4 traverse_cwbvh(const float3 O, const float3 D, const float3 rD, const float t,
                      const uint lane)
{
    // prepare traversal
    float4 hit = float4(t, 0, 0, 0); // returned unchanged if we miss everything
    uint hitAddr = 0, stackPtr = 0;
    float2 uv = float2(0, 0);
    float tmax = t;
    const uint octinv4 = (7 - ((D.x < 0 ? 4 : 0) | (D.y < 0 ? 2 : 0) | (D.z < 0 ? 1 : 0))) * 0x01010101;
    const float3 rDs = rD * QUNPACK_SCALE; // per-ray, hoisted out of the node loop
    // a 'group' is (base index, mask); the root is node 0 with only bit 31 set.
    uint2 ngroup = uint2(0, 0x80000000), tgroup = uint2(0, 0);
    while (true)
    {
        if (ngroup.y > 0x00FFFFFF)
        {
            // the node group still holds child nodes; pop the last one
            const uint imask = ngroup.y;
            const uint child_bit_index = bfind(ngroup.y);
            const uint child_node_base_index = ngroup.x;
            ngroup.y &= ~(1u << child_bit_index);
            if (ngroup.y > 0x00FFFFFF && stackPtr < CWBVH_STACK_SIZE)
                STACK_PUSH(ngroup); // there are siblings left; save them
            const uint slot_index = (child_bit_index - 24) ^ (octinv4 & 255);
            const uint relative_index = countbits(imask & ~(0xFFFFFFFFu << slot_index));
            const uint child_node_index = (child_node_base_index + relative_index) * 5;
            const float4 n0 = cwbvhNodes[child_node_index + 0];
            const float4 n1 = cwbvhNodes[child_node_index + 1];
            const float4 n2 = cwbvhNodes[child_node_index + 2];
            const float4 n3 = cwbvhNodes[child_node_index + 3];
            const float4 n4 = cwbvhNodes[child_node_index + 4];
            ngroup.x = asuint(n1.x), tgroup = uint2(asuint(n1.y), 0);
            // n0.w packs three signed per-axis exponents plus imask
            const uint packed_e = asuint(n0.w);
            const int3 e = int3(asint(packed_e << 24), asint(packed_e << 16),
                                asint(packed_e << 8)) >> 24; // arithmetic shift
            const float3 idir = float3(asfloat((e.x + 127) << 23),
                                       asfloat((e.y + 127) << 23),
                                       asfloat((e.z + 127) << 23)) * rDs;
            // dequantized bounds start at QUNPACK_BIAS, not 0; subtract it once
            // per node instead of per child.
            const float3 orig = (n0.xyz - O) * rD - QUNPACK_BIAS * idir;
            // children 0..3: quantized bounds live in the low bytes of n2/n3/n4
            uint hitmask = intersect_quad(asuint(n1.z), octinv4,
                unpack_q8x4(asuint(rD.x < 0 ? n3.z : n2.x)), unpack_q8x4(asuint(rD.x < 0 ? n2.x : n3.z)),
                unpack_q8x4(asuint(rD.y < 0 ? n4.x : n2.z)), unpack_q8x4(asuint(rD.y < 0 ? n2.z : n4.x)),
                unpack_q8x4(asuint(rD.z < 0 ? n4.z : n3.x)), unpack_q8x4(asuint(rD.z < 0 ? n3.x : n4.z)),
                idir, orig, tmax);
            // children 4..7: same, one float further into each pair
            hitmask |= intersect_quad(asuint(n1.w), octinv4,
                unpack_q8x4(asuint(rD.x < 0 ? n3.w : n2.y)), unpack_q8x4(asuint(rD.x < 0 ? n2.y : n3.w)),
                unpack_q8x4(asuint(rD.y < 0 ? n4.y : n2.w)), unpack_q8x4(asuint(rD.y < 0 ? n2.w : n4.y)),
                unpack_q8x4(asuint(rD.z < 0 ? n4.w : n3.y)), unpack_q8x4(asuint(rD.z < 0 ? n3.y : n4.w)),
                idir, orig, tmax);
            // split the hitmask: high byte selects child nodes, low 24 bits triangles
            ngroup.y = (hitmask & 0xFF000000) | (packed_e >> 24);
            tgroup.y = hitmask & 0x00FFFFFF;
        }
        else
            tgroup = ngroup, ngroup = uint2(0, 0);
        // process the triangles referenced by the current triangle group
        while (tgroup.y != 0)
        {
            const uint triangleIndex = bfind(tgroup.y);
            const uint triAddr = tgroup.x + triangleIndex * TRI_STRIDE;
            tgroup.y -= 1u << triangleIndex;
#ifdef CWBVH_COMPRESSED_TRIS
            // "Fast Ray-Triangle Intersections by Coordinate Transformation",
            // Baldwin & Weber, 2016.
            const float4 T2 = cwbvhTris[triAddr + 2];
            const float transS = T2.x * O.x + T2.y * O.y + T2.z * O.z + T2.w;
            const float transD = T2.x * D.x + T2.y * D.y + T2.z * D.z;
            const float d = -transS / transD;
            if (d <= 0 || d >= tmax)
                continue;
            const float4 T0 = cwbvhTris[triAddr + 0], T1 = cwbvhTris[triAddr + 1];
            const float3 I = O + d * D;
            const float u = T0.x * I.x + T0.y * I.y + T0.z * I.z + T0.w;
            const float v = T1.x * I.x + T1.y * I.y + T1.z * I.z + T1.w;
            if (u < 0 || v < 0 || u + v >= 1)
                continue;
            uv = float2(u, v), tmax = d;
            hitAddr = asuint(cwbvhTris[triAddr + 3].w);
#else
            // Moeller-Trumbore, iquilezles.org version. Tris are stored as 3x16 bytes: 
            // two edges plus vertex 0, whose w holds the original primitive index.
            const float3 e1 = cwbvhTris[triAddr].xyz;
            const float3 e2 = cwbvhTris[triAddr + 1].xyz;
            const float4 v0 = cwbvhTris[triAddr + 2];
            const float3 r = cross(D, e1);
            const float a = dot(e2, r);
            const float f = 1.0f / a;
            const float3 s = O - v0.xyz;
            const float u = f * dot(s, r);
            const float3 q = cross(s, e2);
            const float v = f * dot(D, q);
            if (u < 0 || v < 0 || u + v > 1)
                continue;
            const float d = f * dot(e1, q);
            if (d <= 0.0f || d >= tmax)
                continue;
            uv = float2(u, v), tmax = d;
            hitAddr = asuint(v0.w);
#endif
        }
        // out of child nodes: continue with the group on top of the stack
        if (ngroup.y <= 0x00FFFFFF)
        {
            if (stackPtr == 0)
            {
                hit = float4(tmax, uv.x, uv.y, asfloat(hitAddr));
                break;
            }
            STACK_POP(ngroup);
        }
    }
    return hit;
}

[numthreads(GROUP_X, GROUP_Y, 1)]
void TraceRays_8_wide(uint3 tid : SV_DispatchThreadID, uint gi : SV_GroupIndex)
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
    const float4 hit = traverse_cwbvh(O, D, rD, 1e30f, gi);
    // visualize depth
    const float t = (hit.x >= 1e30f) ? 0.0f : (1.0f - min(1.0f, hit.x * 0.01f));
    uav[tid.xy] = float4(t, t, t, 1);
}