// Software ray tracing compute shader, converted from GLSL by Opus 4.8.
//
// Consumes three read-only buffers produced using tinybvh's BVH_GPU layout -
// see tiny_bvh.h, class BVH_GPU / struct BVHNode:
//
//   bvhNodes : BVH_GPU::bvhNode[]        (Aila-Laine 64-byte nodes)  t0, space1
//   primIdx  : BVH_GPU::bvh.primIdx[]    (one uint per primitive)    t1, space1
//   triData  : vertex positions          (3x float4 per triangle)    t2, space1
//
// Rays are read from the program's existing per-pixel ray buffer (the same
// StructuredBuffer<RayData> that the DXR path in shader.hlsl uses), so the
// traced image matches the camera set up on the CPU.
//
// Output is written to the render-target UAV as a simple depth visualization
// so the result is visible through the existing swap-chain copy path.

struct BVHNode
{
    float4 lmin; // .xyz = left child aabb min,  asuint(.w) = left  child node index
    float4 lmax; // .xyz = left child aabb max,  asuint(.w) = right child node index
    float4 rmin; // .xyz = right child aabb min, asuint(.w) = triCount (>0 => leaf)
    float4 rmax; // .xyz = right child aabb max, asuint(.w) = firstTri (index into primIdx)
};

struct RayData
{
    float3 origin;
    float pad0;
    float3 direction;
    float pad1;
};

// --- resources -------------------------------------------------------------
StructuredBuffer<RayData> rayBuffer : register(t1); // space0: shared with the DXR path
StructuredBuffer<BVHNode> bvhNodes : register(t0, space1); // tinybvh BVH_GPU nodes
StructuredBuffer<uint> primIdx : register(t1, space1); // tinybvh primitive index array
StructuredBuffer<float4> triData : register(t2, space1); // 3x float4 per triangle
RWTexture2D<float4> uav : register(u0); // render target

// Direct port of traverse_ailalaine() from traverse.comp.
float4 traverse_ailalaine(const float3 O, const float3 D, const float3 rD, const float tmax)
{
    // traverse BVH
    float4 hit;
    hit.x = tmax;
    uint node = 0, stack[32], stackPtr = 0;
    while (true) // for (uint guard = 0; guard < 256; guard++) // cap at 512 traversal steps
    {
        const BVHNode n = bvhNodes[node];
        const float4 lmin = n.lmin, lmax = n.lmax;
        const float4 rmin = n.rmin, rmax = n.rmax;
        const uint triCount = asuint(rmin.w);
        if (triCount > 0)
        {
            const uint firstTri = asuint(rmax.w);
            for (uint i = 0; i < triCount; i++)
            {
                const uint triIdx = primIdx[firstTri + i];
                const uint v0 = triIdx * 3;
                const float4 vert0 = triData[v0];
                const float4 edge1 = triData[v0 + 1] - vert0;
                const float4 edge2 = triData[v0 + 2] - vert0;
                const float3 h = cross(D, edge2.xyz);
                const float a = dot(edge1.xyz, h);
                if (abs(a) < 0.0000001f)
                    continue;
                const float f = 1.0f / a;
                const float3 s = O - vert0.xyz;
                const float u = f * dot(s, h);
                const float3 q = cross(s, edge1.xyz);
                const float v = f * dot(D, q);
                if (u < 0 || v < 0 || u + v > 1)
                    continue;
                const float d = f * dot(edge2.xyz, q);
                if (d > 0.0f && d < hit.x)
                    hit = float4(d, u, v, asfloat(triIdx));
            }
            if (stackPtr == 0)
                break;
            node = stack[--stackPtr];
            continue;
        }
        uint left = asuint(lmin.w), right = asuint(lmax.w);
        const float3 t1a = (lmin.xyz - O) * rD, t2a = (lmax.xyz - O) * rD;
        const float3 t1b = (rmin.xyz - O) * rD, t2b = (rmax.xyz - O) * rD;
        const float3 minta = min(t1a, t2a), maxta = max(t1a, t2a);
        const float3 mintb = min(t1b, t2b), maxtb = max(t1b, t2b);
        const float tmina = max(max(max(minta.x, minta.y), minta.z), 0);
        const float tminb = max(max(max(mintb.x, mintb.y), mintb.z), 0);
        const float tmaxa = min(min(min(maxta.x, maxta.y), maxta.z), hit.x);
        const float tmaxb = min(min(min(maxtb.x, maxtb.y), maxtb.z), hit.x);
        float dist1 = tmina > tmaxa ? 1e30f : tmina;
        float dist2 = tminb > tmaxb ? 1e30f : tminb;
        if (dist1 > dist2)
        {
            float t = dist1;
            dist1 = dist2, dist2 = t;
            uint ti = left;
            left = right, right = ti;
        }
        if (dist1 == 1e30f)
        {
            if (stackPtr == 0)
                break;
            else
                node = stack[--stackPtr];
        }
        else
        {
            node = left;
            if (dist2 != 1e30f && stackPtr < 32)
                stack[stackPtr++] = right;
        }
    }
    return hit;
}

float safe_rcp(float x)
{
    return 1.0f / (sign(x) * max(abs(x), 1e-5f));
}

[numthreads(8, 8, 1)]void TraceRays(uint3 tid : SV_DispatchThreadID)
{
    uint w, h;
    uav.GetDimensions(w, h);
    if (tid.x >= w || tid.y >= h)
        return; // out of bounds
    // primary ray for this pixel (built on the CPU into rayBuffer)
    const uint id = tid.y * w + tid.x;
    const RayData r = rayBuffer[id];
    const float3 O = r.origin;
    const float3 D = r.direction;
    const float3 rD = float3(safe_rcp(D.x), safe_rcp(D.y), safe_rcp(D.z));
    // trace
    const float4 hit = traverse_ailalaine(O, D, rD, 1e30f);
    // simple depth visualization (miss -> black), same style as shader.hlsl
    const float t = (hit.x >= 1e30f) ? 0.0f : (1.0f - min(1.0f, hit.x * 0.01f));
    uav[tid.xy] = float4(t, t, t, 1);
}