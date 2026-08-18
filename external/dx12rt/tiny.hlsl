// Software ray tracing compute shader, for BVH_GPU layout.
// Converted from GLSL and optimized by Claude.

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

#define STACK_SIZE 32

float4 traverse_ailalaine(const float3 O, const float3 D, const float3 rD,
                          const float3 Ord, const float tmax)
{
    float4 hit = float4(tmax, 0, 0, 0);
    uint node = 0, stack[STACK_SIZE], stackPtr = 0;
    while (true)
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
                const float3 vert0 = triData[v0].xyz;
                const float3 edge1 = triData[v0 + 1].xyz - vert0;
                const float3 edge2 = triData[v0 + 2].xyz - vert0;
                const float3 h = cross(D, edge2);
                const float a = dot(edge1, h);
                const float f = 1.0f / a;
                const float3 s = O - vert0;
                const float u = f * dot(s, h);
                const float3 q = cross(s, edge1);
                const float v = f * dot(D, q);
                const float d = f * dot(edge2, q);
                const bool valid = (u >= 0.0f) && (v >= 0.0f) && (u + v <= 1.0f) &&
                                   (d > 0.0f) && (d < hit.x);
                hit = valid ? float4(d, u, v, asfloat(triIdx)) : hit;
            }
            if (stackPtr == 0)
                break;
            node = stack[--stackPtr];
            continue;
        }
        const uint left = asuint(lmin.w), right = asuint(lmax.w);
        const float3 t1a = lmin.xyz * rD - Ord, t2a = lmax.xyz * rD - Ord;
        const float3 t1b = rmin.xyz * rD - Ord, t2b = rmax.xyz * rD - Ord;
        const float3 minta = min(t1a, t2a), maxta = max(t1a, t2a);
        const float3 mintb = min(t1b, t2b), maxtb = max(t1b, t2b);
        const float tmina = max(max(max(minta.x, minta.y), minta.z), 0);
        const float tminb = max(max(max(mintb.x, mintb.y), mintb.z), 0);
        const float tmaxa = min(min(min(maxta.x, maxta.y), maxta.z), hit.x);
        const float tmaxb = min(min(min(maxtb.x, maxtb.y), maxtb.z), hit.x);
        const bool hitL = tmina <= tmaxa, hitR = tminb <= tmaxb;
        if (hitL && hitR)
        {
            const bool leftFirst = tmina <= tminb;
            node = leftFirst ? left : right;
            const uint far = leftFirst ? right : left;
            if (stackPtr < STACK_SIZE)
                stack[stackPtr++] = far;
        }
        else if (hitL)
            node = left;
        else if (hitR)
            node = right;
        else
        {
            if (stackPtr == 0)
                break;
            node = stack[--stackPtr];
        }
    }
    return hit;
}

float safe_rcp(float x)
{
    return 1.0f / (sign(x) * max(abs(x), 1e-5f));
}

[numthreads(8, 8, 1)]
void TraceRays(uint3 tid : SV_DispatchThreadID)
{
    uint w, h;
    uav.GetDimensions(w, h);
    if (tid.x >= w || tid.y >= h)
        return; // out of bounds
    // read primary ray
    const RayData r = rayBuffer[tid.y * w + tid.x];
    const float3 O = r.origin, D = r.direction;
    const float3 rD = float3(safe_rcp(D.x), safe_rcp(D.y), safe_rcp(D.z));
    const float3 Ord = O * rD; // hoisted out of the traversal loop
    // trace
    const float4 hit = traverse_ailalaine(O, D, rD, Ord, 1e30f);
    // visualize depth
    const float t = (hit.x >= 1e30f) ? 0.0f : (1.0f - min(1.0f, hit.x * 0.01f));
    uav[tid.xy] = float4(t, t, t, 1);
}