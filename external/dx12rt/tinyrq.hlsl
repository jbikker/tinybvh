// Hardware ray tracing compute shader, using DXR 1.1 inline ray tracing.
// This is a reference backend.

struct RayData
{
    float3 origin;
    float pad0;
    float3 direction;
    float pad1;
};

// --- resources -------------------------------------------------------------
// t0 space0 is the same TLAS root SRV the DXR path binds, so this backend needs
// no additions to the root signature.
RaytracingAccelerationStructure sceneTLAS : register(t0);
StructuredBuffer<RayData> rayBuffer : register(t1); // space0: shared with the DXR path
RWTexture2D<float4> uav : register(u0); // render target

[numthreads(8, 8, 1)]
void TraceRays_Inline(uint3 tid : SV_DispatchThreadID)
{
    uint w, h;
    uav.GetDimensions(w, h);
    if (tid.x >= w || tid.y >= h)
        return; // out of bounds
    // get primary ray
    const RayData r = rayBuffer[tid.y * w + tid.x];
    RayDesc ray;
    ray.Origin = r.origin;
    ray.Direction = r.direction;
    ray.TMin = 0.0f;
    ray.TMax = 1e30f;
    // trace.
    RayQuery<RAY_FLAG_FORCE_OPAQUE> q;
    q.TraceRayInline(sceneTLAS, RAY_FLAG_NONE, 0xFF, ray);
    q.Proceed();
    const float d = (q.CommittedStatus() == COMMITTED_TRIANGLE_HIT) ? q.CommittedRayT() : 1e30f;
    // visualize depth
    const float t = (d >= 1e30f) ? 0.0f : (1.0f - min(1.0f, d * 0.01f));
    uav[tid.xy] = float4(t, t, t, 1);
}
