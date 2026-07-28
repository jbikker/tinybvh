struct Payload
{
    float t;
};

struct RayData
{
    float3 origin;
    float pad0;
    float3 direction;
    float pad1;
};

RaytracingAccelerationStructure scene : register(t0);
StructuredBuffer<RayData> rayBuffer : register(t1);
RWTexture2D<float4> uav : register(u0);

[shader("raygeneration")]void RayGeneration()
{
    uint2 idx = DispatchRaysIndex().xy;
    uint2 dims = DispatchRaysDimensions().xy;
    RayData rd = rayBuffer[idx.y * dims.x + idx.x];
    RayDesc ray;
    ray.Origin = rd.origin, ray.Direction = rd.direction;
    ray.TMin = 0.001, ray.TMax = 1000;
    Payload payload;
    TraceRay(scene, RAY_FLAG_FORCE_OPAQUE, 0xFF, 0, 0, 0, ray, payload);
    float t = 1 - min(1, payload.t * 0.01f);
    uav[idx] = float4(t, t, t, 1);
}
[shader("miss")]void Miss(inout Payload payload)
{
    payload.t = 1e30f;
}
[shader("closesthit")]void ClosestHit(inout Payload payload, BuiltInTriangleIntersectionAttributes attribs)
{
    payload.t = RayTCurrent();
}