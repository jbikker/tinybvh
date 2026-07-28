struct Settings
{
    float eye_x, eye_y, eye_z;
    float p1_x, p1_y, p1_z;
    float p2_x, p2_y, p2_z;
    float p3_x, p3_y, p3_z;
};

struct Payload
{
    float t;
};

RaytracingAccelerationStructure scene : register(t0);
ConstantBuffer<Settings> settings : register(b0);
RWTexture2D<float4> uav : register(u0);

[shader("raygeneration")]void RayGeneration()
{
    uint2 idx = DispatchRaysIndex().xy;
    float2 size = DispatchRaysDimensions().xy;
    float2 uv = idx / size;
    float3 E = float3(settings.eye_x, settings.eye_y, settings.eye_z);
    float3 p1 = float3(settings.p1_x, settings.p1_y, settings.p1_z);
    float3 p2 = float3(settings.p2_x, settings.p2_y, settings.p2_z);
    float3 p3 = float3(settings.p3_x, settings.p3_y, settings.p3_z);
    float3 P = p1 + uv.x * (p2 - p1) + uv.y * (p3 - p1);
    RayDesc ray;
    ray.Origin = E, ray.Direction = normalize(P - E);
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