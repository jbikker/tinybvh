RWTexture2D<float4> uav : register(u0);

[numthreads(8, 8, 1)]
void FillCircle(uint3 id : SV_DispatchThreadID)
{
    uint w, h;
    uav.GetDimensions(w, h);
    if (id.x >= w || id.y >= h)
        return;
    float2 center = float2(w, h) * 0.5;
    float radius = min(w, h) * 0.25;
    float2 p = float2(id.xy) - center;
    uav[id.xy] = (dot(p, p) < radius * radius) ? float4(1, 0, 0, 1) : float4(0, 0, 0, 1);
}