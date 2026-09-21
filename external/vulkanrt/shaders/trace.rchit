// Vulkan translation of the closesthit shader in shader.hlsl.

#version 460
#extension GL_EXT_ray_tracing : require

layout( location = 0 ) rayPayloadInEXT float payloadT;
hitAttributeEXT vec2 baryCoord; // built-in triangle attributes; unused here

void main()
{
	payloadT = gl_HitTEXT; // RayTCurrent()
}
