// Vulkan translation of the miss shader in shader.hlsl.

#version 460
#extension GL_EXT_ray_tracing : require

layout( location = 0 ) rayPayloadInEXT float payloadT;

void main()
{
	payloadT = 1e30f;
}
