#pragma once

// Ray batches through the GPU driver's own acceleration structures, via Vulkan ray queries: the benchmark's
// reference layout on the GPU, as Embree and madmann91 are on the CPU. The Vulkan loader is opened at run
// time (it ships with the GPU driver) and external/vulkan/vk_min.h holds the declarations: no Vulkan SDK.

namespace tinybvh
{

bool InitVulkan( const char* gpuVendor = 0 ); // e.g. "AMD", "Intel", "NVIDIA"; 0: prefer a discrete GPU
bool VulkanAvailable();

class VulkanRT
{
public:
	VulkanRT();
	~VulkanRT();
	// Geometry for the next builds: 'primCount' triangles, 3 vertices each (16-byte stride), optionally indexed.
	// objectToWorld: row-major 3x4 transform of the TLAS instance (a rotated BLAS); 0: identity.
	void SetGeometry( const bvhvec4* verts, const uint32_t* indices, const uint32_t primCount, const float* objectToWorld = 0 );
	// BLAS (compacted) and a one-instance TLAS; quality 0: fast build, 1: default, 2: fast trace. Waits for the GPU.
	// The benchmark's build time for this layout is the wall time of this call: compaction, TLAS and allocations too.
	void Build( const int quality );
	// 'rayCount' 64-byte Ray records (O, D, tmax in hit.t): returns the fastest of 'runs' dispatches in seconds,
	// after 0.5 s of warm-up dispatches. hits: 4 floats per ray (t, u, v, prim bits) from the last dispatch, or 0.
	float Trace( const char* rayData, const int rayCount, const bool anyHit, const int runs, float* hits = 0 );
	uint64_t BlasBytes() const;
private:
	struct Impl;
	Impl* impl = 0;
};

} // namespace tinybvh

// EOF
