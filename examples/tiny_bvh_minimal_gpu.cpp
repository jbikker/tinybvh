// Minimal GPU example for TinyBVH.

#define TINYBVH_IMPLEMENTATION
#include "tiny_bvh.h"
using namespace tinybvh;

// This application uses tinyocl - And this file will include the implementation.
#define TINY_OCL_IMPLEMENTATION
#include "tiny_ocl.h"

#include <cstdlib>
#include <cstdio>
#include <cstring>

// CPU-side data
static constexpr uint32_t TRIANGLE_COUNT = 8192;
static constexpr uint32_t RAY_COUNT = 1024;
static constexpr uint32_t GPU_RAY_SIZE = 64;
static bvhvec4 vertices[TRIANGLE_COUNT * 3]; // must be 16 byte!

// RNG convenience
float uniform_rand() { return (float)rand() / (float)RAND_MAX; }

// aplication entry point
int main()
{
	// create a scene consisting of some random small triangles
	for (uint32_t i = 0; i < TRIANGLE_COUNT; i++)
	{
		// create a random triangle
		bvhvec4& v0 = vertices[i * 3 + 0];
		bvhvec4& v1 = vertices[i * 3 + 1];
		bvhvec4& v2 = vertices[i * 3 + 2];
		// triangle position, x/y/z = 0..1
		float x = uniform_rand();
		float y = uniform_rand();
		float z = uniform_rand();
		// set first vertex
		v0.x = x + 0.1f * uniform_rand();
		v0.y = y + 0.1f * uniform_rand();
		v0.z = z + 0.1f * uniform_rand();
		// set second vertex
		v1.x = x + 0.1f * uniform_rand();
		v1.y = y + 0.1f * uniform_rand();
		v1.z = z + 0.1f * uniform_rand();
		// set third vertex
		v2.x = x + 0.1f * uniform_rand();
		v2.y = y + 0.1f * uniform_rand();
		v2.z = z + 0.1f * uniform_rand();
	}

	// build a BVH over the scene, in a format suitable for GPU
	tinybvh::BVH_GPU gpubvh;
	gpubvh.Build( vertices, TRIANGLE_COUNT );

	// load and compile the OpenCL kernel using tinyocl
	tinyocl::Kernel trace_kernel( "kernels/traverse.cl", "batch_nearest" );

	// create and populate GPU buffers
	// 1. triangle data, per tri: vertex 0 (with the original triangle index in w), edge 0, edge 1.
	//    Note that the indirection used in BVH doesn't happen in BVH_GPU.
	tinyocl::Buffer* triData = new tinyocl::Buffer( 
		gpubvh.idxCount * 3 * sizeof( bvhvec4 ),	// size (in bytes) of the buffer
		(bvhvec4*)gpubvh.orderedVerts.data,			// location of the data on the host
		tinyocl::Buffer::READONLY					// the device-side data will not be written to.
	);
	// 2. BVH node data: taken from gpubvh.bvhNode, count is gpubvh.usedNodes.
	//    If the tree is rebuilt per frame, use gpubvh.allocatedNodes instead.
	tinyocl::Buffer* gpuNodes = new tinyocl::Buffer( 
		gpubvh.usedNodes * sizeof( BVH_GPU::BVHNode ), 
		gpubvh.bvhNode, 
		tinyocl::Buffer::READONLY
	);
	// 3. ray buffer. We will always trace batches of rays, for efficiency.
	//    For GPU code, a ray is 64 bytes. On the CPU it has extra data, so copy carefully.
	tinyocl::Buffer* rayData = new tinyocl::Buffer( RAY_COUNT * GPU_RAY_SIZE );
	unsigned char* hostData = (unsigned char*)rayData->GetHostPtr();
	for (uint32_t i = 0; i < RAY_COUNT; i++)
	{
		bvhvec3 O( 0.5f, 0.5f, -1 );
		bvhvec3 D( 0.1f, uniform_rand() - 0.5f, 2 );
		Ray ray( O, D );
		memcpy( hostData + GPU_RAY_SIZE * i, &ray, GPU_RAY_SIZE /* just the first 64 bytes! */ );
	}
	// 4. data is created on CPU - sync to GPU.
	triData->CopyToDevice();
	gpuNodes->CopyToDevice();
	rayData->CopyToDevice();

	// invoke the kernel.
	trace_kernel.SetArguments( gpuNodes, triData, rayData );
	trace_kernel.Run( RAY_COUNT /* a thread per ray, ensure a multiple of 64. */ );

	// obtain traversal result.
	rayData->CopyFromDevice();
	for (uint32_t i = 0; i < RAY_COUNT; i++)
	{
		Ray ray;
		memcpy( &ray, hostData + GPU_RAY_SIZE * i, GPU_RAY_SIZE );
		if (ray.hit.t < BVH_FAR)
			printf( "ray %i hit prim %u at t=%f\n", i, ray.hit.prim, ray.hit.t );
		else
			printf( "ray %i, no hit\n", i );
	}

	// all done.
	delete triData;
	delete gpuNodes;
	delete rayData;
	return 0;
}