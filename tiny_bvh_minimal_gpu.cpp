// Minimal GPU example for TinyBVH.

#define TINYBVH_IMPLEMENTATION
#include "tiny_bvh.h"
using namespace tinybvh;

// This application uses tinyocl - And this file will include the implementation.
#define TINY_OCL_IMPLEMENTATION
#include "tiny_ocl.h"

#include <cstdlib>
#include <cstdio>

#define TRIANGLE_COUNT	8192

// CPU-side data
bvhvec4 triangles[TRIANGLE_COUNT * 3]; // must be 16 byte!

// RNG convenience
float uniform_rand() { return (float)rand() / (float)RAND_MAX; }

// Application entry point
int main()
{
	// Create a scene consisting of some random small triangles.
	for (int i = 0; i < TRIANGLE_COUNT; i++)
	{
		// create a random triangle
		bvhvec4& v0 = triangles[i * 3 + 0];
		bvhvec4& v1 = triangles[i * 3 + 1];
		bvhvec4& v2 = triangles[i * 3 + 2];
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

	// Build a BVH over the scene. We use the BVH_GPU layout.
	tinybvh::BVH_GPU gpubvh;
	gpubvh.Build( triangles, TRIANGLE_COUNT );

	// Load and compile the OpenCL kernel.
	tinyocl::Kernel ailalaine_kernel( "traverse.cl", "batch_ailalaine" );

	// Create and populate the OpenCL buffers.
	// 1. Triangle data: For each triangle, 3 times a vec4 vertex.
	tinyocl::Buffer* triData = new tinyocl::Buffer( TRIANGLE_COUNT * 3 * sizeof( bvhvec4 ), triangles );
	// 2. BVH node data: Taken from gpubvh.bvhNode; count is gpubvh.usedNodes.
	//    If the tree is rebuilt per frame, use gpubvh.allocatedNodes instead.
	tinyocl::Buffer* gpuNodes = new tinyocl::Buffer( gpubvh.usedNodes * sizeof( BVH_GPU::BVHNode ), gpubvh.bvhNode );
	// 3. Triangle index data, used in BVH leafs. This is taken from the base BVH.
	tinyocl::Buffer* idxData = new tinyocl::Buffer( gpubvh.idxCount * sizeof( uint32_t ), gpubvh.bvh.primIdx );
	// 4. Ray buffer. We will always trace batches of rays, for efficiency.
	//    For GPU code, a ray is 64 bytes. On the CPU it has extra data, so copy carefully.
	tinyocl::Buffer* rayData = new tinyocl::Buffer( 1024 * 64 );
	unsigned char* hostData = (unsigned char*)rayData->GetHostPtr();
	for (int i = 0; i < 1024; i++)
	{
		bvhvec3 O( 0.5f, 0.5f, -1 );
		bvhvec3 D( 0.1f, uniform_rand() - 0.5f, 2 );
		Ray ray( O, D );
		memcpy( hostData + 64 * i, &ray, 64 /* just the first 64 bytes! */ );
	}
	// 5. Sync all data to the GPU. Repeat if anything changes.
	triData->CopyToDevice();
	gpuNodes->CopyToDevice();
	idxData->CopyToDevice();
	rayData->CopyToDevice();

	// Invoke the kernel.
	ailalaine_kernel.SetArguments( gpuNodes, idxData, triData, rayData );
	ailalaine_kernel.Run( 1024 /* a thread per ray, make it a multiple of 64. */ );

	// Obtain traversal result.
	rayData->CopyFromDevice();
	for (int i = 0; i < 1024; i++)
	{
		Ray ray;
		memcpy( &ray, hostData + 64 * i, 64 );
		printf( "ray %i, nearest intersection: %f\n", i, ray.hit.t );
	}

	// All done.
	delete triData;
	delete gpuNodes;
	delete idxData;
	delete rayData;
	return 0;
}