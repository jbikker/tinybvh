#include <iostream>
#include "NXB/BVHBuilder.h"
#include "CudaUtils.h"
#include <vector>
#include <fstream>

static float4* tris = 0;
static int triCount = 0;
void AddMesh( const char* file, float scale = 1, float3 pos = {}, int c = 0, int N = 0 )
{
	std::fstream s{ file, s.binary | s.in }; s.read( (char*)&N, 4 );
	float4* data = (float4*)malloc( (N + triCount) * 48 );
	if (tris) memcpy( data, tris, triCount * 48 ), free( tris );
	tris = data, s.read( (char*)tris + triCount * 48, N * 48 ), triCount += N;
	for (int* b = (int*)tris + (triCount - N) * 12, i = 0; i < N * 3; i++)
		*(float3*)b = *(float3*)b * scale + pos, b[3] = c ? c : b[3], b += 4;
}

struct TinyBVHVerboseNode
{
	float3 aabbMin; uint32_t left;
	float3 aabbMax; uint32_t right;
	uint32_t triCount, firstTri, parent;
	float dummy[5]; // total: 64 bytes.
};

int main( void )
{
	AddMesh( "../../testdata/bistro_ext_part1.bin", 1, float3(), 0xffffff );
	AddMesh( "../../testdata/bistro_ext_part2.bin", 1, float3(), 0xffffff );
	std::vector<NXB::Triangle> triangles( triCount );
	NXB::AABB bounds;
	bounds.Clear();
	for (size_t i = 0; i < triCount; ++i)
	{
		float3 v0 = make_float3( tris[i * 3 + 0].x, tris[i * 3 + 0].y, tris[i * 3 + 0].z );
		float3 v1 = make_float3( tris[i * 3 + 1].x, tris[i * 3 + 1].y, tris[i * 3 + 1].z );
		float3 v2 = make_float3( tris[i * 3 + 2].x, tris[i * 3 + 2].y, tris[i * 3 + 2].z );
		triangles[i] = { v0, v1, v2 };
		NXB::AABB primBounds( v0, v1, v2 );
		bounds.Grow( primBounds );
	}

	// build a BVH2 using H-PLOC on GPU
	NXB::Triangle* dTriangles = CudaMemory::Allocate<NXB::Triangle>( triCount );
	CudaMemory::Copy<NXB::Triangle>( dTriangles, triangles.data(), triCount, cudaMemcpyHostToDevice );
	NXB::BuildConfig buildConfig;
	buildConfig.prioritizeSpeed = false;
	NXB::BVH2 deviceBVH2 = NXB::BuildBinary<NXB::Triangle>( dTriangles, triCount, buildConfig );
	NXB::BVH2 hostBVH2 = NXB::ToHost( deviceBVH2 );

	// modify the BVH to TinyBVH's Verbose layout
	TinyBVHVerboseNode* finalNodes = new TinyBVHVerboseNode[hostBVH2.nodeCount];
	memset( finalNodes, 0, hostBVH2.nodeCount * 64 );
	int lastNode = hostBVH2.nodeCount - 1;
	for( int i = 0; i <= lastNode; i++ )
	{
		NXB::BVH2::Node& src = hostBVH2.nodes[i];
		int dstIdx = (i == lastNode) ? 0 : (i + 1);
		TinyBVHVerboseNode& dst = finalNodes[dstIdx];
		if (src.leftChild == INVALID_IDX)
		{
			dst.triCount = 1;
			dst.firstTri = src.rightChild;
		}
		else
		{
			dst.triCount = 0;
			dst.left = src.leftChild + 1;
			dst.right = src.rightChild + 1;
		}
		dst.aabbMin = src.bounds.bMin;
		dst.aabbMax = src.bounds.bMax;
	}

	// write the constructed BVH2
	FILE* f = fopen( "../../hploc.bin", "wb" );
	printf( "writing %i nodes... (%i on device)\n", hostBVH2.nodeCount, deviceBVH2.nodeCount );
	fwrite( &hostBVH2.bounds, 1, sizeof( NXB::AABB ), f );
	fwrite( &hostBVH2.nodeCount, 1, 4, f );
	fwrite( finalNodes, 64, hostBVH2.nodeCount, f );
	fclose( f );

	// all done.
	CudaMemory::Free( dTriangles );
	return EXIT_SUCCESS;
}