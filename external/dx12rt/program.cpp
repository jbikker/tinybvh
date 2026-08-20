// heavily based on Laura Andelare's great DXR tutorial:
// https://landelare.github.io/2023/02/18/dxr-tutorial.html
// Sonnet 5 and Claude helped (a lot) with extensions and improvements.

#define NOMINMAX
#include <DirectXMath.h> // for XMMATRIX
#include <d3d12.h>
#include <dxgi1_6.h>
#include "shader.fxh"
#include "tiny.fxh"
#include "tiny4.fxh"
#include "tiny8.fxh"
#include <cassert>
#include <fstream>

extern "C" { __declspec(dllexport) DWORD NvOptimusEnablement = 1; }
extern "C" { __declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1; }

#pragma comment(lib, "user32") // for DefWindowProcW, etc.
#pragma comment(lib, "d3d12")
#pragma comment(lib, "dxgi")

constexpr UINT rtWidth = 1024, rtHeight = 1024;
constexpr DXGI_SAMPLE_DESC NO_AA = { .Count = 1, .Quality = 0 };
constexpr D3D12_HEAP_PROPERTIES UPLOAD_HEAP = { .Type = D3D12_HEAP_TYPE_UPLOAD };
constexpr D3D12_HEAP_PROPERTIES DEFAULT_HEAP = { .Type = D3D12_HEAP_TYPE_DEFAULT };
constexpr D3D12_RESOURCE_DESC BASIC_BUFFER_DESC = {
	.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER, .Width = 0, // will be changed in copies
	.Height = 1, .DepthOrArraySize = 1, .MipLevels = 1, .SampleDesc = NO_AA,
	.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR
};
constexpr UINT NUM_INSTANCES = 1, FRAME_COUNT = 2;
constexpr UINT64 NUM_SHADER_IDS = 3;
constexpr UINT NUM_DISPATCHES = 20;

IDXGIFactory6* factory;
ID3D12Device5* device;
ID3D12CommandQueue* cmdQueue;
ID3D12Fence* fence;
IDXGISwapChain3* swapChain;
ID3D12DescriptorHeap* uavHeap;
ID3D12Resource* renderTarget, * backBuffer, * meshVB, * blas, * instances, * shaderIDs;
ID3D12Resource* tlas, * tlasUpdateScratch, * queryResultBuffer, * compactionSizeReadback, * rayBuffer = nullptr;
ID3D12CommandAllocator* cmdAllocs[FRAME_COUNT]; // one allocator per frame-in-flight slot
ID3D12GraphicsCommandList4* cmdList;
D3D12_RAYTRACING_INSTANCE_DESC* instanceData;
ID3D12RootSignature* rootSignature;
ID3D12StateObject* pso;
ID3D12PipelineState* computePso, * computePso4, * computePso8;
ID3D12QueryHeap* queryHeap;
HANDLE fenceEvent = nullptr;

// The ray tracing backends that are cycled through, one per frame.
enum Backend { BACKEND_DXR = 0, BACKEND_BVH_GPU, BACKEND_BVH4_GPU, BACKEND_BVH8_CWBVH, BACKEND_COUNT };
static const char* backendName[BACKEND_COUNT] = { "DXR", "BVH_GPU", "BVH4_GPU", "CWBVH" };

#define TINYBVH_IMPLEMENTATION
#include "../../tiny_bvh.h"
using namespace tinybvh;

// One high-quality (SBVH) BVH2 is built; BVH_GPU and BVH4_GPU are derived from it.
BVH bvh2;
BVH_GPU bvh;
MBVH<4> mbvh4;
BVH4_GPU bvh4;
BVH8_CWBVH cwbvh;

// tinybvh BVH_GPU node.
struct GPUBVHNode
{
	bvhvec3 lmin; unsigned left;
	bvhvec3 lmax; unsigned right;
	bvhvec3 rmin; unsigned triCount;
	bvhvec3 rmax; unsigned firstTri; // total: 64 bytes
};

// Software-RT compute inputs.
ID3D12Resource* bvhNodeBuffer = nullptr, * bvhIdxBuffer = nullptr, * bvhVertBuffer = nullptr;
ID3D12Resource* bvh4DataBuffer = nullptr;
ID3D12Resource* cwbvhNodeBuffer = nullptr, * cwbvhTriBuffer = nullptr;

// Scene management - Append a file, with optional position, scale and color override, tinyfied
int triCount = 0;
bvhvec4* verts = 0;
void AddMesh( const char* file, int N = 0 )
{
	std::fstream s{ file, s.binary | s.in };
	s.read( (char*)&N, 4 );
	bvhvec4* data = (bvhvec4*)_aligned_malloc( (N + triCount) * 48, 64 );
	if (verts) memcpy( data, verts, triCount * 48 ), _aligned_free( verts );
	verts = data, s.read( (char*)verts + triCount * 48, N * 48 ), triCount += N;
}

static UINT64 fenceValue = 0;
static UINT64 frameFenceValues[FRAME_COUNT] = {};

void WaitForGpu()
{
	fenceValue++;
	const UINT64 fenceValueToWait = fenceValue;
	cmdQueue->Signal( fence, fenceValueToWait );
	if (fence->GetCompletedValue() >= fenceValueToWait) return;
	fence->SetEventOnCompletion( fenceValueToWait, fenceEvent );
	WaitForSingleObject( fenceEvent, INFINITE );
}

void UpdateRayBuffer()
{
    if (rayBuffer) rayBuffer->Release();
    D3D12_RESOURCE_DESC desc = BASIC_BUFFER_DESC;
    desc.Width = 32 * rtWidth * rtHeight;
    // CPU-fillable UPLOAD staging buffer.
    ID3D12Resource* staging = nullptr;
    device->CreateCommittedResource( &UPLOAD_HEAP, D3D12_HEAP_FLAG_NONE, &desc,
        D3D12_RESOURCE_STATE_GENERIC_READ, nullptr, IID_PPV_ARGS( &staging ) );
    float* data;
    staging->Map( 0, nullptr, reinterpret_cast<void**>(&data) );
	FILE* f = fopen( "raysets/diffrays.bin", "rb" );
	memset( data, 0, 32 * 1024 * 1024 );
	for( int i = 0; i < rtWidth * rtHeight; i++ ) fread( data + i * 8, 4, 3, f );
	for( int i = 0; i < rtWidth * rtHeight; i++ ) fread( data + i * 8 + 4, 4, 3, f );
	staging->Unmap( 0, nullptr );
    // DEVICE-LOCAL (VRAM) buffer the shaders actually read from on every ray.
    device->CreateCommittedResource( &DEFAULT_HEAP, D3D12_HEAP_FLAG_NONE, &desc,
        D3D12_RESOURCE_STATE_COPY_DEST, nullptr, IID_PPV_ARGS( &rayBuffer ) );
    rayBuffer->SetName( L"rayBuffer" );
    // One-shot copy staging -> VRAM, then flip to a shader-readable state.
    cmdAllocs[0]->Reset();
    cmdList->Reset( cmdAllocs[0], nullptr );
    cmdList->CopyResource( rayBuffer, staging );
    D3D12_RESOURCE_BARRIER rb = { .Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION,
        .Transition = { .pResource = rayBuffer,
            .StateBefore = D3D12_RESOURCE_STATE_COPY_DEST,
            .StateAfter = D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE } };
    cmdList->ResourceBarrier( 1, &rb );
    cmdList->Close();
    cmdQueue->ExecuteCommandLists( 1, reinterpret_cast<ID3D12CommandList**>(&cmdList) );
    WaitForGpu();
    staging->Release();
}

void InitBVHBuffers()
{
	// UPLOAD-heap staging: CPU-fillable, released once the copy has completed.
	auto makeStaging = []( UINT64 size, const void* src, UINT64 srcBytes ) {
		D3D12_RESOURCE_DESC desc = BASIC_BUFFER_DESC;
		desc.Width = size ? size : 1; // 0-width buffers are invalid
		ID3D12Resource* res;
		device->CreateCommittedResource( &UPLOAD_HEAP, D3D12_HEAP_FLAG_NONE, &desc,
			D3D12_RESOURCE_STATE_GENERIC_READ, nullptr, IID_PPV_ARGS( &res ) );
		void* mapped;
		res->Map( 0, nullptr, &mapped );
		memset( mapped, 0, desc.Width );          // zero the unused tail (e.g. spare nodes)
		if (src) memcpy( mapped, src, srcBytes ); // then the real data
		res->Unmap( 0, nullptr );
		return res;
		};
	// DEFAULT-heap (VRAM) buffers the compute shader actually reads from.
	auto makeDefault = []( UINT64 size ) {
		D3D12_RESOURCE_DESC desc = BASIC_BUFFER_DESC;
		desc.Width = size ? size : 1;
		ID3D12Resource* res;
		device->CreateCommittedResource( &DEFAULT_HEAP, D3D12_HEAP_FLAG_NONE, &desc,
			D3D12_RESOURCE_STATE_COPY_DEST, nullptr, IID_PPV_ARGS( &res ) );
		return res;
		};
	const UINT64 nodeBytes = (UINT64)bvh.allocatedNodes * sizeof( GPUBVHNode );
	const UINT64 idxBytes = (UINT64)bvh.bvh.idxCount * sizeof( unsigned );
	const UINT64 vertBytes = (UINT64)bvh.bvh.idxCount * 3 * sizeof( bvhvec4 );
	// BVH4_GPU counts storage in 16-byte 'blocks'; usedBlocks is what traversal touches.
	const UINT64 bvh4Bytes = (UINT64)bvh4.usedBlocks * sizeof( bvhvec4 );
	// CWBVH keeps nodes (5 blocks each, again counted in usedBlocks) and triangles
	// in two separate blobs; the triangle stride must match the tiny8.hlsl define.
	const UINT64 cwbvhNodeBytes = (UINT64)cwbvh.usedBlocks * sizeof( bvhvec4 );
#ifdef CWBVH_COMPRESSED_TRIS
	const UINT64 cwbvhTriBytes = (UINT64)cwbvh.idxCount * 4 * sizeof( bvhvec4 );
#else
	const UINT64 cwbvhTriBytes = (UINT64)cwbvh.idxCount * 3 * sizeof( bvhvec4 );
#endif
	ID3D12Resource* nodeStaging = makeStaging( nodeBytes, bvh.bvhNode, nodeBytes );
	ID3D12Resource* idxStaging = makeStaging( idxBytes, bvh.bvh.primIdx, idxBytes );
	bvhvec4* vertexData = (bvhvec4*)bvh.orderedVerts.data;
	ID3D12Resource* vertStaging = makeStaging( vertBytes, vertexData, vertBytes );
	ID3D12Resource* bvh4Staging = makeStaging( bvh4Bytes, bvh4.bvh4Data, bvh4Bytes );
	ID3D12Resource* cwbvhNodeStaging = makeStaging( cwbvhNodeBytes, cwbvh.bvh8Data, cwbvhNodeBytes );
	ID3D12Resource* cwbvhTriStaging = makeStaging( cwbvhTriBytes, cwbvh.bvh8Tris, cwbvhTriBytes );
	bvhNodeBuffer = makeDefault( nodeBytes );
	bvhIdxBuffer = makeDefault( idxBytes );
	bvhVertBuffer = makeDefault( vertBytes );
	bvh4DataBuffer = makeDefault( bvh4Bytes );
	cwbvhNodeBuffer = makeDefault( cwbvhNodeBytes );
	cwbvhTriBuffer = makeDefault( cwbvhTriBytes );
	bvhNodeBuffer->SetName( L"bvhNodeBuffer" );
	bvhIdxBuffer->SetName( L"bvhIdxBuffer" );
	bvhVertBuffer->SetName( L"bvhVertBuffer" );
	bvh4DataBuffer->SetName( L"bvh4DataBuffer" );
	cwbvhNodeBuffer->SetName( L"cwbvhNodeBuffer" );
	cwbvhTriBuffer->SetName( L"cwbvhTriBuffer" );
	// copy staging -> VRAM, then flip the VRAM buffers to a compute-shader-readable state. 
	// Uses slot 0's allocator and fully drains the GPU, matching the other pre-render-loop setup helpers.
	cmdAllocs[0]->Reset();
	cmdList->Reset( cmdAllocs[0], nullptr );
	cmdList->CopyResource( bvhNodeBuffer, nodeStaging );
	cmdList->CopyResource( bvhIdxBuffer, idxStaging );
	cmdList->CopyResource( bvhVertBuffer, vertStaging );
	cmdList->CopyResource( bvh4DataBuffer, bvh4Staging );
	cmdList->CopyResource( cwbvhNodeBuffer, cwbvhNodeStaging );
	cmdList->CopyResource( cwbvhTriBuffer, cwbvhTriStaging );
	auto toSRV = []( ID3D12Resource* res ) {
		D3D12_RESOURCE_BARRIER rb = { .Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION,
			.Transition = {.pResource = res,
				.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST,
				.StateAfter = D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE } };
		return rb; };
	D3D12_RESOURCE_BARRIER barriers[] = {
		toSRV( bvhNodeBuffer ), toSRV( bvhIdxBuffer ), toSRV( bvhVertBuffer ), toSRV( bvh4DataBuffer ),
		toSRV( cwbvhNodeBuffer ), toSRV( cwbvhTriBuffer ) };
	cmdList->ResourceBarrier( (UINT)std::size( barriers ), barriers );
	cmdList->Close();
	cmdQueue->ExecuteCommandLists( 1, reinterpret_cast<ID3D12CommandList**>(&cmdList) );
	WaitForGpu();
	nodeStaging->Release();
	idxStaging->Release();
	vertStaging->Release();
	bvh4Staging->Release();
	cwbvhNodeStaging->Release();
	cwbvhTriStaging->Release();
}

void Resize( HWND hwnd )
{	
	WaitForGpu(); // must fully drain before touching swapchain buffers
	swapChain->ResizeBuffers( 0, rtWidth, rtHeight, DXGI_FORMAT_UNKNOWN, 0 );
	if (renderTarget) renderTarget->Release();
	D3D12_RESOURCE_DESC rtDesc = {
		.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D,
		.Width = rtWidth, .Height = rtHeight, .DepthOrArraySize = 1,
		.MipLevels = 1, .Format = DXGI_FORMAT_R8G8B8A8_UNORM, .SampleDesc = NO_AA,
		.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS };
	device->CreateCommittedResource( &DEFAULT_HEAP, D3D12_HEAP_FLAG_NONE, &rtDesc,
		D3D12_RESOURCE_STATE_UNORDERED_ACCESS, nullptr, IID_PPV_ARGS( &renderTarget ) );
	D3D12_UNORDERED_ACCESS_VIEW_DESC uavDesc = {
		.Format = DXGI_FORMAT_R8G8B8A8_UNORM, .ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D };
	device->CreateUnorderedAccessView( renderTarget, nullptr, &uavDesc, uavHeap->GetCPUDescriptorHandleForHeapStart() );
	UpdateRayBuffer();
	for (UINT i = 0; i < FRAME_COUNT; i++) frameFenceValues[i] = 0;
}

LRESULT WINAPI WndProc( HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam )
{
	switch (msg)
	{
	case WM_CLOSE: case WM_DESTROY: PostQuitMessage( 0 ); [[fallthrough]];
	case WM_SIZING: case WM_SIZE: if (swapChain) Resize( hwnd ); [[fallthrough]];
	default: return DefWindowProcW( hwnd, msg, wparam, lparam );
	}
}

void InitDevice()
{
	if (FAILED( CreateDXGIFactory2( DXGI_CREATE_FACTORY_DEBUG, IID_PPV_ARGS( &factory ) ) ))
		CreateDXGIFactory2( 0, IID_PPV_ARGS( &factory ) );
	IDXGIAdapter1* adapter = nullptr;
	factory->EnumAdapterByGpuPreference( 0, DXGI_GPU_PREFERENCE_HIGH_PERFORMANCE, IID_PPV_ARGS( &adapter ) );
	// factory->EnumWarpAdapter( IID_PPV_ARGS( &adapter ) ); // uncomment for software RT
	// create device
	D3D12CreateDevice( adapter, D3D_FEATURE_LEVEL_12_1, IID_PPV_ARGS( &device ) );
	adapter->Release();
	D3D12_COMMAND_QUEUE_DESC cmdQueueDesc = { .Type = D3D12_COMMAND_LIST_TYPE_DIRECT };
	device->CreateCommandQueue( &cmdQueueDesc, IID_PPV_ARGS( &cmdQueue ) );
	device->CreateFence( 0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS( &fence ) );
	fenceEvent = CreateEvent( nullptr, FALSE, FALSE, nullptr );
}

void InitComputePipeline()
{
	D3D12_COMPUTE_PIPELINE_STATE_DESC desc = { .pRootSignature = rootSignature,
		.CS = {.pShaderBytecode = compiledComputeShader1, .BytecodeLength = std::size( compiledComputeShader1 ) } };
	device->CreateComputePipelineState( &desc, IID_PPV_ARGS( &computePso ) );
	D3D12_COMPUTE_PIPELINE_STATE_DESC desc4 = { .pRootSignature = rootSignature,
		.CS = {.pShaderBytecode = compiledComputeShader2, .BytecodeLength = std::size( compiledComputeShader2 ) } };
	device->CreateComputePipelineState( &desc4, IID_PPV_ARGS( &computePso4 ) );
	D3D12_COMPUTE_PIPELINE_STATE_DESC desc8 = { .pRootSignature = rootSignature,
		.CS = {.pShaderBytecode = compiledComputeShader3, .BytecodeLength = std::size( compiledComputeShader3 ) } };
	device->CreateComputePipelineState( &desc8, IID_PPV_ARGS( &computePso8 ) );
}

void InitSurfaces( HWND hwnd )
{
	DXGI_SWAP_CHAIN_DESC1 scDesc = {
		.Format = DXGI_FORMAT_R8G8B8A8_UNORM, .SampleDesc = NO_AA, 
		.BufferCount = FRAME_COUNT, .SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD,
	};
	IDXGISwapChain1* swapChain1;
	factory->CreateSwapChainForHwnd( cmdQueue, hwnd, &scDesc, nullptr, nullptr, &swapChain1 );
	swapChain1->QueryInterface( &swapChain );
	swapChain1->Release();
	factory->Release();
	D3D12_DESCRIPTOR_HEAP_DESC uavHeapDesc = {
		.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV,
		.NumDescriptors = 1, .Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE };
	device->CreateDescriptorHeap( &uavHeapDesc, IID_PPV_ARGS( &uavHeap ) );
	Resize( hwnd );
}

void InitCommand()
{
	for (UINT i = 0; i < FRAME_COUNT; i++)
		device->CreateCommandAllocator( D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS( &cmdAllocs[i] ) );
	// command list is created against slot 0's allocator; it gets Reset() against
	// the correct slot's allocator at the top of every Render() call anyway.
	device->CreateCommandList1( 0, D3D12_COMMAND_LIST_TYPE_DIRECT, D3D12_COMMAND_LIST_FLAG_NONE, IID_PPV_ARGS( &cmdList ) );
}

void InitMeshes()
{
	auto makeAndCopy = []( void* data, uint64_t size ) {
		D3D12_RESOURCE_DESC desc = BASIC_BUFFER_DESC;
		desc.Width = size;
		ID3D12Resource* res;
		device->CreateCommittedResource( &UPLOAD_HEAP, D3D12_HEAP_FLAG_NONE, &desc, D3D12_RESOURCE_STATE_COMMON, nullptr, IID_PPV_ARGS( &res ) );
		void* ptr;
		res->Map( 0, nullptr, &ptr );
		memcpy( ptr, data, size );
		res->Unmap( 0, nullptr );
		return res;
		};
	AddMesh( "../../testdata/cryteksponza.bin" );
	meshVB = makeAndCopy( verts, (uint64_t)triCount * 3 * sizeof( bvhvec4 ) );
	// One spatial-split build, converted to both GPU layouts.
	bvh2.BuildHQ( verts, triCount );
	bvh.ConvertFrom( bvh2, true );    // Aila & Laine 64-byte nodes + primIdx + separate tris
	mbvh4.ConvertFrom( bvh2, true );  // collapse the BVH2 into a 4-wide BVH
	bvh4.ConvertFrom( mbvh4, true );  // quantize into the single-blob BVH4_GPU layout
	cwbvh.BuildHQ( verts, triCount ); // separate SBVH build
}

ID3D12Resource* MakeAccelerationStructure( const D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS& inputs, UINT64* updateScratchSize = nullptr )
{
	auto makeBuffer = []( UINT64 size, D3D12_RESOURCE_STATES initialState ) {
		D3D12_RESOURCE_DESC desc = BASIC_BUFFER_DESC;
		desc.Width = size, desc.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;
		ID3D12Resource* buffer;
		device->CreateCommittedResource( &DEFAULT_HEAP, D3D12_HEAP_FLAG_NONE, &desc, initialState, nullptr, IID_PPV_ARGS( &buffer ) );
		return buffer;
		};
	D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO prebuildInfo;
	device->GetRaytracingAccelerationStructurePrebuildInfo( &inputs, &prebuildInfo );
	if (updateScratchSize) *updateScratchSize = prebuildInfo.UpdateScratchDataSizeInBytes;
	ID3D12Resource* scratch = makeBuffer( prebuildInfo.ScratchDataSizeInBytes, D3D12_RESOURCE_STATE_COMMON );
	ID3D12Resource* as = makeBuffer( prebuildInfo.ResultDataMaxSizeInBytes, D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE );
	D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC buildDesc = {
		.DestAccelerationStructureData = as->GetGPUVirtualAddress(),
		.Inputs = inputs, .ScratchAccelerationStructureData = scratch->GetGPUVirtualAddress() };
	// One-off, pre-render-loop build: use slot 0's allocator and fully drain the GPU.
	cmdAllocs[0]->Reset();
	cmdList->Reset( cmdAllocs[0], nullptr );
	cmdList->BuildRaytracingAccelerationStructure( &buildDesc, 0, nullptr );
	cmdList->Close();
	cmdQueue->ExecuteCommandLists( 1, reinterpret_cast<ID3D12CommandList**>(&cmdList) );
	WaitForGpu();
	scratch->Release();
	return as;
}

ID3D12Resource* MakeBLAS( ID3D12Resource* vertexBuffer, UINT vertexCount, UINT vertexStride,
	ID3D12Resource* indexBuffer = nullptr, UINT indices = 0 )
{
	D3D12_RAYTRACING_GEOMETRY_DESC geometryDesc = {
		.Type = D3D12_RAYTRACING_GEOMETRY_TYPE_TRIANGLES, .Flags = D3D12_RAYTRACING_GEOMETRY_FLAG_OPAQUE,
		.Triangles = {.Transform3x4 = 0, .IndexFormat = indexBuffer ? DXGI_FORMAT_R32_UINT : DXGI_FORMAT_UNKNOWN,
			.VertexFormat = DXGI_FORMAT_R32G32B32_FLOAT, .IndexCount = indexBuffer ? indices : 0, .VertexCount = vertexCount,
			.IndexBuffer = indexBuffer ? indexBuffer->GetGPUVirtualAddress() : 0,
			.VertexBuffer = {.StartAddress = vertexBuffer->GetGPUVirtualAddress(), .StrideInBytes = vertexStride}} };
	D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS inputs = {
		.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL,
		.Flags = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE | D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_ALLOW_COMPACTION,
		.NumDescs = 1, .DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY, .pGeometryDescs = &geometryDesc };
	return MakeAccelerationStructure( inputs );
}

ID3D12Resource* CompactBLAS( ID3D12Resource* as )
{
	D3D12_GPU_VIRTUAL_ADDRESS asAddr = as->GetGPUVirtualAddress();
	D3D12_RESOURCE_BARRIER uavBarrier = { .Type = D3D12_RESOURCE_BARRIER_TYPE_UAV, .UAV = {.pResource = as} };
	D3D12_RESOURCE_DESC postbuildDescBuf = BASIC_BUFFER_DESC;
	postbuildDescBuf.Width = sizeof( UINT64 );
	postbuildDescBuf.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;
	ID3D12Resource* postbuildBuffer = nullptr;
	device->CreateCommittedResource( &DEFAULT_HEAP, D3D12_HEAP_FLAG_NONE, &postbuildDescBuf,
		D3D12_RESOURCE_STATE_UNORDERED_ACCESS, nullptr, IID_PPV_ARGS( &postbuildBuffer ) );
	postbuildBuffer->SetName( L"postbuildBuffer" );
	D3D12_RAYTRACING_ACCELERATION_STRUCTURE_POSTBUILD_INFO_DESC postbuildDesc = {
		.DestBuffer = postbuildBuffer->GetGPUVirtualAddress(),
		.InfoType = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_POSTBUILD_INFO_COMPACTED_SIZE };
	cmdAllocs[0]->Reset();
	cmdList->Reset( cmdAllocs[0], nullptr );
	cmdList->ResourceBarrier( 1, &uavBarrier ); // must barrier before reading postbuild info
	cmdList->EmitRaytracingAccelerationStructurePostbuildInfo( &postbuildDesc, 1, &asAddr );
	D3D12_RESOURCE_BARRIER toCopySrc = { .Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION,
		.Transition = {.pResource = postbuildBuffer,
			.StateBefore = D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
			.StateAfter = D3D12_RESOURCE_STATE_COPY_SOURCE } };
	cmdList->ResourceBarrier( 1, &toCopySrc );
	cmdList->CopyBufferRegion( compactionSizeReadback, 0, postbuildBuffer, 0, sizeof( UINT64 ) );
	cmdList->Close();
	cmdQueue->ExecuteCommandLists( 1, reinterpret_cast<ID3D12CommandList**>(&cmdList) );
	WaitForGpu();
	postbuildBuffer->Release();
	UINT64 compactedSize;
	D3D12_RANGE readRange = { 0, sizeof( UINT64 ) }, writeRange = { 0, 0 };
	void* mapped;
	compactionSizeReadback->Map( 0, &readRange, &mapped );
	compactedSize = *reinterpret_cast<UINT64*>(mapped);
	compactionSizeReadback->Unmap( 0, &writeRange );
	D3D12_RESOURCE_DESC compactDesc = BASIC_BUFFER_DESC;
	compactDesc.Width = compactedSize;
	compactDesc.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;
	ID3D12Resource* compactedAS;
	device->CreateCommittedResource( &DEFAULT_HEAP, D3D12_HEAP_FLAG_NONE, &compactDesc,
		D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE, nullptr, IID_PPV_ARGS( &compactedAS ) );
	cmdAllocs[0]->Reset();
	cmdList->Reset( cmdAllocs[0], nullptr );
	cmdList->CopyRaytracingAccelerationStructure( compactedAS->GetGPUVirtualAddress(),
		asAddr, D3D12_RAYTRACING_ACCELERATION_STRUCTURE_COPY_MODE_COMPACT );
	cmdList->Close();
	cmdQueue->ExecuteCommandLists( 1, reinterpret_cast<ID3D12CommandList**>(&cmdList) );
	WaitForGpu();
	as->Release();
	return compactedAS;
}

ID3D12Resource* MakeTLAS( ID3D12Resource* instances, UINT numInstances, UINT64* updateScratchSize )
{
	D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS inputs = {
		.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL,
		.Flags = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE
			| D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_ALLOW_UPDATE,
		.NumDescs = numInstances, .DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY,
		.InstanceDescs = instances->GetGPUVirtualAddress() };
	return MakeAccelerationStructure( inputs, updateScratchSize );
}

void InitScene()
{
	D3D12_RESOURCE_DESC instancesDesc = BASIC_BUFFER_DESC;
	instancesDesc.Width = sizeof( D3D12_RAYTRACING_INSTANCE_DESC ) * NUM_INSTANCES;
	device->CreateCommittedResource( &UPLOAD_HEAP, D3D12_HEAP_FLAG_NONE,
		&instancesDesc, D3D12_RESOURCE_STATE_COMMON, nullptr, IID_PPV_ARGS( &instances ) );
	instances->Map( 0, nullptr, reinterpret_cast<void**>(&instanceData) );
	instanceData[0] = { .InstanceID = 0, .InstanceMask = 1, .AccelerationStructure = blas->GetGPUVirtualAddress() };
	DirectX::XMMATRIX T = DirectX::XMMatrixTranslation( 0, 0, 0 );
	DirectX::XMStoreFloat3x4( (DirectX::XMFLOAT3X4*)&instanceData[0].Transform, T );
}

void InitTopLevel()
{
	UINT64 updateScratchSize = 0;
	tlas = MakeTLAS( instances, NUM_INSTANCES, &updateScratchSize );
	D3D12_RESOURCE_DESC desc = BASIC_BUFFER_DESC;
	desc.Width = updateScratchSize;
	desc.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;
	device->CreateCommittedResource( &DEFAULT_HEAP, D3D12_HEAP_FLAG_NONE, &desc, D3D12_RESOURCE_STATE_COMMON, nullptr, IID_PPV_ARGS( &tlasUpdateScratch ) );
}

void InitRootSignature()
{
	D3D12_DESCRIPTOR_RANGE uavRange = { .RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_UAV, .NumDescriptors = 1 };
	D3D12_ROOT_PARAMETER params[] = {
	{.ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE,
	 .DescriptorTable = {.NumDescriptorRanges = 1, .pDescriptorRanges = &uavRange}},
	{.ParameterType = D3D12_ROOT_PARAMETER_TYPE_SRV, .Descriptor = {.ShaderRegister = 0, .RegisterSpace = 0}}, // t0: tlas (DXR)
	{.ParameterType = D3D12_ROOT_PARAMETER_TYPE_SRV, .Descriptor = {.ShaderRegister = 1, .RegisterSpace = 0}}, // t1: ray buffer (DXR + compute)
	{.ParameterType = D3D12_ROOT_PARAMETER_TYPE_SRV, .Descriptor = {.ShaderRegister = 0, .RegisterSpace = 1}}, // t0 space1: BVH nodes
	{.ParameterType = D3D12_ROOT_PARAMETER_TYPE_SRV, .Descriptor = {.ShaderRegister = 1, .RegisterSpace = 1}}, // t1 space1: primIdx
	{.ParameterType = D3D12_ROOT_PARAMETER_TYPE_SRV, .Descriptor = {.ShaderRegister = 2, .RegisterSpace = 1}}, // t2 space1: triangles
	{.ParameterType = D3D12_ROOT_PARAMETER_TYPE_SRV, .Descriptor = {.ShaderRegister = 3, .RegisterSpace = 1}}, // t3 space1: BVH4_GPU blob
	{.ParameterType = D3D12_ROOT_PARAMETER_TYPE_SRV, .Descriptor = {.ShaderRegister = 4, .RegisterSpace = 1}}, // t4 space1: CWBVH nodes
	{.ParameterType = D3D12_ROOT_PARAMETER_TYPE_SRV, .Descriptor = {.ShaderRegister = 5, .RegisterSpace = 1}} // t5 space1: CWBVH triangles
	};
	D3D12_ROOT_SIGNATURE_DESC desc = { .NumParameters = std::size( params ), .pParameters = params };
	ID3DBlob* blob;
	D3D12SerializeRootSignature( &desc, D3D_ROOT_SIGNATURE_VERSION_1_0, &blob, nullptr );
	device->CreateRootSignature( 0, blob->GetBufferPointer(), blob->GetBufferSize(), IID_PPV_ARGS( &rootSignature ) );
	blob->Release();
}

void InitQueryHeap()
{
	D3D12_QUERY_HEAP_DESC queryHeapDesc = { .Type = D3D12_QUERY_HEAP_TYPE_TIMESTAMP, .Count = 8 };
	device->CreateQueryHeap( &queryHeapDesc, IID_PPV_ARGS( &queryHeap ) );
	D3D12_HEAP_PROPERTIES heapProps = { .Type = D3D12_HEAP_TYPE_READBACK };
	D3D12_RESOURCE_DESC bufferDesc = BASIC_BUFFER_DESC;
	bufferDesc.Width = 8 * sizeof( UINT64 );
	device->CreateCommittedResource( &heapProps, D3D12_HEAP_FLAG_NONE, &bufferDesc,
		D3D12_RESOURCE_STATE_COPY_DEST, nullptr, IID_PPV_ARGS( &queryResultBuffer ) );
}

void InitCompactionReadback()
{
	D3D12_HEAP_PROPERTIES heapProps = { .Type = D3D12_HEAP_TYPE_READBACK };
	D3D12_RESOURCE_DESC bufDesc = BASIC_BUFFER_DESC;
	bufDesc.Width = sizeof( UINT64 );
	device->CreateCommittedResource( &heapProps, D3D12_HEAP_FLAG_NONE, &bufDesc,
		D3D12_RESOURCE_STATE_COPY_DEST, nullptr, IID_PPV_ARGS( &compactionSizeReadback ) );
}

void InitPipeline()
{
	D3D12_DXIL_LIBRARY_DESC lib = { .DXILLibrary = {.pShaderBytecode = compiledShader, .BytecodeLength = std::size( compiledShader )} };
	D3D12_HIT_GROUP_DESC hitGroup = { .HitGroupExport = L"HitGroup", .Type = D3D12_HIT_GROUP_TYPE_TRIANGLES, .ClosestHitShaderImport = L"ClosestHit" };
	D3D12_RAYTRACING_SHADER_CONFIG shaderCfg = { .MaxPayloadSizeInBytes = 4, .MaxAttributeSizeInBytes = 8 };
	D3D12_GLOBAL_ROOT_SIGNATURE globalSig = { rootSignature };
	D3D12_RAYTRACING_PIPELINE_CONFIG pipelineCfg = { .MaxTraceRecursionDepth = 1 };
	D3D12_STATE_SUBOBJECT subobjects[] = {
		{.Type = D3D12_STATE_SUBOBJECT_TYPE_DXIL_LIBRARY, .pDesc = &lib},
		{.Type = D3D12_STATE_SUBOBJECT_TYPE_HIT_GROUP, .pDesc = &hitGroup},
		{.Type = D3D12_STATE_SUBOBJECT_TYPE_RAYTRACING_SHADER_CONFIG, .pDesc = &shaderCfg},
		{.Type = D3D12_STATE_SUBOBJECT_TYPE_GLOBAL_ROOT_SIGNATURE, .pDesc = &globalSig},
		{.Type = D3D12_STATE_SUBOBJECT_TYPE_RAYTRACING_PIPELINE_CONFIG, .pDesc = &pipelineCfg} };
	D3D12_STATE_OBJECT_DESC desc = { .Type = D3D12_STATE_OBJECT_TYPE_RAYTRACING_PIPELINE,
		.NumSubobjects = std::size( subobjects ), .pSubobjects = subobjects };
	device->CreateStateObject( &desc, IID_PPV_ARGS( &pso ) );
}

void InitShaderTables()
{
	D3D12_RESOURCE_DESC idDesc = BASIC_BUFFER_DESC;
	idDesc.Width = NUM_SHADER_IDS * D3D12_RAYTRACING_SHADER_TABLE_BYTE_ALIGNMENT;
	device->CreateCommittedResource( &UPLOAD_HEAP, D3D12_HEAP_FLAG_NONE, &idDesc,
		D3D12_RESOURCE_STATE_COMMON, nullptr, IID_PPV_ARGS( &shaderIDs ) );
	ID3D12StateObjectProperties* props;
	pso->QueryInterface( &props );
	void* data;
	auto writeId = [&]( const wchar_t* name ) { void* id = props->GetShaderIdentifier( name );
	memcpy( data, id, D3D12_SHADER_IDENTIFIER_SIZE_IN_BYTES );
	data = static_cast<char*>(data) + D3D12_RAYTRACING_SHADER_TABLE_BYTE_ALIGNMENT; };
	shaderIDs->Map( 0, nullptr, &data );
	writeId( L"RayGeneration" );
	writeId( L"Miss" );
	writeId( L"HitGroup" );
	shaderIDs->Unmap( 0, nullptr );
	props->Release();
}

void UpdateScene()
{
	D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC desc = {
		.DestAccelerationStructureData = tlas->GetGPUVirtualAddress(),
		.Inputs = {
			.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL,
			// Must match the flags the TLAS was originally built with, plus PERFORM_UPDATE.
			.Flags = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE
				| D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_ALLOW_UPDATE
				| D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PERFORM_UPDATE,
			.NumDescs = NUM_INSTANCES, .DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY,
			.InstanceDescs = instances->GetGPUVirtualAddress()},
		.SourceAccelerationStructureData = tlas->GetGPUVirtualAddress(),
		.ScratchAccelerationStructureData = tlasUpdateScratch->GetGPUVirtualAddress(),
	};
	cmdList->BuildRaytracingAccelerationStructure( &desc, 0, nullptr );
	D3D12_RESOURCE_BARRIER barrier = { .Type = D3D12_RESOURCE_BARRIER_TYPE_UAV, .UAV = {.pResource = tlas} };
	cmdList->ResourceBarrier( 1, &barrier );
}

void Render()
{
	UINT frameIndex = swapChain->GetCurrentBackBufferIndex();
	if (frameFenceValues[frameIndex] != 0 && fence->GetCompletedValue() < frameFenceValues[frameIndex])
	{
		fence->SetEventOnCompletion( frameFenceValues[frameIndex], fenceEvent );
		WaitForSingleObject( fenceEvent, INFINITE );
	}
	UINT baseSlot = frameIndex * 2;
	// Round-robin the ray tracing backends.
	static UINT64 frameCounter = 0;
	static int slotBackend[FRAME_COUNT] = {}; // which backend last ran in each frame slot
	const int backend = (int)(frameCounter % BACKEND_COUNT);
	if (frameFenceValues[frameIndex] != 0)
	{
		UINT64* timestamps = nullptr;
		D3D12_RANGE readRange = { baseSlot * sizeof( UINT64 ), (baseSlot + 2) * sizeof( UINT64 ) };
		HRESULT hr = queryResultBuffer->Map( 0, &readRange, reinterpret_cast<void**>(&timestamps) );
		if (FAILED( hr ) || !timestamps)
		{
			if (hr == DXGI_ERROR_DEVICE_REMOVED || hr == DXGI_ERROR_DEVICE_HUNG) 
				printf( "device removed: 0x%08X\n", device->GetDeviceRemovedReason() );
			else printf( "Map failed: 0x%08X\n", hr );
			__debugbreak();
		}
		UINT64 startTimestamp = timestamps[baseSlot], endTimestamp = timestamps[baseSlot + 1];
		D3D12_RANGE writeRange = { 0, 0 };
		queryResultBuffer->Unmap( 0, &writeRange );
		UINT64 frequency = 0;
		cmdQueue->GetTimestampFrequency( &frequency );
		double rtTime = static_cast<double>(endTimestamp - startTimestamp) / static_cast<double>(frequency);
		rtTime /= (double)NUM_DISPATCHES;
		D3D12_RESOURCE_DESC rtDescForTiming = renderTarget->GetDesc();
		double raysPerSecond = (rtDescForTiming.Width * rtDescForTiming.Height) / rtTime;
		const int k = slotBackend[frameIndex]; // which backend produced this timestamp
		static double smoothed[BACKEND_COUNT] = {};
		static int frames[BACKEND_COUNT] = {};
		if (++frames[k] == 1) smoothed[k] = raysPerSecond;
		else if (frames[k] < 10) smoothed[k] = 0.9 * smoothed[k] + 0.1 * raysPerSecond;
		else smoothed[k] = 0.99 * smoothed[k] + 0.01 * raysPerSecond;
		// DXR is the reference; report the software kernels as a fraction of it.
		const double ref = smoothed[BACKEND_DXR];
		if (k == BACKEND_DXR || ref <= 0.0)
			printf( "%s: %6.1f MRays/s",
				backendName[k], smoothed[k] / 1e6 );
		else
			printf( "%s: %6.1f MRays/s, %4.1f%% of DXR",
				backendName[k], smoothed[k] / 1e6, 100.0 * smoothed[k] / ref );
		if (k == BACKEND_COUNT - 1) printf( "\n" ); else printf( "; " );
	}
	cmdAllocs[frameIndex]->Reset();
	cmdList->Reset( cmdAllocs[frameIndex], nullptr );
	static bool sceneDirty = true;
	if (sceneDirty) { UpdateScene(); sceneDirty = false; }
	if (backend != BACKEND_DXR)
	{
		// The software kernels share the root signature, the ray buffer and the
		// render target UAV; they differ only in the PSO and the BVH bindings.
		cmdList->SetPipelineState( backend == BACKEND_BVH4_GPU ? computePso4 :
			backend == BACKEND_BVH8_CWBVH ? computePso8 : computePso );
		cmdList->SetComputeRootSignature( rootSignature );
		cmdList->SetDescriptorHeaps( 1, &uavHeap );
		cmdList->SetComputeRootDescriptorTable( 0, uavHeap->GetGPUDescriptorHandleForHeapStart() );
		cmdList->SetComputeRootShaderResourceView( 2, rayBuffer->GetGPUVirtualAddress() );         // t1: rays
		if (backend == BACKEND_BVH4_GPU)
			cmdList->SetComputeRootShaderResourceView( 6, bvh4DataBuffer->GetGPUVirtualAddress() ); // t3 space1: BVH4_GPU blob
		else if (backend == BACKEND_BVH8_CWBVH)
		{
			cmdList->SetComputeRootShaderResourceView( 7, cwbvhNodeBuffer->GetGPUVirtualAddress() ); // t4 space1: CWBVH nodes
			cmdList->SetComputeRootShaderResourceView( 8, cwbvhTriBuffer->GetGPUVirtualAddress() );  // t5 space1: CWBVH triangles
		}
		else
		{
			cmdList->SetComputeRootShaderResourceView( 3, bvhNodeBuffer->GetGPUVirtualAddress() ); // t0 space1: BVH nodes
			cmdList->SetComputeRootShaderResourceView( 4, bvhIdxBuffer->GetGPUVirtualAddress() );  // t1 space1: primIdx
			cmdList->SetComputeRootShaderResourceView( 5, bvhVertBuffer->GetGPUVirtualAddress() );  // t2 space1: triangles
		}
		D3D12_RESOURCE_DESC rtDesc = renderTarget->GetDesc();
		cmdList->EndQuery( queryHeap, D3D12_QUERY_TYPE_TIMESTAMP, baseSlot );
		for( int i = 0; i < NUM_DISPATCHES; i++ ) cmdList->Dispatch( (static_cast<UINT>(rtDesc.Width) + 7) / 8, (rtDesc.Height + 7) / 8, 1 );
		cmdList->EndQuery( queryHeap, D3D12_QUERY_TYPE_TIMESTAMP, baseSlot + 1 );
	}
	else
	{
		cmdList->SetPipelineState1( pso );
		cmdList->SetComputeRootSignature( rootSignature );
		cmdList->SetDescriptorHeaps( 1, &uavHeap );
		D3D12_GPU_DESCRIPTOR_HANDLE uavTable = uavHeap->GetGPUDescriptorHandleForHeapStart();
		cmdList->SetComputeRootDescriptorTable( 0, uavTable );
		cmdList->SetComputeRootShaderResourceView( 1, tlas->GetGPUVirtualAddress() );
		cmdList->SetComputeRootShaderResourceView( 2, rayBuffer->GetGPUVirtualAddress() );
		D3D12_RESOURCE_DESC rtDesc = renderTarget->GetDesc();
		D3D12_DISPATCH_RAYS_DESC dispatchDesc = {
			.RayGenerationShaderRecord = {
				.StartAddress = shaderIDs->GetGPUVirtualAddress(),
				.SizeInBytes = D3D12_SHADER_IDENTIFIER_SIZE_IN_BYTES },
			.MissShaderTable = {
				.StartAddress = shaderIDs->GetGPUVirtualAddress() + D3D12_RAYTRACING_SHADER_TABLE_BYTE_ALIGNMENT,
				.SizeInBytes = D3D12_SHADER_IDENTIFIER_SIZE_IN_BYTES, .StrideInBytes = 0 },
			.HitGroupTable = {
				.StartAddress = shaderIDs->GetGPUVirtualAddress() + 2 * D3D12_RAYTRACING_SHADER_TABLE_BYTE_ALIGNMENT,
				.SizeInBytes = D3D12_SHADER_IDENTIFIER_SIZE_IN_BYTES, .StrideInBytes = 32 },
			.Width = static_cast<UINT>(rtDesc.Width), .Height = rtDesc.Height, .Depth = 1 };
		cmdList->EndQuery( queryHeap, D3D12_QUERY_TYPE_TIMESTAMP, baseSlot );
		for( int i = 0; i < NUM_DISPATCHES; i++ ) cmdList->DispatchRays( &dispatchDesc );
		cmdList->EndQuery( queryHeap, D3D12_QUERY_TYPE_TIMESTAMP, baseSlot + 1 );
	}
	slotBackend[frameIndex] = backend; // for the timing readback next time this slot runs
	frameCounter++;
	cmdList->ResolveQueryData( queryHeap, D3D12_QUERY_TYPE_TIMESTAMP, baseSlot, 2, queryResultBuffer, baseSlot * sizeof( UINT64 ) );
	swapChain->GetBuffer( frameIndex, IID_PPV_ARGS( &backBuffer ) );
	auto barrier = []( auto* resource, auto before, auto after ) { D3D12_RESOURCE_BARRIER rb = {
			.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION,
			.Transition = {.pResource = resource, .StateBefore = before, .StateAfter = after} };
	cmdList->ResourceBarrier( 1, &rb ); };
	barrier( renderTarget, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_COPY_SOURCE );
	barrier( backBuffer, D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_COPY_DEST );
	cmdList->CopyResource( backBuffer, renderTarget );
	barrier( backBuffer, D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_PRESENT );
	barrier( renderTarget, D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS );
	backBuffer->Release();
	cmdList->Close();
	cmdQueue->ExecuteCommandLists( 1, reinterpret_cast<ID3D12CommandList**>(&cmdList) );
	fenceValue++;
	cmdQueue->Signal( fence, fenceValue );
	frameFenceValues[frameIndex] = fenceValue;
	swapChain->Present( 0, 0 );
}

void Init( HWND hwnd )
{
	InitDevice();
	InitCommand();
	InitQueryHeap();
	InitCompactionReadback();
	InitSurfaces( hwnd );
	InitMeshes();
	InitBVHBuffers();
	UpdateRayBuffer();
	blas = CompactBLAS( MakeBLAS( meshVB, triCount * 3, sizeof( bvhvec4 ) ) );
	InitScene();
	InitTopLevel();
	InitRootSignature();
	InitPipeline();
	InitComputePipeline();
	InitShaderTables();
}

int main()
{
	SetProcessDpiAwarenessContext( DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2 /* or DPI_AWARENESS_CONTEXT_UNAWARE */ );
	WNDCLASSW wcw = { .lpfnWndProc = &WndProc, .hCursor = LoadCursor( nullptr, IDC_ARROW ), .lpszClassName = L"μDXR" };
	RegisterClassW( &wcw );
	HWND hwnd = CreateWindowExW( 0, L"μDXR", L"_DXR", WS_VISIBLE | WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT, rtWidth, rtHeight, 0, 0, 0, 0 );
	Init( hwnd );
	for (MSG msg;;)
	{
		while (PeekMessageW( &msg, nullptr, 0, 0, PM_REMOVE ))
		{
			if (msg.message == WM_QUIT) return 0;
			TranslateMessage( &msg );
			DispatchMessageW( &msg );
		}
		Render(); // Render the next frame
	}
}