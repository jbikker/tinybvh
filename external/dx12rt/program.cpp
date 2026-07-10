// heavily based on Laura Andelare's great DXR tutorial:
// https://landelare.github.io/2023/02/18/dxr-tutorial.html

#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <algorithm>     // std::size, typed std::max, etc.
#include <DirectXMath.h> // for XMMATRIX
#include <Windows.h>
#include <d3d12.h>
#include <dxgi1_4.h>
#include "shader.fxh"
#include <fstream>

#pragma comment(lib, "user32") // For DefWindowProcW, etc.
#pragma comment(lib, "d3d12")  // You'll never guess this one
#pragma comment(lib, "dxgi")   // Another enigma

#define DECLARE_AND_CALL(fn) void fn(); fn()

constexpr DXGI_SAMPLE_DESC NO_AA = { .Count = 1, .Quality = 0 };
constexpr D3D12_HEAP_PROPERTIES UPLOAD_HEAP = { .Type = D3D12_HEAP_TYPE_UPLOAD };
constexpr D3D12_HEAP_PROPERTIES DEFAULT_HEAP = { .Type = D3D12_HEAP_TYPE_DEFAULT };
constexpr D3D12_RESOURCE_DESC BASIC_BUFFER_DESC = {
	.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER,
	.Width = 0, // will be changed in copies
	.Height = 1, .DepthOrArraySize = 1,
	.MipLevels = 1, .SampleDesc = NO_AA,
	.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR
};
constexpr UINT NUM_INSTANCES = 1;
constexpr UINT64 NUM_SHADER_IDS = 3;
IDXGIFactory4* factory;
ID3D12Device5* device;
ID3D12CommandQueue* cmdQueue;
ID3D12Fence* fence;
IDXGISwapChain3* swapChain;
ID3D12DescriptorHeap* uavHeap;
ID3D12Resource* renderTarget, * backBuffer, * cubeVB, * cubeIB, * cubeBlas, * instances, * shaderIDs;
ID3D12Resource* tlas, * tlasUpdateScratch;
ID3D12CommandAllocator* cmdAlloc;
ID3D12GraphicsCommandList4* cmdList;
D3D12_RAYTRACING_INSTANCE_DESC* instanceData;
ID3D12RootSignature* rootSignature;
ID3D12StateObject* pso;
ID3D12QueryHeap* queryHeap;
ID3D12Resource* queryResultBuffer;
HANDLE fenceEvent = nullptr;

struct bvhvec3 { float x, y, z; };
struct bvhvec4 { float x, y, z, w; };
static bvhvec3 renderSettings[4] = {
	{ 0, 0, -7 }, // eye
	{ 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } // p1, p2, p3
};

// Scene management - Append a file, with optional position, scale and color override, tinyfied
int triCount = 0;
bvhvec4* tris = 0;
bvhvec3* t3 = 0;
unsigned* vidx = 0;
void AddMesh( const char* file, int N = 0 )
{
	std::fstream s{ file, s.binary | s.in };
	s.read( (char*)&N, 4 );
	bvhvec4* data = (bvhvec4*)_aligned_malloc( (N + triCount) * 48, 64 );
	if (tris) memcpy( data, tris, triCount * 48 ), _aligned_free( tris );
	tris = data, s.read( (char*)tris + triCount * 48, N * 48 ), triCount += N;
}

static UINT64 fenceValue = 1;

void Flush()
{
	cmdQueue->Signal( fence, fenceValue );
	fence->SetEventOnCompletion( fenceValue++, nullptr );
}

void WaitForGpu()
{
	const UINT64 fenceValueToWait = fenceValue;
	cmdQueue->Signal( fence, fenceValueToWait );
	fenceValue++;
	if (fence->GetCompletedValue() < fenceValueToWait)
	{
		fence->SetEventOnCompletion( fenceValueToWait, fenceEvent );
		WaitForSingleObject( fenceEvent, INFINITE );
	}
}

void Resize( HWND hwnd )
{
	RECT rect;
	GetClientRect( hwnd, &rect );
	UINT width = std::max<UINT>( rect.right - rect.left, 1 );
	UINT height = std::max<UINT>( rect.bottom - rect.top, 1 );
	Flush();
	swapChain->ResizeBuffers( 0, width, height, DXGI_FORMAT_UNKNOWN, 0 );
	if (renderTarget) renderTarget->Release();
	D3D12_RESOURCE_DESC rtDesc = {
		.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D,
		.Width = width, .Height = height,
		.DepthOrArraySize = 1,
		.MipLevels = 1, .Format = DXGI_FORMAT_R8G8B8A8_UNORM, .SampleDesc = NO_AA,
		.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS };
	device->CreateCommittedResource( &DEFAULT_HEAP, D3D12_HEAP_FLAG_NONE, &rtDesc,
		D3D12_RESOURCE_STATE_UNORDERED_ACCESS, nullptr, IID_PPV_ARGS( &renderTarget ) );
	D3D12_UNORDERED_ACCESS_VIEW_DESC uavDesc = {
		.Format = DXGI_FORMAT_R8G8B8A8_UNORM, .ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D };
	device->CreateUnorderedAccessView( renderTarget, nullptr, &uavDesc, uavHeap->GetCPUDescriptorHandleForHeapStart() );
}

LRESULT WINAPI WndProc( HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam )
{
	switch (msg)
	{
	case WM_CLOSE:
	case WM_DESTROY: PostQuitMessage( 0 ); [[fallthrough]];
	case WM_SIZING:
	case WM_SIZE: if (swapChain) Resize( hwnd ); [[fallthrough]];
	default: return DefWindowProcW( hwnd, msg, wparam, lparam );
	}
}

void Init( HWND hwnd )
{
	DECLARE_AND_CALL( InitDevice );
	DECLARE_AND_CALL( InitQueryHeap );
	void InitSurfaces( HWND ); InitSurfaces( hwnd );
	DECLARE_AND_CALL( InitCommand );
	DECLARE_AND_CALL( InitMeshes );
	DECLARE_AND_CALL( InitBottomLevel );
	DECLARE_AND_CALL( InitScene );
	DECLARE_AND_CALL( InitTopLevel );
	DECLARE_AND_CALL( InitRootSignature );
	DECLARE_AND_CALL( InitPipeline );
	DECLARE_AND_CALL( InitShaderTables );
}

void InitDevice()
{
	if (FAILED( CreateDXGIFactory2( DXGI_CREATE_FACTORY_DEBUG, IID_PPV_ARGS( &factory ) ) ))
		CreateDXGIFactory2( 0, IID_PPV_ARGS( &factory ) );
	if (ID3D12Debug* debug; SUCCEEDED( D3D12GetDebugInterface( IID_PPV_ARGS( &debug ) ) ))
		debug->EnableDebugLayer(), debug->Release();
	IDXGIAdapter* adapter = nullptr;
	// Uncomment the following line to use software rendering with WARP:
	// factory->EnumWarpAdapter(IID_PPV_ARGS(&adapter));
	D3D12CreateDevice( adapter, D3D_FEATURE_LEVEL_12_1, IID_PPV_ARGS( &device ) );
	D3D12_COMMAND_QUEUE_DESC cmdQueueDesc = { .Type = D3D12_COMMAND_LIST_TYPE_DIRECT };
	device->CreateCommandQueue( &cmdQueueDesc, IID_PPV_ARGS( &cmdQueue ) );
	device->CreateFence( 0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS( &fence ) );
	fenceEvent = CreateEvent( nullptr, FALSE, FALSE, nullptr );
}

void InitSurfaces( HWND hwnd )
{
	DXGI_SWAP_CHAIN_DESC1 scDesc = {
		.Format = DXGI_FORMAT_R8G8B8A8_UNORM,
		.SampleDesc = NO_AA, .BufferCount = 2,
		.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD,
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
	device->CreateCommandAllocator( D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS( &cmdAlloc ) );
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
#if 1
	// view1 for sponza
	renderSettings[0] = { -15.2399998f, 21.5000000f, 2.53999996f };
	renderSettings[1] = { -12.8712616f, 21.3436279f, 2.60801458f };
	renderSettings[2] = { -13.6628551f, 21.3436279f, 0.771338582f };
	renderSettings[3] = { -13.5145569f, 19.9051208f, 2.88527060f };
#elif 1
	// view2 for sponza
	renderSettings[0] = { -34.0000000f, 5.00000000f, 11.2600002f };
	renderSettings[1] = { -31.8041172f, 5.85805798f, 11.5460672f };
	renderSettings[2] = { -32.4691925f, 5.85805798f, 9.65988636f };
	renderSettings[3] = { -31.7600574f, 4.25874043f, 11.5305319f };
#else
	// view3 for sponza
	renderSettings[0] = { -1.29999995f, 4.96000004f, 12.2799997f };
	renderSettings[1] = { -3.09493661f, 5.86036921f, 11.0121126f };
	renderSettings[2] = { -3.37909675f, 5.86036921f, 12.9918222f };
	renderSettings[3] = { -3.17523217f, 4.26242685f, 11.0005865f };
#endif
	t3 = new bvhvec3[triCount * 3];
	for (int i = 0; i < triCount * 3; i++) t3[i].x = tris[i].x, t3[i].y = tris[i].y, t3[i].z = tris[i].z;
	vidx = new unsigned[triCount * 3];
	for (int i = 0; i < triCount * 3; i++) vidx[i] = i;
	cubeVB = makeAndCopy( (float*)t3, triCount * 36 );
	cubeIB = makeAndCopy( (void*)vidx, triCount * 3 * 4 );
}

ID3D12Resource* MakeAccelerationStructure( const D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS& inputs, UINT64* updateScratchSize = nullptr )
{
	auto makeBuffer = []( UINT64 size, auto initialState ) {
		auto desc = BASIC_BUFFER_DESC;
		desc.Width = size;
		desc.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;
		ID3D12Resource* buffer;
		device->CreateCommittedResource( &DEFAULT_HEAP, D3D12_HEAP_FLAG_NONE, &desc, initialState, nullptr, IID_PPV_ARGS( &buffer ) );
		return buffer;
		};
	D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO prebuildInfo;
	device->GetRaytracingAccelerationStructurePrebuildInfo( &inputs, &prebuildInfo );
	if (updateScratchSize) *updateScratchSize = prebuildInfo.UpdateScratchDataSizeInBytes;
	auto* scratch = makeBuffer( prebuildInfo.ScratchDataSizeInBytes, D3D12_RESOURCE_STATE_COMMON );
	auto* as = makeBuffer( prebuildInfo.ResultDataMaxSizeInBytes, D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE );
	D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC buildDesc = {
		.DestAccelerationStructureData = as->GetGPUVirtualAddress(),
		.Inputs = inputs, .ScratchAccelerationStructureData = scratch->GetGPUVirtualAddress() };
	cmdAlloc->Reset();
	cmdList->Reset( cmdAlloc, nullptr );
	cmdList->BuildRaytracingAccelerationStructure( &buildDesc, 0, nullptr );
	cmdList->Close();
	cmdQueue->ExecuteCommandLists( 1, reinterpret_cast<ID3D12CommandList**>(&cmdList) );
	Flush();
	scratch->Release();
	return as;
}

ID3D12Resource* MakeBLAS( ID3D12Resource* vertexBuffer, UINT vertexFloats, ID3D12Resource* indexBuffer = nullptr, UINT indices = 0 )
{
	D3D12_RAYTRACING_GEOMETRY_DESC geometryDesc = {
		.Type = D3D12_RAYTRACING_GEOMETRY_TYPE_TRIANGLES,
		.Flags = D3D12_RAYTRACING_GEOMETRY_FLAG_OPAQUE,
		.Triangles = {
			.Transform3x4 = 0,
			.IndexFormat = indexBuffer ? DXGI_FORMAT_R32_UINT : DXGI_FORMAT_UNKNOWN,
			.VertexFormat = DXGI_FORMAT_R32G32B32_FLOAT,
			.IndexCount = indices, .VertexCount = vertexFloats / 3,
			.IndexBuffer = indexBuffer ? indexBuffer->GetGPUVirtualAddress() : 0,
			.VertexBuffer = {.StartAddress = vertexBuffer->GetGPUVirtualAddress(), .StrideInBytes = sizeof( float ) * 3}} };
	D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS inputs = {
		.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL,
		.Flags = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE,
		.NumDescs = 1, .DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY,
		.pGeometryDescs = &geometryDesc };
	return MakeAccelerationStructure( inputs );
}

void InitBottomLevel()
{
	// cubeBlas = MakeBLAS( cubeVB, std::size( verts ), cubeIB, std::size( indices ) );
	cubeBlas = MakeBLAS( cubeVB, triCount * 9, cubeIB, triCount * 3 );
}

ID3D12Resource* MakeTLAS( ID3D12Resource* instances, UINT numInstances, UINT64* updateScratchSize )
{
	D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS inputs = {
		.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL,
		.Flags = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_ALLOW_UPDATE,
		.NumDescs = numInstances,
		.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY,
		.InstanceDescs = instances->GetGPUVirtualAddress() };
	return MakeAccelerationStructure( inputs, updateScratchSize );
}

void UpdateTransforms()
{
	auto cube = DirectX::XMMatrixTranslation( 0, 0, 0 );
	DirectX::XMStoreFloat3x4( (DirectX::XMFLOAT3X4*)&instanceData[0].Transform, cube );
}

void InitScene()
{
	auto instancesDesc = BASIC_BUFFER_DESC;
	instancesDesc.Width = sizeof( D3D12_RAYTRACING_INSTANCE_DESC ) * NUM_INSTANCES;
	device->CreateCommittedResource( &UPLOAD_HEAP, D3D12_HEAP_FLAG_NONE,
		&instancesDesc, D3D12_RESOURCE_STATE_COMMON, nullptr, IID_PPV_ARGS( &instances ) );
	instances->Map( 0, nullptr, reinterpret_cast<void**>(&instanceData) );
	instanceData[0] = { .InstanceID = 0, .InstanceMask = 1, .AccelerationStructure = cubeBlas->GetGPUVirtualAddress() };
	UpdateTransforms();
}

void InitTopLevel()
{
	UINT64 updateScratchSize;
	tlas = MakeTLAS( instances, NUM_INSTANCES, &updateScratchSize );
	auto desc = BASIC_BUFFER_DESC;
	// WARP bug workaround: use 8 if the required size was reported as less
	desc.Width = std::max( updateScratchSize, 8ULL );
	desc.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;
	device->CreateCommittedResource( &DEFAULT_HEAP, D3D12_HEAP_FLAG_NONE, &desc, D3D12_RESOURCE_STATE_COMMON, nullptr, IID_PPV_ARGS( &tlasUpdateScratch ) );
}

void InitRootSignature()
{
	D3D12_DESCRIPTOR_RANGE uavRange = { .RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_UAV, .NumDescriptors = 1 };
	D3D12_ROOT_PARAMETER params[] = {
		{.ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE,
		 .DescriptorTable = {.NumDescriptorRanges = 1, .pDescriptorRanges = &uavRange}},
		{.ParameterType = D3D12_ROOT_PARAMETER_TYPE_SRV, .Descriptor = {.ShaderRegister = 0, .RegisterSpace = 0}},
		{.ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS,
		 .Constants = {.ShaderRegister = 0, .RegisterSpace = 0, .Num32BitValues = 12 },
		 .ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL }
	};
	D3D12_ROOT_SIGNATURE_DESC desc = { .NumParameters = std::size( params ), .pParameters = params };
	ID3DBlob* blob;
	D3D12SerializeRootSignature( &desc, D3D_ROOT_SIGNATURE_VERSION_1_0, &blob, nullptr );
	device->CreateRootSignature( 0, blob->GetBufferPointer(), blob->GetBufferSize(), IID_PPV_ARGS( &rootSignature ) );
	blob->Release();
}

void InitQueryHeap()
{
	D3D12_QUERY_HEAP_DESC queryHeapDesc = {};
	queryHeapDesc.Count = 8; // start and end, times four
	queryHeapDesc.Type = D3D12_QUERY_HEAP_TYPE_TIMESTAMP;
	queryHeapDesc.NodeMask = 0;
	device->CreateQueryHeap( &queryHeapDesc, IID_PPV_ARGS( &queryHeap ) );
	D3D12_HEAP_PROPERTIES heapProps = {};
	heapProps.Type = D3D12_HEAP_TYPE_READBACK;
	D3D12_RESOURCE_DESC bufferDesc = {};
	bufferDesc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
	bufferDesc.Width = 8 * sizeof( UINT64 );
	bufferDesc.Height = 1;
	bufferDesc.DepthOrArraySize = 1;
	bufferDesc.MipLevels = 1;
	bufferDesc.SampleDesc.Count = 1;
	bufferDesc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
	device->CreateCommittedResource(
		&heapProps, D3D12_HEAP_FLAG_NONE, &bufferDesc,
		D3D12_RESOURCE_STATE_COPY_DEST, nullptr, IID_PPV_ARGS( &queryResultBuffer )
	);
}

void InitPipeline()
{
	D3D12_DXIL_LIBRARY_DESC lib = { .DXILLibrary = {.pShaderBytecode = compiledShader, .BytecodeLength = std::size( compiledShader )} };
	D3D12_HIT_GROUP_DESC hitGroup = { .HitGroupExport = L"HitGroup", .Type = D3D12_HIT_GROUP_TYPE_TRIANGLES, .ClosestHitShaderImport = L"ClosestHit" };
	D3D12_RAYTRACING_SHADER_CONFIG shaderCfg = { .MaxPayloadSizeInBytes = 20, .MaxAttributeSizeInBytes = 8 };
	D3D12_GLOBAL_ROOT_SIGNATURE globalSig = { rootSignature };
	D3D12_RAYTRACING_PIPELINE_CONFIG pipelineCfg = { .MaxTraceRecursionDepth = 3 };
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
	auto idDesc = BASIC_BUFFER_DESC;
	idDesc.Width = NUM_SHADER_IDS * D3D12_RAYTRACING_SHADER_TABLE_BYTE_ALIGNMENT;
	device->CreateCommittedResource( &UPLOAD_HEAP, D3D12_HEAP_FLAG_NONE, &idDesc,
		D3D12_RESOURCE_STATE_COMMON, nullptr, IID_PPV_ARGS( &shaderIDs ) );
	ID3D12StateObjectProperties* props;
	pso->QueryInterface( &props );
	void* data;
	auto writeId = [&]( const wchar_t* name ) {
		void* id = props->GetShaderIdentifier( name );
		memcpy( data, id, D3D12_SHADER_IDENTIFIER_SIZE_IN_BYTES );
		data = static_cast<char*>(data) + D3D12_RAYTRACING_SHADER_TABLE_BYTE_ALIGNMENT;
		};
	shaderIDs->Map( 0, nullptr, &data );
	writeId( L"RayGeneration" );
	writeId( L"Miss" );
	writeId( L"HitGroup" );
	shaderIDs->Unmap( 0, nullptr );
	props->Release();
}

void UpdateScene()
{
	UpdateTransforms();
	D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC desc = {
		.DestAccelerationStructureData = tlas->GetGPUVirtualAddress(),
		.Inputs = {
			.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL,
			.Flags = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PERFORM_UPDATE,
			.NumDescs = NUM_INSTANCES,
			.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY,
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
	cmdAlloc->Reset();
	cmdList->Reset( cmdAlloc, nullptr );
	UpdateScene();
	cmdList->SetPipelineState1( pso );
	cmdList->SetComputeRootSignature( rootSignature );
	cmdList->SetComputeRoot32BitConstants( 2, 12, renderSettings, 0 );
	cmdList->SetDescriptorHeaps( 1, &uavHeap );
	auto uavTable = uavHeap->GetGPUDescriptorHandleForHeapStart();
	cmdList->SetComputeRootDescriptorTable( 0, uavTable ); // ←u0 ↓t0
	cmdList->SetComputeRootShaderResourceView( 1, tlas->GetGPUVirtualAddress() );
	auto rtDesc = renderTarget->GetDesc();
	D3D12_DISPATCH_RAYS_DESC dispatchDesc = {
		.RayGenerationShaderRecord = {
			.StartAddress = shaderIDs->GetGPUVirtualAddress(),
			.SizeInBytes = D3D12_SHADER_IDENTIFIER_SIZE_IN_BYTES
		},
		.MissShaderTable = {
			.StartAddress = shaderIDs->GetGPUVirtualAddress() + D3D12_RAYTRACING_SHADER_TABLE_BYTE_ALIGNMENT,
			.SizeInBytes = D3D12_SHADER_IDENTIFIER_SIZE_IN_BYTES,
			.StrideInBytes = 0 },
		.HitGroupTable = {
			.StartAddress = shaderIDs->GetGPUVirtualAddress() + 2 * D3D12_RAYTRACING_SHADER_TABLE_BYTE_ALIGNMENT,
			.SizeInBytes = D3D12_SHADER_IDENTIFIER_SIZE_IN_BYTES,
			.StrideInBytes = 32 },
		.Width = static_cast<UINT>(rtDesc.Width), .Height = rtDesc.Height, .Depth = 1 };
	static int readSlot = 0, writeSlot = 2;
	cmdList->EndQuery( queryHeap, D3D12_QUERY_TYPE_TIMESTAMP, writeSlot );
	cmdList->DispatchRays( &dispatchDesc );
	cmdList->EndQuery( queryHeap, D3D12_QUERY_TYPE_TIMESTAMP, writeSlot + 1 );
	UINT64 bufferOffset = writeSlot * sizeof( UINT64 );
	cmdList->ResolveQueryData( queryHeap, D3D12_QUERY_TYPE_TIMESTAMP, writeSlot, 2, queryResultBuffer, bufferOffset );
	swapChain->GetBuffer( swapChain->GetCurrentBackBufferIndex(), IID_PPV_ARGS( &backBuffer ) );
	auto barrier = []( auto* resource, auto before, auto after ) {
		D3D12_RESOURCE_BARRIER rb = {
			.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION,
			.Transition = {.pResource = resource, .StateBefore = before, .StateAfter = after}
		};
		cmdList->ResourceBarrier( 1, &rb );
		};
	barrier( renderTarget, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_COPY_SOURCE );
	barrier( backBuffer, D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_COPY_DEST );
	cmdList->CopyResource( backBuffer, renderTarget );
	barrier( backBuffer, D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_PRESENT );
	barrier( renderTarget, D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS );
	backBuffer->Release();
	cmdList->Close();
	cmdQueue->ExecuteCommandLists( 1, reinterpret_cast<ID3D12CommandList**>(&cmdList) );
	Flush();
#if 1
	// get GPU HW RT performance
	// WaitForGpu();
	UINT64* timestamps = nullptr;
	D3D12_RANGE readRange = { 0, 8 * sizeof( UINT64 ) };
	queryResultBuffer->Map( 0, &readRange, reinterpret_cast<void**>(&timestamps) );
	UINT64 startTimestamp = timestamps[readSlot];
	UINT64 endTimestamp = timestamps[readSlot + 1];
	D3D12_RANGE writeRange = { 0, 0 };
	queryResultBuffer->Unmap( 0, &writeRange );
	UINT64 frequency = 0;
	cmdQueue->GetTimestampFrequency( &frequency );
	double rtTime = static_cast<double>(endTimestamp - startTimestamp) / static_cast<double>(frequency);
	double raysPerSecond = (rtDesc.Width * rtDesc.Height) / rtTime;
	printf( "frame rendered: %.4fms (%.1fMRays/s)\n", (float)rtTime * 1000.0f, (float)(raysPerSecond / 1000000.0) );
#endif
	swapChain->Present( 1, 0 );
	readSlot = (readSlot + 2) & 7;
	writeSlot = (writeSlot + 2) & 7;
}

int main()
{
	// Alternatively, DPI_AWARENESS_CONTEXT_UNAWARE
	SetProcessDpiAwarenessContext( DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2 );
	WNDCLASSW wcw = { .lpfnWndProc = &WndProc, .hCursor = LoadCursor( nullptr, IDC_ARROW ), .lpszClassName = L"μDXR" };
	RegisterClassW( &wcw );
	HWND hwnd = CreateWindowExW( 0, L"μDXR", L"_DXR", WS_VISIBLE | WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT, 1024, 1024, 0, 0, 0, 0 );
	Init( hwnd );
	// device->SetStablePowerState( true );
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