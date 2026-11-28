#include "headers.h"

#ifdef ENABLE_VULKAN

#include "../external/vulkan/vk_min.h" // the declarations used here, from the Khronos headers (tools/gen_vk_min.py)
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <dlfcn.h>
#endif
#include <vector>

extern FILE* csv;

namespace tinybvh
{

namespace
{

// SPIR-V of kernels/traverse_vk.comp (kernels/compile_vk.bat)
const uint32_t spvNearest[] =
#include "../kernels/traverse_vk_nearest.inc"
;
const uint32_t spvAny[] =
#include "../kernels/traverse_vk_any.inc"
;

// entry points, fetched at run time from the loader
#define VK_INSTANCE_FUNCS VKF( vkDestroyInstance ) VKF( vkEnumeratePhysicalDevices ) VKF( vkGetPhysicalDeviceProperties ) \
	VKF( vkGetPhysicalDeviceProperties2 ) VKF( vkGetPhysicalDeviceFeatures2 ) VKF( vkGetPhysicalDeviceMemoryProperties ) \
	VKF( vkGetPhysicalDeviceQueueFamilyProperties ) VKF( vkEnumerateDeviceExtensionProperties ) VKF( vkCreateDevice ) \
	VKF( vkGetDeviceProcAddr )
#define VK_DEVICE_FUNCS VKF( vkGetDeviceQueue ) VKF( vkCreateCommandPool ) VKF( vkAllocateCommandBuffers ) \
	VKF( vkFreeCommandBuffers ) VKF( vkBeginCommandBuffer ) VKF( vkEndCommandBuffer ) VKF( vkQueueSubmit ) VKF( vkQueueWaitIdle ) \
	VKF( vkCreateBuffer ) VKF( vkDestroyBuffer ) VKF( vkGetBufferMemoryRequirements ) VKF( vkAllocateMemory ) VKF( vkFreeMemory ) \
	VKF( vkBindBufferMemory ) VKF( vkMapMemory ) VKF( vkGetBufferDeviceAddress ) VKF( vkCmdCopyBuffer ) VKF( vkCmdPipelineBarrier ) \
	VKF( vkCreateQueryPool ) VKF( vkDestroyQueryPool ) VKF( vkCmdResetQueryPool ) VKF( vkCmdWriteTimestamp ) VKF( vkGetQueryPoolResults ) \
	VKF( vkCreateShaderModule ) VKF( vkDestroyShaderModule ) VKF( vkCreatePipelineLayout ) VKF( vkCreateComputePipelines ) \
	VKF( vkCmdBindPipeline ) VKF( vkCmdPushConstants ) VKF( vkCmdDispatch ) VKF( vkCreateAccelerationStructureKHR ) \
	VKF( vkDestroyAccelerationStructureKHR ) VKF( vkGetAccelerationStructureBuildSizesKHR ) \
	VKF( vkCmdBuildAccelerationStructuresKHR ) VKF( vkGetAccelerationStructureDeviceAddressKHR ) \
	VKF( vkCmdWriteAccelerationStructuresPropertiesKHR ) VKF( vkCmdCopyAccelerationStructureKHR )
#define VKF( f ) PFN_##f f = 0;
VKF( vkGetInstanceProcAddr ) VKF( vkCreateInstance ) VKF( vkEnumerateInstanceLayerProperties )
VK_INSTANCE_FUNCS VK_DEVICE_FUNCS
#undef VKF

VkInstance instance = 0;
VkPhysicalDevice pdev = 0;
VkDevice device = 0;
VkQueue queue = 0;
VkCommandPool cmdPool = 0;
VkPhysicalDeviceMemoryProperties memProps = {};
VkPipelineLayout pipeLayout = 0;
VkPipeline pipeNearest = 0, pipeAny = 0;
VkQueryPool queryPool = 0;
double timestampPeriod = 1; // ns per tick
uint64_t timestampMask = ~0ull; // the counter's valid bits: a difference across a wrap stays correct
uint32_t queueFamily = 0, scratchAlign = 256;
bool available = false;
constexpr uint32_t MAX_RUNS = 64;

bool Fail( const char* msg ) { printf( "Vulkan: %s\n", msg ); return false; }
#define VK_TRY( x ) do { if ((x) != VK_SUCCESS) { printf( "Vulkan: %s failed (line %d)\n", #x, __LINE__ ); exit( 1 ); } } while (0)

struct Buf { VkBuffer buf = 0; VkDeviceMemory mem = 0; VkDeviceAddress addr = 0; void* map = 0; VkDeviceSize size = 0; };

Buf MakeBuf( const VkDeviceSize size, const VkBufferUsageFlags usage, const bool hostVisible )
{
	Buf b;
	b.size = size;
	VkBufferCreateInfo bci = { VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
	bci.size = size, bci.usage = usage | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
	VK_TRY( vkCreateBuffer( device, &bci, 0, &b.buf ) );
	VkMemoryRequirements req;
	vkGetBufferMemoryRequirements( device, b.buf, &req );
	const VkMemoryPropertyFlags want = hostVisible ? (VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT) : VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;
	uint32_t type = ~0u;
	for (uint32_t i = 0; i < memProps.memoryTypeCount && type == ~0u; i++)
		if ((req.memoryTypeBits & (1u << i)) && (memProps.memoryTypes[i].propertyFlags & want) == want) type = i;
	if (type == ~0u) Fail( "no suitable memory type" ), exit( 1 );
	VkMemoryAllocateFlagsInfo fi = { VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_FLAGS_INFO };
	fi.flags = VK_MEMORY_ALLOCATE_DEVICE_ADDRESS_BIT;
	VkMemoryAllocateInfo mai = { VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
	mai.pNext = &fi, mai.allocationSize = req.size, mai.memoryTypeIndex = type;
	VK_TRY( vkAllocateMemory( device, &mai, 0, &b.mem ) );
	VK_TRY( vkBindBufferMemory( device, b.buf, b.mem, 0 ) );
	VkBufferDeviceAddressInfo ai = { VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO };
	ai.buffer = b.buf;
	b.addr = vkGetBufferDeviceAddress( device, &ai );
	if (hostVisible) VK_TRY( vkMapMemory( device, b.mem, 0, size, 0, &b.map ) );
	return b;
}

void FreeBuf( Buf& b )
{
	if (b.buf) vkDestroyBuffer( device, b.buf, 0 );
	if (b.mem) vkFreeMemory( device, b.mem, 0 ); // also unmaps
	b = Buf();
}

void Barrier( VkCommandBuffer cb );
VkCommandBuffer Begin() // starts with a full barrier: earlier submissions' writes become visible
{
	VkCommandBufferAllocateInfo cai = { VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
	cai.commandPool = cmdPool, cai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY, cai.commandBufferCount = 1;
	VkCommandBuffer cb;
	VK_TRY( vkAllocateCommandBuffers( device, &cai, &cb ) );
	VkCommandBufferBeginInfo bi = { VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
	bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
	VK_TRY( vkBeginCommandBuffer( cb, &bi ) );
	Barrier( cb );
	return cb;
}

void Submit( VkCommandBuffer cb ) // and wait
{
	VK_TRY( vkEndCommandBuffer( cb ) );
	VkSubmitInfo si = { VK_STRUCTURE_TYPE_SUBMIT_INFO };
	si.commandBufferCount = 1, si.pCommandBuffers = &cb;
	VK_TRY( vkQueueSubmit( queue, 1, &si, 0 ) );
	VK_TRY( vkQueueWaitIdle( queue ) );
	vkFreeCommandBuffers( device, cmdPool, 1, &cb );
}

void Barrier( VkCommandBuffer cb ) // everything before, before everything after
{
	VkMemoryBarrier mb = { VK_STRUCTURE_TYPE_MEMORY_BARRIER };
	mb.srcAccessMask = VK_ACCESS_MEMORY_WRITE_BIT, mb.dstAccessMask = VK_ACCESS_MEMORY_READ_BIT | VK_ACCESS_MEMORY_WRITE_BIT;
	vkCmdPipelineBarrier( cb, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 1, &mb, 0, 0, 0, 0 );
}

void HostBarrier( VkCommandBuffer cb ) // transfers before, host reads after
{
	VkMemoryBarrier mb = { VK_STRUCTURE_TYPE_MEMORY_BARRIER };
	mb.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT, mb.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
	vkCmdPipelineBarrier( cb, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_HOST_BIT, 0, 1, &mb, 0, 0, 0, 0 );
}

// copy host data into a device-local buffer through a staging buffer
void Upload( const Buf& dst, const void* data, const VkDeviceSize size )
{
	Buf s = MakeBuf( size, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, true );
	memcpy( s.map, data, (size_t)size );
	VkCommandBuffer cb = Begin();
	VkBufferCopy c = { 0, 0, size };
	vkCmdCopyBuffer( cb, s.buf, dst.buf, 1, &c );
	Submit( cb );
	FreeBuf( s );
}

bool HasExtension( const std::vector<VkExtensionProperties>& list, const char* name )
{
	for (const VkExtensionProperties& e : list) if (!strcmp( e.extensionName, name )) return true;
	return false;
}

VkPipeline MakePipeline( const uint32_t* code, const size_t bytes )
{
	VkShaderModuleCreateInfo smi = { VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO };
	smi.codeSize = bytes, smi.pCode = code;
	VkShaderModule sm;
	VK_TRY( vkCreateShaderModule( device, &smi, 0, &sm ) );
	VkComputePipelineCreateInfo cpi = { VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO };
	cpi.stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
	cpi.stage.stage = VK_SHADER_STAGE_COMPUTE_BIT, cpi.stage.module = sm, cpi.stage.pName = "main";
	cpi.layout = pipeLayout;
	VkPipeline p;
	VK_TRY( vkCreateComputePipelines( device, 0, 1, &cpi, 0, &p ) );
	vkDestroyShaderModule( device, sm, 0 );
	return p;
}

} // namespace

bool VulkanAvailable() { return available; }

bool InitVulkan( const char* gpuVendor )
{
	static bool tried = false;
	if (tried) return available;
	tried = true;
	// the loader comes with the GPU driver
#ifdef _WIN32
	HMODULE lib = LoadLibraryA( "vulkan-1.dll" );
	if (lib) vkGetInstanceProcAddr = (PFN_vkGetInstanceProcAddr)(void*)GetProcAddress( lib, "vkGetInstanceProcAddr" );
#else
	void* lib = dlopen( "libvulkan.so.1", RTLD_NOW );
	if (lib) vkGetInstanceProcAddr = (PFN_vkGetInstanceProcAddr)dlsym( lib, "vkGetInstanceProcAddr" );
#endif
	if (!vkGetInstanceProcAddr) return Fail( "no Vulkan loader found" );
	vkCreateInstance = (PFN_vkCreateInstance)vkGetInstanceProcAddr( 0, "vkCreateInstance" );
	vkEnumerateInstanceLayerProperties = (PFN_vkEnumerateInstanceLayerProperties)vkGetInstanceProcAddr( 0, "vkEnumerateInstanceLayerProperties" );
	if (!vkCreateInstance || !vkEnumerateInstanceLayerProperties) return Fail( "incomplete loader" );
	// validation only on request (TINYBVH_VK_VALIDATION set), and only when installed (it comes with the SDK)
	const char* layer = "VK_LAYER_KHRONOS_validation";
	bool validate = false;
	if (getenv( "TINYBVH_VK_VALIDATION" ))
	{
		uint32_t n = 0;
		vkEnumerateInstanceLayerProperties( &n, 0 );
		std::vector<VkLayerProperties> layers( n );
		vkEnumerateInstanceLayerProperties( &n, layers.data() );
		for (const VkLayerProperties& l : layers) validate |= !strcmp( l.layerName, layer );
		if (!validate) printf( "Vulkan: validation layer requested but not installed\n" );
	}
	VkApplicationInfo app = { VK_STRUCTURE_TYPE_APPLICATION_INFO };
	app.pApplicationName = "tiny_bvh_benchmark", app.apiVersion = VK_API_VERSION_1_2;
	VkInstanceCreateInfo ici = { VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO };
	ici.pApplicationInfo = &app, ici.enabledLayerCount = validate ? 1 : 0, ici.ppEnabledLayerNames = &layer;
	if (vkCreateInstance( &ici, 0, &instance ) != VK_SUCCESS) return Fail( "vkCreateInstance failed" );
#define VKF( f ) f = (PFN_##f)vkGetInstanceProcAddr( instance, #f ); if (!f) return Fail( "missing " #f );
	VK_INSTANCE_FUNCS
#undef VKF
	// a GPU with ray queries: the requested vendor, else a discrete GPU, else the first
	uint32_t n = 0;
	vkEnumeratePhysicalDevices( instance, &n, 0 );
	std::vector<VkPhysicalDevice> devs( n );
	vkEnumeratePhysicalDevices( instance, &n, devs.data() );
	int best = -1, bestScore = -1;
	VkPhysicalDeviceProperties props = {};
	for (uint32_t i = 0; i < n; i++)
	{
		VkPhysicalDeviceProperties p;
		vkGetPhysicalDeviceProperties( devs[i], &p );
		uint32_t e = 0;
		vkEnumerateDeviceExtensionProperties( devs[i], 0, &e, 0 );
		std::vector<VkExtensionProperties> ext( e );
		vkEnumerateDeviceExtensionProperties( devs[i], 0, &e, ext.data() );
		if (p.apiVersion < VK_API_VERSION_1_2 || !HasExtension( ext, VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME ) ||
			!HasExtension( ext, VK_KHR_RAY_QUERY_EXTENSION_NAME ) || !HasExtension( ext, VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME )) continue;
		// features: device addresses, acceleration structures, ray queries, 64-bit integers
		VkPhysicalDeviceRayQueryFeaturesKHR rq = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_QUERY_FEATURES_KHR };
		VkPhysicalDeviceAccelerationStructureFeaturesKHR asf = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR };
		VkPhysicalDeviceVulkan12Features v12 = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES };
		VkPhysicalDeviceFeatures2 f2 = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2 };
		f2.pNext = &v12, v12.pNext = &asf, asf.pNext = &rq;
		vkGetPhysicalDeviceFeatures2( devs[i], &f2 );
		if (!v12.bufferDeviceAddress || !asf.accelerationStructure || !rq.rayQuery || !f2.features.shaderInt64) continue;
		int score = p.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU ? 1 : 0;
		if (gpuVendor)
		{
			const bool match = (!strcmp( gpuVendor, "NVIDIA" ) && p.vendorID == 0x10de) || (!strcmp( gpuVendor, "AMD" ) && p.vendorID == 0x1002) ||
				(!strcmp( gpuVendor, "Intel" ) && p.vendorID == 0x8086) || strstr( p.deviceName, gpuVendor );
			score += match ? 2 : 0;
		}
		if (score > bestScore) best = (int)i, bestScore = score, props = p;
	}
	if (best < 0) return Fail( "no GPU with ray queries" );
	pdev = devs[best];
	VkPhysicalDeviceAccelerationStructurePropertiesKHR asProps = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_PROPERTIES_KHR };
	VkPhysicalDeviceProperties2 p2 = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2 };
	p2.pNext = &asProps;
	vkGetPhysicalDeviceProperties2( pdev, &p2 );
	scratchAlign = tinybvh_max( asProps.minAccelerationStructureScratchOffsetAlignment, 1u );
	timestampPeriod = props.limits.timestampPeriod;
	vkGetPhysicalDeviceMemoryProperties( pdev, &memProps );
	// a queue family with compute and timestamps
	vkGetPhysicalDeviceQueueFamilyProperties( pdev, &n, 0 );
	std::vector<VkQueueFamilyProperties> families( n );
	vkGetPhysicalDeviceQueueFamilyProperties( pdev, &n, families.data() );
	queueFamily = ~0u;
	for (uint32_t i = 0; i < n && queueFamily == ~0u; i++)
		if ((families[i].queueFlags & VK_QUEUE_COMPUTE_BIT) && families[i].timestampValidBits > 0) queueFamily = i;
	if (queueFamily == ~0u) return Fail( "no compute queue with timestamps" );
	const uint32_t bits = families[queueFamily].timestampValidBits;
	timestampMask = bits >= 64 ? ~0ull : (1ull << bits) - 1;
	VkPhysicalDeviceRayQueryFeaturesKHR rqOn = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_QUERY_FEATURES_KHR };
	VkPhysicalDeviceAccelerationStructureFeaturesKHR asOn = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR };
	VkPhysicalDeviceVulkan12Features v12On = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES };
	VkPhysicalDeviceFeatures2 f2On = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2 };
	f2On.pNext = &v12On, v12On.pNext = &asOn, asOn.pNext = &rqOn;
	v12On.bufferDeviceAddress = rqOn.rayQuery = asOn.accelerationStructure = f2On.features.shaderInt64 = 1;
	const char* exts[] = { VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME, VK_KHR_RAY_QUERY_EXTENSION_NAME, VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME };
	const float priority = 1;
	VkDeviceQueueCreateInfo qci = { VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO };
	qci.queueFamilyIndex = queueFamily, qci.queueCount = 1, qci.pQueuePriorities = &priority;
	VkDeviceCreateInfo dci = { VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO };
	dci.pNext = &f2On, dci.queueCreateInfoCount = 1, dci.pQueueCreateInfos = &qci, dci.enabledExtensionCount = 3, dci.ppEnabledExtensionNames = exts;
	if (vkCreateDevice( pdev, &dci, 0, &device ) != VK_SUCCESS) return Fail( "vkCreateDevice failed" );
#define VKF( f ) f = (PFN_##f)vkGetDeviceProcAddr( device, #f ); if (!f) return Fail( "missing " #f );
	VK_DEVICE_FUNCS
#undef VKF
	vkGetDeviceQueue( device, queueFamily, 0, &queue );
	VkCommandPoolCreateInfo cpci = { VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO };
	cpci.queueFamilyIndex = queueFamily;
	VK_TRY( vkCreateCommandPool( device, &cpci, 0, &cmdPool ) );
	VkQueryPoolCreateInfo qpi = { VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO };
	qpi.queryType = VK_QUERY_TYPE_TIMESTAMP, qpi.queryCount = MAX_RUNS * 2;
	VK_TRY( vkCreateQueryPool( device, &qpi, 0, &queryPool ) );
	// the two trace pipelines; push constants: TLAS, rays, hits (device addresses), ray count
	VkPushConstantRange pcr = { VK_SHADER_STAGE_COMPUTE_BIT, 0, 32 };
	VkPipelineLayoutCreateInfo pli = { VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
	pli.pushConstantRangeCount = 1, pli.pPushConstantRanges = &pcr;
	VK_TRY( vkCreatePipelineLayout( device, &pli, 0, &pipeLayout ) );
	pipeNearest = MakePipeline( spvNearest, sizeof( spvNearest ) );
	pipeAny = MakePipeline( spvAny, sizeof( spvAny ) );
	printf( "Vulkan ray queries on %s%s\n", props.deviceName, validate ? " (validation on)" : "" );
	printf( "----------------------------------------------------------------\n" );
	if (csv) fprintf( csv, "vulkan gpu:,%s,driver,0x%x\n", props.deviceName, props.driverVersion ), fflush( csv );
	available = true;
	return true;
}

struct VulkanRT::Impl
{
	Buf verts, indices, instances, blasMem, tlasMem, scratch, rays, hits;
	VkAccelerationStructureKHR blas = 0, tlas = 0;
	uint32_t primCount = 0, vertCount = 0, rayCount = 0;
	bool indexed = false;
	float xf[12] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0 };
	VkDeviceSize blasBytes = 0;
	void FreeAS()
	{
		if (blas) vkDestroyAccelerationStructureKHR( device, blas, 0 ), blas = 0;
		if (tlas) vkDestroyAccelerationStructureKHR( device, tlas, 0 ), tlas = 0;
		FreeBuf( blasMem ), FreeBuf( tlasMem );
	}
	VkAccelerationStructureKHR MakeAS( Buf& mem, const VkDeviceSize size, const VkAccelerationStructureTypeKHR type )
	{
		mem = MakeBuf( size, VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR, false );
		VkAccelerationStructureCreateInfoKHR ci = { VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR };
		ci.buffer = mem.buf, ci.size = size, ci.type = type;
		VkAccelerationStructureKHR as;
		VK_TRY( vkCreateAccelerationStructureKHR( device, &ci, 0, &as ) );
		return as;
	}
	VkDeviceAddress Scratch( const VkDeviceSize size ) // kept between builds
	{
		if (scratch.size < size + scratchAlign)
			FreeBuf( scratch ), scratch = MakeBuf( size + scratchAlign, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, false );
		return (scratch.addr + scratchAlign - 1) & ~(VkDeviceAddress)(scratchAlign - 1);
	}
};

VulkanRT::VulkanRT() { impl = new Impl(); }

VulkanRT::~VulkanRT()
{
	if (available)
	{
		impl->FreeAS();
		FreeBuf( impl->verts ), FreeBuf( impl->indices ), FreeBuf( impl->instances ), FreeBuf( impl->scratch );
		FreeBuf( impl->rays ), FreeBuf( impl->hits );
	}
	delete impl;
}

uint64_t VulkanRT::BlasBytes() const { return impl->blasBytes; }

void VulkanRT::SetGeometry( const bvhvec4* verts, const uint32_t* indices, const uint32_t primCount, const float* objectToWorld )
{
	if (!available) return;
	Impl& m = *impl;
	FreeBuf( m.verts ), FreeBuf( m.indices );
	m.primCount = primCount, m.indexed = indices != 0;
	uint32_t vc = primCount * 3;
	if (indices) { vc = 0; for (uint32_t i = 0; i < primCount * 3; i++) vc = tinybvh_max( vc, indices[i] + 1 ); }
	m.vertCount = vc;
	const VkBufferUsageFlags input = VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
	m.verts = MakeBuf( (VkDeviceSize)vc * 16, input, false );
	Upload( m.verts, verts, (VkDeviceSize)vc * 16 );
	if (indices) m.indices = MakeBuf( (VkDeviceSize)primCount * 12, input, false ), Upload( m.indices, indices, (VkDeviceSize)primCount * 12 );
	const float identity[12] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0 };
	memcpy( m.xf, objectToWorld ? objectToWorld : identity, sizeof( m.xf ) );
}

void VulkanRT::Build( const int quality )
{
	if (!available) return;
	Impl& m = *impl;
	m.FreeAS();
	// BLAS over the triangles
	VkAccelerationStructureGeometryKHR g = { VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR };
	g.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR, g.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;
	VkAccelerationStructureGeometryTrianglesDataKHR& t = g.geometry.triangles;
	t.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR;
	t.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT, t.vertexData.deviceAddress = m.verts.addr, t.vertexStride = 16;
	t.maxVertex = m.vertCount - 1, t.indexType = m.indexed ? VK_INDEX_TYPE_UINT32 : VK_INDEX_TYPE_NONE_KHR;
	if (m.indexed) t.indexData.deviceAddress = m.indices.addr;
	VkBuildAccelerationStructureFlagsKHR flags = VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_COMPACTION_BIT_KHR;
	if (quality == 0) flags |= VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_BUILD_BIT_KHR;
	if (quality == 2) flags |= VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
	VkAccelerationStructureBuildGeometryInfoKHR bi = { VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR };
	bi.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR, bi.flags = flags, bi.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
	bi.geometryCount = 1, bi.pGeometries = &g;
	VkAccelerationStructureBuildSizesInfoKHR sz = { VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR };
	vkGetAccelerationStructureBuildSizesKHR( device, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR, &bi, &m.primCount, &sz );
	Buf full;
	VkAccelerationStructureKHR fullAS = m.MakeAS( full, sz.accelerationStructureSize, VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR );
	bi.dstAccelerationStructure = fullAS, bi.scratchData.deviceAddress = m.Scratch( sz.buildScratchSize );
	VkAccelerationStructureBuildRangeInfoKHR range = { m.primCount, 0, 0, 0 };
	const VkAccelerationStructureBuildRangeInfoKHR* ranges = &range;
	VkQueryPoolCreateInfo qpi = { VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO };
	qpi.queryType = VK_QUERY_TYPE_ACCELERATION_STRUCTURE_COMPACTED_SIZE_KHR, qpi.queryCount = 1;
	VkQueryPool sizePool;
	VK_TRY( vkCreateQueryPool( device, &qpi, 0, &sizePool ) );
	VkCommandBuffer cb = Begin();
	vkCmdResetQueryPool( cb, sizePool, 0, 1 );
	vkCmdBuildAccelerationStructuresKHR( cb, 1, &bi, &ranges );
	Barrier( cb );
	vkCmdWriteAccelerationStructuresPropertiesKHR( cb, 1, &fullAS, VK_QUERY_TYPE_ACCELERATION_STRUCTURE_COMPACTED_SIZE_KHR, sizePool, 0 );
	Submit( cb );
	VkDeviceSize compact = 0;
	VK_TRY( vkGetQueryPoolResults( device, sizePool, 0, 1, sizeof( compact ), &compact, sizeof( compact ), VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT ) );
	vkDestroyQueryPool( device, sizePool, 0 );
	// compacted copy
	m.blas = m.MakeAS( m.blasMem, compact, VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR );
	m.blasBytes = compact;
	VkCopyAccelerationStructureInfoKHR copy = { VK_STRUCTURE_TYPE_COPY_ACCELERATION_STRUCTURE_INFO_KHR };
	copy.src = fullAS, copy.dst = m.blas, copy.mode = VK_COPY_ACCELERATION_STRUCTURE_MODE_COMPACT_KHR;
	cb = Begin();
	vkCmdCopyAccelerationStructureKHR( cb, &copy );
	Submit( cb );
	vkDestroyAccelerationStructureKHR( device, fullAS, 0 );
	FreeBuf( full );
	// TLAS: one instance with the given transform
	VkAccelerationStructureDeviceAddressInfoKHR ai = { VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR };
	ai.accelerationStructure = m.blas;
	VkAccelerationStructureInstanceKHR inst = {};
	memcpy( inst.transform.matrix, m.xf, sizeof( m.xf ) );
	inst.mask = 0xff, inst.flags = VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR;
	inst.accelerationStructureReference = vkGetAccelerationStructureDeviceAddressKHR( device, &ai );
	if (!m.instances.buf) m.instances = MakeBuf( sizeof( inst ), VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR, true );
	memcpy( m.instances.map, &inst, sizeof( inst ) );
	VkAccelerationStructureGeometryKHR ig = { VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR };
	ig.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR, ig.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;
	ig.geometry.instances.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR;
	ig.geometry.instances.data.deviceAddress = m.instances.addr;
	VkAccelerationStructureBuildGeometryInfoKHR ti = { VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR };
	ti.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR, ti.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
	ti.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR, ti.geometryCount = 1, ti.pGeometries = &ig;
	const uint32_t one = 1;
	VkAccelerationStructureBuildSizesInfoKHR tsz = { VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR };
	vkGetAccelerationStructureBuildSizesKHR( device, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR, &ti, &one, &tsz );
	m.tlas = m.MakeAS( m.tlasMem, tsz.accelerationStructureSize, VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR );
	ti.dstAccelerationStructure = m.tlas, ti.scratchData.deviceAddress = m.Scratch( tsz.buildScratchSize );
	VkAccelerationStructureBuildRangeInfoKHR trange = { 1, 0, 0, 0 };
	const VkAccelerationStructureBuildRangeInfoKHR* tranges = &trange;
	cb = Begin();
	vkCmdBuildAccelerationStructuresKHR( cb, 1, &ti, &tranges );
	Submit( cb );
}

float VulkanRT::Trace( const char* rayData, const int rayCount, const bool anyHit, const int runs, float* hits )
{
	if (!available || !impl->tlas) return 0;
	Impl& m = *impl;
	const int R = tinybvh_min( tinybvh_max( runs, 1 ), (int)MAX_RUNS );
	// rays as 32 bytes each (O, tmax; D, 0), uploaded outside the measurement
	if (m.rayCount != (uint32_t)rayCount)
	{
		FreeBuf( m.rays ), FreeBuf( m.hits );
		m.rays = MakeBuf( (VkDeviceSize)rayCount * 32, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT, false );
		m.hits = MakeBuf( (VkDeviceSize)rayCount * 16, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT, false );
		m.rayCount = rayCount;
	}
	std::vector<float> packed( (size_t)rayCount * 8 );
	for (int i = 0; i < rayCount; i++)
	{
		const Ray& r = *(const Ray*)(rayData + (size_t)i * 64);
		float* d = &packed[(size_t)i * 8];
		d[0] = r.O.x, d[1] = r.O.y, d[2] = r.O.z, d[3] = r.hit.t, d[4] = r.D.x, d[5] = r.D.y, d[6] = r.D.z, d[7] = 0;
	}
	Upload( m.rays, packed.data(), packed.size() * 4 );
	VkAccelerationStructureDeviceAddressInfoKHR ai = { VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR };
	ai.accelerationStructure = m.tlas;
	struct { uint64_t tlas, rays, hits; uint32_t count, pad; } pc = { vkGetAccelerationStructureDeviceAddressKHR( device, &ai ), m.rays.addr, m.hits.addr, (uint32_t)rayCount, 0 };
	const uint32_t groups = ((uint32_t)rayCount + 63) / 64;
	auto Record = [&]( VkCommandBuffer cb ) {
		vkCmdBindPipeline( cb, VK_PIPELINE_BIND_POINT_COMPUTE, anyHit ? pipeAny : pipeNearest );
		vkCmdPushConstants( cb, pipeLayout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof( pc ), &pc );
	};
	// warm up for 0.5 s of GPU time (laptop GPUs need that to reach full clocks; 20 short dispatches are not
	// enough), then 'runs' dispatches with a timestamp pair each; a barrier between dispatches keeps them apart
	uint64_t ts[MAX_RUNS * 2];
	VkCommandBuffer cb;
	for (double warm = 0; warm < 0.5;)
	{
		cb = Begin();
		Record( cb );
		vkCmdResetQueryPool( cb, queryPool, 0, 2 );
		vkCmdWriteTimestamp( cb, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, queryPool, 0 );
		for (int i = 0; i < 20; i++) vkCmdDispatch( cb, groups, 1, 1 ), Barrier( cb );
		vkCmdWriteTimestamp( cb, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, queryPool, 1 );
		Submit( cb );
		VK_TRY( vkGetQueryPoolResults( device, queryPool, 0, 2, sizeof( ts ), ts, 8, VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT ) );
		warm += tinybvh_max( (double)((ts[1] - ts[0]) & timestampMask) * timestampPeriod * 1e-9, 1e-3 ); // at least 1 ms per batch: ends
	}
	cb = Begin();
	Record( cb );
	vkCmdResetQueryPool( cb, queryPool, 0, R * 2 );
	for (int i = 0; i < R; i++)
	{
		Barrier( cb );
		vkCmdWriteTimestamp( cb, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, queryPool, i * 2 );
		vkCmdDispatch( cb, groups, 1, 1 );
		vkCmdWriteTimestamp( cb, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, queryPool, i * 2 + 1 );
	}
	Submit( cb );
	VK_TRY( vkGetQueryPoolResults( device, queryPool, 0, R * 2, sizeof( ts ), ts, 8, VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT ) );
	uint64_t fastest = ~0ull;
	for (int i = 0; i < R; i++) fastest = tinybvh_min( fastest, (ts[i * 2 + 1] - ts[i * 2]) & timestampMask );
	if (hits)
	{
		Buf s = MakeBuf( (VkDeviceSize)rayCount * 16, VK_BUFFER_USAGE_TRANSFER_DST_BIT, true );
		cb = Begin();
		VkBufferCopy c = { 0, 0, (VkDeviceSize)rayCount * 16 };
		vkCmdCopyBuffer( cb, m.hits.buf, s.buf, 1, &c );
		HostBarrier( cb );
		Submit( cb );
		memcpy( hits, s.map, (size_t)rayCount * 16 );
		FreeBuf( s );
	}
	return (float)((double)fastest * timestampPeriod * 1e-9);
}

} // namespace tinybvh

#else // no Vulkan: the layout reports itself unavailable

namespace tinybvh
{
bool InitVulkan( const char* ) { return false; }
bool VulkanAvailable() { return false; }
VulkanRT::VulkanRT() {}
VulkanRT::~VulkanRT() {}
void VulkanRT::SetGeometry( const bvhvec4*, const uint32_t*, const uint32_t, const float* ) {}
void VulkanRT::Build( const int ) {}
float VulkanRT::Trace( const char*, const int, const bool, const int, float* ) { return 0; }
uint64_t VulkanRT::BlasBytes() const { return 0; }
} // namespace tinybvh

#endif

// EOF
