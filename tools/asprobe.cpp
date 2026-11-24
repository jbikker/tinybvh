// asprobe: build tiny Vulkan acceleration structures from probe geometry and dump their bytes.
// Black-box study of driver AS formats via public API output only (no driver disassembly).
//
//   asprobe <gpu> info
//   asprobe <gpu> dump  <scene> <outprefix> [options]   -> .raw.bin .compact.bin .ser.bin .txt
//   asprobe <gpu> sweep <kind> <n0> <n1> <step> [options]  -> CSV on stdout
//
// <gpu>: substring of the device name, e.g. "890M" or "RTX".
// <scene>: tri1 | grid:N | strip:N | fan:N | soup:N | same:N | quad:N | txt:path (9 floats per line) | bin:path (tinybvh .bin)
// options: -f flags   comma list of fasttrace,fastbuild,lowmem,compact,update,dataaccess (default fasttrace,compact)
//          -fp16      store vertices as R16G16B16A16_SFLOAT
//          -idx       indexed (vertices welded on exact position)
//          -geoms G   split the triangles over G geometries
//          -noopaque  omit VK_GEOMETRY_OPAQUE_BIT_KHR
//
// Build: g++ -std=c++20 -O2 -I"%VULKAN_SDK%/Include" tools/asprobe.cpp -o asprobe.exe

#define VK_NO_PROTOTYPES
#include <vulkan/vulkan.h>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <dlfcn.h>
#endif
#include <cstdarg>
#include <cstdio>
#include <tuple>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cmath>
#include <string>
#include <vector>
#include <map>

#define VKF( name ) static PFN_##name name = 0;
#define INSTANCE_FUNCS \
	VKF( vkEnumeratePhysicalDevices ) VKF( vkGetPhysicalDeviceProperties ) VKF( vkGetPhysicalDeviceProperties2 ) \
	VKF( vkGetPhysicalDeviceFeatures2 ) VKF( vkGetPhysicalDeviceMemoryProperties ) VKF( vkGetPhysicalDeviceQueueFamilyProperties ) \
	VKF( vkEnumerateDeviceExtensionProperties ) VKF( vkCreateDevice ) VKF( vkGetDeviceProcAddr ) VKF( vkDestroyInstance ) \
	VKF( vkGetPhysicalDeviceFormatProperties )
#define DEVICE_FUNCS \
	VKF( vkGetDeviceQueue ) VKF( vkCreateBuffer ) VKF( vkDestroyBuffer ) VKF( vkGetBufferMemoryRequirements ) \
	VKF( vkAllocateMemory ) VKF( vkFreeMemory ) VKF( vkBindBufferMemory ) VKF( vkMapMemory ) VKF( vkGetBufferDeviceAddress ) \
	VKF( vkCreateCommandPool ) VKF( vkAllocateCommandBuffers ) VKF( vkFreeCommandBuffers ) VKF( vkBeginCommandBuffer ) \
	VKF( vkEndCommandBuffer ) VKF( vkQueueSubmit ) VKF( vkQueueWaitIdle ) VKF( vkCmdPipelineBarrier ) VKF( vkCmdFillBuffer ) \
	VKF( vkCmdCopyBuffer ) VKF( vkCreateQueryPool ) VKF( vkDestroyQueryPool ) VKF( vkCmdResetQueryPool ) \
	VKF( vkGetQueryPoolResults ) VKF( vkDestroyDevice ) VKF( vkDeviceWaitIdle ) \
	VKF( vkGetAccelerationStructureBuildSizesKHR ) VKF( vkCreateAccelerationStructureKHR ) \
	VKF( vkDestroyAccelerationStructureKHR ) VKF( vkCmdBuildAccelerationStructuresKHR ) \
	VKF( vkCmdCopyAccelerationStructureKHR ) VKF( vkCmdCopyAccelerationStructureToMemoryKHR ) \
	VKF( vkCmdWriteAccelerationStructuresPropertiesKHR ) VKF( vkGetAccelerationStructureDeviceAddressKHR )
VKF( vkGetInstanceProcAddr ) VKF( vkCreateInstance )
INSTANCE_FUNCS
DEVICE_FUNCS
#undef VKF

static void Fatal( const char* fmt, ... )
{
	va_list a;
	va_start( a, fmt );
	vfprintf( stderr, fmt, a );
	va_end( a );
	fputc( '\n', stderr );
	exit( 1 );
}
#define VK_CHECK( x ) do { VkResult r_ = (x); if (r_ != VK_SUCCESS) Fatal( "%s failed: %d (line %d)", #x, (int)r_, __LINE__ ); } while (0)

static VkInstance instance;
static VkPhysicalDevice pdev;
static VkDevice device;
static VkQueue queue;
static VkCommandPool cmdPool;
static VkPhysicalDeviceProperties props;
static VkPhysicalDeviceIDProperties idProps{ VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ID_PROPERTIES };
static VkPhysicalDeviceAccelerationStructurePropertiesKHR asProps{ VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_PROPERTIES_KHR };
static VkPhysicalDeviceMemoryProperties memProps;
static bool hasMaint1 = false, hasPosFetch = false;
static uint32_t queueFamily = 0;

// ----------------------------------------------------------------------------
// Vulkan setup
// ----------------------------------------------------------------------------

static void LoadVulkan()
{
#ifdef _WIN32
	HMODULE lib = LoadLibraryA( "vulkan-1.dll" );
	if (!lib) Fatal( "vulkan-1.dll not found" );
	vkGetInstanceProcAddr = (PFN_vkGetInstanceProcAddr)(void*)GetProcAddress( lib, "vkGetInstanceProcAddr" );
#else
	void* lib = dlopen( "libvulkan.so.1", RTLD_NOW );
	if (!lib) Fatal( "libvulkan.so.1 not found" );
	vkGetInstanceProcAddr = (PFN_vkGetInstanceProcAddr)dlsym( lib, "vkGetInstanceProcAddr" );
#endif
	vkCreateInstance = (PFN_vkCreateInstance)vkGetInstanceProcAddr( 0, "vkCreateInstance" );
}

static bool HasExt( const std::vector<VkExtensionProperties>& e, const char* name )
{
	for (auto& x : e) if (!strcmp( x.extensionName, name )) return true;
	return false;
}

static void InitDevice( const char* want )
{
	LoadVulkan();
	VkApplicationInfo app{ VK_STRUCTURE_TYPE_APPLICATION_INFO };
	app.pApplicationName = "asprobe", app.apiVersion = VK_API_VERSION_1_3;
	VkInstanceCreateInfo ici{ VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO };
	ici.pApplicationInfo = &app;
	VK_CHECK( vkCreateInstance( &ici, 0, &instance ) );
#define VKF( name ) name = (PFN_##name)vkGetInstanceProcAddr( instance, #name ); if (!name) Fatal( "missing %s", #name );
	INSTANCE_FUNCS
#undef VKF
	uint32_t n = 0;
	vkEnumeratePhysicalDevices( instance, &n, 0 );
	std::vector<VkPhysicalDevice> devs( n );
	vkEnumeratePhysicalDevices( instance, &n, devs.data() );
	for (auto d : devs)
	{
		VkPhysicalDeviceProperties p;
		vkGetPhysicalDeviceProperties( d, &p );
		if (strstr( p.deviceName, want )) { pdev = d; break; }
	}
	if (!pdev) Fatal( "no device matching '%s'", want );
	idProps.pNext = &asProps;
	VkPhysicalDeviceProperties2 p2{ VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2 };
	p2.pNext = &idProps;
	vkGetPhysicalDeviceProperties2( pdev, &p2 );
	props = p2.properties;
	vkGetPhysicalDeviceMemoryProperties( pdev, &memProps );
	vkEnumerateDeviceExtensionProperties( pdev, 0, &n, 0 );
	std::vector<VkExtensionProperties> ext( n );
	vkEnumerateDeviceExtensionProperties( pdev, 0, &n, ext.data() );
	if (!HasExt( ext, VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME )) Fatal( "no VK_KHR_acceleration_structure" );
	hasMaint1 = HasExt( ext, VK_KHR_RAY_TRACING_MAINTENANCE_1_EXTENSION_NAME );
	hasPosFetch = HasExt( ext, VK_KHR_RAY_TRACING_POSITION_FETCH_EXTENSION_NAME );
	std::vector<const char*> en = { VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME, VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME };
	if (hasMaint1) en.push_back( VK_KHR_RAY_TRACING_MAINTENANCE_1_EXTENSION_NAME );
	if (hasPosFetch) en.push_back( VK_KHR_RAY_TRACING_POSITION_FETCH_EXTENSION_NAME );
	vkGetPhysicalDeviceQueueFamilyProperties( pdev, &n, 0 );
	std::vector<VkQueueFamilyProperties> qf( n );
	vkGetPhysicalDeviceQueueFamilyProperties( pdev, &n, qf.data() );
	queueFamily = UINT32_MAX;
	for (uint32_t i = 0; i < n; i++) if (qf[i].queueFlags & VK_QUEUE_COMPUTE_BIT) { queueFamily = i; break; }
	if (queueFamily == UINT32_MAX) Fatal( "no compute queue" );
	const float prio = 1;
	VkDeviceQueueCreateInfo qci{ VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO };
	qci.queueFamilyIndex = queueFamily, qci.queueCount = 1, qci.pQueuePriorities = &prio;
	VkPhysicalDeviceRayTracingPositionFetchFeaturesKHR posFetch{ VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_POSITION_FETCH_FEATURES_KHR };
	posFetch.rayTracingPositionFetch = VK_TRUE;
	VkPhysicalDeviceRayTracingMaintenance1FeaturesKHR maint1{ VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_MAINTENANCE_1_FEATURES_KHR };
	maint1.rayTracingMaintenance1 = VK_TRUE;
	VkPhysicalDeviceAccelerationStructureFeaturesKHR asf{ VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR };
	asf.accelerationStructure = VK_TRUE;
	VkPhysicalDeviceVulkan12Features v12{ VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES };
	v12.bufferDeviceAddress = VK_TRUE;
	v12.pNext = &asf;
	void** tail = &asf.pNext;
	if (hasMaint1) *tail = &maint1, tail = &maint1.pNext;
	if (hasPosFetch) *tail = &posFetch, tail = &posFetch.pNext;
	VkDeviceCreateInfo dci{ VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO };
	dci.pNext = &v12;
	dci.queueCreateInfoCount = 1, dci.pQueueCreateInfos = &qci;
	dci.enabledExtensionCount = (uint32_t)en.size(), dci.ppEnabledExtensionNames = en.data();
	VK_CHECK( vkCreateDevice( pdev, &dci, 0, &device ) );
#define VKF( name ) name = (PFN_##name)vkGetDeviceProcAddr( device, #name ); if (!name) Fatal( "missing %s", #name );
	DEVICE_FUNCS
#undef VKF
	vkGetDeviceQueue( device, queueFamily, 0, &queue );
	VkCommandPoolCreateInfo cpi{ VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO };
	cpi.queueFamilyIndex = queueFamily, cpi.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
	VK_CHECK( vkCreateCommandPool( device, &cpi, 0, &cmdPool ) );
}

struct Buf { VkBuffer buf = 0; VkDeviceMemory mem = 0; VkDeviceAddress addr = 0; VkDeviceSize size = 0; void* map = 0; };

static uint32_t MemType( uint32_t bits, VkMemoryPropertyFlags want )
{
	for (uint32_t i = 0; i < memProps.memoryTypeCount; i++)
		if ((bits & (1u << i)) && (memProps.memoryTypes[i].propertyFlags & want) == want) return i;
	Fatal( "no memory type" );
	return 0;
}

static Buf MakeBuf( VkDeviceSize size, VkBufferUsageFlags usage, bool host )
{
	Buf b;
	b.size = size;
	VkBufferCreateInfo bci{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
	bci.size = size, bci.usage = usage | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
	VK_CHECK( vkCreateBuffer( device, &bci, 0, &b.buf ) );
	VkMemoryRequirements mr;
	vkGetBufferMemoryRequirements( device, b.buf, &mr );
	VkMemoryAllocateFlagsInfo fi{ VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_FLAGS_INFO };
	fi.flags = VK_MEMORY_ALLOCATE_DEVICE_ADDRESS_BIT;
	VkMemoryAllocateInfo mai{ VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
	mai.pNext = &fi, mai.allocationSize = mr.size;
	mai.memoryTypeIndex = MemType( mr.memoryTypeBits, host ? VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT : VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT );
	VK_CHECK( vkAllocateMemory( device, &mai, 0, &b.mem ) );
	VK_CHECK( vkBindBufferMemory( device, b.buf, b.mem, 0 ) );
	VkBufferDeviceAddressInfo ai{ VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO };
	ai.buffer = b.buf;
	b.addr = vkGetBufferDeviceAddress( device, &ai );
	if (host) VK_CHECK( vkMapMemory( device, b.mem, 0, VK_WHOLE_SIZE, 0, &b.map ) );
	return b;
}

static void FreeBuf( Buf& b )
{
	if (b.buf) vkDestroyBuffer( device, b.buf, 0 );
	if (b.mem) vkFreeMemory( device, b.mem, 0 );
	b = Buf();
}

static VkCommandBuffer Begin()
{
	VkCommandBufferAllocateInfo ai{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
	ai.commandPool = cmdPool, ai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY, ai.commandBufferCount = 1;
	VkCommandBuffer cb;
	VK_CHECK( vkAllocateCommandBuffers( device, &ai, &cb ) );
	VkCommandBufferBeginInfo bi{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
	bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
	VK_CHECK( vkBeginCommandBuffer( cb, &bi ) );
	return cb;
}

static void Submit( VkCommandBuffer cb )
{
	VK_CHECK( vkEndCommandBuffer( cb ) );
	VkSubmitInfo si{ VK_STRUCTURE_TYPE_SUBMIT_INFO };
	si.commandBufferCount = 1, si.pCommandBuffers = &cb;
	VK_CHECK( vkQueueSubmit( queue, 1, &si, 0 ) );
	VK_CHECK( vkQueueWaitIdle( queue ) );
	vkFreeCommandBuffers( device, cmdPool, 1, &cb );
}

static void Barrier( VkCommandBuffer cb )
{
	VkMemoryBarrier mb{ VK_STRUCTURE_TYPE_MEMORY_BARRIER };
	mb.srcAccessMask = VK_ACCESS_MEMORY_WRITE_BIT;
	mb.dstAccessMask = VK_ACCESS_MEMORY_READ_BIT | VK_ACCESS_MEMORY_WRITE_BIT;
	vkCmdPipelineBarrier( cb, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 1, &mb, 0, 0, 0, 0 );
}

// ----------------------------------------------------------------------------
// Probe scenes
// ----------------------------------------------------------------------------

struct Scene { std::vector<float> v; std::vector<uint32_t> idx; uint32_t tris = 0; };

static uint32_t rngState = 0x2545F491;
static float Rnd() { rngState ^= rngState << 13, rngState ^= rngState >> 17, rngState ^= rngState << 5; return (rngState >> 8) * (1.0f / 16777216.0f); }

static void AddTri( Scene& s, const float* a, const float* b, const float* c )
{
	s.v.insert( s.v.end(), a, a + 3 ), s.v.insert( s.v.end(), b, b + 3 ), s.v.insert( s.v.end(), c, c + 3 ), s.tris++;
}

static Scene MakeScene( const std::string& spec )
{
	Scene s;
	const size_t colon = spec.find( ':' );
	const std::string kind = spec.substr( 0, colon );
	const std::string arg = colon == std::string::npos ? "" : spec.substr( colon + 1 );
	const int N = arg.empty() ? 1 : atoi( arg.c_str() );
	rngState = 0x2545F491;
	if (kind == "tri1")
	{
		// marker vertices with recognisable fp32 bit patterns
		const float a[3] = { 1.0f, 2.0f, 3.0f }, b[3] = { 7.25f, 0.3f, 1234.5678f }, c[3] = { -3.5f, 100.125f, 0.0625f };
		AddTri( s, a, b, c );
	}
	else if (kind == "grid") for (int i = 0; i < N; i++)
	{
		// disjoint unit triangles on a 2D grid, spacing 2
		const float x = (float)(i % 64) * 2, y = (float)(i / 64) * 2;
		const float a[3] = { x, y, 0 }, b[3] = { x + 1, y, 0 }, c[3] = { x, y + 1, 0 };
		AddTri( s, a, b, c );
	}
	else if (kind == "strip") for (int i = 0; i < N; i++)
	{
		const float x0 = (float)(i / 2), x1 = (float)(i / 2 + 1);
		const float a[3] = { x0, 0, 0 }, b[3] = { x1, 0, 0 }, c[3] = { x0, 1, 0 }, d[3] = { x1, 1, 0 };
		if (i & 1) AddTri( s, b, d, c ); else AddTri( s, a, b, c );
	}
	else if (kind == "same") for (int i = 0; i < N; i++)
	{
		// N copies of one triangle: 3 unique vertices, isolates the per-triangle cost
		const float a[3] = { 0, 0, 0 }, b[3] = { 1, 0, 0 }, c[3] = { 0, 1, 0 };
		AddTri( s, a, b, c );
	}
	else if (kind == "quad")
	{
		// a compact W x W quad mesh (2 tris per quad, shared verts), first N triangles in serpentine order
		int W = 1;
		while (2 * W * W < N) W++;
		for (int i = 0; i < N; i++)
		{
			const int q = i / 2, row = q / W, col = (row & 1) ? W - 1 - q % W : q % W;
			const float x0 = (float)col, x1 = (float)col + 1, y0 = (float)row, y1 = (float)row + 1;
			const float a[3] = { x0, y0, 0 }, b[3] = { x1, y0, 0 }, c[3] = { x0, y1, 0 }, d[3] = { x1, y1, 0 };
			if (i & 1) AddTri( s, b, d, c ); else AddTri( s, a, b, c );
		}
	}
	else if (kind == "fan") for (int i = 0; i < N; i++)
	{
		const float a0 = 6.2831853f * i / (N + 1), a1 = 6.2831853f * (i + 1) / (N + 1);
		const float o[3] = { 0, 0, 0 }, b[3] = { cosf( a0 ), sinf( a0 ), 0 }, c[3] = { cosf( a1 ), sinf( a1 ), 0 };
		AddTri( s, o, b, c );
	}
	else if (kind == "soup") for (int i = 0; i < N; i++)
	{
		const float px = Rnd() * 10, py = Rnd() * 10, pz = Rnd() * 10;
		const float a[3] = { px, py, pz }, b[3] = { px + Rnd(), py + Rnd(), pz + Rnd() }, c[3] = { px + Rnd(), py + Rnd(), pz + Rnd() };
		AddTri( s, a, b, c );
	}
	else if (kind == "txt")
	{
		// text file, one triangle per line: 9 floats (x y z of three vertices); '#' starts a comment
		FILE* f = fopen( arg.c_str(), "r" );
		if (!f) Fatal( "cannot open %s", arg.c_str() );
		char line[1024];
		while (fgets( line, sizeof( line ), f ))
		{
			if (line[0] == '#') continue;
			float t[9];
			if (sscanf( line, "%f %f %f %f %f %f %f %f %f", t, t + 1, t + 2, t + 3, t + 4, t + 5, t + 6, t + 7, t + 8 ) == 9) AddTri( s, t, t + 3, t + 6 );
		}
		fclose( f );
		if (!s.tris) Fatal( "no triangles in %s", arg.c_str() );
	}
	else if (kind == "bin")
	{
		FILE* f = fopen( arg.c_str(), "rb" );
		if (!f) Fatal( "cannot open %s", arg.c_str() );
		uint32_t n = 0;
		if (fread( &n, 4, 1, f ) != 1) Fatal( "bad file" );
		std::vector<float> v4( (size_t)n * 12 );
		if (fread( v4.data(), 48, n, f ) != n) Fatal( "bad file" );
		fclose( f );
		for (uint32_t i = 0; i < n * 3; i++) s.v.insert( s.v.end(), &v4[i * 4], &v4[i * 4] + 3 );
		s.tris = n;
	}
	else Fatal( "unknown scene '%s'", spec.c_str() );
	return s;
}

// weld identical positions into an index buffer.
static void Weld( Scene& s )
{
	std::map<std::tuple<uint32_t, uint32_t, uint32_t>, uint32_t> m;
	std::vector<float> v;
	s.idx.resize( (size_t)s.tris * 3 );
	for (size_t i = 0; i < (size_t)s.tris * 3; i++)
	{
		uint32_t k[3];
		memcpy( k, &s.v[i * 3], 12 );
		auto key = std::make_tuple( k[0], k[1], k[2] );
		auto it = m.find( key );
		if (it == m.end()) it = m.emplace( key, (uint32_t)(v.size() / 3) ).first, v.insert( v.end(), &s.v[i * 3], &s.v[i * 3] + 3 );
		s.idx[i] = it->second;
	}
	s.v = v;
}

static uint16_t ToHalf( float f )
{
	uint32_t x;
	memcpy( &x, &f, 4 );
	const uint32_t sign = (x >> 16) & 0x8000;
	int32_t e = (int32_t)((x >> 23) & 0xff) - 127 + 15;
	uint32_t m = x & 0x7fffff;
	if (e <= 0) return (uint16_t)sign; // flush small values; probes avoid them
	if (e >= 31) return (uint16_t)(sign | 0x7c00);
	uint32_t h = sign | (e << 10) | (m >> 13);
	if ((m >> 12) & 1) h++; // round half up
	return (uint16_t)h;
}

// ----------------------------------------------------------------------------
// Build, measure, dump
// ----------------------------------------------------------------------------

struct Options { VkBuildAccelerationStructureFlagsKHR flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR | VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_COMPACTION_BIT_KHR; bool fp16 = false, indexed = false, opaque = true; uint32_t geoms = 1; };

struct Result
{
	VkDeviceSize buildSize = 0, scratchSize = 0, compactedSize = 0, serializedSize = 0, usedSize = 0;
	VkDeviceAddress bufAddr = 0, asAddr = 0, compactBufAddr = 0, compactAsAddr = 0;
	std::vector<uint8_t> raw, compact, ser;
};

static VkBuildAccelerationStructureFlagsKHR ParseFlags( const char* s )
{
	VkBuildAccelerationStructureFlagsKHR f = 0;
	std::string str( s );
	size_t p = 0;
	while (p <= str.size())
	{
		size_t q = str.find( ',', p );
		if (q == std::string::npos) q = str.size();
		const std::string t = str.substr( p, q - p );
		if (t == "fasttrace") f |= VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
		else if (t == "fastbuild") f |= VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_BUILD_BIT_KHR;
		else if (t == "lowmem") f |= VK_BUILD_ACCELERATION_STRUCTURE_LOW_MEMORY_BIT_KHR;
		else if (t == "compact") f |= VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_COMPACTION_BIT_KHR;
		else if (t == "update") f |= VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
		else if (t == "dataaccess") f |= VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_DATA_ACCESS_BIT_KHR;
		else if (t == "none" || t.empty()) {}
		else Fatal( "unknown flag '%s'", t.c_str() );
		p = q + 1;
	}
	return f;
}

static std::vector<uint8_t> ReadBack( VkBuffer src, VkDeviceSize size )
{
	Buf st = MakeBuf( size, VK_BUFFER_USAGE_TRANSFER_DST_BIT, true );
	VkCommandBuffer cb = Begin();
	Barrier( cb );
	VkBufferCopy c{ 0, 0, size };
	vkCmdCopyBuffer( cb, src, st.buf, 1, &c );
	Submit( cb );
	std::vector<uint8_t> out( (uint8_t*)st.map, (uint8_t*)st.map + size );
	FreeBuf( st );
	return out;
}

static const VkBufferUsageFlags AS_USAGE = VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;

static VkAccelerationStructureKHR CreateAS( Buf& b, VkDeviceSize size )
{
	b = MakeBuf( size, AS_USAGE, false );
	VkCommandBuffer cb = Begin();
	vkCmdFillBuffer( cb, b.buf, 0, VK_WHOLE_SIZE, 0xCDCDCDCDu ); // untouched bytes stay 0xCD
	Submit( cb );
	VkAccelerationStructureCreateInfoKHR ci{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR };
	ci.buffer = b.buf, ci.size = size, ci.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
	VkAccelerationStructureKHR as;
	VK_CHECK( vkCreateAccelerationStructureKHR( device, &ci, 0, &as ) );
	return as;
}

static VkDeviceAddress ASAddr( VkAccelerationStructureKHR as )
{
	VkAccelerationStructureDeviceAddressInfoKHR ai{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR };
	ai.accelerationStructure = as;
	return vkGetAccelerationStructureDeviceAddressKHR( device, &ai );
}

static VkDeviceSize Query( VkAccelerationStructureKHR as, VkQueryType type )
{
	VkQueryPoolCreateInfo qpi{ VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO };
	qpi.queryType = type, qpi.queryCount = 1;
	VkQueryPool pool;
	VK_CHECK( vkCreateQueryPool( device, &qpi, 0, &pool ) );
	VkCommandBuffer cb = Begin();
	vkCmdResetQueryPool( cb, pool, 0, 1 );
	Barrier( cb );
	vkCmdWriteAccelerationStructuresPropertiesKHR( cb, 1, &as, type, pool, 0 );
	Submit( cb );
	uint64_t v = 0;
	VK_CHECK( vkGetQueryPoolResults( device, pool, 0, 1, 8, &v, 8, VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT ) );
	vkDestroyQueryPool( device, pool, 0 );
	return v;
}

static Result BuildBLAS( const Scene& sIn, const Options& o, bool dump )
{
	Scene s = sIn;
	if (o.indexed) Weld( s );
	Result r;
	// vertex data
	const uint32_t vcount = (uint32_t)(s.v.size() / 3);
	const VkDeviceSize vstride = o.fp16 ? 8 : 12;
	const VkBufferUsageFlags inUsage = VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR;
	Buf vb = MakeBuf( vcount * vstride + 16, inUsage, true );
	if (o.fp16)
	{
		uint16_t* h = (uint16_t*)vb.map;
		for (uint32_t i = 0; i < vcount; i++) h[i * 4 + 0] = ToHalf( s.v[i * 3] ), h[i * 4 + 1] = ToHalf( s.v[i * 3 + 1] ), h[i * 4 + 2] = ToHalf( s.v[i * 3 + 2] ), h[i * 4 + 3] = 0;
	}
	else memcpy( vb.map, s.v.data(), vcount * 12 );
	Buf ib;
	if (o.indexed) ib = MakeBuf( s.idx.size() * 4 + 16, inUsage, true ), memcpy( ib.map, s.idx.data(), s.idx.size() * 4 );
	// geometries: an even split of the triangles
	const uint32_t G = o.geoms < 1 ? 1 : (o.geoms > s.tris ? s.tris : o.geoms);
	std::vector<VkAccelerationStructureGeometryKHR> geom( G );
	std::vector<VkAccelerationStructureBuildRangeInfoKHR> range( G );
	std::vector<uint32_t> counts( G );
	for (uint32_t g = 0, first = 0; g < G; g++)
	{
		const uint32_t cnt = s.tris / G + (g < s.tris % G ? 1 : 0);
		VkAccelerationStructureGeometryKHR& gm = geom[g];
		gm = { VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR };
		gm.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR;
		gm.flags = o.opaque ? VK_GEOMETRY_OPAQUE_BIT_KHR : 0;
		auto& t = gm.geometry.triangles;
		t.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR;
		t.vertexFormat = o.fp16 ? VK_FORMAT_R16G16B16A16_SFLOAT : VK_FORMAT_R32G32B32_SFLOAT;
		t.vertexData.deviceAddress = vb.addr, t.vertexStride = vstride, t.maxVertex = vcount - 1;
		t.indexType = o.indexed ? VK_INDEX_TYPE_UINT32 : VK_INDEX_TYPE_NONE_KHR;
		if (o.indexed) t.indexData.deviceAddress = ib.addr;
		range[g] = {};
		range[g].primitiveCount = cnt;
		range[g].primitiveOffset = o.indexed ? first * 12 : (uint32_t)(first * 3 * vstride); // byte offsets
		counts[g] = cnt, first += cnt;
	}
	VkAccelerationStructureBuildGeometryInfoKHR bi{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR };
	bi.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR, bi.flags = o.flags;
	bi.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
	bi.geometryCount = G, bi.pGeometries = geom.data();
	VkAccelerationStructureBuildSizesInfoKHR sz{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR };
	vkGetAccelerationStructureBuildSizesKHR( device, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR, &bi, counts.data(), &sz );
	r.buildSize = sz.accelerationStructureSize, r.scratchSize = sz.buildScratchSize;
	Buf asb;
	VkAccelerationStructureKHR as = CreateAS( asb, sz.accelerationStructureSize );
	const VkDeviceSize al = asProps.minAccelerationStructureScratchOffsetAlignment;
	Buf scratch = MakeBuf( sz.buildScratchSize + al, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, false );
	bi.dstAccelerationStructure = as;
	bi.scratchData.deviceAddress = (scratch.addr + al - 1) & ~(al - 1);
	const VkAccelerationStructureBuildRangeInfoKHR* pr = range.data();
	VkCommandBuffer cb = Begin();
	vkCmdBuildAccelerationStructuresKHR( cb, 1, &bi, &pr );
	Submit( cb );
	FreeBuf( scratch );
	r.bufAddr = asb.addr, r.asAddr = ASAddr( as );
	if (o.flags & VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_COMPACTION_BIT_KHR) r.compactedSize = Query( as, VK_QUERY_TYPE_ACCELERATION_STRUCTURE_COMPACTED_SIZE_KHR );
	r.serializedSize = Query( as, VK_QUERY_TYPE_ACCELERATION_STRUCTURE_SERIALIZATION_SIZE_KHR );
	if (hasMaint1) r.usedSize = Query( as, VK_QUERY_TYPE_ACCELERATION_STRUCTURE_SIZE_KHR );
	if (dump)
	{
		r.raw = ReadBack( asb.buf, asb.size );
		if (r.compactedSize)
		{
			Buf cbuf;
			VkAccelerationStructureKHR cas = CreateAS( cbuf, r.compactedSize );
			VkCopyAccelerationStructureInfoKHR cp{ VK_STRUCTURE_TYPE_COPY_ACCELERATION_STRUCTURE_INFO_KHR };
			cp.src = as, cp.dst = cas, cp.mode = VK_COPY_ACCELERATION_STRUCTURE_MODE_COMPACT_KHR;
			cb = Begin();
			vkCmdCopyAccelerationStructureKHR( cb, &cp );
			Submit( cb );
			r.compactBufAddr = cbuf.addr, r.compactAsAddr = ASAddr( cas );
			r.compact = ReadBack( cbuf.buf, cbuf.size );
			vkDestroyAccelerationStructureKHR( device, cas, 0 );
			FreeBuf( cbuf );
		}
		Buf sb = MakeBuf( (r.serializedSize + 255) & ~255ull, VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT, false );
		VkCopyAccelerationStructureToMemoryInfoKHR cm{ VK_STRUCTURE_TYPE_COPY_ACCELERATION_STRUCTURE_TO_MEMORY_INFO_KHR };
		cm.src = as, cm.dst.deviceAddress = sb.addr, cm.mode = VK_COPY_ACCELERATION_STRUCTURE_MODE_SERIALIZE_KHR;
		cb = Begin();
		vkCmdFillBuffer( cb, sb.buf, 0, VK_WHOLE_SIZE, 0xCDCDCDCDu );
		Barrier( cb );
		vkCmdCopyAccelerationStructureToMemoryKHR( cb, &cm );
		Submit( cb );
		r.ser = ReadBack( sb.buf, r.serializedSize );
		FreeBuf( sb );
	}
	vkDestroyAccelerationStructureKHR( device, as, 0 );
	FreeBuf( asb ), FreeBuf( vb ), FreeBuf( ib );
	return r;
}

// ----------------------------------------------------------------------------
// Commands
// ----------------------------------------------------------------------------

static void PrintUUID( FILE* f, const uint8_t* u )
{
	for (int i = 0; i < 16; i++) fprintf( f, "%02x%s", u[i], (i == 3 || i == 5 || i == 7 || i == 9) ? "-" : "" );
}

static void WriteFile( const std::string& name, const std::vector<uint8_t>& d )
{
	if (d.empty()) return;
	FILE* f = fopen( name.c_str(), "wb" );
	if (!f) Fatal( "cannot write %s", name.c_str() );
	fwrite( d.data(), 1, d.size(), f );
	fclose( f );
}

static void ParseOptions( int argc, char** argv, int first, Options& o )
{
	for (int i = first; i < argc; i++)
	{
		if (!strcmp( argv[i], "-f" ) && i + 1 < argc) o.flags = ParseFlags( argv[++i] );
		else if (!strcmp( argv[i], "-fp16" )) o.fp16 = true;
		else if (!strcmp( argv[i], "-idx" )) o.indexed = true;
		else if (!strcmp( argv[i], "-noopaque" )) o.opaque = false;
		else if (!strcmp( argv[i], "-geoms" ) && i + 1 < argc) o.geoms = (uint32_t)atoi( argv[++i] );
		else Fatal( "unknown option '%s'", argv[i] );
	}
}

static void PrintDevice( FILE* f )
{
	fprintf( f, "device: %s\ndriverVersion: 0x%08x\ndriverUUID: ", props.deviceName, props.driverVersion );
	PrintUUID( f, idProps.driverUUID );
	fprintf( f, "\nmaintenance1: %d, positionFetch: %d\n", (int)hasMaint1, (int)hasPosFetch );
}

// ----------------------------------------------------------------------------
// AMD GPURT (RTIP 2.0) decoder, for calibration: walks a dumped BLAS and checks the triangles
// against the input scene. Layout per GPURT gpurtAccelStruct.h / ShaderDefs.hlsli.
// ----------------------------------------------------------------------------

static std::vector<uint8_t> ReadFile( const char* name )
{
	FILE* f = fopen( name, "rb" );
	if (!f) Fatal( "cannot open %s", name );
	fseek( f, 0, SEEK_END );
	std::vector<uint8_t> d( (size_t)ftell( f ) );
	fseek( f, 0, SEEK_SET );
	if (fread( d.data(), 1, d.size(), f ) != d.size()) Fatal( "read error %s", name );
	fclose( f );
	return d;
}

static float HalfToFloat( uint16_t h )
{
	const uint32_t s = (h & 0x8000u) << 16, e = (h >> 10) & 31, m = h & 1023;
	uint32_t x;
	if (e == 0) x = s; // flush denormals; bounds only
	else if (e == 31) x = s | 0x7f800000u | (m << 13);
	else x = s | ((e + 112) << 23) | (m << 13);
	float f;
	memcpy( &f, &x, 4 );
	return f;
}

struct DecTri { uint32_t prim, geom; float v[3][3]; };

static int DecodeAMD( const char* file, const char* sceneSpec, const Options& o )
{
	const std::vector<uint8_t> d = ReadFile( file );
	auto u32 = [&]( size_t off ) { uint32_t v = 0; if (off + 4 <= d.size()) memcpy( &v, &d[off], 4 ); return v; };
	auto f32 = [&]( size_t off ) { float v = 0; if (off + 4 <= d.size()) memcpy( &v, &d[off], 4 ); return v; };
	const uint32_t M = u32( 8 );
	const size_t H = M;
	printf( "metadata size %u, header.metadataSize %u, sizeInBytes %u, info 0x%08x (type %u, triCompression %u, fp16 %u)\n",
		M, u32( H + 4 ), u32( H + 8 ), u32( H ), u32( H ) & 1, (u32( H ) >> 6) & 7, (u32( H ) >> 9) & 3 );
	printf( "numPrimitives %u, numDescs %u, offsets internal %u leaf %u geomInfo %u primPtrs %u, fp32 nodes %u, fp16 nodes %u, leaves %u, version 0x%08x, rtIp %u\n",
		u32( H + 12 ), u32( H + 24 ), u32( H + 32 ), u32( H + 36 ), u32( H + 40 ), u32( H + 44 ), u32( H + 48 ), u32( H + 52 ), u32( H + 56 ), u32( H + 60 ), u32( H + 72 ) );
	std::vector<DecTri> tris;
	uint32_t boxes32 = 0, boxes16 = 0, errors = 0;
	std::vector<uint32_t> stack = { 0x15 }; // root: fp32 box at header + 128
	while (!stack.empty())
	{
		const uint32_t ptr = stack.back();
		stack.pop_back();
		const uint32_t type = ptr & 7;
		const size_t node = H + (size_t)(ptr & ~7u) * 8;
		if (node + 64 > d.size()) { errors++; continue; }
		if (type == 5 || type == 4)
		{
			type == 5 ? boxes32++ : boxes16++;
			for (int c = 0; c < 4; c++) if (u32( node + c * 4 ) != 0xffffffffu) stack.push_back( u32( node + c * 4 ) );
			continue;
		}
		if (type > 1) { errors++, printf( "unexpected node type %u\n", type ); continue; }
		// triangle node: v0..v3, geometryIndexAndFlags @48, primIdx0 @52, primIdx1 @56, triangleId @60
		const uint32_t triId = u32( node + 60 );
		static const uint32_t slotTable[2] = { 0x210, 0x231 };
		for (uint32_t t = 0; t <= type; t++)
		{
			const uint32_t i = (triId >> (8 * t)) & 3, j = (triId >> (8 * t + 2)) & 3, k = 3 - i - j;
			const uint32_t slots = slotTable[t], order[3] = { k, i, j };
			DecTri dt;
			dt.prim = u32( node + 52 + t * 4 ), dt.geom = u32( node + 48 ) & 0xffffff;
			for (int c = 0; c < 3; c++)
			{
				const uint32_t vslot = (slots >> (order[c] * 4)) & 15;
				for (int a = 0; a < 3; a++) dt.v[c][a] = f32( node + vslot * 12 + a * 4 );
			}
			tris.push_back( dt );
		}
	}
	printf( "decoded: %u fp32 boxes, %u fp16 boxes, %zu triangles, %u errors\n", boxes32, boxes16, tris.size(), errors );
	if (!sceneSpec) return errors ? 1 : 0;
	// check against the input: every primitive exactly once, with its vertices bit for bit (any rotation)
	Scene s = MakeScene( sceneSpec );
	if (o.fp16) for (float& x : s.v) x = HalfToFloat( ToHalf( x ) ); // the BLAS holds the fp16 values
	std::vector<int> seen( s.tris, 0 );
	// primitive indices are per geometry; geometries split the triangles evenly, as in BuildBLAS
	const uint32_t G = o.geoms < 1 ? 1 : (o.geoms > s.tris ? s.tris : o.geoms);
	std::vector<uint32_t> geomFirst( G + 1, 0 );
	for (uint32_t g = 0; g < G; g++) geomFirst[g + 1] = geomFirst[g] + s.tris / G + (g < s.tris % G ? 1 : 0);
	uint32_t bad = 0;
	for (const DecTri& t : tris)
	{
		const uint32_t prim = t.geom < G ? geomFirst[t.geom] + t.prim : UINT32_MAX;
		if (prim >= s.tris) { bad++; continue; }
		seen[prim]++;
		bool match = false;
		for (int r = 0; r < 3 && !match; r++)
		{
			match = true;
			for (int c = 0; c < 3; c++) if (memcmp( t.v[c], &s.v[((size_t)prim * 3 + (c + r) % 3) * 3], 12 )) match = false;
		}
		if (!match && bad++ < 5)
		{
			uint32_t actual = UINT32_MAX;
			for (uint32_t p = 0; p < s.tris && actual == UINT32_MAX; p++) if (!memcmp( t.v[0], &s.v[(size_t)p * 9], 12 ) || !memcmp( t.v[0], &s.v[(size_t)p * 9 + 3], 12 ) || !memcmp( t.v[0], &s.v[(size_t)p * 9 + 6], 12 )) actual = p;
			printf( "  geom %u prim %u -> expected global %u, vertices found at prim %u\n", t.geom, t.prim, prim, actual );
		}
	}
	uint32_t missing = 0, dup = 0;
	for (uint32_t p = 0; p < s.tris; p++) if (!seen[p])
	{
		// the driver may drop inactive (degenerate / NaN) triangles: report them separately
		const float* v = &s.v[(size_t)p * 9];
		const float e1[3] = { v[3] - v[0], v[4] - v[1], v[5] - v[2] }, e2[3] = { v[6] - v[0], v[7] - v[1], v[8] - v[2] };
		const float cx = e1[1] * e2[2] - e1[2] * e2[1], cy = e1[2] * e2[0] - e1[0] * e2[2], cz = e1[0] * e2[1] - e1[1] * e2[0];
		const bool degenerate = cx == 0 && cy == 0 && cz == 0;
		if (!degenerate) missing++;
		printf( "  prim %u not in BVH (%s)\n", p, degenerate ? "zero area: inactive, allowed" : "NOT degenerate" );
	}
	for (uint32_t p = 0; p < s.tris; p++) dup += seen[p] > 1;
	printf( "check vs %s: %u tris, %u missing, %u duplicated, %u mismatched -> %s\n", sceneSpec, s.tris, missing, dup, bad,
		(missing | dup | bad | errors) ? "FAIL" : "PASS" );
	(void)o;
	return (missing | dup | bad | errors) ? 1 : 0;
}

int main( int argc, char** argv )
{
	if (argc < 3) Fatal( "usage: asprobe <gpu> info | dump <scene> <outprefix> [opts] | sweep <kind> <n0> <n1> <step> [opts]\n"
		"       asprobe - decode-amd <dump.bin> [scene]" );
	if (!strcmp( argv[1], "-" ))
	{
		if (!strcmp( argv[2], "decode-amd" ) && argc >= 4) { Options o; if (argc > 5) ParseOptions( argc, argv, 5, o ); return DecodeAMD( argv[3], argc >= 5 ? argv[4] : 0, o ); }
		Fatal( "bad command line" );
	}
	InitDevice( argv[1] );
	const std::string cmd = argv[2];
	if (cmd == "info")
	{
		PrintDevice( stdout );
		printf( "minAccelerationStructureScratchOffsetAlignment: %u\n", asProps.minAccelerationStructureScratchOffsetAlignment );
	}
	else if (cmd == "dump" && argc >= 5)
	{
		Options o;
		ParseOptions( argc, argv, 5, o );
		const Scene s = MakeScene( argv[3] );
		const Result r = BuildBLAS( s, o, true );
		const std::string pre = argv[4];
		WriteFile( pre + ".raw.bin", r.raw ), WriteFile( pre + ".compact.bin", r.compact ), WriteFile( pre + ".ser.bin", r.ser );
		FILE* f = fopen( (pre + ".txt").c_str(), "w" );
		if (!f) Fatal( "cannot write %s.txt", pre.c_str() );
		PrintDevice( f );
		fprintf( f, "scene: %s, tris: %u, flags: 0x%x, fp16: %d, indexed: %d, geoms: %u, opaque: %d\n", argv[3], s.tris, o.flags, (int)o.fp16, (int)o.indexed, o.geoms, (int)o.opaque );
		fprintf( f, "buildSize: %llu\nscratchSize: %llu\ncompactedSize: %llu\nserializedSize: %llu\nusedSize: %llu\n",
			(unsigned long long)r.buildSize, (unsigned long long)r.scratchSize, (unsigned long long)r.compactedSize, (unsigned long long)r.serializedSize, (unsigned long long)r.usedSize );
		fprintf( f, "bufAddr: 0x%llx\nasAddr: 0x%llx\ncompactBufAddr: 0x%llx\ncompactAsAddr: 0x%llx\n",
			(unsigned long long)r.bufAddr, (unsigned long long)r.asAddr, (unsigned long long)r.compactBufAddr, (unsigned long long)r.compactAsAddr );
		if (r.ser.size() >= 56)
		{
			fprintf( f, "ser.driverUUID: " ), PrintUUID( f, r.ser.data() );
			fprintf( f, "\nser.compatUUID: " ), PrintUUID( f, r.ser.data() + 16 );
			uint64_t q[3];
			memcpy( q, r.ser.data() + 32, 24 );
			fprintf( f, "\nser.serializedSize: %llu\nser.deserializedSize: %llu\nser.word48: 0x%016llx\n", (unsigned long long)q[0], (unsigned long long)q[1], (unsigned long long)q[2] );
		}
		fclose( f );
		printf( "%s: %u tris, build %llu, compacted %llu, serialized %llu B -> %s.*\n", argv[3], s.tris,
			(unsigned long long)r.buildSize, (unsigned long long)r.compactedSize, (unsigned long long)r.serializedSize, pre.c_str() );
	}
	else if (cmd == "sweep" && argc >= 7)
	{
		Options o;
		ParseOptions( argc, argv, 7, o );
		const int n0 = atoi( argv[4] ), n1 = atoi( argv[5] ), step = atoi( argv[6] ) < 1 ? 1 : atoi( argv[6] );
		printf( "kind,n,tris,buildSize,scratchSize,compactedSize,serializedSize,usedSize\n" );
		for (int n = n0; n <= n1; n += step)
		{
			const Scene s = MakeScene( std::string( argv[3] ) + ":" + std::to_string( n ) );
			const Result r = BuildBLAS( s, o, false );
			printf( "%s,%d,%u,%llu,%llu,%llu,%llu,%llu\n", argv[3], n, s.tris, (unsigned long long)r.buildSize, (unsigned long long)r.scratchSize,
				(unsigned long long)r.compactedSize, (unsigned long long)r.serializedSize, (unsigned long long)r.usedSize );
			fflush( stdout );
		}
	}
	else Fatal( "bad command line" );
	vkDeviceWaitIdle( device );
	return 0;
}
