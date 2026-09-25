// Vulkan port of the D3D12 tinybvh GPU ray tracing benchmark (program.cpp).

#if defined(_WIN32)
#define VK_USE_PLATFORM_WIN32_KHR
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#define VK_USE_PLATFORM_XCB_KHR
#include <xcb/xcb.h>
#include <unistd.h>
#include <time.h>
#endif
#include <vulkan/vulkan.h>
#include <cstdarg>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <vector>

#if defined(_WIN32)
extern "C" { __declspec( dllexport ) DWORD NvOptimusEnablement = 1; }
extern "C" { __declspec( dllexport ) int AmdPowerXpressRequestHighPerformance = 1; }
#endif
#if defined(_MSC_VER)
#pragma comment(lib, "user32")
#pragma comment(lib, "vulkan-1")
#endif

// ----------------------------------------------------------------------------
// platform primitives
// ----------------------------------------------------------------------------

#if defined(_WIN32)
static void* AlignedAlloc64( size_t bytes ) { return _aligned_malloc( bytes, 64 ); }
static void AlignedFree64( void* p ) { _aligned_free( p ); }
static void SleepMs( unsigned ms ) { Sleep( ms ); }
static void GetCwd( char* buf, size_t n ) { if (!GetCurrentDirectoryA( (DWORD)n, buf )) buf[0] = 0; }
static void ShowFatal( const char* msg ) { MessageBoxA( nullptr, msg, "tinybvh Vulkan benchmark", MB_OK | MB_ICONERROR ); }
static void QuitProcess() { ExitProcess( 1 ); }
#else
static void* AlignedAlloc64( size_t bytes )
{
	// aligned_alloc requires a size that is a multiple of the alignment.
	void* p = nullptr;
	if (posix_memalign( &p, 64, (bytes + 63) & ~(size_t)63 ) != 0) return nullptr;
	return p;
}
static void AlignedFree64( void* p ) { free( p ); }
static void SleepMs( unsigned ms ) { struct timespec t{ ms / 1000, (long)(ms % 1000) * 1000000L }; nanosleep( &t, nullptr ); }
static void GetCwd( char* buf, size_t n ) { if (!getcwd( buf, n )) buf[0] = 0; }
static void ShowFatal( const char* /*msg*/ ) {} // the console message is enough
static void QuitProcess() { exit( 1 ); }
#endif

// settings
constexpr uint32_t rtWidth = 1024, rtHeight = 1024;
#define ENABLE_HWRT			true	// display info on hardware RT pipeline performance
#define ENABLE_BVH2			true	// display info on BVH_GPU performance
#define ENABLE_BVH4			false	// display info on BVH4_GPU performance
#define ENABLE_CWBVH		true	// display info on BVH8_CWBVH performance
#define ENABLE_RAY_QUERIES	false	// display info on inline ray query performance
#define ENABLE_VALIDATION	false	// VK_LAYER_KHRONOS_validation; costs performance
#define BARRIER_BETWEEN_DISPATCHES false
#define ALLOW_SYNTHETIC_RAYS false

constexpr uint32_t NUM_INSTANCES = 1, FRAME_COUNT = 2;
constexpr uint32_t NUM_DISPATCHES = 20;
constexpr uint32_t NUM_SHADER_GROUPS = 3; // raygen, miss, hit group

// The ray tracing backends that are cycled through, one per frame.
enum Backend {
	BACKEND_HWRT = 0, BACKEND_BVH_GPU, BACKEND_BVH4_GPU, BACKEND_BVH8_CWBVH, BACKEND_RAYQUERY, BACKEND_COUNT
};
static const char* backendName[BACKEND_COUNT] = { "HWRT", "BVH_GPU", "BVH4_GPU", "CWBVH", "RayQuery" };
static const bool backendPrint[BACKEND_COUNT] = { ENABLE_HWRT, ENABLE_BVH2, ENABLE_BVH4, ENABLE_CWBVH, ENABLE_RAY_QUERIES };
static bool rayQuerySupported = false;
static int activeBackends = BACKEND_COUNT;

#define TINYBVH_IMPLEMENTATION
#include "../../tiny_bvh.h"
using namespace tinybvh;

#ifndef CWBVH_COMPRESSED_TRIS
#error "tiny8.comp assumes CWBVH_COMPRESSED_TRIS; enable it, or set TRI_STRIDE 3 in tiny8.comp and recompile"
#endif

// tinybvh BVH_GPU node.
struct GPUBVHNode
{
	bvhvec3 lmin; unsigned left;
	bvhvec3 lmax; unsigned right;
	bvhvec3 rmin; unsigned triCount;
	bvhvec3 rmax; unsigned firstTri; // total: 64 bytes
};

// One high-quality (SBVH) BVH2 is built; BVH_GPU and BVH4_GPU are derived from it.
static BVH bvh2;
static BVH_GPU bvh;
static MBVH<4> mbvh4;
static BVH4_GPU bvh4;
static BVH8_CWBVH cwbvh;

// ----------------------------------------------------------------------------
// Vulkan state
// ----------------------------------------------------------------------------

static VkInstance instance = VK_NULL_HANDLE;
static VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
static VkDevice device = VK_NULL_HANDLE;
static uint32_t queueFamily = 0;
static VkQueue queue = VK_NULL_HANDLE;
static VkPhysicalDeviceMemoryProperties memProps{};
static VkPhysicalDeviceProperties deviceProps{};
static VkPhysicalDeviceAccelerationStructurePropertiesKHR asProps{};
static VkPhysicalDeviceRayTracingPipelinePropertiesKHR rtProps{};
static uint64_t timestampMask = ~0ull;

static VkSurfaceKHR surface = VK_NULL_HANDLE;
static VkSwapchainKHR swapChain = VK_NULL_HANDLE;
static VkFormat swapFormat = VK_FORMAT_UNDEFINED;
static VkExtent2D swapExtent{};
static std::vector<VkImage> swapImages;
static std::vector<VkSemaphore> renderFinished; // one per swapchain image

static VkImage renderTarget = VK_NULL_HANDLE;
static VkDeviceMemory renderTargetMem = VK_NULL_HANDLE;
static VkImageView renderTargetView = VK_NULL_HANDLE;

static VkCommandPool cmdPool = VK_NULL_HANDLE, setupPool = VK_NULL_HANDLE;
static VkCommandBuffer cmdBufs[FRAME_COUNT]{};
static VkFence frameFences[FRAME_COUNT]{};
static VkSemaphore imageAvailable[FRAME_COUNT]{};
static bool frameSubmitted[FRAME_COUNT] = {};

static VkQueryPool timestampPool = VK_NULL_HANDLE;

static VkDescriptorSetLayout descLayout = VK_NULL_HANDLE;
static VkDescriptorPool descPool = VK_NULL_HANDLE;
static VkDescriptorSet descSet = VK_NULL_HANDLE;
static VkPipelineLayout pipelineLayout = VK_NULL_HANDLE;

static VkPipeline rtPipeline = VK_NULL_HANDLE;
static VkPipeline computePso = VK_NULL_HANDLE, computePso4 = VK_NULL_HANDLE;
static VkPipeline computePso8 = VK_NULL_HANDLE, computePsoRQ = VK_NULL_HANDLE;

static struct VulkanFns
{
	PFN_vkGetBufferDeviceAddressKHR GetBufferDeviceAddress;
	PFN_vkCreateAccelerationStructureKHR CreateAccelerationStructure;
	PFN_vkDestroyAccelerationStructureKHR DestroyAccelerationStructure;
	PFN_vkGetAccelerationStructureBuildSizesKHR GetAccelerationStructureBuildSizes;
	PFN_vkGetAccelerationStructureDeviceAddressKHR GetAccelerationStructureDeviceAddress;
	PFN_vkCmdBuildAccelerationStructuresKHR CmdBuildAccelerationStructures;
	PFN_vkCmdWriteAccelerationStructuresPropertiesKHR CmdWriteAccelerationStructuresProperties;
	PFN_vkCmdCopyAccelerationStructureKHR CmdCopyAccelerationStructure;
	PFN_vkCreateRayTracingPipelinesKHR CreateRayTracingPipelines;
	PFN_vkGetRayTracingShaderGroupHandlesKHR GetRayTracingShaderGroupHandles;
	PFN_vkCmdTraceRaysKHR CmdTraceRays;
} vk{};

// ----------------------------------------------------------------------------
// small helpers
// ----------------------------------------------------------------------------

static void Fatal( const char* fmt, ... )
{
	char msg[1024];
	va_list args;
	va_start( args, fmt );
	vsnprintf( msg, sizeof( msg ), fmt, args );
	va_end( args );
	printf( "fatal: %s\n", msg );
	fflush( stdout );
	ShowFatal( msg );
	QuitProcess();
}

static void CheckVk( VkResult r, const char* expr, int line )
{
	if (r != VK_SUCCESS) Fatal( "%s failed with VkResult %d (line %d)", expr, (int)r, line );
}
#define VK_CHECK(x) CheckVk( (x), #x, __LINE__ )

template <typename T> static T AlignUp( T v, T a ) { return (v + a - 1) & ~(a - 1); }

static uint32_t FindMemoryType( uint32_t typeBits, VkMemoryPropertyFlags want )
{
	for (uint32_t i = 0; i < memProps.memoryTypeCount; i++)
		if ((typeBits & (1u << i)) && (memProps.memoryTypes[i].propertyFlags & want) == want) return i;
	Fatal( "no memory type with properties 0x%X", (unsigned)want );
	return 0;
}

struct Buffer
{
	VkBuffer buf = VK_NULL_HANDLE;
	VkDeviceMemory mem = VK_NULL_HANDLE;
	VkDeviceSize size = 0;
	VkDeviceAddress addr = 0;
	void* mapped = nullptr;
};

static Buffer CreateBuffer( VkDeviceSize size, VkBufferUsageFlags usage,
	VkMemoryPropertyFlags props, bool keepMapped = false )
{
	Buffer b;
	b.size = size ? size : 1; // zero-size buffers are invalid
	VkBufferCreateInfo bi{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
	bi.size = b.size;
	bi.usage = usage;
	bi.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
	VK_CHECK( vkCreateBuffer( device, &bi, nullptr, &b.buf ) );
	VkMemoryRequirements mr;
	vkGetBufferMemoryRequirements( device, b.buf, &mr );
	VkMemoryAllocateFlagsInfo fi{ VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_FLAGS_INFO };
	fi.flags = VK_MEMORY_ALLOCATE_DEVICE_ADDRESS_BIT;
	VkMemoryAllocateInfo ai{ VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
	ai.allocationSize = mr.size;
	ai.memoryTypeIndex = FindMemoryType( mr.memoryTypeBits, props );
	if (usage & VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT) ai.pNext = &fi;
	VK_CHECK( vkAllocateMemory( device, &ai, nullptr, &b.mem ) );
	VK_CHECK( vkBindBufferMemory( device, b.buf, b.mem, 0 ) );
	if (usage & VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT)
	{
		VkBufferDeviceAddressInfo di{ VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO };
		di.buffer = b.buf;
		b.addr = vk.GetBufferDeviceAddress( device, &di );
	}
	if (keepMapped) VK_CHECK( vkMapMemory( device, b.mem, 0, VK_WHOLE_SIZE, 0, &b.mapped ) );
	return b;
}

static void DestroyBuffer( Buffer& b )
{
	if (b.mapped) vkUnmapMemory( device, b.mem ), b.mapped = nullptr;
	if (b.buf) vkDestroyBuffer( device, b.buf, nullptr ), b.buf = VK_NULL_HANDLE;
	if (b.mem) vkFreeMemory( device, b.mem, nullptr ), b.mem = VK_NULL_HANDLE;
}

static VkCommandBuffer BeginOneShot()
{
	VkCommandBufferAllocateInfo ai{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
	ai.commandPool = setupPool;
	ai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	ai.commandBufferCount = 1;
	VkCommandBuffer cb;
	VK_CHECK( vkAllocateCommandBuffers( device, &ai, &cb ) );
	VkCommandBufferBeginInfo bi{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
	bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
	VK_CHECK( vkBeginCommandBuffer( cb, &bi ) );
	return cb;
}

static void EndOneShot( VkCommandBuffer cb )
{
	VK_CHECK( vkEndCommandBuffer( cb ) );
	VkSubmitInfo si{ VK_STRUCTURE_TYPE_SUBMIT_INFO };
	si.commandBufferCount = 1;
	si.pCommandBuffers = &cb;
	VK_CHECK( vkQueueSubmit( queue, 1, &si, VK_NULL_HANDLE ) );
	VK_CHECK( vkQueueWaitIdle( queue ) );
	vkFreeCommandBuffers( device, setupPool, 1, &cb );
}

static Buffer MakeDeviceBuffer( const void* src, VkDeviceSize srcBytes, VkDeviceSize size,
	VkBufferUsageFlags usage, const char* /*name*/ = nullptr )
{
	if (size == 0) size = 1;
	Buffer staging = CreateBuffer( size, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, true );
	memset( staging.mapped, 0, (size_t)size );        // zero the unused tail (e.g. spare nodes)
	if (src) memcpy( staging.mapped, src, (size_t)srcBytes ); // then the real data
	Buffer dst = CreateBuffer( size, usage | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT );
	VkCommandBuffer cb = BeginOneShot();
	VkBufferCopy region{ 0, 0, size };
	vkCmdCopyBuffer( cb, staging.buf, dst.buf, 1, &region );
	EndOneShot( cb );
	DestroyBuffer( staging );
	return dst;
}

// ----------------------------------------------------------------------------
// resources
// ----------------------------------------------------------------------------

static Buffer meshVB{};        // triangle soup for the hardware BLAS
static Buffer rayBuffer{};     // the ray set, device local
static Buffer instanceBuffer{};// VkAccelerationStructureInstanceKHR array, host visible
static Buffer sbt{};           // shader binding table
static VkStridedDeviceAddressRegionKHR sbtRaygen{}, sbtMiss{}, sbtHit{}, sbtCallable{};

// Software-RT compute inputs.
static Buffer bvhNodeBuffer{}, bvhIdxBuffer{}, bvhVertBuffer{};
static Buffer bvh4DataBuffer{};
static Buffer cwbvhNodeBuffer{}, cwbvhTriBuffer{};

struct AccelStruct
{
	VkAccelerationStructureKHR handle = VK_NULL_HANDLE;
	Buffer buffer{};
	VkDeviceAddress addr = 0;
};

static AccelStruct blas{}, tlas{};
static Buffer tlasUpdateScratch{};

// Scene management - Append a file, with optional position, scale and color override, tinyfied
static int triCount = 0;
static bvhvec4* verts = 0;
static void AddMesh( const char* file, int N = 0 )
{
	std::fstream s{ file, s.binary | s.in };
	if (!s.is_open()) Fatal( "could not open mesh '%s'", file );
	s.read( (char*)&N, 4 );
	bvhvec4* data = (bvhvec4*)AlignedAlloc64( (size_t)(N + triCount) * 48 );
	if (verts) memcpy( data, verts, (size_t)triCount * 48 ), AlignedFree64( verts );
	verts = data, s.read( (char*)verts + triCount * 48, N * 48 ), triCount += N;
}

// ----------------------------------------------------------------------------
// window
// ----------------------------------------------------------------------------

static bool windowClosed = false, windowResized = false;

#if defined(_WIN32)

#define PLATFORM_SURFACE_EXTENSION_NAME VK_KHR_WIN32_SURFACE_EXTENSION_NAME
static HWND g_hwnd = nullptr;

static LRESULT WINAPI WndProc( HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam )
{
	switch (msg)
	{
	case WM_CLOSE: case WM_DESTROY: windowClosed = true; return 0;
	case WM_SIZE: windowResized = true; return 0;
	default: return DefWindowProcW( hwnd, msg, wparam, lparam );
	}
}

static void CreateAppWindow( uint32_t w, uint32_t h )
{
	SetProcessDpiAwarenessContext( DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2 );
	WNDCLASSW wcw = { .lpfnWndProc = &WndProc, .hCursor = LoadCursor( nullptr, IDC_ARROW ), .lpszClassName = L"uVKRT" };
	RegisterClassW( &wcw );
	// size the window so its client area is exactly w x h, keeping the presentation blit 1:1
	RECT r = { 0, 0, (LONG)w, (LONG)h };
	AdjustWindowRect( &r, WS_OVERLAPPEDWINDOW, FALSE );
	g_hwnd = CreateWindowExW( 0, L"uVKRT", L"_VK", WS_VISIBLE | WS_OVERLAPPEDWINDOW,
		CW_USEDEFAULT, CW_USEDEFAULT, r.right - r.left, r.bottom - r.top, 0, 0, 0, 0 );
	if (!g_hwnd) Fatal( "could not create a window" );
}

static VkSurfaceKHR CreatePlatformSurface( VkInstance inst )
{
	VkWin32SurfaceCreateInfoKHR ci{ VK_STRUCTURE_TYPE_WIN32_SURFACE_CREATE_INFO_KHR };
	ci.hinstance = GetModuleHandleW( nullptr );
	ci.hwnd = g_hwnd;
	VkSurfaceKHR s;
	VK_CHECK( vkCreateWin32SurfaceKHR( inst, &ci, nullptr, &s ) );
	return s;
}

static void PumpEvents()
{
	for (MSG msg; PeekMessageW( &msg, nullptr, 0, 0, PM_REMOVE );)
	{
		if (msg.message == WM_QUIT) { windowClosed = true; return; }
		TranslateMessage( &msg );
		DispatchMessageW( &msg );
	}
}

#else // XCB

#define PLATFORM_SURFACE_EXTENSION_NAME VK_KHR_XCB_SURFACE_EXTENSION_NAME
static xcb_connection_t* g_conn = nullptr;
static xcb_window_t g_window = 0;
static xcb_atom_t g_atomDeleteWindow = 0;
static uint16_t g_winW = 0, g_winH = 0;

static xcb_atom_t InternAtom( const char* name, uint8_t onlyIfExists )
{
	xcb_intern_atom_cookie_t c = xcb_intern_atom( g_conn, onlyIfExists, (uint16_t)strlen( name ), name );
	xcb_intern_atom_reply_t* r = xcb_intern_atom_reply( g_conn, c, nullptr );
	const xcb_atom_t a = r ? r->atom : (xcb_atom_t)0;
	free( r );
	return a;
}

static void CreateAppWindow( uint32_t w, uint32_t h )
{
	int screenIndex = 0;
	g_conn = xcb_connect( nullptr, &screenIndex );
	if (!g_conn || xcb_connection_has_error( g_conn ))
		Fatal( "could not connect to an X server (is DISPLAY set? Wayland-only\n"
			"sessions need XWayland, which is normally present)" );
	xcb_screen_iterator_t it = xcb_setup_roots_iterator( xcb_get_setup( g_conn ) );
	for (int i = 0; i < screenIndex; i++) xcb_screen_next( &it );
	xcb_screen_t* screen = it.data;
	g_window = xcb_generate_id( g_conn );
	const uint32_t mask = XCB_CW_BACK_PIXEL | XCB_CW_EVENT_MASK;
	const uint32_t values[2] = { screen->black_pixel, XCB_EVENT_MASK_STRUCTURE_NOTIFY };
	xcb_create_window( g_conn, XCB_COPY_FROM_PARENT, g_window, screen->root,
		0, 0, (uint16_t)w, (uint16_t)h, 0, XCB_WINDOW_CLASS_INPUT_OUTPUT,
		screen->root_visual, mask, values );
	g_winW = (uint16_t)w, g_winH = (uint16_t)h;
	const char* title = "_VK";
	xcb_change_property( g_conn, XCB_PROP_MODE_REPLACE, g_window, XCB_ATOM_WM_NAME,
		XCB_ATOM_STRING, 8, (uint32_t)strlen( title ), title );
	// Ask the window manager to send a ClientMessage on close instead of just severing the connection.
	const xcb_atom_t protocols = InternAtom( "WM_PROTOCOLS", 1 );
	g_atomDeleteWindow = InternAtom( "WM_DELETE_WINDOW", 0 );
	if (protocols && g_atomDeleteWindow)
		xcb_change_property( g_conn, XCB_PROP_MODE_REPLACE, g_window, protocols,
			XCB_ATOM_ATOM, 32, 1, &g_atomDeleteWindow );
	xcb_map_window( g_conn, g_window );
	xcb_flush( g_conn );
	// Wait for the map to complete.
	for (int spins = 0; spins < 1000; spins++)
	{
		xcb_generic_event_t* e = xcb_poll_for_event( g_conn );
		if (!e) { SleepMs( 2 ); continue; }
		const uint8_t type = e->response_type & 0x7F;
		free( e );
		if (type == XCB_MAP_NOTIFY) break;
	}
}

static VkSurfaceKHR CreatePlatformSurface( VkInstance inst )
{
	VkXcbSurfaceCreateInfoKHR ci{ VK_STRUCTURE_TYPE_XCB_SURFACE_CREATE_INFO_KHR };
	ci.connection = g_conn;
	ci.window = g_window;
	VkSurfaceKHR s;
	VK_CHECK( vkCreateXcbSurfaceKHR( inst, &ci, nullptr, &s ) );
	return s;
}

static void PumpEvents()
{
	for (xcb_generic_event_t* e; (e = xcb_poll_for_event( g_conn )) != nullptr; free( e ))
	{
		switch (e->response_type & 0x7F)
		{
		case XCB_CLIENT_MESSAGE:
			if (((xcb_client_message_event_t*)e)->data.data32[0] == g_atomDeleteWindow)
				windowClosed = true;
			break;
		case XCB_CONFIGURE_NOTIFY:
		{
			const xcb_configure_notify_event_t* c = (const xcb_configure_notify_event_t*)e;
			if (c->width != g_winW || c->height != g_winH) g_winW = c->width, g_winH = c->height, windowResized = true;
			break;
		}
		case XCB_DESTROY_NOTIFY: windowClosed = true; break;
		default: break;
		}
	}
	if (xcb_connection_has_error( g_conn )) windowClosed = true;
}

#endif

// ----------------------------------------------------------------------------
// device
// ----------------------------------------------------------------------------

static bool HasExtension( const std::vector<VkExtensionProperties>& list, const char* name )
{
	for (const VkExtensionProperties& e : list) if (!strcmp( e.extensionName, name )) return true;
	return false;
}

static void InitDevice()
{
	// instance
	VkApplicationInfo app{ VK_STRUCTURE_TYPE_APPLICATION_INFO };
	app.pApplicationName = "tinybvh Vulkan benchmark";
	app.apiVersion = VK_API_VERSION_1_2; // ray tracing needs SPIR-V 1.4, core in 1.2
	const char* instExt[] = { VK_KHR_SURFACE_EXTENSION_NAME, PLATFORM_SURFACE_EXTENSION_NAME };
	const char* layers[] = { "VK_LAYER_KHRONOS_validation" };
	VkInstanceCreateInfo ici{ VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO };
	ici.pApplicationInfo = &app;
	ici.enabledExtensionCount = 2;
	ici.ppEnabledExtensionNames = instExt;
	if (ENABLE_VALIDATION) ici.enabledLayerCount = 1, ici.ppEnabledLayerNames = layers;
	if (vkCreateInstance( &ici, nullptr, &instance ) != VK_SUCCESS)
	{
		// retry without the validation layer, which may not be installed
		ici.enabledLayerCount = 0;
		VK_CHECK( vkCreateInstance( &ici, nullptr, &instance ) );
	}
	// physical device: prefer a discrete GPU that can do ray tracing pipelines
	uint32_t count = 0;
	VK_CHECK( vkEnumeratePhysicalDevices( instance, &count, nullptr ) );
	if (count == 0) Fatal( "no Vulkan devices found" );
	std::vector<VkPhysicalDevice> devices( count );
	VK_CHECK( vkEnumeratePhysicalDevices( instance, &count, devices.data() ) );
	int bestScore = -1;
	std::vector<VkExtensionProperties> bestExt;
	for (VkPhysicalDevice pd : devices)
	{
		uint32_t n = 0;
		vkEnumerateDeviceExtensionProperties( pd, nullptr, &n, nullptr );
		std::vector<VkExtensionProperties> ext( n );
		vkEnumerateDeviceExtensionProperties( pd, nullptr, &n, ext.data() );
		if (!HasExtension( ext, VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME )) continue;
		if (!HasExtension( ext, VK_KHR_RAY_TRACING_PIPELINE_EXTENSION_NAME )) continue;
		VkPhysicalDeviceProperties p;
		vkGetPhysicalDeviceProperties( pd, &p );
		if (p.apiVersion < VK_API_VERSION_1_2) continue;
		int score = (p.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) ? 2 : 1;
		if (score > bestScore) bestScore = score, physicalDevice = pd, bestExt = ext, deviceProps = p;
	}
	if (physicalDevice == VK_NULL_HANDLE)
		Fatal( "no device with VK_KHR_acceleration_structure + VK_KHR_ray_tracing_pipeline" );
	rayQuerySupported = HasExtension( bestExt, VK_KHR_RAY_QUERY_EXTENSION_NAME );
	if (!rayQuerySupported)
	{
		activeBackends = BACKEND_COUNT - 1;
		printf( "VK_KHR_ray_query not available; the %s backend is disabled.\n", backendName[BACKEND_RAYQUERY] );
	}
	printf( "device: %s\n", deviceProps.deviceName );
	vkGetPhysicalDeviceMemoryProperties( physicalDevice, &memProps );
	// ray tracing / acceleration structure limits
	asProps.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_PROPERTIES_KHR;
	rtProps.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_PROPERTIES_KHR;
	asProps.pNext = &rtProps;
	VkPhysicalDeviceProperties2 props2{ VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2 };
	props2.pNext = &asProps;
	vkGetPhysicalDeviceProperties2( physicalDevice, &props2 );
	deviceProps = props2.properties;
	if (deviceProps.limits.timestampPeriod == 0.0f) Fatal( "device does not support timestamp queries" );
	// queue: one graphics+compute family, which is also the present family on Windows
	uint32_t qCount = 0;
	vkGetPhysicalDeviceQueueFamilyProperties( physicalDevice, &qCount, nullptr );
	std::vector<VkQueueFamilyProperties> qProps( qCount );
	vkGetPhysicalDeviceQueueFamilyProperties( physicalDevice, &qCount, qProps.data() );
	bool found = false;
	for (uint32_t i = 0; i < qCount && !found; i++)
		if ((qProps[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) && (qProps[i].queueFlags & VK_QUEUE_COMPUTE_BIT) &&
			qProps[i].timestampValidBits > 0)
			queueFamily = i, found = true;
	if (!found) Fatal( "no graphics+compute queue with timestamp support" );
	const uint32_t validBits = qProps[queueFamily].timestampValidBits;
	timestampMask = (validBits >= 64) ? ~0ull : ((1ull << validBits) - 1ull);
	// device
	std::vector<const char*> devExt = {
		VK_KHR_SWAPCHAIN_EXTENSION_NAME,
		VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME,
		VK_KHR_RAY_TRACING_PIPELINE_EXTENSION_NAME,
		VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME };
	if (rayQuerySupported) devExt.push_back( VK_KHR_RAY_QUERY_EXTENSION_NAME );
	VkPhysicalDeviceRayQueryFeaturesKHR fRQ{ VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_QUERY_FEATURES_KHR };
	fRQ.rayQuery = VK_TRUE;
	VkPhysicalDeviceRayTracingPipelineFeaturesKHR fRT{ VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_FEATURES_KHR };
	fRT.rayTracingPipeline = VK_TRUE;
	fRT.pNext = rayQuerySupported ? (void*)&fRQ : nullptr;
	VkPhysicalDeviceAccelerationStructureFeaturesKHR fAS{ VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR };
	fAS.accelerationStructure = VK_TRUE;
	fAS.pNext = &fRT;
	VkPhysicalDeviceVulkan12Features f12{ VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES };
	f12.bufferDeviceAddress = VK_TRUE; // acceleration structure builds need it
	f12.pNext = &fAS;
	VkPhysicalDeviceFeatures2 f2{ VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2 };
	f2.pNext = &f12;
	const float priority = 1.0f;
	VkDeviceQueueCreateInfo qci{ VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO };
	qci.queueFamilyIndex = queueFamily;
	qci.queueCount = 1;
	qci.pQueuePriorities = &priority;
	VkDeviceCreateInfo dci{ VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO };
	dci.pNext = &f2;
	dci.queueCreateInfoCount = 1;
	dci.pQueueCreateInfos = &qci;
	dci.enabledExtensionCount = (uint32_t)devExt.size();
	dci.ppEnabledExtensionNames = devExt.data();
	VK_CHECK( vkCreateDevice( physicalDevice, &dci, nullptr, &device ) );
	vkGetDeviceQueue( device, queueFamily, 0, &queue );
	vk.GetBufferDeviceAddress = (PFN_vkGetBufferDeviceAddressKHR)vkGetDeviceProcAddr( device, "vkGetBufferDeviceAddress" );
	if (!vk.GetBufferDeviceAddress)
		vk.GetBufferDeviceAddress = (PFN_vkGetBufferDeviceAddressKHR)vkGetDeviceProcAddr( device, "vkGetBufferDeviceAddressKHR" );
	if (!vk.GetBufferDeviceAddress) Fatal( "could not load vkGetBufferDeviceAddress" );
#define LOAD_FN(name) \
	vk.name = (PFN_vk##name##KHR)vkGetDeviceProcAddr( device, "vk" #name "KHR" ); \
	if (!vk.name) Fatal( "could not load vk" #name "KHR" )
	LOAD_FN( CreateAccelerationStructure );
	LOAD_FN( DestroyAccelerationStructure );
	LOAD_FN( GetAccelerationStructureBuildSizes );
	LOAD_FN( GetAccelerationStructureDeviceAddress );
	LOAD_FN( CmdBuildAccelerationStructures );
	LOAD_FN( CmdWriteAccelerationStructuresProperties );
	LOAD_FN( CmdCopyAccelerationStructure );
	LOAD_FN( CreateRayTracingPipelines );
	LOAD_FN( GetRayTracingShaderGroupHandles );
	LOAD_FN( CmdTraceRays );
#undef LOAD_FN
}

static void InitCommand()
{
	VkCommandPoolCreateInfo pci{ VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO };
	pci.queueFamilyIndex = queueFamily;
	pci.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
	VK_CHECK( vkCreateCommandPool( device, &pci, nullptr, &cmdPool ) );
	pci.flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
	VK_CHECK( vkCreateCommandPool( device, &pci, nullptr, &setupPool ) );
	VkCommandBufferAllocateInfo ai{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
	ai.commandPool = cmdPool;
	ai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	ai.commandBufferCount = FRAME_COUNT;
	VK_CHECK( vkAllocateCommandBuffers( device, &ai, cmdBufs ) );
	// Fences start signalled so the first pass through Render() does not block.
	VkFenceCreateInfo fci{ VK_STRUCTURE_TYPE_FENCE_CREATE_INFO };
	fci.flags = VK_FENCE_CREATE_SIGNALED_BIT;
	VkSemaphoreCreateInfo sci{ VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO };
	for (uint32_t i = 0; i < FRAME_COUNT; i++)
	{
		VK_CHECK( vkCreateFence( device, &fci, nullptr, &frameFences[i] ) );
		VK_CHECK( vkCreateSemaphore( device, &sci, nullptr, &imageAvailable[i] ) );
	}
}

static void InitQueryPool()
{
	VkQueryPoolCreateInfo qpi{ VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO };
	qpi.queryType = VK_QUERY_TYPE_TIMESTAMP;
	qpi.queryCount = FRAME_COUNT * 2;
	VK_CHECK( vkCreateQueryPool( device, &qpi, nullptr, &timestampPool ) );
}

// ----------------------------------------------------------------------------
// surfaces
// ----------------------------------------------------------------------------

static void DestroySwapChain()
{
	for (VkSemaphore s : renderFinished) vkDestroySemaphore( device, s, nullptr );
	renderFinished.clear();
	if (swapChain) vkDestroySwapchainKHR( device, swapChain, nullptr ), swapChain = VK_NULL_HANDLE;
	VkSemaphoreCreateInfo sci{ VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO };
	for (uint32_t i = 0; i < FRAME_COUNT; i++)
	{
		if (imageAvailable[i]) vkDestroySemaphore( device, imageAvailable[i], nullptr );
		VK_CHECK( vkCreateSemaphore( device, &sci, nullptr, &imageAvailable[i] ) );
	}
}

static void CreateSwapChain()
{
	VkSurfaceCapabilitiesKHR caps;
	VK_CHECK( vkGetPhysicalDeviceSurfaceCapabilitiesKHR( physicalDevice, surface, &caps ) );
	if (caps.currentExtent.width == 0xFFFFFFFF)
		swapExtent = { rtWidth, rtHeight };
	else
		swapExtent = caps.currentExtent;
	if (swapExtent.width == 0 || swapExtent.height == 0) return; // minimized
	uint32_t n = 0;
	vkGetPhysicalDeviceSurfaceFormatsKHR( physicalDevice, surface, &n, nullptr );
	std::vector<VkSurfaceFormatKHR> formats( n );
	vkGetPhysicalDeviceSurfaceFormatsKHR( physicalDevice, surface, &n, formats.data() );
	VkSurfaceFormatKHR chosen = formats[0];
	for (const VkSurfaceFormatKHR& f : formats)
		if (f.format == VK_FORMAT_R8G8B8A8_UNORM && f.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) { chosen = f; break; }
	swapFormat = chosen.format;
	vkGetPhysicalDeviceSurfacePresentModesKHR( physicalDevice, surface, &n, nullptr );
	std::vector<VkPresentModeKHR> modes( n );
	vkGetPhysicalDeviceSurfacePresentModesKHR( physicalDevice, surface, &n, modes.data() );
	VkPresentModeKHR present = VK_PRESENT_MODE_FIFO_KHR;
	for (VkPresentModeKHR m : modes) if (m == VK_PRESENT_MODE_MAILBOX_KHR) present = m;
	for (VkPresentModeKHR m : modes) if (m == VK_PRESENT_MODE_IMMEDIATE_KHR) present = m;
	uint32_t imageCount = caps.minImageCount < FRAME_COUNT ? FRAME_COUNT : caps.minImageCount;
	if (caps.maxImageCount && imageCount > caps.maxImageCount) imageCount = caps.maxImageCount;
	VkSwapchainCreateInfoKHR sci{ VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR };
	sci.surface = surface;
	sci.minImageCount = imageCount;
	sci.imageFormat = swapFormat;
	sci.imageColorSpace = chosen.colorSpace;
	sci.imageExtent = swapExtent;
	sci.imageArrayLayers = 1;
	sci.imageUsage = VK_IMAGE_USAGE_TRANSFER_DST_BIT;
	sci.preTransform = caps.currentTransform;
	sci.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
	sci.presentMode = present;
	sci.clipped = VK_TRUE;
	VK_CHECK( vkCreateSwapchainKHR( device, &sci, nullptr, &swapChain ) );
	VK_CHECK( vkGetSwapchainImagesKHR( device, swapChain, &n, nullptr ) );
	swapImages.resize( n );
	VK_CHECK( vkGetSwapchainImagesKHR( device, swapChain, &n, swapImages.data() ) );
	VkSemaphoreCreateInfo semci{ VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO };
	renderFinished.resize( n );
	for (uint32_t i = 0; i < n; i++) VK_CHECK( vkCreateSemaphore( device, &semci, nullptr, &renderFinished[i] ) );
}

static void InitSurfaces()
{
	surface = CreatePlatformSurface( instance );
	VkBool32 ok = VK_FALSE;
	VK_CHECK( vkGetPhysicalDeviceSurfaceSupportKHR( physicalDevice, queueFamily, surface, &ok ) );
	if (!ok) Fatal( "queue family %u cannot present to this surface", queueFamily );
	CreateSwapChain();
	VkImageCreateInfo ii{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
	ii.imageType = VK_IMAGE_TYPE_2D;
	ii.format = VK_FORMAT_R8G8B8A8_UNORM;
	ii.extent = { rtWidth, rtHeight, 1 };
	ii.mipLevels = 1;
	ii.arrayLayers = 1;
	ii.samples = VK_SAMPLE_COUNT_1_BIT;
	ii.tiling = VK_IMAGE_TILING_OPTIMAL;
	ii.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
	ii.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	VK_CHECK( vkCreateImage( device, &ii, nullptr, &renderTarget ) );
	VkMemoryRequirements mr;
	vkGetImageMemoryRequirements( device, renderTarget, &mr );
	VkMemoryAllocateInfo ai{ VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
	ai.allocationSize = mr.size;
	ai.memoryTypeIndex = FindMemoryType( mr.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT );
	VK_CHECK( vkAllocateMemory( device, &ai, nullptr, &renderTargetMem ) );
	VK_CHECK( vkBindImageMemory( device, renderTarget, renderTargetMem, 0 ) );
	VkImageViewCreateInfo vi{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
	vi.image = renderTarget;
	vi.viewType = VK_IMAGE_VIEW_TYPE_2D;
	vi.format = VK_FORMAT_R8G8B8A8_UNORM;
	vi.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
	VK_CHECK( vkCreateImageView( device, &vi, nullptr, &renderTargetView ) );
	// UNDEFINED -> GENERAL once; it stays in GENERAL except during the blit.
	VkCommandBuffer cb = BeginOneShot();
	VkImageMemoryBarrier b{ VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER };
	b.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	b.newLayout = VK_IMAGE_LAYOUT_GENERAL;
	b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	b.image = renderTarget;
	b.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
	b.dstAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
	vkCmdPipelineBarrier( cb, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
		VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 0, nullptr, 0, nullptr, 1, &b );
	EndOneShot( cb );
}

// ----------------------------------------------------------------------------
// scene and BVH data
// ----------------------------------------------------------------------------

static void InitMeshes()
{
	AddMesh( "../../testdata/cryteksponza.bin" );
	meshVB = MakeDeviceBuffer( verts, (VkDeviceSize)triCount * 3 * sizeof( bvhvec4 ),
		(VkDeviceSize)triCount * 3 * sizeof( bvhvec4 ),
		VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, "meshVB" );
	// One spatial-split build, converted to both GPU layouts.
	bvh2.BuildHQ( verts, triCount );
	bvh2.Optimize();
	bvh.ConvertFrom( bvh2, true );    // Aila & Laine 64-byte nodes + primIdx + separate tris
	mbvh4.ConvertFrom( bvh2, true );  // collapse the BVH2 into a 4-wide BVH
	bvh4.ConvertFrom( mbvh4, true );  // quantize into the single-blob BVH4_GPU layout
	cwbvh.BuildHQ( verts, triCount ); // separate SBVH build: see the note at 'cwbvh'.
	cwbvh.Optimize();
}

static void InitBVHBuffers()
{
	const VkDeviceSize nodeBytes = (VkDeviceSize)bvh.allocatedNodes * sizeof( GPUBVHNode );
	const VkDeviceSize idxBytes = (VkDeviceSize)bvh.bvh.idxCount * sizeof( unsigned );
	const VkDeviceSize vertBytes = (VkDeviceSize)bvh.bvh.idxCount * 3 * sizeof( bvhvec4 );
	// BVH4_GPU counts storage in 16-byte 'blocks'; usedBlocks is what traversal touches.
	const VkDeviceSize bvh4Bytes = (VkDeviceSize)bvh4.usedBlocks * sizeof( bvhvec4 );
	// CWBVH keeps nodes (5 blocks each, again counted in usedBlocks) and triangles
	// in two separate blobs; the triangle stride must match the tiny8.comp define.
	const VkDeviceSize cwbvhNodeBytes = (VkDeviceSize)cwbvh.usedBlocks * sizeof( bvhvec4 );
#ifdef CWBVH_COMPRESSED_TRIS
	const VkDeviceSize cwbvhTriBytes = (VkDeviceSize)cwbvh.idxCount * 4 * sizeof( bvhvec4 );
#else
	const VkDeviceSize cwbvhTriBytes = (VkDeviceSize)cwbvh.idxCount * 3 * sizeof( bvhvec4 );
#endif
	const VkBufferUsageFlags use = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
	bvhNodeBuffer = MakeDeviceBuffer( bvh.bvhNode, nodeBytes, nodeBytes, use, "bvhNodeBuffer" );
	bvhIdxBuffer = MakeDeviceBuffer( bvh.bvh.primIdx, idxBytes, idxBytes, use, "bvhIdxBuffer" );
	bvhVertBuffer = MakeDeviceBuffer( (bvhvec4*)bvh.orderedVerts.data, vertBytes, vertBytes, use, "bvhVertBuffer" );
	bvh4DataBuffer = MakeDeviceBuffer( bvh4.bvh4Data, bvh4Bytes, bvh4Bytes, use, "bvh4DataBuffer" );
	cwbvhNodeBuffer = MakeDeviceBuffer( cwbvh.bvh8Data, cwbvhNodeBytes, cwbvhNodeBytes, use, "cwbvhNodeBuffer" );
	cwbvhTriBuffer = MakeDeviceBuffer( cwbvh.bvh8Tris, cwbvhTriBytes, cwbvhTriBytes, use, "cwbvhTriBuffer" );
}

static void UpdateRayBuffer()
{
	const VkDeviceSize size = 32ull * rtWidth * rtHeight; // 32 bytes per ray
	Buffer staging = CreateBuffer( size, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, true );
	float* data = (float*)staging.mapped;
	memset( data, 0, (size_t)size );
	const char* raySetPath = "../../view3rays.bin";
	FILE* f = fopen( raySetPath, "rb" );
	if (f)
	{
		const long long expected = 2ll * rtWidth * rtHeight * 3 * (long long)sizeof( float );
		fseek( f, 0, SEEK_END );
		const long long bytes = ftell( f );
		fseek( f, 0, SEEK_SET );
		size_t got = 0;
		for (uint32_t i = 0; i < rtWidth * rtHeight; i++) got += fread( data + i * 8, 4, 3, f );
		for (uint32_t i = 0; i < rtWidth * rtHeight; i++) got += fread( data + i * 8 + 4, 4, 3, f );
		fclose( f );
		printf( "rays: %s, %u x %u\n", raySetPath, rtWidth, rtHeight );
	}
	else
	{
		char cwd[1024] = {};
		GetCwd( cwd, sizeof( cwd ) );
		Fatal( "could not open '%s' (working directory: %s).\n\n", raySetPath, cwd );
	}
	// Device-local buffer the shaders actually read from on every ray.
	rayBuffer = CreateBuffer( size, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT );
	VkCommandBuffer cb = BeginOneShot();
	VkBufferCopy region{ 0, 0, size };
	vkCmdCopyBuffer( cb, staging.buf, rayBuffer.buf, 1, &region );
	EndOneShot( cb );
	DestroyBuffer( staging );
}

// ----------------------------------------------------------------------------
// acceleration structures
// ----------------------------------------------------------------------------

static VkDeviceAddress ASAddress( VkAccelerationStructureKHR as )
{
	VkAccelerationStructureDeviceAddressInfoKHR ai{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR };
	ai.accelerationStructure = as;
	return vk.GetAccelerationStructureDeviceAddress( device, &ai );
}

static AccelStruct MakeAccelerationStructure( VkAccelerationStructureTypeKHR type,
	VkBuildAccelerationStructureFlagsKHR flags,
	const VkAccelerationStructureGeometryKHR& geometry, uint32_t primitiveCount,
	VkDeviceSize* updateScratchSize = nullptr )
{
	VkAccelerationStructureBuildGeometryInfoKHR bi{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR };
	bi.type = type;
	bi.flags = flags;
	bi.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
	bi.geometryCount = 1;
	bi.pGeometries = &geometry;
	VkAccelerationStructureBuildSizesInfoKHR sizes{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR };
	vk.GetAccelerationStructureBuildSizes( device, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
		&bi, &primitiveCount, &sizes );
	if (updateScratchSize) *updateScratchSize = sizes.updateScratchSize;
	AccelStruct as;
	as.buffer = CreateBuffer( sizes.accelerationStructureSize,
		VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT );
	VkAccelerationStructureCreateInfoKHR ci{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR };
	ci.buffer = as.buffer.buf;
	ci.size = sizes.accelerationStructureSize;
	ci.type = type;
	VK_CHECK( vk.CreateAccelerationStructure( device, &ci, nullptr, &as.handle ) );
	const VkDeviceSize align = asProps.minAccelerationStructureScratchOffsetAlignment;
	Buffer scratch = CreateBuffer( sizes.buildScratchSize + align,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT );
	bi.dstAccelerationStructure = as.handle;
	bi.scratchData.deviceAddress = AlignUp( scratch.addr, align );
	VkAccelerationStructureBuildRangeInfoKHR range{};
	range.primitiveCount = primitiveCount;
	const VkAccelerationStructureBuildRangeInfoKHR* pRange = &range;
	VkCommandBuffer cb = BeginOneShot();
	vk.CmdBuildAccelerationStructures( cb, 1, &bi, &pRange );
	EndOneShot( cb );
	DestroyBuffer( scratch );
	as.addr = ASAddress( as.handle );
	return as;
}

static AccelStruct MakeBLAS( const Buffer& vertexBuffer, uint32_t vertexCount, uint32_t vertexStride )
{
	VkAccelerationStructureGeometryKHR geom{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR };
	geom.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR;
	geom.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;
	geom.geometry.triangles.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR;
	geom.geometry.triangles.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT;
	geom.geometry.triangles.vertexData.deviceAddress = vertexBuffer.addr;
	geom.geometry.triangles.vertexStride = vertexStride;
	geom.geometry.triangles.maxVertex = vertexCount - 1; // highest index, not the count
	geom.geometry.triangles.indexType = VK_INDEX_TYPE_NONE_KHR;
	return MakeAccelerationStructure( VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR,
		VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR |
		VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_COMPACTION_BIT_KHR,
		geom, vertexCount / 3 );
}

static AccelStruct CompactBLAS( AccelStruct as )
{
	VkQueryPoolCreateInfo qpi{ VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO };
	qpi.queryType = VK_QUERY_TYPE_ACCELERATION_STRUCTURE_COMPACTED_SIZE_KHR;
	qpi.queryCount = 1;
	VkQueryPool pool;
	VK_CHECK( vkCreateQueryPool( device, &qpi, nullptr, &pool ) );
	VkCommandBuffer cb = BeginOneShot();
	vkCmdResetQueryPool( cb, pool, 0, 1 );
	// must barrier the build's writes before reading postbuild info
	VkMemoryBarrier mb{ VK_STRUCTURE_TYPE_MEMORY_BARRIER };
	mb.srcAccessMask = VK_ACCESS_ACCELERATION_STRUCTURE_WRITE_BIT_KHR;
	mb.dstAccessMask = VK_ACCESS_ACCELERATION_STRUCTURE_READ_BIT_KHR;
	vkCmdPipelineBarrier( cb, VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR,
		VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR, 0, 1, &mb, 0, nullptr, 0, nullptr );
	vk.CmdWriteAccelerationStructuresProperties( cb, 1, &as.handle,
		VK_QUERY_TYPE_ACCELERATION_STRUCTURE_COMPACTED_SIZE_KHR, pool, 0 );
	EndOneShot( cb );
	VkDeviceSize compactedSize = 0;
	VK_CHECK( vkGetQueryPoolResults( device, pool, 0, 1, sizeof( compactedSize ), &compactedSize,
		sizeof( compactedSize ), VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT ) );
	vkDestroyQueryPool( device, pool, nullptr );
	printf( "BLAS: %i tris, %.2f MB compacted\n", triCount, (double)compactedSize / (1024 * 1024) );
	AccelStruct compacted;
	compacted.buffer = CreateBuffer( compactedSize,
		VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT );
	VkAccelerationStructureCreateInfoKHR ci{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR };
	ci.buffer = compacted.buffer.buf;
	ci.size = compactedSize;
	ci.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
	VK_CHECK( vk.CreateAccelerationStructure( device, &ci, nullptr, &compacted.handle ) );
	cb = BeginOneShot();
	VkCopyAccelerationStructureInfoKHR cpy{ VK_STRUCTURE_TYPE_COPY_ACCELERATION_STRUCTURE_INFO_KHR };
	cpy.src = as.handle;
	cpy.dst = compacted.handle;
	cpy.mode = VK_COPY_ACCELERATION_STRUCTURE_MODE_COMPACT_KHR;
	vk.CmdCopyAccelerationStructure( cb, &cpy );
	EndOneShot( cb );
	vk.DestroyAccelerationStructure( device, as.handle, nullptr );
	DestroyBuffer( as.buffer );
	compacted.addr = ASAddress( compacted.handle );
	return compacted;
}

static void InitScene()
{
	instanceBuffer = CreateBuffer( sizeof( VkAccelerationStructureInstanceKHR ) * NUM_INSTANCES,
		VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, true );
	VkAccelerationStructureInstanceKHR* inst = (VkAccelerationStructureInstanceKHR*)instanceBuffer.mapped;
	memset( inst, 0, sizeof( VkAccelerationStructureInstanceKHR ) * NUM_INSTANCES );
	// row-major 3x4 identity, i.e. XMMatrixTranslation( 0, 0, 0 )
	inst[0].transform.matrix[0][0] = inst[0].transform.matrix[1][1] = inst[0].transform.matrix[2][2] = 1.0f;
	inst[0].instanceCustomIndex = 0;
	inst[0].mask = 1; // the shaders trace with cull mask 0xFF, so this is visible
	inst[0].instanceShaderBindingTableRecordOffset = 0;
	inst[0].flags = 0;
	inst[0].accelerationStructureReference = blas.addr;
}

static void InitTopLevel()
{
	VkAccelerationStructureGeometryKHR geom{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR };
	geom.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
	geom.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;
	geom.geometry.instances.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR;
	geom.geometry.instances.arrayOfPointers = VK_FALSE;
	geom.geometry.instances.data.deviceAddress = instanceBuffer.addr;
	VkDeviceSize updateScratchSize = 0;
	tlas = MakeAccelerationStructure( VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR,
		VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR |
		VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR,
		geom, NUM_INSTANCES, &updateScratchSize );
	tlasUpdateScratch = CreateBuffer( updateScratchSize + asProps.minAccelerationStructureScratchOffsetAlignment,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT );
}

// ----------------------------------------------------------------------------
// descriptors and pipelines
// ----------------------------------------------------------------------------

static void InitDescriptors()
{
	const VkShaderStageFlags stages = VK_SHADER_STAGE_COMPUTE_BIT |
		VK_SHADER_STAGE_RAYGEN_BIT_KHR | VK_SHADER_STAGE_MISS_BIT_KHR |
		VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR;
	VkDescriptorSetLayoutBinding bindings[9]{};
	for (uint32_t i = 0; i < 9; i++)
	{
		bindings[i].binding = i;
		bindings[i].descriptorCount = 1;
		bindings[i].stageFlags = stages;
		bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
	}
	bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;            // render target
	bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR; // TLAS
	VkDescriptorSetLayoutCreateInfo lci{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
	lci.bindingCount = 9;
	lci.pBindings = bindings;
	VK_CHECK( vkCreateDescriptorSetLayout( device, &lci, nullptr, &descLayout ) );
	VkDescriptorPoolSize sizes[3] = {
		{ VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1 },
		{ VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR, 1 },
		{ VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 7 } };
	VkDescriptorPoolCreateInfo pci{ VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
	pci.maxSets = 1;
	pci.poolSizeCount = 3;
	pci.pPoolSizes = sizes;
	VK_CHECK( vkCreateDescriptorPool( device, &pci, nullptr, &descPool ) );
	VkDescriptorSetAllocateInfo dai{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
	dai.descriptorPool = descPool;
	dai.descriptorSetCount = 1;
	dai.pSetLayouts = &descLayout;
	VK_CHECK( vkAllocateDescriptorSets( device, &dai, &descSet ) );
	VkPipelineLayoutCreateInfo plci{ VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
	plci.setLayoutCount = 1;
	plci.pSetLayouts = &descLayout;
	VK_CHECK( vkCreatePipelineLayout( device, &plci, nullptr, &pipelineLayout ) );
	VkDescriptorImageInfo imgInfo{ VK_NULL_HANDLE, renderTargetView, VK_IMAGE_LAYOUT_GENERAL };
	VkWriteDescriptorSetAccelerationStructureKHR asInfo{ VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET_ACCELERATION_STRUCTURE_KHR };
	asInfo.accelerationStructureCount = 1;
	asInfo.pAccelerationStructures = &tlas.handle;
	const Buffer* bufs[7] = { &rayBuffer, &bvhNodeBuffer, &bvhIdxBuffer, &bvhVertBuffer,
		&bvh4DataBuffer, &cwbvhNodeBuffer, &cwbvhTriBuffer };
	VkDescriptorBufferInfo bufInfo[7]{};
	VkWriteDescriptorSet writes[9]{};
	for (uint32_t i = 0; i < 9; i++)
	{
		writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		writes[i].dstSet = descSet;
		writes[i].dstBinding = i;
		writes[i].descriptorCount = 1;
	}
	writes[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
	writes[0].pImageInfo = &imgInfo;
	writes[1].descriptorType = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR;
	writes[1].pNext = &asInfo;
	for (uint32_t i = 0; i < 7; i++)
	{
		bufInfo[i] = { bufs[i]->buf, 0, VK_WHOLE_SIZE };
		writes[i + 2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
		writes[i + 2].pBufferInfo = &bufInfo[i];
	}
	vkUpdateDescriptorSets( device, 9, writes, 0, nullptr );
}

static VkShaderModule LoadShaderModule( const char* name )
{
	static const char* dirs[] = { "shaders/", "../shaders/", "../../shaders/", "" };
	for (const char* dir : dirs)
	{
		char path[512];
		snprintf( path, sizeof( path ), "%s%s", dir, name );
		std::ifstream f( path, std::ios::binary | std::ios::ate );
		if (!f.is_open()) continue;
		const std::streamsize bytes = f.tellg();
		if (bytes <= 0 || (bytes & 3)) Fatal( "'%s' is not a valid SPIR-V module", path );
		std::vector<uint32_t> code( (size_t)bytes / 4 );
		f.seekg( 0 );
		f.read( (char*)code.data(), bytes );
		VkShaderModuleCreateInfo ci{ VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO };
		ci.codeSize = (size_t)bytes;
		ci.pCode = code.data();
		VkShaderModule m;
		VK_CHECK( vkCreateShaderModule( device, &ci, nullptr, &m ) );
		return m;
	}
	Fatal( "could not find '%s'; run compile_shaders.bat first", name );
	return VK_NULL_HANDLE;
}

static void InitPipeline()
{
	VkShaderModule rgen = LoadShaderModule( "trace.rgen.spv" );
	VkShaderModule rmiss = LoadShaderModule( "trace.rmiss.spv" );
	VkShaderModule rchit = LoadShaderModule( "trace.rchit.spv" );
	VkPipelineShaderStageCreateInfo stages[3]{};
	const VkShaderStageFlagBits stageBits[3] = { VK_SHADER_STAGE_RAYGEN_BIT_KHR,
		VK_SHADER_STAGE_MISS_BIT_KHR, VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR };
	const VkShaderModule modules[3] = { rgen, rmiss, rchit };
	for (uint32_t i = 0; i < 3; i++)
	{
		stages[i].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
		stages[i].stage = stageBits[i];
		stages[i].module = modules[i];
		stages[i].pName = "main";
	}
	VkRayTracingShaderGroupCreateInfoKHR groups[NUM_SHADER_GROUPS]{};
	for (uint32_t i = 0; i < NUM_SHADER_GROUPS; i++)
	{
		groups[i].sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
		groups[i].generalShader = VK_SHADER_UNUSED_KHR;
		groups[i].closestHitShader = VK_SHADER_UNUSED_KHR;
		groups[i].anyHitShader = VK_SHADER_UNUSED_KHR;
		groups[i].intersectionShader = VK_SHADER_UNUSED_KHR;
	}
	groups[0].type = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR, groups[0].generalShader = 0;
	groups[1].type = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR, groups[1].generalShader = 1;
	groups[2].type = VK_RAY_TRACING_SHADER_GROUP_TYPE_TRIANGLES_HIT_GROUP_KHR, groups[2].closestHitShader = 2;
	VkRayTracingPipelineCreateInfoKHR ci{ VK_STRUCTURE_TYPE_RAY_TRACING_PIPELINE_CREATE_INFO_KHR };
	ci.stageCount = 3;
	ci.pStages = stages;
	ci.groupCount = NUM_SHADER_GROUPS;
	ci.pGroups = groups;
	ci.maxPipelineRayRecursionDepth = 1;
	ci.layout = pipelineLayout;
	VK_CHECK( vk.CreateRayTracingPipelines( device, VK_NULL_HANDLE, VK_NULL_HANDLE, 1, &ci, nullptr, &rtPipeline ) );
	for (VkShaderModule m : modules) vkDestroyShaderModule( device, m, nullptr );
}

static VkPipeline MakeComputePipeline( const char* spv )
{
	VkShaderModule m = LoadShaderModule( spv );
	VkComputePipelineCreateInfo ci{ VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO };
	ci.stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
	ci.stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
	ci.stage.module = m;
	ci.stage.pName = "main";
	ci.layout = pipelineLayout;
	VkPipeline p;
	VK_CHECK( vkCreateComputePipelines( device, VK_NULL_HANDLE, 1, &ci, nullptr, &p ) );
	vkDestroyShaderModule( device, m, nullptr );
	return p;
}

static void InitComputePipeline()
{
	computePso = MakeComputePipeline( "tiny.comp.spv" );
	computePso4 = MakeComputePipeline( "tiny4.comp.spv" );
	computePso8 = MakeComputePipeline( "tiny8.comp.spv" );
	if (rayQuerySupported) computePsoRQ = MakeComputePipeline( "tinyrq.comp.spv" );
}

static void InitShaderTables()
{
	const uint32_t handleSize = rtProps.shaderGroupHandleSize;
	const uint32_t handleAlign = rtProps.shaderGroupHandleAlignment;
	const uint32_t baseAlign = rtProps.shaderGroupBaseAlignment;
	// aligning to the larger of the two satisfies both, as both are powers of two
	const uint32_t stride = AlignUp( handleSize, handleAlign > baseAlign ? handleAlign : baseAlign );
	std::vector<uint8_t> handles( (size_t)handleSize * NUM_SHADER_GROUPS );
	VK_CHECK( vk.GetRayTracingShaderGroupHandles( device, rtPipeline, 0, NUM_SHADER_GROUPS,
		handles.size(), handles.data() ) );
	// over-allocate so the first record can start at an aligned device address
	sbt = CreateBuffer( (VkDeviceSize)stride * NUM_SHADER_GROUPS + baseAlign,
		VK_BUFFER_USAGE_SHADER_BINDING_TABLE_BIT_KHR | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, true );
	const VkDeviceAddress base = AlignUp( sbt.addr, (VkDeviceAddress)baseAlign );
	uint8_t* dst = (uint8_t*)sbt.mapped + (base - sbt.addr);
	memset( dst, 0, (size_t)stride * NUM_SHADER_GROUPS );
	for (uint32_t i = 0; i < NUM_SHADER_GROUPS; i++)
		memcpy( dst + (size_t)i * stride, handles.data() + (size_t)i * handleSize, handleSize );
	sbtRaygen = { base + 0 * stride, stride, stride };
	sbtMiss = { base + 1 * stride, stride, stride };
	sbtHit = { base + 2 * stride, stride, stride };
	sbtCallable = { 0, 0, 0 };
}

// ----------------------------------------------------------------------------
// per-frame work
// ----------------------------------------------------------------------------

static void UpdateScene( VkCommandBuffer cb )
{
	VkAccelerationStructureGeometryKHR geom{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR };
	geom.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
	geom.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;
	geom.geometry.instances.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR;
	geom.geometry.instances.arrayOfPointers = VK_FALSE;
	geom.geometry.instances.data.deviceAddress = instanceBuffer.addr;
	VkAccelerationStructureBuildGeometryInfoKHR bi{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR };
	bi.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
	// Must match the flags the TLAS was originally built with.
	bi.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR |
		VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
	bi.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_UPDATE_KHR;
	bi.srcAccelerationStructure = tlas.handle;
	bi.dstAccelerationStructure = tlas.handle; // in-place update
	bi.geometryCount = 1;
	bi.pGeometries = &geom;
	bi.scratchData.deviceAddress = AlignUp( tlasUpdateScratch.addr,
		(VkDeviceAddress)asProps.minAccelerationStructureScratchOffsetAlignment );
	VkAccelerationStructureBuildRangeInfoKHR range{};
	range.primitiveCount = NUM_INSTANCES;
	const VkAccelerationStructureBuildRangeInfoKHR* pRange = &range;
	vk.CmdBuildAccelerationStructures( cb, 1, &bi, &pRange );
	VkMemoryBarrier mb{ VK_STRUCTURE_TYPE_MEMORY_BARRIER };
	mb.srcAccessMask = VK_ACCESS_ACCELERATION_STRUCTURE_WRITE_BIT_KHR;
	mb.dstAccessMask = VK_ACCESS_ACCELERATION_STRUCTURE_READ_BIT_KHR | VK_ACCESS_SHADER_READ_BIT;
	vkCmdPipelineBarrier( cb, VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR,
		VK_PIPELINE_STAGE_RAY_TRACING_SHADER_BIT_KHR | VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
		0, 1, &mb, 0, nullptr, 0, nullptr );
}

static void ImageBarrier( VkCommandBuffer cb, VkImage image,
	VkImageLayout oldLayout, VkImageLayout newLayout,
	VkAccessFlags srcAccess, VkAccessFlags dstAccess,
	VkPipelineStageFlags srcStage, VkPipelineStageFlags dstStage )
{
	VkImageMemoryBarrier b{ VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER };
	b.srcAccessMask = srcAccess;
	b.dstAccessMask = dstAccess;
	b.oldLayout = oldLayout;
	b.newLayout = newLayout;
	b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	b.image = image;
	b.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
	vkCmdPipelineBarrier( cb, srcStage, dstStage, 0, 0, nullptr, 0, nullptr, 1, &b );
}

static void Resize();

static void Render()
{
	static uint64_t frameCounter = 0;
	static int slotBackend[FRAME_COUNT] = {}; // which backend last ran in each frame slot
	const uint32_t slot = (uint32_t)(frameCounter % FRAME_COUNT);
	const uint32_t baseQuery = slot * 2;
	// Round-robin the ray tracing backends.
	const int backend = (int)(frameCounter % activeBackends);
	// A zero-size client area (minimized) leaves the swapchain uncreatable.
	if (swapChain == VK_NULL_HANDLE) { Resize(); if (swapChain == VK_NULL_HANDLE) { SleepMs( 16 ); return; } }
	VK_CHECK( vkWaitForFences( device, 1, &frameFences[slot], VK_TRUE, UINT64_MAX ) );
	// This slot's previous submission has retired, so its timestamps are readable.
	if (frameSubmitted[slot])
	{
		frameSubmitted[slot] = false; // consume it, so an early return cannot double count
		uint64_t timestamps[2] = {};
		VK_CHECK( vkGetQueryPoolResults( device, timestampPool, baseQuery, 2,
			sizeof( timestamps ), timestamps, sizeof( uint64_t ),
			VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT ) );
		const uint64_t start = timestamps[0] & timestampMask, end = timestamps[1] & timestampMask;
		// timestampPeriod is nanoseconds per tick
		double rtTime = (double)(end - start) * (double)deviceProps.limits.timestampPeriod * 1e-9;
		rtTime /= (double)NUM_DISPATCHES;
		const double raysPerSecond = (double)(rtWidth * rtHeight) / rtTime;
		const int k = slotBackend[slot]; // which backend produced this timestamp
		static double smoothed[BACKEND_COUNT] = {};
		static double alpha = 0.8;
		static int frames[BACKEND_COUNT] = {};
		if (++frames[k] == 1) smoothed[k] = raysPerSecond;
		else smoothed[k] = alpha * smoothed[k] + (1 - alpha) * raysPerSecond;
		if (alpha < 0.99) alpha += 0.005;
		// the hardware RT pipeline is the reference; report the rest as a fraction of it.
		const double ref = smoothed[BACKEND_HWRT];
		if (backendPrint[k])
		{
			if (k == BACKEND_HWRT || ref <= 0.0)
				printf( "%s: %6.1f MRays/s", backendName[k], smoothed[k] / 1e6 );
			else
				printf( "%s: %6.1f MRays/s, %4.1f%% of %s", backendName[k], smoothed[k] / 1e6,
					100.0 * smoothed[k] / ref, backendName[BACKEND_HWRT] );
		}
		if (k == activeBackends - 1) printf( "\n" ); else printf( "; " );
	}
	uint32_t imageIndex = 0;
	VkResult acq = vkAcquireNextImageKHR( device, swapChain, UINT64_MAX,
		imageAvailable[slot], VK_NULL_HANDLE, &imageIndex );
	if (acq == VK_ERROR_OUT_OF_DATE_KHR) { Resize(); return; }
	if (acq != VK_SUCCESS && acq != VK_SUBOPTIMAL_KHR) VK_CHECK( acq );
	VK_CHECK( vkResetFences( device, 1, &frameFences[slot] ) );
	VkCommandBuffer cb = cmdBufs[slot];
	VK_CHECK( vkResetCommandBuffer( cb, 0 ) );
	VkCommandBufferBeginInfo bi{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
	bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
	VK_CHECK( vkBeginCommandBuffer( cb, &bi ) );
	vkCmdResetQueryPool( cb, timestampPool, baseQuery, 2 );
	static bool sceneDirty = true;
	if (sceneDirty) { UpdateScene( cb ); sceneDirty = false; }
	if (backend != BACKEND_HWRT)
	{
		vkCmdBindPipeline( cb, VK_PIPELINE_BIND_POINT_COMPUTE,
			backend == BACKEND_BVH4_GPU ? computePso4 :
			backend == BACKEND_BVH8_CWBVH ? computePso8 :
			backend == BACKEND_RAYQUERY ? computePsoRQ : computePso );
		vkCmdBindDescriptorSets( cb, VK_PIPELINE_BIND_POINT_COMPUTE, pipelineLayout,
			0, 1, &descSet, 0, nullptr );
		vkCmdWriteTimestamp( cb, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, timestampPool, baseQuery );
		for (uint32_t i = 0; i < NUM_DISPATCHES; i++)
		{
			vkCmdDispatch( cb, (rtWidth + 7) / 8, (rtHeight + 7) / 8, 1 );
			if (BARRIER_BETWEEN_DISPATCHES && i + 1 < NUM_DISPATCHES)
			{
				VkMemoryBarrier mb{ VK_STRUCTURE_TYPE_MEMORY_BARRIER };
				mb.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
				mb.dstAccessMask = VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_SHADER_READ_BIT;
				vkCmdPipelineBarrier( cb, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
					VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1, &mb, 0, nullptr, 0, nullptr );
			}
		}
		vkCmdWriteTimestamp( cb, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, timestampPool, baseQuery + 1 );
	}
	else
	{
		vkCmdBindPipeline( cb, VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR, rtPipeline );
		vkCmdBindDescriptorSets( cb, VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR, pipelineLayout,
			0, 1, &descSet, 0, nullptr );
		vkCmdWriteTimestamp( cb, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, timestampPool, baseQuery );
		for (uint32_t i = 0; i < NUM_DISPATCHES; i++)
		{
			vk.CmdTraceRays( cb, &sbtRaygen, &sbtMiss, &sbtHit, &sbtCallable, rtWidth, rtHeight, 1 );
			if (BARRIER_BETWEEN_DISPATCHES && i + 1 < NUM_DISPATCHES)
			{
				VkMemoryBarrier mb{ VK_STRUCTURE_TYPE_MEMORY_BARRIER };
				mb.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
				mb.dstAccessMask = VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_SHADER_READ_BIT;
				vkCmdPipelineBarrier( cb, VK_PIPELINE_STAGE_RAY_TRACING_SHADER_BIT_KHR,
					VK_PIPELINE_STAGE_RAY_TRACING_SHADER_BIT_KHR, 0, 1, &mb, 0, nullptr, 0, nullptr );
			}
		}
		vkCmdWriteTimestamp( cb, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, timestampPool, baseQuery + 1 );
	}
	slotBackend[slot] = backend; // for the timing readback next time this slot runs
	frameCounter++;
	const VkPipelineStageFlags traceStage = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT |
		VK_PIPELINE_STAGE_RAY_TRACING_SHADER_BIT_KHR;
	ImageBarrier( cb, renderTarget, VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
		VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_TRANSFER_READ_BIT, traceStage, VK_PIPELINE_STAGE_TRANSFER_BIT );
	ImageBarrier( cb, swapImages[imageIndex], VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
		0, VK_ACCESS_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT );
	VkImageBlit blit{};
	blit.srcSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 };
	blit.dstSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 };
	blit.srcOffsets[1] = { (int32_t)rtWidth, (int32_t)rtHeight, 1 };
	blit.dstOffsets[1] = { (int32_t)swapExtent.width, (int32_t)swapExtent.height, 1 };
	vkCmdBlitImage( cb, renderTarget, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
		swapImages[imageIndex], VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &blit, VK_FILTER_LINEAR );
	ImageBarrier( cb, swapImages[imageIndex], VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
		VK_ACCESS_TRANSFER_WRITE_BIT, 0, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT );
	ImageBarrier( cb, renderTarget, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_IMAGE_LAYOUT_GENERAL,
		VK_ACCESS_TRANSFER_READ_BIT, VK_ACCESS_SHADER_WRITE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, traceStage );
	VK_CHECK( vkEndCommandBuffer( cb ) );
	const VkPipelineStageFlags waitStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
	VkSubmitInfo si{ VK_STRUCTURE_TYPE_SUBMIT_INFO };
	si.waitSemaphoreCount = 1;
	si.pWaitSemaphores = &imageAvailable[slot];
	si.pWaitDstStageMask = &waitStage;
	si.commandBufferCount = 1;
	si.pCommandBuffers = &cb;
	si.signalSemaphoreCount = 1;
	si.pSignalSemaphores = &renderFinished[imageIndex];
	VK_CHECK( vkQueueSubmit( queue, 1, &si, frameFences[slot] ) );
	frameSubmitted[slot] = true;
	VkPresentInfoKHR pi{ VK_STRUCTURE_TYPE_PRESENT_INFO_KHR };
	pi.waitSemaphoreCount = 1;
	pi.pWaitSemaphores = &renderFinished[imageIndex];
	pi.swapchainCount = 1;
	pi.pSwapchains = &swapChain;
	pi.pImageIndices = &imageIndex;
	const VkResult pres = vkQueuePresentKHR( queue, &pi );
	if (pres == VK_ERROR_OUT_OF_DATE_KHR || pres == VK_SUBOPTIMAL_KHR) Resize();
	else if (pres != VK_SUCCESS) VK_CHECK( pres );
}

static void Resize()
{
	VK_CHECK( vkDeviceWaitIdle( device ) ); // must fully drain before touching swapchain images
	DestroySwapChain();
	CreateSwapChain();
	for (uint32_t i = 0; i < FRAME_COUNT; i++) frameSubmitted[i] = false;
}

// ----------------------------------------------------------------------------
// entry point
// ----------------------------------------------------------------------------

static void Init()
{
	InitDevice();
	InitCommand();
	InitQueryPool();
	InitSurfaces();
	InitMeshes();
	InitBVHBuffers();
	UpdateRayBuffer();
	blas = CompactBLAS( MakeBLAS( meshVB, triCount * 3, sizeof( bvhvec4 ) ) );
	InitScene();
	InitTopLevel();
	InitDescriptors();
	InitPipeline();
	InitComputePipeline();
	InitShaderTables();
}

int main()
{
	CreateAppWindow( rtWidth, rtHeight );
	Init();
	while (!windowClosed)
		{
		PumpEvents();
		if (windowClosed) break;
		if (windowResized) windowResized = false, Resize();
		Render(); // Render the next frame
	}
	vkDeviceWaitIdle( device );
	return 0;
}