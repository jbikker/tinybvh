#include "headers.h"

#ifdef ENABLE_OPENCL
#define TINY_OCL_IMPLEMENTATION
#include "tiny_ocl.h"
tinyocl::Kernel* kernel_nearest = 0;
tinyocl::Kernel* kernel_any = 0;
tinyocl::Kernel* gpu4way_kernel = 0;
tinyocl::Kernel* gpu4way_kernel_any = 0;
tinyocl::Kernel* cwbvh_kernel = 0;
tinyocl::Kernel* cwbvh_kernel_any = 0;
#endif

FILE* csv = 0;

void PrintHeader()
{
	// open csv file for appending
	csv = fopen( "benchmark_results.csv", "a" );

	// report TinyBVH version
	int major = TINY_BVH_VERSION_MAJOR;
	int minor = TINY_BVH_VERSION_MINOR;
	int sub = TINY_BVH_VERSION_SUB;
	printf( "TINY_BVH BENCHMARK TOOL\n" );
	printf( "library version: %i.%i.%i performance statistics ", major, minor, sub );
	if (csv) fprintf( csv, "\ntinybvh,v %i.%i.%i\n", major, minor, sub );
	std::time_t time = std::time({});
	char buffer[128];
    std::strftime( std::data( buffer ), 128, "time:,%T,%F", std::gmtime( &time ) );
	if (csv) fprintf( csv, "%s\n", buffer );
	if (csv) fflush( csv );

	// determine compiler
#ifdef _MSC_VER
	printf( "(MSVC %i build)\n", _MSC_VER );
	if (csv) fprintf( csv, "compiler:,msc,v %i\n", _MSC_VER );
#elif defined __EMSCRIPTEN__
	// EMSCRIPTEN needs to be before clang or gcc
	printf( "(emcc %i.%i build)\n", __EMSCRIPTEN_major__, __EMSCRIPTEN_minor__ );
	if (csv) fprintf( csv, "compiler:,emcc,v %i.%i\n", __EMSCRIPTEN_major__, __EMSCRIPTEN_minor__ );
#elif defined __clang__
	printf( "(clang %i.%i build)\n", __clang_major__, __clang_minor__ );
	if (csv) fprintf( csv, "compiler:,clang,v %i.%i\n", __clang_major__, __clang_minor__ );
#elif defined __GNUC__
	printf( "(gcc %i.%i build)\n", __GNUC__, __GNUC_MINOR__ );
	if (csv) fprintf( csv, "compiler:,gcc,v %i.%i\n", __GNUC__, __GNUC_MINOR__ );
#else
	printf( "\n" );
#endif

	// determine what CPU is running the tests.
#if (defined(__x86_64__) || defined(_M_X64)) && (defined (_WIN32) || defined(__GNUC__))
	char model[64]{};
	for (unsigned i = 0; i < 3; ++i)
	{
	#ifdef _WIN32
		__cpuidex( (int*)(model + i * 16), i + 0x80000002, 0 );
	#elif defined(__GNUC__)
		__get_cpuid( i + 0x80000002,
			(unsigned*)model + i * 4 + 0, (unsigned*)model + i * 4 + 1,
			(unsigned*)model + i * 4 + 2, (unsigned*)model + i * 4 + 3 );
	#endif
	}
	printf( "running on %s\n", model );
	if (csv) fprintf( csv, "cpu:,%s\n", model );
#endif
	printf( "----------------------------------------------------------------\n" );
	if (csv) fflush( csv );
}

void InitOpenCL()
{
#ifdef ENABLE_OPENCL
	// load and compile the OpenCL kernel code
	kernel_nearest = new tinyocl::Kernel( "kernels/traverse.cl", "batch_nearest" );
	kernel_any = new tinyocl::Kernel( "kernels/traverse.cl", "batch_any" );
	gpu4way_kernel = new tinyocl::Kernel( "kernels/traverse.cl", "batch_gpu4way" );
	gpu4way_kernel_any = new tinyocl::Kernel( "kernels/traverse.cl", "batch_gpu4way_any" );
	cwbvh_kernel = new tinyocl::Kernel( "kernels/traverse.cl", "batch_cwbvh" );
	cwbvh_kernel_any = new tinyocl::Kernel( "kernels/traverse.cl", "batch_cwbvh_any" );
	printf( "----------------------------------------------------------------\n" );
	if (csv)
	{
		if (Kernel::isAMD) fprintf( csv, "gpu:,AMD\n" );
		else if (Kernel::isIntel) fprintf( csv, "gpu:,INTEL\n" );
		else if (Kernel::isNVidia) fprintf( csv, "gpu:,NVIDIA\n" );
		else if (Kernel::isApple) fprintf( csv, "gpu:,APPLE\n" );
		else fprintf( csv, "gpu:,UNKOWN\n" );	
	}
#endif
	if (csv) fflush( csv );
}