// Template, 2024 IGAD Edition
// Get the latest version from: https://github.com/jbikker/tmpl8
// IGAD/NHTV/BUAS/UU - Jacco Bikker - 2006-2024

// A precompiled header speeds up compilation by precompiling stable code.
// The content in this file is source code that you will likely not change
// for your project. More info:
// www.codeproject.com/Articles/1188975/How-to-Optimize-Compilation-Times-with-Precompil

// common C++ headers
#include <chrono>				// timing: struct Timer depends on this
#include <fstream>				// file i/o
#include <vector>				// standard template library std::vector
#include <list>					// standard template library std::list
#include <algorithm>			// standard algorithms for stl containers
#include <string>				// strings
// #include <thread>			// currently unused; enable to use Windows threads.
#include <math.h>				// c standard math library
#include <assert.h>				// runtime assertions

// header for AVX, and every technology before it.
// if your CPU does not support this (unlikely), include the appropriate header instead.
// see: https://stackoverflow.com/a/11228864/2844473
#include <immintrin.h>

// shorthand for basic types
typedef unsigned char uchar;
typedef unsigned int uint;
typedef unsigned short ushort;
typedef unsigned short half;

// "leak" common namespaces to all compilation units. This is not standard // C++ practice
// but a deliberate simplification for template projects. Feel free to remove this if it
// offends you.
using namespace std;

// low-level: aligned memory allocations
#ifdef _MSC_VER
#define ALIGN( x ) __declspec( align( x ) )
#define MALLOC64( x ) ( ( x ) == 0 ? 0 : _aligned_malloc( ( x ), 64 ) )
#define FREE64( x ) _aligned_free( x )
#else
#define ALIGN( x ) __attribute__( ( aligned( x ) ) )
#define MALLOC64( x ) ( ( x ) == 0 ? 0 : aligned_alloc( 64, ( x ) ) )
#define FREE64( x ) free( x )
#endif
#if defined(__GNUC__) && (__GNUC__ >= 4)
#define CHECK_RESULT __attribute__ ((warn_unused_result))
#elif defined(_MSC_VER) && (_MSC_VER >= 1700)
#define CHECK_RESULT _Check_return_
#else
#define CHECK_RESULT
#endif

// math classes
#include "tmpl8math.h"

// template headers
#include "surface.h"
#include "sprite.h"

// namespaces
using namespace Tmpl8;

// clang-format off

// windows.h: disable a few things to speed up compilation.
#define NOMINMAX
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#define VC_EXTRALEAN
#endif
#define NOGDICAPMASKS
#define NOWINMESSAGES
#define NOWINSTYLES
#define NOSYSMETRICS
#define NOMENUS
#define NOICONS
#define NOKEYSTATES
#define NOSYSCOMMANDS
#define NORASTEROPS
#define NOSHOWWINDOW
#define OEMRESOURCE
#define NOATOM
#define NOCLIPBOARD
#define NOCOLOR
#define NOCTLMGR
#define NODRAWTEXT
#define NOKERNEL
// #define NONLS <== causes issues with tinygltf
#define NOMEMMGR
#define NOMETAFILE
#define NOMINMAX
#define NOMSG
#define NOOPENFILE
#define NOSCROLL
#define NOSERVICE
#define NOSOUND
#define NOTEXTMETRIC
#define NOWH
#define NOWINOFFSETS
#define NOCOMM
#define NOKANJI
#define NOHELP
#define NOPROFILER
#define NODEFERWINDOWPOS
#define NOMCX
#define NOIME
#include "windows.h"

// cross-platform directory access
#ifdef _MSC_VER
#include <direct.h>
#define getcwd _getcwd
#define chdir _chdir
#else
#include <unistd.h>
#endif

// GLFW
#define GLFW_USE_CHDIR 0
#define GLFW_EXPOSE_NATIVE_WIN32
#define GLFW_EXPOSE_NATIVE_WGL
#include <glad.h>
#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>

// zlib
#include "zlib.h"

// opengl
#include "opengl.h"

// fatal error reporting (with a pretty window)
#define FATALERROR( fmt, ... ) FatalError( "Error on line %d of %s: " fmt "\n", __LINE__, __FILE__, ##__VA_ARGS__ )
#define FATALERROR_IF( condition, fmt, ... ) do { if ( ( condition ) ) FATALERROR( fmt, ##__VA_ARGS__ ); } while ( 0 )
#define FATALERROR_IN( prefix, errstr, fmt, ... ) FatalError( prefix " returned error '%s' at %s:%d" fmt "\n", errstr, __FILE__, __LINE__, ##__VA_ARGS__ );
#define FATALERROR_IN_CALL( stmt, error_parser, fmt, ... ) do { auto ret = ( stmt ); if ( ret ) FATALERROR_IN( #stmt, error_parser( ret ), fmt, ##__VA_ARGS__ ) } while ( 0 )

// timer
struct Timer
{
	Timer() { reset(); }
	float elapsed() const
	{
		chrono::high_resolution_clock::time_point t2 = chrono::high_resolution_clock::now();
		chrono::duration<double> time_span = chrono::duration_cast<chrono::duration<double>>(t2 - start);
		return (float)time_span.count();
	}
	void reset() { start = chrono::high_resolution_clock::now(); }
	chrono::high_resolution_clock::time_point start;
};

// Nils's jobmanager
class Job
{
public:
	virtual void Main() = 0;
protected:
	friend class JobThread;
	void RunCodeWrapper();
};
class JobThread
{
public:
	void CreateAndStartThread( unsigned int threadId );
	void Go();
	void BackgroundTask();
	HANDLE m_GoSignal, m_ThreadHandle;
	int m_ThreadID;
};
class JobManager	// singleton class!
{
protected:
	JobManager( unsigned int numThreads );
public:
	~JobManager();
	static void CreateJobManager( unsigned int numThreads );
	static JobManager* GetJobManager();
	static void GetProcessorCount( uint& cores, uint& logical );
	void AddJob2( Job* a_Job );
	unsigned int GetNumThreads() { return m_NumThreads; }
	void RunJobs();
	void ThreadDone( unsigned int n );
	int MaxConcurrent() { return m_NumThreads; }
protected:
	friend class JobThread;
	Job* GetNextJob();
	static JobManager* m_JobManager;
	Job* m_JobList[256];
	CRITICAL_SECTION m_CS;
	HANDLE m_ThreadDone[64];
	unsigned int m_NumThreads, m_JobCount;
	JobThread* m_JobThreadList;
};

// forward declaration of helper functions
void FatalError( const char* fmt, ... );
bool FileIsNewer( const char* file1, const char* file2 );
bool FileExists( const char* f );
bool RemoveFile( const char* f );
string TextFileRead( const char* _File );
int LineCount( const string s );
void TextFileWrite( const string& text, const char* _File );

// global project settigs; shared with OpenCL.
// If you change these a lot, consider moving the include out of precomp.h.
#include "common.h"

// low-level: instruction set detection
#ifdef _WIN32
#define cpuid(info, x) __cpuidex(info, x, 0)
#else
#include <cpuid.h>
void cpuid( int info[4], int InfoType ) { __cpuid_count( InfoType, 0, info[0], info[1], info[2], info[3] ); }
#endif
class CPUCaps // from https://github.com/Mysticial/FeatureDetector
{
public:
	static inline bool HW_MMX = false, HW_x64 = false, HW_ABM = false, HW_RDRAND = false;
	static inline bool HW_BMI1 = false, HW_BMI2 = false, HW_ADX = false, HW_PREFETCHWT1 = false;
	// SIMD: 128-bit
	static inline bool HW_SSE = false, HW_SSE2 = false, HW_SSE3 = false, HW_SSSE3 = false;
	static inline bool HW_SSE41 = false, HW_SSE42 = false, HW_SSE4a = false;
	static inline bool HW_AES = false, HW_SHA = false;
	// SIMD: 256-bit
	static inline bool HW_AVX = false, HW_XOP = false, HW_FMA3 = false, HW_FMA4 = false;
	static inline bool HW_AVX2 = false;
	// SIMD: 512-bit
	static inline bool HW_AVX512F = false;    //  AVX512 Foundation
	static inline bool HW_AVX512CD = false;   //  AVX512 Conflict Detection
	static inline bool HW_AVX512PF = false;   //  AVX512 Prefetch
	static inline bool HW_AVX512ER = false;   //  AVX512 Exponential + Reciprocal
	static inline bool HW_AVX512VL = false;   //  AVX512 Vector Length Extensions
	static inline bool HW_AVX512BW = false;   //  AVX512 Byte + Word
	static inline bool HW_AVX512DQ = false;   //  AVX512 Doubleword + Quadword
	static inline bool HW_AVX512IFMA = false; //  AVX512 Integer 52-bit Fused Multiply-Add
	static inline bool HW_AVX512VBMI = false; //  AVX512 Vector Byte Manipulation Instructions
	// constructor
	CPUCaps()
	{
		int info[4];
		cpuid( info, 0 );
		int nIds = info[0];
		cpuid( info, 0x80000000 );
		unsigned nExIds = info[0];
		// detect cpu features
		if (nIds >= 0x00000001)
		{
			cpuid( info, 0x00000001 );
			HW_MMX = (info[3] & ((int)1 << 23)) != 0;
			HW_SSE = (info[3] & ((int)1 << 25)) != 0;
			HW_SSE2 = (info[3] & ((int)1 << 26)) != 0;
			HW_SSE3 = (info[2] & ((int)1 << 0)) != 0;
			HW_SSSE3 = (info[2] & ((int)1 << 9)) != 0;
			HW_SSE41 = (info[2] & ((int)1 << 19)) != 0;
			HW_SSE42 = (info[2] & ((int)1 << 20)) != 0;
			HW_AES = (info[2] & ((int)1 << 25)) != 0;
			HW_AVX = (info[2] & ((int)1 << 28)) != 0;
			HW_FMA3 = (info[2] & ((int)1 << 12)) != 0;
			HW_RDRAND = (info[2] & ((int)1 << 30)) != 0;
		}
		if (nIds >= 0x00000007)
		{
			cpuid( info, 0x00000007 );
			HW_AVX2 = (info[1] & ((int)1 << 5)) != 0;
			HW_BMI1 = (info[1] & ((int)1 << 3)) != 0;
			HW_BMI2 = (info[1] & ((int)1 << 8)) != 0;
			HW_ADX = (info[1] & ((int)1 << 19)) != 0;
			HW_SHA = (info[1] & ((int)1 << 29)) != 0;
			HW_PREFETCHWT1 = (info[2] & ((int)1 << 0)) != 0;
			HW_AVX512F = (info[1] & ((int)1 << 16)) != 0;
			HW_AVX512CD = (info[1] & ((int)1 << 28)) != 0;
			HW_AVX512PF = (info[1] & ((int)1 << 26)) != 0;
			HW_AVX512ER = (info[1] & ((int)1 << 27)) != 0;
			HW_AVX512VL = (info[1] & ((int)1 << 31)) != 0;
			HW_AVX512BW = (info[1] & ((int)1 << 30)) != 0;
			HW_AVX512DQ = (info[1] & ((int)1 << 17)) != 0;
			HW_AVX512IFMA = (info[1] & ((int)1 << 21)) != 0;
			HW_AVX512VBMI = (info[2] & ((int)1 << 1)) != 0;
		}
		if (nExIds >= 0x80000001)
		{
			cpuid( info, 0x80000001 );
			HW_x64 = (info[3] & ((int)1 << 29)) != 0;
			HW_ABM = (info[2] & ((int)1 << 5)) != 0;
			HW_SSE4a = (info[2] & ((int)1 << 6)) != 0;
			HW_FMA4 = (info[2] & ((int)1 << 16)) != 0;
			HW_XOP = (info[2] & ((int)1 << 11)) != 0;
		}
	}
};

// application base class
class TheApp
{
public:
	virtual void Init() = 0;
	virtual void Tick( float deltaTime ) = 0;
	virtual void Shutdown() = 0;
	virtual void MouseUp( int button ) = 0;
	virtual void MouseDown( int button ) = 0;
	virtual void MouseMove( int x, int y ) = 0;
	virtual void MouseWheel( float y ) = 0;
	virtual void KeyUp( int key ) = 0;
	virtual void KeyDown( int key ) = 0;
	Surface* screen = 0;
};

// use template vector types in tinybvh
#define TINYBVH_USE_CUSTOM_VECTOR_TYPES
#define NO_DOUBLE_PRECISION_SUPPORT
namespace tinybvh
{
using bvhint2 = int2;
using bvhint3 = int3;
using bvhuint2 = uint2;
using bvhuint3 = uint3;
using bvhuint4 = uint4;
using bvhvec2 = float2;
using bvhvec3 = float3;
using bvhvec4 = float4;
using bvhmat4 = mat4;
}
#include "tiny_ocl.h"
#include "tiny_bvh.h"

#define TINYSCENE_USE_CUSTOM_VECTOR_TYPES
namespace tinyscene
{
using ts_int2 = int2;
using ts_int3 = int3;
using ts_uint2 = uint2;
using ts_uint3 = uint3;
using ts_uint4 = uint4;
using ts_vec2 = float2;
using ts_vec3 = float3;
using ts_vec4 = float4;
using ts_mat4 = mat4;
}
#define TINYSCENE_STBIMAGE_ALREADY_IMPLEMENTED
#include "tiny_scene.h"

// EOF