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

// opencl
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

// lighthouse 2 object change tracking system
// USAGE: Add "TRACKCHANGES;" to the end of an object to add some new methods:
// bool Changed(): returns true if object is 'dirty' and resets dirty to false.
// bool IsDirty(): returns true if object is 'dirty'.
// void MarkAsDirty(): marks the object as 'dirty'.
// void MarkAsNotDirty(): marks the object as 'not dirty'.
// Use these methods to keep e.g. CPU and GPU copies of the data in sync, to
// render changed sprites, or to save only changed objects to disk.
#define UINT64C(x) ((uint64_t) x##ULL)
#define CLEARCRC64 (UINT64C( 0xffffffffffffffff ))
const uint64_t crc64_table[256] = {
	UINT64C( 0x0000000000000000 ), UINT64C( 0x42F0E1EBA9EA3693 ), UINT64C( 0x85E1C3D753D46D26 ), UINT64C( 0xC711223CFA3E5BB5 ),
	UINT64C( 0x493366450E42ECDF ), UINT64C( 0x0BC387AEA7A8DA4C ), UINT64C( 0xCCD2A5925D9681F9 ), UINT64C( 0x8E224479F47CB76A ),
	UINT64C( 0x9266CC8A1C85D9BE ), UINT64C( 0xD0962D61B56FEF2D ), UINT64C( 0x17870F5D4F51B498 ), UINT64C( 0x5577EEB6E6BB820B ),
	UINT64C( 0xDB55AACF12C73561 ), UINT64C( 0x99A54B24BB2D03F2 ), UINT64C( 0x5EB4691841135847 ), UINT64C( 0x1C4488F3E8F96ED4 ),
	UINT64C( 0x663D78FF90E185EF ), UINT64C( 0x24CD9914390BB37C ), UINT64C( 0xE3DCBB28C335E8C9 ), UINT64C( 0xA12C5AC36ADFDE5A ),
	UINT64C( 0x2F0E1EBA9EA36930 ), UINT64C( 0x6DFEFF5137495FA3 ), UINT64C( 0xAAEFDD6DCD770416 ), UINT64C( 0xE81F3C86649D3285 ),
	UINT64C( 0xF45BB4758C645C51 ), UINT64C( 0xB6AB559E258E6AC2 ), UINT64C( 0x71BA77A2DFB03177 ), UINT64C( 0x334A9649765A07E4 ),
	UINT64C( 0xBD68D2308226B08E ), UINT64C( 0xFF9833DB2BCC861D ), UINT64C( 0x388911E7D1F2DDA8 ), UINT64C( 0x7A79F00C7818EB3B ),
	UINT64C( 0xCC7AF1FF21C30BDE ), UINT64C( 0x8E8A101488293D4D ), UINT64C( 0x499B3228721766F8 ), UINT64C( 0x0B6BD3C3DBFD506B ),
	UINT64C( 0x854997BA2F81E701 ), UINT64C( 0xC7B97651866BD192 ), UINT64C( 0x00A8546D7C558A27 ), UINT64C( 0x4258B586D5BFBCB4 ),
	UINT64C( 0x5E1C3D753D46D260 ), UINT64C( 0x1CECDC9E94ACE4F3 ), UINT64C( 0xDBFDFEA26E92BF46 ), UINT64C( 0x990D1F49C77889D5 ),
	UINT64C( 0x172F5B3033043EBF ), UINT64C( 0x55DFBADB9AEE082C ), UINT64C( 0x92CE98E760D05399 ), UINT64C( 0xD03E790CC93A650A ),
	UINT64C( 0xAA478900B1228E31 ), UINT64C( 0xE8B768EB18C8B8A2 ), UINT64C( 0x2FA64AD7E2F6E317 ), UINT64C( 0x6D56AB3C4B1CD584 ),
	UINT64C( 0xE374EF45BF6062EE ), UINT64C( 0xA1840EAE168A547D ), UINT64C( 0x66952C92ECB40FC8 ), UINT64C( 0x2465CD79455E395B ),
	UINT64C( 0x3821458AADA7578F ), UINT64C( 0x7AD1A461044D611C ), UINT64C( 0xBDC0865DFE733AA9 ), UINT64C( 0xFF3067B657990C3A ),
	UINT64C( 0x711223CFA3E5BB50 ), UINT64C( 0x33E2C2240A0F8DC3 ), UINT64C( 0xF4F3E018F031D676 ), UINT64C( 0xB60301F359DBE0E5 ),
	UINT64C( 0xDA050215EA6C212F ), UINT64C( 0x98F5E3FE438617BC ), UINT64C( 0x5FE4C1C2B9B84C09 ), UINT64C( 0x1D14202910527A9A ),
	UINT64C( 0x93366450E42ECDF0 ), UINT64C( 0xD1C685BB4DC4FB63 ), UINT64C( 0x16D7A787B7FAA0D6 ), UINT64C( 0x5427466C1E109645 ),
	UINT64C( 0x4863CE9FF6E9F891 ), UINT64C( 0x0A932F745F03CE02 ), UINT64C( 0xCD820D48A53D95B7 ), UINT64C( 0x8F72ECA30CD7A324 ),
	UINT64C( 0x0150A8DAF8AB144E ), UINT64C( 0x43A04931514122DD ), UINT64C( 0x84B16B0DAB7F7968 ), UINT64C( 0xC6418AE602954FFB ),
	UINT64C( 0xBC387AEA7A8DA4C0 ), UINT64C( 0xFEC89B01D3679253 ), UINT64C( 0x39D9B93D2959C9E6 ), UINT64C( 0x7B2958D680B3FF75 ),
	UINT64C( 0xF50B1CAF74CF481F ), UINT64C( 0xB7FBFD44DD257E8C ), UINT64C( 0x70EADF78271B2539 ), UINT64C( 0x321A3E938EF113AA ),
	UINT64C( 0x2E5EB66066087D7E ), UINT64C( 0x6CAE578BCFE24BED ), UINT64C( 0xABBF75B735DC1058 ), UINT64C( 0xE94F945C9C3626CB ),
	UINT64C( 0x676DD025684A91A1 ), UINT64C( 0x259D31CEC1A0A732 ), UINT64C( 0xE28C13F23B9EFC87 ), UINT64C( 0xA07CF2199274CA14 ),
	UINT64C( 0x167FF3EACBAF2AF1 ), UINT64C( 0x548F120162451C62 ), UINT64C( 0x939E303D987B47D7 ), UINT64C( 0xD16ED1D631917144 ),
	UINT64C( 0x5F4C95AFC5EDC62E ), UINT64C( 0x1DBC74446C07F0BD ), UINT64C( 0xDAAD56789639AB08 ), UINT64C( 0x985DB7933FD39D9B ),
	UINT64C( 0x84193F60D72AF34F ), UINT64C( 0xC6E9DE8B7EC0C5DC ), UINT64C( 0x01F8FCB784FE9E69 ), UINT64C( 0x43081D5C2D14A8FA ),
	UINT64C( 0xCD2A5925D9681F90 ), UINT64C( 0x8FDAB8CE70822903 ), UINT64C( 0x48CB9AF28ABC72B6 ), UINT64C( 0x0A3B7B1923564425 ),
	UINT64C( 0x70428B155B4EAF1E ), UINT64C( 0x32B26AFEF2A4998D ), UINT64C( 0xF5A348C2089AC238 ), UINT64C( 0xB753A929A170F4AB ),
	UINT64C( 0x3971ED50550C43C1 ), UINT64C( 0x7B810CBBFCE67552 ), UINT64C( 0xBC902E8706D82EE7 ), UINT64C( 0xFE60CF6CAF321874 ),
	UINT64C( 0xE224479F47CB76A0 ), UINT64C( 0xA0D4A674EE214033 ), UINT64C( 0x67C58448141F1B86 ), UINT64C( 0x253565A3BDF52D15 ),
	UINT64C( 0xAB1721DA49899A7F ), UINT64C( 0xE9E7C031E063ACEC ), UINT64C( 0x2EF6E20D1A5DF759 ), UINT64C( 0x6C0603E6B3B7C1CA ),
	UINT64C( 0xF6FAE5C07D3274CD ), UINT64C( 0xB40A042BD4D8425E ), UINT64C( 0x731B26172EE619EB ), UINT64C( 0x31EBC7FC870C2F78 ),
	UINT64C( 0xBFC9838573709812 ), UINT64C( 0xFD39626EDA9AAE81 ), UINT64C( 0x3A28405220A4F534 ), UINT64C( 0x78D8A1B9894EC3A7 ),
	UINT64C( 0x649C294A61B7AD73 ), UINT64C( 0x266CC8A1C85D9BE0 ), UINT64C( 0xE17DEA9D3263C055 ), UINT64C( 0xA38D0B769B89F6C6 ),
	UINT64C( 0x2DAF4F0F6FF541AC ), UINT64C( 0x6F5FAEE4C61F773F ), UINT64C( 0xA84E8CD83C212C8A ), UINT64C( 0xEABE6D3395CB1A19 ),
	UINT64C( 0x90C79D3FEDD3F122 ), UINT64C( 0xD2377CD44439C7B1 ), UINT64C( 0x15265EE8BE079C04 ), UINT64C( 0x57D6BF0317EDAA97 ),
	UINT64C( 0xD9F4FB7AE3911DFD ), UINT64C( 0x9B041A914A7B2B6E ), UINT64C( 0x5C1538ADB04570DB ), UINT64C( 0x1EE5D94619AF4648 ),
	UINT64C( 0x02A151B5F156289C ), UINT64C( 0x4051B05E58BC1E0F ), UINT64C( 0x87409262A28245BA ), UINT64C( 0xC5B073890B687329 ),
	UINT64C( 0x4B9237F0FF14C443 ), UINT64C( 0x0962D61B56FEF2D0 ), UINT64C( 0xCE73F427ACC0A965 ), UINT64C( 0x8C8315CC052A9FF6 ),
	UINT64C( 0x3A80143F5CF17F13 ), UINT64C( 0x7870F5D4F51B4980 ), UINT64C( 0xBF61D7E80F251235 ), UINT64C( 0xFD913603A6CF24A6 ),
	UINT64C( 0x73B3727A52B393CC ), UINT64C( 0x31439391FB59A55F ), UINT64C( 0xF652B1AD0167FEEA ), UINT64C( 0xB4A25046A88DC879 ),
	UINT64C( 0xA8E6D8B54074A6AD ), UINT64C( 0xEA16395EE99E903E ), UINT64C( 0x2D071B6213A0CB8B ), UINT64C( 0x6FF7FA89BA4AFD18 ),
	UINT64C( 0xE1D5BEF04E364A72 ), UINT64C( 0xA3255F1BE7DC7CE1 ), UINT64C( 0x64347D271DE22754 ), UINT64C( 0x26C49CCCB40811C7 ),
	UINT64C( 0x5CBD6CC0CC10FAFC ), UINT64C( 0x1E4D8D2B65FACC6F ), UINT64C( 0xD95CAF179FC497DA ), UINT64C( 0x9BAC4EFC362EA149 ),
	UINT64C( 0x158E0A85C2521623 ), UINT64C( 0x577EEB6E6BB820B0 ), UINT64C( 0x906FC95291867B05 ), UINT64C( 0xD29F28B9386C4D96 ),
	UINT64C( 0xCEDBA04AD0952342 ), UINT64C( 0x8C2B41A1797F15D1 ), UINT64C( 0x4B3A639D83414E64 ), UINT64C( 0x09CA82762AAB78F7 ),
	UINT64C( 0x87E8C60FDED7CF9D ), UINT64C( 0xC51827E4773DF90E ), UINT64C( 0x020905D88D03A2BB ), UINT64C( 0x40F9E43324E99428 ),
	UINT64C( 0x2CFFE7D5975E55E2 ), UINT64C( 0x6E0F063E3EB46371 ), UINT64C( 0xA91E2402C48A38C4 ), UINT64C( 0xEBEEC5E96D600E57 ),
	UINT64C( 0x65CC8190991CB93D ), UINT64C( 0x273C607B30F68FAE ), UINT64C( 0xE02D4247CAC8D41B ), UINT64C( 0xA2DDA3AC6322E288 ),
	UINT64C( 0xBE992B5F8BDB8C5C ), UINT64C( 0xFC69CAB42231BACF ), UINT64C( 0x3B78E888D80FE17A ), UINT64C( 0x7988096371E5D7E9 ),
	UINT64C( 0xF7AA4D1A85996083 ), UINT64C( 0xB55AACF12C735610 ), UINT64C( 0x724B8ECDD64D0DA5 ), UINT64C( 0x30BB6F267FA73B36 ),
	UINT64C( 0x4AC29F2A07BFD00D ), UINT64C( 0x08327EC1AE55E69E ), UINT64C( 0xCF235CFD546BBD2B ), UINT64C( 0x8DD3BD16FD818BB8 ),
	UINT64C( 0x03F1F96F09FD3CD2 ), UINT64C( 0x41011884A0170A41 ), UINT64C( 0x86103AB85A2951F4 ), UINT64C( 0xC4E0DB53F3C36767 ),
	UINT64C( 0xD8A453A01B3A09B3 ), UINT64C( 0x9A54B24BB2D03F20 ), UINT64C( 0x5D45907748EE6495 ), UINT64C( 0x1FB5719CE1045206 ),
	UINT64C( 0x919735E51578E56C ), UINT64C( 0xD367D40EBC92D3FF ), UINT64C( 0x1476F63246AC884A ), UINT64C( 0x568617D9EF46BED9 ),
	UINT64C( 0xE085162AB69D5E3C ), UINT64C( 0xA275F7C11F7768AF ), UINT64C( 0x6564D5FDE549331A ), UINT64C( 0x279434164CA30589 ),
	UINT64C( 0xA9B6706FB8DFB2E3 ), UINT64C( 0xEB46918411358470 ), UINT64C( 0x2C57B3B8EB0BDFC5 ), UINT64C( 0x6EA7525342E1E956 ),
	UINT64C( 0x72E3DAA0AA188782 ), UINT64C( 0x30133B4B03F2B111 ), UINT64C( 0xF7021977F9CCEAA4 ), UINT64C( 0xB5F2F89C5026DC37 ),
	UINT64C( 0x3BD0BCE5A45A6B5D ), UINT64C( 0x79205D0E0DB05DCE ), UINT64C( 0xBE317F32F78E067B ), UINT64C( 0xFCC19ED95E6430E8 ),
	UINT64C( 0x86B86ED5267CDBD3 ), UINT64C( 0xC4488F3E8F96ED40 ), UINT64C( 0x0359AD0275A8B6F5 ), UINT64C( 0x41A94CE9DC428066 ),
	UINT64C( 0xCF8B0890283E370C ), UINT64C( 0x8D7BE97B81D4019F ), UINT64C( 0x4A6ACB477BEA5A2A ), UINT64C( 0x089A2AACD2006CB9 ),
	UINT64C( 0x14DEA25F3AF9026D ), UINT64C( 0x562E43B4931334FE ), UINT64C( 0x913F6188692D6F4B ), UINT64C( 0xD3CF8063C0C759D8 ),
	UINT64C( 0x5DEDC41A34BBEEB2 ), UINT64C( 0x1F1D25F19D51D821 ), UINT64C( 0xD80C07CD676F8394 ), UINT64C( 0x9AFCE626CE85B507 )
};
__inline uint64_t calccrc64( unsigned char* pbData, int len ) // crc64, from https://sourceforge.net/projects/crc64/
{
	uint64_t crc = CLEARCRC64;
	unsigned char* p = pbData;
	unsigned int t, l = len;
	while (l-- > 0)
		t = ((uint)(crc >> 56) ^ *p++) & 255,
		crc = crc64_table[t] ^ (crc << 8);
	return crc ^ CLEARCRC64;
}
#define TRACKCHANGES public: bool Changed() { uint64_t currentcrc = __crc64; \
__crc64 = CLEARCRC64; uint64_t newcrc = calccrc64( (uchar*)this, (int)(__int64( &__dirty ) + 4 - __int64( this )) ); \
bool changed = newcrc != currentcrc; __crc64 = newcrc; return changed; } \
bool IsDirty() { uint64_t t = __crc64; bool c = Changed(); __crc64 = t; return c; } \
void MarkAsDirty() { __dirty++; } \
void MarkAsNotDirty() { Changed(); } \
private: uint64_t __crc64 = CLEARCRC64; uint __dirty = 0; public:

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

// EOF