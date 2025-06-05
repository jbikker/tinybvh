// Template, 2024 IGAD Edition
// Get the latest version from: https://github.com/jbikker/tmpl8
// IGAD/NHTV/BUAS/UU - Jacco Bikker - 2006-2024

#include "precomp.h"

using namespace std;

// source file information
static int sourceFiles = 0;
static char* sourceFile[64]; // yup, ugly constant

// access to GLFW window in template.cpp
extern GLFWwindow* window;

#define CHECKCL(r) CheckCL( r, __FILE__, __LINE__ )

void FatalError( const char* fmt, ... )
{
	char t[65536];
	va_list args;
	va_start( args, fmt );
	vsnprintf( t, sizeof( t ) - 2, fmt, args );
	va_end( args );
#ifdef _MSC_VER
	MessageBox( NULL, t, "Fatal error", MB_OK );
#else
	fprintf( stderr, t );
#endif
	while (1) exit( 0 );
}

// CHECKCL method
// OpenCL error handling.
// ----------------------------------------------------------------------------
bool CheckCL( cl_int result, const char* file, int line )
{
	if (result == CL_SUCCESS) return true;
	if (result == CL_DEVICE_NOT_FOUND) FatalError( "Error: CL_DEVICE_NOT_FOUND\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_DEVICE_NOT_AVAILABLE) FatalError( "Error: CL_DEVICE_NOT_AVAILABLE\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_COMPILER_NOT_AVAILABLE) FatalError( "Error: CL_COMPILER_NOT_AVAILABLE\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_MEM_OBJECT_ALLOCATION_FAILURE) FatalError( "Error: CL_MEM_OBJECT_ALLOCATION_FAILURE\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_OUT_OF_RESOURCES) FatalError( "Error: CL_OUT_OF_RESOURCES\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_OUT_OF_HOST_MEMORY) FatalError( "Error: CL_OUT_OF_HOST_MEMORY\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_PROFILING_INFO_NOT_AVAILABLE) FatalError( "Error: CL_PROFILING_INFO_NOT_AVAILABLE\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_MEM_COPY_OVERLAP) FatalError( "Error: CL_MEM_COPY_OVERLAP\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_IMAGE_FORMAT_MISMATCH) FatalError( "Error: CL_IMAGE_FORMAT_MISMATCH\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_IMAGE_FORMAT_NOT_SUPPORTED) FatalError( "Error: CL_IMAGE_FORMAT_NOT_SUPPORTED\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_BUILD_PROGRAM_FAILURE) FatalError( "Error: CL_BUILD_PROGRAM_FAILURE\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_MAP_FAILURE) FatalError( "Error: CL_MAP_FAILURE\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_MISALIGNED_SUB_BUFFER_OFFSET) FatalError( "Error: CL_MISALIGNED_SUB_BUFFER_OFFSET\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_EXEC_STATUS_ERROR_FOR_EVENTS_IN_WAIT_LIST) FatalError( "Error: CL_EXEC_STATUS_ERROR_FOR_EVENTS_IN_WAIT_LIST\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_VALUE) FatalError( "Error: CL_INVALID_VALUE\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_DEVICE_TYPE) FatalError( "Error: CL_INVALID_DEVICE_TYPE\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_PLATFORM) FatalError( "Error: CL_INVALID_PLATFORM\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_DEVICE) FatalError( "Error: CL_INVALID_DEVICE\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_CONTEXT) FatalError( "Error: CL_INVALID_CONTEXT\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_QUEUE_PROPERTIES) FatalError( "Error: CL_INVALID_QUEUE_PROPERTIES\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_COMMAND_QUEUE) FatalError( "Error: CL_INVALID_COMMAND_QUEUE\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_HOST_PTR) FatalError( "Error: CL_INVALID_HOST_PTR\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_MEM_OBJECT) FatalError( "Error: CL_INVALID_MEM_OBJECT\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_IMAGE_FORMAT_DESCRIPTOR) FatalError( "Error: CL_INVALID_IMAGE_FORMAT_DESCRIPTOR\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_IMAGE_SIZE) FatalError( "Error: CL_INVALID_IMAGE_SIZE\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_SAMPLER) FatalError( "Error: CL_INVALID_SAMPLER\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_BINARY) FatalError( "Error: CL_INVALID_BINARY\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_BUILD_OPTIONS) FatalError( "Error: CL_INVALID_BUILD_OPTIONS\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_PROGRAM) FatalError( "Error: CL_INVALID_PROGRAM\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_PROGRAM_EXECUTABLE) FatalError( "Error: CL_INVALID_PROGRAM_EXECUTABLE\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_KERNEL_NAME) FatalError( "Error: CL_INVALID_KERNEL_NAME\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_KERNEL_DEFINITION) FatalError( "Error: CL_INVALID_KERNEL_DEFINITION\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_KERNEL) FatalError( "Error: CL_INVALID_KERNEL\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_ARG_INDEX) FatalError( "Error: CL_INVALID_ARG_INDEX\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_ARG_VALUE) FatalError( "Error: CL_INVALID_ARG_VALUE\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_ARG_SIZE) FatalError( "Error: CL_INVALID_ARG_SIZE\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_KERNEL_ARGS) FatalError( "Error: CL_INVALID_KERNEL_ARGS\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_WORK_DIMENSION) FatalError( "Error: CL_INVALID_WORK_DIMENSION\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_WORK_GROUP_SIZE) FatalError( "Error: CL_INVALID_WORK_GROUP_SIZE\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_WORK_ITEM_SIZE) FatalError( "Error: CL_INVALID_WORK_ITEM_SIZE\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_GLOBAL_OFFSET) FatalError( "Error: CL_INVALID_GLOBAL_OFFSET\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_EVENT_WAIT_LIST) FatalError( "Error: CL_INVALID_EVENT_WAIT_LIST\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_EVENT) FatalError( "Error: CL_INVALID_EVENT\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_OPERATION) FatalError( "Error: CL_INVALID_OPERATION\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_GL_OBJECT) FatalError( "Error: CL_INVALID_GL_OBJECT\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_BUFFER_SIZE) FatalError( "Error: CL_INVALID_BUFFER_SIZE\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_MIP_LEVEL) FatalError( "Error: CL_INVALID_MIP_LEVEL\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_GLOBAL_WORK_SIZE) FatalError( "Error: CL_INVALID_GLOBAL_WORK_SIZE\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_PROPERTY) FatalError( "Error: CL_INVALID_PROPERTY\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_IMAGE_DESCRIPTOR) FatalError( "Error: CL_INVALID_IMAGE_DESCRIPTOR\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_COMPILER_OPTIONS) FatalError( "Error: CL_INVALID_COMPILER_OPTIONS\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_LINKER_OPTIONS) FatalError( "Error: CL_INVALID_LINKER_OPTIONS\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_DEVICE_PARTITION_COUNT) FatalError( "Error: CL_INVALID_DEVICE_PARTITION_COUNT\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_PIPE_SIZE) FatalError( "Error: CL_INVALID_PIPE_SIZE\n%s, line %i", file, line, "OpenCL error" );
	if (result == CL_INVALID_DEVICE_QUEUE) FatalError( "Error: CL_INVALID_DEVICE_QUEUE\n%s, line %i", file, line, "OpenCL error" );
	return false;
}

// getFirstDevice
// ----------------------------------------------------------------------------
static cl_device_id getFirstDevice( cl_context context )
{
	size_t dataSize;
	cl_device_id* devices;
	clGetContextInfo( context, CL_CONTEXT_DEVICES, 0, NULL, &dataSize );
	devices = (cl_device_id*)malloc( dataSize );
	clGetContextInfo( context, CL_CONTEXT_DEVICES, dataSize, devices, NULL );
	cl_device_id first = devices[0];
	free( devices );
	return first;
}

// getPlatformID
// ----------------------------------------------------------------------------
static cl_int getPlatformID( cl_platform_id* platform )
{
	char chBuffer[1024];
	cl_uint num_platforms, devCount;
	cl_platform_id* clPlatformIDs;
	cl_int error;
	*platform = NULL;
	CHECKCL( error = clGetPlatformIDs( 0, NULL, &num_platforms ) );
	if (num_platforms == 0) CHECKCL( -1 );
	clPlatformIDs = (cl_platform_id*)malloc( num_platforms * sizeof( cl_platform_id ) );
	error = clGetPlatformIDs( num_platforms, clPlatformIDs, NULL );
	cl_uint deviceType[2] = { CL_DEVICE_TYPE_GPU, CL_DEVICE_TYPE_CPU };
	const char* deviceOrder[2][3] = { { "NVIDIA", "AMD", "" }, { "", "", "" } };
	printf( "available OpenCL platforms:\n" );
	for (cl_uint i = 0; i < num_platforms; ++i)
	{
		CHECKCL( error = clGetPlatformInfo( clPlatformIDs[i], CL_PLATFORM_NAME, 1024, &chBuffer, NULL ) );
		printf( "#%i: %s\n", i, chBuffer );
	}
	for (cl_uint j = 0; j < 2; j++) for (int k = 0; k < 3; k++) for (cl_uint i = 0; i < num_platforms; ++i)
	{
		error = clGetDeviceIDs( clPlatformIDs[i], deviceType[j], 0, NULL, &devCount );
		if ((error != CL_SUCCESS) || (devCount == 0)) continue;
		CHECKCL( error = clGetPlatformInfo( clPlatformIDs[i], CL_PLATFORM_NAME, 1024, &chBuffer, NULL ) );
		if (deviceOrder[j][k][0]) if (!strstr( chBuffer, deviceOrder[j][k] )) continue;
		printf( "OpenCL device: %s\n", chBuffer );
		*platform = clPlatformIDs[i], j = 2, k = 3;
		break;
	}
	free( clPlatformIDs );
	return CL_SUCCESS;
}

// Buffer constructor
// ----------------------------------------------------------------------------
Buffer::Buffer( unsigned int N, void* ptr, unsigned int t )
{
	if (!Kernel::clStarted) Kernel::InitCL();
	type = t;
	ownData = false;
	int rwFlags = CL_MEM_READ_WRITE;
	if (t & READONLY) rwFlags = CL_MEM_READ_ONLY;
	if (t & WRITEONLY) rwFlags = CL_MEM_WRITE_ONLY;
	aligned = false;
	if ((t & (TEXTURE | TARGET)) == 0)
	{
		size = N;
		textureID = 0; // not representing a texture
		if (N > 0) deviceBuffer = clCreateBuffer( Kernel::GetContext(), rwFlags, size, 0, 0 );
		hostBuffer = (uint*)ptr;
	}
	else
	{
		textureID = N; // representing texture N
		if (!Kernel::candoInterop) FatalError( "didn't expect to get here." );
		int error = 0;
		if (t == TARGET) deviceBuffer = clCreateFromGLTexture( Kernel::GetContext(), CL_MEM_WRITE_ONLY, GL_TEXTURE_2D, 0, N, &error );
		else deviceBuffer = clCreateFromGLTexture( Kernel::GetContext(), CL_MEM_READ_ONLY, GL_TEXTURE_2D, 0, N, &error );
		CHECKCL( error );
		hostBuffer = 0;
	}
}

// Buffer destructor
// ----------------------------------------------------------------------------
Buffer::~Buffer()
{
	if (size > 0)
	{
		if (ownData)
		{
			FREE64( hostBuffer );
			hostBuffer = 0;
		}
		if ((type & (TEXTURE | TARGET)) == 0) clReleaseMemObject( deviceBuffer );
	}
}

// GetHostPtr method
// ----------------------------------------------------------------------------
unsigned int* Buffer::GetHostPtr()
{
	if (size == 0) return 0;
	if (!hostBuffer)
	{
		hostBuffer = (uint*)MALLOC64( size );
		ownData = true;
		aligned = true;
	}
	return hostBuffer;
}

// CopyToDevice methods
// ----------------------------------------------------------------------------
void Buffer::CopyToDevice( const bool blocking )
{
	if (size == 0) return;
	cl_int error;
	if (!hostBuffer)
	{
		hostBuffer = (uint*)MALLOC64( size );
		ownData = true;
		aligned = true;
	}
	CHECKCL( error = clEnqueueWriteBuffer( Kernel::GetQueue(), deviceBuffer, blocking, 0, size, hostBuffer, 0, 0, 0 ) );
}
void Buffer::CopyToDevice( const int offset, const int byteCount, const bool blocking )
{
	if (size == 0) return;
	cl_int error;
	if (!hostBuffer)
	{
		hostBuffer = (uint*)MALLOC64( size );
		ownData = true;
		aligned = true;
	}
	CHECKCL( error = clEnqueueWriteBuffer( Kernel::GetQueue(), deviceBuffer, blocking, offset, byteCount, hostBuffer, 0, 0, 0 ) );
}

// CopyToDevice2 method (uses 2nd queue)
// ----------------------------------------------------------------------------
void Buffer::CopyToDevice2( const bool blocking, cl_event* eventToSet, const size_t s )
{
	if (size == 0) return;
	cl_int error;
	CHECKCL( error = clEnqueueWriteBuffer( Kernel::GetQueue2(), deviceBuffer, blocking ? CL_TRUE : CL_FALSE, 0, s == 0 ? size : s, hostBuffer, 0, 0, eventToSet ) );
}

// CopyFromDevice method
// ----------------------------------------------------------------------------
void Buffer::CopyFromDevice( const bool blocking )
{
	if (size == 0) return;
	cl_int error;
	if (!hostBuffer)
	{
		hostBuffer = (uint*)MALLOC64( size );
		ownData = true;
		aligned = true;
	}
	CHECKCL( error = clEnqueueReadBuffer( Kernel::GetQueue(), deviceBuffer, blocking, 0, size, hostBuffer, 0, 0, 0 ) );
}
void Buffer::CopyFromDevice( const int offset, const int byteCount, const bool blocking )
{
	if (size == 0) return;
	cl_int error;
	if (!hostBuffer)
	{
		hostBuffer = (uint*)MALLOC64( size );
		ownData = true;
		aligned = true;
	}
	CHECKCL( error = clEnqueueReadBuffer( Kernel::GetQueue(), deviceBuffer, blocking, offset, byteCount, hostBuffer, 0, 0, 0 ) );
}

// CopyTo
// ----------------------------------------------------------------------------
void Buffer::CopyTo( Buffer* buffer )
{
	if (size > 0) clEnqueueCopyBuffer( Kernel::GetQueue(), deviceBuffer, buffer->deviceBuffer, 0, 0, size, 0, 0, 0 );
}

// Clear
// ----------------------------------------------------------------------------
void Buffer::Clear()
{
	if (size == 0) return;
	uint value = 0;
	cl_int error;
	CHECKCL( error = clEnqueueFillBuffer( Kernel::GetQueue(), deviceBuffer, &value, 4, 0, size, 0, 0, 0 ) );
}

// Kernel constructor
// ----------------------------------------------------------------------------
Kernel::Kernel( char* file, char* entryPoint )
{
	if (!clStarted) InitCL();
	// see if we have seen this source file before
	for (int s = (int)loadedKernels.size(), i = 0; i < s; i++)
	{
		if (!_stricmp( file, loadedKernels[i]->sourceFile ))
		{
			cl_int error;
			program = loadedKernels[i]->program;
			kernel = clCreateKernel( program, entryPoint, &error );
			CHECKCL( error );
			return;
		}
	}
	// backup working folder
	char dirBackup[2048];
	getcwd( dirBackup, 2047 );
	// change directory
	char* dir = new char[strlen( file ) + 1], * lastSlash, * fileName = dir;
	strcpy( dir, file );
	lastSlash = strstr( dir, "/" );
	if (!lastSlash) lastSlash = strstr( dir, "\\" );
	while (lastSlash)
	{
		char* nextSlash = strstr( lastSlash + 1, "/" );
		if (!nextSlash) nextSlash = strstr( lastSlash + 1, "\\" );
		if (!nextSlash) break;
		lastSlash = nextSlash;
	}
	if (lastSlash)
	{
		*lastSlash = 0;
		fileName = lastSlash + 1;
		chdir( dir );
	}
	// load a cl file
	sourceFile = new char[strlen( file ) + 1];
	strcpy( sourceFile, file );
	string csText = TextFileRead( fileName );
	if (csText.size() == 0) FatalError( "File %s not found", file );
	// add vendor defines
	vendorLines = 0;
	if (isNVidia) csText = "#define ISNVIDIA\n" + csText, vendorLines++;
	if (isAMD) csText = "#define ISAMD\n" + csText, vendorLines++;
	if (isIntel) csText = "#define ISINTEL\n" + csText, vendorLines++;
	if (isOther) csText = "#define ISOTHER\n" + csText, vendorLines++;
	if (isAmpere) csText = "#define ISAMPERE\n" + csText, vendorLines++;
	if (isTuring) csText = "#define ISTURING\n" + csText, vendorLines++;
	if (isPascal) csText = "#define ISPASCAL\n" + csText, vendorLines++;
	if (isAda) csText = "#define ISADA\n" + csText, vendorLines++;
	// expand #include directives: cl compiler doesn't support these natively
	// warning: this simple system does not handle nested includes.
	struct Include { int start, end; string file; } includes[64];
	int Ninc = 0;
#if 1 // should not be needed, but AMD seems to require it anyway...
	if (isAMD) while (1)
	{
		// see if any #includes remain
		size_t pos = csText.find( "#include" );
		if (pos == string::npos) break;
		// start of expanded source construction
		string tmp;
		if (pos > 0)
			tmp = csText.substr( 0, pos - 1 ) + "\n",
			includes[Ninc].start = LineCount( tmp ); // record first line of #include content
		else
			includes[Ninc].start = 0;
		// parse filename of include file
		pos = csText.find( "\"", pos + 1 );
		if (pos == string::npos) FatalError( "Expected \" after #include in shader." );
		size_t end = csText.find( "\"", pos + 1 );
		if (end == string::npos) FatalError( "Expected second \" after #include in shader." );
		string incFile = csText.substr( pos + 1, end - pos - 1 );
		// load include file content
		string incText = TextFileRead( incFile.c_str() );
		includes[Ninc].end = includes[Ninc].start + LineCount( incText );
		includes[Ninc++].file = incFile;
		if (incText.size() == 0) FatalError( "#include file not found:\n%s", incFile.c_str() );
		// cleanup include file content: we get some crap first sometimes, but why?
		int firstValidChar = 0;
		while (incText[firstValidChar] < 0) firstValidChar++;
		// add include file content and remainder of source to expanded source string
		tmp += incText.substr( firstValidChar, string::npos );
		tmp += csText.substr( end + 1, string::npos ) + "\n";
		// repeat until no #includes left
		csText = tmp;
	}
#endif
	// attempt to compile the loaded source text
	const char* source = csText.c_str();
	size_t size = strlen( source );
	cl_int error;
	program = clCreateProgramWithSource( context, 1, (const char**)&source, &size, &error );
	CHECKCL( error );
	// why does the nvidia compiler not support these:
	// -cl-nv-maxrregcount=64 not faster than leaving it out (same for 128)
	// -cl-no-subgroup-ifp ? fails on nvidia.
	// AMD-compatible compilation, thanks Rosalie de Winther
	char buildString[1024];
	strcpy( buildString, "-cl-std=CL2.0 " );
	strcat( buildString, "-cl-strict-aliasing " );
	strcat( buildString, "-cl-fast-relaxed-math " );
	strcat( buildString, "-cl-single-precision-constant " );
	// strcat( buildString, "-cl-uniform-work-group-size " );
	// strcat( buildString, "-cl-no-subgroup-ifp " );
	if (isNVidia) strcat( buildString, "-cl-nv-opt-level=9 " );
	// strcat( buildString, "-cl-nv-maxrregcount=32 " );
	error = clBuildProgram( program, 0, NULL, buildString, NULL, NULL );
	// handle errors
	if (error == CL_SUCCESS)
	{
		// dump PTX via: https://forums.developer.nvidia.com/t/pre-compiling-opencl-kernels-tutorial/17089
		// and: https://stackoverflow.com/questions/12868889/clgetprograminfo-cl-program-binary-sizes-incorrect-results
		cl_uint devCount;
		CHECKCL( clGetProgramInfo( program, CL_PROGRAM_NUM_DEVICES, sizeof( cl_uint ), &devCount, NULL ) );
		size_t* sizes = new size_t[devCount];
		sizes[0] = 0;
		size_t received;
		CHECKCL( clGetProgramInfo( program, CL_PROGRAM_BINARY_SIZES /* wrong data... */, devCount * sizeof( size_t ), sizes, &received ) );
		char** binaries = new char* [devCount];
		for (uint i = 0; i < devCount; i++)
			binaries[i] = new char[sizes[i] + 1];
		CHECKCL( clGetProgramInfo( program, CL_PROGRAM_BINARIES, devCount * sizeof( size_t ), binaries, NULL ) );
		FILE* f = fopen( "buildlog.txt", "wb" );
		for (uint i = 0; i < devCount; i++)
			fwrite( binaries[i], 1, sizes[i] + 1, f );
		fclose( f );
	}
	else
	{
		// obtain the error log from the cl compiler
		if (!log) log = new char[256 * 1024]; // can be quite large
		log[0] = 0;
		clGetProgramBuildInfo( program, getFirstDevice( context ), CL_PROGRAM_BUILD_LOG, 256 * 1024, log, &size );
		// save error log for closer inspection
		FILE* f = fopen( "errorlog.txt", "wb" );
		fwrite( log, 1, size, f );
		fclose( f );
	#if 0
		// find and display the first errormat; just dump it to a window
		log[2048] = 0; // truncate very long logs
		FatalError( log, "Build error" );
	#else
		// find and display the first error. Note: platform specific sadly; code below is for NVIDIA
		char* errorString = strstr( log, ": error:" );
		if (errorString)
		{
			int errorPos = (int)(errorString - log);
			while (errorPos > 0) if (log[errorPos - 1] == '\n') break; else errorPos--;
			// translate file and line number of error and report
			log[errorPos + 2048] = 0;
			int lineNr = 0, linePos = 0;
			char* lns = strstr( log + errorPos, ">:" ), * eol;
			if (!lns) FatalError( log + errorPos ); else
			{
				lns += 2;
				while (*lns >= '0' && *lns <= '9') lineNr = lineNr * 10 + (*lns++ - '0');
				lns++; // proceed to line number
				while (*lns >= '0' && *lns <= '9') linePos = linePos * 10 + (*lns++ - '0');
				lns += 9; // proceed to error message
				eol = lns;
				while (*eol != '\n' && *eol > 0) eol++;
				*eol = 0;
				lineNr--; // we count from 0 instead of 1
				// adjust file and linenr based on include file data
				string errorFile = file;
				bool errorInInclude = false;
				for (int i = Ninc - 1; i >= 0; i--)
				{
					if (lineNr > includes[i].end)
					{
						for (int j = 0; j <= i; j++) lineNr -= includes[j].end - includes[j].start;
						break;
					}
					else if (lineNr > includes[i].start)
					{
						errorFile = includes[i].file;
						lineNr -= includes[i].start;
						errorInInclude = true;
						break;
					}
				}
				if (!errorInInclude) lineNr -= vendorLines;
				// present error message
				char t[1024];
				sprintf( t, "file %s, line %i, pos %i:\n%s", errorFile.c_str(), lineNr + 1, linePos, lns );
				FatalError( t, "Build error" );
			}
		}
		else
		{
			// error string has unknown format; just dump it to a window
			log[2048] = 0; // truncate very long logs
			FatalError( log, "Build error" );
		}
	#endif
	}
	kernel = clCreateKernel( program, entryPoint, &error );
	if (kernel == 0) FatalError( "clCreateKernel failed: entry point not found." );
	CHECKCL( error );
	loadedKernels.push_back( this );
	// restore working directory
	chdir( dirBackup );
}

Kernel::Kernel( cl_program& existingProgram, char* entryPoint )
{
	CheckCLStarted();
	cl_int error;
	program = existingProgram;
	kernel = clCreateKernel( program, entryPoint, &error );
	if (kernel == 0) FatalError( "clCreateKernel failed: entry point not found." );
	CHECKCL( error );
}

// Kernel destructor
// ----------------------------------------------------------------------------
Kernel::~Kernel()
{
	if (kernel) clReleaseKernel( kernel );
	// if (program) clReleaseProgram( program ); // NOTE: may be shared with other kernels
	kernel = 0;
	// program = 0;
}

// InitCL method
// ----------------------------------------------------------------------------
bool Kernel::InitCL()
{
	cl_platform_id platform;
	cl_device_id* devices;
	cl_uint devCount;
	cl_int error;
	if (!CHECKCL( error = getPlatformID( &platform ) )) return false;
	if (!CHECKCL( error = clGetDeviceIDs( platform, CL_DEVICE_TYPE_ALL, 0, NULL, &devCount ) )) return false;
	devices = new cl_device_id[devCount];
	if (!CHECKCL( error = clGetDeviceIDs( platform, CL_DEVICE_TYPE_ALL, devCount, devices, NULL ) )) return false;
	int deviceUsed = -1;
	// search a capable OpenCL device
	char device_string[1024], device_platform[1024];
	for (uint i = 0; i < devCount; i++)
	{
		size_t extensionSize;
		CHECKCL( error = clGetDeviceInfo( devices[i], CL_DEVICE_EXTENSIONS, 0, NULL, &extensionSize ) );
		if (extensionSize > 0)
		{
			char* extensions = (char*)malloc( extensionSize );
			CHECKCL( error = clGetDeviceInfo( devices[i], CL_DEVICE_EXTENSIONS, extensionSize, extensions, &extensionSize ) );
			string deviceList( extensions );
			free( extensions );
			string mustHave[] = {
				"cl_khr_gl_sharing",
				"cl_khr_global_int32_base_atomics"
			};
			bool hasAll = true;
			for (int j = 0; j < 2; j++)
			{
				size_t o = 0, s = deviceList.find( ' ', o );
				bool hasFeature = false;
				while (s != deviceList.npos)
				{
					string subs = deviceList.substr( o, s - o );
					if (strcmp( mustHave[j].c_str(), subs.c_str() ) == 0) hasFeature = true;
					do { o = s + 1, s = deviceList.find( ' ', o ); } while (s == o);
				}
				if (!hasFeature) hasAll = false;
			}
			if (hasAll)
			{
				cl_context_properties props[] =
				{
					CL_GL_CONTEXT_KHR, (cl_context_properties)glfwGetWGLContext( window ),
					CL_WGL_HDC_KHR, (cl_context_properties)wglGetCurrentDC(),
					CL_CONTEXT_PLATFORM, (cl_context_properties)platform, 0
				};
				// attempt to create a context with the requested features
				context = clCreateContext( props, 1, &devices[i], NULL, NULL, &error );
				if (error == CL_SUCCESS)
				{
					candoInterop = true;
					deviceUsed = i;
					break;
				}
			}
			if (deviceUsed > -1) break;
		}
	}
	if (deviceUsed == -1) FatalError( "No capable OpenCL device found." );
	device = getFirstDevice( context );
	if (!CHECKCL( error )) return false;
	// print device name
	clGetDeviceInfo( devices[deviceUsed], CL_DEVICE_NAME, 1024, &device_string, NULL );
	clGetDeviceInfo( devices[deviceUsed], CL_DEVICE_VERSION, 1024, &device_platform, NULL );
	printf( "Device # %u, %s (%s)\n", deviceUsed, device_string, device_platform );
	// print local memory size
	size_t localMem;
	clGetDeviceInfo( devices[deviceUsed], CL_DEVICE_LOCAL_MEM_SIZE, sizeof( size_t ), &localMem, NULL );
	printf( "Local memory size: %iKB\n", (int)localMem >> 10 );
	// digest device string
	char* d = device_string;
	for (int i = 0; i < strlen( d ); i++) if (d[i] >= 'A' && d[i] <= 'Z') d[i] -= 'A' - 'a';
	if (strstr( d, "nvidia" ))
	{
		isNVidia = true;
		if (strstr( d, "rtx" ))
		{
			// detect Ampere GPUs
			if (strstr( d, "4050" ) || strstr( d, "4060" ) || strstr( d, "4070" ) || strstr( d, "4080" ) || strstr( d, "4090" )) isAda = true;
			if (strstr( d, "3050" ) || strstr( d, "3060" ) || strstr( d, "3070" ) || strstr( d, "3080" ) || strstr( d, "3090" )) isAmpere = true;
			if (strstr( d, "a2000" ) || strstr( d, "a3000" ) || strstr( d, "a4000" ) || strstr( d, "a5000" ) || strstr( d, "a6000" )) isAmpere = true;
			// detect Turing GPUs
			if (strstr( d, "2060" ) || strstr( d, "2070" ) || strstr( d, "2080" )) isTuring = true;
			// detect Titan RTX
			if (strstr( d, "titan rtx" )) isTuring = true;
			// detect Turing Quadro
			if (strstr( d, "quadro" ))
			{
				if (strstr( d, "3000" ) || strstr( d, "4000" ) || strstr( d, "5000" ) || strstr( d, "6000" ) || strstr( d, "8000" )) isTuring = true;
			}
		}
		else if (strstr( d, "gtx" ))
		{
			// detect Turing GPUs
			if (strstr( d, "1650" ) || strstr( d, "1660" )) isTuring = true;
			// detect Pascal GPUs
			if (strstr( d, "1010" ) || strstr( d, "1030" ) || strstr( d, "1050" ) || strstr( d, "1060" ) || strstr( d, "1070" ) || strstr( d, "1080" )) isPascal = true;
		}
		else if (strstr( d, "quadro" ))
		{
			// detect Pascal GPUs
			if (strstr( d, "p2000" ) || strstr( d, "p1000" ) || strstr( d, "p600" ) || strstr( d, "p400" ) || strstr( d, "p5000" ) || strstr( d, "p100" )) isPascal = true;
		}
		else
		{
			// detect Pascal GPUs
			if (strstr( d, "titan x" )) isPascal = true;
		}
	}
	else if (strstr( d, "amd" ) || strstr( d, "ellesmere" ) || strstr( d, "gfx1100" ) || strstr( d, "AMD" )) // JdW
	{
		isAMD = true;
	}
	else if (strstr( d, "intel" ))
	{
		isIntel = true;
	}
	else
	{
		isOther = true;
	}
	// report on findings
	printf( "hardware detected: " );
	if (isNVidia)
	{
		printf( "NVIDIA, " );
		if (isAmpere) printf( "AMPERE class.\n" );
		else if (isTuring) printf( "TURING class.\n" );
		else if (isPascal) printf( "PASCAL class.\n" );
		else if (isAda) printf( "ADA class.\n" );
		else printf( "PRE-PASCAL hardware (warning: slow).\n" );
	}
	else if (isAMD)
	{
		printf( "AMD.\n" );
	}
	else if (isIntel)
	{
		printf( "Intel.\n" );
	}
	else
	{
		printf( "identification failed.\n" );
	}
	// create a command-queue
	cl_queue_properties props[] = { CL_QUEUE_PROPERTIES, CL_QUEUE_PROFILING_ENABLE, 0 };
	queue = clCreateCommandQueueWithProperties( context, devices[deviceUsed], props, &error );
	if (!CHECKCL( error )) return false;
	// create a second command queue for asynchronous copies
	queue2 = clCreateCommandQueueWithProperties( context, devices[deviceUsed], props, &error );
	if (!CHECKCL( error )) return false;
	// cleanup
	delete devices;
	clStarted = true;
	return true;
}

// KillCL method
// ----------------------------------------------------------------------------
void Kernel::KillCL()
{
	if (!clStarted) return;
	clReleaseCommandQueue( queue2 );
	clReleaseCommandQueue( queue );
	clReleaseContext( context );
}

// CheckCLStarted method
// ----------------------------------------------------------------------------
void Kernel::CheckCLStarted()
{
	if (!clStarted) FatalError( "Call InitCL() before using OpenCL functionality." );
}

// SetArgument methods
// ----------------------------------------------------------------------------
void Kernel::SetArgument( int idx, Buffer* buffer )
{
	CheckCLStarted();
	if (!buffer)
	{
		clSetKernelArg( kernel, idx, sizeof( cl_mem ), 0 );
	}
	else
	{
		clSetKernelArg( kernel, idx, sizeof( cl_mem ), buffer->GetDevicePtr() );
		if (buffer->type & Buffer::TARGET)
		{
			if (acqBuffer) FatalError( "Kernel can take only one texture target buffer argument." );
			acqBuffer = buffer;
		}
	}
}
void Kernel::SetArgument( int idx, float3 value )
{
	CheckCLStarted(); float4 tmp( value, 0 );
	clSetKernelArg( kernel, idx, sizeof( float4 ), &tmp );
}
void Kernel::SetArgument( int idx, cl_mem* buffer )
{
	CheckCLStarted();
	clSetKernelArg( kernel, idx, sizeof( cl_mem ), buffer );
}

// Run method
// ----------------------------------------------------------------------------
void Kernel::Run( const size_t count, const size_t localSize, cl_event* eventToWaitFor, cl_event* eventToSet )
{
	CheckCLStarted();
	cl_int error;
	if (acqBuffer)
	{
		if (!Kernel::candoInterop) FatalError( "OpenGL interop functionality required but not available." );
		CHECKCL( error = clEnqueueAcquireGLObjects( queue, 1, acqBuffer->GetDevicePtr(), 0, 0, 0 ) );
		CHECKCL( error = clEnqueueNDRangeKernel( queue, kernel, 1, 0, &count, localSize == 0 ? 0 : &localSize, eventToWaitFor ? 1 : 0, eventToWaitFor, eventToSet ) );
		CHECKCL( error = clEnqueueReleaseGLObjects( queue, 1, acqBuffer->GetDevicePtr(), 0, 0, 0 ) );
	}
	else
	{
		CHECKCL( error = clEnqueueNDRangeKernel( queue, kernel, 1, 0, &count, localSize == 0 ? 0 : &localSize, eventToWaitFor ? 1 : 0, eventToWaitFor, eventToSet ) );
	}
}

void Kernel::Run2D( const int2 count, const int2 lsize, cl_event* eventToWaitFor, cl_event* eventToSet )
{
	CheckCLStarted();
	size_t workSize[2] = { (size_t)count.x, (size_t)count.y };
	size_t localSize[2];
	if (lsize.x > 0 && lsize.y > 0)
	{
		// use specified workgroup size
		localSize[0] = (size_t)lsize.x;
		localSize[1] = (size_t)lsize.y;
	}
	else
	{
		// workgroup size not specified; use something reasonable
		localSize[0] = 32;
		localSize[1] = 4;
	}
	cl_int error;
	if (acqBuffer)
	{
		if (!Kernel::candoInterop) FatalError( "OpenGL interop functionality required but not available." );
		CHECKCL( error = clEnqueueAcquireGLObjects( queue, 1, acqBuffer->GetDevicePtr(), 0, 0, 0 ) );
		CHECKCL( error = clEnqueueNDRangeKernel( queue, kernel, 2, 0, workSize, localSize, eventToWaitFor ? 1 : 0, eventToWaitFor, eventToSet ) );
		CHECKCL( error = clEnqueueReleaseGLObjects( queue, 1, acqBuffer->GetDevicePtr(), 0, 0, 0 ) );
	}
	else
	{
		CHECKCL( error = clEnqueueNDRangeKernel( queue, kernel, 2, 0, workSize, localSize, eventToWaitFor ? 1 : 0, eventToWaitFor, eventToSet ) );
	}
}