// Template, 2024 IGAD Edition
// Get the latest version from: https://github.com/jbikker/tmpl8
// IGAD/NHTV/BUAS/UU - Jacco Bikker - 2006-2024

#pragma once

// OpenCL buffer
class Buffer
{
public:
	enum { DEFAULT = 0, TEXTURE = 8, TARGET = 16, READONLY = 1, WRITEONLY = 2 };
	// constructor / destructor
	Buffer() : hostBuffer( 0 ) {}
	Buffer( unsigned int N, void* ptr = 0, unsigned int t = DEFAULT );
	~Buffer();
	cl_mem* GetDevicePtr() { return &deviceBuffer; }
	unsigned int* GetHostPtr();
	void CopyToDevice( const bool blocking = true );
	void CopyToDevice( const int offset, const int size, const bool blocking = true );
	void CopyToDevice2( bool blocking, cl_event* e = 0, const size_t s = 0 );
	void CopyFromDevice( const bool blocking = true );
	void CopyFromDevice( const int offset, const int size, const bool blocking = true );
	void CopyTo( Buffer* buffer );
	void Clear();
	// data members
	unsigned int* hostBuffer;
	cl_mem deviceBuffer = 0;
	unsigned int type, size /* in bytes */, textureID;
	bool ownData, aligned;
};

// OpenCL kernel
class Kernel
{
	friend class Buffer;
public:
	// constructor / destructor
	Kernel( char* file, char* entryPoint );
	Kernel( cl_program& existingProgram, char* entryPoint );
	~Kernel();
	// get / set
	cl_kernel& GetKernel() { return kernel; }
	cl_program& GetProgram() { return program; }
	static cl_command_queue& GetQueue() { return queue; }
	static cl_command_queue& GetQueue2() { return queue2; }
	static cl_context& GetContext() { return context; }
	static cl_device_id& GetDevice() { return device; }
	// run methods
#if 1
	void Run( cl_event* eventToWaitFor = 0, cl_event* eventToSet = 0 );
	void Run( cl_mem* buffers, const int count = 1, cl_event* eventToWaitFor = 0, cl_event* eventToSet = 0, cl_event* acq = 0, cl_event* rel = 0 );
	void Run( Buffer* buffer, const int2 localSize = make_int2( 32, 2 ), cl_event* eventToWaitFor = 0, cl_event* eventToSet = 0, cl_event* acq = 0, cl_event* rel = 0 );
	void Run( Buffer* buffer, const int count = 1, cl_event* eventToWaitFor = 0, cl_event* eventToSet = 0, cl_event* acq = 0, cl_event* rel = 0 );
#endif
	void Run( const size_t count, const size_t localSize = 0, cl_event* eventToWaitFor = 0, cl_event* eventToSet = 0 );
	void Run2D( const int2 count, const int2 lsize = 0, cl_event* eventToWaitFor = 0, cl_event* eventToSet = 0 );
	// Argument passing with template trickery; up to 20 arguments in a single call;
	// each argument may be of any of the supported types. Approach borrowed from NVIDIA/CUDA.
#define T_ typename
	template<T_ A> void SetArguments( A a ) { InitArgs(); SetArgument( 0, a ); }
	template<T_ A, T_ B> void SetArguments( A a, B b ) { InitArgs(); S( 0, a ); S( 1, b ); }
	template<T_ A, T_ B, T_ C> void SetArguments( A a, B b, C c ) { InitArgs(); S( 0, a ); S( 1, b ); S( 2, c ); }
	template<T_ A, T_ B, T_ C, T_ D> void SetArguments( A a, B b, C c, D d ) { InitArgs(); S( 0, a ); S( 1, b ); S( 2, c ); S( 3, d ); }
	template<T_ A, T_ B, T_ C, T_ D, T_ E> void SetArguments( A a, B b, C c, D d, E e ) { InitArgs(); S( 0, a ); S( 1, b ); S( 2, c ); S( 3, d ); S( 4, e ); }
	template<T_ A, T_ B, T_ C, T_ D, T_ E, T_ F> void SetArguments( A a, B b, C c, D d, E e, F f )
	{
		InitArgs(); S( 0, a ); S( 1, b ); S( 2, c ); S( 3, d ); S( 4, e ); S( 5, f );
	}
	template<T_ A, T_ B, T_ C, T_ D, T_ E, T_ F, T_ G> void SetArguments( A a, B b, C c, D d, E e, F f, G g )
	{
		InitArgs(); S( 0, a ); S( 1, b ); S( 2, c ); S( 3, d ); S( 4, e ); S( 5, f ); S( 6, g );
	}
	template<T_ A, T_ B, T_ C, T_ D, T_ E, T_ F, T_ G, T_ H> void SetArguments( A a, B b, C c, D d, E e, F f, G g, H h )
	{
		InitArgs(); S( 0, a ); S( 1, b ); S( 2, c ); S( 3, d ); S( 4, e ); S( 5, f ); S( 6, g ); S( 7, h );
	}
	template<T_ A, T_ B, T_ C, T_ D, T_ E, T_ F, T_ G, T_ H, T_ I> void SetArguments( A a, B b, C c, D d, E e, F f, G g, H h, I i )
	{
		InitArgs(); S( 0, a ); S( 1, b ); S( 2, c ); S( 3, d ); S( 4, e ); S( 5, f ); S( 6, g ); S( 7, h ); S( 8, i );
	}
	template<T_ A, T_ B, T_ C, T_ D, T_ E, T_ F, T_ G, T_ H, T_ I, T_ J>
	void SetArguments( A a, B b, C c, D d, E e, F f, G g, H h, I i, J j )
	{
		InitArgs();
		S( 0, a ); S( 1, b ); S( 2, c ); S( 3, d ); S( 4, e );
		S( 5, f ); S( 6, g ); S( 7, h ); S( 8, i ); S( 9, j );
	}
	template<T_ A, T_ B, T_ C, T_ D, T_ E, T_ F, T_ G, T_ H, T_ I, T_ J, T_ K>
	void SetArguments( A a, B b, C c, D d, E e, F f, G g, H h, I i, J j, K k )
	{
		InitArgs();
		S( 0, a ); S( 1, b ); S( 2, c ); S( 3, d ); S( 4, e ); S( 5, f );
		S( 6, g ); S( 7, h ); S( 8, i ); S( 9, j ); S( 10, k );
	}
	template<T_ A, T_ B, T_ C, T_ D, T_ E, T_ F, T_ G, T_ H, T_ I, T_ J, T_ K, T_ L>
	void SetArguments( A a, B b, C c, D d, E e, F f, G g, H h, I i, J j, K k, L l )
	{
		InitArgs();
		S( 0, a ); S( 1, b ); S( 2, c ); S( 3, d ); S( 4, e ); S( 5, f );
		S( 6, g ); S( 7, h ); S( 8, i ); S( 9, j ); S( 10, k ); S( 11, l );
	}
	template<T_ A, T_ B, T_ C, T_ D, T_ E, T_ F, T_ G, T_ H, T_ I, T_ J, T_ K, T_ L, T_ M>
	void SetArguments( A a, B b, C c, D d, E e, F f, G g, H h, I i, J j, K k, L l, M m )
	{
		InitArgs();
		S( 0, a ); S( 1, b ); S( 2, c ); S( 3, d ); S( 4, e ); S( 5, f ); S( 6, g );
		S( 7, h ); S( 8, i ); S( 9, j ); S( 10, k ); S( 11, l ); S( 12, m );
	}
	template<T_ A, T_ B, T_ C, T_ D, T_ E, T_ F, T_ G, T_ H, T_ I, T_ J, T_ K, T_ L, T_ M, T_ N>
	void SetArguments( A a, B b, C c, D d, E e, F f, G g, H h, I i, J j, K k, L l, M m, N n )
	{
		InitArgs();
		S( 0, a ); S( 1, b ); S( 2, c ); S( 3, d ); S( 4, e ); S( 5, f ); S( 6, g );
		S( 7, h ); S( 8, i ); S( 9, j ); S( 10, k ); S( 11, l ); S( 12, m ), S( 13, n );
	}
	template<T_ A, T_ B, T_ C, T_ D, T_ E, T_ F, T_ G, T_ H, T_ I, T_ J, T_ K, T_ L, T_ M, T_ N, T_ O>
	void SetArguments( A a, B b, C c, D d, E e, F f, G g, H h, I i, J j, K k, L l, M m, N n, O o )
	{
		InitArgs();
		S( 0, a ); S( 1, b ); S( 2, c ); S( 3, d ); S( 4, e ); S( 5, f ); S( 6, g ); S( 7, h );
		S( 8, i ); S( 9, j ); S( 10, k ); S( 11, l ); S( 12, m ), S( 13, n ); S( 14, o );
	}
	template<T_ A, T_ B, T_ C, T_ D, T_ E, T_ F, T_ G, T_ H, T_ I, T_ J, T_ K, T_ L, T_ M, T_ N, T_ O, T_ P>
	void SetArguments( A a, B b, C c, D d, E e, F f, G g, H h, I i, J j, K k, L l, M m, N n, O o, P p )
	{
		InitArgs();
		S( 0, a ); S( 1, b ); S( 2, c ); S( 3, d ); S( 4, e ); S( 5, f ); S( 6, g ); S( 7, h );
		S( 8, i ); S( 9, j ); S( 10, k ); S( 11, l ); S( 12, m ), S( 13, n ); S( 14, o ), S( 15, p );
	}
	template<T_ A, T_ B, T_ C, T_ D, T_ E, T_ F, T_ G, T_ H, T_ I, T_ J, T_ K, T_ L, T_ M, T_ N, T_ O, T_ P, T_ Q>
	void SetArguments( A a, B b, C c, D d, E e, F f, G g, H h, I i, J j, K k, L l, M m, N n, O o, P p, Q q )
	{
		InitArgs();
		S( 0, a ); S( 1, b ); S( 2, c ); S( 3, d ); S( 4, e ); S( 5, f ); S( 6, g ); S( 7, h );
		S( 8, i ); S( 9, j ); S( 10, k ); S( 11, l ); S( 12, m ), S( 13, n ); S( 14, o ), S( 15, p );
		S( 16, q );
	}
	template<T_ A, T_ B, T_ C, T_ D, T_ E, T_ F, T_ G, T_ H, T_ I, T_ J, T_ K, T_ L, T_ M, T_ N, T_ O, T_ P, T_ Q, T_ R>
	void SetArguments( A a, B b, C c, D d, E e, F f, G g, H h, I i, J j, K k, L l, M m, N n, O o, P p, Q q, R r )
	{
		InitArgs();
		S( 0, a ); S( 1, b ); S( 2, c ); S( 3, d ); S( 4, e ); S( 5, f ); S( 6, g ); S( 7, h );
		S( 8, i ); S( 9, j ); S( 10, k ); S( 11, l ); S( 12, m ), S( 13, n ); S( 14, o ), S( 15, p );
		S( 16, q ), S( 17, r );
	}
	template<T_ A, T_ B, T_ C, T_ D, T_ E, T_ F, T_ G, T_ H, T_ I, T_ J, T_ K, T_ L, T_ M, T_ N, T_ O, T_ P, T_ Q, T_ R, T_ T>
	void SetArguments( A a, B b, C c, D d, E e, F f, G g, H h, I i, J j, K k, L l, M m, N n, O o, P p, Q q, R r, T t )
	{
		InitArgs();
		S( 0, a ); S( 1, b ); S( 2, c ); S( 3, d ); S( 4, e ); S( 5, f ); S( 6, g ); S( 7, h );
		S( 8, i ); S( 9, j ); S( 10, k ); S( 11, l ); S( 12, m ), S( 13, n ); S( 14, o ), S( 15, p );
		S( 16, q ), S( 17, r ), S( 18, t );
	}
	template<T_ A, T_ B, T_ C, T_ D, T_ E, T_ F, T_ G, T_ H, T_ I, T_ J, T_ K, T_ L, T_ M, T_ N, T_ O, T_ P, T_ Q, T_ R, T_ T, T_ U>
	void SetArguments( A a, B b, C c, D d, E e, F f, G g, H h, I i, J j, K k, L l, M m, N n, O o, P p, Q q, R r, T t, U u )
	{
		InitArgs();
		S( 0, a ); S( 1, b ); S( 2, c ); S( 3, d ); S( 4, e ); S( 5, f ); S( 6, g ); S( 7, h );
		S( 8, i ); S( 9, j ); S( 10, k ); S( 11, l ); S( 12, m ), S( 13, n ); S( 14, o ), S( 15, p );
		S( 16, q ), S( 17, r ), S( 18, t ), S( 19, u );
	}
	template<T_ T> void S( uint i, T t ) { SetArgument( i, t ); }
	void InitArgs() { acqBuffer = 0; /* nothing to acquire until told otherwise */ }
#undef T_
private:
	void SetArgument( int idx, cl_mem* buffer );
	void SetArgument( int idx, Buffer* buffer );
	void SetArgument( int idx, float3 );
	template<class T> void SetArgument( int idx, T value )
	{
		CheckCLStarted();
		clSetKernelArg( kernel, idx, sizeof( T ), &value );
	}
	// other methods
public:
	static bool InitCL();
	static void CheckCLStarted();
	static void KillCL();
private:
	// data members
	char* sourceFile = 0;
	Buffer* acqBuffer = 0;
	cl_kernel kernel;
	cl_mem vbo_cl;
	cl_program program;
	inline static cl_device_id device;
	inline static cl_context context; // simplifies some things, but limits us to one device
	inline static cl_command_queue queue, queue2;
	inline static char* log = 0;
	inline static bool isNVidia = false, isAMD = false, isIntel = false, isOther = false;
	inline static bool isAmpere = false, isTuring = false, isPascal = false, isAda = false, isHopper = false;
	inline static int vendorLines = 0;
	inline static vector<Kernel*> loadedKernels;
public:
	inline static bool candoInterop = false, clStarted = false;
};