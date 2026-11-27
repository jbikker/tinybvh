// tiny_bvh_tlas_gpu.cpp
//
// Demo for tlas_batch.cl: a TLAS over BLASses of mixed layouts is converted to
// a set of OpenCL buffers by GPUTLAS, after which batches of rays are traced
// against it on the GPU with the 'batch_nearest_tlas' and 'batch_any_tlas'
// kernels. Primary ray setup and shading happen on the CPU; the two kernels
// are the only GPU-side code, which keeps tlas_batch.cl usable as a drop-in
// traversal service for any renderer.
//
// GPUTLAS is intentionally generic: hand it any tinybvh TLAS plus the BLAS
// array it was built over and it produces the buffers the kernels expect.

#define FENSTER_APP_IMPLEMENTATION
#define SCRWIDTH 800
#define SCRHEIGHT 600
#define TILESIZE 16
#include "external/fenster.h" // https://github.com/zserge/fenster

#define GRIDSIZE 3
#define INSTCOUNT (GRIDSIZE * GRIDSIZE * GRIDSIZE)

#define NO_DOUBLE_PRECISION_SUPPORT
#define NO_VOXEL_SUPPORT
#define TINYBVH_IMPLEMENTATION
#define INST_IDX_BITS 8 // override default; space for 256 instances.
#include "tiny_bvh.h"
#define TINY_OCL_IMPLEMENTATION
#include "tiny_ocl.h"

#include <atomic>
#include <fstream>
#include <thread>
#include <vector>

using namespace tinybvh;
using namespace tinyocl;

// Set to 1 to validate every GPU hit against the CPU traversal on frame 0.
#define VERIFY 0

// ============================================================================
//
//        G P U T L A S
//
// Converts an existing TLAS + BLAS set into the buffers that tlas_batch.cl
// consumes, and keeps them in sync when instances move.
//
// ============================================================================

// 48-byte ray, matching 'struct Ray' in tlas_batch.cl.
struct GPURay
{
	GPURay() = default;
	GPURay( const bvhvec3& origin, const bvhvec3& direction, float tmax = BVH_FAR,
		uint32_t rayMask = RAY_MASK_INTERSECT_ALL )
	{
		O = bvhvec4( origin, tinybvh_as_float( rayMask ) );
		D = bvhvec4( direction, 0 );
		hit = bvhvec4( tmax, 0, 0, 0 );
	}
	bvhvec4 O;		// w: ray mask, as uint bits.
	bvhvec4 D;		// w: unused.
	bvhvec4 hit;	// x: tmax in, t out. y, z: barycentrics. w: prim + inst.
};

class GPUTLAS
{
public:
	// BLAS layout identifiers; keep in sync with the BLASTYPE_ defines in
	// tlas_batch.cl. Values 4 and up are free for new layouts.
	enum
	{
		BLASTYPE_AILALAINE = 0,		// tinybvh::BVH_GPU
		BLASTYPE_CWBVH = 1,			// tinybvh::BVH8_CWBVH
		BLASTYPE_AMD_HW_BVH4 = 2,	// tinybvh::BVH4_AMD_HW
		BLASTYPE_AMD_HW_CWBVH8 = 3	// tinybvh::CWBVH8_AMD_HW
	};
	// Per-BLAS record; matches 'struct BLASDesc' in tlas_batch.cl. Offsets are
	// in 16-byte blocks from the start of the pool for the layout, which is
	// the common unit of every layout tinybvh emits.
	struct BLASDesc
	{
		uint32_t blasType = 0;
		uint32_t nodeOffset = 0;
		uint32_t primOffset = 0;
		uint32_t dummy[5] = {};
	};
	~GPUTLAS() { Clear(); }

	// Build: converts a TLAS in BVH_GPU layout plus the BLASses it was built
	// over into device buffers. Returns false, with a message on stderr, when
	// a BLAS uses a layout the kernels do not implement.
	bool Build( BVH_GPU& tlas, BLASInstance* instances, uint32_t instCount,
		BVHBase** blasList, uint32_t blasCount );

	// SyncTLAS: re-uploads the TLAS and instance data. Call after rebuilding
	// the TLAS because instances moved; BLAS data is untouched.
	void SyncTLAS();

	// SetArgs: binds all scene buffers plus a ray batch to a kernel. Argument
	// order matches batch_nearest_tlas / batch_any_tlas.
	void SetArgs( Kernel& kernel, Buffer* rays, uint32_t rayCount )
	{
		kernel.SetArguments( tlasNodeBuf, tlasIdxBuf, instBuf, descBuf,
			alNodeBuf, alTriBuf, cwNodeBuf, cwTriBuf, amd4Buf, amd8Buf,
			rays, rayCount );
	}

	void Clear();

	// device buffers, in kernel argument order
	Buffer* tlasNodeBuf = 0;
	Buffer* tlasIdxBuf = 0;
	Buffer* instBuf = 0;
	Buffer* descBuf = 0;
	Buffer* alNodeBuf = 0;	// BVH_GPU node pool
	Buffer* alTriBuf = 0;	// BVH_GPU ordered vertex pool
	Buffer* cwNodeBuf = 0;	// CWBVH node pool
	Buffer* cwTriBuf = 0;	// CWBVH triangle pool
	Buffer* amd4Buf = 0;	// BVH4_AMD_HW pool
	Buffer* amd8Buf = 0;	// CWBVH8_AMD_HW pool

private:
	// One pool: a host-side array of 16-byte blocks plus its device buffer.
	struct Pool
	{
		bvhvec4* data = 0;
		uint32_t blocks = 0, capacity = 0;
		uint32_t Append( const void* src, uint32_t srcBlocks );
		void Reserve( uint32_t n );
		void Free() { tinybvh::free64( data ), data = 0, blocks = capacity = 0; }
	};
	// Each BLAS chunk starts on a 128-byte boundary: the AMD layouts address
	// nodes relative to the pointer we hand them and use 128-byte records.
	static const uint32_t CHUNK_ALIGN = 8;
	Pool alNodes, alTris, cwNodes, cwTris, amd4, amd8;
	BLASDesc* desc = 0;
	uint32_t descCount = 0;
	// kept for SyncTLAS
	BVH_GPU* tlas = 0;
	BLASInstance* inst = 0;
	uint32_t instCount = 0;
	static Buffer* MakeBuffer( void* data, uint32_t bytes );
	static Buffer* MakePoolBuffer( Pool& pool );
};

// Append: copies srcBlocks 16-byte blocks into the pool; returns the block
// offset the chunk starts at.
uint32_t GPUTLAS::Pool::Append( const void* src, uint32_t srcBlocks )
{
	const uint32_t start = blocks;
	Reserve( start + srcBlocks );
	memcpy( data + start, src, (size_t)srcBlocks * 16 );
	blocks = start + srcBlocks;
	// pad to the next chunk boundary
	const uint32_t padded = (blocks + CHUNK_ALIGN - 1) & ~(CHUNK_ALIGN - 1);
	Reserve( padded );
	memset( data + blocks, 0, (size_t)(padded - blocks) * 16 );
	blocks = padded;
	return start;
}

void GPUTLAS::Pool::Reserve( uint32_t n )
{
	if (n <= capacity) return;
	uint32_t newCapacity = tinybvh_max( n, tinybvh_max( 64u, capacity * 2 ) );
	bvhvec4* newData = (bvhvec4*)tinybvh::malloc64( (size_t)newCapacity * 16 );
	if (data) memcpy( newData, data, (size_t)blocks * 16 ), tinybvh::free64( data );
	data = newData, capacity = newCapacity;
}

Buffer* GPUTLAS::MakeBuffer( void* data, uint32_t bytes )
{
	Buffer* buffer = new Buffer( bytes, data );
	buffer->CopyToDevice();
	return buffer;
}

// MakePoolBuffer: an empty pool still needs a valid buffer, since the kernel
// signature is fixed; one block of zeroes is enough.
Buffer* GPUTLAS::MakePoolBuffer( Pool& pool )
{
	if (pool.blocks == 0) pool.Reserve( 1 ), memset( pool.data, 0, 16 ), pool.blocks = 1;
	return MakeBuffer( pool.data, pool.blocks * 16 );
}

bool GPUTLAS::Build( BVH_GPU& tlasIn, BLASInstance* instances, uint32_t instanceCount,
	BVHBase** blasList, uint32_t blasCount )
{
	// the host and kernel structs must agree byte for byte
	static_assert(sizeof( GPURay ) == 48, "GPURay must be 48 bytes." );
	static_assert(sizeof( BLASInstance ) == 192, "BLASInstance must be 192 bytes." );
	static_assert(sizeof( BLASDesc ) == 32, "BLASDesc must be 32 bytes." );
	static_assert(sizeof( BVH_GPU::BVHNode ) == 64, "BVH_GPU::BVHNode must be 64 bytes." );
	static_assert(INST_IDX_SHFT == 24, "tlas_batch.cl defaults to INST_IDX_SHFT 24; "
		"change the #define at the top of the kernel to match INST_IDX_BITS." );
	Clear();
	if (tlasIn.layout != LAYOUT_BVH_GPU)
	{
		fprintf( stderr, "GPUTLAS: the TLAS must be in BVH_GPU layout.\n" );
		return false;
	}
	tlas = &tlasIn, inst = instances, instCount = instanceCount;
	// pack each BLAS into the pool for its layout and record where it landed
	desc = new BLASDesc[blasCount], descCount = blasCount;
	for (uint32_t i = 0; i < blasCount; i++)
	{
		BVHBase* blas = blasList[i];
		switch (blas->layout)
		{
		case LAYOUT_BVH_GPU:
		{
			BVH_GPU* b = (BVH_GPU*)blas;
			if (b->orderedVerts.stride != sizeof( bvhvec4 ))
			{
				fprintf( stderr, "GPUTLAS: BLAS %u has strided vertices.\n", i );
				return false;
			}
			desc[i].blasType = BLASTYPE_AILALAINE;
			desc[i].nodeOffset = alNodes.Append( b->bvhNode, (uint32_t)b->usedNodes * 4 );
			desc[i].primOffset = alTris.Append( b->orderedVerts.data, b->orderedVerts.count );
			break;
		}
		case LAYOUT_CWBVH:
		{
			BVH8_CWBVH* b = (BVH8_CWBVH*)blas;
			desc[i].blasType = BLASTYPE_CWBVH;
			desc[i].nodeOffset = cwNodes.Append( b->bvh8Data, b->usedBlocks );
			desc[i].primOffset = cwTris.Append( b->bvh8Tris, b->usedTriBlocks );
			break;
		}
		case LAYOUT_BVH4_AMD_HW:
		{
			BVH4_AMD_HW* b = (BVH4_AMD_HW*)blas;
			desc[i].blasType = BLASTYPE_AMD_HW_BVH4;
			desc[i].nodeOffset = amd4.Append( b->bvh4Data, b->usedBlocks );
			break;
		}
		case LAYOUT_CWBVH8_AMD_HW:
		{
			CWBVH8_AMD_HW* b = (CWBVH8_AMD_HW*)blas;
			desc[i].blasType = BLASTYPE_AMD_HW_CWBVH8;
			desc[i].nodeOffset = amd8.Append( b->bvh8Data, b->usedBlocks );
			break;
		}
		default:
			fprintf( stderr, "GPUTLAS: BLAS %u uses a layout the kernels do not "
				"implement (layout id %i).\n", i, (int)blas->layout );
			return false;
		}
	}
	// create and fill the device buffers
	tlasNodeBuf = MakeBuffer( tlas->bvhNode, (uint32_t)tlas->usedNodes * 64 );
	tlasIdxBuf = MakeBuffer( tlas->bvh.primIdx, (uint32_t)tlas->bvh.idxCount * 4 );
	instBuf = MakeBuffer( inst, instCount * (uint32_t)sizeof( BLASInstance ) );
	descBuf = MakeBuffer( desc, descCount * (uint32_t)sizeof( BLASDesc ) );
	alNodeBuf = MakePoolBuffer( alNodes ), alTriBuf = MakePoolBuffer( alTris );
	cwNodeBuf = MakePoolBuffer( cwNodes ), cwTriBuf = MakePoolBuffer( cwTris );
	amd4Buf = MakePoolBuffer( amd4 ), amd8Buf = MakePoolBuffer( amd8 );
	return true;
}

void GPUTLAS::SyncTLAS()
{
	// A rebuild can grow the TLAS or move its arrays, so check before copying.
	const uint32_t nodeBytes = (uint32_t)tlas->usedNodes * 64;
	const uint32_t idxBytes = (uint32_t)tlas->bvh.idxCount * 4;
	if (tlasNodeBuf->hostBuffer != (unsigned*)tlas->bvhNode || tlasNodeBuf->size < nodeBytes)
		delete tlasNodeBuf, tlasNodeBuf = MakeBuffer( tlas->bvhNode, nodeBytes );
	else tlasNodeBuf->CopyToDevice( 0, (int)nodeBytes );
	if (tlasIdxBuf->hostBuffer != (unsigned*)tlas->bvh.primIdx || tlasIdxBuf->size < idxBytes)
		delete tlasIdxBuf, tlasIdxBuf = MakeBuffer( tlas->bvh.primIdx, idxBytes );
	else tlasIdxBuf->CopyToDevice( 0, (int)idxBytes );
	instBuf->CopyToDevice();
}

void GPUTLAS::Clear()
{
	delete tlasNodeBuf, delete tlasIdxBuf, delete instBuf, delete descBuf;
	delete alNodeBuf, delete alTriBuf, delete cwNodeBuf, delete cwTriBuf;
	delete amd4Buf, delete amd8Buf;
	tlasNodeBuf = tlasIdxBuf = instBuf = descBuf = 0;
	alNodeBuf = alTriBuf = cwNodeBuf = cwTriBuf = amd4Buf = amd8Buf = 0;
	alNodes.Free(), alTris.Free(), cwNodes.Free(), cwTris.Free();
	amd4.Free(), amd8.Free();
	delete[] desc, desc = 0, descCount = 0;
}

// ============================================================================
//
//        D E M O
//
// ============================================================================

// scene data
static BVH_GPU bunnyBlas;		// BLAS in Aila & Laine layout
static BVH8_CWBVH sponzaBlas;	// BLAS in CWBVH layout
static BVHBase* blasList[] = { &bunnyBlas, &sponzaBlas };
static BLASInstance inst[INSTCOUNT + 1 /* one extra for sponza */];
static BVH_GPU tlas;
static GPUTLAS gpuScene;

static bvhvec4* triangles = 0;	// sponza vertices
static bvhvec4* bunny = 0;		// bunny vertices
static int verts = 0, bverts = 0, frameIdx = 0;

// GPU state
static Kernel* nearestKernel = 0;
static Kernel* anyKernel = 0;
static Buffer* primaryBuf = 0;
static Buffer* shadowBuf = 0;
static GPURay* primary = 0;
static GPURay* shadow = 0;
static uint32_t* shadowSrc = 0;	// pixel index for each compacted shadow ray
static uint32_t shadowCount = 0;

// light
static const bvhvec3 lightPos( 20, 27, 3 );

// view pyramid for a pinhole camera
static bvhvec3 eye( -15.24f, 21.5f, 2.54f ), p1, p2, p3;
static bvhvec3 view = tinybvh_normalize( bvhvec3( 0.826f, -0.438f, -0.356f ) );

// tile scheduling
static std::atomic<int> tileIdx( 0 );
static unsigned threadCount = std::thread::hardware_concurrency();

void Init()
{
	// load raw vertex data for Crytek's Sponza
	std::fstream s{ "./testdata/cryteksponza.bin", s.binary | s.in };
	s.read( (char*)&verts, 4 );
	printf( "Loading triangle data (%i tris).\n", verts );
	verts *= 3, triangles = (bvhvec4*)tinybvh::malloc64( verts * 16 );
	s.read( (char*)triangles, verts * 16 );

	// load the bunny
	std::fstream b{ "./testdata/bunny.bin", b.binary | b.in };
	b.read( (char*)&bverts, 4 );
	bverts *= 3, bunny = (bvhvec4*)tinybvh::malloc64( bverts * 16 );
	b.read( (char*)bunny, bverts * 16 );

	// build the BLASses, deliberately in two different layouts
	sponzaBlas.BuildHQ( triangles, verts / 3 );
	bunnyBlas.Build( bunny, bverts / 3 );

	// set up the instances: a grid of bunnies plus one sponza
	int idx = 0;
	for (int x = 0; x < GRIDSIZE; x++) for (int y = 0; y < GRIDSIZE; y++)
		for (int z = 0; z < GRIDSIZE; z++, idx++)
		{
			inst[idx] = BLASInstance( 0 /* bunny */ );
			inst[idx].transform[0] = inst[idx].transform[5] = inst[idx].transform[10] = 0.6f;
			inst[idx].transform[3] = (float)x * 5 - GRIDSIZE * 2.5f;
			inst[idx].transform[7] = (float)y * 5 - GRIDSIZE * 2.5f + 7;
			inst[idx].transform[11] = (float)z * 5 - GRIDSIZE * 2.5f + 1;
		}
	inst[idx] = BLASInstance( 1 /* sponza */ );
	tlas.Build( inst, 1 + INSTCOUNT, blasList, 2 );

	// convert the whole structure to GPU buffers
	if (!gpuScene.Build( tlas, inst, 1 + INSTCOUNT, blasList, 2 )) exit( 1 );

	// prepare the kernels and the ray batches
	nearestKernel = new Kernel( "tlas_batch.cl", "batch_nearest_tlas" );
	anyKernel = new Kernel( "tlas_batch.cl", "batch_any_tlas" );
	const uint32_t rayCount = SCRWIDTH * SCRHEIGHT;
	primary = (GPURay*)tinybvh::malloc64( rayCount * sizeof( GPURay ) );
	shadow = (GPURay*)tinybvh::malloc64( rayCount * sizeof( GPURay ) );
	shadowSrc = (uint32_t*)tinybvh::malloc64( rayCount * 4 );
	primaryBuf = new Buffer( rayCount * (uint32_t)sizeof( GPURay ), primary );
	shadowBuf = new Buffer( rayCount * (uint32_t)sizeof( GPURay ), shadow );
}

bool UpdateCamera( float delta_time_s, fenster& f )
{
	bvhvec3 right = tinybvh_normalize( tinybvh_cross( bvhvec3( 0, 1, 0 ), view ) );
	bvhvec3 up = 0.8f * tinybvh_cross( view, right );
	float moved = 0, spd = 10.0f * delta_time_s;
	if (f.keys['A'] || f.keys['D']) eye += right * (f.keys['D'] ? spd : -spd), moved = 1;
	if (f.keys['W'] || f.keys['S']) eye += view * (f.keys['W'] ? spd : -spd), moved = 1;
	if (f.keys['R'] || f.keys['F']) eye += up * 2.0f * (f.keys['R'] ? spd : -spd), moved = 1;
	if (f.keys[20]) view = tinybvh_normalize( view + right * -0.1f * spd ), moved = 1;
	if (f.keys[19]) view = tinybvh_normalize( view + right * 0.1f * spd ), moved = 1;
	if (f.keys[17]) view = tinybvh_normalize( view + up * -0.1f * spd ), moved = 1;
	if (f.keys[18]) view = tinybvh_normalize( view + up * 0.1f * spd ), moved = 1;
	// recalculate right, up
	right = tinybvh_normalize( tinybvh_cross( bvhvec3( 0, 1, 0 ), view ) );
	up = 0.8f * tinybvh_cross( view, right );
	bvhvec3 C = eye + 1.2f * view;
	p1 = C - right + up, p2 = C + right + up, p3 = C - right - up;
	return moved > 0;
}

// AnimateInstances: spin the bunnies so the TLAS has to be rebuilt and
// re-uploaded every frame, which is the case this demo is meant to exercise.
void AnimateInstances( float t )
{
	int idx = 0;
	for (int x = 0; x < GRIDSIZE; x++) for (int y = 0; y < GRIDSIZE; y++)
		for (int z = 0; z < GRIDSIZE; z++, idx++)
		{
			const float a = t + (float)idx * 0.37f, c = cosf( a ), s = sinf( a );
			bvhmat4& T = inst[idx].transform;
			T[0] = 0.6f * c, T[1] = 0, T[2] = 0.6f * s;
			T[4] = 0, T[5] = 0.6f, T[6] = 0;
			T[8] = -0.6f * s, T[9] = 0, T[10] = 0.6f * c;
			T[3] = (float)x * 5 - GRIDSIZE * 2.5f;
			T[7] = (float)y * 5 - GRIDSIZE * 2.5f + 7 + sinf( a ) * 0.5f;
			T[11] = (float)z * 5 - GRIDSIZE * 2.5f + 1;
		}
}

// PrimaryRayThread: fills one tile of the primary ray batch.
void PrimaryRayThread()
{
	const int xtiles = SCRWIDTH / TILESIZE, ytiles = SCRHEIGHT / TILESIZE;
	const int tiles = xtiles * ytiles;
	int tile = tileIdx++;
	while (tile < tiles)
	{
		const int tx = tile % xtiles, ty = tile / xtiles;
		for (int y = 0; y < TILESIZE; y++) for (int x = 0; x < TILESIZE; x++)
		{
			const int px = tx * TILESIZE + x, py = ty * TILESIZE + y;
			const float u = (float)px / SCRWIDTH, v = (float)py / SCRHEIGHT;
			const bvhvec3 D = tinybvh_normalize( p1 + u * (p2 - p1) + v * (p3 - p1) - eye );
			primary[px + py * SCRWIDTH] = GPURay( eye, D );
		}
		tile = tileIdx++;
	}
}

// RunThreads: runs 'fn' on all cores, or serially in a debug build.
template <typename F> void RunThreads( const F& fn )
{
	tileIdx = 0;
#ifdef _DEBUG
	fn();
#else
	std::vector<std::thread> threads;
	for (uint32_t i = 0; i < threadCount; i++) threads.emplace_back( fn );
	for (auto& thread : threads) thread.join();
#endif
}

// Normal at a hit: both BLASses hold triangles, so the geometric normal comes
// straight from the vertex data of the mesh the instance refers to.
bvhvec3 HitNormal( uint32_t instIdx, uint32_t primIdx, const bvhvec3& D )
{
	const BLASInstance& instance = inst[instIdx];
	const bvhvec4* mesh = instance.blasIdx == 0 ? bunny : triangles;
	const bvhvec3 v0 = mesh[primIdx * 3], v1 = mesh[primIdx * 3 + 1], v2 = mesh[primIdx * 3 + 2];
	bvhvec3 N = tinybvh_normalize( tinybvh_cross( v1 - v0, v2 - v0 ) );
	N = tinybvh_normalize( tinybvh_transform_vector( N, instance.transform ) );
	if (tinybvh_dot( N, D ) > 0) N = -N;
	return N;
}

#if VERIFY
// Verify: trace the same batch on the CPU and report any disagreement.
void Verify()
{
	uint32_t mismatches = 0;
	for (uint32_t i = 0; i < SCRWIDTH * SCRHEIGHT; i++)
	{
		Ray ray( bvhvec3( primary[i].O ), bvhvec3( primary[i].D ) );
		tlas.Intersect( ray );
		const float gpuT = primary[i].hit.x;
		const bool gpuHit = gpuT < BVH_FAR, cpuHit = ray.hit.t < BVH_FAR;
		bool ok = gpuHit == cpuHit;
		if (ok && gpuHit)
		{
			ok = fabsf( gpuT - ray.hit.t ) < 1e-3f * tinybvh_max( 1.0f, ray.hit.t ) &&
				tinybvh_as_uint( primary[i].hit.w ) == ray.hit.prim;
		}
		if (!ok && mismatches++ < 8)
			printf( "pixel %u: gpu t %.5f prim %08x, cpu t %.5f prim %08x\n", i,
				gpuT, tinybvh_as_uint( primary[i].hit.w ), ray.hit.t, ray.hit.prim );
	}
	printf( "verification: %u mismatches out of %u rays.\n", mismatches,
		(uint32_t)(SCRWIDTH * SCRHEIGHT) );
}
#endif

void Tick( float delta_time_s, fenster& f, uint32_t* buf )
{
	UpdateCamera( delta_time_s, f );

	// animate the instances and rebuild the TLAS over them
	AnimateInstances( (float)frameIdx * 0.02f );
	tlas.Build( inst, 1 + INSTCOUNT, blasList, 2 );
	gpuScene.SyncTLAS();

	// generate and trace the primary rays
	const uint32_t rayCount = SCRWIDTH * SCRHEIGHT;
	RunThreads( PrimaryRayThread );
	primaryBuf->CopyToDevice();
	gpuScene.SetArgs( *nearestKernel, primaryBuf, rayCount );
	nearestKernel->Run( rayCount, 64 );
	primaryBuf->CopyFromDevice();

#if VERIFY
	if (frameIdx == 0) Verify();
#endif

	// build one shadow ray per hit; a compacted batch keeps every work item busy
	shadowCount = 0;
	for (uint32_t i = 0; i < rayCount; i++)
	{
		if (primary[i].hit.x >= BVH_FAR) { buf[i] = 0x2b2b40; continue; }
		const bvhvec3 O = bvhvec3( primary[i].O ) + primary[i].hit.x * bvhvec3( primary[i].D );
		bvhvec3 L = lightPos - O;
		const float d = tinybvh_length( L );
		L *= 1.0f / d;
		shadow[shadowCount] = GPURay( O + L * 0.001f, L, d - 0.002f );
		shadowSrc[shadowCount++] = i;
	}
	if (shadowCount > 0)
	{
		shadowBuf->CopyToDevice( 0, shadowCount * (int)sizeof( GPURay ) );
		gpuScene.SetArgs( *anyKernel, shadowBuf, shadowCount );
		anyKernel->Run( (shadowCount + 63) & ~63u, 64 );
		shadowBuf->CopyFromDevice( 0, shadowCount * (int)sizeof( GPURay ) );
	}

	// shade
	for (uint32_t i = 0; i < shadowCount; i++)
	{
		const uint32_t pixelIdx = shadowSrc[i];
		const uint32_t packed = tinybvh_as_uint( primary[pixelIdx].hit.w );
		const uint32_t primIdx = packed & PRIM_IDX_MASK;
		const uint32_t instIdx = packed >> INST_IDX_SHFT;
		const bvhvec3 O = bvhvec3( primary[pixelIdx].O );
		const bvhvec3 D = bvhvec3( primary[pixelIdx].D );
		const bvhvec3 N = HitNormal( instIdx, primIdx, D );
		const bvhvec3 L = tinybvh_normalize( lightPos - (O + primary[pixelIdx].hit.x * D) );
		const bool occluded = tinybvh_as_uint( shadow[i].hit.w ) != 0;
		const int c = (int)((occluded ? 50.0f : 255.9f) *
			tinybvh_max( 0.05f, tinybvh_dot( N, L ) ));
		buf[pixelIdx] = c + (c << 8) + (c << 16);
	}

	// report frame rate and ray throughput
	char title[128];
	const float rays = (float)(rayCount + shadowCount) * 1e-6f;
	const float mrays = rays / tinybvh_max( 1e-6f, delta_time_s );
	sprintf( title, "tiny_bvh_tlas_gpu - %.2f Hz - %.1f Mrays/s (%u primary, %u shadow)",
		1.0f / delta_time_s, mrays, rayCount, shadowCount );
	fenster_update_title( &f, title );
	frameIdx++;
}

void Shutdown()
{
	gpuScene.Clear();
	delete nearestKernel, delete anyKernel;
	delete primaryBuf, delete shadowBuf;
	tinybvh::free64( primary ), tinybvh::free64( shadow ), tinybvh::free64( shadowSrc );
	tinybvh::free64( triangles ), tinybvh::free64( bunny );
}
