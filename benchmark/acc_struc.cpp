#include "headers.h"

namespace tinybvh
{

extern bvh::v2::ThreadPool thread_pool;
extern bvh::v2::ParallelExecutor executor;
extern RTCScene embreeScene;
extern RTCDevice embreeDevice;
extern RTCGeometry embreeGeom;

BVHContext context; // use a default context for the threading hooks.

AccStruc::AccStruc( BVHLayout bvhLayout, BuildFlags bvhFlags )
{
	layout = bvhLayout;
	flags = bvhFlags;
	// Note: BVH_Verbose, BVH_Double and VoxelSet currently do not make sense.
	if (layout == BVH2) bvh = new BVH();
	if (layout == BVH4) bvh = new MBVH<4>();
	if (layout == BVH4_WIVE) bvh = new BVH4_CPU();
	if (layout == BVH8_WIVE) bvh = new BVH8_CPU();
	if (layout == GPU_BVH) bvh = new BVH_GPU();
	if (layout == GPU_BVH4) bvh = new BVH4_GPU();
	if (layout == CWBVH) bvh = new BVH8_CWBVH();
	// set specified flags
	if (bvh)
	{
		bvh->settings.usePresplitting = false;
		bvh->settings.useSpatialSplits = false;
		bvh->settings.presplitPostPass = false;
		bvh->settings.useFullSweep = false;
		bvh->settings.useSIMDifavailable = false;
		if (flags & BuildFlags::FULLSWEEP) bvh->settings.useFullSweep = true;
		if (flags & BuildFlags::PRESPLIT) bvh->settings.usePresplitting = true;
		if (flags & BuildFlags::AVXBUILD) bvh->settings.useSIMDifavailable = true;
		if (flags & BuildFlags::SPATIALSPLITS) bvh->settings.useSpatialSplits = true;
	}
	// construct description
	switch (layout)
	{
	case BVH2: strncpy( desc, "2-wide BVH", 256 ); break;
	case BVH4: strncpy( desc, "4-wide BVH", 256 ); break;
	case BVH4_WIVE: strncpy( desc, "SSE 4-wide CPU BVH", 256 ); break;
	case BVH8_WIVE: strncpy( desc, "AVX2 8-wide CPU BVH", 256 ); break;
	case GPU_BVH: strncpy( desc, "2-wide GPU BVH", 256 ); break;
	case GPU_BVH4: strncpy( desc, "4-wide GPU BVH", 256 ); break;
	case CWBVH: strncpy( desc, "8-wide CWBVH", 256 ); break;
	case MADMANN91: strncpy( desc, "madmann91 BVH", 256 ); break;
	case EMBREE: strncpy( desc, "embree4 BVH", 256 ); break;
	default: strncpy( desc, "UNKNOWN LAYOUT", 256 );
	}
	uint32_t f = flags;
	bool first = true;
	while (f != BuildFlags::NO_FLAGS)
	{
		if (first) strncat( desc, " (", 128 ); else strncat( desc, " + ", 128 );
		if (f & BuildFlags::AVXBUILD) { strncat( desc, "AVX builder", 128 ); f -= BuildFlags::AVXBUILD; }
		else if (f & BuildFlags::INDEXED) { strncat( desc, "indexed", 128 ); f -= BuildFlags::INDEXED; }
		else if (f & BuildFlags::FULLSWEEP) { strncat( desc, "full-sweep", 128 ); f -= BuildFlags::FULLSWEEP; }
		else if (f & BuildFlags::SPATIALSPLITS) { strncat( desc, "SBVH", 128 ); f -= BuildFlags::SPATIALSPLITS; }
		else if (f & BuildFlags::PRESPLIT) { strncat( desc, "presplit", 128 ); f -= BuildFlags::PRESPLIT; }
		else if (f & BuildFlags::OPTIMIZE) { strncat( desc, "optimize", 128 ); f -= BuildFlags::OPTIMIZE; }
		else if (f & BuildFlags::LOW) { strncat( desc, "LOW", 128 ); f -= BuildFlags::LOW; }
		else if (f & BuildFlags::MEDIUM) { strncat( desc, "MEDIUM", 128 ); f -= BuildFlags::MEDIUM; }
		else if (f & BuildFlags::HIGH) { strncat( desc, "HIGH", 128 ); f -= BuildFlags::HIGH; }
		first = false;
	}
	if (flags) strncat( desc, ")", 128 );
}

BVHBase* AccStruc::Build( PrimitiveSet* prims )
{
	primSet = prims;
	if (bvh)
	{
		if (flags & BuildFlags::SPATIALSPLITS) bvh->settings.useSpatialSplits = true;
		if (flags & BuildFlags::AVXBUILD) bvh->settings.useSIMDifavailable = true;
		if (flags & BuildFlags::PRESPLIT) bvh->settings.usePresplitting = true;
		if (flags & BuildFlags::FULLSWEEP) bvh->settings.useFullSweep = true;
		if (flags & BuildFlags::OPTIMIZE) bvh->settings.postOptimize = true, bvh->settings.optimizeIterations = 50;
	}
	switch (layout)
	{
	case BVH2:
	{
		BVH* accstruc = (BVH*)bvh;
		if (flags & INDEXED) accstruc->Build( primSet->verts, primSet->indices, primSet->primCount );
		else accstruc->Build( primSet->verts, primSet->primCount );
		break;
	}
#if 0
	case BVH4:
	{
		MBVH<4>* accstruc = (MBVH<4>*)bvh;
		if (flags & INDEXED) accstruc->Build( primSet->verts, primSet->indices, primSet->primCount );
		else accstruc->Build( primSet->verts, primSet->primCount );
		break;
	}
#endif
	case BVH4_WIVE:
	{
		BVH4_CPU* accstruc = (BVH4_CPU*)bvh;
		if (flags & INDEXED) accstruc->Build( primSet->verts, primSet->indices, primSet->primCount );
		else accstruc->Build( primSet->verts, primSet->primCount );
		break;
	}
	case BVH8_WIVE:
	{
		BVH8_CPU* accstruc = (BVH8_CPU*)bvh;
		if (flags & INDEXED) accstruc->Build( primSet->verts, primSet->indices, primSet->primCount );
		else accstruc->Build( primSet->verts, primSet->primCount );
		break;
	}
	case GPU_BVH:
	{
		BVH_GPU* accstruc = (BVH_GPU*)bvh;
		if (flags & INDEXED) accstruc->Build( primSet->verts, primSet->indices, primSet->primCount );
		else accstruc->Build( primSet->verts, primSet->primCount );
		break;
	}
	case GPU_BVH4:
	{
		BVH4_GPU* accstruc = (BVH4_GPU*)bvh;
		if (flags & INDEXED) accstruc->Build( primSet->verts, primSet->indices, primSet->primCount );
		else accstruc->Build( primSet->verts, primSet->primCount );
		break;
	}
	case CWBVH:
	{
		BVH8_CWBVH* accstruc = (BVH8_CWBVH*)bvh;
		if (flags & INDEXED) accstruc->Build( primSet->verts, primSet->indices, primSet->primCount );
		else accstruc->Build( primSet->verts, primSet->primCount );
		break;
	}
	case MADMANN91:
	{
		bvh::v2::ThreadPool thread_pool;
		bvh::v2::ParallelExecutor executor( thread_pool );
		typename bvh::v2::DefaultBuilder<_Node>::Config config;
		// abuse tinybvh flags to steer madmann91 build speed / quality tradeoff.
		if (flags & MEDIUM) config.quality = bvh::v2::DefaultBuilder<_Node>::Quality::Medium;
		else if (flags & HIGH) config.quality = bvh::v2::DefaultBuilder<_Node>::Quality::High;
		else config.quality = bvh::v2::DefaultBuilder<_Node>::Quality::Low;
		madmannbvh = bvh::v2::DefaultBuilder<_Node>::build( thread_pool, primSet->bboxes, primSet->centers, config );
		// precompute tris if not done yet
		if (!mmTrisPrecomputed)
		{
			std::vector<size_t>& prim_ids = madmannbvh.prim_ids;
			precomputed_tris.resize( primSet->tris.size() );
			executor.for_each( 0, primSet->tris.size(), [&]( size_t begin, size_t end )
				{
					for (size_t i = begin; i < end; ++i) precomputed_tris[i] = primSet->tris[prim_ids[i]];
				} );
			mmTrisPrecomputed = true;
		}
		break;
	}
	case EMBREE:
	{
		RTCBuildQuality q = RTC_BUILD_QUALITY_LOW;
		if (flags & MEDIUM) q = RTC_BUILD_QUALITY_MEDIUM;
		if (flags & HIGH) q = RTC_BUILD_QUALITY_HIGH;
		rtcSetGeometryBuildQuality( embreeGeom, q );
		rtcCommitGeometry( embreeGeom );
		rtcAttachGeometry( embreeScene, embreeGeom );
		rtcReleaseGeometry( embreeGeom );
		rtcSetSceneBuildQuality( embreeScene, q );
		rtcCommitScene( embreeScene );
		break;
	}
	default:
		exit( 0 ); // unsupported layout. See note in constructor.
	};
	return bvh;
}

float AccStruc::SAHCost()
{
	switch (layout)
	{
	case BVH2: return ((BVH*)bvh)->SAHCost();
	case BVH4_WIVE: return ((BVH4_CPU*)bvh)->bvh4.SAHCost();
	case BVH8_WIVE: return ((BVH8_CPU*)bvh)->bvh8.SAHCost();
	case GPU_BVH:return ((BVH_GPU*)bvh)->SAHCost();
	case GPU_BVH4: return ((BVH4_GPU*)bvh)->SAHCost();
	case CWBVH: return ((BVH8_CWBVH*)bvh)->SAHCost();
	case MADMANN91:
	{
		// directly walk the madmann91 bvh to calculate its SAH cost.
		uint32_t nodeIdx = 0, stack[64], stackPtr = 0;
		const _Node& root = madmannbvh.nodes[0];
		float cost = 0;
		while (1)
		{
			_Node& node = madmannbvh.nodes[nodeIdx];
			const bvhvec3 bmin( node.bounds[0], node.bounds[2], node.bounds[4] );
			const bvhvec3 bmax( node.bounds[1], node.bounds[3], node.bounds[5] );
			if (node.is_leaf())
			{
				cost += C_INT * tinybvh_halfarea( bmax - bmin ) * node.index.prim_count();
				if (!stackPtr) break; else nodeIdx = stack[--stackPtr];
			}
			else
			{
				cost += C_TRAV * tinybvh_halfarea( bmax - bmin );
				uint32_t childId = node.index.first_id();
				stack[stackPtr++] = childId + 1, nodeIdx = childId;
			}
		}
		const bvhvec3 rbmin( root.bounds[0], root.bounds[2], root.bounds[4] );
		const bvhvec3 rbmax( root.bounds[1], root.bounds[3], root.bounds[5] );
		return cost / tinybvh_halfarea( rbmax - rbmin );
	}
	case EMBREE:
	{
		return 0; // TODO; see kernels/bvh/bvh_statistics.h
	}
	default: return -1; // unsupported layout.
	}
}

float AccStruc::EPOCost()
{
	switch (layout)
	{
	case BVH2: return ((BVH*)bvh)->EPOCost();
	case BVH4_WIVE: return ((BVH4_CPU*)bvh)->bvh4.bvh.EPOCost();
	case BVH8_WIVE: return ((BVH8_CPU*)bvh)->bvh8.bvh.EPOCost();
	case GPU_BVH:return ((BVH_GPU*)bvh)->bvh.EPOCost();
	case GPU_BVH4: return ((BVH4_GPU*)bvh)->bvh4.bvh.EPOCost();
	case CWBVH: return ((BVH8_CWBVH*)bvh)->bvh8.bvh.EPOCost();
	default: return 0; // unsupported layout. See note in constructor.
	}
	// note: there is (intentionally) no full class derivation in tinybvh.
	// We thus have to specialize here for each possible layout.
}

struct BatchIntersectArgs { AccStruc* accstruc; char* rayData; int rayCount; int slices; int sliceSize; };
static void IntersectBatchSlice( uint32_t i, void* payload )
{
	BatchIntersectArgs* a = (BatchIntersectArgs*)payload;
	int size = a->sliceSize;
	if (i == a->slices - 1) size = a->rayCount - (a->slices - 1) * a->sliceSize;
	a->accstruc->IntersectBatch( a->rayData + a->sliceSize * i * 64, size );
}
void AccStruc::IntersectBatchMT( char* rayData, int rayCount )
{
	int slices = std::thread::hardware_concurrency() * 4;
	int sliceSize = rayCount / slices;
	BatchIntersectArgs args = { this, rayData, rayCount, slices, sliceSize };
	tinybvh_parallel_for( context, slices, &IntersectBatchSlice, &args );
}

float AccStruc::IntersectBatch( char* rayData, int rayCount )
{
	float origDist = ((Ray*)rayData)[0].hit.t;
	float dist = origDist;
	switch (layout)
	{
	case BVH2:
	{
		BVH* accstruc = (BVH*)bvh;
		if (rayCount == 1)
		{
			accstruc->Intersect( ((Ray*)rayData)[0] );
			dist = ((Ray*)rayData)[0].hit.t, ((Ray*)rayData)[0].hit.t = origDist; // reset
		}
		else for (int i = 0; i < rayCount; i++, rayData += 64)
		{
			accstruc->Intersect( ((Ray*)rayData)[0] );
			((Ray*)rayData)[0].hit.t = origDist; // reset
		}
		break;
	}
	case BVH4_WIVE:
	{
		BVH4_CPU* accstruc = (BVH4_CPU*)bvh;
		if (rayCount == 1)
		{
			accstruc->Intersect( ((Ray*)rayData)[0] );
			dist = ((Ray*)rayData)[0].hit.t, ((Ray*)rayData)[0].hit.t = origDist; // reset
		}
		else for (int i = 0; i < rayCount; i++, rayData += 64)
		{
			accstruc->Intersect( ((Ray*)rayData)[0] );
			((Ray*)rayData)[0].hit.t = origDist; // reset
		}
		break;
	}
	case BVH8_WIVE:
	{
		BVH8_CPU* accstruc = (BVH8_CPU*)bvh;
		if (rayCount == 1)
		{
			accstruc->Intersect( ((Ray*)rayData)[0] );
			dist = ((Ray*)rayData)[0].hit.t, ((Ray*)rayData)[0].hit.t = origDist; // reset
		}
		else for (int i = 0; i < rayCount; i++, rayData += 64)
		{
			accstruc->Intersect( ((Ray*)rayData)[0] );
			((Ray*)rayData)[0].hit.t = origDist; // reset
		}
		break;
	}
	case GPU_BVH:
	{
		BVH_GPU* accstruc = (BVH_GPU*)bvh;
		if (rayCount == 1)
		{
			accstruc->Intersect( ((Ray*)rayData)[0] );
			dist = ((Ray*)rayData)[0].hit.t, ((Ray*)rayData)[0].hit.t = origDist; // reset
		}
		else for (int i = 0; i < rayCount; i++, rayData += 64)
		{
			accstruc->Intersect( ((Ray*)rayData)[0] );
			((Ray*)rayData)[0].hit.t = origDist; // reset
		}
		break;
	}
	case GPU_BVH4:
	{
		BVH4_GPU* accstruc = (BVH4_GPU*)bvh;
		if (rayCount == 1)
		{
			accstruc->Intersect( ((Ray*)rayData)[0] );
			dist = ((Ray*)rayData)[0].hit.t, ((Ray*)rayData)[0].hit.t = origDist; // reset
		}
		else for (int i = 0; i < rayCount; i++, rayData += 64)
		{
			accstruc->Intersect( ((Ray*)rayData)[0] );
			((Ray*)rayData)[0].hit.t = origDist; // reset
		}
		break;
	}
	case CWBVH:
	{
		BVH8_CWBVH* accstruc = (BVH8_CWBVH*)bvh;
		if (rayCount == 1)
		{
			accstruc->Intersect( ((Ray*)rayData)[0] );
			dist = ((Ray*)rayData)[0].hit.t, ((Ray*)rayData)[0].hit.t = origDist; // reset
		}
		else for (int i = 0; i < rayCount; i++, rayData += 64)
		{
			accstruc->Intersect( ((Ray*)rayData)[0] );
			((Ray*)rayData)[0].hit.t = origDist; // reset
		}
		break;
	}
	case MADMANN91:
	{
		// for this experiment we use ideal circumstances for the Manmann91 library:
		// - Precomputed triangle data (Woop?), precomputation not included in timing;
		// - The fast traversal path (rather than the robust one).
		static constexpr size_t invalid_id = 9999999;
		bvh::v2::SmallStack<_Bvh::Index, 64> stack;
		float u, v;
		std::optional<std::tuple<float,float,float>> hit;
		for (int i = 0; i < rayCount; i++)
		{
			const Ray& r = *(Ray*)(rayData + i * 64);
			size_t prim_id = invalid_id;
			_Ray ray( _Vec3( r.O.x, r.O.y, r.O.z ), _Vec3( r.D.x, r.D.y, r.D.z ) );
			madmannbvh.intersect<false, false>( ray, madmannbvh.get_root().index, stack, [&]( size_t begin, size_t end )
				{
					for (size_t i = begin; i < end; ++i)
						if (hit = precomputed_tris[i].intersect( ray )) prim_id = i, std::tie( ray.tmax, u, v ) = *hit;
					return prim_id != invalid_id;
				} );
		}
		dist = hit.has_value() ? get<0>(hit.value()) : 1e30f;
		break;
	}
	case EMBREE:
	{
		RTCRayHit embreeRay;
		for (int i = 0; i < rayCount; i++)
		{
			const Ray& r = *(Ray*)(rayData + i * 64);
			embreeRay.ray.org_x = r.O.x, embreeRay.ray.org_y = r.O.y, embreeRay.ray.org_z = r.O.z;
			embreeRay.ray.dir_x = r.D.x, embreeRay.ray.dir_y = r.D.y, embreeRay.ray.dir_z = r.D.z;
			embreeRay.ray.tnear = 0, embreeRay.ray.tfar = r.hit.t;
			embreeRay.ray.mask = -1, embreeRay.ray.flags = 0;
			embreeRay.hit.geomID = RTC_INVALID_GEOMETRY_ID;
			embreeRay.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
			rtcIntersect1( embreeScene, &embreeRay );
		}
		dist = embreeRay.ray.tfar;
		break;
	}
	default: // unsupported layout. See note in constructor.
		break;
	}
	return dist;
}

struct BatchOcclusionArgs { AccStruc* accstruc; char* rayData; int rayCount; int sliceSize; };
static void OcclusionBatchSlice( uint32_t i, void* payload )
{
	BatchOcclusionArgs* a = (BatchOcclusionArgs*)payload;
	a->accstruc->OcclusionBatch( a->rayData + a->sliceSize * i * 64, a->sliceSize );
}
void AccStruc::OcclusionBatchMT( char* rayData, int rayCount )
{
	constexpr int slices = 64;
	int sliceSize = rayCount / slices;
	BatchIntersectArgs args = { this, rayData, rayCount, sliceSize };
	tinybvh_parallel_for( context, slices, &OcclusionBatchSlice, &args );
}

void AccStruc::OcclusionBatch( char* rayData, int rayCount )
{
	switch (layout)
	{
	case BVH2:
	{
		BVH* accstruc = (BVH*)bvh;
		for (int i = 0; i < rayCount; i++, rayData += 64) accstruc->IsOccluded( ((Ray*)rayData)[0] );
		break;
	}
	case BVH4_WIVE:
	{
		BVH4_CPU* accstruc = (BVH4_CPU*)bvh;
		for (int i = 0; i < rayCount; i++, rayData += 64) accstruc->IsOccluded( ((Ray*)rayData)[0] );
		break;
	}
	case BVH8_WIVE:
	{
		BVH8_CPU* accstruc = (BVH8_CPU*)bvh;
		for (int i = 0; i < rayCount; i++, rayData += 64) accstruc->IsOccluded( ((Ray*)rayData)[0] );
		break;
	}
	case GPU_BVH:
	{
		BVH_GPU* accstruc = (BVH_GPU*)bvh;
		for (int i = 0; i < rayCount; i++, rayData += 64) accstruc->IsOccluded( ((Ray*)rayData)[0] );
		break;
	}
	case GPU_BVH4:
	{
		BVH4_GPU* accstruc = (BVH4_GPU*)bvh;
		for (int i = 0; i < rayCount; i++, rayData += 64) accstruc->IsOccluded( ((Ray*)rayData)[0] );
		break;
	}
	case CWBVH:
	{
		BVH8_CWBVH* accstruc = (BVH8_CWBVH*)bvh;
		for (int i = 0; i < rayCount; i++, rayData += 64) accstruc->IsOccluded( ((Ray*)rayData)[0] );
		break;
	}
	case MADMANN91:
	{
		// for this experiment we use ideal circumstances for the Manmann91 library:
		// - Precomputed triangle data (Woop?), precomputation not included in timing;
		// - The fast traversal path (rather than the robust one);
		// - For shadow rays we use the AnyHit code.
		// NOTE: for now we skip this test as a stack overflow is triggered.
		bvh::v2::SmallStack<_Bvh::Index, 64> stack;
		for (int i = 0; i < rayCount; i++)
		{
			const Ray& r = *(Ray*)(rayData + 64 * i);
			_Ray ray = { _Vec3( r.O.x, r.O.y, r.O.z ), _Vec3( r.D.x, r.D.y, r.D.z ), 0, r.hit.t };
			madmannbvh.intersect<true, false>( ray, madmannbvh.get_root().index, stack, [&]( size_t begin, size_t end )
				{
					for (size_t i = begin; i < end; ++i)
						if (precomputed_tris[i].intersect( ray )) return false;
					return true;
				} );
		}
		break;
	}
	case EMBREE:
	{
		RTCRay embreeRay;
		for (int i = 0; i < rayCount; i++)
		{
			const Ray& r = *(Ray*)(rayData + 64 * i);
			embreeRay.org_x = r.O.x, embreeRay.org_y = r.O.y, embreeRay.org_z = r.O.z;
			embreeRay.dir_x = r.D.x, embreeRay.dir_y = r.D.y, embreeRay.dir_z = r.D.z;
			embreeRay.tnear = 0, embreeRay.tfar = r.hit.t;
			embreeRay.mask = -1, embreeRay.flags = RTC_RAY_QUERY_FLAG_COHERENT;
			rtcOccluded1( embreeScene, &embreeRay );
		}
		break;
	}
	default: // unsupported layout. See note in constructor.
		break;
	}
}

bvhvec3 AccStruc::SceneExtent()
{ 
	if (layout == EMBREE || layout == MADMANN91)
	{
		// hacky way to get scene extents, via a TinyBVH bvh.
		BVH bvh;
		if (flags & INDEXED) bvh.Build( primSet->verts, primSet->indices, primSet->primCount );
		else bvh.Build( primSet->verts, primSet->primCount );
		bvhvec3 extent = bvh.aabbMax - bvh.aabbMin;
		return extent;
	}
	else return bvh->aabbMax - bvh->aabbMin;
}

}; // namespace tinybvh