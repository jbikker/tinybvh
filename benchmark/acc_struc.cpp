#include "headers.h"

namespace tinybvh
{

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
	default: strncpy( desc, "UNKNOWN LAYOUT", 256 );
	}
	uint32_t f = flags;
	bool first = true;
	while (f != BuildFlags::NO_FLAGS)
	{
		if (first) strncat( desc, " (", 128 ); else strncat( desc, " + ", 128 );
		if (f & BuildFlags::AVXBUILD) { strncat( desc, "AVX builder", 128 ); f -= BuildFlags::AVXBUILD; }
		else if (f & BuildFlags::FULLSWEEP) { strncat( desc, "full-sweep", 128 ); f -= BuildFlags::FULLSWEEP; }
		else if (f & BuildFlags::SPATIALSPLITS) { strncat( desc, "SBVH", 128 ); f -= BuildFlags::SPATIALSPLITS; }
		else if (f & BuildFlags::PRESPLIT) { strncat( desc, "presplit", 128 ); f -= BuildFlags::PRESPLIT; }
		else if (f & BuildFlags::OPTIMIZE) { strncat( desc, "optimize", 128 ); f -= BuildFlags::OPTIMIZE; }
		first = false;
	}
	if (flags) strncat( desc, ")", 128 );
}

BVHBase* AccStruc::Build( PrimitiveSet* primSet )
{
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
		accstruc->Build( primSet->verts, primSet->primCount );
		break;
	}
#if 0
	case BVH4:
	{
		MBVH<4>* accstruc = (MBVH<4>*)bvh;
		accstruc->Build( primSet->verts, primSet->primCount );
		break;
	}
#endif
	case BVH4_WIVE:
	{
		BVH4_CPU* accstruc = (BVH4_CPU*)bvh;
		accstruc->Build( primSet->verts, primSet->primCount );
		break;
	}
	case BVH8_WIVE:
	{
		BVH8_CPU* accstruc = (BVH8_CPU*)bvh;
		accstruc->Build( primSet->verts, primSet->primCount );
		break;
	}
	case GPU_BVH:
	{
		BVH_GPU* accstruc = (BVH_GPU*)bvh;
		accstruc->Build( primSet->verts, primSet->primCount );
		break;
	}
	case GPU_BVH4:
	{
		BVH4_GPU* accstruc = (BVH4_GPU*)bvh;
		accstruc->Build( primSet->verts, primSet->primCount );
		break;
	}
	case CWBVH:
	{
		BVH8_CWBVH* accstruc = (BVH8_CWBVH*)bvh;
		accstruc->Build( primSet->verts, primSet->primCount );
		break;
	}
	case MADMANN91:
	{
		bvh::v2::ThreadPool thread_pool;
		bvh::v2::ParallelExecutor executor( thread_pool );
		typename bvh::v2::DefaultBuilder<_Node>::Config config;
		// abuse tinybvh flags to steer madmann91 build speed / quality tradeoff.
		if (flags & OPTIMIZE) config.quality = bvh::v2::DefaultBuilder<_Node>::Quality::Medium;
		else if (flags & SPATIALSPLITS) config.quality = bvh::v2::DefaultBuilder<_Node>::Quality::High;
		else config.quality = bvh::v2::DefaultBuilder<_Node>::Quality::Low;
		madmannbvh = bvh::v2::DefaultBuilder<_Node>::build( thread_pool, primSet->bboxes, primSet->centers, config );
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

void AccStruc::IntersectBatch( Ray* raySet, int rayCount )
{
	switch (layout)
	{
	case BVH2:
	{
		BVH* accstruc = (BVH*)bvh;
		for (int i = 0; i < rayCount; i++) accstruc->Intersect( raySet[i] );
		break;
	}
	case BVH4_WIVE:
	{
		BVH4_CPU* accstruc = (BVH4_CPU*)bvh;
		for (int i = 0; i < rayCount; i++) accstruc->Intersect( raySet[i] );
		break;
	}
	case BVH8_WIVE:
	{
		BVH8_CPU* accstruc = (BVH8_CPU*)bvh;
		for (int i = 0; i < rayCount; i++) accstruc->Intersect( raySet[i] );
		break;
	}
	case GPU_BVH:
	{
		BVH_GPU* accstruc = (BVH_GPU*)bvh;
		for (int i = 0; i < rayCount; i++) accstruc->Intersect( raySet[i] );
		break;
	}
	case GPU_BVH4:
	{
		BVH4_GPU* accstruc = (BVH4_GPU*)bvh;
		for (int i = 0; i < rayCount; i++) accstruc->Intersect( raySet[i] );
		break;
	}
	case CWBVH:
	{
		BVH8_CWBVH* accstruc = (BVH8_CWBVH*)bvh;
		for (int i = 0; i < rayCount; i++) accstruc->Intersect( raySet[i] );
		break;
	}
	default: // unsupported layout. See note in constructor.
		break;
	}
}

void AccStruc::OcclusionBatch( Ray* raySet, int rayCount )
{
	switch (layout)
	{
	case BVH2:
	{
		BVH* accstruc = (BVH*)bvh;
		for (int i = 0; i < rayCount; i++) accstruc->IsOccluded( raySet[i] );
		break;
	}
	case BVH4_WIVE:
	{
		BVH4_CPU* accstruc = (BVH4_CPU*)bvh;
		for (int i = 0; i < rayCount; i++) accstruc->IsOccluded( raySet[i] );
		break;
	}
	case BVH8_WIVE:
	{
		BVH8_CPU* accstruc = (BVH8_CPU*)bvh;
		for (int i = 0; i < rayCount; i++) accstruc->IsOccluded( raySet[i] );
		break;
	}
	case GPU_BVH:
	{
		BVH_GPU* accstruc = (BVH_GPU*)bvh;
		for (int i = 0; i < rayCount; i++) accstruc->IsOccluded( raySet[i] );
		break;
	}
	case GPU_BVH4:
	{
		BVH4_GPU* accstruc = (BVH4_GPU*)bvh;
		for (int i = 0; i < rayCount; i++) accstruc->IsOccluded( raySet[i] );
		break;
	}
	case CWBVH:
	{
		BVH8_CWBVH* accstruc = (BVH8_CWBVH*)bvh;
		for (int i = 0; i < rayCount; i++) accstruc->IsOccluded( raySet[i] );
		break;
	}
	default: // unsupported layout. See note in constructor.
		break;
	}
}

}; // namespace tinybvh