#include "tiny_bvh.h"

using namespace std;
using namespace tinybvh;

#include "acc_struc.h"
#include "primitive_set.h"

namespace tinybvh
{

AccStruc::AccStruc( BVHBase::BVHType bvhLayout, BuildFlags bvhFlags )
{
	layout = bvhLayout;
	flags = bvhFlags;
	if (layout == BVHBase::BVHType::LAYOUT_BVH)
	{
		bvh = new BVH();
		// reset default flags
		((BVH*)bvh)->settings.usePresplitting = false;
		((BVH*)bvh)->settings.useSpatialSplits = false;
		((BVH*)bvh)->settings.presplitPostPass = false;
		((BVH*)bvh)->settings.useFullSweep = false;
		((BVH*)bvh)->settings.useSIMDifavailable = false;
		// set specified flags
		if (flags & BuildFlags::FULLSWEEP) ((BVH*)bvh)->settings.useFullSweep = true;
		if (flags & BuildFlags::PRESPLIT) ((BVH*)bvh)->settings.usePresplitting = true;
		if (flags & BuildFlags::AVXBUILD) ((BVH*)bvh)->settings.useSIMDifavailable = true;
		if (flags & BuildFlags::SPATIALSPLITS) ((BVH*)bvh)->settings.useSpatialSplits = true;
	}
	// Note: BVH_Verbose, BVH_Double and VoxelSet currently do not make sense.
	if (layout == BVHBase::BVHType::LAYOUT_BVH_SOA) bvh = new BVH_SoA();
	if (layout == BVHBase::BVHType::LAYOUT_BVH_GPU) bvh = new BVH_GPU();
	if (layout == BVHBase::BVHType::LAYOUT_MBVH) bvh = new MBVH<4>();
	if (layout == BVHBase::BVHType::LAYOUT_BVH4_CPU) bvh = new BVH4_CPU();
	if (layout == BVHBase::BVHType::LAYOUT_BVH4_GPU) bvh = new BVH4_GPU();
	if (layout == BVHBase::BVHType::LAYOUT_CWBVH) bvh = new BVH8_CWBVH();
	if (layout == BVHBase::BVHType::LAYOUT_BVH8_AVX2) bvh = new BVH8_CPU();
	// construct description
	switch (layout)
	{
	case BVHBase::BVHType::LAYOUT_BVH: strncpy( desc, "2-wide BVH", 256 ); break;
	case BVHBase::BVHType::LAYOUT_BVH_SOA: strncpy( desc, "2-wide SOA BVH", 256 ); break;
	case BVHBase::BVHType::LAYOUT_BVH_GPU: strncpy( desc, "2-wide GPU BVH", 256 ); break;
	case BVHBase::BVHType::LAYOUT_MBVH: strncpy( desc, "4-wide BVH", 256 ); break;
	case BVHBase::BVHType::LAYOUT_BVH4_CPU: strncpy( desc, "SSE 4-wide CPU BVH", 256 ); break;
	case BVHBase::BVHType::LAYOUT_BVH4_GPU: strncpy( desc, "4-wide GPU BVH", 256 ); break;
	case BVHBase::BVHType::LAYOUT_CWBVH: strncpy( desc, "8-wide CWBVH", 256 ); break;
	case BVHBase::BVHType::LAYOUT_BVH8_AVX2: strncpy( desc, "AVX 8-wide CPU BVH", 256 ); break;
	default: strncpy( desc, "UNKNOWN LAYOUT", 256 );
	}
	uint32_t f = flags;
	bool first = true;
	while (f != BuildFlags::NO_FLAGS)
	{
		if (first) strncat( desc, " (", 256 ); else strncat( desc, " + ", 256 );
		if (f & BuildFlags::AVXBUILD) { strncat( desc, "AVX builder", 256 ); f -= BuildFlags::AVXBUILD; }
		else if (f & BuildFlags::FULLSWEEP) { strncat( desc, "full-sweep", 256 ); f -= BuildFlags::FULLSWEEP; }
		else if (f & BuildFlags::SPATIALSPLITS) { strncat( desc, "SBVH", 256 ); f -= BuildFlags::SPATIALSPLITS; }
		else if (f & BuildFlags::PRESPLIT) { strncat( desc, "presplit", 256 ); f -= BuildFlags::PRESPLIT; }
		first = false;
	}
	if (flags) strncat( desc, ")", 256 );
}

BVHBase* AccStruc::Build( PrimitiveSet* primSet )
{
	switch (layout)
	{
	case BVHBase::BVHType::LAYOUT_BVH:
	{
		BVH* accstruc = (BVH*)bvh;
		if (flags & BuildFlags::SPATIALSPLITS) accstruc->BuildHQ( primSet->verts, primSet->primCount );
		else if (flags & BuildFlags::AVXBUILD) accstruc->BuildAVX( primSet->verts, primSet->primCount );
		else accstruc->Build( primSet->verts, primSet->primCount );
		break;
	}
	case BVHBase::BVHType::LAYOUT_BVH_SOA:
	{
		BVH_SoA* accstruc = (BVH_SoA*)bvh;
		accstruc->Build( primSet->verts, primSet->primCount );
		break;
	}
	case BVHBase::BVHType::LAYOUT_BVH_GPU:
	{
		BVH_GPU* accstruc = (BVH_GPU*)bvh;
		accstruc->Build( primSet->verts, primSet->primCount );
		break;
	}
#if 0
	case BVHBase::BVHType::LAYOUT_MBVH:
	{
		MBVH<4>* accstruc = (MBVH<4>*)bvh;
		accstruc->Build( primSet->verts, primSet->primCount );
		break;
	}
#endif
	case BVHBase::BVHType::LAYOUT_BVH4_CPU:
	{
		BVH4_CPU* accstruc = (BVH4_CPU*)bvh;
		accstruc->Build( primSet->verts, primSet->primCount );
		break;
	}
	case BVHBase::BVHType::LAYOUT_BVH4_GPU:
	{
		BVH4_GPU* accstruc = (BVH4_GPU*)bvh;
		accstruc->Build( primSet->verts, primSet->primCount );
		break;
	}
	case BVHBase::BVHType::LAYOUT_CWBVH:
	{
		BVH8_CWBVH* accstruc = (BVH8_CWBVH*)bvh;
		accstruc->Build( primSet->verts, primSet->primCount );
		break;
	}
	case BVHBase::BVHType::LAYOUT_BVH8_AVX2:
	{
		BVH8_CPU* accstruc = (BVH8_CPU*)bvh;
		accstruc->Build( primSet->verts, primSet->primCount );
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
	case BVHBase::BVHType::LAYOUT_BVH: return ((BVH*)bvh)->SAHCost();
	case BVHBase::BVHType::LAYOUT_BVH_SOA: return ((BVH_SoA*)bvh)->SAHCost();
	case BVHBase::BVHType::LAYOUT_BVH_GPU:return ((BVH_GPU*)bvh)->SAHCost();
	case BVHBase::BVHType::LAYOUT_BVH4_CPU: return ((BVH4_CPU*)bvh)->bvh4.SAHCost();
	case BVHBase::BVHType::LAYOUT_BVH4_GPU: return ((BVH4_GPU*)bvh)->SAHCost();
	case BVHBase::BVHType::LAYOUT_CWBVH: return ((BVH8_CWBVH*)bvh)->SAHCost();
	case BVHBase::BVHType::LAYOUT_BVH8_AVX2: return ((BVH8_CPU*)bvh)->bvh8.SAHCost();
	default: return 0; // unsupported layout. See note in constructor.
	}
}

float AccStruc::EPOCost()
{
	switch (layout)
	{
	case BVHBase::BVHType::LAYOUT_BVH: return ((BVH*)bvh)->EPOCost();
	case BVHBase::BVHType::LAYOUT_BVH_SOA: return ((BVH_SoA*)bvh)->bvh.EPOCost();
	case BVHBase::BVHType::LAYOUT_BVH_GPU:return ((BVH_GPU*)bvh)->bvh.EPOCost();
	case BVHBase::BVHType::LAYOUT_BVH4_CPU: return ((BVH4_CPU*)bvh)->bvh4.bvh.EPOCost();
	case BVHBase::BVHType::LAYOUT_BVH4_GPU: return ((BVH4_GPU*)bvh)->bvh4.bvh.EPOCost();
	case BVHBase::BVHType::LAYOUT_CWBVH: return ((BVH8_CWBVH*)bvh)->bvh8.bvh.EPOCost();
	case BVHBase::BVHType::LAYOUT_BVH8_AVX2: return ((BVH8_CPU*)bvh)->bvh8.bvh.EPOCost();
	default: return 0; // unsupported layout. See note in constructor.
	}
	// note: there is (intentionally) no full class derivation in tinybvh.
	// We thus have to specialize here for each possible layout.
}

void AccStruc::IntersectBatch( Ray* raySet, int rayCount )
{
	switch (layout)
	{
	case BVHBase::BVHType::LAYOUT_BVH:
	{
		BVH* accstruc = (BVH*)bvh;
		for( int i = 0; i < rayCount; i++ ) accstruc->Intersect( raySet[i] );
		break;
	}
	case BVHBase::BVHType::LAYOUT_BVH_SOA:
	{
		BVH_SoA* accstruc = (BVH_SoA*)bvh;
		for( int i = 0; i < rayCount; i++ ) accstruc->Intersect( raySet[i] );
		break;
	}
	case BVHBase::BVHType::LAYOUT_BVH_GPU:
	{
		BVH_GPU* accstruc = (BVH_GPU*)bvh;
		for( int i = 0; i < rayCount; i++ ) accstruc->Intersect( raySet[i] );
		break;
	}
	case BVHBase::BVHType::LAYOUT_BVH4_CPU:
	{
		BVH4_CPU* accstruc = (BVH4_CPU*)bvh;
		for( int i = 0; i < rayCount; i++ ) accstruc->Intersect( raySet[i] );
		break;
	}
	case BVHBase::BVHType::LAYOUT_BVH4_GPU:
	{
		BVH4_GPU* accstruc = (BVH4_GPU*)bvh;
		for( int i = 0; i < rayCount; i++ ) accstruc->Intersect( raySet[i] );
		break;
	}
	case BVHBase::BVHType::LAYOUT_CWBVH:
	{
		BVH8_CWBVH* accstruc = (BVH8_CWBVH*)bvh;
		for( int i = 0; i < rayCount; i++ ) accstruc->Intersect( raySet[i] );
		break;
	}
	case BVHBase::BVHType::LAYOUT_BVH8_AVX2:
	{
		BVH8_CPU* accstruc = (BVH8_CPU*)bvh;
		for( int i = 0; i < rayCount; i++ ) accstruc->Intersect( raySet[i] );
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
	case BVHBase::BVHType::LAYOUT_BVH:
	{
		BVH* accstruc = (BVH*)bvh;
		for( int i = 0; i < rayCount; i++ ) accstruc->IsOccluded( raySet[i] );
		break;
	}
	case BVHBase::BVHType::LAYOUT_BVH_SOA:
	{
		BVH_SoA* accstruc = (BVH_SoA*)bvh;
		for( int i = 0; i < rayCount; i++ ) accstruc->IsOccluded( raySet[i] );
		break;
	}
	case BVHBase::BVHType::LAYOUT_BVH_GPU:
	{
		BVH_GPU* accstruc = (BVH_GPU*)bvh;
		for( int i = 0; i < rayCount; i++ ) accstruc->IsOccluded( raySet[i] );
		break;
	}
	case BVHBase::BVHType::LAYOUT_BVH4_CPU:
	{
		BVH4_CPU* accstruc = (BVH4_CPU*)bvh;
		for( int i = 0; i < rayCount; i++ ) accstruc->IsOccluded( raySet[i] );
		break;
	}
	case BVHBase::BVHType::LAYOUT_BVH4_GPU:
	{
		BVH4_GPU* accstruc = (BVH4_GPU*)bvh;
		for( int i = 0; i < rayCount; i++ ) accstruc->IsOccluded( raySet[i] );
		break;
	}
	case BVHBase::BVHType::LAYOUT_CWBVH:
	{
		BVH8_CWBVH* accstruc = (BVH8_CWBVH*)bvh;
		for( int i = 0; i < rayCount; i++ ) accstruc->IsOccluded( raySet[i] );
		break;
	}
	case BVHBase::BVHType::LAYOUT_BVH8_AVX2:
	{
		BVH8_CPU* accstruc = (BVH8_CPU*)bvh;
		for( int i = 0; i < rayCount; i++ ) accstruc->IsOccluded( raySet[i] );
		break;
	}
	default: // unsupported layout. See note in constructor.
		break;
	}
}

}; // namespace tinybvh