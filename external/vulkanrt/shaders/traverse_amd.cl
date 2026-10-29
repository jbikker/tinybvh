// AMD hardware ray traversal (RDNA3: BVH4, RDNA4: BVH4 + BVH8) for the Vulkan benchmark.

// Same as traverse.cl; see the notes there.
struct RayData
{
	float4 O; // .xyz = origin,    .w unused
	float4 D; // .xyz = direction, .w unused
};

// ============================================================================
//
//        T R A V E R S E _ G P U 8 W A Y
//
// ============================================================================

inline uint bvh8_id_to_offset( uint id ) { return ( id & ( ~7u ) ) << 3; }
inline uint bvh8_id_to_type( uint id ) { return id & 7u; }

#define TRIANGLE 0u
#define INVALID_NODE 0xffffffffu

#define BVH_STACK_TERMINAL_NODE 0xfffffffeu
#define BVH_STACK_SKIP_0_TO_7 0xfffffff9u
#ifndef HW_BVH_STACK_ENTRIES
#define HW_BVH_STACK_ENTRIES 16
#endif
#if HW_BVH_STACK_ENTRIES != 8 && HW_BVH_STACK_ENTRIES != 16
#error "GFX12 hardware BVH stacks support 8 or 16 entries per ray in this kernel."
#endif
// GFX12 stack encoding.
#define BVH_STACK_CONTROLS HW_BVH_STACK_ENTRIES
#define HW_BVH_STACK_BASE_SHIFT 15
// Hardware-stack kernels require 64 work-items and reserve one stack per ray.
#define HW_BVH_WORKGROUP_SIZE 64
// BVH descriptor: zero base, distance sorting, 42-bit size, pointer flags enabled.
// Type 8; GFX12 triangle results contain t, barycentrics and primitive IDs.
#define BVH8_DESCRIPTOR                                      \
    ((uint4)(                                               \
        0u,                                                 \
        (1u << 31) | /* box sorting */                       \
        (1u << 20),  /* triangles first */                   \
        0xffffffffu, /* address-range low bits */            \
        (8u << 28) | /* BVH resource type */                 \
        (1u << 24) | /* triangle IJ return */                \
        (1u << 23) | /* pointer flags */                     \
        (1u << 22) | /* instance enable */                   \
        (1u << 21) | /* wide sorting */                      \
        (1u << 19) | /* compressed format */                 \
        0x3ffu       /* address-range high bits */           \
    ))

typedef uint uint10 __attribute__((ext_vector_type(10)));
typedef uint uint8 __attribute__((ext_vector_type(8)));

float4 traverse_gpu8wayHW( const global float4* alt4Node, const float4 O, const float4 D, const float4 rD, const float tmax, __local uint* hwStack )
{
	const ulong encoded_base = ( (ulong)alt4Node >> 3 ) & ( ( ( 1UL << 42 ) - 1UL ) << 3 );
	uint prevNode = INVALID_NODE;
	uint parentBoxID = INVALID_NODE;
	// LDS stack entries have a fixed stride of 32 words, even with wave64.
	uint lane = get_local_id( 1 ) * get_local_size( 0 ) + get_local_id( 0 );
	uint stackAddr = ( ( (uint)(size_t)hwStack >> 2 ) + ( lane & 31u ) +
					  ( lane >> 5 ) * ( 32u * HW_BVH_STACK_ENTRIES ) ) << HW_BVH_STACK_BASE_SHIFT;
	float4 hit = (float4)( tmax, 0.0f, 0.0f, as_float( INVALID_NODE ) );

	uint currentNodeID = 5; // start at root BOX32 node at byte offset zero.
	while (1)
	{
		uint2 stackResults;
		float3 outOrigin, outDirection;
		uint10 result = __builtin_amdgcn_image_bvh8_intersect_ray(
			encoded_base, prevNode != INVALID_NODE ? tmax : hit.x, (uchar)0xff, O.xyz, D.xyz, currentNodeID, BVH8_DESCRIPTOR, &outOrigin, &outDirection);

		uint type = bvh8_id_to_type( currentNodeID );
		if (type == TRIANGLE)
		{
			for (uint i = 0; i < 2; ++i)
			{
				const uint k = i * 4;
				const float t = as_float( result[k] );
				if (isfinite( t ) && t >= 0.0f && t <= hit.x)
				{
					const float u = fabs( as_float( result[k + 1] ) );
					const float v = fabs( as_float( result[k + 2] ) );
					const uint id = result[k + 3] >> 1;
					hit = (float4)( t, u, v, as_float( id ) );
				}
			}
			uint8 children = (uint8)INVALID_NODE;
			children.s7 = BVH_STACK_SKIP_0_TO_7;
			stackResults = __builtin_amdgcn_ds_bvh_stack_push8_pop1_rtn(
				stackAddr, INVALID_NODE, children, BVH_STACK_CONTROLS );
		}
		else
		{
			parentBoxID = currentNodeID;
			uint8 children;
			for (uint i = 0; i < 8; ++i) children[i] = result[i];
			stackResults = __builtin_amdgcn_ds_bvh_stack_push8_pop1_rtn(
				stackAddr, prevNode, children, BVH_STACK_CONTROLS );
		}

		uint nextNodeID = stackResults.x;
		stackAddr = stackResults.y;
		prevNode = INVALID_NODE;

		if (nextNodeID != INVALID_NODE && nextNodeID != BVH_STACK_TERMINAL_NODE)
		{
			currentNodeID = nextNodeID;
			continue;
		}
		else if (nextNodeID == BVH_STACK_TERMINAL_NODE)
		{
			break;
		}

		// continue with nearest node or first node on the stack
		prevNode = currentNodeID;
		uint parentID;
		const global uint* words =
			(const global uint*)( (const global uchar*)alt4Node +
								  bvh8_id_to_offset( currentNodeID ) );
		if (type == TRIANGLE)
		{
			parentID = parentBoxID;
		}
		else
		{
			parentID = words[2];
		}
		currentNodeID = parentID;

		if (currentNodeID == INVALID_NODE)
		{
			break;
		}
	}
	return hit;
}

#define GROUP_X    8
#define GROUP_Y    8

uint depth_to_pixel( const float t )
{
	const float g = t >= 1e30f ? 0.0f : (1.0f - fmin( 1.0f, t * 0.01f ));
	const uint v = (uint)(g * 255.0f + 0.5f);
	return 0xff000000 | (v << 16) | (v << 8) | v; // R in the low byte: R8G8B8A8_UNORM
}

#define BVH_DIR_MIN 0x1p-100f

float safe_rcp( float x )
{
	const uint s = as_uint( x ) & 0x80000000u;
	return 1.0f / as_float( as_uint( fmax( fabs( x ), BVH_DIR_MIN ) ) | s );
}

// Launched 2D over the image, exactly like bench_bvh2.
__attribute__((reqd_work_group_size( GROUP_X, GROUP_Y, 1 )))
void kernel bench_amd_bvh8( const global float4* alt4Node, const global struct RayData* rayData, global uint* pixels )
{
	local uint hwStack[HW_BVH_WORKGROUP_SIZE * HW_BVH_STACK_ENTRIES];
	const uint threadId = get_global_id( 1 ) * get_global_size( 0 ) + get_global_id( 0 );
	const float4 O = (float4)( rayData[threadId].O.xyz, 0 );
	const float4 D = (float4)( rayData[threadId].D.xyz, 0 );
	const float4 rD = (float4)( safe_rcp( D.x ), safe_rcp( D.y ), safe_rcp( D.z ), 0 );
	const float4 hit = traverse_gpu8wayHW( alt4Node, O, D, rD, 1e30f, hwStack );
	pixels[threadId] = depth_to_pixel( hit.x );
}
