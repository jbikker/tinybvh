// ============================================================================
//
//        T R A V E R S E _ G P U 4 W A Y
//
// ============================================================================

// Node pointer encoding follows Mesa RADV (src/amd/vulkan/nir/radv_nir_rt_common.c).

inline uint id_to_offset( uint id ) { return ( id & ( ~7u ) ) << 3; }
inline uint id_to_type( uint id ) { return id & 7u; }

#define TRIANGLE 0u
#define INVALID_NODE 0xffffffffu

#define BVH_STACK_TERMINAL_NODE 0xfffffffeu
#ifndef HW_BVH_STACK_ENTRIES
#define HW_BVH_STACK_ENTRIES 16
#endif
#if HW_BVH_STACK_ENTRIES != 8 && HW_BVH_STACK_ENTRIES != 16 &&                 \
    HW_BVH_STACK_ENTRIES != 32 && HW_BVH_STACK_ENTRIES != 64
#error "AMD hardware BVH stacks require 8, 16, 32 or 64 entries per ray."
#endif
#ifdef ISRDNA4
// GFX12 stack encoding.
#define BVH_STACK_CONTROLS HW_BVH_STACK_ENTRIES
#define HW_BVH_STACK_BASE_SHIFT 15
#else
#if HW_BVH_STACK_ENTRIES == 8
#define BVH_STACK_CONTROLS 0x0000
#elif HW_BVH_STACK_ENTRIES == 16
#define BVH_STACK_CONTROLS 0x1000
#elif HW_BVH_STACK_ENTRIES == 32
#define BVH_STACK_CONTROLS 0x2000
#else
#define BVH_STACK_CONTROLS 0x3000
#endif
#define HW_BVH_STACK_BASE_SHIFT 18
#endif
// Hardware-stack kernels require 64 work-items and reserve one stack per ray.
#define HW_BVH_WORKGROUP_SIZE 64
// BVH descriptor: zero base, distance sorting, 42-bit size, pointer flags enabled.
// Type 8; triangle results are (t numerator, denominator, u numerator, v numerator).
// Reference: AMD RDNA3 ISA, section 10.9, BVH Resource Descriptor.
#define BVH_DESCRIPTOR                        \
	( (uint4)( 0u, ( 1u << 31 ), 0xffffffffu, \
			   ( 1u << 31 ) | ( 1u << 24 ) | ( 1u << 23 ) | 0x3ffu ) )
// Return triangle hit status instead of barycentrics for occlusion.
#define BVH_DESCRIPTOR_OCCLUSION \
	( (uint4)( 0u, ( 1u << 31 ), 0xffffffffu, ( 1u << 31 ) | ( 1u << 23 ) | 0x3ffu ) )

float4 traverse_gpu4wayHW( const global float4* alt4Node, const float4 O, const float4 D, const float4 rD, const float tmax, __local uint* hwStack )
{
	const ulong encoded_base = ( (ulong)alt4Node >> 3 ) & ( ( ( 1UL << 42 ) - 1UL ) << 3 );
	uint prevNode = INVALID_NODE;
	// LDS stack entries have a fixed stride of 32 words, even with wave64.
	uint lane = get_local_id( 0 );
	uint stackAddr = ( ( (uint)(size_t)hwStack >> 2 ) + ( lane & 31u ) +
					  ( lane >> 5 ) * ( 32u * HW_BVH_STACK_ENTRIES ) ) << HW_BVH_STACK_BASE_SHIFT;
	float4 hit = (float4)( tmax, 0.0f, 0.0f, as_float( INVALID_NODE ) );

	uint currentNodeID = 5; // start at root BOX32 node at byte offset zero.
	while (1)
	{
		uint2 stackResults;
		ulong currentPointer = encoded_base + (ulong)currentNodeID;
		uint4 dst4 = __builtin_amdgcn_image_bvh_intersect_ray_l(
			currentPointer, hit.x, O, D, rD, BVH_DESCRIPTOR );

		uint lastVisited = BVH_STACK_TERMINAL_NODE;
		uint type = id_to_type( currentNodeID );
		if (type == TRIANGLE)
		{
			float4 triData = as_float4( dst4 );
			float invDet = native_recip( triData[1] );
			float t = triData[0] * invDet;

			const global uint* triWords =
				(const global uint*)( (const global uchar*)alt4Node +
									  id_to_offset( currentNodeID ) );
			uint id = triWords[12];

			if (t >= 0.0f && t <= hit.x)
			{
				float u = triData[2] * invDet;
				float v = triData[3] * invDet;
			#ifdef ISRDNA4
				// GFX12 uses the barycentric signs for metadata.
				u = fabs( u ), v = fabs( v );
			#endif
				hit = (float4)( t, u, v, as_float( id ) );
			}
			stackResults = __builtin_amdgcn_ds_bvh_stack_push4_pop1_rtn(
				stackAddr, BVH_STACK_TERMINAL_NODE, (uint4)INVALID_NODE, BVH_STACK_CONTROLS );
		}
		else
		{
			stackResults = __builtin_amdgcn_ds_bvh_stack_push4_pop1_rtn(
				stackAddr, prevNode, dst4, BVH_STACK_CONTROLS );
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
		#ifndef ISRDNA4
		// Restore the GFX11 stack index after an empty-stack pop.
		stackAddr += 1u;
		#endif
		prevNode = currentNodeID;
		uint parentID;
		const global uint* words =
			(const global uint*)( (const global uchar*)alt4Node +
								  id_to_offset( currentNodeID ) );
		if (type == TRIANGLE)
		{
			parentID = words[14]; // TinyBVH parent ID at byte 56
		}
		else
		{
			parentID = words[31]; // TinyBVH parent ID at byte 124.
		}
		currentNodeID = parentID;

		if (currentNodeID == INVALID_NODE)
		{
			break;
		}
	}
	return hit;
}

bool isoccluded_gpu4wayHW( const global float4* alt4Node, const float4 O, const float4 D, const float4 rD, const float tmax, __local uint* hwStack )
{
	const ulong encoded_base = ( (ulong)alt4Node >> 3 ) & ( ( ( 1UL << 42 ) - 1UL ) << 3 );
	uint prevNode = INVALID_NODE;
	// LDS stack entries have a fixed stride of 32 words, even with wave64.
	uint lane = get_local_id( 0 );
	uint stackAddr = ( ( (uint)(size_t)hwStack >> 2 ) + ( lane & 31u ) +
					  ( lane >> 5 ) * ( 32u * HW_BVH_STACK_ENTRIES ) ) << HW_BVH_STACK_BASE_SHIFT;

	uint currentNodeID = 5; // start at root BOX32 node at byte offset zero.
	while (1)
	{
		uint2 stackResults;
		ulong currentPointer = encoded_base + (ulong)currentNodeID;
		uint4 dst4 = __builtin_amdgcn_image_bvh_intersect_ray_l(
			currentPointer, tmax, O, D, rD, BVH_DESCRIPTOR_OCCLUSION );

		uint type = id_to_type( currentNodeID );
		if (type == TRIANGLE)
		{
			if (dst4.w != 0u) return true;
			stackResults = __builtin_amdgcn_ds_bvh_stack_push4_pop1_rtn(
				stackAddr, BVH_STACK_TERMINAL_NODE, (uint4)INVALID_NODE, BVH_STACK_CONTROLS );
		}
		else
		{
			stackResults = __builtin_amdgcn_ds_bvh_stack_push4_pop1_rtn(
				stackAddr, prevNode, dst4, BVH_STACK_CONTROLS );
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
		#ifndef ISRDNA4
		// Restore the GFX11 stack index after an empty-stack pop.
		stackAddr += 1u;
		#endif
		prevNode = currentNodeID;
		uint parentID;
		const global uint* words =
			(const global uint*)( (const global uchar*)alt4Node +
								  id_to_offset( currentNodeID ) );
		if (type == TRIANGLE)
		{
			parentID = words[14]; // TinyBVH parent ID at byte 56
		}
		else
		{
			parentID = words[31]; // TinyBVH parent ID at byte 124.
		}
		currentNodeID = parentID;

		if (currentNodeID == INVALID_NODE)
		{
			break;
		}
	}
	return false;
}

__attribute__(( reqd_work_group_size( HW_BVH_WORKGROUP_SIZE, 1, 1 ) ))
void kernel batch_gpu4wayHW( const global float4* alt4Node, global struct Ray* rayData )
{
	// fetch ray
	const unsigned threadId = get_global_id( 0 );
	const float4 O = rayData[threadId].O;
	const float4 D = rayData[threadId].D;
	const float4 rD = rayData[threadId].rD;

	__local uint hwStack[HW_BVH_WORKGROUP_SIZE * HW_BVH_STACK_ENTRIES];
	float4 hit = traverse_gpu4wayHW( alt4Node, O, D, rD, 1e30f, hwStack );

	rayData[threadId].hit = hit;
}

__attribute__(( reqd_work_group_size( HW_BVH_WORKGROUP_SIZE, 1, 1 ) ))
void kernel batch_gpu4wayHW_any( const global float4* alt4Node, global struct Ray* rayData )
{
	// fetch ray
	const unsigned threadId = get_global_id( 0 );
	const float4 O = rayData[threadId].O;
	const float4 D = rayData[threadId].D;
	const float4 rD = rayData[threadId].rD;
	const float tmax = rayData[threadId].hit.x;

	__local uint hwStack[HW_BVH_WORKGROUP_SIZE * HW_BVH_STACK_ENTRIES];
	float4 hit = 0;
	if (isoccluded_gpu4wayHW( alt4Node, O, D, rD, tmax, hwStack )) hit.w = as_float( 1 );
	rayData[threadId].hit = hit;
}
