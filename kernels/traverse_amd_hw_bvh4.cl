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
#define HW_BVH_STACK_ENTRIES 16
#ifdef ISRDNA4
// GFX12 stack encoding.
#define BVH_STACK_CONTROLS HW_BVH_STACK_ENTRIES
#define HW_BVH_STACK_BASE_SHIFT 15
#else
#define BVH_STACK_CONTROLS 0x2000
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
	uint node_id = 5; // root BOX32 node at byte offset zero.
	uint prevNode = INVALID_NODE;
	// LDS stack entries have a fixed stride of 32 words, even with wave64.
	uint lane = get_local_id( 0 );
	uint baseWord = ( (uint)(size_t)hwStack >> 2 ) + ( lane & 31u ) +
					( lane >> 5 ) * ( 32u * HW_BVH_STACK_ENTRIES );
	uint stackAddr = baseWord << HW_BVH_STACK_BASE_SHIFT;
	float4 hit = (float4)( tmax, 0.0f, 0.0f, as_float( INVALID_NODE ) );

	while (1)
	{
		ulong pointer = encoded_base + (ulong)node_id;
		uint type = id_to_type( node_id );
		uint lastVisited = BVH_STACK_TERMINAL_NODE;
		uint parentID = INVALID_NODE;

		uint4 dst4 = __builtin_amdgcn_image_bvh_intersect_ray_l(
			pointer, hit.x, O, D, rD, BVH_DESCRIPTOR );
		if (type == TRIANGLE)
		{
			float4 triData = as_float4( dst4 );
			float invDet = native_recip( triData[1] );
			float t = triData[0] * invDet;

			const global uint* triWords =
				(const global uint*)( (const global uchar*)alt4Node +
									  id_to_offset( node_id ) );
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
			dst4 = (uint4)INVALID_NODE;
			parentID = triWords[14]; // TinyBVH parent ID at byte 56.
		}
		else
		{
			lastVisited = prevNode;
			const global uint* words =
				(const global uint*)( (const global uchar*)alt4Node +
									  id_to_offset( node_id ) );

			parentID = words[31]; // TinyBVH parent ID at byte 124.
		}

		uint visitedNode = node_id;
		uint2 stackResults = __builtin_amdgcn_ds_bvh_stack_push4_pop1_rtn(
			stackAddr, lastVisited, dst4, BVH_STACK_CONTROLS );

		node_id = stackResults.x;
		stackAddr = stackResults.y;
		prevNode = INVALID_NODE;

		// continue with nearest node or first node on the stack
		if (node_id == INVALID_NODE)
		{
		#ifndef ISRDNA4
			// Restore the GFX11 stack index after an empty-stack pop.
			stackAddr += 1u;
		#endif
			prevNode = visitedNode;
			node_id = parentID;

			if (node_id == INVALID_NODE)
			{
				break;
			}
		}
		else if (node_id == BVH_STACK_TERMINAL_NODE)
		{
			break;
		}
	}
	return hit;
}

bool isoccluded_gpu4wayHW( const global float4* alt4Node, const float4 O, const float4 D, const float4 rD, const float tmax )
{
	const ulong encoded_base = ( (ulong)alt4Node >> 3 ) & ( ( ( 1UL << 42 ) - 1UL ) << 3 );
	uint node_id = 5; // root BOX32 node at byte offset zero.
	float4 hit = (float4)( tmax, 0.0f, 0.0f, as_float( INVALID_NODE ) );

	// traverse the BVH
	unsigned stack[STACK_SIZE], stackPtr = 0;
	while (1)
	{
		ulong pointer = encoded_base + (ulong)node_id;
		uint4 dst4 = __builtin_amdgcn_image_bvh_intersect_ray_l(
			pointer, hit.x, O, D, rD, BVH_DESCRIPTOR_OCCLUSION );
		uint type = id_to_type( node_id );

		unsigned nextNode = 0;
		if (type == TRIANGLE)
		{
			if (dst4.w != 0u)
				return true;
		}
		else
		{
			// Hardware returns sorted children; push the farthest first.
			if (dst4.w != 0xffffffffu)
			{
				nextNode = dst4.w;
			}
			if (dst4.z != 0xffffffffu)
			{
				if (nextNode)
					stack[stackPtr++] = nextNode;
				nextNode = dst4.z;
			}
			if (dst4.y != 0xffffffffu)
			{
				if (nextNode)
					stack[stackPtr++] = nextNode;
				nextNode = dst4.y;
			}
			if (dst4.x != 0xffffffffu)
			{
				if (nextNode)
					stack[stackPtr++] = nextNode;
				nextNode = dst4.x;
			}
		}
		// continue with nearest node or first node on the stack
		if (nextNode)
			node_id = nextNode;
		else
		{
			if (!stackPtr)
				break;
			node_id = stack[--stackPtr];
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
