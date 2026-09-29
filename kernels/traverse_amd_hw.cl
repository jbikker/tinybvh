// gpu-side code for hardware ray traversal

struct Ray
{
	// Match the 64-byte ray layout used by traverse.cl.
	float4 O, D, rD;
	float4 hit;
};

// BVH traversal stack size
#define STACK_SIZE 32

// Includes for hardware traversal kernel implementations:
// Minimum required for this to work. Does not check for
// compiler compatability
#if defined( ISAMD ) && ( defined( ISRDNA3 ) || defined( ISRDNA4 ) )
#include "traverse_amd_hw_bvh4.cl"
#else
#error \
	"TinyBVH AMD hardware ray tracing requires RDNA3/RDNA4 and compiler support "
"for AMD BVH intrinsics."
#endif
