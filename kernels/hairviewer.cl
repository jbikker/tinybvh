// gpu-side code for ray traversal (hair demo)

#include "tools.cl"

struct BVHNode
{
	float4 lmin; // unsigned left in w
	float4 lmax; // unsigned right in w
	float4 rmin; // unsigned triCount in w
	float4 rmax; // unsigned firstTri in w
};

struct Strand
{
	uint offset, N;
};

struct Ray
{
	// data is defined here as 16-byte values to encourage the compilers
	// to fetch 16 bytes at a time: 12 (so, 8 + 4) will be slower.
	float4 O, D, rD; // 48 byte
	float4 hit; // 16 byte
};

// BVH traversal stack size 
#define STACK_SIZE 32

// Hair specifics
#include "rocap.cl"

// View
float4 eye, C, p0, p1, p2;

// Set the view
void kernel SetRenderData( float4 _eye, float4 _p0, float4 _p1, float4 _p2 )
{
	if (get_global_id( 0 ) != 0) return;
	eye = _eye, p0 = _p0, p1 = _p1, p2 = _p2;
}

// BVH_GPU traversal 
uint traverse_hair_bvh( const global struct BVHNode* bvhNode, const global uint* idx, const float3 O, const float3 D, const float3 rD, const float tmax, const global struct Strand* hairs, const global float4* hairVerts, float4* hitOut )
{
	// traverse BVH
	const float3 rO = O * -rD;
	float4 hit = (float4)( tmax, 0, 0, as_float( ~0u ) ); // .w = ~0: no strand hit
	uint nodeIdx = 0, stack[STACK_SIZE], stackPtr = 0, steps = 0;
	while (1)
	{
		steps++;
		if (nodeIdx & 0x80000000)
		{
			const uint hairCount = (nodeIdx >> 24) & 127;
			uint firstHairIdx = nodeIdx & 0xffffff;
			for( uint i = 0; i < hairCount; i++ )
			{
				uint hairIdx = idx[firstHairIdx + i];
				eval_rocap( hairIdx, O, D, &hit, hairs, hairVerts );
			}
			if (stackPtr == 0) break;
			nodeIdx = stack[--stackPtr];
			continue;
		}
		const float4 lmin = bvhNode[nodeIdx].lmin, lmax = bvhNode[nodeIdx].lmax;
		const float4 rmin = bvhNode[nodeIdx].rmin, rmax = bvhNode[nodeIdx].rmax;
		uint left = as_uint( lmin.w ), right = as_uint( lmax.w );
		const float3 t1a = fma( lmin.xyz, rD, rO ), t2a = fma( lmax.xyz, rD, rO );
		const float3 t1b = fma( rmin.xyz, rD, rO ), t2b = fma( rmax.xyz, rD, rO );
		const float3 minta = fmin( t1a, t2a ), maxta = fmax( t1a, t2a );
		const float3 mintb = fmin( t1b, t2b ), maxtb = fmax( t1b, t2b );
		const float tmina = fmax( fmax( fmax( minta.x, minta.y ), minta.z ), 0 );
		const float tminb = fmax( fmax( fmax( mintb.x, mintb.y ), mintb.z ), 0 );
		const float tmaxa = fmin( fmin( fmin( maxta.x, maxta.y ), maxta.z ), hit.x );
		const float tmaxb = fmin( fmin( fmin( maxtb.x, maxtb.y ), maxtb.z ), hit.x );
		const bool hitA = tmina <= tmaxa, hitB = tminb <= tmaxb;
		if (hitA && hitB)
		{
			uint near = left, far = right;
			if (tminb < tmina) near = right, far = left;
			stack[stackPtr++] = far, nodeIdx = near;
		}
		else if (hitA) nodeIdx = left;
		else if (hitB) nodeIdx = right;
		else { if (stackPtr == 0) break; nodeIdx = stack[--stackPtr]; }
	}
	*hitOut = hit;
	return steps;
}


// Kernel
void kernel Render( global uint* pixels, const global struct BVHNode* bvhNode, const global uint* idx, const global struct Strand* hairs, const global float4* hairVerts )
{
	const uint x = get_global_id( 0 ), y = get_global_id( 1 );
	// setup primary ray
	const float u = (float)x / get_global_size( 0 );
	const float v = (float)y / get_global_size( 1 );
	const float4 P = p0 + u * (p1 - p0) + v * (p2 - p0);
	const float3 O = eye.xyz;
	const float3 D = normalize( (P - eye).xyz );
	const float3 rD = native_recip( D );
	// trace primary ray
	float4 hit;
	const uint steps = traverse_hair_bvh( bvhNode, idx, O, D, rD, 1e30f, hairs, hairVerts, &hit );
	// hit.x = distance, hit.w = strand index (~0 if the ray missed). Shade with
	// e.g. eval_rocap_normal( hit, O, D, hairs, hairVerts ) once you need normals.
	// visualize result
	float4 p = (float4)( 0 );
	if (hit.x < 1e30f)
	{
		const float3 N = eval_rocap_normal( hit, O, D, hairs, hairVerts );
		p = (float4)( (N + 1) * 0.5f, 1 );
	}
	int3 rgb = convert_int3( min( sqrt( p.xyz ), (float3)(1.0f, 1.0f, 1.0f) ) * 255.0f );
	const uint pixelIdx = x + y * get_global_size( 0 );
	pixels[pixelIdx] = (rgb.x << 16) + (rgb.y << 8) + rgb.z;
}