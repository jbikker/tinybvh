// gpu-side path tracing (wavefront) with TLAS/BLAS support.
// Used by tiny_bvh_gpu2.cpp.

#pragma OPENCL EXTENSION cl_khr_global_int32_base_atomics : enable

#define DEPRECATED_TLAS_PATH
#include "traverse.cl"
#include "tools.cl"

#define PATH_LAST_SPECULAR	1	// previous vertex was camera or mirror
#define PATH_VIA_DIFFUSE	2	// path has at least one diffuse vertex

#define MATERIAL_LIGHT		1	// material emits light - end of path
#define MATERIAL_SPECULAR	2	// material is pure specular

struct Instance
{
	float transform[16];
	float invTransform[16];
	float xmin, ymin, zmin; uint blasIdx;
	float xmax, ymax, zmax; uint dummy[9];
};

// rendering parameters
float4 eye, C, p0, p1, p2;
uint frameIdx, width, height, dummy3;
global float4* bistroNodes;
global float4* bistroTris;
global float4* bistroVerts;
global float4* dragonNodes;
global float4* dragonTris;
global float4* dragonVerts;
global struct BVHNode* tlasNodes;
global uint* tlasIdx;
global struct Instance* instances;
global uint* blueNoise;
global volatile int extendTasks, shadeTasks, connectTasks; // atomic counters
const float3 lightColor = (float3)(25,25,22);
const float3 lightPos = (float3)(-22, 12, 2);

#include "traverse_tlas.cl" // needs access to above arrays for now.

// Blue noise interface for fixed 128x128x8 dataset.
float2 Noise( const uint x, const uint y, const uint page /* 0..7 */ )
{
	const uint ix = x & 127, iy = y & 127;
	const uint v2 = blueNoise[(page << 14) + (iy << 7) + ix];
	const uint r = v2 >> 16, g = (v2 >> 8) & 255;
	return (float2)( (float)r * 0.00392f, (float)g * 0.00392f );
}

// PathState: path throughput, current extension ray, pixel index
struct PathState
{
	float4 T; // xyz = rgb, postponed MIS pdf in w
	float4 O; // O.w: 24-bit pixel index, 4-bit path depth, 4-bit path flags
	float4 D; // t in D.w
	float4 hit;
};

// Potential contribution: shadoww ray origin & dir, throughput
struct Potential
{
	float4 T;
	float4 O; // pixel index in O.w
	float4 D; // t in D.w
};

// atomic counter management - prepare for primary ray wavefront
void kernel SetRenderData( int _primaryRayCount,
	float4 _eye, float4 _p0, float4 _p1, float4 _p2, uint _frameIdx, uint _width, uint _height,
	global float4* _bistroNodes, global float4* _bistroTris, global float4* _bistroVerts, 
	global float4* _dragonNodes, global float4* _dragonTris, global float4* _dragonVerts, 
	global struct BVHNode* _tlasNodes, global uint* _tlasIndices, global struct Instance* _instances, 
	global uint* _blueNoise
)
{
	if (get_global_id( 0 ) != 0) return;
	// set camera parameters
	eye = _eye, p0 = _p0, p1 = _p1, p2 = _p2;
	frameIdx = _frameIdx, width = _width, height = _height;
	// set BVH pointers
	bistroNodes = _bistroNodes;
	bistroTris = _bistroTris;
	bistroVerts = _bistroVerts;
	dragonNodes = _dragonNodes;
	dragonTris = _dragonTris;
	dragonVerts = _dragonVerts;
	tlasNodes = _tlasNodes;
	tlasIdx = _tlasIndices;
	instances = _instances;
	blueNoise = _blueNoise;
	// initialize atomic counters
	extendTasks = shadeTasks = _primaryRayCount;
	connectTasks = 0;
}

// clear accumulator
void kernel Clear( global float4* accumulator )
{
	const uint pixelIdx = get_global_id( 0 );
	accumulator[pixelIdx] = (float4)(0);
}

// primary ray generation
void kernel Generate( global struct PathState* raysOut, uint frameSeed )
{
	const uint x = get_global_id( 0 ), y = get_global_id( 1 );
	const uint id = x + y * get_global_size( 0 );
	uint seed = WangHash( id * 13131 + frameSeed );
	const float u = ((float)x + RandomFloat( &seed )) / (float)get_global_size( 0 );
	const float v = ((float)y + RandomFloat( &seed )) / (float)get_global_size( 1 );
	const float4 P = p0 + u * (p1 - p0) + v * (p2 - p0);
	raysOut[id].T = (float4)(1, 1, 1, 1 );
	raysOut[id].O = (float4)(eye.xyz, as_float( (id << 8) + PATH_LAST_SPECULAR ));
	raysOut[id].D = (float4)(fast_normalize( P.xyz - eye.xyz ), 1e30f);
	raysOut[id].hit = (float4)(1e30f, 0, 0, as_float( 0 ));
}

// extend: trace the generated rays to find the nearest intersection point.
void kernel Extend( global struct PathState* raysIn )
{
	// we use a worker thread system here, where a fixed number of threads 'fight for food'
	// by decreasing an atomic counter. This way, the counter can stay on the GPU, saving
	// expensive transfers: The host doesn't need to know the exact amount of tasks.
	while (1)
	{
		// obtain task
		if (extendTasks < 1) break;
		const int pathId = atomic_dec( &extendTasks ) - 1;
		if (pathId < 0) break; // someone else could have decreased it before us.
		const float4 O4 = raysIn[pathId].O;
		const float4 D4 = raysIn[pathId].D;
		const float4 rD4 = native_recip( D4 );
		raysIn[pathId].hit = traverse_tlas( O4, D4, rD4, 1e30f, 0 );
	}
}

// lightpdf: approximate probability density of hemisphere directions towards light.
float LightPDF( const float distance, const float3 D )  
{
	float solidAngle = min( TWOPI, 9 * 5 * (1.0f / (distance * distance)) * fabs( D.y ) /* NLdotL */ );
	return 1 / solidAngle;
}

// syncing counters: at this point, we need to reset the extendTasks and connectTasks counters.
void kernel UpdateCounters1() { if (get_global_id( 0 ) == 0) extendTasks = 0; }

// shade: process intersection results; this evaluates the BRDF and creates 
// extension rays and shadow rays.
void kernel Shade( global float4* accumulator,
	global struct PathState* raysIn, global struct PathState* raysOut,
	global struct Potential* shadowOut, uint sampleIdx )
{
	while (1)
	{
		// obtain task - see note on worker threads in Extend
		if (shadeTasks < 1) break;
		const int pathId = atomic_dec( &shadeTasks ) - 1;
		if (pathId < 0) break;
		// fetch path data
		float4 T4 = raysIn[pathId].T;		// xyz = rgb, postponed pdf in w
		float4 O4 = raysIn[pathId].O;		// pixel index in O.w
		float4 D4 = raysIn[pathId].D;		// t in D.w
		float4 hit = raysIn[pathId].hit;	// dist, u, v, prim
		// prepare for shading
		uint pathState = as_uint( O4.w );
		uint pixelIdx = pathState >> 8;
		uint depth = (pathState >> 4) & 15;
		uint seed = WangHash( as_uint( O4.w ) + frameIdx * 17117 );
		float3 T = T4.xyz;
		float t = hit.x;
		// end path on sky
		if (t == 1e30f)
		{
			float3 skyColor = (float3)(0.7f, 0.7f, 1.2f);
			accumulator[pixelIdx] += (float4)(T * skyColor, 1);
			continue;
		}
		// fetch geometry at intersection point
		uint primIdx = as_uint( hit.w );
		uint instIdx = primIdx >> 24;
		uint vertIdx = (primIdx & 0xffffff) * 3;
		const global float4* vdata = instIdx == 0 ? bistroVerts : dragonVerts;
		const global struct Instance* inst = instances + instIdx;
		float4 v0 = vdata[vertIdx];
		uint materialType = as_uint( v0.w ) >> 24;
		float brdfPDF = T4.w;
		float3 D = D4.xyz;
		// end path on light
		if (materialType == MATERIAL_LIGHT)
		{
			float MISweight;
			if (pathState & PATH_LAST_SPECULAR)
			{
				// we came via a mirror; there is no alternative technique.
				MISweight = 1;
			}
			else
			{
				// two techniques could have taken us here; apply MIS.
				float lightPDF = LightPDF( D4.w, D );
				MISweight = 1 / (lightPDF + brdfPDF);
			}
			accumulator[pixelIdx] += (float4)(T * MISweight * lightColor, 1);
			continue;
		}
		// apply postponed hemisphere PDF
		T *= 1.0f / brdfPDF;
		// generate four random numbers
		float r0, r1, r2, r3;
		if (depth == 0 && sampleIdx < 4)
		{
			float2 noise0 = Noise( pixelIdx % height, pixelIdx / height, sampleIdx * 2 );
			float2 noise1 = Noise( pixelIdx % height, pixelIdx / height, sampleIdx * 2 + 1 );
			r0 = noise0.x, r1 = noise0.y;
			r2 = noise1.x, r3 = noise1.y;
		}
		else
		{
			r0 = RandomFloat( &seed ), r1 = RandomFloat( &seed );
			r2 = RandomFloat( &seed ), r3 = RandomFloat( &seed );
		}
		// prepare data for bounce
		float3 vert0 = v0.xyz, vert1 = vdata[vertIdx + 1].xyz, vert2 = vdata[vertIdx + 2].xyz;
		float3 I = O4.xyz + t * D;
		float3 N = fast_normalize( TransformVector( cross( vert1 - vert0, vert2 - vert0 ), inst->transform ) );
		if (dot( N, D ) > 0) N *= -1;
		float3 materialColor;
		if (instIdx == 0) materialColor = rgb32_to_vec3( as_uint( v0.w ) ); else
		{
			if (instIdx == 66) materialColor = (float3)( 1, 1, 0 ), materialType = MATERIAL_SPECULAR;
			else if (instIdx & 1) materialColor = (float3)( 0.13f, 0.13f, 0.16f ), materialType = MATERIAL_SPECULAR; 
			else materialColor = (float3)( 1.0f );
		}
		float3 BRDF = materialColor * INVPI; // lambert BRDF: albedo / pi
		// direct illumination: next event estimation
		if (materialType != MATERIAL_SPECULAR)
		{
			float3 P = lightPos + (float3)(r0 * 9.0f - 4.5f, 0, r1 * 5.0f - 2.5f);
			float3 L = P - I;
			float dist2 = dot( L, L ), dist = sqrt( dist2 );
			L *= native_recip( dist );
			float NdotL = dot( N, L );
			if (NdotL > 0)
			{
				// use MIS pdf to calculate potential direct light contribution
				float lightPDF = LightPDF( dist, L );
				float brdfPDF = dot( L, N ) * INVPI;
				float3 contribution = T * lightColor * BRDF * (NdotL / (lightPDF + brdfPDF));
				uint newShadowIdx = atomic_inc( &connectTasks );
				shadowOut[newShadowIdx].T = (float4)(contribution, 0);
				shadowOut[newShadowIdx].O = (float4)(I + L * EPSILON, as_float( pixelIdx ));
				shadowOut[newShadowIdx].D = (float4)(L, dist - 2 * EPSILON);
			}
		}
		// handle pure specular BRDF
		if (depth >= 3) continue;
		if (materialType == MATERIAL_SPECULAR)
		{
			uint newRayIdx = atomic_inc( &extendTasks );
			float3 R = Reflect( D, N );
			raysOut[newRayIdx].T = (float4)(T * materialColor, 1);
			raysOut[newRayIdx].O = (float4)(I + R * EPSILON, as_float( (pixelIdx << 8) + ((depth + 1) << 4) + PATH_LAST_SPECULAR ));
			raysOut[newRayIdx].D = (float4)(R, 1e30f);
		}
		else /* materialType == MATERIAL_DIFFUSE */ if ((pathState & PATH_VIA_DIFFUSE) == 0 )
		{
			uint newRayIdx = atomic_inc( &extendTasks );
			float3 R = CosWeightedDiffReflection( N, r2, r3 );
			float PDF = dot( N, R ) * INVPI;
			T *= dot( N, R ) * BRDF;
			raysOut[newRayIdx].T = (float4)(T, PDF /* for MIS, we postpone the pdf until after light sampling */ );
			raysOut[newRayIdx].O = (float4)(I + R * EPSILON, as_float( (pixelIdx << 8) + ((depth + 1) << 4) + PATH_VIA_DIFFUSE ));
			raysOut[newRayIdx].D = (float4)(R, 1e30f);
		}
	}
}

// syncing counters: we generated extensions; those will need shading too.
void kernel UpdateCounters2()
{
	if (get_global_id( 0 ) != 0) return;
	shadeTasks = extendTasks;
}

// connect: trace shadow rays and deposit their potential contribution to the pixels
// if not occluded.
void kernel Connect( global float4* accumulator, global struct Potential* shadowIn )
{
	while (1)
	{
		// obtain task - see note on worker threads in Extend
		if (connectTasks < 1) break;
		const int rayId = atomic_dec( &connectTasks ) - 1;
		if (rayId < 0) break;
		const float4 T4 = shadowIn[rayId].T, O4 = shadowIn[rayId].O, D4 = shadowIn[rayId].D;
		const float4 rD4 = native_recip( D4 );
		if (isoccluded_tlas( O4, D4, rD4, D4.w )) continue;
		accumulator[as_uint( O4.w )] += T4;
	}
}

// finalize: convert the accumulated values into final pixel values.
// NOTE: rendering result is emitted to global uint array, which needs to be copied back 
// to the host. This is not efficient. A proper scheme should use OpenGL / D3D / Vulkan 
// interop do write directly to a texture.
void kernel Finalize( global float4* accumulator, const float scale, global uint* pixels )
{
	const uint x = get_global_id( 0 ), y = get_global_id( 1 );
	const uint pixelIdx = x + y * get_global_size( 0 );
	const float4 p = accumulator[pixelIdx] * scale;
	int3 rgb = convert_int3( min( sqrt( p.xyz ), (float3)(1.0f, 1.0f, 1.0f) ) * 255.0f );
	pixels[pixelIdx] = (rgb.x << 16) + (rgb.y << 8) + rgb.z;
}
// This version of Finalize does the proper thing: pixels are plotted straight to an OpenGL
// texture, which bypasses transfers to and from the host altogether.
void kernel FinalizeGL( global float4* accumulator, const float scale, write_only image2d_t pixels )
{
	const uint x = get_global_id( 0 ), y = get_global_id( 1 );
	const uint pixelIdx = x + y * get_global_size( 0 );
	const float4 p = sqrt( accumulator[pixelIdx] * scale );
	write_imagef( pixels, (int2)(x, y), p );
}