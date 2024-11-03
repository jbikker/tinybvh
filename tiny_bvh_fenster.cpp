#include "external/fenster.h" // https://github.com/zserge/fenster

#define TINYBVH_IMPLEMENTATION
#include "tiny_bvh.h"

using namespace tinybvh;

bvhvec4 triangles[259 /* level 3 */ * 6 * 2 * 49 * 3]{};
int verts = 0;
BVH bvh;

static int frame_num;
static uint64_t frame_time_total;

void sphere_flake( float x, float y, float z, float s, int d = 0 )
{
	// procedural tesselated sphere flake object
#define P(F,a,b,c) p[i+F*64]={(float)a ,(float)b,(float)c}
	bvhvec3 p[384], pos( x, y, z ), ofs( 3.5 );
	for (int i = 0, u = 0; u < 8; u++) for (int v = 0; v < 8; v++, i++)
		P( 0, u, v, 0 ), P( 1, u, 0, v ), P( 2, 0, u, v ),
		P( 3, u, v, 7 ), P( 4, u, 7, v ), P( 5, 7, u, v );
	for (int i = 0; i < 384; i++) p[i] = normalize( p[i] - ofs ) * s + pos;
	for (int i = 0, side = 0; side < 6; side++, i += 8)
		for (int u = 0; u < 7; u++, i++) for (int v = 0; v < 7; v++, i++)
			triangles[verts++] = p[i], triangles[verts++] = p[i + 8],
			triangles[verts++] = p[i + 1], triangles[verts++] = p[i + 1],
			triangles[verts++] = p[i + 9], triangles[verts++] = p[i + 8];
	if (d < 3) sphere_flake( x + s * 1.55f, y, z, s * 0.5f, d + 1 );
	if (d < 3) sphere_flake( x - s * 1.5f, y, z, s * 0.5f, d + 1 );
	if (d < 3) sphere_flake( x, y + s * 1.5f, z, s * 0.5f, d + 1 );
	if (d < 3) sphere_flake( x, x - s * 1.5f, z, s * 0.5f, d + 1 );
	if (d < 3) sphere_flake( x, y, z + s * 1.5f, s * 0.5f, d + 1 );
	if (d < 3) sphere_flake( x, y, z - s * 1.5f, s * 0.5f, d + 1 );
}

void Init()
{
	// generate a sphere flake scene
	sphere_flake( 0, 0, 0, 1.5f );

	// build a BVH over the scene
	bvh.Build( (bvhvec4*)triangles, verts / 3 );
}

void Tick( uint32_t* buf )
{
	uint64_t frame_start_time = fenster_time();
	// setup camera
	bvhvec3 eye( -3.5f, -1.5f, -6.5f ), view = normalize( bvhvec3( 3, 1.5f, 5 ) );
	bvhvec3 right = normalize( cross( bvhvec3( 0, 1, 0 ), view ) );
	bvhvec3 up = 0.8f * cross( view, right ), C = eye + 2 * view;
	bvhvec3 p1 = C - right + up, p2 = C + right + up, p3 = C - right - up;
	// generate primary rays in a buffer
	int N = 0;
	Ray* rays = new Ray[SCRWIDTH * SCRHEIGHT * 16];
	for (int y = 0; y < SCRHEIGHT; y++) for (int x = 0; x < SCRWIDTH; x++)
	{
		for (int s = 0; s < 16; s++) // 16 samples per pixel
		{
			float u = (float)(x * 4 + (s & 3)) / (SCRWIDTH * 4);
			float v = (float)(y * 4 + (s >> 2)) / (SCRHEIGHT * 4);
			bvhvec3 P = p1 + u * (p2 - p1) + v * (p3 - p1);
			rays[N++] = Ray( eye, normalize( P - eye ) );
		}
	}
	// trace primary rays
	for (int i = 0; i < N; i++) bvh.Intersect( rays[i] );
	// visualize result
	for (int i = 0, y = 0; y < SCRHEIGHT; y++) for (int x = 0; x < SCRWIDTH; x++)
	{
		float avg = 0;
		for (int s = 0; s < 16; s++, i++) if (rays[i].hit.t < 1000)
		{
			int primIdx = rays[i].hit.prim;
			bvhvec3 v0 = *(bvhvec3*)&triangles[primIdx * 3 + 0];
			bvhvec3 v1 = *(bvhvec3*)&triangles[primIdx * 3 + 1];
			bvhvec3 v2 = *(bvhvec3*)&triangles[primIdx * 3 + 2];
			bvhvec3 N = normalize( cross( v1 - v0, v2 - v0 ) );
			avg += fabs( dot( N, normalize( bvhvec3( 1, 2, 3 ) ) ) );
		}
		int c = (int)(15.9f * avg);
		buf[x + y * SCRWIDTH] = c + (c << 8) + (c << 16);
	}
	uint64_t frame_time = fenster_time() - frame_start_time;
	frame_time_total += frame_time;
	++frame_num;
	fprintf(stderr, "Frame %d time: %d ms, avg: %.2f ms\n", frame_num,
		(int)(frame_time),
		(float)(frame_time_total / (float)frame_num)
	);
}
