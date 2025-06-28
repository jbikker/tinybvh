#define SCRWIDTH 800
#define SCRHEIGHT 600
#include "external/fenster.h"
#include "tiny_bvh.h"
#include <fstream>
#include <thread>
#define TINYSCENE_USE_CUSTOM_VECTOR_TYPES
namespace tinyscene // override tinyscene's vector types with tinybvh's for easier interop
{
using ts_int2 = tinybvh::bvhint2;
using ts_int3 = tinybvh::bvhint3;
using ts_uint2 = tinybvh::bvhuint2;
using ts_uint3 = tinybvh::bvhuint3;
using ts_uint4 = tinybvh::bvhuint4;
using ts_vec2 = tinybvh::bvhvec2;
using ts_vec3 = tinybvh::bvhvec3;
using ts_vec4 = tinybvh::bvhvec4;
using ts_mat4 = tinybvh::bvhmat4;
}
#include "tiny_scene.h"
using namespace tinybvh;
using namespace tinyscene;

// scene data
Scene scene;

// useful constants
#define PI			3.14159265358979323846264f
#define INVPI		0.31830988618379067153777f
#define INV2PI		0.15915494309189533576888f
#define TWOPI		6.28318530717958647692528f

// view pyramid for a pinhole camera
static bvhvec3 eye( -15.24f, 21.5f, 2.54f ), p1, p2, p3;
static bvhvec3 view = tinybvh_normalize( bvhvec3( 0.826f, -0.438f, -0.356f ) );

// Application start: Initialize scene.
void Init()
{
	// Load a scene from a GLTF file using tinyscene.
	// scene.SetBVHDefault( BVH_RIGID );
	bvhmat4 Tdrone, Ttree;
	Tdrone[0] = Tdrone[5] = Tdrone[10] = 0.03f;
	Ttree[0] = Ttree[5] = Ttree[10] = 2.0f, Ttree[3] = 5.0f, Ttree[7] = -2.9f;
	scene.AddScene( "./testdata/drone/scene.gltf", Tdrone );
	scene.AddScene( "./testdata/mangotree/scene.gltf", Ttree );
	scene.SetSkyDome( new SkyDome( "./testdata/sky_15.hdr" ) );
	// Load camera position / direction from file.
	std::fstream t = std::fstream{ "camera.bin", t.binary | t.in };
	if (!t.is_open()) return;
	t.read( (char*)&eye, sizeof( eye ) );
	t.read( (char*)&view, sizeof( view ) );
	t.close();
}

// Camera interaction: WASD+RF for translation; cursor keys for rotation.
bool UpdateCamera( float delta_time_s, fenster& f )
{
	bvhvec3 right = tinybvh_normalize( tinybvh_cross( bvhvec3( 0, 1, 0 ), view ) ), up = 0.8f * tinybvh_cross( view, right );
	float moved = 0, spd = 10.0f * delta_time_s;
	if (f.keys['A'] || f.keys['D']) eye += right * (f.keys['D'] ? spd : -spd), moved = 1;
	if (f.keys['W'] || f.keys['S']) eye += view * (f.keys['W'] ? spd : -spd), moved = 1;
	if (f.keys['R'] || f.keys['F']) eye += up * 2.0f * (f.keys['R'] ? spd : -spd), moved = 1;
	if (f.keys[20]) view = tinybvh_normalize( view + right * -0.1f * spd ), moved = 1;
	if (f.keys[19]) view = tinybvh_normalize( view + right * 0.1f * spd ), moved = 1;
	if (f.keys[17]) view = tinybvh_normalize( view + up * -0.1f * spd ), moved = 1;
	if (f.keys[18]) view = tinybvh_normalize( view + up * 0.1f * spd ), moved = 1;
	// recalculate right, up
	right = tinybvh_normalize( tinybvh_cross( bvhvec3( 0, 1, 0 ), view ) ), up = 0.8f * tinybvh_cross( view, right );
	bvhvec3 C = eye + 1.2f * view;
	p1 = C - right + up, p2 = C + right + up, p3 = C - right - up;
	return moved > 0;
}

// Helper function to obtain HDR sky sample from the loaded scene.
bvhvec3 SampleSky( const bvhvec3& D )
{
	SkyDome* sky = Scene::sky;
	if (!sky) return 0;
	const float p = atan2f( D.z, D.x );
	const uint32_t u = (uint32_t)(sky->width * (p + (p < 0 ? PI * 2 : 0)) * INV2PI - 0.5f);
	const uint32_t v = (uint32_t)(sky->height * acosf( D.y ) * INVPI - 0.5f);
	const uint32_t idx = tinybvh_min( u + v * sky->width, (uint32_t)(sky->width * sky->height - 1) );
	const bvhvec3 sample = sky->pixels[idx];
	return bvhvec3( sample.z, sample.y, sample.x );
}

// Helper function to obtain detailed shading data for the hitpoint.
void GetShadingData( const Ray& ray, bvhvec3& albedo, float& alpha, bvhvec3& N, bvhvec3& iN )
{
	const uint32_t primIdx = ray.hit.prim;
	const uint32_t instIdx = ray.hit.inst;
	const BLASInstance& instance = scene.instPool[instIdx];
	const uint32_t meshIdx = instance.blasIdx;
	const FatTri& triangle = scene.meshPool[meshIdx]->triangles[primIdx];
	const uint32_t matIdx = triangle.material;
	const Material* material = Scene::materials[matIdx];
	// albedo at hit point - ignoring detail textures and MIP-maps for now.
	const float u = ray.hit.u, v = ray.hit.v, w = 1 - u - v; // barycentrics
	float tu = u * triangle.u1 + v * triangle.u2 + w * triangle.u0;
	float tv = u * triangle.v1 + v * triangle.v2 + w * triangle.v0;
	tu -= floorf( tu ), tv -= floorf( tv );
	alpha = 1;
	if (material->color.textureID == -1) albedo = material->color.value; else
	{
		Texture* tex = Scene::textures[material->color.textureID];
		const int iu = (int)(tu * tex->width);
		const int iv = (int)(tv * tex->height);
		if (tex->fdata) /* HDR */ albedo = tex->fdata[iu + iv * tex->width]; else
		{
			const ts_uchar4 pixel = tex->idata[iu + iv * tex->width];
			albedo = bvhvec3( (float)pixel.x, (float)pixel.y, (float)pixel.z ) * (1.0f / 256.0f);
			alpha = (pixel.x + pixel.y + pixel.z > 5) ? 1.0f : 0.0f;
		}
	}
	// geometric normal, transformed to world space
	N = bvhvec3( triangle.Nx, triangle.Ny, triangle.Nz );
	N = tinybvh_normalize( tinybvh_transform_vector( N, instance.transform ) );
	if (tinybvh_dot( N, ray.D ) > 0) N *= -1;
	// interpolated normal, modified by normal map, transformed to world space
	iN = u * triangle.vN1 + v * triangle.vN2 + (1 - u - v) * triangle.vN0;
	if (material->normals.textureID != -1)
	{
		Texture* tex = Scene::textures[material->normals.textureID];
		const int iu = (int)(tu * tex->width);
		const int iv = (int)(tv * tex->height);
		const ts_uchar4 pixel = tex->idata[iu + iv * tex->width];
		bvhvec3 mN( (float)pixel.x, (float)pixel.y, (float)pixel.z );
		mN *= 1.0f / 128.0f, mN += -1.0f;
		iN = mN.x * triangle.T + mN.y * triangle.B + mN.z * iN;
	}
	iN = tinybvh_normalize( tinybvh_transform_vector( iN, instance.transform ) );
	if (tinybvh_dot( iN, N ) < 0) iN *= -1;
}

// Main ray tracing function: Calculates the (floating point) color for a pixel.
bvhvec3 Trace( Ray& ray, const int depth = 0 )
{
	bvhvec3 albedo, N, iN, I;
	for (int i = 0; i < 8; i++)
	{
		Scene::tlas->Intersect( ray );
		if (ray.hit.t >= 10000) return SampleSky( ray.D );
		I = ray.O + ray.D * ray.hit.t;
		float alpha;
		GetShadingData( ray, albedo, alpha, N, iN );
		if (alpha > 0) break;
		ray.O = I + ray.D * 0.0001f; // we hit an alpha masked pixel, continue
		ray.hit.t = 1e34f;
	}
	static bvhvec3 L = tinybvh_normalize( bvhvec3( 2, 4, 5 ) );
	const bvhvec3 R = ray.D - 2 * tinybvh_dot( iN, ray.D ) * iN;
	bvhvec3 indirect = SampleSky( R );
	return albedo * (0.5f * indirect + tinybvh_max( 0.2f, tinybvh_dot( iN, L ) ));
	// return (iN + 1) * 0.5f;
}

// Render 20x20 pixel tiles using all cores.
static std::atomic<int> jobCount( 0 );
void WorkerThread( uint32_t* buf )
{
	int xtiles = SCRWIDTH / 20, ytiles = SCRHEIGHT / 20, tile;
tileloop:
	if ((tile = --jobCount) < 0) return; else tile = (xtiles * ytiles - 1) - tile;
	const int tx = tile % xtiles, ty = tile / xtiles;
	for (int y = 0; y < 20; y++) for (int x = 0; x < 20; x++) // trace 400 primary rays
	{
		const int pixelx = tx * 20 + x, pixely = ty * 20 + y;
		const float u = (float)pixelx / SCRWIDTH, v = (float)pixely / SCRHEIGHT;
		const bvhvec3 D = tinybvh_normalize( p1 + u * (p2 - p1) + v * (p3 - p1) - eye );
		Ray ray( eye, D );
		const bvhvec3 E = tinybvh_min( Trace( ray ), bvhvec3( 1 ) ) * 255.0f;
		buf[pixelx + pixely * SCRWIDTH] = (int)E.x + ((int)E.y << 8) + ((int)E.z << 16);
	}
	goto tileloop;
}

// Application Tick, exectuted once per frame.
void Tick( float delta_time_s, fenster& f, uint32_t* buf )
{
	static unsigned threadCount = std::thread::hardware_concurrency();
	UpdateCamera( delta_time_s, f );
	scene.UpdateSceneGraph( delta_time_s );
	jobCount = SCRWIDTH * SCRHEIGHT / 400;
	std::vector<std::thread> threads;
#ifdef _DEBUG
	for (unsigned i = 0; i < threadCount; i++) WorkerThread( buf ); // single thread in debug.
#else
	for (unsigned i = 0; i < threadCount; i++) threads.emplace_back( &WorkerThread, buf );
#endif
	for (auto& thread : threads) thread.join();
	// print frame time / rate in window title
	char title[50];
	static float fps = 20;
	fps = 0.98f * fps + 0.02f * (1.0f / delta_time_s);
	sprintf( title, "tiny_bvh %.2f Hz", fps );
	fenster_update_title( &f, title );
}

// Application Shutdown.
void Shutdown()
{
	// save camera position / direction to file
	std::fstream s = std::fstream{ "camera.bin", s.binary | s.out };
	s.write( (char*)&eye, sizeof( eye ) );
	s.write( (char*)&view, sizeof( view ) );
	s.close();
}