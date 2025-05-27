// Template, 2024 IGAD Edition
// Get the latest version from: https://github.com/jbikker/tmpl8
// IGAD/NHTV/BUAS/UU - Jacco Bikker - 2006-2024

#pragma once
#pragma warning( disable : 4324 ) // padding warning
#define DRAGONS 100

namespace Tmpl8
{
// View pyramid for a pinhole camera
struct RenderData
{
	float4 eye = float4( 0, 30, 0, 0 ), view = float4( -1, 0, 0, 0 ), C, p0, p1, p2;
	uint frameIdx, dummy1, dummy2, dummy3;
};

class Game : public TheApp
{
public:
	// game flow methods
	void Init();
	void Tick( float );
	bool UpdateCamera( float );
	void Shutdown() { /* implement if you want to do something on exit */ }
	// input handling
	void MouseUp( int ) { /* implement if you want to detect mouse button presses */ }
	void MouseDown( int ) { /* implement if you want to detect mouse button presses */ }
	void MouseMove( int x, int y ) { mousePos.x = x, mousePos.y = y; }
	void MouseWheel( float ) { /* implement if you want to handle the mouse wheel */ }
	void KeyUp( int ) { /* implement if you want to handle keys */ }
	void KeyDown( int ) { /* implement if you want to handle keys */ }
	// helpers
	float3 splinePos( float t )
	{
		uint s = (uint)t;
		t -= (float)s;
		const float3 P = spline[s - 1], Q = spline[s], R = spline[s + 1], S = spline[s + 2];
		const float3 a = 2.0f * Q, b = R - P, c = 2.0f * P - 5.0f * Q + 4.0f * R - S;
		return 0.5f * (a + (b * t) + (c * t * t) + ((3.0f * Q - 3.0f * R + S - P) * t * t * t));
	}
	void splineCam( float t )
	{
		uint s = (uint)t;
		t -= (float)s, s *= 2;
		const float3 Pp = cam[s - 2], Qp = cam[s], Rp = cam[s + 2], Sp = cam[s + 4];
		const float3 Pt = cam[s - 1], Qt = cam[s + 1], Rt = cam[s + 3], St = cam[s + 5];
		float3 a = 2.0f * Qp, b = Rp - Pp, c = 2.0f * Pp - 5.0f * Qp + 4.0f * Rp - Sp;
		rd.eye = 0.5f * (a + (b * t) + (c * t * t) + ((3.0f * Qp - 3.0f * Rp + Sp - Pp) * t * t * t));
		a = 2.0f * Qt, b = Rt - Pt, c = 2.0f * Pt - 5.0f * Qt + 4.0f * Rt - St;
		float3 target = 0.5f * (a + (b * t) + (c * t * t) + ((3.0f * Qt - 3.0f * Rt + St - Pt) * t * t * t));
		rd.view = normalize( target - float3( rd.eye ) );
	}
	void AddMesh( const char* file, float scale = 1, float3 pos = {}, int c = 0, int N = 0 )
	{
		std::fstream s{ file, s.binary | s.in }; s.read( (char*)&N, 4 );
		float4* data = (float4*)tinybvh::malloc64( (N + triCount) * 48 );
		if (verts) memcpy( data, verts, triCount * 48 ), tinybvh::free64( verts );
		verts = data, s.read( (char*)verts + triCount * 48, N * 48 ), triCount += N;
		for (int* b = (int*)verts + (triCount - N) * 12, i = 0; i < N * 3; i++)
			*(float3*)b = *(float3*)b * scale + pos, b[3] = c ? c : b[3], b += 4;
	}
	void AddQuad( const float3 pos, const float w, const float d, int c )
	{
		float4* data = (float4*)tinybvh::malloc64( (triCount + 2) * 48 );
		if (verts) memcpy( data + 6, verts, triCount * 48 ), tinybvh::free64( verts );
		data[0] = float3( -w, 0, -d ), data[1] = float3( w, 0, -d );
		data[2] = float3( w, 0, d ), data[3] = float3( -w, 0, -d ), verts = data;
		data[4] = float3( w, 0, d ), data[5] = float3( -w, 0, d ), triCount += 2;
		for (int i = 0; i < 6; i++) data[i] = 0.5f * data[i] + float4( pos, 0 ), data[i].w = *(float*)&c;
	}
	void LoadBlueNoise()
	{
		std::fstream s{ "./testdata/blue_noise_128x128x8_2d.raw", s.binary | s.in };
		s.read( (char*)blueNoise, 128 * 128 * 8 * 4 );
	}
	// data members
	int2 mousePos;
	uint32_t frameIdx = 0, spp = 0;
	RenderData rd;
	// host-side mesh data
	BLASInstance instance[DRAGONS + 2];
	float4* verts = 0;
	float4* dragonVerts = 0;
	uint32_t triCount = 0;
	uint32_t dragonTriCount = 0;
	// host-side acceleration structures
	BVH_GPU tlas;
	BVH8_CWBVH bistro;
	BVH8_CWBVH dragon;
	BVHBase* blasList[2] = { &bistro, &dragon };
	size_t computeUnits;
	uint32_t* blueNoise = new uint32_t[128 * 128 * 8];
	// OpenCL kernels
	Kernel* init;
	Kernel* clear;
	Kernel* rayGen;
	Kernel* extend;
	Kernel* shade;
	Kernel* updateCounters1;
	Kernel* updateCounters2;
	Kernel* traceShadows;
	Kernel* finalize;
	// OpenCL buffers
	Buffer* pixels;
	Buffer* accumulator;
	Buffer* raysIn;
	Buffer* raysOut;
	Buffer* connections;
	Buffer* bistroNodes = 0;
	Buffer* bistroTris = 0;
	Buffer* bistroVerts;
	Buffer* dragonNodes = 0;
	Buffer* dragonTris = 0;
	Buffer* drVerts;
	Buffer* noise = 0;
	Buffer* tlasNodes = 0;
	Buffer* tlasIndices = 0;
	Buffer* blasInstances = 0;
	// splines
	float3 spline[24] = {
		float3( -3.378f, 0, -38.44f ),float3( -1.91f, 0, -36.10f ),
		float3( -1.775f, 0, -31.95f ),float3( -3.15f, 0, -28.50f ),
		float3( -6.027f, 0, -24.12f ),float3( -9.32f, 0, -19.11f ),
		float3( -12.40f, 0, -15.25f ),float3( -15.40f, 0, -12.02f ),
		float3( -18.42f, 0, -7.97f ),float3( -20.30f, 0, -3.28f ),
		float3( -19.36f, 0, 0.809f ),float3( -16.90f, 0, 2.53f ),
		float3( -13.10f, 0, 4.788f ),float3( -8.25f, 0, 6.87f ),
		float3( -3.060f, 0, 9.029f ),float3( 5.6988f, 0, 12.67f ),
		float3( 12.176f, 0, 15.38f ),float3( 17.394f, 0, 18.44f ),
		float3( 20.821f, 0, 21.96f ),float3( 25.406f, 0, 25.10f ),
		float3( 29.196f, 0, 23.99f ),float3( 31.381f, 0, 19.60f ),
		float3( 28.708f, 0, 14.89f ),float3( 21.821f, 0, 13.16f )
	};
	float3 cam[26] = {
		float3( 1.86f, -7.21f, -31.52f ), float3( 0.97f, -7.25f, -31.08f ),
		float3( -1.02f, -3.32f, -29.35f ), float3( -1.77f, -3.69f, -28.79f ),
		float3( -7.80f, -4.41f, -21.00f ), float3( -8.15f, -5.24f, -20.57f ),
		float3( -14.72f, -6.29f, -15.06f ), float3( -14.78f, -6.97f, -14.32f ),
		float3( -20.33f, -6.91f, -10.36f ), float3( -19.81f, -7.57f, -9.81f ),
		float3( -23.23f, -7.53f, -3.91f ), float3( -22.36f, -8.02f, -3.95f ),
		float3( -21.29f, -8.22f, 3.83f ), float3( -20.51f, -8.38f, 3.23f ),
		float3( -15.81f, -8.34f, 6.93f ), float3( -15.37f, -8.50f, 6.05f ),
		float3( -11.27f, -8.34f, 8.90f ), float3( -10.94f, -8.50f, 7.97f ),
		float3( -1.07f, -5.07f, 9.41f ), float3( -1.98f, -5.29f, 9.051f ),
		float3( 9.77f, -7.38f, 13.64f ), float3( 8.84f, -7.33f, 13.28f ),
		float3( 12.93f, 0.06f, 19.42f ), float3( 12.28f, -0.48f, 18.89f ),
		float3( 19.39f, 8.09f, 23.33f ), float3( 18.89f, 7.33f, 22.92f ),
	};
};
} // namespace Tmpl8