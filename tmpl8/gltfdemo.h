// Template, 2024 IGAD Edition
// Get the latest version from: https://github.com/jbikker/tmpl8
// IGAD/NHTV/BUAS/UU - Jacco Bikker - 2006-2024

#pragma once
#pragma warning( disable : 4324 ) // padding warning

#define MAX_BLAS_COUNT	256

namespace Tmpl8
{

struct RenderData
{
	// View pyramid for a pinhole camera
	float4 eye = float4( 0, 30, 0, 0 );
	float4 view = float4( -1, 0, 0, 0 );
	float4 C, p0, p1, p2;
};

class GLTFDemo : public TheApp
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
	// data members
	int2 mousePos;
	RenderData rd;
	// host-side mesh data
	tinyscene::Scene scene;
	// OpenCL kernels
	Kernel* init = 0;
	Kernel* render = 0;
	// OpenCL buffers
	Buffer* pixels = 0;
	Buffer* instances = 0;
	Buffer* blasNode[MAX_BLAS_COUNT];
	Buffer* blasIdx[MAX_BLAS_COUNT];
	Buffer* blasTri[MAX_BLAS_COUNT];
	Buffer* tlasNode = 0;
	Buffer* tlasIdx = 0;
	Buffer* materials = 0;
	Buffer* textures = 0;
	Buffer* texels = 0;
	Buffer* skyPixels = 0;
};

} // namespace Tmpl8