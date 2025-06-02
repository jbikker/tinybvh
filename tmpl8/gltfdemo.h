// Template, 2024 IGAD Edition
// Get the latest version from: https://github.com/jbikker/tmpl8
// IGAD/NHTV/BUAS/UU - Jacco Bikker - 2006-2024

#pragma once
#pragma warning( disable : 4324 ) // padding warning

namespace Tmpl8
{

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
	bvhvec3 eye = bvhvec3( -15.24f, 21.5f, 2.54f );
	bvhvec3 view = tinybvh_normalize( bvhvec3( 0.826f, -0.438f, -0.356f ) );
	bvhvec3 p1, p2, p3;
	// host-side mesh data
	tinyscene::Scene scene;
	// OpenCL kernels
	Kernel* init = 0;
	Kernel* render = 0;
	// OpenCL buffers
	Buffer* pixels = 0;
	Buffer* instances = 0;
	Buffer* blasNode = 0;
	Buffer* blasIdx = 0;
	Buffer* blasTri = 0;
	Buffer* blasFatTri = 0;
	Buffer* blasOffsets = 0;
	Buffer* tlasNode = 0;
	Buffer* tlasIdx = 0;
	Buffer* materials = 0;
	Buffer* texels = 0;
	Buffer* skyPixels = 0;
};

} // namespace Tmpl8