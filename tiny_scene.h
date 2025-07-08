/*
The MIT License (MIT)

Copyright (c) 2024-2025, Jacco Bikker / Breda University of Applied Sciences.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

// How to use:
//
// Use this in *one* .c or .cpp
//   #define TINYSCENE_IMPLEMENTATION
//   #include "tiny_scene.h"

// tinyscene can use custom vector types by defining TINYSCENE_USE_CUSTOM_VECTOR_TYPES
// once before inclusion. To define custom vector types create a tinybvh namespace with
// the appropriate using directives, e.g.:
//	 namespace tinyscene
//   {
//     using ts_int2 = math::int2;
//     using ts_int3 = math::int3;
//     using ts_uint2 = math::uint2;
//     using ts_uint2 = math::uint3;
//     using ts_uint2 = math::uint4;
//     using ts_vec2 = math::float2;
//     using ts_vec3 = math::float3;
//     using ts_vec4 = math::float4;
//   }
//
//	 #define TINYSCENE_USE_CUSTOM_VECTOR_TYPES
//   #include <tiny_scene.h>

// tiny_scene.h depends on tiny_obj_loader, tiny_gltf and stb_image.
// If you already implement these elsewhere, use the following defines
// to supress duplicate implementation in this header file.

// #define TINYSCENE_TINYOBJ_AREADY_IMPLEMENTED
// #define TINYSCENE_TINYGLTF_ALREADY_IMPLEMENTED
// #define TINYSCENE_STBIMAGE_ALREADY_IMPLEMENTED

// access syoyo's tiny_obj and tiny_gltf implementations
#if defined TINYSCENE_IMPLEMENTATION && !defined TINYSCENE_TINYOBJ_AREADY_IMPLEMENTED
#define TINYOBJLOADER_IMPLEMENTATION
#endif
#include "tiny_obj_loader.h"
#if defined TINYSCENE_IMPLEMENTATION && !defined TINYSCENE_TINYGLTF_ALREADY_IMPLEMENTED
#define TINYGLTF_IMPLEMENTATION
#endif
#define TINYGLTF_NO_STB_IMAGE_WRITE
#include "tiny_gltf.h"
#if defined TINYSCENE_IMPLEMENTATION && !defined TINYSCENE_STBIMAGE_ALREADY_IMPLEMENTED
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#endif

#ifndef TINY_SCENE_H_
#define TINY_SCENE_H_

// LIGHTHOUSE 2 SCENE MANAGEMENT CODE - QUICK OVERVIEW
//
// This file defines the data structures for the Lighthouse 2 scene graph.
// It is designed to conveniently load and/or construct 3D scenes, using a
// combination of .gtlf / .obj files and extra triangles.
//
// The basis is a scenegraph: a hierarchy of nodes with 4x4 matrix transforms and
// (optionally) a triangle mesh. Meshes have materials and (optionally) data for
// animation.
//
// The data structure closely follows the gltf 2.0 format and supports all types
// of animation in pure CPU code: See Scene::SetPose for details.
//
// The triangle data is optimized for ray tracing rather than rasterization:
// - Triangle data is split in 'vertex only' and 'everything else' (see FatTri);
// - A mesh can have multiple materials. However: when loaded from a gltf file,
//   each mesh will have only one material.
//
// Note about the use of pointers: This is intentionally minimal. Most objects are
// stored in vectors in class Scene; references to these are specified as indices
// in these vectors. E.g.: Mesh::ID stores the index of a mesh in Scene::meshPool.
//
// Architectural limitations: There is very little support for *deleting* anything
// in the scene. Adding this properly may require significant engineering.
// Scene::RemoveNode exists, but it only removes nodes themselves, not any
// linked materials or textures.

#define MIPLEVELCOUNT		5
#define BINTEXFILEVERSION	0x10001001
#define CACHEIMAGES

#define	BVH_DYNAMIC			0	// BVH will be built as tinybvh::BVH / Build, for fast rebuilds
#define BVH_RIGID			1	// BVH will be built as tinybvh::BVH8_CPU / BuildHQ, for fast traversal
#define GPU_DYNAMIC			2	// BVH will be built with tinybvh::BVH_GPU::Build(..) - for refits/rebuilds
#define GPU_RIGID			3	// BVH will be built as tinybvh::BVH8_GPU::BuildHQ(..) - for opacity maps
#define GPU_STATIC			4	// BVH will be built as tinybvh::BVH8_CWBVH::BuildHQ(..) - for fast traversal

namespace tinyscene
{

// aligned memory allocation
// note: formally, size needs to be a multiple of 'alignment', see:
// https://en.cppreference.com/w/c/memory/aligned_alloc.
// EMSCRIPTEN enforces this.
// Copy of the same construct in tinyocl/tinybvh, in a different namespace.
inline size_t make_multiple_of( size_t x, size_t alignment ) { return (x + (alignment - 1)) & ~(alignment - 1); }
#ifdef _MSC_VER // Visual Studio / C11
#define ALIGNED( x ) __declspec( align( x ) )
#define _ALIGNED_ALLOC(alignment,size) _aligned_malloc( make_multiple_of( size, alignment ), alignment );
#define _ALIGNED_FREE(ptr) _aligned_free( ptr );
#else // EMSCRIPTEN / gcc / clang
#define ALIGNED( x ) __attribute__( ( aligned( x ) ) )
#if !defined TINYBVH_NO_SIMD && (defined __x86_64__ || defined _M_X64 || defined __wasm_simd128__ || defined __wasm_relaxed_simd__)
#include <xmmintrin.h>
#define _ALIGNED_ALLOC(alignment,size) _mm_malloc( make_multiple_of( size, alignment ), alignment );
#define _ALIGNED_FREE(ptr) _mm_free( ptr );
#else
#if defined __APPLE__ || defined __aarch64__ || (defined __ANDROID_API__ && (__ANDROID_API__ >= 28))
#define _ALIGNED_ALLOC(alignment,size) aligned_alloc( alignment, make_multiple_of( size, alignment ) );
#elif defined __GNUC__
#ifdef __linux__
#define _ALIGNED_ALLOC(alignment,size) aligned_alloc( alignment, make_multiple_of( size, alignment ) );
#else
#define _ALIGNED_ALLOC(alignment,size) _aligned_malloc( alignment, make_multiple_of( size, alignment ) );
#endif
#endif
#define _ALIGNED_FREE(ptr) free( ptr );
#endif
#endif
inline void* malloc64( size_t size, void* = nullptr ) { return size == 0 ? 0 : _ALIGNED_ALLOC( 64, size ); }
inline void* malloc4k( size_t size, void* = nullptr ) { return size == 0 ? 0 : _ALIGNED_ALLOC( 4096, size ); }
inline void* malloc32k( size_t size, void* = nullptr ) { return size == 0 ? 0 : _ALIGNED_ALLOC( 32768, size ); }
inline void free64( void* ptr, void* = nullptr ) { _ALIGNED_FREE( ptr ); }
inline void free4k( void* ptr, void* = nullptr ) { _ALIGNED_FREE( ptr ); }
inline void free32k( void* ptr, void* = nullptr ) { _ALIGNED_FREE( ptr ); }

#ifndef TINYSCENE_USE_CUSTOM_VECTOR_TYPES

// tiny_scene.h sports its own vector types so it doesn't have a dependency on
// something external. TODO: 'bring your own vector type' as in tiny_bvh.h.

struct ts_vec3;
struct ALIGNED( 16 ) ts_vec4
{
	// vector naming is designed to not cause any name clashes.
	ts_vec4() = default;
	ts_vec4( const float a, const float b, const float c, const float d ) : x( a ), y( b ), z( c ), w( d ) {}
	ts_vec4( const float a ) : x( a ), y( a ), z( a ), w( a ) {}
	ts_vec4( const ts_vec3 & a );
	ts_vec4( const ts_vec3 & a, float b );
	float& operator [] ( const int32_t i ) { return cell[i]; }
	const float& operator [] ( const int32_t i ) const { return cell[i]; }
	union { struct { float x, y, z, w; }; float cell[4]; };
};

struct ALIGNED( 8 ) ts_vec2
{
	ts_vec2() = default;
	ts_vec2( const float a, const float b ) : x( a ), y( b ) {}
	ts_vec2( const float a ) : x( a ), y( a ) {}
	ts_vec2( const ts_vec4 a ) : x( a.x ), y( a.y ) {}
	float& operator [] ( const int32_t i ) { return cell[i]; }
	const float& operator [] ( const int32_t i ) const { return cell[i]; }
	union { struct { float x, y; }; float cell[2]; };
};

struct ts_vec3
{
	ts_vec3() = default;
	ts_vec3( const float a, const float b, const float c ) : x( a ), y( b ), z( c ) {}
	ts_vec3( const float a ) : x( a ), y( a ), z( a ) {}
	ts_vec3( const ts_vec4 a ) : x( a.x ), y( a.y ), z( a.z ) {}
	float& operator [] ( const int32_t i ) { return cell[i]; }
	const float& operator [] ( const int32_t i ) const { return cell[i]; }
	union { struct { float x, y, z; }; float cell[3]; };
};

struct ts_int3
{
	ts_int3() = default;
	ts_int3( const int32_t a, const int32_t b, const int32_t c ) : x( a ), y( b ), z( c ) {}
	ts_int3( const int32_t a ) : x( a ), y( a ), z( a ) {}
	ts_int3( const ts_vec3& a ) { x = (int32_t)a.x, y = (int32_t)a.y, z = (int32_t)a.z; }
	int32_t& operator [] ( const int32_t i ) { return cell[i]; }
	union { struct { int32_t x, y, z; }; int32_t cell[3]; };
};

struct ts_int2
{
	ts_int2() = default;
	ts_int2( const int32_t a, const int32_t b ) : x( a ), y( b ) {}
	ts_int2( const int32_t a ) : x( a ), y( a ) {}
	int32_t x, y;
};

struct ts_uint2
{
	ts_uint2() = default;
	ts_uint2( const uint32_t a, const uint32_t b ) : x( a ), y( b ) {}
	ts_uint2( const uint32_t a ) : x( a ), y( a ) {}
	uint32_t x, y;
};

struct ts_uint4
{
	ts_uint4() = default;
	ts_uint4( const uint32_t a, const uint32_t b, const uint32_t c, const uint32_t d ) : x( a ), y( b ), z( c ), w( d ) {}
	ts_uint4( const uint32_t a ) : x( a ), y( a ), z( a ), w( a ) {}
	uint32_t x, y, z, w;
};

struct ts_mat4
{
	ts_mat4() = default;
	static ts_mat4 scale( const float s ) { ts_mat4 r; r[0] = r[5] = r[10] = s; return r; }
	static ts_mat4 scale( const ts_vec3 s ) { ts_mat4 r; r[0] = s.x, r[5] = s.y, r[10] = s.z; return r; }
	static ts_mat4 translate( const ts_vec3 t ) { ts_mat4 r; r[3] = t.x, r[7] = t.y, r[11] = t.z; return r; }
	float cell[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
	float& operator [] ( const int idx ) { return cell[idx]; }
	const float& operator [] ( const int idx ) const { return cell[idx]; }
	float operator()( const int i, const int j ) const { return cell[i * 4 + j]; }
	float& operator()( const int i, const int j ) { return cell[i * 4 + j]; }
	ts_mat4& operator += ( const ts_mat4& a )
	{
		for (int i = 0; i < 16; i++) cell[i] += a.cell[i];
		return *this;
	}
};

inline ts_vec2 operator-( const ts_vec2& a ) { return ts_vec2( -a.x, -a.y ); }
inline ts_vec3 operator-( const ts_vec3& a ) { return ts_vec3( -a.x, -a.y, -a.z ); }
inline ts_vec4 operator-( const ts_vec4& a ) { return ts_vec4( -a.x, -a.y, -a.z, -a.w ); }
inline ts_vec2 operator+( const ts_vec2& a, const ts_vec2& b ) { return ts_vec2( a.x + b.x, a.y + b.y ); }
inline ts_vec3 operator+( const ts_vec3& a, const ts_vec3& b ) { return ts_vec3( a.x + b.x, a.y + b.y, a.z + b.z ); }
inline ts_vec4 operator+( const ts_vec4& a, const ts_vec4& b ) { return ts_vec4( a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w ); }
inline ts_vec4 operator+( const ts_vec4& a, const ts_vec3& b ) { return ts_vec4( a.x + b.x, a.y + b.y, a.z + b.z, a.w ); }
inline ts_vec2 operator-( const ts_vec2& a, const ts_vec2& b ) { return ts_vec2( a.x - b.x, a.y - b.y ); }
inline ts_vec3 operator-( const ts_vec3& a, const ts_vec3& b ) { return ts_vec3( a.x - b.x, a.y - b.y, a.z - b.z ); }
inline ts_vec4 operator-( const ts_vec4& a, const ts_vec4& b ) { return ts_vec4( a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w ); }
inline void operator+=( ts_vec2& a, const ts_vec2& b ) { a.x += b.x; a.y += b.y; }
inline void operator+=( ts_vec3& a, const ts_vec3& b ) { a.x += b.x; a.y += b.y; a.z += b.z; }
inline void operator+=( ts_vec4& a, const ts_vec4& b ) { a.x += b.x; a.y += b.y; a.z += b.z; a.w += b.w; }
inline ts_vec2 operator*( const ts_vec2& a, const ts_vec2& b ) { return ts_vec2( a.x * b.x, a.y * b.y ); }
inline ts_vec3 operator*( const ts_vec3& a, const ts_vec3& b ) { return ts_vec3( a.x * b.x, a.y * b.y, a.z * b.z ); }
inline ts_vec4 operator*( const ts_vec4& a, const ts_vec4& b ) { return ts_vec4( a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w ); }
inline ts_vec2 operator*( const ts_vec2& a, float b ) { return ts_vec2( a.x * b, a.y * b ); }
inline ts_vec3 operator*( const ts_vec3& a, float b ) { return ts_vec3( a.x * b, a.y * b, a.z * b ); }
inline ts_vec4 operator*( const ts_vec4& a, float b ) { return ts_vec4( a.x * b, a.y * b, a.z * b, a.w * b ); }
inline ts_vec2 operator*( float b, const ts_vec2& a ) { return ts_vec2( b * a.x, b * a.y ); }
inline ts_vec3 operator*( float b, const ts_vec3& a ) { return ts_vec3( b * a.x, b * a.y, b * a.z ); }
inline ts_vec4 operator*( float b, const ts_vec4& a ) { return ts_vec4( b * a.x, b * a.y, b * a.z, b * a.w ); }
inline ts_vec2 operator/( float b, const ts_vec2& a ) { return ts_vec2( b / a.x, b / a.y ); }
inline ts_vec3 operator/( float b, const ts_vec3& a ) { return ts_vec3( b / a.x, b / a.y, b / a.z ); }
inline ts_vec4 operator/( float b, const ts_vec4& a ) { return ts_vec4( b / a.x, b / a.y, b / a.z, b / a.w ); }
inline void operator*=( ts_vec3& a, const float b ) { a.x *= b; a.y *= b; a.z *= b; }

#ifndef TINYSCENE_IMPLEMENTATION

ts_vec4::ts_vec4( const ts_vec3& a ) { x = a.x, y = a.y, z = a.z, w = 0; }
ts_vec4::ts_vec4( const ts_vec3& a, const float b ) { x = a.x, y = a.y, z = a.z, w = b; }

#endif

#endif // TINYSCENE_USE_CUSTOM_VECTOR_TYPES

#include <vector>

struct ts_uchar4
{
	ts_uchar4() = default;
	ts_uchar4( const uint8_t a, const uint8_t b, const uint8_t c, const uint8_t d ) : x( a ), y( b ), z( c ), w( d ) {}
	ts_uchar4( const uint8_t a ) : x( a ), y( a ), z( a ), w( a ) {}
	uint8_t x, y, z, w;
};

struct ts_aabb
{
	ts_aabb() = default;
	ts_vec3 bmin = ts_vec3( 1e30f );
	ts_vec3 bmax = ts_vec3( -1e30f );
	void grow( const ts_vec3& p );
};

class ts_quat // based on https://github.com/adafruit. Only what we actually need.
{
public:
	ts_quat() = default;
	ts_quat( float _w, float _x, float _y, float _z ) : w( _w ), x( _x ), y( _y ), z( _z ) {}
	ts_quat( float _w, ts_vec3 v ) : w( _w ), x( v.x ), y( v.y ), z( v.z ) {}
	float magnitude() const { return sqrtf( w * w + x * x + y * y + z * z ); }
	void ts_normalize() { float m = magnitude(); *this = this->scale( 1 / m ); }
	static ts_quat slerp( const ts_quat& a, const ts_quat& b, const float t );
	ts_quat operator + ( const ts_quat& q ) const { return ts_quat( w + q.w, x + q.x, y + q.y, z + q.z ); }
	ts_quat operator - ( const ts_quat& q ) const { return ts_quat( w - q.w, x - q.x, y - q.y, z - q.z ); }
	ts_quat operator / ( float s ) const { return ts_quat( w / s, x / s, y / s, z / s ); }
	ts_quat operator * ( float s ) const { return scale( s ); }
	ts_quat scale( float s ) const { return ts_quat( w * s, x * s, y * s, z * s ); }
	float w = 1, x = 0, y = 0, z = 0;
};

//  +-----------------------------------------------------------------------------+
//  |  FatTri                                                                     |
//  |  Full triangle data (for shading only), carefully layed out.          LH2'25|
//  +-----------------------------------------------------------------------------+
class FatTri
{
public:
	FatTri() { ::memset( this, 0, sizeof( FatTri ) ); ltriIdx = -1; }
	// 128 bytes of data needed for basic shading with texture and normal interpolation.
	float u0, u1, u2;					// 12 bytes for layer 0 texture coordinates
	int ltriIdx;						// 4, set only for emissive triangles, used for MIS
	float v0, v1, v2;					// 12 bytes for layer 0 texture coordinates
	uint32_t material;					// 4 bytes for triangle material index
	ts_vec3 vN0;						// 12 bytes for vertex0 normal
	float Nx;							// 4 bytes for x-component of geometric triangle normal
	ts_vec3 vN1;						// 12 bytes for vertex1 normal
	float Ny;							// 4 bytes for y-component of geometric triangle normal
	ts_vec3 vN2;						// 12 bytes for vertex2 normal
	float Nz;							// 4 bytes for z-component of geometric triangle normal
	ts_vec3 vertex0; float u0_2;		// vertex 0 position + second layer u0
	ts_vec3 vertex1; float u1_2;		// vertex 1 position + second layer u1
	ts_vec3 vertex2; float u2_2;		// vertex 2 position + second layer u2
	// 64 bytes of data needed for normal mapping, anisotropic, multi-texture etc.
	ts_vec3 T;							// 12 bytes for tangent vector
	float area;							// 4 bytes for triangle area
	ts_vec3 B;							// 12 bytes for bitangent vector
	float invArea;						// 4 bytes for reciprocal triangle area
	ts_vec3 alpha;						// better spot than vertex0..2.w for reasons
	float LOD;							// for MIP mapping
	float v0_2, v1_2, v2_2;				// 12 bytes for second layer texture coordinates
	float dummy4;						// padding
	// total FatTri size: 192 bytes.
	void UpdateArea();
};

//  +-----------------------------------------------------------------------------+
//  |  SkyDome                                                                    |
//  |  Stores data for a HDR sky dome.                                      LH2'25|
//  +-----------------------------------------------------------------------------+
class SkyDome
{
public:
	// constructor / destructor
	SkyDome() = default;
	SkyDome( const char* file ) { Load( file ); }
	void Load( const char* filename, const ts_vec3 scale = { 1.f, 1.f, 1.f } );
	// public data members
	ts_vec3* pixels = nullptr;			// HDR texture data for sky dome
	int width = 0, height = 0;			// width and height of the sky texture
	ts_mat4 worldToLight;				// for PBRT scenes; transform for skydome
};

//  +-----------------------------------------------------------------------------+
//  |  Skin                                                                       |
//  |  Skin data storage.                                                   LH2'25|
//  +-----------------------------------------------------------------------------+
class Skin
{
public:
	Skin( const ::tinygltf::Skin& gltfSkin, const ::tinygltf::Model& gltfModel, const int nodeBase );
	void ConvertFromGLTFSkin( const ::tinygltf::Skin& gltfSkin, const ::tinygltf::Model& gltfModel, const int nodeBase );
	::std::string name;
	int skeletonRoot = 0;
	::std::vector<ts_mat4> inverseBindMatrices, jointMat;
	::std::vector<int> joints; // node indices of the joints
};

//  +-----------------------------------------------------------------------------+
//  |  Mesh                                                                       |
//  |  Mesh data storage.                                                   LH2'25|
//  +-----------------------------------------------------------------------------+
class Mesh
{
public:
	struct Pose { ::std::vector<ts_vec3> positions, normals, tangents; };
	struct AccStruc
	{
		// BVH can be one of four types:
		// tinybvh::BVH for refittable / rebuildable meshes
		// tinybvh::BVH8_CPU for static / rigid geometry (other types when AVX2 is not available)
		// tinybvh::BVH_GPU for refittable / rebuildable meshes - targetted at GPU rendering
		// tinybvh::BVH8_CWBVH for static / rigid geometry - targetted at GPU rendering.
		// Default is dynamic / CPU; rigid is restrictive but much faster to trace.
		uint32_t bvhType = BVH_DYNAMIC;
		union
		{
			tinybvh::BVH* dynamicBVH = 0;
		#if defined BVH_USEAVX2
			tinybvh::BVH8_CPU* rigidBVH;
		#elif defined BVH_USESSE
			tinybvh::BVH4_CPU* rigidBVH;
		#else
			tinybvh::BVH_SoA* rigidBVH;
		#endif
			tinybvh::BVH_GPU* dynamicGPU;
			tinybvh::BVH_GPU* rigidGPU;
			tinybvh::BVH8_CWBVH* staticGPU;
		};
	};
	// constructor / destructor
	Mesh() = default;
	Mesh( const int triCount );
	Mesh( const char* name, const char* dir, const float scale = 1.0f, const bool flatShaded = false );
	Mesh( const ::tinygltf::Mesh& gltfMesh, const ::tinygltf::Model& gltfModel, const ::std::vector<int>& matIdx, const int materialOverride = -1 );
	~Mesh() { /* TODO */ }
	// methods
	void LoadGeometry( const char* file, const char* dir, const float scale = 1.0f, const bool flatShaded = false );
	void LoadGeometryFromOBJ( const ::std::string& fileName, const char* directory, const ts_mat4& transform, const bool flatShaded = false );
	void ConvertFromGTLFMesh( const ::tinygltf::Mesh& gltfMesh, const ::tinygltf::Model& gltfModel, const ::std::vector<int>& matIdx, const int materialOverride );
	void BuildFromIndexedData( const ::std::vector<int>& tmpIndices, const ::std::vector<ts_vec3>& tmpVertices,
		const ::std::vector<ts_vec3>& tmpNormals, const ::std::vector<ts_vec2>& tmpUvs, const ::std::vector<ts_vec2>& tmpUv2s,
		const ::std::vector<ts_vec4>& tmpTs, const ::std::vector<Pose>& tmpPoses,
		const ::std::vector<ts_uint4>& tmpJoints, const ::std::vector<ts_vec4>& tmpWeights, const int materialIdx );
	void BuildMaterialList();
	void CreateOpacityMicroMaps();
	void SetPose( const ::std::vector<float>& weights );
	void SetPose( const Skin* skin );
	// data members
	::std::string name = "unnamed";		// name for the mesh
	::std::string fileName = "";		// file from which the mesh was loaded
	int ID = -1;						// unique ID for the mesh: position in mesh array
	::std::vector<ts_vec4> vertices;	// model vertices, always 3 per triangle: vertices are *not* indexed.
	::std::vector<FatTri> triangles;	// full triangles, to be used for shading
	uint32_t* omaps = 0;				// at N=32: 1024 bit / 128 byte per triangle
	::std::vector<ts_vec3> vertexNormals;	// vertex normals, 1 per vertex
	::std::vector<int> materialList;	// list of materials used by the mesh; used to efficiently track light changes
	::std::vector<ts_vec4> original;	// skinning: base pose; will be transformed into vector vertices
	::std::vector<ts_vec3> origNormal;	// skinning: base pose normals
	::std::vector<ts_uint4> joints;		// skinning: joints
	::std::vector<ts_vec4> weights;		// skinning: joint weights
	::std::vector<Pose> poses;			// morph target data
	bool geomChanged = true;			// triangle data was modified; blas update is needed
	bool hasOpacityMicroMaps = false;	// if true, 5 vec4's per triangle are passed to tinybvh.
	AccStruc blas;						// bottom-level acceleration structure
};

//  +-----------------------------------------------------------------------------+
//  |  Node                                                                       |
//  |  Simple node for construction of a scene graph for the scene.         LH2'25|
//  +-----------------------------------------------------------------------------+
class Node
{
public:
	// constructor / destructor
	Node() = default;
	Node( const int meshIdx, const ts_mat4& transform );
	Node( const ::tinygltf::Node& gltfNode, const int nodeBase, const int meshBase, const int skinBase );
	~Node();
	// methods
	void ConvertFromGLTFNode( const ::tinygltf::Node& gltfNode, const int nodeBase, const int meshBase, const int skinBase );
	void Update( const ts_mat4& T );	// recursively update the transform of this node and its children
	void UpdateTransformFromTRS();		// process T, R, S data to localTransform
	void PrepareLights();				// create light trianslges from detected emissive triangles
	void UpdateLights();				// fix light triangles when the transform changes
	// data members
	::std::string name;					// node name as specified in the GLTF file
	ts_vec3 translation = { 0 };		// T
	ts_quat rotation;					// R
	ts_vec3 scale = ts_vec3( 1 );		// S
	ts_mat4 matrix;						// object transform
	ts_mat4 localTransform;				// = matrix * T * R * S, in case of animation
	ts_mat4 combinedTransform;			// transform combined with ancestor transforms
	int ID = -1;						// unique ID for the node: position in node array
	int meshID = -1;					// id of the mesh this node refers to (if any, -1 otherwise)
	int skinID = -1;					// id of the skin this node refers to (if any, -1 otherwise)
	::std::vector<float> weights;		// morph target weights
	bool hasLights = false;				// true if this instance uses an emissive material
	bool morphed = false;				// node mesh should update pose
	bool transformed = false;			// local transform of node should be updated
	bool treeChanged = false;			// this node or one of its children got updated
	::std::vector<int> childIdx;		// child nodes of this node
protected:
	int instanceID = -1;				// for mesh nodes: location in the instance array. For internal use only.
};

//  +-----------------------------------------------------------------------------+
//  |  Material                                                                   |
//  |  Full material definition, which contains everything that can be read from  |
//  |  a gltf file. Will need to be digested to something more practical for      |
//  |  rendering: For that there is 'Material'.                             LH2'25|
//  +-----------------------------------------------------------------------------+
class Material
{
public:
	enum { DISNEYBRDF = 1, LAMBERTBSDF, /* add extra here */ };
	struct vec3Value
	{
		// ts_vec3Value / ScalarValue: all material parameters can be spatially variant or invariant.
		// If a map is used, this map may have an offset and scale. The map values may also be
		// scaled, to facilitate parameter map reuse.
		vec3Value() = default;
		vec3Value( const float f ) : value( ts_vec3( f ) ) {}
		vec3Value( const ts_vec3 f ) : value( f ) {}
		ts_vec3 value = ts_vec3( 1e-32f );		// default value if map is absent; 1e-32 means: not set
		float dummy;							// because ts_vec3 is 12 bytes.
		int textureID = -1;						// texture ID; 'value'field is used if -1
		float scale = 1;						// map values will be scaled by this
		ts_vec2 uvscale = ts_vec2( 1 );			// uv coordinate scale
		ts_vec2 uvoffset = ts_vec2( 0 );		// uv coordinate offset
		ts_uint2 size = ts_uint2( 0 );			// texture dimensions
		// a parameter that has not been specified has a -1 textureID and a 1e-32f value
		bool Specified() { return value.x != 1e32f || value.y != 1e32f || value.z != 1e32f || textureID != -1; }
		ts_vec3& operator()() { return value; }
	};
	struct ScalarValue
	{
		ScalarValue() = default;
		ScalarValue( const float f ) : value( f ) {}
		float value = 1e-32f;				// default value if map is absent; 1e32 means: not set
		int textureID = -1;					// texture ID; -1 denotes empty slot
		int component = 0;					// 0 = x, 1 = y, 2 = z, 3 = w
		float scale = 1;					// map values will be scaled by this
		ts_vec2 uvscale = ts_vec2( 1 );		// uv coordinate scale
		ts_vec2 uvoffset = ts_vec2( 0 );	// uv coordinate offset
		ts_uint2 size = ts_uint2( 0 );		// texture dimensions
		bool Specified() { return value != 1e32f || textureID != -1; }
		float& operator()() { return value; }
	};
	enum
	{
		SMOOTH = 1,						// material uses normal interpolation
		FROM_MTL = 4,					// changes are persistent for these, not for others
		SINGLE_COLOR_COPY = 8			// material was created for a tri that uses a single texel
	};
	// constructor / destructor
	Material() = default;
	// methods
	void ConvertFrom( const ::tinyobj::material_t& );
	void ConvertFrom( const ::tinygltf::Material&, const ::std::vector<int>& texIdx );
	bool IsEmissive() { ts_vec3& c = color(); return c.x > 1 || c.y > 1 || c.z > 1; /* ignores vec3map */ }
	// material properties
	vec3Value color = vec3Value( 1 );	// universal material property: base color
	vec3Value detailColor;				// universal material property: detail texture
	vec3Value normals;					// universal material property: normal map
	vec3Value detailNormals;			// universal material property: detail normal map
	uint32_t flags = SMOOTH;			// material flags: default is SMOOTH
	// Disney BRDF properties: data for the Disney Principled BRDF
	vec3Value absorption;
	ScalarValue metallic, subsurface, specular, roughness, specularTint, anisotropic;
	ScalarValue sheen, sheenTint, clearcoat, clearcoatGloss, transmission, eta;
	// Lambert BSDF properties, augmented with pure specular reflection and refraction
	// FloatValue absorption;			// shared with disney brdf
	ScalarValue reflection, refraction, ior;
	// identifier and name
	::std::string name = "unnamed";		// material name, not for unique identification
	::std::string origin;				// origin: file from which the data was loaded, with full path
	int ID = -1;						// unique integer ID of this material
	uint32_t refCount = 1;				// the number of models that use this material
	// field for the BuildMaterialList method of Mesh
	bool visited = false;				// last mesh that checked this material
	// internal
private:
	uint32_t prevFlags = SMOOTH;		// initially identical to flags
};

//  +-----------------------------------------------------------------------------+
//  |  Material                                                                   |
//  |  Material definition. This is the version for actual rendering; it stores   |
//  |  a subset of the full data of a Material.                             LH2'25|
//  +-----------------------------------------------------------------------------+
class RenderMaterial
{
	enum
	{
		HASDIFFUSEMAP = 1,
		HASNORMALMAP = 2,
		ISDIELECTRIC = 4,
		HASSPECULARITYMAP = 8,
		HASROUGHNESSMAP = 16,
		HAS2NDNORMALMAP = 32,
		HAS2NDDIFFUSEMAP = 64,
		HASSMOOTHNORMALS = 128,
		HASALPHA = 256,
		DIFFUSEMAPISHDR = 512
	};
	struct Map { short width, height; uint16_t uscale, vscale, uoffs, voffs; uint32_t addr; };
public:
	RenderMaterial( const Material* source, const ::std::vector<int>& offsets );
	Map MakeMap( Material::vec3Value source, const ::std::vector<int>& offsets );
	void SetDiffuse( ts_vec3 d );
	void SetTransmittance( ts_vec3 t );
	uint16_t diffuse_r, diffuse_g, diffuse_b, transmittance_r, transmittance_g, transmittance_b;
	uint32_t flags;
	ts_uint4 parameters; // 16 Disney principled BRDF parameters, 0.8 fixed point
	Map tex0, tex1, nmap0, nmap1, smap, rmap; // total Material size: 128 bytes
};

//  +-----------------------------------------------------------------------------+
//  |  Animation                                                                  |
//  |  Animation definition.                                                LH2'25|
//  +-----------------------------------------------------------------------------+
class Animation
{
	class Sampler
	{
	public:
		enum { LINEAR = 0, SPLINE, STEP };
		Sampler( const ::tinygltf::AnimationSampler& gltfSampler, const ::tinygltf::Model& gltfModel );
		void ConvertFromGLTFSampler( const ::tinygltf::AnimationSampler& gltfSampler, const ::tinygltf::Model& gltfModel );
		float SampleFloat( float t, int k, int i, int count ) const;
		ts_vec3 SampleVec3( float t, int k ) const;
		ts_quat SampleQuat( float t, int k ) const;
		::std::vector<float> t;			// key frame times
		::std::vector<ts_vec3> vec3Key;	// vec3 key frames (location or scale)
		::std::vector<ts_quat> vec4Key;	// vec4 key frames (rotation)
		::std::vector<float> floatKey;	// float key frames (weight)
		int interpolation;				// interpolation type: linear, spline, step
	};
	class Channel
	{
	public:
		Channel( const ::tinygltf::AnimationChannel& gltfChannel, const int nodeBase );
		int samplerIdx;					// sampler used by this channel
		int nodeIdx;					// index of the node this channel affects
		int target;						// 0: translation, 1: rotation, 2: scale, 3: weights
		void Reset() { t = 0, k = 0; }
		void SetTime( const float v ) { t = v, k = 0; }
		void Update( const float t, const Sampler* sampler );	// apply this channel to the target nde for time t
		void ConvertFromGLTFChannel( const ::tinygltf::AnimationChannel& gltfChannel, const int nodeBase );
		// data
		float t = 0;					// animation timer
		int k = 0;						// current keyframe
	};
public:
	Animation( ::tinygltf::Animation& gltfAnim, ::tinygltf::Model& gltfModel, const int nodeBase );
	::std::vector<Sampler*> sampler;	// animation samplers
	::std::vector<Channel*> channel;	// animation channels
	void Reset();						// reset all channels
	void SetTime( const float t );
	void Update( const float dt );		// advance and apply all channels
	void ConvertFromGLTFAnim( ::tinygltf::Animation& gltfAnim, ::tinygltf::Model& gltfModel, const int nodeBase );
};

//  +-----------------------------------------------------------------------------+
//  |  Texture                                                                    |
//  |  Stores a texture, with either integer or floating point data.              |
//  |  Policy regarding texture reuse:                                            |
//  |  - The owner of the textures is the scene.                                  |
//  |  - Multiple materials may use a texture. A refCount keeps track of this.    |
//  |  - A file name does not uniquely identify a texture: the file may be        |
//  |    different between folders, and the texture may have been loaded with     |
//  |    'modFlags'. Instead, a texture is uniquely identified by its full file   |
//  |    name, including path, as well as the mods field.                   LH2'25|
//  +-----------------------------------------------------------------------------+
class Texture
{
public:
	enum
	{
		NORMALMAP = 2,					// this texture is a normal map
		LDR = 4,						// this texture stores integer pixels in Texture::idata
		HDR = 8							// this texture stores float pixels in Texture::fdata
	};
	enum { LINEARIZED = 1, FLIPPED = 2 };
	// constructor / destructor / conversion
	Texture() = default;
	Texture( const char* fileName, const uint32_t modFlags = 0 );
	// methods
	bool Equals( const ::std::string& o, const uint32_t m );
	void Load( const char* fileName, const uint32_t modFlags, bool normalMap = false );
	static void sRGBtoLinear( unsigned char* pixels, const uint32_t size, const uint32_t stride );
	void BumpToNormalMap( float heightScale );
	uint32_t* GetLDRPixels() { return (uint32_t*)idata; }
	ts_vec4* GetHDRPixels() { return fdata; }
	// internal methods
	int PixelsNeeded( int w, int h, const int l ) const;
	void ConstructMIPmaps();
	// public properties
public:
	uint32_t width = 0, height = 0;		// width and height in pixels
	uint32_t MIPlevels = 1;				// number of MIPmaps
	uint32_t ID = 0;					// unique integer ID of this texture
	::std::string name;					// texture name, not for unique identification
	::std::string origin;				// origin: file from which the data was loaded, with full path
	uint32_t flags = 0;					// flags
	uint32_t mods = 0;					// modifications to original data
	uint32_t refCount = 1;				// the number of materials that use this texture
	ts_uchar4* idata = nullptr;			// pointer to a 32-bit ARGB bitmap
	ts_vec4* fdata = nullptr;			// pointer to a 128-bit ARGB bitmap
};

//  +-----------------------------------------------------------------------------+
//  |  TriLight                                                                   |
//  |  Light triangle.                                                      LH2'25|
//  +-----------------------------------------------------------------------------+
class TriLight
{
public:
	// constructor / destructor
	TriLight() = default;
	TriLight( FatTri* origTri, int origIdx, int origInstance );
	// data members
	int triIdx = 0;						// the index of the triangle this ltri is based on
	int instIdx = 0;					// the instance to which this triangle belongs
	ts_vec3 vertex0 = { 0 };			// vertex 0 position
	ts_vec3 vertex1 = { 0 };			// vertex 1 position
	ts_vec3 vertex2 = { 0 };			// vertex 2 position
	ts_vec3 centre = { 0 };				// barycenter of the triangle
	ts_vec3 radiance = { 0 };			// radiance per unit area
	ts_vec3 N = ts_vec3( 0, -1, 0 );	// geometric triangle normal
	float area = 0;						// triangle area
	float energy = 0;					// total radiance
};

//  +-----------------------------------------------------------------------------+
//  |  PointLight                                                                 |
//  |  Point light definition.                                              LH2'25|
//  +-----------------------------------------------------------------------------+
class PointLight
{
public:
	// constructor / destructor
	PointLight() = default;
	// data members
	ts_vec3 position = { 0 };			// position of the point light
	ts_vec3 radiance = { 0 };			// emitted radiance
	int ID = 0;							// position in Scene::pointLights
};

//  +-----------------------------------------------------------------------------+
//  |  SpotLight                                                                  |
//  |  Spot light definition.                                               LH2'19|
//  +-----------------------------------------------------------------------------+
class SpotLight
{
public:
	// constructor / destructor
	SpotLight() = default;
	// data members
	ts_vec3 position = { 0 };			// position of the spot light
	float cosInner = 0;					// cosine of the inner boundary
	ts_vec3 radiance = { 0 };			// emitted radiance
	float cosOuter = 0;					// cosine of the outer boundary
	ts_vec3 direction = ts_vec3( 0, -1, 0 ); // spot light direction
	int ID = 0;							// position in Scene::spotLights
};

//  +-----------------------------------------------------------------------------+
//  |  DirectionalLight                                                           |
//  |  Directional light definition.                                        LH2'25|
//  +-----------------------------------------------------------------------------+
class DirectionalLight
{
public:
	// constructor / destructor
	DirectionalLight() = default;
	// data members
	ts_vec3 direction = ts_vec3( 0, -1, 0 );
	ts_vec3 radiance = { 0 };
	int ID = 0;
};

//  +-----------------------------------------------------------------------------+
//  |  Scene                                                                      |
//  |  Module for scene I/O and host-side management.                             |
//  |  This is a pure static class; we will not have more than one scene.   LH2'25|
//  +-----------------------------------------------------------------------------+
class Scene
{
public:
	// constructor / destructor
	Scene() = default;
	~Scene();
	// methods
	static void Init() { /* nothing here for now */ }
	static void SetSkyDome( SkyDome* skydome ) { sky = skydome; }
	static int FindOrCreateTexture( const ::std::string& origin, const uint32_t modFlags = 0 );
	static int FindTextureID( const char* name );
	static int CreateTexture( const ::std::string& origin, const uint32_t modFlags = 0 );
	static int FindOrCreateMaterial( const ::std::string& name );
	static int FindOrCreateMaterialCopy( const int matID, const uint32_t color );
	static int FindMaterialID( const char* name );
	static int FindMaterialIDByOrigin( const char* name );
	static int FindNextMaterialID( const char* name, const int matID );
	static int FindNode( const char* name );
	static int FindMeshNode( const int nodeId, const int meshId );
	static int CollapseMeshes( const int nodeId );
	static void CreateOpacityMicroMaps( const int nodeId );
	static void SetBVHType( const int nodeId, const int t );
	static void SetNodeTransform( const int nodeId, const ts_mat4& transform );
	static const ts_mat4& GetNodeTransform( const int nodeId );
	static void ResetAnimation( const int animId );
	static void ResetAnimations();
	static void UpdateAnimation( const int animId, const float dt );
	static int AnimationCount() { return (int)animations.size(); }
	// scene construction / maintenance
	static int AddMesh( Mesh* mesh );
	static int AddMesh( const char* objFile, const char* dir, const float scale = 1.0f, const bool flatShaded = false );
	static int AddMesh( const char* objFile, const float scale = 1.0f, const bool flatShaded = false );
	static int AddScene( const char* sceneFile, const ts_mat4& transform = ts_mat4() );
	static int AddScene( const char* sceneFile, const float scale );
	static int AddScene( const char* sceneFile, const char* dir, const ts_mat4& transform );
	static int AddMesh( const int triCount );
	static void AddTriToMesh( const int meshId, const ts_vec3& v0, const ts_vec3& v1, const ts_vec3& v2, const int matId );
	static int AddQuad( const ts_vec3 N, const ts_vec3 pos, const float width, const float height, const int matId, const int meshID = -1 );
	static int AddNode( Node* node );
	static int AddChildNode( const int parentNodeId, const int childNodeId );
	static int GetChildId( const int parentId, const int childIdx );
	static int AddInstance( const int nodeId );
	static void RemoveNode( const int instId );
	static int AddMaterial( Material* material );
	static int AddMaterial( const ts_vec3 color, const char* name = 0 );
	static int AddPointLight( const ts_vec3 pos, const ts_vec3 radiance );
	static int AddSpotLight( const ts_vec3 pos, const ts_vec3 direction, const float inner, const float outer, const ts_vec3 radiance );
	static int AddDirectionalLight( const ts_vec3 direction, const ts_vec3 radiance );
	static void UpdateSceneGraph( const float deltaTime );
	static void SetBVHDefault( const uint32_t t ) { defaultBVHType = t; }
	static void CacheBVHs( bool v = true ) { bvhCaching = v; }
	// data members
	static inline ::std::vector<int> rootNodes;				// root node indices of loaded (or instanced) objects
	static inline ::std::vector<Node*> nodePool;			// all scene nodes
	static inline ::std::vector<Mesh*> meshPool;			// all scene meshes
	static inline ::std::vector<tinybvh::BLASInstance> instPool; // all scene instances
	static inline ::std::vector<Skin*> skins;				// all scene skins
	static inline ::std::vector<Animation*> animations;		// all scene animations
	static inline ::std::vector<Material*> materials;		// all scene materials
	static inline ::std::vector<Texture*> textures;			// all scene textures
	static inline ::std::vector<TriLight*> triLights;		// light emitting triangles
	static inline ::std::vector<PointLight*> pointLights;	// scene point lights
	static inline ::std::vector<SpotLight*> spotLights;		// scene spot lights
	static inline ::std::vector<DirectionalLight*> directionalLights;	// scene directional lights
	static inline SkyDome* sky;								// HDR skydome
	static inline tinybvh::BVH* tlas = 0;					// top-level acceleration structure
	static inline tinybvh::BVH_GPU* gpuTlas = 0;			// top-level acceleration structure, gpu version
	static inline uint32_t defaultBVHType = BVH_DYNAMIC;	// BVH_RIGID is faster but more restrictive
	static inline bool bvhCaching = false;					// caching for acceleration structures
};

} // namespace tinyscene

#endif // TINY_SCENE_H_

// ============================================================================
//
//        I M P L E M E N T A T I O N
//
// ============================================================================

#ifdef TINYSCENE_IMPLEMENTATION

// error handling
#define SCENE_FATAL_ERROR(s) { SCENE_FATAL_ERROR_IF(1,(s)) }
#ifdef _WINDOWS_ // windows.h has been included
#define SCENE_FATAL_ERROR_IF(c,s) { if (c) { char t[512]; sprintf( t, \
	"Fatal error in tiny_scene.h, line %i:\n%s\n", __LINE__, s ); \
	MessageBox( NULL, t, "Fatal error", MB_OK ); exit( 1 ); } }
#else
#define SCENE_FATAL_ERROR_IF(c,s) if (c) { fprintf( stderr, \
	"Fatal error in tiny_scene.h, line %i:\n%s\n", __LINE__, s ); exit( 1 ); }
#endif

namespace tinyscene {

// basic vector math operations
#ifndef TINYSCENE_USE_CUSTOM_VECTOR_TYPES
ts_mat4 operator*( const float s, const ts_mat4& a )
{
	ts_mat4 r;
	for (uint32_t i = 0; i < 16; i++) r.cell[i] = a.cell[i] * s;
	return r;
}
ts_mat4 operator*( const ts_mat4& a, const ts_mat4& b )
{
	ts_mat4 r;
	for (uint32_t i = 0; i < 16; i += 4) for (uint32_t j = 0; j < 4; ++j)
		r[i + j] = (a[i + 0] * b[j + 0]) + (a[i + 1] * b[j + 4]) +
		(a[i + 2] * b[j + 8]) + (a[i + 3] * b[j + 12]);
	return r;
}
#endif
void ts_invert( ts_mat4& T )
{
	// from MESA, via http://stackoverflow.com/questions/1148309/inverting-a-4x4-matrix
	const float inv[16] = {
		T[5] * T[10] * T[15] - T[5] * T[11] * T[14] - T[9] * T[6] * T[15] + T[9] * T[7] * T[14] + T[13] * T[6] * T[11] - T[13] * T[7] * T[10],
		-T[1] * T[10] * T[15] + T[1] * T[11] * T[14] + T[9] * T[2] * T[15] - T[9] * T[3] * T[14] - T[13] * T[2] * T[11] + T[13] * T[3] * T[10],
		T[1] * T[6] * T[15] - T[1] * T[7] * T[14] - T[5] * T[2] * T[15] + T[5] * T[3] * T[14] + T[13] * T[2] * T[7] - T[13] * T[3] * T[6],
		-T[1] * T[6] * T[11] + T[1] * T[7] * T[10] + T[5] * T[2] * T[11] - T[5] * T[3] * T[10] - T[9] * T[2] * T[7] + T[9] * T[3] * T[6],
		-T[4] * T[10] * T[15] + T[4] * T[11] * T[14] + T[8] * T[6] * T[15] - T[8] * T[7] * T[14] - T[12] * T[6] * T[11] + T[12] * T[7] * T[10],
		T[0] * T[10] * T[15] - T[0] * T[11] * T[14] - T[8] * T[2] * T[15] + T[8] * T[3] * T[14] + T[12] * T[2] * T[11] - T[12] * T[3] * T[10],
		-T[0] * T[6] * T[15] + T[0] * T[7] * T[14] + T[4] * T[2] * T[15] - T[4] * T[3] * T[14] - T[12] * T[2] * T[7] + T[12] * T[3] * T[6],
		T[0] * T[6] * T[11] - T[0] * T[7] * T[10] - T[4] * T[2] * T[11] + T[4] * T[3] * T[10] + T[8] * T[2] * T[7] - T[8] * T[3] * T[6],
		T[4] * T[9] * T[15] - T[4] * T[11] * T[13] - T[8] * T[5] * T[15] + T[8] * T[7] * T[13] + T[12] * T[5] * T[11] - T[12] * T[7] * T[9],
		-T[0] * T[9] * T[15] + T[0] * T[11] * T[13] + T[8] * T[1] * T[15] - T[8] * T[3] * T[13] - T[12] * T[1] * T[11] + T[12] * T[3] * T[9],
		T[0] * T[5] * T[15] - T[0] * T[7] * T[13] - T[4] * T[1] * T[15] + T[4] * T[3] * T[13] + T[12] * T[1] * T[7] - T[12] * T[3] * T[5],
		-T[0] * T[5] * T[11] + T[0] * T[7] * T[9] + T[4] * T[1] * T[11] - T[4] * T[3] * T[9] - T[8] * T[1] * T[7] + T[8] * T[3] * T[5],
		-T[4] * T[9] * T[14] + T[4] * T[10] * T[13] + T[8] * T[5] * T[14] - T[8] * T[6] * T[13] - T[12] * T[5] * T[10] + T[12] * T[6] * T[9],
		T[0] * T[9] * T[14] - T[0] * T[10] * T[13] - T[8] * T[1] * T[14] + T[8] * T[2] * T[13] + T[12] * T[1] * T[10] - T[12] * T[2] * T[9],
		-T[0] * T[5] * T[14] + T[0] * T[6] * T[13] + T[4] * T[1] * T[14] - T[4] * T[2] * T[13] - T[12] * T[1] * T[6] + T[12] * T[2] * T[5],
		T[0] * T[5] * T[10] - T[0] * T[6] * T[9] - T[4] * T[1] * T[10] + T[4] * T[2] * T[9] + T[8] * T[1] * T[6] - T[8] * T[2] * T[5]
	};
	const float det = T[0] * inv[0] + T[1] * inv[4] + T[2] * inv[8] + T[3] * inv[12];
	if (det == 0) return;
	const float invdet = 1.0f / det;
	for (int i = 0; i < 16; i++) T[i] = inv[i] * invdet;
}
inline float ts_dot( const ts_vec2& a, const ts_vec2& b ) { return a.x * b.x + a.y * b.y; }
inline float ts_dot( const ts_vec3& a, const ts_vec3& b ) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline float ts_dot( const ts_vec4& a, const ts_vec4& b ) { return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w; }
inline ts_vec3 ts_cross( const ts_vec3& a, const ts_vec3& b ) { return ts_vec3( a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x ); }
inline float ts_length( const ts_vec3& a ) { return sqrtf( a.x * a.x + a.y * a.y + a.z * a.z ); }
inline ts_vec3 ts_normalize( const ts_vec3& a )
{
	float l = ts_length( a ), rl = l == 0 ? 0 : (1.0f / l);
	return a * rl;
}
inline ts_vec3 ts_transform_point( const ts_vec3& v, const ts_mat4& T )
{
	const ts_vec3 res(
		T[0] * v.x + T[1] * v.y + T[2] * v.z + T[3],
		T[4] * v.x + T[5] * v.y + T[6] * v.z + T[7],
		T[8] * v.x + T[9] * v.y + T[10] * v.z + T[11] );
	const float w = T[12] * v.x + T[13] * v.y + T[14] * v.z + T[15];
	if (w == 1) return res; else return res * (1.f / w);
}
inline ts_vec3 ts_transform_vector( const ts_vec3& v, const ts_mat4& T )
{
	return ts_vec3( T[0] * v.x + T[1] * v.y + T[2] * v.z, T[4] * v.x +
		T[5] * v.y + T[6] * v.z, T[8] * v.x + T[9] * v.y + T[10] * v.z );
}
inline float ts_min( const float a, const float b ) { return a < b ? a : b; }
inline float ts_max( const float a, const float b ) { return a > b ? a : b; }
inline ts_vec3 ts_min( const ts_vec3& a, const ts_vec3& b ) { return ts_vec3( ts_min( a.x, b.x ), ts_min( a.y, b.y ), ts_min( a.z, b.z ) ); }
inline ts_vec3 ts_max( const ts_vec3& a, const ts_vec3& b ) { return ts_vec3( ts_max( a.x, b.x ), ts_max( a.y, b.y ), ts_max( a.z, b.z ) ); }

// basic 16-bit float support
static uint32_t ts_as_uint( const float x ) { return *(uint32_t*)&x; }
float ts_as_float( const uint32_t x ) { return *(float*)&x; }
float ts_half_to_float( const uint16_t x ) {
	const uint32_t e = (x & 0x7C00) >> 10, m = (x & 0x03FF) << 13, v = ts_as_uint( (float)m ) >> 23;
	return ts_as_float( (x & 0x8000) << 16 | (e != 0) * ((e + 112) << 23 | m) | ((e == 0) & (m != 0)) *
		((v - 37) << 23 | ((m << (150 - v)) & 0x007FE000)) );
}
uint16_t ts_float_to_half( const float x )
{
	const uint32_t b = ts_as_uint( x ) + 0x00001000, e = (b & 0x7F800000) >> 23, m = b & 0x007FFFFF;
	return (uint16_t)((b & 0x80000000) >> 16 | (e > 112) * ((((e - 112) << 10) & 0x7C00) | m >> 13) |
		((e < 113) & (e > 101)) * ((((0x007FF000 + m) >> (125 - e)) + 1) >> 1) | (e > 143) * 0x7FFF);
}

void ts_aabb::grow( const ts_vec3& p )
{
	bmin = ts_min( bmin, p );
	bmax = ts_max( bmin, p );
}

ts_quat ts_quat::slerp( const ts_quat& a, const ts_quat& b, const float t )
{
	// from GLM, via blog.magnum.graphics/backstage/the-unnecessarily-short-ways-to-do-a-quaternion-slerp
	ts_quat r = b;
	float cosTheta = a.w * r.w + a.x * r.x + a.y * r.y + a.z * r.z;
	if (cosTheta < 0) r = r * -1.0f, cosTheta = -cosTheta;
	if (cosTheta > 0.99f)
	{
		// Linear interpolation
		r.w = (1 - t) * a.w + t * r.w, r.x = (1 - t) * a.x + t * r.x;
		r.y = (1 - t) * a.y + t * r.y, r.z = (1 - t) * a.z + t * r.z;
	}
	else
	{
		float angle = acosf( cosTheta );
		float s1 = sinf( (1 - t) * angle ), s2 = sinf( t * angle ), rs3 = 1.0f / sinf( angle );
		r.w = (s1 * a.w + s2 * r.w) * rs3, r.x = (s1 * a.x + s2 * r.x) * rs3;
		r.y = (s1 * a.y + s2 * r.y) * rs3, r.z = (s1 * a.z + s2 * r.z) * rs3;
	}
	return r;
}

void FatTri::UpdateArea()
{
	const float a = ts_length( vertex1 - vertex0 ), b = ts_length( vertex2 - vertex1 );
	const float c = ts_length( vertex0 - vertex2 ), s = (a + b + c) * 0.5f;
	area = sqrtf( s * (s - a) * (s - b) * (s - c) ); // Heron's formula
}

void RenderMaterial::SetDiffuse( ts_vec3 d )
{
	diffuse_r = ts_float_to_half( d.x );
	diffuse_g = ts_float_to_half( d.y );
	diffuse_b = ts_float_to_half( d.z );
}

void RenderMaterial::SetTransmittance( ts_vec3 t )
{
	transmittance_r = ts_float_to_half( t.x );
	transmittance_g = ts_float_to_half( t.y );
	transmittance_b = ts_float_to_half( t.z );
}

#ifdef _WIN32
#include <direct.h>
#define getcwd _getcwd // stupid MSFT "deprecation" warning
#define chdir _chdir
#elif
#include <unistd.h>
#endif
#include <thread>

// 'declaration of x hides previous local declaration'
#pragma warning( disable: 4456)

// bring std
using namespace ::std;

//  +-----------------------------------------------------------------------------+
//  |  SkyDome::Load                                                              |
//  |  Load a skydome.                                                      LH2'25|
//  +-----------------------------------------------------------------------------+
void SkyDome::Load( const char* filename, const ts_vec3 scale )
{
	tinyscene::free64( pixels ); // just in case we're reloading
	pixels = 0;
	// Append ".bin" to the filename:
	char bin_name[1024];
	strncpy( bin_name, filename, sizeof( bin_name ) );
	strncat( bin_name, ".bin", sizeof( bin_name ) - strlen( bin_name ) - 1 );
	// attempt to load skydome from binary file
	FILE* f = fopen( bin_name, "rb" );
	if (f)
	{
		printf( "loading cached hdr data... " );
		fread( &width, 1, 4, f );
		fread( &height, 1, 4, f );
		pixels = (ts_vec3*)malloc64( width * height * sizeof( ts_vec3 ) );
		fread( pixels, 1, sizeof( ts_vec3 ) * width * height, f );
		fclose( f );
		printf( "done.\n" );
	}
	if (!pixels)
	{
		// load HDR sky
		int bpp = 0;
		float* tmp = stbi_loadf( filename, &width, &height, &bpp, 0 );
		if (!tmp) SCENE_FATAL_ERROR( "File does not exist" );
		if (bpp == 3)
		{
			pixels = (ts_vec3*)malloc64( width * height * sizeof( ts_vec3 ) );
			for (int i = 0; i < width * height; i++)
				pixels[i] = ts_vec3( sqrtf( tmp[i * 3 + 0] ), sqrtf( tmp[i * 3 + 1] ), sqrtf( tmp[i * 3 + 2] ) );
		}
		else SCENE_FATAL_ERROR( "Unsupported skydome image channel count." );
		// save skydome to binary file, .hdr is slow to load
		f = fopen( bin_name, "wb" );
		fwrite( &width, 1, 4, f );
		fwrite( &height, 1, 4, f );
		fwrite( pixels, 1, sizeof( ts_vec3 ) * width * height, f );
		fclose( f );
	}
	// cache does not include scale so we can change it later
	for (int p = 0; p < width * height; ++p) pixels[p] = pixels[p] * pixels[p] * scale;
}

//  +-----------------------------------------------------------------------------+
//  |  Skin::Skin                                                                 |
//  |  Constructor.                                                         LH2'25|
//  +-----------------------------------------------------------------------------+
Skin::Skin( const ::tinygltf::Skin& gltfSkin, const ::tinygltf::Model& gltfModel, const int nodeBase )
{
	ConvertFromGLTFSkin( gltfSkin, gltfModel, nodeBase );
}

//  +-----------------------------------------------------------------------------+
//  |  Skin::Skin                                                                 |
//  |  Constructor.                                                         LH2'25|
//  +-----------------------------------------------------------------------------+
void Skin::ConvertFromGLTFSkin( const ::tinygltf::Skin& gltfSkin, const ::tinygltf::Model& gltfModel, const int nodeBase )
{
	name = gltfSkin.name;
	skeletonRoot = (gltfSkin.skeleton == -1 ? 0 : gltfSkin.skeleton) + nodeBase;
	for (int jointIndex : gltfSkin.joints) joints.push_back( jointIndex + nodeBase );
	if (gltfSkin.inverseBindMatrices > -1)
	{
		const auto& accessor = gltfModel.accessors[gltfSkin.inverseBindMatrices];
		const auto& bufferView = gltfModel.bufferViews[accessor.bufferView];
		const auto& buffer = gltfModel.buffers[bufferView.buffer];
		inverseBindMatrices.resize( accessor.count );
		::memcpy( inverseBindMatrices.data(), &buffer.data[accessor.byteOffset + bufferView.byteOffset], accessor.count * sizeof( ts_mat4 ) );
		jointMat.resize( accessor.count );
		// convert gltf's column-major to row-major
		for (int k = 0; k < accessor.count; k++)
		{
			ts_mat4 M = inverseBindMatrices[k];
			for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) inverseBindMatrices[k].cell[j * 4 + i] = M.cell[i * 4 + j];
		}
	}
}

//  +-----------------------------------------------------------------------------+
//  |  Mesh::Mesh                                                                 |
//  |  Constructors.                                                        LH2'25|
//  +-----------------------------------------------------------------------------+
Mesh::Mesh( const int triCount )
{
	triangles.resize( triCount ); // precallocate; to be used for procedural meshes.
	vertices.resize( triCount * 3 );
	blas.bvhType = Scene::defaultBVHType;
}

Mesh::Mesh( const char* file, const char* dir, const float scale, const bool flatShaded )
{
	LoadGeometry( file, dir, scale, flatShaded );
	blas.bvhType = Scene::defaultBVHType;
}

Mesh::Mesh( const ::tinygltf::Mesh& gltfMesh, const ::tinygltf::Model& gltfModel, const vector<int>& matIdx, const int materialOverride )
{
	ConvertFromGTLFMesh( gltfMesh, gltfModel, matIdx, materialOverride );
	blas.bvhType = Scene::defaultBVHType;
}

//  +-----------------------------------------------------------------------------+
//  |  Mesh::LoadGeometry                                                         |
//  |  Load geometry data from disk. Obj files only.                        LH2'25|
//  +-----------------------------------------------------------------------------+
void Mesh::LoadGeometry( const char* file, const char* dir, const float scale, const bool flatShaded )
{
	// process supplied file name
	ts_mat4 T; // assume these are initialized to identity.
	T[0] = T[5] = T[10] = scale, T[15] = 1; // scaling matrix
	string combined = string( dir ) + (dir[strlen( dir ) - 1] == '/' ? "" : "/") + string( file );
	for (int l = (int)combined.size(), i = 0; i < l; i++) if (combined[i] >= 'A' && combined[i] <= 'Z') combined[i] -= 'Z' - 'z';
	string extension = (combined.find_last_of( "." ) != string::npos) ? combined.substr( combined.find_last_of( "." ) + 1 ) : "";
	if (extension.compare( "obj" ) != 0) SCENE_FATAL_ERROR( "Unsupported extension in file" );
	LoadGeometryFromOBJ( combined.c_str(), dir, T, flatShaded );
}

//  +-----------------------------------------------------------------------------+
//  |  Mesh::LoadGeometryFromObj                                                  |
//  |  Load an obj file using tinyobj.                                      LH2'25|
//  +-----------------------------------------------------------------------------+
void Mesh::LoadGeometryFromOBJ( const string& file, const char* directory, const ts_mat4& T, const bool flatShaded )
{
	// load obj file
	::tinyobj::attrib_t attrib;
	vector<::tinyobj::shape_t> shapes;
	vector<::tinyobj::material_t> materials;
	map<string, uint32_t> textures;
	string err, warn;
	::tinyobj::LoadObj( &attrib, &shapes, &materials, &err, &warn, file.c_str(), directory );
	SCENE_FATAL_ERROR_IF( err.size() > 0 || shapes.size() == 0, "tinyobj failed to load obj" /* , file.c_str(), err.c_str() */ );
	// material offset: if we loaded an object before this one, material indices should not start at 0.
	int matIdxOffset = (int)Scene::materials.size();
	// process materials
	char currDir[1024];
	getcwd( currDir, 1024 ); // GetCurrentDirectory( 1024, currDir );
	chdir( directory ); // SetCurrentDirectory( directory );
	materialList.clear();
	materialList.reserve( materials.size() );
	for (auto& mtl : materials)
	{
		// initialize
		Material* material = new Material();
		material->ID = (int)Scene::materials.size();
		material->origin = file;
		material->ConvertFrom( mtl );
		material->flags |= Material::FROM_MTL;
		Scene::materials.push_back( material );
		materialList.push_back( material->ID );
	}
	chdir( currDir );
	// calculate values for consistent normal interpolation
	const uint32_t verts = (uint32_t)attrib.normals.size() / 3;
	vector<float> alphas;
	alphas.resize( verts, 1.0f ); // we will have one alpha value per unique vertex normal
	for (uint32_t s = (uint32_t)shapes.size(), i = 0; i < s; i++)
	{
		vector<::tinyobj::index_t>& indices = shapes[i].mesh.indices;
		if (flatShaded) for (uint32_t s = (uint32_t)indices.size(), f = 0; f < s; f++) alphas[indices[f].normal_index] = 1; else
		{
			for (uint32_t s = (uint32_t)indices.size(), f = 0; f < s; f += 3)
			{
				const int idx0 = indices[f + 0].vertex_index, nidx0 = indices[f + 0].normal_index;
				const int idx1 = indices[f + 1].vertex_index, nidx1 = indices[f + 1].normal_index;
				const int idx2 = indices[f + 2].vertex_index, nidx2 = indices[f + 2].normal_index;
				const ts_vec3 vert0 = ts_vec3( attrib.vertices[idx0 * 3 + 0], attrib.vertices[idx0 * 3 + 1], attrib.vertices[idx0 * 3 + 2] );
				const ts_vec3 vert1 = ts_vec3( attrib.vertices[idx1 * 3 + 0], attrib.vertices[idx1 * 3 + 1], attrib.vertices[idx1 * 3 + 2] );
				const ts_vec3 vert2 = ts_vec3( attrib.vertices[idx2 * 3 + 0], attrib.vertices[idx2 * 3 + 1], attrib.vertices[idx2 * 3 + 2] );
				ts_vec3 N = ts_normalize( ts_cross( vert1 - vert0, vert2 - vert0 ) );
				ts_vec3 vN0, vN1, vN2;
				if (nidx0 > -1)
				{
					vN0 = ts_vec3( attrib.normals[nidx0 * 3 + 0], attrib.normals[nidx0 * 3 + 1], attrib.normals[nidx0 * 3 + 2] );
					vN1 = ts_vec3( attrib.normals[nidx1 * 3 + 0], attrib.normals[nidx1 * 3 + 1], attrib.normals[nidx1 * 3 + 2] );
					vN2 = ts_vec3( attrib.normals[nidx2 * 3 + 0], attrib.normals[nidx2 * 3 + 1], attrib.normals[nidx2 * 3 + 2] );
					if (ts_dot( N, vN0 ) < 0 && ts_dot( N, vN1 ) < 0 && ts_dot( N, vN2 ) < 0) N *= -1.0f; // flip if not consistent with vertex normals
					alphas[nidx0] = ts_min( alphas[nidx0], ts_max( 0.7f, ts_dot( vN0, N ) ) );
					alphas[nidx1] = ts_min( alphas[nidx1], ts_max( 0.7f, ts_dot( vN1, N ) ) );
					alphas[nidx2] = ts_min( alphas[nidx2], ts_max( 0.7f, ts_dot( vN2, N ) ) );
				}
				else vN0 = vN1 = vN2 = N;
			}
		}
	}
	// finalize alpha values based on max dots
	const float w = 0.03632f;
	for (uint32_t i = 0; i < verts; i++)
	{
		const float nnv = alphas[i]; // temporarily stored there
		alphas[i] = acosf( nnv ) * (1 + w * (1 - nnv) * (1 - nnv));
	}
	// extract data for ray tracing: raw vertex and index data
	ts_aabb sceneBounds;
	int toReserve = 0;
	for (auto& shape : shapes) toReserve += (int)shape.mesh.indices.size();
	vertices.reserve( toReserve );
	for (auto& shape : shapes) for (int f = 0; f < shape.mesh.indices.size(); f += 3)
	{
		const uint32_t idx0 = shape.mesh.indices[f + 0].vertex_index;
		const uint32_t idx1 = shape.mesh.indices[f + 1].vertex_index;
		const uint32_t idx2 = shape.mesh.indices[f + 2].vertex_index;
		const ts_vec3 v0 = ts_vec3( attrib.vertices[idx0 * 3 + 0], attrib.vertices[idx0 * 3 + 1], attrib.vertices[idx0 * 3 + 2] );
		const ts_vec3 v1 = ts_vec3( attrib.vertices[idx1 * 3 + 0], attrib.vertices[idx1 * 3 + 1], attrib.vertices[idx1 * 3 + 2] );
		const ts_vec3 v2 = ts_vec3( attrib.vertices[idx2 * 3 + 0], attrib.vertices[idx2 * 3 + 1], attrib.vertices[idx2 * 3 + 2] );
		const ts_vec4 tv0 = ts_transform_point( v0, T );
		const ts_vec4 tv1 = ts_transform_point( v1, T );
		const ts_vec4 tv2 = ts_transform_point( v2, T );
		vertices.push_back( tv0 );
		vertices.push_back( tv1 );
		vertices.push_back( tv2 );
		sceneBounds.grow( ts_vec3( tv0 ) );
		sceneBounds.grow( ts_vec3( tv1 ) );
		sceneBounds.grow( ts_vec3( tv2 ) );
	}
	// extract full model data and materials
	triangles.resize( vertices.size() / 3 );
	for (int s = (int)shapes.size(), face = 0, i = 0; i < s; i++)
	{
		vector<::tinyobj::index_t>& indices = shapes[i].mesh.indices;
		for (int s = (int)shapes[i].mesh.indices.size(), f = 0; f < s; f += 3, face++)
		{
			FatTri& tri = triangles[face];
			tri.vertex0 = ts_vec3( vertices[face * 3 + 0] );
			tri.vertex1 = ts_vec3( vertices[face * 3 + 1] );
			tri.vertex2 = ts_vec3( vertices[face * 3 + 2] );
			const int tidx0 = indices[f + 0].texcoord_index, nidx0 = indices[f + 0].normal_index; // , idx0 = indices[f + 0].vertex_index;
			const int tidx1 = indices[f + 1].texcoord_index, nidx1 = indices[f + 1].normal_index; // , idx1 = indices[f + 1].vertex_index;
			const int tidx2 = indices[f + 2].texcoord_index, nidx2 = indices[f + 2].normal_index; // , idx2 = indices[f + 2].vertex_index;
			const ts_vec3 e1 = tri.vertex1 - tri.vertex0;
			const ts_vec3 e2 = tri.vertex2 - tri.vertex0;
			ts_vec3 N = ts_normalize( ts_cross( e1, e2 ) );
			if (nidx0 > -1)
			{
				tri.vN0 = ts_vec3( attrib.normals[nidx0 * 3 + 0], attrib.normals[nidx0 * 3 + 1], attrib.normals[nidx0 * 3 + 2] );
				tri.vN1 = ts_vec3( attrib.normals[nidx1 * 3 + 0], attrib.normals[nidx1 * 3 + 1], attrib.normals[nidx1 * 3 + 2] );
				tri.vN2 = ts_vec3( attrib.normals[nidx2 * 3 + 0], attrib.normals[nidx2 * 3 + 1], attrib.normals[nidx2 * 3 + 2] );
				if (ts_dot( N, tri.vN0 ) < 0) N *= -1.0f; // flip face normal if not consistent with vertex normal
			}
			else tri.vN0 = tri.vN1 = tri.vN2 = N;
			if (flatShaded) tri.vN0 = tri.vN1 = tri.vN2 = N;
			if (tidx0 > -1)
			{
				tri.u0 = attrib.texcoords[tidx0 * 2 + 0], tri.v0 = attrib.texcoords[tidx0 * 2 + 1];
				tri.u1 = attrib.texcoords[tidx1 * 2 + 0], tri.v1 = attrib.texcoords[tidx1 * 2 + 1];
				tri.u2 = attrib.texcoords[tidx2 * 2 + 0], tri.v2 = attrib.texcoords[tidx2 * 2 + 1];
				// calculate tangent vectors
				ts_vec2 uv01 = ts_vec2( tri.u1 - tri.u0, tri.v1 - tri.v0 );
				ts_vec2 uv02 = ts_vec2( tri.u2 - tri.u0, tri.v2 - tri.v0 );
				if (ts_dot( uv01, uv01 ) == 0 || ts_dot( uv02, uv02 ) == 0)
					tri.T = ts_normalize( tri.vertex1 - tri.vertex0 ),
					tri.B = ts_normalize( ts_cross( N, tri.T ) );
				else
					tri.T = ts_normalize( e1 * uv02.y - e2 * uv01.y ),
					tri.B = ts_normalize( e2 * uv01.x - e1 * uv02.x );
			}
			else
				tri.T = ts_normalize( e1 ),
				tri.B = ts_normalize( ts_cross( N, tri.T ) );
			tri.Nx = N.x, tri.Ny = N.y, tri.Nz = N.z;
			tri.material = shapes[i].mesh.material_ids[f / 3] + matIdxOffset;
			tri.area = 0; // we don't actually use it, except for lights, where it is also calculated
			tri.invArea = 0; // todo
			if (nidx0 > -1)
				tri.alpha = ts_vec3( alphas[nidx0], tri.alpha.y = alphas[nidx1], tri.alpha.z = alphas[nidx2] );
			else
				tri.alpha = ts_vec3( 0 );
			// calculate triangle LOD data
			if (tri.material < Scene::materials.size())
			{
				Material* mat = Scene::materials[tri.material];
				int textureID = mat->color.textureID;
				if (textureID > -1)
				{
					Texture* texture = Scene::textures[textureID];
					float Ta = (float)(texture->width * texture->height) * fabs( (tri.u1 - tri.u0) * (tri.v2 - tri.v0) - (tri.u2 - tri.u0) * (tri.v1 - tri.v0) );
					float Pa = ts_length( ts_cross( tri.vertex1 - tri.vertex0, tri.vertex2 - tri.vertex0 ) );
					tri.LOD = 0.5f * log2f( Ta / Pa );
				}
			}
		}
	}
}

//  +-----------------------------------------------------------------------------+
//  |  Mesh::ConvertFromGTLFMesh                                                  |
//  |  Convert a gltf mesh to a Mesh.                                       LH2'25|
//  +-----------------------------------------------------------------------------+
void Mesh::ConvertFromGTLFMesh( const ::tinygltf::Mesh& gltfMesh, const ::tinygltf::Model& gltfModel, const vector<int>& matIdx, const int materialOverride )
{
	const int targetCount = (int)gltfMesh.weights.size();
	for (auto& prim : gltfMesh.primitives)
	{
		// load indices
		const ::tinygltf::Accessor& accessor = gltfModel.accessors[prim.indices];
		const ::tinygltf::BufferView& view = gltfModel.bufferViews[accessor.bufferView];
		const ::tinygltf::Buffer& buffer = gltfModel.buffers[view.buffer];
		const uint8_t* a /* brevity */ = buffer.data.data() + view.byteOffset + accessor.byteOffset;
		const int byteStride = accessor.ByteStride( view );
		const size_t count = accessor.count;
		// allocate the index array in the pointer-to-base declared in the parent scope
		vector<int> tmpIndices;
		vector<ts_vec3> tmpNormals, tmpVertices;
		vector<ts_vec2> tmpUvs, tmpUv2s /* texture layer 2 */;
		vector<ts_uint4> tmpJoints;
		vector<ts_vec4> tmpWeights, tmpTs;
		switch (accessor.componentType)
		{
		case TINYGLTF_COMPONENT_TYPE_BYTE: for (int k = 0; k < count; k++, a += byteStride) tmpIndices.push_back( *((char*)a) ); break;
		case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE: for (int k = 0; k < count; k++, a += byteStride) tmpIndices.push_back( *((uint8_t*)a) ); break;
		case TINYGLTF_COMPONENT_TYPE_SHORT: for (int k = 0; k < count; k++, a += byteStride) tmpIndices.push_back( *((short*)a) ); break;
		case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT: for (int k = 0; k < count; k++, a += byteStride) tmpIndices.push_back( *((uint16_t*)a) ); break;
		case TINYGLTF_COMPONENT_TYPE_INT: for (int k = 0; k < count; k++, a += byteStride) tmpIndices.push_back( *((int*)a) ); break;
		case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT: for (int k = 0; k < count; k++, a += byteStride) tmpIndices.push_back( *((uint32_t*)a) ); break;
		default: break;
		}
		// turn into faces - re-arrange the indices so that it describes a simple list of triangles
		if (prim.mode == TINYGLTF_MODE_TRIANGLE_FAN)
		{
			vector<int> fan = move( tmpIndices );
			tmpIndices.clear();
			for (size_t s = fan.size(), i = 2; i < s; i++)
			{
				tmpIndices.push_back( fan[0] );
				tmpIndices.push_back( fan[i - 1] );
				tmpIndices.push_back( fan[i] );
			}
		}
		else if (prim.mode == TINYGLTF_MODE_TRIANGLE_STRIP)
		{
			vector<int> strip = move( tmpIndices );
			tmpIndices.clear();
			for (size_t s = strip.size(), i = 2; i < s; i++)
			{
				tmpIndices.push_back( strip[i - 2] );
				tmpIndices.push_back( strip[i - 1] );
				tmpIndices.push_back( strip[i] );
			}
		}
		else if (prim.mode != TINYGLTF_MODE_TRIANGLES) /* skipping non-triangle primitive. */ continue;
		// we now have a simple list of vertex indices, 3 per triangle (TINYGLTF_MODE_TRIANGLES)
		for (const auto& attribute : prim.attributes)
		{
			const ::tinygltf::Accessor attribAccessor = gltfModel.accessors[attribute.second];
			const ::tinygltf::BufferView& bufferView = gltfModel.bufferViews[attribAccessor.bufferView];
			const ::tinygltf::Buffer& buffer = gltfModel.buffers[bufferView.buffer];
			const uint8_t* a = buffer.data.data() + bufferView.byteOffset + attribAccessor.byteOffset;
			const int byte_stride = attribAccessor.ByteStride( bufferView );
			const size_t count = attribAccessor.count;
			if (attribute.first == "POSITION")
			{
				ts_vec3 boundsMin = ts_vec3( (float)attribAccessor.minValues[0], (float)attribAccessor.minValues[1], (float)attribAccessor.minValues[2] );
				ts_vec3 boundsMax = ts_vec3( (float)attribAccessor.maxValues[0], (float)attribAccessor.maxValues[1], (float)attribAccessor.maxValues[2] );
				if (attribAccessor.type == TINYGLTF_TYPE_VEC3)
					if (attribAccessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT)
						for (size_t i = 0; i < count; i++, a += byte_stride) tmpVertices.push_back( *((ts_vec3*)a) );
					else { SCENE_FATAL_ERROR( "double precision positions not supported in gltf file" ); }
				else SCENE_FATAL_ERROR( "unsupported position definition in gltf file" );
			}
			else if (attribute.first == "NORMAL")
			{
				if (attribAccessor.type == TINYGLTF_TYPE_VEC3)
					if (attribAccessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT)
						for (size_t i = 0; i < count; i++, a += byte_stride) tmpNormals.push_back( *((ts_vec3*)a) );
					else { SCENE_FATAL_ERROR( "double precision normals not supported in gltf file" ); }
				else SCENE_FATAL_ERROR( "expected vec3 normals in gltf file" );
			}
			else if (attribute.first == "TANGENT")
			{
				/* if (attribAccessor.type == TINYGLTF_TYPE_VEC4)
					if (attribAccessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT)
						for (size_t i = 0; i < count; i++, a += byte_stride) tmpTs.push_back( *((ts_vec4*)a) );
					else FATALERROR( "double precision tangents not supported in gltf file" );
				else FATALERROR( "expected vec4 uvs in gltf file" ); */ // TODO: Causing crashes atm...
			}
			else if (attribute.first == "TEXCOORD_0")
			{
				if (attribAccessor.type == TINYGLTF_TYPE_VEC2)
					if (attribAccessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT)
						for (size_t i = 0; i < count; i++, a += byte_stride) tmpUvs.push_back( *((ts_vec2*)a) );
					else { SCENE_FATAL_ERROR( "double precision uvs not supported in gltf file" ); }
				else SCENE_FATAL_ERROR( "expected vec2 uvs in gltf file" );
			}
			else if (attribute.first == "TEXCOORD_1")
			{
				if (attribAccessor.type == TINYGLTF_TYPE_VEC2)
					if (attribAccessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT)
						for (size_t i = 0; i < count; i++, a += byte_stride) tmpUv2s.push_back( *((ts_vec2*)a) );
					else { SCENE_FATAL_ERROR( "double precision uvs not supported in gltf file" ); }
				else SCENE_FATAL_ERROR( "expected vec2 uvs in gltf file" );
			}
			else if (attribute.first == "COLOR_0")
			{
				// TODO; ignored for now.
			}
			else if (attribute.first == "JOINTS_0")
			{
				if (attribAccessor.type == TINYGLTF_TYPE_VEC4)
					if (attribAccessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT)
						for (size_t i = 0; i < count; i++, a += byte_stride)
							tmpJoints.push_back( ts_uint4( *((uint16_t*)a), *((uint16_t*)(a + 2)), *((uint16_t*)(a + 4)), *((uint16_t*)(a + 6)) ) );
					else if (attribAccessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE)
						for (size_t i = 0; i < count; i++, a += byte_stride)
							tmpJoints.push_back( ts_uint4( *((uint8_t*)a), *((uint8_t*)(a + 1)), *((uint8_t*)(a + 2)), *((uint8_t*)(a + 3)) ) );
					else { SCENE_FATAL_ERROR( "expected ushorts or uchars for joints in gltf file" ); }
				else SCENE_FATAL_ERROR( "expected vec4s for joints in gltf file" );
			}
			else if (attribute.first == "WEIGHTS_0")
			{
				if (attribAccessor.type == TINYGLTF_TYPE_VEC4)
					if (attribAccessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT)
						for (size_t i = 0; i < count; i++, a += byte_stride)
						{
							ts_vec4 w4;
							::memcpy( &w4, a, sizeof( ts_vec4 ) );
							float norm = 1.0f / (w4.x + w4.y + w4.z + w4.w);
							w4 = w4 * norm;
							tmpWeights.push_back( w4 );
						}
					else { SCENE_FATAL_ERROR( "double precision uvs not supported in gltf file" ); }
				else SCENE_FATAL_ERROR( "expected vec4 weights in gltf file" );
			}
			else if (attribute.first == "TEXCOORD_2") { /* TODO */ }
			else if (attribute.first == "TEXCOORD_3") { /* TODO */ }
			else if (attribute.first == "TEXCOORD_4") { /* TODO */ }
			else { assert( false ); /* unkown property */ }
		}
		// obtain morph targets
		vector<Pose> tmpPoses;
		if (targetCount > 0)
		{
			// store base pose
			tmpPoses.push_back( Pose() );
			for (int s = (int)tmpVertices.size(), i = 0; i < s; i++)
			{
				tmpPoses[0].positions.push_back( tmpVertices[i] );
				tmpPoses[0].normals.push_back( tmpNormals[i] );
				tmpPoses[0].tangents.push_back( ts_vec3( 0 ) /* TODO */ );
			}
		}
		for (int i = 0; i < targetCount; i++)
		{
			tmpPoses.push_back( Pose() );
			for (const auto& target : prim.targets[i])
			{
				const ::tinygltf::Accessor accessor = gltfModel.accessors[target.second];
				const ::tinygltf::BufferView& view = gltfModel.bufferViews[accessor.bufferView];
				const float* a = (const float*)(gltfModel.buffers[view.buffer].data.data() + view.byteOffset + accessor.byteOffset);
				for (int j = 0; j < accessor.count; j++)
				{
					ts_vec3 v = ts_vec3( a[j * 3], a[j * 3 + 1], a[j * 3 + 2] );
					if (target.first == "POSITION") tmpPoses[i + 1].positions.push_back( v );
					if (target.first == "NORMAL") tmpPoses[i + 1].normals.push_back( v );
					if (target.first == "TANGENT") tmpPoses[i + 1].tangents.push_back( v );
				}
			}
		}
		// all data has been read; add triangles to the Mesh
		BuildFromIndexedData( tmpIndices, tmpVertices, tmpNormals, tmpUvs, tmpUv2s, tmpTs, tmpPoses,
			tmpJoints, tmpWeights, materialOverride == -1 ? matIdx[prim.material] : materialOverride );
	}
}

//  +-----------------------------------------------------------------------------+
//  |  Mesh::BuildFromIndexedData                                                 |
//  |  We use non-indexed triangles, so three subsequent vertices form a tri,     |
//  |  to skip one indirection during intersection. glTF and obj store indexed    |
//  |  data, which we now convert to the final representation.              LH2'25|
//  +-----------------------------------------------------------------------------+
void Mesh::BuildFromIndexedData( const vector<int>& tmpIndices, const vector<ts_vec3>& tmpVertices,
	const vector<ts_vec3>& tmpNormals, const vector<ts_vec2>& tmpUvs, const vector<ts_vec2>& tmpUv2s,
	const vector<ts_vec4>& /* tmpTs */, const vector<Pose>& tmpPoses,
	const vector<ts_uint4>& tmpJoints, const vector<ts_vec4>& tmpWeights, const int materialIdx )
{
	// calculate values for consistent normal interpolation
	vector<float> tmpAlphas;
	tmpAlphas.resize( tmpVertices.size(), 1.0f ); // we will have one alpha value per unique vertex
	for (size_t s = tmpIndices.size(), i = 0; i < s; i += 3)
	{
		const uint32_t v0idx = tmpIndices[i + 0], v1idx = tmpIndices[i + 1], v2idx = tmpIndices[i + 2];
		const ts_vec3 vert0 = tmpVertices[v0idx], vert1 = tmpVertices[v1idx], vert2 = tmpVertices[v2idx];
		ts_vec3 N = ts_normalize( ts_cross( vert1 - vert0, vert2 - vert0 ) );
		ts_vec3 vN0, vN1, vN2;
		if (tmpNormals.size() > 0)
		{
			vN0 = tmpNormals[v0idx], vN1 = tmpNormals[v1idx], vN2 = tmpNormals[v2idx];
			if (ts_dot( N, vN0 ) < 0 && ts_dot( N, vN1 ) < 0 && ts_dot( N, vN2 ) < 0) N *= -1.0f; // flip if not consistent with vertex normals
		}
		else
		{
			// no normals supplied; copy face normal
			vN0 = vN1 = vN2 = N;
		}
		// Note: we clamp at approx. 45 degree angles; beyond this the approach fails.
		tmpAlphas[v0idx] = ts_min( tmpAlphas[v0idx], ts_dot( vN0, N ) );
		tmpAlphas[v1idx] = ts_min( tmpAlphas[v1idx], ts_dot( vN1, N ) );
		tmpAlphas[v2idx] = ts_min( tmpAlphas[v2idx], ts_dot( vN2, N ) );
	}
	for (size_t s = tmpAlphas.size(), i = 0; i < s; i++)
	{
		const float nnv = tmpAlphas[i]; // temporarily stored there
		tmpAlphas[i] = acosf( nnv ) * (1 + 0.03632f * (1 - nnv) * (1 - nnv));
	}
	// prepare poses
	for (int i = 0; i < (int)tmpPoses.size(); i++) poses.push_back( Pose() );
	// build final mesh structures
	const size_t newTriangleCount = tmpIndices.size() / 3;
	size_t triIdx = triangles.size();
	triangles.resize( triIdx + newTriangleCount );
	for (size_t i = 0; i < newTriangleCount; i++, triIdx++)
	{
		FatTri& tri = triangles[triIdx];
		tri.material = materialIdx;
		const uint32_t v0idx = tmpIndices[i * 3 + 0];
		const uint32_t v1idx = tmpIndices[i * 3 + 1];
		const uint32_t v2idx = tmpIndices[i * 3 + 2];
		const ts_vec3 v0pos = tmpVertices[v0idx];
		const ts_vec3 v1pos = tmpVertices[v1idx];
		const ts_vec3 v2pos = tmpVertices[v2idx];
		vertices.push_back( ts_vec4( v0pos, 1 ) );
		vertices.push_back( ts_vec4( v1pos, 1 ) );
		vertices.push_back( ts_vec4( v2pos, 1 ) );
		const ts_vec3 N = ts_normalize( ts_cross( v1pos - v0pos, v2pos - v0pos ) );
		tri.Nx = N.x, tri.Ny = N.y, tri.Nz = N.z;
		tri.vertex0 = tmpVertices[v0idx];
		tri.vertex1 = tmpVertices[v1idx];
		tri.vertex2 = tmpVertices[v2idx];
		tri.alpha = ts_vec3( tmpAlphas[v0idx], tmpAlphas[v1idx], tmpAlphas[v2idx] );
		if (tmpNormals.size() > 0)
			tri.vN0 = tmpNormals[v0idx],
			tri.vN1 = tmpNormals[v1idx],
			tri.vN2 = tmpNormals[v2idx];
		else
			tri.vN0 = tri.vN1 = tri.vN2 = N;
		if (tmpUvs.size() > 0)
		{
			tri.u0 = tmpUvs[v0idx].x, tri.v0 = tmpUvs[v0idx].y;
			tri.u1 = tmpUvs[v1idx].x, tri.v1 = tmpUvs[v1idx].y;
			tri.u2 = tmpUvs[v2idx].x, tri.v2 = tmpUvs[v2idx].y;
			if (tri.u0 == tri.u1 && tri.u1 == tri.u2 && tri.v0 == tri.v1 && tri.v1 == tri.v2)
			{
				// this triangle uses only a single point on the texture; replace by single color material.
				int textureID = Scene::materials[materialIdx]->color.textureID;
				if (textureID != -1)
				{
					Texture* texture = Scene::textures[textureID];
					uint32_t u = (uint32_t)(tri.u0 * texture->width) % texture->width;
					uint32_t v = (uint32_t)(tri.v0 * texture->height) % texture->height;
					uint32_t texel = ((uint32_t*)texture->idata)[u + v * texture->width] & 0xffffff;
					tri.material = Scene::FindOrCreateMaterialCopy( materialIdx, texel );
				}
			}
			// calculate tangent vector based on uvs
			ts_vec2 uv01 = ts_vec2( tri.u1 - tri.u0, tri.v1 - tri.v0 );
			ts_vec2 uv02 = ts_vec2( tri.u2 - tri.u0, tri.v2 - tri.v0 );
			if (ts_dot( uv01, uv01 ) == 0 || ts_dot( uv02, uv02 ) == 0)
			{
			#if 1
				// PBRT:
				// https://github.com/mmp/pbrt-v3/blob/3f94503ae1777cd6d67a7788e06d67224a525ff4/src/shapes/triangle.cpp#L381
				if (fabs( N.x ) > fabs( N.y ))
					tri.T = ts_vec3( -N.z, 0, N.x ) * (1.0f / sqrtf( N.x * N.x + N.z * N.z ));
				else
					tri.T = ts_vec3( 0, N.z, -N.y ) * (1.0f / sqrtf( N.y * N.y + N.z * N.z ));
			#else
				tri.T = ts_normalize( tri.vertex1 - tri.vertex0 );
			#endif
				tri.B = ts_normalize( ts_cross( N, tri.T ) );
			}
			else
			{
				tri.T = ts_normalize( (tri.vertex1 - tri.vertex0) * uv02.y - (tri.vertex2 - tri.vertex0) * uv01.y );
				tri.B = ts_normalize( (tri.vertex2 - tri.vertex0) * uv01.x - (tri.vertex1 - tri.vertex0) * uv02.x );
			}
			// catch bad tangents
			if (isnan( tri.T.x + tri.T.y + tri.T.z + tri.B.x + tri.B.y + tri.B.z ))
			{
				tri.T = ts_normalize( tri.vertex1 - tri.vertex0 );
				tri.B = ts_normalize( ts_cross( N, tri.T ) );
			}
		}
		else
		{
			// no uv information; use edges to calculate tangent vectors
			tri.T = ts_normalize( tri.vertex1 - tri.vertex0 );
			tri.B = ts_normalize( ts_cross( N, tri.T ) );
		}
		// handle second and third set of uv coordinates, if available
		if (tmpUv2s.size() > 0)
		{
			tri.u0_2 = tmpUv2s[v0idx].x, tri.v0_2 = tmpUv2s[v0idx].y;
			tri.u1_2 = tmpUv2s[v1idx].x, tri.v1_2 = tmpUv2s[v1idx].y;
			tri.u2_2 = tmpUv2s[v2idx].x, tri.v2_2 = tmpUv2s[v2idx].y;
		}
		// process joints / weights
		if (tmpJoints.size() > 0)
		{
			joints.push_back( tmpJoints[v0idx] );
			joints.push_back( tmpJoints[v1idx] );
			joints.push_back( tmpJoints[v2idx] );
			weights.push_back( tmpWeights[v0idx] );
			weights.push_back( tmpWeights[v1idx] );
			weights.push_back( tmpWeights[v2idx] );
		}
		// build poses
		for (int s = (int)tmpPoses.size(), i = 0; i < s; i++)
		{
			auto& pose = tmpPoses[i];
			poses[i].positions.push_back( pose.positions[v0idx] );
			poses[i].positions.push_back( pose.positions[v1idx] );
			poses[i].positions.push_back( pose.positions[v2idx] );
			poses[i].normals.push_back( pose.normals[v0idx] );
			poses[i].normals.push_back( pose.normals[v1idx] );
			poses[i].normals.push_back( pose.normals[v2idx] );
			if (pose.tangents.size() > 0)
			{
				poses[i].tangents.push_back( pose.tangents[v0idx] );
				poses[i].tangents.push_back( pose.tangents[v1idx] );
				poses[i].tangents.push_back( pose.tangents[v2idx] );
			}
			else
			{
				// have some dummies for now
				poses[i].tangents.push_back( ts_vec3( 0, 1, 0 ) );
				poses[i].tangents.push_back( ts_vec3( 0, 1, 0 ) );
				poses[i].tangents.push_back( ts_vec3( 0, 1, 0 ) );
			}
		}
	}
}

//  +-----------------------------------------------------------------------------+
//  |  Mesh::BuildMaterialList                                                    |
//  |  Update the list of materials used by this mesh. We will use this list to   |
//  |  efficiently find meshes using a specific material, which in turn is useful |
//  |  when a material becomes emissive or non-emissive.                    LH2'25|
//  +-----------------------------------------------------------------------------+
void Mesh::BuildMaterialList()
{
	// mark all materials as 'not seen yet'
	for (auto material : Scene::materials) material->visited = false;
	// add each material
	materialList.clear();
	for (auto tri : triangles)
	{
		Material* material = Scene::materials[tri.material];
		if (!material->visited)
		{
			material->visited = true;
			materialList.push_back( material->ID );
		}
	}
}

//  +-----------------------------------------------------------------------------+
//  |  Mesh::CreateOpacityMicroMaps                                               |
//  |  Fill the opacity micro maps vector.                                  LH2'25|
//  +-----------------------------------------------------------------------------+
void CreateOpacityMicroMap( Mesh* mesh, const int N, const int first, const int last )
{
	const float fN = (float)N;
	const float rN = 1.0f / fN;
	for (int i = first; i < last; i++)
	{
		uint32_t* map = mesh->omaps + i * ((N * N) / 32);
		FatTri& tri = mesh->triangles[i];
		const uint32_t matIdx = tri.material;
		const Material* material = Scene::materials[matIdx];
		Texture* tex = 0;
		if (material->color.textureID > 1)
		{
			tex = Scene::textures[material->color.textureID];
			if (!tex->idata) tex = 0; // can't use hdr textures for now.
		}
		if (!tex) { memset( map, 255, N * N / 8 ); continue; }
		memset( map, 0, N * N / 8 );
		const float u0 = tri.u0, u1 = tri.u1, u2 = tri.u2, w = (float)tex->width;
		const float v0 = tri.v0, v1 = tri.v1, v2 = tri.v2, h = (float)tex->height;
		const int iw = tex->width;
		for (int y = 0; y < N * 4; y++)
		{
			float v = ((float)y + 0.5f) * 0.25f * rN, u = 0.125f / fN;
			for (int x = 0; x < N * 4; x++, u += 0.25f / fN)
			{
				if (u + v >= 1) break;
				// formula from the paper: Sub-triangle opacity masks for faster ray
				// tracing of transparent objects, Gruen et al., 2020
				const int row = int( (u + v) * fN ), diag = int( (1 - u) * fN );
				const int idx = (row * row) + int( v * fN ) + (diag - (N - 1 - row));
				const float tu = u * u1 + v * u2 + (1 - u - v) * u0;
				const float tv = u * v1 + v * v2 + (1 - u - v) * v0;
				const int iu = (int)((tu - floorf( tu )) * w);
				const int iv = (int)((tv - floorf( tv )) * h);
				const uint32_t pixel = ((uint32_t*)tex->idata)[iu + iv * iw];
				if ((pixel >> 24) > 2) map[idx >> 5] |= 1 << (idx & 31);
			}
		}
	}
}
void Mesh::CreateOpacityMicroMaps()
{
	// fill the opacity maps: WIP, we simply take one texture sample.
	printf( "creating opacity micromaps... " );
	const int N = 32; // i.e.: 32 * 32 = 1024bits = 128 bytes = 32 uints per tri.
	free64( omaps );
	omaps = (uint32_t*)malloc64( triangles.size() * N * N / 8 );
	constexpr int slices = 32;
	const int slice = (int)triangles.size() / slices;
	vector<thread> jobs;
	for (int i = 0; i < slices; i++)
	{
		const int first = i * slice;
		const int last = (i == (slices - 1)) ? (int)triangles.size() : (first + slice);
		jobs.emplace_back( std::thread( &CreateOpacityMicroMap, this, N, first, last ) );
	}
	for (int i = 0; i < slices; i++) jobs[i].join();
	hasOpacityMicroMaps = true;
	printf( "done.\n" );
}

//  +-----------------------------------------------------------------------------+
//  |  Mesh::SetPose                                                              |
//  |  Update the geometry data in this mesh using the weights from the node,     |
//  |  and update all dependent data.                                       LH2'25|
//  +-----------------------------------------------------------------------------+
void Mesh::SetPose( const vector<float>& w )
{
	assert( w.size() == poses.size() - 1 /* first pose is base pose */ );
	const int weightCount = (int)w.size();
	// adjust intersection geometry data
	for (int s = (int)vertices.size(), i = 0; i < s; i++)
	{
		vertices[i] = ts_vec4( poses[0].positions[i], 1 );
		for (int j = 1; j <= weightCount; j++) vertices[i] += w[j - 1] * ts_vec4( poses[j].positions[i], 0 );
	}
	// adjust full triangles
	for (int s = (int)triangles.size(), i = 0; i < s; i++)
	{
		triangles[i].vertex0 = ts_vec3( vertices[i * 3 + 0] );
		triangles[i].vertex1 = ts_vec3( vertices[i * 3 + 1] );
		triangles[i].vertex2 = ts_vec3( vertices[i * 3 + 2] );
		triangles[i].vN0 = poses[0].normals[i * 3 + 0];
		triangles[i].vN1 = poses[0].normals[i * 3 + 1];
		triangles[i].vN2 = poses[0].normals[i * 3 + 2];
		for (int j = 1; j <= weightCount; j++)
			triangles[i].vN0 += poses[j].normals[i * 3 + 0],
			triangles[i].vN1 += poses[j].normals[i * 3 + 1],
			triangles[i].vN2 += poses[j].normals[i * 3 + 2];
		triangles[i].vN0 = ts_normalize( triangles[i].vN0 );
		triangles[i].vN1 = ts_normalize( triangles[i].vN1 );
		triangles[i].vN2 = ts_normalize( triangles[i].vN2 );
	}
}

//  +-----------------------------------------------------------------------------+
//  |  Mesh::SetPose                                                              |
//  |  Update the geometry data in this mesh using a skin.                        |
//  |  Called from RenderSystem::UpdateSceneGraph, for skinned mesh nodes.  LH2'25|
//  +-----------------------------------------------------------------------------+
void Mesh::SetPose( const Skin* skin )
{
	// ensure that we have a backup of the original vertex positions
	if (original.size() == 0)
	{
		for (auto& vert : vertices) original.push_back( vert );
		for (auto& tri : triangles)
		{
			origNormal.push_back( tri.vN0 );
			origNormal.push_back( tri.vN1 );
			origNormal.push_back( tri.vN2 );
		}
		vertexNormals.resize( vertices.size() );
	}
	// transform original into vertex vector using skin matrices
	for (int s = (int)vertices.size(), i = 0; i < s; i++)
	{
		ts_uint4 j4 = joints[i];
		ts_vec4 w4 = weights[i];
		ts_mat4 skinMatrix = w4.x * skin->jointMat[j4.x];
		skinMatrix += w4.y * skin->jointMat[j4.y];
		skinMatrix += w4.z * skin->jointMat[j4.z];
		skinMatrix += w4.w * skin->jointMat[j4.w];
		vertices[i] = ts_transform_point( original[i], skinMatrix );
		vertexNormals[i] = ts_normalize( ts_transform_vector( origNormal[i], skinMatrix ) );
	}
	// adjust full triangles
	for (int s = (int)triangles.size(), i = 0; i < s; i++)
	{
		triangles[i].vertex0 = ts_vec3( vertices[i * 3 + 0] );
		triangles[i].vertex1 = ts_vec3( vertices[i * 3 + 1] );
		triangles[i].vertex2 = ts_vec3( vertices[i * 3 + 2] );
		ts_vec3 N = ts_normalize( ts_cross( triangles[i].vertex1 - triangles[i].vertex0, triangles[i].vertex2 - triangles[i].vertex0 ) );
		triangles[i].vN0 = vertexNormals[i * 3 + 0];
		triangles[i].vN1 = vertexNormals[i * 3 + 1];
		triangles[i].vN2 = vertexNormals[i * 3 + 2];
		triangles[i].Nx = N.x;
		triangles[i].Ny = N.y;
		triangles[i].Nz = N.z;
	}
}

// helper function
static FatTri TransformedFatTri( FatTri* tri, ts_mat4 T )
{
	FatTri transformedTri = *tri;
	transformedTri.vertex0 = ts_transform_point( transformedTri.vertex0, T );
	transformedTri.vertex1 = ts_transform_point( transformedTri.vertex1, T );
	transformedTri.vertex2 = ts_transform_point( transformedTri.vertex2, T );
	const ts_vec4 N = ts_normalize( ts_transform_vector( ts_vec3( transformedTri.Nx, transformedTri.Ny, transformedTri.Nz ), T ) );
	transformedTri.Nx = N.x;
	transformedTri.Ny = N.y;
	transformedTri.Nz = N.z;
	return transformedTri;
}

//  +-----------------------------------------------------------------------------+
//  |  Node::Node                                                                 |
//  |  Constructors.                                                        LH2'25|
//  +-----------------------------------------------------------------------------+
Node::Node( const tinygltf::Node& gltfNode, const int nodeBase, const int meshBase, const int skinBase )
{
	ConvertFromGLTFNode( gltfNode, nodeBase, meshBase, skinBase );
}

Node::Node( const int meshIdx, const ts_mat4& transform )
{
	// setup a node based on a mesh index and a transform
	meshID = meshIdx;
	localTransform = transform;
	// process light emitting surfaces
	PrepareLights();
}

//  +-----------------------------------------------------------------------------+
//  |  Node::~Node                                                                |
//  |  Destructor.                                                          LH2'25|
//  +-----------------------------------------------------------------------------+
Node::~Node()
{
	if (meshID > -1 && hasLights)
	{
		// this node is an instance and has emissive materials;
		// remove the relevant area lights.
		Mesh* mesh = Scene::meshPool[meshID];
		for (auto materialIdx : mesh->materialList)
		{
			Material* material = Scene::materials[materialIdx];
			if (material->IsEmissive())
			{
				// mesh contains an emissive material; remove related area lights
				vector<TriLight*>& lightList = Scene::triLights;
				for (int s = (int)lightList.size(), i = 0; i < s; i++)
					if (lightList[i]->instIdx == ID) lightList.erase( lightList.begin() + i-- );
			}
		}
	}
}

//  +-----------------------------------------------------------------------------+
//  |  Node::ConvertFromGLTFNode                                                  |
//  |  Create a node from a GLTF node.                                      LH2'25|
//  +-----------------------------------------------------------------------------+
void Node::ConvertFromGLTFNode( const tinygltf::Node& gltfNode, const int nodeBase, const int meshBase, const int skinBase )
{
	// copy node name
	name = gltfNode.name;
	// set mesh / skin ID
	meshID = gltfNode.mesh == -1 ? -1 : (gltfNode.mesh + meshBase);
	skinID = gltfNode.skin == -1 ? -1 : (gltfNode.skin + skinBase);
	// if the mesh has morph targets, the node should have weights for them
	if (meshID != -1)
	{
		const int morphTargets = (int)Scene::meshPool[meshID]->poses.size() - 1;
		if (morphTargets > 0) weights.resize( morphTargets, 0.0f );
	}
	// copy child node indices
	for (int s = (int)gltfNode.children.size(), i = 0; i < s; i++) childIdx.push_back( gltfNode.children[i] + nodeBase );
	// obtain matrix
	bool buildFromTRS = false;
	if (gltfNode.matrix.size() == 16)
	{
		// we get a full matrix
		for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) matrix.cell[i * 4 + j] = (float)gltfNode.matrix[j * 4 + i];
		buildFromTRS = true;
	}
	if (gltfNode.translation.size() == 3)
	{
		// the GLTF node contains a translation
		translation = ts_vec3( (float)gltfNode.translation[0], (float)gltfNode.translation[1], (float)gltfNode.translation[2] );
		buildFromTRS = true;
	}
	if (gltfNode.rotation.size() == 4)
	{
		// the GLTF node contains a rotation
		rotation = ts_quat( (float)gltfNode.rotation[3], (float)gltfNode.rotation[0], (float)gltfNode.rotation[1], (float)gltfNode.rotation[2] );
		buildFromTRS = true;
	}
	if (gltfNode.scale.size() == 3)
	{
		// the GLTF node contains a scale
		scale = ts_vec3( (float)gltfNode.scale[0], (float)gltfNode.scale[1], (float)gltfNode.scale[2] );
		buildFromTRS = true;
	}
	// if we got T, R and/or S, reconstruct final matrix
	if (buildFromTRS) UpdateTransformFromTRS();
	// process light emitting surfaces
	PrepareLights();
}

//  +-----------------------------------------------------------------------------+
//  |  Node::UpdateTransformFromTRS                                               |
//  |  Process T, R, S data to localTransform.                              LH2'25|
//  +-----------------------------------------------------------------------------+
void Node::UpdateTransformFromTRS()
{
	ts_mat4 T, R, S;
	T[3] = translation.x, T[7] = translation.y, T[11] = translation.z;
	float rx = rotation.x, ry = rotation.y, rz = rotation.z, rw = rotation.w;
	R[0] = 1 - 2 * ry * ry - 2 * rz * rz, R[1] = 2 * rx * ry - 2 * rw * rz;
	R[2] = 2 * rx * rz + 2 * rw * ry, R[4] = 2 * rx * ry + 2 * rw * rz;
	R[5] = 1 - 2 * rx * rx - 2 * rz * rz, R[6] = 2 * ry * rz - 2 * rw * rx;
	R[8] = 2 * rx * rz - 2 * rw * ry, R[9] = 2 * ry * rz + 2 * rw * rx;
	R[10] = 1 - 2 * rx * rx - 2 * ry * ry;
	S[0] = scale.x, S[5] = scale.y, S[10] = scale.z;
	localTransform = T * R * S * matrix;
}

//  +-----------------------------------------------------------------------------+
//  |  Node::Update                                                               |
//  |  Calculates the combined transform for this node and recurses into the      |
//  |  child nodes. If a change is detected, the light triangles are updated      |
//  |  as well.                                                             LH2'25|
//  +-----------------------------------------------------------------------------+
void Node::Update( const ts_mat4& T )
{
	if (transformed /* true if node was affected by animation channel */)
	{
		UpdateTransformFromTRS();
	}
	combinedTransform = T * localTransform;
	// update the combined transforms of the children
	for (int s = (int)childIdx.size(), i = 0; i < s; i++)
	{
		Node* child = Scene::nodePool[childIdx[i]];
		child->Update( combinedTransform );
	}
	// update animations
	if (meshID > -1)
	{
		Mesh* mesh = Scene::meshPool[meshID];
		if (morphed /* true if bone weights were affected by animation channel */)
		{
			mesh->SetPose( weights );
			morphed = false;
		}
		if (skinID > -1)
		{
			ts_mat4 invTransform = combinedTransform;
			ts_invert( invTransform );
			Skin* skin = Scene::skins[skinID];
			for (int s = (int)skin->joints.size(), j = 0; j < s; j++)
			{
				Node* jointNode = Scene::nodePool[skin->joints[j]];
				skin->jointMat[j] = invTransform * jointNode->combinedTransform * skin->inverseBindMatrices[j];
			}
			mesh->SetPose( skin ); // TODO: I hope this doesn't overwrite SetPose(weights) ?
		}
		// update blas
		if (mesh->geomChanged)
		{
			switch (mesh->blas.bvhType)
			{
			case BVH_DYNAMIC:
				if (mesh->blas.dynamicBVH == 0)
				{
					mesh->blas.dynamicBVH = new tinybvh::BVH();
					if (mesh->omaps) mesh->blas.dynamicBVH->SetOpacityMicroMaps( mesh->omaps, 32 );
				}
				mesh->blas.dynamicBVH->Build( (tinybvh::bvhvec4*)mesh->vertices.data(), (unsigned)mesh->triangles.size() );
				break;
			case BVH_RIGID:
				if (mesh->blas.rigidBVH == 0)
				{
				#if defined BVH_USEAVX2
					mesh->blas.rigidBVH = new tinybvh::BVH8_CPU();
				#elif defined BVH_USESSE
					mesh->blas.rigidBVH = new tinybvh::BVH4_CPU();
				#else
					mesh->blas.rigidBVH = new tinybvh::BVH_SoA();
				#endif
					if (mesh->omaps) mesh->blas.rigidBVH->SetOpacityMicroMaps( mesh->omaps, 32 );
				}
				mesh->blas.rigidBVH->BuildHQ( (tinybvh::bvhvec4*)mesh->vertices.data(), (unsigned)mesh->triangles.size() );
				break;
			case GPU_DYNAMIC:
				if (mesh->blas.dynamicGPU == 0)
				{
					mesh->blas.dynamicGPU = new tinybvh::BVH_GPU();
					if (mesh->omaps) mesh->blas.dynamicGPU->SetOpacityMicroMaps( mesh->omaps, 32 );
				}
				mesh->blas.dynamicGPU->Build( (tinybvh::bvhvec4*)mesh->vertices.data(), (unsigned)mesh->triangles.size() );
				break;
			case GPU_RIGID:
			{
				if (mesh->blas.rigidGPU == 0)
				{
					mesh->blas.rigidGPU = new tinybvh::BVH_GPU();
					if (mesh->omaps) mesh->blas.rigidGPU->SetOpacityMicroMaps( mesh->omaps, 32 );
				}
				// attempt to load cached BVH
				bool loaded = false;
				char t[265] = "./cache/", b[265] = "./cache/";
				if (Scene::bvhCaching && mesh->triangles.size() > 50'000)
				{
					printf( "attempt to load cached bvh... " );
					strcat( t, mesh->fileName.c_str() );
					strcat( b, mesh->fileName.c_str() );
					strcat( t, ".tri" );
					strcat( b, ".bvh" );
					tinybvh::BVH& bvh = mesh->blas.rigidGPU->bvh;
					if (bvh.Load( b, (tinybvh::bvhvec4*)mesh->vertices.data(), (unsigned)mesh->triangles.size() ))
					{
						printf( "done. Finalizing... " );
						mesh->blas.rigidGPU->ConvertFrom( bvh, true );
						loaded = true;
						printf( "done.\n" );
					}
				}
				if (!loaded)
				{
					mesh->blas.rigidGPU->BuildHQ( (tinybvh::bvhvec4*)mesh->vertices.data(), (unsigned)mesh->triangles.size() );
					if (Scene::bvhCaching && mesh->triangles.size() > 50'000)
					{
						printf( "not found: building... " );
						FILE* f = fopen( t, "wb" );
						unsigned count = (unsigned)mesh->triangles.size();
						fwrite( &count, 1, 4, f );
						fwrite( mesh->vertices.data(), 16 * 3, count, f );
						fclose( f );
						mesh->blas.rigidGPU->bvh.Save( b );
						printf( "done.\n" );
					}
				}
			}
			break;
			case GPU_STATIC:
			{
				if (mesh->blas.staticGPU == 0)
				{
					mesh->blas.staticGPU = new tinybvh::BVH8_CWBVH();
					if (mesh->omaps) mesh->blas.staticGPU->SetOpacityMicroMaps( mesh->omaps, 32 );
				}
				// attempt to load cached BVH
				bool loaded = false;
				char t[265] = "./cache/", b[265] = "./cache/";
				if (Scene::bvhCaching)
				{
					printf( "attempt to load cached bvh... " );
					strcat( t, mesh->fileName.c_str() );
					strcat( b, mesh->fileName.c_str() );
					strcat( t, ".tri" );
					strcat( b, ".bvh" );
					tinybvh::BVH& bvh = mesh->blas.staticGPU->bvh8.bvh;
					if (bvh.Load( b, (tinybvh::bvhvec4*)mesh->vertices.data(), (unsigned)mesh->triangles.size() ))
					{
						printf( "done. Finalizing... " );
						loaded = true;
						bvh.SplitLeafs( 3 );
						mesh->blas.staticGPU->bvh8.ConvertFrom( bvh, true );
						mesh->blas.staticGPU->ConvertFrom( mesh->blas.staticGPU->bvh8, true );
						printf( "done.\n" );
					}
				}
				if (!loaded)
				{
					mesh->blas.staticGPU->BuildHQ( (tinybvh::bvhvec4*)mesh->vertices.data(), (unsigned)mesh->triangles.size() );
					if (Scene::bvhCaching)
					{
						printf( "not found: building... " );
						FILE* f = fopen( t, "wb" );
						unsigned count = (unsigned)mesh->triangles.size();
						fwrite( &count, 1, 4, f );
						fwrite( mesh->vertices.data(), 16 * 3, count, f );
						fclose( f );
						mesh->blas.staticGPU->bvh8.bvh.Save( b );
						printf( "done.\n" );
					}
				}
			}
			break;
			default:
				// .. invalid bvh type.
				break;
			}
			mesh->geomChanged = false;
		}
		// update instance
		if (instanceID == -1)
		{
			instanceID = (int)Scene::instPool.size();
			Scene::instPool.push_back( tinybvh::BLASInstance( meshID ) );
			transformed = true;
		}
		tinybvh::BLASInstance& instance = Scene::instPool[instanceID];
		bool transformChanged = false;
		for (int i = 0; i < 16; i++) if (instance.transform[i] != combinedTransform[i]) transformChanged = true;
		if (transformChanged)
		{
			instance.transform = *(tinybvh::bvhmat4*)&combinedTransform;
			instance.Update( mesh->blas.dynamicBVH /* type doesn't matter */ );
		}
	}
	transformed = false;
}

//  +-----------------------------------------------------------------------------+
//  |  Node::PrepareLights                                                        |
//  |  Detects emissive triangles and creates light triangles for them.     LH2'25|
//  +-----------------------------------------------------------------------------+
void Node::PrepareLights()
{
	if (meshID > -1)
	{
		Mesh* mesh = Scene::meshPool[meshID];
		for (int s = (int)mesh->triangles.size(), i = 0; i < s; i++)
		{
			FatTri* tri = &mesh->triangles[i];
			uint32_t matIdx = tri->material;
			if (matIdx >= Scene::materials.size()) continue;
			Material* mat = Scene::materials[matIdx];
			if (mat->IsEmissive())
			{
				tri->UpdateArea();
				FatTri transformedTri = TransformedFatTri( tri, localTransform );
				TriLight* light = new TriLight( &transformedTri, i, ID );
				tri->ltriIdx = (int)Scene::triLights.size(); // TODO: can't duplicate a light due to this.
				Scene::triLights.push_back( light );
				hasLights = true;
				// Note: TODO:
				// 1. if a mesh is deleted it should scan the list of area lights
				//    to delete those that no longer exist.
				// 2. if a material is changed from emissive to non-emissive,
				//    meshes using the material should remove their light emitting
				//    triangles from the list of area lights.
				// 3. if a material is changed from non-emissive to emissive,
				//    meshes using the material should update the area lights list.
				// Item 1 can be done efficiently. Items 2 and 3 require a list
				// of materials per mesh to be efficient.
			}
		}
	}
}

//  +-----------------------------------------------------------------------------+
//  |  Node::UpdateLights                                                         |
//  |  Update light triangles belonging to this instance after the tansform for   |
//  |  the node changed.                                                    LH2'25|
//  +-----------------------------------------------------------------------------+
void Node::UpdateLights()
{
	if (!hasLights) return;
	Mesh* mesh = Scene::meshPool[meshID];
	for (int s = (int)mesh->triangles.size(), i = 0; i < s; i++)
	{
		FatTri* tri = &mesh->triangles[i];
		if (tri->ltriIdx > -1)
		{
			// triangle is light emitting; update it
			tri->UpdateArea();
			FatTri transformedTri = TransformedFatTri( tri, combinedTransform );
			*Scene::triLights[tri->ltriIdx] = TriLight( &transformedTri, i, ID );
		}
	}
}

//  +-----------------------------------------------------------------------------+
//  |  Material::ConvertFrom                                                      |
//  |  Converts a tinyobjloader material to an LH2 Material.                LH2'25|
//  +-----------------------------------------------------------------------------+
void Material::ConvertFrom( const ::tinyobj::material_t& original )
{
	// properties
	name = original.name;
	color.value = ts_vec3( original.diffuse[0], original.diffuse[1], original.diffuse[2] ); // Kd
	absorption.value = ts_vec3( original.transmittance[0], original.transmittance[1], original.transmittance[2] ); // Kt
	ior.value = original.ior; // Ni
	roughness = 1.0f;
	// maps
	if (original.diffuse_texname != "")
	{
		color.textureID = Scene::FindOrCreateTexture( original.diffuse_texname, Texture::LINEARIZED | Texture::FLIPPED );
		color.value = ts_vec3( 1 ); // we have a texture now; default modulation to white
	}
	if (original.normal_texname != "")
	{
		normals.textureID = Scene::FindOrCreateTexture( original.normal_texname, Texture::FLIPPED );
		Scene::textures[normals.textureID]->flags |= Texture::NORMALMAP; // TODO: what if it's also used as regular texture?
	}
	else if (original.bump_texname != "")
	{
		int bumpMapID = normals.textureID = Scene::CreateTexture( original.bump_texname, Texture::FLIPPED ); // cannot reuse, height scale may differ
		float heightScaler = 1.0f;
		auto heightScalerIt = original.unknown_parameter.find( "bump_height" );
		if (heightScalerIt != original.unknown_parameter.end()) heightScaler = static_cast<float>(atof( (*heightScalerIt).second.c_str() ));
		Scene::textures[bumpMapID]->BumpToNormalMap( heightScaler );
		Scene::textures[bumpMapID]->flags |= Texture::NORMALMAP; // TODO: what if it's also used as regular texture?
	}
	if (original.specular_texname != "")
	{
		roughness.textureID = Scene::FindOrCreateTexture( original.specular_texname.c_str(), Texture::FLIPPED );
		roughness() = 1.0f;
	}
	// finalize
	auto shadingIt = original.unknown_parameter.find( "shading" );
	if (shadingIt != original.unknown_parameter.end() && shadingIt->second == "flat") flags &= ~SMOOTH; else flags |= SMOOTH;
}

//  +-----------------------------------------------------------------------------+
//  |  Material::ConvertFrom                                                      |
//  |  Converts a tinygltf material to an LH2 Material.                     LH2'25|
//  +-----------------------------------------------------------------------------+
void Material::ConvertFrom( const tinygltf::Material& original, const vector<int>& texIdx )
{
	name = original.name;
	flags |= Material::FROM_MTL; // this material will be serialized on exit.
	// set normal map, if any
	if (original.normalTexture.index > -1)
	{
		// note: may be overwritten by the "normalTexture" field in additionalValues.
		normals.textureID = texIdx[original.normalTexture.index];
		normals.scale = (float)original.normalTexture.scale;
		Scene::textures[normals.textureID]->flags |= Texture::NORMALMAP;
	}
	// process values list
	for (const auto& value : original.values)
	{
		if (value.first == "baseColorFactor")
		{
			tinygltf::Parameter p = value.second;
			color.value = ts_vec3( (float)p.number_array[0], (float)p.number_array[1], (float)p.number_array[2] );
		}
		else if (value.first == "metallicFactor")
		{
			if (value.second.has_number_value)
			{
				metallic.value = (float)value.second.number_value;
			}
		}
		else if (value.first == "roughnessFactor")
		{
			if (value.second.has_number_value)
			{
				roughness.value = (float)value.second.number_value;
			}
		}
		else if (value.first == "baseColorTexture")
		{
			for (auto& item : value.second.json_double_value)
			{
				if (item.first == "index") color.textureID = texIdx[(int)item.second];
			}
		}
		else if (value.first == "metallicRoughnessTexture")
		{
			for (auto& item : value.second.json_double_value)
			{
				if (item.first == "index")
				{
					roughness.textureID = texIdx[(int)item.second];	// green channel contains roughness
					metallic.textureID = texIdx[(int)item.second];	// blue channel contains metalness
				}
			}
		}
		else { /* whatjusthappened */ }
	}
	// process additionalValues list
	for (const auto& value : original.additionalValues)
	{
		if (value.first == "doubleSided") { /* ignored; all faces are double sided in LH2. */ }
		else if (value.first == "normalTexture")
		{
			tinygltf::Parameter p = value.second;
			for (auto& item : value.second.json_double_value)
			{
				if (item.first == "index") normals.textureID = texIdx[(int)item.second];
				if (item.first == "scale") normals.scale = (float)item.second;
				if (item.first == "texCoord") { /* TODO */ };
			}
		}
		else if (value.first == "occlusionTexture") { /* ignored; the occlusion map stores baked AO */ }
		else if (value.first == "emissiveFactor") { /* TODO (used in drone) */ }
		else if (value.first == "emissiveTexture") { /* TODO (used in drone) */ }
		else if (value.first == "alphaMode") { /* TODO (used in drone) */ }
		else { /* capture unexpected values */ }
	}
}

//  +-----------------------------------------------------------------------------+
//  |  RenderMaterial::RenderMaterial                                             |
//  |  Constructor.                                                         LH2'25|
//  +-----------------------------------------------------------------------------+
RenderMaterial::RenderMaterial( const Material* source, const vector<int>& offsets )
{
#define TOCHAR(a) ((uint32_t)((a)*255.0f))
#define TOUINT4(a,b,c,d) (TOCHAR(a)+(TOCHAR(b)<<8)+(TOCHAR(c)<<16)+(TOCHAR(d)<<24))
	memset( this, 0, sizeof( RenderMaterial ) );
	SetDiffuse( source->color.value );
	SetTransmittance( ts_vec3( 1 ) - source->absorption.value );
	parameters.x = TOUINT4( source->metallic.value, source->subsurface.value, source->specular.value, source->roughness.value );
	parameters.y = TOUINT4( source->specularTint.value, source->anisotropic.value, source->sheen.value, source->sheenTint.value );
	parameters.z = TOUINT4( source->clearcoat.value, source->clearcoatGloss.value, source->transmission.value, 0 );
	parameters.w = *((uint32_t*)&source->eta);
	if (source->color.textureID != -1) tex0 = MakeMap( source->color, offsets );
	if (source->detailColor.textureID != -1) tex1 = MakeMap( source->detailColor, offsets );
	if (source->normals.textureID != -1) nmap0 = MakeMap( source->normals, offsets );
	if (source->detailNormals.textureID != -1) nmap1 = MakeMap( source->detailNormals, offsets );
	// if (source->roughness.textureID != -1) rmap = MakeMap( source->roughness, offsets ); // TODO
	// if (source->specular.textureID != -1) smap = MakeMap( source->specular, offsets ); // TODO
	bool hdr = false;
	if (source->color.textureID != -1) if (Scene::textures[source->color.textureID]->flags & Texture::HDR) hdr = true;
	flags = (source->eta.value < 1 ? ISDIELECTRIC : 0) +
		(source->color.textureID != -1 ? HASDIFFUSEMAP : 0) + (hdr ? DIFFUSEMAPISHDR : 0) +
		(source->normals.textureID != -1 ? HASNORMALMAP : 0) +
		(source->specular.textureID != -1 ? HASSPECULARITYMAP : 0) +
		(source->roughness.textureID != -1 ? HASROUGHNESSMAP : 0) +
		(source->detailNormals.textureID != -1 ? HAS2NDNORMALMAP : 0) +
		(source->detailColor.textureID != -1 ? HAS2NDDIFFUSEMAP : 0) +
		((source->flags & 1) ? HASSMOOTHNORMALS : 0) + ((source->flags & 2) ? HASALPHA : 0);
}

//  +-----------------------------------------------------------------------------+
//  |  RenderMaterial::MakeMap                                                    |
//  |  Helper function: Converts a spatially varying (i.e., texture-mapped)       |
//  |  ts_vec3 material property into a 'Map', which is a compact representation  |
//  |  for rendering.                                                       LH2'25|
//  +-----------------------------------------------------------------------------+
RenderMaterial::Map RenderMaterial::MakeMap( Material::vec3Value source, const vector<int>& offsets )
{
	Map map;
	Texture* texture = Scene::textures[source.textureID];
	map.width = (uint16_t)texture->width;
	map.height = (uint16_t)texture->width;
	map.uscale = ts_float_to_half( source.uvscale.x );
	map.vscale = ts_float_to_half( source.uvscale.y );
	map.uoffs = ts_float_to_half( source.uvoffset.x );
	map.voffs = ts_float_to_half( source.uvoffset.y );
	map.addr = offsets[source.textureID];
	return map;
}

//  +-----------------------------------------------------------------------------+
//  |  Animation::Sampler::Sampler                                                |
//  |  Constructor.                                                         LH2'25|
//  +-----------------------------------------------------------------------------+
Animation::Sampler::Sampler( const tinygltf::AnimationSampler& gltfSampler, const tinygltf::Model& gltfModel )
{
	ConvertFromGLTFSampler( gltfSampler, gltfModel );
}

//  +-----------------------------------------------------------------------------+
//  |  Animation::Sampler::ConvertFromGLTFSampler                                 |
//  |  Convert a gltf animation sampler.                                    LH2'25|
//  +-----------------------------------------------------------------------------+
void Animation::Sampler::ConvertFromGLTFSampler( const tinygltf::AnimationSampler& gltfSampler, const tinygltf::Model& gltfModel )
{
	// https://github.com/KhronosGroup/glTF/blob/master/specification/2.0/README.md#animations
	// store interpolation type
	if (gltfSampler.interpolation == "STEP") interpolation = STEP;
	else if (gltfSampler.interpolation == "CUBICSPLINE") interpolation = SPLINE;
	else /* if (gltfSampler.interpolation == "LINEAR" ) */ interpolation = LINEAR;
	// extract animation times
	auto inputAccessor = gltfModel.accessors[gltfSampler.input];
	assert( inputAccessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT );
	auto bufferView = gltfModel.bufferViews[inputAccessor.bufferView];
	auto buffer = gltfModel.buffers[bufferView.buffer];
	const float* a = (const float*)(buffer.data.data() + bufferView.byteOffset + inputAccessor.byteOffset);
	size_t count = inputAccessor.count;
	for (int i = 0; i < count; i++) t.push_back( a[i] );
	// extract animation keys
	auto outputAccessor = gltfModel.accessors[gltfSampler.output];
	bufferView = gltfModel.bufferViews[outputAccessor.bufferView];
	buffer = gltfModel.buffers[bufferView.buffer];
	const uint8_t* b = (const uint8_t*)(buffer.data.data() + bufferView.byteOffset + outputAccessor.byteOffset);
	if (outputAccessor.type == TINYGLTF_TYPE_VEC3)
	{
		// b is an array of floats (for scale or translation)
		float* f = (float*)b;
		const int N = (int)outputAccessor.count;
		for (int i = 0; i < N; i++) vec3Key.push_back( ts_vec3( f[i * 3], f[i * 3 + 1], f[i * 3 + 2] ) );
	}
	else if (outputAccessor.type == TINYGLTF_TYPE_SCALAR)
	{
		// b can be FLOAT, BYTE, UBYTE, SHORT or uint16_t... (for weights)
		::std::vector<float> fdata;
		const int N = (int)outputAccessor.count;
		switch (outputAccessor.componentType)
		{
		case TINYGLTF_COMPONENT_TYPE_FLOAT: for (int k = 0; k < N; k++, b += 4) fdata.push_back( *((float*)b) ); break;
		case TINYGLTF_COMPONENT_TYPE_BYTE: for (int k = 0; k < N; k++, b++) fdata.push_back( max( *((char*)b) / 127.0f, -1.0f ) ); break;
		case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE: for (int k = 0; k < N; k++, b++) fdata.push_back( *((char*)b) / 255.0f ); break;
		case TINYGLTF_COMPONENT_TYPE_SHORT: for (int k = 0; k < N; k++, b += 2) fdata.push_back( max( *((char*)b) / 32767.0f, -1.0f ) ); break;
		case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT: for (int k = 0; k < N; k++, b += 2) fdata.push_back( *((char*)b) / 65535.0f ); break;
		}
		for (int i = 0; i < N; i++) floatKey.push_back( fdata[i] );
	}
	else if (outputAccessor.type == TINYGLTF_TYPE_VEC4)
	{
		// b can be FLOAT, BYTE, UBYTE, SHORT or uint16_t... (for rotation)
		vector<float> fdata;
		const int N = (int)outputAccessor.count * 4;
		switch (outputAccessor.componentType)
		{
		case TINYGLTF_COMPONENT_TYPE_FLOAT: for (int k = 0; k < N; k++, b += 4) fdata.push_back( *((float*)b) ); break;
		case TINYGLTF_COMPONENT_TYPE_BYTE: for (int k = 0; k < N; k++, b++) fdata.push_back( max( *((char*)b) / 127.0f, -1.0f ) ); break;
		case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE: for (int k = 0; k < N; k++, b++) fdata.push_back( *((char*)b) / 255.0f ); break;
		case TINYGLTF_COMPONENT_TYPE_SHORT: for (int k = 0; k < N; k++, b += 2) fdata.push_back( max( *((char*)b) / 32767.0f, -1.0f ) ); break;
		case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT: for (int k = 0; k < N; k++, b += 2) fdata.push_back( *((char*)b) / 65535.0f ); break;
		}
		for (int i = 0; i < outputAccessor.count; i++)
		{
			for (int j = 0; j < 4; j++) if (fpclassify( fdata[i * 4 + j] ) == FP_SUBNORMAL) fdata[i * 4 + j] = 0;
			vec4Key.push_back( ts_quat( fdata[i * 4 + 3], fdata[i * 4], fdata[i * 4 + 1], fdata[i * 4 + 2] ) );
		}
	}
	else assert( false );
}

//  +-----------------------------------------------------------------------------+
//  |  Sampler::SampleFloat, SampleVec3, SampleVec4                               |
//  |  Get a value from the sampler.                                        LH2'25|
//  +-----------------------------------------------------------------------------+
float Animation::Sampler::SampleFloat( float currentTime, int k, int i, int count ) const
{
	// handle valid out-of-bounds
	if (k == 0 && currentTime < t[0]) return interpolation == SPLINE ? floatKey[1] : floatKey[0];
	// determine interpolation parameters
	const float t0 = t[k], t1 = t[k + 1];
	const float f = (currentTime - t0) / (t1 - t0);
	// sample
	if (f <= 0) return floatKey[0]; else switch (interpolation)
	{
	case SPLINE:
	{
		const float tt = f, t2 = tt * tt, t3 = t2 * tt;
		const float p0 = floatKey[(k * count + i) * 3 + 1];
		const float m0 = (t1 - t0) * floatKey[(k * count + i) * 3 + 2];
		const float p1 = floatKey[((k + 1) * count + i) * 3 + 1];
		const float m1 = (t1 - t0) * floatKey[((k + 1) * count + i) * 3];
		return m0 * (t3 - 2 * t2 + tt) + p0 * (2 * t3 - 3 * t2 + 1) + p1 * (-2 * t3 + 3 * t2) + m1 * (t3 - t2);
	}
	case Sampler::STEP:
		return floatKey[k];
	default:
		return (1 - f) * floatKey[k * count + i] + f * floatKey[(k + 1) * count + i];
	};
}
ts_vec3 Animation::Sampler::SampleVec3( float currentTime, int k ) const
{
	// handle valid out-of-bounds
	if (k == 0 && currentTime < t[0]) return interpolation == SPLINE ? vec3Key[1] : vec3Key[0];
	// determine interpolation parameters
	const float t0 = t[k], t1 = t[k + 1];
	const float f = (currentTime - t0) / (t1 - t0);
	// sample
	if (f <= 0) return vec3Key[0]; else switch (interpolation)
	{
	case SPLINE:
	{
		const float tt = f, t2 = tt * tt, t3 = t2 * tt;
		const ts_vec3 p0 = vec3Key[k * 3 + 1];
		const ts_vec3 m0 = (t1 - t0) * vec3Key[k * 3 + 2];
		const ts_vec3 p1 = vec3Key[(k + 1) * 3 + 1];
		const ts_vec3 m1 = (t1 - t0) * vec3Key[(k + 1) * 3];
		return m0 * (t3 - 2 * t2 + tt) + p0 * (2 * t3 - 3 * t2 + 1) + p1 * (-2 * t3 + 3 * t2) + m1 * (t3 - t2);
	}
	case Sampler::STEP: return vec3Key[k];
	default: return (1 - f) * vec3Key[k] + f * vec3Key[k + 1];
	};
}
ts_quat Animation::Sampler::SampleQuat( float currentTime, int k ) const
{
	// handle valid out-of-bounds
	if (k == 0 && currentTime < t[0]) return interpolation == SPLINE ? vec4Key[1] : vec4Key[0];
	// determine interpolation parameters
	const float t0 = t[k], t1 = t[k + 1];
	const float f = (currentTime - t0) / (t1 - t0);
	// sample
	ts_quat key;
	if (f <= 0) return vec4Key[0]; else switch (interpolation)
	{
	#if 1
	case SPLINE:
	{
		const float tt = f, t2 = tt * tt, t3 = t2 * tt;
		const ts_quat p0 = vec4Key[k * 3 + 1];
		const ts_quat m0 = vec4Key[k * 3 + 2] * (t1 - t0);
		const ts_quat p1 = vec4Key[(k + 1) * 3 + 1];
		const ts_quat m1 = vec4Key[(k + 1) * 3] * (t1 - t0);
		key = m0 * (t3 - 2 * t2 + tt) + p0 * (2 * t3 - 3 * t2 + 1) + p1 * (-2 * t3 + 3 * t2) + m1 * (t3 - t2);
		key.ts_normalize();
		break;
	}
#endif
	case STEP:
		key = vec4Key[k];
		break;
	default:
		key = ts_quat::slerp( vec4Key[k], vec4Key[k + 1], f );
		key.ts_normalize();
		break;
	};
	return key;
}

//  +-----------------------------------------------------------------------------+
//  |  Animation::Channel::Channel                                                |
//  |  Constructor.                                                         LH2'25|
//  +-----------------------------------------------------------------------------+
Animation::Channel::Channel( const tinygltf::AnimationChannel& gltfChannel, const int nodeBase )
{
	ConvertFromGLTFChannel( gltfChannel, nodeBase );
}

//  +-----------------------------------------------------------------------------+
//  |  Animation::Channel::ConvertFromGLTFChannel                                 |
//  |  Convert a gltf animation channel.                                    LH2'25|
//  +-----------------------------------------------------------------------------+
void Animation::Channel::ConvertFromGLTFChannel( const tinygltf::AnimationChannel& gltfChannel, const int nodeBase )
{
	samplerIdx = gltfChannel.sampler;
	nodeIdx = gltfChannel.target_node + nodeBase;
	if (gltfChannel.target_path.compare( "translation" ) == 0) target = 0;
	if (gltfChannel.target_path.compare( "rotation" ) == 0) target = 1;
	if (gltfChannel.target_path.compare( "scale" ) == 0) target = 2;
	if (gltfChannel.target_path.compare( "weights" ) == 0) target = 3;
}

//  +-----------------------------------------------------------------------------+
//  |  Animation::Channel::Update                                                 |
//  |  Advance channel animation time.                                      LH2'25|
//  +-----------------------------------------------------------------------------+
void Animation::Channel::Update( const float dt, const Sampler* sampler )
{
	// advance animation timer
	t += dt;
	const int keyCount = (int)sampler->t.size();
	const float animDuration = sampler->t[keyCount - 1];
	if (animDuration == 0 /* book scene */ || keyCount == 1 /* bird */)
	{
		if (target == 0) // translation
		{
			Scene::nodePool[nodeIdx]->translation = sampler->vec3Key[0];
			Scene::nodePool[nodeIdx]->transformed = true;
		}
		else if (target == 1) // rotation
		{
			Scene::nodePool[nodeIdx]->rotation = sampler->vec4Key[0];
			Scene::nodePool[nodeIdx]->transformed = true;
		}
		else if (target == 2) // scale
		{
			Scene::nodePool[nodeIdx]->scale = sampler->vec3Key[0];
			Scene::nodePool[nodeIdx]->transformed = true;
		}
		else // target == 3, weight
		{
			int weightCount = (int)Scene::nodePool[nodeIdx]->weights.size();
			for (int i = 0; i < weightCount; i++)
				Scene::nodePool[nodeIdx]->weights[i] = sampler->floatKey[0];
			Scene::nodePool[nodeIdx]->morphed = true;
		}
	}
	else
	{
		while (t >= animDuration) t -= animDuration, k = 0;
		while (t >= sampler->t[(k + 1) % keyCount]) k++;
		// apply anination key
		if (target == 0) // translation
		{
			Scene::nodePool[nodeIdx]->translation = sampler->SampleVec3( t, k );
			Scene::nodePool[nodeIdx]->transformed = true;
		}
		else if (target == 1) // rotation
		{
			Scene::nodePool[nodeIdx]->rotation = sampler->SampleQuat( t, k );
			Scene::nodePool[nodeIdx]->transformed = true;
		}
		else if (target == 2) // scale
		{
			Scene::nodePool[nodeIdx]->scale = sampler->SampleVec3( t, k );
			Scene::nodePool[nodeIdx]->transformed = true;
		}
		else // target == 3, weight
		{
			int weightCount = (int)Scene::nodePool[nodeIdx]->weights.size();
			for (int i = 0; i < weightCount; i++)
				Scene::nodePool[nodeIdx]->weights[i] = sampler->SampleFloat( t, k, i, weightCount );
			Scene::nodePool[nodeIdx]->morphed = true;
		}
	}
}

//  +-----------------------------------------------------------------------------+
//  |  Animation::Animation                                                       |
//  |  Constructor.                                                         LH2'25|
//  +-----------------------------------------------------------------------------+
Animation::Animation( tinygltf::Animation& gltfAnim, tinygltf::Model& gltfModel, const int nodeBase )
{
	ConvertFromGLTFAnim( gltfAnim, gltfModel, nodeBase );
}

//  +-----------------------------------------------------------------------------+
//  |  Animation::ConvertFromGLTFAnim                                             |
//  |  Convert a gltf animation.                                            LH2'25|
//  +-----------------------------------------------------------------------------+
void Animation::ConvertFromGLTFAnim( tinygltf::Animation& gltfAnim, tinygltf::Model& gltfModel, const int nodeBase )
{
	for (int i = 0; i < gltfAnim.samplers.size(); i++) sampler.push_back( new Sampler( gltfAnim.samplers[i], gltfModel ) );
	for (int i = 0; i < gltfAnim.channels.size(); i++) channel.push_back( new Channel( gltfAnim.channels[i], nodeBase ) );
}

//  +-----------------------------------------------------------------------------+
//  |  Animation::SetTime                                                         |
//  |  Set the animation timers of all channels to a specific value.        LH2'25|
//  +-----------------------------------------------------------------------------+
void Animation::SetTime( const float t )
{
	for (int i = 0; i < channel.size(); i++) channel[i]->SetTime( t );
}

//  +-----------------------------------------------------------------------------+
//  |  Animation::Reset                                                           |
//  |  Reset the animation timers of all channels.                          LH2'25|
//  +-----------------------------------------------------------------------------+
void Animation::Reset()
{
	for (int i = 0; i < channel.size(); i++) channel[i]->Reset();
}

//  +-----------------------------------------------------------------------------+
//  |  Animation::Update                                                          |
//  |  Advance channel animation timers.                                    LH2'25|
//  +-----------------------------------------------------------------------------+
void Animation::Update( const float dt )
{
	for (int i = 0; i < channel.size(); i++) channel[i]->Update( dt, sampler[channel[i]->samplerIdx] );
}

//  +-----------------------------------------------------------------------------+
//  |  Texture::Texture                                                           |
//  |  Constructor.                                                         LH2'25|
//  +-----------------------------------------------------------------------------+
Texture::Texture( const char* fileName, const uint32_t modFlags )
{
	Load( fileName, modFlags );
	origin = std::string( fileName );
}

//  +-----------------------------------------------------------------------------+
//  |  Texture::sRGBtoLinear                                                      |
//  |  Convert sRGB data to linear color space.                             LH2'25|
//  +-----------------------------------------------------------------------------+
void Texture::sRGBtoLinear( uint8_t* pixels, const uint32_t size, const uint32_t stride )
{
	for (uint32_t j = 0; j < size; j++)
	{
		pixels[j * stride + 0] = (pixels[j * stride + 0] * pixels[j * stride + 0]) >> 8;
		pixels[j * stride + 1] = (pixels[j * stride + 1] * pixels[j * stride + 1]) >> 8;
		pixels[j * stride + 2] = (pixels[j * stride + 2] * pixels[j * stride + 2]) >> 8;
	}
}

//  +-----------------------------------------------------------------------------+
//  |  Texture::Equals                                                            |
//  |  Returns true if the fields that identify the texture are identical to the  |
//  |  supplied values. Used for texture reuse by the Scene object.         LH2'25|
//  +-----------------------------------------------------------------------------+
bool Texture::Equals( const std::string& o, const uint32_t m )
{
	if (mods != m) return false;
	if (o.compare( origin )) return false;
	return true;
}

//  +-----------------------------------------------------------------------------+
//  |  Texture::PixelsNeeded                                                      |
//  |  Helper function that determines the number of pixels that should be        |
//  |  allocated for the given width, height and MIP level count.           LH2'25|
//  +-----------------------------------------------------------------------------+
int Texture::PixelsNeeded( int w, int h, const int l /* >= 1; includes base layer */ ) const
{
	int needed = 0;
	for (int i = 0; i < l; i++) needed += w * h, w >>= 1, h >>= 1;
	return needed;
}

//  +-----------------------------------------------------------------------------+
//  |  Texture::ConstructMIPmaps                                                  |
//  |  Generate MIP levels for a loaded texture.                            LH2'25|
//  +-----------------------------------------------------------------------------+
void Texture::ConstructMIPmaps()
{
	uint32_t* src = (uint32_t*)idata;
	uint32_t* dst = src + width * height;
	int pw = width, w = width >> 1, ph = height, h = height >> 1;
	for (int i = 1; i < MIPLEVELCOUNT; i++)
	{
		// reduce
		for (int y = 0; y < h; y++) for (int x = 0; x < w; x++)
		{
			const uint32_t src0 = src[x * 2 + (y * 2) * pw];
			const uint32_t src1 = src[x * 2 + 1 + (y * 2) * pw];
			const uint32_t src2 = src[x * 2 + (y * 2 + 1) * pw];
			const uint32_t src3 = src[x * 2 + 1 + (y * 2 + 1) * pw];
			const uint32_t a = min( min( (src0 >> 24) & 255, (src1 >> 24) & 255 ), min( (src2 >> 24) & 255, (src3 >> 24) & 255 ) );
			const uint32_t r = ((src0 >> 16) & 255) + ((src1 >> 16) & 255) + ((src2 >> 16) & 255) + ((src3 >> 16) & 255);
			const uint32_t g = ((src0 >> 8) & 255) + ((src1 >> 8) & 255) + ((src2 >> 8) & 255) + ((src3 >> 8) & 255);
			const uint32_t b = (src0 & 255) + (src1 & 255) + (src2 & 255) + (src3 & 255);
			dst[x + y * w] = (a << 24) + ((r >> 2) << 16) + ((g >> 2) << 8) + (b >> 2);
		}
		// next layer
		src = dst, dst += w * h, pw = w, ph = h, w >>= 1, h >>= 1;
	}
}

//  +-----------------------------------------------------------------------------+
//  |  Texture::Load                                                              |
//  |  Load texture data from disk.                                         LH2'25|
//  +-----------------------------------------------------------------------------+
void Texture::Load( const char* fileName, const uint32_t modFlags, bool normalMap )
{
#ifdef CACHEIMAGES
	// see if we can load a cached version
	if (strlen( fileName ) > 4) if (fileName[strlen( fileName ) - 4] == '.')
	{
		char binFile[1024];
		::memcpy( binFile, fileName, strlen( fileName ) + 1 );
		binFile[strlen( fileName ) - 4] = 0;
		strcat_s( binFile, ".bin" );
		FILE* f;
	#ifdef _MSC_VER
		fopen_s( &f, binFile, "rb" );
	#else
		f = fopen( binFile, "rb" );
	#endif
		if (f)
		{
			uint32_t version;
			::fread( &version, 1, 4, f );
			if (version == BINTEXFILEVERSION)
			{
				::fread( &width, 4, 1, f );
				::fread( &height, 4, 1, f );
				int dataType;
				::fread( &dataType, 4, 1, f );
				::fread( &mods, 4, 1, f );
				::fread( &flags, 4, 1, f );
				::fread( &MIPlevels, 4, 1, f );
				if (dataType == 0)
				{
					int pixelCount = PixelsNeeded( width, height, 1 /* no MIPS for HDR textures */ );
					fdata = (ts_vec4*)malloc64( sizeof( ts_vec4 ) * pixelCount );
					::fread( fdata, sizeof( ts_vec4 ), pixelCount, f );
				}
				else
				{
					int pixelCount = PixelsNeeded( width, height, MIPLEVELCOUNT );
					idata = (ts_uchar4*)malloc64( sizeof( ts_uchar4 ) * pixelCount );
					::fread( idata, 4, pixelCount, f );
				}
				::fclose( f );
				mods = modFlags;
				return;
			}
		}
	}
#endif
	bool hdr = strstr( fileName, ".hdr" ) != 0;
	if (hdr)
	{
		// load .hdr file
		SCENE_FATAL_ERROR( "TODO: Implement .hdr texture loading." );
		flags |= HDR;
	}
	else
	{
		// load integer image data
		int n, w, h;
		uint8_t* data = stbi_load( fileName, &w, &h, &n, 0 );
		SCENE_FATAL_ERROR_IF( data == 0, "File does not exist." );
		int pixels = w * h;
		for (int i = 1; i < MIPLEVELCOUNT; i++) pixels += (w * h) >> i;
		idata = (ts_uchar4*)malloc64( pixels * sizeof( uint32_t ) );
		const int s = w * h;
		for (int i = 0; i < s; i++) ((uint32_t*)idata)[i] = (data[i * n + 0] << 16) + (data[i * n + 1] << 8) + data[i * n + 2];
		stbi_image_free( data );
		flags |= LDR, width = w, height = h;
		// perform sRGB -> linear conversion if requested
		if (mods & LINEARIZED) sRGBtoLinear( (uint8_t*)idata, width * height, 4 );
		// produce the MIP maps
		ConstructMIPmaps();
	}
	// mark normal map
	if (normalMap) flags |= NORMALMAP;
#ifdef CACHEIMAGES
	// prepare binary blob to be faster next time
	if (strlen( fileName ) > 4) if (fileName[strlen( fileName ) - 4] == '.')
	{
		char binFile[1024];
		::memcpy( binFile, fileName, strlen( fileName ) + 1 );
		binFile[strlen( fileName ) - 4] = 0;
		strcat_s( binFile, ".bin" );
		FILE* f = fopen( binFile, "rb" );
		if (f)
		{
			uint32_t version = BINTEXFILEVERSION;
			fwrite( &version, 4, 1, f );
			fwrite( &width, 4, 1, f );
			fwrite( &height, 4, 1, f );
			int dataType = fdata ? 0 : 1;
			fwrite( &dataType, 4, 1, f );
			fwrite( &mods, 4, 1, f );
			fwrite( &flags, 4, 1, f );
			fwrite( &MIPlevels, 4, 1, f );
			if (dataType == 0) fwrite( fdata, sizeof( ts_vec4 ), PixelsNeeded( width, height, 1 ), f );
			else fwrite( idata, 4, PixelsNeeded( width, height, MIPLEVELCOUNT ), f );
			fclose( f );
		}
	}
#endif
}

//  +-----------------------------------------------------------------------------+
//  |  Texture::BumpToNormalMap                                                   |
//  |  Convert a bumpmap to a normalmap.                                    LH2'25|
//  +-----------------------------------------------------------------------------+
void Texture::BumpToNormalMap( float heightScale )
{
	uint8_t* normalMap = new uint8_t[width * height * 4];
	const float stepZ = 1.0f / 255.0f;
	for (uint32_t i = 0; i < width* height; i++)
	{
		uint32_t xCoord = i % width, yCoord = i / width;
		float xPrev = xCoord > 0 ? idata[i - 1].x * stepZ : idata[i].x * stepZ;
		float xNext = xCoord < width - 1 ? idata[i + 1].x * stepZ : idata[i].x * stepZ;
		float yPrev = yCoord < height - 1 ? idata[i + width].x * stepZ : idata[i].x * stepZ;
		float yNext = yCoord > 0 ? idata[i - width].x * stepZ : idata[i].x * stepZ;
		ts_vec3 normal;
		normal.x = (xPrev - xNext) * heightScale;
		normal.y = (yPrev - yNext) * heightScale;
		normal.z = 1;
		normal = ts_normalize( normal );
		normalMap[i * 4 + 0] = (uint8_t)round( (normal.x * 0.5 + 0.5) * 255 );
		normalMap[i * 4 + 1] = (uint8_t)round( (normal.y * 0.5 + 0.5) * 255 );
		normalMap[i * 4 + 2] = (uint8_t)round( (normal.z * 0.5 + 0.5) * 255 );
		normalMap[i * 4 + 3] = 255;
	}
	if (width * height > 0) memcpy( idata, normalMap, width * height * 4 );
	delete normalMap;
}

//  +-----------------------------------------------------------------------------+
//  |  TriLight::TriLight                                                         |
//  |  Constructor.                                                         LH2'25|
//  +-----------------------------------------------------------------------------+
TriLight::TriLight( FatTri* origTri, int origIdx, int origInstance )
{
	triIdx = origIdx;
	instIdx = origInstance;
	vertex0 = origTri->vertex0;
	vertex1 = origTri->vertex1;
	vertex2 = origTri->vertex2;
	centre = 0.333333f * (vertex0 + vertex1 + vertex2);
	N = ts_vec3( origTri->Nx, origTri->Ny, origTri->Nz );
	const float a = ts_length( vertex1 - vertex0 );
	const float b = ts_length( vertex2 - vertex1 );
	const float c = ts_length( vertex0 - vertex2 );
	const float s = (a + b + c) * 0.5f;
	area = sqrtf( s * (s - a) * (s - b) * (s - c) ); // Heron's formula
	radiance = Scene::materials[origTri->material]->color();
	const ts_vec3 E = radiance * area;
	energy = E.x + E.y + E.z;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::~Scene                                                        LH2'25|
//  +-----------------------------------------------------------------------------+
Scene::~Scene()
{
	// clean up allocated objects
	for (auto mesh : meshPool) delete mesh;
	for (auto material : materials) delete material;
	for (auto texture : textures) delete texture;
	delete sky;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::AddMesh                                                             |
//  |  Add an existing Mesh to the list of meshes and return the mesh ID.   LH2'25|
//  +-----------------------------------------------------------------------------+
int Scene::AddMesh( Mesh* mesh )
{
	// see if the mesh is already in the scene
	for (int s = (int)meshPool.size(), i = 0; i < s; i++) if (meshPool[i] == mesh)
	{
		assert( mesh->ID == i );
		return i;
	}
	// add the mesh
	mesh->ID = (int)meshPool.size();
	meshPool.push_back( mesh );
	return mesh->ID;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::AddMesh                                                             |
//  |  Create a mesh specified by a file name and data dir, apply a scale, add    |
//  |  the mesh to the list of meshes and return the mesh ID.               LH2'25|
//  +-----------------------------------------------------------------------------+
int Scene::AddMesh( const char* objFile, const float scale, const bool flatShaded )
{
	// extract directory from specified file name
	char* tmp = new char[strlen( objFile ) + 1];
	memcpy( tmp, objFile, strlen( objFile ) + 1 );
	char* lastSlash = tmp, * pos = tmp;
	while (*pos) { if (*pos == '/' || *pos == '\\') lastSlash = pos; pos++; }
	*lastSlash = 0;
	return AddMesh( lastSlash + 1, tmp, scale, flatShaded );
}
int Scene::AddMesh( const char* objFile, const char* dir, const float scale, const bool flatShaded )
{
	Mesh* newMesh = new Mesh( objFile, dir, scale, flatShaded );
	return AddMesh( newMesh );
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::AddMesh                                                             |
//  |  Create a mesh with the specified amount of triangles without actually      |
//  |  setting the triangles. Set these via the AddTriToMesh function.      LH2'25|
//  +-----------------------------------------------------------------------------+
int Scene::AddMesh( const int triCount )
{
	Mesh* newMesh = new Mesh( triCount );
	return AddMesh( newMesh );
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::AddTriToMesh                                                        |
//  |  Add a single triangle to a mesh.                                     LH2'25|
//  +-----------------------------------------------------------------------------+
void Scene::AddTriToMesh( const int meshId, const ts_vec3& v0, const ts_vec3& v1, const ts_vec3& v2, const int matId )
{
	Mesh* m = Scene::meshPool[meshId];
	m->vertices.push_back( ts_vec4( v0, 1 ) );
	m->vertices.push_back( ts_vec4( v1, 1 ) );
	m->vertices.push_back( ts_vec4( v2, 1 ) );
	FatTri tri;
	tri.material = matId;
	ts_vec3 N = ts_normalize( ts_cross( v1 - v0, v2 - v0 ) );
	tri.vN0 = tri.vN1 = tri.vN2 = N;
	tri.Nx = N.x, tri.Ny = N.y, tri.Nz = N.z;
	tri.vertex0 = v0;
	tri.vertex1 = v1;
	tri.vertex2 = v2;
	m->triangles.push_back( tri );
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::AddScene                                                            |
//  |  Loads a collection of meshes from a gltf file. An instance and a scene     |
//  |  graph node is created for each mesh.                                 LH2'25|
//  +-----------------------------------------------------------------------------+
int Scene::AddScene( const char* sceneFile, const ts_mat4& transform )
{
	// extract directory from specified file name
	char* tmp = new char[strlen( sceneFile ) + 1];
	::memcpy( tmp, sceneFile, strlen( sceneFile ) + 1 );
	char* lastSlash = tmp, * pos = tmp;
	while (*pos) { if (*pos == '/' || *pos == '\\') lastSlash = pos; pos++; }
	*lastSlash = 0;
	int retVal = AddScene( lastSlash + 1, tmp, transform );
	delete tmp;
	return retVal;
}
int Scene::AddScene( const char* sceneFile, const float scale )
{
	ts_mat4 T;
	T[0] = T[5] = T[10] = scale;
	return AddScene( sceneFile, T );
}
int Scene::AddScene( const char* sceneFile, const char* dir, const ts_mat4& transform )
{
	// offsets: if we loaded an object before this one, indices should not start at 0.
	// based on https://github.com/SaschaWillems/Vulkan-glTF-PBR/blob/master/base/VulkanglTFModel.hpp
	const int meshBase = (int)meshPool.size();
	const int skinBase = (int)skins.size();
	const int retVal = (int)nodePool.size();
	const int nodeBase = (int)nodePool.size() + 1;
	// load gltf file
	string cleanFileName = string( dir ) + (dir[strlen( dir ) - 1] == '/' ? "" : "/") + string( sceneFile );
	tinygltf::Model gltfModel;
	tinygltf::TinyGLTF loader;
	std::string err, warn;
	bool ret = false;
	if (cleanFileName.size() > 4)
	{
		string extension4 = cleanFileName.substr( cleanFileName.size() - 5, 5 );
		string extension3 = cleanFileName.substr( cleanFileName.size() - 4, 4 );
		if (extension3.compare( ".obj" ) == 0)
		{
			// ugly to have this here, but it sure does make it easier to make a scene
			// out of a single .obj file...
			ts_vec3 scale3( transform.cell[0], transform.cell[5], transform.cell[10] );
			float scale = (scale3.x == scale3.y && scale3.y == scale3.z) ? scale3.x : 1.0f;
			uint32_t meshId = AddMesh( sceneFile, dir, scale );
			ts_mat4 S;
			S[0] = S[5] = S[10] = 1.0f / scale;
			AddInstance( AddNode( new Node( meshId, transform * S ) ) );
			return retVal;
		}
		else if (extension4.compare( ".gltf" ) == 0)
		{
			ret = loader.LoadASCIIFromFile( &gltfModel, &err, &warn, cleanFileName.c_str() );
		}
		else if (extension3.compare( ".bin" ) == 0 || extension3.compare( ".glb" ) == 0)
		{
			ret = loader.LoadBinaryFromFile( &gltfModel, &err, &warn, cleanFileName.c_str() );
		}
	}
	if (!warn.empty()) printf( "Warn: %s\n", warn.c_str() );
	if (!err.empty()) printf( "Err: %s\n", err.c_str() );
	SCENE_FATAL_ERROR_IF( !ret, "could not load glTF file" /* , cleanFileName.c_str() */ );
	// convert textures
	vector<int> texIdx;
	for (size_t s = gltfModel.textures.size(), i = 0; i < s; i++)
	{
		char t[1024];
		sprintf_s( t, "%s-%s-%03i", dir, sceneFile, (int)i );
		int textureID = FindTextureID( t );
		if (textureID != -1)
		{
			// push id of existing texture
			texIdx.push_back( textureID );
		}
		else
		{
			// create new texture
			tinygltf::Texture& gltfTexture = gltfModel.textures[i];
			Texture* texture = new Texture();
			const tinygltf::Image& image = gltfModel.images[gltfTexture.source];
			const size_t size = image.component * image.width * image.height;
			texture->name = t;
			texture->width = image.width;
			texture->height = image.height;
			texture->idata = (ts_uchar4*)malloc64( texture->PixelsNeeded( image.width, image.height, MIPLEVELCOUNT ) * sizeof( uint32_t ) );
			texture->ID = (uint32_t)textures.size();
			texture->flags |= Texture::LDR;
			::memcpy( texture->idata, image.image.data(), size );
			texture->ConstructMIPmaps();
			textures.push_back( texture );
			texIdx.push_back( texture->ID );
		}
	}
	// convert materials
	vector<int> matIdx;
	for (size_t s = gltfModel.materials.size(), i = 0; i < s; i++)
	{
		char t[1024];
		sprintf_s( t, "%s-%s-%03i", dir, sceneFile, (int)i );
		int matID = FindMaterialIDByOrigin( t );
		if (matID != -1)
		{
			// material already exists; reuse
			matIdx.push_back( matID );
		}
		else
		{
			// create new material
			tinygltf::Material& gltfMaterial = gltfModel.materials[i];
			Material* material = new Material();
			material->ID = (int)materials.size();
			material->origin = t;
			material->ConvertFrom( gltfMaterial, texIdx );
			material->flags |= Material::FROM_MTL;
			materials.push_back( material );
			matIdx.push_back( material->ID );
			// materialList.push_back( material->ID ); // can't do that, need something smarter.
		}
	}
	// convert meshes
	for (size_t s = gltfModel.meshes.size(), i = 0; i < s; i++)
	{
		tinygltf::Mesh& gltfMesh = gltfModel.meshes[i];
		Mesh* newMesh = new Mesh( gltfMesh, gltfModel, matIdx, gltfModel.materials.size() == 0 ? 0 : -1 );
		newMesh->ID = (int)i + meshBase;
		// prepare file name ID for mesh
		char fileID[256];
		strcpy( fileID, sceneFile );
		if (strlen( fileID ) > 4)
		{
			if (fileID[strlen( fileID ) - 4] == '.') fileID[strlen( fileID ) - 4] = 0;
			if (fileID[strlen( fileID ) - 5] == '.') fileID[strlen( fileID ) - 5] = 0;
		}
		strcat( fileID, "_9999" );
		for (int d = 1000, i = 0; i < 4; i++, d /= 10) fileID[strlen( fileID ) - 4 + i] = (newMesh->ID / d) % 10 + '0';
		newMesh->fileName = string( fileID );
		meshPool.push_back( newMesh );
	}
	// push an extra node that holds a transform for the gltf scene
	Node* newNode = new Node();
	newNode->localTransform = transform;
	newNode->ID = nodeBase - 1;
	nodePool.push_back( newNode );
	// convert nodes
	for (size_t s = gltfModel.nodes.size(), i = 0; i < s; i++)
	{
		tinygltf::Node& gltfNode = gltfModel.nodes[i];
		Node* node = new Node( gltfNode, nodeBase, meshBase, skinBase );
		node->ID = (int)nodePool.size();
		nodePool.push_back( node );
	}
	// convert animations and skins
	for (tinygltf::Animation& gltfAnim : gltfModel.animations)
	{
		Animation* anim = new Animation( gltfAnim, gltfModel, nodeBase );
		animations.push_back( anim );
	}
	for (tinygltf::Skin& source : gltfModel.skins)
	{
		Skin* newSkin = new Skin( source, gltfModel, nodeBase );
		skins.push_back( newSkin );
	}
	// construct a scene graph for scene 0, assuming the GLTF file has one scene
	tinygltf::Scene& glftScene = gltfModel.scenes[0];
	// add the root nodes to the scene transform node
	for (size_t i = 0; i < glftScene.nodes.size(); i++) nodePool[nodeBase - 1]->childIdx.push_back( glftScene.nodes[i] + nodeBase );
	// add the root transform to the scene
	rootNodes.push_back( nodeBase - 1 );
	// return index of first created node
	return retVal;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::AddQuad                                                             |
//  |  Create a mesh that consists of two triangles, described by a normal, a     |
//  |  centroid position and a material.                                    LH2'25|
//  +-----------------------------------------------------------------------------+
int Scene::AddQuad( ts_vec3 N, const ts_vec3 pos, const float width, const float height, const int matId, const int meshID )
{
	Mesh* newMesh = meshID > -1 ? meshPool[meshID] : new Mesh();
	N = ts_normalize( N ); // let's not assume the normal is normalized.
#if 1
	const ts_vec3 tmp = fabs( N.x ) > 0.9f ? ts_vec3( 0, 1, 0 ) : ts_vec3( 1, 0, 0 );
	const ts_vec3 T = 0.5f * width * ts_normalize( ts_cross( N, tmp ) );
	const ts_vec3 B = 0.5f * height * ts_normalize( ts_cross( ts_normalize( T ), N ) );
#else
	// "Building an Orthonormal Basis, Revisited"
	const float sign = copysignf( 1.0f, N.z ), a = -1.0f / (sign + N.z), b = N.x * N.y * a;
	const ts_vec3 B = 0.5f * width * ts_vec3( 1.0f + sign * N.x * N.x * a, sign * b, -sign * N.x );
	const ts_vec3 T = 0.5f * height * ts_vec3( b, sign + N.y * N.y * a, -N.y );
#endif
	// calculate corners
	uint32_t vertBase = (uint32_t)newMesh->vertices.size();
	newMesh->vertices.push_back( ts_vec4( pos - B - T, 1 ) );
	newMesh->vertices.push_back( ts_vec4( pos + B - T, 1 ) );
	newMesh->vertices.push_back( ts_vec4( pos - B + T, 1 ) );
	newMesh->vertices.push_back( ts_vec4( pos + B - T, 1 ) );
	newMesh->vertices.push_back( ts_vec4( pos + B + T, 1 ) );
	newMesh->vertices.push_back( ts_vec4( pos - B + T, 1 ) );
	// triangles
	FatTri tri1, tri2;
	tri1.material = tri2.material = matId;
	tri1.vN0 = tri1.vN1 = tri1.vN2 = N;
	tri2.vN0 = tri2.vN1 = tri2.vN2 = N;
	tri1.Nx = N.x, tri1.Ny = N.y, tri1.Nz = N.z;
	tri2.Nx = N.x, tri2.Ny = N.y, tri2.Nz = N.z;
	tri1.u0 = tri1.u1 = tri1.u2 = tri1.v0 = tri1.v1 = tri1.v2 = 0;
	tri2.u0 = tri2.u1 = tri2.u2 = tri2.v0 = tri2.v1 = tri2.v2 = 0;
	tri1.vertex0 = ts_vec3( newMesh->vertices[vertBase + 0] );
	tri1.vertex1 = ts_vec3( newMesh->vertices[vertBase + 1] );
	tri1.vertex2 = ts_vec3( newMesh->vertices[vertBase + 2] );
	tri2.vertex0 = ts_vec3( newMesh->vertices[vertBase + 3] );
	tri2.vertex1 = ts_vec3( newMesh->vertices[vertBase + 4] );
	tri2.vertex2 = ts_vec3( newMesh->vertices[vertBase + 5] );
	tri1.T = tri2.T = T * (2.0f / height);
	tri1.B = tri2.B = B * (2.0f / width);
	newMesh->triangles.push_back( tri1 );
	newMesh->triangles.push_back( tri2 );
	// if the mesh was newly created, add it to scene mesh list
	if (meshID == -1)
	{
		newMesh->ID = (int)meshPool.size();
		newMesh->materialList.push_back( matId );
		meshPool.push_back( newMesh );
	}
	return newMesh->ID;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::AddInstance                                                         |
//  |  Add an instance of an existing mesh to the scene.                    LH2'25|
//  +-----------------------------------------------------------------------------+
int Scene::AddInstance( const int nodeId )
{
	const uint32_t instId = (uint32_t)rootNodes.size();
	rootNodes.push_back( nodeId );
	return instId;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::AddNode                                                             |
//  |  Add a node to the scene.                                             LH2'25|
//  +-----------------------------------------------------------------------------+
int Scene::AddNode( Node* newNode )
{
	newNode->ID = (int)nodePool.size();
	nodePool.push_back( newNode );
	return newNode->ID;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::AddChildNode                                                        |
//  |  Add a child to a node.                                               LH2'25|
//  +-----------------------------------------------------------------------------+
int Scene::AddChildNode( const int parentNodeId, const int childNodeId )
{
	const int childIdx = (int)nodePool[parentNodeId]->childIdx.size();
	nodePool[parentNodeId]->childIdx.push_back( childNodeId );
	return childIdx;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::GetChildId                                                          |
//  |  Get the node index of a child of a node.                             LH2'25|
//  +-----------------------------------------------------------------------------+
int Scene::GetChildId( const int parentId, const int childIdx )
{
	return nodePool[parentId]->childIdx[childIdx];
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::RemoveNode                                                          |
//  |  Remove a node from the scene.                                              |
//  |  This also removes the node from the rootNodes vector. Note that this will  |
//  |  only work correctly if the node is not part of a hierarchy, which is the   |
//  |  case for nodes that have been created using AddInstance.                   |
//  |  TODO: This will not delete any textures or materials.                LH2'25|
//  +-----------------------------------------------------------------------------+
void Scene::RemoveNode( const int nodeId )
{
	// remove the instance from the scene graph
	for (int s = (int)rootNodes.size(), i = 0; i < s; i++) if (rootNodes[i] == nodeId)
	{
		rootNodes[i] = rootNodes[s - 1];
		rootNodes.pop_back();
		break;
	}
	// delete the instance
	Node* node = nodePool[nodeId];
	nodePool[nodeId] = 0; // safe; we only access the nodes vector indirectly.
	delete node;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::FindTextureID                                                       |
//  |  Return a texture ID if it already exists.                            LH2'25|
//  +-----------------------------------------------------------------------------+
int Scene::FindTextureID( const char* name )
{
	for (auto texture : textures) if (strcmp( texture->name.c_str(), name ) == 0) return texture->ID;
	return -1;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::FindOrCreateTexture                                                 |
//  |  Return a texture: if it already exists, return the existing texture (after |
//  |  increasing its refCount), otherwise, create a new texture.           LH2'25|
//  +-----------------------------------------------------------------------------+
int Scene::FindOrCreateTexture( const string& origin, const uint32_t modFlags )
{
	// search list for existing texture
	for (auto texture : textures) if (texture->Equals( origin, modFlags ))
	{
		texture->refCount++;
		return texture->ID;
	}
	// nothing found, create a new texture
	return CreateTexture( origin, modFlags );
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::FindOrCreateMaterial                                                |
//  |  Return a material: if it already exists, return the existing material      |
//  |  (after increasing its refCount), otherwise, create a new texture.    LH2'25|
//  +-----------------------------------------------------------------------------+
int Scene::FindOrCreateMaterial( const string& name )
{
	// search list for existing texture
	for (auto material : materials) if (material->name.compare( name ) == 0)
	{
		material->refCount++;
		return material->ID;
	}
	// nothing found, create a new texture
	const int newID = AddMaterial( ts_vec3( 0 ) );
	materials[newID]->name = name;
	return newID;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::FindOrCreateMaterialCopy                                            |
//  |  Create an untextured material, based on an existing material. This copy is |
//  |  to be used for a triangle that only reads a single texel from a texture;   |
//  |  using a single color is more efficient.                              LH2'25|
//  +-----------------------------------------------------------------------------+
int Scene::FindOrCreateMaterialCopy( const int matID, const uint32_t color )
{
	// search list for existing material copy
	const int r = (color >> 16) & 255, g = (color >> 8) & 255, b = color & 255;
	const ts_vec3 c = ts_vec3( b * (1.0f / 255.0f), g * (1.0f / 255.0f), r * (1.0f / 255.0f) );
	for (auto material : materials)
	{
		if (material->flags & Material::SINGLE_COLOR_COPY &&
			material->color.value.x == c.x && material->color.value.y == c.y && material->color.value.z == c.z)
		{
			material->refCount++;
			return material->ID;
		}
	}
	// nothing found, create a new material copy
	const int newID = AddMaterial( ts_vec3( 0 ) );
	*materials[newID] = *materials[matID];
	materials[newID]->color.textureID = -1;
	materials[newID]->color.value = c;
	materials[newID]->flags |= Material::SINGLE_COLOR_COPY;
	materials[newID]->ID = newID;
	char t[256];
	sprintf( t, "copied_mat_%i", newID );
	materials[newID]->name = t;
	return newID;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::FindMaterialID                                                      |
//  |  Find the ID of a material with the specified name.                   LH2'25|
//  +-----------------------------------------------------------------------------+
int Scene::FindMaterialID( const char* name )
{
	for (auto material : materials) if (material->name.compare( name ) == 0) return material->ID;
	return -1;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::FindMaterialIDByOrigin                                              |
//  |  Find the ID of a material with the specified origin.                 LH2'25|
//  +-----------------------------------------------------------------------------+
int Scene::FindMaterialIDByOrigin( const char* name )
{
	for (auto material : materials) if (material->origin.compare( name ) == 0) return material->ID;
	return -1;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::FindNextMaterialID                                                  |
//  |  Find the ID of a material with the specified name, with an ID greater than |
//  |  the specified one. Used to find materials with the same name.        LH2'25|
//  +-----------------------------------------------------------------------------+
int Scene::FindNextMaterialID( const char* name, const int matID )
{
	for (int s = (int)materials.size(), i = matID + 1; i < s; i++)
		if (materials[i]->name.compare( name ) == 0) return materials[i]->ID;
	return -1;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::FindNode                                                            |
//  |  Find the ID of a node with the specified name.                       LH2'25|
//  +-----------------------------------------------------------------------------+
int Scene::FindNode( const char* name )
{
	for (auto node : nodePool) if (node->name.compare( name ) == 0) return node->ID;
	return -1;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::CreateOpacityMicroMaps                                              |
//  |  process all meshes in the specified subtree and produce opacity            |
//  |  maps for them.                                                       LH2'25|
//  +-----------------------------------------------------------------------------+
void Scene::CreateOpacityMicroMaps( const int nodeId )
{
	Node* node = nodePool[nodeId];
	int m = node->meshID;
	if (m > -1) meshPool[m]->CreateOpacityMicroMaps();
	for (int id : node->childIdx) CreateOpacityMicroMaps( id );
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::SetBVHType                                                          |
//  |  set the BVH type for the meshes in a subtree.                        LH2'25|
//  +-----------------------------------------------------------------------------+
void Scene::SetBVHType( const int nodeId, const int t )
{
	Node* node = nodePool[nodeId];
	int m = node->meshID;
	if (m > -1) meshPool[m]->blas.bvhType = t;
	for (int id : node->childIdx) SetBVHType( id, t );
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::FindMeshNode                                                        |
//  |  Find in a subtree the node that references the specified mesh.       LH2'25|
//  +-----------------------------------------------------------------------------+
int Scene::FindMeshNode( const int nodeId, const int meshId )
{
	Node* node = nodePool[nodeId];
	if (node->meshID == meshId) return nodeId;
	for (int id : node->childIdx)
	{
		int r = FindMeshNode( id, meshId );
		if (r != -1) return r;
	}
	return -1;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::CollapseMeshes                                                      |
//  |  Combine all meshes in this subtree into a single one.                      |
//  |  Returns the node index of the mesh.                                  LH2'25|
//  +-----------------------------------------------------------------------------+
int Scene::CollapseMeshes( const int subtreeRoot )
{
	// count triangles in subtree
	int triCount = 0, firstMesh = -1, firstNode = -1, stack[128], stackPtr = 0;
	bool badMesh = false;
	int nodeId = subtreeRoot;
	printf( "collapsing subtree into single mesh... " );
	while (1)
	{
		Node* node = nodePool[nodeId];
		int m = node->meshID;
		if (m > -1)
		{
			if (meshPool[m]->joints.size() > 0 || meshPool[m]->omaps != 0)
			{
				// can't collapse animation / meshes with opacity micromaps
				badMesh = true;
				break;
			}
			triCount += (int)meshPool[m]->triangles.size();
			if (firstMesh == -1) firstMesh = m, firstNode = nodeId;
		}
		for (int id : node->childIdx) stack[stackPtr++] = id;
		if (stackPtr == 0) break;
		nodeId = stack[--stackPtr];
	}
	// safety net
	if (triCount == 0 /* no meshes in subtree */ || badMesh /* issues */) return 0;
	// make room for extra data in 'firstMesh'
	Mesh* first = meshPool[firstMesh];
	first->vertices.resize( triCount * 3 );
	first->triangles.resize( triCount );
	// append data from all meshes in subtree and delete them.
	nodeId = subtreeRoot;
	while (1)
	{
		Node* node = nodePool[nodeId];
		int m = node->meshID;
		if (m > -1 && m != firstMesh)
		{
			Mesh* second = meshPool[m];
			first->vertices.insert( first->vertices.end(), second->vertices.begin(), second->vertices.end() );
			first->triangles.insert( first->triangles.end(), second->triangles.begin(), second->triangles.end() );
			second->ID = -1; // mesh will not be included in BVH builds.
			node->meshID = -1;
			// erase storage of second node
			second->vertices.clear();
			second->triangles.clear();
			second->materialList.clear();
			meshPool[m] = 0;
		#if 0
			// erasing it from the meshPool messes up our administration..
			meshPool.erase( std::find( meshPool.begin(), meshPool.end(), second ) );
		#endif
		}
		for (int id : node->childIdx) stack[stackPtr++] = id;
		if (stackPtr == 0) break;
		nodeId = stack[--stackPtr];
	}
	// renumber mesh 'first'
	for (int i = 0; i < (int)meshPool.size(); i++) if (meshPool[i] == first)
	{
		first->ID = i;
		nodePool[firstNode]->meshID = i;
	}
	// all done!
	printf( "done.\n" );
	return firstMesh;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::SetNodeTransform                                                    |
//  |  Set the local transform for the specified node.                      LH2'25|
//  +-----------------------------------------------------------------------------+
void Scene::SetNodeTransform( const int nodeId, const ts_mat4& transform )
{
	if (nodeId < 0 || nodeId >= nodePool.size()) return;
	nodePool[nodeId]->localTransform = transform;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::GetNodeTransform                                                    |
//  |  Set the local transform for the specified node.                      LH2'25|
//  +-----------------------------------------------------------------------------+
const ts_mat4& Scene::GetNodeTransform( const int nodeId )
{
	static ts_mat4 dummyIdentity;
	if (nodeId < 0 || nodeId >= nodePool.size()) return dummyIdentity;
	return nodePool[nodeId]->localTransform;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::ResetAnimation                                                      |
//  |  Reset the indicated animation.                                       LH2'25|
//  +-----------------------------------------------------------------------------+
void Scene::ResetAnimation( const int animId )
{
	if (animId < 0 || animId >= animations.size()) return;
	animations[animId]->Reset();
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::ResetAnimations                                                     |
//  |  Reset all animations.                                                LH2'25|
//  +-----------------------------------------------------------------------------+
void Scene::ResetAnimations()
{
	for (auto a : animations) a->Reset();
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::ResetAnimation                                                      |
//  |  Update the indicated animation.                                      LH2'25|
//  +-----------------------------------------------------------------------------+
void Scene::UpdateAnimation( const int animId, const float dt )
{
	if (animId < 0 || animId >= animations.size()) return;
	animations[animId]->Update( dt );
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::CreateTexture                                                       |
//  |  Return a texture. Create it anew, even if a texture with the same origin   |
//  |  already exists.                                                      LH2'25|
//  +-----------------------------------------------------------------------------+
int Scene::CreateTexture( const string& origin, const uint32_t modFlags )
{
	// create a new texture
	Texture* newTexture = new Texture( origin.c_str(), modFlags );
	textures.push_back( newTexture );
	return newTexture->ID = (int)textures.size() - 1;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::AddMaterial                                                         |
//  |  Adds an existing Material* and returns the ID. If the material             |
//  |  with that pointer is already added, it is not added again.           LH2'25|
//  +-----------------------------------------------------------------------------+
int Scene::AddMaterial( Material* material )
{
	auto res = find( materials.begin(), materials.end(), material );
	if (res != materials.end()) return (int)distance( materials.begin(), res );
	int matid = (int)materials.size();
	materials.push_back( material );
	return matid;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::AddMaterial                                                         |
//  |  Create a material, with a limited set of parameters.                 LH2'25|
//  +-----------------------------------------------------------------------------+
int Scene::AddMaterial( const ts_vec3 color, const char* name )
{
	Material* material = new Material();
	material->color = color;
	if (name) material->name = name;
	return material->ID = AddMaterial( material );
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::AddPointLight                                                       |
//  |  Create a point light and add it to the scene.                        LH2'25|
//  +-----------------------------------------------------------------------------+
int Scene::AddPointLight( const ts_vec3 pos, const ts_vec3 radiance )
{
	PointLight* light = new PointLight();
	light->position = pos;
	light->radiance = radiance;
	light->ID = (int)pointLights.size();
	pointLights.push_back( light );
	return light->ID;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::AddSpotLight                                                        |
//  |  Create a spot light and add it to the scene.                         LH2'25|
//  +-----------------------------------------------------------------------------+
int Scene::AddSpotLight( const ts_vec3 pos, const ts_vec3 direction, const float inner, const float outer, const ts_vec3 radiance )
{
	SpotLight* light = new SpotLight();
	light->position = pos;
	light->direction = direction;
	light->radiance = radiance;
	light->cosInner = inner;
	light->cosOuter = outer;
	light->ID = (int)spotLights.size();
	spotLights.push_back( light );
	return light->ID;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::AddDirectionalLight                                                 |
//  |  Create a directional light and add it to the scene.                  LH2'25|
//  +-----------------------------------------------------------------------------+
int Scene::AddDirectionalLight( const ts_vec3 direction, const ts_vec3 radiance )
{
	DirectionalLight* light = new DirectionalLight();
	light->direction = direction;
	light->radiance = radiance;
	light->ID = (int)directionalLights.size();
	directionalLights.push_back( light );
	return light->ID;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::UpdateSceneGraph (formerly in RenderSystem)                         |
//  |  Walk the scene graph, updating all node matrices.                    LH2'25|
//  +-----------------------------------------------------------------------------+
void Scene::UpdateSceneGraph( const float deltaTime )
{
	// play animations
	for (int s = AnimationCount(), i = 0; i < s; i++)
		UpdateAnimation( i, deltaTime );
	// update poses, concatenate matrices, rebuild BVHs
	for (int nodeIdx : rootNodes)
	{
		Node* node = nodePool[nodeIdx];
		ts_mat4 T;
		node->Update( T /* start with an identity matrix */ );
	}
	// update tlas
	static tinybvh::BVHBase** bvhList = 0;
	static uint32_t bvhListSize = 0;
	if (bvhListSize != meshPool.size())
	{
		delete bvhList;
		bvhList = new tinybvh::BVHBase * [meshPool.size()];
		for (int i = 0; i < meshPool.size(); i++) if (meshPool[i])
			bvhList[i] = meshPool[i]->blas.dynamicBVH;
		bvhListSize = (uint32_t)meshPool.size();
	}
	if (defaultBVHType == GPU_DYNAMIC || defaultBVHType == GPU_RIGID || defaultBVHType == GPU_STATIC)
	{
		if (!gpuTlas) gpuTlas = new tinybvh::BVH_GPU();
		gpuTlas->Build( instPool.data(), (uint32_t)instPool.size(), bvhList, bvhListSize );
	}
	else
	{
		if (!tlas) tlas = new tinybvh::BVH();
		tlas->Build( instPool.data(), (uint32_t)instPool.size(), bvhList, bvhListSize );
	}
}

} // namespace tinyscene

#endif // TINYSCENE_IMPLEMENTATION