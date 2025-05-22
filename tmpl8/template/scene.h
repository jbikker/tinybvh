// Template, IGAD version 3
// Get the latest version from: https://github.com/jbikker/tmpl8
// IGAD/NHTV/BUAS/UU - Jacco Bikker - 2006-2023

#pragma once

#define ENABLE_OPENCL_BVH

// LIGHTHOUSE 2 SCENE MANAGEMENT CODE - QUICK OVERVIEW
//
// This file defines the data structures for the Lighthouse 2 scene graph. 
// It is designed to conveniently load and/or construct 3D scenes, using a 
// combination of .gtlf / .obj files and extra triangles.
// 
// The basis is a 'scenegraph': a hierarchy of nodes with 4x4 matrix transforms and
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
// Note about macro TRACKCHANGES: This is used to automatically detect changed
// objects int the scene, so they can be synchronized to the GPU.
// 
// Note about the use of pointers: This is intentionally minimal. Most objects are
// stored in vectors in class Scene; references to these are specified as indices
// in these vectors. E.g.: Mesh::ID stores the index of a mesh in Scene::meshPool.
//
// Architectural Limitations: There is very little support for *deleting* anything 
// in the scene. Adding this properly may require significant engineering.
// Scene::RemoveNode exists, but it only removes nodes themselves, not any
// linked materials or textures.

#define TINYGLTF_NO_STB_IMAGE_WRITE
#include "tiny_gltf.h"
#include "tiny_obj_loader.h"

#define MIPLEVELCOUNT		5
#define BINTEXFILEVERSION	0x10001001
#define CACHEIMAGES

namespace Tmpl8
{

//  +-----------------------------------------------------------------------------+
//  |  FatTri                                                                     |
//  |  Full triangle data (for shading only), carefully layed out.          LH2'24|
//  +-----------------------------------------------------------------------------+
class FatTri
{
public:
	FatTri() { memset( this, 0, sizeof( FatTri ) ); ltriIdx = -1; }
	// 128 bytes of data needed for basic shading with texture and normal interpolation.
	float u0, u1, u2;					// 12 bytes for layer 0 texture coordinates
	int ltriIdx;						// 4, set only for emissive triangles, used for MIS
	float v0, v1, v2;					// 12 bytes for layer 0 texture coordinates
	uint material;						// 4 bytes for triangle material index
	float3 vN0;							// 12 bytes for vertex0 normal
	float Nx;							// 4 bytes for x-component of geometric triangle normal
	float3 vN1;							// 12 bytes for vertex1 normal
	float Ny;							// 4 bytes for y-component of geometric triangle normal
	float3 vN2;							// 12 bytes for vertex2 normal
	float Nz;							// 4 bytes for z-component of geometric triangle normal
	float3 vertex0; float u0_2;			// vertex 0 position + second layer u0
	float3 vertex1; float u1_2;			// vertex 1 position + second layer u1
	float3 vertex2; float u2_2;			// vertex 2 position + second layer u2
	// 64 bytes of data needed for normal mapping, anisotropic, multi-texture etc.
	float3 T;							// 12 bytes for tangent vector
	float area;							// 4 bytes for triangle area
	float3 B;							// 12 bytes for bitangent vector
	float invArea;						// 4 bytes for reciprocal triangle area
	float3 alpha;						// better spot than vertex0..2.w for reasons
	float LOD;							// for MIP mapping
	float v0_2, v1_2, v2_2;				// 12 bytes for second layer texture coordinates
	float dummy4;						// padding
	// total FatTri size: 192 bytes.
	void UpdateArea()
	{
		const float a = length( vertex1 - vertex0 ), b = length( vertex2 - vertex1 );
		const float c = length( vertex0 - vertex2 ), s = (a + b + c) * 0.5f;
		area = sqrtf( s * (s - a) * (s - b) * (s - c) ); // Heron's formula
	}
};

//  +-----------------------------------------------------------------------------+
//  |  SkyDome                                                                    |
//  |  Stores data for a HDR sky dome.                                      LH2'24|
//  +-----------------------------------------------------------------------------+
class SkyDome
{
public:
	// constructor / destructor
	SkyDome() = default;
	SkyDome( const char* file ) { Load( file ); }
	void Load( const char* filename, const float3 scale = { 1.f, 1.f, 1.f } );
	// public data members
	float3* pixels = nullptr;			// HDR texture data for sky dome
	int width = 0, height = 0;			// width and height of the sky texture
	mat4 worldToLight;					// for PBRT scenes; transform for skydome
	TRACKCHANGES;						// add Changed(), MarkAsDirty() methods, see system.h
};

//  +-----------------------------------------------------------------------------+
//  |  Skin                                                                       |
//  |  Skin data storage.                                                   LH2'24|
//  +-----------------------------------------------------------------------------+
class Skin
{
public:
	Skin( const tinygltf::Skin& gltfSkin, const tinygltf::Model& gltfModel, const int nodeBase );
	void ConvertFromGLTFSkin( const tinygltf::Skin& gltfSkin, const tinygltf::Model& gltfModel, const int nodeBase );
	string name;
	int skeletonRoot = 0;
	vector<mat4> inverseBindMatrices, jointMat;
	vector<int> joints; // node indices of the joints
};

//  +-----------------------------------------------------------------------------+
//  |  Mesh                                                                       |
//  |  Mesh data storage.                                                   LH2'24|
//  +-----------------------------------------------------------------------------+
class Mesh
{
public:
	struct Pose { vector<float3> positions, normals, tangents; };
	// constructor / destructor
	Mesh() = default;
	Mesh( const int triCount );
	Mesh( const char* name, const char* dir, const float scale = 1.0f, const bool flatShaded = false );
	Mesh( const tinygltf::Mesh& gltfMesh, const tinygltf::Model& gltfModel, const vector<int>& matIdx, const int materialOverride = -1 );
	~Mesh() { /* TODO */ }
	// methods
	void LoadGeometry( const char* file, const char* dir, const float scale = 1.0f, const bool flatShaded = false );
	void LoadGeometryFromOBJ( const string& fileName, const char* directory, const mat4& transform, const bool flatShaded = false );
	void ConvertFromGTLFMesh( const tinygltf::Mesh& gltfMesh, const tinygltf::Model& gltfModel, const vector<int>& matIdx, const int materialOverride );
	void BuildFromIndexedData( const vector<int>& tmpIndices, const vector<float3>& tmpVertices,
		const vector<float3>& tmpNormals, const vector<float2>& tmpUvs, const vector<float2>& tmpUv2s,
		const vector<float4>& tmpTs, const vector<Pose>& tmpPoses,
		const vector<uint4>& tmpJoints, const vector<float4>& tmpWeights, const int materialIdx );
	void BuildMaterialList();
	void SetPose( const vector<float>& weights );
	void SetPose( const Skin* skin );
	void UpdateBVH();
	void UpdateWorldBounds();
	int Intersect( Ray& ray );
	// data members
	string name = "unnamed";			// name for the mesh						
	int ID = -1;						// unique ID for the mesh: position in mesh array
	vector<float4> vertices;			// model vertices, always 3 per triangle: meshes are *not* indexed.
	vector<float3> vertexNormals;		// vertex normals, 1 per triangle
	vector<float4> original;			// skinning: base pose; will be transformed into vector vertices
	vector<float3> origNormal;			// skinning: base pose normals
	vector<FatTri> triangles;			// full triangles, to be used for shading
	vector<int> materialList;			// list of materials used by the mesh; used to efficiently track light changes
	vector<uint4> joints;				// skinning: joints
	vector<float4> weights;				// skinning: joint weights
	vector<Pose> poses;					// morph target data
	bool isAnimated;					// true when this mesh has animation data
	bool excludeFromNavmesh = false;	// prevents mesh from influencing navmesh generation (e.g. curtains)
	mat4 transform, invTransform;		// copy of combined transform of parent node, for TLAS construction
	TRACKCHANGES;						// add Changed(), MarkAsDirty() methods, see system.h
	aabb worldBounds;					// mesh bounds transformed by the transform of the parent node, for TLAS builds
	BVH* bvh = 0;						// bounding volume hierarchy for ray tracing; excluded from change tracking.
};

//  +-----------------------------------------------------------------------------+
//  |  Node                                                                       |
//  |  Simple node for construction of a scene graph for the scene.         LH2'24|
//  +-----------------------------------------------------------------------------+
class Node
{
public:
	// constructor / destructor
	Node() = default;
	Node( const int meshIdx, const mat4& transform );
	Node( const tinygltf::Node& gltfNode, const int nodeBase, const int meshBase, const int skinBase );
	~Node();
	// methods
	void ConvertFromGLTFNode( const tinygltf::Node& gltfNode, const int nodeBase, const int meshBase, const int skinBase );
	void Update( const mat4& T );		// recursively update the transform of this node and its children
	void UpdateTransformFromTRS();		// process T, R, S data to localTransform
	void PrepareLights();				// create light trianslges from detected emissive triangles
	void UpdateLights();				// fix light triangles when the transform changes
	// data members
	string name;						// node name as specified in the GLTF file
	float3 translation = { 0 };			// T
	quat rotation;						// R
	float3 scale = make_float3( 1 );	// S
	mat4 matrix;						// object transform
	mat4 localTransform;				// = matrix * T * R * S, in case of animation
	mat4 combinedTransform;				// transform combined with ancestor transforms
	int ID = -1;						// unique ID for the node: position in node array
	int meshID = -1;					// id of the mesh this node refers to (if any, -1 otherwise)
	int skinID = -1;					// id of the skin this node refers to (if any, -1 otherwise)
	vector<float> weights;				// morph target weights
	bool hasLights = false;				// true if this instance uses an emissive material
	bool morphed = false;				// node mesh should update pose
	bool transformed = false;			// local transform of node should be updated
	bool treeChanged = false;			// this node or one of its children got updated
	vector<int> childIdx;				// child nodes of this node
	TRACKCHANGES;
protected:
	int instanceID = -1;				// for mesh nodes: location in the instance array. For internal use only.
};

//  +-----------------------------------------------------------------------------+
//  |  Material                                                                   |
//  |  Full material definition, which contains everything that can be read from  |
//  |  a gltf file. Will need to be digested to something more practical for      |
//  |  rendering: For that there is 'Material'.                             LH2'24|
//  +-----------------------------------------------------------------------------+
class Material
{
public:
	enum { DISNEYBRDF = 1, LAMBERTBSDF, /* add extra here */ };
	struct Float3Value
	{
		// Float3Value / ScalarValue: all material parameters can be spatially variant or invariant.
		// If a map is used, this map may have an offset and scale. The map values may also be
		// scaled, to facilitate parameter map reuse.
		Float3Value() = default;
		Float3Value( const float f ) : value( make_float3( f ) ) {}
		Float3Value( const float3 f ) : value( f ) {}
		float3 value = make_float3( 1e-32f );	// default value if map is absent; 1e-32 means: not set
		float dummy;							// because float3 is 12 bytes.
		int textureID = -1;						// texture ID; 'value'field is used if -1
		float scale = 1;						// map values will be scaled by this
		float2 uvscale = make_float2( 1 );		// uv coordinate scale
		float2 uvoffset = make_float2( 0 );		// uv coordinate offset
		uint2 size = make_uint2( 0 );			// texture dimensions
		// a parameter that has not been specified has a -1 textureID and a 1e-32f value
		bool Specified() { return value.x != 1e32f || value.y != 1e32f || value.z != 1e32f || textureID != -1; }
		float3& operator()() { return value; }
	};
	struct ScalarValue
	{
		ScalarValue() = default;
		ScalarValue( const float f ) : value( f ) {}
		float value = 1e-32f;					// default value if map is absent; 1e32 means: not set
		int textureID = -1;						// texture ID; -1 denotes empty slot
		int component = 0;						// 0 = x, 1 = y, 2 = z, 3 = w
		float scale = 1;						// map values will be scaled by this
		float2 uvscale = make_float2( 1 );		// uv coordinate scale
		float2 uvoffset = make_float2( 0 );		// uv coordinate offset
		uint2 size = make_uint2( 0 );			// texture dimensions
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
	void ConvertFrom( const tinyobj::material_t& );
	void ConvertFrom( const tinygltf::Material&, const vector<int>& texIdx );
	bool IsEmissive() { float3& c = color(); return c.x > 1 || c.y > 1 || c.z > 1; /* ignores vec3map */ }
	// material properties
	Float3Value color = Float3Value( 1 );	// universal material property: base color
	Float3Value detailColor;			// universal material property: detail texture
	Float3Value normals;				// universal material property: normal map
	Float3Value detailNormals;			// universal material property: detail normal map			
	uint flags = SMOOTH;				// material flags: default is SMOOTH
	// Disney BRDF properties: data for the Disney Principled BRDF
	Float3Value absorption;
	ScalarValue metallic, subsurface, specular, roughness, specularTint, anisotropic;
	ScalarValue sheen, sheenTint, clearcoat, clearcoatGloss, transmission, eta;
	// Lambert BSDF properties, augmented with pure specular reflection and refraction
	// FloatValue absorption;			// shared with disney brdf
	ScalarValue reflection, refraction, ior;
	// identifier and name
	string name = "unnamed";			// material name, not for unique identification
	string origin;						// origin: file from which the data was loaded, with full path
	int ID = -1;						// unique integer ID of this material
	uint refCount = 1;					// the number of models that use this material
	// field for the BuildMaterialList method of Mesh
	bool visited = false;				// last mesh that checked this material
	// internal
private:
	uint prevFlags = SMOOTH;			// initially identical to flags
	TRACKCHANGES;						// add Changed(), MarkAsDirty() methods, see system.h
};

//  +-----------------------------------------------------------------------------+
//  |  Material                                                                   |
//  |  Material definition. This is the version for actual rendering; it stores   |
//  |  a subset of the full data of a Material.                             LH2'24|
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
	struct Map { short width, height; half uscale, vscale, uoffs, voffs; uint addr; };
public:
	RenderMaterial( const Material* source, const vector<int>& offsets );
	Map MakeMap( Material::Float3Value source, const vector<int>& offsets );
	void SetDiffuse( float3 d )
	{
		diffuse_r = float_to_half( d.x );
		diffuse_g = float_to_half( d.y );
		diffuse_b = float_to_half( d.z );
	}
	void SetTransmittance( float3 t )
	{
		transmittance_r = float_to_half( t.x );
		transmittance_g = float_to_half( t.y );
		transmittance_b = float_to_half( t.z );
	}
	half diffuse_r, diffuse_g, diffuse_b, transmittance_r, transmittance_g, transmittance_b;
	uint flags;
	uint4 parameters; // 16 Disney principled BRDF parameters, 0.8 fixed point
	Map tex0, tex1, nmap0, nmap1, smap, rmap; // total Material size: 128 bytes
};

//  +-----------------------------------------------------------------------------+
//  |  Animation                                                                  |
//  |  Animation definition.                                                LH2'24|
//  +-----------------------------------------------------------------------------+
class Animation
{
	class Sampler
	{
	public:
		enum { LINEAR = 0, SPLINE, STEP };
		Sampler( const tinygltf::AnimationSampler& gltfSampler, const tinygltf::Model& gltfModel );
		void ConvertFromGLTFSampler( const tinygltf::AnimationSampler& gltfSampler, const tinygltf::Model& gltfModel );
		float SampleFloat( float t, int k, int i, int count ) const;
		float3 SampleVec3( float t, int k ) const;
		quat SampleQuat( float t, int k ) const;
		vector<float> t;				// key frame times
		vector<float3> vec3Key;			// vec3 key frames (location or scale)
		vector<quat> vec4Key;			// vec4 key frames (rotation)
		vector<float> floatKey;			// float key frames (weight)
		int interpolation;				// interpolation type: linear, spline, step
	};
	class Channel
	{
	public:
		Channel( const tinygltf::AnimationChannel& gltfChannel, const int nodeBase );
		int samplerIdx;					// sampler used by this channel
		int nodeIdx;					// index of the node this channel affects
		int target;						// 0: translation, 1: rotation, 2: scale, 3: weights
		void Reset() { t = 0, k = 0; }
		void SetTime( const float v ) { t = v, k = 0; }
		void Update( const float t, const Sampler* sampler );	// apply this channel to the target nde for time t
		void ConvertFromGLTFChannel( const tinygltf::AnimationChannel& gltfChannel, const int nodeBase );
		// data
		float t = 0;					// animation timer
		int k = 0;						// current keyframe
	};
public:
	Animation( tinygltf::Animation& gltfAnim, tinygltf::Model& gltfModel, const int nodeBase );
	vector<Sampler*> sampler;			// animation samplers
	vector<Channel*> channel;			// animation channels
	void Reset();						// reset all channels
	void SetTime( const float t );
	void Update( const float dt );		// advance and apply all channels
	void ConvertFromGLTFAnim( tinygltf::Animation& gltfAnim, tinygltf::Model& gltfModel, const int nodeBase );
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
//  |    name, including path, as well as the mods field.                   LH2'24|
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
	Texture( const char* fileName, const uint modFlags = 0 );
	// methods
	bool Equals( const string& o, const uint m );
	void Load( const char* fileName, const uint modFlags, bool normalMap = false );
	static void sRGBtoLinear( uchar* pixels, const uint size, const uint stride );
	void BumpToNormalMap( float heightScale );
	uint* GetLDRPixels() { return (uint*)idata; }
	float4* GetHDRPixels() { return fdata; }
	// internal methods
	int PixelsNeeded( int w, int h, const int l ) const;
	void ConstructMIPmaps();
	// public properties
public:
	uint width = 0, height = 0;			// width and height in pixels
	uint MIPlevels = 1;					// number of MIPmaps
	uint ID = 0;						// unique integer ID of this texture
	string name;						// texture name, not for unique identification
	string origin;						// origin: file from which the data was loaded, with full path
	uint flags = 0;						// flags
	uint mods = 0;						// modifications to original data
	uint refCount = 1;					// the number of materials that use this texture
	uchar4* idata = nullptr;			// pointer to a 32-bit ARGB bitmap
	float4* fdata = nullptr;			// pointer to a 128-bit ARGB bitmap
	TRACKCHANGES;						// add Changed(), MarkAsDirty() methods, see system.h
};

//  +-----------------------------------------------------------------------------+
//  |  TriLight                                                                   |
//  |  Light triangle.                                                      LH2'24|
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
	float3 vertex0 = { 0 };				// vertex 0 position
	float3 vertex1 = { 0 };				// vertex 1 position
	float3 vertex2 = { 0 };				// vertex 2 position
	float3 centre = { 0 };				// barycenter of the triangle
	float3 radiance = { 0 };			// radiance per unit area
	float3 N = float3( 0, -1, 0 );		// geometric triangle normal
	float area = 0;						// triangle area
	float energy = 0;					// total radiance
	TRACKCHANGES;
};

//  +-----------------------------------------------------------------------------+
//  |  PointLight                                                                 |
//  |  Point light definition.                                              LH2'24|
//  +-----------------------------------------------------------------------------+
class PointLight
{
public:
	// constructor / destructor
	PointLight() = default;
	// data members
	float3 position = { 0 };			// position of the point light
	float3 radiance = { 0 };			// emitted radiance
	int ID = 0;							// position in Scene::pointLights
	TRACKCHANGES;
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
	float3 position = { 0 };			// position of the spot light
	float cosInner = 0;					// cosine of the inner boundary
	float3 radiance = { 0 };			// emitted radiance
	float cosOuter = 0;					// cosine of the outer boundary
	float3 direction = make_float3( 0, -1, 0 ); // spot light direction
	int ID = 0;							// position in Scene::spotLights
	TRACKCHANGES;
};

//  +-----------------------------------------------------------------------------+
//  |  DirectionalLight                                                           |
//  |  Directional light definition.                                        LH2'24|
//  +-----------------------------------------------------------------------------+
class DirectionalLight
{
public:
	// constructor / destructor
	DirectionalLight() = default;
	// data members
	float3 direction = make_float3( 0, -1, 0 );
	float3 radiance = { 0 };
	int ID = 0;
	TRACKCHANGES;
};

//  +-----------------------------------------------------------------------------+
//  |  Scene                                                                      |
//  |  Module for scene I/O and host-side management.                             |
//  |  This is a pure static class; we will not have more than one scene.   LH2'24|
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
	static int FindOrCreateTexture( const string& origin, const uint modFlags = 0 );
	static int FindTextureID( const char* name );
	static int CreateTexture( const string& origin, const uint modFlags = 0 );
	static int FindOrCreateMaterial( const string& name );
	static int FindOrCreateMaterialCopy( const int matID, const uint color );
	static int FindMaterialID( const char* name );
	static int FindMaterialIDByOrigin( const char* name );
	static int FindNextMaterialID( const char* name, const int matID );
	static int FindNode( const char* name );
	static void SetNodeTransform( const int nodeId, const mat4& transform );
	static const mat4& GetNodeTransform( const int nodeId );
	static void ResetAnimation( const int animId );
	static void UpdateAnimation( const int animId, const float dt );
	static int AnimationCount() { return (int)animations.size(); }
	// scene construction / maintenance
	static int AddMesh( Mesh* mesh );
	static int AddMesh( const char* objFile, const char* dir, const float scale = 1.0f, const bool flatShaded = false );
	static int AddMesh( const char* objFile, const float scale = 1.0f, const bool flatShaded = false );
	static int AddScene( const char* sceneFile, const mat4& transform = mat4::Identity() );
	static int AddScene( const char* sceneFile, const char* dir, const mat4& transform );
	static int AddMesh( const int triCount );
	static void AddTriToMesh( const int meshId, const float3& v0, const float3& v1, const float3& v2, const int matId );
	static int AddQuad( const float3 N, const float3 pos, const float width, const float height, const int matId, const int meshID = -1 );
	static int AddNode( Node* node );
	static int AddChildNode( const int parentNodeId, const int childNodeId );
	static int GetChildId( const int parentId, const int childIdx );
	static int AddInstance( const int nodeId );
	static void RemoveNode( const int instId );
	static int AddMaterial( Material* material );
	static int AddMaterial( const float3 color, const char* name = 0 );
	static int AddPointLight( const float3 pos, const float3 radiance );
	static int AddSpotLight( const float3 pos, const float3 direction, const float inner, const float outer, const float3 radiance );
	static int AddDirectionalLight( const float3 direction, const float3 radiance );
	// scene graph / TLAS operations
	static void UpdateSceneGraph( const float deltaTime );
	static int Intersect( Ray& ray );
	// data members
	static inline vector<int> rootNodes;						// root node indices of loaded (or instanced) objects
	static inline vector<Node*> nodePool;						// all scene nodes
	static inline vector<Mesh*> meshPool;						// all scene meshes 
	static inline vector<Skin*> skins;							// all scene skins
	static inline vector<Animation*> animations;				// all scene animations
	static inline vector<Material*> materials;					// all scene materials
	static inline vector<Texture*> textures;					// all scene textures
	static inline vector<TriLight*> triLights;					// light emitting triangles
	static inline vector<PointLight*> pointLights;				// scene point lights
	static inline vector<SpotLight*> spotLights;				// scene spot lights
	static inline vector<DirectionalLight*> directionalLights;	// scene directional lights
	static inline SkyDome* sky;									// HDR skydome
	static inline BVH tlas;										// top-level acceleration structure
#ifdef ENABLE_OPENCL_BVH
	// OpenCL buffers for transferring data from CPU to GPU
	static inline Buffer* bvhNodeData;							// tlas and blas node data
	static inline Buffer* altNodeData;							// blas node data, alternative layout
	static inline Buffer* bvh8Compacted;						// compacted 8-way bvh node data
	static inline Buffer* bvh8TriData;							// 'by-value' triangle data for 8-way bvh
	static inline Buffer* bvh4Compacted;						// compacted / interleaved 4-way bvh node data
	static inline Buffer* triangleData;							// triangle data
	static inline Buffer* fatTriData;							// full-featured triangle records 
	static inline Buffer* triangleIdxData;						// triangle index arrays
	static inline Buffer* offsetData;							// blas offsets
	static inline Buffer* transformData;						// blas transforms
	static inline Buffer* skyData = 0;							// skybox pixels
	static inline Buffer* ldrData, * hdrData;					// texture data (regular and hdr)
	static inline Buffer* materialData;							// materials 
	// methods
	static void InitializeGPUData();
	static void UpdateGPUData();
#endif
};

}