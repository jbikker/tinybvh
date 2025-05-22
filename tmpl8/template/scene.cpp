#include "precomp.h"

// access syoyo's tiny_obj and tiny_gltf
#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"
#define TINYGLTF_IMPLEMENTATION
#define TINYGLTF_NO_STB_IMAGE_WRITE
#include "tiny_gltf.h"

// 'declaration of x hides previous local declaration'
#pragma warning( disable: 4456)

// IMPLEMENTATION OF THE LIGHTHOUSE 2 SCENE GRAPH

// see scene.h for details.

//  +-----------------------------------------------------------------------------+
//  |  SkyDome::Load                                                              |
//  |  Load a skydome.                                                      LH2'24|
//  +-----------------------------------------------------------------------------+
void SkyDome::Load( const char* filename, const float3 scale )
{
	FREE64( pixels ); // just in case we're reloading
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
		pixels = (float3*)MALLOC64( width * height * sizeof( float3 ) );
		fread( pixels, 1, sizeof( float3 ) * width * height, f );
		fclose( f );
	}
	if (!pixels)
	{
		// load HDR sky
		int bpp = 0;
		float* tmp = stbi_loadf( filename, &width, &height, &bpp, 0 );
		if (!tmp) FATALERROR( "File does not exist: %s", filename );
		if (bpp == 3)
		{
			pixels = (float3*)MALLOC64( width * height * sizeof( float3 ) );
			for (int i = 0; i < width * height; i++)
				pixels[i] = float3( sqrtf( tmp[i * 3 + 0] ), sqrtf( tmp[i * 3 + 1] ), sqrtf( tmp[i * 3 + 2] ) );
		}
		else FATALERROR( "Reading a skydome with %d channels is not implemented!", bpp );
		// save skydome to binary file, .hdr is slow to load
		f = fopen( bin_name, "wb" );
		fwrite( &width, 1, 4, f );
		fwrite( &height, 1, 4, f );
		fwrite( pixels, 1, sizeof( float3 ) * width * height, f );
		fclose( f );
	}
	// cache does not include scale so we can change it later
	if (scale.x != 1.f || scale.y != 1.f || scale.z != 1.f)
		for (int p = 0; p < width * height; ++p) pixels[p] *= scale;
	// done
	MarkAsDirty();
}

//  +-----------------------------------------------------------------------------+
//  |  Skin::Skin                                                                 |
//  |  Constructor.                                                         LH2'24|
//  +-----------------------------------------------------------------------------+
Skin::Skin( const tinygltf::Skin& gltfSkin, const tinygltf::Model& gltfModel, const int nodeBase )
{
	ConvertFromGLTFSkin( gltfSkin, gltfModel, nodeBase );
}

//  +-----------------------------------------------------------------------------+
//  |  Skin::Skin                                                                 |
//  |  Constructor.                                                         LH2'24|
//  +-----------------------------------------------------------------------------+
void Skin::ConvertFromGLTFSkin( const tinygltf::Skin& gltfSkin, const tinygltf::Model& gltfModel, const int nodeBase )
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
		memcpy( inverseBindMatrices.data(), &buffer.data[accessor.byteOffset + bufferView.byteOffset], accessor.count * sizeof( mat4 ) );
		jointMat.resize( accessor.count );
		// convert gltf's column-major to row-major
		for (int k = 0; k < accessor.count; k++)
		{
			mat4 M = inverseBindMatrices[k];
			for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) inverseBindMatrices[k].cell[j * 4 + i] = M.cell[i * 4 + j];
		}
	}
}

//  +-----------------------------------------------------------------------------+
//  |  Mesh::Mesh                                                                 |
//  |  Constructors.                                                        LH2'24|
//  +-----------------------------------------------------------------------------+
Mesh::Mesh( const int triCount )
{
	triangles.resize( triCount ); // precallocate; to be used for procedural meshes.
	vertices.resize( triCount * 3 );
}

Mesh::Mesh( const char* file, const char* dir, const float scale, const bool flatShaded )
{
	LoadGeometry( file, dir, scale, flatShaded );
}

Mesh::Mesh( const tinygltf::Mesh& gltfMesh, const tinygltf::Model& gltfModel, const vector<int>& matIdx, const int materialOverride )
{
	ConvertFromGTLFMesh( gltfMesh, gltfModel, matIdx, materialOverride );
}

//  +-----------------------------------------------------------------------------+
//  |  Mesh::LoadGeometry                                                         |
//  |  Load geometry data from disk. Obj files only.                        LH2'24|
//  +-----------------------------------------------------------------------------+
void Mesh::LoadGeometry( const char* file, const char* dir, const float scale, const bool flatShaded )
{
	// process supplied file name
	mat4 T = mat4::Scale( scale ); // may include scale, translation, axis exchange
	string combined = string( dir ) + (dir[strlen( dir ) - 1] == '/' ? "" : "/") + string( file );
	for (int l = (int)combined.size(), i = 0; i < l; i++) if (combined[i] >= 'A' && combined[i] <= 'Z') combined[i] -= 'Z' - 'z';
	string extension = (combined.find_last_of( "." ) != string::npos) ? combined.substr( combined.find_last_of( "." ) + 1 ) : "";
	if (extension.compare( "obj" ) != 0) FATALERROR( "unsupported extension in file %s", combined.c_str() );
	LoadGeometryFromOBJ( combined.c_str(), dir, T, flatShaded );
}

//  +-----------------------------------------------------------------------------+
//  |  Mesh::LoadGeometryFromObj                                                  |
//  |  Load an obj file using tinyobj.                                      LH2'24|
//  +-----------------------------------------------------------------------------+
void Mesh::LoadGeometryFromOBJ( const string& fileName, const char* directory, const mat4& T, const bool flatShaded )
{
	// load obj file
	tinyobj::attrib_t attrib;
	vector<tinyobj::shape_t> shapes;
	vector<tinyobj::material_t> materials;
	map<string, GLuint> textures;
	string err, warn;
	tinyobj::LoadObj( &attrib, &shapes, &materials, &err, &warn, fileName.c_str(), directory );
	FATALERROR_IF( err.size() > 0 || shapes.size() == 0, "tinyobj failed to load %s: %s", fileName.c_str(), err.c_str() );
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
		material->origin = fileName;
		material->ConvertFrom( mtl );
		material->flags |= Material::FROM_MTL;
		material->MarkAsDirty();
		Scene::materials.push_back( material );
		materialList.push_back( material->ID );
	}
	chdir( currDir );
	// calculate values for consistent normal interpolation
	const uint verts = (uint)attrib.normals.size() / 3;
	vector<float> alphas;
	alphas.resize( verts, 1.0f ); // we will have one alpha value per unique vertex normal
	for (uint s = (uint)shapes.size(), i = 0; i < s; i++)
	{
		vector<tinyobj::index_t>& indices = shapes[i].mesh.indices;
		if (flatShaded) for (uint s = (uint)indices.size(), f = 0; f < s; f++) alphas[indices[f].normal_index] = 1; else
		{
			for (uint s = (uint)indices.size(), f = 0; f < s; f += 3)
			{
				const int idx0 = indices[f + 0].vertex_index, nidx0 = indices[f + 0].normal_index;
				const int idx1 = indices[f + 1].vertex_index, nidx1 = indices[f + 1].normal_index;
				const int idx2 = indices[f + 2].vertex_index, nidx2 = indices[f + 2].normal_index;
				const float3 vert0 = make_float3( attrib.vertices[idx0 * 3 + 0], attrib.vertices[idx0 * 3 + 1], attrib.vertices[idx0 * 3 + 2] );
				const float3 vert1 = make_float3( attrib.vertices[idx1 * 3 + 0], attrib.vertices[idx1 * 3 + 1], attrib.vertices[idx1 * 3 + 2] );
				const float3 vert2 = make_float3( attrib.vertices[idx2 * 3 + 0], attrib.vertices[idx2 * 3 + 1], attrib.vertices[idx2 * 3 + 2] );
				float3 N = normalize( cross( vert1 - vert0, vert2 - vert0 ) );
				float3 vN0, vN1, vN2;
				if (nidx0 > -1)
				{
					vN0 = make_float3( attrib.normals[nidx0 * 3 + 0], attrib.normals[nidx0 * 3 + 1], attrib.normals[nidx0 * 3 + 2] );
					vN1 = make_float3( attrib.normals[nidx1 * 3 + 0], attrib.normals[nidx1 * 3 + 1], attrib.normals[nidx1 * 3 + 2] );
					vN2 = make_float3( attrib.normals[nidx2 * 3 + 0], attrib.normals[nidx2 * 3 + 1], attrib.normals[nidx2 * 3 + 2] );
					if (dot( N, vN0 ) < 0 && dot( N, vN1 ) < 0 && dot( N, vN2 ) < 0) N *= -1.0f; // flip if not consistent with vertex normals
					alphas[nidx0] = min( alphas[nidx0], max( 0.7f, dot( vN0, N ) ) );
					alphas[nidx1] = min( alphas[nidx1], max( 0.7f, dot( vN1, N ) ) );
					alphas[nidx2] = min( alphas[nidx2], max( 0.7f, dot( vN2, N ) ) );
				}
				else vN0 = vN1 = vN2 = N;
			}
		}
	}
	// finalize alpha values based on max dots
	const float w = 0.03632f;
	for (uint i = 0; i < verts; i++)
	{
		const float nnv = alphas[i]; // temporarily stored there
		alphas[i] = acosf( nnv ) * (1 + w * (1 - nnv) * (1 - nnv));
	}
	// extract data for ray tracing: raw vertex and index data
	aabb sceneBounds;
	int toReserve = 0;
	for (auto& shape : shapes) toReserve += (int)shape.mesh.indices.size();
	vertices.reserve( toReserve );
	for (auto& shape : shapes) for (int f = 0; f < shape.mesh.indices.size(); f += 3)
	{
		const uint idx0 = shape.mesh.indices[f + 0].vertex_index;
		const uint idx1 = shape.mesh.indices[f + 1].vertex_index;
		const uint idx2 = shape.mesh.indices[f + 2].vertex_index;
		const float3 v0 = make_float3( attrib.vertices[idx0 * 3 + 0], attrib.vertices[idx0 * 3 + 1], attrib.vertices[idx0 * 3 + 2] );
		const float3 v1 = make_float3( attrib.vertices[idx1 * 3 + 0], attrib.vertices[idx1 * 3 + 1], attrib.vertices[idx1 * 3 + 2] );
		const float3 v2 = make_float3( attrib.vertices[idx2 * 3 + 0], attrib.vertices[idx2 * 3 + 1], attrib.vertices[idx2 * 3 + 2] );
		const float4 tv0 = make_float4( v0, 1 ) * T;
		const float4 tv1 = make_float4( v1, 1 ) * T;
		const float4 tv2 = make_float4( v2, 1 ) * T;
		vertices.push_back( tv0 );
		vertices.push_back( tv1 );
		vertices.push_back( tv2 );
		sceneBounds.Grow( make_float3( tv0 ) );
		sceneBounds.Grow( make_float3( tv1 ) );
		sceneBounds.Grow( make_float3( tv2 ) );
	}
	// extract full model data and materials
	triangles.resize( vertices.size() / 3 );
	for (int s = (int)shapes.size(), face = 0, i = 0; i < s; i++)
	{
		vector<tinyobj::index_t>& indices = shapes[i].mesh.indices;
		for (int s = (int)shapes[i].mesh.indices.size(), f = 0; f < s; f += 3, face++)
		{
			FatTri& tri = triangles[face];
			tri.vertex0 = make_float3( vertices[face * 3 + 0] );
			tri.vertex1 = make_float3( vertices[face * 3 + 1] );
			tri.vertex2 = make_float3( vertices[face * 3 + 2] );
			const int tidx0 = indices[f + 0].texcoord_index, nidx0 = indices[f + 0].normal_index; // , idx0 = indices[f + 0].vertex_index;
			const int tidx1 = indices[f + 1].texcoord_index, nidx1 = indices[f + 1].normal_index; // , idx1 = indices[f + 1].vertex_index;
			const int tidx2 = indices[f + 2].texcoord_index, nidx2 = indices[f + 2].normal_index; // , idx2 = indices[f + 2].vertex_index;
			const float3 e1 = tri.vertex1 - tri.vertex0;
			const float3 e2 = tri.vertex2 - tri.vertex0;
			float3 N = normalize( cross( e1, e2 ) );
			if (nidx0 > -1)
			{
				tri.vN0 = make_float3( attrib.normals[nidx0 * 3 + 0], attrib.normals[nidx0 * 3 + 1], attrib.normals[nidx0 * 3 + 2] );
				tri.vN1 = make_float3( attrib.normals[nidx1 * 3 + 0], attrib.normals[nidx1 * 3 + 1], attrib.normals[nidx1 * 3 + 2] );
				tri.vN2 = make_float3( attrib.normals[nidx2 * 3 + 0], attrib.normals[nidx2 * 3 + 1], attrib.normals[nidx2 * 3 + 2] );
				if (dot( N, tri.vN0 ) < 0) N *= -1.0f; // flip face normal if not consistent with vertex normal
			}
			else tri.vN0 = tri.vN1 = tri.vN2 = N;
			if (flatShaded) tri.vN0 = tri.vN1 = tri.vN2 = N;
			if (tidx0 > -1)
			{
				tri.u0 = attrib.texcoords[tidx0 * 2 + 0], tri.v0 = attrib.texcoords[tidx0 * 2 + 1];
				tri.u1 = attrib.texcoords[tidx1 * 2 + 0], tri.v1 = attrib.texcoords[tidx1 * 2 + 1];
				tri.u2 = attrib.texcoords[tidx2 * 2 + 0], tri.v2 = attrib.texcoords[tidx2 * 2 + 1];
				// calculate tangent vectors
				float2 uv01 = make_float2( tri.u1 - tri.u0, tri.v1 - tri.v0 );
				float2 uv02 = make_float2( tri.u2 - tri.u0, tri.v2 - tri.v0 );
				if (dot( uv01, uv01 ) == 0 || dot( uv02, uv02 ) == 0)
					tri.T = normalize( tri.vertex1 - tri.vertex0 ),
					tri.B = normalize( cross( N, tri.T ) );
				else
					tri.T = normalize( e1 * uv02.y - e2 * uv01.y ),
					tri.B = normalize( e2 * uv01.x - e1 * uv02.x );
			}
			else
				tri.T = normalize( e1 ),
				tri.B = normalize( cross( N, tri.T ) );
			tri.Nx = N.x, tri.Ny = N.y, tri.Nz = N.z;
			tri.material = shapes[i].mesh.material_ids[f / 3] + matIdxOffset;
			tri.area = 0; // we don't actually use it, except for lights, where it is also calculated
			tri.invArea = 0; // todo
			if (nidx0 > -1)
				tri.alpha = make_float3( alphas[nidx0], tri.alpha.y = alphas[nidx1], tri.alpha.z = alphas[nidx2] );
			else
				tri.alpha = make_float3( 0 );
			// calculate triangle LOD data
			if (tri.material < Scene::materials.size())
			{
				Material* mat = Scene::materials[tri.material];
				int textureID = mat->color.textureID;
				if (textureID > -1)
				{
					Texture* texture = Scene::textures[textureID];
					float Ta = (float)(texture->width * texture->height) * fabs( (tri.u1 - tri.u0) * (tri.v2 - tri.v0) - (tri.u2 - tri.u0) * (tri.v1 - tri.v0) );
					float Pa = length( cross( tri.vertex1 - tri.vertex0, tri.vertex2 - tri.vertex0 ) );
					tri.LOD = 0.5f * log2f( Ta / Pa );
				}
			}
		}
	}
}

//  +-----------------------------------------------------------------------------+
//  |  Mesh::ConvertFromGTLFMesh                                                  |
//  |  Convert a gltf mesh to a Mesh.                                       LH2'24|
//  +-----------------------------------------------------------------------------+
void Mesh::ConvertFromGTLFMesh( const tinygltf::Mesh& gltfMesh, const tinygltf::Model& gltfModel, const vector<int>& matIdx, const int materialOverride )
{
	const int targetCount = (int)gltfMesh.weights.size();
	for (auto& prim : gltfMesh.primitives)
	{
		// load indices
		const tinygltf::Accessor& accessor = gltfModel.accessors[prim.indices];
		const tinygltf::BufferView& view = gltfModel.bufferViews[accessor.bufferView];
		const tinygltf::Buffer& buffer = gltfModel.buffers[view.buffer];
		const uchar* a /* brevity */ = buffer.data.data() + view.byteOffset + accessor.byteOffset;
		const int byteStride = accessor.ByteStride( view );
		const size_t count = accessor.count;
		// allocate the index array in the pointer-to-base declared in the parent scope
		vector<int> tmpIndices;
		vector<float3> tmpNormals, tmpVertices;
		vector<float2> tmpUvs, tmpUv2s /* texture layer 2 */;
		vector<uint4> tmpJoints;
		vector<float4> tmpWeights, tmpTs;
		switch (accessor.componentType)
		{
		case TINYGLTF_COMPONENT_TYPE_BYTE: for (int k = 0; k < count; k++, a += byteStride) tmpIndices.push_back( *((char*)a) ); break;
		case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE: for (int k = 0; k < count; k++, a += byteStride) tmpIndices.push_back( *((uchar*)a) ); break;
		case TINYGLTF_COMPONENT_TYPE_SHORT: for (int k = 0; k < count; k++, a += byteStride) tmpIndices.push_back( *((short*)a) ); break;
		case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT: for (int k = 0; k < count; k++, a += byteStride) tmpIndices.push_back( *((ushort*)a) ); break;
		case TINYGLTF_COMPONENT_TYPE_INT: for (int k = 0; k < count; k++, a += byteStride) tmpIndices.push_back( *((int*)a) ); break;
		case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT: for (int k = 0; k < count; k++, a += byteStride) tmpIndices.push_back( *((uint*)a) ); break;
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
			const tinygltf::Accessor attribAccessor = gltfModel.accessors[attribute.second];
			const tinygltf::BufferView& bufferView = gltfModel.bufferViews[attribAccessor.bufferView];
			const tinygltf::Buffer& buffer = gltfModel.buffers[bufferView.buffer];
			const uchar* a = buffer.data.data() + bufferView.byteOffset + attribAccessor.byteOffset;
			const int byte_stride = attribAccessor.ByteStride( bufferView );
			const size_t count = attribAccessor.count;
			if (attribute.first == "POSITION")
			{
				float3 boundsMin = make_float3( (float)attribAccessor.minValues[0], (float)attribAccessor.minValues[1], (float)attribAccessor.minValues[2] );
				float3 boundsMax = make_float3( (float)attribAccessor.maxValues[0], (float)attribAccessor.maxValues[1], (float)attribAccessor.maxValues[2] );
				if (attribAccessor.type == TINYGLTF_TYPE_VEC3)
					if (attribAccessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT)
						for (size_t i = 0; i < count; i++, a += byte_stride) tmpVertices.push_back( *((float3*)a) );
					else FATALERROR( "double precision positions not supported in gltf file" );
				else FATALERROR( "unsupported position definition in gltf file" );
			}
			else if (attribute.first == "NORMAL")
			{
				if (attribAccessor.type == TINYGLTF_TYPE_VEC3)
					if (attribAccessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT)
						for (size_t i = 0; i < count; i++, a += byte_stride) tmpNormals.push_back( *((float3*)a) );
					else FATALERROR( "double precision normals not supported in gltf file" );
				else FATALERROR( "expected vec3 normals in gltf file" );
			}
			else if (attribute.first == "TANGENT")
			{
				/* if (attribAccessor.type == TINYGLTF_TYPE_VEC4)
					if (attribAccessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT)
						for (size_t i = 0; i < count; i++, a += byte_stride) tmpTs.push_back( *((float4*)a) );
					else FATALERROR( "double precision tangents not supported in gltf file" );
				else FATALERROR( "expected vec4 uvs in gltf file" ); */ // TODO: Causing crashes atm...
			}
			else if (attribute.first == "TEXCOORD_0")
			{
				if (attribAccessor.type == TINYGLTF_TYPE_VEC2)
					if (attribAccessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT)
						for (size_t i = 0; i < count; i++, a += byte_stride) tmpUvs.push_back( *((float2*)a) );
					else FATALERROR( "double precision uvs not supported in gltf file" );
				else FATALERROR( "expected vec2 uvs in gltf file" );
			}
			else if (attribute.first == "TEXCOORD_1")
			{
				if (attribAccessor.type == TINYGLTF_TYPE_VEC2)
					if (attribAccessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT)
						for (size_t i = 0; i < count; i++, a += byte_stride) tmpUv2s.push_back( *((float2*)a) );
					else FATALERROR( "double precision uvs not supported in gltf file" );
				else FATALERROR( "expected vec2 uvs in gltf file" );
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
							tmpJoints.push_back( make_uint4( *((ushort*)a), *((ushort*)(a + 2)), *((ushort*)(a + 4)), *((ushort*)(a + 6)) ) );
					else if (attribAccessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE)
						for (size_t i = 0; i < count; i++, a += byte_stride)
							tmpJoints.push_back( make_uint4( *((uchar*)a), *((uchar*)(a + 1)), *((uchar*)(a + 2)), *((uchar*)(a + 3)) ) );
					else FATALERROR( "expected ushorts or uchars for joints in gltf file" );
				else FATALERROR( "expected vec4s for joints in gltf file" );
			}
			else if (attribute.first == "WEIGHTS_0")
			{
				if (attribAccessor.type == TINYGLTF_TYPE_VEC4)
					if (attribAccessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT)
						for (size_t i = 0; i < count; i++, a += byte_stride)
						{
							float4 w4;
							memcpy( &w4, a, sizeof( float4 ) );
							float norm = 1.0f / (w4.x + w4.y + w4.z + w4.w);
							w4 *= norm;
							tmpWeights.push_back( w4 );
						}
					else FATALERROR( "double precision uvs not supported in gltf file" );
				else FATALERROR( "expected vec4 weights in gltf file" );
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
				tmpPoses[0].tangents.push_back( make_float3( 0 ) /* TODO */ );
			}
		}
		for (int i = 0; i < targetCount; i++)
		{
			tmpPoses.push_back( Pose() );
			for (const auto& target : prim.targets[i])
			{
				const tinygltf::Accessor accessor = gltfModel.accessors[target.second];
				const tinygltf::BufferView& view = gltfModel.bufferViews[accessor.bufferView];
				const float* a = (const float*)(gltfModel.buffers[view.buffer].data.data() + view.byteOffset + accessor.byteOffset);
				for (int j = 0; j < accessor.count; j++)
				{
					float3 v = make_float3( a[j * 3], a[j * 3 + 1], a[j * 3 + 2] );
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
//  |  data, which we now convert to the final representation.              LH2'24|
//  +-----------------------------------------------------------------------------+
void Mesh::BuildFromIndexedData( const vector<int>& tmpIndices, const vector<float3>& tmpVertices,
	const vector<float3>& tmpNormals, const vector<float2>& tmpUvs, const vector<float2>& tmpUv2s,
	const vector<float4>& /* tmpTs */, const vector<Pose>& tmpPoses,
	const vector<uint4>& tmpJoints, const vector<float4>& tmpWeights, const int materialIdx )
{
	// calculate values for consistent normal interpolation
	vector<float> tmpAlphas;
	tmpAlphas.resize( tmpVertices.size(), 1.0f ); // we will have one alpha value per unique vertex
	for (size_t s = tmpIndices.size(), i = 0; i < s; i += 3)
	{
		const uint v0idx = tmpIndices[i + 0], v1idx = tmpIndices[i + 1], v2idx = tmpIndices[i + 2];
		const float3 vert0 = tmpVertices[v0idx], vert1 = tmpVertices[v1idx], vert2 = tmpVertices[v2idx];
		float3 N = normalize( cross( vert1 - vert0, vert2 - vert0 ) );
		float3 vN0, vN1, vN2;
		if (tmpNormals.size() > 0)
		{
			vN0 = tmpNormals[v0idx], vN1 = tmpNormals[v1idx], vN2 = tmpNormals[v2idx];
			if (dot( N, vN0 ) < 0 && dot( N, vN1 ) < 0 && dot( N, vN2 ) < 0) N *= -1.0f; // flip if not consistent with vertex normals
		}
		else
		{
			// no normals supplied; copy face normal
			vN0 = vN1 = vN2 = N;
		}
		// Note: we clamp at approx. 45 degree angles; beyond this the approach fails.
		tmpAlphas[v0idx] = min( tmpAlphas[v0idx], dot( vN0, N ) );
		tmpAlphas[v1idx] = min( tmpAlphas[v1idx], dot( vN1, N ) );
		tmpAlphas[v2idx] = min( tmpAlphas[v2idx], dot( vN2, N ) );
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
		const uint v0idx = tmpIndices[i * 3 + 0];
		const uint v1idx = tmpIndices[i * 3 + 1];
		const uint v2idx = tmpIndices[i * 3 + 2];
		const float3 v0pos = tmpVertices[v0idx];
		const float3 v1pos = tmpVertices[v1idx];
		const float3 v2pos = tmpVertices[v2idx];
		vertices.push_back( make_float4( v0pos, 1 ) );
		vertices.push_back( make_float4( v1pos, 1 ) );
		vertices.push_back( make_float4( v2pos, 1 ) );
		const float3 N = normalize( cross( v1pos - v0pos, v2pos - v0pos ) );
		tri.Nx = N.x, tri.Ny = N.y, tri.Nz = N.z;
		tri.vertex0 = tmpVertices[v0idx];
		tri.vertex1 = tmpVertices[v1idx];
		tri.vertex2 = tmpVertices[v2idx];
		tri.alpha = make_float3( tmpAlphas[v0idx], tmpAlphas[v1idx], tmpAlphas[v2idx] );
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
					uint u = (uint)(tri.u0 * texture->width) % texture->width;
					uint v = (uint)(tri.v0 * texture->height) % texture->height;
					uint texel = ((uint*)texture->idata)[u + v * texture->width] & 0xffffff;
					tri.material = Scene::FindOrCreateMaterialCopy( materialIdx, texel );
				}
			}
			// calculate tangent vector based on uvs
			float2 uv01 = make_float2( tri.u1 - tri.u0, tri.v1 - tri.v0 );
			float2 uv02 = make_float2( tri.u2 - tri.u0, tri.v2 - tri.v0 );
			if (dot( uv01, uv01 ) == 0 || dot( uv02, uv02 ) == 0)
			{
			#if 1
				// PBRT:
				// https://github.com/mmp/pbrt-v3/blob/3f94503ae1777cd6d67a7788e06d67224a525ff4/src/shapes/triangle.cpp#L381
				if (std::abs( N.x ) > std::abs( N.y ))
					tri.T = make_float3( -N.z, 0, N.x ) / std::sqrt( N.x * N.x + N.z * N.z );
				else
					tri.T = make_float3( 0, N.z, -N.y ) / std::sqrt( N.y * N.y + N.z * N.z );
			#else
				tri.T = normalize( tri.vertex1 - tri.vertex0 );
			#endif
				tri.B = normalize( cross( N, tri.T ) );
			}
			else
			{
				tri.T = normalize( (tri.vertex1 - tri.vertex0) * uv02.y - (tri.vertex2 - tri.vertex0) * uv01.y );
				tri.B = normalize( (tri.vertex2 - tri.vertex0) * uv01.x - (tri.vertex1 - tri.vertex0) * uv02.x );
			}
			// catch bad tangents
			if (isnan( tri.T.x + tri.T.y + tri.T.z + tri.B.x + tri.B.y + tri.B.z ))
			{
				tri.T = normalize( tri.vertex1 - tri.vertex0 );
				tri.B = normalize( cross( N, tri.T ) );
			}
		}
		else
		{
			// no uv information; use edges to calculate tangent vectors
			tri.T = normalize( tri.vertex1 - tri.vertex0 );
			tri.B = normalize( cross( N, tri.T ) );
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
				poses[i].tangents.push_back( make_float3( 0, 1, 0 ) );
				poses[i].tangents.push_back( make_float3( 0, 1, 0 ) );
				poses[i].tangents.push_back( make_float3( 0, 1, 0 ) );
			}
		}
	}
}

//  +-----------------------------------------------------------------------------+
//  |  Mesh::BuildMaterialList                                                    |
//  |  Update the list of materials used by this mesh. We will use this list to   |
//  |  efficiently find meshes using a specific material, which in turn is useful |
//  |  when a material becomes emissive or non-emissive.                    LH2'24|
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
//  |  Mesh::SetPose                                                              |
//  |  Update the geometry data in this mesh using the weights from the node,     |
//  |  and update all dependent data.                                       LH2'24|
//  +-----------------------------------------------------------------------------+
void Mesh::SetPose( const vector<float>& w )
{
	assert( w.size() == poses.size() - 1 /* first pose is base pose */ );
	const int weightCount = (int)w.size();
	// adjust intersection geometry data
	for (int s = (int)vertices.size(), i = 0; i < s; i++)
	{
		vertices[i] = make_float4( poses[0].positions[i], 1 );
		for (int j = 1; j <= weightCount; j++) vertices[i] += w[j - 1] * make_float4( poses[j].positions[i], 0 );
	}
	// adjust full triangles
	for (int s = (int)triangles.size(), i = 0; i < s; i++)
	{
		triangles[i].vertex0 = make_float3( vertices[i * 3 + 0] );
		triangles[i].vertex1 = make_float3( vertices[i * 3 + 1] );
		triangles[i].vertex2 = make_float3( vertices[i * 3 + 2] );
		triangles[i].vN0 = poses[0].normals[i * 3 + 0];
		triangles[i].vN1 = poses[0].normals[i * 3 + 1];
		triangles[i].vN2 = poses[0].normals[i * 3 + 2];
		for (int j = 1; j <= weightCount; j++)
			triangles[i].vN0 += poses[j].normals[i * 3 + 0],
			triangles[i].vN1 += poses[j].normals[i * 3 + 1],
			triangles[i].vN2 += poses[j].normals[i * 3 + 2];
		triangles[i].vN0 = normalize( triangles[i].vN0 );
		triangles[i].vN1 = normalize( triangles[i].vN1 );
		triangles[i].vN2 = normalize( triangles[i].vN2 );
	}
	// mark as dirty; changing vector contents doesn't trigger this
	MarkAsDirty();
}

//  +-----------------------------------------------------------------------------+
//  |  Mesh::SetPose                                                              |
//  |  Update the geometry data in this mesh using a skin.                        |
//  |  Called from RenderSystem::UpdateSceneGraph, for skinned mesh nodes.  LH2'24|
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
		uint4 j4 = joints[i];
		float4 w4 = weights[i];
		mat4 skinMatrix = w4.x * skin->jointMat[j4.x];
		skinMatrix += w4.y * skin->jointMat[j4.y];
		skinMatrix += w4.z * skin->jointMat[j4.z];
		skinMatrix += w4.w * skin->jointMat[j4.w];
		vertices[i] = skinMatrix * original[i];
		vertexNormals[i] = normalize( make_float3( make_float4( origNormal[i], 0 ) * skinMatrix ) );
	}
	// adjust full triangles
	for (int s = (int)triangles.size(), i = 0; i < s; i++)
	{
		triangles[i].vertex0 = make_float3( vertices[i * 3 + 0] );
		triangles[i].vertex1 = make_float3( vertices[i * 3 + 1] );
		triangles[i].vertex2 = make_float3( vertices[i * 3 + 2] );
		float3 N = normalize( cross( triangles[i].vertex1 - triangles[i].vertex0, triangles[i].vertex2 - triangles[i].vertex0 ) );
		triangles[i].vN0 = vertexNormals[i * 3 + 0];
		triangles[i].vN1 = vertexNormals[i * 3 + 1];
		triangles[i].vN2 = vertexNormals[i * 3 + 2];
		triangles[i].Nx = N.x;
		triangles[i].Ny = N.y;
		triangles[i].Nz = N.z;
	}
	// mark as dirty; changing vector contents doesn't trigger this
	MarkAsDirty();
}

// helper function
static FatTri TransformedFatTri( FatTri* tri, mat4 T )
{
	FatTri transformedTri = *tri;
	transformedTri.vertex0 = make_float3( make_float4( transformedTri.vertex0, 1 ) * T );
	transformedTri.vertex1 = make_float3( make_float4( transformedTri.vertex1, 1 ) * T );
	transformedTri.vertex2 = make_float3( make_float4( transformedTri.vertex2, 1 ) * T );
	const float4 N = normalize( make_float4( transformedTri.Nx, transformedTri.Ny, transformedTri.Nz, 0 ) * T );
	transformedTri.Nx = N.x;
	transformedTri.Ny = N.y;
	transformedTri.Nz = N.z;
	return transformedTri;
}

//  +-----------------------------------------------------------------------------+
//  |  Mesh::UpdateBVH()                                                          |
//  |  Create or update the mesh BVH.                                       LH2'24|
//  +-----------------------------------------------------------------------------+
void Mesh::UpdateBVH()
{
	if (!bvh)
	{
		bvh = new BVH();
	#if 0
		bvh->Build( this );
		// calculate mesh bounds
		int count = bvh->triCount;
		float3 bmin( 1e30f ), bmax( -1e30f );
		for (int i = 0; i < count * 3; i++)
			bmin = fminf( bmin, (float3)bvh->tris[i] ),
			bmax = fmaxf( bmax, (float3)bvh->tris[i] );
		float3 e = bmax - bmin;
		float emax = fmaxf( fmaxf( e.x, e.y ), e.z ), scale = 10.0f / emax;
		// scale mesh data
		float4* tmp = new float4[bvh->triCount * 3];
		for (int i = 0; i < count * 3; i++)
		{
			tmp[i] = bvh->tris[i] * scale;
			tmp[i] = float4( tmp[i].x, tmp[i].y, tmp[i].z, 0 );
		}
		// encode triangle color
		for (int i = 0; i < count; i++)
		{
			unsigned col0, col1, col2, col3;
			col0 = col1 = col2 = col3 = 0xffffff;
			FatTri& ft = triangles[i];
			unsigned matId = ft.material;
			Material* mat = 0;
			if (matId < Scene::materials.size()) mat = Scene::materials[matId];
			float2 uv0( ft.u0, ft.v0 );
			float2 uv1( ft.u1, ft.v1 );
			float2 uv2( ft.u2, ft.v2 );
			float2 uv3 = (uv0 + uv1 + uv2) * 0.33333f;
			if (mat)
			{
				if (mat->color.textureID == -1)
				{
					// no texture
					int r = (int)(mat->color.value.x * 255);
					int g = (int)(mat->color.value.y * 255);
					int b = (int)(mat->color.value.z * 255);
					col0 = col1 = col2 = col3 = (r << 16) + (g << 8) + b;
				}
				else
				{
					Texture* texture = Scene::textures[mat->color.textureID];
					int iu0 = ((int)(texture->width * uv0.x) + texture->width * 10000) % texture->width;
					int iu1 = ((int)(texture->width * uv1.x) + texture->width * 10000) % texture->width;
					int iu2 = ((int)(texture->width * uv2.x) + texture->width * 10000) % texture->width;
					int iu3 = ((int)(texture->width * uv3.x) + texture->width * 10000) % texture->width;
					int iv0 = ((int)(texture->height * uv0.y) + texture->height * 10000) % texture->height;
					int iv1 = ((int)(texture->height * uv1.y) + texture->height * 10000) % texture->height;
					int iv2 = ((int)(texture->height * uv2.y) + texture->height * 10000) % texture->height;
					int iv3 = ((int)(texture->height * uv3.y) + texture->height * 10000) % texture->height;
					col0 = *(uint*)&texture->idata[iu0 + iv0 * texture->height] & 0xffffff;
					col1 = *(uint*)&texture->idata[iu1 + iv1 * texture->height] & 0xffffff;
					col2 = *(uint*)&texture->idata[iu2 + iv2 * texture->height] & 0xffffff;
					col3 = *(uint*)&texture->idata[iu3 + iv3 * texture->height] & 0xffffff;
				}
			}
			// unsigned col = (col0 >> 2) + (col1 >> 2) + (col2 >> 2) + (col3 >> 2);
			tmp[i * 3 + 0].w = *(float*)&col3;
			tmp[i * 3 + 1].w = *(float*)&col3;
			tmp[i * 3 + 2].w = *(float*)&col3;
		}
		// save scaled mesh data
		FILE* f = fopen( "head.bin", "wb" );
		fwrite( &count, 1, 4, f );
		fwrite( tmp, count * 3 * sizeof( float4 ), 1, f );
		fclose( f );
		exit( 0 );
	#endif
	#if 1
		if (!bvh->LoadBVH( "bvhdump.bin" ))
		{
			bvh->BuildHQ( this );
			bvh->Verbose();
			int nodeCount = bvh->VerboseNodeCount();
			float sahCostBefore = bvh->SAHCost();
			for (int i = 0; i < 1000000; i++) bvh->Optimize();
			float sahCostAfter = bvh->SAHCostVerbose();
			bvh->UnVerbose();
			bvh->GPUFormatBVH2();
			float cptrs = bvh->Collapse8(); // avg. over nodes: 4.25, during traversal: 7.77
			bvh->Collapse4();
			bvh->GPUFormatBVH4();
			float sah8Cost = bvh->SAHCost8();
			printf( "BVH for %i tris: %i nodes, SAH = %.2f, optimized: %.2f, 8-wide: %.2f (%.2f ptrs)\n", bvh->triCount, nodeCount, sahCostBefore, sahCostAfter, sah8Cost, cptrs );
			bvh->SaveBVH( "bvhdump.bin" );
		}
		bvh->GPU8WayCompressed();
	#else
		float best = 1e30f;
		while (1)
		{
			Timer t;
			bvh->Build( this );
			float tm = t.elapsed() * 1000;
			if (tm < best) best = tm;
		#if 0
			int nodeCount = bvh->NodeCount();
			float sahCost = bvh->SAHCost();
			printf( "build time for %i tris: %5.1fms (best: %5.1f) for %i nodes; SAH = %.2f\n", bvh->triCount, tm, best, nodeCount, sahCost );
		#else
			bvh->Verbose();
			bvh->RefitVerbose();
			while (1)
			{
				for (int i = 0; i < 100; i++) bvh->Optimize();
				int nodeCount = bvh->VerboseNodeCount();
				float sahCost = bvh->SAHCostVerbose();
				printf( "build time for %i tris: %5.1fms (best: %5.1f) for %i nodes; SAH = %.2f\n", bvh->triCount, tm, best, nodeCount, sahCost );
			}
			while (1) {}
		#endif
		}
	#endif
	}
	else
	{
		// bvh->BuildAVX( this );
		// bvh->Refit(); // that will have to do for now.
	}
}

//  +-----------------------------------------------------------------------------+
//  |  Mesh::Intersect()                                                          |
//  |  Intersect the (transformed) mesh BVH with a ray.                     LH2'24|
//  +-----------------------------------------------------------------------------+
int Mesh::Intersect( Ray& ray )
{
	// backup ray and transform original
	Ray backupRay = ray;
	ray.O = TransformPosition( ray.O, invTransform );
	ray.D = TransformVector( ray.D, invTransform );
	ray.rD = float3( safercp( ray.D.x ), safercp( ray.D.y ), safercp( ray.D.z ) );
	// trace ray through BVH
	uint steps = bvh->altNode ? bvh->IntersectAltNodes( ray ) : bvh->Intersect( ray );
	// restore ray origin and direction
	backupRay.hit = ray.hit;
	ray = backupRay;
	return steps;
}

//  +-----------------------------------------------------------------------------+
//  |  Mesh::UpdateWorldBounds()                                                  |
//  |  Update world-space bounds over the transformed AABB.                 LH2'24|
//  +-----------------------------------------------------------------------------+
void Mesh::UpdateWorldBounds()
{
	worldBounds.Reset();
	float3 bmin = bvh->bvhNode[0].aabbMin, bmax = bvh->bvhNode[0].aabbMax;
	for (int i = 0; i < 8; i++)
	{
		float3 corner( i & 1 ? bmax.x : bmin.x, i & 2 ? bmax.y : bmin.y, i & 4 ? bmax.z : bmin.z );
		worldBounds.Grow( TransformPosition( corner, transform ) );
	}
}

//  +-----------------------------------------------------------------------------+
//  |  Node::Node                                                                 |
//  |  Constructors.                                                        LH2'24|
//  +-----------------------------------------------------------------------------+
Node::Node( const tinygltf::Node& gltfNode, const int nodeBase, const int meshBase, const int skinBase )
{
	ConvertFromGLTFNode( gltfNode, nodeBase, meshBase, skinBase );
}

Node::Node( const int meshIdx, const mat4& transform )
{
	// setup a node based on a mesh index and a transform
	meshID = meshIdx;
	localTransform = transform;
	// process light emitting surfaces
	PrepareLights();
}

//  +-----------------------------------------------------------------------------+
//  |  Node::~Node                                                                |
//  |  Destructor.                                                          LH2'24|
//  +-----------------------------------------------------------------------------+
Node::~Node()
{
	if ((meshID > -1) && hasLights)
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
//  |  Create a node from a GLTF node.                                      LH2'24|
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
		translation = make_float3( (float)gltfNode.translation[0], (float)gltfNode.translation[1], (float)gltfNode.translation[2] );
		buildFromTRS = true;
	}
	if (gltfNode.rotation.size() == 4)
	{
		// the GLTF node contains a rotation
		rotation = quat( (float)gltfNode.rotation[3], (float)gltfNode.rotation[0], (float)gltfNode.rotation[1], (float)gltfNode.rotation[2] );
		buildFromTRS = true;
	}
	if (gltfNode.scale.size() == 3)
	{
		// the GLTF node contains a scale
		scale = make_float3( (float)gltfNode.scale[0], (float)gltfNode.scale[1], (float)gltfNode.scale[2] );
		buildFromTRS = true;
	}
	// if we got T, R and/or S, reconstruct final matrix
	if (buildFromTRS) UpdateTransformFromTRS();
	// process light emitting surfaces
	PrepareLights();
}

//  +-----------------------------------------------------------------------------+
//  |  Node::UpdateTransformFromTRS                                               |
//  |  Process T, R, S data to localTransform.                              LH2'24|
//  +-----------------------------------------------------------------------------+
void Node::UpdateTransformFromTRS()
{
	mat4 T = mat4::Translate( translation );
	mat4 R = rotation.toMatrix();
	mat4 S = mat4::Scale( scale );
	localTransform = T * R * S * matrix;
}

//  +-----------------------------------------------------------------------------+
//  |  Node::Update                                                               |
//  |  Calculates the combined transform for this node and recurses into the      |
//  |  child nodes. If a change is detected, the light triangles are updated      |
//  |  as well.                                                             LH2'24|
//  +-----------------------------------------------------------------------------+
void Node::Update( const mat4& T )
{
	if (transformed /* true if node was affected by animation channel */)
	{
		UpdateTransformFromTRS();
		transformed = false;
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
		mesh->transform = combinedTransform;
		mesh->invTransform = combinedTransform.Inverted();
		if (morphed /* true if bone weights were affected by animation channel */)
		{
			mesh->SetPose( weights );
			morphed = false;
		}
		if (skinID > -1)
		{
			Skin* skin = Scene::skins[skinID];
			for (int s = (int)skin->joints.size(), j = 0; j < s; j++)
			{
				Node* jointNode = Scene::nodePool[skin->joints[j]];
				skin->jointMat[j] = mesh->invTransform * jointNode->combinedTransform * skin->inverseBindMatrices[j];
			}
			mesh->SetPose( skin ); // TODO: I hope this doesn't overwrite SetPose(weights) ?
		}
		if (mesh->Changed())
		{
			// build or refit BVH
			mesh->UpdateBVH();
			mesh->UpdateWorldBounds();
		}
	}
}

//  +-----------------------------------------------------------------------------+
//  |  Node::PrepareLights                                                        |
//  |  Detects emissive triangles and creates light triangles for them.     LH2'24|
//  +-----------------------------------------------------------------------------+
void Node::PrepareLights()
{
	if (meshID > -1)
	{
		Mesh* mesh = Scene::meshPool[meshID];
		for (int s = (int)mesh->triangles.size(), i = 0; i < s; i++)
		{
			FatTri* tri = &mesh->triangles[i];
			uint matIdx = tri->material;
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
//  |  the node changed.                                                    LH2'24|
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
//  |  Converts a tinyobjloader material to an LH2 Material.                LH2'24|
//  +-----------------------------------------------------------------------------+
void Material::ConvertFrom( const tinyobj::material_t& original )
{
	// properties
	name = original.name;
	color.value = make_float3( original.diffuse[0], original.diffuse[1], original.diffuse[2] ); // Kd
	absorption.value = make_float3( original.transmittance[0], original.transmittance[1], original.transmittance[2] ); // Kt
	ior.value = original.ior; // Ni
	roughness = 1.0f;
	// maps
	if (original.diffuse_texname != "")
	{
		color.textureID = Scene::FindOrCreateTexture( original.diffuse_texname, Texture::LINEARIZED | Texture::FLIPPED );
		color.value = make_float3( 1 ); // we have a texture now; default modulation to white
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
//  |  Converts a tinygltf material to an LH2 Material.                     LH2'24|
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
			color.value = make_float3( (float)p.number_array[0], (float)p.number_array[1], (float)p.number_array[2] );
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
//  |  Constructor.                                                         LH2'24|
//  +-----------------------------------------------------------------------------+
RenderMaterial::RenderMaterial( const Material* source, const vector<int>& offsets )
{
#define TOCHAR(a) ((uint)((a)*255.0f))
#define TOUINT4(a,b,c,d) (TOCHAR(a)+(TOCHAR(b)<<8)+(TOCHAR(c)<<16)+(TOCHAR(d)<<24))
	memset( this, 0, sizeof( RenderMaterial ) );
	SetDiffuse( source->color.value );
	SetTransmittance( float3( 1 ) - source->absorption.value );
	parameters.x = TOUINT4( source->metallic.value, source->subsurface.value, source->specular.value, source->roughness.value );
	parameters.y = TOUINT4( source->specularTint.value, source->anisotropic.value, source->sheen.value, source->sheenTint.value );
	parameters.z = TOUINT4( source->clearcoat.value, source->clearcoatGloss.value, source->transmission.value, 0 );
	parameters.w = *((uint*)&source->eta);
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
//  |  float3 material property into a 'Map', which is a compact representation   |
//  |  for rendering.                                                       LH2'24|
//  +-----------------------------------------------------------------------------+
RenderMaterial::Map RenderMaterial::MakeMap( Material::Float3Value source, const vector<int>& offsets )
{
	Map map;
	Texture* texture = Scene::textures[source.textureID];
	map.width = (ushort)texture->width;
	map.height = (ushort)texture->width;
	map.uscale = float_to_half( source.uvscale.x );
	map.vscale = float_to_half( source.uvscale.y );
	map.uoffs = float_to_half( source.uvoffset.x );
	map.voffs = float_to_half( source.uvoffset.y );
	map.addr = offsets[source.textureID];
	return map;
}

//  +-----------------------------------------------------------------------------+
//  |  Animation::Sampler::Sampler                                                |
//  |  Constructor.                                                         LH2'24|
//  +-----------------------------------------------------------------------------+
Animation::Sampler::Sampler( const tinygltf::AnimationSampler& gltfSampler, const tinygltf::Model& gltfModel )
{
	ConvertFromGLTFSampler( gltfSampler, gltfModel );
}

//  +-----------------------------------------------------------------------------+
//  |  Animation::Sampler::ConvertFromGLTFSampler                                 |
//  |  Convert a gltf animation sampler.                                    LH2'24|
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
	const uchar* b = (const uchar*)(buffer.data.data() + bufferView.byteOffset + outputAccessor.byteOffset);
	if (outputAccessor.type == TINYGLTF_TYPE_VEC3)
	{
		// b is an array of floats (for scale or translation)
		float* f = (float*)b;
		const int N = (int)outputAccessor.count;
		for (int i = 0; i < N; i++) vec3Key.push_back( make_float3( f[i * 3], f[i * 3 + 1], f[i * 3 + 2] ) );
	}
	else if (outputAccessor.type == TINYGLTF_TYPE_SCALAR)
	{
		// b can be FLOAT, BYTE, UBYTE, SHORT or USHORT... (for weights)
		vector<float> fdata;
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
		// b can be FLOAT, BYTE, UBYTE, SHORT or USHORT... (for rotation)
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
			vec4Key.push_back( quat( fdata[i * 4 + 3], fdata[i * 4], fdata[i * 4 + 1], fdata[i * 4 + 2] ) );
		}
	}
	else assert( false );
}

//  +-----------------------------------------------------------------------------+
//  |  Sampler::SampleFloat, SampleVec3, SampleVec4                               |
//  |  Get a value from the sampler.                                        LH2'24|
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
float3 Animation::Sampler::SampleVec3( float currentTime, int k ) const
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
		const float3 p0 = vec3Key[k * 3 + 1];
		const float3 m0 = (t1 - t0) * vec3Key[k * 3 + 2];
		const float3 p1 = vec3Key[(k + 1) * 3 + 1];
		const float3 m1 = (t1 - t0) * vec3Key[(k + 1) * 3];
		return m0 * (t3 - 2 * t2 + tt) + p0 * (2 * t3 - 3 * t2 + 1) + p1 * (-2 * t3 + 3 * t2) + m1 * (t3 - t2);
	}
	case Sampler::STEP: return vec3Key[k];
	default: return (1 - f) * vec3Key[k] + f * vec3Key[k + 1];
	};
}
quat Animation::Sampler::SampleQuat( float currentTime, int k ) const
{
	// handle valid out-of-bounds
	if (k == 0 && currentTime < t[0]) return interpolation == SPLINE ? vec4Key[1] : vec4Key[0];
	// determine interpolation parameters
	const float t0 = t[k], t1 = t[k + 1];
	const float f = (currentTime - t0) / (t1 - t0);
	// sample
	quat key;
	if (f <= 0) return vec4Key[0]; else switch (interpolation)
	{
	#if 1
	case SPLINE:
	{
		const float tt = f, t2 = tt * tt, t3 = t2 * tt;
		const quat p0 = vec4Key[k * 3 + 1];
		const quat m0 = vec4Key[k * 3 + 2] * (t1 - t0);
		const quat p1 = vec4Key[(k + 1) * 3 + 1];
		const quat m1 = vec4Key[(k + 1) * 3] * (t1 - t0);
		key = m0 * (t3 - 2 * t2 + tt) + p0 * (2 * t3 - 3 * t2 + 1) + p1 * (-2 * t3 + 3 * t2) + m1 * (t3 - t2);
		key.normalize();
		break;
	}
#endif
	case STEP:
		key = vec4Key[k];
		break;
	default:
		key = quat::slerp( vec4Key[k], vec4Key[k + 1], f );
		key.normalize();
		break;
	};
	return key;
}

//  +-----------------------------------------------------------------------------+
//  |  Animation::Channel::Channel                                                |
//  |  Constructor.                                                         LH2'24|
//  +-----------------------------------------------------------------------------+
Animation::Channel::Channel( const tinygltf::AnimationChannel& gltfChannel, const int nodeBase )
{
	ConvertFromGLTFChannel( gltfChannel, nodeBase );
}

//  +-----------------------------------------------------------------------------+
//  |  Animation::Channel::ConvertFromGLTFChannel                                 |
//  |  Convert a gltf animation channel.                                    LH2'24|
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
//  |  Advance channel animation time.                                      LH2'24|
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
//  |  Constructor.                                                         LH2'24|
//  +-----------------------------------------------------------------------------+
Animation::Animation( tinygltf::Animation& gltfAnim, tinygltf::Model& gltfModel, const int nodeBase )
{
	ConvertFromGLTFAnim( gltfAnim, gltfModel, nodeBase );
}

//  +-----------------------------------------------------------------------------+
//  |  Animation::ConvertFromGLTFAnim                                             |
//  |  Convert a gltf animation.                                            LH2'24|
//  +-----------------------------------------------------------------------------+
void Animation::ConvertFromGLTFAnim( tinygltf::Animation& gltfAnim, tinygltf::Model& gltfModel, const int nodeBase )
{
	for (int i = 0; i < gltfAnim.samplers.size(); i++) sampler.push_back( new Sampler( gltfAnim.samplers[i], gltfModel ) );
	for (int i = 0; i < gltfAnim.channels.size(); i++) channel.push_back( new Channel( gltfAnim.channels[i], nodeBase ) );
}

//  +-----------------------------------------------------------------------------+
//  |  Animation::SetTime                                                         |
//  |  Set the animation timers of all channels to a specific value.        LH2'24|
//  +-----------------------------------------------------------------------------+
void Animation::SetTime( const float t )
{
	for (int i = 0; i < channel.size(); i++) channel[i]->SetTime( t );
}

//  +-----------------------------------------------------------------------------+
//  |  Animation::Reset                                                           |
//  |  Reset the animation timers of all channels.                          LH2'24|
//  +-----------------------------------------------------------------------------+
void Animation::Reset()
{
	for (int i = 0; i < channel.size(); i++) channel[i]->Reset();
}

//  +-----------------------------------------------------------------------------+
//  |  Animation::Update                                                          |
//  |  Advance channel animation timers.                                    LH2'24|
//  +-----------------------------------------------------------------------------+
void Animation::Update( const float dt )
{
	for (int i = 0; i < channel.size(); i++) channel[i]->Update( dt, sampler[channel[i]->samplerIdx] );
}

//  +-----------------------------------------------------------------------------+
//  |  Texture::Texture                                                           |
//  |  Constructor.                                                         LH2'24|
//  +-----------------------------------------------------------------------------+
Texture::Texture( const char* fileName, const uint modFlags )
{
	Load( fileName, modFlags );
	origin = string( fileName );
}

//  +-----------------------------------------------------------------------------+
//  |  Texture::sRGBtoLinear                                                      |
//  |  Convert sRGB data to linear color space.                             LH2'24|
//  +-----------------------------------------------------------------------------+
void Texture::sRGBtoLinear( uchar* pixels, const uint size, const uint stride )
{
	for (uint j = 0; j < size; j++)
	{
		pixels[j * stride + 0] = (pixels[j * stride + 0] * pixels[j * stride + 0]) >> 8;
		pixels[j * stride + 1] = (pixels[j * stride + 1] * pixels[j * stride + 1]) >> 8;
		pixels[j * stride + 2] = (pixels[j * stride + 2] * pixels[j * stride + 2]) >> 8;
	}
}

//  +-----------------------------------------------------------------------------+
//  |  Texture::Equals                                                            |
//  |  Returns true if the fields that identify the texture are identical to the  |
//  |  supplied values. Used for texture reuse by the Scene object.         LH2'24|
//  +-----------------------------------------------------------------------------+
bool Texture::Equals( const string& o, const uint m )
{
	if (mods != m) return false;
	if (o.compare( origin )) return false;
	return true;
}

//  +-----------------------------------------------------------------------------+
//  |  Texture::PixelsNeeded                                                      |
//  |  Helper function that determines the number of pixels that should be        |
//  |  allocated for the given width, height and MIP level count.           LH2'24|
//  +-----------------------------------------------------------------------------+
int Texture::PixelsNeeded( int w, int h, const int l /* >= 1; includes base layer */ ) const
{
	int needed = 0;
	for (int i = 0; i < l; i++) needed += w * h, w >>= 1, h >>= 1;
	return needed;
}

//  +-----------------------------------------------------------------------------+
//  |  Texture::ConstructMIPmaps                                                  |
//  |  Generate MIP levels for a loaded texture.                            LH2'24|
//  +-----------------------------------------------------------------------------+
void Texture::ConstructMIPmaps()
{
	uint* src = (uint*)idata;
	uint* dst = src + width * height;
	int pw = width, w = width >> 1, ph = height, h = height >> 1;
	for (int i = 1; i < MIPLEVELCOUNT; i++)
	{
		// reduce
		for (int y = 0; y < h; y++) for (int x = 0; x < w; x++)
		{
			const uint src0 = src[x * 2 + (y * 2) * pw];
			const uint src1 = src[x * 2 + 1 + (y * 2) * pw];
			const uint src2 = src[x * 2 + (y * 2 + 1) * pw];
			const uint src3 = src[x * 2 + 1 + (y * 2 + 1) * pw];
			const uint a = min( min( (src0 >> 24) & 255, (src1 >> 24) & 255 ), min( (src2 >> 24) & 255, (src3 >> 24) & 255 ) );
			const uint r = ((src0 >> 16) & 255) + ((src1 >> 16) & 255) + ((src2 >> 16) & 255) + ((src3 >> 16) & 255);
			const uint g = ((src0 >> 8) & 255) + ((src1 >> 8) & 255) + ((src2 >> 8) & 255) + ((src3 >> 8) & 255);
			const uint b = (src0 & 255) + (src1 & 255) + (src2 & 255) + (src3 & 255);
			dst[x + y * w] = (a << 24) + ((r >> 2) << 16) + ((g >> 2) << 8) + (b >> 2);
		}
		// next layer
		src = dst, dst += w * h, pw = w, ph = h, w >>= 1, h >>= 1;
	}
}

//  +-----------------------------------------------------------------------------+
//  |  Texture::Load                                                              |
//  |  Load texture data from disk.                                         LH2'24|
//  +-----------------------------------------------------------------------------+
void Texture::Load( const char* fileName, const uint modFlags, bool normalMap )
{
	// check if texture exists
	FATALERROR_IF( !FileExists( fileName ), "File %s not found", fileName );
#ifdef CACHEIMAGES
	// see if we can load a cached version
	if (strlen( fileName ) > 4) if (fileName[strlen( fileName ) - 4] == '.')
	{
		char binFile[1024];
		memcpy( binFile, fileName, strlen( fileName ) + 1 );
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
			uint version;
			fread( &version, 1, 4, f );
			if (version == BINTEXFILEVERSION)
			{
				fread( &width, 4, 1, f );
				fread( &height, 4, 1, f );
				int dataType;
				fread( &dataType, 4, 1, f );
				fread( &mods, 4, 1, f );
				fread( &flags, 4, 1, f );
				fread( &MIPlevels, 4, 1, f );
				if (dataType == 0)
				{
					int pixelCount = PixelsNeeded( width, height, 1 /* no MIPS for HDR textures */ );
					fdata = (float4*)MALLOC64( sizeof( float4 ) * pixelCount );
					fread( fdata, sizeof( float4 ), pixelCount, f );
				}
				else
				{
					int pixelCount = PixelsNeeded( width, height, MIPLEVELCOUNT );
					idata = (uchar4*)MALLOC64( sizeof( uchar4 ) * pixelCount );
					fread( idata, 4, pixelCount, f );
				}
				fclose( f );
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
		FATALERROR( "TODO: Implement .hdr texture loading." );
		flags |= HDR;
	}
	else
	{
		// load integer image data
		Surface* tmp = new Surface( fileName );
		idata = (uchar4*)MALLOC64( sizeof( uchar4 ) * PixelsNeeded( tmp->width, tmp->height, MIPLEVELCOUNT ) );
		memcpy( idata, tmp->pixels, tmp->width * tmp->height * 4 );
		flags |= LDR, width = tmp->width, height = tmp->height;
		delete tmp;
		// perform sRGB -> linear conversion if requested
		if (mods & LINEARIZED) sRGBtoLinear( (uchar*)idata, width * height, 4 );
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
		memcpy( binFile, fileName, strlen( fileName ) + 1 );
		binFile[strlen( fileName ) - 4] = 0;
		strcat_s( binFile, ".bin" );
		FILE* f = fopen( binFile, "rb" );
		if (f)
		{
			uint version = BINTEXFILEVERSION;
			fwrite( &version, 4, 1, f );
			fwrite( &width, 4, 1, f );
			fwrite( &height, 4, 1, f );
			int dataType = fdata ? 0 : 1;
			fwrite( &dataType, 4, 1, f );
			fwrite( &mods, 4, 1, f );
			fwrite( &flags, 4, 1, f );
			fwrite( &MIPlevels, 4, 1, f );
			if (dataType == 0) fwrite( fdata, sizeof( float4 ), PixelsNeeded( width, height, 1 ), f );
			else fwrite( idata, 4, PixelsNeeded( width, height, MIPLEVELCOUNT ), f );
			fclose( f );
		}
	}
#endif
}

//  +-----------------------------------------------------------------------------+
//  |  Texture::BumpToNormalMap                                                   |
//  |  Convert a bumpmap to a normalmap.                                    LH2'24|
//  +-----------------------------------------------------------------------------+
void Texture::BumpToNormalMap( float heightScale )
{
	uchar* normalMap = new uchar[width * height * 4];
	const float stepZ = 1.0f / 255.0f;
	for (uint i = 0; i < width * height; i++)
	{
		uint xCoord = i % width, yCoord = i / width;
		float xPrev = xCoord > 0 ? idata[i - 1].x * stepZ : idata[i].x * stepZ;
		float xNext = xCoord < width - 1 ? idata[i + 1].x * stepZ : idata[i].x * stepZ;
		float yPrev = yCoord < height - 1 ? idata[i + width].x * stepZ : idata[i].x * stepZ;
		float yNext = yCoord > 0 ? idata[i - width].x * stepZ : idata[i].x * stepZ;
		float3 normal;
		normal.x = (xPrev - xNext) * heightScale;
		normal.y = (yPrev - yNext) * heightScale;
		normal.z = 1;
		normal = normalize( normal );
		normalMap[i * 4 + 0] = (uchar)round( (normal.x * 0.5 + 0.5) * 255 );
		normalMap[i * 4 + 1] = (uchar)round( (normal.y * 0.5 + 0.5) * 255 );
		normalMap[i * 4 + 2] = (uchar)round( (normal.z * 0.5 + 0.5) * 255 );
		normalMap[i * 4 + 3] = 255;
	}
	if (width * height > 0) memcpy( idata, normalMap, width * height * 4 );
	delete normalMap;
}

//  +-----------------------------------------------------------------------------+
//  |  TriLight::TriLight                                                         |
//  |  Constructor.                                                         LH2'24|
//  +-----------------------------------------------------------------------------+
TriLight::TriLight( FatTri* origTri, int origIdx, int origInstance )
{
	triIdx = origIdx;
	instIdx = origInstance;
	vertex0 = origTri->vertex0;
	vertex1 = origTri->vertex1;
	vertex2 = origTri->vertex2;
	centre = 0.333333f * (vertex0 + vertex1 + vertex2);
	N = make_float3( origTri->Nx, origTri->Ny, origTri->Nz );
	const float a = length( vertex1 - vertex0 );
	const float b = length( vertex2 - vertex1 );
	const float c = length( vertex0 - vertex2 );
	const float s = (a + b + c) * 0.5f;
	area = sqrtf( s * (s - a) * (s - b) * (s - c) ); // Heron's formula
	radiance = Scene::materials[origTri->material]->color();
	const float3 E = radiance * area;
	energy = E.x + E.y + E.z;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::~Scene                                                        LH2'24|
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
//  |  Add an existing Mesh to the list of meshes and return the mesh ID.   LH2'24|
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
//  |  the mesh to the list of meshes and return the mesh ID.               LH2'24|
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
//  |  setting the triangles. Set these via the AddTriToMesh function.      LH2'24|
//  +-----------------------------------------------------------------------------+
int Scene::AddMesh( const int triCount )
{
	Mesh* newMesh = new Mesh( triCount );
	return AddMesh( newMesh );
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::AddTriToMesh                                                        |
//  |  Add a single triangle to a mesh.                                     LH2'24|
//  +-----------------------------------------------------------------------------+
void Scene::AddTriToMesh( const int meshId, const float3& v0, const float3& v1, const float3& v2, const int matId )
{
	Mesh* m = Scene::meshPool[meshId];
	m->vertices.push_back( make_float4( v0, 1 ) );
	m->vertices.push_back( make_float4( v1, 1 ) );
	m->vertices.push_back( make_float4( v2, 1 ) );
	FatTri tri;
	tri.material = matId;
	float3 N = normalize( cross( v1 - v0, v2 - v0 ) );
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
//  |  graph node is created for each mesh.                                 LH2'24|
//  +-----------------------------------------------------------------------------+
int Scene::AddScene( const char* sceneFile, const mat4& transform )
{
	// extract directory from specified file name
	char* tmp = new char[strlen( sceneFile ) + 1];
	memcpy( tmp, sceneFile, strlen( sceneFile ) + 1 );
	char* lastSlash = tmp, * pos = tmp;
	while (*pos) { if (*pos == '/' || *pos == '\\') lastSlash = pos; pos++; }
	*lastSlash = 0;
	int retVal = AddScene( lastSlash + 1, tmp, transform );
	delete tmp;
	return retVal;
}
int Scene::AddScene( const char* sceneFile, const char* dir, const mat4& transform )
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
	string err, warn;
	bool ret = false;
	if (cleanFileName.size() > 4)
	{
		string extension4 = cleanFileName.substr( cleanFileName.size() - 5, 5 );
		string extension3 = cleanFileName.substr( cleanFileName.size() - 4, 4 );
		if (extension3.compare( ".obj" ) == 0)
		{
			// ugly to have this here, but it sure does make it easier to make a scene
			// out of a single .obj file...
			float3 scale3( transform.cell[0], transform.cell[5], transform.cell[10] );
			float scale = (scale3.x == scale3.y && scale3.y == scale3.z) ? scale3.x : 1.0f;
			uint meshId = AddMesh( sceneFile, dir, scale );
			AddInstance( AddNode( new Node( meshId, transform * mat4::Scale( 1.0f / scale ) ) ) );
			return retVal;
		}
		else if (extension4.compare( ".gltf" ) == 0)
			ret = loader.LoadASCIIFromFile( &gltfModel, &err, &warn, cleanFileName.c_str() );
		else if (extension3.compare( ".bin" ) == 0 || extension3.compare( ".glb" ) == 0)
			ret = loader.LoadBinaryFromFile( &gltfModel, &err, &warn, cleanFileName.c_str() );
	}
	if (!warn.empty()) printf( "Warn: %s\n", warn.c_str() );
	if (!err.empty()) printf( "Err: %s\n", err.c_str() );
	FATALERROR_IF( !ret, "could not load glTF file:\n%s", cleanFileName.c_str() );
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
			texture->idata = (uchar4*)MALLOC64( texture->PixelsNeeded( image.width, image.height, MIPLEVELCOUNT ) * sizeof( uint ) );
			texture->ID = (uint)textures.size();
			texture->flags |= Texture::LDR;
			memcpy( texture->idata, image.image.data(), size );
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
//  |  centroid position and a material.                                    LH2'24|
//  +-----------------------------------------------------------------------------+
int Scene::AddQuad( float3 N, const float3 pos, const float width, const float height, const int matId, const int meshID )
{
	Mesh* newMesh = meshID > -1 ? meshPool[meshID] : new Mesh();
	N = normalize( N ); // let's not assume the normal is normalized.
#if 1
	const float3 tmp = fabs( N.x ) > 0.9f ? make_float3( 0, 1, 0 ) : make_float3( 1, 0, 0 );
	const float3 T = 0.5f * width * normalize( cross( N, tmp ) );
	const float3 B = 0.5f * height * normalize( cross( normalize( T ), N ) );
#else
	// "Building an Orthonormal Basis, Revisited"
	const float sign = copysignf( 1.0f, N.z ), a = -1.0f / (sign + N.z), b = N.x * N.y * a;
	const float3 B = 0.5f * width * make_float3( 1.0f + sign * N.x * N.x * a, sign * b, -sign * N.x );
	const float3 T = 0.5f * height * make_float3( b, sign + N.y * N.y * a, -N.y );
#endif
	// calculate corners
	uint vertBase = (uint)newMesh->vertices.size();
	newMesh->vertices.push_back( make_float4( pos - B - T, 1 ) );
	newMesh->vertices.push_back( make_float4( pos + B - T, 1 ) );
	newMesh->vertices.push_back( make_float4( pos - B + T, 1 ) );
	newMesh->vertices.push_back( make_float4( pos + B - T, 1 ) );
	newMesh->vertices.push_back( make_float4( pos + B + T, 1 ) );
	newMesh->vertices.push_back( make_float4( pos - B + T, 1 ) );
	// triangles
	FatTri tri1, tri2;
	tri1.material = tri2.material = matId;
	tri1.vN0 = tri1.vN1 = tri1.vN2 = N;
	tri2.vN0 = tri2.vN1 = tri2.vN2 = N;
	tri1.Nx = N.x, tri1.Ny = N.y, tri1.Nz = N.z;
	tri2.Nx = N.x, tri2.Ny = N.y, tri2.Nz = N.z;
	tri1.u0 = tri1.u1 = tri1.u2 = tri1.v0 = tri1.v1 = tri1.v2 = 0;
	tri2.u0 = tri2.u1 = tri2.u2 = tri2.v0 = tri2.v1 = tri2.v2 = 0;
	tri1.vertex0 = make_float3( newMesh->vertices[vertBase + 0] );
	tri1.vertex1 = make_float3( newMesh->vertices[vertBase + 1] );
	tri1.vertex2 = make_float3( newMesh->vertices[vertBase + 2] );
	tri2.vertex0 = make_float3( newMesh->vertices[vertBase + 3] );
	tri2.vertex1 = make_float3( newMesh->vertices[vertBase + 4] );
	tri2.vertex2 = make_float3( newMesh->vertices[vertBase + 5] );
	tri1.T = tri2.T = T / (0.5f * height);
	tri1.B = tri2.B = B / (0.5f * width);
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
//  |  Add an instance of an existing mesh to the scene.                    LH2'24|
//  +-----------------------------------------------------------------------------+
int Scene::AddInstance( const int nodeId )
{
	const uint instId = (uint)rootNodes.size();
	rootNodes.push_back( nodeId );
	return instId;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::AddNode                                                             |
//  |  Add a node to the scene.                                             LH2'24|
//  +-----------------------------------------------------------------------------+
int Scene::AddNode( Node* newNode )
{
	newNode->ID = (int)nodePool.size();
	nodePool.push_back( newNode );
	return newNode->ID;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::AddChildNode                                                        |
//  |  Add a child to a node.                                               LH2'24|
//  +-----------------------------------------------------------------------------+
int Scene::AddChildNode( const int parentNodeId, const int childNodeId )
{
	const int childIdx = (int)nodePool[parentNodeId]->childIdx.size();
	nodePool[parentNodeId]->childIdx.push_back( childNodeId );
	return childIdx;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::GetChildId                                                          |
//  |  Get the node index of a child of a node.                             LH2'24|
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
//  |  TODO: This will not delete any textures or materials.                LH2'24|
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
//  |  Return a texture ID if it already exists.                            LH2'24|
//  +-----------------------------------------------------------------------------+
int Scene::FindTextureID( const char* name )
{
	for (auto texture : textures) if (strcmp( texture->name.c_str(), name ) == 0) return texture->ID;
	return -1;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::FindOrCreateTexture                                                 |
//  |  Return a texture: if it already exists, return the existing texture (after |
//  |  increasing its refCount), otherwise, create a new texture.           LH2'24|
//  +-----------------------------------------------------------------------------+
int Scene::FindOrCreateTexture( const string& origin, const uint modFlags )
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
//  |  (after increasing its refCount), otherwise, create a new texture.    LH2'24|
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
	const int newID = AddMaterial( make_float3( 0 ) );
	materials[newID]->name = name;
	return newID;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::FindOrCreateMaterialCopy                                            |
//  |  Create an untextured material, based on an existing material. This copy is |
//  |  to be used for a triangle that only reads a single texel from a texture;   |
//  |  using a single color is more efficient.                              LH2'24|
//  +-----------------------------------------------------------------------------+
int Scene::FindOrCreateMaterialCopy( const int matID, const uint color )
{
	// search list for existing material copy
	const int r = (color >> 16) & 255, g = (color >> 8) & 255, b = color & 255;
	const float3 c = make_float3( b * (1.0f / 255.0f), g * (1.0f / 255.0f), r * (1.0f / 255.0f) );
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
	const int newID = AddMaterial( make_float3( 0 ) );
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
//  |  Find the ID of a material with the specified name.                   LH2'24|
//  +-----------------------------------------------------------------------------+
int Scene::FindMaterialID( const char* name )
{
	for (auto material : materials) if (material->name.compare( name ) == 0) return material->ID;
	return -1;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::FindMaterialIDByOrigin                                              |
//  |  Find the ID of a material with the specified origin.                 LH2'24|
//  +-----------------------------------------------------------------------------+
int Scene::FindMaterialIDByOrigin( const char* name )
{
	for (auto material : materials) if (material->origin.compare( name ) == 0) return material->ID;
	return -1;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::FindNextMaterialID                                                  |
//  |  Find the ID of a material with the specified name, with an ID greater than |
//  |  the specified one. Used to find materials with the same name.        LH2'24|
//  +-----------------------------------------------------------------------------+
int Scene::FindNextMaterialID( const char* name, const int matID )
{
	for (int s = (int)materials.size(), i = matID + 1; i < s; i++)
		if (materials[i]->name.compare( name ) == 0) return materials[i]->ID;
	return -1;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::FindNode                                                            |
//  |  Find the ID of a node with the specified name.                       LH2'24|
//  +-----------------------------------------------------------------------------+
int Scene::FindNode( const char* name )
{
	for (auto node : nodePool) if (node->name.compare( name ) == 0) return node->ID;
	return -1;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::SetNodeTransform                                                    |
//  |  Set the local transform for the specified node.                      LH2'24|
//  +-----------------------------------------------------------------------------+
void Scene::SetNodeTransform( const int nodeId, const mat4& transform )
{
	if (nodeId < 0 || nodeId >= nodePool.size()) return;
	nodePool[nodeId]->localTransform = transform;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::GetNodeTransform                                                    |
//  |  Set the local transform for the specified node.                      LH2'24|
//  +-----------------------------------------------------------------------------+
const mat4& Scene::GetNodeTransform( const int nodeId )
{
	static mat4 dummyIdentity;
	if (nodeId < 0 || nodeId >= nodePool.size()) return dummyIdentity;
	return nodePool[nodeId]->localTransform;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::ResetAnimation                                                      |
//  |  Reset the indicated animation.                                       LH2'24|
//  +-----------------------------------------------------------------------------+
void Scene::ResetAnimation( const int animId )
{
	if (animId < 0 || animId >= animations.size()) return;
	animations[animId]->Reset();
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::ResetAnimation                                                      |
//  |  Update the indicated animation.                                      LH2'24|
//  +-----------------------------------------------------------------------------+
void Scene::UpdateAnimation( const int animId, const float dt )
{
	if (animId < 0 || animId >= animations.size()) return;
	animations[animId]->Update( dt );
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::CreateTexture                                                       |
//  |  Return a texture. Create it anew, even if a texture with the same origin   |
//  |  already exists.                                                      LH2'24|
//  +-----------------------------------------------------------------------------+
int Scene::CreateTexture( const string& origin, const uint modFlags )
{
	// create a new texture
	Texture* newTexture = new Texture( origin.c_str(), modFlags );
	textures.push_back( newTexture );
	return newTexture->ID = (int)textures.size() - 1;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::AddMaterial                                                         |
//  |  Adds an existing Material* and returns the ID. If the material             |
//  |  with that pointer is already added, it is not added again.           LH2'24|
//  +-----------------------------------------------------------------------------+
int Scene::AddMaterial( Material* material )
{
	auto res = std::find( materials.begin(), materials.end(), material );
	if (res != materials.end()) return (int)std::distance( materials.begin(), res );
	int matid = (int)materials.size();
	materials.push_back( material );
	return matid;
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::AddMaterial                                                         |
//  |  Create a material, with a limited set of parameters.                 LH2'24|
//  +-----------------------------------------------------------------------------+
int Scene::AddMaterial( const float3 color, const char* name )
{
	Material* material = new Material();
	material->color = color;
	if (name) material->name = name;
	return material->ID = AddMaterial( material );
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::AddPointLight                                                       |
//  |  Create a point light and add it to the scene.                        LH2'24|
//  +-----------------------------------------------------------------------------+
int Scene::AddPointLight( const float3 pos, const float3 radiance )
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
//  |  Create a spot light and add it to the scene.                         LH2'24|
//  +-----------------------------------------------------------------------------+
int Scene::AddSpotLight( const float3 pos, const float3 direction, const float inner, const float outer, const float3 radiance )
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
//  |  Create a directional light and add it to the scene.                  LH2'24|
//  +-----------------------------------------------------------------------------+
int Scene::AddDirectionalLight( const float3 direction, const float3 radiance )
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
//  |  Walk the scene graph, updating all node matrices.                    LH2'24|
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
		mat4 T;
		node->Update( T /* start with an identity matrix */ );
	}
	// construct TLAS
	static Mesh m( 512 /* this will be our max BLAS count for now */ );
	const uint blasCount = (uint)meshPool.size();
	for (uint i = 0; i < blasCount; i++)
	{
		m.vertices[i * 3 + 0] = meshPool[i]->worldBounds.bmin3;
		m.vertices[i * 3 + 1] = m.vertices[i * 3 + 2] = meshPool[i]->worldBounds.bmax3;
	}
	tlas.Build( &m, blasCount );
}

#ifdef ENABLE_OPENCL_BVH

//  +-----------------------------------------------------------------------------+
//  |  Scene::InitializeGPUData                                                   |
//  |  Gather accstruc, material and texture data and store it in OpenCL buffers. |
//  |  Note: The scene layout is assumed to be finalized at this point; adding    |
//  |  meshes or changing mesh polygon counts will require more work.       LH2'24|
//  +-----------------------------------------------------------------------------+
void Scene::InitializeGPUData()
{
	// force bvh build so we have data to copy to GPU
	UpdateSceneGraph( 0 );
	// figure out how many BVH nodes and indices we have in total
	int nodeCount = 0, node4Quads = 0, altNodeCount = 0, primCount = 0, idxCount = 0;
	for (int s = (int)meshPool.size(), i = 0; i < s; i++)
	{
		BVH* bvh = meshPool[i]->bvh;
		nodeCount += bvh->newNodePtr;
		node4Quads += bvh->newAlt4Ptr;
		altNodeCount += bvh->newAltNodePtr;
		primCount += (int)meshPool[i]->vertices.size() / 3;
		idxCount += bvh->idxCount;
	}
	nodeCount += tlas.newNodePtr;
	altNodeCount += tlas.newAltNodePtr;
	primCount += (int)meshPool.size();
	idxCount += (int)meshPool.size();
	// count textures and texture pixels
	uint ldrTexCount = 0, ldrPixelCount = 0;
	uint hdrTexCount = 0, hdrPixelCount = 0;
	for (int s = (int)textures.size(), i = 0; i < s; i++)
	{
		if (textures[i]->flags & Texture::HDR)
		{
			hdrTexCount++;
			hdrPixelCount += textures[i]->width * textures[i]->height;
		}
		else
		{
			ldrTexCount++;
			ldrPixelCount += textures[i]->width * textures[i]->height;
		}
	}
	// allocate buffers for GPU data
	bvhNodeData = new Buffer( nodeCount * sizeof( BVH::BVHNode ) );
	altNodeData = new Buffer( altNodeCount * sizeof( BVH::BVHNodeAlt ) );
	bvh8Compacted = new Buffer( nodeCount * sizeof( float4 ) * 5 );
	bvh8TriData = new Buffer( idxCount * sizeof( float4 ) * 3 );
	bvh4Compacted = new Buffer( node4Quads * 16 );
	triangleData = new Buffer( primCount * 3 * sizeof( float4 ) );
	triangleIdxData = new Buffer( idxCount * sizeof( uint ) );
	offsetData = new Buffer( (int)meshPool.size() * 2 * sizeof( uint4 ) );
	transformData = new Buffer( (int)meshPool.size() * 2 * sizeof( mat4 ) );
	fatTriData = new Buffer( primCount * sizeof( FatTri ) );
	ldrData = new Buffer( ldrPixelCount * sizeof( uint ) );
	hdrData = new Buffer( hdrPixelCount * sizeof( float4 ) );
	materialData = new Buffer( (int)materials.size() * sizeof( RenderMaterial ) );
	// populate the buffers
	uchar* bvhPtr = (uchar*)bvhNodeData->GetHostPtr();
	uchar* altPtr = (uchar*)altNodeData->GetHostPtr();
	uchar* cmpPtr = (uchar*)bvh8Compacted->GetHostPtr();
	uchar* tr8Ptr = (uchar*)bvh8TriData->GetHostPtr();
	uchar* n4qPtr = (uchar*)bvh4Compacted->GetHostPtr();
	uchar* triPtr = (uchar*)triangleData->GetHostPtr();
	uchar* fatPtr = (uchar*)fatTriData->GetHostPtr();
	uchar* idxPtr = (uchar*)triangleIdxData->GetHostPtr();
	uint4* offset = (uint4*)offsetData->GetHostPtr();
	memcpy( bvhPtr, tlas.bvhNode, tlas.newNodePtr * sizeof( BVH::BVHNode ) );
	memcpy( idxPtr, tlas.triIdx, meshPool.size() * sizeof( uint ) );
	if (sky)
	{
		skyData = new Buffer( sky->width * sky->height * sizeof( float4 ) );
		for (int s = sky->width * sky->height, i = 0; i < s; i++)
			((float4*)skyData->GetHostPtr())[i] = float4( sky->pixels[i], 0 );
	}
	bvhPtr += 2 * (int)meshPool.size() * sizeof( BVH::BVHNode );
	idxPtr += (int)meshPool.size() * sizeof( uint );
	for (int s = (int)meshPool.size(), i = 0; i < s; i++)
	{
		Mesh* mesh = meshPool[i];
		memcpy( bvhPtr, mesh->bvh->bvhNode, mesh->bvh->newNodePtr * 32 );
		memcpy( altPtr, mesh->bvh->altNode, mesh->bvh->newAltNodePtr * 64 );
		memcpy( cmpPtr, mesh->bvh->bvh8Compact, mesh->bvh->newNodePtr * 5 * sizeof( float4 ) );
		memcpy( tr8Ptr, mesh->bvh->bvh8Tris, mesh->bvh->idxCount * sizeof( float4 ) * 3 );
		memcpy( n4qPtr, mesh->bvh->alt4Data, mesh->bvh->newAlt4Ptr * sizeof( float4 ) );
		memcpy( idxPtr, mesh->bvh->triIdx, mesh->bvh->idxCount * 4 );
		memcpy( fatPtr, mesh->triangles.data(), ((int)mesh->vertices.size() / 3) * sizeof( FatTri ) );
		memcpy( triPtr, mesh->vertices.data(), (int)mesh->vertices.size() * 16 );
		offset[i * 2 + 0] = uint4(
			(int)(bvhPtr - (uchar*)bvhNodeData->GetHostPtr()) / 16,
			(int)(idxPtr - (uchar*)triangleIdxData->GetHostPtr()) / 4,
			(int)(triPtr - (uchar*)triangleData->GetHostPtr()) / 16,
			(int)(fatPtr - (uchar*)fatTriData->GetHostPtr()) / sizeof( FatTri )
		);
		offset[i * 2 + 1] = uint4(
			(int)(altPtr - (uchar*)altNodeData->GetHostPtr()) / 16,
			(int)(cmpPtr - (uchar*)bvh8Compacted->GetHostPtr()) / 16,
			(int)(n4qPtr - (uchar*)bvh4Compacted->GetHostPtr()) / 16,
			(int)(tr8Ptr - (uchar*)bvh8TriData->GetHostPtr()) / 16
		);
		memcpy( transformData->GetHostPtr() + i * 32, &mesh->transform, 64 );
		memcpy( transformData->GetHostPtr() + i * 32 + 16, &mesh->invTransform, 64 );
		bvhPtr += mesh->bvh->newNodePtr * 32;
		altPtr += mesh->bvh->newAltNodePtr * 64;
		cmpPtr += mesh->bvh->newNodePtr * 5 * sizeof( float4 );
		tr8Ptr += mesh->bvh->idxCount * 3 * sizeof( float4 );
		n4qPtr += mesh->bvh->newAlt4Ptr * sizeof( float4 );
		idxPtr += mesh->bvh->idxCount * 4;
		fatPtr += ((int)mesh->vertices.size() / 3) * sizeof( FatTri );
		triPtr += (int)mesh->vertices.size() * 16;
	}
	// copy texture pixels to buffers
	vector<int> offsets;
	int ldrOffset = 0, hdrOffset = 0;
	for (int s = (int)textures.size(), i = 0; i < s; i++)
	{
		int texelCount = textures[i]->width * textures[i]->height;
		if (textures[i]->flags & Texture::HDR)
		{
			memcpy( (float4*)hdrData->GetHostPtr() + hdrOffset, textures[i]->fdata, texelCount * sizeof( float4 ) );
			offsets.push_back( hdrOffset );
			hdrOffset += texelCount;
		}
		else
		{
			memcpy( (uint*)ldrData->GetHostPtr() + ldrOffset, textures[i]->idata, texelCount * sizeof( uint ) );
			offsets.push_back( ldrOffset );
			ldrOffset += texelCount;
		}
	}
	// convert materials
	for (int s = (int)materials.size(), i = 0; i < s; i++)
	{
		RenderMaterial m( materials[i], offsets );
		((RenderMaterial*)materialData->GetHostPtr())[i] = m;
	}
	// send the data to the gpu
	bvhNodeData->CopyToDevice();
	altNodeData->CopyToDevice();
	bvh8Compacted->CopyToDevice();
	bvh8TriData->CopyToDevice();
	bvh4Compacted->CopyToDevice();
	triangleData->CopyToDevice();
	fatTriData->CopyToDevice();
	triangleIdxData->CopyToDevice();
	offsetData->CopyToDevice();
	transformData->CopyToDevice();
	if (skyData) skyData->CopyToDevice();
	ldrData->CopyToDevice();
	hdrData->CopyToDevice();
	materialData->CopyToDevice();
}

//  +-----------------------------------------------------------------------------+
//  |  Scene::UpdateGPUData                                                       |
//  |  Synchronize TLAS and transform changes to the GPU.                         |
//  |  Note that this only handles scenes with rigid animation.             LH2'24|
//  +-----------------------------------------------------------------------------+
void Scene::UpdateGPUData()
{
	// tlas and matrices will be synced each frame
	uchar* idxPtr = (uchar*)triangleIdxData->GetHostPtr();
	uchar* bvhPtr = (uchar*)bvhNodeData->GetHostPtr();
	memcpy( bvhPtr, tlas.bvhNode, tlas.newNodePtr * 32 );
	memcpy( idxPtr, tlas.triIdx, meshPool.size() * 4 );
	for (int s = (int)meshPool.size(), i = 0; i < s; i++)
	{
		memcpy( transformData->GetHostPtr() + i * 32, &meshPool[i]->transform, 64 );
		memcpy( transformData->GetHostPtr() + i * 32 + 16, &meshPool[i]->invTransform, 64 );
	}
	// send the data to the gpu
	bvhNodeData->CopyToDevice( 0, tlas.newNodePtr * 32 /* just the tlas */ );
	triangleIdxData->CopyToDevice( 0, (int)meshPool.size() * 4 /* just the tlas */ );
	transformData->CopyToDevice();
}

#endif

//  +-----------------------------------------------------------------------------+
//  |  Scene::Intersect                                                           |
//  |  Intersect a TLAS with a ray.                                         LH2'24|
//  +-----------------------------------------------------------------------------+
int Scene::Intersect( Ray& ray )
{
	// use a local stack instead of a recursive function
	BVH::BVHNode* node = &tlas.bvhNode[0], * stack[128];
	uint stackPtr = 0, steps = 0;
	// traversl loop; terminates when the stack is empty
	while (1)
	{
		steps++;
		if (node->isLeaf())
		{
			// current node is a leaf: intersect BLAS
			int blasCount = node->triCount;
			for (int i = 0; i < blasCount; i++)
			{
				const uint meshIdx = tlas.triIdx[node->leftFirst + i];
				steps += meshPool[meshIdx]->Intersect( ray );
			}
			// pop a node from the stack; terminate if none left
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		// current node is an interior node: visit child nodes, ordered
		BVH::BVHNode* child1 = &tlas.bvhNode[node->leftFirst];
		BVH::BVHNode* child2 = &tlas.bvhNode[node->leftFirst + 1];
		float dist1 = child1->Intersect( ray ), dist2 = child2->Intersect( ray );
		if (dist1 > dist2) { swap( dist1, dist2 ); swap( child1, child2 ); }
		if (dist1 == 1e30f)
		{
			if (stackPtr == 0) break; else node = stack[--stackPtr];
		}
		else
		{
			node = child1;
			if (dist2 != 1e30f) stack[stackPtr++] = child2;
		}
	}
	return steps;
}

#if 0
// generic code for bin processing, here for reference only.
__m256 lBox8[BVHBINS], rBox8[BVHBINS];
float ANLR[BVHBINS] = { 0 };
lBox8[0] = bb[0], rBox8[BVHBINS - 2] = bb[BVHBINS - 1];
uint lN = count[a][0], rN = count[a][BVHBINS - 1];
ANLR[0] = lN == 0 ? 1e30f : (halfArea( lBox8[0] ) * (float)lN);
ANLR[BVHBINS - 2] = rN == 0 ? 1e30f : (halfArea( rBox8[BVHBINS - 2] ) * (float)rN);
// handle subsequent bins
for (uint i = 1; i < BVHBINS - 1; i++)
{
	const __m256 lb = lBox8[i] = _mm256_max_ps( lBox8[i - 1], bb[i] );
	const __m256 rb = rBox8[BVHBINS - 2 - i] = _mm256_max_ps( rBox8[BVHBINS - 1 - i], bb[BVHBINS - 1 - i] );
	lN += count[a][i], rN += count[a][BVHBINS - 1 - i];
	ANLR[i] += lN == 0 ? 1e30f : (halfArea( lb ) * (float)lN);
	ANLR[BVHBINS - 2 - i] += rN == 0 ? 1e30f : (halfArea( rb ) * (float)rN);
}
// evaluate bin totals to find best position for object split
for (uint i = 0; i < BVHBINS - 1; i++) if (ANLR[i] < splitCost)
{
	splitCost = ANLR[i], bestAxis = a, bestPos = i;
	bestLBox = lBox8[i], bestRBox = rBox8[i];
}
#endif