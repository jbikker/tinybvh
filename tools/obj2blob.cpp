// obj2blob - converts a Wavefront .obj (+ .mtl + textures) into a single binary blob.
// Supported: v, vt, f (polygons are fan-triangulated; v, v/vt, v//vn, v/vt/vn; negative
// indices), mtllib, usemtl. Materials: Kd, d, map_Kd, map_d. Everything else is ignored.
// Textures are searched relative to the .mtl: as written, in <mtldir>/textures/, and in
// <mtldir>/ itself. Images are decoded with stb_image (PNG, JPG, TGA, BMP, ...).
//
// Build: g++ -O2 -std=c++17 obj2blob.cpp -o obj2blob
//        cl /O2 /std:c++17 /EHsc obj2blob.cpp
// Usage: obj2blob input.obj output.blob
//
// Blob layout (little endian, all fields 32-bit):
//   BlobHeader
//   bvhvec4 verts[triCount * 3]   x, y, z, w = material index (uint32 bit pattern);
//                                 directly usable as tinybvh triangle input.
//   float   uvs[triCount * 3 * 2]
//   BlobMaterial materials[materialCount]
//   BlobTexture  textures[textureCount]
//   uint32  texels[texelCount]    0xAARRGGBB, row 0 is the top row of the image.

#define _CRT_SECURE_NO_WARNINGS
#define STB_IMAGE_IMPLEMENTATION
#include "../external/stb_image.h" // ships with TinyBVH
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

static const uint32_t BLOB_MAGIC = 0x424c4254; // "TBLB"
static const uint32_t BLOB_VERSION = 1;
enum { MAT_ALPHA_MASK = 1, MAT_ALPHA_BLEND = 2 };

struct BlobHeader { uint32_t magic, version, triCount, materialCount, textureCount, texelCount; };
struct BlobMaterial
{
	uint32_t color;		// 0xAARRGGBB; Kd, alpha = d
	int32_t texture;	// index into textures, -1 if none
	uint32_t flags;		// MAT_ALPHA_MASK: alpha test at 0.5; MAT_ALPHA_BLEND: d < 1
	float opacity;		// d
};
struct BlobTexture { uint32_t width, height, firstTexel, pad; };

struct Material
{
	std::string name, mapKd, mapD;
	float kd[3] = { 1, 1, 1 }, d = 1;
};

static std::string DirOf( const std::string& p )
{
	size_t s = p.find_last_of( "/\\" );
	return s == std::string::npos ? std::string() : p.substr( 0, s + 1 );
}
static std::string FileOf( const std::string& p )
{
	size_t s = p.find_last_of( "/\\" );
	return s == std::string::npos ? p : p.substr( s + 1 );
}
static bool Exists( const std::string& p ) { FILE* f = fopen( p.c_str(), "rb" ); if (f) fclose( f ); return f != 0; }
static std::string Trim( const std::string& s )
{
	size_t a = s.find_first_not_of( " \t\r\n" ), b = s.find_last_not_of( " \t\r\n" );
	return a == std::string::npos ? std::string() : s.substr( a, b - a + 1 );
}

// Map statements may carry options ("-imfchan m file.png"); the file name is what follows
// the last option. Options with arguments are skipped by their known argument counts.
static std::string MapFile( std::istringstream& ss )
{
	static const std::map<std::string, int> args = {
		{ "-blendu", 1 }, { "-blendv", 1 }, { "-bm", 1 }, { "-boost", 1 }, { "-cc", 1 }, { "-clamp", 1 },
		{ "-imfchan", 1 }, { "-mm", 2 }, { "-o", 3 }, { "-s", 3 }, { "-t", 3 }, { "-texres", 1 } };
	std::string tok;
	while (ss >> tok)
	{
		auto it = args.find( tok );
		if (it == args.end()) break;
		for (int i = 0; i < it->second; i++) ss >> std::ws, ss >> tok;
	}
	std::string rest; std::getline( ss, rest );
	return Trim( tok + rest );
}

static std::string FindTexture( const std::string& mtlDir, const std::string& ref )
{
	std::string name = FileOf( ref );
	const std::string candidates[3] = { mtlDir + ref, mtlDir + "textures/" + name, mtlDir + name };
	for (const std::string& c : candidates) if (Exists( c )) return c;
	return std::string();
}

static void LoadMTL( const std::string& file, std::vector<Material>& mats, std::map<std::string, int>& lut )
{
	std::ifstream in( file );
	if (!in) { printf( "warning: cannot open material library %s\n", file.c_str() ); return; }
	std::string line;
	Material* m = 0;
	while (std::getline( in, line ))
	{
		std::istringstream ss( line );
		std::string key; ss >> key;
		if (key == "newmtl")
		{
			Material n; std::getline( ss, n.name ), n.name = Trim( n.name );
			lut[n.name] = (int)mats.size(), mats.push_back( n ), m = &mats.back();
		}
		else if (!m) continue;
		else if (key == "Kd") ss >> m->kd[0] >> m->kd[1] >> m->kd[2];
		else if (key == "d") ss >> m->d;
		else if (key == "Tr") { float tr; ss >> tr, m->d = 1 - tr; }
		else if (key == "map_Kd") m->mapKd = MapFile( ss );
		else if (key == "map_d") m->mapD = MapFile( ss );
	}
}

static uint32_t PackColor( float r, float g, float b, float a )
{
	auto q = []( float v ) { v = v < 0 ? 0 : (v > 1 ? 1 : v); return (uint32_t)(v * 255.0f + 0.5f); };
	return (q( a ) << 24) | (q( r ) << 16) | (q( g ) << 8) | q( b );
}

int main( int argc, char** argv )
{
	if (argc != 3) { printf( "usage: obj2blob input.obj output.blob\n" ); return 1; }
	const std::string objFile = argv[1], objDir = DirOf( objFile );
	std::ifstream in( objFile );
	if (!in) { printf( "error: cannot open %s\n", objFile.c_str() ); return 1; }

	// parse obj
	std::vector<float> pos, tex;			// xyz per v, uv per vt
	std::vector<float> outVerts, outUVs;	// 4 floats / 2 floats per triangle vertex
	std::vector<Material> mats;
	std::map<std::string, int> matLUT;
	std::vector<std::string> mtlDirs;		// per material: directory of its .mtl
	int curMat = -1, defaultMat = -1;
	size_t skipped = 0;
	std::string line;
	while (std::getline( in, line ))
	{
		std::istringstream ss( line );
		std::string key; ss >> key;
		if (key == "v") { float x = 0, y = 0, z = 0; ss >> x >> y >> z; pos.push_back( x ), pos.push_back( y ), pos.push_back( z ); }
		else if (key == "vt") { float u = 0, v = 0; ss >> u >> v; tex.push_back( u ), tex.push_back( v ); }
		else if (key == "mtllib")
		{
			std::string lib; std::getline( ss, lib );
			const std::string path = objDir + Trim( lib );
			const size_t first = mats.size();
			LoadMTL( path, mats, matLUT );
			for (size_t i = first; i < mats.size(); i++) mtlDirs.push_back( DirOf( path ) );
		}
		else if (key == "usemtl")
		{
			std::string name; std::getline( ss, name ), name = Trim( name );
			auto it = matLUT.find( name );
			if (it != matLUT.end()) curMat = it->second; else
			{
				printf( "warning: unknown material '%s', using default\n", name.c_str() );
				curMat = -1;
			}
		}
		else if (key == "f")
		{
			int pi[64], ti[64], n = 0;
			std::string tok;
			while (ss >> tok && n < 64)
			{
				int p = 0, t = 0;
				const char* s = tok.c_str();
				p = atoi( s );
				const char* slash = strchr( s, '/' );
				if (slash && slash[1] != '/') t = atoi( slash + 1 );
				pi[n] = p < 0 ? (int)(pos.size() / 3) + p : p - 1;
				ti[n] = t == 0 ? -1 : (t < 0 ? (int)(tex.size() / 2) + t : t - 1);
				n++;
			}
			if (n < 3) { skipped++; continue; }
			if (curMat < 0)
			{
				if (defaultMat < 0)
				{
					Material d; d.name = "__default";
					defaultMat = (int)mats.size(), mats.push_back( d ), mtlDirs.push_back( objDir );
				}
				curMat = defaultMat;
			}
			for (int k = 1; k + 1 < n; k++)
			{
				const int idx[3] = { 0, k, k + 1 };
				for (int j = 0; j < 3; j++)
				{
					const int p = pi[idx[j]], t = ti[idx[j]];
					if (p < 0 || p * 3 + 2 >= (int)pos.size()) { printf( "error: vertex index out of range\n" ); return 1; }
					float w; uint32_t m = (uint32_t)curMat; memcpy( &w, &m, 4 );
					outVerts.insert( outVerts.end(), { pos[p * 3], pos[p * 3 + 1], pos[p * 3 + 2], w } );
					const bool hasT = t >= 0 && t * 2 + 1 < (int)tex.size();
					outUVs.push_back( hasT ? tex[t * 2] : 0 ), outUVs.push_back( hasT ? tex[t * 2 + 1] : 0 );
				}
			}
		}
	}
	const uint32_t triCount = (uint32_t)(outVerts.size() / 12);

	// textures: deduplicate on (color map, alpha map) file pair
	std::vector<BlobMaterial> outMats;
	std::vector<BlobTexture> outTex;
	std::vector<uint32_t> texels;
	std::map<std::string, int> texLUT;
	for (size_t i = 0; i < mats.size(); i++)
	{
		const Material& m = mats[i];
		BlobMaterial bm;
		bm.color = PackColor( m.kd[0], m.kd[1], m.kd[2], m.d );
		bm.texture = -1, bm.opacity = m.d;
		bm.flags = (m.mapD.empty() ? 0 : MAT_ALPHA_MASK) | (m.d < 1 ? MAT_ALPHA_BLEND : 0);
		const std::string kdFile = m.mapKd.empty() ? "" : FindTexture( mtlDirs[i], m.mapKd );
		const std::string dFile = m.mapD.empty() ? "" : FindTexture( mtlDirs[i], m.mapD );
		if (!m.mapKd.empty() && kdFile.empty()) printf( "warning: texture not found: %s\n", m.mapKd.c_str() );
		if (!m.mapD.empty() && dFile.empty()) printf( "warning: alpha map not found: %s\n", m.mapD.c_str() );
		if (dFile.empty()) bm.flags &= ~MAT_ALPHA_MASK;
		if (!kdFile.empty())
		{
			const std::string key = kdFile + "|" + (dFile == kdFile ? "" : dFile);
			auto it = texLUT.find( key );
			if (it != texLUT.end()) bm.texture = it->second; else
			{
				int w, h, c;
				unsigned char* img = stbi_load( kdFile.c_str(), &w, &h, &c, 4 );
				if (!img) printf( "warning: failed to decode %s\n", kdFile.c_str() ); else
				{
					// separate alpha map: take its alpha channel, or its first channel if it has none
					if (!dFile.empty() && dFile != kdFile)
					{
						int aw, ah, ac;
						unsigned char* a = stbi_load( dFile.c_str(), &aw, &ah, &ac, 0 );
						if (a && aw == w && ah == h) for (int p = 0; p < w * h; p++)
							img[p * 4 + 3] = a[p * ac + (ac == 2 || ac == 4 ? ac - 1 : 0)];
						else printf( "warning: alpha map %s unusable (size mismatch?)\n", dFile.c_str() );
						stbi_image_free( a );
					}
					BlobTexture t = { (uint32_t)w, (uint32_t)h, (uint32_t)texels.size(), 0 };
					for (int p = 0; p < w * h; p++)
					{
						const unsigned char* q = img + p * 4;
						texels.push_back( ((uint32_t)q[3] << 24) | ((uint32_t)q[0] << 16) | ((uint32_t)q[1] << 8) | q[2] );
					}
					stbi_image_free( img );
					bm.texture = (int32_t)outTex.size(), texLUT[key] = bm.texture, outTex.push_back( t );
				}
			}
		}
		else if (!dFile.empty()) bm.flags &= ~MAT_ALPHA_MASK; // alpha map without color map: unsupported
		outMats.push_back( bm );
	}

	// write
	BlobHeader hdr = { BLOB_MAGIC, BLOB_VERSION, triCount, (uint32_t)outMats.size(), (uint32_t)outTex.size(), (uint32_t)texels.size() };
	std::ofstream out( argv[2], std::ios::binary );
	if (!out) { printf( "error: cannot write %s\n", argv[2] ); return 1; }
	out.write( (const char*)&hdr, sizeof( hdr ) );
	out.write( (const char*)outVerts.data(), outVerts.size() * 4 );
	out.write( (const char*)outUVs.data(), outUVs.size() * 4 );
	out.write( (const char*)outMats.data(), outMats.size() * sizeof( BlobMaterial ) );
	out.write( (const char*)outTex.data(), outTex.size() * sizeof( BlobTexture ) );
	out.write( (const char*)texels.data(), texels.size() * 4 );
	if (!out) { printf( "error: write failed\n" ); return 1; }
	printf( "%u triangles, %u materials, %u textures, %u texels", triCount, hdr.materialCount, hdr.textureCount, hdr.texelCount );
	if (skipped) printf( ", %zu degenerate faces skipped", skipped );
	printf( "\n" );
	return 0;
}
