// Template, 2024 IGAD Edition
// Get the latest version from: https://github.com/jbikker/tmpl8
// IGAD/NHTV/BUAS/UU - Jacco Bikker - 2006-2024

#include "precomp.h"
#include "game.h"
#define TINYBVH_NO_SIMD
#define TINYBVH_IMPLEMENTATION
#include "tiny_bvh.h"

tinybvh::BVH_GPU bvh;

// -----------------------------------------------------------
// Create a sphere flake for testing
// -----------------------------------------------------------
void Game::SphereFlake( float x, float y, float z, float s, int d )
{
	// procedural tesselated sphere flake object
#define P(F,a,b,c) p[i+F*64]={(float)a ,(float)b,(float)c}
	float3 p[384], pos( x, y, z ), ofs( 3.5 );
	for (int i = 0, u = 0; u < 8; u++) for (int v = 0; v < 8; v++, i++)
		P( 0, u, v, 0 ), P( 1, u, 0, v ), P( 2, 0, u, v ),
		P( 3, u, v, 7 ), P( 4, u, 7, v ), P( 5, 7, u, v );
	for (int i = 0; i < 384; i++) p[i] = normalize( p[i] - ofs ) * s + pos;
	for (int i = 0, side = 0; side < 6; side++, i += 8)
		for (int u = 0; u < 7; u++, i++) for (int v = 0; v < 7; v++, i++)
			vertices[verts++] = p[i], vertices[verts++] = p[i + 8],
			vertices[verts++] = p[i + 1], vertices[verts++] = p[i + 1],
			vertices[verts++] = p[i + 9], vertices[verts++] = p[i + 8];
	if (d < 3) SphereFlake( x + s * 1.55f, y, z, s * 0.5f, d + 1 );
	if (d < 3) SphereFlake( x - s * 1.5f, y, z, s * 0.5f, d + 1 );
	if (d < 3) SphereFlake( x, y + s * 1.5f, z, s * 0.5f, d + 1 );
	if (d < 3) SphereFlake( x, y - s * 1.5f, z, s * 0.5f, d + 1 );
	if (d < 3) SphereFlake( x, y, z + s * 1.5f, s * 0.5f, d + 1 );
	if (d < 3) SphereFlake( x, y, z - s * 1.5f, s * 0.5f, d + 1 );
}

// -----------------------------------------------------------
// Initialize the application
// -----------------------------------------------------------
void Game::Init()
{
	// load and compile compute shader
	int compute = glCreateShader( GL_COMPUTE_SHADER );
	const string shaderSource = TextFileRead( "shaders/traverse.comp" );
	const char* source = shaderSource.c_str();
	glShaderSource( compute, 1, &source, 0 );
	glCompileShader( compute );
	int result, logSize = 0;
	char log[16384];
	glGetShaderiv( compute, GL_COMPILE_STATUS, &result );
	if (!result /* compilation failed */)
	{
		glGetShaderInfoLog( compute, sizeof( log ), &logSize, log );
		FATALERROR( "%s", log );
	}
	computeProgram = glCreateProgram();
	glAttachShader( computeProgram, compute );
	glLinkProgram( computeProgram );
	glGetProgramiv( computeProgram, GL_LINK_STATUS, &result );
	if (!result /* link failed */)
	{
		glGetProgramInfoLog( computeProgram, sizeof( log ), &logSize, log );
		FATALERROR( "%s", log );
	}
	
	// load scene
	vertices = (float4*)MALLOC64( 259 /* level 3 */ * 6 * 2 * 49 * 3 * sizeof( float4 ) );
	indices = (uint*)MALLOC64( 259 /* level 3 */ * 6 * 2 * 49 * 3 * sizeof( uint ) );
	SphereFlake(  0, 0, 0, 1.5f );
	bvh.Build( (tinybvh::bvhvec4*)vertices, verts / 3 );

	// prepare buffers
	glGenBuffers( 1, &nodeData );		// BVH node data
	glGenBuffers( 1, &idxData );		// triangle index data
	glGenBuffers( 1, &triData );		// triangle vertex data, 3 * float4 per triangle
	glGenBuffers( 1, &hitData );		// buffer to return intersection results in, one float4 per pixel
	glBindBuffer( GL_SHADER_STORAGE_BUFFER, nodeData );
	glBindBufferBase( GL_SHADER_STORAGE_BUFFER, 0, nodeData );
	glBufferStorage( GL_SHADER_STORAGE_BUFFER, bvh.usedNodes * sizeof( tinybvh::BVH_GPU::BVHNode ), bvh.bvhNode, GL_MAP_PERSISTENT_BIT|GL_MAP_COHERENT_BIT|GL_MAP_WRITE_BIT);
	glBindBuffer( GL_SHADER_STORAGE_BUFFER, idxData );
	glBindBufferBase( GL_SHADER_STORAGE_BUFFER, 1, idxData );
	glBufferStorage( GL_SHADER_STORAGE_BUFFER, bvh.idxCount * sizeof( uint32_t ), bvh.bvh.primIdx, GL_MAP_PERSISTENT_BIT|GL_MAP_COHERENT_BIT|GL_MAP_WRITE_BIT );
	glBindBuffer( GL_SHADER_STORAGE_BUFFER, triData );
	glBindBufferBase( GL_SHADER_STORAGE_BUFFER, 2, triData );
	glBufferStorage( GL_SHADER_STORAGE_BUFFER, bvh.triCount * sizeof( tinybvh::bvhvec4 ) * 3, bvh.bvh.verts.data, GL_MAP_PERSISTENT_BIT|GL_MAP_COHERENT_BIT|GL_MAP_WRITE_BIT );
	glBindBuffer( GL_SHADER_STORAGE_BUFFER, hitData );
	glBindBufferBase( GL_SHADER_STORAGE_BUFFER, 3, hitData );
	hits = (float4*)MALLOC64( SCRWIDTH * SCRHEIGHT * sizeof( float4 ) );
	glBufferStorage( GL_SHADER_STORAGE_BUFFER, SCRWIDTH * SCRHEIGHT * sizeof( float4 ), hits, GL_MAP_PERSISTENT_BIT|GL_MAP_READ_BIT  );
}

// -----------------------------------------------------------
// Main application tick function
// -----------------------------------------------------------
void Game::Tick( float /* deltaTime */ )
{
	Timer t;
	t.reset();
	glUseProgram( computeProgram );
	glDispatchCompute( SCRWIDTH / 32 * SCRHEIGHT, 1, 1 );
	// get the results
	glMemoryBarrier( GL_SHADER_STORAGE_BARRIER_BIT );
	glBindBuffer( GL_SHADER_STORAGE_BUFFER, hitData );
	float4* results = (float4*)glMapBufferRange( GL_SHADER_STORAGE_BUFFER, 0, SCRWIDTH * SCRHEIGHT * 16, GL_MAP_READ_BIT );
	memcpy( hits, results, SCRWIDTH * SCRHEIGHT * 16 );
	glUnmapBuffer( GL_SHADER_STORAGE_BUFFER );
	// display results
	for( int y = 0; y < SCRHEIGHT; y++ )
	{
		for( int x = 0; x < SCRWIDTH; x++ )
		{
			float dist = hits[x + y * SCRWIDTH].x;
			int c = (int)(255.0f * min( 1.0f, (dist * 0.1f)) ); 
			screen->pixels[x + y * SCRWIDTH] = c + (c << 8) + (c << 16);
		}
	}
	printf( "Duration: %6.1fms\n", t.elapsed() * 1000.0f );
}