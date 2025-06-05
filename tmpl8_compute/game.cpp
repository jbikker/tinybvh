// Template, 2024 IGAD Edition
// Get the latest version from: https://github.com/jbikker/tmpl8
// IGAD/NHTV/BUAS/UU - Jacco Bikker - 2006-2024

#include "precomp.h"
#include "game.h"

// -----------------------------------------------------------
// Initialize the application
// -----------------------------------------------------------
void Game::Init()
{
	int compute = glCreateShader( GL_COMPUTE_SHADER );
#ifdef BASIC_SHADER
	const string shaderSource = TextFileRead( "shaders/compute.comp" );
#else
	const string shaderSource = TextFileRead( "shaders/voronoi.comp" );
#endif
	const char* source = shaderSource.c_str(); // any other way to do this?
	glShaderSource( compute, 1, &source, 0 );
	glCompileShader( compute );
	// check if the compilation was succesfull - totally optional if we didn't make mistakes
	int result, logSize = 0;
	char log[16384];
	glGetShaderiv( compute, GL_COMPILE_STATUS, &result );
	if (result == GL_FALSE)
	{
		// compilation didn't succeed; request an error log
		glGetShaderInfoLog( compute, sizeof( log ), &logSize, log );
		printf( "%s\n", log );
		exit( 0 /* that's graceful application shutdown */ );
	}
	// create the program
	computeProgram = glCreateProgram();
	glAttachShader( computeProgram, compute );
	glLinkProgram( computeProgram );
	// check the program
	glGetProgramiv( computeProgram, GL_LINK_STATUS, &result );
	if (result == GL_FALSE)
	{
		// linking was problematic; report
		glGetProgramInfoLog( computeProgram, sizeof( log ), &logSize, log );
		printf( "%s\n", log );
		exit( 0 /* like I said, graceful */ );
	}
	// prepare some data for the compute shader
	uint inputBuffer;
	uint numbers[10] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
	glGenBuffers( 1, &inputBuffer );
	glBindBuffer( GL_SHADER_STORAGE_BUFFER, inputBuffer );
	glBufferData( GL_SHADER_STORAGE_BUFFER, sizeof( uint ) * 10, (void*)numbers, GL_STREAM_COPY );
	glBindBufferBase( GL_SHADER_STORAGE_BUFFER, 1, inputBuffer );
	uint outputBuffer;
	glGenBuffers( 0, &outputBuffer );
	glBindBuffer( GL_SHADER_STORAGE_BUFFER, outputBuffer );
	glBindBufferBase( GL_SHADER_STORAGE_BUFFER, 0, outputBuffer );
	glBufferData( GL_SHADER_STORAGE_BUFFER, sizeof( GLuint ) * 10, nullptr, GL_DYNAMIC_READ );
	// run the shader
	glUseProgram( computeProgram );
	glDispatchCompute( 10, 1, 1 );
	// get the results
	glMemoryBarrier( GL_SHADER_STORAGE_BARRIER_BIT );
	uint* results = (uint*)glMapBufferRange( GL_SHADER_STORAGE_BUFFER, 0, sizeof( GLuint ) * 10, GL_MAP_READ_BIT );
	printf( "First result: %i\n", results[0] );
	glUnmapBuffer( GL_SHADER_STORAGE_BUFFER );
}

// -----------------------------------------------------------
// Main application tick function - Executed once per frame
// -----------------------------------------------------------
void Game::Tick( float /* deltaTime */ )
{
}