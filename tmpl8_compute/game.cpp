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
	// create a sweet compute shader, based on stackoverflow.com/questions/51245319
	// please note the alternative includes in template.h!
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
	// prepare output buffer for voronoi noise computation
	glUseProgram( computeProgram );
	glGenBuffers( 1, &proceduralData );
	glBindBuffer( GL_SHADER_STORAGE_BUFFER, proceduralData );
	glBindBufferBase( GL_SHADER_STORAGE_BUFFER, 0, proceduralData );
	glBufferData( GL_SHADER_STORAGE_BUFFER, SCRWIDTH * SCRHEIGHT * 4, nullptr, GL_DYNAMIC_READ );
	glUniform2i( glGetUniformLocation( computeProgram, "screenSize" ), SCRWIDTH, SCRHEIGHT );
	screen->Clear( 0 );
}

// -----------------------------------------------------------
// Main application tick function
// -----------------------------------------------------------
void Game::Tick( float deltaTime )
{
	Timer t;
	t.reset();
	glUseProgram( computeProgram );
	glDispatchCompute( SCRWIDTH / 32, SCRHEIGHT, 1 );
	// get the results
	glMemoryBarrier( GL_SHADER_STORAGE_BARRIER_BIT );
	uint* results = (uint*)glMapBufferRange( GL_SHADER_STORAGE_BUFFER, 0, SCRWIDTH * SCRHEIGHT * 4, GL_MAP_READ_BIT );
	for( int y = 0; y < SCRHEIGHT; y++ )
	{
		for( int x = 0; x < SCRWIDTH; x++ )
		{
			int v = results[x + y * SCRWIDTH] & 255;
			screen->pixels[x + y * SCRWIDTH] = v + (v << 8) + (v << 16);
		}
	}
	glUnmapBuffer( GL_SHADER_STORAGE_BUFFER );
	printf( "Duration: %6.1fms\n", t.elapsed() * 1000.0f );
}