// Template, 2024 IGAD Edition
// Get the latest version from: https://github.com/jbikker/tmpl8
// IGAD/NHTV/BUAS/UU - Jacco Bikker - 2006-2024

#include "precomp.h"

extern bool IGP_detected;

// OpenGL helper functions
void _CheckGL( const char* f, int l )
{
	GLenum error = glGetError();
	if (error != GL_NO_ERROR)
	{
		const char* errStr = "UNKNOWN ERROR";
		if (error == 0x500) errStr = "INVALID ENUM";
		else if (error == 0x502) errStr = "INVALID OPERATION";
		else if (error == 0x501) errStr = "INVALID VALUE";
		else if (error == 0x506) errStr = "INVALID FRAMEBUFFER OPERATION";
		FatalError( "GL error %d: %s at %s:%d\n", error, errStr, f, l );
	}
}

GLuint CreateVBO( const GLfloat* data, const uint size )
{
	GLuint id;
	glGenBuffers( 1, &id );
	glBindBuffer( GL_ARRAY_BUFFER, id );
	glBufferData( GL_ARRAY_BUFFER, size, data, GL_STATIC_DRAW );
	CheckGL();
	return id;
}

void BindVBO( const uint idx, const uint N, const GLuint id )
{
	glEnableVertexAttribArray( idx );
	glBindBuffer( GL_ARRAY_BUFFER, id );
	glVertexAttribPointer( idx, N, GL_FLOAT, GL_FALSE, 0, (void*)0 );
	CheckGL();
}

void CheckShader( GLuint shader )
{
	char buffer[1024];
	memset( buffer, 0, sizeof( buffer ) );
	GLsizei length = 0;
	glGetShaderInfoLog( shader, sizeof( buffer ), &length, buffer );
	CheckGL();
	FATALERROR_IF( length > 0 && strstr( buffer, "ERROR" ), "Shader compile error:\n%s", buffer );
}

void CheckProgram( GLuint id )
{
	char buffer[1024];
	memset( buffer, 0, sizeof( buffer ) );
	GLsizei length = 0;
	glGetProgramInfoLog( id, sizeof( buffer ), &length, buffer );
	CheckGL();
	FATALERROR_IF( length > 0, "Shader link error:\n%s", buffer );
}

void DrawQuad()
{
	static GLuint vao = 0;
	if (!vao)
	{
		// generate buffers
		static const GLfloat verts[] = { -1, 1, 1, 1, -1, -1, 1, 1, -1, -1, 1, -1 };
		GLuint vbo = CreateVBO( verts, sizeof( verts ) );
		glGenVertexArrays( 1, &vao );
		glBindVertexArray( vbo );
		glEnableVertexAttribArray( 0 );
		glBindBuffer( GL_ARRAY_BUFFER, vbo );
		glVertexAttribPointer( 0, 2, GL_FLOAT, GL_FALSE, 0, NULL );
		glBindVertexArray( 0 );
		CheckGL();
	}
	glViewport( 0, 0, SCRWIDTH, SCRHEIGHT );
	glBindVertexArray( vao );
	glDrawArrays( GL_TRIANGLES, 0, 6 );
	glBindVertexArray( 0 );
}

// OpenGL texture wrapper class
GLTexture::GLTexture( uint w, uint h, uint type )
{
	width = w, height = h;
	glGenTextures( 1, &ID );
	glBindTexture( GL_TEXTURE_2D, ID );
	if (type == DEFAULT)
	{
		// regular texture
		glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_BGR, GL_UNSIGNED_BYTE, 0 );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
	}
	else if (type == INTTARGET)
	{
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
		glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0 );
	}
	else /* type == FLOAT */
	{
		// floating point texture
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
		glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA32F, width, height, 0, GL_RGBA, GL_FLOAT, 0 );
	}
	glBindTexture( GL_TEXTURE_2D, 0 );
	CheckGL();
}

GLTexture::~GLTexture()
{
	glDeleteTextures( 1, &ID );
	CheckGL();
}

void GLTexture::Bind( const uint slot )
{
	glActiveTexture( GL_TEXTURE0 + slot );
	glBindTexture( GL_TEXTURE_2D, ID );
	CheckGL();
}

void GLTexture::CopyFrom( Surface* src )
{
	glBindTexture( GL_TEXTURE_2D, ID );
	glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_BGRA, GL_UNSIGNED_BYTE, src->pixels );
	CheckGL();
}

void GLTexture::CopyTo( Surface* dst )
{
	glBindTexture( GL_TEXTURE_2D, ID );
	glGetTexImage( GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, dst->pixels );
	CheckGL();
}

// Shader class implementation
Shader::Shader( const char* vfile, const char* pfile, bool fromString )
{
	if (fromString)
	{
		Compile( vfile, pfile );
	}
	else
	{
		Init( vfile, pfile );
	}
}

Shader::~Shader()
{
	glDetachShader( ID, pixel );
	glDetachShader( ID, vertex );
	glDeleteShader( pixel );
	glDeleteShader( vertex );
	glDeleteProgram( ID );
	CheckGL();
}

void Shader::Init( const char* vfile, const char* pfile )
{
	string vsText = TextFileRead( vfile );
	string fsText = TextFileRead( pfile );
	FATALERROR_IF( vsText.size() == 0, "File %s not found", vfile );
	FATALERROR_IF( fsText.size() == 0, "File %s not found", pfile );
	const char* vertexText = vsText.c_str();
	const char* fragmentText = fsText.c_str();
	Compile( vertexText, fragmentText );
}

void Shader::Compile( const char* vtext, const char* ftext )
{
	vertex = glCreateShader( GL_VERTEX_SHADER );
	pixel = glCreateShader( GL_FRAGMENT_SHADER );
	glShaderSource( vertex, 1, &vtext, 0 );
	glCompileShader( vertex );
	CheckShader( vertex );
	glShaderSource( pixel, 1, &ftext, 0 );
	glCompileShader( pixel );
	CheckShader( pixel );
	ID = glCreateProgram();
	glAttachShader( ID, vertex );
	glAttachShader( ID, pixel );
	glBindAttribLocation( ID, 0, "pos" );
	glBindAttribLocation( ID, 1, "tuv" );
	CheckGL();
	glLinkProgram( ID );
	CheckProgram( ID );
	CheckGL();
}

void Shader::Bind()
{
	glUseProgram( ID );
	CheckGL();
}

void Shader::Unbind()
{
	glUseProgram( 0 );
	CheckGL();
}

void Shader::SetInputTexture( uint slot, const char* name, GLTexture* texture )
{
	glActiveTexture( GL_TEXTURE0 + slot );
	glBindTexture( GL_TEXTURE_2D, texture->ID );
	glUniform1i( glGetUniformLocation( ID, name ), slot );
	CheckGL();
}

void Shader::SetInputMatrix( const char* name, const mat4& matrix )
{
	const GLfloat* data = (const GLfloat*)&matrix;
	glUniformMatrix4fv( glGetUniformLocation( ID, name ), 1, GL_FALSE, data );
	CheckGL();
}

void Shader::SetFloat( const char* name, const float v )
{
	glUniform1f( glGetUniformLocation( ID, name ), v );
	CheckGL();
}

void Shader::SetInt( const char* name, const int v )
{
	glUniform1i( glGetUniformLocation( ID, name ), v );
	CheckGL();
}

void Shader::SetUInt( const char* name, const uint v )
{
	glUniform1ui( glGetUniformLocation( ID, name ), v );
	CheckGL();
}