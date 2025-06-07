// Template, 2024 IGAD Edition
// Get the latest version from: https://github.com/jbikker/tmpl8
// IGAD/NHTV/BUAS/UU - Jacco Bikker - 2006-2024

// In this file: implementation of various functions of the template
// math library defined in tmpl8math.h.

#include "precomp.h"

// random number generator - Marsaglia's xor32
// This is a high-quality RNG that uses a single 32-bit seed. More info:
// https://www.researchgate.net/publication/5142825_Xorshift_RNGs

// RNG seed. NOTE: in a multithreaded application, don't use a single seed!
static uint seed = 0x12345678;

// WangHash: calculates a high-quality seed based on an arbitrary non-zero
// integer. Use this to create your own seed based on e.g. thread index.
uint WangHash( uint s )
{
	s = (s ^ 61) ^ (s >> 16);
	s *= 9, s = s ^ (s >> 4);
	s *= 0x27d4eb2d;
	s = s ^ (s >> 15);
	return s;
}
uint InitSeed( uint seedBase )
{
	return WangHash( (seedBase + 1) * 17 );
}

// RandomUInt()
// Update the seed and return it as a random 32-bit unsigned int.
uint RandomUInt()
{
	seed ^= seed << 13;
	seed ^= seed >> 17;
	seed ^= seed << 5;
	return seed;
}

// RandomFloat()
// Calculate a random unsigned int and cast it to a float in the range
// [0..1)
float RandomFloat() { return RandomUInt() * 2.3283064365387e-10f; }
float Rand( float range ) { return RandomFloat() * range; }

// Calculate a random number based on a specific seed
uint RandomUInt( uint& customSeed )
{
	customSeed ^= customSeed << 13;
	customSeed ^= customSeed >> 17;
	customSeed ^= customSeed << 5;
	return customSeed;
}
float RandomFloat( uint& customSeed ) { return RandomUInt( customSeed ) * 2.3283064365387e-10f; }

// Perlin noise implementation - https://stackoverflow.com/questions/29711668/perlin-noise-generation
static int numX = 512, numY = 512, numOctaves = 7, primeIndex = 0;
static float persistence = 0.5f;
static int primes[10][3] = {
	{ 995615039, 600173719, 701464987 }, { 831731269, 162318869, 136250887 }, { 174329291, 946737083, 245679977 },
	{ 362489573, 795918041, 350777237 }, { 457025711, 880830799, 909678923 }, { 787070341, 177340217, 593320781 },
	{ 405493717, 291031019, 391950901 }, { 458904767, 676625681, 424452397 }, { 531736441, 939683957, 810651871 },
	{ 997169939, 842027887, 423882827 }
};
static float Noise( const int i, const int x, const int y )
{
	int n = x + y * 57;
	n = (n << 13) ^ n;
	const int a = primes[i][0], b = primes[i][1], c = primes[i][2];
	const int t = (n * (n * n * a + b) + c) & 0x7fffffff;
	return 1.0f - (float)t / 1073741824.0f;
}
static float SmoothedNoise( const int i, const int x, const int y )
{
	const float corners = (Noise( i, x - 1, y - 1 ) + Noise( i, x + 1, y - 1 ) + Noise( i, x - 1, y + 1 ) + Noise( i, x + 1, y + 1 )) / 16;
	const float sides = (Noise( i, x - 1, y ) + Noise( i, x + 1, y ) + Noise( i, x, y - 1 ) + Noise( i, x, y + 1 )) / 8;
	const float center = Noise( i, x, y ) / 4;
	return corners + sides + center;
}
static float Interpolate( const float a, const float b, const float x )
{
	const float ft = x * 3.1415927f, f = (1 - cosf( ft )) * 0.5f;
	return a * (1 - f) + b * f;
}
static float InterpolatedNoise( const int i, const float x, const float y )
{
	const int integer_X = (int)x, integer_Y = (int)y;
	const float fractional_X = x - integer_X, fractional_Y = y - integer_Y;
	const float v1 = SmoothedNoise( i, integer_X, integer_Y );
	const float v2 = SmoothedNoise( i, integer_X + 1, integer_Y );
	const float v3 = SmoothedNoise( i, integer_X, integer_Y + 1 );
	const float v4 = SmoothedNoise( i, integer_X + 1, integer_Y + 1 );
	const float i1 = Interpolate( v1, v2, fractional_X );
	const float i2 = Interpolate( v3, v4, fractional_X );
	return Interpolate( i1, i2, fractional_Y );
}
float noise2D( const float x, const float y )
{
	float total = 0, frequency = (float)(2 << numOctaves), amplitude = 1;
	for (int i = 0; i < numOctaves; ++i)
	{
		frequency /= 2, amplitude *= persistence;
		total += InterpolatedNoise( (primeIndex + i) % 10, x / frequency, y / frequency ) * amplitude;
	}
	return total / frequency;
}

float noise3D( const float x, const float y, const float z )
{
	float noise = 0.0f;
	float frequency = 5; // (float)(2 << numOctaves);
	float amplitude = 0.5f / 6.0f;
	for (int i = 0; i < numOctaves; ++i)
	{
		// get all permutations of noise for each individual axis
		const float noiseXY = InterpolatedNoise( i, x * frequency, y * frequency );
		const float noiseXZ = InterpolatedNoise( i, x * frequency, z * frequency );
		const float noiseYZ = InterpolatedNoise( i, y * frequency, z * frequency );
		// reverse of the permutations of noise for each individual axis
		const float noiseYX = InterpolatedNoise( i, y * frequency, x * frequency );
		const float noiseZX = InterpolatedNoise( i, z * frequency, x * frequency );
		const float noiseZY = InterpolatedNoise( i, z * frequency, y * frequency );
		// use the average of the noise functions
		noise += (noiseXY + noiseXZ + noiseYZ + noiseYX + noiseZX + noiseZY) * amplitude;
		amplitude *= persistence;
		frequency *= 2.0f;
	}
	// use the average of all octaves
	return noise;
}

// math implementations
int3::int3( const float3& a )
{
	x = (int)a.x, y = (int)a.y, z = (int)a.z;
}
uint3::uint3( const float3& a )
{
	x = (uint)a.x, y = (uint)a.y, z = (uint)a.z;
}
float4::float4( const float3& a, const float d )
{
	x = a.x, y = a.y, z = a.z;
	w = d;
}
float4::float4( const float3& a )
{
	x = a.x, y = a.y, z = a.z;
	w = 0;
}
int4::int4( const int3& a, const int d )
{
	x = a.x, y = a.y, z = a.z;
	w = d;
}
uint4::uint4( const uint3& a, const uint d )
{
	x = a.x, y = a.y, z = a.z;
	w = d;
}
mat4 operator*( const mat4& a, const mat4& b )
{
	mat4 r;
	for (uint i = 0; i < 16; i += 4)
		for (uint j = 0; j < 4; ++j)
		{
			r[i + j] =
				(a.cell[i + 0] * b.cell[j + 0]) +
				(a.cell[i + 1] * b.cell[j + 4]) +
				(a.cell[i + 2] * b.cell[j + 8]) +
				(a.cell[i + 3] * b.cell[j + 12]);
		}
	return r;
}
mat4 operator*( const mat4& a, const float s )
{
	mat4 r;
	for (uint i = 0; i < 16; i += 4) r.cell[i] = a.cell[i] * s;
	return r;
}
mat4 operator*( const float s, const mat4& a )
{
	mat4 r;
	for (uint i = 0; i < 16; i++) r.cell[i] = a.cell[i] * s;
	return r;
}
mat4 operator+( const mat4& a, const mat4& b )
{
	mat4 r;
	for (uint i = 0; i < 16; i += 4) r.cell[i] = a.cell[i] + b.cell[i];
	return r;
}
bool operator==( const mat4& a, const mat4& b )
{
	for (uint i = 0; i < 16; i++)
		if (a.cell[i] != b.cell[i]) return false;
	return true;
}
bool operator!=( const mat4& a, const mat4& b ) { return !(a == b); }
float4 operator*( const mat4& a, const float4& b )
{
	return make_float4( a.cell[0] * b.x + a.cell[1] * b.y + a.cell[2] * b.z + a.cell[3] * b.w,
		a.cell[4] * b.x + a.cell[5] * b.y + a.cell[6] * b.z + a.cell[7] * b.w,
		a.cell[8] * b.x + a.cell[9] * b.y + a.cell[10] * b.z + a.cell[11] * b.w,
		a.cell[12] * b.x + a.cell[13] * b.y + a.cell[14] * b.z + a.cell[15] * b.w );
}
float4 operator*( const float4& b, const mat4& a )
{
	return make_float4( a.cell[0] * b.x + a.cell[1] * b.y + a.cell[2] * b.z + a.cell[3] * b.w,
		a.cell[4] * b.x + a.cell[5] * b.y + a.cell[6] * b.z + a.cell[7] * b.w,
		a.cell[8] * b.x + a.cell[9] * b.y + a.cell[10] * b.z + a.cell[11] * b.w,
		a.cell[12] * b.x + a.cell[13] * b.y + a.cell[14] * b.z + a.cell[15] * b.w );
}
float3 TransformPosition( const float3& a, const mat4& M )
{
	return make_float3( make_float4( a, 1 ) * M );
}
float3 TransformVector( const float3& a, const mat4& M )
{
	return make_float3( make_float4( a, 0 ) * M );
}

// Fast matrix-vector multiplication using SSE
float3 TransformPosition_SSE( const __m128& a, const mat4& M )
{
	__m128 a4 = a;
	a4.m128_f32[3] = 1;
	__m128 v0 = _mm_mul_ps( a4, _mm_load_ps( &M.cell[0] ) );
	__m128 v1 = _mm_mul_ps( a4, _mm_load_ps( &M.cell[4] ) );
	__m128 v2 = _mm_mul_ps( a4, _mm_load_ps( &M.cell[8] ) );
	__m128 v3 = _mm_mul_ps( a4, _mm_load_ps( &M.cell[12] ) );
	_MM_TRANSPOSE4_PS( v0, v1, v2, v3 );
	__m128 v = _mm_add_ps( _mm_add_ps( v0, v1 ), _mm_add_ps( v2, v3 ) );
	return float3( v.m128_f32[0], v.m128_f32[1], v.m128_f32[2] );
}
float3 TransformVector_SSE( const __m128& a, const mat4& M )
{
	__m128 v0 = _mm_mul_ps( a, _mm_load_ps( &M.cell[0] ) );
	__m128 v1 = _mm_mul_ps( a, _mm_load_ps( &M.cell[4] ) );
	__m128 v2 = _mm_mul_ps( a, _mm_load_ps( &M.cell[8] ) );
	__m128 v3 = _mm_mul_ps( a, _mm_load_ps( &M.cell[12] ) );
	_MM_TRANSPOSE4_PS( v0, v1, v2, v3 );
	__m128 v = _mm_add_ps( _mm_add_ps( v0, v1 ), v2 );
	return float3( v.m128_f32[0], v.m128_f32[1], v.m128_f32[2] );
}

// 16-bit floats
static uint as_uint( const float x ) { return *(uint*)&x; }
float as_float( const uint x ) { return *(float*)&x; }
float half_to_float( const half x )
{
	const uint e = (x & 0x7C00) >> 10, m = (x & 0x03FF) << 13, v = as_uint( (float)m ) >> 23;
	return as_float( (x & 0x8000) << 16 | (e != 0) * ((e + 112) << 23 | m) | ((e == 0) & (m != 0)) * ((v - 37) << 23 | ((m << (150 - v)) & 0x007FE000)) );
}
half float_to_half( const float x )
{
	const uint b = as_uint( x ) + 0x00001000, e = (b & 0x7F800000) >> 23, m = b & 0x007FFFFF;
	return (half)((b & 0x80000000) >> 16 | (e > 112) * ((((e - 112) << 10) & 0x7C00) | m >> 13) | ((e < 113) & (e > 101)) * ((((0x007FF000 + m) >> (125 - e)) + 1) >> 1) | (e > 143) * 0x7FFF); // sign : normalized : denormalized : saturate
}