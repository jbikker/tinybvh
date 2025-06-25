// Common constants
#define PI			3.14159265358979323846264f
#define INVPI		0.31830988618379067153777f
#define INV2PI		0.15915494309189533576888f
#define TWOPI		6.28318530717958647692528f
#define EPSILON		0.0001f // for a 100^3 world

// Xor32 RNG
uint WangHash( uint s ) { s = (s ^ 61) ^ (s >> 16), s *= 9, s = s ^ (s >> 4), s *= 0x27d4eb2d; return s ^ (s >> 15); }
uint RandomUInt( uint* seed ) { *seed ^= *seed << 13, * seed ^= *seed >> 17, * seed ^= *seed << 5; return *seed; }
float RandomFloat( uint* seed ) { return RandomUInt( seed ) * 2.3283064365387e-10f; }

// Color conversion
float3 rgb32_to_vec3( uint c )
{
	return (float3)((float)((c >> 16) & 255), (float)((c >> 8) & 255), (float)(c & 255)) * 0.00392f;
}

// Specular reflection
float3 Reflect( const float3 D, const float3 N ) { return D - 2.0f * N * dot( N, D ); }

// DiffuseReflection: Uniform random bounce in the hemisphere
float3 DiffuseReflection( float3 N, uint* seed )
{
	float3 R;
	do
	{
		R = (float3)(RandomFloat( seed ) * 2 - 1, RandomFloat( seed ) * 2 - 1, RandomFloat( seed ) * 2 - 1);
	} while (dot( R, R ) > 1);
	return fast_normalize( dot( R, N ) > 0 ? R : -R );
}

// CosWeightedDiffReflection: Cosine-weighted random bounce in the hemisphere
float3 CosWeightedDiffReflection( const float3 N, const float r0, const float r1 )
{
	const float r = sqrt( 1 - r1 * r1 ), phi = 4 * PI * r0;
	const float3 R = (float3)( cos( phi ) * r, sin( phi ) * r, r1);
	return fast_normalize( N + R );
}

// TransformPoint - apply a 4x4 matrix transform to a 3D position
float3 TransformPoint( const float3 v, const float* T )
{
	const float3 res = (float3)(
		T[0] * v.x + T[1] * v.y + T[2] * v.z + T[3],
		T[4] * v.x + T[5] * v.y + T[6] * v.z + T[7],
		T[8] * v.x + T[9] * v.y + T[10] * v.z + T[11] 
	);
	const float w = T[12] * v.x + T[13] * v.y + T[14] * v.z + T[15];
	if (w == 1.0f) return res; else return res * (1.0f / w);
}

// TransformVector - apply a 4x4 matrix transform to a 3D vector.
float3 TransformVector( const float3 v, const float* T )
{
	return (float3)( T[0] * v.x + T[1] * v.y + T[2] * v.z, T[4] * v.x +
		T[5] * v.y + T[6] * v.z, T[8] * v.x + T[9] * v.y + T[10] * v.z );
}

// AGX tone mapping - From: https://github.com/sobotka/AgX
float3 agxDefaultContrastApprox(float3 x) 
{
  float3 y = x * x, z = y * y;
  return 15.5 * z * y - 40.14f * z * x + 31.96f * z - 6.868f * y * x + 0.4298f * y + 0.1191 * x - 0.00232f;
}
float3 agx( float3 val ) 
{
	float3 mx = (float3)(0.842479062253094f, 0.0423282422610123f, 0.0423756549057051f);
	float3 my = (float3)(0.0784335999999992f,  0.878468636469772f,  0.0784336f);
	float3 mz = (float3)(0.0792237451477643f, 0.0791661274605434f, 0.879142973793104f);
	const float min_ev = -12.47393f, max_ev = 4.026069f;
	val = (float3)( dot( mx, val ), dot( my, val ), dot( mz, val ) );
	val = clamp( log2( val ), min_ev, max_ev );
	val = (val - min_ev) / (max_ev - min_ev);
	return agxDefaultContrastApprox( val );
}

float3 unreal( float3 c ) 
{
    return (float3)( 
		c.x / (c.x + 0.155f) * 1.019f,
		c.y / (c.y + 0.155f) * 1.019f,
		c.z / (c.z + 0.155f) * 1.019f
	);
}

float3 aces(float3 x) 
{
    // Narkowicz 2015, "ACES Filmic Tone Mapping Curve"
    const float a = 2.51f;
    const float b = 0.03f;
    const float c = 2.43f;
    const float d = 0.59f;
    const float e = 0.14f;
    return (x * (a * x + b)) / (x * (c * x + d) + e);
}

float3 inferno_quintic( float x ) // https://www.shadertoy.com/view/XtGGzG
{
	x = min( 1.0f, x );
	float4 x1 = (float4)( 1, x, x * x, x * x * x ), x2 = x1 * x1.w * x;
	return (float3)(
		dot( x1, (float4)( -0.027780558, +1.228188385, +0.278906882, +3.892783760 ) ) + dot( x2.xy, (float2)( -8.490712758, +4.069046086 ) ),
		dot( x1, (float4)( +0.014065206, +0.015360518, +1.605395918, -4.821108251 ) ) + dot( x2.xy, (float2)( +8.389314011, -4.193858954 ) ),
		dot( x1, (float4)( -0.019628385, +3.122510347, -5.893222355, +2.798380308 ) ) + dot( x2.xy, (float2)( -3.608884658, +4.324996022 ) ) 
	);
}

float3 plasma_quintic( float x )
{
	x = min( 1.0f, x );
	float4 x1 = (float4)( 1, x, x * x, x * x * x ), x2 = x1 * x1.w * x;
	return (float3)(
		dot( x1, (float4)( +0.063861086, +1.992659096, -1.023901152, -0.490832805 ) ) + dot( x2.xy, (float2)( +1.308442123, -0.914547012 ) ),
		dot( x1, (float4)( +0.049718590, -0.791144343, +2.892305078, +0.811726816 ) ) + dot( x2.xy, (float2)( -4.686502417, +2.717794514 ) ),
		dot( x1, (float4)( +0.513275779, +1.580255060, -5.164414457, +4.559573646 ) ) + dot( x2.xy, (float2)( -1.916810682, +0.570638854 ) ) 
	);
}