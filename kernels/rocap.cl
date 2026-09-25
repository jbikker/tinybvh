// ----------------------------------------------------------------------------
// Roving Capsules - Reshetov & Hart, SIGGRAPH 2024
// https://doi.org/10.1145/3641519.3657450
//
// A strand is a cubic B-spline through hairVerts[offset .. offset + N - 1],
// where .xyz is position and .w is radius. Each 4-point window is one segment,
// which is converted into the "dcurve" (dc) basis used by the paper and then
// intersected with roving capsules.
//
// dc basis (see position()/velocity() below):
//   C(u) = c0 + (u^3/6 + (u - u^2)/2) c1 + (u^2 - 2u^3/3) c2 + (u^3/6) c3
// so c0 = C(0), c1 = 2 C'(0), c3 = 2 C'(1), and c2 is the middle Bernstein
// coefficient of C'. Converting a uniform cubic B-spline b0..b3 is nearly free:
//   c0 = (b0 + 4 b1 + b2) / 6,  c1 = b2 - b0,  c2 = b2 - b1,  c3 = b3 - b1
// (verified exact; the same weights apply to the radius in .w).
//
// Direct translation from the Shadertoy code at:
// https://www.shadertoy.com/view/4ffXWs
// ----------------------------------------------------------------------------

// 0: adaptive roving capsules. > 0: uniform capsules with h = 1 / ROCAP_SPLITS.
#define ROCAP_SPLITS		0

// Duplicated control points at each end. 1: phantom ends (N-1 segments, curve
// starts at (5 b0 + b1) / 6). 2: clamped ends (N+1 segments, curve interpolates
// b0 and bN-1, at the cost of a zero-velocity segment at each tip).
#define ROCAP_END_DUP		1

// Safety net; the loop advances by at least 0.1 * minh = 0.002 per iteration.
#define ROCAP_MAX_STEPS		512

// Evaluate a dc curve (xyz = position, w = radius).
float4 rocap_position( const float4 c0, const float4 c1, const float4 c2, const float4 c3, const float u )
{
	const float uu = u * u, u3 = (1.0f / 6.0f) * uu * u;
	return c0 + (u3 + 0.5f * (u - uu)) * c1 + (uu - 4.0f * u3) * c2 + u3 * c3;
}

// Evaluate a scalar dc cubic packed as (c0, c1, c2, c3).
float rocap_position1( const float4 q, const float u )
{
	const float uu = u * u, u3 = (1.0f / 6.0f) * uu * u;
	return q.x + (u3 + 0.5f * (u - uu)) * q.y + (uu - 4.0f * u3) * q.z + u3 * q.w;
}

float4 rocap_velocity( const float4 c1, const float4 c2, const float4 c3, const float u )
{
	const float v = 1.0f - u;
	return 0.5f * v * v * c1 + 2.0f * v * u * c2 + 0.5f * u * u * c3;
}

float rocap_L1( const float4 v ) { return fabs( v.x ) + fabs( v.y ) + fabs( v.z ); }

// Capsule length in curve parameter space.
float rocap_capsule_step( const float4 c0, const float4 c1, const float4 c2, const float4 c3, const float rdlen )
{
#if ROCAP_SPLITS > 0
	return 1.0f / (float)ROCAP_SPLITS;
#else
	const float minh = 1.0f / 50.0f, maxh = 1.0f / 4.0f;
	const float vel = rocap_L1( c1 );
	if (!(vel > 1e-20f)) return maxh; // degenerate segment; avoid 0/0
	const float acc = rocap_L1( 2.0f * c2 - c1 );
	const float jerk = rocap_L1( c1 + c3 - 4.0f * c2 ) * (1.0f / 3.0f);
	const float rad = fabs( 2.0f * c2.w - c1.w );
	const float avgr = c0.w + c1.w * 0.125f + c2.w * (1.0f / 6.0f) + c3.w * (1.0f / 24.0f);
	float du = vel / (acc + jerk);
	du *= fmin( 1.0f, 0.5f / rad );
	du = minh * (1.0f + du);
	du *= clamp( rdlen / vel, 0.05f, 3.0f );
	du = fmin( du, 20.0f * avgr / vel );
	return fmax( fmin( du, maxh ), 0.1f * minh );
#endif
}

// Ray/capsule test in ray-centric coords: ray is O = (0,0,0), D = (0,0,1).
// pa, pb are (x, y, z, radius) endpoints. Returns t along the ray and u in [0,1].
bool rocap_capsule_intersection( const float4 pa, const float4 pb, const float ray_tmax, float* t, float* u )
{
	const float2 Q = pa.xy;					// pa - ray origin
	const float2 A = pb.xy - Q;				// axis vector
	const float dr = pb.w - pa.w;
	const float au_e = dr * dr - dot( A, A );
	const float Az = pb.z - pa.z;
	const float e = Az * Az - au_e;
	const float f = dot( A, Q ) - pa.w * dr;
	const float g = dot( Q, Q ) - pa.w * pa.w;
	const float udisc = (f * f + g * au_e) / e;
	if (udisc < 0.0f) return false;
	const float squdisc = Az * sqrt( udisc );
	float u0 = (f + squdisc) / au_e;
	float u1 = (f - squdisc) / au_e;
	if (u0 * dr < -pa.w) u0 = 1.0f - u0;	// flip negative caps to the other side
	if (u1 * dr < -pa.w) u1 = 1.0f - u1;
	const float umin = clamp( ((Az >= 0.0f) != (u1 > u0)) ? u1 : u0, 0.0f, 1.0f );
	const float tdisc = umin * (umin * au_e - 2.0f * f) - g;
	if (tdisc < 0.0f) return false;
	const float tc = Az * umin - sqrt( tdisc ) + pa.z; // pa.z compensates for Q.z
	*t = tc, *u = umin;
	return (tc > 0.0f && tc <= ray_tmax);
}

float rocap_cubic_value( const float4 q, const float u ) { return q.x + u * (q.y + u * (q.z + q.w * u)); }
float rocap_cubic_velocity( const float4 q, const float u ) { return q.y + u * (2.0f * q.z + 3.0f * u * q.w); }
float rocap_cubic_acceleration( const float4 q, const float u ) { return 2.0f * q.z + 6.0f * u * q.w; }

// Conservative estimate of the first root of a cubic on [us, ue].
float rocap_conservative_root( const float4 cubic, const float us, const float ue )
{
	float f0 = rocap_cubic_value( cubic, us );
	float f1 = rocap_cubic_velocity( cubic, us );
	const bool convex = rocap_cubic_acceleration( cubic, us ) > 0.0f;
	float inflection = (-1.0f / 3.0f) * cubic.z / cubic.w;
	const bool consistent = inflection < us || inflection > ue;
	if (convex && f1 < 0.0f)
	{
		const float un = us - f0 / f1;
		if (consistent || un < inflection) return un;
	}
	else if (consistent)
	{
		if (convex) return ue;
		inflection = ue;
	}
	float ul, ur, fi, fl, fr;
	fi = rocap_cubic_value( cubic, inflection );
	if (fi < 0.0f) ul = us, ur = inflection, fl = f0, fr = fi;
	else if (consistent) return ue;
	else if (!convex)
	{
		f1 = cubic.y + cubic.z * inflection;
		if (f1 > 0.0f) return ue;
		return inflection - fi / f1;
	}
	else
	{
		fr = rocap_cubic_value( cubic, ue );
		if (fr > 0.0f) return ue;
		ul = inflection, ur = ue, fl = fi;
	}
	// secant( ul, Newton( ur ) )
	ur -= fr / rocap_cubic_velocity( cubic, ur );
	fr = rocap_cubic_value( cubic, ur );
	return (fr * ul - fl * ur) / (fr - fl);
}

// Skip the part of [us, ue] that provably cannot be hit by the ray.
float rocap_tighten_interval( const float4* p, const float4 sn, const float us, const float ue )
{
	const float nn = sn.x * sn.x + sn.y * sn.y;
	if (nn <= sn.w * sn.w) return us;	// the ray intersects sphere( us )
	const float nl = rsqrt( nn );		// g( u ) = dot( position( u ), n )
	const float4 n = (float4)( sn.x * nl, sn.y * nl, 0.0f, -1.0f );
	const float p1n = dot( p[1], n ), p2n = dot( p[2], n ), p3n = dot( p[3], n );
	float4 cubic;						// convert dc layout to monomial
	cubic.x = dot( p[0], n );
	cubic.y = p1n * 0.5f;
	cubic.z = p2n - cubic.y;
	cubic.w = (p1n - 4.0f * p2n + p3n) * (1.0f / 6.0f);
	return rocap_conservative_root( cubic, us, ue );
}

// Root of f' where f'' > 0, i.e. the parameter at which the cubic is minimal.
float rocap_min_root( const float4 px )
{
	const float a = px.y;				// f' = a + 2 b u - c u^2
	const float b = 2.0f * px.z - a;
	const float c = 2.0f * px.z + b - px.w;
	float det = b * b + a * c;
	if (det <= 0.0f) return -1.0f;
	det = sqrt( det );
	if (a < 0.0f) det = -det;
	det = a == 0.0f ? -b : (c == 0.0f ? b : det);
	const float root1 = (b * det <= 0.0f) ? (b - det) / c : -a / (b + det);
	const float root2 = -a / (root1 * c); // Vieta
	return det > 0.0f ? root1 : root2;
}

// Orthogonal to rd, assuming dot( rd, rd ) == 1. http://jcgt.org/published/0006/01/01/
float3 rocap_ort1( const float3 rd )
{
	const float sgn = rd.z >= 0.0f ? 1.0f : -1.0f;
	const float a = 1.0f / (sgn + rd.z);
	const float b = a * rd.x * rd.y;
	return (float3)( a * rd.x * rd.x - sgn, b, rd.x );
}

// Separating plane cull. Returns true on a guaranteed miss; otherwise fills
// prcc with the dc control points expressed in ray-centric coordinates.
bool rocap_separated( const float3 rd, const float us, const float ue,
	const float4 c0, const float4 c1, const float4 c2, const float4 c3, float4* prcc )
{
	const float3 base = rocap_velocity( c1, c2, c3, 0.5f ).xyz;
	const float3 pm = rocap_position( c0, c1, c2, c3, 0.5f ).xyz;
	// scale-invariant: |cross( rd, base )| / |base| is the sine of the angle
	const float bl = fmax( length( base ), 1e-30f );
	float3 sn = cross( rd, base ) * (1.0f / bl);
	float sl = length( sn );
	if (sl < 0.001f) sn = rocap_ort1( rd ), sl = 1.0f;
	sn *= 1.0f / sl;
	if (dot( pm, sn ) < 0.0f) sn = -sn;
	const float4 pxc = (float4)( dot( sn, c0.xyz ), dot( sn, c1.xyz ), dot( sn, c2.xyz ), dot( sn, c3.xyz ) );
	const float4 radii = (float4)( c0.w, c1.w, c2.w, c3.w );
	const float4 pxw = pxc - radii;
	if (rocap_position1( pxw, us ) >= 0.0f && rocap_position1( pxw, ue ) >= 0.0f)
	{
		const float xroot = rocap_min_root( pxw );
		if (xroot < us || xroot > ue || rocap_position1( pxw, xroot ) >= 0.0f) return true;
	}
	// basis is ( sn, cross( rd, sn ), rd ): orthonormal and right-handed
	const float3 axisy = cross( rd, sn );
	prcc[0] = (float4)( pxc.x, dot( axisy, c0.xyz ), dot( rd, c0.xyz ), radii.x );
	prcc[1] = (float4)( pxc.y, dot( axisy, c1.xyz ), dot( rd, c1.xyz ), radii.y );
	prcc[2] = (float4)( pxc.z, dot( axisy, c2.xyz ), dot( rd, c2.xyz ), radii.z );
	prcc[3] = (float4)( pxc.w, dot( axisy, c3.xyz ), dot( rd, c3.xyz ), radii.w );
	return false;
}

// Trace one dc segment, given ray-relative control points. Returns true and
// updates *tbest / *ubest if a closer hit than *tbest was found.
bool rocap_trace_segment( const float3 D, const float4 c0, const float4 c1, const float4 c2, const float4 c3,
	float* tbest, float* ubest )
{
	const float us = 0.0f, ue = 1.0f;
	float4 prcc[4];
	if (rocap_separated( D, us, ue, c0, c1, c2, c3, prcc )) return false;
	const float h = rocap_capsule_step( c0, c1, c2, c3, 1.0f );
	float u0, u1 = us;
	float4 p0, p1 = rocap_position( prcc[0], prcc[1], prcc[2], prcc[3], u1 );
	bool found = false;
	int steps = 0;
	do // walk the unresolved subsegments of [us, ue]
	{
		u0 = u1, p0 = p1;
	#if ROCAP_SPLITS == 0
		const float un = rocap_tighten_interval( prcc, p0, u0, ue );
		if (un > u0)
		{
			if (un >= ue) break;			// no more intersections possible
			u0 = fmin( un, ue - h );		// avoid slim capsules
			p0 = rocap_position( prcc[0], prcc[1], prcc[2], prcc[3], u0 );
		}
	#endif
		u1 = fmin( ue, u0 + h );			// [u0, u1] defines the capsule
		p1 = rocap_position( prcc[0], prcc[1], prcc[2], prcc[3], u1 );
		float uc, tc;
		if (rocap_capsule_intersection( p0, p1, *tbest, &tc, &uc ))
			*tbest = tc, *ubest = u0 * (1.0f - uc) + u1 * uc, found = true;
	}
	while (u1 < ue && ++steps < ROCAP_MAX_STEPS);
	return found;
}

// Fetch the four B-spline control points of segment s and convert them to dc.
// Indices are clamped, which duplicates the end points ROCAP_END_DUP times.
void rocap_segment_dc( const global float4* v, const int N, const int s, float4* c )
{
	const int last = N - 1;
	const float4 b0 = v[min( max( s - ROCAP_END_DUP, 0 ), last )];
	const float4 b1 = v[min( max( s + 1 - ROCAP_END_DUP, 0 ), last )];
	const float4 b2 = v[min( max( s + 2 - ROCAP_END_DUP, 0 ), last )];
	const float4 b3 = v[min( max( s + 3 - ROCAP_END_DUP, 0 ), last )];
	c[0] = (b0 + 4.0f * b1 + b2) * (1.0f / 6.0f);
	c[1] = b2 - b0;						// c1..c3 are differences, so they are
	c[2] = b2 - b1;						// translation invariant: only c0 needs
	c[3] = b3 - b1;						// the ray origin subtracted from it
}

int rocap_segment_count( const int N ) { return N - 3 + 2 * ROCAP_END_DUP; }

// Roving Capsules entry point.
// If a closer hit is found, *hit is overwritten with:
//   .x = ray t, 
//   .y = u in [0,1] within the segment, 
//   .z = segment index,
//   .w = strand index. 
void eval_rocap( const uint hairIdx, const float3 O, const float3 D, float4* hit, const global struct Strand* hairs, const global float4* hairVerts )
{
	const struct Strand strand = hairs[hairIdx];
	const int N = (int)strand.N;
	if (N < 2) return;					// need at least one segment
	const global float4* v = hairVerts + strand.offset;
	const int segCount = rocap_segment_count( N );
	for (int s = 0; s < segCount; s++)
	{
		float4 c[4];
		rocap_segment_dc( v, N, s, c );
		c[0].xyz -= O;					// make the segment ray-relative
		float t = hit->x, u;
		if (rocap_trace_segment( D, c[0], c[1], c[2], c[3], &t, &u ))
			*hit = (float4)( t, u, as_float( (uint)s ), as_float( hairIdx ) );
	}
}

// ---- shading -----------------------------------------------------------------
// Optional: recover the surface normal at a hit recorded by eval_rocap.

// dot( raypos - curve( u ), curve'( u ) ) + r r', zero on the swept surface.
float rocap_capsule_error( const float4* c, const float3 raypos, const float u )
{
	const float4 c04 = rocap_position( c[0], c[1], c[2], c[3], u );
	const float4 c14 = rocap_velocity( c[1], c[2], c[3], u );
	return dot( raypos - c04.xyz, c14.xyz ) + c04.w * c14.w;
}

// The capsule hit is linearized; nudge u onto the true swept surface.
float rocap_capsule_tune( const float4* c, const float3 raypos, const float us )
{
	float u0 = us, u1, f0 = 0.0f, f1;
	float h = 0.0f;						// u1 == u0 on the first iteration
	for (int i = 0; i <= 3; i++)
	{
		u1 = u0 + h;
		f1 = rocap_capsule_error( c, raypos, u1 );
		if (h == 0.0f)
		{
			h = 0.1f * rocap_capsule_step( c[0], c[1], c[2], c[3], 1.0f );
			if (f1 < 0.0f) h = -h;
		}
		else if ((f1 > 0.0f) != (f0 > 0.0f))
		{
			return clamp( (f0 * u1 - f1 * u0) / (f0 - f1), 0.0f, 1.0f );
		}
		else if (fabs( f1 ) > fabs( f0 )) return us; // diverging, keep the original
		u0 = u1, f0 = f1;
	}
	return u0;
}

// Returns the normalized shading normal for the hit produced by eval_rocap.
float3 eval_rocap_normal( const float4 hit, const float3 O, const float3 D, const global struct Strand* hairs, const global float4* hairVerts )
{
	const struct Strand strand = hairs[as_uint( hit.w )];
	const global float4* v = hairVerts + strand.offset;
	float4 c[4];
	rocap_segment_dc( v, (int)strand.N, (int)as_uint( hit.z ), c );
	const float3 raypos = O + hit.x * D;
	float u = hit.y;
	if (u != 0.0f && u != 1.0f) u = rocap_capsule_tune( c, raypos, u );
	return normalize( raypos - rocap_position( c[0], c[1], c[2], c[3], u ).xyz );
}