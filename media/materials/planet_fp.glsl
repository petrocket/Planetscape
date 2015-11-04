// continents
uniform int numContinents;
uniform float continentSeed;
// islands
uniform int numIslands;
uniform float islandSeed;
// Ogre doesn't support arrays of structures :(

// noise layers
#define MAX_NOISE_LAYERS 4
#define BLEND_TYPE_ADD 0
#define BLEND_TYPE_MUL 1
#define NOISE_TYPE_PUFFY 0
#define NOISE_TYPE_SPECKLED 1
#define NOISE_TYPE_TURBULENCE 2
#define NOISE_TYPE_RIDGED 3
#define NOISE_TYPE_IQ 4
#define NOISE_TYPE_ALIEN 5
uniform int noiseLayerTypes[MAX_NOISE_LAYERS];
uniform float noiseLayerSeeds[MAX_NOISE_LAYERS];
uniform float noiseLayerMinHeights[MAX_NOISE_LAYERS];
uniform float noiseLayerMaxHeights[MAX_NOISE_LAYERS];
uniform int noiseLayerBlendTypes[MAX_NOISE_LAYERS];
uniform int numNoiseLayers;
// num noise layers actually used

uniform float equator_offset;
uniform float pole_size;
uniform float colorTableOffsets[8];
uniform float equatorColorTableColors[24];
uniform float poleColorTableColors[24];
uniform float colorTableDitherAmounts[8];
uniform int numColorTableEntries;
// num color tables entries used

uniform float waterShallowLevel;
uniform vec3 waterShallowColor;
uniform float waterDeepLevel;
uniform vec3 waterDeepColor;
uniform float waterSpecularAmount;
uniform vec3 waterSpecularColor;


///////////////////////////////////////////////////////////////////////////////////////////////////
//	Given an arbitrary 3D point this calculates the 4 vectors from the corners of the simplex pyramid to the point
//	It also returns the integer grid index information for the corners
//
void Simplex3D_GetCornerVectors( 	vec3 P,					//	input point
									out vec3 Pi,			//	integer grid index for the origin
									out vec3 Pi_1,			//	offsets for the 2nd and 3rd corners.  ( the 4th = Pi + 1.0 )
									out vec3 Pi_2,
									out vec4 v1234_x,		//	vectors from the 4 corners to the intput point
									out vec4 v1234_y,
									out vec4 v1234_z )
{
    //
	//	Simplex math from Stefan Gustavson's and Ian McEwan's work at...
	//	http://github.com/ashima/webgl-noise
	//

	//	simplex math constants
	const float SKEWFACTOR = 1.0/3.0;
    const float UNSKEWFACTOR = 1.0/6.0;
    const float SIMPLEX_CORNER_POS = 0.5;
    const float SIMPLEX_PYRAMID_HEIGHT = 0.70710678118654752440084436210485;
    // sqrt( 0.5 )	height of simplex pyramid.

	P *= SIMPLEX_PYRAMID_HEIGHT;
    // scale space so we can have an approx feature size of 1.0  ( optional )

	//	Find the vectors to the corners of our simplex pyramid
	Pi = floor( P + dot( P, vec3( SKEWFACTOR) ) );
    vec3 x0 = P - Pi + dot(Pi, vec3( UNSKEWFACTOR ) );
    vec3 g = step(x0.yzx, x0.xyz);
    vec3 l = 1.0 - g;
    Pi_1 = min( g.xyz, l.zxy );
    Pi_2 = max( g.xyz, l.zxy );
    vec3 x1 = x0 - Pi_1 + UNSKEWFACTOR;
    vec3 x2 = x0 - Pi_2 + SKEWFACTOR;
    vec3 x3 = x0 - SIMPLEX_CORNER_POS;
    //	pack them into a parallel-friendly arrangement
	v1234_x = vec4( x0.x, x1.x, x2.x, x3.x );
    v1234_y = vec4( x0.y, x1.y, x2.y, x3.y );
    v1234_z = vec4( x0.z, x1.z, x2.z, x3.z );
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void FAST32_hash_3D( 	vec3 gridcell,
						vec3 v1_mask,		//	user definable v1 and v2.  ( 0's and 1's )
						vec3 v2_mask,
						out vec4 hash_0,
						out vec4 hash_1,
						out vec4 hash_2	)		//	generates 3 random numbers for each of the 4 3D cell corners.  cell corners:  v0=0,0,0  v3=1,1,1  the other two are user definable
{
    //    gridcell is assumed to be an integer coordinate

	//	TODO: 	these constants need tweaked to find the best possible noise.
	//			probably requires some kind of brute force computational searching or something....
	const vec2 OFFSET = vec2( 50.0, 161.0 );
    const float DOMAIN = 69.0;
    const vec3 SOMELARGEFLOATS = vec3( 635.298681, 682.357502, 668.926525 );
    const vec3 ZINC = vec3( 48.500388, 65.294118, 63.934599 );
    //	truncate the domain
	gridcell.xyz = gridcell.xyz - floor(gridcell.xyz * ( 1.0 / DOMAIN )) * DOMAIN;
    vec3 gridcell_inc1 = step( gridcell, vec3( DOMAIN - 1.5 ) ) * ( gridcell + 1.0 );
    //	compute x*x*y*y for the 4 corners
	vec4 P = vec4( gridcell.xy, gridcell_inc1.xy ) + OFFSET.xyxy;
    P *= P;
    vec4 V1xy_V2xy = mix( P.xyxy, P.zwzw, vec4( v1_mask.xy, v2_mask.xy ) );
    //	apply mask for v1 and v2
	P = vec4( P.x, V1xy_V2xy.xz, P.z ) * vec4( P.y, V1xy_V2xy.yw, P.w );
    //	get the lowz and highz mods
	vec3 lowz_mods = vec3( 1.0 / ( SOMELARGEFLOATS.xyz + gridcell.zzz * ZINC.xyz ) );
    vec3 highz_mods = vec3( 1.0 / ( SOMELARGEFLOATS.xyz + gridcell_inc1.zzz * ZINC.xyz ) );
    //	apply mask for v1 and v2 mod values
	v1_mask = ( v1_mask.z < 0.5 ) ? lowz_mods : highz_mods;
    v2_mask = ( v2_mask.z < 0.5 ) ? lowz_mods : highz_mods;
    //	compute the final hash
	hash_0 = fract( P * vec4( lowz_mods.x, v1_mask.x, v2_mask.x, highz_mods.x ) );
    hash_1 = fract( P * vec4( lowz_mods.y, v1_mask.y, v2_mask.y, highz_mods.y ) );
    hash_2 = fract( P * vec4( lowz_mods.z, v1_mask.z, v2_mask.z, highz_mods.z ) );
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//	SimplexPerlin3D_Deriv
//	SimplexPerlin3D noise with derivatives
//	returns vec3( xderiv, yderiv, zderiv, value )
//
vec4 SimplexPerlin3D_Deriv(vec3 P)
{
    //	calculate the simplex vector and index math
	vec3 Pi;
    vec3 Pi_1;
    vec3 Pi_2;
    vec4 v1234_x;
    vec4 v1234_y;
    vec4 v1234_z;
    Simplex3D_GetCornerVectors( P, Pi, Pi_1, Pi_2, v1234_x, v1234_y, v1234_z );
    //	generate the random vectors
	//	( various hashing methods listed in order of speed )
	vec4 hash_0;
    vec4 hash_1;
    vec4 hash_2;
    FAST32_hash_3D( Pi, Pi_1, Pi_2, hash_0, hash_1, hash_2 );
    //SGPP_hash_3D( Pi, Pi_1, Pi_2, hash_0, hash_1, hash_2 );
	hash_0 -= 0.49999;
    hash_1 -= 0.49999;
    hash_2 -= 0.49999;
    //	normalize random gradient vectors
	vec4 norm = inversesqrt( hash_0 * hash_0 + hash_1 * hash_1 + hash_2 * hash_2 );
    hash_0 *= norm;
    hash_1 *= norm;
    hash_2 *= norm;
    //	evaluate gradients
	vec4 grad_results = hash_0 * v1234_x + hash_1 * v1234_y + hash_2 * v1234_z;
    //	evaluate the surflet f(x)=(0.5-x*x)^3
	vec4 m = v1234_x * v1234_x + v1234_y * v1234_y + v1234_z * v1234_z;
    m = max(0.5 - m, 0.0);
    //	The 0.5 here is SIMPLEX_PYRAMID_HEIGHT^2
	vec4 m2 = m*m;
    vec4 m3 = m*m2;
    //	calc the deriv
	vec4 temp = -6.0 * m2 * grad_results;
    float xderiv = dot( temp, v1234_x ) + dot( m3, hash_0 );
    float yderiv = dot( temp, v1234_y ) + dot( m3, hash_1 );
    float zderiv = dot( temp, v1234_z ) + dot( m3, hash_2 );
    const float FINAL_NORMALIZATION = 37.837227241611314102871574478976;
    //	scales the final result to a strict 1.0->-1.0 range

	//	sum with the surflet and return
	return vec4( xderiv, yderiv, zderiv, dot( m3, grad_results ) ) * FINAL_NORMALIZATION;
}

//
// Description : Array and textureless GLSL 2D/3D/4D simplex 
//               noise functions.
//      Author : Ian McEwan, Ashima Arts.
//  Maintainer : ijm
//     Lastmod : 20110822 (ijm)
//     License : Copyright (C) 2011 Ashima Arts. All rights reserved.
//               Distributed under the MIT License. See LICENSE file.
//               https://github.com/ashima/webgl-noise
// 

vec3 mod289(vec3 x) {
    return x - floor(x * (1.0 / 289.0)) * 289.0;
}

vec4 mod289(vec4 x) {
    return x - floor(x * (1.0 / 289.0)) * 289.0;
}

vec4 permute(vec4 x) {
    return mod289(((x*34.0)+1.0)*x);
}

vec4 taylorInvSqrt(vec4 r)
{
    return 1.79284291400159 - 0.85373472095314 * r;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
float snoise(vec3 v)
  {
    const vec2  C = vec2(1.0/6.0, 1.0/3.0) ;
    const vec4  D = vec4(0.0, 0.5, 1.0, 2.0);
    // First corner
  vec3 i  = floor(v + dot(v, C.yyy) );
    vec3 x0 =   v - i + dot(i, C.xxx) ;
    // Other corners
  vec3 g = step(x0.yzx, x0.xyz);
    vec3 l = 1.0 - g;
    vec3 i1 = min( g.xyz, l.zxy );
    vec3 i2 = max( g.xyz, l.zxy );
    //   x0 = x0 - 0.0 + 0.0 * C.xxx;
  //   x1 = x0 - i1  + 1.0 * C.xxx;
  //   x2 = x0 - i2  + 2.0 * C.xxx;
  //   x3 = x0 - 1.0 + 3.0 * C.xxx;
  vec3 x1 = x0 - i1 + C.xxx;
    vec3 x2 = x0 - i2 + C.yyy;
    // 2.0*C.x = 1/3 = C.y
  vec3 x3 = x0 - D.yyy;
    // -1.0+3.0*C.x = -0.5 = -D.y

// Permutations
  i = mod289(i);
    vec4 p = permute( permute( permute( 
			 i.z + vec4(0.0, i1.z, i2.z, 1.0 ))
		   + i.y + vec4(0.0, i1.y, i2.y, 1.0 )) 
		   + i.x + vec4(0.0, i1.x, i2.x, 1.0 ));
    // Gradients: 7x7 points over a square, mapped onto an octahedron.
// The ring size 17*17 = 289 is close to a multiple of 49 (49*6 = 294)
  float n_ = 0.142857142857;
    // 1.0/7.0
  vec3  ns = n_ * D.wyz - D.xzx;
    vec4 j = p - 49.0 * floor(p * ns.z * ns.z);
    //  mod(p,7*7)

  vec4 x_ = floor(j * ns.z);
    vec4 y_ = floor(j - 7.0 * x_ );
    // mod(j,N)

  vec4 x = x_ *ns.x + ns.yyyy;
    vec4 y = y_ *ns.x + ns.yyyy;
    vec4 h = 1.0 - abs(x) - abs(y);
    vec4 b0 = vec4( x.xy, y.xy );
    vec4 b1 = vec4( x.zw, y.zw );
    //vec4 s0 = vec4(lessThan(b0,0.0))*2.0 - 1.0;
  //vec4 s1 = vec4(lessThan(b1,0.0))*2.0 - 1.0;
  vec4 s0 = floor(b0)*2.0 + 1.0;
    vec4 s1 = floor(b1)*2.0 + 1.0;
    vec4 sh = -step(h, vec4(0.0));
    vec4 a0 = b0.xzyw + s0.xzyw*sh.xxyy ;
    vec4 a1 = b1.xzyw + s1.xzyw*sh.zzww ;
    vec3 p0 = vec3(a0.xy,h.x);
    vec3 p1 = vec3(a0.zw,h.y);
    vec3 p2 = vec3(a1.xy,h.z);
    vec3 p3 = vec3(a1.zw,h.w);
    //Normalise gradients
  vec4 norm = taylorInvSqrt(vec4(dot(p0,p0), dot(p1,p1), dot(p2, p2), dot(p3,p3)));
    p0 *= norm.x;
    p1 *= norm.y;
    p2 *= norm.z;
    p3 *= norm.w;
    // Mix final noise value
  vec4 m = max(0.6 - vec4(dot(x0,x0), dot(x1,x1), dot(x2,x2), dot(x3,x3)), 0.0);
    m = m * m;
    return 42.0 * dot( m*m, vec4( dot(p0,x0), dot(p1,x1), 
								dot(p2,x2), dot(p3,x3) ) );
}

///////////////////////////////////////////////////////////////////////////////////////////////////
float saturate(float f)
{
    return clamp(f, 0.0, 1.0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
float turbulence(vec3 p, float seed, int octaves,
				   float lacunarity = 1.92, float gain = 0.75)
{
    float sum = 0.0;
    float freq = 1.0, amp = 1.0;
	float scale = 0.0;
	float offset = seed;
    for (int i=0; i < octaves; i++)
	{
        float n = snoise(p*freq + vec3(offset,offset,offset));
        sum += n * amp;
		scale += amp;
        freq *= lacunarity;
        amp *= gain;
    }
	return (sum / scale) + 0.5;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
float ridgedTurbulence(vec3 p, float seed, int octaves,
				   float lacunarity = 1.92, float gain = 0.65)
{
    float sum = 0.0;
    float freq = 1.0, amp = 1.0;
	float scale = 0.0;
	float offset = seed;
    for (int i=0; i < octaves; i++)
	{
        float n = 1.0 - abs(snoise(p*freq + vec3(offset,offset,offset)));
		// i like the look better without the offset...
		//offset += 1.0;
        sum += n * amp;
		scale += amp;
        freq *= lacunarity;
        amp *= gain;
    }
	return sum / scale;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
float alienTurbulence(vec3 p, float seed, int octaves,
				   float lacunarity = 1.92, float gain = 0.67)
{
    float sum = 0.0;
    float freq = 1.0, amp = 1.0;
	float scale = 0.0;
	float offset = seed;
    for (int i=0; i < octaves; i++)
	{
        float n = 1.0 - abs(snoise(p*freq + vec3(offset,offset,offset)));
		// adding n is crazy...
		offset += n;
        sum += n * amp;
		scale += amp;
        freq *= lacunarity;
        amp *= gain;
    }
	return sum / scale;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
float iqTurbulence(vec3 p, float seed, int octaves,
				   float lacunarity = 2.77, float gain = 0.95)
{
    float sum = 0.0;
    float freq = 1.0, amp = 1.0;
    vec3 dsum = vec3(0.0,0.0,0.0);
    float offset = seed;
	float scale = 0.0;
    for (int i=0; i < octaves; i++)
	{
        vec4 n = SimplexPerlin3D_Deriv(p*freq + vec3(offset,offset,offset));
		scale += amp;
		dsum += n.xyz;
        sum += amp * n.w / (1.0 + dot(dsum, dsum));
        freq *= lacunarity;
        amp *= gain;
    }
	return (sum * 0.5) + 0.5;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
float swissTurbulence(vec3 p, float seed, int octaves,
					  float lacunarity = 1.92, float gain = 0.55,
					  float warp = 0.05)
{
    float sum = 0.0;
    float freq = 1.0, amp = 1.0;
    vec3 dsum = vec3(0.0,0.0,0.0);
    float offset = seed;
	float scale = 0.0;
    for(int i=0; i < octaves; i++)
	{
        vec4 n = SimplexPerlin3D_Deriv((p + warp * dsum)*freq + vec3(offset,offset,offset));
		offset += 1.0;
		
		float n2 = snoise((p + warp * dsum)*freq + vec3(offset,offset,offset));
		//sum += n2 * amp;

        //vec3 n = perlinNoiseDeriv((p + warp * dsum)*freq, seed + i);
		// our div function returns w as the value instead of x
		scale += amp;

		sum += amp * (1.0 - abs(n2));
		
        dsum += amp * n.xyz * -n2;
        freq *= lacunarity;
        amp *= gain; // * saturate(sum);
    }

	return sum * 0.4;
}


///////////////////////////////////////////////////////////////////////////////////////////////////
float jordanTurbulence(vec3 p, float seed, int octaves, float lacunarity = 2.35,
					   float gain1 = 0.8, float gain = 0.7,
					   float warp0 = 0.4, float warp = 0.35,
					   float damp0 = 1.0, float damp = 0.8,
					   float damp_scale = 1.0)
{
	float offset = seed;
	vec4 n = SimplexPerlin3D_Deriv(p + vec3(offset,offset,offset));
    vec4 n2 = n * n.w;
    float sum = n2.w;
    vec3 dsum_warp = warp0*n2.xyz;
    vec3 dsum_damp = damp0*n2.xyz;
    float amp = gain1;
    float freq = lacunarity;
    float damped_amp = amp * gain;
    for(int i=1; i < octaves; i++)
	{
        //n = perlinNoiseDeriv(p * freq + dsum_warp.xy, seed + i / 256.0);
		vec4 n = SimplexPerlin3D_Deriv(p * freq + dsum_warp.xyz + vec3(offset,offset,offset));
		offset += 1.0;
        n2 = n * n.w;
        sum += damped_amp * n2.w;
        dsum_warp += warp * n2.xyz;
        dsum_damp += damp * n2.xyz;
        freq *= lacunarity;
        amp *= gain;
        damped_amp = amp * (1.0-damp_scale/(1.0+dot(dsum_damp,dsum_damp)));
    }
	return sum;
}

varying vec3 vertexPos;
varying vec2 outUV;

///////////////////////////////////////////////////////////////////////////////////////////////////
float heightForNoise(int noiseType, float seed) 
{
    if (noiseType == NOISE_TYPE_PUFFY) {
        return jordanTurbulence(vertexPos, seed, 8);
    }
	else if(noiseType == NOISE_TYPE_SPECKLED) {
        return swissTurbulence(vertexPos, seed, 8);
    }
	else if(noiseType == NOISE_TYPE_TURBULENCE) {
        return turbulence(vertexPos, seed, 8);
    }
	else if(noiseType == NOISE_TYPE_RIDGED) {
        return ridgedTurbulence(vertexPos, seed, 8);
    }
	else if(noiseType == NOISE_TYPE_IQ) {
        return iqTurbulence(vertexPos, seed, 8);
    }
	else if(noiseType == NOISE_TYPE_ALIEN) {
        return alienTurbulence(vertexPos, seed, 8);
    }
	return 0.0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void main( void )
{
    float height = 0.0;
    float scale = 0.0;
    for( int i = 0; i < numNoiseLayers && i < MAX_NOISE_LAYERS; i++) {
		if( noiseLayerBlendTypes[i] == BLEND_TYPE_MUL) {
			// BLEND_TYPE_MUL
			height *= heightForNoise(noiseLayerTypes[i], noiseLayerSeeds[i]);
		}
		else {
			// BLEND_TYPE_ADD default
			height += heightForNoise(noiseLayerTypes[i], noiseLayerSeeds[i]);
			scale += 1.0;
		}
    }

    // keep height in range 0 .. 1
    height = height / scale;

	vec3 color = vec3(height,height,height);
	if (height < waterDeepLevel) {
		color = waterDeepColor;
	}
	else if (height < waterShallowLevel) {
		color = mix(waterShallowColor, waterDeepColor, (waterShallowLevel - height) / (waterShallowLevel - waterDeepLevel));
	}
	else {
		for (int i = 1; i < numColorTableEntries; i++) {
			if(height > colorTableOffsets[i - 1] && height <= colorTableOffsets[i]) {
				vec3 current = vec3(equatorColorTableColors[i * 3], equatorColorTableColors[i * 3 + 1], equatorColorTableColors[i * 3 + 2]);
				vec3 prev = vec3(equatorColorTableColors[(i - 1) * 3], equatorColorTableColors[(i - 1) * 3 + 1], equatorColorTableColors[(i - 1) * 3 + 2]);
				color = mix(current, prev, (colorTableOffsets[i] - height) / (colorTableOffsets[i] - colorTableOffsets[i - 1]));
			}
		}
	}
    
	//vec3 vertexPosNormalized = normalize(vertexPos);
	//float latitude = abs(dot(vertexPosNormalized, vec3(0.0,1.0,0.0)));
	float latitude = abs((outUV.t + equator_offset * 0.5)* 2.0 - 1.0) - (1.0 - pole_size * 2.0);
	//color.r = abs(latitude * 2.0 - 1.0);
	float cold = min(1.0,(latitude + height ));
	//float cold = min(1.0,height);
	color.r = 1.0 - cold;
	color.g = 0.0;
	color.b = cold;

	//height = heightForNoise(NOISE_TYPE_RIDGED, 0.0);
	//height = noiseLayerTypes[0];
    gl_FragColor.rgb = color;
    gl_FragColor.a = 1.0;
}



