uniform mat4 worldViewProj;
varying vec3 vertexPos;
varying vec2 outUV;

void main()
{
	gl_Position = worldViewProj * gl_Vertex;
	outUV = gl_MultiTexCoord0.xy;
	vertexPos = normalize(gl_Vertex.xyz);
}
