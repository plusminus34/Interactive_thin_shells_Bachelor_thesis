varying vec3 position;
varying vec3 normal;
varying vec3 tangent;
varying vec3 bitangent;
varying vec2 uv;

void main()
{
	position = vec3(gl_ModelViewMatrix * gl_Vertex);
	normal = vec3(gl_NormalMatrix * gl_Normal);
	bitangent = cross(normal, gl_NormalMatrix * gl_MultiTexCoord1.xyz);
	tangent = cross(bitangent, normal);
	uv = gl_MultiTexCoord0.xy;
	
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
}