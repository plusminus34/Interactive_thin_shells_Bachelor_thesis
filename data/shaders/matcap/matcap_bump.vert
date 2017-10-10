varying vec3 position;
varying vec3 normal;
varying vec3 color;
varying vec2 uv;

void main()
{
	position = vec3(gl_ModelViewMatrix * gl_Vertex);
	normal = vec3(gl_NormalMatrix * gl_Normal);
	color = gl_Color.rgb;
	uv = gl_MultiTexCoord0.xy;
	
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
}