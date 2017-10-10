
#version 430 compatibility

in vec4 in_Position;

out vec3 color;

void main(void)
{
///< If CPU skining, fall back onto:
	gl_Position = gl_ModelViewProjectionMatrix  * vec4(in_Position.xyz, 1.0);

	color = vec3(0.5, 0.0, 0.5);
}