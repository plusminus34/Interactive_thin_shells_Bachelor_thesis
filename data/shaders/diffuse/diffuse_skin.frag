
#version 430 compatibility

in vec3 color;

out vec4 outputColor;

void main(void)
{
	outputColor = vec4(color,1);
}