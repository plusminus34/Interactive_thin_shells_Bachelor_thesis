
uniform float u_offset1; // offset along normal

void main(void){
    vec3 dir = gl_Normal;
//	vec3 dir = vec3(gl_Vertex);
	float len = length (dir);
	if (len > 0.0)
	    dir = dir / len;

	vec4 tPos   = vec4(vec3(gl_Vertex) + dir * u_offset1, 1.0);
	gl_Position	= gl_ModelViewProjectionMatrix * tPos;
}
