varying vec3 position;
varying vec3 normal;
varying vec3 color;

uniform float rimPower;
uniform vec3 rimColor;

void main()
{
	vec3 n = normalize(normal);
	vec3 c = vec3(0.0,0.0,0.0);
	for (int i=0; i<4; i++)
	{
		vec3 l = normalize(gl_LightSource[i].position.xyz - position);
		float ndl = max(0.0, dot(n, l));
		c += ndl*gl_FrontLightProduct[i].diffuse.rgb;
	}
	
	gl_FragColor.rgb = gl_FrontLightModelProduct.sceneColor.rgb + c + pow(1.0 - max(0.0, n.z), rimPower) * rimColor;
	gl_FragColor.w = 1.0;
}