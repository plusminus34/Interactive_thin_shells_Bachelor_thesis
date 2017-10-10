varying vec3 position;
varying vec3 normal;
varying vec3 color;

uniform sampler2D matcapSampler;

void main()
{
	vec3 n = normalize(normal);
	vec3 mc = texture2D(matcapSampler, 0.5*(n.xy + 1.0)).rgb;
	
	gl_FragColor.rgb = mc;
	gl_FragColor.w = 1.0;
}