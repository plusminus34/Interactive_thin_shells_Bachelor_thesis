varying vec3 position;
varying vec3 normal;
varying vec3 color;
varying vec2 uv;

uniform sampler2D matcapSampler;
uniform sampler2D normalMapSampler;

void main()
{
//	vec3 n = normalize(normal);
//	vec3 mc = texture(matcapSampler, 0.5*(n.xy + 1.0)).rgb;

	vec3 nb = texture2D(normalMapSampler, uv).rgb;
	nb = normalize(gl_NormalMatrix * (2.0*nb - 1.0));
	
	vec3 mc = texture2D(matcapSampler, 0.5*nb.xy + 0.5).rgb;
	
	gl_FragColor.rgb = mc;
	gl_FragColor.w = 1.0;
}