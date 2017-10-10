varying vec3 position;
varying vec3 normal;
varying vec3 color;
varying vec2 uv;
varying vec3 tangent;
varying vec3 bitangent;

uniform sampler2D textureSampler;
uniform sampler2D normalSampler;

void main()
{
	vec3 normalMap = 2.0*texture2D(normalSampler, uv).rgb - 1.0;
	vec3 n = normalMap.r*normalize(tangent) + normalMap.g*normalize(bitangent) + normalMap.b*normalize(normal);
//	vec3 n = normalize(normal);	
	
	vec4 tc = texture2D(textureSampler, uv);
	
	vec3 c = vec3(0.0,0.0,0.0);
	for (int i=0; i<4; i++)
	{
		vec3 l = normalize(gl_LightSource[i].position.xyz - position);
		float ndl = max(0.0, dot(n, l));
		c += ndl * tc.rgb * gl_FrontLightProduct[i].diffuse.rgb;
	}
	
	gl_FragColor.rgb = gl_FrontLightModelProduct.sceneColor.rgb + c;
	gl_FragColor.w = 1.0;
}

