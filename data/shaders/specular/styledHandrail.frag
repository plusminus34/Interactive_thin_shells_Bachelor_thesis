varying vec3 position;
varying vec3 normal;
varying vec3 tangent;
varying vec3 bitangent;
varying vec2 uv;

uniform sampler2D matcapSampler;

uniform vec3 ambientColor;
uniform vec3 diffuseColor;
uniform vec3 specularMatcapColor;

uniform vec3 keyLightDiffuseColor;
uniform vec3 keyLightSpecularColor;
uniform vec2 keyLightFalloff;

uniform vec3 fillLightDiffuseColor;
uniform vec3 fillLightSpecularColor;
uniform vec2 fillLightFalloff;

uniform float specularStrength;
uniform float glossiness;

uniform float alpha;

void main()
{
	vec3 n = normalize(normal);
	
	vec3 v = normalize(vec3(0,0,0) - position);
	
	vec3 c = ambientColor;
	// Key light
	{
		vec3 l = gl_LightSource[0].position.xyz - position;
		float falloff = 1 - clamp((length(l) - keyLightFalloff.x) / (keyLightFalloff.y - keyLightFalloff.x), 0., 1.);
		
		l = normalize(l);
		float ndl = dot(n,l);
		
		vec3 r = normalize(2*ndl*n - l);
		float ndr = pow(clamp(dot(r,v), 0., 1.), glossiness);
		
		c += falloff*clamp(ndl, 0., 1.)*keyLightDiffuseColor*diffuseColor + falloff*specularStrength*ndr*keyLightSpecularColor;
	}
	// Fill light
	{
		vec3 l = gl_LightSource[1].position.xyz - position;
		float falloff = 1 - clamp((length(l) - fillLightFalloff.x) / (fillLightFalloff.y - fillLightFalloff.x), 0., 1.);
		
		l = normalize(l);
		float ndl = dot(n,l);
		
		vec3 r = normalize(2*ndl*n - l);
		float ndr = pow(clamp(dot(r,v), 0., 1.), glossiness);
		
		c += falloff*clamp(ndl, 0., 1.)*fillLightDiffuseColor*diffuseColor + falloff*specularStrength*ndr*fillLightSpecularColor;
	}
	
	// Additive matcap
	vec3 matcapVal = texture(matcapSampler, 0.5*n.xy + 0.5).rgb;
	c += specularMatcapColor * matcapVal;
	
	gl_FragColor = vec4(c,alpha);
}