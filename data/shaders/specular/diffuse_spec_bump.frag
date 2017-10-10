varying vec3 position;
varying vec3 normal;
varying vec3 tangent;
varying vec3 bitangent;
varying vec2 uv;

uniform sampler2D diffuseSampler;
uniform sampler2D normalSampler;
uniform sampler2D specularSampler;
uniform sampler2D glossinessSampler;

uniform vec3 ambientColor;
uniform vec3 diffuseColor;
uniform float specPowerFactor;
uniform vec3 specularColor;
uniform float nLights;

void main()
{
	vec3 normalMap = 2.0*texture2D(normalSampler, uv).rgb - 1.0;
	vec3 n = normalMap.r*normalize(tangent) + normalMap.g*normalize(bitangent) + normalMap.b*normalize(normal);
	
	vec3 diffuse = texture2D(diffuseSampler, uv).rgb;
	
	vec3 v = normalize(vec3(0.0,0.0,0.0) - position);
	float s = texture2D(specularSampler, uv).r;
	float g = texture2D(glossinessSampler, uv).r;
	
	vec3 c = ambientColor*diffuse;
	for (float i=0.0; i<nLights-0.5; i+=1.0)
	{
		vec3 l = normalize(gl_LightSource[int(i)].position.xyz - position);
		float ndl = dot(n,l);
		
		vec3 r = normalize(2.0*ndl*n - l);
		
		float ndr = pow(clamp(dot(r,v), 0, 1), g*specPowerFactor);
		
		c += clamp(ndl,0,1)*diffuseColor*diffuse + s*ndr*specularColor;
	}
	
	gl_FragColor = vec4(c,1);
}