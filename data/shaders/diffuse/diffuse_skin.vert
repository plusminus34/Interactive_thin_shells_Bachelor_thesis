
#version 430 compatibility

in vec4 in_Position;
in vec4 in_boneIds;
in vec4 in_boneWeights;

const int MAX_INFLUENCES = 4;
const int MAX_BONES = 50;

uniform mat4 boneMatrices[MAX_BONES]; 

out vec3 color;

void main(void)
{
	float totalWeight = 0.0;
	mat4 transform = mat4(0.0);
	
	for (int i = 0; i < MAX_INFLUENCES; ++i) 
	{
		highp int boneId = int(in_boneIds[i]);
		float w = in_boneWeights[i];
		if (boneId != -1) 
		{
			mat4 boneM = boneMatrices[boneId];	
			transform += boneM * w;
			totalWeight += w;
		}
	}

	if (totalWeight > 0.0) 
	{
		transform /= totalWeight; 
		vec4 newVertPos = vec4(in_Position.xyz, 1.0) * transform;
		gl_Position = gl_ModelViewProjectionMatrix * vec4(newVertPos.xyz, 1.0);
	} 
	else
		gl_Position = gl_ModelViewProjectionMatrix  * vec4(in_Position.xyz, 1.0);
		
	color = vec3(0.0, 0.5, 0.5);

}