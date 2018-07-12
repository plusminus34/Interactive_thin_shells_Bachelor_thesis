#include <string.h>
#include "KineSimLib/KS_LoaderUtils.h"

KeyWord keywords[] = {
	{"positionInWorld", KS_POSITION_IN_WORLD},
	{"phase", KS_ANGLE},
	{"AssemblyTickCount", KS_ASSEMBLY_TICK_COUNT},
	{"beta", KS_BETA},
	{"gamma", KS_GAMMA},
	{"betaAxis", KS_BETA_AXIS},
	{"gammaAxis", KS_GAMMA_AXIS},
	{"name", KS_NAME},
	{"thickness", KS_THICKNESS},
	{"radius", KS_RADIUS},
	{"PlanarComponentConnection", KS_PLANAR_COMP_CON},
	{"componentIn", KS_COMPONENT_IN},
	{"componentOut", KS_COMPONENT_OUT},
	{"invertRotationDirection", KS_INVERT_ROTATION_DIRECTION},
	{"rotationSpeedRatio", KS_ROTATION_SPEED_RATIO},
	{"length", KS_LENGTH},
	{"width", KS_WIDTH},
	{"hollowInterval", KS_HOLLOW_INTERVAL},
	{"barEndPoints", KS_BAR_END_POINTS},
	{"Bar", KS_BAR},
	{"pinOnCompIn", KS_PIN_ON_COMP_IN},
	{"pinOnCompOut", KS_PIN_ON_COMP_OUT},
	{"BindComponentsConnection", KS_BIND_COMPONENTS_CON},
	{"BoundToWorldConnection", KS_BOUND_TO_WORLD_CON},
	{"vecOnCompIn", KS_VEC_ON_COMP_IN},
	{"vecOnCompOut", KS_VEC_ON_COMP_OUT},
	{"Shaft", KS_SHAFT},
	{"weldComponents", KS_WELD_COMPONENTS},
	{"allowArbitraryRelativeRotation", KS_ALLOW_ARBITRARY_ROTATION},
	{"lineOnCompOut", KS_LINE_ON_COMP_OUT},
	{"PointOnLineConnection", KS_POINT_ON_LINE_CON},
	{"inputMesh", KS_INPUT_MESH},
	{"GenericComponent", KS_GENERIC_COMPONENT},
	{"scale", KS_SCALE},
	{"PARAMETER", KS_PARAMETER},
	{"CONSTANT", KS_CONSTANT},	
	{"speedProfile", KS_NONCONSTANT_SPEED_PROFILE},
	{"constantSpeedProfile", KS_CONSTANT_SPEED_PROFILE},
	{"End", KS_END},
	{"color", KS_COLOR},
	{"Quad", KS_QUAD},
	{"quadPoints", KS_QUAD_POINTS},
	{"MultiLinkBar", KS_MULTI_LINK_BAR},
	{"HermiteSplineLinkBar", KS_HERMITE_SPLINE_LINK_BAR},
	{"hermiteSplineNSamples", KS_HERMITE_SPLINE_N_SAMPLES},
	{"hermiteSplineAngles", KS_HERMITE_SPLINE_ANGLES},
	{"hermiteSplineMagnitudes", KS_HERMITE_SPLINE_MAGNITUDES},
	{"rMotorConnection",KS_R_MOTOR_CON },
	{"vec2OnCompIn", KS_VEC2_ON_COMP_IN },
	{"vec2OnCompOut", KS_VEC2_ON_COMP_OUT },
	{"layer", KS_LAYER_NUMBER}
};

/**
	This method returns the string associated with the given token
*/
char* getKSString(int token){
	//declare a list of keywords
	int keyWordCount = sizeof(keywords)/sizeof(keywords[0]);

	for (int i=0;i<keyWordCount;i++){
		if (token == keywords[i].retVal)
			return keywords[i].keyWord;
	}

	return NULL;
}


/**
	This method is used to determine the type of a line that was used in the input file.
	It is assumed that there are no white spaces at the beginning of the string that is passed in. The pointer buffer
	will be updated to point at the first character after the keyword.
*/
int getKSLineType(char* &buffer){
	//declare a list of keywords
	int keyWordCount = sizeof(keywords)/sizeof(keywords[0]);

	for (int i=0;i<keyWordCount;i++){ 
		if (strncmp(buffer, keywords[i].keyWord, strlen(keywords[i].keyWord)) == 0 && isWhiteSpace(buffer[strlen(keywords[i].keyWord)])){
			buffer += strlen(keywords[i].keyWord);
			return keywords[i].retVal;
		}
	}

	return KS_NOT_IMPORTANT;
}


