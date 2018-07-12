#pragma once

#pragma warning(disable:4996)

#include <stdio.h>
#include <Utils/Utils.h>

enum KS_KEYWORDS {
	KS_NOT_IMPORTANT = 1,
	KS_COMMENT,
	KS_ASSEMBLY_TICK_COUNT,
	KS_END,
	KS_POSITION_IN_WORLD,
	KS_ANGLE,
	KS_NAME,
	KS_THICKNESS,
	KS_RADIUS,
	KS_COMPONENT_IN,
	KS_COMPONENT_OUT,
	KS_INVERT_ROTATION_DIRECTION,
	KS_ROTATION_SPEED_RATIO,
	KS_POINT_ON_LINE_CON,
	KS_LENGTH,
	KS_WIDTH,
	KS_HOLLOW_INTERVAL,
	KS_BAR_END_POINTS,
	KS_BAR,
	KS_PIN_ON_COMP_IN,
	KS_PIN_ON_COMP_OUT,
	KS_LINE_ON_COMP_OUT,
	KS_BIND_COMPONENTS_CON,
	KS_BOUND_TO_WORLD_CON,
	KS_PLANAR_COMP_CON,
	KS_VEC_ON_COMP_IN,
	KS_VEC_ON_COMP_OUT,
	KS_SHAFT,
	KS_WELD_COMPONENTS,
	KS_ALLOW_ARBITRARY_ROTATION,
	KS_BETA,
	KS_BETA_AXIS,
	KS_GAMMA,
	KS_GAMMA_AXIS,
	KS_COLOR,
	KS_INPUT_MESH,
	KS_GENERIC_COMPONENT,
	KS_SCALE,
	KS_NONCONSTANT_SPEED_PROFILE,
	KS_CONSTANT_SPEED_PROFILE,
	KS_CONSTANT,
	KS_PARAMETER,
	KS_QUAD,
	KS_QUAD_POINTS,
	KS_MULTI_LINK_BAR,
	KS_HERMITE_SPLINE_LINK_BAR,
	KS_HERMITE_SPLINE_N_SAMPLES,
	KS_HERMITE_SPLINE_ANGLES,
	KS_HERMITE_SPLINE_MAGNITUDES,
	KS_R_MOTOR_CON,
	KS_VEC2_ON_COMP_IN,
	KS_VEC2_ON_COMP_OUT,
	KS_LAYER_NUMBER
};

/**
	This method is used to determine the type of a line that was used in the input file.
	It is assumed that there are no white spaces at the beginning of the string that is passed in. The pointer buffer
	will be updated to point at the first character after the keyword.
*/
int getKSLineType(char* &buffer);

/**
	This method returns the string associated with the given token
*/
char* getKSString(int token);

