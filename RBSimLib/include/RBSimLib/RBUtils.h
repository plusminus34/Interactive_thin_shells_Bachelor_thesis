#pragma once

enum RB_KEYWORDS{
 RB_NOT_IMPORTANT = -1,
 RB_RB,
 RB_END_RB,
 RB_NAME,
 RB_MASS,
 RB_MOI,
 RB_POSITION,
 RB_ORIENTATION,
 RB_VELOCITY,
 RB_ANGULAR_VELOCITY,
 RB_FRICTION_COEFF,
 RB_RESTITUTION_COEFF,
 RB_IS_FROZEN,
 RB_CDP,
 RB_MESH_NAME,
 RB_MATERIAL,
 RB_MATERIAL_DEFINITION,
 RB_COLOR,
 RB_CHILD,
 RB_PARENT,
 RB_PPOS,
 RB_CPOS,
 RB_HINGE_JOINT,
 RB_UNIVERSAL_JOINT,
 RB_BALL_AND_SOCKET_JOINT,
 RB_WELDED_JOINT,
 RB_JOINT_LIMITS,
 RB_JOINT_ROT_AXES,
 RB_JOINT_CONTROL_MODE,
 RB_JOINT_END,
 RB_END_EFFECTOR,
 RB_BODY_POINT_FEATURE,
 RB_THICKNESSS,
 RB_MOTOR_ID,
 RB_FLIPMOTORAXISDIR,
 RB_MESH_TRANSFORMATION,
 RB_MAPPING_INFO,
 RB_MESH_DESCRIPTION,
 RB_DEFAULT_ANGLE
};

// determine the type of a line that was used in the input file for a rigid body
int getRBLineType(char* &buffer);

// returns the string associated with the given token
char* getRBString(int token);
