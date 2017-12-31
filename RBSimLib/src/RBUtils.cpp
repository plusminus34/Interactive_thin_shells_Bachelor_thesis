#include <RBSimLib/RBUtils.h>
#include <Utils/Utils.h>

KeyWord RBkeywords[] = {
	{ "RigidBody", RB_RB},
	{ "/End", RB_END_RB},
	{ "name", RB_NAME},
	{ "mass", RB_MASS},
	{ "moi", RB_MOI},
	{ "position", RB_POSITION},
	{ "orientation", RB_ORIENTATION},
	{ "velocity", RB_VELOCITY},
	{ "angularVelocity", RB_ANGULAR_VELOCITY},
	{ "frictionCoefficient", RB_FRICTION_COEFF},
	{ "restitutionCoefficient", RB_RESTITUTION_COEFF},
	{ "frozen", RB_IS_FROZEN},
	{ "CDP", RB_CDP},
	{ "mesh", RB_MESH_NAME},
	{ "material", RB_MATERIAL},
	{ "defineMaterial", RB_MATERIAL_DEFINITION },
	{ "colour", RB_COLOR},
	{ "child", RB_CHILD},
	{ "parent", RB_PARENT},
	{ "jointPPos", RB_PPOS},
	{ "jointCPos", RB_CPOS},
	{ "hingeJoint", RB_HINGE_JOINT},
	{ "universalJoint", RB_UNIVERSAL_JOINT},
	{ "ballAndSocketJoint", RB_BALL_AND_SOCKET_JOINT},
	{ "weldedJoint", RB_WELDED_JOINT},
	{ "jointLimits", RB_JOINT_LIMITS},
	{ "jointAxes", RB_JOINT_ROT_AXES},
	{ "controlMode", RB_JOINT_CONTROL_MODE},
	{ "/Joint", RB_JOINT_END},
	{ "endEffector", RB_END_EFFECTOR},
	{ "bodyPointFeature", RB_BODY_POINT_FEATURE},
	{ "thickness", RB_THICKNESSS},
	{ "motorID", RB_MOTOR_ID },
	{ "flipMotorAxis", RB_FLIPMOTORAXISDIR },
	{ "meshTransformation", RB_MESH_TRANSFORMATION },
	{ "mappingInfo", RB_MAPPING_INFO},
	{ "meshDescription", RB_MESH_DESCRIPTION},
	{ "defaultAngle", RB_DEFAULT_ANGLE }
};

// determine the type of a line that was used in the input file for a rigid body
int getRBLineType(char* &buffer) {
	return getLineType(buffer, RBkeywords, sizeof(RBkeywords) / sizeof(RBkeywords[0]));
}

// returns the string associated with the given token
char* getRBString(int token) {
	return getKeyword(token, RBkeywords, sizeof(RBkeywords) / sizeof(RBkeywords[0]));
}



