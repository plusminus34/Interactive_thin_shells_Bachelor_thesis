#include <GUILib/GLUtils.h>
#include <RBSimLib/HingeJoint.h>
#include <Utils/Utils.h>

HingeJoint::HingeJoint(){
}

HingeJoint::~HingeJoint(void){

}

/**
	Projects the orientation of the child onto the constraint manifold that satisfies the type of joint this is
*/
void HingeJoint::fixOrientationConstraint() {
	Quaternion qRel = computeRelativeOrientation();
	//make sure that the relative rotation between the child and the parent is around the a axis
	V3D axis = qRel.v; axis.toUnit();
	//this is the rotation angle around the axis above, which may not be the rotation axis
	double rotAngle = qRel.getRotationAngle(axis);
	//get the rotation angle around the correct axis now (we are not in the world frame now)
	double ang = axis.dot(rotationAxis) * rotAngle;
	//compute the correct child orientation
	child->state.orientation = parent->state.orientation * getRotationQuaternion(ang, rotationAxis);
}

/**
	Projects the angular velocity of the child onto the constraint manifold that satisfies the type of joint this is
*/
void HingeJoint::fixAngularVelocityConstraint() {
	V3D wRel = child->state.angularVelocity - parent->state.angularVelocity;
	V3D worldRotAxis = parent->getWorldCoordinates(rotationAxis);
	//only keep the part that is aligned with the rotation axis...
	child->state.angularVelocity = parent->state.angularVelocity + worldRotAxis * wRel.getComponentAlong(worldRotAxis);
}

/**
	Processes a line of input, if it is specific to this type of joint. Returns true if processed, false otherwise.
*/
bool HingeJoint::processInputLine(char* line) {
	int lineType = getRBLineType(line);
	switch (lineType) {
	case RB_JOINT_ROT_AXES:
		sscanf(line, "%lf %lf %lf", &rotationAxis[0], &rotationAxis[1], &rotationAxis[2]);
		rotationAxis.toUnit();
		return true;
		break;
	case RB_JOINT_LIMITS:
		sscanf(line, "%lf %lf", &minAngle, &maxAngle);
		return true;
		break;
	case RB_MOTOR_ID:
		sscanf(line, "%d", &motorProperties.motorID);
		return true;
		break;
	case RB_FLIPMOTORAXISDIR:
		motorProperties.flipMotorAxis = true;
		return true;
		break;
	case RB_MAPPING_INFO:
		sscanf(line, "%d %d", &mappingInfo.index1, &mappingInfo.index2);
		return true;
		break;
	case RB_DEFAULT_ANGLE:
		sscanf(line, "%lf", &defaultAngle);
		return true;
		break;

	default:
		return false;
	}
}

/**
	draws the axes of rotation
*/
void HingeJoint::drawAxes() {
	glColor3d(1, 0, 0);
	drawArrow(getWorldPosition(), getWorldPosition() + parent->getWorldCoordinates(rotationAxis)*0.1, 0.0025);
}

/**
	writes the joint info to file...
*/
void HingeJoint::writeToFile(FILE* fp) {
	fprintf(fp, "\t%s\n", getRBString(RB_HINGE_JOINT));
	fprintf(fp, "\t\t%s %lf %lf %lf\n", getRBString(RB_JOINT_ROT_AXES), rotationAxis[0], rotationAxis[1], rotationAxis[2]);
	if (shouldUseJointLimits())
		fprintf(fp, "\t\t%s %lf %lf\n", getRBString(RB_JOINT_LIMITS), minAngle, maxAngle);
	if (motorProperties.flipMotorAxis) {
		fprintf(fp, "\t\t%s\n", getRBString(RB_FLIPMOTORAXISDIR));
	}

	fprintf(fp, "\t\t%s %d\n", getRBString(RB_MOTOR_ID), motorProperties.motorID);
	fprintf(fp, "\t\t%s %lf\n", getRBString(RB_DEFAULT_ANGLE), defaultAngle);
	fprintf(fp, "\t\t%s %d %d\n", getRBString(RB_MAPPING_INFO), mappingInfo.index1, mappingInfo.index2);
	writeCommonAttributesToFile(fp);
}

/**
	return true if joint limist should be used, false otherwise
*/
bool HingeJoint::shouldUseJointLimits() {
	return maxAngle > minAngle;
}



