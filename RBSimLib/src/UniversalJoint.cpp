#include <GUILib/GLUtils.h>
#include <RBSimLib/UniversalJoint.h>
#include <Utils/Utils.h>

UniversalJoint::UniversalJoint(){
}

UniversalJoint::~UniversalJoint(void){

}

/**
	Projects the orientation of the child onto the constraint manifold that satisfies the type of joint this is
*/
void UniversalJoint::fixOrientationConstraint() {
	double rotAngleParent = 0, rotAngleChild = 0;
	//qRel is the orientation from child frame to parent frame
	Quaternion qRel = computeRelativeOrientation();
	computeEulerAnglesFromQuaternion(qRel, rotAxisChild, rotAxisParent, rotAngleChild, rotAngleParent);
	child->state.orientation = parent->state.orientation * getRotationQuaternion(rotAngleParent, rotAxisParent) * getRotationQuaternion(rotAngleChild, rotAxisChild);
}

/**
	Projects the angular velocity of the child onto the constraint manifold that satisfies the type of joint this is
*/
void UniversalJoint::fixAngularVelocityConstraint() {
	V3D wRel = child->state.angularVelocity - parent->state.angularVelocity;
	//the two rotation axes are always orthogonal to each other
	V3D worldRotAxisParent = parent->getWorldCoordinates(rotAxisParent);
	V3D worldRotAxisChild = child->getWorldCoordinates(rotAxisChild);

	//only keep the part that is aligned with the rotation axis...
	child->state.angularVelocity = parent->state.angularVelocity + worldRotAxisParent * wRel.getComponentAlong(worldRotAxisParent) + worldRotAxisChild * wRel.getComponentAlong(worldRotAxisChild);
}

/**
	Processes a line of input, if it is specific to this type of joint. Returns true if processed, false otherwise.
*/
bool UniversalJoint::processInputLine(char* line) {
	int lineType = getRBLineType(line);
	switch (lineType) {
	case RB_JOINT_ROT_AXES:
		sscanf(line, "%lf %lf %lf %lf %lf %lf", &rotAxisParent[0], &rotAxisParent[1], &rotAxisParent[2], &rotAxisChild[0], &rotAxisChild[1], &rotAxisChild[2]);
		rotAxisParent.toUnit();
		rotAxisChild.toUnit();
		return true;
		break;
	case RB_JOINT_LIMITS:
		sscanf(line, "%lf %lf %lf %lf", &minAnglePRA, &maxAnglePRA, &minAngleCRA, &maxAngleCRA);
		return true;
		break;
	default:
		return false;
	}
}

/**
	draws the axes of rotation
*/
void UniversalJoint::drawAxes() {
	glColor3d(1, 0, 0);
	drawArrow(getWorldPosition(), getWorldPosition() + parent->getWorldCoordinates(rotAxisParent)*0.1, 0.01);
	glColor3d(0, 1, 0);
	drawArrow(getWorldPosition(), getWorldPosition() + child->getWorldCoordinates(rotAxisChild)*0.1, 0.01);
}

/**
	writes the joint info to file...
*/
void UniversalJoint::writeToFile(FILE* fp) {
	fprintf(fp, "\t%s\n", getRBString(RB_UNIVERSAL_JOINT));
	fprintf(fp, "\t\t%s %lf %lf %lf %lf %lf %lf\n", getRBString(RB_JOINT_ROT_AXES), rotAxisParent[0], rotAxisParent[1], rotAxisParent[2], rotAxisChild[0], rotAxisChild[1], rotAxisChild[2]);
	if (shouldUseJointLimits())
		fprintf(fp, "\t\t%s %lf %lf %lf %lf\n", getRBString(RB_JOINT_LIMITS), minAnglePRA, maxAnglePRA, minAngleCRA, maxAngleCRA);
	writeCommonAttributesToFile(fp);
}

/**
	return true if joint limist should be used, false otherwise
*/
bool UniversalJoint::shouldUseJointLimits() {
	return maxAnglePRA > minAnglePRA || maxAngleCRA > minAngleCRA;
}




