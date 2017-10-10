#include <RBSimLib/BallAndSocketJoint.h>
#include <Utils/Utils.h>
#include <GUILib/GLUtils.h>

BallAndSocketJoint::BallAndSocketJoint(){
}

BallAndSocketJoint::~BallAndSocketJoint(void){

}

/**
	Projects the orientation of the child onto the constraint manifold that satisfies the type of joint this is
*/
void BallAndSocketJoint::fixOrientationConstraint() {
	//unrestricted range of motion -- nothing to do here...
}

/**
	Projects the angular velocity of the child onto the constraint manifold that satisfies the type of joint this is
*/
void BallAndSocketJoint::fixAngularVelocityConstraint() {
	//unrestricted range of motion -- nothing to do here...
}

/**
	draws the axes of rotation
*/
void BallAndSocketJoint::drawAxes() {
	glColor3d(1, 0, 0);
	drawArrow(getWorldPosition(), getWorldPosition() + parent->getWorldCoordinates(V3D(1, 0, 0))*0.1, 0.01);
	glColor3d(0, 1, 0);
	drawArrow(getWorldPosition(), getWorldPosition() + parent->getWorldCoordinates(V3D(0, 1, 0))*0.1, 0.01);
	glColor3d(0, 0, 1);
	drawArrow(getWorldPosition(), getWorldPosition() + parent->getWorldCoordinates(V3D(0, 0, 1))*0.1, 0.01);
}

/**
	writes the joint info to file...
*/
void BallAndSocketJoint::writeToFile(FILE* fp) {
	fprintf(fp, "\t%s\n", getRBString(RB_BALL_AND_SOCKET_JOINT));
	writeCommonAttributesToFile(fp);
}

/**
	Processes a line of input, if it is specific to this type of joint. Returns true if processed, false otherwise.
*/
bool BallAndSocketJoint::processInputLine(char* line) {
	return false;
}

