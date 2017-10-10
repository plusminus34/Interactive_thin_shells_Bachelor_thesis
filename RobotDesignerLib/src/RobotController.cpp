#include <RobotDesignerLib/RobotController.h>

RobotController::RobotController(Robot* robot, LocomotionEngineMotionPlan *motionPlan) : desiredState(robot) {
	totalTime = 0;
	this->robot = robot;
	this->motionPlan = motionPlan;

	//all the joints of the robots will be controlled via position control, so initialize this mode of operation
	initialize();
}


RobotController::~RobotController(){
}

void RobotController::advanceInTime(double timeStep) {
	totalTime += timeStep;
	stridePhase += timeStep / this->motionPlan->motionPlanDuration;
	if (stridePhase > 1.0)
		stridePhase -= 1.0;
}

void RobotController::initialize() {
	//all the joints of the robots will be controlled via position control, so initialize this mode of operation
	for (uint i = 0; i < robot->jointList.size(); i++)
		robot->jointList[i]->controlMode = POSITION_MODE;
	stridePhase = 0;
	ReducedRobotState moptRobotState(robot);
	motionPlan->robotStateTrajectory.getRobotStateAt(stridePhase, motionPlan->motionPlanDuration, moptRobotState);
	robot->setState(&moptRobotState);
}

void RobotController::computeControlSignals(double simTimeStep) {
	computeDesiredState();
}

void RobotController::drawDebugInfo() {
}

void RobotController::applyControlSignals() {
	for (int i = 0; i<robot->getJointCount(); i++) {
		HingeJoint* joint = dynamic_cast<HingeJoint*> (robot->getJoint(i));
		joint->desiredRelativeOrientation = desiredState.getJointRelativeOrientation(joint->jIndex);
	}
}

void RobotController::computeDesiredState() {
	motionPlan->robotStateTrajectory.getRobotPoseAt(stridePhase, desiredState);
}

