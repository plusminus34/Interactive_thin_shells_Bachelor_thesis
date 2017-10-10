#include <RobotDesignerLib/PositionBasedRobotController.h>
#include <GUILib/GLUtils.h>


PositionBasedRobotController::PositionBasedRobotController(Robot *robot, LocomotionEngineMotionPlan *motionPlan) : RobotController(robot, motionPlan){
	totalTime = 0;
	this->robot = robot;
	this->motionPlan = motionPlan;

	//all the joints of the robots will be controlled via position control, so initialize this mode of operation
	initialize();
}

void PositionBasedRobotController::initialize() {
	//all the joints of the robots will be controlled via position control, so initialize this mode of operation
	for (uint i = 0; i < robot->jointList.size(); i++)
		robot->jointList[i]->controlMode = POSITION_MODE;
	stridePhase = 0;
	ReducedRobotState moptRobotState(robot);
	motionPlan->robotStateTrajectory.getRobotStateAt(stridePhase, motionPlan->motionPlanDuration, moptRobotState);
	robot->setState(&moptRobotState);
}

PositionBasedRobotController::~PositionBasedRobotController(void){
}

void PositionBasedRobotController::drawDebugInfo() {
}

void PositionBasedRobotController::computeControlSignals(double simTimeStep){
	computeDesiredState();
}

void PositionBasedRobotController::applyControlSignals() {
	for (int i = 0; i<robot->getJointCount(); i++) {
		HingeJoint* joint = dynamic_cast<HingeJoint*> (robot->getJoint(i));
		joint->desiredRelativeOrientation = desiredState.getJointRelativeOrientation(joint->jIndex);
	}
}
