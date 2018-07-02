#include "PlaybackController.h"

PlaybackController::PlaybackController(Robot *robot, LocomotionEngineMotionPlan *motionPlan) : RobotController(robot, motionPlan) {
	loadMotionPlan(motionPlan);
}

PlaybackController::~PlaybackController(void) {

}

void PlaybackController::draw() {

}

bool PlaybackController::advanceInTime(double timeStep) {
	stridePhase += timeStep / this->motionPlan->motionPlanDuration;

	return false;
}

void PlaybackController::computeDesiredState() {
	RobotController::computeDesiredState();
}

void PlaybackController::loadMotionPlan(LocomotionEngineMotionPlan* motionPlan, double phase){
	stridePhase = phase;
	this->motionPlan = motionPlan;

	RobotState startRobotState(robot);
	motionPlan->robotStateTrajectory.getRobotPoseAt(phase, startRobotState);
	robot->setState(&startRobotState);
}

void PlaybackController::computeControlSignals(double timeStep) {
	computeDesiredState();
}

void PlaybackController::applyControlSignals(double timeStep) {
	robot->setState(&desiredState);
}

void PlaybackController::drawDebugInfo() {

}

void PlaybackController::initialize() {
	stridePhase = 0;
	RobotState moptRobotState(robot);
	motionPlan->robotStateTrajectory.getRobotStateAt(stridePhase, motionPlan->motionPlanDuration, moptRobotState);
	robot->setState(&moptRobotState);
}

