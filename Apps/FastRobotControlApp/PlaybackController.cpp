#include "PlaybackController.h"

PlaybackController::PlaybackController(Robot *robot, MotionPlanner *motionPlanner) : RobotController(robot, motionPlanner->locomotionManager->motionPlan) {
	stridePhase = 0;
	this->motionPlanner = motionPlanner;
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
	desiredState = motionPlanner->getPreplanedRobotStateAtTime(motionPlanner->motionPlanStartTime + stridePhase * motionPlanner->locomotionManager->motionPlan->motionPlanDuration);
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

