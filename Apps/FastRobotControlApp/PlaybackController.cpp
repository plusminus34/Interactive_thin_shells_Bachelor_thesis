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
//	desiredState = motionPlanner->getPreplanedRobotStateAtStridePhase(stridePhase);
	desiredState = motionPlanner->getMOPTRobotStateAtStridePhase(stridePhase);
}

void PlaybackController::computeControlSignals(double timeStep) {
	computeDesiredState();
}

void PlaybackController::applyControlSignals(double timeStep) {
	robot->setState(&desiredState);
}

void PlaybackController::drawDebugInfo() {
	applyControlSignals(0.01);
}

void PlaybackController::initialize() {
	stridePhase = 0;
}
