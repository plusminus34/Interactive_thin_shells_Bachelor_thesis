#include <RobotDesignerLib/PololuMaestroRobotController.h>

PololuMaestroRobotController::PololuMaestroRobotController(Robot *robot, LocomotionEngineMotionPlan *motionPlan) : KinematicRobotController(robot, motionPlan) {
	loadMotionPlan(motionPlan);
}

PololuMaestroRobotController::~PololuMaestroRobotController(void) {

}

void PololuMaestroRobotController::draw() {

}

void PololuMaestroRobotController::advanceInTime(double timeStep) {
	KinematicRobotController::advanceInTime(timeStep);

}

void PololuMaestroRobotController::computeDesiredState() {
	KinematicRobotController::computeDesiredState();
}

void PololuMaestroRobotController::loadMotionPlan(LocomotionEngineMotionPlan* motionPlan, double phase){
	KinematicRobotController::loadMotionPlan(motionPlan, phase);

}

void PololuMaestroRobotController::computeControlSignals(double simTimeStep) {
	KinematicRobotController::computeControlSignals(simTimeStep);
}

void PololuMaestroRobotController::applyControlSignals() {
	KinematicRobotController::applyControlSignals();
	robot->setState(&desiredState);

}

void PololuMaestroRobotController::drawDebugInfo() {

}

void PololuMaestroRobotController::initialize() {
	KinematicRobotController::initialize();

}

