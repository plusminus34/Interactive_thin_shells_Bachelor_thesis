#include <RobotDesignerLib/KinematicRobotController.h>

KinematicRobotController::KinematicRobotController(Robot *robot, LocomotionEngineMotionPlan *motionPlan) : RobotController(robot, motionPlan) {
	loadMotionPlan(motionPlan);
}

KinematicRobotController::~KinematicRobotController(void) {

}

void KinematicRobotController::draw() {

}

void KinematicRobotController::computeMotionPlanPeriodDistAndHeading(){
	ReducedRobotState startRobotState(robot);
	ReducedRobotState endRobotState(robot);

//	motionPlan->robotStateTrajectory.getRobotPoseAt(0, startRobotState);
//	motionPlan->robotStateTrajectory.getRobotPoseAt(1, endRobotState);
//	mpTravelDist = endRobotState.getPosition() - startRobotState.getPosition();
//	mpTravelHeading = computeHeading(endRobotState.getOrientation() * startRobotState.getOrientation().getInverse(), V3D(0, 1, 0));
}

void KinematicRobotController::advanceInTime(double timeStep) {
//	vRel = V3D();
//	qRel = Quaternion();
	V3D posOffset;
	Quaternion qHeadingOffset = Quaternion();

	ReducedRobotState currentMPState(robot);
	ReducedRobotState nextMPState(robot);

	motionPlan->robotStateTrajectory.getRobotPoseAt(stridePhase, currentMPState);

	totalTime += timeStep;
	stridePhase += timeStep / this->motionPlan->motionPlanDuration;

	if (stridePhase > 1.0) {
		stridePhase -= 1.0;
	}
	else {
		motionPlan->robotStateTrajectory.getRobotPoseAt(stridePhase, nextMPState);
		posOffset = nextMPState.getPosition() - currentMPState.getPosition();
		qHeadingOffset = computeHeading(nextMPState.getOrientation() * currentMPState.getOrientation().getInverse(), V3D(0, 1, 0));
	}

	posInPlane += overallHeading * computeHeading(currentMPState.getOrientation(), V3D(0, 1, 0)).getInverse() * posOffset;
	overallHeading = qHeadingOffset * overallHeading;

//	motionPlan->robotStateTrajectory.getRobotPoseAt(stridePhase, nextMPState);
//	qRel = nextMPState.getOrientation() * qRel * currentMPState.getOrientation().getInverse();
//	vRel += nextMPState.getPosition() - currentMPState.getPosition();
//	vRel = computeHeading(currentMPState.getOrientation(), V3D(0, 1, 0)).getInverse().rotate(vRel);
//	qRel = nextMPState.getPosition() - currentMPState.getPosition();

}

void KinematicRobotController::computeDesiredState() {
//	ReducedRobotState currentRobotState(robot);
	motionPlan->robotStateTrajectory.getRobotPoseAt(stridePhase, desiredState);

	desiredState.setHeading(overallHeading.getRotationAngle(V3D(0,1,0)));
	P3D pos = P3D() + posInPlane;
	pos.y() = desiredState.getPosition().y();
	desiredState.setPosition(pos);

}

void KinematicRobotController::loadMotionPlan(LocomotionEngineMotionPlan* motionPlan, double phase){
	stridePhase = phase;

	this->motionPlan = motionPlan;
	computeMotionPlanPeriodDistAndHeading();
	ReducedRobotState startRobotState(robot);
	ReducedRobotState currentRobotState(robot);
	motionPlan->robotStateTrajectory.getRobotPoseAt(phase, startRobotState);

	// position
	P3D newPos = currentRobotState.getPosition();
	newPos[1] = startRobotState.getPosition()[1];

	// orientation
	Quaternion currentHeading = computeHeading(currentRobotState.getOrientation(), V3D(0, 1, 0));
	Quaternion mpWithoutHeading = computeHeading(startRobotState.getOrientation(), V3D(0, 1, 0)).getInverse() * startRobotState.getOrientation();
	Quaternion newOrientation = currentHeading * mpWithoutHeading;

	currentRobotState = startRobotState;
	currentRobotState.setPosition(newPos);
	currentRobotState.setOrientation(newOrientation);

	robot->setState(&currentRobotState);
}

void KinematicRobotController::computeControlSignals(double simTimeStep) {
	computeDesiredState();
}

void KinematicRobotController::applyControlSignals() {
	robot->setState(&desiredState);
}

void KinematicRobotController::drawDebugInfo() {

}

void KinematicRobotController::initialize() {
	stridePhase = 0;
	ReducedRobotState moptRobotState(robot);
	motionPlan->robotStateTrajectory.getRobotStateAt(stridePhase, motionPlan->motionPlanDuration, moptRobotState);
	robot->setState(&moptRobotState);

	posInPlane = V3D();
	overallHeading = Quaternion();
}

