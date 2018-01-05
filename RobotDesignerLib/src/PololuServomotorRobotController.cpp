#include <RobotDesignerLib/KinematicRobotController.h>

KinematicRobotController::KinematicRobotController(Robot *robot, LocomotionEngineMotionPlan *motionPlan) : RobotController(robot, motionPlan) {
	loadMotionPlan(motionPlan);
}

KinematicRobotController::~KinematicRobotController(void) {

}

void KinematicRobotController::draw() {

}

void KinematicRobotController::advanceInTime(double timeStep) {
//	vRel = V3D();
//	qRel = Quaternion();
	V3D posOffset;
	Quaternion qHeadingOffset = Quaternion();

	RobotState currentMPState(robot);
	RobotState nextMPState(robot);

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

}

void KinematicRobotController::computeDesiredState() {
	RobotController::computeDesiredState();

	desiredState.setHeading(overallHeading.getRotationAngle(V3D(0,1,0)));
	P3D pos = P3D() + posInPlane;
	pos.y() = desiredState.getPosition().y();
	desiredState.setPosition(pos);
}

void KinematicRobotController::loadMotionPlan(LocomotionEngineMotionPlan* motionPlan, double phase){
	stridePhase = phase;

	this->motionPlan = motionPlan;

	RobotState startRobotState(robot);
	RobotState currentRobotState(robot);
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
	timeStep = simTimeStep;
	computeDesiredState();
}

void KinematicRobotController::applyControlSignals() {
	robot->setState(&desiredState);

	//integrate forward in time the motion of the weels...
	for (uint j = 0; j < motionPlan->endEffectorTrajectories.size(); j++) {
		LocomotionEngine_EndEffectorTrajectory* eeTraj = &motionPlan->endEffectorTrajectories[j];
		if (eeTraj->isWheel) {
			RigidBody* rb = eeTraj->endEffectorRB;
			int eeIndex = eeTraj->CPIndex;
			int meshIndex = rb->rbProperties.endEffectorPoints[eeIndex].meshIndex;
			if (meshIndex >= 0)
				rb->meshTransformations[meshIndex].R = getRotationQuaternion(timeStep * rb->rbProperties.endEffectorPoints[eeIndex].wheelSpeed_rel, rb->rbProperties.endEffectorPoints[eeIndex].localCoordsWheelAxis).getRotationMatrix() * rb->meshTransformations[meshIndex].R;
		}
	}
}

void KinematicRobotController::drawDebugInfo() {

}

void KinematicRobotController::initialize() {
	stridePhase = 0;
	RobotState moptRobotState(robot);
	motionPlan->robotStateTrajectory.getRobotStateAt(stridePhase, motionPlan->motionPlanDuration, moptRobotState);
	robot->setState(&moptRobotState);

	//reset the orientation of the wheels to their initial values
	for (uint j = 0; j < motionPlan->endEffectorTrajectories.size(); j++) {
		LocomotionEngine_EndEffectorTrajectory* eeTraj = &motionPlan->endEffectorTrajectories[j];
		if (eeTraj->isWheel) {
			RigidBody* rb = eeTraj->endEffectorRB;
			int eeIndex = eeTraj->CPIndex;
			int meshIndex = rb->rbProperties.endEffectorPoints[eeIndex].meshIndex;
			Joint* wheelJoint = rb->rbProperties.endEffectorPoints[eeIndex].wheelJoint;

			if (meshIndex >= 0 && wheelJoint)
				rb->meshTransformations[meshIndex].R = rb->rbProperties.endEffectorPoints[eeIndex].initialMeshTransformation.R;
		}
	}

	posInPlane = V3D();
	overallHeading = Quaternion();
}

