#include <RobotDesignerLib/PololuMaestroRobotController.h>

PololuMaestroRobotController::PololuMaestroRobotController(Robot *robot, LocomotionEngineMotionPlan *motionPlan) : KinematicRobotController(robot, motionPlan) {
	loadMotionPlan(motionPlan);
}

PololuMaestroRobotController::~PololuMaestroRobotController(void) {

}

void PololuMaestroRobotController::draw() {

}

void PololuMaestroRobotController::loadMotionPlan(LocomotionEngineMotionPlan* motionPlan, double phase){
	KinematicRobotController::loadMotionPlan(motionPlan, phase);

}

void PololuMaestroRobotController::computeControlSignals(double timeStep) {
	KinematicRobotController::computeControlSignals(timeStep);
/*
	double dStridePhase = simTimeStep / motionPlan->motionPlanDuration;
	RobotState futureState(robot);
	motionPlan->robotStateTrajectory.getRobotPoseAt(stridePhase + dStridePhase, futureState);

	//the velocity here should be indicative of where we wish the servomotors to end up at the end of the step...
	for (int i = 0; i < robot->getJointCount(); i++) {
		HingeJoint* hj = dynamic_cast<HingeJoint*>(robot->getJoint(i));
		if (!hj) continue;
		double angleNow = desiredState.getJointRelativeOrientation(i).getRotationAngle(hj->rotationAxis);
		double angleNext = futureState.getJointRelativeOrientation(i).getRotationAngle(hj->rotationAxis);
		desiredState.setJointRelativeAngVelocity(hj->rotationAxis * ((angleNext - angleNow) / simTimeStep), hj->jIndex);
	}
*/
}

void PololuMaestroRobotController::applyControlSignals(double timeStep) {
	KinematicRobotController::applyControlSignals(timeStep);

	if (rci && rci->isConnected()) {
		//TODO: should turn that on...
		rci->writeAllTargetCommandsAtOnce = false;
		rci->controlPositionsOnly = false;
		rci->syncPhysicalRobotWithSimRobot(timeStep);
	}
}

void PololuMaestroRobotController::drawDebugInfo() {

}

void PololuMaestroRobotController::readRobotMappingParametersFromFile(const char* fName) {
	FILE* fp = fopen(fName, "r");

	if (!fp) return;

	int jCount = 0, wCount = 0;

	char line[100];

	readValidLine(line, 100, fp);
	sscanf(line, "%d %d", &jCount, &wCount);

	for (int i = 0; i < jCount; i++) {
		int jID = -1;
		int mID = -1;
		int pwmMin = 1000;
		int pwmMax = 2000;
		int pwmFor0 = 1500;
		int pwmFor45 = 2000;
		char flipAxis = 'f';
		readValidLine(line, 100, fp);
		sscanf(line, "%d %d %d %d %d %d %c", &jID, &mID, &pwmMin, &pwmMax, &pwmFor0, &pwmFor45, &flipAxis);

		HingeJoint* hj = dynamic_cast<HingeJoint*>(robot->getJoint(jID));
		if (!hj) continue;

		hj->motor.motorID = mID;

		hj->motor.pwmMin = pwmMin;//depends on the type of servomotor
		hj->motor.pwmMax = pwmMax;//depends on type of servomotor
		hj->motor.pwmFor0Deg = pwmFor0; //this depends on how the horn is mounted...
		hj->motor.pwmFor45Deg = pwmFor45; //this depends on how the horn is mounted...
		if (flipAxis != 'f')
			hj->motor.flipMotorAxis = true;
	}

	fclose(fp);
}

void PololuMaestroRobotController::initialize() {
	KinematicRobotController::initialize();

	if (rci == NULL) {
		rci = new PololuServoControlInterface(robot);
		rci->comNumber = 4;
		rci->openCommunicationPort();

	}
}

