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

	char line[500];

	readValidLine(line, 500, fp);
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

	for (int i = 0; i < wCount; i++) {
		int jID = -1;
		int mID = -1;
		int pwmMin = 1000;
		int pwmMax = 2000;
		int pwmFor0 = 1500;
		int pwmDeadband = 20;
		int pwmFor50rpm = 1750;
		char flipAxis = 'f';
		readValidLine(line, 500, fp);
		sscanf(line, "%d %d %d %d %d %d %d %c", &jID, &mID, &pwmMin, &pwmMax, &pwmFor0, &pwmDeadband, &pwmFor50rpm, &flipAxis);

		HingeJoint* hj = dynamic_cast<HingeJoint*>(robot->getAuxiliaryJoint(jID));
		if (!hj) continue;

		hj->motor.motorID = mID;

		hj->motor.pwmMin = pwmMin;//depends on type of servomotor
		hj->motor.pwmMax = pwmMax;//depends on type of servomotor
		hj->motor.pwmFor0Deg = pwmFor0; //this is the neutral signal of the motor, depends on servomotor
		hj->motor.pwmDeadBand = pwmDeadband; //depends on servomotor
		hj->motor.pwmFor50RPM = pwmFor50rpm; //this is pwm value for 50RPM, depends on servomotor
		if (flipAxis != 'f')
			hj->motor.flipMotorAxis = true;
	}

	fclose(fp);

	if (rci)
		rci->createMultiWriteClusters();
}

void PololuMaestroRobotController::initialize() {
	KinematicRobotController::initialize();

	//all the joints of the robots will be controlled via position control, so initialize this mode of operation
	for (uint i = 0; i < robot->jointList.size(); i++)
		robot->jointList[i]->controlMode = POSITION_MODE;

	for (int i = 0; i < robot->getRigidBodyCount(); i++) {
		RigidBody* rb = robot->getRigidBody(i);
		for (uint j = 0; j < rb->rbProperties.endEffectorPoints.size(); j++) {
			RBEndEffector* ee = &(rb->rbProperties.endEffectorPoints[j]);
			if (ee->wheelJoint && (ee->isActiveWheel() || ee->isFreeToMoveWheel())) {
				if (ee->isActiveWheel())
					ee->wheelJoint->controlMode = VELOCITY_MODE;
				else
					ee->wheelJoint->controlMode = PASSIVE;
			}
		}
	}

	if (rci == NULL) {
		rci = new PololuServoControlInterface(robot);
		rci->comNumber = 4;
//		rci->signalPeriod = 3;//ms - CHECK WITH THE BOARD SETTINGS!!!
		rci->openCommunicationPort();
		rci->createMultiWriteClusters();
	}
	else
		rci->driveMotorPositionsToZero();

}


