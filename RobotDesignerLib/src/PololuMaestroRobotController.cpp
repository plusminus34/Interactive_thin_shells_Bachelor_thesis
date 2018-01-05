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
		rci->controlPositionsOnly = false;
		rci->syncPhysicalRobotWithSimRobot(timeStep);
	}
}

void PololuMaestroRobotController::drawDebugInfo() {

}

void PololuMaestroRobotController::initialize() {
	KinematicRobotController::initialize();

	if (rci == NULL) {
		rci = new PololuServoControlInterface(robot);
		rci->comNumber = 4;
		rci->openCommunicationPort();

		//load/create the list of motor IDs, range of motion, etc here
		for (int i=0;i<robot->getJointCount();i++){
			HingeJoint* hj = dynamic_cast<HingeJoint*>(robot->getJoint(i));
			if (!hj) continue;

			hj->motor.motorID = i;
				
			//settings for the TURNIGY S306G-HV
			hj->motor.pwmMin = 910;//depends on the type of servomotor
			hj->motor.pwmMax = 2100;//depends on type of servomotor
			hj->motor.pwmFor0Deg = 1430; //this depends on how the horn is mounted...
			hj->motor.pwmFor45Deg = 1865; //this depends on how the horn is mounted...
		}
	}

}

