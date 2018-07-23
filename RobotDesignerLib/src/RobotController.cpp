#include <RobotDesignerLib/RobotController.h>

RobotController::RobotController(Robot* robot, LocomotionEngineMotionPlan *motionPlan) : desiredState(robot) {
	this->robot = robot;
	this->motionPlan = motionPlan;

	//all the joints of the robots will be controlled via position control, so initialize this mode of operation
	initialize();
}

RobotController::~RobotController(){
}

bool RobotController::advanceInTime(double timeStep) {
	stridePhase += timeStep / this->motionPlan->motionPlanDuration;
	if (stridePhase > 1.0){
		stridePhase -= 1.0;
		return true;
	}
	return false;
}

void RobotController::initialize() {
	//all the joints of the robots will be controlled via position control, so initialize this mode of operation
	for (uint i = 0; i < robot->jointList.size(); i++)
		robot->jointList[i]->controlMode = POSITION_MODE;
	stridePhase = 0;
	RobotState moptRobotState(robot);
	motionPlan->robotStateTrajectory.getRobotStateAt(stridePhase, motionPlan->motionPlanDuration, moptRobotState);
	robot->setState(&moptRobotState);
}

void RobotController::computeControlSignals(double timeStep) {
	computeDesiredState();
}

void RobotController::drawDebugInfo() {
}

void RobotController::applyControlSignals(double timeStep) {
	for (int i = 0; i<robot->getJointCount(); i++) {
		HingeJoint* joint = dynamic_cast<HingeJoint*> (robot->getJoint(i));
		joint->desiredRelativeOrientation = desiredState.getJointRelativeOrientation(joint->jIndex);
	}
}

void RobotController::computeDesiredState() {
	motionPlan->robotStateTrajectory.getRobotStateAt(stridePhase, motionPlan->motionPlanDuration, desiredState);

	RobotState rs(robot);
	robot->setState(&desiredState);

	//we now have the pose of the robot, and we should also read off the wheel speed...
	for (uint i = 0; i < motionPlan->endEffectorTrajectories.size(); i++) {
		LocomotionEngine_EndEffectorTrajectory* eeTraj = &motionPlan->endEffectorTrajectories[i];
		if (eeTraj->isWheel) {
			RigidBody* rb = eeTraj->endEffectorRB;
			int eeIndex = eeTraj->CPIndex;
			RBEndEffector* ee = &rb->rbProperties.endEffectorPoints[eeIndex];
			
			//NOTE: due to interpolation artifacts, this is not a very good estimate... compute it instead based on a no-slip assumption...
//			ee->wheelSpeed_w = -eeTraj->getWheelSpeedAt(stridePhase);

			RBEndEffector worldEE = *ee;		//tmpEE holds all quantities of the wheel in world coordinates, placed according to its parent RB
			worldEE.localCoordsWheelAxis = rb->getWorldCoordinates(ee->getWheelAxis());
			worldEE.coords = rb->getWorldCoordinates(ee->coords);

			//due to the speed of the wheel, the point at rho (i.e. bottom of the wheel) should have zero speed relative to the ground... but first remove the contribution that is not in the direction the wheel is moving on
			V3D wheelCenterSpeed = rb->getAbsoluteVelocityForLocalPoint(ee->coords).getProjectionOn(worldEE.getWheelTiltAxis());

			//We know we want it to lead to zero velocity at the contact point...
			//i.e.: wheelCenterSpeed + (rotAxis_w).cross(V3D(tmpEE.getWheelRho()) * -1) * ee->wheelSpeed_w = 0;
			double wheelSpeed_w = wheelCenterSpeed.dot(worldEE.getWheelTiltAxis()) / worldEE.getWheelRho().norm();

			//now, to compute the speed of the wheel relative to the parent RB, we need to know how much of this rigid body's angular velocity aligns with the rotation axis, this is the part that we need to factor out... 
			V3D rbAngVelocity = rb->state.angularVelocity;
			double rbSpeed = rbAngVelocity.dot(worldEE.localCoordsWheelAxis);

			double wheelSpeed_rel = wheelSpeed_w - rbSpeed;

			if (ee->isWeldedWheel())
				wheelSpeed_rel = 0;

			if (ee->wheelJoint != NULL)
				desiredState.setAuxiliaryJointRelativeAngVelocity(ee->wheelJoint->rotationAxis * wheelSpeed_rel, ee->wheelJoint->jIndex);

			//now, check if we've succeeded...
			V3D contactPointVelocity = wheelCenterSpeed + (worldEE.localCoordsWheelAxis * wheelSpeed_w).cross(V3D(worldEE.getWheelRho()) * -1);
			if (!IS_ZERO(contactPointVelocity.norm()))
				Logger::consolePrint("velocity @ contact point:\t%lf\t%lf\t%lf\t%lf\n", contactPointVelocity.x(), contactPointVelocity.y(), contactPointVelocity.z(), contactPointVelocity.norm());

			V3D wheelAngularVelocity = rbAngVelocity + V3D(worldEE.getWheelAxis()) * wheelSpeed_rel;
			V3D tmpCPVel_global = rb->getAbsoluteVelocityForLocalPoint(ee->coords) - wheelAngularVelocity.cross(worldEE.getWheelRho());
			tmpCPVel_global = tmpCPVel_global.getProjectionOn(worldEE.getWheelTiltAxis());

//			if (!IS_ZERO(tmpCPVel_global.norm()))
//				Logger::consolePrint("velocity @ contact point:\t%lf\t%lf\t%lf\t%lf\n", tmpCPVel_global.x(), tmpCPVel_global.y(), tmpCPVel_global.z(), tmpCPVel_global.norm());

		}
	}
	robot->setState(&rs);
}

