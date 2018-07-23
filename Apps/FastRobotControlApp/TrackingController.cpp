#include "TrackingController.h"
#include <GUILib/GLUtils.h>
#include <RBSimLib/AbstractRBEngine.h>

TrackingController::TrackingController(Robot *robot, LocomotionEngineMotionPlan *motionPlan) : RobotController(robot, motionPlan) {
	this->robot = robot;
	this->motionPlan = motionPlan;

	controller = new SimpleMOPTTrackingController(robot);
}

void TrackingController::initialize() {
	//all the joints of the robots will be controlled via position control, so initialize this mode of operation
	for (uint i = 0; i < robot->jointList.size(); i++)
		robot->jointList[i]->controlMode = TORQUE_MODE;
//	stridePhase = 0;
//	RobotState rs = motionPlan->initialRS;
//	robot->setState(&rs);
}

TrackingController::~TrackingController(void){
}

void TrackingController::drawDebugInfo() {
	controller->draw();
}

void TrackingController::computeControlSignals(double timeStep){
	robot->bFrame->updateLimbGroundContactInformation();
	robot->bFrame->updateStateInformation();
	controller->computeControlSignals(motionPlan, stridePhase, timeStep);
}

void TrackingController::applyControlSignals(double timeStep) {
	GeneralizedCoordinatesRobotRepresentation gcrr(robot);
	gcrr.computeWorldCoordinateTorquesFromU(controller->qpPlan->u);
}


