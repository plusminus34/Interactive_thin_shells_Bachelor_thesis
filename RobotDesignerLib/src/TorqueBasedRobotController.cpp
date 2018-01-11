#include <RobotDesignerLib/TorqueBasedRobotController.h>
#include <GUILib/GLUtils.h>
#include <RBSimLib/AbstractRBEngine.h>

TorqueBasedRobotController::TorqueBasedRobotController(Robot *robot, LocomotionEngineMotionPlan *motionPlan) : RobotController(robot, motionPlan) {
	totalTime = 0;

	this->robot = robot;
	this->motionPlan = motionPlan;

	controller = new MOPTQPTrackingController(robot);
}

void TorqueBasedRobotController::initialize() {
	//all the joints of the robots will be controlled via position control, so initialize this mode of operation
	for (uint i = 0; i < robot->jointList.size(); i++)
		robot->jointList[i]->controlMode = TORQUE_MODE;
	stridePhase = 0;
	RobotState rs = controller->getTargetRobotState(motionPlan, stridePhase);
	robot->setState(&rs);
}

TorqueBasedRobotController::~TorqueBasedRobotController(void){
}

void TorqueBasedRobotController::drawDebugInfo() {
	controller->draw();
}

void TorqueBasedRobotController::computeControlSignals(double timeStep){
	robot->bFrame->updateLimbGroundContactInformation();
	robot->bFrame->updateStateInformation();
	controller->computeControlSignals(motionPlan, stridePhase, timeStep);
}

void TorqueBasedRobotController::applyControlSignals(double timeStep) {
	GeneralizedCoordinatesRobotRepresentation gcrr(robot);
	gcrr.computeWorldCoordinateTorquesFromU(controller->qpPlan->u);
}


