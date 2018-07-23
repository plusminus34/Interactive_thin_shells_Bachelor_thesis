#include "SimpleMOPTTrackingController.h"
#include <string>
#include <map>
#include <Utils/Timer.h>
#include <GUILib/GLUtils.h>

using namespace std;

// TODO: add feedforward term for accelerations...
// TODO: revisit strategy for early or late contacts
// TODO: make a robot like this also: http://www.agilityrobotics.com/. 3 dof hip, 1 dof to change leg length, linkage system otherwise...
// TODO: there are potentially still issues due to conflicting goals: if falling, need to take a longer step in the direction of the fall. This can be achieving by moving the body in that direction too, but that makes the robot fall faster! This is why we need to decouple what the body is doing from what swing limbs are doing, which is now implemented through "swingLimbEndEffectors/QPC_SwingLimbJointAccelerationsObjective" targets, which get mapped directly to joint acceleration targets, and not operational space targets.


SimpleMOPTTrackingController::SimpleMOPTTrackingController(Robot *robot) {
	this->robot = robot;

	//all the joints of the robots will be controlled via position control, so initialize this mode of operation
	for (uint i = 0; i < robot->jointList.size(); i++)
		robot->jointList[i]->controlMode = TORQUE_MODE;

	qpPlan = new QPControlPlan(robot);
	qpEngine = new QPControlEngine(qpPlan);
}

SimpleMOPTTrackingController::~SimpleMOPTTrackingController(void) {

}

//based on the motion plan, estimate the velocity for the ith EE...
V3D SimpleMOPTTrackingController::estimateMOPTEEVelocity(LocomotionEngineMotionPlan *motionPlan, double stridePhase, int EEIndex) {
	assert(stridePhase >= 0 && stridePhase <= 1);
	double dStridePhase = 0.01;
	double dt = dStridePhase * motionPlan->motionPlanDuration;

	P3D posNow = motionPlan->endEffectorTrajectories[EEIndex].getEEPositionAt(stridePhase);
	P3D posFuture = motionPlan->endEffectorTrajectories[EEIndex].getEEPositionAt(stridePhase + dStridePhase);
	if (stridePhase + dt > 1) {
		posNow = motionPlan->endEffectorTrajectories[EEIndex].getEEPositionAt(stridePhase - dStridePhase);
		posFuture = motionPlan->endEffectorTrajectories[EEIndex].getEEPositionAt(stridePhase);
	}

	return V3D(posNow, posFuture) / dt;
}

void SimpleMOPTTrackingController::prepareForControlStep(LocomotionEngineMotionPlan *motionPlan, double stridePhase, double dt) {
	qpEngine->qpPlan->dt = dt;
	qpEngine->qpPlan->robotRepresentation->syncGeneralizedCoordinatesWithRobotState();
	//update swing phases, grounded status, etc...
	qpEngine->qpPlan->clearEndEffectorList();
	qpEngine->qpPlan->initializeControlStep();

	FootFallPattern ffp;
	motionPlan->syncFootFallPatternWithMotionPlan(ffp);

	//we need to figure out where the body wants to be... expressed relative to the stance feet...
	for (uint i = 0; i < motionPlan->endEffectorTrajectories.size(); i++) {
		GenericLimb* theLimb = motionPlan->endEffectorTrajectories[i].theLimb;
		theLimb->swingPhase = ffp.getSwingPhaseForMotionPhase(theLimb, stridePhase);
	}
}

void SimpleMOPTTrackingController::computeControlSignals(LocomotionEngineMotionPlan *motionPlan, double stridePhase, double dt) {
	prepareForControlStep(motionPlan, stridePhase, dt);
	RobotState targetRobotState = motionPlan->initialRS;
	double groundHeight = 0;
	double footSize = 0.01;

	//now we're ready to set various objectives based on the target robot state...
	qpEngine->qpPlan->setBodyPositionAndVelocityTarget(motionPlan->bodyTrajectory.getCOMPositionAt(stridePhase), motionPlan->bodyTrajectory.getCOMVelocityAt(stridePhase), gainBodyPos, gainBodyVel, gainBodyPos, 2 * sqrt(gainBodyPos));
	qpEngine->qpPlan->setBodyOrientationAndAngularVelocityTarget(motionPlan->bodyTrajectory.getCOMOrientationAt(stridePhase), motionPlan->bodyTrajectory.getCOMAngularVelocityAt(stridePhase), gainBodyOrientation, 2 * sqrt(gainBodyOrientation));
	qpEngine->qpPlan->setJointAngleTargets(targetRobotState, gainJointAngles, 2 * sqrt(gainJointAngles));

	//we read off end effector targets from the motion plan...
	for (uint i = 0; i < motionPlan->endEffectorTrajectories.size(); i++) {
		GenericLimb* theLimb = motionPlan->endEffectorTrajectories[i].theLimb;
		if (theLimb->swingPhase >= 0 && theLimb->swingPhase <= 1) {
			//position of the end effector must be computed in world coords...
			P3D eePos = motionPlan->endEffectorTrajectories[i].getEEPositionAt(stridePhase);
			eePos.y() += groundHeight + footSize;

			qpEngine->qpPlan->addSwingLimbAccelerationConstraint(motionPlan->endEffectorTrajectories[i].endEffectorRB, motionPlan->endEffectorTrajectories[i].endEffectorLocalCoords, eePos, estimateMOPTEEVelocity(motionPlan, stridePhase, i), gainSwingFoot, 2 * sqrt(gainSwingFoot));
		}
		else {
			qpEngine->qpPlan->addContactEndEffector(motionPlan->endEffectorTrajectories[i].endEffectorRB, motionPlan->endEffectorTrajectories[i].endEffectorLocalCoords, motionPlan->endEffectorTrajectories[i].theLimb->inContact);
		}
	}

	if (doDebug) {
		testGeneralizedCoordinateRepresentation(robot);
		dVector p;
		qpEngine->qpPlan->writeParametersToList(p);
		qpEngine->energyFunction->testGradientWithFD(p);
		qpEngine->energyFunction->testHessianWithFD(p);
	}

	Timer timer;

	qpEngine->qpPlan->computeSwingLimbJointAccelerationTargets();
	qpEngine->energyFunction->printDebugInfo = doDebug;
	qpEngine->optimizePlan();

	if (doDebug) {
		Logger::consolePrint("total time ellapsed for QP solve: %lfs\n", timer.timeEllapsed());
		string s_u, s_qDotDot;
		for (int i = 6; i < qpEngine->qpPlan->u.size(); i++){
			s_u += to_string(qpEngine->qpPlan->u[i]) + " ";
			s_qDotDot += to_string(qpEngine->qpPlan->u[i]) + " ";
		}
		Logger::consolePrint("u: %s\n", s_u.c_str());
		Logger::consolePrint("qDotDot: %s\n", s_qDotDot.c_str());
	}
	qpPlan->robotRepresentation->computeWorldCoordinateTorquesFromU(qpPlan->u);
}

//draws debugging info...
void SimpleMOPTTrackingController::draw() {
	for (uint i = 0; i < qpPlan->contactEndEffectors.size(); i++) {
		if (qpPlan->contactEndEffectors[i].inContact)
			glColor3d(0.0, 1.0, 0);
		else
			glColor3d(1.0, 0.0, 0);
		drawSphere(qpPlan->contactEndEffectors[i].targetPosition, 0.01, 12);
	}

	for (uint i = 0; i < qpPlan->generalEndEffectors.size(); i++) {
		glColor3d(0.0, 0.0, 1.0);
		drawSphere(qpPlan->generalEndEffectors[i].targetPosition, 0.01, 12);
	}

	for (uint i = 0; i < qpPlan->swingLimbEndEffectors.size(); i++) {
		glColor3d(1.0, 0.0, 0.0);
		drawSphere(qpPlan->swingLimbEndEffectors[i].targetPosition, 0.01, 12);
	}

	glPushMatrix();
	glEnable(GL_LIGHTING);

//	Logger::consolePrint("-----------------------------------------\n");
	glTranslated(1,0,0);
	glColor3d(1,0,0);
	drawSphere(robot->root->getCMPosition(), 0.02, 12);
	drawArrow(robot->root->getCMPosition(), robot->root->getCMPosition() + robot->root->getCMVelocity(), 0.01, 12);
//	Logger::consolePrint("red: current robot state\n");

	glColor3d(0, 1, 0);
	drawSphere(qpPlan->targetBodyPos, 0.02, 12);
	drawArrow(qpPlan->targetBodyPos, qpPlan->targetBodyPos + qpPlan->targetBodyVel, 0.01, 12);
//	Logger::consolePrint("green: target robot state\n");

	glColor3d(0, 0, 1);
	drawSphere(qpPlan->predictedBodyPos, 0.02, 12);
	drawArrow(qpPlan->predictedBodyPos, qpPlan->predictedBodyPos + qpPlan->predictedBodyVel, 0.01, 12);
//	Logger::consolePrint("blue: predicted robot state, if it was evolving with the target acceleration...\n");

	glColor3d(0.5, 0.5, 0.5);
	drawSphere(qpPlan->startingBodyPos, 0.02, 12);
	drawArrow(qpPlan->startingBodyPos, qpPlan->startingBodyPos + qpPlan->startingBodyVel, 0.01, 12);
//	Logger::consolePrint("gray: old/starting robot state\n");

	glPopMatrix();
}
