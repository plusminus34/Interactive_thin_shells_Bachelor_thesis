#include <RobotDesignerLib/MOPTQPTrackingController.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>
#include <string>
#include <map>
#include <Utils/Timer.h>
#include <GUILib/GLUtils.h>

using namespace std;

// TODO: add feedforward term for accelerations...
// TODO: smooth out trajectories some more... right now target velocities change discontinuously, which does not seem good
// TODO: implement ability to overide desired body orientation, height, etc. Get a spot mini-like demo, where the body can be controlled nicely while the legs are all on the ground
// TODO: revisit strategy for early or late contacts
// TODO: adapt height of swing legs based on terrain
// TODO: make a robot like this also: http://www.agilityrobotics.com/. 3 dof hip, 1 dof to change leg length, linkage system otherwise...
// TODO: there are potentially still issues due to conflicting goals: if falling, need to take a longer step in the direction of the fall. This can be achieving by moving the body in that direction too, but that makes the robot fall faster! This is why we need to decouple what the body is doing from what swing limbs are doing, which is now implemented through "swingLimbEndEffectors/QPC_SwingLimbJointAccelerationsObjective" targets, which get mapped directly to joint acceleration targets, and not operational space targets.


/**
Demonstrate it on:
- bipeds
- quadrupeds
- high-level tasks, such as line following
- bipeds with nice toe-off motions
*/

/**
show this on an ostrich/t-rex baby (4 motors per leg, 2 * mechanical coupling to get a nice leg shape and foot)
show this on a nice quadruped (2 motors per leg, 1 * mechanical coupling to get animal-like legs, fabricate it, since we can make it conservative enough not to need feedback...)
*/


MOPTQPTrackingController::MOPTQPTrackingController(Robot *robot) {
	this->robot = robot;

	//all the joints of the robots will be controlled via position control, so initialize this mode of operation
	for (uint i = 0; i < robot->jointList.size(); i++)
		robot->jointList[i]->controlMode = TORQUE_MODE;

	qpPlan = new QPControlPlan(robot);
	qpEngine = new QPControlEngine(qpPlan);
}

MOPTQPTrackingController::~MOPTQPTrackingController(void) {

}

//this is the target robot state, copied directly from the motion plan...
ReducedRobotState MOPTQPTrackingController::getTargetRobotState(LocomotionEngineMotionPlan *motionPlan, double stridePhase) {
	if (stridePhase > 1) stridePhase -= 1.0;
	ReducedRobotState desiredState(robot);
	motionPlan->robotStateTrajectory.getRobotStateAt(stridePhase, motionPlan->motionPlanDuration, desiredState);
	return desiredState;
}

//this target mopt state needs to be adapted based on the current configuration of the robot... this method computes the quantities required to perform this retargetting operation...
void MOPTQPTrackingController::computeRobotStateTransferQuantities(LocomotionEngineMotionPlan *motionPlan, double stridePhase, double &moptGroundHeight, double &currentGroundHeight, Quaternion& headingOffset, P3D& moptEEFrameOrigin, P3D&robotEEFrameOrigin) {
	headingOffset = computeHeading(motionPlan->COMTrajectory.getCOMOrientationAt(stridePhase), Globals::worldUp).getComplexConjugate() * computeHeading(robot->root->getOrientation(), Globals::worldUp);
	currentGroundHeight = 0;
	moptGroundHeight = 0;

	int nStanceEEs = 0;

	FootFallPattern ffp;
	motionPlan->syncFootFallPatternWithMotionPlan(ffp);

	//we need to figure out where the body wants to be... expressed relative to the stance feet...
	for (uint i = 0; i < motionPlan->endEffectorTrajectories.size(); i++) {
		GenericLimb* theLimb = motionPlan->endEffectorTrajectories[i].theLimb;
		if (!(theLimb->swingPhase >= 0 && theLimb->swingPhase <= 1)) {
			moptEEFrameOrigin += motionPlan->endEffectorTrajectories[i].getEEPositionAt(stridePhase);
			robotEEFrameOrigin += motionPlan->endEffectorTrajectories[i].endEffectorRB->getWorldCoordinates(motionPlan->endEffectorTrajectories[i].endEffectorLocalCoords);
			nStanceEEs++;
		}
	}
	if (nStanceEEs > 0) {
		moptEEFrameOrigin /= nStanceEEs;
		robotEEFrameOrigin /= nStanceEEs;
	}
	else {
		moptEEFrameOrigin = motionPlan->COMTrajectory.getCOMPositionAt(stridePhase);
		robotEEFrameOrigin = robot->bFrame->bodyState.position;
	}
	moptEEFrameOrigin.y() = moptGroundHeight;
	robotEEFrameOrigin.y() = currentGroundHeight;
}

//computes a stepping offset based on tracking error, in world coordinates...
V3D MOPTQPTrackingController::computeStepOffset(double currentHeight, const V3D& currentVelocity, const V3D& plannedVelocity) {
	V3D vErr = currentVelocity - plannedVelocity;
	vErr.y() = 0;

	return vErr * sqrt(currentHeight / fabs(Globals::g));
}

//based on the motion plan, estimate the target COM velocity...
V3D MOPTQPTrackingController::estimateMOPTCOMVelocity(LocomotionEngineMotionPlan *motionPlan, double stridePhase){
	assert(stridePhase >= 0 && stridePhase <= 1);
	double dStridePhase = 0.01;
	double dt = dStridePhase * motionPlan->motionPlanDuration;

	P3D posNow = motionPlan->COMTrajectory.getCOMPositionAt(stridePhase);
	P3D posFuture = motionPlan->COMTrajectory.getCOMPositionAt(stridePhase + dStridePhase);
	if (stridePhase + dt > 1) {
		posNow = motionPlan->COMTrajectory.getCOMPositionAt(stridePhase - dStridePhase);
		posFuture = motionPlan->COMTrajectory.getCOMPositionAt(stridePhase);
	}

	return V3D(posNow, posFuture) / dt;
}

//based on the motion plan, estimate the velocity for the ith EE...
V3D MOPTQPTrackingController::estimateMOPTEEVelocity(LocomotionEngineMotionPlan *motionPlan, double stridePhase, int EEIndex) {
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



void MOPTQPTrackingController::prepareForControlStep(LocomotionEngineMotionPlan *motionPlan, double stridePhase, double dt) {
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

void MOPTQPTrackingController::computeControlSignals(LocomotionEngineMotionPlan *motionPlan, double stridePhase, double dt) {
	Quaternion headingOffset;
	double groundHeight = 0, moptGroundHeight = 0;
	P3D moptEEFrameOrigin, robotEEFrameOrigin;

	prepareForControlStep(motionPlan, stridePhase, dt);
	ReducedRobotState targetRobotState = getTargetRobotState(motionPlan, stridePhase);
	computeRobotStateTransferQuantities(motionPlan, stridePhase, moptGroundHeight, groundHeight, headingOffset, moptEEFrameOrigin, robotEEFrameOrigin);

	//now we're ready to set various objectives based on the target robot state...
	qpEngine->qpPlan->setBodyPositionAndVelocityTarget(robotEEFrameOrigin + headingOffset.rotate(V3D(moptEEFrameOrigin, targetRobotState.getPosition())), headingOffset.rotate(targetRobotState.getVelocity()), gainBodyPos, gainBodyVel, gainBodyPos, 2 * sqrt(gainBodyPos));
	qpEngine->qpPlan->setBodyOrientationAndAngularVelocityTarget(headingOffset * targetRobotState.getOrientation(), headingOffset.rotate(targetRobotState.getAngularVelocity()), gainBodyOrientation, 2 * sqrt(gainBodyOrientation));
	qpEngine->qpPlan->setJointAngleTargets(targetRobotState, gainJointAngles, 2 * sqrt(gainJointAngles));

//	V3D stepOffset = computeStepOffset(robot->bFrame->bodyState.position.y() - groundHeight, robot->bFrame->bodyState.velocity, headingOffset.rotate(estimateMOPTCOMVelocity(motionPlan, stridePhase)));
	V3D stepOffset = computeStepOffset(robot->bFrame->bodyState.position.y() - groundHeight, robot->root->getCMVelocity(), headingOffset.rotate(targetRobotState.getVelocity()));
	V3D velOffset = robot->root->getCMVelocity() - headingOffset.rotate(targetRobotState.getVelocity());
	velOffset.y() = 0;

	//we read off end effector targets from the motion plan...
	for (uint i = 0; i < motionPlan->endEffectorTrajectories.size(); i++) {
		GenericLimb* theLimb = motionPlan->endEffectorTrajectories[i].theLimb;
		//		theLimb->swingPhase = -1;
		if (theLimb->swingPhase >= 0 && theLimb->swingPhase <= 1) {
			//position of the end effector must be computed in world coords...
			P3D eePos = motionPlan->endEffectorTrajectories[i].getEEPositionAt(stridePhase);
			double swingFootHeight = eePos.y() - moptGroundHeight;
			V3D eeBodyOffset(targetRobotState.getPosition(), eePos);
			//TODO: adjust the height of the feet too, based on the ground height...
			eePos = robot->root->getCMPosition() + headingOffset.rotate(eeBodyOffset);
			eePos.y() = groundHeight + swingFootHeight;
//			Logger::consolePrint("step offset length: %lf\n", stepOffset.length());
			eePos += stepOffset;

			P3D currentEEPos = motionPlan->endEffectorTrajectories[i].endEffectorRB->getWorldCoordinates(motionPlan->endEffectorTrajectories[i].endEffectorLocalCoords);
			double interpValue = theLimb->swingPhase / 0.5;
			boundToRange(interpValue, 0, 1);
			eePos.x() = interpValue * eePos.x() + (1 - interpValue) * currentEEPos.x();
			eePos.z() = interpValue * eePos.z() + (1 - interpValue) * currentEEPos.z();

			qpEngine->qpPlan->addSwingLimbAccelerationConstraint(motionPlan->endEffectorTrajectories[i].endEffectorRB, motionPlan->endEffectorTrajectories[i].endEffectorLocalCoords, eePos, estimateMOPTEEVelocity(motionPlan, stridePhase, i) + velOffset, gainSwingFoot, 2 * sqrt(gainSwingFoot));
//			qpEngine->qpPlan->addEndEffectorTarget(motionPlan->endEffectorTrajectories[i].endEffectorRB, motionPlan->endEffectorTrajectories[i].endEffectorLocalCoords, eePos, estimateMOPTEEVelocity(motionPlan, stridePhase, i) + velOffset, gainSwingFoot, 2 * sqrt(gainSwingFoot));
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
void MOPTQPTrackingController::draw() {
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
