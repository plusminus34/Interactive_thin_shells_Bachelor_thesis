#include "MotionPlanner.h"
#include <MathLib/MathLib.h>
#include <ControlLib/QPControlPlan.h>

/*
	TODO: 
	- learn parameters of first order dynamics model, including acceleration limits, perhaps state dependent
	- right now rotations are ignored, except for the heading
	- add step feedback based on velocity error...
	- learn step feedback parameters as well
	- figure out how to sync the MOPT plan and the pre-plan. They will not be the same, of course, but they mean the same thing, so one shouldn't overreact to mismatches between the two...
	- add tracking controller...
*/

MotionPlanner::MotionPlanner(){
	
}

MotionPlanner::~MotionPlanner(void){
}

void MotionPlanner::preplan(RobotState* currentRobotState) {
	plannerStartState = *currentRobotState;
	RobotState tmpState(robot);
	robot->setState(&plannerStartState);

	//first model the body as a particle moving under the influence of high level goals but with reasonable acceleration limits
	P3D currentBodyPos = currentRobotState->getPosition();
	V3D currentBodyVel = currentRobotState->getVelocity();
	double currentHeading = currentRobotState->getHeading();
	double currentTurningSpeed = currentRobotState->getAngularVelocity().dot(Globals::worldUp);

	prePlanBodyTrajectory.clear();
	prePlanBodyVelocityTrajectory.clear();
	prePlanHeadingTrajectory.clear();
	prePlanTurningSpeedTrajectory.clear();

	//we will be adding samples every 0.1s
	double dt = 0.1;
	for (double t = motionPlanStartTime; t <= motionPlanStartTime + preplanTimeHorizon; t += dt) {
		prePlanBodyTrajectory.addKnot(t, V3D() + currentBodyPos);
		prePlanBodyVelocityTrajectory.addKnot(t, currentBodyVel);
		prePlanHeadingTrajectory.addKnot(t, currentHeading);
		prePlanTurningSpeedTrajectory.addKnot(t, currentTurningSpeed);

		V3D targetSpeed =
			robot->forward * forwardSpeedTarget +
			robot->right * sidewaysSpeedTarget;

		double kp = 100;
		double kd = 20;

		targetSpeed = targetSpeed.rotate(currentHeading, Globals::worldUp);

		V3D acceleration = (targetSpeed - currentBodyVel) / dt;

		acceleration.setComponentAlong(Globals::worldUp,
			getTargetAcceleration_implicitPD(currentBodyPos.getComponentAlong(Globals::worldUp) - bodyHeightTarget, currentBodyVel.getComponentAlong(Globals::worldUp), kp, kd, dt)
		);

		boundToRange(acceleration.x(), -1, 1);
		boundToRange(acceleration.y(), -1, 1);
		boundToRange(acceleration.z(), -1, 1);

		currentBodyVel += acceleration * dt;
		currentBodyPos += currentBodyVel * dt;

//		Logger::consolePrint("t: %lf, vel: %lf\n", t, currentBodyVel.z());

		double headingAccel = (turningSpeedTarget - currentTurningSpeed) / dt;
		boundToRange(headingAccel, -1, 1);
		currentTurningSpeed += headingAccel * dt;
		currentHeading += currentTurningSpeed * dt;
	}

	//now handle the foot locations...

	prePlanEETrajectories.clear();

	cffp = ContinuousFootFallPattern();

	auto eeTraj = locomotionManager->motionPlan->endEffectorTrajectories;
	for (uint i = 0; i < eeTraj.size(); i++) {
		prePlanEETrajectories.push_back(Trajectory3D());
		prePlanEETrajectories[i].addKnot(motionPlanStartTime, V3D() + eeTraj[i].endEffectorRB->getWorldCoordinates(eeTraj[i].endEffectorLocalCoords));
		cffp.addStepPattern(eeTraj[i].theLimb, eeTraj[i].endEffectorRB, eeTraj[i].endEffectorLocalCoords);
	}

	cffp.populateFrom(defaultFootFallPattern, moptParams.motionPlanDuration, motionPlanStartTime - moptParams.motionPlanDuration, motionPlanStartTime + preplanTimeHorizon);
	for (uint eeIndex = 0; eeIndex < eeTraj.size(); eeIndex++) {
		//we must determine the stance phases for each limb
		double t = motionPlanStartTime;
		bool eeStartsInStance = false;

		while (t < motionPlanStartTime + preplanTimeHorizon){
			double stancePhaseStart = cffp.stepPatterns[eeIndex].getFirstTimeInStanceAfter(t);
			double stancePhaseEnd = cffp.stepPatterns[eeIndex].getFirstTimeInSwingAfter(stancePhaseStart+0.01);

			//make sure there is another swing phase to reason about...
			if (stancePhaseEnd > stancePhaseStart) {
				//we will place the stance end effectors in the middle of the trajectory generated by the body...
				P3D localCoordsEEPos = eeTraj[eeIndex].rootToEEOriginalOffset_local;
				P3D eePosAtStancePhaseStart = prePlanBodyTrajectory.evaluate_linear(stancePhaseStart) + (V3D() + localCoordsEEPos).rotate(prePlanHeadingTrajectory.evaluate_linear(stancePhaseStart), Globals::worldUp);
				P3D eePosAtStancePhaseEnd = prePlanBodyTrajectory.evaluate_linear(stancePhaseEnd) + (V3D() + localCoordsEEPos).rotate(prePlanHeadingTrajectory.evaluate_linear(stancePhaseEnd), Globals::worldUp);

				//now, decide where this stance location should be in world coordinates (assuming the leg wasn't in stance to begin with)
				if (stancePhaseStart > motionPlanStartTime) {
					P3D worldEEPos = (eePosAtStancePhaseStart + eePosAtStancePhaseEnd) / 2.0;
					worldEEPos.setComponentAlong(Globals::worldUp, 0);
					prePlanEETrajectories[eeIndex].addKnot(stancePhaseStart, worldEEPos);
					prePlanEETrajectories[eeIndex].addKnot(stancePhaseEnd, worldEEPos);
				}
				else {
					eeStartsInStance = true;
					//the leg started off in stance, so make sure it remains in stance long enough...
					prePlanEETrajectories[eeIndex].addKnot(stancePhaseEnd, V3D() + eeTraj[eeIndex].endEffectorRB->getWorldCoordinates(eeTraj[eeIndex].endEffectorLocalCoords));
				}

				t = stancePhaseEnd + 0.01;
			}
			else
				t = motionPlanStartTime + preplanTimeHorizon + 0.01;
		}

		//the stance phases are all in there, so now add the swing phases too...
		int count;
		if (eeStartsInStance) 
			count = 0;
		else {
			count = 1;
			//if the end effector starts out in swing, then we need to make sure the height profile is still ok
			double swingPhaseAtStart = cffp.stepPatterns[eeIndex].getSwingPhase(motionPlanStartTime);
			if (swingPhaseAtStart < 0) Logger::consolePrint("Uh-oh, we have a problem...\n");
			if (swingPhaseAtStart < 0.5) {
				double timeToGroundStrike = cffp.stepPatterns[eeIndex].getFirstTimeInStanceAfter(motionPlanStartTime) - motionPlanStartTime;
				double swingDuration = timeToGroundStrike / (1 - swingPhaseAtStart);
//				Logger::consolePrint("swing duration: %lf\n", swingDuration);
				double timeToMidSwing = motionPlanStartTime + timeToGroundStrike - 0.5 * swingDuration;
				double w1 = (0.5 - swingPhaseAtStart) / (1 - swingPhaseAtStart);
				double w2 = 1 - w1;
				P3D eePosMidSwing = prePlanEETrajectories[eeIndex].getKnotValue(0) * (1-w1) + prePlanEETrajectories[eeIndex].getKnotValue(1) * (1-w2);
				eePosMidSwing.setComponentAlong(Globals::worldUp, moptParams.swingFootHeight);
				prePlanEETrajectories[eeIndex].addKnot(timeToMidSwing, V3D() + eePosMidSwing);
				count = 2;
			}
		}
		for (count; count < prePlanEETrajectories[eeIndex].getKnotCount()-2; count+=3) {
			P3D eePosMidSwing = (prePlanEETrajectories[eeIndex].getKnotValue(count+1) + prePlanEETrajectories[eeIndex].getKnotValue(count+2))/2.0;
			double t = (prePlanEETrajectories[eeIndex].getKnotPosition(count + 1) + prePlanEETrajectories[eeIndex].getKnotPosition(count + 2)) / 2.0;
			eePosMidSwing.setComponentAlong(Globals::worldUp, moptParams.swingFootHeight);
			prePlanEETrajectories[eeIndex].addKnot(t, V3D() + eePosMidSwing);
		}
	}

	robot->setState(&tmpState);
}

RobotState MotionPlanner::getPreplanedRobotStateAtTime(double t) {
	RobotState oldState(robot);
	RobotState newState(robot);
	newState.setPosition(prePlanBodyTrajectory.evaluate_linear(t));
	newState.setOrientation(getRotationQuaternion(prePlanHeadingTrajectory.evaluate_linear(t), Globals::worldUp));
	newState.setVelocity(prePlanBodyVelocityTrajectory.evaluate_linear(t));
	newState.setAngularVelocity(Globals::worldUp * prePlanTurningSpeedTrajectory.evaluate_linear(t));
	robot->setState(&newState);

	IK_Solver ikSolver(robot);
	ikSolver.ikPlan->setTargetIKStateFromRobot();
	ikSolver.ikPlan->optimizeRootConfiguration = false;

	int nLegs = robot->bFrame->limbs.size();
	for (uint eeIndex = 0; eeIndex < prePlanEETrajectories.size(); eeIndex++) {
		ikSolver.ikPlan->endEffectors.push_back(IK_EndEffector());
		ikSolver.ikPlan->endEffectors.back().endEffectorLocalCoords = cffp.stepPatterns[eeIndex].eeLocalCoords;
		ikSolver.ikPlan->endEffectors.back().endEffectorRB = cffp.stepPatterns[eeIndex].eeRB;
		ikSolver.ikPlan->endEffectors.back().targetEEPos = prePlanEETrajectories[eeIndex].evaluate_linear(t);
	}

	ikSolver.ikEnergyFunction->setupSubObjectives();
	ikSolver.ikEnergyFunction->regularizer = 1;
	ikSolver.solve(20);

	newState = RobotState(robot);
	robot->setState(&oldState);
	return newState;
}

void MotionPlanner::draw() {

	glColor4d(1,0,0,0.2);

	RobotState tmpState(robot);
	robot->setState(&plannerStartState);
	robot->draw(SHOW_ABSTRACT_VIEW);
	robot->setState(&tmpState);

//	drawArrow(P3D(0, 1, 0), V3D(-1, 0, 0), 0.01);

	double dt = 0.1;
	glBegin(GL_LINE_STRIP);
	for (double t = motionPlanStartTime; t <= motionPlanStartTime + preplanTimeHorizon; t += dt) {
		P3D pos = prePlanBodyTrajectory.evaluate_linear(t);
		glVertex3d(pos.x(), pos.y(), pos.z());
	}
	glEnd();

	glColor4d(0, 0, 1, 0.2);
	dt = 0.001;
	for (uint i = 0; i < prePlanEETrajectories.size(); i++) {
		glBegin(GL_LINE_STRIP);
		for (double t = motionPlanStartTime; t <= motionPlanStartTime + preplanTimeHorizon; t += dt) {
			P3D pos = prePlanEETrajectories[i].evaluate_linear(t);
			glVertex3d(pos.x(), pos.y(), pos.z());
		}
		glEnd();
	}

	glColor4d(1, 0, 0, 0.2);
	dt = preplanTimeHorizon / 20.0;
	for (double t = motionPlanStartTime; t <= motionPlanStartTime + preplanTimeHorizon; t += dt) {
		P3D pos = prePlanBodyTrajectory.evaluate_linear(t);
		drawSphere(pos, 0.01);
		V3D forward = robot->forward.rotate(prePlanHeadingTrajectory.evaluate_linear(t), Globals::worldUp);
		drawArrow(pos, forward*0.05, 0.0025);
	}
}

void MotionPlanner::prepareMOPTPlan(LocomotionEngineMotionPlan* motionPlan) {
	//the motion plan will be synced with the start of the motion pre-plan
	motionPlan->motionPlanDuration = moptParams.motionPlanDuration;
	motionPlan->wrapAroundBoundaryIndex = -1;

	double h = motionPlan->motionPlanDuration / (motionPlan->nSamplePoints - 1);

	for (int i = 0; i < motionPlan->nSamplePoints; i++) {
		double t = motionPlanStartTime + h * i;
		//take care of contact flags and end effector trajectories
		for (uint j = 0; j < motionPlan->endEffectorTrajectories.size(); j++) {
			auto eeTraj = &motionPlan->endEffectorTrajectories[j];
			eeTraj->contactForce[i] = V3D();
			eeTraj->EEPos[i] = prePlanEETrajectories[j].evaluate_linear(t);

			bool inStanceNow = cffp.stepPatterns[j].isInStanceAt(t);
			bool inStanceNext = cffp.stepPatterns[j].isInStanceAt(t + h / 2);

			if (inStanceNow && !inStanceNext) {				//transitioning from stance to swing right now
				eeTraj->contactFlag[i] = false;
			} else if (!inStanceNow && inStanceNext)		//transitioning from swing to stance
				eeTraj->contactFlag[i] = true;
			else
				eeTraj->contactFlag[i] = (inStanceNow)?1:0;

			if (eeTraj->contactFlag[i] != 0)
				eeTraj->contactForce[i].y() = motionPlan->verticalGRFLowerBoundVal + motionPlan->GRFEpsilon * 1.1;
		}

		P3D comPos = prePlanBodyTrajectory.evaluate_linear(t);
		for (int k=0;k<3;k++)
			motionPlan->bodyTrajectory.pos[k][i] = motionPlan->bodyTrajectory.desiredPos[k][i] = comPos[k];

//		TODO: should this be the real orientation, coming from the robot, with some first order dynamics?!?
		motionPlan->bodyTrajectory.orientation[0][i] = motionPlan->bodyTrajectory.desiredOrientation[0][i] = prePlanHeadingTrajectory.evaluate_linear(t);
		motionPlan->bodyTrajectory.orientation[1][i] = motionPlan->bodyTrajectory.orientation[2][i] = 0;
		motionPlan->bodyTrajectory.desiredOrientation[1][i] = motionPlan->bodyTrajectory.desiredOrientation[2][i] = 0;
	}

	motionPlan->bodyTrajectory.useInitialVelocities = true;
	motionPlan->bodyTrajectory.initialLinearVelocity = plannerStartState.getVelocity();
	motionPlan->bodyTrajectory.initialAngularVelocity = plannerStartState.getAngularVelocity();

	motionPlan->syncFootFallPatternWithMotionPlan(moptFootFallPattern);
}

LocomotionEngineManager* MotionPlanner::initializeMOPTEngine() {
	delete locomotionManager;

	locomotionManager = new LocomotionEngineManagerGRFv3(robot, &moptFootFallPattern, moptFootFallPattern.strideSamplePoints + 1);

	bodyHeightTarget = locomotionManager->motionPlan->initialRS.getPosition().getComponentAlong(Globals::worldUp);
	robot->setState(&locomotionManager->motionPlan->initialRS);

	locomotionManager->setDefaultOptimizationFlags();
	locomotionManager->energyFunction->regularizer = globalMOPTRegularizer;

	return locomotionManager;
}

void MotionPlanner::generateMotionPlan() {
	locomotionManager->printDebugInfo = false;
	locomotionManager->checkDerivatives = false;

	Timer t;
	if (defaultFootFallPattern.stepPatterns.size() < moptFootFallPattern.stepPatterns.size())
		defaultFootFallPattern = moptFootFallPattern;

	RobotState rs(robot);
	preplan(&rs);
	prepareMOPTPlan(locomotionManager->motionPlan);

	locomotionManager->runMOPTStep(OPT_GRFS);
	double energyVal = 0;
	for (int i = 0; i<10; i++)
		energyVal = locomotionManager->runMOPTStep(OPT_GRFS | OPT_COM_POSITIONS);

	Logger::consolePrint("It took %lfs to generate motion plan, final energy value: %lf\n", t.timeEllapsed(), energyVal);
}

void MotionPlanner::advanceMotionPlanGlobalTime(int nSteps) {
	//we are using a discrete number of steps to keep the (discrete) footfall pattern consistent. Otherwise we'd need to interpolate between stance/swing phases and there are no good answers...
	motionPlanStartTime += nSteps * moptParams.motionPlanDuration / (locomotionManager->motionPlan->nSamplePoints - 1);
}
