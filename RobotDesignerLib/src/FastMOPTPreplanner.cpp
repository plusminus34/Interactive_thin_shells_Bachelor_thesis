#include <RobotDesignerLib/FastMOPTPreplanner.h>
#include <MathLib/MathLib.h>
#include <ControlLib/QPControlPlan.h>

/*
	TODO: 
	- learn parameters of first order dynamics model, including acceleration limits, perhaps state dependent
	- add step feedback based on velocity error...
	- learn step feedback parameters as well
	- add MOPT based on pre-plan
	- figure out how to sync the MOPT plan and the pre-plan. They will not be the same, of course, but they mean the same thing, so one shouldn't overreact to mismatches between the two...
	- add tracking controller...
*/

FastMOPTPreplanner::FastMOPTPreplanner(FastMOPTWindow* moptWindow){
	this->moptWindow = moptWindow;
}

FastMOPTPreplanner::~FastMOPTPreplanner(void){
}

void FastMOPTPreplanner::preplan(RobotState* currentRobotState) {
	startState = *currentRobotState;
	RobotState tmpState(moptWindow->robot);
	moptWindow->robot->setState(&startState);

	//first model the body as a particle moving under the influence of high level goals but with reasonable acceleration limits
	P3D currentBodyPos = currentRobotState->getPosition();
	V3D currentBodyVel = currentRobotState->getVelocity();
	double currentHeading = currentRobotState->getHeading();
	double currentTurningSpeed = currentRobotState->getAngularVelocity().dot(Globals::worldUp);

	initialLinearVelocity = currentRobotState->getVelocity();
	initialAngularVelocity = currentRobotState->getAngularVelocity();
	initialOrientation = currentRobotState->getOrientation();

	bodyTrajectory.clear();
	bodyVelocityTrajectory.clear();
	headingTrajectory.clear();
	turningSpeedTrajectory.clear();

	//we will be adding samples every 0.1s
	double dt = 0.1;
	for (double t = moptWindow->motionPlanStartTime; t <= moptWindow->motionPlanStartTime + moptWindow->preplanTimeHorizon; t += dt) {
		bodyTrajectory.addKnot(t, V3D() + currentBodyPos);
		bodyVelocityTrajectory.addKnot(t, currentBodyVel);
		headingTrajectory.addKnot(t, currentHeading);
		turningSpeedTrajectory.addKnot(t, currentTurningSpeed);

		V3D targetSpeed =
			moptWindow->robot->forward * moptWindow->forwardSpeedTarget +
			moptWindow->robot->right * moptWindow->sidewaysSpeedTarget;

		double kp = 100;
		double kd = 20;

		targetSpeed = targetSpeed.rotate(currentHeading, Globals::worldUp);

		V3D acceleration = (targetSpeed - currentBodyVel) / dt;

		acceleration.setComponentAlong(Globals::worldUp,
			getTargetAcceleration_implicitPD(currentBodyPos.getComponentAlong(Globals::worldUp) - moptWindow->bodyHeightTarget, currentBodyVel.getComponentAlong(Globals::worldUp), kp, kd, dt)
		);

		boundToRange(acceleration.x(), -1, 1);
		boundToRange(acceleration.y(), -1, 1);
		boundToRange(acceleration.z(), -1, 1);

		currentBodyVel += acceleration * dt;
		currentBodyPos += currentBodyVel * dt;

//		Logger::consolePrint("t: %lf, vel: %lf\n", t, currentBodyVel.z());

		double headingAccel = (moptWindow->turningSpeedTarget - currentTurningSpeed) / dt;
		boundToRange(headingAccel, -1, 1);
		currentTurningSpeed += headingAccel * dt;
		currentHeading += currentTurningSpeed * dt;
	}

	//now handle the foot locations...

	eeTrajectories.clear();

	cffp = ContinuousFootFallPattern();

	auto eeTraj = moptWindow->locomotionManager->motionPlan->endEffectorTrajectories;
	for (uint i = 0; i < eeTraj.size(); i++) {
		eeTrajectories.push_back(Trajectory3D());
		eeTrajectories[i].addKnot(moptWindow->motionPlanStartTime, V3D() + eeTraj[i].endEffectorRB->getWorldCoordinates(eeTraj[i].endEffectorLocalCoords));
		cffp.addStepPattern(eeTraj[i].theLimb, eeTraj[i].endEffectorRB, eeTraj[i].endEffectorLocalCoords);
	}

	cffp.populateFrom(moptWindow->defaultFootFallPattern, moptWindow->moptParams.motionPlanDuration, moptWindow->motionPlanStartTime - moptWindow->moptParams.motionPlanDuration, moptWindow->motionPlanStartTime + moptWindow->preplanTimeHorizon);
	for (uint eeIndex = 0; eeIndex < eeTraj.size(); eeIndex++) {
		//we must determine the stance phases for each limb
		double t = moptWindow->motionPlanStartTime;
		bool eeStartsInStance = false;

		while (t < moptWindow->motionPlanStartTime + moptWindow->preplanTimeHorizon){
			double stancePhaseStart = cffp.stepPatterns[eeIndex].getFirstTimeInStanceAfter(t);
			double stancePhaseEnd = cffp.stepPatterns[eeIndex].getFirstTimeInSwingAfter(stancePhaseStart+0.01);

			//make sure there is another swing phase to reason about...
			if (stancePhaseEnd > stancePhaseStart) {

				//we will place the stance end effectors in the middle of the trajectory generated by the body...
				P3D localCoordsEEPos = eeTraj[eeIndex].rootToEEOriginalOffset_local;
				P3D eePosAtStancePhaseStart = bodyTrajectory.evaluate_linear(stancePhaseStart) + (V3D() + localCoordsEEPos).rotate(headingTrajectory.evaluate_linear(stancePhaseStart), Globals::worldUp);
				P3D eePosAtStancePhaseEnd = bodyTrajectory.evaluate_linear(stancePhaseEnd) + (V3D() + localCoordsEEPos).rotate(headingTrajectory.evaluate_linear(stancePhaseEnd), Globals::worldUp);

				//now, decide where this stance location should be in world coordinates (assuming the leg wasn't in stance to begin with)
				if (stancePhaseStart > moptWindow->motionPlanStartTime) {
					P3D worldEEPos = (eePosAtStancePhaseStart + eePosAtStancePhaseEnd) / 2.0;
					worldEEPos.setComponentAlong(Globals::worldUp, 0);
					eeTrajectories[eeIndex].addKnot(stancePhaseStart, worldEEPos);
					eeTrajectories[eeIndex].addKnot(stancePhaseEnd, worldEEPos);
				}
				else {
					eeStartsInStance = true;
					//the leg started off in stance, so make sure it remains in stance long enough...
					eeTrajectories[eeIndex].addKnot(stancePhaseEnd, V3D() + eeTraj[eeIndex].endEffectorRB->getWorldCoordinates(eeTraj[eeIndex].endEffectorLocalCoords));
				}

				t = stancePhaseEnd + 0.01;
			}
			else
				t = moptWindow->motionPlanStartTime + moptWindow->preplanTimeHorizon + 0.01;
		}

		//the stance phases are all in there, so now add the swing phases too...
		int count;
		if (eeStartsInStance) 
			count = 0;
		else {
			count = 1;
			//if the end effector starts out in swing, then we need to make sure the height profile is still ok
			double swingPhaseAtStart = cffp.stepPatterns[eeIndex].getSwingPhase(moptWindow->motionPlanStartTime);
			if (swingPhaseAtStart < 0) Logger::consolePrint("Uh-oh, we have a problem...\n");
			if (swingPhaseAtStart < 0.5) {
				double timeToGroundStrike = cffp.stepPatterns[eeIndex].getFirstTimeInStanceAfter(moptWindow->motionPlanStartTime) - moptWindow->motionPlanStartTime;
				double swingDuration = timeToGroundStrike / (1 - swingPhaseAtStart);
//				Logger::consolePrint("swing duration: %lf\n", swingDuration);
				double timeToMidSwing = moptWindow->motionPlanStartTime + timeToGroundStrike - 0.5 * swingDuration;
				double w1 = (0.5 - swingPhaseAtStart) / (1 - swingPhaseAtStart);
				double w2 = 1 - w1;
				P3D eePosMidSwing = eeTrajectories[eeIndex].getKnotValue(0) * (1-w1) + eeTrajectories[eeIndex].getKnotValue(1) * (1-w2);
				eePosMidSwing.setComponentAlong(Globals::worldUp, moptWindow->moptParams.swingFootHeight);
				eeTrajectories[eeIndex].addKnot(timeToMidSwing, V3D() + eePosMidSwing);
				count = 2;
			}
		}
		for (count; count < eeTrajectories[eeIndex].getKnotCount()-2; count+=3) {
			P3D eePosMidSwing = (eeTrajectories[eeIndex].getKnotValue(count+1) + eeTrajectories[eeIndex].getKnotValue(count+2))/2.0;
			double t = (eeTrajectories[eeIndex].getKnotPosition(count + 1) + eeTrajectories[eeIndex].getKnotPosition(count + 2)) / 2.0;
			eePosMidSwing.setComponentAlong(Globals::worldUp, moptWindow->moptParams.swingFootHeight);
			eeTrajectories[eeIndex].addKnot(t, V3D() + eePosMidSwing);
		}
	}

	moptWindow->robot->setState(&tmpState);
}

RobotState FastMOPTPreplanner::getRobotStateAtTime(double t) {
	Robot* robot = moptWindow->robot;
	RobotState oldState(robot);
	RobotState newState(robot);
	newState.setPosition(bodyTrajectory.evaluate_linear(t));
	newState.setOrientation(getRotationQuaternion(headingTrajectory.evaluate_linear(t), Globals::worldUp));
	newState.setVelocity(bodyVelocityTrajectory.evaluate_linear(t));
	newState.setAngularVelocity(Globals::worldUp * turningSpeedTrajectory.evaluate_linear(t));
	robot->setState(&newState);

	IK_Solver ikSolver(robot);
	ikSolver.ikPlan->setTargetIKStateFromRobot();
	ikSolver.ikPlan->optimizeRootConfiguration = false;

	int nLegs = robot->bFrame->limbs.size();
	for (uint eeIndex = 0; eeIndex < eeTrajectories.size(); eeIndex++) {
		ikSolver.ikPlan->endEffectors.push_back(IK_EndEffector());
		ikSolver.ikPlan->endEffectors.back().endEffectorLocalCoords = cffp.stepPatterns[eeIndex].eeLocalCoords;
		ikSolver.ikPlan->endEffectors.back().endEffectorRB = cffp.stepPatterns[eeIndex].eeRB;
		ikSolver.ikPlan->endEffectors.back().targetEEPos = eeTrajectories[eeIndex].evaluate_linear(t);
	}

	ikSolver.ikEnergyFunction->setupSubObjectives();
	ikSolver.ikEnergyFunction->regularizer = 1;
	ikSolver.solve(20);

	newState = RobotState(robot);
	robot->setState(&oldState);
	return newState;
}

void FastMOPTPreplanner::draw() {

	glColor4d(1,0,0,0.2);

	RobotState tmpState(moptWindow->robot);
	moptWindow->robot->setState(&startState);
	moptWindow->robot->draw(SHOW_ABSTRACT_VIEW);
	moptWindow->robot->setState(&tmpState);

//	drawArrow(P3D(0, 1, 0), V3D(-1, 0, 0), 0.01);

	double dt = 0.1;
	glBegin(GL_LINE_STRIP);
	for (double t = moptWindow->motionPlanStartTime; t <= moptWindow->motionPlanStartTime + moptWindow->preplanTimeHorizon; t += dt) {
		P3D pos = bodyTrajectory.evaluate_linear(t);
		glVertex3d(pos.x(), pos.y(), pos.z());
	}
	glEnd();

	glColor4d(0, 0, 1, 0.2);
	dt = 0.001;
	for (uint i = 0; i < eeTrajectories.size(); i++) {
		glBegin(GL_LINE_STRIP);
		for (double t = moptWindow->motionPlanStartTime; t <= moptWindow->motionPlanStartTime + moptWindow->preplanTimeHorizon; t += dt) {
			P3D pos = eeTrajectories[i].evaluate_linear(t);
			glVertex3d(pos.x(), pos.y(), pos.z());
		}
		glEnd();
	}

	glColor4d(1, 0, 0, 0.2);
	dt = moptWindow->preplanTimeHorizon / 20.0;
	for (double t = moptWindow->motionPlanStartTime; t <= moptWindow->motionPlanStartTime + moptWindow->preplanTimeHorizon; t += dt) {
		P3D pos = bodyTrajectory.evaluate_linear(t);
		drawSphere(pos, 0.01);
		V3D forward = moptWindow->robot->forward.rotate(headingTrajectory.evaluate_linear(t), Globals::worldUp);
		drawArrow(pos, forward*0.05, 0.0025);
	}

}

void FastMOPTPreplanner::prepareMOPTPlan(LocomotionEngineMotionPlan* motionPlan) {
	//the motion plan will be synced with the start of the motion pre-plan
	motionPlan->motionPlanDuration = moptWindow->moptParams.motionPlanDuration;
	motionPlan->wrapAroundBoundaryIndex = -1;

	double h = motionPlan->motionPlanDuration / (motionPlan->nSamplePoints - 1);

	for (int i = 0; i < motionPlan->nSamplePoints; i++) {
		double t = moptWindow->motionPlanStartTime + h * i;
		//take care of contact flags and end effector trajectories
		for (uint j = 0; j < motionPlan->endEffectorTrajectories.size(); j++) {
			auto eeTraj = &motionPlan->endEffectorTrajectories[j];
			eeTraj->contactForce[i] = V3D();
			eeTraj->EEPos[i] = eeTrajectories[j].evaluate_linear(t);

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

		P3D comPos = bodyTrajectory.evaluate_linear(t);
		for (int k=0;k<3;k++)
			motionPlan->bodyTrajectory.pos[k][i] = motionPlan->bodyTrajectory.desiredPos[k][i] = comPos[k];

//		TODO: should this be the real orientation, coming from the robot, with some first order dynamics?!?
		motionPlan->bodyTrajectory.orientation[0][i] = motionPlan->bodyTrajectory.desiredOrientation[0][i] = headingTrajectory.evaluate_linear(t);
		motionPlan->bodyTrajectory.orientation[1][i] = motionPlan->bodyTrajectory.orientation[2][i] = 0;
		motionPlan->bodyTrajectory.desiredOrientation[1][i] = motionPlan->bodyTrajectory.desiredOrientation[2][i] = 0;
	}

	motionPlan->bodyTrajectory.useInitialVelocities = true;
	motionPlan->bodyTrajectory.initialLinearVelocity = initialLinearVelocity;
	motionPlan->bodyTrajectory.initialAngularVelocity = initialAngularVelocity;

	motionPlan->syncFootFallPatternWithMotionPlan(moptWindow->footFallPattern);
}



