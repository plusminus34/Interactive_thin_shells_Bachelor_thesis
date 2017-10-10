#include <RobotDesignerLib/LocomotionEngineManager.h>
#include <RobotDesignerLib/MPO_PeriodicRobotStateTrajectoriesObjective.h>


LocomotionEngineManager::LocomotionEngineManager(){
}

void LocomotionEngineManager::drawMotionPlan(double f, int animationCycle, bool drawRobot, bool drawSkeleton, bool drawPlanDetails, bool drawContactForces, bool drawOrientation){
	// motionPlan->syncMotionPlanWithFootFallPattern(*footFallPattern);
	motionPlan->drawMotionPlan(f, animationCycle, drawRobot, drawSkeleton, drawPlanDetails, drawContactForces, drawOrientation);
}

LocomotionEngineManager::~LocomotionEngineManager()
{
}

void LocomotionEngineManager::warmStartMOptGRF()
{
	FootFallPattern originalFootFallPattern = *footFallPattern;
	double desSwingHeight = motionPlan->swingFootHeight;
	double desSpeedX = motionPlan->desDistanceToTravel.x();
	double desSpeedZ = motionPlan->desDistanceToTravel.z();
	double desTurningAngle = motionPlan->desTurningAngle;



	ObjectiveFunction* robotEEObj= NULL;
	ObjectiveFunction* robotCOMObj = NULL;
	ObjectiveFunction* smoothCOMMotionObj = NULL;
	for (auto obj : locomotionEngine->energyFunction->objectives)
	{
		if (obj->description == "robot EE objective")
			robotEEObj = obj;
		else if (obj->description == "robot COM objective")
			robotCOMObj = obj;
		else if (obj->description == "smoothCOM")
			smoothCOMMotionObj = obj;
	}
	
	double robotEEWeight = robotEEObj->weight;
	double robotCOMWeight = robotCOMObj->weight;
	double smoothCOMMotionWeight = smoothCOMMotionObj->weight;
	robotEEObj->weight *= 0;
	robotCOMObj->weight *= 0;
	smoothCOMMotionObj->weight = 60000;


	//	maybe add a regularizer for COM motion directly (keep height fixed, smooth motions... the things the robot COM is now doing...)

#ifdef DEBUG_WARMSTART
	static int tmpWSIndex = 0;
	int wsLimit = 1;
	tmpWSIndex++;
#endif

	footFallPattern->stepPatterns.clear();
	motionPlan->swingFootHeight = 0.0;
	motionPlan->desDistanceToTravel.x() = 0;
	motionPlan->desDistanceToTravel.z() = 0;
	motionPlan->desTurningAngle = 0;


	for (int iT = 0; iT < motionPlan->nSamplePoints; iT++)
		for (uint iEE = 0; iEE < motionPlan->endEffectorTrajectories.size(); iEE++) {
			motionPlan->endEffectorTrajectories[iEE].verticalGRFUpperBoundValues[iT] = 1000.0;
			motionPlan->endEffectorTrajectories[iEE].tangentGRFBoundValues[iT] = 1000.0;
		}

	for (int i = 0; i < 2; i++) {
		runMOPTStep(OPT_GRFS);
#ifdef DEBUG_WARMSTART
		Logger::consolePrint("WARM START prestep %d: equal force distribution...\n", i);
		if (tmpWSIndex <= wsLimit++) {
			*footFallPattern = originalFootFallPattern;
			return;
		}
#endif
	}

	locomotionEngine->energyFunction->objectives.push_back(new MPO_COMTrajectoryObjective(motionPlan, "intermediate periodic COM trajectory plan", 10000.0, motionPlan->nSamplePoints - 1, 0));
	motionPlan->syncMotionPlanWithFootFallPattern(*footFallPattern);
	double fLimit = 0;
	for (int iT = 0; iT < motionPlan->nSamplePoints; iT++)
		for (uint iEE = 0; iEE < motionPlan->endEffectorTrajectories.size(); iEE++)
			fLimit = MAX(fLimit, motionPlan->endEffectorTrajectories[iEE].contactForce[iT][1]);
	fLimit += 1;

	runMOPTStep(OPT_GRFS | OPT_COM_POSITIONS);

#ifdef DEBUG_WARMSTART
	Logger::consolePrint("WARM START final prestep of equal force distribution...\n");
	if (tmpWSIndex <= wsLimit++) {
		*footFallPattern = originalFootFallPattern;
		return;
	}
#endif

	int nSteps = 101;
	for (int i = 0; i < nSteps; i++) {
		//the factor will go from 1 down to 0 as it is making progress in the warmstart process...
		double factor = 1 - (double)i / (nSteps - 1.0);
		for (int iT = 0; iT < motionPlan->nSamplePoints; iT++) {
			for (uint iEE = 0; iEE < motionPlan->endEffectorTrajectories.size(); iEE++) {
				if (!originalFootFallPattern.isInStance(motionPlan->endEffectorTrajectories[iEE].theLimb, iT)) {
					//if the limb is in swing mode, it should not be able to apply GRFs, but get there gradually...
					motionPlan->endEffectorTrajectories[iEE].verticalGRFUpperBoundValues[iT] = fLimit * factor + -motionPlan->verticalGRFLowerBoundVal * (1 - factor);
					motionPlan->endEffectorTrajectories[iEE].tangentGRFBoundValues[iT] = fLimit * factor;
				}
			}
		}

		//now that the limits have been set on the upper bounds of the GRFs of the swing feet, run a mopt...
		runMOPTStep(OPT_GRFS | OPT_COM_POSITIONS);
#ifdef DEBUG_WARMSTART
		Logger::consolePrint("WARM START after iteration %d GRF limit: %lf (%lf)...\n", i, fLimit * factor + -motionPlan->verticalGRFLowerBoundVal * (1 - factor), factor);
		if (i % 10 == 0)
			if (tmpWSIndex <= wsLimit++) {
				*footFallPattern = originalFootFallPattern;
				return;
			}
#endif
	}
	*footFallPattern = originalFootFallPattern;

	for (int iT = 0; iT < motionPlan->nSamplePoints; iT++)
		for (uint iEE = 0; iEE < motionPlan->endEffectorTrajectories.size(); iEE++) {
			motionPlan->endEffectorTrajectories[iEE].verticalGRFUpperBoundValues[iT] = 1000.0;
			motionPlan->endEffectorTrajectories[iEE].tangentGRFBoundValues[iT] = 1000.0;
		}

	locomotionEngine->energyFunction->regularizer = 0.001;
	double lastVal = 0;
	for (int i = 0; i < 200; i++) {
		double val = runMOPTStep(OPT_GRFS | OPT_COM_POSITIONS);
		if (fabs(lastVal - val) < 1e-5 && i > 0)
			break;
		lastVal = val;
#ifdef DEBUG_WARMSTART
		Logger::consolePrint("WARM START, no more GRFs for swing legs, proper footfall pattern set now...\n");
		if (tmpWSIndex <= wsLimit++)
			return;
#endif
	}

#ifdef CHECK_DERIVATIVES_AFTER_WARMSTART

	runMOPTStep(OPT_GRFS | OPT_COM_POSITIONS | OPT_END_EFFECTORS | OPT_COM_ORIENTATIONS | OPT_ROBOT_STATES);
	dVector params;
	motionPlan->writeMPParametersToList(params);

	checkDerivatives = true;

	for (uint i = 0; i < locomotionEngine->energyFunction->objectives.size(); i++) {
		Logger::print("checking objective %s\n", locomotionEngine->energyFunction->objectives[i]->description.c_str());
		Logger::logPrint("checking objective %s\n", locomotionEngine->energyFunction->objectives[i]->description.c_str());
		for (uint j = 0; j < locomotionEngine->energyFunction->objectives.size(); j++)
			if (i == j)
				locomotionEngine->energyFunction->objectives[j]->weight = 1.0;
			else
				locomotionEngine->energyFunction->objectives[j]->weight = 0.0;
		locomotionEngine->energyFunction->testGradientWithFD(params);
		locomotionEngine->energyFunction->testHessianWithFD(params);
		motionPlan->setMPParametersFromList(params);
	}

	checkDerivatives = false;
	exit(0);
#endif

	locomotionEngine->energyFunction->objectives.pop_back();
	robotEEObj->weight = robotEEWeight;
	robotCOMObj->weight = robotCOMWeight;
	smoothCOMMotionObj->weight = smoothCOMMotionWeight;


	locomotionEngine->energyFunction->regularizer = 0.5;

	//	return;

	//now that we have a reasonable motion plan as far as GRFs and body motion are concerned, make the robot's motion match...

	for (int i = 0; i < 10; i++) {
		motionPlan->swingFootHeight = desSwingHeight * ((double)i / 9.0);

		double val = runMOPTStep(OPT_ROBOT_STATES);
//		if (fabs(lastVal - val) < 1e-5 && i > 0)
//			break;
//		lastVal = val;
#ifdef DEBUG_WARMSTART
		Logger::consolePrint("WARM START, robot state optimizer step %d...\n", i);
		if (tmpWSIndex <= wsLimit++)
			return;
#endif
	}
	motionPlan->swingFootHeight = desSwingHeight;

	for (int i = 0; i < 10; i++) {
		double val = runMOPTStep(OPT_ROBOT_STATES | OPT_GRFS | OPT_COM_POSITIONS);
		if (fabs(lastVal - val) < 1e-5 && i > 0)
			break;
		lastVal = val;
#ifdef DEBUG_WARMSTART
		Logger::consolePrint("WARM START, robot state optimizer step %d...\n", i);
		if (tmpWSIndex <= wsLimit++)
			return;
#endif
	}


//	return;

	for (int i = 0; i < 5; i++) {
		motionPlan->desDistanceToTravel.x() = desSpeedX * ((double)i / 4.0);
		motionPlan->desDistanceToTravel.z() = desSpeedZ * ((double)i / 4.0);
		motionPlan->desTurningAngle = desTurningAngle * ((double)i / 4.0);

		runMOPTStep(OPT_GRFS | OPT_COM_POSITIONS | OPT_END_EFFECTORS | OPT_COM_ORIENTATIONS | OPT_ROBOT_STATES);
#ifdef DEBUG_WARMSTART
		Logger::consolePrint("WARM START, alltogether optimizer step %d...\n", i);
		if (tmpWSIndex <= wsLimit++)
			return;
#endif
	}

	motionPlan->optimizeEndEffectorPositions = motionPlan->optimizeCOMPositions = motionPlan->optimizeCOMOrientations = motionPlan->optimizeRobotStates = motionPlan->optimizeContactForces = true;
	motionPlan->swingFootHeight = desSwingHeight;
	motionPlan->desDistanceToTravel.x() = desSpeedX;
	motionPlan->desDistanceToTravel.z() = desSpeedZ;
	motionPlan->desTurningAngle = desTurningAngle;
	
}

double LocomotionEngineManager::runMOPTStep() {
	motionPlan->syncMotionPlanWithFootFallPattern(*footFallPattern);

	Timer timer;
	if (checkDerivatives) {
		dVector params;
		motionPlan->writeMPParametersToList(params);

		Logger::print("\n --------- ContactForceStartIndex: %d -----------\n", motionPlan->contactForcesParamsStartIndex);
		Logger::logPrint("\n --------- ContactForceStartIndex: %d -----------\n", motionPlan->contactForcesParamsStartIndex);

		locomotionEngine->energyFunction->testGradientWithFD(params);
		locomotionEngine->energyFunction->testHessianWithFD(params);
		locomotionEngine->energyFunction->testIndividualGradient(params);
		locomotionEngine->energyFunction->testIndividualHessian(params);
	}
	locomotionEngine->energyFunction->printDebugInfo = printDebugInfo;

	double energyVal = 0;
	if (useBFGS)
		energyVal = locomotionEngine->optimizePlan_BFGS();
	else
		energyVal = locomotionEngine->optimizePlan();

	motionPlan->writeParamsToFile("..//out//MPParams.p");

	if (printDebugInfo)
		Logger::consolePrint("total time ellapsed: %lfs\n", timer.timeEllapsed());

	return energyVal;
}
