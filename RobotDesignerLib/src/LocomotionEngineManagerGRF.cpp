#include <RobotDesignerLib/LocomotionEngineManagerGRF.h>
#include <RobotDesignerLib/MPO_PeriodicRobotStateTrajectoriesObjective.h>

//#define DEBUG_WARMSTART
//#define CHECK_DERIVATIVES_AFTER_WARMSTART

//- stance leg regularizer should ensure average pose is the same as default pose
//- add objectives that mimic legs in their absence, for when we're optimizing just the motion plan, without any robot...
//- different parts of the parameter state should probably have different regularizers...

LocomotionEngineManagerGRF::LocomotionEngineManagerGRF()
{
}

LocomotionEngineManagerGRF::LocomotionEngineManagerGRF(Robot* robot, FootFallPattern* footFallPattern, int nSamplePoints) {
	
	if (footFallPattern)
	{
		this->footFallPattern = footFallPattern;
		origFootFallPattern = *footFallPattern;
	}
	else {
		this->footFallPattern = &origFootFallPattern;
	}

	motionPlan = new LocomotionEngineMotionPlan(robot, nSamplePoints);

	motionPlan->syncMotionPlanWithFootFallPattern(*footFallPattern);

	//for boundary conditions, make sure initial and final conditions match
	motionPlan->setPeriodicBoundaryConditionsToTimeSample(0);

	motionPlan->frictionCoeff = -1;

	locomotionEngine = new LocomotionEngine(motionPlan);
	locomotionEngine->energyFunction->setupGRFSubObjectives();
}

void LocomotionEngineManagerGRF::warmStartMOpt(){
	warmStartMOptGRF();
}


LocomotionEngineManagerGRF::~LocomotionEngineManagerGRF()
{
	delete motionPlan;
	delete locomotionEngine;
}

/***************************************************************************************************/


LocomotionEngineManagerGRFv2::LocomotionEngineManagerGRFv2(){
}

LocomotionEngineManagerGRFv2::LocomotionEngineManagerGRFv2(Robot* robot, FootFallPattern* ffp, int nSamplePoints){
	
	if (ffp)
	{
		this->footFallPattern = ffp;
		origFootFallPattern = *ffp;
	}
	else {
		this->footFallPattern = &origFootFallPattern;
	}

	motionPlan = new LocomotionEngineMotionPlan(robot, nSamplePoints);
	motionPlan->syncMotionPlanWithFootFallPattern(*footFallPattern);

	//for boundary conditions, make sure initial and final conditions match
	motionPlan->setPeriodicBoundaryConditionsToTimeSample(0);

	motionPlan->frictionCoeff = -1;

	locomotionEngine = new LocomotionEngine(motionPlan);
	locomotionEngine->energyFunction->setupGRFv2SubObjectives();
	locomotionEngine->useObjectivesOnly = true;
}

void LocomotionEngineManagerGRFv2::warmStartMOpt() {
	warmStartMOptGRF();
}

LocomotionEngineManagerGRFv2::~LocomotionEngineManagerGRFv2(){
	delete motionPlan;
	delete locomotionEngine;
}

/***************************************************************************************************/


LocomotionEngineManagerGRFv3::LocomotionEngineManagerGRFv3() {
}

LocomotionEngineManagerGRFv3::LocomotionEngineManagerGRFv3(Robot* robot, FootFallPattern* footFallPattern, int nSamplePoints) {
	
	if (footFallPattern)
	{
		this->footFallPattern = footFallPattern;
		origFootFallPattern = *footFallPattern;
	}
	else {
		this->footFallPattern = &origFootFallPattern;
	}

	motionPlan = new LocomotionEngineMotionPlan(robot, nSamplePoints);
	motionPlan->syncMotionPlanWithFootFallPattern(*footFallPattern);

	//for boundary conditions, make sure initial and final conditions match
	motionPlan->setPeriodicBoundaryConditionsToTimeSample(0);

	motionPlan->frictionCoeff = -1;

	locomotionEngine = new LocomotionEngine(motionPlan);

	locomotionEngine->energyFunction->setupGRFv3SubObjectives();
	locomotionEngine->useObjectivesOnly = true;
}

void LocomotionEngineManagerGRFv3::warmStartMOpt() {

	FootFallPattern originalFootFallPattern = *footFallPattern;
	double desSwingHeight = motionPlan->swingFootHeight;

	//	int robotEEIndex = 2;
	//	int robotCOMIndex = 3;
	int smoothCOMMotionObjIndex = locomotionEngine->energyFunction->objectives.size() - 1;
	//	double robotEEWeight = locomotionEngine->energyFunction->objectives[robotEEIndex]->weight;
	//	double robotCOMWeight = locomotionEngine->energyFunction->objectives[robotCOMIndex]->weight;
	double smoothCOMMotionWeight = locomotionEngine->energyFunction->objectives[smoothCOMMotionObjIndex]->weight;
	//	locomotionEngine->energyFunction->objectives[robotEEIndex]->weight *= 0;
	//	locomotionEngine->energyFunction->objectives[robotCOMIndex]->weight *= 0;
	locomotionEngine->energyFunction->objectives[smoothCOMMotionObjIndex]->weight = 60000;



	//	maybe add a regularizer for COM motion directly (keep height fixed, smooth motions... the things the robot COM is now doing...)

#ifdef DEBUG_WARMSTART
	static int tmpWSIndex = 0;
	int wsLimit = 1;
	tmpWSIndex++;
#endif

	footFallPattern->stepPatterns.clear();
	motionPlan->swingFootHeight = 0.0;

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

	int nSteps = 101; //101
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
	for (int i = 0; i < 200; i++) { //i<200
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
	//	locomotionEngine->energyFunction->objectives[robotEEIndex]->weight = robotEEWeight;
	//	locomotionEngine->energyFunction->objectives[robotCOMIndex]->weight = robotCOMWeight;
	locomotionEngine->energyFunction->objectives[smoothCOMMotionObjIndex]->weight = smoothCOMMotionWeight;

	motionPlan->swingFootHeight = desSwingHeight / 2.0;

	locomotionEngine->energyFunction->regularizer = 0.5;

	for (int i = 0; i < 50; i++) {
		runMOPTStep(OPT_GRFS | OPT_COM_POSITIONS | OPT_END_EFFECTORS | OPT_COM_ORIENTATIONS);
	}

	return;
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


	//now that we have a reasonable motion plan as far as GRFs and body motion are concerned, make the robot's motion match...

	for (int i = 0; i < 10; i++) {
		double val = runMOPTStep(OPT_ROBOT_STATES);
		if (fabs(lastVal - val) < 1e-5 && i > 0)
			break;
		lastVal = val;
#ifdef DEBUG_WARMSTART
		Logger::consolePrint("WARM START, robot state optimizer step %d...\n", i);
		if (tmpWSIndex <= wsLimit++)
			return;
#endif
	}

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

	motionPlan->optimizeEndEffectorPositions = motionPlan->optimizeCOMPositions = motionPlan->optimizeCOMOrientations = motionPlan->optimizeRobotStates = motionPlan->optimizeContactForces = true;

	return;

	for (int i = 0; i < 1; i++) {
		runMOPTStep(OPT_GRFS | OPT_COM_POSITIONS | OPT_END_EFFECTORS | OPT_COM_ORIENTATIONS | OPT_ROBOT_STATES);
#ifdef DEBUG_WARMSTART
		Logger::consolePrint("WARM START, alltogether optimizer step %d...\n", i);
		if (tmpWSIndex <= wsLimit++)
			return;
#endif
	}


	//	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO
	//	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO
	//	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO
	//	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO
	//	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO
	//	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO	TODO

	//double desSwingHeight = motionPlan->swingFootHeight;

	//motionPlan->swingFootHeight = desSwingHeight / 2.0;

}


LocomotionEngineManagerGRFv3::~LocomotionEngineManagerGRFv3()
{
	delete motionPlan;
	delete locomotionEngine;
}
