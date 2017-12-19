#include <RobotDesignerLib/LocomotionEngineManagerGRF.h>

#include <RobotDesignerLib/MPO_VelocitySoftConstraints.h>
#include <RobotDesignerLib/MPO_WheelSpeedConstraint.h>
#include <RobotDesignerLib/MPO_WheelSpeedRegularizer.h>
#include <RobotDesignerLib/MPO_WheelAngleRegularizer.h>
#include <RobotDesignerLib/MPO_WheelAccelerationConstraint.h>
#include <RobotDesignerLib/MPO_PeriodicWheelTrajectoriesObjective.h>
#include <RobotDesignerLib/MPO_EEPosSwingObjective.h>
#include <RobotDesignerLib/MPO_RobotWheelAxisObjective.h>
#include <RobotDesignerLib/MPO_COMZeroVelocityConstraint.h>
#include <RobotDesignerLib/MPO_DefaultRobotStateConstraint.h>
#include <RobotDesignerLib/MPO_VelocityL0Regularization.h>
#include <RobotDesignerLib/MPO_StateMatchObjective.h>


//#define DEBUG_WARMSTART
//#define CHECK_DERIVATIVES_AFTER_WARMSTART

//- stance leg regularizer should ensure average pose is the same as default pose
//- add objectives that mimic legs in their absence, for when we're optimizing just the motion plan, without any robot...
//- different parts of the parameter state should probably have different regularizers...

LocomotionEngineManagerGRF::LocomotionEngineManagerGRF() {
}

LocomotionEngineManagerGRF::LocomotionEngineManagerGRF(Robot* robot, FootFallPattern* ffp, int nSamplePoints, bool periodicMotion) {
	this->periodicMotion = periodicMotion;

	this->footFallPattern = ffp;

	motionPlan = new LocomotionEngineMotionPlan(robot, nSamplePoints);

	motionPlan->syncMotionPlanWithFootFallPattern(*footFallPattern);

	if (periodicMotion) {
		//for boundary conditions, make sure initial and final conditions match
		motionPlan->setPeriodicBoundaryConditionsToTimeSample(0);
	}

	motionPlan->frictionCoeff = -1;

	createSolverComponents();
}

LocomotionEngineManagerGRF::~LocomotionEngineManagerGRF(){
	delete motionPlan;
}

void LocomotionEngineManagerGRF::warmStartMOpt() {

	FootFallPattern originalFootFallPattern = *footFallPattern;
	double desSwingHeight = motionPlan->swingFootHeight;
	double desSpeedX = motionPlan->desDistanceToTravel.x();
	double desSpeedZ = motionPlan->desDistanceToTravel.z();
	double desTurningAngle = motionPlan->desTurningAngle;

	ObjectiveFunction* robotEEObj = NULL;
	ObjectiveFunction* robotCOMObj = NULL;
	ObjectiveFunction* smoothCOMMotionObj = NULL;
	for (auto obj : energyFunction->objectives)
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

	energyFunction->objectives.push_back(new MPO_COMTrajectoryObjective(motionPlan, "intermediate periodic COM trajectory plan", 10000.0, motionPlan->nSamplePoints - 1, 0));
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

	energyFunction->regularizer = 0.001;
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

	energyFunction->objectives.pop_back();
	robotEEObj->weight = robotEEWeight;
	robotCOMObj->weight = robotCOMWeight;
	smoothCOMMotionObj->weight = smoothCOMMotionWeight;


	energyFunction->regularizer = 0.5;

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

		runMOPTStep(OPT_GRFS | OPT_COM_POSITIONS | OPT_END_EFFECTORS | OPT_WHEELS | OPT_COM_ORIENTATIONS | OPT_ROBOT_STATES);
#ifdef DEBUG_WARMSTART
		Logger::consolePrint("WARM START, alltogether optimizer step %d...\n", i);
		if (tmpWSIndex <= wsLimit++)
			return;
#endif
	}

	setDefaultOptimizationFlags();

	motionPlan->swingFootHeight = desSwingHeight;
	motionPlan->desDistanceToTravel.x() = desSpeedX;
	motionPlan->desDistanceToTravel.z() = desSpeedZ;
	motionPlan->desTurningAngle = desTurningAngle;
}

void LocomotionEngineManagerGRF::setDefaultOptimizationFlags() {
	motionPlan->optimizeEndEffectorPositions = true;
	motionPlan->optimizeWheels = true;
	motionPlan->optimizeCOMPositions = true;
	motionPlan->optimizeCOMOrientations = true;
	motionPlan->optimizeRobotStates = true;
	motionPlan->optimizeContactForces = true;
	motionPlan->optimizeBarycentricWeights = false;
}



/*************************************************************************************************************/
/* GRFv1 employs an SQP solver operating on a mix of objectives and constraints to generate the motion plan*/
/*************************************************************************************************************/
LocomotionEngineManagerGRFv1::LocomotionEngineManagerGRFv1() : LocomotionEngineManagerGRF(){
}

LocomotionEngineManagerGRFv1::LocomotionEngineManagerGRFv1(Robot* robot, FootFallPattern* ffp, int nSamplePoints) : LocomotionEngineManagerGRF(robot, ffp, nSamplePoints) {
	setupObjectives();
}

void LocomotionEngineManagerGRFv1::setupObjectives() {
	LocomotionEngine_EnergyFunction* ef = energyFunction;
	for (uint i = 0; i < ef->objectives.size(); i++)
		delete ef->objectives[i];
	ef->objectives.clear();

	//soft constraints
	ef->objectives.push_back(new MPO_ForceAccelObjective(ef->theMotionPlan, "force acceleration objective", 10000.0));
	ef->objectives.push_back(new MPO_TorqueAngularAccelObjective(ef->theMotionPlan, "torque angular acceleration objective", 10000.0));
	ef->objectives.push_back(new MPO_RobotCOMObjective(ef->theMotionPlan, "robot COM objective", 10000.0));
	ef->objectives.push_back(new MPO_RobotEndEffectorsObjective(ef->theMotionPlan, "robot EE objective", 10000.0));
	ef->objectives.push_back(new MPO_RobotWheelAxisObjective(ef->theMotionPlan, "robot wheel axis objective", 10000.0));
	ef->objectives.push_back(new MPO_RobotCOMOrientationsObjective(ef->theMotionPlan, "robot COM orientations objective", 10000.0));
	ef->objectives.push_back(new MPO_VelocitySoftBoundConstraints(ef->theMotionPlan, "joint angle velocity constraint", 1e4, 6, ef->theMotionPlan->robotRepresentation->getDimensionCount() - 1));
	ef->objectives.push_back(new MPO_WheelSpeedConstraints(ef->theMotionPlan, "wheel speed bound constraint", 1e4));
	ef->objectives.push_back(new MPO_WheelAccelerationConstraints(ef->theMotionPlan, "wheel accel. bound constraint", 1e2));

	//functional objectives
	ef->objectives.push_back(new MPO_COMTravelObjective(ef->theMotionPlan, "COM Travel objective", 50.0));
	ef->objectives.push_back(new MPO_COMTurningObjective(ef->theMotionPlan, "robot turning rate (YAW)", 50.0));

	//motion regularizers
	ef->objectives.push_back(new MPO_SmoothCOMTrajectories(ef->theMotionPlan, "smoothCOM", 10));
	ef->objectives.push_back(new MPO_SmoothStanceLegMotionObjective(ef->theMotionPlan, "robot stance leg smooth joint angle trajectories", 0.01));
	ef->objectives.push_back(new MPO_StanceLegMotionRegularizer(ef->theMotionPlan, "robot stance legs motion regularizer", 0.01));
	ef->objectives.push_back(new MPO_FeetPathSmoothnessObjective(ef->theMotionPlan, "foot path smoothness objective", 1.0));

	ef->objectives.push_back(new MPO_RobotStateRegularizer(ef->theMotionPlan, "robot joint angles regularizer objective", 0.0010 * 10, 6, ef->theMotionPlan->robotRepresentation->getDimensionCount() - 1));
	ef->objectives.push_back(new MPO_RobotStateRegularizer(ef->theMotionPlan, "robot body orientation regularizer objective (ROLL)", 1, 5, 5));
	ef->objectives.push_back(new MPO_RobotStateRegularizer(ef->theMotionPlan, "robot body orientation regularizer objective (PITCH)", 1, 4, 4));
	ef->objectives.push_back(new MPO_RobotStateRegularizer(ef->theMotionPlan, "robot body orientation regularizer objective (YAW)", 1, 3, 3));
	ef->objectives.push_back(new MPO_SmoothRobotMotionTrajectories(ef->theMotionPlan, "robot smooth joint angle trajectories", 0.001, 6, ef->theMotionPlan->robotRepresentation->getDimensionCount() - 1));
	ef->objectives.push_back(new MPO_SmoothRobotMotionTrajectories(ef->theMotionPlan, "robot smooth body orientation trajectories", 1, 3, 5));
}

LocomotionEngineManagerGRFv1::~LocomotionEngineManagerGRFv1(){
}

/*************************************************************************************************************/
/* For GRFv2, constraints are transformed into objectives with high weights, so this one employs a Newton-style solver   */
/*************************************************************************************************************/
LocomotionEngineManagerGRFv2::LocomotionEngineManagerGRFv2() : LocomotionEngineManagerGRF() {
}

LocomotionEngineManagerGRFv2::LocomotionEngineManagerGRFv2(Robot* robot, FootFallPattern* ffp, int nSamplePoints, bool periodicMotion) : LocomotionEngineManagerGRF(robot, ffp, nSamplePoints, periodicMotion) {
	setupObjectives();
	useObjectivesOnly = true;
}

void LocomotionEngineManagerGRFv2::setupObjectives() {
	LocomotionEngine_EnergyFunction* ef = energyFunction;

	for (uint i = 0; i < ef->objectives.size(); i++)
		delete ef->objectives[i];
	ef->objectives.clear();

	//GRF constraints
	ef->objectives.push_back(new MPO_GRFRegularizer(ef->theMotionPlan, "GRF regularizers", 10000.0));
	ef->objectives.push_back(new MPO_GRFSoftBoundConstraints(ef->theMotionPlan, "GRF bound constraints", 10000.0));

	//consistancy constraints (between robot states and other auxiliary variables)
	ef->objectives.push_back(new MPO_RobotEndEffectorsObjective(ef->theMotionPlan, "robot EE objective", 10000.0));
	ef->objectives.push_back(new MPO_RobotWheelAxisObjective(ef->theMotionPlan, "robot wheel axis objective", 10000.0));
	ef->objectives.push_back(new MPO_RobotCOMObjective(ef->theMotionPlan, "robot COM objective", 10000.0));
	ef->objectives.push_back(new MPO_RobotCOMOrientationsObjective(ef->theMotionPlan, "robot COM orientations objective", 10000.0));

	//dynamics constraints
	ef->objectives.push_back(new MPO_ForceAccelObjective(ef->theMotionPlan, "force acceleration objective", 1.0));
	ef->objectives.push_back(new MPO_TorqueAngularAccelObjective(ef->theMotionPlan, "torque angular acceleration objective", 1.0));
	ef->objectives.push_back(new MPO_VelocitySoftBoundConstraints(ef->theMotionPlan, "joint angle velocity constraint", 1e4, 6, ef->theMotionPlan->robotRepresentation->getDimensionCount() - 1));
	ef->objectives.push_back(new MPO_WheelSpeedConstraints(ef->theMotionPlan, "wheel speed bound constraint", 1e4));
	ef->objectives.push_back(new MPO_WheelAccelerationConstraints(ef->theMotionPlan, "wheel accel. bound constraint", 1e2));

	//constraints ensuring feet don't slide...
	ef->objectives.push_back(new MPO_FeetSlidingObjective(ef->theMotionPlan, "feet sliding objective", 10000.0));
	ef->objectives.push_back(new MPO_WheelGroundObjective(ef->theMotionPlan, "wheel ground objective", 10000.0));

	ef->objectives.push_back(new MPO_COMZeroVelocityConstraint(ef->theMotionPlan, "start velocity zero objective", 0, 10000.0));
	ef->objectives.back()->isActive = false;
	ef->objectives.push_back(new MPO_COMZeroVelocityConstraint(ef->theMotionPlan, "end velocity zero objective", ef->theMotionPlan->nSamplePoints-2, 10000.0));
	ef->objectives.back()->isActive = false;

	ef->objectives.push_back(new MPO_DefaultRobotStateConstraint(ef->theMotionPlan, "start default rs objective", 0, 10000.0));
	ef->objectives.back()->isActive = false;
	ef->objectives.push_back(new MPO_DefaultRobotStateConstraint(ef->theMotionPlan, "end default rs objective", ef->theMotionPlan->nSamplePoints-2, 10000.0));
	ef->objectives.back()->isActive = false;

	// constraint ensuring the y component of the EE position follows the swing motion
	ef->objectives.push_back(new MPO_EEPosSwingObjective(ef->theMotionPlan, "EE pos swing objective", 10000.0));

	//periodic boundary constraints...
	if (ef->theMotionPlan->wrapAroundBoundaryIndex >= 0 && periodicMotion) {
		ef->objectives.push_back(new MPO_PeriodicRobotStateTrajectoriesObjective(ef->theMotionPlan, "periodic joint angles", 10000.0, ef->theMotionPlan->nSamplePoints - 1, ef->theMotionPlan->wrapAroundBoundaryIndex, 6, ef->theMotionPlan->robotRepresentation->getDimensionCount() - 1));
		ef->objectives.push_back(new MPO_PeriodicRobotStateTrajectoriesObjective(ef->theMotionPlan, "periodic body orientations (ROLL)", 10000.0, ef->theMotionPlan->nSamplePoints - 1, ef->theMotionPlan->wrapAroundBoundaryIndex, 4, 4));
		ef->objectives.push_back(new MPO_PeriodicRobotStateTrajectoriesObjective(ef->theMotionPlan, "periodic body orientations (PITCH)", 10000.0, ef->theMotionPlan->nSamplePoints - 1, ef->theMotionPlan->wrapAroundBoundaryIndex, 5, 5));
		ef->objectives.push_back(new MPO_PeriodicWheelTrajectoriesObjective(ef->theMotionPlan, "periodic wheel speed", 10000.0, ef->theMotionPlan->nSamplePoints - 1, ef->theMotionPlan->wrapAroundBoundaryIndex));
	}

	//if there are no periodic motion constraints, then we must provide some targets for the start and end of the motion...
	if (periodicMotion == false) {
		ef->objectives.push_back(new MPO_StateMatchObjective(ef->theMotionPlan, "state boundary constraint @ first", 10000, 0, ef->theMotionPlan->initialRobotState));
		ef->objectives.push_back(new MPO_StateMatchObjective(ef->theMotionPlan, "state boundary constraint @ second", 10000, 1, ef->theMotionPlan->initialRobotState));
		ef->objectives.push_back(new MPO_StateMatchObjective(ef->theMotionPlan, "state boundary constraint @ second last", 10000, ef->theMotionPlan->nSamplePoints - 2, ef->theMotionPlan->initialRobotState));
		ef->objectives.push_back(new MPO_StateMatchObjective(ef->theMotionPlan, "state boundary constraint @ last", 10000, ef->theMotionPlan->nSamplePoints - 1, ef->theMotionPlan->initialRobotState));
	}

	//functional objectives
	ef->objectives.push_back(new MPO_COMTravelObjective(ef->theMotionPlan, "COM Travel objective", 50.0));
	ef->objectives.push_back(new MPO_COMTurningObjective(ef->theMotionPlan, "COM turning objective (YAW)", 50.0));

	//motion regularizers
	ef->objectives.push_back(new MPO_SmoothStanceLegMotionObjective(ef->theMotionPlan, "robot stance leg smooth joint angle trajectories", 0.01));
	ef->objectives.push_back(new MPO_StanceLegMotionRegularizer(ef->theMotionPlan, "robot stance legs motion regularizer", 0.01));
	ef->objectives.push_back(new MPO_FeetPathSmoothnessObjective(ef->theMotionPlan, "foot path smoothness objective", 10.0));

	ef->objectives.push_back(new MPO_WheelSpeedRegularizer(ef->theMotionPlan, "wheel speed regularizer", 1e-4));
	ef->objectives.push_back(new MPO_WheelAngleRegularizer(ef->theMotionPlan, "wheel angle regularizer", 1e-4));

	ef->objectives.push_back(new MPO_RobotStateRegularizer(ef->theMotionPlan, "robot joint angles regularizer objective", 0.0010 * 1, 6, ef->theMotionPlan->robotRepresentation->getDimensionCount() - 1));
	ef->objectives.push_back(new MPO_NonLimbMotionRegularizer(ef->theMotionPlan, "robot joint angles regularizer objective (non-limb)", 0.01));

	ef->objectives.push_back(new MPO_RobotStateRegularizer(ef->theMotionPlan, "robot body orientation regularizer objective (ROLL)", 1, 5, 5));
	ef->objectives.push_back(new MPO_RobotStateRegularizer(ef->theMotionPlan, "robot body orientation regularizer objective (PITCH)", 1, 4, 4));
	ef->objectives.push_back(new MPO_RobotStateRegularizer(ef->theMotionPlan, "robot body orientation regularizer objective (YAW)", 1, 3, 3));
	ef->objectives.push_back(new MPO_SmoothRobotMotionTrajectories(ef->theMotionPlan, "robot smooth joint angle trajectories", 0.01, 6, ef->theMotionPlan->robotRepresentation->getDimensionCount() - 1));
	ef->objectives.push_back(new MPO_SmoothRobotMotionTrajectories(ef->theMotionPlan, "robot smooth body orientation trajectories", 1, 3, 5));
	ef->objectives.push_back(new MPO_NonLimbSmoothMotionObjective(ef->theMotionPlan, "robot smooth joint angles objective (non-limb)", 0.01));

	ef->objectives.push_back(new MPO_SmoothCOMTrajectories(ef->theMotionPlan, "smoothCOM", 50));

	ef->objectives.push_back(new MPO_VelocityL0Regularization(ef->theMotionPlan, "joint angle velocity L0 regularization", 1, 6, ef->theMotionPlan->robotRepresentation->getDimensionCount() - 1)); ef->objectives.back()->isActive = false;
}

LocomotionEngineManagerGRFv2::~LocomotionEngineManagerGRFv2(){
}
