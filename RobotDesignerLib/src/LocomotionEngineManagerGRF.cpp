#include <RobotDesignerLib/LocomotionEngineManagerGRF.h>

#include <RobotDesignerLib/MPO_VelocitySoftConstraints.h>
#include <RobotDesignerLib/MPO_JointsAnglesSoftConstraint.h>
#include <RobotDesignerLib/MPO_EndEffectorCollisionEnergy.h>
#include <RobotDesignerLib/MPO_WheelSpeedConstraint.h>
#include <RobotDesignerLib/MPO_WheelSpeedRegularizer.h>
#include <RobotDesignerLib/MPO_WheelSpeedSmoothRegularizer.h>
#include <RobotDesignerLib/MPO_WheelAngleSmoothRegularizer.h>
#include <RobotDesignerLib/MPO_WheelSpeedTargetObjective.h>
#include <RobotDesignerLib/MPO_WheelAccelerationConstraint.h>
#include <RobotDesignerLib/MPO_PeriodicWheelTrajectoriesObjective.h>
#include <RobotDesignerLib/MPO_EEPosSwingObjective.h>
#include <RobotDesignerLib/MPO_RobotWheelAxisObjective.h>
#include <RobotDesignerLib/MPO_COMZeroVelocityConstraint.h>
#include <RobotDesignerLib/MPO_DefaultRobotStateConstraint.h>
#include <RobotDesignerLib/MPO_VelocityL0Regularization.h>
#include <RobotDesignerLib/MPO_StateMatchObjective.h>
#include <RobotDesignerLib/MPO_GRFFrictionConstraints.h>
#include <RobotDesignerLib/MPO_PassiveWheelsGRFConstraints.h>
#include <RobotDesignerLib/MPO_FixedWheelObjective.h>
#include <RobotDesignerLib/MPO_WheelTiltObjective.h>
#include <RobotDesignerLib/MPO_EEPosObjective.h>
#include <RobotDesignerLib/MPO_BodyFrameObjective.h>

//#define DEBUG_WARMSTART
//#define CHECK_DERIVATIVES_AFTER_WARMSTART


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

	//add tmp objectives which will be released later on during the warmstart procedure...
	MPO_COMTrajectoryObjective* tmpCOMTrajectoryObjective = new MPO_COMTrajectoryObjective(motionPlan, "intermediate periodic COM trajectory plan", 10000.0, motionPlan->nSamplePoints - 1, 0);
	energyFunction->objectives.push_back(tmpCOMTrajectoryObjective);
	MPO_GRFVerticalUpperBoundConstraints* tmpGRFVerticalForceConstraint = new MPO_GRFVerticalUpperBoundConstraints(motionPlan, "tmpGRFVerticalForceConstraint", 10000.0);
	MPO_GRFTangentialBoundConstraints* tmpGRFTangentForceConstraint = new MPO_GRFTangentialBoundConstraints(motionPlan, "tmpGRFTangentForceConstraint", 10000.0);
	energyFunction->objectives.push_back(tmpGRFVerticalForceConstraint);
	energyFunction->objectives.push_back(tmpGRFTangentForceConstraint);

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
			tmpGRFVerticalForceConstraint->verticalGRFUpperBoundValues[iEE][iT] = 1000.0;
			tmpGRFTangentForceConstraint->tangentGRFBoundValues[iEE][iT] = 1000.0;
		}

	energyFunction->regularizer = 0.1;
	for (int i = 0; i < 10; i++) {
		runMOPTStep(OPT_GRFS);
#ifdef DEBUG_WARMSTART
		Logger::consolePrint("WARM START prestep %d: equal force distribution...\n", i);
		if (tmpWSIndex <= wsLimit++) {
			*footFallPattern = originalFootFallPattern;
			energyFunction->objectives.pop_back();
			energyFunction->objectives.pop_back();
			delete tmpGRFVerticalForceConstraint;
			delete tmpGRFTangentForceConstraint;
			energyFunction->objectives.pop_back();
			delete tmpCOMTrajectoryObjective;
			return;
		}
#endif
	}

	energyFunction->regularizer = 1;

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
		energyFunction->objectives.pop_back();
		energyFunction->objectives.pop_back();
		delete tmpGRFVerticalForceConstraint;
		delete tmpGRFTangentForceConstraint;
		energyFunction->objectives.pop_back();
		delete tmpCOMTrajectoryObjective;
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
					tmpGRFVerticalForceConstraint->verticalGRFUpperBoundValues[iEE][iT] = fLimit * factor + -motionPlan->verticalGRFLowerBoundVal * (1 - factor);
					tmpGRFTangentForceConstraint->tangentGRFBoundValues[iEE][iT] = fLimit * factor;
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
				energyFunction->objectives.pop_back();
				energyFunction->objectives.pop_back();
				delete tmpGRFVerticalForceConstraint;
				delete tmpGRFTangentForceConstraint;
				energyFunction->objectives.pop_back();
				delete tmpCOMTrajectoryObjective;
				return;
			}
#endif
	}
	*footFallPattern = originalFootFallPattern;

	energyFunction->objectives.pop_back();
	energyFunction->objectives.pop_back();
	delete tmpGRFVerticalForceConstraint;
	delete tmpGRFTangentForceConstraint;

	energyFunction->regularizer = 0.001;
	double lastVal = 0;
	for (int i = 0; i < 200; i++) {
		double val = runMOPTStep(OPT_GRFS | OPT_COM_POSITIONS);
		if (fabs(lastVal - val) < 1e-5 && i > 0)
			break;
		lastVal = val;
#ifdef DEBUG_WARMSTART
		Logger::consolePrint("WARM START, no more GRFs for swing legs, proper footfall pattern set now...\n");
		if (tmpWSIndex <= wsLimit++) {
			energyFunction->objectives.pop_back();
			delete tmpCOMTrajectoryObjective;
			return;
		}
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
	delete tmpCOMTrajectoryObjective;
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
	int nCount = 10;
	for (int i = 0; i < nCount; i++) {
		motionPlan->desDistanceToTravel.x() = desSpeedX * ((double)i / (nCount - 1));
		motionPlan->desDistanceToTravel.z() = desSpeedZ * ((double)i / (nCount - 1));
		motionPlan->desTurningAngle = desTurningAngle * ((double)i / (nCount - 1));

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
	int nSamples = ef->theMotionPlan->nSamplePoints;
	int dimCount = ef->theMotionPlan->robotRepresentation->getDimensionCount();

	for (uint i = 0; i < ef->objectives.size(); i++)
		delete ef->objectives[i];
	ef->objectives.clear();


	//consistancy constraints (between robot states and other auxiliary variables)
	ef->addObjectiveFunction(new MPO_RobotEndEffectorsObjective(ef->theMotionPlan, "robot EE objective", 10000.0), "Consistency Constraints (Kinematics)"); ef->objectives.back()->hackHessian = true;
	ef->addObjectiveFunction(new MPO_RobotWheelAxisObjective(ef->theMotionPlan, "robot wheel axis objective", 10000.0), "Consistency Constraints (Kinematics)"); ef->objectives.back()->hackHessian = true;
	ef->addObjectiveFunction(new MPO_RobotCOMObjective(ef->theMotionPlan, "robot COM objective", 10000.0), "Consistency Constraints (Kinematics)");
	ef->addObjectiveFunction(new MPO_RobotCOMOrientationsObjective(ef->theMotionPlan, "robot COM orientations objective", 10000.0), "Consistency Constraints (Kinematics)");
	ef->addObjectiveFunction(new MPO_FeetSlidingObjective(ef->theMotionPlan, "feet sliding objective", 10000.0), "Consistency Constraints (Kinematics)");
	ef->addObjectiveFunction(new MPO_EndEffectorGroundObjective(ef->theMotionPlan, "EE height objective (stance)", 10000.0), "Consistency Constraints (Kinematics)");
	ef->addObjectiveFunction(new MPO_EEPosSwingObjective(ef->theMotionPlan, "EE height objective (swing)", 10000.0), "Consistency Constraints (Kinematics)");
	ef->addObjectiveFunction(new MPO_FixedWheelObjective(ef->theMotionPlan, "Fixed wheels objective", 1.0), "Consistency Constraints (Kinematics)");

	//consistancy constraints (dynamics, F=ma, GRF feasibility, etc)
	ef->addObjectiveFunction(new MPO_GRFSwingRegularizer(ef->theMotionPlan, "GRF 0 in swing constraint", 10000.0), "Consistency Constraints (Dynamics)");
	ef->addObjectiveFunction(new MPO_GRFVerticalLowerBoundConstraints(ef->theMotionPlan, "GRF is positive constraint", 10000.0), "Consistency Constraints (Dynamics)");
	ef->addObjectiveFunction(new MPO_ForceAccelObjective(ef->theMotionPlan, "force acceleration objective", 100.0), "Consistency Constraints (Dynamics)");
	ef->addObjectiveFunction(new MPO_TorqueAngularAccelObjective(ef->theMotionPlan, "torque angular acceleration objective", 100.0), "Consistency Constraints (Dynamics)");
	ef->addObjectiveFunction(new MPO_GRFFrictionConstraints(ef->theMotionPlan, "GRF friction constraints", 100.0), "Consistency Constraints (Dynamics)");
	ef->addObjectiveFunction(new MPO_PassiveWheelsGRFConstraints(ef->theMotionPlan, "Passive wheels (w/o friction)", 100.0), "Consistency Constraints (Dynamics)");
//	ef->addObjectiveFunction(new MPO_PassiveWheelsGRFFrictionConstraints(ef->theMotionPlan, "Passive wheels constraints (w/ friction)", 1.0), "Consistency Constraints (Dynamics)");


	//range of motion/speed/acceleration constraints
	ef->addObjectiveFunction(new MPO_VelocitySoftBoundConstraints(ef->theMotionPlan, "joint angle velocity constraint", 1, 6, dimCount - 1), "Bound Constraints");
	ef->addObjectiveFunction(new MPO_JointsAnglesSoftConstraint(ef->theMotionPlan, "joint angle bound constraint", 1, 6, dimCount - 1), "Bound Constraints"); ef->objectives.back()->isActive=false;
	ef->addObjectiveFunction(new MPO_WheelSpeedConstraints(ef->theMotionPlan, "wheel speed bound constraint", 1), "Bound Constraints");
	ef->addObjectiveFunction(new MPO_EndEffectorCollisionEnergy(ef->theMotionPlan, "EE collision objective", 10000), "Bound Constraints");
//	ef->addObjectiveFunction(new MPO_WheelAccelerationConstraints(ef->theMotionPlan, "wheel accel. bound constraint", 1e2), "Bound Constraints");
//	ef->objectives.back()->isActive = false;


//	ef->addObjectiveFunction(new MPO_COMZeroVelocityConstraint(ef->theMotionPlan, "start velocity zero objective", 0, 10000.0), "Boundary Constraints");
//	ef->objectives.back()->isActive = false;
//	ef->addObjectiveFunction(new MPO_COMZeroVelocityConstraint(ef->theMotionPlan, "end velocity zero objective", nSamples-2, 10000.0), "Boundary Constraints");
//	ef->objectives.back()->isActive = false;

//	ef->addObjectiveFunction(new MPO_DefaultRobotStateConstraint(ef->theMotionPlan, "start default rs objective", 0, 10000.0), "Boundary Constraints");
//	ef->objectives.back()->isActive = false;
//	ef->addObjectiveFunction(new MPO_DefaultRobotStateConstraint(ef->theMotionPlan, "end default rs objective", nSamples-2, 10000.0), "Boundary Constraints");
//	ef->objectives.back()->isActive = false;

//	ef->addObjectiveFunction(new MPO_WheelSpeedTargetObjective(ef->theMotionPlan, "wheel speed zero @t=0", 0, 0, 10000.0), "Boundary Constraints");
//	ef->objectives.back()->isActive = false;
//	ef->addObjectiveFunction(new MPO_WheelSpeedTargetObjective(ef->theMotionPlan, "wheel speed zero @t=end", nSamples-1, 0, 10000.0), "Boundary Constraints");
//	ef->objectives.back()->isActive = false;


	//periodic boundary constraints...
	if (ef->theMotionPlan->wrapAroundBoundaryIndex >= 0 && periodicMotion) {
		ef->addObjectiveFunction(new MPO_PeriodicRobotStateTrajectoriesObjective(ef->theMotionPlan, "periodic joint angles", 10000.0, nSamples - 1, ef->theMotionPlan->wrapAroundBoundaryIndex, 6, dimCount - 1), "Periodic Constraints");
		ef->addObjectiveFunction(new MPO_PeriodicRobotStateTrajectoriesObjective(ef->theMotionPlan, "periodic body orientations (ROLL)", 10000.0, nSamples - 1, ef->theMotionPlan->wrapAroundBoundaryIndex, 4, 4), "Periodic Constraints");
		ef->addObjectiveFunction(new MPO_PeriodicRobotStateTrajectoriesObjective(ef->theMotionPlan, "periodic body orientations (PITCH)", 10000.0, nSamples - 1, ef->theMotionPlan->wrapAroundBoundaryIndex, 5, 5), "Periodic Constraints");
		ef->addObjectiveFunction(new MPO_PeriodicWheelTrajectoriesObjective(ef->theMotionPlan, "periodic wheel speed", 10000.0, nSamples - 1, ef->theMotionPlan->wrapAroundBoundaryIndex), "Periodic Constraints");
	}

	//if there are no periodic motion constraints, then we must provide some targets for the start and end of the motion...
	if (periodicMotion == false) {
		ef->addObjectiveFunction(new MPO_StateMatchObjective(ef->theMotionPlan, "state boundary constraint @ first", 10000, 0, ef->theMotionPlan->initialRobotState), "Boundary Constraints");
		ef->addObjectiveFunction(new MPO_StateMatchObjective(ef->theMotionPlan, "state boundary constraint @ second", 10000, 1, ef->theMotionPlan->initialRobotState), "Boundary Constraints");
		ef->addObjectiveFunction(new MPO_StateMatchObjective(ef->theMotionPlan, "state boundary constraint @ second last", 10000, nSamples - 2, ef->theMotionPlan->initialRobotState), "Boundary Constraints");
		ef->addObjectiveFunction(new MPO_StateMatchObjective(ef->theMotionPlan, "state boundary constraint @ last", 10000, nSamples - 1, ef->theMotionPlan->initialRobotState), "Boundary Constraints");

		ef->addObjectiveFunction(new MPO_WheelSpeedTargetObjective(ef->theMotionPlan, "wheel speed zero @t=0", 0, 0, 10000.0), "Boundary Constraints");
		ef->addObjectiveFunction(new MPO_WheelSpeedTargetObjective(ef->theMotionPlan, "wheel speed zero @t=1", 1, 0, 10000.0), "Boundary Constraints");
		ef->addObjectiveFunction(new MPO_WheelSpeedTargetObjective(ef->theMotionPlan, "wheel speed zero @t=end-2", nSamples - 2, 0, 10000.0), "Boundary Constraints");
		ef->addObjectiveFunction(new MPO_WheelSpeedTargetObjective(ef->theMotionPlan, "wheel speed zero @t=end-1", nSamples - 1, 0, 10000.0), "Boundary Constraints");
	}

	//functional objectives
	ef->addObjectiveFunction(new MPO_COMTravelObjective(ef->theMotionPlan, "COM Travel objective", 50.0), "Objectives");
	ef->addObjectiveFunction(new MPO_COMTurningObjective(ef->theMotionPlan, "COM turning objective (YAW)", 50.0), "Objectives");
	ef->addObjectiveFunction(new MPO_BodyFrameObjective(ef->theMotionPlan, "body choreography objective", 10000), "Objectives");
	ef->addObjectiveFunction(new MPO_EEPosObjective(ef->theMotionPlan, "EE choreography objective", 10000), "Objectives");

	//smooth motion regularizers
	ef->addObjectiveFunction(new MPO_SmoothStanceLegMotionObjective(ef->theMotionPlan, "robot stance leg smooth joint angle trajectories", 0.01), "Smooth Regularizer");
	ef->addObjectiveFunction(new MPO_StanceLegMotionRegularizer(ef->theMotionPlan, "robot stance legs motion regularizer", 0.01), "Smooth Regularizer");
	ef->addObjectiveFunction(new MPO_FeetPathSmoothnessObjective(ef->theMotionPlan, "foot path smoothness objective", 10.0), "Smooth Regularizer");


//	ef->addObjectiveFunction(new MPO_WheelSpeedRegularizer(ef->theMotionPlan, "wheel speed regularizer", 1e-4), "Regularizers");
//	ef->objectives.back()->isActive = false;
	ef->addObjectiveFunction(new MPO_WheelSpeedSmoothRegularizer(ef->theMotionPlan, "wheel speed smooth regularizer", 1e-4), "Smooth Regularizers");
	ef->addObjectiveFunction(new MPO_WheelAngleSmoothRegularizer(ef->theMotionPlan, "wheel angle smooth regularizer", 1e-4), "Smooth Regularizers");
	ef->addObjectiveFunction(new MPO_WheelTiltObjective(ef->theMotionPlan, "Minimize wheel tilt", 1), "Smooth Regularizers"); ef->objectives.back()->isActive = false;

	ef->addObjectiveFunction(new MPO_NonLimbMotionRegularizer(ef->theMotionPlan, "robot joint angles regularizer objective (non-limb)", 0.01), "Smooth Regularizer");
	ef->addObjectiveFunction(new MPO_SmoothRobotMotionTrajectories(ef->theMotionPlan, "robot smooth joint angle trajectories", 0.01, 6, dimCount - 1), "Smooth Regularizer");
	ef->addObjectiveFunction(new MPO_SmoothRobotMotionTrajectories(ef->theMotionPlan, "robot smooth body orientation trajectories", 1, 3, 5), "Smooth Regularizer");
	ef->addObjectiveFunction(new MPO_NonLimbSmoothMotionObjective(ef->theMotionPlan, "robot smooth joint angles objective (non-limb)", 0.01), "Smooth Regularizer");
	ef->addObjectiveFunction(new MPO_SmoothCOMTrajectories(ef->theMotionPlan, "smoothCOM", 50), "Smooth Regularizer");

	ef->addObjectiveFunction(new MPO_GRFStanceRegularizer(ef->theMotionPlan, "GRF stance regularizer", 1e-5), "Regularizers");
	ef->addObjectiveFunction(new MPO_RobotStateRegularizer(ef->theMotionPlan, "robot body orientation regularizer objective (ROLL)", 1, 5, 5), "Regularizers");
	ef->addObjectiveFunction(new MPO_RobotStateRegularizer(ef->theMotionPlan, "robot body orientation regularizer objective (PITCH)", 1, 4, 4), "Regularizers");
	ef->addObjectiveFunction(new MPO_RobotStateRegularizer(ef->theMotionPlan, "robot body orientation regularizer objective (YAW)", 1, 3, 3), "Regularizers");
	ef->addObjectiveFunction(new MPO_RobotStateRegularizer(ef->theMotionPlan, "robot joint angles regularizer objective", 0.0010 * 1, 6, dimCount - 1), "Regularizers");


	ef->addObjectiveFunction(new MPO_VelocityL0Regularization(ef->theMotionPlan, "joint angle velocity L0 regularization (Local)", 1, 6, dimCount - 1,true), "L0 Regularizers"); ef->objectives.back()->isActive = false;
	ef->addObjectiveFunction(new MPO_VelocityL0Regularization(ef->theMotionPlan, "joint angle velocity L0 regularization (Global)", 1, 6, dimCount - 1,false), "L0 Regularizers"); ef->objectives.back()->isActive = false;
}

LocomotionEngineManagerGRFv2::~LocomotionEngineManagerGRFv2(){
}




/*************************************************************************************************************************/
/* For GRFv3, constraints are transformed into objectives with high weights, so this one employs a Newton-style solver   */
/* Furthermore, robot states are not explicitly optimized, at least not at the same time as everything else              */
/*************************************************************************************************************************/
LocomotionEngineManagerGRFv3::LocomotionEngineManagerGRFv3() : LocomotionEngineManagerGRF() {
}

LocomotionEngineManagerGRFv3::LocomotionEngineManagerGRFv3(Robot* robot, FootFallPattern* ffp, int nSamplePoints) : LocomotionEngineManagerGRF(robot, ffp, nSamplePoints, false) {
	setupObjectives();
	useObjectivesOnly = true;
}

void LocomotionEngineManagerGRFv3::setupObjectives() {
	LocomotionEngine_EnergyFunction* ef = energyFunction;
	int nSamples = ef->theMotionPlan->nSamplePoints;
	int dimCount = ef->theMotionPlan->robotRepresentation->getDimensionCount();

	for (uint i = 0; i < ef->objectives.size(); i++)
		delete ef->objectives[i];
	ef->objectives.clear();

	//we will assume here that only GRFs and COM trajectories will be optimized here...

	//consistancy constraints (between robot states and other auxiliary variables)
	ef->addObjectiveFunction(new MPO_FeetSlidingObjective(ef->theMotionPlan, "feet sliding objective", 10000.0), "Consistency Constraints (Kinematics)");

	//consistancy constraints (dynamics, F=ma, GRF feasibility, etc)
	ef->addObjectiveFunction(new MPO_GRFSwingRegularizer(ef->theMotionPlan, "GRF 0 in swing constraint", 1.0), "Consistency Constraints (Dynamics)");
	ef->addObjectiveFunction(new MPO_GRFVerticalLowerBoundConstraints(ef->theMotionPlan, "GRF is positive constraint", 1.0), "Consistency Constraints (Dynamics)");
	ef->addObjectiveFunction(new MPO_ForceAccelObjective(ef->theMotionPlan, "force acceleration objective", 100.0), "Consistency Constraints (Dynamics)");
	ef->addObjectiveFunction(new MPO_TorqueAngularAccelObjective(ef->theMotionPlan, "torque angular acceleration objective", 1.0), "Consistency Constraints (Dynamics)");
	ef->addObjectiveFunction(new MPO_GRFFrictionConstraints(ef->theMotionPlan, "GRF friction constraints", 1.0), "Consistency Constraints (Dynamics)");

	ef->addObjectiveFunction(new MPO_DefaultBodyTrajectoryObjective(ef->theMotionPlan, "Default body trajectories objective", 1000), "Objectives");
	ef->addObjectiveFunction(new MPO_DefaultEEPosObjective(ef->theMotionPlan, "Default end effector trajectories objective", 10), "Objectives");

	//smooth motion regularizers
	ef->addObjectiveFunction(new MPO_FeetPathSmoothnessObjective(ef->theMotionPlan, "foot path smoothness objective", 10.0), "Smooth Regularizer");
	ef->addObjectiveFunction(new MPO_SmoothCOMTrajectories(ef->theMotionPlan, "smoothCOM", 1000), "Smooth Regularizer");

	ef->addObjectiveFunction(new MPO_GRFStanceRegularizer(ef->theMotionPlan, "GRF stance regularizer", 1e-8), "Regularizers");
}

LocomotionEngineManagerGRFv3::~LocomotionEngineManagerGRFv3() {
}

void LocomotionEngineManagerGRFv3::warmStartMOpt() {

}

void LocomotionEngineManagerGRFv3::setDefaultOptimizationFlags() {
	motionPlan->optimizeEndEffectorPositions = false;
	motionPlan->optimizeWheels = false;
	motionPlan->optimizeCOMPositions = true;
	motionPlan->optimizeCOMOrientations = true;
	motionPlan->optimizeRobotStates = false;
	motionPlan->optimizeContactForces = true;
	motionPlan->optimizeBarycentricWeights = false;
}
