#include <RobotDesignerLib/LocomotionEngineManagerIP.h>

LocomotionEngineManagerIP::LocomotionEngineManagerIP(){
}

LocomotionEngineManagerIP::LocomotionEngineManagerIP(Robot* robot, FootFallPattern* ffp, int nSamplePoints){
	this->footFallPattern = ffp;

	motionPlan = new LocomotionEngineMotionPlan(robot, nSamplePoints);

	motionPlan->minBaricentricWeight = 0.15;

	motionPlan->syncMotionPlanWithFootFallPattern(*footFallPattern);

	//for boundary conditions, make sure initial and final conditions match
	motionPlan->setPeriodicBoundaryConditionsToTimeSample(0);

	createSolverComponents();
}

LocomotionEngineManagerIP::~LocomotionEngineManagerIP(){
	delete motionPlan;
}

/*****************************************************************************************/
/* IPv1 is the original mopt described in robot designer.                                */
/******************************************************************************************/
LocomotionEngineManagerIPv1::LocomotionEngineManagerIPv1() {

}

LocomotionEngineManagerIPv1::LocomotionEngineManagerIPv1(Robot* robot, FootFallPattern* footFallPattern, int nSamplePoints) : LocomotionEngineManagerIP(robot, footFallPattern, nSamplePoints) {
	setupObjectives();
}

LocomotionEngineManagerIPv1::~LocomotionEngineManagerIPv1() {
}

void LocomotionEngineManagerIPv1::setupObjectives() {
	LocomotionEngine_EnergyFunction* ef = energyFunction;
	for (uint i = 0; i<ef->objectives.size(); i++)
		delete ef->objectives[i];
	ef->objectives.clear();

	//soft constraints
	ef->objectives.push_back(new MPO_DynamicStabilityObjective(ef->theMotionPlan, "dynamic stability objective", 10000.0));
	ef->objectives.push_back(new MPO_RobotCOMObjective(ef->theMotionPlan, "robot COM objective", 10000.0));
	ef->objectives.push_back(new MPO_RobotEndEffectorsObjective(ef->theMotionPlan, "robot EE objective", 10000.0));

	//functional objectives
	ef->objectives.push_back(new MPO_COMTravelObjective(ef->theMotionPlan, "COM Travel objective", 50.0));
	ef->objectives.push_back(new MPO_RobotTurningObjective(ef->theMotionPlan, "robot turning rate (YAW)", 50.0));

	//motion regularizers
	ef->objectives.push_back(new MPO_SmoothCOMTrajectories(ef->theMotionPlan, "smoothCOM", 10));
	ef->objectives.push_back(new MPO_FeetPathSmoothnessObjective(ef->theMotionPlan, "foot path smoothness objective", 1.0));
	ef->objectives.push_back(new MPO_BarycentricWeightsRegularizerObjective(ef->theMotionPlan, "barycentric weights regularizer objective", 0.001 * 0.001));

	ef->objectives.push_back(new MPO_SmoothStanceLegMotionObjective(ef->theMotionPlan, "robot stance leg smooth joint angle trajectories", 0.01));
	ef->objectives.push_back(new MPO_StanceLegMotionRegularizer(ef->theMotionPlan, "robot stance legs motion regularizer", 0.01));
	ef->objectives.push_back(new MPO_RobotStateRegularizer(ef->theMotionPlan, "robot joint angles regularizer objective", 0.0010 * 10, 6, ef->theMotionPlan->robotRepresentation->getDimensionCount() - 1));
	ef->objectives.push_back(new MPO_RobotStateRegularizer(ef->theMotionPlan, "robot body orientation regularizer objective (ROLL)", 1, 5, 5));
	ef->objectives.push_back(new MPO_RobotStateRegularizer(ef->theMotionPlan, "robot body orientation regularizer objective (PITCH)", 1, 4, 4));
	ef->objectives.push_back(new MPO_RobotStateRegularizer(ef->theMotionPlan, "robot body orientation regularizer objective (YAW)", 1, 3, 3));
	ef->objectives.push_back(new MPO_SmoothRobotMotionTrajectories(ef->theMotionPlan, "robot smooth joint angle trajectories", 0.001, 6, ef->theMotionPlan->robotRepresentation->getDimensionCount() - 1));
	ef->objectives.push_back(new MPO_SmoothRobotMotionTrajectories(ef->theMotionPlan, "robot smooth body orientation trajectories", 1, 3, 5));
}

void LocomotionEngineManagerIPv1::warmStartMOpt() {
	double desSwingHeight = motionPlan->swingFootHeight;

	motionPlan->optimizeEndEffectorPositions = false;
	motionPlan->optimizeWheels = false;
	motionPlan->optimizeCOMPositions = false;
	motionPlan->optimizeCOMOrientations = false;
	motionPlan->optimizeRobotStates = false;
	motionPlan->optimizeContactForces = false;
	motionPlan->optimizeBarycentricWeights = true;
	motionPlan->swingFootHeight = 0.0;
	runMOPTStep();

	motionPlan->optimizeCOMPositions = true;
	motionPlan->optimizeRobotStates = true;
	motionPlan->optimizeBarycentricWeights = true;

	for (int i = 0; i < 2; i++)
		runMOPTStep();
	motionPlan->swingFootHeight = desSwingHeight / 2.0;
	for (int i = 0; i < 5; i++)
		runMOPTStep();
	motionPlan->swingFootHeight = desSwingHeight;
	for (int i = 0; i < 5; i++)
		runMOPTStep();

	motionPlan->optimizeWheels = true;
	motionPlan->optimizeEndEffectorPositions = true;
	for (int i = 0; i < 5; i++)
		runMOPTStep();

	setDefaultOptimizationFlags();
}

void LocomotionEngineManagerIPv1::setDefaultOptimizationFlags() {
	motionPlan->optimizeEndEffectorPositions = true;
	motionPlan->optimizeWheels = true;
	motionPlan->optimizeCOMPositions = true;
	motionPlan->optimizeCOMOrientations = false;
	motionPlan->optimizeRobotStates = true;
	motionPlan->optimizeContactForces = false;
	motionPlan->optimizeBarycentricWeights = true;
}

/***************************************************************************************/
/* IPv2 uses separate DOFs for rotations.                                              */
/***************************************************************************************/
LocomotionEngineManagerIPv2::LocomotionEngineManagerIPv2() {

}

LocomotionEngineManagerIPv2::LocomotionEngineManagerIPv2(Robot* robot, FootFallPattern* footFallPattern, int nSamplePoints) : LocomotionEngineManagerIP(robot, footFallPattern, nSamplePoints){
	setupObjectives();
}

LocomotionEngineManagerIPv2::~LocomotionEngineManagerIPv2() {
}

void LocomotionEngineManagerIPv2::setupObjectives() {
	LocomotionEngine_EnergyFunction* ef = energyFunction;
	for (uint i = 0; i<ef->objectives.size(); i++)
		delete ef->objectives[i];
	ef->objectives.clear();

	//soft constraints
	ef->objectives.push_back(new MPO_DynamicStabilityObjective(ef->theMotionPlan, "dynamic stability objective", 10000.0));
	ef->objectives.push_back(new MPO_RobotCOMObjective(ef->theMotionPlan, "robot COM objective", 10000.0));
	ef->objectives.push_back(new MPO_RobotEndEffectorsObjective(ef->theMotionPlan, "robot EE objective", 10000.0));
	ef->objectives.push_back(new MPO_RobotCOMOrientationsObjective(ef->theMotionPlan, "robot COM orientations objective", 10000.0));

	//functional objectives
	ef->objectives.push_back(new MPO_COMTravelObjective(ef->theMotionPlan, "COM Travel objective", 50.0));
	ef->objectives.push_back(new MPO_COMTurningObjective(ef->theMotionPlan, "robot turning rate (YAW)", 50.0));

	//motion regularizers
	ef->objectives.push_back(new MPO_SmoothCOMTrajectories(ef->theMotionPlan, "smoothCOM", 10));
	ef->objectives.push_back(new MPO_FeetPathSmoothnessObjective(ef->theMotionPlan, "foot path smoothness objective", 1.0));
	ef->objectives.push_back(new MPO_BarycentricWeightsRegularizerObjective(ef->theMotionPlan, "barycentric weights regularizer objective", 0.001 * 0.001));

	ef->objectives.push_back(new MPO_SmoothStanceLegMotionObjective(ef->theMotionPlan, "robot stance leg smooth joint angle trajectories", 0.01));
	ef->objectives.push_back(new MPO_StanceLegMotionRegularizer(ef->theMotionPlan, "robot stance legs motion regularizer", 0.01));
	ef->objectives.push_back(new MPO_RobotStateRegularizer(ef->theMotionPlan, "robot joint angles regularizer objective", 0.0010 * 10, 6, ef->theMotionPlan->robotRepresentation->getDimensionCount() - 1));
	ef->objectives.push_back(new MPO_RobotStateRegularizer(ef->theMotionPlan, "robot body orientation regularizer objective (ROLL)", 1, 5, 5));
	ef->objectives.push_back(new MPO_RobotStateRegularizer(ef->theMotionPlan, "robot body orientation regularizer objective (PITCH)", 1, 4, 4));
	ef->objectives.push_back(new MPO_RobotStateRegularizer(ef->theMotionPlan, "robot body orientation regularizer objective (YAW)", 1, 3, 3));
	ef->objectives.push_back(new MPO_SmoothRobotMotionTrajectories(ef->theMotionPlan, "robot smooth joint angle trajectories", 0.001, 6, ef->theMotionPlan->robotRepresentation->getDimensionCount() - 1));
	ef->objectives.push_back(new MPO_SmoothRobotMotionTrajectories(ef->theMotionPlan, "robot smooth body orientation trajectories", 1, 3, 5));
}

void LocomotionEngineManagerIPv2::warmStartMOpt(){
	double desSwingHeight = motionPlan->swingFootHeight;

	motionPlan->optimizeEndEffectorPositions = false;
	motionPlan->optimizeWheels = false;
	motionPlan->optimizeCOMPositions = false;
	motionPlan->optimizeCOMOrientations = false;
	motionPlan->optimizeRobotStates = false;
	motionPlan->optimizeContactForces = false;
	motionPlan->optimizeBarycentricWeights = true;
	motionPlan->swingFootHeight = 0.0;
	runMOPTStep();

	motionPlan->optimizeCOMPositions = true;
	motionPlan->optimizeRobotStates = true;
	motionPlan->optimizeBarycentricWeights = true;
	motionPlan->optimizeCOMOrientations = true;

	for (int i = 0; i < 2; i++)
		runMOPTStep();
	motionPlan->swingFootHeight = desSwingHeight / 2.0;
	for (int i = 0; i < 5; i++)
		runMOPTStep();
	motionPlan->swingFootHeight = desSwingHeight;
	for (int i = 0; i < 5; i++)
		runMOPTStep();

	motionPlan->optimizeEndEffectorPositions = true;
	motionPlan->optimizeWheels = true;
	for (int i = 0; i < 5; i++)
		runMOPTStep();

	setDefaultOptimizationFlags();
}

void LocomotionEngineManagerIPv2::setDefaultOptimizationFlags() {
	motionPlan->optimizeEndEffectorPositions = true;
	motionPlan->optimizeWheels = true;
	motionPlan->optimizeCOMPositions = true;
	motionPlan->optimizeCOMOrientations = true;
	motionPlan->optimizeRobotStates = true;
	motionPlan->optimizeContactForces = false;
	motionPlan->optimizeBarycentricWeights = true;
}
