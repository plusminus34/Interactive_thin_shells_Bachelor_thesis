#include <RobotDesignerLib/LocomotionEngineManagerIP.h>

LocomotionEngineManagerIP::LocomotionEngineManagerIP(){
}

LocomotionEngineManagerIP::LocomotionEngineManagerIP(Robot* robot, FootFallPattern* footFallPattern, int nSamplePoints){
	
	if (footFallPattern)
	{
		this->footFallPattern = footFallPattern;
		origFootFallPattern = *footFallPattern;
	}
	else {
		this->footFallPattern = &origFootFallPattern;
	}

	motionPlan = new LocomotionEngineMotionPlan(robot, nSamplePoints);

	motionPlan->minBaricentricWeight = 0.15;

	motionPlan->syncMotionPlanWithFootFallPattern(*footFallPattern);

	//for boundary conditions, make sure initial and final conditions match
	motionPlan->setPeriodicBoundaryConditionsToTimeSample(0);

	locomotionEngine = new LocomotionEngine(motionPlan);
	locomotionEngine->energyFunction->setupIPSubObjectives();
}

void LocomotionEngineManagerIP::warmStartMOpt()
{
	double desSwingHeight = motionPlan->swingFootHeight;

	motionPlan->optimizeEndEffectorPositions = false;
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

	motionPlan->optimizeEndEffectorPositions = true;
	for (int i = 0; i < 5; i++)
		runMOPTStep();
}

LocomotionEngineManagerIP::~LocomotionEngineManagerIP(){
	delete motionPlan;
	delete locomotionEngine;
}

LocomotionEngineManagerIPv2::LocomotionEngineManagerIPv2() {

}

LocomotionEngineManagerIPv2::LocomotionEngineManagerIPv2(Robot* robot, FootFallPattern* footFallPattern, int nSamplePoints){
	
	if (footFallPattern)
	{
		this->footFallPattern = footFallPattern;
		origFootFallPattern = *footFallPattern;
	}
	else {
		this->footFallPattern = &origFootFallPattern;
	}

	motionPlan = new LocomotionEngineMotionPlan(robot, nSamplePoints);

	motionPlan->minBaricentricWeight = 0.15;

	motionPlan->syncMotionPlanWithFootFallPattern(*footFallPattern);

	//for boundary conditions, make sure initial and final conditions match
	motionPlan->setPeriodicBoundaryConditionsToTimeSample(0);

	locomotionEngine = new LocomotionEngine(motionPlan);
	locomotionEngine->energyFunction->setupIPv2SubObjectives();
}

LocomotionEngineManagerIPv2::~LocomotionEngineManagerIPv2() {

}

void LocomotionEngineManagerIPv2::warmStartMOpt()
{
	double desSwingHeight = motionPlan->swingFootHeight;

	motionPlan->optimizeEndEffectorPositions = false;
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
	for (int i = 0; i < 5; i++)
		runMOPTStep();
}

