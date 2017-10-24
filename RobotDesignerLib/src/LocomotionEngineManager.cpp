#include <RobotDesignerLib/LocomotionEngineManager.h>
#include <RobotDesignerLib/MPO_PeriodicRobotStateTrajectoriesObjective.h>

LocomotionEngineManager::LocomotionEngineManager(){
}

void LocomotionEngineManager::createSolverComponents() {
	energyFunction = new LocomotionEngine_EnergyFunction(motionPlan);
	constraints = new LocomotionEngine_Constraints(motionPlan);
	constrainedObjectiveFunction = new ConstrainedObjectiveFunction(energyFunction, constraints);
}

LocomotionEngineManager::~LocomotionEngineManager(void) {
	delete energyFunction;
	delete constraints;
	delete constrainedObjectiveFunction;
}

void LocomotionEngineManager::drawMotionPlan(double f, int animationCycle, bool drawRobot, bool drawSkeleton, bool drawPlanDetails, bool drawContactForces, bool drawOrientation){
	// motionPlan->syncMotionPlanWithFootFallPattern(*footFallPattern);
	motionPlan->drawMotionPlan(f, animationCycle, drawRobot, drawSkeleton, drawPlanDetails, drawContactForces, drawOrientation);
}

double LocomotionEngineManager::optimizeMoptionPlan(int maxIterations) {
	double val = 0;
	dVector params;
	this->motionPlan->writeMPParametersToList(params);
	if (params.size() == 0)
		return 0;
	if (energyFunction->printDebugInfo)
		Logger::logPrint("Starting Problem size: %d\n", params.size());

	if (useObjectivesOnly) {
		NewtonFunctionMinimizer minimizer(maxIterations);
		minimizer.maxLineSearchIterations = 12;
		minimizer.printOutput = energyFunction->printDebugInfo;
		minimizer.minimize(energyFunction, params, val);
	}
	else {
		SQPFunctionMinimizer minimizer(maxIterations);
		minimizer.maxLineSearchIterations_ = 12;
		minimizer.printOutput_ = energyFunction->printDebugInfo;
		//Logger::printStatic("There are %d parameters\n", params.size());
		minimizer.minimize(constrainedObjectiveFunction, params, val);
	}
	return val;
}

double LocomotionEngineManager::runMOPTStep() {
	motionPlan->syncMotionPlanWithFootFallPattern(*footFallPattern);

	Timer timer;
	if (checkDerivatives) {
		dVector params;
		motionPlan->writeMPParametersToList(params);

		energyFunction->testGradientWithFD(params);
		energyFunction->testHessianWithFD(params);
		energyFunction->testIndividualGradient(params);
		energyFunction->testIndividualHessian(params);
	}
	energyFunction->printDebugInfo = printDebugInfo;

	double energyVal = optimizeMoptionPlan();

	motionPlan->writeParamsToFile("..//out//MPParams.p");

	if (printDebugInfo)
		Logger::consolePrint("total time ellapsed: %lfs\n", timer.timeEllapsed());

	return energyVal;
}
