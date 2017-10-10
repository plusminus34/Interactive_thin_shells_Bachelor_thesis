#include <RobotDesignerLib/LocomotionEngine.h>
#include <OptimizationLib/NewtonFunctionMinimizer.h>


LocomotionEngine::LocomotionEngine(LocomotionEngineMotionPlan* motionPlan){
	this->motionPlan = motionPlan;
	energyFunction = new LocomotionEngine_EnergyFunction(this->motionPlan);
	constraints = new LocomotionEngine_Constraints(this->motionPlan);
	constrainedObjectiveFunction = new ConstrainedObjectiveFunction(energyFunction, constraints);
	BFGSSQPminimizer = new SQPFunctionMinimizer_BFGS(10);
	BFGSminimizer = new BFGSFunctionMinimizer(10);
}

LocomotionEngine::~LocomotionEngine(void){
	delete energyFunction;
	delete constraints;
	delete constrainedObjectiveFunction;
	delete BFGSSQPminimizer;
	delete BFGSminimizer;
}

double LocomotionEngine::optimizePlan(int maxIterations){
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

double LocomotionEngine::optimizePlan_BFGS(){
	double val = 0;
	dVector params;
	this->motionPlan->writeMPParametersToList(params);

	if (params.size() == 0)
		return 0;
	if (energyFunction->printDebugInfo)
		Logger::consolePrint("Problem size: %d\n", params.size());

	if (useObjectivesOnly) {
		BFGSminimizer->maxLineSearchIterations = 12;
		BFGSminimizer->printOutput = energyFunction->printDebugInfo;
		BFGSminimizer->minimize(energyFunction, params, val);
	}
	else {
		BFGSSQPminimizer->maxLineSearchIterations_ = 12;
		BFGSSQPminimizer->printOutput_ = energyFunction->printDebugInfo;
		BFGSSQPminimizer->minimize(constrainedObjectiveFunction, params, val);
	}
	return val;
}

