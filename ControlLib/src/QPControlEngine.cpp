#include <ControlLib/QPControlEngine.h>
#include <OptimizationLib/NewtonFunctionMinimizer.h>
#include <OptimizationLib/SQPFunctionMinimizer.h>
#include <OptimizationLib/SQPFunctionMinimizer_BFGS.h>


QPControlEngine::QPControlEngine(QPControlPlan* qpPlan){
	this->qpPlan = qpPlan;
	energyFunction = new QPControlEnergyFunction(this->qpPlan);
	constraints = new QPControlConstraints(this->qpPlan);
	constrainedObjectiveFunction = new ConstrainedObjectiveFunction(energyFunction, constraints);
}

QPControlEngine::~QPControlEngine(void){
	delete energyFunction;
	delete constraints;
	delete constrainedObjectiveFunction;
}

double QPControlEngine::optimizePlan(){
	SQPFunctionMinimizer minimizer(1);

	minimizer.maxLineSearchIterations_ = 1;
	minimizer.printOutput_ = energyFunction->printDebugInfo;
	dVector params;

	this->qpPlan->writeParametersToList(params);

//	constrainedObjectiveFunction->getObjectiveFunction()->testGradientWithFD(params);

	//Logger::printStatic("There are %d parameters\n", params.size());
	double val = 0;
	if(params.size() == 0)
		return 0;
	if (minimizer.printOutput_)
		Logger::consolePrint("Problem size: %d\n", params.size());
	minimizer.minimize(constrainedObjectiveFunction, params, val);
	return val;
}

