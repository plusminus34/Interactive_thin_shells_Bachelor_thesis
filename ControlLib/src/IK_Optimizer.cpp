#include <ControlLib/IK_Optimizer.h>
#include <OptimizationLib/NewtonFunctionMinimizer.h>

IK_Optimizer::IK_Optimizer(IK_Plan* IKPlan, IK_EnergyFunction* energyFunction){
	this->IKPlan = IKPlan;
	this->energyFunction = energyFunction;
}

IK_Optimizer::~IK_Optimizer(void){
	//delete energyFunction;
}

double IK_Optimizer::optimizePlan(int maxIterNum){
	NewtonFunctionMinimizer minimizer(maxIterNum);

	minimizer.printOutput = false;//true;
	dVector params;
	this->IKPlan->writeParametersToList(params);

	if (checkDerivatives){
		energyFunction->testGradientWithFD(params);
		energyFunction->testHessianWithFD(params);
	}

	double val = 0;
	minimizer.minimize(energyFunction, params, val);
//	Logger::consolePrint("IK energy val: %lf\n", val);

	return val;
}
