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
	double val = 0;
	for (int i = 0; i < 1; i++) {
		if (checkDerivatives) {
			energyFunction->testGradientWithFD(params);
			energyFunction->testHessianWithFD(params);
		}

		minimizer.minimize(energyFunction, params, val);
		if (minimizer.printOutput)
			Logger::consolePrint("Final IK energy val: %lf\n", val);
	}

	return val;
}
