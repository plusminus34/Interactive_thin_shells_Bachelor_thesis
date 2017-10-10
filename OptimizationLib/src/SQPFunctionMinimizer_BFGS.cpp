//TODO: all jacobians and constraints should be represented in terms of the triplets...
#include <MathLib/Matrix.h>

#include <OptimizationLib/SQPFunctionMinimizer_BFGS.h>
#include <OptimizationLib/ObjectiveFunction.h>

#include <OptimizationLib/OoqpEigenInterface.hpp>
#include <OptimizationLib/ooqpei_assert_macros.hpp>

#include <fstream>
#include <iostream>


void SQPFunctionMinimizer_BFGS::computeGradient(ConstrainedObjectiveFunction *function, const dVector& pi) {
	int nParameters = (int)pi.size();
	resize(gradient, nParameters);
	gradient.setZero();
	function->getObjectiveFunction()->addGradientTo(gradient, pi);

	if (printOutput_) {
		Logger::logPrint("SQP_BFGS: gradient norm: %lf, ", gradient.norm());
		Logger::consolePrint("SQP_BFGS: gradient norm: %lf, ", gradient.norm());
	}

	bfgsApproximator.add_x_and_dfdx_to_history(pi, gradient);
	bfgsApproximator.compute_Hinv_v_inplace(gradient);

	if (printOutput_) {
		Logger::logPrint("L-BFGS transformed gradient norm: %lf\n", gradient.norm());
		Logger::consolePrint("L-BFGS transformed gradient norm: %lf\n", gradient.norm());
	}
}

void SQPFunctionMinimizer_BFGS::computeHessian(ConstrainedObjectiveFunction *function, const dVector& pi) {
	int nParameters = (int)pi.size();
	resize(H, nParameters, nParameters);
	H.setIdentity();
//	H *= 1000;
}
