#pragma once
#include <OptimizationLib/ConstrainedObjectiveFunction.h>
#include <OptimizationLib/ObjectiveFunction.h>
#include <OptimizationLib/SQPFunctionMinimizer.h>
#include <OptimizationLib/BFGSHessianApproximator.h>

/**
	Use the Sequential Quadratic Programming method to optimize a function, subject to constraints. However, a BFGS solver is used to solve the quadratic objective part,
	instead of computing the Hessian. This can be useful if the Hessian is difficult to compute, or if it is not positive-definite.
*/
class SQPFunctionMinimizer_BFGS : public SQPFunctionMinimizer {
protected:
	 virtual void computeGradient(ConstrainedObjectiveFunction *function, const dVector& pi);
	 virtual void computeHessian(ConstrainedObjectiveFunction *function, const dVector& pi);

	 BFGSHessianApproximator bfgsApproximator;
public:
	SQPFunctionMinimizer_BFGS(int maxIterations = 2000,
		double solveResidual = 0.0001,
		double regularizer = 0.0,
		int maxLineSearchIterations = 0,
		double solveFunctionValue = -DBL_MAX,
		bool printOutput = false,
		bool checkConstraints = false) : SQPFunctionMinimizer(maxIterations, solveResidual, regularizer, maxLineSearchIterations, solveFunctionValue, printOutput, checkConstraints) {}
};

