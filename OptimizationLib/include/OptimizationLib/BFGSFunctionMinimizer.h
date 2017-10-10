#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <OptimizationLib/GradientBasedFunctionMinimizer.h>
#include <OptimizationLib/BFGSHessianApproximator.h>

class BFGSFunctionMinimizer : public GradientBasedFunctionMinimizer {
public:
	BFGSFunctionMinimizer(int p_maxIterations = 100, double p_solveResidual = 0.0001, int p_maxLineSearchIterations = 15, bool p_printOutput = false) : GradientBasedFunctionMinimizer(p_maxIterations, p_solveResidual, p_maxLineSearchIterations, p_printOutput) {
		optName = "BFGS";
	}

	BFGSFunctionMinimizer() {
		optName = "BFGS";
	}

	virtual ~BFGSFunctionMinimizer() {}

	// Since the gradient of a function gives the direction of steepest descent, all one needs to do is go in that direction...
	virtual void computeSearchDirection(ObjectiveFunction *function, const dVector &p, dVector& dp);

public:
	BFGSHessianApproximator bfgsApproximator;
};

