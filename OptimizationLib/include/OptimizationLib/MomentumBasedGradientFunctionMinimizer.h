#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <OptimizationLib/GradientBasedFunctionMinimizer.h>

// https://distill.pub/2017/momentum/ - Why Momentum Really Works

class MomentumBasedGradientFunctionMinimizer : public GradientBasedFunctionMinimizer {
public:
	MomentumBasedGradientFunctionMinimizer(bool enableLineSearch, double initialStep, double beta,
		int p_maxIterations = 100, double p_solveResidual = 0.0001, int p_maxLineSearchIterations = 15, bool p_printOutput = false) :
		GradientBasedFunctionMinimizer(p_maxIterations, p_solveResidual, p_maxLineSearchIterations, p_printOutput) {
		optName = "Gradient Descent with Momentum";
		this->enableLineSearch = enableLineSearch;
		this->initialStep = initialStep;
		this->beta = beta;
	}

	MomentumBasedGradientFunctionMinimizer() {
		optName = "Gradient Descent with Momentum";
	}

	virtual ~MomentumBasedGradientFunctionMinimizer() {}

	// Since the gradient of a function gives the direction of steepest Momentum, all one needs to do is go in that direction...
	virtual void computeSearchDirection(ObjectiveFunction *function, const dVector &p, dVector& dp);

	//Beichen Li: modified minimization function that accepts dpLast as input. After calculation it is replaced by new dp.
	virtual bool minimize(ObjectiveFunction *function, dVector &p, double &functionValue, dVector& dpLast);
	virtual bool minimize(ObjectiveFunction *function, dVector &p, double &functionValue);

	//Beichen Li: modified line search function that enables parameter control
	virtual double doLineSearch(ObjectiveFunction *function, dVector& pi, const dVector& dp);

	//Beichen Li: last update dp is stored in dpLast
	dVector dpLast;

	//Advanced control parameters
	//Beichen Li: enable line search
	bool enableLineSearch = true;

	//Beichen Li: initial step in line search
	double initialStep = 1.0;

	//Beichen Li: beta parameter (the effect of dpLast on new dp)
	double beta = 0.5;
};
