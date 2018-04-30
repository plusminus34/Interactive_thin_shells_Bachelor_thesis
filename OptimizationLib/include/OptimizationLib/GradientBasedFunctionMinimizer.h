#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <string>
#include <Utils/Timer.h>

class GradientBasedFunctionMinimizer{
public:
	GradientBasedFunctionMinimizer(int p_maxIterations, double p_solveResidual, int p_maxLineSearchIterations, bool p_printOutput);
	GradientBasedFunctionMinimizer() {}

	virtual ~GradientBasedFunctionMinimizer();

	/**
		use gradient-based method to minimize this function.
	*/
	virtual bool minimize(ObjectiveFunction *function, dVector &p, double & functionValue);

	virtual void computeSearchDirection(ObjectiveFunction *function, const dVector &p, dVector& dp) = 0;
	virtual double doLineSearch(ObjectiveFunction *function, dVector& pi, const dVector& dp);

	double doLineSearchStandard(ObjectiveFunction *function, dVector& pi, const dVector& dp);
	double doLineSearchTweaked(ObjectiveFunction *function, dVector& pi, const dVector& dp);

public:

	double solveResidual = 1e-5;
	int maxIterations = 100;
	bool printOutput = false;
	std::string optName;

	bool useTweakedLineSearch = false;
	// "standard" line search
	double lineSearchStartValue = 1.0;
	int maxLineSearchIterations = 15;

	// "tweaked" line search
	double lineSearchEndValue = 3.0e-5;
	bool adaptiveLineSearch = false;
	int lineSearchIterationLimit = -1;

protected:
	dVector pi, dp, gradient;
	Timer timer;

protected:
	double lineSearchValueOld = -1.0;
};

