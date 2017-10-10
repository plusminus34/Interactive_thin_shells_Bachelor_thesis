#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
extern "C"
{
#include <cmaes/cmaes_interface.h>
}

/**
	Minimizes a given function using the Covariance Matrix Adaptation evolution strategy method (CMA-ES).
	The function can have any form and can be noisy, no gradients are required.

	With this version, CMA is initialized once, and the iteration can be continued from the outside by calling
	'step()'.

	NOTE: Call setBounds(...) before calling the minimize(...) function. The given lower and upper bounds will
	be used to scale the state vector p to assume the same distribution over all dimensions, which impacts the performance.
	Also, provide an initial guess for the values of p.
*/
class CMAIterativeFunctionMinimizer
{
public:
	/**
		Verbosity level.
		0: No output (default).
		1: Only results are displayed.
		2: Output intermediate results at each iteration.
	*/
	int printLevel;

public:
	/**
		Constructor.
		- maxIterations: The maximum number of iterations taken by CMA. The maximum number of function iterations is maxIterations * populationSize.
		- populationSize: The number of sampling points taken at each CMA iteration. Choose a larger number to sample more densely within one generation.
		- initialStdDev: Initial value for standard deviation. CMA adjusts these values in the process. However, the initial value can impact performance. If you have confidence in your initial guess, choose a small value.
		- solveFunctionValue: The absolute objective function value at which CMA stops.
		- solveHistToleranceValue: Stop if the maximum function value difference of all iteration-best solutions of the last 10 + 30*N/lambda iterations become smaller than solveHistToleranceValue
	*/
	CMAIterativeFunctionMinimizer(int populationSize = 16, double solveFunctionValue = 0.001, double solveHistToleranceValue = 1e-13);
	virtual ~CMAIterativeFunctionMinimizer();

	/**
		Sets the objective.
		NOTE: We keep a pointer to 'function', but don't take ownership.
	*/
	void setObjective(ObjectiveFunction *function);

	/**
		Sets the initial standard deviation
	*/
	void setInitialStandardDeviation(double initialStdDev);
	void setInitialStandardDeviation(const dVector &initialStdDev, double stdDevModifier);

	/**
		Sets the initial guess, and an interval [pMin, pMax]
		Provides an interval [pMin, pMax] for parameter values p. p is scaled to [0,1] based on these value, which apparently increases CMA performance.
	*/
	void setInitialParameterSet(const dVector &p0, const dVector &pMin, const dVector &pMax);

	/**
		Initializes the CMA minimization given the information set by the setters.
		NOTE: The first call to 'step()' will call this, but you can call it yourself if you really want to
	*/
	void initialize();

	/**
		Continue optimization for 'numIterations' iterations.
		Returns true iff the iteration converged, which happens if either the objective is smaller than 'solveFunctionValue' or
		if a number of iterations yielded function values within 'solveHistToleranceValue' as given in the constructor.
	*/
	bool step(int numIterations = 1);

	/**
		Returns the number of parameters
	*/
	int getParameterCount() const { return (int)p.size(); }

	/**
		Returns the total number of iterations taken
	*/
	int getTotalNumberOfIterations() const { return totalNumberOfIterations; }

	/**
		Writes the current best solution to 'p' and returns f(p)
	*/
	double getCurrentBestSolution(dVector &p);
	
	/**
		Writes the current mean to 'p'
	*/
	void getCurrentMean(dVector &p);

	/**
		Writes the current standard deviation to 'stdDev'
	*/
	void getCurrentStdDev(dVector &stdDev);

	/**
		Replaces the current sampling mean by the given values
		NOTE: Only call this between two calls to 'step()'.
			  Should not be done repeatedly, could lead to unexpected behaviour
	*/
	void setCurrentMean(const dVector &mean);

	/**
		Widens or narrows the current sampling ellipsoid by the given factor.
	*/
	void multiplyStdDevBy(double factor);

protected:
	ObjectiveFunction *function;
	dVector p;
	dVector pMin, pMax;
	int populationSize;
	double solveFunctionValue;
	double solveHistToleranceValue;
	double initialStdDevValue;
	dVector initialStdDev;

	cmaes_t evo;
	double *arFunVals;

	bool initialized;
	int totalNumberOfIterations;
};
