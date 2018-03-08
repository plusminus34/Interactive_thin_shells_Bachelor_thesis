#pragma once

//#define USE_PARDISO

#include <OptimizationLib/ObjectiveFunction.h>
#include <Utils/Timer.h>
#include <OptimizationLib/GradientBasedFunctionMinimizer.h>
#include <memory>

#include <Eigen/IterativeLinearSolvers>



#ifdef USE_PARDISO
#include <OptimizationLib/PardisoSolver.h>
#endif



/**
	use Newton's method to optimize a function. p will store the final value that minimizes the function, and its initial value
	is used to start the optimization method.

	Task: find p that minimize f(p). This means that df/dp(p) = 0.
	df/dp(p+dp) ~ df/dp(p) + d/dp(df/dp) * dp = 0 ==> -df/dp(p) = d/dp(df/dp) * dp
	Iterating the above, will hopefully get p that minimizes f.
*/
class LazyNewtonFunctionMinimizer : public GradientBasedFunctionMinimizer {

public:
	LazyNewtonFunctionMinimizer(int p_maxIterations = 100, double p_solveResidual = 0.0001, int p_maxLineSearchIterations = 15, bool p_printOutput = false) : GradientBasedFunctionMinimizer(p_maxIterations, p_solveResidual, p_maxLineSearchIterations, p_printOutput) {
		optName = "Newton\'s method";
#ifdef USE_PARDISO
		pardiso = std::make_unique<PardisoSolver<std::vector<int>, std::vector<double>>>();
		pardiso->set_type(2, true);
#endif
	}


	// TODO: ambiguos call
	//NewtonFunctionMinimizer() {
	//	optName = "Newton\'s method";
	//}

	virtual ~LazyNewtonFunctionMinimizer() {}

	// The search direction is given by -Hinv * g
	virtual void computeSearchDirection(ObjectiveFunction *function, const dVector &p, dVector& dp);

public:
	Timer timerN;
	SparseMatrix H;
	DynamicArray<MTriplet> hessianEntries;
	std::vector<double *> hessianEntries_Hptr;

	bool newHessianStructure = true;
	//Eigen::SimplicialLDLT<SparseMatrix, Eigen::Lower> solver;
	//Eigen::ConjugateGradient<SparseMatrix, Eigen::Lower|Eigen::Upper> solver;
	Eigen::BiCGSTAB<SparseMatrix, Eigen::IncompleteLUT<double> > solver;

	dVector dp_old;

	int nMaxStabSteps = 10;		// maximum number of stabilization steps
	double stabValue = 1e-4;	// value that gets added to hessian diagonal during stabilization step
	enum HessCorrectionMethod {
		None = 0,
		DynamicRegularization,
		Projection
	};
	HessCorrectionMethod hessCorrectionMethod = None;
#ifdef USE_PARDISO
	std::unique_ptr<PardisoSolver<std::vector<int>, std::vector<double>>> pardiso = nullptr;
#endif
};
