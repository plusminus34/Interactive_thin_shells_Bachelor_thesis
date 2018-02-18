#include <OptimizationLib/GradientBasedFunctionMinimizer.h>

#include <iostream>


GradientBasedFunctionMinimizer::GradientBasedFunctionMinimizer(int p_maxIterations, double p_solveResidual, int p_maxLineSearchIterations, bool p_printOutput){
	maxIterations = p_maxIterations;
	maxLineSearchIterations = p_maxLineSearchIterations;
	printOutput = p_printOutput;
	solveResidual = p_solveResidual;
}

GradientBasedFunctionMinimizer::~GradientBasedFunctionMinimizer(){
}

bool GradientBasedFunctionMinimizer::minimize(ObjectiveFunction *function, dVector &p, double& functionValue){
	if(printOutput){
		Logger::logPrint("Starting %s...\n", optName.c_str());
	}

	//number of parameters...
	int N = (int) p.size();
	resize(pi, N);
	resize(dp, N);
	resize(gradient, N);

	pi = p;

	function->setCurrentBestSolution(pi);

	bool optimizationConverged = false;

	for(int i=0; i < maxIterations; i++) {
		if (printOutput) {
			gradient.setZero();
			function->addGradientTo(gradient, pi);
			Logger::logPrint("Iteration: %d. Initial function value: %10.10lf. Gradient norm: %lf\n", i, function->computeValue(pi), gradient.norm());
		}

		timer.restart();
		computeSearchDirection(function, pi, dp);

		if (printOutput)
			Logger::logPrint("\tTime to compute search direction: %lf\n", timer.timeEllapsed());

		if (printOutput) {
			gradient.setZero();
			function->addGradientTo(gradient, pi);
			Logger::logPrint("\tSearch direction norm: %lf. Gradient norm: %lf. g.dp: %lf\n", dp.norm(), gradient.norm(), gradient.dot(dp));
		}

		if (dp.norm() < solveResidual){
			optimizationConverged = true;
			break;
		}

		timer.restart();
		doLineSearch(function, pi, dp);
		if (printOutput)
			Logger::logPrint("\tTime to do line search: %lf\n", timer.timeEllapsed());

		function->setCurrentBestSolution(pi);
	}

	functionValue = function->computeValue(pi);

	if(printOutput) {
		Logger::logPrint("Done optimization step. Final function value: %10.10lf\n", functionValue);
		
		if (optimizationConverged){
			Logger::logPrint("Converged! Gradient norm: %lf. FunctionValue: %10.10lf\n", dp.norm(), functionValue);
		}else{
			Logger::logPrint("Did NOT converge! Gradient norm: %lf. FunctionValue: %10.10lf\n", dp.norm(), functionValue);
		}
	}

	if (printOutput && optimizationConverged)
		Logger::logPrint("Converged! Gradient norm: %lf. FunctionValue: %10.10lf\n", dp.norm(), functionValue);

	//p now holds the parameter values at the start of the iteration...
	p = pi;
	//and done!
	return optimizationConverged;
}

double GradientBasedFunctionMinimizer::doLineSearch(ObjectiveFunction *function, dVector& pi, const dVector& dp){
	// line search now...
	double alpha = lineSearchStartValue;
	dVector pc(pi);
	double initialValue = function->computeValue(pc);

	double newLineSearchValue;
	for(int j = 0; j < maxLineSearchIterations; j++) {
		// try a new solution
		pi = pc - dp * alpha;

		// now check the new function value at this point...
		newLineSearchValue = function->computeValue(pi);

		if (printOutput)
			Logger::logPrint("\t--> LINE SEARCH iteration %02d: alpha is %10.10lf, function value is: %10.10lf\n", j, alpha, newLineSearchValue);

		if(isfinite(newLineSearchValue) && newLineSearchValue < initialValue) {
			// found a good value: return it!
			return(alpha);
		} else {
			// no good value: keep searching
			alpha /= 2.0;
		}

	}

	// no good value could be found at all: return the initial set of parameters
	alpha = 0.0;
	pi = pc;

	//alpha = 0.0;
	//pi = pc + dp*alpha;

	return alpha;
}

