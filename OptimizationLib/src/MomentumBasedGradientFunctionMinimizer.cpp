#include <OptimizationLib/MomentumBasedGradientFunctionMinimizer.h>
#include <OptimizationLib/ObjectiveFunction.h>

void MomentumBasedGradientFunctionMinimizer::computeSearchDirection(ObjectiveFunction *function, const dVector& p, dVector& dp) {
	dVector grad;
	function->addGradientTo(grad, p);
	if (dpLast.size() != p.size())
		resize(dpLast, p.size());
	dp = beta * dpLast + grad;
	dpLast = dp;
}

bool MomentumBasedGradientFunctionMinimizer::minimize(ObjectiveFunction *function, dVector &p, double &functionValue, dVector& dpLast) {
	this->dpLast = dpLast;
	bool ret = GradientBasedFunctionMinimizer::minimize(function, p, functionValue);
	dpLast = this->dpLast;
	return ret;
}

bool MomentumBasedGradientFunctionMinimizer::minimize(ObjectiveFunction *function, dVector &p, double &functionValue) {
	return GradientBasedFunctionMinimizer::minimize(function, p, functionValue);
}


double MomentumBasedGradientFunctionMinimizer::doLineSearch(ObjectiveFunction *function, dVector& pi, const dVector& dp) {
	if (enableLineSearch) {
		lineSearchStartValue = initialStep;
		return GradientBasedFunctionMinimizer::doLineSearch(function, pi, dp);
	}

	pi -= dp * initialStep;
	return initialStep;
}
