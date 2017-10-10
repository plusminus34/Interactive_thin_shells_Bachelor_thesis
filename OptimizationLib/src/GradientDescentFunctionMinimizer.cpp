#include <OptimizationLib/GradientDescentFunctionMinimizer.h>

//TODO: add momentum - http://distill.pub/2017/momentum/

/**
	Since the gradient of a function gives the direction of steepest descent, all one needs to do is go in that direction...
*/
void GradientDescentFunctionMinimizer::computeSearchDirection(ObjectiveFunction *function, const dVector &p, dVector& dp) {
	dp.setZero();
	function->addGradientTo(dp, p);
}
