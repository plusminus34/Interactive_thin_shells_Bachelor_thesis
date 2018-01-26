#include <utility>

#include "ParameterConstraintObjective.h"




ParameterConstraintObjective::ParameterConstraintObjective(ParameterSet * parameterSet, int parameterIndexLocal,
														   bool useLowerLimit, bool useUpperLimit,
														   double stiffness, double epsilon)
	: parameterSet(parameterSet), parameterIndexLocal(parameterIndexLocal)
{
	std::pair<double, double> limits = parameterSet->getParameterLimitsByLocalIdx(parameterIndexLocal);

	if(useLowerLimit) {
		lowerConstraint = new SoftUnilateralConstraint(limits.first, stiffness, epsilon);
	}
	if(useUpperLimit) {
		upperConstraint = new SoftUnilateralUpperConstraint(limits.second, stiffness, epsilon);
	}
}


ParameterConstraintObjective::~ParameterConstraintObjective()
{
	if(lowerConstraint) {
		delete lowerConstraint;
	}
	if(upperConstraint) {
		delete upperConstraint;
	}
}


double ParameterConstraintObjective::computeValue(const dVector& p)
{
	int parameterIndexGlobal = parameterIndexLocal + parameterSet->parametersStartIndex;

	double result = 0;
	if(lowerConstraint) {
		result += lowerConstraint->computeValue(p[parameterIndexGlobal]);
	}
	if(upperConstraint) {
		result += upperConstraint->computeValue(p[parameterIndexGlobal]);
	}

	return(result);
}

void ParameterConstraintObjective::addGradientTo(dVector& grad, const dVector& p)
{
	int parameterIndexGlobal = parameterIndexLocal + parameterSet->parametersStartIndex;

	if(lowerConstraint) {
		grad[parameterIndexGlobal] += lowerConstraint->computeDerivative(p[parameterIndexGlobal]);
	}
	if(upperConstraint) {
		grad[parameterIndexGlobal]  += upperConstraint->computeDerivative(p[parameterIndexGlobal]);
	}

}



void ParameterConstraintObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p)
{
	int parameterIndexGlobal = parameterIndexLocal + parameterSet->parametersStartIndex;

	double hessian_i_i = 0.0;
	if(lowerConstraint) {
		hessian_i_i += lowerConstraint->computeSecondDerivative(p[parameterIndexGlobal]);
	}
	if(upperConstraint) {
		 hessian_i_i += upperConstraint->computeSecondDerivative(p[parameterIndexGlobal]);
	}

	hessianEntries.push_back(MTriplet(parameterIndexGlobal, parameterIndexGlobal, hessian_i_i));
}
