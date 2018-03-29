#pragma once

#include <limits>

#include "OptimizationLib/ObjectiveFunction.h"
#include "OptimizationLib/SoftUnilateralConstraint.h"
#include "ParameterSet.h"





class ParameterConstraintObjective : public ObjectiveFunction {

private:
	ParameterSet * parameterSet;
	int parameterIndexLocal;

	SoftUnilateralConstraint* lowerConstraint = NULL;
	SoftUnilateralUpperConstraint* upperConstraint = NULL;

public : 
	ParameterConstraintObjective(ParameterSet * parameterSet, int parameterIndexLocal,
								 bool useLowerLimit, bool useUpperLimit,
								 double stiffness, double epsilon,
								 double cap_lower_limit = std::numeric_limits<double>::lowest(), double cap_upper_limit = std::numeric_limits<double>::max());

	~ParameterConstraintObjective();
	
	virtual double computeValue(const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);
	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);

};



