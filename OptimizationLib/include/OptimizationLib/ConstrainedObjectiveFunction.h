#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <OptimizationLib/FunctionConstraints.h>

/*!
	A multi-dimensional function that expresses linear equality and inequality constraints that might be applied to an objective function.
*/
class ConstrainedObjectiveFunction {
protected:
	ObjectiveFunction* objective_;
	FunctionConstraints* constraints_;
public:
	ConstrainedObjectiveFunction(ObjectiveFunction* obj, FunctionConstraints* C);
	virtual ~ConstrainedObjectiveFunction();

	ObjectiveFunction* getObjectiveFunction(){return objective_;}
	FunctionConstraints* getFunctionConstraints(){return constraints_;}

	void setCurrentBestSolution(const dVector& p);
};

