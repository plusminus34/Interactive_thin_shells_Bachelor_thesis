#pragma once

#include <OptimizationLib/ObjectiveFunction.h>

template<int NDim>
class InverseDeformationSolver;


template<int NDim>
class InverseDeformationObjectiveFunction : public ObjectiveFunction {

public:

	InverseDeformationSolver<NDim> * idSolver;

public:
	InverseDeformationObjectiveFunction() {};
	InverseDeformationObjectiveFunction(InverseDeformationSolver<NDim> * idSolver) : idSolver(idSolver) {};

	virtual double computeValue(const dVector& p);

	virtual void addGradientTo(dVector& grad, const dVector& s);

	virtual void setCurrentBestSolution(const dVector& s);
	
};