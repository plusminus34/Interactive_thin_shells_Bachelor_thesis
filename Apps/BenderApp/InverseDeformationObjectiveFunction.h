#pragma once

#include <OptimizationLib/ObjectiveFunction.h>

template<int NDim>
class InverseDeformationSolver;


template<int NDim>
class InverseDeformationObjectiveFunction : public ObjectiveFunction {

public:

	InverseDeformationSolver<NDim> * idSolver;

private:
	bool use_regularizer = false;
	double regularizer;
	dVector p0_reg;

public:
	InverseDeformationObjectiveFunction() {};
	InverseDeformationObjectiveFunction(InverseDeformationSolver<NDim> * idSolver) : idSolver(idSolver) {};

	virtual double computeValue(const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& s);
	virtual void setCurrentBestSolution(const dVector& s);


	void updateRegularizingSolutionTo(const dVector &p0_new);
	void setRegularizerValue(double r);

	void setRegularizer(double r, const dVector& p0);
	void unsetRegularizer();

};