#pragma once

#include <memory>

#include <OptimizationLib/ObjectiveFunction.h>

template<int NDim>
class InverseDeformationSolver;
template<int NDim>
class ParameterValueRegularizer;


template<int NDim>
class InverseDeformationObjectiveFunction : public ObjectiveFunction {

public:

	InverseDeformationSolver<NDim> * idSolver;

	std::vector<ObjectiveFunction *> parameterConstraints;
	std::vector<ObjectiveFunction *> collisionAvoidance;
	ParameterValueRegularizer<NDim> parameterValueRegularizer;
	ParameterValueRegularizer<NDim> parameterStepSizeRegularizer;


private:


	//bool use_regularizer = false;
	//double regularizer;
	//dVector p0_reg;

public:
	InverseDeformationObjectiveFunction() {};
	InverseDeformationObjectiveFunction(InverseDeformationSolver<NDim> * idSolver) : idSolver(idSolver) {};

	virtual double computeValue(const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& s);
	virtual void setCurrentBestSolution(const dVector& s);

	void setReferenceStateP();


	//void updateRegularizingSolutionTo(const dVector &p0_new);
	//void setRegularizerValue(double r);

	//void setRegulatiyerSolution
	//void setRegularizer(double r, const dVector& p0);
	//void unsetRegularizer();

};



template<int NDim>
class ParameterValueRegularizer : public ObjectiveFunction {
public:
	double r = 0;
	dVector pRef;
public:
	virtual double computeValue(const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

	void setReferenceState(dVector const & p);
};

