#pragma once

#include "KS_ParameterizedMechanicalAssembly.h"
#include <MathLib/ObjectiveFunction.h>
#include <MathLib/Matrix.h>

/**
	This is an abstract implementation for objective functions that operates on parameterized mechanical assembly. 
	This new version assumes that derivatives are not readily available, so it should be used with
	stochastic optimziation methods. By default, this class adds a term that prevents the assembly from
	getting close to singularities. More conretely, it penalizes the condition number of the constraint jacobian.
*/
class KS_ParameterizedMechanismObjectiveFunction_v2 : public ObjectiveFunction {
friend class KS_Optimizer;
friend class KS_WalkingMachinesOptimizer;
friend class KS_Optimizer;
friend class KS_Linkage_Optimizer;

public:
	KS_ParameterizedMechanismObjectiveFunction_v2(KS_ParameterizedMechanicalAssembly* mechanism, int numberOfEvaluationPoints, double solveRegularizer);
	virtual ~KS_ParameterizedMechanismObjectiveFunction_v2();
protected:
	//not using gradient information of any sort...
	virtual SparseMatrix* getCurrentHessian() {return NULL;}
	virtual dVector* getCurrentGradient(){return NULL;}
	virtual void precomputeDerivativeInformationAt(const dVector &p){}

	bool computeAssemblyMotionGivenParameters(const dVector &p, DynamicArray<dVector> *stateArray, DynamicArray<double> *tickerValueArray);
	//checks to make sure that the solution is valid... in particular, the states should not "flip", and each of the driving assemblies should be moving strictly forward...
	bool solutionIsValid(const dVector& safeAssemblyState, const dVector& newAssemblyStates);

	//this should always return the current value of the objective function
	virtual double computeValue(double const *p);

	virtual void setCurrentBestSolution(const dVector& p);

	KS_ParameterizedMechanicalAssembly*	parameterizedAssembly;

	//keep track of the best set of parameters, as well as the corresponding sampled states and the ticker values...
	dVector									startState;
	double									startTickerValue;
	dVector									best_parameterValues;
	DynamicArray<dVector>					best_assemblyStateArray;
	dVector									best_correspondingTickerValues;

	int numberOfEvaluationPoints;
	double solveRegularizer;

	//tmp variables that tell us what's the last set of parameters we tried, and the result we get...
	dVector									tmp_currentParameterSet;
	DynamicArray<dVector>					tmp_assemblyStateArray;
	dVector									tmp_correspondingTickerValues;

	double energyWeight, condNumberWeight;


};


