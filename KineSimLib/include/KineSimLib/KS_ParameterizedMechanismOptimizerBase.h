#pragma once

#include "KS_ParameterizedMechanicalAssembly.h"
#include <OptimizationLib/ObjectiveFunction.h>
#include <MathLib/Matrix.h>

/**
	This is an abstract implementation for objective functions that operates on parameterized mechanical assembly. These objective functions rely on the jacobian
	ds/dp, which is evaluated at a series of points (si, ti) (state and corresponding ticker value).
*/
class KS_ParameterizedMechanismOptimizerBase{
public:
	KS_ParameterizedMechanismOptimizerBase(KS_ParameterizedMechanicalAssembly* mechanism, int numberOfEvaluationPoints);
	virtual ~KS_ParameterizedMechanismOptimizerBase();

	bool computeAssemblyMotionGivenParameters(const dVector &p, DynamicArray<dVector> *stateArray, DynamicArray<double> *tickerValueArray);
	bool computeAssemblyMotionGivenParameters(const dVector &p);
	
	void updateStartingState();
	void loadStartingState();


protected:

	KS_ParameterizedMechanicalAssembly*	parameterizedAssembly;
	//over one run, keep track of the state of the assembly, the point trajectory, etc...
	dVector									startState;
	double									startTickerValue;
	int										numberOfEvaluationPoints;

private:
	//these are the cached states of the assembly that were computed last...
	dVector									cached_parameterSet;
	DynamicArray<dVector>					cached_assemblyStateArray;
	dVector									cached_correspondingTickerValues;
};


