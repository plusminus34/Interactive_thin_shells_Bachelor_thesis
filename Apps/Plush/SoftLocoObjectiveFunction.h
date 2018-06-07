#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <MathLib/Matrix.h>

class SoftLocoSolver;

class SoftLocoObjectiveFunction : public ObjectiveFunction {

public:
	SoftLocoSolver *solver;

public:
	SoftLocoObjectiveFunction(SoftLocoSolver *);
	virtual ~SoftLocoObjectiveFunction(void) {}
 
	virtual double computeValue(const dVector &x); 
	virtual void addGradientTo(dVector &grad, const dVector &x);
	virtual void addHessianEntriesTo(DynamicArray<MTriplet> &hessianEntries, const dVector &x); 
	virtual void setCurrentBestSolution(const dVector& x);

};

