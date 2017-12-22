#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <MathLib/Matrix.h>
#include <ControlLib/IK_Plan.h>

class IK_EnergyFunction : public ObjectiveFunction {
public:
	DynamicArray<ObjectiveFunction*> objectives;

	IK_EnergyFunction(IK_Plan* IKPlan);
	virtual ~IK_EnergyFunction(void);

	//regularizer looks like: r/2 * (p-p0)'*(p-p0). This function can update p0 if desired, given the current value of s.
	void updateRegularizingSolutionTo(const dVector &currentP);
	virtual double computeValue(const dVector& p);

	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

	//this method gets called whenever a new best solution to the objective function is found
	virtual void setCurrentBestSolution(const dVector& p);

	void setupSubObjectives();

	void setupSubObjectives_EEMatch();

	bool printDebugInfo;

public:
	double regularizer;

private:
	//this is the solution used as a regularizer...
	dVector m_p0;
	dVector tmpVec;
	//the energy function operates on a motion plan...
	IK_Plan* IKPlan;
};

