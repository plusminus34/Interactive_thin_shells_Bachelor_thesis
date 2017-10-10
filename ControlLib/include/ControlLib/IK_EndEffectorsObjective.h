#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <MathLib/Matrix.h>
#include <ControlLib/IK_Plan.h>

class IK_EndEffectorsObjective : public ObjectiveFunction {

public:
	IK_EndEffectorsObjective(IK_Plan* mp, const std::string& objectiveDescription, double weight);
	virtual ~IK_EndEffectorsObjective(void);

	virtual double computeValue(const dVector& p);
	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

private:
	//the energy function operates on a motion plan...
	IK_Plan* IKPlan;
};

