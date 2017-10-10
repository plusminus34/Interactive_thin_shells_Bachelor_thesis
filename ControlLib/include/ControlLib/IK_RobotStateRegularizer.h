#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <MathLib/Matrix.h>
#include <ControlLib/IK_Plan.h>

class IK_RobotStateRegularizer : public ObjectiveFunction {
public:
	IK_RobotStateRegularizer(IK_Plan* mp, int startQIndex, int endQIndex, const std::string& objectiveDescription, double weight);
	virtual ~IK_RobotStateRegularizer(void);

	virtual double computeValue(const dVector& p);
	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);


private:

	//the energy function operates on a motion plan...
	IK_Plan* IKPlan;

	//these are the start and end indices for the parts of the state space that we penalize
	int startQIndex, endQIndex;
};

