#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>
#include <OptimizationLib/SoftUnilateralConstraint.h>

class MPO_GRFFrictionConstraints : public ObjectiveFunction {
public:
	MPO_GRFFrictionConstraints(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight);
	virtual ~MPO_GRFFrictionConstraints(void);

	virtual double computeValue(const dVector& p);

	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

private:
	//the energy function operates on a motion plan...
	LocomotionEngineMotionPlan* theMotionPlan;
};
