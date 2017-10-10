#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>

/**
	Motion Plan Objective: Dynamic Stability Criteria
*/

class MPO_DynamicStabilityObjective : public ObjectiveFunction {
public:
	MPO_DynamicStabilityObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight);
	virtual ~MPO_DynamicStabilityObjective(void);

	virtual double computeValue(const dVector& p);

	//evaluates the error vector at sample j
	V3D getErrorVector(int j);

	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

private:
	//the energy function operates on a motion plan...
	LocomotionEngineMotionPlan* theMotionPlan;
};

