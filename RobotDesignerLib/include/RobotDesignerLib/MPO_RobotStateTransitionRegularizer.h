#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <MathLib/Matrix.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>

class MPO_RobotStateTransitionRegularizer : public ObjectiveFunction {

public:
	MPO_RobotStateTransitionRegularizer(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight, int startQIndex, int endQIndex, int stateIndex, dVector& targetRobotState);
	virtual ~MPO_RobotStateTransitionRegularizer(void);

	virtual double computeValue(const dVector& p);

	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

private:
	//the energy function operates on a motion plan...
	LocomotionEngineMotionPlan* theMotionPlan;
	dVector targetRobotState;

	//these are the start and end indices for the parts of the state space that we penalize
	int startQIndex, endQIndex;
	int stateIndex;
};

