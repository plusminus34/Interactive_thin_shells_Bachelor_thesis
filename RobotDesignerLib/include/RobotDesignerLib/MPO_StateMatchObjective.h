#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <MathLib/Matrix.h>
#include "LocomotionEngineMotionPlan.h"

class MPO_StateMatchObjective : public ObjectiveFunction {
public:
	MPO_StateMatchObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight, int stateIndex, dVector& targetRobotState, bool includeGlobalStateDimensions = false);
	virtual ~MPO_StateMatchObjective(void);

	virtual double computeValue(const dVector& p);

	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

private:
	//the energy function operates on a motion plan...
	LocomotionEngineMotionPlan* theMotionPlan;
	dVector targetRobotState;

	//these are the indices for the parts of the state space that we penalize (y-axis, roll+pitch and all joint angles). This makes the objective invariant to where the robot is in the plane and to its yaw angle/orientation
	DynamicArray<int> qIndices;

	//in the state trajectory of the robot, this is the one the objective applies to...
	int stateIndex;
};

