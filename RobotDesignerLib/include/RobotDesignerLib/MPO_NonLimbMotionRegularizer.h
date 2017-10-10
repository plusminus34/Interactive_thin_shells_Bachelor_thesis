#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <MathLib/Matrix.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>

/**
	Motion Plan Objective: Smooth COM trajectories
*/
class MPO_NonLimbMotionRegularizer : public ObjectiveFunction {
public:
	MPO_NonLimbMotionRegularizer(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight);
	virtual ~MPO_NonLimbMotionRegularizer(void);

	virtual double computeValue(const dVector& p);

	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

private:
	//the energy function operates on a motion plan...
	LocomotionEngineMotionPlan* theMotionPlan;
	DynamicArray<int> nonLimbJointIndices;
};

