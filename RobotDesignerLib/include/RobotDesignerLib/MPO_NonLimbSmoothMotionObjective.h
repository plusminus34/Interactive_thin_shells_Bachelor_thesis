#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <MathLib/Matrix.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>

class MPO_NonLimbSmoothMotionObjective : public ObjectiveFunction {
public:
	MPO_NonLimbSmoothMotionObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight);
	virtual ~MPO_NonLimbSmoothMotionObjective(void);

	virtual double computeValue(const dVector& p);

	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

private:
	//the energy function operates on a motion plan...
	LocomotionEngineMotionPlan* theMotionPlan;
	DynamicArray<int> nonLimbJointIndices;
};

