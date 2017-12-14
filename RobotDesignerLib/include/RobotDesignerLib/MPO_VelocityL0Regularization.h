#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <MathLib/Matrix.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>

#include <memory>

class MPO_VelocityL0Regularization : public ObjectiveFunction {
public:
	MPO_VelocityL0Regularization(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight, int startQIndex, int endQIndex);
	virtual ~MPO_VelocityL0Regularization(void);

	virtual double computeValue(const dVector& p);

	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

private:
	//the energy function operates on a motion plan...
	LocomotionEngineMotionPlan* theMotionPlan;

	int startQIndex, endQIndex;
};
