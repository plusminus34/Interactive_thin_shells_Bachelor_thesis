#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <MathLib/Matrix.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>
#include <OptimizationLib/SoftUnilateralConstraint.h>

#include <memory>

class MPO_JointsAnglesSoftConstraint : public ObjectiveFunction {
public:
	MPO_JointsAnglesSoftConstraint(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight, int startQIndex, int endQIndex);
	virtual ~MPO_JointsAnglesSoftConstraint(void);

	virtual double computeValue(const dVector& p);

	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

private:
	//the energy function operates on a motion plan...
	LocomotionEngineMotionPlan* theMotionPlan;

	std::unique_ptr<SoftSymmetricBarrierConstraint> constraintSymmetricBound;

	int startQIndex, endQIndex;
};
