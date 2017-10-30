#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <MathLib/Matrix.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>
#include <OptimizationLib/SoftUnilateralConstraint.h>

#include <memory>

#if 0
class MPO_GRFRegularizer : public ObjectiveFunction {
public:
	MPO_GRFRegularizer(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight);
	virtual ~MPO_GRFRegularizer(void);

	virtual double computeValue(const dVector& p);

	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

private:
	//the energy function operates on a motion plan...
	LocomotionEngineMotionPlan* theMotionPlan;
};

#endif // 0

class MPO_VelocitySoftBoundConstraints : public ObjectiveFunction {
public:
	MPO_VelocitySoftBoundConstraints(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight, int startQIndex, int endQIndex);
	virtual ~MPO_VelocitySoftBoundConstraints(void);

	virtual double computeValue(const dVector& p);

	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

private:
	//the energy function operates on a motion plan...
	LocomotionEngineMotionPlan* theMotionPlan;

	 int startQIndex, endQIndex; // TODO: not used right now!

	std::shared_ptr<SoftUnilateralConstraint> constraintLowerBound;
	std::shared_ptr<SoftUnilateralConstraint> constraintUpperBound;
};
