#ifndef MPO_VELOCITY_SOFT_BOUND_CONSTRAINTS_H
#define MPO_VELOCITY_SOFT_BOUND_CONSTRAINTS_H

#include <OptimizationLib/ObjectiveFunction.h>
#include <MathLib/Matrix.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>
#include <OptimizationLib/SoftUnilateralConstraint.h>

#include <memory>

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

	std::unique_ptr<SoftSymmetricBarrierConstraint> constraintSymmetricBound;

	int startQIndex, endQIndex;
};

#endif // MPO_VELOCITY_SOFT_BOUND_CONSTRAINTS_H
