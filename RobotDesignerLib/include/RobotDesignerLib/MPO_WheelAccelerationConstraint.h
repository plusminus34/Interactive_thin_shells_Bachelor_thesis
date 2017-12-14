#ifndef MPO_WHEEL_ACCELERATION_CONSTRAINTS_H
#define MPO_WHEEL_ACCELERATION_CONSTRAINTS_H

#include <OptimizationLib/ObjectiveFunction.h>
#include <MathLib/Matrix.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>
#include <OptimizationLib/SoftUnilateralConstraint.h>

#include <memory>

class MPO_WheelAccelerationConstraints : public ObjectiveFunction {
public:
	MPO_WheelAccelerationConstraints(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight);
	virtual ~MPO_WheelAccelerationConstraints(void);

	virtual double computeValue(const dVector& p);

	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

private:
	//the energy function operates on a motion plan...
	LocomotionEngineMotionPlan* theMotionPlan;

	std::shared_ptr<SoftUnilateralConstraint> constraintLowerBound;
	std::shared_ptr<SoftUnilateralUpperConstraint> constraintUpperBound;
};

#endif // MPO_VELOCITY_SOFT_BOUND_CONSTRAINTS_H
