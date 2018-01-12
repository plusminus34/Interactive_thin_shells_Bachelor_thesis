#pragma once
#include <memory>
#include <OptimizationLib/ObjectiveFunction.h>
#include <MathLib/Matrix.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>
#include <OptimizationLib/SoftUnilateralConstraint.h>


class MPO_EndEffectorCollisionEnergy : public ObjectiveFunction {

public:
	MPO_EndEffectorCollisionEnergy(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight);
	virtual ~MPO_EndEffectorCollisionEnergy(void);

	virtual double computeValue(const dVector& p);

	virtual void addGradientTo(dVector& grad, const dVector& p);
	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);

private:
	LocomotionEngineMotionPlan* theMotionPlan;
	std::unique_ptr<SoftUnilateralConstraint> boundFunction;
};
