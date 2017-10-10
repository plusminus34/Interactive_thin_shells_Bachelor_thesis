#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <MathLib/Matrix.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>
#include <OptimizationLib/SoftUnilateralConstraint.h>


class MPO_PseudoLimbLengthConstraint : public ObjectiveFunction {
public:
	MPO_PseudoLimbLengthConstraint(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight);
	virtual ~MPO_PseudoLimbLengthConstraint(void);

	virtual double computeValue(const dVector& p);

	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

	double tolerance = 1.5;

private:
	//the energy function operates on a motion plan...
	LocomotionEngineMotionPlan* theMotionPlan;

	double pseudoLimbEpsilon;

};