#pragma once
#include <OptimizationLib/ObjectiveFunction.h>
#include <MathLib/Matrix.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>


class MPO_ForceAccelObjective : public ObjectiveFunction {
public:
	MPO_ForceAccelObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight);
	virtual ~MPO_ForceAccelObjective(void);

	virtual double computeValue(const dVector& p);

	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

private:
	//the energy function operates on a motion plan...
	LocomotionEngineMotionPlan* theMotionPlan;

	void addDiagonalHessianBlock(DynamicArray<MTriplet>& hessianEntries, int startIndexI, int startIndexJ, double val);
};
