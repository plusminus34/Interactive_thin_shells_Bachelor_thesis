#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>

#include <memory>

class MPO_WheelSpeedTargetObjective : public ObjectiveFunction {
public:
	MPO_WheelSpeedTargetObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, int timeIndex, double targetWheelSpeed, double weight);
	virtual ~MPO_WheelSpeedTargetObjective(void);

	virtual double computeValue(const dVector& p);

	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

private:
	//the energy function operates on a motion plan...
	LocomotionEngineMotionPlan* theMotionPlan;

	int timeIndex;
	double targetWheelSpeed;
};
