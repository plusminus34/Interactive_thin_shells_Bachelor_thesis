#pragma once


#include <OptimizationLib/ObjectiveFunction.h>
#include <MathLib/Matrix.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>

class MPO_PeriodicWheelTrajectoriesObjective : public ObjectiveFunction {

public:
	MPO_PeriodicWheelTrajectoriesObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight, int timeIndex1, int timeIndex2);
	virtual ~MPO_PeriodicWheelTrajectoriesObjective(void);

	virtual double computeValue(const dVector& p);

	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

private:
	//the energy function operates on a motion plan...
	LocomotionEngineMotionPlan* theMotionPlan;

	int timeIndex1, timeIndex2;
};
