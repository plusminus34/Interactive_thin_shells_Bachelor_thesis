#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <MathLib/Matrix.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>

class MPO_EEPosSwingObjective : public ObjectiveFunction {

public:
	MPO_EEPosSwingObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight);
	virtual ~MPO_EEPosSwingObjective(void);

	virtual double computeValue(const dVector& p);

	virtual void addGradientTo(dVector& grad, const dVector& p);
	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);

private:
	template<class T>
	static T computeEnergy(const T &eePosY, const T &targetEEPosY, const T &c);

private:
	//the energy function operates on a motion plan...
	LocomotionEngineMotionPlan* theMotionPlan;
};

