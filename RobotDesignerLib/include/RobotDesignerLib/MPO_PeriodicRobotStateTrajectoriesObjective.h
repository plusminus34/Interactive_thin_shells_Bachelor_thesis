#pragma once


#include <OptimizationLib/ObjectiveFunction.h>
#include <MathLib/Matrix.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>

class MPO_PeriodicRobotStateTrajectoriesObjective : public ObjectiveFunction {

public:
	MPO_PeriodicRobotStateTrajectoriesObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight, int timeIndex1, int timeIndex2, int startQIndex, int endQIndex);
	virtual ~MPO_PeriodicRobotStateTrajectoriesObjective(void);

	virtual double computeValue(const dVector& p);

	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

private:
	//the energy function operates on a motion plan...
	LocomotionEngineMotionPlan* theMotionPlan;

	//these are the start and end indices for the parts of the state space that we penalize
	int startQIndex, endQIndex;
	int timeIndex1, timeIndex2;
};

// enforce periodic boundary conditions on linear and angular part of the COM trajectory
class MPO_COMTrajectoryObjective : public ObjectiveFunction {

public:
	MPO_COMTrajectoryObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight, int timeIndex1, int timeIndex2);
	virtual ~MPO_COMTrajectoryObjective(void);

	virtual double computeValue(const dVector& p);

	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

private:
	//the energy function operates on a motion plan...
	LocomotionEngineMotionPlan* theMotionPlan;

	//these are the start and end indices that describe the boundary conditions...
	int timeIndex1, timeIndex2;
};

