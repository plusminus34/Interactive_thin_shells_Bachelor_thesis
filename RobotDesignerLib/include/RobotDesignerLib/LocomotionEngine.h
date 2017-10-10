#pragma once

#include <RobotDesignerLib/LocomotionEngineEnergyFunction.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>
#include <RobotDesignerLib/LocomotionEngineConstraints.h>
#include <OptimizationLib/ConstrainedObjectiveFunction.h>
#include <OptimizationLib/SQPFunctionMinimizer.h>
#include <OptimizationLib/SQPFunctionMinimizer_BFGS.h>
#include <OptimizationLib/NewtonFunctionMinimizer.h>
#include <OptimizationLib/BFGSFunctionMinimizer.h>


/**
	This is a locomotion engine that works for arbitrary robot types
*/
class LocomotionEngine{

public:
	LocomotionEngine(LocomotionEngineMotionPlan* motionPlan);
	virtual ~LocomotionEngine(void);

	double optimizePlan(int maxIterations = 1);
	double optimizePlan_BFGS();
public:
	LocomotionEngine_EnergyFunction* energyFunction;
	LocomotionEngine_Constraints* constraints;
	LocomotionEngineMotionPlan* motionPlan;

	ConstrainedObjectiveFunction* constrainedObjectiveFunction;

	SQPFunctionMinimizer_BFGS* BFGSSQPminimizer = NULL;
	BFGSFunctionMinimizer* BFGSminimizer = NULL;

	bool useObjectivesOnly = false;
};



