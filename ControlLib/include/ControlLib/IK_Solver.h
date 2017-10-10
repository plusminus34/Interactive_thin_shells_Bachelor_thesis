#pragma once

#include <MathLib/MathLib.h>
#include <MathLib/P3D.h>
#include <MathLib/V3D.h>
#include <MathLib/mathLib.h>
#include <MathLib/Trajectory.h>
#include <ControlLib/Robot.h>
#include <ControlLib/GeneralizedCoordinatesRobotRepresentation.h>
#include <vector>
#include <ControlLib/IK_RobotStateRegularizer.h>
#include <ControlLib/IK_Optimizer.h>
#include <ControlLib/IK_Plan.h>



/**
	This plan is for one specific moment in time - given targets for end effectors, COM and full-body state (with various weights to mimic regularizers or hard constraints), we need to compute the robot's joint angles
*/
class IK_Solver{
public:
	IK_Solver(Robot* robot);
	IK_Solver(Robot* robot, bool freezeRootConfiguration);

	virtual ~IK_Solver(void);

public:
	//stores all objectives
	IK_Plan *ikPlan;
	//this is the energy function that operates on objectives stored in ik plan
	IK_EnergyFunction *ikEnergyFunction;
	//and the optimizer that minimizes the energy function
	IK_Optimizer *ikOptimizer;

	void solve(int nSteps = 10, bool resetTargetState = false);
};

