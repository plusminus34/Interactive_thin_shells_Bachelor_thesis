#pragma once

#include <Utils/Utils.h>
#include <ControlLib/GenericLimb.h>
#include <RobotDesignerLib/FastMOPTWindow.h>

/**
	Sync'ed with the robot state, this class implements a very simple first 
	order model to generate body trajectories and footfall placement.
*/
class FastMOPTPreplanner{
public:
	FastMOPTWindow* moptWindow;
	Trajectory3D bodyTrajectory;
	Trajectory3D bodyVelocityTrajectory;
	Trajectory1D headingTrajectory;
	Trajectory1D turningSpeedTrajectory;
	ContinuousFootFallPattern cffp;

	V3D initialLinearVelocity, initialAngularVelocity;
	Quaternion initialOrientation;

	DynamicArray<Trajectory3D> eeTrajectories;

	RobotState startState = RobotState(13);

	RobotState getRobotStateAtTime(double t);

public:
	FastMOPTPreplanner(FastMOPTWindow* moptWindow);
	~FastMOPTPreplanner(void);

	void preplan(RobotState* currentRobotState);

	void prepareMOPTPlan(LocomotionEngineMotionPlan* motionPlan);

	void draw();

};

