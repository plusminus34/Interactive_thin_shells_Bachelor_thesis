#pragma once

#include <Utils/Utils.h>
#include <ControlLib/GenericLimb.h>
#include <RobotDesignerLib/LocomotionEngineManagerGRF.h>
#include "MotionPlannerWindow.h"

/**
	Sync'ed with the robot state, this class implements a very simple first 
	order model to generate body trajectories and footfall placement.
*/
class MotionPlanner{
public:
	Robot* robot = NULL;

	//these are the global goals for the longer horizon plan...
	double preplanTimeHorizon = 5;		//seconds
	double forwardSpeedTarget = 1.0;	//speed target for the longer horizon plan
	double sidewaysSpeedTarget = 0;		//speed target for the longer horizon plan
	double turningSpeedTarget = 0;		//turning speed target for the longer horizon plan
	double bodyHeightTarget = 0.5;		//body height target for the longer horizon plan
	double motionPlanStartTime = 0;		//the global time for the entire planning/control framework

	FootFallPattern defaultFootFallPattern; //TODO: at some point we can also change the footfall pattern to make transitions, stand-to-walk-to-stand, etc...

	//planned trajectories
	Trajectory3D prePlanBodyTrajectory;
	Trajectory3D prePlanBodyVelocityTrajectory;
	Trajectory1D prePlanHeadingTrajectory;
	Trajectory1D prePlanTurningSpeedTrajectory;
	DynamicArray<Trajectory3D> prePlanEETrajectories;
	ContinuousFootFallPattern cffp;

	RobotState plannerStartState;

	RobotState getPreplanedRobotStateAtTime(double t);

	double globalMOPTRegularizer = 0.01;
	MOPTParams moptParams;
	FootFallPattern moptFootFallPattern;
	LocomotionEngineManager* locomotionManager = nullptr;

	void preplan(RobotState* currentRobotState);
	void prepareMOPTPlan(LocomotionEngineMotionPlan* motionPlan);

public:
	MotionPlanner();
	~MotionPlanner(void);

	LocomotionEngineManager* initializeMOPTEngine();

	void generateMotionPlan();

	void advanceMotionPlanGlobalTime(int nSteps);

	void draw();

};

