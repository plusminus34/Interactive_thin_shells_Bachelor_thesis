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
	bool initialized = false;
	Robot* robot = NULL;

	//these are the global goals for the longer horizon plan...
	double preplanTimeHorizon = 2;			//seconds
	double forwardSpeedTarget = 1.0;		//speed target for the longer horizon plan
	double sidewaysSpeedTarget = 0;			//speed target for the longer horizon plan
	double turningSpeedTarget = 0;		//turning speed target for the longer horizon plan
	double bodyHeightTarget = 0.5;			//body height target for the longer horizon plan
	double motionPlanStartTime = 0;			//the global time for the entire planning/control framework
	double swingFootHeight = 0.03;			//peak height of the swing foot

	FootFallPattern defaultFootFallPattern; //TODO: at some point we can also change the footfall pattern to make transitions, stand-to-walk-to-stand, etc...
	int nTimeStepsForSync = 0;				//whenever we replan, we will begin at some specific location, specified by this parameter, in the default footfall pattern

	//planned trajectories
	Trajectory3D prePlanBodyTrajectory;
	Trajectory3D prePlanBodyVelocityTrajectory;
	Trajectory1D prePlanHeadingTrajectory;
	Trajectory1D prePlanTurningSpeedTrajectory;
	DynamicArray<Trajectory3D> prePlanEETrajectories;
	ContinuousFootFallPattern cffp;

	RobotState plannerStartState;

	RobotState getPreplanedRobotStateAtTime(double t);
	RobotState getPreplanedRobotStateAtStridePhase(double p);

	RobotState getMOPTRobotStateAtStridePhase(double p);

	double globalMOPTRegularizer = 0.01;
	double motionPlanDuration = 0.8;
	FootFallPattern currentMOPTFootFallPattern;
	LocomotionEngineManager* locomotionManager = nullptr;

	void preplan(RobotState* currentRobotState);
	void prepareMOPTPlan(LocomotionEngineMotionPlan* motionPlan);

public:
	MotionPlanner();
	~MotionPlanner(void);

	LocomotionEngineManager* initializeMOPTEngine();

	void generateMotionPlan();

	void draw();
};

