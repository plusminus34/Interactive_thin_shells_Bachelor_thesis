#pragma once

#include <RBSimLib/AbstractRBEngine.h>
#include <RBSimLib/WorldOracle.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>
#include <RobotDesignerLib/LocomotionEngine.h>
#include <RobotDesignerLib/FootFallPatternViewer.h>
#include <RobotDesignerLib/LocomotionEngineManagerGRF.h>
#include <RobotDesignerLib/LocomotionEngineManagerIP.h>

class RobotController {
public:
	RobotController(Robot *robot, LocomotionEngineMotionPlan *motionPlan);
	virtual ~RobotController(void);

	virtual void advanceInTime(double timeStep);
	virtual void computeControlSignals(double simTimeStep);
	virtual void applyControlSignals();
	virtual void computeDesiredState();
	virtual void drawDebugInfo();
	virtual void initialize();
	virtual void setDebugMode(bool doDebug) {}

public:
	double totalTime = 0;
	double stridePhase = 0;
	Robot *robot;
	LocomotionEngineMotionPlan *motionPlan;
	//this is the pose that the virtual agent is aiming to achieve
	RobotState desiredState;
};

