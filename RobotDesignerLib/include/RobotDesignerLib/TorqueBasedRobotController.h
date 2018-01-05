#pragma once

#include <ControlLib/Robot.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>
#include <RobotDesignerLib/MOPTQPTrackingController.h>
#include <RobotDesignerLib/RobotController.h>

class TorqueBasedRobotController : public RobotController {
public:
	//it is assumed that the motion plan is already optimized, so we just query it...
	TorqueBasedRobotController(Robot *robot, LocomotionEngineMotionPlan *motionPlan);
	virtual ~TorqueBasedRobotController(void);

	//timeStep indicates the time that will take until the next call to this function, esentially how long do we expect the current control signals to be constant for...
	virtual void applyControlSignals(double timeStep);

	//compute the control signals based on the current stride phase. timeStep indicates the planning horizon...
	virtual void computeControlSignals(double timeStep);

	virtual void drawDebugInfo();
	virtual void initialize();
	virtual void setDebugMode(bool doDebug) {
		controller->doDebug = doDebug;
	}

	MOPTQPTrackingController* controller = NULL;

public:

};

