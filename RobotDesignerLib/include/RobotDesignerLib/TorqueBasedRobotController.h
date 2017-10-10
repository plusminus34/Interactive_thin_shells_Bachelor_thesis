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

	virtual void computeControlSignals(double simTimeStep);
	virtual void applyControlSignals();
	virtual void drawDebugInfo();
	virtual void initialize();
	virtual void setDebugMode(bool doDebug) {
		controller->doDebug = doDebug;
	}

	MOPTQPTrackingController* controller = NULL;

public:

};

