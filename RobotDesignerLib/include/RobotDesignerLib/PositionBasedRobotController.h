#pragma once

#include <ControlLib/Robot.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>
#include <RobotDesignerLib/RobotController.h>

class PositionBasedRobotController : public RobotController{
public:
	//it is assumed that the motion plan is already optimized, so we just query it...
	PositionBasedRobotController(Robot *robot, LocomotionEngineMotionPlan *motionPlan);
	virtual ~PositionBasedRobotController(void);

	virtual void computeControlSignals(double simTimeStep);
	virtual void applyControlSignals();
	virtual void drawDebugInfo();
	virtual void initialize();

public:

};
