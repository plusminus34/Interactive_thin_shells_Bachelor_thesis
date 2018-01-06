#pragma once

#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>
#include <RobotDesignerLib/LocomotionEngine.h>
#include <RobotDesignerLib/FootFallPatternViewer.h>
#include <RobotDesignerLib/LocomotionEngineManagerGRF.h>
#include <RobotDesignerLib/LocomotionEngineManagerIP.h>
#include <RobotDesignerLib/KinematicRobotController.h>
#include <ControlLib/PololuServoControlInterface.h>

class PololuMaestroRobotController : public KinematicRobotController {
public:
	PololuMaestroRobotController(Robot* robot, LocomotionEngineMotionPlan *motionPlan);
	~PololuMaestroRobotController(void);

	//timeStep indicates the time that will take until the next call to this function, esentially how long do we expect the current control signals to be constant for...
	virtual void applyControlSignals(double timeStep);

	//compute the control signals based on the current stride phase. timeStep indicates the planning horizon...
	virtual void computeControlSignals(double timeStep);

	virtual void drawDebugInfo();
	virtual void initialize();

	virtual void loadMotionPlan(LocomotionEngineMotionPlan* motionPlan, double phase = 0);

	virtual void draw();

	PololuServoControlInterface* rci = NULL;

public:
	V3D posInPlane;
	Quaternion overallHeading;

	double timeStep = 0;
};

