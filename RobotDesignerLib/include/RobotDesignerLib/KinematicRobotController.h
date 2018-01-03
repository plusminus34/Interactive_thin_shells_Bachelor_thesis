#pragma once

#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>
#include <RobotDesignerLib/LocomotionEngine.h>
#include <RobotDesignerLib/FootFallPatternViewer.h>
#include <RobotDesignerLib/LocomotionEngineManagerGRF.h>
#include <RobotDesignerLib/LocomotionEngineManagerIP.h>
#include <RobotDesignerLib/RobotController.h>

class KinematicRobotController : public RobotController {
public:
	KinematicRobotController(Robot* robot, LocomotionEngineMotionPlan *motionPlan);
	~KinematicRobotController(void);

	virtual void advanceInTime(double timeStep);
	virtual void computeDesiredState();
	virtual void computeControlSignals(double simTimeStep);
	virtual void applyControlSignals();
	virtual void drawDebugInfo();
	virtual void initialize();

	virtual void loadMotionPlan(LocomotionEngineMotionPlan* motionPlan, double phase = 0);

	virtual void draw();

public:
	V3D posInPlane;
	Quaternion overallHeading;
};

