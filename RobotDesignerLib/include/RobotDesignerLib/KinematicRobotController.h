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

	//advance the phase of the motion by timeStep...
	virtual bool advanceInTime(double timeStep);

	//timeStep indicates the time that will take until the next call to this function, esentially how long do we expect the current control signals to be constant for...
	virtual void applyControlSignals(double timeStep);

	//compute the control signals based on the current stride phase. timeStep indicates the planning horizon...
	virtual void computeControlSignals(double timeStep);

	//compute the desired state of the robot at the current stride phase
	virtual void computeDesiredState();


	virtual void drawDebugInfo();
	virtual void initialize();

	virtual void loadMotionPlan(LocomotionEngineMotionPlan* motionPlan, double phase = 0);

	virtual void draw();



public:
	V3D posInPlane;
	Quaternion overallHeading;
};

