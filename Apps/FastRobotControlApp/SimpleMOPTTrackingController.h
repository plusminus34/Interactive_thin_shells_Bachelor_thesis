#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <OptimizationLib/FunctionConstraints.h>
#include <MathLib/Matrix.h>
#include <ControlLib/QPControlEngine.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>

class SimpleMOPTTrackingController {
private:
	virtual void prepareForControlStep(LocomotionEngineMotionPlan *motionPlan, double stridePhase, double dt);

public:
	SimpleMOPTTrackingController(Robot* robot);
	~SimpleMOPTTrackingController(void);

	virtual void computeControlSignals(LocomotionEngineMotionPlan *motionPlan, double stridePhase, double dt);

	//based on the motion plan, estimate the velocity for the ith EE...
	V3D estimateMOPTEEVelocity(LocomotionEngineMotionPlan *motionPlan, double stridePhase, int EEIndex);

	//draws debugging info...
	virtual void draw();

public:
	QPControlEngine* qpEngine = NULL;
	QPControlPlan* qpPlan = NULL;

	Robot *robot;

	bool doDebug = true;

	double gainBodyPos = 1000;
	double gainBodyVel = 1000;
	double gainBodyOrientation = 1000;
	double gainJointAngles = 1.5;
	double gainSwingFoot = 1000;
};

