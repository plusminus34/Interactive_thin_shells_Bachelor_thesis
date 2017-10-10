#pragma once

#include <OptimizationLib/ObjectiveFunction.h>
#include <OptimizationLib/FunctionConstraints.h>
#include <MathLib/Matrix.h>
#include <ControlLib/QPControlEngine.h>

class MOPTQPTrackingController {
private:
	virtual void prepareForControlStep(LocomotionEngineMotionPlan *motionPlan, double stridePhase, double dt);

public:
	MOPTQPTrackingController(Robot* robot);
	~MOPTQPTrackingController(void);

	virtual void computeControlSignals(LocomotionEngineMotionPlan *motionPlan, double stridePhase, double dt);

	//this is the target robot state, copied directly from the motion plan...
	virtual ReducedRobotState getTargetRobotState(LocomotionEngineMotionPlan *motionPlan, double stridePhase);

	//this target mopt state needs to be adapted based on the current configuration of the robot... this method computes the quantities required to perform this retargetting operation...
	virtual void computeRobotStateTransferQuantities(LocomotionEngineMotionPlan *motionPlan, double stridePhase, double &moptGroundHeight, double &currentGroundHeight, Quaternion& headingOffset, P3D& moptEEFrameOrigin, P3D&robotEEFrameOrigin);

	//computes a stepping offset based on tracking error, in world coordinates...
	virtual V3D computeStepOffset(double currentHeight, const V3D& currentVelocity, const V3D& plannedVelocity);

	//based on the motion plan, estimate the target COM velocity...
	V3D estimateMOPTCOMVelocity(LocomotionEngineMotionPlan *motionPlan, double stridePhase);

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

