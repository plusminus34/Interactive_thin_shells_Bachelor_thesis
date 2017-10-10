#pragma once

#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>
#include <RobotDesignerLib/LocomotionEngine.h>
#include <RobotDesignerLib/FootFallPatternViewer.h>

#define OPT_END_EFFECTORS 0x0001
#define OPT_COM_POSITIONS 0x0002
#define OPT_COM_ORIENTATIONS 0x0004
#define OPT_ROBOT_STATES 0x0008
#define OPT_GRFS 0x0010
#define OPT_BARYCENTRIC_WEIGHTS 0x0020

class LocomotionEngineManager{
public:

	bool useBFGS = false;
	bool printDebugInfo = true;
	bool checkDerivatives = false;
	bool locked = false; // once locked, cannot do further optimization

	LocomotionEngineMotionPlan *motionPlan = NULL;
	LocomotionEngine *locomotionEngine = NULL;
	FootFallPattern* footFallPattern = NULL;
	FootFallPattern origFootFallPattern;

public:
	LocomotionEngineManager();
	virtual ~LocomotionEngineManager() = 0;

	virtual void warmStartMOpt() = 0;
	virtual void warmStartMOptGRF();
	virtual double runMOPTStep();

	double runMOPTStep(int optimizationFlags) {
		motionPlan->optimizeEndEffectorPositions = (optimizationFlags & OPT_END_EFFECTORS) != 0;
		motionPlan->optimizeCOMPositions = (optimizationFlags & OPT_COM_POSITIONS) != 0;
		motionPlan->optimizeCOMOrientations = (optimizationFlags & OPT_COM_ORIENTATIONS) != 0;
		motionPlan->optimizeRobotStates = (optimizationFlags & OPT_ROBOT_STATES) != 0;
		motionPlan->optimizeContactForces = (optimizationFlags & OPT_GRFS) != 0;
		motionPlan->optimizeBarycentricWeights = (optimizationFlags & OPT_BARYCENTRIC_WEIGHTS) != 0;
		return runMOPTStep();
	}

	virtual void drawMotionPlan(double f, int animationCycle = 0, bool drawRobot = true, bool drawSkeleton = false, bool drawPlanDetails = false, bool drawContactForces = false, bool drawOrientation = false);

};

