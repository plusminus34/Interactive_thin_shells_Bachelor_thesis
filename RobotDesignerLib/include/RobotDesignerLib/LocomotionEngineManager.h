#pragma once

#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>
#include <RobotDesignerLib/LocomotionEngine.h>
#include <RobotDesignerLib/FootFallPatternViewer.h>

#include <RobotDesignerLib/MPO_PeriodicRobotStateTrajectoriesObjective.h>
#include <RobotDesignerLib/MPO_SmoothCOMTrajectories.h>
#include <RobotDesignerLib/MPO_DynamicStabilityObjective.h>
#include <RobotDesignerLib/MPO_SmoothStanceLegMotionObjective.h>
#include <RobotDesignerLib/MPO_StanceLegMotionRegularizer.h>
#include <RobotDesignerLib/MPO_COMTravelObjective.h>
#include <RobotDesignerLib/MPO_FeetSlidingObjective.h>
#include <RobotDesignerLib/MPO_EndEffectorGroundObjective.h>
#include <RobotDesignerLib/MPO_FeetPathSmoothnessObjective.h>
#include <RobotDesignerLib/MPO_BarycentricWeightsRegularizerObjective.h>
#include <RobotDesignerLib/MPO_RobotStateRegularizer.h>
#include <RobotDesignerLib/MPO_RobotCOMObjective.h>
#include <RobotDesignerLib/MPO_RobotEndEffectorsObjective.h>
#include <RobotDesignerLib/MPO_PeriodicRobotStateTrajectoriesObjective.h>
#include <RobotDesignerLib/MPO_RobotTurningObjective.h>
#include <RobotDesignerLib/MPO_SmoothRobotMotionTrajectories.h>
#include <RobotDesignerLib/MPO_ForceAccelObjective.h>
#include <RobotDesignerLib/MPO_TorqueAngularAccelObjective.h>
#include <RobotDesignerLib/MPO_RobotCOMOrientationsObjective.h>
#include <RobotDesignerLib/MPO_COMTurningObjective.h>
#include <RobotDesignerLib/MPO_GRFSoftConstraints.h>
#include <RobotDesignerLib/MPO_NonLimbMotionRegularizer.h>
#include <RobotDesignerLib/MPO_NonLimbSmoothMotionObjective.h>
#include <RobotDesignerLib/MPO_RobotStateTransitionRegularizer.h>
#include <RobotDesignerLib/MPO_PseudoLimbLengthConstraint.h>
#include <RobotDesignerLib/MPO_PseudoPeriodicEECOMPoseConstraint.h>
#include <RobotDesignerLib/MPO_EEPoseOffsetConstraintToInitial.h>
#include <RobotDesignerLib/MPO_COMOrientationFluctuationRegularizer.h>

#include <RobotDesignerLib/LocomotionEngineEnergyFunction.h>
#include <RobotDesignerLib/LocomotionEngineConstraints.h>
#include <OptimizationLib/ConstrainedObjectiveFunction.h>
#include <OptimizationLib/SQPFunctionMinimizer.h>
#include <OptimizationLib/NewtonFunctionMinimizer.h>

#define OPT_END_EFFECTORS 0x0001
#define OPT_COM_POSITIONS 0x0002
#define OPT_COM_ORIENTATIONS 0x0004
#define OPT_WHEELS 0x0040
#define OPT_ROBOT_STATES 0x0008
#define OPT_GRFS 0x0010
#define OPT_BARYCENTRIC_WEIGHTS 0x0020

class LocomotionEngineManager{
protected:
	void createSolverComponents();
	double optimizeMoptionPlan(int maxIterations = 1);

public:

	bool useBFGS = false;
	bool printDebugInfo = true;
	bool checkDerivatives = false;
	bool locked = false; // once locked, cannot do further optimization
	bool writeVelocityProfileToFile = false;

	LocomotionEngineMotionPlan *motionPlan = NULL;
	FootFallPattern* footFallPattern = NULL;

	LocomotionEngine_EnergyFunction* energyFunction;
	LocomotionEngine_Constraints* constraints;
	ConstrainedObjectiveFunction* constrainedObjectiveFunction;
	bool useObjectivesOnly = false;
	bool writeParamsToFile = true;
	NewtonFunctionMinimizer::HessCorrectionMethod hessCorrectionMethod;
public:
	LocomotionEngineManager();
	virtual ~LocomotionEngineManager() = 0;

	virtual void warmStartMOpt() = 0;
	virtual void setupObjectives() = 0;

	virtual double runMOPTStep();

	virtual void setDefaultOptimizationFlags() = 0;

	double runMOPTStep(int optimizationFlags) {
		motionPlan->optimizeEndEffectorPositions = (optimizationFlags & OPT_END_EFFECTORS) != 0;
		motionPlan->optimizeWheels = (optimizationFlags & OPT_WHEELS) != 0;
		motionPlan->optimizeCOMPositions = (optimizationFlags & OPT_COM_POSITIONS) != 0;
		motionPlan->optimizeCOMOrientations = (optimizationFlags & OPT_COM_ORIENTATIONS) != 0;
		motionPlan->optimizeRobotStates = (optimizationFlags & OPT_ROBOT_STATES) != 0;
		motionPlan->optimizeContactForces = (optimizationFlags & OPT_GRFS) != 0;
		motionPlan->optimizeBarycentricWeights = (optimizationFlags & OPT_BARYCENTRIC_WEIGHTS) != 0;
		return runMOPTStep();
	}

	virtual void drawMotionPlan(double f, int animationCycle = 0, bool drawRobot = true, bool drawSkeleton = false, bool drawPlanDetails = false, bool drawContactForces = false, bool drawOrientation = false);
};

