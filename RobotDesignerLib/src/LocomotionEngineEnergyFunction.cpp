#include <RobotDesignerLib/LocomotionEngineEnergyFunction.h>

#include <RobotDesignerLib/MPO_SmoothCOMTrajectories.h>
#include <RobotDesignerLib/MPO_DynamicStabilityObjective.h>
#include <RobotDesignerLib/MPO_SmoothStanceLegMotionObjective.h>
#include <RobotDesignerLib/MPO_StanceLegMotionRegularizer.h>
#include <RobotDesignerLib/MPO_COMTravelObjective.h>
#include <RobotDesignerLib/MPO_FeetSlidingObjective.h>
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

//TODO: should change IP - based mopt to use additional dofs of body rotation, and then have
//the objective that matches those to robot state, just the way com dofs are matched to robot's com...


LocomotionEngine_EnergyFunction::LocomotionEngine_EnergyFunction(LocomotionEngineMotionPlan* mp){
	theMotionPlan = mp;
	regularizer = 1;

	printDebugInfo = true;
}

LocomotionEngine_EnergyFunction::~LocomotionEngine_EnergyFunction(void){
	for (uint i=0;i<objectives.size();i++)
		delete objectives[i];
}

void LocomotionEngine_EnergyFunction::setupIPSubObjectives(){
	for (uint i=0;i<objectives.size();i++)
		delete objectives[i];
	objectives.clear();

	//soft constraints
	objectives.push_back(new MPO_DynamicStabilityObjective(theMotionPlan, "dynamic stability objective", 10000.0));
	objectives.push_back(new MPO_RobotCOMObjective(theMotionPlan, "robot COM objective", 10000.0));
	objectives.push_back(new MPO_RobotEndEffectorsObjective(theMotionPlan, "robot EE objective", 10000.0));

	//soft constraints that should be made into hard constraints, since it's super easy to do
//	objectives.push_back(new MPO_FeetSlidingObjective(theMotionPlan, "feet sliding objective", 10000.0));
//	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) {
//		objectives.push_back(new MPO_PeriodicRobotStateTrajectoriesObjective(theMotionPlan, "periodic joint angles", 10000.0, theMotionPlan->nSamplePoints - 1, theMotionPlan->wrapAroundBoundaryIndex, 6, theMotionPlan->robotRepresentation->getDimensionCount() - 1));
//		objectives.push_back(new MPO_PeriodicRobotStateTrajectoriesObjective(theMotionPlan, "periodic body orientations (ROLL)", 10000.0, theMotionPlan->nSamplePoints - 1, theMotionPlan->wrapAroundBoundaryIndex, 4, 4));
//		objectives.push_back(new MPO_PeriodicRobotStateTrajectoriesObjective(theMotionPlan, "periodic body orientations (PITCH)", 10000.0, theMotionPlan->nSamplePoints - 1, theMotionPlan->wrapAroundBoundaryIndex, 5, 5));
//	}

	//functional objectives
	objectives.push_back(new MPO_COMTravelObjective(theMotionPlan, "COM Travel objective", 50.0));
	objectives.push_back(new MPO_RobotTurningObjective(theMotionPlan, "robot turning rate (YAW)", 50.0));

	//motion regularizers
	objectives.push_back(new MPO_SmoothCOMTrajectories(theMotionPlan, "smoothCOM", 10));
	objectives.push_back(new MPO_FeetPathSmoothnessObjective(theMotionPlan, "foot path smoothness objective", 1.0));
	objectives.push_back(new MPO_BarycentricWeightsRegularizerObjective(theMotionPlan, "barycentric weights regularizer objective", 0.001 * 0.001));

	objectives.push_back(new MPO_SmoothStanceLegMotionObjective(theMotionPlan, "robot stance leg smooth joint angle trajectories", 0.01));
	objectives.push_back(new MPO_StanceLegMotionRegularizer(theMotionPlan, "robot stance legs motion regularizer", 0.01));
	objectives.push_back(new MPO_RobotStateRegularizer(theMotionPlan, "robot joint angles regularizer objective", 0.0010 * 10,  6, theMotionPlan->robotRepresentation->getDimensionCount() - 1));
	objectives.push_back(new MPO_RobotStateRegularizer(theMotionPlan, "robot body orientation regularizer objective (ROLL)", 1, 5, 5));
	objectives.push_back(new MPO_RobotStateRegularizer(theMotionPlan, "robot body orientation regularizer objective (PITCH)", 1, 4, 4));
	objectives.push_back(new MPO_RobotStateRegularizer(theMotionPlan, "robot body orientation regularizer objective (YAW)", 1, 3, 3));
	objectives.push_back(new MPO_SmoothRobotMotionTrajectories(theMotionPlan, "robot smooth joint angle trajectories", 0.001, 6, theMotionPlan->robotRepresentation->getDimensionCount() - 1));
	objectives.push_back(new MPO_SmoothRobotMotionTrajectories(theMotionPlan, "robot smooth body orientation trajectories", 1, 3, 5));
}

void LocomotionEngine_EnergyFunction::setupIPv2SubObjectives() {
	for (uint i = 0; i<objectives.size(); i++)
		delete objectives[i];
	objectives.clear();

	//soft constraints
	objectives.push_back(new MPO_DynamicStabilityObjective(theMotionPlan, "dynamic stability objective", 10000.0));
	objectives.push_back(new MPO_RobotCOMObjective(theMotionPlan, "robot COM objective", 10000.0));
	objectives.push_back(new MPO_RobotEndEffectorsObjective(theMotionPlan, "robot EE objective", 10000.0));
	objectives.push_back(new MPO_RobotCOMOrientationsObjective(theMotionPlan, "robot COM orientations objective", 10000.0));

	//functional objectives
	objectives.push_back(new MPO_COMTravelObjective(theMotionPlan, "COM Travel objective", 50.0));
	objectives.push_back(new MPO_COMTurningObjective(theMotionPlan, "robot turning rate (YAW)", 50.0));

	//motion regularizers
	objectives.push_back(new MPO_SmoothCOMTrajectories(theMotionPlan, "smoothCOM", 10));
	objectives.push_back(new MPO_FeetPathSmoothnessObjective(theMotionPlan, "foot path smoothness objective", 1.0));
	objectives.push_back(new MPO_BarycentricWeightsRegularizerObjective(theMotionPlan, "barycentric weights regularizer objective", 0.001 * 0.001));

	objectives.push_back(new MPO_SmoothStanceLegMotionObjective(theMotionPlan, "robot stance leg smooth joint angle trajectories", 0.01));
	objectives.push_back(new MPO_StanceLegMotionRegularizer(theMotionPlan, "robot stance legs motion regularizer", 0.01));
	objectives.push_back(new MPO_RobotStateRegularizer(theMotionPlan, "robot joint angles regularizer objective", 0.0010 * 10, 6, theMotionPlan->robotRepresentation->getDimensionCount() - 1));
	objectives.push_back(new MPO_RobotStateRegularizer(theMotionPlan, "robot body orientation regularizer objective (ROLL)", 1, 5, 5));
	objectives.push_back(new MPO_RobotStateRegularizer(theMotionPlan, "robot body orientation regularizer objective (PITCH)", 1, 4, 4));
	objectives.push_back(new MPO_RobotStateRegularizer(theMotionPlan, "robot body orientation regularizer objective (YAW)", 1, 3, 3));
	objectives.push_back(new MPO_SmoothRobotMotionTrajectories(theMotionPlan, "robot smooth joint angle trajectories", 0.001, 6, theMotionPlan->robotRepresentation->getDimensionCount() - 1));
	objectives.push_back(new MPO_SmoothRobotMotionTrajectories(theMotionPlan, "robot smooth body orientation trajectories", 1, 3, 5));
}


void LocomotionEngine_EnergyFunction::setupGRFSubObjectives() {
	for (uint i = 0; i < objectives.size(); i++)
		delete objectives[i];
	objectives.clear();

	//soft constraints
	objectives.push_back(new MPO_ForceAccelObjective(theMotionPlan, "force acceleration objective", 10000.0));
	objectives.push_back(new MPO_TorqueAngularAccelObjective(theMotionPlan, "torque angular acceleration objective", 10000.0));
	objectives.push_back(new MPO_RobotCOMObjective(theMotionPlan, "robot COM objective", 10000.0));
	objectives.push_back(new MPO_RobotEndEffectorsObjective(theMotionPlan, "robot EE objective", 10000.0));
	objectives.push_back(new MPO_RobotCOMOrientationsObjective(theMotionPlan, "robot COM orientations objective", 10000.0));

	//soft constraints that should be made into hard constraints, since it's super easy to do
	//	objectives.push_back(new MPO_FeetSlidingObjective(theMotionPlan, "feet sliding objective", 10000.0));
	//	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) {
	//		objectives.push_back(new MPO_PeriodicRobotStateTrajectoriesObjective(theMotionPlan, "periodic joint angles", 10000.0, theMotionPlan->nSamplePoints - 1, theMotionPlan->wrapAroundBoundaryIndex, 6, theMotionPlan->robotRepresentation->getDimensionCount() - 1));
	//		objectives.push_back(new MPO_PeriodicRobotStateTrajectoriesObjective(theMotionPlan, "periodic body orientations (ROLL)", 10000.0, theMotionPlan->nSamplePoints - 1, theMotionPlan->wrapAroundBoundaryIndex, 4, 4));
	//		objectives.push_back(new MPO_PeriodicRobotStateTrajectoriesObjective(theMotionPlan, "periodic body orientations (PITCH)", 10000.0, theMotionPlan->nSamplePoints - 1, theMotionPlan->wrapAroundBoundaryIndex, 5, 5));
	//	}

	//functional objectives
	objectives.push_back(new MPO_COMTravelObjective(theMotionPlan, "COM Travel objective", 50.0));
	objectives.push_back(new MPO_COMTurningObjective(theMotionPlan, "robot turning rate (YAW)", 50.0));

	//motion regularizers
	objectives.push_back(new MPO_SmoothCOMTrajectories(theMotionPlan, "smoothCOM", 10));
	objectives.push_back(new MPO_SmoothStanceLegMotionObjective(theMotionPlan, "robot stance leg smooth joint angle trajectories", 0.01));
	objectives.push_back(new MPO_StanceLegMotionRegularizer(theMotionPlan, "robot stance legs motion regularizer", 0.01));
	objectives.push_back(new MPO_FeetPathSmoothnessObjective(theMotionPlan, "foot path smoothness objective", 1.0));
	//objectives.push_back(new MPO_BarycentricWeightsRegularizerObjective(theMotionPlan, "barycentric weights regularizer objective", 0.001 * 0.001));

	objectives.push_back(new MPO_RobotStateRegularizer(theMotionPlan, "robot joint angles regularizer objective", 0.0010 * 10, 6, theMotionPlan->robotRepresentation->getDimensionCount() - 1));
	objectives.push_back(new MPO_RobotStateRegularizer(theMotionPlan, "robot body orientation regularizer objective (ROLL)", 1, 5, 5));
	objectives.push_back(new MPO_RobotStateRegularizer(theMotionPlan, "robot body orientation regularizer objective (PITCH)", 1, 4, 4));
	objectives.push_back(new MPO_RobotStateRegularizer(theMotionPlan, "robot body orientation regularizer objective (YAW)", 1, 3, 3));
	objectives.push_back(new MPO_SmoothRobotMotionTrajectories(theMotionPlan, "robot smooth joint angle trajectories", 0.001, 6, theMotionPlan->robotRepresentation->getDimensionCount() - 1));
	objectives.push_back(new MPO_SmoothRobotMotionTrajectories(theMotionPlan, "robot smooth body orientation trajectories", 1, 3, 5));
}

void LocomotionEngine_EnergyFunction::setupGRFv2SubObjectives() {
	for (uint i = 0; i < objectives.size(); i++)
		delete objectives[i];
	objectives.clear();

	//soft constraints. NOTE: important for warmstaring that the order of the constraints does not change!

	//GRF constraints
	objectives.push_back(new MPO_GRFRegularizer(theMotionPlan, "GRF regularizers", 10000.0));
	objectives.push_back(new MPO_GRFSoftBoundConstraints(theMotionPlan, "GRF bound constraints", 10000.0));
	
	//consistancy constraints (between robot states and other auxiliary variables)
	objectives.push_back(new MPO_RobotEndEffectorsObjective(theMotionPlan, "robot EE objective", 10000.0));
	objectives.push_back(new MPO_RobotCOMObjective(theMotionPlan, "robot COM objective", 10000.0));
	objectives.push_back(new MPO_RobotCOMOrientationsObjective(theMotionPlan, "robot COM orientations objective", 10000.0));

	//dynamics constraints
	objectives.push_back(new MPO_ForceAccelObjective(theMotionPlan, "force acceleration objective", 10000.0));
	objectives.push_back(new MPO_TorqueAngularAccelObjective(theMotionPlan, "torque angular acceleration objective", 10000.0));

	//constraints ensuring feet don't slide...
	objectives.push_back(new MPO_FeetSlidingObjective(theMotionPlan, "feet sliding objective", 10000.0));

	//periodic boundary constraints...
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) {
		objectives.push_back(new MPO_PeriodicRobotStateTrajectoriesObjective(theMotionPlan, "periodic joint angles", 10000.0, theMotionPlan->nSamplePoints - 1, theMotionPlan->wrapAroundBoundaryIndex, 6, theMotionPlan->robotRepresentation->getDimensionCount() - 1));
		objectives.push_back(new MPO_PeriodicRobotStateTrajectoriesObjective(theMotionPlan, "periodic body orientations (ROLL)", 10000.0, theMotionPlan->nSamplePoints - 1, theMotionPlan->wrapAroundBoundaryIndex, 4, 4));
		objectives.push_back(new MPO_PeriodicRobotStateTrajectoriesObjective(theMotionPlan, "periodic body orientations (PITCH)", 10000.0, theMotionPlan->nSamplePoints - 1, theMotionPlan->wrapAroundBoundaryIndex, 5, 5));
	}

	//functional objectives
	objectives.push_back(new MPO_COMTravelObjective(theMotionPlan, "COM Travel objective", 50.0));
	objectives.push_back(new MPO_COMTurningObjective(theMotionPlan, "COM turning objective (YAW)", 50.0));

	//motion regularizers
	objectives.push_back(new MPO_SmoothStanceLegMotionObjective(theMotionPlan, "robot stance leg smooth joint angle trajectories", 0.01));
	objectives.push_back(new MPO_StanceLegMotionRegularizer(theMotionPlan, "robot stance legs motion regularizer", 0.01));
	objectives.push_back(new MPO_FeetPathSmoothnessObjective(theMotionPlan, "foot path smoothness objective", 10.0));

	objectives.push_back(new MPO_RobotStateRegularizer(theMotionPlan, "robot joint angles regularizer objective", 0.0010 * 1, 6, theMotionPlan->robotRepresentation->getDimensionCount() - 1));
	objectives.push_back(new MPO_NonLimbMotionRegularizer(theMotionPlan, "robot joint angles regularizer objective (non-limb)", 0.01));

	objectives.push_back(new MPO_RobotStateRegularizer(theMotionPlan, "robot body orientation regularizer objective (ROLL)", 1, 5, 5));
	objectives.push_back(new MPO_RobotStateRegularizer(theMotionPlan, "robot body orientation regularizer objective (PITCH)", 1, 4, 4));
	objectives.push_back(new MPO_RobotStateRegularizer(theMotionPlan, "robot body orientation regularizer objective (YAW)", 1, 3, 3));
	objectives.push_back(new MPO_SmoothRobotMotionTrajectories(theMotionPlan, "robot smooth joint angle trajectories", 0.01, 6, theMotionPlan->robotRepresentation->getDimensionCount() - 1));
	objectives.push_back(new MPO_SmoothRobotMotionTrajectories(theMotionPlan, "robot smooth body orientation trajectories", 1, 3, 5));
	objectives.push_back(new MPO_NonLimbSmoothMotionObjective(theMotionPlan, "robot smooth joint angles objective (non-limb)", 0.01));
	
	objectives.push_back(new MPO_SmoothCOMTrajectories(theMotionPlan, "smoothCOM", 50));


}

void LocomotionEngine_EnergyFunction::setupGRFv3SubObjectives() {
	for (uint i = 0; i < objectives.size(); i++)
		delete objectives[i];
	objectives.clear();

	//PseudoLimbLengthConstraint
	objectives.push_back(new MPO_PseudoLimbLengthConstraint(theMotionPlan, "Constrain dist(ee COM) <limb", 10000.0));
	objectives.push_back(new MPO_PseudoPeriodicEECOMPoseConstraint(theMotionPlan, "relative pose is the same for periodic", 50.0));
	objectives.push_back(new MPO_EEPoseOffsetConstraintToInitial(theMotionPlan, "EEToCOMOffset ~ Initial", 10.0));
	objectives.push_back(new MPO_COMOrientationFluctuationRegularizer(theMotionPlan, "Orientation Fluctuation", 10.0));

	//soft constraints. NOTE: important for warmstaring that the order of the constraints does not change!

	//GRF constraints
	objectives.push_back(new MPO_GRFRegularizer(theMotionPlan, "GRF regularizers", 10000.0));
	objectives.push_back(new MPO_GRFSoftBoundConstraints(theMotionPlan, "GRF bound constraints", 10000.0));

	//consistancy constraints (between robot states and other auxiliary variables)
	/*objectives.push_back(new MPO_RobotEndEffectorsObjective(theMotionPlan, "robot EE objective", 10000.0));
	objectives.push_back(new MPO_RobotCOMObjective(theMotionPlan, "robot COM objective", 10000.0));
	objectives.push_back(new MPO_RobotCOMOrientationsObjective(theMotionPlan, "robot COM orientations objective", 10000.0));*/

	//dynamics constraints
	objectives.push_back(new MPO_ForceAccelObjective(theMotionPlan, "force acceleration objective", 10000.0));
	objectives.push_back(new MPO_TorqueAngularAccelObjective(theMotionPlan, "torque angular acceleration objective", 10000.0));

	//constraints ensuring feet don't slide...
	objectives.push_back(new MPO_FeetSlidingObjective(theMotionPlan, "feet sliding objective", 10000.0));

	////periodic boundary constraints...
	/*if (theMotionPlan->wrapAroundBoundaryIndex >= 0) {
	objectives.push_back(new MPO_PeriodicRobotStateTrajectoriesObjective(theMotionPlan, "periodic joint angles", 10000.0, theMotionPlan->nSamplePoints - 1, theMotionPlan->wrapAroundBoundaryIndex, 6, theMotionPlan->robotRepresentation->getDimensionCount() - 1));
	objectives.push_back(new MPO_PeriodicRobotStateTrajectoriesObjective(theMotionPlan, "periodic body orientations (ROLL)", 10000.0, theMotionPlan->nSamplePoints - 1, theMotionPlan->wrapAroundBoundaryIndex, 4, 4));
	objectives.push_back(new MPO_PeriodicRobotStateTrajectoriesObjective(theMotionPlan, "periodic body orientations (PITCH)", 10000.0, theMotionPlan->nSamplePoints - 1, theMotionPlan->wrapAroundBoundaryIndex, 5, 5));
	}*/

	//functional objectives
	objectives.push_back(new MPO_COMTravelObjective(theMotionPlan, "COM Travel objective", 50.0));
	objectives.push_back(new MPO_COMTurningObjective(theMotionPlan, "COM turning objective (YAW)", 50.0));

	//trajectory speed and turning control	
	objectives.push_back(new MPO_FeetPathSmoothnessObjective(theMotionPlan, "foot path smoothness objective", 10.0));
	objectives.push_back(new MPO_SmoothCOMTrajectories(theMotionPlan, "smoothCOM", 50)); //MUST BE IN THE END CUZ CHANGE WEIGHT SOMEWHERE
}

void LocomotionEngine_EnergyFunction::setupTransitionSubObjectives()
{
	for (uint i = 0; i < objectives.size(); i++)
		delete objectives[i];
	objectives.clear();

	//soft constraints. NOTE: important for warmstaring that the order of the constraints does not change!

	//GRF constraints
	objectives.push_back(new MPO_GRFRegularizer(theMotionPlan, "GRF regularizers", 10000.0));
	objectives.push_back(new MPO_GRFSoftBoundConstraints(theMotionPlan, "GRF bound constraints", 10000.0));

	//consistancy constraints (between robot states and other auxiliary variables)
	objectives.push_back(new MPO_RobotEndEffectorsObjective(theMotionPlan, "robot EE objective", 10000.0));
	objectives.push_back(new MPO_RobotCOMObjective(theMotionPlan, "robot COM objective", 10000.0));
	objectives.push_back(new MPO_RobotCOMOrientationsObjective(theMotionPlan, "robot COM orientations objective", 10000.0));

	//dynamics constraints
	objectives.push_back(new MPO_ForceAccelObjective(theMotionPlan, "force acceleration objective", 10000.0));
	objectives.push_back(new MPO_TorqueAngularAccelObjective(theMotionPlan, "torque angular acceleration objective", 10000.0));

	//constraints ensuring feet don't slide...
	objectives.push_back(new MPO_FeetSlidingObjective(theMotionPlan, "feet sliding objective", 10000.0));

	//functional objectives
	objectives.push_back(new MPO_COMTravelObjective(theMotionPlan, "COM Travel objective", 50.0));
	objectives.push_back(new MPO_COMTurningObjective(theMotionPlan, "COM turning objective (YAW)", 50.0));

	//motion regularizers
	objectives.push_back(new MPO_SmoothStanceLegMotionObjective(theMotionPlan, "robot stance leg smooth joint angle trajectories", 0.01));
	objectives.push_back(new MPO_StanceLegMotionRegularizer(theMotionPlan, "robot stance legs motion regularizer", 0.01));
	objectives.push_back(new MPO_FeetPathSmoothnessObjective(theMotionPlan, "foot path smoothness objective", 10.0));

	objectives.push_back(new MPO_RobotStateRegularizer(theMotionPlan, "robot joint angles regularizer objective", 0.0010 * 1, 6, theMotionPlan->robotRepresentation->getDimensionCount() - 1));
	objectives.push_back(new MPO_NonLimbMotionRegularizer(theMotionPlan, "robot joint angles regularizer objective (non-limb)", 0.01));

	objectives.push_back(new MPO_RobotStateRegularizer(theMotionPlan, "robot body orientation regularizer objective (ROLL)", 1, 5, 5));
	objectives.push_back(new MPO_RobotStateRegularizer(theMotionPlan, "robot body orientation regularizer objective (PITCH)", 1, 4, 4));
	objectives.push_back(new MPO_RobotStateRegularizer(theMotionPlan, "robot body orientation regularizer objective (YAW)", 1, 3, 3));
	objectives.push_back(new MPO_SmoothRobotMotionTrajectories(theMotionPlan, "robot smooth joint angle trajectories", 0.01, 6, theMotionPlan->robotRepresentation->getDimensionCount() - 1));
	objectives.push_back(new MPO_SmoothRobotMotionTrajectories(theMotionPlan, "robot smooth body orientation trajectories", 1, 3, 5));
	objectives.push_back(new MPO_NonLimbSmoothMotionObjective(theMotionPlan, "robot smooth joint angles objective (non-limb)", 0.01));

	objectives.push_back(new MPO_SmoothCOMTrajectories(theMotionPlan, "smoothCOM", 50));

	int transStateNum = 2;
	int totalStateNum = (int)theMotionPlan->robotStateTrajectory.qArray.size();

	// transition start objectives
	for (int i = 0; i < transStateNum; i++)
	{
		dVector targetRobotState;
		theMotionPlan->transitionStartPlan->robotStateTrajectory.getQAtTimeIndex(theMotionPlan->transitionStartIndex + i, targetRobotState);
		objectives.push_back(new MPO_RobotStateTransitionRegularizer(theMotionPlan, "transition regularizer", 0, 1, 1, i, targetRobotState));
		objectives.push_back(new MPO_RobotStateTransitionRegularizer(theMotionPlan, "transition regularizer", 0, 4, 4, i, targetRobotState));
		objectives.push_back(new MPO_RobotStateTransitionRegularizer(theMotionPlan, "transition regularizer", 0, 5, 5, i, targetRobotState));
		objectives.push_back(new MPO_RobotStateTransitionRegularizer(theMotionPlan, "transition regularizer", 0, 6, theMotionPlan->robotRepresentation->getDimensionCount() - 1, i, targetRobotState));
	}

	// transition end objectives
	for (int i = 0; i < transStateNum; i++)
	{
		dVector targetRobotState;
		theMotionPlan->transitionEndPlan->robotStateTrajectory.getQAtTimeIndex(theMotionPlan->transitionEndIndex + i, targetRobotState);
		objectives.push_back(new MPO_RobotStateTransitionRegularizer(theMotionPlan, "transition regularizer", 0, 1, 1, totalStateNum - transStateNum + i, targetRobotState));
		objectives.push_back(new MPO_RobotStateTransitionRegularizer(theMotionPlan, "transition regularizer", 0, 4, 4, totalStateNum - transStateNum + i, targetRobotState));
		objectives.push_back(new MPO_RobotStateTransitionRegularizer(theMotionPlan, "transition regularizer", 0, 5, 5, totalStateNum - transStateNum + i, targetRobotState));
		objectives.push_back(new MPO_RobotStateTransitionRegularizer(theMotionPlan, "transition regularizer", 0, 6, theMotionPlan->robotRepresentation->getDimensionCount() - 1, totalStateNum - transStateNum + i, targetRobotState));
	}
}

void LocomotionEngine_EnergyFunction::testIndividualGradient(dVector& params)
{
	vector<double> origWeights;
	for (auto obj : objectives)
	{
		origWeights.push_back(obj->weight);
		obj->weight = 0;
	}

	for (int i = 0; i < (int)objectives.size(); i++)
	{
		if (i > 0)
			objectives[i - 1]->weight = 0;
		objectives[i]->weight = origWeights[i];

		if (objectives[i]->description != "GRF bound constraints") continue;

		Logger::logPrint("\n----------- Testing objective %s -------------\n", objectives[i]->description.c_str());
		Logger::print("\n----------- Testing objective %s -------------\n", objectives[i]->description.c_str());
		testGradientWithFD(params);
	}

	for (int i = 0; i < (int)objectives.size(); i++)
	{
		objectives[i]->weight = origWeights[i];
	}
}

void LocomotionEngine_EnergyFunction::testIndividualHessian(dVector& params)
{
	vector<double> origWeights;
	for (auto obj : objectives)
	{
		origWeights.push_back(obj->weight);
		obj->weight = 0;
	}

	for (int i = 0; i < (int)objectives.size(); i++)
	{
		if (i > 0)
			objectives[i - 1]->weight = 0;
		objectives[i]->weight = origWeights[i];

		if (objectives[i]->description != "GRF bound constraints") continue;

		Logger::print("\n----------- Testing objective %s -------------\n", objectives[i]->description.c_str());
		Logger::logPrint("\n----------- Testing objective %s -------------\n", objectives[i]->description.c_str());
		testHessianWithFD(params);
	}

	for (int i = 0; i < (int)objectives.size(); i++)
	{
		objectives[i]->weight = origWeights[i];
	}
}

double LocomotionEngine_EnergyFunction::computeValue(const dVector& p){
	assert(p.size() == theMotionPlan->paramCount);

	theMotionPlan->setMPParametersFromList(p);

	double totalEnergy = 0;

	for (uint i=0; i<objectives.size(); i++)
		totalEnergy += objectives[i]->computeValue(p);

	//add the regularizer contribution
	if (regularizer > 0){
		resize(tmpVec, p.size());
		if (m_p0.size() != p.size()) m_p0 = p;
		tmpVec = p - m_p0;
		totalEnergy += 0.5*regularizer*tmpVec.dot(tmpVec);
//		Logger::consolePrint("regularizer: %lf\n", regularizer);
	}

	return totalEnergy;
}

//regularizer looks like: r/2 * (p-p0)'*(p-p0). This function can update p0 if desired, given the current value of s.
void LocomotionEngine_EnergyFunction::updateRegularizingSolutionTo(const dVector &currentP){
	m_p0 = currentP;
}

void LocomotionEngine_EnergyFunction::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	hessianEntries.clear();
	theMotionPlan->setMPParametersFromList(p);

	//add the contribution from the regularizer
	if (regularizer > 0){
		for (int i = 0; i < theMotionPlan->paramCount; i++)
			hessianEntries.push_back(MTriplet(i, i, regularizer));
	}

	//and now the contributions of the individual objectives
	for (uint i = 0; i < objectives.size(); i++)
		objectives[i]->addHessianEntriesTo(hessianEntries, p);
}

void LocomotionEngine_EnergyFunction::addGradientTo(dVector& grad, const dVector& p){
	assert(p.size() == theMotionPlan->paramCount);

	theMotionPlan->setMPParametersFromList(p);
	resize(grad, theMotionPlan->paramCount);

	//add the contribution from the regularizer
	if (regularizer > 0){
		if (m_p0.size() != p.size()) m_p0 = p;
		grad = (p - m_p0) * regularizer;
	}

	//and now the contributions of the individual objectives
	for (uint i=0; i<objectives.size(); i++)
		objectives[i]->addGradientTo(grad, p);
}

//this method gets called whenever a new best solution to the objective function is found
void LocomotionEngine_EnergyFunction::setCurrentBestSolution(const dVector& p){
	updateRegularizingSolutionTo(p);
	theMotionPlan->setMPParametersFromList(p);

	if (printDebugInfo){
		Logger::consolePrint("-------------------------------\n");
		Logger::logPrint("-------------------------------\n");

		double totalVal = computeValue(p);
		Logger::consolePrint("=====> total cost: %lf\n", totalVal);
		Logger::logPrint("=====> total cost: %lf\n", totalVal);

		for (uint i=0; i<objectives.size(); i++){
			double w = objectives[i]->weight;
			double v = objectives[i]->computeValue(p);
			Logger::logPrint("%s: %lf (weight: %lf)\n", objectives[i]->description.c_str(), v, w);

			if (objectives[i]->description == "torque angular acceleration objective")
			{
				static_cast<MPO_TorqueAngularAccelObjective*>(objectives[i])->updateDummyMatrices();
			}
		}
	}
}


