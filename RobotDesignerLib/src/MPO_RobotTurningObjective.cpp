#include <RobotDesignerLib/MPO_RobotTurningObjective.h>

#define YAW_ANGLE_INDEX 3

MPO_RobotTurningObjective::MPO_RobotTurningObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight){
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_RobotTurningObjective::~MPO_RobotTurningObjective(void){
}

double MPO_RobotTurningObjective::computeValue(const dVector& s){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	if (theMotionPlan->wrapAroundBoundaryIndex >= 0){
		double val = (theMotionPlan->robotStateTrajectory.qArray[theMotionPlan->nSamplePoints-1][YAW_ANGLE_INDEX] - theMotionPlan->robotStateTrajectory.qArray[theMotionPlan->wrapAroundBoundaryIndex][YAW_ANGLE_INDEX]) - theMotionPlan->desTurningAngle;
		return 0.5 * val * val * weight;
	}
		
	return 0;
}

void MPO_RobotTurningObjective::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	if (theMotionPlan->wrapAroundBoundaryIndex >= 0){
		double val = (theMotionPlan->robotStateTrajectory.qArray[theMotionPlan->nSamplePoints-1][YAW_ANGLE_INDEX] - theMotionPlan->robotStateTrajectory.qArray[theMotionPlan->wrapAroundBoundaryIndex][YAW_ANGLE_INDEX]) - theMotionPlan->desTurningAngle;
		if (theMotionPlan->robotStatesParamsStartIndex >= 0){
			grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * theMotionPlan->wrapAroundBoundaryIndex + YAW_ANGLE_INDEX] -= val*weight;
			grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * (theMotionPlan->nSamplePoints-1) + YAW_ANGLE_INDEX] += val*weight;
		}
	}

}

void MPO_RobotTurningObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	if (theMotionPlan->robotStatesParamsStartIndex >= 0){
		ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->wrapAroundBoundaryIndex * theMotionPlan->robotStateTrajectory.nStateDim + YAW_ANGLE_INDEX, theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->wrapAroundBoundaryIndex * theMotionPlan->robotStateTrajectory.nStateDim + YAW_ANGLE_INDEX, 1, weight);
		ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + (theMotionPlan->nSamplePoints-1) * theMotionPlan->robotStateTrajectory.nStateDim + YAW_ANGLE_INDEX, theMotionPlan->robotStatesParamsStartIndex + (theMotionPlan->nSamplePoints-1) * theMotionPlan->robotStateTrajectory.nStateDim + YAW_ANGLE_INDEX, 1, weight);
		ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + (theMotionPlan->nSamplePoints-1) * theMotionPlan->robotStateTrajectory.nStateDim + YAW_ANGLE_INDEX, theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->wrapAroundBoundaryIndex * theMotionPlan->robotStateTrajectory.nStateDim + YAW_ANGLE_INDEX, -1, weight);
	}

}
