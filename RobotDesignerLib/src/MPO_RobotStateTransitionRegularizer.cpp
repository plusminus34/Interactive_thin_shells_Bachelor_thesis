#include <RobotDesignerLib/MPO_RobotStateTransitionRegularizer.h>

MPO_RobotStateTransitionRegularizer::MPO_RobotStateTransitionRegularizer(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight, int startQIndex, int endQIndex, int stateIndex, dVector& targetRobotState){
	theMotionPlan = mp;
	this->startQIndex = startQIndex;
	this->endQIndex = endQIndex;
	this->description = objectiveDescription;
	this->weight = weight;
	this->targetRobotState = targetRobotState;
	this->stateIndex = stateIndex;
}

MPO_RobotStateTransitionRegularizer::~MPO_RobotStateTransitionRegularizer(void){
}

double MPO_RobotStateTransitionRegularizer::computeValue(const dVector& s){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	double retVal = 0;
	
	int end = theMotionPlan->nSamplePoints;
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count... the last robot pose is already the same as the first one, so no need to penalize twice...

	//we want the COM to be as close as possible to the weighted position of the grounded feet (sum_i c_i*w_i*p_i=com_p)
	for (int i = startQIndex; i <= endQIndex; i++) {
		double tmpV = theMotionPlan->robotStateTrajectory.qArray[stateIndex][i] - targetRobotState[i];
		retVal += 0.5 * tmpV*tmpV;
	}
	
	return retVal * weight;
}

void MPO_RobotStateTransitionRegularizer::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	int end = theMotionPlan->nSamplePoints;
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count... the last robot pose is already the same as the first one, so no need to penalize twice...

	//and now compute the gradient with respect to the robot q's
	if (theMotionPlan->robotStatesParamsStartIndex >= 0){		
		for (int i = startQIndex; i <= endQIndex; i++)
			grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * stateIndex + i] += (theMotionPlan->robotStateTrajectory.qArray[stateIndex][i] - targetRobotState[i]) * weight;
	}
}

void MPO_RobotStateTransitionRegularizer::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	int end = theMotionPlan->nSamplePoints;
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count... the last robot pose is already the same as the first one, so no need to penalize twice...

	//and now compute the gradient with respect to the robot q's
	if (theMotionPlan->robotStatesParamsStartIndex >= 0){
		for (int i=startQIndex;i<=endQIndex;i++)
			ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * stateIndex + i, theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * stateIndex + i, 1, weight);
	}
}
