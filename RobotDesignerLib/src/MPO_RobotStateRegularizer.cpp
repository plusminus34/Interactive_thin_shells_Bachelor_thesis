#include <RobotDesignerLib/MPO_RobotStateRegularizer.h>

MPO_RobotStateRegularizer::MPO_RobotStateRegularizer(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight, int startQIndex, int endQIndex){
	theMotionPlan = mp;
	this->startQIndex = startQIndex;
	this->endQIndex = endQIndex;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_RobotStateRegularizer::~MPO_RobotStateRegularizer(void){
}

double MPO_RobotStateRegularizer::computeValue(const dVector& s){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	double retVal = 0;
	
	int end = theMotionPlan->nSamplePoints;
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count... the last robot pose is already the same as the first one, so no need to penalize twice...

	//we want the COM to be as close as possible to the weighted position of the grounded feet (sum_i c_i*w_i*p_i=com_p)
		for (int i=startQIndex;i<=endQIndex;i++)	
			for (int j=0;j<end;j++){
				double tmpV = (theMotionPlan->robotStateTrajectory.defaultRobotStates[j][i] - theMotionPlan->robotStateTrajectory.qArray[j][i]);
				retVal += 0.5 * tmpV*tmpV;
			}
	
	return retVal * weight;
}

void MPO_RobotStateRegularizer::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	int end = theMotionPlan->nSamplePoints;
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count... the last robot pose is already the same as the first one, so no need to penalize twice...

	//and now compute the gradient with respect to the robot q's
	if (theMotionPlan->robotStatesParamsStartIndex >= 0){
		for (int i=startQIndex;i<=endQIndex;i++)
			for (int j=0;j<end;j++)
				grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * j + i] += -(theMotionPlan->robotStateTrajectory.defaultRobotStates[j][i] - theMotionPlan->robotStateTrajectory.qArray[j][i]) * weight;
	}
}

void MPO_RobotStateRegularizer::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	int end = theMotionPlan->nSamplePoints;
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count... the last robot pose is already the same as the first one, so no need to penalize twice...

	//and now compute the gradient with respect to the robot q's
	if (theMotionPlan->robotStatesParamsStartIndex >= 0){
		for (int i=startQIndex;i<=endQIndex;i++)
			for (int j=0;j<end;j++)
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * j + i, theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * j + i, 1, weight);
	}
}
