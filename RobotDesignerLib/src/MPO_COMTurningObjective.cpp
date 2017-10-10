#include <RobotDesignerLib/MPO_COMTurningObjective.h>



MPO_COMTurningObjective::MPO_COMTurningObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight){
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_COMTurningObjective::~MPO_COMTurningObjective(void){
}

double MPO_COMTurningObjective::computeValue(const dVector& s){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	int startIndex = std::max(0, theMotionPlan->wrapAroundBoundaryIndex);

	double val = (theMotionPlan->COMTrajectory.getCOMEulerAnglesAtTimeIndex(theMotionPlan->nSamplePoints - 1)[0] - theMotionPlan->COMTrajectory.getCOMEulerAnglesAtTimeIndex(startIndex)[0]) - theMotionPlan->desTurningAngle;
	return 0.5 * val * val * weight;
}

void MPO_COMTurningObjective::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	int startIndex = std::max(0, theMotionPlan->wrapAroundBoundaryIndex);

	double val = (theMotionPlan->COMTrajectory.getCOMEulerAnglesAtTimeIndex(theMotionPlan->nSamplePoints - 1)[0] - theMotionPlan->COMTrajectory.getCOMEulerAnglesAtTimeIndex(startIndex)[0]) - theMotionPlan->desTurningAngle;
	if (theMotionPlan->COMOrientationsParamsStartIndex >= 0) {
		grad[theMotionPlan->COMOrientationsParamsStartIndex + 3 * startIndex] -= val*weight;
		grad[theMotionPlan->COMOrientationsParamsStartIndex + 3 * (theMotionPlan->nSamplePoints - 1)] += val*weight;
	}
}

void MPO_COMTurningObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	int startIndex = std::max(0, theMotionPlan->wrapAroundBoundaryIndex);

	if (theMotionPlan->COMOrientationsParamsStartIndex >= 0){
		ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMOrientationsParamsStartIndex + 3 * startIndex, theMotionPlan->COMOrientationsParamsStartIndex + 3 * startIndex, 1, weight);
		ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMOrientationsParamsStartIndex + 3 * (theMotionPlan->nSamplePoints - 1), theMotionPlan->COMOrientationsParamsStartIndex + 3 * (theMotionPlan->nSamplePoints - 1), 1, weight);
		ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMOrientationsParamsStartIndex + 3 * startIndex, theMotionPlan->COMOrientationsParamsStartIndex + 3 * (theMotionPlan->nSamplePoints - 1), -1, weight);
	}

}
