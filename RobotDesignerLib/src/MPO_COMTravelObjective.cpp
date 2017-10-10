#include <RobotDesignerLib/MPO_COMTravelObjective.h>

MPO_COMTravelObjective::MPO_COMTravelObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_COMTravelObjective::~MPO_COMTravelObjective(void){
}

double MPO_COMTravelObjective::computeValue(const dVector& p){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	V3D err;
	int startIndex = std::max(0, theMotionPlan->wrapAroundBoundaryIndex);

	V3D v(theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(startIndex), theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(theMotionPlan->nSamplePoints - 1));
	err = v - theMotionPlan->desDistanceToTravel;

	return 0.5 * err.length2() * weight;
}

void MPO_COMTravelObjective::addGradientTo(dVector& grad, const dVector& p){
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	V3D err;
	int startIndex = std::max(0, theMotionPlan->wrapAroundBoundaryIndex);

	V3D v(theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(startIndex), theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(theMotionPlan->nSamplePoints - 1));
	err = v - theMotionPlan->desDistanceToTravel;
	if (theMotionPlan->COMPositionsParamsStartIndex >= 0) {
		grad[theMotionPlan->COMPositionsParamsStartIndex + 3 * startIndex + 0] += -err[0] * weight;
		grad[theMotionPlan->COMPositionsParamsStartIndex + 3 * startIndex + 1] += -err[1] * weight;
		grad[theMotionPlan->COMPositionsParamsStartIndex + 3 * startIndex + 2] += -err[2] * weight;

		grad[theMotionPlan->COMPositionsParamsStartIndex + 3 * (theMotionPlan->nSamplePoints - 1) + 0] += err[0] * weight;
		grad[theMotionPlan->COMPositionsParamsStartIndex + 3 * (theMotionPlan->nSamplePoints - 1) + 1] += err[1] * weight;
		grad[theMotionPlan->COMPositionsParamsStartIndex + 3 * (theMotionPlan->nSamplePoints - 1) + 2] += err[2] * weight;
	}
}

void MPO_COMTravelObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	int startIndex = std::max(0, theMotionPlan->wrapAroundBoundaryIndex);

	if (theMotionPlan->COMPositionsParamsStartIndex >= 0){
		ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * startIndex + 0, theMotionPlan->COMPositionsParamsStartIndex + 3 * startIndex + 0, 1, weight);
		ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * startIndex + 1, theMotionPlan->COMPositionsParamsStartIndex + 3 * startIndex + 1, 1, weight);
		ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * startIndex + 2, theMotionPlan->COMPositionsParamsStartIndex + 3 * startIndex + 2, 1, weight);

		ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * (theMotionPlan->nSamplePoints - 1) + 0, theMotionPlan->COMPositionsParamsStartIndex + 3 * (theMotionPlan->nSamplePoints - 1) + 0, 1, weight);
		ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * (theMotionPlan->nSamplePoints - 1) + 1, theMotionPlan->COMPositionsParamsStartIndex + 3 * (theMotionPlan->nSamplePoints - 1) + 1, 1, weight);
		ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * (theMotionPlan->nSamplePoints - 1) + 2, theMotionPlan->COMPositionsParamsStartIndex + 3 * (theMotionPlan->nSamplePoints - 1) + 2, 1, weight);

		ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * startIndex + 0, theMotionPlan->COMPositionsParamsStartIndex + 3 * (theMotionPlan->nSamplePoints - 1) + 0, -1, weight);
		ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * startIndex + 1, theMotionPlan->COMPositionsParamsStartIndex + 3 * (theMotionPlan->nSamplePoints - 1) + 1, -1, weight);
		ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * startIndex + 2, theMotionPlan->COMPositionsParamsStartIndex + 3 * (theMotionPlan->nSamplePoints - 1) + 2, -1, weight);
	}
}


