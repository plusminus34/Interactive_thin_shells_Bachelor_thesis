#include <RobotDesignerLib/MPO_RobotCOMOrientationsObjective.h>

MPO_RobotCOMOrientationsObjective::MPO_RobotCOMOrientationsObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_RobotCOMOrientationsObjective::~MPO_RobotCOMOrientationsObjective(void){
}

double MPO_RobotCOMOrientationsObjective::computeValue(const dVector& p){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	double retVal = 0;
	//we want the COM to be as close as possible to the center of the feet that are in contact with the ground, which corresponds to weights as large as possible and equal to each other...
	for (int j=0;j<theMotionPlan->nSamplePoints;j++){
        V3D comOrientation = (V3D)(theMotionPlan->robotStateTrajectory.qArray[j].segment<3>(3));
		V3D err = V3D(P3D(comOrientation), theMotionPlan->COMTrajectory.getCOMEulerAnglesAtTimeIndex(j));
		retVal += 0.5 * err.length2();
	}
	return retVal * weight;
}

void MPO_RobotCOMOrientationsObjective::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	for (int j=0;j<theMotionPlan->nSamplePoints;j++){
        V3D comOrientation = (V3D)(theMotionPlan->robotStateTrajectory.qArray[j].segment<3>(3));
		P3D desOrientation = theMotionPlan->COMTrajectory.getCOMEulerAnglesAtTimeIndex(j);
		V3D err = V3D(P3D(comOrientation), desOrientation);

		//compute the gradient with respect to the COM positions
		if (theMotionPlan->COMOrientationsParamsStartIndex >= 0){
			grad.segment<3>(theMotionPlan->COMOrientationsParamsStartIndex + 3 * j) += err * weight;
		}

		if (theMotionPlan->robotStatesParamsStartIndex >= 0) {
			grad.segment<3>(theMotionPlan->robotStatesParamsStartIndex + (j * theMotionPlan->robotStateTrajectory.nStateDim) + 3) -= err * weight;
		}
	}
}

void MPO_RobotCOMOrientationsObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	for (int j = 0; j<theMotionPlan->nSamplePoints; j++) {
        V3D comOrientation = (V3D)(theMotionPlan->robotStateTrajectory.qArray[j].segment<3>(3));
		P3D desOrientation = theMotionPlan->COMTrajectory.getCOMEulerAnglesAtTimeIndex(j);
		V3D err = V3D(P3D(comOrientation), desOrientation);
		int indexI = theMotionPlan->COMOrientationsParamsStartIndex + 3 * j;
		int indexJ = theMotionPlan->robotStatesParamsStartIndex + (j * theMotionPlan->robotStateTrajectory.nStateDim) + 3;

		//compute the gradient with respect to the COM positions
		if (theMotionPlan->COMOrientationsParamsStartIndex >= 0) {
			ADD_HES_ELEMENT(hessianEntries, indexI, indexI, 1, weight);
			ADD_HES_ELEMENT(hessianEntries, indexI + 1, indexI + 1, 1, weight);
			ADD_HES_ELEMENT(hessianEntries, indexI + 2, indexI + 2, 1, weight);

			if (theMotionPlan->robotStatesParamsStartIndex >= 0)
			{
				ADD_HES_ELEMENT(hessianEntries, indexI, indexJ, -1, weight);
				ADD_HES_ELEMENT(hessianEntries, indexI + 1, indexJ + 1, -1, weight);
				ADD_HES_ELEMENT(hessianEntries, indexI + 2, indexJ + 2,  -1, weight);
			}
		}

		if (theMotionPlan->robotStatesParamsStartIndex >= 0) {
			ADD_HES_ELEMENT(hessianEntries, indexJ, indexJ, 1, weight);
			ADD_HES_ELEMENT(hessianEntries, indexJ + 1, indexJ + 1, 1, weight);
			ADD_HES_ELEMENT(hessianEntries, indexJ + 2, indexJ + 2, 1, weight);
		}
	}
}


