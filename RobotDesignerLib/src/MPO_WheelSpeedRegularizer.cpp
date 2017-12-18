#include <RobotDesignerLib/MPO_WheelSpeedRegularizer.h>

MPO_WheelSpeedRegularizer::MPO_WheelSpeedRegularizer(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight){
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_WheelSpeedRegularizer::~MPO_WheelSpeedRegularizer(void){
}

double MPO_WheelSpeedRegularizer::computeValue(const dVector& s){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	double retVal = 0;
	
	int end = theMotionPlan->nSamplePoints;
//	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count... the last robot pose is already the same as the first one, so no need to penalize twice...

	for (int j=0;j<end;j++){
		for (int i = 0; i < theMotionPlan->endEffectorTrajectories.size(); ++i) {
			double tmpV = theMotionPlan->endEffectorTrajectories[i].wheelSpeed[j];
			retVal += 0.5 * tmpV*tmpV;
		}
	}
	
	return retVal * weight;
}

void MPO_WheelSpeedRegularizer::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	if(theMotionPlan->wheelParamsStartIndex < 0)
		return;

	int end = theMotionPlan->nSamplePoints;
//	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count... the last robot pose is already the same as the first one, so no need to penalize twice...

	//and now compute the gradient with respect to the robot q's
	if (theMotionPlan->robotStatesParamsStartIndex >= 0){
		for (int i = 0; i < theMotionPlan->endEffectorTrajectories.size(); ++i)
			for (int j=0;j<end;j++)
				grad[theMotionPlan->getWheelSpeedIndex(i, j)] += theMotionPlan->endEffectorTrajectories[i].wheelSpeed[j] * weight;
	}
}

void MPO_WheelSpeedRegularizer::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	if(theMotionPlan->wheelParamsStartIndex < 0)
		return;

	int end = theMotionPlan->nSamplePoints;
//	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count... the last robot pose is already the same as the first one, so no need to penalize twice...

	//and now compute the gradient with respect to the robot q's
	if (theMotionPlan->robotStatesParamsStartIndex >= 0){
		for (int i = 0; i < theMotionPlan->endEffectorTrajectories.size(); ++i)
			for (int j=0;j<end;j++)
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->getWheelSpeedIndex(i, j), theMotionPlan->getWheelSpeedIndex(i, j), 1, weight);
	}
}
