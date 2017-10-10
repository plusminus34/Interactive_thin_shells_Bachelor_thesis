#include <RobotDesignerLib/MPO_NonLimbMotionRegularizer.h>

MPO_NonLimbMotionRegularizer::MPO_NonLimbMotionRegularizer(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight){
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;

	for (uint i = 0; i < theMotionPlan->robot->jointList.size(); i++) {
		int iIndex = theMotionPlan->robotRepresentation->getQIndexForJoint(theMotionPlan->robot->getJoint(i));
		bool isLimbJoint = false;
		for (uint j = 0; j < theMotionPlan->robot->bFrame->limbs.size(); j++) {
			DynamicArray<Joint*> *limbJointList = theMotionPlan->robot->bFrame->limbs[j]->getJointList();
			for (uint k = 0; k < limbJointList->size(); k++) {
				int jIndex = theMotionPlan->robotRepresentation->getQIndexForJoint(limbJointList->at(k));
				if (iIndex == jIndex)
					isLimbJoint = true;
			}
		}
		if (isLimbJoint == false)
			nonLimbJointIndices.push_back(iIndex);
	}
}

MPO_NonLimbMotionRegularizer::~MPO_NonLimbMotionRegularizer(void){
}

double MPO_NonLimbMotionRegularizer::computeValue(const dVector& s){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
//	theMotionPlan->setMPParametersFromList(s);

	double retVal = 0;

	int end = theMotionPlan->nSamplePoints;
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count... the last robot pose is already the same as the first one, so no need to penalize twice...

	for (uint i=0;i<nonLimbJointIndices.size();i++){
		for (int tIndex = 0; tIndex<end; tIndex++) {
			int jIndex = nonLimbJointIndices[i];
			double tmpV = (theMotionPlan->robotStateTrajectory.defaultRobotStates[tIndex][jIndex] - theMotionPlan->robotStateTrajectory.qArray[tIndex][jIndex]);
			retVal += 0.5 * tmpV * tmpV;
		}
	}
	
	return retVal * weight;
}

void MPO_NonLimbMotionRegularizer::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);
	assert(grad.size() == theMotionPlan->paramCount);

	int end = theMotionPlan->nSamplePoints;
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count... the last robot pose is already the same as the first one, so no need to penalize twice...

	//and now compute the gradient with respect to the robot q's
	if (theMotionPlan->robotStatesParamsStartIndex >= 0){
		for (uint i = 0; i<nonLimbJointIndices.size(); i++) {
			for (int tIndex = 0; tIndex<end; tIndex++) {
				int jIndex = nonLimbJointIndices[i];
				double tmpV = (theMotionPlan->robotStateTrajectory.defaultRobotStates[tIndex][jIndex] - theMotionPlan->robotStateTrajectory.qArray[tIndex][jIndex]);
				grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * tIndex + jIndex] += -tmpV * weight;
			}
		}
	}
}

void MPO_NonLimbMotionRegularizer::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	int end = theMotionPlan->nSamplePoints;
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count... the last robot pose is already the same as the first one, so no need to penalize twice...

	//and now compute the gradient with respect to the robot q's
	if (theMotionPlan->robotStatesParamsStartIndex >= 0){
		for (uint i = 0; i<nonLimbJointIndices.size(); i++) {
			for (int tIndex = 0; tIndex<end; tIndex++) {
				int jIndex = nonLimbJointIndices[i];
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * tIndex + jIndex, theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * tIndex + jIndex, 1, weight);
			}
		}
	}
}
