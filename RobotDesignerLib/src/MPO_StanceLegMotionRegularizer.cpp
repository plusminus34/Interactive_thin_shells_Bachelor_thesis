#include <RobotDesignerLib/MPO_StanceLegMotionRegularizer.h>

MPO_StanceLegMotionRegularizer::MPO_StanceLegMotionRegularizer(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight){
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_StanceLegMotionRegularizer::~MPO_StanceLegMotionRegularizer(void){
}

double MPO_StanceLegMotionRegularizer::computeValue(const dVector& s){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
//	theMotionPlan->setMPParametersFromList(s);

	double retVal = 0;
	
	int end = theMotionPlan->nSamplePoints;
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count... the last robot pose is already the same as the first one, so no need to penalize twice...

	int nLimbs = theMotionPlan->endEffectorTrajectories.size();
	for (int j=0;j<nLimbs;j++){
		DynamicArray<Joint*> *limbJointList = theMotionPlan->endEffectorTrajectories[j].theLimb->getJointList();
		for (uint k=0;k<limbJointList->size();k++){
			int jIndex = theMotionPlan->robotRepresentation->getQIndexForJoint(limbJointList->at(k));
			for (int tIndex=0;tIndex<end;tIndex++){
				double c = theMotionPlan->endEffectorTrajectories[j].contactFlag[tIndex];
				double tmpV = (theMotionPlan->robotStateTrajectory.defaultRobotStates[tIndex][jIndex] - theMotionPlan->robotStateTrajectory.qArray[tIndex][jIndex]);
				retVal += 0.5 * tmpV*tmpV*c*c;
			}
		}
	}
	
	return retVal * weight;
}

void MPO_StanceLegMotionRegularizer::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);
	assert(grad.size() == theMotionPlan->paramCount);

	int end = theMotionPlan->nSamplePoints;
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count... the last robot pose is already the same as the first one, so no need to penalize twice...

	//and now compute the gradient with respect to the robot q's
	if (theMotionPlan->robotStatesParamsStartIndex >= 0){
		int nLimbs = theMotionPlan->endEffectorTrajectories.size();
		for (int j=0;j<nLimbs;j++){
			DynamicArray<Joint*> *limbJointList = theMotionPlan->endEffectorTrajectories[j].theLimb->getJointList();
			for (uint k=0;k<limbJointList->size();k++){
				int jIndex = theMotionPlan->robotRepresentation->getQIndexForJoint(limbJointList->at(k));
				for (int tIndex=0;tIndex<end;tIndex++){
					double c = theMotionPlan->endEffectorTrajectories[j].contactFlag[tIndex];
					grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * tIndex + jIndex] += -(theMotionPlan->robotStateTrajectory.defaultRobotStates[tIndex][jIndex] - theMotionPlan->robotStateTrajectory.qArray[tIndex][jIndex]) * c*c * weight;
				}
			}
		}
	}
}

void MPO_StanceLegMotionRegularizer::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	int end = theMotionPlan->nSamplePoints;
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count... the last robot pose is already the same as the first one, so no need to penalize twice...

	//and now compute the gradient with respect to the robot q's
	if (theMotionPlan->robotStatesParamsStartIndex >= 0){
		int nLimbs = theMotionPlan->endEffectorTrajectories.size();
		for (int j=0;j<nLimbs;j++){
			DynamicArray<Joint*> *limbJointList = theMotionPlan->endEffectorTrajectories[j].theLimb->getJointList();
			for (uint k=0;k<limbJointList->size();k++){
				int jIndex = theMotionPlan->robotRepresentation->getQIndexForJoint(limbJointList->at(k));
				for (int tIndex=0;tIndex<end;tIndex++){
					double c = theMotionPlan->endEffectorTrajectories[j].contactFlag[tIndex];
					ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * tIndex + jIndex, theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * tIndex + jIndex, 1*c*c, weight);
				}
			}
		}
	}
}
