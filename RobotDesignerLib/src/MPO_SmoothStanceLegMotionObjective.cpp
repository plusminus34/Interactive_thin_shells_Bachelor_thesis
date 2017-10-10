#include <RobotDesignerLib/MPO_SmoothStanceLegMotionObjective.h>

MPO_SmoothStanceLegMotionObjective::MPO_SmoothStanceLegMotionObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight){
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_SmoothStanceLegMotionObjective::~MPO_SmoothStanceLegMotionObjective(void){
}

double MPO_SmoothStanceLegMotionObjective::computeValue(const dVector& s){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	double retVal = 0;

	int nLimbs = theMotionPlan->endEffectorTrajectories.size();
	for (int i=0;i<nLimbs;i++){
		DynamicArray<Joint*> *limbJointList = theMotionPlan->endEffectorTrajectories[i].theLimb->getJointList();
		for (uint k=0;k<limbJointList->size();k++){
			int jIndex = theMotionPlan->robotRepresentation->getQIndexForJoint(limbJointList->at(k));

			int end = theMotionPlan->nSamplePoints;
			if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count... the last robot pose is already the same as the first one, which means that COM and feet locations are in correct locations relative to each other, so no need to ask for that again explicitely...

			for (int j=0;j<end;j++){
				int jmm, jm, jp, jpp;

				theMotionPlan->getAccelerationTimeIndicesFor(j, jmm, jm, jp, jpp);
				if (jmm == -1 || jm == -1 || jp == -1 || jpp == -1) continue;

				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[jp] * theMotionPlan->endEffectorTrajectories[i].contactFlag[jm] * theMotionPlan->endEffectorTrajectories[i].contactFlag[jpp] * theMotionPlan->endEffectorTrajectories[i].contactFlag[jmm];

				double acceleration = (theMotionPlan->robotStateTrajectory.qArray[jpp][jIndex] - theMotionPlan->robotStateTrajectory.qArray[jp][jIndex])
					-(theMotionPlan->robotStateTrajectory.qArray[jm][jIndex] - theMotionPlan->robotStateTrajectory.qArray[jmm][jIndex]);

				retVal += 0.5 * acceleration*acceleration*c*c;
			}
		}
	}
	
	return retVal * weight;
}

void MPO_SmoothStanceLegMotionObjective::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);
	assert(grad.size() == theMotionPlan->paramCount);

	if (theMotionPlan->robotStatesParamsStartIndex >= 0){
		int nLimbs = theMotionPlan->endEffectorTrajectories.size();
		for (int i=0;i<nLimbs;i++){
			DynamicArray<Joint*> *limbJointList = theMotionPlan->endEffectorTrajectories[i].theLimb->getJointList();
			for (uint k=0;k<limbJointList->size();k++){
				int jIndex = theMotionPlan->robotRepresentation->getQIndexForJoint(limbJointList->at(k));

				int end = theMotionPlan->nSamplePoints;
				if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count... the last robot pose is already the same as the first one, which means that COM and feet locations are in correct locations relative to each other, so no need to ask for that again explicitely...

				for (int j=0;j<end;j++){

					int jmm, jm, jp, jpp;

					theMotionPlan->getAccelerationTimeIndicesFor(j, jmm, jm, jp, jpp);
					if (jmm == -1 || jm == -1 || jp == -1 || jpp == -1) continue;

					double acceleration = (theMotionPlan->robotStateTrajectory.qArray[jpp][jIndex] - theMotionPlan->robotStateTrajectory.qArray[jp][jIndex])
						-(theMotionPlan->robotStateTrajectory.qArray[jm][jIndex] - theMotionPlan->robotStateTrajectory.qArray[jmm][jIndex]);


					double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[jp] * theMotionPlan->endEffectorTrajectories[i].contactFlag[jm] * theMotionPlan->endEffectorTrajectories[i].contactFlag[jpp] * theMotionPlan->endEffectorTrajectories[i].contactFlag[jmm];


					grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * jpp + jIndex] += acceleration*c*weight;
					grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * jp + jIndex] -= acceleration*c*weight;
					grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * jm + jIndex] -= acceleration*c*weight;
					grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * jmm + jIndex] += acceleration*c*weight;
				}
			}
		}
	}
}

void MPO_SmoothStanceLegMotionObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	if (theMotionPlan->robotStatesParamsStartIndex >= 0){
		int nLimbs = theMotionPlan->endEffectorTrajectories.size();
		for (int i=0;i<nLimbs;i++){
			DynamicArray<Joint*> *limbJointList = theMotionPlan->endEffectorTrajectories[i].theLimb->getJointList();
			for (uint k=0;k<limbJointList->size();k++){
				int jIndex = theMotionPlan->robotRepresentation->getQIndexForJoint(limbJointList->at(k));

				int end = theMotionPlan->nSamplePoints;
				if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count... the last robot pose is already the same as the first one, which means that COM and feet locations are in correct locations relative to each other, so no need to ask for that again explicitely...

				for (int j=0;j<end;j++){

					int jmm, jm, jp, jpp;

					theMotionPlan->getAccelerationTimeIndicesFor(j, jmm, jm, jp, jpp);
					if (jmm == -1 || jm == -1 || jp == -1 || jpp == -1) continue;

					double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[jp] * theMotionPlan->endEffectorTrajectories[i].contactFlag[jm] * theMotionPlan->endEffectorTrajectories[i].contactFlag[jpp] * theMotionPlan->endEffectorTrajectories[i].contactFlag[jmm];

					ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, c, weight);
					ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + jpp * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, theMotionPlan->robotStatesParamsStartIndex + jpp * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, c, weight);
					ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, c, weight);
					ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + jmm * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, theMotionPlan->robotStatesParamsStartIndex + jmm * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, c, weight);

					ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + jpp * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, -c, weight);
					ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + jpp * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, -c, weight);
					ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + jpp * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, theMotionPlan->robotStatesParamsStartIndex + jmm * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, c, weight);
					ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, c, weight);
					ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, theMotionPlan->robotStatesParamsStartIndex + jmm * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, -c, weight);
					ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, theMotionPlan->robotStatesParamsStartIndex + jmm * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, -c, weight);

					//since the hessian is symmetric, we don't need to be adding the values both below and above the diagonal. However, when the "off-diagonal" elements are actually on the diagonal, they need to be added twice
					if (jp == jm)
						ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, c, weight);

				}
			}
		}
	}
}
