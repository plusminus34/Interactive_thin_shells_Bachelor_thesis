#include <RobotDesignerLib/MPO_NonLimbSmoothMotionObjective.h>

MPO_NonLimbSmoothMotionObjective::MPO_NonLimbSmoothMotionObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight){
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

MPO_NonLimbSmoothMotionObjective::~MPO_NonLimbSmoothMotionObjective(void){
}

double MPO_NonLimbSmoothMotionObjective::computeValue(const dVector& s){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	double retVal = 0;
	int end = theMotionPlan->nSamplePoints;
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count... the last robot pose is already the same as the first one, which means that COM and feet locations are in correct locations relative to each other, so no need to ask for that again explicitely...

	for (int j = 0; j<end; j++) {
		int jmm, jm, jp, jpp;

		theMotionPlan->getAccelerationTimeIndicesFor(j, jmm, jm, jp, jpp);
		if (jmm == -1 || jm == -1 || jp == -1 || jpp == -1) continue;

		double dt = theMotionPlan->motionPlanDuration / theMotionPlan->nSamplePoints; dt = 1.0;
		//we want the COM to be as close as possible to the weighted position of the grounded feet (sum_i c_i*w_i*p_i=com_p)
		for (uint i = 0; i < nonLimbJointIndices.size(); i++) {
			int jIndex = nonLimbJointIndices[i];
			double acceleration = (theMotionPlan->robotStateTrajectory.qArray[jpp][jIndex] - theMotionPlan->robotStateTrajectory.qArray[jp][jIndex])
				- (theMotionPlan->robotStateTrajectory.qArray[jm][jIndex] - theMotionPlan->robotStateTrajectory.qArray[jmm][jIndex]);
			acceleration /= (dt * dt);
			retVal += 0.5 * acceleration*acceleration;
		}
	}
	
	return retVal * weight;
}

void MPO_NonLimbSmoothMotionObjective::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);
	assert(grad.size() == theMotionPlan->paramCount);

	if (theMotionPlan->robotStatesParamsStartIndex >= 0){
		int end = theMotionPlan->nSamplePoints;
		if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count... the last robot pose is already the same as the first one, which means that COM and feet locations are in correct locations relative to each other, so no need to ask for that again explicitely...

		for (int j = 0; j<end; j++) {
			int jmm, jm, jp, jpp;

			theMotionPlan->getAccelerationTimeIndicesFor(j, jmm, jm, jp, jpp);
			if (jmm == -1 || jm == -1 || jp == -1 || jpp == -1) continue;

			double dt = theMotionPlan->motionPlanDuration / theMotionPlan->nSamplePoints; dt = 1.0;

			//we want the COM to be as close as possible to the weighted position of the grounded feet (sum_i c_i*w_i*p_i=com_p)
			for (uint i = 0; i < nonLimbJointIndices.size(); i++) {
				int jIndex = nonLimbJointIndices[i];
				double acceleration = (theMotionPlan->robotStateTrajectory.qArray[jpp][jIndex] - theMotionPlan->robotStateTrajectory.qArray[jp][jIndex])
					- (theMotionPlan->robotStateTrajectory.qArray[jm][jIndex] - theMotionPlan->robotStateTrajectory.qArray[jmm][jIndex]);

				acceleration /= (dt * dt);

				grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * jpp + jIndex] += acceleration * weight / (dt * dt);
				grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * jp + jIndex] -= acceleration * weight / (dt * dt);
				grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * jm + jIndex] -= acceleration * weight / (dt * dt);
				grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * jmm + jIndex] += acceleration * weight / (dt * dt);
			}
		}
	}
}

void MPO_NonLimbSmoothMotionObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);


	int end = theMotionPlan->nSamplePoints;
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count... the last robot pose is already the same as the first one, which means that COM and feet locations are in correct locations relative to each other, so no need to ask for that again explicitely...

	for (int j = 0; j<end; j++) {
		int jmm, jm, jp, jpp;

		theMotionPlan->getAccelerationTimeIndicesFor(j, jmm, jm, jp, jpp);
		if (jmm == -1 || jm == -1 || jp == -1 || jpp == -1) continue;

		//		logPrint("%d %d %d %d\n", jpp, jp, jm, jmm);

		double dt = theMotionPlan->motionPlanDuration / theMotionPlan->nSamplePoints; dt = 1.0;
		double offset = 1 / (dt * dt * dt * dt);

		for (uint i = 0; i < nonLimbJointIndices.size(); i++) {
			int jIndex = nonLimbJointIndices[i];
			if (theMotionPlan->robotStatesParamsStartIndex >= 0) {
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, 1 * offset, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + jpp * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, theMotionPlan->robotStatesParamsStartIndex + jpp * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, 1 * offset, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, 1 * offset, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + jmm * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, theMotionPlan->robotStatesParamsStartIndex + jmm * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, 1 * offset, weight);

				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + jpp * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, -1 * offset, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + jpp * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, -1 * offset, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + jpp * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, theMotionPlan->robotStatesParamsStartIndex + jmm * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, 1 * offset, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, 1 * offset, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, theMotionPlan->robotStatesParamsStartIndex + jmm * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, -1 * offset, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, theMotionPlan->robotStatesParamsStartIndex + jmm * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, -1 * offset, weight);

				//since the hessian is symmetric, we don't need to be adding the values both below and above the diagonal. However, when the "off-diagonal" elements are actually on the diagonal, they need to be added twice
				if (jp == jm)
					ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + jIndex, 1 * offset, weight);
			}
		}
	}
}
