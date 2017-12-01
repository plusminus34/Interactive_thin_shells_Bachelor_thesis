#include <RobotDesignerLib/MPO_FeetPathSmoothnessObjective.h>

MPO_FeetPathSmoothnessObjective::MPO_FeetPathSmoothnessObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_FeetPathSmoothnessObjective::~MPO_FeetPathSmoothnessObjective(void){
}

double MPO_FeetPathSmoothnessObjective::computeValue(const dVector& p){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);


	double retVal = 0;
	int end = theMotionPlan->nSamplePoints;
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count... the last robot pose is already the same as the first one, which means that COM and feet locations are in correct locations relative to each other, so no need to ask for that again explicitely...

	for (int j=0;j<end;j++){
		int jmm, jm, jp, jpp;

		theMotionPlan->getAccelerationTimeIndicesFor(j, jmm, jm, jp, jpp);
		if (jmm == -1 || jm == -1 || jp == -1 || jpp == -1) continue;

		double dt = theMotionPlan->motionPlanDuration / theMotionPlan->nSamplePoints; dt = 1.0;

		for (uint i=0;i<theMotionPlan->endEffectorTrajectories.size();i++){
			V3D acceleration = (theMotionPlan->endEffectorTrajectories[i].EEPos[jpp] - theMotionPlan->endEffectorTrajectories[i].EEPos[jp])
				-(theMotionPlan->endEffectorTrajectories[i].EEPos[jm] - theMotionPlan->endEffectorTrajectories[i].EEPos[jmm]);		
			acceleration /= (dt * dt);

			retVal += 0.5 * acceleration.length2();
		}
	}
	return retVal * weight;
}

void MPO_FeetPathSmoothnessObjective::addGradientTo(dVector& grad, const dVector& p){
//	assume the parameters of the motion plan have been set already by the collection of objective functions class
//	theMotionPlan->setMPParametersFromList(p);

	//and now compute the gradient with respect c and eePos
	int nLimbs = theMotionPlan->endEffectorTrajectories.size();
	int end = theMotionPlan->nSamplePoints;
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count... the last robot pose is already the same as the first one, which means that COM and feet locations are in correct locations relative to each other, so no need to ask for that again explicitely...

	for (int j=0;j<end;j++){
		int jmm, jm, jp, jpp;

		theMotionPlan->getAccelerationTimeIndicesFor(j, jmm, jm, jp, jpp);
		if (jmm == -1 || jm == -1 || jp == -1 || jpp == -1) continue;

		double dt = theMotionPlan->motionPlanDuration / theMotionPlan->nSamplePoints; dt = 1.0;

		for (int i=0;i<nLimbs;i++){
			V3D acceleration = (theMotionPlan->endEffectorTrajectories[i].EEPos[jpp] - theMotionPlan->endEffectorTrajectories[i].EEPos[jp])
				-(theMotionPlan->endEffectorTrajectories[i].EEPos[jm] - theMotionPlan->endEffectorTrajectories[i].EEPos[jmm]);

			acceleration /= (dt * dt);

			if (theMotionPlan->feetPositionsParamsStartIndex >= 0){
				grad[theMotionPlan->feetPositionsParamsStartIndex + jp * nLimbs * 3 + i * 3 + 0] -= acceleration[0] * weight / (dt * dt);
				grad[theMotionPlan->feetPositionsParamsStartIndex + jpp * nLimbs * 3 + i * 3 + 0] += acceleration[0] * weight / (dt * dt);
				grad[theMotionPlan->feetPositionsParamsStartIndex + jm * nLimbs * 3 + i * 3 + 0] -= acceleration[0] * weight / (dt * dt);
				grad[theMotionPlan->feetPositionsParamsStartIndex + jmm * nLimbs * 3 + i * 3 + 0] += acceleration[0] * weight / (dt * dt);

				grad[theMotionPlan->feetPositionsParamsStartIndex + jp * nLimbs * 3 + i * 3 + 1] -= acceleration[1] * weight / (dt * dt);
				grad[theMotionPlan->feetPositionsParamsStartIndex + jpp * nLimbs * 3 + i * 3 + 1] += acceleration[1] * weight / (dt * dt);
				grad[theMotionPlan->feetPositionsParamsStartIndex + jm * nLimbs * 3 + i * 3 + 1] -= acceleration[1] * weight / (dt * dt);
				grad[theMotionPlan->feetPositionsParamsStartIndex + jmm * nLimbs * 3 + i * 3 + 1] += acceleration[1] * weight / (dt * dt);

				grad[theMotionPlan->feetPositionsParamsStartIndex + jp * nLimbs * 3 + i * 3 + 2] -= acceleration[2] * weight / (dt * dt);
				grad[theMotionPlan->feetPositionsParamsStartIndex + jpp * nLimbs * 3 + i * 3 + 2] += acceleration[2] * weight / (dt * dt);
				grad[theMotionPlan->feetPositionsParamsStartIndex + jm * nLimbs * 3 + i * 3 + 2] -= acceleration[2] * weight / (dt * dt);
				grad[theMotionPlan->feetPositionsParamsStartIndex + jmm * nLimbs * 3 + i * 3 + 2] += acceleration[2] * weight / (dt * dt);
			}
		}
	}
}

void MPO_FeetPathSmoothnessObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	//and now compute the gradient with respect c and eePos
	int nLimbs = theMotionPlan->endEffectorTrajectories.size();
	double retVal = 0;
	int end = theMotionPlan->nSamplePoints;
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count... the last robot pose is already the same as the first one, which means that COM and feet locations are in correct locations relative to each other, so no need to ask for that again explicitely...

	for (int j=0;j<end;j++){
		int jmm, jm, jp, jpp;

		theMotionPlan->getAccelerationTimeIndicesFor(j, jmm, jm, jp, jpp);
		if (jmm == -1 || jm == -1 || jp == -1 || jpp == -1) continue;

//		logPrint("%d %d %d %d\n", jpp, jp, jm, jmm);

		double dt = theMotionPlan->motionPlanDuration / theMotionPlan->nSamplePoints; dt = 1.0;
		double offset = 1 / (dt * dt * dt * dt);

		for (int i=0;i<nLimbs;i++){
			if (theMotionPlan->feetPositionsParamsStartIndex >= 0){
				for (int k = 0; k < 3; ++k) {
					// diagonal, d/dxdx
					ADD_HES_ELEMENT(hessianEntries,
									theMotionPlan->feetPositionsParamsStartIndex + jp * nLimbs * 3 + i * 3 + k,
									theMotionPlan->feetPositionsParamsStartIndex + jp * nLimbs * 3 + i * 3 + k,
									1 * offset,
									weight);
					ADD_HES_ELEMENT(hessianEntries,
									theMotionPlan->feetPositionsParamsStartIndex + jpp * nLimbs * 3 + i * 3 + k,
									theMotionPlan->feetPositionsParamsStartIndex + jpp * nLimbs * 3 + i * 3 + k,
									1 * offset,
									weight);
					ADD_HES_ELEMENT(hessianEntries,
									theMotionPlan->feetPositionsParamsStartIndex + jm * nLimbs * 3 + i * 3 + k,
									theMotionPlan->feetPositionsParamsStartIndex + jm * nLimbs * 3 + i * 3 + k,
									1 * offset,
									weight);
					ADD_HES_ELEMENT(hessianEntries,
									theMotionPlan->feetPositionsParamsStartIndex + jmm * nLimbs * 3 + i * 3 + k,
									theMotionPlan->feetPositionsParamsStartIndex + jmm * nLimbs * 3 + i * 3 + k,
									1 * offset, weight);

					// off-diagonal, d/dxdy
					ADD_HES_ELEMENT(hessianEntries,
									theMotionPlan->feetPositionsParamsStartIndex + jpp * nLimbs * 3 + i * 3 + k,
									theMotionPlan->feetPositionsParamsStartIndex + jp * nLimbs * 3 + i * 3 + k,
									-1 * offset,
									weight);
					ADD_HES_ELEMENT(hessianEntries,
									theMotionPlan->feetPositionsParamsStartIndex + jpp * nLimbs * 3 + i * 3 + k,
									theMotionPlan->feetPositionsParamsStartIndex + jm * nLimbs * 3 + i * 3 + k,
									-1 * offset,
									weight);
					ADD_HES_ELEMENT(hessianEntries,
									theMotionPlan->feetPositionsParamsStartIndex + jpp * nLimbs * 3 + i * 3 + k,
									theMotionPlan->feetPositionsParamsStartIndex + jmm * nLimbs * 3 + i * 3 + k,
									1 * offset,
									weight);
					ADD_HES_ELEMENT(hessianEntries,
									theMotionPlan->feetPositionsParamsStartIndex + jp * nLimbs * 3 + i * 3 + k,
									theMotionPlan->feetPositionsParamsStartIndex + jm * nLimbs * 3 + i * 3 + k,
									1 * offset,
									weight);
					ADD_HES_ELEMENT(hessianEntries,
									theMotionPlan->feetPositionsParamsStartIndex + jp * nLimbs * 3 + i * 3 + k,
									theMotionPlan->feetPositionsParamsStartIndex + jmm * nLimbs * 3 + i * 3 + k,
									-1 * offset,
									weight);
					ADD_HES_ELEMENT(hessianEntries,
									theMotionPlan->feetPositionsParamsStartIndex + jm * nLimbs * 3 + i * 3 + k,
									theMotionPlan->feetPositionsParamsStartIndex + jmm * nLimbs * 3 + i * 3 + k,
									-1 * offset,
									weight);
				}

				if (jp == jm){
					//since the hessian is symmetric, we don't need to be adding the values both below and above the diagonal. However, when the "off-diagonal" elements are actually on the diagonal, they need to be added twice
					for (int k = 0; k < 3; ++k) {
						ADD_HES_ELEMENT(hessianEntries,
										theMotionPlan->feetPositionsParamsStartIndex + jp * nLimbs * 3 + i * 3 + k,
										theMotionPlan->feetPositionsParamsStartIndex + jm * nLimbs * 3 + i * 3 + k,
										1 * offset,
										weight);
					}
				}

			}
		}
	}
}


