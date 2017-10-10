#include <RobotDesignerLib/MPO_SmoothRobotMotionTrajectories.h>

MPO_SmoothRobotMotionTrajectories::MPO_SmoothRobotMotionTrajectories(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight, int startQIndex, int endQIndex){
	theMotionPlan = mp;
	this->startQIndex = startQIndex;
	this->endQIndex = endQIndex;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_SmoothRobotMotionTrajectories::~MPO_SmoothRobotMotionTrajectories(void){
}

double MPO_SmoothRobotMotionTrajectories::computeValue(const dVector& s){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	double retVal = 0;
	
	int end = theMotionPlan->nSamplePoints;
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count... the last robot pose is already the same as the first one, which means that COM and feet locations are in correct locations relative to each other, so no need to ask for that again explicitely...

	for (int j=0;j<end;j++){
		int jmm, jm, jp, jpp;

		theMotionPlan->getAccelerationTimeIndicesFor(j, jmm, jm, jp, jpp);
		if (jmm == -1 || jm == -1 || jp == -1 || jpp == -1) continue;

//		logPrint("%d <-> %d, %d <-> %d\n", jmm, jm, jp, jpp);
		double dt = theMotionPlan->motionPlanDuration / theMotionPlan->nSamplePoints; dt = 1.0;
		//we want the COM to be as close as possible to the weighted position of the grounded feet (sum_i c_i*w_i*p_i=com_p)
		for (int i=startQIndex;i<=endQIndex;i++){
			double acceleration = (theMotionPlan->robotStateTrajectory.qArray[jpp][i] - theMotionPlan->robotStateTrajectory.qArray[jp][i])
				-(theMotionPlan->robotStateTrajectory.qArray[jm][i] - theMotionPlan->robotStateTrajectory.qArray[jmm][i]);
			acceleration /= (dt * dt);
			retVal += 0.5 * acceleration*acceleration;
		}
	}
	
	return retVal * weight;
}

void MPO_SmoothRobotMotionTrajectories::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	//and now compute the gradient with respect to the robot q's
	if (theMotionPlan->robotStatesParamsStartIndex >= 0){
		int end = theMotionPlan->nSamplePoints;
		if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count... the last robot pose is already the same as the first one, which means that COM and feet locations are in correct locations relative to each other, so no need to ask for that again explicitely...

		for (int j=0;j<end;j++){
			int jmm, jm, jp, jpp;

			theMotionPlan->getAccelerationTimeIndicesFor(j, jmm, jm, jp, jpp);
			if (jmm == -1 || jm == -1 || jp == -1 || jpp == -1) continue;

			double dt = theMotionPlan->motionPlanDuration / theMotionPlan->nSamplePoints; dt = 1.0;

			//we want the COM to be as close as possible to the weighted position of the grounded feet (sum_i c_i*w_i*p_i=com_p)
			for (int i=startQIndex;i<=endQIndex;i++){
				double acceleration = (theMotionPlan->robotStateTrajectory.qArray[jpp][i] - theMotionPlan->robotStateTrajectory.qArray[jp][i])
					-(theMotionPlan->robotStateTrajectory.qArray[jm][i] - theMotionPlan->robotStateTrajectory.qArray[jmm][i]);

				acceleration /= (dt * dt);

				grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * jpp + i] += acceleration * weight / (dt * dt);
				grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * jp + i] -= acceleration * weight / (dt * dt);
				grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * jm + i] -= acceleration * weight / (dt * dt);
				grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * jmm + i] += acceleration * weight / (dt * dt);
			}
		}
	}
}

void MPO_SmoothRobotMotionTrajectories::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	int end = theMotionPlan->nSamplePoints;
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count... the last robot pose is already the same as the first one, which means that COM and feet locations are in correct locations relative to each other, so no need to ask for that again explicitely...

	for (int j=0;j<end;j++){
		int jmm, jm, jp, jpp;

		theMotionPlan->getAccelerationTimeIndicesFor(j, jmm, jm, jp, jpp);
		if (jmm == -1 || jm == -1 || jp == -1 || jpp == -1) continue;

//		logPrint("%d %d %d %d\n", jpp, jp, jm, jmm);

		double dt = theMotionPlan->motionPlanDuration / theMotionPlan->nSamplePoints; dt = 1.0;
		double offset = 1 / (dt * dt * dt * dt);

		for (int i=startQIndex;i<=endQIndex;i++){
			if (theMotionPlan->robotStatesParamsStartIndex >= 0){
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + i, theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + i, 1 * offset, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + jpp * theMotionPlan->robotStateTrajectory.nStateDim + i, theMotionPlan->robotStatesParamsStartIndex + jpp * theMotionPlan->robotStateTrajectory.nStateDim + i, 1 * offset, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + i, theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + i, 1 * offset, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + jmm * theMotionPlan->robotStateTrajectory.nStateDim + i, theMotionPlan->robotStatesParamsStartIndex + jmm * theMotionPlan->robotStateTrajectory.nStateDim + i, 1 * offset, weight);

				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + jpp * theMotionPlan->robotStateTrajectory.nStateDim + i, theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + i, -1 * offset, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + jpp * theMotionPlan->robotStateTrajectory.nStateDim + i, theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + i, -1 * offset, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + jpp * theMotionPlan->robotStateTrajectory.nStateDim + i, theMotionPlan->robotStatesParamsStartIndex + jmm * theMotionPlan->robotStateTrajectory.nStateDim + i, 1 * offset, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + i, theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + i, 1 * offset, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + i, theMotionPlan->robotStatesParamsStartIndex + jmm * theMotionPlan->robotStateTrajectory.nStateDim + i, -1 * offset, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + i, theMotionPlan->robotStatesParamsStartIndex + jmm * theMotionPlan->robotStateTrajectory.nStateDim + i, -1 * offset, weight);

				//since the hessian is symmetric, we don't need to be adding the values both below and above the diagonal. However, when the "off-diagonal" elements are actually on the diagonal, they need to be added twice
				if (jp == jm)
					ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + i, theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + i, 1 * offset, weight);
			}
		}
	}
}


