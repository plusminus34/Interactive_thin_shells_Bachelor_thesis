#include <RobotDesignerLib/MPO_SmoothCOMTrajectories.h>

MPO_SmoothCOMTrajectories::MPO_SmoothCOMTrajectories(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight){
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_SmoothCOMTrajectories::~MPO_SmoothCOMTrajectories(void){
}

double MPO_SmoothCOMTrajectories::computeValue(const dVector& p){
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

		//we want the COM acceleration to be as small as possible
		V3D acceleration = (theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(jpp) - theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(jp))
			-(theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(jm) - theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(jmm));
		acceleration /= (dt * dt);
		retVal += 0.5 * acceleration.dot(acceleration);
	}
	
	return retVal * weight;
}

void MPO_SmoothCOMTrajectories::addGradientTo(dVector& grad, const dVector& p){
//	assume the parameters of the motion plan have been set already by the collection of objective functions class
//	theMotionPlan->setMPParametersFromList(p);
	assert(grad.size() == theMotionPlan->paramCount);

	if (theMotionPlan->COMPositionsParamsStartIndex >= 0){
		int end = theMotionPlan->nSamplePoints;
		if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count... the last robot pose is already the same as the first one, which means that COM and feet locations are in correct locations relative to each other, so no need to ask for that again explicitely...

		for (int j=0;j<end;j++){
			int jmm, jm, jp, jpp;

			theMotionPlan->getAccelerationTimeIndicesFor(j, jmm, jm, jp, jpp);
			if (jmm == -1 || jm == -1 || jp == -1 || jpp == -1) continue;

			V3D acceleration = (theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(jpp) - theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(jp))
				-(theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(jm) - theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(jmm));

			double dt = theMotionPlan->motionPlanDuration / theMotionPlan->nSamplePoints; dt = 1.0;

			acceleration /= (dt * dt);

			for (int k=0;k<3;k++){
				grad[theMotionPlan->COMPositionsParamsStartIndex + 3 * jmm + k] += acceleration[k] * weight / (dt * dt);
				grad[theMotionPlan->COMPositionsParamsStartIndex + 3 * jm + k] -= acceleration[k] * weight / (dt * dt);
				grad[theMotionPlan->COMPositionsParamsStartIndex + 3 * jp + k] -= acceleration[k] * weight / (dt * dt);
				grad[theMotionPlan->COMPositionsParamsStartIndex + 3 * jpp + k] += acceleration[k] * weight / (dt * dt);
			}
		}
	}
}

void MPO_SmoothCOMTrajectories::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

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

		if (theMotionPlan->COMPositionsParamsStartIndex >= 0) {
			for (int k = 0; k<3; k++) {
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * jmm + k, theMotionPlan->COMPositionsParamsStartIndex + 3 * jmm + k, 1 * offset, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * jm + k, theMotionPlan->COMPositionsParamsStartIndex + 3 * jm + k, 1 * offset, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * jp + k, theMotionPlan->COMPositionsParamsStartIndex + 3 * jp + k, 1 * offset, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * jpp + k, theMotionPlan->COMPositionsParamsStartIndex + 3 * jpp + k, 1 * offset, weight);

				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * jpp + k, theMotionPlan->COMPositionsParamsStartIndex + 3 * jp + k, -1 * offset, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * jpp + k, theMotionPlan->COMPositionsParamsStartIndex + 3 * jm + k, -1 * offset, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * jpp + k, theMotionPlan->COMPositionsParamsStartIndex + 3 * jmm + k, 1 * offset, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * jp + k, theMotionPlan->COMPositionsParamsStartIndex + 3 * jm + k, 1 * offset, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * jp + k, theMotionPlan->COMPositionsParamsStartIndex + 3 * jmm + k, -1 * offset, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * jm + k, theMotionPlan->COMPositionsParamsStartIndex + 3 * jmm + k, -1 * offset, weight);
				//since the hessian is symmetric, we don't need to be adding the values both below and above the diagonal. However, when the "off-diagonal" elements are actually on the diagonal, they need to be added twice
				if (jp == jm)
					ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * jp + k, theMotionPlan->COMPositionsParamsStartIndex + 3 * jm + k, 1 * offset, weight);
			}
		}
	}
}


