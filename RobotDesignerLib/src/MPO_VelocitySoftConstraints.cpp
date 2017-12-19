#include <RobotDesignerLib/MPO_VelocitySoftConstraints.h>

MPO_VelocitySoftBoundConstraints::MPO_VelocitySoftBoundConstraints(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight, int startQIndex, int endQIndex) {

	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
	this->startQIndex = startQIndex;
	this->endQIndex = endQIndex;

	constraintSymmetricBound = std::make_shared<SoftSymmetricBarrierConstraint>(theMotionPlan->jointVelocityLimit, 10);
}

MPO_VelocitySoftBoundConstraints::~MPO_VelocitySoftBoundConstraints(void) {
}

double MPO_VelocitySoftBoundConstraints::computeValue(const dVector& s) {

	if (theMotionPlan->robotStatesParamsStartIndex >= 0){

		constraintSymmetricBound->limit = theMotionPlan->jointVelocityLimit;

		double retVal = 0;

		int nSamplePoints = theMotionPlan->nSamplePoints;
		if (theMotionPlan->wrapAroundBoundaryIndex >= 0) nSamplePoints -= 1; //don't double count... the last robot pose is already the same as the first one, which means that COM and feet locations are in correct locations relative to each other, so no need to ask for that again explicitely...

		for (int j=0; j<nSamplePoints; j++){

			int jm, jp;

			theMotionPlan->getVelocityTimeIndicesFor(j, jm, jp);
			if (jm == -1 || jp == -1) continue;

			double dt = theMotionPlan->motionPlanDuration / theMotionPlan->nSamplePoints;

			for (int i=startQIndex; i<=endQIndex; i++){
				double velocity = (theMotionPlan->robotStateTrajectory.qArray[jp][i] - theMotionPlan->robotStateTrajectory.qArray[jm][i]) / dt;
				retVal += constraintSymmetricBound->computeValue(velocity);
			}
		}

		return retVal * weight;
	}

	return 0;
}

void MPO_VelocitySoftBoundConstraints::addGradientTo(dVector& grad, const dVector& p) {

	if (theMotionPlan->robotStatesParamsStartIndex >= 0){

		constraintSymmetricBound->limit = theMotionPlan->jointVelocityLimit;

		int nSamplePoints = theMotionPlan->nSamplePoints;
		if (theMotionPlan->wrapAroundBoundaryIndex >= 0) nSamplePoints -= 1; //don't double count... the last robot pose is already the same as the first one, which means that COM and feet locations are in correct locations relative to each other, so no need to ask for that again explicitely...

		for (int j=0; j<nSamplePoints; j++){

			int jm, jp;

			theMotionPlan->getVelocityTimeIndicesFor(j, jm, jp);
			if (jm == -1 || jp == -1) continue;

			double dt = theMotionPlan->motionPlanDuration / theMotionPlan->nSamplePoints;

			for (int i=startQIndex; i<=endQIndex; i++){
				double velocity = (theMotionPlan->robotStateTrajectory.qArray[jp][i] - theMotionPlan->robotStateTrajectory.qArray[jm][i]) / dt;
				double dC = constraintSymmetricBound->computeDerivative(velocity);
				

				grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * jm + i] -= weight * dC / dt;
				grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * jp + i] += weight * dC / dt;
			}
		}
	}
}

void MPO_VelocitySoftBoundConstraints::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {

	if (theMotionPlan->robotStatesParamsStartIndex >= 0){

		constraintSymmetricBound->limit = theMotionPlan->jointVelocityLimit;

		int nSamplePoints = theMotionPlan->nSamplePoints;
		if (theMotionPlan->wrapAroundBoundaryIndex >= 0) nSamplePoints -= 1; //don't double count... the last robot pose is already the same as the first one, which means that COM and feet locations are in correct locations relative to each other, so no need to ask for that again explicitely...

		for (int j=0; j<nSamplePoints; j++){

			int jm, jp;

			theMotionPlan->getVelocityTimeIndicesFor(j, jm, jp);
			if (jm == -1 || jp == -1) continue;

			double dt = theMotionPlan->motionPlanDuration / theMotionPlan->nSamplePoints;

			for (int i=startQIndex; i<=endQIndex; i++){
				double velocity = (theMotionPlan->robotStateTrajectory.qArray[jp][i] - theMotionPlan->robotStateTrajectory.qArray[jm][i]) / dt;

				if (theMotionPlan->robotStatesParamsStartIndex >= 0){

					// lower bound hessian
					// d_jm_jm
					addMTripletToList_reflectUpperElements(
								hessianEntries,
								theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + i,
								theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + i,
								weight * constraintSymmetricBound->computeSecondDerivative(velocity) / (dt*dt));
					// d_jp_jp
					addMTripletToList_reflectUpperElements(
								hessianEntries,
								theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + i,
								theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + i,
								weight * constraintSymmetricBound->computeSecondDerivative(velocity) / (dt*dt));
					// d_jm_jp
					addMTripletToList_reflectUpperElements(
								hessianEntries,
								theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + i,
								theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + i,
								- weight * constraintSymmetricBound->computeSecondDerivative(velocity) / (dt*dt));
				}
			}
		}
	}
}


