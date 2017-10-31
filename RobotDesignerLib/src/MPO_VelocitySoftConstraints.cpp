#include <RobotDesignerLib/MPO_VelocitySoftConstraints.h>
#include <iostream>

MPO_VelocitySoftBoundConstraints::MPO_VelocitySoftBoundConstraints(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight, int startQIndex, int endQIndex) {

	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
	this->startQIndex = startQIndex;
	this->endQIndex = endQIndex;

	constraintLowerBound = std::shared_ptr<SoftUnilateralConstraint>(new SoftUnilateralConstraint(-theMotionPlan->jointVelocityLimit, 10, theMotionPlan->jointVelocityEpsilon));
	constraintUpperBound = std::shared_ptr<SoftUnilateralUpperConstraint>(new SoftUnilateralUpperConstraint(theMotionPlan->jointVelocityLimit, 10, theMotionPlan->jointVelocityEpsilon));
}

MPO_VelocitySoftBoundConstraints::~MPO_VelocitySoftBoundConstraints(void) {
}

double MPO_VelocitySoftBoundConstraints::computeValue(const dVector& s) {

	if (theMotionPlan->robotStatesParamsStartIndex >= 0){

		constraintLowerBound->setLimit(-theMotionPlan->jointVelocityLimit);
		constraintUpperBound->setLimit(theMotionPlan->jointVelocityLimit);

		double retVal = 0;

		int nSamplePoints = theMotionPlan->nSamplePoints;
		if (theMotionPlan->wrapAroundBoundaryIndex >= 0) nSamplePoints -= 1; //don't double count... the last robot pose is already the same as the first one, which means that COM and feet locations are in correct locations relative to each other, so no need to ask for that again explicitely...

		for (int j=0; j<nSamplePoints; j++){

			int jm, jp;

			theMotionPlan->getVelocityTimeIndicesFor(j, jm, jp);
			if (jm == -1 || jp == -1) continue;

			double dt = theMotionPlan->motionPlanDuration / theMotionPlan->nSamplePoints;
			dt = 1.0;

			//we want the COM to be as close as possible to the weighted position of the grounded feet (sum_i c_i*w_i*p_i=com_p)
			for (int i=startQIndex; i<=endQIndex; i++){
				double velocity = (theMotionPlan->robotStateTrajectory.qArray[jp][i] - theMotionPlan->robotStateTrajectory.qArray[jm][i]) / dt;
				retVal += constraintLowerBound->computeValue(velocity);
				retVal += constraintUpperBound->computeValue(velocity);
			}
		}

		return retVal * weight;
	}

	return 0;
}

void MPO_VelocitySoftBoundConstraints::addGradientTo(dVector& grad, const dVector& p) {

	if (theMotionPlan->robotStatesParamsStartIndex >= 0){

		constraintLowerBound->setLimit(-theMotionPlan->jointVelocityLimit);
		constraintUpperBound->setLimit(theMotionPlan->jointVelocityLimit);

		int nSamplePoints = theMotionPlan->nSamplePoints;
		if (theMotionPlan->wrapAroundBoundaryIndex >= 0) nSamplePoints -= 1; //don't double count... the last robot pose is already the same as the first one, which means that COM and feet locations are in correct locations relative to each other, so no need to ask for that again explicitely...

		for (int j=0; j<nSamplePoints; j++){

			int jm, jp;

			theMotionPlan->getVelocityTimeIndicesFor(j, jm, jp);
			if (jm == -1 || jp == -1) continue;

			double dt = theMotionPlan->motionPlanDuration / theMotionPlan->nSamplePoints;
			dt = 1.0;

			//we want the COM to be as close as possible to the weighted position of the grounded feet (sum_i c_i*w_i*p_i=com_p)
			for (int i=startQIndex; i<=endQIndex; i++){
				double velocity = (theMotionPlan->robotStateTrajectory.qArray[jp][i] - theMotionPlan->robotStateTrajectory.qArray[jm][i]) / dt;

				// lower bound gradient
				grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * jm + i]
						-= weight * constraintLowerBound->computeDerivative(velocity) / dt;
				grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * jp + i]
						+= weight * constraintLowerBound->computeDerivative(velocity) / dt;

				// upper bound gradient
				grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * jm + i]
						-= weight * constraintUpperBound->computeDerivative(velocity) / dt;
				grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * jp + i]
						+= weight * constraintUpperBound->computeDerivative(velocity) / dt;
			}
		}
	}
}

void MPO_VelocitySoftBoundConstraints::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {

	if (theMotionPlan->robotStatesParamsStartIndex >= 0){

		constraintLowerBound->setLimit(-theMotionPlan->jointVelocityLimit);
		constraintUpperBound->setLimit(theMotionPlan->jointVelocityLimit);

		int nSamplePoints = theMotionPlan->nSamplePoints;
		if (theMotionPlan->wrapAroundBoundaryIndex >= 0) nSamplePoints -= 1; //don't double count... the last robot pose is already the same as the first one, which means that COM and feet locations are in correct locations relative to each other, so no need to ask for that again explicitely...

		for (int j=0; j<nSamplePoints; j++){

			int jm, jp;

			theMotionPlan->getVelocityTimeIndicesFor(j, jm, jp);
			if (jm == -1 || jp == -1) continue;

			double dt = theMotionPlan->motionPlanDuration / theMotionPlan->nSamplePoints;
			dt = 1.0;

			//we want the COM to be as close as possible to the weighted position of the grounded feet (sum_i c_i*w_i*p_i=com_p)
			for (int i=startQIndex; i<=endQIndex; i++){
				double velocity = (theMotionPlan->robotStateTrajectory.qArray[jp][i] - theMotionPlan->robotStateTrajectory.qArray[jm][i]) / dt;

				if (theMotionPlan->robotStatesParamsStartIndex >= 0){

					// lower bound hessian
					// d_jm_jm
					addMTripletToList_reflectUpperElements(
								hessianEntries,
								theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + i,
								theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + i,
								weight * constraintLowerBound->computeSecondDerivative(velocity) / (dt*dt));
					// d_jp_jp
					addMTripletToList_reflectUpperElements(
								hessianEntries,
								theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + i,
								theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + i,
								weight * constraintLowerBound->computeSecondDerivative(velocity) / (dt*dt));
					// d_jm_jp
					addMTripletToList_reflectUpperElements(
								hessianEntries,
								theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + i,
								theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + i,
								- weight * constraintLowerBound->computeSecondDerivative(velocity) / (dt*dt));

					// upper bound hessian
					// d_jm_jm
					addMTripletToList_reflectUpperElements(
								hessianEntries,
								theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + i,
								theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + i,
								weight * constraintUpperBound->computeSecondDerivative(velocity) / (dt*dt));
					// d_jp_jp
					addMTripletToList_reflectUpperElements(
								hessianEntries,
								theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + i,
								theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + i,
								weight * constraintUpperBound->computeSecondDerivative(velocity) / (dt*dt));
					// d_jm_jp
					addMTripletToList_reflectUpperElements(
								hessianEntries,
								theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + i,
								theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + i,
								- weight * constraintUpperBound->computeSecondDerivative(velocity) / (dt*dt));
				}
			}
		}
	}
}


