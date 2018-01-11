#include <RobotDesignerLib/MPO_VelocitySoftConstraints.h>

MPO_VelocitySoftBoundConstraints::MPO_VelocitySoftBoundConstraints(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight, int startQIndex, int endQIndex) {

	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
	this->startQIndex = startQIndex;
	this->endQIndex = endQIndex;

	constraintSymmetricBound = std::make_unique<SoftSymmetricBarrierConstraint>(theMotionPlan->jointVelocityLimit);
}

MPO_VelocitySoftBoundConstraints::~MPO_VelocitySoftBoundConstraints(void) {
}

double MPO_VelocitySoftBoundConstraints::computeValue(const dVector& s) {

	if (theMotionPlan->robotStatesParamsStartIndex < 0)
		return 0;

	constraintSymmetricBound->limit = theMotionPlan->jointVelocityLimit;

	double retVal = 0;

	int nSamplePoints = theMotionPlan->nSamplePoints;

	double dt = theMotionPlan->motionPlanDuration / (nSamplePoints - 1);

	for (int j=0; j<nSamplePoints-1; j++){
		int jp=j+1;

		for (int i=startQIndex; i<=endQIndex; i++){
			double velocity = (theMotionPlan->robotStateTrajectory.qArray[jp][i] - theMotionPlan->robotStateTrajectory.qArray[j][i]) / dt;
			retVal += constraintSymmetricBound->computeValue(velocity);
		}
	}

	return retVal * weight;
}

void MPO_VelocitySoftBoundConstraints::addGradientTo(dVector& grad, const dVector& p) {

	if (theMotionPlan->robotStatesParamsStartIndex < 0)
		return;

	constraintSymmetricBound->limit = theMotionPlan->jointVelocityLimit;

	int nSamplePoints = theMotionPlan->nSamplePoints;
	double dt = theMotionPlan->motionPlanDuration / (nSamplePoints - 1);


	for (int j=0; j<nSamplePoints-1; j++){

		int jp = j + 1;

		for (int i=startQIndex; i<=endQIndex; i++){
			double velocity = (theMotionPlan->robotStateTrajectory.qArray[jp][i] - theMotionPlan->robotStateTrajectory.qArray[j][i]) / dt;
			double dC = constraintSymmetricBound->computeDerivative(velocity);
				
			grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * j + i] -= weight * dC / dt;
			grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * jp + i] += weight * dC / dt;
		}
	}
}

void MPO_VelocitySoftBoundConstraints::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {

	if (theMotionPlan->robotStatesParamsStartIndex < 0)
		return;

	constraintSymmetricBound->limit = theMotionPlan->jointVelocityLimit;

	int nSamplePoints = theMotionPlan->nSamplePoints;
	double dt = theMotionPlan->motionPlanDuration / (nSamplePoints - 1);

	for (int j=0; j<nSamplePoints-1; j++){

		int jp = j + 1;

		for (int i=startQIndex; i<=endQIndex; i++){
			double velocity = (theMotionPlan->robotStateTrajectory.qArray[jp][i] - theMotionPlan->robotStateTrajectory.qArray[j][i]) / dt;

			// lower bound hessian
			// d_jm_jm
			addMTripletToList_reflectUpperElements(
						hessianEntries,
						theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + i,
						theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + i,
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
						theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + i,
						theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + i,
						- weight * constraintSymmetricBound->computeSecondDerivative(velocity) / (dt*dt));
		}
	}
}

