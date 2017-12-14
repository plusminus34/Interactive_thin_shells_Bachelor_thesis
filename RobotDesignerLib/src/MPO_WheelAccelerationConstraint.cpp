#include <RobotDesignerLib/MPO_WheelAccelerationConstraint.h>

MPO_WheelAccelerationConstraints::MPO_WheelAccelerationConstraints(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {

	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;

	constraintLowerBound = std::shared_ptr<SoftUnilateralConstraint>(new SoftUnilateralConstraint(-theMotionPlan->wheelAccelLimit, 10, theMotionPlan->wheelAccelEpsilon));
	constraintUpperBound = std::shared_ptr<SoftUnilateralUpperConstraint>(new SoftUnilateralUpperConstraint(theMotionPlan->wheelAccelLimit, 10, theMotionPlan->wheelAccelEpsilon));
}

MPO_WheelAccelerationConstraints::~MPO_WheelAccelerationConstraints(void) {
}

double MPO_WheelAccelerationConstraints::computeValue(const dVector& s) {

	if (theMotionPlan->wheelParamsStartIndex < 0)
		return 0;

	constraintLowerBound->setLimit(-theMotionPlan->wheelAccelLimit);
	constraintUpperBound->setLimit(theMotionPlan->wheelAccelLimit);
	constraintLowerBound->setEpsilon(theMotionPlan->wheelAccelEpsilon);
	constraintUpperBound->setEpsilon(theMotionPlan->wheelAccelEpsilon);

	double retVal = 0;

	int nSamplePoints = theMotionPlan->nSamplePoints;
	// don't double count... the last robot pose is already the same as the first one,
	// which means that COM and feet locations are in correct locations relative to each other,
	// so no need to ask for that again explicitely...
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) nSamplePoints -= 1;

	double dt = theMotionPlan->motionPlanDuration / theMotionPlan->nSamplePoints;

	for (int j=0; j<nSamplePoints; j++){

		int jm, jp;
		theMotionPlan->getVelocityTimeIndicesFor(j, jm, jp);
		if (jm == -1 || jp == -1) continue;

		for (uint i=0; i<theMotionPlan->endEffectorTrajectories.size(); i++){
			const LocomotionEngine_EndEffectorTrajectory &ee = theMotionPlan->endEffectorTrajectories[i];

			if(ee.isWheel){
				double wheelSpeedjm = ee.wheelSpeed[jm];
				double wheelSpeedjp = ee.wheelSpeed[jp];
				double wheelAccel = (wheelSpeedjp-wheelSpeedjm) / dt;
				retVal += constraintLowerBound->computeValue(wheelAccel);
				retVal += constraintUpperBound->computeValue(wheelAccel);
			}
		}
	}

	return retVal * weight;
}

void MPO_WheelAccelerationConstraints::addGradientTo(dVector& grad, const dVector& p) {

	if (theMotionPlan->wheelParamsStartIndex >= 0){

		constraintLowerBound->setLimit(-theMotionPlan->wheelAccelLimit);
		constraintUpperBound->setLimit(theMotionPlan->wheelAccelLimit);
		constraintLowerBound->setEpsilon(theMotionPlan->wheelAccelEpsilon);
		constraintUpperBound->setEpsilon(theMotionPlan->wheelAccelEpsilon);

		int nSamplePoints = theMotionPlan->nSamplePoints;
		// don't double count... the last robot pose is already the same as the first one,
		// which means that COM and feet locations are in correct locations relative to each other,
		// so no need to ask for that again explicitely...
		if (theMotionPlan->wrapAroundBoundaryIndex >= 0) nSamplePoints -= 1;

		double dt = theMotionPlan->motionPlanDuration / theMotionPlan->nSamplePoints;

		for (int j=0; j<theMotionPlan->nSamplePoints; j++){

			int jm, jp;
			theMotionPlan->getVelocityTimeIndicesFor(j, jm, jp);
			if (jm == -1 || jp == -1) continue;

			for (uint i=0; i<theMotionPlan->endEffectorTrajectories.size(); i++){

				const LocomotionEngine_EndEffectorTrajectory &ee = theMotionPlan->endEffectorTrajectories[i];

				if(ee.isWheel){
					double wheelSpeedjm = ee.wheelSpeed[jm];
					double wheelSpeedjp = ee.wheelSpeed[jp];
					double wheelAccel = (wheelSpeedjp-wheelSpeedjm) / dt;

					// lower bound gradient
					grad[theMotionPlan->getWheelSpeedIndex(i,jm)] -= weight * constraintLowerBound->computeDerivative(wheelAccel) / dt;
					grad[theMotionPlan->getWheelSpeedIndex(i,jp)] += weight * constraintLowerBound->computeDerivative(wheelAccel) / dt;

					// upper bound gradient
					grad[theMotionPlan->getWheelSpeedIndex(i,jm)] -= weight * constraintUpperBound->computeDerivative(wheelAccel) / dt;
					grad[theMotionPlan->getWheelSpeedIndex(i,jp)] += weight * constraintUpperBound->computeDerivative(wheelAccel) / dt;
				}
			}
		}
	}
}

void MPO_WheelAccelerationConstraints::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {

	if (theMotionPlan->wheelParamsStartIndex >= 0){

		constraintLowerBound->setLimit(-theMotionPlan->wheelAccelLimit);
		constraintUpperBound->setLimit(theMotionPlan->wheelAccelLimit);
		constraintLowerBound->setEpsilon(theMotionPlan->wheelAccelEpsilon);
		constraintUpperBound->setEpsilon(theMotionPlan->wheelAccelEpsilon);

		int nSamplePoints = theMotionPlan->nSamplePoints;
		// don't double count... the last robot pose is already the same as the first one,
		// which means that COM and feet locations are in correct locations relative to each other,
		// so no need to ask for that again explicitely...
		if (theMotionPlan->wrapAroundBoundaryIndex >= 0) nSamplePoints -= 1;

		double dt = theMotionPlan->motionPlanDuration / theMotionPlan->nSamplePoints;

		for (int j=0; j<theMotionPlan->nSamplePoints; j++){

			int jm, jp;
			theMotionPlan->getVelocityTimeIndicesFor(j, jm, jp);
			if (jm == -1 || jp == -1) continue;

			for (uint i=0; i<theMotionPlan->endEffectorTrajectories.size(); i++){

				const LocomotionEngine_EndEffectorTrajectory &ee = theMotionPlan->endEffectorTrajectories[i];

				if(ee.isWheel){

					double wheelSpeedjm = ee.wheelSpeed[jm];
					double wheelSpeedjp = ee.wheelSpeed[jp];
					double wheelAccel = (wheelSpeedjp-wheelSpeedjm) / dt;

					// lower bound hessian
					// djm_djm
					addMTripletToList_reflectUpperElements(
								hessianEntries,
								theMotionPlan->getWheelSpeedIndex(i,jm),
								theMotionPlan->getWheelSpeedIndex(i,jm),
								weight * constraintLowerBound->computeSecondDerivative(wheelAccel) / (dt*dt));

					// djm_djp
					addMTripletToList_reflectUpperElements(
								hessianEntries,
								theMotionPlan->getWheelSpeedIndex(i,jm),
								theMotionPlan->getWheelSpeedIndex(i,jp),
								-weight * constraintLowerBound->computeSecondDerivative(wheelAccel) / (dt*dt));

					// djp_djp
					addMTripletToList_reflectUpperElements(
								hessianEntries,
								theMotionPlan->getWheelSpeedIndex(i,jp),
								theMotionPlan->getWheelSpeedIndex(i,jp),
								weight * constraintLowerBound->computeSecondDerivative(wheelAccel) / (dt*dt));

					// upper bound hessian
					// djm_djm
					addMTripletToList_reflectUpperElements(
								hessianEntries,
								theMotionPlan->getWheelSpeedIndex(i,jm),
								theMotionPlan->getWheelSpeedIndex(i,jm),
								weight * constraintUpperBound->computeSecondDerivative(wheelAccel) / (dt*dt));

					// djm_djp
					addMTripletToList_reflectUpperElements(
								hessianEntries,
								theMotionPlan->getWheelSpeedIndex(i,jm),
								theMotionPlan->getWheelSpeedIndex(i,jp),
								-weight * constraintUpperBound->computeSecondDerivative(wheelAccel) / (dt*dt));

					// djp_djp
					addMTripletToList_reflectUpperElements(
								hessianEntries,
								theMotionPlan->getWheelSpeedIndex(i,jp),
								theMotionPlan->getWheelSpeedIndex(i,jp),
								weight * constraintUpperBound->computeSecondDerivative(wheelAccel) / (dt*dt));
				}
			}
		}
	}
}
