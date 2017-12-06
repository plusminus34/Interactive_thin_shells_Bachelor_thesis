#include <RobotDesignerLib/MPO_WheelSpeedConstraint.h>

MPO_WheelSpeedConstraints::MPO_WheelSpeedConstraints(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {

	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;

	constraintLowerBound = std::shared_ptr<SoftUnilateralConstraint>(new SoftUnilateralConstraint(-theMotionPlan->wheelSpeedLimit, 10, theMotionPlan->wheelSpeedEpsilon));
	constraintUpperBound = std::shared_ptr<SoftUnilateralUpperConstraint>(new SoftUnilateralUpperConstraint(theMotionPlan->wheelSpeedLimit, 10, theMotionPlan->wheelSpeedEpsilon));
}

MPO_WheelSpeedConstraints::~MPO_WheelSpeedConstraints(void) {
}

double MPO_WheelSpeedConstraints::computeValue(const dVector& s) {

	constraintLowerBound->setLimit(-theMotionPlan->wheelSpeedLimit);
	constraintUpperBound->setLimit(theMotionPlan->wheelSpeedLimit);
	constraintLowerBound->setEpsilon(theMotionPlan->wheelSpeedEpsilon);
	constraintUpperBound->setEpsilon(theMotionPlan->wheelSpeedEpsilon);

	double retVal = 0;

	for (int j=0; j<theMotionPlan->nSamplePoints; j++){
		for (uint i=0; i<theMotionPlan->endEffectorTrajectories.size(); i++){

			double wheelSpeed = theMotionPlan->endEffectorTrajectories[i].wheelSpeed[j];
			retVal += constraintLowerBound->computeValue(wheelSpeed);
			retVal += constraintUpperBound->computeValue(wheelSpeed);
		}
	}

	return retVal * weight;
}

void MPO_WheelSpeedConstraints::addGradientTo(dVector& grad, const dVector& p) {

	if (theMotionPlan->wheelParamsStartIndex >= 0){

		constraintLowerBound->setLimit(-theMotionPlan->wheelSpeedLimit);
		constraintUpperBound->setLimit(theMotionPlan->wheelSpeedLimit);
		constraintLowerBound->setEpsilon(theMotionPlan->wheelSpeedEpsilon);
		constraintUpperBound->setEpsilon(theMotionPlan->wheelSpeedEpsilon);

		for (int j=0; j<theMotionPlan->nSamplePoints; j++){
			for (uint i=0; i<theMotionPlan->endEffectorTrajectories.size(); i++){

				double wheelSpeed = theMotionPlan->endEffectorTrajectories[i].wheelSpeed[j];

				// lower bound gradient
				grad[theMotionPlan->getWheelSpeedIndex(i,j)] += weight * constraintLowerBound->computeDerivative(wheelSpeed);

				// upper bound gradient
				grad[theMotionPlan->getWheelSpeedIndex(i,j)] += weight * constraintUpperBound->computeDerivative(wheelSpeed);
			}
		}
	}
}

void MPO_WheelSpeedConstraints::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {

	if (theMotionPlan->wheelParamsStartIndex >= 0){

		constraintLowerBound->setLimit(-theMotionPlan->wheelSpeedLimit);
		constraintUpperBound->setLimit(theMotionPlan->wheelSpeedLimit);
		constraintLowerBound->setEpsilon(theMotionPlan->wheelSpeedEpsilon);
		constraintUpperBound->setEpsilon(theMotionPlan->wheelSpeedEpsilon);

		for (int j=0; j<theMotionPlan->nSamplePoints; j++){
			for (uint i=0; i<theMotionPlan->endEffectorTrajectories.size(); i++){

				double wheelSpeed = theMotionPlan->endEffectorTrajectories[i].wheelSpeed[j];

				// lower bound hessian
				addMTripletToList_reflectUpperElements(
							hessianEntries,
							theMotionPlan->getWheelSpeedIndex(i,j),
							theMotionPlan->getWheelSpeedIndex(i,j),
							weight * constraintLowerBound->computeSecondDerivative(wheelSpeed));

				// upper bound hessian
				addMTripletToList_reflectUpperElements(
							hessianEntries,
							theMotionPlan->getWheelSpeedIndex(i,j),
							theMotionPlan->getWheelSpeedIndex(i,j),
							weight * constraintUpperBound->computeSecondDerivative(wheelSpeed));
			}
		}
	}
}


