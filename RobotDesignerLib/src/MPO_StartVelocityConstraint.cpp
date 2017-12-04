#include <RobotDesignerLib/MPO_StartVelocityConstraint.h>
#include <iostream>

MPO_COMZeroVelocityConstraint::MPO_COMZeroVelocityConstraint(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, int timeIndex, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
	this->timeIndex = timeIndex;
}

MPO_COMZeroVelocityConstraint::~MPO_COMZeroVelocityConstraint(void) {
}

double MPO_COMZeroVelocityConstraint::computeValue(const dVector& s) {

	double retVal = 0;

	int j = timeIndex;
	int jp = (timeIndex == theMotionPlan->nSamplePoints-1) ? 0 : timeIndex+1;

	V3T<double> comPos0 = theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(j);
	V3T<double> comPos1 = theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(jp);

	retVal = computeEnergy(comPos0, comPos1);

	return retVal;
}

void MPO_COMZeroVelocityConstraint::addGradientTo(dVector& grad, const dVector& p) {

	if (theMotionPlan->COMPositionsParamsStartIndex >= 0){

		int j = timeIndex;
		int jp = (timeIndex == theMotionPlan->nSamplePoints-1) ? 0 : timeIndex+1;

		V3T<ScalarDiff> comPos0 = theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(j);
		V3T<ScalarDiff> comPos1 = theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(jp);

		std::vector<DOF<ScalarDiff>> dofs(6);
		int index = 0;
		for (int k = 0; k < 3; ++k) {
			dofs[index].v = &comPos0[k];
			dofs[index].i = theMotionPlan->COMPositionsParamsStartIndex + 3 * j + k;
			index++;
		}

		for (int k = 0; k < 3; ++k) {
			dofs[index].v = &comPos1[k];
			dofs[index].i = theMotionPlan->COMPositionsParamsStartIndex + 3 * jp + k;
			index++;
		}

		for (int k = 0; k < 6; ++k) {
			dofs[k].v->deriv() = 1.0;
			ScalarDiff energy = computeEnergy(comPos0, comPos1);
			grad[dofs[k].i] += energy.deriv();
			dofs[k].v->deriv() = 0.0;
		}
	}
}

void MPO_COMZeroVelocityConstraint::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {

	if (theMotionPlan->COMPositionsParamsStartIndex >= 0){

		int j = timeIndex;
		int jp = (timeIndex == theMotionPlan->nSamplePoints-1) ? 0 : timeIndex+1;

		V3T<ScalarDiffDiff> comPos0 = theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(j);
		V3T<ScalarDiffDiff> comPos1 = theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(jp);

		std::vector<DOF<ScalarDiffDiff>> dofs(6);
		int index = 0;
		for (int k = 0; k < 3; ++k) {
			dofs[index].v = &comPos0[k];
			dofs[index].i = theMotionPlan->COMPositionsParamsStartIndex + 3 * j + k;
			index++;
		}

		for (int k = 0; k < 3; ++k) {
			dofs[index].v = &comPos1[k];
			dofs[index].i = theMotionPlan->COMPositionsParamsStartIndex + 3 * jp + k;
			index++;
		}

		for (int k = 0; k < 6; ++k) {
			dofs[k].v->deriv().value() = 1.0;
			for (int l = 0; l <= k; ++l) {
				dofs[l].v->value().deriv() = 1.0;
				ScalarDiffDiff energy = computeEnergy(comPos0, comPos1);
				ADD_HES_ELEMENT(hessianEntries,
								dofs[k].i,
								dofs[l].i,
								energy.deriv().deriv(),
								1.0);

				dofs[l].v->value().deriv() = 0.0;
			}
			dofs[k].v->deriv().value() = 0.0;
		}
	}
}


