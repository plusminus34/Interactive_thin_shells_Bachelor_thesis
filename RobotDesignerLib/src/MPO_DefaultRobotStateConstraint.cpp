#include <RobotDesignerLib/MPO_DefaultRobotStateConstraint.h>
#include <iostream>

MPO_DefaultRobotStateConstraint::MPO_DefaultRobotStateConstraint(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, int timeIndex, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
	this->timeIndex = timeIndex;
}

MPO_DefaultRobotStateConstraint::~MPO_DefaultRobotStateConstraint(void) {
}

double MPO_DefaultRobotStateConstraint::computeValue(const dVector& s) {

	const VectorXT<double> &qqDefault = theMotionPlan->robotStateTrajectory.defaultRobotStates[timeIndex];
	const VectorXT<double> &qq = theMotionPlan->robotStateTrajectory.qArray[timeIndex];
	VectorXT<double> q(qq.size()-6);
	VectorXT<double> qDefault(qq.size()-6);
	for (int i = 6; i < qq.size(); ++i) {
		q[i-6] = qq[i];
		qDefault[i-6] = qqDefault[i];
	}

	double retVal = computeEnergy(qDefault, q);

	return retVal;
}

void MPO_DefaultRobotStateConstraint::addGradientTo(dVector& grad, const dVector& p) {

	if (theMotionPlan->robotStatesParamsStartIndex >= 0){

		int j = timeIndex;

		VXT<ScalarDiff> qqDefault = theMotionPlan->robotStateTrajectory.defaultRobotStates[timeIndex];
		VXT<ScalarDiff> qq = theMotionPlan->robotStateTrajectory.qArray[timeIndex];

		VectorXT<ScalarDiff> q(qq.size()-6);
		VectorXT<ScalarDiff> qDefault(qq.size()-6);
		for (int i = 6; i < qq.size(); ++i) {
			q[i-6] = qq[i];
			qDefault[i-6] = qqDefault[i];
		}


		std::vector<DOF<ScalarDiff>> dofs(q.size());
		int index = 0;
		for (int k = 0; k < q.size(); ++k) {
			dofs[index].v = &q[k];
			dofs[index].i = theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * j + k+6;
			index++;
		}

		for (int k = 0; k < dofs.size(); ++k) {
			dofs[k].v->deriv() = 1.0;
			ScalarDiff energy = computeEnergy(qDefault, q);
			grad[dofs[k].i] += energy.deriv();
			dofs[k].v->deriv() = 0.0;
		}
	}
}

void MPO_DefaultRobotStateConstraint::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {

	if (theMotionPlan->robotStatesParamsStartIndex >= 0){

		int j = timeIndex;

		VXT<ScalarDiffDiff> qqDefault = theMotionPlan->robotStateTrajectory.defaultRobotStates[timeIndex];
		VXT<ScalarDiffDiff> qq = theMotionPlan->robotStateTrajectory.qArray[timeIndex];
		VectorXT<ScalarDiffDiff> q(qq.size()-6);
		VectorXT<ScalarDiffDiff> qDefault(qq.size()-6);
		for (int i = 6; i < qq.size(); ++i) {
			q[i-6] = qq[i];
			qDefault[i-6] = qqDefault[i];
		}


		std::vector<DOF<ScalarDiffDiff>> dofs(q.size());
		int index = 0;
		for (int k = 0; k < q.size(); ++k) {
			dofs[index].v = &q[k];
			dofs[index].i = theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * j + k+6;
			index++;
		}

		for (int k = 0; k < dofs.size(); ++k) {
			dofs[k].v->deriv().value() = 1.0;
			for (int l = 0; l <= k; ++l) {
				dofs[l].v->value().deriv() = 1.0;
				ScalarDiffDiff energy = computeEnergy(qDefault, q);
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


