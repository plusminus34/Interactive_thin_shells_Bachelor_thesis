#include <RobotDesignerLib/MPO_JointsAnglesSoftConstraint.h>

MPO_JointsAnglesSoftConstraint::MPO_JointsAnglesSoftConstraint(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight, int startQIndex, int endQIndex) {

	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
	this->startQIndex = startQIndex;
	this->endQIndex = endQIndex;

	constraintSymmetricBound = std::make_unique<SoftSymmetricBarrierConstraint>(theMotionPlan->jointVelocityLimit);
}

MPO_JointsAnglesSoftConstraint::~MPO_JointsAnglesSoftConstraint(void) {
}

double MPO_JointsAnglesSoftConstraint::computeValue(const dVector& s) {
	constraintSymmetricBound->limit = theMotionPlan->jointAngleLimit;
	if (theMotionPlan->robotStatesParamsStartIndex < 0)
		return 0;

	constraintSymmetricBound->limit = theMotionPlan->jointAngleLimit;

	double retVal = 0;

	int nSamplePoints = theMotionPlan->nSamplePoints;

	Eigen::Map<const Eigen::MatrixXd> Q(s.data() + theMotionPlan->robotStatesParamsStartIndex, theMotionPlan->robotStateTrajectory.qArray[0].size(), nSamplePoints);
	MatrixNxM Qt = Q.bottomRows(endQIndex - startQIndex + 1).eval();
	Qt=Qt.unaryExpr([&](double val) {return constraintSymmetricBound->computeValue(val); }).eval();
	retVal = Qt.sum();
	return retVal * weight;
}

void MPO_JointsAnglesSoftConstraint::addGradientTo(dVector& grad, const dVector& p) {
	constraintSymmetricBound->limit = theMotionPlan->jointAngleLimit;
	if (theMotionPlan->robotStatesParamsStartIndex < 0)
		return;

	constraintSymmetricBound->limit = theMotionPlan->jointAngleLimit;

	int nSamplePoints = theMotionPlan->nSamplePoints;

	Eigen::Map<const Eigen::MatrixXd> Q(p.data() + theMotionPlan->robotStatesParamsStartIndex, theMotionPlan->robotStateTrajectory.qArray[0].size(), nSamplePoints);
	auto Qt = Q.bottomRows(endQIndex - startQIndex + 1);

	Eigen::Map<Eigen::MatrixXd> G(grad.data() + theMotionPlan->robotStatesParamsStartIndex, theMotionPlan->robotStateTrajectory.qArray[0].size(), nSamplePoints);
	auto Gt = G.bottomRows(endQIndex - startQIndex + 1);
	Gt += weight * Qt.unaryExpr([&](double val) {return constraintSymmetricBound->computeDerivative(val); });
}

void MPO_JointsAnglesSoftConstraint::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	constraintSymmetricBound->limit = theMotionPlan->jointAngleLimit;
	if (theMotionPlan->robotStatesParamsStartIndex < 0)
		return;

	constraintSymmetricBound->limit = theMotionPlan->jointAngleLimit;

	int nSamplePoints = theMotionPlan->nSamplePoints;

	for (int j=0; j<nSamplePoints-1; j++){
		for (int i=startQIndex; i<=endQIndex; i++){
			int I = theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + i;
			double Qij = theMotionPlan->robotStateTrajectory.qArray[j][i];
			
			// lower bound hessian
			// d_jm_jm
			ADD_HES_ELEMENT(hessianEntries, I, I, constraintSymmetricBound->computeSecondDerivative(Qij), weight);
		}
	}
}

