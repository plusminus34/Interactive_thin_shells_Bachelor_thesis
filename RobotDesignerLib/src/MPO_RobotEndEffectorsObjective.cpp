#include <RobotDesignerLib/MPO_RobotEndEffectorsObjective.h>

MPO_RobotEndEffectorsObjective::MPO_RobotEndEffectorsObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_RobotEndEffectorsObjective::~MPO_RobotEndEffectorsObjective(void){
}

double MPO_RobotEndEffectorsObjective::computeValue(const dVector& p){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	double retVal = 0;
	int nLimbs = theMotionPlan->endEffectorTrajectories.size();
	for (int j=0;j<theMotionPlan->nSamplePoints;j++){
		VectorXT<double> q_t;
		theMotionPlan->robotStateTrajectory.getQAtTimeIndex(j, q_t);

		for (int i=0;i<nLimbs;i++){
			const auto &ee = theMotionPlan->endEffectorTrajectories[i];
			retVal += computeEnergy<double>(ee.EEPos[j], ee.endEffectorLocalCoords, q_t, ee.endEffectorRB);
		}
	}

	return retVal;
}

void MPO_RobotEndEffectorsObjective::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	// number of DOFs for an end effector at one time sample
	int numDOFs = 0;
	// 3 DOFs for eePos
	if (theMotionPlan->feetPositionsParamsStartIndex >= 0)
		numDOFs += 3;
	// size of robot state `q` as DOFs
	if (theMotionPlan->robotStatesParamsStartIndex >= 0)
		numDOFs += theMotionPlan->robotRepresentation->getDimensionCount();

	int nLimbs = theMotionPlan->endEffectorTrajectories.size();
	for (int j=0;j<theMotionPlan->nSamplePoints;j++){
		dVector q_t;
		theMotionPlan->robotStateTrajectory.getQAtTimeIndex(j, q_t);
		VectorXT<ScalarDiff> qAD(q_t.size());
		for (int k = 0; k < q_t.size(); ++k)
			qAD[k] = q_t[k];

		for (int i=0;i<nLimbs;i++){

			const LocomotionEngine_EndEffectorTrajectory &ee = theMotionPlan->endEffectorTrajectories[i];
			Vector3T<ScalarDiff> eePosLocal;
			for (int k = 0; k < 3; ++k)
				eePosLocal[k] = ee.endEffectorLocalCoords[k];

			Vector3T<ScalarDiff> eePos;
			for (int k = 0; k < 3; ++k)
				eePos[k] = ee.EEPos[j][k];


			std::vector<DOF<ScalarDiff>> dofs(numDOFs);
			int index = 0;
			if (theMotionPlan->feetPositionsParamsStartIndex >= 0){
				for (int k = 0; k < 3; ++k) {
					dofs[index].v = &eePos[k];
					dofs[index].i = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + k;
					index++;
				}
			}
			if (theMotionPlan->robotStatesParamsStartIndex >= 0){
				for (int k = 0; k < qAD.size(); ++k){
					dofs[index].v = &qAD[k];
					dofs[index].i = theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + k;
					index++;
				}
			}

			for (int k = 0; k < numDOFs; ++k) {
				dofs[k].v->deriv() = 1.0;
				ScalarDiff energy = computeEnergy<ScalarDiff>(eePos, eePosLocal, qAD, ee.endEffectorRB);
				grad[dofs[k].i] += energy.deriv();
				dofs[k].v->deriv() = 0.0;
			}
		}
	}

}

void MPO_RobotEndEffectorsObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	// number of DOFs for an end effector at one time sample
	int numDOFs = 0;
	// 3 DOFs for eePos
	if (theMotionPlan->feetPositionsParamsStartIndex >= 0)
		numDOFs += 3;
	// size of robot state `q` as DOFs
	if (theMotionPlan->robotStatesParamsStartIndex >= 0)
		numDOFs += theMotionPlan->robotRepresentation->getDimensionCount();

	int nLimbs = theMotionPlan->endEffectorTrajectories.size();
	for (int j=0;j<theMotionPlan->nSamplePoints;j++){
		dVector q_t;
		theMotionPlan->robotStateTrajectory.getQAtTimeIndex(j, q_t);
		VectorXT<ScalarDiffDiff> qAD(q_t.size());
		for (int k = 0; k < q_t.size(); ++k)
			qAD[k] = q_t[k];

		for (int i=0;i<nLimbs;i++){
			const LocomotionEngine_EndEffectorTrajectory &ee = theMotionPlan->endEffectorTrajectories[i];
			Vector3T<ScalarDiffDiff> eePosLocal;
			for (int k = 0; k < 3; ++k)
				eePosLocal[k] = ee.endEffectorLocalCoords[k];

			Vector3T<ScalarDiffDiff> eePos;
			for (int k = 0; k < 3; ++k)
				eePos[k] = ee.EEPos[j][k];


			std::vector<DOF<ScalarDiffDiff>> dofs(numDOFs);
			int index = 0;
			if (theMotionPlan->feetPositionsParamsStartIndex >= 0){
				for (int k = 0; k < 3; ++k) {
					dofs[index].v = &eePos[k];
					dofs[index].i = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + k;
					index++;
				}
			}
			if (theMotionPlan->robotStatesParamsStartIndex >= 0){
				for (int k = 0; k < qAD.size(); ++k){
					dofs[index].v = &qAD[k];
					dofs[index].i = theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + k;
					index++;
				}
			}

			for (int k = 0; k < numDOFs; ++k) {
				dofs[k].v->deriv().value() = 1.0;
				for (int l = 0; l <= k; ++l) {
					dofs[l].v->value().deriv() = 1.0;
					ScalarDiffDiff energy = computeEnergy<ScalarDiffDiff>(eePos, eePosLocal, qAD, ee.endEffectorRB);
					ADD_HES_ELEMENT(hessianEntries,
									dofs[k].i,
									dofs[l].i,
									energy.deriv().deriv(), 1.0);
					dofs[l].v->value().deriv() = 0.0;
				}
				dofs[k].v->deriv().value() = 0.0;
			}
		}
	}
}
