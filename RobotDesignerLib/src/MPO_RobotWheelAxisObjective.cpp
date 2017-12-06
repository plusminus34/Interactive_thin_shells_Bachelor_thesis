#include <RobotDesignerLib/MPO_RobotWheelAxisObjective.h>

#include <MathLib/AutoDiff.h>

MPO_RobotWheelAxisObjective::MPO_RobotWheelAxisObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_RobotWheelAxisObjective::~MPO_RobotWheelAxisObjective(void){
}

double MPO_RobotWheelAxisObjective::computeValue(const dVector& p){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	double retVal = 0;
	int nEEs = theMotionPlan->endEffectorTrajectories.size();
	for (int j=0;j<theMotionPlan->nSamplePoints;j++){
		dVector q_t;
		theMotionPlan->robotStateTrajectory.getQAtTimeIndex(j, q_t);

		for (int i=0;i<nEEs;i++){
			const LocomotionEngine_EndEffectorTrajectory &ee = theMotionPlan->endEffectorTrajectories[i];

			retVal += computeEnergy(ee.wheelAxis, ee.endEffectorLocalCoords,
									ee.endEffectorRB, q_t,
									ee.wheelYawAxis, ee.wheelYawAngle[j],
									ee.wheelTiltAxis, ee.wheelTiltAngle[j]);
		}
	}

	return retVal;
}

void MPO_RobotWheelAxisObjective::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	// number of DOFs for an end effector at one time sample
	int numDOFs = 0;
	// 2 DOFs for yaw and tilt angle
	if (theMotionPlan->wheelParamsStartIndex >= 0)
		numDOFs += 2;
	// size of robot state `q` as DOFs
	if (theMotionPlan->robotStatesParamsStartIndex >= 0)
		numDOFs += theMotionPlan->robotRepresentation->getDimensionCount();

	int nLimbs = theMotionPlan->endEffectorTrajectories.size();
	for (int j=0; j<theMotionPlan->nSamplePoints; j++){
		dVector qd;
		theMotionPlan->robotStateTrajectory.getQAtTimeIndex(j, qd);
		VectorXT<ScalarDiff> q(qd.size());
		for (int k = 0; k < qd.size(); ++k)
			q[k] = qd[k];

		for (int i=0;i<nLimbs;i++){
			const LocomotionEngine_EndEffectorTrajectory &ee = theMotionPlan->endEffectorTrajectories[i];

			V3T<ScalarDiff> eePosLocal = ee.endEffectorLocalCoords;
			V3T<ScalarDiff> wheelAxis = ee.wheelAxis;

			ScalarDiff yawAngle = ee.wheelYawAngle[j];
			V3T<ScalarDiff> yawAxis = ee.wheelYawAxis;
			ScalarDiff tiltAngle = ee.wheelTiltAngle[j];
			V3T<ScalarDiff> tiltAxis = ee.wheelTiltAxis;

			std::vector<DOF<ScalarDiff>> dofs(numDOFs);
			int index = 0;
			if (theMotionPlan->wheelParamsStartIndex >= 0){
				dofs[index].v = &yawAngle;
				dofs[index].i = theMotionPlan->getWheelYawAngleIndex(i, j);
				index++;
				dofs[index].v = &tiltAngle;
				dofs[index].i = theMotionPlan->getWheelTiltAngleIndex(i, j);
				index++;
			}
			if (theMotionPlan->robotStatesParamsStartIndex >= 0){
				for (int k = 0; k < q.size(); ++k){
					dofs[index].v = &q[k];
					dofs[index].i = theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + k;
					index++;
				}
			}

			for (int k = 0; k < numDOFs; ++k) {
				dofs[k].v->deriv() = 1.0;
				ScalarDiff energy = computeEnergy(wheelAxis, eePosLocal,
												  ee.endEffectorRB, q,
												  yawAxis, yawAngle,
												  tiltAxis, tiltAngle);
				grad[dofs[k].i] += energy.deriv();
				dofs[k].v->deriv() = 0.0;
			}
		}
	}
}

void MPO_RobotWheelAxisObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	// number of DOFs for an end effector at one time sample
	int numDOFs = 0;
	// 2 DOFs for yaw and tilt angle
	if (theMotionPlan->wheelParamsStartIndex >= 0)
		numDOFs += 2;
	// size of robot state `q` as DOFs
	if (theMotionPlan->robotStatesParamsStartIndex >= 0)
		numDOFs += theMotionPlan->robotRepresentation->getDimensionCount();

	int nLimbs = theMotionPlan->endEffectorTrajectories.size();
	for (int j=0; j<theMotionPlan->nSamplePoints; j++){
		dVector qd;
		theMotionPlan->robotStateTrajectory.getQAtTimeIndex(j, qd);
		VectorXT<ScalarDiffDiff> q(qd.size());
		for (int k = 0; k < qd.size(); ++k)
			q[k] = qd[k];

		for (int i=0;i<nLimbs;i++){
			const LocomotionEngine_EndEffectorTrajectory &ee = theMotionPlan->endEffectorTrajectories[i];

			V3T<ScalarDiffDiff> eePosLocal = ee.endEffectorLocalCoords;
			V3T<ScalarDiffDiff> wheelAxis = ee.wheelAxis;

			ScalarDiffDiff yawAngle = ee.wheelYawAngle[j];
			V3T<ScalarDiffDiff> yawAxis = ee.wheelYawAxis;
			ScalarDiffDiff tiltAngle = ee.wheelTiltAngle[j];
			V3T<ScalarDiffDiff> tiltAxis = ee.wheelTiltAxis;

			std::vector<DOF<ScalarDiffDiff>> dofs(numDOFs);
			int index = 0;
			if (theMotionPlan->wheelParamsStartIndex >= 0){
				dofs[index].v = &yawAngle;
				dofs[index].i = theMotionPlan->getWheelYawAngleIndex(i, j);
				index++;
				dofs[index].v = &tiltAngle;
				dofs[index].i = theMotionPlan->getWheelTiltAngleIndex(i, j);
				index++;
			}
			if (theMotionPlan->robotStatesParamsStartIndex >= 0){
				for (int k = 0; k < q.size(); ++k){
					dofs[index].v = &q[k];
					dofs[index].i = theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + k;
					index++;
				}
			}

			for (int k = 0; k < numDOFs; ++k) {
				dofs[k].v->deriv().value() = 1.0;
				for (int l = 0; l <= k; ++l) {
					dofs[l].v->value().deriv() = 1.0;
					ScalarDiffDiff energy = computeEnergy(wheelAxis, eePosLocal,
													  ee.endEffectorRB, q,
													  yawAxis, yawAngle,
													  tiltAxis, tiltAngle);
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
