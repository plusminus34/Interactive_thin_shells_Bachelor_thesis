#include <RobotDesignerLib/MPO_FixedWheelObjective.h>

#include <MathLib/AutoDiff.h>

MPO_FixedWheelObjective::MPO_FixedWheelObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_FixedWheelObjective::~MPO_FixedWheelObjective(void){
}

double MPO_FixedWheelObjective::computeValue(const dVector& p){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	double retVal = 0;

	int end = theMotionPlan->nSamplePoints;
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1;

	int nEEs = theMotionPlan->endEffectorTrajectories.size();

	const double dt = theMotionPlan->motionPlanDuration / theMotionPlan->nSamplePoints;

	for (int j=0; j<end; j++){

		int jm, jp;
		theMotionPlan->getVelocityTimeIndicesFor(j, jm, jp);
		if (jm == -1 || jp == -1) continue;

		dVector qi; theMotionPlan->robotStateTrajectory.getQAtTimeIndex(j, qi);
		dVector qip; theMotionPlan->robotStateTrajectory.getQAtTimeIndex(jp, qip);

		for (int i=0;i<nEEs;i++){
			const LocomotionEngine_EndEffectorTrajectory &ee = theMotionPlan->endEffectorTrajectories[i];

			double yawAnglej =  ee.wheelYawAngle[j];
			double yawAnglejp =  ee.wheelYawAngle[jp];
			double tiltAnglej =  ee.wheelTiltAngle[j];
			double tiltAnglejp =  ee.wheelTiltAngle[jp];
			double wheelSpeedj =  ee.wheelSpeed[j];
			double wheelSpeedjp =  ee.wheelSpeed[jp];

			if(ee.isFixedWheel || ee.isWeldedWheel)
			{
				retVal += computeEnergy(ee.getWheelRhoLocal(), ee.wheelAxisLocal,
										ee.endEffectorRB, qi, qip,
										ee.wheelYawAxis, yawAnglej, yawAnglejp,
										ee.wheelTiltAxis, tiltAnglej, tiltAnglejp,
										wheelSpeedj, wheelSpeedjp, dt);
			}
		}
	}

	return retVal;
}

void MPO_FixedWheelObjective::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	// number of DOFs for an end effector at one time sample
	int numDOFs = 0;
	// 3 DOFs for yaw, tilt and speed, 2 time steps
	if (theMotionPlan->wheelParamsStartIndex >= 0)
		numDOFs += 3*2;
	// size of robot state `q` as DOFs, 2 time steps
	if (theMotionPlan->robotStatesParamsStartIndex >= 0)
		numDOFs += theMotionPlan->robotRepresentation->getDimensionCount()*2;

	int end = theMotionPlan->nSamplePoints;
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1;

	int nEEs = theMotionPlan->endEffectorTrajectories.size();

	const ScalarDiff dt = theMotionPlan->motionPlanDuration / theMotionPlan->nSamplePoints;

	for (int j=0; j<end; j++){

		int jm, jp;
		theMotionPlan->getVelocityTimeIndicesFor(j, jm, jp);
		if (jm == -1 || jp == -1) continue;

		VectorXT<ScalarDiff> qi; theMotionPlan->robotStateTrajectory.getQAtTimeIndex(j, qi);
		VectorXT<ScalarDiff> qip; theMotionPlan->robotStateTrajectory.getQAtTimeIndex(jp, qip);

		for (int i=0;i<nEEs;i++){
			const LocomotionEngine_EndEffectorTrajectory &ee = theMotionPlan->endEffectorTrajectories[i];

			if(ee.isFixedWheel || ee.isWeldedWheel)
			{
				ScalarDiff yawAnglej =  ee.wheelYawAngle[j];
				ScalarDiff yawAnglejp =  ee.wheelYawAngle[jp];
				ScalarDiff tiltAnglej =  ee.wheelTiltAngle[j];
				ScalarDiff tiltAnglejp =  ee.wheelTiltAngle[jp];
				ScalarDiff wheelSpeedj =  ee.wheelSpeed[j];
				ScalarDiff wheelSpeedjp =  ee.wheelSpeed[jp];

				std::vector<DOF<ScalarDiff>> dofs(numDOFs);
				int index = 0;
				if (theMotionPlan->wheelParamsStartIndex >= 0){
					dofs[index].v = &yawAnglej;
					dofs[index].i = theMotionPlan->getWheelYawAngleIndex(i, j);
					index++;
					dofs[index].v = &yawAnglejp;
					dofs[index].i = theMotionPlan->getWheelYawAngleIndex(i, jp);
					index++;

					dofs[index].v = &tiltAnglej;
					dofs[index].i = theMotionPlan->getWheelTiltAngleIndex(i, j);
					index++;
					dofs[index].v = &tiltAnglejp;
					dofs[index].i = theMotionPlan->getWheelTiltAngleIndex(i, jp);
					index++;

					dofs[index].v = &wheelSpeedj;
					dofs[index].i = theMotionPlan->getWheelSpeedIndex(i, j);
					index++;
					dofs[index].v = &wheelSpeedjp;
					dofs[index].i = theMotionPlan->getWheelSpeedIndex(i, jp);
					index++;
				}
				if (theMotionPlan->robotStatesParamsStartIndex >= 0){
					for (int k = 0; k < qi.size(); ++k){
						dofs[index].v = &qi[k];
						dofs[index].i = theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + k;
						index++;
						dofs[index].v = &qip[k];
						dofs[index].i = theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + k;
						index++;
					}
				}

				for (int k = 0; k < numDOFs; ++k) {
					dofs[k].v->deriv() = 1.0;
					ScalarDiff energy = computeEnergy((V3T<ScalarDiff>)ee.getWheelRhoLocal(), (V3T<ScalarDiff>)ee.wheelAxisLocal,
													  ee.endEffectorRB, qi, qip,
													  (V3T<ScalarDiff>)ee.wheelYawAxis, yawAnglej, yawAnglejp,
													  (V3T<ScalarDiff>)ee.wheelTiltAxis, tiltAnglej, tiltAnglejp,
													  wheelSpeedj, wheelSpeedjp, dt);
					grad[dofs[k].i] += energy.deriv();
					dofs[k].v->deriv() = 0.0;
				}
			}
		}
	}
}

void MPO_FixedWheelObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	// number of DOFs for an end effector at one time sample
	int numDOFs = 0;
	// 3 DOFs for yaw, tilt and speed, 2 time steps
	if (theMotionPlan->wheelParamsStartIndex >= 0)
		numDOFs += 3*2;
	// size of robot state `q` as DOFs, 2 time steps
	if (theMotionPlan->robotStatesParamsStartIndex >= 0)
		numDOFs += theMotionPlan->robotRepresentation->getDimensionCount()*2;

	int end = theMotionPlan->nSamplePoints;
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1;

	int nEEs = theMotionPlan->endEffectorTrajectories.size();

	const ScalarDiffDiff dt = theMotionPlan->motionPlanDuration / theMotionPlan->nSamplePoints;

	for (int j=0; j<end; j++){

		int jm, jp;
		theMotionPlan->getVelocityTimeIndicesFor(j, jm, jp);
		if (jm == -1 || jp == -1) continue;

		VectorXT<ScalarDiffDiff> qi; theMotionPlan->robotStateTrajectory.getQAtTimeIndex(j, qi);
		VectorXT<ScalarDiffDiff> qip; theMotionPlan->robotStateTrajectory.getQAtTimeIndex(jp, qip);

		for (int i=0;i<nEEs;i++){
			const LocomotionEngine_EndEffectorTrajectory &ee = theMotionPlan->endEffectorTrajectories[i];

			if(ee.isFixedWheel || ee.isWeldedWheel)
			{
				ScalarDiffDiff yawAnglej =  ee.wheelYawAngle[j];
				ScalarDiffDiff yawAnglejp =  ee.wheelYawAngle[jp];
				ScalarDiffDiff tiltAnglej =  ee.wheelTiltAngle[j];
				ScalarDiffDiff tiltAnglejp =  ee.wheelTiltAngle[jp];
				ScalarDiffDiff wheelSpeedj =  ee.wheelSpeed[j];
				ScalarDiffDiff wheelSpeedjp =  ee.wheelSpeed[jp];

				std::vector<DOF<ScalarDiffDiff>> dofs(numDOFs);
				int index = 0;
				if (theMotionPlan->wheelParamsStartIndex >= 0){
					dofs[index].v = &yawAnglej;
					dofs[index].i = theMotionPlan->getWheelYawAngleIndex(i, j);
					index++;
					dofs[index].v = &yawAnglejp;
					dofs[index].i = theMotionPlan->getWheelYawAngleIndex(i, jp);
					index++;

					dofs[index].v = &tiltAnglej;
					dofs[index].i = theMotionPlan->getWheelTiltAngleIndex(i, j);
					index++;
					dofs[index].v = &tiltAnglejp;
					dofs[index].i = theMotionPlan->getWheelTiltAngleIndex(i, jp);
					index++;

					dofs[index].v = &wheelSpeedj;
					dofs[index].i = theMotionPlan->getWheelSpeedIndex(i, j);
					index++;
					dofs[index].v = &wheelSpeedjp;
					dofs[index].i = theMotionPlan->getWheelSpeedIndex(i, jp);
					index++;
				}
				if (theMotionPlan->robotStatesParamsStartIndex >= 0){
					for (int k = 0; k < qi.size(); ++k){
						dofs[index].v = &qi[k];
						dofs[index].i = theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + k;
						index++;
						dofs[index].v = &qip[k];
						dofs[index].i = theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + k;
						index++;
					}
				}

				for (int k = 0; k < numDOFs; ++k) {
					dofs[k].v->deriv().value() = 1.0;
					for (int l = 0; l <= k; ++l) {
						dofs[l].v->value().deriv() = 1.0;
						ScalarDiffDiff energy = computeEnergy((V3T<ScalarDiffDiff>)ee.getWheelRhoLocal(), (V3T<ScalarDiffDiff>)ee.wheelAxisLocal,
														  ee.endEffectorRB, qi, qip,
														  (V3T<ScalarDiffDiff>)ee.wheelYawAxis, yawAnglej, yawAnglejp,
														  (V3T<ScalarDiffDiff>)ee.wheelTiltAxis, tiltAnglej, tiltAnglejp,
														  wheelSpeedj, wheelSpeedjp, dt);
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
}
