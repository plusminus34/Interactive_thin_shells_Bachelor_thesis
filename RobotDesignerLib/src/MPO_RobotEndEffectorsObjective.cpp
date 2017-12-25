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
		theMotionPlan->robotRepresentation->setQ(q_t);

		for (int i=0;i<nLimbs;i++){
			const LocomotionEngine_EndEffectorTrajectory &ee = theMotionPlan->endEffectorTrajectories[i];
			if(ee.isWheel){
				retVal += computeEnergyWheel<double>(ee.EEPos[j], ee.getWheelRho(),
													 ee.wheelYawAxis, ee.wheelYawAngle[j],
													 ee.wheelTiltAxis, ee.wheelTiltAngle[j],
													 ee.endEffectorLocalCoords, q_t, ee.endEffectorRB);
			}
			else{
				Vector3d err(ee.EEPos[j] - theMotionPlan->robotRepresentation->getWorldCoordinatesFor(ee.endEffectorLocalCoords, ee.endEffectorRB));
				retVal += 0.5 * weight * err.squaredNorm();
			}
		}
	}

	return retVal;
}

void MPO_RobotEndEffectorsObjective::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	MatrixNxM dEndEffectordq;

	// number of DOFs for an end effector at one time sample
	int numDOFs = 0;
	// 3 DOFs for eePos
	if (theMotionPlan->feetPositionsParamsStartIndex >= 0)
		numDOFs += 3;
	// size of robot state `q` as DOFs
	if (theMotionPlan->robotStatesParamsStartIndex >= 0)
		numDOFs += theMotionPlan->robotRepresentation->getDimensionCount();

	// 2 DOFs for yaw and tilt angle
	int numDOFsWheel = numDOFs;
	if(theMotionPlan->wheelParamsStartIndex >= 0)
		numDOFsWheel += 2;

	int nLimbs = theMotionPlan->endEffectorTrajectories.size();
	for (int j=0;j<theMotionPlan->nSamplePoints;j++){

		// get robot state at timestep j
		dVector q_t;
		theMotionPlan->robotStateTrajectory.getQAtTimeIndex(j, q_t);
		theMotionPlan->robotRepresentation->setQ(q_t);


		VectorXT<ScalarDiff> qAD(q_t.size());
		for (int k = 0; k < q_t.size(); ++k)
			qAD[k] = q_t[k];

		for (int i=0;i<nLimbs;i++){
			// This is the energy for end effector i at timestep j
			const LocomotionEngine_EndEffectorTrajectory &ee = theMotionPlan->endEffectorTrajectories[i];

			V3T<ScalarDiff> eePosLocal = ee.endEffectorLocalCoords;
			V3T<ScalarDiff> eePos = ee.EEPos[j];

			if(ee.isWheel){
				V3T<ScalarDiff> rho = ee.getWheelRho();
				V3T<ScalarDiff> yawAxis = ee.wheelYawAxis;
				V3T<ScalarDiff> tiltAxis = ee.wheelTiltAxis;
				ScalarDiff yawAngle = ee.wheelYawAngle[j];
				ScalarDiff tiltAngle = ee.wheelTiltAngle[j];

				std::vector<DOF<ScalarDiff>> dofs(numDOFsWheel);
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
				if (theMotionPlan->wheelParamsStartIndex >= 0){
					dofs[index].v = &yawAngle;
					dofs[index].i = theMotionPlan->getWheelYawAngleIndex(i, j);
					index++;
					dofs[index].v = &tiltAngle;
					dofs[index].i = theMotionPlan->getWheelTiltAngleIndex(i, j);
					index++;
				}

				for (int k = 0; k < numDOFsWheel; ++k) {
					dofs[k].v->deriv() = 1.0;
					ScalarDiff energy = computeEnergyWheel<ScalarDiff>(eePos, rho,
																	   yawAxis, yawAngle,
																	   tiltAxis, tiltAngle,
																	   eePosLocal, qAD, ee.endEffectorRB);
					grad[dofs[k].i] += energy.deriv();
					dofs[k].v->deriv() = 0.0;
				}
			}
			else
			{
				if (theMotionPlan->feetPositionsParamsStartIndex < 0)
					continue;

				Vector3d err(ee.EEPos[j] - theMotionPlan->robotRepresentation->getWorldCoordinatesFor(ee.endEffectorLocalCoords, ee.endEffectorRB));
				//compute the gradient with respect to the feet locations

				int ind = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3;
				grad.segment<3>(ind) += weight*err;

				//and now compute the gradient with respect to the robot q's
				theMotionPlan->robotRepresentation->compute_dpdq(ee.endEffectorLocalCoords, ee.endEffectorRB, dEndEffectordq);

				//dEdee * deedq = dEdq
				ind = theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim;
				grad.segment(ind, dEndEffectordq.cols()) += -weight*dEndEffectordq.transpose()*err;
			}
		}
	}
}

void MPO_RobotEndEffectorsObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);
	MatrixNxM dEndEffectordq, ddEndEffectordq_dqi;

	// number of DOFs for an end effector at one time sample
	int numDOFs = 0;
	// 3 DOFs for eePos
	if (theMotionPlan->feetPositionsParamsStartIndex >= 0)
		numDOFs += 3;
	// size of robot state `q` as DOFs
	if (theMotionPlan->robotStatesParamsStartIndex >= 0)
		numDOFs += theMotionPlan->robotRepresentation->getDimensionCount();

	// 2 DOFs for yaw and tilt angle
	int numDOFsWheel = numDOFs;
	if(theMotionPlan->wheelParamsStartIndex >= 0)
		numDOFsWheel += 2;

	int nLimbs = theMotionPlan->endEffectorTrajectories.size();
	for (int j=0;j<theMotionPlan->nSamplePoints;j++){
		dVector q_t;
		theMotionPlan->robotStateTrajectory.getQAtTimeIndex(j, q_t);
		theMotionPlan->robotRepresentation->setQ(q_t);

		VectorXT<ScalarDiffDiff> qAD(q_t.size());
		for (int k = 0; k < q_t.size(); ++k)
			qAD[k] = q_t[k];

		for (int i=0;i<nLimbs;i++){
			const LocomotionEngine_EndEffectorTrajectory &ee = theMotionPlan->endEffectorTrajectories[i];

			if(ee.isWheel){
				V3T<ScalarDiffDiff> eePosLocal = ee.endEffectorLocalCoords;
				V3T<ScalarDiffDiff> eePos = ee.EEPos[j];
				V3T<ScalarDiffDiff> rho = ee.getWheelRho();
				V3T<ScalarDiffDiff> yawAxis = ee.wheelYawAxis;
				V3T<ScalarDiffDiff> tiltAxis = ee.wheelTiltAxis;
				ScalarDiffDiff yawAngle = ee.wheelYawAngle[j];
				ScalarDiffDiff tiltAngle = ee.wheelTiltAngle[j];

				std::vector<DOF<ScalarDiffDiff>> dofs(numDOFsWheel);
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
				if (theMotionPlan->wheelParamsStartIndex >= 0){
					dofs[index].v = &yawAngle;
					dofs[index].i = theMotionPlan->getWheelYawAngleIndex(i, j);
					index++;
					dofs[index].v = &tiltAngle;
					dofs[index].i = theMotionPlan->getWheelTiltAngleIndex(i, j);
					index++;
				}

				MatrixNxM localH(numDOFsWheel, numDOFsWheel);

				for (int k = 0; k < numDOFsWheel; ++k) {
					dofs[k].v->deriv().value() = 1.0;
					for (int l = 0; l <= k; ++l) {
						dofs[l].v->value().deriv() = 1.0;
						ScalarDiffDiff energy = computeEnergyWheel<ScalarDiffDiff>(eePos, rho,
							yawAxis, yawAngle,
							tiltAxis, tiltAngle,
							eePosLocal, qAD, ee.endEffectorRB);
						localH(k, l) = energy.deriv().deriv();
						dofs[l].v->value().deriv() = 0.0;
					}
					dofs[k].v->deriv().value() = 0.0;
				}

				Eigen::SelfAdjointEigenSolver<MatrixNxM> es(localH);
				Eigen::VectorXd D = es.eigenvalues();
				Eigen::MatrixXd U = es.eigenvectors();
				D = D.unaryExpr([](double x) {return (x < 1e-4) ? 1e-4 : x; });
				localH = U * D.asDiagonal()*U.transpose();

				for (int k = 0; k < numDOFsWheel; ++k) 
					for (int l = 0; l <= k; ++l)
						ADD_HES_ELEMENT(hessianEntries,
							dofs[k].i,
							dofs[l].i,
							localH(k,l), 1.0);
			}
			else{
				Vector3d err(theMotionPlan->robotRepresentation->getWorldCoordinatesFor(ee.endEffectorLocalCoords, ee.endEffectorRB) - ee.EEPos[j]);
				//compute the gradient with respect to the feet locations
				if (theMotionPlan->feetPositionsParamsStartIndex >= 0) {
					int I = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3;
					ADD_HES_ELEMENT(hessianEntries, I    , I    , 1, weight);
					ADD_HES_ELEMENT(hessianEntries, I + 1, I + 1, 1, weight);
					ADD_HES_ELEMENT(hessianEntries, I + 2, I + 2, 1, weight);
				}
				//and now compute the gradient with respect to the robot q's
				if (theMotionPlan->robotStatesParamsStartIndex >= 0) {
					theMotionPlan->robotRepresentation->compute_dpdq(ee.endEffectorLocalCoords, ee.endEffectorRB, dEndEffectordq);
					int I = theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim;
// 					for (int k = 0; k < theMotionPlan->robotRepresentation->getDimensionCount(); k++) {
// 						bool hasNonZeros = theMotionPlan->robotRepresentation->compute_ddpdq_dqi(ee.endEffectorLocalCoords, ee.endEffectorRB, ddEndEffectordq_dqi, k);
// 						if (hasNonZeros == false) continue;
// 						dVector V = ddEndEffectordq_dqi.transpose()*err;
// 						for (int l = k; l < theMotionPlan->robotRepresentation->getDimensionCount(); l++)
// 								ADD_HES_ELEMENT(hessianEntries, I + k, I + l, V(l), weight);
// 					}

					//now add the outer product of the jacobians...
					MatrixNxM outerProd =	dEndEffectordq.row(0).transpose()*dEndEffectordq.row(0) +
											dEndEffectordq.row(1).transpose()*dEndEffectordq.row(1) +
											dEndEffectordq.row(2).transpose()*dEndEffectordq.row(2);

					for (int k = 0; k < theMotionPlan->robotRepresentation->getDimensionCount(); k++) {
						for (int l = k; l < theMotionPlan->robotRepresentation->getDimensionCount(); l++) {
							ADD_HES_ELEMENT(hessianEntries, I + k, I + l, outerProd(k,l), weight);
						}
					}
// 					for (int k = 0; k < theMotionPlan->robotRepresentation->getDimensionCount(); k++) {
// 						for (int l = k; l < theMotionPlan->robotRepresentation->getDimensionCount(); l++) {
// 							double val = 0;
// 							for (int m = 0; m < 3; m++)
// 								val += dEndEffectordq(m, k) * dEndEffectordq(m, l);
// 							ADD_HES_ELEMENT(hessianEntries, I + k, I + l, val, weight);
// 						}
// 					}

					//and now the mixed derivatives
					if (theMotionPlan->feetPositionsParamsStartIndex >= 0) {
						for (int k = 0; k < theMotionPlan->robotRepresentation->getDimensionCount(); k++) {
							int I = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3;
							int J = theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + k;
							ADD_HES_ELEMENT(hessianEntries, I    , J, -dEndEffectordq(0, k), weight);
							ADD_HES_ELEMENT(hessianEntries, I + 1, J, -dEndEffectordq(1, k), weight);
							ADD_HES_ELEMENT(hessianEntries, I + 2, J, -dEndEffectordq(2, k), weight);
						}
					}
				}
			}
		}
	}
}
