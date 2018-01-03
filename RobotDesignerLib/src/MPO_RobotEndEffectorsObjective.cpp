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
				retVal += computeEnergyWheel<double>(ee.EEPos[j], ee.getWheelRhoLocal(),
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

	int nLimbs = theMotionPlan->endEffectorTrajectories.size();
	for (int j=0;j<theMotionPlan->nSamplePoints;j++){

		// get robot state at timestep j
		dVector q_t;
		theMotionPlan->robotStateTrajectory.getQAtTimeIndex(j, q_t);
		theMotionPlan->robotRepresentation->setQ(q_t);

		for (int i=0;i<nLimbs;i++){

			const LocomotionEngine_EndEffectorTrajectory &ee = theMotionPlan->endEffectorTrajectories[i];

			if(ee.isWheel){
				V3D rho = ee.getWheelRhoLocal();
				V3D yawAxis = ee.wheelYawAxis;
				V3D tiltAxis = ee.wheelTiltAxis;
				double yawAngle = ee.wheelYawAngle[j];
				double tiltAngle = ee.wheelTiltAngle[j];
				
				if (theMotionPlan->feetPositionsParamsStartIndex < 0 || theMotionPlan->wheelParamsStartIndex < 0)
					continue;
				
				Vector3d rhoRot = LocomotionEngine_EndEffectorTrajectory::rotateVectorUsingWheelAngles(rho, yawAxis, yawAngle, tiltAxis, tiltAngle);
				Vector3d robotEEPos = theMotionPlan->robotRepresentation->getWorldCoordinatesForPointT(ee.endEffectorLocalCoords, ee.endEffectorRB, q_t);
				Vector3d err(robotEEPos-ee.EEPos[j] - rhoRot);
				//compute the gradient with respect to the feet locations

				int ind = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3;
				grad.segment<3>(ind) -= weight*err;

				//compute the gradient with respect to the robot q's
				theMotionPlan->robotRepresentation->compute_dpdq(ee.endEffectorLocalCoords, ee.endEffectorRB, dEndEffectordq);

				//dEdee * deedq = dEdq
				ind = theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim;
				grad.segment(ind, dEndEffectordq.cols()) += weight*dEndEffectordq.transpose()*err;

				//compute the gradient with respect yaw and tilt angle
				Vector3d drhoRotdYawAngle = LocomotionEngine_EndEffectorTrajectory::drotateVectorUsingWheelAngles_dYawAngle(rho, yawAxis, yawAngle, tiltAxis, tiltAngle);
				grad(theMotionPlan->getWheelYawAngleIndex(i, j)) -= weight*drhoRotdYawAngle.transpose()*err;

				Vector3d drhoRotdTiltAngle = LocomotionEngine_EndEffectorTrajectory::drotateVectorUsingWheelAngles_dTiltAngle(rho, yawAxis, yawAngle, tiltAxis, tiltAngle);
				grad(theMotionPlan->getWheelTiltAngleIndex(i, j)) -= weight*drhoRotdTiltAngle.transpose()*err;
			}
			else
			{
				if (theMotionPlan->feetPositionsParamsStartIndex < 0)
					continue;

				Vector3d err(ee.EEPos[j] - theMotionPlan->robotRepresentation->getWorldCoordinatesForPointT(ee.endEffectorLocalCoords, ee.endEffectorRB,q_t));
				//compute the gradient with respect to the feet locations
				if (theMotionPlan->feetPositionsParamsStartIndex >= 0) {
					int ind = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3;
					grad.segment<3>(ind) += weight*err;
				}

				//and now compute the gradient with respect to the robot q's
				if (theMotionPlan->robotStatesParamsStartIndex >= 0) {
					theMotionPlan->robotRepresentation->compute_dpdq(ee.endEffectorLocalCoords, ee.endEffectorRB, dEndEffectordq);

					//dEdee * deedq = dEdq
					int ind = theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim;
					grad.segment(ind, dEndEffectordq.cols()) += -weight*dEndEffectordq.transpose()*err;
				}
			}
		}
	}
}

void MPO_RobotEndEffectorsObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);
	MatrixNxM dEndEffectordq, ddEndEffectordq_dqi;

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

			V3T<ScalarDiffDiff> eePosLocal = ee.endEffectorLocalCoords;
			V3T<ScalarDiffDiff> eePos = ee.EEPos[j];

			if (ee.isWheel) {
				//compute the gradient with respect to the feet locations
				if (theMotionPlan->feetPositionsParamsStartIndex >= 0) {
					int I = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3;
					ADD_HES_ELEMENT(hessianEntries, I, I, 1, weight);
					ADD_HES_ELEMENT(hessianEntries, I + 1, I + 1, 1, weight);
					ADD_HES_ELEMENT(hessianEntries, I + 2, I + 2, 1, weight);
				}
				//and now compute the gradient with respect to the robot q's
				if (theMotionPlan->robotStatesParamsStartIndex >= 0) {
					theMotionPlan->robotRepresentation->compute_dpdq(ee.endEffectorLocalCoords, ee.endEffectorRB, dEndEffectordq);
					int I = theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim;

					////////////  Taken out for majorization ///////////////////
					// 					for (int k = 0; k < theMotionPlan->robotRepresentation->getDimensionCount(); k++) {
					// 						bool hasNonZeros = theMotionPlan->robotRepresentation->compute_ddpdq_dqi(ee.endEffectorLocalCoords, ee.endEffectorRB, ddEndEffectordq_dqi, k);
					// 						if (hasNonZeros == false) continue;
					// 						dVector V = ddEndEffectordq_dqi.transpose()*err;
					// 						for (int l = k; l < theMotionPlan->robotRepresentation->getDimensionCount(); l++)
					// 								ADD_HES_ELEMENT(hessianEntries, I + k, I + l, V(l), weight);
					// 					}
					///////////////////////////////////////////////////////////

					//now add the outer product of the jacobians...
					MatrixNxM outerProd = dEndEffectordq.row(0).transpose()*dEndEffectordq.row(0) +
						dEndEffectordq.row(1).transpose()*dEndEffectordq.row(1) +
						dEndEffectordq.row(2).transpose()*dEndEffectordq.row(2);

					for (int k = 0; k < theMotionPlan->robotRepresentation->getDimensionCount(); k++) {
						for (int l = k; l < theMotionPlan->robotRepresentation->getDimensionCount(); l++) {
							ADD_HES_ELEMENT(hessianEntries, I + k, I + l, outerProd(k, l), weight);
						}
					}

					//and now the mixed derivatives
					if (theMotionPlan->feetPositionsParamsStartIndex >= 0) {
						for (int k = 0; k < theMotionPlan->robotRepresentation->getDimensionCount(); k++) {
							int I = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3;
							int J = theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + k;
							ADD_HES_ELEMENT(hessianEntries, I, J, -dEndEffectordq(0, k), weight);
							ADD_HES_ELEMENT(hessianEntries, I + 1, J, -dEndEffectordq(1, k), weight);
							ADD_HES_ELEMENT(hessianEntries, I + 2, J, -dEndEffectordq(2, k), weight);
						}
					}
				}
			}
			else {
				//compute the gradient with respect to the feet locations
				if (theMotionPlan->feetPositionsParamsStartIndex >= 0) {
					int I = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3;
					ADD_HES_ELEMENT(hessianEntries, I, I, 1, weight);
					ADD_HES_ELEMENT(hessianEntries, I + 1, I + 1, 1, weight);
					ADD_HES_ELEMENT(hessianEntries, I + 2, I + 2, 1, weight);
				}
				//and now compute the gradient with respect to the robot q's
				if (theMotionPlan->robotStatesParamsStartIndex >= 0) {
					theMotionPlan->robotRepresentation->compute_dpdq(ee.endEffectorLocalCoords, ee.endEffectorRB, dEndEffectordq);
					int I = theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim;
					
					if (!hackHessian) {

						for (int k = 0; k < theMotionPlan->robotRepresentation->getDimensionCount(); k++) {
							Vector3d err(ee.EEPos[j] - theMotionPlan->robotRepresentation->getWorldCoordinatesFor(ee.endEffectorLocalCoords, ee.endEffectorRB));
							bool hasNonZeros = theMotionPlan->robotRepresentation->compute_ddpdq_dqi(ee.endEffectorLocalCoords, ee.endEffectorRB, ddEndEffectordq_dqi, k);
							if (hasNonZeros == false) continue;
							dVector V = ddEndEffectordq_dqi.transpose()*err;
							for (int l = k; l < theMotionPlan->robotRepresentation->getDimensionCount(); l++)
								ADD_HES_ELEMENT(hessianEntries, I + k, I + l, V(l), weight);
						}
					}


					//now add the outer product of the jacobians...
					MatrixNxM outerProd = dEndEffectordq.row(0).transpose()*dEndEffectordq.row(0) +
						dEndEffectordq.row(1).transpose()*dEndEffectordq.row(1) +
						dEndEffectordq.row(2).transpose()*dEndEffectordq.row(2);

					for (int k = 0; k < theMotionPlan->robotRepresentation->getDimensionCount(); k++) {
						for (int l = k; l < theMotionPlan->robotRepresentation->getDimensionCount(); l++) {
							ADD_HES_ELEMENT(hessianEntries, I + k, I + l, outerProd(k, l), weight);
						}
					}

					//and now the mixed derivatives
					if (theMotionPlan->feetPositionsParamsStartIndex >= 0) {
						for (int k = 0; k < theMotionPlan->robotRepresentation->getDimensionCount(); k++) {
							int I = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3;
							int J = theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + k;
							ADD_HES_ELEMENT(hessianEntries, I, J, -dEndEffectordq(0, k), weight);
							ADD_HES_ELEMENT(hessianEntries, I + 1, J, -dEndEffectordq(1, k), weight);
							ADD_HES_ELEMENT(hessianEntries, I + 2, J, -dEndEffectordq(2, k), weight);
						}
					}
				}
			}
		}
	}
}
