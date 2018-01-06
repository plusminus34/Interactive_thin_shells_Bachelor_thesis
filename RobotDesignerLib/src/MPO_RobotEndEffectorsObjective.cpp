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
				
				Vector3d rhoRot = LocomotionEngine_EndEffectorTrajectory::rotVecByYawTilt(rho, yawAxis, yawAngle, tiltAxis, tiltAngle);
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
				Vector3d dYaw = LocomotionEngine_EndEffectorTrajectory::drotVecByYawTilt_dYaw(rho, yawAxis, yawAngle, tiltAxis, tiltAngle);
				Vector3d dTilt = LocomotionEngine_EndEffectorTrajectory::drotVecByYawTilt_dTilt(rho, yawAxis, yawAngle, tiltAxis, tiltAngle);

				grad(theMotionPlan->getWheelYawAngleIndex(i, j)) -= weight*dYaw.transpose()*err;
				grad(theMotionPlan->getWheelTiltAngleIndex(i, j)) -= weight*dTilt.transpose()*err;
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
	
// 	if (!hackHessian)
// 	{
// 		addHessianEntriesTo_unhacked(hessianEntries, p);
// 		return;
// 	}

	MatrixNxM dpdq, ddpdqdqi;

	int nLimbs = theMotionPlan->endEffectorTrajectories.size();
	for (int j=0;j<theMotionPlan->nSamplePoints;j++){
		dVector q_t;
		theMotionPlan->robotStateTrajectory.getQAtTimeIndex(j, q_t);
		theMotionPlan->robotRepresentation->setQ(q_t);


		for (int i=0;i<nLimbs;i++){
			const LocomotionEngine_EndEffectorTrajectory &ee = theMotionPlan->endEffectorTrajectories[i];

			if (ee.isWheel) {				
				V3D rho = ee.getWheelRhoLocal();
				V3D yawAxis = ee.wheelYawAxis;
				V3D tiltAxis = ee.wheelTiltAxis;
				double yawAngle = ee.wheelYawAngle[j];
				double tiltAngle = ee.wheelTiltAngle[j];

				Vector3d rhoRot = LocomotionEngine_EndEffectorTrajectory::rotVecByYawTilt(rho, yawAxis, yawAngle, tiltAxis, tiltAngle);
				Vector3d robotEEPos = theMotionPlan->robotRepresentation->getWorldCoordinatesForPointT(ee.endEffectorLocalCoords, ee.endEffectorRB, q_t);
				Vector3d err=robotEEPos - ee.EEPos[j] - rhoRot;

				theMotionPlan->robotRepresentation->compute_dpdq(ee.endEffectorLocalCoords, ee.endEffectorRB, dpdq);
				Vector3d dYaw = LocomotionEngine_EndEffectorTrajectory::drotVecByYawTilt_dYaw(rho, yawAxis, yawAngle, tiltAxis, tiltAngle);
				Vector3d dTilt = LocomotionEngine_EndEffectorTrajectory::drotVecByYawTilt_dTilt(rho, yawAxis, yawAngle, tiltAxis, tiltAngle);

				int Ifeet = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3;
				int Iq = theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim;
				int Iyaw = theMotionPlan->getWheelYawAngleIndex(i, j);
				int Itilt = theMotionPlan->getWheelTiltAngleIndex(i, j);

				//compute the gradient with respect to the feet locations
				if (theMotionPlan->feetPositionsParamsStartIndex >= 0) {
					
					ADD_HES_ELEMENT(hessianEntries, Ifeet, Ifeet, 1, weight);
					ADD_HES_ELEMENT(hessianEntries, Ifeet + 1, Ifeet + 1, 1, weight);
					ADD_HES_ELEMENT(hessianEntries, Ifeet + 2, Ifeet + 2, 1, weight);
				}
				//and now compute the gradient with respect to the robot q's
				if (theMotionPlan->robotStatesParamsStartIndex >= 0) {
					if (!hackHessian)
						for (int k = 0; k < theMotionPlan->robotRepresentation->getDimensionCount(); k++) {
							bool hasNonZeros = theMotionPlan->robotRepresentation->compute_ddpdq_dqi(ee.endEffectorLocalCoords, ee.endEffectorRB, ddpdqdqi, k);
							if (hasNonZeros == false) continue;
							dVector V = ddpdqdqi.transpose()*err;
							for (int l = k; l < theMotionPlan->robotRepresentation->getDimensionCount(); l++)
								ADD_HES_ELEMENT(hessianEntries, Iq + k, Iq + l, V(l), weight);
					}

					//now add the outer product of the jacobians...
					MatrixNxM outerProd = dpdq.row(0).transpose()*dpdq.row(0) +
						dpdq.row(1).transpose()*dpdq.row(1) +
						dpdq.row(2).transpose()*dpdq.row(2);

					for (int k = 0; k < theMotionPlan->robotRepresentation->getDimensionCount(); k++) {
						for (int l = k; l < theMotionPlan->robotRepresentation->getDimensionCount(); l++) {
							ADD_HES_ELEMENT(hessianEntries, Iq + k, Iq + l, outerProd(k, l), weight);
						}
					}

					//mixed derivatives with feet
					if (theMotionPlan->feetPositionsParamsStartIndex >= 0) {
						for (int k = 0; k < theMotionPlan->robotRepresentation->getDimensionCount(); k++) {
							int J = theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + k;
							ADD_HES_ELEMENT(hessianEntries, Ifeet, J, -dpdq(0, k), weight);
							ADD_HES_ELEMENT(hessianEntries, Ifeet + 1, J, -dpdq(1, k), weight);
							ADD_HES_ELEMENT(hessianEntries, Ifeet + 2, J, -dpdq(2, k), weight);
						}
					}
					//mixed derivatives with angles
					if (theMotionPlan->wheelParamsStartIndex >= 0) {
						MatrixNxM dvdqdTilt = dTilt.transpose()*dpdq;
						MatrixNxM dvdqdYaw = dYaw.transpose()*dpdq;
						for (int k = 0; k < theMotionPlan->robotRepresentation->getDimensionCount(); k++) {
							ADD_HES_ELEMENT(hessianEntries, Iq + k, Itilt, -dvdqdTilt(k), weight);
							ADD_HES_ELEMENT(hessianEntries, Iq + k, Iyaw, -dvdqdYaw(k), weight);
						}

					}
				}
				// Angles part
				if (theMotionPlan->wheelParamsStartIndex >= 0)
				{

					// grad(theMotionPlan->getWheelYawAngleIndex(i, j)) += weight*drhoRotdYawAngle.transpose()*err;
					// second derivatives
					Vector3d ddYaw = LocomotionEngine_EndEffectorTrajectory::ddrotVecByYawTilt_dYaw2(rho, yawAxis, yawAngle, tiltAxis, tiltAngle);
					Vector3d ddTilt = LocomotionEngine_EndEffectorTrajectory::ddrotVecByYawTilt_dTilt2(rho, yawAxis, yawAngle, tiltAxis, tiltAngle);
					Vector3d dYawdTilt = LocomotionEngine_EndEffectorTrajectory::ddrotVecByYawTilt_dYawdTilt(rho, yawAxis, yawAngle, tiltAxis, tiltAngle);

					if (!hackHessian)
					{
						ADD_HES_ELEMENT(hessianEntries, Iyaw, Iyaw, -ddYaw.dot(err), weight);
						ADD_HES_ELEMENT(hessianEntries, Itilt, Itilt, -ddTilt.dot(err), weight);
						ADD_HES_ELEMENT(hessianEntries, Iyaw, Itilt, -dYawdTilt.dot(err), weight);
					}
					// outer product						
					ADD_HES_ELEMENT(hessianEntries, Iyaw, Iyaw, dYaw.dot(dYaw), weight);
					ADD_HES_ELEMENT(hessianEntries, Itilt, Itilt, dTilt.dot(dTilt), weight);
					ADD_HES_ELEMENT(hessianEntries, Iyaw, Itilt, dYaw.dot(dTilt), weight);
					
					// mixed derivatives with feet
					if (theMotionPlan->feetPositionsParamsStartIndex >= 0) {
						ADD_HES_ELEMENT(hessianEntries, Ifeet,     Iyaw,  dYaw(0), weight);
						ADD_HES_ELEMENT(hessianEntries, Ifeet + 1, Iyaw,  dYaw(1), weight);
						ADD_HES_ELEMENT(hessianEntries, Ifeet + 2, Iyaw,  dYaw(2), weight);
						ADD_HES_ELEMENT(hessianEntries, Ifeet,     Itilt, dTilt(0), weight);
						ADD_HES_ELEMENT(hessianEntries, Ifeet + 1, Itilt, dTilt(1), weight);
						ADD_HES_ELEMENT(hessianEntries, Ifeet + 2, Itilt, dTilt(2), weight);
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
					theMotionPlan->robotRepresentation->compute_dpdq(ee.endEffectorLocalCoords, ee.endEffectorRB, dpdq);
					int I = theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim;
					
					if (!hackHessian) {

						for (int k = 0; k < theMotionPlan->robotRepresentation->getDimensionCount(); k++) {
							Vector3d err(ee.EEPos[j] - theMotionPlan->robotRepresentation->getWorldCoordinatesFor(ee.endEffectorLocalCoords, ee.endEffectorRB));
							bool hasNonZeros = theMotionPlan->robotRepresentation->compute_ddpdq_dqi(ee.endEffectorLocalCoords, ee.endEffectorRB, ddpdqdqi, k);
							if (hasNonZeros == false) continue;
							dVector V = ddpdqdqi.transpose()*err;
							for (int l = k; l < theMotionPlan->robotRepresentation->getDimensionCount(); l++)
								ADD_HES_ELEMENT(hessianEntries, I + k, I + l, V(l), weight);
						}
					}


					//now add the outer product of the jacobians...
					MatrixNxM outerProd = dpdq.row(0).transpose()*dpdq.row(0) +
						dpdq.row(1).transpose()*dpdq.row(1) +
						dpdq.row(2).transpose()*dpdq.row(2);

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
							ADD_HES_ELEMENT(hessianEntries, I, J, -dpdq(0, k), weight);
							ADD_HES_ELEMENT(hessianEntries, I + 1, J, -dpdq(1, k), weight);
							ADD_HES_ELEMENT(hessianEntries, I + 2, J, -dpdq(2, k), weight);
						}
					}
				}
			}
		}
	}
}