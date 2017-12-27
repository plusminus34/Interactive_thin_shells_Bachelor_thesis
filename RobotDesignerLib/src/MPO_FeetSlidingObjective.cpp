#include <RobotDesignerLib/MPO_FeetSlidingObjective.h>

#include <MathLib/AutoDiff.h>

//TODO: these should be hard constraints, as should the periodic motions...

MPO_FeetSlidingObjective::MPO_FeetSlidingObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_FeetSlidingObjective::~MPO_FeetSlidingObjective(void){
}

double MPO_FeetSlidingObjective::computeValue(const dVector& p){

	double retVal = 0;

	int nSamplePoints = theMotionPlan->nSamplePoints;
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) nSamplePoints -= 1;

	const double dt = theMotionPlan->motionPlanDuration / (nSamplePoints-1);

	for (int j=0; j<nSamplePoints; j++){
		for (const LocomotionEngine_EndEffectorTrajectory &ee : theMotionPlan->endEffectorTrajectories){

			Vector3d eePosj = ee.EEPos[j];

			if(ee.isWheel){
				Vector3d rho = ee.getWheelRho();
				Vector3d axis = ee.wheelAxis;
				Vector3d axisYaw = ee.wheelYawAxis;
				Vector3d axisTilt = ee.wheelTiltAxis;

				double speedj = ee.wheelSpeed[j];
				double alphaj = ee.wheelYawAngle[j];
				double betaj = ee.wheelTiltAngle[j];

				double c = ee.contactFlag[j];

				Vector3d eePosjp1 = ee.EEPos[j+1];
				double speedjp1 = ee.wheelSpeed[j+1];
				double alphajp1 = ee.wheelYawAngle[j+1];
				double betajp1 = ee.wheelTiltAngle[j+1];

				retVal += computeEnergyWheel(eePosjp1, eePosj, dt,
											 rho, axis,
											 axisYaw, alphaj, alphajp1,
											 axisTilt, betaj, betajp1,
											 speedj, speedjp1, c, weight);
			}
			else{ // ee is foot
				if (j>0){
					double c = ee.contactFlag[j] * ee.contactFlag[j-1];
					Vector3d eePosjm1 = ee.EEPos[j-1];

					retVal += computeEnergyFoot(eePosj, eePosjm1, c, weight);

				}
				if (j<theMotionPlan->nSamplePoints-1){
					double c = ee.contactFlag[j];
					Vector3d eePosjp1 = ee.EEPos[j+1];

					retVal += computeEnergyFoot(eePosjp1, eePosj, c, weight);
				}
			}
		}
	}
	return retVal;
}

void MPO_FeetSlidingObjective::addGradientTo(dVector& grad, const dVector& p) {

	if (theMotionPlan->feetPositionsParamsStartIndex < 0 || theMotionPlan->wheelParamsStartIndex < 0)
		return;

	typedef AutoDiffT<double, double> ScalarDiff;

	const int nLimbs = theMotionPlan->endEffectorTrajectories.size();

	int nSamplePoints = theMotionPlan->nSamplePoints;
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) nSamplePoints -= 1;

	const double dt = theMotionPlan->motionPlanDuration / (nSamplePoints-1);

	for (int j=0; j<nSamplePoints; j++){
		for (int i=0; i<nLimbs; i++){

			const LocomotionEngine_EndEffectorTrajectory &ee = theMotionPlan->endEffectorTrajectories[i];

			// Position of foot i at time sample j
			int iEEj = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3;
			V3T<ScalarDiff> eePosj;
			for (int k = 0; k < 3; ++k)
				eePosj(k) = p[iEEj + k];

			if(ee.isWheel){
				// get wheel axes
				V3T<ScalarDiff> rho = ee.getWheelRho();
				V3T<ScalarDiff> wheelAxisAD(ee.wheelAxis);
				V3T<ScalarDiff> wheelYawAxis(ee.wheelYawAxis);
				V3T<ScalarDiff> wheelTiltAxis(ee.wheelTiltAxis);

				// get wheel motion parameters at time j
				ScalarDiff alphaj = p[theMotionPlan->getWheelYawAngleIndex(i, j)];
				ScalarDiff betaj = p[theMotionPlan->getWheelTiltAngleIndex(i, j)];//theMotionPlan->endEffectorTrajectories[i].wheelAxisBeta[j];
				ScalarDiff speedj = p[theMotionPlan->getWheelSpeedIndex(i, j)];

				double c = ee.contactFlag[j];

				int iEEjp1 = theMotionPlan->feetPositionsParamsStartIndex + (j+1) * nLimbs * 3 + i * 3;
				V3T<ScalarDiff> eePosjp1;
				for (int k = 0; k < 3; ++k)
					eePosjp1(k) = p[iEEjp1 + k];

				ScalarDiff alphajp1 = p[theMotionPlan->getWheelYawAngleIndex(i, j+1)];
				ScalarDiff betajp1 = p[theMotionPlan->getWheelTiltAngleIndex(i, j+1)];
				ScalarDiff speedjp1 = p[theMotionPlan->getWheelSpeedIndex(i, j+1)];

				DOF<ScalarDiff> dofs[numDOFsWheel];
				for (int k = 0; k < 3; ++k) {
					// ePosj
					dofs[k].v = &eePosj(k);
					dofs[k].i = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + k;
					// ePosjp1
					dofs[3+k].v = &eePosjp1(k);
					dofs[3+k].i = theMotionPlan->feetPositionsParamsStartIndex + (j+1) * nLimbs * 3 + i * 3 + k;
				}
				// speedj
				dofs[6].v = &speedj;
				dofs[6].i = theMotionPlan->getWheelSpeedIndex(i, j);
				// speedjp1
				dofs[7].v = &speedjp1;
				dofs[7].i = theMotionPlan->getWheelSpeedIndex(i, j+1);
				// alphaj
				dofs[8].v = &alphaj;
				dofs[8].i = theMotionPlan->getWheelYawAngleIndex(i, j);
				// alphajp1
				dofs[9].v = &alphajp1;
				dofs[9].i = theMotionPlan->getWheelYawAngleIndex(i, j+1);
				// betaj
				dofs[10].v = &betaj;
				dofs[10].i = theMotionPlan->getWheelTiltAngleIndex(i, j);
				// betajp1
				dofs[11].v = &betajp1;
				dofs[11].i = theMotionPlan->getWheelTiltAngleIndex(i, j+1);

				// derive by all dofs
				for (int k = 0; k < numDOFsWheel; ++k) {
					dofs[k].v->deriv() = 1.0;
					ScalarDiff energy = computeEnergyWheel(eePosjp1, eePosj, dt,
														   rho, wheelAxisAD,
														   wheelYawAxis, alphaj, alphajp1,
														   wheelTiltAxis,betaj, betajp1,
														   speedj, speedjp1, c, weight);
					grad[dofs[k].i] += energy.deriv();
					dofs[k].v->deriv() = 0.0;
				}
			}
			else{ // ee is foot

				if (j>0){
					double c = ee.contactFlag[j] * ee.contactFlag[j-1];

					// Position of foot i at time sample j-1
					int iEEjm1 = theMotionPlan->feetPositionsParamsStartIndex + (j-1) * nLimbs * 3 + i * 3;
					V3T<ScalarDiff> eePosjm1;
					for (int k = 0; k < 3; ++k)
						eePosjm1(k) = p[iEEjm1 + k];

					DOF<ScalarDiff> dofs[numDOFsFoot];
					for (int k = 0; k < 3; ++k) {
						// ePosj
						dofs[k].v = &eePosj(k);
						dofs[k].i = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + k;
						// ePosjm1
						dofs[3+k].v = &eePosjm1(k);
						dofs[3+k].i = theMotionPlan->feetPositionsParamsStartIndex + (j-1) * nLimbs * 3 + i * 3 + k;
					}

					for (int k = 0; k < numDOFsFoot; ++k) {
						dofs[k].v->deriv() = 1.0;
						ScalarDiff energy = computeEnergyFoot(eePosj, eePosjm1, c, weight);
						grad[dofs[k].i] += energy.deriv();
						dofs[k].v->deriv() = 0.0;
					}
				}
				if (j<theMotionPlan->nSamplePoints-1){
					double c = ee.contactFlag[j];

					int iEEjp1 = theMotionPlan->feetPositionsParamsStartIndex + (j+1) * nLimbs * 3 + i * 3;
					V3T<ScalarDiff> eePosjp1;
					for (int k = 0; k < 3; ++k)
						eePosjp1(k) = p[iEEjp1 + k];

					DOF<ScalarDiff> dofs[numDOFsFoot];
					for (int k = 0; k < 3; ++k) {
						// ePosj
						dofs[k].v = &eePosj(k);
						dofs[k].i = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + k;
						// ePosjp1
						dofs[3+k].v = &eePosjp1(k);
						dofs[3+k].i = theMotionPlan->feetPositionsParamsStartIndex + (j+1) * nLimbs * 3 + i * 3 + k;
					}

					// derive by all dofs
					for (int k = 0; k < numDOFsFoot; ++k) {
						dofs[k].v->deriv() = 1.0;
						ScalarDiff energy = computeEnergyFoot(eePosjp1, eePosj, c, weight);
						grad[dofs[k].i] += energy.deriv();
						dofs[k].v->deriv() = 0.0;
					}
				}
			}

		}
	}
}

void MPO_FeetSlidingObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	if (theMotionPlan->feetPositionsParamsStartIndex < 0 || theMotionPlan->wheelParamsStartIndex < 0)
		return;

	typedef AutoDiffT<double, double> ScalarDiff;
	typedef AutoDiffT<ScalarDiff, ScalarDiff> ScalarDiffDiff;

	const int nLimbs = theMotionPlan->endEffectorTrajectories.size();

	int nSamplePoints = theMotionPlan->nSamplePoints;
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) nSamplePoints -= 1;

	const double dt = theMotionPlan->motionPlanDuration / (nSamplePoints-1);

	for (int j=0; j<nSamplePoints; j++){
		for (int i=0; i<nLimbs; i++){

			const LocomotionEngine_EndEffectorTrajectory &ee = theMotionPlan->endEffectorTrajectories[i];

			int iEEj = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3;
			V3T<ScalarDiffDiff> eePosj;
			for (int k = 0; k < 3; ++k)
				eePosj(k) = p[iEEj + k];

			if(ee.isWheel)
			{
				// get wheel axes
				V3T<ScalarDiffDiff> rho = ee.getWheelRho();
				V3T<ScalarDiffDiff> wheelAxisAD(ee.wheelAxis);
				V3T<ScalarDiffDiff> wheelYawAxis(ee.wheelYawAxis);
				V3T<ScalarDiffDiff> wheelTiltAxis(ee.wheelTiltAxis);

				// get wheel motion parameters at time j
				ScalarDiffDiff alphaj = p[theMotionPlan->getWheelYawAngleIndex(i, j)];
				ScalarDiffDiff betaj = theMotionPlan->endEffectorTrajectories[i].wheelTiltAngle[j];
				ScalarDiffDiff speedj = p[theMotionPlan->getWheelSpeedIndex(i, j)];

				double c = ee.contactFlag[j];

				int iEEjp1 = theMotionPlan->feetPositionsParamsStartIndex + (j+1) * nLimbs * 3 + i * 3;
				V3T<ScalarDiffDiff> eePosjp1;
				for (int k = 0; k < 3; ++k)
					eePosjp1(k) = p[iEEjp1 + k];

				// get wheel motion parameters at time j+1
				ScalarDiffDiff alphajp1 = p[theMotionPlan->getWheelYawAngleIndex(i, j+1)];
				ScalarDiffDiff betajp1 = p[theMotionPlan->getWheelTiltAngleIndex(i, j+1)];
				ScalarDiffDiff speedjp1 = p[theMotionPlan->getWheelSpeedIndex(i, j+1)];

				DOF<ScalarDiffDiff> dofs[numDOFsWheel];
				for (int k = 0; k < 3; ++k) {
					// ePosj
					dofs[k].v = &eePosj(k);
					dofs[k].i = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + k;
					// ePosjm1
					dofs[3+k].v = &eePosjp1(k);
					dofs[3+k].i = theMotionPlan->feetPositionsParamsStartIndex + (j+1) * nLimbs * 3 + i * 3 + k;
				}
				// speedj
				dofs[6].v = &speedj;
				dofs[6].i = theMotionPlan->getWheelSpeedIndex(i, j);
				// speedjp1
				dofs[7].v = &speedjp1;
				dofs[7].i = theMotionPlan->getWheelSpeedIndex(i, j+1);
				// alphaj
				dofs[8].v = &alphaj;
				dofs[8].i = theMotionPlan->getWheelYawAngleIndex(i, j);
				// alphajp1
				dofs[9].v = &alphajp1;
				dofs[9].i = theMotionPlan->getWheelYawAngleIndex(i, j+1);
				// betaj
				dofs[10].v = &betaj;
				dofs[10].i = theMotionPlan->getWheelTiltAngleIndex(i, j);
				// betajp1
				dofs[11].v = &betajp1;
				dofs[11].i = theMotionPlan->getWheelTiltAngleIndex(i, j+1);

				for (int k = 0; k < numDOFsWheel; ++k) {

					dofs[k].v->deriv().value() = 1.0;

					for (int l = 0; l <= k; ++l) {
						dofs[l].v->value().deriv() = 1.0;

						ScalarDiffDiff energy = computeEnergyWheel(eePosjp1, eePosj, dt,
																   rho, wheelAxisAD,
																   wheelYawAxis, alphaj, alphajp1,
																   wheelTiltAxis, betaj, betajp1,
																   speedj, speedjp1, c, weight);

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
			else{ // ee is foot

				if (j>0){
					double c = ee.contactFlag[j] * ee.contactFlag[j-1];

					int iEEjm1 = theMotionPlan->feetPositionsParamsStartIndex + (j-1) * nLimbs * 3 + i * 3;
					V3T<ScalarDiffDiff> eePosjm1;
					for (int k = 0; k < 3; ++k)
						eePosjm1(k) = p[iEEjm1 + k];

					DOF<ScalarDiffDiff> dofs[numDOFsFoot];
					for (int k = 0; k < 3; ++k) {
						// ePosj
						dofs[k].v = &eePosj(k);
						dofs[k].i = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + k;
						// ePosjm1
						dofs[3+k].v = &eePosjm1(k);
						dofs[3+k].i = theMotionPlan->feetPositionsParamsStartIndex + (j-1) * nLimbs * 3 + i * 3 + k;
					}

					for (int k = 0; k < numDOFsFoot; ++k) {

						dofs[k].v->deriv().value() = 1.0;

						for (int l = 0; l <= k; ++l) {
							dofs[l].v->value().deriv() = 1.0;

							ScalarDiffDiff energy = computeEnergyFoot(eePosj, eePosjm1, c, weight);

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
				if (j<theMotionPlan->nSamplePoints-1){
					double c = ee.contactFlag[j];

					int iEEjp1 = theMotionPlan->feetPositionsParamsStartIndex + (j+1) * nLimbs * 3 + i * 3;
					V3T<ScalarDiffDiff> eePosjp1;
					for (int k = 0; k < 3; ++k)
						eePosjp1(k) = p[iEEjp1 + k];

					DOF<ScalarDiffDiff> dofs[numDOFsFoot];
					for (int k = 0; k < 3; ++k) {
						// ePosj
						dofs[k].v = &eePosj(k);
						dofs[k].i = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + k;
						// ePosjm1
						dofs[3+k].v = &eePosjp1(k);
						dofs[3+k].i = theMotionPlan->feetPositionsParamsStartIndex + (j+1) * nLimbs * 3 + i * 3 + k;
					}

					for (int k = 0; k < numDOFsFoot; ++k) {

						dofs[k].v->deriv().value() = 1.0;

						for (int l = 0; l <= k; ++l) {
							dofs[l].v->value().deriv() = 1.0;

							ScalarDiffDiff energy = computeEnergyFoot(eePosjp1, eePosj, c, weight);

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
		}
	}
}
