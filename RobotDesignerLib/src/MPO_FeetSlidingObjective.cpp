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

	if (theMotionPlan->feetPositionsParamsStartIndex < 0 || theMotionPlan->wheelParamsStartIndex < 0)
		return 0;

	double retVal = 0;
	const double dt = theMotionPlan->motionPlanDuration / theMotionPlan->nSamplePoints;
	const int nLimbs = theMotionPlan->endEffectorTrajectories.size();

	for (int j=0;j<theMotionPlan->nSamplePoints;j++){
		for (uint i=0;i<theMotionPlan->endEffectorTrajectories.size();i++){

			int iEEj = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3;
			Eigen::Vector3d eePosj(p[iEEj + 0], p[iEEj + 1], p[iEEj + 2]);

			double speed = p[theMotionPlan->wheelParamsStartIndex + theMotionPlan->nWheelParams*(j*nLimbs + i)];
			double wheelRadius = theMotionPlan->endEffectorTrajectories[i].wheelRadius;
			double alphaj = p[theMotionPlan->getWheelAxisAlphaIndex(i, j)];

			if (j>0){
				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j] * theMotionPlan->endEffectorTrajectories[i].contactFlag[j-1];

				int iEEjm1 = theMotionPlan->feetPositionsParamsStartIndex + (j-1) * nLimbs * 3 + i * 3;
				Eigen::Vector3d eePosjm1(p[iEEjm1 + 0], p[iEEjm1 + 1], p[iEEjm1 + 2]);
				double alphajm1 = p[theMotionPlan->getWheelAxisAlphaIndex(i, j-1)];

				retVal += computeEnergy(eePosj, eePosjm1, dt, wheelRadiusV, wheelRadius, wheelAxis, alphaj, alphajm1, speed, c, weight);

			}
			if (j<theMotionPlan->nSamplePoints-1){
				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];

				int iEEjp1 = theMotionPlan->feetPositionsParamsStartIndex + (j+1) * nLimbs * 3 + i * 3;
				Eigen::Vector3d eePosjp1(p[iEEjp1 + 0], p[iEEjp1 + 1], p[iEEjp1 + 2]);
				double alphajp1 = p[theMotionPlan->getWheelAxisAlphaIndex(i, j+1)];

				retVal += computeEnergy(eePosjp1, eePosj, dt, wheelRadiusV, wheelRadius, wheelAxis, alphaj, alphajp1, speed, c, weight);
			}
		}
	}
	return retVal;
}

void MPO_FeetSlidingObjective::addGradientTo(dVector& grad, const dVector& p) {

	if (theMotionPlan->feetPositionsParamsStartIndex < 0 || theMotionPlan->wheelParamsStartIndex < 0)
		return;

	typedef AutoDiffT<double, double> ScalarDiff;

	//and now compute the gradient with respect c and eePos
	const double dt = theMotionPlan->motionPlanDuration / theMotionPlan->nSamplePoints;
	const int nLimbs = theMotionPlan->endEffectorTrajectories.size();

	for (int j=0; j<theMotionPlan->nSamplePoints; j++){
		for (int i=0; i<nLimbs; i++){

			// Position of foot i at time sample j
			int iEEj = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3;
			Vector3T<ScalarDiff> eePosj;
			for (int k = 0; k < 3; ++k)
				eePosj(k) = p[iEEj + k];

			// Angular velocity vector and speed
			ScalarDiff alphaj = p[theMotionPlan->getWheelAxisAlphaIndex(i, j)];
			Vector3T<ScalarDiff> wheelAxisAD;
			for (int k = 0; k < 3; ++k)
				wheelAxisAD(k) = wheelAxis[k];
			ScalarDiff speed(p[theMotionPlan->wheelParamsStartIndex + theMotionPlan->nWheelParams*(j*nLimbs + i)], 0);

			// Wheel radius
			Vector3T<ScalarDiff> wheelRadiusAD;
			for (int k = 0; k < 3; ++k)
				wheelRadiusAD(k) = wheelRadiusV[k];
			ScalarDiff r = theMotionPlan->endEffectorTrajectories[i].wheelRadius;

			if (j>0){
				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j] * theMotionPlan->endEffectorTrajectories[i].contactFlag[j-1];

				// Position of foot i at time sample j-1
				int iEEjm1 = theMotionPlan->feetPositionsParamsStartIndex + (j-1) * nLimbs * 3 + i * 3;
				Vector3T<ScalarDiff> eePosjm1;
				for (int k = 0; k < 3; ++k)
					eePosjm1(k) = p[iEEjm1 + k];

				ScalarDiff alphajm1 = p[theMotionPlan->getWheelAxisAlphaIndex(i, j-1)];

				DOF<ScalarDiff> dofs[numDOFs];
				for (int k = 0; k < 3; ++k) {
					// ePosj
					dofs[k].v = &eePosj(k);
					dofs[k].i = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + k;
					// ePosjm1
					dofs[3+k].v = &eePosjm1(k);
					dofs[3+k].i = theMotionPlan->feetPositionsParamsStartIndex + (j-1) * nLimbs * 3 + i * 3 + k;
				}
				// speed
				dofs[6].v = &speed;
				dofs[6].i = theMotionPlan->wheelParamsStartIndex + theMotionPlan->nWheelParams*(j*nLimbs + i);
				// alphaj
				dofs[7].v = &alphaj;
				dofs[7].i = theMotionPlan->getWheelAxisAlphaIndex(i, j);
				// alphajm1
				dofs[8].v = &alphajm1;
				dofs[8].i = theMotionPlan->getWheelAxisAlphaIndex(i, j-1);

				for (int k = 0; k < numDOFs; ++k) {
					dofs[k].v->deriv() = 1.0;
					ScalarDiff energy = computeEnergy(eePosj, eePosjm1, dt, wheelRadiusAD, r, wheelAxisAD, alphaj, alphajm1, speed, c, weight);
					grad[dofs[k].i] += energy.deriv();
					dofs[k].v->deriv() = 0.0;
				}
			}
			if (j<theMotionPlan->nSamplePoints-1){
				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];

				int iEEjp1 = theMotionPlan->feetPositionsParamsStartIndex + (j+1) * nLimbs * 3 + i * 3;
				Vector3T<ScalarDiff> eePosjp1;
				for (int k = 0; k < 3; ++k)
					eePosjp1(k) = p[iEEjp1 + k];

				ScalarDiff alphajp1 = p[theMotionPlan->getWheelAxisAlphaIndex(i, j+1)];

				DOF<ScalarDiff> dofs[numDOFs];
				for (int k = 0; k < 3; ++k) {
					// ePosj
					dofs[k].v = &eePosj(k);
					dofs[k].i = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + k;
					// ePosjp1
					dofs[3+k].v = &eePosjp1(k);
					dofs[3+k].i = theMotionPlan->feetPositionsParamsStartIndex + (j+1) * nLimbs * 3 + i * 3 + k;
				}
				// speed
				dofs[6].v = &speed;
				dofs[6].i = theMotionPlan->wheelParamsStartIndex + theMotionPlan->nWheelParams*(j*nLimbs + i);
				// alphaj
				dofs[7].v = &alphaj;
				dofs[7].i = theMotionPlan->getWheelAxisAlphaIndex(i, j);
				// alphajp1
				dofs[8].v = &alphajp1;
				dofs[8].i = theMotionPlan->getWheelAxisAlphaIndex(i, j+1);

				// derive by eePosj, eePosjm1 and speed
				for (int k = 0; k < numDOFs; ++k) {
					dofs[k].v->deriv() = 1.0;
					ScalarDiff energy = computeEnergy(eePosjp1, eePosj, dt, wheelRadiusAD, r, wheelAxisAD, alphaj, alphajp1, speed, c, weight);
					grad[dofs[k].i] += energy.deriv();
					dofs[k].v->deriv() = 0.0;
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

	//and now compute the gradient with respect c and eePos
	const double dt = theMotionPlan->motionPlanDuration / theMotionPlan->nSamplePoints;
	const int nLimbs = theMotionPlan->endEffectorTrajectories.size();


	for (int j=0;j<theMotionPlan->nSamplePoints;j++){
		for (int i=0;i<nLimbs;i++){

			int iEEj = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3;
			Vector3T<ScalarDiffDiff> eePosj;
			for (int k = 0; k < 3; ++k)
				eePosj(k) = p[iEEj + k];

			// Angular velocity vector
			ScalarDiffDiff alphaj = p[theMotionPlan->getWheelAxisAlphaIndex(i, j)];
			Vector3T<ScalarDiffDiff> wheelAxisAD;
			for (int k = 0; k < 3; ++k)
				wheelAxisAD(k) = wheelAxis[k];
			ScalarDiffDiff speed(p[theMotionPlan->wheelParamsStartIndex + theMotionPlan->nWheelParams*(j*nLimbs + i)], 0);

			// Wheel radius
			Vector3T<ScalarDiffDiff> wheelRadiusAD;
			for (int k = 0; k < 3; ++k)
				wheelRadiusAD(k) = wheelRadiusV[k];
			ScalarDiffDiff r = theMotionPlan->endEffectorTrajectories[i].wheelRadius;

			if (j>0){
				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j] * theMotionPlan->endEffectorTrajectories[i].contactFlag[j-1];

				int iEEjm1 = theMotionPlan->feetPositionsParamsStartIndex + (j-1) * nLimbs * 3 + i * 3;
				Vector3T<ScalarDiffDiff> eePosjm1;
				for (int k = 0; k < 3; ++k)
					eePosjm1(k) = p[iEEjm1 + k];

				ScalarDiffDiff alphajm1 = p[theMotionPlan->getWheelAxisAlphaIndex(i, j-1)];

				DOF<ScalarDiffDiff> dofs[numDOFs];
				for (int k = 0; k < 3; ++k) {
					// ePosj
					dofs[k].v = &eePosj(k);
					dofs[k].i = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + k;
					// ePosjm1
					dofs[3+k].v = &eePosjm1(k);
					dofs[3+k].i = theMotionPlan->feetPositionsParamsStartIndex + (j-1) * nLimbs * 3 + i * 3 + k;
				}
				// speed
				dofs[6].v = &speed;
				dofs[6].i = theMotionPlan->wheelParamsStartIndex + theMotionPlan->nWheelParams*(j*nLimbs + i);
				// alphaj
				dofs[7].v = &alphaj;
				dofs[7].i = theMotionPlan->getWheelAxisAlphaIndex(i, j);
				// alphajm1
				dofs[8].v = &alphajm1;
				dofs[8].i = theMotionPlan->getWheelAxisAlphaIndex(i, j-1);

				for (int k = 0; k < numDOFs; ++k) {

					dofs[k].v->deriv().value() = 1.0;

					for (int l = 0; l <= k; ++l) {
						dofs[l].v->value().deriv() = 1.0;

						ScalarDiffDiff energy = computeEnergy(eePosj, eePosjm1, dt, wheelRadiusAD, r, wheelAxisAD, alphaj, alphajm1, speed, c, weight);

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
				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];

				int iEEjp1 = theMotionPlan->feetPositionsParamsStartIndex + (j+1) * nLimbs * 3 + i * 3;
				Vector3T<ScalarDiffDiff> eePosjp1;
				for (int k = 0; k < 3; ++k)
					eePosjp1(k) = p[iEEjp1 + k];

				ScalarDiffDiff alphajp1 = p[theMotionPlan->getWheelAxisAlphaIndex(i, j+1)];


				DOF<ScalarDiffDiff> dofs[numDOFs];
				for (int k = 0; k < 3; ++k) {
					// ePosj
					dofs[k].v = &eePosj(k);
					dofs[k].i = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + k;
					// ePosjm1
					dofs[3+k].v = &eePosjp1(k);
					dofs[3+k].i = theMotionPlan->feetPositionsParamsStartIndex + (j+1) * nLimbs * 3 + i * 3 + k;
				}
				// speed
				dofs[6].v = &speed;
				dofs[6].i = theMotionPlan->wheelParamsStartIndex + theMotionPlan->nWheelParams*(j*nLimbs + i);
				// alphaj
				dofs[7].v = &alphaj;
				dofs[7].i = theMotionPlan->getWheelAxisAlphaIndex(i, j);
				// alphajm1
				dofs[8].v = &alphajp1;
				dofs[8].i = theMotionPlan->getWheelAxisAlphaIndex(i, j+1);

				for (int k = 0; k < numDOFs; ++k) {

					dofs[k].v->deriv().value() = 1.0;

					for (int l = 0; l <= k; ++l) {
						dofs[l].v->value().deriv() = 1.0;

						ScalarDiffDiff energy = computeEnergy(eePosjp1, eePosj, dt, wheelRadiusAD, r, wheelAxisAD, alphaj, alphajp1, speed, c, weight);

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
