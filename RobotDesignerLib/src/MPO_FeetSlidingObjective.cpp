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

				// derive by eePosj, eePosjm1 and speed (= 7 DOFs)
				for (int k = 0; k < 9; ++k) {
					if(k < 3)
						eePosj(k).deriv() = 1.0;
					else if(k < 6)
						eePosjm1(k-3).deriv() = 1.0;
					else if(k < 7)
						speed.deriv() = 1.0;
					else if(k < 8)
						alphaj.deriv() = 1.0;
					else if(k < 9)
						alphajm1.deriv() = 1.0;

					ScalarDiff energy = computeEnergy(eePosj, eePosjm1, dt, wheelRadiusAD, r, wheelAxisAD, alphaj, alphajm1, speed, c, weight);

					if(k < 3)
						grad[theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + k] += energy.deriv();
					else if(k < 6)
						grad[theMotionPlan->feetPositionsParamsStartIndex + (j-1) * nLimbs * 3 + i * 3 + k-3] += energy.deriv();
					else if(k < 7)
						grad[theMotionPlan->wheelParamsStartIndex + theMotionPlan->nWheelParams*(j*nLimbs + i)] += energy.deriv();
					else if(k < 8)
						grad[theMotionPlan->getWheelAxisAlphaIndex(i, j)] += energy.deriv();
					else if(k < 9)
						grad[theMotionPlan->getWheelAxisAlphaIndex(i, j-1)] += energy.deriv();

					if(k < 3)
						eePosj(k).deriv() = 0.0;
					else if(k < 6)
						eePosjm1(k-3).deriv() = 0.0;
					else if(k < 7)
						speed.deriv() = 0.0;
					else if(k < 8)
						alphaj.deriv() = 0.0;
					else if(k < 9)
						alphajm1.deriv() = 0.0;
				}
			}
			if (j<theMotionPlan->nSamplePoints-1){
				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];

				int iEEjp1 = theMotionPlan->feetPositionsParamsStartIndex + (j+1) * nLimbs * 3 + i * 3;
				Vector3T<ScalarDiff> eePosjp1;
				for (int k = 0; k < 3; ++k)
					eePosjp1(k) = p[iEEjp1 + k];

				ScalarDiff alphajp1 = p[theMotionPlan->getWheelAxisAlphaIndex(i, j+1)];

				// derive by eePosj, eePosjm1 and speed
				for (int k = 0; k < 9; ++k) {
					if(k < 3)
						eePosj(k).deriv() = 1.0;
					else if(k < 6)
						eePosjp1(k-3).deriv() = 1.0;
					else if(k < 7)
						speed.deriv() = 1.0;
					else if(k < 8)
						alphaj.deriv() = 1.0;
					else if(k < 9)
						alphajp1.deriv() = 1.0;

					ScalarDiff energy = computeEnergy(eePosjp1, eePosj, dt, wheelRadiusAD, r, wheelAxisAD, alphaj, alphajp1, speed, c, weight);

					if(k < 3)
						grad[theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + k] += energy.deriv();
					else if(k < 6)
						grad[theMotionPlan->feetPositionsParamsStartIndex + (j+1) * nLimbs * 3 + i * 3 + k-3] += energy.deriv();
					else if(k < 7)
						grad[theMotionPlan->wheelParamsStartIndex + theMotionPlan->nWheelParams*(j*nLimbs + i)] += energy.deriv();
					else if(k < 8)
						grad[theMotionPlan->getWheelAxisAlphaIndex(i, j)] += energy.deriv();
					else if(k < 9)
						grad[theMotionPlan->getWheelAxisAlphaIndex(i, j+1)] += energy.deriv();

					if(k < 3)
						eePosj(k).deriv() = 0.0;
					else if(k < 6)
						eePosjp1(k-3).deriv() = 0.0;
					else if(k < 7)
						speed.deriv() = 0.0;
					else if(k < 8)
						alphaj.deriv() = 0.0;
					else if(k < 9)
						alphajp1.deriv() = 0.0;
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


				// derive by eePosj, eePosjm1 and speed
				for (int k = 0; k < 9; ++k) {
					if(k < 3)
						eePosj(k).deriv().value() = 1.0;
					else if(k < 6)
						eePosjm1(k-3).deriv().value() = 1.0;
					else if(k < 7)
						speed.deriv().value() = 1.0;
					else if(k < 8)
						alphaj.deriv().value() = 1.0;
					else if(k < 9)
						alphajm1.deriv().value() = 1.0;

					int global_k;
					if(k < 3)
						global_k = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + k;
					else if(k < 6)
						global_k = theMotionPlan->feetPositionsParamsStartIndex + (j-1) * nLimbs * 3 + i * 3 + k-3;
					else if(k < 7)
						global_k = theMotionPlan->wheelParamsStartIndex + theMotionPlan->nWheelParams*(j*nLimbs + i);
					else if(k < 8)
						global_k = theMotionPlan->getWheelAxisAlphaIndex(i, j);
					else if(k < 9)
						global_k = theMotionPlan->getWheelAxisAlphaIndex(i, j-1);

					for (int l = 0; l <= k; ++l) {
						if(l < 3)
							eePosj(l).value().deriv() = 1.0;
						else if(l < 6)
							eePosjm1(l-3).value().deriv() = 1.0;
						else if(l < 7)
							speed.value().deriv() = 1.0;
						else if(l < 8)
							alphaj.value().deriv() = 1.0;
						else if(l < 9)
							alphajm1.value().deriv() = 1.0;

						int global_l;
						if(l < 3)
							global_l = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + l;
						else if(l < 6)
							global_l = theMotionPlan->feetPositionsParamsStartIndex + (j-1) * nLimbs * 3 + i * 3 + l-3;
						else if(l < 7)
							global_l = theMotionPlan->wheelParamsStartIndex + theMotionPlan->nWheelParams*(j*nLimbs + i);
						else if(l < 8)
							global_l = theMotionPlan->getWheelAxisAlphaIndex(i, j);
						else if(l < 9)
							global_l = theMotionPlan->getWheelAxisAlphaIndex(i, j-1);

						ScalarDiffDiff energy = computeEnergy(eePosj, eePosjm1, dt, wheelRadiusAD, r, wheelAxisAD, alphaj, alphajm1, speed, c, weight);

						ADD_HES_ELEMENT(hessianEntries,
										global_k,
										global_l,
										energy.deriv().deriv(),
										1.0);

						if(l < 3)
							eePosj(l).value().deriv() = 0.0;
						else if(l < 6)
							eePosjm1(l-3).value().deriv() = 0.0;
						else if(l < 7)
							speed.value().deriv() = 0.0;
						else if(l < 8)
							alphaj.value().deriv() = 0.0;
						else if(l < 9)
							alphajm1.value().deriv() = 0.0;
					}

					if(k < 3)
						eePosj(k).deriv().value() = 0.0;
					else if(k < 6)
						eePosjm1(k-3).deriv().value() = 0.0;
					else if(k < 7)
						speed.deriv().value() = 0.0;
					else if(k < 8)
						alphaj.deriv().value() = 0.0;
					else if(k < 9)
						alphajm1.deriv().value() = 0.0;
				}
			}
			if (j<theMotionPlan->nSamplePoints-1){
				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];

				int iEEjp1 = theMotionPlan->feetPositionsParamsStartIndex + (j+1) * nLimbs * 3 + i * 3;
				Vector3T<ScalarDiffDiff> eePosjp1;
				for (int k = 0; k < 3; ++k)
					eePosjp1(k) = p[iEEjp1 + k];

				ScalarDiffDiff alphajp1 = p[theMotionPlan->getWheelAxisAlphaIndex(i, j+1)];


				// derive by eePosj, eePosjp1 and speed
				for (int k = 0; k < 9; ++k) {
					if(k < 3)
						eePosj(k).deriv().value() = 1.0;
					else if(k < 6)
						eePosjp1(k-3).deriv().value() = 1.0;
					else if(k < 7)
						speed.deriv().value() = 1.0;
					else if(k < 8)
						alphaj.deriv().value() = 1.0;
					else if(k < 9)
						alphajp1.deriv().value() = 1.0;

					int global_k;
					if(k < 3)
						global_k = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + k;
					else if(k < 6)
						global_k = theMotionPlan->feetPositionsParamsStartIndex + (j+1) * nLimbs * 3 + i * 3 + k-3;
					else if(k < 7)
						global_k = theMotionPlan->wheelParamsStartIndex + theMotionPlan->nWheelParams*(j*nLimbs + i);
					else if(k < 8)
						global_k = theMotionPlan->getWheelAxisAlphaIndex(i, j);
					else if(k < 9)
						global_k = theMotionPlan->getWheelAxisAlphaIndex(i, j+1);

					for (int l = 0; l <= k; ++l) {
						if(l < 3)
							eePosj(l).value().deriv() = 1.0;
						else if(l < 6)
							eePosjp1(l-3).value().deriv() = 1.0;
						else if(l < 7)
							speed.value().deriv() = 1.0;
						else if(l < 8)
							alphaj.value().deriv() = 1.0;
						else if(l < 9)
							alphajp1.value().deriv() = 1.0;

						int global_l;
						if(l < 3)
							global_l = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + l;
						else if(l < 6)
							global_l = theMotionPlan->feetPositionsParamsStartIndex + (j+1) * nLimbs * 3 + i * 3 + l-3;
						else if(l < 7)
							global_l = theMotionPlan->wheelParamsStartIndex + theMotionPlan->nWheelParams*(j*nLimbs + i);
						else if(l < 8)
							global_l = theMotionPlan->getWheelAxisAlphaIndex(i, j);
						else if(l < 9)
							global_l = theMotionPlan->getWheelAxisAlphaIndex(i, j+1);

						ScalarDiffDiff energy = computeEnergy(eePosjp1, eePosj, dt, wheelRadiusAD, r, wheelAxisAD, alphaj, alphajp1, speed, c, weight);

						ADD_HES_ELEMENT(hessianEntries,
										global_k,
										global_l,
										energy.deriv().deriv(),
										1.0);

						if(l < 3)
							eePosj(l).value().deriv() = 0.0;
						else if(l < 6)
							eePosjp1(l-3).value().deriv() = 0.0;
						else if(l < 7)
							speed.value().deriv() = 0.0;
						else if(l < 8)
							alphaj.value().deriv() = 0.0;
						else if(l < 9)
							alphajp1.value().deriv() = 0.0;
					}

					if(k < 3)
						eePosj(k).deriv().value() = 0.0;
					else if(k < 6)
						eePosjp1(k-3).deriv().value() = 0.0;
					else if(k < 7)
						speed.deriv().value() = 0.0;
					else if(k < 8)
						alphaj.deriv().value() = 0.0;
					else if(k < 9)
						alphajp1.deriv().value() = 0.0;
				}
			}
		}
	}
}
