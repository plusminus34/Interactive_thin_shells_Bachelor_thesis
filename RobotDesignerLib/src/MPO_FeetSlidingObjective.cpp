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
			if (j>0){
				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j] * theMotionPlan->endEffectorTrajectories[i].contactFlag[j-1];

				int iEEj = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3;
				int iEEjm1 = theMotionPlan->feetPositionsParamsStartIndex + (j-1) * nLimbs * 3 + i * 3;

				Eigen::Vector3d eePosj(p[iEEj + 0], p[iEEj + 1], p[iEEj + 2]);
				Eigen::Vector3d eePosjm1(p[iEEjm1 + 0], p[iEEjm1 + 1], p[iEEjm1 + 2]);

				// Angular velocity vector
				double speed = p[theMotionPlan->wheelParamsStartIndex + theMotionPlan->nWheelParams*(j*nLimbs + i)];
				Eigen::Vector3d omega = wheelAxis * speed;

				// Wheel radius
				Eigen::Vector3d r = wheelRadius * wheelRadiusV;

				Eigen::Vector3d constraint = computeConstraint(eePosj, eePosjm1, dt, omega, r);

				retVal += 0.5 * constraint.squaredNorm() * c;
			}
			if (j<theMotionPlan->nSamplePoints-1){
				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];

				int iEEj = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3;
				int iEEjp1 = theMotionPlan->feetPositionsParamsStartIndex + (j+1) * nLimbs * 3 + i * 3;

				Eigen::Vector3d eePosj(p[iEEj + 0], p[iEEj + 1], p[iEEj + 2]);
				Eigen::Vector3d eePosjp1(p[iEEjp1 + 0], p[iEEjp1 + 1], p[iEEjp1 + 2]);

				// Angular velocity vector
				double speed = p[theMotionPlan->wheelParamsStartIndex + theMotionPlan->nWheelParams*(j*nLimbs + i)];
				Eigen::Vector3d omega = wheelAxis * speed;

				// Wheel radius
				Eigen::Vector3d r = wheelRadius * wheelRadiusV;

				Eigen::Vector3d constraint = computeConstraint(eePosjp1, eePosj, dt, omega, r);

				retVal += 0.5 * constraint.squaredNorm() * c;
			}
		}
	}
	return retVal * weight;
}

void MPO_FeetSlidingObjective::addGradientTo(dVector& grad, const dVector& p) {

	if (theMotionPlan->feetPositionsParamsStartIndex < 0 || theMotionPlan->wheelParamsStartIndex < 0)
		return;

	typedef AutoDiffT<double, double> ScalarDiff;

	//and now compute the gradient with respect c and eePos
	const double dt = theMotionPlan->motionPlanDuration / theMotionPlan->nSamplePoints;
	const int nLimbs = theMotionPlan->endEffectorTrajectories.size();

	for (int j=0;j<theMotionPlan->nSamplePoints;j++){
		for (int i=0;i<nLimbs;i++){
			if (j>0){
				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j] * theMotionPlan->endEffectorTrajectories[i].contactFlag[j-1];

				int iEEj = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3;
				int iEEjm1 = theMotionPlan->feetPositionsParamsStartIndex + (j-1) * nLimbs * 3 + i * 3;

				Vector3T<ScalarDiff> eePosj;
				for (int k = 0; k < 3; ++k)
					eePosj(k) = p[iEEj + k];
				Vector3T<ScalarDiff> eePosjm1;
				for (int k = 0; k < 3; ++k)
					eePosjm1(k) = p[iEEjm1 + k];

				// Angular velocity vector
				Vector3T<ScalarDiff> wheelAxisAD;
				for (int k = 0; k < 3; ++k)
					wheelAxisAD(k) = wheelAxis[k];
				ScalarDiff speed(p[theMotionPlan->wheelParamsStartIndex + theMotionPlan->nWheelParams*(j*nLimbs + i)], 0);

				// Wheel radius
				Vector3T<ScalarDiff> wheelRadiusAD;
				for (int k = 0; k < 3; ++k)
					wheelRadiusAD(k) = wheelRadiusV[k];
				ScalarDiff r(wheelRadius, 0);

				// derive by eePosj, eePosjm1 and speed
				for (int k = 0; k < 7; ++k) {
					if(k < 3)
						eePosj(k).deriv() = 1.0;
					else if(k < 6)
						eePosjm1(k-3).deriv() = 1.0;
					else if(k < 7)
						speed.deriv() = 1.0;

					Vector3T<ScalarDiff> rr = wheelRadiusAD*r;
					Vector3T<ScalarDiff> omega = wheelAxisAD*speed;
					Vector3T<ScalarDiff> constraint = computeConstraint(eePosj, eePosjm1, dt, omega, rr);
					ScalarDiff energy = 0.5 * constraint.squaredNorm() * c * weight;

					if(k < 3)
						grad[theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + k] += energy.deriv();
					else if(k < 6)
						grad[theMotionPlan->feetPositionsParamsStartIndex + (j-1) * nLimbs * 3 + i * 3 + k-3] += energy.deriv();
					else if(k < 7)
						grad[theMotionPlan->wheelParamsStartIndex + theMotionPlan->nWheelParams*(j*nLimbs + i)] += energy.deriv();

					if(k < 3)
						eePosj(k).deriv() = 0.0;
					else if(k < 6)
						eePosjm1(k-3).deriv() = 0.0;
					else if(k < 7)
						speed.deriv() = 0.0;
				}
			}
			if (j<theMotionPlan->nSamplePoints-1){
				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];

				int iEEj = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3;
				int iEEjp1 = theMotionPlan->feetPositionsParamsStartIndex + (j+1) * nLimbs * 3 + i * 3;

				Vector3T<ScalarDiff> eePosj;
				for (int k = 0; k < 3; ++k)
					eePosj(k) = p[iEEj + k];
				Vector3T<ScalarDiff> eePosjp1;
				for (int k = 0; k < 3; ++k)
					eePosjp1(k) = p[iEEjp1 + k];

				// Angular velocity vector
				Vector3T<ScalarDiff> wheelAxisAD;
				for (int k = 0; k < 3; ++k)
					wheelAxisAD(k) = wheelAxis[k];
				ScalarDiff speed(p[theMotionPlan->wheelParamsStartIndex + theMotionPlan->nWheelParams*(j*nLimbs + i)], 0);

				// Wheel radius
				Vector3T<ScalarDiff> wheelRadiusAD;
				for (int k = 0; k < 3; ++k)
					wheelRadiusAD(k) = wheelRadiusV[k];
				ScalarDiff r(wheelRadius, 0);

				// derive by eePosj, eePosjm1 and speed
				for (int k = 0; k < 7; ++k) {
					if(k < 3)
						eePosj(k).deriv() = 1.0;
					else if(k < 6)
						eePosjp1(k-3).deriv() = 1.0;
					else if(k < 7)
						speed.deriv() = 1.0;

					Vector3T<ScalarDiff> rr = wheelRadiusAD*r;
					Vector3T<ScalarDiff> omega = wheelAxisAD*speed;
					Vector3T<ScalarDiff> constraint = computeConstraint(eePosjp1, eePosj, dt, omega, rr);
					ScalarDiff energy = 0.5 * constraint.squaredNorm() * c * weight;

					if(k < 3)
						grad[theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + k] += energy.deriv();
					else if(k < 6)
						grad[theMotionPlan->feetPositionsParamsStartIndex + (j+1) * nLimbs * 3 + i * 3 + k-3] += energy.deriv();
					else if(k < 7)
						grad[theMotionPlan->wheelParamsStartIndex + theMotionPlan->nWheelParams*(j*nLimbs + i)] += energy.deriv();

					if(k < 3)
						eePosj(k).deriv() = 0.0;
					else if(k < 6)
						eePosjp1(k-3).deriv() = 0.0;
					else if(k < 7)
						speed.deriv() = 0.0;
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
			if (j>0){
				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j] * theMotionPlan->endEffectorTrajectories[i].contactFlag[j-1];

				int iEEj = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3;
				int iEEjm1 = theMotionPlan->feetPositionsParamsStartIndex + (j-1) * nLimbs * 3 + i * 3;

				Vector3T<ScalarDiffDiff> eePosj;
				for (int k = 0; k < 3; ++k)
					eePosj(k) = p[iEEj + k];
				Vector3T<ScalarDiffDiff> eePosjm1;
				for (int k = 0; k < 3; ++k)
					eePosjm1(k) = p[iEEjm1 + k];

				// Angular velocity vector
				Vector3T<ScalarDiffDiff> wheelAxisAD;
				for (int k = 0; k < 3; ++k)
					wheelAxisAD(k) = wheelAxis[k];
				ScalarDiffDiff speed(p[theMotionPlan->wheelParamsStartIndex + theMotionPlan->nWheelParams*(j*nLimbs + i)], 0);

				// Wheel radius
				Vector3T<ScalarDiffDiff> wheelRadiusAD;
				for (int k = 0; k < 3; ++k)
					wheelRadiusAD(k) = wheelRadiusV[k];
				ScalarDiffDiff r(wheelRadius, 0);

				// derive by eePosj, eePosjm1 and speed
				for (int k = 0; k < 7; ++k) {
					if(k < 3)
						eePosj(k).deriv().value() = 1.0;
					else if(k < 6)
						eePosjm1(k-3).deriv().value() = 1.0;
					else if(k < 7)
						speed.deriv().value() = 1.0;

					int global_k;
					if(k < 3)
						global_k = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + k;
					else if(k < 6)
						global_k = theMotionPlan->feetPositionsParamsStartIndex + (j-1) * nLimbs * 3 + i * 3 + k-3;
					else if(k < 7)
						global_k = theMotionPlan->wheelParamsStartIndex + theMotionPlan->nWheelParams*(j*nLimbs + i);

					for (int l = 0; l <= k; ++l) {
						if(l < 3)
							eePosj(l).value().deriv() = 1.0;
						else if(l < 6)
							eePosjm1(l-3).value().deriv() = 1.0;
						else if(l < 7)
							speed.value().deriv() = 1.0;

						int global_l;
						if(l < 3)
							global_l = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + l;
						else if(l < 6)
							global_l = theMotionPlan->feetPositionsParamsStartIndex + (j-1) * nLimbs * 3 + i * 3 + l-3;
						else if(l < 7)
							global_l = theMotionPlan->wheelParamsStartIndex + theMotionPlan->nWheelParams*(j*nLimbs + i);

						Vector3T<ScalarDiffDiff> rr = wheelRadiusAD*r;
						Vector3T<ScalarDiffDiff> omega = wheelAxisAD*speed;
						Vector3T<ScalarDiffDiff> constraint = computeConstraint(eePosj, eePosjm1, dt, omega, rr);
						ScalarDiffDiff energy = (ScalarDiffDiff)0.5 * constraint.squaredNorm() * c * weight;

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
					}

					if(k < 3)
						eePosj(k).deriv().value() = 0.0;
					else if(k < 6)
						eePosjm1(k-3).deriv().value() = 0.0;
					else if(k < 7)
						speed.deriv().value() = 0.0;
				}
			}
			if (j<theMotionPlan->nSamplePoints-1){
				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];

				int iEEj = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3;
				int iEEjp1 = theMotionPlan->feetPositionsParamsStartIndex + (j+1) * nLimbs * 3 + i * 3;

				Vector3T<ScalarDiffDiff> eePosj;
				for (int k = 0; k < 3; ++k)
					eePosj(k) = p[iEEj + k];
				Vector3T<ScalarDiffDiff> eePosjp1;
				for (int k = 0; k < 3; ++k)
					eePosjp1(k) = p[iEEjp1 + k];

				// Angular velocity vector
				Vector3T<ScalarDiffDiff> wheelAxisAD;
				for (int k = 0; k < 3; ++k)
					wheelAxisAD(k) = wheelAxis[k];
				ScalarDiffDiff speed(p[theMotionPlan->wheelParamsStartIndex + theMotionPlan->nWheelParams*(j*nLimbs + i)], 0);

				// Wheel radius
				Vector3T<ScalarDiffDiff> wheelRadiusAD;
				for (int k = 0; k < 3; ++k)
					wheelRadiusAD(k) = wheelRadiusV[k];
				ScalarDiffDiff r(wheelRadius, 0);

				// derive by eePosj, eePosjp1 and speed
				for (int k = 0; k < 7; ++k) {
					if(k < 3)
						eePosj(k).deriv().value() = 1.0;
					else if(k < 6)
						eePosjp1(k-3).deriv().value() = 1.0;
					else if(k < 7)
						speed.deriv().value() = 1.0;

					int global_k;
					if(k < 3)
						global_k = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + k;
					else if(k < 6)
						global_k = theMotionPlan->feetPositionsParamsStartIndex + (j+1) * nLimbs * 3 + i * 3 + k-3;
					else if(k < 7)
						global_k = theMotionPlan->wheelParamsStartIndex + theMotionPlan->nWheelParams*(j*nLimbs + i);

					for (int l = 0; l <= k; ++l) {
						if(l < 3)
							eePosj(l).value().deriv() = 1.0;
						else if(l < 6)
							eePosjp1(l-3).value().deriv() = 1.0;
						else if(l < 7)
							speed.value().deriv() = 1.0;

						int global_l;
						if(l < 3)
							global_l = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + l;
						else if(l < 6)
							global_l = theMotionPlan->feetPositionsParamsStartIndex + (j+1) * nLimbs * 3 + i * 3 + l-3;
						else if(l < 7)
							global_l = theMotionPlan->wheelParamsStartIndex + theMotionPlan->nWheelParams*(j*nLimbs + i);

						Vector3T<ScalarDiffDiff> rr = wheelRadiusAD*r;
						Vector3T<ScalarDiffDiff> omega = wheelAxisAD*speed;
						Vector3T<ScalarDiffDiff> constraint = computeConstraint(eePosjp1, eePosj, dt, omega, rr);
						ScalarDiffDiff energy = (ScalarDiffDiff)0.5 * constraint.squaredNorm() * c * weight;

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
					}

					if(k < 3)
						eePosj(k).deriv().value() = 0.0;
					else if(k < 6)
						eePosjp1(k-3).deriv().value() = 0.0;
					else if(k < 7)
						speed.deriv().value() = 0.0;
				}
			}
		}
	}
}
