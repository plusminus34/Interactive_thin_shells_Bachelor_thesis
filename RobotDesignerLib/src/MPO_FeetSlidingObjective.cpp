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
				Eigen::Vector3d omega(0, 0, 1);
				omega *= p[theMotionPlan->wheelParamsStartIndex + theMotionPlan->nWheelParams*(j*nLimbs + i)];

				// Wheel radius
				Eigen::Vector3d radius(0, -1, 0);
				radius *= wheelRadius;

				Eigen::Vector3d constraint = computeConstraint(eePosj, eePosjm1, dt, omega, radius);

				retVal += 0.5 * constraint.squaredNorm() * c;
			}
			if (j<theMotionPlan->nSamplePoints-1){
				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];

				int iEEj = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3;
				int iEEjp1 = theMotionPlan->feetPositionsParamsStartIndex + (j+1) * nLimbs * 3 + i * 3;

				Eigen::Vector3d eePosj(p[iEEj + 0], p[iEEj + 1], p[iEEj + 2]);
				Eigen::Vector3d eePosjp1(p[iEEjp1 + 0], p[iEEjp1 + 1], p[iEEjp1 + 2]);

				// Angular velocity vector
				Eigen::Vector3d omega(0, 0, 1);
				omega *= p[theMotionPlan->wheelParamsStartIndex + theMotionPlan->nWheelParams*(j*nLimbs + i)];

				// Wheel radius
				Eigen::Vector3d radius(0, -1, 0);
				radius *= wheelRadius;

				Eigen::Vector3d constraint = computeConstraint(eePosjp1, eePosj, dt, omega, radius);

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
				Vector3T<ScalarDiff> omega; omega(0) = 0; omega(1) = 0; omega(2) = 1;
				ScalarDiff speed(p[theMotionPlan->wheelParamsStartIndex + theMotionPlan->nWheelParams*(j*nLimbs + i)], 0);

				// Wheel radius
				Vector3T<ScalarDiff> radius; radius(0) = 0; radius(1) = -1; radius(2) = 0;
				ScalarDiff r(wheelRadius, 0);

				// derive by eePosj, eePosjm1 and speed
				for (int k = 0; k < 7; ++k) {
					if(k < 3)
						eePosj(k).deriv() = 1.0;
					else if(k < 6)
						eePosjm1(k-3).deriv() = 1.0;
					else if(k < 7)
						speed.deriv() = 1.0;

					Vector3T<ScalarDiff> rr = radius*r;
					Vector3T<ScalarDiff> ss = omega*speed;
					Vector3T<ScalarDiff> constraint = computeConstraint(eePosj, eePosjm1, dt, ss, rr);
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
				Vector3T<ScalarDiff> omega; omega(0) = 0; omega(1) = 0; omega(2) = 1;
				ScalarDiff speed(p[theMotionPlan->wheelParamsStartIndex + theMotionPlan->nWheelParams*(j*nLimbs + i)], 0);

				// Wheel radius
				Vector3T<ScalarDiff> radius; radius(0) = 0; radius(1) = -1; radius(2) = 0;
				ScalarDiff r(wheelRadius, 0);

				// derive by eePosj, eePosjm1 and speed
				for (int k = 0; k < 7; ++k) {
					if(k < 3)
						eePosj(k).deriv() = 1.0;
					else if(k < 6)
						eePosjp1(k-3).deriv() = 1.0;
					else if(k < 7)
						speed.deriv() = 1.0;

					Vector3T<ScalarDiff> rr = radius*r;
					Vector3T<ScalarDiff> ss = omega*speed;
					Vector3T<ScalarDiff> constraint = computeConstraint(eePosjp1, eePosj, dt, ss, rr);
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

#if 0
void MPO_FeetSlidingObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	//and now compute the gradient with respect c and eePos
	int nLimbs = theMotionPlan->endEffectorTrajectories.size();
	for (int j=0;j<theMotionPlan->nSamplePoints;j++){
		for (int i=0;i<nLimbs;i++){
			if (theMotionPlan->feetPositionsParamsStartIndex >= 0){
//				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];
				//at the boundaries we cannot count foot sliding on "either side", so make up for it...
//				c *= (j==0 || j==theMotionPlan->nSamplePoints-1)?2.0:1.0;
				if (j>0){
					double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j] * theMotionPlan->endEffectorTrajectories[i].contactFlag[j - 1];

					for (int k = 0; k < 3; ++k) {
						ADD_HES_ELEMENT(hessianEntries,
										theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + k,
										theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + k,
										c,
										weight);
						ADD_HES_ELEMENT(hessianEntries,
										theMotionPlan->feetPositionsParamsStartIndex + (j-1) * nLimbs * 3 + i * 3 + k,
										theMotionPlan->feetPositionsParamsStartIndex + (j-1) * nLimbs * 3 + i * 3 + k,
										c,
										weight);
						ADD_HES_ELEMENT(hessianEntries,
										theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + k,
										theMotionPlan->feetPositionsParamsStartIndex + (j-1) * nLimbs * 3 + i * 3 + k,
										-c,
										weight);

					}
				}
				if (j<theMotionPlan->nSamplePoints-1){
					double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];// *theMotionPlan->endEffectorTrajectories[i].contactFlag[j + 1];

					for (int k = 0; k < 3; ++k) {
						ADD_HES_ELEMENT(hessianEntries,
										theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + k,
										theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + k,
										c,
										weight);
						ADD_HES_ELEMENT(hessianEntries,
										theMotionPlan->feetPositionsParamsStartIndex + (j+1) * nLimbs * 3 + i * 3 + k,
										theMotionPlan->feetPositionsParamsStartIndex + (j+1) * nLimbs * 3 + i * 3 + k,
										c,
										weight);
						ADD_HES_ELEMENT(hessianEntries,
										theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + k,
										theMotionPlan->feetPositionsParamsStartIndex + (j+1) * nLimbs * 3 + i * 3 + k,
										-c,
										weight);

					}
				}
			}
		}
	}
}
#endif
