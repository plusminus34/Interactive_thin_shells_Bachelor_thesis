#include <RobotDesignerLib/MPO_WheelGroundObjective.h>

#include <MathLib/AutoDiff.h>

//TODO: these should be hard constraints, as should the periodic motions...

MPO_WheelGroundObjective::MPO_WheelGroundObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_WheelGroundObjective::~MPO_WheelGroundObjective(void){
}

double MPO_WheelGroundObjective::computeValue(const dVector& p){

	// TODO: for now we assume beta (tilt angle) to be constant
	if (theMotionPlan->feetPositionsParamsStartIndex < 0/* || theMotionPlan->wheelParamsStartIndex < 0*/)
		return 0;

	double retVal = 0;
	const int nLimbs = theMotionPlan->endEffectorTrajectories.size();

	for (int j=0; j<theMotionPlan->nSamplePoints; j++){
		for (uint i=0; i<nLimbs; i++){

			double eePosY = theMotionPlan->endEffectorTrajectories[i].EEPos[j](1);
			double wheelRadius = theMotionPlan->endEffectorTrajectories[i].wheelRadius;
			double beta = theMotionPlan->endEffectorTrajectories[i].wheelAxisBeta[j];
			double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];


			retVal += computeEnergy(eePosY, wheelRadius, beta, c, weight);
		}
	}

	return retVal;
}

void MPO_WheelGroundObjective::addGradientTo(dVector& grad, const dVector& p) {

	if (theMotionPlan->feetPositionsParamsStartIndex < 0/* || theMotionPlan->wheelParamsStartIndex < 0*/)
		return;

	typedef AutoDiffT<double, double> ScalarDiff;

	const int nLimbs = theMotionPlan->endEffectorTrajectories.size();

	for (int j=0; j<theMotionPlan->nSamplePoints; j++){
		for (int i=0; i<nLimbs; i++){

			// Position of foot i at time sample j
			int iEE = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3;
			ScalarDiff eePosY = p[iEE + 1];
			ScalarDiff beta = theMotionPlan->endEffectorTrajectories[i].wheelAxisBeta[j];
			ScalarDiff r = theMotionPlan->endEffectorTrajectories[i].wheelRadius;
			double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j] * theMotionPlan->endEffectorTrajectories[i].contactFlag[j-1];

			DOF<ScalarDiff> dofs[numDOFs];
			// ePosjY
			dofs[0].v = &eePosY;
			dofs[0].i = iEE+1;

			for (int k = 0; k < numDOFs; ++k) {
				dofs[k].v->deriv() = 1.0;
				ScalarDiff energy = computeEnergy(eePosY, r, beta, c, weight);
				grad[dofs[k].i] += energy.deriv();
				dofs[k].v->deriv() = 0.0;
			}
		}
	}
}

void MPO_WheelGroundObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {

	if (theMotionPlan->feetPositionsParamsStartIndex < 0 /*|| theMotionPlan->wheelParamsStartIndex < 0*/)
		return;

	typedef AutoDiffT<double, double> ScalarDiff;
	typedef AutoDiffT<ScalarDiff, ScalarDiff> ScalarDiffDiff;

	const int nLimbs = theMotionPlan->endEffectorTrajectories.size();


	for (int j=0;j<theMotionPlan->nSamplePoints;j++){
		for (int i=0;i<nLimbs;i++){

			int iEE = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3;
			ScalarDiffDiff eePosY = p[iEE + 1];
			ScalarDiffDiff beta = theMotionPlan->endEffectorTrajectories[i].wheelAxisBeta[j];
			ScalarDiffDiff r = theMotionPlan->endEffectorTrajectories[i].wheelRadius;
			double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j] * theMotionPlan->endEffectorTrajectories[i].contactFlag[j-1];

			DOF<ScalarDiffDiff> dofs[numDOFs];
			// ePosjY
			dofs[0].v = &eePosY;
			dofs[0].i = iEE+1;

			for (int k = 0; k < numDOFs; ++k) {

				dofs[k].v->deriv().value() = 1.0;

				for (int l = 0; l <= k; ++l) {
					dofs[l].v->value().deriv() = 1.0;

					ScalarDiffDiff energy = computeEnergy(eePosY, r, beta, c, weight);

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
