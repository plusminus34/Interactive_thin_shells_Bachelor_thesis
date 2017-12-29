#include <RobotDesignerLib/MPO_GRFFrictionConstraints.h>

MPO_GRFFrictionConstraints::MPO_GRFFrictionConstraints(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_GRFFrictionConstraints::~MPO_GRFFrictionConstraints(void) {
}

double MPO_GRFFrictionConstraints::computeValue(const dVector& s) {
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//theMotionPlan->setMPParametersFromList(s);

	double frictionCoeff = theMotionPlan->frictionCoeff;
	if(frictionCoeff < 0)
		return 0;

	double retVal = 0;
	for (int j = 0; j<theMotionPlan->nSamplePoints; j++) {
		for (uint i = 0; i<theMotionPlan->endEffectorTrajectories.size(); i++) {

			double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];
			V3D contactForce = theMotionPlan->endEffectorTrajectories[i].contactForce[j];
			double fn = contactForce(1);
			double fx = std::abs(contactForce(0));
			double fz = std::abs(contactForce(2));

			SoftUnilateralUpperConstraint upperBound = SoftUnilateralUpperConstraint(0, 10, theMotionPlan->frictionEpsilon);

			retVal += upperBound.computeValue(fx-frictionCoeff*fn) * c;
			retVal += upperBound.computeValue(fz-frictionCoeff*fn) * c;
		}
	}

	return retVal * weight;
}


void MPO_GRFFrictionConstraints::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	if (theMotionPlan->contactForcesParamsStartIndex >= 0)
	{
		double frictionCoeff = theMotionPlan->frictionCoeff;
		if(frictionCoeff < 0)
			return;

		for (int j = 0; j<theMotionPlan->nSamplePoints; j++) {
			for (uint i = 0; i<theMotionPlan->endEffectorTrajectories.size(); i++) {

				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];
				V3D contactForce = theMotionPlan->endEffectorTrajectories[i].contactForce[j];
				double fn = contactForce(1);
				double fx = std::abs(contactForce(0));
				double fz = std::abs(contactForce(2));

				SoftUnilateralUpperConstraint upperBound = SoftUnilateralUpperConstraint(0, 10, theMotionPlan->frictionEpsilon);

				int indexGRF = theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i);

				double dux_d = upperBound.computeDerivative(fx-frictionCoeff*fn);
				double dfx_d = (contactForce(0) < 0) ? -1 : 1;
				grad[indexGRF + 0] += dux_d * c * weight * dfx_d;
				grad[indexGRF + 1] -= dux_d * frictionCoeff * c * weight;

				double duz_d = upperBound.computeDerivative(fz-frictionCoeff*fn);
				double dfz_d = (contactForce(2) < 0) ? -1 : 1;
				grad[indexGRF + 1] -= duz_d * frictionCoeff * c * weight;
				grad[indexGRF + 2] += duz_d * c * weight * dfz_d;
			}
		}
	}
}
#include <iostream>
void MPO_GRFFrictionConstraints::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);
	if (theMotionPlan->contactForcesParamsStartIndex >= 0)
	{
		double frictionCoeff = theMotionPlan->frictionCoeff;
		if(frictionCoeff < 0)
			return;

		for (int j = 0; j<theMotionPlan->nSamplePoints; j++) {
			for (uint i = 0; i<theMotionPlan->endEffectorTrajectories.size(); i++) {
				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];
				V3D contactForce = theMotionPlan->endEffectorTrajectories[i].contactForce[j];
				double fn = contactForce(1);
				double fx = std::abs(contactForce(0));
				double fz = std::abs(contactForce(2));

				SoftUnilateralUpperConstraint upperBound = SoftUnilateralUpperConstraint(0, 10, theMotionPlan->frictionEpsilon);

				int indexGRF = theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i);

//				std::cout << "*** indexGRF = "<< indexGRF << std::endl;

				double ddux_dd = upperBound.computeSecondDerivative(fx-frictionCoeff*fn);
				double dfx_d = (contactForce(0) < 0) ? -1 : 1;
				ADD_HES_ELEMENT(hessianEntries,
								indexGRF + 0,
								indexGRF + 0,
								ddux_dd * c, weight); // dfx_d*dfx_d = 1
				ADD_HES_ELEMENT(hessianEntries,
								indexGRF + 0,
								indexGRF + 1,
								-ddux_dd * dfx_d * frictionCoeff * c, weight);
				ADD_HES_ELEMENT(hessianEntries,
								indexGRF + 1,
								indexGRF + 1,
								ddux_dd * frictionCoeff * frictionCoeff * c, weight);

				double dduz_dd = upperBound.computeSecondDerivative(fz-frictionCoeff*fn);
				double dfz_d = (contactForce(2) < 0) ? -1 : 1;
				ADD_HES_ELEMENT(hessianEntries,
								indexGRF + 2,
								indexGRF + 2,
								dduz_dd * c, weight); // dfx_d*dfx_d = 1
				ADD_HES_ELEMENT(hessianEntries,
								indexGRF + 2,
								indexGRF + 1,
								-dduz_dd * dfz_d * frictionCoeff * c, weight);
				ADD_HES_ELEMENT(hessianEntries,
								indexGRF + 1,
								indexGRF + 1,
								dduz_dd * frictionCoeff * frictionCoeff * c, weight);
			}
		}
	}
}
