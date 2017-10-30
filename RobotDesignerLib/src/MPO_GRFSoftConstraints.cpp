#include <RobotDesignerLib/MPO_GRFSoftConstraints.h>

/*****************************************************************************************************
***************************** Lower bound on GRFs - want pushing-only forces *************************
******************************************************************************************************/


MPO_GRFRegularizer::MPO_GRFRegularizer(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_GRFRegularizer::~MPO_GRFRegularizer(void){
}

double MPO_GRFRegularizer::computeValue(const dVector& s){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//theMotionPlan->setMPParametersFromList(s);


	double retVal = 0;
	for (int j=0;j<theMotionPlan->nSamplePoints;j++){
		for (uint i=0;i<theMotionPlan->endEffectorTrajectories.size();i++){
			//we want the vertical component of the GRF to be > 0
			double fVertical = theMotionPlan->endEffectorTrajectories[i].contactForce[j](1);
			double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];
			//if a GRF is not used/useful (e.g. the leg is in swing), then we should encourage a small value for it...
			retVal += 0.5 * theMotionPlan->endEffectorTrajectories[i].contactForce[j].length2() * (1 - c);
		}
	}

/*
	double val = -10;
	for (int i = 0; i < 1000; i++) {
		Logger::logPrint("%lf\t%lf\n", val, suc.computeValue(val));
		val += 0.1;
	}
	exit(0);
*/
	return retVal * weight;
}


void MPO_GRFRegularizer::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
//	theMotionPlan->setMPParametersFromList(p);


	if (theMotionPlan->contactForcesParamsStartIndex >= 0)
		for (int j = 0; j<theMotionPlan->nSamplePoints; j++) {
			for (uint i = 0; i<theMotionPlan->endEffectorTrajectories.size(); i++) {
				//we want the vertical component of the GRF to be > 0
				double fVertical = theMotionPlan->endEffectorTrajectories[i].contactForce[j](1);
				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];


				for (int k = 0; k < 3;k++)
					grad[theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + k] += theMotionPlan->endEffectorTrajectories[i].contactForce[j][k] * (1-c) * weight;

			}
		}
}

void MPO_GRFRegularizer::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	if (theMotionPlan->contactForcesParamsStartIndex >= 0)
		for (int j = 0; j<theMotionPlan->nSamplePoints; j++) {
			for (uint i = 0; i<theMotionPlan->endEffectorTrajectories.size(); i++) {
				//we want the vertical component of the GRF to be > 0
				double fVertical = theMotionPlan->endEffectorTrajectories[i].contactForce[j](1);
				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];

				for (int k = 0; k < 3; k++)
					ADD_HES_ELEMENT(hessianEntries, theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + k, theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + k, (1-c), weight);
			}
		}
}



/*****************************************************************************************************
 ***************************** Upper limit on GRFs ***************************************************
 *****************************************************************************************************/

MPO_GRFSoftBoundConstraints::MPO_GRFSoftBoundConstraints(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_GRFSoftBoundConstraints::~MPO_GRFSoftBoundConstraints(void) {
}

double MPO_GRFSoftBoundConstraints::computeValue(const dVector& s) {
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//theMotionPlan->setMPParametersFromList(s);

	double retVal = 0;
	for (int j = 0; j<theMotionPlan->nSamplePoints; j++) {
		for (uint i = 0; i<theMotionPlan->endEffectorTrajectories.size(); i++) {
			//we want the vertical component of the GRF to be > 0
			double fVertical = theMotionPlan->endEffectorTrajectories[i].contactForce[j](1);
			double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];
			SoftUnilateralConstraint sucVerticalLowerBound = SoftUnilateralConstraint(theMotionPlan->verticalGRFLowerBoundVal, 10, theMotionPlan->GRFEpsilon);
			SoftUnilateralConstraint sucVerticalUpperBound = SoftUnilateralConstraint(-theMotionPlan->endEffectorTrajectories[i].verticalGRFUpperBoundValues[j], 10, theMotionPlan->GRFEpsilon);

			retVal += sucVerticalLowerBound.computeValue(fVertical) * c;
			retVal += sucVerticalUpperBound.computeValue(-fVertical) * c;

			SoftUnilateralConstraint sucVerticalTangentLowerBound = SoftUnilateralConstraint(-theMotionPlan->endEffectorTrajectories[i].tangentGRFBoundValues[j], 10, theMotionPlan->GRFEpsilon);

			// TODO: this is not any different from the lower bound. shouldn't it be '+' instead of '-' ?
			SoftUnilateralConstraint sucVerticalTangentUpperBound = SoftUnilateralConstraint(-theMotionPlan->endEffectorTrajectories[i].tangentGRFBoundValues[j], 10, theMotionPlan->GRFEpsilon);

			retVal += sucVerticalTangentLowerBound.computeValue(theMotionPlan->endEffectorTrajectories[i].contactForce[j](0)) * c;
			retVal += sucVerticalTangentUpperBound.computeValue(-theMotionPlan->endEffectorTrajectories[i].contactForce[j](0)) * c;

			retVal += sucVerticalTangentLowerBound.computeValue(theMotionPlan->endEffectorTrajectories[i].contactForce[j](2)) * c;
			retVal += sucVerticalTangentUpperBound.computeValue(-theMotionPlan->endEffectorTrajectories[i].contactForce[j](2)) * c;

/*
			suc = SoftUnilateralConstraint(-10, 10, 1);
			double val = -50;
			for (int i = 0; i < 201; i++) {
				Logger::logPrint("%lf\t%lf\n", val, suc.computeValue(-val));
				val += 0.5;
			}
			exit(0);
*/
		}
	}

	return retVal * weight;
}


void MPO_GRFSoftBoundConstraints::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	if (theMotionPlan->contactForcesParamsStartIndex >= 0)
		for (int j = 0; j<theMotionPlan->nSamplePoints; j++) {
			for (uint i = 0; i<theMotionPlan->endEffectorTrajectories.size(); i++) {
				//we want the vertical component of the GRF to be > 0
				double fVertical = theMotionPlan->endEffectorTrajectories[i].contactForce[j](1);
				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];

				SoftUnilateralConstraint sucVerticalUpperBound = SoftUnilateralConstraint(-theMotionPlan->endEffectorTrajectories[i].verticalGRFUpperBoundValues[j], 10, theMotionPlan->GRFEpsilon);
				SoftUnilateralConstraint sucVerticalLowerBound = SoftUnilateralConstraint(theMotionPlan->verticalGRFLowerBoundVal, 10, theMotionPlan->GRFEpsilon);

				grad[theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 1] += -1 * sucVerticalUpperBound.computeDerivative(-fVertical) * c * weight;
				grad[theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 1] += sucVerticalLowerBound.computeDerivative(fVertical) * c * weight;

				SoftUnilateralConstraint sucVerticalTangentLowerBound = SoftUnilateralConstraint(-theMotionPlan->endEffectorTrajectories[i].tangentGRFBoundValues[j], 10, theMotionPlan->GRFEpsilon);
				SoftUnilateralConstraint sucVerticalTangentUpperBound = SoftUnilateralConstraint(-theMotionPlan->endEffectorTrajectories[i].tangentGRFBoundValues[j], 10, theMotionPlan->GRFEpsilon);

				grad[theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 0] += sucVerticalTangentLowerBound.computeDerivative(theMotionPlan->endEffectorTrajectories[i].contactForce[j](0)) * c * weight;
				grad[theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 0] += -sucVerticalTangentUpperBound.computeDerivative(-theMotionPlan->endEffectorTrajectories[i].contactForce[j](0)) * c * weight;

				grad[theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 2] += sucVerticalTangentLowerBound.computeDerivative(theMotionPlan->endEffectorTrajectories[i].contactForce[j](2)) * c * weight;
				grad[theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 2] += -sucVerticalTangentUpperBound.computeDerivative(-theMotionPlan->endEffectorTrajectories[i].contactForce[j](2)) * c * weight;

			}
		}
}

void MPO_GRFSoftBoundConstraints::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);
	if (theMotionPlan->contactForcesParamsStartIndex >= 0)
		for (int j = 0; j<theMotionPlan->nSamplePoints; j++) {
			for (uint i = 0; i<theMotionPlan->endEffectorTrajectories.size(); i++) {
				//we want the vertical component of the GRF to be > 0
				double fVertical = theMotionPlan->endEffectorTrajectories[i].contactForce[j](1);
				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];
				SoftUnilateralConstraint sucVerticalUpperBound = SoftUnilateralConstraint(-theMotionPlan->endEffectorTrajectories[i].verticalGRFUpperBoundValues[j], 10, theMotionPlan->GRFEpsilon);
				SoftUnilateralConstraint sucVerticalLowerBound = SoftUnilateralConstraint(theMotionPlan->verticalGRFLowerBoundVal, 10, theMotionPlan->GRFEpsilon);

				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 1, theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 1, sucVerticalUpperBound.computeSecondDerivative(-fVertical) * c, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 1, theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 1, sucVerticalLowerBound.computeSecondDerivative(fVertical) * c, weight);


				SoftUnilateralConstraint sucVerticalTangentLowerBound = SoftUnilateralConstraint(-theMotionPlan->endEffectorTrajectories[i].tangentGRFBoundValues[j], 10, theMotionPlan->GRFEpsilon);
				SoftUnilateralConstraint sucVerticalTangentUpperBound = SoftUnilateralConstraint(-theMotionPlan->endEffectorTrajectories[i].tangentGRFBoundValues[j], 10, theMotionPlan->GRFEpsilon);

				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 0, theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 0, sucVerticalTangentLowerBound.computeSecondDerivative(theMotionPlan->endEffectorTrajectories[i].contactForce[j](0)) * c, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 0, theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 0, sucVerticalTangentUpperBound.computeSecondDerivative(-theMotionPlan->endEffectorTrajectories[i].contactForce[j](0)) * c, weight);

				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 2, theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 2, sucVerticalTangentLowerBound.computeSecondDerivative(theMotionPlan->endEffectorTrajectories[i].contactForce[j](2)) * c, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 2, theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 2, sucVerticalTangentUpperBound.computeSecondDerivative(-theMotionPlan->endEffectorTrajectories[i].contactForce[j](2)) * c, weight);
			}
		}
}


