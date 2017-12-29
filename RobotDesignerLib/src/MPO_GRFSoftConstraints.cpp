#include <RobotDesignerLib/MPO_GRFSoftConstraints.h>

/*****************************************************************************************************
***************************** GRF Regularizer when in swing ******************************************
******************************************************************************************************/

//if a GRF is not used/useful (e.g. the leg is in swing), then we should encourage a small value for it...

MPO_GRFSwingRegularizer::MPO_GRFSwingRegularizer(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_GRFSwingRegularizer::~MPO_GRFSwingRegularizer(void){
}

double MPO_GRFSwingRegularizer::computeValue(const dVector& s){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//theMotionPlan->setMPParametersFromList(s);

	double retVal = 0;
	for (int j=0;j<theMotionPlan->nSamplePoints;j++){
		for (uint i=0;i<theMotionPlan->endEffectorTrajectories.size();i++){
			//if a GRF is not used/useful (e.g. the leg is in swing), then we should encourage a small value for it...
			double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];
			retVal += 0.5 * theMotionPlan->endEffectorTrajectories[i].contactForce[j].length2() * (1 - c);
		}
	}

	return retVal * weight;
}


void MPO_GRFSwingRegularizer::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);


	if (theMotionPlan->contactForcesParamsStartIndex >= 0)
		for (int j = 0; j<theMotionPlan->nSamplePoints; j++) {
			for (uint i = 0; i<theMotionPlan->endEffectorTrajectories.size(); i++) {
				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];
				for (int k = 0; k < 3;k++)
					grad[theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + k] += theMotionPlan->endEffectorTrajectories[i].contactForce[j][k] * (1-c) * weight;

			}
		}
}

void MPO_GRFSwingRegularizer::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	if (theMotionPlan->contactForcesParamsStartIndex >= 0)
		for (int j = 0; j<theMotionPlan->nSamplePoints; j++) {
			for (uint i = 0; i<theMotionPlan->endEffectorTrajectories.size(); i++) {
				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];
				for (int k = 0; k < 3; k++)
					ADD_HES_ELEMENT(hessianEntries, theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + k, theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + k, (1-c), weight);
			}
		}
}

/*****************************************************************************************************
***************************** GRF Regularizer when in stance *****************************************
******************************************************************************************************/

MPO_GRFStanceRegularizer::MPO_GRFStanceRegularizer(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_GRFStanceRegularizer::~MPO_GRFStanceRegularizer(void){
}

double MPO_GRFStanceRegularizer::computeValue(const dVector& s){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//theMotionPlan->setMPParametersFromList(s);

	double retVal = 0;
	for (int j=0;j<theMotionPlan->nSamplePoints;j++){
		for (uint i=0;i<theMotionPlan->endEffectorTrajectories.size();i++){
			double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];
			retVal += 0.5 * theMotionPlan->endEffectorTrajectories[i].contactForce[j].length2() * c;
		}
	}

	return retVal * weight;
}


void MPO_GRFStanceRegularizer::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
//	theMotionPlan->setMPParametersFromList(p);


	if (theMotionPlan->contactForcesParamsStartIndex >= 0)
		for (int j = 0; j<theMotionPlan->nSamplePoints; j++) {
			for (uint i = 0; i<theMotionPlan->endEffectorTrajectories.size(); i++) {
				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];
				for (int k = 0; k < 3;k++)
					grad[theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + k] += theMotionPlan->endEffectorTrajectories[i].contactForce[j][k] * c * weight;

			}
		}
}

void MPO_GRFStanceRegularizer::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	if (theMotionPlan->contactForcesParamsStartIndex >= 0)
		for (int j = 0; j<theMotionPlan->nSamplePoints; j++) {
			for (uint i = 0; i<theMotionPlan->endEffectorTrajectories.size(); i++) {
				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];
				for (int k = 0; k < 3; k++)
					ADD_HES_ELEMENT(hessianEntries, theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + k, theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + k, c, weight);
			}
		}
}

/*****************************************************************************************************
***************************** Bound on tangential GRFs ***********************************************
******************************************************************************************************/

MPO_GRFTangentialBoundConstraints::MPO_GRFTangentialBoundConstraints(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_GRFTangentialBoundConstraints::~MPO_GRFTangentialBoundConstraints(void) {
}

double MPO_GRFTangentialBoundConstraints::computeValue(const dVector& s) {
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//theMotionPlan->setMPParametersFromList(s);

	double retVal = 0;
	for (int j = 0; j<theMotionPlan->nSamplePoints; j++) {
		for (uint i = 0; i<theMotionPlan->endEffectorTrajectories.size(); i++) {

			double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];

			SoftUnilateralConstraint sucVerticalTangentLowerBound = SoftUnilateralConstraint(-theMotionPlan->endEffectorTrajectories[i].tangentGRFBoundValues[j], 10, theMotionPlan->GRFEpsilon);
			SoftUnilateralConstraint sucVerticalTangentUpperBound = SoftUnilateralConstraint(-theMotionPlan->endEffectorTrajectories[i].tangentGRFBoundValues[j], 10, theMotionPlan->GRFEpsilon);

			retVal += sucVerticalTangentLowerBound.computeValue(theMotionPlan->endEffectorTrajectories[i].contactForce[j](0)) * c;
			retVal += sucVerticalTangentUpperBound.computeValue(-theMotionPlan->endEffectorTrajectories[i].contactForce[j](0)) * c;

			retVal += sucVerticalTangentLowerBound.computeValue(theMotionPlan->endEffectorTrajectories[i].contactForce[j](2)) * c;
			retVal += sucVerticalTangentUpperBound.computeValue(-theMotionPlan->endEffectorTrajectories[i].contactForce[j](2)) * c;
		}
	}

	return retVal * weight;
}


void MPO_GRFTangentialBoundConstraints::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	if (theMotionPlan->contactForcesParamsStartIndex >= 0)
		for (int j = 0; j<theMotionPlan->nSamplePoints; j++) {
			for (uint i = 0; i<theMotionPlan->endEffectorTrajectories.size(); i++) {

				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];

				SoftUnilateralConstraint sucVerticalTangentLowerBound = SoftUnilateralConstraint(-theMotionPlan->endEffectorTrajectories[i].tangentGRFBoundValues[j], 10, theMotionPlan->GRFEpsilon);
				SoftUnilateralConstraint sucVerticalTangentUpperBound = SoftUnilateralConstraint(-theMotionPlan->endEffectorTrajectories[i].tangentGRFBoundValues[j], 10, theMotionPlan->GRFEpsilon);

				grad[theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 0] += sucVerticalTangentLowerBound.computeDerivative(theMotionPlan->endEffectorTrajectories[i].contactForce[j](0)) * c * weight;
				grad[theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 0] += -sucVerticalTangentUpperBound.computeDerivative(-theMotionPlan->endEffectorTrajectories[i].contactForce[j](0)) * c * weight;

				grad[theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 2] += sucVerticalTangentLowerBound.computeDerivative(theMotionPlan->endEffectorTrajectories[i].contactForce[j](2)) * c * weight;
				grad[theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 2] += -sucVerticalTangentUpperBound.computeDerivative(-theMotionPlan->endEffectorTrajectories[i].contactForce[j](2)) * c * weight;

			}
		}
}

void MPO_GRFTangentialBoundConstraints::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);
	if (theMotionPlan->contactForcesParamsStartIndex >= 0)
		for (int j = 0; j<theMotionPlan->nSamplePoints; j++) {
			for (uint i = 0; i<theMotionPlan->endEffectorTrajectories.size(); i++) {

				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];

				SoftUnilateralConstraint sucVerticalTangentLowerBound = SoftUnilateralConstraint(-theMotionPlan->endEffectorTrajectories[i].tangentGRFBoundValues[j], 10, theMotionPlan->GRFEpsilon);
				SoftUnilateralConstraint sucVerticalTangentUpperBound = SoftUnilateralConstraint(-theMotionPlan->endEffectorTrajectories[i].tangentGRFBoundValues[j], 10, theMotionPlan->GRFEpsilon);

				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 0, theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 0, sucVerticalTangentLowerBound.computeSecondDerivative(theMotionPlan->endEffectorTrajectories[i].contactForce[j](0)) * c, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 0, theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 0, sucVerticalTangentUpperBound.computeSecondDerivative(-theMotionPlan->endEffectorTrajectories[i].contactForce[j](0)) * c, weight);

				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 2, theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 2, sucVerticalTangentLowerBound.computeSecondDerivative(theMotionPlan->endEffectorTrajectories[i].contactForce[j](2)) * c, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 2, theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 2, sucVerticalTangentUpperBound.computeSecondDerivative(-theMotionPlan->endEffectorTrajectories[i].contactForce[j](2)) * c, weight);
			}
		}
}


/*****************************************************************************************************
***************************** Lower bound on GRFs - want pushing-only forces *************************
******************************************************************************************************/

MPO_GRFVerticalLowerBoundConstraints::MPO_GRFVerticalLowerBoundConstraints(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_GRFVerticalLowerBoundConstraints::~MPO_GRFVerticalLowerBoundConstraints(void) {
}

double MPO_GRFVerticalLowerBoundConstraints::computeValue(const dVector& s) {
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//theMotionPlan->setMPParametersFromList(s);

	double retVal = 0;
	for (int j = 0; j<theMotionPlan->nSamplePoints; j++) {
		for (uint i = 0; i<theMotionPlan->endEffectorTrajectories.size(); i++) {

			// we want the vertical component of the GRF to be > 0
			double fVertical = theMotionPlan->endEffectorTrajectories[i].contactForce[j](1);
			double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];

			SoftUnilateralConstraint sucVerticalLowerBound = SoftUnilateralConstraint(theMotionPlan->verticalGRFLowerBoundVal, 10, theMotionPlan->GRFEpsilon);
			retVal += sucVerticalLowerBound.computeValue(fVertical) * c;
		}
	}

	return retVal * weight;
}


void MPO_GRFVerticalLowerBoundConstraints::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	if (theMotionPlan->contactForcesParamsStartIndex >= 0)
		for (int j = 0; j<theMotionPlan->nSamplePoints; j++) {
			for (uint i = 0; i<theMotionPlan->endEffectorTrajectories.size(); i++) {

				// we want the vertical component of the GRF to be > 0
				double fVertical = theMotionPlan->endEffectorTrajectories[i].contactForce[j](1);
				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];

				SoftUnilateralConstraint sucVerticalLowerBound = SoftUnilateralConstraint(theMotionPlan->verticalGRFLowerBoundVal, 10, theMotionPlan->GRFEpsilon);
				grad[theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 1] += sucVerticalLowerBound.computeDerivative(fVertical) * c * weight;
			}
		}
}

void MPO_GRFVerticalLowerBoundConstraints::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);
	if (theMotionPlan->contactForcesParamsStartIndex >= 0)
		for (int j = 0; j<theMotionPlan->nSamplePoints; j++) {
			for (uint i = 0; i<theMotionPlan->endEffectorTrajectories.size(); i++) {

				// we want the vertical component of the GRF to be > 0
				double fVertical = theMotionPlan->endEffectorTrajectories[i].contactForce[j](1);
				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];

				SoftUnilateralConstraint sucVerticalLowerBound = SoftUnilateralConstraint(theMotionPlan->verticalGRFLowerBoundVal, 10, theMotionPlan->GRFEpsilon);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 1, theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 1, sucVerticalLowerBound.computeSecondDerivative(fVertical) * c, weight);
			}
		}
}
