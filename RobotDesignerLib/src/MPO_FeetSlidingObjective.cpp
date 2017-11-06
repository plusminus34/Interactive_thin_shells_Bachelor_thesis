#include <RobotDesignerLib/MPO_FeetSlidingObjective.h>

//TODO: these should be hard constraints, as should the periodic motions...

MPO_FeetSlidingObjective::MPO_FeetSlidingObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_FeetSlidingObjective::~MPO_FeetSlidingObjective(void){
}

double MPO_FeetSlidingObjective::computeValue(const dVector& s){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);


	double retVal = 0;
	for (int j=0;j<theMotionPlan->nSamplePoints;j++){
		for (uint i=0;i<theMotionPlan->endEffectorTrajectories.size();i++){
//			double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];

			//note, because these are foot positions which are not periodic, we shouldn't try to wrap around boundaries...

			//at the boundaries we cannot count foot sliding on "either side", so make up for it...
//			c *= (j==0 || j==theMotionPlan->nSamplePoints-1)?2.0:1.0;

			if (j>0){
				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j] * theMotionPlan->endEffectorTrajectories[i].contactFlag[j-1];
				V3D eeOffset = V3D(theMotionPlan->endEffectorTrajectories[i].EEPos[j], theMotionPlan->endEffectorTrajectories[i].EEPos[j-1]);
				retVal += 0.5 * eeOffset.length2() * c;
			}
			if (j<theMotionPlan->nSamplePoints-1){
				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];// *theMotionPlan->endEffectorTrajectories[i].contactFlag[j + 1];
				V3D eeOffset = V3D(theMotionPlan->endEffectorTrajectories[i].EEPos[j], theMotionPlan->endEffectorTrajectories[i].EEPos[j+1]);
				retVal += 0.5 * eeOffset.length2() * c;				
			}
		}
	}
	return retVal * weight;
}


void MPO_FeetSlidingObjective::addGradientTo(dVector& grad, const dVector& p) {
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

					V3D eeOffset = V3D(theMotionPlan->endEffectorTrajectories[i].EEPos[j], theMotionPlan->endEffectorTrajectories[i].EEPos[j-1]);

					for (int k = 0; k < 3; ++k) {
						grad[theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + k] -= eeOffset[k] * c * weight;
						grad[theMotionPlan->feetPositionsParamsStartIndex + (j-1) * nLimbs * 3 + i * 3 + k] += eeOffset[k] * c * weight;
					}
				}
				if (j<theMotionPlan->nSamplePoints-1){
					double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];// *theMotionPlan->endEffectorTrajectories[i].contactFlag[j + 1];
					V3D eeOffset = V3D(theMotionPlan->endEffectorTrajectories[i].EEPos[j], theMotionPlan->endEffectorTrajectories[i].EEPos[j+1]);

					for (int k = 0; k < 3; ++k) {
						grad[theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + k] -= eeOffset[k] * c * weight;
						grad[theMotionPlan->feetPositionsParamsStartIndex + (j+1) * nLimbs * 3 + i * 3 + k] += eeOffset[k] * c * weight;
					}

				}
			}
		}
	}
}


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


