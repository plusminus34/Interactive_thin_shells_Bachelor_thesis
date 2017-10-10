#include <RobotDesignerLib/MPO_BarycentricWeightsRegularizerObjective.h>

MPO_BarycentricWeightsRegularizerObjective::MPO_BarycentricWeightsRegularizerObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_BarycentricWeightsRegularizerObjective::~MPO_BarycentricWeightsRegularizerObjective(void){
}

double MPO_BarycentricWeightsRegularizerObjective::computeValue(const dVector& p){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	double retVal = 0;
	//we want the COM to be as close as possible to the center of the feet that are in contact with the ground, which corresponds to weights as large as possible and equal to each other...

	int end = theMotionPlan->nSamplePoints;
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count...

	for (int j=0;j<end;j++){
		for (uint i=0;i<theMotionPlan->endEffectorTrajectories.size();i++){
			double bw = theMotionPlan->endEffectorTrajectories[i].EEWeights[j];
			//by default, each weight should aim to be as large as possible...
			retVal += 0.5 * (bw - 1)*(bw - 1);
		}
	}
	return retVal * weight;
}

void MPO_BarycentricWeightsRegularizerObjective::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	double retVal = 0;
	int nLimbs = theMotionPlan->endEffectorTrajectories.size();

	//we want the COM to be as close as possible to the center of the feet that are in contact with the ground, which corresponds to weights as large as possible and equal to each other...
	if (theMotionPlan->barycentricWeightsParamsStartIndex>=0){
		int end = theMotionPlan->nSamplePoints;
		if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count...

		for (int j=0;j<end;j++){
			for (int i=0;i<nLimbs;i++){
				double bw = theMotionPlan->endEffectorTrajectories[i].EEWeights[j];
				//by default, each weight should aim to be as large as possible...
				grad[theMotionPlan->barycentricWeightsParamsStartIndex + j * nLimbs + i] += (bw - 1) * weight;
			}
		}
	}
}

void MPO_BarycentricWeightsRegularizerObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	int nLimbs = theMotionPlan->endEffectorTrajectories.size();

	//we want the COM to be as close as possible to the center of the feet that are in contact with the ground, which corresponds to weights as large as possible and equal to each other...
	if (theMotionPlan->barycentricWeightsParamsStartIndex>=0){
		int end = theMotionPlan->nSamplePoints;
		if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count...

		for (int j=0;j<end;j++){
			for (int i=0;i<nLimbs;i++){
				double bw = theMotionPlan->endEffectorTrajectories[i].EEWeights[j];
				//by default, each weight should aim to be as large as possible...
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->barycentricWeightsParamsStartIndex + j * nLimbs + i, theMotionPlan->barycentricWeightsParamsStartIndex + j * nLimbs + i, 1, weight);
			}
		}
	}
}


