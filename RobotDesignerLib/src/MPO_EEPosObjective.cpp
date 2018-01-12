#include <RobotDesignerLib/MPO_EEPosObjective.h>

MPO_EEPosObjective::MPO_EEPosObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_EEPosObjective::~MPO_EEPosObjective(void){
}

double MPO_EEPosObjective::computeValue(const dVector& p){
	// assume the parameters of the motion plan have been set already by the collection of objective functions class
	// theMotionPlan->setMPParametersFromList(p);
	if (theMotionPlan->feetPositionsParamsStartIndex < 0)
		return 0;
	double retVal = 0;
	for (auto PosObj : theMotionPlan->EEPosObjectives)
	{
		P3D pos = theMotionPlan->endEffectorTrajectories[PosObj->endEffectorInd].EEPos[PosObj->sampleNum];
		double dx = pos(0) - PosObj->pos(0);
		double dz = pos(2) - PosObj->pos(2);
		retVal += 0.5*(dx*dx + dz*dz);
	}
	return retVal * weight;
}

void MPO_EEPosObjective::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	if (theMotionPlan->feetPositionsParamsStartIndex < 0)
		return;
	int nLimbs = theMotionPlan->endEffectorTrajectories.size();

	for (auto PosObj : theMotionPlan->EEPosObjectives)
	{
		int I = theMotionPlan->feetPositionsParamsStartIndex + PosObj->sampleNum * nLimbs * 3 + PosObj->endEffectorInd * 3;
		P3D pos = theMotionPlan->endEffectorTrajectories[PosObj->endEffectorInd].EEPos[PosObj->sampleNum];
		double dx = pos(0) - PosObj->pos(0);
		double dz = pos(2) - PosObj->pos(2);
		grad[I] += weight*dx;
		grad[I+2] += weight*dz;
	}
}

void MPO_EEPosObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	if (theMotionPlan->feetPositionsParamsStartIndex < 0)
		return;
	int nLimbs = theMotionPlan->endEffectorTrajectories.size();

	for (auto PosObj : theMotionPlan->EEPosObjectives)
	{
		int I = theMotionPlan->feetPositionsParamsStartIndex + PosObj->sampleNum * nLimbs * 3 + PosObj->endEffectorInd * 3;
		ADD_HES_ELEMENT(hessianEntries, I, I, 1, weight);
		ADD_HES_ELEMENT(hessianEntries, I+2, I+2, 1, weight);
	}
}
