#include <RobotDesignerLib/MPO_COMPosObjective.h>

MPO_COMPosObjective::MPO_COMPosObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_COMPosObjective::~MPO_COMPosObjective(void){
}

double MPO_COMPosObjective::computeValue(const dVector& p){
	// assume the parameters of the motion plan have been set already by the collection of objective functions class
	// theMotionPlan->setMPParametersFromList(p);
	if (theMotionPlan->COMPositionsParamsStartIndex < 0)
		return 0;
	double retVal = 0;
	for (auto PosObj : theMotionPlan->BodyPosObjectives)
	{
		P3D pos = theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(PosObj->sampleNum);
		retVal += 0.5*(pos - PosObj->pos).squaredNorm();;
	}
	return retVal * weight;
}

void MPO_COMPosObjective::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	if (theMotionPlan->COMPositionsParamsStartIndex < 0)
		return;

	for (auto PosObj : theMotionPlan->BodyPosObjectives)
	{
		int I = theMotionPlan->COMPositionsParamsStartIndex + PosObj->sampleNum * 3;
		P3D pos = theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(PosObj->sampleNum);
		V3D dp = pos - PosObj->pos;
		grad[I] +=   weight*dp(0);
		grad[I+1] += weight*dp(1);
		grad[I+2] += weight*dp(2);
	}
}

void MPO_COMPosObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	if (theMotionPlan->COMPositionsParamsStartIndex < 0)
		return;

	for (auto PosObj : theMotionPlan->BodyPosObjectives)
	{
		int I = theMotionPlan->COMPositionsParamsStartIndex + PosObj->sampleNum * 3;
		ADD_HES_ELEMENT(hessianEntries, I, I, 1, weight);
		ADD_HES_ELEMENT(hessianEntries, I + 1, I + 1, 1, weight);
		ADD_HES_ELEMENT(hessianEntries, I + 2, I + 2, 1, weight);
	}
}
