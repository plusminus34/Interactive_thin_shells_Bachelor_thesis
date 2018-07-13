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

/** ---------------------------------------------------------------------------------------------------- **/

MPO_DefaultEEPosObjective::MPO_DefaultEEPosObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_DefaultEEPosObjective::~MPO_DefaultEEPosObjective(void) {
}

double MPO_DefaultEEPosObjective::computeValue(const dVector& p) {
	// assume the parameters of the motion plan have been set already by the collection of objective functions class
	// theMotionPlan->setMPParametersFromList(p);
	double retVal = 0;
	double wMax = 10000;

	for (int j = 0; j < theMotionPlan->nSamplePoints; j++) {
		double w = wMax;
		//we want the end effector positions to not change at all in the beginning, and thereafter increasingly a bit more, but only if needed...
		double h = theMotionPlan->motionPlanDuration * j / (theMotionPlan->nSamplePoints - 1);
		if (h > 0.05) w = 1;
		for (uint i = 0; i < theMotionPlan->endEffectorTrajectories.size(); i++) {
			V3D error = theMotionPlan->endEffectorTrajectories[i].targetEEPos[j] - theMotionPlan->endEffectorTrajectories[i].EEPos[j];

			retVal += 0.5 * (wMax * error.y() * error.y() + w * error.x() * error.x() + w * error.z() * error.z());
		}
	}
	return retVal * weight;
}

void MPO_DefaultEEPosObjective::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	if (theMotionPlan->feetPositionsParamsStartIndex < 0)
		return;
	int nLimbs = theMotionPlan->endEffectorTrajectories.size();

	double wMax = 10000;

	for (int j = 0; j < theMotionPlan->nSamplePoints; j++) {
		double w = wMax;
		//we want the end effector positions to not change at all in the beginning, and thereafter increasingly a bit more, but only if needed...
		double h = theMotionPlan->motionPlanDuration * j / (theMotionPlan->nSamplePoints - 1);
		if (h > 0.05) w = 1;
		for (uint i = 0; i < theMotionPlan->endEffectorTrajectories.size(); i++) {
			int idx = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3;
			V3D error = theMotionPlan->endEffectorTrajectories[i].targetEEPos[j] - theMotionPlan->endEffectorTrajectories[i].EEPos[j];
			grad[idx + 0] -= weight*w*error.x();
			grad[idx + 1] -= weight*wMax*error.y();
			grad[idx + 2] -= weight*w*error.z();
		}
	}
}

void MPO_DefaultEEPosObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	if (theMotionPlan->feetPositionsParamsStartIndex < 0)
		return;

	int nLimbs = theMotionPlan->endEffectorTrajectories.size();
	double wMax = 10000;
	for (int j = 0; j < theMotionPlan->nSamplePoints; j++) {
		double w = wMax;
		//we want the end effector positions to not change at all in the beginning, and thereafter increasingly a bit more, but only if needed...
		double h = theMotionPlan->motionPlanDuration * j / (theMotionPlan->nSamplePoints - 1);
		if (h > 0.05) w = 1;
		for (uint i = 0; i < theMotionPlan->endEffectorTrajectories.size(); i++) {
			int idx = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3;
			ADD_HES_ELEMENT(hessianEntries, idx + 0, idx + 0, 1, weight*w);
			ADD_HES_ELEMENT(hessianEntries, idx + 1, idx + 1, 1, weight*wMax);
			ADD_HES_ELEMENT(hessianEntries, idx + 2, idx + 2, 1, weight*w);
		}
	}
}


