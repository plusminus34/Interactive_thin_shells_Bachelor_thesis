#include <RobotDesignerLib/MPO_BodyFrameObjective.h>

MPO_BodyFrameObjective::MPO_BodyFrameObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_BodyFrameObjective::~MPO_BodyFrameObjective(void){
}

double MPO_BodyFrameObjective::computeValue(const dVector& p){
	// assume the parameters of the motion plan have been set already by the collection of objective functions class
	// theMotionPlan->setMPParametersFromList(p);
	if (theMotionPlan->COMPositionsParamsStartIndex < 0)
		return 0;
	double retVal = 0;
	for (auto PosObj : theMotionPlan->BodyFrameObjectives)
	{
		P3D pos = theMotionPlan->bodyTrajectory.getCOMPositionAtTimeIndex(PosObj->sampleNum);
		retVal += 0.5*(pos - PosObj->pos).squaredNorm();
		P3D orientation = theMotionPlan->bodyTrajectory.getCOMEulerAnglesAtTimeIndex(PosObj->sampleNum);
		V3D PermedOrietation = V3D(PosObj->orientation(1), PosObj->orientation(2), PosObj->orientation(0));
		retVal += 0.5*(orientation - PermedOrietation).squaredNorm();
	}
	return retVal * weight;
}

void MPO_BodyFrameObjective::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	if (theMotionPlan->COMPositionsParamsStartIndex < 0)
		return;

	for (auto PosObj : theMotionPlan->BodyFrameObjectives)
	{
		int I = theMotionPlan->COMPositionsParamsStartIndex + PosObj->sampleNum * 3;
		P3D pos = theMotionPlan->bodyTrajectory.getCOMPositionAtTimeIndex(PosObj->sampleNum);
		V3D dp = pos - PosObj->pos;
		grad[I] +=   weight*dp(0);
		grad[I+1] += weight*dp(1);
		grad[I+2] += weight*dp(2);

		I = theMotionPlan->COMOrientationsParamsStartIndex + PosObj->sampleNum * 3;
		P3D orientation = theMotionPlan->bodyTrajectory.getCOMEulerAnglesAtTimeIndex(PosObj->sampleNum);
		V3D PermedOrietation = V3D(PosObj->orientation(1), PosObj->orientation(2), PosObj->orientation(0));
		dp = orientation - PermedOrietation;
		grad[I] += weight * dp(0);
		grad[I + 1] += weight * dp(1);
		grad[I + 2] += weight * dp(2);
	}
}

void MPO_BodyFrameObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	if (theMotionPlan->COMPositionsParamsStartIndex < 0)
		return;

	for (auto PosObj : theMotionPlan->BodyFrameObjectives)
	{
		int I = theMotionPlan->COMPositionsParamsStartIndex + PosObj->sampleNum * 3;
		ADD_HES_ELEMENT(hessianEntries, I, I, 1, weight);
		ADD_HES_ELEMENT(hessianEntries, I + 1, I + 1, 1, weight);
		ADD_HES_ELEMENT(hessianEntries, I + 2, I + 2, 1, weight);

		I = theMotionPlan->COMOrientationsParamsStartIndex + PosObj->sampleNum * 3;
		ADD_HES_ELEMENT(hessianEntries, I, I, 1, weight);
		ADD_HES_ELEMENT(hessianEntries, I + 1, I + 1, 1, weight);
		ADD_HES_ELEMENT(hessianEntries, I + 2, I + 2, 1, weight);
	}
}



/////////////////////////////////////////////////////////////////////
MPO_DesiredBodyTrajectoryObjective::MPO_DesiredBodyTrajectoryObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_DesiredBodyTrajectoryObjective::~MPO_DesiredBodyTrajectoryObjective(void) {
}

double MPO_DesiredBodyTrajectoryObjective::computeValue(const dVector& p) {
	// assume the parameters of the motion plan have been set already by the collection of objective functions class
	// theMotionPlan->setMPParametersFromList(p);
	double retVal = 0;
	for (int i = 0; i < theMotionPlan->nSamplePoints; i++) {
		double w = 1;
		if (i == 0) w = weightAtStart;
		if (i == theMotionPlan->nSamplePoints - 2) w = weightAtEnd;
		P3D pos = theMotionPlan->bodyTrajectory.getCOMPositionAtTimeIndex(i);
		P3D dPos = theMotionPlan->bodyTrajectory.getTargetCOMPositionAtTimeIndex(i);
		retVal += 0.5*(pos - dPos).squaredNorm()*w;
		P3D orientation = theMotionPlan->bodyTrajectory.getCOMEulerAnglesAtTimeIndex(i);
		P3D dOrientation = theMotionPlan->bodyTrajectory.getTargetCOMEulerAnglesAtTimeIndex(i);
		retVal += 0.5*(orientation - dOrientation).squaredNorm()*w;
	}
	return retVal * weight;
}

void MPO_DesiredBodyTrajectoryObjective::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	for (uint i = 0; i < theMotionPlan->nSamplePoints; i++){
		double w = 1;
		if (i == 0) w = weightAtStart;
		if (i == theMotionPlan->nSamplePoints - 2) w = weightAtEnd;
		if (theMotionPlan->COMPositionsParamsStartIndex >= 0) {
			int I = theMotionPlan->COMPositionsParamsStartIndex + i * 3;

			P3D pos = theMotionPlan->bodyTrajectory.getCOMPositionAtTimeIndex(i);
			P3D dPos = theMotionPlan->bodyTrajectory.getTargetCOMPositionAtTimeIndex(i);
			V3D dp = pos - dPos;

			grad[I + 0] += w*weight*dp(0);
			grad[I + 1] += w*weight*dp(1);
			grad[I + 2] += w*weight*dp(2);
		}
		if (theMotionPlan->COMOrientationsParamsStartIndex >= 0) {
			int I = theMotionPlan->COMOrientationsParamsStartIndex + i * 3;
			P3D orientation = theMotionPlan->bodyTrajectory.getCOMEulerAnglesAtTimeIndex(i);
			P3D dOrientation = theMotionPlan->bodyTrajectory.getTargetCOMEulerAnglesAtTimeIndex(i);
			V3D dp = orientation - dOrientation;
			grad[I + 0] += w*weight * dp(0);
			grad[I + 1] += w*weight * dp(1);
			grad[I + 2] += w*weight * dp(2);
		}
	}
}

void MPO_DesiredBodyTrajectoryObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	for (uint i = 0; i < theMotionPlan->nSamplePoints; i++){
		double w = 1;
		if (i == 0) w = weightAtStart;
		if (i == theMotionPlan->nSamplePoints - 2) w = weightAtEnd;

		if (theMotionPlan->COMPositionsParamsStartIndex >= 0) {
			int I = theMotionPlan->COMPositionsParamsStartIndex + i * 3;
			ADD_HES_ELEMENT(hessianEntries, I, I, 1, w*weight);
			ADD_HES_ELEMENT(hessianEntries, I + 1, I + 1, 1, w*weight);
			ADD_HES_ELEMENT(hessianEntries, I + 2, I + 2, 1, w*weight);
		}

		if (theMotionPlan->COMOrientationsParamsStartIndex >= 0) {
			int I = theMotionPlan->COMOrientationsParamsStartIndex + i * 3;
			ADD_HES_ELEMENT(hessianEntries, I, I, 1, w*weight);
			ADD_HES_ELEMENT(hessianEntries, I + 1, I + 1, 1, w*weight);
			ADD_HES_ELEMENT(hessianEntries, I + 2, I + 2, 1, w*weight);
		}
	}
}
