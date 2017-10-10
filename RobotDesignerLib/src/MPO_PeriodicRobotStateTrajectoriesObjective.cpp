#include <RobotDesignerLib/MPO_PeriodicRobotStateTrajectoriesObjective.h>

MPO_PeriodicRobotStateTrajectoriesObjective::MPO_PeriodicRobotStateTrajectoriesObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight, int timeIndex1, int timeIndex2, int startQIndex, int endQIndex){
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
	this->startQIndex = startQIndex;
	this->endQIndex = endQIndex;
	this->timeIndex1 = timeIndex1;
	this->timeIndex2 = timeIndex2;
}

MPO_PeriodicRobotStateTrajectoriesObjective::~MPO_PeriodicRobotStateTrajectoriesObjective(void){
}

double MPO_PeriodicRobotStateTrajectoriesObjective::computeValue(const dVector& s){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	double retVal = 0;
	
	//we want the COM to be as close as possible to the weighted position of the grounded feet (sum_i c_i*w_i*p_i=com_p)
	for (int i=startQIndex;i<=endQIndex;i++){
		double tmpV = (theMotionPlan->robotStateTrajectory.qArray[timeIndex2][i] - theMotionPlan->robotStateTrajectory.qArray[timeIndex1][i]);
		retVal += 0.5 * tmpV*tmpV;
	}
	
	return retVal * weight;
}

void MPO_PeriodicRobotStateTrajectoriesObjective::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	//and now compute the gradient with respect to the robot q's
	if (theMotionPlan->robotStatesParamsStartIndex >= 0){
		for (int i=startQIndex;i<=endQIndex;i++){
			double tmpV = (theMotionPlan->robotStateTrajectory.qArray[timeIndex2][i] - theMotionPlan->robotStateTrajectory.qArray[timeIndex1][i]);
			grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * timeIndex1 + i] += -tmpV * weight;
			grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * timeIndex2 + i] += tmpV * weight;
		}
	}
}

void MPO_PeriodicRobotStateTrajectoriesObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	//and now compute the gradient with respect to the robot q's
	if (theMotionPlan->robotStatesParamsStartIndex >= 0){
		for (int i=startQIndex;i<=endQIndex;i++){
			ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * timeIndex1 + i, theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * timeIndex1 + i, 1, weight);
			ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * timeIndex2 + i, theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * timeIndex2 + i, 1, weight);
			ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * timeIndex1 + i, theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * timeIndex2 + i, -1, weight);
		}
	}
}


/*******************************************************************************************/

MPO_COMTrajectoryObjective::MPO_COMTrajectoryObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight, int timeIndex1, int timeIndex2) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
	this->timeIndex1 = timeIndex1;
	this->timeIndex2 = timeIndex2;
}

MPO_COMTrajectoryObjective::~MPO_COMTrajectoryObjective(void) {
}

double MPO_COMTrajectoryObjective::computeValue(const dVector& s) {
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	double retVal = 0;

	V3D errV = theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(timeIndex2) - theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(timeIndex1);
	retVal += 0.5 * errV.length2();

	//we also want the COM, on average, to not move much...
	P3D avgCOMPos;
	for (int i = 0; i < theMotionPlan->nSamplePoints; i++)
		avgCOMPos += theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(i) / theMotionPlan->nSamplePoints;
	V3D comPosErr(theMotionPlan->defaultCOMPosition, avgCOMPos);

	retVal += 0.5 * comPosErr.length2();


	return retVal * weight;
}

void MPO_COMTrajectoryObjective::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	//and now compute the gradient with respect to the robot q's
	if (theMotionPlan->COMPositionsParamsStartIndex >= 0) {
		V3D errV = theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(timeIndex2) - theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(timeIndex1);
		for (int i = 0; i < 3;i++){
			grad[theMotionPlan->COMPositionsParamsStartIndex + 3 * timeIndex2 + i] += errV[i] * weight;
			grad[theMotionPlan->COMPositionsParamsStartIndex + 3 * timeIndex1 + i] -= errV[i] * weight;
		}

		P3D avgCOMPos;
		for (int i = 0; i < theMotionPlan->nSamplePoints; i++)
			avgCOMPos += theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(i) / theMotionPlan->nSamplePoints;
		V3D comPosErr(theMotionPlan->defaultCOMPosition, avgCOMPos);

		for (int j = 0; j < 3; j++)
			for (int i = 0; i < theMotionPlan->nSamplePoints; i++)
				grad[theMotionPlan->COMPositionsParamsStartIndex + 3 * i + j] += comPosErr[j] * (1.0 / theMotionPlan->nSamplePoints) * weight;
	}
}

void MPO_COMTrajectoryObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	if (theMotionPlan->COMPositionsParamsStartIndex >= 0) {
		for (int i = 0; i < 3; i++) {
			ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * timeIndex2 + i, theMotionPlan->COMPositionsParamsStartIndex + 3 * timeIndex2 + i, 1, weight);
			ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * timeIndex1 + i, theMotionPlan->COMPositionsParamsStartIndex + 3 * timeIndex1 + i, 1, weight);
			ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * timeIndex1 + i, theMotionPlan->COMPositionsParamsStartIndex + 3 * timeIndex2 + i, -1, weight);
		}

		for (int k = 0; k < 3; k++)
			for (int i = 0; i < theMotionPlan->nSamplePoints; i++)
				for (int j = i; j < theMotionPlan->nSamplePoints; j++)
					ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * i + k, theMotionPlan->COMPositionsParamsStartIndex + 3 * j + k, (1.0 / (theMotionPlan->nSamplePoints * theMotionPlan->nSamplePoints)), weight);
	}
}


