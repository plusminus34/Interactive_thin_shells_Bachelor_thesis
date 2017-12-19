#include <RobotDesignerLib/MPO_PeriodicWheelTrajectoriesObjective.h>

MPO_PeriodicWheelTrajectoriesObjective::MPO_PeriodicWheelTrajectoriesObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight, int timeIndex1, int timeIndex2){
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
	this->timeIndex1 = timeIndex1;
	this->timeIndex2 = timeIndex2;
}

MPO_PeriodicWheelTrajectoriesObjective::~MPO_PeriodicWheelTrajectoriesObjective(void){
}

double MPO_PeriodicWheelTrajectoriesObjective::computeValue(const dVector& s){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	double retVal = 0;
	for (int i = 0; i < theMotionPlan->endEffectorTrajectories.size(); ++i) {
		double tmpV = theMotionPlan->endEffectorTrajectories[i].wheelSpeed[timeIndex2] - theMotionPlan->endEffectorTrajectories[i].wheelSpeed[timeIndex1];
		retVal += 0.5 * tmpV*tmpV;
	}
	
	return retVal * weight;
}

void MPO_PeriodicWheelTrajectoriesObjective::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	//and now compute the gradient with respect to the robot q's
	if (theMotionPlan->wheelParamsStartIndex >= 0){
		for (int i = 0; i < theMotionPlan->endEffectorTrajectories.size(); ++i) {
			double tmpV = theMotionPlan->endEffectorTrajectories[i].wheelSpeed[timeIndex2] - theMotionPlan->endEffectorTrajectories[i].wheelSpeed[timeIndex1];
			grad[theMotionPlan->getWheelSpeedIndex(i, timeIndex1)] += -tmpV * weight;
			grad[theMotionPlan->getWheelSpeedIndex(i, timeIndex2)] += tmpV * weight;
		}
	}
}

void MPO_PeriodicWheelTrajectoriesObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	//and now compute the gradient with respect to the robot q's
	if (theMotionPlan->wheelParamsStartIndex >= 0){
		for (int i = 0; i < theMotionPlan->endEffectorTrajectories.size(); ++i) {
			ADD_HES_ELEMENT(hessianEntries, theMotionPlan->getWheelSpeedIndex(i, timeIndex1), theMotionPlan->getWheelSpeedIndex(i, timeIndex1), 1, weight);
			ADD_HES_ELEMENT(hessianEntries, theMotionPlan->getWheelSpeedIndex(i, timeIndex2), theMotionPlan->getWheelSpeedIndex(i, timeIndex2), 1, weight);
			ADD_HES_ELEMENT(hessianEntries, theMotionPlan->getWheelSpeedIndex(i, timeIndex1), theMotionPlan->getWheelSpeedIndex(i, timeIndex2), -1, weight);
		}
	}
}
