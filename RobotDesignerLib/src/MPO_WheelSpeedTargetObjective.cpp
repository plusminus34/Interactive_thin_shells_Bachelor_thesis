#include <RobotDesignerLib/MPO_WheelSpeedTargetObjective.h>
#include <iostream>

MPO_WheelSpeedTargetObjective::MPO_WheelSpeedTargetObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, int timeIndex, double targetWheelSpeed, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
	this->timeIndex = timeIndex;
	this->targetWheelSpeed = targetWheelSpeed;
}

MPO_WheelSpeedTargetObjective::~MPO_WheelSpeedTargetObjective(void) {
}

double MPO_WheelSpeedTargetObjective::computeValue(const dVector& s) {

	double retVal = 0;
	for (uint i = 0; i < theMotionPlan->endEffectorTrajectories.size(); ++i) {
		if(theMotionPlan->endEffectorTrajectories[i].isWheel){
			double c = theMotionPlan->endEffectorTrajectories[i].wheelSpeed[timeIndex] - targetWheelSpeed;
			retVal += c*c;
		}
	}

	retVal *= 0.5*weight;
	return retVal;
}

void MPO_WheelSpeedTargetObjective::addGradientTo(dVector& grad, const dVector& p) {

	if (theMotionPlan->wheelParamsStartIndex >= 0){
		for (uint i = 0; i < theMotionPlan->endEffectorTrajectories.size(); ++i) {
			if(theMotionPlan->endEffectorTrajectories[i].isWheel){
				double c = theMotionPlan->endEffectorTrajectories[i].wheelSpeed[timeIndex] - targetWheelSpeed;
				grad[theMotionPlan->getWheelSpeedIndex(i, timeIndex)] += weight*c;
			}
		}
	}
}

void MPO_WheelSpeedTargetObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {

	if (theMotionPlan->wheelParamsStartIndex >= 0){
		for (uint i = 0; i < theMotionPlan->endEffectorTrajectories.size(); ++i) {
			if(theMotionPlan->endEffectorTrajectories[i].isWheel){
				ADD_HES_ELEMENT(hessianEntries,
								theMotionPlan->getWheelSpeedIndex(i, timeIndex),
								theMotionPlan->getWheelSpeedIndex(i, timeIndex),
								1,
								weight);
			}
		}
	}
}
