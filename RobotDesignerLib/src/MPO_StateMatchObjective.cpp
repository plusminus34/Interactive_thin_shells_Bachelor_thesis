#include "RobotDesignerLib/MPO_StateMatchObjective.h"

//some problems:
//- if initial robot state is not made such that all EEs hit the ground, then it will pose major conflicts with other objectives.
//- are F=ma and angular equivalent not active when the motion is not periodic? Or why are there no GRFs?


MPO_StateMatchObjective::MPO_StateMatchObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight, int stateIndex, dVector& targetRobotState) {
	theMotionPlan = mp;

	//altogether, these indices provide a yaw orientation and plane translation independent way of measuring differences to target state... 
//	qIndices.push_back(1);	//this is the height of the robot's body
//	qIndices.push_back(4);	//the roll
//	qIndices.push_back(5);	//the pitch

	for (int i = 0; i < mp->robot->getJointCount(); i++)	// add all the joint angles now...
		qIndices.push_back(6 + i);

	this->description = objectiveDescription;
	this->weight = weight;
	this->targetRobotState = targetRobotState;
	this->stateIndex = stateIndex;
}

MPO_StateMatchObjective::~MPO_StateMatchObjective(void) {
}

double MPO_StateMatchObjective::computeValue(const dVector& s) {
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	double retVal = 0;
													   //we want the COM to be as close as possible to the weighted position of the grounded feet (sum_i c_i*w_i*p_i=com_p)
	for (uint j = 0; j < qIndices.size(); j++) {
		int i = qIndices[j];
		double tmpV = theMotionPlan->robotStateTrajectory.qArray[stateIndex][i] - targetRobotState[i];
		retVal += 0.5 * tmpV*tmpV;
	}

	return retVal * weight;
}

void MPO_StateMatchObjective::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);
															   //and now compute the gradient with respect to the robot q's
	if (theMotionPlan->robotStatesParamsStartIndex >= 0) {
		for (uint j = 0; j < qIndices.size(); j++) {
			int i = qIndices[j];
			grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * stateIndex + i] += (theMotionPlan->robotStateTrajectory.qArray[stateIndex][i] - targetRobotState[i]) * weight;
		}
	}
}

void MPO_StateMatchObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	if (theMotionPlan->robotStatesParamsStartIndex >= 0) {
		for (uint j = 0; j < qIndices.size(); j++) {
			int i = qIndices[j];
			ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * stateIndex + i, theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * stateIndex + i, 1, weight);
		}
	}
}

