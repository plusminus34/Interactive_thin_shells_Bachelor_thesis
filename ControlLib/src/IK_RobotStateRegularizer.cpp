#include <ControlLib/IK_RobotStateRegularizer.h>

IK_RobotStateRegularizer::IK_RobotStateRegularizer(IK_Plan* mp, int startQIndex, int endQIndex, const std::string& objectiveDescription, double weight){
	IKPlan = mp;
	this->startQIndex = startQIndex;
	this->endQIndex = endQIndex;
	this->description = objectiveDescription;
	this->weight = weight;
}

IK_RobotStateRegularizer::~IK_RobotStateRegularizer(void){
}

double IK_RobotStateRegularizer::computeValue(const dVector& p){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//IKPlan->setParametersFromList(p);

	double retVal = 0;
	
	for (int i=startQIndex;i<=endQIndex;i++){	
		double tmpV = (IKPlan->targetRobotState[i] - IKPlan->currentRobotState[i]);
		retVal += 0.5 * tmpV*tmpV * weight;
	}
	
	return retVal;
}

void IK_RobotStateRegularizer::addGradientTo(dVector& grad, const dVector& p){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//IKPlan->setParametersFromList(p);

	//and now compute the gradient with respect to the robot q's
	for (int i=startQIndex; i<=endQIndex; i++)
		if (IKPlan->optimizeRootConfiguration)
			grad[i] += -(IKPlan->targetRobotState[i] - IKPlan->currentRobotState[i]) * weight;
		else
			if (i>=6)
				grad[i-6] += -(IKPlan->targetRobotState[i] - IKPlan->currentRobotState[i]) * weight;
}

void IK_RobotStateRegularizer::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//IKPlan->setParametersFromList(p);

	for (int i=startQIndex;i<=endQIndex;i++)
		if (IKPlan->optimizeRootConfiguration)
			ADD_HES_ELEMENT(hessianEntries, i, i, 1, weight);
		else
			if (i>=6)
				ADD_HES_ELEMENT(hessianEntries, i-6, i-6, 1, weight);
}
