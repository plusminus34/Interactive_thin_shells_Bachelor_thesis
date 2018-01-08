#include <RobotDesignerLib/MPO_WheelTiltObjective.h>


MPO_WheelTiltObjective::MPO_WheelTiltObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_WheelTiltObjective::~MPO_WheelTiltObjective(void){
}

double MPO_WheelTiltObjective::computeValue(const dVector& p){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);
	if (theMotionPlan->wheelParamsStartIndex < 0)
		return 0;
	int nEEs = theMotionPlan->endEffectorTrajectories.size();
	int nSamples = theMotionPlan->nSamplePoints;
	int nWheels = theMotionPlan->nWheels;
	Eigen::Map<const dVector> tilt(p.data() + theMotionPlan->wheelParamsStartIndex+ 2 * nSamples*nWheels, nEEs*nSamples);

	return 0.5*tilt.squaredNorm()*weight;
}

void MPO_WheelTiltObjective::addGradientTo(dVector& grad, const dVector& p) {
	if (theMotionPlan->wheelParamsStartIndex >= 0)
	{
		int nEEs = theMotionPlan->endEffectorTrajectories.size();
		int nSamples = theMotionPlan->nSamplePoints;
		int nWheels = theMotionPlan->nWheels;
		Eigen::Map<const dVector> tilt(p.data() + theMotionPlan->wheelParamsStartIndex+ 2 * nSamples*nWheels, nEEs*nSamples);

		Eigen::Map<dVector> G(grad.data() + theMotionPlan->wheelParamsStartIndex+ 2*nSamples*nWheels, nEEs*nSamples);
		G = tilt*weight;
	}
}

void MPO_WheelTiltObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);
	int nEEs = theMotionPlan->endEffectorTrajectories.size();
	int nSamples = theMotionPlan->nSamplePoints;
	int nWheels = theMotionPlan->nWheels;
	if (theMotionPlan->wheelParamsStartIndex >= 0)
	{
		for (int j = 0; j < nSamples; j++) {
			for (int i = 0; i < nEEs; i++) {
				int I = theMotionPlan->getWheelTiltAngleIndex(i, j);
				ADD_HES_ELEMENT(hessianEntries, I, I, 1, weight);
			}
		}
	}
}
