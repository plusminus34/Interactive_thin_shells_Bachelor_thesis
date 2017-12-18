#include <RobotDesignerLib/MPO_WheelAngleRegularizer.h>

MPO_WheelAngleRegularizer::MPO_WheelAngleRegularizer(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight){
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_WheelAngleRegularizer::~MPO_WheelAngleRegularizer(void){
}

double MPO_WheelAngleRegularizer::computeValue(const dVector& s){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	double retVal = 0;
	
	int end = theMotionPlan->nSamplePoints;
//	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count... the last robot pose is already the same as the first one, so no need to penalize twice...

	for (int j=0;j<end;j++){
		int jp = (j+1 > end-1) ? 0 : j+1;
		for (int i = 0; i < theMotionPlan->endEffectorTrajectories.size(); ++i) {
			if(theMotionPlan->endEffectorTrajectories[i].isWheel)
			{
				double tmpV = theMotionPlan->endEffectorTrajectories[i].wheelYawAngle[j] - theMotionPlan->endEffectorTrajectories[i].wheelYawAngle[jp];
				retVal += 0.5 * tmpV*tmpV;

				double tmpW = theMotionPlan->endEffectorTrajectories[i].wheelTiltAngle[j] - theMotionPlan->endEffectorTrajectories[i].wheelTiltAngle[jp];
				retVal += 0.5 * tmpW*tmpW;
			}
		}
	}
	
	return retVal * weight;
}

void MPO_WheelAngleRegularizer::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	int end = theMotionPlan->nSamplePoints;
//	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count... the last robot pose is already the same as the first one, so no need to penalize twice...

	//and now compute the gradient with respect to the robot q's
	if (theMotionPlan->wheelParamsStartIndex >= 0){
		for (int i = 0; i < theMotionPlan->endEffectorTrajectories.size(); ++i)
			if(theMotionPlan->endEffectorTrajectories[i].isWheel)
				for (int j=0;j<end;j++)
				{
					int jp = (j+1 > end-1) ? 0 : j+1;
					double tmpV = theMotionPlan->endEffectorTrajectories[i].wheelYawAngle[j] - theMotionPlan->endEffectorTrajectories[i].wheelYawAngle[jp];
					grad[theMotionPlan->getWheelYawAngleIndex(i, j)] += tmpV * weight;
					grad[theMotionPlan->getWheelYawAngleIndex(i, jp)] += -tmpV * weight;
					double tmpW = theMotionPlan->endEffectorTrajectories[i].wheelTiltAngle[j] - theMotionPlan->endEffectorTrajectories[i].wheelTiltAngle[jp];
					grad[theMotionPlan->getWheelTiltAngleIndex(i, j)] += tmpW * weight;
					grad[theMotionPlan->getWheelTiltAngleIndex(i, jp)] += -tmpW * weight;
				}
	}
}

void MPO_WheelAngleRegularizer::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	int end = theMotionPlan->nSamplePoints;
//	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count... the last robot pose is already the same as the first one, so no need to penalize twice...

	//and now compute the gradient with respect to the robot q's
	if (theMotionPlan->wheelParamsStartIndex >= 0){
		for (int i = 0; i < theMotionPlan->endEffectorTrajectories.size(); ++i)
			if(theMotionPlan->endEffectorTrajectories[i].isWheel)
				for (int j=0;j<end;j++)
				{
					int jp = (j+1 > end-1) ? 0 : j+1;
					{
						int global_j = theMotionPlan->getWheelYawAngleIndex(i, j);
						int global_jp = theMotionPlan->getWheelYawAngleIndex(i, jp);
						ADD_HES_ELEMENT(hessianEntries, global_j, global_j, 1, weight);
						ADD_HES_ELEMENT(hessianEntries, global_j, global_jp, -1, weight);
						ADD_HES_ELEMENT(hessianEntries, global_jp, global_jp, 1, weight);
					}
					{
						int global_j = theMotionPlan->getWheelTiltAngleIndex(i, j);
						int global_jp = theMotionPlan->getWheelTiltAngleIndex(i, jp);
						ADD_HES_ELEMENT(hessianEntries, global_j, global_j, 1, weight);
						ADD_HES_ELEMENT(hessianEntries, global_j, global_jp, -1, weight);
						ADD_HES_ELEMENT(hessianEntries, global_jp, global_jp, 1, weight);
					}
				}
	}
}
