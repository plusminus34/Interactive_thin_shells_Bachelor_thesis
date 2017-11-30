#include <RobotDesignerLib/MPO_PseudoPeriodicEECOMPoseConstraint.h>

MPO_PseudoPeriodicEECOMPoseConstraint::MPO_PseudoPeriodicEECOMPoseConstraint(LocomotionEngineMotionPlan * mp, const std::string & objectiveDescription, double weight){
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_PseudoPeriodicEECOMPoseConstraint::~MPO_PseudoPeriodicEECOMPoseConstraint(void){
}

double MPO_PseudoPeriodicEECOMPoseConstraint::computeValue(const dVector & p){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//theMotionPlan->setMPParametersFromList(s);

	double retVal = 0;
	uint timeIndexstart = theMotionPlan->nSamplePoints - 1;
	uint timeIndexend = theMotionPlan->wrapAroundBoundaryIndex;

	for (uint i = 0; i < theMotionPlan->endEffectorTrajectories.size(); i++) {
		V3D temp=(theMotionPlan->endEffectorTrajectories[i].EEPos[timeIndexend] - theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(timeIndexend)) -
			(theMotionPlan->endEffectorTrajectories[i].EEPos[timeIndexstart] - theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(timeIndexstart));
				
		retVal += temp.length2();
	}

	return retVal*weight;
}

void MPO_PseudoPeriodicEECOMPoseConstraint::addGradientTo(dVector & grad, const dVector & p){
	if (theMotionPlan->contactForcesParamsStartIndex >= 0) {//Cause we first optimize Contact first only, we need to do nothing here if =-1
		uint timeIndexstart = theMotionPlan->nSamplePoints - 1;
		uint timeIndexend = theMotionPlan->wrapAroundBoundaryIndex;
		int nLimbs = theMotionPlan->endEffectorTrajectories.size();
		V3D dFdcom=V3D();
		
		for (uint i = 0; i < theMotionPlan->endEffectorTrajectories.size(); i++) {
			V3D temp = (theMotionPlan->endEffectorTrajectories[i].EEPos[timeIndexend] - theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(timeIndexend)) -
				(theMotionPlan->endEffectorTrajectories[i].EEPos[timeIndexstart] - theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(timeIndexstart));
			
			if (theMotionPlan->feetPositionsParamsStartIndex >= 0) {
				//dFdEE_end
				grad[theMotionPlan->feetPositionsParamsStartIndex + timeIndexend * nLimbs * 3 + i * 3 + 0] += 2 * temp[0] * weight;
				grad[theMotionPlan->feetPositionsParamsStartIndex + timeIndexend * nLimbs * 3 + i * 3 + 1] += 2 * temp[1] * weight;
				grad[theMotionPlan->feetPositionsParamsStartIndex + timeIndexend * nLimbs * 3 + i * 3 + 2] += 2 * temp[2] * weight;

				//dFdEE_start
				grad[theMotionPlan->feetPositionsParamsStartIndex + timeIndexstart * nLimbs * 3 + i * 3 + 0] += -2 * temp[0] * weight;
				grad[theMotionPlan->feetPositionsParamsStartIndex + timeIndexstart * nLimbs * 3 + i * 3 + 1] += -2 * temp[1] * weight;
				grad[theMotionPlan->feetPositionsParamsStartIndex + timeIndexstart * nLimbs * 3 + i * 3 + 2] += -2 * temp[2] * weight;
			}

			dFdcom += temp;
		}

		if (theMotionPlan->COMPositionsParamsStartIndex >= 0) {
			//dFdCom_end
			grad[theMotionPlan->COMPositionsParamsStartIndex + 3 * timeIndexend + 0] += -2 * dFdcom[0] * weight;
			grad[theMotionPlan->COMPositionsParamsStartIndex + 3 * timeIndexend + 1] += -2 * dFdcom[1] * weight;
			grad[theMotionPlan->COMPositionsParamsStartIndex + 3 * timeIndexend + 2] += -2 * dFdcom[2] * weight;

			//dFdCom_start
			grad[theMotionPlan->COMPositionsParamsStartIndex + 3 * timeIndexstart + 0] += 2 * dFdcom[0] * weight;
			grad[theMotionPlan->COMPositionsParamsStartIndex + 3 * timeIndexstart + 1] += 2 * dFdcom[1] * weight;
			grad[theMotionPlan->COMPositionsParamsStartIndex + 3 * timeIndexstart + 2] += 2 * dFdcom[2] * weight;
		}
	}
}

void MPO_PseudoPeriodicEECOMPoseConstraint::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector & p){
	if (theMotionPlan->contactForcesParamsStartIndex >= 0) {
		int nLimbs = theMotionPlan->endEffectorTrajectories.size();
		Matrix3x3 I; I.setIdentity();

 		for (int s = 0; s <= 1; s++) {
			for (int t = 0; t <= 1; t++) {
				double sign_s = 2 * s - 1; // s or t=0   sign_s or t=-1   time start
				double sign_t = 2 * t - 1; // s or t=1   sign_s or t=+1   time end
				double signterm = sign_s*sign_t;

				//s or t == 0 ->theMotionPlan->wrapAroundBoundaryIndex (start);
				uint timeIndex_s = (s == 0) ? theMotionPlan->wrapAroundBoundaryIndex : theMotionPlan->nSamplePoints - 1;
				uint timeIndex_t = (t == 0) ? theMotionPlan->wrapAroundBoundaryIndex : theMotionPlan->nSamplePoints - 1;

				// d/dq_s  (dF/dq_t)

				for (int i = 0; i < nLimbs; i++) {

					//FD asks me to do this @@...
					if (s == 1 && t == 0) {} // MGSTUCK: why? or: what is s and t
					else 
					{
						//dsame
						for (uint j = 0; j < 3; j++) {
							for (uint k = 0; k < 3; k++) {


								if (theMotionPlan->COMPositionsParamsStartIndex >= 0)
									ADD_HES_ELEMENT(hessianEntries,
										theMotionPlan->COMPositionsParamsStartIndex + 3 * timeIndex_s + j,
										theMotionPlan->COMPositionsParamsStartIndex + 3 * timeIndex_t + k,
										signterm * 2 * I(j, k), weight);


								if (theMotionPlan->feetPositionsParamsStartIndex >= 0) {
									ADD_HES_ELEMENT(hessianEntries,
										theMotionPlan->feetPositionsParamsStartIndex + timeIndex_s * nLimbs * 3 + i * 3 + j,
										theMotionPlan->feetPositionsParamsStartIndex + timeIndex_t * nLimbs * 3 + i * 3 + k,
										signterm * 2 * I(j, k), weight);
								}
							}
						}
					}
										
					//ddifferent  
					for (uint j = 0; j < 3; j++) {
						for (uint k = 0; k < 3; k++) {
							if (theMotionPlan->feetPositionsParamsStartIndex >= 0 && theMotionPlan->COMPositionsParamsStartIndex >= 0) {
								ADD_HES_ELEMENT(hessianEntries,
									theMotionPlan->COMPositionsParamsStartIndex + 3 * timeIndex_s + j,
									theMotionPlan->feetPositionsParamsStartIndex + timeIndex_t * nLimbs * 3 + i * 3 + k,
									-signterm * 2 * I(j, k), weight);

								//FD asks me to comment this @@...
								/*ADD_HES_ELEMENT(hessianEntries,
									theMotionPlan->feetPositionsParamsStartIndex + timeIndex_t * nLimbs * 2 + i * 2 + k,
									theMotionPlan->COMPositionsParamsStartIndex + 3 * timeIndex_s + j,
									-signterm * 2 * I(kk, j), weight);*/
							}
						}
					}
				}
			}		
		}
	}
}


