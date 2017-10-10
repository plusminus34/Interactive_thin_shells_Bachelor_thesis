#include <RobotDesignerLib/MPO_RobotEndEffectorsObjective.h>

MPO_RobotEndEffectorsObjective::MPO_RobotEndEffectorsObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_RobotEndEffectorsObjective::~MPO_RobotEndEffectorsObjective(void){
}

double MPO_RobotEndEffectorsObjective::computeValue(const dVector& p){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	double retVal = 0;
	int nLimbs = theMotionPlan->endEffectorTrajectories.size();
	for (int j=0;j<theMotionPlan->nSamplePoints;j++){
		dVector q_t;
		theMotionPlan->robotStateTrajectory.getQAtTimeIndex(j, q_t);
		theMotionPlan->robotRepresentation->setQ(q_t);
		for (int i=0;i<nLimbs;i++){
			V3D err(theMotionPlan->endEffectorTrajectories[i].EEPos[j], theMotionPlan->robotRepresentation->getWorldCoordinatesFor(theMotionPlan->endEffectorTrajectories[i].endEffectorLocalCoords, theMotionPlan->endEffectorTrajectories[i].endEffectorRB));
			retVal += 0.5 * err.length2();
		}
	}

	return retVal * weight;
}

void MPO_RobotEndEffectorsObjective::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	MatrixNxM dEndEffectordq;

	int nLimbs = theMotionPlan->endEffectorTrajectories.size();
	for (int j=0;j<theMotionPlan->nSamplePoints;j++){
		dVector q_t;
		theMotionPlan->robotStateTrajectory.getQAtTimeIndex(j, q_t);
		theMotionPlan->robotRepresentation->setQ(q_t);

		for (int i=0;i<nLimbs;i++){
			V3D err(theMotionPlan->endEffectorTrajectories[i].EEPos[j], theMotionPlan->robotRepresentation->getWorldCoordinatesFor(theMotionPlan->endEffectorTrajectories[i].endEffectorLocalCoords, theMotionPlan->endEffectorTrajectories[i].endEffectorRB));

			//compute the gradient with respect to the feet locations
			if (theMotionPlan->feetPositionsParamsStartIndex >= 0){
				grad[theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 2 + i * 2 + 0] += -err[0]*weight;
				grad[theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 2 + i * 2 + 1] += -err[2]*weight;
			}

			//and now compute the gradient with respect to the robot q's
			if (theMotionPlan->robotStatesParamsStartIndex >= 0){
				theMotionPlan->robotRepresentation->compute_dpdq(theMotionPlan->endEffectorTrajectories[i].endEffectorLocalCoords, theMotionPlan->endEffectorTrajectories[i].endEffectorRB, dEndEffectordq);
	
				//dEdee * deedq = dEdq
				for (int k=0;k<3;k++)
					for (int l=0;l<theMotionPlan->robotRepresentation->getDimensionCount();l++)
						grad[theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + l] += dEndEffectordq(k, l) * err[k] * weight;
			}
		}
	}

}

void MPO_RobotEndEffectorsObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	MatrixNxM dEndEffectordq, ddEndEffectordq_dqi;

	int nLimbs = theMotionPlan->endEffectorTrajectories.size();
	for (int j=0;j<theMotionPlan->nSamplePoints;j++){
		dVector q_t;
		theMotionPlan->robotStateTrajectory.getQAtTimeIndex(j, q_t);
		theMotionPlan->robotRepresentation->setQ(q_t);

		for (int i=0;i<nLimbs;i++){
			V3D err(theMotionPlan->endEffectorTrajectories[i].EEPos[j], theMotionPlan->robotRepresentation->getWorldCoordinatesFor(theMotionPlan->endEffectorTrajectories[i].endEffectorLocalCoords, theMotionPlan->endEffectorTrajectories[i].endEffectorRB));

			//compute the gradient with respect to the feet locations
			if (theMotionPlan->feetPositionsParamsStartIndex >= 0){
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 2 + i * 2 + 0, theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 2 + i * 2 + 0, 1, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 2 + i * 2 + 1, theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 2 + i * 2 + 1, 1, weight);
			}


			//and now compute the gradient with respect to the robot q's
			if (theMotionPlan->robotStatesParamsStartIndex >= 0){
				theMotionPlan->robotRepresentation->compute_dpdq(theMotionPlan->endEffectorTrajectories[i].endEffectorLocalCoords, theMotionPlan->endEffectorTrajectories[i].endEffectorRB, dEndEffectordq);

				for (int k=0;k<theMotionPlan->robotRepresentation->getDimensionCount();k++){
					bool hasNonZeros = theMotionPlan->robotRepresentation->compute_ddpdq_dqi(theMotionPlan->endEffectorTrajectories[i].endEffectorLocalCoords, theMotionPlan->endEffectorTrajectories[i].endEffectorRB, ddEndEffectordq_dqi, k);
					if (hasNonZeros == false) continue;
					for (int l=k;l<theMotionPlan->robotRepresentation->getDimensionCount();l++)
						for (int m=0;m<3;m++){
							double val = ddEndEffectordq_dqi(m, l) * err[m];
							ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + k, theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + l, val, weight);
						}
				}

				//now add the outer product of the jacobians...
				for (int k=0;k<theMotionPlan->robotRepresentation->getDimensionCount();k++){
					for (int l=k;l<theMotionPlan->robotRepresentation->getDimensionCount();l++){
						double val = 0;
						for (int m=0;m<3;m++)
							val += dEndEffectordq(m, k) * dEndEffectordq(m, l);
						ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + k, theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + l, val, weight);
					}
				}

				//and now the mixed derivatives
				if (theMotionPlan->feetPositionsParamsStartIndex >= 0){
					for (int k=0;k<theMotionPlan->robotRepresentation->getDimensionCount();k++){
						ADD_HES_ELEMENT(hessianEntries, theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 2 + i * 2 + 0, theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + k, -dEndEffectordq(0, k), weight);
						ADD_HES_ELEMENT(hessianEntries, theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 2 + i * 2 + 1, theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + k, -dEndEffectordq(2, k), weight);
					}
				}
			}		
		}
	}

}



