#include <RobotDesignerLib/MPO_RobotCOMObjective.h>

MPO_RobotCOMObjective::MPO_RobotCOMObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_RobotCOMObjective::~MPO_RobotCOMObjective(void){
}

double MPO_RobotCOMObjective::computeValue(const dVector& p){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	double retVal = 0;
	//we want the COM to be as close as possible to the center of the feet that are in contact with the ground, which corresponds to weights as large as possible and equal to each other...
	for (int j=0;j<theMotionPlan->nSamplePoints;j++){
		dVector q_t;
		theMotionPlan->robotStateTrajectory.getQAtTimeIndex(j, q_t);
		theMotionPlan->robotRepresentation->setQ(q_t);

		double totalMass = 0;
		P3D comPos;

		for (uint k=0; k<theMotionPlan->robot->bFrame->bodyLinks.size(); k++){
			totalMass += theMotionPlan->robot->bFrame->bodyLinks[k]->rbProperties.mass;
			comPos += theMotionPlan->robotRepresentation->getWorldCoordinatesFor(P3D(), theMotionPlan->robot->bFrame->bodyLinks[k]) * theMotionPlan->robot->bFrame->bodyLinks[k]->rbProperties.mass;
		}

		comPos /= totalMass;

		V3D err = V3D(comPos, theMotionPlan->bodyTrajectory.getCOMPositionAtTimeIndex(j));
		retVal += 0.5 * err.length2();
	}
	return retVal * weight;
}

void MPO_RobotCOMObjective::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	MatrixNxM dpdq;

	for (int j=0;j<theMotionPlan->nSamplePoints;j++){
		P3D desComPos = theMotionPlan->bodyTrajectory.getCOMPositionAtTimeIndex(j);

		dVector q_t;
		theMotionPlan->robotStateTrajectory.getQAtTimeIndex(j, q_t);
		theMotionPlan->robotRepresentation->setQ(q_t);

//		P3D comPos = theMotionPlan->bodyTrajectory.getCOMPositionAtTimeIndex(j);

		double totalMass = 0;
		P3D comPos;

		for (uint k=0; k<theMotionPlan->robot->bFrame->bodyLinks.size(); k++){
			totalMass += theMotionPlan->robot->bFrame->bodyLinks[k]->rbProperties.mass;
			comPos += theMotionPlan->robotRepresentation->getWorldCoordinatesFor(P3D(), theMotionPlan->robot->bFrame->bodyLinks[k]) * theMotionPlan->robot->bFrame->bodyLinks[k]->rbProperties.mass;
		}

		comPos /= totalMass;

		V3D err = desComPos - comPos;

		//compute the gradient with respect to the COM positions
		if (theMotionPlan->COMPositionsParamsStartIndex >= 0){
			grad[theMotionPlan->COMPositionsParamsStartIndex + 3 * j + 0] += err[0] * weight;
			grad[theMotionPlan->COMPositionsParamsStartIndex + 3 * j + 1] += err[1] * weight;
			grad[theMotionPlan->COMPositionsParamsStartIndex + 3 * j + 2] += err[2] * weight;
		}

		for (uint k=0; k<theMotionPlan->robot->bFrame->bodyLinks.size(); k++){
			//compute the gradient with respect to the COM positions
			if (theMotionPlan->robotStatesParamsStartIndex >= 0){
				theMotionPlan->robotRepresentation->compute_dpdq(P3D(), theMotionPlan->robot->bFrame->bodyLinks[k], dpdq);
	
				for (int m=0;m<3;m++)
					for (int l=0;l<theMotionPlan->robotRepresentation->getDimensionCount();l++)
						grad[theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + l] += dpdq(m, l) * -err[m] * (theMotionPlan->robot->bFrame->bodyLinks[k]->rbProperties.mass / totalMass)*weight;
			}
		}
	}
}

void MPO_RobotCOMObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	MatrixNxM ddpdq_dqi;

	DynamicArray<MatrixNxM> dpdq;
	dpdq.resize(theMotionPlan->robot->bFrame->bodyLinks.size());

	for (int j=0;j<theMotionPlan->nSamplePoints;j++){
		P3D desComPos = theMotionPlan->bodyTrajectory.getCOMPositionAtTimeIndex(j);

		dVector q_t;
		theMotionPlan->robotStateTrajectory.getQAtTimeIndex(j, q_t);
		theMotionPlan->robotRepresentation->setQ(q_t);

//		P3D comPos = theMotionPlan->bodyTrajectory.getCOMPositionAtTimeIndex(j);

		double totalMass = 0;
		P3D comPos;

		for (uint k=0; k<theMotionPlan->robot->bFrame->bodyLinks.size(); k++){
			theMotionPlan->robotRepresentation->compute_dpdq(P3D(), theMotionPlan->robot->bFrame->bodyLinks[k], dpdq[k]);
			totalMass += theMotionPlan->robot->bFrame->bodyLinks[k]->rbProperties.mass;
			comPos += theMotionPlan->robotRepresentation->getWorldCoordinatesFor(P3D(), theMotionPlan->robot->bFrame->bodyLinks[k]) * theMotionPlan->robot->bFrame->bodyLinks[k]->rbProperties.mass;
		}

		comPos /= totalMass;

		V3D err = desComPos - comPos;

			//compute the gradient with respect to the feet locations
		if (theMotionPlan->COMPositionsParamsStartIndex >= 0){
			ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * j + 0, theMotionPlan->COMPositionsParamsStartIndex + 3 * j + 0, 1, weight);
			ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * j + 1, theMotionPlan->COMPositionsParamsStartIndex + 3 * j + 1, 1, weight);
			ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * j + 2, theMotionPlan->COMPositionsParamsStartIndex + 3 * j + 2, 1, weight);
		}

		for (uint k=0; k<theMotionPlan->robot->bFrame->bodyLinks.size(); k++){

			double massContribution = (theMotionPlan->robot->bFrame->bodyLinks[k]->rbProperties.mass / totalMass);

			//compute the gradient with respect to the COM positions
			if (theMotionPlan->robotStatesParamsStartIndex >= 0){
				//get the mixed derivatives... 
				if (theMotionPlan->COMPositionsParamsStartIndex >= 0){
					for (int l=0;l<theMotionPlan->robotRepresentation->getDimensionCount();l++){
						if (!IS_ZERO(dpdq[k](0, l))) ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * j + 0, theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + l, -dpdq[k](0, l) * massContribution, weight);
						if (!IS_ZERO(dpdq[k](1, l))) ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * j + 1, theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + l, -dpdq[k](1, l) * massContribution, weight);
						if (!IS_ZERO(dpdq[k](2, l))) ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * j + 2, theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + l, -dpdq[k](2, l) * massContribution, weight);
					}
				}

				//the main part of the derivative consists of an outer product of jacobians followed by higher order derivatives.... do the outer product part now...
				for (uint k2=k;k2<theMotionPlan->robot->bFrame->bodyLinks.size();k2++){
					double massContribution2 = (theMotionPlan->robot->bFrame->bodyLinks[k2]->rbProperties.mass / totalMass);
					for (int n=0;n<theMotionPlan->robotRepresentation->getDimensionCount();n++){
						for (int l=n;l<theMotionPlan->robotRepresentation->getDimensionCount();l++){
							double val = 0;
							for (int m=0;m<3;m++){
								val += dpdq[k](m, n) * dpdq[k2](m, l) * massContribution * massContribution2;
								if (k!=k2)
									val += dpdq[k2](m, n) * dpdq[k](m, l) * massContribution * massContribution2;
							}
							if (!IS_ZERO(val)) ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + n, theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + l, val, weight);
						}
					}
				}

				//this is the second part - higher order derivatives...
				for (int n=0;n<theMotionPlan->robotRepresentation->getDimensionCount();n++){
					bool hasNonZeros = theMotionPlan->robotRepresentation->compute_ddpdq_dqi(P3D(), theMotionPlan->robot->bFrame->bodyLinks[k], ddpdq_dqi, n);
					if (hasNonZeros == false) continue;
					for (int l=n;l<theMotionPlan->robotRepresentation->getDimensionCount();l++)
						for (int m=0;m<3;m++){
							double val = ddpdq_dqi(m, l) * -err[m] * massContribution;
							if (!IS_ZERO(val)) ADD_HES_ELEMENT(hessianEntries, theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + n, theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + l, val, weight);
						}
				}
			}
		}
	}
}


