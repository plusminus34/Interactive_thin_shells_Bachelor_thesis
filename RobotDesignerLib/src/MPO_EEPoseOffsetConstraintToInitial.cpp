
#include <RobotDesignerLib/MPO_EEPoseOffsetConstraintToInitial.h>
using namespace std;
#include <iostream>

MPO_EEPoseOffsetConstraintToInitial::MPO_EEPoseOffsetConstraintToInitial(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_EEPoseOffsetConstraintToInitial::~MPO_EEPoseOffsetConstraintToInitial(void) {
}

double MPO_EEPoseOffsetConstraintToInitial::computeValue(const dVector& s) {
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//theMotionPlan->setMPParametersFromList(s);

	double retVal = 0;
	for (int j = 0; j<theMotionPlan->nSamplePoints; j++) {
		for (uint i = 0; i<theMotionPlan->endEffectorTrajectories.size(); i++) {

			//we want the vector from EE to COM similar to the original (targetOffsetFromCOM) pose which looks good
			V3D pseudoLimb(theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(j), theMotionPlan->endEffectorTrajectories[i].EEPos[j]);
			V3D diff = pseudoLimb - theMotionPlan->endEffectorTrajectories[i].targetOffsetFromCOM;
			retVal += diff.length2();
		}
		//cout << "\n";
	}

	//cout << "---------------------------------------------------------\n";
	return retVal * weight;
}


void MPO_EEPoseOffsetConstraintToInitial::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	if (theMotionPlan->contactForcesParamsStartIndex >= 0)
		for (int j = 0; j<theMotionPlan->nSamplePoints; j++) {
			int nLimbs = theMotionPlan->endEffectorTrajectories.size();

			for (int i = 0; i<nLimbs; i++) {
				
				V3D pseudoLimb(theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(j), theMotionPlan->endEffectorTrajectories[i].EEPos[j]);
				V3D diff = pseudoLimb - theMotionPlan->endEffectorTrajectories[i].targetOffsetFromCOM;
				
				if (theMotionPlan->COMPositionsParamsStartIndex >= 0) {
					grad[theMotionPlan->COMPositionsParamsStartIndex + 3 * j + 0] += -2 * diff[0] * weight;
					grad[theMotionPlan->COMPositionsParamsStartIndex + 3 * j + 1] += -2 * diff[1] * weight;
					grad[theMotionPlan->COMPositionsParamsStartIndex + 3 * j + 2] += -2 * diff[2] * weight;
				}

				if (theMotionPlan->feetPositionsParamsStartIndex >= 0) {
					grad[theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + 0] += 2 * diff[0] * weight;
					grad[theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + 1] += 2 * diff[1] * weight;
					grad[theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + 2] += 2 * diff[2] * weight;

				}
			}
		}
}

void MPO_EEPoseOffsetConstraintToInitial::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	for (int j = 0; j<theMotionPlan->nSamplePoints; j++) {
		int nLimbs = theMotionPlan->endEffectorTrajectories.size();
		for (uint i = 0; i<theMotionPlan->endEffectorTrajectories.size(); i++) {

			Matrix3x3 I; I.setIdentity(); I = I * 2;
			
			for (uint s = 0; s < 3; s++) {
				for (uint t = 0; t < 3; t++) {
					if (theMotionPlan->COMPositionsParamsStartIndex >= 0)
						ADD_HES_ELEMENT(hessianEntries,
							theMotionPlan->COMPositionsParamsStartIndex + 3 * j + s,
							theMotionPlan->COMPositionsParamsStartIndex + 3 * j + t,
							I(s, t), weight);

					if (theMotionPlan->feetPositionsParamsStartIndex >= 0) {

						ADD_HES_ELEMENT(hessianEntries,
							theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + s,
							theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + t,
							I(s, t), weight);
					}
				}
			}
			
			for (uint s = 0; s < 3; s++) {
				for (uint t = 0; t < 3; t++) {
					if (theMotionPlan->feetPositionsParamsStartIndex >= 0 && theMotionPlan->COMPositionsParamsStartIndex >= 0) {
						ADD_HES_ELEMENT(hessianEntries,
							theMotionPlan->COMPositionsParamsStartIndex + 3 * j + s,
							theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + t,
							-I(s, t), weight);

						/*ADD_HES_ELEMENT(hessianEntries,
							theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 2 + i * 2 + t,
							theMotionPlan->COMPositionsParamsStartIndex + 3 * j + s,
							-I(tt, s), weight);*/
					}
				}
			}
						
		}
	}
}

