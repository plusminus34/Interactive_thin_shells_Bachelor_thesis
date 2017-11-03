#include <RobotDesignerLib/MPO_PseudoLimbLengthConstraint.h>
using namespace std;
#include <iostream>

MPO_PseudoLimbLengthConstraint::MPO_PseudoLimbLengthConstraint(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_PseudoLimbLengthConstraint::~MPO_PseudoLimbLengthConstraint(void) {
}

double MPO_PseudoLimbLengthConstraint::computeValue(const dVector& s) {
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//theMotionPlan->setMPParametersFromList(s);

	double retVal = 0;
	for (int j = 0; j<theMotionPlan->nSamplePoints; j++) {
		for (uint i = 0; i<theMotionPlan->endEffectorTrajectories.size(); i++) {
			
			//we want the dist from ee to com to be > limb length
			double limb_length= (theMotionPlan->endEffectorTrajectories[i].theLimb->getLength()*tolerance + theMotionPlan->pseudoLimbEpsilon);
			V3D pseudoLimb(theMotionPlan->endEffectorTrajectories[i].EEPos[j], theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(j));
			double pseudoLimb_length = pseudoLimb.length();
			//cout << pseudoLimb_length << ",";

			SoftUnilateralConstraint pseudoLengthUpbdd = SoftUnilateralConstraint(-limb_length, 10, theMotionPlan->pseudoLimbEpsilon);
			retVal += pseudoLengthUpbdd.computeValue(-pseudoLimb_length);

			/*double fVertical = theMotionPlan->endEffectorTrajectories[i].contactForce[j](1);
			double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];
			SoftUnilateralConstraint sucVerticalLowerBound = SoftUnilateralConstraint(theMotionPlan->verticalGRFLowerBoundVal, 10, theMotionPlan->GRFEpsilon);
			SoftUnilateralConstraint sucVerticalUpperBound = SoftUnilateralConstraint(-theMotionPlan->endEffectorTrajectories[i].verticalGRFUpperBoundValues[j], 10, theMotionPlan->GRFEpsilon);
			retVal += sucVerticalLowerBound.computeValue(fVertical) * c;
			retVal += sucVerticalUpperBound.computeValue(-fVertical) * c;*/
			
			//Logger::logPrint("%lf\t%lf\n", val, suc.computeValue(-val));

		}
		//cout << "\n";
	}

	//cout << "---------------------------------------------------------\n";
	return retVal * weight;
}


void MPO_PseudoLimbLengthConstraint::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	if (theMotionPlan->contactForcesParamsStartIndex >= 0)
		for (int j = 0; j<theMotionPlan->nSamplePoints; j++) {
			int nLimbs = theMotionPlan->endEffectorTrajectories.size();

			for (int i = 0; i<nLimbs; i++) {
				//we want the vertical component of the GRF to be > 0
				
				double limb_length = (theMotionPlan->endEffectorTrajectories[i].theLimb->getLength()*tolerance + theMotionPlan->pseudoLimbEpsilon);
				P3D ee = theMotionPlan->endEffectorTrajectories[i].EEPos[j];
				P3D xcom = theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(j);				
				V3D pseudoLimb(ee,xcom);
				double pseudoLimb_length = pseudoLimb.length(); // =D  ie. SoftUnilateralConstraint(D)
				
				SoftUnilateralConstraint pseudoLengthUpbdd = SoftUnilateralConstraint(-limb_length, 10, theMotionPlan->pseudoLimbEpsilon);
				
				// d SoftUnilateralConstraint dD
				double dConstraintdD =  pseudoLengthUpbdd.computeDerivative(-pseudoLimb_length) * weight;				
				pseudoLimb.toUnit();

				if (theMotionPlan->COMPositionsParamsStartIndex >= 0) {
					grad[theMotionPlan->COMPositionsParamsStartIndex + 3 * j + 0] += dConstraintdD*pseudoLimb[0];
					grad[theMotionPlan->COMPositionsParamsStartIndex + 3 * j + 1] += dConstraintdD*pseudoLimb[1];
					grad[theMotionPlan->COMPositionsParamsStartIndex + 3 * j + 2] += dConstraintdD*pseudoLimb[2];
				}

				if (theMotionPlan->feetPositionsParamsStartIndex >= 0) {
					grad[theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + 0] += -dConstraintdD*pseudoLimb[0];
					grad[theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + 1] += -dConstraintdD*pseudoLimb[1];
					grad[theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + 2] += -dConstraintdD*pseudoLimb[2];

				}
				

				/*double fVertical = theMotionPlan->endEffectorTrajectories[i].contactForce[j](1);
				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];

				SoftUnilateralConstraint sucVerticalUpperBound = SoftUnilateralConstraint(-theMotionPlan->endEffectorTrajectories[i].verticalGRFUpperBoundValues[j], 10, theMotionPlan->GRFEpsilon);
				SoftUnilateralConstraint sucVerticalLowerBound = SoftUnilateralConstraint(theMotionPlan->verticalGRFLowerBoundVal, 10, theMotionPlan->GRFEpsilon);

				grad[theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 1] += -1 * sucVerticalUpperBound.computeDerivative(-fVertical) * c * weight;
				grad[theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 1] += sucVerticalLowerBound.computeDerivative(fVertical) * c * weight;*/

				
			}
		}
}

void MPO_PseudoLimbLengthConstraint::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);
	
		for (int j = 0; j<theMotionPlan->nSamplePoints; j++) {
			int nLimbs = theMotionPlan->endEffectorTrajectories.size();
			for (uint i = 0; i<theMotionPlan->endEffectorTrajectories.size(); i++) {
				
				double limb_length = (theMotionPlan->endEffectorTrajectories[i].theLimb->getLength()*tolerance + theMotionPlan->pseudoLimbEpsilon);
				P3D ee = theMotionPlan->endEffectorTrajectories[i].EEPos[j];
				P3D xcom = theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(j);
				V3D pseudoLimb(ee, xcom);
				double pseudoLimb_length = pseudoLimb.length(); // =D  ie. SoftUnilateralConstraint(D)

				SoftUnilateralConstraint pseudoLengthUpbdd = SoftUnilateralConstraint(-limb_length, 10, theMotionPlan->pseudoLimbEpsilon);

				// d SoftUnilateralConstraint dD
				double dConstraintdD =  pseudoLengthUpbdd.computeDerivative(-pseudoLimb_length);
				// dd SoftUnilateralConstraint dDdD
				double ddConstraintdDdD = pseudoLengthUpbdd.computeSecondDerivative(-pseudoLimb_length); 
				
				Matrix3x3 I; I.setIdentity();
				pseudoLimb.toUnit();
				Matrix3x3 ddT = pseudoLimb.outerProductWith(pseudoLimb);
				
				// dd Obj ddxcom
				Matrix3x3 dsame			= ddConstraintdDdD*ddT + dConstraintdD / pseudoLimb_length*(I - ddT);
				Matrix3x3 ddifferent	= -dsame;
				
				for (uint s = 0; s < 3; s++) {
					for (uint t = 0; t < 3; t++) {
						if (theMotionPlan->COMPositionsParamsStartIndex >= 0)
							ADD_HES_ELEMENT(hessianEntries,
								theMotionPlan->COMPositionsParamsStartIndex + 3 * j + s,
								theMotionPlan->COMPositionsParamsStartIndex + 3 * j + t,
								dsame(s, t), weight);

						if (theMotionPlan->feetPositionsParamsStartIndex >= 0){
							ADD_HES_ELEMENT(hessianEntries,
								theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + s,
								theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + t,
								dsame(s, t), weight);
						}
					}
				}


				for (uint s = 0; s < 3; s++) {
					for (uint t = 0; t < 3; t++) {
						if (theMotionPlan->feetPositionsParamsStartIndex >= 0 && theMotionPlan->COMPositionsParamsStartIndex >= 0){
							ADD_HES_ELEMENT(hessianEntries,
								theMotionPlan->COMPositionsParamsStartIndex + 3 * j + s,
								theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + t,
								ddifferent(s, t), weight);

							ADD_HES_ELEMENT(hessianEntries,
								theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + t,
								theMotionPlan->COMPositionsParamsStartIndex + 3 * j + s,
								ddifferent(t, s), weight);
						}
					}
				}

				/*double fVertical = theMotionPlan->endEffectorTrajectories[i].contactForce[j](1);
				double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];
				SoftUnilateralConstraint sucVerticalUpperBound = SoftUnilateralConstraint(-theMotionPlan->endEffectorTrajectories[i].verticalGRFUpperBoundValues[j], 10, theMotionPlan->GRFEpsilon);
				SoftUnilateralConstraint sucVerticalLowerBound = SoftUnilateralConstraint(theMotionPlan->verticalGRFLowerBoundVal, 10, theMotionPlan->GRFEpsilon);

				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 1, theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 1, sucVerticalUpperBound.computeSecondDerivative(-fVertical) * c, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 1, theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + 1, sucVerticalLowerBound.computeSecondDerivative(fVertical) * c, weight);*/


			}
		}
}


