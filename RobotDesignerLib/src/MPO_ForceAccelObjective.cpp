#include <RobotDesignerLib/MPO_ForceAccelObjective.h>



MPO_ForceAccelObjective::MPO_ForceAccelObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

double MPO_ForceAccelObjective::computeValue(const dVector& p) {
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	double err = 0;
	double h = theMotionPlan->motionPlanDuration / (theMotionPlan->nSamplePoints - 1);

	for (int j = 0; j < theMotionPlan->nSamplePoints; j++) {
		int jmm, jm, jp, jpp;

		theMotionPlan->getAccelerationTimeIndicesFor(j, jmm, jm, jp, jpp);
		if (jp == -1 || jpp == -1) continue;
		if ((jmm == -1 || jm == -1) && theMotionPlan->bodyTrajectory.useInitialVelocities == false) continue;

		V3D vP, vM;
		vP = (theMotionPlan->bodyTrajectory.getCOMPositionAtTimeIndex(jpp) - theMotionPlan->bodyTrajectory.getCOMPositionAtTimeIndex(jp)) / h;
		if (jmm == -1 || jm == -1)
			vM = theMotionPlan->bodyTrajectory.initialLinearVelocity;
		else
			vM = (theMotionPlan->bodyTrajectory.getCOMPositionAtTimeIndex(jm) - theMotionPlan->bodyTrajectory.getCOMPositionAtTimeIndex(jmm)) / h;

		//compute COM acceleration
		V3D acceleration = (vP - vM) / h;

		V3D ma = acceleration * theMotionPlan->totalMass;
		V3D force = Globals::worldUp * Globals::g * theMotionPlan->totalMass;
		force += theMotionPlan->externalForce;
		for (uint i = 0; i < theMotionPlan->endEffectorTrajectories.size(); i++) {
			force += theMotionPlan->endEffectorTrajectories[i].contactForce[j] * theMotionPlan->endEffectorTrajectories[i].contactFlag[j];
		}

		err += 0.5 * (force - ma).length2() * weight;
	}

	return err;
}

void MPO_ForceAccelObjective::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	double h = theMotionPlan->motionPlanDuration / (theMotionPlan->nSamplePoints - 1);

	for (int j = 0; j < theMotionPlan->nSamplePoints; j++) {
		int jmm, jm, jp, jpp;

		theMotionPlan->getAccelerationTimeIndicesFor(j, jmm, jm, jp, jpp);
		if (jp == -1 || jpp == -1) continue;
		if ((jmm == -1 || jm == -1) && theMotionPlan->bodyTrajectory.useInitialVelocities == false) continue;

		V3D vP, vM;
		vP = (theMotionPlan->bodyTrajectory.getCOMPositionAtTimeIndex(jpp) - theMotionPlan->bodyTrajectory.getCOMPositionAtTimeIndex(jp)) / h;
		if (jmm == -1 || jm == -1)
			vM = theMotionPlan->bodyTrajectory.initialLinearVelocity;
		else
			vM = (theMotionPlan->bodyTrajectory.getCOMPositionAtTimeIndex(jm) - theMotionPlan->bodyTrajectory.getCOMPositionAtTimeIndex(jmm)) / h;

		//compute COM acceleration
		V3D acceleration = (vP - vM) / h;

		V3D ma = acceleration * theMotionPlan->totalMass;
		V3D force = Globals::worldUp * Globals::g * theMotionPlan->totalMass;
		force += theMotionPlan->externalForce;

		for (uint i = 0; i < theMotionPlan->endEffectorTrajectories.size(); i++) {
			force += theMotionPlan->endEffectorTrajectories[i].contactForce[j] * theMotionPlan->endEffectorTrajectories[i].contactFlag[j];
		}

		if (theMotionPlan->COMPositionsParamsStartIndex > -1) {
			V3D pGrad = (ma - force) * theMotionPlan->totalMass / (h * h) * weight;

			if (jpp > -1) grad.segment<3>(theMotionPlan->COMPositionsParamsStartIndex + 3 * jpp) += pGrad;
			if (jmm > -1) grad.segment<3>(theMotionPlan->COMPositionsParamsStartIndex + 3 * jmm) += pGrad;
			if (jp > -1) grad.segment<3>(theMotionPlan->COMPositionsParamsStartIndex + 3 * jp) -= pGrad;
			if (jm > -1) grad.segment<3>(theMotionPlan->COMPositionsParamsStartIndex + 3 * jm) -= pGrad;
		}

		if (theMotionPlan->contactForcesParamsStartIndex > -1) {
			for (uint i = 0; i < theMotionPlan->endEffectorTrajectories.size(); i++) {
				if (theMotionPlan->endEffectorTrajectories[i].contactFlag[j] > 0)
					grad.segment<3>(theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i)) += weight * (force - ma);
			}
		}
	}

}

void MPO_ForceAccelObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	double h = theMotionPlan->motionPlanDuration / (theMotionPlan->nSamplePoints - 1);
	double scale = theMotionPlan->totalMass / (h * h);

	for (int j = 0; j < theMotionPlan->nSamplePoints; j++) {
		int jmm, jm, jp, jpp;

		theMotionPlan->getAccelerationTimeIndicesFor(j, jmm, jm, jp, jpp);
		if (jp == -1 || jpp == -1) continue;
		if ((jmm == -1 || jm == -1) && theMotionPlan->bodyTrajectory.useInitialVelocities == false) continue;

		int startIndexjpp = theMotionPlan->COMPositionsParamsStartIndex + 3 * jpp;
		int startIndexjmm = theMotionPlan->COMPositionsParamsStartIndex + 3 * jmm;
		int startIndexjm = theMotionPlan->COMPositionsParamsStartIndex + 3 * jm;
		int startIndexjp = theMotionPlan->COMPositionsParamsStartIndex + 3 * jp;

		if (theMotionPlan->COMPositionsParamsStartIndex > -1)
		{
			double HVal = scale * scale;
			if (jpp > -1) addDiagonalHessianBlock(hessianEntries, startIndexjpp, startIndexjpp, HVal);
			if (jmm > -1) addDiagonalHessianBlock(hessianEntries, startIndexjmm, startIndexjmm, HVal);
			if (jp > -1) addDiagonalHessianBlock(hessianEntries, startIndexjp, startIndexjp, HVal);
			if (jm > -1) addDiagonalHessianBlock(hessianEntries, startIndexjm, startIndexjm, HVal);

			if (jpp > -1 && jp > -1) addDiagonalHessianBlock(hessianEntries, startIndexjpp, startIndexjp, -HVal);
			if (jpp > -1 && jmm > -1) addDiagonalHessianBlock(hessianEntries, startIndexjpp, startIndexjmm, HVal);
			if (jpp > -1 && jm > -1) addDiagonalHessianBlock(hessianEntries, startIndexjpp, startIndexjm, -HVal);
			if (jmm > -1 && jp > -1) addDiagonalHessianBlock(hessianEntries, startIndexjmm, startIndexjp, -HVal);
			if (jmm > -1 && jm > -1) addDiagonalHessianBlock(hessianEntries, startIndexjmm, startIndexjm, -HVal);
			if (jp > -1 && jm > -1) addDiagonalHessianBlock(hessianEntries, startIndexjp, startIndexjm, HVal);

			// no reflect for diagonal block in Hessian, so we need to count twice.
			if (jp == jm)
				if (jp > -1 && jm > -1) addDiagonalHessianBlock(hessianEntries, startIndexjp, startIndexjm, HVal);

			if (theMotionPlan->contactForcesParamsStartIndex > -1)
			{
				for (uint i = 0; i < theMotionPlan->endEffectorTrajectories.size(); i++) {

					if (theMotionPlan->endEffectorTrajectories[i].contactFlag[j] > 0) {
						int startIndexF = theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i);
						if (jpp > -1) addDiagonalHessianBlock(hessianEntries, startIndexF, startIndexjpp, -scale);
						if (jmm > -1) addDiagonalHessianBlock(hessianEntries, startIndexF, startIndexjmm, -scale);
						if (jp > -1) addDiagonalHessianBlock(hessianEntries, startIndexF, startIndexjp, scale);
						if (jm > -1) addDiagonalHessianBlock(hessianEntries, startIndexF, startIndexjm, scale);
					}
				}
			}
		}

		if (theMotionPlan->contactForcesParamsStartIndex > -1)
		{
			for (uint i = 0; i < theMotionPlan->endEffectorTrajectories.size(); i++) {
				if (theMotionPlan->endEffectorTrajectories[i].contactFlag[j] > 0) {
					int startIndexFI = theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i);

					for (uint k = i; k < theMotionPlan->endEffectorTrajectories.size(); k++) {
						if (theMotionPlan->endEffectorTrajectories[k].contactFlag[j] > 0) {
							int startIndexFK = theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + k);
							addDiagonalHessianBlock(hessianEntries, startIndexFI, startIndexFK, 1);
						}
					}
				}

			}
		}
	}
}

void MPO_ForceAccelObjective::addDiagonalHessianBlock(DynamicArray<MTriplet>& hessianEntries, int startIndexI, int startIndexJ, double val)
{
	ADD_HES_ELEMENT(hessianEntries, startIndexI, startIndexJ, val, weight);
	ADD_HES_ELEMENT(hessianEntries, startIndexI + 1, startIndexJ + 1, val, weight);
	ADD_HES_ELEMENT(hessianEntries, startIndexI + 2, startIndexJ + 2, val, weight);
}


MPO_ForceAccelObjective::~MPO_ForceAccelObjective()
{
}
