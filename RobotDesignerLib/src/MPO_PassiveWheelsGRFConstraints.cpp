#include <RobotDesignerLib/MPO_PassiveWheelsGRFConstraints.h>
#include <OptimizationLib/SoftUnilateralConstraint.h>
#include <MathLib/AutoDiff.h>

MPO_PassiveWheelsGRFConstraints::MPO_PassiveWheelsGRFConstraints(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_PassiveWheelsGRFConstraints::~MPO_PassiveWheelsGRFConstraints(void){
}

double MPO_PassiveWheelsGRFConstraints::computeValue(const dVector& p){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	double retVal = 0;
	int nEEs = theMotionPlan->endEffectorTrajectories.size();
	for (int j=0;j<theMotionPlan->nSamplePoints;j++){
		dVector q_t;
		theMotionPlan->robotStateTrajectory.getQAtTimeIndex(j, q_t);

		for (int i=0; i<nEEs; i++){
			const LocomotionEngine_EndEffectorTrajectory &ee = theMotionPlan->endEffectorTrajectories[i];

			const V3D &wheelAxisLocal = ee.wheelAxisLocal_WF;
			double yawAngle = ee.wheelYawAngle[j];
			const V3D &yawAxis = ee.wheelYawAxis_WF;
			double tiltAngle = ee.wheelTiltAngle[j];
			const V3D &tiltAxis = ee.wheelTiltAxis_WF;
			const V3D &grf = ee.contactForce[j];

			if(ee.isPassiveWheel)
			{
				retVal += computeEnergy(wheelAxisLocal, yawAxis, yawAngle, tiltAxis, tiltAngle, grf);
			}
		}
	}

	return retVal;
}

void MPO_PassiveWheelsGRFConstraints::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	// number of DOFs for an end effector at one time sample
	int numDOFs = 0;
	// 2 DOFs for yaw and tilt angle
	if (theMotionPlan->wheelParamsStartIndex >= 0)
		numDOFs += 2;
	// 3 DOFs for contact force
	if (theMotionPlan->contactForcesParamsStartIndex >= 0)
		numDOFs += 3;

	int nEEs = theMotionPlan->endEffectorTrajectories.size();
	for (int j=0; j<theMotionPlan->nSamplePoints; j++){

		for (int i=0 ;i<nEEs; i++){
			const LocomotionEngine_EndEffectorTrajectory &ee = theMotionPlan->endEffectorTrajectories[i];

			if(ee.isPassiveWheel)
			{

				V3T<ScalarDiff> wheelAxisLocal = ee.wheelAxisLocal_WF;
				ScalarDiff yawAngle = ee.wheelYawAngle[j];
				V3T<ScalarDiff> yawAxis = ee.wheelYawAxis_WF;
				ScalarDiff tiltAngle = ee.wheelTiltAngle[j];
				V3T<ScalarDiff> tiltAxis = ee.wheelTiltAxis_WF;
				V3T<ScalarDiff> grf = ee.contactForce[j];

				std::vector<DOF<ScalarDiff>> dofs(numDOFs);
				int index = 0;
				if (theMotionPlan->wheelParamsStartIndex >= 0){
					dofs[index].v = &yawAngle;
					dofs[index].i = theMotionPlan->getWheelYawAngleIndex(i, j);
					index++;
					dofs[index].v = &tiltAngle;
					dofs[index].i = theMotionPlan->getWheelTiltAngleIndex(i, j);
					index++;
				}
				if (theMotionPlan->contactForcesParamsStartIndex >= 0){
					for (int k = 0; k < 3; ++k){
						dofs[index].v = &grf[k];
						dofs[index].i = theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + k;
						index++;
					}
				}

				for (int k = 0; k < numDOFs; ++k) {
					dofs[k].v->deriv() = 1.0;
					ScalarDiff energy = computeEnergy(wheelAxisLocal, yawAxis, yawAngle, tiltAxis, tiltAngle, grf);
					grad[dofs[k].i] += energy.deriv();
					dofs[k].v->deriv() = 0.0;
				}
			}
		}
	}
}

void MPO_PassiveWheelsGRFConstraints::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	// number of DOFs for an end effector at one time sample
	int numDOFs = 0;
	// 2 DOFs for yaw and tilt angle
	if (theMotionPlan->wheelParamsStartIndex >= 0)
		numDOFs += 2;
	// 3 DOFs for contact force
	if (theMotionPlan->contactForcesParamsStartIndex >= 0)
		numDOFs += 3;

	int nEEs = theMotionPlan->endEffectorTrajectories.size();
	for (int j=0; j<theMotionPlan->nSamplePoints; j++){

		for (int i=0; i<nEEs; i++){
			const LocomotionEngine_EndEffectorTrajectory &ee = theMotionPlan->endEffectorTrajectories[i];

			if(ee.isPassiveWheel)
			{

				V3T<ScalarDiffDiff> wheelAxisLocal = ee.wheelAxisLocal_WF;
				ScalarDiffDiff yawAngle = ee.wheelYawAngle[j];
				V3T<ScalarDiffDiff> yawAxis = ee.wheelYawAxis_WF;
				ScalarDiffDiff tiltAngle = ee.wheelTiltAngle[j];
				V3T<ScalarDiffDiff> tiltAxis = ee.wheelTiltAxis_WF;
				V3T<ScalarDiffDiff> grf = ee.contactForce[j];

				std::vector<DOF<ScalarDiffDiff>> dofs(numDOFs);
				int index = 0;
				if (theMotionPlan->wheelParamsStartIndex >= 0){
					dofs[index].v = &yawAngle;
					dofs[index].i = theMotionPlan->getWheelYawAngleIndex(i, j);
					index++;
					dofs[index].v = &tiltAngle;
					dofs[index].i = theMotionPlan->getWheelTiltAngleIndex(i, j);
					index++;
				}
				if (theMotionPlan->contactForcesParamsStartIndex >= 0){
					for (int k = 0; k < 3; ++k){
						dofs[index].v = &grf[k];
						dofs[index].i = theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + k;
						index++;
					}
				}

				for (int k = 0; k < numDOFs; ++k) {
					dofs[k].v->deriv().value() = 1.0;
					for (int l = 0; l <= k; ++l) {
						dofs[l].v->value().deriv() = 1.0;
						ScalarDiffDiff energy = computeEnergy(wheelAxisLocal, yawAxis, yawAngle, tiltAxis, tiltAngle, grf);
						ADD_HES_ELEMENT(hessianEntries,
										dofs[k].i,
										dofs[l].i,
										energy.deriv().deriv(), 1.0);
						dofs[l].v->value().deriv() = 0.0;
					}
					dofs[k].v->deriv().value() = 0.0;
				}
			}
		}
	}
}

MPO_PassiveWheelsGRFFrictionConstraints::MPO_PassiveWheelsGRFFrictionConstraints(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_PassiveWheelsGRFFrictionConstraints::~MPO_PassiveWheelsGRFFrictionConstraints(void){
}

double MPO_PassiveWheelsGRFFrictionConstraints::computeValue(const dVector& p){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	double frictionCoeff = theMotionPlan->frictionCoeff;

	double retVal = 0;
	int nEEs = theMotionPlan->endEffectorTrajectories.size();
	for (int j=0;j<theMotionPlan->nSamplePoints;j++){
		dVector q_t;
		theMotionPlan->robotStateTrajectory.getQAtTimeIndex(j, q_t);

		for (int i=0; i<nEEs; i++){
			const LocomotionEngine_EndEffectorTrajectory &ee = theMotionPlan->endEffectorTrajectories[i];
			if(ee.isPassiveWheel)
			{
				const V3D &wheelAxisLocal = ee.wheelAxisLocal_WF;
				double yawAngle = ee.wheelYawAngle[j];
				const V3D &yawAxis = ee.wheelYawAxis_WF;
				double tiltAngle = ee.wheelTiltAngle[j];
				const V3D &tiltAxis = ee.wheelTiltAxis_WF;
				const V3D &grf = ee.contactForce[j];

				double fr = fabs(computeForceForward(wheelAxisLocal, yawAxis, yawAngle, tiltAxis, tiltAngle, grf));
				SoftUnilateralUpperConstraintT<double> upperBound = SoftUnilateralUpperConstraintT<double>(0, 10, theMotionPlan->frictionEpsilon);
				retVal += upperBound.computeValue(fr-frictionCoeff*grf(1)) * weight;
			}
		}
	}

	return retVal;
}

void MPO_PassiveWheelsGRFFrictionConstraints::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	// number of DOFs for an end effector at one time sample
	int numDOFs = 0;
	// 2 DOFs for yaw and tilt angle
	if (theMotionPlan->wheelParamsStartIndex >= 0)
		numDOFs += 2;
	// 3 DOFs for contact force
	if (theMotionPlan->contactForcesParamsStartIndex >= 0)
		numDOFs += 3;

	ScalarDiff frictionCoeff = theMotionPlan->frictionCoeff;

	int nEEs = theMotionPlan->endEffectorTrajectories.size();
	for (int j=0; j<theMotionPlan->nSamplePoints; j++){

		for (int i=0 ;i<nEEs; i++){
			const LocomotionEngine_EndEffectorTrajectory &ee = theMotionPlan->endEffectorTrajectories[i];

			if(ee.isPassiveWheel)
			{

				V3T<ScalarDiff> wheelAxisLocal = ee.wheelAxisLocal_WF;
				ScalarDiff yawAngle = ee.wheelYawAngle[j];
				V3T<ScalarDiff> yawAxis = ee.wheelYawAxis_WF;
				ScalarDiff tiltAngle = ee.wheelTiltAngle[j];
				V3T<ScalarDiff> tiltAxis = ee.wheelTiltAxis_WF;
				V3T<ScalarDiff> grf = ee.contactForce[j];

				std::vector<DOF<ScalarDiff>> dofs(numDOFs);
				int index = 0;
				if (theMotionPlan->wheelParamsStartIndex >= 0){
					dofs[index].v = &yawAngle;
					dofs[index].i = theMotionPlan->getWheelYawAngleIndex(i, j);
					index++;
					dofs[index].v = &tiltAngle;
					dofs[index].i = theMotionPlan->getWheelTiltAngleIndex(i, j);
					index++;
				}
				if (theMotionPlan->contactForcesParamsStartIndex >= 0){
					for (int k = 0; k < 3; ++k){
						dofs[index].v = &grf[k];
						dofs[index].i = theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + k;
						index++;
					}
				}

				for (int k = 0; k < numDOFs; ++k) {
					dofs[k].v->deriv() = 1.0;

					ScalarDiff fr = fabs(computeForceForward(wheelAxisLocal, yawAxis, yawAngle, tiltAxis, tiltAngle, grf));
					SoftUnilateralUpperConstraintT<ScalarDiff> upperBound = SoftUnilateralUpperConstraintT<ScalarDiff>(0, 10, theMotionPlan->frictionEpsilon);
					ScalarDiff energy = upperBound.computeValue(fr-frictionCoeff*grf(1)) * weight;

					grad[dofs[k].i] += energy.deriv();
					dofs[k].v->deriv() = 0.0;
				}
			}
		}
	}
}

void MPO_PassiveWheelsGRFFrictionConstraints::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	// number of DOFs for an end effector at one time sample
	int numDOFs = 0;
	// 2 DOFs for yaw and tilt angle
	if (theMotionPlan->wheelParamsStartIndex >= 0)
		numDOFs += 2;
	// 3 DOFs for contact force
	if (theMotionPlan->contactForcesParamsStartIndex >= 0)
		numDOFs += 3;

	ScalarDiffDiff frictionCoeff = theMotionPlan->frictionCoeff;

	int nEEs = theMotionPlan->endEffectorTrajectories.size();
	for (int j=0; j<theMotionPlan->nSamplePoints; j++){

		for (int i=0; i<nEEs; i++){
			const LocomotionEngine_EndEffectorTrajectory &ee = theMotionPlan->endEffectorTrajectories[i];

			if(ee.isPassiveWheel)
			{

				V3T<ScalarDiffDiff> wheelAxisLocal = ee.wheelAxisLocal_WF;
				ScalarDiffDiff yawAngle = ee.wheelYawAngle[j];
				V3T<ScalarDiffDiff> yawAxis = ee.wheelYawAxis_WF;
				ScalarDiffDiff tiltAngle = ee.wheelTiltAngle[j];
				V3T<ScalarDiffDiff> tiltAxis = ee.wheelTiltAxis_WF;
				V3T<ScalarDiffDiff> grf = ee.contactForce[j];

				std::vector<DOF<ScalarDiffDiff>> dofs(numDOFs);
				int index = 0;
				if (theMotionPlan->wheelParamsStartIndex >= 0){
					dofs[index].v = &yawAngle;
					dofs[index].i = theMotionPlan->getWheelYawAngleIndex(i, j);
					index++;
					dofs[index].v = &tiltAngle;
					dofs[index].i = theMotionPlan->getWheelTiltAngleIndex(i, j);
					index++;
				}
				if (theMotionPlan->contactForcesParamsStartIndex >= 0){
					for (int k = 0; k < 3; ++k){
						dofs[index].v = &grf[k];
						dofs[index].i = theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i) + k;
						index++;
					}
				}

				for (int k = 0; k < numDOFs; ++k) {
					dofs[k].v->deriv().value() = 1.0;
					for (int l = 0; l <= k; ++l) {
						dofs[l].v->value().deriv() = 1.0;

//						ScalarDiffDiff energy = computeForceForward(wheelAxisLocal, yawAxis, yawAngle, tiltAxis, tiltAngle, grf);

						ScalarDiffDiff fr = fabs(computeForceForward(wheelAxisLocal, yawAxis, yawAngle, tiltAxis, tiltAngle, grf));
						SoftUnilateralUpperConstraintT<ScalarDiffDiff> upperBound = SoftUnilateralUpperConstraintT<ScalarDiffDiff>(0, 10, theMotionPlan->frictionEpsilon);
						ScalarDiffDiff energy = upperBound.computeValue(fr-frictionCoeff*grf(1)) * weight;

						ADD_HES_ELEMENT(hessianEntries,
										dofs[k].i,
										dofs[l].i,
										energy.deriv().deriv(), 1.0);
						dofs[l].v->value().deriv() = 0.0;
					}
					dofs[k].v->deriv().value() = 0.0;
				}
			}
		}
	}
}
