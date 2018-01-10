#include <RobotDesignerLib/MPO_RobotWheelAxisObjective.h>

#include <MathLib/AutoDiff.h>

MPO_RobotWheelAxisObjective::MPO_RobotWheelAxisObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_RobotWheelAxisObjective::~MPO_RobotWheelAxisObjective(void) {
}

double MPO_RobotWheelAxisObjective::computeValue(const dVector& p) {
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	double retVal = 0;
	int nEEs = theMotionPlan->endEffectorTrajectories.size();
	for (int j = 0; j < theMotionPlan->nSamplePoints; j++) {
		dVector q_t;
		theMotionPlan->robotStateTrajectory.getQAtTimeIndex(j, q_t);

		for (int i = 0; i < nEEs; i++) {
			const LocomotionEngine_EndEffectorTrajectory &ee = theMotionPlan->endEffectorTrajectories[i];

			if (ee.isWheel)
			{
				retVal += computeEnergy(ee, q_t,
					ee.wheelYawAngle[j],
					ee.wheelTiltAngle[j]);
			}
		}
	}

	return retVal;
}

void MPO_RobotWheelAxisObjective::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	MatrixNxM dvdq;

	int nLimbs = theMotionPlan->endEffectorTrajectories.size();
	for (int j = 0; j < theMotionPlan->nSamplePoints; j++) {
		dVector q;
		theMotionPlan->robotStateTrajectory.getQAtTimeIndex(j, q);
		theMotionPlan->robotRepresentation->setQ(q);


		for (int i = 0; i < nLimbs; i++) {
			const LocomotionEngine_EndEffectorTrajectory &ee = theMotionPlan->endEffectorTrajectories[i];

			if (ee.isWheel)
			{
				if (theMotionPlan->wheelParamsStartIndex < 0)
					continue;

				Vector3d wheelAxisLocal_WF = ee.wheelAxisLocal_WF;
				Vector3d wheelAxisLocal_RBF = ee.endEffectorRB->rbProperties.endEffectorPoints[ee.CPIndex].localCoordsWheelAxis;

				double yawAngle = ee.wheelYawAngle[j];
				Vector3d yawAxis = ee.wheelYawAxis_WF;
				double tiltAngle = ee.wheelTiltAngle[j];
				Vector3d tiltAxis = ee.wheelTiltAxis_WF;

				// wheel axis from robot
				Vector3d wheelAxisRobot = theMotionPlan->robotRepresentation->getWorldCoordinatesForVectorT(wheelAxisLocal_RBF, ee.endEffectorRB, q);
				// wheel axis from wheel angles
				Vector3d wheelAxisWorld = LocomotionEngine_EndEffectorTrajectory::rotVecByYawTilt(wheelAxisLocal_WF, yawAxis, yawAngle, tiltAxis, tiltAngle);
				Vector3d err = wheelAxisWorld - wheelAxisRobot;

				//compute the gradient with respect to the robot q's
				theMotionPlan->robotRepresentation->compute_dvdq(wheelAxisLocal_RBF, ee.endEffectorRB, dvdq);

				//dEdee * deedq = dEdq
				int ind = theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim;
				grad.segment(ind, dvdq.cols()) -= weight*dvdq.transpose()*err;

				//compute the gradient with respect yaw and tilt angle
				Vector3d dYaw = LocomotionEngine_EndEffectorTrajectory::drotVecByYawTilt_dYaw(wheelAxisLocal_WF, yawAxis, yawAngle, tiltAxis, tiltAngle);
				grad(theMotionPlan->getWheelYawAngleIndex(i, j)) += weight*dYaw.transpose()*err;

				Vector3d dTilt = LocomotionEngine_EndEffectorTrajectory::drotVecByYawTilt_dTilt(wheelAxisLocal_WF, yawAxis, yawAngle, tiltAxis, tiltAngle);
				grad(theMotionPlan->getWheelTiltAngleIndex(i, j)) += weight*dTilt.transpose()*err;
			}
		}
	}
}

void MPO_RobotWheelAxisObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);
	int nLimbs = theMotionPlan->endEffectorTrajectories.size();

	MatrixNxM dvdq, ddvdq2;

	for (int j = 0; j < theMotionPlan->nSamplePoints; j++) {
		dVector q;
		theMotionPlan->robotStateTrajectory.getQAtTimeIndex(j, q);
		theMotionPlan->robotRepresentation->setQ(q);

		for (int i = 0; i < nLimbs; i++) {
			const LocomotionEngine_EndEffectorTrajectory &ee = theMotionPlan->endEffectorTrajectories[i];
			Vector3d wheelAxisLocal_WF = ee.wheelAxisLocal_WF;
			Vector3d wheelAxisLocal_RBF = ee.endEffectorRB->rbProperties.endEffectorPoints[ee.CPIndex].localCoordsWheelAxis;

			V3D yawAxis = ee.wheelYawAxis_WF;
			V3D tiltAxis = ee.wheelTiltAxis_WF;
			double yawAngle = ee.wheelYawAngle[j];
			double tiltAngle = ee.wheelTiltAngle[j];
			theMotionPlan->robotRepresentation->compute_dvdq(wheelAxisLocal_RBF, ee.endEffectorRB, dvdq);
			Vector3d dYaw = LocomotionEngine_EndEffectorTrajectory::drotVecByYawTilt_dYaw(wheelAxisLocal_WF, yawAxis, yawAngle, tiltAxis, tiltAngle);
			Vector3d dTilt = LocomotionEngine_EndEffectorTrajectory::drotVecByYawTilt_dTilt(wheelAxisLocal_WF, yawAxis, yawAngle, tiltAxis, tiltAngle);
			int Iyaw = theMotionPlan->getWheelYawAngleIndex(i, j);
			int Itilt = theMotionPlan->getWheelTiltAngleIndex(i, j);
				
			int Iq = theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim;

			// wheel axis from robot
			Vector3d wheelAxisRobot = theMotionPlan->robotRepresentation->getWorldCoordinatesForVectorT(wheelAxisLocal_RBF, ee.endEffectorRB, q);
			// wheel axis from wheel angles
			Vector3d wheelAxisWorld = LocomotionEngine_EndEffectorTrajectory::rotVecByYawTilt(wheelAxisLocal_WF, yawAxis, yawAngle, tiltAxis, tiltAngle);
			Vector3d err = wheelAxisWorld - wheelAxisRobot;

			//and now compute the gradient with respect to the robot q's
			if (theMotionPlan->robotStatesParamsStartIndex >= 0) {
				
				// second derivatives
				if (!hackHessian)
					for (int k = 0; k < theMotionPlan->robotRepresentation->getDimensionCount(); k++) {
						bool hasNonZeros = theMotionPlan->robotRepresentation->compute_ddvdq_dqi(wheelAxisLocal_RBF, ee.endEffectorRB, ddvdq2, k);
						if (hasNonZeros == false) continue;
						dVector V = -ddvdq2.transpose()*err;
						for (int l = k; l < theMotionPlan->robotRepresentation->getDimensionCount(); l++)
							ADD_HES_ELEMENT(hessianEntries, Iq + k, Iq + l, V(l), weight);
					}

				//outer product of the jacobians
				MatrixNxM outerProd = dvdq.row(0).transpose()*dvdq.row(0) +
					dvdq.row(1).transpose()*dvdq.row(1) +
					dvdq.row(2).transpose()*dvdq.row(2);

				for (int k = 0; k < theMotionPlan->robotRepresentation->getDimensionCount(); k++) {
					for (int l = k; l < theMotionPlan->robotRepresentation->getDimensionCount(); l++) {
						ADD_HES_ELEMENT(hessianEntries, Iq + k, Iq + l, outerProd(k, l), weight);
					}
				}
			}
			// Angles part
			if (theMotionPlan->wheelParamsStartIndex >= 0)
			{

				// grad(theMotionPlan->getWheelYawAngleIndex(i, j)) += weight*drhoRotdYawAngle.transpose()*err;
				// second derivatives
				Vector3d ddYaw = LocomotionEngine_EndEffectorTrajectory::ddrotVecByYawTilt_dYaw2(wheelAxisLocal_WF, yawAxis, yawAngle, tiltAxis, tiltAngle);
				Vector3d ddTilt = LocomotionEngine_EndEffectorTrajectory::ddrotVecByYawTilt_dTilt2(wheelAxisLocal_WF, yawAxis, yawAngle, tiltAxis, tiltAngle);
				Vector3d dYawdTilt = LocomotionEngine_EndEffectorTrajectory::ddrotVecByYawTilt_dYawdTilt(wheelAxisLocal_WF, yawAxis, yawAngle, tiltAxis, tiltAngle);
				
				if (!hackHessian)
				{
					ADD_HES_ELEMENT(hessianEntries, Iyaw, Iyaw, ddYaw.dot(err), weight);
					ADD_HES_ELEMENT(hessianEntries, Itilt, Itilt, ddTilt.dot(err), weight);
					ADD_HES_ELEMENT(hessianEntries, Iyaw, Itilt, dYawdTilt.dot(err), weight);
				}
				// outer product						
				ADD_HES_ELEMENT(hessianEntries, Iyaw, Iyaw, dYaw.dot(dYaw), weight);
				ADD_HES_ELEMENT(hessianEntries, Iyaw, Itilt, dYaw.dot(dTilt), weight);
				ADD_HES_ELEMENT(hessianEntries, Itilt, Itilt, dTilt.dot(dTilt), weight);
			}
			// mixed derivatives
// 			if (!hackHessian)
			if (theMotionPlan->wheelParamsStartIndex >= 0 && theMotionPlan->robotStatesParamsStartIndex >= 0)
			{
				MatrixNxM dvdqdTilt = dTilt.transpose()*dvdq;
				MatrixNxM dvdqdYaw = dYaw.transpose()*dvdq;
				for (int k = 0; k < theMotionPlan->robotRepresentation->getDimensionCount(); k++) {
					ADD_HES_ELEMENT(hessianEntries, Iq+k, Itilt, -dvdqdTilt(k), weight);
					ADD_HES_ELEMENT(hessianEntries, Iq+k, Iyaw, -dvdqdYaw(k), weight);
				}
			}
		}
	}
}
