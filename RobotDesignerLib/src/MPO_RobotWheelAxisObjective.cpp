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
				retVal += computeEnergy(ee.wheelAxisLocal, ee.endEffectorRB, q_t,
					ee.wheelYawAxis, ee.wheelYawAngle[j],
					ee.wheelTiltAxis, ee.wheelTiltAngle[j]);
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


		for (int i = 0; i < nLimbs; i++) {
			const LocomotionEngine_EndEffectorTrajectory &ee = theMotionPlan->endEffectorTrajectories[i];

			if (ee.isWheel)
			{
				if (theMotionPlan->wheelParamsStartIndex < 0)
					continue;

				Vector3d wheelAxisLocal = ee.wheelAxisLocal;

				double yawAngle = ee.wheelYawAngle[j];
				Vector3d yawAxis = ee.wheelYawAxis;
				double tiltAngle = ee.wheelTiltAngle[j];
				Vector3d tiltAxis = ee.wheelTiltAxis;

				// wheel axis from robot
				Vector3d wheelAxisRobot = theMotionPlan->robotRepresentation->getWorldCoordinatesForVectorT(wheelAxisLocal, ee.endEffectorRB, q);
				// wheel axis from wheel angles
				Vector3d wheelAxisWorld = LocomotionEngine_EndEffectorTrajectory::rotVecByYawTilt(wheelAxisLocal, yawAxis, yawAngle, tiltAxis, tiltAngle);
				Vector3d err = wheelAxisWorld - wheelAxisRobot;

				//compute the gradient with respect to the robot q's
				theMotionPlan->robotRepresentation->compute_dvdq(wheelAxisLocal, ee.endEffectorRB, dvdq);

				//dEdee * deedq = dEdq
				int ind = theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim;
				grad.segment(ind, dvdq.cols()) -= weight*dvdq.transpose()*err;

				//compute the gradient with respect yaw and tilt angle
				Vector3d drhoRotdYawAngle = LocomotionEngine_EndEffectorTrajectory::drotVecByYawTilt_dYaw(wheelAxisLocal, yawAxis, yawAngle, tiltAxis, tiltAngle);
				grad(theMotionPlan->getWheelYawAngleIndex(i, j)) += weight*drhoRotdYawAngle.transpose()*err;

				Vector3d drhoRotdTiltAngle = LocomotionEngine_EndEffectorTrajectory::drotVecByYawTilt_dTilt(wheelAxisLocal, yawAxis, yawAngle, tiltAxis, tiltAngle);
				grad(theMotionPlan->getWheelTiltAngleIndex(i, j)) += weight*drhoRotdTiltAngle.transpose()*err;
			}
		}
	}
}

void MPO_RobotWheelAxisObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);
	int nLimbs = theMotionPlan->endEffectorTrajectories.size();

	if (!hackHessian)
	{
		// number of DOFs for an end effector at one time sample
		int numDOFs = 0;
		// 2 DOFs for yaw and tilt angle
		if (theMotionPlan->wheelParamsStartIndex >= 0)
			numDOFs += 2;
		// size of robot state `q` as DOFs
		if (theMotionPlan->robotStatesParamsStartIndex >= 0)
			numDOFs += theMotionPlan->robotRepresentation->getDimensionCount();
		if (numDOFs == 0)
			return;
		for (int j = 0; j < theMotionPlan->nSamplePoints; j++) {
			dVector qd;
			theMotionPlan->robotStateTrajectory.getQAtTimeIndex(j, qd);
			VectorXT<ScalarDiffDiff> q(qd.size());
			for (int k = 0; k < qd.size(); ++k)
				q[k] = qd[k];

			for (int i = 0; i < nLimbs; i++) {
				const LocomotionEngine_EndEffectorTrajectory &ee = theMotionPlan->endEffectorTrajectories[i];

				if (ee.isWheel)
				{

					V3T<ScalarDiffDiff> eePosLocal = ee.endEffectorLocalCoords;
					V3T<ScalarDiffDiff> wheelAxisLocal = ee.wheelAxisLocal;

					ScalarDiffDiff yawAngle = ee.wheelYawAngle[j];
					V3T<ScalarDiffDiff> yawAxis = ee.wheelYawAxis;
					ScalarDiffDiff tiltAngle = ee.wheelTiltAngle[j];
					V3T<ScalarDiffDiff> tiltAxis = ee.wheelTiltAxis;

					std::vector<DOF<ScalarDiffDiff>> dofs(numDOFs);
					int index = 0;
					if (theMotionPlan->wheelParamsStartIndex >= 0) {
						dofs[index].v = &yawAngle;
						dofs[index].i = theMotionPlan->getWheelYawAngleIndex(i, j);
						index++;
						dofs[index].v = &tiltAngle;
						dofs[index].i = theMotionPlan->getWheelTiltAngleIndex(i, j);
						index++;
					}
					if (theMotionPlan->robotStatesParamsStartIndex >= 0) {
						for (int k = 0; k < q.size(); ++k) {
							dofs[index].v = &q[k];
							dofs[index].i = theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + k;
							index++;
						}
					}

					MatrixNxM localH(numDOFs, numDOFs);

					for (int k = 0; k < numDOFs; ++k) {
						dofs[k].v->deriv().value() = 1.0;
						for (int l = 0; l <= k; ++l) {
							dofs[l].v->value().deriv() = 1.0;
							ScalarDiffDiff energy = computeEnergy(wheelAxisLocal, ee.endEffectorRB, q,
								yawAxis, yawAngle,
								tiltAxis, tiltAngle);
							localH(k, l) = energy.deriv().deriv();
							dofs[l].v->value().deriv() = 0.0;
						}
						dofs[k].v->deriv().value() = 0.0;
					}

					if (hackHessian) // Roi: legacy. will not be run
					{
						Eigen::SelfAdjointEigenSolver<MatrixNxM> es(localH);
						Eigen::VectorXd D = es.eigenvalues();
						Eigen::MatrixXd U = es.eigenvectors();
						D = D.unaryExpr([](double x) {return (x < 1e-4) ? 1e-4 : x; });
						localH = U * D.asDiagonal()*U.transpose();
					}

					for (int k = 0; k < numDOFs; ++k) {
						for (int l = 0; l <= k; ++l) {
							ADD_HES_ELEMENT(hessianEntries,
								dofs[k].i,
								dofs[l].i,
								localH(k, l), 1.0);
						}
					}
				}
			}
		}
	}
	else {
		MatrixNxM dvdq, ddvdq2;

		for (int j = 0; j < theMotionPlan->nSamplePoints; j++) {
			dVector q;
			theMotionPlan->robotStateTrajectory.getQAtTimeIndex(j, q);

			for (int i = 0; i < nLimbs; i++) {
				const LocomotionEngine_EndEffectorTrajectory &ee = theMotionPlan->endEffectorTrajectories[i];
				Vector3d wheelAxisLocal = ee.wheelAxisLocal;
				V3D yawAxis = ee.wheelYawAxis;
				V3D tiltAxis = ee.wheelTiltAxis;
				double yawAngle = ee.wheelYawAngle[j];
				double tiltAngle = ee.wheelTiltAngle[j];
				theMotionPlan->robotRepresentation->compute_dvdq(wheelAxisLocal, ee.endEffectorRB, dvdq);

				// wheel axis from robot
				Vector3d wheelAxisRobot = theMotionPlan->robotRepresentation->getWorldCoordinatesForVectorT(wheelAxisLocal, ee.endEffectorRB, q);
				// wheel axis from wheel angles
				Vector3d wheelAxisWorld = LocomotionEngine_EndEffectorTrajectory::rotVecByYawTilt(wheelAxisLocal, yawAxis, yawAngle, tiltAxis, tiltAngle);
				Vector3d err = wheelAxisWorld - wheelAxisRobot;

				//and now compute the gradient with respect to the robot q's
				if (theMotionPlan->robotStatesParamsStartIndex >= 0) {
					int I = theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim;

					// second derivatives
					for (int k = 0; k < theMotionPlan->robotRepresentation->getDimensionCount(); k++) {
						bool hasNonZeros = theMotionPlan->robotRepresentation->compute_ddvdq_dqi(ee.endEffectorLocalCoords, ee.endEffectorRB, ddvdq2, k);
						if (hasNonZeros == false) continue;
						dVector V = ddvdq2.transpose()*err;
						for (int l = k; l < theMotionPlan->robotRepresentation->getDimensionCount(); l++)
							ADD_HES_ELEMENT(hessianEntries, I + k, I + l, V(l), weight);
					}

					//outer product of the jacobians
					MatrixNxM outerProd = dvdq.row(0).transpose()*dvdq.row(0) +
						dvdq.row(1).transpose()*dvdq.row(1) +
						dvdq.row(2).transpose()*dvdq.row(2);

					for (int k = 0; k < theMotionPlan->robotRepresentation->getDimensionCount(); k++) {
						for (int l = k; l < theMotionPlan->robotRepresentation->getDimensionCount(); l++) {
							ADD_HES_ELEMENT(hessianEntries, I + k, I + l, outerProd(k, l), weight);
						}
					}
				}
				// Angles part
				if (theMotionPlan->wheelParamsStartIndex >= 0)
					{
						// grad(theMotionPlan->getWheelYawAngleIndex(i, j)) -= weight*drhoRotdYawAngle.transpose()*err;
						Vector3d dYaw = LocomotionEngine_EndEffectorTrajectory::drotVecByYawTilt_dYaw(wheelAxisLocal, yawAxis, yawAngle, tiltAxis, tiltAngle);
						Vector3d dTilt = LocomotionEngine_EndEffectorTrajectory::drotVecByYawTilt_dTilt(wheelAxisLocal, yawAxis, yawAngle, tiltAxis, tiltAngle);

						
						int Iyaw = theMotionPlan->getWheelYawAngleIndex(i, j);
						int Itilt = theMotionPlan->getWheelTiltAngleIndex(i, j);
						ADD_HES_ELEMENT(hessianEntries, Iyaw, Iyaw, dYaw.dot(dYaw), weight);
						ADD_HES_ELEMENT(hessianEntries, Iyaw, Itilt, dYaw.dot(dTilt), weight);
						ADD_HES_ELEMENT(hessianEntries, Itilt, Itilt, dTilt.dot(dTilt), weight);
					}
				}
			// mixed derivatives
// 			if (theMotionPlan->wheelParamsStartIndex >= 0 && theMotionPlan->robotStatesParamsStartIndex >= 0)
// 			{
// 				for (int k = 0; k < theMotionPlan->robotRepresentation->getDimensionCount(); k++) {
// 					int I = theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3;
// 					int J = theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + k;
// 					ADD_HES_ELEMENT(hessianEntries, I, J, -dvdq(0, k), weight);
// 					ADD_HES_ELEMENT(hessianEntries, I + 1, J, -dvdq(1, k), weight);
// 					ADD_HES_ELEMENT(hessianEntries, I + 2, J, -dvdq(2, k), weight);
// 				}
// 			}
		}
	}
}

