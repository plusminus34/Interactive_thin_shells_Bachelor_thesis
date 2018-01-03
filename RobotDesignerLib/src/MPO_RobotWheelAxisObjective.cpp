#include <RobotDesignerLib/MPO_RobotWheelAxisObjective.h>

#include <MathLib/AutoDiff.h>

MPO_RobotWheelAxisObjective::MPO_RobotWheelAxisObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_RobotWheelAxisObjective::~MPO_RobotWheelAxisObjective(void){
}

double MPO_RobotWheelAxisObjective::computeValue(const dVector& p){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	double retVal = 0;
	int nEEs = theMotionPlan->endEffectorTrajectories.size();
	for (int j=0;j<theMotionPlan->nSamplePoints;j++){
		dVector q_t;
		theMotionPlan->robotStateTrajectory.getQAtTimeIndex(j, q_t);

		for (int i=0;i<nEEs;i++){
			const LocomotionEngine_EndEffectorTrajectory &ee = theMotionPlan->endEffectorTrajectories[i];

			if(ee.isWheel)
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
	for (int j=0; j<theMotionPlan->nSamplePoints; j++){
		dVector q;
		theMotionPlan->robotStateTrajectory.getQAtTimeIndex(j, q);


		for (int i=0;i<nLimbs;i++){
			const LocomotionEngine_EndEffectorTrajectory &ee = theMotionPlan->endEffectorTrajectories[i];

			if(ee.isWheel)
			{
				if (theMotionPlan->wheelParamsStartIndex < 0)
					continue;

				Vector3d eePosLocal = ee.endEffectorLocalCoords;
				Vector3d wheelAxisLocal = ee.wheelAxisLocal;

				double yawAngle = ee.wheelYawAngle[j];
				Vector3d yawAxis = ee.wheelYawAxis;
				double tiltAngle = ee.wheelTiltAngle[j];
				Vector3d tiltAxis = ee.wheelTiltAxis;

				// wheel axis from robot
				Vector3d wheelAxisRobot = theMotionPlan->robotRepresentation->getWorldCoordinatesForVectorT(wheelAxisLocal, ee.endEffectorRB, q);
				// wheel axis from wheel angles
				Vector3d wheelAxisWorld = LocomotionEngine_EndEffectorTrajectory::rotateVectorUsingWheelAngles(wheelAxisLocal, yawAxis, yawAngle, tiltAxis, tiltAngle);
				Vector3d err = wheelAxisWorld - wheelAxisRobot;

				//compute the gradient with respect to the robot q's
				theMotionPlan->robotRepresentation->compute_dvdq(wheelAxisLocal, ee.endEffectorRB, dvdq);
				
				//dEdee * deedq = dEdq
				int ind = theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim;
				grad.segment(ind, dvdq.cols()) -= weight*dvdq.transpose()*err;

				//compute the gradient with respect yaw and tilt angle
				Vector3d drhoRotdYawAngle = LocomotionEngine_EndEffectorTrajectory::drotateVectorUsingWheelAngles_dYawAngle(wheelAxisLocal, yawAxis, yawAngle, tiltAxis, tiltAngle);
				grad(theMotionPlan->getWheelYawAngleIndex(i, j)) += weight*drhoRotdYawAngle.transpose()*err;

				Vector3d drhoRotdTiltAngle = LocomotionEngine_EndEffectorTrajectory::drotateVectorUsingWheelAngles_dTiltAngle(wheelAxisLocal, yawAxis, yawAngle, tiltAxis, tiltAngle);
				grad(theMotionPlan->getWheelTiltAngleIndex(i, j)) += weight*drhoRotdTiltAngle.transpose()*err;
			}
		}
	}
}

void MPO_RobotWheelAxisObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

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
	int nLimbs = theMotionPlan->endEffectorTrajectories.size();
	for (int j=0; j<theMotionPlan->nSamplePoints; j++){
		dVector qd;
		theMotionPlan->robotStateTrajectory.getQAtTimeIndex(j, qd);
		VectorXT<ScalarDiffDiff> q(qd.size());
		for (int k = 0; k < qd.size(); ++k)
			q[k] = qd[k];

		for (int i=0;i<nLimbs;i++){
			const LocomotionEngine_EndEffectorTrajectory &ee = theMotionPlan->endEffectorTrajectories[i];

			if(ee.isWheel)
			{
				
				V3T<ScalarDiffDiff> eePosLocal = ee.endEffectorLocalCoords;
				V3T<ScalarDiffDiff> wheelAxisLocal = ee.wheelAxisLocal;

				ScalarDiffDiff yawAngle = ee.wheelYawAngle[j];
				V3T<ScalarDiffDiff> yawAxis = ee.wheelYawAxis;
				ScalarDiffDiff tiltAngle = ee.wheelTiltAngle[j];
				V3T<ScalarDiffDiff> tiltAxis = ee.wheelTiltAxis;

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
				if (theMotionPlan->robotStatesParamsStartIndex >= 0){
					for (int k = 0; k < q.size(); ++k){
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

				if (hackHessian)
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
							localH(k,l), 1.0);
					}
				}
			}
		}
	}
}
