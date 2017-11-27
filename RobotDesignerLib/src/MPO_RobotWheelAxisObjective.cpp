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
		theMotionPlan->robotRepresentation->setQ(q_t);
		for (int i=0;i<nEEs;i++){

			const V3D &wheelAxis =  theMotionPlan->endEffectorTrajectories[i].wheelAxis;

			// point at center of wheel in world coordinates
			P3D pO = theMotionPlan->robotRepresentation->getWorldCoordinatesFor(
						theMotionPlan->endEffectorTrajectories[i].endEffectorLocalCoords,
						theMotionPlan->endEffectorTrajectories[i].endEffectorRB
						);
			// point at 'end' of wheel axis in world coordinates
			P3D pW = theMotionPlan->robotRepresentation->getWorldCoordinatesFor(
						theMotionPlan->endEffectorTrajectories[i].endEffectorLocalCoords + wheelAxis,
						theMotionPlan->endEffectorTrajectories[i].endEffectorRB
						);
			// wheel axis in world coordinates, according to robot pose
			V3D currentAxis = pW-pO;

			// get wheel angles
			double alpha = theMotionPlan->endEffectorTrajectories[i].wheelYawAngle[j];
			double beta = theMotionPlan->endEffectorTrajectories[i].wheelTiltAngle[j];
			const V3D &yawAxis =  theMotionPlan->endEffectorTrajectories[i].wheelYawAxis;
			const V3D &tiltAxis =  theMotionPlan->endEffectorTrajectories[i].wheelTiltAxis;

			// wheel axis from wheel angles
			V3D axis = LocomotionEngine_EndEffectorTrajectory::rotateWheelAxisWith(wheelAxis, yawAxis, alpha, tiltAxis, beta);
			V3D err = axis - currentAxis;

			retVal += 0.5 * err.length2();
		}
	}

	return retVal * weight;
}

void MPO_RobotWheelAxisObjective::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);
	typedef AutoDiffT<double, double> ScalarDiff;

	MatrixNxM dOdq;
	MatrixNxM dWdq;

	int nLimbs = theMotionPlan->endEffectorTrajectories.size();
	for (int j=0; j<theMotionPlan->nSamplePoints; j++){
		dVector q_t;
		theMotionPlan->robotStateTrajectory.getQAtTimeIndex(j, q_t);
		theMotionPlan->robotRepresentation->setQ(q_t);

		for (int i=0;i<nLimbs;i++){

			const V3D &wheelAxis =  theMotionPlan->endEffectorTrajectories[i].wheelAxis;

			P3D pO = theMotionPlan->robotRepresentation->getWorldCoordinatesFor(
						theMotionPlan->endEffectorTrajectories[i].endEffectorLocalCoords,
						theMotionPlan->endEffectorTrajectories[i].endEffectorRB
						);
			P3D pW = theMotionPlan->robotRepresentation->getWorldCoordinatesFor(
						theMotionPlan->endEffectorTrajectories[i].endEffectorLocalCoords + wheelAxis,
						theMotionPlan->endEffectorTrajectories[i].endEffectorRB
						);
			V3D currentAxis = pW-pO;

			V3D err;
			{
				double alpha = theMotionPlan->endEffectorTrajectories[i].wheelYawAngle[j];
				double beta = theMotionPlan->endEffectorTrajectories[i].wheelTiltAngle[j];
				const V3D &yawAxis =  theMotionPlan->endEffectorTrajectories[i].wheelYawAxis;
				const V3D &tiltAxis =  theMotionPlan->endEffectorTrajectories[i].wheelTiltAxis;

				V3D axis = LocomotionEngine_EndEffectorTrajectory::rotateWheelAxisWith(wheelAxis, yawAxis, alpha, tiltAxis, beta);
				err = axis - currentAxis;
			}

			//compute the gradient with respect to the wheel axis rotation
			if (theMotionPlan->wheelParamsStartIndex >= 0){


				{
					ScalarDiff alpha = theMotionPlan->endEffectorTrajectories[i].wheelYawAngle[j];
					ScalarDiff beta = theMotionPlan->endEffectorTrajectories[i].wheelTiltAngle[j];
					V3T<ScalarDiff> wheelAxisAD(wheelAxis);
					V3T<ScalarDiff> yawAxis(theMotionPlan->endEffectorTrajectories[i].wheelYawAxis);
					V3T<ScalarDiff> tiltAxis(theMotionPlan->endEffectorTrajectories[i].wheelTiltAxis);
					// compute `d axis/d alpha`
					alpha.deriv() = 1.0;
					V3T<ScalarDiff> axisAD = LocomotionEngine_EndEffectorTrajectory::rotateWheelAxisWith(wheelAxisAD, yawAxis, alpha, tiltAxis, beta);
					V3D daxisdalpha; for (int k = 0; k < 3; ++k) daxisdalpha[k] = axisAD[k].deriv();
					alpha.deriv() = 0.0;

					grad[theMotionPlan->getWheelAxisAlphaIndex(i, j)] += err.dot(daxisdalpha) * weight;
				}

				{
					ScalarDiff alpha = theMotionPlan->endEffectorTrajectories[i].wheelYawAngle[j];
					ScalarDiff beta = theMotionPlan->endEffectorTrajectories[i].wheelTiltAngle[j];
					V3T<ScalarDiff> wheelAxisAD(wheelAxis);
					V3T<ScalarDiff> yawAxis(theMotionPlan->endEffectorTrajectories[i].wheelYawAxis);
					V3T<ScalarDiff> tiltAxis(theMotionPlan->endEffectorTrajectories[i].wheelTiltAxis);
					// compute `d axis/d beta`
					beta.deriv() = 1.0;
					Vector3T<ScalarDiff> axisAD = LocomotionEngine_EndEffectorTrajectory::rotateWheelAxisWith(wheelAxisAD, yawAxis, alpha, tiltAxis, beta);
					V3D daxisdbeta; for (int k = 0; k < 3; ++k) daxisdbeta[k] = axisAD[k].deriv();
					beta.deriv() = 0.0;

					grad[theMotionPlan->getWheelAxisBetaIndex(i, j)] += err.dot(daxisdbeta) * weight;
				}
			}

			//and now compute the gradient with respect to the robot q's
			if (theMotionPlan->robotStatesParamsStartIndex >= 0){
				theMotionPlan->robotRepresentation->compute_dpdq(
							theMotionPlan->endEffectorTrajectories[i].endEffectorLocalCoords,
							theMotionPlan->endEffectorTrajectories[i].endEffectorRB, dOdq
							);

				theMotionPlan->robotRepresentation->compute_dpdq(
							theMotionPlan->endEffectorTrajectories[i].endEffectorLocalCoords + wheelAxis,
							theMotionPlan->endEffectorTrajectories[i].endEffectorRB, dWdq
							);
	
				//dEdee * deedq = dEdq
				for (int k=0;k<3;k++)
					for (int l=0;l<theMotionPlan->robotRepresentation->getDimensionCount();l++)
						grad[theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + l]
								-= (dWdq(k, l) - dOdq(k, l)) * err[k] * weight;
			}
		}
	}

}

void MPO_RobotWheelAxisObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	typedef AutoDiffT<double, double> ScalarDiff;
	typedef AutoDiffT<ScalarDiff, ScalarDiff> ScalarDiffDiff;

	MatrixNxM dOdq;
	MatrixNxM dWdq;
	MatrixNxM ddOdq_dqi;
	MatrixNxM ddWdq_dqi;

	int nLimbs = theMotionPlan->endEffectorTrajectories.size();
	for (int j=0;j<theMotionPlan->nSamplePoints;j++){
		dVector q_t;
		theMotionPlan->robotStateTrajectory.getQAtTimeIndex(j, q_t);
		theMotionPlan->robotRepresentation->setQ(q_t);

		for (int i=0;i<nLimbs;i++){

			const V3D &wheelAxis =  theMotionPlan->endEffectorTrajectories[i].wheelAxis;
			P3D pO = theMotionPlan->robotRepresentation->getWorldCoordinatesFor(
						theMotionPlan->endEffectorTrajectories[i].endEffectorLocalCoords,
						theMotionPlan->endEffectorTrajectories[i].endEffectorRB
						);
			P3D pW = theMotionPlan->robotRepresentation->getWorldCoordinatesFor(
						theMotionPlan->endEffectorTrajectories[i].endEffectorLocalCoords + wheelAxis,
						theMotionPlan->endEffectorTrajectories[i].endEffectorRB
						);
			V3D currentAxis = pW-pO;

			ScalarDiffDiff alpha = theMotionPlan->endEffectorTrajectories[i].wheelYawAngle[j];
			ScalarDiffDiff beta = theMotionPlan->endEffectorTrajectories[i].wheelTiltAngle[j];
			V3T<ScalarDiffDiff> wheelAxisAD(wheelAxis);
			V3T<ScalarDiffDiff> yawAxis(theMotionPlan->endEffectorTrajectories[i].wheelYawAxis);
			V3T<ScalarDiffDiff> tiltAxis(theMotionPlan->endEffectorTrajectories[i].wheelTiltAxis);

			V3D axis_dalpha;
			V3D axis_dalpha2;
			V3D err;
			{
				alpha.deriv().value() = alpha.value().deriv() = 1.0;
				Vector3T<ScalarDiffDiff> axisAD = LocomotionEngine_EndEffectorTrajectory::rotateWheelAxisWith(wheelAxisAD, yawAxis, alpha, tiltAxis, beta);
				alpha.deriv().value() = alpha.value().deriv() = 0.0;

				for (int k = 0; k < 3; ++k) axis_dalpha[k] = axisAD[k].deriv().value();
				for (int k = 0; k < 3; ++k) axis_dalpha2[k] = axisAD[k].deriv().deriv();
				V3D axis;
				for (int k = 0; k < 3; ++k) axis[k] = axisAD[k].value().value();

				err = axis - currentAxis;
			}

			V3D axis_dbeta;
			V3D axis_dbeta2;
			{
				beta.deriv().value() = beta.value().deriv() = 1.0;
				V3T<ScalarDiffDiff> axisAD = LocomotionEngine_EndEffectorTrajectory::rotateWheelAxisWith(wheelAxisAD, yawAxis, alpha, tiltAxis, beta);
				beta.deriv().value() = beta.value().deriv() = 0.0;

				for (int k = 0; k < 3; ++k) axis_dbeta[k] = axisAD[k].deriv().value();
				for (int k = 0; k < 3; ++k) axis_dbeta2[k] = axisAD[k].deriv().deriv();
			}

			// compute ddE/dalpha_dalpha
			if (theMotionPlan->wheelParamsStartIndex >= 0){
				ADD_HES_ELEMENT(hessianEntries,
								theMotionPlan->getWheelAxisAlphaIndex(i, j),
								theMotionPlan->getWheelAxisAlphaIndex(i, j),
								axis_dalpha.dot(axis_dalpha) + err.dot(axis_dalpha2), weight);

				ADD_HES_ELEMENT(hessianEntries,
								theMotionPlan->getWheelAxisBetaIndex(i, j),
								theMotionPlan->getWheelAxisBetaIndex(i, j),
								axis_dbeta.dot(axis_dbeta) + err.dot(axis_dbeta2), weight);
			}


			//and now compute the gradient with respect to the robot q's
			if (theMotionPlan->robotStatesParamsStartIndex >= 0){
				theMotionPlan->robotRepresentation->compute_dpdq(
							theMotionPlan->endEffectorTrajectories[i].endEffectorLocalCoords,
							theMotionPlan->endEffectorTrajectories[i].endEffectorRB, dOdq
							);

				theMotionPlan->robotRepresentation->compute_dpdq(
							theMotionPlan->endEffectorTrajectories[i].endEffectorLocalCoords + wheelAxis,
							theMotionPlan->endEffectorTrajectories[i].endEffectorRB, dWdq
							);

				for (int k=0;k<theMotionPlan->robotRepresentation->getDimensionCount();k++){
					bool hasNonZeros = theMotionPlan->robotRepresentation->compute_ddpdq_dqi(
								theMotionPlan->endEffectorTrajectories[i].endEffectorLocalCoords,
								theMotionPlan->endEffectorTrajectories[i].endEffectorRB,
								ddOdq_dqi, k);
					hasNonZeros &= theMotionPlan->robotRepresentation->compute_ddpdq_dqi(
								theMotionPlan->endEffectorTrajectories[i].endEffectorLocalCoords + wheelAxis,
								theMotionPlan->endEffectorTrajectories[i].endEffectorRB,
								ddWdq_dqi, k);

					if (hasNonZeros == false) continue;
					for (int l=k;l<theMotionPlan->robotRepresentation->getDimensionCount();l++)
						for (int m=0;m<3;m++){
							double val = (ddOdq_dqi(m, l) - ddWdq_dqi(m, l)) * err[m];
						ADD_HES_ELEMENT(hessianEntries,
										theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + k,
										theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + l,
										val, weight);
						}
				}

				MatrixNxM dOdq_minus_dWdq = dOdq - dWdq;
				//now add the outer product of the jacobians...
				for (int k=0;k<theMotionPlan->robotRepresentation->getDimensionCount();k++){
					for (int l=k;l<theMotionPlan->robotRepresentation->getDimensionCount();l++){
						double val = 0;
						for (int m=0;m<3;m++)
							val += dOdq_minus_dWdq(m, k) * dOdq_minus_dWdq(m, l);
						ADD_HES_ELEMENT(hessianEntries,
										theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + k,
										theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + l,
										val, weight);
					}
				}

				//and now the mixed derivatives
				if (theMotionPlan->wheelParamsStartIndex >= 0){
					for (int k=0;k <theMotionPlan->robotRepresentation->getDimensionCount(); k++){
						{
							double val = 0;
							for (int m = 0; m < 3; ++m)
								val += dOdq_minus_dWdq(m, k) * axis_dalpha[m];
							ADD_HES_ELEMENT(hessianEntries,
											theMotionPlan->getWheelAxisAlphaIndex(i, j),
											theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + k,
											val, weight);
						}
						{
							double val = 0;
							for (int m = 0; m < 3; ++m)
								val += dOdq_minus_dWdq(m, k) * axis_dbeta[m];
							ADD_HES_ELEMENT(hessianEntries,
											theMotionPlan->getWheelAxisBetaIndex(i, j),
											theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + k,
											val, weight);
						}
					}
				}
			}
		}
	}

}
