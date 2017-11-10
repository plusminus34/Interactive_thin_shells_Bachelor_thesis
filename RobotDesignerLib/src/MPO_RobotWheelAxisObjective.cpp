#include <RobotDesignerLib/MPO_RobotWheelAxisObjective.h>

#include <MathLib/AutoDiff.h>

MPO_RobotWheelAxisObjective::MPO_RobotWheelAxisObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_RobotWheelAxisObjective::~MPO_RobotWheelAxisObjective(void){
}
#include <iostream>
double MPO_RobotWheelAxisObjective::computeValue(const dVector& p){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	double retVal = 0;
	int nLimbs = theMotionPlan->endEffectorTrajectories.size();
	for (int j=0;j<theMotionPlan->nSamplePoints;j++){
		dVector q_t;
		theMotionPlan->robotStateTrajectory.getQAtTimeIndex(j, q_t);
		theMotionPlan->robotRepresentation->setQ(q_t);
		for (int i=0;i<nLimbs;i++){
			P3D pO = theMotionPlan->robotRepresentation->getWorldCoordinatesFor(
						theMotionPlan->endEffectorTrajectories[i].endEffectorLocalCoords,
						theMotionPlan->endEffectorTrajectories[i].endEffectorRB
						);
			P3D pW = theMotionPlan->robotRepresentation->getWorldCoordinatesFor(
						theMotionPlan->endEffectorTrajectories[i].endEffectorLocalCoords + wheelAxis,
						theMotionPlan->endEffectorTrajectories[i].endEffectorRB
						);
			V3D currentAxis = pW-pO;
			double alpha = theMotionPlan->endEffectorTrajectories[i].wheelAxisAlpha[j];
			V3D axis = rotateVec(wheelAxis, alpha, V3D(0, 1, 0));
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
			P3D pO = theMotionPlan->robotRepresentation->getWorldCoordinatesFor(
						theMotionPlan->endEffectorTrajectories[i].endEffectorLocalCoords,
						theMotionPlan->endEffectorTrajectories[i].endEffectorRB
						);
			P3D pW = theMotionPlan->robotRepresentation->getWorldCoordinatesFor(
						theMotionPlan->endEffectorTrajectories[i].endEffectorLocalCoords + wheelAxis,
						theMotionPlan->endEffectorTrajectories[i].endEffectorRB
						);
			V3D currentAxis = pW-pO;

			// compute d_axis/d_alpha
			ScalarDiff alpha = theMotionPlan->endEffectorTrajectories[i].wheelAxisAlpha[j];
			Vector3T<ScalarDiff> wheelAxisAD;
			for (int k = 0; k < 3; ++k) wheelAxisAD[k] = wheelAxis[k];
			alpha.deriv() = 1.0;
			Vector3T<ScalarDiff> rotAround(0, 1, 0);
			Vector3T<ScalarDiff> axisAD = rotateVec(wheelAxisAD, alpha, rotAround);

			V3D axisDeriv;
			for (int k = 0; k < 3; ++k) axisDeriv[k] = axisAD[k].deriv();
			V3D axis;
			for (int k = 0; k < 3; ++k) axis[k] = axisAD[k].value();

			V3D err = axis - currentAxis;

			//compute the gradient with respect to the wheel axis rotation
			if (theMotionPlan->wheelParamsStartIndex >= 0){

				grad[theMotionPlan->getWheelAxisAlphaIndex(i, j)] += err.dot(axisDeriv) * weight;

//				std::cout << "wheel axis index: " << theMotionPlan->getWheelAxisAlphaIndex(i, j) << std::endl;
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
			P3D pO = theMotionPlan->robotRepresentation->getWorldCoordinatesFor(
						theMotionPlan->endEffectorTrajectories[i].endEffectorLocalCoords,
						theMotionPlan->endEffectorTrajectories[i].endEffectorRB
						);
			P3D pW = theMotionPlan->robotRepresentation->getWorldCoordinatesFor(
						theMotionPlan->endEffectorTrajectories[i].endEffectorLocalCoords + wheelAxis,
						theMotionPlan->endEffectorTrajectories[i].endEffectorRB
						);
			V3D currentAxis = pW-pO;

			// compute dd_axis/dd_alpha
			ScalarDiffDiff alpha = theMotionPlan->endEffectorTrajectories[i].wheelAxisAlpha[j];
			Vector3T<ScalarDiffDiff> wheelAxisAD;
			for (int k = 0; k < 3; ++k) wheelAxisAD[k] = wheelAxis[k];
			alpha.deriv().value() = alpha.value().deriv() = 1.0;
			Vector3T<ScalarDiffDiff> rotAround(0, 1, 0);
			Vector3T<ScalarDiffDiff> axisAD = rotateVec(wheelAxisAD, alpha, rotAround);

			V3D axis_dalpha;
			for (int k = 0; k < 3; ++k) axis_dalpha[k] = axisAD[k].deriv().value();
			V3D axis_dalpha2;
			for (int k = 0; k < 3; ++k) axis_dalpha2[k] = axisAD[k].deriv().deriv();
			V3D axis;
			for (int k = 0; k < 3; ++k) axis[k] = axisAD[k].value().value();

			V3D err = axis - currentAxis;

			// compute ddE/dalpha_dalpha
			if (theMotionPlan->wheelParamsStartIndex >= 0){
				ADD_HES_ELEMENT(hessianEntries,
								theMotionPlan->getWheelAxisAlphaIndex(i, j),
								theMotionPlan->getWheelAxisAlphaIndex(i, j),
								axis_dalpha.dot(axis_dalpha) + err.dot(axis_dalpha2), weight);
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
						double val = 0;
						for (int m = 0; m < 3; ++m)
							val += dOdq_minus_dWdq(m, k) * axis_dalpha[m];
						ADD_HES_ELEMENT(hessianEntries,
										theMotionPlan->getWheelAxisAlphaIndex(i, j),
										theMotionPlan->robotStatesParamsStartIndex + j * theMotionPlan->robotStateTrajectory.nStateDim + k,
										val, weight);
					}
				}
			}
		}
	}

}
