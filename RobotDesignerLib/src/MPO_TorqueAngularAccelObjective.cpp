#include <RobotDesignerLib/MPO_TorqueAngularAccelObjective.h>
#define DUMMY_ANGLE 1e-4

MPO_TorqueAngularAccelObjective::MPO_TorqueAngularAccelObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;

	dummyR.resize(theMotionPlan->nSamplePoints, Matrix3x3::Identity());
	dummyQ.resize(theMotionPlan->nSamplePoints, Matrix3x3::Identity());
}

double MPO_TorqueAngularAccelObjective::computeValue(const dVector& p) {
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	double err = 0;
	double h = theMotionPlan->motionPlanDuration / (theMotionPlan->nSamplePoints - 1);

	V3D axis[3];
	for (int i = 0; i < 3; i++) {
		axis[i] = theMotionPlan->COMTrajectory.getAxis(i);
	}

	int end = theMotionPlan->nSamplePoints;

	for (int j = 0; j < end; j++) {
		int jmm, jm, jp, jpp;

		theMotionPlan->getAccelerationTimeIndicesFor(j, jmm, jm, jp, jpp);
		if (jmm == -1 || jm == -1 || jp == -1 || jpp == -1) continue;

		AngleAxisd angVel_j, angVel_jp;
		Matrix3x3 R_jp, R_jpp, R_jmm, R_jm;
		R_jp.setIdentity(); R_jpp.setIdentity(); R_jmm.setIdentity(); R_jm.setIdentity();
		V3D eulerAngles_jpp = theMotionPlan->COMTrajectory.getCOMEulerAnglesAtTimeIndex(jpp);
		V3D eulerAngles_jp = theMotionPlan->COMTrajectory.getCOMEulerAnglesAtTimeIndex(jp);
		V3D eulerAngles_jmm = theMotionPlan->COMTrajectory.getCOMEulerAnglesAtTimeIndex(jmm);
		V3D eulerAngles_jm = theMotionPlan->COMTrajectory.getCOMEulerAnglesAtTimeIndex(jm);

		for (int i = 0; i < 3; i++) {
			R_jp *= getRotationQuaternion(eulerAngles_jp[i], axis[i]).getRotationMatrix();
			R_jpp *= getRotationQuaternion(eulerAngles_jpp[i], axis[i]).getRotationMatrix();
			R_jmm *= getRotationQuaternion(eulerAngles_jmm[i], axis[i]).getRotationMatrix();
			R_jm *= getRotationQuaternion(eulerAngles_jm[i], axis[i]).getRotationMatrix();
		}

		//note: dummies are used to prevent rotations from being exactly the identity, as the axis of that rotation is not defined
		//these are the angular velocities that, when applied for time h, lead from R_t to R_t+1: R_t+1 = R(w * h) * R_t
		angVel_jp = (dummyR[j] * R_jpp * R_jp.transpose()); angVel_jp.angle() *= 1 / h;
		angVel_j = (dummyQ[j] * R_jm * R_jmm.transpose()); angVel_j.angle() *= 1 / h;

		//w_t+1 = w_t + h * angAccel
		Vector3d angularAccel = (angVel_jp.axis() * angVel_jp.angle() - angVel_j.axis() * angVel_j.angle()) / h;

		//the rate of change of angular momentum, dLdt, needs to be equal to the net torque. If we assume the moment of intertia I (L=I*ang_vel) is constant, then dLdt = I*ang_acc
		Vector3d dLdt = angularAccel * theMotionPlan->totalInertia;

		Vector3d torque(0, 0, 0);

		for (uint i = 0; i < theMotionPlan->endEffectorTrajectories.size(); i++) {
			V3D R = theMotionPlan->endEffectorTrajectories[i].EEPos[j] - theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(j);
			torque += R.cross(theMotionPlan->endEffectorTrajectories[i].contactForce[j] * theMotionPlan->endEffectorTrajectories[i].contactFlag[j]);
		}

		err += 0.5 * weight * (torque - dLdt).squaredNorm();
	}

	return err;
}

void MPO_TorqueAngularAccelObjective::addGradientTo(dVector& grad, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	double h = theMotionPlan->motionPlanDuration / (theMotionPlan->nSamplePoints - 1);

	V3D axis[3];
	Matrix3x3 crossMat[3];
	for (int i = 0; i < 3; i++)
	{
		axis[i] = axis[i] = theMotionPlan->COMTrajectory.getAxis(i);
		crossMat[i] = getCrossProductMatrix(axis[i]);
	}

	int end = theMotionPlan->nSamplePoints;

	for (int j = 0; j < end; j++) {
		int jmm, jm, jp, jpp;

		theMotionPlan->getAccelerationTimeIndicesFor(j, jmm, jm, jp, jpp);
		if (jmm == -1 || jm == -1 || jp == -1 || jpp == -1) continue;

		AngleAxisd tmpAngleAxis1, tmpAngleAxis2;
		Matrix3x3 A1[3], A2[3], A3[3], A4[3];
		Matrix3x3 R1, R2, R3, R4;
		R1.setIdentity(); R2.setIdentity(); R3.setIdentity(); R4.setIdentity();
		V3D eulerAngles_jpp = theMotionPlan->COMTrajectory.getCOMEulerAnglesAtTimeIndex(jpp);
		V3D eulerAngles_jp = theMotionPlan->COMTrajectory.getCOMEulerAnglesAtTimeIndex(jp);
		V3D eulerAngles_jmm = theMotionPlan->COMTrajectory.getCOMEulerAnglesAtTimeIndex(jmm);
		V3D eulerAngles_jm = theMotionPlan->COMTrajectory.getCOMEulerAnglesAtTimeIndex(jm);

		for (int i = 0; i < 3; i++)
		{
			A1[i] = getRotationQuaternion(eulerAngles_jp[i], axis[i]).getRotationMatrix();
			A2[i] = getRotationQuaternion(eulerAngles_jpp[i], axis[i]).getRotationMatrix();
			A3[i] = getRotationQuaternion(eulerAngles_jmm[i], axis[i]).getRotationMatrix();
			A4[i] = getRotationQuaternion(eulerAngles_jm[i], axis[i]).getRotationMatrix();
			R1 *= A1[i];
			R2 *= A2[i];
			R3 *= A3[i];
			R4 *= A4[i];
		}

		Matrix3x3 R = dummyR[j] * R2 * R1.transpose();
		Matrix3x3 Q = dummyQ[j] * R4 * R3.transpose();

		tmpAngleAxis1 = R;
		tmpAngleAxis2 = Q;

		double scale = theMotionPlan->totalInertia / (h * h);
		V3D angularAccel = tmpAngleAxis1.axis() * tmpAngleAxis1.angle() - tmpAngleAxis2.axis() * tmpAngleAxis2.angle();
		angularAccel *= scale;

		V3D torque(0, 0, 0);

		for (uint i = 0; i < theMotionPlan->endEffectorTrajectories.size(); i++) {

			if (theMotionPlan->endEffectorTrajectories[i].contactFlag[j] > 0) {

				V3D R = theMotionPlan->endEffectorTrajectories[i].EEPos[j] - theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(j);
				torque += R.cross(theMotionPlan->endEffectorTrajectories[i].contactForce[j]);
			}
		}

		//************************* Angle-Axis Gradient ************************

		Matrix3x3 dw1, dw2, dw3, dw4;
		Matrix3x3 dwdR[3]; // represent the derivative matrix of dw / dR, dwdR[0] represent dw_x / dR ...
		Matrix3x3 dw00dR[3], dw01dR[3], dw02dR[3], dw12dR[3];
		Matrix3x3 dR1[3], dR2[3], dR3[3], dR4[4];

		getdwdRFromRotationMatrix(dwdR, R);

		if (theMotionPlan->COMOrientationsParamsStartIndex > -1)
		{
			// jp

			dR1[0] = dummyR[j] * R2 * (crossMat[0] * R1).transpose();
			dR1[1] = dummyR[j] * R2 * (A1[0] * crossMat[1] * A1[1] * A1[2]).transpose();
			dR1[2] = dummyR[j] * R2 * (A1[0] * A1[1] * crossMat[2] * A1[2]).transpose();
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					dw1(i, j) = (dwdR[j].array() * dR1[i].array()).sum();

			grad.segment<3>(theMotionPlan->COMOrientationsParamsStartIndex + 3 * jp)
				+= weight * scale * dw1 * (angularAccel - torque);

			// jpp
			dR2[0] = dummyR[j] * crossMat[0] * R2 * R1.transpose();
			dR2[1] = dummyR[j] * A2[0] * crossMat[1] * A2[1] * A2[2] * R1.transpose();
			dR2[2] = dummyR[j] * A2[0] * A2[1] * crossMat[2] * A2[2] * R1.transpose();

			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					dw2(i, j) = (dwdR[j].array() * dR2[i].array()).sum();

			grad.segment<3>(theMotionPlan->COMOrientationsParamsStartIndex + 3 * jpp)
				+= weight * scale * dw2 * (angularAccel - torque);


			getdwdRFromRotationMatrix(dwdR, Q);

			// jmm
			dR3[0] = dummyQ[j] * R4 * (crossMat[0] * R3).transpose();
			dR3[1] = dummyQ[j] * R4 * (A3[0] * crossMat[1] * A3[1] * A3[2]).transpose();
			dR3[2] = dummyQ[j] * R4 * (A3[0] * A3[1] * crossMat[2] * A3[2]).transpose();
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					dw3(i, j) = (dwdR[j].array() * dR3[i].array()).sum();

			grad.segment<3>(theMotionPlan->COMOrientationsParamsStartIndex + 3 * jmm)
				-= weight * scale * dw3 * (angularAccel - torque);

			// jm
			dR4[0] = dummyQ[j] * crossMat[0] * R4 * R3.transpose();
			dR4[1] = dummyQ[j] * A4[0] * crossMat[1] * A4[1] * A4[2] * R3.transpose();
			dR4[2] = dummyQ[j] * A4[0] * A4[1] * crossMat[2] * A4[2] * R3.transpose();
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					dw4(i, j) = (dwdR[j].array() * dR4[i].array()).sum();

			grad.segment<3>(theMotionPlan->COMOrientationsParamsStartIndex + 3 * jm)
				-= weight * scale * dw4 * (angularAccel - torque);
		}

		//************************* Torque Gradient ************************

		for (uint i = 0; i < theMotionPlan->endEffectorTrajectories.size(); i++) {

			if (theMotionPlan->endEffectorTrajectories[i].contactFlag[j] > 0) {

				V3D R = theMotionPlan->endEffectorTrajectories[i].EEPos[j] - theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(j);
				V3D F = theMotionPlan->endEffectorTrajectories[i].contactForce[j];

				Matrix3x3 dTdF = -getCrossProductMatrix(R);
				Matrix3x3 dTdE = getCrossProductMatrix(F);
				Matrix3x3 dTdC = -dTdE;

				if (theMotionPlan->contactForcesParamsStartIndex > -1)
				{
					grad.segment<3>(theMotionPlan->contactForcesParamsStartIndex +
						3 * (j * theMotionPlan->endEffectorTrajectories.size() + i))
						+= weight * dTdF * (torque - angularAccel);
				}

				if (theMotionPlan->COMPositionsParamsStartIndex > -1)
				{
					grad.segment<3>(theMotionPlan->COMPositionsParamsStartIndex + 3 * j)
						+= weight * dTdC * (torque - angularAccel);
				}

				if (theMotionPlan->feetPositionsParamsStartIndex > -1)
				{
					V3D tmp = dTdE * (torque - angularAccel);
					grad.segment<2>(theMotionPlan->feetPositionsParamsStartIndex +
						2 * (j * theMotionPlan->endEffectorTrajectories.size() + i))
						+= weight * Vector2d(tmp[0], tmp[2]);
				}
			}
		}
	}

}

void MPO_TorqueAngularAccelObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);

	double h = theMotionPlan->motionPlanDuration / (theMotionPlan->nSamplePoints - 1);

	V3D axis[3];
	Matrix3x3 crossMat[3];
	for (int i = 0; i < 3; i++)
	{
		axis[i] = theMotionPlan->COMTrajectory.getAxis(i);
		crossMat[i] = getCrossProductMatrix(axis[i]);
	}

	int end = theMotionPlan->nSamplePoints;

	for (int j = 0; j < end; j++) {
		int jmm, jm, jp, jpp;

		theMotionPlan->getAccelerationTimeIndicesFor(j, jmm, jm, jp, jpp);
		if (jmm == -1 || jm == -1 || jp == -1 || jpp == -1) continue;

		AngleAxisd tmpAngleAxis1, tmpAngleAxis2;
		Matrix3x3 A1[3], A2[3], A3[3], A4[3];
		Matrix3x3 R1, R2, R3, R4;
		R1.setIdentity(); R2.setIdentity(); R3.setIdentity(); R4.setIdentity();
		V3D eulerAngles_jpp = theMotionPlan->COMTrajectory.getCOMEulerAnglesAtTimeIndex(jpp);
		V3D eulerAngles_jp = theMotionPlan->COMTrajectory.getCOMEulerAnglesAtTimeIndex(jp);
		V3D eulerAngles_jmm = theMotionPlan->COMTrajectory.getCOMEulerAnglesAtTimeIndex(jmm);
		V3D eulerAngles_jm = theMotionPlan->COMTrajectory.getCOMEulerAnglesAtTimeIndex(jm);

		for (int i = 0; i < 3; i++)
		{
			A1[i] = getRotationQuaternion(eulerAngles_jp[i], axis[i]).getRotationMatrix();
			A2[i] = getRotationQuaternion(eulerAngles_jpp[i], axis[i]).getRotationMatrix();
			A3[i] = getRotationQuaternion(eulerAngles_jmm[i], axis[i]).getRotationMatrix();
			A4[i] = getRotationQuaternion(eulerAngles_jm[i], axis[i]).getRotationMatrix();
			R1 *= A1[i];
			R2 *= A2[i];
			R3 *= A3[i];
			R4 *= A4[i];
		}

		Matrix3x3 R = dummyR[j] * R2 * R1.transpose();
		Matrix3x3 Q = dummyQ[j] * R4 * R3.transpose();

		tmpAngleAxis1 = R;
		tmpAngleAxis2 = Q;

		double scale = theMotionPlan->totalInertia / (h * h);
		V3D angularAccel = tmpAngleAxis1.axis() * tmpAngleAxis1.angle() - tmpAngleAxis2.axis() * tmpAngleAxis2.angle();
		angularAccel *= scale;

		V3D torque(0, 0, 0);

		for (uint i = 0; i < theMotionPlan->endEffectorTrajectories.size(); i++) {

			if (theMotionPlan->endEffectorTrajectories[i].contactFlag[j] > 0) {

				V3D R = theMotionPlan->endEffectorTrajectories[i].EEPos[j] - theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(j);
				torque += R.cross(theMotionPlan->endEffectorTrajectories[i].contactForce[j]);
			}
		}

		Matrix3x3 dw1, dw2, dw3, dw4;
		Vector3d vec = angularAccel - torque;

		TorqueInnerHessianHelper(hessianEntries, -vec, j);

		if (theMotionPlan->COMOrientationsParamsStartIndex == -1)
			continue;

		AngleAxisInnerHessianHelper(hessianEntries, vec, jp, jpp, dummyR[j], R, R1, R2, A1, A2, crossMat, dw1, dw2, scale);
		AngleAxisInnerHessianHelper(hessianEntries, vec, jmm, jm, dummyQ[j], Q, R3, R4, A3, A4, crossMat, dw3, dw4, -scale);

		AngleAxisOutterHessianHelper(hessianEntries, jpp, jp, jm, jmm, dw1, dw2, dw3, dw4, scale);

		AngleAxisTorqueCrossHessianHelper(hessianEntries, j, jpp, jp, jm, jmm, dw1, dw2, dw3, dw4, scale);
	}

}

void MPO_TorqueAngularAccelObjective::AngleAxisTorqueCrossHessianHelper(DynamicArray<MTriplet>& hessianEntries, int j, int jpp, int jp, int jm, int jmm, Matrix3x3& dw1, Matrix3x3& dw2, Matrix3x3& dw3, Matrix3x3& dw4, double scale)
{
	int startIndexjmm = theMotionPlan->COMOrientationsParamsStartIndex + 3 * jmm;
	int startIndexjp = theMotionPlan->COMOrientationsParamsStartIndex + 3 * jp;
	int startIndexjm = theMotionPlan->COMOrientationsParamsStartIndex + 3 * jm;
	int startIndexjpp = theMotionPlan->COMOrientationsParamsStartIndex + 3 * jpp;

	for (uint i = 0; i < theMotionPlan->endEffectorTrajectories.size(); i++) {

		if (theMotionPlan->endEffectorTrajectories[i].contactFlag[j] > 0) {

			V3D R = theMotionPlan->endEffectorTrajectories[i].EEPos[j] - theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(j);
			V3D F = theMotionPlan->endEffectorTrajectories[i].contactForce[j];

			Matrix3x3 dTdF = -getCrossProductMatrix(R);
			Matrix3x3 dTdE = getCrossProductMatrix(F);
			Matrix3x3 dTdC = -dTdE;

			int startIndexF = theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i);
			int startIndexC = theMotionPlan->COMPositionsParamsStartIndex + 3 * j;
			int startIndexE = theMotionPlan->feetPositionsParamsStartIndex + 2 * (j * theMotionPlan->endEffectorTrajectories.size() + i);

			if (theMotionPlan->contactForcesParamsStartIndex > -1)
			{
				Matrix3x3 H;
				H = -dTdF * scale * dw1.transpose();

				for (int i = 0; i < 3; i++)
					for (int j = 0; j < 3; j++)
						ADD_HES_ELEMENT(hessianEntries, startIndexF + i, startIndexjp + j, H(i, j), weight);

				H = -dTdF * scale * dw2.transpose();

				for (int i = 0; i < 3; i++)
					for (int j = 0; j < 3; j++)
						ADD_HES_ELEMENT(hessianEntries, startIndexF + i, startIndexjpp + j, H(i, j), weight);

				H = dTdF * scale * dw3.transpose();

				for (int i = 0; i < 3; i++)
					for (int j = 0; j < 3; j++)
						ADD_HES_ELEMENT(hessianEntries, startIndexF + i, startIndexjmm + j, H(i, j), weight);

				H = dTdF * scale * dw4.transpose();

				for (int i = 0; i < 3; i++)
					for (int j = 0; j < 3; j++)
						ADD_HES_ELEMENT(hessianEntries, startIndexF + i, startIndexjm + j, H(i, j), weight);

			}

			if (theMotionPlan->COMPositionsParamsStartIndex > -1)
			{
				Matrix3x3 H;
				H = -dTdC * scale * dw1.transpose();

				for (int i = 0; i < 3; i++)
					for (int j = 0; j < 3; j++)
						ADD_HES_ELEMENT(hessianEntries, startIndexC + i, startIndexjp + j, H(i, j), weight);

				H = -dTdC * scale * dw2.transpose();

				for (int i = 0; i < 3; i++)
					for (int j = 0; j < 3; j++)
						ADD_HES_ELEMENT(hessianEntries, startIndexC + i, startIndexjpp + j, H(i, j), weight);

				H = dTdC * scale * dw3.transpose();

				for (int i = 0; i < 3; i++)
					for (int j = 0; j < 3; j++)
						ADD_HES_ELEMENT(hessianEntries, startIndexC + i, startIndexjmm + j, H(i, j), weight);

				H = dTdC * scale * dw4.transpose();

				for (int i = 0; i < 3; i++)
					for (int j = 0; j < 3; j++)
						ADD_HES_ELEMENT(hessianEntries, startIndexC + i, startIndexjm + j, H(i, j), weight);
			}

			if (theMotionPlan->feetPositionsParamsStartIndex > -1)
			{
				Matrix3x3 H;
				H = -dTdE * scale * dw1.transpose();

				for (int i = 0; i < 2; i++)
					for (int j = 0; j < 3; j++)
						ADD_HES_ELEMENT(hessianEntries, startIndexE + i, startIndexjp + j, H(2 * i, j), weight);

				H = -dTdE * scale * dw2.transpose();

				for (int i = 0; i < 2; i++)
					for (int j = 0; j < 3; j++)
						ADD_HES_ELEMENT(hessianEntries, startIndexE + i, startIndexjpp + j, H(2 * i, j), weight);

				H = dTdE * scale * dw3.transpose();

				for (int i = 0; i < 2; i++)
					for (int j = 0; j < 3; j++)
						ADD_HES_ELEMENT(hessianEntries, startIndexE + i, startIndexjmm + j, H(2 * i, j), weight);

				H = dTdE * scale * dw4.transpose();

				for (int i = 0; i < 2; i++)
					for (int j = 0; j < 3; j++)
						ADD_HES_ELEMENT(hessianEntries, startIndexE + i, startIndexjm + j, H(2 * i, j), weight);
			}

		}
	}


}

void MPO_TorqueAngularAccelObjective::TorqueInnerHessianHelper(DynamicArray<MTriplet>& hessianEntries, Vector3d vec, int j)
{
	Matrix3x3 tensor[3];
	getCrossProductTensor(tensor);

	for (uint i = 0; i < theMotionPlan->endEffectorTrajectories.size(); i++) {

		if (theMotionPlan->endEffectorTrajectories[i].contactFlag[j] > 0) {

			V3D R = theMotionPlan->endEffectorTrajectories[i].EEPos[j] - theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(j);
			V3D F = theMotionPlan->endEffectorTrajectories[i].contactForce[j];

			Matrix3x3 dTdF = -getCrossProductMatrix(R);
			Matrix3x3 dTdE = getCrossProductMatrix(F);
			Matrix3x3 dTdC = -dTdE;

			int startIndexF = theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + i);
			int startIndexC = theMotionPlan->COMPositionsParamsStartIndex + 3 * j;
			int startIndexE = theMotionPlan->feetPositionsParamsStartIndex + 2 * (j * theMotionPlan->endEffectorTrajectories.size() + i);

			for (uint k = 0; k < theMotionPlan->endEffectorTrajectories.size(); k++)
			{
				if (theMotionPlan->endEffectorTrajectories[k].contactFlag[j] > 0) {

					V3D t_R = theMotionPlan->endEffectorTrajectories[k].EEPos[j] - theMotionPlan->COMTrajectory.getCOMPositionAtTimeIndex(j);
					V3D t_F = theMotionPlan->endEffectorTrajectories[k].contactForce[j];

					Matrix3x3 t_dTdF = -getCrossProductMatrix(t_R);
					Matrix3x3 t_dTdE = getCrossProductMatrix(t_F);
					Matrix3x3 t_dTdC = -t_dTdE;

					int t_startIndexF = theMotionPlan->contactForcesParamsStartIndex + 3 * (j * theMotionPlan->endEffectorTrajectories.size() + k);
					int t_startIndexE = theMotionPlan->feetPositionsParamsStartIndex + 2 * (j * theMotionPlan->endEffectorTrajectories.size() + k);

					if (theMotionPlan->contactForcesParamsStartIndex > -1)
					{
						// dTdF / dF					
						Matrix3x3 H;
						if (k >= i) {
							H = t_dTdF * dTdF.transpose();

							for (int p = 0; p < 3; p++)
								for (int j = 0; j <= (i == k ? p : 2); j++)
									ADD_HES_ELEMENT(hessianEntries, t_startIndexF + p, startIndexF + j, H(p, j), weight);
						}

						// dTdF / dE
						if (theMotionPlan->feetPositionsParamsStartIndex > -1) {
							for (int p = 0; p < 3; p++)
							{
								if (i == k)
									H.row(p) = tensor[p] * vec + dTdF * dTdE.row(p).transpose();
								else
									H.row(p) = dTdF * t_dTdE.row(p).transpose();
							}

							for (int i = 0; i < 2; i++)
								for (int j = 0; j < 3; j++)
									ADD_HES_ELEMENT(hessianEntries, t_startIndexE + i, startIndexF + j, H(i * 2, j), weight);
						}


						// dTdF / dC
						if (theMotionPlan->COMPositionsParamsStartIndex > -1) {
							for (int p = 0; p < 3; p++)
							{
								if (i == k)
									H.row(p) = -tensor[p] * vec + dTdF * dTdC.row(p).transpose();
								else
									H.row(p) = dTdF * t_dTdC.row(p).transpose();
							}
							for (int i = 0; i < 3; i++)
								for (int j = 0; j < 3; j++)
									ADD_HES_ELEMENT(hessianEntries, startIndexC + i, startIndexF + j, H(i, j), weight);
						}

					}

					if (theMotionPlan->COMPositionsParamsStartIndex > -1)
					{
						// dTdC / dC
						Matrix3x3 H;
						if (true) {
							H = t_dTdC * dTdC.transpose();

							for (int p = 0; p < 3; p++)
								for (int j = 0; j <= p; j++)
									ADD_HES_ELEMENT(hessianEntries, startIndexC + p, startIndexC + j, H(p, j), weight);
						}

						// dTdC / dE
						if (theMotionPlan->feetPositionsParamsStartIndex > -1)
						{
							H = t_dTdE * dTdC.transpose();

							for (int i = 0; i < 2; i++)
								for (int j = 0; j < 3; j++)
									ADD_HES_ELEMENT(hessianEntries, t_startIndexE + i, startIndexC + j, H(i * 2, j), weight);
						}

					}

					if (theMotionPlan->feetPositionsParamsStartIndex > -1)
					{
						// dTdE / dE
						if (k >= i) {
							Matrix3x3 H;
							H = t_dTdE * dTdE.transpose();

							for (int p = 0; p < 2; p++)
								for (int j = 0; j <= (i == k ? p : 1); j++)
									ADD_HES_ELEMENT(hessianEntries, t_startIndexE + p, startIndexE + j, H(p * 2, j * 2), weight);
						}
					}
				}


			}


		}
	}
}

void MPO_TorqueAngularAccelObjective::AngleAxisOutterHessianHelper(DynamicArray<MTriplet>& hessianEntries, int jpp, int jp, int jm, int jmm, Matrix3x3& dw1, Matrix3x3& dw2, Matrix3x3& dw3, Matrix3x3& dw4, double scale)
{
	int startIndexjmm = theMotionPlan->COMOrientationsParamsStartIndex + 3 * jmm;
	int startIndexjp = theMotionPlan->COMOrientationsParamsStartIndex + 3 * jp;
	int startIndexjm = theMotionPlan->COMOrientationsParamsStartIndex + 3 * jm;
	int startIndexjpp = theMotionPlan->COMOrientationsParamsStartIndex + 3 * jpp;

	// jp / jmm
	Matrix3x3 H;
	H = -scale * dw3 * scale * dw1.transpose();

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			ADD_HES_ELEMENT(hessianEntries, startIndexjmm + i, startIndexjp + j, H(i, j), weight);

	// jp / jm
	H = -scale * dw4 * scale * dw1.transpose();

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			ADD_HES_ELEMENT(hessianEntries, startIndexjm + i, startIndexjp + j, (i == j && jm == jp ? 2 : 1) * H(i, j), weight);

	// jpp / jmm
	H = -scale * dw3 * scale * dw2.transpose();

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			ADD_HES_ELEMENT(hessianEntries, startIndexjmm + i, startIndexjpp + j, H(i, j), weight);

	// jpp / jm
	H = -scale * dw4 * scale * dw2.transpose();

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			ADD_HES_ELEMENT(hessianEntries, startIndexjm + i, startIndexjpp + j, H(i, j), weight);
}


void MPO_TorqueAngularAccelObjective::AngleAxisInnerHessianHelper(DynamicArray<MTriplet>& hessianEntries, Vector3d vec, int index1, int index2, Matrix3x3& dummy, Matrix3x3& R, Matrix3x3& R1, Matrix3x3& R2, Matrix3x3* A1, Matrix3x3* A2, Matrix3x3* crossMat, Matrix3x3& dw1, Matrix3x3& dw2, double scale)
{
	Matrix3x3 dwdR[3]; // represent the derivative matrix of dw / dR, dwdR[0] represent dw_x / dR ...
	Matrix3x3 dw00dR[3], dw01dR[3], dw02dR[3], dw12dR[3];
	Matrix3x3 dR1[3], dR2[3], dR3[3], dR4[4];

	getdwdRFromRotationMatrix(dwdR, dw00dR, dw01dR, dw02dR, dw12dR, R);

	// jp
	dR1[0] = dummy * R2 * (crossMat[0] * R1).transpose();
	dR1[1] = dummy * R2 * (A1[0] * crossMat[1] * A1[1] * A1[2]).transpose();
	dR1[2] = dummy * R2 * (A1[0] * A1[1] * crossMat[2] * A1[2]).transpose();

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			dw1(i, j) = (dwdR[j].array() * dR1[i].array()).sum();

	// jpp
	dR2[0] = dummy * crossMat[0] * R2 * R1.transpose();
	dR2[1] = dummy * A2[0] * crossMat[1] * A2[1] * A2[2] * R1.transpose();
	dR2[2] = dummy * A2[0] * A2[1] * crossMat[2] * A2[2] * R1.transpose();

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			dw2(i, j) = (dwdR[j].array() * dR2[i].array()).sum();

	Matrix3x3 dR1dR1[3][3], dR2dR2[3][3], dR1dR2[3][3];

	dR1dR1[0][0] = dummy * R2 * (crossMat[0] * crossMat[0] * R1).transpose();
	dR1dR1[1][1] = dummy * R2 * (A1[0] * crossMat[1] * crossMat[1] * A1[1] * A1[2]).transpose();
	dR1dR1[2][2] = dummy * R2 * (A1[0] * A1[1] * crossMat[2] * crossMat[2] * A1[2]).transpose();
	dR1dR1[0][1] = dummy * R2 * (crossMat[0] * A1[0] * crossMat[1] * A1[1] * A1[2]).transpose(); dR1dR1[1][0] = dR1dR1[0][1];
	dR1dR1[0][2] = dummy * R2 * (crossMat[0] * A1[0] * A1[1] * crossMat[2] * A1[2]).transpose(); dR1dR1[2][0] = dR1dR1[0][2];
	dR1dR1[1][2] = dummy * R2 * (A1[0] * crossMat[1] * A1[1] * crossMat[2] * A1[2]).transpose(); dR1dR1[2][1] = dR1dR1[1][2];

	dR2dR2[0][0] = dummy * crossMat[0] * crossMat[0] * R2 * R1.transpose();
	dR2dR2[1][1] = dummy * A2[0] * crossMat[1] * crossMat[1] * A2[1] * A2[2] * R1.transpose();
	dR2dR2[2][2] = dummy * A2[0] * A2[1] * crossMat[2] * crossMat[2] * A2[2] * R1.transpose();
	dR2dR2[0][1] = dummy * crossMat[0] * A2[0] * crossMat[1] * A2[1] * A2[2] * R1.transpose(); dR2dR2[1][0] = dR2dR2[0][1];
	dR2dR2[0][2] = dummy * crossMat[0] * A2[0] * A2[1] * crossMat[2] * A2[2] * R1.transpose(); dR2dR2[2][0] = dR2dR2[0][2];
	dR2dR2[1][2] = dummy * A2[0] * crossMat[1] * A2[1] * crossMat[2] * A2[2] * R1.transpose(); dR2dR2[2][1] = dR2dR2[1][2];

	dR1dR2[0][0] = dummy * crossMat[0] * R2 * (crossMat[0] * R1).transpose();
	dR1dR2[0][1] = dummy * A2[0] * crossMat[1] * A2[1] * A2[2] * (crossMat[0] * R1).transpose();
	dR1dR2[0][2] = dummy * A2[0] * A2[1] * crossMat[2] * A2[2] * (crossMat[0] * R1).transpose();
	dR1dR2[1][0] = dummy * crossMat[0] * R2 * (A1[0] * crossMat[1] * A1[1] * A1[2]).transpose();
	dR1dR2[1][1] = dummy * A2[0] * crossMat[1] * A2[1] * A2[2] * (A1[0] * crossMat[1] * A1[1] * A1[2]).transpose();
	dR1dR2[1][2] = dummy * A2[0] * A2[1] * crossMat[2] * A2[2] * (A1[0] * crossMat[1] * A1[1] * A1[2]).transpose();
	dR1dR2[2][0] = dummy * crossMat[0] * R2 * (A1[0] * A1[1] * crossMat[2] * A1[2]).transpose();
	dR1dR2[2][1] = dummy * A2[0] * crossMat[1] * A2[1] * A2[2] * (A1[0] * A1[1] * crossMat[2] * A1[2]).transpose();
	dR1dR2[2][2] = dummy * A2[0] * A2[1] * crossMat[2] * A2[2] * (A1[0] * A1[1] * crossMat[2] * A1[2]).transpose();

	// *******************  dR1dR1 ****************************
	Matrix3x3 ddwTensor[3];
	for (int k = 0; k < 3; k++)
	{
		Matrix3x3& ddw = ddwTensor[k];

		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
			{
				Matrix3x3 ddwdR;
				ddwdR(0, 0) = (dw00dR[j].array() * dR1[k].array()).sum();
				ddwdR(1, 1) = ddwdR(0, 0); ddwdR(2, 2) = ddwdR(0, 0);
				ddwdR(0, 1) = (dw01dR[j].array() * dR1[k].array()).sum(); ddwdR(1, 0) = -ddwdR(0, 1);
				ddwdR(0, 2) = (dw02dR[j].array() * dR1[k].array()).sum(); ddwdR(2, 0) = -ddwdR(0, 2);
				ddwdR(1, 2) = (dw12dR[j].array() * dR1[k].array()).sum(); ddwdR(2, 1) = -ddwdR(1, 2);

				ddw(i, j) = (ddwdR.array() * dR1[i].array()).sum() + (dwdR[j].array() * dR1dR1[i][k].array()).sum();
			}
	}

	Matrix3x3 H;
	for (int i = 0; i < 3; i++)
	{
		H.row(i) = scale * ddwTensor[i] * vec + scale * dw1 * scale * dw1.row(i).transpose();
	}

	int startIndex = theMotionPlan->COMOrientationsParamsStartIndex + 3 * index1;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j <= i; j++)
			ADD_HES_ELEMENT(hessianEntries, startIndex + i, startIndex + j, H(i, j), weight);


	// *******************  dR2dR2 ****************************
	for (int k = 0; k < 3; k++)
	{
		Matrix3x3& ddw = ddwTensor[k];

		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
			{
				Matrix3x3 ddwdR;
				ddwdR(0, 0) = (dw00dR[j].array() * dR2[k].array()).sum();
				ddwdR(1, 1) = ddwdR(0, 0); ddwdR(2, 2) = ddwdR(0, 0);
				ddwdR(0, 1) = (dw01dR[j].array() * dR2[k].array()).sum(); ddwdR(1, 0) = -ddwdR(0, 1);
				ddwdR(0, 2) = (dw02dR[j].array() * dR2[k].array()).sum(); ddwdR(2, 0) = -ddwdR(0, 2);
				ddwdR(1, 2) = (dw12dR[j].array() * dR2[k].array()).sum(); ddwdR(2, 1) = -ddwdR(1, 2);

				ddw(i, j) = (ddwdR.array() * dR2[i].array()).sum() + (dwdR[j].array() * dR2dR2[i][k].array()).sum();
			}
	}

	for (int i = 0; i < 3; i++)
	{
		H.row(i) = scale * ddwTensor[i] * vec + scale * dw2 * scale * dw2.row(i).transpose();
	}

	startIndex = theMotionPlan->COMOrientationsParamsStartIndex + 3 * index2;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j <= i; j++)
			ADD_HES_ELEMENT(hessianEntries, startIndex + i, startIndex + j, H(i, j), weight);

	// *******************  dR1dR2 ****************************
	for (int k = 0; k < 3; k++)
	{
		Matrix3x3& ddw = ddwTensor[k];

		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
			{
				Matrix3x3 ddwdR;
				ddwdR(0, 0) = (dw00dR[j].array() * dR2[k].array()).sum();
				ddwdR(1, 1) = ddwdR(0, 0); ddwdR(2, 2) = ddwdR(0, 0);
				ddwdR(0, 1) = (dw01dR[j].array() * dR2[k].array()).sum(); ddwdR(1, 0) = -ddwdR(0, 1);
				ddwdR(0, 2) = (dw02dR[j].array() * dR2[k].array()).sum(); ddwdR(2, 0) = -ddwdR(0, 2);
				ddwdR(1, 2) = (dw12dR[j].array() * dR2[k].array()).sum(); ddwdR(2, 1) = -ddwdR(1, 2);

				ddw(i, j) = (ddwdR.array() * dR1[i].array()).sum() + (dwdR[j].array() * dR1dR2[i][k].array()).sum();
			}
	}

	for (int i = 0; i < 3; i++)
	{
		H.row(i) = scale * ddwTensor[i] * vec + scale * dw1 * scale * dw2.row(i).transpose();
	}

	int startIndexI = theMotionPlan->COMOrientationsParamsStartIndex + 3 * index2;
	int startIndexJ = theMotionPlan->COMOrientationsParamsStartIndex + 3 * index1;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			ADD_HES_ELEMENT(hessianEntries, startIndexI + i, startIndexJ + j, H(i, j), weight);
}


void MPO_TorqueAngularAccelObjective::getdwdRFromRotationMatrix(Matrix3x3* dwdRArray, Matrix3x3* dw00dRArray, Matrix3x3* dw01dRArray, Matrix3x3* dw02dRArray, Matrix3x3* dw12dRArray, Matrix3x3& R)
{
	V3D diff;
	diff[0] = R(2, 1) - R(1, 2);
	diff[1] = R(2, 0) - R(0, 2);
	diff[2] = R(1, 0) - R(0, 1);

	double sumDiag = 0.5 * (R(0, 0) + R(1, 1) + R(2, 2) - 1);
	double angle = acos(sumDiag);
	double diffNorm = diff.norm();
	double diffNorm3 = diffNorm * diffNorm * diffNorm;
	double diffNorm5 = diffNorm3 * diffNorm * diffNorm;
	double tmp = sqrt(1 - sumDiag * sumDiag);


	// ************** X **************
	Matrix3x3& dwdR_x = dwdRArray[0];
	double k = diff[0] / diffNorm;
	double p = -0.5 / tmp;
	dwdR_x(0, 0) = k * p; dwdR_x(1, 1) = dwdR_x(0, 0); dwdR_x(2, 2) = dwdR_x(0, 0);
	dwdR_x(0, 1) = angle * diff[0] * diff[2] / diffNorm3; dwdR_x(1, 0) = -dwdR_x(0, 1);
	dwdR_x(0, 2) = angle * diff[0] * diff[1] / diffNorm3; dwdR_x(2, 0) = -dwdR_x(0, 2);
	dwdR_x(1, 2) = -angle * (diff[2] * diff[2] + diff[1] * diff[1]) / diffNorm3; dwdR_x(2, 1) = -dwdR_x(1, 2);

	Matrix3x3& dw00dR_x = dw00dRArray[0];
	dw00dR_x(0, 0) = k * -0.25 * sumDiag / pow(1 - sumDiag * sumDiag, 1.5); dw00dR_x(1, 1) = dw00dR_x(0, 0); dw00dR_x(2, 2) = dw00dR_x(0, 0);
	dw00dR_x(0, 1) = p * diff[0] * diff[2] / diffNorm3; dw00dR_x(1, 0) = -dw00dR_x(0, 1);
	dw00dR_x(0, 2) = p * diff[0] * diff[1] / diffNorm3; dw00dR_x(2, 0) = -dw00dR_x(0, 2);
	dw00dR_x(1, 2) = -p * (diff[2] * diff[2] + diff[1] * diff[1]) / diffNorm3; dw00dR_x(2, 1) = -dw00dR_x(1, 2);

	Matrix3x3& dw01dR_x = dw01dRArray[0];
	dw01dR_x(0, 0) = dw00dR_x(0, 1); dw01dR_x(1, 1) = dw01dR_x(0, 0); dw01dR_x(2, 2) = dw01dR_x(0, 0);
	dw01dR_x(0, 1) = -angle * diff[0] * (diffNorm * diffNorm - 3 * diff[2] * diff[2]) / diffNorm5; dw01dR_x(1, 0) = -dw01dR_x(0, 1);
	dw01dR_x(0, 2) = angle * 3 * diff[0] * diff[1] * diff[2] / diffNorm5; dw01dR_x(2, 0) = -dw01dR_x(0, 2);
	dw01dR_x(1, 2) = -angle * diff[2] * (diffNorm * diffNorm - 3 * diff[0] * diff[0]) / diffNorm5; dw01dR_x(2, 1) = -dw01dR_x(1, 2);

	Matrix3x3& dw02dR_x = dw02dRArray[0];
	dw02dR_x(0, 0) = dw00dR_x(0, 2); dw02dR_x(1, 1) = dw02dR_x(0, 0); dw02dR_x(2, 2) = dw02dR_x(0, 0);
	dw02dR_x(0, 1) = angle * 3 * diff[0] * diff[1] * diff[2] / diffNorm5; dw02dR_x(1, 0) = -dw02dR_x(0, 1);
	dw02dR_x(0, 2) = -angle * diff[0] * (diffNorm * diffNorm - 3 * diff[1] * diff[1]) / diffNorm5; dw02dR_x(2, 0) = -dw02dR_x(0, 2);
	dw02dR_x(1, 2) = -angle * diff[1] * (diffNorm * diffNorm - 3 * diff[0] * diff[0]) / diffNorm5; dw02dR_x(2, 1) = -dw02dR_x(1, 2);

	Matrix3x3& dw12dR_x = dw12dRArray[0];
	dw12dR_x(0, 0) = dw00dR_x(1, 2); dw12dR_x(1, 1) = dw12dR_x(0, 0); dw12dR_x(2, 2) = dw12dR_x(0, 0);
	dw12dR_x(0, 1) = angle * diff[2] * (2 * diff[0] * diff[0] - diff[1] * diff[1] - diff[2] * diff[2]) / diffNorm5; dw12dR_x(1, 0) = -dw12dR_x(0, 1);
	dw12dR_x(0, 2) = angle * diff[1] * (2 * diff[0] * diff[0] - diff[1] * diff[1] - diff[2] * diff[2]) / diffNorm5; dw12dR_x(2, 0) = -dw12dR_x(0, 2);
	dw12dR_x(1, 2) = -angle * 3 * diff[0] * (diff[1] * diff[1] + diff[2] * diff[2]) / diffNorm5; dw12dR_x(2, 1) = -dw12dR_x(1, 2);


	// ************** Y **************
	Matrix3x3& dwdR_y = dwdRArray[1];
	k = diff[1] / diffNorm;
	dwdR_y(0, 0) = -0.5 * k / tmp; dwdR_y(1, 1) = dwdR_y(0, 0); dwdR_y(2, 2) = dwdR_y(0, 0);
	dwdR_y(0, 1) = angle * diff[1] * diff[2] / diffNorm3; dwdR_y(1, 0) = -dwdR_y(0, 1);
	dwdR_y(0, 2) = -angle * (diff[2] * diff[2] + diff[0] * diff[0]) / diffNorm3; dwdR_y(2, 0) = -dwdR_y(0, 2);
	dwdR_y(1, 2) = angle * diff[1] * diff[0] / diffNorm3; dwdR_y(2, 1) = -dwdR_y(1, 2);

	Matrix3x3& dw00dR_y = dw00dRArray[1];
	dw00dR_y(0, 0) = k * -0.25 * sumDiag / pow(1 - sumDiag * sumDiag, 1.5); dw00dR_y(1, 1) = dw00dR_y(0, 0); dw00dR_y(2, 2) = dw00dR_y(0, 0);
	dw00dR_y(0, 1) = p * diff[1] * diff[2] / diffNorm3; dw00dR_y(1, 0) = -dw00dR_y(0, 1);
	dw00dR_y(0, 2) = -p * (diff[2] * diff[2] + diff[0] * diff[0]) / diffNorm3; dw00dR_y(2, 0) = -dw00dR_y(0, 2);
	dw00dR_y(1, 2) = p * diff[1] * diff[0] / diffNorm3; dw00dR_y(2, 1) = -dw00dR_y(1, 2);

	Matrix3x3& dw01dR_y = dw01dRArray[1];
	dw01dR_y(0, 0) = dw00dR_y(0, 1); dw01dR_y(1, 1) = dw01dR_y(0, 0); dw01dR_y(2, 2) = dw01dR_y(0, 0);
	dw01dR_y(0, 1) = -angle * diff[1] * (diffNorm * diffNorm - 3 * diff[2] * diff[2]) / diffNorm5; dw01dR_y(1, 0) = -dw01dR_y(0, 1);
	dw01dR_y(0, 2) = -angle * diff[2] * (diffNorm * diffNorm - 3 * diff[1] * diff[1]) / diffNorm5; dw01dR_y(2, 0) = -dw01dR_y(0, 2);
	dw01dR_y(1, 2) = angle * 3 * diff[0] * diff[1] * diff[2] / diffNorm5; dw01dR_y(2, 1) = -dw01dR_y(1, 2);

	Matrix3x3& dw02dR_y = dw02dRArray[1];
	dw02dR_y(0, 0) = dw00dR_y(0, 2); dw02dR_y(1, 1) = dw02dR_y(0, 0); dw02dR_y(2, 2) = dw02dR_y(0, 0);
	dw02dR_y(0, 1) = angle * diff[2] * (2 * diff[1] * diff[1] - diff[0] * diff[0] - diff[2] * diff[2]) / diffNorm5; dw02dR_y(1, 0) = -dw02dR_y(0, 1);
	dw02dR_y(0, 2) = -angle * 3 * diff[1] * (diff[0] * diff[0] + diff[2] * diff[2]) / diffNorm5; dw02dR_y(2, 0) = -dw02dR_y(0, 2);
	dw02dR_y(1, 2) = angle * diff[0] * (2 * diff[1] * diff[1] - diff[0] * diff[0] - diff[2] * diff[2]) / diffNorm5; dw02dR_y(2, 1) = -dw02dR_y(1, 2);

	Matrix3x3& dw12dR_y = dw12dRArray[1];
	dw12dR_y = dw02dR_x;

	// ************** Z **************
	Matrix3x3& dwdR_z = dwdRArray[2];
	k = diff[2] / diffNorm;
	dwdR_z(0, 0) = -0.5 * k / tmp; dwdR_z(1, 1) = dwdR_z(0, 0); dwdR_z(2, 2) = dwdR_z(0, 0);
	dwdR_z(0, 1) = -angle * (diff[1] * diff[1] + diff[0] * diff[0]) / diffNorm3; dwdR_z(1, 0) = -dwdR_z(0, 1);
	dwdR_z(0, 2) = angle * diff[2] * diff[1] / diffNorm3; dwdR_z(2, 0) = -dwdR_z(0, 2);
	dwdR_z(1, 2) = angle * diff[2] * diff[0] / diffNorm3; dwdR_z(2, 1) = -dwdR_z(1, 2);

	Matrix3x3& dw00dR_z = dw00dRArray[2];
	dw00dR_z(0, 0) = k * -0.25 * sumDiag / pow(1 - sumDiag * sumDiag, 1.5); dw00dR_z(1, 1) = dw00dR_z(0, 0); dw00dR_z(2, 2) = dw00dR_z(0, 0);
	dw00dR_z(0, 1) = -p * (diff[1] * diff[1] + diff[0] * diff[0]) / diffNorm3; dw00dR_z(1, 0) = -dw00dR_z(0, 1);
	dw00dR_z(0, 2) = p * diff[2] * diff[1] / diffNorm3; dw00dR_z(2, 0) = -dw00dR_z(0, 2);
	dw00dR_z(1, 2) = p * diff[2] * diff[0] / diffNorm3; dw00dR_z(2, 1) = -dw00dR_z(1, 2);

	Matrix3x3& dw01dR_z = dw01dRArray[2];
	dw01dR_z(0, 0) = dw00dR_z(0, 1); dw01dR_z(1, 1) = dw01dR_z(0, 0); dw01dR_z(2, 2) = dw01dR_z(0, 0);
	dw01dR_z(0, 1) = -angle * 3 * diff[2] * (diff[0] * diff[0] + diff[1] * diff[1]) / diffNorm5; dw01dR_z(1, 0) = -dw01dR_z(0, 1);
	dw01dR_z(0, 2) = angle * diff[1] * (2 * diff[2] * diff[2] - diff[0] * diff[0] - diff[1] * diff[1]) / diffNorm5; dw01dR_z(2, 0) = -dw01dR_z(0, 2);
	dw01dR_z(1, 2) = angle * diff[0] * (2 * diff[2] * diff[2] - diff[0] * diff[0] - diff[1] * diff[1]) / diffNorm5; dw01dR_z(2, 1) = -dw01dR_z(1, 2);

	Matrix3x3& dw02dR_z = dw02dRArray[2];
	dw02dR_z = dw01dR_y;

	Matrix3x3& dw12dR_z = dw12dRArray[2];
	dw12dR_z = dw01dR_x;

	dwdR_y *= -1;
	dw00dR_y *= -1;
	dw01dR_y *= -1;
	dw02dR_y *= -1;
	dw12dR_y *= -1;
}

void MPO_TorqueAngularAccelObjective::getdwdRFromRotationMatrix(Matrix3x3* dwdRArray, Matrix3x3& R)
{
	V3D diff;
	diff[0] = R(2, 1) - R(1, 2);
	diff[1] = R(2, 0) - R(0, 2);
	diff[2] = R(1, 0) - R(0, 1);

	double sumDiag = 0.5 * (R(0, 0) + R(1, 1) + R(2, 2) - 1);
	double angle = acos(sumDiag);
	double diffNorm = diff.norm();
	double diffNorm3 = diffNorm * diffNorm * diffNorm;
	double tmp = sqrt(1 - sumDiag * sumDiag);

	// ************** X **************
	Matrix3x3& dwdR_x = dwdRArray[0];
	double k = diff[0] / diffNorm;
	double p = -0.5 / tmp;
	dwdR_x(0, 0) = k * p; dwdR_x(1, 1) = dwdR_x(0, 0); dwdR_x(2, 2) = dwdR_x(0, 0);
	dwdR_x(0, 1) = angle * diff[0] * diff[2] / diffNorm3; dwdR_x(1, 0) = -dwdR_x(0, 1);
	dwdR_x(0, 2) = angle * diff[0] * diff[1] / diffNorm3; dwdR_x(2, 0) = -dwdR_x(0, 2);
	dwdR_x(1, 2) = -angle * (diff[2] * diff[2] + diff[1] * diff[1]) / diffNorm3; dwdR_x(2, 1) = -dwdR_x(1, 2);

	// ************** Y **************
	Matrix3x3& dwdR_y = dwdRArray[1];
	k = diff[1] / diffNorm;
	dwdR_y(0, 0) = -0.5 * k / tmp; dwdR_y(1, 1) = dwdR_y(0, 0); dwdR_y(2, 2) = dwdR_y(0, 0);
	dwdR_y(0, 1) = angle * diff[1] * diff[2] / diffNorm3; dwdR_y(1, 0) = -dwdR_y(0, 1);
	dwdR_y(0, 2) = -angle * (diff[2] * diff[2] + diff[0] * diff[0]) / diffNorm3; dwdR_y(2, 0) = -dwdR_y(0, 2);
	dwdR_y(1, 2) = angle * diff[1] * diff[0] / diffNorm3; dwdR_y(2, 1) = -dwdR_y(1, 2);

	// ************** Z **************
	Matrix3x3& dwdR_z = dwdRArray[2];
	k = diff[2] / diffNorm;
	dwdR_z(0, 0) = -0.5 * k / tmp; dwdR_z(1, 1) = dwdR_z(0, 0); dwdR_z(2, 2) = dwdR_z(0, 0);
	dwdR_z(0, 1) = -angle * (diff[1] * diff[1] + diff[0] * diff[0]) / diffNorm3; dwdR_z(1, 0) = -dwdR_z(0, 1);
	dwdR_z(0, 2) = angle * diff[2] * diff[1] / diffNorm3; dwdR_z(2, 0) = -dwdR_z(0, 2);
	dwdR_z(1, 2) = angle * diff[2] * diff[0] / diffNorm3; dwdR_z(2, 1) = -dwdR_z(1, 2);

	dwdR_y *= -1;
}

void MPO_TorqueAngularAccelObjective::getCrossProductTensor(Matrix3x3* tensor)
{
	tensor[0].setZero();
	tensor[0](1, 2) = 1;
	tensor[0](2, 1) = -1;
	tensor[1].setZero();
	tensor[1](0, 2) = -1;
	tensor[1](2, 0) = 1;
	tensor[2].setZero();
	tensor[2](0, 1) = 1;
	tensor[2](1, 0) = -1;
}

bool MPO_TorqueAngularAccelObjective::checkAngleAxisSingularity(Matrix3x3& R)
{
	const double thresold = 1e-5;
	return fabs(R(1, 0) - R(0, 1)) < thresold && fabs(R(2, 0) - R(0, 2)) < thresold && fabs(R(2, 1) - R(1, 2)) < thresold;
}


MPO_TorqueAngularAccelObjective::~MPO_TorqueAngularAccelObjective()
{
}
