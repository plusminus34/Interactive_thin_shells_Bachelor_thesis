#include <RobotDesignerLib/ControlUtil.h>

// tracking a motion plan effectively requires us to consider a translation and heading-invariant approach. 
// The question is, what is the transformation that best aligns the motion plan to the current configuration of the robot?
Quaternion getHeadingOffsetFromMotionPlanToRobotState(LocomotionEngineMotionPlan* mp, double mpPhase, Robot* robot) {
	/*
	int nEEs = mp->endEffectorTrajectories.size();
	if (nEEs < 2)
	return Quaternion();
	//we want to first compute the linear transformation that maps body-EE offsets from the motion plan (X) to the robot (x): x = A X
	MatrixNxM x, X;
	resize(x, 2, nEEs);
	resize(X, 2, nEEs);

	P3D robotPos;
	P3D motionPlanPos;

	for (int i = 0; i < nEEs; i++) {
	robotPos += mp->endEffectorTrajectories[i].endEffectorRB->getWorldCoordinates(mp->endEffectorTrajectories[i].endEffectorLocalCoords);
	motionPlanPos += mp->endEffectorTrajectories[i].getEEPositionAt(mpPhase);
	}
	robotPos /= nEEs;
	motionPlanPos /= nEEs;

	for (int i = 0; i < nEEs; i++) {
	P3D robotEE = mp->endEffectorTrajectories[i].endEffectorRB->getWorldCoordinates(mp->endEffectorTrajectories[i].endEffectorLocalCoords);
	P3D motionPlanEE = mp->endEffectorTrajectories[i].getEEPositionAt(mpPhase);

	V3D rEEOffset(robotPos, robotEE);
	V3D mpEEOffset(motionPlanPos, motionPlanEE);
	x.coeffRef(0, i) = rEEOffset.x();
	x.coeffRef(1, i) = rEEOffset.z();
	X.coeffRef(0, i) = mpEEOffset.x();
	X.coeffRef(1, i) = mpEEOffset.z();
	}

	//the matrix A (where x = A X) can now be computed using the pseudoinverse of X: A = x * X' (X * X') ^-1
	Matrix2x2 A = x * X.transpose() * (X * X.transpose()).inverse();

	//now we need to decompose A into a part that gives us a pure rotation, and one that gives us a deformation
	Eigen::JacobiSVD<Matrix2x2> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

	//A = U S V' = U V' V S V' = R D, where R is a rotation, U is the deformation/stretch part, which is a positive semidefinite hermetian . So, R = UV'
	Matrix2x2 R = svd.matrixU() * svd.matrixV().transpose();

	//and now, we must obtain the rotation angle represented by the above matrix
	double headingAngle = atan2(R.coeff(1, 0), R.coeff(0,0));

	Logger::consolePrint("Det(R): %lf, heading angle: %lf\n", R.determinant(), headingAngle);

	//and that's it...
	return getRotationQuaternion(headingAngle, Globals::worldUp);
	*/
	RobotState rs(robot);
	RobotState moptrs(robot);
	mp->robotStateTrajectory.getRobotPoseAt(mpPhase, moptrs);
	double headingAngle1 = rs.getHeading();
	double headingAngle2 = moptrs.getHeading();
	return getRotationQuaternion(headingAngle2 - headingAngle1, Globals::worldUp);
}