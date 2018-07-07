#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>
#include <ControlLib/BodyFrame.h>
#include <ControlLib/GenericLimb.h>
#include <Utils/Utils.h>
#include <MathLib/MathLib.h>
#include <GUILib/GLUtils.h>
#include <MathLib/ConvexHull3D.h>
#include <set>


LocomotionEngine_EndEffectorTrajectory::LocomotionEngine_EndEffectorTrajectory(int nPos){
	endEffectorRB = NULL;
	theLimb = NULL;
	CPIndex = -1;
	initialize(nPos);
}

LocomotionEngine_EndEffectorTrajectory::LocomotionEngine_EndEffectorTrajectory(){
	endEffectorRB = NULL;
	theLimb = NULL;
	CPIndex = -1;
}

void LocomotionEngine_EndEffectorTrajectory::initialize(int nPos){
	contactForce.resize(nPos);
	EEPos.resize(nPos);
	wheelSpeed.resize(nPos);
	wheelYawAngle.resize(nPos);
	wheelTiltAngle.resize(nPos, 0/*M_PI*0.25*/);
	targetEEPosY.resize(nPos);

	contactFlag.resize(nPos, 0);
	EEWeights.resize(nPos, 0.05);
}

V3D LocomotionEngine_EndEffectorTrajectory::getContactForceAt(double t) {
	int tIndex = (int)(t * (double)(contactFlag.size() - 1));
	if (contactFlag[tIndex] == 0)
		return V3D(0, 0, 0);

	//very slow method, but easy to implement...
	Trajectory3D traj;
	for (uint i = 0; i<contactForce.size(); i++)
		traj.addKnot((double)i / (contactForce.size() - 1), contactForce[i]);
	return traj.evaluate_linear(t);
//	boundToRange(&t, 0, 1);
//	int tIndex = (int)(t * (double)(contactFlag.size() - 1));
//	return contactForce[tIndex];
}

P3D LocomotionEngine_EndEffectorTrajectory::getEEPositionAt(double t) const {
	//very slow method, but easy to implement...
	Trajectory3D traj;
	for (uint i = 0;i<EEPos.size();i++)
		traj.addKnot((double)i / (EEPos.size() - 1), EEPos[i]);
	return P3D() + traj.evaluate_linear(t);
}

V3D LocomotionEngine_EndEffectorTrajectory::getWheelRhoLocal() const
{
	Vector3d rhoLocal = wheelTiltAxis.cross(wheelAxisLocal).normalized() * wheelRadius;
	return rhoLocal;
}

P3D LocomotionEngine_EndEffectorTrajectory::getWheelCenterPositionAt(double t) const {
	V3D rho = getWheelRhoLocal();
	double yawAngle = getWheelYawAngleAt(t);
	double tiltAngle = getWheelTiltAngleAt(t);
	rho = rotVecByYawTilt(rho, wheelYawAxis, yawAngle, wheelTiltAxis, tiltAngle);
	return getEEPositionAt(t) + rho;
}

double LocomotionEngine_EndEffectorTrajectory::getWheelYawAngleAt(double t) const {
	//very slow method, but easy to implement...
	Trajectory1D traj;
	for (uint i = 0; i<wheelYawAngle.size(); i++)
		traj.addKnot((double)i / (wheelYawAngle.size() - 1), wheelYawAngle[i]);
	return traj.evaluate_linear(t);
}

double LocomotionEngine_EndEffectorTrajectory::getWheelTiltAngleAt(double t) const {
	//very slow method, but easy to implement...
	Trajectory1D traj;
	for (uint i = 0; i<wheelTiltAngle.size(); i++)
		traj.addKnot((double)i / (wheelTiltAngle.size() - 1), wheelTiltAngle[i]);
	return traj.evaluate_linear(t);
}

double LocomotionEngine_EndEffectorTrajectory::getWheelSpeedAt(double t) const
{
	//very slow method, but easy to implement...
	Trajectory1D traj;
	for (uint i = 0; i<wheelSpeed.size(); i++)
		traj.addKnot((double)i / (wheelSpeed.size() - 1), wheelSpeed[i]);
	return traj.evaluate_linear(t);
}

Vector3d LocomotionEngine_EndEffectorTrajectory::drotVecByYawTilt_dTilt(const Vector3d &axis, const Vector3d &axisYaw, double alpha, const Vector3d &axisTilt, double beta)
{
	// First tilt the axis ...
	Vector3d axisRot = drotateVec_dalpha(axis, beta, axisTilt);
	// ... and then yaw
	axisRot = rotateVec(axisRot, alpha, axisYaw);
	return axisRot;
}

Vector3d LocomotionEngine_EndEffectorTrajectory::drotVecByYawTilt_dYaw(const Vector3d &axis, const Vector3d &axisYaw, double alpha, const Vector3d &axisTilt, double beta)
{
	// First tilt the axis ...
	Vector3d axisRot = rotateVec(axis, beta, axisTilt);
	// ... and then yaw
	axisRot = drotateVec_dalpha(axisRot, alpha, axisYaw);
	return axisRot;
}

Vector3d LocomotionEngine_EndEffectorTrajectory::ddrotVecByYawTilt_dYaw2(const Vector3d &axis, const Vector3d &axisYaw, double alpha, const Vector3d &axisTilt, double beta)
{
	// First tilt the axis ...
	Vector3d axisRot = rotateVec(axis, beta, axisTilt);
	// ... and then yaw
	axisRot = ddrotateVec_dalpha2(axisRot, alpha, axisYaw);
	return axisRot;
}

Vector3d LocomotionEngine_EndEffectorTrajectory::ddrotVecByYawTilt_dYawdTilt(const Vector3d &axis, const Vector3d &axisYaw, double alpha, const Vector3d &axisTilt, double beta)
{
	// First tilt the axis ...
	Vector3d axisRot = drotateVec_dalpha(axis, beta, axisTilt);
	// ... and then yaw
	axisRot = drotateVec_dalpha(axisRot, alpha, axisYaw);
	return axisRot;
}

Vector3d LocomotionEngine_EndEffectorTrajectory::ddrotVecByYawTilt_dTilt2(const Vector3d &axis, const Vector3d &axisYaw, double alpha, const Vector3d &axisTilt, double beta)
{
	// First tilt the axis ...
	Vector3d axisRot = ddrotateVec_dalpha2(axis, beta, axisTilt);
	// ... and then yaw
	axisRot = rotateVec(axisRot, alpha, axisYaw);
	return axisRot;
}

double LocomotionEngine_EndEffectorTrajectory::getContactFlagAt(double t){
	boundToRange(&t, 0, 1);
	int tIndex = (int)(t * (double)(contactFlag.size()-1));
	return contactFlag[tIndex];
}

bool LocomotionEngine_EndEffectorTrajectory::isInStance(double t) {
	return getContactFlagAt(t) > 0.5;
}

bool LocomotionEngine_EndEffectorTrajectory::isInSwing(double t) {
	return !isInStance(t);
}

double LocomotionEngine_EndEffectorTrajectory::getEEWeightAt(double t){
	boundToRange(&t, 0, 1);
	int tIndex = (int)floor(t * (EEWeights.size()));
	if (tIndex >= (int)EEWeights.size()) tIndex = (int)EEWeights.size()-1;
	return EEWeights[tIndex];
}

LocomotionEngine_COMTrajectory::LocomotionEngine_COMTrajectory(){

}

void LocomotionEngine_COMTrajectory::initialize(int nPoints, const P3D &desComPos, const V3D& comRotationAngles, const V3D &axis_0, const V3D &axis_1, const V3D &axis_2){
	this->nPoints = nPoints;
	pos[0] = dVector(nPoints); pos[1] = dVector(nPoints); pos[2] = dVector(nPoints);
	orientation[0] = dVector(nPoints); orientation[1] = dVector(nPoints); orientation[2] = dVector(nPoints);

	desiredPos[0] = dVector(nPoints); desiredPos[1] = dVector(nPoints); desiredPos[2] = dVector(nPoints);
	desiredOrientation[0] = dVector(nPoints); desiredOrientation[1] = dVector(nPoints); desiredOrientation[2] = dVector(nPoints);

	axis[0] = axis_0; axis[1] = axis_1; axis[2] = axis_2;

	//note - fixed parameterization!
	for (int i=0;i<nPoints;i++)
		for (int j = 0; j < 3; j++) {
			pos[j][i] = desiredPos[j][i] = desComPos[j];
			orientation[j][i] = desiredOrientation[j][i] = comRotationAngles[j];
		}
}

V3D LocomotionEngine_COMTrajectory::getAxis(int i) {
	return axis[i];
}

P3D LocomotionEngine_COMTrajectory::getCOMPositionAt(double p){
	boundToRange(&p, 0, 1);
	double tSize = p * (double)(pos[0].size()-1);

	int tLow = (int)floor(tSize);
	int tHigh = (int)ceil(tSize);

	double kLow = 1.0 - (tSize-tLow);
	double kHigh = 1.0 - kLow;

	double interpolatedX = pos[0][tLow]*kLow+pos[0][tHigh]*kHigh;
	double interpolatedY = pos[1][tLow]*kLow+pos[1][tHigh]*kHigh;
	double interpolatedZ = pos[2][tLow]*kLow+pos[2][tHigh]*kHigh;

	P3D interpolatedPoint = P3D(interpolatedX,interpolatedY,interpolatedZ);

	return interpolatedPoint;
}

V3D LocomotionEngine_COMTrajectory::getCOMVelocityAt(double p, double strideDuration) {
	double dp = 0.010417;
	double dt = dp * strideDuration;

//	P3D pNow = getCOMPositionAt(p);
//	P3D pNext = getCOMPositionAt(p + dp);

	return (getCOMPositionAt(p + dp) - getCOMPositionAt(p)) / dt;
}

V3D LocomotionEngine_COMTrajectory::getCOMAngularVelocityAt(double p, double strideDuration) {
	double dp = 0.010417;
	double dt = dp * strideDuration;

	return estimateAngularVelocity(getCOMOrientationAt(p), getCOMOrientationAt(p + dp), dt);
}

P3D LocomotionEngine_COMTrajectory::getCOMEulerAnglesAt(double p) {
	boundToRange(&p, 0, 1);
	double tSize = p * (double)(orientation[0].size() - 1);

	int tLow = (int)floor(tSize);
	int tHigh = (int)ceil(tSize);

	double kLow = 1.0 - (tSize - tLow);
	double kHigh = 1.0 - kLow;

	double interpolatedX = orientation[0][tLow] * kLow + orientation[0][tHigh] * kHigh;
	double interpolatedY = orientation[1][tLow] * kLow + orientation[1][tHigh] * kHigh;
	double interpolatedZ = orientation[2][tLow] * kLow + orientation[2][tHigh] * kHigh;

	P3D interpolatedPoint = P3D(interpolatedX, interpolatedY, interpolatedZ);

	return interpolatedPoint;
}

Quaternion LocomotionEngine_COMTrajectory::getCOMOrientationAt(double p) {
	P3D eulerAngles = getCOMEulerAnglesAt(p);
	return getRotationQuaternion(eulerAngles[0], axis[0]) *
			getRotationQuaternion(eulerAngles[1], axis[1]) * getRotationQuaternion(eulerAngles[2], axis[2]);
}

P3D LocomotionEngine_COMTrajectory::getTargetCOMPositionAtTimeIndex(int j) {
	if (j < 0)
		return P3D();
	return P3D(desiredPos[0][j], desiredPos[1][j], desiredPos[2][j]);
}


P3D LocomotionEngine_COMTrajectory::getCOMPositionAtTimeIndex(int j){
	if (j < 0)
		return P3D();
	return P3D(pos[0][j], pos[1][j], pos[2][j]);
}

P3D LocomotionEngine_COMTrajectory::getCOMEulerAnglesAtTimeIndex(int j) {
	if (j < 0)
		return P3D();
	return P3D(orientation[0][j], orientation[1][j], orientation[2][j]);
}

P3D LocomotionEngine_COMTrajectory::getTargetCOMEulerAnglesAtTimeIndex(int j) {
	if (j < 0)
		return P3D();
	return P3D(desiredOrientation[0][j], desiredOrientation[1][j], desiredOrientation[2][j]);
}


Quaternion LocomotionEngine_COMTrajectory::getCOMOrientationAtTimeIndex(int j) {
	P3D eulerAngles = getCOMEulerAnglesAtTimeIndex(j);
	return getRotationQuaternion(eulerAngles[0], axis[0]) *
			getRotationQuaternion(eulerAngles[1], axis[1]) * getRotationQuaternion(eulerAngles[2], axis[2]);
}

LocomotionEngine_RobotStateTrajectory::LocomotionEngine_RobotStateTrajectory(GeneralizedCoordinatesRobotRepresentation *robotRepresentation, int nPos){
	this->robotRepresentation = robotRepresentation;
	initialize(nPos);
}

LocomotionEngine_RobotStateTrajectory::LocomotionEngine_RobotStateTrajectory(){
	this->robotRepresentation = NULL;
}

void LocomotionEngine_RobotStateTrajectory::initialize(int nPoints){
	nStateDim = 0;
	//we will keep angles separated by translational DOFs
	if (robotRepresentation == NULL) return;
	nStateDim = robotRepresentation->getDimensionCount();
	dVector q;
	robotRepresentation->getQ(q);
	qArray.resize(nPoints);
	defaultRobotStates.resize(nPoints);
	for (int j=0;j<nPoints;j++){
		qArray[j] = q;
		defaultRobotStates[j] = q;
	}
}

P3D LocomotionEngine_RobotStateTrajectory::getBodyPositionAt(double t){
	if (robotRepresentation == NULL) return P3D();
	if (t > 1) t -= 1; 	if (t < 0) t += 1;
	int tIndex = (int)floor(t * (qArray.size()));
	if (tIndex >= (int)qArray.size()) tIndex = (int)qArray.size()-1;
	return P3D(qArray[tIndex][0], qArray[tIndex][1], qArray[tIndex][2]);
}

void LocomotionEngine_RobotStateTrajectory::getQ(double t, dVector &q_t) {
	if (robotRepresentation == NULL) return;
	q_t = dVector(qArray[0].size());
	for (int i=0;i<qArray[0].size();i++){
		Trajectory1D traj;
		for (uint j=0;j<qArray.size();j++)
			traj.addKnot((double)j/(qArray.size()-1), qArray[j][i]);
		q_t[i] = traj.evaluate_linear(t);
		//q_t.push_back(traj.evaluate_catmull_rom(t));
	}
}

void LocomotionEngine_RobotStateTrajectory::getRobotPoseAt(double t, RobotState &robotPose){
	if (robotRepresentation == NULL) return;
	dVector q; getQ(t, q);
	robotRepresentation->setQ(q);
	robotRepresentation->getReducedRobotState(robotPose);
}

void LocomotionEngine_RobotStateTrajectory::getRobotStateAt(double t, double motionPlanDuration, RobotState &robotState) {
	if (t > 1) t -= 1.0;
	double dStridePhase = 0.01;
	double dt = dStridePhase * motionPlanDuration;

	RobotState futureRobotState(robotState.getJointCount());

	if (t+dStridePhase < 1){
		getRobotPoseAt(t, robotState);
		getRobotPoseAt(t + dStridePhase, futureRobotState);
	}
	else {
		getRobotPoseAt(t - dStridePhase, robotState);
		getRobotPoseAt(t, futureRobotState);
	}

	robotState.setAngularVelocity(estimateAngularVelocity(robotState.getOrientation(), futureRobotState.getOrientation(), dt));
	robotState.setVelocity((futureRobotState.getPosition() - robotState.getPosition()) / dt);
	for (int i = 0; i < robotState.getJointCount(); i++){
		robotState.setJointRelativeAngVelocity(estimateAngularVelocity(robotState.getJointRelativeOrientation(i), futureRobotState.getJointRelativeOrientation(i), dt), i);
		if (t + dStridePhase >= 1)
			robotState.setJointRelativeOrientation(futureRobotState.getJointRelativeOrientation(i), i);
	}
}

void LocomotionEngine_RobotStateTrajectory::writeRobotMotionTrajectoriesToFile(const char *fName, Robot* robot, double mpDuration) {
	FILE* fp = fopen(fName, "w");

	fprintf(fp, "# every line describes the angle trajectory for a specific joint. Each trajectory is described by n points equally distributed over a planning horizon with length t. These two parameters are provided below.\n");

	fprintf(fp, "%d %lf\n\n", qArray.size(), mpDuration);
	//every line in the file will correspond to the angle trajectory for one of the joints
	for (int j = 0; j < robot->getJointCount(); j++) {
		fprintf(fp, "%s:\t", robot->getJoint(j)->name.c_str());
		int nSamples = (int)qArray.size();
		for (int i = 0; i<nSamples; i++)
			fprintf(fp, "%lf\t", qArray[i][robotRepresentation->getQIndexForJoint(robot->getJoint(j))]);
		fprintf(fp, "\n");
	}
	fclose(fp);
}

void LocomotionEngine_RobotStateTrajectory::getQAtTimeIndex(int j, dVector &q_t){
	q_t = qArray[j];
}

P3D LocomotionEngine_RobotStateTrajectory::getBodyPositionAtTimeIndex(int j){
	return P3D(qArray[j][0], qArray[j][1], qArray[j][2]);
}

LocomotionEngineMotionPlan::LocomotionEngineMotionPlan(Robot* robot, int nSamplingPoints){
	wrapAroundBoundaryIndex = -1;
	optimizeCOMPositions = true;
	optimizeEndEffectorPositions = false;
	optimizeWheels = false;
	optimizeBarycentricWeights = true;
	optimizeRobotStates = true;
	optimizeContactForces = true;
	enforceGRFConstraints = false;
	desTurningAngle = 0;

	this->robot = robot;
	this->nSamplePoints = nSamplingPoints;

	/**
		we must set up the robot such that its feet are all on the ground...
	*/
	IK_Solver ikSolver(robot);
	ikSolver.ikPlan->setTargetIKStateFromRobot();

	int nLegs = this->robot->bFrame->limbs.size();
	for (int i = 0; i<nLegs; i++)
		addIKInitEE(this->robot->bFrame->limbs[i]->getLastLimbSegment(), ikSolver.ikPlan);
	addIKInitEE(robot->root, ikSolver.ikPlan);

	ikSolver.ikEnergyFunction->setupSubObjectives_EEMatch();
	ikSolver.ikEnergyFunction->regularizer = 10;
//	ikSolver.ikOptimizer->checkDerivatives = true;
//	ikSolver.ikEnergyFunction->printDebugInfo = true;
	ikSolver.solve(20);

	//first off, compute the current position of the COM, and ensure the whole robot lies on the ground
	totalMass = 0;

	//this is the position of the "COM"
	for (uint k = 0; k<robot->bFrame->bodyLinks.size(); k++) {
		totalMass += robot->bFrame->bodyLinks[k]->rbProperties.mass;
		defaultCOMPosition += robot->bFrame->bodyLinks[k]->getCMPosition() * robot->bFrame->bodyLinks[k]->rbProperties.mass;
	}
	defaultCOMPosition /= totalMass;

	desCOMHeight = defaultCOMPosition[1];


	//approximate the inertia of the body
	double Ixx = robot->root->rbProperties.MOI_local.coeff(0, 0); // = m(y2 + z2)/12
	double Iyy = robot->root->rbProperties.MOI_local.coeff(1, 1); // = m(z2 + x2)/12
	double Izz = robot->root->rbProperties.MOI_local.coeff(2, 2); // = m(y2 + x2)/12
	double x = sqrt((Iyy + Izz - Ixx) * 6 / robot->root->rbProperties.mass);
	double y = sqrt((Izz + Ixx - Iyy) * 6 / robot->root->rbProperties.mass);
	double z = sqrt((Ixx + Iyy - Izz) * 6 / robot->root->rbProperties.mass);
	double bodyVolume = x * y * z;
	double approxDim = pow(bodyVolume, 1 / 3.0);
	totalInertia = totalMass * (approxDim*approxDim + approxDim*approxDim) / 12;


/*
	//and average position of the feet. We want to make sure they end up touching the ground
	double eeY = 0;
	int totalNEEs = 0;

	for (int i = 0; i<nLegs; i++) {
		int nEEs = this->robot->bFrame->limbs[i]->getLastLimbSegment()->rbProperties.getEndEffectorPointCount();
		for (int j = 0; j<nEEs; j++) {
			P3D eeLocalCoords = this->robot->bFrame->limbs[i]->getLastLimbSegment()->rbProperties.getEndEffectorPoint(j);
			P3D eeWorldCoords = this->robot->bFrame->limbs[i]->getLastLimbSegment()->getWorldCoordinates(eeLocalCoords);
			eeY += eeWorldCoords[1];
			totalNEEs++;
		}
	}
	if (totalNEEs > 0) eeY /= totalNEEs;
	//move the whole robot so that the feet are on the ground
	RobotState rs(robot);
	P3D rootPos = rs.getPosition();
	rootPos.addOffsetToComponentAlong(V3D(0, 1, 0), -eeY);
	rs.setPosition(rootPos);
	robot->setState(&rs);
*/

	//and now proceed to initialize everything else...
	this->robotRepresentation = new GeneralizedCoordinatesRobotRepresentation(robot);
	robotRepresentation->getQ(initialRobotState);
	initialRS = RobotState(robot);

	//create the end effector trajectories here based on the robot configuration...
	for (int i=0;i<nLegs;i++){
		int nEEs = this->robot->bFrame->limbs[i]->getLastLimbSegment()->rbProperties.getEndEffectorPointCount();
		for (int j=0; j<nEEs; j++){
			addEndEffector(this->robot->bFrame->limbs[i], this->robot->bFrame->limbs[i]->getLastLimbSegment(), j, nSamplingPoints);
		}
	}

	//the root may also have end effectros (e.g. wheels attached directly to the main body), so we should go through those as well...
	int nEEs = this->robot->root->rbProperties.getEndEffectorPointCount();
	for (int j = 0; j<nEEs; j++)
		addEndEffector(NULL, robot->root, j, nSamplingPoints);

	robotStateTrajectory.robotRepresentation = this->robotRepresentation;
	robotStateTrajectory.initialize(nSamplingPoints);

	V3D comRotationAngles(initialRobotState[3], initialRobotState[4], initialRobotState[5]);

	bodyTrajectory.initialize(nSamplePoints, defaultCOMPosition, comRotationAngles, robotRepresentation->getQAxis(3),
		robotRepresentation->getQAxis(4), robotRepresentation->getQAxis(5));

	verticalGRFLowerBoundVal = fabs(totalMass * Globals::g / endEffectorTrajectories.size() / 5.0);
	GRFEpsilon = verticalGRFLowerBoundVal;
	for (uint i = 0; i < endEffectorTrajectories.size(); i++)
		for (uint j = 0; j < endEffectorTrajectories[i].contactForce.size(); j++)
			endEffectorTrajectories[i].contactForce[j].y() = verticalGRFLowerBoundVal + GRFEpsilon;
}

void LocomotionEngineMotionPlan::updateEEs() {
	for (uint i = 0; i < endEffectorTrajectories.size(); i++) {
		endEffectorTrajectories[i].endEffectorLocalCoords = endEffectorTrajectories[i].endEffectorRB->rbProperties.endEffectorPoints[endEffectorTrajectories[i].CPIndex].coords;
	}
}

void LocomotionEngineMotionPlan::addIKInitEE(RigidBody* rb, IK_Plan* ikPlan) {
	int nEEs = rb->rbProperties.getEndEffectorPointCount();
	for (int j = 0; j < nEEs; j++) {
		P3D eeLocalCoords = rb->rbProperties.getEndEffectorPoint(j);
		P3D eeWorldCoords = rb->getWorldCoordinates(eeLocalCoords);
		eeWorldCoords[1] = 0;

		if (rb->rbProperties.endEffectorPoints[j].isWheel()){
			RBEndEffector worldEE = rb->rbProperties.endEffectorPoints[j];		//holds all quantities of the wheel in world coordinates, placed according to its parent RB
			worldEE.localCoordsWheelAxis = rb->getWorldCoordinates(rb->rbProperties.endEffectorPoints[j].getWheelAxis());
			Vector3d rho = worldEE.getWheelRho();

//			Vector3d rho = rb->rbProperties.endEffectorPoints[j].getWheelRho();
			eeWorldCoords[1] += rho[1];
		}

		ikPlan->endEffectors.push_back(IK_EndEffector());
		ikPlan->endEffectors.back().endEffectorLocalCoords = eeLocalCoords;
		ikPlan->endEffectors.back().endEffectorRB = rb;
		ikPlan->endEffectors.back().targetEEPos = eeWorldCoords;
		ikPlan->endEffectors.back().positionMask = V3D(0, 1, 0);
	}
}

void LocomotionEngineMotionPlan::addEndEffector(GenericLimb* theLimb, RigidBody* rb, int eeIndex, int nSamplingPoints) {
	P3D eeLocalCoords = rb->rbProperties.getEndEffectorPoint(eeIndex);
	P3D eeWorldCoords = rb->getWorldCoordinates(eeLocalCoords);

	endEffectorTrajectories.push_back(LocomotionEngine_EndEffectorTrajectory(nSamplingPoints));
	int index = endEffectorTrajectories.size() - 1;
	LocomotionEngine_EndEffectorTrajectory &eeTraj = endEffectorTrajectories[index];

	eeTraj.CPIndex = eeIndex;

	eeTraj.theLimb = theLimb;
	eeTraj.endEffectorRB = rb;
	eeTraj.endEffectorLocalCoords = eeLocalCoords;

	// is end effector a wheel?
	const RBProperties &rbProperties = rb->rbProperties;
//	const RBEndEffector &rbEndEffector = rbProperties.endEffectorPoints[eeIndex];

	if (rbProperties.endEffectorPoints[eeIndex].isWheel())
	{
		RBEndEffector worldEE = rbProperties.endEffectorPoints[eeIndex];		//holds all quantities of the wheel in world coordinates, placed according to its parent RB
		worldEE.localCoordsWheelAxis = rb->getWorldCoordinates(rbProperties.endEffectorPoints[eeIndex].getWheelAxis());
		wheelToEEIndex[nWheels] = index;
		eeToWheelIndex[index] = nWheels;
		nWheels++;

		eeTraj.isWheel = true;
		eeTraj.isFixedWheel = false;
		eeTraj.isPassiveWheel = worldEE.isFreeToMoveWheel();
		eeTraj.isWeldedWheel = worldEE.isWeldedWheel();

		// set wheel radius from rb properties
		eeTraj.wheelRadius = worldEE.featureSize;

		// set wheel axis
		eeTraj.wheelAxisLocal = worldEE.getWheelAxis();

		// set yaw axis (always going to be y-axis)
		eeTraj.wheelYawAxis = worldEE.getWheelYawAxis();

		// set tilt axis
		eeTraj.wheelTiltAxis = worldEE.getWheelTiltAxis();

		// this is the wheel axis from the robot point of view
		V3D axisRobot = rb->getWorldCoordinates(rbProperties.endEffectorPoints[eeTraj.CPIndex = eeIndex].getWheelAxis());

		// first adjust yaw angle to match wheel axis from robot (order is always tilt then yaw):
		// wheel axis in tilt plane
		V3D axisWheelTilt = eeTraj.wheelAxisLocal - eeTraj.wheelAxisLocal.getProjectionOn(eeTraj.wheelTiltAxis);
		// robot wheel axis in tilt plane
		V3D axisRBTilt = axisRobot - axisRobot.getProjectionOn(eeTraj.wheelTiltAxis);
		// and now compute the angle
		double tiltAngle = axisWheelTilt.angleWith(axisRBTilt, eeTraj.wheelTiltAxis);

		// same for yaw angle, but with wheel axis rotated by tilt angle
		V3D axisWheelRot = rotateVec(eeTraj.wheelAxisLocal, tiltAngle, eeTraj.wheelTiltAxis);
		V3D axisWheelYaw = axisWheelRot - axisWheelRot.getProjectionOn(eeTraj.wheelYawAxis);
		V3D axisRBYaw = axisRobot - axisRobot.getProjectionOn(eeTraj.wheelYawAxis);
		double yawAngle = axisWheelYaw.angleWith(axisRBYaw, eeTraj.wheelYawAxis);

		// verify if it worked:
//		V3D axisVer = eeTraj.getRotatedWheelAxis(yawAngle, tiltAngle);
//		std::cout << "yawAngle  = " << yawAngle << std::endl;
//		std::cout << "tiltAngle = " << tiltAngle << std::endl;
//		std::cout << "axisWheel = " << eeTraj.wheelAxisLocal.transpose() << std::endl;
//		std::cout << "axisRobot = " << axisRobot.transpose() << std::endl;
//		std::cout << "axisVer   = " << axisVer.transpose() << std::endl;
//		std::cout << "err   = " << (axisVer - axisRobot).transpose() << std::endl;

		// set wheel angles
		eeTraj.wheelYawAngle = DynamicArray<double>(nSamplingPoints, yawAngle);
		eeTraj.wheelTiltAngle = DynamicArray<double>(nSamplingPoints, tiltAngle);

		eeWorldCoords -= LocomotionEngine_EndEffectorTrajectory::rotVecByYawTilt(eeTraj.getWheelRhoLocal(), eeTraj.wheelYawAxis, yawAngle, eeTraj.wheelTiltAxis, tiltAngle);

//		std::cout << "eeWorldCoors = " << eeWorldCoords.transpose() << std::endl;

		// output some info
//		Logger::consolePrint("Wheel radius %d: %f", index, eeTraj.wheelRadius);
	}
	else // foot
	{
		eeWorldCoords[1] = 0;
	}

	for (int k = 0; k<nSamplingPoints; k++) {
		eeTraj.EEPos[k] = eeWorldCoords;
		eeTraj.contactFlag[k] = 1.0;
	}

	eeTraj.rootToEEOriginalOffset_local = robot->getRoot()->getLocalCoordinates(V3D(robot->getRoot()->getCMPosition(), eeWorldCoords));
}

//syncs the footfall pattern with the current motion plan
void LocomotionEngineMotionPlan::syncFootFallPatternWithMotionPlan(FootFallPattern& ffp) {
	int nPoints = nSamplePoints - 1;
	//if this is a periodic motion, then we know we know the last data point is the exact same as the first...
//	if (wrapAroundBoundaryIndex != -1)
//		nPoints -= 1;

	ffp = FootFallPattern();
	ffp.strideSamplePoints = nPoints;
	for (uint i = 0; i < robot->bFrame->limbs.size(); i++) {
		//find an end effector that belongs to this limb...
		int eeIndex = 0;
		for (uint j = 0; j < endEffectorTrajectories.size(); j++)
			if (endEffectorTrajectories[j].theLimb == robot->bFrame->limbs[i])
				eeIndex = j;
		//now we have a few different cases to handle... the limb can be always in swing, always in stance, or a mix...
		int firstSwingIndex = -1;
		bool alwaysInSwing = true; 
		bool alwaysInStance = true;
		for (int j = 0; j < nPoints; j++) {
			if (endEffectorTrajectories[eeIndex].contactFlag[j] == 0) //0 means in swing
				alwaysInStance = false;
			else
				alwaysInSwing = false;
			if (endEffectorTrajectories[eeIndex].contactFlag[j] == 0 && firstSwingIndex == -1)
				firstSwingIndex = j;
		}

		//so, if always in swing...
		if (alwaysInSwing){
			ffp.addStepPattern(endEffectorTrajectories[eeIndex].theLimb, 0, nPoints-1);
			continue;
		}
		
		//if always in stance...
		if (alwaysInStance) {
			ffp.addStepPattern(endEffectorTrajectories[eeIndex].theLimb, 10 * nPoints, 10 * nPoints);
			continue;
		}

		//ok, otherwise, we need to find the proper start and end...
		if (firstSwingIndex == 0) {
			if (endEffectorTrajectories[eeIndex].contactFlag[nPoints-1] == 0){
				firstSwingIndex = nPoints - 1;
				while (endEffectorTrajectories[eeIndex].contactFlag[firstSwingIndex - 1] == 0) firstSwingIndex -= 1;
			}
		}
		int lastSwingIndex = firstSwingIndex;
		while (endEffectorTrajectories[eeIndex].contactFlag[(lastSwingIndex + 1) % nPoints] == 0) lastSwingIndex = (lastSwingIndex + 1) % nPoints;
		ffp.addStepPattern(endEffectorTrajectories[eeIndex].theLimb, firstSwingIndex, lastSwingIndex);
	}
}

void LocomotionEngineMotionPlan::syncMotionPlanWithFootFallPattern(FootFallPattern& ffp){

	for (uint i=0;i<endEffectorTrajectories.size();i++){
		DynamicArray<double> &targetEEPosY = endEffectorTrajectories[i].targetEEPosY;
		GenericLimb* limb = endEffectorTrajectories[i].theLimb;

		targetEEPosY.resize(nSamplePoints);
		for (int j=0;j<nSamplePoints;j++){
			if (ffp.isInSwing(limb, j)){
				if (!ffp.isStart(limb, j) || ffp.isAlwaysInSwing(limb)){
					double swingPhase = ffp.getSwingPhaseForTimeIndex(limb, j);
					double heightRatio = 1 - fabs(0.5 - swingPhase) / 0.5;
//					Logger::consolePrint("swing phase: %lf height: %lf\n", swingPhase, heightRatio);
//					heightRatio = 1.0;
					targetEEPosY[j] = swingFootHeight * heightRatio;
				}
			}
		}
	}

	for (int j=0;j<nSamplePoints;j++){
		int nStanceLimbs = 0;
		double sumWeights = 0;
		for (uint i=0;i<this->endEffectorTrajectories.size();i++){
			GenericLimb* limb = endEffectorTrajectories[i].theLimb;
			if (ffp.isInStance(limb, j)){
				nStanceLimbs += 1;
				sumWeights += endEffectorTrajectories[i].EEWeights[j];
			}
		}

		for (uint i=0;i<endEffectorTrajectories.size();i++){
			GenericLimb* limb = endEffectorTrajectories[i].theLimb;
			endEffectorTrajectories[i].contactFlag[j] = 1;
//			endEffectorTrajectories[i].EEPos[j][1] = 0;

			if (ffp.isInSwing(limb, j)){
				endEffectorTrajectories[i].contactFlag[j] = 0;
//				endEffectorTrajectories[i].EEPos[j][1] = yPositions[i][j];
			}

			if (ffp.isInStance(limb, j)){
				if (sumWeights <= 0){
					endEffectorTrajectories[i].EEWeights[j] = 1.0 / nStanceLimbs;
				}else
					endEffectorTrajectories[i].EEWeights[j] /= sumWeights;
			}
		}
	}

/*
	//debug...
	FootFallPattern other;
	syncFootFallPatternWithMotionPlan(other);
	if (ffp.isSameAs(other))
		Logger::consolePrint("footfall patterns are the same!\n");
	else {
		Logger::consolePrint("footfall patterns are NOT the same!\n");
		ffp.writeToFile("../out/ffpOrig.ffp");
		other.writeToFile("../out/ffpOther.ffp");
//		exit(0);
	}
*/
}

void LocomotionEngineMotionPlan::getParameterMinValues(dVector &minV){
	std::vector<double> minLimits;

	if (optimizeCOMPositions){
		for (int i=0; i<nSamplePoints;i++)
			for (int j=0;j<3;j++){
				minLimits.push_back(0);
			}
	}

	if (optimizeCOMOrientations) {
		for (int i = 0; i < nSamplePoints; i++)
			for (int j = 0; j < 3; j++) {
				minLimits.push_back(0);
			}
	}

	if (optimizeEndEffectorPositions){
		for (int j=0; j<nSamplePoints;j++)
			for (uint i=0;i<endEffectorTrajectories.size();i++)
				for (int k=0; k<3; k++){
					minLimits.push_back(0);
				}
	}

	if (optimizeWheels){
		for (int j=0; j<nSamplePoints;j++)
			for (int i=0;i<nWheels;i++)
				for (int k=0; k<nWheelParams; k++){
					minLimits.push_back(0);
				}
		for (int j=0; j<nSamplePoints;j++)
			for (int i=0;i<nWheels;i++)
				minLimits.push_back(0);
		for (int j=0; j<nSamplePoints;j++)
			for (int i=0;i<nWheels;i++)
				minLimits.push_back(0);
	}

	if (optimizeBarycentricWeights){
		for (int j=0; j<nSamplePoints;j++)
			for (uint i=0;i<endEffectorTrajectories.size();i++){
				int nEECount = endEffectorTrajectories[i].theLimb->getLastLimbSegment()->rbProperties.getEndEffectorPointCount();
				minLimits.push_back(minBaricentricWeight / nEECount);
			}
	}

	if (optimizeRobotStates){
		for (int i=0;i<nSamplePoints;i++){
			for (int j=0;j<6;j++)
				minLimits.push_back(0);
			for (int j=6;j<robotStateTrajectory.nStateDim;j++)
				minLimits.push_back(((HingeJoint *)robot->getJoint(j-6))->minAngle);

		}
	}

	if (optimizeContactForces) {
		for (int j = 0; j<nSamplePoints; j++)
			for (uint i = 0; i<endEffectorTrajectories.size(); i++)
				for (int k = 0; k<3; k++) {
					minLimits.push_back(0);
				}
	}

	resize(minV, minLimits.size());
	for (int i = 0;i < minV.size();i++)
		minV[i] = minLimits[i];
}

void LocomotionEngineMotionPlan::getParameterMaxValues(dVector &maxV){
	std::vector<double> maxLimits;

	if (optimizeCOMPositions){
		for (int i=0; i<nSamplePoints;i++)
			for (int j=0;j<3;j++){
				maxLimits.push_back(0);
			}
	}

	if (optimizeCOMOrientations) {
		for (int i = 0; i < nSamplePoints; i++)
			for (int j = 0; j < 3; j++) {
				maxLimits.push_back(0);
			}
	}

	if (optimizeEndEffectorPositions){
		for (uint i=0;i<endEffectorTrajectories.size();i++)
		{
			for (int j=0; j<nSamplePoints;j++)
				for (int k=0; k<3; k++){
					maxLimits.push_back(0);
				}
			maxLimits.push_back(0);
		}
	}

	if (optimizeWheels){
		for (int j=0; j<nSamplePoints;j++)
			for (int i=0;i<nWheels;i++)
				for (int k=0; k<nWheelParams; k++){
					maxLimits.push_back(0);
				}
		for (int j=0; j<nSamplePoints;j++)
			for (int i=0;i<nWheels;i++)
				maxLimits.push_back(0);
		for (int j=0; j<nSamplePoints;j++)
			for (int i=0;i<nWheels;i++)
				maxLimits.push_back(0);
	}

	if (optimizeBarycentricWeights){
		for (int j=0; j<nSamplePoints;j++)
			for (uint i=0;i<endEffectorTrajectories.size();i++){
				maxLimits.push_back(1);
			}
	}

	if (optimizeRobotStates){
		for (int i=0;i<nSamplePoints;i++){
			for (int j=0;j<6;j++)
				maxLimits.push_back(0);
			for (int j=6;j<robotStateTrajectory.nStateDim;j++)
				maxLimits.push_back(((HingeJoint *)robot->getJoint(j-6))->maxAngle);
		}
	}

	if (optimizeContactForces) {
		for (int j = 0; j<nSamplePoints; j++)
			for (uint i = 0; i < endEffectorTrajectories.size(); i++) {
				maxLimits.push_back(0);
				if (enforceGRFConstraints)
					maxLimits.push_back(std::numeric_limits<double>::infinity());
				else
					maxLimits.push_back(0);
				maxLimits.push_back(0);
			}
	}

	resize(maxV, maxLimits.size());
	for (int i = 0;i < maxV.size();i++)
		maxV[i] = maxLimits[i];
}

dVector LocomotionEngineMotionPlan::getMPParameters()
{
	dVector params;
	writeMPParametersToList(params);
	return params;
}

void LocomotionEngineMotionPlan::writeMPParametersToList(dVector &p){
	updateParameterStartIndices();
	std::vector<double> params;

	if (optimizeCOMPositions){
		for (int i=0; i<nSamplePoints;i++)
			for (int j=0;j<3;j++)
				params.push_back(bodyTrajectory.pos[j][i]);
	}

	if (optimizeCOMOrientations) {
		for (int i = 0; i < nSamplePoints; i++)
			for (int j = 0; j < 3; j++)
				params.push_back(bodyTrajectory.orientation[j][i]);
	}

	if (optimizeEndEffectorPositions){
		for (int j=0; j<nSamplePoints;j++)
			for (uint i=0;i<endEffectorTrajectories.size();i++){
				params.push_back(endEffectorTrajectories[i].EEPos[j][0]);
				params.push_back(endEffectorTrajectories[i].EEPos[j][1]);
				params.push_back(endEffectorTrajectories[i].EEPos[j][2]);
			}
	}

	if (optimizeWheels){
		for (int j=0; j<nSamplePoints; j++)
			for (int i=0; i<nWheels; i++)
				params.push_back(endEffectorTrajectories[wheelToEEIndex[i]].wheelSpeed[j]);
		for (int j=0; j<nSamplePoints; j++)
			for (int i=0; i<nWheels; i++)
				params.push_back(endEffectorTrajectories[wheelToEEIndex[i]].wheelYawAngle[j]);
		for (int j=0; j<nSamplePoints; j++)
			for (int i=0; i<nWheels; i++)
				params.push_back(endEffectorTrajectories[wheelToEEIndex[i]].wheelTiltAngle[j]);
	}

	if (optimizeBarycentricWeights){
		for (int j=0; j<nSamplePoints;j++)
			for (uint i=0;i<endEffectorTrajectories.size();i++)
				params.push_back(endEffectorTrajectories[i].EEWeights[j]);
	}

	if (optimizeRobotStates){
		for (int i=0;i<nSamplePoints;i++)
			for (int j=0;j<robotStateTrajectory.nStateDim;j++)
				params.push_back(robotStateTrajectory.qArray[i][j]);
	}

	if (optimizeContactForces) {
		for (int j = 0; j<nSamplePoints; j++)
			for (uint i = 0; i<endEffectorTrajectories.size(); i++) {
				params.push_back(endEffectorTrajectories[i].contactForce[j][0]);
				params.push_back(endEffectorTrajectories[i].contactForce[j][1]);
				params.push_back(endEffectorTrajectories[i].contactForce[j][2]);
			}
	}

	resize(p, params.size());
	for (int i = 0;i < p.size();i++)
		p[i] = params[i];
}

void LocomotionEngineMotionPlan::setMPParametersFromList(const dVector &p){
	int pIndex = 0;
	if (optimizeCOMPositions){
		for (int i=0; i<nSamplePoints;i++)
			for (int j=0;j<3;j++)
				bodyTrajectory.pos[j][i] = p[pIndex++];
	}

	if (optimizeCOMOrientations) {
		for (int i = 0; i < nSamplePoints; i++)
			for (int j = 0; j < 3; j++)
				bodyTrajectory.orientation[j][i] = p[pIndex++];
	}

	if (optimizeEndEffectorPositions){
		for (int j=0; j<nSamplePoints;j++){
			for (uint i=0;i<endEffectorTrajectories.size();i++){
				endEffectorTrajectories[i].EEPos[j][0] = p[pIndex++];
				endEffectorTrajectories[i].EEPos[j][1] = p[pIndex++];
				endEffectorTrajectories[i].EEPos[j][2] = p[pIndex++];
			}
		}
	}

	if (optimizeWheels){
		for (int j=0; j<nSamplePoints;j++)
			for (int i=0;i<nWheels;i++)
				endEffectorTrajectories[wheelToEEIndex[i]].wheelSpeed[j] = p[pIndex++];
		for (int j=0; j<nSamplePoints;j++)
			for (int i=0;i<nWheels;i++)
				endEffectorTrajectories[wheelToEEIndex[i]].wheelYawAngle[j] = p[pIndex++];
		for (int j=0; j<nSamplePoints;j++)
			for (int i=0;i<nWheels;i++)
				endEffectorTrajectories[wheelToEEIndex[i]].wheelTiltAngle[j] = p[pIndex++];
	}

	if (optimizeBarycentricWeights){
		for (int j=0; j<nSamplePoints;j++)
			for (uint i=0;i<endEffectorTrajectories.size();i++)
				endEffectorTrajectories[i].EEWeights[j] = p[pIndex++];
	}

	if (optimizeRobotStates){
		for (int i=0;i<nSamplePoints;i++){
			for (int j=0;j<robotStateTrajectory.nStateDim;j++)
				robotStateTrajectory.qArray[i][j] = p[pIndex++];
		}
	}

	if (optimizeContactForces) {
		for (int j = 0; j<nSamplePoints; j++) {
			for (uint i = 0; i<endEffectorTrajectories.size(); i++) {
				endEffectorTrajectories[i].contactForce[j][0] = p[pIndex++];
				endEffectorTrajectories[i].contactForce[j][1] = p[pIndex++];
				endEffectorTrajectories[i].contactForce[j][2] = p[pIndex++];
			}
		}
	}
}

void LocomotionEngineMotionPlan::setPeriodicBoundaryConditionsToTimeSample(int loopAroundIndex){
	wrapAroundBoundaryIndex = loopAroundIndex;
	int from = loopAroundIndex;
	int to = nSamplePoints-1;

	for (uint j=0;j<endEffectorTrajectories.size();j++){
		endEffectorTrajectories[j].contactFlag[to] = endEffectorTrajectories[j].contactFlag[from];
		endEffectorTrajectories[j].EEWeights[to] = endEffectorTrajectories[j].EEWeights[from];
		endEffectorTrajectories[j].EEPos[to] = endEffectorTrajectories[j].EEPos[from] + desDistanceToTravel;
	}
}

void LocomotionEngineMotionPlan::writeDesiredValues(const char *fName){
	FILE* fp = fopen(fName, "w");

	fprintf(fp, "%lf %lf %lf\n", desDistanceToTravel[0], desDistanceToTravel[1], desDistanceToTravel[2]);

	for (int i=0;i<nSamplePoints;i++)
		for (int j=0;j<robotStateTrajectory.nStateDim;j++)
			fprintf(fp, "%lf\n", robotStateTrajectory.defaultRobotStates[i][j]);

	fclose(fp);
}

void LocomotionEngineMotionPlan::readDesiredValues(const char *fName){
	FILE* fp = fopen(fName, "r");

	fscanf(fp, "%lf %lf %lf", &desDistanceToTravel[0], &desDistanceToTravel[1], &desDistanceToTravel[2]);

	for (int i=0;i<nSamplePoints;i++)
		for (int j=0;j<robotStateTrajectory.nStateDim;j++)
			fscanf(fp, "%lf", &robotStateTrajectory.defaultRobotStates[i][j]);

	fclose(fp);
}

void LocomotionEngineMotionPlan::writeParamsToFile(FILE *fp) {

	fprintf(fp, "%d %d %d %lf\n", nSamplePoints, endEffectorTrajectories.size(), robotStateTrajectory.nStateDim, motionPlanDuration);

	fprintf(fp, "\n\n");

	for (int i = 0; i < nSamplePoints; i++)
		fprintf(fp, "%10.10lf %10.10lf %10.10lf\n", bodyTrajectory.pos[0][i], bodyTrajectory.pos[1][i], bodyTrajectory.pos[2][i]);

	fprintf(fp, "\n\n");

	for (int i = 0; i < nSamplePoints; i++)
		fprintf(fp, "%10.10lf %10.10lf %10.10lf\n", bodyTrajectory.orientation[0][i], bodyTrajectory.orientation[1][i], bodyTrajectory.orientation[2][i]);

	fprintf(fp, "\n\n");

	for (int j = 0; j < nSamplePoints; j++)
		for (uint i = 0; i < endEffectorTrajectories.size(); i++)
			fprintf(fp, "%10.10lf %10.10lf %10.10lf\n", endEffectorTrajectories[i].EEPos[j][0], endEffectorTrajectories[i].EEPos[j][1], endEffectorTrajectories[i].EEPos[j][2]);

	fprintf(fp, "\n\n");

	for (int j = 0; j < nSamplePoints; j++)
		for (uint i = 0; i < endEffectorTrajectories.size(); i++)
			fprintf(fp, "%10.10lf %10.10lf\n", endEffectorTrajectories[i].EEWeights[j], endEffectorTrajectories[i].contactFlag[j]);

	fprintf(fp, "\n\n");

	for (int j = 0; j < nSamplePoints; j++)
		for (uint i = 0; i < endEffectorTrajectories.size(); i++)
			fprintf(fp, "%10.10lf %10.10lf %10.10lf\n", endEffectorTrajectories[i].contactForce[j][0],
					endEffectorTrajectories[i].contactForce[j][1], endEffectorTrajectories[i].contactForce[j][2]);

	fprintf(fp, "\n\n");

	for (int j = 0; j < nSamplePoints; j++)
		for (uint i = 0; i < endEffectorTrajectories.size(); i++)
			if (endEffectorTrajectories[i].isWheel)
				fprintf(fp, "%10.10lf %10.10lf %10.10lf\n", endEffectorTrajectories[i].wheelSpeed[j], endEffectorTrajectories[i].wheelYawAngle[j], endEffectorTrajectories[i].wheelTiltAngle[j]);

	fprintf(fp, "\n\n");

	for (int i = 0; i < nSamplePoints; i++)
		for (int j = 0; j < robotStateTrajectory.nStateDim; j++)
			fprintf(fp, "%10.10lf\n", robotStateTrajectory.qArray[i][j]);

	fprintf(fp, "\n\n%10.10lf %10.10lf %10.10lf %10.10lf %10.10lf %10.10lf %10.10lf\n", swingFootHeight, desDistanceToTravel[0], desDistanceToTravel[2], desTurningAngle,
			motionPlanDuration, verticalGRFLowerBoundVal, GRFEpsilon);

	fprintf(fp, "\n\n%d\n", wrapAroundBoundaryIndex);

	fprintf(fp, "\n\n%lf %lf\n", externalForce[0], externalForce[2]);


	fprintf(fp, "\n\n");
}

void LocomotionEngineMotionPlan::writeParamsToFile(const char *fName){
	FILE* fp = fopen(fName, "w");
	writeParamsToFile(fp);
	fclose(fp);
}

void LocomotionEngineMotionPlan::readParamsFromFile(FILE *fp) {
	int nSamplePoints_, eeCount_, robotDim_;

	fscanf(fp, "%d %d %d %lf\n", &nSamplePoints_, &eeCount_, &robotDim_, &motionPlanDuration);

	if (eeCount_ != endEffectorTrajectories.size() || robotDim_ != robotStateTrajectory.nStateDim) {
		throwError("Loading Motion Plan: motion plan is not compatible with current robot!\n");
		return;
	}

	nSamplePoints = nSamplePoints_;


	robotRepresentation->setQ(initialRobotState);
	robotStateTrajectory.initialize(nSamplePoints);

	for (uint i = 0; i < endEffectorTrajectories.size(); i++)
		endEffectorTrajectories[i].initialize(nSamplePoints);

	V3D comRotationAngles(initialRobotState[3], initialRobotState[4], initialRobotState[5]);
	bodyTrajectory.initialize(nSamplePoints, defaultCOMPosition, comRotationAngles, robotRepresentation->getQAxis(3),
		robotRepresentation->getQAxis(4), robotRepresentation->getQAxis(5));

	for (int i = 0; i < nSamplePoints; i++)
		fscanf(fp, "%lf %lf %lf", &bodyTrajectory.pos[0][i], &bodyTrajectory.pos[1][i], &bodyTrajectory.pos[2][i]);

	for (int i = 0; i < nSamplePoints; i++)
		fscanf(fp, "%lf %lf %lf", &bodyTrajectory.orientation[0][i], &bodyTrajectory.orientation[1][i], &bodyTrajectory.orientation[2][i]);

	for (int j = 0; j < nSamplePoints; j++)
		for (uint i = 0; i < endEffectorTrajectories.size(); i++)
			fscanf(fp, "%lf %lf %lf", &endEffectorTrajectories[i].EEPos[j][0], &endEffectorTrajectories[i].EEPos[j][1], &endEffectorTrajectories[i].EEPos[j][2]);

	for (int j = 0; j < nSamplePoints; j++)
		for (uint i = 0; i < endEffectorTrajectories.size(); i++)
			fscanf(fp, "%lf %lf", &endEffectorTrajectories[i].EEWeights[j], &endEffectorTrajectories[i].contactFlag[j]);

	for (int j = 0; j < nSamplePoints; j++)
		for (uint i = 0; i < endEffectorTrajectories.size(); i++)
			fscanf(fp, "%lf %lf %lf", &endEffectorTrajectories[i].contactForce[j][0],
					&endEffectorTrajectories[i].contactForce[j][1], &endEffectorTrajectories[i].contactForce[j][2]);

	for (int j = 0; j < nSamplePoints; j++)
		for (uint i = 0; i < endEffectorTrajectories.size(); i++)
			if (endEffectorTrajectories[i].isWheel)
				fscanf(fp, "%lf %lf %lf", &endEffectorTrajectories[i].wheelSpeed[j], &endEffectorTrajectories[i].wheelYawAngle[j], &endEffectorTrajectories[i].wheelTiltAngle[j]);

	for (int i = 0; i < nSamplePoints; i++)
		for (int j = 0; j < robotStateTrajectory.nStateDim; j++)
			fscanf(fp, "%lf", &robotStateTrajectory.qArray[i][j]);

	fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf", &swingFootHeight, &desDistanceToTravel[0], &desDistanceToTravel[2], &desTurningAngle,
			&motionPlanDuration, &verticalGRFLowerBoundVal, &GRFEpsilon);

	wrapAroundBoundaryIndex = 0;
	fscanf(fp, "%d", &wrapAroundBoundaryIndex);

	fscanf(fp, "%lf %lf", &externalForce[0], &externalForce[2]);

}

void LocomotionEngineMotionPlan::readParamsFromFile(const char *fName){
	FILE* fp = fopen(fName, "r");
	readParamsFromFile(fp);
	fclose(fp);
}

void LocomotionEngineMotionPlan::writeRobotMotionAnglesToFile(const char *fName){

	auto& qArray = robotStateTrajectory.qArray;

	int nStateDim = robotStateTrajectory.nStateDim;

	//set<int> flipIndice = { 1, 3, 4, 6, 8, 10, 12, 14 };
	int flipIndice[18];
	for (int i = 0; i < 18; i++) flipIndice[i] = 1;
	flipIndice[0] = -1;
	flipIndice[2] = -1;
	flipIndice[4] = -1;

	FILE* fp = fopen(fName, "w+");

	fprintf(fp, "StateNum: %d, MotorNum: %d\n", qArray.size() - 1, robot->getJointCount());

	fprintf(fp, "{\n");

	//every line in the file will correspond to all joint angles of one state

	for (int i = 0; i < (int)qArray.size() - 1; i++) {

		// hard code for wheel example.

		fprintf(fp, "{");

		for (int j = 0; j < (robot->getJointCount() / 6); j++) {

			//double scale = flipIndice.count(j) ? -1 : 1;
			double scale = 1;
			int jIndex1 = robotRepresentation->getQIndexForJoint(robot->getJoint(6 * j));
			int jIndex2 = robotRepresentation->getQIndexForJoint(robot->getJoint(6 * j + 1));
			int jIndex3 = robotRepresentation->getQIndexForJoint(robot->getJoint(6 * j + 2));
			int jIndex4 = robotRepresentation->getQIndexForJoint(robot->getJoint(6 * j + 3));
			int jIndex5 = robotRepresentation->getQIndexForJoint(robot->getJoint(6 * j + 4));
			int jIndex6 = robotRepresentation->getQIndexForJoint(robot->getJoint(6 * j + 5));
			//fprintf(fp, "parent: %d", robot->getJoint(j)->parent->id);
			//fprintf(fp, "child: %d", robot->getJoint(j)->child->id);
			fprintf(fp, "%lf,%lf,%lf,%lf,%lf,%lf", DEG(qArray[i][jIndex2]) * flipIndice[6 * j], DEG(qArray[i][jIndex1])*flipIndice[6 * j + 1], DEG(qArray[i][jIndex6]) * flipIndice[6 * j + 2], DEG(qArray[i][jIndex5])*flipIndice[6 * j + 3], DEG(qArray[i][jIndex4]) * flipIndice[6 * j + 4], DEG(qArray[i][jIndex3])*flipIndice[6 * j + 5]);

			if ((6 * j + 5) < robot->getJointCount() - 1)

				fprintf(fp, ", ");

		}

		fprintf(fp, "},\n");

	}

	fprintf(fp, "},\n");

	fclose(fp);
}

LocomotionEngineMotionPlan::~LocomotionEngineMotionPlan(void){
	delete robotRepresentation;
}

void LocomotionEngineMotionPlan::updateRobotRepresentation(){
	//	this->robotRepresentation->setupDOFAxesAndParentOffsets();
	for(uint i = 0; i < endEffectorTrajectories.size(); i++)
		endEffectorTrajectories[i].endEffectorLocalCoords = endEffectorTrajectories[i].theLimb->getLastLimbSegment()->rbProperties.getEndEffectorPoint(endEffectorTrajectories[i].CPIndex);
}

void LocomotionEngineMotionPlan::updateParameterStartIndices(){
	paramCount = 0;
	COMPositionsParamsStartIndex = COMOrientationsParamsStartIndex = feetPositionsParamsStartIndex = wheelParamsStartIndex = barycentricWeightsParamsStartIndex = robotStatesParamsStartIndex = contactForcesParamsStartIndex = -1;

	if (optimizeCOMPositions){
		COMPositionsParamsStartIndex = paramCount;
		paramCount += 3 * nSamplePoints;
	}

	if (optimizeCOMOrientations) {
		COMOrientationsParamsStartIndex = paramCount;
		paramCount += 3 * nSamplePoints;
	}

	if (optimizeEndEffectorPositions){
		feetPositionsParamsStartIndex = paramCount;
		paramCount += nSamplePoints * endEffectorTrajectories.size() * 3;
	}

	if (optimizeWheels && nWheels>0){
		// per end effector wheel params: speed
		wheelParamsStartIndex = paramCount;
		paramCount += nSamplePoints * nWheels * nWheelParams;
		paramCount += nSamplePoints * nWheels * nWheelParamsEE;
		paramCount += nSamplePoints * nWheels * nWheelParamsEE;
	}

	if (optimizeBarycentricWeights){
		barycentricWeightsParamsStartIndex = paramCount;
		paramCount += nSamplePoints * endEffectorTrajectories.size();
	}

	if (optimizeRobotStates){
		robotStatesParamsStartIndex = paramCount;
		paramCount += robotStateTrajectory.nStateDim * nSamplePoints;
	}

	if (optimizeContactForces) {
		contactForcesParamsStartIndex = paramCount;
		paramCount += nSamplePoints * endEffectorTrajectories.size() * 3;
	}

}

/*
void LocomotionEngineMotionPlan::writeRobotMotionAnglesToFile(const char* fName)
{
	auto& qArray = robotStateTrajectory.qArray;
	int nStateDim = robotStateTrajectory.nStateDim;
	set<int> flipIndice = {1, 3, 4, 6, 8, 10, 12, 14};

	FILE* fp = fopen(fName, "w+");
	fprintf(fp, "StateNum: %d, MotorNum: %d\n", qArray.size() - 1, robot->getJointCount());
	fprintf(fp, "{\n");
	//every line in the file will correspond to all joint angles of one state
	for (int i = 0; i < (int)qArray.size() - 1; i++) {
		// hard code for wheel example.
		fprintf(fp, "{");
		for (int j = 0; j < robot->getJointCount(); j++) {
			double scale = flipIndice.count(j) ? -1 : 1;
			int jIndex = robotRepresentation->getQIndexForJoint(robot->getJoint(j));
			fprintf(fp, "%lf", DEG(qArray[i][jIndex]) * scale);
			if (j < robot->getJointCount() - 1)
				fprintf(fp, ", ");
		}
		fprintf(fp, "},\n");
	}
	fprintf(fp, "},\n");

	fclose(fp);
}
*/
P3D LocomotionEngineMotionPlan::getCOP(int tIndex) {
	P3D x = bodyTrajectory.getCOMPositionAtTimeIndex(tIndex);
	double h = x[1];
	x[1] = 0;

	//GLUtils::glLColor(1,1,1);
	//GLUtils::drawSphere(x, 0.01);
	int nIntervalCounts = nSamplePoints - 1;
	double t = motionPlanDuration / (nIntervalCounts);
	int jmm, jm, jp, jpp;
	getAccelerationTimeIndicesFor(tIndex, jmm, jm, jp, jpp);
	if (jmm >= 0 && jm >= 0 && jp >= 0 && jpp >= 0) {
		V3D vp = (bodyTrajectory.getCOMPositionAtTimeIndex(jpp) - bodyTrajectory.getCOMPositionAtTimeIndex(jp)) / t;
		V3D vm = (bodyTrajectory.getCOMPositionAtTimeIndex(jm) - bodyTrajectory.getCOMPositionAtTimeIndex(jmm)) / t;
		V3D xDotDot = (vp - vm) / t;
		double hDotDot = xDotDot[1];
		xDotDot[1] = 0;

		hDotDot = 0;
		h = desCOMHeight;
		V3D zmp = V3D(x) - xDotDot * (h / (hDotDot + fabs(Globals::g)));
		return P3D() + zmp;
	}
	return x;
}

P3D LocomotionEngineMotionPlan::getCenterOfRotationAt(double t, Eigen::VectorXd &error) const
{
	int nWheels = endEffectorTrajectories.size();

	Eigen::MatrixXd A(3*nWheels, 3);
	Eigen::VectorXd b(3*nWheels);

	int i = 0;
	for (const auto &ee : endEffectorTrajectories) {
		double alpha = ee.getWheelYawAngleAt(t);
		double beta = ee.getWheelTiltAngleAt(t);
		V3D wheelAxis = ee.wheelAxisLocal;
		V3D yawAxis = ee.wheelYawAxis;
		V3D tiltAxis = ee.wheelTiltAxis;
		P3D p3 = ee.getEEPositionAt(t);

		V3D v3 = LocomotionEngine_EndEffectorTrajectory::rotVecByYawTilt(wheelAxis, yawAxis, alpha, tiltAxis, beta);

		Eigen::Vector3d p = p3;
		p[1] = 0;
		Eigen::Vector3d v = v3;
		v[1] = 0;

		Matrix3x3 vvt = v*v.transpose();
		Matrix3x3 tmp = Matrix3x3::Identity() - vvt;

		for (int j = 0; j < 3; ++j) {
			for (int l = 0; l < 3; ++l) {
				A(3*i+j, l) = tmp(j, l);
			}
		}

		Eigen::Vector3d bb = tmp*p;
		for (int j = 0; j < 3; ++j) {
			b(3*i+j) = bb(j);
		}

		i++;
	}

	Eigen::Vector3d centerOfRotation = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
	error = (A*centerOfRotation - b).transpose();

	// TODO: fix operator = overloading of P3D
	P3D centerOfRotation3d;
	centerOfRotation3d = centerOfRotation;

	return centerOfRotation3d;
}

void LocomotionEngineMotionPlan::getVelocityTimeIndicesFor(int tIndex, int &tm, int &tp, bool wrapAround) const {
	tm = tIndex; tp = tIndex+1;
	if (tp == nSamplePoints){
		if (wrapAroundBoundaryIndex >= 0 && wrapAround){
			tm = wrapAroundBoundaryIndex;
			tp = wrapAroundBoundaryIndex+1;
		}
		else{
			tp = -1;
		}
	}
}

void LocomotionEngineMotionPlan::getAccelerationTimeIndicesFor(int tIndex, int& tmm, int& tm, int& tp, int& tpp, bool wrapAround){
	tp = tIndex, tpp = tIndex+1, tm = tIndex, tmm=tIndex-1;
	if (tm == 0){
		if (wrapAroundBoundaryIndex==0 && wrapAround){
			tm = nSamplePoints-1;
			tmm = nSamplePoints-2;
		}
		else{
			tm = -1;
			tmm = -1;
		}
	}

	if (tp == nSamplePoints-1){
		if (wrapAroundBoundaryIndex >= 0 && wrapAround){
			tp = wrapAroundBoundaryIndex;
			tpp = wrapAroundBoundaryIndex+1;
		}
		else{
			tp = -1;
			tpp = -1;
		}
	}
}

bool LocomotionEngineMotionPlan::getJointAngleVelocityProfile(std::vector<JointVelocity> &velocityProfile, std::string &error) const
{
	if (robotStatesParamsStartIndex < 0)
	{
		error = "robotStatesParamsStartIndex < 0";
		return false;
	}

	int startQIndex = 6;
	int endQIndex = robotRepresentation->getDimensionCount() - 1;

	int nTimeSteps = nSamplePoints;
	if (wrapAroundBoundaryIndex >= 0) nTimeSteps--;

	for (int j=0; j<nTimeSteps; j++){

		int jm, jp;

		getVelocityTimeIndicesFor(j, jm, jp);
		if (jm == -1 || jp == -1) continue;

		double dt = motionPlanDuration / nSamplePoints;
		double t = (double)j*dt;

		for (int i=startQIndex; i<=endQIndex; i++){
			double velocity = (robotStateTrajectory.qArray[jp][i] - robotStateTrajectory.qArray[jm][i]) / dt;
			velocityProfile.push_back(JointVelocity(t, i, velocity));
		}
	}

	return true;
}

bool LocomotionEngineMotionPlan::getJointAngleVelocityProfile(dVector &velocityProfile, int jointIndex) const
{
	if (robotStatesParamsStartIndex < 0)
	{
//		error = "robotStatesParamsStartIndex < 0";
		return false;
	}
	
	int nTimeSteps = nSamplePoints;
	if (wrapAroundBoundaryIndex >= 0) nTimeSteps--;
	int i = jointIndex + 6;
	velocityProfile.resize(nTimeSteps);
	double dt = motionPlanDuration / nTimeSteps;

	for (int j = 0; j < nTimeSteps; j++) {
		int jm, jp;

		getVelocityTimeIndicesFor(j, jm, jp);
		if (jm == -1 || jp == -1) continue;

		double velocity = (robotStateTrajectory.qArray[jp][i] - robotStateTrajectory.qArray[jm][i]) / dt;
		velocityProfile(j) = velocity;
	}

	return true;
}


//TODO: redo motion plan animation to update state in terms of deltas, rather than the animation cycle thing?
void LocomotionEngineMotionPlan::drawMotionPlan(double f, 
	bool drawRobotMesh,
	bool drawSkeleton,
	bool drawAxesOfRotation,
	bool drawWheels,
	bool drawContactForces,
	bool drawSupportPolygon,
	bool drawEndEffectorTrajectories,
	bool drawCOMTrajectory,
	bool drawOrientation){

	if (drawSupportPolygon) {
		//draw the support polygon...
		DynamicArray<int> stanceLimbs;
		DynamicArray<P3D> limbPositions;
		for (uint i = 0; i < endEffectorTrajectories.size(); i++) {
			if (endEffectorTrajectories[i].isInStance(f)) {
				stanceLimbs.push_back(i);
				limbPositions.push_back(endEffectorTrajectories[i].getEEPositionAt(f));
			}
		}
		DynamicArray<ConvexHull2D_Vertex> convexHullVertices;
		ConvexHull3D::planarConvexHullFromSetOfPoints(limbPositions, V3D(0, 1, 0), convexHullVertices);

		glDisable(GL_LIGHTING);
		glColor3d(1, 0, 0);
		glLineWidth(3.0);
		glBegin(GL_LINE_LOOP);
		for (uint i = 0; i < convexHullVertices.size(); i++) {
			int limbIndex = stanceLimbs[convexHullVertices[i].index];
			P3D p = endEffectorTrajectories[limbIndex].getEEPositionAt(f);
			glVertex3d(p[0], p[1] + 0.001, p[2]);
		}
		glEnd();
		glLineWidth(1.0);
	}
		//-----------------------------------

		//draw the COP and projected COM
		//glEnable(GL_LIGHTING);

		//----------------------------------

		//draw the COP trajectory
// 		if (!drawContactForces){
// 			int nIntervalCounts = nSamplePoints - 1;
// 			int tIndex = (int)(f * nIntervalCounts);
// 
// 			glColor3d(0.776, 0.733, 0.223);
// 			drawSphere(getCOP(tIndex), 0.003);
// 
// 			glLineWidth(5);
// 			glColor3d(0.776, 0.733, 0.223);
// 			glBegin(GL_LINE_STRIP);
// 				for (int i=0; i<nSamplePoints;i++){
// 					glColor3d(0.8,0.8,0.8);
// 					//GLUtils::drawSphere(x, 0.01);
// 					P3D zmp = getCOP(i);
// 					glVertex3d(zmp[0], 0.0001, zmp[2]);
// 				}
// 			glEnd();
// 			glLineWidth(1.0);
// 		}

	if (drawEndEffectorTrajectories) {
		//draw end effector trajectories
		glColor3d(0.5, 0.5, 1.0);
		glEnable(GL_LIGHTING);
		for (uint i = 0; i < endEffectorTrajectories.size(); i++) {
			P3D lastPoint = endEffectorTrajectories[i].getEEPositionAt(0);
			for (double j = 0.01; j <= 1; j += 0.01) {
				P3D p = endEffectorTrajectories[i].getEEPositionAt(j);
				drawCylinder(lastPoint, p, 0.001, 12, false);
				lastPoint = p;
			}
		}

		glColor3d(0.0, 0.0, 1.0);
		//draw end effector positions...
		for (uint i = 0; i < endEffectorTrajectories.size(); i++)
		{
/*
			if (endEffectorTrajectories[i].isHighlighted)
				glColor3d(0, 0, 1);
			else
				if (endEffectorTrajectories[i].isInStance(f))
					glColor3d(1, 0, 0);
				else
					glColor3d(0, 1, 0);
*/
			P3D p = endEffectorTrajectories[i].getEEPositionAt(f);
			drawSphere(p, 0.007, 12);
//			Logger::consolePrint("ee %d: %lf\n", i, p.y());
		}
	}

	if (drawCOMTrajectory) {
		//draw the COM trajectory
		glColor3d(1, 0.0, 0.0);
		P3D comP = bodyTrajectory.getCOMPositionAt(f);
		drawSphere(comP, 0.007, 12);

/*
		comP.y() = 0.001;
		drawSphere(comP, 0.003);
		glLineWidth(5);
		glColor3d(0.7, 0.7, 0.7);
		glBegin(GL_LINE_STRIP);
		for (int i = 0; i < nSamplePoints; i++) {
			glVertex3d(bodyTrajectory.pos[0][i], 0.001, bodyTrajectory.pos[2][i]);
		}
		glEnd();
		glLineWidth(1.0);
*/
//		glLineWidth(5);
		glColor3d(1.0, 0.5, 0.5);
//		glBegin(GL_LINE_STRIP);
		for (int i = 1; i < nSamplePoints; i++) {
			//glVertex3d(bodyTrajectory.pos[0][i], bodyTrajectory.pos[1][i], bodyTrajectory.pos[2][i]);
			P3D p1(bodyTrajectory.pos[0][i-1], bodyTrajectory.pos[1][i-1], bodyTrajectory.pos[2][i-1]);
			P3D p2(bodyTrajectory.pos[0][i], bodyTrajectory.pos[1][i], bodyTrajectory.pos[2][i]);
			drawCylinder(p1, p2, 0.001, 12, false);
		}
//		glEnd();
//		glLineWidth(1.0);
	}


	if (robot && (drawRobotMesh || drawSkeleton)){
		RobotState oldState(robot);
		RobotState robotState(robot);
		robotStateTrajectory.getRobotPoseAt(f, robotState);
		robot->setState(&robotState);
		
		if (drawAxesOfRotation){
			for (int i=0;i<robot->getJointCount();i++){
				HingeJoint* hj = dynamic_cast<HingeJoint*> (robot->getJoint(i));
				V3D v = hj->child->getWorldCoordinates(hj->rotationAxis) * 0.1;
				P3D p = hj->child->getWorldCoordinates(hj->cJPos);
				glBegin(GL_LINES);
					glVertex3d(p[0], p[1], p[2]);
					glVertex3d(p[0]+v[0], p[1]+v[1], p[2]+v[2]);
				glEnd();
			}
		}

		glEnable(GL_LIGHTING);


		//--------------------------------------

//		int flags = SHOW_ABSTRACT_VIEW;

		int flags = SHOW_MESH;
		if (drawSkeleton)
			flags = SHOW_ABSTRACT_VIEW;
		flags |= HIGHLIGHT_SELECTED;
		robot->draw(flags);
		//-------------------------------------

		robot->setState(&oldState);	

/*
		// draw center of rotation
		{
			glLineWidth(5);
			Eigen::VectorXd error;
			P3D COR = getCenterOfRotationAt(f, error);

			glColor4d(0.3, 0.3, 0.3, 0.5);
			for (uint i = 0; i < endEffectorTrajectories.size(); i++) {
				P3D wheelCenter = endEffectorTrajectories[i].getEEPositionAt(f);
				glBegin(GL_LINES);
					gl_Vertex3d(COR);
					gl_Vertex3d(wheelCenter);
				glEnd();

			}

			glColor4d(1.0, 0., 0., 1.0);
			glBegin(GL_LINES);
				gl_Vertex3d(COR);
				gl_Vertex3d(COR + V3D(0, error.norm(), 0));
			glEnd();

			glColor4d(0.0, 0.0, 1.0, 0.5);
			for (const auto &ee : endEffectorTrajectories) {
				P3D wheelCenter = ee.getEEPositionAt(f);
				double alpha = ee.getWheelYawAngleAt(f);
				double beta = ee.getWheelTiltAngleAt(f);
				V3D v = ee.getRotatedWheelAxis(alpha, beta);
//				V3D vWorld = endEffectorTrajectories[i].endEffectorRB->getWorldCoordinates(v);

				v[1] = 0;

				P3D a = wheelCenter + v*10;
				P3D b = wheelCenter - v*10;

				glBegin(GL_LINES);
					gl_Vertex3d(a);
					gl_Vertex3d(b);
				glEnd();

			}
		}
*/
	}


	glEnable(GL_LIGHTING);

	// drawContactForce
	if (drawContactForces) {
		glColor3d(0.0, 1.0, 0.0);
		for (uint i = 0; i < endEffectorTrajectories.size(); i++) {
			V3D contactForce = endEffectorTrajectories[i].getContactForceAt(f);
			P3D EEPos = endEffectorTrajectories[i].getEEPositionAt(f);
			drawArrow(EEPos, EEPos + contactForce * 0.05, 0.003, 12);
		}
	}


	// draw wheels at end effectors
	if (drawWheels){
		if (drawAxesOfRotation)
			for (const LocomotionEngine_EndEffectorTrajectory &ee : endEffectorTrajectories) {
				double alpha = ee.getWheelYawAngleAt(f);
				double beta = ee.getWheelTiltAngleAt(f);
				double radius = ee.wheelRadius;
				V3D axis = ee.getRotatedWheelAxis(alpha, beta);
				P3D wheelCenter = ee.getWheelCenterPositionAt(f);

	//			glColor4d(0.8, 0.2, 0.6, 0.8);
	//			drawArrow(wheelCenter, wheelCenter + ee.wheelYawAxis*radius*2.0, 0.003);

	//			glColor4d(0.8, 0.6, 0.2, 0.8);
	//			drawArrow(wheelCenter, wheelCenter + ee.wheelTiltAxis*radius*2.0, 0.003);

	//			glColor4d(0, 0, 0.5, 0.8);
	//			drawArrow(wheelCenter, wheelCenter + ee.wheelAxisLocal*radius*2.0, 0.003);

				glColor4d(0.2, 0.6, 0.8, 0.8);
				drawArrow(wheelCenter, wheelCenter + axis*0.05, 0.003);
			}

		double width = 0.01;
		for (const LocomotionEngine_EndEffectorTrajectory &ee : endEffectorTrajectories) {
			double alpha = ee.getWheelYawAngleAt(f);
			double beta = ee.getWheelTiltAngleAt(f);
			double radius = ee.wheelRadius;
			V3D axis = ee.getRotatedWheelAxis(alpha, beta);
			P3D wheelCenter = ee.getWheelCenterPositionAt(f);

			glColor4d(0.2, 0.6, 0.8, 0.8);
			drawCylinder(wheelCenter - axis*0.5*width, axis*width, radius, 24);
		}
	}

	// drawOrientation
	if (drawOrientation) {
		glPushMatrix();
		P3D comPos = bodyTrajectory.getCOMPositionAt(f);
		glTranslated(comPos[0], comPos[1], comPos[2]);
		//and rotation part
		V3D rotAxis; double rotAngle;
		Quaternion q = bodyTrajectory.getCOMOrientationAt(f);
		q.getAxisAngle(rotAxis, rotAngle);
		glRotated(DEG(rotAngle), rotAxis[0], rotAxis[1], rotAxis[2]);
		glEnable(GL_BLEND);
		glColor4d(0.5, 0.5, 0.5, 0.2);

		glEnable(GL_CULL_FACE);
		glCullFace(GL_FRONT);

		Matrix3x3 moi = Matrix3x3::Identity();
		moi(2, 2) /= 2.5;
		moi(1, 1) *= 1.6;
		moi(0, 0) *= 1.4;

		drawMOIApproximation(moi * totalInertia, totalMass);
		glCullFace(GL_BACK);
		drawMOIApproximation(moi * totalInertia, totalMass);
		glDisable(GL_CULL_FACE);

		/*glColor3d(1.0, 1.0, 0.0);
		drawArrow(P3D(), P3D(0.1, 0, 0), 0.005, 12);
		glColor3d(0.0, 1.0, 1.0);
		drawArrow(P3D(), P3D(0, 0.1, 0), 0.005, 12);
		glColor3d(1.0, 0.0, 1.0);
		drawArrow(P3D(), P3D(0, 0, 0.1), 0.005, 12);*/
		glPopMatrix();
	}

	glDisable(GL_LIGHTING);
}


int LocomotionEngineMotionPlan::getWheelSpeedIndex(int i, int j) const
{
	return wheelParamsStartIndex + j*nWheels + eeToWheelIndex.at(i);
}

int LocomotionEngineMotionPlan::getWheelYawAngleIndex(int i, int j) const
{
	return wheelParamsStartIndex + (nSamplePoints+j)*nWheels + eeToWheelIndex.at(i);
}

int LocomotionEngineMotionPlan::getWheelTiltAngleIndex(int i, int j) const
{
	return wheelParamsStartIndex + (2*nSamplePoints+j)*nWheels + eeToWheelIndex.at(i);
}
