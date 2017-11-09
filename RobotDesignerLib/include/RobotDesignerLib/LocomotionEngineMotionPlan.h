#pragma once

#include <MathLib/MathLib.h>
#include <MathLib/P3D.h>
#include <MathLib/V3D.h>
#include <MathLib/MathLib.h>
#include <MathLib/Trajectory.h>
#include <ControlLib/Robot.h>
#include <ControlLib/GeneralizedCoordinatesRobotRepresentation.h>
#include <RobotDesignerLib/FootFallPattern.h>
#include <vector>
#include <RBSimLib/HingeJoint.h>

class LocomotionEngine_EndEffectorTrajectory{
public:
	DynamicArray<P3D> EEPos;
	DynamicArray<V3D> contactForce;
	DynamicArray<double> contactFlag;
	DynamicArray<double> EEWeights;
	DynamicArray<P3D> defaultEEPos;
	DynamicArray<double> verticalGRFUpperBoundValues;
	DynamicArray<double> tangentGRFBoundValues;
	DynamicArray<double> wheelSpeed;
	double wheelRadius = 0.1;
	DynamicArray<double> wheelAxisAlpha;

	V3D targetOffsetFromCOM;
	RigidBody* endEffectorRB;
	P3D endEffectorLocalCoords;

	DynamicArray<double> targetEEPosY;

	//this is the limb the end effector trajectory belongs to
	GenericLimb* theLimb;
	//and this is the index of the end effector contact point that it represents
	int CPIndex;

	LocomotionEngine_EndEffectorTrajectory(int nPos){
		endEffectorRB = NULL;
		theLimb = NULL;
		CPIndex = -1;
		initialize(nPos);
	}

	LocomotionEngine_EndEffectorTrajectory(){
		endEffectorRB = NULL;
		theLimb = NULL;
		CPIndex = -1;
	}

	void initialize(int nPos){
		contactForce.resize(nPos);
		EEPos.resize(nPos);
		wheelSpeed.resize(nPos);
		wheelAxisAlpha.resize(nPos);
		defaultEEPos.resize(nPos);
		contactFlag.resize(nPos, 0);
		EEWeights.resize(nPos, 0.05);
		verticalGRFUpperBoundValues.resize(nPos, 1000.0);
		tangentGRFBoundValues.resize(nPos, 1000.0);
	}

	V3D getContactForceAt(double t) {
		//very slow method, but easy to implement...
//		Trajectory3D traj;
//		for (uint i = 0; i<contactForce.size(); i++)
//			traj.addKnot((double)i / (contactForce.size() - 1), contactForce[i]);
//		return traj.evaluate_linear(t);
		boundToRange(&t, 0, 1);
		int tIndex = (int)(t * (double)(contactFlag.size() - 1));
		return contactForce[tIndex];
	}

	//t is assumed to be between 0 and 1, which is a normalized scale of the whole motion plan...
	P3D getEEPositionAt(double t){
		//very slow method, but easy to implement...
		Trajectory3D traj;
		for (uint i = 0;i<EEPos.size();i++)
			traj.addKnot((double)i / (EEPos.size() - 1), EEPos[i]);
		return P3D() + traj.evaluate_linear(t);
	}

	//t is assumed to be between 0 and 1, which is a normalized scale of the whole motion plan...
	double getWheelAxisAlphaAt(double t){
		//very slow method, but easy to implement...
		Trajectory1D traj;
		for (uint i = 0; i<wheelAxisAlpha.size(); i++)
			traj.addKnot((double)i / (wheelAxisAlpha.size() - 1), wheelAxisAlpha[i]);
		return traj.evaluate_linear(t);
	}

	//t is assumed to be between 0 and 1, which is a normalized scale of the whole motion plan...
	double getContactFlagAt(double t){
		boundToRange(&t, 0, 1);
		int tIndex = (int)(t * (double)(contactFlag.size()-1));
		return contactFlag[tIndex];
	}

	bool isInStance(double t) {
		return getContactFlagAt(t) > 0.5;
	}

	bool isInSwing(double t) {
		return !isInStance(t);
	}

	double getEEWeightAt(double t){
		boundToRange(&t, 0, 1);
		int tIndex = (int)floor(t * (EEWeights.size()));
		if (tIndex >= (int)EEWeights.size()) tIndex = (int)EEWeights.size()-1;
		return EEWeights[tIndex];
	}

};

class LocomotionEngine_COMTrajectory{
public:

	int nPoints;
	dVector pos[3];
	//orientation is stored as euler angles...
	dVector orientation[3];
	V3D axis[3];

	LocomotionEngine_COMTrajectory(){

	}

	void initialize(int nPoints, const P3D& desComPos, const V3D& axis_0,
		const V3D& axis_1, const V3D& axis_2){
		this->nPoints = nPoints;
		pos[0] = dVector(nPoints); pos[1] = dVector(nPoints); pos[2] = dVector(nPoints);
		orientation[0] = dVector(nPoints); orientation[1] = dVector(nPoints); orientation[2] = dVector(nPoints);
		orientation[0].setZero(); orientation[1].setZero(); orientation[2].setZero();
		axis[0] = axis_0; axis[1] = axis_1; axis[2] = axis_2;

		//note - fixed parameterization!
		for (int i=0;i<nPoints;i++)
			for (int j=0;j<3;j++)
				pos[j][i] = desComPos[j];
	}

	V3D getAxis(int i) {
		return axis[i];
	}

	//t is assumed to be between 0 and 1, which is a normalized scale of the whole motion plan...
	P3D getCOMPositionAt(double t){
		boundToRange(&t, 0, 1);
		double tSize = t * (double)(pos[0].size()-1);

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

	P3D getCOMEulerAnglesAt(double t) {
		boundToRange(&t, 0, 1);
		double tSize = t * (double)(orientation[0].size() - 1);

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

	Quaternion getCOMOrientationAt(double t) {
		P3D eulerAngles = getCOMEulerAnglesAt(t);
		return getRotationQuaternion(eulerAngles[0], axis[0]) *
			getRotationQuaternion(eulerAngles[1], axis[1]) * getRotationQuaternion(eulerAngles[2], axis[2]);
	}

	P3D getCOMPositionAtTimeIndex(int j){
		return P3D(pos[0][j], pos[1][j], pos[2][j]);
	}

	P3D getCOMEulerAnglesAtTimeIndex(int j) {
		return P3D(orientation[0][j], orientation[1][j], orientation[2][j]);
	}

	Quaternion getCOMOrientationAtTimeIndex(int j) {
		P3D eulerAngles = getCOMEulerAnglesAtTimeIndex(j);
		return getRotationQuaternion(eulerAngles[0], axis[0]) *
			getRotationQuaternion(eulerAngles[1], axis[1]) * getRotationQuaternion(eulerAngles[2], axis[2]);
	}
};


class LocomotionEngine_RobotStateTrajectory{
public:
	DynamicArray<dVector> qArray;
	GeneralizedCoordinatesRobotRepresentation* robotRepresentation;
	DynamicArray<dVector> defaultRobotStates;

	int nStateDim;

	LocomotionEngine_RobotStateTrajectory(GeneralizedCoordinatesRobotRepresentation* robotRepresentation, int nPos){
		this->robotRepresentation = robotRepresentation;
		initialize(nPos);
	}

	LocomotionEngine_RobotStateTrajectory(){
		this->robotRepresentation = NULL;
	}

	void initialize(int nPoints){
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

	//t is assumed to be between 0 and 1, which is a normalized scale of the whole motion plan...
	P3D getBodyPositionAt(double t){
		if (robotRepresentation == NULL) return P3D();
		if (t > 1) t -= 1; 	if (t < 0) t += 1;
		int tIndex = (int)floor(t * (qArray.size()));
		if (tIndex >= (int)qArray.size()) tIndex = (int)qArray.size()-1;
		return P3D(qArray[tIndex][0], qArray[tIndex][1], qArray[tIndex][2]);
	}

	void getQ(double t, dVector& q_t) {
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

	//t is assumed to be between 0 and 1, which is a normalized scale of the whole motion plan...
	void getRobotPoseAt(double t, ReducedRobotState& robotPose){
		if (robotRepresentation == NULL) return;
		dVector q; getQ(t, q);
		robotRepresentation->setQ(q);
		robotRepresentation->getReducedRobotState(robotPose);
	}

	//t is assumed to be between 0 and 1, which is a normalized scale of the whole motion plan...
	void getRobotStateAt(double t, double motionPlanDuration, ReducedRobotState& robotState) {
		if (t > 1) t -= 1.0;
		double dStridePhase = 0.01;
		double dt = dStridePhase * motionPlanDuration;

		ReducedRobotState futureRobotState(robotState.getStateSize());

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

	void writeRobotMotionTrajectoriesToFile(const char* fName) {
		FILE* fp = fopen(fName, "w");

		fprintf(fp, "%d\n", qArray.size());
		//every line in the file will correspond to the angle trajectory for one of the joints
		for (int j = 0; j<nStateDim; j++) {
			for (uint i = 0; i<qArray.size(); i++)
				fprintf(fp, "%lf\t", qArray[i][j]);
			fprintf(fp, "\n");
		}
		fclose(fp);
	}

	void loadRobotMotionTrajectoriesToFile(const char* fName) {
		FILE* fp = fopen(fName, "r");

		int nSamples;
		fscanf(fp, "%d", &nSamples);
		initialize(nSamples);

		//every line in the file will correspond to the angle trajectory for one of the joints
		for (int j = 0; j<nStateDim; j++) {
			for (uint i = 0; i<qArray.size(); i++)
				fscanf(fp, "%lf", &qArray[i][j]);
		}
		fclose(fp);
	}

	void getQAtTimeIndex(int j, dVector& q_t){
		q_t = qArray[j];
	}

	P3D getBodyPositionAtTimeIndex(int j){
		return P3D(qArray[j][0], qArray[j][1], qArray[j][2]);
	}
};

/**
	This is a motion plan for an arbitrary robot model
*/
class LocomotionEngineMotionPlan{
public:
	LocomotionEngineMotionPlan(Robot* robot, int nSamplingPoints);

	virtual ~LocomotionEngineMotionPlan(void);

	void drawMotionPlan(double f, int animationCycle = 0, bool drawRobot = true, bool drawSkeleton = false, bool drawPlanDetails = false, bool drawContactForces = false, bool drawOrientation = false);
	void drawMotionPlan2(double f, int animationCycle = 0, bool drawRobotPose = true, bool drawPlanDetails = false);

	double motionPlanDuration = 0.8; //1.5
	double swingFootHeight = 0.02;	

	V3D desDistanceToTravel;
	double desTurningAngle = 0.0;
	double desCOMHeight = 0.0;
	P3D defaultCOMPosition;
	double minBaricentricWeight = 0.15;
	double verticalGRFLowerBoundVal = 0;
	double GRFEpsilon = 0.4;				// for SoftUnilateralConstraint
	double pseudoLimbEpsilon = 0.1;

	// Parameters for motor velocity constraint
	double jointVelocityLimit = 0;
	double jointVelocityEpsilon = 0.4;		// for SoftUnilateralConstraint

protected:
	
public:
	bool optimizeCOMPositions;
	bool optimizeCOMOrientations;
	bool optimizeEndEffectorPositions;
	bool optimizeWheels;
	bool optimizeBarycentricWeights;
	bool optimizeRobotStates;
	bool optimizeContactForces;

	bool enforceGRFConstraints;

	const static int nWheelParams = 1; // wheel speed
	const static int nWheelParamsEE = 1; // wheel speed

	//TODO: optimize contact flags too?!?

	int nSamplePoints = -1;

	int COMPositionsParamsStartIndex;
	int COMOrientationsParamsStartIndex;
	int feetPositionsParamsStartIndex;
	int wheelParamsStartIndex;
	int barycentricWeightsParamsStartIndex;
	int robotStatesParamsStartIndex;
	int contactForcesParamsStartIndex;
	int paramCount;

	//points to the point that, in joint angle space, should correspond to the last time index, for boundary conditions...
	int wrapAroundBoundaryIndex;

	// the time index that transition starts
	int transitionStartIndex = -1;
	// the time index that transition ends
	int transitionEndIndex = -1;
	// the stride phase that transition starts
	double transitionStartPhase = -1.0;
	// the time index that transition ends
	double transitionEndPhase = -1.0;
	// the motion plan that the transition starts from
	LocomotionEngineMotionPlan* transitionStartPlan = NULL;
	// the motion plan that the transition ends with
	LocomotionEngineMotionPlan* transitionEndPlan = NULL;

	double totalMass;
	double totalInertia;
	double frictionCoeff = -1.0;     // when frictionCoeff < 0, friction cone constraints are disabled.

public:
	int getWheelAxisAlphaIndex(int i, int j) const;

public:
	DynamicArray<LocomotionEngine_EndEffectorTrajectory> endEffectorTrajectories;
	LocomotionEngine_COMTrajectory COMTrajectory;
	LocomotionEngine_RobotStateTrajectory robotStateTrajectory;

	Robot* robot;
	GeneralizedCoordinatesRobotRepresentation* robotRepresentation;

	void updateRobotRepresentation();

	void updateParameterStartIndices(){
		paramCount = 0;
		COMPositionsParamsStartIndex = COMOrientationsParamsStartIndex = feetPositionsParamsStartIndex = barycentricWeightsParamsStartIndex = robotStatesParamsStartIndex = contactForcesParamsStartIndex = -1;

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

		if (optimizeWheels){
			// per end effector wheel params: speed
			wheelParamsStartIndex = paramCount;
			paramCount += nSamplePoints * endEffectorTrajectories.size() * nWheelParams;
			paramCount += nSamplePoints * endEffectorTrajectories.size() * nWheelParamsEE;
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

	//syncs the footfall pattern with the current motion plan
	void syncFootFallPatternWithMotionPlan(FootFallPattern& ffp);
	//syncs the current motion plan with the footfall pattern
	void syncMotionPlanWithFootFallPattern(FootFallPattern& ffp);
	void syncMotionPlanWithFootFallPattern(FootFallPattern& ffp, const std::vector<std::vector<double> > &yPositions);

	//if minV is equal to maxV, then there are no bounds for that variable...
	virtual void getParameterMinValues(dVector& minV){
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
				for (uint i=0;i<endEffectorTrajectories.size();i++)
					for (int k=0; k<nWheelParams; k++){
						minLimits.push_back(0);
					}
			for (int j=0; j<nSamplePoints;j++)
				for (uint i=0;i<endEffectorTrajectories.size();i++)
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

	//if minV is equal to maxV, then there are no bounds for that variable...
	virtual void getParameterMaxValues(dVector& maxV){
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
				for (uint i=0;i<endEffectorTrajectories.size();i++)
					for (int k=0; k<nWheelParams; k++){
						maxLimits.push_back(0);
					}
			for (int j=0; j<nSamplePoints;j++)
				for (uint i=0;i<endEffectorTrajectories.size();i++)
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

	virtual void writeMPParametersToList(dVector& p){
		updateParameterStartIndices();
		std::vector<double> params;

		if (optimizeCOMPositions){
			for (int i=0; i<nSamplePoints;i++)
				for (int j=0;j<3;j++)
				params.push_back(COMTrajectory.pos[j][i]);
		}

		if (optimizeCOMOrientations) {
			for (int i = 0; i < nSamplePoints; i++)
				for (int j = 0; j < 3; j++)
					params.push_back(COMTrajectory.orientation[j][i]);
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
				for (uint i=0; i<endEffectorTrajectories.size(); i++){
					params.push_back(endEffectorTrajectories[i].wheelSpeed[j]);
				}
			for (int j=0; j<nSamplePoints; j++)
				for (uint i=0; i<endEffectorTrajectories.size(); i++)
					params.push_back(endEffectorTrajectories[i].wheelAxisAlpha[j]);
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

	virtual void setMPParametersFromList(const dVector& p){
		int pIndex = 0;
		if (optimizeCOMPositions){
			for (int i=0; i<nSamplePoints;i++)
				for (int j=0;j<3;j++)
					COMTrajectory.pos[j][i] = p[pIndex++];
		}

		if (optimizeCOMOrientations) {
			for (int i = 0; i < nSamplePoints; i++)
				for (int j = 0; j < 3; j++)
					COMTrajectory.orientation[j][i] = p[pIndex++];
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
			for (int j=0; j<nSamplePoints;j++){
				for (uint i=0;i<endEffectorTrajectories.size();i++){
					endEffectorTrajectories[i].wheelSpeed[j] = p[pIndex++];
				}
			}
			for (int j=0; j<nSamplePoints;j++)
				for (uint i=0;i<endEffectorTrajectories.size();i++)
					endEffectorTrajectories[i].wheelAxisAlpha[j] = p[pIndex++];
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

	void setPeriodicBoundaryConditionsToTimeSample(int loopAroundIndex){
		wrapAroundBoundaryIndex = loopAroundIndex;
		int from = loopAroundIndex;
		int to = nSamplePoints-1;

		for (uint j=0;j<endEffectorTrajectories.size();j++){
			endEffectorTrajectories[j].contactFlag[to] = endEffectorTrajectories[j].contactFlag[from];
			endEffectorTrajectories[j].EEWeights[to] = endEffectorTrajectories[j].EEWeights[from];
			endEffectorTrajectories[j].EEPos[to] = endEffectorTrajectories[j].EEPos[from] + desDistanceToTravel;
		}
	}

	void writeDesiredValues(const char* fName){
		FILE* fp = fopen(fName, "w");

		fprintf(fp, "%lf %lf %lf\n", desDistanceToTravel[0], desDistanceToTravel[1], desDistanceToTravel[2]);

		for (int i=0;i<nSamplePoints;i++)
			for (int j=0;j<robotStateTrajectory.nStateDim;j++)
				fprintf(fp, "%lf\n", robotStateTrajectory.defaultRobotStates[i][j]);

		fclose(fp);
	}

	void readDesiredValues(const char* fName){
		FILE* fp = fopen(fName, "r");

		fscanf(fp, "%lf %lf %lf", &desDistanceToTravel[0], &desDistanceToTravel[1], &desDistanceToTravel[2]);

		for (int i=0;i<nSamplePoints;i++)
			for (int j=0;j<robotStateTrajectory.nStateDim;j++)
				fscanf(fp, "%lf", &robotStateTrajectory.defaultRobotStates[i][j]);

		fclose(fp);
	}

	void writeParamsToFile(FILE* fp) {

		fprintf(fp, "%d %d %d %lf\n", nSamplePoints, endEffectorTrajectories.size(), robotStateTrajectory.nStateDim, motionPlanDuration);

		fprintf(fp, "\n\n");

		for (int i = 0; i < nSamplePoints; i++)
			fprintf(fp, "%10.10lf %10.10lf %10.10lf\n", COMTrajectory.pos[0][i], COMTrajectory.pos[1][i], COMTrajectory.pos[2][i]);

		fprintf(fp, "\n\n");

		for (int i = 0; i < nSamplePoints; i++)
			fprintf(fp, "%10.10lf %10.10lf %10.10lf\n", COMTrajectory.orientation[0][i], COMTrajectory.orientation[1][i], COMTrajectory.orientation[2][i]);

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

		for (int i = 0; i < nSamplePoints; i++)
			for (int j = 0; j < robotStateTrajectory.nStateDim; j++)
				fprintf(fp, "%10.10lf\n", robotStateTrajectory.qArray[i][j]);

		fprintf(fp, "\n\n%10.10lf %10.10lf %10.10lf %10.10lf %10.10lf %10.10lf %10.10lf\n", swingFootHeight, desDistanceToTravel[0], desDistanceToTravel[2], desTurningAngle,
			motionPlanDuration, verticalGRFLowerBoundVal, GRFEpsilon);

		fprintf(fp, "\n\n%d %d %10.10lf %10.10lf\n", transitionStartIndex, transitionEndIndex, transitionStartPhase, transitionEndPhase);

		fprintf(fp, "\n\n%d\n", wrapAroundBoundaryIndex);

		fprintf(fp, "\n\n");
	}

	void writeParamsToFile(const char* fName){
		FILE* fp = fopen(fName, "w");
		writeParamsToFile(fp);
		fclose(fp);
	}

	void readParamsFromFile(FILE* fp) {

		int nSamplePoints_, eeCount_, robotDim_;

		fscanf(fp, "%d %d %d %lf\n", &nSamplePoints_, &eeCount_, &robotDim_, &motionPlanDuration);

		if (eeCount_ != endEffectorTrajectories.size() || robotDim_ != robotStateTrajectory.nStateDim) {
			throwError("Loading Motion Plan: motion plan is not compatible with current robot!\n");
			return;
		}

		nSamplePoints = nSamplePoints_;
		robotStateTrajectory.initialize(nSamplePoints);
		for (uint i = 0; i < endEffectorTrajectories.size(); i++)
			endEffectorTrajectories[i].initialize(nSamplePoints);
		COMTrajectory.initialize(nSamplePoints, P3D(), robotRepresentation->getQAxis(3),
			robotRepresentation->getQAxis(4), robotRepresentation->getQAxis(5));

		for (int i = 0; i < nSamplePoints; i++)
			fscanf(fp, "%lf %lf %lf", &COMTrajectory.pos[0][i], &COMTrajectory.pos[1][i], &COMTrajectory.pos[2][i]);

		for (int i = 0; i < nSamplePoints; i++)
			fscanf(fp, "%lf %lf %lf", &COMTrajectory.orientation[0][i], &COMTrajectory.orientation[1][i], &COMTrajectory.orientation[2][i]);

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

		for (int i = 0; i < nSamplePoints; i++)
			for (int j = 0; j < robotStateTrajectory.nStateDim; j++)
				fscanf(fp, "%lf", &robotStateTrajectory.qArray[i][j]);

		fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf", &swingFootHeight, &desDistanceToTravel[0], &desDistanceToTravel[2], &desTurningAngle,
			&motionPlanDuration, &verticalGRFLowerBoundVal, &GRFEpsilon);

		fscanf(fp, "%d %d %lf %lf", &transitionStartIndex, &transitionEndIndex, &transitionStartPhase, &transitionEndPhase);

		wrapAroundBoundaryIndex = 0;
		fscanf(fp, "%d", &wrapAroundBoundaryIndex);
	}

	void readParamsFromFile(const char* fName){
		FILE* fp = fopen(fName, "r");
		readParamsFromFile(fp);
		fclose(fp);
	}


	void writeRobotMotionAnglesToFile(const char* fName){

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


	P3D getCOP(int tIndex);

	void getVelocityTimeIndicesFor(int tIndex, int& tm, int& tp, bool wrapAround = true) const;

	void getAccelerationTimeIndicesFor(int tIndex, int& tmm, int& tm, int& tp, int& tpp, bool wrapAround = true);

	struct JointVelocity {
		JointVelocity() {}
		JointVelocity(double t, int qIndex, double velocity)
			: t(t), qIndex(qIndex), velocity(velocity){}
		double t; int qIndex; double velocity;
	};
	bool getJointAngleVelocityProfile(std::vector<JointVelocity> &velocityProfile, std::string &error) const;
};

