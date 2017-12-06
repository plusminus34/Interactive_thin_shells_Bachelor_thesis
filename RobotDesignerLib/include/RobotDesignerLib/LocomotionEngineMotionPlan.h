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
	// world coords of end effector position
	// for a wheel, EEPos is the point of contact with the ground
	DynamicArray<P3D> EEPos;
	DynamicArray<V3D> contactForce;
	DynamicArray<double> contactFlag;
	DynamicArray<double> EEWeights;
	DynamicArray<P3D> defaultEEPos;
	DynamicArray<double> verticalGRFUpperBoundValues;
	DynamicArray<double> tangentGRFBoundValues;

	double wheelRadius = 0.1;			// wheel radius
	DynamicArray<double> wheelSpeed;	// angular speed of wheel around `wheelAxis`
	V3D wheelAxis;						// wheel axis in world coords.
	V3D wheelYawAxis;					// yaw axis in world coords.
	V3D wheelTiltAxis;					// tilt axis in world coords.
	DynamicArray<double> wheelYawAngle;	// rotation around yaw axis
	DynamicArray<double> wheelTiltAngle;// rotation around tilt axis

	V3D targetOffsetFromCOM;
	RigidBody* endEffectorRB;
	P3D endEffectorLocalCoords;

	DynamicArray<double> targetEEPosY;

	//this is the limb the end effector trajectory belongs to
	GenericLimb* theLimb;
	//and this is the index of the end effector contact point that it represents
	int CPIndex;

public:
	LocomotionEngine_EndEffectorTrajectory(int nPos);

	LocomotionEngine_EndEffectorTrajectory();

	void initialize(int nPos);

	V3D getContactForceAt(double t);

	//t is assumed to be between 0 and 1, which is a normalized scale of the whole motion plan...
	P3D getEEPositionAt(double t) const;

	// TODO: maybe we can store rho alongside with wheelAxis etc.
	V3D getWheelRho() const;

	P3D getWheelCenterPositionAt(double t) const;

	//t is assumed to be between 0 and 1, which is a normalized scale of the whole motion plan...
	double getWheelYawAngleAt(double t) const;

	double getWheelTiltAngleAt(double t) const;

	template<class T>
	static Vector3T<T> rotateWheelAxisWith(const Vector3T<T> &axis, const Vector3T<T> &axisYaw,  T alpha, const Vector3T<T> &axisTilt, T beta) {
		// First tilt the axis ...
		Vector3T<T> axisRot = rotateVec(axis, beta, axisTilt);
		// ... and then yaw
		axisRot = rotateVec(axisRot, alpha, axisYaw);
		return axisRot;
	}

	template<class T>
	Vector3T<T> getRotatedWheelAxis(T angleYaw, T angleTilt) const
	{
		Vector3T<T> axis(wheelAxis);
		Vector3T<T> axisYaw(wheelYawAxis);
		Vector3T<T> axisTilt(wheelTiltAxis);

		return rotateWheelAxisWith(axis, axisYaw, angleYaw, axisTilt, angleTilt);
	}

	//t is assumed to be between 0 and 1, which is a normalized scale of the whole motion plan...
	double getContactFlagAt(double t);

	bool isInStance(double t);

	bool isInSwing(double t);

	double getEEWeightAt(double t);
};

class LocomotionEngine_COMTrajectory{
public:

	int nPoints;
	dVector pos[3];
	//orientation is stored as euler angles...
	dVector orientation[3];
	V3D axis[3];

public:
	LocomotionEngine_COMTrajectory();

	void initialize(int nPoints, const P3D& desComPos, const V3D& axis_0, const V3D& axis_1, const V3D& axis_2);

	V3D getAxis(int i);

	//t is assumed to be between 0 and 1, which is a normalized scale of the whole motion plan...
	P3D getCOMPositionAt(double t);

	P3D getCOMEulerAnglesAt(double t);

	Quaternion getCOMOrientationAt(double t);

	P3D getCOMPositionAtTimeIndex(int j);

	P3D getCOMEulerAnglesAtTimeIndex(int j);

	Quaternion getCOMOrientationAtTimeIndex(int j);
};

class LocomotionEngine_RobotStateTrajectory{
public:
	DynamicArray<dVector> qArray;
	GeneralizedCoordinatesRobotRepresentation* robotRepresentation;
	DynamicArray<dVector> defaultRobotStates;

	int nStateDim;

public:
	LocomotionEngine_RobotStateTrajectory(GeneralizedCoordinatesRobotRepresentation* robotRepresentation, int nPos);

	LocomotionEngine_RobotStateTrajectory();

	void initialize(int nPoints);

	//t is assumed to be between 0 and 1, which is a normalized scale of the whole motion plan...
	P3D getBodyPositionAt(double t);

	void getQ(double t, dVector& q_t);

	//t is assumed to be between 0 and 1, which is a normalized scale of the whole motion plan...
	void getRobotPoseAt(double t, ReducedRobotState& robotPose);

	//t is assumed to be between 0 and 1, which is a normalized scale of the whole motion plan...
	void getRobotStateAt(double t, double motionPlanDuration, ReducedRobotState& robotState);

	void writeRobotMotionTrajectoriesToFile(const char* fName);

	void loadRobotMotionTrajectoriesToFile(const char* fName);

	void getQAtTimeIndex(int j, dVector& q_t);

	P3D getBodyPositionAtTimeIndex(int j);
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

	// Parameters for joint motor velocity constraint
	double jointVelocityLimit = 0;
	double jointVelocityEpsilon = 0.4;		// for SoftUnilateralConstraint
	
	// Parameters for wheel motor speed constraint
	double wheelSpeedLimit = 0;
	double wheelSpeedEpsilon = 0.4;		// for SoftUnilateralConstraint

public:
	bool optimizeCOMPositions;
	bool optimizeCOMOrientations;
	bool optimizeEndEffectorPositions;
	bool optimizeWheels;
	bool optimizeBarycentricWeights;
	bool optimizeRobotStates;
	bool optimizeContactForces;

	bool enforceGRFConstraints;

	// TODO: clean up / consolidate nWheelParams and nWheelParamsEE
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
	int getWheelSpeedIndex(int i, int j) const;
	int getWheelYawAngleIndex(int i, int j) const;
	int getWheelTiltAngleIndex(int i, int j) const;

public:
	DynamicArray<LocomotionEngine_EndEffectorTrajectory> endEffectorTrajectories;
	LocomotionEngine_COMTrajectory COMTrajectory;
	LocomotionEngine_RobotStateTrajectory robotStateTrajectory;

	Robot* robot;
	GeneralizedCoordinatesRobotRepresentation* robotRepresentation;

	void updateRobotRepresentation();

	void updateParameterStartIndices();

	//syncs the footfall pattern with the current motion plan
	void syncFootFallPatternWithMotionPlan(FootFallPattern& ffp);
	//syncs the current motion plan with the footfall pattern
	void syncMotionPlanWithFootFallPattern(FootFallPattern& ffp);
	void syncMotionPlanWithFootFallPattern(FootFallPattern& ffp, const std::vector<std::vector<double> > &yPositions);

	//if minV is equal to maxV, then there are no bounds for that variable...
	virtual void getParameterMinValues(dVector& minV);

	//if minV is equal to maxV, then there are no bounds for that variable...
	virtual void getParameterMaxValues(dVector& maxV);

	virtual void writeMPParametersToList(dVector& p);

	virtual void setMPParametersFromList(const dVector& p);

	void setPeriodicBoundaryConditionsToTimeSample(int loopAroundIndex);

	void writeDesiredValues(const char* fName);

	void readDesiredValues(const char* fName);

	void writeParamsToFile(FILE* fp);

	void writeParamsToFile(const char* fName);

	void readParamsFromFile(FILE* fp);

	void readParamsFromFile(const char* fName);

	void writeRobotMotionAnglesToFile(const char* fName);

	P3D getCOP(int tIndex);

	P3D getCenterOfRotationAt(double t, Eigen::VectorXd &error) const;

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
