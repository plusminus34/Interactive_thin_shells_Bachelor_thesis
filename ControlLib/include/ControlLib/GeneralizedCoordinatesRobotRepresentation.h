#pragma once

#include <ControlLib/Robot.h>
#include <MathLib/Matrix.h>

class ReducedRobotState;

//TODO: it's not ideal that the robot and its reduced representation can have different states...

//replace all direct accesses to world coords things through the getter function...

/**
	This class implements a reduced representation of an robot (i.e. we represent the configuration using vectors q and qDot, which represent the generalized coordinates of the robot.
*/
class GeneralizedCoordinatesRobotRepresentation{
	friend class QPControlPlan;
private:

	//this is the reference to the robot whose reduced representation is being stored
	Robot* robot;

//PARAMETERS THAT DEFINE THE MORPHOLOGY OF THE ROBOT

	//for each joint, the index at which its generalized coordinates start can be read off from this array
	DynamicArray<int> jointCoordStartIndex;
	//and for each joint, this is how many generalized coords we need to represent it (depending on what type of joints these are)...
	DynamicArray<int> jointCoordsDimSize;
	//this is all one big, tree-based hierarchichal transformation. Keep track of the hierarchy, by storing, for each degree of freedom, the index of the parent dof
	DynamicArray<int> qParentIndex;
	//for every q, keep track of the joint that it corresponds to...
	DynamicArray<int> jointIndexForQ;

//PARAMETERS THAT DEFINE THE STATE OF THE ROBOT - NEED TO BE UPDATED EVERY TIME THE STATE OF THE ROBOT CHANGES

	//generalized coordinates - pos and velocities
	dVector q, qDot;

	//store the (world) orientation for each q, for much quicker processing...
	DynamicArray<Quaternion> worldRotations;

public:

	//sets up the whole structure of the robot
	void setupGeneralizedCoordinatesStructure();

	//returns the local coord vector from the parent of q(qIndex) to q(qIndex)
	V3D getOffsetFromParentToQ(int qIndex);

	//returns the axis correponding to the indexed generalized coordinate, expressed in local coordinates
	V3D getQAxis(int qIndex);

	//updates the world-coords rotation axes. This method should be called whenever the state of the robot changes.
	void updateWorldOrientations();
	Quaternion getWorldRotationForQ(int qIndex);
	V3D getWorldCoordsAxisForQ(int qIndex);

	//returns the local position of the point that rb pivots about (i.e. location of the parent joint), in coordinate frame of rb
	P3D getPivotPointLocalPosition(RigidBody* rb);

	//projection to/from world coords and generalized coords
	void projectWorldCoordsValuesIntoGeneralizedSpace(const V3D& linearVal, const V3D& angularVal, const DynamicArray<V3D>& jointVal, dVector& generalizedArray);

	void projectVectorOnGeneralizedCoordsAxes(const V3D& vector, const V3D& a, const V3D& b, const V3D& c, double& aVal, double& bVal, double& cVal);
	void projectVectorOnGeneralizedCoordsAxes(const V3D& vector, const V3D& a, const V3D& b, double& aVal, double& bVal);
	void projectVectorOnGeneralizedCoordsAxes(const V3D& vector, const V3D& a, double& aVal);

	//computes the rigidbody rb's contribution (corioli and centrifugal force part) to the coriolisMatrix term, CMatrix(q, qDot) = (J'McJDot + J'[w]McJ)
	void computeCoriolisMatrix(RigidBody* rb, MatrixNxM &coriolisMatrix);

	//computes the generalized mass matrix for rigidbody rb: M = J'McJ, where Mc is a 6x6, local coordinates matrix, and M is a |q|x|q| generalized matrix...
	void computeMassMatrixForRB(RigidBody* rb, MatrixNxM &massMatrix);

public:
	/**
		the constructor
	*/
	GeneralizedCoordinatesRobotRepresentation(Robot* a);
	/**
		the destructor
	*/
	virtual ~GeneralizedCoordinatesRobotRepresentation(void);


	int getDimensionCount() {
		return (int)q.size();
	}

	//given the current state of the generalized representation, output the reduced state of the robot
	void getReducedRobotState(ReducedRobotState& state);

	//updates robot state given current q and qDot values...
	void syncRobotStateWithGeneralizedCoordinates();

	//updates q and qDot given current state of robot...
	void syncGeneralizedCoordinatesWithRobotState();

	//integrates state forward in time using input accelerations...
	void integrateGenerlizedAccelerationsForwardInTime(const dVector& a, double dt);

	// computes joint torques, expressed in world coordinates, given generalized torques u
	void computeWorldCoordinateTorquesFromU(const dVector& u);

	//sets the current q values
	void setQ(const dVector& qNew);

	//gets the current q values
	void getQ(dVector& q_copy);

    //sets the current qDot values
    void setQDot(const dVector& qDot);

    //gets the current qDot values
    void getQDot(dVector& qDot_copy);

	void getQFromReducedState(const ReducedRobotState& rs, dVector& q_copy);
	void getQAndQDotFromReducedState(const ReducedRobotState& rs, dVector& q_copy, dVector& qDot_copy);

	//returns the world coordinates for point p, which is specified in the local coordinates of rb (relative to its COM). I.e. p(q)
	P3D getWorldCoordinatesFor(const P3D& p, RigidBody* rb);

	//returns the velocity (world coordinates) of the point p, which is specified in the local coordinates of rb (relative to its COM). I.e. p(q)
	V3D getVelocityFor(const P3D& p, RigidBody* rb);

	//returns the angular velocity (world coordinates) for the rigid body rb
	V3D getAngularVelocityFor(RigidBody* rb);

	//returns the world-relative orientation for rb
	Quaternion getOrientationFor(RigidBody* rb);

	//computes the jacobian dp/dq that tells you how the world coordinates of p change with q. p is expressed in the local coordinates of rb
	void compute_dpdq(const P3D& p, RigidBody* rb, MatrixNxM &dpdq);


	//computes dpdq_dot, dpdq_dot = sigma(dpdq_dqi * qiDot) : JDot = dJ/dq * qDot
	void compute_dpdq_dot(const P3D& p, RigidBody* rb, MatrixNxM &dpdq_dot);

	//computed dRdq_dot, dRdq_dot = sigma(dRdq_dqi * qiDot) : JDot = dJ/dq * qDot
	void compute_angular_jacobian_dot(RigidBody* rb, MatrixNxM &dRdq_dot);

	//estimates the jacobian dp/dq using finite differences
	void estimate_linear_jacobian(const P3D& p, RigidBody* rb, MatrixNxM &dpdq);

	bool test_linear_jacobian(const P3D& p, RigidBody* rb);

	//computes the angular part of the jacobian, that, roughly speaking, relates changes in the orientation of a link to changes in q
	void compute_angular_jacobian(RigidBody* rb, MatrixNxM &dRdq);

	//estimates the angular jacobian using finite differences
	void estimate_angular_jacobian(RigidBody* rb, MatrixNxM &dRdq);

	bool test_angular_jacobian(RigidBody* rb);

	//computes the matrix that tells you how the jacobian dp/dq changes with respect to q_i. Returns true if it contains non-zero elements, false otherwise
	bool compute_ddpdq_dqi(const P3D& p, RigidBody* rb, MatrixNxM &ddpdq_dqi, int q_i);

	//estimates the change of dp/dq with respect to q_i
	void estimate_ddpdq_dqi(const P3D& p, RigidBody* rb, MatrixNxM &ddpdq_dqi, int q_i);

	bool test_linear_jacobian_derivatives(const P3D& p, RigidBody* rb);

	//computes the d(Jw)/dqi. Returns true if it contains non-zero elements, false otherwise
	bool compute_dangular_jacobian_dqi(RigidBody* rb, MatrixNxM &ddRdqdqi, int q_i);

	//estimates the change of angular jacobian with respect to q_i using finite differences
	void estimate_dangular_jacobian_dqi(RigidBody* rb, MatrixNxM &ddRdq_dqi, int q_i);

	bool test_angular_jacobian_derivatives(RigidBody* rb);


	//computes the mass matrix for the whole robot
	void computeMassMatrix(MatrixNxM &massMatrix);

	//computes the Coriolis Vector for the whole robot
	void computeCoriolisAndCentrifugalForcesTerm(dVector &C);

	//returns the qIndex at which this joint starts
	int getQIndexForJoint(Joint* joint);

	int getQIndexForJoint(int jIndex) {
		return jointCoordStartIndex[jIndex];
	}
};


inline void testGeneralizedCoordinateRepresentation(Robot* robot) {
	Logger::consolePrint("testing generalized coordinates representation...\n");

	//make sure we project errors introduced by physics engine (i.e. hinge joints not rotating only about their axis)
	robot->fixJointConstraints();
	GeneralizedCoordinatesRobotRepresentation gcrrNew(robot);

	//test out projections between robot state and generalized coordinates...
	ReducedRobotState robotState1(robot);

	dVector q1, q1Dot;
	gcrrNew.getQ(q1);
	gcrrNew.getQDot(q1Dot);

	gcrrNew.syncRobotStateWithGeneralizedCoordinates();
	ReducedRobotState robotState2(robot);

	ReducedRobotState robotState3(robot);
	gcrrNew.getReducedRobotState(robotState3);
	robot->setState(&robotState3);
	gcrrNew.syncGeneralizedCoordinatesWithRobotState();

	dVector q2, q2Dot;
	gcrrNew.getQ(q2);
	gcrrNew.getQDot(q2Dot);

	if (!robotState1.isSameAs(robotState2)) {
		Logger::consolePrint("TESTING GENERALIZED COORDINATES: robot state is not the same after projection...\n");
	}

	if ((q1 - q2).norm() > TINY) {
		Logger::consolePrint("TESTING GENERALIZED COORDINATES: generalized positions (q) are not the same before and after projection...\n");
	}

	if ((q1Dot - q2Dot).norm() > TINY) {
		Logger::consolePrint("TESTING GENERALIZED COORDINATES: generalized velocities (qDot) are not the same before and after projection...\n");
	}

	//test forward kinematics (world pos of points on RBs, velocity of points, etc)...
	for (int i = 0; i < robot->getJointCount(); i++) {
		P3D point = P3D() + getRandomUnitVector() * 0.2;
		P3D wc1 = robot->getJoint(i)->child->getWorldCoordinates(point);
		P3D wc2 = gcrrNew.getWorldCoordinatesFor(point, robot->getJoint(i)->child);
		if (V3D(wc1, wc2).length() > TINY)
			Logger::consolePrint("TESTING GENERALIZED COORDINATES: world coordinates of point on rigid body do not match up... error: %2.20lf\n", V3D(wc1, wc2).length());

		V3D wv1 = robot->getJoint(i)->child->getAbsoluteVelocityForLocalPoint(point);
		V3D wv2 = gcrrNew.getVelocityFor(point, robot->getJoint(i)->child);
		if ((wv1 - wv2).length() > TINY)
			Logger::consolePrint("TESTING GENERALIZED COORDINATES: velocities of point on rigid body do not match up... error: %2.20lf\n", (wv1 - wv2).length());

		wv1 = robot->getJoint(i)->child->getCMVelocity();
		wv2 = gcrrNew.getVelocityFor(P3D(), robot->getJoint(i)->child);
		if ((wv1 - wv2).length() > TINY)
			Logger::consolePrint("TESTING GENERALIZED COORDINATES: velocities of rigid body do not match up... error: %2.20lf\n", (wv1 - wv2).length());

		wv1 = robot->getJoint(i)->child->getAngularVelocity();
		wv2 = gcrrNew.getAngularVelocityFor(robot->getJoint(i)->child);
		if ((wv1 - wv2).length() > TINY)
			Logger::consolePrint("TESTING GENERALIZED COORDINATES: angular velocities of rigid body do not match up... error: %2.20lf\n", (wv1 - wv2).length());

		Quaternion q1 = robot->getJoint(i)->child->getOrientation();
		Quaternion q2 = gcrrNew.getOrientationFor(robot->getJoint(i)->child);
		if (q1 != q2 && q1 != (q2 * -1)) {
			Logger::consolePrint("TESTING GENERALIZED COORDINATES: orientations of rigid body do not match up... error: %lf %lf\n", q1.s, q2.s);
		}

		if (!gcrrNew.test_linear_jacobian(point, robot->getJoint(i)->child))
			Logger::consolePrint("TESTING GENERALIZED COORDINATES: linear jacobian does not match FD...\n");

		if (!gcrrNew.test_angular_jacobian(robot->getJoint(i)->child))
			Logger::consolePrint("TESTING GENERALIZED COORDINATES: angular jacobian does not match FD...\n");

		if (!gcrrNew.test_linear_jacobian_derivatives(point, robot->getJoint(i)->child))
			Logger::consolePrint("TESTING GENERALIZED COORDINATES: linear jacobian derivatives do not match FD...\n");

		if (!gcrrNew.test_angular_jacobian_derivatives(robot->getJoint(i)->child))
			Logger::consolePrint("TESTING GENERALIZED COORDINATES: angular jacobian derivatives do not match FD...\n");

	}

}



