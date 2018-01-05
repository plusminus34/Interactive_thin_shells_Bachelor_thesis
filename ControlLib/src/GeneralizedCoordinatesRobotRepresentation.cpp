#include <ControlLib/GeneralizedCoordinatesRobotRepresentation.h>
#include <Utils/Utils.h>
#include <stdio.h>
#include <RBSimLib/BallAndSocketJoint.h>
#include <RBSimLib/UniversalJoint.h>
#include <RBSimLib/HingeJoint.h>

GeneralizedCoordinatesRobotRepresentation::GeneralizedCoordinatesRobotRepresentation(Robot* a){
	robot = a;
	setupGeneralizedCoordinatesStructure();
	syncGeneralizedCoordinatesWithRobotState();
}

GeneralizedCoordinatesRobotRepresentation::~GeneralizedCoordinatesRobotRepresentation(void) {

}

//sets up the whole structure of the robot - a tree structure
void GeneralizedCoordinatesRobotRepresentation::setupGeneralizedCoordinatesStructure() {
	int nTotalDim = 6;//at the very least, the dofs for the root of the robot
	for (int i = 0; i<6; i++) {
		qParentIndex.push_back(i - 1);
		jointIndexForQ.push_back(-1);
	}

	for (uint i = 0; i<robot->jointList.size(); i++) {
		int nDim = 0;
		if (dynamic_cast<BallAndSocketJoint*>(robot->jointList[i]) != NULL)
			nDim = 3;
		if (dynamic_cast<UniversalJoint*>(robot->jointList[i]) != NULL)
			nDim = 2;
		if (dynamic_cast<HingeJoint*>(robot->jointList[i]) != NULL)
			nDim = 1;

		assert(nDim > 0);

		jointCoordStartIndex.push_back(nTotalDim);
		jointCoordsDimSize.push_back(nDim);
		for (int j = 0; j<nDim; j++) jointIndexForQ.push_back(i);

		nTotalDim += nDim;

		//the child can be connected straight to the root...
		if (robot->jointList[i]->parent->pJoints.size() == 0) {
			//the hierarchichal parent for this joint is the last root rotation DOF
			qParentIndex.push_back(5);
		}
		else {
			//this, going backwards through the transformation hierarchy, is the index of the first q of the parent...
			int qpIndex = robot->jointList[i]->parent->pJoints[0]->jIndex;
			//check where the parent joint's q DOFs start -- that's the hierarchichal parent for the last DOF of the current joint
			qParentIndex.push_back(jointCoordStartIndex[qpIndex] + jointCoordsDimSize[qpIndex] - 1);
		}

		for (int j = 1; j<nDim; j++)
			qParentIndex.push_back(jointCoordStartIndex[i] + j - 1);
	}

	resize(q, nTotalDim);
	resize(qDot, nTotalDim);
//	qAxes.resize(nTotalDim);
//	qOffsetFromParent.resize(nTotalDim);

	worldRotations.resize(nTotalDim);

//	setupDOFAxes();
}

//returns the qIndex at which this joint starts
int GeneralizedCoordinatesRobotRepresentation::getQIndexForJoint(Joint* joint) {
	return jointCoordStartIndex[joint->jIndex];
}



/*
//updates all the rotation/translation axes that qs act about. Important if there are 3DOF rotational jointList, since we can re-align the axes to avoid gimble locks
void GeneralizedCoordinatesRobotRepresentation::setupDOFAxes() {
	//the first three are the translational dofs of the body
	qAxes[0] = V3D(1, 0, 0);
	qAxes[1] = V3D(0, 1, 0);
	qAxes[2] = V3D(0, 0, 1);
	//the next three are the rotational dofs of the body, which we align with the current root rotation to stay well away from gimbal locks and all that

	V3D v1, v2, v3;
	//we'll be using a yaw/pitch/roll parameterization here
	v3 = Globals::worldUp;	//yaw
	v2 = robot->right;		//pitch
	v1 = robot->forward;	//roll

	//although we could re-parameterize if ever we get too close to a gimbal lock
	//	computeEulerAxesFromQuaternion(robot->root->state.orientation, v1, v2, v3);

	qAxes[3] = v3;//y
	qAxes[4] = v2;//x
	qAxes[5] = v1;//z

	//now go through each joint, and decompose it as appropriate...
	for (uint i = 0; i<robot->jointList.size(); i++) {
		if (BallAndSocketJoint* j = dynamic_cast<BallAndSocketJoint*>(robot->jointList[i])) {
			Quaternion q;
			V3D v1, v2, v3;
			q = j->computeRelativeOrientation();
			computeEulerAxesFromQuaternion(robot->root->state.orientation, v1, v2, v3);
			qAxes[jointCoordStartIndex[i] + 0] = v3;
			qAxes[jointCoordStartIndex[i] + 1] = v2;
			qAxes[jointCoordStartIndex[i] + 2] = v1;
			continue;
		}
		if (UniversalJoint* j = dynamic_cast<UniversalJoint*>(robot->jointList[i])) {
			qAxes[jointCoordStartIndex[i] + 0] = j->rotAxisParent;
			qAxes[jointCoordStartIndex[i] + 1] = j->rotAxisChild;
			continue;
		}
		if (HingeJoint* j = dynamic_cast<HingeJoint*>(robot->jointList[i])) {
			qAxes[jointCoordStartIndex[i] + 0] = j->rotationAxis;
			continue;
		}
		//not sure what other types of jointList there are...
		assert(false);
	}


	for (int qIndex = 0; qIndex < getDimensionCount(); qIndex++) {
		int qIndexParent = qParentIndex[qIndex];
		//if they both belong to the same joint, then their locations coincide
		if (jointIndexForQ[qIndex] == -1 || jointIndexForQ[qIndex] == jointIndexForQ[qIndexParent])
			qOffsetFromParent[qIndex] = V3D();
		else
			qOffsetFromParent[qIndex] = V3D(getPivotPointLocalPosition(robot->jointList[jointIndexForQ[qIndex]]->parent), robot->jointList[jointIndexForQ[qIndex]]->pJPos);
	}
}
*/


//updates the world-coords rotation axes. This method should be called when the state or structure of the robot changes.
void GeneralizedCoordinatesRobotRepresentation::updateWorldOrientations(){
	worldRotations[0] = worldRotations[1] = worldRotations[2] = Quaternion(1, 0, 0, 0);

	for (int j = 3; j<6; j++)
		worldRotations[j] = worldRotations[j - 1] * getRotationQuaternion(q[j], getQAxis(j));

	for (uint i = 0; i<robot->jointList.size(); i++) {
		Quaternion parentOrientation = getOrientationFor(robot->jointList[i]->parent);
		for (int j = 0; j<jointCoordsDimSize[i]; j++) {
			int jIndex = jointCoordStartIndex[i] + j;
			worldRotations[jIndex] = parentOrientation * getRotationQuaternion(q[jIndex], getQAxis(jIndex));
			parentOrientation = worldRotations[jIndex];
		}
	}
}

void GeneralizedCoordinatesRobotRepresentation::integrateGenerlizedAccelerationsForwardInTime(const dVector& a, double dt){
	setQDot(qDot + a * dt);
	setQ(q + qDot * dt);
}

// computes joint torques, expressed in world coordinates, given generalized torques u
void GeneralizedCoordinatesRobotRepresentation::computeWorldCoordinateTorquesFromU(const dVector& u) {
	// clear desired torque
	for (uint i = 0; i < robot->jointList.size(); ++i)
		robot->jointList[i]->desiredJointTorque = V3D();

	// joints list size
	assert(u.size() == q.size());

	for (int i = 6; i < getDimensionCount(); ++i) {
		robot->jointList[jointIndexForQ[i]]->controlMode = TORQUE_MODE;
		robot->jointList[jointIndexForQ[i]]->desiredJointTorque += getWorldCoordsAxisForQ(i) * u[i] * -1;
	}
}

//updates q and qDot given current state of robot
void GeneralizedCoordinatesRobotRepresentation::syncGeneralizedCoordinatesWithRobotState() {
	//write out the position of the root...
	RobotState state(robot);
	P3D pos = state.getPosition();
	Quaternion orientation = state.getOrientation();

	q[0] = pos[0];
	q[1] = pos[1];
	q[2] = pos[2];

	// Root
	computeEulerAnglesFromQuaternion(orientation, getQAxis(5), getQAxis(4), getQAxis(3), q[5], q[4], q[3]);

	// Now go through each joint, and decompose it as appropriate...
	for (uint i = 0; i<robot->jointList.size(); i++) {
		if (BallAndSocketJoint* j = dynamic_cast<BallAndSocketJoint*>(robot->jointList[i])){
			computeEulerAnglesFromQuaternion(state.getJointRelativeOrientation(i),
				getQAxis(jointCoordStartIndex[i] + 2), getQAxis(jointCoordStartIndex[i] + 1), getQAxis(jointCoordStartIndex[i] + 0),
				q[jointCoordStartIndex[i] + 2], q[jointCoordStartIndex[i] + 1], q[jointCoordStartIndex[i] + 0]);
			continue;
		}
		if (UniversalJoint* j = dynamic_cast<UniversalJoint*>(robot->jointList[i])) {
			computeEulerAnglesFromQuaternion(state.getJointRelativeOrientation(i), getQAxis(jointCoordStartIndex[i] + 1), getQAxis(jointCoordStartIndex[i] + 0), q[jointCoordStartIndex[i] + 1], q[jointCoordStartIndex[i] + 0]);
			continue;
		}
		if (HingeJoint* j = dynamic_cast<HingeJoint*>(robot->jointList[i])) {
			computeRotationAngleFromQuaternion(state.getJointRelativeOrientation(i), getQAxis(jointCoordStartIndex[i] + 0), q[jointCoordStartIndex[i] + 0]);
			continue;
		}
		//not sure what other types of jointList there are...
		assert(false);
	}

	//the rotation angles (q values) have changed, so update the world coords axes/rotations...
	updateWorldOrientations();

	//now update the velocities qDot
	DynamicArray<V3D> worldRelJointVelocities;
	for (uint i = 0; i<robot->jointList.size(); i++)
		//the reduced state stores angular velocities in parent coords, so change them to world
		worldRelJointVelocities.push_back(getOrientationFor(robot->jointList[i]->parent).rotate(state.getJointRelativeAngVelocity(i)));

	projectWorldCoordsValuesIntoGeneralizedSpace(state.getVelocity(), state.getAngularVelocity(), worldRelJointVelocities, qDot);
}


Quaternion GeneralizedCoordinatesRobotRepresentation::getWorldRotationForQ(int qIndex) {
	return worldRotations[qIndex];
}

V3D GeneralizedCoordinatesRobotRepresentation::getWorldCoordsAxisForQ(int qIndex) {
	return worldRotations[qIndex].rotate(getQAxis(qIndex));
}

//returns the local coord vector from the parent of q(qIndex) to q(qIndex)
V3D GeneralizedCoordinatesRobotRepresentation::getOffsetFromParentToQ(int qIndex) const {
	int qIndexParent = qParentIndex[qIndex];
	//if they both belong to the same joint, then their locations coincide
	if (jointIndexForQ[qIndex] == -1 || jointIndexForQ[qIndex] == jointIndexForQ[qIndexParent])
		return V3D();
	return V3D(getPivotPointLocalPosition(robot->jointList[jointIndexForQ[qIndex]]->parent), robot->jointList[jointIndexForQ[qIndex]]->pJPos);

	//	return qOffsetFromParent[qIndex];
}

//returns the axis corresponding to the indexed generalized coordinate, expressed in local coordinates
V3D GeneralizedCoordinatesRobotRepresentation::getQAxis(int qIndex) const {
	if (qIndex>=0 || qIndex <6){
		//the first three are the translational dofs of the body
		if (qIndex == 0) return V3D(1, 0, 0);
		if (qIndex == 1) return V3D(0, 1, 0);
		if (qIndex == 2) return V3D(0, 0, 1);

		//the next three are the rotational dofs of the body, which we align with the current root rotation to stay well away from gimbal locks and all that
		//we'll be using a yaw/pitch/roll parameterization here
 		//v3 = Globals::worldUp;	//yaw
 		//v2 = robot->right;		//pitch
 		//v1 = robot->forward;	//roll

		//although we could re-parameterize if ever we get too close to a gimbal lock
		//	computeEulerAxesFromQuaternion(robot->root->state.orientation, v1, v2, v3);

		if (qIndex == 3) return Globals::worldUp;//y
		if (qIndex == 4) return robot->right;//x
		if (qIndex == 5) return robot->forward;//z
	}


	int jIndex = jointIndexForQ[qIndex];

	if (HingeJoint* j = dynamic_cast<HingeJoint*>(robot->jointList[jIndex])) {
		return j->rotationAxis;
	}

	if (BallAndSocketJoint* j = dynamic_cast<BallAndSocketJoint*>(robot->jointList[jIndex])) {
//		Quaternion q;
//		V3D v1, v2, v3;
//		q = j->computeRelativeOrientation();
//		computeEulerAxesFromQuaternion(robot->root->state.orientation, v1, v2, v3);
//		qAxes[jointCoordStartIndex[i] + 0] = v3;
//		qAxes[jointCoordStartIndex[i] + 1] = v2;
//		qAxes[jointCoordStartIndex[i] + 2] = v1;
		Logger::logPrint("BallAndSocketJoint not yet supported...\n");
		exit(0);
	}
	if (UniversalJoint* j = dynamic_cast<UniversalJoint*>(robot->jointList[jIndex])) {
//		qAxes[jointCoordStartIndex[i] + 0] = j->rotAxisParent;
//		qAxes[jointCoordStartIndex[i] + 1] = j->rotAxisChild;
		Logger::logPrint("Universal joint not yet supported...\n");
		exit(0);
	}

	Logger::logPrint("Not sure what kind of joint this is...\n");
	exit(0);
}


//returns the local position of the point that rb pivots about (i.e. location of the parent joint), in coordinate frame of rb
P3D GeneralizedCoordinatesRobotRepresentation::getPivotPointLocalPosition(RigidBody* rb)  const {
	if (rb->pJoints.size() == 0)
		return P3D();

	return rb->pJoints[0]->cJPos;
}

void GeneralizedCoordinatesRobotRepresentation::syncRobotStateWithGeneralizedCoordinates() {
	RobotState rs(robot);
	getReducedRobotState(rs);
	robot->setState(&rs);
}

//given the current state of the generalized representation, output the reduced state of the robot
void GeneralizedCoordinatesRobotRepresentation::getReducedRobotState(RobotState& state) {
	//set the position, velocity, rotation and angular velocity for the root
	state.setPosition(P3D() + getQAxis(0) * q[0] + getQAxis(1) * q[1] + getQAxis(2) * q[2]);
	state.setVelocity(V3D(getQAxis(0) * qDot[0] + getQAxis(1) * qDot[1] + getQAxis(2) * qDot[2]));
	state.setAngularVelocity(getWorldCoordsAxisForQ(3) * qDot[3] + getWorldCoordsAxisForQ(4) * qDot[4] + getWorldCoordsAxisForQ(5) * qDot[5]);
	state.setOrientation(getWorldRotationForQ(5));

	for (uint i = 0; i<robot->jointList.size(); i++) {
		Quaternion jointOrientation;
		V3D jointAngularVel;
		for (int j = 0; j<jointCoordsDimSize[i]; j++) {
			int jIndex = jointCoordStartIndex[i] + j;
			jointOrientation *= getRotationQuaternion(q[jIndex], getQAxis(jIndex));
			jointAngularVel += getWorldCoordsAxisForQ(jIndex) * qDot[jIndex];
		}
		//the relative angular velocity is now in world coords, so we need to change it in the coords of the parent
		V3D tmp; getOrientationFor(robot->jointList[i]->parent).inverseRotate(jointAngularVel, &tmp);
		state.setJointRelativeOrientation(jointOrientation, i);
		state.setJointRelativeAngVelocity(tmp, i);
	}
	//and done...
}

//sets the current q values
void GeneralizedCoordinatesRobotRepresentation::setQ(const dVector& qNew) {
	assert(q.size() == qNew.size());
	//NOTE: we don't update the angular velocities. The assumption is that the correct behavior is that the joint relative angular velocities don't change, although the world relative values of the rotations do
	q = qNew;
	updateWorldOrientations();
}

//gets the current q values
void GeneralizedCoordinatesRobotRepresentation::getQ(dVector& q_copy) {
	q_copy = q;
}

//sets the current qDot values
void GeneralizedCoordinatesRobotRepresentation::setQDot(const dVector& qDotNew) {
	assert(q.size() == qDotNew.size());
	qDot = qDotNew;
	//updateWorldOrientationsAndAxes();
}

//gets the current qDot values
void GeneralizedCoordinatesRobotRepresentation::getQDot(dVector& qDot_copy) {
	qDot_copy = qDot;
}

//V3D GeneralizedCoordinatesRobotRepresentation::getQAxis(int qIndex) {
//	return getQAxis(qIndex);
//}

void GeneralizedCoordinatesRobotRepresentation::getQAndQDotFromReducedState(const RobotState& rs, dVector& q_copy, dVector& qDot_copy) {
	dVector q_old = q;
	dVector qDot_old = qDot;

	RobotState oldState(robot);
	robot->setState((RobotState*)&rs);
	syncGeneralizedCoordinatesWithRobotState();
	getQ(q_copy);
	getQDot(qDot_copy);
	robot->setState(&oldState);

	setQ(q_old);
	setQDot(qDot);
}


void GeneralizedCoordinatesRobotRepresentation::getQFromReducedState(const RobotState& rs, dVector& q_copy) {
	dVector q_old = q;
	dVector qDot_old = qDot;

	RobotState oldState(robot);
	robot->setState((RobotState*)&rs);
	syncGeneralizedCoordinatesWithRobotState();
	getQ(q_copy);
	robot->setState(&oldState);

	setQ(q_old);
	setQDot(qDot);
}


void GeneralizedCoordinatesRobotRepresentation::projectVectorOnGeneralizedCoordsAxes(const V3D& vec, const V3D& a, const V3D& b, const V3D& c, double& aVal, double& bVal, double& cVal) {
	//we cannot assume the vectors form an orthogonal basis, so we have to solve a system to compute the unknowns aVal, bVal, cVal. We assume that vec, a, b and c are all specified in the same coordinate frame
	Matrix3x3 m, mInv;
	m(0, 0) = a[0]; m(0, 1) = b[0]; m(0, 2) = c[0];
	m(1, 0) = a[1]; m(1, 1) = b[1]; m(1, 2) = c[1];
	m(2, 0) = a[2]; m(2, 1) = b[2]; m(2, 2) = c[2];

	mInv = m.inverse();

	V3D sol = V3D(mInv * vec);

	aVal = sol[0]; bVal = sol[1]; cVal = sol[2];

	//check the solution...
	V3D res = V3D(m * sol) - vec;
	if (res.length() > 0.0001)
		Logger::consolePrint("failed to properly compute generalized coordinates for the input :(.\n");
}

void GeneralizedCoordinatesRobotRepresentation::projectVectorOnGeneralizedCoordsAxes(const V3D& vec, const V3D& a, const V3D& b, double& aVal, double& bVal) {
	//we assume the two vectors are orthogonal to each other...
	assert(IS_ZERO(a.dot(b)));

	//in this setting, projecting the vector on the generalized coord axes is just a matter of taking dot products...
	aVal = vec.dot(a);
	bVal = vec.dot(b);
}

void GeneralizedCoordinatesRobotRepresentation::projectVectorOnGeneralizedCoordsAxes(const V3D& vec, const V3D& a, double& aVal) {
	aVal = vec.dot(a);
}

void GeneralizedCoordinatesRobotRepresentation::projectWorldCoordsValuesIntoGeneralizedSpace(const V3D& linearVal, const V3D& angularVal, const DynamicArray<V3D>& jointVal, dVector& generalizedCoordinates) {
	resize(generalizedCoordinates, q.size());
	projectVectorOnGeneralizedCoordsAxes(linearVal, getWorldCoordsAxisForQ(0), getWorldCoordsAxisForQ(1), getWorldCoordsAxisForQ(2), generalizedCoordinates[0], generalizedCoordinates[1], generalizedCoordinates[2]);
	projectVectorOnGeneralizedCoordsAxes(angularVal, getWorldCoordsAxisForQ(3), getWorldCoordsAxisForQ(4), getWorldCoordsAxisForQ(5), generalizedCoordinates[3], generalizedCoordinates[4], generalizedCoordinates[5]);

	//now go through each joint, and decompose it as appropriate...
	for (uint i = 0; i<robot->jointList.size(); i++) {
		int startIndex = jointCoordStartIndex[i];
		if (BallAndSocketJoint* j = dynamic_cast<BallAndSocketJoint*>(robot->jointList[i])) {
			projectVectorOnGeneralizedCoordsAxes(jointVal[i], getWorldCoordsAxisForQ(startIndex + 0), getWorldCoordsAxisForQ(startIndex + 1), getWorldCoordsAxisForQ(startIndex + 2), generalizedCoordinates[startIndex + 0], generalizedCoordinates[startIndex + 1], generalizedCoordinates[startIndex + 2]);
			continue;
		}
		if (UniversalJoint* j = dynamic_cast<UniversalJoint*>(robot->jointList[i])) {
			projectVectorOnGeneralizedCoordsAxes(jointVal[i], getWorldCoordsAxisForQ(startIndex + 0), getWorldCoordsAxisForQ(startIndex + 1), generalizedCoordinates[startIndex + 0], generalizedCoordinates[startIndex + 1]);
			continue;
		}
		if (HingeJoint* j = dynamic_cast<HingeJoint*>(robot->jointList[i])) {
			projectVectorOnGeneralizedCoordsAxes(jointVal[i], getWorldCoordsAxisForQ(startIndex + 0), generalizedCoordinates[startIndex + 0]);
			continue;
		}
		//not sure what other types of jointList there are...
		assert(false);
	}
}

//returns the world coordinates for point p, which is specified in the local coordinates of rb (relative to its COM): p(q)
P3D GeneralizedCoordinatesRobotRepresentation::getWorldCoordinatesFor(const P3D& p, RigidBody* rb) {
	return P3D(getWorldCoordinatesForPointT(p, rb, q));
}

//TODO: we will also want derivatives of v(q) (vector to world) e.g. dv/dq and d2v/dqq2s

//returns the world coordinates for vector b, which is specified in the local coordinates of rb: v(q)
V3D GeneralizedCoordinatesRobotRepresentation::getWorldCoordinatesFor(const V3D& v, RigidBody* rb) {
	return V3D(getWorldCoordinatesForVectorT(v, rb, q));
}

//returns the velocity (world coordinates) of the point p, which is specified in the local coordinates of rb (relative to its COM). I.e. p(q)
V3D GeneralizedCoordinatesRobotRepresentation::getVelocityFor(const P3D& p, RigidBody* rb) {
	//pDot(q) = dp/dq * qDot
	MatrixNxM dpdq;
	dVector res;

	compute_dpdq(p, rb, dpdq);
	res = dpdq * qDot;

	return V3D(res[0], res[1], res[2]);
}

V3D GeneralizedCoordinatesRobotRepresentation::getAngularVelocityFor(RigidBody* rb) {
	//w(q) = dR/dq * qDot
	MatrixNxM dRdq;
	dVector res;

	compute_angular_jacobian(rb, dRdq);
	res = dRdq * qDot;

	return V3D(res[0], res[1], res[2]);
}

//returns the world-relative orientation for rb
Quaternion GeneralizedCoordinatesRobotRepresentation::getOrientationFor(RigidBody* rb) {
	//	return Quaternion();
	if (rb->pJoints.size() == 0)
		return getWorldRotationForQ(5);

	int jIndex = rb->pJoints[0]->jIndex;

	int startIndex = jointCoordStartIndex[jIndex];

	int jDim = jointCoordsDimSize[jIndex];

	return getWorldRotationForQ(startIndex + jDim - 1);
}


//computes the jacobian dp/dq that tells you how the world coordinates of p change with q. p is expressed in the local coordinates of rb
void GeneralizedCoordinatesRobotRepresentation::compute_dpdq(const P3D& p, RigidBody* rb, MatrixNxM &dpdq) {
	resize(dpdq, 3, q.size());
	dpdq.fill(0);

	int startIndex = 5;
	if (rb->pJoints.size() != 0)
		startIndex = jointCoordStartIndex[rb->pJoints[0]->jIndex] + jointCoordsDimSize[rb->pJoints[0]->jIndex] - 1;

	int loopIndex = startIndex;
	//2 here is the index of the first translational DOF of the root
	while (loopIndex > 2) {
		V3D offset(p);
		if (rb->pJoints.size() != 0)
			offset = V3D(rb->pJoints[0]->cJPos, p);
		int qIndex = startIndex;

		while (qIndex > loopIndex) {
			offset = getOffsetFromParentToQ(qIndex) + offset.rotate(q[qIndex], getQAxis(qIndex));
			qIndex = qParentIndex[qIndex];
		}

		offset = getQAxis(qIndex).cross(offset);

		while (qIndex > 2) {
			offset = offset.rotate(q[qIndex], getQAxis(qIndex));
			qIndex = qParentIndex[qIndex];
		}

		for (int i = 0; i<3; i++)
			dpdq(i, loopIndex) = offset[i];

		loopIndex = qParentIndex[loopIndex];
	}

	dpdq(0, 0) = 1;
	dpdq(1, 1) = 1;
	dpdq(2, 2) = 1;
}

//computes the jacobian dp/dq that tells you how the world coordinates of p change with q. p is expressed in the local coordinates of rb
void GeneralizedCoordinatesRobotRepresentation::compute_dvdq(const V3D& v, RigidBody* rb, MatrixNxM &dvdq) {
	resize(dvdq, 3, q.size());
	dvdq.fill(0);

	int startIndex = 5;
	if (rb->pJoints.size() != 0)
		startIndex = jointCoordStartIndex[rb->pJoints[0]->jIndex] + jointCoordsDimSize[rb->pJoints[0]->jIndex] - 1;

	int loopIndex = startIndex;
	//2 here is the index of the first translational DOF of the root
	while (loopIndex > 2) {
		V3D theVector(v);
		int qIndex = startIndex;

		while (qIndex > loopIndex) {
			theVector = theVector.rotate(q[qIndex], getQAxis(qIndex));
			qIndex = qParentIndex[qIndex];
		}

		theVector = getQAxis(qIndex).cross(theVector);

		while (qIndex > 2) {
			theVector = theVector.rotate(q[qIndex], getQAxis(qIndex));
			qIndex = qParentIndex[qIndex];
		}

		for (int i = 0; i<3; i++)
			dvdq(i, loopIndex) = theVector[i];

		loopIndex = qParentIndex[loopIndex];
	}
}

//estimates the linear jacobian dp/dq using finite differences
void GeneralizedCoordinatesRobotRepresentation::estimate_linear_jacobian(const P3D& p, RigidBody* rb, MatrixNxM &dpdq) {
	resize(dpdq, 3, (int)q.size());

	for (int i = 0; i<q.size(); i++) {
		double val = q[i];
		double h = 0.0001;

		q[i] = val + h;
		P3D p_p = getWorldCoordinatesFor(p, rb);

		q[i] = val - h;
		P3D p_m = getWorldCoordinatesFor(p, rb);

		q[i] = val;
		V3D dpdq_i = V3D(p_m, p_p) / (2 * h);
		dpdq(0, i) = dpdq_i[0];
		dpdq(1, i) = dpdq_i[1];
		dpdq(2, i) = dpdq_i[2];
	}
}

//estimates the jacobian dv/dq using finite differences
void GeneralizedCoordinatesRobotRepresentation::estimate_linear_jacobian(const V3D& v, RigidBody* rb, MatrixNxM &dvdq) {
	resize(dvdq, 3, (int)q.size());

	for (int i = 0; i<q.size(); i++) {
		double val = q[i];
		double h = 0.0001;

		q[i] = val + h;
		V3D p_p = getWorldCoordinatesFor(v, rb);

		q[i] = val - h;
		V3D p_m = getWorldCoordinatesFor(v, rb);

		q[i] = val;
		V3D dvdq_i = (p_p - p_m) / (2 * h);
		dvdq(0, i) = dvdq_i[0];
		dvdq(1, i) = dvdq_i[1];
		dvdq(2, i) = dvdq_i[2];
	}
}

bool GeneralizedCoordinatesRobotRepresentation::test_linear_jacobian(const P3D& p, RigidBody* rb) {
	MatrixNxM dpdq_analytic, dpdq_estimated;
	compute_dpdq(p, rb, dpdq_analytic);
	estimate_linear_jacobian(p, rb, dpdq_estimated);

//	print("../out/dpdq_analytic.mat", dpdq_analytic);
//	print("../out/dpdq_estimated.mat", dpdq_estimated);

	bool error = false;

	for (int i = 0; i<dpdq_analytic.rows(); i++)
		for (int j = 0; j<dpdq_analytic.cols(); j++) {
			double err = dpdq_analytic(i, j) - dpdq_estimated(i, j);
			if (fabs(err) > 0.0001) {
				Logger::consolePrint("error at: %d %d: analytic: %lf estimated %lf error: %lf\n", i, j, dpdq_analytic(i, j), dpdq_estimated(i, j), err);
				error = true;
			}
		}

	return !error;
}

bool GeneralizedCoordinatesRobotRepresentation::test_linear_jacobian(const V3D& v, RigidBody* rb) {
	MatrixNxM dvdq_analytic, dvdq_estimated;
	compute_dvdq(v, rb, dvdq_analytic);
	estimate_linear_jacobian(v, rb, dvdq_estimated);

	//	print("../out/dpdq_analytic.mat", dpdq_analytic);
	//	print("../out/dpdq_estimated.mat", dpdq_estimated);

	bool error = false;

	for (int i = 0; i<dvdq_analytic.rows(); i++)
		for (int j = 0; j<dvdq_analytic.cols(); j++) {
			double err = dvdq_analytic(i, j) - dvdq_estimated(i, j);
			if (fabs(err) > 0.0001) {
				Logger::consolePrint("error at: %d %d: analytic: %lf estimated %lf error: %lf\n", i, j, dvdq_analytic(i, j), dvdq_estimated(i, j), err);
				error = true;
			}
		}

	return !error;
}

//computes the matrix that tells you how the jacobian dp/dq changes with respect to q_i. Returns true if it contains non-zero elements, false otherwise
bool GeneralizedCoordinatesRobotRepresentation::compute_ddpdq_dqi(const P3D& p, RigidBody* rb, MatrixNxM &ddpdq_dqi, int q_i) {
	resize(ddpdq_dqi, 3, (int)q.size());

	int startIndex = 5;
	if (rb->pJoints.size() != 0)
		startIndex = jointCoordStartIndex[rb->pJoints[0]->jIndex] + jointCoordsDimSize[rb->pJoints[0]->jIndex] - 1;

	//if q_i is not one of the ancestors of rb, then it means the jacobian dpdq does not depent on it, so check first...
	int qIndex = startIndex;
	bool isAncestor = false;
	while (qIndex > 2) {
		if (q_i == qIndex) {
			isAncestor = true;
			break;
		}
		qIndex = qParentIndex[qIndex];
	}

	if (!isAncestor) return false;

	//input is valid, so we must compute this derivative...
	int loopIndex = startIndex;
	//2 here is the index of the first translational DOF of the root
	while (loopIndex > 2) {
		V3D offset(p);
		if (rb->pJoints.size() != 0)	offset = V3D(rb->pJoints[0]->cJPos, p);
		int qIndex = startIndex;

		int stopIndex = loopIndex;
		if (q_i > loopIndex) stopIndex = q_i;

		while (qIndex > stopIndex) {
			offset = getOffsetFromParentToQ(qIndex) + offset.rotate(q[qIndex], getQAxis(qIndex));
			qIndex = qParentIndex[qIndex];
		}

		while (qIndex > 2) {
			if (qIndex == loopIndex)
				offset = getQAxis(qIndex).cross(offset);
			if (qIndex == q_i)
				offset = getQAxis(qIndex).cross(offset);

			offset = offset.rotate(q[qIndex], getQAxis(qIndex));
			qIndex = qParentIndex[qIndex];
		}

		for (int i = 0; i<3; i++)
			ddpdq_dqi(i, loopIndex) = offset[i];

		loopIndex = qParentIndex[loopIndex];
	}

	return true;
}

//computes the matrix that tells you how the jacobian dv/dq changes with respect to q_i. Returns true if it contains non-zero elements, false otherwise
bool GeneralizedCoordinatesRobotRepresentation::compute_ddvdq_dqi(const V3D& v, RigidBody* rb, MatrixNxM &ddvdq_dqi, int q_i) {
	resize(ddvdq_dqi, 3, (int)q.size());

	int startIndex = 5;
	if (rb->pJoints.size() != 0)
		startIndex = jointCoordStartIndex[rb->pJoints[0]->jIndex] + jointCoordsDimSize[rb->pJoints[0]->jIndex] - 1;

	//if q_i is not one of the ancestors of rb, then it means the jacobian dpdq does not depent on it, so check first...
	int qIndex = startIndex;
	bool isAncestor = false;
	while (qIndex > 2) {
		if (q_i == qIndex) {
			isAncestor = true;
			break;
		}
		qIndex = qParentIndex[qIndex];
	}

	if (!isAncestor) return false;

	//input is valid, so we must compute this derivative...
	int loopIndex = startIndex;
	//2 here is the index of the first translational DOF of the root
	while (loopIndex > 2) {
		V3D theVector(v);
		int qIndex = startIndex;

		int stopIndex = loopIndex;
		if (q_i > loopIndex) stopIndex = q_i;

		while (qIndex > stopIndex) {
			theVector = theVector.rotate(q[qIndex], getQAxis(qIndex));
			qIndex = qParentIndex[qIndex];
		}

		while (qIndex > 2) {
			if (qIndex == loopIndex)
				theVector = getQAxis(qIndex).cross(theVector);
			if (qIndex == q_i)
				theVector = getQAxis(qIndex).cross(theVector);

			theVector = theVector.rotate(q[qIndex], getQAxis(qIndex));
			qIndex = qParentIndex[qIndex];
		}

		for (int i = 0; i<3; i++)
			ddvdq_dqi(i, loopIndex) = theVector[i];

		loopIndex = qParentIndex[loopIndex];
	}

	return true;
}

//estimates the change of dp/dq with respect to q_i
void GeneralizedCoordinatesRobotRepresentation::estimate_ddpdq_dqi(const P3D& p, RigidBody* rb, MatrixNxM &ddpdq_dqi, int q_i) {
	resize(ddpdq_dqi, 3, (int)q.size());
	MatrixNxM dpdq_p, dpdq_m;
	dpdq_p = ddpdq_dqi;	dpdq_m = ddpdq_dqi;

	double val = q[q_i];
	double h = 0.0001;

	q[q_i] = val + h;
	compute_dpdq(p, rb, dpdq_p);

	q[q_i] = val - h;
	compute_dpdq(p, rb, dpdq_m);

	q[q_i] = val;

	ddpdq_dqi = (dpdq_p - dpdq_m) / (2 * h);
}

//estimates the change of dv/dq with respect to q_i
void GeneralizedCoordinatesRobotRepresentation::estimate_ddvdq_dqi(const V3D& v, RigidBody* rb, MatrixNxM &ddvdq_dqi, int q_i) {
	resize(ddvdq_dqi, 3, (int)q.size());
	MatrixNxM dvdq_p, dvdq_m;
	dvdq_p = ddvdq_dqi;	dvdq_m = ddvdq_dqi;

	double val = q[q_i];
	double h = 0.0001;

	q[q_i] = val + h;
	compute_dvdq(v, rb, dvdq_p);

	q[q_i] = val - h;
	compute_dvdq(v, rb, dvdq_m);

	q[q_i] = val;

	ddvdq_dqi = (dvdq_p - dvdq_m) / (2 * h);
}

bool GeneralizedCoordinatesRobotRepresentation::test_linear_jacobian_derivatives(const P3D& p, RigidBody* rb) {
	MatrixNxM dpdq_analytic, dpdq_estimated;

	bool error = false;

	for (int k = 0; k<getDimensionCount(); k++) {

		compute_ddpdq_dqi(p, rb, dpdq_analytic, k);
		estimate_ddpdq_dqi(p, rb, dpdq_estimated, k);

		//		dpdq_analytic.printMatrix("out\\dpdq.mat");

		for (int i = 0; i<dpdq_analytic.rows(); i++)
			for (int j = 0; j<dpdq_analytic.cols(); j++) {
				double err = dpdq_analytic(i, j) - dpdq_estimated(i, j);
				if (fabs(err) > 0.0001) {
					Logger::consolePrint("error when computing ddpdq_dq%d at: %d %d: analytic: %lf estimated %lf error: %lf\n", k, i, j, dpdq_analytic(i, j), dpdq_estimated(i, j), err);
					error = true;
				}
			}
	}

	return !error;
}

bool GeneralizedCoordinatesRobotRepresentation::test_linear_jacobian_derivatives(const V3D& v, RigidBody* rb) {
	MatrixNxM dvdq_analytic, dvdq_estimated;

	bool error = false;

	for (int k = 0; k<getDimensionCount(); k++) {

		compute_ddvdq_dqi(v, rb, dvdq_analytic, k);
		estimate_ddvdq_dqi(v, rb, dvdq_estimated, k);

		//		dpdq_analytic.printMatrix("out\\dpdq.mat");

		for (int i = 0; i<dvdq_analytic.rows(); i++)
			for (int j = 0; j<dvdq_analytic.cols(); j++) {
				double err = dvdq_analytic(i, j) - dvdq_estimated(i, j);
				if (fabs(err) > 0.0001) {
					Logger::consolePrint("error when computing ddpdq_dq%d at: %d %d: analytic: %lf estimated %lf error: %lf\n", k, i, j, dvdq_analytic(i, j), dvdq_estimated(i, j), err);
					error = true;
				}
			}
	}

	return !error;
}

//computes the angular part of the jacobian, that relates changes in the orientation of a link to changes in q
void GeneralizedCoordinatesRobotRepresentation::compute_angular_jacobian(RigidBody* rb, MatrixNxM &dRdq) {
	//the orientation of an RB is obtained by rotating by all the rotation axes up the hierarchy... so the jacobian consists of the world-coords axes
	resize(dRdq, 3, (int)q.size());

	while (rb->pJoints.size() != 0) {
		int jIndex = rb->pJoints[0]->jIndex;
		//q rotates the rigid body (grand...child) about its world coordinate axis... these will be the entries of dp/dq...
		for (int i = 0; i<jointCoordsDimSize[jIndex]; i++) {
			int index = jointCoordStartIndex[jIndex] + jointCoordsDimSize[jIndex] - i - 1;
			V3D dRdq_i = getWorldCoordsAxisForQ(index);
			dRdq(0, index) = dRdq_i[0];
			dRdq(1, index) = dRdq_i[1];
			dRdq(2, index) = dRdq_i[2];
		}
		rb = rb->pJoints[0]->parent;
	}

	for (int index = 3; index<6; index++) {
		V3D dRdq_i = getWorldCoordsAxisForQ(index);
		dRdq(0, index) = dRdq_i[0];
		dRdq(1, index) = dRdq_i[1];
		dRdq(2, index) = dRdq_i[2];
	}
}

//estimates the angular jacobian using finite differences
void GeneralizedCoordinatesRobotRepresentation::estimate_angular_jacobian(RigidBody* rb, MatrixNxM &dRdq){
	resize(dRdq, 3, (int)q.size());

	for (int i = 0; i<q.size(); i++) {
		double val = q[i];
		double h = 0.001;

		q[i] = val + h;
		updateWorldOrientations();
		Quaternion R_p = getOrientationFor(rb);

		q[i] = val - h;
		updateWorldOrientations();
		Quaternion R_m = getOrientationFor(rb);

		q[i] = val;
		updateWorldOrientations();
		V3D axis;
		double angle;
		Quaternion rotate = R_p * R_m.getInverse();
		rotate.getAxisAngle(axis, angle);
		axis.normalize();
		axis *= angle;

		V3D dRdq_i = axis / (2 * h);
		dRdq(0, i) = dRdq_i[0];
		dRdq(1, i) = dRdq_i[1];
		dRdq(2, i) = dRdq_i[2];
	}
}

bool GeneralizedCoordinatesRobotRepresentation::test_angular_jacobian(RigidBody* rb){
	MatrixNxM dRdq_analytic, dRdq_estimated;
	compute_angular_jacobian(rb, dRdq_analytic);
	estimate_angular_jacobian(rb, dRdq_estimated);

//	print("../out/angular_jacobian_analytic.mat", dRdq_analytic);
//	print("../out/angular_jacobian_estimated.mat", dRdq_estimated);

	bool error = false;

	for (int i = 0; i<dRdq_analytic.rows(); i++)
		for (int j = 0; j<dRdq_analytic.cols(); j++) {
			double err = dRdq_analytic(i, j) - dRdq_estimated(i, j);
			if (fabs(err) > 0.0001) {
				Logger::consolePrint("error at: %d %d: analytic: %lf estimated %lf error: %lf\n", i, j, dRdq_analytic(i, j), dRdq_estimated(i, j), err);
				error = true;
			}
		}

	return !error;
}

//computes the d(Jw)/dqi
bool GeneralizedCoordinatesRobotRepresentation::compute_dangular_jacobian_dqi(RigidBody* rb, MatrixNxM &ddRdqdqi, int q_i){
	resize(ddRdqdqi, 3, (int)q.size());
	if (q_i < 3) return false;

	int startIndex = 5;
	if (rb->pJoints.size() != 0)
		startIndex = jointCoordStartIndex[rb->pJoints[0]->jIndex] + jointCoordsDimSize[rb->pJoints[0]->jIndex] - 1;

	//if q_i is not one of the ancestors of rb, then it means the jacobian dpdq does not depent on it, so check first...
	int qIndex = startIndex;
	bool isAncestor = false;
	while (qIndex > 2) {
		if (q_i == qIndex) {
			isAncestor = true;
			break;
		}
		qIndex = qParentIndex[qIndex];
	}

	if (!isAncestor) return false;

	//input is valid, so we must compute this derivative...
	int loopIndex = startIndex;
	//2 here is the index of the first translational DOF of the root
	while (loopIndex > q_i) {
//		Logger::consolePrint("NEW: adding contribution for index: %d\n", loopIndex);
		V3D derivativeVal = getWorldCoordsAxisForQ(q_i).cross(getWorldCoordsAxisForQ(loopIndex));
		ddRdqdqi(0, loopIndex) = derivativeVal(0);
		ddRdqdqi(1, loopIndex) = derivativeVal(1);
		ddRdqdqi(2, loopIndex) = derivativeVal(2);

		loopIndex = qParentIndex[loopIndex];
	}

	return true;
}

//estimates the change of angular jacobian with respect to q_i using finite differences
void GeneralizedCoordinatesRobotRepresentation::estimate_dangular_jacobian_dqi(RigidBody* rb, MatrixNxM &ddRdq_dqi, int q_i) {
	resize(ddRdq_dqi, 3, (int)q.size());
	MatrixNxM dRdq_p, dRdq_m;
	dRdq_p = ddRdq_dqi;	dRdq_m = ddRdq_dqi;

	double val = q[q_i];
	double h = 0.0001;

	q[q_i] = val + h;
	updateWorldOrientations();
	compute_angular_jacobian(rb, dRdq_p);

	q[q_i] = val - h;
	updateWorldOrientations();
	compute_angular_jacobian(rb, dRdq_m);

	q[q_i] = val;
	updateWorldOrientations();

	ddRdq_dqi = (dRdq_p - dRdq_m) / (2 * h);
}

bool GeneralizedCoordinatesRobotRepresentation::test_angular_jacobian_derivatives(RigidBody* rb){
	MatrixNxM ddRdqdq_analytic, ddRdqdq_estimated;

	bool error = false;

	for (int k = 0;k<getDimensionCount();k++) {
		compute_dangular_jacobian_dqi(rb, ddRdqdq_analytic, k);
		estimate_dangular_jacobian_dqi(rb, ddRdqdq_estimated, k);

//		print("../out/angular_jacobian_dqi_analytic.mat", ddRdqdq_analytic);
//		print("../out/angular_jacobian_dqi_estimated.mat", ddRdqdq_estimated);

		for (int i = 0; i<ddRdqdq_analytic.rows(); i++)
			for (int j = 0; j<ddRdqdq_analytic.cols(); j++) {
				double err = ddRdqdq_analytic(i, j) - ddRdqdq_estimated(i, j);
				if (fabs(err) > 0.0001) {
					Logger::consolePrint("error when computing ddpdq_dq%d at: %d %d: analytic: %lf estimated %lf error: %lf\n", k, i, j, ddRdqdq_analytic(i, j), ddRdqdq_estimated(i, j), err);
					error = true;
//					exit(0);
				}
			}
	}

	return !error;
}


//computes the generalized mass matrix for rigidbody rb: M = J'McJ, where Mc is a 6x6, local coordinates matrix, and M is a |q|x|q| generalized matrix...
void GeneralizedCoordinatesRobotRepresentation::computeMassMatrixForRB(RigidBody* rb, MatrixNxM &massMatrix){
	/*
	M = J'McJ
	Mc = [	m 0 0  0
			0 m 0  0
			0 0 m  0
			0 0 0 MoI]
	*/
	resize(massMatrix, q.size(), q.size());
	
	Matrix3x3 MoI;
	MatrixNxM dRdq, dpdq;

	//TODO: hmmm, should the MOI be in local or global coordinates?
	MoI.setZero();
	MoI = rb->getWorldMOI(getOrientationFor(rb));

	compute_dpdq(P3D(0, 0, 0), rb, dpdq);
	compute_angular_jacobian(rb, dRdq);

	massMatrix.noalias() = rb->rbProperties.mass * dpdq.transpose() * dpdq;
	massMatrix.noalias() += dRdq.transpose() * MoI * dRdq;
}

//computes the mass matrix for the whole robot
void GeneralizedCoordinatesRobotRepresentation::computeMassMatrix(MatrixNxM &massMatrix){
	resize(massMatrix, q.size(), q.size());

	computeMassMatrixForRB(robot->root, massMatrix);

	MatrixNxM curMassMatrix;
	for (uint i = 0; i < robot->jointList.size(); ++i){
		computeMassMatrixForRB(robot->jointList[i]->child, curMassMatrix);
		massMatrix += curMassMatrix;
	}
}

//computes dpdq_dot, dpdq_dot = sigma(dpdq_dqi * qiDot): JDot = dJ/dq * qDot
void GeneralizedCoordinatesRobotRepresentation::compute_dpdq_dot(const P3D& p, RigidBody* rb, MatrixNxM &dpdq_dot){
	MatrixNxM ddpdq_dqi;

	resize(dpdq_dot, 3, (int)q.size());
	dpdq_dot.setZero();
	for (int i = 0; i < (int)q.size(); ++i){
		compute_ddpdq_dqi(p, rb, ddpdq_dqi, i);
		dpdq_dot += ddpdq_dqi * qDot[i];
	}
}

//computed dRdq_dot, dRdq_dot = sigma(dRdq_dqi * qiDot): JDot = dJ/dq * qDot
void GeneralizedCoordinatesRobotRepresentation::compute_angular_jacobian_dot(RigidBody* rb, MatrixNxM &dRdq_dot){
	MatrixNxM ddRdq_dqi;

	resize(dRdq_dot, 3, (int)q.size());
	dRdq_dot.setZero();

	for (int i = 0; i < (int)q.size(); ++i){
		compute_dangular_jacobian_dqi(rb, ddRdq_dqi, i);
		dRdq_dot += ddRdq_dqi * qDot[i];
	}
}

//computes the Coriolis Vector for the whole robot
void GeneralizedCoordinatesRobotRepresentation::computeCoriolisAndCentrifugalForcesTerm(dVector &C) {
	MatrixNxM coriolisMatrix;
	resize(coriolisMatrix, (int)q.size(), (int)q.size());

	MatrixNxM curCoriolisMatrix;
	computeCoriolisMatrix(robot->root, coriolisMatrix);
	for (uint i = 0; i < robot->jointList.size(); ++i) {
		computeCoriolisMatrix(robot->jointList[i]->child, curCoriolisMatrix);
		coriolisMatrix += curCoriolisMatrix;
	}
	C = coriolisMatrix * qDot;
}

//computes the rigidbody rb's contribution (corioli and centrifugal force part) to the coriolisMatrix term, CMatrix(q, qDot) = (J'McJDot + J'[w]McJ)
void GeneralizedCoordinatesRobotRepresentation::computeCoriolisMatrix(RigidBody* rb, MatrixNxM &coriolisMatrix){
    /*
        [w] = [0 0
               0 [Jw*qDot] ]
    */
    resize(coriolisMatrix, (int)q.size(), (int)q.size());
    coriolisMatrix.setZero();

    // calculate linear part contribution
	MatrixNxM dpdq, dpdq_dot;
    compute_dpdq(P3D(0, 0, 0), rb, dpdq);
    compute_dpdq_dot(P3D(0, 0, 0), rb, dpdq_dot);

    coriolisMatrix.noalias() = rb->rbProperties.mass * dpdq.transpose() * dpdq_dot;
 //   print("dpdq_dot", dpdq_dot);
    // calculate angular part contribution
	MatrixNxM dRdq, dRdq_dot, omega;
	Matrix3x3 MoI = rb->getWorldMOI(getOrientationFor(rb));
    compute_angular_jacobian(rb, dRdq);
    compute_angular_jacobian_dot(rb, dRdq_dot);
    omega = V3D(dRdq * qDot).getSkewSymmetricMatrix();
    coriolisMatrix.noalias() += (dRdq.transpose() * MoI * dRdq_dot + dRdq.transpose() * omega * MoI * dRdq);
}
















