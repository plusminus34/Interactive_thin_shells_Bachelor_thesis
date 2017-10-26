#pragma once

#include <MathLib/MathLib.h>
#include <MathLib/P3D.h>
#include <MathLib/V3D.h>
#include <MathLib/MathLib.h>
#include <MathLib/Trajectory.h>
#include <ControlLib/Robot.h>
#include <ControlLib/GeneralizedCoordinatesRobotRepresentation.h>
#include <vector>
#include <RBSimLib/HingeJoint.h>

template <typename T> T getTargetAcceleration_explicitPD(const T& posError, const T& velError, double kp, double kd) {
	return -posError * kp - velError * kd;
}

template <typename T> T getTargetAcceleration_implicitPD(const T& curPosError, const T& curVelError, double kp, double kd, double dt) {
	//what we know:
	//	P_t+1 = P_t + h * V_t+1
	//	V_t+1 = V_t + h * a
	//	a = -kp * P_t+1 - kd * V_t+1
	//	plug the expressions for P_t+1 and V_t+1 into a and solve...
	return -(curPosError * kp + curVelError * kd + curVelError * dt * kp) / (1 + dt * dt * kp + dt * kd);
}

template <typename T> T getTargetAcceleration_posConstraint(const T& targetPos, const T& curPos, const T& curVel, double dt) {
	//what we know:
	//	P_t+1 = P_t + h * V_t+1
	//	V_t+1 = V_t + h * a
	//if we want P_t+1 to be targetPos (e.g. position error vanishes)
	//then a = (targetPos - P_t - h * vt) / (h * h)
	return (targetPos - curPos - curVel * dt) / (dt * dt);
}

class QPControl_EndEffector {
public:

	QPControl_EndEffector() {
		endEffectorRB = NULL;
	}

	QPControl_EndEffector(RigidBody* endEffectorRB, const P3D& endEffectorLocalCoords) {
		this->endEffectorRB = endEffectorRB;
		this->endEffectorLocalCoords = endEffectorLocalCoords;
	}

	//this is the rigid body that holds the end effector
	RigidBody* endEffectorRB;
	//and this holds the position of the end effector in local coordinates
	P3D endEffectorLocalCoords;

	//this is the world coords target acceleration that we have for this end effector
	V3D targetEEAcceleration;

	P3D targetPosition;

	P3D getWorldCoordsPosition() {
		return endEffectorRB->getWorldCoordinates(endEffectorLocalCoords);
	}

	void updateJacobians(GeneralizedCoordinatesRobotRepresentation* robotRepresentation) {
		robotRepresentation->compute_dpdq(endEffectorLocalCoords, endEffectorRB, J);
		robotRepresentation->compute_dpdq_dot(endEffectorLocalCoords, endEffectorRB, Jdot);
	}

	//we will cache the end effector Jacobian and its time derivative (should be recomputed whenever the robot state changes)...
	MatrixNxM J;
	MatrixNxM Jdot;
};

/**
	End effectors in contact have special semantic meaning, as this is where ground reaction forces can be modulated
*/
class QPControl_ContactEndEffector : public QPControl_EndEffector {
public:
	//world coordinates of the contact force being applied at the end effector - this is one of the unknowns we solve for
	V3D contactForce;

	bool inContact;

	//friction coefficient...
	double frictionCoeff = 0.7;

	QPControl_ContactEndEffector() {

	}

	QPControl_ContactEndEffector(RigidBody* endEffectorRB, const P3D& endEffectorLocalCoords) : QPControl_EndEffector(endEffectorRB, endEffectorLocalCoords) {

	}

};

/**
	This is a motion plan for an arbitrary robot model
*/
class QPControlPlan{
	friend class QPC_NetBodyAccelerationObjective;
	friend class QPControlConstraints;
	friend class QPControlEnergyFunction;
public:
	QPControlPlan(Robot* robot);

	virtual ~QPControlPlan(void);

public:
	int groundReactionForcesParamsStartIndex;
	int generalizedAccelerationsParamsStartIndex;
	int jointTorquesParamsStartIndex;
	int paramCount;

	double maxBodyLinearAccelerationTarget = 50; //m/s/s
	double maxBodyAngularAccelerationTarget = 50; //rad/s/s
	double maxJointAngularAccelerationTarget = 50; //rad/s/s
	double maxEndEffectorAccelerationTarget = 25; //m/s/s

public:
	dVector targetGeneralizedAccelerations;
	DynamicArray<bool> dofUsedInSwingLimbs;

	//used just for debugging purposes...
	P3D startingBodyPos;
	V3D startingBodyVel;
	P3D predictedBodyPos;
	V3D predictedBodyVel;
	P3D targetBodyPos;
	V3D targetBodyVel;
	V3D targetBodyAccel;


private:
	//these quantities depend only on q/qDot, so they can be precomputed before each control step
	MatrixNxM M;
	//sum of gravitational forces applied at each body part of the robot...
	dVector gravitationalForces;
	//term that combines coriolis and centrifugal forces/inertial effects
	dVector C;

	dVector qDotTmp, qDotDotTmp;
	MatrixNxM tmpA;
	dVector tmpRHS, tmpV;

public:
	DynamicArray<QPControl_ContactEndEffector> contactEndEffectors;
	DynamicArray<QPControl_EndEffector> generalEndEffectors;
	//end effectors of swinging limbs need to be concerned with reaching their target placement, 
	//not with helping the body move in the right direction. Similarly, the body should be doing 
	//its own thing, rather than moving so as to "help" the swing limbs (if falling, swing limbs 
	//should be moving in the direction of the fall, but the body should try to resist it, if 
	//possible). For this reason, swing limb end effectors will receive a special treatment 
	DynamicArray<QPControl_EndEffector> swingLimbEndEffectors;

	void computeSwingLimbJointAccelerationTargets();

	//the degrees of freedom we optimize for are generalized accelerations and generalized forces/torques
	dVector a;		//generalized accelerations
	dVector u;		//generalized forces/torques

	double dt = 1 / 120.0;

	Robot* robot;
	GeneralizedCoordinatesRobotRepresentation* robotRepresentation;

	void updateParameterStartIndices(){
		paramCount = 0;
		groundReactionForcesParamsStartIndex = generalizedAccelerationsParamsStartIndex = jointTorquesParamsStartIndex = -1;

		groundReactionForcesParamsStartIndex = paramCount;
		paramCount += 3 * contactEndEffectors.size();

		generalizedAccelerationsParamsStartIndex = paramCount;
		paramCount += robotRepresentation->getDimensionCount();

		jointTorquesParamsStartIndex = paramCount;
		paramCount += robotRepresentation->getDimensionCount() - 6;	//only optimizing for joint torques, and not forces/torques acting on the root
	}

	virtual void readParametersFromFile(char* fName) {
		dVector p;
		writeParametersToList(p);
		FILE* fp = fopen(fName, "r");
		for (int i = 0; i < p.size(); i++)
			fscanf(fp, "%lf", &p[i]);
		fclose(fp);
		setParametersFromList(p);
	}

	//if minV is equal to maxV, then there are no bounds for that variable...
	virtual void getParameterMinValues(dVector& minV){
		std::vector<double> minLimits;

		//ground reaction forces at end effectors in contact
		for (uint i = 0; i < 3 * contactEndEffectors.size(); i++)
			minLimits.push_back(0);

		//generalized accelerations
		for (int i = 0; i < robotRepresentation->getDimensionCount(); i++)
			minLimits.push_back(-150);

		//generalized forces
		for (int i = 6; i < robotRepresentation->getDimensionCount(); i++)
			minLimits.push_back(0);

		resize(minV, minLimits.size());
		for (int i = 0;i < minV.size();i++)
			minV[i] = minLimits[i];
	}

	//if minV is equal to maxV, then there are no bounds for that variable...
	virtual void getParameterMaxValues(dVector& maxV){
		std::vector<double> maxLimits;

		//ground reaction forces at end effectors in contact
		for (uint i = 0; i < contactEndEffectors.size(); i++) {
			maxLimits.push_back(0);
			maxLimits.push_back(10000);
			maxLimits.push_back(0);
		}

		//generalized accelerations
		for (int i = 0; i < robotRepresentation->getDimensionCount(); i++)
			maxLimits.push_back(150);

		//generalized forces
		for (int i = 6; i < robotRepresentation->getDimensionCount(); i++)
			maxLimits.push_back(0);

		resize(maxV, maxLimits.size());
		for (int i = 0;i < maxV.size();i++)
			maxV[i] = maxLimits[i];
	}

	virtual void writeParametersToList(dVector& p){
		updateParameterStartIndices();
		std::vector<double> params;

		//ground reaction forces at end effectors in contact
		for (uint i = 0; i < contactEndEffectors.size(); i++)
			for (int j = 0; j < 3;j++)
				params.push_back(contactEndEffectors[i].contactForce[j]);

		//generalized accelerations
		for (int i = 0; i < robotRepresentation->getDimensionCount(); i++)
			params.push_back(a[i]);

		//generalized forces
		for (int i = 6; i < robotRepresentation->getDimensionCount(); i++)
			params.push_back(u[i]);

		resize(p, params.size());
		for (int i = 0;i < p.size();i++)
			p[i] = params[i];
	}

	virtual void setParametersFromList(const dVector& p){
		int pIndex = 0;

		//ground reaction forces at end effectors in contact
		for (uint i = 0; i < contactEndEffectors.size(); i++)
			for (int j = 0; j < 3; j++)
				contactEndEffectors[i].contactForce[j] = p[pIndex++];

		//generalized accelerations
		for (int i = 0; i < robotRepresentation->getDimensionCount(); i++)
			a[i] = p[pIndex++];

		//generalized forces
		for (int i = 6; i < robotRepresentation->getDimensionCount(); i++)
			u[i] = p[pIndex++];

	}

	//this method should be called before the qp solver is invoked...
	virtual void initializeControlStep();

	virtual void setBodyLinearAccelerationTarget(const V3D& blaTarget) {
		for (int i=0;i<3;i++){
			targetGeneralizedAccelerations[i] = blaTarget[i];
			boundToRange(targetGeneralizedAccelerations[i], -maxBodyLinearAccelerationTarget, maxBodyLinearAccelerationTarget);
		}
	}

	virtual void setEndEffectorTargetAcceleration(QPControl_EndEffector* ee, const V3D& eeAcceleration) {
		ee->targetEEAcceleration = eeAcceleration;
		for (int i = 0; i<3; i++)
			boundToRange(ee->targetEEAcceleration[i], -maxEndEffectorAccelerationTarget, maxEndEffectorAccelerationTarget);
	}

	virtual void setEndEffectorPositionAndVelocityTarget(QPControl_EndEffector* ee, const P3D& pos, const V3D& vel, double kp, double kd) {
		V3D posError(pos, ee->endEffectorRB->getWorldCoordinates(ee->endEffectorLocalCoords));
		V3D velError = ee->endEffectorRB->getAbsoluteVelocityForLocalPoint(ee->endEffectorLocalCoords) - vel;
		setEndEffectorTargetAcceleration(ee, getTargetAcceleration_implicitPD(posError, velError, kp, kd, dt));
	}

	virtual void setBodyAngularAccelerationTarget(const V3D& baaTarget) {
		robotRepresentation->projectVectorOnGeneralizedCoordsAxes(baaTarget, robotRepresentation->getWorldCoordsAxisForQ(3), robotRepresentation->getWorldCoordsAxisForQ(4), robotRepresentation->getWorldCoordsAxisForQ(5), targetGeneralizedAccelerations[3], targetGeneralizedAccelerations[4], targetGeneralizedAccelerations[5]);
		for (int i = 3; i<5; i++)
			boundToRange(targetGeneralizedAccelerations[i], -maxBodyAngularAccelerationTarget, maxBodyAngularAccelerationTarget);
	}

	virtual void setBodyOrientationAndAngularVelocityTarget(const Quaternion& orientation, const V3D& angVel, double kp, double kd) {
		V3D bodyOrientationError = estimateAngularVelocity(robot->root->getOrientation(), orientation, dt) * dt * -1;
		V3D bodyAngularVelError = robot->root->getAngularVelocity() - angVel;
		V3D desBodyAngAccel = getTargetAcceleration_implicitPD(bodyOrientationError, bodyAngularVelError, kp, kd, dt);
		setBodyAngularAccelerationTarget(desBodyAngAccel);
	}

	virtual void setBodyPositionAndVelocityTarget(const P3D& pos, const V3D& vel, double kpTangential, double kdTangential, double kpVertical, double kdVertical) {

		V3D bodyPosError(pos, robot->root->getCMPosition());
		V3D bodyVelError = robot->root->getCMVelocity() - vel;
		V3D desBodyAccel = getTargetAcceleration_implicitPD(bodyPosError, bodyVelError, kpTangential, kdTangential, dt);
		V3D desBodyAccelV = getTargetAcceleration_implicitPD(bodyPosError, bodyVelError, kpVertical, kdVertical, dt);
		desBodyAccel.y() = desBodyAccelV.y();

		setBodyLinearAccelerationTarget(desBodyAccel);

		startingBodyPos = robot->root->getCMPosition();
		startingBodyVel = robot->root->getCMVelocity();
		targetBodyPos = pos;
		targetBodyVel = vel;
		for (int i = 0; i < 3; i++)
			targetBodyAccel[i] = targetGeneralizedAccelerations[i];
		predictedBodyVel = startingBodyVel + targetBodyAccel * dt;
		predictedBodyPos = startingBodyPos + predictedBodyVel * dt;

//		Logger::consolePrint("  current vel: %lf %lf %lf\n", startingBodyVel[0], startingBodyVel[1], startingBodyVel[2]);
//		Logger::consolePrint("   target vel: %lf %lf %lf\n", targetBodyVel[0], targetBodyVel[1], targetBodyVel[2]);
//		Logger::consolePrint("predicted vel: %lf %lf %lf\n", predictedBodyVel[0], predictedBodyVel[1], predictedBodyVel[2]);
	}

	virtual void setJointAngleAccelerationTargets(const dVector& fullBodyAccelerationTarget) {
		for (int i = 6; i < targetGeneralizedAccelerations.size(); i++){
			targetGeneralizedAccelerations[i] = fullBodyAccelerationTarget[i];
			boundToRange(targetGeneralizedAccelerations[i], -maxJointAngularAccelerationTarget, maxJointAngularAccelerationTarget);
		}
	}

	virtual void setJointAngleTargets(const ReducedRobotState& targetState, double kp, double kd) {
		GeneralizedCoordinatesRobotRepresentation qcrr(robot);
		dVector qTarget, qDotTarget;
		qcrr.getQAndQDotFromReducedState(targetState, qTarget, qDotTarget);
		ReducedRobotState currentState(robot);
		dVector qCurrent, qDotCurrent;
		qcrr.getQAndQDotFromReducedState(currentState, qCurrent, qDotCurrent);

		dVector jointAccTarget;
		resize(jointAccTarget, targetGeneralizedAccelerations.size());

		for (int i = 6; i < targetGeneralizedAccelerations.size(); i++)
			jointAccTarget[i] = getTargetAcceleration_implicitPD(qCurrent[i] - qTarget[i], qDotCurrent[i] - qDotTarget[i], kp, kd, dt);

		setJointAngleAccelerationTargets(jointAccTarget);
	}

	void clearEndEffectorList() {
		generalEndEffectors.clear();
		contactEndEffectors.clear();
		swingLimbEndEffectors.clear();
	}

	void addEndEffectorTarget(RigidBody* rb, const P3D& localCoordsEE, const P3D& worldCoordsTargetPos, const V3D& worldCoordsTargetVel, double kp, double kd) {
		generalEndEffectors.push_back(QPControl_EndEffector(rb, localCoordsEE));
		generalEndEffectors.back().updateJacobians(robotRepresentation);
		generalEndEffectors.back().targetPosition = worldCoordsTargetPos;
		setEndEffectorPositionAndVelocityTarget(&generalEndEffectors.back(), worldCoordsTargetPos, worldCoordsTargetVel, kp, kd);

//		Logger::consolePrint("target accel: %lf %lf %lf\n", generalEndEffectors.back().targetEEAcceleration.x(), generalEndEffectors.back().targetEEAcceleration.y(), generalEndEffectors.back().targetEEAcceleration.z());
	}

	void addSwingLimbAccelerationConstraint(RigidBody* rb, const P3D& localCoordsEE, const P3D& worldCoordsTargetPos, const V3D& worldCoordsTargetVel, double kp, double kd) {
		swingLimbEndEffectors.push_back(QPControl_EndEffector(rb, localCoordsEE));
		swingLimbEndEffectors.back().updateJacobians(robotRepresentation);
		swingLimbEndEffectors.back().targetPosition = worldCoordsTargetPos;
		setEndEffectorPositionAndVelocityTarget(&swingLimbEndEffectors.back(), worldCoordsTargetPos, worldCoordsTargetVel, kp, kd);

//		Logger::consolePrint("target accel: %lf %lf %lf\n", generalEndEffectors.back().targetEEAcceleration.x(), generalEndEffectors.back().targetEEAcceleration.y(), generalEndEffectors.back().targetEEAcceleration.z());
	}

/*
	void addEndEffectorTarget(RigidBody* rb, const P3D& localCoordsEE, const P3D& worldCoordsTargetPos) {
		generalEndEffectors.push_back(QPControl_EndEffector(rb, localCoordsEE));
		generalEndEffectors.back().updateJacobians(robotRepresentation);
		generalEndEffectors.back().targetPosition = worldCoordsTargetPos;

		V3D v = rb->getAbsoluteVelocityForLocalPoint(localCoordsEE);

		V3D targetAcc = getTargetAcceleration_posConstraint(V3D(worldCoordsTargetPos), V3D(rb->getWorldCoordinates(localCoordsEE)), v, dt);

		//now we have to limit the acceleration properly, and this will be done as a function of the velocity v_t+1. The idea is that we want the velocity to be small enough that 
		//the maximum acceleration limit will be able to cause a 0 velocity at the next time step.
		V3D futureAcc = -v / dt - targetAcc;
		for (int i = 0; i < 3; i++) {
			boundToRange(futureAcc[i], -maxEndEffectorAccelerationTarget, maxEndEffectorAccelerationTarget);
			targetAcc[i] = -v[i] / dt - futureAcc[i];
		}
		generalEndEffectors.back().targetEEAcceleration = targetAcc;

		Logger::consolePrint("target accel: %lf %lf %lf\n", generalEndEffectors.back().targetEEAcceleration.x(), generalEndEffectors.back().targetEEAcceleration.y(), generalEndEffectors.back().targetEEAcceleration.z());
	}
*/

	void addContactEndEffector(RigidBody* rb, const P3D& localCoordsEE, bool grounded) {
		dVector qDot;
		robotRepresentation->getQDot(qDot);

		contactEndEffectors.push_back(QPControl_ContactEndEffector(rb, localCoordsEE));
		contactEndEffectors.back().updateJacobians(robotRepresentation);
		contactEndEffectors.back().targetPosition = rb->getWorldCoordinates(localCoordsEE);
		//we want the contact end effectors to not move... so the target acceleration that we set up here should do just that...
        V3D tmp = (V3D)(contactEndEffectors.back().J * qDot) / dt * -1;
		contactEndEffectors.back().inContact = grounded;
		if (!grounded){
			tmp += Globals::worldUp * Globals::g;
			//want non-grounded end effectors to mostly be moving vertically...
			contactEndEffectors.back().frictionCoeff = 0.05;
		}
		else {
			contactEndEffectors.back().frictionCoeff = 0.7;
		}
		setEndEffectorTargetAcceleration(&contactEndEffectors.back(), tmp);
	}


};

