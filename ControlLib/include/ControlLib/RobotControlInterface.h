#pragma once

#include <GUILib/GLApplication.h>
#include <string>
#include <map>
#include <GUILib/TranslateWidget.h>
#include <RBSimLib/AbstractRBEngine.h>
#include <RBSimLib/WorldOracle.h>
#include <RBSimLib/HingeJoint.h>
#include <GUILib/GLWindow3D.h>
#include <GUILib/GLWindow3DWithMesh.h>
#include <GUILib/GLWindowContainer.h>
#include <ControlLib/IK_Solver.h>

#include <iostream>

using namespace	std;

class HJ {
public:
	HingeJoint* j;
	bool isAuxiliary = false;

	//the partition into normal joints and auxiliary joints lets us employ potentially very different control strategies, for example for the upper body of a robot vs wheels

	HJ(HingeJoint* hj, bool isAuxiliary) {
		j = hj;
		this->isAuxiliary = isAuxiliary;
	}
};

/**
* Robot Design and Simulation interface
*/
class RobotControlInterface{
protected:
	Robot* robot = NULL;
	bool connected = false;
	bool motorsOn = false;

	/* This array of joints stores ALL hinge joints (e.g. the ones directly controllable through physical motors) of the robot, including the auxiliary ones... */
	DynamicArray<HJ> mJoints;

public:

	// constructor
	RobotControlInterface(Robot* robot) { 
		this->robot = robot; 
		for (auto j : robot->jointList) {
			HingeJoint* hj = dynamic_cast<HingeJoint*>(j);
			if (hj)
				mJoints.push_back(HJ(hj, false));
		}
		for (auto j : robot->auxiliaryJointList) {
			HingeJoint* hj = dynamic_cast<HingeJoint*>(j);
			if (hj)
				mJoints.push_back(HJ(hj, true));
		}
	}
	// destructor
	virtual ~RobotControlInterface(void) {}

	virtual void setSimRobotStateFromCurrentMotorValues() {
		//given the values stored in the joint's dxl properties structure (which are updated either from the menu or by sync'ing with the dynamixels), update the state of the robot... 
		RobotState rs(robot);

		for (auto hj : mJoints) {
			Quaternion q;
			q.setRotationFrom(hj.j->motor.currentMotorAngle, hj.j->rotationAxis);
			if (hj.isAuxiliary) {
				rs.setAuxiliaryJointRelativeOrientation(q, hj.j->jIndex);
				rs.setAuxiliaryJointRelativeAngVelocity(hj.j->rotationAxis * hj.j->motor.currentMotorVelocity, hj.j->jIndex);
			}else{
				rs.setJointRelativeOrientation(q, hj.j->jIndex);
				rs.setJointRelativeAngVelocity(hj.j->rotationAxis * hj.j->motor.currentMotorVelocity, hj.j->jIndex);
			}
		}
		robot->setState(&rs);
	}

	virtual void setTargetMotorValuesFromSimRobotState(double dt) {
		//given the values stored in the joint's dxl properties structure (which are updated either from the menu or by sync'ing with the dynamixels), update the state of the robot... 
		for (auto hj : mJoints) {
			Quaternion q = robot->getRelativeOrientationForJoint(hj.j);
			V3D w = robot->getRelativeLocalCoordsAngularVelocityForJoint(hj.j);
			hj.j->motor.targetMotorAngle = q.getRotationAngle(hj.j->rotationAxis);
			//hj.j->motor.targetMotorVelocity = w.dot(hj.j->rotationAxis);
		}
	}

	//set current sim values from robot readings...
	virtual void syncSimRobotWithPhysicalRobot() {
        //std::cout << "syncSimRobotWithPhysicalRobot" << std::endl;
		readPhysicalRobotMotorPositions();
		readPhysicalRobotMotorVelocities();
		setSimRobotStateFromCurrentMotorValues();
	}

	//the time window dt estimates the amount of time before the next command is issued (or, alternatively, how long we'd expect the physical robot to take before it can match the target values)
	virtual void syncPhysicalRobotWithSimRobot(double dt = 0.1) {
        setTargetMotorValuesFromSimRobotState(dt);
		sendControlInputsToPhysicalRobot();
	}

	//set motor goals from target values
	virtual void sendControlInputsToPhysicalRobot() = 0;
	//read motor positions
	virtual void readPhysicalRobotMotorPositions() = 0;
	//read motor positions
	virtual void readPhysicalRobotMotorVelocities() = 0;

	virtual void openCommunicationPort() = 0;
	virtual void closeCommunicationPort() = 0;
	virtual void driveMotorPositionsToZero() = 0;
    virtual void driveMotorPositionsToTestPos() = 0;
	virtual void grip(std::string arm) = 0;

	void toggleMotorPower() {
		motorsOn = !motorsOn;
	}

	bool motorsHavePower() {
		return motorsOn;
	}

	bool isConnected() {
		return connected;
	}

	bool controlPositionsOnly = false;

};
