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

/**
* Robot Design and Simulation interface
*/
class RobotControlInterface{
protected:
	Robot* robot = NULL;
	bool connected = false;
	bool motorsOn = false;

public:

	// constructor
	RobotControlInterface(Robot* robot) { this->robot = robot; }
	// destructor
	virtual ~RobotControlInterface(void) {}

	virtual void setSimRobotStateFromCurrentMotorValues() {
		//given the values stored in the joint's dxl properties structure (which are updated either from the menu or by sync'ing with the dynamixels), update the state of the robot... 
		ReducedRobotState rs(robot);

		for (int i = 0; i < robot->getJointCount(); i++) {
			HingeJoint* hj = dynamic_cast<HingeJoint*>(robot->getJoint(i));
			if (!hj) continue;
			Quaternion q;
			q.setRotationFrom(hj->motor.currentMotorAngle, hj->rotationAxis);
			rs.setJointRelativeOrientation(q, i);
			rs.setJointRelativeAngVelocity(hj->rotationAxis * hj->motor.currentMotorVelocity, i);
		}
		robot->setState(&rs);
	}

	virtual void setTargetMotorValuesFromSimRobotState(double dt) {
		//given the values stored in the joint's dxl properties structure (which are updated either from the menu or by sync'ing with the dynamixels), update the state of the robot... 
		ReducedRobotState rs(robot);

		for (int i = 0; i < robot->getJointCount(); i++) {
			HingeJoint* hj = dynamic_cast<HingeJoint*>(robot->getJoint(i));
			if (!hj) continue;
			Quaternion q = rs.getJointRelativeOrientation(i);
			V3D w = rs.getJointRelativeAngVelocity(i);
			hj->motor.targetMotorAngle = q.getRotationAngle(hj->rotationAxis);
			hj->motor.targetMotorVelocity = w.dot(hj->rotationAxis);
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
        //std::cout << "syncPhysicalRobotWithSimRobot" << std::endl;
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
