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

using namespace	std;

/**
* Robot Design and Simulation interface
*/
class RobotControlInterface{
private:
	Robot* robot = NULL;

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
			q.setRotationFrom(hj->motorProperties.currentMotorAngle, hj->rotationAxis);
			rs.setJointRelativeOrientation(q, i);
			rs.setJointRelativeAngVelocity(hj->rotationAxis * hj->motorProperties.currentMotorVelocity, i);
		}
		robot->setState(&rs);
	}

	virtual void setTargetMotorValuesFromSimRobotState() {
		//given the values stored in the joint's dxl properties structure (which are updated either from the menu or by sync'ing with the dynamixels), update the state of the robot... 
		ReducedRobotState rs(robot);

		for (int i = 0; i < robot->getJointCount(); i++) {
			HingeJoint* hj = dynamic_cast<HingeJoint*>(robot->getJoint(i));
			if (!hj) continue;
			Quaternion q = rs.getJointRelativeOrientation(i);
			V3D w = rs.getJointRelativeAngVelocity(i);
			hj->motorProperties.targetMotorAngle = q.getRotationAngle(hj->rotationAxis);
			hj->motorProperties.targetMotorVelocity = w.dot(hj->rotationAxis);
		}
	}

	//set current sim values from robot readings...
	virtual void syncSimRobotWithPhysicalRobot() {
		readPhysicalRobotMotorPositions();
		readPhysicalRobotMotorVelocities();
		setSimRobotStateFromCurrentMotorValues();
	}

	//motors may be assembled in a different way than how they are modeled, so account for that...
	void flipMotorTargetsIfNeeded() {
		for (int i = 0; i < robot->getJointCount(); i++) {
			HingeJoint* hj = dynamic_cast<HingeJoint*>(robot->getJoint(i));
			if (!hj) continue;
			if (hj->motorProperties.flipMotorAxis) {
				hj->motorProperties.targetMotorAngle *= -1;
				hj->motorProperties.targetMotorVelocity *= -1;
				hj->motorProperties.targetMotorAcceleration *= -1;
			}
		}
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

	virtual void deactivateMotors() = 0;

};
