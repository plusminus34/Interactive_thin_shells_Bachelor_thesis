#pragma once

#include "RobotControlInterface.h"
#include "../YuMiLib/include/YuMiLib/YuMiArm.h"

#include <ControlLib/IK_Solver.h>
#include <MathLib/P3D.h>

#include <string>
#include <iostream>

/**
* Implements communication with yumi robot
*/

class YuMiControlInterface : public RobotControlInterface{
protected:
    //arms
    YuMiArm leftArm, rightArm;

	YuMiJoints savedLeftJoints, savedRightJoints;

	//End-effector speed variable for yumi (left + right arm) -> mm/s
	struct TCPSpeed{
		unsigned int current = 50, target = 50;
	};
	TCPSpeed tcpSpeedRight;
	TCPSpeed tcpSpeedLeft;

	P3D localTCPLeft = P3D(0.017977, -0.0169495, 0.01949);
	P3D localTCPRight = P3D(0.0200485, -0.0189025, -0.02173559);

	struct TCP{
		V3D current, target;
	};

	TCP globalTCPLeft;
	TCP globalTCPRight;

	double lengthVecTCPLeft;
	double lengthVecTCPRight;

	unsigned int minSpeed = 1;
	unsigned int maxSpeed = 1500;
	float speedWeight = 1.0f;

public:
	bool sendControlInputsDelayed = true;

public:
	// constructor
    YuMiControlInterface(Robot* robot);

	// destructor
    ~YuMiControlInterface();

	//set motor goals from target values
	virtual void sendControlInputsToPhysicalRobot(double dt);
	//read motor positions
	virtual void readPhysicalRobotMotorPositions();
	//read motor positions
	virtual void readPhysicalRobotMotorVelocities();

	virtual void setTargetMotorValuesFromSimRobotState(double dt);

	virtual void openCommunicationPort();
	virtual void closeCommunicationPort();
	virtual void driveMotorPositionsToZero();

	virtual void driveMotorPositionsToTestPos1(IK_Solver* ikSolverPtr);
	virtual void driveMotorPositionsToTestPos2(IK_Solver* ikSolverPtr);
	void driveMotorPositionsToInputPos(std::vector<float> leftJoints, std::vector<float> rightJoints, IK_Solver* ikSolverPtr);
	void grip(std::string arm);
	virtual void printJointValues();
	bool sendJointInputsCheck(std::string arm);
	bool sendSpeedInputCheck();

	bool movementCheck(YuMiJoints targetJoints, YuMiJoints savedJoints);

};
