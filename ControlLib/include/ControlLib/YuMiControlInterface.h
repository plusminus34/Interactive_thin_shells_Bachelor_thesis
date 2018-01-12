#pragma once

#include "RobotControlInterface.h"
#include "../YuMiLib/include/YuMiLib/YuMiArm.h"

#include <string>
#include <iostream>

/**
* Implements communication with yumi robot
*/

class YuMiControlInterface : public RobotControlInterface{
private:
    //arms
    YuMiArm leftArm, rightArm;

public:
	// constructor
    YuMiControlInterface(Robot* robot);

	// destructor
    ~YuMiControlInterface();

	//set motor goals from target values
	virtual void sendControlInputsToPhysicalRobot();
	//read motor positions
	virtual void readPhysicalRobotMotorPositions();
	//read motor positions
	virtual void readPhysicalRobotMotorVelocities();

	virtual void setTargetMotorValuesFromSimRobotState(double dt);

	virtual void openCommunicationPort();
	virtual void closeCommunicationPort();
	virtual void driveMotorPositionsToZero();
    virtual void driveMotorPositionsToTestPos();

	void driveMotorPositionsToInputPos(std::vector<float> leftJoints, std::vector<float> rightJoints);
	void grip(std::string arm);

};
