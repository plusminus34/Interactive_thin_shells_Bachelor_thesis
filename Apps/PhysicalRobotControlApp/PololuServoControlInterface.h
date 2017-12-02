#pragma once

#include <string>
#include <map>
#include "RobotControlInterface.h"

using namespace	std;

/**
* Implements communication with servomotors via the Pololu Maestro 12/18/24-Channel USB Servo Controller
*/
class PololuServoControlInterface : public RobotControlInterface{
private:
	Robot* robot = NULL;

	int comNumber = 6;
	int fd = -1;

	//this is the refresh/update frequency for the servomotors
	int refreshRate = 50; //in Hz; 50Hz is the nominal refresh rate that most/all servomotors should work with, but some can use refresh rates of up to 333Hz

	//TODO these parameters are all servomotor specific, and should perhaps be set per motor rather than globally...

	//range of motion (radians) is -maxAngle..maxAngle
	double angleRange = RAD(45.0);
	//we will be sending the motors a pwm signal, whose width is measured in microseconds...
	double pwmWidthAtZero = 1520;//in microseconds
	double pwmRange = 500;

	double getServomotorAngle(int motorID);
	void setServomotorAngle(int motorID, double val);
	void setServomotorMaxSpeed(int motorID, double val);

public:
	// constructor
	PololuServoControlInterface(Robot* robot) : RobotControlInterface() {}

	// destructor
	virtual ~PololuServoControlInterface(void) {}

	//set motor goals from target values
	virtual void sendControlInputsToPhysicalRobot() = 0;
	//read motor positions
	virtual void readPhysicalRobotMotorPositions() = 0;
	//read motor positions
	virtual void readPhysicalRobotMotorVelocities() = 0;

	virtual void openCommunicationPort() = 0;
	virtual void closeCommunicationPort() = 0;
	virtual void driveMotorPositionsToZero() = 0;


	virtual void deactivateMotors() {}

};
