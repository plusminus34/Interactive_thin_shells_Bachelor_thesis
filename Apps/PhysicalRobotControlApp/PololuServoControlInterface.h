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

//SOME OF THESE PARAMETERS MUST BE SET WITH THE POLOLU SERVOMOTOR CONTROLLER, INCLUDING THE MAESTRO�S SERIAL MODE WHICH NEEDS TO BE SET TO �USB Dual Port�
	int comNumber = 4;
	//this is the refresh/update frequency for the servomotors
	int refreshRate = 50; //in Hz; 50Hz is the nominal refresh rate that most/all servomotors should work with, but some can use refresh rates of up to 333Hz

	int fd = -1;

	double getServomotorAngle(Motor& mp);
	void setServomotorAngle(Motor& mp, double val);
	void setServomotorMaxSpeed(Motor& mp, double val);

	bool servosAreMoving();
	int maestroGetPosition(Motor& mp);
	int maestroSetTargetPosition(Motor& mp, unsigned short target);
	int maestroSetTargetSpeed(Motor& mp, unsigned short target);


public:
	// constructor
	PololuServoControlInterface(Robot* robot);

	// destructor
	virtual ~PololuServoControlInterface(void) {}

	//set motor goals from target values
	virtual void sendControlInputsToPhysicalRobot();
	//read motor positions
	virtual void readPhysicalRobotMotorPositions();
	//read motor positions
	virtual void readPhysicalRobotMotorVelocities();

	virtual void openCommunicationPort();
	virtual void closeCommunicationPort();
	virtual void driveMotorPositionsToZero();
	void toggleMotorPower();

};
