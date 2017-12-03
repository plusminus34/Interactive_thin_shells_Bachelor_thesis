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

	int comNumber = 6;
	int fd = -1;

	//this is the refresh/update frequency for the servomotors
	int refreshRate = 50; //in Hz; 50Hz is the nominal refresh rate that most/all servomotors should work with, but some can use refresh rates of up to 333Hz

	//TODO these parameters are all servomotor specific, and should perhaps be set per motor rather than globally...


	//It appears that the max range of pwm signals gets mapped to the max range of angles...
	//range of motion (radians) is -maxAngle..maxAngle
	double angleRange = RAD(45.0);
	double pwmRange = 500; //width of signal is measured in miscroseconds....
	//this corresponds to the pwm signal that gets the servomotor to be at 0 (or in the very middle of its range of motion)
	double pwmWidthAtZero = 1520;//in microseconds

	double getServomotorAngle(int motorID);
	void setServomotorAngle(int motorID, double val);
	void setServomotorMaxSpeed(int motorID, double val);

	bool servosAreMoving();
	int maestroGetPosition(unsigned char channel);
	int maestroSetTargetPosition(unsigned char channel, unsigned short target);
	int maestroSetTargetSpeed(unsigned char channel, unsigned short target);


public:
	// constructor
	PololuServoControlInterface(Robot* robot) : RobotControlInterface(robot) {}

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
