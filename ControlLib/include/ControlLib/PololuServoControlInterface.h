#pragma once

#include <string>
#include <map>
#include "RobotControlInterface.h"

using namespace	std;

class ServoMotorCommandBlock {
public:
	int motorStartID;
	DynamicArray<unsigned short> targetVals;
	DynamicArray<HingeJoint*> robotJoints;
};

/**
* Implements communication with servomotors via the Pololu Maestro 12/18/24-Channel USB Servo Controller
*/
class PololuServoControlInterface : public RobotControlInterface{
public:

//SOME OF THESE PARAMETERS MUST BE SET WITH THE POLOLU SERVOMOTOR CONTROLLER, INCLUDING THE MAESTRO’S SERIAL MODE WHICH NEEDS TO BE SET TO “USB Dual Port”
	int comNumber = 4;
	//this is the period of servo pulses, expressed in milliseconds - set by the maestro control center, so make sure it matches up...
	int signalPeriod = 20; //in ms
	//file handle used for communication with the maestro board
	int fd = -1;
	//the target commands can be sent all at once, or one-by-one... which do we want?
	bool writeAllTargetCommandsAtOnce = true;

	//the servo control board expects a block of continuous servo IDs for the multi-write command... so, store this here...
	DynamicArray<ServoMotorCommandBlock> multiTargetCommands;

	double getServomotorAngle(Motor& mp);
	void setServomotorAngle(Motor& mp, double val);
	void setServomotorMaxSpeed(Motor& mp, double val);

	bool servosAreMoving();
	int maestroGetPosition(Motor& mp);
	int maestroSetTargetPosition(Motor& mp, unsigned short target);
	int maestroSetTargetSpeed(Motor& mp, unsigned short target);
	int maestroSetMultipleTargets(int startID, const DynamicArray<unsigned short>& targetValues);


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

	virtual void setTargetMotorValuesFromSimRobotState(double dt);

	virtual void openCommunicationPort();
	virtual void closeCommunicationPort();
	virtual void driveMotorPositionsToZero();
	void toggleMotorPower();

};
