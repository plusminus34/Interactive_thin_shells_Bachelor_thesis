#pragma once

//#include <string>
//#include <map>
#include "RobotControlInterface.h"

//using namespace	std;

/**
* Implements communication with yumi robot
*/
class YuMiControlInterface : public RobotControlInterface{
private:
    // nothing so far ...

public:
	// constructor
    YuMiControlInterface(Robot* robot);

	// destructor
    virtual ~YuMiControlInterface(void) {}

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

};
