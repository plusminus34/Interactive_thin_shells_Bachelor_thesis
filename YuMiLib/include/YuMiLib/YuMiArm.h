#pragma once

#include "YuMiJoints.h"
#include "YuMiConstants.h"
#include "YuMiCom.h"

#include <string>
#include <vector>
#include <mutex>


class YuMiArm{

private:
    std::string armSide;
    int robotSocket;
	bool connected = false;
	std::mutex sendRecvMutex;
	bool gripperOpen;

	YuMiJoints currentJoints, targetJoints;
	unsigned int TCPSpeed;

public:
    //Constructor
    YuMiArm();

    //Destructor
    ~YuMiArm();

    //Functions
    bool init(std::string arm);
    bool connectServer(const char* p, unsigned int port);
    bool closeConnection();
	bool sendAndReceive(char *message, int messageLength, char* reply, int idCode);

    bool pingRobot();
	bool getCurrentJointsFromRobot(bool setTargetToCurrent);
	bool sendRobotToJointPose(YuMiJoints inputJoints);
	bool setRobotTCPSpeed(unsigned int inputSpeed);

	bool initGripper();
	bool closeGripper();
	bool openGripper();

	YuMiJoints getCurrentJointValues();
	unsigned int getTCPSpeedValue();
	bool getConnectedValue();
	bool getGripperOpenValue();

};
