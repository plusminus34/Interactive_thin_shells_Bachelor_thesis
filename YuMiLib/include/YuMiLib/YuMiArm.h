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

	double execTime = 0.0;

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
	bool sendWithoutReceive(char *message, int messageLength);

    bool pingRobot();
	bool getCurrentJointsFromRobot(bool setTargetToCurrent);
	bool sendRobotToJointPose(YuMiJoints yumiJoints, double moveTime, bool wait);
	bool setRobotTCPSpeed(unsigned int speed, bool wait);
	bool getAndSendJointsToRobot(YuMiJoints yumiJoints, double moveTime);

	bool initGripper();
	bool closeGripper();
	bool openGripper();

	YuMiJoints getCurrentJointValues();
	unsigned int getTCPSpeedValue();
	bool getConnectedValue();
	bool getGripperOpenValue();

	YuMiJoints convertVectorToYuMiJoints(std::vector<float> v);

	double getExecTime();

};
