#pragma once

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

    float joint1, joint2, joint3, joint4, joint5, joint6, joint7; //joint values in rad!
    unsigned int speed;

public:
    //Constructor
    YuMiArm();

    //Destructor
    ~YuMiArm();

    //Functions
    bool init(std::string arm);
    bool connectServer(const char* p, unsigned int port);
    bool closeConnection();
	bool sendAndReceive(char *message, int messageLength, char* reply, int idCode, bool waitForReply);

    bool pingRobot();
    std::vector<float> getJoints();
    bool gotoJointPose(std::vector<float> joints);
    bool setSpeed(unsigned int s);
    unsigned int getSpeed();

    bool getConnected();

	bool initGripper();
	bool closeGripper();
	bool openGripper();
	bool getGripperOpen();

};
