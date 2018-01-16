#pragma once

#include "YuMiArm.h"

#include <string>

class YuMiCom{
private:

public:
    //Constructor
    YuMiCom();

    //Destructor
    ~YuMiCom();

    //Functions
    static std::string ping(int idCode);
    static std::string closeConnection(int idCode);
    static std::string getJoints(int idCode);
	static void parseJoints(std::string msg, YuMiJoints& yumiJoints);
	static std::string gotoJointPose(int idCode, YuMiJoints yumiJoints);
	static std::string setTCPSpeed(int idCode, unsigned int speed);
	static std::string getAndSetJointsAndTCPSpeed(int idCode, YuMiJoints yumiJoints, unsigned int speed);
	static std::string initGripper(int idCode, float maxSpd, float holdForce, float phyLimit, bool calibrate);
	static std::string openGripper(int idCode, float targetPos, bool noWait);
	static std::string closeGripper(int idCode, bool noWait);

};
