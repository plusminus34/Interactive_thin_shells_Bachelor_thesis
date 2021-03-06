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
	static void parseJointsAndExecTime(std::string msg, YuMiJoints& yumiJoints, double& execTime);
	static void parseExecTime(std::string msg, double& execTime);
	static std::string gotoJointPose(int idCode, YuMiJoints yumiJoints, double moveTime, bool wait);
	static std::string setTCPSpeed(int idCode, unsigned int speed, bool wait);
	static std::string initGripper(int idCode, float maxSpd, float holdForce, float phyLimit, bool calibrate);
	static std::string openGripper(int idCode, float targetPos, bool noWait);
	static std::string closeGripper(int idCode, bool noWait);

};
