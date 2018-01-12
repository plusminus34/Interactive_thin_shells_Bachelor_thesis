#pragma once

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
    static void parseJoints(std::string msg, float &j1, float &j2, float &j3, float &j4, float &j5, float &j6, float &j7);
    static std::string gotoJointPose(int idCode, float j1, float j2, float j3, float j4, float j5, float j6, float j7);
    static std::string setSpeed(int idCode, unsigned int speed);
	static std::string initGripper(int idCode, float maxSpd, float holdForce, float phyLimit, bool calibrate);
	static std::string openGripper(int idCode, float targetPos, bool noWait);
	static std::string closeGripper(int idCode, bool noWait);

};
