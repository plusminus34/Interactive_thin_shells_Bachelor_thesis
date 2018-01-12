#include <YuMiLib/YuMiCom.h>
#include <YuMiLib/YuMiConstants.h>
#include "../../MathLib/include/MathLib/MathLib.h"

#include <string.h>
#include <cstring>
#include <iostream>
#include <vector>

//Constructor
YuMiCom::YuMiCom(){}

//Destructor
YuMiCom::~YuMiCom(){}

//Ping robot
std::string YuMiCom::ping(int idCode){
    std::string msg;
    if(idCode < 10){
        msg = "0";
    } else {
        msg = "";
    }
    msg += std::to_string(idCode);
    msg += " #";

    return (msg);
}

std::string YuMiCom::closeConnection(int idCode){
    std::string msg;
    if(idCode < 10){
        msg = "0";
    } else {
        msg = "";
    }
    msg += std::to_string(idCode);
    msg += " #";

    return (msg);
}


//Get joint values of robot
std::string YuMiCom::getJoints(int idCode) {
    std::string msg;
    if(idCode < 10){
        msg = "0";
    } else {
        msg = "";
    }
    msg += std::to_string(idCode);
    msg += " #";

    return (msg);
}

void YuMiCom::parseJoints(std::string msg, float &j1, float &j2, float &j3, float &j4, float &j5, float &j6, float &j7){
    char * cstr = new char [msg.length()+1];
    std::strcpy (cstr, msg.c_str());
    char * p = std::strtok (cstr," ");
    std::vector<float> joints(YuMiConstants::NUM_JOINTS, 0.0);

    int iter = 0;
    while(p!=0){
        if(iter > 1){ //1. = idCode, 2. = ok
			joints[iter-2] = RAD(float(atof(p)));
        }
        p = std::strtok(NULL," ");
        iter++;
    }
    delete[] cstr;

    //Switch because of weird naming of ABB joints (1, 2, 7, 3, 4, 5, 6)
    j1 = joints[0]; j2 = joints[1]; j3 = joints[6]; j4 = joints[2]; j5 = joints[3]; j6 = joints[4]; j7 = joints[5];
}


std::string YuMiCom::gotoJointPose(int idCode, float j1, float j2, float j3, float j4, float j5, float j6, float j7){
    std::string msg;
    if(idCode < 10){
        msg = "0";
    } else {
        msg = "";
    }
    //Switch because of weird naming of ABB joints (1, 2, 7, 3, 4, 5, 6)
    msg += std::to_string(idCode); msg += " ";
	msg += std::to_string(DEG(j1)); msg += " ";
	msg += std::to_string(DEG(j2)); msg += " ";
	msg += std::to_string(DEG(j4)); msg += " ";
	msg += std::to_string(DEG(j5)); msg += " ";
	msg += std::to_string(DEG(j6)); msg += " ";
	msg += std::to_string(DEG(j7)); msg += " ";
	msg += std::to_string(DEG(j3)); msg += " ";

    msg += "#";

	//std::cout << "msg: " << msg << std::endl;

    return (msg);
}


std::string YuMiCom::setSpeed(int idCode, unsigned int s){
    std::string msg;
    if(idCode < 10){
        msg = "0";
    } else {
        msg = "";
    }
	msg += std::to_string(idCode); msg += " ";
	msg += std::to_string(s); msg += " ";
	msg += std::to_string(YuMiConstants::SPEED_DATA_ROT); msg += " ";
	msg += std::to_string(s); msg += " ";
	msg += std::to_string(YuMiConstants::SPEED_DATA_ROT); msg += " ";
	msg += "#";

	//std::cout << "msg: " << msg << std::endl;

    return (msg);
}

std::string YuMiCom::initGripper(int idCode, float maxSpd, float holdForce, float phyLimit, bool calibrate){
	std::string msg;
	if(idCode < 10){
		msg = "0";
	} else {
		msg = "";
	}
	msg += std::to_string(idCode); msg += " ";
	msg += std::to_string(maxSpd); msg += " ";
	msg += std::to_string(holdForce); msg += " ";
	msg += std::to_string(phyLimit); msg += " ";
	if(calibrate){
		msg += std::to_string(0); msg += " ";
	}
	msg += "#";

//    std::cout << "msg: " << msg << std::endl;

	return (msg);
}


std::string YuMiCom::openGripper(int idCode, float targetPos, bool noWait){
	std::string msg;
	if(idCode < 10){
		msg = "0";
	} else {
		msg = "";
	}

	if(targetPos < 0 || targetPos > YuMiConstants::GRIP_PHYLIMIT){
		std::cerr << "ERROR: Gripper target pos not in correct range!" << std::endl;
	} else {
		msg += std::to_string(idCode); msg += " ";
		msg += std::to_string(targetPos); msg += " ";
		if(noWait){
			msg += std::to_string(0); msg += " ";
		}
		msg += "#";
	}

	//    std::cout << "msg: " << msg << std::endl;

	return (msg);
}


std::string YuMiCom::closeGripper(int idCode, bool noWait){
	std::string msg;
	if(idCode < 10){
		msg = "0";
	} else {
		msg = "";
	}
	msg += std::to_string(idCode); msg += " ";
	if(noWait){
		msg += std::to_string(0); msg += " ";
	}
	msg += "#";

	//    std::cout << "msg: " << msg << std::endl;

	return (msg);
}
