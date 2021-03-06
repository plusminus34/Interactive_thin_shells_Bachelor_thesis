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

	//std::cout << "getJoints-msg: " << msg << std::endl;

    return (msg);
}


void YuMiCom::parseJoints(std::string msg, YuMiJoints& yumiJoints){
	char * cstr = new char [msg.length()+1];
	std::strcpy (cstr, msg.c_str());
	char * p = std::strtok (cstr," ");
	std::vector<float> jointsTemp(YuMiConstants::NUM_JOINTS, 0.0);

	int iter = 0;
	while(p!=0){
		if(iter > 1){ //1. = idCode, 2. = ok
			jointsTemp[iter-2] = RAD(float(atof(p)));
		}
		p = std::strtok(NULL," ");
		iter++;
	}
	delete[] cstr;

	//Switch because of weird naming of ABB joints (1, 2, 7, 3, 4, 5, 6)
	yumiJoints.j1 = jointsTemp[0];
	yumiJoints.j2 = jointsTemp[1];
	yumiJoints.j3 = jointsTemp[6];
	yumiJoints.j4 = jointsTemp[2];
	yumiJoints.j5 = jointsTemp[3];
	yumiJoints.j6 = jointsTemp[4];
	yumiJoints.j7 = jointsTemp[5];
}


void YuMiCom::parseJointsAndExecTime(std::string msg, YuMiJoints& yumiJoints, double& execTime){
	char * cstr = new char [msg.length()+1];
	std::strcpy (cstr, msg.c_str());
	char * p = std::strtok (cstr," ");
	std::vector<float> jointsTemp(YuMiConstants::NUM_JOINTS, 0.0);

	int iter = 0;
	while(p!=0){
		if(iter > 1){ //1. = idCode, 2. = ok
			jointsTemp[iter-2] = RAD(float(atof(p)));
		}
		if(iter > 8){
			execTime = double(atof(p));
		}
		p = std::strtok(NULL," ");
		iter++;
	}
	delete[] cstr;

	//Switch because of weird naming of ABB joints (1, 2, 7, 3, 4, 5, 6)
	yumiJoints.j1 = jointsTemp[0];
	yumiJoints.j2 = jointsTemp[1];
	yumiJoints.j3 = jointsTemp[6];
	yumiJoints.j4 = jointsTemp[2];
	yumiJoints.j5 = jointsTemp[3];
	yumiJoints.j6 = jointsTemp[4];
	yumiJoints.j7 = jointsTemp[5];
}


void YuMiCom::parseExecTime(std::string msg, double& execTime){
	char * cstr = new char [msg.length()+1];
	std::strcpy (cstr, msg.c_str());
	char * p = std::strtok (cstr," ");

	int iter = 0;
	while(p!=0){
		if(iter > 1){ //1. = idCode, 2. = ok
			execTime = double(atof(p));
		}
		p = std::strtok(NULL," ");
		iter++;
	}
	delete[] cstr;
}


std::string YuMiCom::gotoJointPose(int idCode, YuMiJoints yumiJoints, double moveTime, bool wait){
    std::string msg;
    if(idCode < 10){
        msg = "0";
    } else {
        msg = "";
    }
    //Switch because of weird naming of ABB joints (1, 2, 7, 3, 4, 5, 6)
	msg += std::to_string(idCode); msg += " ";

	msg += std::to_string((float)((int)(DEG(yumiJoints.j1)*100))/100); msg.erase(msg.end()-4, msg.end()); msg += " ";
	msg += std::to_string((float)((int)(DEG(yumiJoints.j2)*100))/100); msg.erase(msg.end()-4, msg.end()); msg += " ";
	msg += std::to_string((float)((int)(DEG(yumiJoints.j4)*100))/100); msg.erase(msg.end()-4, msg.end()); msg += " ";
	msg += std::to_string((float)((int)(DEG(yumiJoints.j5)*100))/100); msg.erase(msg.end()-4, msg.end()); msg += " ";
	msg += std::to_string((float)((int)(DEG(yumiJoints.j6)*100))/100); msg.erase(msg.end()-4, msg.end()); msg += " ";
	msg += std::to_string((float)((int)(DEG(yumiJoints.j7)*100))/100); msg.erase(msg.end()-4, msg.end()); msg += " ";
	msg += std::to_string((float)((int)(DEG(yumiJoints.j3)*100))/100); msg.erase(msg.end()-4, msg.end()); msg += " ";

	msg += std::to_string(moveTime); msg += " "; msg.erase(msg.end()-3, msg.end()); msg += " ";

	if(wait){
		msg += std::to_string(999); msg += " ";
	}

	msg += "#";

	//std::cout << "gotoJointPose - msg: " << msg << std::endl;

	return (msg);
}


std::string YuMiCom::setTCPSpeed(int idCode, unsigned int speed, bool wait){
    std::string msg;
    if(idCode < 10){
        msg = "0";
    } else {
        msg = "";
    }
	msg += std::to_string(idCode); msg += " ";
	msg += std::to_string(speed); msg += " ";
	msg += std::to_string(YuMiConstants::SPEED_DATA_ROT); msg += " ";
	msg += std::to_string(speed); msg += " ";
	msg += std::to_string(YuMiConstants::SPEED_DATA_ROT); msg += " ";

	if(wait){
		msg += std::to_string(999); msg += " ";
	}

	msg += "#";

	//std::cout << "setSpeed-msg: " << msg << std::endl;

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

	//std::cout << "initGripper-msg: " << msg << std::endl;

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

	//std::cout << "openGripper-msg: " << msg << std::endl;

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
