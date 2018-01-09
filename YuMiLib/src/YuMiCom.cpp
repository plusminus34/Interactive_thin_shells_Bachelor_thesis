#include <YuMiLib/YuMiCom.h>
#include <YuMiLib/YuMiConstants.h>

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
            joints[iter-2] = degToRad(float(atof(p)));
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
    msg += std::to_string(radToDeg(j1)); msg += " ";
    msg += std::to_string(radToDeg(j2)); msg += " ";
    msg += std::to_string(radToDeg(j4)); msg += " ";
    msg += std::to_string(radToDeg(j5)); msg += " ";
    msg += std::to_string(radToDeg(j6)); msg += " ";
    msg += std::to_string(radToDeg(j7)); msg += " ";
    msg += std::to_string(radToDeg(j3)); msg += " ";

    msg += "#";

    std::cout << "msg: " << msg << std::endl;

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

//    std::cout << "msg: " << msg << std::endl;

    return (msg);
}


float YuMiCom::degToRad(float v){
    return v*3.14159265359/180.0;
}

float YuMiCom::radToDeg(float v){
    return v*180.0/3.14159265359;
}
