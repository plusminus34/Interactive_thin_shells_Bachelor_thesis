#pragma once

#include <vector>

class YuMiConstants{
private:


public:
    //Constructor
    YuMiConstants();

    //Destructor
    ~YuMiConstants();

    //Functions
    void setYuMiConstants();

    //Constants
    static const char* IP;
    static const unsigned int PORT_LEFT_SERVER;
    static const unsigned int PORT_RIGHT_SERVER;
    static const unsigned int BUFSIZE; //the same as MAX_BUFFER
    static const unsigned int SERVER_OK;
    static const unsigned int SERVER_COLLISION;
    static const unsigned int NUM_JOINTS;
    static const unsigned int SPEED_DATA_ROT;
    static const unsigned int INIT_SPEED;

    static const unsigned int ID_PING;
    static const unsigned int ID_GOTO_JOINT_POSE;
    static const unsigned int ID_GET_JOINTS;
    static const unsigned int ID_SET_SPEED;
    static const unsigned int ID_CLOSE_CONNECTION;

    static const std::vector<float> CALIB_STATE_LEFT;
    static const std::vector<float>  CALIB_STATE_RIGHT;
    static const std::vector<float> INIT_STATE_LEFT;
    static const std::vector<float> INIT_STATE_RIGHT;
};
