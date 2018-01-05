#pragma once

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
    const char* IP;
    unsigned int PORT_LEFT_SERVER;
//    unsigned int PORT_LEFT_STATES;
//    unsigned int PORT_LEFT_POSES;
//    unsigned int PORT_LEFT_TORQUES;

    unsigned int PORT_RIGHT_SERVER;
//    unsigned int PORT_RIGHT_STATES;
//    unsigned int PORT_RIGHT_POSES;
//    unsigned int PORT_RIGHT_TORQUES;

    unsigned int BUFSIZE; //the same as MAX_BUFFER
//    unsigned int MOTION_TIMEOUT;
//    unsigned int COMM_TIMEOUT;
//    unsigned int PROCESS_TIMEOUT;
//    float PROCESS_SLEEP_TIME;
//    unsigned int MOTION_BUFFER_SIZE;

//    float MAX_GRIP_HOLDFORCE;
//    float MAX_GRIP_PHYLIMIT;

//    float COMM_PERIOD;

    unsigned int ID_CODE_MAX;

    unsigned int SERVER_BAD_MSG;
    unsigned int SERVER_OK;
    unsigned int SERVER_COLLISION;

    unsigned int NUM_JOINTS;

};
