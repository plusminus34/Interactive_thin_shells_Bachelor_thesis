#include <YuMiLib/YuMiConstants.h>

//Constructor
YuMiConstants::YuMiConstants(){
    setYuMiConstants();
}

//Destructor
YuMiConstants::~YuMiConstants(){}

//Functions
void YuMiConstants::setYuMiConstants(){
    IP = "192.168.125.1";
    PORT_LEFT_SERVER = 5000;
//    PORT_LEFT_STATES = 5010;
//    PORT_LEFT_POSES = 5012;
//    PORT_LEFT_TORQUES = 5014;

    PORT_RIGHT_SERVER = 5001;
//    PORT_RIGHT_STATES = 5011;
//    PORT_RIGHT_POSES = 5013;
//    PORT_RIGHT_TORQUES = 5015;

    BUFSIZE = 4096;
//    MOTION_TIMEOUT = 8;
//    COMM_TIMEOUT = 5;
//    PROCESS_TIMEOUT = 10;
//    PROCESS_SLEEP_TIME = 0.01;
//    MOTION_BUFFER_SIZE = 512;

//    MAX_GRIP_HOLDFORCE = 20.0;
//    MAX_GRIP_PHYLIMIT = 25.0;

//    COMM_PERIOD = 0.04;

    ID_CODE_MAX = 999;

    SERVER_BAD_MSG = 0;
    SERVER_OK = 1;
    SERVER_COLLISION = 2;

    NUM_JOINTS = 7;



}
