#include <YuMiLib/YuMiConstants.h>

//Variables
const char* YuMiConstants::IP = "192.168.125.1";
const unsigned int YuMiConstants::PORT_LEFT_SERVER = 5000;
const unsigned int YuMiConstants::PORT_RIGHT_SERVER = 5001;
const unsigned int YuMiConstants::BUFSIZE = 4096; //the same as MAX_BUFFER
const unsigned int YuMiConstants::SERVER_OK = 1;
const unsigned int YuMiConstants::SERVER_COLLISION = 2;
const unsigned int YuMiConstants::NUM_JOINTS = 7;
const unsigned int YuMiConstants::SPEED_DATA_ROT = 500;
const unsigned int YuMiConstants::INIT_SPEED = 50;

const unsigned int YuMiConstants::ID_PING = 0;
const unsigned int YuMiConstants::ID_GOTO_JOINT_POSE = 2;
const unsigned int YuMiConstants::ID_GET_JOINTS = 4;
const unsigned int YuMiConstants::ID_SET_SPEED = 8;
const unsigned int YuMiConstants::ID_CLOSE_CONNECTION = 99;

const std::vector<float> CALIB_STATE_LEFT = {0.0, -2.26893, 0.0, 0.69813, 0.0, -2.35619, 0.52360};
const std::vector<float> CALIB_STATE_RIGHT = {0.0, -2.26893, 0.0, 0.69813, 0.0, 2.35619, 0.52360};
const std::vector<float> INIT_STATE_LEFT = {-1.86715, -1.74027, 1.21143, 1.80642, 0.28955, 0.44331, -0.46810};
const std::vector<float> INIT_STATE_RIGHT = {2.16683, -1.01159, 4.37432, 2.21674, -0.44576, -0.94859, 0.35500};

//Constructor
YuMiConstants::YuMiConstants(){}

//Destructor
YuMiConstants::~YuMiConstants(){}
