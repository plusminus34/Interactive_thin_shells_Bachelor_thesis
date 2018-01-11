#pragma once

#include <vector>
#include <string>

class YuMiConstants{
private:


public:
    //Constructor
    YuMiConstants();

    //Destructor
    ~YuMiConstants();

    //Constants
	static std::string IP;
	static const unsigned int PORT_LEFT_SERVER = 5000;
	static const unsigned int PORT_RIGHT_SERVER = 5001;
	static const unsigned int BUFSIZE = 4096; //the same as MAX_BUFFER
	static const unsigned int SERVER_OK = 1;
	static const unsigned int SERVER_COLLISION = 2;
	static const unsigned int NUM_JOINTS = 7;
	static const unsigned int SPEED_DATA_ROT = 500;
	static const unsigned int INIT_SPEED = 50;

	static const unsigned int ID_PING = 0;
	static const unsigned int ID_GOTO_JOINT_POSE = 2;
	static const unsigned int ID_GET_JOINTS = 4;
	static const unsigned int ID_SET_SPEED = 8;
	static const unsigned int ID_CLOSE_CONNECTION = 99;

	const std::vector<float> CALIB_STATE_LEFT = {0.0f, -2.26893f, 0.0f, 0.69813f, 0.0f, -2.35619f, 0.52360f};
	const std::vector<float> CALIB_STATE_RIGHT = {0.0f, -2.26893f, 0.0f, 0.69813f, 0.0f, 2.35619f, 0.52360f};
	const std::vector<float> INIT_STATE_LEFT = {-1.86715f, -1.74027f, 1.21143f, 1.80642f, 0.28955f, 0.44331f, -0.46810f};
	const std::vector<float> INIT_STATE_RIGHT = {2.16683f, -1.01159f, 4.37432f, 2.21674f, -0.44576f, -0.94859f, 0.35500f};
};
