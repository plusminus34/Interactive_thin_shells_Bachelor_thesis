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
	static const unsigned int BUFSIZE = 4096; //the same as MAX_BUFFER (default: 4096)
	static const unsigned int SERVER_OK = 1;
	static const unsigned int SERVER_COLLISION = 2;
	static const unsigned int NUM_JOINTS = 7;
	static const unsigned int SPEED_DATA_ROT = 500;
	static const unsigned int INIT_SPEED = 100;

	static constexpr float GRIP_MAXSPD = 25.0f; //[mm/s]
	static constexpr float GRIP_HOLDFORCE = 20.0f; //[N]
	static constexpr float GRIP_PHYLIMIT = 25.0f; //[mm]
	static constexpr float GRIP_MOVETOOPEN = 20.0f; //[mm]
	static const bool GRIP_CALIB = true;
	static const bool GRIP_NOWAIT = true;

	static std::vector<float> CALIB_STATE_LEFT;
	static std::vector<float> CALIB_STATE_RIGHT;

	static std::vector<float> INIT_STATE_LEFT;
	static std::vector<float> INIT_STATE_RIGHT;

	static std::vector<float> HOME_STATE_LEFT;
	static std::vector<float> HOME_STATE_RIGHT;

	static const unsigned int ID_PING = 0;
	static const unsigned int ID_GOTO_JOINT_POSE = 2; //2 for unsync, 12 for sync
	static const unsigned int ID_GET_JOINTS = 4;
	static const unsigned int ID_SET_SPEED = 8;
	static const unsigned int ID_GRIP_INIT = 60;
	static const unsigned int ID_GRIP_OPEN = 63;
	static const unsigned int ID_GRIP_CLOSE = 64;
	static const unsigned int ID_GET_SEND_JOINTS_SPEED = 70;
	static const unsigned int ID_CLOSE_CONNECTION = 99;

	static const unsigned int MOVE_ZONE = 200; //length of corner path given in mm
	static const unsigned int MOVE_STOPPOINTDATA = 50; //20, 50 or 100 -> specifies convergence criteria for position of the robots TCP in the stop point




};
