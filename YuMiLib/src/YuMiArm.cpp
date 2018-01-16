#include <YuMiLib/YuMiArm.h>

#include <iostream>
#ifdef WIN32
#include <Winsock2.h>
#include <Ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <unistd.h>
#endif

#include <fcntl.h>
#include <string.h>
#include <algorithm>


//Constructor
YuMiArm::YuMiArm(){}

//Destructor
YuMiArm::~YuMiArm(){}


//Initialize object
bool YuMiArm::init(std::string arm){

    //Set constants
    connected = false;
    armSide = arm;
	const char* ip = YuMiConstants::IP.c_str();
    unsigned int port = 0;
    if(armSide.compare("right") == 0){
        port = YuMiConstants::PORT_RIGHT_SERVER;
    } else if(armSide.compare("left") == 0){
        port = YuMiConstants::PORT_LEFT_SERVER;
    } else {
        std::cerr << "This arm does not exist!" << std::endl;
        return false;
    }

    //Connect arm to server
    bool socketConnected = connectServer(ip, port);

    //Set speed and joint variables
	bool jointsReceived = getCurrentJointsFromRobot(true);
	bool tcpSpeedSent = setRobotTCPSpeed(YuMiConstants::INIT_SPEED);

	//Init gripper
	bool gripInit = initGripper();
	bool gripOpened = openGripper();

	if(socketConnected && jointsReceived && tcpSpeedSent && gripInit && gripOpened){
        connected = true;
        return true;
    } else {
        std::cerr << "ERROR: Problem in YumiArm init -> socket connection, joints or speed..." << std::endl;
        return false;
    }
}


//Open socket and connect
bool YuMiArm::connectServer(const char* ip, unsigned int port) {
    // Init bool
    bool socketConnected = false;

    // Create a socket for robot
#ifdef WIN32
	unsigned short version = 2;
	WSADATA w;
	int ret = WSAStartup(version, &w);
	if(ret != 0) {
		std::cerr << "Failed to start Winsocket2" << std::endl;
	}
#endif
    if ((robotSocket = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1)
        std::cerr << "Problem creating the socket" << std::endl;
    else {
        // Now try to connect to the robot server
        struct sockaddr_in remoteSocket;
        remoteSocket.sin_family = AF_INET;
        remoteSocket.sin_port = htons(port);
        inet_pton(AF_INET, ip, &remoteSocket.sin_addr.s_addr);
        if(connect(robotSocket, (sockaddr*)&remoteSocket, sizeof(remoteSocket)) == -1){
            std::cerr << "Could not connect to the ABB robot" << std::endl;
        } else {
            socketConnected = true;
            std::cout << "Successfully connected to the ABB robot" << std::endl;
        }
    }

    return socketConnected;
}


bool YuMiArm::closeConnection(){
    char message[YuMiConstants::BUFSIZE];
    char reply[YuMiConstants::BUFSIZE];
    int idCode = YuMiConstants::ID_CLOSE_CONNECTION;

    strcpy(message, YuMiCom::closeConnection(idCode).c_str());

	sendAndReceive(message, strlen(message), reply, idCode); //TODO: Now reply from robot yet if successfully closed or not
	connected = false;
	return true;
}


//Send message to robot and receive answer
bool YuMiArm::sendAndReceive(char *message, int messageLength, char* reply, int idCode){
    bool success = false;

    sendRecvMutex.lock();
    if (send(robotSocket, message, messageLength, 0) == -1){
        if(connected){
            std::cerr << "Failed to send command to robot" << std::endl;
        }
	} else {
        // Read the reply to the message we just sent, and make sure
        // it's not corrupt, and the command was executed successfully
		#ifndef WIN32
			usleep(5000);
		#endif
		int t = recv(robotSocket, reply, YuMiConstants::BUFSIZE-1, 0);
		if(t > 0){
			reply[t] = '\0';
			int ok, rcvIdCode;
			sscanf(reply,"%d %d", &rcvIdCode, &ok);
			//std::cout << "reply: " << reply << std::endl;
			//std::cout << "message: " << message << std::endl;
			if(idCode!=-1) {
				if ((ok == YuMiConstants::SERVER_OK) && (rcvIdCode == idCode)) {
					success = true;
				} else if ((ok == YuMiConstants::SERVER_COLLISION) && (rcvIdCode == idCode)) {
					std::cerr << "WARNING: Collision Detected" << std::endl;
				} else {
					std::cerr << "WARNING: Corrupt message 1:" << std::endl;
					std::cerr << "msg = " << message << ";  reply = " << reply << ";  idCode = " << idCode << ";  ok = " << ok << ";  rcvCode = " << rcvIdCode << std::endl;
				}
			} else {
				if (ok == YuMiConstants::SERVER_OK) {
					success = true;
				} else if (ok == YuMiConstants::SERVER_COLLISION) {
					std::cerr << "WARNING: Collision Detected" << std::endl;
				} else {
					std::cerr << "WARNING: Corrupt message 2:" << std::endl;
					std::cerr << "msg = " << message << ";  reply = " << reply << ";  idCode = " << idCode << ";  ok = " << ok << ";  rcvCode = " << rcvIdCode << std::endl;
				}
			}
		} else {
			if(connected && idCode != YuMiConstants::ID_CLOSE_CONNECTION){
				std::cerr << "WARNING: Failed to receive answer from robot" << std::endl;
			}
		}
    }

	sendRecvMutex.unlock();

    return success;
}


//Ping Robot
bool YuMiArm::pingRobot(){
    char message[YuMiConstants::BUFSIZE];
    char reply[YuMiConstants::BUFSIZE];
    int idCode = YuMiConstants::ID_PING;

    strcpy(message, YuMiCom::ping(idCode).c_str());

	if(sendAndReceive(message, strlen(message), reply, idCode)){
        std::cout << "reply: " << reply << std::endl;
        return true;
    } else {
        return false;
    }
}


// Query robot for the current joint positions
bool YuMiArm::getCurrentJointsFromRobot(bool setTargetToCurrent){
	bool jointsReceived = false;
	char message[YuMiConstants::BUFSIZE];
    char reply[YuMiConstants::BUFSIZE];
    int idCode = YuMiConstants::ID_GET_JOINTS;

    strcpy(message, YuMiCom::getJoints(idCode).c_str());

	if(sendAndReceive(message, strlen(message), reply, idCode)){
        //Parse joints
		YuMiCom::parseJoints(reply, currentJoints);
		if(setTargetToCurrent){
			targetJoints = currentJoints;
		}
		jointsReceived = true;
    } else {
		if(connected){
			std::cerr << "ERROR: Something is wrong in YuMiArm getJoints!" << std::endl;
		}
    }

	return jointsReceived;
}


bool YuMiArm::sendRobotToJointPose(YuMiJoints yumiJoints){
    char message[YuMiConstants::BUFSIZE];
    char reply[YuMiConstants::BUFSIZE];
    int idCode = YuMiConstants::ID_GOTO_JOINT_POSE;

	strcpy(message, YuMiCom::gotoJointPose(idCode, yumiJoints).c_str());

	if(sendAndReceive(message, strlen(message), reply, idCode)){
		targetJoints = yumiJoints;
		//std::cout << "gotoJointPose-reply: " << reply << std::endl;
        return true;
    } else {
        return false;
    }
}


bool YuMiArm::setRobotTCPSpeed(unsigned int speed){
	if(speed > 0){
        char message[YuMiConstants::BUFSIZE];
        char reply[YuMiConstants::BUFSIZE];
        int idCode = YuMiConstants::ID_SET_SPEED;

		strcpy(message, YuMiCom::setTCPSpeed(idCode, speed).c_str());

		if(sendAndReceive(message, strlen(message), reply, idCode)){
			TCPSpeed = speed;
			//std::cout << "setSpeed-reply: " << reply << std::endl;
            return true;
        } else {
            return false;
        }
    } else {
		std::cerr << "ERROR in YuMiArm setSpeed -> speed must be larger than 0!" << std::endl;
        return false;
    }
}


bool YuMiArm::initGripper(){
	char message[YuMiConstants::BUFSIZE];
	char reply[YuMiConstants::BUFSIZE];
	int idCode = YuMiConstants::ID_GRIP_INIT;

	strcpy(message, YuMiCom::initGripper(idCode, YuMiConstants::GRIP_MAXSPD, YuMiConstants::GRIP_HOLDFORCE, YuMiConstants::GRIP_PHYLIMIT, YuMiConstants::GRIP_CALIB).c_str());

	if(sendAndReceive(message, strlen(message), reply, idCode)){
		gripperOpen = false;
		return true;
	} else {
		return false;
	}
}

bool YuMiArm::openGripper(){
	char message[YuMiConstants::BUFSIZE];
	char reply[YuMiConstants::BUFSIZE];
	int idCode = YuMiConstants::ID_GRIP_OPEN;

	strcpy(message, YuMiCom::openGripper(idCode, YuMiConstants::GRIP_MOVETOOPEN, YuMiConstants::GRIP_NOWAIT).c_str());

	if(sendAndReceive(message, strlen(message), reply, idCode)){
		gripperOpen = true;
		return true;
	} else {
		return false;
	}
}

bool YuMiArm::closeGripper(){
	char message[YuMiConstants::BUFSIZE];
	char reply[YuMiConstants::BUFSIZE];
	int idCode = YuMiConstants::ID_GRIP_OPEN;

	strcpy(message, YuMiCom::closeGripper(idCode, YuMiConstants::GRIP_NOWAIT).c_str());

	if(sendAndReceive(message, strlen(message), reply, idCode)){
		gripperOpen = false;
		return true;
	} else {
		return false;
	}
}


YuMiJoints YuMiArm::getCurrentJointValues(){
	return currentJoints;
}

unsigned int YuMiArm::getTCPSpeedValue(){
	return TCPSpeed;
}


bool YuMiArm::getConnectedValue(){
	return connected;
}


bool YuMiArm::getGripperOpenValue(){
	return gripperOpen;
}
