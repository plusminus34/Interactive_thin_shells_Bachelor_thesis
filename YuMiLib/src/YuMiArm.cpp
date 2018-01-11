#include <YuMiLib/YuMiArm.h>

#include <iostream>
#ifdef WIN32
#include <Winsock2.h>
#include <Ws2tcpip.h>
#else
#include <arpa/inet.h>
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
    const char* ip = YuMiConstants::IP;
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
    std::vector<float> joints = getJoints();
    bool joints_ok = !(std::all_of(joints.begin(), joints.end(), [](int i) { return i==0; }));
    bool speed_ok = setSpeed(YuMiConstants::INIT_SPEED);

//    std::cout << "socket: " << socketConnected << std::endl;
//    std::cout << "joints: " << joints_ok << std::endl;
//    std::cout << "speed: " << speed_ok << std::endl;

    if(socketConnected && joints_ok && speed_ok){
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
    if ((robotSocket = socket(PF_INET, SOCK_STREAM, 0)) == -1)
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

    if(sendAndReceive(message, strlen(message), reply, idCode)){
//        std::cout << "reply: " << reply << std::endl;
        return true;
    } else {
        return false;
    }
}


//Send message to robot and receive answer
bool YuMiArm::sendAndReceive(char *message, int messageLength, char* reply, int idCode){
    bool success = false;

    //pthread_mutex_lock(&sendRecvMutex);
    sendRecvMutex.lock();
    if (send(robotSocket, message, messageLength, 0) == -1){
        std::cerr << "Failed to send command to robot" << std::endl;
    } else {
        // Read the reply to the message we just sent, and make sure
        // it's not corrupt, and the command was executed successfully
        int t;
        if ((t=recv(robotSocket, reply, YuMiConstants::BUFSIZE-1, 0)) > 0) {
            reply[t] = '\0';
            int ok, rcvIdCode;
            sscanf(reply,"%d %d", &rcvIdCode, &ok);
            //std::cout << "reply: " << reply << std::endl;
            if(idCode!=-1) {
                if ((ok == YuMiConstants::SERVER_OK) && (rcvIdCode == idCode)) {
                    //pthread_mutex_unlock(&sendRecvMutex);
					sendRecvMutex.unlock();
                    success = true;
                } else if ((ok == YuMiConstants::SERVER_COLLISION) && (rcvIdCode == idCode)) {
                    std::cerr << "WARNING: Collision Detected" << std::endl;
                } else {
                    std::cerr << "WARNING: Corrupt message 1:" << std::endl;
                    std::cerr << "msg = " << message << ";  reply = " << reply << ";  idCode = " << idCode << ";  ok = " << ok << ";  rcvCode = " << rcvIdCode << std::endl;
                }
            } else {
                if (ok == YuMiConstants::SERVER_OK) {
                    //pthread_mutex_unlock(&sendRecvMutex);
					sendRecvMutex.unlock();
                    success = true;
                } else if (ok == YuMiConstants::SERVER_COLLISION) {
                    std::cerr << "WARNING: Collision Detected" << std::endl;
                } else {
                    std::cerr << "WARNING: Corrupt message 2:" << std::endl;
                    std::cerr << "msg = " << message << ";  reply = " << reply << ";  idCode = " << idCode << ";  ok = " << ok << ";  rcvCode = " << rcvIdCode << std::endl;
                }
            }
        } else {
            std::cerr << "WARNING: Failed to receive answer from robot" << std::endl;
        }
    }

    //pthread_mutex_unlock(&sendRecvMutex);
	sendRecvMutex.unlock();

    return success;
}


//Ping robot
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
std::vector<float> YuMiArm::getJoints(){
    char message[YuMiConstants::BUFSIZE];
    char reply[YuMiConstants::BUFSIZE];
    int idCode = YuMiConstants::ID_GET_JOINTS;
    std::vector<float> joints(YuMiConstants::NUM_JOINTS, 0.0);

    strcpy(message, YuMiCom::getJoints(idCode).c_str());

    if(sendAndReceive(message, strlen(message), reply, idCode)){

        //Parse joints
        YuMiCom::parseJoints(reply, joint1, joint2, joint3, joint4, joint5, joint6, joint7);
        joints[0] = joint1; joints[1] = joint2; joints[2] = joint3; joints[3] = joint4; joints[4] = joint5; joints[5] = joint6; joints[6] = joint7;

    } else {
        std::cerr << "ERROR: Something is wrong in YuMiArm getJoints!" << std::endl;
    }

    return joints;
}


bool YuMiArm::gotoJointPose(std::vector<float> joints){
    char message[YuMiConstants::BUFSIZE];
    char reply[YuMiConstants::BUFSIZE];
    int idCode = YuMiConstants::ID_GOTO_JOINT_POSE;

    if(joints.size() == YuMiConstants::NUM_JOINTS){
        joint1 = joints[0]; joint2 = joints[1]; joint3 = joints[2]; joint4 = joints[3]; joint5 = joints[4]; joint6 = joints[5]; joint7 = joints[6];
    } else {
        std::cerr << "ERROR: Something with the joint vector in YuMiArm setJoints is wrong!" << std::endl;
        return false;
    }
    strcpy(message, YuMiCom::gotoJointPose(idCode, joint1, joint2, joint3, joint4, joint5, joint6, joint7).c_str());

    if(sendAndReceive(message, strlen(message), reply, idCode)){
        return true;
    } else {
        return false;
    }
}


bool YuMiArm::setSpeed(unsigned int s){
    if(s > 0){
        char message[YuMiConstants::BUFSIZE];
        char reply[YuMiConstants::BUFSIZE];
        int idCode = YuMiConstants::ID_SET_SPEED;

        strcpy(message, YuMiCom::setSpeed(idCode, s).c_str());

        if(sendAndReceive(message, strlen(message), reply, idCode)){
            return true;
        } else {
            return false;
        }
    } else {
        std::cerr << "ERROR in YuMiArm setSpeed -> speed must be larger than 0!" << std::endl;
        return false;
    }
}

unsigned int YuMiArm::getSpeed(){
    return speed;
}

bool YuMiArm::getConnected(){
    return connected;
}


