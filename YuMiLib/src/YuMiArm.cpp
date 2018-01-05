#include <YuMiLib/YuMiArm.h>

#include <string.h>
#include <cstring>
#include <iostream>
#include <arpa/inet.h>
#include <fcntl.h>


//Constructor
YuMiArm::YuMiArm(std::string arm){
    init(arm);
}


//Destructor
YuMiArm::~YuMiArm(){}


//Initialize object
void YuMiArm::init(std::string arm){
    //Set constants
    armSide = arm;
    const char* ip = YMC.IP;
    unsigned int port = 0;
    if(arm.compare("right") == 0){
        port = YMC.PORT_RIGHT_SERVER;
        std::cout << "port: " << port << std::endl;
    } else if(arm.compare("left") == 0){
        port = YMC.PORT_LEFT_SERVER;
        std::cout << "port: " << port << std::endl;
    } else {
        std::cout << "This arm does not exist!" << std::endl;
        return;
    }

    //Connect arm to server
    socketConnected = connectServer(ip, port);
}


//Open socket and connect
bool YuMiArm::connectServer(const char* ip, unsigned int port) {
    std::cout << "connectServer started..." << std::endl;

    // Init bool
    socketConnected = false;

    // Create a socket for robot
    if ((robotSocket = socket(PF_INET, SOCK_STREAM, 0)) == -1)
        std::cout << "Problem creating the socket" << std::endl;
    else {
        // Now try to connect to the robot server
        struct sockaddr_in remoteSocket;
        remoteSocket.sin_family = AF_INET;
        remoteSocket.sin_port = htons(port);
        inet_pton(AF_INET, ip, &remoteSocket.sin_addr.s_addr);
        if(connect(robotSocket, (sockaddr*)&remoteSocket, sizeof(remoteSocket)) == -1){
            std::cout << "Could not connect to the ABB robot" << std::endl;
        } else {
            socketConnected = true;
            std::cout << "Successfully connected to the ABB robot" << std::endl;
        }
    }

    return socketConnected;
}


//Send message to robot and receive answer
bool YuMiArm::sendAndReceive(char *message, int messageLength, char* reply, int idCode){
    bool success = false;

    pthread_mutex_lock(&sendRecvMutex);
    if (send(robotSocket, message, messageLength, 0) == -1){
        std::cout << "Failed to send command to robot" << std::endl;
    } else {
        // Read the reply to the message we just sent, and make sure
        // it's not corrupt, and the command was executed successfully
        int t;
        if ((t=recv(robotSocket, reply, YMC.BUFSIZE-1, 0)) > 0) {
            reply[t] = '\0';
            int ok, rcvIdCode;
            sscanf(reply,"%d %d", &rcvIdCode, &ok);
            if(idCode!=-1) {
                if ((ok == YMC.SERVER_OK) && (rcvIdCode == idCode)) {
                    pthread_mutex_unlock(&sendRecvMutex);
                    success = true;
                } else if ((ok == YMC.SERVER_COLLISION) && (rcvIdCode == idCode)) {
                    std::cout << "WARNING: Collision Detected" << std::endl;
                } else {
                    std::cout << "WARNING: Corrupt message 1:" << std::endl;
                    std::cout << "msg = " << message << ";  reply = " << reply << ";  idCode = " << idCode << ";  ok = " << ok << ";  rcvCode = " << rcvIdCode << std::endl;
                }
            } else {
                if (ok == YMC.SERVER_OK) {
                    pthread_mutex_unlock(&sendRecvMutex);
                    success = true;
                } else if (ok == YMC.SERVER_COLLISION) {
                    std::cout << "WARNING: Collision Detected" << std::endl;
                } else {
                    std::cout << "WARNING: Corrupt message 2:" << std::endl;
                    std::cout << "msg = " << message << ";  reply = " << reply << ";  idCode = " << idCode << ";  ok = " << ok << ";  rcvCode = " << rcvIdCode << std::endl;
                }
            }
        } else {
            std::cout << "WARNING: Failed to receive answer from robot" << std::endl;
        }
    }

    pthread_mutex_unlock(&sendRecvMutex);

    return success;
}

//Ping robot
bool YuMiArm::ping(){
    char message[YMC.BUFSIZE];
    char reply[YMC.BUFSIZE];
    int idCode = 0;

    strcpy(message, yumiCom.ping(idCode).c_str());

    if(sendAndReceive(message, strlen(message), reply, idCode)){
        std::cout << "reply: " << reply << std::endl;
        return true;
    } else {
        return false;
    }
}


// Query robot for the current joint positions
bool YuMiArm::getState(std::vector<double> &joints){
    char message[YMC.BUFSIZE];
    char reply[YMC.BUFSIZE];
    int idCode = 4;

    strcpy(message, yumiCom.getState(idCode).c_str());

    if(sendAndReceive(message, strlen(message), reply, idCode)){

        //Parse joints
        parseMessage(reply, joints);

        return true;
    } else {
        return false;
    }
}

void YuMiArm::parseMessage(std::string message, std::vector<double> &parsedValues){
    std::vector<double> testVector(7, 0.00);
    char * cstr = new char [message.length()+1];
    std::strcpy (cstr, message.c_str());
    char * p = std::strtok (cstr," ");
    int iter = 0;
    while(p!=0){
        if(iter > 1){ //1. = idCode, 2. = ok
            parsedValues[iter-2] = atof(p);
        }
        p = std::strtok(NULL," ");
        iter++;
    }
    delete[] cstr;
}

