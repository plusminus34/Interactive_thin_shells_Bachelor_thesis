#pragma once

#include "YuMiConstants.h"
#include "YuMiCom.h"

#include <string>
#include <vector>


class YuMiArm{
private:
    std::string armSide;
    int robotSocket;
    bool socketConnected;
    pthread_mutex_t sendRecvMutex;

    YuMiConstants YMC;
    YuMiCom yumiCom;

public:
    //Constructor
    YuMiArm(std::string arm);

    //Destructor
    ~YuMiArm();

    //Functions
    void init(std::string arm);
    bool connectServer(const char* ip, unsigned int port);
    bool ping();
    bool sendAndReceive(char *message, int messageLength, char* reply, int idCode);

    bool getState(std::vector<double> &joints);

    void parseMessage(std::string message, std::vector<double> &parsedValues);

};
