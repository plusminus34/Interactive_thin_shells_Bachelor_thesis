#pragma once

#include <string>


class YuMiCom{
private:
    unsigned int buffSize = 10;

public:
    //Constructor
    YuMiCom();

    //Destructor
    ~YuMiCom();

    //Functions
    std::string ping(int idCode);
    std::string getState(int idCode);
};
