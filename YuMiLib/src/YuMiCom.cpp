#include <YuMiLib/YuMiCom.h>

#include <string.h>
#include <iostream>

//Constructor
YuMiCom::YuMiCom(){}

//Destructor
YuMiCom::~YuMiCom(){}

//Ping robot
std::string YuMiCom::ping(int idCode){
//    char buff[10];
//    std::string msg("00 ");//instruction code;
//    sprintf(buff,"%.3d ",idCode); //identification code
//    msg += buff;
//    msg += "#";

    std::string msg("00 #");//instruction code;

    return (msg);
}

//Get joint values of robot
std::string YuMiCom::getState(int idCode)
{
//    char buff[10];
//    std::string msg("04 ");//instruction code;

//    sprintf(buff,"%.3d ",idCode); //identification code
//    msg += buff;
//    msg += "#";

    std::string msg("04 #");

    return (msg);
}
