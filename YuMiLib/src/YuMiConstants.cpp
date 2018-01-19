#include <YuMiLib/YuMiConstants.h>
#include "../../MathLib/include/MathLib/MathLib.h"

//Constructor
YuMiConstants::YuMiConstants(){}

//Destructor
YuMiConstants::~YuMiConstants(){}

//Variables
std::string YuMiConstants::IP = "192.168.125.1";

std::vector<float> YuMiConstants::CALIB_STATE_LEFT = {0.0f, -2.26893f, 2.35169, 0.52360f, 0.0f, 0.69813f, 0.0f};
std::vector<float> YuMiConstants::CALIB_STATE_RIGHT = {0.0f, -2.26893f, -2.35169, 0.52360f, 0.0f, 0.69813f, 0.0f};

std::vector<float> YuMiConstants::INIT_STATE_LEFT = {-1.86715f, -1.74027f, 0.44331f, -0.46810f, 1.21143f, 1.80642f, 0.28955f};
std::vector<float> YuMiConstants::INIT_STATE_RIGHT = {2.16683f, -1.01159f, -0.94859f, 0.35500f, 4.37432f, 2.21674f, -0.44576f};

std::vector<float> YuMiConstants::HOME_STATE_LEFT = {RAD(-92.65f), RAD(-63.97f), RAD(95.43f), RAD(-36.14f), RAD(-10.77f), RAD(5.22f), RAD(0.37f)};
std::vector<float> YuMiConstants::HOME_STATE_RIGHT = {RAD(92.69f), RAD(-63.02f), RAD(-92.02f), RAD(-36.87f), RAD(187.22f), RAD(-13.07f), RAD(-0.42f)};
