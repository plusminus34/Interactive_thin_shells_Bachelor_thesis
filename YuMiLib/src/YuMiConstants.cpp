#include <YuMiLib/YuMiConstants.h>
#include "../../MathLib/include/MathLib/MathLib.h"

//Constructor
YuMiConstants::YuMiConstants(){}

//Destructor
YuMiConstants::~YuMiConstants(){}

//Variables
std::string YuMiConstants::IP = "192.168.125.1";

std::vector<float> YuMiConstants::CALIB_STATE_LEFT = {0.0f, -2.26893f, 2.35169f, 0.52360f, 0.0f, 0.69813f, 0.0f};
std::vector<float> YuMiConstants::CALIB_STATE_RIGHT = {0.0f, -2.26893f, -2.35169f, 0.52360f, 0.0f, 0.69813f, 0.0f};

std::vector<float> YuMiConstants::INIT_STATE_LEFT = {-1.86715f, -1.74027f, 0.44331f, -0.46810f, 1.21143f, 1.80642f, 0.28955f};
std::vector<float> YuMiConstants::INIT_STATE_RIGHT = {2.16683f, -1.01159f, -0.94859f, 0.35500f, 4.37432f, 2.21674f, -0.44576f};

std::vector<float> YuMiConstants::HOME_STATE_LEFT = {RADF(-92.65f), RADF(-63.97f), RADF(95.43f), RADF(-36.14f), RADF(-10.77f), RADF(5.22f), RADF(0.37f)};
std::vector<float> YuMiConstants::HOME_STATE_RIGHT = {RADF(92.69f), RADF(-63.02f), RADF(-92.02f), RADF(-36.87f), RADF(187.22f), RADF(-13.07f), RADF(-0.42f)};
