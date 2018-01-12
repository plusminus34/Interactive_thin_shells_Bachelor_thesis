
#include "PhysicalRobotControlApp.h"

#include "../YuMiLib/include/YuMiLib/YuMiArm.h"
#include "../YuMiLib/include/YuMiLib/YuMiConstants.h"

#include <iostream>
#include <string>
#include <vector>
#include <cstdio>

int main(void){

	PhysicalRobotControlApp app;
	app.runMainLoop();

//	std::cout << "Test Main Loop" << std::endl;
//	YuMiArm yumiArmRight;
//	yumiArmRight.init("right");

//	std::vector<float> joints1 = yumiArmRight.getJoints();
//	std::vector<float> joints2 = joints1;
//	joints2[5] += RAD(20);

//	for(int i = 0; i < 7; i++){
//		std::cout << "joint " << (i+1) << ": " << joints1[i] << "   " << joints2[i] << std::endl;
//	}

//	int iter = 100;
//	while(iter < 500){
//		iter += 10;
//		yumiArmRight.setSpeed(iter);
//		yumiArmRight.gotoJointPose(joints2);
//		yumiArmRight.gotoJointPose(joints1);
//	}

//	yumiArmRight.closeConnection();


//	std::cout << "Main Test Right" << std::endl;
//	YuMiArm yumiArmRight;
//	yumiArmRight.init("right");

//	yumiArmRight.setSpeed(YuMiConstants::INIT_SPEED);

//	//std::vector<float> posRight = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
//	//std::vector<float> posRight = YuMiConstants::CALIB_STATE_RIGHT;
//	//std::vector<float> posRight = YuMiConstants::INIT_STATE_RIGHT;
//	std::vector<float> posRight = YuMiConstants::HOME_STATE_RIGHT;
//	yumiArmRight.gotoJointPose(posRight);

//	yumiArmRight.closeConnection();


//	std::cout << "Main Test Left" << std::endl;
//	YuMiArm yumiArmLeft;
//	yumiArmLeft.init("left");

//	yumiArmLeft.setSpeed(YuMiConstants::INIT_SPEED);

//	//std::vector<float> posLeft = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
//	//std::vector<float> posLeft = YuMiConstants::CALIB_STATE_LEFT;
//	//std::vector<float> posLeft = YuMiConstants::INIT_STATE_LEFT;
//	std::vector<float> posLeft = YuMiConstants::HOME_STATE_LEFT;
//	yumiArmLeft.gotoJointPose(posLeft);

//	yumiArmLeft.closeConnection();

    return 0;
}


