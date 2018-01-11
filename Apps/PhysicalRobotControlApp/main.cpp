
#include "PhysicalRobotControlApp.h"

#include "../YuMiLib/include/YuMiLib/YuMiArm.h"
#include "../YuMiLib/include/YuMiLib/YuMiConstants.h"

#include <iostream>
#include <string>
#include <vector>

int main(void){

	PhysicalRobotControlApp app;
	app.runMainLoop();

	//	std::cout << "Test Main Loop" << std::endl;
	//    YuMiArm yumiArmRight;
	//    yumiArmRight.init("right");

	//    std::vector<float> joints1 = yumiArmRight.getJoints();
	//    std::vector<float> joints2 = joints1;
	//    joints2[1] += RAD(20);

	//    for(int i = 0; i < 7; i++){
	//        std::cout << "joint " << (i+1) << ": " << joints1[i] << "   " << joints2[i] << std::endl;
	//    }

	//    for(int i = 0; i < 2; i++){
	//        yumiArmRight.gotoJointPose(joints2);
	//        yumiArmRight.gotoJointPose(joints1);
	//    }

	//    yumiArmRight.closeConnection();

    return 0;
}


