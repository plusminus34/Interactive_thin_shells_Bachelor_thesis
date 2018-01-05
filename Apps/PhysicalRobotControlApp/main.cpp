
#include "PhysicalRobotControlApp.h"

#include "../YuMiLib/include/YuMiLib/YuMiArm.h"
#include "../YuMiLib/include/YuMiLib/YuMiConstants.h"

#include <iostream>
#include <string>
#include <unistd.h>
#include <vector>

int main(void){

    //PhysicalRobotControlApp app;
    //app.runMainLoop();

    // --- TESTS ---
    YuMiArm yumiArmRight("right");
    //YuMiArm yumiArmLeft("left");
    YuMiConstants YMC;
    std::cout << "Main loop" << std::endl;

    while(true){
        bool ping = yumiArmRight.ping();

        std::vector<double> joints(YMC.NUM_JOINTS, 0.00);
        bool state = yumiArmRight.getState(joints);

        std::cout << "J1: " << joints[0] << std::endl;
        std::cout << "J2: " << joints[1] << std::endl;
        std::cout << "J3: " << joints[2] << std::endl;
        std::cout << "J4: " << joints[3] << std::endl;
        std::cout << "J5: " << joints[4] << std::endl;
        std::cout << "J6: " << joints[5] << std::endl;
        std::cout << "J7: " << joints[6] << std::endl;

        usleep(500000);
    }

	return 0;
}


