
#include "PhysicalRobotControlApp.h"

#include "../YuMiLib/include/YuMiLib/YuMiArm.h"
#include "../YuMiLib/include/YuMiLib/YuMiConstants.h"

#include <iostream>
#include <string>
#include <vector>
#include <cstdio>
#include <unistd.h>

int main(void){

	PhysicalRobotControlApp app;
	app.runMainLoop();

    return 0;
}


