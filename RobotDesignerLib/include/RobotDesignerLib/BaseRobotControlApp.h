#pragma once

#include <GUILib/GLApplication.h>
#include <ControlLib/Robot.h>


/**
 * Robot Design and Simulation interface
 */
class BaseRobotControlApp : public GLApplication {
public:
	Robot* robot = NULL;
	RobotState startingRobotState;

	double slowMoFactor = 5.0;


public:
	// constructor
	BaseRobotControlApp() {}
	// destructor
	virtual ~BaseRobotControlApp(void) {}
};




