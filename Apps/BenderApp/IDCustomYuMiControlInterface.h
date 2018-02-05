#pragma once

#include <future>

#include "ControlLib/YuMiControlInterface.h"
#include "ControlLib/GeneralizedCoordinatesRobotRepresentation.h"



class IDCustomYuMiControlInterface : public YuMiControlInterface {

public:
	GeneralizedCoordinatesRobotRepresentation * robotParameters;
	dVector q_GCRR_last;

	std::future<void> arm_left_synchronized;
	std::future<void> arm_right_synchronized;

public:
	IDCustomYuMiControlInterface(Robot * robot, GeneralizedCoordinatesRobotRepresentation * robotParameters);

	virtual void syncPhysicalRobotWithSimRobot(double dt = 0.1);
	virtual void sendControlInputsToPhysicalRobot();
	
	void setTargetMotorValuesFromGCRR(double dt);




};