#pragma once

#include <future>
#include <atomic>
#include <queue>

#include "ControlLib/YuMiControlInterface.h"
#include "ControlLib/GeneralizedCoordinatesRobotRepresentation.h"


class IDCustomYuMiControlInterface;


class YuMiJointTarget 
{
public:
	YuMiJoints targetJointsLeft, targetJointsRight;
	double targetTime;
};

class YuMiCommandBuffer 
{
	friend IDCustomYuMiControlInterface;

protected:
	atomic_int buffer_target_size = 3;
	std::queue<YuMiJointTarget> targets;
	std::mutex queueAccess;

public:
	void push(YuMiJoints targetJointsLeft, YuMiJoints targetJointsRight, double targetTime);

	

};



class IDCustomYuMiControlInterface : public YuMiControlInterface {

public:
	GeneralizedCoordinatesRobotRepresentation * robotParameters;	// TODO: remove
	dVector q_GCRR_last;

	std::future<bool> arm_left_synchronized;
	std::future<bool> arm_right_synchronized;
	atomic_bool robotStreamTask_isRunning = false;

	YuMiCommandBuffer commandBuffer;

public:
	IDCustomYuMiControlInterface(Robot * robot, GeneralizedCoordinatesRobotRepresentation * robotParameters);

	virtual void openCommunicationPort();
	virtual void syncPhysicalRobotWithSimRobot(double dt = 0.1);
	virtual void sendControlInputsToPhysicalRobot(double dt);
	
	//void setTargetMotorValuesFromGCRR(double dt);

	void streamToRobotTask();//YuMiCommandBuffer* buffer);


};





