#pragma once

#include <future>
#include <atomic>
#include <queue>

#include "ControlLib/YuMiControlInterface.h"
#include "RobotMount.h"


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
	atomic_int buffer_target_size = 4;
	std::queue<YuMiJointTarget> targets;
	std::mutex queueAccess;

public:
	void push(YuMiJoints targetJointsLeft, YuMiJoints targetJointsRight, double targetTime);

	

};



class IDCustomYuMiControlInterface : public YuMiControlInterface {

protected:
	RobotParameters *robotParameters;

public:
	//GeneralizedCoordinatesRobotRepresentation * robotParameters;	// TODO: remove
	//dVector q_GCRR_last;

	std::future<bool> arm_left_synchronized;
	std::future<bool> arm_right_synchronized;
	atomic_bool robotStreamTask_isRunning = false;
	std::future<void> robotStreamTaskFuture;

	YuMiCommandBuffer commandBuffer;

public:
	IDCustomYuMiControlInterface(Robot * robot, RobotParameters * robotParameters);

	virtual void openCommunicationPort();
	virtual void syncPhysicalRobotWithSimRobot(double dt = 0.1);
	virtual void sendControlInputsToPhysicalRobot(double dt);

	void setTargetMotorValuesIDRobotParameters(double dt);
	
	//void setTargetMotorValuesFromGCRR(double dt);

	void streamToRobotTask();//YuMiCommandBuffer* buffer);


};





