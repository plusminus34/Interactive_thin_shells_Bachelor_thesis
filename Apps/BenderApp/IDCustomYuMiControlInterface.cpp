
#include <thread>
#include <chrono>

#include "IDCustomYuMiControlInterface.h"


IDCustomYuMiControlInterface::IDCustomYuMiControlInterface(Robot * robot) 
	: YuMiControlInterface(robot)
{
	globalTCPLeft.target = V3D(0.0, 0.0, 0.0);
	globalTCPRight.target = V3D(0.0, 0.0, 0.0);
	globalTCPLeft.current = V3D(0.0, 0.0, 0.0);
	globalTCPRight.current = V3D(0.0, 0.0, 0.0);

}


void IDCustomYuMiControlInterface::openCommunicationPort()
{
	YuMiControlInterface::openCommunicationPort();

std::cout << "communication port is opened" << std::endl;
	if(connected) {
		if(!robotStreamTask_isRunning) {
			robotStreamTaskFuture = std::async(std::launch::async, &IDCustomYuMiControlInterface::streamToRobotTask, this);
		}
	}
}

void IDCustomYuMiControlInterface::syncPhysicalRobotWithSimRobot(double dt) 
{
	//setTargetMotorValuesFromGCRR(dt);
	setTargetMotorValuesFromSimRobotState(dt);
	sendControlInputsToPhysicalRobot(dt);
}




void IDCustomYuMiControlInterface::sendControlInputsToPhysicalRobot(double dt) {

	//Get right and left joint targets
	YuMiJoints rightTargetJoints, leftTargetJoints;

	float* rightTargetJointsPtr = &rightTargetJoints.j1;
	float* leftTargetJointsPtr = &leftTargetJoints.j1;

	for (int i = 0; i < robot->getJointCount(); i++) {
		HingeJoint* hj = dynamic_cast<HingeJoint*>(robot->getJoint(i));
		if (!hj) continue;

		if(i % 2 == 0){
			*rightTargetJointsPtr = hj->motor.targetMotorAngle;
			rightTargetJointsPtr++;
		} else {
			*leftTargetJointsPtr = hj->motor.targetMotorAngle;
			leftTargetJointsPtr++;
		}
	}

	
	commandBuffer.push(leftTargetJoints, rightTargetJoints, dt);

}


void IDCustomYuMiControlInterface::streamToRobotTask()//YuMiCommandBuffer* buffer)
{
	robotStreamTask_isRunning = true;

	while(connected)
	{
		commandBuffer.queueAccess.lock();
		int buffer_size = commandBuffer.targets.size();
		if(buffer_size > 0) {
			// get data from buffer
			YuMiJointTarget yuMiJointTarget = commandBuffer.targets.front();
			commandBuffer.targets.pop();

			commandBuffer.queueAccess.unlock();

			// compute control factor for time, to get closer to the target buffer size
			double target_size = static_cast<double>(commandBuffer.buffer_target_size);
			double size = static_cast<double>(buffer_size);
			double k = std::pow(target_size/(std::max(1.0,size)), 1.0);
			double t = k * yuMiJointTarget.targetTime;
			//std::cout << "streaming: n_buff = " << buffer_size << "  k = " << k << "  t/t_new = " << yuMiJointTarget.targetTime << " / " << t << std::endl;
			// wait for previous robot-communications to finish
			if(arm_left_synchronized.valid()) {
				arm_left_synchronized.wait();
			}
			if(arm_right_synchronized.valid()) {
				arm_right_synchronized.wait();
			}
			// send new commands to robot
			arm_left_synchronized = std::async(std::launch::async, &YuMiArm::getAndSendJointsToRobot, &leftArm, yuMiJointTarget.targetJointsLeft, t);
			arm_right_synchronized = std::async(std::launch::async, &YuMiArm::getAndSendJointsToRobot, &rightArm, yuMiJointTarget.targetJointsRight, t);
		}
		else {
			commandBuffer.queueAccess.unlock();
			// wait for further action (buffer is filled, or robot becomes disconnected)
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}

	robotStreamTask_isRunning = false;
}



void YuMiCommandBuffer::push(YuMiJoints targetJointsLeft, YuMiJoints targetJointsRight, double targetTime)
{
	queueAccess.lock();
	targets.push(YuMiJointTarget({targetJointsLeft, targetJointsRight, targetTime}));
	queueAccess.unlock();
}




