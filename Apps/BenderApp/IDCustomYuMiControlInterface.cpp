
#include <thread>
#include <chrono>

#include "IDCustomYuMiControlInterface.h"


IDCustomYuMiControlInterface::IDCustomYuMiControlInterface(Robot * robot, GeneralizedCoordinatesRobotRepresentation * robotParameters) 
	: YuMiControlInterface(robot), robotParameters(robotParameters)
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



// TODO: remove (cannot work as expected)
/*
void IDCustomYuMiControlInterface::setTargetMotorValuesFromGCRR(double dt)
{

	// the generalized coordinates, i.e. 
	dVector q_GCRR;
	robotParameters->getQ(q_GCRR);
	// set the target motor values of all joints of the robot, using the values provided by the GeneralizedCoordinatesRobotRepresentation
	for(int i = 0; i < robot->getJointCount(); ++i) {
		HingeJoint* hj = dynamic_cast<HingeJoint*>(robot->getJoint(i));
		if (!hj) continue;

		int idx_GCRR = robotParameters->getQIndexForJoint(hj);
		
		if(q_GCRR_last.size() > 0) {

			double delta_q = q_GCRR[idx_GCRR] - q_GCRR_last[idx_GCRR];
			if(delta_q > PI) {
				delta_q -= 2.0*PI;
			}
			else if(delta_q < -PI) {
				delta_q += 2.0*PI;
			}
			q_GCRR[idx_GCRR] = q_GCRR_last[idx_GCRR] + delta_q;
			hj->motor.targetMotorAngle = q_GCRR[idx_GCRR];

		}

		hj->motor.targetMotorAngle = q_GCRR[idx_GCRR];

		//we expect we have dt time to go from the current position to the target position... we ideally want to ensure that the motor gets there exactly dt time from now, so we must limit its velocity...
		double speedLimit = fabs(hj->motor.targetMotorAngle - hj->motor.currentMotorAngle) / dt;
		hj->motor.targetMotorVelocity = speedLimit;
	}
	q_GCRR_last = q_GCRR;

	//Set tcpSpeed
	globalTCPLeft.current = globalTCPLeft.target;
	globalTCPRight.current = globalTCPRight.target;
	//tcpSpeedLeft.current = tcpSpeedLeft.target;
	//tcpSpeedRight.current = tcpSpeedRight.target;

	globalTCPLeft.target = robotParameters->getWorldCoordinatesFor(localTCPLeft, robot->getRBByName("link_7_l"));
	V3D vecTCPLeft = globalTCPLeft.target - globalTCPLeft.current;
	double lengthVecTCPLeft = vecTCPLeft.length();
	tcpSpeedLeft.target = (unsigned int)round(lengthVecTCPLeft*1000.0f/dt*speedWeight);

	if(tcpSpeedLeft.target < minSpeed){
		tcpSpeedLeft.target = minSpeed;
	} else if(tcpSpeedLeft.target > maxSpeed){
		tcpSpeedLeft.target = maxSpeed;
	}

	globalTCPRight.target = robotParameters->getWorldCoordinatesFor(localTCPRight, robot->getRBByName("link_7_r"));
	V3D vecTCPRight = globalTCPRight.target - globalTCPRight.current;
	double lengthVecTCPRight = vecTCPRight.length();
	tcpSpeedRight.target = (unsigned int)round(lengthVecTCPRight*1000.0f/dt*speedWeight);

	if(tcpSpeedRight.target < minSpeed){
		tcpSpeedRight.target = minSpeed;
	} else if(tcpSpeedRight.target > maxSpeed){
		tcpSpeedRight.target = maxSpeed;
	}

}
*/


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
	
	/*
	// check if old robot-communication is still running, wait if necessary
	if(arm_left_synchronized.valid()) {
		std::cout << "waiting for left arm ... ";
		arm_left_synchronized.wait();
		std::cout << "done" << std::endl;
	}

	if(arm_right_synchronized.valid()) {
		std::cout << "waiting for right arm ... ";
		arm_right_synchronized.wait();
		std::cout << "done" << std::endl;
	}

	//Send commands to robot, don't wait for them to be executed
	
	auto getAndSendArm = [](YuMiArm * arm, YuMiJoints targetJoints, double targetTime)
	{
		arm->getAndSendJointsToRobot(targetJoints, targetTime);
	};
	
std::cout << "tcpSpeeds r/l : " << tcpSpeedRight.target << " " << tcpSpeedLeft.target << std::endl;
	
	arm_left_synchronized = std::async(getAndSendArm, &leftArm, leftTargetJoints, dt);
	arm_right_synchronized = std::async(getAndSendArm, &rightArm, rightTargetJoints, dt);
	*/

	
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
std::cout << "streaming: n_buff = " << buffer_size << "  k = " << k << "  t/t_new = " 
		  << yuMiJointTarget.targetTime << " / " << t << std::endl;
			// wait for previous robot-communications to finish
			if(arm_left_synchronized.valid()) {
				std::cout << "waiting for left arm ... ";
				arm_left_synchronized.wait();
				std::cout << "done" << std::endl;
			}
			if(arm_right_synchronized.valid()) {
				std::cout << "waiting for right arm ... ";
				arm_right_synchronized.wait();
				std::cout << "done" << std::endl;
			}
			// send new commands to robot
			arm_left_synchronized = std::async(std::launch::async, &YuMiArm::getAndSendJointsToRobot, &leftArm, yuMiJointTarget.targetJointsLeft, t);
			arm_right_synchronized = std::async(std::launch::async, &YuMiArm::getAndSendJointsToRobot, &rightArm, yuMiJointTarget.targetJointsRight, t);
		}
		else {
			std::cout <<"    no commands in buffer" << std::endl;
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




