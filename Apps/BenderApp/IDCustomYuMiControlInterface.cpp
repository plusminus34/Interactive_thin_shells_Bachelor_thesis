
#include <thread>
#include <chrono>

#include "IDCustomYuMiControlInterface.h"
#include "RobotMount.h"


IDCustomYuMiControlInterface::IDCustomYuMiControlInterface(Robot * robot, RobotParameters * robotParameters) 
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
	setTargetMotorValuesIDRobotParameters(dt);
	//setTargetMotorValuesFromSimRobotState(dt);
	sendControlInputsToPhysicalRobot(dt);
}



void IDCustomYuMiControlInterface::setTargetMotorValuesIDRobotParameters(double dt)
{

	// the generalized coordinates, i.e. 
	dVector q_IDRP;
	robotParameters->pullVec(q_IDRP);
	// set the target motor values of all joints of the robot, using the values provided by the robotParameter class of the Inverse Deformation App
	for(int i = 0; i < robot->getJointCount(); ++i) {
		HingeJoint* hj = dynamic_cast<HingeJoint*>(robot->getJoint(i));
		if (!hj) continue;

		int idx_GCRR = robotParameters->robotParameters->getQIndexForJoint(hj);
		int idx_IDRP = idx_GCRR-6;

		hj->motor.targetMotorAngle = q_IDRP[idx_IDRP];

		//we expect we have dt time to go from the current position to the target position... we ideally want to ensure that the motor gets there exactly dt time from now, so we must limit its velocity...
		double speedLimit = fabs(hj->motor.targetMotorAngle - hj->motor.currentMotorAngle) / dt;
		hj->motor.targetMotorVelocity = speedLimit;
	}

	globalTCPLeft.target = robotParameters->robotParameters->getWorldCoordinatesFor(localTCPLeft, robot->getRBByName("link_7_l"));
	V3D vecTCPLeft = globalTCPLeft.target - globalTCPLeft.current;
	double lengthVecTCPLeft = vecTCPLeft.length();
	tcpSpeedLeft.target = (unsigned int)round(lengthVecTCPLeft*1000.0f/dt*speedWeight);

	if(tcpSpeedLeft.target < minSpeed){
		tcpSpeedLeft.target = minSpeed;
	} else if(tcpSpeedLeft.target > maxSpeed){
		tcpSpeedLeft.target = maxSpeed;
	}

	globalTCPRight.target = robotParameters->robotParameters->getWorldCoordinatesFor(localTCPRight, robot->getRBByName("link_7_r"));
	V3D vecTCPRight = globalTCPRight.target - globalTCPRight.current;
	double lengthVecTCPRight = vecTCPRight.length();
	tcpSpeedRight.target = (unsigned int)round(lengthVecTCPRight*1000.0f/dt*speedWeight);

	if(tcpSpeedRight.target < minSpeed){
		tcpSpeedRight.target = minSpeed;
	} else if(tcpSpeedRight.target > maxSpeed){
		tcpSpeedRight.target = maxSpeed;
	}

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


void IDCustomYuMiControlInterface::streamToRobotTask()
{
	robotStreamTask_isRunning = true;

	while(connected)
	{
		YuMiJointTarget yuMiJointTarget;
		bool target_available = commandBuffer.getAdjustedCommand(yuMiJointTarget);

		if(target_available) {
			if(arm_left_synchronized.valid()) {
				arm_left_synchronized.wait();
			}
			if(arm_right_synchronized.valid()) {
				arm_right_synchronized.wait();
			}
			// send new commands to robot
			arm_left_synchronized = std::async(std::launch::async, &YuMiArm::getAndSendJointsToRobot, 
																	&leftArm, yuMiJointTarget.targetJointsLeft, yuMiJointTarget.targetTime);
			arm_right_synchronized = std::async(std::launch::async, &YuMiArm::getAndSendJointsToRobot, 
																	&rightArm, yuMiJointTarget.targetJointsRight, yuMiJointTarget.targetTime);
		}
		else {
			// wait for further action (buffer is filled, or robot becomes disconnected)
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}

	robotStreamTask_isRunning = false;
}


void YuMiCommandBuffer::push(YuMiJoints targetJointsLeft, YuMiJoints targetJointsRight, double targetTime)
{
	queueAccess.lock();
	targets.push_back(YuMiJointTarget({targetJointsLeft, targetJointsRight, targetTime}));
	queueAccess.unlock();
}


bool YuMiCommandBuffer::getAdjustedCommand(YuMiJointTarget & yuMiJointTarget)
{

	double t_queue;
	queueAccess.lock();
	if(targets.size() > 0) {
		t_queue = bufferedTime();
		yuMiJointTarget = targets.front();
		targets.pop_front();
		queueAccess.unlock();
	}
	else {
		queueAccess.unlock();
		return(false);
	}

	double tgt_delay = target_delay;
	double dt_assumed = 1.0 / assumed_sync_framerate;
	double uncertainty = sync_framerate_uncerteinty;

	double k;
	double time_ratio = std::max(t_queue, 0.01) / tgt_delay;
	if(t_queue < 0.5*tgt_delay) {	// buffer smaller than the target: move to roughly match an estimated frame rate
		double uncertainty_adjustment = uncertainty;// * (1.0 - time_ratio);
		k = (1.0 + uncertainty_adjustment); // slow down a bit when the buffer is close to empty
		yuMiJointTarget.targetTime = k * dt_assumed;
	}
	else {	// buffer us bigger than the target: aim to empty the buffer within the target_delay
		k = 1.0 / time_ratio;
		yuMiJointTarget.targetTime *= k;

	}
	
	return(true);
}


double YuMiCommandBuffer::bufferedTime()
{
	double t_tot = 0.0;
	for(YuMiJointTarget & target : targets) {
		t_tot += target.targetTime;
	}
	return(t_tot);
}

