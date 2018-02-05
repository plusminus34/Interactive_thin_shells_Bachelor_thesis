

#include "IDCustomYuMiControlInterface.h"


IDCustomYuMiControlInterface::IDCustomYuMiControlInterface(Robot * robot, GeneralizedCoordinatesRobotRepresentation * robotParameters) 
	: YuMiControlInterface(robot), robotParameters(robotParameters)
{
	globalTCPLeft.target = V3D(0.0, 0.0, 0.0);
	globalTCPRight.target = V3D(0.0, 0.0, 0.0);
	globalTCPLeft.current = V3D(0.0, 0.0, 0.0);
	globalTCPRight.current = V3D(0.0, 0.0, 0.0);

}




void IDCustomYuMiControlInterface::syncPhysicalRobotWithSimRobot(double dt) 
{
	setTargetMotorValuesFromGCRR(dt);
	//setTargetMotorValuesFromSimRobotState(dt);
	sendControlInputsToPhysicalRobot();
}




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

			hj->motor.targetMotorAngle = q_GCRR_last[idx_GCRR] + delta_q;

		}
		else {
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


void IDCustomYuMiControlInterface::sendControlInputsToPhysicalRobot() {

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
	
	auto getAndSendArm = [](YuMiArm * arm, YuMiJoints targetJoints, unsigned int targetSpeed)
	{
		arm->getAndSendJointsAndTCPSpeedToRobot(targetJoints, targetSpeed);
	};
	
std::cout << "tcpSpeeds r/l : " << tcpSpeedRight.target << " " << tcpSpeedLeft.target << std::endl;
	
	arm_left_synchronized = std::async(getAndSendArm, &leftArm, leftTargetJoints, tcpSpeedLeft.target);
	arm_right_synchronized = std::async(getAndSendArm, &rightArm, rightTargetJoints, tcpSpeedRight.target);

}