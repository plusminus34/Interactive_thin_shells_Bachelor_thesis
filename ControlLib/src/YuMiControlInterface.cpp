#include <ControlLib/YuMiControlInterface.h>

#include <iostream>

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>


//Constructor
YuMiControlInterface::YuMiControlInterface(Robot* robot) : RobotControlInterface(robot) {}

//Destructor
YuMiControlInterface::~YuMiControlInterface() {}

//set motor goals from target values
void YuMiControlInterface::sendControlInputsToPhysicalRobot(double dt) {

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

	//Send position and velocity commands to robot
	if(movementCheck(rightTargetJoints, savedRightJoints)){
		rightArm.sendRobotToJointPose(rightTargetJoints, dt, false);
	}

	if(movementCheck(leftTargetJoints, savedLeftJoints)){
		leftArm.sendRobotToJointPose(leftTargetJoints, dt, false);
	}

	//Update current values
	savedLeftJoints = leftTargetJoints;
	savedRightJoints = rightTargetJoints;

	globalTCPLeft.current = globalTCPLeft.target;
	globalTCPRight.current = globalTCPRight.target;

	tcpSpeedLeft.current = tcpSpeedLeft.target;
	tcpSpeedRight.current = tcpSpeedRight.target;
}

//read motor positions
void YuMiControlInterface::readPhysicalRobotMotorPositions() {
	YuMiJoints rightCurrentJoints = rightArm.getCurrentJointValues();
	YuMiJoints leftCurrentJoints = leftArm.getCurrentJointValues();

	float* rightCurrentJointsPtr = &rightCurrentJoints.j1;
	float* leftCurrentJointsPtr = &leftCurrentJoints.j1;

	for (int i = 0; i < robot->getJointCount(); i++) {
		HingeJoint* hj = dynamic_cast<HingeJoint*>(robot->getJoint(i));
		if (!hj) continue;

		float tempJoint = 0.0f;
		if(i % 2 == 0){
			tempJoint = *rightCurrentJointsPtr;
			rightCurrentJointsPtr++;
		} else {
			tempJoint = *leftCurrentJointsPtr;
			leftCurrentJointsPtr++;
		}

		hj->motor.currentMotorAngle = tempJoint;
	}
}

//read motor positions
void YuMiControlInterface::readPhysicalRobotMotorVelocities() {}

void YuMiControlInterface::setTargetMotorValuesFromSimRobotState(double dt) {
	//readPhysicalRobotMotorPositions();  //optional!

    //given the values stored in the joint's dxl properties structure (which are updated either from the menu or by sync'ing with the dynamixels), update the state of the robot...
    RobotState rs(robot);

    for (int i = 0; i < robot->getJointCount(); i++) {
        HingeJoint* hj = dynamic_cast<HingeJoint*>(robot->getJoint(i));
        if (!hj) continue;
        Quaternion q = rs.getJointRelativeOrientation(i);
        V3D w = rs.getJointRelativeAngVelocity(i);
        hj->motor.targetMotorAngle = q.getRotationAngle(hj->rotationAxis);

        //we expect we have dt time to go from the current position to the target position... we ideally want to ensure that the motor gets there exactly dt time from now, so we must limit its velocity...
        double speedLimit = fabs(hj->motor.targetMotorAngle - hj->motor.currentMotorAngle) / dt;
        hj->motor.targetMotorVelocity = speedLimit;
    }

	//Set tcpSpeed
	globalTCPLeft.target = robot->getRBByName("link_7_l")->getWorldCoordinates(localTCPLeft);
	V3D vecTCPLeft = globalTCPLeft.target - globalTCPLeft.current;
	lengthVecTCPLeft = vecTCPLeft.length();
	tcpSpeedLeft.target = (unsigned int)round(lengthVecTCPLeft*1000.0f/dt*speedWeight);

	if(tcpSpeedLeft.target < minSpeed){
		tcpSpeedLeft.target = minSpeed;
	} else if(tcpSpeedLeft.target > maxSpeed){
		tcpSpeedLeft.target = maxSpeed;
	}
	//tcpSpeedLeft.target = 1000000; //RESET

	globalTCPRight.target = robot->getRBByName("link_7_r")->getWorldCoordinates(localTCPRight);
	V3D vecTCPRight = globalTCPRight.target - globalTCPRight.current;
	lengthVecTCPRight = vecTCPRight.length();
	tcpSpeedRight.target = (unsigned int)round(lengthVecTCPRight*1000.0f/dt*speedWeight);

	if(tcpSpeedRight.target < minSpeed){
		tcpSpeedRight.target = minSpeed;
	} else if(tcpSpeedRight.target > maxSpeed){
		tcpSpeedRight.target = maxSpeed;
	}
	//tcpSpeedRight.target = 1000000; //RESET


//	std::cout << "TCPSpeed Target Left: " << tcpSpeedLeft.target << std::endl;
//	std::cout << "TCPSpeed Target Right: " << tcpSpeedRight.target << std::endl;

}

void YuMiControlInterface::openCommunicationPort() {
	bool leftConnect = false;
	bool rightConnect = false;

	if(leftArm.getConnectedValue() == false){
		leftConnect = leftArm.init("left");
    }
	if(rightArm.getConnectedValue() == false){
		rightConnect = rightArm.init("right");
    }

	if(leftConnect && !rightConnect){
		leftArm.closeConnection();
		leftConnect = false;
	} else if(!leftConnect && rightConnect){
		rightArm.closeConnection();
		rightConnect = false;
	}

	if(leftConnect && rightConnect){
		connected = true;
	}
}

void YuMiControlInterface::closeCommunicationPort() {
    leftArm.closeConnection();
    rightArm.closeConnection();
	connected = false;
}

void YuMiControlInterface::driveMotorPositionsToZero(){}

void YuMiControlInterface::driveMotorPositionsToTestPos1(IK_Solver* ikSolverPtr){
	std::cout << "driveMotorPositionsToTestPos1" << std::endl;

	std::vector<float> leftTargetJoints = YuMiConstants::HOME_STATE_LEFT;
	std::vector<float> rightTargetJoints = YuMiConstants::HOME_STATE_RIGHT;

	driveMotorPositionsToInputPos(leftTargetJoints, rightTargetJoints, ikSolverPtr);
}


void YuMiControlInterface::driveMotorPositionsToTestPos2(IK_Solver* ikSolverPtr){
	std::cout << "driveMotorPositionsToTestPos2" << std::endl;

	std::vector<float> leftTargetJoints = YuMiConstants::INIT_STATE_LEFT;
	//std::vector<float> rightTargetJoints = YuMiConstants::INIT_STATE_RIGHT;
	std::vector<float> rightTargetJoints = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

	driveMotorPositionsToInputPos(leftTargetJoints, rightTargetJoints, ikSolverPtr);
}

void YuMiControlInterface::driveMotorPositionsToInputPos(std::vector<float> leftJoints, std::vector<float> rightJoints, IK_Solver* ikSolverPtr){
	RobotState rs(robot);

	int jointIndex = 0;
	for (int i = 0; i < robot->getJointCount(); i++) {
		HingeJoint* hj = dynamic_cast<HingeJoint*>(robot->getJoint(i));
		if (!hj) continue;

		float angleNext = 0.0f;
		if(i % 2 == 0){
			angleNext = rightJoints.at(jointIndex);

		} else {
			angleNext = leftJoints.at(jointIndex);
			jointIndex++;
		}
		rs.setJointRelativeOrientation(getRotationQuaternion(angleNext, hj->rotationAxis), hj->jIndex);
	}

	//robot->setState(&rs);
	ikSolverPtr->ikPlan->setTargetIKState(rs);
}

void YuMiControlInterface::grip(std::string arm){
	if(arm.compare("right") == 0){
		if(rightArm.getGripperOpenValue()){
			rightArm.closeGripper();
		} else {
			rightArm.openGripper();
		}
	} else if(arm.compare("left") == 0){
		if(leftArm.getGripperOpenValue()){
			leftArm.closeGripper();
		} else {
			leftArm.openGripper();
		}
	}
}

void YuMiControlInterface::printJointValues(){
	std::vector<float> rightCurrentJoints(robot->getJointCount()/2, 0.0f);
	std::vector<float> leftCurrentJoints(robot->getJointCount()/2, 0.0f);

	std::vector<float> rightTargetJoints(robot->getJointCount()/2, 0.0f);
	std::vector<float> leftTargetJoints(robot->getJointCount()/2, 0.0f);

	int jointIndex = 0;
	for (int i = 0; i < robot->getJointCount(); i++) {
		HingeJoint* hj = dynamic_cast<HingeJoint*>(robot->getJoint(i));
		if (!hj) continue;

		if(i % 2 == 0){
			rightCurrentJoints.at(jointIndex) = hj->motor.currentMotorAngle;
			rightTargetJoints.at(jointIndex) = hj->motor.targetMotorAngle;
		} else {
			leftCurrentJoints.at(jointIndex) = hj->motor.currentMotorAngle;
			leftTargetJoints.at(jointIndex) = hj->motor.targetMotorAngle;
			jointIndex++;
		}
	}

	std::cout << "CURRENT: Right joint angles: ";
	for(int i = 0; i < 7; i++){
		std::cout << "J" << (i+1) << ": " << rightCurrentJoints.at(i) << "   /   ";
	}
	std::cout << "" << std::endl;

	std::cout << "TARGET: Right joint angles: ";
	for(int i = 0; i < 7; i++){
		std::cout << "J" << (i+1) << ": " << rightTargetJoints.at(i) << "   /   ";
	}
	std::cout << "" << std::endl;

//	std::cout << "CURRENT: Left joint angles: ";
//	for(int i = 0; i < 7; i++){
//		std::cout << "J" << (i+1) << ": " << leftCurrentJoints.at(i) << "   /   ";
//	}
//	std::cout << "" << std::endl;

//	std::cout << "TARGET: Left joint angles: ";
//	for(int i = 0; i < 7; i++){
//		std::cout << "J" << (i+1) << ": " << leftTargetJoints.at(i) << "   /   ";
//	}
//	std::cout << "" << std::endl;
}

bool YuMiControlInterface::sendJointInputsCheck(std::string arm){
	bool sendInput = false;
	double tol = 0.001f;
	for (int i = 0; i < robot->getJointCount(); i++) {
		HingeJoint* hj = dynamic_cast<HingeJoint*>(robot->getJoint(i));
		if (!hj) continue;

		if(arm.compare("right") == 0 && (i % 2 == 0)){
			double delta = fabs(hj->motor.currentMotorAngle - hj->motor.targetMotorAngle);
			if(delta > tol){
				sendInput = true;
				break;
			}
		} else if(arm.compare("left") == 0 && (i % 2 == 1)){
			double delta = fabs(hj->motor.currentMotorAngle - hj->motor.targetMotorAngle);
			if(delta > tol){
				sendInput = true;
				break;
			}
		}
	}
	return sendInput;
}

bool YuMiControlInterface::sendSpeedInputCheck(){
	bool sendInput = false;
	double tol = 0.001f;
	HingeJoint* hj = dynamic_cast<HingeJoint*>(robot->getJoint(0));
	//double delta = fabs(hj->motor.currenttcpSpeed - hj->motor.targettcpSpeed);
	double delta = 0.0f;
	if(delta > tol){
		sendInput = true;
	}
	return sendInput;
}

bool YuMiControlInterface::movementCheck(YuMiJoints targetJoints, YuMiJoints savedJoints){
	float* targetJointsPtr = &targetJoints.j1;
	float* savedJointsPtr = &savedJoints.j1;

	for(int i = 0; i < YuMiConstants::NUM_JOINTS; i++){
		if(*targetJointsPtr != *savedJointsPtr){
			return true;
		}
		targetJointsPtr++;
		savedJointsPtr++;
	}

	return false;
}


