#include <ControlLib/YuMiControlInterface.h>

#include <iostream>


//Constructor
YuMiControlInterface::YuMiControlInterface(Robot* robot) : RobotControlInterface(robot) {}

//Destructor
YuMiControlInterface::~YuMiControlInterface() {}

//set motor goals from target values
void YuMiControlInterface::sendControlInputsToPhysicalRobot() {

	if(sendJointInputsCheck()){
		std::vector<float> rightTargetJoints(robot->getJointCount()/2, 0.0);
		std::vector<float> leftTargetJoints(robot->getJointCount()/2, 0.0);

		unsigned int speed = 1;

		int jointIndex = 0;
		for (int i = 0; i < robot->getJointCount(); i++) {
			HingeJoint* hj = dynamic_cast<HingeJoint*>(robot->getJoint(i));
			if (!hj) continue;

			if(i % 2 == 0){
				rightTargetJoints.at(jointIndex) = hj->motor.targetMotorAngle;
			} else {
				leftTargetJoints.at(jointIndex) = hj->motor.targetMotorAngle;
				//std::cout << "leftTargetJoints: " << hj->motor.targetMotorAngle << std::endl;
				jointIndex++;
			}
		}

		rightArm.gotoJointPose(rightTargetJoints);
		leftArm.gotoJointPose(leftTargetJoints);
	}

	if(sendSpeedInputCheck()){
		HingeJoint* hj = dynamic_cast<HingeJoint*>(robot->getJoint(0));
		unsigned int speed = hj->motor.targetYuMiTCPSpeed;
		rightArm.setSpeed(speed);
		leftArm.setSpeed(speed);
		hj->motor.currentYuMiTCPSpeed = speed;
	}
}

//read motor positions
void YuMiControlInterface::readPhysicalRobotMotorPositions() {
    //std::cout << "readPhysicalRobotMotorPositions" << std::endl;

	std::vector<float> rightJoints = rightArm.getJoints();
	std::vector<float> leftJoints = leftArm.getJoints();
	//std::vector<float> rightJoints = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	//std::vector<float> leftJoints = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    int jointIndex = 0;
    for (int i = 0; i < robot->getJointCount(); i++) {
        HingeJoint* hj = dynamic_cast<HingeJoint*>(robot->getJoint(i));
        if (!hj) continue;

        float tempJoint = 0.0;
        if(i % 2 == 0){
            tempJoint = rightJoints.at(jointIndex);
        } else {
            tempJoint = leftJoints.at(jointIndex);
            jointIndex++;
        }

        hj->motor.currentMotorAngle = tempJoint;
    }
}

//read motor positions
void YuMiControlInterface::readPhysicalRobotMotorVelocities() {}

void YuMiControlInterface::setTargetMotorValuesFromSimRobotState(double dt) {
    readPhysicalRobotMotorPositions();

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
}

void YuMiControlInterface::openCommunicationPort() {
	bool leftConnect = false;
	bool rightConnect = false;
	if(leftArm.getConnected() == false){
		leftConnect = leftArm.init("left");
    }
	if(rightArm.getConnected() == false){
		rightConnect = rightArm.init("right");
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
		if(rightArm.getGripperOpen()){
			rightArm.closeGripper();
		} else {
			rightArm.openGripper();
		}
	} else if(arm.compare("left") == 0){
		if(leftArm.getGripperOpen()){
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

bool YuMiControlInterface::sendJointInputsCheck(){
	bool sendInput = false;
	double tol = 0.001f;
	for (int i = 0; i < robot->getJointCount(); i++) {
		HingeJoint* hj = dynamic_cast<HingeJoint*>(robot->getJoint(i));
		if (!hj) continue;

		double delta = fabs(hj->motor.currentMotorAngle - hj->motor.targetMotorAngle);
		if(delta > tol){
			sendInput = true;
			break;
		}
	}
	return sendInput;
}

bool YuMiControlInterface::sendSpeedInputCheck(){
	bool sendInput = false;
	double tol = 0.001f;
	HingeJoint* hj = dynamic_cast<HingeJoint*>(robot->getJoint(0));
	double delta = fabs(hj->motor.currentYuMiTCPSpeed - hj->motor.targetYuMiTCPSpeed);
	if(delta > tol){
		sendInput = true;
	}
	return sendInput;
}


