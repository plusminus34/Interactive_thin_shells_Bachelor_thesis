#include <ControlLib/YuMiControlInterface.h>

#include <iostream>


//Constructor
YuMiControlInterface::YuMiControlInterface(Robot* robot) : RobotControlInterface(robot) {}

//Destructor
YuMiControlInterface::~YuMiControlInterface() {}

//set motor goals from target values
void YuMiControlInterface::sendControlInputsToPhysicalRobot() {

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

		speed = (unsigned int)hj->motor.yumiSpeed;
    }

    rightArm.gotoJointPose(rightTargetJoints);
    leftArm.gotoJointPose(leftTargetJoints);

	//Send also velocity command (for now: for end-effector)
	rightArm.setSpeed(speed);
	leftArm.setSpeed(speed);
}

//read motor positions
void YuMiControlInterface::readPhysicalRobotMotorPositions() {
    //std::cout << "readPhysicalRobotMotorPositions" << std::endl;

    std::vector<float> rightJoints = rightArm.getJoints();
    std::vector<float> leftJoints = leftArm.getJoints();

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

void YuMiControlInterface::driveMotorPositionsToZero(){
	std::cout << "driveMotorPositionsToZero" << std::endl;

	std::vector<float> leftTargetJoints = YuMiConstants::HOME_STATE_LEFT;
	std::vector<float> rightTargetJoints = YuMiConstants::HOME_STATE_RIGHT;

	driveMotorPositionsToInputPos(leftTargetJoints, rightTargetJoints);
}


void YuMiControlInterface::driveMotorPositionsToTestPos(){
	std::cout << "driveMotorPositionsToTestPos" << std::endl;

	std::vector<float> leftTargetJoints = YuMiConstants::INIT_STATE_LEFT;
	//std::vector<float> rightTargetJoints = YuMiConstants::INIT_STATE_RIGHT;
	std::vector<float> rightTargetJoints = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

	driveMotorPositionsToInputPos(leftTargetJoints, rightTargetJoints);

//    int jointIndex = 0;
//    for (int i = 0; i < robot->getJointCount(); i++) {
//        HingeJoint* hj = dynamic_cast<HingeJoint*>(robot->getJoint(i));
//        if (!hj) continue;

//        if(i % 2 == 0){
//            hj->motor.targetMotorAngle = rightTargetJoints.at(jointIndex);
//        } else {
//            hj->motor.targetMotorAngle = leftTargetJoints.at(jointIndex);
//            jointIndex++;
//        }

//        //hj->motor.targetMotorVelocity = 1.0;//make sure the motors all go to zero slowly...
//    }
//    sendControlInputsToPhysicalRobot();
}

void YuMiControlInterface::driveMotorPositionsToInputPos(std::vector<float> leftJoints, std::vector<float> rightJoints){
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

	robot->setState(&rs);
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




