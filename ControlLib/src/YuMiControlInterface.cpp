#include <ControlLib/YuMiControlInterface.h>

#include <iostream>


//Constructor
YuMiControlInterface::YuMiControlInterface(Robot* robot) : RobotControlInterface(robot) {
    leftArm.init("left");
    rightArm.init("right");
}

//Destructor
YuMiControlInterface::~YuMiControlInterface() {}

//set motor goals from target values
void YuMiControlInterface::sendControlInputsToPhysicalRobot() {
    std::cout << "sendControlInputsToPhysicalRobot" << std::endl;

    std::vector<float> rightTargetJoints(robot->getJointCount()/2, 0.0);
    std::vector<float> leftTargetJoints(robot->getJointCount()/2, 0.0);

    int jointIndex = 0;
    for (int i = 0; i < robot->getJointCount(); i++) {
        HingeJoint* hj = dynamic_cast<HingeJoint*>(robot->getJoint(i));
        if (!hj) continue;

        if(i % 2 == 0){
            rightTargetJoints.at(jointIndex) = hj->motor.targetMotorAngle;
        } else {
            leftTargetJoints.at(jointIndex) = hj->motor.targetMotorAngle;
            jointIndex++;
        }

        //hj->motor.targetMotorVelocity = 1.0;//make sure the motors all go to zero slowly...
    }

    rightArm.gotoJointPose(rightTargetJoints);
    leftArm.gotoJointPose(leftTargetJoints);
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
    std::cout << "setTargetMotorValuesFromSimRobotState" << std::endl;
    readPhysicalRobotMotorPositions();

    //given the values stored in the joint's dxl properties structure (which are updated either from the menu or by sync'ing with the dynamixels), update the state of the robot...
    ReducedRobotState rs(robot);

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
    std::cout << "openCommunicationPort" << std::endl;
    if(!leftArm.getConnected()){
        leftArm.init("left");
    }
    if(!rightArm.getConnected()){
        rightArm.init("right");
    }
}

void YuMiControlInterface::closeCommunicationPort() {
    std::cout << "closeCommunicationPort" << std::endl;
    leftArm.closeConnection();
    rightArm.closeConnection();
}

void YuMiControlInterface::driveMotorPositionsToZero(){
    std::vector<float> leftTargetJoints = {0.0, -2.26893, 0.0, 0.69813, 0.0, -2.35619, 0.52360};
    std::vector<float> rightTargetJoints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    int jointIndex = 0;
    for (int i = 0; i < robot->getJointCount(); i++) {
        HingeJoint* hj = dynamic_cast<HingeJoint*>(robot->getJoint(i));
        if (!hj) continue;

        if(i % 2 == 0){
            hj->motor.targetMotorAngle = rightTargetJoints.at(jointIndex);
        } else {
            hj->motor.targetMotorAngle = leftTargetJoints.at(jointIndex);
            jointIndex++;
        }

        //hj->motor.targetMotorVelocity = 1.0;//make sure the motors all go to zero slowly...
    }
    sendControlInputsToPhysicalRobot();
}


void YuMiControlInterface::driveMotorPositionsToTestPos(){

    std::vector<float> leftTargetJoints = {0.0, -2.26893, 0.0, 0.69813, 0.0, -2.35619, 0.52360};
    std::vector<float> rightTargetJoints = {0.0, 0.0, 0.0, 0.349066, 0.0, 0.0, 0.0};

    int jointIndex = 0;
    for (int i = 0; i < robot->getJointCount(); i++) {
        HingeJoint* hj = dynamic_cast<HingeJoint*>(robot->getJoint(i));
        if (!hj) continue;

        if(i % 2 == 0){
            hj->motor.targetMotorAngle = rightTargetJoints.at(jointIndex);
        } else {
            hj->motor.targetMotorAngle = leftTargetJoints.at(jointIndex);
            jointIndex++;
        }

        //hj->motor.targetMotorVelocity = 1.0;//make sure the motors all go to zero slowly...
    }
    sendControlInputsToPhysicalRobot();
}


