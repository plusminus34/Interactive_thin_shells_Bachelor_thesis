#include <ControlLib/YuMiControlInterface.h>

//#include <fcntl.h>
//#include <stdio.h>
//#ifdef _WIN32
//#include <io.h>
//#else
//#include <unistd.h>
//#endif

//#ifdef _WIN32
//#define O_NOCTTY 0
//#else
//#include <termios.h>
//#endif


//set motor goals from target values
void YuMiControlInterface::sendControlInputsToPhysicalRobot() {

}

//read motor positions
void YuMiControlInterface::readPhysicalRobotMotorPositions() {

}

//read motor positions
void YuMiControlInterface::readPhysicalRobotMotorVelocities() {

}

void YuMiControlInterface::setTargetMotorValuesFromSimRobotState(double dt) {
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
//	// Open the Maestro's virtual COM port.
//	std::string portName = "\\\\.\\COM" + std::to_string(comNumber);
//	//std::string portName = "/dev/ttyACM0";  // Linux
//	//std::string portName = "/dev/cu.usbmodem00034567";  // Mac OS X

//	fd = open(portName.c_str(), O_RDWR | O_NOCTTY);
//	if (fd == -1){
//		Logger::consolePrint("Could not open port %s\n", portName.c_str());
//		return;
//	}

//	Logger::consolePrint("Port %s opened successfuly\n", portName.c_str());
//	connected = true;

//#ifdef _WIN32
//	_setmode(fd, _O_BINARY);
//#else
//	struct termios options;
//	tcgetattr(fd, &options);
//	options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
//	options.c_oflag &= ~(ONLCR | OCRNL);
//	options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
//	tcsetattr(fd, TCSANOW, &options);
//#endif

//	driveMotorPositionsToZero();
}

void YuMiControlInterface::closeCommunicationPort() {
//	if (!connected) return;
//	close(fd);
//	connected = false;
}

void YuMiControlInterface::driveMotorPositionsToZero() {

}

//Constructor (destructor defined in .h file)
YuMiControlInterface::YuMiControlInterface(Robot* robot) : RobotControlInterface(robot) {

}

