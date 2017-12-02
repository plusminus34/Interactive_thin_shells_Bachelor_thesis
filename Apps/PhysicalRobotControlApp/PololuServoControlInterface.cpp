#include "PololuServoControlInterface.h"

#include <fcntl.h>
#include <stdio.h>
#ifdef _WIN32
#include <io.h>
#else
#include <unistd.h>
#endif

#ifdef _WIN32
#define O_NOCTTY 0
#else
#include <termios.h>
#endif

//the assumption here is that the max range of pwm signals gets mapped to the max range of angles... should be easy-ish to check if i have a servomotor with 180deg range and another with 90...
//should also implement functionality that sends out all the signals at once...
//should also implement nicely the method that computes speed limits based on current angle and target angle and dt...

unsigned short getMaestroSignalFromAngle(double angle, double angleRange, double pwmWidthAtZero, double pwmRange) {
	boundToRange(angle, -angleRange, angleRange);
	//maestro units are measured in 0.25microseconds increments, hence the factor of 4
	return (unsigned short)((pwmWidthAtZero + angle / angleRange * pwmRange) * 4);
}

//a value of 40 corresponds to a speed limit of pwm=1000microseconds/s which corresponds to 90deg/s (if pwmRange is 500 and angle range is 45).
unsigned short getMaestroSignalFromAngularSpeed(double speed, double angleRange, double pwmRange) {
	//maestro units are measured in 0.25microseconds/10ms increments, hence the factor of 4 / 100

	//NOTE: units do change if the refreshRate us not 50Hz! (https://www.pololu.com/docs/0J40/4.b)

	return (unsigned short)(speed / angleRange * pwmRange * 4 / 100);
}

double getAngleFromMaestroSignal(unsigned short ms, double angleRange, double pwmWidthAtZero, double pwmRange) {
	//maestro signal units are measured in 0.25microseconds increments, hence the factor of 4
	double pwm = ms / 4.0;
	boundToRange(pwm, pwmWidthAtZero - pwmRange, pwmWidthAtZero + pwmRange);
	return (pwm - pwmWidthAtZero) / pwmRange * angleRange;
}


// Gets the position of a Maestro channel.
// See the "Serial Servo Commands" section of the user's guide.
int PololuServoControlInterface::maestroGetPosition(unsigned char channel){
	if (!connected) return 0;
	unsigned char command[] = { 0x90, channel };
	if (write(fd, command, sizeof(command)) == -1)
	{
		perror("error writing");
		return -1;
	}

	unsigned char response[2];
	if (read(fd, response, 2) != 2)
	{
		perror("error reading");
		return -1;
	}

	return response[0] + 256 * response[1];
}


// Sets the target of a Maestro channel.
// See the "Serial Servo Commands" section of the user's guide.
// The units of 'target' are quarter-microseconds. If 0, the motor will no longer receive pulses
int PololuServoControlInterface::maestroSetTargetPosition(unsigned char channel, unsigned short target) {
	if (!connected) return 0;
	if (motorsOn == false)
		target = 0;
	unsigned char command[] = {0x84, channel, (unsigned short)(target & 0x7F), (unsigned short)(target >> 7 & 0x7F)};
	if (write(fd, command, sizeof(command)) == -1)
	{
		Logger::consolePrint("error writing target position\n");
		return -1;
	}
	return 0;
}

// Sets the velocity limit of a Maestro channel.
// See the "Serial Servo Commands" section of the user's guide.
// The units of 'target' are (0.25 micros)/(10 ms). If 0, the motor will have no speed limit
int PololuServoControlInterface::maestroSetTargetSpeed(unsigned char channel, unsigned short target) {
	if (!connected) return 0;
	if (motorsOn == false)
		target = 0;
	unsigned char command[] = { 0x87, channel, (unsigned short) (target & 0x7F), (unsigned short)(target >> 7 & 0x7F) };
	if (write(fd, command, sizeof(command)) == -1) {
		Logger::consolePrint("error writing target velocity\n");
		return -1;
	}
	return 0;
}

bool PololuServoControlInterface::servosAreMoving() {
	if (!connected) return 0;
	unsigned char command[] = { 0x93 };
	if (write(fd, command, sizeof(command)) == -1)
	{
		perror("error writing");
		return false;
	}

	unsigned char response[1];
	if (read(fd, response, 1) != 1)
	{
		perror("error reading");
		return false;
	}
	return response != 0x00;
}

double PololuServoControlInterface::getServomotorAngle(int motorID) {
	return getAngleFromMaestroSignal(maestroGetPosition((unsigned char)motorID), angleRange, pwmWidthAtZero, pwmRange);
}

void PololuServoControlInterface::setServomotorAngle(int motorID, double val) {
	maestroSetTargetPosition((unsigned char)motorID, getMaestroSignalFromAngle(val, angleRange, pwmWidthAtZero, pwmRange));
}

//val is specified in rad/s
void PololuServoControlInterface::setServomotorMaxSpeed(int motorID, double val) {
	maestroSetTargetPosition((unsigned char)motorID, getMaestroSignalFromAngularSpeed(val, angleRange, pwmRange));
}

//set motor goals from target values
void PololuServoControlInterface::sendControlInputsToPhysicalRobot() {
	for (int i = 0; i < robot->getJointCount(); i++) {
		HingeJoint* hj = dynamic_cast<HingeJoint*>(robot->getJoint(i));
		if (!hj) continue;
		if (hj->motorProperties.flipMotorAxis) {
			hj->motorProperties.targetMotorAngle *= -1;
		}
		setServomotorAngle(hj->motorProperties.motorID, hj->motorProperties.targetMotorAngle);
	}

	//TODO: should implement here the option of sending all position commands at once...
	//TODO: somewhere we should be computing speed limits...
}

//read motor positions
void PololuServoControlInterface::readPhysicalRobotMotorPositions() {
	for (int i = 0; i < robot->getJointCount(); i++) {
		HingeJoint* hj = dynamic_cast<HingeJoint*>(robot->getJoint(i));
		if (!hj) continue;

		hj->motorProperties.currentMotorAngle = getServomotorAngle(hj->motorProperties.motorID);

		if (hj->motorProperties.flipMotorAxis)
			hj->motorProperties.currentMotorAngle *= -1;
	}
}

//read motor positions
void PololuServoControlInterface::readPhysicalRobotMotorVelocities() {
	//we cannot read motor velocities from the servomotors...
}

void PololuServoControlInterface::openCommunicationPort() {
	// Open the Maestro's virtual COM port.
	std::string portName = "\\\\.\\COM" + comNumber;
	//std::string portName = "/dev/ttyACM0";  // Linux
	//std::string portName = "/dev/cu.usbmodem00034567";  // Mac OS X

	fd = open(portName.c_str(), O_RDWR | O_NOCTTY);
	if (fd == -1){
		Logger::consolePrint("Could not open port COM%d\n", comNumber);
		return;
	}

	Logger::consolePrint("Port COM%d opened successfuly\n", comNumber);
	connected = true;

#ifdef _WIN32
	_setmode(fd, _O_BINARY);
#else
	struct termios options;
	tcgetattr(fd, &options);
	options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
	options.c_oflag &= ~(ONLCR | OCRNL);
	options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tcsetattr(fd, TCSANOW, &options);
#endif
	motorsOn = true;
	toggleMotorPower();
}

void PololuServoControlInterface::closeCommunicationPort() {
	if (!connected) return;
	close(fd);
	connected = false;
}

void PololuServoControlInterface::driveMotorPositionsToZero() {
	motorsOn = true;
	toggleMotorPower();

	for (int i = 0; i < robot->getJointCount(); i++) {
		HingeJoint* hj = dynamic_cast<HingeJoint*>(robot->getJoint(i));
		if (!hj) continue;
		hj->motorProperties.targetMotorAngle = 0;
		maestroSetTargetSpeed(hj->motorProperties.motorID, 4);//make sure the motors all go to zero slowly...
	}
	sendControlInputsToPhysicalRobot();

	while (servosAreMoving());

	for (int i = 0; i < robot->getJointCount(); i++) {
		HingeJoint* hj = dynamic_cast<HingeJoint*>(robot->getJoint(i));
		if (!hj) continue;
		maestroSetTargetSpeed(hj->motorProperties.motorID, 0);//remove the speed limits...
	}
	toggleMotorPower();

}

void PololuServoControlInterface::toggleMotorPower() {
	motorsOn = !motorsOn;
	if (motorsOn == false) {
		for (int i = 0; i < robot->getJointCount(); i++) {
			HingeJoint* hj = dynamic_cast<HingeJoint*>(robot->getJoint(i));
			if (!hj) continue;
			maestroSetTargetPosition(hj->motorProperties.motorID, 0);
		}
	}
	else {
		readPhysicalRobotMotorPositions();
		for (int i = 0; i < robot->getJointCount(); i++) {
			HingeJoint* hj = dynamic_cast<HingeJoint*>(robot->getJoint(i));
			if (!hj) continue;
			hj->motorProperties.targetMotorAngle = hj->motorProperties.currentMotorAngle;
		}
		sendControlInputsToPhysicalRobot();
	}
}

