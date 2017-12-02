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

unsigned short getMaestroSignalFromAngle(double angle, double angleRange, double pwmWidthAtZero, double pwmRange) {
	boundToRange(angle, -angleRange, angleRange);
	//maestro units are measured in 0.25microseconds increments, hence the factor of 4
	return (pwmWidthAtZero + angle / angleRange * pwmRange) * 4;
}

//a value of 40 corresponds to a speed limit of pwm=1000microseconds/s which corresponds to 90deg/s (if pwmRange is 500 and angle range is 45).
unsigned short getMaestroSignalFromAngularSpeed(double speed, double angleRange, double pwmRange) {
	//maestro units are measured in 0.25microseconds/10ms increments, hence the factor of 4 / 100

	//TODO: units do change if the refreshRate us not 50Hz!

	return speed / angleRange * pwmRange * 4 / 100;
}

double getAngleFromMaestroSignal(unsigned short ms, double angleRange, double pwmWidthAtZero, double pwmRange) {
	//maestro signal units are measured in 0.25microseconds increments, hence the factor of 4
	double pwm = ms / 4.0;
	boundToRange(pwm, pwmWidthAtZero - pwmRange, pwmWidthAtZero + pwmRange);
	return (pwm - pwmWidthAtZero) / pwmRange * angleRange;
}

// Gets the position of a Maestro channel.
// See the "Serial Servo Commands" section of the user's guide.
int maestroGetPosition(int fd, unsigned char channel){
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
int maestroSetTargetPosition(int fd, unsigned char channel, unsigned short target) {
	unsigned char command[] = { 0x84, channel, target & 0x7F, target >> 7 & 0x7F };
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
int maestroSetTargetSpeed(int fd, unsigned char channel, unsigned short target) {
	unsigned char command[] = { 0x87, channel, target & 0x7F, target >> 7 & 0x7F };
	if (write(fd, command, sizeof(command)) == -1) {
		Logger::consolePrint("error writing target velocity\n");
		return -1;
	}
	return 0;
}

double PololuServoControlInterface::getServomotorAngle(int motorID) {
	return getAngleFromMaestroSignal(maestroGetPosition(fd, (unsigned char)motorID), angleRange, pwmWidthAtZero, pwmRange);
}

void PololuServoControlInterface::setServomotorAngle(int motorID, double val) {
	maestroSetTargetPosition(fd, (unsigned char)motorID, getMaestroSignalFromAngle(val, angleRange, pwmWidthAtZero, pwmRange));
}

//val is specified in rad/s
void PololuServoControlInterface::setServomotorMaxSpeed(int motorID, double val) {
	maestroSetTargetPosition(fd, (unsigned char)motorID, getMaestroSignalFromAngularSpeed(val, angleRange, pwmRange));
}

//set motor goals from target values
void PololuServoControlInterface::sendControlInputsToPhysicalRobot() {
	
}

//read motor positions
void PololuServoControlInterface::readPhysicalRobotMotorPositions() {

}

//read motor positions
void PololuServoControlInterface::readPhysicalRobotMotorVelocities() {

}

void PololuServoControlInterface::openCommunicationPort() {
	// Open the Maestro's virtual COM port.
	std::string portName = "\\\\.\\COM6" + comNumber;
	//std::string portName = "/dev/ttyACM0";  // Linux
	//std::string portName = "/dev/cu.usbmodem00034567";  // Mac OS X

	fd = open(portName.c_str(), O_RDWR | O_NOCTTY);
	if (fd == -1){
		Logger::consolePrint("Could not open port COM%d\n", comNumber);
		return;
	}
	else {
		Logger::consolePrint("Port COM%d opened successfuly\n", comNumber);
	}

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
}

void PololuServoControlInterface::closeCommunicationPort() {
	close(fd);
}

void PololuServoControlInterface::driveMotorPositionsToZero() {

}

void PololuServoControlInterface::deactivateMotors() {

}

