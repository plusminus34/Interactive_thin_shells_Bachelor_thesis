#include <ControlLib/PololuServoControlInterface.h>
#include <Utils/Timer.h>

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

/**
BK DS-3002HV: minPWM: 910, maxPWM: 2090
*/

//- how well is the speed limit working? Does the board go to the final target just in time? Or way early? Or doesn't get there? Can at least plot desired vs final angles...
//- do the DXL communication protocol as well...

unsigned short getMaestroSignalFromAngle(double angle, Motor& mp) {
	//maestro units are measured in 0.25microseconds increments, hence the factor of 4
	return (unsigned short)(mp.getPWMForAngle(angle) * 4);
}

unsigned short getMaestroSignalFromSpeed(double angle, Motor& mp) {
	//maestro units are measured in 0.25microseconds increments, hence the factor of 4
	return (unsigned short)(mp.getPWMForSpeed(angle) * 4);
}

//a value of 40 corresponds to a speed limit of pwm=1000microseconds/s which corresponds to 90deg/s (if pwmRange is 500 and angle range is 45).
unsigned short getMaestroSignalFromSpeedLimit(double speed, Motor& mp, int signalPeriod) {
	//this means no speed limit
	if (speed <= 0)
		return 0;

	//speed is measured in rad/s. The number we pass to the maestro board needs to be measured in 0.25microseconds/10ms

	//1 rad corresponds to 1 / anglePerPWMunit() PWM units
	//1s is 100 of the time units...

	//maestro units are measured in 0.25microseconds/10ms increments, hence the factor of 4 / 100
	double speedUnits = 4.0 * 10.0 / 1000.0;

	//however, this is at the default signal period... it changes as described here otherwise...
	if (signalPeriod < 20)
		speedUnits = 4.0 * signalPeriod / 1000.0;
	if (signalPeriod > 20)
		speedUnits = 4.0 * (signalPeriod/2) / 1000.0;

	return (unsigned short)(speed / mp.anglePerPWMunit() * speedUnits);
}

double getAngleFromMaestroSignal(unsigned short ms, Motor& mp) {
	//maestro signal units are measured in 0.25microseconds increments, hence the factor of 4
	return mp.getAngleForPWM(ms / 4.0);
}

double getSpeedFromMaestroSignal(unsigned short ms, Motor& mp) {
	//maestro signal units are measured in 0.25microseconds increments, hence the factor of 4
	return mp.getSpeedForPWM(ms / 4.0);
}

// Gets the position of a Maestro channel.
// See the "Serial Servo Commands" section of the user's guide.
int PololuServoControlInterface::maestroGetPosition(Motor& mp){
	//NOTE: this position value represents the current pulse width that the Maestro is transmitting 
	//on the channel, reflecting the effects of any previous commands, speed and acceleration limits, 
	//or scripts running on the Maestro. It is not the actual position recorded by the servomotor's 
	//potentiometer

	if (!connected || mp.motorID < 0) return 0;

	if (!connected) return (int)mp.pwmFor0Deg * 4;
	unsigned char command[2] = { 0x90, 1};
	command[1] = (unsigned char)mp.motorID;
	if (write(fd, command, sizeof(command)) == -1){
		perror("error writing");
		return -1;
	}

	unsigned char response[2];
	if (read(fd, response, 2) != 2){
		perror("error reading");
		return -1;
	}

	return response[0] + 256 * response[1];
}

// Sets the target of a Maestro channel.
// See the "Serial Servo Commands" section of the user's guide.
// The units of 'target' are quarter-microseconds. If 0, the motor will no longer receive pulses
int PololuServoControlInterface::maestroSetTargetPosition(Motor& mp, unsigned short target) {
	if (!connected || mp.motorID < 0) return 0;

	unsigned char command[] = {0x84, 1, (unsigned short)(target & 0x7F), (unsigned short)(target >> 7 & 0x7F)};
	command[1] = (unsigned char)mp.motorID;
	if (write(fd, command, sizeof(command)) == -1)
	{
		Logger::consolePrint("error writing target position\n");
		return -1;
	}
	return 0;
}

// Sets the target of a Maestro channel.
// See the "Serial Servo Commands" section of the user's guide.
// The units of 'target' are quarter-microseconds. If 0, the motor will no longer receive pulses
int PololuServoControlInterface::maestroSetMultipleTargets(int startID, const DynamicArray<unsigned short>& targetValues) {
	if (!connected) return 0;

	DynamicArray<unsigned char> command;
	command.push_back(0x9F);
	command.push_back((unsigned char)targetValues.size());
	command.push_back((unsigned char)startID);

	for (uint i = 0; i < targetValues.size(); i++) {
		command.push_back((unsigned short)(targetValues[i] & 0x7F));
		command.push_back((unsigned short)(targetValues[i] >> 7 & 0x7F));
	}

	if (write(fd, &command[0], command.size()) == -1){
		Logger::consolePrint("error writing target position\n");
		return -1;
	}

	return 0;
}

// Sets the velocity limit of a Maestro channel.
// See the "Serial Servo Commands" section of the user's guide.
// The units of 'target' are (0.25 micros)/(10 ms). If 0, the motor will have no speed limit
int PololuServoControlInterface::maestroSetTargetSpeed(Motor& mp, unsigned short target) {
	if (!connected || mp.motorID < 0) return 0;
	if (controlPositionsOnly) target = 0;

	unsigned char command[] = { 0x87, 0, (unsigned short) (target & 0x7F), (unsigned short)(target >> 7 & 0x7F) };
	command[1] = (unsigned char)mp.motorID;
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
	return response[0] != 0x00;
}

double PololuServoControlInterface::getServomotorAngle(Motor& mp) {
	return getAngleFromMaestroSignal(maestroGetPosition(mp), mp);
}

double PololuServoControlInterface::getServomotorSpeed(Motor& mp) {
	return getSpeedFromMaestroSignal(maestroGetPosition(mp), mp);
}

void PololuServoControlInterface::setServomotorAngle(Motor& mp, double val) {
	maestroSetTargetPosition(mp, getMaestroSignalFromAngle(val, mp));
}

void PololuServoControlInterface::setServomotorSpeed(Motor& mp, double val) {
	maestroSetTargetPosition(mp, getMaestroSignalFromSpeed(val, mp));
}

//val is specified in rad/s
void PololuServoControlInterface::setServomotorMaxSpeed(Motor& mp, double val) {
	maestroSetTargetSpeed(mp, getMaestroSignalFromSpeedLimit(val, mp, signalPeriod));
}

//set motor goals from target values
void PololuServoControlInterface::sendControlInputsToPhysicalRobot() {
	if (motorsOn == false)
		return;

	for (auto hj : mJoints) {
		if (hj.j->motor.flipMotorAxis && hj.j->controlMode == POSITION_MODE)
			hj.j->motor.targetMotorAngle *= -1;

		if (hj.j->motor.flipMotorAxis && hj.j->controlMode == VELOCITY_MODE)
			hj.j->motor.targetMotorVelocity *= -1;

		if (hj.j->controlMode == POSITION_MODE)
			setServomotorMaxSpeed(hj.j->motor, hj.j->motor.targetMotorVelocity);
	}

	if (writeAllTargetCommandsAtOnce == false) {
		for (auto hj : mJoints) {
			if (hj.j->controlMode == POSITION_MODE)
				setServomotorAngle(hj.j->motor, hj.j->motor.targetMotorAngle);
			else if (hj.j->controlMode == VELOCITY_MODE)
				setServomotorSpeed(hj.j->motor, hj.j->motor.targetMotorVelocity);
		}
	} else {
		//this mode needs a contiguous block of motor ids that are stored in ascending order... 
		//if we're not to make any assumption about the order in which the motorID's are assigned, then we have to search for each contiguous block, send those commands and then start over...

		for (uint i = 0; i < multiTargetCommands.size(); i++) {
			for (uint j = 0; j < multiTargetCommands[i].robotJoints.size(); j++) {
				HingeJoint* hj = multiTargetCommands[i].robotJoints[j];
				if (hj->controlMode == POSITION_MODE)
					multiTargetCommands[i].targetVals[j] = getMaestroSignalFromAngle(hj->motor.targetMotorAngle, hj->motor);
				else if (hj->controlMode == VELOCITY_MODE)
					multiTargetCommands[i].targetVals[j] = getMaestroSignalFromSpeed(hj->motor.targetMotorVelocity, hj->motor);
			}
			maestroSetMultipleTargets(multiTargetCommands[i].motorStartID, multiTargetCommands[i].targetVals);
		}
	}
}

//read motor positions
void PololuServoControlInterface::readPhysicalRobotMotorPositions() {
	for (auto hj : mJoints) {
		if (hj.j->controlMode == POSITION_MODE){
			hj.j->motor.currentMotorAngle = getServomotorAngle(hj.j->motor);
			if (hj.j->motor.flipMotorAxis)
				hj.j->motor.currentMotorAngle *= -1;
		}
		else if (hj.j->controlMode == VELOCITY_MODE) {
			hj.j->motor.currentMotorVelocity = getServomotorSpeed(hj.j->motor);
			if (hj.j->motor.flipMotorAxis)
				hj.j->motor.currentMotorVelocity *= -1;
		}
	}
}

//read motor positions
void PololuServoControlInterface::readPhysicalRobotMotorVelocities() {
	//we cannot read motor velocities from the servomotors...
}

void PololuServoControlInterface::openCommunicationPort() {
	// Open the Maestro's virtual COM port.
	std::string portName = "\\\\.\\COM" + std::to_string(comNumber);
	//std::string portName = "/dev/ttyACM0";  // Linux
	//std::string portName = "/dev/cu.usbmodem00034567";  // Mac OS X

	fd = open(portName.c_str(), O_RDWR | O_NOCTTY);
	if (fd == -1){
		Logger::consolePrint("Could not open port %s\n", portName.c_str());
		return;
	}

	Logger::consolePrint("Port %s opened successfuly\n", portName.c_str());
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

	driveMotorPositionsToZero();
}

void PololuServoControlInterface::closeCommunicationPort() {
	if (!connected) return;
	close(fd);
	connected = false;
}

void PololuServoControlInterface::driveMotorPositionsToZero() {
	motorsOn = true;

	for (auto hj : mJoints) {
		if (hj.j->controlMode == POSITION_MODE) {
			hj.j->motor.targetMotorAngle = 0;
			hj.j->motor.targetMotorVelocity = 0.5;//make sure the motors all go to zero slowly...
		}
		else if (hj.j->controlMode == VELOCITY_MODE)
			hj.j->motor.targetMotorVelocity = 0;
	}
	sendControlInputsToPhysicalRobot();

	Timer t;
//	while (t.timeEllapsed() < 1000);
	while (servosAreMoving());

	for (auto hj : mJoints)
		if (hj.j->controlMode == POSITION_MODE)
			hj.j->motor.targetMotorVelocity = -1.0;//-1 here we will take to mean no speed limit...

	sendControlInputsToPhysicalRobot();
}

void PololuServoControlInterface::toggleMotorPower() {
	motorsOn = !motorsOn;
}

void PololuServoControlInterface::setTargetMotorValuesFromSimRobotState(double dt) {
	readPhysicalRobotMotorPositions();

	for (auto hj : mJoints){
		if (hj.j->controlMode == POSITION_MODE){
			Quaternion q = robot->getRelativeOrientationForJoint(hj.j);

			hj.j->motor.targetMotorAngle = q.getRotationAngle(hj.j->rotationAxis);

			//set the speed limit for the motor based on the difference between the current motor value and the target value

			//we expect we have dt time to go from the current position to the target position... we ideally want to ensure that the motor gets there exactly dt time from now, so we must limit its velocity...
			double speedLimit = fabs(hj.j->motor.targetMotorAngle - hj.j->motor.currentMotorAngle) / dt;
			hj.j->motor.targetMotorVelocity = speedLimit;
		} else if (hj.j->controlMode == VELOCITY_MODE) {
			hj.j->motor.targetMotorVelocity = robot->getRelativeLocalCoordsAngularVelocityForJoint(hj.j).dot(hj.j->rotationAxis);
		}
	}
}

PololuServoControlInterface::PololuServoControlInterface(Robot* robot) : RobotControlInterface(robot) {
	createMultiWriteClusters();
}

void ::PololuServoControlInterface::createMultiWriteClusters() {
	multiTargetCommands.clear();

	int maxIDFound = -1;
	for (auto hj : mJoints)
		maxIDFound = max(maxIDFound, hj.j->motor.motorID);

	for (int j = 0; j < maxIDFound; j++) {
		ServoMotorCommandBlock smcb;
		smcb.motorStartID = j; //this is the index of the motor we're starting from. We'll be looking for this motor and a continuous block from thereon in the list of robot joints...

		for (int i = 0; i<(int)mJoints.size(); i++) {
			HJ& hj = mJoints[i];
			if (hj.j->motor.motorID == j) {
				smcb.targetVals.push_back(0);
				smcb.robotJoints.push_back(hj.j);
				j++;	//now look for the next motor to add to the list...
				i = -1; //this is so that we start looking for the next motor id starting from the very first motor in the robot joint list
			}
		}

		if (smcb.targetVals.size() > 0)
			multiTargetCommands.push_back(smcb);
		//j is now the first motor we could not find, so increasing it in the upper loop means we give up on it and try to find the one right after
	}

	//now test it out...

	for (uint i = 0; i < multiTargetCommands.size(); i++) {
		string cmd;
		cmd = "Start: " + to_string(multiTargetCommands[i].motorStartID) + ". Motor id list: ";
		for (uint j = 0; j < multiTargetCommands[i].robotJoints.size(); j++) {
			HingeJoint* hj = multiTargetCommands[i].robotJoints[j];
			cmd += to_string(hj->motor.motorID) + " ";
		}
		Logger::consolePrint(cmd.c_str());
	}

}
