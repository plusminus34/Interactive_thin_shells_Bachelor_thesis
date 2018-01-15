#pragma once

#include <RBSimLib/Joint.h>

class Motor {
public:
	//the id of the motor that a virtual joint corresponds to
	int motorID = -1;
	//angles are stored in radians, velocities in radians/sec
	double targetMotorAngle = 0;
	double targetMotorVelocity = 0;
	double targetMotorAcceleration = 0;
	//read-only variables that stores the state of physical motors
	double currentMotorAngle = 0;
	double currentMotorVelocity = 0;

	//depending on how the robot is assembled, the axis may point in the wrong direction, relative to the simulation mode. Make this easy to fix...
	bool flipMotorAxis = false;

	//temporary end-effector speed variable for yumi
	unsigned int currentYuMiTCPSpeed = 100;
	unsigned int targetYuMiTCPSpeed = currentYuMiTCPSpeed;

/* everything below is specific to servomotors... */

	//for servomotors (as opposed to say dynamixels) there needs to be a bit of calibration to know how they need to be controlled properly
	//all this data depends on the type of servomotor used, as well as on the way in which the horn is assembled...
	//PWM signals are measured in microseconds... the min/max values can either be found in specs or experimentally
	double pwmMin = 1000;
	double pwmMax = 2000;

	//the numbers below depend on how the servomotor is assembled, as well as on the specs of the servomotors...
	double pwmFor0Deg = 1500;
	double pwmFor45Deg = 2000;

	//if the servomotor is in continuous rotation mode, then we need to map RPM to PWM.
	//these servomotors have a deadband (e.g. interval of PWM values that don't make the motor move)
	double pwmDeadBand = 30;
	double pwmFor50RPM = 1750;

	//the units of angle returned here are measured in radians
	double anglePerPWMunit() {
		return RAD(45) / (pwmFor45Deg - pwmFor0Deg);
	}

	//angle is measured in radians
	double getPWMForAngle(double angle) {
		double angleMin = getAngleForPWM(pwmMin);
		double angleMax = getAngleForPWM(pwmMax);

		boundToRange(angle, angleMin, angleMax);
//		Logger::consolePrint("angle min: %lf angle max: %lf, angle: %lf, pwm: %lf\n", angleMin, angleMax, angle, pwmFor0Deg + angle / anglePerPWMunit());

		return pwmFor0Deg + angle / anglePerPWMunit();
	}

	//angle will be measured in radians
	double getAngleForPWM(double pwm) {
		boundToRange(pwm, pwmMin, pwmMax);

		return (pwm - pwmFor0Deg) * anglePerPWMunit();
	}

	//the units of speed returned here are measured in radians per sec
	double speedPerPWMunit() {
		return (2 * PI / 60.0) * 50 / (pwmFor50RPM - (pwmFor0Deg + pwmDeadBand));
	}

	//speed is measured in RAD/S
	double getPWMForSpeed(double speed) {
		double speedMin = getSpeedForPWM(pwmMin);
		double speedMax = getSpeedForPWM(pwmMax);

		boundToRange(speed, speedMin, speedMax);
		if (speed > 0)
			return (pwmFor0Deg + pwmDeadBand) + speed / speedPerPWMunit();
		else
			return (pwmFor0Deg - pwmDeadBand) + speed / speedPerPWMunit();
	}

	//angle will be measured in radians
	double getSpeedForPWM(double pwm) {
		boundToRange(pwm, pwmMin, pwmMax);
		if (pwm > pwmFor0Deg - pwmDeadBand && pwm < pwmFor0Deg + pwmDeadBand)
			return 0;

		if (pwm > pwmFor0Deg)
			return (pwm - (pwmFor0Deg + pwmDeadBand)) * speedPerPWMunit();
		else
			return (pwm - (pwmFor0Deg - pwmDeadBand)) * speedPerPWMunit();
	}


};

/*================================================================================================================================*
 * This class is used to implements joints that allow relative rotation between the parent and the child only about a given axis  *
 *================================================================================================================================*/
class HingeJoint : public Joint{
public:
	//local coordinates of the rotation axis. Since the child and parent only rotate relative to each other about this joint, the local coordinates for the rotation axis are the same in parent and child frame 
	V3D rotationAxis = V3D(1,0,0);
	//joint limits 
	double minAngle = 0, maxAngle = 0;

	//keep a 'default' angle... useful for applications that create a robot and its state at the same time
	double defaultAngle = 0;

	Motor motor;
public:
	HingeJoint();
	virtual ~HingeJoint(void);

	/**
		Projects the orientation of the child onto the constraint manifold that satisfies the type of joint this is
	*/
	virtual void fixOrientationConstraint();

	/**
		Projects the angular velocity of the child onto the constraint manifold that satisfies the type of joint this is
	*/
	virtual void fixAngularVelocityConstraint();

	/**
		Processes a line of input, if it is specific to this type of joint. Returns true if processed, false otherwise.
	*/
	virtual bool processInputLine(char* line);

	/**
		draws the axes of rotation
	*/
	virtual void drawAxes();

	/**
		writes the joint info to file...
	*/
	virtual void writeToFile(FILE* fp);

	/**
		return true if joint limist should be used, false otherwise
	*/
	virtual bool shouldUseJointLimits();

};
