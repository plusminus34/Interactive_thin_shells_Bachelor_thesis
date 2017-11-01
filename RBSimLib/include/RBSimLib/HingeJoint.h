#pragma once

#include <RBSimLib/Joint.h>

class DXL_Properites {
public:
	//the id of the motor that a virtual joint corresponds to
	int dxl_id = -1;
	//angles are stored in radians, velocities in radians/sec
	double targetMotorAngle = 0;
	double targetMotorVelocity = 0;
	double targetMotorAcceleration = 0;
	//read-only variables that stores the state of physical motors
	double currentMotorAngle = 0;
	double currentMotorVelocity = 0;

	//depending on how the robot is assembled, the axis may point in the wrong direction, relative to the simulation mode. Make this easy to fix...
	bool flipMotorAxis = false;
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

	DXL_Properites dynamixelProperties;
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
