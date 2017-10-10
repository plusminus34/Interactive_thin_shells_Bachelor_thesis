
#pragma once

#include <RBSimLib/Joint.h>

/*=========================================================================================================================*
 * This class is used to implement two degree-of-freedom joints (univeral joints).                                         *
 * To get to the orientation of the child, you start from the parent's orientation, rotate about the parent rotation axis  *
 * and then 
 *=========================================================================================================================*/

class UniversalJoint : public Joint{
public:
	//This joint can only rotate about this vecotr, stored in parent coordinates
	V3D rotAxisParent = V3D(1,0,0);
	//or about this one, stored in child coordinates
	V3D rotAxisChild = V3D(0,1,0);
	//the min and max allowed angles about the parent rotation axis
	double minAnglePRA = 0, maxAnglePRA = 0;
	//the min and max allowed angles about the child rotation axis
	double minAngleCRA = 0, maxAngleCRA = 0;
public:
	UniversalJoint();
	~UniversalJoint(void);

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


