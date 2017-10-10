
#pragma once

#include <RBSimLib/Joint.h>
#include <RBSimLib/RBUtils.h>

/*=======================================================================================================================================================*
 * This class implements a ball and socket joint type. No joint limits are implemented, so these types of joints allow unrestricted relative rotations                                                                                                         *
 *=======================================================================================================================================================*/
class BallAndSocketJoint : public Joint{
private:

public:
	BallAndSocketJoint();
	virtual ~BallAndSocketJoint(void);

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
	virtual bool shouldUseJointLimits() {
		return false;
	}

};
