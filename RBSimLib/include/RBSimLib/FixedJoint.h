#pragma once

#include <RBSimLib/Joint.h>

/*======================================================================================================================================*
 * This class implements a fixed joint type.                                                                                            *
 *======================================================================================================================================*/
class FixedJoint : public Joint{
public:
	FixedJoint(){}
	~FixedJoint(void){}

	/**
		Projects the orientation of the child onto the constraint manifold that satisfies the type of joint this is
	*/
	virtual void fixOrientationConstraint() {
		child->state.orientation = parent->state.orientation;
	}

	/**
		Projects the angular velocity of the child onto the constraint manifold that satisfies the type of joint this is
	*/
	virtual void fixAngularVelocityConstraint() {
		child->state.angularVelocity = parent->state.angularVelocity;
	}

	/**
		Processes a line of input, if it is specific to this type of joint. Returns true if processed, false otherwise.
	*/
	virtual bool processInputLine(char* line) {
		//there are no specific lines needed for this type of joint
		return false;
	}

	/**
		draws the axes of rotation
	*/
	virtual void drawAxes() {
		//this type of joint doesn't have any relative motion, so no axes to draw...
	}

	/**
		writes the joint info to file...
	*/
	virtual void writeToFile(FILE* fp) {
		fprintf(fp, "\t%s\n", getRBString(RB_WELDED_JOINT));
		writeCommonAttributesToFile(fp);
	}

	/**
		return true if joint limist should be used, false otherwise
	*/
	virtual bool shouldUseJointLimits() {
		return false;
	}

};
