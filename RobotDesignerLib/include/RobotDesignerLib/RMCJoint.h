#pragma once

#include <RBSimLib/Joint.h>
#include <MathLib/Transformation.h>
#include <RobotDesignerLib/RMCPin.h>

/*================================================================================================================================*
 * This class is used to implements joints that connects two RMCs                                                                  *
 *================================================================================================================================*/

using namespace std;

class RMC;
class RMCPin;

class RMCJoint : public Joint{
public:
	RMCPin* parentPin = NULL;
	RMCPin* childPin = NULL;

	// child's coordinate frame transformation related to parent's.
	vector<Transformation> transformations;
	// the current transformation index into transformation array.
	int curTransIndex;

public:
	RMCJoint();
	RMCJoint(RMCPin* parentPin, RMCPin* childPin, const vector<Transformation>& transformations, int transIndex = 0);
	RMCJoint(RMC* parent, RMC* child, const vector<Transformation>& transformations, int transIndex = 0);
	void switchToNextTransformation();

	virtual ~RMCJoint(void);

	// parent controls child
	void fixRMCConstraint();

	// child controls parent
	void fixRMCConstraintInverse();

	RMC* getChild() {
		return (RMC*)child;
	}

	RMC* getParent() {
		return (RMC*)parent;
	}

	/**
		Projects the orientation of the child onto the constraint manifold that satisfies the type of joint this is
	*/
	virtual void fixOrientationConstraint() {}

	/**
		Projects the angular velocity of the child onto the constraint manifold that satisfies the type of joint this is
	*/
	virtual void fixAngularVelocityConstraint() {}

	/**
		Processes a line of input, if it is specific to this type of joint. Returns true if processed, false otherwise.
	*/
	virtual bool processInputLine(char* line) { return false; }

	/**
		draws the axes of rotation
	*/
	virtual void drawAxes() {}

	/**
		writes the joint info to file...
	*/
	virtual void writeToFile(FILE* fp);

	/**
		return true if joint limist should be used, false otherwise
	*/
	virtual bool shouldUseJointLimits() { return false; }

	
};
