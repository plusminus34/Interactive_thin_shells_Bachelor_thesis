#pragma once

#include <MathLib/Quaternion.h>
#include <MathLib/Trajectory.h>
#include <RBSimLib/RigidBody.h>
#include <RBSimLib/Joint.h>
#include <ControlLib/GenericLimb.h>

/**
	This class represents a generic limb (i.e. leg or arm). Each limb has an origin RB, a bunch of segments connected by joints,
	and optionally a foot/hand. The end effector of the limb is refered to as the end point of the last segment of the limb before the foot/hand
*/
class SimpleLimb : public GenericLimb{
public:
	double limbLength = 0;

public:
	/**
		constructor
	*/
	SimpleLimb(const char *limbName, RigidBody* eeRB, RigidBody* root);

	/**
		destructor
	*/
	virtual ~SimpleLimb(void);

	/**
		returns the length of the limb
	*/
	virtual double getLength();

	/**
		returns the first segment of the limb
	*/
	virtual RigidBody* getFirstLimbSegment();
	
	/**
		returns the last segment of the limb (before the foot/hand, if there is one)
	*/
	virtual RigidBody* getLastLimbSegment();

	/**
		this method is used to collect all the joint links of the leg to the list...
	*/
	virtual void initializeJointList();

};


