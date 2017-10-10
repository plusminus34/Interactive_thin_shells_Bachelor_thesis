#pragma once

#include <MathLib/Quaternion.h>
#include <MathLib/Trajectory.h>
#include <RBSimLib/RigidBody.h>
#include <RBSimLib/Joint.h>
#include <ControlLib/BodyFrame.h>

/**
	This class represents a generic limb (i.e. leg or arm). Each limb has an origin RB, a bunch of segments connected by joints,
	and optionally a foot/hand. The end effector of the limb is refered to as the end point of the last segment of the limb before the foot/hand
*/
class GenericLimb{
public:
	//this is the name of the limb
	std::string name;
	//this is the body frame the limb is attached to
	BodyFrame* parentBodyFrame;
	//all limbs have an origin. This is it.
	RigidBody *origin;
	//and this is a list of all the limb's joints - for easy access...
	DynamicArray<Joint*> jointList;

	//a bit of a hack adding these here... but useful for control purposes...
	double swingPhase = -1;
	bool inContact = false;

public:
	/**
		constructor
	*/
	GenericLimb(void);

	/**
		destructor
	*/
	virtual ~GenericLimb(void);

	/**
		returns the origin (i.e. the parent of the leg or the arm).
	*/
	RigidBody* getOrigin(){
		return origin;
	}

	/**
		this method is used to return the list of joints for the current limb...
	*/
	DynamicArray<Joint*>* getJointList();

	/**
		This method returns true if the rb passed in is a part of the limb.
	*/
	bool isPartOfLimb(RigidBody* rb);

	/**
		returns the first joint of this limb - the hip
	*/
	Joint* getParentJoint(){
		if (getFirstLimbSegment()->pJoints.size() > 0)
			return getFirstLimbSegment()->pJoints[0];
		else
			return NULL;
	}

	/**
		returns the world coordinates of the base (i.e. hip or shoulder)
	*/
	P3D getParentJointPosition_world(){
		return getParentJoint()->child->getWorldCoordinates(getParentJoint()->cJPos);
	}

	/**
		returns the local coordinates of the first joint of the limb in the coordinate frame of the base/origin
	*/
	P3D getParentJointPosition_local(){
		return getParentJoint()->pJPos;
	}

/*************************************** Methods that have to be implemented by each kind of limb *********************************/

	/**
		returns the length of the limb
	*/
	virtual double getLength() = 0;

	/**
		returns the first segment of the limb
	*/
	virtual RigidBody* getFirstLimbSegment() = 0;
	
	/**
		returns the last segment of the limb (before the foot/hand, if there is one)
	*/
	virtual RigidBody* getLastLimbSegment() = 0;

	/**
		this method is used to collect all the joint links of the leg to the list...
	*/
	virtual void initializeJointList() = 0;
	
};


