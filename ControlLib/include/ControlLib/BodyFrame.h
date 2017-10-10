#pragma once
#include <RBSimLib/RigidBody.h>

class RigidBody;
class Robot;
class GenericLimb;

/**
	This class acts as a container for all the limbs of a character. It is assumed that the body links constituting the body frame are all connected
	to each other, and that they are the parents/ancestors of all other links of a character.

	It is implicitly implied that the root of the figure belongs to the body frame. Also, all the rigid links that constitute the body frame must be
	connected through joints.
*/
class BodyFrame{
public:
	//this is the robot that owns the body frame...
	Robot* robot;
	//keep track of the rigid bodies that constitute the body frame
	DynamicArray<RigidBody*> bodyLinks;
	//and the limbs that are attached to it...
	DynamicArray<GenericLimb*> limbs;

	//maintain a list of joints that connect the body frame to other body parts such as limbs, tails, neck etc
	DynamicArray<Joint*> jointsToTheOutside;
	//and this is the list of joints that connects different rigid bodies of the bodyframe to each other
	DynamicArray<Joint*> internalJoints;

	//keep track of the estimated state of the body frame
	RBState bodyState;
	//and its heading, which transforms vectors from a coordinate frame aligned with the heading of the body to world
	Quaternion bodyHeading;

	//and the estimated moment of inertia (in world frame) for the body
	Matrix3x3 bodyMomentOfInertia, bodyMomentOfInertia_local;
	//total mass of the body frame
	double totalMass;

// these quantities get updated every time new contact information is available...
	V3D centerOfPressure;	
	V3D netContactForce;
public:

public:
	BodyFrame(Robot* robot);
	~BodyFrame(void);

	// adds a limb to the body frame
	void addLimb(GenericLimb* l);

	// adds a new rigid body to the bodyframe
	void addBodyLink(RigidBody* rb);

	// returns the closest rigid body on the body frame for rb
	RigidBody* getClosestBodyFrameLinkTo(RigidBody* rb);

	// returns the index of the rb in the list of links making up the bodyframe, or -1 if it doesn't belong to it
	int getRBIndex(RigidBody* rb);

	// returns true if the rb is a part of the body frame
	bool isPartOfBodyFrame(RigidBody* rb);

	// returns true if the joint belongs to the set of internal joints
	bool isInternalJoint(Joint* j);

	// updates the state and heading of the body frame
	void updateStateInformation();

	void updateLimbGroundContactInformation();

	//draws the body frame
	void drawBodyFrame();

	//There are three coordinate frames that we might care about:
	//- the worldFrame
	//- the bodyFrame
	//- the headingFrame, which only keeps the yaw/heading of the body's orientation; the heading frame is as close as possible to the body frame, but has the same up axis as the world frame
	//The methods below transform vectors/points between all these different coordinate frames

	P3D getBodyFrameCoordsForWorldFramePoint(const P3D& wfPoint);
	P3D getWorldFrameCoordsForBodyFramePoint(const P3D& bfPoint);
	P3D getHeadingFrameCoordsForBodyFramePoint(const P3D& bfPoint);
	P3D getHeadingFrameCoordsForWorldFramePoint(const P3D& wfPoint);
	P3D getWorldFrameCoordsForHeadingFramePoint(const P3D& hfPoint);

	V3D getBodyFrameCoordsForWorldFrameVector(const V3D& wfVector);
	V3D getWorldFrameCoordsForBodyFrameVector(const V3D& bfVector);
	V3D getHeadingFrameCoordsForBodyFrameVector(const V3D& bfVector);
	V3D getHeadingFrameCoordsForWorldFrameVector(const V3D& wfVector);
	V3D getWorldFrameCoordsForHeadingFrameVector(const V3D& hfVector);

};


