#pragma once

#include <MathLib/V3D.h>
#include <RBSimLib/RigidBody.h>
#include <RBSimLib/RBUtils.h>
#include <string>

enum JOINT_MODE {
	PASSIVE = 0,
	TORQUE_MODE,
	POSITION_MODE,
	VELOCITY_MODE,
};

/*==================================================================================================================================================================*
 * Joints impose constraints on the relative motion between the two reigid bodies they connect. Different types of joints implement different types of constraints  *
 *=======================================================================================================================================================================*/
class AbstractRBEngine;
class Joint{
public:
	//parent rigid body
	RigidBody* parent = NULL;
	//this is the location of the joint on the parent - expressed in the parent's local coordinates
	P3D pJPos;
	//this is the child link
	RigidBody* child = NULL;
	//this is the location of the joint on the child - expressed in the child's local coordinates 
	P3D cJPos;

	//maximum torque (magnitude) that can be applied at this joint
	double maxTorque = 1e5;
	//maximum speed (magnitude) for this joint, in rad/s
	double maxSpeed = 100.0;
	//a force regularizer for the motors
	double motorConstraintForceRegularizer = 0.001;

	//torque applied to this joint, set by a controller acting on this joint.
	V3D desiredJointTorque;
	//target relative orientation
	Quaternion desiredRelativeOrientation;
	//target relative angular velocity
	V3D desiredRelativeAngVelocity;

	//the name of the joint
	std::string name;
	//unique index of the joint
	int jIndex = -1;
	//depending on the joint mode, control inputs will be computed and applied differently...
	int controlMode = PASSIVE;

	MappingInfo mappingInfo;

	/**
		computes the relative orientation between the parent and the child rigid bodies
	*/
	Quaternion computeRelativeOrientation();

	/**
		Projects the orientation of the child onto the constraint manifold that satisfies the type of joint this is
	*/
	virtual void fixOrientationConstraint() = 0;

	/**
		Projects the angular velocity of the child onto the constraint manifold that satisfies the type of joint this is
	*/
	virtual void fixAngularVelocityConstraint() = 0;

	/**
		This method is used to fix the errors in the joints (i.e. project state of child such that joint configuration is consistent). The state
		of the parent does not change.
	*/
	void fixJointConstraints(bool fixPositions, bool fixOrientations, bool fixLinVelocities, bool fixAngularVelocities);

	/**
		writes the joint info to file...
	*/
	virtual void writeToFile(FILE* fp) = 0;

	/**
		draws the axes of rotation
	*/
	virtual void drawAxes() = 0;

	/**
		This method is used to load the details of a joint from file.
	*/
	virtual void loadFromFile(FILE* fp, AbstractRBEngine* world);

	/**
		Processes a line of input, if it is specific to this type of joint. Returns true if processed, false otherwise.
	*/
	virtual bool processInputLine(char* line) = 0;

	/**
		writes the joint info to file...
	*/
	void writeCommonAttributesToFile(FILE* fp);

	/**
		return true if joint limist should be used, false otherwise
	*/
	virtual bool shouldUseJointLimits() = 0;

public:
	/**
		Default constructor
	*/
	Joint(void);

	/**
		Default destructor
	*/
	virtual ~Joint(void);

	/**
		Returns the world position of the joint
	*/
	P3D getWorldPosition();

};


