
#pragma once

#include <MathLib/MathLib.h>
#include <Utils/Utils.h>
#include <RBSimLib/RigidBody.h>
#include <RBSimLib/Joint.h>
#include <RBSimLib/ContactForce.h>
#include <Utils/Utils.h>

/*--------------------------------------------------------------------------------------------------------------------------------------------*
 * This class implements a container for rigid bodies (both stand alone and articulated). It reads a .rbs file and interprets it.             *
 *--------------------------------------------------------------------------------------------------------------------------------------------*/
class AbstractRBEngine{
public:
	bool autoGenerateCDPs = false;

	//this is a list of all the objects in the world
	DynamicArray<RigidBody*> rbs;
	//and we'll keep a list of all the joints in the world as well - they impose constraints on the relative movement between the rigid bodies they connet
	DynamicArray<Joint*> joints;
	//this is a list of all the contact points - gets written from scratch whenever a new simulation step is taken
	DynamicArray<ContactForce> contactForces;



public:
	//the constructor
	AbstractRBEngine(void);
	//the destructor
	virtual ~AbstractRBEngine(void);

	virtual void destroy();

	//draw all the rigid bodies in the world
	void drawRBs(int flags = SHOW_MESH);

	// draw all the contact forces
	void drawContactForces();

	//goes through all the contact forces and marks the rigid bodies that are affected as inContact
	void markRBContacts(double fMagTreshold = 0.0);


	/**
		This method is used to return a pointer to the list of contact forces
	*/
	inline DynamicArray<ContactForce>* getContactForces(){
		return &contactForces;
	}

	/**
		This method is used to integrate the forward simulation in time.
	*/
	virtual void step(double deltaT){
		throwError("Not implemented!");
	}
	
	/**
		This method is used to integrate the simulation forward in time.
	*/
	virtual void loadRBsFromFile(const char* fName);

	/**
		This method returns the reference to the rigid body with the given name, or NULL if it is not found
	*/
	RigidBody* getRBByName(char* name);

	/**
		This method returns the reference to the joint whose name matches, or NULL if it is not found
	*/
	Joint* getJointByName(char* name);

	/**
		This method is used to get the state of all the rigid bodies
	*/
	void getState(DynamicArray<double>* state);

	/**
		This method is used to set the state of all the rigid bodies
	*/
	void setState(DynamicArray<double>* state, int start = 0);

	/**
		this method applies a force to a rigid body, at the specified point. The point is specified in local coordinates,
		and the force is specified in world coordinates.
	*/
	//virtual void applyForceTo(RigidBody* b, const V3D& f, const P3D& p);
	virtual void applyForceTo(RigidBody* b, const V3D& f, const P3D& p) = 0;
	
	/**
		this method applies a torque to a rigid body. The torque is specified in world coordinates.
	*/
	virtual void applyTorqueTo(RigidBody* b, const V3D& t)=0;
	
	/**
	this method applies a torque to a rigid body. The torque is specified in world coordinates.
	*/
	virtual void createODERB(RigidBody* rb)=0;

	virtual DynamicArray<ContactForce> getContactForceOnRB(RigidBody* b)=0;
};

