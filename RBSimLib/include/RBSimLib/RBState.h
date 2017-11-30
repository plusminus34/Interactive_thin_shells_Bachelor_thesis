#pragma once

#include <MathLib/P3D.h>
#include <MathLib/V3D.h>
#include <MathLib/Quaternion.h>

/*======================================================================================================================================================================*
 * This class acts as a container for the state information (position, orientation, velocity and angular velocity - all of them stored in world coordinates, about the  *
 * center of mass) of a rigid body.                                                                                                                                     *
 *======================================================================================================================================================================*/

class RBState{
public:
	// the position of the center of mass of the rigid body, in world coords
	P3D position;
	// its orientation - rotates from local coordinate frame to world coordinate frame
	Quaternion orientation;
	// the velocity of the center of mass, in world coords
	V3D velocity;
	// and finally, the angular velocity about the center of mass, in world coords
	V3D angularVelocity;
	
public:
	/**
		Default constructor - populate the data members using safe values..
	*/
	RBState(void);

	/**
		A copy constructor.
	*/
	RBState(const RBState& other);

	/**
		and a copy operator	
	*/
	RBState& operator = (const RBState& other);
	/**
		Default destructor.
	*/
	~RBState(void);
	
	/**
		This method returns the coordinates of the point that is passed in as a parameter(expressed in local coordinates), in world coordinates.
	*/
	inline P3D getWorldCoordinates(const P3D& localPoint) const {
		return position + getWorldCoordinates(V3D(localPoint));
	}

	/**
		This method returns the vector that is passed in as a parameter(expressed in local coordinates), in world coordinates.
	*/
	inline V3D getWorldCoordinates(const V3D& localVector) const {
		//the rigid body's orientation is a unit quaternion. Using this, we can obtain the global coordinates of a local vector
		return orientation.rotate(localVector);
	}

	/**
		This method is used to return the local coordinates of the point that is passed in as a parameter (expressed in global coordinates)
	*/
	inline P3D getLocalCoordinates(const P3D& globalPoint){
		V3D v = getLocalCoordinates(V3D(position, globalPoint));
		return P3D(0,0,0) + v;
	}

	/**
		This method is used to return the local coordinates of the vector that is passed in as a parameter (expressed in global coordinates)
	*/
	inline V3D getLocalCoordinates(const V3D& globalVector){
		//the rigid body's orientation is a unit quaternion. Using this, we can obtain the global coordinates of a local vector
		return orientation.getComplexConjugate().rotate(globalVector);
	}

	/**
		This method returns the absolute velocity of a point that is passed in as a parameter. The point is expressed in local coordinates, and the
		resulting velocity will be expressed in world coordinates.
	*/
	inline V3D getAbsoluteVelocityForLocalPoint(const P3D& localPoint){
		//we need to compute the vector r, from the origin of the body to the point of interest
		V3D r(P3D(), localPoint);
		//the velocity is given by omega x r + v. omega and v are already expressed in world coordinates, so we need to express r in world coordinates first.
		return angularVelocity.cross(getWorldCoordinates(r)) + velocity;
	}

	/**
		This method returns the absolute velocity of a point that is passed in as a parameter. The point is expressed in global coordinates, and the
		resulting velocity will be expressed in world coordinates.
	*/
	inline V3D getAbsoluteVelocityForGlobalPoint(const P3D& globalPoint){
		//we need to compute the vector r, from the origin of the body to the point of interest
		V3D r(position, globalPoint);
		//the velocity is given by omega x r + v. omega and v are already expressed in world coordinates, so we need to express r in world coordinates first.
		return angularVelocity.cross(r) + velocity;
	}


};
