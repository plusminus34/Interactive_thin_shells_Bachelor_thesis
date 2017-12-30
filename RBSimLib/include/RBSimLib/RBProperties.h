#pragma once

#include <MathLib/Matrix.h>
#include <MathLib/P3D.h>
#include <MathLib/V3D.h>
#include <MathLib/MathLib.h>

class RBFeaturePoint {
public:
	P3D coords;
	double featureSize = 0.01;
	bool selected = false;
	RBFeaturePoint(const P3D& p, double fSize = 0.01) {
		coords = p;
		featureSize = fSize;
	}
	//in some cases we may want to know which mesh of the parent RB is associated with this feature (e.g. if they are wheels that must rotate about their axis)
	int meshIndex = -1;
};

enum END_EFFECTOR_TYPE {
	EE_POINT = 0,
	EE_ACTIVE_WHEEL,
	EE_PASSIVE_WHEEL,
	EE_WELDED_WHEEL
};

class RigidBody;

class RBEndEffector : public RBFeaturePoint {
public:
	RBEndEffector(const P3D& p, double fSize = 0.01) : RBFeaturePoint(p, fSize) {
	}

public:
	END_EFFECTOR_TYPE eeType = EE_POINT;

	bool isWheel() const { return eeType != EE_POINT; }
	bool isActiveWheel() const { return eeType == EE_ACTIVE_WHEEL; }
	bool isWeldedWheel() const { return eeType == EE_WELDED_WHEEL; }
	bool isFreeToMoveWheel() const { return eeType == EE_PASSIVE_WHEEL; }

	void setMode(END_EFFECTOR_TYPE eet) { this->eeType = eet; }

	Vector3d getWheelAxis() const;
	Vector3d getWheelYawAxis() const;
	Vector3d getWheelTiltAxis() const;
	Vector3d getWheelRho() const;


	//if this is a wheel, we keep track of its axis of rotation, expressed in local coordinates...
	V3D localCoordsWheelAxis;
	//the radius is stored in feature size...
	
	//this represents desired instantaneous speed for the wheel EEs
	double rotationSpeed = 0;
};

/*================================================================================================================*
 * This class represents a container for the various properties of a rigid body, such as mass inertia matrix.     *
 *================================================================================================================*/
class RBProperties{
public:
	//the mass
	double mass = 1;
	//we'll store the moment of inertia of the rigid body, in the local coordinate frame
	Matrix3x3 MOI_local = Matrix3x3::Identity();
	//we will also store the coefficient of restitution
	double restitutionCoeff = 0.2;
	//and the coefficient of friction
	double frictionCoeff = 0.8;
	//this variable indicates wether or not this rigid body is fixed. 
	bool isFrozen = false;

	double thickness = 0.03;

	// rigid bodies with the same group ID will not be collided with each other...
	int collisionGroupID = -1;

	//each rigid body can have an arbitrary number of body point features, which can be interpreted in specific ways by different applications
	DynamicArray<RBFeaturePoint> bodyPointFeatures;
	//and an arbitrary number of end effectors
	DynamicArray<RBEndEffector> endEffectorPoints;

public:
	/**
		default constructor.
	*/
	RBProperties();

	/**
		default destructor.
	*/
	~RBProperties();

	/**
		set the moment of inertia of the rigid body - symmetric 3x3 matrix, so we need the six values for it.
	*/
	void setMOI(double moi00, double moi11, double moi22, double moi01, double moi02, double moi12);

	void addEndEffectorPoint(const P3D& p, double eeFeatureSize) {
		endEffectorPoints.push_back(RBEndEffector(p, eeFeatureSize));
	}

	P3D getEndEffectorPoint(int i) {
		if (i < (int)endEffectorPoints.size())
			return endEffectorPoints[i].coords;
		return P3D();
	}

	P3D getEndEffectorPosition() {
		P3D res;
		for (uint i = 0;i < endEffectorPoints.size();i++)
			res += endEffectorPoints[i].coords / endEffectorPoints.size();
		return res;
	}

	int getEndEffectorPointCount() {
		return endEffectorPoints.size();
	}
};
