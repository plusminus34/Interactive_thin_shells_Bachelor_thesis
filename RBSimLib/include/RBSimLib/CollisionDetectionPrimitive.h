#pragma once

#include <Utils/Utils.h>
#include <MathLib/P3D.h>

#include <MathLib/Plane.h>


/*=========================================================================================================*
 * This class implements an interface for collision detection primitives such as spheres, capsules, etc.   *
 *=========================================================================================================*/
class CollisionDetectionPrimitive{
public:
	CollisionDetectionPrimitive();
	virtual ~CollisionDetectionPrimitive(void);

	// draw an outline of the primitive...
	virtual void draw() = 0;

	virtual std::string getDefinitionString() = 0;
	virtual std::string getKeyword() = 0;

	static CollisionDetectionPrimitive* getCDPFromDefinition(const std::string& def);
};

class SphereCDP : public CollisionDetectionPrimitive {
public:
	static std::string keyword;
	
	//origin of the sphere, expressed in local coordinates.
	P3D p;
	//and radius
	double r;

	SphereCDP();
	SphereCDP(const SphereCDP& other);
	SphereCDP(const P3D& p, double r);
	SphereCDP(const std::string& def);

	virtual void draw();
	virtual std::string getDefinitionString();
	virtual std::string getKeyword() { return keyword; }

};

class BoxCDP : public CollisionDetectionPrimitive {
public:
	static std::string keyword;

	//these are the two corners of the box, expressed in local coordinates.
	P3D p1, p2;

	BoxCDP();
	BoxCDP(const BoxCDP& other);
	BoxCDP(const P3D& p1, const P3D& p2);
	BoxCDP(const std::string& def);

	virtual void draw();
	virtual std::string getDefinitionString();
	virtual std::string getKeyword() { return keyword; }

	/**
	return the center of the box, expressed in local coordinates
	*/
	inline P3D getCenter() {
		return P3D((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2, (p1[2] + p2[2]) / 2);
	}

	/**
	returns the length in the x-direction
	*/
	inline double getXLen() {
		return (fabs(p1[0] - p2[0]));
	}

	/**
	returns the length in the y-direction
	*/
	inline double getYLen() {
		return (fabs(p1[1] - p2[1]));
	}

	/**
	returns the length in the z-direction
	*/
	inline double getZLen() {
		return (fabs(p1[2] - p2[2]));
	}
};

class PlaneCDP : public CollisionDetectionPrimitive {
public:
	static std::string keyword;

	//the plane...
	Plane p;

	PlaneCDP();
	PlaneCDP(const PlaneCDP& other);
	PlaneCDP(const Plane& p);
	PlaneCDP(const std::string& def);

	virtual void draw();
	virtual std::string getDefinitionString();
	virtual std::string getKeyword() { return keyword; }
};

class CapsuleCDP : public CollisionDetectionPrimitive {
public:
	static std::string keyword;

	//end points of the cylinder, expressed in local coordinates.
	P3D p1, p2;
	double r;

	bool hasFlatCaps = false;

	CapsuleCDP();
	CapsuleCDP(const CapsuleCDP& other);
	CapsuleCDP(const P3D& p1, const P3D& p2, double r, bool hasFlatCaps = false);
	CapsuleCDP(const std::string& def);

	virtual void draw();
	virtual std::string getDefinitionString();
	virtual std::string getKeyword() { return keyword; }
};


