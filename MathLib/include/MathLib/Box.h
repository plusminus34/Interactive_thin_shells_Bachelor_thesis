#pragma once

#include "P3D.h"
#include "Quaternion.h"
#include "Ray.h"
#include "Segment.h"

class Box{
public:
	P3D position;
	//in the coordinate frame of the box, we need to know two opposite corners, expressed relative to the '0' point which is the position P3D
	P3D c1;
	P3D c2;

	Quaternion orientation;

	Box();
	Box(double heading, const P3D &pos, const P3D &c1, const P3D &c2);
	Box(const Quaternion& q, const P3D &pos, const P3D &c1, const P3D &c2);
	virtual ~Box() {}
	
	/**
		Returns the distance between a point and the box.
		If the point lies inside the box, zero is returned.
	*/
	double getDistanceToPoint(const P3D &pt, P3D *closestPointOnBox = NULL) const;
	
	/**
		Intersects the box with the given ray.
		Returns true iff they intersect.
		Optionally, a segment can be returned which will contain the part of the ray which is inside the box.
		If the ray origin lies inside the box, then seg = [origin, intersection] is returned.
	*/
	bool intersectWithRay(const Ray &ray, Segment *seg = NULL) const;
};

