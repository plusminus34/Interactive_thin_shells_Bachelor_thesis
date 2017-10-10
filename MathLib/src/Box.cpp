#include "../include/MathLib/Box.h"
#include "Utils/Utils.h"

Box::Box(){
}

Box::Box(double heading, const P3D &pos, const P3D &c1, const P3D &c2) : orientation(getRotationQuaternion(heading, Globals::worldUp)), position(pos), c1(c1), c2(c2){
}

Box::Box(const Quaternion &q, const P3D &pos, const P3D &c1, const P3D &c2) : orientation(q), position(pos), c1(c1), c2(c2) {
}

double Box::getDistanceToPoint(const P3D &pt, P3D *closestPointOnBox) const{
	// Vector of sphere center in box coordinates
	V3D p = pt - position;
	V3D t = orientation.inverseRotate(p);

	// Project 't' onto the box faces if its outside the box
	bool onBorder = false;
	if (t[0] < c1[0]) { t[0] = c1[0]; onBorder = true; }
	if (t[0] > c2[0]) { t[0] = c2[0]; onBorder = true; }

	if (t[1] < c1[1]) { t[1] = c1[1]; onBorder = true; }
	if (t[1] > c2[1]) { t[1] = c2[1]; onBorder = true; }

	if (t[2] < c1[2]) { t[2] = c1[2]; onBorder = true; }
	if (t[2] > c2[2]) { t[2] = c2[2]; onBorder = true; }

	if (!onBorder){
		// Point is inside the box.
		if (closestPointOnBox)
			*closestPointOnBox = pt;
		return 0;
	}
	else {
		// point is outside the box. 't' is its projection onto the box in box space.
		V3D q = orientation.rotate(t);
		if (closestPointOnBox)
			*closestPointOnBox = position + q;
		return (p - q).length();
	}
}

bool Box::intersectWithRay(const Ray &ray, Segment *seg) const{
	// Transform the ray to box space
	P3D rp = P3D(orientation.inverseRotate(V3D(position, ray.origin)));
	V3D rd = orientation.inverseRotate(ray.direction);

	// Smits' method
	// See http://people.csail.mit.edu/amy/papers/box-jgt.pdf
	double tmin, tmax, tymin, tymax, tzmin, tzmax;
	double divx = 1.0 / rd[0];
	if (divx >= 0) {
		tmin = (c1[0] - rp[0]) * divx;
		tmax = (c2[0] - rp[0]) * divx;
	}
	else {
		tmin = (c2[0] - rp[0]) * divx;
		tmax = (c1[0] - rp[0]) * divx;
	}

	double divy = 1.0 / rd[1];
	if (divy >= 0) {
		tymin = (c1[1] - rp[1]) * divy;
		tymax = (c2[1] - rp[1]) * divy;
	}
	else {
		tymin = (c2[1] - rp[1]) * divy;
		tymax = (c1[1] - rp[1]) * divy;
	}

	if (tmin > tymax || tymin > tmax)
		return false;
	if (tymin > tmin)
		tmin = tymin;
	if (tymax < tmax)
		tmax = tymax;

	double divz = 1.0 / rd[2];
	if (divz >= 0) {
		tzmin = (c1[2] - rp[2]) * divz;
		tzmax = (c2[2] - rp[2]) * divz;
	}
	else {
		tzmin = (c2[2] - rp[2]) * divz;
		tzmax = (c1[2] - rp[2]) * divz;
	}

	if (tmin > tzmax || tzmin > tmax)
		return false;
	if (tzmin > tmin)
		tmin = tzmin;
	if (tzmax < tmax)
		tmax = tzmax;

	if (tmin < 0 && tmax < 0)
	{
		// Box is behind the ray origin
		return false;
	}
	else if (tmin < 0 && tmax >= 0)
	{
		// Ray origin is within the box
		if (seg != NULL)
			*seg = Segment(ray.origin, ray.getPointAt(tmax));
		return true;
	}
	else
	{
		if (seg != NULL)
			*seg = Segment(ray.getPointAt(tmin), ray.getPointAt(tmax));
		return true;
	}
}

