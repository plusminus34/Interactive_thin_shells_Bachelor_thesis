#pragma once

#include <MathLib/P3D.h>
#include <MathLib/Ray.h>

class Triangle{
public:
	//coords of the triangle...
	P3D p1, p2, p3;

public:
	Triangle(void);
	Triangle(const P3D &p1_, const P3D &p2_, const P3D &p3_);
	~Triangle(void);

	void computeBarycentricCoordinatesFor(const P3D& p, double &w1, double &w2, double &w3);
	//bool isIntersectedByRay(const Ray& r, P3D& result);

};
