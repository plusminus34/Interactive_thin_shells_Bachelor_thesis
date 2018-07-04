#include "KineSimLib/Triangle.h"
#include "KineSimLib/KS_LoaderUtils.h"
#include <MathLib/Matrix.h>
#include <MathLib/P3D.h>

#define TINY_NUMBER 0.000000001

Triangle::Triangle(void){
}

Triangle::Triangle(const P3D &p1_, const P3D &p2_, const P3D &p3_){
	p1 = p1_;
	p2 = p2_;
	p3 = p3_;
}

Triangle::~Triangle(void){
}

void Triangle::computeBarycentricCoordinatesFor(const P3D& p, double &w1, double &w2, double &w3){
	//we want to write p as: p = p1 * w1 + p2 * w2 + p3 * w3, so we just need to solve a 3x3 system and we'll compute the weights...
	Matrix3x3 m;
	m(0, 0) = p1[0]; m(0, 1) = p2[0]; m(0, 2) = p3[0];
	m(1, 0) = p1[1]; m(1, 1) = p2[1]; m(1, 2) = p3[1];
	m(2, 0) = p1[2]; m(2, 1) = p2[2]; m(2, 2) = p3[2];

	Matrix3x3 mInv;
	//mInv.setToInverseOf(m);
	mInv=m.inverse();


	Vector3d sol = mInv * Vector3d(p);

	w1 = sol[0]; w2 = sol[1]; w3 = sol[2];

	//check the solution...
	Vector3d res = m * sol - p;
	if (res.norm() > 0.0001)
		Logger::print("Computing barycentric coordinates failed :(.\n");
}

/*bool Triangle::isIntersectedByRay(const Ray& r, P3D& result){
	Plane plane(p1, p2, p3);
	if (!r.getIntersectionWithPlane(plane, result))
		return false;
	double w1, w2, w3;
	computeBarycentricCoordinatesFor(result, w1, w2, w3);

	//if the weights are all positive and sum up to one, it means we're in the point is in the triangle
	return (w1 > 0 && w2 > 0 && w3 > 0 && fabs(w1 + w2 + w3 - 1) < TINY_NUMBER);
}*/



