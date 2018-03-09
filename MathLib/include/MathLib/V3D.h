#pragma once

#include "MathLib.h"
#include "Matrix.h"
#include "AutoDiff.h"

// TODO: move somewhere else
typedef AutoDiffT<double, double> ScalarDiff;
typedef AutoDiffT<ScalarDiff, ScalarDiff> ScalarDiffDiff;

class P3D;
/**
* Vector in 3d.
*/
class V3D : public Vector3d {
public:

	/**
	Contructors and destructor
	*/

	V3D();

	explicit V3D(double val);

	V3D(double x, double y, double z);

	V3D(double x, double y);

	V3D(const V3D& v);

	V3D(const Vector3d& v);

	explicit V3D(const P3D& p);

	V3D(const V3D &other, double scale);

	V3D(const P3D &p1, const P3D &p2);

	~V3D();

	/**
	operators
	*/

	bool operator == (const V3D& v) const;

	bool operator != (const V3D& v) const;

	double at(int i) const;

	double& at(int i);

	//return *this + v
	V3D operator + (const V3D &v) const;

	//return *this - v
	V3D operator - (const V3D &v) const;

	//*this += v
	V3D& operator += (const V3D &v);

	//*this -= v
	V3D& operator -= (const V3D &v);

	//return *this / v
	V3D operator / (double val) const;

	//return *this * v
	V3D operator * (double val) const;

	// * this /= v
	V3D& operator /= (double val);

	// *this /= v
	V3D& operator *= (double val);

	// return - *this
	V3D operator - () const;

	/**
	useful methods
	*/

	double getComponentAlong(const V3D& other);

	double getComponentAlong(const V3D& other) const;

	void setComponentAlong(const V3D& other, double val);

	void addOffsetToComponentAlong(const V3D& other, double offset);

	void scaleComponentAlong(const V3D& other, double scale);

	void boundComponentAlong(const V3D& other, double min, double max);

	// Returns a new vector obtained by rotating the current vector. Alpha is specified in radians, and axis is assumed to be a unit vector
	V3D rotate(double alpha, const V3D &axis) const;

	// computes two vectors (a and b) that are orhtogonal to the current vector.
	void getOrthogonalVectors(V3D& a, V3D& b) const;

	// computes dot product with v
	double dot(const V3D &v) const;

	// computes cross product with v
	V3D cross(const V3D &v) const;

	// computes outerproduct matrix *this * v'
	Matrix3x3 outerProductWith(const V3D& v);

	// computes the length of the vector
	double length() const;

	// computes the squared length of the vector
	double length2() const;

	// computes the projection of the current vector on v
	V3D getProjectionOn(const V3D &v) const;

	// returns the angle between this vector and v - this method only returns angles between 0 and PI. The angle returned is measured in radians.
	double angleWith(const V3D &v) const;

	// returns the angle between between this vector and v. Given a direction n, the angle returned is between -PI and PI.
	double angleWith(const V3D &v, const V3D& n) const;

	// normalizes the vector
	V3D& toUnit();

	// returns a unit vector
	V3D unit() const;

	// makes it all zero
	void zero();

	//returns a skew symmetric matrix corresponding to this vector...
	Matrix3x3 getSkewSymmetricMatrix();

};

// Returns a new vector obtained by rotating v. Alpha is specified in radians, and axis is assumed to be a unit vector
// rotate vector using Rodrigues' rotation formula

template<class T>
Vector3T<T> rotateVec(const Vector3T<T> &v, T alpha, const Vector3T<T> &axis)
{
	assert(IS_EQUAL(axis.squaredNorm(), 1));

	T cosa = cos(alpha);
	T sina = sin(alpha);

	Vector3T<T> result = v*cosa + axis.cross(v)*sina + axis*(axis.dot(v))*(T(1) - cosa);
	return result;
}

template<class T>
Vector3T<T> drotateVec_dalpha(const Vector3T<T> &v, T alpha, const Vector3T<T> &axis)
{
	assert(IS_EQUAL(axis.squaredNorm(), 1));

	T cosa = cos(alpha);
	T sina = sin(alpha);

	Vector3T<T> result = -v*sina + axis.cross(v)*cosa + axis*(axis.dot(v))*sina;
	return result;
}

template<class T>
Vector3T<T> ddrotateVec_dalpha2(const Vector3T<T> &v, T alpha, const Vector3T<T> &axis)
{
	assert(IS_EQUAL(axis.squaredNorm(), 1));

	T cosa = cos(alpha);
	T sina = sin(alpha);

	Vector3T<T> result = -v*cosa - axis.cross(v)*sina + axis*(axis.dot(v))*cosa;
	return result;
}

// Returns a new vector obtained by rotating v. Alpha is specified in radians, and axis is assumed to be a unit vector
template<class T>
Vector3T<T> rotateVec2(const Vector3T<T> &v, T alpha, const Vector3T<T> &axis)
{
	assert(IS_EQUAL(axis.squaredNorm(), 1));
	T xP = axis(0);
	T yP = axis(1);
	T zP = axis(2);
	T cosa = cos(alpha);
	T sina = sin(alpha);

	T s[3][3] = {
		{ 0,			-zP,		yP },
		{ zP,			0,			-xP },
		{ -yP,			xP,			0 }
	};
	T UUT[3][3] = {
		{ xP*xP,		xP*yP,		xP*zP },
		{ yP*xP,		yP*yP,		yP*zP },
		{ zP*xP,		zP*yP,		zP*zP }
	};
	T I[3][3] = { { 1,0,0 },{ 0,1,0 },{ 0,0,1 } };
	T R[3][3] = { { 0,0,0 },{ 0,0,0 },{ 0,0,0 } };

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			R[i][j] = UUT[i][j] + cosa*(I[i][j] - UUT[i][j]) + sina*s[i][j];

	//now that we finally have the transformation matrix set up, we can rotate the vector
	Vector3T<T> result;

	result(0) = (R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2]);
	result(1) = (R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2]);
	result(2) = (R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2]);

	return result;
}
// Returns a (uniformly) random unit vector
V3D getRandomUnitVector();

Matrix3x3 getCrossProductMatrix(const V3D& v);

