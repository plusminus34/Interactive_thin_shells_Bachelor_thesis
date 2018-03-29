#include "../include/MathLib/P3D.h"
#include "../include/MathLib/V3D.h"
#include "../include/MathLib/MathLib.h"

P3D::P3D() {
	at(0) = at(1) = at(2) = 0;
}

P3D::P3D(double val) {
	at(0) = at(1) = at(2) = val;
}

P3D::P3D(double x, double y, double z) {
	this->at(0) = x;
	this->at(1) = y;
	this->at(2) = z;
}

P3D::P3D(double x, double y) {
	this->at(0) = x;
	this->at(1) = y;
	this->at(2) = 0;
}

P3D::P3D(const V3D& v) {
	Vector3d::operator=(v);
}

P3D::P3D(const P3D& p) {
	Vector3d::operator=(p);
}

P3D::P3D(const Vector3d& v) {
	Vector3d::operator=(v);
}

P3D::~P3D() {
}

bool P3D::operator == (const P3D& p) const {
	return IS_EQUAL(this->at(0), p.at(0)) && IS_EQUAL(this->at(1), p.at(1)) && IS_EQUAL(this->at(2), p.at(2));
}

bool P3D::operator != (const P3D& p) const {
	return !(*this == p);
}

bool P3D::operator < (const P3D& p) const {
	return (this->at(0) < p.at(0) - EPSILON) || (this->at(0) < p.at(0) + EPSILON && this->at(1) < p.at(1) - EPSILON)
		|| (this->at(0) < p.at(0) + EPSILON && this->at(1) < p.at(1) + EPSILON && this->at(2) < p.at(2) - EPSILON);
}

double P3D::at(int i) const {
	return (*this)[i];
}

double& P3D::at(int i) {
	return (*this)[i];
}

//return *this + v
P3D P3D::operator+(const Vector3d &v) const {
	return P3D(Vector3d::operator+(v));
}

//return *this + v
P3D P3D::operator+(const V3D &v) const {
	return P3D(Vector3d::operator+(v));
}

//*this += p
P3D& P3D::operator += (const P3D& p) {
	Vector3d::operator+=(p);
	return *this;
}

//*this += v
P3D& P3D::operator += (const V3D &v) {
	Vector3d::operator+=(v);
	return *this;
}

//*this -= v
P3D& P3D::operator -= (const V3D &v) {
	Vector3d::operator-=(v);
	return *this;
}

//return *this / v
P3D P3D::operator / (double val) const {
	return P3D(Vector3d::operator/(val));
}

//return *this * v
P3D P3D::operator * (double val) const {
	return P3D(Vector3d::operator*(val));
}

// * this /= v
P3D& P3D::operator /= (double val) {
	Vector3d::operator/=(val);
	return *this;
}

// *this *= v
P3D& P3D::operator *= (double val) {
	Vector3d::operator*=(val);
	return *this;
}

// return - *this
P3D P3D::operator-() const {
	return P3D(Vector3d::operator-());
}

// return the vector between the two points
V3D P3D::operator-(const P3D &p) const {
	return V3D(Vector3d::operator-(p));
}

P3D P3D::operator-(const V3D &p) const
{
	return P3D(Vector3d::operator-(p));
}

double P3D::getComponentAlong(const V3D& other) {
	return Vector3d::dot(other);
}

double P3D::getComponentAlong(const V3D& other) const {
	return Vector3d::dot(other);
}

void P3D::setComponentAlong(const V3D& other, double val) {
	double oldVal = getComponentAlong(other);
	*this += other*(val - oldVal);
}

void P3D::addOffsetToComponentAlong(const V3D& other, double offset) {
	*this += (other*offset);
}

void P3D::scaleComponentAlong(const V3D& other, double scale) {
	setComponentAlong(other, getComponentAlong(other) * scale);
}

void P3D::boundComponentAlong(const V3D& other, double min, double max) {
	double val = getComponentAlong(other);
	if (val < min) setComponentAlong(other, min);
	if (val > max) setComponentAlong(other, max);
}

// makes it all zero
void P3D::zero() {
	at(0) = at(1) = at(2) = 0;
}

// compute surface area of a triangle
double computeTriangleSurfaceArea(P3D & p0, P3D & p1, P3D & p2) {
	return 0.5 * V3D(p0, p1).cross(V3D(p0, p2)).length();
}



