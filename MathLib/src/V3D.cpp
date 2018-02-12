#include "../include/MathLib/V3D.h"
#include "../include/MathLib/P3D.h"
#include <assert.h>

V3D::V3D() {
	at(0) = at(1) = at(2) = 0;
}

V3D::V3D(double val) {
	at(0) = at(1) = at(2) = val;
}

V3D::V3D(double x, double y, double z) {
	this->at(0) = x;
	this->at(1) = y;
	this->at(2) = z;
}

V3D::V3D(double x, double y) {
	this->at(0) = x;
	this->at(1) = y;
	this->at(2) = 0;
}

V3D::V3D(const V3D& v) {
	Vector3d::operator=(v);
}

V3D::V3D(const P3D& v) {
	Vector3d::operator=(v);
}

V3D::V3D(const Vector3d& v) {
	Vector3d::operator=(v);
}

V3D::V3D(const V3D &other, double scale) {
	Vector3d::operator=((scale * other).eval());
}

V3D::V3D(const P3D &p1, const P3D &p2) {
	Vector3d::operator=((p2 - p1).eval());
}

V3D::~V3D() {
}


bool V3D::operator == (const V3D& v) const {
	return IS_EQUAL(this->at(0), v.at(0)) && IS_EQUAL(this->at(1), v.at(1)) && IS_EQUAL(this->at(2), v.at(2));
}

bool V3D::operator != (const V3D& v) const {
	return !(*this == v);
}

double V3D::at(int i) const {
	return (*this)[i];
}

double& V3D::at(int i) {
	return (*this)[i];
}

//return *this + v
V3D V3D::operator + (const V3D &v) const {
	return V3D(Vector3d::operator+(v));
}

//*this += v
V3D& V3D::operator += (const V3D &v) {
	Vector3d::operator+=(v);
	return *this;
}

//*this -= v
V3D& V3D::operator -= (const V3D &v) {
	Vector3d::operator-=(v);
	return *this;
}

//return *this / v
V3D V3D::operator / (double val) const {
	return V3D(Vector3d::operator/(val));
}

//return *this * v
V3D V3D::operator * (double val) const {
	return V3D(Vector3d::operator*(val));
}

// * this /= v
V3D& V3D::operator /= (double val) {
	Vector3d::operator/=(val);
	return *this;
}

// *this *= v
V3D& V3D::operator *= (double val) {
	Vector3d::operator*=(val);
	return *this;
}

// return - *this
V3D V3D::operator - () const {

	return static_cast<V3D>(Vector3d::operator-());
}

// *this - v
V3D V3D::operator - (const V3D &v) const {
	return static_cast<V3D>(Vector3d::operator-(v));
}

double V3D::getComponentAlong(const V3D& other) {
	return Vector3d::dot(other);
}

double V3D::getComponentAlong(const V3D& other) const {
	return Vector3d::dot(other);
}

void V3D::setComponentAlong(const V3D& other, double val) {
	double oldVal = getComponentAlong(other);
	*this *= (val - oldVal);
}

void V3D::addOffsetToComponentAlong(const V3D& other, double offset) {
	*this += (other*offset);
}

void V3D::scaleComponentAlong(const V3D& other, double scale) {
	setComponentAlong(other, getComponentAlong(other) * scale);
}

void V3D::boundComponentAlong(const V3D& other, double min, double max) {
	double val = getComponentAlong(other);
	if (val < min) setComponentAlong(other, min);
	if (val > max) setComponentAlong(other, max);
}

// Returns a new vector obtained by rotating the current vector. Alpha is specified in radians, and axis is assumed to be a unit vector
V3D V3D::rotate(double alpha, const V3D &axis) const {
	return rotateVec(*this, alpha, axis);
}


// computes two vectors (a and b) that are orhtogonal to the current vector.
void V3D::getOrthogonalVectors(V3D& a, V3D& b) const {
	//try to choose a vector in the y-z plane, if the z-coordinate is significant enough
	if (at(0)*at(0) / (length2()) < 0.5) //if the x component of the current vector is not too large, relatively speaking, then choose x as the main component for a (they won't be aligned perfectly)
		a = V3D(1, 0, 0);
	else //if the x component of the current vector is large, then the y (or z) components cannot be the only non-zero values, so a is not conlinear with the current vector
		a = V3D(0, 1, 0);
	b = this->cross(a); //b is orthogonal to both the current vector and a
	a = b.cross(*this); //and a is orthogonal to both a and this...
	a.toUnit();
	b.toUnit();
}

// computes outerproduct matrix v * v'
Matrix3x3 V3D::outerProductWith(const V3D& v) {
	return Vector3d(*this) * v.transpose();
}

// computes dot product with v
double V3D::dot(const V3D &v) const {
	return Vector3d::dot(v);
}

// computes cross product with v
V3D V3D::cross(const V3D &v) const {
	return Vector3d::cross(v);
}

double sqrt__(double val) { return sqrt(val); }

// computes the length of the vector
double V3D::length() const {
	return sqrt__(length2());
}

// computes the squared length of the vector
double V3D::length2() const {
	return (*this).squaredNorm();
}

// computes the projection of the current vector on v
V3D V3D::getProjectionOn(const V3D &v) const {
	/*  Here's how it goes...
	proj = V/|V| * |U| * cos(angle) = V/|V| * |U| * U.V / |U| / |V|
	after simplifying we get:
	U.V
	proj = ------- * V
	|V|*|V|
	*/
	V3D u = *this;
	return v*(u.dot(v) / (v.length2()));
}

// returns the angle between this vector and v - this method only returns angles between 0 and PI. The angle returned is measured in radians.
double V3D::angleWith(const V3D &v) const {
	// U.V = |U|*|V|*cos(angle)
	// therefore angle = inverse cos (U.V/(|U|*|V|))
	double result = this->dot(v) / (this->length() * v.length());
	return safeACOS(result);
}


// returns the angle between between this vector and v. Given a direction n, the angle returned is between -PI and PI.
double V3D::angleWith(const V3D &v, const V3D& n) const {
	// U.V = |U|*|V|*cos(angle)
	double cosAVal = this->dot(v) / (this->length() * v.length());
	V3D crossProd = this->cross(v);
	// |U x V| = |U|*|V|*sin(angle)
	double sinAVal = crossProd.length() / (this->length() * v.length());

	if (crossProd.dot(n) < 0) sinAVal *= -1;

	return atan2(sinAVal, cosAVal);
}

// normalizes the vector
V3D& V3D::toUnit() {
	double d = Vector3d::norm();
	if (IS_ZERO(d))
		return *this;
	*this /= d;
	return (*this);
}

// returns a unit vector
V3D V3D::unit() const {
	V3D v(this->at(0), this->at(1), this->at(2));
	v.toUnit();
	return v;
}

// makes it all zero
void V3D::zero() {
	at(0) = at(1) = at(2) = 0;
}

// Returns a (uniformly) random unit vector
V3D getRandomUnitVector() {
	return V3D(randNumberIn01Range(), randNumberIn01Range(), randNumberIn01Range()).unit();
}

//make skew symmetric for vec
Matrix3x3 V3D::getSkewSymmetricMatrix() {
	Matrix3x3 result;
	result << 0, -z(), y(), z(), 0, -x(), -y(), x(), 0;
	return result;
}

Matrix3x3 getCrossProductMatrix(const V3D& v)
{
	Matrix3x3 M;
	M.setZero();

	M(0, 1) = -v[2]; M(0, 2) = v[1];
	M(1, 0) = v[2];  M(1, 2) = -v[0];
	M(2, 0) = -v[1]; M(2, 1) = v[0];

	return M;
}
