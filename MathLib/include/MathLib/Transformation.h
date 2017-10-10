#pragma once
#include "Quaternion.h"
#include "P3D.h"

class Transformation {
public:
	Matrix3x3 R;
	Vector3d T;

public:
	Transformation(const Matrix3x3& _R = Matrix3x3::Identity(), const Vector3d& _T = Vector3d::Zero()) : R(_R), T(_T) {}
	~Transformation();

	P3D transform(const P3D& p) {
		Vector3d tp = R * p + T;
		return P3D(tp[0], tp[1], tp[2]);
	}

	V3D transform(const V3D& v) {
		Vector3d tv = R * v;
		return V3D(tv[0], tv[1], tv[2]);
	}

	// TODO: put this somewhere else
	// apply this transformation to current openGL matrix.
	//void applyGLMatrixTransform();

	Transformation inverse() {
		Transformation trans;
		trans.R = R.inverse();
		trans.T = -trans.R * T;
		return trans;

	}

	Transformation operator*(const Transformation& other) {
		Transformation trans;
		// use rotation matrix based multiplication to avoid singularity.
		trans.R = R * other.R;
		trans.T = R * other.T + T;
		return trans;
	}

	Transformation& operator*=(const Transformation& other) {
		Transformation trans;
		// use rotation matrix based multiplication to avoid singularity.
		trans.R = R * other.R;
		trans.T = R * other.T + T;
		*this = trans;
		return *this;
	}
};

