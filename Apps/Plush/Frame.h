#pragma once
// --
#include <PlushHelpers/helpers_star.h>

class Frame {

public:
	Frame(const Matrix3x3 &, const P3D &);
	Frame(const vector<V3D> &, const P3D &);
	virtual void draw();

public:
	Matrix3x3 B;
	P3D s;

public:
	void glAffineTransform();
	Matrix4x4 get_A();
	P3D local2world(const P3D &);
	P3D world2local(const P3D &);
};
