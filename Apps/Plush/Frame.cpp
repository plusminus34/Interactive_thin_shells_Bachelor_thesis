#include "Frame.h"
 
Frame::Frame(const Matrix3x3 &B, const P3D &s) {
	this->B = B;
	this->s = s;
}

Frame::Frame(const vector<V3D> &V, const P3D &s) {
	Matrix3x3 B;
	B.setZero();
	B.block<3, 1>(0, 0) = V[0];
	B.block<3, 1>(0, 1) = V[1];
	B.block<3, 1>(0, 2) = V[2];
	*this = Frame(B, s);
}

void Frame::draw() {}

void Frame::glAffineTransform() { 
	glTranslateP3D(s);
	Quaternion q;
	q.setRotationFrom(B);
	glRotateQuat(q);
}

Matrix4x4 Frame::get_A() { 
	Matrix4x4 A;
	A.setIdentity();
	A.block<3, 3>(0, 0) = B;
	A.block<3, 1>(0, 3) = s;
	return A;
}

P3D Frame::local2world(const P3D &p_local) {
	// -- // https://github.com/openMVG/openMVG/issues/709
	Vector4d p_local_hom = p_local.homogeneous();
	Vector3d world = (get_A() * p_local_hom).hnormalized();
	return (P3D)world;
}


P3D Frame::world2local(const P3D &p_world) {
	// -- // ""
	Vector4d p_world_hom = p_world.homogeneous();
	Vector3d local = (get_A().inverse() * p_world_hom).hnormalized();
	return (P3D)local;
}
