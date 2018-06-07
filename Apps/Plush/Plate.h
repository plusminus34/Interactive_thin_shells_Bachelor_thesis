#pragma once

#include "Frame.h"

class PlateMesh;

class Plate {

public:
	Plate(vector<int>, PlateMesh *);
	inline ~Plate() {}
	P3D get_n_color();
	void draw();

public: 
	PlateMesh *mesh;
	vector<int> face_;

public:
	Frame get_frame();
	P3D hom_mm_extract(const Matrix4x4 &, const P3D &);
	P3D O2uvnt(const P3D &);
	P3D uvnt2O(const P3D &);
	// --
	bool triangle_contains(const vector<P3D> &, const P3D &);

public:
	vector<P3D> get_face_as_vecP3D();
	vector<P3D> get_shew_tri_as_vecP3D(const vector<int> &);

public:
	void recompute_triangulation();
	vector<P3D>           v_shew;
	vector<vector<int>> tri_shew;

public:
	V3D get_n();
	vector<V3D> get_uvn();
	P3D get_t();
	shared_ptr<Plane> get_Pi();

public:
	bool contains(const P3D &);


};