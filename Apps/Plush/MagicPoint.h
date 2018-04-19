#pragma once
// --
#include "Plate.h"
// --
#include <GUILib/GLUtils.h>
#include <GUILib/GLTrackingCamera.h>
// --
#include <MathLib/Ray.h>

class PlateMesh; 

class MagicPoint {

public:
	virtual void draw();

private:
	P3D s = P3D(0., 0.); 

public:
	inline P3D get_s() { return this->s; }
	inline V3D get_n() { return (PLATE != nullptr) ? PLATE->get_n() : get_fallback_plane()->n; };

public:
	MagicPoint(GLTrackingCamera *, PlateMesh *, Plane *);
	GLTrackingCamera *camera;
	PlateMesh *mesh;
	Plane *fallback_plane_;
	shared_ptr<Plane> get_fallback_plane();
	P3D s_prime_prev_;
	void SAFE_update_s(const P3D &s);
 
	bool attempt_project_onto_mesh();
	bool project_onto_fallback_plane();
	P3D get_projection_of(const P3D &s, shared_ptr<Plane> Pi);
	void enforce_fallback();

	P3D get_intersection_of_curr_far_ray_with(shared_ptr<Plane> Pi);
	P3D get_intersection_of_curr_far_ray_with(shared_ptr<Plane> Pi, double &d);

	typedef Ray FarRay;
	static FarRay getFarRayFromScreenCoords(double, double);
	FarRay curr_far_ray = FarRay(P3D(), V3D(0.,0.,-1.));

	bool intersects(FarRay);
	
	// --

	bool rebind();

public:
	bool SELECTED = false; 
	bool DROPPED = false;
	bool ON_MESH = false;
	Plate *PLATE = nullptr;
	P3D COLOR = WHITE;

public:
	// int get_closest_point_i(double, double);
	// int selected_point_i   = -1;

public:
	// virtual bool mouse_move(double xPos, double yPos); 
	// virtual bool mouse_button(int button, int action, int mods, double xPos, double yPos);

public:
	const double POINT_THRESH = .05;
	const double SEGMENT_THRESH = .05;

};
