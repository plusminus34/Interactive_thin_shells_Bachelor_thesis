#pragma once
// --
#include <MathLib\Plane.h>
#include <GUILib\GLTrackingCamera.h>

class Handler {

public:
	Handler();
	inline virtual void draw() {};
	V3D *offset = new V3D();

public:
	P3D get_xy0(double, double);
	P3D get_xyz(double, double, GLTrackingCamera *, Plane *);

public:
	virtual bool mouse_move(double xPos, double yPos)                                     { return false; }
	virtual bool mouse_button(int button, int action, int mods, double xPos, double yPos) { return false; }
	virtual bool mouse_wheel(double xOffset, double yOffset)                              { return false; }
	virtual bool key_event(int key, int action, int mods)                                 { return false; }

public:
	bool LEFT_CLICKED = false;
	bool RIGHT_CLICKED = false; 

};
