#pragma once
// --
#include <MathLib\Plane.h>
#include <GUILib\GLTrackingCamera.h>
#include "Frame.h"

class Handler {

public:
	Handler();
	inline virtual void draw() {};

public:
	P3D get_xy0(double, double);
	P3D get_xyz(double, double, GLTrackingCamera *, Plane *);

public:
	bool ACTIVE = true;

public:
	virtual bool mouse_move_(double xPos, double yPos)                                     { return false; }
	virtual bool mouse_button_(int button, int action, int mods, double xPos, double yPos) { return false; }
	virtual bool mouse_wheel_(double xOffset, double yOffset)                              { return false; }
	virtual bool key_event_(int key, int action, int mods)                                 { return false; }
	bool mouse_move(double xPos, double yPos)                                     { return (ACTIVE) ? mouse_move_(xPos, yPos) : false; }
	bool mouse_button(int button, int action, int mods, double xPos, double yPos) { return (ACTIVE) ? mouse_button_(button, action, mods, xPos, yPos) : false; }
	bool mouse_wheel(double xOffset, double yOffset)                              { return (ACTIVE) ? mouse_wheel_(xOffset, yOffset) : false; }
	bool key_event(int key, int action, int mods)                                 { return (ACTIVE) ? key_event_(key, action, mods) : false; }

public:
	bool LEFT_CLICKED = false;
	bool RIGHT_CLICKED = false; 

};
