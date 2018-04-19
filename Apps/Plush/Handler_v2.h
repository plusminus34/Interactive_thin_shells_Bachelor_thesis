#pragma once
// --
#include <MathLib\Plane.h>
#include <GUILib\GLTrackingCamera.h>
#include "Frame.h"

class Handler_v2 {

public:
	Handler_v2(Frame *local2world=nullptr);
	inline virtual void draw() {};
	Frame *frame = new Frame();

public:
	P3D get_xy0(double, double);
	// P3D get_xyz(double, double, GLTrackingCamera *, Plane *);

public:
	bool ACTIVE = true;

public:
	virtual bool mouse_move_(P3D xy0_local) { return false; }
	virtual bool mouse_button_(int button, int action, int mods, P3D xy0_local) { return false; }
	virtual bool mouse_wheel_(double xOffset, double yOffset) { return false; }
	virtual bool key_event_(int key, int action, int mods) { return false; }
	virtual bool character_event_(int key, int mods) { return false; }
	// --
	virtual bool mouse_move(double xPos, double yPos)                                     final { return (ACTIVE) ? mouse_move_(get_xy0(xPos, yPos)) : false; }
	virtual bool mouse_button(int button, int action, int mods, double xPos, double yPos) final { return (ACTIVE) ? mouse_button_(button, action, mods, get_xy0(xPos, yPos)) : false; }
	virtual bool mouse_wheel(double xOffset, double yOffset)                              final { return (ACTIVE) ? mouse_wheel_(xOffset, yOffset) : false; }
	virtual bool key_event(int key, int action, int mods)                                 final { return (ACTIVE) ? key_event_(key, action, mods) : false; }
	virtual bool character_event(int key, int mods)                                       final { return (ACTIVE) ? character_event_(key, mods) : false; }

public:
	bool LEFT_CLICKED = false;
	bool RIGHT_CLICKED = false; 

};
