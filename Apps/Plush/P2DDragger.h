#pragma once
// --
#include <PlushHelpers/helpers_star.h>
// --
#include "Handler.h"

class P2DDragger : public Handler {

public:
	P2DDragger(const vector<P3D *> &);
	P2DDragger(P3D *p);
	// --
	vector<P3D *> points;

public:
	int get_closest_point_i(double, double);
	int selected_point_i   = -1;

public:
	virtual bool mouse_move_(double xPos, double yPos); 
	virtual bool mouse_button_(int button, int action, int mods, double xPos, double yPos);

public:
	const double POINT_THRESH = .1;

};
