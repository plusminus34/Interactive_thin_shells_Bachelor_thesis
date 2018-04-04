#pragma once
// --
#include <PlushHelpers/helpers_star.h>
// --
#include "Handler_v2.h"

class P2DDragger_v2 : public Handler_v2 {

public:
	P2DDragger_v2(const vector<P3D *> &, Frame *frame=nullptr, bool FREEZE_X=false, bool FREEZE_Y=false);
	// P2DDragger_v2(P3D *p, Frame *frame);

public:
	vector<P3D *> points;
	bool FREEZE_X;
	bool FREEZE_Y;

public:
	int get_closest_point_i(P3D xy0);
	int selected_point_i   = -1;

public:
	virtual bool mouse_move_(P3D xy0_local); // const &
	virtual bool mouse_button_(int button, int action, int mods, P3D xy0_local);

public:
	const double POINT_THRESH = .1;

};
