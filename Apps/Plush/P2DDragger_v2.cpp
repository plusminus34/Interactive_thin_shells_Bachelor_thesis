#include "P2DDragger_v2.h"
// --
#include <GUILib/InteractiveWidget.h>
#include <MathLib/Ray.h>

P2DDragger_v2::P2DDragger_v2(const vector<P3D *> &points, Frame *frame, bool FREEZE_X, bool FREEZE_Y) : Handler_v2(frame) {
	this->points = points;
	this->FREEZE_X = FREEZE_X;
	this->FREEZE_Y = FREEZE_Y;
}

// P2DDragger_v2::P2DDragger_v2(P3D *point, Frame *frame) : Handler_v2() {
// 	this->points = { point };
// }

int P2DDragger_v2::get_closest_point_i(P3D xy0) {
	double min_d = INFINITY;
	int min_i = -1;
	for (size_t i = 0; i < points.size(); ++i) {
		P3D *p = points[i];
		double d = V3D(*p, xy0).length();
		bool better_candidate = (d < min_d);
		if (d < min_d) {
			min_d = d;
			min_i = i;
		}
	}

	if (min_d < POINT_THRESH) {
		return min_i;
	} else {
		return -1;
	}
}

bool P2DDragger_v2::mouse_move_(P3D xy0) {
	if (LEFT_CLICKED) {
		if (selected_point_i != -1) {
			if (!FREEZE_X) { points[selected_point_i]->x() = xy0.x(); };
			if (!FREEZE_Y) { points[selected_point_i]->y() = xy0.y(); };
			return true;
		}
	}
	return false;
}

bool P2DDragger_v2::mouse_button_(int button, int action, int mods, P3D xy0) {
	if (action == GLFW_PRESS) {
		if (button == GLFW_MOUSE_BUTTON_LEFT) {
			LEFT_CLICKED = true;
			selected_point_i = get_closest_point_i(xy0);
		}
	} else if (action == GLFW_RELEASE) {
		LEFT_CLICKED = false;
		RIGHT_CLICKED = false;
		selected_point_i = -1;
	}
	return false;
}
 
