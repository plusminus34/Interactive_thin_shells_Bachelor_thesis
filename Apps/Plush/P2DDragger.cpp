#include "P2DDragger.h"
// --
#include <GUILib/InteractiveWidget.h>
#include <MathLib/Ray.h>

P2DDragger::P2DDragger(const vector<P3D *> &points) : Handler() {
	this->points = points;
}

P2DDragger::P2DDragger(P3D *point) : Handler() {
	this->points = { point };
}

int P2DDragger::get_closest_point_i(double xPos, double yPos) {
	Ray ray = InteractiveWidget::getRayFromScreenCoords(xPos, yPos);
	// --
	double min_d = INFINITY;
	int min_i = -1;
	for (size_t i = 0; i < points.size(); ++i) {
		P3D *p = points[i];
		double d = ray.getDistanceToPoint(*p);
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

bool P2DDragger::mouse_move(double xPos, double yPos) {
	if (LEFT_CLICKED) {
		if (selected_point_i != -1) {
			*points[selected_point_i] = get_xy0(xPos, yPos);
			return true;
		}
	}
	return false;
}

bool P2DDragger::mouse_button(int button, int action, int mods, double xPos, double yPos) {
	if (action == GLFW_PRESS) {
		if (button == GLFW_MOUSE_BUTTON_LEFT) {
			LEFT_CLICKED = true;
			selected_point_i = get_closest_point_i(xPos, yPos);
		}
	} else if (action == GLFW_RELEASE) {
		LEFT_CLICKED = false;
		RIGHT_CLICKED = false;
		selected_point_i = -1;
	}
	return false;
}
 
