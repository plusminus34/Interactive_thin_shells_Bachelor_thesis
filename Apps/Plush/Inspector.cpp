#include "Inspector.h"
#include "GUILib\InteractiveWidget.h"

Inspector::Inspector(SimulationMesh *mesh) : Handler() {
	this->mesh = mesh;
	this->nodes   = mesh->nodes;
	this->tendons = mesh->tendons;
	// tweak_bar = TwNewBar("inspector");
	// TwDefine(" inspector position='300 0' ");
	// TwDefine(" inspector color='0 0 0' text=light ");
}

void Inspector::draw() {
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glPointSize(8.);
	glLineWidth(8.);

	dVector &y = mesh->x;

	auto guarded_node_draw = [this, y](int i) {
		if (i != -1) {
			auto &node = nodes[i];
			// --
			glBegin(GL_POINTS); {
				glP3Dz(node->getCoordinates(y), Z);
			} glEnd();
		}
	};

	auto guarded_tendon_draw = [this, y](int i) {
		if (i != -1) {
			auto &tendon = tendons[i];
			// --
			glBegin(GL_LINE_STRIP); {
				for (auto &waypoint : tendon->waypoints) {
					glP3Dz(waypoint->getCoordinates(y), Z);
				}
			} glEnd();

			glBegin(GL_POINTS); {
				for (auto &waypoint : tendon->waypoints) {
					glP3Dz(waypoint->getCoordinates(y), Z);
				}
			} glEnd();
		}
	};

	// nodes
	if (hovered_node_i != selected_node_i) {
		set_color(HOVER_COLOR);
		guarded_node_draw(hovered_node_i);
		set_color(SELECTED_COLOR);
		guarded_node_draw(selected_node_i);
	} else {
		set_color(color_swirl(.5, HOVER_COLOR, SELECTED_COLOR));
		guarded_node_draw(selected_node_i); 
	}

	// tendons
	if (!PREFER_NODES) {
		if (hovered_tendon_i != selected_tendon_i) {
			set_color(HOVER_COLOR);
			guarded_tendon_draw(hovered_tendon_i);
			set_color(SELECTED_COLOR);
			guarded_tendon_draw(selected_tendon_i);
		} else {
			set_color(color_swirl(.5, HOVER_COLOR, SELECTED_COLOR));
			guarded_tendon_draw(selected_tendon_i);
		}
	}

	glPopAttrib();
}

pair<int, double> Inspector::get_closest_node_i(double xPos, double yPos) {
	Ray ray = InteractiveWidget::getRayFromScreenCoords(xPos, yPos);
	// --
	double min_d = INFINITY;
	int min_i = -1;
	for (size_t i = 0; i < nodes.size(); ++i) {
		P3D p = nodes[i]->getCurrentPosition(); // FORNOW
		double d = ray.getDistanceToPoint(p);
		bool better_candidate = (d < min_d);
		if (d < min_d) {
			min_d = d;
			min_i = i;
		}
	}

	if (min_d < POINT_THRESH) {
		return make_pair(min_i, min_d);
	} else {
		return make_pair(-1, INFINITY);
	}
}

pair<int, double> Inspector::get_closest_tendon_i(double xPos, double yPos) {
	Ray ray = InteractiveWidget::getRayFromScreenCoords(xPos, yPos);
	// --
	double min_d = INFINITY;
	int min_i = -1;
	for (size_t i = 0; i < tendons.size(); ++i) {
		auto &tendon = tendons[i];
		for (int a = 0; a < (int) tendon->waypoints.size() - 1; ++a) {
			int b = a + 1;
			P3D pa = tendon->waypoints[a]->getCurrentPosition(); // FORNOW
			P3D pb = tendon->waypoints[b]->getCurrentPosition(); // FORNOW

			double d = ray.getDistanceToSegment(pa, pb);
			bool better_candidate = (d < min_d);
			if (d < min_d) {
				min_d = d;
				min_i = i;
			}
		}
	}

	if (min_d < SEGMENT_THRESH) {
		return make_pair(min_i, min_d);
	} else {
		return make_pair(-1, INFINITY);
	}
}


void Inspector::interrogate(double xPos, double yPos, int &node_i, double &node_d, int &tendon_i, double &tendon_d) {
	pair<int, double> node_ret = get_closest_node_i(xPos, yPos);
	pair<int, double> tendon_ret = get_closest_tendon_i(xPos, yPos);
	node_i   = node_ret.first;
	node_d   = node_ret.second;
	tendon_i = tendon_ret.first;
	tendon_d = tendon_ret.second;

}
 
bool Inspector::key_event(int key, int action, int mods) {
	PREFER_NODES = mods & GLFW_MOD_SHIFT;
	return true;
}

bool Inspector::mouse_move(double xPos, double yPos) {
	if (selected_tendon_i != -1) {
		tendons[selected_tendon_i]->set_alphac(TMP_ALPHAC);
	}
	// --
	int node_i, tendon_i;               double node_d, tendon_d;
	interrogate(xPos, yPos, node_i, node_d, tendon_i, tendon_d);
	// --
	bool NODE_HIT = node_i != -1;
	bool TENDON_HIT = tendon_i != -1;
	bool NODE_CLOSER = node_d < tendon_d;
	// --
	auto hover_node   = [this, node_i] ()   { hovered_node_i    = node_i;   };
	auto hover_tendon = [this, tendon_i] () { hovered_tendon_i  = tendon_i; };
	// --
	hovered_node_i   = -1;
	hovered_tendon_i = -1; 
	if (NODE_HIT && TENDON_HIT) {
		if (PREFER_NODES) {
			hover_node();
		} else {
			if (NODE_CLOSER) {
				hover_node();
			} else {
				hover_tendon();
			}
		}
	} else if (NODE_HIT) {
		hover_node();
	} else if (TENDON_HIT) {
		hover_tendon();
	}
	return false;
}

bool Inspector::mouse_button(int button, int action, int mods, double xPos, double yPos) {
	int node_i, tendon_i;               double node_d, tendon_d;
	interrogate(xPos, yPos, node_i, node_d, tendon_i, tendon_d);

	if (action == GLFW_PRESS) {
		if (button == GLFW_MOUSE_BUTTON_LEFT) {
			LEFT_CLICKED = true;
			// --
			bool NODE_HIT = node_i != -1;
			bool TENDON_HIT = tendon_i != -1;
			bool NODE_CLOSER = node_d < tendon_d;
			// --
			auto toggle_node = [this, node_i] () { selected_node_i = (node_i != selected_node_i) ? node_i : -1; };
			auto toggle_tendon = [this, tendon_i] () {
				selected_tendon_i = (tendon_i != selected_tendon_i) ? tendon_i : -1;
				TMP_ALPHAC = (selected_tendon_i != -1) ? tendons[selected_tendon_i]->get_alphac(mesh->balphac) : 0.;
			}; 
			// --
			if (NODE_HIT && TENDON_HIT) {
				if (PREFER_NODES) {
					toggle_node();
				} else {
					if (NODE_CLOSER) {
						toggle_node();
					} else {
						toggle_tendon();
					}
				}
			} else if (NODE_HIT) {
				toggle_node();
			} else if (TENDON_HIT) {
				toggle_tendon();
			}
		}
	// FORNOW
	} else if (action == GLFW_RELEASE) {
		LEFT_CLICKED = false;
		RIGHT_CLICKED = false;
		// selected_node_i = -1;
	}

	/*
	// Refresh TW
	TwHandleErrors(dummyErrorHandler); {
		TwRemoveVar(tweak_bar, "node i");
		TwRemoveVar(tweak_bar, "tendon i");
		TwRemoveVar(tweak_bar, "tendon alphac");
	} TwHandleErrors(defaultErrorHandler);
	// --
	if (selected_node_i != -1) {
		TwAddVarRO(tweak_bar, "node i", TW_TYPE_INT32, &(nodes[selected_node_i]->nodeIndex), "");
	}
	// --
	if (selected_tendon_i != -1) {
		TwAddVarRO(tweak_bar, "tendon i", TW_TYPE_INT32,       &(tendons[selected_tendon_i]->tendonIndex), "");
		TwAddVarRW(tweak_bar, "tendon alphac", TW_TYPE_DOUBLE, &TMP_ALPHAC, "step=.01 keyIncr=. keyDecr=,");
	}
	*/
 
	return false;
}
 
bool Inspector::mouse_wheel(double xOffset, double yOffset) {
	if (selected_tendon_i != -1) {
		auto &tendon = tendons[selected_tendon_i];
		TMP_ALPHAC = tendon->get_alphac(mesh->balphac) + .01*yOffset;
		tendon->set_alphac(TMP_ALPHAC);
		return true;
	}
	return false;
}
 