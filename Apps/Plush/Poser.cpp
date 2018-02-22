#include "Poser.h"

Poser::Poser(SimulationMesh *mesh, SoftIKSolver *ik) : Handler() {
	this->mesh = mesh;
	this->ik = ik;
}

void Poser::draw() {
	vector<P3D> tmp_x;
	vector<P3D> tmp_x_prime;

	for (auto &node : active_nodes) {
		tmp_x.push_back(node->getCurrentPosition());
		tmp_x_prime.push_back(node->getTargetPosition());
	}

	glPushAttrib(GL_ALL_ATTRIB_BITS); {
		glLineWidth(3);
		glBegin(GL_LINES); {
			for (size_t i = 0; i < tmp_x.size(); ++i) {
				set_color(ORCHID);
				glP3D(tmp_x[i]);
				set_color(HENN1NK);
				glP3D(tmp_x_prime[i]);
			} 
		} glEnd();
	} glPopAttrib();
}

Node *Poser::get_closest_node(double xPos, double yPos, const dVector &y, vector<Node *> query_nodes) {
	Ray ray = InteractiveWidget::getRayFromScreenCoords(xPos, yPos);
	// --
	double min_d = INFINITY;
	Node *min_node = nullptr;
	for (auto &node : query_nodes) {
		P3D p = node->getCoordinates(y);
		double d = ray.getDistanceToPoint(p);
		bool better_candidate = (d < min_d);
		if (d < min_d) {
			min_d = d;
			min_node = node;
		}
	}

	if (min_d < POINT_THRESH) {
		return min_node;
	} else {
		return nullptr;
	}
}
 
void Poser::toggle_node(Node *node, const P3D &target) {
	if (contains(active_nodes, node)) {
		remove_first_instance(active_nodes, node);
		node->setTargetPosition(node->getUndeformedPosition());
	} else {
		active_nodes.push_back(node);
		node->setTargetPosition(target);
	}
	// --
	ik->toggle_Z(node);
}

void Poser::move_node_target(Node *node, const P3D &target) { 
	selected_node->setTargetPosition(target);
}

bool Poser::mouse_move_(double xPos, double yPos) {
	if (LEFT_CLICKED) {
		if (selected_node != nullptr) {
			move_node_target(selected_node, get_xy0(xPos, yPos));
			return true;
		}
	}
	return false;
}

bool Poser::mouse_button_(int button, int action, int mods, double xPos, double yPos) {
	if (action == GLFW_PRESS) {
		if (button == GLFW_MOUSE_BUTTON_LEFT) {
			LEFT_CLICKED = true;
			// --
			auto node = get_closest_node(xPos, yPos, mesh->x_prime, active_nodes); 
			if (node != nullptr) {
				selected_node = node;
			}
		} else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
			RIGHT_CLICKED = true;
			// --
			P3D target;
			{
				Ray ray = InteractiveWidget::getRayFromScreenCoords(xPos, yPos);
				Plane Pi = Plane(P3D(), V3D(0., 0., 1.));
				ray.getDistanceToPlane(Pi, &target);
			}
			// --
			auto node = get_closest_node(xPos, yPos, mesh->x_prime, active_nodes); 
			if (node != nullptr) {
				toggle_node(node, target);
			} else {
				auto node2 = get_closest_node(xPos, yPos, mesh->x, active_nodes); 
				if (node2 != nullptr) {
					toggle_node(node2, target);
				} else {
					auto node3 = get_closest_node(xPos, yPos, mesh->x, mesh->nodes); 
					if (node3 != nullptr) {
						toggle_node(node3, target);
					}
				}
			}
		}
	} else if (action == GLFW_RELEASE) {
		LEFT_CLICKED = false;
		RIGHT_CLICKED = false;
		selected_node = nullptr;
	}
	return false;
}
 
