#include "ShapeWindow.h"
#include "BendingEdge.h"
#include "Paper3DMesh.h"
#include "Pin.h"
#include <GUILib/GLTrackingCamera.h>

ShapeWindow::ShapeWindow(int x, int y, int w, int h, Paper3DApp *glApp) : GLWindow3D(x, y, w, h) {
	paperApp = glApp;

	dynamic_cast<GLTrackingCamera*>(this->camera)->camDistance = -4.0;
}

ShapeWindow::~ShapeWindow(){
}

void ShapeWindow::getSaveData(int &grid_width, int &grid_height, double &cell_size, MatrixNxM &pins) {
	grid_width = dim_x;
	grid_height = dim_y;
	cell_size = h;
	int num_pins = pinHandles.size() / 2;
	pins.resize(num_pins, 8);
	for (int i = 0; i < num_pins; ++i) {
		// convert flip to double: -1.0 (< 0.0) for false, 1.0 (>= 0.0) for true
		double f0 = -1.0, f1 = -1.0;
		if (pinHandles[2 * i]->flipped) f0 = 1.0;
		if (pinHandles[2 * i + 1]->flipped) f1 = 1.0;
		pins.row(i) << pinHandles[2 * i]->x, pinHandles[2 * i]->y, pinHandles[2 * i]->angle, f0,
			pinHandles[2 * i + 1]->x, pinHandles[2 * i + 1]->y, pinHandles[2 * i + 1]->angle, f1;
	}
}

void ShapeWindow::applyLoadData(int &grid_width, int &grid_height, double &cell_size, MatrixNxM &pins) {
	setGridDimensions(grid_width, grid_height, cell_size);
	for (uint i = 0; i < pinHandles.size(); ++i)
		delete pinHandles[i];
	pinHandles.clear();
	next_pin_id = 0;

	for (int i = 0; i < pins.rows(); ++i) {
		int new_id = next_pin_id++;

		PinHandle* new_handle = new PinHandle;
		new_handle->pin_id = new_id;
		new_handle->index = 0;
		new_handle->x = pins(i, 0);
		new_handle->y = pins(i, 1);
		new_handle->angle = pins(i, 2);
		new_handle->flipped = (pins(i, 3) >= 0.0);
		pinHandles.push_back(new_handle);

		new_handle = new PinHandle;
		new_handle->pin_id = new_id;
		new_handle->index = 1;
		new_handle->x = pins(i, 4);
		new_handle->y = pins(i, 5);
		new_handle->angle = pins(i, 6);
		new_handle->flipped = (pins(i, 7) >= 0.0);
		pinHandles.push_back(new_handle);

		Pin* toAdd = createPinFromHandles(2 * i, 2 * i + 1);
		if (toAdd != NULL)
			paperApp->addMeshElement(toAdd);
	}
}

void ShapeWindow::setGridDimensions(int dim_x, int dim_y, double h) {
	this->dim_x = dim_x;
	this->dim_y = dim_y;
	this->h = h;
}

bool ShapeWindow::onMouseMoveEvent(double xPos, double yPos) {
	pushViewportTransformation();
	Ray lastClickedRay = getRayFromScreenCoords(xPos, yPos);
	P3D p;
	lastClickedRay.getDistanceToPlane(Plane(P3D(), V3D(0, 0, 1)), &p);
	if (paperApp->getMouseMode() == mouse_pin_move && GlobalMouseState::dragging && selected_i != -1) {
		pinHandles[selected_i]->x = p[0];
		pinHandles[selected_i]->y = p[1];
	} else if (paperApp->getMouseMode() == mouse_pin_rotate && GlobalMouseState::dragging && selected_i != -1) {
		double angle_current = atan2(p[1] - pinHandles[selected_i]->y, p[0] - pinHandles[selected_i]->x);
		double angle_start = atan2(yDrag - pinHandles[selected_i]->y, xDrag - pinHandles[selected_i]->x);
		pinHandles[selected_i]->angle = initialAngle + angle_current - angle_start;
	}
	else if (paperApp->getMouseMode() == mouse_cut && GlobalMouseState::dragging) {
		int n = findNodeClosestTo(p[0], p[1]);
		// each node may only show up once
		bool is_in_path = false;
		for (uint i = 0; i < cutPath.size(); ++i)
			is_in_path = (is_in_path || cutPath[i] == n);
		// checking only proximity often leads to unwanted results -> extra condition
		P3D pos_before = paperApp->getNodeRestPos(cutPath[cutPath.size() - 1]);
		P3D pos_n = paperApp->getNodeRestPos(n);
		V3D dir_n = (pos_n - pos_before).normalized();
		V3D dir_mouse = (P3D(p[0], p[1], 0) - pos_before).normalized();
		bool direction_matches = (dir_mouse.dot(dir_n) > 0.95);
		if (direction_matches && !is_in_path && cutPath.size() > 0) {
			int last = cutPath[cutPath.size() - 1];
			bool is_adjacent = paperApp->acessMesh()->areNodesAdjacent(last, n);
			if (is_adjacent)
				cutPath.push_back(n);
		}
	}
	else if (paperApp->getMouseMode() == mouse_drag && GlobalMouseState::dragging) {
		P3D new_target(camera_x -(xPos - xDrag) / viewportWidth, camera_y + (yPos - yDrag) / viewportHeight, 0);
		camera->setCameraTarget(new_target);
	}

	popViewportTransformation();
	return true;
}

bool ShapeWindow::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
	pushViewportTransformation();
	MouseMode mode = paperApp->getMouseMode();
	Ray clickedRay = getRayFromScreenCoords(xPos, yPos);
	P3D p;
	clickedRay.getDistanceToPlane(Plane(P3D(), V3D(0, 0, 1)), &p);
	if (mode == mouse_pin_create  && action == GLFW_PRESS) {
		if (p[0] >= 0 && p[0] <= h * dim_x && p[1] >= 0 && p[1] <= h * dim_y) {
			if (!first_point_set) {
				xPin = p[0];
				yPin = p[1];
				first_point_set = true;
			}
			else {
				if ((p[0] - xPin)*(p[0] - xPin) + (p[1] - yPin)*(p[1] - yPin) < 4*h*h) {// TODO: better distance measure
					printf("Error: Pin endpoints are too close\n");
				}
				else {
					Vector2d end0(xPin, yPin);
					Vector2d end1(p[0], p[1]);
					Vector2d dir = (end1 - end0).normalized();

					int new_id = next_pin_id++;

					PinHandle* new_handle = new PinHandle;
					new_handle->pin_id = new_id;
					new_handle->index = 0;
					new_handle->x = end0[0];
					new_handle->y = end0[1];
					new_handle->angle = atan2(dir[1], dir[0]);
					new_handle->flipped = true;
					pinHandles.push_back(new_handle);

					new_handle = new PinHandle;
					new_handle->pin_id = new_id;
					new_handle->index = 1;
					new_handle->x = end1[0];
					new_handle->y = end1[1];
					new_handle->angle = atan2(-dir[1], -dir[0]);
					new_handle->flipped = false;
					pinHandles.push_back(new_handle);

					Pin* toAdd = createPinFromHandles(pinHandles.size() - 2, pinHandles.size() - 1);
					if(toAdd != NULL)
						paperApp->addMeshElement(toAdd);

					first_point_set = false;
				}
			}
		}
	}
	else if (mode == mouse_pin_create && action == GLFW_RELEASE) {}
	else if (mode == mouse_pin_move && action == GLFW_PRESS) {
		xDrag = p[0];
		yDrag = p[1];
		selected_i = findPinHandleClosestTo(p[0], p[1]);
	}
	else if (mode == mouse_pin_move && action == GLFW_RELEASE) {
		if (selected_i != -1) {
			int i_handle_a = selected_i - (selected_i % 2);
			int i_handle_b = i_handle_a + 1;
			Pin* toAdd = createPinFromHandles(i_handle_a, i_handle_b);
			if (toAdd != NULL) {
				Paper3DMesh* paperMesh = dynamic_cast<Paper3DMesh*>(paperApp->acessMesh());
				paperMesh->replacePin(toAdd->getID(), toAdd);
			}
			else {
				pinHandles[selected_i]->x = xDrag;
				pinHandles[selected_i]->y = yDrag;
			}
			selected_i = -1;
		}
	}
	else if (mode == mouse_pin_rotate && action == GLFW_PRESS) {
		xDrag = p[0];
		yDrag = p[1];
		selected_i = findPinHandleClosestTo(p[0], p[1]);
		if (selected_i != -1) {
			initialAngle = pinHandles[selected_i]->angle;
		}
	}
	else if (mode == mouse_pin_rotate && action == GLFW_RELEASE) {
		if (selected_i != -1) {
			int i_handle_a = selected_i - (selected_i % 2);
			int i_handle_b = i_handle_a + 1;
			Pin* toAdd = createPinFromHandles(i_handle_a, i_handle_b);
			if (toAdd != NULL) {
				Paper3DMesh* paperMesh = dynamic_cast<Paper3DMesh*>(paperApp->acessMesh());
				paperMesh->replacePin(toAdd->getID(), toAdd);
			}
			else {
				pinHandles[selected_i]->angle = initialAngle;
			}
			selected_i = -1;
		}
	}
	else if (mode == mouse_pin_flip && action == GLFW_PRESS) {
		int handle_i = findPinHandleClosestTo(p[0], p[1]);
		if (handle_i != -1) {
			pinHandles[handle_i]->flipped = !(pinHandles[handle_i]->flipped);
			int i_handle_a = handle_i - (handle_i % 2);
			int i_handle_b = i_handle_a + 1;
			Pin* toAdd = createPinFromHandles(i_handle_a, i_handle_b);
			if (toAdd != NULL) {
				Paper3DMesh* paperMesh = dynamic_cast<Paper3DMesh*>(paperApp->acessMesh());
				paperMesh->replacePin(toAdd->getID(), toAdd);
			}
			else {
				pinHandles[handle_i]->flipped = !(pinHandles[handle_i]->flipped);
			}
		}
	}
	else if (mode == mouse_pin_flip && action == GLFW_RELEASE) {}
	else if (mode == mouse_pin_delete && action == GLFW_PRESS) {
		int handle_i = findPinHandleClosestTo(p[0], p[1]);
		if (handle_i != -1) {
			int i_handle_a = handle_i - (handle_i % 2);
			int i_handle_b = i_handle_a + 1;
			paperApp->acessMesh()->deletePin(pinHandles[handle_i]->pin_id);
			delete pinHandles[i_handle_a];
			delete pinHandles[i_handle_b];
			pinHandles[i_handle_a] = pinHandles[pinHandles.size() - 2];
			pinHandles[i_handle_b] = pinHandles[pinHandles.size() - 1];
			pinHandles.pop_back(); pinHandles.pop_back();
		}

	}
	else if (mode == mouse_pin_delete && action == GLFW_RELEASE) {}
	else if (mode == mouse_cut && action == GLFW_PRESS) {
		cutPath.clear();
		cutPath.push_back(findNodeClosestTo(p[0], p[1]));
	}
	else if (mode == mouse_cut && action == GLFW_RELEASE) {
		paperApp->acessMesh()->makeCut(cutPath);
		cutPath.clear();
	}
	else if (mode == mouse_drag && action == GLFW_PRESS){
		xDrag = xPos;
		yDrag = yPos;
	}
	else if (mode == mouse_drag && action == GLFW_RELEASE) {
		P3D cameraPos = camera->getCameraPosition();
		camera_x = cameraPos[0];
		camera_y = cameraPos[1];
	}
	popViewportTransformation();
	return GLWindow3D::onMouseButtonEvent(button, action, mods, xPos, yPos);
}

void ShapeWindow::drawScene() {
	preDraw();
	
	paperApp->simMesh->drawRestConfiguration();

	if(display_bending){
		// colour edges according to their current energy
		Paper3DMesh* mesh = paperApp->acessMesh();
		double max_energy_now = 0.0;
		for (uint i = 0; i < mesh->elements.size(); ++i) {
			if (BendingEdge* e = dynamic_cast<BendingEdge*>(mesh->elements[i])) {
				double energy = e->getEnergy(mesh->x, mesh->X);
				max_energy_now = std::max(max_energy_now, e_bend_max);
				P3D p0 = e->n[0]->getUndeformedPosition();
				P3D p1 = e->n[1]->getUndeformedPosition();
				double c = std::min(1.0, energy / e_bend_max);
				glColor3d(c, 0.5 + 0.25 * c, 0.75 - 0.25 * c);
				glBegin(GL_LINES);
				glVertex3d(p0[0], p0[1], 0.001);
				glVertex3d(p1[0], p1[1], 0.001);
				glEnd();
			}
		}
		// adapt to current maximum bending energy
		e_bend_max = std::max(max_energy_now, 0.002);
	}

	if (first_point_set) {
		//Draw cross at pin start point
		glColor3d(0, 1, 1);
		glBegin(GL_LINES);
		glVertex3d(xPin-0.03, yPin-0.03, 0.01);
		glVertex3d(xPin+0.03, yPin+0.03, 0.01);
		glVertex3d(xPin + 0.03, yPin - 0.03, 0.01);
		glVertex3d(xPin - 0.03, yPin + 0.03, 0.01);
		glEnd();
	}

	if (paperApp->getMouseMode() == mouse_pin_move && selected_i != -1 && GlobalMouseState::dragging) {
		int other_i = selected_i;
		if (selected_i % 2 == 0) {
			other_i += 1;
		}
		else {
			other_i -= 1;
		}
		glColor3d(0, 1, 1);
		glBegin(GL_LINES);
		glVertex3d(pinHandles[selected_i]->x, pinHandles[selected_i]->y, 0.01);
		glVertex3d(pinHandles[other_i]->x, pinHandles[other_i]->y, 0.01);
		glEnd();
	}

	if (paperApp->getMouseMode() == mouse_pin_rotate && selected_i != -1 && GlobalMouseState::dragging) {
		Vector2d corners[3];
		for (int i = 0; i < 3; ++i)corners[i] = pinHandles[selected_i]->getPoint(i);
		glColor3d(0, 1, 1);
		glBegin(GL_LINE_LOOP);
		for (int i = 0; i < 3; ++i)
			glVertex3d(corners[i][0], corners[i][1], 0.01);
		glEnd();
	}
	else if (paperApp->getMouseMode() == mouse_cut && GlobalMouseState::dragging && cutPath.size() > 1) {
		glColor3d(1, 1, 0);
		glBegin(GL_LINE_STRIP);
		for (uint i = 0; i < cutPath.size(); ++i) {
			P3D p = paperApp->getNodeRestPos(cutPath[i]);
			glVertex3d(p[0], p[1], 0.011);
		}
		glEnd();
	}

	drawBorders();

	postDraw();
}

void ShapeWindow::drawAuxiliarySceneInfo() {
}

int ShapeWindow::findNodeClosestTo(double x, double y) {
	// map (x,y) to triangle
	double x_c = std::max(0.0, std::min(x, h * (dim_x - 2)));
	double y_c = std::max(0.0, std::min(y, h * (dim_y - 2)));
	int a = (int)floor(x_c / h);
	int b = (int)floor(y_c / h);
	int upper = 0;
	if (x_c - a * h < y_c - b * h) upper = 1;
	int t = 2 * (a*(dim_y - 1) + b) + upper;
	// compute distance to triangle corners
	int c[3];
	double d[3];
	paperApp->acessMesh()->cornersOfTriangle(t, c[0], c[1], c[2]);
	for (int i = 0; i < 3; ++i) {
		P3D p = paperApp->getNodeRestPos(c[i]);
		d[i] = (p[0] - x)*(p[0] - x) + (p[1] - y)*(p[1] - y);
	}
	double d_min = std::min(d[0], std::min(d[1], d[2]));
	// closest triangle corner is the closest node
	for (int i = 0; i < 3; ++i) {
		if (d[i] == d_min)
			return c[i];
	}
	return 0;
}

int ShapeWindow::findPinHandleClosestTo(double x, double y, double max_distance) {
	double smallest_distance = HUGE_VAL;
	int res = -1;
	for (uint i = 0; i < pinHandles.size(); ++i) {
		double distance = (pinHandles[i]->x - x) * (pinHandles[i]->x - x) + (pinHandles[i]->y - y) * (pinHandles[i]->y - y);
		if (distance < smallest_distance) {
			smallest_distance = distance;
			res = i;
		}
	}
	if (smallest_distance > max_distance * max_distance) res = -1;
	return res;
}

BarycentricZeroLengthSpring* ShapeWindow::createZeroLengthSpring(double x0, double y0, double x1, double y1) {
	SimulationMesh* simMesh = paperApp->acessMesh();

	double x[2], y[2];
	x[0] = x0; y[0] = y0;
	x[1] = x1; y[1] = y1;

	int ni[2][3];
	for (int i = 0; i < 6; ++i)
		ni[i / 3][i % 3] = -1;
	//figure out the triangle corners ni
	for (int i = 0; i < 2; ++i) {
		// map (x,y) to triangle
		double x_c = std::max(0.0, std::min(x[i], h * (dim_x - 2)));
		double y_c = std::max(0.0, std::min(y[i], h * (dim_y - 2)));
		int a = (int)floor(x_c / h);
		int b = (int)floor(y_c / h);
		int upper = 0;
		if (x_c - a * h < y_c - b * h) upper = 1;
		int t = 2 * (a*(dim_y - 1) + b) + upper;
		// compute distance to triangle corners
		paperApp->acessMesh()->cornersOfTriangle(t, ni[i][0], ni[i][1], ni[i][2]);
	}

	Node* n[6];
	for (int i = 0; i < 2; ++i)
		for (int j = 0; j < 3; ++j) {
			if (ni[i][j] == -1) {
				return NULL;
			}
			else {
				n[3 * i + j] = paperApp->acessNode(ni[i][j]);
			}
		}

	//Compute weights
	Vector3d w[2];
	for (int i = 0; i < 2; ++i) {
		double nx[3], ny[3];
		for (int j = 0; j < 3; ++j) {
			nx[j] = (ni[i][j] / dim_y) * h;
			ny[j] = (ni[i][j] % dim_y) * h;
		}
		Matrix3x3 A;
		Vector3d b;
		A << nx[0], nx[1], nx[2],
			ny[0], ny[1], ny[2],
			1.0, 1.0, 1.0;
		b << x[i], y[i], 1.0;
		w[i] = A.inverse() * b;// TODO: don't compute the inverse!

		for (int j = 0; j < 3; ++j)
			if (w[i][j] < 0.0) return NULL;

		double outx = nx[0] * w[i][0] + nx[1] * w[i][1] + nx[2] * w[i][2];
		double outy = ny[0] * w[i][0] + ny[1] * w[i][1] + ny[2] * w[i][2];
	}

	BarycentricZeroLengthSpring* res = new BarycentricZeroLengthSpring(simMesh, n[0], n[1], n[2], n[3], n[4], n[5]);
	for (int i = 0; i < 2; ++i)
		res->setWeights(i, w[i][0], w[i][1], w[i][2]);

	return res;
}

Pin* ShapeWindow::createPinFromHandles(uint h0, uint h1) {

	if (h0 >= pinHandles.size() || h1 >= pinHandles.size()) return NULL;
	if (pinHandles[h0]->pin_id != pinHandles[h1]->pin_id) return NULL;
	if (pinHandles[h0]->index != 0) std::swap(h0, h1);

	BarycentricZeroLengthSpring* ols[3];
	Vector2d p0, p1;
	for (int i = 0; i < 3; ++i) {
		p0 = pinHandles[h0]->getPoint(i);
		p1 = pinHandles[h1]->getPoint(i);
		ols[i] = createZeroLengthSpring(p0[0], p0[1], p1[0], p1[1]);
		if (ols[i] == NULL) {
			for (int j = 0; j < i; ++j) delete ols[j];
			return NULL;
		}
	}
	Pin* res = new Pin(paperApp->acessMesh(), pinHandles[h0]->pin_id, ols[0], ols[1], ols[2]);
	return res;
}