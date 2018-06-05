#include "ShapeWindow.h"
#include "Paper3DMesh.h"
#include "Pin.h"
#include <GUILib/GLTrackingCamera.h>

ShapeWindow::ShapeWindow(int x, int y, int w, int h, Paper3DApp *glApp) : GLWindow3D(x, y, w, h) {
	paperApp = glApp;

	dynamic_cast<GLTrackingCamera*>(this->camera)->camDistance = -4.0;

	dragging = false;
}

ShapeWindow::~ShapeWindow(){
}

void ShapeWindow::setGridDimensions(int dim_x, int dim_y, double h) {
	this->dim_x = dim_x;
	this->dim_y = dim_y;
	this->h = h;

	if (true) {//starting pin TODO: move elsewhere
		int new_id = next_pin_id++;

		PinHandle* new_handle = new PinHandle;
		new_handle->pin_id = new_id;
		new_handle->index = 0;
		new_handle->x = 0.11;
		new_handle->y = 0.5;
		new_handle->angle = PI*7/4;
		new_handle->flipped = true;
		pinHandles.push_back(new_handle);

		new_handle = new PinHandle;
		new_handle->pin_id = new_id;
		new_handle->index = 1;
		new_handle->x = 0.95;
		new_handle->y = 0.2;
		new_handle->angle = PI*3/4;
		new_handle->flipped = false;
		pinHandles.push_back(new_handle);

		Pin* toAdd = createPinFromHandles(pinHandles.size() - 2, pinHandles.size() - 1);
		paperApp->addMeshElement(toAdd);
	}
}

bool ShapeWindow::onMouseMoveEvent(double xPos, double yPos) {
	pushViewportTransformation();
	Ray lastClickedRay = getRayFromScreenCoords(xPos, yPos);
	P3D p;
	lastClickedRay.getDistanceToPlane(Plane(P3D(), V3D(0, 0, 1)), &p);
	if (paperApp->getMouseMode() == mouse_pin_move && dragging && selected_i != -1) {
		pinHandles[selected_i]->x = p[0];
		pinHandles[selected_i]->y = p[1];
	} else if (paperApp->getMouseMode() == mouse_pin_rotate && dragging && selected_i != -1) {
		//TODO update angle
		double angle_current = atan2(p[1] - pinHandles[selected_i]->y, p[0] - pinHandles[selected_i]->x);
		double angle_start = atan2(yDrag - pinHandles[selected_i]->y, xDrag - pinHandles[selected_i]->x);
		pinHandles[selected_i]->angle = initialAngle + angle_current - angle_start;
	}
	/*TODO camera
	if (GlobalMouseState::dragging) {
		if (dragging) {
			//TODO: move camera instead of target
			P3D new_target(-(xPos - xDrag) / viewportWidth, (yPos - yDrag) / viewportHeight, 0);
			camera->setCameraTarget(new_target);
		}
	}
	else {
		dragging = true;
		xDrag = xPos; yDrag = yPos;
	}
	*/
	popViewportTransformation();
	return true;
}

bool ShapeWindow::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
	pushViewportTransformation();
	MouseMode mode = paperApp->getMouseMode();
	Ray clickedRay = getRayFromScreenCoords(xPos, yPos);
	P3D p;
	clickedRay.getDistanceToPlane(Plane(P3D(), V3D(0, 0, 1)), &p);
	if (mode == mouse_pin_create  && action == 1) {
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
	else if (mode == mouse_pin_move && action == 1) {
		xDrag = p[0];
		yDrag = p[1];
		selected_i = findPinHandleClosestTo(p[0], p[1]);
		dragging = (selected_i != -1);
	}
	else if (mode == mouse_pin_move && action == 0) {
		dragging = false;
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
	else if (mode == mouse_pin_rotate && action == 1) {
		xDrag = p[0];
		yDrag = p[1];
		selected_i = findPinHandleClosestTo(p[0], p[1]);
		if (selected_i != -1) {
			initialAngle = pinHandles[selected_i]->angle;
			dragging = true;
		}
	}
	else if (mode == mouse_pin_rotate && action == 0) {
		dragging = false;
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
	else if (mode == mouse_pin_flip && action == 1) {
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
	else if (mode == mouse_pin_delete && action == 1) {
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
	popViewportTransformation();
	return GLWindow3D::onMouseButtonEvent(button, action, mods, xPos, yPos);
}

void ShapeWindow::drawScene() {
	preDraw();
	
	paperApp->simMesh->drawRestConfiguration();

	if (first_point_set) {
		//Draw cross at pin start point
		glColor3d(0, 1, 1);
		glBegin(GL_LINES);
		glVertex3d(xPin-0.03, yPin-0.03, 0.01);
		glVertex3d(xPin+0.03, yPin+0.03, 0.01);
		glEnd();
		glBegin(GL_LINES);
		glVertex3d(xPin + 0.03, yPin - 0.03, 0.01);
		glVertex3d(xPin - 0.03, yPin + 0.03, 0.01);
		glEnd();
	}

	if (dragging && paperApp->getMouseMode() == mouse_pin_move && selected_i != -1) {
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

	if (dragging && paperApp->getMouseMode() == mouse_pin_rotate && selected_i != -1) {
		Vector2d corners[3];
		for (int i = 0; i < 3; ++i)corners[i] = pinHandles[selected_i]->getPoint(i);
		glColor3d(0, 1, 1);
		glBegin(GL_LINE_LOOP);
		for (int i = 0; i < 3; ++i)
			glVertex3d(corners[i][0], corners[i][1], 0.01);
		glEnd();
	}

	drawBorders();

	postDraw();
}

void ShapeWindow::drawAuxiliarySceneInfo() {
}

int ShapeWindow::findNodeClosestTo(double x, double y) {
	//currently works only for rectangular mesh
	if (x < 0) x = 0;
	if (x > h * dim_x) x = h * dim_x;
	if (y < 0) y = 0;
	if (y > h * dim_y) y = h * dim_y;
	int a = (int)round(x / h);
	int b = (int)round(y / h);
	return (a*dim_y + b);
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

	int ni[2][3];
	double x[2], y[2], dx[2], dy[2];

	x[0] = x0; y[0] = y0;
	x[1] = x1; y[1] = y1;

	for (int i = 0; i < 2; ++i) {
		ni[i][0] = findNodeClosestTo(x[i], y[i]);
		dx[i] = x[i] - (ni[i][0] / dim_y) * h;
		dy[i] = y[i] - (ni[i][0] % dim_y) * h;
	}

	//figure out the triangle corners (rectangle mesh only)
	for (int i = 0; i < 2; ++i) {
		if (dx[i] < 0 && dy[i] < 0 && dx[i] < dy[i]) {
			ni[i][1] = ni[i][0] - dim_y;
			ni[i][2] = ni[i][0] - dim_y - 1;
		}
		else if (dx[i] < 0 && dy[i] < 0 && dx[i] >= dy[i]) {
			ni[i][1] = ni[i][0] - dim_y - 1;
			ni[i][2] = ni[i][0] - 1;
		}
		else if (dx[i] >= 0 && dy[i] < 0) {
			ni[i][1] = ni[i][0] - 1;
			ni[i][2] = ni[i][0] + dim_y;
		}
		else if (dx[i] >= 0 && dy[i] >= 0 && dx[i] > dy[i]) {
			ni[i][1] = ni[i][0] + dim_y;
			ni[i][2] = ni[i][0] + dim_y + 1;
		}
		else if (dx[i] >= 0 && dy[i] >= 0 && dx[i] <= dy[i]) {
			ni[i][1] = ni[i][0] + dim_y + 1;
			ni[i][2] = ni[i][0] + 1;
		}
		else if (dx[i] < 0 && dy[i] >= 0) {
			ni[i][1] = ni[i][0] + 1;
			ni[i][2] = ni[i][0] - dim_y;
		}
	}

	Node* n[6];
	for (int i = 0; i < 2; ++i)
		for (int j = 0; j < 3; ++j) {
			if (ni[i][j] < 0 || ni[i][j] >= dim_x * dim_y) { return NULL; }
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