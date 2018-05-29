#include "ShapeWindow.h"
#include "Pin.h"
#include <GUILib/GLTrackingCamera.h>

ShapeWindow::ShapeWindow(int x, int y, int w, int h, Paper3DApp *glApp) : GLWindow3D(x, y, w, h) {
	paperApp = glApp;

	dynamic_cast<GLTrackingCamera*>(this->camera)->camDistance = -4.0;

	dragging = false;
}

ShapeWindow::~ShapeWindow(){
}

bool ShapeWindow::onMouseMoveEvent(double xPos, double yPos) {
	pushViewportTransformation();
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
	popViewportTransformation();
	return true;
}

bool ShapeWindow::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
	pushViewportTransformation();
	if (paperApp->getMouseMode() == mouse_pin) {
		Ray clickedRay = getRayFromScreenCoords(xPos, yPos);
		P3D p;
		clickedRay.getDistanceToPlane(Plane(P3D(), V3D(0, 0, 1)), &p);
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
					BarycentricZeroLengthSpring* ols[3];
					Vector2d end0(xPin, yPin);
					Vector2d end1(p[0], p[1]);
					Vector2d dir = (end1 - end0).normalized();
					Matrix2x2 R;
					R << cos(PI*2.0 / 3.0), -sin(PI*2.0 / 3.0), sin(PI*2.0 / 3.0), cos(PI*2.0 / 3.0);
					Vector2d dp[3];
					dp[0] = dir * 0.05;
					dp[1] = R * dp[0];
					dp[2] = R * dp[1];
					Vector2d p0 = end0 - dp[0];
					Vector2d p1 = end1 + dp[0];
					ols[0] = createZeroLengthSpring(p0[0], p0[1], p1[0], p1[1]);
					p0 = end0 - dp[2];
					p1 = end1 + dp[1];
					ols[1] = createZeroLengthSpring(p0[0], p0[1], p1[0], p1[1]);
					p0 = end0 - dp[1];
					p1 = end1 + dp[2];
					ols[2] = createZeroLengthSpring(p0[0], p0[1], p1[0], p1[1]);
					bool addpin = true;
					for (int i = 0; i < 3; ++i)
						if (ols[i] == NULL) {
							addpin = false;
						}
					if (addpin) {
						Pin* toAdd = new Pin(paperApp->acessMesh(), ols[0], ols[1], ols[2]);
						paperApp->addMeshElement(toAdd);
						first_point_set = false;
					}
				}
			}
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

BarycentricZeroLengthSpring* ShapeWindow::createZeroLengthSpring(double x0, double y0, double x1, double y1) {
	SimulationMesh* simMesh = paperApp->acessMesh();

	int ni[2][3];
	double x[2], y[2], dx[2], dy[2];

	x[0] = x0; y[0] = y0;
	x[1] = x1; y[1] = y1;
	printf("to connect: (%f, %f) and (%f, %f)\n", x[0], y[0], x[1], y[1]);

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

		double outx = nx[0] * w[i][0] + nx[1] * w[i][1] + nx[2] * w[i][2];
		double outy = ny[0] * w[i][0] + ny[1] * w[i][1] + ny[2] * w[i][2];
		printf("after weights (%f, %f)\n", outx, outy);
	}

	BarycentricZeroLengthSpring* res = new BarycentricZeroLengthSpring(simMesh, n[0], n[1], n[2], n[3], n[4], n[5]);
	for (int i = 0; i < 2; ++i)
		res->setWeights(i, w[i][0], w[i][1], w[i][2]);

	//printf("Spring point %f %f\n", p[0], p[1]);

	return res;
}