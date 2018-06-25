#include "SimulationWindow.h"
#include <GUILib/GLTrackingCamera.h>
SimulationWindow::SimulationWindow(int x, int y, int w, int h, Paper3DApp *glApp) : GLWindow3D(x, y, w, h) {
	paperApp = glApp;

	selectedNodeID = -1;

//	dynamic_cast<GLTrackingCamera*>(this->camera)->rotAboutRightAxis = 0.75;
//	dynamic_cast<GLTrackingCamera*>(this->camera)->rotAboutUpAxis = 0.95;
	dynamic_cast<GLTrackingCamera*>(this->camera)->camDistance = -4.0;
}

SimulationWindow::~SimulationWindow(){
}

bool SimulationWindow::onMouseMoveEvent(double xPos, double yPos) {
	bool do_alter_camera = true;
	pushViewportTransformation();
	lastClickedRay = getRayFromScreenCoords(xPos, yPos);
	MouseMode mode = paperApp->getMouseMode();
	if (mode == mouse_drag && GlobalMouseState::dragging) {
		if (selectedNodeID >= 0) {
			P3D p;
			lastClickedRay.getDistanceToPlane(draggingPlane, &p);
			paperApp->simMesh->setPinnedNode(selectedNodeID, p);
			popViewportTransformation();
			do_alter_camera = false;
		}
	}
	else if (mode == mouse_cut && GlobalMouseState::dragging) {
		int n = paperApp->simMesh->getSelectedNodeID(lastClickedRay);
		bool is_in_path = false;
		for (uint i = 0; i < cutPath.size(); ++i)
			if (cutPath[i] == n)
				is_in_path = true;
		if (n != -1 && !is_in_path && cutPath.size() > 0) {
			int last = cutPath[cutPath.size() - 1];
			bool is_adjacent = paperApp->acessMesh()->areNodesAdjacent(last, n);
			if (is_adjacent)
				cutPath.push_back(n);
		}
		do_alter_camera = false;
	}
	popViewportTransformation();
	if (do_alter_camera)
		return camera->onMouseMoveEvent(xPos, yPos);
	else
		return true;
}

bool SimulationWindow::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
	pushViewportTransformation();
	MouseMode mode = paperApp->getMouseMode();
	if (mode == mouse_drag && action == GLFW_PRESS) {
		lastClickedRay = getRayFromScreenCoords(xPos, yPos);
		selectedNodeID = paperApp->simMesh->getSelectedNodeID(lastClickedRay);
		if (selectedNodeID != -1) {
			draggingPlane = Plane(paperApp->getNodePos(selectedNodeID), - lastClickedRay.direction);
		}
		popViewportTransformation();
		return true;
	}
	else if (mode == mouse_drag && action == GLFW_RELEASE) {
		if (selectedNodeID != -1) {
			if(!(mods & GLFW_MOD_CONTROL))
				paperApp->simMesh->unpinNode(selectedNodeID);
			selectedNodeID = -1;
		}
		popViewportTransformation();
		return true;
	}
	else if (mode == mouse_cut && action == GLFW_PRESS) {
		cutPath.clear();
		lastClickedRay = getRayFromScreenCoords(xPos, yPos);
		selectedNodeID = paperApp->simMesh->getSelectedNodeID(lastClickedRay);
		if (selectedNodeID != -1)
			cutPath.push_back(selectedNodeID);
	}
	else if (mode == mouse_cut && action == GLFW_RELEASE) {
		paperApp->acessMesh()->makeCut(cutPath);
		cutPath.clear();
	}
	popViewportTransformation();
	return GLWindow3D::onMouseButtonEvent(button, action, mods, xPos, yPos);
}

void SimulationWindow::drawScene() {
	preDraw();

	paperApp->simMesh->drawSimulationMesh();
	drawBorders();

	if (paperApp->getMouseMode() == mouse_cut && cutPath.size() > 1) {
		P3D cPos = camera->getCameraPosition();
		glColor3d(1, 1, 0);
		glBegin(GL_LINE_STRIP);
		for (uint i = 0; i < cutPath.size(); ++i) {
			P3D nPos = paperApp->getNodePos(cutPath[i]);
			P3D p = cPos * 0.95 + nPos * 0.05;
			glVertex3d(p[0], p[1], p[2]);
		}
		glEnd();
	}

	postDraw();
}

void SimulationWindow::drawAuxiliarySceneInfo() {
}
