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
	pushViewportTransformation();
	lastClickedRay = getRayFromScreenCoords(xPos, yPos);
	if (GlobalMouseState::dragging) {
		if (selectedNodeID >= 0) {
			P3D p;
			lastClickedRay.getDistanceToPlane(draggingPlane, &p);
			paperApp->simMesh->setPinnedNode(selectedNodeID, p);
			popViewportTransformation();
			return true;
		}
	}
	popViewportTransformation();
	return camera->onMouseMoveEvent(xPos, yPos);
}

bool SimulationWindow::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
	pushViewportTransformation();
	if (paperApp->getMouseMode() == mouse_drag && action == GLFW_PRESS) {
		lastClickedRay = getRayFromScreenCoords(xPos, yPos);
		selectedNodeID = paperApp->simMesh->getSelectedNodeID(lastClickedRay);
		if (selectedNodeID != -1) {
			draggingPlane = Plane(paperApp->getNodePos(selectedNodeID), - lastClickedRay.direction);
		}
		popViewportTransformation();
		return true;
	}
	else if (paperApp->getMouseMode() == mouse_drag && action == GLFW_RELEASE) {
		if (selectedNodeID != -1) {
			if(!(mods & GLFW_MOD_CONTROL))
				paperApp->simMesh->unpinNode(selectedNodeID);
			selectedNodeID = -1;
		}
		popViewportTransformation();
		return true;
	}
	popViewportTransformation();
	return GLWindow3D::onMouseButtonEvent(button, action, mods, xPos, yPos);
}

void SimulationWindow::drawScene() {
	preDraw();

	paperApp->simMesh->drawSimulationMesh();
	drawBorders();

	postDraw();
}

void SimulationWindow::drawAuxiliarySceneInfo() {
}
