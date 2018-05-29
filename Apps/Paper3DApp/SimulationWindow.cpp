#include "SimulationWindow.h"
#include <GUILib/GLTrackingCamera.h>
SimulationWindow::SimulationWindow(int x, int y, int w, int h, Paper3DApp *glApp) : GLWindow3D(x, y, w, h) {
	paperApp = glApp;

	dragging = false;
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
	if (dragging) {
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
	//action=1: button down, 0: button up
	printf("click\n");
	pushViewportTransformation();
	if (paperApp->getMouseMode() == mouse_drag && action == 1) {// action==1 means the mouse button is being pressed
		printf("letsstart\n");
		lastClickedRay = getRayFromScreenCoords(xPos, yPos);
		selectedNodeID = paperApp->simMesh->getSelectedNodeID(lastClickedRay);
		if (selectedNodeID != -1) {
			dragging = true;
			draggingPlane = Plane(paperApp->getNodePos(selectedNodeID), - lastClickedRay.direction);
		}
		popViewportTransformation();
		return true;
	}
	else if (paperApp->getMouseMode() == mouse_drag && action == 0) {// action==1 means the mouse button is being released
		printf("let the shackles be released\n");
		if (selectedNodeID != -1) {
			paperApp->simMesh->unpinNode(selectedNodeID);
			dragging = false;
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
