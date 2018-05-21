#include "SimulationWindow.h"
#include <GUILib/GLTrackingCamera.h>
SimulationWindow::SimulationWindow(int x, int y, int w, int h, Paper3DApp *glApp) : GLWindow3D(x, y, w, h) {
	paperApp = glApp;

	selectedNodeID = -1;

//	dynamic_cast<GLTrackingCamera*>(this->camera)->rotAboutRightAxis = 0.75;
//	dynamic_cast<GLTrackingCamera*>(this->camera)->rotAboutUpAxis = 0.95;
	dynamic_cast<GLTrackingCamera*>(this->camera)->camDistance = -4.5;
}

SimulationWindow::~SimulationWindow(){
}

bool SimulationWindow::onMouseMoveEvent(double xPos, double yPos) {
	pushViewportTransformation();
	lastClickedRay = getRayFromScreenCoords(xPos, yPos);
	if (GlobalMouseState::dragging) {
		if (selectedNodeID >= 0) {
			P3D p;
			P3D selectedNodePos = paperApp->getNodePos(selectedNodeID);
			V3D planeNormal = lastClickedRay.direction;
			lastClickedRay.getDistanceToPlane(Plane(selectedNodePos, planeNormal), &p);
			paperApp->simMesh->setPinnedNode(selectedNodeID, p);
			return true;
		}
	}
	else {
		selectedNodeID = paperApp->simMesh->getSelectedNodeID(lastClickedRay);
	}
	popViewportTransformation();
	return camera->onMouseMoveEvent(xPos, yPos);
}

void SimulationWindow::drawScene() {
	pushViewportTransformation();

	paperApp->simMesh->drawSimulationMesh();

	popViewportTransformation();
}

void SimulationWindow::drawAuxiliarySceneInfo() {
}
