#include "ShapeWindow.h"
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

void ShapeWindow::drawScene() {
	preDraw();
	
	paperApp->simMesh->drawRestConfiguration();
	drawBorders();

	postDraw();
}

void ShapeWindow::drawAuxiliarySceneInfo() {
}