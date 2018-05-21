#include "ShapeWindow.h"
#include <GUILib/GLTrackingCamera.h>

ShapeWindow::ShapeWindow(int x, int y, int w, int h, Paper3DApp *glApp) : GLWindow2D(x, y, w, h) {
	paperApp = glApp;
}

ShapeWindow::~ShapeWindow(){
}

void ShapeWindow::drawScene() {
	pushViewportTransformation();
	
	paperApp->simMesh->drawRestConfiguration();

//	popViewportTransformation();
}

void ShapeWindow::drawAuxiliarySceneInfo() {
}