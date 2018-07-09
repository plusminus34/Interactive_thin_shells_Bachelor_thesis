#pragma warning(disable : 4996)

#include "DesignWindow.h"


DesignWindow::DesignWindow(int x, int y, int w, int h, GLApplication* theApp) : GLWindow3D(x, y, w, h) {
	this->theApp = theApp;

	dynamic_cast<GLTrackingCamera*>(this->camera)->rotAboutRightAxis = 0.25;
	dynamic_cast<GLTrackingCamera*>(this->camera)->rotAboutUpAxis = 0.95;
	dynamic_cast<GLTrackingCamera*>(this->camera)->camDistance = -1.5;

	showGroundPlane = false;
	showReflections = true;
}

void DesignWindow::addMenuItems() {
	{
		auto tmpVar = theApp->mainMenu->addVariable("body length", bodyLength);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.05);
	}
}



void DesignWindow::drawScene() {
	glColor3d(1, 1, 1);
	glEnable(GL_LIGHTING);

	/* draw the abstract view of the robot*/

	// body

	//assume the entire robot is centered at (0,0,0)
	drawCylinder(P3D(-bodyLength / 2, 0, 0), P3D(bodyLength / 2, 0, 0), 0.01);

}

void DesignWindow::drawAuxiliarySceneInfo(){
	glClear(GL_DEPTH_BUFFER_BIT);

	preDraw();

	postDraw();

	glClear(GL_DEPTH_BUFFER_BIT);

}

//any time a physical key is pressed, this event will trigger. Useful for reading off special keys...
bool DesignWindow::onKeyEvent(int key, int action, int mods) {
	return GLWindow3D::onKeyEvent(key, action, mods);
}

bool DesignWindow::onMouseMoveEvent(double xPos, double yPos){

	return GLWindow3D::onMouseMoveEvent(xPos, yPos);
}

bool DesignWindow::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos){
	return GLWindow3D::onMouseButtonEvent(button, action, mods, xPos, yPos);
}

void DesignWindow::setViewportParameters(int posX, int posY, int sizeX, int sizeY){
	GLWindow3D::setViewportParameters(posX, posY, sizeX, sizeY);
}

