#pragma warning(disable : 4996)

#include <GUILib/GLApplication.h>

#include <RobotDesignerLib/ModuleDisplayWindow.h>
#include <Utils/Utils.h>
#include <GUILib/GLUtils.h>

#include <GUILib/GLContentManager.h>
#include <GUILib/GLMesh.h>
#include <GUILib/GLCamera.h>

/**
3D Window with a generic object type
*/
ModuleDisplayWindow::ModuleDisplayWindow(RMC* rmc, int posX, int posY, int sizeX, int sizeY) : GLWindow3D(posX, posY, sizeX, sizeY) {
	delete camera;
	camera = new GLTrackingCamera();
	((GLTrackingCamera*)camera)->camDistance = -0.1;
	//((GLTrackingCamera*)camera)->rotAboutUpAxis = PI / 4;

	this->rmc = rmc;
}

ModuleDisplayWindow::ModuleDisplayWindow(RMC* rmc) : GLWindow3D() {
	delete camera;
	camera = new GLTrackingCamera();
	((GLTrackingCamera*)camera)->camDistance = -0.1;
	((GLTrackingCamera*)camera)->rotAboutUpAxis = 0.5;
	((GLTrackingCamera*)camera)->rotAboutRightAxis = 0.5;

	this->rmc = rmc;
}

ModuleDisplayWindow::ModuleDisplayWindow() : GLWindow3D() {
	delete camera;
	camera = new GLTrackingCamera();
	((GLTrackingCamera*)camera)->camDistance = -0.1;
	//((GLTrackingCamera*)camera)->rotAboutUpAxis = PI / 4;

}

void ModuleDisplayWindow::drawScene() {
	if (rmc) {
		rmc->draw(SHOW_MESH | SHOW_PINS);
	}
}
