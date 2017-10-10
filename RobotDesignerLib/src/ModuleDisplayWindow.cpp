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
		rmc->draw(SHOW_MESH);
	}
		
}

void ModuleDisplayWindow::setupLights() {

	GLfloat bright[] = { 0.8f, 0.8f, 0.8f, 1.0f };
	GLfloat mediumbright[] = { 0.3f, 0.3f, 0.3f, 1.0f };

	glLightfv(GL_LIGHT1, GL_DIFFUSE, bright);
	glLightfv(GL_LIGHT2, GL_DIFFUSE, mediumbright);
	glLightfv(GL_LIGHT3, GL_DIFFUSE, mediumbright);
	glLightfv(GL_LIGHT4, GL_DIFFUSE, mediumbright);


	GLfloat light0_position[] = { 0.0f, 10000.0f, 10000.0f, 0.0f };
	GLfloat light0_direction[] = { 0.0f, -10000.0f, -10000.0f, 0.0f };

	GLfloat light1_position[] = { 0.0f, 10000.0f, -10000.0f, 0.0f };
	GLfloat light1_direction[] = { 0.0f, -10000.0f, 10000.0f, 0.0f };

	GLfloat light2_position[] = { 0.0f, -10000.0f, 0.0f, 0.0f };
	GLfloat light2_direction[] = { 0.0f, 10000.0f, -0.0f, 0.0f };

	GLfloat light3_position[] = { 10000.0f, -10000.0f, 0.0f, 0.0f };
	GLfloat light3_direction[] = { -10000.0f, 10000.0f, -0.0f, 0.0f };

	GLfloat light4_position[] = { -10000.0f, -10000.0f, 0.0f, 0.0f };
	GLfloat light4_direction[] = { 10000.0f, 10000.0f, -0.0f, 0.0f };


	glLightfv(GL_LIGHT0, GL_POSITION, light0_position);
	glLightfv(GL_LIGHT1, GL_POSITION, light1_position);
	glLightfv(GL_LIGHT2, GL_POSITION, light2_position);
	glLightfv(GL_LIGHT3, GL_POSITION, light3_position);
	glLightfv(GL_LIGHT4, GL_POSITION, light4_position);


	glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, light0_direction);
	glLightfv(GL_LIGHT1, GL_SPOT_DIRECTION, light1_direction);
	glLightfv(GL_LIGHT2, GL_SPOT_DIRECTION, light2_direction);
	glLightfv(GL_LIGHT3, GL_SPOT_DIRECTION, light3_direction);
	glLightfv(GL_LIGHT4, GL_SPOT_DIRECTION, light4_direction);


	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	glEnable(GL_LIGHT2);
	glEnable(GL_LIGHT3);
	glEnable(GL_LIGHT4);
}