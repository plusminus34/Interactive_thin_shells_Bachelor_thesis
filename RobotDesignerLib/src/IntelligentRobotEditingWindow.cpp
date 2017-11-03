#pragma warning(disable : 4996)

#include <RobotDesignerLib/IntelligentRobotEditingWindow.h>
#include <RobotDesignerLib/LocomotionEngineManagerGRF.h>
#include <RobotDesignerLib/LocomotionEngineManagerIP.h>
#include <RBSimLib/ODERBEngine.h>

//move joints. When ALT is down (or the one that is same as in design window!) then move just joint and/or everything lower down in hierarchy
//resize bones. When ALT is down, move only child. Otherwise move both the child and parent joints...

IntelligentRobotEditingWindow::IntelligentRobotEditingWindow(int x, int y, int w, int h, RobotDesignerApp* rdApp) : GLWindow3D(x, y, w, h) {
	this->rdApp = rdApp;

	dynamic_cast<GLTrackingCamera*>(this->camera)->rotAboutRightAxis = 0.25;
	dynamic_cast<GLTrackingCamera*>(this->camera)->rotAboutUpAxis = 0.95;
	dynamic_cast<GLTrackingCamera*>(this->camera)->camDistance = -1.5;
}

void IntelligentRobotEditingWindow::addMenuItems() {

}

IntelligentRobotEditingWindow::~IntelligentRobotEditingWindow(){

}

void IntelligentRobotEditingWindow::drawAuxiliarySceneInfo(){
	glClear(GL_DEPTH_BUFFER_BIT);

	preDraw();
	postDraw();

	glClear(GL_DEPTH_BUFFER_BIT);
}

bool IntelligentRobotEditingWindow::onMouseMoveEvent(double xPos, double yPos){
	int shiftDown = glfwGetKey(this->rdApp->glfwWindow, GLFW_KEY_LEFT_SHIFT);
	if (GlobalMouseState::lButtonPressed && shiftDown == GLFW_PRESS) {
		return true;
	}

	return GLWindow3D::onMouseMoveEvent(xPos, yPos);
}

bool IntelligentRobotEditingWindow::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos){
	int shiftDown = glfwGetKey(this->rdApp->glfwWindow, GLFW_KEY_LEFT_SHIFT);
	if (GlobalMouseState::lButtonPressed && shiftDown == GLFW_PRESS) {
		return true;
	}

	return GLWindow3D::onMouseButtonEvent(button, action, mods, xPos, yPos);
}

void IntelligentRobotEditingWindow::setViewportParameters(int posX, int posY, int sizeX, int sizeY){
	GLWindow3D::setViewportParameters(posX, posY, sizeX, sizeY);
}

void IntelligentRobotEditingWindow::drawScene() {
	glColor3d(1, 1, 1);
	glDisable(GL_LIGHTING);
	drawGround(GLContentManager::getTexture("../data/textures/ground_TileLight2.bmp"));
	glEnable(GL_LIGHTING);

	int flags = SHOW_ABSTRACT_VIEW | SHOW_BODY_FRAME | SHOW_JOINTS;

	ReducedRobotState rs(13);
	if (rdApp->robot){
		rs = ReducedRobotState(rdApp->robot);
		rdApp->robot->setState(rdApp->initialRobotState);
	}

	glEnable(GL_LIGHTING);
	if (rdApp->simWindow->rbEngine)
		rdApp->simWindow->rbEngine->drawRBs(flags);

	if (rdApp->robot) {
		rdApp->robot->setState(&rs);
	}


}

void IntelligentRobotEditingWindow::setupLights() {
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

