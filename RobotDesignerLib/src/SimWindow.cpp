#pragma warning(disable : 4996)

#include <RobotDesignerLib/SimWindow.h>
#include <RobotDesignerLib/LocomotionEngineManagerGRF.h>
#include <RobotDesignerLib/LocomotionEngineManagerIP.h>
#include <RBSimLib/ODERBEngine.h>

SimWindow::SimWindow(int x, int y, int w, int h, GLApplication* glApp) : GLWindow3D(x, y, w, h) {
	this->glApp = glApp;

	simTimeStep = 1 / 120.0;
	//	Globals::g = 0;

	dynamic_cast<GLTrackingCamera*>(this->camera)->rotAboutRightAxis = 0.25;
	dynamic_cast<GLTrackingCamera*>(this->camera)->rotAboutUpAxis = 0.95;
	dynamic_cast<GLTrackingCamera*>(this->camera)->camDistance = -1.5;
}

void SimWindow::addMenuItems() {
	glApp->mainMenu->addVariable("draw meshes", drawMeshes);
	glApp->mainMenu->addVariable("draw MOIs", drawMOIs);
	glApp->mainMenu->addVariable("draw CDPs", drawCDPs);
	glApp->mainMenu->addVariable("draw skeleton", drawSkeletonView);
	glApp->mainMenu->addVariable("draw joints", drawJoints);
}

SimWindow::~SimWindow(){
	clear();
}

void SimWindow::clear(){

}

void SimWindow::reset(){
}

void SimWindow::drawAuxiliarySceneInfo(){
	glClear(GL_DEPTH_BUFFER_BIT);

	preDraw();
	postDraw();

	glClear(GL_DEPTH_BUFFER_BIT);
}

bool SimWindow::onMouseMoveEvent(double xPos, double yPos){
	perturbationForce = V3D();
	int shiftDown = glfwGetKey(this->glApp->glfwWindow, GLFW_KEY_LEFT_SHIFT);
	if (GlobalMouseState::lButtonPressed && shiftDown == GLFW_PRESS) {
		setPerturbationForceFromMouseInput(xPos, yPos);
		return true;
	}

	return GLWindow3D::onMouseMoveEvent(xPos, yPos);
}

bool SimWindow::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos){
	perturbationForce = V3D();
	int shiftDown = glfwGetKey(this->glApp->glfwWindow, GLFW_KEY_LEFT_SHIFT);
	if (GlobalMouseState::lButtonPressed && shiftDown == GLFW_PRESS) {
		setPerturbationForceFromMouseInput(xPos, yPos);
		return true;
	}

	return GLWindow3D::onMouseButtonEvent(button, action, mods, xPos, yPos);
}

void SimWindow::setViewportParameters(int posX, int posY, int sizeX, int sizeY){
	GLWindow3D::setViewportParameters(posX, posY, sizeX, sizeY);
}

Robot* SimWindow::loadRobot(const char* fName) {
	delete rbEngine;
	delete robot;
	delete worldOracle;
	rbEngine = new ODERBEngine();
	worldOracle = new WorldOracle(Globals::worldUp, Globals::groundPlane);
	rbEngine->loadRBsFromFile(fName);
	robot = new Robot(rbEngine->rbs[0]);
	setupSimpleRobotStructure(robot);
	worldOracle->writeRBSFile("../out/tmpEnvironment.rbs");
	rbEngine->loadRBsFromFile("../out/tmpEnvironment.rbs");
	return robot;
}

void SimWindow::loadMotionPlan(LocomotionEngineMotionPlan* mp) {
	delete positionController;
	delete torqueController;
	delete kinematicController;

	positionController = new PositionBasedRobotController(robot, mp);
	torqueController = new TorqueBasedRobotController(robot, mp);
	kinematicController = new KinematicRobotController(robot, mp);
}

void SimWindow::drawScene() {
	glColor3d(1, 1, 1);
	glDisable(GL_LIGHTING);
	drawGround(GLContentManager::getTexture("../data/textures/ground_TileLight2.bmp"));
	glEnable(GL_LIGHTING);

	int flags = 0;
	if (drawMeshes) flags |= SHOW_MESH | SHOW_MATERIALS | SHOW_ABSTRACT_VIEW;
	if (drawSkeletonView) flags |= SHOW_BODY_FRAME | SHOW_ABSTRACT_VIEW;
	if (drawMOIs) flags |= SHOW_MOI_BOX;
	if (drawCDPs) flags |= SHOW_CD_PRIMITIVES;
	if (drawJoints) flags |= SHOW_JOINTS;

	glEnable(GL_LIGHTING);
	if (rbEngine)
		rbEngine->drawRBs(flags);
	glDisable(GL_LIGHTING);

	glColor3d(1.0, 0.7, 0.7);
	if (robot)
		drawArrow(robot->root->getCMPosition(), robot->root->getCMPosition() + perturbationForce, 0.01, 12);
}


void SimWindow::setPerturbationForceFromMouseInput(double xPos, double yPos) {
	preDraw();
	Ray ray = getRayFromScreenCoords(xPos, yPos);
	postDraw();
	P3D pForce;
	//ray.getDistanceToPoint(robot->root->getCMPosition(), &pForce);
	ray.getDistanceToPlane(Plane(robot->root->getCMPosition(), V3D(0, 1, 0)), &pForce);
	//now we know how to compute this perturbation force...
	perturbationForce = V3D(robot->root->getCMPosition(), pForce);
	perturbationForce.y() = 0;
	if (perturbationForce.length() > 2.0) perturbationForce = perturbationForce.unit() * 2.0;
	forceScale = robot->getMass() * 1.5;
}

void SimWindow::step() {
	if (!activeController)
		return;
	activeController->computeControlSignals(simTimeStep);

	for (int i = 0; i < nPhysicsStepsPerControlStep; i++) {
		activeController->applyControlSignals();
		rbEngine->applyForceTo(robot->root, perturbationForce * forceScale, P3D());
		if (activeController != kinematicController)
			rbEngine->step(simTimeStep / nPhysicsStepsPerControlStep);
		robot->bFrame->updateStateInformation();
	}

	activeController->advanceInTime(simTimeStep);

}

void SimWindow::setupLights() {
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

