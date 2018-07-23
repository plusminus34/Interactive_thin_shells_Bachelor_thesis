#pragma warning(disable : 4996)

#include "SimulationWindow.h"
#include <RBSimLib/ODERBEngine.h>

SimulationWindow::SimulationWindow(int x, int y, int w, int h, GLApplication* glApp) : GLWindow3D(x, y, w, h) {
	this->glApp = glApp;

	simTimeStep = 1 / 120.0;
	//	Globals::g = 0;

	dynamic_cast<GLTrackingCamera*>(this->camera)->rotAboutRightAxis = 0.25;
	dynamic_cast<GLTrackingCamera*>(this->camera)->rotAboutUpAxis = 0.95;
	dynamic_cast<GLTrackingCamera*>(this->camera)->camDistance = -1.5;

	showReflections = true;
	showGroundPlane = true;
}

void SimulationWindow::addMenuItems() {
	nanogui::Widget *tools = new nanogui::Widget(glApp->mainMenu->window());
	glApp->mainMenu->addWidget("", tools);
	tools->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
		nanogui::Alignment::Middle, 0, 4));

	nanogui::Button* button;

	button = new nanogui::Button(tools, "Meshes");
	button->setFlags(nanogui::Button::ToggleButton);
	button->setPushed(drawMeshes);
	button->setChangeCallback([this, button](bool val) {  drawMeshes = val; });
	button->setTooltip("Draw meshes");

	button = new nanogui::Button(tools, "MOIs");
	button->setFlags(nanogui::Button::ToggleButton);
	button->setPushed(drawMOIs);
	button->setChangeCallback([this, button](bool val) {  drawMOIs = val; });
	button->setTooltip("Draw moments of intertia");

	button = new nanogui::Button(tools, "CDPs");
	button->setFlags(nanogui::Button::ToggleButton);
	button->setPushed(drawCDPs);
	button->setChangeCallback([this, button](bool val) {  drawCDPs = val; });
	button->setTooltip("Draw collision detection primitives");

	button = new nanogui::Button(tools, "Skel");
	button->setFlags(nanogui::Button::ToggleButton);
	button->setPushed(drawSkeletonView);
	button->setChangeCallback([this, button](bool val) {  drawSkeletonView = val; });
	button->setTooltip("Draw skeleton and joints");
}

SimulationWindow::~SimulationWindow(){
	clear();
}

void SimulationWindow::clear(){

}

void SimulationWindow::reset(){
}

void SimulationWindow::drawAuxiliarySceneInfo(){
	glClear(GL_DEPTH_BUFFER_BIT);

	preDraw();
	postDraw();

	glClear(GL_DEPTH_BUFFER_BIT);
}

bool SimulationWindow::onMouseMoveEvent(double xPos, double yPos){
	perturbationForce = V3D();
	int shiftDown = glfwGetKey(this->glApp->glfwWindow, GLFW_KEY_LEFT_SHIFT);
	if (GlobalMouseState::lButtonPressed && shiftDown == GLFW_PRESS) {
		setPerturbationForceFromMouseInput(xPos, yPos);
		return true;
	}

	return GLWindow3D::onMouseMoveEvent(xPos, yPos);
}

bool SimulationWindow::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos){
	perturbationForce = V3D();
	int shiftDown = glfwGetKey(this->glApp->glfwWindow, GLFW_KEY_LEFT_SHIFT);
	if (GlobalMouseState::lButtonPressed && shiftDown == GLFW_PRESS) {
		setPerturbationForceFromMouseInput(xPos, yPos);
		return true;
	}

	return GLWindow3D::onMouseButtonEvent(button, action, mods, xPos, yPos);
}

void SimulationWindow::setViewportParameters(int posX, int posY, int sizeX, int sizeY){
	GLWindow3D::setViewportParameters(posX, posY, sizeX, sizeY);
}

Robot* SimulationWindow::loadRobot(const char* fName) {
	delete rbEngine;
	delete robot;
	delete worldOracle;
	rbEngine = new ODERBEngine();
	worldOracle = new WorldOracle(Globals::worldUp, Globals::groundPlane);
	rbEngine->loadRBsFromFile(fName);
	robot = new Robot(rbEngine->rbs[0]);

	setupSimpleRobotStructure(robot);

	robot->addWheelsAsAuxiliaryRBs(rbEngine);

	worldOracle->writeRBSFile("../out/tmpEnvironment.rbs");
	rbEngine->loadRBsFromFile("../out/tmpEnvironment.rbs");

	stridePhase = 0;

	return robot;
}

void SimulationWindow::loadMotionPlan(MotionPlanner* mp) {
	delete trackingController;
	delete playbackController;

	trackingController = new TrackingController(robot, mp->locomotionManager->motionPlan);
	playbackController = new PlaybackController(robot, mp);

	this->mp = mp->locomotionManager->motionPlan;

	stridePhase = 0;
}

void SimulationWindow::drawScene() {
	glEnable(GL_LIGHTING);

	int flags = 0;
	if (drawMeshes) flags |= SHOW_MESH | SHOW_MATERIALS;
	if (drawSkeletonView) flags |= SHOW_BODY_FRAME | SHOW_ABSTRACT_VIEW | SHOW_JOINTS | SHOW_WHEELS;
	if (drawMOIs) flags |= SHOW_MOI_BOX;
	if (drawCDPs) flags |= SHOW_CD_PRIMITIVES;

	glEnable(GL_LIGHTING);
	if (rbEngine)
		rbEngine->drawRBs(flags);
	glDisable(GL_LIGHTING);

	glColor3d(1.0, 0.7, 0.7);
	if (robot)
		drawArrow(robot->root->getCMPosition(), robot->root->getCMPosition() + perturbationForce, 0.01, 12);
}


void SimulationWindow::setPerturbationForceFromMouseInput(double xPos, double yPos) {
	pushViewportTransformation();
	Ray ray = getRayFromScreenCoords(xPos, yPos);
	popViewportTransformation();
	P3D pForce;
	//ray.getDistanceToPoint(robot->root->getCMPosition(), &pForce);
	ray.getDistanceToPlane(Plane(robot->root->getCMPosition(), V3D(0, 1, 0)), &pForce);
	//now we know how to compute this perturbation force...
	perturbationForce = V3D(robot->root->getCMPosition(), pForce);
	perturbationForce.y() = 0;
	if (perturbationForce.length() > 2.0) perturbationForce = perturbationForce.unit() * 2.0;
	forceScale = robot->getMass() * 1.5;
}

void SimulationWindow::doPhysicsStep(double simStep) {
	for (int i = 0; i < nPhysicsSubsteps; i++) {
		activeController->applyControlSignals(simStep / nPhysicsSubsteps);
		rbEngine->applyForceTo(robot->root, perturbationForce * forceScale, P3D());
		rbEngine->step(simStep / nPhysicsSubsteps);
		robot->bFrame->updateStateInformation();
	}
}

void SimulationWindow::advanceSimulation(double dt) {
	if (!activeController){
		stridePhase += dt / mp->motionPlanDuration;
		return;
	}

	activeController->stridePhase = stridePhase;

	if (activeController == playbackController){
		activeController->advanceInTime(dt);
		activeController->computeControlSignals(dt);
		activeController->applyControlSignals(dt);
	}
	else {
		double simulationTime = 0;

		while (simulationTime < dt) {
			simulationTime += simTimeStep;
			activeController->advanceInTime(simTimeStep);
			activeController->computeControlSignals(simTimeStep);
			doPhysicsStep(simTimeStep);
		}
	}

	stridePhase = activeController->stridePhase;
}
