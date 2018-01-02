#pragma warning(disable : 4996)

//window3d and application should both extend some base class that has camera, knows how to draw ground, reflections, etc...


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

	showReflections = true;
	showGroundPlane = true;
}

void SimWindow::addMenuItems() {
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
	glEnable(GL_LIGHTING);

	int flags = 0;
	if (drawMeshes) flags |= SHOW_MESH | SHOW_MATERIALS | SHOW_ABSTRACT_VIEW;
	if (drawSkeletonView) flags |= SHOW_BODY_FRAME | SHOW_ABSTRACT_VIEW;
	if (drawMOIs) flags |= SHOW_MOI_BOX;
	if (drawCDPs) flags |= SHOW_CD_PRIMITIVES;
	if (drawSkeletonView) flags |= SHOW_JOINTS;

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


	//do the integration of wheel motions here...
	if (activeController == kinematicController) {
		for (uint j = 0; j < activeController->motionPlan->endEffectorTrajectories.size(); j++) {
			LocomotionEngine_EndEffectorTrajectory* eeTraj = &activeController->motionPlan->endEffectorTrajectories[j];
			if (eeTraj->isWheel) {
				RigidBody* rb = eeTraj->endEffectorRB;
				int eeIndex = eeTraj->CPIndex;
				int meshIndex = rb->rbProperties.endEffectorPoints[eeIndex].meshIndex;
				if (meshIndex >= 0)
					rb->meshTransformations[meshIndex].R = getRotationQuaternion(simTimeStep * rb->rbProperties.endEffectorPoints[eeIndex].wheelSpeed_rel, rb->rbProperties.endEffectorPoints[eeIndex].localCoordsWheelAxis).getRotationMatrix() * rb->meshTransformations[meshIndex].R;
			}
		}
	}

	activeController->advanceInTime(simTimeStep);

}

