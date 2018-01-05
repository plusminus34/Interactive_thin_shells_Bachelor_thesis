#include <GUILib/GLUtils.h>

#include "IKApp.h"
#include <GUILib/GLMesh.h>
#include <GUILib/GLContentManager.h>
#include <MathLib/MathLib.h>
#include <RBSimLib/ODERBEngine.h>
#include <ControlLib/SimpleLimb.h>

IKApp::IKApp() {
	setWindowTitle("RobotIK");

//	loadFile("../data/rbs/trex.rbs");
//	loadFile("../data/rbs/trex.rs");

    //loadFile("../data/rbs/bip/bip.rbs");

    loadFile("../data/rbs/yumi/yumi.rbs");

	showMesh = true;


	mainMenu->addGroup("IK App Visualization options");

	mainMenu->addVariable("Draw Meshes", showMesh);
	mainMenu->addVariable("Draw MOIs", showMOI);
	mainMenu->addVariable("Draw CDPs", showCDPs);
	mainMenu->addVariable("Draw Joints", showRotationAxes);
//	mainMenu->addVariable("Draw SkeletonView", drawSkeletonView);

	menuScreen->performLayout();

	showGroundPlane = true;
	showReflections = true;
}

void IKApp::loadRobot(const char* fName) {
	delete robot;
	delete rbEngine;
//	delete poseSolver;

	rbEngine = new ODERBEngine();
	rbEngine->loadRBsFromFile(fName);
	robot = new Robot(rbEngine->rbs[0]);
	startState = ReducedRobotState(robot);
	setupSimpleRobotStructure(robot);

	delete ikSolver;
	ikSolver = new IK_Solver(robot, true);

//	poseSolver = new IKPoseSolver(robot);
//	poseSolver->setTargetLimb(targetLimb);

	//controller = new IKPoseSolver(robot, motionPlan);
}

void IKApp::loadFile(const char* fName) {
	Logger::consolePrint("Loading file \'%s\'...\n", fName);
	std::string fileName;
	fileName.assign(fName);

	std::string fNameExt = fileName.substr(fileName.find_last_of('.') + 1);

	if (fNameExt.compare("rbs") == 0)
		loadRobot(fName);
	if (fNameExt.compare("rs") == 0) {
		if (robot) {
			robot->loadReducedStateFromFile(fName);
			startState = ReducedRobotState(robot);
			ikSolver->ikPlan->setTargetIKStateFromRobot();
		}
	}
}

IKApp::~IKApp(void) {
//	delete poseSolver;
	delete rbEngine;
	delete robot;
}

// Restart the application.
void IKApp::restart() {
	loadFile("../data/rbs/bip/bip.rbs");
}

// Run the App tasks
void IKApp::process() {
	ikSolver->ikEnergyFunction->regularizer = 100;
	ikSolver->ikOptimizer->checkDerivatives = true;
	ikSolver->solve();
}

//triggered when mouse moves
bool IKApp::onMouseMoveEvent(double xPos, double yPos) {
	Ray ray = getRayFromScreenCoords(xPos, yPos);

	if (robot && selectedRigidBody == NULL){
		for (int i = 0; i < robot->getRigidBodyCount(); i++)
			robot->getRigidBody(i)->selected = false;

		highlightedRigidBody = NULL;
		P3D pLocal;
		double tMin = DBL_MAX;
		for (int i = 0; i < robot->getRigidBodyCount(); i++)
			if (robot->getRigidBody(i)->getRayIntersectionPointTo(ray, &pLocal)) {
				double tVal = ray.getRayParameterFor(robot->getRigidBody(i)->getWorldCoordinates(pLocal));
				if (tVal < tMin) {
					tMin = tVal;
					highlightedRigidBody = robot->getRigidBody(i);
				}
			}
		if (highlightedRigidBody){
			highlightedRigidBody->selected = true;
			Logger::consolePrint("highlighted rb %s, positioned at %lf %lf %lf\n", highlightedRigidBody->name.c_str(), highlightedRigidBody->getCMPosition().x(), highlightedRigidBody->getCMPosition().y(), highlightedRigidBody->getCMPosition().z());
		}
	}

	if (selectedRigidBody) {
		V3D viewPlaneNormal = V3D(camera->getCameraPosition(), camera->getCameraTarget()).unit();
		ray.getDistanceToPlane(Plane(selectedRigidBody->getWorldCoordinates(selectedPoint), viewPlaneNormal), &targetPoint);
//		ray.getDistanceToPoint(selectedRigidBody->getWorldCoordinates(selectedPoint), &targetPoint);
		ikSolver->ikPlan->endEffectors.back().targetEEPos = targetPoint;
		return true;
	}

	if (GLApplication::onMouseMoveEvent(xPos, yPos)) return true;

	return false;
}

//triggered when mouse buttons are pressed
bool IKApp::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {

	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
		selectedRigidBody = highlightedRigidBody;
		if (selectedRigidBody) {
			selectedRigidBody->getRayIntersectionPointTo(getRayFromScreenCoords(xPos, yPos), &selectedPoint);
			getRayFromScreenCoords(xPos, yPos).getDistanceToPoint(selectedRigidBody->getWorldCoordinates(selectedPoint), &targetPoint);
			ikSolver->ikPlan->endEffectors.push_back(IK_EndEffector());
			ikSolver->ikPlan->endEffectors.back().endEffectorLocalCoords = selectedPoint;
			ikSolver->ikPlan->endEffectors.back().endEffectorRB = selectedRigidBody;
			ikSolver->ikPlan->endEffectors.back().targetEEPos = targetPoint;
		}
	}

	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE){
		ikSolver->ikPlan->endEffectors.clear();
		selectedRigidBody = NULL;
	}

	if (GLApplication::onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;

	return false;
}

//triggered when using the mouse wheel
bool IKApp::onMouseWheelScrollEvent(double xOffset, double yOffset) {
	if (GLApplication::onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	return false;
}

bool IKApp::onKeyEvent(int key, int action, int mods) {

	if (GLApplication::onKeyEvent(key, action, mods)) return true;

	return false;
}

bool IKApp::onCharacterPressedEvent(int key, int mods) {
	if (GLApplication::onCharacterPressedEvent(key, mods)) return true;
	return false;
}

void IKApp::saveFile(const char* fName) {
	Logger::consolePrint("SAVE FILE: Do not know what to do with file \'%s\'\n", fName);
}

// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
void IKApp::drawScene() {
	glColor3d(1, 1, 1);
	glDisable(GL_LIGHTING);

	int flags = SHOW_ABSTRACT_VIEW | HIGHLIGHT_SELECTED;
	if (showMesh)
		flags |= SHOW_MESH;
	if (showMOI){
		flags |= SHOW_MOI_BOX;
		Logger::consolePrint("total mass: %lf\n", robot->mass);
	}
	if (showRotationAxes)
		flags |= SHOW_JOINTS;
	if (showCDPs)
		flags |= SHOW_CD_PRIMITIVES;

	glEnable(GL_LIGHTING);
	glPushMatrix();
	rbEngine->drawRBs(flags);

	glPopMatrix();
	glDisable(GL_LIGHTING);

	if (selectedRigidBody){
		glColor3d(1,0,0);
		glBegin(GL_LINES);
			gl_Vertex3d(selectedRigidBody->getWorldCoordinates(selectedPoint));
			gl_Vertex3d(targetPoint);
		glEnd();
	}
}

// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
void IKApp::drawAuxiliarySceneInfo() {

}

bool IKApp::processCommandLine(const std::string& cmdLine) {
	if (GLApplication::processCommandLine(cmdLine)) return true;

	return false;
}

