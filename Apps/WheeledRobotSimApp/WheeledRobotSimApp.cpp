#include <GUILib/GLUtils.h>
#include "WheeledRobotSimApp.h"
#include <GUILib/GLMesh.h>
#include <GUILib/GLContentManager.h>
#include <MathLib/MathLib.h>
#include <RBSimLib/ODERBEngine.h>
#include <ControlLib/GeneralizedCoordinatesRobotRepresentation.h>

WheeledRobotSimApp::WheeledRobotSimApp(bool maximizeWindows)
    : GLApplication(maximizeWindows)
{
	setWindowTitle("Test Application for RBSim");

	drawCDPs = false;
	drawSkeletonView = true;
	showGroundPlane = true;

	mainMenu->addGroup("RB Sim Visualization options");

	mainMenu->addVariable("Draw Meshes", drawMeshes);
	mainMenu->addVariable("Draw MOIs", drawMOIs);
	mainMenu->addVariable("Draw CDPs", drawCDPs);
	mainMenu->addVariable("Draw SkeletonView", drawSkeletonView);
	mainMenu->addVariable("Draw Joints", drawJoints);
	mainMenu->addVariable("Draw ContactForces", drawContactForces);

	menuScreen->performLayout();


	worldOracle = new WorldOracle(Globals::worldUp, Globals::groundPlane);

//	loadFile("../data/rbs/trex.rbs");
//	loadFile("../data/rbs/trex.rs");

    drawCDPs = true;

    loadFile("../data/rbs/wheely.rbs");
//    loadFile("../data/robotsAndMotionPlans/crab/robot.rbs");

//    loadFile("../data/robotsAndMotionPlans/starlETH/robot.rbs");

//	loadFile("../data/rbs/starlETH.rbs");
//	loadFile("../data/rbs/dinoV1.rbs");
//	loadFile("../data/rbs/trex2.rbs");
//  loadFile("../data/rbs/YunfeiSKEL003.rbs");
//  loadFile("../data/rbs/chain.rbs");
 //   loadFile("../data/rbs/sample2.rbs");
//	loadFile("../data/rbs/test.rbs");

//	loadFile("../data/rbs/tmp.rbs");

	simTimeStep = 1 / 100.0;
}

WheeledRobotSimApp::~WheeledRobotSimApp(void){
}


//triggered when mouse moves
bool WheeledRobotSimApp::onMouseMoveEvent(double xPos, double yPos) {
	if (tWidget.onMouseMoveEvent(xPos, yPos) == true) return true;
	if (GLApplication::onMouseMoveEvent(xPos, yPos) == true) return true;

	return false;
}

//triggered when mouse buttons are pressed
bool WheeledRobotSimApp::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
	if (tWidget.onMouseButtonEvent(button, action, mods, xPos, yPos) == true) return true;

	if (GLApplication::onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;

//	if (mods & GLFW_MOD_SHIFT) {
//		return true;
//	}

	return false;
}

//triggered when using the mouse wheel
bool WheeledRobotSimApp::onMouseWheelScrollEvent(double xOffset, double yOffset) {
	if (GLApplication::onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	return false;
}

bool WheeledRobotSimApp::onKeyEvent(int key, int action, int mods) {
	if (GLApplication::onKeyEvent(key, action, mods)) return true;

    if (key == GLFW_KEY_UP && action == GLFW_PRESS)
    {
        for(Joint* j : rbEngine->joints)
            addAngularVelocityTo(j, 1);
    }
    else if (key == GLFW_KEY_DOWN && action == GLFW_PRESS)
    {
        for(Joint* j : rbEngine->joints)
            addAngularVelocityTo(j, -1);
    }

	return false;
}

bool WheeledRobotSimApp::onCharacterPressedEvent(int key, int mods) {
	if (GLApplication::onCharacterPressedEvent(key, mods)) return true;

	return false;
}

void WheeledRobotSimApp::loadFile(const char* fName) {
	if (strcmp(lastLoadedFile.c_str(), fName) != 0)
		lastLoadedFile = string("") + fName;
	Logger::consolePrint("Loading file \'%s\'...\n", fName);
	std::string fileName;
	fileName.assign(fName);

	std::string fNameExt = fileName.substr(fileName.find_last_of('.') + 1);

	if (fNameExt.compare("rbs") == 0) {
		delete rbEngine;
		rbEngine = new ODERBEngine();

		rbEngine->loadRBsFromFile(fName);

        // create the ground plane rigid body
        worldOracle->writeRBSFile("../out/tmpRB.rbs");
		rbEngine->loadRBsFromFile("../out/tmpRB.rbs");

		return;
	}

	if (fNameExt.compare("rs") == 0) {
		if (rbEngine->rbs.size() > 0){
			Robot* robot = new Robot(rbEngine->rbs[0]);
			robot->loadReducedStateFromFile(fName);
			delete robot;
		}
		return;
	}

}

void WheeledRobotSimApp::saveFile(const char* fName) {
	Logger::consolePrint("SAVE FILE: Do not know what to do with file \'%s\'\n", fName);
}

// Run the App tasks
void WheeledRobotSimApp::process() {
	//do the work here...
	double simulationTime = 0;
	double maxRunningTime = 1.0 / desiredFrameRate;

	//if we still have time during this frame, or if we need to finish the physics step, do this until the simulation time reaches the desired value
	while (simulationTime / maxRunningTime < animationSpeedupFactor)
	{

		simulationTime += simTimeStep;
		rbEngine->step(simTimeStep);
	}
}

#include <fstream>

// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
void WheeledRobotSimApp::drawScene() {
	glDisable(GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);
	glColor3d(1,1,1);

	glEnable(GL_LIGHTING);

	int flags = 0;
	if (drawMeshes) flags |= SHOW_MESH | SHOW_MATERIALS;
	if (drawSkeletonView) flags |= SHOW_BODY_FRAME | SHOW_ABSTRACT_VIEW;
	if (drawMOIs) flags |= SHOW_MOI_BOX;
	if (drawCDPs) flags |= SHOW_CD_PRIMITIVES;
	if (drawJoints) flags |= SHOW_JOINTS;

	if (rbEngine)
		rbEngine->drawRBs(flags);

	if (drawContactForces)
		rbEngine->drawContactForces();

	tWidget.draw();

}

// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
void WheeledRobotSimApp::drawAuxiliarySceneInfo() {

}

// Restart the application.
void WheeledRobotSimApp::restart() {
	loadFile(lastLoadedFile.c_str());
}

bool WheeledRobotSimApp::processCommandLine(const std::string& cmdLine) {

	if (GLApplication::processCommandLine(cmdLine)) return true;

	return false;
}

void WheeledRobotSimApp::addAngularVelocityTo(Joint *j, double d)
{
    HingeJoint *hingeJoint = dynamic_cast<HingeJoint*>(j);
    if(hingeJoint != nullptr && j->controlMode == JOINT_MODE::VELOCITY_MODE)
    {
        V3D axis = hingeJoint->rotationAxis.unit();
        j->desiredRelativeAngVelocity += axis*d;
    }
}
