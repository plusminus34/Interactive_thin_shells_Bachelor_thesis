#include <GUILib/GLUtils.h>

#include "RobotDesignerApp.h"
#include <GUILib/GLMesh.h>
#include <GUILib/GLContentManager.h>
#include <MathLib/MathLib.h>
#include <ControlLib/SimpleLimb.h>

//make a Cassie model and simulation, especially once limb collision are enabled. A little cassie-like robot could also be a good target for Siggraph, maybe...
//make a base robot design window class that does nothing, only loads rbs and returns it
//have a bool display design window, such that when it ain't, it shows full screen robot
//fix all the old Tw references, lots of them around
//create a robot parameterization + add option to do one design step optimization, as a test, like in the old app I once wrote...
//add collision objectives both to IK solver and to MOPT
//add velocity limit objective to MOPT

RobotDesignerApp::RobotDesignerApp(){
	bgColor[0] = bgColor[1] = bgColor[2] = 1;
	setWindowTitle("RobotDesigner");

	moptWindow = new MOPTWindow(0, 0, 100, 100, this);
	simWindow = new SimWindow(0, 0, 100, 100, this);
	setupWindows();

	mainMenu->addGroup("RobotDesigner Options");
	mainMenu->addVariable("Execution Mode", runOption, true)->setItems({ "MOPT", "Play", "SimPD", "SimTau"});
	mainMenu->addVariable("Show MOPT Window", drawMOPTWindow);
	moptWindow->addMenuItems();
	mainMenu->addButton("Warmstart MOTP", [this]() { warmStartMOPT(true); });

	showGroundPlane = false;

/*
	TwAddSeparator(mainMenuBar, "sep2", "");

	drawCDPs = false;
	drawSkeletonView = false;
	showGroundPlane = false;

	TwAddButton(mainMenuBar, "LoadRobotToSim", LoadRobotToSim, this, " label='Load To Simulation' group='Sim' key='l' ");
	TwAddButton(mainMenuBar, "WarmstartMOPT", RestartMOPT, this, " label='WarmStart MOPT' group='Sim' key='w' ");

	TwAddVarRW(mainMenuBar, "RunOptions", TwDefineEnumFromString("RunOptions", "..."), &runOption, "group='Sim'");
	DynamicArray<std::string> runOptionList;
	runOptionList.push_back("\\Motion Optimization");
	runOptionList.push_back("\\Motion Plan Animation");
	runOptionList.push_back("\\Simulation-PositionControl");
	runOptionList.push_back("\\Simulation-TorqueControl");
	generateMenuEnumFromFileList("MainMenuBar/RunOptions", runOptionList);

	TwAddVarRW(mainMenuBar, "do debug", TW_TYPE_BOOLCPP, &doDebug, " label='doDebug' group='Viz2'");
	TwAddVarRW(mainMenuBar, "drawMotionPlan", TW_TYPE_BOOLCPP, &drawMotionPlan, " label='drawMotionPlan' group='Viz2'");
	TwAddVarRW(mainMenuBar, "DrawMeshes", TW_TYPE_BOOLCPP, &drawMeshes, " label='drawMeshes' group='Viz2'");
	TwAddVarRW(mainMenuBar, "DrawMOIs", TW_TYPE_BOOLCPP, &drawMOIs, " label='drawMOIs' group='Viz2'");
	TwAddVarRW(mainMenuBar, "DrawCDPs", TW_TYPE_BOOLCPP, &drawCDPs, " label='drawCDPs' group='Viz2'");
	TwAddVarRW(mainMenuBar, "DrawSkeletonView", TW_TYPE_BOOLCPP, &drawSkeletonView, " label='drawSkeleton' group='Viz2'");
	TwAddVarRW(mainMenuBar, "DrawJoints", TW_TYPE_BOOLCPP, &drawJoints, " label='drawJoints' group='Viz2'");
	TwAddVarRW(mainMenuBar, "drawContactForces", TW_TYPE_BOOLCPP, &drawContactForces, " label='drawContactForces' group='Viz2'");
	TwAddVarRW(mainMenuBar, "drawOrientation", TW_TYPE_BOOLCPP, &drawOrientation, " label='drawOrientation' group='Viz2'");

	TwAddSeparator(mainMenuBar, "sep3", "");

	TwAddSeparator(mainMenuBar, "sep4", "");
*/


	bgColor[0] = bgColor[1] = bgColor[2] = 0.75;

	loadFile("../data/robotsAndMotionplans/spotMini/robot2.rbs");
	loadFile("../data/robotsAndMotionplans/spotMini/robot.rs");
//	loadToSim();
	loadToSim(false);
	loadFile("../data/robotsAndMotionplans/spotMini/trot.p");



	menuScreen->performLayout();
	setupWindows();


	followCameraTarget = true;
}

void RobotDesignerApp::setupWindows() {
	int w, h;

	int offset = (int)(mainMenu->window()->width() * menuScreen->pixelRatio());

	w = (GLApplication::getMainWindowWidth()) - offset;
	h = GLApplication::getMainWindowHeight();

	if (drawMOPTWindow){
		moptWindow->setViewportParameters(offset, 0, w / 2, h);
		moptWindow->ffpViewer->setViewportParameters(offset, 0, w/2, h/4);

		consoleWindow->setViewportParameters(offset + w / 2, 0, w / 2, 280);
		simWindow->setViewportParameters(offset + w / 2, 0, w / 2, h);
	}
	else {
		consoleWindow->setViewportParameters(offset, 0, w, 280);
		simWindow->setViewportParameters(offset, 0, w, h);
	}

}


RobotDesignerApp::~RobotDesignerApp(void){
}

//triggered when mouse moves
bool RobotDesignerApp::onMouseMoveEvent(double xPos, double yPos) {
	if (simWindow->isActive() || simWindow->mouseIsWithinWindow(xPos, yPos))
		if (simWindow->onMouseMoveEvent(xPos, yPos)) return true;

	if (drawMOPTWindow)
		if (moptWindow->isActive() || moptWindow->mouseIsWithinWindow(xPos, yPos))
			if (moptWindow->onMouseMoveEvent(xPos, yPos)) return true;

	if (GLApplication::onMouseMoveEvent(xPos, yPos)) return true;

	return false;
}

//triggered when mouse buttons are pressed
bool RobotDesignerApp::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
	if (simWindow->isActive() || simWindow->mouseIsWithinWindow(xPos, yPos))
		if (simWindow->onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;

	if (drawMOPTWindow)
		if (moptWindow->isActive() || moptWindow->mouseIsWithinWindow(xPos, yPos))
			if (moptWindow->onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;

	if (GLApplication::onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;

	return false;
}

//triggered when using the mouse wheel
bool RobotDesignerApp::onMouseWheelScrollEvent(double xOffset, double yOffset) {
	if (simWindow->mouseIsWithinWindow(GlobalMouseState::lastMouseX, GlobalMouseState::lastMouseY))
		if (simWindow->onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	if (drawMOPTWindow)
		if (moptWindow->mouseIsWithinWindow(GlobalMouseState::lastMouseX, GlobalMouseState::lastMouseY))
			if (moptWindow->onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	if (GLApplication::onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	return false;
}

bool RobotDesignerApp::onKeyEvent(int key, int action, int mods) {
	if (drawMOPTWindow) {
		if (moptWindow->locomotionManager && moptWindow->locomotionManager->motionPlan) {
			if (key == GLFW_KEY_UP && action == GLFW_PRESS)
				moptWindow->moptParams.desTravelDistZ += 0.1;
			if (key == GLFW_KEY_DOWN && action == GLFW_PRESS)
				moptWindow->moptParams.desTravelDistZ -= 0.1;
			if (key == GLFW_KEY_LEFT && action == GLFW_PRESS)
				moptWindow->moptParams.desTravelDistX += 0.1;
			if (key == GLFW_KEY_RIGHT && action == GLFW_PRESS)
				moptWindow->moptParams.desTravelDistX -= 0.1;
			if (key == GLFW_KEY_PERIOD && action == GLFW_PRESS)
				moptWindow->moptParams.desTurningAngle += 0.1;
			if (key == GLFW_KEY_SLASH && action == GLFW_PRESS)
				moptWindow->moptParams.desTurningAngle -= 0.1;

			boundToRange(&moptWindow->moptParams.desTravelDistZ, -0.5, 0.5);
			boundToRange(&moptWindow->moptParams.desTravelDistX, -0.5, 0.5);
			boundToRange(&moptWindow->moptParams.desTurningAngle, -0.5, 0.5);
		}

		if (key == GLFW_KEY_O && action == GLFW_PRESS)
			moptWindow->locomotionManager->motionPlan->writeRobotMotionAnglesToFile("../out/tmpMPAngles.mpa");
	}

	if (key == GLFW_KEY_1 && action == GLFW_PRESS)
		runOption = MOTION_PLAN_OPTIMIZATION;
	if (key == GLFW_KEY_2 && action == GLFW_PRESS)
		runOption = MOTION_PLAN_ANIMATION;
	if (key == GLFW_KEY_3 && action == GLFW_PRESS)
		runOption = PHYSICS_SIMULATION_WITH_POSITION_CONTROL;
	if (key == GLFW_KEY_4 && action == GLFW_PRESS)
		runOption = PHYSICS_SIMULATION_WITH_TORQUE_CONTROL;

	mainMenu->refresh();

	if (GLApplication::onKeyEvent(key, action, mods)) return true;

	return false;
}

bool RobotDesignerApp::onCharacterPressedEvent(int key, int mods) {
	if (GLApplication::onCharacterPressedEvent(key, mods)) return true;
	return false;
}

void RobotDesignerApp::loadFile(const char* fName) {
	Logger::consolePrint("Loading file \'%s\'...\n", fName);
	std::string fileName;
	fileName.assign(fName);

	std::string fNameExt = fileName.substr(fileName.find_last_of('.') + 1);

	if (fNameExt.compare("rs") == 0) {
		Logger::consolePrint("Load robot state from '%s'\n", fName);
		if (robot) {
			robot->loadReducedStateFromFile(fName);
			delete initialRobotState;
			initialRobotState = new ReducedRobotState(robot);
		}
		return;
	}

	if (fNameExt.compare("rbs") == 0 ){ 
		robot = simWindow->loadRobot(fName);
		delete initialRobotState;
		initialRobotState = new ReducedRobotState(robot);
		return;
	}

	if (fNameExt.compare("ffp") == 0) {
		moptWindow->footFallPattern.loadFromFile(fName);
		moptWindow->footFallPattern.writeToFile("..\\out\\tmpFFP.ffp");
		return;
	}

	if (fNameExt.compare("p") == 0) {
		if (moptWindow->locomotionManager && moptWindow->locomotionManager->motionPlan){
			moptWindow->locomotionManager->motionPlan->readParamsFromFile(fName);
			moptWindow->locomotionManager->motionPlan->syncFootFallPatternWithMotionPlan(moptWindow->footFallPattern);
			moptWindow->footFallPattern.writeToFile("..\\out\\tmpFFP.ffp");
			moptWindow->syncMOPTWindowParameters();
		}
		return;
	}

}

void RobotDesignerApp::loadToSim(bool initializeMOPT){
/* ------ load an initial motion plan for the robot ------*/

//now, start MOPT...
	moptWindow->loadRobot(robot, initialRobotState);
	warmStartMOPT(initializeMOPT);
	Logger::consolePrint("Warmstart successful...\n");

	Logger::consolePrint("The robot has %d legs, weighs %lfkgs and is %lfm tall...\n", robot->bFrame->limbs.size(), robot->getMass(), robot->root->getCMPosition().y());
}

void RobotDesignerApp::warmStartMOPT(bool initializeMotionPlan) {
	moptWindow->initializeNewMP(initializeMotionPlan);
	simWindow->loadMotionPlan(moptWindow->locomotionManager->motionPlan);
}

void RobotDesignerApp::saveFile(const char* fName) {
	Logger::consolePrint("SAVE FILE: Do not know what to do with file \'%s\'\n", fName);
}

void RobotDesignerApp::runMOPTStep() {
	double energyVal = moptWindow->runMOPTStep();
}


P3D RobotDesignerApp::getCameraTarget() {
	if (runOption != MOTION_PLAN_OPTIMIZATION)
		return robot->root->getCMPosition();
	else
		return P3D(0, 1, 0);
}

void RobotDesignerApp::setActiveController() {
	if (runOption == PHYSICS_SIMULATION_WITH_POSITION_CONTROL)
		simWindow->setActiveController(simWindow->positionController);
	else if (runOption == PHYSICS_SIMULATION_WITH_TORQUE_CONTROL)
		simWindow->setActiveController(simWindow->torqueController);
	else if (runOption == MOTION_PLAN_ANIMATION)
		simWindow->setActiveController(simWindow->kinematicController);
	else
		simWindow->setActiveController(NULL);
}

// Run the App tasks
void RobotDesignerApp::process() {
	//we need to sync the state of the robot with the motion plan when we first start physics-based tracking...
	static int lastRunOptionSelected = runOption + 1;
	setActiveController();

	if (lastRunOptionSelected != runOption && (runOption != MOTION_PLAN_OPTIMIZATION)) {
		Logger::consolePrint("Syncronizing robot state\n");
		simWindow->getActiveController()->initialize();
		simWindow->getActiveController()->setDebugMode(doDebug);
	}

	lastRunOptionSelected = runOption;

	if (runOption != MOTION_PLAN_OPTIMIZATION && simWindow->getActiveController()) {
		double simulationTime = 0;
		double maxRunningTime = 1.0 / desiredFrameRate;

		while (simulationTime / maxRunningTime < animationSpeedupFactor) {
			simulationTime += simWindow->simTimeStep;
			simWindow->step();

			if (slowMo)
				break;
		}

	}
	else if (runOption == MOTION_PLAN_OPTIMIZATION) {
		runMOPTStep();
	}
	if (simWindow->getActiveController())
		moptWindow->ffpViewer->cursorPosition = simWindow->getActiveController()->stridePhase;
}

// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
void RobotDesignerApp::drawScene() {
	setupWindows();
}

// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
void RobotDesignerApp::drawAuxiliarySceneInfo() {
	if (drawMOPTWindow && moptWindow) {
		moptWindow->setAnimationParams(moptWindow->ffpViewer->cursorPosition, 0);
		moptWindow->draw();
		moptWindow->drawAuxiliarySceneInfo();
	}

	if (simWindow) {
		if (followCameraTarget)
			simWindow->getCamera()->followTarget(getCameraTarget());

		simWindow->draw();
		simWindow->drawAuxiliarySceneInfo();
	}
}

// Restart the application.
void RobotDesignerApp::restart() {

}

bool RobotDesignerApp::processCommandLine(const std::string& cmdLine) {
	if (GLApplication::processCommandLine(cmdLine)) return true;
	return false;
}

