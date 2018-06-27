#include <GUILib/GLUtils.h>

#include "FastRobotControlApp.h"
#include <GUILib/GLMesh.h>
#include <GUILib/GLContentManager.h>
#include <MathLib/MathLib.h>
#include <ControlLib/SimpleLimb.h>
#include <RobotDesignerLib/IntelligentRobotEditingWindow.h>
#include <RobotDesignerLib/FastMOPTPreplanner.h>



//work towards: 
//	- long horizon motion plans that include a change in gait, legs being used as arms, etc
//	- mopt that combines locomotion and manipulation
//	- MOPT for multi-robot systems, including locomotion and manipulation
//	- interactive MOPT - add an obstacle in the scene, given current state of robot it figures out how to avoid it, etc.


FastRobotControlApp::FastRobotControlApp(){
	bgColorR = bgColorG = bgColorB = bgColorA = 1;
	setWindowTitle("RobotDesigner");

	showGroundPlane = false;

	moptWindow = new FastMOPTWindow(0, 0, 100, 100, this);
	moptWindow->optimizeOption = FastMOPTWindow::GRF_OPT_V3;
	simWindow = new SimWindow(0, 0, 100, 100, this);
	motionPlanAnalysis = new MotionPlanAnalysis(menuScreen);
	energyWindow = new EnergyWindow(NULL);

	mainMenu->addGroup("RobotDesigner Options");
	mainMenu->addVariable("Run Mode", runOption, true)->setItems({ "MOPT", "Play", "SimPD", "SimTau"});

	nanogui::Widget *tools = new nanogui::Widget(mainMenu->window());
	mainMenu->addWidget("", tools);
	tools->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
		nanogui::Alignment::Middle, 0, 4));

	nanogui::Button* button;

	button = new nanogui::Button(tools, "");
	button->setIcon(ENTYPO_ICON_LOG_OUT);
	button->setCallback([this]() { createRobotFromCurrentDesign(); });
	button->setTooltip("Load Robot Design To Sim (T)");

	button = new nanogui::Button(tools, "");
	button->setIcon(ENTYPO_ICON_BAIDU);
	button->setCallback([this]() { warmStartMOPT(true); });
	button->setTooltip("Warmstart MOPT (M)");

	mainMenu->addGroup("Analysis");

	nanogui::Widget *widget = new nanogui::Widget(mainMenu->window());
	mainMenu->addWidget("", widget);
	widget->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Middle, 0, 4));

	// button to toggle motion plan analysis
	nanogui::Button *mpaButton = new nanogui::Button(widget, "Motion Plan Analysis");
	mpaButton->setFontSize(14);
	mpaButton->setFlags(nanogui::Button::Flags::ToggleButton);
	mpaButton->setChangeCallback([this](bool state) {
		motionPlanAnalysis->window->setVisible(state);
		if (moptWindow->locomotionManager->motionPlan)
			motionPlanAnalysis->updateFromMotionPlan(moptWindow->locomotionManager->motionPlan);
	});

	// button to toggle energy window
	button = new nanogui::Button(widget, "Energy Window");
	button->setFontSize(14);
	button->setFlags(nanogui::Button::Flags::ToggleButton);
	button->setChangeCallback([this](bool state) {
		if (moptWindow->locomotionManager->energyFunction && doMotionAnalysis) {
			if (state)
				energyWindow->createEnergyMenu(moptWindow->locomotionManager->energyFunction, menuScreen);
			energyWindow->updateEnergiesWith(moptWindow->locomotionManager->energyFunction, moptWindow->locomotionManager->motionPlan->getMPParameters());
			energyWindow->setVisible(state);
		}
	});

	// button to check energy
	button = new nanogui::Button(widget, "Update");
	button->setIcon(ENTYPO_ICON_PUBLISH);
	button->setFontSize(14);
	button->setCallback([this]() {
		if (moptWindow->locomotionManager->energyFunction && doMotionAnalysis) {
			LocomotionEngineMotionPlan * motionPlan = moptWindow->locomotionManager->motionPlan;
			energyWindow->updateEnergiesWith(moptWindow->locomotionManager->energyFunction, motionPlan->getMPParameters());
			motionPlanAnalysis->updateFromMotionPlan(moptWindow->locomotionManager->motionPlan);
		}
	});

	mainMenu->addGroup("MOPT Options");
	moptWindow->addMenuItems();

	button = new nanogui::Button(widget, "goMOPT");
	button->setFontSize(14);
	button->setCallback([this]() {
		moptWindow->optimizeMotionPlan();
	});


	mainMenu->addGroup("Sim Options");
	simWindow->addMenuItems();

	showGroundPlane = false;
	bgColorR = bgColorG = bgColorB = 249.0 / 255.0;

	menuScreen->performLayout();
	setupWindows();

	loadFile("..\\data\\RobotDesigner\\SpotMiniDemo.batch");
	robot->forward = V3D(0, 0, 1);
	robot->right = V3D(-1, 0, 0);
	moptWindow->generateMotionPreplan();

	followCameraTarget = true;
}

void FastRobotControlApp::setupWindows() {
	int w, h;

	int offset = (int)(mainMenu->window()->width() * menuScreen->pixelRatio());

	w = (GLApplication::getMainWindowWidth()) - offset;
	h = GLApplication::getMainWindowHeight();

	if (viewOptions == SIM_AND_MOPT_WINDOWS) {
		moptWindow->setViewportParameters(offset, 0, w / 2, h);
		moptWindow->ffpViewer->setViewportParameters(offset, 0, w / 2, h / 4);

		consoleWindow->setViewportParameters(offset + w / 2, 0, w / 2, 280);
		simWindow->setViewportParameters(offset + w / 2, 0, w / 2, h);
		showConsole = true;
	} else if (viewOptions == SIM_WINDOW_ONLY) {
		consoleWindow->setViewportParameters(offset, 0, w, 280);
		simWindow->setViewportParameters(offset, 0, w, h);
		showConsole = true;
	}
	else {
		moptWindow->setViewportParameters(offset, 0, w, h);
		moptWindow->ffpViewer->setViewportParameters(offset, 0, w, h / 4);
		showConsole = false;
	}

}

FastRobotControlApp::~FastRobotControlApp(void){
}

bool FastRobotControlApp::shouldShowSimWindow() {
	return viewOptions != MOPT_WINDOW_ONLY;
}

bool FastRobotControlApp::shouldShowMOPTWindow() {
	return viewOptions != SIM_WINDOW_ONLY;
}

//triggered when mouse moves
bool FastRobotControlApp::onMouseMoveEvent(double xPos, double yPos) {
	if (shouldShowSimWindow() && (simWindow->isActive() || simWindow->mouseIsWithinWindow(xPos, yPos)))
		if (simWindow->onMouseMoveEvent(xPos, yPos)) {
			return true;
		}

	if (shouldShowMOPTWindow() && (moptWindow->isActive() || moptWindow->mouseIsWithinWindow(xPos, yPos)))
		if (moptWindow->onMouseMoveEvent(xPos, yPos)) {
			return true;
		}

	if (GLApplication::onMouseMoveEvent(xPos, yPos)) return true;

	return false;
}

//triggered when mouse buttons are pressed
bool FastRobotControlApp::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
	if (shouldShowSimWindow() && (simWindow->isActive() || simWindow->mouseIsWithinWindow(xPos, yPos)))
		if (simWindow->onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;

	if (shouldShowMOPTWindow() && (moptWindow->isActive() || moptWindow->mouseIsWithinWindow(xPos, yPos)))
		if (moptWindow->onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;

	if (GLApplication::onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;

	return false;
}

//triggered when using the mouse wheel
bool FastRobotControlApp::onMouseWheelScrollEvent(double xOffset, double yOffset) {
	if (shouldShowSimWindow() && (simWindow->isActive() || simWindow->mouseIsWithinWindow(GlobalMouseState::lastMouseX, GlobalMouseState::lastMouseY)))
		if (simWindow->onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	if (shouldShowMOPTWindow() && (moptWindow->isActive() || moptWindow->mouseIsWithinWindow(GlobalMouseState::lastMouseX, GlobalMouseState::lastMouseY)))
		if (moptWindow->onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	if (GLApplication::onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	return false;
}

bool FastRobotControlApp::onKeyEvent(int key, int action, int mods) {

	if (shouldShowMOPTWindow() && moptWindow->isActive()){
		moptWindow->onKeyEvent(key, action, mods);
	}

	if (moptWindow->locomotionManager && moptWindow->locomotionManager->motionPlan) {
		if (key == GLFW_KEY_UP && action == GLFW_PRESS)
			moptWindow->forwardSpeedTarget += 0.1;
		if (key == GLFW_KEY_DOWN && action == GLFW_PRESS)
			moptWindow->forwardSpeedTarget -= 0.1;
		if (key == GLFW_KEY_LEFT && action == GLFW_PRESS)
			moptWindow->sidewaysSpeedTarget += 0.1;
		if (key == GLFW_KEY_RIGHT && action == GLFW_PRESS)
			moptWindow->sidewaysSpeedTarget -= 0.1;
		if (key == GLFW_KEY_PERIOD && action == GLFW_PRESS)
			moptWindow->turningSpeedTarget += 0.1;
		if (key == GLFW_KEY_SLASH && action == GLFW_PRESS)
			moptWindow->turningSpeedTarget -= 0.1;

		if (key == GLFW_KEY_O && action == GLFW_PRESS)
			moptWindow->locomotionManager->motionPlan->writeRobotMotionAnglesToFile("../out/tmpMPAngles.mpa");
	}

	if (key == GLFW_KEY_T && action == GLFW_PRESS) {
		createRobotFromCurrentDesign();
	}

	if (key == GLFW_KEY_Y && action == GLFW_PRESS) {
		loadFile("..\\data\\RobotDesigner\\SpotMiniDemo.batch");
	}

	if (key == GLFW_KEY_M && action == GLFW_PRESS) {
		warmStartMOPT(true);
	}

	if (key == GLFW_KEY_F1 && action == GLFW_PRESS) {
		viewOptions = SIM_WINDOW_ONLY;
		setupWindows();
	}
	if (key == GLFW_KEY_F2 && action == GLFW_PRESS) {
		viewOptions = MOPT_WINDOW_ONLY;
		setupWindows();
	}
	if (key == GLFW_KEY_F3 && action == GLFW_PRESS) {
		viewOptions = SIM_AND_MOPT_WINDOWS;
		setupWindows();
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

bool FastRobotControlApp::onCharacterPressedEvent(int key, int mods) {
	if (GLApplication::onCharacterPressedEvent(key, mods)) return true;
	return false;
}

void FastRobotControlApp::loadFile(const char* fName) {
	Logger::consolePrint("Loading file \'%s\'...\n", fName);
	std::string fileName;
	fileName.assign(fName);

	std::string fNameExt = fileName.substr(fileName.find_last_of('.') + 1);

	if (fNameExt.compare("batch") == 0) {
		Logger::consolePrint("Batch loading from \'%s\'\n", fName);
		FILE* fp = fopen(fName, "r");

		while (!feof(fp)){
			char line[200];
			readValidLine(line, 200, fp);
			char token[100], argument[100];
			sscanf(line, "%s %s", &token, &argument);
			if (strcmp(token, "load") == 0)
				loadFile(argument);
			if (strcmp(token, "toSim") == 0)
				createRobotFromCurrentDesign();
			if (strcmp(token, "end") == 0)
				break;
		}
		fclose(fp);
	}

	if (fNameExt.compare("pololu") == 0) {
		Logger::consolePrint("Loading servomotor mapping/calibration file '%s'\n", fName);
		if (simWindow && simWindow->pololuMaestroController)
			simWindow->pololuMaestroController->readRobotMappingParametersFromFile(fName);
		return;
	}

	if (fNameExt.compare("rs") == 0) {
		Logger::consolePrint("Load robot state from '%s'\n", fName);
		if (robot) {
			robot->loadReducedStateFromFile(fName);
			startingRobotState = RobotState(robot);
		}
		return;
	}

	if (fNameExt.compare("rbs") == 0 ){ 
		robot = simWindow->loadRobot(fName);

		startingRobotState = RobotState(robot);
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
			moptWindow->locomotionManager->motionPlan->syncMotionPlanWithFootFallPattern(moptWindow->footFallPattern);

			moptWindow->footFallPattern.writeToFile("..\\out\\tmpFFP.ffp");
			moptWindow->syncMOPTWindowParameters();
			moptWindow->printCurrentObjectiveValues();
		}
		return;
	}

}

void FastRobotControlApp::createRobotFromCurrentDesign() {
	if (robot)
		loadToSim(false);
}

void FastRobotControlApp::loadToSim(bool initializeMOPT){
/* ------ load an initial motion plan for the robot ------*/

//now, start MOPT...
	Logger::consolePrint("MoptWindow loading robot...\n");
	moptWindow->loadRobot(robot);
	if (initializeMOPT)
		Logger::consolePrint("..... successful.\n Warmstarting...\n");
	warmStartMOPT(initializeMOPT);
	if (initializeMOPT)
		Logger::consolePrint("Warmstart successful...\n");
	Logger::consolePrint("The robot has %d legs, weighs %lf kgs and is %lf m tall...\n", robot->bFrame->limbs.size(), robot->getMass(), robot->root->getCMPosition().y());

//	CreateParametersDesignWindow();
}


void FastRobotControlApp::exportMeshes() {
	if (!robot) {
		Logger::consolePrint("A robot must first be loaded before meshes can be exported...\n");
		return;
	}

	RobotState rs(robot);

	robot->setState(&startingRobotState);
	robot->renderMeshesToFile("..\\out\\robotMeshes.obj");
	Logger::consolePrint("Exported robot meshes to \'..\\out\\robotMeshes.obj\'\n");

	robot->setState(&rs);
}

void FastRobotControlApp::warmStartMOPT(bool initializeMotionPlan) {
	if (!robot) {
		Logger::consolePrint("Please load a robot first...\n");
		return;
	}

	if (!moptWindow || !simWindow)
		return;

	//reset the state of the robot, to make sure we're always starting from the same configuration
	robot->setState(&startingRobotState);

	moptWindow->initializeNewMP(initializeMotionPlan);
	simWindow->loadMotionPlan(moptWindow->locomotionManager->motionPlan);

	if(moptWindow->locomotionManager->energyFunction && doMotionAnalysis){
		motionPlanAnalysis->updateFromMotionPlan(moptWindow->locomotionManager->motionPlan);
	}
}

void FastRobotControlApp::saveFile(const char* fName) {
	Logger::consolePrint("SAVE FILE: Do not know what to do with file \'%s\'\n", fName);
}

void FastRobotControlApp::runMOPTStep() {
	double energyVal = moptWindow->runMOPTStep();

/*
	static int count = 0;
	Logger::log2Print("%lf\n", energyVal);
	count++;
	if (count > 400)
		exit(0);
*/
}

P3D FastRobotControlApp::getCameraTarget() {
	if (robot)
		return robot->root->getCMPosition();
	else
		return P3D(0, 1, 0);
}

void FastRobotControlApp::setActiveController() {
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
void FastRobotControlApp::process() {
	double dt = 1.0 / desiredFrameRate;

	if (slowMo)
		dt /= slowMoFactor;
	time += dt;
	phase += dt / moptWindow->locomotionManager->motionPlan->motionPlanDuration;
	double dPhase = 1.0 / (moptWindow->locomotionManager->motionPlan->nSamplePoints - 1);
	int n = 1;
	if (phase > n * dPhase) {
		phase -= n * dPhase;
		RobotState plannedRobotState = moptWindow->fmpp->getRobotStateAtTime(time);
		robot->setState(&plannedRobotState);
		moptWindow->advanceMotionPlanGlobalTime(n);
		moptWindow->generateMotionPreplan();
	}

	return;

	//we need to sync the state of the robot with the motion plan when we first start physics-based tracking...
	static int lastRunOptionSelected = runOption + 1;
	setActiveController();

	if (lastRunOptionSelected != runOption && (runOption != MOTION_PLAN_OPTIMIZATION)) {
		Logger::consolePrint("Syncronizing robot state\n");
		simWindow->getActiveController()->initialize();
		simWindow->getActiveController()->setDebugMode(doDebug);
		lastRunOptionSelected = runOption;
	}

	auto DoMOPTStep = [&]() {
		runMOPTStep();
		if (motionPlanAnalysis->window->visible() && doMotionAnalysis)
			motionPlanAnalysis->updateFromMotionPlan(moptWindow->locomotionManager->motionPlan);
	};

	if (runOption != MOTION_PLAN_OPTIMIZATION && simWindow->getActiveController()) {
//		if (optimizeWhileAnimating)
//			DoMOPTStep();

		double dt = 1.0 / desiredFrameRate;
		if (slowMo) 
			dt /= slowMoFactor;

		bool motionPhaseReset = simWindow->advanceSimulation(dt);
		if (motionPhaseReset)
			walkCycleIndex++;
	}
	else if (runOption == MOTION_PLAN_OPTIMIZATION)
		DoMOPTStep();

	if (simWindow->getActiveController())
		moptWindow->ffpViewer->cursorPosition = simWindow->getActiveController()->stridePhase;
}

// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
void FastRobotControlApp::drawScene() {
//	moptWindow->generateMotionPreplan();
}

// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
void FastRobotControlApp::drawAuxiliarySceneInfo() {
	if (shouldShowMOPTWindow()) {
		if (appIsRunning)
			moptWindow->ffpViewer->cursorPosition = phase;

		moptWindow->setAnimationParams(moptWindow->ffpViewer->cursorPosition, walkCycleIndex);
		moptWindow->draw();
		moptWindow->drawAuxiliarySceneInfo();
	}

	if (shouldShowSimWindow()) {
		if (followCameraTarget)
			simWindow->getCamera()->followTarget(getCameraTarget());

		simWindow->draw();
		simWindow->drawAuxiliarySceneInfo();
	}

	if(motionPlanAnalysis->window->visible()){
		motionPlanAnalysis->setTimeAt((float)moptWindow->ffpViewer->cursorPosition);
	}
}

// Restart the application.
void FastRobotControlApp::restart() {

}

bool FastRobotControlApp::processCommandLine(const std::string& cmdLine) {
	if (GLApplication::processCommandLine(cmdLine)) return true;
	return false;
}

