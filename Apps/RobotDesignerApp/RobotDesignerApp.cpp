#include <GUILib/GLUtils.h>

#include "RobotDesignerApp.h"
#include <GUILib/GLMesh.h>
#include <GUILib/GLContentManager.h>
#include <MathLib/MathLib.h>
#include <ControlLib/SimpleLimb.h>
#include <RobotDesignerLib/IntelligentRobotEditingWindow.h>

RobotDesignerApp::RobotDesignerApp(){
	bgColorR = bgColorG = bgColorB = bgColorA = 1;
	setWindowTitle("RobotDesigner");

	showGroundPlane = false;

	moptWindow = new MOPTWindow(0, 0, 100, 100, this);
	simWindow = new SimWindow(0, 0, 100, 100, this);
	iEditWindow = new IntelligentRobotEditingWindow(0, 0, 100, 100, this);
	motionPlanAnalysis = new MotionPlanAnalysis(menuScreen);
	energyWindow = new EnergyWindow();

	mainMenu->addGroup("RobotDesigner Options");
	mainMenu->addVariable("Run Mode", runOption, true)->setItems({ "MOPT", "Play", "SimPD", "SimTau"});
	mainMenu->addVariable<RD_VIEW_OPTIONS>("View Mode",
		[this](const RD_VIEW_OPTIONS &val) {viewOptions = val; setupWindows(); },
		[this]() {return viewOptions;},
		true)->setItems({ "Sim Only", "Sim+MOPT", "Sim+Design", "MOPT+iEdit" });

	nanogui::Widget *tools = new nanogui::Widget(mainMenu->window());
	mainMenu->addWidget("", tools);
	tools->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
		nanogui::Alignment::Middle, 0, 4));

	nanogui::Button* button;

	button = new nanogui::Button(tools, "");
	button->setIcon(ENTYPO_ICON_SAVE);
	button->setCallback([this]() { if (designWindow) designWindow->saveFile("../out/tmpModularRobotDesign.dsn"); });
	button->setTooltip("Quick Save");

	button = new nanogui::Button(tools, "");
	button->setIcon(ENTYPO_ICON_DOWNLOAD);
	button->setCallback([this]() { if (designWindow) designWindow->loadDesignFromFile("../out/tmpModularRobotDesign.dsn"); });
	button->setTooltip("Quick Load");

	button = new nanogui::Button(tools, "ToSim");
	button->setCallback([this]() { createRobotFromCurrentDesign(); });
	button->setTooltip("Load Robot Design To Sim");

	button = new nanogui::Button(tools, "GoMOPT");
	button->setCallback([this]() { warmStartMOPT(true); });
	button->setTooltip("Warmstart MOPT");

	{
		using namespace nanogui;

		mainMenu->addGroup("Toggle Windows");

		Widget *widget = new Widget(mainMenu->window());
		mainMenu->addWidget("", widget);
		widget->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 4));

		// button to toggle motion plan analysis
		Button *mpaButton = new Button(widget, "Motion Plan Analysis");
		mpaButton->setFontSize(14);
		mpaButton->setFlags(Button::Flags::ToggleButton);
		mpaButton->setChangeCallback([this](bool state){
			motionPlanAnalysis->window->setVisible(state);
			if(moptWindow->locomotionManager->motionPlan)
				motionPlanAnalysis->updateFromMotionPlan(moptWindow->locomotionManager->motionPlan);
		});

		// button to toggle energy window
		Button *button = new Button(widget, "Energy Window");
		button->setFontSize(14);
		button->setFlags(Button::Flags::ToggleButton);
		button->setChangeCallback([this](bool state){
			if(moptWindow->locomotionManager->energyFunction){
				if(state)
					energyWindow->createEnergyMenu(moptWindow->locomotionManager->energyFunction, menuScreen);
				energyWindow->updateEnergiesWith(moptWindow->locomotionManager->energyFunction, moptWindow->locomotionManager->motionPlan->getMPParameters());
				energyWindow->setVisible(state);
			}
		});
	}

	mainMenu->addGroup("MOPT Options");
	moptWindow->addMenuItems();

	mainMenu->addGroup("Sim Options");
	simWindow->addMenuItems();

	showGroundPlane = false;
	bgColorR = bgColorG = bgColorB = 0.75;

#ifdef START_WITH_VISUAL_DESIGNER
	designWindow = new ModularDesignWindow(0, 0, 100, 100, this, "../data/robotDesigner/configXM-430-V1.cfg");
	
#else
    loadFile("../data/robotsAndMotionPlans/spotMini/robot2.rbs");
    loadFile("../data/robotsAndMotionPlans/spotMini/robot.rs");
//	loadToSim();
	loadToSim(false);
    loadFile("../data/robotsAndMotionPlans/spotMini/trot4.p");
#endif

	menuScreen->performLayout();
	setupWindows();

	/*
	TwAddSeparator(mainMenuBar, "sep2", "");

	drawCDPs = false;
	drawSkeletonView = false;
	showGroundPlane = false;

	TwAddVarRW(mainMenuBar, "do debug", TW_TYPE_BOOLCPP, &doDebug, " label='doDebug' group='Viz2'");

	TwAddSeparator(mainMenuBar, "sep3", "");

	TwAddSeparator(mainMenuBar, "sep4", "");
	*/
	followCameraTarget = true;
}

void RobotDesignerApp::setupWindows() {
	int w, h;

	int offset = (int)(mainMenu->window()->width() * menuScreen->pixelRatio());

	w = (GLApplication::getMainWindowWidth()) - offset;
	h = GLApplication::getMainWindowHeight();

	iEditWindow->hideMenu();

	if (viewOptions == SIM_AND_MOPT){
		moptWindow->setViewportParameters(offset, 0, w / 2, h);
		moptWindow->ffpViewer->setViewportParameters(offset, 0, w/2, h/4);

		consoleWindow->setViewportParameters(offset + w / 2, 0, w / 2, 280);
		simWindow->setViewportParameters(offset + w / 2, 0, w / 2, h);
	}
	else if (viewOptions == SIM_AND_DESIGN && designWindow){
		designWindow->setViewportParameters(offset, 0, w / 2, h);
		designWindow->componentLibrary->setViewportParameters(offset, (int)(h * 3.0 / 4), w / 2, h / 4);

		consoleWindow->setViewportParameters(offset + w / 2, 0, w / 2, 280);
		simWindow->setViewportParameters(offset + w / 2, 0, w / 2, h);
	} else if (viewOptions == MOPT_AND_IEDIT && iEditWindow) {
		moptWindow->setViewportParameters(offset, 0, w / 2, h);
		moptWindow->ffpViewer->setViewportParameters(offset, 0, w / 2, h / 4);

		consoleWindow->setViewportParameters(offset + w / 2, 0, w / 2, 280);
		iEditWindow->setViewportParameters(offset + w / 2, 0, w / 2, h);
		iEditWindow->showMenu();
	}
	else {
		consoleWindow->setViewportParameters(offset, 0, w, 280);
		simWindow->setViewportParameters(offset, 0, w, h);
	}
}

RobotDesignerApp::~RobotDesignerApp(void){
}

bool RobotDesignerApp::shouldShowSimWindow() {
	return viewOptions != MOPT_AND_IEDIT && simWindow;
}

bool RobotDesignerApp::shouldShowMOPTWindow() {
	return (viewOptions == SIM_AND_MOPT || viewOptions == MOPT_AND_IEDIT) && moptWindow;
}

bool RobotDesignerApp::shouldShowIEditWindow() {
	return (viewOptions == MOPT_AND_IEDIT) && iEditWindow;
}

bool RobotDesignerApp::shouldShowDesignWindow() {
	return (viewOptions == SIM_AND_DESIGN) && designWindow;
}

//triggered when mouse moves
bool RobotDesignerApp::onMouseMoveEvent(double xPos, double yPos) {
	if (shouldShowSimWindow() && (simWindow->isActive() || simWindow->mouseIsWithinWindow(xPos, yPos)))
		if (simWindow->onMouseMoveEvent(xPos, yPos)) return true;

	if (shouldShowMOPTWindow() && (moptWindow->isActive() || moptWindow->mouseIsWithinWindow(xPos, yPos)))
		if (moptWindow->onMouseMoveEvent(xPos, yPos)) return true;

	if (shouldShowDesignWindow() && (designWindow->isActive() || designWindow->mouseIsWithinWindow(xPos, yPos)))
		if (designWindow->onMouseMoveEvent(xPos, yPos)) return true;

	if (shouldShowIEditWindow() && (iEditWindow->isActive() || iEditWindow->mouseIsWithinWindow(xPos, yPos)))
		if (iEditWindow->onMouseMoveEvent(xPos, yPos)) return true;

	if (GLApplication::onMouseMoveEvent(xPos, yPos)) return true;

	return false;
}

//triggered when mouse buttons are pressed
bool RobotDesignerApp::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
	if (shouldShowSimWindow() && (simWindow->isActive() || simWindow->mouseIsWithinWindow(xPos, yPos)))
		if (simWindow->onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;

	if (shouldShowMOPTWindow() && (moptWindow->isActive() || moptWindow->mouseIsWithinWindow(xPos, yPos)))
		if (moptWindow->onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;

	if (shouldShowDesignWindow() && (designWindow->isActive() || designWindow->mouseIsWithinWindow(xPos, yPos)))
		if (designWindow->onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;

	if (shouldShowIEditWindow() && (iEditWindow->isActive() || iEditWindow->mouseIsWithinWindow(xPos, yPos)))
		if (iEditWindow->onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;

	if (GLApplication::onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;

	return false;
}

//triggered when using the mouse wheel
bool RobotDesignerApp::onMouseWheelScrollEvent(double xOffset, double yOffset) {
	if (shouldShowSimWindow() && (simWindow->isActive() || simWindow->mouseIsWithinWindow(GlobalMouseState::lastMouseX, GlobalMouseState::lastMouseY)))
		if (simWindow->onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	if (shouldShowMOPTWindow() && (moptWindow->isActive() || moptWindow->mouseIsWithinWindow(GlobalMouseState::lastMouseX, GlobalMouseState::lastMouseY)))
		if (moptWindow->onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	if (shouldShowDesignWindow() && (designWindow->isActive() || designWindow->mouseIsWithinWindow(GlobalMouseState::lastMouseX, GlobalMouseState::lastMouseY)))
		if (designWindow->onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	if (shouldShowIEditWindow() && (iEditWindow->isActive() || iEditWindow->mouseIsWithinWindow(GlobalMouseState::lastMouseX, GlobalMouseState::lastMouseY)))
		if (iEditWindow->onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	if (GLApplication::onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	return false;
}

bool RobotDesignerApp::onKeyEvent(int key, int action, int mods) {
	if (viewOptions == SIM_AND_DESIGN && designWindow) {
		designWindow->onKeyEvent(key, action, mods);
	}

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

		boundToRange(&moptWindow->moptParams.desTravelDistZ, -1.5, 1.5);
		boundToRange(&moptWindow->moptParams.desTravelDistX, -1.5, 1.5);
		boundToRange(&moptWindow->moptParams.desTurningAngle, -1.5, 1.5);
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

	if (key == GLFW_KEY_A && action == GLFW_PRESS)
		optimizeWhileAnimating = !optimizeWhileAnimating;
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
			if (prd)
				prd->updateMorphology();
		}
		return;
	}

	if (fNameExt.compare("rbs") == 0 ){ 
		robot = simWindow->loadRobot(fName);

		delete prd;
		prd = new SymmetricParameterizedRobotDesign(robot);
//		CreateParametersDesignWindow();
//		menuScreen->performLayout();
//		slidervalues.resize(prd->getNumberOfParameters());
//		slidervalues.setZero();

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

	if (fNameExt.compare("dsn") == 0 && designWindow) {
		Logger::consolePrint("Load robot state from '%s'\n", fName);
		designWindow->loadFile(fName);
		return;
	}

}

void RobotDesignerApp::createRobotFromCurrentDesign() {
	if (designWindow) {
		designWindow->saveToRBSFile("../out/tmpRobot.rbs");
		loadFile("../out/tmpRobot.rbs");
		delete initialRobotState;
		initialRobotState = new ReducedRobotState(robot);
		robot->populateState(initialRobotState, true);
		robot->setState(initialRobotState);
		if (prd)
			prd->updateMorphology();
	}

	if (robot)
		loadToSim(false);
}

void RobotDesignerApp::loadToSim(bool initializeMOPT){
/* ------ load an initial motion plan for the robot ------*/

//now, start MOPT...
	Logger::consolePrint("MoptWindow loading robot...\n");
	moptWindow->loadRobot(robot, initialRobotState);
	Logger::consolePrint("..... successful.\n Warmstarting...\n");
	warmStartMOPT(initializeMOPT);
	Logger::consolePrint("Warmstart successful...\n");
	Logger::consolePrint("The robot has %d legs, weighs %lfkgs and is %lfm tall...\n", robot->bFrame->limbs.size(), robot->getMass(), robot->root->getCMPosition().y());

//	CreateParametersDesignWindow();
}

void RobotDesignerApp::warmStartMOPT(bool initializeMotionPlan) {
	moptWindow->initializeNewMP(initializeMotionPlan);
	simWindow->loadMotionPlan(moptWindow->locomotionManager->motionPlan);
	motionPlanAnalysis->updateFromMotionPlan(moptWindow->locomotionManager->motionPlan);

	if(moptWindow->locomotionManager->energyFunction)
	{
		energyWindow->createEnergyMenu(moptWindow->locomotionManager->energyFunction, menuScreen);
		energyWindow->updateEnergiesWith(moptWindow->locomotionManager->energyFunction, moptWindow->locomotionManager->motionPlan->getMPParameters());
	}
}

void RobotDesignerApp::saveFile(const char* fName) {
	Logger::consolePrint("SAVE FILE: Do not know what to do with file \'%s\'\n", fName);
}

void RobotDesignerApp::runMOPTStep() {
	double energyVal = moptWindow->runMOPTStep();

/*
	static int count = 0;
	Logger::log2Print("%lf\n", energyVal);
	count++;
	if (count > 400)
		exit(0);
*/
}

P3D RobotDesignerApp::getCameraTarget() {
	if (robot)
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

	auto DoMOPTStep = [&]() {
		runMOPTStep();
		if (motionPlanAnalysis->window->visible())
			motionPlanAnalysis->updateFromMotionPlan(moptWindow->locomotionManager->motionPlan);
		if(moptWindow->locomotionManager->energyFunction)
			energyWindow->updateEnergiesWith(moptWindow->locomotionManager->energyFunction, moptWindow->locomotionManager->motionPlan->getMPParameters());
	};

	if (runOption != MOTION_PLAN_OPTIMIZATION && simWindow->getActiveController()) {
		if (optimizeWhileAnimating)
			DoMOPTStep();

		double simulationTime = 0;
		double maxRunningTime = 1.0 / desiredFrameRate;

		while (simulationTime / maxRunningTime < animationSpeedupFactor) {
			simulationTime += simWindow->simTimeStep;

//			update wheel rotations here...

			simWindow->step();

			if (slowMo)
				break;
		}

	}
	else if (runOption == MOTION_PLAN_OPTIMIZATION)
		DoMOPTStep();

	if (simWindow->getActiveController())
		moptWindow->ffpViewer->cursorPosition = simWindow->getActiveController()->stridePhase;
}

// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
void RobotDesignerApp::drawScene() {

}

// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
void RobotDesignerApp::drawAuxiliarySceneInfo() {
	if (shouldShowMOPTWindow()) {
		moptWindow->setAnimationParams(moptWindow->ffpViewer->cursorPosition, 0);
		moptWindow->draw();
		moptWindow->drawAuxiliarySceneInfo();
	}

	if (shouldShowDesignWindow()) {
		designWindow->draw();
		designWindow->drawAuxiliarySceneInfo();
	}

	if (shouldShowSimWindow()) {
		if (followCameraTarget)
			simWindow->getCamera()->followTarget(getCameraTarget());

		simWindow->draw();
		simWindow->drawAuxiliarySceneInfo();
	}

	if (shouldShowIEditWindow()) {
		iEditWindow->draw();
		iEditWindow->drawAuxiliarySceneInfo();
	}

	if(motionPlanAnalysis->window->visible())
	{
		motionPlanAnalysis->setTimeAt((float)moptWindow->ffpViewer->cursorPosition);
	}
}

// Restart the application.
void RobotDesignerApp::restart() {

}

bool RobotDesignerApp::processCommandLine(const std::string& cmdLine) {
	if (GLApplication::processCommandLine(cmdLine)) return true;
	return false;
}

