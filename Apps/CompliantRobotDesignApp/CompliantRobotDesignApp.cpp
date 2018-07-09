#include <GUILib/GLUtils.h>

#include "CompliantRobotDesignApp.h"
#include <GUILib/GLMesh.h>
#include <GUILib/GLContentManager.h>
#include <MathLib/MathLib.h>
#include <ControlLib/SimpleLimb.h>


CompliantRobotDesignApp::CompliantRobotDesignApp(){
	bgColorR = bgColorG = bgColorB = bgColorA = 1;
	setWindowTitle("CompliantRobotDesigner");

	showGroundPlane = false;

	designWindow = new DesignWindow(0, 0, 100, 100, this);
	simWindow = new SimulationWindow(0, 0, 100, 100, this);

	nanogui::Widget *tools = new nanogui::Widget(mainMenu->window());
	mainMenu->addWidget("", tools);
	tools->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
		nanogui::Alignment::Middle, 0, 4));

	nanogui::Button* button;

	mainMenu->addGroup("MOPT Options");
	designWindow->addMenuItems();

	mainMenu->addGroup("Sim Options");
	simWindow->addMenuItems();

	showGroundPlane = false;
	bgColorR = bgColorG = bgColorB = 249.0 / 255.0;

	menuScreen->performLayout();
	setupWindows();

//	loadFile("..\\data\\RobotDesigner\\SpotMiniDemo.batch");

	followCameraTarget = true;
}

void CompliantRobotDesignApp::setupWindows() {
	int w, h;

	int offset = (int)(mainMenu->window()->width() * menuScreen->pixelRatio());

	w = (GLApplication::getMainWindowWidth()) - offset;
	h = GLApplication::getMainWindowHeight();

	if (viewOptions == SIM_AND_DESIGN_WINDOWS) {
		designWindow->setViewportParameters(offset, 0, w / 2, h);
		consoleWindow->setViewportParameters(offset + w / 2, 0, w / 2, 280);
		simWindow->setViewportParameters(offset + w / 2, 0, w / 2, h);
		showConsole = true;
	} else if (viewOptions == SIM_WINDOW_ONLY) {
		consoleWindow->setViewportParameters(offset, 0, w, 280);
		simWindow->setViewportParameters(offset, 0, w, h);
		showConsole = true;
	}
	else {
		designWindow->setViewportParameters(offset, 0, w, h);
		showConsole = false;
	}

}

CompliantRobotDesignApp::~CompliantRobotDesignApp(void){
}

bool CompliantRobotDesignApp::shouldShowSimWindow() {
	return viewOptions != DESIGN_WINDOW_ONLY;
}

bool CompliantRobotDesignApp::shouldShowDesignWindow() {
	return viewOptions != SIM_WINDOW_ONLY;
}

//triggered when mouse moves
bool CompliantRobotDesignApp::onMouseMoveEvent(double xPos, double yPos) {
	if (shouldShowSimWindow() && (simWindow->isActive() || simWindow->mouseIsWithinWindow(xPos, yPos)))
		if (simWindow->onMouseMoveEvent(xPos, yPos)) {
			return true;
		}

	if (shouldShowDesignWindow() && (designWindow->isActive() || designWindow->mouseIsWithinWindow(xPos, yPos)))
		if (designWindow->onMouseMoveEvent(xPos, yPos)) {
			return true;
		}

	if (GLApplication::onMouseMoveEvent(xPos, yPos)) return true;

	return false;
}

//triggered when mouse buttons are pressed
bool CompliantRobotDesignApp::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
	if (shouldShowSimWindow() && (simWindow->isActive() || simWindow->mouseIsWithinWindow(xPos, yPos)))
		if (simWindow->onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;

	if (shouldShowDesignWindow() && (designWindow->isActive() || designWindow->mouseIsWithinWindow(xPos, yPos)))
		if (designWindow->onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;

	if (GLApplication::onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;

	return false;
}

//triggered when using the mouse wheel
bool CompliantRobotDesignApp::onMouseWheelScrollEvent(double xOffset, double yOffset) {
	if (shouldShowSimWindow() && (simWindow->isActive() || simWindow->mouseIsWithinWindow(GlobalMouseState::lastMouseX, GlobalMouseState::lastMouseY)))
		if (simWindow->onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	if (shouldShowDesignWindow() && (designWindow->isActive() || designWindow->mouseIsWithinWindow(GlobalMouseState::lastMouseX, GlobalMouseState::lastMouseY)))
		if (designWindow->onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	if (GLApplication::onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	return false;
}

bool CompliantRobotDesignApp::onKeyEvent(int key, int action, int mods) {
	if (shouldShowDesignWindow() && designWindow->isActive()){
		designWindow->onKeyEvent(key, action, mods);
	}

	if (designWindow) {

	}

	if (key == GLFW_KEY_Y && action == GLFW_PRESS) {
		loadFile("..\\data\\RobotDesigner\\SpotMiniDemo.batch");
	}

	if (key == GLFW_KEY_F1 && action == GLFW_PRESS) {
		viewOptions = SIM_WINDOW_ONLY;
		setupWindows();
	}
	if (key == GLFW_KEY_F2 && action == GLFW_PRESS) {
		viewOptions = DESIGN_WINDOW_ONLY;
		setupWindows();
	}
	if (key == GLFW_KEY_F3 && action == GLFW_PRESS) {
		viewOptions = SIM_AND_DESIGN_WINDOWS;
		setupWindows();
	}

	mainMenu->refresh();

	if (GLApplication::onKeyEvent(key, action, mods)) return true;

	return false;
}

bool CompliantRobotDesignApp::onCharacterPressedEvent(int key, int mods) {
	if (GLApplication::onCharacterPressedEvent(key, mods)) return true;
	return false;
}

void CompliantRobotDesignApp::loadFile(const char* fName) {
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
				loadToSim();
			if (strcmp(token, "end") == 0)
				break;
		}
		fclose(fp);
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

}

void CompliantRobotDesignApp::loadToSim(){
/* ------ load an initial motion plan for the robot ------*/
	Logger::consolePrint("designWindow loading robot...\n");
	//reset the state of the robot, to make sure we're always starting from the same configuration
	robot->setState(&startingRobotState);
}

void CompliantRobotDesignApp::saveFile(const char* fName) {
	Logger::consolePrint("SAVE FILE: Do not know what to do with file \'%s\'\n", fName);
}

P3D CompliantRobotDesignApp::getCameraTarget() {
	if (robot)
		return robot->root->getCMPosition();
	else
		return P3D(0, 1, 0);
}

// Run the App tasks
void CompliantRobotDesignApp::process() {
	double dt = controlFrequency;

	double stepTime = 0;

	while (stepTime < 1.0 / desiredFrameRate) {
		stepTime += dt;
		globalTime += dt;
		simWindow->advanceSimulation(dt);
	}
}

// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
void CompliantRobotDesignApp::drawScene() {
//	designWindow->generateMotionPreplan();
}

// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
void CompliantRobotDesignApp::drawAuxiliarySceneInfo() {
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
}

// Restart the application.
void CompliantRobotDesignApp::restart() {

}

bool CompliantRobotDesignApp::processCommandLine(const std::string& cmdLine) {
	if (GLApplication::processCommandLine(cmdLine)) return true;
	return false;
}
