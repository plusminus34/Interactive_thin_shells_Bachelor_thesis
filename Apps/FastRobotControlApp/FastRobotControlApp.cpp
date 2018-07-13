#include <GUILib/GLUtils.h>

#include "FastRobotControlApp.h"
#include <GUILib/GLMesh.h>
#include <GUILib/GLContentManager.h>
#include <MathLib/MathLib.h>
#include <ControlLib/SimpleLimb.h>
#include "MotionPlanner.h"

//- change the stride duration. 0.8 is quite long...
//- try a few different footfall patterns (walking, flying trot, pacing)
//- optimize foot positions too? And maybe a shorter horizon in MOPT?

//should we use the old MOPT plan to initialize the new one?!? It just needs to be shifted over by a number of steps, which we know already...
//there might be drift due to the global motion time... perhaps re-sync?

//#define DEBUG_MOPT

//#define PRINT_TRACKING_DEBUG

//work towards:
//  - implement some graceful failure mode, when the robot gets too far from the plan, or when the plan generation seems to fail...
//	- validate on a quadruped, and on a cassie-like biped
//	- long horizon motion plans that include a change in gait, legs being used as arms, etc
//	- mopt that combines locomotion and manipulation
//	- MOPT for multi-robot systems, including locomotion and manipulation
//	- interactive MOPT - add an obstacle in the scene, given current state of robot it figures out how to avoid it, etc.
//	- gait transitions


FastRobotControlApp::FastRobotControlApp(){
	bgColorR = bgColorG = bgColorB = bgColorA = 1;
	setWindowTitle("RobotDesigner");

	showGroundPlane = false;

	plannerWindow = new MotionPlannerWindow(0, 0, 100, 100, this);
	simWindow = new SimulationWindow(0, 0, 100, 100, this);

	mainMenu->addGroup("Options");
	mainMenu->addVariable("Run Mode", runOption, true)->setItems({ "Playback", "Tracking", "No controller"});

	mainMenu->addVariable("Pause After Replan", breakAfterReplan);

	nanogui::Widget *tools = new nanogui::Widget(mainMenu->window());
	mainMenu->addWidget("", tools);
	tools->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
		nanogui::Alignment::Middle, 0, 4));

	nanogui::Button* button;

	button = new nanogui::Button(tools, "goMOPT");
	button->setFontSize(14);
	button->setCallback([this]() {
		plannerWindow->motionPlanner->generateMotionPlan();
	});

	button = new nanogui::Button(tools, "optMOPT");
	button->setFontSize(14);
	button->setCallback([this]() {
		plannerWindow->motionPlanner->optimizeMotionPlan();
	});

	mainMenu->addGroup("MOPT Options");
	plannerWindow->addMenuItems();

	mainMenu->addGroup("Sim Options");
	simWindow->addMenuItems();

	showGroundPlane = false;
	bgColorR = bgColorG = bgColorB = 249.0 / 255.0;

	menuScreen->performLayout();
	setupWindows();

	loadFile("..\\data\\RobotDesigner\\SpotMiniTrot.batch");
	robot->forward = V3D(0, 0, 1);
	robot->right = V3D(-1, 0, 0);

	plannerWindow->motionPlanner->motionPlanDuration = 0.6;

#ifdef DEBUG_MOPT
	/**------DEBUG-------**/
	RobotState rs(robot);
/*
	rs.readFromFile("..\\out\\badStartingState.rs");
	plannerWindow->motionPlanner->motionPlanStartTime = 1.2;
	globalTime = 1.208333;
	simWindow->stridePhase = 0.010417;
	plannerWindow->motionPlanner->forwardSpeedTarget = 0.6;
	plannerWindow->motionPlanner->sidewaysSpeedTarget = 0;
	plannerWindow->motionPlanner->turningSpeedTarget = -0.1;
*/
	robot->setState(&rs);

#endif

	plannerWindow->motionPlanner->generateMotionPlan();

	followCameraTarget = true;
}

void FastRobotControlApp::setupWindows() {
	int w, h;

	int offset = (int)(mainMenu->window()->width() * menuScreen->pixelRatio());

	w = (GLApplication::getMainWindowWidth()) - offset;
	h = GLApplication::getMainWindowHeight();

	if (viewOptions == SIM_AND_MOPT_WINDOWS) {
		plannerWindow->setViewportParameters(offset, 0, w / 2, h);
		plannerWindow->ffpViewer->setViewportParameters(offset, 0, w / 2, h / 4);

		consoleWindow->setViewportParameters(offset + w / 2, 0, w / 2, 280);
		simWindow->setViewportParameters(offset + w / 2, 0, w / 2, h);
		showConsole = true;
	} else if (viewOptions == SIM_WINDOW_ONLY) {
		consoleWindow->setViewportParameters(offset, 0, w, 280);
		simWindow->setViewportParameters(offset, 0, w, h);
		showConsole = true;
	}
	else {
		plannerWindow->setViewportParameters(offset, 0, w, h);
		plannerWindow->ffpViewer->setViewportParameters(offset, 0, w, h / 4);
		showConsole = false;
	}

}

FastRobotControlApp::~FastRobotControlApp(void){
}

bool FastRobotControlApp::shouldShowSimWindow() {
	return viewOptions != MOPT_WINDOW_ONLY;
}

bool FastRobotControlApp::shouldShowPlannerWindow() {
	return viewOptions != SIM_WINDOW_ONLY;
}

//triggered when mouse moves
bool FastRobotControlApp::onMouseMoveEvent(double xPos, double yPos) {
	if (shouldShowSimWindow() && (simWindow->isActive() || simWindow->mouseIsWithinWindow(xPos, yPos)))
		if (simWindow->onMouseMoveEvent(xPos, yPos)) {
			return true;
		}

	if (shouldShowPlannerWindow() && (plannerWindow->isActive() || plannerWindow->mouseIsWithinWindow(xPos, yPos)))
		if (plannerWindow->onMouseMoveEvent(xPos, yPos)) {
			return true;
		}

	if (GLApplication::onMouseMoveEvent(xPos, yPos)) return true;

	return false;
}

//triggered when mouse buttons are pressed
bool FastRobotControlApp::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
	if (shouldShowSimWindow() && (simWindow->isActive() || simWindow->mouseIsWithinWindow(xPos, yPos)))
		if (simWindow->onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;

	if (shouldShowPlannerWindow() && (plannerWindow->isActive() || plannerWindow->mouseIsWithinWindow(xPos, yPos)))
		if (plannerWindow->onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;

	if (GLApplication::onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;

	return false;
}

//triggered when using the mouse wheel
bool FastRobotControlApp::onMouseWheelScrollEvent(double xOffset, double yOffset) {
	if (shouldShowSimWindow() && (simWindow->isActive() || simWindow->mouseIsWithinWindow(GlobalMouseState::lastMouseX, GlobalMouseState::lastMouseY)))
		if (simWindow->onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	if (shouldShowPlannerWindow() && (plannerWindow->isActive() || plannerWindow->mouseIsWithinWindow(GlobalMouseState::lastMouseX, GlobalMouseState::lastMouseY)))
		if (plannerWindow->onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	if (GLApplication::onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	return false;
}

bool FastRobotControlApp::onKeyEvent(int key, int action, int mods) {

	if (shouldShowPlannerWindow() && plannerWindow->isActive()){
		plannerWindow->onKeyEvent(key, action, mods);
	}

	if (plannerWindow) {
		if (key == GLFW_KEY_UP && action == GLFW_PRESS)
			plannerWindow->motionPlanner->forwardSpeedTarget += 0.1;
		if (key == GLFW_KEY_DOWN && action == GLFW_PRESS)
			plannerWindow->motionPlanner->forwardSpeedTarget -= 0.1;
		if (key == GLFW_KEY_LEFT && action == GLFW_PRESS)
			plannerWindow->motionPlanner->sidewaysSpeedTarget += 0.1;
		if (key == GLFW_KEY_RIGHT && action == GLFW_PRESS)
			plannerWindow->motionPlanner->sidewaysSpeedTarget -= 0.1;
		if (key == GLFW_KEY_PERIOD && action == GLFW_PRESS)
			plannerWindow->motionPlanner->turningSpeedTarget += 0.1;
		if (key == GLFW_KEY_SLASH && action == GLFW_PRESS)
			plannerWindow->motionPlanner->turningSpeedTarget -= 0.1;
	}

	if (key == GLFW_KEY_Y && action == GLFW_PRESS) {
		loadFile("..\\data\\RobotDesigner\\SpotMiniDemo.batch");
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
		runOption = MOTION_PLAN_PLAYBACK;
	if (key == GLFW_KEY_2 && action == GLFW_PRESS)
		runOption = MOTION_PLAN_TRACKING;

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

	if (fNameExt.compare("ffp") == 0) {
		plannerWindow->loadFFPFromFile(fName);
		return;
	}

	if (fNameExt.compare("p") == 0) {
		plannerWindow->loadMotionPlanFromFile(fName);
		return;
	}
}

void FastRobotControlApp::loadToSim(){
/* ------ load an initial motion plan for the robot ------*/
	Logger::consolePrint("plannerWindow loading robot...\n");
	plannerWindow->loadRobot(robot);
	//reset the state of the robot, to make sure we're always starting from the same configuration
	robot->setState(&startingRobotState);

	plannerWindow->initializeLocomotionEngine();
	startingRobotState = plannerWindow->motionPlanner->locomotionManager->motionPlan->initialRS;
	simWindow->loadMotionPlan(plannerWindow->motionPlanner);
	Logger::consolePrint("The robot has %d legs, weighs %lf kgs and is %lf m tall...\n", robot->bFrame->limbs.size(), robot->getMass(), robot->root->getCMPosition().y());
}

void FastRobotControlApp::saveFile(const char* fName) {
	Logger::consolePrint("SAVE FILE: Do not know what to do with file \'%s\'\n", fName);
}

P3D FastRobotControlApp::getCameraTarget() {
	if (robot)
		return robot->root->getCMPosition();
	else
		return P3D(0, 1, 0);
}

void FastRobotControlApp::setActiveController() {
	if (runOption == MOTION_PLAN_PLAYBACK)
		simWindow->setActiveController(simWindow->playbackController);
	else if (runOption == MOTION_PLAN_TRACKING)
		simWindow->setActiveController(simWindow->trackingController);
	else
		simWindow->setActiveController(NULL);
}

// Run the App tasks
void FastRobotControlApp::process() {
	if (plannerWindow->motionPlanner->initialized == false)
		plannerWindow->motionPlanner->generateMotionPlan();

	setActiveController();

	double dt = controlFrequency;

	double stepTime = 0;

	while (stepTime < 1.0 / desiredFrameRate) {
		stepTime += dt;
		globalTime += dt;
		simWindow->advanceSimulation(dt);


		double dPhase = 1.0 / (plannerWindow->motionPlanner->locomotionManager->motionPlan->nSamplePoints - 1);
		double timePerdPhase = plannerWindow->motionPlanner->locomotionManager->motionPlan->motionPlanDuration / (plannerWindow->motionPlanner->locomotionManager->motionPlan->nSamplePoints - 1);
		int n = 2;

		//we are using a discrete number of steps to keep the (discrete) footfall pattern consistent. Otherwise we'd need to interpolate between stance/swing phases and there are no good answers...
		if (simWindow->stridePhase > n * dPhase) {
#ifdef PRINT_TRACKING_DEBUG
/*
double p_heading = plannerWindow->motionPlanner->prePlanHeadingTrajectory.evaluate_linear(globalTime);
double p_turningSpeed = plannerWindow->motionPlanner->prePlanTurningSpeedTrajectory.evaluate_linear(globalTime);
double p_speedForward = plannerWindow->motionPlanner->prePlanBodyVelocityTrajectory.evaluate_linear(globalTime).z();
double p_posForward = plannerWindow->motionPlanner->prePlanBodyTrajectory.evaluate_linear(globalTime).z();
Logger::consolePrint("Target --> heading: %lf, turning speed: %lf, p.z: %lf, v.z: %lf, globalTime: %lf\n",
p_heading, p_turningSpeed, p_posForward, p_speedForward, globalTime);

//current quantities
RobotState currentRS(robot);
double c_heading = currentRS.getOrientation().getRotationAngle(Globals::worldUp);
double c_turningSpeed = currentRS.getAngularVelocity().dot(Globals::worldUp);
double c_speedForward = currentRS.getVelocity().z();
double c_posForward = currentRS.getPosition().z();
Logger::consolePrint("Current--> heading: %lf, turning speed: %lf, p.z: %lf, v.z: %lf, stridePhase: %lf\n",
c_heading, c_turningSpeed, c_posForward, c_speedForward, simWindow->stridePhase);

*/

			//planned qunatities
			P3D p_pos = plannerWindow->motionPlanner->prePlanBodyTrajectory.evaluate_linear(globalTime);
			P3D p_vel = plannerWindow->motionPlanner->prePlanBodyVelocityTrajectory.evaluate_linear(globalTime);
			double p_heading = plannerWindow->motionPlanner->prePlanHeadingTrajectory.evaluate_linear(globalTime);
			double p_turningSpeed = plannerWindow->motionPlanner->prePlanTurningSpeedTrajectory.evaluate_linear(globalTime);
			Logger::consolePrint("Target --> pos: %lf %lf %lf, vel: %lf %lf %lf, heading: %lf, turning speed: %lf\n",
				p_pos.x(), p_pos.y(), p_pos.z(),
				p_vel.x(), p_vel.y(), p_vel.z(),
				p_heading, p_turningSpeed);

			//current quantities
			RobotState currentRS(robot);
			P3D c_pos = currentRS.getPosition();
			P3D c_vel = currentRS.getVelocity();
			double c_heading = currentRS.getOrientation().getRotationAngle(Globals::worldUp);
			double c_turningSpeed = currentRS.getAngularVelocity().dot(Globals::worldUp);
			Logger::consolePrint("Current--> pos: %lf %lf %lf, vel: %lf %lf %lf, heading: %lf, turning speed: %lf\n",
				c_pos.x(), c_pos.y(), c_pos.z(),
				c_vel.x(), c_vel.y(), c_vel.z(),
				c_heading, c_turningSpeed);
#endif

			//restart the entire motion planning process...
			//double timeAtStrideStart = globalTime - simWindow->stridePhase * plannerWindow->motionPlanner->locomotionManager->motionPlan->motionPlanDuration;
			//simWindow->stridePhase -= n * dPhase;
			//plannerWindow->motionPlanner->motionPlanStartTime = timeAtStrideStart + n * timePerdPhase;
			simWindow->stridePhase = 0;
			plannerWindow->motionPlanner->motionPlanStartTime = globalTime;
			plannerWindow->motionPlanner->nTimeStepsForSync += n;
			plannerWindow->ffpViewer->cursorPosition = simWindow->stridePhase;

#ifdef DEBUG_MOPT
			plannerWindow->motionPlanner->locomotionManager->printDebugInfo = true;
			plannerWindow->motionPlanner->locomotionManager->checkDerivatives = true;
#else
			plannerWindow->motionPlanner->locomotionManager->printDebugInfo = false;
			plannerWindow->motionPlanner->locomotionManager->checkDerivatives = false;
#endif

			//hmmm, there will be a slight mismatch here. We are using the current state of the robot as initial conditions for motion planning, but the motion planning process starts "a little while back" when the stride phase was actually 0... but I guess we're within the resoultion of the timestep anyway...
			plannerWindow->motionPlanner->generateMotionPlan();

#ifdef DEBUG_MOPT
			RobotState rs(robot);
			rs.writeToFile("..\\out\\startingState.rs");
			Logger::consolePrint("motionPlanStartTime: %lf, global time: %lf, stridePhase: %lf, walking speed forward: %lf, walking speed sideways: %lf, turning speed: %lf\n", plannerWindow->motionPlanner->motionPlanStartTime, globalTime, simWindow->stridePhase, plannerWindow->motionPlanner->forwardSpeedTarget, plannerWindow->motionPlanner->sidewaysSpeedTarget, plannerWindow->motionPlanner->turningSpeedTarget);
#endif
			if (breakAfterReplan)
				appIsRunning = false;

		}
		if (slowMo)
			break;
	}

	plannerWindow->ffpViewer->cursorPosition = simWindow->stridePhase;

}

// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
void FastRobotControlApp::drawScene() {
//	plannerWindow->generateMotionPreplan();
}

// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
void FastRobotControlApp::drawAuxiliarySceneInfo() {
	if (shouldShowPlannerWindow()) {

		plannerWindow->setAnimationParams(plannerWindow->ffpViewer->cursorPosition, walkCycleIndex);

		if (followCameraTarget)
			plannerWindow->getCamera()->followTarget(getCameraTarget());

		plannerWindow->draw();
		plannerWindow->drawAuxiliarySceneInfo();
	}

	if (shouldShowSimWindow()) {
		if (followCameraTarget)
			simWindow->getCamera()->followTarget(getCameraTarget());

		simWindow->draw();
		simWindow->drawAuxiliarySceneInfo();
	}
}

// Restart the application.
void FastRobotControlApp::restart() {

}

bool FastRobotControlApp::processCommandLine(const std::string& cmdLine) {
	if (GLApplication::processCommandLine(cmdLine)) return true;
	return false;
}

