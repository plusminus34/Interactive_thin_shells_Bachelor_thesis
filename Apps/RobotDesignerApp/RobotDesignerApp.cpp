#include <GUILib/GLUtils.h>

#include "RobotDesignerApp.h"
#include <GUILib/GLMesh.h>
#include <GUILib/GLContentManager.h>
#include <MathLib/MathLib.h>
#include <ControlLib/SimpleLimb.h>

#define START_WITH_VISUAL_DESIGNER

//need a proper save/load routine: design, an entire robot, motion plan...

//make a bunch of robot templates:
//cassie
//three legged robot
//a strider robot
//a dog robot
//hexa-like robot
//probably need to work a bit on body shape/body characteristics

//can we go all the way to creating 3d printable geometry?

//tangentGRFBoundValues is useful for warmstarting to incrementally bound tangential forces. But need to implement proper friction, methinks...
//perhaps make the non-mesh version of the renderer prettier... it will allow us to test the code before (or even if it wont happen that) the visual designer part is integrated.
//everything needs to happen via rbs files. The visual designer will output an rbs file which gets loaded the normal way, then it knows how to sync back changes with the rbs, and that's all...
//the constraint system should be much more modular, with each type of MOPT adding its own constraints to a global list
//will need to add the option to save/load ffps, mopt, rbs/robotdesigner files, etc
//the wheel type model can also play a key role in removing the assumption of point feet. Can we model a human-like foot rolling? The COP can travel around during walking, and this can be modeled by having a wheel-like foot. Can this then, in conjunction with a MOPT QP tracker lead to natural toe-off motions?
//wheels can be locked to the feet (then you ask that the whole orientation of the foot matches that of the upper leg), or they can be moving independently, passive (GRF straight up only) or active...

//try a flying trot gait for the robot. Try also to match the new robot gait capabilities (Laikago) maybe with a slightly more conservative gait (some overlap in stance for all 4) as well as shorter gait duration overall
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

	showGroundPlane = false;

	moptWindow = new MOPTWindow(0, 0, 100, 100, this);
	simWindow = new SimWindow(0, 0, 100, 100, this);

	mainMenu->addGroup("RobotDesigner Options");
	mainMenu->addVariable("Run Mode", runOption, true)->setItems({ "MOPT", "Play", "SimPD", "SimTau"});
	mainMenu->addVariable("View Mode", viewOptions, true)->setItems({ "Sim Only", "MOPT", "Design"});
//	mainMenu->addButton("Warmstart MOPT", [this]() { warmStartMOPT(true); });
//	mainMenu->addButton("Load Robot Design", [this]() { createRobotFromCurrentDesign(); });

	mainMenu->addButton("Compute Jacobian", [this]() { test_dmdp_Jacobian(); });

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

	mainMenu->addGroup("MOPT Options");
	moptWindow->addMenuItems();

	mainMenu->addGroup("Sim Options");
	simWindow->addMenuItems();

	showGroundPlane = false;
	bgColor[0] = bgColor[1] = bgColor[2] = 0.75;

#ifdef START_WITH_VISUAL_DESIGNER
	designWindow = new ModularDesignWindow(0, 0, 100, 100, this, "../data/robotDesigner/configXM-430-V1.cfg");
	
#else
    loadFile("../data/robotsAndMotionPlans/spotMini/robot2.rbs");
    loadFile("../data/robotsAndMotionPlans/spotMini/robot.rs");
//	loadToSim();
	loadToSim(false);
    loadFile("../data/robotsAndMotionPlans/spotMini/trot.p");

	mainMenu->addGroup("Design Parameters");
	addDesignParameterSliders();

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
	} else {
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

	if (viewOptions == SIM_AND_MOPT)
		if (moptWindow->isActive() || moptWindow->mouseIsWithinWindow(xPos, yPos))
			if (moptWindow->onMouseMoveEvent(xPos, yPos)) return true;

	if (viewOptions == SIM_AND_DESIGN && designWindow)
		if (designWindow->isActive() || designWindow->mouseIsWithinWindow(xPos, yPos))
			if (designWindow->onMouseMoveEvent(xPos, yPos)) return true;

	if (GLApplication::onMouseMoveEvent(xPos, yPos)) return true;

	return false;
}

//triggered when mouse buttons are pressed
bool RobotDesignerApp::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
	if (simWindow->isActive() || simWindow->mouseIsWithinWindow(xPos, yPos))
		if (simWindow->onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;

	if (viewOptions == SIM_AND_MOPT)
		if (moptWindow->isActive() || moptWindow->mouseIsWithinWindow(xPos, yPos))
			if (moptWindow->onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;

	if (viewOptions == SIM_AND_DESIGN && designWindow)
		if (designWindow->isActive() || designWindow->mouseIsWithinWindow(xPos, yPos))
			if (designWindow->onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;

	if (GLApplication::onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;

	return false;
}

//triggered when using the mouse wheel
bool RobotDesignerApp::onMouseWheelScrollEvent(double xOffset, double yOffset) {
	if (simWindow->mouseIsWithinWindow(GlobalMouseState::lastMouseX, GlobalMouseState::lastMouseY))
		if (simWindow->onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	if (viewOptions == SIM_AND_MOPT)
		if (moptWindow->mouseIsWithinWindow(GlobalMouseState::lastMouseX, GlobalMouseState::lastMouseY))
			if (moptWindow->onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	if (viewOptions == SIM_AND_DESIGN && designWindow)
		if (designWindow->isActive() || designWindow->mouseIsWithinWindow(GlobalMouseState::lastMouseX, GlobalMouseState::lastMouseY))
			if (designWindow->onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	if (GLApplication::onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	return false;
}

bool RobotDesignerApp::onKeyEvent(int key, int action, int mods) {
	if (viewOptions == SIM_AND_DESIGN && designWindow) {
		designWindow->onKeyEvent(key, action, mods);
	}

	if (viewOptions == SIM_AND_MOPT){
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

		//todo: just a test for now
		prd = new TestParameterizedRobotDesign(robot);

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
		designWindow->saveRSFile("../out/tmpRobot.rs", robot);
		loadFile("../out/tmpRobot.rs");
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
	if (viewOptions == SIM_AND_MOPT && moptWindow) {
		moptWindow->setAnimationParams(moptWindow->ffpViewer->cursorPosition, 0);
		moptWindow->draw();
		moptWindow->drawAuxiliarySceneInfo();
	}

	if (viewOptions == SIM_AND_DESIGN && designWindow) {
		designWindow->draw();
		designWindow->drawAuxiliarySceneInfo();
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

void RobotDesignerApp::compute_dmdp_Jacobian(dVector& m, DynamicArray<double>& p, MatrixNxM& dmdp) {
//evaluates dm/dp at (m,p). It is assumed that m corresponds to a minimum of the energy (i.e. m = arg min (E(m(p)))
// Let g = \partial E / \partial m
// partial g / partial p + partial g / partial m * dm/dp = dg/dp = 0
// dm/dp = -partial g / partial m ^ -1 * partial g / partial p

	SparseMatrix dgdm;
	dVector dgdpi, dmdpi;
	dVector g_m, g_p;

	resize(dgdm, m.size(), m.size());
	resize(dmdp, m.size(), p.size());
	resize(dgdpi, m.size());

	DynamicArray<MTriplet> triplets;
	moptWindow->locomotionManager->energyFunction->addHessianEntriesTo(triplets, m);
	dgdm.setFromTriplets(triplets.begin(), triplets.end());

	Eigen::SimplicialLDLT<SparseMatrix, Eigen::Lower> solver;
	//	Eigen::SparseLU<SparseMatrix> solver;
	solver.compute(dgdm);

	double dp = 0.001;
	//now, for every design parameter, estimate change in gradient, and use that to compute the corresponding entry in dm/dp...
	for (uint i = 0; i < p.size(); i++) {
		resize(g_m, m.size());
		resize(g_p, m.size());

		double pVal = p[i];
		p[i] = pVal + dp;
		prd->setParameters(p);
		moptWindow->locomotionManager->energyFunction->addGradientTo(g_p, m);
		p[i] = pVal - dp;
		prd->setParameters(p);
		moptWindow->locomotionManager->energyFunction->addGradientTo(g_m, m);
		p[i] = pVal;
		prd->setParameters(p);

		dgdpi = (g_p - g_m) / (2 * dp);

		dmdp.col(i) = solver.solve(dgdpi) * -1;
	}

}

void RobotDesignerApp::test_dmdp_Jacobian() {
	dVector m; moptWindow->locomotionManager->motionPlan->writeMPParametersToList(m);
	DynamicArray<double> p;	prd->getCurrentSetOfParameters(p);

	//dm/dp, the analytic version. 
	MatrixNxM dmdp;	compute_dmdp_Jacobian(m, p, dmdp);

	//Now estimate this jacobian with finite differences...
	MatrixNxM dmdp_FD;
	dVector m_initial, m_m, m_p;
	resize(dmdp_FD, m.size(), p.size());
	double dp = 0.001;
	moptWindow->locomotionManager->motionPlan->writeMPParametersToList(m_initial);
	//now, for every design parameter, estimate change in gradient, and use that to compute the corresponding entry in dm/dp...
	for (uint i = 0; i < p.size(); i++) {
		resize(m_m, m.size());
		resize(m_p, m.size());

		double pVal = p[i];
		p[i] = pVal + dp;
		prd->setParameters(p);
		//now we must solve this thing a loooot...

		moptWindow->locomotionManager->motionPlan->setMPParametersFromList(m_initial);
		for (int j = 0; j < 300; j++)
			moptWindow->runMOPTStep();

		moptWindow->locomotionManager->motionPlan->writeMPParametersToList(m_m);
		p[i] = pVal - dp;
		prd->setParameters(p);
		moptWindow->locomotionManager->motionPlan->setMPParametersFromList(m_initial);
		for (int j = 0; j < 300; j++)
			moptWindow->runMOPTStep();
		moptWindow->locomotionManager->motionPlan->writeMPParametersToList(m_p);

		moptWindow->locomotionManager->motionPlan->setMPParametersFromList(m_initial);
		p[i] = pVal;
		prd->setParameters(p);

		dmdp_FD.col(i) = (m_p - m_m) / (2 * dp);
	}

	print("../out/dmdp.m", dmdp);
	print("../out/dmdp_FD.m", dmdp_FD);
}

void RobotDesignerApp::testOptimizeDesign() {
	dVector m; moptWindow->locomotionManager->motionPlan->writeMPParametersToList(m);
	DynamicArray<double> p;	prd->getCurrentSetOfParameters(p);
	MatrixNxM dmdp; compute_dmdp_Jacobian(m, p, dmdp);

	//If we have some objective O, expressed as a function of m, then dO/dp = dO/dm * dm/dp
	dVector dOdm;
	dVector dOdp;
	resize(dOdm, m.size());
	resize(dOdp, p.size());

	moptWindow->locomotionManager->energyFunction->objectives[11]->addGradientTo(dOdm, m);

	dOdp = dmdp.transpose() * dOdm;

	Logger::consolePrint("dOdp[0]: %lf\n", dOdp[0]);

	double len = dOdp.norm();
	if (len > 0.01)
		dOdp = dOdp / len * 0.01;

	Logger::consolePrint("p[0] before: %lf\n", p[0]);

	for (uint i = 0; i < p.size(); i++)
		p[i] -= dOdp[i];
	Logger::consolePrint("p[0] after: %lf\n", p[0]);
	prd->setParameters(p);
}

void RobotDesignerApp::addDesignParameterSliders()
{
	using namespace nanogui;
	DynamicArray<double> p;	prd->getCurrentSetOfParameters(p);

	Widget *panel = new Widget(mainMenu->window());
	GridLayout *layout =
		new GridLayout(Orientation::Horizontal, 2,
			Alignment::Middle, 15, 5);
	layout->setColAlignment(
	{ Alignment::Maximum, Alignment::Fill });
	layout->setSpacing(0, 10);
	panel->setLayout(layout);


	auto removeTrailingZeros = [](string &&s) {return s.erase(s.find_last_not_of('0') + 1, string::npos); };
	for (int i = 0; i < prd->getNumberOfParameters(); i++)
	{
		Slider *slider = new Slider(panel);
		slider->setValue(p[i]);
		slider->setRange({-0.1,0.1});
		slider->setFixedWidth(150);
		TextBox *textBox = new TextBox(panel);
		textBox->setValue(removeTrailingZeros(to_string(p[i])));
		textBox->setFixedWidth(50);
		textBox->setEditable(true);
		textBox->setFixedHeight(18);

		slider->setCallback([&, i, textBox](float value) {
			DynamicArray<double> p;	prd->getCurrentSetOfParameters(p);
			p[i] = value;
			prd->setParameters(p);
			textBox->setValue(removeTrailingZeros(to_string(value)));
		});
		textBox->setCallback([&, i, slider](const std::string &str) {
			DynamicArray<double> p;	prd->getCurrentSetOfParameters(p);
			p[i] = std::stod(str);
			prd->setParameters(p);
			slider->setValue(std::stod(str));
			return true;
		});

	}
	mainMenu->addWidget("", panel);
}
