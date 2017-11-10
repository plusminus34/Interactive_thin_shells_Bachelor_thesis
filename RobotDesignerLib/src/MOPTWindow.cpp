#pragma warning(disable : 4996)

#include <RobotDesignerLib/MOPTWindow.h>
#include <RobotDesignerLib/LocomotionEngineManagerGRF.h>
#include <RobotDesignerLib/LocomotionEngineManagerIP.h>

MOPTWindow::MOPTWindow(int x, int y, int w, int h, GLApplication* glApp) : GLWindow3D(x, y, w, h) {
	this->glApp = glApp;

	ffpViewer = new FootFallPatternViewer(x, 0, (int)(w), (int)(h / 4.0));
	ffpViewer->ffp = &footFallPattern;
	ffpViewer->cursorMovable = true;

	dynamic_cast<GLTrackingCamera*>(this->camera)->rotAboutRightAxis = 0.25;
	dynamic_cast<GLTrackingCamera*>(this->camera)->rotAboutUpAxis = 0.95;
	dynamic_cast<GLTrackingCamera*>(this->camera)->camDistance = -1.5;
}

void MOPTWindow::addMenuItems() {
/*
	auto tmp = new nanogui::Label(glApp->mainMenu->window(), "Popup buttons", "sans-bold");
	glApp->mainMenu->addWidget("", tmp);

	nanogui::PopupButton *popupBtn = new nanogui::PopupButton(glApp->mainMenu->window(), "Popup", ENTYPO_ICON_EXPORT);
	glApp->mainMenu->addWidget("", popupBtn);

	nanogui::Popup *popup = popupBtn->popup();
	popup->setLayout(new nanogui::GroupLayout());
	new nanogui::Label(popup, "Arbitrary widgets can be placed here");
	new nanogui::CheckBox(popup, "A check box");
*/
	{
		auto tmpVar = glApp->mainMenu->addVariable("swingFootHeight", moptParams.swingFootHeight);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.01);
	}
	{
		auto tmpVar = glApp->mainMenu->addVariable("des speed X", moptParams.desTravelDistX);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.05);
	}
	{
		auto tmpVar = glApp->mainMenu->addVariable("des Speed Z", moptParams.desTravelDistZ);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.05);
	}
	{
		auto tmpVar = glApp->mainMenu->addVariable("des turning angle", moptParams.desTurningAngle);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.05);
	}
	{
		auto tmpVar = glApp->mainMenu->addVariable("gait duration", moptParams.motionPlanDuration);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.05);
	}
	{
		auto tmpVar = glApp->mainMenu->addVariable("joint velocity limit", moptParams.jointVelocityLimit);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.05);
	}
	{
		auto tmpVar = glApp->mainMenu->addVariable("joint velocity epsilon", moptParams.jointVelocityEpsilon);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.01);
	}
	{
		glApp->mainMenu->addVariable("write joint velocity profile", moptParams.writeJointVelocityProfile);
	}

	glApp->mainMenu->addVariable("check derivatives", moptParams.checkDerivatives);
	glApp->mainMenu->addVariable("Mopt Mode", optimizeOption, true)->setItems({ "GRFv1", "GRFv2", "IPv1", "IPv2" });

/*
	//this is the slider for the phase...
	new nanogui::Label(glApp->mainMenu->window(), "Slider and text box", "sans-bold");

	nanogui::Widget *panel = new nanogui::Widget(glApp->mainMenu->window());
	panel->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
		nanogui::Alignment::Middle, 0, 20));

	nanogui::Slider *slider = new nanogui::Slider(panel);
	slider->setValue(0.5f);
	slider->setFixedWidth(80);

	nanogui::TextBox *textBox = new nanogui::TextBox(panel);
	textBox->setFixedSize(Eigen::Vector2i(60, 25));
	textBox->setValue("50");
	textBox->setUnits("%");
	slider->setCallback([textBox](float value) {
		textBox->setValue(std::to_string((int)(value * 100)));
	});
	slider->setFinalCallback([&](float value) {
		Logger::consolePrint("Final slider value: %d\n", (int)(value * 100));
	});
	textBox->setFixedSize(Eigen::Vector2i(60, 25));
	textBox->setFontSize(20);
	textBox->setAlignment(nanogui::TextBox::Alignment::Right);
*/
}


MOPTWindow::~MOPTWindow()
{
	clear();
}

void MOPTWindow::clear()
{

}

void MOPTWindow::loadRobot(Robot* robot, ReducedRobotState* startState)
{
	clear();

	initialized = true;
	this->robot = robot;
	this->startState = *startState;

	int nLegs = robot->bFrame->limbs.size();
	nPoints = 3 * nLegs;

	// ******************* footfall patern *******************
	footFallPattern = FootFallPattern();

	bool createDefaultFFP = false;
	if (createDefaultFFP){
		int iMin = 0, iMax = nPoints / nLegs - 1;
		footFallPattern.strideSamplePoints = nPoints;
		for (int j = 0; j < nLegs; j++)
			footFallPattern.addStepPattern(robot->bFrame->limbs[j], iMin + j*nPoints / nLegs, iMax + j*nPoints / nLegs);
	}

	footFallPattern.loadFromFile("../out/tmpFFP.ffp");
}

void MOPTWindow::syncMOPTWindowParameters() {
	moptParams.swingFootHeight = locomotionManager->motionPlan->swingFootHeight;
	moptParams.desTravelDistX = locomotionManager->motionPlan->desDistanceToTravel.x();
	moptParams.desTravelDistZ = locomotionManager->motionPlan->desDistanceToTravel.z();
	moptParams.desTurningAngle = locomotionManager->motionPlan->desTurningAngle;
	moptParams.jointVelocityLimit = locomotionManager->motionPlan->jointVelocityLimit;
	moptParams.jointVelocityEpsilon = locomotionManager->motionPlan->jointVelocityEpsilon;
	moptParams.writeJointVelocityProfile = locomotionManager->writeVelocityProfileToFile;
	moptParams.motionPlanDuration = locomotionManager->motionPlan->motionPlanDuration;
	moptParams.checkDerivatives = locomotionManager->checkDerivatives;
}

void MOPTWindow::syncMotionPlanParameters(){
	locomotionManager->motionPlan->swingFootHeight = moptParams.swingFootHeight;
	locomotionManager->motionPlan->desDistanceToTravel.x() = moptParams.desTravelDistX;
	locomotionManager->motionPlan->desDistanceToTravel.z() = moptParams.desTravelDistZ;
	locomotionManager->motionPlan->desTurningAngle = moptParams.desTurningAngle;
	locomotionManager->motionPlan->jointVelocityLimit = moptParams.jointVelocityLimit;
	locomotionManager->motionPlan->jointVelocityEpsilon = moptParams.jointVelocityEpsilon;
	locomotionManager->writeVelocityProfileToFile = moptParams.writeJointVelocityProfile;
	locomotionManager->motionPlan->motionPlanDuration = moptParams.motionPlanDuration;
	locomotionManager->checkDerivatives = moptParams.checkDerivatives;
}

LocomotionEngineManager* MOPTWindow::initializeNewMP(bool doWarmStart){
	delete locomotionManager;

	footFallPattern.writeToFile("../out/tmpFFP.ffp");

	/* ----------- Reset the state of the robot ------------ */
	robot->setState(&startState);

	/* ---------- Set up the motion plan ---------- */
	switch (optimizeOption)
	{
	case GRF_OPT:
		locomotionManager = new LocomotionEngineManagerGRFv1(robot, &footFallPattern, nPoints + 1); break;
	case GRF_OPT_V2:
		locomotionManager = new LocomotionEngineManagerGRFv2(robot, &footFallPattern, nPoints + 1); break;
	case IP_OPT:
		locomotionManager = new LocomotionEngineManagerIPv1(robot, &footFallPattern, nPoints + 1); break;
	case IP_OPT_V2:
		locomotionManager = new LocomotionEngineManagerIPv2(robot, &footFallPattern, nPoints + 1); break;
	default:
		locomotionManager = new LocomotionEngineManagerGRFv2(robot, &footFallPattern, nPoints + 1); break;
	}

	syncMotionPlanParameters();

	if (doWarmStart)
		locomotionManager->warmStartMOpt();

	syncMOPTWindowParameters();

	locomotionManager->setDefaultOptimizationFlags();

	return locomotionManager;
}

double MOPTWindow::runMOPTStep(){
	syncMotionPlanParameters();

	double energyVal = locomotionManager->runMOPTStep();

	return energyVal;
}

void MOPTWindow::reset(){
	locomotionManager = NULL;
}

void MOPTWindow::setAnimationParams(double f, int animationCycle){
	moptParams.phase = f;
	moptParams.gaitCycle = animationCycle;
	ffpViewer->cursorPosition = f;
}

void MOPTWindow::loadFFPFromFile(const char* fName){
	footFallPattern.loadFromFile(fName);
}

void MOPTWindow::drawScene() {
	glColor3d(1, 1, 1);
	glDisable(GL_LIGHTING);
	drawGround(GLContentManager::getTexture("../data/textures/ground_TileLight2.bmp"));
	glEnable(GL_LIGHTING);

	if (locomotionManager){
		locomotionManager->drawMotionPlan(moptParams.phase, moptParams.gaitCycle, moptParams.drawRobotPose, moptParams.drawPlanDetails, moptParams.drawContactForces, moptParams.drawOrientation);
	}
}

void MOPTWindow::drawAuxiliarySceneInfo(){
	glClear(GL_DEPTH_BUFFER_BIT);

	preDraw();
	ffpViewer->draw();
	postDraw();

	glClear(GL_DEPTH_BUFFER_BIT);

}

bool MOPTWindow::onMouseMoveEvent(double xPos, double yPos){
	if (initialized) {
		if (ffpViewer->mouseIsWithinWindow(xPos, yPos) || ffpViewer->isDragging())
			if (ffpViewer->onMouseMoveEvent(xPos, yPos)) return true;
	}

	return GLWindow3D::onMouseMoveEvent(xPos, yPos);
}

bool MOPTWindow::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos)
{
	if (initialized) {
		if (ffpViewer->mouseIsWithinWindow(xPos, yPos) || ffpViewer->isDragging())
			if (ffpViewer->onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;
	}

	return GLWindow3D::onMouseButtonEvent(button, action, mods, xPos, yPos);
}

void MOPTWindow::setViewportParameters(int posX, int posY, int sizeX, int sizeY){
	GLWindow3D::setViewportParameters(posX, posY, sizeX, sizeY);
	ffpViewer->setViewportParameters(posX, (int)(sizeY * 3.0 / 4), sizeX, (int)(sizeY / 4.0));
}

void MOPTWindow::setupLights() {
	GLfloat bright[] = { 0.8f, 0.8f, 0.8f, 1.0f };
	GLfloat mediumbright[] = { 0.3f, 0.3f, 0.3f, 1.0f };

	glLightfv(GL_LIGHT1, GL_DIFFUSE, bright);
	glLightfv(GL_LIGHT2, GL_DIFFUSE, mediumbright);
	glLightfv(GL_LIGHT3, GL_DIFFUSE, mediumbright);
	glLightfv(GL_LIGHT4, GL_DIFFUSE, mediumbright);


	GLfloat light0_position[] = { 0.0f, 10000.0f, 10000.0f, 0.0f };
	GLfloat light0_direction[] = { 0.0f, -10000.0f, -10000.0f, 0.0f };

	GLfloat light1_position[] = { 0.0f, 10000.0f, -10000.0f, 0.0f };
	GLfloat light1_direction[] = { 0.0f, -10000.0f, 10000.0f, 0.0f };

	GLfloat light2_position[] = { 0.0f, -10000.0f, 0.0f, 0.0f };
	GLfloat light2_direction[] = { 0.0f, 10000.0f, -0.0f, 0.0f };

	GLfloat light3_position[] = { 10000.0f, -10000.0f, 0.0f, 0.0f };
	GLfloat light3_direction[] = { -10000.0f, 10000.0f, -0.0f, 0.0f };

	GLfloat light4_position[] = { -10000.0f, -10000.0f, 0.0f, 0.0f };
	GLfloat light4_direction[] = { 10000.0f, 10000.0f, -0.0f, 0.0f };


	glLightfv(GL_LIGHT0, GL_POSITION, light0_position);
	glLightfv(GL_LIGHT1, GL_POSITION, light1_position);
	glLightfv(GL_LIGHT2, GL_POSITION, light2_position);
	glLightfv(GL_LIGHT3, GL_POSITION, light3_position);
	glLightfv(GL_LIGHT4, GL_POSITION, light4_position);


	glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, light0_direction);
	glLightfv(GL_LIGHT1, GL_SPOT_DIRECTION, light1_direction);
	glLightfv(GL_LIGHT2, GL_SPOT_DIRECTION, light2_direction);
	glLightfv(GL_LIGHT3, GL_SPOT_DIRECTION, light3_direction);
	glLightfv(GL_LIGHT4, GL_SPOT_DIRECTION, light4_direction);


	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	glEnable(GL_LIGHT2);
	glEnable(GL_LIGHT3);
	glEnable(GL_LIGHT4);
}

