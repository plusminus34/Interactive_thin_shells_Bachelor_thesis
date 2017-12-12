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
	auto tmp = new nanoFui::Label(glApp->mainMenu->window(), "Popup buttons", "sans-bold");
	glApp->mainMenu->addWidget("", tmp);

	nanogui::PopupButton *popupBtn = new nanogui::PopupButton(glApp->mainMenu->window(), "Popup", ENTYPO_ICON_EXPORT);
	glApp->mainMenu->addWidget("", popupBtn);

	nanogui::Popup *popup = popupBtn->popup();
	popup->setLayout(new nanogui::GroupLayout());
	new nanogui::Label(popup, "Arbitrary widgets can be placed here");
	new nanogui::CheckBox(popup, "A check box");
*/

	{
		auto tmpVar = glApp->mainMenu->addVariable("startWithEmptyFFP", startWithEmptyFFP);
	}

	{
		auto tmpVar = glApp->mainMenu->addVariable("# of MOPT sample points", nTimeSteps);
		tmpVar->setSpinnable(true);
	}

	{
		auto tmpVar = glApp->mainMenu->addVariable("globalMOPTRegularizer", globalMOPTRegularizer);
		tmpVar->setSpinnable(false); tmpVar->setMinValue(0); tmpVar->setMaxValue(100);
	}

	

	glApp->mainMenu->addVariable<bool>("Show energies menu",
		[this](bool value) {
			showWeightsAndEnergyValues = value;
			ToggleEnergyMenu();
		},
		[this]() {return showWeightsAndEnergyValues; });

	{
		auto tmpVar = glApp->mainMenu->addVariable("swingFootHeight", moptParams.swingFootHeight);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.01);
	}
	{
		auto tmpVar = glApp->mainMenu->addVariable("des speed X", moptParams.desTravelDistX);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.05);
	}

	glApp->mainMenu->addVariable("curr speed X", COMSpeed(0))->setEditable(false);
	{
		auto tmpVar = glApp->mainMenu->addVariable("des Speed Z", moptParams.desTravelDistZ);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.05);
	}
	glApp->mainMenu->addVariable("curr speed Z", COMSpeed(2))->setEditable(false);
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
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.5);
	}
	{
		auto tmpVar = glApp->mainMenu->addVariable("joint velocity epsilon", moptParams.jointVelocityEpsilon);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.01);
	}
	{
		auto tmpVar = glApp->mainMenu->addVariable("wheel speed limit", moptParams.wheelSpeedLimit);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.5);
	}
	{
		auto tmpVar = glApp->mainMenu->addVariable("wheel speed epsilon", moptParams.wheelSpeedEpsilon);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.01);
	}
	{
		auto tmpVar = glApp->mainMenu->addVariable("wheel accel. limit", moptParams.wheelAccelLimit);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.5);
	}
	{
		auto tmpVar = glApp->mainMenu->addVariable("wheel accel. epsilon", moptParams.wheelAccelEpsilon);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.01);
	}
	{
		glApp->mainMenu->addVariable("write joint velocity profile", moptParams.writeJointVelocityProfile);
	}

	glApp->mainMenu->addVariable("check derivatives", moptParams.checkDerivatives);
	glApp->mainMenu->addVariable<bool>("Log data",
		[this](bool val) {if (locomotionManager) locomotionManager->printDebugInfo = val; },
		[this] { if (locomotionManager) return locomotionManager->printDebugInfo; else return false; });
	glApp->mainMenu->addVariable("Mopt Mode", optimizeOption, true)->setItems({ "GRFv1", "GRFv2", "IPv1", "IPv2" });



	{
		using namespace nanogui;

		Window *window = new Window(glApp->menuScreen, "Wheel Control");
		window->setPosition(Eigen::Vector2i(300, 0));
		window->setWidth(300);
		window->setLayout(new GroupLayout());

		energyGraph = window->add<Graph>("Energy function");
	}

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

void MOPTWindow::loadRobot(Robot* robot, ReducedRobotState* startState){
	clear();

	initialized = true;
	this->robot = robot;
	this->startState = *startState;

	int nLegs = robot->bFrame->limbs.size();

	// ******************* footfall patern *******************
	footFallPattern = FootFallPattern();

	if (startWithEmptyFFP == false){
		int iMin = 0, iMax = nTimeSteps / nLegs - 1;
		footFallPattern.strideSamplePoints = nTimeSteps;
		for (int j = 0; j < nLegs; j++)
			footFallPattern.addStepPattern(robot->bFrame->limbs[j], iMin + j*nTimeSteps / nLegs, iMax + j*nTimeSteps / nLegs);
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

	moptParams.wheelSpeedLimit = locomotionManager->motionPlan->wheelSpeedLimit;
	moptParams.wheelSpeedEpsilon = locomotionManager->motionPlan->wheelSpeedEpsilon;

	moptParams.wheelAccelLimit = locomotionManager->motionPlan->wheelAccelLimit;
	moptParams.wheelAccelEpsilon = locomotionManager->motionPlan->wheelAccelEpsilon;

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

	locomotionManager->motionPlan->wheelSpeedLimit = moptParams.wheelSpeedLimit;
	locomotionManager->motionPlan->wheelSpeedEpsilon = moptParams.wheelSpeedEpsilon;

	locomotionManager->motionPlan->wheelAccelLimit = moptParams.wheelAccelLimit;
	locomotionManager->motionPlan->wheelAccelEpsilon = moptParams.wheelAccelEpsilon;

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
		locomotionManager = new LocomotionEngineManagerGRFv1(robot, &footFallPattern, nTimeSteps + 1); break;
	case GRF_OPT_V2:
		locomotionManager = new LocomotionEngineManagerGRFv2(robot, &footFallPattern, nTimeSteps + 1); break;
	case IP_OPT:
		locomotionManager = new LocomotionEngineManagerIPv1(robot, &footFallPattern, nTimeSteps + 1); break;
	case IP_OPT_V2:
		locomotionManager = new LocomotionEngineManagerIPv2(robot, &footFallPattern, nTimeSteps + 1); break;
	default:
		locomotionManager = new LocomotionEngineManagerGRFv2(robot, &footFallPattern, nTimeSteps + 1); break;
	}

	syncMotionPlanParameters();

	if (doWarmStart)
		locomotionManager->warmStartMOpt();

	syncMOPTWindowParameters();

	locomotionManager->setDefaultOptimizationFlags();
	locomotionManager->energyFunction->regularizer = globalMOPTRegularizer;

	CreateEnergyMenu();



	return locomotionManager;
}

double MOPTWindow::runMOPTStep(){
	syncMotionPlanParameters();

	locomotionManager->energyFunction->regularizer = globalMOPTRegularizer;

	double energyVal = locomotionManager->runMOPTStep();

	// plot energy value
	{
		energyGraphValues.push_back(energyVal);
		int start = std::max(0, (int)energyGraphValues.size()-100);
		int size = std::min(100, (int)energyGraphValues.size());
		Eigen::Map<Eigen::VectorXf> values(&energyGraphValues[start], size);
		energyGraph->setValues(values);
	}


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

		int startIndex = locomotionManager->motionPlan->wrapAroundBoundaryIndex;
		if (startIndex < 0)  startIndex = 0;
		COMSpeed = locomotionManager->motionPlan->COMTrajectory.getCOMPositionAtTimeIndex(locomotionManager->motionPlan->nSamplePoints - 1) - 
			       locomotionManager->motionPlan->COMTrajectory.getCOMPositionAtTimeIndex(startIndex);
			
	}
	if (showWeightsAndEnergyValues)
		updateSliders();


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

void MOPTWindow::ToggleEnergyMenu()
{
	energyMenu->setVisible(showWeightsAndEnergyValues);
}

void MOPTWindow::CreateEnergyMenu()
{
	if (energyMenu)
		energyMenu->dispose();
	nanogui::Window* mainwindow = glApp->mainMenu->window();
	energyMenu = glApp->mainMenu->addWindow(Eigen::Vector2i(viewportX, viewportY), "Energy values and weights");

	using namespace nanogui;

	Widget *panel = new Widget(glApp->mainMenu->window());
	GridLayout *layout =
		new GridLayout(Orientation::Horizontal, 5,
			Alignment::Middle);
	layout->setColAlignment(
	{ Alignment::Maximum, Alignment::Fill });
	layout->setSpacing(0, 10);
	panel->setLayout(layout);

	int NE = locomotionManager->energyFunction->objectives.size();
	energySliders.resize(NE);
	energyTextboxes.resize(NE);
	weightTextboxes.resize(NE);

	for (int i = 0; i < NE; i++)
	{
		double value = 0;
		new Label(panel, locomotionManager->energyFunction->objectives[i]->description , "sans-bold");
		CheckBox *chkBox = new CheckBox(panel,"");
		chkBox->setChecked(locomotionManager->energyFunction->objectives[i]->isActive);
		chkBox->setCallback([this, i](bool value){locomotionManager->energyFunction->objectives[i]->isActive = value; });
		Slider *slider = new Slider(panel);
		slider->setValue(0);
		slider->setRange({ 0.0,1.0 });
		slider->setFixedWidth(50);
		energySliders[i] = slider;

		FloatBox<double> *textBox = new FloatBox<double>(panel);
		textBox->setFixedWidth(80);
		textBox->setEditable(false);
		textBox->setFixedHeight(18);
		energyTextboxes[i] = textBox;

		textBox = new FloatBox<double>(panel);
		textBox->setFixedWidth(80);
		textBox->setFixedHeight(18);
		textBox->setEditable(true);
		textBox->setValue(locomotionManager->energyFunction->objectives[i]->weight);
		textBox->setCallback([this, i](double value){locomotionManager->energyFunction->objectives[i]->weight=value; });
		weightTextboxes[i] = textBox;
	}
	energyMenu->setVisible(false);
	glApp->mainMenu->addWidget("", panel);
	glApp->mainMenu->setWindow(mainwindow);
	glApp->menuScreen->performLayout();
}

void MOPTWindow::updateSliders()
{
	int NE = locomotionManager->energyFunction->objectives.size();
	dVector params;
	locomotionManager->motionPlan->writeMPParametersToList(params);
	auto double2string = [](double val) {
		ostringstream s;
		s.precision(2);
		s << val;
		return s.str(); };
	
	for (int i = 0; i < NE; i++)
	{
		double value = locomotionManager->energyFunction->objectives[i]->computeValue(params);
		energySliders[i]->setValue((float)value);
		energyTextboxes[i]->TextBox::setValue(double2string(value));
	}
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

