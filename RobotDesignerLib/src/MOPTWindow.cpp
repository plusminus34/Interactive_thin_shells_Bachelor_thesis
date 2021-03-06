#pragma warning(disable : 4996)

#include <RobotDesignerLib/MOPTWindow.h>
#include <RobotDesignerLib/LocomotionEngineManagerGRF.h>
#include <RobotDesignerLib/LocomotionEngineManagerIP.h>
#include "../Apps/RobotDesignerApp/RobotDesignerApp.h"

MOPTWindow::MOPTWindow(int x, int y, int w, int h, BaseRobotControlApp* theApp) : GLWindow3D(x, y, w, h) {
	this->theApp = theApp;

	ffpViewer = new FootFallPatternViewer(x, 0, (int)(w), (int)(h / 4.0));
	ffpViewer->ffp = &footFallPattern;
	ffpViewer->cursorMovable = true;

	dynamic_cast<GLTrackingCamera*>(this->camera)->rotAboutRightAxis = 0.25;
	dynamic_cast<GLTrackingCamera*>(this->camera)->rotAboutUpAxis = 0.95;
	dynamic_cast<GLTrackingCamera*>(this->camera)->camDistance = -1.5;
	

	showGroundPlane = true;
	showReflections = true;

}

void MOPTWindow::addMenuItems() {
/*
	auto tmp = new nanoFui::Label(theApp->mainMenu->window(), "Popup buttons", "sans-bold");
	theApp->mainMenu->addWidget("", tmp);

	nanogui::PopupButton *popupBtn = new nanogui::PopupButton(theApp->mainMenu->window(), "Popup", ENTYPO_ICON_EXPORT);
	theApp->mainMenu->addWidget("", popupBtn);

	nanogui::Popup *popup = popupBtn->popup();
	popup->setLayout(new nanogui::GroupLayout());
	new nanogui::Label(popup, "Arbitrary widgets can be placed here");
	new nanogui::CheckBox(popup, "A check box");
*/

	{
		auto tmpVar = theApp->mainMenu->addVariable("generate periodic motion", periodicMotion);
	}

	{
		auto tmpVar = theApp->mainMenu->addVariable("# of MOPT sample points", nTimeSteps);
		tmpVar->setSpinnable(true);
	}

	theApp->mainMenu->addVariable("Allow Dynamic regularization", moptParams.hessCorrectionMethod)->setItems({ "None", "DynamicRegularization", "Projection"});

	{
		auto tmpVar = theApp->mainMenu->addVariable("globalMOPTRegularizer", globalMOPTRegularizer);
		tmpVar->setSpinnable(false); tmpVar->setMinValue(0); tmpVar->setMaxValue(100);
	}

	{
		auto tmpVar = theApp->mainMenu->addVariable("swingFootHeight", moptParams.swingFootHeight);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.01);
	}
	{
		auto tmpVar = theApp->mainMenu->addVariable("des speed X", moptParams.desTravelDistX);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.05);
	}

	theApp->mainMenu->addVariable("curr speed X", COMSpeed(0))->setEditable(false);
	{
		auto tmpVar = theApp->mainMenu->addVariable("des Speed Z", moptParams.desTravelDistZ);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.05);
	}
	theApp->mainMenu->addVariable("curr speed Z", COMSpeed(2))->setEditable(false);
	{
		auto tmpVar = theApp->mainMenu->addVariable("des turning angle", moptParams.desTurningAngle);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.05);
	}
	{
		auto tmpVar = theApp->mainMenu->addVariable("Ext force X", moptParams.externalForceX);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.05);
	}
	{
		auto tmpVar = theApp->mainMenu->addVariable("Ext force Z", moptParams.externalForceZ);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.05);
	}
	{
		auto tmpVar = theApp->mainMenu->addVariable("gait duration", moptParams.motionPlanDuration);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.05);
	}
	{
		auto tmpVar = theApp->mainMenu->addVariable("joint velocity limit", moptParams.jointVelocityLimit);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.5);
	}
	{
		auto tmpVar = theApp->mainMenu->addVariable("joint velocity epsilon", moptParams.jointVelocityEpsilon);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.01);
	}
	{
		auto tmpVar = theApp->mainMenu->addVariable("EE min distance", moptParams.EEminDistance);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.05);
	}
	{
		auto tmpVar = theApp->mainMenu->addVariable("joint angle limit", moptParams.jointAngleLimit);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.5);
	}
	{
		auto tmpVar = theApp->mainMenu->addVariable("wheel speed limit", moptParams.wheelSpeedLimit);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.5);
	}
	{
		auto tmpVar = theApp->mainMenu->addVariable("wheel speed epsilon", moptParams.wheelSpeedEpsilon);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.01);
	}
	{
		auto tmpVar = theApp->mainMenu->addVariable("wheel accel. limit", moptParams.wheelAccelLimit);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.5);
	}
	{
		auto tmpVar = theApp->mainMenu->addVariable("wheel accel. epsilon", moptParams.wheelAccelEpsilon);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.01);
	}
	{
		auto tmpVar = theApp->mainMenu->addVariable("joint velocity L0 delta", moptParams.jointL0Delta);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.05);
	}
	{
		auto tmpVar = theApp->mainMenu->addVariable("friction coeff", moptParams.frictionCoeff);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.05);
	}
	{
		theApp->mainMenu->addVariable("write joint velocity profile", moptParams.writeJointVelocityProfile);
	}

	theApp->mainMenu->addButton("Check Derivatives", [this]() {
		bool temp = moptParams.checkDerivatives; moptParams.checkDerivatives = true;
		runMOPTStep();
		moptParams.checkDerivatives = temp;
	});

	theApp->mainMenu->addButton("Check Hessians PSD", [this]() {
		bool temp = moptParams.checkHessianPSD; moptParams.checkHessianPSD = true;
		runMOPTStep();
		moptParams.checkHessianPSD = temp;
	});

	theApp->mainMenu->addVariable<bool>("Log data",
		[this](bool val) {if (locomotionManager) locomotionManager->printDebugInfo = val; },
		[this] { if (locomotionManager) return locomotionManager->printDebugInfo; else return false; });
	theApp->mainMenu->addVariable("Mopt Mode", optimizeOption, true)->setItems({ "GRFv1", "GRFv2", "GRFv3", "IPv1", "IPv2" });
	
	theApp->mainMenu->addVariable("Optimization method", moptParams.optimizationMethod, true)->setItems({ "Newton", "BFGS" });

	{
		using namespace nanogui;

		Window *window = new Window(theApp->menuScreen, "MOPT Energy");
		window->setWidth(300);
		window->setPosition(Eigen::Vector2i(theApp->getMainWindowWidth() / theApp->menuScreen->pixelRatio() - 220, 20));
		window->setLayout(new GroupLayout());

		energyGraph = window->add<Graph>("Energy function");
	}

/*
	//this is the slider for the phase...
	new nanogui::Label(theApp->mainMenu->window(), "Slider and text box", "sans-bold");

	nanogui::Widget *panel = new nanogui::Widget(theApp->mainMenu->window());
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

void MOPTWindow::loadRobot(Robot* robot){
	clear();

	initialized = true;
	this->robot = robot;

	int nLegs = robot->bFrame->limbs.size();

	// ******************* footfall patern *******************
	footFallPattern = FootFallPattern();

	if (nLegs > 0){
		int iMin = 0, iMax = nTimeSteps / nLegs - 1;
		footFallPattern.strideSamplePoints = nTimeSteps;
		for (int j = 0; j < nLegs; j++)
			footFallPattern.addStepPattern(robot->bFrame->limbs[j], iMin + j*nTimeSteps / nLegs, iMax + j*nTimeSteps / nLegs);

		footFallPattern.loadFromFile("../out/tmpFFP.ffp");
	}
}

void MOPTWindow::syncMOPTWindowParameters() {
	moptParams.swingFootHeight = locomotionManager->motionPlan->swingFootHeight;
	moptParams.desTravelDistX = locomotionManager->motionPlan->desDistanceToTravel.x();
	moptParams.desTravelDistZ = locomotionManager->motionPlan->desDistanceToTravel.z();
	moptParams.desTurningAngle = locomotionManager->motionPlan->desTurningAngle;

	moptParams.externalForceX = locomotionManager->motionPlan->externalForce.x();
	moptParams.externalForceZ = locomotionManager->motionPlan->externalForce.z();

	moptParams.jointVelocityLimit = locomotionManager->motionPlan->jointVelocityLimit;
	moptParams.jointVelocityEpsilon = locomotionManager->motionPlan->jointVelocityEpsilon;
	moptParams.jointAngleLimit = locomotionManager->motionPlan->jointAngleLimit;
	moptParams.EEminDistance = locomotionManager->motionPlan->EEminDistance;

	moptParams.jointL0Delta = locomotionManager->motionPlan->jointL0Delta;

	moptParams.wheelSpeedLimit = locomotionManager->motionPlan->wheelSpeedLimit;
	moptParams.wheelSpeedEpsilon = locomotionManager->motionPlan->wheelSpeedEpsilon;

	moptParams.wheelAccelLimit = locomotionManager->motionPlan->wheelAccelLimit;
	moptParams.wheelAccelEpsilon = locomotionManager->motionPlan->wheelAccelEpsilon;

	moptParams.frictionCoeff = locomotionManager->motionPlan->frictionCoeff;

	moptParams.writeJointVelocityProfile = locomotionManager->writeVelocityProfileToFile;
	moptParams.motionPlanDuration = locomotionManager->motionPlan->motionPlanDuration;
	moptParams.checkDerivatives = locomotionManager->checkDerivatives;
	moptParams.checkHessianPSD = locomotionManager->checkHessianPSD;
	moptParams.hessCorrectionMethod = locomotionManager->hessCorrectionMethod;
	moptParams.optimizationMethod = (MOPTParams::OptMethod) locomotionManager->optimizationMethod;
}

void MOPTWindow::syncMotionPlanParameters(){
	locomotionManager->motionPlan->swingFootHeight = moptParams.swingFootHeight;
	locomotionManager->motionPlan->desDistanceToTravel.x() = moptParams.desTravelDistX;
	locomotionManager->motionPlan->desDistanceToTravel.z() = moptParams.desTravelDistZ;
	
	locomotionManager->motionPlan->externalForce.x() = moptParams.externalForceX;
	locomotionManager->motionPlan->externalForce.z() = moptParams.externalForceZ;

	locomotionManager->motionPlan->desTurningAngle = moptParams.desTurningAngle;

	locomotionManager->motionPlan->jointVelocityLimit = moptParams.jointVelocityLimit;
	locomotionManager->motionPlan->jointVelocityEpsilon = moptParams.jointVelocityEpsilon;
	locomotionManager->motionPlan->jointAngleLimit = moptParams.jointAngleLimit;
	locomotionManager->motionPlan->EEminDistance = moptParams.EEminDistance;


	locomotionManager->motionPlan->jointL0Delta = moptParams.jointL0Delta;

	locomotionManager->motionPlan->wheelSpeedLimit = moptParams.wheelSpeedLimit;
	locomotionManager->motionPlan->wheelSpeedEpsilon = moptParams.wheelSpeedEpsilon;

	locomotionManager->motionPlan->wheelAccelLimit = moptParams.wheelAccelLimit;
	locomotionManager->motionPlan->wheelAccelEpsilon = moptParams.wheelAccelEpsilon;

	locomotionManager->motionPlan->frictionCoeff = moptParams.frictionCoeff;

	locomotionManager->writeVelocityProfileToFile = moptParams.writeJointVelocityProfile;
	locomotionManager->motionPlan->motionPlanDuration = moptParams.motionPlanDuration;
	locomotionManager->checkDerivatives = moptParams.checkDerivatives;
	locomotionManager->checkHessianPSD = moptParams.checkHessianPSD;
	locomotionManager->hessCorrectionMethod = moptParams.hessCorrectionMethod;
	locomotionManager->optimizationMethod = (LocomotionEngineManager::OptMethod) moptParams.optimizationMethod;

}

LocomotionEngineManager* MOPTWindow::initializeNewMP(bool doWarmStart){
	delete locomotionManager;

	footFallPattern.writeToFile("../out/tmpFFP.ffp");

	//make sure the ffp does not get out of sync with the number of samples in the locomotion engine...
	nTimeSteps = footFallPattern.strideSamplePoints;

	/* ---------- Set up the motion plan ---------- */
	switch (optimizeOption)
	{
	case GRF_OPT:
		locomotionManager = new LocomotionEngineManagerGRFv1(robot, &footFallPattern, nTimeSteps + 1); break;
	case GRF_OPT_V2:
		locomotionManager = new LocomotionEngineManagerGRFv2(robot, &footFallPattern, nTimeSteps + 1, periodicMotion); break;
	case GRF_OPT_V3:
		locomotionManager = new LocomotionEngineManagerGRFv3(robot, &footFallPattern, nTimeSteps + 1); break;
	case IP_OPT:
		locomotionManager = new LocomotionEngineManagerIPv1(robot, &footFallPattern, nTimeSteps + 1); break;
	case IP_OPT_V2:
		locomotionManager = new LocomotionEngineManagerIPv2(robot, &footFallPattern, nTimeSteps + 1); break;
	default:
		locomotionManager = new LocomotionEngineManagerGRFv2(robot, &footFallPattern, nTimeSteps + 1); break;
	}

	syncMotionPlanParameters();

	printCurrentObjectiveValues();

	if (doWarmStart)
		locomotionManager->warmStartMOpt();

	syncMOPTWindowParameters();

	locomotionManager->setDefaultOptimizationFlags();
	locomotionManager->energyFunction->regularizer = globalMOPTRegularizer;

	return locomotionManager;
}

double MOPTWindow::runMOPTStep(){
	syncMotionPlanParameters();

	locomotionManager->energyFunction->regularizer = globalMOPTRegularizer;

	double energyVal = locomotionManager->runMOPTStep();

	// plot energy value
	{
		energyGraphValues.push_back((float)energyVal);
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
	glEnable(GL_LIGHTING);

	if (locomotionManager){

		//hacks...
		if (0){
			moptParams.drawRobotMesh = moptParams.drawSkeleton = moptParams.drawAxesOfRotation = moptParams.drawWheels = moptParams.drawContactForces = moptParams.drawSupportPolygon = moptParams.drawEndEffectorTrajectories = moptParams.drawCOMTrajectory = moptParams.drawOrientation = false;
			if (moptParams.gaitCycle == 0) {
				theApp->slowMoFactor = 5;
				moptParams.drawEndEffectorTrajectories = true;
				if (moptParams.phase > 0.33)
					moptParams.drawContactForces = true;
				if (moptParams.phase > 0.66)
					moptParams.drawCOMTrajectory = moptParams.drawOrientation = true;
			}
			else if (moptParams.gaitCycle == 1) {
				theApp->slowMoFactor = 5;
				moptParams.drawEndEffectorTrajectories = true;
			}
			else if (moptParams.gaitCycle == 2) {
				theApp->slowMoFactor = 5;
				moptParams.drawContactForces = moptParams.drawEndEffectorTrajectories = true;
			}
			else if (moptParams.gaitCycle == 3) {
				theApp->slowMoFactor = 5;
				moptParams.drawContactForces = moptParams.drawEndEffectorTrajectories = moptParams.drawCOMTrajectory = moptParams.drawOrientation = true;
			}
			else if (moptParams.gaitCycle == 4) {
				theApp->slowMoFactor = 5;
				moptParams.drawContactForces = moptParams.drawEndEffectorTrajectories = moptParams.drawCOMTrajectory = moptParams.drawOrientation = moptParams.drawWheels = true;
			}
			else if (moptParams.gaitCycle == 5) {
				theApp->slowMoFactor = 5;
				moptParams.drawContactForces = moptParams.drawEndEffectorTrajectories = moptParams.drawCOMTrajectory = moptParams.drawWheels = moptParams.drawSkeleton = moptParams.drawOrientation = true;
			}
			else if (moptParams.gaitCycle == 6) {
				theApp->slowMoFactor = 5;
				moptParams.drawContactForces = moptParams.drawEndEffectorTrajectories = moptParams.drawCOMTrajectory = moptParams.drawOrientation = true;
				if (moptParams.phase > 0.5)
					moptParams.drawWheels = true;
			}
			else if (moptParams.gaitCycle == 7) {
				theApp->slowMoFactor = 5;
				moptParams.drawContactForces = moptParams.drawEndEffectorTrajectories = moptParams.drawCOMTrajectory = moptParams.drawOrientation = moptParams.drawWheels = true;
				if (moptParams.phase > 0.5)
					moptParams.drawSkeleton = true;
			}
			else {
				theApp->slowMoFactor = 1.0;
				moptParams.drawRobotMesh = true;
			}
		}

		locomotionManager->motionPlan->drawMotionPlan(moptParams.phase, moptParams.drawRobotMesh, moptParams.drawSkeleton, moptParams.drawAxesOfRotation, moptParams.drawWheels, moptParams.drawContactForces, moptParams.drawSupportPolygon, moptParams.drawEndEffectorTrajectories, moptParams.drawCOMTrajectory, moptParams.drawOrientation);

		int startIndex = locomotionManager->motionPlan->wrapAroundBoundaryIndex;
		if (startIndex < 0)  startIndex = 0;
		COMSpeed = locomotionManager->motionPlan->bodyTrajectory.getCOMPositionAtTimeIndex(locomotionManager->motionPlan->nSamplePoints - 1) - 
			       locomotionManager->motionPlan->bodyTrajectory.getCOMPositionAtTimeIndex(startIndex);
			
	}
	for (auto widget : EEwidgets)
	{
		if (fabs(EEwidget2constraint[widget]->phase - moptParams.phase) < (moptParams.motionPlanDuration / nTimeSteps))
			widget->transparent = false;
		else
			widget->transparent = true;
		widget->draw();
	}
	for (auto widget : COMWidgets)
	{
		if (fabs(COMwidget2constraint[widget]->phase - moptParams.phase) < (moptParams.motionPlanDuration / nTimeSteps))
			widget->transparent = false;
		else
			widget->transparent = true;
		widget->draw();
	}
}

void MOPTWindow::drawAuxiliarySceneInfo(){
	glClear(GL_DEPTH_BUFFER_BIT);

	preDraw();
	if (showFFPViewer) ffpViewer->draw();
	postDraw();

	glClear(GL_DEPTH_BUFFER_BIT);

}

//any time a physical key is pressed, this event will trigger. Useful for reading off special keys...
bool MOPTWindow::onKeyEvent(int key, int action, int mods) {
	if (initialized) {
		if (ffpViewer && showFFPViewer && ffpViewer->onKeyEvent(key, action, mods))
			return true;
		if (key == GLFW_KEY_DELETE && action == GLFW_PRESS)
		{
			for (auto itr = EEwidgets.begin(); itr != EEwidgets.end(); itr++)
				if ((*itr)->active)
				{
					locomotionManager->motionPlan->EEPosObjectives.remove(EEwidget2constraint[*itr]);
					EEwidgets.erase(itr);
					break;
				}
			for (auto itr = COMWidgets.begin(); itr != COMWidgets.end(); itr++)
				if ((*itr)->active)
				{
					locomotionManager->motionPlan->BodyFrameObjectives.remove(COMwidget2constraint[*itr]);
					COMWidgets.erase(itr);
					break;
				}
		}
		if (key == GLFW_KEY_L && action == GLFW_PRESS)
		{
			int timeStep = (int)round(moptParams.phase*nTimeSteps);
			Logger::consolePrint("Picked body at time step %d", timeStep);
			auto widget = std::make_shared<CompositeWidget>();
			COMWidgets.push_back(widget);

			widget->setPos(locomotionManager->motionPlan->bodyTrajectory.getCOMPositionAtTimeIndex(timeStep));
			auto bodyPosObj = make_shared<BodyFrameObjective>();
			bodyPosObj->sampleNum = timeStep;
			bodyPosObj->pos = P3D(widget->getPos());
			bodyPosObj->phase = moptParams.phase;
			locomotionManager->motionPlan->BodyFrameObjectives.push_back(bodyPosObj);
			COMwidget2constraint[widget] = bodyPosObj;
		}
	}
	return false;
}

bool MOPTWindow::onMouseMoveEvent(double xPos, double yPos){
	if (initialized) {
		if (showFFPViewer && (ffpViewer->mouseIsWithinWindow(xPos, yPos) || ffpViewer->isDragging()))
			if (ffpViewer->onMouseMoveEvent(xPos, yPos)) return true;
	}



	pushViewportTransformation();
	Ray ray = getRayFromScreenCoords(xPos, yPos);
	popViewportTransformation();

	RobotState oldState(robot);
	RobotState robotState(robot);
	locomotionManager->motionPlan->robotStateTrajectory.getRobotPoseAt(moptParams.phase, robotState);
	robot->setState(&robotState);
	updateJointVelocityProfileWindowOnMouseMove(ray, xPos, yPos);
	robot->setState(&oldState);

	DynamicArray<LocomotionEngine_EndEffectorTrajectory> &EET = locomotionManager->motionPlan->endEffectorTrajectories;

	endEffectorInd = -1;
	for (uint i = 0; i < EET.size(); i++) {
		EET[i].isHighlighted = false;
		P3D p;
		P3D EEPos = EET[i].getEEPositionAt(moptParams.phase);
		double dist = ray.getDistanceToPoint(EEPos, &p);
		double tVal = ray.getRayParameterFor(p);
		if (dist < 0.01 * 1.2 && tVal < DBL_MAX) {
			EET[i].isHighlighted = true;
			endEffectorInd = i;
			break;
		}
	}
	pushViewportTransformation();
	for (auto widget : EEwidgets)
		if (widget->onMouseMoveEvent(xPos, yPos))
		{
			EEwidget2constraint[widget]->pos = widget->pos;
			popViewportTransformation();
			return true;
		}
	for (auto widget : COMWidgets)
		if (widget->onMouseMoveEvent(xPos, yPos))
		{
			COMwidget2constraint[widget]->pos = widget->getPos();
			COMwidget2constraint[widget]->orientation = widget->getOrientation().toAxisAngle();
			popViewportTransformation();
			return true;
		}
	popViewportTransformation();

	return GLWindow3D::onMouseMoveEvent(xPos, yPos);
}

void MOPTWindow::updateJointVelocityProfileWindowOnMouseMove(Ray &ray, double xPos, double yPos)
{
	return;

	Joint* joint;
	dVector velocity;
	double tMinJ = DBL_MAX;

	int i;
	for (i = 0; i < robot->getJointCount(); i++) {
		P3D p;
		double dist = ray.getDistanceToPoint(robot->getJoint(i)->getWorldPosition(), &p);
		double tVal = ray.getRayParameterFor(p);
		if (dist < robot->getJoint(i)->parent->abstractViewCylinderRadius * 1.2 && tVal < tMinJ) {
			tMinJ = tVal;
			joint = robot->getJoint(i);
			locomotionManager->motionPlan->getJointAngleVelocityProfile(velocity, i);
			break;
		}
	}

	using namespace nanogui;
	if (i == robot->getJointCount())
	{
		if (velocityProfileWindow != nullptr)
		{
			velocityProfileWindow->dispose();
			velocityProfileWindow = nullptr;
		}
	}
	else
	{
		if (velocityProfileWindow == nullptr)
		{
			velocityProfileWindow = new Window(theApp->menuScreen, "Velocity Profile");
			velocityProfileWindow->setWidth(300);
			velocityProfileWindow->setLayout(new GroupLayout());
			velocityProfileGraph = velocityProfileWindow->add<Graph>("Velocity");
		}
		velocityProfileWindow->setPosition(Eigen::Vector2i(xPos / 1.5, yPos / 1.5));
		velocityProfileGraph->setValues(velocity.cast<float>());

		theApp->menuScreen->performLayout();
	}
}

bool MOPTWindow::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos)
{
	if (initialized) {
		if (showFFPViewer && (ffpViewer->mouseIsWithinWindow(xPos, yPos) || ffpViewer->isDragging()))
			if (ffpViewer->onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;
	}
	if (action == GLFW_PRESS && endEffectorInd > -1)
	{
		int timeStep = (int)round(moptParams.phase*nTimeSteps);

		for (const auto &EEPosObj: locomotionManager->motionPlan->EEPosObjectives)
			if(EEPosObj->endEffectorInd == endEffectorInd && EEPosObj->sampleNum == timeStep)
				return GLWindow3D::onMouseButtonEvent(button, action, mods, xPos, yPos);

		auto widget = std::make_shared<TranslateWidget>(AXIS_X | AXIS_Z);
		EEwidgets.push_back(widget);
		Logger::consolePrint("Picked end effector %d at time step %d", endEffectorInd, timeStep);
		widget->pos = locomotionManager->motionPlan->endEffectorTrajectories[endEffectorInd].EEPos[timeStep];
		auto EEPosObj = make_shared<EndEffectorPositionObjective>();
		EEPosObj->endEffectorInd = endEffectorInd;
		EEPosObj->sampleNum = timeStep;
		EEPosObj->pos = P3D(widget->pos);
		EEPosObj->phase = moptParams.phase;
		locomotionManager->motionPlan->EEPosObjectives.push_back(EEPosObj);
		EEwidget2constraint[widget] = EEPosObj;
		if (button == GLFW_MOUSE_BUTTON_LEFT) // make it temporary
		{

		}
	}
	return GLWindow3D::onMouseButtonEvent(button, action, mods, xPos, yPos);
}

void MOPTWindow::setViewportParameters(int posX, int posY, int sizeX, int sizeY){
	GLWindow3D::setViewportParameters(posX, posY, sizeX, sizeY);
	ffpViewer->setViewportParameters(posX, (int)(sizeY * 3.0 / 4), sizeX, (int)(sizeY / 4.0));
}
