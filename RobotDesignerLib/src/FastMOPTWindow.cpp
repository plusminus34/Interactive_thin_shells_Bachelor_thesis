#pragma warning(disable : 4996)

#include <RobotDesignerLib/FastMOPTWindow.h>
#include <RobotDesignerLib/LocomotionEngineManagerGRF.h>
#include <RobotDesignerLib/LocomotionEngineManagerIP.h>
#include "../Apps/RobotDesignerApp/RobotDesignerApp.h"
#include <RobotDesignerLib/FastMOPTPreplanner.h>

FastMOPTWindow::FastMOPTWindow(int x, int y, int w, int h, BaseRobotControlApp* theApp) : GLWindow3D(x, y, w, h) {
	this->theApp = theApp;

	ffpViewer = new FootFallPatternViewer(x, 0, (int)(w), (int)(h / 4.0));
	ffpViewer->ffp = &footFallPattern;
	ffpViewer->cursorMovable = true;

	dynamic_cast<GLTrackingCamera*>(this->camera)->rotAboutRightAxis = 0.25;
	dynamic_cast<GLTrackingCamera*>(this->camera)->rotAboutUpAxis = 0.95;
	dynamic_cast<GLTrackingCamera*>(this->camera)->camDistance = -1.5;
	
	fmpp = new FastMOPTPreplanner(this);

	showGroundPlane = true;
	showReflections = true;

	forwardSpeedTarget = 1.0;
	sidewaysSpeedTarget = 0;
	bodyHeightTarget = 0;
	turningSpeedTarget = 0;
}

void FastMOPTWindow::addMenuItems() {
	{
		auto tmpVar = theApp->mainMenu->addVariable("Forward Speed", forwardSpeedTarget);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.05);
	}
	{
		auto tmpVar = theApp->mainMenu->addVariable("Sideways Speed", sidewaysSpeedTarget);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.05);
	}
	{
		auto tmpVar = theApp->mainMenu->addVariable("Turning Speed", turningSpeedTarget);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.05);
	}
	{
		auto tmpVar = theApp->mainMenu->addVariable("Body Height", bodyHeightTarget);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.01);
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
		auto tmpVar = theApp->mainMenu->addVariable("gait duration", moptParams.motionPlanDuration);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.05);
	}
}


void FastMOPTWindow::loadRobot(Robot* robot){
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

LocomotionEngineManager* FastMOPTWindow::initializeLocomotionEngine(){
	delete locomotionManager;

	footFallPattern.writeToFile("../out/tmpFFP.ffp");

	//make sure the ffp does not get out of sync with the number of samples in the locomotion engine...
	nTimeSteps = footFallPattern.strideSamplePoints;

	locomotionManager = new LocomotionEngineManagerGRFv3(robot, &footFallPattern, nTimeSteps + 1);

	bodyHeightTarget = locomotionManager->motionPlan->initialRS.getPosition().getComponentAlong(Globals::worldUp);
	robot->setState(&locomotionManager->motionPlan->initialRS);

	locomotionManager->setDefaultOptimizationFlags();
	locomotionManager->energyFunction->regularizer = globalMOPTRegularizer;

	return locomotionManager;
}

void FastMOPTWindow::setAnimationParams(double f, int animationCycle){
	moptParams.phase = f;
	moptParams.gaitCycle = animationCycle;
	ffpViewer->cursorPosition = f;
}

void FastMOPTWindow::loadFFPFromFile(const char* fName){
	footFallPattern.loadFromFile(fName);
}

void FastMOPTWindow::generateMotionPreplan() {
	locomotionManager->printDebugInfo = true;
	locomotionManager->checkDerivatives = true;

	Timer t;

	if (defaultFootFallPattern.stepPatterns.size() < footFallPattern.stepPatterns.size())
		defaultFootFallPattern = footFallPattern;

	RobotState rs(robot);
	fmpp->preplan(&rs);
	fmpp->prepareMOPTPlan(locomotionManager->motionPlan);
	locomotionManager->runMOPTStep(OPT_GRFS);
	locomotionManager->runMOPTStep(OPT_GRFS);
	Logger::consolePrint("It took %lfs to generate motion plan\n", t.timeEllapsed());
}

void FastMOPTWindow::optimizeMotionPlan() {
	Timer t;
	locomotionManager->runMOPTStep(OPT_GRFS | OPT_COM_POSITIONS | OPT_COM_ORIENTATIONS);
//	locomotionManager->runMOPTStep(OPT_COM_POSITIONS);
	Logger::consolePrint("It took %lfs to run optimization step\n", t.timeEllapsed());

//	add a new objective for the COM position/orientation trajectory
}

void FastMOPTWindow::advanceMotionPlanGlobalTime(int nSteps) {
	motionPlanStartTime += nSteps * moptParams.motionPlanDuration / (locomotionManager->motionPlan->nSamplePoints - 1);
}

void FastMOPTWindow::drawScene() {
	glColor3d(1, 1, 1);
	glEnable(GL_LIGHTING);

	fmpp->draw();
	moptParams.drawRobotMesh = moptParams.drawSkeleton = moptParams.drawAxesOfRotation = moptParams.drawWheels = moptParams.drawSupportPolygon =false;
	moptParams.drawContactForces = moptParams.drawEndEffectorTrajectories = moptParams.drawCOMTrajectory = moptParams.drawOrientation = true;

	glTranslated(1, 0, 0);
	locomotionManager->motionPlan->drawMotionPlan(moptParams.phase, moptParams.drawRobotMesh, moptParams.drawSkeleton, moptParams.drawAxesOfRotation, moptParams.drawWheels, moptParams.drawContactForces, moptParams.drawSupportPolygon, moptParams.drawEndEffectorTrajectories, moptParams.drawCOMTrajectory, moptParams.drawOrientation);

//	RobotState animationState = fmpp->getRobotStateAtTime();
//	planTime += 0.001;
//	robot->setState(&animationState);
//	robot->draw(SHOW_ABSTRACT_VIEW);
}

void FastMOPTWindow::drawAuxiliarySceneInfo(){
	glClear(GL_DEPTH_BUFFER_BIT);

	preDraw();
	if (showFFPViewer) ffpViewer->draw();
	postDraw();

	glClear(GL_DEPTH_BUFFER_BIT);

}

//any time a physical key is pressed, this event will trigger. Useful for reading off special keys...
bool FastMOPTWindow::onKeyEvent(int key, int action, int mods) {
	if (ffpViewer && showFFPViewer && ffpViewer->onKeyEvent(key, action, mods))
		return true;
	return GLWindow3D::onKeyEvent(key, action, mods);
}

bool FastMOPTWindow::onMouseMoveEvent(double xPos, double yPos){
	if (showFFPViewer && (ffpViewer->mouseIsWithinWindow(xPos, yPos) || ffpViewer->isDragging()))
		if (ffpViewer->onMouseMoveEvent(xPos, yPos)) return true;

	return GLWindow3D::onMouseMoveEvent(xPos, yPos);
}

bool FastMOPTWindow::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos){
	if (showFFPViewer && (ffpViewer->mouseIsWithinWindow(xPos, yPos) || ffpViewer->isDragging()))
		if (ffpViewer->onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;
	return GLWindow3D::onMouseButtonEvent(button, action, mods, xPos, yPos);
}

void FastMOPTWindow::setViewportParameters(int posX, int posY, int sizeX, int sizeY){
	GLWindow3D::setViewportParameters(posX, posY, sizeX, sizeY);
	ffpViewer->setViewportParameters(posX, (int)(sizeY * 3.0 / 4), sizeX, (int)(sizeY / 4.0));
}

