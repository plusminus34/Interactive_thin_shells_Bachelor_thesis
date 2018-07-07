#pragma warning(disable : 4996)

#include "MotionPlannerWindow.h"
#include <RobotDesignerLib/LocomotionEngineManagerGRF.h>
#include <RobotDesignerLib/LocomotionEngineManagerIP.h>
#include "MotionPlanner.h"

MotionPlannerWindow::MotionPlannerWindow(int x, int y, int w, int h, BaseRobotControlApp* theApp) : GLWindow3D(x, y, w, h) {
	this->theApp = theApp;
	motionPlanner = new MotionPlanner();

	ffpViewer = new FootFallPatternViewer(x, 0, (int)(w), (int)(h / 4.0));
	ffpViewer->cursorMovable = true;
	ffpViewer->ffp = &motionPlanner->currentMOPTFootFallPattern;

	dynamic_cast<GLTrackingCamera*>(this->camera)->rotAboutRightAxis = 0.25;
	dynamic_cast<GLTrackingCamera*>(this->camera)->rotAboutUpAxis = 0.95;
	dynamic_cast<GLTrackingCamera*>(this->camera)->camDistance = -1.5;

	showGroundPlane = true;
	showReflections = true;
}

void MotionPlannerWindow::addMenuItems() {
	{
		auto tmpVar = theApp->mainMenu->addVariable("Forward Speed", motionPlanner->forwardSpeedTarget);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.05);
	}
	{
		auto tmpVar = theApp->mainMenu->addVariable("Sideways Speed", motionPlanner->sidewaysSpeedTarget);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.05);
	}
	{
		auto tmpVar = theApp->mainMenu->addVariable("Turning Speed", motionPlanner->turningSpeedTarget);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.05);
	}
	{
		auto tmpVar = theApp->mainMenu->addVariable("Body Height", motionPlanner->bodyHeightTarget);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.01);
	}

	{
		auto tmpVar = theApp->mainMenu->addVariable("globalMOPTRegularizer", motionPlanner->globalMOPTRegularizer);
		tmpVar->setSpinnable(false); tmpVar->setMinValue(0); tmpVar->setMaxValue(100);
	}

	{
		auto tmpVar = theApp->mainMenu->addVariable("swingFootHeight", motionPlanner->swingFootHeight);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.01);
	}

	{
		auto tmpVar = theApp->mainMenu->addVariable("gait duration", motionPlanner->motionPlanDuration);
		tmpVar->setSpinnable(true); tmpVar->setValueIncrement(0.05);
	}
}


void MotionPlannerWindow::loadRobot(Robot* robot){
	this->robot = robot;
	motionPlanner->robot = robot;

	int nLegs = robot->bFrame->limbs.size();

	// ******************* footfall patern *******************
	motionPlanner->currentMOPTFootFallPattern = FootFallPattern();
	motionPlanner->currentMOPTFootFallPattern.strideSamplePoints = 10;
}

LocomotionEngineManager* MotionPlannerWindow::initializeLocomotionEngine(){
	return motionPlanner->initializeMOPTEngine();
}

void MotionPlannerWindow::setAnimationParams(double f, int animationCycle){
	ffpViewer->cursorPosition = f;
}

void MotionPlannerWindow::loadFFPFromFile(const char* fName){
	motionPlanner->defaultFootFallPattern.loadFromFile(fName);
	motionPlanner->defaultFootFallPattern.writeToFile("..\\out\\tmpFFP.ffp");
	motionPlanner->currentMOPTFootFallPattern = motionPlanner->defaultFootFallPattern;
}

void MotionPlannerWindow::loadMotionPlanFromFile(const char* fName) {
	motionPlanner->locomotionManager->motionPlan->readParamsFromFile(fName);
	motionPlanner->locomotionManager->motionPlan->syncFootFallPatternWithMotionPlan(motionPlanner->currentMOPTFootFallPattern);
	motionPlanner->defaultFootFallPattern = motionPlanner->currentMOPTFootFallPattern;
	motionPlanner->currentMOPTFootFallPattern.writeToFile("..\\out\\tmpFFP.ffp");
}

void MotionPlannerWindow::drawScene() {
	glColor3d(1, 1, 1);
	glEnable(GL_LIGHTING);

	if (motionPlanner) {
		ffpViewer->ffp = &motionPlanner->currentMOPTFootFallPattern;

		motionPlanner->draw();
		V3D offset = motionPlanner->plannerStartState.getOrientation().rotate(V3D(0.5, 0, 0));
		glTranslated(offset.x(), offset.y(), offset.z());
		motionPlanner->locomotionManager->motionPlan->drawMotionPlan(ffpViewer->cursorPosition, false, false, false, false, true, true, true, true, true);
	}

//	RobotState animationState = motionPlanner->getRobotStateAtTime();
//	planTime += 0.001;
//	robot->setState(&animationState);
//	robot->draw(SHOW_ABSTRACT_VIEW);
}

void MotionPlannerWindow::drawAuxiliarySceneInfo(){
	glClear(GL_DEPTH_BUFFER_BIT);

	preDraw();
	if (showFFPViewer) ffpViewer->draw();
	postDraw();

	glClear(GL_DEPTH_BUFFER_BIT);

}

//any time a physical key is pressed, this event will trigger. Useful for reading off special keys...
bool MotionPlannerWindow::onKeyEvent(int key, int action, int mods) {
	if (ffpViewer && showFFPViewer && ffpViewer->onKeyEvent(key, action, mods))
		return true;
	return GLWindow3D::onKeyEvent(key, action, mods);
}

bool MotionPlannerWindow::onMouseMoveEvent(double xPos, double yPos){
	if (showFFPViewer && (ffpViewer->mouseIsWithinWindow(xPos, yPos) || ffpViewer->isDragging()))
		if (ffpViewer->onMouseMoveEvent(xPos, yPos)) return true;

	return GLWindow3D::onMouseMoveEvent(xPos, yPos);
}

bool MotionPlannerWindow::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos){
	if (showFFPViewer && (ffpViewer->mouseIsWithinWindow(xPos, yPos) || ffpViewer->isDragging()))
		if (ffpViewer->onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;
	return GLWindow3D::onMouseButtonEvent(button, action, mods, xPos, yPos);
}

void MotionPlannerWindow::setViewportParameters(int posX, int posY, int sizeX, int sizeY){
	GLWindow3D::setViewportParameters(posX, posY, sizeX, sizeY);
	ffpViewer->setViewportParameters(posX, (int)(sizeY * 3.0 / 4), sizeX, (int)(sizeY / 4.0));
}

