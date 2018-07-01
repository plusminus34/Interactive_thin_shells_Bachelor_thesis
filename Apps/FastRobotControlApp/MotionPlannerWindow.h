#pragma once

#include <GUILib/GLApplication.h>
#include <GUILib/GLWindow3D.h>
#include <GUILib/GLTrackingCamera.h>
#include <GUILib/TranslateWidget.h>
#include <GUILib/CompositeWidget.h>
#include <RobotDesignerLib/BaseRobotControlApp.h>
#include <RobotDesignerLib/MoptWindow.h>

#include <memory>

class RobotDesignerApp;
class MotionPlanner;

class MotionPlannerWindow : public GLWindow3D {
public:
	MotionPlanner* motionPlanner;
	BaseRobotControlApp* theApp;
	Robot* robot = nullptr;
	FootFallPatternViewer* ffpViewer = nullptr;
	bool showFFPViewer = true;

	void addMenuItems();

public:
	MotionPlannerWindow(int x, int y, int w, int h, BaseRobotControlApp* glApp);
	~MotionPlannerWindow() {}

	void loadRobot(Robot* robot);

	LocomotionEngineManager* initializeLocomotionEngine();

	void setAnimationParams(double f, int animationCycle);

	void loadFFPFromFile(const char* fName);

	virtual void drawScene();
	virtual void drawAuxiliarySceneInfo();

	virtual bool onMouseMoveEvent(double xPos, double yPos);

	virtual bool onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos);

	//any time a physical key is pressed, this event will trigger. Useful for reading off special keys...
	virtual bool onKeyEvent(int key, int action, int mods);

	virtual void setViewportParameters(int posX, int posY, int sizeX, int sizeY);
	virtual void drawGround() {
		drawTexturedGround(GLContentManager::getTexture("../data/textures/lightGray.bmp"));
	}
private:

};
