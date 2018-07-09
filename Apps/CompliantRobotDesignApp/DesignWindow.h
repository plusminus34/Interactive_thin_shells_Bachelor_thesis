#pragma once

#include <GUILib/GLApplication.h>
#include <GUILib/GLWindow3D.h>
#include <GUILib/GLTrackingCamera.h>
#include <GUILib/TranslateWidget.h>
#include <GUILib/CompositeWidget.h>
#include <ControlLib/Robot.h>

#include <memory>

class DesignWindow : public GLWindow3D {
public:
	GLApplication* theApp = NULL;
	Robot* robot = nullptr;

	void addMenuItems();

	double bodyLength = 0.1;


public:
	DesignWindow(int x, int y, int w, int h, GLApplication* glApp);
	~DesignWindow() {}

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
