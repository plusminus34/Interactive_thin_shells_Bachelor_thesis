#pragma once

#include <../Apps/RobotDesignerApp/RobotDesignerApp.h>

class IntelligentRobotEditingWindow : public GLWindow3D {
public:
	RobotDesignerApp* rdApp;

	RigidBody* highlightedRigidBody = NULL;

	void addMenuItems();

public:
	IntelligentRobotEditingWindow(int x, int y, int w, int h, RobotDesignerApp* rdApp);
	~IntelligentRobotEditingWindow();

	virtual void drawScene();
	virtual void drawAuxiliarySceneInfo();
	virtual void setupLights();

	virtual bool onMouseMoveEvent(double xPos, double yPos);
	virtual bool onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos);
	//triggered when using the mouse wheel
	virtual bool onMouseWheelScrollEvent(double xOffset, double yOffset);



	virtual void setViewportParameters(int posX, int posY, int sizeX, int sizeY);
};

