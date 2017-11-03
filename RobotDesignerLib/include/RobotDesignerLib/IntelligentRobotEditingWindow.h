#pragma once

#include <../Apps/RobotDesignerApp/RobotDesignerApp.h>

class IntelligentRobotEditingWindow : public GLWindow3D {
public:
	RobotDesignerApp* rdApp;

	void addMenuItems();

public:
	IntelligentRobotEditingWindow(int x, int y, int w, int h, RobotDesignerApp* rdApp);
	~IntelligentRobotEditingWindow();

	virtual void drawScene();
	virtual void drawAuxiliarySceneInfo();
	virtual void setupLights();

	virtual bool onMouseMoveEvent(double xPos, double yPos);
	virtual bool onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos);

	virtual void setViewportParameters(int posX, int posY, int sizeX, int sizeY);
};

