#pragma once

#include <GUILib/GLWindow3D.h>

#include "Paper3DApp.h"

class SimulationWindow: public GLWindow3D{
protected:
	Paper3DApp* paperApp;

	//Dragging nodes
	Plane draggingPlane;
	int selectedNodeID;
	Ray lastClickedRay;

public:
	SimulationWindow(int x, int y, int w, int h, Paper3DApp *glApp);
	~SimulationWindow();

	virtual void drawScene();
	virtual void drawAuxiliarySceneInfo();

	virtual bool onMouseMoveEvent(double xPos, double yPos);
	virtual bool onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos);
};

