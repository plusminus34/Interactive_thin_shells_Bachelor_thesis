#pragma once

#include <GUILib/GLWindow3D.h>

#include "Paper3DApp.h"

class ShapeWindow: public GLWindow3D{
protected:
	Paper3DApp* paperApp;

	bool dragging;
	double xDrag, yDrag;

public:
	ShapeWindow(int x, int y, int w, int h, Paper3DApp *glApp);
	~ShapeWindow();

	virtual void drawScene();
	virtual void drawAuxiliarySceneInfo();

	virtual bool onMouseMoveEvent(double xPos, double yPos);
};

