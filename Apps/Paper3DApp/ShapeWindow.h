#pragma once

#include <GUILib/GLWindow2D.h>

#include "Paper3DApp.h"

class ShapeWindow: public GLWindow2D{
protected:
	Paper3DApp* paperApp;

public:
	ShapeWindow(int x, int y, int w, int h, Paper3DApp *glApp);
	~ShapeWindow();

	virtual void drawScene();
	virtual void drawAuxiliarySceneInfo();
};

