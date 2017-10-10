#pragma once
#define GLFW_INCLUDE_GLU
#include <GUILib/GLWindow3D.h>
#include <GUILib/GLCamera.h>
#include <RobotDesignerLib/RMC.h>
#include <GUILib/GLTrackingCamera.h>

/**
* Used for any window showing cylinder that needs to be displayed in the OpenGL window
*/

class ModuleDisplayWindow : public GLWindow3D {
protected:
public:

	RMC* rmc = NULL;

	// constructors
	ModuleDisplayWindow(RMC* rmc, int x, int y, int w, int h);
	ModuleDisplayWindow(RMC* rmc);
	ModuleDisplayWindow();

	// destructor
	virtual ~ModuleDisplayWindow() {};

	virtual void drawScene();
	virtual void setupLights();

};

