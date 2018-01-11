#pragma once
#include "GLWindow.h"
#include <Utils/Logger.h>
#include <Utils/Timer.h>
#include "GLWindow2D.h"
#include <GUILib/GLMesh.h>
#include <GLFW/glfw3.h>
#include <GUILib/Canvas3D.h>

/**
* Used for any window that needs to be displayed in the OpenGL window
*/
class GLWindow3D : public GLWindow, public Canvas3D {
protected:

	// sets up the window for drawing
	virtual void preDraw();

	// clean up
	virtual void postDraw();

	virtual void pushViewportTransformation();
	virtual void popViewportTransformation();

	void init();

	virtual void drawScene() {};

	virtual void clear();

public:

	// constructors
	GLWindow3D(int x, int y, int w, int h);
	GLWindow3D();

	// destructor
	virtual ~GLWindow3D() {};

	// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
	virtual void draw();


	//IO callbacks...

	//all these methods should returns true if the event is processed, false otherwise...
	//triggered when mouse buttons are pressed
	virtual bool onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos);
	//triggered when mouse moves
	virtual bool onMouseMoveEvent(double xPos, double yPos);

	//triggered when using the mouse wheel
	virtual bool onMouseWheelScrollEvent(double xOffset, double yOffset);
};
