#pragma once
#include <Utils/Logger.h>
#include "InteractiveWidget.h"
#include <MathLib/Ray.h>
#include "GlobalMouseState.h"
#include "GLIncludes.h"
#include "GLUtils.h"
#include "GLTexture.h"


/**
  * Used for any window that needs to be displayed in the OpenGL window
  */
class GLWindow : public InteractiveWidget {
protected:

	// The viewport of the window
	int viewportX, viewportY, viewportWidth, viewportHeight;

	// convert from global window coordinates to viewport coordinates
	double getViewportXFromWindowX(double wX);
	double getViewportYFromWindowY(double wY);
	// converts viewport coordinates to a relative position within the sub window
	double getRelativeXFromViewportX(double vpX);
	double getRelativeYFromViewportY(double vpY);
	// converts relative subwindow coordinates to viewport coordinates
	double getViewportXFromRelativeX(double relX);
	double getViewportYFromRelativeY(double relY);

	virtual void drawBorders(int lineThickness = 8);

public:
	/**
		Default constructor
	*/
	GLWindow(int posX, int posY, int sizeX, int sizeY);
	GLWindow();
	virtual ~GLWindow() {}

	//sets viewport parameters
	virtual void setViewportParameters(int posX, int posY, int sizeX, int sizeY);

	//all these methods should returns true if the event is processed, false otherwise...
	//any time a physical key is pressed, this event will trigger. Useful for reading off special keys...
	virtual bool onKeyEvent(int key, int action, int mods) { return false; }
	//this one gets triggered on UNICODE characters only...
	virtual bool onCharacterPressedEvent(int key, int mods) { return false; }

	bool isActive() {
		return mouseIsWithinWindow(GlobalMouseState::lastMouseX, GlobalMouseState::lastMouseY);
	}

	bool isSelected() {
		return selected;
	}

	bool isClicked() {
		return clicked;
	}

	void updateSelectedFlag(int button, int action) {
		if (isActive())
			if (action == GLFW_PRESS) {
				selected = !selected;
			}
	}

	void updateClickedFlag(int button, int action) {
		clicked = false;
		if (isActive())
			if (action == GLFW_PRESS) {
				clicked = true;
			}
	}

	//triggered when mouse buttons are pressed
	virtual bool onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
		return false;
	}

	//triggered when mouse moves
	virtual bool onMouseMoveEvent(double xPos, double yPos) { return false; }
	//triggered when using the mouse wheel
	virtual bool onMouseWheelScrollEvent(double xOffset, double yOffset) { return false; }

	virtual void draw() {}

	bool mouseIsWithinWindow(double mouseX, double mouseY);

	bool selected = false;
	bool clicked = false;



};

