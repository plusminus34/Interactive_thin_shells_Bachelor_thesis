#pragma once

#include "GLWindow.h"

/**
  * Used for any window that needs to be displayed in the OpenGL window
  */
class GLWindow2D : public GLWindow {
protected:
	//relative coordinates normalized to 0..1, with a square aspect ratio (i.e. largest dimension goes to 1, smaller dimension is relative to larger one)
	double bgColorR = 1, bgColorG = 1, bgColorB = 1, bgColorA = 0.5;

	bool dragging = false;


protected:


	// sets up the window for drawing
	virtual void preDraw();

	// clean up
	virtual void postDraw();

public:

	GLWindow2D( int posX, int posY, int sizeX, int sizeY );
	GLWindow2D();

	virtual ~GLWindow2D() {}

	virtual void draw();

	//all these methods should returns true if the event is processed, false otherwise...
	//any time a physical key is pressed, this event will trigger. Useful for reading off special keys...
	virtual bool onKeyEvent(int key, int action, int mods) { return false; }
	//this one gets triggered on UNICODE characters only...
	virtual bool onCharacterPressedEvent(int key, int mods) { return false; }
	//triggered when mouse buttons are pressed
	virtual bool onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
		if (isActive() && mods & GLFW_MOD_CONTROL) {
			dragging = true;
			return true;
		}else
			dragging = false;

		return false; 
	}

	virtual bool isDragging() {
		return dragging;
	}

	//triggered when mouse moves
	virtual bool onMouseMoveEvent(double xPos, double yPos) { 
		if (dragging && GlobalMouseState::lButtonPressed) {
			this->viewportX -= (int)GlobalMouseState::mouseMoveX;
			this->viewportY -= (int)GlobalMouseState::mouseMoveY;
			return true;
		}
		return false; 
	}

	//triggered when using the mouse wheel
	virtual bool onMouseWheelScrollEvent(double xOffset, double yOffset) { return false; }

};

