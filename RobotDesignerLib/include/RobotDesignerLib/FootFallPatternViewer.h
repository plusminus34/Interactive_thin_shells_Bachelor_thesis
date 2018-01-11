#pragma once

#include <GUILib/GLWindow2D.h>

class FootFallPattern;

class FootFallPatternViewer : public GLWindow2D {
public:
	//this is the gait we will be using for the preview...
	FootFallPattern* ffp;

	bool cursorMovable = true;
	bool ffpMovable = true;
	double cursorPosition;

	int selectedLimbIndex;
	int selectedColIndex;
	int oldCol;
	int selectedRowIndex;

private:
	double labelsStart;
	double boxStart;
	double boxLength;

public:
	FootFallPatternViewer(int posX, int posY, int sizeX, int sizeY);

	virtual ~FootFallPatternViewer(void);

	virtual void draw();

	//triggered when mouse buttons are pressed
	virtual bool onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos);

	//triggered when mouse moves
	virtual bool onMouseMoveEvent(double xPos, double yPos);

	//any time a physical key is pressed, this event will trigger. Useful for reading off special keys...
	virtual bool onKeyEvent(int key, int action, int mods);
};


