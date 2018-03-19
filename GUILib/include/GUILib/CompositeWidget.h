#pragma once

#include "TransformWidget.h"
#include <GUILib/TranslateWidget.h>
#include <GUILib/RotateWidgetV2.h>

#include <MathLib/V3D.h>
#include <MathLib/Quaternion.h>
#include <MathLib/Ray.h>



/**
	This class draws a widget to translate and rotate a point. It also handles mouse
	interactions.
*/
class CompositeWidget : public TransformWidget {
public:
	P3D getPos();
	void setPos(P3D pos);

private:
	TranslateWidget translateWidget;
	RotateWidgetV2 rotateWidget;
	// Translation
	P3D pos;

public:
	CompositeWidget(uint translateAxes = AXIS_X | AXIS_Y | AXIS_Z, uint rotationAxes = AXIS_X | AXIS_Y | AXIS_Z);
	virtual ~CompositeWidget();

	/**
		Draws the widget
	*/
	virtual void draw();
	virtual bool isPicked();


	//all these methods should returns true if the event is processed, false otherwise...
	//any time a physical key is pressed, this event will trigger. Useful for reading off special keys...
	virtual bool onKeyEvent(int key, int action, int mods) { return false; }
	//this one gets triggered on UNICODE characters only...
	virtual bool onCharacterPressedEvent(int key, int mods) { return false; }
	//triggered when mouse buttons are pressed
	virtual bool onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) { 
		return active;
	}
	//triggered when mouse moves
	virtual bool onMouseMoveEvent(double xPos, double yPos);
	//triggered when using the mouse wheel
	virtual bool onMouseWheelScrollEvent(double xOffset, double yOffset) { return false; }
};

