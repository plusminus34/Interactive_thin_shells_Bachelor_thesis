#include <GUILib/GLUtils.h>
#include <GUILib/CompositeWidget.h>
#include <MathLib/MathLib.h>
#include <Utils/Logger.h>
#include <GUILib/GlobalMouseState.h>


CompositeWidget::CompositeWidget(uint translationAxes, uint rotationAxes) : 
	translateWidget(translationAxes),
	rotateWidget(rotationAxes) { }

P3D CompositeWidget::getPos()
{
	return pos;
}

Quaternion CompositeWidget::getOrientation()
{
	return orientation;
}

void CompositeWidget::setPos(P3D pos)
{
	this->pos = pos;
	translateWidget.pos = pos;
	rotateWidget.pos = pos;
}

CompositeWidget::~CompositeWidget(){
}

bool CompositeWidget::isPicked() {
	if (!visible)
		return false;
	if (translateWidget.isPicked())
		return true;
	if (rotateWidget.isPicked())
		return true;
	return false;
}


void CompositeWidget::draw(){
	if (visible) {
		translateWidget.draw();
		rotateWidget.draw();
	}
}

//triggered when mouse moves
bool CompositeWidget::onMouseMoveEvent(double xPos, double yPos) {
	if (translateWidget.onMouseMoveEvent(xPos, yPos))
	{
		pos = translateWidget.pos;
		rotateWidget.pos = pos;
		return true;
	}
	if (rotateWidget.onMouseMoveEvent(xPos, yPos))
	{
		orientation = rotateWidget.orientation;
		return true;
	}
	return false;
}