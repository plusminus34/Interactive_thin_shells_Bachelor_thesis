#include <GUILib/GLApplication.h>

#include <GUILib/GLWindow3D.h>
#include <Utils/Utils.h>
#include <GUILib/GLUtils.h>

#include <GUILib/GLContentManager.h>
#include <GUILib/GLMesh.h>
#include <GUILib/GLCamera.h>
#include <GUILib/GLTrackingCamera.h>

/**
	Default constructor
*/
GLWindow3D::GLWindow3D(int posX, int posY, int sizeX, int sizeY) : GLWindow(posX, posY, sizeX, sizeY) {
	init();
}

GLWindow3D::GLWindow3D() : GLWindow() {
	init();
}

void GLWindow3D::draw() {
	preDraw();
	if (showReflections && showGroundPlane) drawReflections();

	drawScene();
	drawBorders();
	postDraw();
}

void GLWindow3D::init() {
	camera = new GLTrackingCamera();
}

void GLWindow3D::preDraw() {
	glPushAttrib(GL_ENABLE_BIT | GL_TRANSFORM_BIT | GL_VIEWPORT_BIT | GL_SCISSOR_BIT | GL_POINT_BIT | GL_LINE_BIT | GL_TRANSFORM_BIT);

	glDisable(GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);

	// set viewport
	glViewport(viewportX, viewportY, viewportWidth, viewportHeight);

	glMatrixMode(GL_PROJECTION);						// Select The Projection Matrix
	glPushMatrix();
	glLoadIdentity();									// Reset The Projection Matrix
	gluPerspective(45.0, (double)viewportWidth / viewportHeight, 0.05, 150.0);
	glMatrixMode(GL_MODELVIEW);							// Select The Modelview Matrix
	glPushMatrix();
	glLoadIdentity();									// Reset The Modelview Matrix

	glEnable(GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);

	if (camera)
		camera->applyCameraTransformations();

	setupLights();
}

// clean up
void GLWindow3D::postDraw() {
	glMatrixMode(GL_MODELVIEW);							// Select The Modelview Matrix
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);						// Select The Projection Matrix
	glPopMatrix();
	glPopAttrib();
}

//all these methods should returns true if the event is processed, false otherwise...

//triggered when mouse moves
bool GLWindow3D::onMouseMoveEvent(double xPos, double yPos) {
	if (camera)
		if (camera->onMouseMoveEvent(xPos, yPos) == true)
			return true;
	return false;
}

//triggered when mouse buttons are pressed
bool GLWindow3D::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
	if (camera) {
		if(camera->onMouseButtonEvent(button, action, mods, xPos, yPos))
			return true;
	}

	return false;
}

//triggered when using the mouse wheel
bool GLWindow3D::onMouseWheelScrollEvent(double xOffset, double yOffset) {
	if (camera)
		if (camera->onMouseWheelScrollEvent(xOffset, yOffset))
			return true;
	return false;
}

