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

void GLWindow3D::drawReflections() {
	glDisable(GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);
	glClear(GL_STENCIL_BUFFER_BIT);

	glEnable(GL_STENCIL_TEST); //Enable using the stencil buffer
	glColorMask(0, 0, 0, 0); //Disable drawing colors to the screen
	glDisable(GL_DEPTH_TEST); //Disable depth testing
	glStencilFunc(GL_ALWAYS, 1, 1); //Make the stencil test always pass
									//Make pixels in the stencil buffer be set to 1 when the stencil test passes
	glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
	drawGround();

	glColorMask(1, 1, 1, 1); //Enable drawing colors to the screen
	glEnable(GL_DEPTH_TEST); //Enable depth testing
							 //Make the stencil test pass only when the pixel is 1 in the stencil buffer
	glStencilFunc(GL_EQUAL, 1, 1);
	glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP); //Make the stencil buffer not change
	glPushMatrix();

	V3D scale(1, 1, 1);
	scale -= Globals::groundPlane.n * 2;

	glScalef((float)scale[0], (float)scale[1], (float)scale[2]);

	glEnable(GL_CLIP_PLANE0);

	double plane[4];
	Globals::groundPlane.getCartesianEquationCoefficients(plane[0], plane[1], plane[2], plane[3]);
	glClipPlane(GL_CLIP_PLANE0, plane);
	drawScene();
	glPopMatrix();

	glDisable(GL_CLIP_PLANE0);

	glDisable(GL_STENCIL_TEST);

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);
	glDisable(GL_LIGHTING);
	glColor4f(1, 1, 1, 0.85f);
	drawGround();
}

void GLWindow3D::drawGround() {
	drawTexturedGround(GLContentManager::getTexture("../data/textures/ground_TileLight2.bmp"));
}


void GLWindow3D::init() {
	camera = new GLTrackingCamera();
}

void GLWindow3D::setupLights() {

	GLfloat bright[] = { 0.8f, 0.8f, 0.8f, 1.0f };
	GLfloat mediumbright[] = { 0.3f, 0.3f, 0.3f, 1.0f };

	glLightfv(GL_LIGHT1, GL_DIFFUSE, bright);
	glLightfv(GL_LIGHT2, GL_DIFFUSE, mediumbright);

	GLfloat light0_position[] = { 0.0f, 10000.0f, 10000.0f, 0.0f };
	GLfloat light0_direction[] = { 0.0f, -10000.0f, -10000.0f, 0.0f };

	GLfloat light1_position[] = { 0.0f, 10000.0f, -10000.0f, 0.0f };
	GLfloat light1_direction[] = { 0.0f, -10000.0f, 10000.0f, 0.0f };

	GLfloat light2_position[] = { 0.0f, -10000.0f, 0.0f, 0.0f };
	GLfloat light2_direction[] = { 0.0f, 10000.0f, -0.0f, 0.0f };

	glLightfv(GL_LIGHT0, GL_POSITION, light0_position);
	glLightfv(GL_LIGHT1, GL_POSITION, light1_position);
	glLightfv(GL_LIGHT2, GL_POSITION, light2_position);

	glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, light0_direction);
	glLightfv(GL_LIGHT1, GL_SPOT_DIRECTION, light1_direction);
	glLightfv(GL_LIGHT2, GL_SPOT_DIRECTION, light2_direction);

	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	glEnable(GL_LIGHT2);
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

