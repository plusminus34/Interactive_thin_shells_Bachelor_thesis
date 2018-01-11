#include <GUILib/GLWindow2D.h>
#include <Utils/Utils.h>
#include <MathLib/MathLib.h>
#include <GUILib/GLIncludes.h>

//TODO: Just like GLWindow3D, this predraw should be separated into setting up viewport transformations and then drawing things...

/**
	Default constructor
*/
GLWindow2D::GLWindow2D( int posX, int posY, int sizeX, int sizeY ) : GLWindow(posX, posY, sizeX, sizeY) {
}

GLWindow2D::GLWindow2D() : GLWindow(){
}

void GLWindow2D::pushViewportTransformation() {
	double minX = 0;
	double maxX = 1;
	double minY = 0;
	double maxY = 1;

	glPushAttrib(GL_TRANSFORM_BIT | GL_VIEWPORT_BIT);
	// set viewport
	glViewport(viewportX, viewportY, viewportWidth, viewportHeight);

	// Save projection matrix and sets it to a simple orthogonal projection
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(minX, maxX, minY, maxY);

	// Save model-view matrix and sets it to identity
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

}

void GLWindow2D::popViewportTransformation() {
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glPopAttrib();

}

// sets up the window for drawing
void GLWindow2D::preDraw() {
	double minX = 0;
	double maxX = 1;
	double minY = 0;
	double maxY = 1;

	glPushAttrib(GL_ENABLE_BIT | GL_TRANSFORM_BIT | GL_VIEWPORT_BIT | GL_SCISSOR_BIT | GL_POINT_BIT | GL_LINE_BIT | GL_TRANSFORM_BIT);

	pushViewportTransformation();

	glDisable(GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_DEPTH_TEST);

	glColor4d(1, 1, 1, 1);
	glPointSize(1);
	glLineWidth(1);

	glScissor(viewportX, viewportY, viewportWidth, viewportHeight);
	glEnable(GL_SCISSOR_TEST);

	// Draw on a clean background
	glClear(GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

	glEnable(GL_BLEND);

	glColor4d(bgColorR, bgColorG, bgColorB, bgColorA);

	glBegin(GL_QUADS);
	glVertex2d(minX, minY);
	glVertex2d(maxX, minY);
	glVertex2d(maxX, maxY);
	glVertex2d(minX, maxY);
	glEnd();

	glLineWidth(5);
	glColor3d(0, 0, 0);
	glBegin(GL_LINE_LOOP);
		glVertex2d(minX, minY);
		glVertex2d(minX, maxY);
		glVertex2d(maxX, maxY);
		glVertex2d(maxX, minY);
	glEnd();

	glLineWidth(1);

	// Draw a bit of a border as a visual indication that the window is or is not active...
	if (isActive() || isSelected()) {
		drawBorders(1);
	}

}

// clean up
void GLWindow2D::postDraw() {
	popViewportTransformation();
	glPopAttrib();
}

void GLWindow2D::draw() {
	preDraw();

//	glColor4d(bgColorR, bgColorG, bgColorB, bgColorA);
//	glBegin(GL_QUADS);
//	glVertex2d(minX, minY);
//	glVertex2d(maxX, minY);
//	glVertex2d(maxX, maxY);
//	glVertex2d(minX, maxY);
//	glEnd();

	postDraw();
}
