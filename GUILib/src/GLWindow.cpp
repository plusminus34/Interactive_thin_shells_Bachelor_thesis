#include <GUILib/GLWindow.h>
#include <GUILib/GLApplication.h>
#include <Utils/Utils.h>
#include <MathLib/MathLib.h>
#include <GUILib/GLShaderMaterial.h>

/**
	Default constructor
*/
GLWindow::GLWindow( int posX, int posY, int sizeX, int sizeY ) {
	setViewportParameters(posX, posY, sizeX, sizeY);
}

GLWindow::GLWindow() {
	setViewportParameters(0,0,100,100);
}

//sets viewport parameters
void GLWindow::setViewportParameters(int posX, int posY, int sizeX, int sizeY){
	this->viewportX = posX;
	this->viewportY = posY;
	this->viewportWidth = sizeX;
	this->viewportHeight = sizeY;
}
double GLWindow::getViewportXFromWindowX(double wX) {
	return wX - viewportX;
}

// NOTE: Viewport origin is bottom-left of the viewport, mouse origin is top-left of the containing window
double GLWindow::getViewportYFromWindowY(double wY) {
	return GLApplication::getGLAppInstance()->getMainWindowHeight() - wY - viewportY;
}

// converts viewport coordinates to a relative position within the sub window
double GLWindow::getRelativeXFromViewportX(double vpX) {
	return (vpX) / (viewportWidth);
}

double GLWindow::getRelativeYFromViewportY(double vpY) {
	return (vpY) / (viewportHeight);
}

double GLWindow::getViewportXFromRelativeX(double relX) {
	return relX*viewportWidth;
}

double GLWindow::getViewportYFromRelativeY(double relY) {
	return relY*viewportHeight;
}

bool GLWindow::mouseIsWithinWindow(double mouseX, double mouseY){
	double vX = getViewportXFromWindowX(mouseX);
	double vY = getViewportYFromWindowY(mouseY);

	//Logger::consolePrint("mouseX: %lf, mouseY: %lf, vX: %lf, vY: %lf\n", mouseX, mouseY, vX, vY);

	return vX >= 0 && vY >= 0 && vX <= viewportWidth && vY <= viewportHeight;
}

void GLWindow::drawBorders(int lineThickness) {
	//relative coordinates normalized to 0..1, with a square aspect ratio (i.e. largest dimension goes to 1, smaller dimension is relative to larger one)
	double minX = 0, minY = 0, maxX = (double)viewportWidth / MAX(viewportWidth, viewportHeight), maxY = (double)viewportHeight / MAX(viewportWidth, viewportHeight);

	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	// Save projection matrix and sets it to a simple orthogonal projection
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(minX, maxX, minY, maxY);

	// Save model-view matrix and sets it to identity
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	// Draw borders
	glLineWidth(5);
	glColor3d(0, 0, 0);
	if (selected) glColor3d(1, .3, .3);
	glBegin(GL_LINE_LOOP);
	glVertex2d(minX, minY);
	glVertex2d(minX, maxY);
	glVertex2d(maxX, maxY);
	glVertex2d(maxX, minY);
	glEnd();

	// Draw a bit of a border as a visual indication that the window is or is not active...
	if (lineThickness > 0 && (isActive() || isSelected())) {
		glLineWidth((float)lineThickness);
		glColor3d(0, 0, 1);

		if (isSelected())
			glColor3d(1, 0, 0);

		double s = MIN(maxX, maxY) * 0.02;
		glBegin(GL_LINES);
		glVertex2d(s, 5 * s);
		glVertex2d(s, s);
		glVertex2d(s, s);
		glVertex2d(5 * s, s);

		glVertex2d(s, maxY - 5 * s);
		glVertex2d(s, maxY - s);
		glVertex2d(s, maxY - s);
		glVertex2d(5 * s, maxY - s);

		glVertex2d(maxX - s, 5 * s);
		glVertex2d(maxX - s, s);
		glVertex2d(maxX - s, s);
		glVertex2d(maxX - 5 * s, s);

		glVertex2d(maxX - s, maxY - 5 * s);
		glVertex2d(maxX - s, maxY - s);
		glVertex2d(maxX - s, maxY - s);
		glVertex2d(maxX - 5 * s, maxY - s);
		glEnd();
	}

	glLineWidth(1);

	glEnable(GL_DEPTH_TEST);

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
}

