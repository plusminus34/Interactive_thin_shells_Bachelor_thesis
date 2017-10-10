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

