#include <GUILib/GLUtils.h>
#include "KineSimApp.h"
#include <GUILib/GLMesh.h>
#include <GUILib/GLContentManager.h>
#include <MathLib/MathLib.h>

KineSimApp::KineSimApp()
{
	setWindowTitle("Test Application for KineSim");

}

KineSimApp::~KineSimApp(void){

}

void KineSimApp::process() {
	
	double simulationTime = 0;
	double maxRunningTime = 1.0 / desiredFrameRate;


	while (simulationTime < 1.0 * maxRunningTime) {



	}

}




void KineSimApp::drawScene() {
	P3D p0(0.0, 0.0, 0.0);
	double l = 0.2;
	//double dtheta = 0.1;
	P3D px(l, 0.0, 0.0);
	P3D py(0.0, l, 0.0);
	P3D pz(0.0, 0.0, l);
	glColor3d(0.8, 0, 0);
	glBegin(GL_LINES);
	glVertex3d(p0[0], p0[1], p0[2]);
	glVertex3d(px[0], px[1], px[2]);
	glEnd();
	glColor3d(0, 0.8, 0);
	glBegin(GL_LINES);
	glVertex3d(p0[0], p0[1], p0[2]);
	glVertex3d(py[0], py[1], py[2]);
	glEnd();
	glColor3d(0, 0, 0.8);
	glBegin(GL_LINES);
	glVertex3d(p0[0], p0[1], p0[2]);
	glVertex3d(pz[0], pz[1], pz[2]);
	glEnd();

}

void KineSimApp::drawAuxiliarySceneInfo() {

}

// Restart the application.
void KineSimApp::restart() {
	
}

bool KineSimApp::onKeyEvent(int key, int action, int mods)
{
	return false;
}

bool KineSimApp::onCharacterPressedEvent(int key, int mods)
{
	return false;
}

bool KineSimApp::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos)
{
	return false;
}

bool KineSimApp::onMouseMoveEvent(double xPos, double yPos)
{
	return false;
}

bool KineSimApp::onMouseWheelScrollEvent(double xOffset, double yOffset)
{
	return false;
}

bool KineSimApp::processCommandLine(const std::string & cmdLine)
{
	return false;
}

void KineSimApp::saveFile(const char * fName)
{
}

void KineSimApp::loadFile(const char * fName)
{
}





