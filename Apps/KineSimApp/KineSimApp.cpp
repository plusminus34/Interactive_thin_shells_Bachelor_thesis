#include <GUILib/GLUtils.h>
#include "KineSimApp.h"
#include <GUILib/GLMesh.h>
#include <GUILib/GLContentManager.h>
#include <MathLib/MathLib.h>
#include <KineSimLib/KS_BindComponentsConnection.h>
#include <KineSimLib/KS_rMotorConnection.h>
#include <KineSimLib/KS_UIMechanismController.h>


KineSimApp::KineSimApp()
{   
	setWindowTitle("Test Application for KineSim");
	delete mech1; delete uiController;
	mech1 = new KS_MechanicalAssembly();
	mech1->readFromFile("../data/KineSimApp/threeLink3d.mech");
	mech1->readMechStateFromFile("../data/KineSimApp/s_threeLink3d.txt");
	mech1->setAssemblyState(mech1->s);

	uiController = new KS_UIMechanismController(mech1, this);

	mainMenu->addGroup("sim parameters");
	mainMenu->addVariable("logState", logState);
	mainMenu->addVariable("newtonSolver", mech1->newtonSolver);
	mainMenu->addVariable("bfgsSolver", mech1->bfgsSolver);

	uiController->boxes = false;	uiController->sliders = true;
	uiController->setMotorAngleValues(uiController->motorAngleValues);
	menuScreen->performLayout();
}
	
	

KineSimApp::~KineSimApp(void){
	delete mech1;
	delete uiController;
}

void KineSimApp::process() {
	
	double simulationTime = 0;
	double maxRunningTime = 1.0 / desiredFrameRate;


	while (simulationTime < 1.0 * maxRunningTime) {

		simulationTime += simTimeStep;
		uiController->activateMechanismController();
		mech1->solveAssembly();
		if (logState)
			mech1->logMechS("../data/KineSimApp/mechState.txt");

	}


}


void KineSimApp::drawScene() {

	glEnable(GL_LIGHTING);

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
	drawSphere(p0, l/16);
	mech1->draw();

}

void KineSimApp::drawAuxiliarySceneInfo() {

}

// Restart the application.
void KineSimApp::restart() {
	mech1->setAssemblyState(uiController->startingMechState);
	mech1->s = uiController->startingMechState;
	uiController->motorAngleValues.setZero();
}

bool KineSimApp::onKeyEvent(int key, int action, int mods)
{
	if (GLApplication::onKeyEvent(key, action, mods) == true) return true;

	return false;
}

bool KineSimApp::onCharacterPressedEvent(int key, int mods)
{
	if (GLApplication::onCharacterPressedEvent(key, mods) == true) return true;

	return false;
}

bool KineSimApp::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos)
{
	if (GLApplication::onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;

	return false;
}

bool KineSimApp::onMouseMoveEvent(double xPos, double yPos)
{
	if (GLApplication::onMouseMoveEvent(xPos, yPos) == true) return true;

	return false;
}

bool KineSimApp::onMouseWheelScrollEvent(double xOffset, double yOffset)
{
	if (GLApplication::onMouseWheelScrollEvent(xOffset, yOffset) == true) return true;
	return false;
}

bool KineSimApp::processCommandLine(const std::string & cmdLine)
{
	if (GLApplication::processCommandLine(cmdLine) == true) return true;
	return false;
}

void KineSimApp::saveFile(const char * fName)
{
}

void KineSimApp::loadFile(const char * fName)
{
}


