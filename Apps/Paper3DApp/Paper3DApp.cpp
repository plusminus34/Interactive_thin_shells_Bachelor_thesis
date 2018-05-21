#include <GUILib/GLUtils.h>
#include "Paper3DApp.h"
#include <GUILib/GLMesh.h>
#include <GUILib/GLContentManager.h>
#include <MathLib/MathLib.h>
#include <MathLib/Plane.h>

#include <GUILib/GLUtils.h>
#include <GUILib/GLTrackingCamera.h>
//#include <OptimizationLib\SQPFunctionMinimizer.h>
#include <vector>
#include "Paper3DMesh.h"
#include <FEMSimLib/CSTriangle3D.h>
#include "BendingEdge.h"
#include "ZeroLengthSpring3D.h"
#include "BarycentricZeroLengthSpring.h"

Paper3DApp::Paper3DApp() {
	setWindowTitle("Test Paper3D Application...");
	/*
	delete camera;
	GLTrackingCamera *cam = new GLTrackingCamera();
	cam->ignoreRotations = false;
	camera = cam;
	*/
	showGroundPlane = false;

	simWindow = new SimulationWindow(0, 0, 100, 100, this);
	shapeWindow = new ShapeWindow(0, 0, 100, 100, this);

	bgColorR = bgColorG = bgColorB = 0.5;

	shearModulus = 750;
	bulkModulus = 0.5;
	bend_k = 0.0008;
	pin_k = 0;//50 is a reasonable value, ... at least for rectangle with dimensions 11 x 7
	mouse_mode = mouse_drag;

	int dim_x = 11;
	int dim_y = 7;
	Paper3DMesh::generateRectangleSystem("../data/FEM/3d/testCSTriangleSystem.tri3d", dim_x, dim_y, 0.1*dim_x, 0.1*dim_y);

	simMesh = new Paper3DMesh();
	simMesh->readMeshFromFile("../data/FEM/3d/testCSTriangleSystem.tri3d");

	//Fix left edge of the rectangle
	for (int i=0;i<2*dim_y;++i)
		simMesh->setPinnedNode(i, simMesh->nodes[i]->getUndeformedPosition());
	//Pin the top right corner to a spot on the bottom edge
	//simMesh->elements.push_back(new ZeroLengthSpring3D(simMesh, simMesh->nodes[5*dim_y], simMesh->nodes[dim_y * dim_x-1]));

	//simMesh->addGravityForces(V3D(0.0, 0.0, 0.0));

	mainMenu->addVariable("Check derivatives", checkDerivatives);

	mainMenu->addGroup("FEM Sim options");
	mainMenu->addVariable("Shear modulus", shearModulus);
	mainMenu->addVariable("Bulk modulus", bulkModulus);
	mainMenu->addVariable("Bending stiffness", bend_k);
	mainMenu->addVariable("Pin stiffness", pin_k);
	//mainMenu->addVariable("Mouse Mode", mouse_mode, true)->setItems({ mouse_drag, mouse_none, mouse_pin });//not yet working

	menuScreen->performLayout();
	setupWindows();
	followCameraTarget = false;
}

Paper3DApp::~Paper3DApp(void){
}

void Paper3DApp::setupWindows() {
	int w, h;

	//int offset = (int)(mainMenu->window()->width() * menuScreen->pixelRatio());

	//w = (GLApplication::getMainWindowWidth()) - offset;
	w = GLApplication::getMainWindowWidth();
	h = GLApplication::getMainWindowHeight();

	shapeWindow->setViewportParameters(0, 0, w / 2, h);
	simWindow->setViewportParameters(w / 2, 0, w / 2, h);
	consoleWindow->setViewportParameters(0, 0, w, 50);
}

P3D Paper3DApp::getNodePos(int i) {
	return simMesh->nodes[i]->getWorldPosition();
}

//triggered when mouse moves
bool Paper3DApp::onMouseMoveEvent(double xPos, double yPos) {
	if (simWindow->mouseIsWithinWindow(xPos, yPos)) {
		simWindow->onMouseMoveEvent(xPos, yPos);
	}
	else if (shapeWindow->mouseIsWithinWindow(xPos, yPos)) {
		shapeWindow->onMouseMoveEvent(xPos, yPos);
	}
	return true;
	//if (GLApplication::onMouseMoveEvent(xPos, yPos) == true) return true;
	//return false;
}

//triggered when mouse buttons are pressed
bool Paper3DApp::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
	if (GLApplication::onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;
	return false;
}

//triggered when using the mouse wheel
bool Paper3DApp::onMouseWheelScrollEvent(double xOffset, double yOffset) {
	//if (simWindow->mouseIsWithinWindow(xPos, yPos)) {
		simWindow->onMouseWheelScrollEvent(xOffset, yOffset);
	//}
	//else if (shapeWindow->mouseIsWithinWindow(xPos, yPos)) {
		shapeWindow->onMouseWheelScrollEvent(xOffset, yOffset);
	//}
	if (GLApplication::onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	return false;
}

bool Paper3DApp::onKeyEvent(int key, int action, int mods) {
	if (GLApplication::onKeyEvent(key, action, mods)) return true;

	return false;
}

bool Paper3DApp::onCharacterPressedEvent(int key, int mods) {
	if (GLApplication::onCharacterPressedEvent(key, mods)) return true;

	return false;
}


void Paper3DApp::loadFile(const char* fName) {
	Logger::consolePrint("Loading file \'%s\'...\n", fName);
	Logger::consolePrint("...or not.\n");
}

void Paper3DApp::saveFile(const char* fName) {
	Logger::consolePrint("SAVE FILE: Do not know what to do with file \'%s\'\n", fName);
}


// Run the App tasks
void Paper3DApp::process() {
	//do the work here...
	updateParams();
	simMesh->solve_statics();
	//simMesh->solve_dynamics(0.25);
}

// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
void Paper3DApp::drawScene() {
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);
	glColor3d(1, 1, 1);

	//simMesh->drawRestConfiguration();
	//simMesh->drawSimulationMesh();

	if (GlobalMouseState::dragging && GlobalMouseState::lButtonPressed && selectedNodeID < 0) {
		glColor3d(0.5, 0.5, 1.0);
		glBegin(GL_LINE_LOOP);
		glVertex3d(startDragPoint.x(), startDragPoint.y(), 0);
		glVertex3d(startDragPoint.x(), endDragPoint.y(), 0);
		glVertex3d(endDragPoint.x(), endDragPoint.y(), 0);
		glVertex3d(endDragPoint.x(), startDragPoint.y(), 0);
		glEnd();
	}
}

// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
void Paper3DApp::drawAuxiliarySceneInfo() {

	shapeWindow->drawScene();
	shapeWindow->drawAuxiliarySceneInfo();
	simWindow->drawScene();
	simWindow->drawAuxiliarySceneInfo();
}

// Restart the application.
void Paper3DApp::restart() {

}

bool Paper3DApp::processCommandLine(const std::string& cmdLine) {

	if (GLApplication::processCommandLine(cmdLine)) return true;

	return false;
}

void Paper3DApp::updateParams() {
	simMesh->checkDerivatives = checkDerivatives;
	for (uint i = 0; i < simMesh->elements.size(); i++) {
		if (BendingEdge* e = dynamic_cast<BendingEdge*>(simMesh->elements[i])) {
			e->k = bend_k;
		}
		else if (CSTriangle3D* e = dynamic_cast<CSTriangle3D*>(simMesh->elements[i])) {
			e->shearModulus = shearModulus;
			e->bulkModulus = bulkModulus;
		}
		else if (ZeroLengthSpring3D* e = dynamic_cast<ZeroLengthSpring3D*>(simMesh->elements[i])) {
			e->k = pin_k;
		}
		else if (BarycentricZeroLengthSpring* e = dynamic_cast<BarycentricZeroLengthSpring*>(simMesh->elements[i])) {
			e->k = pin_k;
		}
	}
}

