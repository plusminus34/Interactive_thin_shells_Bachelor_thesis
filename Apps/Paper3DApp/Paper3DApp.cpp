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

Paper3DApp::Paper3DApp() {
	setWindowTitle("Test Paper3D Application...");

	delete camera;
	GLTrackingCamera *cam = new GLTrackingCamera();
	cam->ignoreRotations = false;
	camera = cam;

	bgColorR = bgColorG = bgColorB = 0.5;

	int dim_x = 6;
	int dim_y = 4;
	
	//Paper3DMesh::generateTestSystem("../data/FEM/3d/testCSTriangleSystem.tri3d", 4);
	Paper3DMesh::generateRectangleSystem("../data/FEM/3d/testCSTriangleSystem.tri3d", dim_x, dim_y, 0.1*dim_x, 0.1*dim_y);
	simMesh = new Paper3DMesh();
	simMesh->readMeshFromFile("../data/FEM/3d/testCSTriangleSystem.tri3d");

	//pin left half of top node row
	for (int i=0;i<dim_x/2;++i)
		simMesh->setPinnedNode(i*dim_y+dim_y-1, simMesh->nodes[i*dim_y+dim_y-1]->getUndeformedPosition());

	simMesh->addGravityForces(V3D(0, -9.8, 0));
	/*
	Paper2DMesh::generateSinMassSpringSystem("../data/FEM/3d/sinMassSpringSystem.ms",N);
	simMesh = new Paper2DMesh();
	simMesh->readMeshFromFile("../data/FEM/3d/sinMassSpringSystem.ms");
	simMesh->addGravityForces(V3D(0, -9.8, 0));

	simMesh->setPinnedNode(0, V3D(0.0, 0.0, 0.0));
	simMesh->setPinnedNode(N-1, simMesh->nodes[N - 1]->getUndeformedPosition());
	*/
}

Paper3DApp::~Paper3DApp(void){
}

//triggered when mouse moves
bool Paper3DApp::onMouseMoveEvent(double xPos, double yPos) {
	if (GLApplication::onMouseMoveEvent(xPos, yPos) == true) return true;
	return false;
}

//triggered when mouse buttons are pressed
bool Paper3DApp::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
	if (GLApplication::onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;
	return false;
}

//triggered when using the mouse wheel
bool Paper3DApp::onMouseWheelScrollEvent(double xOffset, double yOffset) {
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
	simMesh->checkDerivatives = true;
	simMesh->solve_statics();
	//simMesh->solve_dynamics(0.25);
}

// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
void Paper3DApp::drawScene() {
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);
	glColor3d(1, 1, 1);

	simMesh->drawSimulationMesh();
}

// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
void Paper3DApp::drawAuxiliarySceneInfo() {

}

// Restart the application.
void Paper3DApp::restart() {

}

bool Paper3DApp::processCommandLine(const std::string& cmdLine) {

	if (GLApplication::processCommandLine(cmdLine)) return true;

	return false;
}

