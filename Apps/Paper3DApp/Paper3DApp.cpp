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
#include "BendingEdge.h"
#include "ZeroLengthSpring3D.h"

Paper3DApp::Paper3DApp() {
	setWindowTitle("Test Paper3D Application...");

	delete camera;
	GLTrackingCamera *cam = new GLTrackingCamera();
	cam->ignoreRotations = false;
	camera = cam;

	bgColorR = bgColorG = bgColorB = 0.5;

	shearModulus = 80;
	bulkModulus = 0.5;
	bend_k = 1;

#define NRECT
#ifdef RECT
	int dim_x = 9;
	int dim_y = 6;
	Paper3DMesh::generateRectangleSystem("../data/FEM/3d/testCSTriangleSystem.tri3d", dim_x, dim_y, 0.1*dim_x, 0.1*dim_y);
#else
	int N = 13;
	Paper3DMesh::generateTestSystem("../data/FEM/3d/testCSTriangleSystem.tri3d", N);
#endif

	simMesh = new Paper3DMesh();
	simMesh->readMeshFromFile("../data/FEM/3d/testCSTriangleSystem.tri3d");

#ifdef RECT
	//pin left half of top node row
	for (int i=0;i<dim_y;++i)
		simMesh->setPinnedNode(i, simMesh->nodes[i]->getUndeformedPosition());
	//simMesh->elements.push_back(new ZeroLengthSpring3D(simMesh, simMesh->nodes[0], simMesh->nodes[dim_y * (dim_x-1)]));
#else
	//Pin the two ends
	simMesh->setPinnedNode(0, simMesh->nodes[0]->getUndeformedPosition());
	simMesh->setPinnedNode(N- 1, simMesh->nodes[N - 1]->getUndeformedPosition());
	//simMesh->elements.push_back(new ZeroLengthSpring3D(simMesh, simMesh->nodes[3], simMesh->nodes[5]));
#endif

	mainMenu->addGroup("FEM Sim options");
	mainMenu->addVariable("Shear modulus", shearModulus);
	mainMenu->addVariable("Bulk modulus", bulkModulus);
	mainMenu->addVariable("Bending stiffness", bend_k);

	menuScreen->performLayout();

}

Paper3DApp::~Paper3DApp(void){
}

//triggered when mouse moves
bool Paper3DApp::onMouseMoveEvent(double xPos, double yPos) {
	lastClickedRay = getRayFromScreenCoords(xPos, yPos);
	if (GlobalMouseState::dragging) {
		if (selectedNodeID >= 0) {
			P3D p;
			lastClickedRay.getDistanceToPlane(Plane(P3D(), V3D(0, 0, -1)), &p);
			simMesh->setPinnedNode(selectedNodeID, p);
		}
	}
	else {
		selectedNodeID = simMesh->getSelectedNodeID(lastClickedRay);
	}
	if (GLApplication::onMouseMoveEvent(xPos, yPos) == true) return true;
	return false;
}

//triggered when mouse buttons are pressed
bool Paper3DApp::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
	lastClickedRay = getRayFromScreenCoords(xPos, yPos);

	if (button == GLFW_MOUSE_BUTTON_LEFT) {
		if (action == GLFW_PRESS) {
			if (selectedNodeID < 0) {
				lastClickedRay.getDistanceToPlane(Plane(P3D(), V3D(0, 0, -1)), &startDragPoint);
				endDragPoint = startDragPoint;
			}
		}
	}
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
	updateParams();
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

}

// Restart the application.
void Paper3DApp::restart() {

}

bool Paper3DApp::processCommandLine(const std::string& cmdLine) {

	if (GLApplication::processCommandLine(cmdLine)) return true;

	return false;
}

void Paper3DApp::updateParams() {
	for (uint i = 0; i < simMesh->elements.size(); i++) {
		simMesh->elements[i]->getMass();
		if (BendingEdge* e = dynamic_cast<BendingEdge*>(simMesh->elements[i])) {
			e->k = bend_k;
		}
		else if (CSTriangle3D* e = dynamic_cast<CSTriangle3D*>(simMesh->elements[i])) {
			e->shearModulus = shearModulus;
			e->bulkModulus = bulkModulus;
		}
	}
}

