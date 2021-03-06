#include <GUILib/GLUtils.h>
#include "FEMSimApp.h"
#include <GUILib/GLMesh.h>
#include <GUILib/GLContentManager.h>
#include <MathLib/MathLib.h>
#include <MathLib/Plane.h>
#include <FEMSimLib/CSTSimulationMesh2D.h>
#include <FEMSimLib/CSTSimulationMesh3D.h>
#include <FEMSimLib/MassSpringSimulationMesh3D.h>
#include <GUILib/GLUtils.h>
#include <GUILib/GLTrackingCamera.h>


#define EDIT_BOUNDARY_CONDITIONS

//#define CONSTRAINED_DYNAMICS_DEMO

FEMSimApp::FEMSimApp() {
	setWindowTitle("Test FEM Sim Application...");

#ifdef CONSTRAINED_DYNAMICS_DEMO
	CSTSimulationMesh2D::generateSquareTriMesh("../data/FEM/2d/triMeshTMP.tri2d", -1, 0, 0.1, 0.1, 1, 1);

	femMesh = new CSTSimulationMesh2D();
	femMesh->readMeshFromFile("../data/FEM/2d/triMeshTMP.tri2d");
	femMesh->nodes[0]->addMassContribution(1.0);
	femMesh->addGravityForces(V3D(0, -9.8, 0));
#else

	delete camera;
	GLTrackingCamera *cam = new GLTrackingCamera();
	cam->ignoreRotations = true;
	camera = cam;

	int nRows = 10;
	int nCols = 10;
	CSTSimulationMesh2D::generateSquareTriMesh("../data/FEM/2d/triMeshTMP.tri2d", -1, 0, 0.1, 0.1, nRows, nCols);

	femMesh = new CSTSimulationMesh2D();
	femMesh->readMeshFromFile("../data/FEM/2d/triMeshTMP.tri2d");
	femMesh->addGravityForces(V3D(0, -9.8, 0));
	//set some boundary conditions...
	for (int i = 0; i < nCols;i++)
		femMesh->setPinnedNode(i, femMesh->nodes[i]->getWorldPosition());
#endif

/*
	MassSpringSimulationMesh3D::generateTestMassSpringSystem("../data/FEM/3d/massSpringSystem.ms");
	femMesh = new MassSpringSimulationMesh3D();
	femMesh->readMeshFromFile("../data/FEM/3d/massSpringSystem.ms");
	femMesh->addGravityForces(V3D(0, -9.8, 0));
*/

	showGroundPlane = false;

	mainMenu->addGroup("FEM Sim options");

	mainMenu->addVariable<MaterialModel2D>("MaterialModel",
		[this](const MaterialModel2D &val) {
		matModel = val;
		for (uint i = 0; i < femMesh->elements.size(); i++) {
			if (CSTElement2D* e = dynamic_cast<CSTElement2D*>(femMesh->elements[i]))
				e->matModel = matModel;
		}

	},
		[this]() {return matModel; },
		true)->setItems({ "Linear Isotropic", "StVK", "Neo-Hookean" });

	mainMenu->addVariable("Static solve", computeStaticSolution);
	mainMenu->addVariable("Check derivatives", checkDerivatives);
	menuScreen->performLayout();

}

FEMSimApp::~FEMSimApp(void){
}

//triggered when mouse moves
bool FEMSimApp::onMouseMoveEvent(double xPos, double yPos) {
	lastClickedRay = getRayFromScreenCoords(xPos, yPos);

	currentRay = getRayFromScreenCoords(xPos, yPos);
	if (selectedNodeID != -1){
		Plane plane(camera->getCameraTarget(), V3D(camera->getCameraPosition(), camera->getCameraTarget()).unit());
		P3D targetPinPos; getRayFromScreenCoords(xPos, yPos).getDistanceToPlane(plane, &targetPinPos);
		femMesh->setPinnedNode(selectedNodeID, targetPinPos);
		return true;
	}
	else if (GLApplication::onMouseMoveEvent(xPos, yPos) == true) return true;
	return false;
}

//triggered when mouse buttons are pressed
bool FEMSimApp::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
#ifdef EDIT_BOUNDARY_CONDITIONS

	if (button == GLFW_MOUSE_BUTTON_RIGHT) {
		femMesh->removePinnedNodeConstraints();
	}

	if (button == GLFW_MOUSE_BUTTON_LEFT){
		if (action == GLFW_PRESS && mods & GLFW_MOD_SHIFT) {
			selectedNodeID = femMesh->getSelectedNodeID(lastClickedRay);
			if (selectedNodeID >= 0)
				return true;
		}
		else
			selectedNodeID = -1;
	}
#endif

	if (GLApplication::onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;

//	Logger::consolePrint("--> %d\n", selectedNodeID);
//	Logger::consolePrint("--> %lf %lf %lf\n", lastClickedRay.origin[0], lastClickedRay.origin[1], lastClickedRay.origin[2]);
//	Logger::consolePrint("--> %lf %lf %lf\n", lastClickedRay.direction[0], lastClickedRay.direction[1], lastClickedRay.direction[2]);
	return false;
}

//triggered when using the mouse wheel
bool FEMSimApp::onMouseWheelScrollEvent(double xOffset, double yOffset) {
	if (GLApplication::onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	return false;
}

bool FEMSimApp::onKeyEvent(int key, int action, int mods) {
	if (GLApplication::onKeyEvent(key, action, mods)) return true;

	return false;
}

bool FEMSimApp::onCharacterPressedEvent(int key, int mods) {
	if (GLApplication::onCharacterPressedEvent(key, mods)) return true;

	return false;
}


void FEMSimApp::loadFile(const char* fName) {
	Logger::consolePrint("Loading file \'%s\'...\n", fName);
	std::string fileName;
	fileName.assign(fName);

	std::string fNameExt = fileName.substr(fileName.find_last_of('.') + 1);
	if (fNameExt == "tri2d") {
		delete femMesh;
		femMesh = new CSTSimulationMesh2D;
		femMesh->readMeshFromFile(fName);
		Logger::consolePrint("...Done!");
	} else if (fNameExt == "obj") {
		delete femMesh;
		femMesh = new CSTSimulationMesh3D;
		femMesh->readMeshFromFile(fName);
		Logger::consolePrint("...Done!");
	} else {
		Logger::consolePrint("...but how to do with that?");
	}

}

void FEMSimApp::saveFile(const char* fName) {
	Logger::consolePrint("SAVE FILE: Do not know what to do with file \'%s\'\n", fName);
}


// Run the App tasks
void FEMSimApp::process() {
	//do the work here...

	//add mouse drag tempEEUnits
	//femMesh->tempEEUnits.clear();
	//femMesh->tempEEUnits.push_back(new CSTDrag2D(this, nodes[9], P3D(-5, 20)));
	//
	double simulationTime = 0;
	double maxRunningTime = 1.0 / desiredFrameRate;
	femMesh->checkDerivatives = checkDerivatives != 0;

	//if we still have time during this frame, or if we need to finish the physics step, do this until the simulation time reaches the desired value
	while (simulationTime < 1.0 * maxRunningTime) {
        //femMesh->checkDerivatives = true;
//		shearModulus = MAX(1, shearModulus);
//		bulkModulus = MAX(1, bulkModulus);
//		if (autoUpdateShearModulusAndBulkModulus)
//			femMesh->setShearModulusAndBulkModulus(shearModulus, bulkModulus);
		simulationTime += simTimeStep;
		if (computeStaticSolution)
			femMesh->solve_statics();
		else {

#ifdef CONSTRAINED_DYNAMICS_DEMO
//figure out a constraint force here...
			V3D f_orig = femMesh->nodes[0]->getExternalForce();
			V3D x = femMesh->nodes[0]->getWorldPosition();
			V3D xDot = femMesh->nodes[0]->getVelocity();
			double m = 1.0;

			double C = 0.5 * (x.dot(x) - 1);
			double CDot = x.dot(xDot);

			double lambda = -f_orig.dot(x) - xDot.dot(xDot) - 50 * C - 5 * CDot;
			lambda /= x.dot(x);
			V3D fConstraint = x * lambda;


			femMesh->nodes[0]->setCoordinates(P3D() + fConstraint + f_orig, femMesh->f_ext);
			femMesh->solve_dynamics(simTimeStep);
			femMesh->nodes[0]->setCoordinates(P3D() + f_orig, femMesh->f_ext);
#else
			femMesh->solve_dynamics(simTimeStep);
#endif
//			femMesh->fakeContactWithPlane(Plane(P3D(0, 0, 0), V3D(0, 1, 0)));
		}

//		break;
	}
}

// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
void FEMSimApp::drawScene() {
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);
	glColor3d(1,1,1);
	femMesh->drawSimulationMesh();

#ifdef CONSTRAINED_DYNAMICS_DEMO
	glColor3d(1, 1, 1);
	drawCircle(0, 0, 1, 100);
#endif
}

// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
void FEMSimApp::drawAuxiliarySceneInfo() {

}

// Restart the application.
void FEMSimApp::restart() {

}

bool FEMSimApp::processCommandLine(const std::string& cmdLine) {

	if (GLApplication::processCommandLine(cmdLine)) return true;

	return false;
}

