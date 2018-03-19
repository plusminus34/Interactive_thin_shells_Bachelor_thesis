#include <GUILib/GLUtils.h>
#include "TopOptApp.h"
#include <GUILib/GLMesh.h>
#include <GUILib/GLContentManager.h>
#include <MathLib/MathLib.h>
#include <MathLib/Plane.h>

#include <GUILib/GLUtils.h>
#include <GUILib/GLTrackingCamera.h>

//paint and unpaint boundary conditions...
//use color map as a function of von misses stress or strain...
//edit also forces
//sag-free simulations as an application of shape optimization
//adjoint method for non-linear top-opt
//have menu items for density, gravity, material params...

TopOptApp::TopOptApp() {
	setWindowTitle("Test FEM Sim Application...");

	int nRows = 20;
	int nCols = 20;
	CSTSimulationMesh2D::generateSquareTriMesh("../data/FEM/2d/triMeshTMP.tri2d", -1, 0, 0.1, 0.1, nRows, nCols);

	delete camera;
	GLTrackingCamera *cam = new GLTrackingCamera();
	cam->ignoreRotations = true;
	camera = cam;

	bgColorR = bgColorG = bgColorB = 1.0;

	femMesh = new CSTSimulationMesh2D();
	femMesh->readMeshFromFile("../data/FEM/2d/triMeshTMP.tri2d");
	//set some boundary conditions...
//	for (int i = 0; i < nCols;i++)
//		femMesh->setPinnedNode(i, femMesh->nodes[i]->getWorldPosition());
	femMesh->setPinnedNode(0, femMesh->nodes[0]->getWorldPosition());
	Globals::g = 0;

	externalLoads.resize(femMesh->nodes.size());

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
		true)->setItems({ "Linear Isotropic", "StVK", "Neo-Hookean"});

	mainMenu->addVariable("G:", Globals::g);

	mainMenu->addVariable("force scale:", forceScale);

	mainMenu->addVariable("Shear modulus", shearModulus);
	mainMenu->addVariable("Bulk modulus", bulkModulus);
	mainMenu->addButton("set params", [this]() {
		for (uint i = 0; i < femMesh->elements.size(); i++) {
			if (CSTElement2D* e = dynamic_cast<CSTElement2D*>(femMesh->elements[i])) {
				e->shearModulus = shearModulus;
				e->bulkModulus = bulkModulus;
				e->matModel = matModel;
			}
		}
	});


	menuScreen->performLayout();
}

TopOptApp::~TopOptApp(void){
}

//triggered when mouse moves
bool TopOptApp::onMouseMoveEvent(double xPos, double yPos) {
	lastClickedRay = getRayFromScreenCoords(xPos, yPos);

	if (GlobalMouseState::dragging && GlobalMouseState::lButtonPressed && selectedNodeID < 0){
		lastClickedRay.getDistanceToPlane(Plane(P3D(), V3D(0, 0, -1)), &endDragPoint);

		startDragPoint.z() = 1.0; 
		endDragPoint.z() = -1.0;

		AxisAlignedBoundingBox bb(startDragPoint, endDragPoint);

		if (glfwGetKey(glfwWindow, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS){
			for (uint i = 0; i < femMesh->nodes.size(); i++)
				if (bb.isInside(femMesh->nodes[i]->getWorldPosition())){
					femMesh->setPinnedNode(i, femMesh->nodes[i]->getWorldPosition());
				}
		}
		else if (glfwGetKey(glfwWindow, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) {
			for (uint i = 0; i < femMesh->nodes.size(); i++)
				if (bb.isInside(femMesh->nodes[i]->getWorldPosition()))
					femMesh->unpinNode(i);
		}
	}
	else{
		if (GlobalMouseState::dragging) {
			if (selectedNodeID >= 0 && getGLAppInstance()->appIsRunning == false) {
				P3D p;
				lastClickedRay.getDistanceToPlane(Plane(P3D(), V3D(0, 0, -1)), &p);
				externalLoads[selectedNodeID] = V3D(femMesh->nodes[selectedNodeID]->getWorldPosition(), p);
			}
		}else
			selectedNodeID = femMesh->getSelectedNodeID(lastClickedRay);
	}
//	if (selectedNodeID != -1){
//		Plane plane(camera->getCameraTarget(), V3D(camera->getCameraPosition(), camera->getCameraTarget()).unit());
//		P3D targetPinPos; getRayFromScreenCoords(xPos, yPos).getDistanceToPlane(plane, &targetPinPos);
//		femMesh->setPinnedNode(selectedNodeID, targetPinPos);
//		return true;
//	}
	
	if (GLApplication::onMouseMoveEvent(xPos, yPos) == true) return true;
	return false;
}

//triggered when mouse buttons are pressed
bool TopOptApp::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
	lastClickedRay = getRayFromScreenCoords(xPos, yPos);

	if (button == GLFW_MOUSE_BUTTON_LEFT){
		if (action == GLFW_PRESS) {
			if (selectedNodeID < 0) {
				lastClickedRay.getDistanceToPlane(Plane(P3D(), V3D(0, 0, -1)), &startDragPoint);
				endDragPoint = startDragPoint;
			}
		}
	}

	if (GLApplication::onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;

//	Logger::consolePrint("--> %d\n", selectedNodeID);
//	Logger::consolePrint("--> %lf %lf %lf\n", lastClickedRay.origin[0], lastClickedRay.origin[1], lastClickedRay.origin[2]);
//	Logger::consolePrint("--> %lf %lf %lf\n", lastClickedRay.direction[0], lastClickedRay.direction[1], lastClickedRay.direction[2]);
	return false;
}

//triggered when using the mouse wheel
bool TopOptApp::onMouseWheelScrollEvent(double xOffset, double yOffset) {
	if (GLApplication::onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	return false;
}

bool TopOptApp::onKeyEvent(int key, int action, int mods) {
	if (GLApplication::onKeyEvent(key, action, mods)) return true;

	return false;
}

bool TopOptApp::onCharacterPressedEvent(int key, int mods) {
	if (GLApplication::onCharacterPressedEvent(key, mods)) return true;

	return false;
}


void TopOptApp::loadFile(const char* fName) {
	Logger::consolePrint("Loading file \'%s\'...\n", fName);
	std::string fileName;
	fileName.assign(fName);

	std::string fNameExt = fileName.substr(fileName.find_last_of('.') + 1);
	if (fNameExt == "tri2d") {
		delete femMesh;
		femMesh = new CSTSimulationMesh2D;
		femMesh->readMeshFromFile(fName);
		Logger::consolePrint("...Done!");
	} else {
		Logger::consolePrint("...but how to do with that?");
	}

}

void TopOptApp::saveFile(const char* fName) {
	Logger::consolePrint("SAVE FILE: Do not know what to do with file \'%s\'\n", fName);
}


// Run the App tasks
void TopOptApp::process() {
	//do the work here...

	//add mouse drag tempEEUnits
	//femMesh->tempEEUnits.clear();
	//femMesh->tempEEUnits.push_back(new CSTDrag2D(this, nodes[9], P3D(-5, 20)));
	//
	double simulationTime = 0;
	double maxRunningTime = 1.0 / desiredFrameRate;
	femMesh->checkDerivatives = checkDerivatives != 0;

	femMesh->addGravityForces(V3D(0, Globals::g, 0));

	for (int i = 0; i < femMesh->nodes.size(); i++) {
		femMesh->f_ext[2 * i + 0] = externalLoads[i].x() * forceScale;
		femMesh->f_ext[2 * i + 1] = externalLoads[i].y() * forceScale;
	}

	femMesh->solve_statics();
}

// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
void TopOptApp::drawScene() {
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);
	glColor3d(1,1,1);
	femMesh->drawSimulationMesh();

	if (selectedNodeID >= 0) {
		P3D pos = femMesh->nodes[selectedNodeID]->getWorldPosition();
		drawSphere(pos, 0.02);
	}

	glColor3d(0,1,0);
	for (int i = 0; i < externalLoads.size(); i++) {
		P3D pos = femMesh->nodes[i]->getWorldPosition();
		V3D force = externalLoads[i];
		if (force.length() > 0.01)
			drawArrow(pos, pos + force, 0.005);
	}

	if (GlobalMouseState::dragging && GlobalMouseState::lButtonPressed && selectedNodeID < 0) {
		glColor3d(0.5, 0.5, 1.0);
		glBegin(GL_LINE_LOOP);
		glVertex3d(startDragPoint.x(), startDragPoint.y(), 0);
		glVertex3d(startDragPoint.x(), endDragPoint.y(), 0);
		glVertex3d(endDragPoint.x(), endDragPoint.y(), 0);
		glVertex3d(endDragPoint.x(), startDragPoint.y(), 0);
		glEnd();
	}

#ifdef CONSTRAINED_DYNAMICS_DEMO
	glColor3d(1, 1, 1);
	drawCircle(0, 0, 1, 100);
#endif
}

// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
void TopOptApp::drawAuxiliarySceneInfo() {

}

// Restart the application.
void TopOptApp::restart() {

}

bool TopOptApp::processCommandLine(const std::string& cmdLine) {

	if (GLApplication::processCommandLine(cmdLine)) return true;

	return false;
}

