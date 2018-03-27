#include <GUILib/GLUtils.h>
#include "TopOptApp.h"
#include <GUILib/GLMesh.h>
#include <GUILib/GLContentManager.h>
#include <MathLib/MathLib.h>
#include <MathLib/Plane.h>

#include <GUILib/GLUtils.h>
#include <GUILib/GLTrackingCamera.h>
#include <OptimizationLib\SQPFunctionMinimizer.h>

//sag-free simulations as an application of shape optimization
//adjoint method for non-linear top-opt

//Objectives:
//- compliance/stored energy
//- amount of material used
//- smoothness of solution (in case dithering artifacts come up)
//- L0 regularizer to force solution to choose a side...

//Constraints:
//- upper bound on total mass based on desired value
//- per element density limits

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

	simMesh = new CSTSimulationMesh2D();
	simMesh->readMeshFromFile("../data/FEM/2d/triMeshTMP.tri2d");

	//set some boundary conditions...
	for (int i = 0; i < nCols;i++)
		simMesh->setPinnedNode(i, simMesh->nodes[i]->getWorldPosition());

	Globals::g = 0;

	externalLoads.resize(simMesh->nodes.size());
	resize(densityParams, simMesh->elements.size()); densityParams.setOnes();

	showGroundPlane = false;

	mainMenu->addGroup("FEM Sim options");
	mainMenu->addVariable<MaterialModel2D>("MaterialModel",
		[this](const MaterialModel2D &val) {
			matModel = val;  
			for (uint i = 0; i < simMesh->elements.size(); i++) {
				if (CSTElement2D* e = dynamic_cast<CSTElement2D*>(simMesh->elements[i])) 
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
		applyDensityParametersToSimMesh();
	});

	initialMass = 0;
	for (uint i = 0; i < simMesh->elements.size(); i++) {
		if (CSTElement2D* e = dynamic_cast<CSTElement2D*>(simMesh->elements[i]))
			initialMass += e->getMass();
	}


	mainMenu->addVariable("Optimize Topology", optimizeTopology);

	nanogui::Widget *panel = new nanogui::Widget(mainMenu->window());
	mainMenu->addWidget("", panel);
	panel->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
		nanogui::Alignment::Middle, 0, 4));

	auto label = new nanogui::Label(panel, "Target mass ratio (%)", "sans-bold");
	auto slider = new nanogui::Slider(panel);
	slider->setValue(0.5f);
	slider->setFixedWidth(80);
	std::pair<float, float> range; range.first = 10; range.second = 100;
	slider->setRange(range);
	slider->setValue((float)targetMassRatio);
	slider->setCallback([this](float val) { targetMassRatio = val; });

	menuScreen->performLayout();

	energyFunction = new TopOptEnergyFunction(simMesh);
	constraints = new TopOptConstraints(simMesh);
	constrainedObjectiveFunction = new ConstrainedObjectiveFunction(energyFunction, constraints);
}

void TopOptApp::applyDensityParametersToSimMesh() {
	constraints->totalMassUpperBound = initialMass * targetMassRatio / 100.0;

	//make sure we're on the constraint manifold
	double currentMass = 0;
	for (uint i = 0; i < simMesh->elements.size(); i++) {
		if (CSTElement2D* e = dynamic_cast<CSTElement2D*>(simMesh->elements[i]))
			currentMass += e->getMass() * densityParams[i];
	}

	Logger::consolePrint("total mass: %lf (upper bound: %lf)\n", currentMass, initialMass * targetMassRatio / 100.0);

	if (currentMass > initialMass * targetMassRatio / 100.0) {
		double ratio = initialMass * targetMassRatio / 100.0 / currentMass;
		for (uint i = 0; i < simMesh->elements.size(); i++) {
			if (CSTElement2D* e = dynamic_cast<CSTElement2D*>(simMesh->elements[i]))
				densityParams[i] *= ratio;
		}
	}

	for (uint i = 0; i < simMesh->elements.size(); i++) {
		if (CSTElement2D* e = dynamic_cast<CSTElement2D*>(simMesh->elements[i])) {
			e->shearModulus = pow(densityParams[i], 1) * (shearModulus);
			e->bulkModulus = pow(densityParams[i], 1) * (bulkModulus);
			e->densityForDrawing = densityParams[i];
		}
	}
}


TopOptApp::~TopOptApp(void){
}

//triggered when mouse moves
bool TopOptApp::onMouseMoveEvent(double xPos, double yPos) {
	lastClickedRay = getRayFromScreenCoords(xPos, yPos);

	if (GlobalMouseState::dragging == false && glfwGetKey(glfwWindow, GLFW_KEY_LEFT_ALT) == GLFW_PRESS) {
		int elementID = simMesh->getSelectedElementID(lastClickedRay);
		if (elementID >= 0) 
			densityParams[elementID] = 0.001;
	}

	if (GlobalMouseState::dragging == false && glfwGetKey(glfwWindow, GLFW_KEY_RIGHT_ALT) == GLFW_PRESS) {
		int elementID = simMesh->getSelectedElementID(lastClickedRay);
		if (elementID >= 0)
			densityParams[elementID] = 1;
	}

	if (GlobalMouseState::dragging && GlobalMouseState::lButtonPressed && selectedNodeID < 0){
		lastClickedRay.getDistanceToPlane(Plane(P3D(), V3D(0, 0, -1)), &endDragPoint);

		startDragPoint.z() = 1.0; 
		endDragPoint.z() = -1.0;

		AxisAlignedBoundingBox bb(startDragPoint, endDragPoint);

		if (glfwGetKey(glfwWindow, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS){
			for (uint i = 0; i < simMesh->nodes.size(); i++)
				if (bb.isInside(simMesh->nodes[i]->getWorldPosition())){
					simMesh->setPinnedNode(i, simMesh->nodes[i]->getWorldPosition());
				}
		}
		else if (glfwGetKey(glfwWindow, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) {
			for (uint i = 0; i < simMesh->nodes.size(); i++)
				if (bb.isInside(simMesh->nodes[i]->getWorldPosition()))
					simMesh->unpinNode(i);
		}
	}
	else{
		if (GlobalMouseState::dragging) {
			if (selectedNodeID >= 0) {
				P3D p;
				lastClickedRay.getDistanceToPlane(Plane(P3D(), V3D(0, 0, -1)), &p);
				externalLoads[selectedNodeID] = V3D(simMesh->nodes[selectedNodeID]->getWorldPosition(), p);
			}
		}else
			selectedNodeID = simMesh->getSelectedNodeID(lastClickedRay);
	}
//	if (selectedNodeID != -1){
//		Plane plane(camera->getCameraTarget(), V3D(camera->getCameraPosition(), camera->getCameraTarget()).unit());
//		P3D targetPinPos; getRayFromScreenCoords(xPos, yPos).getDistanceToPlane(plane, &targetPinPos);
//		simMesh->setPinnedNode(selectedNodeID, targetPinPos);
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
		delete simMesh;
		simMesh = new CSTSimulationMesh2D;
		simMesh->readMeshFromFile(fName);
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
	if (GlobalMouseState::dragging)
		return;

	//do the work here...
	double simulationTime = 0;
	double maxRunningTime = 1.0 / desiredFrameRate;
	simMesh->checkDerivatives = checkDerivatives != 0;

	simMesh->addGravityForces(V3D(0, Globals::g, 0));

	for (uint i = 0; i < simMesh->nodes.size(); i++) {
		simMesh->f_ext[2 * i + 0] = externalLoads[i].x() * forceScale;
		simMesh->f_ext[2 * i + 1] = externalLoads[i].y() * forceScale;
	}

	applyDensityParametersToSimMesh();

	if (optimizeTopology) {
		double val;
		SQPFunctionMinimizer minimizer(1);
		minimizer.maxLineSearchIterations_ = 12;
		minimizer.printOutput_ = energyFunction->printDebugInfo;
		minimizer.minimize(constrainedObjectiveFunction, densityParams, val);
	}

	simMesh->solve_statics();
	Logger::consolePrint("overall deformation energy: %lf", simMesh->getCurrentDeformationEnergy());
}

// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
void TopOptApp::drawScene() {
	applyDensityParametersToSimMesh();
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);
	glColor3d(1,1,1);
	simMesh->drawSimulationMesh();

	if (selectedNodeID >= 0) {
		P3D pos = simMesh->nodes[selectedNodeID]->getWorldPosition();
		drawSphere(pos, 0.02);
	}

	glColor3d(0,1,0);
	for (uint i = 0; i < externalLoads.size(); i++) {
		P3D pos = simMesh->nodes[i]->getWorldPosition();
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

