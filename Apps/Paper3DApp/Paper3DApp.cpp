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
#include <FEMSimLib/BendingEdge.h>
#include <FEMSimLib/Pin.h>

Paper3DApp::Paper3DApp() {
	setWindowTitle("3D Paper Application");
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
	pin_k = 100;
	mouse_mode = mouse_drag;

	int dim_x = 30;
	int dim_y = 21;
	double h = 0.1;
	Paper3DMesh::generateRectangleSystem("../data/FEM/3d/PaperSheet.tri3d", dim_x, dim_y, h*(dim_x-1), h*(dim_y-1));

	simMesh = new Paper3DMesh();
	simMesh->readMeshFromFile("../data/FEM/3d/PaperSheet.tri3d");

	simMesh->setPinnedNode(0, simMesh->nodes[0]->getUndeformedPosition());
	simMesh->setPinnedNode(dim_y, simMesh->nodes[dim_y]->getUndeformedPosition());
	simMesh->setPinnedNode(dim_y + 1, simMesh->nodes[dim_y + 1]->getUndeformedPosition());

	// obvious note: real paper is affected by gravity
	//simMesh->addGravityForces(V3D(0.0, 0.0, 0.0));

	//slightly deformed initial configuration to avoid some annoying problems occurring if everything is in one plane
	for (int i = 0; i < dim_x; ++i)
		for (int j = 0; j < dim_y; ++j) {
			double dz = 0.5 * (i + j) * h / (dim_x + dim_y);
			P3D newPos(i*h, j*h, dz*dz);
			simMesh->nodes[dim_y*i + j]->setWorldPosition(newPos);
		}

	mainMenu->addGroup("File options");
	nanogui::Widget *tools = new nanogui::Widget(mainMenu->window());
	mainMenu->addWidget("", tools);
	tools->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
		nanogui::Alignment::Middle, 0, 4));
	nanogui::Button* button;
	button = new nanogui::Button(tools, "");
	button->setIcon(ENTYPO_ICON_SAVE);
	button->setCallback([this]() {
		std::string fileName = nanogui::file_dialog({ { "pap3d","Paper 3D file" } }, true);
		if (fileName.length() < 6 || 0 != fileName.compare(fileName.length() - 6, 6, ".pap3d"))
			fileName.append(".pap3d");
		saveFile(fileName.c_str());
	});
	button->setTooltip("Save File");

	button = new nanogui::Button(tools, "");
	button->setIcon(ENTYPO_ICON_DOWNLOAD);
	button->setCallback([this]() {
		std::string fileName = nanogui::file_dialog({ { "pap3d","Paper 3D file" } }, false);
		loadFile(fileName.c_str());
	});
	button->setTooltip("Load File");

	mainMenu->addGroup("FEM Sim options");
	mainMenu->addVariable("Check derivatives", checkDerivatives);
	mainMenu->addVariable("Shear modulus", shearModulus);
	mainMenu->addVariable("Bulk modulus", bulkModulus);
	mainMenu->addVariable("Bending stiffness", bend_k);
	mainMenu->addVariable("Pin stiffness", pin_k);
	mainMenu->addVariable("Mouse Mode", mouse_mode, true)->setItems(
		{ "Drag", "Cut", "Create Pin", "Move Pin", "Rotate Pin", "Flip Pin", "Delete Pin" });

	menuScreen->performLayout();
	setupWindows();
	shapeWindow->setGridDimensions(dim_x, dim_y, h);
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

P3D Paper3DApp::getNodeRestPos(int i) {
	return simMesh->nodes[i]->getUndeformedPosition();
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
	if (simWindow->mouseIsWithinWindow(xPos, yPos)) {
		simWindow->onMouseButtonEvent(button, action, mods, xPos, yPos);
	}
	else if (shapeWindow->mouseIsWithinWindow(xPos, yPos)) {
		shapeWindow->onMouseButtonEvent(button, action, mods, xPos, yPos);
	}
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
	Logger::consolePrint("LOAD FILE: Loading file \'%s\'...\n", fName);

	int num_nodes, num_triangles, num_fixed, num_pins;
	MatrixNxM xX;
	Eigen::MatrixXi T;
	VectorXT<int> fixNode;
	MatrixNxM fixPos;
	MatrixNxM pins;
	int grid_width, grid_height;
	double cell_size;

	FILE* fp = fopen(fName, "r");
	fscanf(fp, "%d %d %d", &num_nodes, &num_triangles, &num_fixed);
	fscanf(fp, "%d %d %lf %d", &grid_width, &grid_height, &cell_size, &num_pins);
	xX.resize(num_nodes, 6);
	for (int i = 0; i < num_nodes; ++i)
		fscanf(fp, "%lf %lf %lf %lf %lf %lf", &xX(i, 0), &xX(i, 1), &xX(i, 2), &xX(i, 3), &xX(i, 4), &xX(i, 5));
	T.resize(num_triangles, 3);
	for (int i = 0; i < num_triangles; ++i)
		fscanf(fp, "%d %d %d", &T(i, 0), &T(i, 1), &T(i, 2));
	fixNode.resize(num_fixed);
	fixPos.resize(num_fixed, 3);
	for (int i = 0; i < num_fixed; ++i)
		fscanf(fp, "%d %lf %lf %lf", &fixNode[i], &fixPos(i,0), &fixPos(i, 1), &fixPos(i, 2));
	pins.resize(num_pins, 8);
	for (int i = 0; i < num_pins; ++i)
		fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf", &pins(i, 0), &pins(i, 1), &pins(i, 2), &pins(i, 3),
			&pins(i, 4), &pins(i, 5), &pins(i, 6), &pins(i, 7));
	fclose(fp);
	Logger::consolePrint("LOAD FILE: File \'%s\' read\n", fName);

	simMesh->applyLoadData(xX, T, fixNode, fixPos);
	shapeWindow->applyLoadData(grid_width, grid_height, cell_size, pins);
	Logger::consolePrint("LOAD FILE: Loading complete\n", fName);
}

void Paper3DApp::saveFile(const char* fName) {
	Logger::consolePrint("SAVE FILE: Write to file \'%s\'\n", fName);
	// Should material parameters be saved too?
	MatrixNxM xX;
	Eigen::MatrixXi T;
	VectorXT<int> fixNode;
	MatrixNxM fixPos;
	MatrixNxM pins;
	int grid_width, grid_height;
	double cell_size;

	simMesh->getSaveData(xX,T,fixNode,fixPos);
	shapeWindow->getSaveData(grid_width, grid_height, cell_size, pins);

	FILE* fp = fopen(fName, "w");
	fprintf(fp, "%d %d %d\n", xX.rows(), T.rows(), fixNode.size());
	fprintf(fp, "%d %d %f %d\n\n", grid_width, grid_height, cell_size, pins.rows());
	for (int i = 0; i < xX.rows(); ++i)
		fprintf(fp, "%f %f %f %f %f %f\n", xX(i, 0), xX(i, 1), xX(i, 2), xX(i, 3), xX(i, 4), xX(i, 5));
	fprintf(fp, "\n");
	for (int i = 0; i < T.rows(); ++i)
		fprintf(fp, "%d %d %d\n", T(i, 0), T(i, 1), T(i, 2));
	fprintf(fp, "\n");
	for (int i = 0; i < fixNode.size(); ++i)
		fprintf(fp, "%d %f %f %f\n", fixNode[i], fixPos(i, 0), fixPos(i, 1), fixPos(i, 2));
	fprintf(fp, "\n");
	for (int i = 0; i < pins.rows(); ++i)
		fprintf(fp, "%f %f %f %f %f %f %f %f\n", pins(i,0), pins(i, 1), pins(i, 2), pins(i, 3),
			pins(i, 4), pins(i, 5), pins(i, 6), pins(i, 7));
	fclose(fp);
	Logger::consolePrint("SAVE FILE: File \'%s\' written\n", fName);
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

	shapeWindow->drawScene();
	simWindow->drawScene();

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

	shapeWindow->drawAuxiliarySceneInfo();
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
		else if (Pin* e = dynamic_cast<Pin*>(simMesh->elements[i])) {
			e->setStiffness(pin_k);
		}
	}
}

