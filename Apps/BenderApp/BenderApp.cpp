#include <GUILib/GLUtils.h>
#include "BenderApp.h"
#include <GUILib/GLMesh.h>
#include <GUILib/GLContentManager.h>
#include <MathLib/MathLib.h>
#include <MathLib/Plane.h>
#include <FEMSimLib/CSTSimulationMesh2D.h>
#include <FEMSimLib/CSTSimulationMesh3D.h>
#include <FEMSimLib/MassSpringSimulationMesh3D.h>
#include <GUILib/GLUtils.h>


#define EDIT_BOUNDARY_CONDITIONS

//#define CONSTRAINED_DYNAMICS_DEMO

BenderApp::BenderApp() 
: mounts(9)
{
	setWindowTitle("Test FEM Sim Application...");

#ifdef CONSTRAINED_DYNAMICS_DEMO
	CSTSimulationMesh2D::generateSquareTriMesh("../data/FEM/2d/triMeshTMP.tri2d", -1, 0, 0.1, 0.1, 1, 1);

	femMesh = new CSTSimulationMesh2D();
	femMesh->readMeshFromFile("../data/FEM/2d/triMeshTMP.tri2d");
	femMesh->nodes[0]->addMassContribution(1.0);
	femMesh->addGravityForces(V3D(0, -9.8, 0));
#else

	int nRows = 20;
	int nCols = 20;
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
	mainMenu->addVariable("Static solve", computeStaticSolution);
	mainMenu->addVariable("Check derivatives", checkDerivatives);
	menuScreen->performLayout();


}

BenderApp::~BenderApp(void){
}

//triggered when mouse moves
bool BenderApp::onMouseMoveEvent(double xPos, double yPos) {
	lastClickedRay = getRayFromScreenCoords(xPos, yPos);

	currentRay = getRayFromScreenCoords(xPos, yPos);
	if (selectedNodeID != -1){
		int selectedMountID = getMountId(selectedNodeID);
		std::cout << "node belongs to mount " << selectedMountID << std::endl;
		if (selectedMountID < 0) {
			Plane plane(camera->getCameraTarget(),V3D(camera->getCameraPosition(),camera->getCameraTarget()).unit());
			P3D targetPinPos; getRayFromScreenCoords(xPos,yPos).getDistanceToPlane(plane,&targetPinPos);
			femMesh->setPinnedNode(selectedNodeID,targetPinPos);
			return true;
		}
		else {
			Plane plane(camera->getCameraTarget(),V3D(camera->getCameraPosition(),camera->getCameraTarget()).unit());
			P3D targetPos; 
			getRayFromScreenCoords(xPos,yPos).getDistanceToPlane(plane,&targetPos);
			V3D delta = targetPos - femMesh->nodes[selectedNodeID]->getWorldPosition();
			mounts[selectedMountID].shift(delta);
			updateMountEnergy();
			return true;
		}
	}
	else if (GLApplication::onMouseMoveEvent(xPos, yPos) == true) return true;
	return false;
}

//triggered when mouse buttons are pressed
bool BenderApp::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
#ifdef EDIT_BOUNDARY_CONDITIONS

	if (button == GLFW_MOUSE_BUTTON_RIGHT) {
		if (action == GLFW_PRESS && mods & GLFW_MOD_SHIFT) {
			femMesh->removePinnedNodeConstraints();
			for (Mount & m : mounts) {
				m.clear();
			}
		}
	}

	if (button == GLFW_MOUSE_BUTTON_LEFT){
		if (action == GLFW_PRESS && mods & GLFW_MOD_SHIFT) {
			selectedNodeID = femMesh->getSelectedNodeID(lastClickedRay);
			if (selectedNodeID >= 0) {
				if (selected_mount >= 0 && selected_mount <= 9) {
					addMountedNode(selected_mount,selectedNodeID);
				}
				return true;
			}

		}
		else {
			selectedNodeID = -1;
		}
	}
#endif

	if (GLApplication::onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;

//	Logger::consolePrint("--> %d\n", selectedNodeID);
//	Logger::consolePrint("--> %lf %lf %lf\n", lastClickedRay.origin[0], lastClickedRay.origin[1], lastClickedRay.origin[2]);
//	Logger::consolePrint("--> %lf %lf %lf\n", lastClickedRay.direction[0], lastClickedRay.direction[1], lastClickedRay.direction[2]);
	return false;
}

//triggered when using the mouse wheel
bool BenderApp::onMouseWheelScrollEvent(double xOffset, double yOffset) {
	
	if (selected_mount > 0) {
		std::cout << "Offsets: " << xOffset << " " << yOffset << std::endl;
		Plane plane(camera->getCameraTarget(),V3D(camera->getCameraPosition(),camera->getCameraTarget()).unit());
		P3D origin; 
		lastClickedRay.getDistanceToPlane(plane,&origin);
		mounts[selected_mount].rotate2D(origin, yOffset * 0.05);
		updateMountEnergy();
		return(true);
	}
	
	if (GLApplication::onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	return false;
}

bool BenderApp::onKeyEvent(int key, int action, int mods) {	
	
	if (GLApplication::onKeyEvent(key, action, mods)) return true;

	return false;
}

bool BenderApp::onCharacterPressedEvent(int key, int mods) {
#ifdef EDIT_BOUNDARY_CONDITIONS
	if (!mods) {
		if (key >= '0' && key <= '9') {
			int mount_id = key - '0';
			selected_mount = mount_id;
			std::cout << "selected mount: " << selected_mount << std::endl;
		}
		else if (key == 'x') {
			selected_mount = -1;
			std::cout << "no mount selected" << std::endl;
		}
	}
#endif	
	if (GLApplication::onCharacterPressedEvent(key, mods)) return true;

	return false;
}


void BenderApp::loadFile(const char* fName) {
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

void BenderApp::saveFile(const char* fName) {
	Logger::consolePrint("SAVE FILE: Do not know what to do with file \'%s\'\n", fName);
}


// Run the App tasks
void BenderApp::process() {
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
	}
}

// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
void BenderApp::drawScene() {
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
void BenderApp::drawAuxiliarySceneInfo() {

}

// Restart the application.
void BenderApp::restart() {

}


void BenderApp::addMountedNode(int mount_id, int node_id)
{
	for (Mount & m : mounts) {
		m.unassignPinnedNode(mount_id);
	}

	mounts[mount_id].assignPinnedNode(node_id, femMesh->nodes[node_id]->getWorldPosition());
	updateMountEnergy();
}

void BenderApp::updateMountEnergy()
{
	for (Mount & m : mounts) {
		for (size_t i = 0; i < m.node_id.size(); ++i) {
			femMesh->setPinnedNode(m.node_id[i],m.position[i]);
		}
	}
}

int BenderApp::getMountId(int node_id)
{
	for (size_t i = 0; i < mounts.size(); ++i) {
		for (int id : mounts[i].node_id) {
			if (id == node_id) {
				return(i);
			}
		}
	}
	return(-1);
}

bool BenderApp::processCommandLine(const std::string& cmdLine) {

	if (GLApplication::processCommandLine(cmdLine)) return true;

	return false;
}

