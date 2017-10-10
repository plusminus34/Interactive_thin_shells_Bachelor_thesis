#include <GUILib/GLUtils.h>
#include "FEMSim3DApp.h"
#include <GUILib/GLMesh.h>
#include <GUILib/GLContentManager.h>
#include <MathLib/MathLib.h>
#include <MathLib/Plane.h>
#include <FEMSimLib/CSTSimulationMesh3D.h>

FEMSim3DApp::FEMSim3DApp() {
    setWindowTitle("Test FEM Sim Application...");

    //CSTSimulationMesh2D::generateSquareTriMesh("../data/FEM/2d/triMeshTMP.tri2d", 0, 0, 0.1, 0.1, 2, 2);
    //CSTSimulationMesh2D::generateSquareTriMesh("../data/FEM/2d/grid.tri2d");
	CSTSimulationMesh3D::generateCubeTriMesh("../data/FEM/3d/tmpMesh.tet3d");
    femMesh = new CSTSimulationMesh3D();
    femMesh->readMeshFromFile("../data/FEM/3d/tmpMesh.tet3d");
    femMesh->addGravityForces(V3D(0, -4.9, 0));

	mainMenu->addGroup("FEM Sim options");
	mainMenu->addVariable("Static solve", computeStaticSolution);
	mainMenu->addVariable("Check derivatives", checkDerivatives);
	menuScreen->performLayout();

	showGroundPlane = false;
}

FEMSim3DApp::~FEMSim3DApp(void) {
}


//triggered when mouse moves
bool FEMSim3DApp::onMouseMoveEvent(double xPos, double yPos) {
    if (GLApplication::onMouseMoveEvent(xPos, yPos) == true) return true;

    return false;
}

//triggered when mouse buttons are pressed
bool FEMSim3DApp::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
    if (GLApplication::onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;

    return false;
}

//triggered when using the mouse wheel
bool FEMSim3DApp::onMouseWheelScrollEvent(double xOffset, double yOffset) {
    if (GLApplication::onMouseWheelScrollEvent(xOffset, yOffset)) return true;

    return false;
}

bool FEMSim3DApp::onKeyEvent(int key, int action, int mods) {
    if (GLApplication::onKeyEvent(key, action, mods)) return true;

    return false;
}

bool FEMSim3DApp::onCharacterPressedEvent(int key, int mods) {
    if (GLApplication::onCharacterPressedEvent(key, mods)) return true;

    return false;
}


void FEMSim3DApp::loadFile(const char* fName) {
    Logger::consolePrint("Loading file \'%s\'...\n", fName);
    std::string fileName;
    fileName.assign(fName);

    std::string fNameExt = fileName.substr(fileName.find_last_of('.') + 1);

}

void FEMSim3DApp::saveFile(const char* fName) {
    Logger::consolePrint("SAVE FILE: Do not know what to do with file \'%s\'\n", fName);
}


// Run the App tasks
void FEMSim3DApp::process() {
    //do the work here...
    double simulationTime = 0;
    double maxRunningTime = 1.0 / desiredFrameRate;
    femMesh->checkDerivatives = checkDerivatives != 0;

    //if we still have time during this frame, or if we need to finish the physics step, do this until the simulation time reaches the desired value
    while (simulationTime < 1.0 * maxRunningTime) {
        simulationTime += simTimeStep;
        if (computeStaticSolution)
            femMesh->solve_statics();
        else {
            femMesh->solve_dynamics(simTimeStep);
            femMesh->fakeContactWithPlane(Plane(P3D(0, 0, 0), V3D(0, 1, 0)));
        }
		if (slowMo)
			break;
    }
}

// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
void FEMSim3DApp::drawScene() {
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);
    glColor3d(1, 1, 1);
    femMesh->drawSimulationMesh();
}

// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
void FEMSim3DApp::drawAuxiliarySceneInfo() {

}

// Restart the application.
void FEMSim3DApp::restart() {

}

bool FEMSim3DApp::processCommandLine(const std::string& cmdLine) {

    if (GLApplication::processCommandLine(cmdLine)) return true;

    return false;
}

