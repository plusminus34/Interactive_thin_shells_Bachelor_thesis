#include <GUILib/GLUtils.h>
#include "Paper2DApp.h"
#include <GUILib/GLMesh.h>
#include <GUILib/GLContentManager.h>
#include <MathLib/MathLib.h>
#include <MathLib/Plane.h>
#include <FEMSimLib/MassSpringSimulationMesh3D.h>

#include <GUILib/GLUtils.h>
#include <GUILib/GLTrackingCamera.h>
//#include <OptimizationLib\SQPFunctionMinimizer.h>
#include "Paper2DMesh.h"
#include <vector>

double xdum, ydum;
// TODO move all of this elsewhere
void generateSinMassSpringSystem(char* fName, int nNodes) {
	FILE* fp = fopen(fName, "w");
	fprintf(fp, "%d %d %d\n\n", nNodes, nNodes-1, nNodes-2);

	//Silly way to get some equidistant nodes
	std::vector<double> xpos(nNodes);
	std::vector<double> ypos(nNodes);

	xpos[0] = ypos[0] = 0.0;

	double dx = 0.1;
	double nnn = 0.5;
	double target_length = 3.0;
	double eps = 0.01;
	for (int i = 1;i < nNodes;++i) {
		xpos[i] = xpos[i - 1] + dx;
		ypos[i] = sin(xpos[i]*nnn*2*PI);
		double l2 = (xpos[i] - xpos[i - 1])*(xpos[i] - xpos[i - 1]) + (ypos[i] - ypos[i - 1])*(ypos[i] - ypos[i - 1]);
		double target_l2 = target_length / (nNodes - 1);
		if ((l2 - target_l2)*(l2 - target_l2) > eps) {
			if (l2 > target_l2){
				dx *= 0.9;
			}else{
				dx *= 1.1;
			}
			--i;
		}
	}
	xdum = xpos[nNodes - 1];ydum = ypos[nNodes - 1];// TODO: I REALLY shouldn't do it like this.

	double h = 1.0 / (nNodes - 1);

	//generate particles
	for (int j = 0; j<nNodes; j++)
		fprintf(fp, "%lf %lf %lf\n", xpos[j], ypos[j], 0.0);

	fprintf(fp, "\n\n");

	//generate springs between particles
	for (int j = 1; j<nNodes; j++)
		fprintf(fp, "%li %li\n", j-1, j);

	//generate angle springs
	for (int j = 1; j<nNodes-1; j++)
		fprintf(fp, "%li %li %li\n", j - 1, j, j+1);

	fclose(fp);
}

Paper2DApp::Paper2DApp() {
	setWindowTitle("Test Paper2D Application...");


	delete camera;
	GLTrackingCamera *cam = new GLTrackingCamera();
	cam->ignoreRotations = true;
	camera = cam;

	bgColorR = bgColorG = bgColorB = 0.5;

	int N = 15;

	generateSinMassSpringSystem("../data/FEM/3d/sinMassSpringSystem.ms",N);
	simMesh = new Paper2DMesh();
	simMesh->readMeshFromFile("../data/FEM/3d/sinMassSpringSystem.ms");
	simMesh->addGravityForces(V3D(0, -9.8, 0));

	simMesh->setPinnedNode(0, V3D(0.0, 0.0, 0.0));
	simMesh->setPinnedNode(N-1, V3D(xdum, ydum, 0.0));
}

void Paper2DApp::applyDensityParametersToSimMesh() {
	//TODO remove?
}


Paper2DApp::~Paper2DApp(void){
}

//triggered when mouse moves
bool Paper2DApp::onMouseMoveEvent(double xPos, double yPos) {
	if (GLApplication::onMouseMoveEvent(xPos, yPos) == true) return true;
	return false;
}

//triggered when mouse buttons are pressed
bool Paper2DApp::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
	if (GLApplication::onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;
	return false;
}

//triggered when using the mouse wheel
bool Paper2DApp::onMouseWheelScrollEvent(double xOffset, double yOffset) {
	if (GLApplication::onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	return false;
}

bool Paper2DApp::onKeyEvent(int key, int action, int mods) {
	if (GLApplication::onKeyEvent(key, action, mods)) return true;

	return false;
}

bool Paper2DApp::onCharacterPressedEvent(int key, int mods) {
	if (GLApplication::onCharacterPressedEvent(key, mods)) return true;

	return false;
}


void Paper2DApp::loadFile(const char* fName) {
	Logger::consolePrint("Loading file \'%s\'...\n", fName);
	Logger::consolePrint("...or not.\n");
}

void Paper2DApp::saveFile(const char* fName) {
	Logger::consolePrint("SAVE FILE: Do not know what to do with file \'%s\'\n", fName);
}


// Run the App tasks
void Paper2DApp::process() {
	//do the work here...
	simMesh->checkDerivatives = true;
	simMesh->solve_statics();
	//simMesh->solve_dynamics(0.25);
}

// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
void Paper2DApp::drawScene() {
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);
	glColor3d(1, 1, 1);

	simMesh->drawSimulationMesh();
	/*
	for (int i = 3; i < 9; ++i) {
		glColor3d(0.5, 0.1 + 0.1 * i, 1.0);
		glBegin(GL_LINE_LOOP);
		for (int j = 0; j < i; ++j) {
			glVertex3d(0.2*i+0.2*j-1, 0.1*j*j, 0);
		}
		glEnd();
	}*/
}

// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
void Paper2DApp::drawAuxiliarySceneInfo() {

}

// Restart the application.
void Paper2DApp::restart() {

}

bool Paper2DApp::processCommandLine(const std::string& cmdLine) {

	if (GLApplication::processCommandLine(cmdLine)) return true;

	return false;
}

