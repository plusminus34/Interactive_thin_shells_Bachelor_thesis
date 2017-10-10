#include <GUILib/GLUtils.h>
#include "PDEsApp.h"
#include <GUILib/GLMesh.h>
#include <GUILib/GLContentManager.h>
#include <MathLib/MathLib.h>
#include <MathLib/Plane.h>

//we assume the domain we are simulating over is centered at 0 and mapped to the interval -1..1 (no need to move the camera around)
#define N 200
//u
double gridVals[N][N] = { {0,}, };
//an old grid, used for tmp values
double oldGridVals[N][N] = { { 0, }, };
//a velocity grid used to store the time derivative of the function we solve for
double gridValsDot[N][N] = { { 0, }, };


//grid size and starting location
double startX = -1;
double startY = -0.8;
double dx = 2.0 / (N - 1);
double dy = 2.0 / (N - 1);

PDEsApp::PDEsApp() {
	setWindowTitle("Test PDE Application...");

	showGroundPlane = false;

	mainMenu->addGroup("PDE App options");
	mainMenu->addVariable("PDE", runOption, true)->setItems({ "Heat Diffusion Equation", "Wave equation"});
	menuScreen->performLayout();


}

PDEsApp::~PDEsApp(void){
}

//triggered when mouse moves
bool PDEsApp::onMouseMoveEvent(double xPos, double yPos) {
	lastClickedRay = getRayFromScreenCoords(xPos, yPos);

	currentRay = getRayFromScreenCoords(xPos, yPos);

	if (isDrawing) {
		P3D clickedPoint;
		getRayFromScreenCoords(xPos, yPos).getDistanceToPlane(Plane(P3D(), V3D(0, 0, 1)), &clickedPoint);
		//check to see what's the closest point that is in the grid that we did click on...
		if (clickedPoint[0] >= startX && clickedPoint[0] <= startX + 2 && clickedPoint[1] >= startY && clickedPoint[1] <= startY + 2.0) {
			int indexX = (int)((clickedPoint[0] - startX) / dx + 0.5);
			int indexY = (int)((clickedPoint[1] - startY) / dy + 0.5);
			if (indexX > 0 && indexX < N - 1 && indexY > 0 && indexY < N - 1) {
				gridVals[indexX - 1][indexY - 1] =
					gridVals[indexX - 1][indexY + 0] =
					gridVals[indexX - 1][indexY + 1] =
					gridVals[indexX - 0][indexY - 1] =
					gridVals[indexX - 0][indexY + 0] =
					gridVals[indexX - 0][indexY + 1] =
					gridVals[indexX + 1][indexY - 1] =
					gridVals[indexX + 1][indexY + 0] =
					gridVals[indexX + 1][indexY + 1] = 1.0;
			}
			else {
				gridVals[indexX][indexY] = 1.0;
			}
		}
		return true;
	}

	if (GLApplication::onMouseMoveEvent(xPos, yPos) == true) return true;
	return false;
}

//triggered when mouse buttons are pressed
bool PDEsApp::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
	isDrawing = false;
	if (button == GLFW_MOUSE_BUTTON_LEFT) {
		P3D clickedPoint;
		getRayFromScreenCoords(xPos, yPos).getDistanceToPlane(Plane(P3D(), V3D(0,0,1)), &clickedPoint);
		//check to see what's the closest point that is in the grid that we did click on...
		if (clickedPoint[0] >= startX && clickedPoint[0] <= startX + 2 && clickedPoint[1] >= startY && clickedPoint[1] <= startY + 2.0) {
			isDrawing = true;
			int indexX = (int)((clickedPoint[0] - startX) / dx + 0.5);
			int indexY = (int)((clickedPoint[1] - startY) / dy + 0.5);
			if (indexX > 0 && indexX < N-1 && indexY > 0 && indexY < N-1) {
				gridVals[indexX - 1][indexY - 1] = 
				gridVals[indexX - 1][indexY + 0] = 
				gridVals[indexX - 1][indexY + 1] = 
				gridVals[indexX - 0][indexY - 1] = 
				gridVals[indexX - 0][indexY + 0] = 
				gridVals[indexX - 0][indexY + 1] = 
				gridVals[indexX + 1][indexY - 1] = 
				gridVals[indexX + 1][indexY + 0] = 
				gridVals[indexX + 1][indexY + 1] = 1.0;
			}
			else {
				gridVals[indexX][indexY] = 1.0;
			}
		}
	}

	if (GLApplication::onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;

	return false;
}

//triggered when using the mouse wheel
bool PDEsApp::onMouseWheelScrollEvent(double xOffset, double yOffset) {
	if (GLApplication::onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	return false;
}

bool PDEsApp::onKeyEvent(int key, int action, int mods) {
	if (GLApplication::onKeyEvent(key, action, mods)) return true;

	return false;
}

bool PDEsApp::onCharacterPressedEvent(int key, int mods) {
	if (GLApplication::onCharacterPressedEvent(key, mods)) return true;

	return false;
}


void PDEsApp::loadFile(const char* fName) {
	Logger::consolePrint("Loading file \'%s\'...\n", fName);
	std::string fileName;
	fileName.assign(fName);

	std::string fNameExt = fileName.substr(fileName.find_last_of('.') + 1);
	Logger::consolePrint("...but how to do with that?");
}

void PDEsApp::saveFile(const char* fName) {
	Logger::consolePrint("SAVE FILE: Do not know what to do with file \'%s\'\n", fName);
}


// Run the App tasks
void PDEsApp::process() {
	//do the work here...

	static int lastRunOptionSol = -1;
	if (lastRunOptionSol != runOption) {
		memset(gridVals, 0, sizeof(double) * N * N);
		lastRunOptionSol = runOption;
	}

	double dt = 0.001;
	double simulationTime = 0;
	double maxRunningTime = 1.0 / desiredFrameRate;
	while (simulationTime < 1.0 * maxRunningTime) {
		simulationTime += dt;

		if (runOption == HEAT_DIFFUSION_EQUATION) {

			//solving the heat diffusion equation
			//uDot = au'' or du/dt

			//first, make sure we evaluate the derivatives on a mesh that is different than the one we write values onto
			memcpy(oldGridVals, gridVals, sizeof(double) * N * N);

			double thermalDiffusivity = 1.0e-3;

			for (uint i = 1; i < N - 1; i++) {
				for (uint j = 1; j < N - 1; j++) {
					//we need to estimate the lapacian of the variable we are solving for (u), using finite differences on the grid.
					//We will assume that the first and last row/column denote boundary conditions, and we will not update their values.
					double delSquaredU = (2 * oldGridVals[i][j] - oldGridVals[i + 1][j] - oldGridVals[i - 1][j]) / (dx * dx)
						+ (2 * oldGridVals[i][j] - oldGridVals[i][j + 1] - oldGridVals[i][j - 1]) / (dy * dy);

					//with this, we now know what the time derivative at the current grid point is, so we can integrate forward in time
					gridVals[i][j] += dt * delSquaredU * thermalDiffusivity * -1;
				}
			}
		}
		else {

			//solving the wave equation
			//uDotDot = au'' or du/dt

			//first, make sure we evaluate the derivatives on a mesh that is different than the one we write values onto
			memcpy(oldGridVals, gridVals, sizeof(double) * N * N);

			double c = 0.05;
			for (uint i = 1; i < N - 1; i++) {
				for (uint j = 1; j < N - 1; j++) {
					//we need to estimate the lapacian of the variable we are solving for (u), using finite differences on the grid.
					//We will assume that the first and last row/column denote boundary conditions, and we will not update their values.
					double delSquaredU = (2 * oldGridVals[i][j] - oldGridVals[i + 1][j] - oldGridVals[i - 1][j]) / (dx * dx)
						+ (2 * oldGridVals[i][j] - oldGridVals[i][j + 1] - oldGridVals[i][j - 1]) / (dy * dy);

					//with this, we can now update the time derivative
					gridValsDot[i][j] += dt * delSquaredU * c * -1;
					gridValsDot[i][j] *= 0.9999;//add some artificial damping...
												//and now update the function values
					gridVals[i][j] += dt * gridValsDot[i][j];

					//				if (gridVals[i][j] < 0 || gridVals[i][j] > 1) {
					//					Logger::consolePrint("%lf\n", gridVals[i][j]);
					//				}
				}
			}
		}
	}
}

double PDEsApp::getColorFor(double val) {
if (runOption == HEAT_DIFFUSION_EQUATION)
	return 1 - val;
else
	return 1 - (0.5 + 0.5 * val);
}

// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
void PDEsApp::drawScene() {
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);
	glColor3d(1,1,1);

	//draw the grid and the values it stores
/*	glPointSize(5.0);
	glBegin(GL_POINTS);
	for (int i = 0; i < N; i++)
		for (int j = 0; j < N; j++) {
			glColor3d(1, 1 - gridVals[i][j], 1 - gridVals[i][j]);
			glVertex3d(startX + i * dx, startY + j * dy, 0);
		}
	glEnd();
	glPointSize(1.0);
*/

	glBegin(GL_QUADS);
	for (int i = 0; i < N-1; i++)
		for (int j = 0; j < N-1; j++) {
			double c = 0;
			c = getColorFor(gridVals[i + 0][j + 0]);
			glColor3d(1, c, c);
			glVertex3d(startX + (i + 0) * dx, startY + (j + 0) * dy, 0);

			c = getColorFor(gridVals[i + 1][j + 0]);
			glColor3d(1, c, c);
			glVertex3d(startX + (i + 1) * dx, startY + (j + 0) * dy, 0);

			c = getColorFor(gridVals[i + 1][j + 1]);
			glColor3d(1, c, c);
			glVertex3d(startX + (i + 1) * dx, startY + (j + 1) * dy, 0);

			c = getColorFor(gridVals[i + 0][j + 1]);
			glColor3d(1, c, c);
			glVertex3d(startX + (i + 0) * dx, startY + (j + 1) * dy, 0);
		}
	glEnd();

}

// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
void PDEsApp::drawAuxiliarySceneInfo() {

}

// Restart the application.
void PDEsApp::restart() {

}

bool PDEsApp::processCommandLine(const std::string& cmdLine) {

	if (GLApplication::processCommandLine(cmdLine)) return true;

	return false;
}

