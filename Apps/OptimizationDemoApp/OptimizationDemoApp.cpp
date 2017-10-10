#include <GUILib/GLUtils.h>
#include "OptimizationDemoApp.h"
#include <GUILib/GLMesh.h>
#include <GUILib/GLContentManager.h>
#include <GUILib/GLTexture.h>
#include <GUILib/GLWindow2D.h>
#include <MathLib/MathLib.h>
#include <OptimizationLib/GradientDescentFunctionMinimizer.h>
#include <OptimizationLib/NewtonFunctionMinimizer.h>
#include <OptimizationLib/MomentumBasedGradientFunctionMinimizer.h>
#include <Utils/Logger.h>
#include <algorithm>

//add parameters for momentum based optimizer and so on
//how is it that the history of bfgs and MBGD get initialized?

OptimizationDemoApp::OptimizationDemoApp(): LBFGSParam(), LBFGSSolver(LBFGSParam), minimizerGM(enableLineSearch, initialStep, betaMomentum, numIterations) {
	setWindowTitle("Test Application for Optimization Strategies");

	showGroundPlane = false;
	showDesignEnvironmentBox = false;

	mainMenu->addGroup("Optimization Demo");

	mainMenu->addVariable("Draw Contours", enableContours);
	mainMenu->addVariable("Contour Colors", enableContourColors);
	mainMenu->addVariable("Show Gradient Descent", enableGradient);
	mainMenu->addVariable("Show Newton's Method", enableNewton);
	mainMenu->addVariable("Show Gradient Momentum", enableGradientMomentum);
	mainMenu->addVariable("Show LBFGS", enableLBFGS);

	mainMenu->addVariable("Gradient Descent", colorGradient, true)->setItems({ "Red","Green","Blue","Magenta","Cyan","Yellow","Black","Grey" });
	mainMenu->addVariable("Newton's Method", colorNewton, true)->setItems({ "Red","Green","Blue","Magenta","Cyan","Yellow","Black","Grey" });
	mainMenu->addVariable("Gradient Momentum", colorGradientMomentum, true)->setItems({ "Red","Green","Blue","Magenta","Cyan","Yellow","Black","Grey" });
	mainMenu->addVariable("LBFGS", colorLBFGS, true)->setItems({ "Red","Green","Blue","Magenta","Cyan","Yellow","Black","Grey" });

	//Gradient momentum algorithm control panel
	mainMenu->addGroup("Momentum GD Control Panel");
	mainMenu->addVariable("Enable Line Search", enableLineSearch);
	mainMenu->addVariable("Initial Step", initialStep);
	mainMenu->addVariable("Momentum", betaMomentum);

	menuScreen->performLayout();

	//Beichen Li: initialize the objective function
	function = new TestFunction();

	//Beichen Li: camera distance
	camera->setCameraTarget(P3D(0.0, 0.0, 8.0));

	//Beichen Li: allocate cache array
	cache = new int[resolution << 1 | 1];

	//Beichen Li: initialize isolevel points and color areas
	initContours();

	//Beichen Li: initialize optimization task
	initOptimization();
}

OptimizationDemoApp::~OptimizationDemoApp(void){
	delete[] cache;
	delete function;
}


//triggered when mouse moves
bool OptimizationDemoApp::onMouseMoveEvent(double xPos, double yPos) {

	if (showConsole)
		consoleWindow->onMouseMoveEvent(xPos, yPos);
	return false;
}

//triggered when mouse buttons are pressed
bool OptimizationDemoApp::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
	if (button == GLFW_MOUSE_BUTTON_1 && action == GLFW_PRESS) {
		glEnd();

		Ray thisRay = getRayFromScreenCoords(xPos, yPos);
		P3D point = thisRay.origin;
		V3D direction = thisRay.direction;
		point -= direction * (point.z() / direction.z());

		if (fabs(point.x()) < planeWidth && fabs(point.y()) < planeWidth) {
			resetOptimization(P3D(point.x(), point.y()));
		}

		return true;
	}

	return false;
}

//triggered when using the mouse wheel
bool OptimizationDemoApp::onMouseWheelScrollEvent(double xOffset, double yOffset) {
	return false;
}

bool OptimizationDemoApp::onKeyEvent(int key, int action, int mods) {
	if (action == GLFW_PRESS) {
		switch (key) {
		case GLFW_KEY_1:
			glEnd(); enableGradient = !enableGradient; break;
		case GLFW_KEY_2:
			glEnd(); enableNewton = !enableNewton; break;
		case GLFW_KEY_3:
			glEnd(); enableGradientMomentum = !enableGradientMomentum; break;
		case GLFW_KEY_4:
			glEnd(); enableLBFGS = !enableLBFGS; break;
		case GLFW_KEY_W:
			glEnd(); betaMomentum += 0.01; break;
		case GLFW_KEY_S:
			glEnd(); betaMomentum -= 0.01; break;
		case GLFW_KEY_A:
			glEnd(); initialStep += 0.01; break;
		case GLFW_KEY_D:
			glEnd(); initialStep -= 0.01; break;
		case GLFW_KEY_E:
			glEnd(); enableLineSearch = !enableLineSearch; break;
		case GLFW_KEY_9:
			glEnd(); enableContours = !enableContours; break;
		case GLFW_KEY_0:
			glEnd(); enableContourColors = !enableContourColors; break;
		case GLFW_KEY_R:
			glEnd(); restart(); break;
		case GLFW_KEY_BACKSPACE:
			glEnd(); resume(); break;
		}
	}
	
	if (GLApplication::onKeyEvent(key, action, mods)) return true;

	return false;
}

bool OptimizationDemoApp::onCharacterPressedEvent(int key, int mods) {
	if (GLApplication::onCharacterPressedEvent(key, mods)) return true;

	return false;
}

void OptimizationDemoApp::loadFile(const char* fName) {
	Logger::consolePrint("Loading file \'%s\'...\n", fName);
	std::string fileName;
	fileName.assign(fName);
	
	std::string fNameExt = fileName.substr(fileName.find_last_of('.') + 1);

}

void OptimizationDemoApp::saveFile(const char* fName) {
	Logger::consolePrint("SAVE FILE: Do not know what to do with file \'%s\'\n", fName);
}


// Run the App tasks
void OptimizationDemoApp::process() {
	//do the work here...
	dVector p;

	iter += numIterations;

	GradientDescentFunctionMinimizer minimizerG(numIterations);
	p = point2Vector(pathGradient.back());
	minimizerG.minimize(function, p, energyGradient);
	pathGradient.push_back(P3D(p[0], p[1]));

	NewtonFunctionMinimizer minimizerN(numIterations);
	p = point2Vector(pathNewton.back());
	minimizerN.minimize(function, p, energyNewton);
	pathNewton.push_back(P3D(p[0], p[1]));

	minimizerGM.enableLineSearch = enableLineSearch;
	minimizerGM.initialStep = initialStep;
	minimizerGM.beta = betaMomentum;
	minimizerGM.maxIterations = numIterations;

	p = point2Vector(pathGradientMomentum.back());
	minimizerGM.minimize(function, p, energyGradientMomentum);
	pathGradientMomentum.push_back(P3D(p[0], p[1]));

	p = point2Vector(pathLBFGS.back());
	LBFGSSolver.DEMOMinimize(*this, p, energyLBFGS);
	pathLBFGS.push_back(P3D(p[0], p[1]));

	if (logFlag)
		logStatus();
}

void OptimizationDemoApp::resume() {
	P3D current;
	if (iter > 0) {
		iter--;

		pathGradient.pop_back();
		pathNewton.pop_back();
		pathGradientMomentum.pop_back();
		pathLBFGS.pop_back();

		energyGradient = function->computeValue(point2Vector(pathGradient.back()));
		energyNewton = function->computeValue(point2Vector(pathNewton.back()));
		energyGradientMomentum = function->computeValue(point2Vector(pathGradientMomentum.back()));
		energyLBFGS = function->computeValue(point2Vector(pathLBFGS.back()));

		LBFGSSolver.resume();

		logStatus();
	}
}

// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
void OptimizationDemoApp::drawScene() {
	//Check control parameters
	if (enableLineSearch != lastEnableLineSearch || initialStep != lastInitialStep || betaMomentum != lastBetaMomentum) {
		lastEnableLineSearch = enableLineSearch;
		lastInitialStep = initialStep;
		lastBetaMomentum = betaMomentum;

		int lastIter = iter;
		resetOptimization(pathGradientMomentum.front());
		logFlag = false;
		for (int i = 0; i < lastIter; i++)
			process();
		logFlag = true;
		logStatus();
	}

	double x = planeWidth;
	
	//Beichen Li: draw a white square as background
	glColor3d(1.0, 1.0, 1.0);
	glBegin(GL_POLYGON);
	glVertex3d(x, x, 0.0);
	glVertex3d(-x, x, 0.0);
	glVertex3d(-x, -x, 0.0);
	glVertex3d(x, -x, 0.0);
	glEnd();

	//Beichen Li: draw contour lines (isolevel curves)
	if (enableContours) {
		drawContours();
		drawColorAreas();
	}

	//Beichen Li: draw optimization paths
	drawPaths();
}

void OptimizationDemoApp::drawContours() {
	int end = isolevelPoints.size();
	double color, x, y;
	double dt = planeWidth / resolution;
	double red = 0.0, blue = 0.0;

	for (int i = 0; i < end; i++) {
		color = (isolevelPoints[i].colorDeg - minDeg) * 1.0 / (maxDeg - minDeg);
		x = isolevelPoints[i].ix * dt;
		y = isolevelPoints[i].iy * dt;

		//Beichen Li: Isolevel point color is set by interpolation between red and blue
		if (enableContourColors) {
			red = color < 0.5 ? 1.0 : 2 * (1 - color);
			blue = color < 0.5 ? 2 * color : 1.0;
		}

		glColor3d(red, 0, blue);
		glBegin(GL_POLYGON);
		gl_Vertex3d(P3D(x, y, 0));
		gl_Vertex3d(P3D(x + dt, y, 0));
		gl_Vertex3d(P3D(x + dt, y + dt, 0));
		gl_Vertex3d(P3D(x, y + dt, 0));
		glEnd();
	}
}

void OptimizationDemoApp::drawColorAreas() {
	int end = colorAreas.size();
	double color, x, y1, y2;
	double dt = planeWidth / resolution;
	double red = 0.0, blue = 0.0, green = 0.0;
	double saturation = 0.25;
	double saturationGrey = 0.50;

	for (int i = 0; i < end; i++) {
		color = (colorAreas[i].colorDeg - minDeg) * 1.0 / (maxDeg - minDeg);
		x = colorAreas[i].ix * dt;
		y1 = colorAreas[i].iyStart * dt;
		y2 = colorAreas[i].iyEnd * dt;

		if (enableContourColors) {
			red = (color < 0.5 ? 1.0 : 2 * (1 - color)) * saturation + (1 - saturation);
			green = 1 - saturation;
			blue = (color < 0.5 ? 2 * color : 1.0) * saturation + (1 - saturation);
		}
		else
			red = green = blue = color * saturationGrey + (1 - saturationGrey);

		glColor3d(red, green, blue);
		glBegin(GL_POLYGON);
		gl_Vertex3d(P3D(x, y1, 0));
		gl_Vertex3d(P3D(x + dt, y1, 0));
		gl_Vertex3d(P3D(x + dt, y2 + dt, 0));
		gl_Vertex3d(P3D(x, y2 + dt, 0));
		glEnd();
	}
}

void OptimizationDemoApp::drawPaths() {
	//Beichen Li: Gradient descent (color = red)
	if (enableGradient) {
		switchColor(colorGradient);
		drawOptimizationPath(pathGradient);
	}

	//Beichen Li: Newton's method (color = green)
	if (enableNewton) {
		switchColor(colorNewton);
		drawOptimizationPath(pathNewton);
	}

	//Beichen Li: Gradient descent with momentum (color = blue)
	if (enableGradientMomentum) {
		switchColor(colorGradientMomentum);
		drawOptimizationPath(pathGradientMomentum);
	}

	//Beichen Li: LBFGS (color = magenta)
	if (enableLBFGS) {
		switchColor(colorLBFGS);
		drawOptimizationPath(pathLBFGS);
	}
}

void OptimizationDemoApp::drawOptimizationPath(DynamicArray<P3D>& path) {
	int end = path.size();
	glPointSize(10.0);
	glBegin(GL_POINTS);
	for (int i = 0; i < end; i++)
		gl_Vertex3d(path[i]);
	glEnd();
	
	glLineWidth(4.0);
	glBegin(GL_LINE_STRIP);
	for (int i = 0; i < end; i++)
		gl_Vertex3d(path[i]);
	glEnd();
}

// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
void OptimizationDemoApp::drawAuxiliarySceneInfo() {

}

// Restart the application.
void OptimizationDemoApp::restart() {
	resetOptimization(pathGradient[0]);
}

bool OptimizationDemoApp::processCommandLine(const std::string& cmdLine) {

//	if (GLApplication::processCommandLine(cmdLine)) return true;

	return false;
}
void OptimizationDemoApp::initContours() {
	int end = resolution;
	double dt = planeWidth / resolution;
	double x, y;
	int f1, f2, f3, f4;
	int colorDeg;
	int areaStart;

	for (int ix = -end; ix < end; ix++) {
		//Beichen Li: set the position indicating the starting point of a color area
		areaStart = -end;

		for (int iy = -end; iy < end; iy++) {
			x = ix * dt;
			y = iy * dt;

			dVector p(2);
			p[0] = x, p[1] = y;
			f1 = iy == -end ? (ix == -end ? int(sqrt(function->computeValue(p)) / isolevelInterval) : cache[iy + end]) : f2;
			p[1] = y + dt;
			f2 = ix == -end ? int(sqrt(function->computeValue(p)) / isolevelInterval) : cache[iy + end + 1];
			p[0] = x + dt; p[1] = y;
			f3 = cache[iy + end] = iy == -end ? int(sqrt(function->computeValue(p)) / isolevelInterval) : f4;
			p[1] = y + dt;
			f4 = int(sqrt(function->computeValue(p)) / isolevelInterval);

			//Beichen Li: create a isolevel point here
			if (f1 != f2 || f2 != f3 || f3 != f4) {
				if (areaStart < iy)
					colorAreas.push_back(ColorArea(ix, areaStart, iy - 1, colorDeg));

				colorDeg = f1 > f2 ? f1 : f2;
				colorDeg = f3 > colorDeg ? f3 : colorDeg;
				colorDeg = f4 > colorDeg ? f4 : colorDeg;
				isolevelPoints.push_back(IsolevelPoint(ix, iy, colorDeg));
				areaStart = iy + 1;
			}
			else
				colorDeg = f1;
		}

		if (areaStart <= end)
			colorAreas.push_back(ColorArea(ix, areaStart, end, colorDeg));

		cache[end << 1] = f4;
	}

	end = isolevelPoints.size();
	minDeg = 0x7fffffff;
	maxDeg = 0x80000000;
	for (int i = 0; i < end; i++) {
		maxDeg = (std::max)(isolevelPoints[i].colorDeg, maxDeg);
		minDeg = (std::min)(isolevelPoints[i].colorDeg, minDeg);
	}
	end = colorAreas.size();
	for (int i = 0; i < end; i++) {
		maxDeg = (std::max)(colorAreas[i].colorDeg, maxDeg);
		minDeg = (std::min)(colorAreas[i].colorDeg, minDeg);
	}
}

void OptimizationDemoApp::initOptimization() {
	P3D start(-3.0, -4.0);
	resize(minimizerGM.dpLast, 2);

	//Setting the starting point for each optimization method
	pathGradient.push_back(start);
	pathNewton.push_back(start);
	pathGradientMomentum.push_back(start);
	pathLBFGS.push_back(start);
}

void OptimizationDemoApp::resetOptimization(const P3D& origin) {
	pathGradient.clear();
	pathNewton.clear();
	pathGradientMomentum.clear();
	pathLBFGS.clear();

	pathGradient.push_back(origin);
	pathNewton.push_back(origin);
	pathGradientMomentum.push_back(origin);
	pathLBFGS.push_back(origin);

	LBFGSSolver.k = 0;
	LBFGSSolver.end = 0;
	iter = 0;
	resize(minimizerGM.dpLast, 2);
}
