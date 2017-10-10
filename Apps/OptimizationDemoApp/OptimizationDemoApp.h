#pragma once

#include <GUILib/GLApplication.h>
#include <string>
#include <map>

#include <GUILib/TranslateWidget.h>
#include <OptimizationLib/ObjectiveFunction.h>
#include <OptimizationLib/LBFGS.h>
#include <OptimizationLib/MomentumBasedGradientFunctionMinimizer.h>

//Beichen Li: class definition for isolevel points
class IsolevelPoint {
public:
	IsolevelPoint(int x, int y, int colorDeg): ix(x), iy(y), colorDeg(colorDeg) {}
	
	int ix;
	int iy;
	int colorDeg;
};

//Beichen Li: enum for colors
enum Color {COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_MAGENTA, COLOR_CYAN, COLOR_YELLOW, COLOR_BLACK, COLOR_GREY};

#pragma once

#include <OptimizationLib/ObjectiveFunction.h>

class TestFunction : public ObjectiveFunction {
public:
	TestFunction() {}
	~TestFunction() {}

	double alpha = 0.05;

	double computeValue(const dVector& m) {
		//Test case 1: Rosenbrock function
		double c1 = 1 - m[0];
		double c2 = m[1] - 0.5 * m[0] * m[0];
		return alpha * (c1 * c1 + 10 * c2 * c2);

		//Test case 2: narrow ellipse
		//		double c1 = 2 - m[0];
		//		double c2 = m[1] - m[0];
		//		return alpha * (c1 * c1 + 10 * c2 * c2);
	}

	/*void addGradientTo(dVector& grad, const dVector& m) {
	resize(grad, m.size());
	dVector dm(2);
	dm[0] = m[0], dm[1] = 0.5 * m[1];
	grad += dm * 2.0 * alpha;
	}

	void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& m) {
	ADD_HES_ELEMENT(hessianEntries, 0, 0, 2.0, alpha);
	ADD_HES_ELEMENT(hessianEntries, 1, 1, 1.0, alpha);
	}*/
};

//Beichen Li: class definition for color area
class ColorArea {
public:
	ColorArea(int x, int y1, int y2, int colorDeg) : ix(x), iyStart(y1), iyEnd(y2), colorDeg(colorDeg) {}

	int ix;
	int iyStart;
	int iyEnd;
	int colorDeg;
};

/**
 * Test App for shaders
 */
class OptimizationDemoApp : public GLApplication {
private:

	//Beichen Li: scene drawing parameters
	double planeWidth = 5.0;
	int resolution = 1000;
	double isolevelInterval = 0.5;
	
	//Beichen Li: maximum and minimum isolevel degrees
	int minDeg, maxDeg;

	//Beichen Li: objective function we are investigating
	ObjectiveFunction *function = NULL;

	//Beichen Li: cache array for contour drawing acceleration
	int *cache = NULL;

	//Beichen Li: isolevel points
	DynamicArray<IsolevelPoint> isolevelPoints;

	//Beichen Li: color areas
	DynamicArray<ColorArea> colorAreas;

	//Beichen Li: the following are paths (the points reached) of different optimization methods
	DynamicArray<P3D> pathGradient;
	DynamicArray<P3D> pathNewton;
	DynamicArray<P3D> pathGradientMomentum;
	DynamicArray<P3D> pathLBFGS;

	//Beichen Li: energy values of four algorithms
	double energyGradient = 0.0;
	double energyNewton = 0.0;
	double energyGradientMomentum = 0.0;
	double energyLBFGS = 0.0;

	//Beichen Li: LBFGS parameters and solver
	LBFGSpp::LBFGSParam<double> LBFGSParam;
	LBFGSpp::LBFGSSolver<double> LBFGSSolver;

	MomentumBasedGradientFunctionMinimizer minimizerGM;


	//Beichen Li: numIterations controls the number of iterations per step
	int numIterations = 1;

	//Beichen Li: counter of iteration process
	int iter = 0;

	//Beichen Li: enable screen log
	bool logFlag = true;

	//Beichen Li: control panel
	bool enableContours = true;
	bool enableContourColors = true;
	bool enableAxes = true;
	bool enableGradient = true;
	bool enableNewton = true;
	bool enableGradientMomentum = true;
	bool enableLBFGS = true;
	bool enableHints = true;
	Color colorGradient = COLOR_RED;
	Color colorNewton = COLOR_GREEN;
	Color colorGradientMomentum = COLOR_BLUE;
	Color colorLBFGS = COLOR_MAGENTA;


	//Beichen Li: control parameters for Gradient Momentum
	bool enableLineSearch = true, lastEnableLineSearch = true;
	double betaMomentum = 0.5, lastBetaMomentum = 0.5;
	double initialStep = 1.0, lastInitialStep = 1.0;

public:
	// constructor
	OptimizationDemoApp();
	// destructor
	virtual ~OptimizationDemoApp(void);
	// Run the App tasks
	virtual void process();
	// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
	virtual void drawScene();
	// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
	virtual void drawAuxiliarySceneInfo();
	// Restart the application.
	virtual void restart();
	//Beichen Li: take one step back
	void resume();

	//input callbacks...

	//all these methods should returns true if the event is processed, false otherwise...
	//any time a physical key is pressed, this event will trigger. Useful for reading off special keys...
	virtual bool onKeyEvent(int key, int action, int mods);
	//this one gets triggered on UNICODE characters only...
	virtual bool onCharacterPressedEvent(int key, int mods);
	//triggered when mouse buttons are pressed
	virtual bool onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos);
	//triggered when mouse moves
	virtual bool onMouseMoveEvent(double xPos, double yPos);
	//triggered when using the mouse wheel
	virtual bool onMouseWheelScrollEvent(double xOffset, double yOffset);

	virtual bool processCommandLine(const std::string& cmdLine);
	
	virtual void saveFile(const char* fName);
	virtual void loadFile(const char* fName);

	//Beichen Li: draw the contours (isolevel curves)
	void drawContours();

	//Beichen Li: draw the optimization paths for different methods
	void drawPaths();

	//Beichen Li: draw the color areas (where the function value is at a constant level)
	void drawColorAreas();

	//Beichen Li: draw a line strip for optimization path
	void drawOptimizationPath(DynamicArray<P3D>& path);

	//Beichen Li: initialize isolevel points
	void initContours();

	//Beichen Li: optimization task initialization, including setting starting point etc.
	void initOptimization();

	//Beichen Li: restart the optimization process at point origin
	void resetOptimization(const P3D& origin);

	double operator () (const dVector& x, dVector& grad) {
		resize(grad, x.size());
		function->addGradientTo(grad, x);
		return function->computeValue(x);
	}

	//Beichen Li: converts 3D point to 2D vector (x and y only)
	dVector point2Vector(const P3D& point) {
		dVector v;
		resize(v, 2);
		v[0] = point.x(), v[1] = point.y();
		return v;
	}

	void logStatus() {
		Logger::consolePrint("--------\n");
		Logger::consolePrint("Iteration %d:\n", iter);
		Logger::consolePrint("    Gradient     %.6lf\n", energyGradient);
		Logger::consolePrint("    Newton       %.6lf\n", energyNewton);
		Logger::consolePrint("    Gradient M.  %.6lf\n", energyGradientMomentum);
		Logger::consolePrint("    LBFGS        %.6lf\n", energyLBFGS);
	}

	void switchColor(int color) {
		switch (color) {
		case COLOR_RED:
			glColor3d(1.0, 0.0, 0.0); break;
		case COLOR_GREEN:
			glColor3d(0.0, 1.0, 0.0); break;
		case COLOR_BLUE:
			glColor3d(0.0, 0.0, 1.0); break;
		case COLOR_MAGENTA:
			glColor3d(1.0, 0.0, 1.0); break;
		case COLOR_CYAN:
			glColor3d(0.0, 1.0, 1.0); break;
		case COLOR_YELLOW:
			glColor3d(1.0, 1.0, 0.0); break;
		case COLOR_BLACK:
			glColor3d(0.0, 0.0, 0.0); break;
		case COLOR_GREY:
			glColor3d(0.5, 0.5, 0.5); break;
		}
	}
};



