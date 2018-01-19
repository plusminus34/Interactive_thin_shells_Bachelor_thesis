#pragma once

#include <GUILib/GLApplication.h>
#include <GUILib/GLWindow3D.h>
#include <GUILib/GLTrackingCamera.h>
#include <RobotDesignerLib/LocomotionEngineManager.h>
#include <GUILib/TranslateWidget.h>
#include <memory>

struct MOPTParams {
	double phase = 0;

	int gaitCycle = 0;

	bool drawRobotPose = false;
	bool drawPlanDetails = true;
	bool drawContactForces = true;
	bool drawOrientation = true;

	double swingFootHeight = 0.02;
	double desTravelDistX = 0;
	double desTravelDistZ = 0;
	double desTurningAngle = 0;

	double jointVelocityLimit = 10;
	double jointVelocityEpsilon = 0.4;
	double jointAngleLimit = PI/4;
	double EEminDistance = 0.02;

	double jointL0Delta = 1;

	double wheelSpeedLimit = 20;
	double wheelSpeedEpsilon = 0.4;

	double wheelAccelLimit = 20;
	double wheelAccelEpsilon = 1.0;

	double frictionCoeff = 0.5;

	bool writeJointVelocityProfile = false;
	double motionPlanDuration = 0.8;
	bool checkDerivatives = false;
	bool useDynamicRegularization = true;
	NewtonFunctionMinimizer::HessCorrectionMethod hessCorrectionMethod = NewtonFunctionMinimizer::DynamicRegularization;
	bool checkHessianPSD = false;
	double externalForceX=0;
	double externalForceZ=0;
};


class MOPTWindow : public GLWindow3D {
public:
	bool initialized = false;
	GLApplication* glApp;

	int nTimeSteps = 12;
	double globalMOPTRegularizer = 0.01;

	nanogui::Graph* energyGraph = NULL;
	std::vector<float> energyGraphValues;

	nanogui::Graph* velocityProfileGraph;

	MOPTParams moptParams;

	Robot* robot = nullptr;

	FootFallPattern footFallPattern;
	FootFallPatternViewer* ffpViewer = nullptr;
	LocomotionEngineManager* locomotionManager = nullptr;

	enum OPT_OPTIONS {
		GRF_OPT = 0,
		GRF_OPT_V2,
		IP_OPT,
		IP_OPT_V2
	};
	OPT_OPTIONS optimizeOption = GRF_OPT_V2;

	bool printDebugInfo;
	void addMenuItems();

	bool periodicMotion = true;

	std::list<shared_ptr<TranslateWidget>> EEwidgets;
	std::list<shared_ptr<TranslateWidget>> COMWidgets;
public:
	MOPTWindow(int x, int y, int w, int h, GLApplication* glApp);
	~MOPTWindow();

	void clear();
	void loadRobot(Robot* robot);
	void syncMotionPlanParameters();
	void syncMOPTWindowParameters();

	LocomotionEngineManager* initializeNewMP(bool doWarmStart = true);

	double runMOPTStep();

	void printCurrentObjectiveValues() {
		locomotionManager->setDefaultOptimizationFlags();
		dVector params = locomotionManager->motionPlan->getMPParameters();
		locomotionManager->energyFunction->setCurrentBestSolution(params);
		Logger::consolePrint("Current MOPT objective function: %lf\n", locomotionManager->energyFunction->computeValue(params));
	}

	void reset();

	void setAnimationParams(double f, int animationCycle);

	void loadFFPFromFile(const char* fName);

	virtual void drawScene();
	virtual void drawAuxiliarySceneInfo();

	virtual bool onMouseMoveEvent(double xPos, double yPos);

	void updateJointVelocityProfileWindowOnMouseMove(Ray &ray, double xPos, double yPos);

	virtual bool onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos);

	//any time a physical key is pressed, this event will trigger. Useful for reading off special keys...
	virtual bool onKeyEvent(int key, int action, int mods);

	virtual void setViewportParameters(int posX, int posY, int sizeX, int sizeY);

private:

	V3D COMSpeed;
	nanogui::Window* velocityProfileWindow=nullptr;
	int endEffectorInd = -1;
	map<shared_ptr<TranslateWidget>, shared_ptr<EndEffectorPositionObjective>> EEwidget2constraint;
	map<shared_ptr<TranslateWidget>, shared_ptr<COMPositionObjective>> COMwidget2constraint;
};
