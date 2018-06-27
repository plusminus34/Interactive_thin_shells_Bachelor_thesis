#pragma once

#include <GUILib/GLApplication.h>
#include <GUILib/GLWindow3D.h>
#include <GUILib/GLTrackingCamera.h>
#include <RobotDesignerLib/LocomotionEngineManager.h>
#include <GUILib/TranslateWidget.h>
#include <GUILib/CompositeWidget.h>
#include <RobotDesignerLib/BaseRobotControlApp.h>
#include <RobotDesignerLib/MoptWindow.h>

#include <memory>

class RobotDesignerApp;
class FastMOPTPreplanner;

class FastMOPTWindow : public GLWindow3D {
public:
	//these are the global goals for the longer horizon plan...
	double preplanTimeHorizon = 5;	//seconds
	double forwardSpeedTarget;		//speed target for the longer horizon plan
	double sidewaysSpeedTarget;		//speed target for the longer horizon plan
	double turningSpeedTarget;		//turning speed target for the longer horizon plan
	double bodyHeightTarget;		//body height target for the longer horizon plan
	double motionPlanStartTime = 0;	//the global time for the entire planning/control framework

	FootFallPattern defaultFootFallPattern; //TODO: at some point we can also change the footfall pattern to make transitions, stand-to-walk-to-stand, etc...

	FastMOPTPreplanner* fmpp;

	bool initialized = false;
	BaseRobotControlApp* theApp;

	int nTimeSteps = 12;
	double globalMOPTRegularizer = 0.01;

	nanogui::Graph* energyGraph = NULL;
	std::vector<float> energyGraphValues;

	nanogui::Graph* velocityProfileGraph;

	MOPTParams moptParams;

	Robot* robot = nullptr;

	FootFallPattern footFallPattern;
	FootFallPatternViewer* ffpViewer = nullptr;
	bool showFFPViewer = true;
	LocomotionEngineManager* locomotionManager = nullptr;

	enum OPT_OPTIONS {
		GRF_OPT = 0,
		GRF_OPT_V2,
		GRF_OPT_V3,
		IP_OPT,
		IP_OPT_V2
	};

	OPT_OPTIONS optimizeOption = GRF_OPT_V2;
	

	bool printDebugInfo;
	void addMenuItems();

	std::list<shared_ptr<TranslateWidget>> EEwidgets;
	std::list<shared_ptr<CompositeWidget>> COMWidgets;
public:
	FastMOPTWindow(int x, int y, int w, int h, BaseRobotControlApp* glApp);
	~FastMOPTWindow();

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

	void generateMotionPreplan();

	void optimizeMotionPlan();

	void advanceMotionPlanGlobalTime(int nSteps);

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
	virtual void drawGround() {
		drawTexturedGround(GLContentManager::getTexture("../data/textures/lightGray.bmp"));
	}
private:

	V3D COMSpeed;
	nanogui::Window* velocityProfileWindow = nullptr;
	int endEffectorInd = -1;
	map<shared_ptr<TranslateWidget>, shared_ptr<EndEffectorPositionObjective>> EEwidget2constraint;
	map<shared_ptr<CompositeWidget>, shared_ptr<BodyFrameObjective>> COMwidget2constraint;
};
