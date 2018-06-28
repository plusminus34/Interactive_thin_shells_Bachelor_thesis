#pragma once


#include <GUILib/GLApplication.h>
#include <string>
#include <map>
#include <GUILib/TranslateWidget.h>
#include <GUILib/GLWindow3D.h>
#include <GUILib/GLWindow3DWithMesh.h>
#include <GUILib/GLWindowContainer.h>
#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>
#include <RobotDesignerLib/LocomotionEngine.h>
#include <RobotDesignerLib/FootFallPatternViewer.h>
#include <RobotDesignerLib/LocomotionEngineManagerGRF.h>
#include <RobotDesignerLib/LocomotionEngineManagerIP.h>
#include <RobotDesignerLib/ModularDesignWindow.h>
#include <RobotDesignerLib/ParameterizedRobotDesign.h>
#include <RobotDesignerLib/FastMOPTWindow.h>
#include <RobotDesignerLib/SimWindow.h>
#include <RobotDesignerLib/MotionPlanAnalysis.h>
#include <RobotDesignerLib/EnergyWindow.h>
#include <RobotDesignerLib/BaseRobotControlApp.h>

class IntelligentRobotEditingWindow;
class EnergyWindow;

/**
 * Robot Design and Simulation interface
 */
class FastRobotControlApp : public BaseRobotControlApp {
public:
	double time = 0;
	double phase = 0;

	FastMOPTWindow* plannerWindow = NULL;
	SimWindow* simWindow = NULL;

	bool shouldShowSimWindow();
	bool shouldShowPlannerWindow();

	P3D getCameraTarget();

	void setupWindows();

	enum RD_RUN_OPTIONS {
		MOTION_PLAN_OPTIMIZATION = 0,
		MOTION_PLAN_ANIMATION,
		PHYSICS_SIMULATION_WITH_POSITION_CONTROL,
		PHYSICS_SIMULATION_WITH_TORQUE_CONTROL,
	};
	RD_RUN_OPTIONS runOption = MOTION_PLAN_OPTIMIZATION;

	enum VIEW_OPTIONS {
		SIM_WINDOW_ONLY = 0,
		MOPT_WINDOW_ONLY,
		SIM_AND_MOPT_WINDOWS
	};
	VIEW_OPTIONS viewOptions = MOPT_WINDOW_ONLY;

	int walkCycleIndex = 0;

public:
	// constructor
	FastRobotControlApp();
	// destructor
	virtual ~FastRobotControlApp(void);
	// Run the App tasks
	virtual void process();
	// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
	virtual void drawScene();
	// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
	virtual void drawAuxiliarySceneInfo();
	// Restart the application.
	virtual void restart();

	void setActiveController();

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

	void loadToSim();
};




