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
#include <RobotDesignerLib/MOPTWindow.h>
#include <RobotDesignerLib/SimWindow.h>



/**
 * Robot Design and Simulation interface
 */
class RobotDesignerApp : public GLApplication {
public:
	ModularDesignWindow *designWindow = NULL;
	MOPTWindow* moptWindow = NULL;

	SimWindow* simWindow = NULL;

	Robot* robot = NULL;
	ReducedRobotState* initialRobotState = NULL;

	bool drawMotionPlan = false;

	bool drawControllerDebugInfo = false;
	bool doubleCheckControllerSolution = false;

	P3D getCameraTarget();

	void setupWindows();

	enum RD_RUN_OPTIONS {
		MOTION_PLAN_OPTIMIZATION = 0,
		MOTION_PLAN_ANIMATION,
		PHYSICS_SIMULATION_WITH_POSITION_CONTROL,
		PHYSICS_SIMULATION_WITH_TORQUE_CONTROL
	};
	RD_RUN_OPTIONS runOption = MOTION_PLAN_OPTIMIZATION;

	enum RD_VIEW_OPTIONS {
		SIM_WINDOW_ONLY = 0,
		SIM_AND_MOPT,
		SIM_AND_DESIGN
	};
	RD_VIEW_OPTIONS viewOptions = SIM_AND_MOPT;

	bool doDebug = false;

public:
	// constructor
	RobotDesignerApp();
	// destructor
	virtual ~RobotDesignerApp(void);
	// Run the App tasks
	virtual void process();
	// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
	virtual void drawScene();
	// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
	virtual void drawAuxiliarySceneInfo();
	// Restart the application.
	virtual void restart();

	void setActiveController();

	void runMOPTStep();

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

	void warmStartMOPT(bool initializeMotionPlan);
	void loadToSim(bool initializeMOPT = true);
	void createRobotFromCurrentDesign();


	ParameterizedRobotDesign* prd;
	void test_dmdp_Jacobian();
	void compute_dmdp_Jacobian(dVector& m, DynamicArray<double>& p, MatrixNxM& dmdp);
	void testOptimizeDesign();
	void addDesignParameterSliders();

	void resyncRBS();

private:
	void addWheelParameterUI();
};




