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
#include <RobotDesignerLib/MotionPlanAnalysis.h>
#include <RobotDesignerLib/EnergyWindow.h>

#ifdef USE_MATLAB
	#define RUN_IN_MATLAB(x) x
	#include <igl/matlab/matlabinterface.h>
#else
//	RUN_IN_MATLAB(x)
#endif

#define START_WITH_VISUAL_DESIGNER

class IntelligentRobotEditingWindow;
class EnergyWindow;
/**
 * Robot Design and Simulation interface
 */
class RobotDesignerApp : public GLApplication {
public:
	ModularDesignWindow *designWindow = NULL;
	MOPTWindow* moptWindow = NULL;
	SimWindow* simWindow = NULL;
	IntelligentRobotEditingWindow* iEditWindow = NULL;
	MotionPlanAnalysis *motionPlanAnalysis = nullptr;
	EnergyWindow *energyWindow = nullptr;

	bool doMotionAnalysis = true;

	bool shouldShowSimWindow();
	bool shouldShowMOPTWindow();
	bool shouldShowIEditWindow();
	bool shouldShowDesignWindow();


	Robot* robot = NULL;
	RobotState startingRobotState = RobotState(13);

	bool drawMotionPlan = false;

	bool drawControllerDebugInfo = false;
	bool doubleCheckControllerSolution = false;

	P3D getCameraTarget();

	void setupWindows();

	enum RD_RUN_OPTIONS {
		MOTION_PLAN_OPTIMIZATION = 0,
		MOTION_PLAN_ANIMATION,
		PHYSICS_SIMULATION_WITH_POSITION_CONTROL,
		PHYSICS_SIMULATION_WITH_TORQUE_CONTROL,
		PHYSICAL_ROBOT_CONTROL_VIA_POLOLU_MAESTRO
	};
	RD_RUN_OPTIONS runOption = MOTION_PLAN_OPTIMIZATION;

	enum RD_VIEW_OPTIONS {
		SIM_WINDOW_ONLY = 0,
		SIM_AND_MOPT,
		SIM_AND_DESIGN,
		MOPT_AND_IEDIT
	};
#ifdef START_WITH_VISUAL_DESIGNER
	RD_VIEW_OPTIONS viewOptions = SIM_AND_DESIGN;
#else //  START_WITH_VISUAL_DESIGNER
	RD_VIEW_OPTIONS viewOptions = SIM_AND_MOPT;// MOPT_AND_IEDIT;// SIM_AND_DESIGN;
#endif
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

	void exportMeshes();

	SymmetricParameterizedRobotDesign* prd = NULL;

#ifdef USE_MATLAB
	Engine *matlabengine;
#endif

	MatrixNxM dmdp; //The jacobian at a point
	dVector m0;
	bool useSVD = false;
	MatrixNxM dmdp_V;
	dVector p0;
	dVector slidervalues;
	bool updateMotionBasedOnJacobian = false;
	bool optimizeWhileAnimating = false;
	bool syncCameras = false;
};




