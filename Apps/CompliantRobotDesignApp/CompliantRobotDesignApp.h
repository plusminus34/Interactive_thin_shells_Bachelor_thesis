#pragma once


#include <GUILib/GLApplication.h>
#include <string>
#include <map>
#include <GUILib/TranslateWidget.h>
#include <GUILib/GLWindow3D.h>
#include <GUILib/GLWindow3DWithMesh.h>
#include <GUILib/GLWindowContainer.h>
#include "DesignWindow.h"
#include "SimulationWindow.h"

class IntelligentRobotEditingWindow;
class EnergyWindow;

/**
 * Robot Design and Simulation interface
 */
class CompliantRobotDesignApp : public GLApplication {
public:
	double globalTime = 0;
	double controlFrequency = 1 / 120.0;

	Robot* robot = NULL;
	RobotState startingRobotState;

	DesignWindow* designWindow = NULL;
	SimulationWindow* simWindow = NULL;

	bool shouldShowSimWindow();
	bool shouldShowDesignWindow();

	P3D getCameraTarget();

	void setupWindows();


	enum VIEW_OPTIONS {
		SIM_WINDOW_ONLY = 0,
		DESIGN_WINDOW_ONLY,
		SIM_AND_DESIGN_WINDOWS
	};
	VIEW_OPTIONS viewOptions = SIM_AND_DESIGN_WINDOWS;


public:
	// constructor
	CompliantRobotDesignApp();
	// destructor
	virtual ~CompliantRobotDesignApp(void);
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




