#pragma once

#include <GUILib/GLApplication.h>
#include <string>
#include <map>
#include <GUILib/TranslateWidget.h>
#include <RBSimLib/AbstractRBEngine.h>
#include <RBSimLib/WorldOracle.h>
#include <GUILib/GLWindow3D.h>
#include <GUILib/GLWindow3DWithMesh.h>
#include <GUILib/GLWindowContainer.h>
#include <ControlLib/IK_Solver.h>
#include <ControlLib/RobotControlInterface.h>

using namespace	std;



/**
* Robot Design and Simulation interface
*/
class PhysicalRobotControlApp : public GLApplication {
private:
	Robot* robot = NULL;
	AbstractRBEngine* rbEngine = NULL;
	RobotState startState = RobotState(14);
	bool showMesh = false;
	bool showMOI = false;
	bool showRotationAxes = false;
	bool showCDPs = false;

    bool playFFTrajectory = false;
	double trajPhase = 0.0;
	double trajDuration = 1.0;

	bool controlPositionsOnly = false;
    bool syncPhysicalRobot = false;
	bool requestPosition = false;

	bool startAtHomeState = true; //true = uses predefined home position / false = uses position where robot is currently in as home position
	bool saveCurrentAsHomePosition = false;
	std::string homeFilePath = "/scp/data/rbs/yumi/yumiHomeState.rs";

	unsigned int speed = 100;

	RigidBody* selectedRigidBody = NULL;
	RigidBody* highlightedRigidBody = NULL;
	P3D selectedPoint;	//in local coordinates of selected rigid body
	P3D targetPoint;	//in world coordinates

	IK_Solver* ikSolver = NULL;
	RobotControlInterface* rci = NULL;

	Trajectory1D FFTrajectory;

public:

	// constructor
	PhysicalRobotControlApp();
	// destructor
	virtual ~PhysicalRobotControlApp(void);
	// Run the App tasks
	virtual void process();
	// Restart the application.
	virtual void restart();

	// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
	virtual void drawScene();
	// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
	virtual void drawAuxiliarySceneInfo();

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

	void loadRobot(const char* fName);
	virtual void loadFile(const char* fName);
	virtual void saveFile(const char* fName);

	void updateSpeedParameter();

};
