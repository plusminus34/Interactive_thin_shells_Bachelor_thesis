#pragma once

#include <GUILib/GLApplication.h>
#include <string>
#include <map>

#include <FEMSimLib/SimulationMesh.h>
#include "ShapeWindow.h"
#include "SimulationWindow.h"

enum MouseMode { mouse_none, mouse_drag, mouse_select, mouse_cut, mouse_pin };

/*
  Test App
 */
class Paper3DApp : public GLApplication {
	friend class SimulationWindow;
	friend class ShapeWindow;
private:
	ShapeWindow* shapeWindow = NULL;
	SimulationWindow* simWindow = NULL;

	SimulationMesh* simMesh;

	Ray lastClickedRay = Ray(P3D(0, 0, 0), V3D(0, 0, 1));
	int selectedNodeID = -1;

	P3D startDragPoint, endDragPoint;
	bool dragging = false;

	bool checkDerivatives = false;

	double shearModulus;
	double bulkModulus;
	double bend_k;
	double pin_k;
	MouseMode mouse_mode;

	P3D getNodePos(int i);

public:
	// constructor
	Paper3DApp();
	// destructor
	virtual ~Paper3DApp(void);
	// Run the App tasks
	virtual void process();
	// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
	virtual void drawScene();
	// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
	virtual void drawAuxiliarySceneInfo();
	// Restart the application.
	virtual void restart();

	void setupWindows();

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

	//Update the mesh with parameters from menu
	virtual void updateParams();
};



