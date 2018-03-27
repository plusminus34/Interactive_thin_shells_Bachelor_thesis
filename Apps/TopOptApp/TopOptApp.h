#pragma once

#include <GUILib/GLApplication.h>
#include <string>
#include <map>

#include <FEMSimLib/SimulationMesh.h>
#include <FEMSimLib/CSTSimulationMesh2D.h>
#include <FEMSimLib/CSTElement2D.h>
#include <OptimizationLib/ConstrainedObjectiveFunction.h>
#include "TopOptConstraints.h"
#include "TopOptEnergyFunction.h"

/**
 * Test App
 */
class TopOptApp : public GLApplication {
private:
	SimulationMesh* simMesh;

	Ray lastClickedRay = Ray(P3D(0, 0, 0), V3D(0, 0, 1));
	int selectedNodeID = -1;

	P3D startDragPoint, endDragPoint;
	bool dragging = false;

	double forceScale = 10.0;

	MaterialModel2D matModel = MM_NEO_HOOKEAN;
	double shearModulus = 50, bulkModulus = 50;
	bool checkDerivatives = false;

	DynamicArray<V3D> externalLoads;
	dVector densityParams;

	double targetMassRatio = 100;
	double initialMass = 0;
	bool optimizeTopology = false;


	void applyDensityParametersToSimMesh();

	TopOptEnergyFunction* energyFunction;
	TopOptConstraints* constraints;
	ConstrainedObjectiveFunction* constrainedObjectiveFunction;


public:
	// constructor
	TopOptApp();
	// destructor
	virtual ~TopOptApp(void);
	// Run the App tasks
	virtual void process();
	// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
	virtual void drawScene();
	// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
	virtual void drawAuxiliarySceneInfo();
	// Restart the application.
	virtual void restart();

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

};



