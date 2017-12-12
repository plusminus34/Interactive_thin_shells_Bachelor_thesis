#pragma once

#include <GUILib/GLApplication.h>
#include <string>
#include <array>
#include <map>

#include "BenderSimulationMesh2D.h"
//#include "Mount.h"


/**
 * Test App
 */
class BenderApp : public GLApplication {
private:
	BenderSimulationMesh2D* femMesh;
	
	dVector xi;

	dVector dOdxi;
	dVector dOdx;
	std::vector<dVector> deltaFdeltaxi;
	std::vector<dVector> deltaxdeltaxi;

	int selected_mount = -1;


	// state of the app
	bool computeStaticSolution = true;
	bool optimizeObjective = true;

	bool checkDerivatives = false;

	double simTimeStep = 1/100.0;

	Ray lastClickedRay = Ray(P3D(0, 0, 0), V3D(0, 0, 1));
	Ray lastMovedRay = Ray(P3D(0, 0, 0), V3D(0, 0, 1));
	Ray currentRay = Ray(P3D(0, 0, 0), V3D(0, 0, 1));
	int selectedNodeID = -1;
	double shearModulus = 50, bulkModulus = 50;
	bool autoUpdateShearModulusAndBulkModulus = false;

public:
	// constructor
	BenderApp();
	// destructor
	virtual ~BenderApp(void);
	// Run the App tasks
	virtual void process();
	// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
	virtual void drawScene();
	// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
	virtual void drawAuxiliarySceneInfo();
	// Restart the application.
	virtual void restart();

	// add a node to a mount
	void addMountedNode(int node_id, int mount_id);
	//void updateMountEnergy();
	//int getMountId(int node_id);

	// optimization process
	void computeDoDxi(dVector & dodxi);
	double peekOofXi(dVector const & xi_in);

	void pullXi();
	void pushXi();


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



