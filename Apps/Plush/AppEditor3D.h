#pragma once 

#include <PlushHelpers/helpers_star.h>

#include <GUILib/GLUtils.h>
#include <GUILib/GLMesh.h>
#include <GUILib/GLContentManager.h>
#include <GUILib/GLTrackingCamera.h>

#include "PlushApplication.h"
#include "PlateMesh.h"
#include "CSTSimulationMesh3D.h"
 
class AppEditor3D : public PlushApplication {

public:
	AppEditor3D();
	~AppEditor3D(void) {}
	virtual void drawScene();

private:
	PlateMesh *design = nullptr;
	Plane *drag_plane = new Plane(P3D(0., 0., .5), V3D(0., 0., 1.));
	// CSTSimulationMesh3D *plushie = nullptr;
	const string DESIGN_PATH_ = "../Apps/Plush/data/loop/sugar";
	const string PLUSHIE_PATH = "../Apps/Plush/data/tri/sugar"; 

private:
	enum View { Designer, Checker, Simulator };
	View view = Designer; 

private:
	bool DRAW_AXES = true;

private:
	bool LOAD_DESIGN = false;
	bool SAVE_DESIGN = false;
	bool LOAD_PLUSHIE = false;
	bool EXPORT_PLUSHIE = false;


	void loadDesign(const char* fName);

	virtual void process();
	// Wild west.
	virtual void drawAuxiliarySceneInfo();


	// Callbacks. 
	virtual bool onKeyEvent(int key, int action, int mods);
	virtual bool onCharacterPressedEvent(int key, int mods);
	virtual bool onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos);
	virtual bool onMouseMoveEvent(double xPos, double yPos);
	virtual bool onMouseWheelScrollEvent(double xOffset, double yOffset);

	virtual bool processCommandLine(const std::string& cmdLine);
	
	virtual void saveFile(const char* fName);
	virtual void loadFile(const char* fName);

};



