#pragma once 

#include <PlushHelpers/helpers_star.h>

#include <GUILib/GLUtils.h>
#include <GUILib/GLMesh.h>
#include <GUILib/GLContentManager.h>
#include <GUILib/GLTrackingCamera.h>

#include "PlushApplication.h"
#include "CSTSimulationMesh2D.h"
#include "LoopMesh.h"
 
class AppEditor2D : public PlushApplication {

private:
	LoopMesh *mesh;
	CSTSimulationMesh2D *preview = nullptr;

private:
	bool DRAW_AXES = true;

private:
	bool LOAD_LOOP = false;
	bool DUMP_LOOP = false;
	bool LOAD_PLUSHIE = false;
	bool DUMP_PLUSHIE = false;
	bool SYMMETRIZE = false;
	bool SOLVE = false;
	bool CLEAR_ERRORS_AND_WARNINGS = false;
	

public:
	AppEditor2D();
	virtual ~AppEditor2D(void);

	void load_sugar(const char* fName);

	virtual void process();
	virtual void handle_toggles();
	virtual void step();
	virtual void drawScene();
	// Wild west.
	virtual void drawAuxiliarySceneInfo();
	virtual void restart();


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



