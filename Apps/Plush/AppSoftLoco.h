#pragma once 
// --
#include <PlushHelpers/helpers_star.h>
// --
#include "PlushApplication.h"
#include "SoftLocoSolver.h"
#include "CSTSimulationMesh2D.h"
#include "CSTSimulationMesh3D.h"
#include "P2DDragger.h"
#include "Poser.h"

class AppSoftLoco : public PlushApplication {

public:
    AppSoftLoco();
	inline virtual ~AppSoftLoco() {};

public:
    SimulationMesh *mesh;
	SoftLocoSolver *ik;

public:
	// SimulationMesh *Zmesh;
	// SoftIKSolver   *Zik;

public:
	vector<P2DDragger *> COM_handlers;

public:
	bool PLAY_PREVIEW = false;
	bool POPULATED_PREVIEW_TRAJEC = false;
	const int LEADIN_FRAMES = 30;
	int PREVIEW_i = -LEADIN_FRAMES;
	vector<dVector> uJ_preview;
	vector<dVector> xJ_preview;

public:
	bool CAPTURE_TEST_SESSION = false;
	bool CAPTURED_TEST_SESSION_ = false;
	// --
	bool PLAY_CAPTURE = false;
	bool POPULATED_CAPTURE_TRAJEC = false;
	int CAPTURE_i = -30;
	vector<dVector> uJ_capture;
	vector<dVector> xJ_capture;
	// -- 
	dVector xm1_capture;
	dVector vm1_capture;

public:
	bool SOLVE_IK = true;
	bool INTEGRATE_FORWARD_IN_TIME = true;

public:
    virtual void process();
    virtual void drawScene();

public:
	virtual void processToggles();

public:
    virtual bool onKeyEvent(int key, int action, int mods);
    virtual bool onCharacterPressedEvent(int key, int mods);
    virtual bool onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos);
    virtual bool onMouseMoveEvent(double xPos, double yPos);
    virtual bool onMouseWheelScrollEvent(double xOffset, double yOffset);

 
};