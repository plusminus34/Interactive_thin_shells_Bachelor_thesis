#pragma once 
// --
#include <PlushHelpers/helpers_star.h>
// --
#include "PlushApplication.h"
#include "SoftLocoSolver.h"
#include "CSTSimulationMesh2D.h"
#include "CSTSimulationMesh3D.h"
#include "P2DDragger.h"
#include "P2DDragger_v2.h"
#include "Poser.h"
#include "Handler_v2.h"
#include "CubicHermiteSpline_v2.h"

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
	vector<vector<P3D*>> test_splines;
	Frame test_frame = Frame(Matrix3x3::Identity(), V3D(-1., 1.));
	Handler_v2 test_handler;
	// TODO: test out use of Handler_v2
	// TODO: Consider exposing translation and scaling as widgets

public:
	bool ENABLE_SPLINE_INTERACTION = true;

public:
	bool PLAY_PREVIEW = false;
	bool POPULATED_PREVIEW_TRAJEC = false;
	const int LEADIN_FRAMES = 25;
	const int LEADOUT_FRAMES = 250;
	int PREVIEW_i = -LEADIN_FRAMES;
	vector<dVector> uJ_preview;
	vector<dVector> xJ_preview;

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