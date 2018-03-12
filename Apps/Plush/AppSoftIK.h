#pragma once 
// --
#include <PlushHelpers/helpers_star.h>
// --
#include "PlushApplication.h"
#include "SoftIKSolver.h"
#include "Inspector.h"
#include "CSTSimulationMesh2D.h"
#include "CSTSimulationMesh3D.h"

class AppSoftIK : public PlushApplication {

public:
    AppSoftIK();
	inline virtual ~AppSoftIK() {};

public:
    SimulationMesh *mesh;
	SoftIKSolver *ik;
	Inspector *inspector;

public:
	bool SOLVE_IK = true;
	bool SOLVE_DYNAMICS = true;
	double timeStep = .01;
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