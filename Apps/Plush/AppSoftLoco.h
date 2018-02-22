#pragma once 
// --
#include <PlushHelpers/helpers_star.h>
// --
#include "PlushApplication.h"
#include "SoftLocoSolver.h"
#include "CSTSimulationMesh2D.h"
#include "CSTSimulationMesh3D.h"

class AppSoftLoco : public PlushApplication {

public:
    AppSoftLoco();
	inline virtual ~AppSoftLoco() {};

public:
    SimulationMesh *mesh;
	SoftLocoSolver *ik;

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