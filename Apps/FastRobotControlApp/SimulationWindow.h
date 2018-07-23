#pragma once

#include <GUILib/GLApplication.h>
#include <GUILib/GLWindow3D.h>
#include <GUILib/GLTrackingCamera.h>
#include <RobotDesignerLib/LocomotionEngineManager.h>
#include <RBSimLib/AbstractRBEngine.h>
#include <RBSimLib/WorldOracle.h>
#include "PlaybackController.h"
#include "TrackingController.h"


class SimulationWindow : public GLWindow3D {
public:
	GLApplication* glApp;

	Robot* robot = NULL;

	double stridePhase = 0;

	bool drawMeshes = true, drawMOIs = false, drawCDPs = false, drawSkeletonView = true, drawContactForces = true, drawOrientation = true;

	void addMenuItems();

	AbstractRBEngine* rbEngine = NULL;
	WorldOracle* worldOracle = NULL;

	double simTimeStep = 1/120.0;
	int nPhysicsSubsteps = 4;

	TrackingController* trackingController = NULL;
	PlaybackController* playbackController = NULL;

	RobotController* activeController = NULL;

	LocomotionEngineMotionPlan* mp = NULL;

	void setActiveController(RobotController* con) { activeController = con; }
	RobotController* getActiveController() { return activeController; }

	V3D perturbationForce;
	double forceScale = 0;

	void setPerturbationForceFromMouseInput(double xPos, double yPos);

	void doPhysicsStep(double simStep);

public:
	SimulationWindow(int x, int y, int w, int h, GLApplication* glApp);
	~SimulationWindow();

	void clear();
	Robot* loadRobot(const char*);
	void loadMotionPlan(MotionPlanner* mp);

	void advanceSimulation(double dt);

	void reset();

	virtual void drawScene();
	virtual void drawAuxiliarySceneInfo();

	virtual bool onMouseMoveEvent(double xPos, double yPos);
	virtual bool onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos);

	virtual void setViewportParameters(int posX, int posY, int sizeX, int sizeY);

	virtual void drawGround() {
		drawTexturedGround(GLContentManager::getTexture("../data/textures/grid.bmp"));
	}

};

