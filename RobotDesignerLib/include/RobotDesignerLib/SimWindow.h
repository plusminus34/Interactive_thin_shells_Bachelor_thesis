#pragma once

#include <GUILib/GLApplication.h>
#include <GUILib/GLWindow3D.h>
#include <GUILib/GLTrackingCamera.h>
#include <RobotDesignerLib/LocomotionEngineManager.h>
#include <RBSimLib/AbstractRBEngine.h>
#include <RBSimLib/WorldOracle.h>
#include <RobotDesignerLib/PositionBasedRobotController.h>
#include <RobotDesignerLib/TorqueBasedRobotController.h>
#include <RobotDesignerLib/KinematicRobotController.h>
#include <RobotDesignerLib/PololuMaestroRobotController.h>



class SimWindow : public GLWindow3D {
public:
	GLApplication* glApp;

	Robot* robot = NULL;

	bool drawMeshes = true, drawMOIs = false, drawCDPs = false, drawSkeletonView = true, drawContactForces = true, drawOrientation = true;

	void addMenuItems();

	AbstractRBEngine* rbEngine = NULL;
	WorldOracle* worldOracle = NULL;

	double simTimeStep;
	int nPhysicsStepsPerControlStep = 4;

	PositionBasedRobotController* positionController = NULL;
	TorqueBasedRobotController* torqueController = NULL;
	KinematicRobotController* kinematicController = NULL;
	PololuMaestroRobotController* pololuMaestroController = NULL;

	RobotController* activeController = NULL;

	void setActiveController(RobotController* con) { activeController = con; }
	RobotController* getActiveController() { return activeController; }

	V3D perturbationForce;
	double forceScale = 0;

	void setPerturbationForceFromMouseInput(double xPos, double yPos);

	void doPhysicsStep(double simStep);

public:
	SimWindow(int x, int y, int w, int h, GLApplication* glApp);
	~SimWindow();

	void clear();
	Robot* loadRobot(const char*);
	void loadMotionPlan(LocomotionEngineMotionPlan* mp);

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

