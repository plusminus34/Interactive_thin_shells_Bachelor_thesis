#pragma once

#include <GUILib/GLApplication.h>
#include <GUILib/GLWindow3D.h>
#include <GUILib/GLTrackingCamera.h>
#include <KineSimLib\KS_MechanicalAssembly.h>
#include <KineSimLib\KS_UIMechanismController.h>


class SimWindow : public GLWindow3D {
public:
	GLApplication* glApp;

	KS_MechanicalAssembly* mechanism = NULL;

	bool drawMeshes = true;

	void addMenuItems();

	double simTimeStep = 1;

	KS_UIMechanismController* UIController = NULL;
	

	KS_MechanismController* activeController = NULL;

	void setActiveController(KS_MechanismController* con) { activeController = con; }
	KS_MechanismController* getActiveController() { return activeController; }


	void doPhysicsStep(double simStep);

public:
	SimWindow(int x, int y, int w, int h, GLApplication* glApp);
	~SimWindow();

	void clear();
	KS_MechanicalAssembly* loadMechanism(const char*);
	void loadController();

	//returns true if the motion plan finished/motion phase was reset, false otherwise...
	bool advanceSimulation(double dt);

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

