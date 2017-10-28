#pragma once
#include <GUILib/GLWindow3D.h>
#include <RBSimLib/AbstractRBEngine.h>
#include <ControlLib/Robot.h>

class AbstractDesignWindow : public GLWindow3D{
public:
	bool freezeRobotRoot = false;

public:
	AbstractDesignWindow(int x, int y, int w, int h) : GLWindow3D(x, y, w, h) {};

	virtual void onMenuMouseButtonProcessedEvent() {}
	virtual ReducedRobotState getStartState(Robot* robot) {
		return ReducedRobotState(robot);
	}

	virtual void setStartStateFName(const char* startStateFName) {
	}

	virtual void prepareForSimulation(AbstractRBEngine* rbEngine) {}
	virtual void loadFile(const char* fName){}
	virtual void saveFile(const char* fName) {}
	virtual void drawAuxiliarySceneInfo() {}
	virtual bool process() { return false; }
};

