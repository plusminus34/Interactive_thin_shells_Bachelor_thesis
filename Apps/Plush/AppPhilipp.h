#pragma once

#include "PlushApplication.h"
#include "CSTSimulationMesh2D.h"
#include "CSTSimulationMesh3D.h"
#include "QueuePlot.h"

class AppPhilipp : public PlushApplication {

public: 
	AppPhilipp();
	inline virtual ~AppPhilipp(void) {}
	inline virtual void restart() {}
	virtual void drawScene();

public:
    CSTSimulationMesh2D *mesh;
	QueuePlot *plot;

public:
	// std::vector (C++ version of a C array)
	vector<double> u_vec;
	vector<double> t_vec;
	const int NUM_KEYFRAMES = 10;
	P3D xy0 = P3D();

public:
    bool TEST = false;
public:
	inline virtual void drawAuxiliarySceneInfo() {}
	virtual void process();

	virtual bool onKeyEvent(int key, int action, int mods);
	virtual bool onCharacterPressedEvent(int key, int mods);
	virtual bool onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos);
	virtual bool onMouseMoveEvent(double xPos, double yPos);
	virtual bool onMouseWheelScrollEvent(double xOffset, double yOffset);
	virtual bool processCommandLine(const std::string& cmdLine);
	
	virtual void saveFile(const char* fName);
	virtual void loadFile(const char* fName);

};



