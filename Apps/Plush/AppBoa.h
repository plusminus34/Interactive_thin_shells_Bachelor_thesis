#pragma once

#include "PlushApplication.h"
#include "QueuePlot.h"
#include "SimpleSerialPort.h"

class AppBoa : public PlushApplication {

public: 
	AppBoa();
	inline virtual ~AppBoa(void) {}
	inline virtual void restart() {}
	virtual void drawScene();

public:
	bool   connect2Arduino();
	bool   connect2MCDC3006();
	void   flushArduino();
	void   flushMCDC();
	int    queryArduino();            // String pot voltage
	double queryMCDC();               // Motor encoder position
	void   setPosition(const double &); // Motor encoder position
	// --
	bool CONNECTED2ARDUINO = false;
	bool CONNECTED2MCDC = false;

public:
	double currentReading = 0;
	int poseState = 0;
	void updatePlushieState(const double &); 
	// --
	double bentThresh = .6;
	double flatThresh = .3;
	double timeThresh = (.33)*2.;
	double savedTime = 0;

public:
	vector<P3D> POSE_COLORS = { GOLDCLOVER, PUMPKIN, CLAY, ORCHID };

public:
	SimpleSerialPort *ASP;
	const static int ARDUINO_LEN = 4096;
	char arduinoBuffer[ARDUINO_LEN] = "";

public:
	double contraction = 0.;
	QueuePlot *string_plot;
	QueuePlot *tendon_plot;

public:
	void constrict();

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



