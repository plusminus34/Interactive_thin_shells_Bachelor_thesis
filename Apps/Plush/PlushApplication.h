#pragma once 
// --
#include <PlushHelpers/helpers_star.h>
// --
#include <GUILib/GLApplication.h>
#include <GUILib/GLTrackingCamera.h>
// --
#include "Handler.h"

class PlushApplication : public GLApplication {

public:
    PlushApplication();
	inline virtual ~PlushApplication() { }
 
public:
	int    test_int    = 0 ;
	double test_double = 0.;
	P3D    test_P3D = P3D(0., 0., 0.);

public:
	double t = 0.;

public:
	void incrementTime();
	virtual void processToggles();
	void flashError();
	void drawHandlers();
	// --
	bool DRAW_HANDLERS = true;
	bool DRAW_TEST_POINT = false;

public:
    virtual void process();
	bool STEP = false;
	bool STEPPED = false;

public:
	void recordVideo();
	bool RECORD_VIDEO = false;
	bool RECORDING = false;
	uint FRAME_i = 0;

public:
	virtual void resetCamera(); bool RESET_CAMERA = false;
	        void printCamera(); bool PRINT_CAMERA = false;

public:
	bool SPOOF_2D_CAMERA = false;
 
public:
	double DEFAULT_CAM_ROT_ABOUT_RIGHT_AXIS = 0.;
	double DEFAULT_CAM_ROT_ABOUT_UP_AXIS___ = 0.;
	double DEFAULT_CAM_DISTANCE____________ = -3.;
	P3D    DEFAULT_CAM_TARGET______________ = P3D();

public:
	vector<Handler *> handlers; 

public:
	inline void push_back_handler(Handler *handler) { handlers.push_back(handler); }

public:
    virtual bool onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos);
    virtual bool onMouseMoveEvent(double xPos, double yPos);
    virtual bool onMouseWheelScrollEvent(double xOffset, double yOffset);
    virtual bool onKeyEvent(int key, int action, int mods);
    virtual bool onCharacterPressedEvent(int key, int mods);
public:

public:
	virtual void drawScene();
	inline virtual void drawAuxiliarySceneInfo() { } 
	inline virtual void restart() { } 
	inline virtual bool processCommandLine(const std::string& cmdLine) { return false;  }
 
};



