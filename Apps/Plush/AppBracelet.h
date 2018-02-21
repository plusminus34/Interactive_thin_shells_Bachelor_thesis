#pragma once
#include "spline.h"
#include <stdio.h>
#include <conio.h>
#include <string.h>
#include <Windows.h>
#include <GUILib/GLApplication.h>

#include <FEMSimLib/CSTSimulationMesh2D.h>
#include <Marshmallow/SoftIKSolver2D.h>

#include "main.h"


class AppBracelet : public GLApplication {

// spline interface
public:
	tk::spline spline_L;
	tk::spline spline_R;
	int frame = -1;

// serial interface
public:

	bool flush_buffer(HANDLE &hComm);
	bool FLUSH_ALL_BUFFERS = false;

	bool initiate_connection(HANDLE &hComm, int COM_PORT);
	bool INITIATE_ALL_CONNECTIONS = false;
	bool CONNECTIONS_INITIATED = false;

	bool query_position(HANDLE &hComm);
	bool QUERY_ALL_POSITIONS = false;

	bool set_home(HANDLE &hComm);
	bool SET_ALL_HOMES = false;

	bool goto_home(HANDLE &hComm); 
	bool GOTO_ALL_HOMES = false;

	bool close_connection(HANDLE &hComm);
	bool CLOSE_ALL_CONNECTIONS = false;

	bool set_position(HANDLE &hComm, int position);
	bool send_splines();
	bool SEND_SPLINES = false; 

////////////////////////////////////////////////////////////////////////////////
// marriage ////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// // hardware
public: // ports
	int COM_PORT_L = 21;
	int COM_PORT_R = 20;
	const int MAX_POS_L = 3400000;
	const int MAX_POS_R = 4000000;

// // simulation
public: // tendon indices
	int TENDON_i_L = 1;
	int TENDON_i_R = 2;
public: // actuation bounds in simulation units
	const double alphamax_L = .1;
	const double alphamax_R = .1;

////////////////////////////////////////////////////////////////////////////////
// misc ////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// handles, etc.
public:
	HANDLE hCommL = INVALID_HANDLE_VALUE;
	HANDLE hCommR = INVALID_HANDLE_VALUE;
	DWORD read, written; // FORNOW

// cout
public:
	void CoutLastError();
	void cout_failure(std::string);
	void cout_success(std::string);
	void cout_warning(std::string);
 
public:
    AppBracelet();
    virtual ~AppBracelet(void);
    virtual void process();
    virtual void drawScene();
    virtual void drawAuxiliarySceneInfo();
    virtual void restart();

    virtual bool onKeyEvent(int key, int action, int mods);
    virtual bool onCharacterPressedEvent(int key, int mods);
    virtual bool onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos);
    virtual bool onMouseMoveEvent(double xPos, double yPos);
    virtual bool onMouseWheelScrollEvent(double xOffset, double yOffset);

    virtual bool processCommandLine(const std::string& cmdLine);
};
