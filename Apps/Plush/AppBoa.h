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
	bool   flushMCDC();
	int    queryArduino();            // String pot voltage
	double queryMCDC();               // Motor encoder position
	void   setPosition(const double &); // Motor encoder position
	// --
	bool CONNECTED2ARDUINO = false;
	bool CONNECTED2MCDC = false;

// FORNOW MCDC
public:
	int COM_PORT = 4;
	HANDLE hComm = INVALID_HANDLE_VALUE;
	DWORD read, written; // FORNOW
 
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
	double currentPosition = 0;
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

//Returns the last Win32 error, in string format. Returns an empty string if there is no error.
std::string GetLastErrorAsString() {
    //Get the error message, if any.
    DWORD errorMessageID = ::GetLastError();
    if(errorMessageID == 0)
        return std::string(); //No error message has been recorded

    LPSTR messageBuffer = nullptr;
    size_t size = FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
                                 NULL, errorMessageID, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&messageBuffer, 0, NULL);

    std::string message(messageBuffer, size);

    //Free the buffer.
    LocalFree(messageBuffer);

    return message;
} 

void CoutLastError() { 
	cout_warning(GetLastErrorAsString());
}

void cout_failure(std::string msg) {
	HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
	SetConsoleTextAttribute(hConsole, 12);
	cout << msg;
	SetConsoleTextAttribute(hConsole, 15);
}

void cout_success(std::string msg) {
	HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
	SetConsoleTextAttribute(hConsole, 10);
	cout << msg;
	SetConsoleTextAttribute(hConsole, 15);
}

void cout_warning(std::string msg) {
	HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
	SetConsoleTextAttribute(hConsole, 14);
	cout << msg;
	SetConsoleTextAttribute(hConsole, 15);
} 


};



