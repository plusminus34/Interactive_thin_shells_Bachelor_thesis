#include "AppBoa.h"
#include <PlushHelpers\helpers_star.h>
#include "P2DDragger.h"


AppBoa::AppBoa() {
	setWindowTitle("AppBoa");

	string_plot = new QueuePlot(50, 1);
	tendon_plot = new QueuePlot(50, 1);
	string_plot->SPEC_COLOR = HENN1NK;
	tendon_plot->SPEC_COLOR = ORCHID;
	*string_plot->origin = P3D(-1., 0.);
	*tendon_plot->origin = P3D( 0.5, 0.);
	*string_plot->top_right = P3D(0., 1.);
	*tendon_plot->top_right = P3D( 1.5, 1.);
	handlers.push_back(new P2DDragger(string_plot->origin));
	handlers.push_back(new P2DDragger(tendon_plot->origin));
	handlers.push_back(new P2DDragger(string_plot->top_right));
	handlers.push_back(new P2DDragger(tendon_plot->top_right));

	mainMenu->addGroup("app");
	mainMenu->addButton("connect2Arduino", [this]() { if (!CONNECTED2ARDUINO) { CONNECTED2ARDUINO = this->connect2Arduino(); } });
	mainMenu->addButton("connect2MCDC",    [this]() { if (!CONNECTED2MCDC)    { CONNECTED2MCDC    = this->connect2MCDC3006(); }});
	mainMenu->addButton("flushArduino",    [this]() { this->flushArduino(); });
	mainMenu->addButton("flushMCDC",       [this]() { this->flushMCDC(); });
	mainMenu->addButton("queryArduino",    [this]() { this->queryArduino(); });
	mainMenu->addButton("queryMCDC",       [this]() { this->queryMCDC(); });
	// --
	mainMenu->addButton("constrict",       [this]() { this->constrict(); });
	menuScreen->performLayout(); 
}

void AppBoa::drawScene() {
	PlushApplication::drawScene();
	{
		string_plot->SPEC_COLOR = POSE_COLORS[poseState];
	}
	string_plot->draw();
	tendon_plot->draw();
	// draw_floor2d();
}

void AppBoa::process() {

	if (CONNECTED2ARDUINO) {
		// TODO: Draw thresholds.
		currentReading = queryArduino() / 1000.;
		string_plot->add_new_data_point(currentReading);
		updatePlushieState(currentReading);
	}

	if (CONNECTED2MCDC) {
		currentPosition = queryMCDC() / 10000000.;
		tendon_plot->add_new_data_point(currentPosition);
	}
} 

bool AppBoa::connect2Arduino() {
	this->ASP = new SimpleSerialPort("\\\\.\\COM3");
	return ASP->IsConnected();
}

bool AppBoa::connect2MCDC3006() {
	cout << "Initiating connection on COM" << COM_PORT << "..." << endl; 
	cout << "--- Opening port..." << endl;
	std::wstring comPrefix = L"\\\\.\\COM";
	std::wostringstream tmp;
	tmp << COM_PORT;
	const std::wstring comSuffix(tmp.str()); 
	std::wstring port_spec = comPrefix + comSuffix;
	// https://support.microsoft.com/en-us/help/115831/howto-specify-serial-ports-larger-than-com9
	hComm = CreateFileW(
		port_spec.c_str(),
		GENERIC_READ | GENERIC_WRITE,
		0,
		0,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		0);
	if (hComm == INVALID_HANDLE_VALUE) {
		return false;
	}

	cout << "--- Applying settings..." << endl;
	std::wstring dcb_spec = L"9600, n, 8, 1";
	DCB dcb;
	FillMemory(&dcb, sizeof(dcb), 0);
	dcb.DCBlength = sizeof(dcb);
	if (!BuildCommDCBW(dcb_spec.c_str(), &dcb)) {
		return false;
	}
	if (!SetCommState(hComm, &dcb)) {
		return false;
	}

	cout << "--- Specifying timeouts..." << endl;
    COMMTIMEOUTS timeouts;
	timeouts.ReadIntervalTimeout = 20; 
	timeouts.ReadTotalTimeoutMultiplier = 0;
	timeouts.ReadTotalTimeoutConstant = 100;
	timeouts.WriteTotalTimeoutMultiplier = 10;
	timeouts.WriteTotalTimeoutConstant = 100;
	if (!SetCommTimeouts(hComm, &timeouts)) {
		return false;
	}

	cout_success("Successfully initiated connection.\n");
	return true; 
}

void AppBoa::flushArduino() {
	while (ASP->ReadData(arduinoBuffer, ARDUINO_LEN - 1) != 0); 
}

bool AppBoa::flushMCDC() { 
	cout << "Flushing buffer..." << endl;

	char read_buffer[1024];
	DWORD bytesRead = 0;
	if (!ReadFile(hComm, read_buffer, sizeof(read_buffer), &bytesRead, NULL)) {
		CoutLastError();
		return false;
	}

	for (int i = 0; i < (int)bytesRead; ++i) {
		cout << read_buffer[i];
	}

	cout_success("Successfully flushed buffer.\n");
	return true; 
}

int AppBoa::queryArduino() {
	flushArduino();
	while (ASP->ReadData(arduinoBuffer, ARDUINO_LEN - 1, 32) == 0);
	char *start = arduinoBuffer;
	while (*start++ != '\n');
	int read = strtol(start, nullptr, 10);
	return read;
}

double AppBoa::queryMCDC() {
	cout << "Querying position." << endl;

	char write_buffer[] = "POS\r\n";
	if (!WriteFile(hComm, write_buffer, sizeof(write_buffer), &written, NULL)) {
		CoutLastError();
		return false; 
	}
	char read_buffer[128];
	DWORD bytesRead = 0;
	if (!ReadFile(hComm, read_buffer, sizeof(read_buffer), &bytesRead, NULL)) {
		CoutLastError();
		return false;
	}

	read_buffer[(int)bytesRead - 2] = 0; // Killing off carriage return
	cout << "Position: " << read_buffer << endl;

	cout_success("Successfully queried position.\n");

	int ret;
	sscanf(read_buffer, "%d", &ret); 
	return double(ret);
}
 
void AppBoa::updatePlushieState(const double &reading) {
	int oldPoseState = -1;
	while (poseState != oldPoseState) {
		oldPoseState = poseState;

		// Initial state, arm is flat on table.
		if (poseState == 0) {
			if (reading > bentThresh) {
				poseState = 1;
				// --
				savedTime = t;
			}
			// Arm was bent, looking for second part of gesture.
		}
		else if (poseState == 1) {
			if (t - savedTime >= timeThresh) {
				poseState = 2;
			}
			else if (reading < flatThresh) {
				poseState = 3;
				// --
				savedTime = t;
			}
			// Ran out of time...
		}
		else if (poseState == 2) {
			if (reading < flatThresh) {
				poseState = 0;
			}
			// constrict()
		}
		else if (poseState == 3) {
			// TODO: constrict() once
			if (t - savedTime >= timeThresh) {
				poseState = 2;
			}
		}
	}
}

void AppBoa::constrict() {

}

void AppBoa::setPosition(const double &) {

}

bool AppBoa::onMouseMoveEvent(double xPos, double yPos) {
    return(PlushApplication::onMouseMoveEvent(xPos, yPos));
}

bool AppBoa::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
    return(PlushApplication::onMouseButtonEvent(button, action, mods, xPos, yPos));
}

bool AppBoa::onMouseWheelScrollEvent(double xOffset, double yOffset) {
    return(PlushApplication::onMouseWheelScrollEvent(xOffset, yOffset));
}

bool AppBoa::onKeyEvent(int key, int action, int mods) {	
	return PlushApplication::onKeyEvent(key, action, mods);
}

bool AppBoa::onCharacterPressedEvent(int key, int mods) {
	return PlushApplication::onCharacterPressedEvent(key, mods);
}

bool AppBoa::processCommandLine(const std::string& cmdLine) {
	return PlushApplication::processCommandLine(cmdLine);
}

void AppBoa::loadFile(const char* fName) { } 
void AppBoa::saveFile(const char* fName) { }

