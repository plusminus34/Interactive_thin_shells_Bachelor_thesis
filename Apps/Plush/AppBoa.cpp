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
	mainMenu->addButton("connect2MCDC",    [this]() { this->connect2MCDC3006(); });
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
		contraction = sin(t);
		tendon_plot->add_new_data_point(contraction);
	}
} 

bool AppBoa::connect2Arduino() {
	this->ASP = new SimpleSerialPort("\\\\.\\COM3");
	return ASP->IsConnected();
}

bool AppBoa::connect2MCDC3006() {
	return false;
}

void AppBoa::flushArduino() {
	while (ASP->ReadData(arduinoBuffer, ARDUINO_LEN - 1) != 0); 
}

void AppBoa::flushMCDC() {

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
	return 0.0; 
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

