#include "AppXD.h"
#include <PlushHelpers\helpers_star.h>

AppXD::AppXD() {
	setWindowTitle("AppXD");
    // -- 
	mainMenu->addGroup("Group 1");
	mainMenu->addVariable("Variable", TEST);
	mainMenu->addButton("Button", [this](){ std::cout << "TEST = " << TEST << std::endl; }); 
	menuScreen->performLayout();
}

void AppXD::drawScene() {
	PlushApplication::drawScene();
	// draw_floor2d();
}

void AppXD::process() {
    ;
}

bool AppXD::onMouseMoveEvent(double xPos, double yPos) {
    return(PlushApplication::onMouseMoveEvent(xPos, yPos));
}

bool AppXD::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
    return(PlushApplication::onMouseButtonEvent(button, action, mods, xPos, yPos));
}

bool AppXD::onMouseWheelScrollEvent(double xOffset, double yOffset) {
    return(PlushApplication::onMouseWheelScrollEvent(xOffset, yOffset));
}

bool AppXD::onKeyEvent(int key, int action, int mods) {	
	return PlushApplication::onKeyEvent(key, action, mods);
}

bool AppXD::onCharacterPressedEvent(int key, int mods) {
	return PlushApplication::onCharacterPressedEvent(key, mods);
}

bool AppXD::processCommandLine(const std::string& cmdLine) {
	return PlushApplication::processCommandLine(cmdLine);
}

void AppXD::loadFile(const char* fName) { } 
void AppXD::saveFile(const char* fName) { }

