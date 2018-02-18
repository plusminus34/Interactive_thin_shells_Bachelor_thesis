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
	draw_floor2d();
}

void AppXD::process() {
    ;
}

bool AppXD::onMouseMoveEvent(double xPos, double yPos) {
    return(GLApplication::onMouseMoveEvent(xPos, yPos));
}

bool AppXD::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
    return(GLApplication::onMouseButtonEvent(button, action, mods, xPos, yPos));
}

bool AppXD::onMouseWheelScrollEvent(double xOffset, double yOffset) {
    return(GLApplication::onMouseWheelScrollEvent(xOffset, yOffset));
}

bool AppXD::onKeyEvent(int key, int action, int mods) {	
	return GLApplication::onKeyEvent(key, action, mods);
}

bool AppXD::onCharacterPressedEvent(int key, int mods) {
	return GLApplication::onCharacterPressedEvent(key, mods);
}

bool AppXD::processCommandLine(const std::string& cmdLine) {
	return GLApplication::processCommandLine(cmdLine);
	return false;
}

void AppXD::loadFile(const char* fName) { } 
void AppXD::saveFile(const char* fName) { }

